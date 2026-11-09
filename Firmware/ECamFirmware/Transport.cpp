/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#include "Transport.h"

#include "CamEngine.h"
#include "ClearCore.h"
#include "Console.h"
#include "ECamConfig.h"
#include "EthernetTcpClient.h"
#include "EthernetTcpServer.h"
#include "EthernetUdp.h"
#include "Settings.h"
#include "lwip/etharp.h"
#include "lwip/memp.h"
#include "lwip/stats.h"

#include <stdio.h>
#include <string.h>

#define UsbPort ConnectorUsb
/* libClearCore's USB send buffer (UsbManager.h USB_SERIAL_BUFFER_SIZE, internal header). */
static const int32_t kUsbSendBuffer = 64;

/* USB console */
static char g_usbLine[ECAM_MAX_LINE];
static uint16_t g_usbLen = 0;
static bool g_usbOverflow = false;
static uint32_t g_usbTelPeriod = 0, g_usbTelLast = 0;

/* Ethernet */
static EthernetTcpServer g_consoleServer(ECAM_TCP_CONSOLE_PORT);
static EthernetTcpClient g_consoleClient;
static EthernetUdp g_discoveryUdp;
static bool g_ethernetReady = false;
static bool g_dhcpFellBack = false;  /* DHCP mode, no server: on the stored static address */
/* Gratuitous ARP after the address is set: EthernetMgr writes the address straight into the interface, so
 * lwIP never announces it. Until the board sends something, a PC or PLC that tried the address while it
 * was restarting (Windows: neighbour entry stale or unreachable, backing off) or a switch that dropped
 * the port when the link went down may not reach it for a minute or more. Announced at 0, 1 and 3 s. */
static uint32_t g_ethernetReadyMs = 0;
static uint8_t g_announced = 0;
static bool g_consoleConnected = false;
static char g_tcpLine[ECAM_MAX_LINE];
static uint16_t g_tcpLen = 0;
static bool g_tcpOverflow = false;
static uint32_t g_tcpTelPeriod = 0, g_tcpTelLast = 0;
static uint32_t g_tcpSessions = 0;
static uint32_t g_telSkipped = 0;

/* ------------------------------------------------------------------------- */
/* USB                                                                        */
/* ------------------------------------------------------------------------- */

void TransportUsbSendLine(const char *line) {
    UsbPort.SendLine(line);
}

static void UsbOut(const char *line) {
    UsbPort.SendLine(line);
}

void TransportInitUsb() {
    UsbPort.Mode(Connector::USB_CDC);
    UsbPort.Speed(ECAM_USB_BAUD);
    UsbPort.PortOpen();
    const uint32_t start = Milliseconds();
    while (!UsbPort && Milliseconds() - start < ECAM_WAIT_USB_MS) {
        continue;
    }
    Delay_ms(100);
}

/* Returns true when a command was run (or refused), so the caller can give the main loop back: one command
 * per pass keeps a host that sends without pause from starving the EtherNet/IP adapter and discovery. */
static bool LineChar(char c, char *line, uint16_t *len, bool *overflow, ConsoleOut out, bool *closed) {
    if (c == '\r' || c == '\n') {
        bool ran = false;
        if (*overflow) {
            out("error: line too long");
            ran = true;
        } else if (*len > 0) {
            line[*len] = '\0';
            if (ConsoleHandleLine(line, out) && closed) {
                *closed = true;
            }
            ran = true;
        }
        *len = 0;
        *overflow = false;
        return ran;
    }
    if (*len >= ECAM_MAX_LINE - 1) {
        *overflow = true;
        return false;
    }
    line[(*len)++] = c;
    return false;
}

static void PollUsb() {
    for (int budget = 512; budget > 0; budget--) {
        const int16_t ch = UsbPort.CharGet();
        if (ch < 0) {
            break;
        }
        if (LineChar((char)ch, g_usbLine, &g_usbLen, &g_usbOverflow, UsbOut, nullptr)) {
            break;  /* one command per pass */
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Ethernet                                                                   */
/* ------------------------------------------------------------------------- */

void TransportInitEthernet() {
    if (g_ethernetReady || !EthernetMgr.PhyLinkActive()) {
        return;
    }
    EthernetMgr.Setup();
    if (g_settings.netMode == 1) {
        const uint8_t *ip = g_settings.ip;
        const uint8_t *nm = g_settings.netmask;
        const uint8_t *gw = g_settings.gateway;
        EthernetMgr.LocalIp(IpAddress(ip[0], ip[1], ip[2], ip[3]));
        EthernetMgr.NetmaskIp(IpAddress(nm[0], nm[1], nm[2], nm[3]));
        EthernetMgr.GatewayIp(IpAddress(gw[0], gw[1], gw[2], gw[3]));
    } else if (!EthernetMgr.DhcpBegin()) {
        /* No DHCP server: the stored static address (as set from the console or EtherNet/IP), so a
         * board put on DHCP stays reachable where it was. */
        const uint8_t *ip = g_settings.ip;
        const uint8_t *nm = g_settings.netmask;
        const uint8_t *gw = g_settings.gateway;
        EthernetMgr.LocalIp(IpAddress(ip[0], ip[1], ip[2], ip[3]));
        EthernetMgr.NetmaskIp(IpAddress(nm[0], nm[1], nm[2], nm[3]));
        EthernetMgr.GatewayIp(IpAddress(gw[0], gw[1], gw[2], gw[3]));
        g_dhcpFellBack = true;
    }
    g_consoleServer.Begin();
    g_discoveryUdp.Begin(ECAM_UDP_DISCOVERY_PORT);
    g_ethernetReady = true;
    g_ethernetReadyMs = Milliseconds();
    g_announced = 0;
    UsbPort.Send("ECam IP=");
    UsbPort.SendLine(EthernetMgr.LocalIp().StringValue());
}

bool TransportEthernetReady() {
    return g_ethernetReady;
}

bool TransportDhcpFellBack() {
    return g_dhcpFellBack;
}

const char *TransportIpString() {
    return g_ethernetReady ? EthernetMgr.LocalIp().StringValue() : "none";
}

static void TcpOut(const char *line) {
    if (!g_consoleConnected) {
        return;
    }
    /* One Send per line: Send can wait up to 5 ms when the send queue is half full, once per call. A TEL line
     * is the longest there is, and anything longer still goes out, in two. */
    char buf[258];
    const size_t n = strlen(line);
    if (n + 2 <= sizeof(buf)) {
        memcpy(buf, line, n);
        buf[n] = '\r';
        buf[n + 1] = '\n';
        g_consoleClient.Send(reinterpret_cast<const uint8_t *>(buf), static_cast<uint32_t>(n + 2));
    } else {
        g_consoleClient.Send(line);
        g_consoleClient.Send("\r\n");
    }
}

static void PollTcpConsole() {
    if (!g_consoleConnected || !g_consoleClient.Connected()) {
        if (g_consoleConnected) {
            g_consoleClient.Close();
            g_consoleConnected = false;
            g_tcpTelPeriod = 0;
        }
        EthernetTcpClient next = g_consoleServer.Accept();
        if (!next.Connected()) {
            return;
        }
        g_consoleClient = next;
        g_consoleConnected = true;
        g_tcpLen = 0;
        g_tcpOverflow = false;
        g_tcpSessions++;
        ConsoleBanner(TcpOut);
    }
    bool closed = false;
    /* One command per pass, and at most one line's worth of bytes: Send() services the stack, so a client
     * that keeps sending would otherwise keep this loop going and starve everything else in the main loop. */
    for (int budget = ECAM_MAX_LINE; !closed && budget > 0 && g_consoleClient.BytesAvailable() > 0; budget--) {
        const int16_t ch = g_consoleClient.Read();
        if (ch < 0) {
            break;
        }
        if (LineChar((char)ch, g_tcpLine, &g_tcpLen, &g_tcpOverflow, TcpOut, &closed)) {
            break;
        }
    }
    if (closed) {
        g_consoleClient.Close();
        g_consoleConnected = false;
        g_tcpTelPeriod = 0;
    }
}

static void PollDiscovery() {
    const uint16_t size = g_discoveryUdp.PacketParse();
    if (size == 0) {
        return;
    }
    unsigned char packet[64];
    const int32_t n = g_discoveryUdp.PacketRead(packet, sizeof(packet) - 1);
    if (n <= 0) {
        return;
    }
    packet[n] = '\0';
    if (strncmp((const char *)packet, ECAM_DISCOVERY_REQUEST, strlen(ECAM_DISCOVERY_REQUEST)) != 0) {
        return;
    }
    char reply[128];
    snprintf(reply, sizeof(reply), "ECAM %s IP=%s CONSOLE=%u FW=%s", ECAM_FIRMWARE_NAME,
             EthernetMgr.LocalIp().StringValue(), (unsigned)ECAM_TCP_CONSOLE_PORT, ECAM_FIRMWARE_VERSION);
    g_discoveryUdp.Connect(g_discoveryUdp.RemoteIp(), g_discoveryUdp.RemotePort());
    g_discoveryUdp.PacketWrite(reply);
    g_discoveryUdp.PacketSend();
}

/* ------------------------------------------------------------------------- */
/* Telemetry                                                                  */
/* ------------------------------------------------------------------------- */

const char *TransportSetTelemetry(void (*out)(const char *), uint32_t periodMs) {
    const uint32_t p = (periodMs == 0) ? 0 : (periodMs < 20 ? 20 : periodMs);
    if (out == TcpOut) {
        g_tcpTelPeriod = p;
        g_tcpTelLast = 0;
    } else if (out == UsbOut) {
        g_usbTelPeriod = p;
        g_usbTelLast = 0;
    } else {
        return "telemetry is not available on this link";
    }
    return nullptr;
}

static void PollTelemetry() {
    const uint32_t now = Milliseconds();
    char line[256];  /* a TEL line with 4 axes and the encoder count */
    bool built = false;
    if (g_tcpTelPeriod && g_consoleConnected && now - g_tcpTelLast >= g_tcpTelPeriod) {
        volatile const EthernetTcp::TcpData *st = g_consoleClient.ConnectionState();
        /* Send waits up to 5 ms once the queue reaches half of TCP_SND_QUEUELEN. A line that would take it there
         * is skipped, so a host that is late acknowledging costs telemetry lines, not main loop time. */
        if (st != nullptr && st->pcb != nullptr && tcp_sndbuf(st->pcb) >= 300 &&
            tcp_sndqueuelen(st->pcb) + 1 < (TCP_SND_QUEUELEN >> 1)) {
            EngineTelemetry(line, sizeof(line));
            built = true;
            TcpOut(line);
            g_tcpTelLast = now;
        } else {
            g_telSkipped++;
            g_tcpTelLast = now;  /* one count per line due, not one per pass */
        }
    }
    if (g_usbTelPeriod && now - g_usbTelLast >= g_usbTelPeriod) {
        if (!built) {
            EngineTelemetry(line, sizeof(line));
        }
        /* The USB send buffer is only 64 bytes, smaller than a TEL line, and SendChar waits for room with no
         * timeout. Starting only when it is empty skips lines for a host that has fallen behind, but a host
         * that stops reading partway through a line still holds up this loop until it reads or closes the port. */
        if (UsbPort.AvailableForWrite() >= kUsbSendBuffer - 1) {
            UsbOut(line);
            g_usbTelLast = now;
        } else {
            g_telSkipped++;
            g_usbTelLast = now;  /* one count per line due, not one per pass */
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Public                                                                     */
/* ------------------------------------------------------------------------- */

static void Announce() {
    static const uint32_t kAtMs[] = {0, 1000, 3000};
    if (g_announced >= sizeof(kAtMs) / sizeof(kAtMs[0]) || Milliseconds() - g_ethernetReadyMs < kAtMs[g_announced]) {
        return;
    }
    etharp_gratuitous(EthernetMgr.MacInterface());
    g_announced++;
}

void TransportPoll() {
    PollUsb();
    if (!g_ethernetReady) {
        TransportInitEthernet();
    } else {
        EthernetMgr.Refresh();
        Announce();
        PollTcpConsole();
        PollDiscovery();
    }
    PollTelemetry();
}

void TransportStats(char *buf, size_t len) {
    snprintf(buf, len, "links: ip=%s tcp_client=%d tcp_sessions=%lu tel_skipped=%lu", TransportIpString(),
             g_consoleConnected ? 1 : 0, (unsigned long)g_tcpSessions, (unsigned long)g_telSkipped);
}

void TransportLwipStats(char *buf, size_t len) {
#if LWIP_STATS && MEM_STATS && MEMP_STATS
    unsigned long poolErr = 0;
    for (int i = 0; i < MEMP_MAX; ++i) {
        if (lwip_stats.memp[i]) {
            poolErr += lwip_stats.memp[i]->err;
        }
    }
    const stats_mem *pb = lwip_stats.memp[MEMP_PBUF_POOL];
    const stats_mem *seg = lwip_stats.memp[MEMP_TCP_SEG];
    const stats_mem *udp = lwip_stats.memp[MEMP_UDP_PCB];
    const stats_mem *tcp = lwip_stats.memp[MEMP_TCP_PCB];
    snprintf(buf, len,
             "lwip: heap %lu/%lu used (max %lu, errors %lu); pool errors %lu; pbuf pool max %lu/%lu; tcp seg max "
             "%lu/%lu; tcp pcb max %lu/%lu; udp pcb max %lu/%lu",
             (unsigned long)lwip_stats.mem.used, (unsigned long)lwip_stats.mem.avail, (unsigned long)lwip_stats.mem.max,
             (unsigned long)lwip_stats.mem.err, poolErr, (unsigned long)(pb ? pb->max : 0), (unsigned long)(pb ? pb->avail : 0),
             (unsigned long)(seg ? seg->max : 0), (unsigned long)(seg ? seg->avail : 0), (unsigned long)(tcp ? tcp->max : 0),
             (unsigned long)(tcp ? tcp->avail : 0), (unsigned long)(udp ? udp->max : 0), (unsigned long)(udp ? udp->avail : 0));
#else
    snprintf(buf, len, "lwip: statistics not compiled in");
#endif
}

static uint32_t g_loopLast = 0;
static uint32_t g_loopMaxCycles = 0;
static uint32_t g_loopPasses = 0;

void TransportLoopTick() {
    const uint32_t now = DWT->CYCCNT;
    if (g_loopLast != 0) {
        const uint32_t d = now - g_loopLast;
        if (d > g_loopMaxCycles) {
            g_loopMaxCycles = d;
        }
    }
    g_loopLast = now;
    g_loopPasses++;
}

void TransportLoopStats(char *buf, size_t len) {
    snprintf(buf, len, "main loop: %lu passes, longest %lu us (since the last diag)", (unsigned long)g_loopPasses,
             (unsigned long)(g_loopMaxCycles / (SystemCoreClock / 1000000u)));
    g_loopMaxCycles = 0;
    g_loopPasses = 0;
    g_loopLast = 0;
}

void TransportRestart() {
    for (int i = 0; i < 10; i++) {
        if (g_ethernetReady) {
            EthernetMgr.Refresh();
        }
        Delay_ms(5);
    }
    SysMgr.ResetBoard();
}
