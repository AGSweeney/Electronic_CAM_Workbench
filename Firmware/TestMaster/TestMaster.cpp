/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
/*
 * TestMaster: a second ClearCore that loads an ECam board the way loadtest does, so no PC is in the timing.
 *
 *   TCP 9500 to the board   telemetry at 50 ms, read and timed here; board replies are copied to USB
 *   UDP 9502 to the board   a discovery ping every 0.5 s
 *   Class 1 to the board    optional, QuickEIP's originator at 20 ms. Leave it off while a PLC owns the connection.
 *
 * Every stream records its longest gap and how often it passed 30 and 80 ms, on this board's clock. Results go out
 * over USB only, so reporting never touches the network under test. Commands, on the USB console:
 *
 *   start [class1]   open the streams        stop     close them
 *   report           print the results       reset    zero them
 *   say TEXT         send TEXT to the board's console, for example "say diag"
 *   flags N          ControlFlags in the Class 1 output
 *
 * This firmware has no profile, no motors and no settings of its own: the addresses are the constants below.
 */

#include "ClearCore.h"
#include "EthernetTcpClient.h"
#include "EthernetUdp.h"
#include "eip.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UsbPort ConnectorUsb

static const IpAddress kOwnIp(10, 168, 41, 7);
static const IpAddress kNetmask(255, 255, 255, 0);
static const IpAddress kBoardIp(10, 168, 41, 5);
static const char *const kBoardIpText = "10.168.41.5";
static const uint16_t kConsolePort = 9500;
static const uint16_t kDiscoveryPort = 9502;
static const uint16_t kAsmCmd = 150, kAsmSts = 100, kAsmCfg = 151, kCmdSize = 64, kStsSize = 160;
static const uint32_t kClass1PeriodUs = 20000;
static const uint32_t kPingPeriodMs = 500;

/* Gaps between events of one stream, in microseconds. */
struct Gaps {
    uint32_t count, last, longest, over30, over80;
    void Hit() {
        const uint32_t now = Microseconds();
        if (count > 0) {
            const uint32_t gap = now - last;
            if (gap > longest) longest = gap;
            if (gap > 30000) over30++;
            if (gap > 80000) over80++;
        }
        last = now;
        count++;
    }
};

static Gaps g_tel, g_class1;
static uint32_t g_pings, g_pongs, g_startMs, g_passes, g_longestPass, g_reconnects;
static bool g_running, g_class1On;

static EthernetTcpClient g_console;
static EthernetUdp g_udp;
static char g_rxLine[320];
static uint16_t g_rxLen;
static uint32_t g_pingLast;

static EipSession g_session;
static EipIo g_io;
static uint8_t g_cmd[kCmdSize];
static uint8_t g_sts[kStsSize];
static uint32_t g_class1Next;

static void Say(const char *text) {
    UsbPort.SendLine(text);
}

static void Reset() {
    memset(&g_tel, 0, sizeof(g_tel));
    memset(&g_class1, 0, sizeof(g_class1));
    g_pings = g_pongs = g_passes = g_longestPass = g_reconnects = 0;
    g_startMs = Milliseconds();
}

static void Report() {
    char line[160];
    snprintf(line, sizeof(line), "report t=%lu s  main pass longest %lu us  console reconnects %lu",
             (unsigned long)((Milliseconds() - g_startMs) / 1000), (unsigned long)g_longestPass, (unsigned long)g_reconnects);
    Say(line);
    snprintf(line, sizeof(line), "tel n=%lu longest %lu us over30 %lu over80 %lu", (unsigned long)g_tel.count,
             (unsigned long)g_tel.longest, (unsigned long)g_tel.over30, (unsigned long)g_tel.over80);
    Say(line);
    snprintf(line, sizeof(line), "class1 n=%lu longest %lu us over30 %lu over80 %lu", (unsigned long)g_class1.count,
             (unsigned long)g_class1.longest, (unsigned long)g_class1.over30, (unsigned long)g_class1.over80);
    Say(line);
    snprintf(line, sizeof(line), "pings %lu/%lu", (unsigned long)g_pongs, (unsigned long)g_pings);
    Say(line);
}

static void SendToBoard(const char *text) {
    if (g_console.Connected()) {
        g_console.Send(text);
        g_console.Send("\n");
    }
}

static bool OpenClass1() {
    EipOpenConfig open;
    memset(&open, 0, sizeof(open));
    open.ip = kBoardIpText;
    open.slot = -1;
    open.vendor_id = 0x1234;
    open.serial_number = 0x7E57;
    open.timeout_ms = 2000;
    int rc = eip_session_open(&g_session, &open);
    if (rc != EIP_OK) {
        Say(eip_status_text(rc));
        return false;
    }
    EipIoConfig io;
    memset(&io, 0, sizeof(io));
    io.config_instance = kAsmCfg;
    io.output_instance = kAsmCmd;
    io.input_instance = kAsmSts;
    io.output_bytes = kCmdSize;
    io.input_bytes = kStsSize;
    io.o2t_rpi_us = io.t2o_rpi_us = kClass1PeriodUs;
    io.run_idle = 1;
    io.timeout_multiplier = 2;  /* 10 ms x 16 */
    rc = eip_io_open(&g_session, &g_io, &io);
    if (rc != EIP_OK) {
        Say(eip_status_text(rc));
        eip_session_close(&g_session);
        return false;
    }
    g_class1Next = Microseconds();
    return true;
}

static void Start(bool class1) {
    if (g_running) {
        Say("error: already running");
        return;
    }
    Reset();
    if (!g_console.Connect(kBoardIp, kConsolePort)) {
        Say("error: no console connection");
        return;
    }
    g_rxLen = 0;
    SendToBoard("tel 50");
    g_udp.Begin(kDiscoveryPort + 1);
    g_pingLast = Milliseconds();
    g_class1On = class1 && OpenClass1();
    g_running = true;
    Say(class1 && !g_class1On ? "running, no Class 1" : "running");
}

static void Stop() {
    if (!g_running) {
        return;
    }
    if (g_class1On) {
        eip_io_close(&g_session, &g_io);
        eip_session_close(&g_session);
        g_class1On = false;
    }
    SendToBoard("tel 0");
    g_console.Close();
    g_running = false;
    Say("stopped");
    Report();
}

static void PollConsole() {
    if (!g_console.Connected()) {
        g_reconnects++;
        if (g_console.Connect(kBoardIp, kConsolePort)) {
            SendToBoard("tel 50");
        }
        return;
    }
    for (int budget = 1024; budget > 0 && g_console.BytesAvailable() > 0; budget--) {
        const int16_t ch = g_console.Read();
        if (ch < 0) {
            break;
        }
        if (ch == '\r') {
            continue;
        }
        if (ch != '\n') {
            if (g_rxLen < sizeof(g_rxLine) - 1) {
                g_rxLine[g_rxLen++] = (char)ch;
            }
            continue;
        }
        g_rxLine[g_rxLen] = '\0';
        if (strncmp(g_rxLine, "TEL ", 4) == 0) {
            g_tel.Hit();
        } else if (g_rxLen > 0) {
            Say(g_rxLine);
        }
        g_rxLen = 0;
    }
}

static void PollPing() {
    if (g_udp.PacketParse() > 0) {
        uint8_t reply[128];
        g_udp.PacketRead(reply, sizeof(reply));
        g_pongs++;
    }
    if (Milliseconds() - g_pingLast >= kPingPeriodMs) {
        g_pingLast += kPingPeriodMs;
        g_pings++;
        g_udp.Connect(kBoardIp, kDiscoveryPort);
        g_udp.PacketWrite("ECAM_DISCOVER?");
        g_udp.PacketSend();
    }
}

static void PollClass1() {
    if ((int32_t)(Microseconds() - g_class1Next) >= 0) {
        g_class1Next += kClass1PeriodUs;
        eip_io_produce(&g_io, g_cmd, kCmdSize, 1);
    }
    uint16_t len = kStsSize;
    if (eip_io_consume(&g_io, g_sts, &len, 1) == EIP_OK) {  /* no non-blocking form: 1 ms is the shortest */
        g_class1.Hit();
    }
}

static void Command(char *line) {
    if (strcmp(line, "start") == 0) {
        Start(false);
    } else if (strcmp(line, "start class1") == 0) {
        Start(true);
    } else if (strcmp(line, "stop") == 0) {
        Stop();
    } else if (strcmp(line, "report") == 0) {
        Report();
    } else if (strcmp(line, "reset") == 0) {
        Reset();
        Say("ok");
    } else if (strncmp(line, "say ", 4) == 0) {
        SendToBoard(line + 4);
    } else if (strncmp(line, "flags ", 6) == 0) {
        const uint16_t flags = (uint16_t)strtoul(line + 6, nullptr, 0);
        g_cmd[0] = (uint8_t)flags;
        g_cmd[1] = (uint8_t)(flags >> 8);
        Say("ok");
    } else if (line[0] != '\0') {
        Say("start [class1] | stop | report | reset | say TEXT | flags N");
    }
}

static void PollUsb() {
    static char line[96];
    static uint16_t len = 0;
    for (int budget = 96; budget > 0; budget--) {
        const int16_t ch = UsbPort.CharGet();
        if (ch < 0) {
            return;
        }
        if (ch == '\r' || ch == '\n') {
            line[len] = '\0';
            len = 0;
            Command(line);
            return;
        }
        if (len < sizeof(line) - 1) {
            line[len++] = (char)ch;
        }
    }
}

int main(void) {
    Delay_ms(300);
    UsbPort.Mode(Connector::USB_CDC);
    UsbPort.Speed(115200);
    UsbPort.PortOpen();
    while (!EthernetMgr.PhyLinkActive()) {
        Delay_ms(100);
    }
    EthernetMgr.Setup();
    EthernetMgr.LocalIp(kOwnIp);
    EthernetMgr.NetmaskIp(kNetmask);
    Say("TestMaster ready, 10.168.41.7");
    uint32_t last = Microseconds();
    while (true) {
        EthernetMgr.Refresh();
        PollUsb();
        if (g_running) {
            PollConsole();
            PollPing();
            if (g_class1On) {
                PollClass1();
            }
            const uint32_t now = Microseconds();
            if (now - last > g_longestPass) {
                g_longestPass = now - last;
            }
            g_passes++;
        }
        last = Microseconds();
    }
}
