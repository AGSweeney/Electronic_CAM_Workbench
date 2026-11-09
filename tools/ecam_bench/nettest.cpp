// MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
// Network settings over CIP: moves the board to ALT and back, puts it on DHCP with no server and back to static, resets
// it, and checks the refusals. Every step ends with the board at IP; its USB console is the way back if it doesn't.
//
//   ecam_bench nettest [IP] [ALT] [--usb COM3]     only on the isolated 10.168.41.x network, with no DHCP server
//   ecam_bench netcheck ADAPTER_IP                 the same requests, only to `ecam_bench adapter` on this PC
//   ecam_bench usbcheck COM3                       the USB way back, read-only
//
// Before nettest: the board static at IP with a profile loaded, and no PLC holding its Class 1 connection.
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "bench_common.h"
#include "eip.h"
#include "eip_hal.h"

namespace {

constexpr const char *kBenchPrefix = "10.168.41.";
constexpr uint8_t kNoReply = 0xFF;  // no CIP status came back
constexpr uint8_t kGetSet = 0x10;   // Set_Attribute_Single, and the "device state conflict" status has the same value
constexpr uint8_t kStateConflict = 0x10, kMultipleFailed = 0x1E;
constexpr uint16_t kTcpIp = 0xF5, kAssemblySettable = 0x10;

EipSession g_session;

class Session {
public:
    explicit Session(const std::string &ip) {
        EipOpenConfig open;
        memset(&open, 0, sizeof(open));
        open.ip = ip.c_str();
        open.slot = -1;
        open.vendor_id = 0x1234;
        open.serial_number = 0x7E59;
        open.timeout_ms = 3000;
        const int rc = eip_session_open(&g_session, &open);
        if (rc != EIP_OK) {
            throw std::runtime_error("session to " + ip + ": " + eip_status_text(rc));
        }
    }
    ~Session() { eip_session_close(&g_session); }
    Session(const Session &) = delete;
    Session &operator=(const Session &) = delete;
};

std::string Hex(uint8_t v) {
    char t[8];
    snprintf(t, sizeof(t), "0x%02x", v);
    return t;
}

// ---- TCP/IP object ---------------------------------------------------------------------------------------------------

struct Net {
    uint32_t capability = 0, control = 0;
    bool settable = false;
    std::string ip, netmask, gateway, host;
};

std::vector<uint8_t> GetAttr(uint16_t attr) {
    uint8_t data[256];
    uint16_t len = sizeof(data);
    uint8_t status = 0;
    const int rc = eip_get_attribute(&g_session, kTcpIp, 1, attr, data, &len, &status);
    if (rc != EIP_OK) {
        throw std::runtime_error("get F5 attr " + std::to_string(attr) + ": " + eip_status_text(rc) + " " + Hex(status));
    }
    return std::vector<uint8_t>(data, data + len);
}

/** CIP carries an address as a UDINT, so its bytes arrive reversed. */
std::string IpAt(const std::vector<uint8_t> &d, size_t at) {
    char t[20];
    snprintf(t, sizeof(t), "%u.%u.%u.%u", d[at + 3], d[at + 2], d[at + 1], d[at]);
    return t;
}

Net ReadNet() {
    Net n;
    const auto a2 = GetAttr(2), a3 = GetAttr(3), a5 = GetAttr(5), a6 = GetAttr(6);
    if (a2.size() < 4 || a3.size() < 4 || a5.size() < 22 || a6.size() < 2) {
        throw std::runtime_error("short TCP/IP attribute");
    }
    n.capability = eip_load_le32(a2.data());
    n.settable = (n.capability & kAssemblySettable) != 0;
    n.control = eip_load_le32(a3.data()) & 0x0F;
    n.ip = IpAt(a5, 0);
    n.netmask = IpAt(a5, 4);
    n.gateway = IpAt(a5, 8);
    const uint16_t hlen = eip_load_le16(a6.data());
    n.host.assign(reinterpret_cast<const char *>(a6.data()) + 2, a6.size() - 2 < hlen ? a6.size() - 2 : hlen);
    return n;
}

Net PrintNet(const std::string &ip) {
    Session s(ip);
    const Net n = ReadNet();
    printf("%s: %s (in use) ip=%s mask=%s gw=%s host='%s' settable=%d\n", ip.c_str(), n.control == 2 ? "dhcp" : "static",
           n.ip.c_str(), n.netmask.c_str(), n.gateway.c_str(), n.host.c_str(), n.settable ? 1 : 0);
    return n;
}

void PutIp(std::vector<uint8_t> &out, const std::string &text) {
    unsigned a = 0, b = 0, c = 0, d = 0;
    sscanf(text.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d);
    for (unsigned v : {d, c, b, a}) {
        out.push_back(static_cast<uint8_t>(v));
    }
}

/** CIP STRING: UINT length and characters, padded to even. */
std::vector<uint8_t> PadString(const std::string &s) {
    std::vector<uint8_t> out = {static_cast<uint8_t>(s.size()), static_cast<uint8_t>(s.size() >> 8)};
    out.insert(out.end(), s.begin(), s.end());
    if (s.size() & 1) {
        out.push_back(0);
    }
    return out;
}

uint8_t SetAttr(uint16_t attr, const std::vector<uint8_t> &data) {
    uint8_t status = kNoReply;
    eip_set_attribute(&g_session, kTcpIp, 1, attr, data.data(), static_cast<uint16_t>(data.size()), &status);
    return status;
}

uint8_t SetIp(const std::string &ip, const std::string &mask, const std::string &gw) {
    std::vector<uint8_t> d;
    PutIp(d, ip);
    PutIp(d, mask);
    PutIp(d, gw);
    PutIp(d, "0.0.0.0");
    PutIp(d, "0.0.0.0");
    const auto none = PadString("");
    d.insert(d.end(), none.begin(), none.end());
    return SetAttr(5, d);
}

uint8_t SetControl(bool dhcp) {
    uint8_t d[4];
    eip_store_le32(d, dhcp ? 2 : 0);
    return SetAttr(3, std::vector<uint8_t>(d, d + 4));
}

uint8_t SetHost(const std::string &name) { return SetAttr(6, PadString(name)); }

uint8_t IdentityReset(uint8_t type) {
    const uint8_t path[] = {0x20, 0x01, 0x24, 0x01};
    uint8_t status = kNoReply;
    eip_cip(&g_session, 0x05, path, sizeof(path), &type, 1, nullptr, nullptr, &status, nullptr);
    return status;
}

/** One Set host inside a Multiple Service Packet. The packet's general status and the entry's status. */
void MultipleSetHost(const std::string &name, uint8_t *general, int *entry) {
    std::vector<uint8_t> e = {kGetSet, 3, 0x20, static_cast<uint8_t>(kTcpIp), 0x24, 0x01, 0x30, 0x06};
    const auto text = PadString(name);
    e.insert(e.end(), text.begin(), text.end());
    std::vector<uint8_t> d = {1, 0, 4, 0};  // one entry, at offset 4
    d.insert(d.end(), e.begin(), e.end());
    const uint8_t path[] = {0x20, 0x02, 0x24, 0x01};
    uint8_t reply[128];
    uint16_t len = sizeof(reply);
    *general = kNoReply;
    *entry = -1;
    eip_cip(&g_session, 0x0A, path, sizeof(path), d.data(), static_cast<uint16_t>(d.size()), reply, &len, general, nullptr);
    if ((*general == 0 || *general == kMultipleFailed) && len >= 8 && eip_load_le16(reply) == 1) {
        const uint16_t at = eip_load_le16(reply + 2);
        if (at + 3u <= len) {
            *entry = reply[at + 2];
        }
    }
}

// ---- the device coming and going -------------------------------------------------------------------------------------

bool Answers(const std::string &ip) {
    EipIdentity id;
    return eip_list_identity(ip.c_str(), &id, 500) == EIP_OK;
}

double WaitForDevice(const std::string &ip, double timeout, bool goneFirst) {
    const double t0 = bench::Seconds();
    if (goneFirst) {
        while (bench::Seconds() - t0 < 8 && Answers(ip)) {
            Sleep(200);
        }
    }
    while (bench::Seconds() - t0 < timeout) {
        if (Answers(ip)) {
            return bench::Seconds() - t0;
        }
        Sleep(300);
    }
    throw std::runtime_error("no device at " + ip + " after " + std::to_string(static_cast<int>(timeout)) + " s");
}

bool Has(const std::vector<std::string> &lines, const std::string &line) {
    for (const std::string &l : lines) {
        if (l == line) {
            return true;
        }
    }
    return false;
}

std::vector<std::string> ConsoleCmd(const std::string &ip, const std::string &c) {
    bench::Console con;
    con.Open(ip.c_str());
    return con.Cmd(c);
}

/** Class 1 packets received in half a second. */
int Class1Count(const std::string &ip) {
    Session s(ip);
    static EipIo io;
    EipIoConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.config_instance = 151;
    cfg.output_instance = 150;
    cfg.input_instance = 100;
    cfg.output_bytes = 64;
    cfg.input_bytes = 160;
    cfg.o2t_rpi_us = cfg.t2o_rpi_us = 20000;
    cfg.run_idle = 1;
    cfg.timeout_multiplier = 1;
    const int rc = eip_io_open(&g_session, &io, &cfg);
    if (rc != EIP_OK) {
        throw std::runtime_error(std::string("Forward_Open: ") + eip_status_text(rc));
    }
    uint8_t out[64] = {};
    uint8_t in[160];
    int got = 0;
    const double t0 = bench::Seconds();
    double next = t0;
    while (bench::Seconds() - t0 < 0.8) {  // 0.3 s to settle, as the suite does, then 0.5 s counted
        if (bench::Seconds() >= next) {
            eip_io_produce(&io, out, sizeof(out), 1);
            next += 0.02;
        }
        uint16_t len = sizeof(in);
        if (eip_io_consume(&io, in, &len, 5) == EIP_OK && bench::Seconds() - t0 >= 0.3) {
            got++;
        }
    }
    eip_io_close(&g_session, &io);
    return got;
}

// ---- USB, the way back -----------------------------------------------------------------------------------------------

/** Commands over the USB console. Retries while the board restarts. restart is sent without waiting for a reply. */
std::vector<std::string> UsbConsole(const std::string &port, const std::vector<std::string> &lines) {
    const std::string path = "\\\\.\\" + port;
    for (int attempt = 0; attempt < 20; attempt++) {
        HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (h == INVALID_HANDLE_VALUE) {
            Sleep(1000);
            continue;
        }
        DCB dcb;
        memset(&dcb, 0, sizeof(dcb));
        dcb.DCBlength = sizeof(dcb);
        GetCommState(h, &dcb);
        dcb.BaudRate = 115200;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        SetCommState(h, &dcb);
        COMMTIMEOUTS t = {MAXDWORD, MAXDWORD, 100, 0, 1000};
        SetCommTimeouts(h, &t);
        Sleep(500);
        PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
        std::vector<std::string> out;
        for (const std::string &line : lines) {
            const std::string text = line + "\n";
            DWORD n = 0;
            WriteFile(h, text.data(), static_cast<DWORD>(text.size()), &n, nullptr);
            if (line == "restart") {
                Sleep(300);
                out.push_back("(restart sent)");
                break;
            }
            std::string reply;
            const double t0 = bench::Seconds();
            while (bench::Seconds() - t0 < 5) {
                char buf[1024];
                n = 0;
                if (ReadFile(h, buf, sizeof(buf), &n, nullptr) && n) {
                    reply.append(buf, n);
                }
                if (("\n" + reply).find("\nok") != std::string::npos || reply.find("error:") != std::string::npos) {
                    break;
                }
            }
            // Read until the board goes quiet: closing the port with bytes still in flight leaves its USB transmit stuck.
            for (int quiet = 0; quiet < 3;) {
                char buf[1024];
                n = 0;
                if (ReadFile(h, buf, sizeof(buf), &n, nullptr) && n) {
                    reply.append(buf, n);
                    quiet = 0;
                } else {
                    quiet++;  // each empty read is the 100 ms timeout
                }
            }
            out.push_back(reply);
        }
        CloseHandle(h);
        return out;
    }
    throw std::runtime_error("USB console " + port + " not available");
}

void Recover(const std::string &ip, const std::string &port, bench::Results &r, const std::string &host = "") {
    printf("== putting the board back at %s (static) over USB %s\n", ip.c_str(), port.c_str());
    std::vector<std::string> cmds = {"set net_mode 1", "set ip " + ip, "set netmask 255.255.255.0", "set gateway 0.0.0.0"};
    if (!host.empty()) {
        cmds.push_back("set host " + host);
    }
    cmds.push_back("restart");
    try {
        UsbConsole(port, cmds);
        WaitForDevice(ip, 40, false);
        r.Check("board back at " + ip, true);
    } catch (const std::exception &x) {
        r.Check("board back at " + ip, false, x.what());
    }
}

void Run(const std::string &ip, const std::string &alt, const std::string &usb, bench::Results &r, std::string &storedHost) {
    printf("== starting point\n");
    const Net n0 = PrintNet(ip);
    r.Check("settable, static", n0.settable && n0.control == 0);

    printf("== refused while the motors are enabled\n");
    {
        bench::Console con;
        con.Open(ip.c_str());
        const std::string hostLine = bench::Find(con.Cmd("config"), "host ");
        const size_t sp = hostLine.find(' ');
        storedHost = hostLine.substr(sp + 1, hostLine.find(' ', sp + 1) - sp - 1);  // "-" is the default from the MAC
        const auto en = con.Cmd("enable");
        const bool enabled = en.back().rfind("ok", 0) == 0;
        r.Check("motors enabled for the refusal checks", enabled, en.back());
        if (enabled) {
            uint8_t g1, g2, g3, gm;
            int entry;
            {
                Session s(ip);
                g1 = SetIp(alt, n0.netmask, n0.gateway);
                g2 = SetHost("should-not-stick");
                g3 = IdentityReset(0);
                MultipleSetHost("nor-this", &gm, &entry);
            }
            r.Check("set IP refused with 0x10", g1 == kStateConflict, Hex(g1));
            r.Check("set host refused with 0x10", g2 == kStateConflict, Hex(g2));
            r.Check("identity reset refused with 0x10", g3 == kStateConflict, Hex(g3));
            r.Check("a set inside a Multiple Service Packet is refused too", gm == kMultipleFailed && entry == kStateConflict,
                    Hex(gm) + " entry " + std::to_string(entry));
        }
        con.Cmd("disable");
    }
    Sleep(300);
    r.Check("nothing changed", PrintNet(ip).host == n0.host);

    printf("== host name applies live, and is stored\n");
    {
        Session s(ip);
        const uint8_t g = SetHost("ecam-bench");
        Sleep(400);
        const std::string h = ReadNet().host;
        r.Check("set host accepted and reads back", g == 0 && h == "ecam-bench", Hex(g) + " '" + h + "'");
    }

    printf("== move to %s\n", alt.c_str());
    {
        Session s(ip);
        const uint8_t g = SetIp(alt, n0.netmask, n0.gateway);
        r.Check("set IP accepted", g == 0, Hex(g));
    }
    printf("    answered at %s after %.1f s\n", alt.c_str(), WaitForDevice(alt, 40, true));
    const Net n1 = PrintNet(alt);
    r.Check("new address running, mask and gateway kept", n1.ip == alt && n1.netmask == n0.netmask && n1.gateway == n0.gateway);
    r.Check("host name survived the restart", n1.host == "ecam-bench");
    r.Check("console at the new address; settings show it", Has(ConsoleCmd(alt, "config"), "ip " + alt));
    const int packets = Class1Count(alt);
    r.Check("Class 1 works at the new address", packets > 15, std::to_string(packets) + " packets in 0.5 s");

    printf("== back to %s\n", ip.c_str());
    {
        Session s(alt);
        SetIp(ip, n0.netmask, n0.gateway);
    }
    WaitForDevice(ip, 40, true);
    r.Check("back at " + ip, PrintNet(ip).ip == ip);

    printf("== DHCP with no server: stays on the stored static address\n");
    {
        Session s(ip);
        const uint8_t g = SetControl(true);
        r.Check("set DHCP accepted", g == 0, Hex(g));
    }
    printf("    answered at %s after %.1f s (DHCP timed out, fell back)\n", ip.c_str(), WaitForDevice(ip, 60, true));
    const Net n2 = PrintNet(ip);
    r.Check("still at the stored address", n2.ip == ip);
    r.Check("attr 3 reports static (the address in use is not from DHCP)", n2.control == 0);
    const std::string e = bench::Find(ConsoleCmd(ip, "eip"), "eip network");
    r.Check("'eip' shows DHCP stored and the fallback",
            e.find("stored=dhcp") != std::string::npos && e.find("no DHCP server") != std::string::npos, e);

    printf("== back to static\n");
    {
        Session s(ip);
        const uint8_t g = SetControl(false);
        r.Check("set static accepted", g == 0, Hex(g));
    }
    WaitForDevice(ip, 40, true);
    const auto cfg = ConsoleCmd(ip, "config");
    r.Check("static again (net_mode 1)", Has(cfg, "net_mode 1") && Has(cfg, "ip " + ip));

    printf("== identity reset type 0 (restart)\n");
    {
        Session s(ip);
        const uint8_t g = IdentityReset(0);
        r.Check("reset 0 accepted", g == 0, Hex(g));
    }
    const double t = WaitForDevice(ip, 40, true);
    r.Check("restarted and back at " + ip, true, std::to_string(t) + " s");

    printf("== identity reset type 1 (factory network settings: DHCP, 192.168.0.110), checked over USB\n");
    {
        Session s(ip);
        const uint8_t g = IdentityReset(1);
        r.Check("reset 1 accepted", g == 0, Hex(g));
    }
    Sleep(12000);  // the restart, then the DHCP attempt
    const std::string factory = UsbConsole(usb, {"config"})[0];
    r.Check("factory network settings (DHCP, 192.168.0.110, default host)",
            factory.find("net_mode 0") != std::string::npos && factory.find("ip 192.168.0.110") != std::string::npos &&
                factory.find("host - (default)") != std::string::npos,
            factory);
    Recover(ip, usb, r, storedHost);
}

}  // namespace

int NetTest(int argc, char **argv) {
    std::string ip = "10.168.41.5", alt = "10.168.41.6", usb = "COM3";
    int plain = 0;
    for (int i = 2; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--usb" && i + 1 < argc) {
            usb = argv[++i];
        } else if (plain++ == 0) {
            ip = a;
        } else {
            alt = a;
        }
    }
    if (ip.rfind(kBenchPrefix, 0) != 0 || alt.rfind(kBenchPrefix, 0) != 0) {
        printf("nettest moves the board and resets its network settings: only on %sx, not %s\n", kBenchPrefix, ip.c_str());
        return 2;
    }
    bench::Results r;
    std::string storedHost;
    try {
        Run(ip, alt, usb, r, storedHost);
    } catch (const std::exception &x) {
        r.Check("nettest ran to the end", false, x.what());
        Recover(ip, usb, r, storedHost);
    }
    printf("\n%d/%d passed\n", r.passed, r.total);
    return r.passed == r.total && r.total > 0 ? 0 : 1;
}

/** Closes the USB port mid-transfer, again and again, and checks that the next open still gets replies:
 *  ecam_bench usbclose COM3 [TRIES] [stalled]. A rude host, on purpose. Motors stay as they are; the board restarts nothing. */
int UsbClose(int argc, char **argv) {
    if (argc < 3) {
        printf("usbclose needs a port, COM3\n");
        return 2;
    }
    const std::string path = std::string("\\\\.\\") + argv[2];
    const int tries = argc > 3 ? atoi(argv[3]) : 30;
    const bool stalled = argc > 4 && strcmp(argv[4], "stalled") == 0;
    auto open = [&]() {
        HANDLE h = INVALID_HANDLE_VALUE;
        for (int i = 0; i < 20 && h == INVALID_HANDLE_VALUE; i++) {
            h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
            if (h == INVALID_HANDLE_VALUE) {
                Sleep(100);
            }
        }
        if (h == INVALID_HANDLE_VALUE) {
            return h;
        }
        DCB dcb;
        memset(&dcb, 0, sizeof(dcb));
        dcb.DCBlength = sizeof(dcb);
        GetCommState(h, &dcb);
        dcb.BaudRate = 115200;
        dcb.fBinary = TRUE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        SetCommState(h, &dcb);
        COMMTIMEOUTS t = {MAXDWORD, MAXDWORD, 50, 0, 500};
        SetCommTimeouts(h, &t);
        return h;
    };
    auto say = [](HANDLE h, const char *text) {
        DWORD n = 0;
        WriteFile(h, text, static_cast<DWORD>(strlen(text)), &n, nullptr);
    };
    int dead = 0, slow = 0;
    for (int i = 1; i <= tries; i++) {
        HANDLE h = open();
        if (h == INVALID_HANDLE_VALUE) {
            printf("try %d: port did not open\n", i);
            return 1;
        }
        // A long reply on the way out, and the port shut under it after a varied few milliseconds.
        if (stalled) {  // the PC stops taking data first, so a transfer is left waiting when the port shuts
            say(h, "tel 20\n");
            Sleep(4000);
        } else {
            say(h, "config\n");
            Sleep(static_cast<DWORD>(1 + i % 12));
        }
        CloseHandle(h);
        Sleep(150);

        // Keep asking on fresh opens: slow to come back is a stall, never coming back is the bug.
        const double asked = bench::PreciseSeconds();
        bool replied = false;
        for (int attempt = 0; attempt < 6 && !replied; attempt++) {
            h = open();
            Sleep(200);
            PurgeComm(h, PURGE_RXCLEAR);
            say(h, stalled ? "tel 0\nversion\n" : "version\n");
            std::string reply;
            const double t0 = bench::PreciseSeconds();
            while (bench::PreciseSeconds() - t0 < 1.5 && reply.find("ECam_Firmware") == std::string::npos) {
                char buf[256];
                DWORD n = 0;
                if (ReadFile(h, buf, sizeof(buf), &n, nullptr) && n) {
                    reply.append(buf, n);
                }
            }
            // Read until quiet, so this test's own close is a clean one.
            for (int quiet = 0; quiet < 3;) {
                char buf[256];
                DWORD n = 0;
                quiet = ReadFile(h, buf, sizeof(buf), &n, nullptr) && n ? 0 : quiet + 1;
            }
            CloseHandle(h);
            replied = reply.find("ECam_Firmware") != std::string::npos;
            if (replied && attempt > 0) {
                slow++;
                printf("try %d: replied on open %d, after %.1f s\n", i, attempt + 1, bench::PreciseSeconds() - asked);
            }
        }
        if (!replied) {
            printf("try %d: NO REPLY in 6 opens\n", i);
            dead++;
            break;
        }
    }
    printf("usbclose: %d tries, %d slow to reply\n", tries, slow);
    printf("usbclose: %s\n", dead ? "USB transmit went dead" : "every reopen replied");
    return dead ? 1 : 0;
}

/** nettest's way back, tried read-only: ecam_bench usbcheck COM3 */
int UsbCheck(int argc, char **argv) {
    if (argc < 3) {
        printf("usbcheck needs a port, COM3\n");
        return 2;
    }
    try {
        for (const std::string &reply : UsbConsole(argv[2], {"version", "config"})) {
            printf("%s\n", reply.c_str());
        }
    } catch (const std::exception &x) {
        printf("STOPPED: %s\n", x.what());
        return 1;
    }
    return 0;
}

int NetCheck(int argc, char **argv) {
    if (argc < 3) {
        printf("netcheck needs the adapter's address on this PC\n");
        return 2;
    }
    const std::string ip = argv[2];
    bool local = false;
    for (uint8_t i = 0; i < 16; i++) {
        EipNetifInfo n;
        if (eip_hal_netif_get(i, &n) != EIP_OK) {
            break;
        }
        char t[20];
        snprintf(t, sizeof(t), "%u.%u.%u.%u", n.ip[0], n.ip[1], n.ip[2], n.ip[3]);
        local = local || ip == t;
    }
    if (!local) {
        printf("netcheck sends real network settings: only to an adapter on this PC, not %s\n", ip.c_str());
        return 2;
    }
    try {
        Session s(ip);
        const Net n = ReadNet();
        printf("read: control %u ip %s mask %s gw %s host '%s' settable %d\n", n.control, n.ip.c_str(), n.netmask.c_str(),
               n.gateway.c_str(), n.host.c_str(), n.settable ? 1 : 0);
        printf("set host: %s\n", Hex(SetHost("ecam-bench")).c_str());
        Sleep(300);
        printf("set dhcp: %s\n", Hex(SetControl(true)).c_str());
        Sleep(300);
        printf("set static: %s\n", Hex(SetControl(false)).c_str());
        Sleep(300);
        printf("control now %u\n", ReadNet().control);
        // Refused with 0x0c unless the adapter is static: a PC on DHCP stays that way, because nothing here applies it.
        printf("set ip: %s\n", Hex(SetIp("10.168.41.6", "255.255.255.0", "10.168.41.254")).c_str());
        Sleep(300);
        uint8_t gm;
        int entry;
        MultipleSetHost("nor-this", &gm, &entry);
        printf("multiple: %s entry %d\n", Hex(gm).c_str(), entry);
        Sleep(300);
        printf("reset 0: %s\n", Hex(IdentityReset(0)).c_str());
        Sleep(500);
        printf("reset 1: %s\n", Hex(IdentityReset(1)).c_str());
        Sleep(500);
    } catch (const std::exception &x) {
        printf("STOPPED: %s\n", x.what());
        return 1;
    }
    return 0;
}
