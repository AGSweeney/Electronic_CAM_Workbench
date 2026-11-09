// MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
// ecam_bench: native bench tools for the ECam board, with no interpreter in the timing path.
//
//   ecam_bench scanner IP [--rpi MS] [--seconds N] [--mult M] [--flags N] [--idle]
//       Own a Class 1 connection through QuickEIP. One spinning thread paces O->T and timestamps T->O.
//       stdin while running: flags N | value OFFSET U32 | status | report | quit
//   ecam_bench console IP [--tel MS --seconds N] [COMMAND ...]
//   ecam_bench serial COMn [--tel MS --seconds N] [COMMAND ...]
//       The board's text console over TCP 9500 or USB; with --tel, time the TEL lines.
//   ecam_bench tag IP NAME COUNT          read a PLC's SINT array by name
//   ecam_bench adapter [SECONDS] [refuse] a local QuickEIP adapter with ECam's assemblies, to test this tool
//   ecam_bench soak ...  /  plcclear ...  the gate soak: see soak.cpp
//   ecam_bench nettest ...  /  netcheck ... network settings over CIP: see nettest.cpp
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>
#include <timeapi.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "eip.h"
#include "eip_hal.h"
#include "suites.h"

namespace {

constexpr uint16_t kAsmCmd = 150, kAsmSts = 100, kAsmCfg = 151;
constexpr uint16_t kCmdSize = 64, kStsSize = 160;
constexpr uint16_t kConsolePort = 9500;
constexpr size_t kHeartbeatOffset = 20;

int64_t NowUs() {
    static const int64_t freq = [] {
        LARGE_INTEGER f;
        QueryPerformanceFrequency(&f);
        return f.QuadPart;
    }();
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    return c.QuadPart / freq * 1000000 + c.QuadPart % freq * 1000000 / freq;
}

/** Gaps between events: count, shortest, longest, and how many passed the limits. */
struct Gaps {
    uint64_t count = 0, over1 = 0, over30 = 0, over80 = 0;
    int64_t last = 0, shortest = INT64_MAX, longest = 0, limit1 = 0;
    void Hit(int64_t now) {
        if (last) {
            const int64_t g = now - last;
            shortest = g < shortest ? g : shortest;
            longest = g > longest ? g : longest;
            over1 += g > limit1;
            over30 += g > 30000;
            over80 += g > 80000;
        }
        last = now;
        count++;
    }
    void Print(const char *name, double seconds) const {
        printf("%s: %llu in %.1f s (%.2f/s), gap %.3f to %.3f ms, over %.1f ms: %llu, over 30 ms: %llu, over 80 ms: %llu\n",
               name, (unsigned long long)count, seconds, seconds > 0 ? count / seconds : 0.0,
               count > 1 ? shortest / 1000.0 : 0.0, longest / 1000.0, limit1 / 1000.0, (unsigned long long)over1,
               (unsigned long long)over30, (unsigned long long)over80);
    }
};

struct Args {
    std::vector<std::string> plain;
    int rpiMs = 20, mult = 1, telMs = 0, flags = 0;
    double seconds = 0;
    bool idle = false;
};

Args Parse(int argc, char **argv, int from) {
    Args a;
    for (int i = from; i < argc; i++) {
        const std::string s = argv[i];
        auto next = [&]() { return i + 1 < argc ? argv[++i] : "0"; };
        if (s == "--rpi") a.rpiMs = atoi(next());
        else if (s == "--mult") a.mult = atoi(next());
        else if (s == "--tel") a.telMs = atoi(next());
        else if (s == "--flags") a.flags = static_cast<int>(strtol(next(), nullptr, 0));
        else if (s == "--seconds") a.seconds = atof(next());
        else if (s == "--idle") a.idle = true;
        else a.plain.push_back(s);
    }
    return a;
}

void Fail(const char *what, int rc, const EipSession *s) {
    printf("%s: %s", what, eip_status_text(rc));
    if (s) {
        printf(" (CIP 0x%02x extended 0x%04x)", eip_last_cip_status(s), eip_last_cip_extended(s));
    }
    printf("\n");
}

// ---------------------------------------------------------------------------------------------------------------------
// Class 1 scanner
// ---------------------------------------------------------------------------------------------------------------------

EipSession g_session;
EipIo g_io;

int Scanner(const Args &a) {
    if (a.plain.empty()) {
        printf("scanner needs an IP address\n");
        return 2;
    }
    EipOpenConfig open;
    memset(&open, 0, sizeof(open));
    open.ip = a.plain[0].c_str();
    open.slot = -1;
    open.vendor_id = 0x1234;
    open.serial_number = 0x7E58;
    open.timeout_ms = 3000;
    int rc = eip_session_open(&g_session, &open);
    if (rc != EIP_OK) {
        Fail("session", rc, nullptr);
        return 1;
    }
    const uint32_t rpiUs = static_cast<uint32_t>(a.rpiMs) * 1000;
    EipIoConfig io;
    memset(&io, 0, sizeof(io));
    io.config_instance = kAsmCfg;
    io.output_instance = kAsmCmd;
    io.input_instance = kAsmSts;
    io.output_bytes = kCmdSize;
    io.input_bytes = kStsSize;
    io.o2t_rpi_us = io.t2o_rpi_us = rpiUs;
    io.run_idle = 1;
    io.timeout_multiplier = static_cast<uint8_t>(a.mult);
    rc = eip_io_open(&g_session, &g_io, &io);
    if (rc != EIP_OK) {
        Fail("Forward_Open", rc, &g_session);
        eip_session_close(&g_session);
        return 1;
    }
    printf("connected, RPI %d ms, timeout %d ms\n", a.rpiMs, a.rpiMs * (4 << a.mult));
    fflush(stdout);

    std::mutex lock;
    uint8_t cmd[kCmdSize] = {};
    uint8_t sts[kStsSize] = {};
    eip_store_le16(cmd, static_cast<uint16_t>(a.flags));
    std::atomic<bool> stop{false}, wantReport{false}, wantStatus{false};

    std::thread input([&] {
        std::string line;
        while (!stop && std::getline(std::cin, line)) {
            std::istringstream in(line);
            std::string word;
            in >> word;
            if (word == "quit") {
                stop = true;
            } else if (word == "report") {
                wantReport = true;
            } else if (word == "status") {
                wantStatus = true;
            } else if (word == "flags") {
                long v = 0;
                in >> std::ws;
                std::string t;
                in >> t;
                v = strtol(t.c_str(), nullptr, 0);
                std::lock_guard<std::mutex> g(lock);
                eip_store_le16(cmd, static_cast<uint16_t>(v));
            } else if (word == "value") {
                size_t off = 0;
                long long v = 0;
                in >> off >> v;
                if (off + 4 <= kCmdSize) {
                    std::lock_guard<std::mutex> g(lock);
                    eip_store_le32(cmd + off, static_cast<uint32_t>(v));
                }
            }
        }
    });
    input.detach();

    timeBeginPeriod(1);
    SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);

    Gaps tx, rx;
    tx.limit1 = rx.limit1 = static_cast<int64_t>(rpiUs) * 3 / 2;
    uint64_t txLate = 0, rxBad = 0;
    uint32_t heartbeat = 0, heartbeatFirst = 0;
    const int64_t start = NowUs();
    int64_t next = start;
    const int64_t end = a.seconds > 0 ? start + static_cast<int64_t>(a.seconds * 1e6) : INT64_MAX;
    auto report = [&] {
        const double t = (NowUs() - start) / 1e6;
        tx.Print("O->T", t);
        rx.Print("T->O", t);
        printf("late sends %llu, bad reads %llu, heartbeat %lu (+%lu)\n", (unsigned long long)txLate,
               (unsigned long long)rxBad, (unsigned long)heartbeat, (unsigned long)(heartbeat - heartbeatFirst));
        fflush(stdout);
    };

    while (!stop) {
        const int64_t now = NowUs();
        if (now >= end) {
            break;
        }
        if (now >= next) {
            uint8_t out[kCmdSize];
            {
                std::lock_guard<std::mutex> g(lock);
                memcpy(out, cmd, sizeof(out));
            }
            eip_io_produce(&g_io, out, kCmdSize, a.idle ? 0 : 1);
            tx.Hit(now);
            next += rpiUs;
            if (next < now) {  // lost a whole period: restart the grid instead of bursting
                next = now + rpiUs;
                txLate++;
            }
        }
        if (eip_hal_wait(&g_io.sock, 1, 0) > 0) {
            uint16_t len = sizeof(sts);
            if (eip_io_consume(&g_io, sts, &len, 1) == EIP_OK) {
                rx.Hit(NowUs());
                heartbeat = eip_load_le32(sts + kHeartbeatOffset);
                if (rx.count == 1) {
                    heartbeatFirst = heartbeat;
                }
            } else {
                rxBad++;
            }
        }
        if (wantReport.exchange(false)) {
            report();
        }
        if (wantStatus.exchange(false)) {
            for (size_t i = 0; i < sizeof(sts); i++) {
                printf("%02x", sts[i]);
            }
            printf("\n");
            fflush(stdout);
        }
    }
    stop = true;
    SetPriorityClass(GetCurrentProcess(), NORMAL_PRIORITY_CLASS);
    timeEndPeriod(1);
    report();
    rc = eip_io_close(&g_session, &g_io);
    printf("Forward_Close: %s\n", eip_status_text(rc));
    eip_session_close(&g_session);
    fflush(stdout);
    _exit(0);  // the input thread may be blocked on stdin
}

// ---------------------------------------------------------------------------------------------------------------------
// Text console, over TCP or USB
// ---------------------------------------------------------------------------------------------------------------------

struct Line {
    std::function<void(const std::string &)> write;
    std::function<int(char *, int, uint32_t)> read;  // bytes, 0 on nothing, < 0 when the link has gone
};

/** Calls onLine for each complete line until done() or the time is up. */
void ReadLines(Line &l, std::string &pending, int64_t until, const std::function<bool(const std::string &)> &onLine) {
    char buf[4096];
    while (NowUs() < until) {
        const int n = l.read(buf, sizeof(buf), 20);
        if (n < 0) {
            printf("link closed\n");
            return;
        }
        pending.append(buf, static_cast<size_t>(n));
        size_t at;
        while ((at = pending.find('\n')) != std::string::npos) {
            std::string one = pending.substr(0, at);
            pending.erase(0, at + 1);
            if (!one.empty() && one.back() == '\r') {
                one.pop_back();
            }
            if (onLine(one)) {
                return;
            }
        }
    }
}

int Console(Line &l, const Args &a) {
    std::string pending;
    ReadLines(l, pending, NowUs() + 500000, [](const std::string &) { return false; });  // banner
    auto command = [&](const std::string &c) {
        l.write(c + "\n");
        bool replied = false;
        ReadLines(l, pending, NowUs() + 3000000, [&](const std::string &s) {
            if (s.rfind("TEL ", 0) == 0) {
                return false;
            }
            printf("%s\n", s.c_str());
            replied = s.rfind("ok", 0) == 0 || s.rfind("error", 0) == 0;
            return replied;
        });
        if (!replied) {
            printf("no reply to '%s'\n", c.c_str());
        }
    };
    for (size_t i = 1; i < a.plain.size(); i++) {
        command(a.plain[i]);
    }
    if (a.telMs && a.seconds > 0) {
        Gaps tel;
        tel.limit1 = static_cast<int64_t>(a.telMs) * 1500;
        uint64_t bytes = 0;
        const int64_t start = NowUs();
        l.write("tel " + std::to_string(a.telMs) + "\n");
        ReadLines(l, pending, start + static_cast<int64_t>(a.seconds * 1e6), [&](const std::string &s) {
            if (s.rfind("TEL ", 0) == 0) {
                tel.Hit(NowUs());
                bytes += s.size() + 2;
            }
            return false;
        });
        tel.Print("TEL", (NowUs() - start) / 1e6);
        printf("average line %.0f bytes\n", tel.count ? static_cast<double>(bytes) / tel.count : 0.0);
        command("tel 0");
    }
    fflush(stdout);
    return 0;
}

int ConsoleTcp(const Args &a) {
    if (a.plain.empty()) {
        printf("console needs an IP address\n");
        return 2;
    }
    in_addr addr;
    addr.s_addr = inet_addr(a.plain[0].c_str());
    uint8_t ip[4];
    memcpy(ip, &addr, 4);
    const eip_sock_t s = eip_hal_tcp_connect(ip, kConsolePort, 3000);
    if (s == EIP_SOCK_INVALID) {
        printf("no console connection\n");
        return 1;
    }
    Line l;
    l.write = [s](const std::string &t) {
        eip_hal_tcp_send(s, reinterpret_cast<const uint8_t *>(t.data()), static_cast<uint16_t>(t.size()));
    };
    l.read = [s](char *b, int cap, uint32_t ms) {
        return eip_hal_tcp_recv(s, reinterpret_cast<uint8_t *>(b), static_cast<uint16_t>(cap), ms);
    };
    const int rc = Console(l, a);
    eip_hal_tcp_close(s);
    return rc;
}

int ConsoleSerial(const Args &a) {
    if (a.plain.empty()) {
        printf("serial needs a port, COM3\n");
        return 2;
    }
    const std::string path = "\\\\.\\" + a.plain[0];
    HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        printf("cannot open %s\n", a.plain[0].c_str());
        return 1;
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
    Sleep(300);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    Line l;
    l.write = [h](const std::string &t) {
        DWORD n = 0;
        WriteFile(h, t.data(), static_cast<DWORD>(t.size()), &n, nullptr);
    };
    l.read = [h](char *b, int cap, uint32_t ms) {
        COMMTIMEOUTS t = {MAXDWORD, MAXDWORD, ms, 0, 1000};
        SetCommTimeouts(h, &t);
        DWORD n = 0;
        return ReadFile(h, b, static_cast<DWORD>(cap), &n, nullptr) ? static_cast<int>(n) : -1;
    };
    const int rc = Console(l, a);
    CloseHandle(h);
    return rc;
}

// ---------------------------------------------------------------------------------------------------------------------
// PLC tags and the local test adapter
// ---------------------------------------------------------------------------------------------------------------------

int Tag(const Args &a) {
    if (a.plain.size() < 3) {
        printf("tag needs IP NAME COUNT\n");
        return 2;
    }
    EipOpenConfig open;
    memset(&open, 0, sizeof(open));
    open.ip = a.plain[0].c_str();
    open.slot = -1;
    open.timeout_ms = 3000;
    int rc = eip_session_open(&g_session, &open);
    if (rc != EIP_OK) {
        Fail("session", rc, nullptr);
        return 1;
    }
    uint8_t data[512];
    uint16_t len = sizeof(data);
    uint16_t type = 0;
    rc = eip_tag_read(&g_session, a.plain[1].c_str(), static_cast<uint16_t>(atoi(a.plain[2].c_str())), data, &len, &type,
                      nullptr);
    if (rc != EIP_OK) {
        Fail("read", rc, &g_session);
    } else {
        printf("type 0x%04x, %u bytes:", type, len);
        for (uint16_t i = 0; i < len; i++) {
            printf(" %02x", data[i]);
        }
        printf("\n");
    }
    eip_session_close(&g_session);
    return rc == EIP_OK ? 0 : 1;
}

int Adapter(const Args &a) {
    static const EipAssemblyConfig assemblies[] = {
        {kAsmCmd, EIP_ASM_CONSUMED, kCmdSize, "Command", nullptr, 0},
        {kAsmSts, EIP_ASM_PRODUCED, kStsSize, "Status", nullptr, 0},
        {kAsmCfg, EIP_ASM_CONFIG, 0, "Config", nullptr, 0},
    };
    static bool refuse = false;
    for (const std::string &p : a.plain) {
        refuse = refuse || p == "refuse";
    }
    EipAdapterConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.assemblies = assemblies;
    cfg.assembly_count = 3;
    // For `ecam_bench netcheck`: print what arrives. This changes nothing on the PC.
    cfg.apply_interface = [](void *, const EipInterfaceConfig *c) {
        printf("applied: control %u ip %u.%u.%u.%u mask %u.%u.%u.%u gw %u.%u.%u.%u host '%s'\n", c->control, c->ip[0],
               c->ip[1], c->ip[2], c->ip[3], c->netmask[0], c->netmask[1], c->netmask[2], c->netmask[3], c->gateway[0],
               c->gateway[1], c->gateway[2], c->gateway[3], c->host_name);
        fflush(stdout);
    };
    cfg.reset = [](void *, uint8_t type) {
        printf("reset type %u\n", type);
        fflush(stdout);
    };
    cfg.allow_change = [](void *, uint8_t) -> uint8_t { return refuse ? 0x10 : 0; };
    const int rc = eip_adapter_start(&cfg);
    if (rc != EIP_OK) {
        Fail("adapter", rc, nullptr);
        return 1;
    }
    timeBeginPeriod(1);
    const double seconds = a.plain.empty() ? 30 : atof(a.plain[0].c_str());
    EipAdapterView up;
    eip_adapter_snapshot(&up);
    printf("adapter up for %.0f s at %u.%u.%u.%u\n", seconds, up.ip[0], up.ip[1], up.ip[2], up.ip[3]);
    fflush(stdout);
    const int64_t start = NowUs();
    uint8_t sts[kStsSize] = {};
    while (NowUs() - start < static_cast<int64_t>(seconds * 1e6)) {
        eip_store_le32(sts + kHeartbeatOffset, static_cast<uint32_t>((NowUs() - start) / 1000));
        eip_adapter_write(kAsmSts, sts, sizeof(sts));
        eip_adapter_poll();
        Sleep(0);
    }
    timeEndPeriod(1);
    EipAdapterView v;
    if (eip_adapter_snapshot(&v) == EIP_OK) {
        printf("adapter: opens %lu timeouts %lu rx %lu tx %lu dropped seq %lu source %lu bad %lu\n",
               (unsigned long)v.opens, (unsigned long)v.timeouts, (unsigned long)v.rx_packets,
               (unsigned long)v.tx_packets, (unsigned long)v.rx_dropped_seq, (unsigned long)v.rx_dropped_source,
               (unsigned long)v.rx_dropped_bad);
    }
    return 0;
}

}  // namespace

int Soak(int argc, char **argv);      // soak.cpp
int PlcClear(int argc, char **argv);  // soak.cpp
int NetTest(int argc, char **argv);   // nettest.cpp
int NetCheck(int argc, char **argv);  // nettest.cpp
int UsbCheck(int argc, char **argv);  // nettest.cpp
int UsbClose(int argc, char **argv);  // nettest.cpp

int main(int argc, char **argv) {
    setvbuf(stdout, nullptr, _IOLBF, 4096);
    const std::string mode = argc > 1 ? argv[1] : "";
    const Args a = Parse(argc, argv, 2);
    if (mode == "scanner") return Scanner(a);
    if (mode == "console") return ConsoleTcp(a);
    if (mode == "serial") return ConsoleSerial(a);
    if (mode == "tag") return Tag(a);
    if (mode == "adapter") return Adapter(a);
    if (mode == "soak") return Soak(argc, argv);
    if (mode == "test" || mode == "statustest" || mode == "cmdtest" || mode == "motiontest" || mode == "livetest" ||
        mode == "loadtest" || mode == "edstest" || mode == "uitest" || mode == "plcloss") {
        // ecam_bench SUITE [IP] [--plc PLC_IP] [--restore FILE] [--workbench EXE PROJECT] [--tools DIR] [--minutes N] [--eds FILE]
        std::string ip = "10.168.41.5", restore, workbench, project, tools = "ProjectData/build/Release";
        std::string eds = "Firmware/eds/ECam_ClearCore.eds";
        double minutes = 30;
        for (int i = 2; i < argc; i++) {
            const std::string s = argv[i];
            if (s == "--restore" && i + 1 < argc) {
                restore = argv[++i];
            } else if (s == "--workbench" && i + 2 < argc) {
                workbench = argv[++i];
                project = argv[++i];
            } else if (s == "--tools" && i + 1 < argc) {
                tools = argv[++i];
            } else if (s == "--minutes" && i + 1 < argc) {
                minutes = atof(argv[++i]);
            } else if (s == "--eds" && i + 1 < argc) {
                eds = argv[++i];
            } else if (s == "--plc" && i + 1 < argc) {
                bench::SetPlc(argv[++i]);
                // Ctrl-C: the PLC would go on holding the last command.
                SetConsoleCtrlHandler([](DWORD) -> BOOL {
                    bench::ClearPlc();
                    ExitProcess(130);
                }, TRUE);
            } else {
                ip = s;
            }
        }
        bench::UseNetworkOf(ip);
        if (mode == "statustest") return bench::SuiteStatus(ip, restore, workbench, project);
        if (mode == "cmdtest") return bench::SuiteCmd(ip, restore);
        if (mode == "motiontest") return bench::SuiteMotion(ip, restore);
        if (mode == "livetest") return bench::SuiteLive(ip, restore, tools);
        if (mode == "loadtest") return bench::SuiteLoad(ip, restore, minutes);
        if (mode == "edstest") return bench::SuiteEds(ip, eds);
        if (mode == "plcloss") {
            if (!bench::UsingPlc()) {
                printf("plcloss needs --plc PLC_IP\n");
                return 2;
            }
            return bench::SuitePlcLoss(ip, restore, minutes == 30 ? 300 : minutes * 60);
        }
        if (mode == "uitest") {
            return bench::SuiteUi(ip, workbench.empty() ? tools + "\\board_ui_plc.exe" : workbench,
                                  project.empty() ? "samples\\bench_driven_crank.ecam" : project);
        }
        return bench::SuiteTest(ip);
    }
    if (mode == "identity" || mode == "scan" || mode == "net" || mode == "io" || mode == "status" || mode == "setip" ||
        mode == "sethost" || mode == "setdhcp" || mode == "reset") {
        // ecam_bench TOOL IP [ARGS]
        const std::string ip = a.plain.empty() ? "10.168.41.5" : a.plain[0];
        bench::UseNetworkOf(ip);
        return bench::NetTool(mode, ip, a.plain.size() > 1 ? std::vector<std::string>(a.plain.begin() + 1, a.plain.end()) : std::vector<std::string>());
    }
    if (mode == "plcclear") return PlcClear(argc, argv);
    if (mode == "nettest") return NetTest(argc, argv);
    if (mode == "netcheck") return NetCheck(argc, argv);
    if (mode == "usbcheck") return UsbCheck(argc, argv);
    if (mode == "usbclose") return UsbClose(argc, argv);
    printf("usage: ecam_bench scanner|console|serial|tag|adapter|soak|plcclear|nettest|netcheck|usbcheck|usbclose|plcloss|test|statustest|cmdtest|motiontest|livetest|loadtest|edstest|uitest|identity|scan|net|io|status|setip|sethost|setdhcp|reset ... (see main.cpp)\n");
    return 2;
}
