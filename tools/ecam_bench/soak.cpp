// MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
// The gate soak: the real setup, with a PLC in control and the motors turning. M-0 is the crank, M-1 a reversing gear.
//
//   ecam_bench soak BOARD_IP PLC_IP MINUTES RESTORE_PROFILE [--tel MS] [--ping] [--tools DIR] [--ccio N]
//   ecam_bench plcclear PLC_IP
//
// By default nothing runs alongside the PLC but a diag each period, because a production machine has no telemetry.
// --tel and --ping add load for a stress run. Commands go by tag write to ECam_O, the only tag this ever writes, and
// the PLC's own Class 1 connection carries them. ECam_O is cleared on every way out, because the PLC goes on holding
// what was written last. If this is killed outright, run plcclear.
//
// DIR holds pos_hw.exe and scope_hw.exe (default ProjectData/build/Release), which read each drive over its own USB.
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <intrin.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "bench_common.h"
#include "bench_link.h"
#include "eip.h"
#include "eip_hal.h"

namespace {

using bench::Console;
using bench::Exec;
using bench::Find;
using bench::IntAfter;
using bench::Results;
using bench::Seconds;

constexpr uint16_t kCmdSize = 64, kStsSize = 160, kDiscoveryPort = 9502;
const char *const kOutTag = "ECam_O";  // the only PLC variable this writes
const char *const kInTag = "ECam_I";

constexpr uint16_t kEnable = 1 << 0, kControl = 1 << 1, kStart = 1 << 2, kStop = 1 << 3;
constexpr uint16_t kFlagEnabled = 1 << 2, kFlagFaulted = 1 << 9, kFlagPlc = 1 << 12, kAxisFault = 1 << 4;
constexpr uint16_t kCrankOff = 0, kCrankRunning = 2;
constexpr size_t kRpmOffset = 4;  // CrankRpm in the command assembly, rpm x 100

constexpr double kCpu = 813.7;
constexpr int kNum = -7, kDen = 3, kCpr = 4000, kRpm = 120;
constexpr double kWant0 = 5.0 * kRpm;                                       // M-0: 4000 counts a rev, 800-step motor
constexpr double kWant1 = kRpm * 7.0 / kDen * kCpu / 800.0;                 // M-1
constexpr const char *kSerialM0 = "65502511", *kSerialM1 = "59401211";      // by serial: Windows renumbers COM ports

std::atomic<bool> g_abort{false};

BOOL WINAPI OnCtrl(DWORD) {
    g_abort = true;
    return TRUE;
}

/** Sleeps in slices. False when the run has been aborted. */
bool Wait(int ms) {
    while (ms > 0 && !g_abort) {
        const int step = ms < 50 ? ms : 50;
        Sleep(static_cast<DWORD>(step));
        ms -= step;
    }
    return !g_abort;
}

// ---------------------------------------------------------------------------------------------------------------------

struct Status {
    bool valid = false;
    uint16_t flags = 0, fault = 0, crank = 0, axisFlags = 0;
    int32_t angle = 0, rev = 0, rpm = 0, counts = 0;
    uint32_t commloss = 0;
};

Status Decode(const uint8_t *b) {
    Status s;
    s.valid = true;
    s.flags = eip_load_le16(b + 2);
    s.fault = eip_load_le16(b + 4);
    s.crank = eip_load_le16(b + 10);
    s.angle = static_cast<int32_t>(eip_load_le32(b + 24));
    s.rev = static_cast<int32_t>(eip_load_le32(b + 28));
    s.rpm = static_cast<int32_t>(eip_load_le32(b + 32));
    s.commloss = eip_load_le32(b + 76);
    s.counts = static_cast<int32_t>(eip_load_le32(b + 96));
    s.axisFlags = eip_load_le16(b + 96 + 8);
    return s;
}

bool OpenSession(EipSession *s, const char *ip) {
    EipOpenConfig open;
    memset(&open, 0, sizeof(open));
    open.ip = ip;
    open.slot = -1;
    open.timeout_ms = 3000;
    return eip_session_open(s, &open) == EIP_OK;
}

bool ClearOutput(const char *plc) { return bench::PlcClearOutput(plc); }

/** The PLC in place of a Class 1 connection of our own: writes ECam_O when it changes, reads ECam_I each cycle. */
class Bridge {
public:
    void Start(const char *plc) {
        m_plc = plc;
        m_thread = std::thread([this] { Run(); });
    }
    void Stop() {
        m_stop = true;
        if (m_thread.joinable()) {
            m_thread.join();
        }
    }
    void SetFlags(uint16_t f) {
        std::lock_guard<std::mutex> g(m_lock);
        eip_store_le16(m_cmd, f);
    }
    uint16_t Flags() {
        std::lock_guard<std::mutex> g(m_lock);
        return eip_load_le16(m_cmd);
    }
    void SetValue(size_t off, int32_t v) {
        std::lock_guard<std::mutex> g(m_lock);
        eip_store_le32(m_cmd + off, static_cast<uint32_t>(v));
    }
    Status Get() {
        std::lock_guard<std::mutex> g(m_lock);
        return m_have ? Decode(m_sts) : Status();
    }
    uint64_t Reads() const { return m_reads; }
    uint64_t Errors() const { return m_errors; }
    uint64_t Faults() const { return m_faults; }
    void Pulse(uint16_t bit) {  // 100 ms each way: over two PLC RPIs, and two of this bridge's cycles
        SetFlags(Flags() | bit);
        Wait(100);
        SetFlags(Flags() & static_cast<uint16_t>(~bit));
        Wait(100);
    }

private:
    void Run() {
        static EipSession s;
        bool open = false, sentAny = false;
        uint8_t sent[kCmdSize] = {};
        while (!m_stop) {
            if (!open) {
                open = OpenSession(&s, m_plc);
                if (!open) {
                    m_errors++;
                    Sleep(200);
                    continue;
                }
            }
            uint8_t out[kCmdSize], in[kStsSize];
            {
                std::lock_guard<std::mutex> g(m_lock);
                memcpy(out, m_cmd, sizeof(out));
            }
            int rc = EIP_OK;
            if (!sentAny || memcmp(out, sent, sizeof(out)) != 0) {
                rc = eip_tag_write(&s, kOutTag, EIP_SINT, kCmdSize, out, kCmdSize, nullptr);
                if (rc == EIP_OK) {
                    memcpy(sent, out, sizeof(sent));
                    sentAny = true;
                }
            }
            uint16_t len = sizeof(in);
            if (rc == EIP_OK) {
                rc = eip_tag_read(&s, kInTag, kStsSize, in, &len, nullptr, nullptr);
            }
            if (rc != EIP_OK || len != kStsSize) {
                m_errors++;
                eip_session_close(&s);
                open = false;
                Sleep(50);
                continue;
            }
            const Status st = Decode(in);
            if ((st.flags & kFlagFaulted) || (st.axisFlags & kAxisFault)) {
                m_faults++;
            }
            {
                std::lock_guard<std::mutex> g(m_lock);
                memcpy(m_sts, in, sizeof(m_sts));
                m_have = true;
            }
            m_reads++;
            Sleep(50);
        }
        if (open) {
            eip_session_close(&s);
        }
    }

    const char *m_plc = nullptr;
    std::thread m_thread;
    std::mutex m_lock;
    uint8_t m_cmd[kCmdSize] = {};
    uint8_t m_sts[kStsSize] = {};
    bool m_have = false;
    std::atomic<bool> m_stop{false};
    std::atomic<uint64_t> m_reads{0}, m_errors{0}, m_faults{0};
};

template <typename Pred>
Status WaitStatus(Bridge &b, Pred pred, double seconds) {
    const double end = Seconds() + seconds;
    while (Seconds() < end && !g_abort) {
        const Status s = b.Get();
        if (s.valid && pred(s)) {
            return s;
        }
        Sleep(10);
    }
    return b.Get();
}

// ---------------------------------------------------------------------------------------------------------------------

void Print(const std::vector<std::string> &lines) {
    for (size_t i = 0; i + 1 < lines.size(); i++) {
        printf("    %s\n", lines[i].c_str());
    }
}

void Upload(Console &c, const std::string &text) {
    c.Cmd("load");
    std::istringstream in(text);
    std::string l;
    while (std::getline(in, l)) {
        if (!l.empty() && l.back() == '\r') {
            l.pop_back();
        }
        if (l.find_first_not_of(" \t") == std::string::npos) {
            continue;
        }
        const auto r = c.Cmd(l);
        if (r.back().rfind("ok", 0) != 0) {
            throw std::runtime_error("upload: '" + l + "' -> " + r.back());
        }
    }
    const auto r = c.Cmd("commit", 10);
    if (r.back().rfind("ok", 0) != 0) {
        throw std::runtime_error("commit -> " + r.back());
    }
    printf("    %s\n", r.back().c_str());
}

// ---------------------------------------------------------------------------------------------------------------------
// The drives, each over its own USB: native tools from the main project
// ---------------------------------------------------------------------------------------------------------------------

struct Drive {
    const char *name;
    const char *serial;
    std::string port;
    double lo = 0, hi = 0;
    bool speedOk = false;
};

/** Finds each motor by serial. */
bool FindDrives(const std::string &tools, Drive d[2]) {
    for (const char *port : {"COM12", "COM13"}) {
        const std::string o = Exec(tools + "\\pos_hw.exe " + port, 15000);
        printf("    %s: %s", port, o.empty() ? "no output\n" : o.c_str());
        for (int i = 0; i < 2; i++) {
            if (o.find(std::string("serial ") + d[i].serial) != std::string::npos) {
                d[i].port = port;
            }
        }
    }
    return !d[0].port.empty() && !d[1].port.empty();
}

long long Commanded(const std::string &tools, const Drive &d) {
    const std::string o = Exec(tools + "\\pos_hw.exe " + d.port, 15000);
    if (o.find(std::string("serial ") + d.serial) == std::string::npos || o.find("commanded ") == std::string::npos) {
        throw std::runtime_error(std::string("pos_hw ") + d.name + ": " + o);
    }
    return IntAfter(o, "commanded ");
}

void ReadSpeed(const std::string &tools, Drive &d, int seconds) {
    const std::string o = Exec(tools + "\\scope_hw.exe " + d.port + " " + std::to_string(seconds) + " 2", (seconds + 30) * 1000);
    const char *key = "Velocity, measured: ";
    const size_t at = o.find(key);
    d.speedOk = at != std::string::npos && sscanf(o.c_str() + at + strlen(key), "%lf .. %lf", &d.lo, &d.hi) == 2;
}

// ---------------------------------------------------------------------------------------------------------------------

int64_t Wrap24(int64_t d) {
    d = (d + (1 << 23)) % (1 << 24);
    if (d < 0) {
        d += 1 << 24;
    }
    return d - (1 << 23);
}

/** floor(a * b / c) for c > 0, through 128 bits: the firmware's gear arithmetic overflows 64. */
int64_t MulDivFloor(int64_t a, int64_t b, int64_t c) {
    int64_t hi = 0;
    const int64_t lo = _mul128(a, b, &hi);
    int64_t rem = 0;
    const int64_t q = _div128(hi, lo, c, &rem);
    return rem < 0 ? q - 1 : q;
}

/** CamRuntime::Build's scaling. */
void GearPq(int64_t *p, int64_t *q) {
    double c = kCpu * kNum, d = static_cast<double>(kDen) * kCpr;
    const double limit = 274877906944.0;
    while (c != std::floor(c) && std::fabs(c) * 2.0 < limit && d * 2.0 < 4.0e18) {
        c *= 2.0;
        d *= 2.0;
    }
    *p = std::llround(c);
    *q = std::llround(d);
}

std::string Profile(int ccio) {
    char t[512];
    snprintf(t, sizeof(t),
             "master src=driven cpr=%d den=1 rpm=40 crank=0 ramp=120 max=150\n"
             "timeline repeat=1 ovr=1 trig=cmd in=0 rise=1 bit=0\n"
             "axis 0 master=crank m=1 cpu=%.1f start=0 closed=0 span=360 units=rev gear=%d:%d name=Gear\n"
             "seg 0 360 %.6f linear\n",
             kCpr, kCpu, kNum, kDen, static_cast<double>(kNum) / kDen);
    return ccio > 0 ? t + std::string("ccio_boards ") + std::to_string(ccio) + "\n" : t;  // none: the link stays closed
}

struct EipCounters {
    long long v[7] = {};
    void Read(const std::string &text) {
        const char *keys[7] = {"opens=", "timeouts=", "drop_seq=", "drop_src=", "drop_bad=", "udp_dropped=", "comm_losses="};
        for (int i = 0; i < 7; i++) {
            v[i] = IntAfter(text, keys[i]);
        }
    }
};

std::string Join(const std::vector<std::string> &lines) {
    std::string s;
    for (const std::string &l : lines) {
        s += l + " ";
    }
    return s;
}

struct Options {
    const char *board = nullptr, *plc = nullptr;
    double minutes = 0;
    std::string restore, tools = "ProjectData/build/Release";
    int telMs = 0, ccio = 0;
    bool ping = false;
};

void Run(const Options &o, Console &con, Bridge &lk, Results &r) {
    Drive drive[2] = {{"M-0", kSerialM0, ""}, {"M-1", kSerialM1, ""}};
    if (!FindDrives(o.tools, drive)) {
        throw std::runtime_error("both motors must be on USB, and " + o.tools + " must hold pos_hw and scope_hw");
    }
    printf("    M-0 on %s, M-1 on %s\n", drive[0].port.c_str(), drive[1].port.c_str());
    const int period = o.minutes >= 10 ? 60 : 20;

    for (const char *c : {"abort", "disable", "sim 0", "clear"}) {
        con.Cmd(c);
    }
    Upload(con, Profile(o.ccio));
    int64_t p = 0, q = 0;
    GearPq(&p, &q);
    printf("    gearP %lld gearQ %lld\n", p, q);

    lk.Start(o.plc);
    WaitStatus(lk, [](const Status &) { return true; }, 3);
    if (!lk.Get().valid) {
        throw std::runtime_error("no status from the PLC");
    }
    Wait(300);
    lk.SetFlags(kControl);
    Wait(150);
    lk.SetFlags(kControl | kEnable);
    Status s = WaitStatus(lk, [](const Status &x) { return (x.flags & kFlagEnabled) != 0; }, 5);
    r.Check("enabled under the PLC, simulate off", (s.flags & kFlagEnabled) != 0);
    Wait(200);

    const Status st0 = lk.Get();
    const long long d0[2] = {Commanded(o.tools, drive[0]), Commanded(o.tools, drive[1])};
    lk.Pulse(kStart);
    lk.SetValue(kRpmOffset, kRpm * 100);
    auto atSpeed = [](const Status &x) { return x.crank == kCrankRunning && std::abs(x.rpm - kRpm * 100) < 5; };
    s = WaitStatus(lk, atSpeed, 8);
    r.Check("crank at " + std::to_string(kRpm) + " rpm", atSpeed(s), std::to_string(s.rpm / 100.0));

    const auto diag0 = con.Cmd("diag");
    Print(diag0);
    const std::string samp0 = Find(diag0, "sample");
    const long long late0 = IntAfter(samp0, "late samples "), carry0 = IntAfter(samp0, "carried-step samples ");
    EipCounters eip0;
    eip0.Read(Join(con.Cmd("eip")));
    if (o.telMs) {
        con.Send("tel " + std::to_string(o.telMs) + "\n");
    }

    eip_sock_t udp = EIP_SOCK_INVALID;
    uint8_t boardIp[4];
    inet_pton(AF_INET, o.board, boardIp);
    if (o.ping) {
        udp = eip_hal_udp_open(0, 0);
    }

    const double t0 = Seconds();
    double nextDiag = t0 + period, lastAt = t0, worstRx = 1e9, worstTel = 1e9, rxRate = 0, telRate = 0;
    uint64_t lastRx = lk.Reads(), lastTel = con.TelLines(), looks = 0, offSpeed = 0, pings = 0, pongs = 0;
    long long worstLoop = 0, worstPeak = 0;
    int windows = 0;
    bool speedsOk[2] = {true, true};
    int speedLooks = 0;
    std::string speedText[2];
    while (Seconds() - t0 < o.minutes * 60 && !g_abort) {
        if (o.ping) {
            const char *ask = "ECAM_DISCOVER?";
            uint8_t buf[256];
            pings++;
            eip_hal_udp_sendto(udp, boardIp, kDiscoveryPort, reinterpret_cast<const uint8_t *>(ask), 14);
            pongs += eip_hal_udp_recvfrom(udp, buf, sizeof(buf), nullptr, nullptr, 500) > 0;
        }
        if (o.telMs) {
            con.Drain(500);
        } else {
            Wait(500);
        }
        const double now = Seconds();
        const Status st = lk.Get();
        looks++;
        offSpeed += !atSpeed(st);
        if (now - lastAt >= 10) {
            rxRate = (lk.Reads() - lastRx) / (now - lastAt);
            telRate = (con.TelLines() - lastTel) / (now - lastAt);
            worstRx = rxRate < worstRx ? rxRate : worstRx;
            worstTel = telRate < worstTel ? telRate : worstTel;
            lastRx = lk.Reads();
            lastTel = con.TelLines();
            lastAt = now;
        }
        if (now >= nextDiag) {
            nextDiag += period;
            const auto d = con.Cmd("diag");
            const std::string samp = Find(d, "sample"), loop = Find(d, "main loop");
            const long long longest = IntAfter(loop, "longest "), peak = IntAfter(samp, "max ");
            worstLoop = longest > worstLoop ? longest : worstLoop;
            worstPeak = peak > worstPeak ? peak : worstPeak;
            windows++;
            std::thread a([&] { ReadSpeed(o.tools, drive[0], 4); });
            std::thread b([&] { ReadSpeed(o.tools, drive[1], 4); });
            a.join();
            b.join();
            speedLooks++;
            for (int i = 0; i < 2; i++) {
                const double want = i ? kWant1 : kWant0;
                const double got = std::fmax(std::fabs(drive[i].lo), std::fabs(drive[i].hi));
                if (!drive[i].speedOk || std::fabs(got - want) >= 0.03 * want) {
                    speedsOk[i] = false;
                    speedText[i] = "last bad " + std::to_string(drive[i].lo) + " .. " + std::to_string(drive[i].hi);
                }
            }
            printf("  %5.1f min  PLC reads %.1f/s  M-0 %.1f..%.1f  M-1 %.1f..%.1f  loop %lld us  callback %lld us\n",
                   (now - t0) / 60, rxRate, drive[0].lo, drive[0].hi, drive[1].lo, drive[1].hi, longest, peak);
            fflush(stdout);
        }
    }
    const bool aborted = g_abort;
    const Status running = lk.Get();
    if (udp != EIP_SOCK_INVALID) {
        eip_hal_udp_close(udp);
    }
    g_abort = false;  // stop the machine properly whatever ended the loop
    lk.Pulse(kStop);
    WaitStatus(lk, [](const Status &x) { return x.crank == kCrankOff; }, 8);
    Wait(300);
    s = lk.Get();
    const int64_t units = static_cast<int64_t>(s.rev) * kCpr + std::llround(s.angle / 90.0);
    const int64_t want = MulDivFloor(units, p, q);
    const int64_t du = units - (static_cast<int64_t>(st0.rev) * kCpr + std::llround(st0.angle / 90.0));
    const int64_t dm = static_cast<int64_t>(s.counts) - st0.counts;
    const long long d1[2] = {Commanded(o.tools, drive[0]), Commanded(o.tools, drive[1])};
    const int64_t e0 = Wrap24(d1[0] - d0[0] + 8 * du), e1 = Wrap24(d1[1] - d0[1] - dm);
    char text[256];
    snprintf(text, sizeof(text), "M-0 %lld units, drive off by %lld; M-1 %lld counts, drive off by %lld", du, e0, dm, e1);
    r.Check("the drives received exactly what the board counted", std::llabs(e0) <= 8 && std::llabs(e1) <= 1, text);

    if (o.telMs) {
        con.Cmd("tel 0");
    }
    const auto diag1 = con.Cmd("diag");
    Print(diag1);
    const auto eipLines = con.Cmd("eip");
    Print(eipLines);
    EipCounters eip1;
    eip1.Read(Join(eipLines));
    const std::string samp1 = Find(diag1, "sample"), lw = Find(diag1, "lwip");
    const long long late1 = IntAfter(samp1, "late samples "), carry1 = IntAfter(samp1, "carried-step samples ");

    r.Check("the run went its full length", !aborted, aborted ? "stopped by hand" : "");
    snprintf(text, sizeof(text), "worst %.1f/s, read errors %llu", worstRx, (unsigned long long)lk.Errors());
    r.Check("the PLC tag bridge kept reading (10 s windows)", worstRx > 5, text);
    bool clean = true;
    std::string counters;
    const char *names[7] = {"opens", "timeouts", "drop_seq", "drop_src", "drop_bad", "udp_dropped", "comm_losses"};
    for (int i = 0; i < 7; i++) {
        clean = clean && eip1.v[i] == eip0.v[i] && eip1.v[i] >= 0;
        counters += std::string(names[i]) + " +" + std::to_string(eip1.v[i] - eip0.v[i]) + " ";
    }
    r.Check("board: the PLC connection never dropped or lost a packet", clean, counters);
    if (o.telMs) {
        r.Check("telemetry kept up", worstTel > 950.0 / o.telMs, std::to_string(worstTel) + "/s");
    }
    if (o.ping) {
        r.Check("every discovery ping answered", pongs == pings, std::to_string(pongs) + "/" + std::to_string(pings));
    }
    snprintf(text, sizeof(text), "off speed %llu of %llu looks, commloss %lu -> %lu", (unsigned long long)offSpeed,
             (unsigned long long)looks, (unsigned long)st0.commloss, (unsigned long)running.commloss);
    r.Check("no connection loss, PLC in control, crank at speed at every look",
            running.commloss == st0.commloss && (running.flags & kFlagPlc) && offSpeed == 0 && looks > 0, text);
    r.Check("no fault on the run", lk.Faults() == 0, std::to_string(lk.Faults()) + " status reads faulted");
    snprintf(text, sizeof(text), "late %lld -> %lld, carried %lld -> %lld, peak callback %lld us", late0, late1, carry0,
             carry1, worstPeak);
    r.Check("no new late samples or carried steps; the callback well inside its 200 us",
            late1 == late0 && carry1 == carry0 && worstPeak < 100 && windows > 0, text);
    r.Check("the main loop's longest pass stays short", windows > 0 && worstLoop < 5000,
            std::to_string(worstLoop) + " us over " + std::to_string(windows) + " windows");
    r.Check("lwIP: no allocation failures",
            lw.find("errors 0") != std::string::npos && lw.find("pool errors 0") != std::string::npos, lw);
    r.Check("M-0 measured " + std::to_string(static_cast<int>(kWant0)) + " rpm at every look", speedLooks > 0 && speedsOk[0], speedText[0]);
    snprintf(text, sizeof(text), "M-1 measured %.1f rpm at every look", kWant1);
    r.Check(text, speedLooks > 0 && speedsOk[1], speedText[1]);
    snprintf(text, sizeof(text), "M-1 %d counts, %lld crank units x P/Q = %lld", s.counts, units, want);
    r.Check("gear exact at rest after " + std::to_string(s.rev) + " crank revolutions", std::llabs(s.counts - want) <= 2, text);

    lk.SetFlags(kControl);
    WaitStatus(lk, [](const Status &x) { return !(x.flags & kFlagEnabled); }, 3);
    lk.SetFlags(0);
    Wait(150);
}

}  // namespace

int PlcClear(int argc, char **argv) {
    if (argc < 3) {
        printf("plcclear needs the PLC's IP address\n");
        return 2;
    }
    const bool ok = ClearOutput(argv[2]);
    printf("%s\n", ok ? "ECam_O is clear" : "ECam_O NOT confirmed clear");
    return ok ? 0 : 1;
}

int Soak(int argc, char **argv) {
    Options o;
    std::vector<std::string> plain;
    for (int i = 2; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--tel" && i + 1 < argc) o.telMs = atoi(argv[++i]);
        else if (a == "--ping") o.ping = true;
        else if (a == "--ccio" && i + 1 < argc) o.ccio = atoi(argv[++i]);
        else if (a == "--tools" && i + 1 < argc) o.tools = argv[++i];
        else plain.push_back(a);
    }
    if (plain.size() < 4) {
        printf("usage: ecam_bench soak BOARD_IP PLC_IP MINUTES RESTORE_PROFILE [--tel MS] [--ping] [--tools DIR]\n");
        return 2;
    }
    char full[MAX_PATH];
    if (GetFullPathNameA(o.tools.c_str(), sizeof(full), full, nullptr)) {  // converts / as well
        o.tools = full;
    }
    o.board = plain[0].c_str();
    o.plc = plain[1].c_str();
    o.minutes = atof(plain[2].c_str());
    std::ifstream in(plain[3], std::ios::binary);
    if (!in) {
        printf("cannot read %s\n", plain[3].c_str());
        return 2;
    }
    std::stringstream text;
    text << in.rdbuf();
    o.restore = text.str();
    SetConsoleCtrlHandler(OnCtrl, TRUE);
    printf("soak %.1f min, telemetry %s, pings %s\n", o.minutes, o.telMs ? std::to_string(o.telMs).c_str() : "off",
           o.ping ? "on" : "off");

    Results r;
    Console con;
    Bridge lk;
    int rc = 1;
    try {
        con.Open(o.board);
        Run(o, con, lk, r);
        rc = r.passed == r.total && r.total > 0 ? 0 : 1;
    } catch (const std::exception &x) {
        printf("STOPPED: %s\n", x.what());
    }
    // Every way out: the PLC first, because it goes on holding what was written last.
    lk.Stop();
    ClearOutput(o.plc);
    try {
        con.Open(o.board);
        for (const char *c : {"tel 0", "abort", "disable", "sim 0", "clear"}) {
            con.Cmd(c);
        }
        Upload(con, o.restore);
        printf("    %s\n", Find(con.Cmd("status"), "state=").c_str());
    } catch (const std::exception &x) {
        printf("WARNING: could not restore the board: %s\n", x.what());
        rc = 1;
    }
    printf("\n%d/%d passed\n", r.passed, r.total);
    fflush(stdout);
    return rc;
}
