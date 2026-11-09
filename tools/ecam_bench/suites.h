// MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
// What the test suites share: the status assembly, the command flags, and a few helpers over Link and Console.
#ifndef ECAM_BENCH_SUITES_H_
#define ECAM_BENCH_SUITES_H_

#include <cstdint>
#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "bench_common.h"
#include "bench_link.h"

namespace bench {

constexpr const char *kFwVersion = "0.11.4";
constexpr int kFwMajor = 0, kFwMinor = 11;
constexpr int kEipRevMajor = 1, kEipRevMinor = kFwMajor * 100 + kFwMinor;

// ControlFlags
constexpr uint16_t C_ENABLE = 1 << 0, C_CONTROL = 1 << 1, C_START = 1 << 2, C_STOP = 1 << 3, C_ABORT = 1 << 4,
                   C_CLEAR = 1 << 5, C_SIM = 1 << 6, C_JOG = 1 << 7, C_TRIGGER = 1 << 8, C_HOME = 1 << 9;
// StatusFlags
constexpr uint16_t F_LOADED = 1 << 0, F_STORED = 1 << 1, F_ENABLED = 1 << 2, F_READY = 1 << 3, F_RUNNING = 1 << 4,
                   F_ARMED = 1 << 5, F_DONE = 1 << 6, F_STOPPING = 1 << 7, F_JOGGING = 1 << 8, F_FAULTED = 1 << 9,
                   F_ESTOP = 1 << 10, F_SIM = 1 << 11, F_PLC = 1 << 12, F_REJECTED = 1 << 13;

struct AxisStatus {
    int32_t counts = 0, travel = 0;
    uint16_t flags = 0, conn = 0;
};

struct Sts {
    bool valid = false, reservedZero = false;
    uint16_t fw = 0, flags = 0, fault = 0, reject = 0, state = 0, crank = 0, tl = 0, outputs = 0, axes = 0, inputs = 0;
    uint32_t hash = 0, heartbeat = 0, commloss = 0;
    uint16_t waitIndex = 0;  // 0.10.0: the timeline wait holding, 1-based
    uint32_t waitedMs = 0;
    int32_t angle = 0, rev = 0, rpm = 0, tUs = 0, cycle = 0, ovr = 0, enc = 0, len = 0, next = 0, lmin = 0, gap = 0, sync = 0;
    AxisStatus axis[4];
};

Sts DecodeStatus(const Bytes &b);
Sts CurrentStatus(Link &l);
Sts WaitStatus(Link &l, const std::function<bool(const Sts &)> &pred, double timeout = 5.0);
std::string FormatStatus(const Sts &s);
const char *Name(const char *const *names, size_t count, unsigned v);
extern const char *const kEngineStates[7];
extern const char *const kCrankStates[5];
extern const char *const kTlStates[6];

/** A rising edge: set the bit, hold it a few packets, clear it. */
void Pulse(Link &l, uint16_t bit, int holdMs = 100);
void TakeControl(Link &l, uint16_t extra = 0);

struct Net {
    uint32_t capability = 0, control = 0;
    bool settable = false;
    std::string ip, netmask, gateway, dns1, dns2, domain, host;
};
Net TcpIpRead(Link &l);

/** The console's status line as a map. */
std::map<std::string, std::string> ConsoleStatus(Console &c);
/** load, the lines, commit. The commit's ok line. */
std::string Upload(Console &c, const std::string &text);
std::string ReadFile(const std::string &path);

/** The suites' final line. True if all passed. */
bool Summary(const Results &r);
/** Clears the PLC given to --plc. */
void ClearPlc();
void SetPlc(const std::string &ip);

extern const char *const kBenchDriven;
extern const char *const kBenchTimeline;

/** Polls the console's status; the values of key seen, in order, without repeats. */
std::vector<std::string> WatchConsole(Console &c, double seconds, const char *key);
/** A connection at the PLC's 20 ms, open and settled, its flags set from the first packet. */
std::unique_ptr<Link> FreshLink(const std::string &ip, uint16_t flags = 0);
inline int64_t CrankPos(const Sts &s) { return static_cast<int64_t>(s.rev) * 360000 + s.angle; }

/** Runs a suite; whatever happens, leaves the board stopped and as it was. */
template <typename F>
int RunSuite(const std::string &ip, const std::string &restore, F body) {
    Results r;
    Console con;
    try {
        con.Open(ip.c_str());
        body(con, r);
    } catch (const std::exception &x) {
        r.Check("the suite ran to the end", false, x.what());
    }
    if (UsingPlc()) {
        ClearPlc();
    }
    try {  // whatever happened, leave the board stopped and as it was
        con.Open(ip.c_str());
        for (const char *c : {"tel 0", "abort", "sim 0", "disable", "clear"}) {
            con.Cmd(c);
        }
        con.Cmd("estop_test 0");
        if (!restore.empty()) {
            printf("== restore the board's profile\n    %s\n", Upload(con, ReadFile(restore)).c_str());
        }
    } catch (const std::exception &x) {
        printf("WARNING: could not restore the board: %s\n", x.what());
    }
    return Summary(r) ? 0 : 1;
}


int SuiteTest(const std::string &ip);
int SuiteStatus(const std::string &ip, const std::string &restore, const std::string &workbench, const std::string &project);
int SuiteCmd(const std::string &ip, const std::string &restore);
int SuiteMotion(const std::string &ip, const std::string &restore);
int SuiteLoad(const std::string &ip, const std::string &restore, double minutes);
int SuiteLive(const std::string &ip, const std::string &restore, const std::string &tools);
int SuitePlcLoss(const std::string &ip, const std::string &restore, double waitSeconds);
int SuiteEds(const std::string &ip, const std::string &path);
int SuiteUi(const std::string &ip, const std::string &exe, const std::string &project);
int NetTool(const std::string &mode, const std::string &ip, const std::vector<std::string> &args);

}  // namespace bench

#endif  // ECAM_BENCH_SUITES_H_
