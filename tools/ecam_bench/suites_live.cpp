// MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
// The PLC's whole job on the real motors, simulate off: each motor's own encoder, over its own USB, says what it did.
// Peak 720 rpm; M-1 moves at most 2 motor revs from where it starts.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <map>
#include <thread>

#include "suites.h"

namespace bench {

namespace {

// Driven crank = M-0 itself (4000 counts per crank rev on an 800-step motor: motor rpm = 5 x crank rpm);
// M-1 cams 1 rev out over 135 deg and back (cycloidal, peak motor rpm 16/3 x crank rpm).
const char *const kLiveCrank =
    "master src=driven cpr=4000 den=1 rpm=40 crank=0 ramp=120 max=150\n"
    "timeline repeat=1 ovr=1 trig=cmd in=0 rise=1 bit=0\n"
    "axis 0 master=crank m=1 cpu=800 start=0 closed=1 span=360 units=rev name=Screw\n"
    "seg 0 90 0 dwell\n"
    "seg 0 225 1 cycloidal\n"
    "seg 0 360 0 cycloidal\n";

// A one-shot 1 s timeline started by the TRIGGER bit; M-0 3 revs (peak 720 rpm), M-1 2 revs (peak 480 rpm).
const char *const kLiveTrigger =
    "master src=encoder cpr=4000 den=1 rpm=20 crank=0 ramp=60 max=600\n"
    "timeline repeat=0 ovr=1 trig=eip in=0 rise=1 bit=0\n"
    "axis 0 master=timeline m=0 cpu=800 start=0 closed=1 span=1 units=rev name=Rotary\n"
    "seg 0 0.5 3 cycloidal\n"
    "seg 0 1 0 cycloidal\n"
    "axis 1 master=timeline m=1 cpu=800 start=0 closed=1 span=1 units=rev name=Screw\n"
    "seg 1 0.5 2 cycloidal\n"
    "seg 1 1 0 cycloidal\n";

constexpr double kPi = 3.14159265358979323846;

/** A closed curve through 24 points on M-1, all Spline: 1 - cos, half a revolution each way. */
std::string SplineText() {
    std::string t =
        "master src=driven cpr=4000 den=1 rpm=40 crank=0 ramp=120 max=150\n"
        "timeline repeat=1 ovr=1 trig=cmd in=0 rise=1 bit=0\n"
        "axis 0 master=crank m=1 cpu=800 start=0 closed=1 span=360 units=rev name=Curve\n";
    for (int k = 1; k <= 24; k++) {
        char l[64];
        snprintf(l, sizeof(l), "seg 0 %d %.12g spline\n", 15 * k, k < 24 ? 0.5 * (1 - std::cos(2 * kPi * k / 24)) : 0.0);
        t += l;
    }
    return t;
}

/** An electronic gear on M-1, num revolutions for every 2 of the crank, the crank on M-0 (5 x crank rpm). */
std::string GearText(int num) {
    char t[320];
    snprintf(t, sizeof(t),
             "master src=driven cpr=4000 den=1 rpm=40 crank=0 ramp=120 max=150\n"
             "timeline repeat=1 ovr=1 trig=cmd in=0 rise=1 bit=0\n"
             "axis 0 master=crank m=1 cpu=800 start=0 closed=0 span=360 units=rev gear=%d:2 name=Gear\n"
             "seg 0 360 %g linear\n",
             num, num / 2.0);
    return t;
}

struct Reading {
    bool ok = false;
    double lo = 0, hi = 0;
    int enable = 0, hlfb = 0;
    double Peak() const { return std::max(std::fabs(lo), std::fabs(hi)); }
    std::string Text() const {
        if (!ok) {
            return "no reading";
        }
        char t[80];
        snprintf(t, sizeof(t), "%.1f .. %.1f rpm, enable %d%%, HLFB %d%%", lo, hi, enable, hlfb);
        return t;
    }
};

/** Each motor's measured velocity from its own encoder (scope_hw), independent of the ClearCore. Never moves a motor. */
class MotorScope {
public:
    explicit MotorScope(const std::string &tools) : m_tools(tools) {
        for (const char *port : {"COM12", "COM13"}) {
            const std::string o = Exec(Tool(port, 0.3), 30000);
            if (o.find("serial 65502511") != std::string::npos) m_port[0] = port;
            if (o.find("serial 59401211") != std::string::npos) m_port[1] = port;
        }
    }
    bool Found() const { return !m_port[0].empty() && !m_port[1].empty(); }
    std::string Ports() const { return "M-0 " + m_port[0] + ", M-1 " + m_port[1]; }
    struct Pair {
        Reading m[2];
    };
    Pair Read(double seconds) const {
        Pair p;
        std::thread a([&] { p.m[0] = One(m_port[0], seconds); });
        std::thread b([&] { p.m[1] = One(m_port[1], seconds); });
        a.join();
        b.join();
        return p;
    }
    std::future<Pair> Start(double seconds) const {
        return std::async(std::launch::async, [this, seconds] { return Read(seconds); });
    }

private:
    std::string Tool(const std::string &port, double seconds) const {
        char t[32];
        snprintf(t, sizeof(t), " %.1f 2", seconds);
        return m_tools + "\\scope_hw.exe " + port + t;
    }
    Reading One(const std::string &port, double seconds) const {
        const std::string o = Exec(Tool(port, seconds), static_cast<DWORD>((seconds + 30) * 1000));
        Reading r;
        const char *vk = "Velocity, measured: ", *hk = "HLFB bit high in ";
        const size_t v = o.find(vk), h = o.find(hk);
        if (v != std::string::npos && h != std::string::npos &&
            sscanf(o.c_str() + v + strlen(vk), "%lf .. %lf", &r.lo, &r.hi) == 2 &&
            sscanf(o.c_str() + h + strlen(hk), "%d%%, enable bit high in %d%%", &r.hlfb, &r.enable) == 2) {
            r.ok = true;
        }
        return r;
    }
    std::string m_tools, m_port[2];
};

/** Every status sample in the background: faults, axis faults, the commanded counts' range per axis. */
class StatusWatch {
public:
    StatusWatch(Link &l, int axes) : m_link(l), m_axes(axes), m_thread([this] { Run(); }) {}
    ~StatusWatch() { End(); }
    void End() {
        m_done = true;
        if (m_thread.joinable()) {
            m_thread.join();
        }
    }
    int lo[2] = {1 << 30, 1 << 30}, hi[2] = {-(1 << 30), -(1 << 30)};
    int faults = 0, samples = 0;

private:
    void Run() {
        while (!m_done) {
            const Sts s = CurrentStatus(m_link);
            if (s.valid) {
                samples++;
                faults += (s.flags & F_FAULTED) != 0;
                for (int i = 0; i < m_axes; i++) {
                    lo[i] = std::min(lo[i], s.axis[i].counts);
                    hi[i] = std::max(hi[i], s.axis[i].counts);
                    faults += (s.axis[i].flags & 16) != 0;
                }
            }
            Sleep(5);
        }
    }
    Link &m_link;
    int m_axes;
    std::atomic<bool> m_done{false};
    std::thread m_thread;
};

std::vector<std::string> LiveMotors(Console &c) {
    std::vector<std::string> out;
    for (const std::string &l : c.Cmd("motors")) {
        if (l.rfind("M-0", 0) == 0 || l.rfind("M-1", 0) == 0) {
            out.push_back(l);
        }
    }
    return out;
}

std::string Num(double v, const char *f = "%.2f") {
    char t[48];
    snprintf(t, sizeof(t), f, v);
    return t;
}

std::function<bool(const Sts &)> Enabled() {
    return [](const Sts &x) { return (x.flags & F_ENABLED) != 0; };
}
std::function<bool(const Sts &)> Disabled() {
    return [](const Sts &x) { return !(x.flags & F_ENABLED); };
}
std::function<bool(const Sts &)> CrankAt(int rpm100) {
    return [rpm100](const Sts &x) { return x.crank == 2 && std::abs(x.rpm - rpm100) < 5; };
}
std::function<bool(const Sts &)> CrankOff() {
    return [](const Sts &x) { return x.crank == 0; };
}

void Release(Link &lk) {
    lk.SetFlags(C_CONTROL);
    WaitStatus(lk, Disabled(), 3);
    lk.SetFlags(0);
    Sleep(150);
    lk.Close();
    Sleep(300);
}

void EnableUnderPlc(Link &lk) {
    lk.SetFlags(C_CONTROL);
    Sleep(150);
    lk.SetFlags(C_CONTROL | C_ENABLE);
    WaitStatus(lk, Enabled(), 5);
    Sleep(200);
}

/** Run at 120 rpm (M-0 600 rpm), lose the connection: the stop, the settle, the disable. M-0's speed against its direction
 *  of travel is the blip every disable makes: up to 15.7 rpm on these motors, measured disabling from rest. The settle in ms. */
double LiveLoss(const std::string &ip, Console &con, const MotorScope &scope, Results &r, const std::string &label) {
    auto lk = FreshLink(ip);
    EnableUnderPlc(*lk);
    lk->SetValue(4, 12000);
    Pulse(*lk, C_START);
    WaitStatus(*lk, CrankAt(12000), 6);
    Sleep(500);
    const uint32_t cl0 = CurrentStatus(*lk).commloss;
    auto job = scope.Start(4);
    Sleep(2000);    // scope_hw connects and streams first
    lk->StopIo();   // the cable pulled: the board stops hearing the PLC
    std::vector<std::string> seen;
    double tOff = 0, tDis = 0;
    const double t0 = PreciseSeconds();
    while (PreciseSeconds() - t0 < 2.5) {
        auto c = ConsoleStatus(con);
        const double now = PreciseSeconds();
        if (seen.empty() || seen.back() != c["crank"]) {
            seen.push_back(c["crank"]);
        }
        if (tOff == 0 && c["crank"] == "off") tOff = now;
        if (tDis == 0 && c["enabled"] == "0") tDis = now;
    }
    const double gap = tOff != 0 && tDis != 0 ? (tDis - tOff) * 1000 : -1;
    const MotorScope::Pair p = job.get();
    auto c = ConsoleStatus(con);
    lk->Close();
    Sleep(300);
    auto rd = FreshLink(ip);
    Sts s = CurrentStatus(*rd);
    const Reading &m0 = p.m[0], &m1 = p.m[1];
    const double rebound = std::min(std::fabs(m0.lo), std::fabs(m0.hi));
    std::string path;
    for (const std::string &x : seen) {
        path += x + " ";
    }
    printf("    %s: crank %s; crank off to disabled %.0f ms; M-0 %s  M-1 %s\n", label.c_str(), path.c_str(), gap, m0.Text().c_str(), m1.Text().c_str());
    r.Check(label + ": controlled stop, then disabled, FaultCode 4, CommLossCount + 1",
            seen.back() == "off" && c["enabled"] == "0" && s.fault == 4 && s.commloss == cl0 + 1,
            "enabled=" + c["enabled"] + " fault " + std::to_string(s.fault) + " commloss " + std::to_string(cl0) + " -> " + std::to_string(s.commloss));
    r.Check(label + ": M-0 measured at 600 rpm, down to rest (under the disable's own 20 rpm), its enable dropped",
            m0.ok && m0.Peak() > 590 && m0.Peak() < 615 && rebound < 20 && m0.enable < 95, m0.Text());
    r.Check(label + ": M-1's enable dropped", m1.ok && m1.enable < 95, m1.Text());
    rd->SetFlags(C_CONTROL);
    Sleep(150);
    Pulse(*rd, C_CLEAR);
    s = WaitStatus(*rd, [](const Sts &x) { return x.fault == 0; }, 2);
    r.Check(label + ": CLEAR_FAULT from the PLC", s.fault == 0, "fault " + std::to_string(s.fault));
    rd->SetFlags(0);
    Sleep(150);
    rd->Close();
    Sleep(300);
    return gap;
}

int64_t FloorDiv(int64_t a, int64_t b) {
    const int64_t q = a / b;
    return (a % b != 0 && (a < 0) != (b < 0)) ? q - 1 : q;
}

void Live(const std::string &ip, Console &con, Results &r, const std::string &tools) {
    MotorScope scope(tools);
    printf("motors by serial: %s\n", scope.Ports().c_str());
    r.Check("both motors found on USB by serial number", scope.Found(), scope.Ports());
    if (!scope.Found()) {
        return;
    }
    for (const char *c : {"abort", "disable", "sim 0", "clear"}) {
        con.Cmd(c);
    }

    printf("== driven crank on M-0, cam on M-1; the PLC in control, simulate off\n    %s\n", Upload(con, kLiveCrank).c_str());
    auto lk = FreshLink(ip);
    lk->SetFlags(C_CONTROL);
    Sts s = WaitStatus(*lk, [](const Sts &x) { return (x.flags & F_PLC) != 0; }, 2);
    r.Check("PLC_CONTROL taken; SIMULATE off", (s.flags & F_PLC) && !(s.flags & F_SIM));
    lk->SetFlags(C_CONTROL | C_ENABLE);
    s = WaitStatus(*lk, [](const Sts &x) { return (x.flags & F_ENABLED) && (x.axis[0].flags & 8); }, 5);
    auto mot = LiveMotors(con);
    bool hlfb = mot.size() == 2;
    for (const std::string &m : mot) {
        hlfb = hlfb && m.find("hlfb=asserted") != std::string::npos;
    }
    r.Check("ENABLE from the PLC: both enabled, HLFB asserted on M-0 (crank) and M-1 (axis)", (s.flags & F_ENABLED) && hlfb);
    {
        StatusWatch w(*lk, 1);
        Pulse(*lk, C_START);
        s = WaitStatus(*lk, CrankAt(4000), 6);
        r.Check("START from the PLC: crank running at the profile's 40 rpm", CrankAt(4000)(s), Num(s.rpm / 100.0));
        for (int rpm : {40, 80, 120}) {
            lk->SetValue(4, rpm == 40 ? 0 : rpm * 100);
            WaitStatus(*lk, [rpm](const Sts &x) { return std::abs(x.rpm - rpm * 100) < 5; }, 5);
            Sleep(500);
            const Sts s0 = CurrentStatus(*lk);
            const double t0 = PreciseSeconds();
            const auto sc = scope.Read(4);
            const Sts s1 = CurrentStatus(*lk);
            const double t1 = PreciseSeconds();
            const double meas = static_cast<double>(CrankPos(s1) - CrankPos(s0)) / 360000 / ((t1 - t0) / 60);
            const std::string tag = "CrankRpm " + std::to_string(rpm * 100);
            r.Check(tag + ": the crank turns at " + std::to_string(rpm) + " rpm (status position over the PC clock)",
                    std::fabs(meas - rpm) < 0.03 * rpm && s1.reject == 0, Num(meas) + " rpm over " + Num(t1 - t0, "%.1f") + " s");
            const Reading &m0 = sc.m[0], &m1 = sc.m[1];
            const double want = 5.0 * rpm, peak = 16.0 * rpm / 3;
            r.Check(tag + ": M-0 measured turning steadily at " + Num(want, "%.0f") + " rpm",
                    m0.ok && std::fabs(m0.Peak() - want) < 0.05 * want && m0.hi - m0.lo < 0.15 * want && m0.enable > 95, m0.Text());
            r.Check(tag + ": M-1 measured camming out and back, peak " + Num(peak, "%.0f") + " rpm each way",
                    m1.ok && std::fabs(m1.hi - peak) < 0.05 * peak && std::fabs(-m1.lo - peak) < 0.05 * peak && m1.enable > 95, m1.Text());
        }
        Pulse(*lk, C_STOP);
        WaitStatus(*lk, CrankOff(), 8);
        Sleep(500);
        s = CurrentStatus(*lk);
        r.Check("STOP from the PLC: the crank comes to rest, still enabled", s.crank == 0 && (s.flags & F_ENABLED), "at " + Num(s.angle / 1000.0) + " deg");
        const auto sc = scope.Read(1);
        bool rest = true;
        for (const Reading &m : sc.m) {
            rest = rest && m.ok && std::fabs(m.lo) < 5 && std::fabs(m.hi) < 5 && m.enable > 95;
        }
        r.Check("STOP: both motors measured at rest, still enabled", rest, sc.m[0].Text() + " / " + sc.m[1].Text());

        printf("== JOG the crank to targets (M-0 turns to them, M-1 follows its cam)\n");
        const int64_t here = CrankPos(s);
        lk->SetValue(16, static_cast<int32_t>(here));
        Sleep(50);
        lk->SetFlags(C_CONTROL | C_ENABLE | C_JOG);
        WaitStatus(*lk, [](const Sts &x) { return x.state == 5; }, 2);
        const int64_t base = FloorDiv(here, 360000) * 360000 + 360000;
        const int targets[3][2] = {{180, 644}, {225, 800}, {360, 0}};
        for (const auto &t : targets) {
            const int64_t tgt = base + t[0] * 1000;
            lk->SetValue(16, static_cast<int32_t>(tgt));
            WaitStatus(*lk, [tgt](const Sts &x) { return std::llabs(CrankPos(x) - tgt) < 200; }, 15);
            Sleep(300);
            s = CurrentStatus(*lk);
            const int a0 = s.axis[0].counts;
            r.Check("JogCrankTarget +" + std::to_string(t[0]) + " deg: the crank (M-0) goes there, M-1 at its cam position",
                    std::llabs(CrankPos(s) - tgt) < 200 && std::abs(a0 - t[1]) < 3,
                    "crank " + Num((CrankPos(s) - base) / 1000.0) + " deg, M-1 " + std::to_string(a0) + " counts (cam " + std::to_string(t[1]) + ")");
        }
        lk->SetFlags(C_CONTROL | C_ENABLE);
        s = WaitStatus(*lk, [](const Sts &x) { return x.state != 5; }, 3);
        r.Check("JOG released: ready", s.state == 2, Name(kEngineStates, 7, s.state));
        w.End();
        r.Check("no fault and no axis fault in any status sample", w.faults == 0, std::to_string(w.samples) + " samples, " + std::to_string(w.faults) + " bad");
        r.Check("M-1 commanded through its whole cam, 0 .. 800 counts (1 rev)", w.lo[0] <= 5 && w.hi[0] >= 795, std::to_string(w.lo[0]) + " .. " + std::to_string(w.hi[0]));
    }
    Release(*lk);

    printf("== connection lost at 120 rpm (M-0 600 rpm): stop, settle on HLFB, disable; three times\n");
    std::vector<double> gaps;
    for (int i = 1; i <= 3; i++) {
        const double g = LiveLoss(ip, con, scope, r, "loss " + std::to_string(i));
        if (g >= 0) {
            gaps.push_back(g);
        }
    }
    std::string gtext;
    for (double g : gaps) {
        gtext += Num(g, "%.0f") + " ";
    }
    r.Check("the disable waits for HLFB, not for the whole 500 ms", gaps.size() == 3 && *std::max_element(gaps.begin(), gaps.end()) < 450, gtext + "ms");
    printf("== the same with hlfb_check 0: nothing says when, so the whole settle time\n");
    con.Cmd("set hlfb_check 0");
    double gap = -1;
    try {
        gap = LiveLoss(ip, con, scope, r, "loss, hlfb_check 0");
    } catch (...) {
        con.Cmd("set hlfb_check 1");
        throw;
    }
    con.Cmd("set hlfb_check 1");
    r.Check("hlfb_check 0: the motors hold for the whole 500 ms", gap > 450 && gap < 650, Num(gap, "%.0f") + " ms");

    printf("== Cubic spline: a closed curve through 24 points on M-1, the crank on M-0 at 120 rpm\n    %s\n", Upload(con, SplineText()).c_str());
    lk = FreshLink(ip);
    EnableUnderPlc(*lk);
    lk->SetValue(4, 12000);
    {
        StatusWatch w(*lk, 1);
        Pulse(*lk, C_START);
        WaitStatus(*lk, CrankAt(12000), 6);
        Sleep(500);
        const auto sc = scope.Read(4);
        const double peak = 120 * kPi;  // 0.5 (1 - cos th) rev: peak 0.5 rev per radian, at 120 rpm 120 x pi rpm
        const Reading &m0 = sc.m[0], &m1 = sc.m[1];
        printf("    measured: M-0 %s  M-1 %s\n", m0.Text().c_str(), m1.Text().c_str());
        r.Check("spline: M-0 measured turning steadily at 600 rpm", m0.ok && std::fabs(m0.Peak() - 600) < 30 && m0.hi - m0.lo < 90, m0.Text());
        r.Check("spline: M-1 measured on the curve, peak " + Num(peak, "%.0f") + " rpm each way",
                m1.ok && std::fabs(m1.hi - peak) < 0.05 * peak && std::fabs(-m1.lo - peak) < 0.05 * peak, m1.Text());
        Pulse(*lk, C_STOP);
        WaitStatus(*lk, CrankOff(), 8);
        w.End();
        r.Check("spline: M-1 commanded 0 .. 800 counts, no faults", w.lo[0] <= 5 && w.hi[0] >= 795 && w.hi[0] <= 805 && w.faults == 0,
                std::to_string(w.lo[0]) + " .. " + std::to_string(w.hi[0]));
    }
    Release(*lk);

    printf("== Electronic gear: M-1 at 3:2 of the crank on M-0, simulate off; then reversed\n");
    for (int num : {3, -3}) {
        printf("    %s\n", Upload(con, GearText(num)).c_str());
        const std::string prof = Find(con.Cmd("profile"), "a0");
        lk = FreshLink(ip);
        EnableUnderPlc(*lk);
        s = CurrentStatus(*lk);
        const std::string tag = "gear " + std::to_string(num) + ":2";
        r.Check(tag + ": profile and status name it", prof.find("electronic " + tag) != std::string::npos && (s.axis[0].flags & 64), prof);
        {
            StatusWatch w(*lk, 1);
            Pulse(*lk, C_START);
            for (int rpm : {40, 120}) {
                lk->SetValue(4, rpm == 40 ? 0 : rpm * 100);
                WaitStatus(*lk, CrankAt(rpm * 100), 6);
                Sleep(500);
                const auto sc = scope.Read(4);
                const Reading &m0 = sc.m[0], &m1 = sc.m[1];
                printf("    crank %d: measured M-0 %s  M-1 %s\n", rpm, m0.Text().c_str(), m1.Text().c_str());
                const double want0 = 5.0 * rpm, want1 = 1.5 * rpm;
                r.Check(tag + " at crank " + std::to_string(rpm) + " rpm: M-1 steady at " + Num(want1, "%.0f") + " rpm, M-0 at " + Num(want0, "%.0f"),
                        m0.ok && m1.ok && std::fabs(m0.Peak() - want0) < 0.03 * want0 && std::fabs(m1.Peak() - want1) < 0.1 * want1 &&
                            m1.hi - m1.lo < 12,  // 12: 3 steps of the reading (3.93 rpm)
                        m0.Text() + " / " + m1.Text());
            }
            Pulse(*lk, C_STOP);
            WaitStatus(*lk, CrankOff(), 8);
            Sleep(300);
            s = CurrentStatus(*lk);
            w.End();
            const int64_t units = static_cast<int64_t>(s.rev) * 4000 + std::llround(s.angle / 90.0);  // 0.09 deg each
            const int64_t want = FloorDiv(units * 800 * num, 2 * 4000);
            r.Check(tag + ": exact at rest after " + std::to_string(s.rev) + " crank revolutions", std::llabs(s.axis[0].counts - want) <= 1 && w.faults == 0,
                    "M-1 " + std::to_string(s.axis[0].counts) + " counts, " + std::to_string(units) + " crank units x " + std::to_string(num) + "/10 = " + std::to_string(want));
        }
        Release(*lk);
    }

    printf("== TRIGGER: a one-shot 1 s timeline on M-0 + M-1, started by the PLC's bit\n    %s\n", Upload(con, kLiveTrigger).c_str());
    lk = FreshLink(ip);
    lk->SetFlags(C_CONTROL);
    Sleep(150);
    lk->SetFlags(C_CONTROL | C_ENABLE);
    auto both = [](const Sts &x) { return (x.flags & F_ENABLED) && (x.axis[0].flags & 8) && (x.axis[1].flags & 8); };
    s = WaitStatus(*lk, both, 5);
    r.Check("ENABLE (timeline profile): both enabled, HLFB on both axes", both(s));
    Pulse(*lk, C_START);
    s = WaitStatus(*lk, [](const Sts &x) { return x.tl == 1; }, 2);
    r.Check("START: timeline armed (waiting for TRIGGER)", s.tl == 1, Name(kTlStates, 6, s.tl));
    auto job = scope.Start(8);
    Sleep(2000);
    int done = 0;
    int hi0, hi1, lo0, lo1, faults;
    {
        StatusWatch w(*lk, 2);
        for (int i = 0; i < 3; i++) {
            WaitStatus(*lk, [](const Sts &x) { return x.tl == 1 || x.tl == 4; }, 3);
            lk->SetFlags(C_CONTROL | C_ENABLE | C_TRIGGER);
            WaitStatus(*lk, [](const Sts &x) { return x.tl == 2; }, 1);
            lk->SetFlags(C_CONTROL | C_ENABLE);
            s = WaitStatus(*lk, [](const Sts &x) { return x.tl == 4; }, 3);
            done += s.tl == 4;
            Sleep(600);
        }
        w.End();
        hi0 = w.hi[0], hi1 = w.hi[1], lo0 = w.lo[0], lo1 = w.lo[1], faults = w.faults;
    }
    const auto sc = job.get();
    r.Check("TRIGGER x3: the timeline runs and completes each time", done == 3, std::to_string(done) + " of 3");
    r.Check("M-0 commanded 0 .. 2400 counts (3 revs), M-1 0 .. 1600 (2 revs), no faults", hi0 >= 2390 && hi1 >= 1590 && lo0 <= 5 && lo1 <= 5 && faults == 0,
            "max " + std::to_string(hi0) + " " + std::to_string(hi1));
    const int peaks[2] = {720, 480};
    for (int i = 0; i < 2; i++) {
        const Reading &m = sc.m[i];
        const double peak = peaks[i];
        r.Check(std::string("TRIGGER: M-") + std::to_string(i) + " measured out and back, peak about " + std::to_string(peaks[i]) + " rpm each way",
                m.ok && std::fabs(m.hi - peak) < 0.1 * peak && std::fabs(-m.lo - peak) < 0.1 * peak, m.Text());
    }
    mot = LiveMotors(con);
    bool clean = mot.size() == 2;
    for (const std::string &m : mot) {
        clean = clean && m.find("alerts=0x00000000") != std::string::npos && m.find("in_fault=0") != std::string::npos;
    }
    r.Check("no alerts on either motor after all of it", clean);
    lk->SetFlags(C_CONTROL);  // ENABLE falls: disable
    s = WaitStatus(*lk, Disabled(), 3);
    r.Check("ENABLE dropped by the PLC: motors disabled", !(s.flags & F_ENABLED));
    lk->SetFlags(0);
    Sleep(150);
    lk->Close();
}

long long CommLosses(Console &c) { return IntAfter(Find(c.Cmd("eip"), "eip control"), "comm_losses="); }

/** A real connection loss, made by a person: the machine runs under the PLC, and someone pulls the PLC's cable or puts it in
 *  Program mode. The console is the only witness, so this keeps working while the PLC is out of reach. */
void PlcLoss(const std::string &ip, Console &con, Results &r, double waitSeconds) {
    for (const char *c : {"abort", "disable", "sim 0", "clear"}) {
        con.Cmd(c);
    }
    printf("    %s\n", Upload(con, kLiveCrank).c_str());
    auto lk = FreshLink(ip);
    EnableUnderPlc(*lk);
    lk->SetValue(4, 12000);
    Pulse(*lk, C_START);
    const Sts s = WaitStatus(*lk, CrankAt(12000), 6);
    r.Check("the machine runs under the PLC at 120 rpm (M-0 600 rpm)", CrankAt(12000)(s) && (s.flags & F_PLC), FormatStatus(s));
    const long long loss0 = CommLosses(con);
    printf("READY: pull the PLC's cable, or put it in Program mode\n");
    fflush(stdout);

    std::vector<std::string> seen;
    double tOff = 0, tDis = 0, tFirst = 0;
    const double t0 = PreciseSeconds();
    while (PreciseSeconds() - t0 < waitSeconds) {
        auto c = ConsoleStatus(con);
        const double now = PreciseSeconds();
        if (seen.empty() || seen.back() != c["crank"]) {
            seen.push_back(c["crank"]);
        }
        if (tFirst == 0 && c["crank"] != "running") tFirst = now;
        if (tOff == 0 && c["crank"] == "off") tOff = now;
        if (tDis == 0 && c["enabled"] == "0") tDis = now;
        if (tFirst != 0 && now - tFirst > 3) {
            break;
        }
    }
    std::string path;
    bool stopping = false;
    for (const std::string &x : seen) {
        path += x + " ";
        stopping = stopping || x == "stopping";
    }
    r.Check("the loss was seen", tFirst != 0, "crank " + path);
    if (tFirst == 0) {
        return;
    }
    auto c = ConsoleStatus(con);
    const long long loss1 = CommLosses(con);
    printf("    %s\n", Find(con.Cmd("eip"), "eip connection").c_str());
    r.Check("controlled stop: crank running -> stopping -> off", stopping && seen.back() == "off", path);
    r.Check("then disabled, COMM_LOSS latched, comm_losses + 1", c["enabled"] == "0" && c["fault"] != "none" && loss1 == loss0 + 1,
            "enabled=" + c["enabled"] + " fault=" + c["fault"] + " comm_losses " + std::to_string(loss0) + " -> " + std::to_string(loss1));
    r.Check("the disable waits for HLFB, not the whole 500 ms", tOff != 0 && tDis != 0 && (tDis - tOff) * 1000 < 450, Num((tDis - tOff) * 1000, "%.0f") + " ms");

    printf("WAITING: reconnect the cable, or put the PLC back in Run\n");
    fflush(stdout);
    bool back = false;
    const double t1 = PreciseSeconds();
    while (PreciseSeconds() - t1 < waitSeconds && !back) {
        const std::string e = Find(con.Cmd("eip"), "eip connection");
        back = e.find("open=1") != std::string::npos && e.find("run=1") != std::string::npos;
        Sleep(200);
    }
    r.Check("the PLC came back on its own", back, Find(con.Cmd("eip"), "eip connection"));
    Sleep(3000);  // its bits are still set: back in Run or reconnected, they must start nothing
    c = ConsoleStatus(con);
    r.Check("coming back with ENABLE and control held starts nothing", c["enabled"] == "0" && c["crank"] == "off",
            "enabled=" + c["enabled"] + " crank=" + c["crank"] + " fault=" + c["fault"]);
    lk->Close();
    Sleep(300);
    con.Cmd("clear");
    c = ConsoleStatus(con);
    r.Check("clear: no fault, disabled", c["fault"] == "none" && c["enabled"] == "0", "fault=" + c["fault"]);
}

}  // namespace

int SuitePlcLoss(const std::string &ip, const std::string &restore, double waitSeconds) {
    return RunSuite(ip, restore, [&](Console &con, Results &r) { PlcLoss(ip, con, r, waitSeconds); });
}

int SuiteLive(const std::string &ip, const std::string &restore, const std::string &tools) {
    char full[MAX_PATH];
    const std::string dir = GetFullPathNameA(tools.c_str(), sizeof(full), full, nullptr) ? full : tools;
    return RunSuite(ip, restore, [&](Console &con, Results &r) { Live(ip, con, r, dir); });
}

}  // namespace bench
