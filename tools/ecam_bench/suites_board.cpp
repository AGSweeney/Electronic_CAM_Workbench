// MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
// Phase 2 and 3: the status assembly against the console, and the command assembly. M-0 and M-1 move, slowly.
// Every connection runs at the PLC's 20 ms with its 160 ms timeout. The Python suite ran 10 ms, and its "80 ms timeout"
// was never sent: Forward_Open always carried x16, so the timeout there was 160 ms as well.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <thread>

#include "suites.h"

namespace bench {

namespace {

const char *const kBenchNoMotor =
    "master src=encoder cpr=4000 den=1 rpm=20 crank=0 ramp=60 max=600\n"
    "timeline repeat=1 ovr=1 trig=cmd in=0 rise=1 bit=0\n"
    "axis 0 master=crank m=2 cpu=800 start=0 closed=1 span=360 units=mm name=Nothing\n"
    "seg 0 180 5 cycloidal\n"
    "seg 0 360 0 cycloidal\n";

uint32_t HashOf(const std::string &commit) {
    const size_t at = commit.find("hash=");
    return at == std::string::npos ? 0 : static_cast<uint32_t>(strtoul(commit.c_str() + at + 5, nullptr, 16));
}

long long LateSamples(Console &c, std::string *line = nullptr) {
    for (const std::string &l : c.Cmd("diag")) {
        if (l.find("late samples ") != std::string::npos) {
            if (line) {
                *line = l;
            }
            return IntAfter(l, "late samples ");
        }
    }
    throw std::runtime_error("diag has no late samples");
}

std::string EipConnection(Console &c) { return Find(c.Cmd("eip"), "eip connection"); }

/** The first error line of a reply, or "". */
std::string ConsoleErr(const std::vector<std::string> &lines) {
    for (const std::string &l : lines) {
        if (l.rfind("error", 0) == 0) {
            return l;
        }
    }
    return "";
}

/** A-9 isn't wired, and set is refused while enabled: estop_test reads it as open. */
Sts TripEstop(Console &c, Link &l) {
    c.Cmd("estop_test 1");
    return WaitStatus(l, [](const Sts &x) { return x.fault != 0; }, 2);
}

double LongestGapMs(const std::vector<double> &times, double from) {
    double last = 0, gap = 0;
    for (double t : times) {
        if (t < from) {
            continue;
        }
        if (last != 0) {
            gap = std::max(gap, (t - last) * 1000);
        }
        last = t;
    }
    return gap;
}

bool Has(const std::string &s, const char *what) { return s.find(what) != std::string::npos; }

std::string F2(double v) {
    char t[32];
    snprintf(t, sizeof(t), "%.2f", v);
    return t;
}

void Status(const std::string &ip, Console &con, Results &r, const std::string &workbench, const std::string &project) {
    for (const char *c : {"disable", "sim 0", "clear"}) {
        con.Cmd(c);
    }

    printf("== identity and the stream\n");
    const Identity me = ListIdentity(ip);
    r.Check("identity revision " + std::to_string(kEipRevMajor) + "." + std::to_string(kEipRevMinor),
            me.revMajor == kEipRevMajor && me.revMinor == kEipRevMinor);
    const auto d = Discover();
    bool fw = false;
    for (const auto &e : d) {
        fw = fw || Has(e.second, (std::string("FW=") + kFwVersion).c_str());
    }
    r.Check(std::string("discovery reports FW=") + kFwVersion, fw, d.empty() ? "nothing" : d[0].second);

    auto link = std::make_unique<Link>(ip, 20);
    link->Open();
    Sleep(200);
    const uint64_t n0 = link->RxCount();
    std::vector<uint32_t> hb;
    const double t0 = PreciseSeconds();
    while (PreciseSeconds() - t0 < 2.0) {
        const uint32_t h = CurrentStatus(*link).heartbeat;
        if (hb.empty() || hb.back() != h) {
            hb.push_back(h);
        }
        Sleep(2);
    }
    const double rate = (link->RxCount() - n0) / 2.0;
    long long lo = 1 << 30, hi = 0, sum = 0;
    for (size_t i = 1; i < hb.size(); i++) {
        const long long st = static_cast<long long>(hb[i]) - hb[i - 1];
        lo = std::min(lo, st);
        hi = std::max(hi, st);
        sum += st;
    }
    const double mean = hb.size() > 1 ? static_cast<double>(sum) / (hb.size() - 1) : 0;
    r.Check("status at 20 ms RPI: ~50 packets/s", rate > 47.5 && rate < 52.5, F2(rate) + "/s");
    r.Check("heartbeat rises every packet, ~20 ms apart", hb.size() > 1 && lo > 0 && mean >= 16 && mean <= 24,
            std::to_string(hb.size()) + " values, steps " + std::to_string(lo) + ".." + std::to_string(hi) + " ms");
    Sts s = CurrentStatus(*link);
    r.Check("FirmwareRev " + Hex(kFwMajor * 256 + kFwMinor, 4) + ", reserved bytes zero",
            s.fw == kFwMajor * 256 + kFwMinor && s.reservedZero, Hex(s.fw, 4));
    {
        Link ex(ip);
        ex.Connect();
        const CipReply c = ex.Explicit(0x0E, Path(0x04, kAsmSts, 3));
        const Sts e = DecodeStatus(c.data);
        r.Check("explicit Get of assembly 100: 160 bytes, current",
                c.gen == 0 && e.valid && std::llabs(static_cast<long long>(e.heartbeat) - CurrentStatus(*link).heartbeat) < 100,
                "general " + Hex(c.gen) + ", " + std::to_string(c.data.size()) + " bytes");
    }

    printf("== driven crank (simulate)\n");
    uint32_t h = HashOf(Upload(con, kBenchDriven));
    s = WaitStatus(*link, [&](const Sts &x) { return x.hash == h; });
    auto c = ConsoleStatus(con);
    r.Check("ProfileHash = the console's; LOADED STORED, disabled, 1 crank axis on M-1",
            s.hash == h && strtoul(c["profile"].c_str(), nullptr, 16) == h && (s.flags & (F_LOADED | F_STORED)) == (F_LOADED | F_STORED) &&
                s.state == 1 && s.axes == 1 && s.axis[0].conn == 1 && (s.axis[0].flags & 3) == 1,
            FormatStatus(s));
    r.Check("inputs: estop OK (estop_a9 0)", (s.inputs & 2) && !(s.flags & F_ESTOP), Hex(s.inputs));
    con.Cmd("sim 1");
    s = WaitStatus(*link, [](const Sts &x) { return (x.flags & F_SIM) != 0; });
    r.Check("sim 1: SIMULATE READY, state ready", (s.flags & (F_SIM | F_READY)) == (F_SIM | F_READY) && s.state == 2, FormatStatus(s));
    const long long late0 = LateSamples(con);
    con.Cmd("start");
    auto at20 = [](const Sts &x) { return x.crank == 2 && std::abs(x.rpm - 2000) < 5; };
    s = WaitStatus(*link, at20, 3);
    r.Check("start: crank running, 20.00 rpm, state running, RUNNING", at20(s) && s.state == 4 && (s.flags & F_RUNNING), FormatStatus(s));
    int32_t prev = CurrentStatus(*link).angle;
    bool steady = true;
    std::string steps;
    for (int i = 0; i < 5; i++) {
        Sleep(100);
        const int32_t a = CurrentStatus(*link).angle;
        const int32_t st = ((a - prev) % 360000 + 360000) % 360000;
        steady = steady && st > 9000 && st < 15000;
        steps += std::to_string(st) + " ";
        prev = a;
    }
    r.Check("crank angle advances 120 deg/s (12 deg per 0.1 s)", steady, steps + "mdeg");
    s = CurrentStatus(*link);
    r.Check("axis travel = counts / 800 (mm x 1000)",
            s.axis[0].travel == std::lround(s.axis[0].counts / 800.0 * 1000) && s.axis[0].counts != 0,
            "counts " + std::to_string(s.axis[0].counts) + ", travel " + std::to_string(s.axis[0].travel));
    con.Cmd("rpm 40");
    s = WaitStatus(*link, [](const Sts &x) { return std::abs(x.rpm - 4000) < 5; }, 3);
    r.Check("rpm 40 on the console: 40.00 rpm in the status", std::abs(s.rpm - 4000) < 5, F2(s.rpm / 100.0));
    con.Cmd("stop");
    s = WaitStatus(*link, [](const Sts &x) { return (x.flags & F_STOPPING) && x.crank == 3; }, 2);
    r.Check("stop: STOPPING while the crank ramps down", s.crank == 3 && (s.flags & F_STOPPING), Name(kCrankStates, 5, s.crank));
    WaitStatus(*link, [](const Sts &x) { return x.crank == 0; }, 5);
    Sleep(100);
    s = CurrentStatus(*link);
    c = ConsoleStatus(con);
    r.Check("stopped: crank off, 0 rpm, state ready", s.crank == 0 && s.rpm == 0 && s.state == 2 && !(s.flags & (F_RUNNING | F_STOPPING)), FormatStatus(s));
    r.Check("at rest the status equals the console's (angle, rev, a0)",
            std::fabs(s.angle - atof(c["deg"].c_str()) * 1000) <= 10 && s.rev == atoi(c["rev"].c_str()) && s.axis[0].counts == atoi(c["a0"].c_str()),
            "eip " + std::to_string(s.angle) + " mdeg rev " + std::to_string(s.rev) + " a0 " + std::to_string(s.axis[0].counts) +
                "; console deg=" + c["deg"] + " rev=" + c["rev"] + " a0=" + c["a0"]);
    std::string dline;
    const long long late1 = LateSamples(con, &dline);
    r.Check("no late samples while running with the 20 ms connection", late1 == late0, dline.substr(0, 110));

    printf("== motors (M-0 crank, M-1 axis)\n");
    con.Cmd("sim 0");
    const auto en = con.Cmd("enable");
    s = WaitStatus(*link, [](const Sts &x) { return (x.flags & F_ENABLED) != 0; }, 3);
    r.Check("enable: ENABLED READY, axis 0 enabled + HLFB",
            en.back().rfind("ok", 0) == 0 && (s.flags & (F_ENABLED | F_READY)) == (F_ENABLED | F_READY) && (s.axis[0].flags & 0b1101) == 0b1101,
            en.back() + "; a0 flags " + Hex(s.axis[0].flags));
    con.Cmd("rpm 20");
    con.Cmd("start");
    s = WaitStatus(*link, at20, 3);
    Sleep(1000);
    const Sts s2 = CurrentStatus(*link);
    r.Check("running on the motors: 20 rpm, a0 moving, no motor fault",
            at20(s2) && s2.axis[0].counts != s.axis[0].counts && !(s2.axis[0].flags & 0x10), FormatStatus(s2));
    // A flash write in motion holds the sample off, and the drive gets steps the board never counted.
    const std::string ref1 = con.Cmd("set stop_ms 501").back(), ref2 = con.Cmd("forget").back();
    r.Check("set and forget refused while moving", Has(ref1, "stop the motion") && Has(ref2, "stop the motion"), ref1 + " / " + ref2);
    con.Cmd("stop");
    WaitStatus(*link, [](const Sts &x) { return x.crank == 0; }, 5);
    // At rest but enabled is refused too, and it takes nothing with it.
    const std::string ref3 = con.Cmd("set stop_ms 501").back(), ref4 = con.Cmd("forget").back();
    r.Check("set and forget refused while enabled at rest", Has(ref3, "disable the motors") && Has(ref4, "disable the motors"), ref3 + " / " + ref4);
    con.Cmd("disable");
    s = WaitStatus(*link, [](const Sts &x) { return !(x.flags & F_ENABLED); }, 2);
    r.Check("disable: ENABLED clear, axis 0 not enabled", !(s.flags & F_ENABLED) && !(s.axis[0].flags & 4), FormatStatus(s));

    printf("== fault code\n");
    con.Cmd("enable");
    WaitStatus(*link, [](const Sts &x) { return (x.flags & F_ENABLED) != 0; }, 3);
    s = TripEstop(con, *link);
    r.Check("estop trips while enabled: FaultCode 1, FAULTED ESTOP_OPEN, state fault, disabled",
            s.fault == 1 && (s.flags & (F_FAULTED | F_ESTOP)) == (F_FAULTED | F_ESTOP) && s.state == 6 && !(s.flags & F_ENABLED) && !(s.inputs & 2),
            FormatStatus(s));
    con.Cmd("estop_test 0");
    con.Cmd("clear");
    con.Cmd("disable");
    s = WaitStatus(*link, [](const Sts &x) { return x.fault == 0 && !(x.flags & F_ENABLED); }, 2);
    r.Check("clear: FaultCode 0, state disabled", s.fault == 0 && s.state == 1 && !(s.flags & F_FAULTED), FormatStatus(s));

    printf("== encoder master and timeline (simulate)\n");
    h = HashOf(Upload(con, kBenchTimeline));
    con.Cmd("sim 1");
    con.Cmd("simenc 4000");  // one crank revolution a second
    con.Cmd("start");
    con.Cmd("trigger");
    s = WaitStatus(*link, [&](const Sts &x) { return x.hash == h && x.tl == 2 && x.crank == 1; }, 3);
    r.Check("crank following, timeline running, 2 axes (crank M-0, timeline M-1)",
            s.crank == 1 && s.tl == 2 && s.axes == 2 && s.axis[0].conn == 0 && s.axis[1].conn == 1 && (s.axis[1].flags & 2) && !(s.axis[0].flags & 2),
            FormatStatus(s));
    std::set<uint16_t> outs;
    int tMin = 1 << 30, tMax = 0;
    const int32_t cyc0 = s.cycle;
    const double t1 = PreciseSeconds();
    while (PreciseSeconds() - t1 < 4.5) {
        s = CurrentStatus(*link);
        outs.insert(s.outputs);
        tMin = std::min(tMin, s.tUs);
        tMax = std::max(tMax, s.tUs);
        Sleep(10);
    }
    r.Check("timeline time runs 0..2 s and the cycle count rises by 2",
            tMax > 1900000 && tMin < 100000 && (s.cycle - cyc0 == 2 || s.cycle - cyc0 == 3),
            "t " + std::to_string(tMin) + ".." + std::to_string(tMax) + " us, cycles +" + std::to_string(s.cycle - cyc0));
    r.Check("Outputs shows IO-1 on during 0.2..0.6 s (simulate: pins stay off)", outs == std::set<uint16_t>{0, 2},
            std::to_string(outs.size()) + " values seen");
    r.Check("override 100.00 %", s.ovr == 10000, std::to_string(s.ovr));
    const int32_t e1 = CurrentStatus(*link).enc;
    c = ConsoleStatus(con);
    r.Check("encoder count follows the simulated encoder (4000/s)", std::abs(atoi(c["enc"].c_str()) - e1) < 400,
            "eip " + std::to_string(e1) + " console " + c["enc"]);
    con.Cmd("override 50");
    s = WaitStatus(*link, [](const Sts &x) { return x.ovr == 5000; }, 5);
    r.Check("override 50 on the console: 50.00 % in the status", s.ovr == 5000, std::to_string(s.ovr));
    con.Cmd("stop");
    s = WaitStatus(*link, [](const Sts &x) { return (x.tl == 0 || x.tl == 4) && x.crank == 0; }, 6);
    r.Check("stop: crank off, timeline idle or done", s.crank == 0 && (s.tl == 0 || s.tl == 4), FormatStatus(s));
    con.Cmd("sim 0");

    printf("== a profile upload while the PLC is connected\n");
    link->Close();  // one owner at a time
    Sleep(300);
    {
        Link plc(ip, 20);  // the PLC's 160 ms timeout
        plc.Open();
        Sleep(300);
        const double u0 = PreciseSeconds();
        Upload(con, kBenchDriven);
        const double u1 = PreciseSeconds();
        Sleep(500);
        const double gap = LongestGapMs(plc.RxTimes(), u0 - 0.05);
        const std::string eipl = EipConnection(con);
        LateSamples(con, &dline);
        const size_t lc = dline.find("last commit: ");
        r.Check("upload + commit with a 20 ms / 160 ms-timeout connection open: it stays open, gap < 80 ms", Has(eipl, "open=1") && gap < 80,
                "longest gap " + F2(gap) + " ms, upload " + F2((u1 - u0) * 1000) + " ms; " + (lc == std::string::npos ? "" : dline.substr(lc + 13)));
    }
    Sleep(300);

    printf("== CommLossCount\n");
    link = std::make_unique<Link>(ip, 20);
    link->Open();
    const uint32_t cl0 = CurrentStatus(*link).commloss;
    link->Drop();  // stop sending without Forward_Close: the board times out
    const double w0 = PreciseSeconds();
    while (PreciseSeconds() - w0 < 8 && !Has(EipConnection(con), "open=0")) {
        Sleep(200);
    }
    link->Close();
    link = std::make_unique<Link>(ip, 20);
    link->Open();
    s = CurrentStatus(*link);
    r.Check("a timed-out connection counts: CommLossCount + 1", s.commloss == cl0 + 1, std::to_string(cl0) + " -> " + std::to_string(s.commloss));

    if (!workbench.empty() && !project.empty()) {
        printf("== the Workbench runs the machine while the PLC reads\n");
        con.Close();  // the Workbench takes the console
        const double b0 = PreciseSeconds();
        std::set<uint16_t> states;
        uint32_t lastHb = 0;
        std::atomic<bool> done{false};
        int rc = -1;
        std::string out;
        std::thread job([&] {
            out = Exec("\"" + workbench + "\" " + ip + " \"" + project + "\"", 300000, &rc);
            done = true;
        });
        while (!done) {
            s = CurrentStatus(*link);
            states.insert(s.state);
            lastHb = s.heartbeat;
            Sleep(5);
        }
        job.join();
        r.Check("board_ui_jog passes alongside", rc == 0, "exit " + std::to_string(rc));
        std::vector<double> times;
        for (double t : link->RxTimes()) {
            if (t > b0) {
                times.push_back(t);
            }
        }
        const double last = times.empty() ? b0 : times.back();
        const double wrate = times.size() > 1 ? (times.size() - 1) / (last - times.front()) : 0;
        r.Check("status stream kept its rate until the board restarted (20 ms RPI)", wrate > 47.5,
                F2(wrate) + "/s over " + F2(last - b0) + " s, longest gap " + F2(LongestGapMs(times, 0)) + " ms (upload, forget)");
        r.Check("the PLC saw the Workbench jog (state jogging)", states.count(5) != 0);
        link->Close();
        const double wd = PreciseSeconds();
        for (bool up = false; !up && PreciseSeconds() - wd < 40;) {
            try {
                ListIdentity(ip, 0.5);
                up = true;
            } catch (const std::exception &) {
                Sleep(300);
            }
        }
        link = std::make_unique<Link>(ip, 20);
        link->Open();
        const uint32_t uptime = CurrentStatus(*link).heartbeat;
        const double since = (PreciseSeconds() - last) * 1000;
        r.Check("the stream ended at the restart: the board's uptime is younger than the gap, the PLC reconnects",
                uptime < since && uptime < lastHb, "uptime " + std::to_string(uptime) + " ms, " + F2(since) + " ms since the last packet");
        con.Open(ip.c_str());
    }
    link->Close();
}

void Cmd(const std::string &ip, Console &con, Results &r) {
    for (const char *c : {"abort", "disable", "sim 0", "clear"}) {
        con.Cmd(c);
    }
    Upload(con, kBenchDriven);
    con.Cmd("rpm 20");
    auto enabled = [](const Sts &x) { return (x.flags & F_ENABLED) != 0; };
    auto notEnabled = [](const Sts &x) { return !(x.flags & F_ENABLED); };
    auto crank = [](uint16_t want) { return [want](const Sts &x) { return x.crank == want; }; };
    auto rejected = [](const Sts &x) { return x.reject != 0; };
    auto at20 = [](const Sts &x) { return x.crank == 2 && std::abs(x.rpm - 2000) < 5; };

    printf("== a PLC without PLC_CONTROL reads, and can only stop, abort, drop ENABLE\n");
    {
        Link link(ip, 20);
        link.Open();
        Sts s = WaitStatus(link, [](const Sts &) { return true; });
        r.Check("FirmwareRev " + std::to_string(kFwMajor) + "." + std::to_string(kFwMinor) + ", status flows, not in control",
                s.fw == kFwMajor * 256 + kFwMinor && !(s.flags & F_PLC), FormatStatus(s));
        Sleep(300);  // past the first packet: later changes are edges
        link.SetFlags(C_ENABLE);
        s = WaitStatus(link, rejected, 1);
        r.Check("ENABLE without PLC_CONTROL: refused, RejectCode 11, REJECTED, not enabled",
                s.reject == 11 && (s.flags & F_REJECTED) && !(s.flags & F_ENABLED), "reject " + std::to_string(s.reject));
        auto c = ConsoleStatus(con);
        r.Check("console status: eip=run eip_from=" + LocalIp() + " eip_rpi=20",
                c["eip"] == "run" && c["eip_from"] == LocalIp() && c["eip_rpi"] == "20", "eip=" + c["eip"] + " from=" + c["eip_from"] + " rpi=" + c["eip_rpi"]);
        const auto en = con.Cmd("enable");
        s = WaitStatus(link, enabled, 2);
        r.Check("the console still enables", ConsoleErr(en).empty() && (s.flags & F_ENABLED), en.back());
        link.SetFlags(0);  // ENABLE drops: always works
        s = WaitStatus(link, notEnabled, 2);
        r.Check("dropping ENABLE disables the motors the console enabled", !(s.flags & F_ENABLED), FormatStatus(s));
        con.Cmd("sim 1");
        con.Cmd("start");
        WaitStatus(link, crank(2), 3);
        Pulse(link, C_STOP);
        s = WaitStatus(link, [](const Sts &x) { return x.crank == 0 || x.crank == 3; }, 2);
        r.Check("STOP edge stops a console-started run (controlled)", s.crank == 0 || s.crank == 3, Name(kCrankStates, 5, s.crank));
        WaitStatus(link, crank(0), 3);
        con.Cmd("start");
        WaitStatus(link, crank(2), 3);
        Pulse(link, C_ABORT);
        s = WaitStatus(link, crank(0), 1);
        r.Check("ABORT edge stops it at once", s.crank == 0, Name(kCrankStates, 5, s.crank));
        con.Cmd("sim 0");
    }
    Sleep(300);

    printf("== bits already set at connect are not edges\n");
    auto link = std::make_unique<Link>(ip, 20);
    link->SetFlags(C_CONTROL | C_ENABLE | C_START);
    link->Open();
    Sleep(800);
    Sts s = CurrentStatus(*link);
    r.Check("connected holding CONTROL+ENABLE+START: in control, but not enabled and not started",
            (s.flags & F_PLC) && !(s.flags & F_ENABLED) && s.crank == 0 && s.reject == 0, FormatStatus(s));

    printf("== PLC control: the console may only stop, abort, disable\n");
    auto c = ConsoleStatus(con);
    r.Check("console status: eip=control", c["eip"] == "control", "eip=" + c["eip"]);
    std::string bad;
    for (const char *cmd : {"enable", "start", "sim 1", "rpm 30", "override 50", "jog on", "trigger", "load", "commit", "forget", "simenc 100", "length 10"}) {
        const std::string e = ConsoleErr(con.Cmd(cmd));
        if (e != "error: the PLC has control") {
            bad += std::string(cmd) + ": '" + e + "' ";
        }
    }
    r.Check("console refuses enable start sim rpm override jog trigger load commit forget simenc length", bad.empty(), bad);
    bad.clear();
    for (const char *cmd : {"stop", "abort", "disable", "status", "config", "eip"}) {
        const std::string e = ConsoleErr(con.Cmd(cmd));
        if (!e.empty()) {
            bad += std::string(cmd) + ": " + e + " ";
        }
    }
    r.Check("console still answers stop, abort, disable, status, config, eip", bad.empty(), bad);

    printf("== edges under control, RejectCode\n");
    link->SetFlags(C_CONTROL);  // ENABLE and START seen off
    Sleep(100);
    Pulse(*link, C_START);
    s = WaitStatus(*link, rejected, 1);
    r.Check("START with the motors disabled: RejectCode 12", s.reject == 12 && s.crank == 0, "reject " + std::to_string(s.reject));
    link->SetFlags(C_CONTROL | C_ENABLE);
    s = WaitStatus(*link, enabled, 3);
    r.Check("ENABLE rising: enabled, RejectCode cleared", (s.flags & F_ENABLED) && s.reject == 0 && !(s.flags & F_REJECTED), FormatStatus(s));
    Pulse(*link, C_START);
    s = WaitStatus(*link, at20, 3);
    r.Check("START: running on the motors at 20 rpm", at20(s), FormatStatus(s));
    Pulse(*link, C_STOP, 50);  // two RPIs and a bit: PROTOCOL.md's minimum
    s = WaitStatus(*link, crank(3), 1);
    r.Check("STOP: controlled (STOPPING while the crank ramps down)", s.crank == 3 && (s.flags & F_STOPPING), Name(kCrankStates, 5, s.crank));
    WaitStatus(*link, crank(0), 3);
    Pulse(*link, C_START);
    WaitStatus(*link, crank(2), 3);
    Pulse(*link, C_ABORT, 50);
    s = WaitStatus(*link, [](const Sts &x) { return x.crank != 2; }, 1);
    r.Check("ABORT: off at once (no STOPPING)", s.crank == 0, Name(kCrankStates, 5, s.crank));
    Pulse(*link, C_START);
    s = WaitStatus(*link, crank(2), 3);
    r.Check("START again after ABORT", s.crank == 2);
    link->SetFlags(C_CONTROL);
    s = WaitStatus(*link, [](const Sts &x) { return !(x.flags & F_ENABLED) && x.crank == 0; }, 2);
    r.Check("ENABLE dropped while running: disabled, stopped", !(s.flags & F_ENABLED) && s.crank == 0, FormatStatus(s));

    printf("== simulate, faults, CLEAR_FAULT\n");
    link->SetFlags(C_CONTROL | C_SIM);
    s = WaitStatus(*link, [](const Sts &x) { return (x.flags & F_SIM) != 0; }, 2);
    r.Check("SIMULATE held: sim 1", (s.flags & F_SIM) && ConsoleStatus(con)["sim"] == "1");
    link->SetFlags(C_CONTROL);
    s = WaitStatus(*link, [](const Sts &x) { return !(x.flags & F_SIM); }, 2);
    r.Check("SIMULATE dropped: sim 0", !(s.flags & F_SIM) && ConsoleStatus(con)["sim"] == "0");
    link->SetFlags(C_CONTROL | C_ENABLE);
    WaitStatus(*link, enabled, 3);
    s = TripEstop(con, *link);
    r.Check("estop: FaultCode 1, disabled", s.fault == 1 && !(s.flags & F_ENABLED), FormatStatus(s));
    Pulse(*link, C_START);
    s = WaitStatus(*link, rejected, 1);
    r.Check("START with a fault latched: RejectCode 19", s.reject == 19, "reject " + std::to_string(s.reject));
    con.Cmd("estop_test 0");
    Pulse(*link, C_CLEAR);
    s = WaitStatus(*link, [](const Sts &x) { return x.fault == 0; }, 1);
    r.Check("CLEAR_FAULT: FaultCode 0, RejectCode 0", s.fault == 0 && s.reject == 0, FormatStatus(s));
    link->SetFlags(C_CONTROL);
    Sleep(100);
    link->SetFlags(C_CONTROL | C_ENABLE);  // ENABLE again after the fault: needs a new edge
    s = WaitStatus(*link, enabled, 3);
    r.Check("ENABLE again (a new edge) after the fault", (s.flags & F_ENABLED) != 0);
    link->SetFlags(C_CONTROL);
    WaitStatus(*link, notEnabled, 2);

    printf("== HLFB on enable (a profile on M-2, no motor there)\n");
    link->Close();  // release control to upload
    Sleep(300);
    Upload(con, kBenchNoMotor);
    link = std::make_unique<Link>(ip, 20);  // the 1 s HLFB wait must not drop a 160 ms connection
    link->SetFlags(C_CONTROL);
    link->Open();
    Sleep(300);
    const double t0 = PreciseSeconds();
    link->SetFlags(C_CONTROL | C_ENABLE);
    s = WaitStatus(*link, [](const Sts &x) { return x.fault != 0; }, 3);
    const double gap = LongestGapMs(link->RxTimes(), t0);
    r.Check("ENABLE with no HLFB: FaultCode 3 latched, RejectCode 19, disabled", s.fault == 3 && s.reject == 19 && !(s.flags & F_ENABLED), FormatStatus(s));
    r.Check("the 1 s HLFB wait keeps the connection running", gap < 60 && Has(EipConnection(con), "open=1"), "longest gap " + F2(gap) + " ms");
    link->SetFlags(C_CONTROL);
    Pulse(*link, C_CLEAR);
    link->Close();
    Sleep(300);
    Upload(con, kBenchDriven);
    con.Cmd("rpm 20");

    printf("== connection loss while the PLC has control (controlled stop, COMM_LOSS)\n");
    for (const char *how : {"timeout (cable pulled)", "Program mode", "Forward_Close"}) {
        auto lk = std::make_unique<Link>(ip, 20);
        lk->SetFlags(C_CONTROL);
        lk->Open();
        Sleep(200);
        lk->SetFlags(C_CONTROL | C_ENABLE);
        WaitStatus(*lk, enabled, 3);
        Pulse(*lk, C_START);
        const uint32_t cl0 = WaitStatus(*lk, at20, 3).commloss;
        if (how[0] == 't') {
            lk->StopIo();
        } else if (how[0] == 'P') {
            lk->SetRun(false);
        } else {
            lk->StopIo();
            lk->ForwardClose();
        }
        const auto cranks = WatchConsole(con, 1.5, "crank");
        auto st = ConsoleStatus(con);
        lk->Close();
        Sleep(300);
        Sts after;
        {
            Link rd(ip, 20);
            rd.Open();
            after = CurrentStatus(rd);
        }
        Sleep(200);
        std::string path;
        bool stopping = false;
        for (const std::string &x : cranks) {
            path += x + " ";
            stopping = stopping || x == "stopping";
        }
        r.Check(std::string(how) + ": controlled stop (crank running -> stopping -> off)", stopping && cranks.back() == "off", path);
        r.Check(std::string(how) + ": then disabled, COMM_LOSS (FaultCode 4), CommLossCount + 1",
                st["enabled"] == "0" && after.fault == 4 && after.state == 6 && after.commloss == cl0 + 1,
                "enabled=" + st["enabled"] + " fault=" + std::to_string(after.fault) + " commloss " + std::to_string(cl0) + " -> " + std::to_string(after.commloss));
        con.Cmd("clear");
    }
    {
        Link lk(ip, 20);
        lk.SetFlags(C_CONTROL);
        lk.Open();
        Sleep(200);
        lk.SetFlags(C_CONTROL | C_ENABLE);
        WaitStatus(lk, enabled, 3);
        lk.StopIo();
        Sleep(600);
        auto st = ConsoleStatus(con);
        lk.Close();
        r.Check("lost while enabled but idle: disabled, no fault", st["enabled"] == "0" && st["fault"] == "none", "enabled=" + st["enabled"] + " fault=" + st["fault"]);
    }

    printf("== reconnect, release control\n");
    Link lk(ip, 20);
    lk.SetFlags(C_CONTROL | C_ENABLE | C_START);  // the PLC program comes back with its bits set
    lk.Open();
    Sleep(800);
    s = CurrentStatus(lk);
    r.Check("reconnecting with ENABLE and START held starts nothing", !(s.flags & F_ENABLED) && s.crank == 0, FormatStatus(s));
    lk.SetFlags(C_CONTROL);
    Sleep(100);
    lk.SetFlags(C_CONTROL | C_ENABLE);
    WaitStatus(lk, enabled, 3);
    Pulse(lk, C_START);
    WaitStatus(lk, crank(2), 3);
    lk.SetFlags(C_ENABLE);  // release control, keep ENABLE
    WaitStatus(lk, [](const Sts &x) { return !(x.flags & F_PLC); }, 1);
    Sleep(300);
    s = CurrentStatus(lk);
    const std::string rp = ConsoleErr(con.Cmd("rpm 30"));
    const Sts s2 = WaitStatus(lk, [](const Sts &x) { return std::abs(x.rpm - 3000) < 5; }, 3);
    r.Check("PLC_CONTROL released while running: the motion goes on, the console takes over (rpm 30)",
            !(s.flags & F_PLC) && s.crank == 2 && rp.empty() && std::abs(s2.rpm - 3000) < 5,
            std::string(Name(kCrankStates, 5, s.crank)) + ", rpm reply '" + rp + "', " + F2(s2.rpm / 100.0) + " rpm");
    con.Cmd("stop");
    WaitStatus(lk, crank(0), 3);
    lk.StopIo();  // lost without control: not COMM_LOSS (ENABLE drops: disabled)
    Sleep(600);
    auto st = ConsoleStatus(con);
    lk.Close();
    r.Check("lost without control: no COMM_LOSS; its held ENABLE dropping disables", st["fault"] == "none" && st["enabled"] == "0",
            "fault=" + st["fault"] + " enabled=" + st["enabled"]);
    con.Cmd("rpm 20");
}

}  // namespace

int SuiteStatus(const std::string &ip, const std::string &restore, const std::string &workbench, const std::string &project) {
    return RunSuite(ip, restore, [&](Console &con, Results &r) { Status(ip, con, r, workbench, project); });
}

int SuiteCmd(const std::string &ip, const std::string &restore) {
    return RunSuite(ip, restore, [&](Console &con, Results &r) { Cmd(ip, con, r); });
}

}  // namespace bench
