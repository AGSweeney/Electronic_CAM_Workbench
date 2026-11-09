// MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
// Phase 4 (TRIGGER, setpoints, jog), the load test, the EDS against the board, and the Workbench's Board panel with a PLC.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <sstream>
#include <thread>

#include "suites.h"

namespace bench {

namespace {

const char *const kEipTrigger =
    "master src=encoder cpr=4000 den=1 rpm=20 crank=0 ramp=60 max=600\n"
    "timeline repeat=0 ovr=1 trig=eip in=0 rise=1 bit=0\n"
    "axis 0 master=timeline m=1 cpu=800 start=0 closed=1 span=0.5 units=mm name=Slide\n"
    "seg 0 0.25 5 cycloidal\n"
    "seg 0 0.5 0 cycloidal\n";

const char *const kLine =
    "master src=encoder cpr=4000 den=1 rpm=20 crank=0 ramp=60 max=600 line=40 kerf=2\n"
    "timeline repeat=1 ovr=1 trig=cmd in=0 rise=1 bit=0\n"
    "axis 0 master=crank m=1 cpu=800 start=0 closed=1 span=360 units=mm name=Knife\n"
    "seg 0 90 0 dwell\n"
    "seg 0 270 10 cycloidal\n"
    "seg 0 360 0 cycloidal\n";

std::string F(double v, const char *fmt = "%.2f") {
    char t[48];
    snprintf(t, sizeof(t), fmt, v);
    return t;
}

void Motion(const std::string &ip, Console &con, Results &r) {
    for (const char *c : {"abort", "disable", "sim 0", "clear"}) {
        con.Cmd(c);
    }
    auto rejected = [](const Sts &x) { return x.reject != 0; };
    auto crank = [](uint16_t w) { return [w](const Sts &x) { return x.crank == w; }; };
    auto rpmIs = [](int w) { return [w](const Sts &x) { return std::abs(x.rpm - w) < 5; }; };
    auto ovrIs = [](int w) { return [w](const Sts &x) { return x.ovr == w; }; };
    auto state = [](uint16_t w) { return [w](const Sts &x) { return x.state == w; }; };
    auto notState5 = [](const Sts &x) { return x.state != 5; };

    printf("== TRIGGER (profile trigger EtherNet/IP, one-shot 0.5 s timeline)\n");
    Upload(con, kEipTrigger);
    const std::string prof = Find(con.Cmd("profile"), "profile");
    r.Check("profile shows the trigger as the TRIGGER bit", prof.find("ethernet/ip (TRIGGER bit)") != std::string::npos, prof);
    auto lk = FreshLink(ip);
    Pulse(*lk, C_TRIGGER);
    Sts s = WaitStatus(*lk, rejected, 1);
    r.Check("TRIGGER without PLC_CONTROL: RejectCode 11", s.reject == 11, "reject " + std::to_string(s.reject));
    TakeControl(*lk, C_SIM);
    Pulse(*lk, C_START);
    s = WaitStatus(*lk, [](const Sts &x) { return x.tl == 1; }, 2);
    r.Check("SIMULATE + START: timeline armed", s.tl == 1, Name(kTlStates, 6, s.tl));
    std::vector<double> lat;
    for (int i = 0; i < 20; i++) {
        WaitStatus(*lk, [](const Sts &x) { return x.tl == 1 || x.tl == 4; }, 2);
        lk->ClearTxLog();
        lk->SetFlags(C_CONTROL | C_SIM | C_TRIGGER);
        s = WaitStatus(*lk, [](const Sts &x) { return x.tl == 2; }, 1);
        const double rx = lk->StatusTime();
        double sent = 0;
        for (const auto &e : lk->TxLog()) {
            if ((e.second & C_TRIGGER) && sent == 0) {
                sent = e.first;
            }
        }
        if (s.tl == 2 && sent != 0) {
            lat.push_back((rx - s.tUs / 1e6 - sent) * 1000);
        }
        lk->SetFlags(C_CONTROL | C_SIM);
        WaitStatus(*lk, [](const Sts &x) { return x.tl == 4; }, 2);
        Sleep(50);
    }
    r.Check("TRIGGER starts the timeline from t = 0, 20 times (one-shot: again from done)", lat.size() == 20, std::to_string(lat.size()) + " of 20");
    if (!lat.empty()) {
        double sum = 0;
        for (double v : lat) {
            sum += v;
        }
        const double lo = *std::min_element(lat.begin(), lat.end()), hi = *std::max_element(lat.begin(), lat.end());
        printf("    latency, the PC sending the packet -> the timeline at t = 0: min %.2f  mean %.2f  max %.2f ms\n", lo, sum / lat.size(), hi);
        r.Check("TRIGGER latency from the packet: under 3 ms", hi < 3.0, "max " + F(hi) + " ms");
    }
    lk->Close();
    Sleep(300);
    Upload(con, kBenchDriven);
    lk = FreshLink(ip, C_CONTROL);
    Pulse(*lk, C_TRIGGER);
    s = WaitStatus(*lk, rejected, 1);
    r.Check("TRIGGER on a profile without an EtherNet/IP trigger: RejectCode 21", s.reject == 21, "reject " + std::to_string(s.reject));

    printf("== CrankRpm (driven crank, simulate)\n");
    lk->SetFlags(C_CONTROL | C_SIM);
    Sleep(100);
    Pulse(*lk, C_START);
    s = WaitStatus(*lk, [](const Sts &x) { return x.crank == 2 && std::abs(x.rpm - 2000) < 5; }, 3);
    r.Check("running at the profile's 20 rpm (CrankRpm 0)", std::abs(s.rpm - 2000) < 5, F(s.rpm / 100.0));
    lk->SetValue(4, 3500);
    s = WaitStatus(*lk, rpmIs(3500), 3);
    r.Check("CrankRpm 3500: 35.00 rpm", std::abs(s.rpm - 3500) < 5 && s.reject == 0, F(s.rpm / 100.0));
    lk->SetValue(4, 70000);
    WaitStatus(*lk, rejected, 1);
    Sleep(300);
    s = CurrentStatus(*lk);
    r.Check("CrankRpm 700 (over the 600 max): RejectCode 18, speed unchanged", s.reject == 18 && std::abs(s.rpm - 3500) < 5,
            "reject " + std::to_string(s.reject) + ", " + F(s.rpm / 100.0) + " rpm");
    lk->SetValue(4, 0);
    s = WaitStatus(*lk, rpmIs(2000), 3);
    r.Check("CrankRpm back to 0: the profile's 20 rpm, RejectCode cleared", std::abs(s.rpm - 2000) < 5 && s.reject == 0, F(s.rpm / 100.0));
    lk->SetFlags(C_SIM);  // release control, keep simulate
    Sleep(150);
    lk->SetValue(4, 2500);
    s = WaitStatus(*lk, rejected, 1);
    r.Check("CrankRpm changed without PLC_CONTROL: RejectCode 11, not applied", s.reject == 11 && std::abs(s.rpm - 2000) < 5,
            "reject " + std::to_string(s.reject) + ", " + F(s.rpm / 100.0) + " rpm");
    lk->SetFlags(C_CONTROL | C_SIM);
    s = WaitStatus(*lk, rpmIs(2500), 3);
    r.Check("taking control applies the non-zero CrankRpm (25 rpm)", std::abs(s.rpm - 2500) < 5, F(s.rpm / 100.0));
    lk->SetValue(4, 0);
    lk->SetValue(12, 150000);
    s = WaitStatus(*lk, rejected, 1);
    r.Check("CutLength on a profile without cut-length mode: RejectCode 16", s.reject == 16, "reject " + std::to_string(s.reject));
    lk->SetValue(12, 0);
    Pulse(*lk, C_STOP);
    WaitStatus(*lk, crank(0), 3);

    printf("== JOG (driven crank, on the motors)\n");
    lk->SetFlags(C_CONTROL);  // simulate off
    Sleep(150);
    Pulse(*lk, C_JOG);
    s = WaitStatus(*lk, rejected, 1);
    r.Check("JOG with the motors disabled: RejectCode 12", s.reject == 12, "reject " + std::to_string(s.reject));
    lk->SetFlags(C_CONTROL | C_ENABLE);
    WaitStatus(*lk, [](const Sts &x) { return (x.flags & F_ENABLED) != 0; }, 3);
    s = CurrentStatus(*lk);
    const int64_t here = CrankPos(s);
    lk->SetValue(16, static_cast<int32_t>(here));  // the target where the crank is, then JOG
    Sleep(50);
    lk->SetFlags(C_CONTROL | C_ENABLE | C_JOG);
    s = WaitStatus(*lk, state(5), 2);
    Sleep(300);
    Sts s2 = CurrentStatus(*lk);
    r.Check("JOG on: state jogging, the crank stays where it was", s.state == 5 && std::llabs(CrankPos(s2) - here) < 500,
            std::string(Name(kEngineStates, 7, s.state)) + ", " + F(here / 1000.0) + " -> " + F(CrankPos(s2) / 1000.0) + " deg");
    for (int64_t target : {here + 90000, here + 450000, here - 30000}) {
        lk->SetValue(16, static_cast<int32_t>(target));
        WaitStatus(*lk, [target](const Sts &x) { return std::llabs(CrankPos(x) - target) < 200; }, 8);
        Sleep(200);
        s = CurrentStatus(*lk);
        r.Check("JogCrankTarget " + F(target / 1000.0, "%.1f") + " deg: the crank (and M-1) goes there",
                std::llabs(CrankPos(s) - target) < 200 && s.state == 5, "at " + F(CrankPos(s) / 1000.0) + " deg, a0 " + std::to_string(s.axis[0].counts));
    }
    lk->SetFlags(C_CONTROL | C_ENABLE);
    s = WaitStatus(*lk, notState5, 3);
    r.Check("JOG dropped: comes to rest and releases (state ready)", s.state == 2, Name(kEngineStates, 7, s.state));
    s = CurrentStatus(*lk);
    lk->SetValue(16, static_cast<int32_t>(CrankPos(s)));
    Sleep(50);
    lk->SetFlags(C_CONTROL | C_ENABLE | C_JOG);
    WaitStatus(*lk, state(5), 2);
    lk->SetValue(16, static_cast<int32_t>(CrankPos(s) + 720000));
    Sleep(400);
    const uint32_t cl0 = CurrentStatus(*lk).commloss;
    lk->StopIo();  // the cable pulled mid-jog
    const auto cranks = WatchConsole(con, 3.0, "crank");
    auto c = ConsoleStatus(con);
    lk->Close();
    Sleep(300);
    {
        auto rd = FreshLink(ip);
        s = CurrentStatus(*rd);
    }
    Sleep(300);
    r.Check("connection lost mid-jog: comes to rest, then disabled, COMM_LOSS (FaultCode 4), CommLossCount + 1",
            cranks.back() == "off" && c["enabled"] == "0" && s.fault == 4 && s.commloss == cl0 + 1,
            "enabled=" + c["enabled"] + " fault " + std::to_string(s.fault) + " commloss " + std::to_string(cl0) + " -> " + std::to_string(s.commloss));
    con.Cmd("clear");

    printf("== TimelineOverride, JogTimeTarget (encoder master + timeline, simulate)\n");
    Upload(con, kBenchTimeline);
    lk = FreshLink(ip, C_CONTROL);
    TakeControl(*lk, C_SIM);  // SIMULATE raised after connecting (held at connect is no edge)
    Pulse(*lk, C_START);
    // trig=cmd: the console triggers it, and may not while the PLC has control: release it meanwhile.
    lk->SetFlags(C_SIM);
    Sleep(150);
    con.Cmd("trigger");
    lk->SetFlags(C_CONTROL | C_SIM);
    s = WaitStatus(*lk, [](const Sts &x) { return x.tl == 2; }, 2);
    r.Check("timeline running at the profile's 100 %", s.tl == 2 && s.ovr == 10000, std::string(Name(kTlStates, 6, s.tl)) + " " + std::to_string(s.ovr));
    lk->SetValue(8, 2500);
    s = WaitStatus(*lk, ovrIs(2500), 3);
    r.Check("TimelineOverride 2500: 25.00 % (ramped)", s.ovr == 2500, std::to_string(s.ovr));
    lk->SetValue(8, 200000);
    s = WaitStatus(*lk, rejected, 1);
    r.Check("TimelineOverride 2000 %: RejectCode 18, unchanged", s.reject == 18 && s.ovr == 2500, "reject " + std::to_string(s.reject));
    lk->SetValue(8, 0);
    s = WaitStatus(*lk, ovrIs(10000), 3);
    r.Check("TimelineOverride 0: the profile's 100 %", s.ovr == 10000, std::to_string(s.ovr));
    lk->SetValue(4, 3000);
    s = WaitStatus(*lk, rejected, 1);
    r.Check("CrankRpm on an encoder master: RejectCode 15", s.reject == 15, "reject " + std::to_string(s.reject));
    lk->SetValue(4, 0);
    Pulse(*lk, C_STOP);
    WaitStatus(*lk, [](const Sts &x) { return (x.tl == 0 || x.tl == 4) && x.crank == 0; }, 4);
    lk->SetValue(20, CurrentStatus(*lk).tUs);
    Sleep(50);
    lk->SetFlags(C_CONTROL | C_SIM | C_JOG);
    WaitStatus(*lk, state(5), 2);
    bool ok = true;
    for (int t : {500000, 1500000, 250000}) {
        lk->SetValue(20, t);
        s = WaitStatus(*lk, [t](const Sts &x) { return std::abs(x.tUs - t) < 2000; }, 5);
        ok = ok && std::abs(s.tUs - t) < 2000;
        printf("    JogTimeTarget %.2f s -> timeline %.4f s, a1 %d\n", t / 1e6, s.tUs / 1e6, s.axis[1].counts);
    }
    r.Check("JogTimeTarget: the timeline goes to 0.5, 1.5, 0.25 s", ok);
    lk->SetFlags(C_CONTROL | C_SIM);
    s = WaitStatus(*lk, notState5, 3);
    r.Check("JOG dropped: released", s.state != 5, Name(kEngineStates, 7, s.state));
    lk->Close();
    Sleep(300);

    printf("== CutLength (cut-length mode, simulated line)\n");
    Upload(con, kLine);
    lk = FreshLink(ip, C_CONTROL);
    TakeControl(*lk, C_SIM);
    s = CurrentStatus(*lk);
    r.Check("cut-length profile: CutLength 98 mm (100 mm cycle less the 2 mm kerf), CutLengthMin 73 mm", s.len == 98000 && s.lmin == 73000,
            "len " + std::to_string(s.len) + " min " + std::to_string(s.lmin));
    lk->SetValue(12, 150000);
    s = WaitStatus(*lk, [](const Sts &x) { return x.len == 150000; }, 2);
    r.Check("CutLength 150 mm while stopped: at once", s.len == 150000 && s.next == 150000, "len " + std::to_string(s.len) + " next " + std::to_string(s.next));
    Pulse(*lk, C_START);
    lk->SetFlags(C_SIM);
    Sleep(150);
    con.Cmd("simenc 8000");  // 200 mm/s of line
    lk->SetFlags(C_CONTROL | C_SIM);
    Sleep(500);
    lk->SetValue(12, 120000);
    s = WaitStatus(*lk, [](const Sts &x) { return x.next == 120000; }, 2);
    r.Check("CutLength 120 mm while running: next 120, now still 150", s.next == 120000 && s.len == 150000, "len " + std::to_string(s.len) + " next " + std::to_string(s.next));
    s = WaitStatus(*lk, [](const Sts &x) { return x.len == 120000; }, 3);
    r.Check("... from the next cut on", s.len == 120000, "len " + std::to_string(s.len));
    lk->SetValue(12, 50000);
    s = WaitStatus(*lk, rejected, 1);
    r.Check("CutLength 50 mm (under the 73 mm minimum): RejectCode 17, unchanged", s.reject == 17 && s.next == 120000,
            "reject " + std::to_string(s.reject) + ", next " + std::to_string(s.next));
    lk->SetValue(12, 0);
    Sleep(200);
    r.Check("CutLength 0: keeps 120 mm", CurrentStatus(*lk).next == 120000);
    Pulse(*lk, C_STOP);
    WaitStatus(*lk, crank(0), 3);
    lk->Close();
    Sleep(300);

    printf("== STOP on a timeline (repeat, 2 s cycle, simulate)\n");
    Upload(con, kBenchTimeline);
    lk = FreshLink(ip, C_CONTROL);
    TakeControl(*lk, C_SIM);
    // trig=cmd: the console triggers it, with the PLC's control released meanwhile.
    auto runIt = [&]() {
        Pulse(*lk, C_START);
        lk->SetFlags(C_SIM);
        Sleep(150);
        con.Cmd("trigger");
        lk->SetFlags(C_CONTROL | C_SIM);
        return WaitStatus(*lk, [](const Sts &x) { return x.tl == 2; }, 2);
    };
    const uint16_t kStopping = 1u << 7;
    s = runIt();
    Sleep(400);
    Pulse(*lk, C_STOP);
    s = WaitStatus(*lk, [&](const Sts &x) { return (x.flags & kStopping) != 0; }, 1);
    r.Check("STOP: the timeline finishes its cycle (still running, STOPPING set)", s.tl == 2 && (s.flags & kStopping),
            std::string(Name(kTlStates, 6, s.tl)) + " flags " + Hex(s.flags, 4));
    s = WaitStatus(*lk, [](const Sts &x) { return x.tl == 0; }, 3);
    r.Check("... and rests at the cycle's end: idle at t = 2 s", s.tl == 0 && s.tUs == 2000000 && !(s.flags & kStopping),
            std::string(Name(kTlStates, 6, s.tl)) + " t " + std::to_string(s.tUs) + " us");
    Pulse(*lk, C_START);
    s = WaitStatus(*lk, [](const Sts &x) { return x.tl == 1 || x.reject != 0; }, 2);
    r.Check("START after it: armed (the axes are at their start)", s.tl == 1 && s.reject == 0,
            std::string(Name(kTlStates, 6, s.tl)) + " reject " + std::to_string(s.reject));
    lk->SetFlags(C_SIM);
    Sleep(150);
    con.Cmd("trigger");
    lk->SetFlags(C_CONTROL | C_SIM);
    WaitStatus(*lk, [](const Sts &x) { return x.tl == 2; }, 2);
    Sleep(400);
    Pulse(*lk, C_STOP);
    Sleep(150);
    Pulse(*lk, C_STOP);
    s = WaitStatus(*lk, [](const Sts &x) { return x.tl == 0; }, 2);
    r.Check("STOP twice: at once, part-way through the cycle", s.tl == 0 && s.tUs > 0 && s.tUs < 1900000,
            std::string(Name(kTlStates, 6, s.tl)) + " t " + std::to_string(s.tUs) + " us");
    const int32_t restUs = s.tUs;
    Pulse(*lk, C_START);
    s = WaitStatus(*lk, rejected, 1);
    r.Check("START part-way: RejectCode 22 (off its cam position), nothing moves", s.reject == 22 && s.tl == 0 && s.tUs == restUs,
            "reject " + std::to_string(s.reject) + " " + Name(kTlStates, 6, s.tl));
    lk->SetValue(20, s.tUs);  // JogTimeTarget where the timeline is, then JOG, then back to 0
    Sleep(50);
    lk->SetFlags(C_CONTROL | C_SIM | C_JOG);
    WaitStatus(*lk, [](const Sts &x) { return x.tl == 5; }, 2);
    lk->SetValue(20, 0);
    s = WaitStatus(*lk, [](const Sts &x) { return x.tl == 5 && x.tUs == 0; }, 4);
    r.Check("JOG the timeline to 0", s.tl == 5 && s.tUs == 0, std::string(Name(kTlStates, 6, s.tl)) + " t " + std::to_string(s.tUs));
    lk->SetFlags(C_CONTROL | C_SIM);
    WaitStatus(*lk, [](const Sts &x) { return x.tl == 0; }, 2);
    Pulse(*lk, C_START);
    s = WaitStatus(*lk, [](const Sts &x) { return x.tl == 1 || x.reject != 0; }, 2);
    r.Check("... then START: armed", s.tl == 1 && s.reject == 0, std::string(Name(kTlStates, 6, s.tl)) + " reject " + std::to_string(s.reject));
    Pulse(*lk, C_STOP);
    WaitStatus(*lk, [](const Sts &x) { return x.tl == 0; }, 2);
    lk->Close();
    Sleep(300);

    // A limit on the motors (M-0 the driven crank, M-1 the Slide, free shafts): an open profile, 10 mm a revolution at
    // 20 rpm, so the Slide moves + all the time at about 200 rpm. The limit trips with limit_test (nothing wired).
    printf("== Limits (driven crank on the motors, 20 rpm)\n");
    Upload(con,
           "master src=driven cpr=4000 den=1 rpm=20 crank=0 ramp=60 max=600\n"
           "timeline repeat=1 ovr=1 trig=cmd in=0 rise=1 bit=0\n"
           "axis 0 master=crank m=1 cpu=800 start=0 closed=0 span=360 units=mm name=Slide\n"
           "seg 0 360 10 linear\n"
           "input board 3 limit+ 0 no\n"
           "limits stop=0.1\n");
    con.Cmd("sim 0");  // the sections before leave simulate on, where a limit is only reported
    lk = FreshLink(ip, C_CONTROL);
    TakeControl(*lk, C_ENABLE);
    WaitStatus(*lk, [](const Sts &x) { return (x.flags & (1u << 2)) != 0; }, 3);
    Pulse(*lk, C_START);
    s = WaitStatus(*lk, crank(2), 3);
    r.Check("running", s.crank == 2, Name(kCrankStates, 5, s.crank));
    Sleep(700);
    con.Cmd("limit_test 0 +");
    s = WaitStatus(*lk, [](const Sts &x) { return x.fault == 5 && x.crank == 0; }, 2);
    r.Check("a limit: FaultCode 5, the crank at rest, the motors still enabled", s.fault == 5 && s.crank == 0 && (s.flags & (1u << 2)),
            "fault " + std::to_string(s.fault) + " " + Name(kCrankStates, 5, s.crank) + " flags " + Hex(s.flags, 4));
    r.Check("the Slide's status: kAxLimitPlus", (s.axis[0].flags & 0x80) != 0 && (s.axis[0].flags & 0x100) == 0, Hex(s.axis[0].flags, 4));
    Pulse(*lk, C_CLEAR);
    WaitStatus(*lk, [](const Sts &x) { return x.fault == 0; }, 2);
    Pulse(*lk, C_START);
    s = WaitStatus(*lk, rejected, 1);
    r.Check("START on the limit: RejectCode 24", s.reject == 24 && s.crank == 0, "reject " + std::to_string(s.reject));
    con.Cmd("limit_test 0 off");
    s = WaitStatus(*lk, [](const Sts &x) { return (x.axis[0].flags & 0x180) == 0; }, 2);
    r.Check("off the limit: the flag clears", (s.axis[0].flags & 0x180) == 0, Hex(s.axis[0].flags, 4));
    lk->SetFlags(C_CONTROL);  // ENABLE falls: disabled
    WaitStatus(*lk, [](const Sts &x) { return (x.flags & (1u << 2)) == 0; }, 3);
    lk->Close();
    Sleep(300);

    // Timeline waits (0.10.0), in simulate: nothing moves. A PLC-bit wait (waitbits_test stands in for the PLC's bits:
    // the PLC path writes only ControlFlags), then a board-input wait with a timeout on an input that reads low.
    {
        printf("== Timeline waits (simulate, no motion)\n");
        int low = -1;  // an input reading low now: its wait for "on" holds
        {
            const auto st = con.Cmd("status");
            unsigned din = 0;
            for (const std::string &l : st) {
                const size_t at = l.find(" din=");
                if (at != std::string::npos) din = static_cast<unsigned>(strtoul(l.c_str() + at + 5, nullptr, 16));
            }
            for (int i = 2; i < 13 && low < 0; ++i) {
                if (!((din >> i) & 1u)) low = i;
            }
        }
        r.Check("an input reads low for the timeout test", low >= 0, "din");
        Upload(con,
               std::string("master src=encoder cpr=4000 den=1 rpm=30 crank=0 ramp=60 max=600\n"
                           "timeline repeat=0 ovr=1 trig=cmd in=0 rise=1 bit=0\n"
                           "axis 0 master=timeline m=1 cpu=800 start=0 closed=1 span=2 units=mm name=Slide\n"
                           "seg 0 0.6 10 cycloidal\n"
                           "seg 0 1 10 dwell\n"
                           "seg 0 1.6 0 cycloidal\n"
                           "seg 0 2 0 dwell\n"
                           "wait 0.8 plc 3 on\n"
                           "wait 1.8 board ") + std::to_string(low < 0 ? 2 : low) + " on timeout=0.3\n");
        for (const char *c : {"abort", "disable", "clear", "sim 1", "waitbits_test 0"}) con.Cmd(c);
        lk = FreshLink(ip, 0);
        con.Cmd("start");
        con.Cmd("trigger");
        s = WaitStatus(*lk, [](const Sts &x) { return x.tl == 6; }, 3);
        r.Check("the PLC-bit wait holds: TimelineState 6, StatusFlags 14, WaitIndex 1", s.tl == 6 && (s.flags & (1u << 14)) && s.waitIndex == 1,
                "tl " + std::to_string(s.tl) + " flags " + Hex(s.flags, 4) + " index " + std::to_string(s.waitIndex));
        Sleep(600);
        s = WaitStatus(*lk, [](const Sts &x) { return x.waitedMs >= 500; }, 2);
        r.Check("WaitedMs counts (no timeout: it holds)", s.tl == 6 && s.waitedMs >= 500 && s.fault == 0, std::to_string(s.waitedMs) + " ms");
        con.Cmd("waitbits_test 8");  // bit 3 on
        s = WaitStatus(*lk, [](const Sts &x) { return x.tl != 6; }, 2);
        r.Check("the bit on: it runs on, WaitIndex 0", s.tl == 2 && s.waitIndex == 0, "tl " + std::to_string(s.tl));
        s = WaitStatus(*lk, [](const Sts &x) { return x.fault == 7; }, 3);
        r.Check("the input wait times out after 0.3 s: FaultCode 7, the timeline at rest", s.fault == 7 && s.tl == 0,
                "fault " + std::to_string(s.fault) + " tl " + std::to_string(s.tl));
        for (const char *c : {"clear", "waitbits_test off", "sim 0"}) con.Cmd(c);
        lk->Close();
        Sleep(300);
    }

    // Homing (0.9.1) on the Slide (M-1, free shaft): required, toward -, 10 mm/s; the switch is home_test's, 6 mm away.
    printf("== Homing (HOME, M-1)\n");
    Upload(con,
           "master src=driven cpr=4000 den=1 rpm=20 crank=0 ramp=60 max=600\n"
           "timeline repeat=1 ovr=1 trig=cmd in=0 rise=1 bit=0\n"
           "axis 0 master=crank m=1 cpu=800 start=0 closed=1 span=360 units=mm name=Slide\n"
           "seg 0 120 10 cycloidal\n"
           "seg 0 360 0 cycloidal\n"
           "input board 3 home 0 no\n"
           "home 0 dir=- fast=10 slow=1 back=2 at=0 then=start order=1 search=50 req=1\n");
    con.Cmd("sim 0");
    lk = FreshLink(ip, C_CONTROL);
    TakeControl(*lk, C_ENABLE);
    WaitStatus(*lk, [](const Sts &x) { return (x.flags & (1u << 2)) != 0; }, 3);
    Pulse(*lk, C_START);
    s = WaitStatus(*lk, rejected, 1);
    r.Check("START before homing: RejectCode 23", s.reject == 23 && s.crank == 0, "reject " + std::to_string(s.reject));
    r.Check("not homed: AxisFlags 9 clear", (s.axis[0].flags & 0x200) == 0, Hex(s.axis[0].flags, 4));
    con.Cmd("home_test 0 -6");
    lk->SetFlags(C_CONTROL | C_ENABLE | C_HOME);
    s = WaitStatus(*lk, [](const Sts &x) { return (x.axis[0].flags & 0x400) != 0; }, 1);
    r.Check("HOME: homing (AxisFlags 10)", (s.axis[0].flags & 0x400) != 0, Hex(s.axis[0].flags, 4));
    lk->SetFlags(C_CONTROL | C_ENABLE);
    s = WaitStatus(*lk, [](const Sts &x) { return (x.axis[0].flags & 0x600) == 0x200; }, 15);
    r.Check("homed (AxisFlags 9), at the cam start", (s.axis[0].flags & 0x600) == 0x200 && s.axis[0].counts == 0 && s.fault == 0,
            Hex(s.axis[0].flags, 4) + " counts " + std::to_string(s.axis[0].counts));
    Pulse(*lk, C_START);
    s = WaitStatus(*lk, crank(2), 3);
    r.Check("START after homing: running", s.crank == 2 && s.reject == 0, Name(kCrankStates, 5, s.crank));
    Pulse(*lk, C_STOP);
    WaitStatus(*lk, [](const Sts &x) { return x.crank == 0; }, 5);
    // A failure: no switch, a search of 50 mm runs out: FaultCode 6, the motors enabled.
    con.Cmd("home_test 0 off");
    Pulse(*lk, C_HOME);
    s = WaitStatus(*lk, [](const Sts &x) { return x.fault == 6; }, 15);
    r.Check("HOME with no switch: FaultCode 6, the motors still enabled", s.fault == 6 && (s.flags & (1u << 2)) && (s.axis[0].flags & 0x600) == 0,
            "fault " + std::to_string(s.fault) + " flags " + Hex(s.flags, 4) + " axis " + Hex(s.axis[0].flags, 4));
    Pulse(*lk, C_CLEAR);
    lk->SetFlags(C_CONTROL);  // ENABLE falls: disabled
    WaitStatus(*lk, [](const Sts &x) { return (x.flags & (1u << 2)) == 0; }, 3);
    lk->Close();
    Sleep(300);
}

/** A stress test, not a gate: telemetry at the Workbench's 50 ms and pings alongside a 20 ms connection, simulated. */
void Load(const std::string &ip, Console &con, Results &r, double minutes) {
    for (const char *c : {"abort", "disable", "sim 0", "clear"}) {
        con.Cmd(c);
    }
    Upload(con, kBenchTimeline);
    con.Cmd("sim 1");
    con.Cmd("simenc 4000");
    con.Cmd("start");
    con.Cmd("trigger");
    const auto d0 = con.Cmd("diag");
    for (size_t i = 0; i + 1 < d0.size(); i++) {
        printf("    %s\n", d0[i].c_str());
    }
    const std::string samp0 = Find(d0, "sample");
    const long long late0 = IntAfter(samp0, "late samples "), carry0 = IntAfter(samp0, "carried-step samples ");
    auto lk = FreshLink(ip, C_CONTROL | C_SIM);  // sim is on already (console): held bits are fine here
    const uint32_t cl0 = CurrentStatus(*lk).commloss;
    con.Send("tel 50\n");
    const double t0 = PreciseSeconds();
    double last = t0, next = t0 + 300, worstRx = 1e9, worstTel = 1e9, rxRate = 0, telRate = 0;
    uint64_t lastRx = lk->RxCount(), lastTel = con.TelLines();
    int pings = 0, pongs = 0;
    std::vector<long long> loops;
    while (PreciseSeconds() - t0 < minutes * 60) {
        pings++;
        pongs += !Discover(0.5).empty();
        con.Drain(500);
        const double now = PreciseSeconds();
        if (now - last >= 10) {
            rxRate = (lk->RxCount() - lastRx) / (now - last);
            telRate = (con.TelLines() - lastTel) / (now - last);
            worstRx = std::min(worstRx, rxRate);
            worstTel = std::min(worstTel, telRate);
            lastRx = lk->RxCount();
            lastTel = con.TelLines();
            last = now;
        }
        if (now >= next) {
            next += 300;
            const auto d = con.Cmd("diag");
            const std::string loop = Find(d, "main loop");
            loops.push_back(IntAfter(loop, "longest "));
            printf("  %5.1f min  EIP %.1f/s  TEL %.1f/s  pings %d/%d  %s\n", (now - t0) / 60, rxRate, telRate, pongs, pings, loop.c_str());
            fflush(stdout);
        }
    }
    const Sts s = CurrentStatus(*lk);
    lk->Close();
    con.Close();
    con.Open(ip.c_str());
    con.Cmd("tel 0");
    const auto d1 = con.Cmd("diag");
    const std::string samp = Find(d1, "sample"), lw = Find(d1, "lwip");
    if (loops.empty()) {
        loops.push_back(IntAfter(Find(d1, "main loop"), "longest "));
    }
    const long long late1 = IntAfter(samp, "late samples "), carry1 = IntAfter(samp, "carried-step samples ");
    r.Check(F(minutes, "%.1f") + " min: the EtherNet/IP stream never fell below 47.5/s (10 s windows)", worstRx > 47.5, "worst " + F(worstRx, "%.1f") + "/s");
    r.Check("telemetry never fell below 19/s", worstTel > 19, "worst " + F(worstTel, "%.1f") + "/s");
    r.Check("every discovery ping answered", pongs == pings, std::to_string(pongs) + "/" + std::to_string(pings));
    r.Check("no connection losses, the PLC still in control, the machine still running", s.commloss == cl0 && (s.flags & F_PLC) && s.tl == 2, FormatStatus(s));
    r.Check("no new late samples or carried steps; the sample callback well inside its 200 us",
            late1 == late0 && carry1 == carry0 && IntAfter(samp, "max ") < 100,
            "late " + std::to_string(late0) + " -> " + std::to_string(late1) + ", carried " + std::to_string(carry0) + " -> " + std::to_string(carry1));
    r.Check("the main loop's longest pass stays short", *std::max_element(loops.begin(), loops.end()) < 5000, std::to_string(*std::max_element(loops.begin(), loops.end())) + " us");
    r.Check("lwIP: no heap or pool allocation failures", lw.find("errors 0") != std::string::npos && lw.find("pool errors 0") != std::string::npos, lw);
}

// ---- the EDS -----------------------------------------------------------------------------------------------------

using Section = std::map<std::string, std::vector<std::string>>;

std::string Trim(const std::string &s) {
    const size_t a = s.find_first_not_of(" \t\r\n");
    return a == std::string::npos ? "" : s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
}

std::string Unquote(const std::string &s) {
    return s.size() >= 2 && s.front() == '"' && s.back() == '"' ? s.substr(1, s.size() - 2) : s;
}

/** $ comments dropped, entries end at ';' outside quotes, fields split on ','. */
std::map<std::string, Section> ParseEds(const std::string &text) {
    std::string body;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        bool q = false;
        for (char ch : line) {
            if (ch == '"') q = !q;
            if (ch == '$' && !q) break;
            body += ch;
        }
        body += '\n';
    }
    std::map<std::string, Section> sections;
    std::string cur, buf;
    bool q = false, have = false;
    for (size_t i = 0; i < body.size(); i++) {
        const char ch = body[i];
        if (ch == '"') q = !q;
        if (!q && ch == '[' && Trim(buf).empty()) {
            const size_t end = body.find(']', i);
            cur = Trim(body.substr(i + 1, end - i - 1));
            sections[cur];
            have = true;
            i = end;
            continue;
        }
        if (!q && ch == ';') {
            const size_t eq = buf.find('=');
            if (eq != std::string::npos && have) {
                std::vector<std::string> fields;
                std::string f;
                bool qq = false;
                for (char c : buf.substr(eq + 1)) {
                    if (c == '"') qq = !qq;
                    if (c == ',' && !qq) {
                        fields.push_back(Trim(f));
                        f.clear();
                    } else {
                        f += c;
                    }
                }
                fields.push_back(Trim(f));
                sections[cur][Trim(buf.substr(0, eq))] = fields;
            }
            buf.clear();
        } else {
            buf += ch;
        }
    }
    return sections;
}

Bytes Key(int vendor, int type, int product, int major, int minor) {
    return {0x34, 0x04, static_cast<uint8_t>(vendor), static_cast<uint8_t>(vendor >> 8), static_cast<uint8_t>(type), static_cast<uint8_t>(type >> 8),
            static_cast<uint8_t>(product), static_cast<uint8_t>(product >> 8), static_cast<uint8_t>(major | 0x80), static_cast<uint8_t>(minor)};
}

Bytes Cat(Bytes a, const Bytes &b) {
    a.insert(a.end(), b.begin(), b.end());
    return a;
}

void Eds(const std::string &ip, Console &con, const std::string &path, Results &r) {
    auto eds = ParseEds(ReadFile(path));
    auto &dev = eds["Device"];
    auto field = [&](const char *k) { return Unquote(dev[k].empty() ? "" : dev[k][0]); };
    const int vend = atoi(field("VendCode").c_str()), type = atoi(field("ProdType").c_str()), prod = atoi(field("ProdCode").c_str());
    const int maj = atoi(field("MajRev").c_str()), min = atoi(field("MinRev").c_str());
    const std::string name = field("ProdName");
    r.Check("EDS identity = the firmware's (vendor 65000, type 43, product 3, revision, name)",
            vend == 65000 && type == 43 && prod == 3 && maj == kEipRevMajor && min == kEipRevMinor && name == "ECam ClearCore",
            std::to_string(vend) + " " + std::to_string(type) + " " + std::to_string(prod) + " " + std::to_string(maj) + "." + std::to_string(min) + " " + name);
    const Identity me = ListIdentity(ip);
    r.Check("the board's identity matches the EDS (ListIdentity)",
            me.vendor == vend && me.type == type && me.product == prod && me.revMajor == maj && me.revMinor == min && me.name == name);
    auto &asmb = eds["Assembly"];
    auto sizeOf = [&](const char *k) { return asmb[k].size() > 2 ? atoi(asmb[k][2].c_str()) : -1; };
    r.Check("EDS assemblies 100 / 150 / 151 = 160 / 64 / 0 bytes", sizeOf("Assem100") == 160 && sizeOf("Assem150") == 64 && sizeOf("Assem151") == 0);
    const auto &p1 = eds["Params"]["Param1"];
    const int rpiMin = atoi(p1.at(9).c_str()), rpiMax = atoi(p1.at(10).c_str()), rpiDef = atoi(p1.at(11).c_str());
    r.Check("RPI parameter 10..100 ms, default 20 ms", rpiMin == 10000 && rpiMax == 100000 && rpiDef == 20000);
    const Bytes key = Key(vend, type, prod, maj, min);
    for (const auto &e : eds["Connection Manager"]) {
        if (e.first.rfind("Connection", 0) != 0) {
            continue;
        }
        const auto &f = e.second;
        const std::string title = Unquote(f.at(12));
        Bytes pathBytes;
        std::istringstream hex(Unquote(f.at(14)));
        std::string b;
        while (hex >> b) {
            pathBytes.push_back(static_cast<uint8_t>(strtoul(b.c_str(), nullptr, 16)));
        }
        const int o2t = f.at(3).empty() ? 0 : atoi(f.at(3).c_str()), t2o = atoi(f.at(6).c_str());
        const bool inputOnly = (strtoul(f.at(0).c_str(), nullptr, 16) & (1u << 25)) != 0;
        for (bool keyed : {false, true}) {
            Link lk(ip, rpiDef / 1000);
            lk.Connect();
            OpenOptions o;
            o.o2tSize = inputOnly ? 0 : 2 + 4 + o2t;
            o.t2oSize = 2 + t2o;
            o.inputOnly = inputOnly;
            o.connPath = keyed ? Cat(key, pathBytes) : pathBytes;
            const OpenReply rep = lk.ForwardOpen(o);
            uint64_t got = 0;
            if (rep.gen == 0) {
                Sleep(1000);
                got = lk.RxCount();
            }
            r.Check("EDS " + e.first + " \"" + title + "\"" + (keyed ? " with a compatible electronic key" : "") + ": opens, status flows at " +
                        std::to_string(rpiDef / 1000) + " ms",
                    rep.gen == 0 && got > 35, "general " + Hex(rep.gen) + " " + HexList(rep.ext) + ", " + std::to_string(got) + " packets in 1 s");
            lk.Close();
            Sleep(300);
        }
    }
    const Bytes normal = {0x20, 0x04, 0x24, 151, 0x2C, 150, 0x2C, 100};
    {
        Link lk(ip, 20);
        OpenOptions o;
        o.multicast = true;
        o.connPath = Cat(key, normal);
        lk.Open(o);
        auto c = ConsoleStatus(con);
        r.Check("multicast T->O: the board names the PLC by its own address, not the group", c["eip_from"] == LocalIp(), "eip_from=" + c["eip_from"]);
    }
    Sleep(300);
    Link lk(ip, 20);
    lk.Connect();
    OpenOptions o;
    o.connPath = Cat(Key(65000, 43, 3, kEipRevMajor + 1, 0), normal);
    const OpenReply rep = lk.ForwardOpen(o);
    r.Check("a key for another major revision is refused (0x01 / 0x0116)", rep.gen == 0x01 && !rep.ext.empty() && rep.ext[0] == 0x0116, Hex(rep.gen) + " " + HexList(rep.ext));
}

/** The Workbench's Board panel with a PLC: board_ui_plc.exe checks the window, this plays the PLC. */
void Ui(const std::string &ip, const std::string &exe, const std::string &project, Results &r) {
    std::atomic<bool> ready{false}, inControl{false}, done{false};
    int rc = -1;
    std::string fails;
    std::thread app([&] {
        Exec("\"" + exe + "\" " + ip + " \"" + project + "\"", 240000, &rc, [&](const std::string &l) {
            printf("    app: %s\n", l.c_str());
            fflush(stdout);
            if (l == "ready") ready = true;
            if (l == "in-control") inControl = true;
            if (l.rfind("FAIL", 0) == 0) fails += l + " ";
        });
        done = true;
    });
    auto waitMark = [&](std::atomic<bool> &m, double t) {
        const double t0 = PreciseSeconds();
        while (!m && !done && PreciseSeconds() - t0 < t) {
            Sleep(50);
        }
        return m.load();
    };
    bool ok = waitMark(ready, 40);
    r.Check("the app connected and uploaded", ok);
    std::unique_ptr<Link> lk;
    if (ok) {
        lk = FreshLink(ip);  // reads only
        Sleep(1500);
        lk->SetFlags(C_CONTROL);  // takes control
        ok = waitMark(inControl, 20);
        r.Check("the app checked the panel with the PLC in control", ok);
        Sleep(500);
        lk->SetFlags(0);  // releases it
    }
    app.join();
    lk.reset();
    r.Check("board_ui_plc: every check passed", rc == 0 && fails.empty(), "exit " + std::to_string(rc) + " " + fails);
}

}  // namespace

int SuiteMotion(const std::string &ip, const std::string &restore) {
    return RunSuite(ip, restore, [&](Console &con, Results &r) { Motion(ip, con, r); });
}

int SuiteLoad(const std::string &ip, const std::string &restore, double minutes) {
    return RunSuite(ip, restore, [&](Console &con, Results &r) { Load(ip, con, r, minutes); });
}

int SuiteEds(const std::string &ip, const std::string &path) {
    return RunSuite(ip, "", [&](Console &con, Results &r) { Eds(ip, con, path, r); });
}

int SuiteUi(const std::string &ip, const std::string &exe, const std::string &project) {
    Results r;
    try {
        Ui(ip, exe, project, r);
    } catch (const std::exception &x) {
        r.Check("the suite ran to the end", false, x.what());
    }
    return Summary(r) ? 0 : 1;
}

}  // namespace bench
