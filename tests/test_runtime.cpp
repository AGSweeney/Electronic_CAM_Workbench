/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
// Host tests of the firmware's profile upload format and cam runtime. No Qt, no ClearCore.
// Exit code 0 = all passed.

#include "CamRuntime.h"
#include "OperatorPanel.h"
#include "ProfileText.h"

#include <math.h>
#include <stdio.h>

#include <memory>
#include <string>
#include <vector>

using namespace ecam;

static int g_fail = 0;

static void Check(bool ok, const char *what, double got = 0, double want = 0) {
    if (!ok) {
        ++g_fail;
        printf("FAIL %s (got %.6f, want %.6f)\n", what, got, want);
    }
}

static CamProject Reload(const CamProject &p, uint32_t *hash = nullptr) {
    ProfileReader r;
    r.Begin();
    for (const std::string &l : ProfileLines(p)) {
        const char *err = r.Line(l.c_str());
        if (err) printf("line error: %s in '%s'\n", err, l.c_str());
    }
    CamProject out;
    const char *err = r.Finish(&out);
    if (err) printf("finish error: %s\n", err);
    if (hash) *hash = r.Hash();
    return out;
}

struct Sim {
    CamRuntime rt;
    CamRuntime::Input in;
    int64_t motor[kMotorConnectors] = {};
    void Tick(int n = 1) {
        for (int i = 0; i < n; ++i) {
            int32_t s[kMotorConnectors];
            rt.Tick(in, s);
            for (int c = 0; c < kMotorConnectors; ++c) motor[c] += s[c];
        }
    }
};

static int64_t Counts(const CamAxis &a, double x) { return llround(Evaluate(a, x).pos * a.countsPerUnit); }

int main() {
    // ---- upload format
    {
        CamProject p = DefaultProject();
        p.axes[0].name = "Pusher arm";
        p.axes.push_back(DefaultAxis("Slide"));
        SetAxisMaster(p, 1, AxisMaster::Timeline);
        p.axes[1].connector = 2;
        p.master.source = MasterSource::DrivenCrank;
        p.master.crankConnector = 3;
        p.master.trigger = TriggerSource::Command;
        uint32_t hash = 0;
        const CamProject q = Reload(p, &hash);
        Check(hash == ProfileHash(ProfileLines(p)), "reader hash = writer hash");
        Check(q.axes.size() == 2 && q.axes[0].name == "Pusher arm", "names with spaces survive");
        Check(q.axes[1].master == AxisMaster::Timeline && fabs(q.axes[1].span - p.axes[1].span) < 1e-12,
              "timeline axis survives");
        Check(q.master.source == MasterSource::DrivenCrank && q.master.crankConnector == 3 &&
                  q.master.trigger == TriggerSource::Command,
              "master survives");
        double worst = 0;
        for (int i = 0; i <= 360; ++i) {
            worst = fmax(worst, fabs(Evaluate(q.axes[0], i).pos - Evaluate(p.axes[0], i).pos));
        }
        Check(worst == 0.0, "positions identical after the round trip", worst, 0);

        ProfileReader r;
        r.Begin();
        Check(r.Line("seg 0 10 1 cycloidal") != nullptr, "seg before its axis is refused");
        r.Begin();
        r.Line("master src=encoder cpr=4000 rpm=60 crank=0 ramp=60 max=600");
        r.Line("axis 0 master=crank m=1 cpu=800 start=0 closed=1 span=360 units=mm name=a");
        r.Line("seg 0 100 5 cycloidal");
        CamProject bad;
        Check(r.Finish(&bad) != nullptr, "crank axis not ending at 360 is refused");
        r.Begin();
        r.Line("master src=driven cpr=4000 rpm=60 crank=1 ramp=60 max=600");
        r.Line("axis 0 master=crank m=1 cpu=800 start=0 closed=1 span=360 units=mm name=a");
        r.Line("seg 0 360 0 dwell");
        Check(r.Finish(&bad) != nullptr, "crank motor sharing an axis connector is refused");
    }

    // ---- encoder master
    {
        CamProject p = DefaultProject();  // cycloid 0..20 mm over 120 deg, 800 counts/mm, 4000 counts/rev
        Sim s;
        Check(s.rt.Build(Reload(p)) == nullptr, "build");
        const CamAxis &a = p.axes[0];
        const int m = a.connector;
        s.in.encoder = 0;
        s.rt.RequestStart();
        s.Tick();
        int64_t worst = 0;
        for (int e = 1; e <= 4000 * 2 + 1234; ++e) {  // two revolutions and a bit, 1 count per sample
            s.in.encoder = e;
            s.Tick();
            const int64_t want = Counts(a, e * 360.0 / 4000) - Counts(a, 0);
            const int64_t err = s.motor[m] - want;
            worst = err < 0 ? (-err > worst ? -err : worst) : (err > worst ? err : worst);
        }
        Check(worst <= 1, "encoder: motor within 1 count of the profile over 2+ revs", (double)worst, 1);
        // Backwards through the wrap
        for (int e = 4000 * 2 + 1234; e >= -500; --e) {
            s.in.encoder = e;
            s.Tick();
        }
        const int64_t want = Counts(a, (4000 - 500) * 360.0 / 4000) - Counts(a, 0);
        Check(llabs(s.motor[m] - want) <= 1, "encoder: backwards past 0 tracks", (double)s.motor[m], (double)want);
        // Stop lets go
        s.rt.RequestStop();
        s.Tick();
        const int64_t held = s.motor[m];
        s.in.encoder += 1000;
        s.Tick(10);
        Check(s.motor[m] == held, "encoder: stopped followers do not move");
        // Engaging mid-rev does not jump
        s.rt.RequestStart();
        s.Tick();
        Check(s.motor[m] == held, "encoder: engaging mid-revolution starts without a jump");
        // Encoder counter wrap-around (int32)
        CamProject open = DefaultProject();
        SetClosed(open.axes[0], false);
        SetKnotPos(open.axes[0], KnotCount(open.axes[0]) - 1, 10.0);  // advances 10 mm per rev
        Sim w;
        w.rt.Build(Reload(open));
        w.in.encoder = 2147483647 - 2000;
        w.rt.RequestStart();
        w.Tick();
        for (int i = 0; i < 4000; ++i) {
            w.in.encoder += 1;  // wraps through INT32_MAX
            w.Tick();
        }
        Check(llabs(w.motor[open.axes[0].connector] - 8000) <= 1, "encoder wrap + open profile: +10 mm per rev",
              (double)w.motor[open.axes[0].connector], 8000);
    }

    // ---- a coarse driven crank and a fast follower: 800 counts per cycle at 15 rpm is a new master count
    // every 25 samples. The follower takes the crank's fraction too, so it moves on every sample, not in
    // bursts: at whole counts it took 62 counts at once and then nothing, a 200 Hz rattle at the motor.
    {
        CamProject p = DefaultProject();
        p.master.source = MasterSource::DrivenCrank;
        p.master.crankConnector = 2;
        p.master.crankPpr = 800;
        p.master.rpm = 15.0;
        p.master.crankRampRpmPerSec = 60.0;
        CamAxis &c = p.axes[0];
        c.connector = 0;
        c.countsPerUnit = 80.0;
        c.drive = AxisDrive::Direct;
        c.segments.clear();
        auto seg = [&](double end, double pos, MotionLaw law) {
            CamSegment s;
            s.endDeg = end;
            s.endPos = pos;
            s.law = law;
            c.segments.push_back(s);
        };
        seg(40, 0, MotionLaw::Dwell);
        seg(100, 30, MotionLaw::Blend);
        seg(160, 90, MotionLaw::Linear);
        seg(220, 120, MotionLaw::Blend);
        seg(350, 0, MotionLaw::Poly345);
        seg(360, 0, MotionLaw::Dwell);
        NormalizeProject(p);
        auto held = std::make_unique<Sim>();  // main's frame already holds as many as a 1 MB stack can
        Sim &s = *held;
        Check(s.rt.Build(Reload(p)) == nullptr, "coarse crank: build");
        s.rt.RequestStart();
        s.Tick(5000);
        int64_t worst = 0, idle = 0, longestIdle = 0;
        for (int i = 0; i < 5000 * 8; ++i) {
            const int64_t before = s.motor[0];
            s.Tick(1);
            const int64_t d = llabs(s.motor[0] - before);
            worst = std::max(worst, d);
            RtStatus st;
            s.rt.Status(&st);
            const double deg = (st.crankPhaseQ24 >> 24) * 360.0 / 800.0;
            if (deg > 110 && deg < 150) {  // the ride: 7200 counts/s is 1.44 a sample
                idle = d == 0 ? idle + 1 : 0;
                longestIdle = std::max(longestIdle, idle);
            }
        }
        printf("     coarse crank: most in one sample %lld counts, longest gap in the ride %lld samples\n",
               (long long)worst, (long long)longestIdle);
        Check(worst <= 4, "coarse crank: the follower moves smoothly (peak 2.5 counts a sample)", (double)worst, 4);
        Check(longestIdle <= 1, "coarse crank: no gaps through the ride", (double)longestIdle, 1);
        s.rt.RequestStop();
        s.Tick(5000);
        RtStatus st;
        s.rt.Status(&st);
        const double x = (st.crankPhaseQ24 / 16777216.0) * 360.0 / 800.0;
        Check(llabs(s.motor[0] - llround(Evaluate(p.axes[0], x).pos * 80.0)) <= 1, "coarse crank: stops on the profile");
    }

    // ---- cut-length mode on a driven crank: the crank motor feeds the line, so its counts are the line.
    // The motor turns steadily through every cut, whatever the length, and the carriage rides it.
    {
        CamProject p = DefaultProject();
        p.master.source = MasterSource::DrivenCrank;
        p.master.lineMode = true;
        p.master.crankConnector = 3;
        p.master.crankPpr = 4000;
        p.master.wheelCircumference = 200.0;  // 200 mm of line per motor rev: 20 counts/mm
        p.master.designLength = 600.0;        // 12000 counts per cut as drawn
        p.master.kerf = 3.0;
        p.master.rpm = 10.0;                  // cuts/min as drawn: 2000 counts/s = 0.4 counts/sample
        p.master.crankRampRpmPerSec = 120.0;
        p.master.crankMaxRpm = 100.0;
        CamAxis &c = p.axes[0];
        c.connector = 0;
        c.countsPerUnit = 100.0;
        c.drive = AxisDrive::Direct;
        c.segments.clear();
        auto seg = [&](double end, double pos, MotionLaw law) {
            CamSegment s;
            s.endDeg = end;
            s.endPos = pos;
            s.law = law;
            c.segments.push_back(s);
        };
        seg(168, 0, MotionLaw::Dwell);
        seg(204, 30, MotionLaw::Blend);
        seg(240, 90, MotionLaw::Linear);
        seg(276, 120, MotionLaw::Blend);
        seg(354, 0, MotionLaw::Poly345);
        seg(360, 0, MotionLaw::Dwell);
        NormalizeProject(p);
        Check(p.master.lineMode && p.master.countsPerRev == 12000.0, "driven line: survives NormalizeProject");
        const CamProject r = Reload(p);
        Check(r.master.source == MasterSource::DrivenCrank && r.master.lineMode && r.master.lineCountsPerMm == 20.0 &&
                  r.master.countsPerRev == 12000.0 && r.master.kerf == 3.0,
              "driven line: round-trips through the profile text");

        auto held = std::make_unique<Sim>();
        Sim &s = *held;
        Check(s.rt.Build(r) == nullptr && s.rt.LineMode() && s.rt.DrivenCrank(), "driven line: build");
        s.rt.RequestStart();
        s.Tick(5000);
        int64_t worst = 0;
        auto run = [&](int n) {  // the most the motor moves in any one sample
            for (int i = 0; i < n; ++i) {
                const int64_t before = s.motor[3];
                s.Tick(1);
                worst = std::max<int64_t>(worst, llabs(s.motor[3] - before));
            }
        };
        run(5000 * 13);  // past two cuts, so m_rev is not zero when the length changes
        RtStatus st;
        s.rt.Status(&st);
        Check(st.crankRev >= 2, "driven line: at least two cuts made", st.crankRev, 2);
        Check(worst <= 1, "driven line: steady feed", (double)worst, 1);

        // A shorter cut, then a longer one: the motor feeds straight through both changes.
        const int64_t before = s.motor[3];
        Check(s.rt.SetCutCounts(10060) == nullptr, "driven line: 503 mm cycle accepted");
        run(5000 * 13);
        Check(worst <= 1, "driven line: no jump when a shorter cut starts", (double)worst, 1);
        Check(s.rt.SetCutCounts(16000) == nullptr, "driven line: 800 mm cycle accepted");
        run(5000 * 17);
        Check(worst <= 1, "driven line: no jump when a longer cut starts", (double)worst, 1);
        const double fed = 5000.0 * 30 * 0.4;  // counts at the line speed, whatever the lengths were
        Check(fabs((double)(s.motor[3] - before) - fed) <= 2.0, "driven line: the line speed ignores the cut length",
              (double)(s.motor[3] - before), fed);

        // rpm stays cuts/min as drawn: the same line speed at any cut length.
        s.rt.Status(&st);
        Check(st.cycleUnits == 16000, "driven line: the 800 mm cycle is running", st.cycleUnits, 16000);
        s.rt.SetRpm(20.0);
        s.Tick(5000);
        s.rt.Status(&st);
        Check(fabs(st.crankVelQ24 / 16777216.0 - 0.8) < 1e-3, "driven line: rpm is the drawn cycle's, not the cut's",
              st.crankVelQ24 / 16777216.0, 0.8);

        // A new length at rest moves nothing, even when the phase is past the end of the shorter cycle.
        s.rt.RequestStop();
        s.Tick(5000);
        s.rt.Status(&st);
        Check(st.crank == CrankState::Off, "driven line: stopped");
        const int64_t rest = s.motor[3];
        Check(s.rt.SetCutCounts(6400) == nullptr, "driven line: shortest cycle accepted");
        worst = 0;
        run(100);
        Check(s.motor[3] == rest, "driven line: a length set at rest moves nothing", (double)(s.motor[3] - rest), 0);
        s.rt.RequestStart();
        s.Tick(5000);
        run(5000);
        Check(worst <= 1, "driven line: restarts without a jump", (double)worst, 1);
    }

    // ---- driven crank
    {
        CamProject p = DefaultProject();
        p.master.source = MasterSource::DrivenCrank;
        p.master.crankConnector = 0;
        p.master.rpm = 60;               // 4000 counts/s = 0.8 counts/sample
        p.master.crankRampRpmPerSec = 120;  // full speed in 0.5 s
        Sim s;
        Check(s.rt.Build(Reload(p)) == nullptr, "driven build");
        s.rt.RequestStart();
        s.Tick(2500);  // 0.5 s ramp
        RtStatus st;
        s.rt.Status(&st);
        Check(fabs(st.crankVelQ24 / 16777216.0 - 0.8) < 1e-3, "driven: at speed after the ramp", st.crankVelQ24 / 16777216.0, 0.8);
        s.Tick(5000 * 3);
        s.rt.Status(&st);
        const double masterCounts = st.crankRev * 4000.0 + (st.crankPhaseQ24 >> 24);
        Check(s.motor[0] == (int64_t)masterCounts, "driven: crank motor steps = master counts", (double)s.motor[0], masterCounts);
        const double phase = st.crankPhaseQ24 / 16777216.0;  // followers take the parts of a count too
        const int64_t want = Counts(p.axes[0], phase * 360.0 / 4000) - Counts(p.axes[0], 0);
        Check(llabs(s.motor[p.axes[0].connector] - want) <= 1, "driven: follower on profile", (double)s.motor[p.axes[0].connector], (double)want);
        s.rt.RequestStop();
        s.Tick(2400);
        s.rt.Status(&st);
        Check(st.crank == CrankState::Stopping, "driven: still ramping down");
        s.Tick(200);
        s.rt.Status(&st);
        Check(st.crank == CrankState::Off && st.crankVelQ24 == 0, "driven: stopped after the ramp");
        Check(s.rt.Build(Reload(p)) == nullptr, "can reload once stopped");
    }

    // ---- timeline
    {
        CamProject p = DefaultProject();
        p.master.rpm = 30;  // 2 s per rev -> 2 s timeline
        SetAxisMaster(p, 0, AxisMaster::Timeline);
        p.master.trigger = TriggerSource::Command;
        const CamAxis &a = p.axes[0];
        const int m = a.connector;
        Sim s;
        s.rt.Build(Reload(p));
        s.rt.RequestTrigger();
        s.Tick(10);
        Check(s.motor[m] == 0, "timeline: trigger before start is ignored");
        s.rt.RequestStart();
        s.Tick(5);
        RtStatus st;
        s.rt.Status(&st);
        Check(st.timeline == TimelineState::Armed, "timeline: armed by start");
        s.rt.RequestTrigger();
        s.Tick();  // t = 1 sample
        int64_t worst = 0;
        for (int i = 2; i <= 10000; ++i) {
            s.Tick();
            const double t = i / 5000.0;  // the trigger sample already advanced time by one sample
            const int64_t err = s.motor[m] - (Counts(a, t) - Counts(a, 0));
            worst = llabs(err) > worst ? llabs(err) : worst;
        }
        Check(worst <= 1, "timeline: motor within 1 count over the run", (double)worst, 1);
        s.Tick(10);
        s.rt.Status(&st);
        Check(st.timeline == TimelineState::Done, "timeline: one-shot done");
        Check(s.motor[m] == 0, "timeline: closed profile ends where it started");

        // Half speed: the same point comes twice as late.
        s.rt.SetOverride(0.5);
        s.rt.RequestTrigger();
        s.Tick(1 + 2 * 2500);  // starts at the override; the trigger sample counts too
        const int64_t want = Counts(a, (1 + 2 * 2500) * 0.5 / 5000.0) - Counts(a, 0);
        Check(llabs(s.motor[m] - want) <= 1, "timeline: 50 % override takes twice as long", (double)s.motor[m], (double)want);
        // Stop ramps the rate down
        s.rt.RequestStop();
        s.Tick(5000);
        s.rt.Status(&st);
        Check(st.timeline == TimelineState::Idle && st.rateQ16 == 0, "timeline: stop ramps to rest");

        // Repeat with an open axis: +10 mm per cycle
        CamProject r = DefaultProject();
        r.master.rpm = 30;
        SetClosed(r.axes[0], false);
        SetKnotPos(r.axes[0], KnotCount(r.axes[0]) - 1, 10.0);
        SetAxisMaster(r, 0, AxisMaster::Timeline);
        r.master.timelineRepeat = true;
        r.master.trigger = TriggerSource::DigitalInput;
        r.master.triggerRising = true;
        Sim q;
        q.rt.Build(Reload(r));
        q.rt.RequestStart();
        q.in.trigger = true;  // already high: not an edge
        q.Tick(100);
        q.rt.Status(&st);
        Check(st.timeline == TimelineState::Armed, "input trigger: a level that was already high is not an edge");
        q.in.trigger = false;
        q.Tick(5);
        q.in.trigger = true;  // rising edge
        q.Tick(1 + 3 * 10000);  // three 2 s cycles
        Check(llabs(q.motor[r.axes[0].connector] - 3 * 8000) <= 1, "repeat: open axis advances every cycle",
              (double)q.motor[r.axes[0].connector], 3 * 8000);
        Check(q.rt.Build(Reload(r)) != nullptr, "cannot load while running");
        q.rt.RequestAbort();
        q.Tick();
        Check(q.rt.Idle(), "abort stops at once");
    }

    // ---- drift: a driven-crank follower must be back on the same count every revolution
    // (the samples/Demo.ecam case: 160 counts/mm, -0.7 .. 17.5 mm, 800 counts/rev at 30 rpm).
    {
        CamProject p = DefaultProject();
        CamAxis &a = p.axes[0];
        a.countsPerUnit = 160;
        a.startPos = -0.7;
        a.segments = {{93, -0.7, MotionLaw::Dwell}, {182, 17.5, MotionLaw::Poly345}, {185, 17.5, MotionLaw::Dwell},
                      {257, -0.7, MotionLaw::Poly345}, {360, -0.7, MotionLaw::Dwell}};
        Normalize(a);
        p.master.source = MasterSource::DrivenCrank;
        p.master.crankPpr = 800;  // x gear 1 = 800 counts per crank rev
        p.master.rpm = 30;
        p.master.crankRampRpmPerSec = 60;
        p.master.crankConnector = 0;
        Sim s;
        s.rt.Build(Reload(p));
        s.rt.RequestStart();
        RtStatus st;
        int32_t lastRev = 0;
        int64_t atRev[64];
        int revs = 0;
        int64_t lastCrank = 0;
        bool backwards = false;
        for (int i = 0; i < 5000 * 2 * 22 && revs < 20; ++i) {  // 30 rpm = 2 s per rev
            s.Tick();
            if (s.motor[0] < lastCrank) backwards = true;
            lastCrank = s.motor[0];
            s.rt.Status(&st);
            if (st.crankRev != lastRev) {
                lastRev = st.crankRev;
                atRev[revs++] = s.motor[a.connector];
            }
        }
        bool same = revs >= 10;
        for (int i = 1; i < revs; ++i) same = same && atRev[i] == atRev[0];
        printf("     Demo: %d revs, follower at each rev start: %lld .. %lld counts\n", revs, (long long)atRev[0],
               (long long)atRev[revs - 1]);
        Check(same, "Demo profile: no commanded drift over 20 revolutions");
        Check(!backwards, "driven crank only turns forward");

        // Direction output on this follower's step stream, old rule vs new (MotorDriver::ApplySampleSteps):
        // a DIR change while the previous sample's pulses are still going out can be miscounted.
        Sim d;
        d.rt.Build(Reload(p));
        d.rt.RequestStart();
        d.Tick(5000 * 4);  // past the crank ramp
        std::vector<int32_t> stream;
        for (int i = 0; i < 5000 * 2; ++i) {  // one revolution at 30 rpm
            int32_t steps[kMotorConnectors];
            d.rt.Tick(d.in, steps);
            stream.push_back(steps[a.connector]);
        }
        auto model = [&](bool newRule, int *changes, int *inFlight, int64_t *net) {
            bool dir = false;  // negative
            int32_t last = 0, carry = 0;
            *changes = *inFlight = 0;
            *net = 0;
            for (int32_t s0 : stream) {
                int32_t s = s0 + carry;
                carry = 0;
                bool want = newRule ? (s != 0 ? s < 0 : dir) : s < 0;
                if (want != dir) {
                    if (newRule && last != 0) {
                        carry = s;  // wait a sample
                        s = 0;
                        want = dir;
                    } else {
                        ++*changes;
                        if (last != 0) ++*inFlight;
                        dir = want;
                    }
                }
                *net += s;
                last = s < 0 ? -s : s;
            }
        };
        int oc, of, nc, nf;
        int64_t on, nn;
        model(false, &oc, &of, &on);
        model(true, &nc, &nf, &nn);
        printf("     DIR changes per rev: old rule %d (%d with pulses in flight), new rule %d (%d)\n", oc, of, nc, nf);
        Check(nf == 0 && nn == on, "new rule: no DIR change with pulses in flight, same net steps");
        Check(nc <= 4, "new rule: DIR changes only at real reversals");
    }

    // ---- a gear ratio that is not whole (50 : 7) stays exact
    {
        CamProject p = DefaultProject();  // cycloid 0..20 mm over 120 deg, closed
        p.master.source = MasterSource::DrivenCrank;
        p.master.crankConnector = 0;
        p.master.crankPpr = 800;
        p.master.crankGearNum = 50;  // 5714.2857 motor steps per crank rev
        p.master.crankGearDen = 7;
        p.master.rpm = 60;
        p.master.crankRampRpmPerSec = 600;
        NormalizeProject(p);
        uint32_t hash = 0;
        const CamProject q = Reload(p, &hash);
        Check(q.master.crankGearDen == 7 && fabs(q.master.countsPerRev - 800.0 * 50 / 7) < 1e-9,
              "upload carries the ratio exactly (cpr 40000, den 7)");
        Sim s;
        Check(s.rt.Build(q) == nullptr, "build with a 50 : 7 crank");
        s.rt.RequestStart();
        RtStatus st;
        int lastRev = 0;
        int64_t followerAtRev = 0;
        bool followerSame = true;
        while (true) {
            s.Tick();
            s.rt.Status(&st);
            if (st.crankRev != lastRev) {
                lastRev = st.crankRev;
                if (lastRev == 1) followerAtRev = s.motor[p.axes[0].connector];
                followerSame = followerSame && s.motor[p.axes[0].connector] == followerAtRev;
                if (lastRev == 7) break;
            }
        }
        // 7 crank revolutions are exactly 50 motor revolutions: 40000 steps, not 7 x round(5714.29).
        const int64_t crankSteps = s.motor[0] - (st.crankPhaseQ24 >> 24) / 7;  // minus the part rev just begun
        printf("     50 : 7 crank: %lld motor steps at the 7th crank turn (40000 expected)\n", (long long)crankSteps);
        Check(crankSteps == 40000, "7 crank turns = exactly 40000 crank-motor steps (no creep)", (double)crankSteps, 40000);
        Check(followerSame, "follower on the same count at every crank turn");

        // Encoder master, same ratio: 40000 encoder counts are exactly 7 crank turns.
        CamProject e = p;
        e.master.source = MasterSource::Encoder;
        Sim r;
        r.rt.Build(Reload(e));
        r.rt.RequestStart();
        r.Tick();
        for (int i = 1; i <= 40000; ++i) {
            r.in.encoder = i;
            r.Tick();
        }
        r.rt.Status(&st);
        Check(st.crankRev == 7 && (st.crankPhaseQ24 >> 24) == 0, "encoder: 40000 counts at 50 : 7 = exactly 7 crank turns",
              st.crankRev, 7);
    }

    // ---- jog (hand mode)
    {
        CamProject p = DefaultProject();  // cycloid 0..20 over 120 deg
        p.master.source = MasterSource::DrivenCrank;
        p.master.crankConnector = 0;
        p.master.crankPpr = 800;  // x gear 1 = 800 counts per crank rev
        p.master.rpm = 120;               // 1600 counts/s = 0.32 counts/sample
        p.master.crankRampRpmPerSec = 600;
        const CamAxis &a = p.axes[0];
        const int m = a.connector;
        Sim s;
        s.rt.Build(Reload(p));
        s.rt.RequestJog(true);
        s.Tick();
        RtStatus st;
        s.rt.Status(&st);
        Check(st.crank == CrankState::Jogging && s.motor[0] == 0 && s.motor[m] == 0, "jog: engages without moving");

        auto settle = [&](int maxTicks) {
            int32_t maxStep = 0;
            for (int i = 0; i < maxTicks; ++i) {
                int32_t steps[kMotorConnectors];
                s.rt.Tick(s.in, steps);
                for (int c = 0; c < kMotorConnectors; ++c) s.motor[c] += steps[c];
                maxStep = steps[0] > maxStep ? steps[0] : (-steps[0] > maxStep ? -steps[0] : maxStep);
                s.rt.Status(&st);
                if (st.crankVelQ24 == 0 && i > 10) break;
            }
            return maxStep;
        };
        s.rt.SetJogCrankCounts(2 * 800 + 200);  // two revs and 90 deg
        const int32_t maxStep = settle(50000);
        Check(s.motor[0] == 1800, "jog: crank motor lands on the target", (double)s.motor[0], 1800);
        Check(maxStep <= 1, "jog: crank speed limited to the profile rpm (<= 1 count/sample)", maxStep, 1);
        int64_t want = Counts(a, 90.0) - Counts(a, 0.0);
        Check(llabs(s.motor[m] - want) <= 1, "jog: follower on its profile at 90 deg", (double)s.motor[m], (double)want);
        s.rt.SetJogCrankCounts(-100);  // back past 0
        settle(50000);
        want = Counts(a, 360.0 - 45.0) - Counts(a, 0.0);
        Check(s.motor[0] == -100 && llabs(s.motor[m] - want) <= 1, "jog: backwards past 0", (double)s.motor[m], (double)want);
        s.rt.RequestJog(false);
        s.Tick(100);
        s.rt.Status(&st);
        Check(st.crank == CrankState::Off, "jog off: crank at rest and released");

        // Encoder master: the encoder is ignored while jogging.
        CamProject e = DefaultProject();
        Sim q;
        q.rt.Build(Reload(e));
        q.rt.RequestJog(true);
        q.Tick();
        q.in.encoder = 1234;
        q.Tick(100);
        Check(q.motor[e.axes[0].connector] == 0, "jog: encoder master ignores the encoder");
        q.rt.SetJogCrankCounts(1000);  // 90 deg of 4000
        for (int i = 0; i < 20000; ++i) q.Tick();
        want = Counts(e.axes[0], 90.0) - Counts(e.axes[0], 0.0);
        Check(llabs(q.motor[e.axes[0].connector] - want) <= 1, "jog: encoder-master profile followed by hand",
              (double)q.motor[e.axes[0].connector], (double)want);
        q.rt.RequestStop();  // stop also ends a jog
        q.Tick(5000);
        q.rt.Status(&st);
        Check(st.crank == CrankState::Off, "stop ends a jog");

        // Timeline jog.
        CamProject tl = DefaultProject();
        tl.master.rpm = 30;
        SetAxisMaster(tl, 0, AxisMaster::Timeline);
        Sim r;
        r.rt.Build(Reload(tl));
        r.rt.RequestJog(true);
        r.Tick();
        r.rt.Status(&st);
        Check(st.timeline == TimelineState::Jogging, "jog: timeline engaged");
        r.rt.SetJogTimeQ4(static_cast<int32_t>(0.5 * 5000 * 16));
        for (int i = 0; i < 20000; ++i) r.Tick();
        want = Counts(tl.axes[0], 0.5) - Counts(tl.axes[0], 0.0);
        Check(llabs(r.motor[tl.axes[0].connector] - want) <= 1, "jog: timeline axis at t = 0.5 s",
              (double)r.motor[tl.axes[0].connector], (double)want);
        r.rt.SetJogTimeQ4(static_cast<int32_t>(99.0 * 5000 * 16));  // past the end: clamped
        for (int i = 0; i < 40000; ++i) r.Tick();
        Check(r.motor[tl.axes[0].connector] == 0, "jog: timeline clamps at its end (closed profile back at 0)");
        r.rt.RequestJog(false);
        r.Tick(10);
        r.rt.Status(&st);
        Check(st.timeline == TimelineState::Idle, "jog off: timeline idle");
    }

    // ---- timeline outputs
    {
        // A crank-only project with outputs: the outputs alone make a timeline (cycle = last off time).
        CamProject p = DefaultProject();
        p.master.trigger = TriggerSource::Command;
        OutputPulse a, b, c;
        a.output = 2; a.on = 0.10; a.off = 0.30;
        b.output = 2; b.on = 0.50; b.off = 0.60;
        c.output = 5; c.on = 0.20; c.off = 0.80;
        p.outputs = {c, a, b};
        NormalizeProject(p);
        Check(p.outputs[0].output == 2 && p.outputs[0].on == 0.10 && p.outputs[2].output == 5, "outputs sorted by output, then on");
        Check(UsesTimeline(p) && fabs(TimelineLength(p) - 0.80) < 1e-12, "outputs alone make a 0.8 s timeline");
        uint32_t hash = 0;
        const CamProject r = Reload(p, &hash);
        Check(r.outputs.size() == 3 && r.outputs[1].on == 0.50 && r.outputs[1].off == 0.60, "out lines round-trip");
        Check(hash == ProfileHash(ProfileLines(p)), "hash covers the out lines");

        Sim s;
        Check(s.rt.Build(r) == nullptr, "build with outputs");
        Check(s.rt.OutputsUsed() == ((1u << 2) | (1u << 5)), "outputs used: IO-2, IO-5", s.rt.OutputsUsed(), 36);
        s.rt.RequestStart();
        s.Tick(10);
        Check(s.rt.Outputs() == 0, "armed: all outputs off");
        s.rt.RequestTrigger();
        auto at = [&](double t) {  // run to timeline time t (from the trigger, speed 1)
            RtStatus st;
            s.rt.Status(&st);
            const int64_t want = llround(t * kRtSampleHz);
            const int64_t now = st.timeQ16 >> 16;
            if (want > now) s.Tick(static_cast<int>(want - now));
            return s.rt.Outputs();
        };
        Check(at(0.05) == 0, "t 0.05: off");
        Check(at(0.15) == (1u << 2), "t 0.15: IO-2 on", s.rt.Outputs(), 4);
        Check(at(0.25) == ((1u << 2) | (1u << 5)), "t 0.25: IO-2 and IO-5 on", s.rt.Outputs(), 36);
        Check(at(0.40) == (1u << 5), "t 0.40: IO-2 off between its pulses", s.rt.Outputs(), 32);
        // Exact edges: on from the sample of `on`.
        Check(at(0.4998) == (1u << 5) && at(0.5000) == ((1u << 2) | (1u << 5)), "IO-2 switches on at 0.5 s");
        Check(at(0.55) == ((1u << 2) | (1u << 5)), "t 0.55: second IO-2 pulse", s.rt.Outputs(), 36);
        s.Tick(kRtSampleHz);  // past the end
        RtStatus st;
        s.rt.Status(&st);
        Check(st.timeline == TimelineState::Done && s.rt.Outputs() == 0 && st.outputs == 0, "one-shot done: all off");

        // Repeat: pulses every cycle; stop and abort turn everything off.
        CamProject q = r;
        q.master.timelineRepeat = true;
        Sim t;
        t.rt.Build(q);
        t.rt.RequestStart();
        t.Tick();
        t.rt.RequestTrigger();
        t.Tick(llround(0.8 * kRtSampleHz) + llround(0.15 * kRtSampleHz));  // second cycle, 0.15 s in
        t.rt.Status(&st);
        Check(st.cycle == 1 && t.rt.Outputs() == (1u << 2), "repeat: IO-2 on again in the next cycle", t.rt.Outputs(), 4);
        t.rt.RequestAbort();
        t.Tick();
        Check(t.rt.Outputs() == 0, "abort: all outputs off");
        t.rt.RequestStart();
        t.Tick();
        t.rt.RequestTrigger();
        t.Tick(llround(0.25 * kRtSampleHz));
        t.rt.RequestStop();
        t.Tick(kRtSampleHz * 2);
        t.rt.Status(&st);
        Check(st.timeline == TimelineState::Idle && t.rt.Outputs() == 0, "stop: idle, all outputs off");

        // Crank pulses: degrees within the revolution, every revolution, while the crank is engaged.
        {
            CamProject cp = DefaultProject();  // encoder master, 4000 counts / rev
            OutputPulse k;
            k.output = 4;
            k.master = AxisMaster::Crank;
            k.on = 90.0;
            k.off = 180.0;
            OutputPulse k0;  // at phase 0, where the crank rests before Start
            k0.output = 5;
            k0.master = AxisMaster::Crank;
            k0.on = 0.0;
            k0.off = 30.0;
            cp.outputs = {k, k0};
            NormalizeProject(cp);
            Check(!UsesTimeline(cp) && UsesCrank(cp), "a crank pulse does not make a timeline");
            const CamProject cr = Reload(cp);
            Check(cr.outputs.size() == 2 && cr.outputs[0].master == AxisMaster::Crank && cr.outputs[0].on == 90.0,
                  "crank out lines round-trip");
            Sim c;
            Check(c.rt.Build(cr) == nullptr, "build with a crank pulse");
            auto deg = [&](double d) {  // turn the encoder to d degrees (absolute, may exceed 360)
                c.in.encoder = static_cast<int32_t>(llround(d / 360.0 * 4000.0));
                c.Tick();
                return c.rt.Outputs();
            };
            // An encoder master engages where the encoder is: start at encoder 0, so phase = encoder angle.
            Check(deg(0) == 0, "crank not engaged: off (also IO-5's 0..30 deg pulse)");
            c.rt.RequestStart();
            c.Tick();
            Check(deg(10) == (1u << 5), "engaged, 10 deg: IO-5 on");
            Check(deg(45) == 0, "45 deg: off");
            Check(deg(89.9) == 0 && deg(90) == (1u << 4), "IO-4 on at 90 deg");
            Check(deg(179.9) == (1u << 4) && deg(180) == 0, "IO-4 off at 180 deg");
            Check(deg(360 + 120) == (1u << 4), "next revolution: on again at 120 deg");
            Check(deg(120) == (1u << 4), "turning back: still by angle");
            c.rt.RequestStop();  // an encoder master lets go at once
            c.Tick();
            Check(c.rt.Outputs() == 0, "stop: crank off, output off");
            ProfileReader rd;
            rd.Begin();
            Check(rd.Line("out 1 350 361 crank") != nullptr && rd.Line("out 1 10 20 crank") == nullptr,
                  "crank out line: within 0..360");
        }

        // Cut-length mode: a flying saw with a 600 mm design cycle on a 20 counts/mm line encoder.
        {
            CamProject lp = DefaultProject();
            lp.master.source = MasterSource::Encoder;
            lp.master.lineMode = true;
            lp.master.crankPpr = 4000;
            lp.master.wheelCircumference = 200.0;  // 20 counts/mm
            lp.master.designLength = 600.0;        // 12000 counts per cut as drawn; 1 deg = 5/3 mm
            lp.master.kerf = 3.0;
            CamAxis &c = lp.axes[0];
            c.countsPerUnit = 100.0;               // direct, 100 counts/mm
            c.drive = AxisDrive::Direct;
            c.segments.clear();
            auto seg = [&](double end, double pos, MotionLaw law) {
                CamSegment s;
                s.endDeg = end;
                s.endPos = pos;
                s.law = law;
                c.segments.push_back(s);
            };
            // wait 0-168 deg (280 mm), blend, sync at line speed (5/3 mm/deg), blend, return.
            seg(168, 0, MotionLaw::Dwell);
            seg(204, 30, MotionLaw::Blend);
            seg(240, 90, MotionLaw::Linear);
            seg(276, 120, MotionLaw::Blend);
            seg(354, 0, MotionLaw::Poly345);
            seg(360, 0, MotionLaw::Dwell);
            OutputPulse saw;
            saw.output = 0;
            saw.master = AxisMaster::Crank;
            saw.on = 201.0;
            saw.off = 243.0;
            lp.outputs = {saw};
            NormalizeProject(lp);
            Check(lp.master.countsPerRev == 12000.0 && lp.master.lineCountsPerMm == 20.0, "line: 12000 counts per design cut");
            Check(fabs(LineWaitDeg(lp) - 168.0) < 1e-9, "line: wait 0..168 deg", LineWaitDeg(lp), 168);
            Check(fabs(LineMinLength(lp) - (600.0 - 280.0 - 3.0)) < 1e-9, "line: shortest piece 317 mm", LineMinLength(lp), 317);
            uint32_t hash = 0;
            const CamProject lr = Reload(lp, &hash);
            Check(lr.master.lineMode && lr.master.lineCountsPerMm == 20.0 && fabs(lr.master.designLength - 600.0) < 1e-9 &&
                      lr.master.kerf == 3.0 && lr.master.countsPerRev == 12000.0,
                  "line: master line round-trips (line, design, kerf)");

            Sim l;
            Check(l.rt.Build(lr) == nullptr && l.rt.LineMode(), "line: build");
            {
                int sa = -1;
                double s0d = 0, s1d = 0;
                Check(LineSync(lr, &sa, &s0d, &s1d) && sa == 0 && s0d == 204.0 && s1d == 240.0, "line: sync = the carriage's Linear 204-240 deg");
                RtStatus s0;
                l.rt.Status(&s0);
                // cut line meets the blade at sync start: 6800 counts of line less the carriage's 30 mm (600 counts of line)
                Check(s0.hasCutGap && fabs(s0.cutGapUnits - 6200.0) < 0.5 && !s0.inSync,  // table rounding: 1 carriage count = 0.2 line counts
                      "line: blade to cut line 6200 counts (310 mm) at the cycle start", s0.cutGapUnits, 6200);
            }
            Check(l.rt.MinCutCounts() == 6400, "line: shortest cycle 6400 counts (320 mm)", l.rt.MinCutCounts(), 6400);
            Check(l.rt.SetCutCounts(6399) != nullptr && l.rt.SetCutCounts(6400) == nullptr, "line: shorter than the fixed part refused");
            l.rt.SetCutCounts(12000);
            l.in.encoder = 0;
            l.rt.RequestStart();
            l.Tick();
            const int m = c.connector;
            // Carriage position at a line position x (counts) into a cycle of L counts: the drawn profile at
            // design phase x - (L - 12000), clamped to the wait.
            auto want = [&](int64_t x, int64_t L) {
                const int64_t xd = std::max<int64_t>(0, x - (L - 12000));
                return llround(Evaluate(lr.axes[0], xd * 360.0 / 12000.0).pos * 100.0);
            };
            int64_t worst = 0, base = 0;
            int32_t enc = 0;
            auto runCycle = [&](int64_t L) {  // one cut of L counts, 4 counts per sample; checks every sample
                for (int64_t x = 4; x <= L; x += 4) {
                    enc += 4;
                    l.in.encoder = enc;
                    l.Tick();
                    const int64_t x0 = x % L;
                    worst = std::max<int64_t>(worst, llabs(l.motor[m] - want(x0, L)));
                    if (x0 == llround(201.0 / 360.0 * 12000.0) + (L - 12000) + 4) {
                        Check(l.rt.Outputs() == 1u, "line: saw output on 201 deg after the stretched wait");
                    }
                }
                base += L;
            };
            runCycle(12000);
            Check(worst <= 1, "line: 600 mm cycle, carriage on profile", worst, 1);
            // Change to a 500 mm piece (+ 3 mm kerf = 503 mm = 10060 counts) halfway through a cut: this cut
            // stays 12000, the next is 10060.
            for (int i = 0; i < 1500; ++i) {
                enc += 4;
                l.in.encoder = enc;
                l.Tick();
            }
            {
                RtStatus sm;
                l.rt.Status(&sm);
                // blending up: 800 counts of line to the sync start, the carriage still behind its sync position
                const double c6000 = Evaluate(lr.axes[0], 180.0).pos;  // mm, x = 6000 counts = 180 deg
                const double want6000 = 800.0 - (30.0 - c6000) * 20.0;
                Check(fabs(sm.cutGapUnits - want6000) < 0.5, "line: blending up, the gap closes faster than the line moves",
                      sm.cutGapUnits, want6000);
            }
            Check(l.rt.SetCutCounts(10060) == nullptr, "line: 503 mm cycle accepted");
            for (int i = 0; i < 400; ++i) {  // into the sync: 7600
                enc += 4;
                l.in.encoder = enc;
                l.Tick();
            }
            {
                RtStatus sp;
                l.rt.Status(&sp);
                Check(sp.inSync && sp.cutGapUnits == 0.0 && fabs(sp.syncLeftUnits - 400.0) < 1e-6,
                      "line: in the sync, on the cut line, 400 counts of sync left", sp.syncLeftUnits, 400);
            }
            for (int i = 0; i < 350; ++i) {  // past the sync: 9000
                enc += 4;
                l.in.encoder = enc;
                l.Tick();
            }
            {
                RtStatus sp;
                l.rt.Status(&sp);
                // the next cut line: next cycle (503 mm) sync start, less the carriage still to return from 9000
                const double c9000 = Evaluate(lr.axes[0], 270.0).pos;  // x = 9000 counts = 270 deg
                const double want9000 = (12000.0 + (10060.0 - 12000.0 + 6800.0) - 9000.0) - (30.0 - c9000) * 20.0;
                Check(!sp.inSync && fabs(sp.cutGapUnits - want9000) < 0.5,
                      "line: after the sync, to the next cut line at the new length", sp.cutGapUnits, want9000);
            }
            for (int i = 2250; i < 3000; ++i) {
                enc += 4;
                l.in.encoder = enc;
                l.Tick();
            }
            RtStatus st;
            l.rt.Status(&st);
            Check(st.crankRev == 2 && st.cycleUnits == 10060, "line: new length starts with the next cut",
                  st.cycleUnits, 10060);
            worst = 0;
            runCycle(10060);
            Check(worst <= 1, "line: 503 mm cycle, carriage on profile (sync and return unchanged)", worst, 1);
            l.rt.Status(&st);
            Check(st.crankRev == 3 && (st.crankPhaseQ24 >> 24) == 0, "line: cuts exactly 10060 counts apart");
            // Sync ratio: during the sync the carriage moves 100/20 = 5 motor counts per encoder count at any length.
            const int64_t x1 = llround(215.0 / 360.0 * 12000.0) + (10060 - 12000), x2 = x1 + 400;
            Check(want(x2, 10060) - want(x1, 10060) == 2000, "line: sync at line speed (5 counts per encoder count)",
                  (double)(want(x2, 10060) - want(x1, 10060)), 2000);
        }

        // Encoder direction travels with the upload (rev=1); not for a driven crank.
        {
            CamProject ep = DefaultProject();
            ep.master.source = MasterSource::Encoder;
            ep.master.encoderReverse = true;
            NormalizeProject(ep);
            Check(Reload(ep).master.encoderReverse, "encoder reverse round-trips");
            ep.master.encoderReverse = false;
            Check(!Reload(ep).master.encoderReverse, "encoder forward round-trips");
            ep.master.source = MasterSource::DrivenCrank;
            ep.master.encoderReverse = true;
            bool hasRev = false;
            for (const std::string &l : ProfileLines(ep)) hasRev = hasRev || l.find(" rev=1") != std::string::npos;
            Check(!hasRev, "no rev= for a driven crank");
        }

        // The board refuses inputs that clash.
        auto refuse = [&](CamProject x, const char *what) {
            ProfileReader rd;
            rd.Begin();
            for (const std::string &l : ProfileLines(x)) rd.Line(l.c_str());
            CamProject out;
            Check(rd.Finish(&out) != nullptr, what);
        };
        CamProject clash = p;
        clash.master.trigger = TriggerSource::DigitalInput;
        clash.master.triggerInput = 2;
        refuse(clash, "trigger input IO-2 is also an output: refused");
        CamProject enc = p;
        enc.master.trigger = TriggerSource::DigitalInput;
        enc.master.triggerInput = 7;  // DI-7 with an encoder master
        refuse(enc, "trigger on DI-7 with an encoder master: refused");
        enc.master.source = MasterSource::DrivenCrank;
        {
            ProfileReader rd;
            rd.Begin();
            for (const std::string &l : ProfileLines(enc)) rd.Line(l.c_str());
            CamProject out;
            Check(rd.Finish(&out) == nullptr, "trigger on DI-7 with a driven crank: allowed");
        }
        ProfileReader bad;
        bad.Begin();
        Check(bad.Line("out 6 0 1") != nullptr && bad.Line("out 1 0.5 0.5") != nullptr && bad.Line("out 1 0.5 0.6") == nullptr,
              "out line checks: IO 0..5, off > on");
    }

    // ---- Cubic spline through the upload and the table
    {
        CamProject p;
        p.master.source = MasterSource::Encoder;
        p.master.crankPpr = 4000;
        p.master.rpm = 60;
        CamAxis a;
        a.name = "Curve";
        a.connector = 1;
        a.countsPerUnit = 800;
        a.startPos = 0;
        a.segments = {{60, 2.5, MotionLaw::Spline}, {130, 4, MotionLaw::Spline}, {200, 1, MotionLaw::Spline},
                      {290, -1.5, MotionLaw::Spline}, {360, 0, MotionLaw::Spline}};
        p.axes.push_back(a);
        NormalizeProject(p);
        const CamProject q = Reload(p);
        Check(q.axes[0].segments[2].law == MotionLaw::Spline, "spline: the law survives the upload text");

        static CamRuntime rt;
        Check(rt.Build(q) == nullptr, "spline: builds");
        rt.RequestStart();
        int32_t steps[kMotorConnectors];
        CamRuntime::Input in;
        rt.Tick(in, steps);
        // Whole revolutions of the encoder, a little at a time: the motor stays on the curve to a count,
        // and comes back to its start each revolution (the cycle is closed).
        const SplineFit fit = FitSplines(q.axes[0]);
        int64_t pos = 0;
        double worst = 0;
        int peak = 0;
        const int64_t startCounts = static_cast<int64_t>(lround(Evaluate(q.axes[0], 0.0, &fit).pos * 800));
        for (int i = 1; i <= 8000; ++i) {
            in.encoder = i;
            rt.Tick(in, steps);
            pos += steps[1];
            peak = steps[1] > peak ? steps[1] : peak;
            const double want = Evaluate(q.axes[0], fmod(i * 360.0 / 4000, 360.0), &fit).pos * 800 - startCounts;
            worst = fabs(pos - want) > worst ? fabs(pos - want) : worst;
        }
        // The table rounds to a count (+-0.5) and the interpolation floors (up to 1): any law's bound.
        Check(worst < 1.6, "spline: the motor stays within a count and a half of the curve", worst, 0);
        Check(pos == 0, "spline: closed: two revolutions end where they started", static_cast<double>(pos), 0);
        rt.RequestAbort();
        rt.Tick(in, steps);
    }

    // ---- Electronic gear: exact, for ever, in whole numbers
    {
        // What the axis must be at after a total of u master units: floor(u * cpu * num / (den * units per rev)).
        auto want = [](int64_t units, int64_t cpu, int64_t num, int64_t den, int64_t unitsPerRev) {
            const int64_t a = units * cpu * num, b = den * unitsPerRev;
            int64_t q = a / b;
            if (a % b != 0 && (a < 0) != (b < 0)) --q;
            return q;
        };
        auto gearProject = [](int num, int den, double cpu) {
            CamProject p;
            p.master.source = MasterSource::Encoder;
            p.master.crankPpr = 4000;
            p.master.rpm = 60;
            CamAxis a;
            a.name = "Gear";
            a.units = "rev";
            a.connector = 1;
            a.countsPerUnit = cpu;
            a.gear = true;
            a.gearNum = num;
            a.gearDen = den;
            a.segments.clear();
            p.axes.push_back(a);
            NormalizeProject(p);
            return p;
        };

        // Awkward ratios, encoder master: every sample exact, forward, back past zero, and home again.
        static Sim s;
        const int ratios[][2] = {{1, 1}, {3, 2}, {3, 7}, {-3, 7}, {22, 7}, {1, 1000}, {997, 991}};
        for (const auto &ratio : ratios) {
            const CamProject p = Reload(gearProject(ratio[0], ratio[1], 800));
            Check(p.axes[0].gear && p.axes[0].gearNum == ratio[0] && p.axes[0].gearDen == ratio[1], "gear: the ratio survives the upload text");
            s = Sim();
            Check(s.rt.Build(p) == nullptr, "gear: builds");
            s.in.encoder = 0;
            s.rt.RequestStart();
            s.Tick();
            int bad = 0;
            int64_t e = 0;
            for (int i = 0; i < 300000; ++i) {
                e += (i % 7) + 1;  // uneven steps, several counts a sample
                s.in.encoder = static_cast<int32_t>(e);
                s.Tick();
                bad += s.motor[1] != want(e, 800, ratio[0], ratio[1], 4000);
            }
            Check(bad == 0, "gear: exact at every sample, forward", bad, 0);
            for (int i = 0; i < 700000; ++i) {
                e -= (i % 5) + 1;
                s.in.encoder = static_cast<int32_t>(e);
                s.Tick();
                bad += s.motor[1] != want(e, 800, ratio[0], ratio[1], 4000);
            }
            Check(bad == 0, "gear: exact at every sample, backward through zero", bad, 0);
            s.in.encoder = 0;
            s.Tick();
            Check(s.motor[1] == 0, "gear: home again, to the count", static_cast<double>(s.motor[1]), 0);
            RtStatus st;
            s.rt.Status(&st);
            Check(st.axisPos[0] == s.motor[1], "gear: status reports the same position");
        }

        // A billion counts of master later there is still no drift: 3:7 in whole turns of 7 revolutions.
        {
            const CamProject p = Reload(gearProject(3, 7, 800));
            s = Sim();
            s.rt.Build(p);
            s.rt.RequestStart();
            s.Tick();
            int64_t e = 0;
            const int64_t per = 7 * 4000;  // seven master revolutions: exactly 3 x 800 counts
            for (int i = 0; i < 40000; ++i) {
                e += per;
                s.in.encoder = static_cast<int32_t>(e);
                s.Tick();
            }
            Check(s.motor[1] == 40000LL * 2400, "gear: 280000 revolutions, not a count of drift", static_cast<double>(s.motor[1]), 40000.0 * 2400);
        }

        // A crank gear ratio that is not whole: 50:7 master units; the follower is per crank revolution.
        {
            CamProject g = gearProject(5, 3, 800);
            g.master.crankGearNum = 50;
            g.master.crankGearDen = 7;
            NormalizeProject(g);
            const CamProject p = Reload(g);
            s = Sim();
            Check(s.rt.Build(p) == nullptr, "gear: on a 50:7 crank");
            s.rt.RequestStart();
            s.Tick();
            // 3 crank revolutions = 3 x 4000 x 50 / 7 encoder counts: take 7 x that, whole.
            const int64_t counts = 3LL * 4000 * 50;
            int64_t e = 0;
            for (int64_t i = 0; i < counts; i += 100) {
                e += 100;
                s.in.encoder = static_cast<int32_t>(e);
                s.Tick();
            }
            Check(s.motor[1] == 7LL * 5 * 800, "gear: 21 crank revolutions on 50:7 are 35 follower revolutions", static_cast<double>(s.motor[1]), 7.0 * 5 * 800);
        }

        // Counts per unit that is not whole: a belt. As exact as the double, far under a count over a long run.
        {
            const double cpu = 800 / (3.14159265358979323846 * 20);
            const CamProject p = Reload(gearProject(100, 1, cpu));
            s = Sim();
            Check(s.rt.Build(p) == nullptr, "gear: a belt's counts per unit");
            s.rt.RequestStart();
            s.Tick();
            int64_t e = 0;
            for (int i = 0; i < 100000; ++i) {
                e += 400;
                s.in.encoder = static_cast<int32_t>(e);
                s.Tick();
            }
            const double exact = e / 4000.0 * 100 * cpu;  // 1e4 revolutions, 1.27e7 counts
            Check(fabs(static_cast<double>(s.motor[1]) - exact) <= 1.0, "gear: a belt stays within a count over 10000 revolutions",
                  static_cast<double>(s.motor[1]), exact);
        }

        // A driven crank: ramps up, runs, ramps down, in step with the crank's own motor; a cam alongside.
        {
            CamProject g = gearProject(3, 2, 800);
            g.master.source = MasterSource::DrivenCrank;
            g.master.crankPpr = 800;
            g.master.crankConnector = 0;
            g.master.rpm = 90;
            g.master.crankRampRpmPerSec = 120;
            g.master.crankMaxRpm = 600;
            CamAxis cam = DefaultAxis("Cam");
            cam.connector = 2;
            g.axes.push_back(cam);
            NormalizeProject(g);
            const CamProject p = Reload(g);
            s = Sim();
            Check(s.rt.Build(p) == nullptr, "gear: driven crank with a cam");
            Check(s.rt.AxisCount() == 2 && s.rt.CrankConnector() == 0, "gear: both axes and the crank motor");
            s.rt.RequestStart();
            int bad = 0;
            for (int i = 0; i < 40000; ++i) {
                s.Tick();
                bad += s.motor[1] != want(s.motor[0], 1, 3, 2, 1);  // 3 : 2, both 800 steps a revolution
            }
            Check(bad == 0, "gear: in step with the crank motor through the ramp and at speed", bad, 0);
            s.rt.RequestStop();
            RtStatus st;
            for (int i = 0; i < 40000; ++i) {
                s.Tick();
                bad += s.motor[1] != want(s.motor[0], 1, 3, 2, 1);
            }
            s.rt.Status(&st);
            Check(st.crank == CrankState::Off && bad == 0, "gear: and down the stop ramp, to rest");
            const int64_t rest = s.motor[1];
            s.Tick(100);
            Check(s.motor[1] == rest, "gear: stays at rest with the crank off");
            // Starting again carries on from where it stopped.
            s.rt.RequestStart();
            s.Tick(10000);
            Check(s.motor[1] == want(s.motor[0], 1, 3, 2, 1), "gear: restarts in step");
            s.rt.RequestAbort();
            s.Tick();
        }

        // Hand jog moves the crank; the gear follows, there and back.
        {
            CamProject g = gearProject(2, 1, 800);
            g.master.source = MasterSource::DrivenCrank;
            g.master.crankPpr = 800;
            g.master.crankConnector = 0;
            g.master.rpm = 60;
            g.master.crankRampRpmPerSec = 600;
            g.master.crankMaxRpm = 600;
            NormalizeProject(g);
            const CamProject p = Reload(g);
            s = Sim();
            s.rt.Build(p);
            s.rt.RequestJog(true);
            s.Tick();
            s.rt.SetJogCrankCounts(2000);
            s.Tick(30000);
            Check(s.motor[0] == 2000 && s.motor[1] == 4000, "gear: follows a jog of the crank", static_cast<double>(s.motor[1]), 4000);
            s.rt.SetJogCrankCounts(-1000);
            s.Tick(30000);
            Check(s.motor[0] == -1000 && s.motor[1] == -2000, "gear: and back past zero", static_cast<double>(s.motor[1]), -2000);
            s.rt.RequestJog(false);
            s.Tick(5000);
        }

        // Stopped while an encoder keeps turning: the axis has let go, and takes up from there with no jump.
        {
            const CamProject p = Reload(gearProject(1, 1, 800));
            s = Sim();
            s.rt.Build(p);
            s.rt.RequestStart();
            s.Tick();
            s.in.encoder = 4000;
            s.Tick();
            Check(s.motor[1] == 800, "gear: one master revolution");
            s.rt.RequestStop();
            s.Tick();
            s.in.encoder = 8000;
            s.Tick();
            Check(s.motor[1] == 800, "gear: lets go when stopped");
            s.rt.RequestStart();
            s.Tick();
            s.in.encoder = 8400;
            s.Tick();
            Check(s.motor[1] == 880, "gear: takes up with no jump", static_cast<double>(s.motor[1]), 880);
        }

        // Cut-length mode: the gear rides the line; changing the cut does not change its ratio.
        {
            CamProject g;
            g.master.source = MasterSource::Encoder;
            g.master.lineMode = true;
            g.master.crankPpr = 4000;
            g.master.wheelCircumference = 200;  // 20 counts per mm
            g.master.designLength = 100;        // 2000 counts a cycle
            g.master.rpm = 30;
            CamAxis knife = DefaultAxis("Knife");
            knife.connector = 2;
            knife.segments = {{90, 0, MotionLaw::Dwell}, {270, 10, MotionLaw::Cycloidal}, {360, 0, MotionLaw::Cycloidal}};
            g.axes.push_back(knife);
            CamAxis feed;
            feed.name = "Feed";
            feed.connector = 1;
            feed.countsPerUnit = 8;
            feed.gear = true;
            feed.gearNum = 100;  // 100 mm of feed per 100 mm cycle: 1:1 with the line
            feed.gearDen = 1;
            feed.segments.clear();
            g.axes.push_back(feed);
            NormalizeProject(g);
            Check(LineWaitDeg(g) == 90, "gear: a gear axis has no say in the cycle's wait", LineWaitDeg(g), 90);
            int ax = -1;
            double s0 = 0, s1 = 0;
            Check(!LineSync(g, &ax, &s0, &s1), "gear: its Linear segment is no sync");
            const CamProject p = Reload(g);
            s = Sim();
            Check(s.rt.Build(p) == nullptr, "gear: in cut-length mode");
            s.rt.RequestStart();
            s.Tick();
            Check(s.rt.SetCutCounts(3000) == nullptr, "gear: cut length changed");
            int64_t e = 0;
            int bad = 0;
            for (int i = 0; i < 20000; ++i) {
                e += 3;
                s.in.encoder = static_cast<int32_t>(e);
                s.Tick();
                bad += s.motor[1] != want(e, 8, 100, 1, 2000);
            }
            Check(bad == 0, "gear: exact on the line through changed cuts", bad, 0);
        }

        // Refused.
        ProfileReader rd;
        auto refused = [&](const char *line) {
            rd.Begin();
            rd.Line("master src=encoder cpr=4000 den=1 rpm=60 crank=0 ramp=60 max=600");
            return rd.Line(line) != nullptr;
        };
        Check(refused("axis 0 master=crank m=1 cpu=800 start=0 closed=0 span=360 units=rev gear=0:1"), "gear: zero refused");
        Check(refused("axis 0 master=crank m=1 cpu=800 start=0 closed=0 span=360 units=rev gear=1:0"), "gear: zero denominator refused");
        Check(refused("axis 0 master=crank m=1 cpu=800 start=0 closed=0 span=360 units=rev gear=2"), "gear: needs both numbers");
        Check(refused("axis 0 master=crank m=1 cpu=800 start=0 closed=0 span=360 units=rev gear=1:-2"), "gear: a negative denominator refused");
        Check(refused("axis 0 master=timeline m=1 cpu=800 start=0 closed=0 span=2 units=rev gear=1:1"), "gear: not on the timeline");
        static CamRuntime big;
        CamProject huge = gearProject(2000000000, 1, 1.0e6);
        Check(big.Build(huge) != nullptr, "gear: a ratio past the arithmetic refused");
    }

    // ---- Motor direction: the steps turn round, nothing else does
    {
        auto project = [](bool crankRev, bool gearRev, bool camRev) {
            CamProject p;
            p.master.source = MasterSource::DrivenCrank;
            p.master.crankConnector = 0;
            p.master.crankPpr = 4000;
            p.master.rpm = 60;
            p.master.crankReverse = crankRev;
            CamAxis g;
            g.name = "Gear";
            g.units = "rev";
            g.connector = 1;
            g.countsPerUnit = 800;
            g.gear = true;
            g.gearNum = 3;
            g.gearDen = 2;
            g.segments.clear();
            g.reverse = gearRev;
            p.axes.push_back(g);
            CamAxis c = DefaultAxis("Cam");
            c.connector = 2;
            c.reverse = camRev;
            p.axes.push_back(c);
            NormalizeProject(p);
            return p;
        };
        const CamProject plain = project(false, false, false);
        const CamProject rev = Reload(project(true, true, true));
        Check(rev.master.crankReverse && rev.axes[0].reverse && rev.axes[1].reverse, "direction: survives the upload text");
        Check(ProfileHash(ProfileLines(plain)) == ProfileHash(ProfileLines(Reload(plain))) &&
                  !Reload(plain).master.crankReverse && !Reload(plain).axes[0].reverse,
              "direction: off by default");
        bool clean = true;
        for (const std::string &l : ProfileLines(plain)) clean = clean && l.find("mrev") == std::string::npos;
        Check(clean, "direction: off writes nothing, so old projects keep their hash");
        {
            bool none = true;
            for (const std::string &l : ProfileLines(plain)) none = none && l.rfind("ccio", 0) != 0;
            Check(none && Reload(plain).ccio.boards == 0, "ccio: no boards writes nothing, so old projects keep their hash");
            CamProject withBoards = plain;
            withBoards.ccio.boards = 3;
            uint32_t hash = 0;
            const CamProject back = Reload(withBoards, &hash);
            Check(back.ccio.boards == 3 && hash == ProfileHash(ProfileLines(withBoards)), "ccio: the count survives the upload text");
            Check(ProfileHash(ProfileLines(withBoards)) != ProfileHash(ProfileLines(plain)), "ccio: the count is in the hash, so a change needs an upload");
            ProfileReader r;
            r.Begin();
            Check(r.Line("ccio_boards 0") != nullptr && r.Line("ccio_boards 9") != nullptr && r.Line("ccio_boards") != nullptr && r.Line("ccio_boards 2 3") != nullptr &&
                      r.Line("ccio_boards 8") == nullptr,
                  "ccio: 1 to 8 boards only");
        }
        static Sim a, b;
        a = Sim();
        b = Sim();
        Check(a.rt.Build(plain) == nullptr && b.rt.Build(rev) == nullptr, "direction: both build");
        a.rt.RequestStart();
        b.rt.RequestStart();
        int bad = 0;
        for (int i = 0; i < 60000; ++i) {
            a.Tick();
            b.Tick();
            for (int m : {0, 1, 2}) bad += a.motor[m] != -b.motor[m];
        }
        Check(a.motor[0] > 1000 && a.motor[1] > 1000, "direction: the motors moved");
        Check(bad == 0, "direction: every sample the exact opposite, crank, gear and cam", bad, 0);
        RtStatus sa, sb;
        a.rt.Status(&sa);
        b.rt.Status(&sb);
        Check(sa.axisPos[0] == sb.axisPos[0] && sa.axisPos[1] == sb.axisPos[1] && sa.crankRev == sb.crankRev,
              "direction: positions report as drawn");
        const CamProject one = Reload(project(true, false, false));
        static Sim c;
        c = Sim();
        c.rt.Build(one);
        c.rt.RequestStart();
        for (int i = 0; i < 60000; ++i) c.Tick();
        Check(c.motor[0] < 0 && c.motor[1] > 0, "direction: only the motor asked for turns round");
    }

    // ---- Stop on a timeline: it finishes the cycle; Start never rewinds an engaged axis
    {
        CamProject p = DefaultProject();  // a closed rise / dwell / return / dwell, 800 counts/mm
        SetAxisMaster(p, 0, AxisMaster::Timeline);
        p.master.trigger = TriggerSource::Command;
        p.master.timelineRepeat = true;
        NormalizeProject(p);
        const CamProject q = Reload(p);
        const int m = p.axes[0].connector;
        const int64_t cycle = static_cast<int64_t>(llround(TimelineLength(p) * kRtSampleHz));
        auto s = std::make_unique<Sim>();
        Check(s->rt.Build(q) == nullptr, "cycle stop: build");
        s->rt.RequestStart();
        s->Tick();
        s->rt.RequestTrigger();
        s->Tick(static_cast<int>(cycle + cycle / 3));  // into the second cycle
        RtStatus st;
        s->rt.Status(&st);
        const int64_t mid = s->motor[m];
        Check(mid != 0 && st.timeline == TimelineState::Running, "cycle stop: running part-way through a cycle", (double)mid, 1);
        s->rt.RequestCycleStop();
        s->Tick();
        s->rt.Status(&st);
        Check(st.timeline == TimelineState::Running && st.endOfCycle, "cycle stop: it keeps running its last cycle");
        int32_t worst = 0;
        int ticks = 0;
        do {
            int32_t steps[kMotorConnectors];
            s->rt.Tick(s->in, steps);
            s->motor[m] += steps[m];
            worst = std::max(worst, std::abs(steps[m]));
            s->rt.Status(&st);
        } while (st.timeline == TimelineState::Running && ++ticks < 100000);
        Check(st.timeline == TimelineState::Idle && !st.endOfCycle, "cycle stop: idle at the cycle's end");
        Check(llabs(ticks - (cycle - cycle / 3)) <= 2, "cycle stop: it ran the rest of the cycle", ticks, (double)(cycle - cycle / 3));
        Check(s->motor[m] == 0 && worst < 40, "cycle stop: the axis is back at its start, at the profile's own speed", (double)s->motor[m], 0);
        Check(st.outputs == 0, "cycle stop: outputs off at rest");
        s->Tick(1000);
        Check(s->motor[m] == 0, "cycle stop: and stays there");
        // Start again: nothing moves until the trigger, then the cycle runs as drawn.
        s->rt.RequestStart();
        s->Tick(500);
        Check(s->motor[m] == 0, "cycle stop: Start arms without moving");
        s->rt.RequestTrigger();
        s->Tick(static_cast<int>(cycle / 3));
        int64_t want = 0;
        {
            const CamAxis &a = q.axes[0];
            want = Counts(a, (cycle / 3) / static_cast<double>(kRtSampleHz)) - Counts(a, 0);
        }
        Check(llabs(s->motor[m] - want) <= 2, "cycle stop: the next cycle runs from the start", (double)s->motor[m], (double)want);

        // Stop again while finishing: at once (the stop ramp).
        s->rt.RequestCycleStop();
        s->Tick();
        s->rt.RequestCycleStop();
        ticks = 0;
        do {
            s->Tick();
            s->rt.Status(&st);
        } while (st.timeline != TimelineState::Idle && ++ticks < 100000);
        Check(ticks < 3000 && st.timeQ16 < static_cast<int64_t>(cycle) << 16, "cycle stop: Stop twice stops part-way, at the ramp", ticks, 2500);

        // The jump: Start after a stop part-way used to rewind the engaged axis to its start in one sample.
        const int64_t atRest = s->motor[m];
        Check(atRest != 0, "stop now: the axis rests part-way", (double)atRest, 1);
        int32_t burst = 0;
        s->rt.RequestStart();
        for (int i = 0; i < 500; ++i) {
            int32_t steps[kMotorConnectors];
            s->rt.Tick(s->in, steps);
            s->motor[m] += steps[m];
            burst = std::max(burst, std::abs(steps[m]));
        }
        Check(burst == 0 && s->motor[m] == atRest, "stop now: Start doesn't move the axis", burst, 0);

        // A one-shot stopped: it finishes, and waits for Start rather than the trigger.
        CamProject o = p;
        o.master.timelineRepeat = false;
        auto u = std::make_unique<Sim>();
        u->rt.Build(Reload(o));
        u->rt.RequestStart();
        u->Tick();
        u->rt.RequestTrigger();
        u->Tick(static_cast<int>(cycle / 2));
        u->rt.RequestCycleStop();
        ticks = 0;
        do {
            u->Tick();
            u->rt.Status(&st);
        } while (st.timeline == TimelineState::Running && ++ticks < 100000);
        Check(st.timeline == TimelineState::Idle && u->motor[m] == 0, "cycle stop: a one-shot finishes and is idle, not done");
        u->rt.RequestTrigger();
        u->Tick(500);
        u->rt.Status(&st);
        Check(st.timeline == TimelineState::Idle && u->motor[m] == 0, "cycle stop: the trigger doesn't run it again");
    }

    // ---- limits: toward a tripped limit, a controlled stop; away from it, nothing
    {
        // A driven crank: the master ramps to rest over the limit stop time, the axis following its profile down.
        CamProject p = DefaultProject();  // rise 0..20 mm over 0..120 deg, return later; on M-1
        p.master.source = MasterSource::DrivenCrank;
        p.master.crankConnector = 0;
        p.master.rpm = 60.0;
        p.master.crankRampRpmPerSec = 600.0;
        p.master.limitStopSeconds = 0.05;
        BoardInput lim;  // the stop time reaches the board only with a limit set up
        lim.pin = 3;
        lim.role = CcioRole::LimitPositive;
        lim.axis = 0;
        SetBoardInput(p, lim);
        NormalizeProject(p);
        Check(Reload(p).master.limitStopSeconds == 0.05 && Reload(p).inputs.size() == 1, "limits: the limit and its stop time reach the board");
        const int m = p.axes[0].connector;
        auto s = std::make_unique<Sim>();
        Check(s->rt.Build(Reload(p)) == nullptr, "limits: build (driven crank)");
        s->rt.RequestStart();
        s->Tick(5000 + 5000 / 6);  // up to speed, then 60 deg into the rise: moving +
        RtStatus st;
        s->rt.Status(&st);
        Check(st.crank == CrankState::Running, "limits: running");
        // Moving +: a tripped - limit does nothing.
        s->in.limitMinus = 1;
        s->Tick(20);
        s->rt.Status(&st);
        Check(st.crank == CrankState::Running && s->rt.LimitEvents() == 0, "limits: a - limit doesn't stop an axis moving +");
        s->in.limitMinus = 0;
        // The + limit trips.
        int32_t lastStep = 0;
        {
            const int64_t before = s->motor[m];
            s->Tick();
            lastStep = static_cast<int32_t>(s->motor[m] - before);
        }
        s->in.limitPlus = 1;
        s->Tick();
        Check(s->rt.LimitEvents() == 1 && s->rt.LimitEventAxis() == 0 && s->rt.LimitEventSide() == 1, "limits: the + limit stops it, axis 0, +");
        int ticks = 0;
        int32_t worst = 0;
        int64_t start = s->motor[m];
        do {
            int32_t steps[kMotorConnectors];
            s->rt.Tick(s->in, steps);
            s->motor[m] += steps[m];
            worst = std::max(worst, std::abs(steps[m]));
            s->rt.Status(&st);
        } while (st.crank != CrankState::Off && ++ticks < 100000);
        std::printf("     limit stop (driven crank): %d samples, %lld counts past the trip, worst %d counts/sample (was %d)\n", ticks,
                    static_cast<long long>(s->motor[m] - start), worst, lastStep);
        Check(st.crank == CrankState::Off && ticks <= 260, "limits: the crank rests within the limit stop time (0.05 s)", ticks, 250);
        Check(worst <= std::abs(lastStep) + 1, "limits: no burst: the axis only slows down");
        Check(s->motor[m] - start >= 0, "limits: and doesn't turn back");
        // Start again on the limit is the engine's to refuse; an axis jog away (-) is allowed, toward (+) ramps to rest.
        s->rt.AxisJog(0, -4000.0, 40000.0);
        s->Tick(1000);
        Check(s->rt.AxisMoving(0), "limits: axis jog away from it runs");
        s->rt.AxisStop(0);
        s->Tick(2000);
        s->rt.AxisJog(0, 4000.0, 40000.0);
        ticks = 0;
        do {
            s->Tick();
        } while (s->rt.AxisMoving(0) && ++ticks < 100000);
        Check(ticks <= 260, "limits: axis jog toward it ramps to rest", ticks, 250);
        Check(s->rt.LimitEvents() == 1, "limits: and that is not a cam limit stop (no fault)");

        // A timeline: its speed ramps to zero over the limit stop time.
        CamProject q = DefaultProject();
        SetAxisMaster(q, 0, AxisMaster::Timeline);
        q.master.trigger = TriggerSource::Command;
        q.master.timelineRepeat = true;
        q.master.limitStopSeconds = 0.1;
        NormalizeProject(q);
        auto u = std::make_unique<Sim>();
        Check(u->rt.Build(Reload(q)) == nullptr, "limits: build (timeline)");
        u->rt.RequestStart();
        u->Tick();
        u->rt.RequestTrigger();
        u->Tick(int(q.axes[0].span * 5000 * 60 / 360));  // 60 deg's worth into the rise
        u->in.limitPlus = 1;
        u->Tick();
        Check(u->rt.LimitEvents() == 1, "limits: the timeline stops for it");
        ticks = 0;
        do {
            u->Tick();
            u->rt.Status(&st);
        } while (st.timeline != TimelineState::Idle && ++ticks < 100000);
        Check(st.timeline == TimelineState::Idle && ticks <= 510, "limits: the timeline rests within 0.1 s", ticks, 500);

        // An encoder master can't be ramped: the axis lets go of it and ramps itself to rest.
        CamProject e = DefaultProject();
        e.master.limitStopSeconds = 0.05;
        SetBoardInput(e, lim);
        NormalizeProject(e);
        auto w = std::make_unique<Sim>();
        Check(w->rt.Build(Reload(e)) == nullptr, "limits: build (encoder)");
        w->in.encoder = 0;
        w->rt.RequestStart();
        w->Tick();
        int enc = 0;
        for (int i = 0; i < 150; ++i) {  // 4 counts a sample: 54 deg of the 4000-count crank, in the rise
            enc += 4;
            w->in.encoder = enc;
            w->Tick();
        }
        w->in.limitPlus = 1;
        enc += 4;
        w->in.encoder = enc;
        w->Tick();
        Check(w->rt.LimitEvents() == 1, "limits: an encoder master's axis stops for it");
        ticks = 0;
        const int64_t at = w->motor[m];
        do {
            enc += 4;  // the line goes on
            w->in.encoder = enc;
            w->Tick();
        } while (w->rt.AxisMoving(0) && ++ticks < 100000);
        w->rt.Status(&st);
        Check(st.crank == CrankState::Off && ticks <= 260, "limits: the axis ramps itself to rest while the line goes on", ticks, 250);
        const int64_t rest = w->motor[m];
        for (int i = 0; i < 500; ++i) {
            enc += 4;
            w->in.encoder = enc;
            w->Tick();
        }
        Check(w->motor[m] == rest && rest >= at, "limits: and stays there");

        // Moving away from a tripped limit with the cam: nothing happens (the return of the profile, going -).
        CamProject r = DefaultProject();
        r.master.source = MasterSource::DrivenCrank;
        r.master.crankConnector = 0;
        r.master.rpm = 60.0;
        r.master.crankRampRpmPerSec = 600.0;
        NormalizeProject(r);
        auto v = std::make_unique<Sim>();
        v->rt.Build(Reload(r));
        v->in.limitPlus = 1;  // on the + limit from the start: the rise goes +, so it stops at once
        v->rt.RequestStart();
        v->Tick(10);
        Check(v->rt.LimitEvents() == 0, "limits: at rest on the + limit, a ramp from rest starts with nothing moving yet");
        v->Tick(3000);
        Check(v->rt.LimitEvents() == 1, "limits: and the first + step stops it");
        v->rt.RequestAbort();
        v->Tick();
        v->rt.Status(&st);
        Check(!st.limitStopping && st.crank == CrankState::Off, "limits: abort ends a limit stop at once");
    }

    // ---- axis jog: one axis on its own, the cam not engaged
    {
        CamProject p = DefaultProject();  // encoder, one crank axis on M-1, 800 counts/mm
        auto s = std::make_unique<Sim>();
        Check(s->rt.Build(Reload(p)) == nullptr, "axis jog: build");
        const int m = p.axes[0].connector;
        int64_t cam = -1;
        Check(s->rt.CamPosition(0, true, &cam) && cam == 0, "axis jog: at the load the axis is at its cam position", (double)cam, 0);
        // Velocity: 8000 counts/s, reached in 0.25 s.
        s->rt.AxisJog(0, 8000.0, 32000.0);
        int64_t worstStep = 0, last = s->motor[m];
        for (int i = 0; i < 5000; ++i) {
            s->Tick();
            worstStep = std::max<int64_t>(worstStep, s->motor[m] - last);
            last = s->motor[m];
        }
        Check(s->rt.AxisMoving(0) && s->rt.AxisVelocityJog(0), "axis jog: a velocity jog runs");
        Check(llabs(s->motor[m] - 7000) <= 2, "axis jog: 1 s at 8000/s after a 0.25 s ramp", (double)s->motor[m], 7000);
        s->rt.AxisStop(0);
        int ticks = 0;
        while (s->rt.AxisMoving(0) && ticks < 20000) {
            s->Tick();
            ++ticks;
        }
        Check(!s->rt.AxisMoving(0) && ticks > 1200 && ticks < 1300, "axis jog: stops in its ramp", ticks, 1250);
        Check(llabs(s->motor[m] - 8000) <= 2, "axis jog: stopping adds half the ramp", (double)s->motor[m], 8000);
        Check(worstStep <= 2, "axis jog: never more than the speed per sample", (double)worstStep, 2);
        Check(s->rt.AxisPosition(0) == s->motor[m], "axis jog: the position is the motor's counts");
        // A move lands exactly, without overshoot, either way.
        for (int64_t target : {int64_t(12345), int64_t(-500), int64_t(-499)}) {
            s->rt.AxisMoveTo(0, target, 6000.0, 30000.0);
            const int64_t from = s->motor[m];
            int64_t over = 0;
            ticks = 0;
            while (s->rt.AxisMoving(0) && ticks < 100000) {
                s->Tick();
                ++ticks;
                over = std::max<int64_t>(over, target > from ? s->motor[m] - target : target - s->motor[m]);
            }
            Check(s->motor[m] == target && s->rt.AxisPosition(0) == target, "axis jog: a move lands on its target",
                  (double)s->motor[m], (double)target);
            if (over > 0) printf("     overshoot %lld moving %lld -> %lld\n", (long long)over, (long long)from, (long long)target);
            Check(over <= 0, "axis jog: a move doesn't overshoot", (double)over, 0);
        }
        Check(s->rt.CamPosition(0, true, &cam) && cam == 0 && s->rt.AxisPosition(0) == -499,
              "axis jog: the cam position doesn't move with the axis");
        // Zero: here is the profile's start; the cam then runs from here.
        s->rt.ZeroAxis(0);
        Check(s->rt.AxisPosition(0) == 0, "axis jog: zero");
        const int64_t base = s->motor[m];
        s->in.encoder = 0;
        s->rt.RequestStart();
        s->Tick();
        for (int e = 1; e <= 1000; ++e) {
            s->in.encoder = e;
            s->Tick();
        }
        const int64_t want = Counts(p.axes[0], 1000 * 360.0 / 4000) - Counts(p.axes[0], 0);
        Check(llabs(s->motor[m] - base - want) <= 1, "axis jog: the cam runs from the zero", (double)(s->motor[m] - base), (double)want);
        Check(s->rt.CamPosition(0, true, &cam) && llabs(cam - s->rt.AxisPosition(0)) <= 1, "axis jog: and stays on its cam position");
        // Engaged: an axis jog does nothing.
        const int64_t engaged = s->motor[m];
        s->rt.AxisJog(0, 8000.0, 32000.0);
        s->Tick(500);
        Check(!s->rt.AxisMoving(0) && s->motor[m] == engaged, "axis jog: refused while the cam has the axis");
        s->rt.RequestStop();
        s->Tick();
        // A start engages an axis that is still jogging: the jog ends.
        s->rt.AxisJog(0, 4000.0, 32000.0);
        s->Tick(200);
        Check(s->rt.AxisMoving(0), "axis jog: jogging again once the cam lets go");
        s->rt.RequestStart();
        s->Tick();
        const int64_t held = s->motor[m];
        s->Tick(100);
        Check(!s->rt.AxisMoving(0) && s->motor[m] == held, "axis jog: a start ends it (the encoder holds still)");
        s->rt.RequestStop();
        s->Tick();
        // Abort ends every jog at once.
        s->rt.AxisJog(0, 4000.0, 32000.0);
        s->Tick(200);
        s->rt.RequestAbort();
        s->Tick();
        const int64_t aborted = s->motor[m];
        s->Tick(100);
        Check(!s->rt.AxisMoving(0) && s->motor[m] == aborted, "axis jog: abort ends it at once");

        // Reversed motor: the steps turn round, the position doesn't.
        CamProject r = DefaultProject();
        r.axes[0].reverse = true;
        auto t = std::make_unique<Sim>();
        t->rt.Build(Reload(r));
        t->rt.AxisMoveTo(0, 2000, 6000.0, 30000.0);
        t->Tick(5000);
        Check(t->rt.AxisPosition(0) == 2000 && t->motor[m] == -2000, "axis jog: a reversed motor steps the other way");

        // A one-shot timeline that finished keeps its axis engaged: axis jog lets it go.
        CamProject q = DefaultProject();
        SetAxisMaster(q, 0, AxisMaster::Timeline);
        q.master.trigger = TriggerSource::Command;
        q.master.timelineRepeat = false;
        NormalizeProject(q);
        auto u = std::make_unique<Sim>();
        Check(u->rt.Build(Reload(q)) == nullptr, "axis jog: timeline build");
        u->rt.RequestStart();
        u->Tick();
        u->rt.RequestTrigger();
        int guard = 0;
        RtStatus st;
        do {
            u->Tick(100);
            u->rt.Status(&st);
        } while (st.timeline != TimelineState::Done && ++guard < 10000);
        Check(st.timeline == TimelineState::Done, "axis jog: the timeline finished");
        const int64_t before = u->rt.AxisPosition(0);
        u->rt.AxisMoveTo(0, before + 800, 6000.0, 30000.0);
        u->Tick(5000);
        Check(u->rt.AxisPosition(0) == before + 800, "axis jog: moves a finished timeline's axis");
        Check(u->rt.CamPosition(0, true, &cam) && cam == 0, "axis jog: a timeline axis's cam position is its start");

        // A gear has no fixed place.
        CamProject g = DefaultProject();
        g.axes[0].gear = true;
        NormalizeProject(g);
        auto w = std::make_unique<Sim>();
        Check(w->rt.Build(Reload(g)) == nullptr, "axis jog: gear build");
        Check(!w->rt.CamPosition(0, true, &cam), "axis jog: a gear has no cam position");
    }

    // ---- homing: a switch at a motor position (tripped beyond it, as a cam-operated switch), the sequence on the axis jog
    // generator, the latch, the home position, then to the cam's start; the failures; Home all in order.
    {
        CamProject p = DefaultProject();  // one crank axis on M-1, 800 counts/mm, no limits set: jog limit 66.7 mm/s
        p.axes.push_back(DefaultAxis("Slide"));
        p.axes[1].connector = 2;
        BoardInput home;
        home.pin = 3;
        home.role = CcioRole::Home;
        home.axis = 0;
        SetBoardInput(p, home);
        NormalizeProject(p);
        const AxisHome &h = p.axes[0].home;
        Check(h.dir == -1 && fabs(h.fast - 53333.0 / 800.0 * 0.1) < 1e-9 && fabs(h.slow - h.fast / 10.0) < 1e-9 && h.thenStart,
              "homing: the defaults (toward -, 10 % of the jog limit, slow a tenth, then to the start)");
        bool lineSeen = false;
        for (const std::string &l : ProfileLines(p)) lineSeen = lineSeen || l.rfind("home 0 dir=- ", 0) == 0;
        Check(lineSeen, "homing: the axis with a home switch has its home line");
        int homeLines = 0;
        for (const std::string &l : ProfileLines(p)) homeLines += l.rfind("home ", 0) == 0 ? 1 : 0;
        Check(homeLines == 1, "homing: only for an axis with a home switch");
        {
            CamProject q = p;
            q.axes[0].home.dir = 1;
            q.axes[0].home.at = -2.5;
            q.axes[0].home.order = 3;
            q.axes[0].home.search = 400;
            q.axes[0].home.required = true;
            q.axes[0].home.thenStart = false;
            const CamProject r = Reload(q);
            const AxisHome &k = r.axes[0].home;
            Check(k.dir == 1 && k.at == -2.5 && k.order == 3 && k.search == 400 && k.required && !k.thenStart && k.fast == q.axes[0].home.fast,
                  "homing: the home line round trip");
            ProfileReader rd;
            rd.Begin();
            for (const std::string &l : ProfileLines(q)) {
                if (l.rfind("input", 0) != 0) rd.Line(l.c_str());
            }
            CamProject out;
            const char *err = rd.Finish(&out);
            Check(err && strstr(err, "no home input"), "homing: a home line without the axis's home input is refused");
            rd.Begin();
            const char *bad = nullptr;
            for (const std::string &l : ProfileLines(q)) {
                std::string s = l;
                if (s.rfind("home ", 0) == 0) s.replace(s.find(" slow="), 1, " slow=99999 x");
                if (!bad) bad = rd.Line(s.c_str());
            }
            Check(bad != nullptr, "homing: a bad home line is refused");
        }
        const int m = p.axes[0].connector;
        const int64_t sw = -3000;  // the switch, in motor counts from where the motor is at the load
        auto s = std::make_unique<Sim>();
        Check(s->rt.Build(Reload(p)) == nullptr, "homing: build");
        Check(s->rt.HasHome(0) && !s->rt.HasHome(1), "homing: axis 0 has its switch, axis 1 none");
        auto run = [&](int64_t swAt, int dir, int maxSamples) {
            int n = 0;
            do {
                s->in.home = static_cast<uint8_t>((dir < 0 ? s->motor[m] <= swAt : s->motor[m] >= swAt) ? 1 : 0);
                s->Tick();
            } while (s->rt.AnyAxisMoving() && ++n < maxSamples);
            return n;
        };
        Check(s->rt.AxisHome(0) && s->rt.Homing(0) && !s->rt.AxisVelocityJog(0), "homing: starts (no deadman on its own jog)");
        Check(!s->rt.AxisHome(1), "homing: an axis without a switch can't");
        int took = run(sw, -1, 200000);
        Check(!s->rt.Homing(0) && (s->rt.HomedMask() & 1) && s->rt.AxisHomeResult(0) == CamRuntime::HomeFail::None, "homing: homed");
        Check(s->rt.AxisLatch(0) == sw, "homing: the latch is the switch's edge, to the count at the slow speed");
        Check(s->rt.AxisPosition(0) == 0 && s->motor[m] == sw, "homing: then at the cam's start, which is the switch (home position 0)");
        printf("     homing took %.2f s\n", took / 5000.0);
        // Again, from 4 mm on the far side: the same place, to the count.
        s->rt.AxisMoveTo(0, 3200, 5000.0, 20000.0);
        s->Tick(10000);
        Check(s->rt.AxisPosition(0) == 3200 && (s->rt.HomedMask() & 1), "homing: an axis jog keeps it homed");
        s->rt.AxisHome(0);
        Check(!(s->rt.HomedMask() & 1), "homing: not homed while it homes again");
        run(sw, -1, 200000);
        Check(s->motor[m] == sw && s->rt.AxisPosition(0) == 0, "homing: again from elsewhere: the same place");
        // Starting on the switch: off it first, then the same.
        s->rt.AxisMoveTo(0, -2000, 5000.0, 20000.0);
        s->Tick(10000);
        s->rt.AxisHome(0);
        run(sw, -1, 200000);
        Check(s->motor[m] == sw && s->rt.AxisPosition(0) == 0 && (s->rt.HomedMask() & 1), "homing: from on the switch: off it, then the same");
        s->rt.ZeroAxis(0);
        Check(!(s->rt.HomedMask() & 1), "homing: zeroing it clears homed");

        // A home position, staying at the switch; toward +.
        {
            CamProject q = p;
            q.axes[0].home.dir = 1;
            q.axes[0].home.at = 5.0;  // mm: 4000 counts
            q.axes[0].home.thenStart = false;
            auto t = std::make_unique<Sim>();
            t->rt.Build(Reload(q));
            std::swap(s, t);
            s->rt.AxisHome(0);
            run(2500, 1, 200000);
            Check((s->rt.HomedMask() & 1) && s->rt.AxisLatch(0) == 2500, "homing +: latched at the switch");
            Check(s->rt.AxisPosition(0) - 4000 == s->motor[m] - 2500, "homing +: the switch is at the home position, 5 mm");
            Check(s->rt.AxisPosition(0) >= 4000 && s->rt.AxisPosition(0) <= 4010, "homing +: it stays at the switch (stopped from the slow speed: v^2 / 2a, 7 counts)");
            std::swap(s, t);
        }
        // Failures.
        {
            CamProject q = p;
            q.axes[0].home.search = 2.0;  // 1600 counts, the switch is 3000 away
            auto t = std::make_unique<Sim>();
            t->rt.Build(Reload(q));
            std::swap(s, t);
            s->rt.AxisHome(0);
            run(sw, -1, 200000);
            Check(s->rt.AxisHomeResult(0) == CamRuntime::HomeFail::NotFound && s->rt.HomeEvents() == 1 && s->rt.HomeEventAxis() == 0 &&
                      !(s->rt.HomedMask() & 1),
                  "homing: not found within the search length: a failure");
            Check(s->motor[m] < -1600 && s->motor[m] > -2400, "homing: it stopped just past the search length");
            std::swap(s, t);
        }
        {
            auto t = std::make_unique<Sim>();
            t->rt.Build(Reload(p));
            std::swap(s, t);
            s->rt.AxisHome(0);
            s->Tick(2000);
            s->in.limitMinus = 1;  // the - limit before the switch
            run(sw, -1, 200000);
            s->in.limitMinus = 0;
            Check(s->rt.AxisHomeResult(0) == CamRuntime::HomeFail::Limit && s->rt.HomeEvents() == 1 && !s->rt.AnyAxisMoving(),
                  "homing: a limit on the search side: a failure, and the axis at rest");
            s->rt.AxisHome(0);
            s->Tick(2000);
            s->rt.AxisStop(0);
            run(sw, -1, 200000);
            Check(s->rt.AxisHomeResult(0) == CamRuntime::HomeFail::Stopped && s->rt.HomeEvents() == 1 && !(s->rt.HomedMask() & 1),
                  "homing: a stop ends it: not homed, not a failure");
            s->rt.AxisHome(0);
            s->Tick(2000);
            s->rt.AxisHalt();
            Check(!s->rt.AnyAxisMoving() && s->rt.AxisHomeResult(0) == CamRuntime::HomeFail::Stopped, "homing: a halt ends it at once");
            Check(s->rt.AxisHomeEnds(0) == 3, "homing: every end counts (failed, stopped, halted)");
            // An axis jog stopped by a limit: counted by side, and where it tripped (the wizard's overtravel).
            s->rt.AxisJog(0, 4000.0, 20000.0);
            s->Tick(2000);
            const int64_t at = s->rt.AxisPosition(0);
            s->in.limitPlus = 1;
            s->Tick(3000);
            s->in.limitPlus = 0;
            Check(s->rt.AxisLimitTrips(0, 0) == 1 && s->rt.AxisLimitTrips(0, 1) == 1 /* the homing test's */ && llabs(s->rt.AxisLimitTripAt(0, 0) - at) <= 2 &&
                      !s->rt.AnyAxisMoving() && s->rt.AxisPosition(0) > at + 100,
                  "limit trip: counted on its side, where it tripped, the axis ran on past it while it ramped");
            s->rt.AxisJog(0, 4000.0, 20000.0);
            s->in.limitPlus = 1;
            s->Tick(3000);
            s->in.limitPlus = 0;
            s->rt.AxisStop(0);
            s->Tick(3000);
            Check(s->rt.AxisLimitTrips(0, 0) == 2, "limit trip: one count per stop, not per sample");
            std::swap(s, t);
        }
        // The homing decel: the run-on past the switch after the search is v^2 / (2 decel); the latch is the same.
        {
            auto runOn = [&](double decel, int64_t *final, std::string *line) {
                CamProject q = p;
                q.axes[0].home.fast = 10.0;  // 8000 counts/s
                q.axes[0].home.slow = 1.0;
                q.axes[0].home.decel = decel;
                NormalizeProject(q);
                for (const std::string &l : ProfileLines(q)) {
                    if (l.rfind("home 0 ", 0) == 0) *line = l;
                }
                auto t = std::make_unique<Sim>();
                t->rt.Build(Reload(q));
                t->rt.AxisHome(0);
                int64_t lowest = 0;
                int n = 0;
                do {
                    t->in.home = static_cast<uint8_t>(t->motor[m] <= sw ? 1 : 0);
                    t->Tick();
                    lowest = std::min(lowest, t->motor[m]);
                } while (t->rt.AnyAxisMoving() && ++n < 400000);
                *final = t->motor[m];
                return sw - lowest;
            };
            int64_t f1 = 0, f2 = 0;
            std::string l1, l2;
            const int64_t asJog = runOn(0.0, &f1, &l1);      // the search speed in a quarter second: 32000 counts/s^2
            const int64_t hard = runOn(1000.0, &f2, &l2);    // 1000 mm/s^2: 800000 counts/s^2
            printf("     run-on past the switch: %lld counts as axis jog, %lld at 1000 mm/s^2\n", (long long)asJog, (long long)hard);
            Check(asJog >= 980 && asJog <= 1020 && hard >= 38 && hard <= 44, "homing decel: the run-on is v^2 / 2a (1000 counts, then 40)");
            Check(f1 == sw && f2 == sw, "homing decel: the latch is the same either way");
            Check(l1.find(" dec=") == std::string::npos && l2.find(" dec=1000") != std::string::npos,
                  "homing decel: dec= only when set (an old profile keeps its text and hash)");
        }
        // Home all: order 1 first, then 2; a failure stops the rest.
        {
            CamProject q = p;
            BoardInput h2;
            h2.pin = 4;
            h2.role = CcioRole::Home;
            h2.axis = 1;
            SetBoardInput(q, h2);
            q.axes[0].home.order = 2;
            q.axes[1].home.order = 1;
            NormalizeProject(q);
            auto t = std::make_unique<Sim>();
            Check(t->rt.Build(Reload(q)) == nullptr, "home all: build");
            const int m2 = q.axes[1].connector;
            t->rt.AxisHomeAll();
            bool axis0Early = false;
            int n = 0;
            do {
                t->in.home = static_cast<uint8_t>((t->motor[m] <= sw ? 1 : 0) | (t->motor[m2] <= -1000 ? 2 : 0));
                t->Tick();
                if (t->rt.Homing(0) && !(t->rt.HomedMask() & 2)) axis0Early = true;
            } while (t->rt.AnyAxisMoving() && ++n < 400000);
            Check(!axis0Early && t->rt.HomedMask() == 3 && t->motor[m] == sw && t->motor[m2] == -1000, "home all: order 1, then order 2; both homed");
        }
    }

    // ---- timeline waits: the clock holds exactly at the wait (every timeline axis and output where it is), runs on from
    // there when the condition comes, a timeout stops it, Stop in a wait stops there, every cycle holds again.
    {
        CamProject p = DefaultProject();  // rise 0..120 deg, dwell 120..180, return, dwell; on the timeline: 2 s at 30 rpm
        SetAxisMaster(p, 0, AxisMaster::Timeline);
        p.master.timelineRepeat = true;
        p.master.trigger = TriggerSource::Command;
        OutputPulse o;
        o.output = 0;
        o.master = AxisMaster::Timeline;
        o.on = 0.8;   // starts exactly at the wait: after it ("wait, then do")
        o.off = 0.9;
        p.outputs.push_back(o);
        TimelineWait w;
        w.t = 0.8;  // in the dwell 0.667..1.0 s
        w.source = WaitSource::Board;
        w.pin = 3;
        w.on = true;
        w.edge = false;  // the level alone (the change is tested below)
        p.waits.push_back(w);
        NormalizeProject(p);
        bool lineSeen = false;
        for (const std::string &l : ProfileLines(p)) {
            lineSeen = lineSeen || (l.rfind("wait 0.8", 0) == 0 && l.size() > 13 && l.compare(l.size() - 11, 11, " board 3 on") == 0);
        }
        Check(lineSeen, "waits: the profile line");
        {
            CamProject q = p;
            q.waits[0].timeout = 1.5;
            q.waits.push_back(w);
            q.waits.back().source = WaitSource::Plc;
            q.waits.back().pin = 7;
            q.waits.back().on = false;
            q.waits.back().t = 1.8;
            const CamProject r = Reload(q);
            Check(r.waits.size() == 2 && r.waits[0].timeout == 1.5 && r.waits[1].source == WaitSource::Plc && r.waits[1].pin == 7 &&
                      !r.waits[1].on && r.waits[1].t == 1.8,
                  "waits: the lines round trip (a timeout, a PLC bit, off)");
            auto refused = [](CamProject bad, const char *why) {
                NormalizeProject(bad);
                ProfileReader rd;
                rd.Begin();
                const char *e = nullptr;
                for (const std::string &l : ProfileLines(bad)) {
                    if (!e) e = rd.Line(l.c_str());
                }
                CamProject out;
                if (!e) e = rd.Finish(&out);
                return e && strstr(e, why);
            };
            CamProject b = p;
            b.waits[0].pin = 0;  // IO-0 is an output pulse's
            Check(refused(b, "output"), "waits: refused on an output's pin");
            b = p;
            b.waits[0].t = 2.5;
            Check(refused(b, "past the end"), "waits: refused past the cycle");
            b = p;
            BoardInput lim;
            lim.pin = 3;
            lim.role = CcioRole::LimitPositive;
            lim.axis = 0;
            SetBoardInput(b, lim);
            Check(refused(b, "home or limit"), "waits: refused on a limit's pin (one job per pin)");
        }
        const int m = p.axes[0].connector;
        auto s = std::make_unique<Sim>();
        Check(s->rt.Build(Reload(p)) == nullptr && s->rt.WaitCount() == 1, "waits: build");
        s->rt.RequestStart();
        s->Tick();
        s->rt.RequestTrigger();
        RtStatus st;
        int n = 0;
        do {
            s->Tick();
            s->rt.Status(&st);
        } while (st.timeline != TimelineState::Waiting && ++n < 20000);
        const int64_t waitQ16 = llround(0.8 * kRtSampleHz * 65536.0);
        Check(st.timeline == TimelineState::Waiting && st.timeQ16 == waitQ16 && st.waitIndex == 1, "waits: holds exactly at 0.8 s");
        Check((s->rt.Outputs() & 1) == 0, "waits: the output that starts at the wait isn't on yet");
        const int64_t at = s->motor[m];
        s->Tick(5000);  // a second
        s->rt.Status(&st);
        Check(st.timeline == TimelineState::Waiting && s->motor[m] == at && st.waitedSamples == 5000, "waits: a second later still holding, nothing moved");
        // The condition: the clock runs on from the wait.
        s->in.waitMet = 1;
        s->Tick();
        s->rt.Status(&st);
        Check(st.timeline == TimelineState::Running && st.timeQ16 > waitQ16 && st.timeQ16 - waitQ16 <= 65536 + 1 && (s->rt.Outputs() & 1),
              "waits: the input on: runs on from the wait, the output comes on");
        s->in.waitMet = 0;
        int64_t biggest = 0, last = s->motor[m];
        n = 0;
        do {  // the rest of the cycle, into the next one's wait
            s->Tick();
            biggest = std::max<int64_t>(biggest, llabs(s->motor[m] - last));
            last = s->motor[m];
            s->rt.Status(&st);
        } while (st.timeline != TimelineState::Waiting && ++n < 20000);
        Check(st.timeline == TimelineState::Waiting && st.cycle == 1 && st.timeQ16 == waitQ16 && s->motor[m] == at && s->rt.WaitHolds(0) == 2,
              "waits: the next cycle holds again, at the same place");
        Check(biggest < 200, "waits: no jump on the way (the steps as drawn)");
        // Stop in a wait: it stops there; it doesn't finish the cycle.
        s->rt.RequestCycleStop();
        s->Tick(100);
        s->rt.Status(&st);
        Check(st.timeline == TimelineState::Idle && st.timeQ16 == waitQ16 && s->motor[m] == at, "waits: Stop in a wait stops there");
        // Skip (simulate): passes the one holding.
        s->rt.RequestStart();
        s->Tick();
        s->rt.RequestTrigger();
        n = 0;
        do {
            s->Tick();
            s->rt.Status(&st);
        } while (st.timeline != TimelineState::Waiting && ++n < 20000);
        s->rt.RequestWaitSkip();
        s->Tick();
        s->rt.Status(&st);
        Check(st.timeline == TimelineState::Running, "waits: a skip passes it");
        // Met on arrival: it doesn't stop.
        s->in.waitMet = 1;
        n = 0;
        bool stopped = false;
        do {
            s->Tick();
            s->rt.Status(&st);
            stopped = stopped || st.timeline == TimelineState::Waiting;
        } while (st.cycle < 3 && ++n < 30000);
        Check(!stopped && st.cycle >= 3, "waits: the input already on: it passes without stopping");
        s->in.waitMet = 0;
        // A timeout: the timeline stops at the wait, an event for the engine's fault.
        CamProject q = p;
        q.waits[0].timeout = 0.2;
        auto t = std::make_unique<Sim>();
        t->rt.Build(Reload(q));
        t->rt.RequestStart();
        t->Tick();
        t->rt.RequestTrigger();
        t->Tick(4000 + 1000 + 10);  // 0.8 s to the wait, 0.2 s of it
        t->rt.Status(&st);
        Check(st.timeline == TimelineState::Idle && st.timeQ16 == waitQ16 && t->rt.WaitEvents() == 1 && t->rt.WaitEventIndex() == 0 &&
                  t->rt.WaitTimeouts(0) == 1,
              "waits: timed out after 0.2 s: stopped there, the event");
        // Abort in a wait.
        t->rt.RequestStart();
        t->Tick();
        t->rt.RequestTrigger();
        t->Tick(4100);
        t->rt.RequestAbort();
        t->Tick();
        t->rt.Status(&st);
        Check(st.timeline == TimelineState::Idle && st.waitIndex == 0, "waits: abort ends it");

        // Needs a change: a stuck sensor holds it; a change at any time since it last passed lets it pass.
        CamProject e = p;
        e.waits[0].edge = true;
        bool edgeLine = false;
        for (const std::string &l : ProfileLines(e)) edgeLine = edgeLine || (l.rfind("wait ", 0) == 0 && l.find(" on edge") != std::string::npos);
        Check(edgeLine && Reload(e).waits[0].edge && !Reload(p).waits[0].edge, "wait change: the line's edge, and a line without it is the level alone");
        auto u = std::make_unique<Sim>();
        u->rt.Build(Reload(e));
        auto toWait = [&](Sim &x) {
            RtStatus z;
            int k = 0;
            do {
                x.Tick();
                x.rt.Status(&z);
            } while (z.timeline != TimelineState::Waiting && z.timeline != TimelineState::Done && ++k < 20000);
            return z.timeline;
        };
        u->in.waitMet = 1;  // on all along, since before Start: stuck
        u->rt.RequestStart();
        u->Tick();
        u->rt.RequestTrigger();
        Check(toWait(*u) == TimelineState::Waiting && !u->rt.WaitArmed(0), "wait change: on since before Start (stuck): it holds");
        u->Tick(2000);
        u->rt.Status(&st);
        Check(st.timeline == TimelineState::Waiting, "wait change: ... and keeps holding while it stays on");
        u->in.waitMet = 0;  // off, then on again: a change
        u->in.waitOpposite = 1;
        u->Tick(5);
        u->in.waitMet = 1;
        u->in.waitOpposite = 0;
        u->Tick();
        u->rt.Status(&st);
        Check(st.timeline == TimelineState::Running && !u->rt.WaitArmed(0), "wait change: off then on while it holds: it passes (and needs another change)");
        // The next cycle: still on (never off since it passed): it holds.
        n = 0;
        do {
            u->Tick();
            u->rt.Status(&st);
        } while (st.timeline != TimelineState::Waiting && ++n < 20000);
        Check(st.timeline == TimelineState::Waiting && st.cycle == 1, "wait change: the next cycle, the sensor never off: it holds");
        // Off and back on before the timeline reaches it: it passes without stopping (the change wasn't missed).
        u->in.waitMet = 0;
        u->in.waitOpposite = 1;
        u->Tick(3);
        u->in.waitMet = 1;
        u->in.waitOpposite = 0;
        u->Tick();
        bool held = false;
        n = 0;
        do {  // into the next cycle: the input goes off at its start and on again well before 0.8 s
            u->Tick();
            u->rt.Status(&st);
            if (st.cycle == 2 && st.timeQ16 < 2000 * 65536LL) {
                u->in.waitMet = 0;
                u->in.waitOpposite = 1;
            } else if (st.cycle == 2) {
                u->in.waitMet = 1;
                u->in.waitOpposite = 0;
            }
            held = held || (st.cycle == 2 && st.timeline == TimelineState::Waiting);
        } while (st.cycle < 3 && ++n < 40000);
        Check(!held && st.cycle >= 3, "wait change: off and on again before the timeline gets there: it passes without stopping");
        // A level wait with the same stuck sensor passes every time.
        auto v = std::make_unique<Sim>();
        v->rt.Build(Reload(p));
        v->in.waitMet = 1;
        v->rt.RequestStart();
        v->Tick();
        v->rt.RequestTrigger();
        v->Tick(3 * 10000 + 100);
        v->rt.Status(&st);
        Check(st.cycle >= 3, "wait level: the same stuck sensor passes every cycle (the level alone)");
    }

    // ---- operator inputs: the panel's filter, edges (never a level at power-up), held Disable / Stop, maintained Enable
    {
        // Enable IO-3 (NO), Disable IO-4 (NC), Fault reset IO-5 (NO), Start A-10 (NO), Stop A-11 (NC).
        const int8_t pins[kOpInputs] = {3, 4, 5, 10, 11};
        const uint8_t nc = (1u << kOpDisable) | (1u << kOpStop);
        const uint16_t rest = (1u << 4) | (1u << 11);  // the NC buttons closed: high at rest
        OperatorPanel op;
        auto run = [&](uint16_t levels, int samples) {
            uint8_t act = 0;
            for (int i = 0; i < samples; ++i) {
                op.Sample(levels);
                if (i % 25 == 0) act |= op.Poll();  // the main loop, now and then
            }
            return static_cast<uint8_t>(act | op.Poll());
        };
        // Start held when the panel is set up (power-up): nothing, then not even later while it stays held.
        op.Configure(pins, nc, false);
        Check(run(rest | (1u << 10), 400) == 0, "panel: Start held at power-up doesn't start");
        Check(run(rest | (1u << 10), 400) == 0, "panel: ... nor while it stays held");
        Check(run(rest, 200) == 0 && run(rest | (1u << 10), 200) == kActStart, "panel: released and pressed again: Start");
        Check(run(rest | (1u << 10), 500) == 0, "panel: one press, one start (held: nothing more)");
        run(rest, 200);
        // Bounce: 5 ms on is not a press (the filter is 10 ms).
        Check(run(rest | (1u << 3), 25) == 0 && run(rest, 200) == 0, "panel: 5 ms of bounce is no press");
        Check(run(rest | (1u << 3), 200) == kActEnable, "panel: Enable pressed: enable");
        run(rest, 200);
        // Disable (NC): pressed = open = low.
        Check(run(rest & ~(1u << 4), 200) == kActDisable, "panel: Disable (NC) opened: disable");
        Check(run((rest & ~(1u << 4)) | (1u << 3), 200) == 0, "panel: Enable while Disable is held: nothing");
        run(rest, 200);
        Check(run(rest | (1u << 5), 200) == kActReset, "panel: Fault reset");
        run(rest, 200);
        Check(run(rest & ~(1u << 11), 200) == kActStop && run((rest & ~(1u << 11)) | (1u << 10), 200) == 0, "panel: Stop, and no Start while it is held");
        run(rest, 200);
        // A broken wire on Disable reads as pressed: it disables.
        Check(run(rest & ~(1u << 4), 200) == kActDisable, "panel: a cut Disable wire (NC) disables");
        run(rest, 200);
        // Maintained: an Enable switch: on enables, off disables.
        op.Configure(pins, nc, true);
        run(rest, 400);
        Check(run(rest | (1u << 3), 200) == kActEnable && run(rest, 200) == kActDisable, "panel maintained: switch on enables, off disables");
        // A maintained switch already on at power-up: not enabled until it goes off and on.
        op.Configure(pins, nc, true);
        Check(run(rest | (1u << 3), 600) == 0, "panel maintained: on at power-up: not enabled");
        Check(run(rest, 200) == kActDisable && run(rest | (1u << 3), 200) == kActEnable, "panel maintained: off, then on: enabled");
        // The bench's forcing (op_test).
        op.Configure(pins, nc, false);
        run(rest, 400);
        op.Test(kOpStart, 1);
        Check(run(rest, 10) == kActStart && (op.Pressed() & (1u << kOpStart)), "panel: op_test press: Start");
        op.Test(kOpStart, -1);
        Check(run(rest, 10) == 0 && !(op.Pressed() & (1u << kOpStart)), "panel: op_test off: back to the wire");
        // No pins: nothing at all.
        const int8_t none[kOpInputs] = {-1, -1, -1, -1, -1};
        op.Configure(none, 0, false);
        Check(!op.Any() && run(0xFFFF, 400) == 0, "panel: none set up: nothing");
    }

    printf(g_fail ? "%d check(s) FAILED\n" : "all checks passed\n", g_fail);
    return g_fail ? 1 : 0;
}
