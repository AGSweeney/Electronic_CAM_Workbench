/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
// Headless checks of the motion laws and the profile model. No Qt.
// Exit code 0 = all passed.

#include "CamLaws.h"
#include "CamPoints.h"
#include "CamProfile.h"
#include "CamTemplates.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

using namespace ecam;

static int g_fail = 0;

static void Check(bool ok, const char *what, double got = 0.0, double want = 0.0) {
    if (!ok) {
        ++g_fail;
        std::printf("FAIL %s (got %.6f, want %.6f)\n", what, got, want);
    }
}

static bool Near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

struct Peak {
    MotionLaw law;
    double cv, ca;  // published peak velocity / acceleration coefficients (Norton)
};

int main() {
    const Peak peaks[] = {
        {MotionLaw::Linear, 1.0, 0.0},
        {MotionLaw::Harmonic, 1.5708, 4.9348},
        {MotionLaw::Cycloidal, 2.0, 6.2832},
        {MotionLaw::ModifiedTrapezoid, 2.0, 4.8881},
        {MotionLaw::ModifiedSine, 1.7596, 5.5280},
        {MotionLaw::Poly345, 1.875, 5.7735},
        {MotionLaw::Poly4567, 2.1875, 7.5132},
    };

    for (const Peak &pk : peaks) {
        const char *name = LawName(pk.law);
        const LawValue a = EvaluateLaw(pk.law, 0.0);
        const LawValue b = EvaluateLaw(pk.law, 1.0);
        char msg[128];
        std::snprintf(msg, sizeof msg, "%s s(0)=0", name);
        Check(Near(a.s, 0.0, 1e-12), msg, a.s, 0.0);
        std::snprintf(msg, sizeof msg, "%s s(1)=1", name);
        Check(Near(b.s, 1.0, 1e-12), msg, b.s, 1.0);
        if (LawStartsAndEndsAtRest(pk.law)) {
            std::snprintf(msg, sizeof msg, "%s v(0)=0", name);
            Check(Near(a.v, 0.0, 1e-9), msg, a.v, 0.0);
            std::snprintf(msg, sizeof msg, "%s v(1)=0", name);
            Check(Near(b.v, 0.0, 1e-9), msg, b.v, 0.0);
        }

        // Derivatives agree with finite differences; find the peaks.
        double cv = 0, ca = 0;
        const double h = 1e-6;
        for (int i = 1; i < 2000; ++i) {
            const double u = i / 2000.0;
            const LawValue m = EvaluateLaw(pk.law, u - h);
            const LawValue c = EvaluateLaw(pk.law, u);
            const LawValue p = EvaluateLaw(pk.law, u + h);
            cv = std::fmax(cv, std::fabs(c.v));
            ca = std::fmax(ca, std::fabs(c.a));
            std::snprintf(msg, sizeof msg, "%s ds/du at u=%.4f", name, u);
            Check(Near((p.s - m.s) / (2 * h), c.v, 1e-5 * (1 + std::fabs(c.v))), msg, (p.s - m.s) / (2 * h), c.v);
            std::snprintf(msg, sizeof msg, "%s dv/du at u=%.4f", name, u);
            Check(Near((p.v - m.v) / (2 * h), c.a, 1e-4 * (1 + std::fabs(c.a))), msg, (p.v - m.v) / (2 * h), c.a);
            // Jerk steps at the piece joins (multiples of 1/8) of the trapezoid and sine laws.
            const double eighths = u * 8.0;
            if (std::fabs(eighths - std::round(eighths)) < 1e-3) continue;
            std::snprintf(msg, sizeof msg, "%s da/du at u=%.4f", name, u);
            Check(Near((p.a - m.a) / (2 * h), c.j, 1e-3 * (1 + std::fabs(c.j))), msg, (p.a - m.a) / (2 * h), c.j);
        }
        std::snprintf(msg, sizeof msg, "%s peak velocity", name);
        Check(Near(cv, pk.cv, 2e-3), msg, cv, pk.cv);
        std::snprintf(msg, sizeof msg, "%s peak acceleration", name);
        Check(Near(ca, pk.ca, 5e-3), msg, ca, pk.ca);
        std::printf("%-20s Cv %.4f  Ca %.4f\n", name, cv, ca);
    }

    // Profile model.
    CamAxis ax = DefaultAxis("t");
    Check(Near(Evaluate(ax, 0).pos, 0.0, 1e-12), "default start");
    Check(Near(Evaluate(ax, 120).pos, 20.0, 1e-9), "default top", Evaluate(ax, 120).pos, 20.0);
    Check(Near(Evaluate(ax, 60).pos, 10.0, 1e-9), "cycloid midpoint", Evaluate(ax, 60).pos, 10.0);
    Check(Analyze(ax).issues.empty(), "default profile has no issues");

    // Dragging the high dwell's knot moves the whole dwell.
    SetKnotPos(ax, 1, 25.0);
    Check(Near(KnotPos(ax, 2), 25.0, 1e-12), "dwell follows knot");
    // Closed: moving the start moves the end.
    SetKnotPos(ax, 0, 2.0);
    Check(Near(KnotPos(ax, 4), 2.0, 1e-12), "closed end follows start");
    Check(Near(KnotPos(ax, 3), 2.0, 1e-12), "closed end dwell follows start");

    // Split keeps the curve.
    const double before = Evaluate(ax, 80).pos;
    const int s = SplitAt(ax, 50.0);
    Check(s == 1, "split index", s, 1);
    Check(Near(KnotPos(ax, 1), Evaluate(ax, 50).pos, 1e-12), "split knot on curve");
    DeleteSegment(ax, 1);
    Check(Near(Evaluate(ax, 80).pos, before, 1e-9), "delete restores", Evaluate(ax, 80).pos, before);

    // A linear segment next to a dwell is a velocity step.
    SetSegmentLaw(ax, 0, MotionLaw::Linear);
    bool step = false;
    for (const CamIssue &i : Analyze(ax).issues) step |= (i.kind == IssueKind::VelocityStep && i.knot == 0);
    Check(step, "linear rise flags a velocity step at 0 deg");

    // Open profile: end is free.
    SetClosed(ax, false);
    SetKnotPos(ax, 4, 10.0);
    Check(Near(KnotPos(ax, 0), 2.0, 1e-12), "open start stays");
    Check(Near(Analyze(ax).advancePerRev, 8.0, 1e-12), "open advance");

    // Speed limits. Default profile: cycloidal rise of 20 over 120 deg, so per degree
    // peak v = 2*20/120 = 1/3 and peak a = 2*pi*20/120^2.
    {
        CamAxis lim = DefaultAxis("lim");
        Check(std::isinf(MaxMasterSpeed(lim, 0.0).maxDegPerSec), "no limits: unlimited speed");
        lim.velMax = 100.0;  // units/s -> 300 deg/s
        SpeedLimit l = MaxMasterSpeed(lim, 0.0, 36000);
        Check(Near(l.maxDegPerSec, 300.0, 0.01) && l.reason == SpeedLimit::Velocity, "velocity-limited speed",
              l.maxDegPerSec, 300.0);
        lim.velMax = 0.0;
        lim.accMax = 1000.0;  // units/s^2
        const double peakA = 2 * 3.14159265358979 * 20.0 / (120.0 * 120.0);
        l = MaxMasterSpeed(lim, 0.0, 36000);
        Check(Near(l.maxDegPerSec, std::sqrt(1000.0 / peakA), 0.05) && l.reason == SpeedLimit::Accel,
              "acceleration-limited speed", l.maxDegPerSec, std::sqrt(1000.0 / peakA));
        Check(Near(l.maxRampDegPerSec2, 3000.0, 0.01), "max ramp = A / peak v", l.maxRampDegPerSec2, 3000.0);
        const double steady = l.maxDegPerSec;
        l = MaxMasterSpeed(lim, 1500.0, 36000);
        Check(l.maxDegPerSec < steady && l.maxDegPerSec > 0.0, "a crank ramp lowers the safe speed", l.maxDegPerSec, steady);
        l = MaxMasterSpeed(lim, 3100.0, 36000);
        Check(l.maxDegPerSec == 0.0 && l.reason == SpeedLimit::Ramp, "too steep a ramp is never safe");
    }

    // Timeline axes.
    {
        CamProject pr = DefaultProject();
        pr.master.rpm = 30.0;  // 2 s per revolution
        const double atCrank = Evaluate(pr.axes[0], 90.0).pos;
        SetAxisMaster(pr, 0, AxisMaster::Timeline);
        CamAxis &ta = pr.axes[0];
        Check(Near(ta.span, 2.0, 1e-12) && !ta.cyclic, "crank -> timeline: one rev at 30 rpm = 2 s", ta.span, 2.0);
        Check(Near(Evaluate(ta, 0.5).pos, atCrank, 1e-9), "same motion at the same moment", Evaluate(ta, 0.5).pos, atCrank);
        Check(Near(Evaluate(ta, -1.0).pos, ta.startPos, 1e-12) && Evaluate(ta, -1.0).vel == 0.0, "rests before t = 0");
        Check(Near(Evaluate(ta, 9.0).pos, ta.segments.back().endPos, 1e-12) && Evaluate(ta, 9.0).vel == 0.0,
              "rests after its end");
        Check(Analyze(ta).issues.empty(), "rise/dwell/return/dwell starts and ends at rest");

        // Dragging the last knot sets the duration.
        SetKnotDeg(ta, KnotCount(ta) - 1, 3.0);
        Check(Near(ta.span, 3.0, 1e-12) && Near(ta.segments.back().endDeg, 3.0, 1e-12), "last knot sets the duration");

        // A linear first segment starts with a velocity step from rest.
        SetSegmentLaw(ta, 0, MotionLaw::Linear);
        bool fromRest = false;
        for (const CamIssue &i : Analyze(ta).issues) fromRest |= (i.kind == IssueKind::VelocityStep && i.knot == 0);
        Check(fromRest, "timeline: a linear start is a velocity step from rest");

        // Repeat: only the axis as long as the cycle wraps end -> start.
        pr.axes.push_back(DefaultAxis("b"));
        SetAxisMaster(pr, 1, AxisMaster::Timeline);  // 2 s, shorter than axis 0 (3 s)
        pr.master.timelineRepeat = true;
        NormalizeProject(pr);
        Check(Near(TimelineLength(pr), 3.0, 1e-12), "timeline length = longest axis");
        Check(pr.axes[0].cyclic && !pr.axes[1].cyclic, "repeat: the longest axis wraps, the shorter one rests");

        // Back to the crank: duration stretches to 360 deg.
        SetAxisMaster(pr, 1, AxisMaster::Crank);
        Check(pr.axes[1].cyclic && Near(pr.axes[1].span, 360.0, 1e-12) && Near(pr.axes[1].segments.back().endDeg, 360.0, 1e-9),
              "timeline -> crank: 360 deg, cyclic");
        Check(UsesCrank(pr) && UsesTimeline(pr), "crank and timeline axes in one project");
        // Speed limit on the timeline is the max override: cycloid 20 over 2/3 s at 100 % peaks at 60 /s.
        CamProject q = DefaultProject();
        q.master.rpm = 30.0;
        SetAxisMaster(q, 0, AxisMaster::Timeline);
        q.axes[0].velMax = 30.0;
        Check(Near(MaxMasterSpeed(q.axes[0], 0.0, 36000).maxDegPerSec, 0.5, 1e-3), "max override = velMax / peak v",
              MaxMasterSpeed(q.axes[0], 0.0, 36000).maxDegPerSec, 0.5);
    }

    // Mechanics: drive -> counts / unit, crank ppr x gear -> counts / rev.
    {
        CamProject mp = DefaultProject();
        CamAxis &ax = mp.axes[0];
        ax.drive = AxisDrive::Ballscrew;
        ax.motorPpr = 800;
        ax.screwPitch = 5;  // mm per rev
        NormalizeProject(mp);
        Check(Near(mp.axes[0].countsPerUnit, 160.0, 1e-12), "ballscrew: 800 ppr / 5 mm = 160 counts/mm",
              mp.axes[0].countsPerUnit, 160.0);
        mp.axes[0].drive = AxisDrive::Belt;
        mp.axes[0].pulleyPd = 20;
        NormalizeProject(mp);
        Check(Near(mp.axes[0].countsPerUnit, 800.0 / (3.14159265358979323846 * 20.0), 1e-9),
              "belt: 800 ppr / (pi x 20 mm PD)", mp.axes[0].countsPerUnit, 12.7324);
        mp.axes[0].drive = AxisDrive::Direct;
        mp.axes[0].countsPerUnit = 123.0;
        NormalizeProject(mp);
        Check(Near(mp.axes[0].countsPerUnit, 123.0, 1e-12), "direct: counts / unit kept");

        mp.master.crankPpr = 800;
        mp.master.crankGearNum = 10;
        mp.master.crankGearDen = 1;
        NormalizeProject(mp);
        Check(Near(mp.master.countsPerRev, 8000.0, 1e-9) && Near(CrankUnitsPerRev(mp.master), 8000.0, 1e-9),
              "crank: 800 ppr x 10 : 1 = 8000 counts / rev");
        mp.master.crankGearNum = 50;
        mp.master.crankGearDen = 7;
        NormalizeProject(mp);
        Check(Near(mp.master.countsPerRev, 800.0 * 50.0 / 7.0, 1e-9) && Near(CrankUnitsPerRev(mp.master), 40000.0, 1e-9),
              "crank: 800 ppr x 50 : 7 = 5714.29 counts / rev, 40000 units of 1/7 count", mp.master.countsPerRev, 5714.2857);
    }

    // Blend: a flying cutoff. Dwell, blend up to line speed, linear sync at 1 unit/deg, blend down,
    // return, dwell. No velocity or acceleration step anywhere, and the sync is exactly line speed.
    {
        CamAxis fc = DefaultAxis("cut");
        fc.segments.clear();
        auto add = [&](double end, double pos, MotionLaw law) {
            CamSegment s;
            s.endDeg = end;
            s.endPos = pos;
            s.law = law;
            fc.segments.push_back(s);
        };
        add(40, 0, MotionLaw::Dwell);
        add(100, 30, MotionLaw::Blend);
        add(160, 90, MotionLaw::Linear);
        add(220, 120, MotionLaw::Blend);
        add(330, 0, MotionLaw::Cycloidal);
        add(360, 0, MotionLaw::Dwell);
        Normalize(fc);
        const AxisStats st = Analyze(fc);
        int steps = 0;
        for (const CamIssue &i : st.issues) {
            steps += i.kind == IssueKind::VelocityStep || i.kind == IssueKind::AccelStep;
        }
        Check(steps == 0, "blend: flying cutoff has no velocity or acceleration steps", steps, 0);
        Check(Near(Evaluate(fc, 130).vel, 1.0, 1e-12), "blend: sync at line speed", Evaluate(fc, 130).vel, 1.0);
        Check(Near(EvaluateSegment(fc, 1, 1.0).vel, 1.0, 1e-12) && Near(EvaluateSegment(fc, 1, 0.0).vel, 0.0, 1e-12),
              "blend: 0 -> line speed");
        Check(Near(EvaluateSegment(fc, 3, 0.0).vel, 1.0, 1e-12) && Near(EvaluateSegment(fc, 3, 1.0).vel, 0.0, 1e-12),
              "blend: line speed -> 0");
        Check(Near(EvaluateSegment(fc, 1, 0.0).acc, 0.0, 1e-12) && Near(EvaluateSegment(fc, 1, 1.0).acc, 0.0, 1e-12),
              "blend: zero acceleration at both ends");
        // Derivatives agree with finite differences across a blend with moving neighbours.
        bool fdOk = true;
        for (double d = 160.5; d < 220.0; d += 1.0) {
            const double e = 1e-4;
            const CamPoint m = Evaluate(fc, d - e), c = Evaluate(fc, d), p = Evaluate(fc, d + e);
            fdOk = fdOk && Near((p.pos - m.pos) / (2 * e), c.vel, 1e-6) && Near((p.vel - m.vel) / (2 * e), c.acc, 1e-6) &&
                   Near((p.acc - m.acc) / (2 * e), c.jerk, 1e-6);
        }
        Check(fdOk, "blend: v, a, j agree with finite differences");
        // Between two rests a blend is the 3-4-5 polynomial.
        CamAxis b = DefaultAxis("b");
        SetSegmentLaw(b, 0, MotionLaw::Blend);
        CamAxis q = DefaultAxis("q");
        SetSegmentLaw(q, 0, MotionLaw::Poly345);
        Check(Near(Evaluate(b, 47).pos, Evaluate(q, 47).pos, 1e-12), "blend between rests = poly 3-4-5");
        MotionLaw k;
        Check(LawFromKey("blend", &k) && k == MotionLaw::Blend, "blend: file key");
    }

    // Templates (the setup wizard): every default builds a smooth, valid project.
    for (int t = 0; t < static_cast<int>(TemplateKind::Count); ++t) {
        const TemplateKind kind = static_cast<TemplateKind>(t);
        const TemplateSpec spec = DefaultTemplate(kind);
        CamProject p = BuildTemplate(spec);
        const char *name = TemplateName(kind);
        bool ok = !p.axes.empty();
        for (const CamAxis &a : p.axes) {
            ok = ok && !a.segments.empty() && std::fabs(a.segments.back().endDeg - a.span) < 1e-9;
            const AxisStats st = Analyze(a);
            for (const CamIssue &is : st.issues) {
                if (is.kind == IssueKind::VelocityStep) {
                    std::printf("  %s / %s: velocity step at %.2f\n", name, a.name.c_str(), is.deg);
                    ok = false;
                }
            }
        }
        Check(ok, name);
        // Normalizing again changes nothing: the builder already left it valid.
        CamProject q = p;
        NormalizeProject(q);
        bool same = q.axes.size() == p.axes.size();
        for (size_t i = 0; same && i < p.axes.size(); ++i) same = q.axes[i].segments.size() == p.axes[i].segments.size();
        Check(same, "template: stable under NormalizeProject");
    }
    {
        // Rise / dwell / return: the phases land where asked, peak stroke is the stroke.
        TemplateSpec s = DefaultTemplate(TemplateKind::RiseDwellReturn);
        s.stroke = 30.0, s.a0 = 20.0, s.a1 = 110.0, s.a2 = 200.0, s.a3 = 290.0;
        const CamAxis a = BuildTemplate(s).axes[0];
        Check(a.segments.size() == 5, "rdr: 5 segments", a.segments.size(), 5);
        Check(Near(Evaluate(a, 150.0).pos, 30.0, 1e-9), "rdr: high dwell at the stroke");
        Check(Near(Evaluate(a, 10.0).pos, 0.0, 1e-9) && Near(Evaluate(a, 300.0).pos, 0.0, 1e-9), "rdr: low dwells");
        // Out of order phases are put in order.
        s.a1 = 5.0, s.a3 = 100.0;
        const CamAxis b = BuildTemplate(s).axes[0];
        bool inc = true;
        for (size_t i = 1; i < b.segments.size(); ++i) inc = inc && b.segments[i].endDeg > b.segments[i - 1].endDeg;
        Check(inc && b.segments.back().endDeg == 360.0, "rdr: phases clamped into order");
    }
    {
        // Index feed: open, advances the feed every cycle.
        const CamProject p = BuildTemplate(DefaultTemplate(TemplateKind::IndexFeed));
        Check(!p.axes[0].closed && Near(Analyze(p.axes[0]).advancePerRev, 150.0, 1e-9), "index feed: advances 150 / rev");
        // Timeline move: duration is the sum of the phases, triggered.
        TemplateSpec t = DefaultTemplate(TemplateKind::TriggeredMove);
        t.delay = 0.1;
        const CamProject tp = BuildTemplate(t);
        Check(tp.axes[0].master == AxisMaster::Timeline && Near(tp.axes[0].span, 0.1 + 0.6 + 0.4 + 0.6, 1e-12),
              "triggered move: duration", tp.axes[0].span, 1.7);
    }
    {
        // Flying cut-off: the carriage rides the line exactly during the sync, the saw plunges inside it.
        const TemplateSpec s = DefaultTemplate(TemplateKind::FlyingCutoff);
        const CamProject p = BuildTemplate(s);
        int ax = -1;
        double s0 = 0, s1 = 0;
        Check(p.master.lineMode && LineSync(p, &ax, &s0, &s1) && ax == 0, "flying: line mode with a sync");
        const double k = s.cutLength / 360.0;
        const double mid = Evaluate(p.axes[0], (s0 + s1) / 2).vel;
        Check(Near(mid, k, 1e-9), "flying: carriage at line speed", mid, k);
        Check(Near(Analyze(p.axes[0]).maxPos, FlyingCarriageStroke(s), 1e-9), "flying: stroke", Analyze(p.axes[0]).maxPos,
              FlyingCarriageStroke(s));
        Check(p.axes.size() == 2 && Evaluate(p.axes[1], s0 - 0.5).pos == 0.0 && Evaluate(p.axes[1], s1 + 0.5).pos == 0.0 &&
                  Near(Analyze(p.axes[1]).maxPos, s.sawStroke, 1e-9),
              "flying: saw plunges within the sync");
        Check(LineWaitDeg(p) > 0.0 && Near(p.master.rpm, s.lineSpeed * 1000.0 / s.cutLength, 1e-9), "flying: wait and cuts/min");
        // Phases that do not fit are squeezed, not dropped.
        TemplateSpec big = s;
        big.rampDeg = 90, big.syncDeg = 120, big.returnDeg = 120;
        const CamProject bp = BuildTemplate(big);
        Check(bp.axes[0].segments.size() == 6 && LineSync(bp, &ax, &s0, &s1), "flying: oversize phases squeezed");
        // The saw motor has its own mechanics and limits, independent of the carriage's.
        TemplateSpec own = s;
        own.motorPpr = 800, own.screwPitch = 10;          // carriage: 80 counts/mm
        own.sawMotorPpr = 1600, own.sawPitch = 4;         // saw: 400 counts/mm
        own.sawConnector = 2, own.sawName = "Blade";
        own.sawVelMax = 75, own.sawAccMax = 900;
        const CamProject op = BuildTemplate(own);
        Check(op.axes.size() == 2 && Near(op.axes[0].countsPerUnit, 80, 1e-9) && Near(op.axes[1].countsPerUnit, 400, 1e-9) &&
                  op.axes[1].connector == 2 && op.axes[1].name == "Blade" && op.axes[1].velMax == 75 && op.axes[1].accMax == 900 &&
                  op.axes[0].velMax == 0,
              "flying: saw motor mechanics and limits are its own");
        own.sawDrive = AxisDrive::Belt, own.sawPulleyPd = 20;
        Check(Near(BuildTemplate(own).axes[1].countsPerUnit, 1600 / (3.14159265358979323846 * 20), 1e-9), "flying: saw belt drive");
        own.saw = false;
        Check(BuildTemplate(own).axes.size() == 1, "flying: no saw axis when unticked");
    }

    // ---- Cubic spline (docs/MATH.md, "Cubic spline")
    {
        const double kPi2 = 2 * 3.14159265358979323846;
        auto spline = [](bool cyclic, double span, double start, std::vector<std::pair<double, double>> knots) {
            CamAxis a;
            a.span = span;
            a.cyclic = cyclic;
            a.master = cyclic ? AxisMaster::Crank : AxisMaster::Timeline;
            a.closed = false;
            a.startPos = start;
            for (const auto &k : knots) a.segments.push_back({k.first, k.second, MotionLaw::Spline});
            return a;
        };
        auto hasIssue = [](const AxisStats &st, IssueKind kind) {
            for (const CamIssue &i : st.issues) {
                if (i.kind == kind) return true;
            }
            return false;
        };

        // Alone between two rests: the law's own closed form.
        {
            CamAxis a = spline(false, 2.0, 1.0, {{2.0, 5.0}});
            for (double u = 0; u <= 1.0; u += 0.125) {
                const LawValue l = EvaluateLaw(MotionLaw::Spline, u);
                const CamPoint p = EvaluateSegment(a, 0, u);
                Check(Near(p.pos, 1.0 + 4.0 * l.s, 1e-12) && Near(p.vel, 4.0 * l.v / 2, 1e-12) &&
                          Near(p.acc, 4.0 * l.a / 4, 1e-12) && Near(p.jerk, 4.0 * l.j / 8, 1e-12),
                      "spline: one segment between rests is 3u^2 - 2u^3", p.pos, 1.0 + 4.0 * l.s);
            }
        }
        // Any cubic with the right end slopes comes back exactly, on uneven knots.
        {
            auto f = [](double x) { return (3 - 2 * x) * x * x; };  // f'(0) = f'(1) = 0
            CamAxis a = spline(false, 1.0, 0.0, {{0.17, f(0.17)}, {0.3, f(0.3)}, {0.55, f(0.55)}, {0.9, f(0.9)}, {1.0, 1.0}});
            double worst = 0;
            for (int i = 0; i <= 1000; ++i) worst = std::max(worst, std::fabs(Evaluate(a, i / 1000.0).pos - f(i / 1000.0)));
            Check(worst < 1e-12, "spline: a cubic is reproduced exactly on uneven knots", worst, 0);
        }
        // The reference's equations, on the solved M: mu M- + 2 M + lambda M+ = 6 f[x-, x, x+]; clamped rows.
        {
            CamAxis a = spline(false, 10.0, 0.0, {{1.5, 2.0}, {4.0, -1.0}, {5.0, 3.0}, {8.0, 3.5}, {10.0, 0.0}});
            const SplineFit fit = FitSplines(a);
            const int n = static_cast<int>(a.segments.size());
            std::vector<double> x(n + 1), y(n + 1), M(n + 1);
            for (int k = 0; k <= n; ++k) {
                x[k] = KnotDeg(a, k);
                y[k] = KnotPos(a, k);
                M[k] = k < n ? fit.m0[k] : fit.m1[n - 1];
            }
            for (int k = 1; k < n; ++k) {
                Check(Near(fit.m1[k - 1], fit.m0[k], 1e-12), "spline: M agrees on both sides of a knot", fit.m1[k - 1], fit.m0[k]);
                const double h0 = x[k] - x[k - 1], h1 = x[k + 1] - x[k];
                const double mu = h0 / (h0 + h1), lambda = h1 / (h0 + h1);
                const double dd = ((y[k + 1] - y[k]) / h1 - (y[k] - y[k - 1]) / h0) / (x[k + 1] - x[k - 1]);
                Check(Near(mu * M[k - 1] + 2 * M[k] + lambda * M[k + 1], 6 * dd, 1e-10), "spline: the three-moment equation holds",
                      mu * M[k - 1] + 2 * M[k] + lambda * M[k + 1], 6 * dd);
            }
            Check(Near(2 * M[0] + M[1], 6 / (x[1] - x[0]) * ((y[1] - y[0]) / (x[1] - x[0]) - 0.0), 1e-10), "spline: clamped first row");
            Check(Near(M[n - 1] + 2 * M[n], 6 / (x[n] - x[n - 1]) * (0.0 - (y[n] - y[n - 1]) / (x[n] - x[n - 1])), 1e-10),
                  "spline: clamped last row");
            for (int k = 0; k <= n; ++k) {
                Check(Near(Evaluate(a, std::min(x[k], 10.0 - 1e-12)).pos, y[k], 1e-9), "spline: passes through every knot");
            }
            // Derivatives against finite differences, off the knots.
            const double h = 1e-5;
            for (double d = 0.21; d < 9.9; d += 0.37) {
                const CamPoint p = Evaluate(a, d), lo = Evaluate(a, d - h), hi = Evaluate(a, d + h);
                if (lo.segment != hi.segment) continue;
                Check(Near(p.vel, (hi.pos - lo.pos) / (2 * h), 1e-6), "spline: velocity is the position's derivative");
                Check(Near(p.acc, (hi.vel - lo.vel) / (2 * h), 1e-6), "spline: acceleration is the velocity's derivative");
                Check(Near(p.jerk, (hi.acc - lo.acc) / (2 * h), 1e-6), "spline: jerk is the acceleration's derivative");
            }
            // A timeline run starts and ends at rest; continuous through its knots.
            const AxisStats st = Analyze(a);
            Check(!hasIssue(st, IssueKind::VelocityStep), "spline: no velocity step, from rest to rest");
            Check(Near(EvaluateSegment(a, 0, 0.0).vel, 0, 1e-12) && Near(EvaluateSegment(a, n - 1, 1.0).vel, 0, 1e-12), "spline: clamped to rest");
        }
        // Natural end beside a Blend, worked by hand: knots (40, 0) (41, 1) (42, 0) give M = 0, -3, 0.
        {
            CamAxis a;
            a.startPos = 0;
            a.segments = {{40, 0, MotionLaw::Blend}, {41, 1, MotionLaw::Spline}, {42, 0, MotionLaw::Spline}, {360, 0, MotionLaw::Blend}};
            const SplineFit fit = FitSplines(a);
            Check(Near(fit.m0[1], 0, 1e-12) && Near(fit.m1[1], -3, 1e-12) && Near(fit.m1[2], 0, 1e-12), "spline: natural ends, by hand", fit.m1[1], -3);
            // A slope of 1.5 at the first knot (s = x - x^3/2 ... ) and the Blend takes it, with no step at all.
            Check(Near(EvaluateSegment(a, 1, 0.0).vel, 1.5, 1e-12), "spline: natural end's slope", EvaluateSegment(a, 1, 0.0).vel, 1.5);
            Check(Near(EvaluateSegment(a, 0, 1.0).vel, 1.5, 1e-12), "spline: the Blend meets the spline's velocity");
            const AxisStats st = Analyze(a);
            Check(!hasIssue(st, IssueKind::VelocityStep) && !hasIssue(st, IssueKind::AccelStep), "spline: C2 with a Blend at both ends");
        }
        // Clamped to a Linear neighbour: no velocity step; an acceleration step, reported.
        {
            CamAxis a;
            a.startPos = 0;
            a.segments = {{90, 90, MotionLaw::Linear}, {180, 100, MotionLaw::Spline}, {270, 50, MotionLaw::Spline}, {360, 0, MotionLaw::Linear}};
            Normalize(a);
            a.closed = false;
            Check(Near(EvaluateSegment(a, 1, 0.0).vel, 1.0, 1e-12), "spline: takes the Linear segment's velocity");
            Check(Near(EvaluateSegment(a, 2, 1.0).vel, EvaluateSegment(a, 3, 0.0).vel, 1e-12), "spline: and the next one's");
            bool velStepAtJoin = false;
            for (const CamIssue &i : Analyze(a).issues) velStepAtJoin |= i.kind == IssueKind::VelocityStep && (i.knot == 1 || i.knot == 3);
            Check(!velStepAtJoin, "spline: no velocity step at a clamped join");
            Check(hasIssue(Analyze(a), IssueKind::AccelStep), "spline: a clamped join's acceleration step is reported");
        }
        // Next to a dwell, across the wrap: one run over 360 -> 0.
        {
            CamAxis a;
            a.startPos = 0;
            a.segments = {{60, 8, MotionLaw::Spline}, {120, 8, MotionLaw::Dwell}, {240, 8, MotionLaw::Dwell}, {300, 4, MotionLaw::Spline}, {360, 0, MotionLaw::Spline}};
            Normalize(a);
            const SplineFit fit = FitSplines(a);
            Check(Near(fit.m1[4], fit.m0[0], 1e-12), "spline: a run wraps through 360 (same M)", fit.m1[4], fit.m0[0]);
            Check(Near(EvaluateSegment(a, 4, 1.0).vel, EvaluateSegment(a, 0, 0.0).vel, 1e-12), "spline: and the same velocity");
            Check(Near(EvaluateSegment(a, 0, 1.0).vel, 0, 1e-12) && Near(EvaluateSegment(a, 3, 0.0).vel, 0, 1e-12), "spline: at rest into the dwell");
        }
        // The whole cycle, closed: periodic, close to the sine it samples (error ~ h^4), C2 through the wrap.
        for (int n : {2, 3, 4, 7, 12, 24}) {
            CamAxis a;
            a.startPos = 0;
            for (int k = 1; k <= n; ++k) a.segments.push_back({360.0 * k / n, std::sin(kPi2 * k / n), MotionLaw::Spline});
            a.segments.back().endPos = 0;
            const SplineFit fit = FitSplines(a);
            Check(Near(fit.m1[n - 1], fit.m0[0], 1e-12), "spline: periodic M through the wrap", fit.m1[n - 1], fit.m0[0]);
            for (int k = 1; k < n; ++k) Check(Near(fit.m1[k - 1], fit.m0[k], 1e-12), "spline: periodic M at a knot");
            const AxisStats st = Analyze(a);
            Check(!hasIssue(st, IssueKind::VelocityStep) && !hasIssue(st, IssueKind::AccelStep), "spline: periodic has no step anywhere");
            // The periodic equation also at knot 0, with the wrapped neighbours.
            const double h = 360.0 / n, d0 = (KnotPos(a, 1) - KnotPos(a, 0)) / h, dn = (KnotPos(a, n) - KnotPos(a, n - 1)) / h;
            const double lhs = 0.5 * fit.m0[n - 1] + 2 * fit.m0[0] + 0.5 * fit.m0[1 % n];
            Check(n == 2 || Near(lhs, 6 * (d0 - dn) / (2 * h), 1e-12), "spline: three-moment equation at the wrap", lhs, 6 * (d0 - dn) / (2 * h));
            if (n >= 12) {
                double worst = 0;
                for (int i = 0; i < 3600; ++i) worst = std::max(worst, std::fabs(Evaluate(a, i / 10.0, &fit).pos - std::sin(kPi2 * i / 3600)));
                Check(worst < (n >= 24 ? 2e-5 : 3e-4), "spline: periodic stays on the sine", worst, 0);
            }
        }
        // Periodic on an open crank axis: the velocity wraps, the advance stays.
        {
            CamAxis a = spline(true, 360.0, 0.0, {{90, 10}, {180, 40}, {270, 55}, {360, 80}});
            const SplineFit fit = FitSplines(a);
            Check(Near(EvaluateSegment(a, 3, 1.0, &fit).vel, EvaluateSegment(a, 0, 0.0, &fit).vel, 1e-12), "spline: open and periodic: velocity wraps");
            Check(Near(fit.m1[3], fit.m0[0], 1e-12), "spline: open and periodic: M wraps");
            Check(Near(EvaluateSegment(a, 3, 1.0, &fit).pos, 80, 1e-9), "spline: open and periodic: ends at its advance");
        }
        // One piece around the whole cycle is a straight line.
        {
            CamAxis a = spline(true, 360.0, 0.0, {{360, 36}});
            Check(Near(Evaluate(a, 90).pos, 9, 1e-12) && Near(Evaluate(a, 90).vel, 0.1, 1e-12), "spline: one periodic piece is a line");
        }
        // Splitting: a new knot on the curve changes nothing (the spline through the same data is unique).
        for (int cyc = 0; cyc < 2; ++cyc) {
            CamAxis a = cyc ? spline(true, 360.0, 0.0, {{70, 3}, {150, 9}, {290, 2}, {360, 0}})
                            : spline(false, 4.0, 1.0, {{1, 3}, {2.5, 2}, {4, 5}});
            a.closed = cyc != 0;
            const CamAxis before = a;
            const double at = a.span * 0.41;
            Check(SplitAt(a, at) >= 0, "spline: split");
            double worst = 0;
            for (int i = 0; i < 500; ++i) {
                const double d = a.span * i / 500;
                worst = std::max(worst, std::fabs(Evaluate(a, d).pos - Evaluate(before, d).pos));
            }
            Check(worst < 1e-9, "spline: splitting leaves the curve alone", worst, 0);
        }
        // Editing one knot reshapes the run, not the far side of a dwell.
        {
            CamAxis a;
            a.startPos = 0;
            a.segments = {{60, 5, MotionLaw::Spline}, {120, 8, MotionLaw::Spline}, {180, 8, MotionLaw::Dwell}, {240, 3, MotionLaw::Spline}, {360, 0, MotionLaw::Spline}};
            Normalize(a);
            const double far = Evaluate(a, 300).pos, nearby = Evaluate(a, 90).pos;
            SetKnotPos(a, 1, 6.5);
            Check(Near(Evaluate(a, 300).pos, far, 1e-12) || a.closed, "spline: a dwell ends the run");
            Check(!Near(Evaluate(a, 90).pos, nearby, 1e-6), "spline: a knot's neighbours move with it");
        }
        // Overshoot: a steep rise to a plateau of knots swings past them.
        {
            CamAxis a = spline(false, 6.0, 0.0, {{1, 0}, {2, 10}, {3, 10}, {6, 10}});
            const AxisStats st = Analyze(a);
            Check(hasIssue(st, IssueKind::SplineOvershoot), "spline: overshoot reported");
            for (const CamIssue &i : st.issues) {
                if (i.kind == IssueKind::SplineOvershoot) {
                    Check(i.left > 10.2 && Near(i.right, 10, 1e-12), "spline: overshoot past the top knot", i.left, 10);
                }
            }
            CamAxis ok = spline(false, 4.0, 0.0, {{1, 1}, {2, 2}, {3, 3}, {4, 4}});
            Check(!hasIssue(Analyze(ok), IssueKind::SplineOvershoot), "spline: none on a steady rise");
        }
        // Keys and limits go through the usual paths.
        MotionLaw k;
        Check(LawFromKey("spline", &k) && k == MotionLaw::Spline && std::string(LawName(k)) == "Cubic spline", "spline: file key");
        CamAxis lim = spline(false, 2.0, 0.0, {{1, 3}, {2, 0}});
        lim.velMax = 2;
        Check(MaxMasterSpeed(lim, 0).reason == SpeedLimit::Velocity, "spline: speed limit sees the curve");
    }

    // ---- Points to a spline
    {
        std::vector<CurvePoint> pts;
        std::string err;
        Check(ParsePoints("0,0\n90,1.5\n180,2\n# a comment\n\n360,0\n", &pts, &err) && pts.size() == 4 && pts[2].y == 2, "points: comma columns");
        Check(ParsePoints("0;0,5\n90;1,25\n", &pts, &err) && pts[0].y == 0.5 && pts[1].y == 1.25, "points: semicolon columns, decimal comma");
        Check(ParsePoints("0\t0,5\n90\t1,25\n", &pts, &err) && pts[1].y == 1.25, "points: tab columns, decimal comma");
        Check(ParsePoints("0 1\n1 2\n", &pts, &err) && pts[1].y == 2, "points: space columns");
        Check(ParsePoints("\xEF\xBB\xBFx,y\r\n0,1\r\n1,2\r\n", &pts, &err) && pts.size() == 2, "points: byte order mark, heading, CRLF");
        Check(ParsePoints("master_deg,master_counts,pos_mm,motor_counts\n0,0,0.6,96\n1,11,0.7,104\n", &pts, &err) && pts[1].y == 0.7,
              "points: an exported table reads its pos column");
        Check(ParsePoints("0,1\n0,1\n1,2\n", &pts, &err) && pts.size() == 2, "points: an exact repeat is dropped");
        Check(!ParsePoints("0,1\n0,2\n", &pts, &err) && err.find("Line 2") == 0, "points: two positions at one x refused");
        Check(!ParsePoints("5,1\n2,2\n", &pts, &err) && err.find("increase") != std::string::npos, "points: x must increase");
        Check(!ParsePoints("0,1\nabc,2\n", &pts, &err) && err.find("Line 2") == 0, "points: a bad line named");
        Check(!ParsePoints("0\n1\n", &pts, &err), "points: needs a second column");
        Check(!ParsePoints("0,1\n", &pts, &err), "points: needs two points");

        // Crank: closed data, the whole cycle: periodic through every point.
        std::vector<CurvePoint> c;
        for (int k = 0; k <= 12; ++k) c.push_back({30.0 * k, std::sin(2 * 3.14159265358979323846 * k / 12)});
        c.back().y = c.front().y;
        CamAxis a = DefaultAxis("x");
        PointsFit info;
        Check(FitAxisToPoints(a, c, 64, &info, &err) && info.knots == 13 && info.closed && a.closed && info.maxError < 1e-9,
              "points: crank, every point a knot, closed", info.maxError, 0);
        Check(a.segments.size() == 12 && a.segments[5].law == MotionLaw::Spline && Near(a.segments.back().endDeg, 360, 1e-12), "points: all Spline");

        // Any x range maps onto the crank; open data leaves it open.
        std::vector<CurvePoint> o = {{10, 1}, {20, 4}, {30, 9}};
        CamAxis b = DefaultAxis("y");
        Check(FitAxisToPoints(b, o, 64, &info, &err) && !b.closed && Near(b.segments[0].endDeg, 180, 1e-9) && b.startPos == 1, "points: x maps to 0..360, open");
        Check(info.firstX == 10 && info.lastX == 30, "points: the range is reported");

        // The timeline: the x range is the duration.
        CamAxis t = DefaultAxis("t");
        t.master = AxisMaster::Timeline;
        t.cyclic = false;
        Check(FitAxisToPoints(t, {{2, 0}, {2.5, 3}, {4, 1}}, 64, &info, &err) && Near(t.span, 2, 1e-12) && Near(t.segments[0].endDeg, 0.5, 1e-12), "points: timeline duration");

        // Too many points: thinned to the limit, the fit stays close, ends kept.
        std::vector<CurvePoint> many;
        for (int k = 0; k <= 700; ++k) {
            const double u = k / 700.0;
            many.push_back({360 * u, std::sin(2 * 3.14159265358979323846 * u) + 0.3 * std::sin(6 * 3.14159265358979323846 * u)});
        }
        many.back().y = many.front().y;
        CamAxis m = DefaultAxis("m");
        Check(FitAxisToPoints(m, many, 32, &info, &err) && info.knots == 33 && static_cast<int>(m.segments.size()) == 32, "points: thinned to the limit");
        Check(info.maxError < 0.002 && info.points == 701 && Near(m.segments.back().endDeg, 360, 1e-12), "points: thinned curve stays on the data", info.maxError, 0);

        // Closer than the minimum segment: merged, and the last point survives.
        CamAxis d = DefaultAxis("d");
        Check(FitAxisToPoints(d, {{0, 0}, {0.1, 0.1}, {100, 2}, {359.9, 1}, {360, 1}}, 64, &info, &err) && d.segments.size() == 2 &&
                  Near(d.segments.back().endPos, 1, 1e-12), "points: too close merged");
        Check(!FitAxisToPoints(d, {{3, 0}, {3, 1}}, 64, &info, &err), "points: no x range refused");
    }

    // ---- Electronic gear: the model
    {
        CamProject p = DefaultProject();
        CamAxis &g = p.axes[0];
        g.gear = true;
        g.gearNum = -3;
        g.gearDen = 2;
        g.master = AxisMaster::Timeline;
        NormalizeProject(p);
        Check(g.master == AxisMaster::Crank && !g.closed && g.cyclic && g.span == 360, "gear: an open crank axis");
        Check(g.segments.size() == 1 && g.segments[0].law == MotionLaw::Linear && g.segments[0].endPos == -1.5, "gear: one Linear segment", g.segments[0].endPos, -1.5);
        Check(Near(Evaluate(g, 180).pos, -0.75, 1e-12) && Near(Evaluate(g, 180).vel, -1.5 / 360, 1e-12), "gear: evaluates as its ratio");
        Check(Analyze(g).issues.empty(), "gear: smooth through the wrap");
        Check(Near(Analyze(g).advancePerRev, -1.5, 1e-12), "gear: its advance is the ratio");
        g.gearNum = 0;
        g.gearDen = 0;
        Normalize(g);
        Check(g.gearNum == 1 && g.gearDen == 1, "gear: a zero repaired");
    }

    std::printf(g_fail ? "%d check(s) FAILED\n" : "all checks passed\n", g_fail);
    return g_fail ? 1 : 0;
}
