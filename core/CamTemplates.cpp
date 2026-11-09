/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "CamTemplates.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ecam {

const char *TemplateName(TemplateKind kind) {
    switch (kind) {
        case TemplateKind::RiseDwellReturn: return "Rise, dwell, return";
        case TemplateKind::IndexFeed: return "Index feed";
        case TemplateKind::Oscillate: return "Oscillate";
        case TemplateKind::TriggeredMove: return "Triggered move";
        case TemplateKind::FlyingCutoff: return "Flying cut-off";
        case TemplateKind::Count: break;
    }
    return "";
}

const char *TemplateSummary(TemplateKind kind) {
    switch (kind) {
        case TemplateKind::RiseDwellReturn:
            return "Move out, wait, come back and wait, once every machine cycle: a pusher, an ejector, a classic "
                   "cam follower.";
        case TemplateKind::IndexFeed:
            return "Advance a fixed distance every machine cycle and stop: a conveyor, a roll feed, an index "
                   "table. The position keeps adding up.";
        case TemplateKind::Oscillate:
            return "Go out and back continuously with no stops: a traverse, a wrapping arm, a shaker.";
        case TemplateKind::TriggeredMove:
            return "No machine shaft: move out, hold and come back once each time an input (or a command) "
                   "triggers it: pick and place, a gate, a stamp.";
        case TemplateKind::FlyingCutoff:
            return "An encoder wheel measures the moving material; a carriage matches its speed while a saw "
                   "cuts, then returns for the next piece. The cut length can be changed while running.";
        case TemplateKind::Count: break;
    }
    return "";
}

bool TemplateOnTimeline(TemplateKind kind) { return kind == TemplateKind::TriggeredMove; }
bool TemplateUsesLine(TemplateKind kind) { return kind == TemplateKind::FlyingCutoff; }

TemplateSpec DefaultTemplate(TemplateKind kind) {
    TemplateSpec s;
    s.kind = kind;
    switch (kind) {
        case TemplateKind::RiseDwellReturn:
            s.name = "Pusher";
            s.stroke = 50.0;
            s.a0 = 0.0, s.a1 = 120.0, s.a2 = 180.0, s.a3 = 300.0;
            break;
        case TemplateKind::IndexFeed:
            s.name = "Feed";
            s.drive = AxisDrive::Belt;
            s.pulleyPd = 50.0;
            s.stroke = 150.0;
            s.a0 = 0.0, s.a1 = 180.0;
            s.law = MotionLaw::ModifiedSine;
            break;
        case TemplateKind::Oscillate:
            s.name = "Traverse";
            s.drive = AxisDrive::Belt;
            s.pulleyPd = 40.0;
            s.stroke = 100.0;
            s.a1 = 180.0;
            break;
        case TemplateKind::TriggeredMove:
            s.name = "Transfer";
            s.drive = AxisDrive::Belt;
            s.pulleyPd = 25.0;
            s.stroke = 200.0;
            s.law = MotionLaw::ModifiedTrapezoid;
            s.delay = 0.0, s.moveTime = 0.6, s.holdTime = 0.4, s.returnTime = 0.6;
            break;
        case TemplateKind::FlyingCutoff:
            s.name = "Carriage";
            s.drive = AxisDrive::Ballscrew;
            s.screwPitch = 10.0;
            s.rpm = 0.0;  // from the line speed
            s.cutLength = 600.0;
            s.kerf = 3.0;
            s.lineSpeed = 5.0;
            break;
        case TemplateKind::Count: break;
    }
    return s;
}

namespace {

constexpr double kGap = 1.0;  // smallest phase the builder keeps, degrees (seconds / 360 on the timeline)

struct Builder {
    CamAxis &a;
    double minLen;
    // Appends a segment ending at `end` (clamped to stay in order and leave room before `limit`), unless
    // it would be shorter than minLen (a phase of zero length is simply left out).
    void Add(double end, double pos, MotionLaw law, double limit) {
        const double start = a.segments.empty() ? 0.0 : a.segments.back().endDeg;
        end = std::min(end, limit);
        if (end - start < minLen) return;
        a.segments.push_back({end, pos, law});
    }
};

CamAxis BaseAxis(const TemplateSpec &s) {
    CamAxis a;
    a.name = s.name.empty() ? std::string("Axis 1") : s.name;
    a.units = s.units.empty() ? std::string("mm") : s.units;
    a.connector = std::clamp(s.connector, 0, kMotorConnectors - 1);
    a.drive = s.drive;
    a.motorPpr = std::max(1.0, s.motorPpr);
    a.screwPitch = s.screwPitch > 0 ? s.screwPitch : 5.0;
    a.pulleyPd = s.pulleyPd > 0 ? s.pulleyPd : 20.0;
    a.countsPerUnit = s.countsPerUnit > 0 ? s.countsPerUnit : 1.0;
    if (a.drive != AxisDrive::Direct) a.countsPerUnit = DriveCountsPerUnit(a);
    a.velMax = std::max(0.0, s.velMax);
    a.accMax = std::max(0.0, s.accMax);
    a.startPos = 0.0;
    return a;
}

void CrankMaster(const TemplateSpec &s, CamMaster &m) {
    m.source = s.source;
    m.crankPpr = std::max(1.0, s.encoderPpr);
    m.crankGearNum = std::max(1, s.gearNum);
    m.crankGearDen = std::max(1, s.gearDen);
    m.rpm = s.rpm > 0 ? s.rpm : 30.0;
    m.crankConnector = std::clamp(s.crankConnector, 0, kMotorConnectors - 1);
    m.crankRampRpmPerSec = std::max(1.0, s.crankRampRpmPerSec);
    m.crankMaxRpm = std::max(1.0, s.crankMaxRpm);
}

// Roomy cam drawing radii for the stroke, so the pressure angle stays moderate.
void CamRadii(CamAxis &a, double stroke) {
    a.baseRadius = std::max(40.0, std::ceil(std::fabs(stroke) * 1.6 / 10.0) * 10.0);
    a.rollerRadius = std::max(8.0, std::round(a.baseRadius / 5.0));
}

}  // namespace

double FlyingCarriageStroke(const TemplateSpec &s) {
    const double k = s.cutLength / 360.0;  // mm of line per crank degree
    return k * (s.rampDeg + s.syncDeg);    // half of each ramp at line speed, plus the sync
}

double FlyingWaitDeg(const TemplateSpec &s) {
    constexpr double kEndDwell = 6.0;
    return 360.0 - kEndDwell - s.returnDeg - 2.0 * s.rampDeg - s.syncDeg;
}

CamProject BuildTemplate(const TemplateSpec &spec) {
    TemplateSpec s = spec;
    CamProject p;
    CamAxis a = BaseAxis(s);
    const double stroke = s.stroke;

    switch (s.kind) {
        case TemplateKind::RiseDwellReturn: {
            CrankMaster(s, p.master);
            double a0 = std::clamp(s.a0, 0.0, 360.0), a1 = std::clamp(s.a1, 0.0, 360.0);
            double a2 = std::clamp(s.a2, 0.0, 360.0), a3 = std::clamp(s.a3, 0.0, 360.0);
            // Keep the phases in order with room for each move.
            a1 = std::max(a1, a0 + 10.0);
            a2 = std::max(a2, a1);
            a3 = std::max(a3, a2 + 10.0);
            if (a3 > 360.0) {  // squeeze back into the revolution
                const double k = 360.0 / a3;
                a0 *= k, a1 *= k, a2 *= k, a3 = 360.0;
            }
            Builder b{a, kGap};
            b.Add(a0, 0.0, MotionLaw::Dwell, 360.0);
            b.Add(a1, stroke, s.law, 360.0);
            b.Add(a2, stroke, MotionLaw::Dwell, 360.0);
            b.Add(a3, 0.0, s.law, 360.0);
            b.Add(360.0, 0.0, MotionLaw::Dwell, 360.0);
            if (a.segments.empty() || a.segments.back().endDeg < 360.0) {
                // The return ran to 360: stretch the last move to the end of the revolution.
                if (!a.segments.empty()) a.segments.back().endDeg = 360.0;
            }
            a.closed = true;
            CamRadii(a, stroke);
            break;
        }
        case TemplateKind::IndexFeed: {
            CrankMaster(s, p.master);
            double a0 = std::clamp(s.a0, 0.0, 350.0);
            double a1 = std::clamp(std::max(s.a1, a0 + 10.0), 0.0, 360.0);
            Builder b{a, kGap};
            b.Add(a0, 0.0, MotionLaw::Dwell, 360.0);
            b.Add(a1, stroke, s.law, 360.0);
            b.Add(360.0, stroke, MotionLaw::Dwell, 360.0);
            if (a.segments.back().endDeg < 360.0) a.segments.back().endDeg = 360.0;
            a.closed = false;
            CamRadii(a, stroke);
            break;
        }
        case TemplateKind::Oscillate: {
            CrankMaster(s, p.master);
            const double a1 = std::clamp(s.a1, 30.0, 330.0);
            a.segments = {{a1, stroke, s.law}, {360.0, 0.0, s.law}};
            a.closed = true;
            CamRadii(a, stroke);
            break;
        }
        case TemplateKind::TriggeredMove: {
            CamMaster &m = p.master;
            m.trigger = s.trigger;
            m.triggerInput = std::clamp(s.triggerInput, 0, kTriggerInputs - 1);
            m.triggerRising = s.triggerRising;
            m.timelineRepeat = s.repeat;
            m.speedOverride = 1.0;
            a.master = AxisMaster::Timeline;
            const double minT = 0.005;
            const double delay = std::max(0.0, s.delay), out = std::max(minT, s.moveTime);
            const double hold = std::max(0.0, s.holdTime), back = std::max(minT, s.returnTime);
            double t = 0.0;
            if (delay >= minT) a.segments.push_back({t += delay, 0.0, MotionLaw::Dwell});
            a.segments.push_back({t += out, stroke, s.law});
            if (hold >= minT) a.segments.push_back({t += hold, stroke, MotionLaw::Dwell});
            a.segments.push_back({t += back, 0.0, s.law});
            a.span = t;
            a.closed = true;
            break;
        }
        case TemplateKind::FlyingCutoff: {
            CamMaster &m = p.master;
            m.source = MasterSource::Encoder;
            m.lineMode = true;
            m.crankPpr = std::max(1.0, s.encoderPpr);
            m.wheelCircumference = std::max(1.0, s.wheelCircumference);
            m.designLength = std::max(10.0, s.cutLength);
            m.kerf = std::max(0.0, s.kerf);
            m.rpm = std::max(0.01, s.lineSpeed * 1000.0 / m.designLength);  // cuts per minute
            s.cutLength = m.designLength;

            const double k = m.designLength / 360.0;
            double ramp = std::max(6.0, s.rampDeg), sync = std::max(6.0, s.syncDeg), ret = std::max(12.0, s.returnDeg);
            constexpr double kEndDwell = 6.0;
            double wait = 360.0 - kEndDwell - ret - 2.0 * ramp - sync;
            if (wait < 6.0) {  // too long: shorten the return first, then everything
                ret = std::max(12.0, ret - (6.0 - wait));
                wait = 360.0 - kEndDwell - ret - 2.0 * ramp - sync;
                if (wait < 6.0) {
                    const double f = (360.0 - kEndDwell - 6.0) / (ret + 2.0 * ramp + sync);
                    ret *= f, ramp *= f, sync *= f;
                    wait = 6.0;
                }
            }
            const double up = k * ramp / 2.0;      // ramping from rest to line speed covers half speed x time
            const double ride = k * sync;
            const double full = up + ride + up;
            double x = wait;
            a.segments = {
                {x, 0.0, MotionLaw::Dwell},
                {x += ramp, up, MotionLaw::Blend},
                {x += sync, up + ride, MotionLaw::Linear},
                {x += ramp, full, MotionLaw::Blend},
                {x += ret, 0.0, MotionLaw::Poly345},
                {360.0, 0.0, MotionLaw::Dwell},
            };
            a.closed = true;
            CamRadii(a, full);
            const double s0 = wait + ramp, s1 = s0 + sync;

            if (s.saw) {
                TemplateSpec sawSpec = s;
                sawSpec.name = s.sawName.empty() ? std::string("Saw plunge") : s.sawName;
                sawSpec.units = s.sawUnits;
                sawSpec.connector = s.sawConnector;
                sawSpec.drive = s.sawDrive;
                sawSpec.motorPpr = s.sawMotorPpr;
                sawSpec.screwPitch = s.sawPitch;
                sawSpec.pulleyPd = s.sawPulleyPd;
                sawSpec.countsPerUnit = s.sawCountsPerUnit;
                sawSpec.velMax = s.sawVelMax;
                sawSpec.accMax = s.sawAccMax;
                CamAxis saw = BaseAxis(sawSpec);
                const double w = sync;
                double y = s0 + w / 12.0;
                saw.segments = {
                    {y, 0.0, MotionLaw::Dwell},
                    {y += w / 3.0, s.sawStroke, MotionLaw::Cycloidal},
                    {y += w / 6.0, s.sawStroke, MotionLaw::Dwell},
                    {y += w / 3.0, 0.0, MotionLaw::Cycloidal},
                    {360.0, 0.0, MotionLaw::Dwell},
                };
                saw.closed = true;
                CamRadii(saw, s.sawStroke);
                p.axes.push_back(a);
                p.axes.push_back(saw);
            } else {
                p.axes.push_back(a);
            }
            if (s.sawOutput) p.outputs.push_back({0, AxisMaster::Crank, s0, s1});
            NormalizeProject(p);
            return p;
        }
        case TemplateKind::Count: break;
    }
    p.axes.push_back(a);
    NormalizeProject(p);
    return p;
}

}  // namespace ecam
