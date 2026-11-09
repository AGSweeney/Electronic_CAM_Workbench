/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "ProjectReview.h"

#include <QLocale>
#include <QStringList>

#include <algorithm>
#include <cmath>

using namespace ecam;

namespace review {

QString Q(const std::string &s) { return QString::fromStdString(s); }

QString InputName(int i) {
    if (i < 6) return QStringLiteral("IO-%1").arg(i);
    if (i < 9) return QStringLiteral("DI-%1").arg(i);
    return QStringLiteral("A-%1").arg(i);
}

QString LawText(MotionLaw law) { return QString::fromLatin1(LawName(law)); }

QString DriveText(AxisDrive d) {
    switch (d) {
        case AxisDrive::Direct: return QStringLiteral("Direct");
        case AxisDrive::Ballscrew: return QStringLiteral("Ballscrew");
        case AxisDrive::Belt: return QStringLiteral("Belt / pulley");
    }
    return {};
}

QString TriggerText(const CamMaster &m) {
    switch (m.trigger) {
        case TriggerSource::Command: return QStringLiteral("Command");
        case TriggerSource::DigitalInput:
            return QStringLiteral("%1, %2 edge").arg(InputName(m.triggerInput),
                                                    m.triggerRising ? QStringLiteral("rising") : QStringLiteral("falling"));
        case TriggerSource::EtherNetIP: return QStringLiteral("PLC, EtherNet/IP TRIGGER bit");
    }
    return {};
}

QString XUnit(const CamAxis &a) { return OnTimeline(a) ? QStringLiteral("s") : QStringLiteral("°"); }

QString Sig(double v) {
    if (!std::isfinite(v)) return QStringLiteral("—");
    return QString::number(v, 'g', 6);
}

CamPoint SampleAt(const CamAxis &a, int i, int n) {
    if (i >= n) return EvaluateSegment(a, static_cast<int>(a.segments.size()) - 1, 1.0);
    return Evaluate(a, a.span * i / n);
}

CamPoint TimelineAt(const CamAxis &a, double t) {
    if (a.cyclic && t >= a.span) return EvaluateSegment(a, static_cast<int>(a.segments.size()) - 1, 1.0);
    return Evaluate(a, t);
}

Peaks SegmentPeaks(const CamAxis &a, int seg) {
    Peaks p;
    constexpr int kN = 96;
    for (int i = 0; i <= kN; ++i) {
        const CamPoint c = EvaluateSegment(a, seg, static_cast<double>(i) / kN);
        p.vel = std::max(p.vel, std::fabs(c.vel));
        p.acc = std::max(p.acc, std::fabs(c.acc));
        p.jerk = std::max(p.jerk, std::fabs(c.jerk));
    }
    return p;
}

int ProjectInfo::errors() const {
    return static_cast<int>(std::count_if(checks.begin(), checks.end(), [](const Check &c) { return c.level == Check::Error; }));
}

int ProjectInfo::warnings() const { return static_cast<int>(checks.size()) - errors(); }

ProjectInfo AnalyzeProject(const CamProject &project, int estopPin, const int *opPins) {
    ProjectInfo pi;
    const CamMaster &m = project.master;
    if (m.source == MasterSource::DrivenCrank) {
        pi.crankMaxRpm = m.crankMaxRpm;
        pi.crankReason = QStringLiteral("crank motor limit");
    }
    for (const CamAxis &a : project.axes) {
        AxisInfo ai;
        ai.stats = Analyze(a);
        ai.w = MasterRate(m, a);
        const double ramp = AxisRamp(m, a);
        ai.limit = MaxMasterSpeed(a, ramp, 1440);
        const double peakSteps = ai.stats.maxVel * ai.w * a.countsPerUnit;
        ai.stepRateHz = peakSteps;
        ai.motorPeakRpm = peakSteps / std::max(a.motorPpr, 1.0) * 60.0;
        for (int i = 0; i < 1440; ++i) {
            const CamPoint p = Evaluate(a, a.span * i / 1440);
            ai.worstAcc = std::max(ai.worstAcc, std::fabs(p.acc) * ai.w * ai.w + std::fabs(p.vel) * ramp);
        }
        const QString name = Q(a.name);
        if (a.master == AxisMaster::Crank) {
            const double rpm = ai.limit.maxDegPerSec / 6.0;
            if (rpm < pi.crankMaxRpm) {
                pi.crankMaxRpm = rpm;
                switch (ai.limit.reason) {
                    case SpeedLimit::Velocity:
                        pi.crankReason = QStringLiteral("%1 velocity @ %2°").arg(name, QString::number(ai.limit.atDeg, 'f', 0));
                        break;
                    case SpeedLimit::Accel:
                        pi.crankReason = QStringLiteral("%1 acceleration @ %2°").arg(name, QString::number(ai.limit.atDeg, 'f', 0));
                        break;
                    case SpeedLimit::Ramp:
                        pi.crankReason = QStringLiteral("%1: crank ramp too steep (max %2 rpm/s)")
                                             .arg(name, Sig(ai.limit.maxRampDegPerSec2 / 6.0));
                        break;
                    case SpeedLimit::None:
                        break;
                }
            }
        } else if (ai.limit.maxDegPerSec < pi.overrideMax) {
            pi.overrideMax = ai.limit.maxDegPerSec;
            pi.overrideReason = QStringLiteral("%1 %2 @ %3 s")
                                    .arg(name, ai.limit.reason == SpeedLimit::Velocity ? QStringLiteral("velocity")
                                                                                       : QStringLiteral("acceleration"),
                                         QString::number(ai.limit.atDeg, 'f', 3));
        }
        pi.axes.push_back(ai);
    }

    // Checks, in the order the app lists them.
    QStringList owners[kMotorConnectors];
    if (m.source == MasterSource::DrivenCrank && UsesCrank(project)) owners[m.crankConnector] << QStringLiteral("crank");
    for (const CamAxis &a : project.axes) {
        if (a.connector >= 0 && a.connector < kMotorConnectors) owners[a.connector] << Q(a.name);
    }
    // Inputs, the board's own and the CCIO-8's: one input per job, and every job on an axis.
    {
        struct Job {
            QString pin;
            CcioRole role;
            int axis;
        };
        std::vector<Job> jobs;
        for (const BoardInput &in : project.inputs) {
            if (in.role != CcioRole::None) jobs.push_back({QString::fromStdString(BoardInputName(in.pin)), in.role, in.axis});
        }
        for (const CcioInput &in : project.ccio.inputs) {
            if (in.role != CcioRole::None) jobs.push_back({QString::fromStdString(CcioPinName(in.board, in.pin)), in.role, in.axis});
        }
        static const char *const kNames[] = {"", "Home", "Limit +", "Limit \xe2\x88\x92"};
        for (size_t a = 0; a < jobs.size(); ++a) {
            const QString job = QString::fromUtf8(kNames[static_cast<int>(jobs[a].role)]);
            if (jobs[a].axis < 0) {
                pi.checks.push_back({Check::Warning, QStringLiteral("Project"), QStringLiteral("%1 is %2 of no axis").arg(jobs[a].pin, job)});
                continue;
            }
            for (size_t b = a + 1; b < jobs.size(); ++b) {
                if (jobs[b].role == jobs[a].role && jobs[b].axis == jobs[a].axis) {
                    pi.checks.push_back({Check::Error, Q(project.axes[jobs[a].axis].name),
                                         QStringLiteral("%1 is on both %2 and %3").arg(job, jobs[a].pin, jobs[b].pin)});
                }
            }
        }
    }
    // A normally open limit: a cut wire reads as not tripped, so the axis loses that limit without a word.
    for (const BoardInput &in : project.inputs) {
        if (in.normallyOpen && (in.role == CcioRole::LimitPositive || in.role == CcioRole::LimitNegative) && in.axis >= 0 &&
            in.axis < static_cast<int>(project.axes.size())) {
            pi.checks.push_back({Check::Warning, QStringLiteral("Inputs"),
                                 QStringLiteral("%1 is a normally open limit: a cut wire leaves %2 without it")
                                     .arg(QString::fromStdString(BoardInputName(in.pin)), Q(project.axes[in.axis].name))});
        }
    }
    for (const CcioInput &in : project.ccio.inputs) {
        if (in.normallyOpen && (in.role == CcioRole::LimitPositive || in.role == CcioRole::LimitNegative) && in.axis >= 0 &&
            in.axis < static_cast<int>(project.axes.size())) {
            pi.checks.push_back({Check::Warning, QStringLiteral("Project"),
                                 QStringLiteral("%1 is a normally open limit: a cut wire leaves %2 without it")
                                     .arg(QString::fromStdString(CcioPinName(in.board, in.pin)), Q(project.axes[in.axis].name))});
        }
    }
    // The board's operator inputs (Enable, Disable, ...): the project can't use their pins (the board refuses it).
    if (opPins) {
        static const char *const kOp[] = {"Enable", "Disable", "Fault reset", "Start", "Stop"};
        for (int k = 0; k < 5; ++k) {
            const int pin = opPins[k];
            if (pin < 0) continue;
            QString use;
            for (const OutputPulse &o : project.outputs) {
                if (o.output == pin) use = QStringLiteral("an output");
            }
            const CamMaster &m = project.master;
            if (UsesTimeline(project) && m.trigger == TriggerSource::DigitalInput && m.triggerInput == pin) use = QStringLiteral("the trigger");
            if (BoardInputAt(project, pin).role != CcioRole::None) use = QStringLiteral("a home or limit input");
            if (WaitOnPin(project, pin)) use = QStringLiteral("a timeline wait's input");
            if (!use.isEmpty()) {
                pi.checks.push_back({Check::Error, QStringLiteral("Project"),
                                     QStringLiteral("%1 is the board's operator %2 input, and the project uses it as %3: the board refuses it")
                                         .arg(QString::fromStdString(BoardInputName(pin)), QLatin1String(kOp[k]), use)});
            }
        }
    }
    // Timeline waits: only where every timeline axis is at rest, inside the cycle, on a pin nothing else has.
    if (!project.waits.empty()) {
        const double cycle = TimelineLength(project);
        for (size_t k = 0; k < project.waits.size(); ++k) {
            const TimelineWait &w = project.waits[k];
            const QString name = QStringLiteral("Wait %1 (%2 s, %3)").arg(k + 1).arg(w.t, 0, 'g', 6).arg(QString::fromStdString(WaitInputName(w)));
            if (!UsesTimeline(project)) {
                pi.checks.push_back({Check::Error, QStringLiteral("Waits"), QStringLiteral("%1: the project has no timeline axes").arg(name)});
                continue;
            }
            if (!(w.t > 0.0) || !(w.t < cycle)) {
                pi.checks.push_back({Check::Error, QStringLiteral("Waits"), QStringLiteral("%1 is outside the cycle (0 to %2 s)").arg(name).arg(cycle, 0, 'g', 6)});
            } else if (!TimelineAtRest(project, w.t)) {
                pi.checks.push_back({Check::Error, QStringLiteral("Waits"),
                                     QStringLiteral("%1: a timeline axis is moving then; a wait goes where they all rest").arg(name)});
            }
            if (w.source == WaitSource::Board) {
                const std::string why = BoardInputBlocked(project, w.pin, estopPin);
                if (!why.empty()) {
                    pi.checks.push_back({Check::Error, QStringLiteral("Waits"), QStringLiteral("%1: the pin is %2").arg(name,
                        why == "estop" ? QStringLiteral("the estop's") : why == "output" ? QStringLiteral("an output")
                        : why == "trigger" ? QStringLiteral("the timeline's trigger") : QStringLiteral("the Encoder Adapter's"))});
                } else if (BoardInputAt(project, w.pin).role != CcioRole::None) {
                    pi.checks.push_back({Check::Error, QStringLiteral("Waits"), QStringLiteral("%1: the pin is a home or limit input").arg(name)});
                }
            } else if (w.source == WaitSource::Ccio) {
                if (w.board >= project.ccio.boards) {
                    pi.checks.push_back({Check::Error, QStringLiteral("Waits"), QStringLiteral("%1: on a CCIO-8 the project doesn't have").arg(name)});
                } else if (CcioInputAt(project.ccio, w.board, w.pin).role != CcioRole::None) {
                    pi.checks.push_back({Check::Error, QStringLiteral("Waits"), QStringLiteral("%1: the pin is a home or limit input").arg(name)});
                }
            }
            if (k > 0 && project.waits[k - 1].t == w.t) {
                pi.checks.push_back({Check::Warning, QStringLiteral("Waits"), QStringLiteral("%1 is at the same time as wait %2").arg(name).arg(k)});
            }
        }
    }
    // Homing: required with no switch can never be met; a search with no end and no limit on that side runs into the
    // hard end if the switch fails.
    for (size_t i = 0; i < project.axes.size(); ++i) {
        const CamAxis &a = project.axes[i];
        const AxisHome &h = a.home;
        const int ax = static_cast<int>(i);
        if (!HasHomeSwitch(project, ax)) {
            if (h.required) {
                pi.checks.push_back({Check::Error, Q(a.name), QStringLiteral("homing is required, but the axis has no home switch")});
            }
            continue;
        }
        const CcioRole searchLimit = h.dir > 0 ? CcioRole::LimitPositive : CcioRole::LimitNegative;
        bool limit = false;
        for (const BoardInput &in : project.inputs) limit = limit || (in.role == searchLimit && in.axis == ax);
        for (const CcioInput &in : project.ccio.inputs) limit = limit || (in.role == searchLimit && in.axis == ax);
        if (h.search <= 0.0 && !limit) {
            pi.checks.push_back({Check::Warning, Q(a.name),
                                 QStringLiteral("homing searches without end and there is no %1 limit: if the home switch fails "
                                                "it runs into the end of travel (set a search length)")
                                     .arg(h.dir > 0 ? QStringLiteral("+") : QStringLiteral("\u2212"))});
        }
    }
    // The board's estop input, used by the project.
    if (estopPin >= 0) {
        const QString pin = QString::fromStdString(BoardInputName(estopPin));
        for (const OutputPulse &o : project.outputs) {
            if (o.output == estopPin) {
                pi.checks.push_back({Check::Error, QStringLiteral("Project"), QStringLiteral("%1 is the board's estop input: it can't be an output").arg(pin)});
                break;
            }
        }
        if (m.trigger == TriggerSource::DigitalInput && UsesTimeline(project) && m.triggerInput == estopPin) {
            pi.checks.push_back({Check::Error, QStringLiteral("Master"), QStringLiteral("Trigger input %1 is the board's estop input").arg(pin)});
        }
        if (m.source == MasterSource::Encoder && UsesCrank(project) && estopPin >= kEncoderInputFirst && estopPin <= kEncoderInputLast) {
            pi.checks.push_back({Check::Error, QStringLiteral("Master"),
                                 QStringLiteral("The board's estop is on %1, which the Encoder Adapter needs").arg(pin)});
        }
    }
    // The board's own inputs on a pin something else has.
    for (const BoardInput &in : project.inputs) {
        const std::string blocked = BoardInputBlocked(project, in.pin, estopPin);
        if (blocked.empty()) continue;
        const QString pin = QString::fromStdString(BoardInputName(in.pin));
        pi.checks.push_back({Check::Error, QStringLiteral("Inputs"),
                             blocked == "estop"     ? QStringLiteral("%1 is set up as an input and is also the board's estop").arg(pin)
                             : blocked == "output"  ? QStringLiteral("%1 is set up as an input and is also an output").arg(pin)
                             : blocked == "trigger" ? QStringLiteral("%1 is set up as an input and is also the timeline's trigger").arg(pin)
                                                    : QStringLiteral("%1 is set up as an input, but the encoder master's Encoder Adapter uses it").arg(pin)});
    }
    for (int i = 0; i < kMotorConnectors; ++i) {
        if (owners[i].size() > 1) {
            pi.checks.push_back({Check::Error, QStringLiteral("Project"),
                                 QStringLiteral("M-%1 is assigned to %2").arg(i).arg(owners[i].join(QStringLiteral(" and ")))});
        }
    }
    if (UsesCrank(project) && m.rpm > pi.crankMaxRpm) {
        pi.checks.push_back({Check::Error, QStringLiteral("Master"),
                             QStringLiteral("%1 %2 rpm is above the max %3 rpm (%4)")
                                 .arg(m.source == MasterSource::DrivenCrank ? QStringLiteral("Run speed")
                                                                            : QStringLiteral("Expected speed"),
                                      QString::number(m.rpm, 'f', 1), QString::number(pi.crankMaxRpm, 'f', 1), pi.crankReason)});
    }
    if (UsesTimeline(project) && m.speedOverride > pi.overrideMax) {
        pi.checks.push_back({Check::Error, QStringLiteral("Master"),
                             QStringLiteral("Timeline speed %1 % is above the max %2 % (%3)")
                                 .arg(QString::number(m.speedOverride * 100.0, 'f', 1),
                                      QString::number(pi.overrideMax * 100.0, 'f', 1), pi.overrideReason)});
    }
    if (m.trigger == TriggerSource::DigitalInput && UsesTimeline(project) && m.triggerInput < kOutputs) {
        for (const OutputPulse &o : project.outputs) {
            if (o.output == m.triggerInput) {
                pi.checks.push_back({Check::Error, QStringLiteral("Master"),
                                     QStringLiteral("Trigger input %1 is also an output").arg(InputName(m.triggerInput))});
                break;
            }
        }
    }
    if (TriggerOnEncoderInput(m, UsesCrank(project)) && m.trigger == TriggerSource::DigitalInput && UsesTimeline(project)) {
        pi.checks.push_back({Check::Error, QStringLiteral("Master"),
                             QStringLiteral("Trigger input %1 is used by the Encoder Adapter").arg(InputName(m.triggerInput))});
    }
    for (size_t ai = 0; ai < project.axes.size(); ++ai) {
        const CamAxis &a = project.axes[ai];
        const AxisInfo &info = pi.axes[ai];
        const QString name = Q(a.name), u = Q(a.units);
        const double w = info.w;
        const int dx = OnTimeline(a) ? 3 : 1;
        auto at = [&](double x) { return QString::number(x, 'f', dx) + (OnTimeline(a) ? QStringLiteral(" s") : XUnit(a)); };
        if (a.velMax > 0.0 && info.stats.maxVel * w > a.velMax * (1 + 1e-9)) {
            pi.checks.push_back({Check::Error, name,
                                 QStringLiteral("Peak velocity %1 > limit %2 %3/s").arg(Sig(info.stats.maxVel * w), Sig(a.velMax), u)});
        }
        if (a.accMax > 0.0 && info.worstAcc > a.accMax * (1 + 1e-9)) {
            pi.checks.push_back({Check::Error, name,
                                 QStringLiteral("Peak acceleration %1 > limit %2 %3/s²%4")
                                     .arg(Sig(info.worstAcc), Sig(a.accMax), u,
                                          AxisRamp(m, a) > 0 ? QStringLiteral(" (with crank ramp)") : QString())});
        }
        for (const CamIssue &is : info.stats.issues) {
            switch (is.kind) {
                case IssueKind::VelocityStep:
                    pi.checks.push_back({Check::Error, name,
                                         QStringLiteral("Velocity step at %1 (%2 → %3 %4/s): infinite acceleration")
                                             .arg(at(is.deg), Sig(is.left * w), Sig(is.right * w), u)});
                    break;
                case IssueKind::AccelStep:
                    pi.checks.push_back({Check::Warning, name,
                                         QStringLiteral("Acceleration step at %1 (%2 → %3 %4/s²): infinite jerk")
                                             .arg(at(is.deg), Sig(is.left * w * w), Sig(is.right * w * w), u)});
                    break;
                case IssueKind::PressureAngle:
                    pi.checks.push_back({Check::Warning, name,
                                         QStringLiteral("Pressure angle %1° > %2° (cam drawing)")
                                             .arg(QString::number(is.left, 'f', 1), QString::number(kMaxPressureAngleDeg, 'f', 0))});
                    break;
                case IssueKind::Undercut:
                    pi.checks.push_back({Check::Warning, name,
                                         QStringLiteral("Undercut near %1°: pitch radius %2 < roller (cam drawing)")
                                             .arg(QString::number(is.deg, 'f', 1), Sig(is.left))});
                    break;
                case IssueKind::SplineOvershoot:
                    pi.checks.push_back({Check::Warning, name,
                                         QStringLiteral("Spline overshoot at %1: reaches %2, past the knots' %3 %4")
                                             .arg(at(is.deg), Sig(is.left), Sig(is.right), u)});
                    break;
            }
        }
    }
    return pi;
}

// ---- Rows ---------------------------------------------------------------------------------------

QString FormatValue(double v, Fmt f) {
    if (!std::isfinite(v)) return QStringLiteral("—");
    QLocale c = QLocale::c();
    c.setNumberOptions(QLocale::DefaultNumberOptions);  // thousands separators
    switch (f) {
        case Fmt::Int: return c.toString(v, 'f', 0);
        case Fmt::One: return c.toString(v, 'f', 1);
        case Fmt::Three: return c.toString(v, 'f', 3);
        case Fmt::Deg: return c.toString(v, 'f', 2);
        case Fmt::Sec: return c.toString(v, 'f', 4);
        case Fmt::Pos: return c.toString(v, 'f', 4);
        case Fmt::Rate: return c.toString(v, 'f', 3);
        case Fmt::Pct: return c.toString(v * 100.0, 'f', 1) + QStringLiteral(" %");
    }
    return {};
}

QString Row::shown() const { return isText() ? (text.isEmpty() ? QStringLiteral("not set") : text) : FormatValue(value, fmt); }

Row TextRow(const QString &k, const QString &v, const QString &note) {
    Row r;
    r.key = k;
    r.text = v;
    r.note = note;
    return r;
}

Row NumRow(const QString &k, double v, Fmt fmt, const QString &unit, const QString &note) {
    Row r;
    r.key = k;
    r.unit = unit;
    r.note = note;
    r.fmt = fmt;
    if (std::isfinite(v)) {
        r.value = v;
    } else {
        r.text = QStringLiteral("not set");
    }
    return r;
}

std::vector<Row> MasterRows(const CamProject &project, const ProjectInfo &pi) {
    const CamMaster &m = project.master;
    std::vector<Row> rows;
    const bool crank = UsesCrank(project), timeline = UsesTimeline(project);
    rows.push_back(TextRow(QStringLiteral("Uses"), crank && timeline ? QStringLiteral("Crank and timeline")
                                                 : crank             ? QStringLiteral("Crank")
                                                 : timeline          ? QStringLiteral("Timeline")
                                                                     : QStringLiteral("Nothing yet")));
    if (crank) {
        if (m.source == MasterSource::Encoder) {
            rows.push_back(TextRow(QStringLiteral("Crank source"), QStringLiteral("External encoder"),
                                   QStringLiteral("Encoder Adapter on DI-6..DI-8; followers track the shaft")));
            rows.push_back(NumRow(QStringLiteral("Encoder counts per rev"), m.crankPpr, Fmt::Int, QStringLiteral("counts"),
                                  QStringLiteral("quadrature, x4")));
            rows.push_back(TextRow(QStringLiteral("Encoder direction"), m.encoderReverse ? QStringLiteral("Reversed") : QStringLiteral("Normal")));
            if (m.lineMode) {
                rows.push_back(TextRow(QStringLiteral("Cut-length mode"), QStringLiteral("On"),
                                       QStringLiteral("one crank revolution is one cut of the design length")));
                rows.push_back(NumRow(QStringLiteral("Wheel circumference"), m.wheelCircumference, Fmt::Three, QStringLiteral("mm")));
                rows.push_back(NumRow(QStringLiteral("Line counts per mm"), m.lineCountsPerMm, Fmt::Three, QStringLiteral("counts/mm")));
                rows.push_back(NumRow(QStringLiteral("Design length"), m.designLength, Fmt::Three, QStringLiteral("mm"),
                                      QStringLiteral("line distance from cut to cut, as drawn")));
                rows.push_back(NumRow(QStringLiteral("Kerf"), m.kerf, Fmt::Three, QStringLiteral("mm")));
                rows.push_back(NumRow(QStringLiteral("Line wait (stretches)"), LineWaitDeg(project), Fmt::Deg, QStringLiteral("°"),
                                      QStringLiteral("opening dwell common to every crank axis")));
                rows.push_back(NumRow(QStringLiteral("Shortest piece"), LineMinLength(project), Fmt::Three, QStringLiteral("mm")));
                rows.push_back(NumRow(QStringLiteral("Cuts per minute"), m.rpm, Fmt::One, QStringLiteral("cuts/min"),
                                      QStringLiteral("at the expected line speed")));
                rows.push_back(NumRow(QStringLiteral("Line speed"), m.rpm * m.designLength / 1000.0, Fmt::Three, QStringLiteral("m/min")));
            } else {
                rows.push_back(TextRow(QStringLiteral("Gear ratio (encoder : crank)"),
                                       QStringLiteral("%1 : %2").arg(m.crankGearNum).arg(m.crankGearDen)));
                rows.push_back(NumRow(QStringLiteral("Counts per crank rev"), m.countsPerRev, Fmt::Three, QStringLiteral("counts")));
                rows.push_back(NumRow(QStringLiteral("Expected speed"), m.rpm, Fmt::One, QStringLiteral("rpm"),
                                      QStringLiteral("for analysis and preview")));
            }
        } else {
            rows.push_back(TextRow(QStringLiteral("Crank source"), QStringLiteral("Driven crank"),
                                   QStringLiteral("the firmware drives the crank motor; followers track its commanded position")));
            rows.push_back(TextRow(QStringLiteral("Crank motor"), QStringLiteral("M-%1").arg(m.crankConnector)));
            rows.push_back(NumRow(QStringLiteral("Motor steps per rev"), m.crankPpr, Fmt::Int, QStringLiteral("steps")));
            if (m.lineMode) {
                rows.push_back(NumRow(QStringLiteral("Line per motor rev"), m.wheelCircumference, Fmt::Three, QStringLiteral("mm")));
                rows.push_back(NumRow(QStringLiteral("Design length"), m.designLength, Fmt::Three, QStringLiteral("mm")));
                rows.push_back(NumRow(QStringLiteral("Kerf"), m.kerf, Fmt::Three, QStringLiteral("mm")));
                rows.push_back(NumRow(QStringLiteral("Line speed"), m.rpm * m.designLength / 1000.0, Fmt::Three, QStringLiteral("m/min")));
            } else {
                rows.push_back(TextRow(QStringLiteral("Gear ratio (motor : crank)"),
                                       QStringLiteral("%1 : %2").arg(m.crankGearNum).arg(m.crankGearDen)));
            }
            rows.push_back(NumRow(QStringLiteral("Counts per crank rev"), m.countsPerRev, Fmt::Three, QStringLiteral("steps")));
            rows.push_back(NumRow(QStringLiteral("Run speed"), m.rpm, Fmt::One, m.lineMode ? QStringLiteral("cuts/min") : QStringLiteral("rpm")));
            rows.push_back(NumRow(QStringLiteral("Crank motor speed"), m.rpm * m.countsPerRev / m.crankPpr, Fmt::One,
                                  QStringLiteral("rpm")));
            rows.push_back(NumRow(QStringLiteral("Ramp"), m.crankRampRpmPerSec, Fmt::One, QStringLiteral("rpm/s")));
            rows.push_back(NumRow(QStringLiteral("Crank max"), m.crankMaxRpm, Fmt::One, QStringLiteral("rpm")));
        }
        rows.push_back(NumRow(QStringLiteral("Max crank speed allowed"), pi.crankMaxRpm, Fmt::One, QStringLiteral("rpm"),
                              std::isfinite(pi.crankMaxRpm) ? pi.crankReason : QStringLiteral("no axis limits set")));
    }
    if (timeline) {
        rows.push_back(NumRow(QStringLiteral("Timeline length"), TimelineLength(project), Fmt::Sec, QStringLiteral("s"),
                              QStringLiteral("longest timeline axis or output pulse, as drawn")));
        rows.push_back(TextRow(QStringLiteral("Runs"), m.timelineRepeat ? QStringLiteral("Repeating") : QStringLiteral("Once per trigger")));
        rows.push_back(NumRow(QStringLiteral("Speed override"), m.speedOverride, Fmt::Pct));
        rows.push_back(TextRow(QStringLiteral("Trigger"), TriggerText(m)));
        rows.push_back(NumRow(QStringLiteral("Max override allowed"), pi.overrideMax, Fmt::Pct, {},
                              std::isfinite(pi.overrideMax) ? pi.overrideReason : QStringLiteral("no axis limits set")));
    }
    return rows;
}

std::vector<Row> AxisSettingsRows(const CamAxis &a) {
    const QString u = Q(a.units);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::vector<Row> rows;
    rows.push_back(TextRow(QStringLiteral("Master"), OnTimeline(a) ? QStringLiteral("Timeline") : QStringLiteral("Crank")));
    rows.push_back(TextRow(QStringLiteral("Motor"), QStringLiteral("M-%1").arg(a.connector)));
    rows.push_back(TextRow(QStringLiteral("Units"), u));
    rows.push_back(TextRow(QStringLiteral("Drive"), DriveText(a.drive)));
    if (a.drive != AxisDrive::Direct) rows.push_back(NumRow(QStringLiteral("Motor steps per rev"), a.motorPpr, Fmt::Int, QStringLiteral("steps")));
    if (a.drive == AxisDrive::Ballscrew) rows.push_back(NumRow(QStringLiteral("Screw pitch"), a.screwPitch, Fmt::Three, u + QStringLiteral("/rev")));
    if (a.drive == AxisDrive::Belt) rows.push_back(NumRow(QStringLiteral("Pulley pitch diameter"), a.pulleyPd, Fmt::Three, u));
    rows.push_back(NumRow(QStringLiteral("Counts per unit"), a.countsPerUnit, Fmt::Three, QStringLiteral("counts/") + u));
    rows.push_back(NumRow(QStringLiteral("Velocity limit"), a.velMax > 0 ? a.velMax : nan, Fmt::Three, u + QStringLiteral("/s")));
    rows.push_back(NumRow(QStringLiteral("Acceleration limit"), a.accMax > 0 ? a.accMax : nan, Fmt::Three, u + QStringLiteral("/s²")));
    rows.push_back(NumRow(QStringLiteral("Start position"), a.startPos, Fmt::Pos, u));
    rows.push_back(TextRow(QStringLiteral("Profile"), a.closed ? QStringLiteral("Closed (ends where it starts)") : QStringLiteral("Open")));
    if (OnTimeline(a)) {
        rows.push_back(NumRow(QStringLiteral("Duration"), a.span, Fmt::Sec, QStringLiteral("s")));
        rows.push_back(TextRow(QStringLiteral("Repeats with the cycle"), a.cyclic ? QStringLiteral("Yes") : QStringLiteral("No (rests after its end)")));
    } else {
        rows.push_back(NumRow(QStringLiteral("Cam base radius"), a.baseRadius, Fmt::Three, u));
        rows.push_back(NumRow(QStringLiteral("Roller radius"), a.rollerRadius, Fmt::Three, u));
    }
    return rows;
}

std::vector<StatRow> AxisStatRows(const CamProject &project, int axis, const AxisInfo &ai) {
    const CamAxis &a = project.axes[axis];
    const AxisStats &st = ai.stats;
    const QString u = Q(a.units), xu = XUnit(a);
    const double w = ai.w;
    auto at = [&](double x) { return OnTimeline(a) ? QString::number(x, 'f', 3) + QStringLiteral(" s") : QString::number(x, 'f', 1) + xu; };
    std::vector<StatRow> stats = {
        {QStringLiteral("Stroke"), st.maxPos - st.minPos, Fmt::Pos, u, {}},
        {QStringLiteral("Lowest position"), st.minPos, Fmt::Pos, u, {}},
        {QStringLiteral("Highest position"), st.maxPos, Fmt::Pos, u, {}},
        {QStringLiteral("Peak velocity"), st.maxVel * w, Fmt::Rate, u + QStringLiteral("/s"), at(st.maxVelDeg)},
        {QStringLiteral("Peak acceleration"), st.maxAcc * w * w, Fmt::Rate, u + QStringLiteral("/s²"), at(st.maxAccDeg)},
        {QStringLiteral("Peak jerk"), st.maxJerk * w * w * w, Fmt::Rate, u + QStringLiteral("/s³"), at(st.maxJerkDeg)},
        {QStringLiteral("Motor peak speed"), ai.motorPeakRpm, Fmt::One, QStringLiteral("rpm"), at(st.maxVelDeg)},
        {QStringLiteral("Peak step rate"), ai.stepRateHz / 1000.0, Fmt::Three, QStringLiteral("kHz"), {}},
        {a.closed ? QStringLiteral("Net move per cycle") : QStringLiteral("Advance per cycle"), st.advancePerRev, Fmt::Pos, u, {}},
    };
    if (a.gear) {
        stats.insert(stats.begin(), {QStringLiteral("Electronic gear"), static_cast<double>(a.gearNum) / a.gearDen, Fmt::Three,
                                     u + QStringLiteral(" per crank rev"),
                                     QStringLiteral("%1 : %2").arg(a.gearNum).arg(a.gearDen)});
    }
    if (OnTimeline(a)) {
        stats.push_back({QStringLiteral("Max speed override"), ai.limit.maxDegPerSec, Fmt::Pct, {}, {}});
    } else {
        stats.push_back({QStringLiteral("Max crank speed"), ai.limit.maxDegPerSec / 6.0, Fmt::One, QStringLiteral("rpm"),
                         std::isfinite(ai.limit.maxDegPerSec) ? at(ai.limit.atDeg) : QString()});
        stats.push_back({QStringLiteral("Peak motor / master counts"),
                         st.maxVel * a.countsPerUnit * 360.0 / std::max(project.master.countsPerRev, 1e-9), Fmt::Three, {}, {}});
        stats.push_back({QStringLiteral("Max pressure angle"), st.maxPressureDeg, Fmt::One, QStringLiteral("°"), at(st.maxPressureAt)});
        stats.push_back({QStringLiteral("Min pitch curvature radius"), st.minCurvature, Fmt::Three, u, at(st.minCurvatureAt)});
    }
    return stats;
}

}  // namespace review
