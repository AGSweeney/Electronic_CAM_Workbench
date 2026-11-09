/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "CamRuntime.h"

#include <math.h>

#include <algorithm>

namespace ecam {

namespace {

int32_t RoundCounts(double v) {
    if (v > 2147483000.0) v = 2147483000.0;
    if (v < -2147483000.0) v = -2147483000.0;
    return static_cast<int32_t>(lround(v));
}

// Table value at a position given as an index and a 16-bit fraction of the next interval.
int64_t Interp(const int32_t *t, int64_t idx, int64_t fracQ16) {
    if (idx >= kRtIntervals) return t[kRtIntervals];
    const int64_t a = t[idx], b = t[idx + 1];
    return a + (((b - a) * fracQ16) >> 16);
}

int64_t Step(int64_t v, int64_t target, int64_t step) {
    if (v < target) return v + step < target ? v + step : target;
    if (v > target) return v - step > target ? v - step : target;
    return v;
}

}  // namespace

const char *CrankStateName(CrankState s) {
    switch (s) {
        case CrankState::Off: return "off";
        case CrankState::Following: return "following";
        case CrankState::Running: return "running";
        case CrankState::Stopping: return "stopping";
        case CrankState::Jogging: return "jogging";
    }
    return "?";
}

const char *TimelineStateName(TimelineState s) {
    switch (s) {
        case TimelineState::Idle: return "idle";
        case TimelineState::Armed: return "armed";
        case TimelineState::Running: return "running";
        case TimelineState::Stopping: return "stopping";
        case TimelineState::Done: return "done";
        case TimelineState::Jogging: return "jogging";
        case TimelineState::Waiting: return "waiting";
    }
    return "?";
}

bool CamRuntime::Idle() const {
    return m_crank == CrankState::Off && m_tl != TimelineState::Running && m_tl != TimelineState::Stopping &&
           m_tl != TimelineState::Jogging && m_tl != TimelineState::Waiting;
}

void CamRuntime::Unload() {
    m_loaded = false;
    m_axisCount = 0;
    m_crank = CrankState::Off;
    m_tl = TimelineState::Idle;
    m_outputs = 0;
    m_outputsUsed = 0;
    m_pulseCount = 0;
}

const char *CamRuntime::Build(const CamProject &p) {
    if (!Idle()) return "stop the motion first";
    if (p.axes.empty() || static_cast<int>(p.axes.size()) > kRtAxes) return "1 to 4 axes";
    Unload();
    const CamMaster &m = p.master;
    if (m.countsPerRev < 1 || m.countsPerRev > 2e9) return "master counts per rev out of range";
    // Master units: 1/den of a count, so a gear ratio that is not whole stays exact. One crank
    // revolution is ppr * gear numerator units.
    const double units = CrankUnitsPerRev(m);
    if (units < 1.0 || units > kMaxCrankUnits) return "master units per rev out of range (ppr x gear numerator)";
    m_cpr = static_cast<int32_t>(llround(units));
    m_den = m.crankGearDen < 1 ? 1 : m.crankGearDen;
    m_cprQ24 = static_cast<int64_t>(m_cpr) << 24;
    m_line = m.lineMode;
    m_cpm = m.lineCountsPerMm;
    m_kerf = m.kerf;
    m_design = m_cpr;
    m_wait = m_line ? static_cast<int32_t>(llround(LineWaitDeg(p) / 360.0 * m_cpr)) : 0;
    m_pendingUnits = m_cpr;
    m_syncAxis = -1;
    {
        int ax = -1;
        double s0 = 0.0, s1 = 0.0;
        if (m_line && LineSync(p, &ax, &s0, &s1) && p.axes[ax].countsPerUnit != 0.0) {
            m_syncAxis = ax;
            m_syncStart = static_cast<int32_t>(llround(s0 / 360.0 * m_design));
            m_syncEnd = static_cast<int32_t>(llround(s1 / 360.0 * m_design));
            m_lineCountsPerCarriageCount = m_cpm / p.axes[ax].countsPerUnit;  // the carriage is in mm of line
        }
    }
    if (m_line && ecam::UsesCrank(p) && m_wait <= 0) return "cut-length mode: every crank axis must start with a dwell";
    m_driven = m.source == MasterSource::DrivenCrank;
    m_hasCrank = ecam::UsesCrank(p);
    m_hasTimeline = ecam::UsesTimeline(p);
    m_crankMotor = (m_driven && m_hasCrank) ? m.crankConnector : -1;
    m_crankMotorSign = m.crankReverse ? -1 : 1;
    m_repeat = m.timelineRepeat;
    m_triggerRising = m.triggerRising;
    m_inputTrigger = m.trigger == TriggerSource::DigitalInput;

    // Crank: target velocity and ramp in master counts per sample (<< 24).
    const double countsPerSamplePerRpm = static_cast<double>(m_cpr) / 60.0 / kRtSampleHz;  // master units
    m_velTargetQ24 = static_cast<int64_t>(llround(m.rpm * countsPerSamplePerRpm * 16777216.0));
    m_rampQ24 = static_cast<int64_t>(llround(m.crankRampRpmPerSec * countsPerSamplePerRpm / kRtSampleHz * 16777216.0));
    if (m_rampQ24 < 1) m_rampQ24 = 1;
    m_limitSamples = static_cast<int64_t>(llround(m.limitStopSeconds * kRtSampleHz));
    if (m_limitSamples < 1) m_limitSamples = 1;
    m_crankStopStepQ24 = 0;
    m_limitStopping = false;
    m_limitAxis = -1;
    m_limitSide = 0;
    m_homed = 0;
    m_homeAll = 0;
    m_homeAxis = -1;
    m_homeFail = HomeFail::None;
    // Timeline waits, in time order (NormalizeProject sorts them).
    m_waitCount = 0;
    for (const TimelineWait &w : p.waits) {
        if (m_waitCount >= kMaxWaits) break;
        Wait &x = m_waits[m_waitCount++];
        x.tQ16 = static_cast<int64_t>(llround(w.t * kRtSampleHz * 65536.0));
        x.timeout = w.timeout > 0.0 ? static_cast<uint32_t>(llround(w.timeout * kRtSampleHz)) : 0;
        x.holds = x.timeouts = 0;
        x.edge = w.edge;
        x.armed = false;
    }
    m_waitNext = 0;
    m_waitCur = -1;
    m_waited = 0;
    m_waitEvents = 0;
    m_waitEventIndex = -1;
    m_waitSkipSeen = m_waitSkipReq;

    // Timeline: speed and the rate it changes (full speed in 0.25 s).
    m_rateTargetQ16 = static_cast<int32_t>(lround(m.speedOverride * 65536.0));
    m_rateStepQ16 = static_cast<int32_t>(65536 / (kRtSampleHz / 4));
    if (m_rateStepQ16 < 1) m_rateStepQ16 = 1;

    double cycle = 0.0;
    for (size_t i = 0; i < p.axes.size(); ++i) {
        const CamAxis &a = p.axes[i];
        Axis &r = m_axes[i];
        r.timeline = a.master == AxisMaster::Timeline;
        r.connector = a.connector;
        r.motorSign = a.reverse ? -1 : 1;
        r.engaged = false;
        r.lastTarget = 0;
        r.pos = 0;
        r.gear = a.gear;
        r.fixed = a.closed && !a.gear;
        r.man.Halt();
        {
            // Homing, in counts: positions with the axis's sign (a negative counts per unit counts the other way).
            const double cpu = a.countsPerUnit;
            const double mag = fabs(cpu) > 0.0 ? fabs(cpu) : 1.0;
            const ecam::AxisHome &h = a.home;
            Axis::HomeCfg &c = r.hc;
            c.has = HasHomeSwitch(p, static_cast<int>(i));
            c.dir = (h.dir > 0) == (cpu >= 0.0) ? 1 : -1;
            c.fast = std::max(h.fast * mag, 1.0);
            c.slow = std::max(std::min(h.slow * mag, c.fast), 1.0);
            // As axis jog: the axis's acceleration limit, or the fast speed in a quarter second.
            c.accel = std::max(a.accMax > 0.0 ? a.accMax * mag : c.fast * 4.0, 100.0);
            c.decel = h.decel > 0.0 ? std::max(h.decel * mag, 100.0) : c.accel;
            c.back = static_cast<int64_t>(llround(h.back * mag));
            c.at = static_cast<int64_t>(llround(h.at * cpu));
            c.search = static_cast<int64_t>(llround(h.search * mag));
            c.thenStart = h.thenStart;
            c.order = h.order;
            r.hphase = 0;
            r.hresult = HomeFail::None;
            r.hlatch = 0;
            r.hends = 0;
            r.trips[0] = r.trips[1] = 0;
            r.tripAt[0] = r.tripAt[1] = 0;
            r.limitStop = false;
        }
        if (a.gear) {
            // Motor counts per master unit, countsPerUnit * num / (den * units per revolution), as whole
            // numbers. A countsPerUnit that is not whole is scaled by powers of two until it is, or the
            // numerator reaches 2^38: then its error is under 1 part in 2^38 of the distance.
            const double limit = 274877906944.0;  // 2^38; leaves room for 2^24 units in one sample
            double c = a.countsPerUnit * a.gearNum, q = static_cast<double>(a.gearDen) * m_design;
            if (fabs(c) >= limit || q >= 4.0e18) return "gear ratio out of range";
            while (c != floor(c) && fabs(c) * 2.0 < limit && q * 2.0 < 4.0e18) {
                c *= 2.0;
                q *= 2.0;
            }
            r.gearP = llround(c);
            r.gearQ = llround(q);
            r.gearAcc = 0;
            if (r.gearP == 0) return "gear ratio too small";
            r.advance = 0;
            r.spanQ16 = 1;
            continue;
        }
        const int last = static_cast<int>(a.segments.size()) - 1;
        const SplineFit fit = FitSplines(a);
        for (int k = 0; k <= kRtIntervals; ++k) {
            if (m_yield && (k & 31) == 0) m_yield();
            const CamPoint pt = (k == kRtIntervals) ? EvaluateSegment(a, last, 1.0, &fit)
                                                    : Evaluate(a, a.span * k / kRtIntervals, &fit);
            r.table[k] = RoundCounts(pt.pos * a.countsPerUnit);
        }
        r.advance = static_cast<int64_t>(r.table[kRtIntervals]) - r.table[0];
        r.spanQ16 = static_cast<int64_t>(llround(a.span * kRtSampleHz * 65536.0));
        if (r.spanQ16 < 1) r.spanQ16 = 1;
        if (r.timeline && a.span > cycle) cycle = a.span;
    }
    if (static_cast<int>(p.outputs.size()) > kMaxOutputPulses) return "too many output pulses";
    m_pulseCount = 0;
    m_outputsUsed = 0;
    for (const OutputPulse &o : p.outputs) {
        if (o.output < 0 || o.output >= kOutputs || o.off <= o.on) return "bad output pulse";
        Pulse &q = m_pulses[m_pulseCount++];
        q.crank = o.master == AxisMaster::Crank;
        if (q.crank) {
            q.on = static_cast<int64_t>(llround(o.on / 360.0 * m_design));
            q.off = static_cast<int64_t>(llround(o.off / 360.0 * m_design));
        } else {
            q.on = static_cast<int64_t>(llround(o.on * kRtSampleHz * 65536.0));
            q.off = static_cast<int64_t>(llround(o.off * kRtSampleHz * 65536.0));
            if (o.off > cycle) cycle = o.off;
        }
        q.mask = static_cast<uint8_t>(1u << o.output);
        m_outputsUsed |= q.mask;
    }
    m_outputs = 0;
    m_cycleQ16 = cycle > 0.0 ? static_cast<int64_t>(llround(cycle * kRtSampleHz * 65536.0)) : 1;
    m_axisCount = static_cast<int>(p.axes.size());
    if (m_syncAxis >= 0) {
        // Carriage motor counts at the sync start, from its table.
        const int64_t num = static_cast<int64_t>(m_syncStart) * kRtIntervals;
        m_carriageAtSync = Interp(m_axes[m_syncAxis].table, num / m_design, ((num % m_design) << 16) / m_design);
    }

    m_phaseQ24 = 0;
    m_rev = 0;
    m_velQ24 = 0;
    m_timeQ16 = 0;
    m_cycle = 0;
    m_rateQ16 = 0;
    m_crankMotorLast = 0;
    m_crankBase = 0;
    m_crankMotorPos = 0;
    m_movedQ24 = 0;
    m_triggerKnown = false;
    // Requests made before this load do not carry over.
    m_startSeen = m_startReq;
    m_stopSeen = m_stopReq;
    m_abortSeen = m_abortReq;
    m_triggerSeen = m_triggerReq;
    m_cycleStopSeen = m_cycleStopReq;
    m_endOfCycle = false;
    m_jogOnSeen = m_jogOnReq;
    m_jogOffSeen = m_jogOffReq;
    m_jogEnding = false;
    m_loaded = true;
    return nullptr;
}

void CamRuntime::SetOverride(double ovr) {
    if (ovr < 0.0) ovr = 0.0;
    if (ovr > 10.0) ovr = 10.0;
    m_rateTargetQ16 = static_cast<int32_t>(lround(ovr * 65536.0));
}

void CamRuntime::SetRpm(double rpm) {
    if (rpm < 0.0) rpm = 0.0;
    // The cycle as drawn: a driven line keeps its speed whatever the cut length.
    m_velTargetQ24 = static_cast<int64_t>(llround(rpm * m_design / 60.0 / kRtSampleHz * 16777216.0));
}

void CamRuntime::SetStopSeconds(double s) { m_stopSeconds = s < 0.01 ? 0.01 : s; }

int64_t CamRuntime::DesignPhase() const {
    const int64_t phase = m_phaseQ24 >> 24;  // 0 .. cpr-1
    if (!m_line) return phase;
    // The cycle as drawn starts with the wait; a longer cut starts it earlier (more wait), a shorter
    // one later (less wait). Everything after the wait keeps its line distance.
    const int64_t x = phase - (static_cast<int64_t>(m_cpr) - m_design);
    return x < 0 ? 0 : x;
}

int64_t CamRuntime::CrankTarget(const Axis &a) const {
    const int64_t x = DesignPhase();  // 0 .. design-1
    const int64_t num = x * kRtIntervals;
    int64_t idx = num / m_design;  // 0 .. kRtIntervals - 1
    // A driven crank moves in parts of a count: take them, or a coarse crank gives the followers bursts of
    // steps with nothing between. An encoder counts whole, so this adds nothing there. In the wait, where
    // DesignPhase holds at 0, there is no part to take.
    const int64_t part = x > 0 || !m_line ? m_phaseQ24 & 0xFFFFFF : 0;
    const int64_t posQ16 = (((num % m_design) << 16) + ((part * kRtIntervals) >> 8)) / m_design;
    idx += posQ16 >> 16;  // a coarse crank's count spans several table intervals
    return Interp(a.table, idx, posQ16 & 0xFFFF) + static_cast<int64_t>(m_rev) * a.advance;
}

void CamRuntime::CutGap(double *gap, bool *inSync, double *syncLeft) const {
    *gap = 0.0;
    *inSync = false;
    *syncLeft = 0.0;
    if (!m_line || m_syncAxis < 0) return;
    const int64_t phase = m_phaseQ24 >> 24;
    const int64_t offset = static_cast<int64_t>(m_cpr) - m_design;  // the stretch of this cycle's wait
    const int64_t syncStart = offset + m_syncStart, syncEnd = offset + m_syncEnd;
    // The carriage now, from its table at the drawn position (an open profile's advance aside).
    const int64_t x = DesignPhase();
    const int64_t num = x * kRtIntervals;
    const int64_t carriage = Interp(m_axes[m_syncAxis].table, num / m_design, ((num % m_design) << 16) / m_design);
    // The next sync start: this cycle's until its sync is over, then the next cycle's at its length.
    int64_t target;
    if (phase < syncEnd) {
        target = syncStart;
        if (phase >= syncStart) {
            *inSync = true;
            *syncLeft = static_cast<double>(syncEnd - phase);
        }
    } else {
        target = static_cast<int64_t>(m_cpr) + (static_cast<int64_t>(m_pendingUnits) - m_design) + m_syncStart;
    }
    // The cut line meets the blade at the sync start: it is (line to go) less (carriage still to travel)
    // behind the blade. 0 throughout the sync.
    *gap = static_cast<double>(target - phase) -
           static_cast<double>(m_carriageAtSync - carriage) * m_lineCountsPerCarriageCount;
    if (*inSync) *gap = 0.0;
}

void CamRuntime::SetCycleUnits(int32_t units) {
    m_crankBase += static_cast<int64_t>(m_rev) * (static_cast<int64_t>(m_cpr) - units);  // the crank motor doesn't move
    m_cpr = units;
    m_cprQ24 = static_cast<int64_t>(units) << 24;
}

const char *CamRuntime::SetCutCounts(int64_t counts) {
    if (!m_loaded || !m_line) return "not in cut-length mode";
    if (counts < static_cast<int64_t>(m_design) - m_wait) return "shorter than the cycle's fixed part";
    if (counts > 2000000000) return "too long";
    m_pendingUnits = static_cast<int32_t>(counts);
    return nullptr;
}

int64_t CamRuntime::TimelineTarget(const Axis &a) const {
    const int64_t base = static_cast<int64_t>(m_cycle) * a.advance;
    if (m_timeQ16 >= a.spanQ16) return a.table[kRtIntervals] + base;  // rests at its end
    const int64_t t = m_timeQ16 < 0 ? 0 : m_timeQ16;
    // t * kRtIntervals stays below 2^63 for any span under ~4 years.
    const int64_t num = t * kRtIntervals;
    const int64_t idx = num / a.spanQ16;
    const int64_t rem = num - idx * a.spanQ16;
    // rem < spanQ16; scale it to a 16-bit fraction without overflowing.
    const int64_t fracQ16 = a.spanQ16 > (int64_t(1) << 46) ? rem / (a.spanQ16 >> 16) : (rem << 16) / a.spanQ16;
    return Interp(a.table, idx, fracQ16) + base;
}

void CamRuntime::EngageCrankAxes() {
    for (int i = 0; i < m_axisCount; ++i) {
        Axis &a = m_axes[i];
        if (a.timeline) continue;
        if (!a.gear) a.lastTarget = CrankTarget(a);
        a.engaged = true;
        a.man.Halt();
    }
}

void CamRuntime::EngageTimelineAxes() {
    for (int i = 0; i < m_axisCount; ++i) {
        Axis &a = m_axes[i];
        if (!a.timeline) continue;
        a.lastTarget = TimelineTarget(a);
        a.engaged = true;
        a.man.Halt();
    }
}

void CamRuntime::Tick(const Input &in, int32_t steps[kMotorConnectors]) {
    for (int c = 0; c < kMotorConnectors; ++c) steps[c] = 0;
    ++m_ticks;
    if (!m_loaded) {
        m_outputs = 0;
        return;
    }

    // ---- requests
    if (m_abortReq != m_abortSeen) {
        m_abortSeen = m_abortReq;
        m_crank = CrankState::Off;
        m_velQ24 = 0;
        m_tl = TimelineState::Idle;
        m_rateQ16 = 0;
        m_endOfCycle = false;
        m_crankStopStepQ24 = 0;
        m_limitStopping = false;
        m_waitCur = -1;
        for (int i = 0; i < m_axisCount; ++i) m_axes[i].engaged = false;
        AxisHalt();
    }
    // Jog on: engage from where everything is, nothing moving.
    if (m_jogOnReq != m_jogOnSeen) {
        m_jogOnSeen = m_jogOnReq;
        if (Idle()) {
            m_jogEnding = false;
            // Limits: the profile's crank speed, reached in a quarter second (a driven crank uses its
            // ramp, which also protects the crank motor); the timeline likewise at its speed.
            m_jogVmaxQ24 = m_velTargetQ24 > 0 ? m_velTargetQ24 : 1;
            m_jogAccQ24 = m_driven ? m_rampQ24 : m_jogVmaxQ24 / (kRtSampleHz / 4);
            if (m_jogAccQ24 < 1) m_jogAccQ24 = 1;
            m_jogRateMaxQ16 = m_rateTargetQ16 > 0 ? m_rateTargetQ16 : 1;
            m_jogRateAccQ16 = m_jogRateMaxQ16 / (kRtSampleHz / 4);
            if (m_jogRateAccQ16 < 1) m_jogRateAccQ16 = 1;
            if (m_hasCrank) {
                m_velQ24 = 0;
                m_crank = CrankState::Jogging;
                m_jogCrankTarget = static_cast<int32_t>(static_cast<int64_t>(m_rev) * m_cpr + (m_phaseQ24 >> 24));
                EngageCrankAxes();
            }
            if (m_hasTimeline) {
                m_rateQ16 = 0;
                if (m_tl == TimelineState::Done) m_timeQ16 = m_cycleQ16;
                m_tl = TimelineState::Jogging;
                m_jogTimeTargetQ4 = static_cast<int32_t>(m_timeQ16 >> 12);
                EngageTimelineAxes();
            }
        }
    }
    if (m_jogOffReq != m_jogOffSeen) {
        m_jogOffSeen = m_jogOffReq;
        EndJog();
    }
    if (m_startReq != m_startSeen) {
        m_startSeen = m_startReq;
        if (m_hasCrank && m_crank == CrankState::Off) {
            m_lastEncoder = in.encoder;
            m_crank = m_driven ? CrankState::Running : CrankState::Following;
            EngageCrankAxes();
        } else if (m_hasCrank && m_crank == CrankState::Stopping) {
            m_crank = CrankState::Running;  // speed back up from where it is
        }
        if (m_hasTimeline && (m_tl == TimelineState::Idle || m_tl == TimelineState::Done)) {
            // The axes still follow the time where it came to rest: rewinding it to 0 with them engaged would command
            // every one back to its start in a single sample. They engage again at the trigger, from where they are.
            for (int i = 0; i < m_axisCount; ++i) {
                if (m_axes[i].timeline) m_axes[i].engaged = false;
            }
            m_tl = TimelineState::Armed;
            m_timeQ16 = 0;
            m_triggerKnown = false;
            m_endOfCycle = false;
            for (int k = 0; k < m_waitCount; ++k) m_waits[k].armed = false;  // a change counts from Start
        }
    }
    bool stopNow = false;  // a cycle stop with nothing to finish (the request counters are the main thread's to write)
    if (m_cycleStopReq != m_cycleStopSeen) {
        m_cycleStopSeen = m_cycleStopReq;
        if (m_tl == TimelineState::Running && !m_endOfCycle) {
            // The timeline finishes the cycle it is in; everything else stops as a stop does.
            m_endOfCycle = true;
            EndJog();
            if (m_crank == CrankState::Following) {
                m_crank = CrankState::Off;
                for (int i = 0; i < m_axisCount; ++i) {
                    if (!m_axes[i].timeline) m_axes[i].engaged = false;
                }
            } else if (m_crank == CrankState::Running) {
                m_crank = CrankState::Stopping;
            }
        } else {
            stopNow = true;  // nothing to finish, or Stop again while finishing: stop now
        }
    }
    if (m_stopReq != m_stopSeen || stopNow) {
        m_stopSeen = m_stopReq;
        m_endOfCycle = false;
        EndJog();
        if (m_crank == CrankState::Following) {
            // An external encoder cannot be ramped: the followers just let go.
            m_crank = CrankState::Off;
            for (int i = 0; i < m_axisCount; ++i) {
                if (!m_axes[i].timeline) m_axes[i].engaged = false;
            }
        } else if (m_crank == CrankState::Running) {
            m_crank = CrankState::Stopping;
        }
        if (m_tl == TimelineState::Running) {
            m_tl = TimelineState::Stopping;
            m_stopStepQ16 = static_cast<int32_t>(m_rateQ16 / (m_stopSeconds * kRtSampleHz));
            if (m_stopStepQ16 < 1) m_stopStepQ16 = 1;
        } else if (m_tl == TimelineState::Waiting) {
            // At rest at the wait already: stop there. Finishing the cycle would make the moves the wait held back.
            m_tl = TimelineState::Idle;
            m_rateQ16 = 0;
            m_waitCur = -1;
        } else if (m_tl == TimelineState::Armed || m_tl == TimelineState::Done) {
            m_tl = TimelineState::Idle;
        }
    }
    // Trigger: a command, or an edge on the input while armed (or finished, for a one-shot rerun).
    bool fire = false;
    if (m_triggerReq != m_triggerSeen) {
        m_triggerSeen = m_triggerReq;
        fire = true;
    }
    if (m_inputTrigger) {
        const bool edge = m_triggerKnown && (m_triggerRising ? (in.trigger && !m_lastTrigger)
                                                              : (!in.trigger && m_lastTrigger));
        m_lastTrigger = in.trigger;
        m_triggerKnown = true;
        fire = fire || edge;
    }
    if (fire && (m_tl == TimelineState::Armed || m_tl == TimelineState::Done)) {
        m_timeQ16 = 0;
        m_waitNext = 0;
        m_waitCur = -1;
        m_rateQ16 = m_rateTargetQ16;  // starts at speed: every axis starts from rest
        m_tl = TimelineState::Running;
        EngageTimelineAxes();
    }

    // ---- jog (master positions; the followers below track them like any other master)
    if (Jogging()) JogTick();

    // ---- crank
    if (m_line && m_crank == CrankState::Off && m_pendingUnits != m_cpr) {
        SetCycleUnits(m_pendingUnits);
        if (m_phaseQ24 >= m_cprQ24) {
            m_crankBase += m_phaseQ24 >> 24;
            m_phaseQ24 = 0;
        }
    }
    if (m_crank == CrankState::Following) {
        const int32_t d = in.encoder - m_lastEncoder;  // wraps correctly
        m_lastEncoder = in.encoder;
        const int64_t moved = (static_cast<int64_t>(d) * m_den) << 24;  // each encoder count is den units
        m_phaseQ24 += moved;
        m_movedQ24 += moved;
    } else if (m_crank == CrankState::Running || m_crank == CrankState::Stopping) {
        const int64_t target = m_crank == CrankState::Stopping ? 0 : m_velTargetQ24;
        m_velQ24 = Step(m_velQ24, target, m_crank == CrankState::Stopping && m_crankStopStepQ24 > 0 ? m_crankStopStepQ24 : m_rampQ24);
        m_phaseQ24 += m_velQ24;
        m_movedQ24 += m_velQ24;
        if (m_crank == CrankState::Stopping && m_velQ24 == 0) {
            m_crank = CrankState::Off;
            m_crankStopStepQ24 = 0;
            for (int i = 0; i < m_axisCount; ++i) {
                if (!m_axes[i].timeline) m_axes[i].engaged = false;
            }
        }
    }
    while (m_phaseQ24 >= m_cprQ24) {
        m_phaseQ24 -= m_cprQ24;
        ++m_rev;
        // Cut-length mode: a new length starts with the next cut. Not while hand jogging: jog targets are
        // whole turns of the length the jog started with; the new length is taken when the jog ends.
        if (m_line && m_pendingUnits != m_cpr && m_crank != CrankState::Jogging) SetCycleUnits(m_pendingUnits);
    }
    while (m_phaseQ24 < 0) {
        m_phaseQ24 += m_cprQ24;
        --m_rev;
    }
    if (m_crankMotor >= 0) {
        // Whole motor steps in the master units so far (floor, also below zero when jogging back).
        const int64_t units = m_crankBase + static_cast<int64_t>(m_rev) * m_cpr + (m_phaseQ24 >> 24);
        const int64_t total = units >= 0 ? units / m_den : -((-units + m_den - 1) / m_den);
        const int64_t d = total - m_crankMotorLast;
        m_crankMotorLast = total;
        steps[m_crankMotor] += m_crankMotorSign * static_cast<int32_t>(d);
        m_crankMotorPos += d;
    }

    // ---- timeline
    for (int k = 0; k < m_waitCount; ++k) {
        if ((in.waitOpposite >> k) & 1u) m_waits[k].armed = true;  // the other level seen: a change can follow
    }
    const uint16_t ready = WaitReady(in.waitMet);
    if (m_tl == TimelineState::Waiting) {
        const bool skip = m_waitSkipReq != m_waitSkipSeen;
        m_waitSkipSeen = m_waitSkipReq;
        const Wait &w = m_waits[m_waitCur];
        if (skip || ((ready >> m_waitCur) & 1u)) {
            // Runs on from the wait, at the rate it had: the axes leave their dwell as drawn.
            WaitPassed(m_waitCur);
            m_waitNext = m_waitCur + 1;
            m_waitCur = -1;
            m_tl = TimelineState::Running;
        } else if (w.timeout != 0 && ++m_waited >= w.timeout) {
            // Timed out: the timeline stops there (the engine faults, the motors holding).
            ++m_waits[m_waitCur].timeouts;
            ++m_waitEvents;
            m_waitEventIndex = m_waitCur;
            m_waitCur = -1;
            m_tl = TimelineState::Idle;
            m_rateQ16 = 0;
            m_endOfCycle = false;
        } else if (w.timeout == 0) {
            ++m_waited;
        }
    } else {
        m_waitSkipSeen = m_waitSkipReq;  // a skip asked with nothing holding is forgotten
    }
    if (m_tl == TimelineState::Running || m_tl == TimelineState::Stopping) {
        if (m_tl == TimelineState::Stopping) {
            m_rateQ16 = static_cast<int32_t>(Step(m_rateQ16, 0, m_stopStepQ16));
        } else {
            m_rateQ16 = static_cast<int32_t>(Step(m_rateQ16, m_rateTargetQ16, m_rateStepQ16));
        }
        int64_t next = m_timeQ16 + m_rateQ16;
        const bool held = WaitCrossing(m_timeQ16, &next, ready);
        m_timeQ16 = next;
        if (held) {
            // On the wait: a stop ramping down ends here (the axes are at rest at it); else wait.
            if (m_tl == TimelineState::Stopping) {
                m_tl = TimelineState::Idle;
                m_rateQ16 = 0;
                m_waitCur = -1;
            } else {
                m_tl = TimelineState::Waiting;
                m_waited = 0;
                ++m_waits[m_waitCur].holds;
            }
        } else if (m_endOfCycle && m_timeQ16 >= m_cycleQ16) {
            // The last cycle's end: rest there. Idle, not Done: a stopped one-shot waits for Start, not the trigger.
            m_timeQ16 = m_cycleQ16;
            m_tl = TimelineState::Idle;
            m_rateQ16 = 0;
            m_endOfCycle = false;
        } else if (m_repeat) {
            while (m_timeQ16 >= m_cycleQ16) {
                m_timeQ16 -= m_cycleQ16;
                ++m_cycle;
                m_waitNext = 0;  // a new cycle: every wait holds again
                int64_t t = m_timeQ16;
                if (WaitCrossing(0, &t, WaitReady(in.waitMet))) {
                    m_timeQ16 = t;
                    m_tl = TimelineState::Waiting;
                    m_waited = 0;
                    ++m_waits[m_waitCur].holds;
                    break;
                }
            }
        } else if (m_timeQ16 >= m_cycleQ16) {
            m_timeQ16 = m_cycleQ16;
            m_tl = TimelineState::Done;
        }
        if (m_tl == TimelineState::Stopping && m_rateQ16 == 0) m_tl = TimelineState::Idle;
    }

    // ---- outputs: crank pulses while the crank is engaged, timeline pulses while the timeline runs
    // (both including stopping and hand jog)
    uint8_t out = 0;
    const bool tlOn = m_tl == TimelineState::Running || m_tl == TimelineState::Stopping || m_tl == TimelineState::Jogging ||
                      m_tl == TimelineState::Waiting;
    const bool crankOn = m_crank != CrankState::Off;
    const int64_t phase = DesignPhase();
    // Holding at a wait: everything before its time has happened, nothing at it ("wait, then do").
    const int64_t tlTime = m_tl == TimelineState::Waiting ? m_timeQ16 - 1 : m_timeQ16;
    for (int i = 0; i < m_pulseCount; ++i) {
        const Pulse &q = m_pulses[i];
        const int64_t x = q.crank ? phase : tlTime;
        if ((q.crank ? crankOn : tlOn) && x >= q.on && x < q.off) out |= q.mask;
    }
    m_outputs = out;

    if (m_limitStopping && m_crank != CrankState::Stopping && m_tl != TimelineState::Stopping) m_limitStopping = false;

    // ---- followers
    const int64_t movedUnits = m_movedQ24 >> 24;  // whole units; the fraction stays for the next sample
    m_movedQ24 -= movedUnits << 24;
    for (int i = 0; i < m_axisCount; ++i) {
        Axis &a = m_axes[i];
        if (!a.engaged) {
            a.lastStep = 0;
            continue;
        }
        int64_t d = 0;
        if (a.gear) {
            if (movedUnits != 0) {
                a.gearAcc += movedUnits * a.gearP;
                d = a.gearAcc / a.gearQ;
                a.gearAcc -= d * a.gearQ;
                if (a.gearAcc < 0) {  // the division truncates: keep the remainder positive, which is floor
                    a.gearAcc += a.gearQ;
                    --d;
                }
            }
        } else {
            const int64_t target = a.timeline ? TimelineTarget(a) : CrankTarget(a);
            d = target - a.lastTarget;
            a.lastTarget = target;
        }
        a.pos += d;
        a.lastStep = d;
        steps[a.connector] += a.motorSign * static_cast<int32_t>(d);
        // Toward a tripped limit: a controlled stop of everything (once; the stop runs on past the switch).
        if (!m_limitStopping && TowardLimit(in, i, d)) LimitStop(i, d > 0 ? 1 : -1);
    }
    // ---- homing: Home all starts the next order once the last one is done
    if (m_homeAll != 0 && !AnyHoming()) {
        int order = kHomeOrderMax + 1;
        for (int i = 0; i < m_axisCount; ++i) {
            if ((m_homeAll >> i) & 1u) order = std::min(order, m_axes[i].hc.order);
        }
        for (int i = 0; i < m_axisCount; ++i) {
            if (((m_homeAll >> i) & 1u) && m_axes[i].hc.order == order) {
                m_homeAll = static_cast<uint8_t>(m_homeAll & ~(1u << i));
                AxisHome(i);
            }
        }
    }
    // ---- axis jog
    for (int i = 0; i < m_axisCount; ++i) {
        Axis &a = m_axes[i];
        if (a.engaged) continue;
        if (a.hphase != 0) HomeStep(i, ((in.home >> i) & 1u) != 0);
        if (!a.man.Active()) {
            a.limitStop = false;
            continue;
        }
        const int64_t now = a.man.Tick(a.pos);
        const int64_t d = now - a.pos;
        steps[a.connector] += a.motorSign * static_cast<int32_t>(d);
        a.pos = now;
        a.lastStep = d;
        // Toward a tripped limit: this axis ramps to rest over the limit stop time; away from it is how it clears.
        if (TowardLimit(in, i, d)) {
            const int64_t v = a.man.VelocityQ24();
            a.man.StopRapid((v < 0 ? -v : v) / m_limitSamples + 1);
            if (!a.limitStop) {  // the first sample of this stop: where the limit tripped
                const int side = d > 0 ? 0 : 1;
                a.limitStop = true;
                ++a.trips[side];
                a.tripAt[side] = a.pos;
            }
            if (a.hphase != 0) HomeEnd(i, HomeFail::Limit);
        }
        if (!a.man.Active()) a.limitStop = false;  // at rest in this sample: the next move's stop counts again
    }
}

const char *CamRuntime::HomeFailText(HomeFail f) {
    switch (f) {
        case HomeFail::None: return "";
        case HomeFail::Stopped: return "stopped";
        case HomeFail::Limit: return "a limit tripped first";
        case HomeFail::NotFound: return "the switch wasn't found within the search length";
        case HomeFail::NoRelease: return "the switch didn't release";
    }
    return "?";
}

namespace {
enum HomePhase : uint8_t {
    kHomeIdle = 0,
    kHomeBegin,      // the first sample: on the switch or not
    kHomeClear,      // started on it: away until it releases
    kHomeClearBack,  // ... then the back-off further
    kHomeSearch,     // toward it at the fast speed
    kHomeSearchStop, // tripped: ramp to rest
    kHomeBack,       // away until it releases
    kHomeBackMove,   // ... then the back-off further
    kHomeLatch,      // toward it at the slow speed
    kHomeLatchStop,  // latched: ramp to rest
    kHomeToStart,    // to the cam's start
};
int64_t Abs64(int64_t v) { return v < 0 ? -v : v; }
}  // namespace

bool CamRuntime::AxisHome(int axis) {
    if (axis < 0 || axis >= m_axisCount) return false;
    Axis &a = m_axes[axis];
    if (!a.hc.has || !Release(a)) return false;
    a.man.Halt();
    a.hphase = kHomeBegin;
    a.hresult = HomeFail::None;
    m_homed = static_cast<uint8_t>(m_homed & ~(1u << axis));
    return true;
}

void CamRuntime::AxisHomeAll() {
    m_homeAll = 0;
    for (int i = 0; i < m_axisCount; ++i) {
        if (m_axes[i].hc.has && !m_axes[i].engaged) m_homeAll = static_cast<uint8_t>(m_homeAll | (1u << i));
    }
    for (int i = 0; i < m_axisCount; ++i) {
        if ((m_homeAll >> i) & 1u) m_homed = static_cast<uint8_t>(m_homed & ~(1u << i));
    }
}

bool CamRuntime::AnyHoming() const {
    for (int i = 0; i < m_axisCount; ++i) {
        if (m_axes[i].hphase != 0) return true;
    }
    return false;
}

uint8_t CamRuntime::HomingMask() const {
    uint8_t m = 0;
    for (int i = 0; i < m_axisCount; ++i) {
        if (m_axes[i].hphase != 0 || ((m_homeAll >> i) & 1u)) m = static_cast<uint8_t>(m | (1u << i));
    }
    return m;
}

// Ends the axis's homing: why (None: homed). A failure stops the rest of a Home all.
void CamRuntime::HomeEnd(int axis, HomeFail why) {
    Axis &a = m_axes[axis];
    a.hphase = kHomeIdle;
    a.hresult = why;
    ++a.hends;
    if (why == HomeFail::None) {
        m_homed = static_cast<uint8_t>(m_homed | (1u << axis));
        return;
    }
    a.man.Stop();
    if (why == HomeFail::Stopped) return;
    ++m_homeEvents;
    m_homeAxis = static_cast<int8_t>(axis);
    m_homeFail = why;
    m_homeAll = 0;
    for (int i = 0; i < m_axisCount; ++i) {
        if (i != axis && m_axes[i].hphase != 0) {
            m_axes[i].hphase = kHomeIdle;
            m_axes[i].hresult = HomeFail::Stopped;
            ++m_axes[i].hends;
            m_axes[i].man.Stop();
        }
    }
}

// One sample of the axis's homing, before its generator moves it. tripped: the home switch now.
void CamRuntime::HomeStep(int axis, bool tripped) {
    Axis &a = m_axes[axis];
    const Axis::HomeCfg &c = a.hc;
    const double toward = c.dir * c.fast, away = -c.dir * c.fast;
    const int64_t moved = Abs64(a.pos - a.hfrom);
    switch (a.hphase) {
        case kHomeBegin:
            a.hfrom = a.pos;
            if (tripped) {
                a.man.Jog(a.pos, away, c.accel);
                a.hphase = kHomeClear;
            } else {
                a.man.Jog(a.pos, toward, c.accel);
                a.hphase = kHomeSearch;
            }
            break;
        case kHomeClear:
            if (!tripped) {
                a.man.MoveTo(a.pos, a.pos - c.dir * c.back, c.fast, c.accel);
                a.hphase = kHomeClearBack;
            } else if (c.search > 0 && moved > c.search) {
                HomeEnd(axis, HomeFail::NoRelease);
            }
            break;
        case kHomeClearBack:
            if (!a.man.Active()) {
                a.hfrom = a.pos;
                a.man.Jog(a.pos, toward, c.accel);
                a.hphase = kHomeSearch;
            }
            break;
        case kHomeSearch:
            if (tripped) {
                a.man.StopAt(c.decel);
                a.hphase = kHomeSearchStop;
            } else if (c.search > 0 && moved > c.search) {
                HomeEnd(axis, HomeFail::NotFound);
            }
            break;
        case kHomeSearchStop:
            if (!a.man.Active()) {
                a.hfrom = a.pos;
                a.man.Jog(a.pos, away, c.accel);
                a.hphase = kHomeBack;
            }
            break;
        case kHomeBack:
            if (!tripped) {
                a.man.MoveTo(a.pos, a.pos - c.dir * c.back, c.fast, c.accel);
                a.hphase = kHomeBackMove;
            } else if (c.search > 0 && moved > c.search) {
                HomeEnd(axis, HomeFail::NoRelease);
            }
            break;
        case kHomeBackMove:
            if (!a.man.Active()) {
                a.hfrom = a.pos;
                a.man.Jog(a.pos, c.dir * c.slow, c.accel);
                a.hphase = kHomeLatch;
            }
            break;
        case kHomeLatch:
            if (tripped) {
                a.hlatch = a.pos;
                a.man.StopAt(c.decel);
                a.hphase = kHomeLatchStop;
            } else if (moved > 4 * c.back + 100) {
                HomeEnd(axis, HomeFail::NotFound);  // it released going away, and doesn't trip again coming back
            }
            break;
        case kHomeLatchStop:
            if (!a.man.Active()) {
                // The latch is the home position: everything the axis knows moves with it (no steps).
                a.pos += c.at - a.hlatch;
                int64_t start = 0;
                if (c.thenStart && CamPosition(axis, true, &start) && start != a.pos) {
                    a.man.MoveTo(a.pos, start, c.fast, c.accel);
                    a.hphase = kHomeToStart;
                } else {
                    HomeEnd(axis, HomeFail::None);
                }
            }
            break;
        case kHomeToStart:
            if (!a.man.Active()) HomeEnd(axis, HomeFail::None);
            break;
        default:
            a.hphase = kHomeIdle;
            break;
    }
}

uint16_t CamRuntime::WaitReady(uint16_t met) const {
    uint16_t ready = 0;
    for (int k = 0; k < m_waitCount; ++k) {
        if (((met >> k) & 1u) && (!m_waits[k].edge || m_waits[k].armed)) ready = static_cast<uint16_t>(ready | (1u << k));
    }
    return ready;
}

// A wait passed: one that needs a change needs another before it passes again.
void CamRuntime::WaitPassed(int k) {
    if (k >= 0 && k < m_waitCount) m_waits[k].armed = false;
}

bool CamRuntime::WaitCrossing(int64_t t0, int64_t *t1, uint16_t met) {
    while (m_waitNext < m_waitCount) {
        const Wait &w = m_waits[m_waitNext];
        if (w.tQ16 < t0) {  // behind the clock already (a jog moved it): not this cycle
            ++m_waitNext;
            continue;
        }
        if (w.tQ16 > *t1) return false;  // ahead: not reached in this sample
        if ((met >> m_waitNext) & 1u) {  // its condition holds: it passes without stopping
            ++m_waits[m_waitNext].holds;
            WaitPassed(m_waitNext);
            ++m_waitNext;
            continue;
        }
        *t1 = w.tQ16;  // reached, not met: the clock stops exactly on it
        m_waitCur = m_waitNext;
        return true;
    }
    return false;
}

bool CamRuntime::TowardLimit(const Input &in, int axis, int64_t d) const {
    const uint8_t bit = static_cast<uint8_t>(1u << axis);
    return (d > 0 && (in.limitPlus & bit)) || (d < 0 && (in.limitMinus & bit));
}

// A limit tripped with the cam moving an axis toward it: everything comes to rest over the limit stop time, the axes
// along their profiles (the master ramps), so they stay in step. An encoder can't be ramped: its axes let go and ramp
// themselves from the speed they had.
void CamRuntime::LimitStop(int axis, int side) {
    m_limitStopping = true;
    m_limitAxis = static_cast<int8_t>(axis);
    m_limitSide = static_cast<int8_t>(side);
    ++m_limitEvents;
    m_endOfCycle = false;
    m_jogEnding = false;
    if (m_crank == CrankState::Following) {
        for (int i = 0; i < m_axisCount; ++i) {
            Axis &a = m_axes[i];
            if (a.timeline || !a.engaged) continue;
            a.engaged = false;
            const int64_t v = a.lastStep * (int64_t(1) << 24);
            a.man.StopFrom(a.pos, v, (v < 0 ? -v : v) / m_limitSamples + 1);
        }
        m_crank = CrankState::Off;
    } else if (m_crank == CrankState::Running || m_crank == CrankState::Stopping || m_crank == CrankState::Jogging) {
        m_crank = CrankState::Stopping;
        m_crankStopStepQ24 = (m_velQ24 < 0 ? -m_velQ24 : m_velQ24) / m_limitSamples + 1;
    }
    if (m_tl == TimelineState::Running || m_tl == TimelineState::Stopping || m_tl == TimelineState::Jogging) {
        m_tl = TimelineState::Stopping;
        const int32_t r = m_rateQ16 < 0 ? -m_rateQ16 : m_rateQ16;
        m_stopStepQ16 = static_cast<int32_t>(r / m_limitSamples) + 1;
    }
}

namespace {

// One sample of a point-to-point move toward target: the fastest speed that can still stop at the
// target, capped at vmax, reached at most acc per sample. Returns the new velocity; snaps onto the
// target once it is within one step.
int64_t JogVelocity(int64_t pos, int64_t target, int64_t vel, int64_t vmax, int64_t acc, bool *arrived) {
    const int64_t err = target - pos;
    const int64_t mag = err < 0 ? -err : err;
    const int64_t vabs = vel < 0 ? -vel : vel;
    if (mag <= acc && vabs <= acc) {
        *arrived = true;
        return err;  // the last step lands on the target
    }
    *arrived = false;
    float vd = sqrtf(2.0f * static_cast<float>(acc) * static_cast<float>(mag));
    if (vd > static_cast<float>(vmax)) vd = static_cast<float>(vmax);
    const int64_t want = err < 0 ? -static_cast<int64_t>(vd) : static_cast<int64_t>(vd);
    return Step(vel, want, acc);
}

}  // namespace

void CamRuntime::EndJog() {
    if (!Jogging() || m_jogEnding) return;
    // Come to rest where the current speed can stop.
    m_jogEnding = true;
    const int64_t pos = static_cast<int64_t>(m_rev) * m_cprQ24 + m_phaseQ24;
    const float v = static_cast<float>(m_velQ24);
    const int64_t d = static_cast<int64_t>(v * v / (2.0f * static_cast<float>(m_jogAccQ24)));
    m_jogCrankEndQ24 = pos + (m_velQ24 >= 0 ? d : -d);
    const float r = static_cast<float>(m_rateQ16);
    const int64_t dt = static_cast<int64_t>(r * r / (2.0f * static_cast<float>(m_jogRateAccQ16)));
    m_jogTimeEndQ16 = m_timeQ16 + (m_rateQ16 >= 0 ? dt : -dt);
}

void CamRuntime::JogTick() {
    if (m_crank == CrankState::Jogging) {
        const int64_t pos = static_cast<int64_t>(m_rev) * m_cprQ24 + m_phaseQ24;
        const int64_t target = m_jogEnding ? m_jogCrankEndQ24 : (static_cast<int64_t>(m_jogCrankTarget) << 24);
        bool arrived = false;
        const int64_t v = JogVelocity(pos, target, m_velQ24, m_jogVmaxQ24, m_jogAccQ24, &arrived);
        m_phaseQ24 += v;  // normalized into the revolution by the crank code
        m_movedQ24 += v;
        m_velQ24 = arrived ? 0 : v;
        if (arrived && m_jogEnding) {
            m_crank = CrankState::Off;
            for (int i = 0; i < m_axisCount; ++i) {
                if (!m_axes[i].timeline) m_axes[i].engaged = false;
            }
        }
    }
    if (m_tl == TimelineState::Jogging) {
        int64_t target = m_jogEnding ? m_jogTimeEndQ16 : (static_cast<int64_t>(m_jogTimeTargetQ4) << 12);
        if (target < 0) target = 0;
        if (target > m_cycleQ16) target = m_cycleQ16;
        bool arrived = false;
        const int64_t v = JogVelocity(m_timeQ16, target, m_rateQ16, m_jogRateMaxQ16, m_jogRateAccQ16, &arrived);
        m_timeQ16 += v;
        m_rateQ16 = arrived ? 0 : static_cast<int32_t>(v);
        if (arrived && m_jogEnding) m_tl = TimelineState::Idle;
    }
    if (m_crank != CrankState::Jogging && m_tl != TimelineState::Jogging) m_jogEnding = false;
}

bool CamRuntime::Release(Axis &a) {
    if (!a.engaged) return true;
    // An axis stays engaged after its master comes to rest (a timeline that finished or stopped): let it go
    // then. Its master engages it again, from where it is, before it moves.
    const bool rest = a.timeline ? (m_tl == TimelineState::Idle || m_tl == TimelineState::Done) : m_crank == CrankState::Off;
    if (!rest) return false;
    a.engaged = false;
    return true;
}

void CamRuntime::AxisJog(int axis, double countsPerSec, double accel) {
    if (axis < 0 || axis >= m_axisCount) return;
    Axis &a = m_axes[axis];
    if ((!a.man.Active() && countsPerSec == 0.0) || !Release(a)) return;
    a.man.Jog(a.pos, countsPerSec, accel);
}

void CamRuntime::AxisMoveTo(int axis, int64_t counts, double countsPerSec, double accel) {
    if (axis < 0 || axis >= m_axisCount) return;
    Axis &a = m_axes[axis];
    if ((!a.man.Active() && counts == a.pos) || !Release(a)) return;
    a.man.MoveTo(a.pos, counts, countsPerSec, accel);
}

void CamRuntime::AxisStop(int axis) {
    if (axis < 0 || axis >= m_axisCount) return;
    if (m_axes[axis].hphase != 0 || ((m_homeAll >> axis) & 1u)) {
        m_homeAll = 0;  // a stop of one ends the Home all
        for (int i = 0; i < m_axisCount; ++i) {
            if (m_axes[i].hphase != 0) HomeEnd(i, HomeFail::Stopped);
        }
    }
    m_axes[axis].man.Stop();
}

void CamRuntime::AxisStopRapid(int axis) {
    if (axis < 0 || axis >= m_axisCount) return;
    const int64_t v = m_axes[axis].man.VelocityQ24();
    m_axes[axis].man.StopRapid((v < 0 ? -v : v) / m_limitSamples + 1);
}

void CamRuntime::AxisHalt() {
    m_homeAll = 0;
    for (int i = 0; i < m_axisCount; ++i) {
        if (m_axes[i].hphase != 0) {
            m_axes[i].hphase = 0;
            m_axes[i].hresult = HomeFail::Stopped;
            ++m_axes[i].hends;
        }
        m_axes[i].man.Halt();
    }
}

bool CamRuntime::AnyAxisMoving() const {
    for (int i = 0; i < m_axisCount; ++i) {
        if (m_axes[i].man.Active() || m_axes[i].hphase != 0) return true;
    }
    return m_homeAll != 0;
}

void CamRuntime::ZeroAxis(int axis) {
    if (axis < 0 || axis >= m_axisCount) return;
    Axis &a = m_axes[axis];
    if (a.man.Active() || a.hphase != 0 || !Release(a)) return;
    a.pos = 0;
    m_homed = static_cast<uint8_t>(m_homed & ~(1u << axis));  // it no longer counts from the switch
}

bool CamRuntime::CamPosition(int axis, bool atStart, int64_t *counts) const {
    if (axis < 0 || axis >= m_axisCount || !m_axes[axis].fixed) return false;
    const Axis &a = m_axes[axis];
    // Positions count from the profile's start (pos is 0 at the load, with the master at 0).
    int64_t target;
    if (!a.timeline) {
        target = CrankTarget(a);
    } else if (atStart) {
        target = a.table[0] + static_cast<int64_t>(m_cycle) * a.advance;
    } else {
        target = TimelineTarget(a);
    }
    *counts = target - a.table[0];
    return true;
}

void CamRuntime::Status(RtStatus *s) const {
    s->crank = m_crank;
    s->timeline = m_tl;
    s->crankPhaseQ24 = m_phaseQ24;
    s->crankRev = m_rev;
    s->crankVelQ24 = m_crank == CrankState::Following ? 0 : m_velQ24;
    s->timeQ16 = m_timeQ16;
    s->cycle = m_cycle;
    s->rateQ16 = m_rateQ16;
    s->crankMotorPos = m_crankMotorPos;
    s->outputs = m_outputs;
    s->endOfCycle = m_endOfCycle;
    s->limitStopping = m_limitStopping;
    s->waitIndex = m_tl == TimelineState::Waiting ? m_waitCur + 1 : 0;
    s->waitedSamples = m_tl == TimelineState::Waiting ? m_waited : 0;
    s->cycleUnits = m_cpr;
    s->pendingUnits = m_pendingUnits;
    s->designPhase = DesignPhase();
    s->hasCutGap = m_line && m_syncAxis >= 0;
    CutGap(&s->cutGapUnits, &s->inSync, &s->syncLeftUnits);
    for (int i = 0; i < kRtAxes; ++i) s->axisPos[i] = i < m_axisCount ? m_axes[i].pos : 0;
    s->ticks = m_ticks;
}

}  // namespace ecam
