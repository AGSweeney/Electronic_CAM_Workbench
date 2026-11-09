/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Runs a loaded CamProject: called once per ClearCore sample (5 kHz) with the master inputs, it
// returns the steps for every motor connector. Portable C++14 with no ClearCore dependency, so
// the same code is tested on the PC (tests/test_runtime.cpp) and runs in the firmware's sample
// interrupt.
//
// Build() (main thread, nothing moving) samples each axis into a table of motor counts at
// kRtIntervals + 1 evenly spaced master positions, in double precision. Tick() uses integers only:
// 64-bit fixed point, linear interpolation between table entries.
//
//   Crank   The master position is kept as a phase within one revolution plus a revolution count,
//           so it never overflows. Encoder: the phase moves by the encoder's change. Driven: the
//           runtime is the crank's motion generator (velocity ramped at the crank ramp) and also
//           outputs the crank motor's steps.
//   Timeline  Time in samples, scaled by the speed override (ramped), within one cycle plus a
//           cycle count. One-shot: stops at the end of the cycle (the longest axis or the last
//           timeline output pulse). Repeat: wraps.
//   Outputs IO-0..IO-5 (Outputs()): on inside one of their pulses, crank pulses in the crank phase while
//           the crank is engaged, timeline pulses in the timeline time while it runs; all off otherwise.
//
// Steps are deltas of each axis's target, so engaging an axis takes its current position as the
// profile position at that moment: no jump. Open profiles add their advance every cycle.
//
// An electronic gear has no table. Each sample it takes the crank's motion in master units, from the encoder,
// the crank's ramp or a jog alike, and multiplies it by an exact fraction, carrying the remainder: it is
// right to the count after any number of revolutions, either way.
//
// Commands come from the main thread through sequence counters (no read-modify-write shared with
// the interrupt); Tick() acts on them at the next sample.
//
// Jog (hand mode): the master goes where the host says instead of following the encoder, the crank
// speed or the clock. Targets are absolute master counts (crank) and samples (timeline); the master
// moves there with a velocity and acceleration limit (the profile's crank speed / timeline speed,
// reached in a quarter second; a driven crank uses its own ramp), and every engaged axis follows its
// profile. A driven crank's motor turns with it.
//
// Axis jog (setup): one axis moves on its own, to set the machine up: a velocity jog, or a move to a
// position, each with its own speed and acceleration. Only while the cam has nothing engaged (the engine
// also refuses it while the timeline is armed). Start, hand jog and a trigger engage axes: an axis jog
// still moving then ends at once (the engine refuses those while one moves); abort ends every one at once.
// Positions are commanded counts from the profile's start (0 at the load, as status reports them), moved
// by the cam and by axis jog alike; ZeroAxis makes the axis's position here 0, the profile's start.
//
// Homing (setup) runs on the axis jog generator, so it moves as axis jog does: off the home switch if it starts on it
// (and the back-off further), a search toward it at the fast speed, a back-off until it releases and the back-off
// distance more, toward it again at the slow speed, where the sample the switch trips is the latch: the axis's position
// there becomes the home position. Then it stays, or moves to the cam's start. A limit, a search past its length, or a
// switch that doesn't release fails it (HomeEvents, for the engine's fault); a stop or an abort ends it, no failure.
// Homed holds until a new profile, ZeroAxis, or ClearHomed (the engine's disable, a fault).

#include "CamProfile.h"
#include "ManualMove.h"

#include <stdint.h>

namespace ecam {

constexpr int kRtIntervals = 2048;
constexpr int32_t kRtSampleHz = 5000;
constexpr int kRtAxes = kMotorConnectors;

enum class CrankState : uint8_t { Off = 0, Following, Running, Stopping, Jogging };
enum class TimelineState : uint8_t { Idle = 0, Armed, Running, Stopping, Done, Jogging, Waiting };

const char *CrankStateName(CrankState s);
const char *TimelineStateName(TimelineState s);

struct RtStatus {
    CrankState crank = CrankState::Off;
    TimelineState timeline = TimelineState::Idle;
    int64_t crankPhaseQ24 = 0;  // master counts within the revolution, << 24
    int32_t crankRev = 0;
    int64_t crankVelQ24 = 0;    // master counts per sample, << 24 (driven crank)
    int64_t timeQ16 = 0;        // samples into the cycle, << 16
    int32_t cycle = 0;
    int32_t rateQ16 = 0;        // timeline speed, 65536 = as drawn
    uint8_t outputs = 0;        // timeline outputs IO-0..IO-5, bit n = IO-n on
    bool endOfCycle = false;    // the timeline is running its last cycle (Stop): it comes to rest at the end
    bool limitStopping = false; // a limit's controlled stop is running
    int waitIndex = 0;          // the wait holding the timeline, 1-based (0 none)
    uint32_t waitedSamples = 0; // how long it has held
    // Cut-length mode: master units (encoder counts) per cut now and from the next cut on, and the
    // position in the cycle as drawn (design units, 0 .. design counts).
    int32_t cycleUnits = 0, pendingUnits = 0;
    // Cut-length mode with a sync: line distance (encoder counts) from the blade to the next cut line on
    // the material, 0 while synchronized on it; and in the sync, the line distance left in it.
    bool hasCutGap = false, inSync = false;
    double cutGapUnits = 0.0, syncLeftUnits = 0.0;
    int64_t designPhase = 0;
    int64_t axisPos[kRtAxes] = {};  // commanded motor counts since the profile was loaded
    int64_t crankMotorPos = 0;
    uint32_t ticks = 0;
};

class CamRuntime {
public:
    // Main thread, nothing moving. nullptr or an error message.
    const char *Build(const CamProject &project);
    void Unload();
    bool Loaded() const { return m_loaded; }
    // Nothing moving: the crank is off and the timeline is not running or stopping.
    bool Idle() const;

    int AxisCount() const { return m_axisCount; }
    int AxisConnector(int i) const { return m_axes[i].connector; }
    // Connector of the crank motor when a driven crank is used, else -1.
    int CrankConnector() const { return m_crankMotor; }
    bool UsesTimeline() const { return m_hasTimeline; }
    bool UsesCrank() const { return m_hasCrank; }
    bool DrivenCrank() const { return m_driven; }
    // Master units per crank revolution (ppr x gear numerator); each encoder / crank-motor count is
    // UnitsPerCount() units (the gear denominator). Crank phase, jog targets and speeds are in units.
    int32_t CountsPerRev() const { return m_cpr; }
    int32_t UnitsPerCount() const { return m_den; }

    // Requests (main thread).
    void RequestStart() { ++m_startReq; }
    // Stop now: the crank and the timeline ramp to rest wherever they are (the timeline at its stop ramp). For a
    // connection loss: a timeline cycle can be long.
    void RequestStop() { ++m_stopReq; }
    // Stop at the end of the cycle (Stop): a running timeline finishes the cycle it is in and comes to rest at its
    // end, where a closed profile is back at its start, so the next Start begins where the machine is. A crank stops
    // as RequestStop. Asked again while the timeline is finishing: stop now.
    void RequestCycleStop() { ++m_cycleStopReq; }
    void RequestAbort() { ++m_abortReq; }
    void RequestTrigger() { ++m_triggerReq; }
    void SetOverride(double ovr);   // timeline speed (1 = as drawn), ramped
    void SetRpm(double rpm);        // driven crank speed, ramped at the crank ramp
    void SetStopSeconds(double s);  // timeline stop ramp
    // Called now and then while Build() computes the tables (it takes a few hundred ms on the board), so
    // the firmware can keep its network alive. It must not touch this runtime.
    void SetBuildYield(void (*fn)()) { m_yield = fn; }
    // Cut-length mode: the cut length in encoder counts, from the next cut on (the cycle's wait
    // stretches; at once while the crank is off; while hand jogging, when the jog ends). nullptr or why not.
    const char *SetCutCounts(int64_t counts);
    bool LineMode() const { return m_line; }
    int32_t DesignCounts() const { return m_design; }
    int32_t MinCutCounts() const { return m_design - m_wait; }
    double LineCountsPerMm() const { return m_cpm; }
    double Kerf() const { return m_kerf; }  // mm: a piece is the cycle less the kerf

    // Jog. RequestJog(true) engages from where everything is (nothing may be moving); targets are
    // then followed until RequestJog(false), which brings the master to rest and disengages.
    void RequestJog(bool on) {
        if (on) {
            ++m_jogOnReq;
        } else {
            ++m_jogOffReq;
        }
    }
    // Absolute master position: crank in master counts since the load (rev * counts/rev + phase),
    // timeline in samples within the cycle, << 4. 32-bit, so a write is atomic for the interrupt.
    void SetJogCrankCounts(int32_t units) { m_jogCrankTarget = units; }  // master units (see CountsPerRev)
    void SetJogTimeQ4(int32_t samplesQ4) { m_jogTimeTargetQ4 = samplesQ4; }
    bool Jogging() const { return m_crank == CrankState::Jogging || m_tl == TimelineState::Jogging; }
    int64_t TimelineSamplesQ16() const { return m_cycleQ16; }  // cycle length

    // Axis jog. The caller holds the sample interrupt off around each call. Speeds are counts/s, accelerations
    // counts/s^2; both are taken as they are (the engine limits them).
    void AxisJog(int axis, double countsPerSec, double accel);  // velocity; 0 ramps to rest
    void AxisMoveTo(int axis, int64_t counts, double countsPerSec, double accel);
    void AxisStop(int axis);  // ramps to rest
    void AxisStopRapid(int axis);  // ramps to rest over the limit stop time
    void AxisHalt();          // every axis, at once
    // Moving by axis jog or homing.
    bool AxisMoving(int axis) const {
        return axis >= 0 && axis < m_axisCount && (m_axes[axis].man.Active() || m_axes[axis].hphase != 0);
    }
    // A velocity jog the deadman watches (not homing's own).
    bool AxisVelocityJog(int axis) const {
        return axis >= 0 && axis < m_axisCount && m_axes[axis].hphase == 0 && m_axes[axis].man.Active() &&
               m_axes[axis].man.VelocityJog();
    }
    bool AnyAxisMoving() const;
    int64_t AxisPosition(int axis) const { return axis >= 0 && axis < m_axisCount ? m_axes[axis].pos : 0; }
    // At rest, not engaged: its position here becomes 0 (the profile's start).
    void ZeroAxis(int axis);
    // Where the cam has the axis now, in the same counts as AxisPosition: at the crank's position, or a
    // timeline axis at its next start (atStart) or at the timeline time now. A gear or an open profile moves
    // on every cycle and has no fixed place: false.
    bool CamPosition(int axis, bool atStart, int64_t *counts) const;

    // Homing. The caller holds the sample interrupt off around each call.
    enum class HomeFail : uint8_t { None = 0, Stopped, Limit, NotFound, NoRelease };
    static const char *HomeFailText(HomeFail f);
    bool HasHome(int axis) const { return axis >= 0 && axis < m_axisCount && m_axes[axis].hc.has; }
    bool AxisHome(int axis);  // starts it: false without a home switch, or while the cam has the axis
    void AxisHomeAll();       // every axis with a home switch: the lowest order first, the same order together
    bool Homing(int axis) const { return axis >= 0 && axis < m_axisCount && m_axes[axis].hphase != 0; }
    bool AnyHoming() const;
    bool HomeAllRunning() const { return m_homeAll != 0 || AnyHoming(); }
    uint8_t HomedMask() const { return m_homed; }
    uint8_t HomingMask() const;
    void ClearHomed(uint8_t mask) { m_homed = static_cast<uint8_t>(m_homed & ~mask); }
    // Failures (not stops): a count, and the last one's axis and why.
    uint32_t HomeEvents() const { return m_homeEvents; }
    int HomeEventAxis() const { return m_homeAxis; }
    HomeFail HomeEventFail() const { return m_homeFail; }
    // The last homing of the axis: where the switch latched (counts, before the position was set), its result.
    HomeFail AxisHomeResult(int axis) const { return axis >= 0 && axis < m_axisCount ? m_axes[axis].hresult : HomeFail::None; }
    int64_t AxisLatch(int axis) const { return axis >= 0 && axis < m_axisCount ? m_axes[axis].hlatch : 0; }
    // Homings of the axis that ended (homed, failed or stopped) since the load: a new result to read.
    uint32_t AxisHomeEnds(int axis) const { return axis >= 0 && axis < m_axisCount ? m_axes[axis].hends : 0; }
    // An axis jog stopped by a limit: how many times on each side (0 +, 1 -) since the load, and the position where the
    // limit tripped the last time (counts). The switch setup wizard's limit test reads the overtravel from it.
    uint32_t AxisLimitTrips(int axis, int side) const {
        return axis >= 0 && axis < m_axisCount && (side == 0 || side == 1) ? m_axes[axis].trips[side] : 0;
    }
    int64_t AxisLimitTripAt(int axis, int side) const {
        return axis >= 0 && axis < m_axisCount && (side == 0 || side == 1) ? m_axes[axis].tripAt[side] : 0;
    }

    struct Input {
        int32_t encoder = 0;   // EncoderIn position (counts)
        bool trigger = false;  // trigger input level (edge per the profile)
        // Limits tripped now, bit i = axis i (filtered, the contact applied; none in simulate). An axis moving toward a
        // tripped limit is stopped: the cam by a controlled stop of everything, an axis jog by ramping that axis.
        uint8_t limitPlus = 0, limitMinus = 0;
        uint8_t home = 0;  // home switches tripped now, bit i = axis i (filtered, the contact applied)
        uint16_t waitMet = 0;  // timeline waits whose condition holds now, bit k = wait k (in time order)
        uint16_t waitOpposite = 0;  // ... whose input is at the other level now (arms a wait that needs a change)
    };
    // Timeline waits (in time order). A timeout is an event for the engine's fault, the timeline stopped at the wait.
    int WaitCount() const { return m_waitCount; }
    uint32_t WaitHolds(int k) const { return k >= 0 && k < m_waitCount ? m_waits[k].holds : 0; }
    uint32_t WaitTimeouts(int k) const { return k >= 0 && k < m_waitCount ? m_waits[k].timeouts : 0; }
    // A wait that needs a change has seen the other level since it last passed.
    bool WaitArmed(int k) const { return k >= 0 && k < m_waitCount && (!m_waits[k].edge || m_waits[k].armed); }
    uint32_t WaitEvents() const { return m_waitEvents; }
    int WaitEventIndex() const { return m_waitEventIndex; }
    // Simulate: pass the wait holding now.
    void RequestWaitSkip() { ++m_waitSkipReq; }
    // Limit stops (the cam's, each a fault for the engine): a count, and the last one's axis and side (+1 / -1). Read
    // with the interrupt held off.
    uint32_t LimitEvents() const { return m_limitEvents; }
    int LimitEventAxis() const { return m_limitAxis; }
    int LimitEventSide() const { return m_limitSide; }
    // A limit's controlled stop is still running.
    bool LimitStopping() const { return m_limitStopping; }
    int64_t LimitStopSamples() const { return m_limitSamples; }
    // Sample interrupt. steps[c] for motor connector c (0 when unused).
    void Tick(const Input &in, int32_t steps[kMotorConnectors]);
    // Outputs after the last Tick(): bit n = IO-n on. A crank pulse counts while the crank is engaged, a
    // timeline pulse while the timeline runs, stops or jogs. OutputsUsed(): the outputs the profile switches.
    uint8_t Outputs() const { return m_outputs; }
    uint8_t OutputsUsed() const { return m_outputsUsed; }
    // Copy of the state; the caller keeps Tick() from running meanwhile.
    void Status(RtStatus *s) const;

private:
    struct Axis {
        bool timeline = false;
        int connector = 0;
        int32_t motorSign = 1;  // -1: the motor turns the other way; positions don't
        int32_t table[kRtIntervals + 1];
        int64_t advance = 0;      // counts per cycle (open profiles)
        int64_t spanQ16 = 0;      // timeline: samples << 16
        int64_t lastTarget = 0;
        int64_t pos = 0;          // commanded counts since the load
        bool engaged = false;
        // Electronic gear: gearP / gearQ motor counts per master unit, whole numbers. gearAcc is the part of a
        // count not output yet, 0 <= gearAcc < gearQ, so nothing is ever lost to rounding.
        bool gear = false;
        int64_t gearP = 0, gearQ = 1, gearAcc = 0;
        bool fixed = false;  // closed and not a gear: the cam has a place for it at every master position
        ManualMove man;  // axis jog
        int64_t lastStep = 0;  // counts moved in the last sample (a limit stop ramps an encoder's axis from it)
        // Homing: the settings in counts, and the sequence's phase (0 none), where the phase began, the latch.
        struct HomeCfg {
            bool has = false, thenStart = true;
            int dir = -1, order = 1;
            double fast = 1.0, slow = 1.0, accel = 100.0;  // counts/s, counts/s^2
            double decel = 100.0;                          // the stops on the switch, counts/s^2
            int64_t back = 0, at = 0, search = 0;          // counts (search 0: no limit)
        } hc;
        uint8_t hphase = 0;
        HomeFail hresult = HomeFail::None;
        int64_t hfrom = 0, hlatch = 0;
        uint32_t hends = 0;
        // Axis jog stopped by a limit: trips by side, where, and whether this move's stop is under way.
        uint32_t trips[2] = {0, 0};
        int64_t tripAt[2] = {0, 0};
        bool limitStop = false;
    };
    void HomeStep(int axis, bool tripped);
    void HomeEnd(int axis, HomeFail why);
    void LimitStop(int axis, int side);
    bool TowardLimit(const Input &in, int axis, int64_t d) const;
    bool Release(Axis &a);

    int64_t CrankTarget(const Axis &a) const;
    // Crank position in the cycle as drawn: the phase, or in cut-length mode the phase less the
    // stretch (clamped to the wait at 0).
    int64_t DesignPhase() const;
    // The carriage's distance to the next cut line (encoder counts), and whether it is in the sync.
    void CutGap(double *gap, bool *inSync, double *syncLeft) const;
    void SetCycleUnits(int32_t units);
    int64_t TimelineTarget(const Axis &a) const;
    void JogTick();
    void EndJog();
    void EngageCrankAxes();
    void EngageTimelineAxes();

    Axis m_axes[kRtAxes];
    int m_axisCount = 0;
    bool m_loaded = false;
    void (*m_yield)() = nullptr;
    bool m_hasCrank = false, m_hasTimeline = false, m_driven = false;
    int m_crankMotor = -1;
    int32_t m_crankMotorSign = 1;

    // Cut-length mode
    bool m_line = false;
    double m_cpm = 0.0;           // encoder counts per mm
    double m_kerf = 0.0;          // mm
    int32_t m_design = 1;         // counts per cut as drawn
    int32_t m_wait = 0;           // the wait at the start of the cycle, design counts
    int m_syncAxis = -1;          // the carriage (the axis with the sync), -1 none
    int32_t m_syncStart = 0, m_syncEnd = 0;  // the sync window, design counts
    int64_t m_carriageAtSync = 0; // carriage motor counts at the sync start
    double m_lineCountsPerCarriageCount = 0.0;  // encoder counts per carriage motor count (cpm / cpu)
    volatile int32_t m_pendingUnits = 0;

    // Crank
    int32_t m_cpr = 1;   // master units per crank revolution (cut-length mode: counts per cut now)
    int32_t m_den = 1;   // master units per encoder / crank-motor count
    int64_t m_cprQ24 = 1;
    CrankState m_crank = CrankState::Off;
    int64_t m_phaseQ24 = 0;
    int32_t m_rev = 0;
    int64_t m_velQ24 = 0;
    volatile int64_t m_velTargetQ24 = 0;
    int64_t m_rampQ24 = 1;   // velocity change per sample
    int32_t m_lastEncoder = 0;
    int64_t m_crankMotorLast = 0;  // total crank counts last output
    int64_t m_crankBase = 0;  // cut-length mode: keeps rev x cycle + phase continuous when the cycle changes
    int64_t m_crankMotorPos = 0;
    int64_t m_movedQ24 = 0;  // the master's motion not yet given to the gears (units << 24)

    // Timeline
    TimelineState m_tl = TimelineState::Idle;
    bool m_repeat = false;
    bool m_triggerRising = true;
    bool m_lastTrigger = false;
    bool m_triggerKnown = false;
    bool m_inputTrigger = false;
    int64_t m_timeQ16 = 0;
    int64_t m_cycleQ16 = 1;
    int32_t m_cycle = 0;
    int32_t m_rateQ16 = 0;
    volatile int32_t m_rateTargetQ16 = 65536;
    int32_t m_rateStepQ16 = 1;  // per sample while changing speed
    int32_t m_stopStepQ16 = 1;
    double m_stopSeconds = 0.5;

    // Timeline outputs
    struct Pulse {
        bool crank = false;
        int64_t on = 0, off = 0;  // crank: master units within the revolution; timeline: samples << 16
        uint8_t mask = 0;
    };
    Pulse m_pulses[kMaxOutputPulses];
    int m_pulseCount = 0;
    uint8_t m_outputs = 0;
    uint8_t m_outputsUsed = 0;

    // Jog
    volatile int32_t m_jogCrankTarget = 0;
    volatile int32_t m_jogTimeTargetQ4 = 0;
    bool m_jogEnding = false;
    int64_t m_jogCrankEndQ24 = 0;   // where the crank comes to rest when jog ends
    int64_t m_jogTimeEndQ16 = 0;
    int64_t m_jogVmaxQ24 = 1, m_jogAccQ24 = 1;
    int64_t m_jogRateMaxQ16 = 1, m_jogRateAccQ16 = 1;

    // Requests
    volatile uint32_t m_startReq = 0, m_stopReq = 0, m_abortReq = 0, m_triggerReq = 0, m_cycleStopReq = 0;
    uint32_t m_startSeen = 0, m_stopSeen = 0, m_abortSeen = 0, m_triggerSeen = 0, m_cycleStopSeen = 0;
    bool m_endOfCycle = false;  // the timeline finishes this cycle, then rests
    // Limits
    int64_t m_limitSamples = 500;     // the limit stop time, in samples
    int64_t m_crankStopStepQ24 = 0;   // a limit stop's crank deceleration (0: the crank ramp)
    bool m_limitStopping = false;
    uint32_t m_limitEvents = 0;
    int8_t m_limitAxis = -1, m_limitSide = 0;
    // Timeline waits
    struct Wait {
        int64_t tQ16 = 0;          // samples << 16 into the cycle
        uint32_t timeout = 0;      // samples, 0 none
        uint32_t holds = 0, timeouts = 0;
        bool edge = false;         // needs a change
        bool armed = false;        // the other level seen since it last passed (since Start)
    };
    uint16_t WaitReady(uint16_t met) const;  // met, and armed where a change is needed
    void WaitPassed(int k);
    Wait m_waits[kMaxWaits];
    int m_waitCount = 0;
    int m_waitNext = 0;            // the first wait of this cycle not passed yet
    int m_waitCur = -1;            // the one holding (Waiting)
    uint32_t m_waited = 0;
    uint32_t m_waitEvents = 0;
    int m_waitEventIndex = -1;
    volatile uint32_t m_waitSkipReq = 0;
    uint32_t m_waitSkipSeen = 0;
    // Moves the timeline from t0 to t1 (samples << 16), stopping on a wait whose condition doesn't hold: true when it
    // stopped there (the time is the wait's).
    bool WaitCrossing(int64_t t0, int64_t *t1, uint16_t met);
    // Homing
    uint8_t m_homed = 0;
    uint8_t m_homeAll = 0;  // Home all: the axes still to start
    uint32_t m_homeEvents = 0;
    int8_t m_homeAxis = -1;
    HomeFail m_homeFail = HomeFail::None;
    volatile uint32_t m_jogOnReq = 0, m_jogOffReq = 0;
    uint32_t m_jogOnSeen = 0, m_jogOffSeen = 0;
    uint32_t m_ticks = 0;
};

}  // namespace ecam
