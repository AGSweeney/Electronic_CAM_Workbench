/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Electronic cam profile model. Portable C++17 (no Qt).
//
// Each axis (follower motor) holds a displacement diagram over a master coordinate x made of
// segments. Each axis picks its master, and both kinds can run at once on one board:
//   - the crank (external encoder, or a crank motor we drive): x is the crank angle, 0..360 deg,
//     and the diagram repeats every revolution (cyclic);
//   - the timeline: x is time in seconds from a t = 0 shared by every timeline axis, scaled by a
//     speed override. Each axis has its own duration and rests at its end position after it. The
//     timeline runs once per trigger, or repeats with a cycle as long as its longest axis.
// Names below say "Deg" for the master coordinate; on a timeline read seconds.
//
// Knots are the segment boundaries: knot 0 is at x = 0 (position = startPos), knot k (k >= 1) is
// the end of segment k-1 (segments[k-1].endDeg / .endPos). The last knot is at x = span.
//
// Constraints (kept by the edit functions below and by Normalize()):
//   - a Dwell segment's end position equals its start position;
//   - a closed profile ends at its start position (knot N == knot 0);
//   - knot positions strictly increase, by at least MinSegment(axis).

#include "CamLaws.h"

#include <string>
#include <vector>

namespace ecam {

constexpr double kMinSegmentDeg = 0.5;  // on a 360 deg crank; scaled with the span otherwise

struct CamSegment {
    double endDeg = 360.0;  // master angle where the segment ends
    double endPos = 0.0;    // slave position at endDeg (user units)
    MotionLaw law = MotionLaw::Cycloidal;
};

constexpr int kMotorConnectors = 4;  // ClearCore M-0 .. M-3

enum class AxisMaster : int { Crank = 0, Timeline };

// How the follower motor moves the load; sets countsPerUnit (NormalizeProject).
enum class AxisDrive : int {
    Direct = 0,  // countsPerUnit entered as is
    Ballscrew,   // countsPerUnit = motorPpr / screwPitch      (pitch: units of travel per motor rev)
    Belt,        // countsPerUnit = motorPpr / (pi * pulleyPd)  (pulley pitch diameter, in units)
};

// Homing (needs a home switch: an input with the Home job on the axis). Positions and speeds in the axis's units. The
// sequence: off the switch if it starts on it, a search toward it at fast, back off at fast until it releases and back
// further, toward it again at slow, where the switch's edge becomes the axis's position at.
struct AxisHome {
    bool required = false;   // Start waits until the axis has homed since it was last enabled
    int dir = -1;            // the switch is toward - (-1) or + (+1)
    double fast = 0.0;       // search speed, units/s (0: set by NormalizeProject, 10 % of the axis's jog limit)
    double slow = 0.0;       // latch speed, units/s (0: fast / 10)
    double back = 0.0;       // back-off past the switch's release before the slow move, units (0: fast x 0.25 s)
    double at = 0.0;         // the axis's position at the switch's edge, from the profile's start
    bool thenStart = true;   // then move to the cam's start position (else stay at the switch)
    int order = 1;           // Home all: lower numbers first; the same number homes together (1..9)
    double search = 0.0;     // the longest search, units; 0 = until the switch or a limit
    // How hard it stops on reaching the switch (the search and the latch), units/s^2; 0 = the axis's Acc max, or the
    // search speed in a quarter second (as axis jog). The run-on past the switch is v^2 / (2 decel).
    double decel = 0.0;
};
constexpr int kHomeOrderMax = 9;

struct CamAxis {
    std::string name = "Axis 1";
    AxisMaster master = AxisMaster::Crank;
    std::string units = "mm";
    int connector = 1;             // motor connector M-n
    double countsPerUnit = 800.0;  // motor steps per user unit (derived unless drive is Direct)
    AxisDrive drive = AxisDrive::Direct;
    double motorPpr = 800.0;       // motor steps per motor revolution (Ballscrew, Belt)
    double screwPitch = 5.0;       // travel per screw revolution, in units (Ballscrew)
    double pulleyPd = 20.0;        // pulley pitch diameter, in units (Belt)
    double velMax = 0.0;           // follower limits in units/s and units/s^2; 0 = not set
    double accMax = 0.0;
    double startPos = 0.0;         // slave position at 0 deg
    bool closed = true;            // ends where it started (every revolution, on a crank)
    // The motor turns the other way. Only its steps change: the diagram and positions stay as drawn.
    bool reverse = false;
    double span = 360.0;           // master length: 360 deg on the crank, the duration (s) on the timeline
    // Derived by NormalizeProject: true when the diagram wraps from its end straight into its start
    // (crank, or a repeating timeline axis as long as the cycle); otherwise it rests before 0 and
    // after span.
    bool cyclic = true;
    // Cam drawing only (translating radial roller follower), in user units.
    double baseRadius = 40.0;
    double rollerRadius = 8.0;
    // Electronic gear: no diagram of its own, the axis travels gearNum units for every gearDen crank
    // revolutions (negative reverses it; in cut-length mode, per cycle as drawn). It always follows the
    // crank. Normalize() keeps its segments as one open Linear segment of gearNum / gearDen units, so the
    // diagrams and the analysis see a constant ratio; the runtime keeps the ratio exact.
    bool gear = false;
    int gearNum = 1;
    int gearDen = 1;
    AxisHome home;
    std::vector<CamSegment> segments;
};

// Where the master (crank) position comes from. One firmware, chosen at run time.
enum class MasterSource : int {
    Encoder = 0,  // external shaft encoder on the Encoder Adapter; followers track it
    DrivenCrank,  // the firmware drives the crank motor; followers track its commanded position
};

// What starts the timeline.
enum class TriggerSource : int {
    Command = 0,   // a host / console command
    DigitalInput,  // an edge on a ClearCore input
    EtherNetIP,    // a bit in an EtherNet/IP output assembly (future)
};
constexpr int kTriggerInputs = 13;  // IO-0..IO-5, DI-6..DI-8, A-9..A-12
// The Encoder Adapter uses DI-6..DI-8 (A, B, index): no trigger there while an encoder is the master.
constexpr int kEncoderInputFirst = 6;
constexpr int kEncoderInputLast = 8;
// The estop input is a board setting (estop_pin): A-9, trigger index 9, unless set otherwise.
constexpr int kEstopInput = 9;

// Outputs: IO-0..IO-5 switched on and off by pulses, for the whole project (not per axis). Each pulse
// picks its master (see AxisMaster below):
//   Crank     on from `on` to `off` crank degrees (0..360), every revolution, like a cam switch; on
//             while the crank is engaged (following, running, stopping, hand jog).
//   Timeline  on from `on` to `off` seconds of timeline time, following the speed override, repeat and
//             hand jog; on while the timeline runs, stops or jogs.
// Every output is off otherwise (idle, armed, done, stopped, aborted, faulted).
constexpr int kOutputs = 6;                 // IO-0..IO-5
constexpr int kMaxOutputPulses = 32;        // all outputs together
constexpr double kMinPulseSeconds = 0.001;  // 5 samples at 5 kHz
constexpr double kMinPulseDeg = 0.1;

struct OutputPulse {
    int output = 0;                            // IO-n
    AxisMaster master = AxisMaster::Timeline;  // crank: degrees, timeline: seconds
    double on = 0.0;
    double off = 0.1;                          // > on
};

struct CamMaster {
    MasterSource source = MasterSource::Encoder;
    // Master counts per crank revolution = crankPpr * crankGearNum / crankGearDen (NormalizeProject):
    // encoder counts (quadrature, x4) per encoder rev for Encoder, motor steps per motor rev for
    // DrivenCrank; the gear ratio is encoder / motor revolutions per crank revolution as a fraction
    // (50 : 7 for a gearbox that is not a whole ratio; 10 : 1 is a 10x reduction). The firmware keeps
    // it exact: it counts in 1/den of a count, crankPpr * crankGearNum per crank revolution.
    double countsPerRev = 4000.0;  // derived, may be fractional
    double crankPpr = 4000.0;      // whole counts per motor / encoder revolution
    int crankGearNum = 1;
    int crankGearDen = 1;
    double rpm = 30.0;  // Encoder: expected speed (analysis, preview). DrivenCrank: run speed.
    // Cut-length mode, for a flying saw / cut-off: the encoder measures the line, or on a driven crank
    // the crank motor feeds it, and one crank revolution (360 deg as drawn) is one cut of designLength mm. The cut length can be
    // changed while running: the cycle's opening wait (where every crank axis dwells, LineWaitDeg)
    // stretches or shrinks, the rest (accelerate, sync, decelerate, return) keeps its line distance.
    // The gear ratio is not used. rpm, and a driven crank's ramp and max, are then cuts per minute at
    // the design length: the line speed, whatever the cut length is now.
    // Encoder master: count the other way (the line or crank moving forward counted down otherwise).
    bool encoderReverse = false;
    bool crankReverse = false;  // driven crank: its motor turns the other way (see CamAxis::reverse)
    bool lineMode = false;
    double wheelCircumference = 200.0;  // mm of line per encoder / motor revolution (0: lineCountsPerMm given)
    double designLength = 360.0;        // mm, the cycle as drawn (line distance from cut to cut)
    // Material the cut removes (saw blade, laser kerf; 0 for a shear). A finished piece is the cycle
    // less the kerf, so a piece of length L needs a cycle of L + kerf.
    double kerf = 0.0;
    double lineCountsPerMm = 20.0;      // derived: crankPpr / wheelCircumference
    // DrivenCrank only
    int crankConnector = 0;           // motor connector M-n
    double crankRampRpmPerSec = 60.0; // acceleration and deceleration of the crank
    double crankMaxRpm = 600.0;       // crank motor limit, at the crank
    // Timeline
    bool timelineRepeat = false;      // false: one run per trigger
    double speedOverride = 1.0;       // timeline speed, 1 = as drawn
    TriggerSource trigger = TriggerSource::DigitalInput;
    int triggerInput = 0;             // index into IO-0..A-12
    bool triggerRising = true;
    int triggerBit = 0;               // EtherNet/IP assembly bit
    // Limits: a tripped limit brings the master to rest over this time (the controlled stop; an encoder master's axes
    // ramp themselves over it). Only sent to the board, and only saved, with a limit set up.
    double limitStopSeconds = 0.1;
};

// CCIO-8 expansion inputs on COM-1. For now the project only names them: the firmware doesn't act on them.
constexpr int kCcioMaxBoards = 8;
constexpr int kCcioPinsPerBoard = 8;
enum class CcioRole { None, Home, LimitPositive, LimitNegative };
struct CcioInput {
    int board = 0;  // 0..kCcioMaxBoards - 1
    int pin = 0;    // 0..kCcioPinsPerBoard - 1
    std::string name;
    CcioRole role = CcioRole::None;
    int axis = -1;  // the axis a role belongs to
    // The switch's contact: normally closed (tripped when the input is low; a cut wire reads as tripped) or normally
    // open (tripped when high; a cut wire reads as not tripped, so a normally open limit is warned about).
    bool normallyOpen = false;
};
struct CcioConfig {
    int boards = 0;                 // how many the machine should have
    std::vector<CcioInput> inputs;  // only the ones with a name or a role, sorted by board, pin
};
// The input as configured, a blank one if it isn't.
CcioInput CcioInputAt(const CcioConfig &config, int board, int pin);
// Stores it, or drops it if blank.
void SetCcioInput(CcioConfig &config, const CcioInput &input);
// After removing an axis: inputs on it lose it, later ones follow their axis.
void CcioAxisRemoved(CcioConfig &config, int axis);
// The connector's name, as in the library: CCIOA0 is the first board's first.
std::string CcioPinName(int board, int pin);

// The ClearCore's own inputs, by the trigger input's index: IO-0..IO-5, DI-6..DI-8, A-9..A-12. Like the CCIO-8 inputs,
// for homes and limits, and for now the project only names them. A pin can't be one while it is an output (an IO pin
// with a pulse), the timeline's trigger input, or the Encoder Adapter's (DI-6..DI-8 with an encoder master):
// BoardInputBlocked says why, ProjectReview flags a project that has both.
struct BoardInput {
    int pin = 0;
    std::string name;
    CcioRole role = CcioRole::None;
    int axis = -1;              // the axis a role belongs to
    bool normallyOpen = false;  // as CcioInput::normallyOpen
};

// A timeline wait: at its time the timeline holds (every timeline axis and output where it is) until the input reads
// the level it wants, then runs on from there. Only where every timeline axis is at rest (the review checks). A
// timeout (0: none, wait forever) faults and stops.
enum class WaitSource { Board = 0, Ccio, Plc };
constexpr int kMaxWaits = 16;
constexpr int kPlcWaitBits = 32;
struct TimelineWait {
    double t = 0.0;                         // s into the cycle, as drawn
    std::string name;                       // the project only
    WaitSource source = WaitSource::Board;
    int board = 0;                          // a CCIO-8's board (0..7)
    int pin = 0;                            // the board's input (0..12), the CCIO-8's pin (0..7), or the PLC bit (0..31)
    bool on = true;                         // the level it waits for
    double timeout = 0.0;                   // s; 0 = wait forever
    // Needs a change: it passes only if the input has been at the other level since this wait last passed (since
    // Start, the first time), so a stuck sensor holds it instead of letting every cycle through. Off: the level alone
    // (a steady condition). New waits have it on; a profile or project from 0.10.0 without it is the level alone.
    bool edge = true;
};

struct CamProject {
    CamMaster master;
    std::vector<CamAxis> axes;
    std::vector<OutputPulse> outputs;  // output pulses, sorted by output, master, on
    CcioConfig ccio;
    std::vector<BoardInput> inputs;    // only the ones with a name or a role, sorted by pin
    std::vector<TimelineWait> waits;   // sorted by time
};
// A wait reads this board input / CCIO-8 input (one job per pin: not with a home or limit too).
bool WaitOnPin(const CamProject &project, int pin);
bool WaitOnCcio(const CamProject &project, int board, int pin);
// The wait's input, by name: IO-2, CCIOA3, PLC bit 4.
std::string WaitInputName(const TimelineWait &wait);
// Every timeline axis is at rest at t (s into the cycle, as drawn): where a wait may go.
bool TimelineAtRest(const CamProject &project, double t);
// The nearest time to t where every timeline axis is at rest (within the cycle, 1 ms steps); -1 if there is none.
double NearestTimelineRest(const CamProject &project, double t);

// The input as configured, a blank one if it isn't.
BoardInput BoardInputAt(const CamProject &project, int pin);
// Stores it, or drops it if blank.
void SetBoardInput(CamProject &project, const BoardInput &input);
// After removing an axis: inputs on it lose it, later ones follow their axis.
void BoardInputsAxisRemoved(CamProject &project, int axis);
// Why the pin can't be an input in this project: "estop" (the board's estop input, estopPin; -1 none), "output",
// "trigger" or "encoder"; empty when it can.
std::string BoardInputBlocked(const CamProject &project, int pin, int estopPin = -1);
// An input is configured on DI-6..DI-8, the Encoder Adapter's: the crank can't use an encoder then.
bool BoardInputsOnEncoderPins(const CamProject &project);
// The pin's name: IO-0, DI-6, A-9...
std::string BoardInputName(int pin);
// The project has a limit (Limit + or Limit -) on some input, the board's or a CCIO-8's.
bool HasLimits(const CamProject &project);
// The axis has an input with the Home job, the board's or a CCIO-8's.
bool HasHomeSwitch(const CamProject &project, int axis);
// The axis's jog speed limit, units/s: its own limit, at most the board's 53333 counts/s.
double AxisJogLimit(const CamAxis &axis);
constexpr double kLimitStopMin = 0.01, kLimitStopMax = 2.0;  // s

// A rise / dwell / return / dwell profile, the usual starting point (on a 360 deg crank).
CamAxis DefaultAxis(const std::string &name);
CamProject DefaultProject();

// Stretches the diagram to a new span (segment ends scale, positions stay).
void ScaleSpan(CamAxis &axis, double span);
// Moves an axis to the other master, keeping its shape: crank -> timeline maps one revolution at
// the current crank speed onto seconds; timeline -> crank stretches its duration to 360 deg.
void SetAxisMaster(CamProject &project, int axis, AxisMaster master);
// The timeline cycle: the longest timeline axis or the last timeline output pulse's end, 0 when neither.
double TimelineLength(const CamProject &project);
// True with a crank axis or a crank output pulse.
bool UsesCrank(const CamProject &project);
// True with a timeline axis or a timeline output pulse.
bool UsesTimeline(const CamProject &project);
// The trigger input is one of the Encoder Adapter's inputs (DI-6..DI-8) while an encoder is the master.
bool TriggerOnEncoderInput(const CamMaster &master, bool usesCrank);
// Clamps each pulse (output index; crank pulses within 0..360 deg and at least kMinPulseDeg, timeline
// pulses on >= 0 and at least kMinPulseSeconds) and sorts them by output, master, on.
void NormalizeOutputs(std::vector<OutputPulse> &outputs);
// Sets every axis's span / cyclic flag for its master and normalizes it; derives countsPerUnit from
// each axis's drive and the master's countsPerRev from its ppr and gear ratio.
void NormalizeProject(CamProject &project);

// countsPerUnit from the drive (Direct: unchanged).
double DriveCountsPerUnit(const CamAxis &axis);
// The firmware's units per crank revolution, crankPpr * crankGearNum (each count is crankGearDen units).
// It must be whole and below kMaxCrankUnits.
double CrankUnitsPerRev(const CamMaster &master);
// Cut-length mode: the wait at the start of the cycle (deg as drawn) that stretches with the cut length:
// the end of the shortest run of Dwell segments at the start of every crank axis, and before every crank
// output pulse. 0 when some crank axis does not start with a dwell (the length cannot change).
double LineWaitDeg(const CamProject &project);
// Cut-length mode: the sync, where the carriage rides the line and the blade stays on the cut line: the
// first Linear segment of the first crank axis that has one (that axis is the carriage). Returns false
// when there is none; else the axis index and the window (deg as drawn).
bool LineSync(const CamProject &project, int *axis, double *startDeg, double *endDeg);
// The shortest finished piece (mm): the design length less the wait, less the kerf.
double LineMinLength(const CamProject &project);
constexpr double kMaxCrankUnits = 2.0e9;

double MinSegment(const CamAxis &axis);

// ---- Knots ----------------------------------------------------------------------------------
int KnotCount(const CamAxis &axis);  // segments + 1
double KnotDeg(const CamAxis &axis, int k);
double KnotPos(const CamAxis &axis, int k);
double SegmentStartDeg(const CamAxis &axis, int seg);
double SegmentStartPos(const CamAxis &axis, int seg);
int SegmentAt(const CamAxis &axis, double deg);  // deg wrapped (cyclic) or clamped into [0, span)

// ---- Edits (each keeps the constraints) -----------------------------------------------------
// Sets the knot and every knot tied to it (across dwells, and 0 <-> N when closed).
void SetKnotPos(CamAxis &axis, int k, double pos);
// Moves an interior knot; clamped between its neighbours. Knot 0 never moves; the last knot moves
// only on a timeline, where it sets the axis's duration.
void SetKnotDeg(CamAxis &axis, int k, double deg);
void SetSegmentLaw(CamAxis &axis, int seg, MotionLaw law);
void SetClosed(CamAxis &axis, bool closed);
// Splits the segment containing deg; both halves keep the law, the new knot sits on the curve.
// Returns the index of the new (second) segment, or -1 if too close to an existing knot.
int SplitAt(CamAxis &axis, double deg);
// Removes a segment; its span goes to the previous segment (the next one for segment 0).
void DeleteSegment(CamAxis &axis, int seg);
// Repairs a loaded or hand-edited axis so every constraint holds.
void Normalize(CamAxis &axis);

// ---- Evaluation -----------------------------------------------------------------------------
struct CamPoint {
    double pos = 0.0;   // user units
    double vel = 0.0;   // units / deg
    double acc = 0.0;   // units / deg^2
    double jerk = 0.0;  // units / deg^3
    int segment = 0;
};

// Cyclic: deg is wrapped into [0, span); for an open profile that is the position within one cycle,
// not accumulated over cycles. Timeline: before 0 / after span the axis rests at its start / end.
CamPoint Evaluate(const CamAxis &axis, double deg);
// One-sided values at a segment's start (u = 0) or end (u = 1).
//
// A run of consecutive Spline segments is one cubic spline through their knots, so a Spline segment depends
// on the whole run. SplineFit is an axis solved once: pass it when evaluating many points (nullptr solves
// on each call). It is valid until the axis changes.
struct SplineFit {
    // Second derivative (units / deg^2) at each segment's start and end; 0 for a segment that is no spline.
    std::vector<double> m0, m1;
};
SplineFit FitSplines(const CamAxis &axis);
CamPoint Evaluate(const CamAxis &axis, double deg, const SplineFit *fit);
CamPoint EvaluateSegment(const CamAxis &axis, int seg, double u, const SplineFit *fit = nullptr);

// Master angular speed in deg/s; per-degree derivatives times this (^n) give per-second values.
inline double DegPerSecond(const CamMaster &m) { return m.rpm * 6.0; }
// Master units per second for an axis: deg/s on the crank, the speed override on the timeline
// (x is seconds of the diagram). Per-x derivatives times this (^n) give per-second values.
inline double MasterRate(const CamMaster &m, const CamAxis &a) {
    return a.master == AxisMaster::Crank ? DegPerSecond(m) : m.speedOverride;
}
// Crank angular acceleration during a ramp (deg/s^2); 0 for an encoder master (not ours to ramp).
inline double RampDegPerSecond2(const CamMaster &m) {
    return m.source == MasterSource::DrivenCrank ? m.crankRampRpmPerSec * 6.0 : 0.0;
}
// Master ramp an axis sees: the crank ramp on the crank, none on the timeline.
inline double AxisRamp(const CamMaster &m, const CamAxis &a) {
    return a.master == AxisMaster::Crank ? RampDegPerSecond2(m) : 0.0;
}

// ---- Analysis -------------------------------------------------------------------------------
enum class IssueKind { VelocityStep, AccelStep, Undercut, PressureAngle, SplineOvershoot };

struct CamIssue {
    IssueKind kind;
    int knot = -1;      // for steps
    double deg = 0.0;
    double left = 0.0;  // value before / after the knot (per-degree units), or the measured value
    double right = 0.0; // SplineOvershoot: left is the curve's extreme, right the knot it passes
};

struct AxisStats {
    double minPos = 0.0, maxPos = 0.0;
    double maxVel = 0.0, maxAcc = 0.0, maxJerk = 0.0;  // |per-degree| peaks
    double maxVelDeg = 0.0, maxAccDeg = 0.0, maxJerkDeg = 0.0;
    double maxPressureDeg = 0.0, maxPressureAt = 0.0;       // cam drawing geometry
    double minCurvature = 0.0, minCurvatureAt = 0.0;        // smallest convex pitch radius
    double advancePerRev = 0.0;                             // end - start
    std::vector<CamIssue> issues;
};

AxisStats Analyze(const CamAxis &axis, int samples = 3600);

// Fastest master speed the axis's velocity / acceleration limits allow (deg/s on the crank; on
// the timeline it is the highest speed override). While the master ramps at
// alpha (deg/s^2) the follower sees s''(theta)*w^2 + s'(theta)*alpha, so a steep ramp lowers the limit
// and, past maxRampDegPerSec2, no speed is safe.
struct SpeedLimit {
    enum Reason { None, Velocity, Accel, Ramp };
    double maxDegPerSec = 0.0;          // +infinity when the axis has no limits set
    Reason reason = None;               // the binding limit
    double atDeg = 0.0;                 // where on the profile it binds
    double maxRampDegPerSec2 = 0.0;     // steepest ramp the accel limit allows (+inf if none)
};

SpeedLimit MaxMasterSpeed(const CamAxis &axis, double rampDegPerSec2, int samples = 3600);

// A spline passing the axis's knots by more than this fraction of their range is reported.
constexpr double kSplineOvershootLimit = 0.02;

// Pressure-angle warning threshold for a translating roller follower.
constexpr double kMaxPressureAngleDeg = 30.0;

}  // namespace ecam
