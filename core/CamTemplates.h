/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Starting points for a new project: a few common machine motions, each built from a handful of plain
// numbers (stroke, where in the cycle it moves, the motor and its mechanics). The setup wizard fills a
// TemplateSpec page by page and calls BuildTemplate; the result is an ordinary project to edit further.
// Portable C++17 (no Qt).

#include "CamProfile.h"

#include <string>

namespace ecam {

enum class TemplateKind : int {
    RiseDwellReturn = 0,  // crank: move out, wait, come back, wait (a pusher, a cam follower)
    IndexFeed,            // crank: advance a fixed distance every cycle (open profile: a conveyor, a roll feed)
    Oscillate,            // crank: continuous out and back with no stops (a traverse)
    TriggeredMove,        // timeline: out, hold, back once per trigger (pick and place, a gate)
    FlyingCutoff,         // encoder on the line, cut-length mode: carriage matches the line, saw cuts, returns
    Count
};

const char *TemplateName(TemplateKind kind);     // "Rise, dwell, return"
const char *TemplateSummary(TemplateKind kind);  // one sentence for the picker
bool TemplateOnTimeline(TemplateKind kind);
bool TemplateUsesLine(TemplateKind kind);        // cut-length mode (encoder on the line)

struct TemplateSpec {
    TemplateKind kind = TemplateKind::RiseDwellReturn;

    // ---- Master (crank templates) ----
    MasterSource source = MasterSource::Encoder;
    double encoderPpr = 4000.0;    // encoder counts per encoder rev (x4), or crank motor steps per rev
    int gearNum = 1;               // encoder or crank motor revs : crank revs
    int gearDen = 1;
    double rpm = 30.0;             // expected (encoder) or run (driven crank) speed, crank rpm
    int crankConnector = 0;        // driven crank motor M-n
    double crankRampRpmPerSec = 60.0;
    double crankMaxRpm = 300.0;

    // ---- Timeline ----
    TriggerSource trigger = TriggerSource::DigitalInput;
    int triggerInput = 0;
    bool triggerRising = true;
    bool repeat = false;

    // ---- Line (flying cut-off): the encoder wheel rides the material ----
    double wheelCircumference = 200.0;  // mm of line per encoder rev
    double cutLength = 600.0;           // mm per piece, as drawn
    double kerf = 0.0;                  // mm
    double lineSpeed = 5.0;             // m/min, for the analysis (sets rpm = cuts per minute)

    // ---- The follower axis ----
    std::string name = "Axis 1";
    std::string units = "mm";
    int connector = 0;
    AxisDrive drive = AxisDrive::Ballscrew;
    double motorPpr = 800.0;
    double screwPitch = 10.0;
    double pulleyPd = 25.0;
    double countsPerUnit = 80.0;   // Direct drive only
    double velMax = 0.0;           // 0 = not set
    double accMax = 0.0;

    // ---- The motion ----
    MotionLaw law = MotionLaw::Cycloidal;
    double stroke = 50.0;  // units: out-and-back distance, feed per cycle or move distance
    // Crank templates, degrees: RiseDwellReturn uses all four (rise a0..a1, return a2..a3); IndexFeed
    // feeds a0..a1; Oscillate goes out 0..a1 and back a1..360.
    double a0 = 0.0, a1 = 120.0, a2 = 180.0, a3 = 300.0;
    // TriggeredMove, seconds: wait, out, hold, back.
    double delay = 0.0, moveTime = 0.5, holdTime = 0.3, returnTime = 0.5;
    // FlyingCutoff, degrees of the cycle: ramp up to line speed, ride the line, ramp down, return.
    double rampDeg = 36.0, syncDeg = 36.0, returnDeg = 78.0;
    bool saw = true;               // a second axis plunges the saw during the sync
    double sawStroke = 6.0;        // saw units (plunge depth)
    bool sawOutput = true;         // IO-0 on during the sync (saw motor / clamp)
    // The saw plunge motor: its own connector, units, mechanics and limits (as the follower's above).
    std::string sawName = "Saw plunge";
    std::string sawUnits = "mm";
    int sawConnector = 1;
    AxisDrive sawDrive = AxisDrive::Ballscrew;
    double sawMotorPpr = 800.0;
    double sawPitch = 5.0;         // travel per rev (ballscrew)
    double sawPulleyPd = 20.0;     // belt
    double sawCountsPerUnit = 160.0;  // Direct only
    double sawVelMax = 0.0;
    double sawAccMax = 0.0;
};

// Sensible values for the template (the wizard's starting point).
TemplateSpec DefaultTemplate(TemplateKind kind);

// The project the spec describes, normalized. Out-of-order or overlapping phases are clamped into order.
CamProject BuildTemplate(const TemplateSpec &spec);

// Flying cut-off: the carriage stroke the spec needs (ramp up + sync + ramp down at line speed), in mm,
// and the degrees left for the wait at the start of the cycle (negative: the phases do not fit).
double FlyingCarriageStroke(const TemplateSpec &spec);
double FlyingWaitDeg(const TemplateSpec &spec);

}  // namespace ecam
