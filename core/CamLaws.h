/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Normalized cam motion laws. Portable C++14 with no Qt and no heap use; the same code builds into
// the app and the ClearCore firmware.
//
// Every law maps u in [0,1] (fraction of the segment's master span) to s in [0,1] (fraction of
// the segment's rise). v, a and j are the 1st..3rd derivatives of s with respect to u.

namespace ecam {

enum class MotionLaw : int {
    Dwell = 0,          // no motion
    Linear,             // constant velocity (velocity steps at the ends)
    Harmonic,           // simple harmonic (acceleration steps at the ends)
    Cycloidal,          // zero acceleration at both ends
    ModifiedTrapezoid,  // low peak acceleration, finite jerk
    ModifiedSine,       // low peak velocity, finite jerk
    Poly345,            // 3-4-5 polynomial
    Poly4567,           // 4-5-6-7 polynomial (zero jerk at the ends)
    Blend,              // starts and ends at its neighbours' velocities, zero acceleration at both ends
                        // (quintic Hermite, see EvaluateSegment); between two rests it is Poly345
    Spline,             // cubic spline: a run of Spline segments is one C2 curve through their knots
                        // (see SplineFit); alone between two rests it is 3u^2 - 2u^3
    Count
};

struct LawValue {
    double s = 0.0;
    double v = 0.0;
    double a = 0.0;
    double j = 0.0;
};

// Blend and Spline depend on their neighbours, so on their own they give the rest-to-rest case; segments
// are evaluated with their neighbours by ecam::EvaluateSegment().
LawValue EvaluateLaw(MotionLaw law, double u);

// Display name ("Cycloidal") and stable file key ("cycloidal").
const char *LawName(MotionLaw law);
const char *LawKey(MotionLaw law);
bool LawFromKey(const char *key, MotionLaw *out);

// True when the law has zero velocity at both ends (it can join a dwell without a velocity step).
bool LawStartsAndEndsAtRest(MotionLaw law);

}  // namespace ecam
