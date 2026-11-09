/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Plate-cam geometry of a crank axis: a radial translating roller follower (knife edge when the
// roller radius is 0). Used by the cam view and the DXF export. Portable C++14.
//
// Cam frame: origin at the cam's centre, follower at 12 o'clock when the crank is at 0 deg, cam
// turning clockwise. The profile point for crank angle theta lies at angle 90 + theta degrees
// (counter-clockwise from +x), at pitch radius base + roller + (s - sMin). Units are the axis's.

#include "CamProfile.h"

#include <vector>

namespace ecam {

struct Pt {
    double x = 0.0, y = 0.0;
};

struct CamShape {
    std::vector<Pt> pitch;    // roller centre path, crank angle 0..360 inclusive
    std::vector<Pt> surface;  // cam outline: pitch offset inward by the roller radius
    std::vector<int> segment; // segment of each sample
    double minPos = 0.0;      // displacement at the base circle
    double stroke = 0.0;      // max - min displacement
    double baseRadius = 0.0;
    double rollerRadius = 0.0;
    double maxRadius = 0.0;   // largest surface radius
    bool closed = true;       // false: an open profile has a radial step at 0 deg
};

// samples points per revolution (plus the closing point).
CamShape BuildCamShape(const CamAxis &axis, int samples = 1440);

// Polar radius of the pitch curve and its outward unit normal at crank angle theta (cam frame).
void PitchPointAndNormal(const CamAxis &axis, double minPos, double thetaDeg, bool endOfCycle, Pt *point, Pt *normal);

}  // namespace ecam
