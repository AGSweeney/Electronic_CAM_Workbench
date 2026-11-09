/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// A list of points (master, position) made into a Cubic spline axis. Portable C++17 with no Qt; the app
// only, the firmware receives the segments.

#include "CamProfile.h"

#include <string>
#include <vector>

namespace ecam {

struct CurvePoint {
    double x = 0.0;
    double y = 0.0;
};

// Reads points from text: one per line, comma, semicolon, tab or space between the columns, with a
// decimal comma when a semicolon or tab separates them. Blank lines and lines starting with # are skipped.
// The first column is x. y is the second column, or with a heading line the first column whose heading
// starts with "pos" (so a table from File > Export reads back). x must increase; a repeated x with the
// same y is dropped. False with the reason in error.
bool ParsePoints(const std::string &text, std::vector<CurvePoint> *points, std::string *error);

struct PointsFit {
    int points = 0;         // read
    int knots = 0;          // used
    double firstX = 0.0;    // the x range as given
    double lastX = 0.0;
    double maxError = 0.0;  // largest distance from the curve to a point, in position units
    bool closed = false;    // it ends where it started
};

// Replaces the axis's segments with Spline segments through the points. The x range maps onto the axis:
// 0..360 deg on the crank; on the timeline the first x is 0 and the last sets the duration. Points closer
// than MinSegment are merged. With more than maxSegments + 1 left, the knots are chosen by adding, one at a
// time, the point furthest from the curve so far. closed is set from the data. False with the reason.
bool FitAxisToPoints(CamAxis &axis, const std::vector<CurvePoint> &points, int maxSegments, PointsFit *info,
                     std::string *error);

}  // namespace ecam
