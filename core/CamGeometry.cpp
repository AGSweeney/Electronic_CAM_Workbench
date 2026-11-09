/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "CamGeometry.h"

#include <math.h>

namespace ecam {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegPerRad = 180.0 / kPi;
}  // namespace

void PitchPointAndNormal(const CamAxis &axis, double minPos, double thetaDeg, bool endOfCycle, Pt *point, Pt *normal) {
    const CamPoint c = endOfCycle ? EvaluateSegment(axis, static_cast<int>(axis.segments.size()) - 1, 1.0)
                                  : Evaluate(axis, thetaDeg);
    const double r = axis.baseRadius + axis.rollerRadius + (c.pos - minPos);
    const double dr = c.vel * kDegPerRad;  // dR/dphi, per radian
    const double phi = (90.0 + thetaDeg) / kDegPerRad;
    const double cp = cos(phi), sp = sin(phi);
    point->x = r * cp;
    point->y = r * sp;
    // Tangent of a polar curve r(phi), phi increasing (counter-clockwise in the cam frame):
    // T = (dr cos - r sin, dr sin + r cos). Outward normal = T rotated clockwise = (Ty, -Tx).
    const double tx = dr * cp - r * sp, ty = dr * sp + r * cp;
    const double len = sqrt(tx * tx + ty * ty);
    normal->x = len > 0 ? ty / len : cp;
    normal->y = len > 0 ? -tx / len : sp;
}

CamShape BuildCamShape(const CamAxis &axis, int samples) {
    CamShape s;
    if (samples < 8) samples = 8;
    s.baseRadius = axis.baseRadius;
    s.rollerRadius = axis.rollerRadius;
    s.closed = axis.closed;
    const AxisStats st = Analyze(axis, samples);
    s.minPos = st.minPos;
    s.stroke = st.maxPos - st.minPos;
    s.pitch.resize(samples + 1);
    s.surface.resize(samples + 1);
    s.segment.resize(samples + 1);
    for (int i = 0; i <= samples; ++i) {
        const double th = 360.0 * i / samples;
        Pt p, n;
        PitchPointAndNormal(axis, s.minPos, th, i == samples, &p, &n);
        s.pitch[i] = p;
        s.surface[i] = p;
        s.segment[i] = i == samples ? static_cast<int>(axis.segments.size()) - 1 : SegmentAt(axis, th);
    }

    // Cam surface = envelope of the roller: along each ray from the centre, the surface is where the
    // ray first enters any roller circle centred on the pitch curve (the cam is all the roller never
    // reaches). Where the profile is smooth this is the pitch curve offset by the roller radius; at a
    // velocity step (a corner in the pitch curve) it trims the overlap of an inside corner and rounds
    // an outside corner, and at the drop of an open profile it cuts the relief the roller rolls off by.
    const double rr = axis.rollerRadius;
    if (rr > 0.0) {
        const double rpMin = axis.baseRadius + rr;  // smallest pitch radius
        // A roller circle at radius >= rpMin spans at most asin(rr / rpMin) either side of its centre.
        const double halfSpan = asin(rr / rpMin < 1.0 ? rr / rpMin : 1.0);
        const int window = static_cast<int>(ceil(halfSpan / (2.0 * kPi / samples))) + 2;
        for (int i = 0; i <= samples; ++i) {
            const double phi = (90.0 + 360.0 * i / samples) / kDegPerRad;
            const double ux = cos(phi), uy = sin(phi);
            double best = 1e300;
            for (int k = -window; k <= window; ++k) {
                int j = i + k;
                // The pitch curve is periodic (an open profile's step is the jump at the seam).
                j = ((j % samples) + samples) % samples;
                const Pt &p = (k == 0) ? s.pitch[i] : s.pitch[j];
                const double along = p.x * ux + p.y * uy;          // P . u
                const double across = p.x * uy - p.y * ux;         // |P x u|
                const double h2 = rr * rr - across * across;
                if (along <= 0.0 || h2 < 0.0) continue;            // ray misses this roller
                const double t = along - sqrt(h2);                 // entry into the circle
                if (t < best) best = t;
            }
            if (best < 1e299) s.surface[i] = {best * ux, best * uy};
        }
    }
    for (int i = 0; i <= samples; ++i) {
        const double r = sqrt(s.surface[i].x * s.surface[i].x + s.surface[i].y * s.surface[i].y);
        if (r > s.maxRadius) s.maxRadius = r;
    }
    return s;
}

}  // namespace ecam
