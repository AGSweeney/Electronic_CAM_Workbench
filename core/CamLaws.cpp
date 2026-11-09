/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "CamLaws.h"

#include <cmath>
#include <cstring>

namespace ecam {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Laws built from sinusoidal acceleration pieces: a(x) = c + k*sin(w*(x - xa) + phi) on [xa, xb].
// v and s are integrated in closed form piece by piece, then the whole law is scaled so s(1) = 1.
struct Piece {
    double xa, xb, c, k, w, phi;
    double va, sa;  // velocity and displacement at xa (filled in by Build)
};

struct PieceLaw {
    Piece p[5];
    int n = 0;
    double scale = 1.0;
};

LawValue EvalPiece(const Piece &pc, double x) {
    const double t = x - pc.xa;
    LawValue r;
    if (pc.w == 0.0) {
        r.j = 0.0;
        r.a = pc.c + pc.k;  // k is unused when w == 0; treated as part of the constant
        r.v = pc.va + r.a * t;
        r.s = pc.sa + pc.va * t + 0.5 * r.a * t * t;
        return r;
    }
    const double ang = pc.w * t + pc.phi;
    const double kw = pc.k / pc.w;
    r.j = pc.k * pc.w * std::cos(ang);
    r.a = pc.c + pc.k * std::sin(ang);
    r.v = pc.va + pc.c * t + kw * (std::cos(pc.phi) - std::cos(ang));
    r.s = pc.sa + pc.va * t + 0.5 * pc.c * t * t + kw * t * std::cos(pc.phi) -
          (kw / pc.w) * (std::sin(ang) - std::sin(pc.phi));
    return r;
}

void Build(PieceLaw &law) {
    double v = 0.0, s = 0.0;
    for (int i = 0; i < law.n; ++i) {
        law.p[i].va = v;
        law.p[i].sa = s;
        const LawValue end = EvalPiece(law.p[i], law.p[i].xb);
        v = end.v;
        s = end.s;
    }
    law.scale = (s != 0.0) ? 1.0 / s : 1.0;
}

LawValue EvalPieceLaw(const PieceLaw &law, double u) {
    int i = 0;
    while (i < law.n - 1 && u > law.p[i].xb) {
        ++i;
    }
    LawValue r = EvalPiece(law.p[i], u);
    r.s *= law.scale;
    r.v *= law.scale;
    r.a *= law.scale;
    r.j *= law.scale;
    return r;
}

const PieceLaw &HarmonicLaw() {
    static const PieceLaw law = [] {
        PieceLaw l;
        l.p[0] = {0.0, 1.0, 0.0, 1.0, kPi, kPi / 2, 0, 0};  // a = cos(pi x)
        l.n = 1;
        Build(l);
        return l;
    }();
    return law;
}

const PieceLaw &CycloidalLaw() {
    static const PieceLaw law = [] {
        PieceLaw l;
        l.p[0] = {0.0, 1.0, 0.0, 1.0, 2 * kPi, 0.0, 0, 0};  // a = sin(2 pi x)
        l.n = 1;
        Build(l);
        return l;
    }();
    return law;
}

// Modified trapezoid: sine ramp up over 1/8, constant to 3/8, half cosine through zero to 5/8,
// constant negative to 7/8, sine ramp back to zero at 1.
const PieceLaw &ModTrapLaw() {
    static const PieceLaw law = [] {
        PieceLaw l;
        const double w = 4 * kPi;
        l.p[0] = {0.0, 0.125, 0.0, 1.0, w, 0.0, 0, 0};
        l.p[1] = {0.125, 0.375, 1.0, 0.0, 0.0, 0.0, 0, 0};
        l.p[2] = {0.375, 0.625, 0.0, 1.0, w, kPi / 2, 0, 0};
        l.p[3] = {0.625, 0.875, -1.0, 0.0, 0.0, 0.0, 0, 0};
        l.p[4] = {0.875, 1.0, 0.0, -1.0, w, kPi / 2, 0, 0};
        l.n = 5;
        Build(l);
        return l;
    }();
    return law;
}

// Modified sine: quarter sine (period 1/2) to 1/8, a long cosine (period 3/2) through zero to
// 7/8, quarter sine back to zero at 1.
const PieceLaw &ModSineLaw() {
    static const PieceLaw law = [] {
        PieceLaw l;
        l.p[0] = {0.0, 0.125, 0.0, 1.0, 4 * kPi, 0.0, 0, 0};
        l.p[1] = {0.125, 0.875, 0.0, 1.0, 4 * kPi / 3, kPi / 2, 0, 0};
        l.p[2] = {0.875, 1.0, 0.0, -1.0, 4 * kPi, kPi / 2, 0, 0};
        l.n = 3;
        Build(l);
        return l;
    }();
    return law;
}

struct LawInfo {
    MotionLaw law;
    const char *name;
    const char *key;
    bool atRest;
};

constexpr LawInfo kLaws[] = {
    {MotionLaw::Dwell, "Dwell", "dwell", true},
    {MotionLaw::Linear, "Linear", "linear", false},
    {MotionLaw::Harmonic, "Simple harmonic", "harmonic", true},
    {MotionLaw::Cycloidal, "Cycloidal", "cycloidal", true},
    {MotionLaw::ModifiedTrapezoid, "Modified trapezoid", "modtrap", true},
    {MotionLaw::ModifiedSine, "Modified sine", "modsine", true},
    {MotionLaw::Poly345, "Polynomial 3-4-5", "poly345", true},
    {MotionLaw::Poly4567, "Polynomial 4-5-6-7", "poly4567", true},
    {MotionLaw::Blend, "Blend (match neighbours)", "blend", false},
    {MotionLaw::Spline, "Cubic spline", "spline", false},
};

}  // namespace

LawValue EvaluateLaw(MotionLaw law, double u) {
    if (u < 0.0) u = 0.0;
    if (u > 1.0) u = 1.0;
    LawValue r;
    switch (law) {
        case MotionLaw::Dwell:
            break;
        case MotionLaw::Linear:
            r.s = u;
            r.v = 1.0;
            break;
        case MotionLaw::Harmonic:
            r = EvalPieceLaw(HarmonicLaw(), u);
            break;
        case MotionLaw::Cycloidal:
            r = EvalPieceLaw(CycloidalLaw(), u);
            break;
        case MotionLaw::ModifiedTrapezoid:
            r = EvalPieceLaw(ModTrapLaw(), u);
            break;
        case MotionLaw::ModifiedSine:
            r = EvalPieceLaw(ModSineLaw(), u);
            break;
        case MotionLaw::Blend:  // rest to rest; with moving neighbours see EvaluateSegment
        case MotionLaw::Poly345: {
            const double u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
            r.s = 10 * u3 - 15 * u4 + 6 * u5;
            r.v = 30 * u2 - 60 * u3 + 30 * u4;
            r.a = 60 * u - 180 * u2 + 120 * u3;
            r.j = 60 - 360 * u + 360 * u2;
            break;
        }
        case MotionLaw::Poly4567: {
            const double u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u, u6 = u5 * u, u7 = u6 * u;
            r.s = 35 * u4 - 84 * u5 + 70 * u6 - 20 * u7;
            r.v = 140 * u3 - 420 * u4 + 420 * u5 - 140 * u6;
            r.a = 420 * u2 - 1680 * u3 + 2100 * u4 - 840 * u5;
            r.j = 840 * u - 5040 * u2 + 8400 * u3 - 4200 * u4;
            break;
        }
        case MotionLaw::Spline:  // rest to rest; through more knots see EvaluateSegment
            r.s = (3 - 2 * u) * u * u;
            r.v = 6 * u * (1 - u);
            r.a = 6 - 12 * u;
            r.j = -12;
            break;
        case MotionLaw::Count:
            break;
    }
    return r;
}

const char *LawName(MotionLaw law) {
    for (const auto &l : kLaws) {
        if (l.law == law) return l.name;
    }
    return "?";
}

const char *LawKey(MotionLaw law) {
    for (const auto &l : kLaws) {
        if (l.law == law) return l.key;
    }
    return "dwell";
}

bool LawFromKey(const char *key, MotionLaw *out) {
    for (const auto &l : kLaws) {
        if (std::strcmp(l.key, key) == 0) {
            *out = l.law;
            return true;
        }
    }
    return false;
}

bool LawStartsAndEndsAtRest(MotionLaw law) {
    for (const auto &l : kLaws) {
        if (l.law == law) return l.atRest;
    }
    return false;
}

}  // namespace ecam
