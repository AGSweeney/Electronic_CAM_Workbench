/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "CamPoints.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace ecam {

namespace {

std::vector<std::string> SplitColumns(std::string line) {
    const bool decimalComma = line.find(';') != std::string::npos || line.find('\t') != std::string::npos;
    for (char &c : line) {
        if (c == ';' || c == '\t') {
            c = ' ';
        } else if (c == ',') {
            c = decimalComma ? '.' : ' ';
        }
    }
    std::vector<std::string> cols;
    std::istringstream in(line);
    for (std::string c; in >> c;) cols.push_back(c);
    return cols;
}

bool Number(const std::string &s, double *out) {
    char *end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0' || !std::isfinite(v)) return false;
    *out = v;
    return true;
}

std::string Lower(std::string s) {
    for (char &c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string LineError(int line, const std::string &what) {
    return "Line " + std::to_string(line) + ": " + what;
}

// The axis with Spline segments through the chosen points.
void SetKnots(CamAxis &axis, const std::vector<CurvePoint> &p, const std::vector<int> &keep) {
    axis.startPos = p[keep.front()].y;
    axis.segments.clear();
    for (size_t i = 1; i < keep.size(); ++i) axis.segments.push_back({p[keep[i]].x, p[keep[i]].y, MotionLaw::Spline});
}

// Largest distance from the curve to any point; its index in *worst.
double CurveError(const CamAxis &axis, const std::vector<CurvePoint> &p, int *worst) {
    const SplineFit fit = FitSplines(axis);
    const int last = static_cast<int>(axis.segments.size()) - 1;
    double err = 0.0;
    *worst = -1;
    for (int i = 0; i < static_cast<int>(p.size()); ++i) {
        const double y = p[i].x >= axis.span ? EvaluateSegment(axis, last, 1.0, &fit).pos : Evaluate(axis, p[i].x, &fit).pos;
        const double e = std::fabs(y - p[i].y);
        if (e > err) {
            err = e;
            *worst = i;
        }
    }
    return err;
}

}  // namespace

bool ParsePoints(const std::string &text, std::vector<CurvePoint> *points, std::string *error) {
    points->clear();
    std::istringstream in(text);
    std::string line;
    int lineNo = 0, yCol = 1;
    bool heading = false;
    while (std::getline(in, line)) {
        ++lineNo;
        if (lineNo == 1 && line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);  // a byte order mark
        const size_t first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || line[first] == '#') continue;
        const std::vector<std::string> cols = SplitColumns(line);
        double x = 0.0, y = 0.0;
        if (!Number(cols[0], &x)) {
            if (!points->empty() || heading) {
                *error = LineError(lineNo, "\"" + cols[0] + "\" is not a number.");
                return false;
            }
            heading = true;
            for (size_t c = 1; c < cols.size(); ++c) {
                if (Lower(cols[c]).compare(0, 3, "pos") == 0) {
                    yCol = static_cast<int>(c);
                    break;
                }
            }
            continue;
        }
        if (static_cast<int>(cols.size()) <= yCol || !Number(cols[yCol], &y)) {
            *error = LineError(lineNo, "needs a number in column " + std::to_string(yCol + 1) + ".");
            return false;
        }
        if (!points->empty()) {
            const CurvePoint &prev = points->back();
            if (x < prev.x) {
                *error = LineError(lineNo, "the first column must increase.");
                return false;
            }
            if (x == prev.x) {
                if (y != prev.y) {
                    *error = LineError(lineNo, "two positions at the same master value.");
                    return false;
                }
                continue;
            }
        }
        points->push_back({x, y});
    }
    if (points->size() < 2) {
        *error = "Need at least two points.";
        return false;
    }
    return true;
}

bool FitAxisToPoints(CamAxis &axis, const std::vector<CurvePoint> &points, int maxSegments, PointsFit *info,
                     std::string *error) {
    if (points.size() < 2 || points.back().x <= points.front().x) {
        *error = "Need at least two points with different master values.";
        return false;
    }
    const double x0 = points.front().x, x1 = points.back().x;
    CamAxis out = axis;
    if (out.master == AxisMaster::Crank) {
        out.span = 360.0;
    } else {
        out.span = x1 - x0;
    }
    // Onto the axis, dropping points too close to the last one kept (the end point stays).
    const double scale = out.span / (x1 - x0), minSeg = MinSegment(out);
    std::vector<CurvePoint> p;
    for (const CurvePoint &q : points) {
        const double x = (q.x - x0) * scale;
        if (p.empty() || x - p.back().x >= minSeg) p.push_back({x, q.y});
    }
    const CurvePoint end{out.span, points.back().y};
    if (p.size() >= 2 && end.x - p.back().x < minSeg) p.pop_back();
    if (p.back().x != end.x) p.push_back(end);
    if (p.size() < 2) {
        *error = "The points are too close together.";
        return false;
    }
    double lo = p[0].y, hi = p[0].y;
    for (const CurvePoint &q : p) {
        lo = std::min(lo, q.y);
        hi = std::max(hi, q.y);
    }
    out.closed = std::fabs(p.back().y - p.front().y) <= 1e-9 * std::max(hi - lo, 1e-12);

    const int count = static_cast<int>(p.size());
    std::vector<int> keep;
    if (count <= maxSegments + 1) {
        for (int i = 0; i < count; ++i) keep.push_back(i);
        SetKnots(out, p, keep);
    } else {
        keep = {0, count - 1};
        SetKnots(out, p, keep);
        while (static_cast<int>(keep.size()) < maxSegments + 1) {
            int worst = -1;
            CurveError(out, p, &worst);
            if (worst < 0 || std::binary_search(keep.begin(), keep.end(), worst)) break;
            keep.insert(std::lower_bound(keep.begin(), keep.end(), worst), worst);
            SetKnots(out, p, keep);
        }
    }
    Normalize(out);
    int worst = -1;
    const double err = CurveError(out, p, &worst);
    axis = out;
    if (info) {
        info->points = static_cast<int>(points.size());
        info->knots = static_cast<int>(keep.size());
        info->firstX = x0;
        info->lastX = x1;
        info->maxError = err;
        info->closed = out.closed;
    }
    return true;
}

}  // namespace ecam
