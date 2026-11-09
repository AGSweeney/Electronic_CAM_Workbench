/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "CamProfile.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ecam {

namespace {

constexpr double kPi = 3.14159265358979323846;

// std::clamp is C++17; the ClearCore toolchain (GCC 6) builds this file as C++14.
inline double Clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline int Clamp(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
constexpr double kDegPerRad = 180.0 / kPi;

double WrapDeg(const CamAxis &axis, double deg) {
    if (!axis.cyclic) return Clamp(deg, 0.0, axis.span);
    double d = std::fmod(deg, axis.span);
    if (d < 0.0) d += axis.span;
    return d;
}

void SetKnotRaw(CamAxis &axis, int k, double pos) {
    if (k == 0) {
        axis.startPos = pos;
    } else {
        axis.segments[k - 1].endPos = pos;
    }
}

// Union-find over the knots: a dwell ties its two knots, a closed profile ties knot 0 and N.
struct KnotGroups {
    std::vector<int> parent;
    explicit KnotGroups(const CamAxis &axis) {
        const int n = KnotCount(axis);
        parent.resize(n);
        for (int i = 0; i < n; ++i) parent[i] = i;
        for (int s = 0; s < static_cast<int>(axis.segments.size()); ++s) {
            if (axis.segments[s].law == MotionLaw::Dwell) Join(s, s + 1);
        }
        if (axis.closed) Join(0, n - 1);
    }
    int Find(int i) {
        while (parent[i] != i) {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        return i;
    }
    void Join(int a, int b) {
        a = Find(a);
        b = Find(b);
        if (a == b) return;
        // Keep the lowest index as the root so it owns the group's value.
        if (a < b) {
            parent[b] = a;
        } else {
            parent[a] = b;
        }
    }
};

}  // namespace

CamAxis DefaultAxis(const std::string &name) {
    CamAxis a;
    a.name = name;
    a.startPos = 0.0;
    a.segments = {
        {120.0, 20.0, MotionLaw::Cycloidal},  // rise
        {180.0, 20.0, MotionLaw::Dwell},      // high dwell
        {300.0, 0.0, MotionLaw::Cycloidal},   // return
        {360.0, 0.0, MotionLaw::Dwell},       // low dwell
    };
    return a;
}

CamProject DefaultProject() {
    CamProject p;
    p.axes.push_back(DefaultAxis("Axis 1"));
    return p;
}

int KnotCount(const CamAxis &axis) { return static_cast<int>(axis.segments.size()) + 1; }

double KnotDeg(const CamAxis &axis, int k) { return k <= 0 ? 0.0 : axis.segments[k - 1].endDeg; }

double KnotPos(const CamAxis &axis, int k) { return k <= 0 ? axis.startPos : axis.segments[k - 1].endPos; }

double SegmentStartDeg(const CamAxis &axis, int seg) { return KnotDeg(axis, seg); }

double SegmentStartPos(const CamAxis &axis, int seg) { return KnotPos(axis, seg); }

double MinSegment(const CamAxis &axis) { return kMinSegmentDeg * axis.span / 360.0; }

void ScaleSpan(CamAxis &axis, double span) {
    if (span <= 0.0 || axis.span <= 0.0) return;
    const double k = span / axis.span;
    for (CamSegment &s : axis.segments) s.endDeg *= k;
    axis.span = span;
}

double TimelineLength(const CamProject &project) {
    double len = 0.0;
    for (const CamAxis &a : project.axes) {
        if (a.master == AxisMaster::Timeline) len = std::max(len, a.span);
    }
    for (const OutputPulse &o : project.outputs) {
        if (o.master == AxisMaster::Timeline) len = std::max(len, o.off);
    }
    return len;
}

bool UsesCrank(const CamProject &project) {
    for (const CamAxis &a : project.axes) {
        if (a.master == AxisMaster::Crank) return true;
    }
    for (const OutputPulse &o : project.outputs) {
        if (o.master == AxisMaster::Crank) return true;
    }
    return false;
}

bool UsesTimeline(const CamProject &project) {
    for (const CamAxis &a : project.axes) {
        if (a.master == AxisMaster::Timeline) return true;
    }
    for (const OutputPulse &o : project.outputs) {
        if (o.master == AxisMaster::Timeline) return true;
    }
    return false;
}

bool TriggerOnEncoderInput(const CamMaster &m, bool usesCrank) {
    return usesCrank && m.source == MasterSource::Encoder && m.triggerInput >= kEncoderInputFirst &&
           m.triggerInput <= kEncoderInputLast;
}

void NormalizeOutputs(std::vector<OutputPulse> &outputs) {
    for (OutputPulse &o : outputs) {
        if (o.output < 0) o.output = 0;
        if (o.output >= kOutputs) o.output = kOutputs - 1;
        if (o.on < 0.0) o.on = 0.0;
        if (o.master == AxisMaster::Crank) {
            // Within one revolution.
            if (o.on > 360.0 - kMinPulseDeg) o.on = 360.0 - kMinPulseDeg;
            if (o.off > 360.0) o.off = 360.0;
            if (o.off < o.on + kMinPulseDeg) o.off = o.on + kMinPulseDeg;
        } else if (o.off < o.on + kMinPulseSeconds) {
            o.off = o.on + kMinPulseSeconds;
        }
    }
    std::stable_sort(outputs.begin(), outputs.end(), [](const OutputPulse &a, const OutputPulse &b) {
        if (a.output != b.output) return a.output < b.output;
        if (a.master != b.master) return a.master < b.master;
        return a.on < b.on;
    });
}

double DriveCountsPerUnit(const CamAxis &axis) {
    switch (axis.drive) {
        case AxisDrive::Ballscrew:
            return axis.screwPitch > 0.0 ? axis.motorPpr / axis.screwPitch : axis.countsPerUnit;
        case AxisDrive::Belt:
            return axis.pulleyPd > 0.0 ? axis.motorPpr / (kPi * axis.pulleyPd) : axis.countsPerUnit;
        case AxisDrive::Direct:
            break;
    }
    return axis.countsPerUnit;
}

double CrankUnitsPerRev(const CamMaster &m) {
    if (m.lineMode) return std::floor(m.designLength * m.lineCountsPerMm + 0.5);  // counts per design cut
    return m.crankPpr * m.crankGearNum;
}

double LineWaitDeg(const CamProject &project) {
    double wait = 360.0;
    bool any = false;
    for (const CamAxis &a : project.axes) {
        if (a.master != AxisMaster::Crank) continue;
        any = true;
        if (a.gear) continue;  // follows the line through the whole cycle: nothing to wait for
        double end = 0.0;
        for (const CamSegment &s : a.segments) {
            if (s.law != MotionLaw::Dwell) break;
            end = s.endDeg;
        }
        wait = std::min(wait, end);
    }
    for (const OutputPulse &o : project.outputs) {
        if (o.master == AxisMaster::Crank) wait = std::min(wait, o.on);
    }
    return any ? wait : 0.0;
}

bool LineSync(const CamProject &project, int *axis, double *startDeg, double *endDeg) {
    for (int i = 0; i < static_cast<int>(project.axes.size()); ++i) {
        const CamAxis &a = project.axes[i];
        if (a.master != AxisMaster::Crank || a.gear) continue;
        double x0 = 0.0;
        for (const CamSegment &seg : a.segments) {
            if (seg.law == MotionLaw::Linear) {
                *axis = i;
                *startDeg = x0;
                *endDeg = seg.endDeg;
                return true;
            }
            x0 = seg.endDeg;
        }
    }
    return false;
}

double LineMinLength(const CamProject &project) {
    return project.master.designLength * (1.0 - LineWaitDeg(project) / 360.0) - project.master.kerf;
}

CcioInput CcioInputAt(const CcioConfig &config, int board, int pin) {
    for (const CcioInput &i : config.inputs) {
        if (i.board == board && i.pin == pin) return i;
    }
    CcioInput blank;
    blank.board = board;
    blank.pin = pin;
    return blank;
}

void SetCcioInput(CcioConfig &config, const CcioInput &input) {
    auto &v = config.inputs;
    v.erase(std::remove_if(v.begin(), v.end(), [&](const CcioInput &i) { return i.board == input.board && i.pin == input.pin; }), v.end());
    if (!input.name.empty() || input.role != CcioRole::None) v.push_back(input);
    std::sort(v.begin(), v.end(), [](const CcioInput &a, const CcioInput &b) {
        return a.board != b.board ? a.board < b.board : a.pin < b.pin;
    });
}

void CcioAxisRemoved(CcioConfig &config, int axis) {
    for (CcioInput &i : config.inputs) {
        if (i.axis == axis) {
            i.axis = -1;
        } else if (i.axis > axis) {
            --i.axis;
        }
    }
}

std::string CcioPinName(int board, int pin) {
    std::string name = "CCIO";
    name += static_cast<char>('A' + board);
    name += static_cast<char>('0' + pin);
    return name;
}

BoardInput BoardInputAt(const CamProject &project, int pin) {
    for (const BoardInput &i : project.inputs) {
        if (i.pin == pin) return i;
    }
    BoardInput blank;
    blank.pin = pin;
    return blank;
}

void SetBoardInput(CamProject &project, const BoardInput &input) {
    auto &v = project.inputs;
    v.erase(std::remove_if(v.begin(), v.end(), [&](const BoardInput &i) { return i.pin == input.pin; }), v.end());
    if (!input.name.empty() || input.role != CcioRole::None) v.push_back(input);
    std::sort(v.begin(), v.end(), [](const BoardInput &a, const BoardInput &b) { return a.pin < b.pin; });
}

void BoardInputsAxisRemoved(CamProject &project, int axis) {
    for (BoardInput &i : project.inputs) {
        if (i.axis == axis) {
            i.axis = -1;
        } else if (i.axis > axis) {
            --i.axis;
        }
    }
}

std::string BoardInputBlocked(const CamProject &project, int pin, int estopPin) {
    if (pin == estopPin) return "estop";
    if (pin < kOutputs) {
        for (const OutputPulse &o : project.outputs) {
            if (o.output == pin) return "output";
        }
    }
    const CamMaster &m = project.master;
    if (m.trigger == TriggerSource::DigitalInput && UsesTimeline(project) && m.triggerInput == pin) return "trigger";
    if (m.source == MasterSource::Encoder && UsesCrank(project) && pin >= kEncoderInputFirst && pin <= kEncoderInputLast) {
        return "encoder";
    }
    return std::string();
}

bool BoardInputsOnEncoderPins(const CamProject &project) {
    for (const BoardInput &i : project.inputs) {
        if (i.pin >= kEncoderInputFirst && i.pin <= kEncoderInputLast) return true;
    }
    return false;
}

std::string BoardInputName(int pin) {
    static const char *const kNames[kTriggerInputs] = {"IO-0", "IO-1", "IO-2", "IO-3", "IO-4", "IO-5", "DI-6",
                                                       "DI-7", "DI-8", "A-9",  "A-10", "A-11", "A-12"};
    return pin >= 0 && pin < kTriggerInputs ? kNames[pin] : "?";
}

bool HasLimits(const CamProject &project) {
    for (const BoardInput &i : project.inputs) {
        if (i.role == CcioRole::LimitPositive || i.role == CcioRole::LimitNegative) return true;
    }
    for (const CcioInput &i : project.ccio.inputs) {
        if (i.role == CcioRole::LimitPositive || i.role == CcioRole::LimitNegative) return true;
    }
    return false;
}

bool WaitOnPin(const CamProject &project, int pin) {
    for (const TimelineWait &w : project.waits) {
        if (w.source == WaitSource::Board && w.pin == pin) return true;
    }
    return false;
}

bool WaitOnCcio(const CamProject &project, int board, int pin) {
    for (const TimelineWait &w : project.waits) {
        if (w.source == WaitSource::Ccio && w.board == board && w.pin == pin) return true;
    }
    return false;
}

std::string WaitInputName(const TimelineWait &w) {
    if (w.source == WaitSource::Ccio) return CcioPinName(w.board, w.pin);
    if (w.source == WaitSource::Plc) return "PLC bit " + std::to_string(w.pin);
    return BoardInputName(w.pin);
}

bool TimelineAtRest(const CamProject &project, double t) {
    for (const CamAxis &a : project.axes) {
        if (a.master != AxisMaster::Timeline || a.segments.empty()) continue;
        if (!a.cyclic && t >= a.span) continue;  // rests at its end
        double stroke = 0.0;
        for (const CamSegment &s : a.segments) stroke = std::max(stroke, std::fabs(s.endPos - a.startPos));
        // Moving at less than a millionth of its stroke per second counts as at rest (the laws' own zeros).
        const double eps = 1e-6 * std::max(stroke, 1e-3) / std::max(a.span, 1e-6);
        if (std::fabs(Evaluate(a, t).vel) > eps) return false;
    }
    return true;
}

double NearestTimelineRest(const CamProject &project, double t) {
    const double cycle = TimelineLength(project);
    if (cycle <= 0.0) return -1.0;
    t = std::min(std::max(t, 0.0), cycle);
    for (int k = 0; k * 0.001 <= cycle; ++k) {
        for (double c : {t - k * 0.001, t + k * 0.001}) {
            if (c > 0.0 && c < cycle && TimelineAtRest(project, c)) return std::round(c * 1000.0) / 1000.0;
        }
    }
    return -1.0;
}

static void NormalizeWaits(CamProject &project) {
    auto &v = project.waits;
    for (TimelineWait &w : v) {
        if (!(w.t >= 0.0)) w.t = 0.0;
        if (!(w.timeout >= 0.0)) w.timeout = 0.0;
        w.board = std::min(std::max(w.board, 0), kCcioMaxBoards - 1);
        const int pins = w.source == WaitSource::Board ? kTriggerInputs : w.source == WaitSource::Ccio ? kCcioPinsPerBoard : kPlcWaitBits;
        w.pin = std::min(std::max(w.pin, 0), pins - 1);
    }
    std::stable_sort(v.begin(), v.end(), [](const TimelineWait &a, const TimelineWait &b) { return a.t < b.t; });
    if (v.size() > static_cast<size_t>(kMaxWaits)) v.resize(kMaxWaits);
}

bool HasHomeSwitch(const CamProject &project, int axis) {
    for (const BoardInput &i : project.inputs) {
        if (i.role == CcioRole::Home && i.axis == axis) return true;
    }
    for (const CcioInput &i : project.ccio.inputs) {
        if (i.role == CcioRole::Home && i.axis == axis) return true;
    }
    return false;
}

double AxisJogLimit(const CamAxis &a) {
    const double cpu = std::fabs(a.countsPerUnit) > 0.0 ? std::fabs(a.countsPerUnit) : 1.0;
    const double board = 53333.0 / cpu;
    return a.velMax > 0.0 ? std::min(a.velMax, board) : board;
}

static void NormalizeHome(CamAxis &a) {
    AxisHome &h = a.home;
    h.dir = h.dir >= 0 ? 1 : -1;
    const double vmax = AxisJogLimit(a);
    if (!(h.fast > 0.0)) h.fast = vmax * 0.1;
    if (h.fast > vmax) h.fast = vmax;
    if (!(h.slow > 0.0)) h.slow = h.fast / 10.0;
    if (h.slow > h.fast) h.slow = h.fast;
    if (!(h.back > 0.0)) h.back = h.fast * 0.25;
    if (!(h.search >= 0.0)) h.search = 0.0;
    if (!(h.decel >= 0.0)) h.decel = 0.0;
    h.order = std::min(std::max(h.order, 1), kHomeOrderMax);
}

static void NormalizeBoardInputs(CamProject &project) {
    auto &v = project.inputs;
    v.erase(std::remove_if(v.begin(), v.end(), [](const BoardInput &i) { return i.pin < 0 || i.pin >= kTriggerInputs; }), v.end());
    for (BoardInput &i : v) {
        if (i.role == CcioRole::None || i.axis >= static_cast<int>(project.axes.size())) i.axis = -1;
    }
}

static void NormalizeCcio(CamProject &project) {
    CcioConfig &c = project.ccio;
    c.boards = std::min(std::max(c.boards, 0), kCcioMaxBoards);
    auto &v = c.inputs;
    v.erase(std::remove_if(v.begin(), v.end(), [&](const CcioInput &i) {
        return i.board < 0 || i.board >= c.boards || i.pin < 0 || i.pin >= kCcioPinsPerBoard;
    }), v.end());
    for (CcioInput &i : v) {
        if (i.role == CcioRole::None || i.axis >= static_cast<int>(project.axes.size())) i.axis = -1;
    }
}

void NormalizeProject(CamProject &project) {
    NormalizeWaits(project);
    NormalizeCcio(project);
    NormalizeBoardInputs(project);
    {
        double &s = project.master.limitStopSeconds;
        if (!(s >= kLimitStopMin)) s = kLimitStopMin;
        if (s > kLimitStopMax) s = kLimitStopMax;
    }
    CamMaster &m = project.master;
    m.crankPpr = floor(m.crankPpr + 0.5);  // whole counts per motor / encoder revolution
    if (m.crankPpr < 1.0) m.crankPpr = 1.0;
    if (m.crankGearNum < 1) m.crankGearNum = 1;
    if (m.crankGearDen < 1) m.crankGearDen = 1;
    m.countsPerRev = m.crankPpr * m.crankGearNum / m.crankGearDen;
    if (m.lineMode) {
        if (m.designLength < 1.0) m.designLength = 1.0;
        if (m.kerf < 0.0) m.kerf = 0.0;
        if (m.wheelCircumference > 0.0) m.lineCountsPerMm = m.crankPpr / m.wheelCircumference;
        if (m.lineCountsPerMm <= 0.0) m.lineCountsPerMm = 1.0;
        m.crankGearNum = 1;
        m.crankGearDen = 1;
        m.countsPerRev = CrankUnitsPerRev(m);  // whole counts per design cut
    }
    for (CamAxis &a : project.axes) {
        if (a.gear) a.master = AxisMaster::Crank;
        if (a.master == AxisMaster::Crank) a.span = 360.0;
        if (a.span <= 0.0) a.span = 1.0;
        a.cyclic = a.master == AxisMaster::Crank;
        Normalize(a);
        NormalizeHome(a);
    }
    NormalizeOutputs(project.outputs);
    // A repeating timeline wraps straight from end to start only on the axes as long as the cycle;
    // shorter ones rest at their end until the next cycle begins.
    const double cycle = TimelineLength(project);
    for (CamAxis &a : project.axes) {
        if (a.master == AxisMaster::Timeline) {
            a.cyclic = project.master.timelineRepeat && a.span >= cycle * (1.0 - 1e-9);
        }
    }
}

void SetAxisMaster(CamProject &project, int index, AxisMaster master) {
    CamAxis &a = project.axes.at(index);
    if (a.master == master) return;
    if (master == AxisMaster::Timeline) {
        ScaleSpan(a, 60.0 / std::max(project.master.rpm, 1e-6));  // one revolution at the current speed
    } else {
        ScaleSpan(a, 360.0);
    }
    a.master = master;
    NormalizeProject(project);
}

int SegmentAt(const CamAxis &axis, double deg) {
    const double d = WrapDeg(axis, deg);
    const int n = static_cast<int>(axis.segments.size());
    for (int i = 0; i < n; ++i) {
        if (d < axis.segments[i].endDeg) return i;
    }
    return n - 1;
}

void SetKnotPos(CamAxis &axis, int k, double pos) {
    KnotGroups g(axis);
    const int root = g.Find(k);
    for (int i = 0; i < KnotCount(axis); ++i) {
        if (g.Find(i) == root) SetKnotRaw(axis, i, pos);
    }
}

void SetKnotDeg(CamAxis &axis, int k, double deg) {
    const int n = KnotCount(axis);
    if (k == n - 1 && !axis.cyclic && n >= 2) {
        // Timeline: the last knot is the axis's duration. Keep the minimum segment in absolute
        // terms so shrinking the span cannot squeeze the earlier segments.
        const double minSeg = MinSegment(axis);
        const double lo = KnotDeg(axis, k - 1) + minSeg;
        axis.span = std::max(deg, lo);
        axis.segments[k - 1].endDeg = axis.span;
        return;
    }
    if (k <= 0 || k >= n - 1) return;
    const double lo = KnotDeg(axis, k - 1) + MinSegment(axis);
    const double hi = KnotDeg(axis, k + 1) - MinSegment(axis);
    axis.segments[k - 1].endDeg = Clamp(deg, lo, hi);
}

void SetSegmentLaw(CamAxis &axis, int seg, MotionLaw law) {
    if (seg < 0 || seg >= static_cast<int>(axis.segments.size())) return;
    axis.segments[seg].law = law;
    Normalize(axis);
}

void SetClosed(CamAxis &axis, bool closed) {
    axis.closed = closed;
    Normalize(axis);
}

int SplitAt(CamAxis &axis, double deg) {
    const double d = WrapDeg(axis, deg);
    const int seg = SegmentAt(axis, d);
    const double d0 = SegmentStartDeg(axis, seg);
    const double d1 = axis.segments[seg].endDeg;
    if (d - d0 < MinSegment(axis) || d1 - d < MinSegment(axis)) return -1;
    CamSegment first;
    first.endDeg = d;
    first.endPos = Evaluate(axis, d).pos;
    first.law = axis.segments[seg].law;
    axis.segments.insert(axis.segments.begin() + seg, first);
    return seg + 1;
}

void DeleteSegment(CamAxis &axis, int seg) {
    const int n = static_cast<int>(axis.segments.size());
    if (n <= 1 || seg < 0 || seg >= n) return;
    if (seg == 0) {
        // Segment 1 now starts at 0 deg from startPos.
        axis.segments.erase(axis.segments.begin());
    } else {
        axis.segments[seg - 1].endDeg = axis.segments[seg].endDeg;
        axis.segments[seg - 1].endPos = axis.segments[seg].endPos;
        axis.segments.erase(axis.segments.begin() + seg);
    }
    Normalize(axis);
}

void Normalize(CamAxis &axis) {
    if (axis.gear) {
        if (axis.gearDen < 1) axis.gearDen = 1;
        if (axis.gearNum == 0) axis.gearNum = 1;
        axis.master = AxisMaster::Crank;
        axis.span = 360.0;
        axis.closed = false;
        axis.segments = {{360.0, axis.startPos + static_cast<double>(axis.gearNum) / axis.gearDen, MotionLaw::Linear}};
    }
    if (axis.span <= 0.0) axis.span = 360.0;
    if (axis.segments.empty()) {
        axis.segments.push_back({axis.span, axis.startPos, MotionLaw::Dwell});
    }
    const int n = static_cast<int>(axis.segments.size());
    // Master positions: strictly increasing, last at the span. Squeeze from the right if needed.
    const double minSeg = MinSegment(axis);
    if (axis.span < minSeg * n) axis.span = minSeg * n;
    axis.segments[n - 1].endDeg = axis.span;
    for (int i = n - 2; i >= 0; --i) {
        const double hi = axis.segments[i + 1].endDeg - minSeg;
        const double lo = minSeg * (i + 1);
        axis.segments[i].endDeg = Clamp(axis.segments[i].endDeg, lo, hi);
    }
    if (axis.baseRadius < 1e-6) axis.baseRadius = 1e-6;
    if (axis.rollerRadius < 0.0) axis.rollerRadius = 0.0;
    if (axis.motorPpr <= 0.0) axis.motorPpr = 1.0;
    if (axis.screwPitch <= 0.0) axis.screwPitch = 1e-6;
    if (axis.pulleyPd <= 0.0) axis.pulleyPd = 1e-6;
    axis.countsPerUnit = DriveCountsPerUnit(axis);
    if (axis.countsPerUnit <= 0.0) axis.countsPerUnit = 1.0;
    axis.connector = Clamp(axis.connector, 0, kMotorConnectors - 1);
    if (axis.velMax < 0.0) axis.velMax = 0.0;
    if (axis.accMax < 0.0) axis.accMax = 0.0;
    // Tied knots take the value of the lowest-index knot in their group.
    KnotGroups g(axis);
    for (int k = 1; k < KnotCount(axis); ++k) {
        const int root = g.Find(k);
        if (root != k) SetKnotRaw(axis, k, KnotPos(axis, root));
    }
}

namespace {

// Velocity (units per master unit) where a blend meets its neighbour: the end of the previous
// segment (start = true) or the start of the next. Past the ends of a timeline, and next to another
// blend, the neighbour counts as at rest. On a crank the profile wraps (an open profile's advance is
// an offset, so its velocity wraps too).
double BlendNeighbourVel(const CamAxis &axis, int seg, bool start, const SplineFit *fit) {
    const int n = static_cast<int>(axis.segments.size());
    int k = start ? seg - 1 : seg + 1;
    if (k < 0 || k >= n) {
        if (!axis.cyclic) return 0.0;
        k = (k + n) % n;
    }
    if (k == seg || axis.segments[k].law == MotionLaw::Blend) return 0.0;
    return EvaluateSegment(axis, k, start ? 1.0 : 0.0, fit).vel;
}

// Solves a tridiagonal system in place: sub[i] x[i-1] + diag[i] x[i] + sup[i] x[i+1] = rhs[i] (Thomas). The
// spline's matrix is strictly diagonally dominant, so it needs no pivoting. rhs becomes x.
void SolveTridiagonal(const std::vector<double> &sub, std::vector<double> diag, const std::vector<double> &sup,
                      std::vector<double> &rhs) {
    const int n = static_cast<int>(rhs.size());
    for (int i = 1; i < n; ++i) {
        const double w = sub[i] / diag[i - 1];
        diag[i] -= w * sup[i - 1];
        rhs[i] -= w * rhs[i - 1];
    }
    rhs[n - 1] /= diag[n - 1];
    for (int i = n - 2; i >= 0; --i) rhs[i] = (rhs[i] - sup[i] * rhs[i + 1]) / diag[i];
}

// The spline through segments first .. first + count - 1 (indices wrap): the second derivatives M at its
// count + 1 knots, by the three-moment equations
//     mu_i M_(i-1) + 2 M_i + lambda_i M_(i+1) = 6 f[x_(i-1), x_i, x_(i+1)]
// mu_i = h_i / (h_i + h_(i+1)), lambda_i = 1 - mu_i. Only the slopes enter, so an open profile's advance
// across the wrap needs nothing. An end is clamped to a known velocity, or natural (M = 0).
struct SplineEnd {
    bool natural = false;
    double vel = 0.0;
};

void FitRun(const CamAxis &axis, int first, int count, SplineEnd left, SplineEnd right, SplineFit &fit) {
    const int n = static_cast<int>(axis.segments.size());
    std::vector<double> h(count), d(count);  // width and slope of each piece
    for (int j = 0; j < count; ++j) {
        const int s = (first + j) % n;
        h[j] = axis.segments[s].endDeg - SegmentStartDeg(axis, s);
        d[j] = (axis.segments[s].endPos - SegmentStartPos(axis, s)) / h[j];
    }
    const int knots = count + 1;
    std::vector<double> sub(knots, 0.0), diag(knots, 2.0), sup(knots, 0.0), rhs(knots, 0.0);
    for (int i = 1; i < count; ++i) {
        const double sum = h[i - 1] + h[i];
        sub[i] = h[i - 1] / sum;
        sup[i] = h[i] / sum;
        rhs[i] = 6.0 * (d[i] - d[i - 1]) / sum;
    }
    if (left.natural) {
        diag[0] = 1.0;
    } else {
        sup[0] = 1.0;
        rhs[0] = 6.0 / h[0] * (d[0] - left.vel);
    }
    if (right.natural) {
        diag[count] = 1.0;
    } else {
        sub[count] = 1.0;
        rhs[count] = 6.0 / h[count - 1] * (right.vel - d[count - 1]);
    }
    SolveTridiagonal(sub, diag, sup, rhs);
    for (int j = 0; j < count; ++j) {
        const int s = (first + j) % n;
        fit.m0[s] = rhs[j];
        fit.m1[s] = rhs[j + 1];
    }
}

// The whole cycle is one spline: the knot after the last is the first, which puts corners in the matrix.
// Sherman-Morrison turns that into two tridiagonal solves.
void FitPeriodic(const CamAxis &axis, SplineFit &fit) {
    const int n = static_cast<int>(axis.segments.size());
    if (n < 2) return;  // one piece: a straight line, M = 0
    std::vector<double> h(n), d(n);
    for (int s = 0; s < n; ++s) {
        h[s] = axis.segments[s].endDeg - SegmentStartDeg(axis, s);
        d[s] = (axis.segments[s].endPos - SegmentStartPos(axis, s)) / h[s];
    }
    // Row i is the knot at the start of segment i: mu on the knot before, lambda on the knot after.
    std::vector<double> mu(n), lambda(n), rhs(n);
    for (int i = 0; i < n; ++i) {
        const int p = (i + n - 1) % n;
        const double sum = h[p] + h[i];
        mu[i] = h[p] / sum;
        lambda[i] = h[i] / sum;
        rhs[i] = 6.0 * (d[i] - d[p]) / sum;
    }
    std::vector<double> m(n);
    if (n == 2) {
        // Both neighbours of each knot are the other knot: 2 M0 + M1 = r0, M0 + 2 M1 = r1.
        m[0] = (2.0 * rhs[0] - rhs[1]) / 3.0;
        m[1] = (2.0 * rhs[1] - rhs[0]) / 3.0;
    } else {
        const double beta = mu[0], alpha = lambda[n - 1];  // corners (0, n-1) and (n-1, 0)
        const double gamma = -2.0;
        std::vector<double> sub(n, 0.0), diag(n, 2.0), sup(n, 0.0);
        for (int i = 1; i < n; ++i) sub[i] = mu[i];
        for (int i = 0; i < n - 1; ++i) sup[i] = lambda[i];
        diag[0] -= gamma;
        diag[n - 1] -= alpha * beta / gamma;
        std::vector<double> z(n, 0.0);
        z[0] = gamma;
        z[n - 1] = alpha;
        SolveTridiagonal(sub, diag, sup, rhs);
        SolveTridiagonal(sub, diag, sup, z);
        const double k = (rhs[0] + beta * rhs[n - 1] / gamma) / (1.0 + z[0] + beta * z[n - 1] / gamma);
        for (int i = 0; i < n; ++i) m[i] = rhs[i] - k * z[i];
    }
    for (int s = 0; s < n; ++s) {
        fit.m0[s] = m[s];
        fit.m1[s] = m[(s + 1) % n];
    }
}

// How a run ends next to segment k: a Blend has zero acceleration at its ends, so a natural end joins it
// with no step (and the Blend takes its velocity from the spline); anything else gives its velocity.
SplineEnd EndNextTo(const CamAxis &axis, int k, bool neighbourEnd) {
    SplineEnd e;
    if (axis.segments[k].law == MotionLaw::Blend) {
        e.natural = true;
    } else {
        e.vel = EvaluateSegment(axis, k, neighbourEnd ? 1.0 : 0.0).vel;
    }
    return e;
}

}  // namespace

SplineFit FitSplines(const CamAxis &axis) {
    SplineFit fit;
    const int n = static_cast<int>(axis.segments.size());
    fit.m0.assign(n, 0.0);
    fit.m1.assign(n, 0.0);
    auto isSpline = [&](int s) { return axis.segments[s].law == MotionLaw::Spline; };
    int splines = 0;
    for (int s = 0; s < n; ++s) splines += isSpline(s);
    if (splines == 0) return fit;
    if (splines == n && axis.cyclic) {
        FitPeriodic(axis, fit);
        return fit;
    }
    if (axis.cyclic) {
        // Runs may wrap: start after a segment that is no spline.
        int start = 0;
        while (isSpline(start)) ++start;
        for (int j = 1; j <= n;) {
            const int s = (start + j) % n;
            if (!isSpline(s)) {
                ++j;
                continue;
            }
            int count = 0;
            while (isSpline((s + count) % n)) ++count;
            FitRun(axis, s, count, EndNextTo(axis, (s + n - 1) % n, true), EndNextTo(axis, (s + count) % n, false), fit);
            j += count;
        }
        return fit;
    }
    // Not cyclic: the axis rests before its start and after its end (SplineEnd's default, velocity 0).
    for (int s = 0; s < n;) {
        if (!isSpline(s)) {
            ++s;
            continue;
        }
        int count = 0;
        while (s + count < n && isSpline(s + count)) ++count;
        FitRun(axis, s, count, s > 0 ? EndNextTo(axis, s - 1, true) : SplineEnd(),
               s + count < n ? EndNextTo(axis, s + count, false) : SplineEnd(), fit);
        s += count;
    }
    return fit;
}

CamPoint EvaluateSegment(const CamAxis &axis, int seg, double u, const SplineFit *fit) {
    CamPoint r;
    r.segment = seg;
    const double d0 = SegmentStartDeg(axis, seg);
    const double p0 = SegmentStartPos(axis, seg);
    const CamSegment &s = axis.segments[seg];
    const double w = s.endDeg - d0;
    const double h = s.endPos - p0;
    if (s.law == MotionLaw::Blend && w > 0.0) {
        // Quintic Hermite: position p0 -> p0 + h, velocity v0 -> v1, zero acceleration at both ends.
        if (u < 0.0) u = 0.0;
        if (u > 1.0) u = 1.0;
        const double v0 = BlendNeighbourVel(axis, seg, true, fit) * w;  // per unit u
        const double v1 = BlendNeighbourVel(axis, seg, false, fit) * w;
        const double u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
        // Basis: P (0 -> 1), B0 (unit start velocity), B1 (unit end velocity).
        const double P = 10 * u3 - 15 * u4 + 6 * u5, P1 = 30 * u2 - 60 * u3 + 30 * u4;
        const double P2 = 60 * u - 180 * u2 + 120 * u3, P3 = 60 - 360 * u + 360 * u2;
        const double B0 = u - 6 * u3 + 8 * u4 - 3 * u5, B01 = 1 - 18 * u2 + 32 * u3 - 15 * u4;
        const double B02 = -36 * u + 96 * u2 - 60 * u3, B03 = -36 + 192 * u - 180 * u2;
        const double B1 = -4 * u3 + 7 * u4 - 3 * u5, B11 = -12 * u2 + 28 * u3 - 15 * u4;
        const double B12 = -24 * u + 84 * u2 - 60 * u3, B13 = -24 + 168 * u - 180 * u2;
        r.pos = p0 + h * P + v0 * B0 + v1 * B1;
        r.vel = (h * P1 + v0 * B01 + v1 * B11) / w;
        r.acc = (h * P2 + v0 * B02 + v1 * B12) / (w * w);
        r.jerk = (h * P3 + v0 * B03 + v1 * B13) / (w * w * w);
        return r;
    }
    if (s.law == MotionLaw::Spline && w > 0.0) {
        SplineFit own;
        if (!fit) {
            own = FitSplines(axis);
            fit = &own;
        }
        if (u < 0.0) u = 0.0;
        if (u > 1.0) u = 1.0;
        // The piece between two knots: its second derivative is linear from m0 to m1; integrated twice,
        // with the knot positions fixing the constants.
        const double m0 = fit->m0[seg], m1 = fit->m1[seg];
        const double a = w - u * w, b = u * w;  // distance to the end and from the start
        const double c0 = p0 / w - m0 * w / 6.0, c1 = s.endPos / w - m1 * w / 6.0;
        r.pos = (m0 * a * a * a + m1 * b * b * b) / (6.0 * w) + c0 * a + c1 * b;
        r.vel = (m1 * b * b - m0 * a * a) / (2.0 * w) + c1 - c0;
        r.acc = (m0 * a + m1 * b) / w;
        r.jerk = (m1 - m0) / w;
        return r;
    }
    const LawValue l = EvaluateLaw(s.law, u);
    r.pos = p0 + h * l.s;
    if (w > 0.0) {
        r.vel = h * l.v / w;
        r.acc = h * l.a / (w * w);
        r.jerk = h * l.j / (w * w * w);
    }
    return r;
}

CamPoint Evaluate(const CamAxis &axis, double deg) { return Evaluate(axis, deg, nullptr); }

CamPoint Evaluate(const CamAxis &axis, double deg, const SplineFit *fit) {
    if (!axis.cyclic && (deg < 0.0 || deg > axis.span)) {
        // At rest before the start and after the end of a timeline diagram.
        CamPoint r;
        const bool before = deg < 0.0;
        r.segment = before ? 0 : static_cast<int>(axis.segments.size()) - 1;
        r.pos = before ? axis.startPos : axis.segments.back().endPos;
        return r;
    }
    const double d = WrapDeg(axis, deg);
    const int seg = SegmentAt(axis, d);
    const double d0 = SegmentStartDeg(axis, seg);
    const double w = axis.segments[seg].endDeg - d0;
    const double u = w > 0.0 ? (d - d0) / w : 0.0;
    return EvaluateSegment(axis, seg, u, fit);
}

AxisStats Analyze(const CamAxis &axis, int samples) {
    AxisStats st;
    if (axis.segments.empty() || samples < 8) return st;

    const SplineFit fit = FitSplines(axis);
    std::vector<CamPoint> pts(samples);
    st.minPos = st.maxPos = axis.startPos;
    for (int i = 0; i < samples; ++i) {
        const double deg = axis.span * i / samples;
        pts[i] = Evaluate(axis, deg, &fit);
        const CamPoint &p = pts[i];
        st.minPos = std::min(st.minPos, p.pos);
        st.maxPos = std::max(st.maxPos, p.pos);
        if (std::fabs(p.vel) > st.maxVel) {
            st.maxVel = std::fabs(p.vel);
            st.maxVelDeg = deg;
        }
        if (std::fabs(p.acc) > st.maxAcc) {
            st.maxAcc = std::fabs(p.acc);
            st.maxAccDeg = deg;
        }
        if (std::fabs(p.jerk) > st.maxJerk) {
            st.maxJerk = std::fabs(p.jerk);
            st.maxJerkDeg = deg;
        }
    }
    const double endPos = axis.segments.back().endPos;
    st.minPos = std::min(st.minPos, endPos);
    st.maxPos = std::max(st.maxPos, endPos);
    st.advancePerRev = endPos - axis.startPos;

    // A spline knows nothing of the range drawn: report it passing the axis's knots.
    {
        double kLo = axis.startPos, kHi = axis.startPos;
        for (const CamSegment &s : axis.segments) {
            kLo = std::min(kLo, s.endPos);
            kHi = std::max(kHi, s.endPos);
        }
        const double room = kSplineOvershootLimit * (kHi - kLo);
        double worst = room, worstPos = 0.0, worstKnot = 0.0, worstDeg = 0.0;
        for (int i = 0; i < samples; ++i) {
            const CamPoint &p = pts[i];
            if (axis.segments[p.segment].law != MotionLaw::Spline) continue;
            const double past = std::max(p.pos - kHi, kLo - p.pos);
            if (past > worst) {
                worst = past;
                worstPos = p.pos;
                worstKnot = p.pos > kHi ? kHi : kLo;
                worstDeg = axis.span * i / samples;
            }
        }
        if (worst > room) st.issues.push_back({IssueKind::SplineOvershoot, -1, worstDeg, worstPos, worstKnot});
    }

    // Cam drawing geometry: pitch radius R = base + roller + (s - sMin). Crank only.
    const double rp = axis.baseRadius + axis.rollerRadius;
    st.minCurvature = 0.0;
    bool haveCurv = false;
    for (int i = 0; i < (axis.master == AxisMaster::Crank ? samples : 0); ++i) {
        const CamPoint &p = pts[i];
        const double r = rp + (p.pos - st.minPos);
        const double r1 = p.vel * kDegPerRad;               // dR/dtheta (per radian)
        const double r2 = p.acc * kDegPerRad * kDegPerRad;  // d2R/dtheta2
        const double pa = std::atan2(std::fabs(r1), r) * kDegPerRad;
        if (pa > st.maxPressureDeg) {
            st.maxPressureDeg = pa;
            st.maxPressureAt = axis.span * i / samples;
        }
        const double den = r * r + 2 * r1 * r1 - r * r2;
        if (den > 0.0) {  // convex
            const double rho = std::pow(r * r + r1 * r1, 1.5) / den;
            if (!haveCurv || rho < st.minCurvature) {
                st.minCurvature = rho;
                st.minCurvatureAt = axis.span * i / samples;
                haveCurv = true;
            }
        }
    }
    if (st.maxPressureDeg > kMaxPressureAngleDeg) {
        st.issues.push_back({IssueKind::PressureAngle, -1, st.maxPressureAt, st.maxPressureDeg, 0.0});
    }
    if (axis.rollerRadius > 0.0 && haveCurv && st.minCurvature < axis.rollerRadius) {
        st.issues.push_back({IssueKind::Undercut, -1, st.minCurvatureAt, st.minCurvature, 0.0});
    }

    // Steps at the knots: on a crank including the wrap from 360 back to 0 (the next revolution);
    // on a timeline the axis starts from rest at t = 0 and comes to rest at its end.
    const int n = static_cast<int>(axis.segments.size());
    const int lastKnot = axis.cyclic ? n - 1 : n;
    for (int k = 0; k <= lastKnot; ++k) {
        CamPoint l, r;  // default: at rest
        if (axis.cyclic || k > 0) l = EvaluateSegment(axis, (k + n - 1) % n, 1.0, &fit);
        if (k < n) r = EvaluateSegment(axis, k, 0.0, &fit);
        const double vScale = std::max(std::fabs(l.vel), std::fabs(r.vel));
        if (std::fabs(l.vel - r.vel) > 1e-6 * vScale + 1e-9) {
            st.issues.push_back({IssueKind::VelocityStep, k, KnotDeg(axis, k), l.vel, r.vel});
            continue;
        }
        const double aScale = std::max(std::fabs(l.acc), std::fabs(r.acc));
        if (std::fabs(l.acc - r.acc) > 1e-6 * aScale + 1e-12) {
            st.issues.push_back({IssueKind::AccelStep, k, KnotDeg(axis, k), l.acc, r.acc});
        }
    }
    return st;
}

SpeedLimit MaxMasterSpeed(const CamAxis &axis, double rampDegPerSec2, int samples) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    SpeedLimit r;
    r.maxDegPerSec = kInf;
    r.maxRampDegPerSec2 = kInf;
    if (axis.segments.empty() || samples < 8) return r;
    const SplineFit fit = FitSplines(axis);
    const double alpha = std::fabs(rampDegPerSec2);
    double peakVel = 0.0;
    for (int i = 0; i < samples; ++i) {
        const double deg = axis.span * i / samples;
        const CamPoint p = Evaluate(axis, deg, &fit);
        const double v = std::fabs(p.vel), a = std::fabs(p.acc);
        if (v > peakVel) peakVel = v;
        if (axis.velMax > 0.0 && v > 0.0) {
            const double w = axis.velMax / v;
            if (w < r.maxDegPerSec) {
                r.maxDegPerSec = w;
                r.reason = SpeedLimit::Velocity;
                r.atDeg = deg;
            }
        }
        if (axis.accMax > 0.0) {
            // |s''| w^2 + |s'| alpha <= A, conservatively (both terms at their worst sign).
            const double room = axis.accMax - v * alpha;
            if (room <= 0.0) {
                r.maxDegPerSec = 0.0;
                r.reason = SpeedLimit::Ramp;
                r.atDeg = deg;
            } else if (a > 0.0) {
                const double w = std::sqrt(room / a);
                if (w < r.maxDegPerSec) {
                    r.maxDegPerSec = w;
                    r.reason = SpeedLimit::Accel;
                    r.atDeg = deg;
                }
            }
        }
    }
    if (axis.accMax > 0.0 && peakVel > 0.0) r.maxRampDegPerSec2 = axis.accMax / peakVel;
    return r;
}

}  // namespace ecam
