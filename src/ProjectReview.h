/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// What a review of a project needs, shared by the Excel workbook and the PDF report so both say the
// same: per-axis analysis at the master speed, the fastest master speed the limits allow, the checks
// (in the order the app lists them), and the master / axis settings and results as rows.

#include "CamProfile.h"

#include <QString>

#include <limits>
#include <vector>

namespace review {

QString Q(const std::string &s);
QString InputName(int i);  // IO-0..IO-5, DI-6..DI-8, A-9..A-12
QString LawText(ecam::MotionLaw law);
QString DriveText(ecam::AxisDrive d);
QString TriggerText(const ecam::CamMaster &m);
inline bool OnTimeline(const ecam::CamAxis &a) { return a.master == ecam::AxisMaster::Timeline; }
QString XUnit(const ecam::CamAxis &a);  // "s" or "°"
QString Sig(double v);                  // six significant digits, "—" when not finite

// The sample at i of n over one cycle; the last one is the end of the cycle, not the wrapped start.
ecam::CamPoint SampleAt(const ecam::CamAxis &a, int i, int n);
// Position on the shared timeline at t (as the timing view draws it).
ecam::CamPoint TimelineAt(const ecam::CamAxis &a, double t);

struct Peaks {
    double vel = 0.0, acc = 0.0, jerk = 0.0;  // |per x| peaks
};
Peaks SegmentPeaks(const ecam::CamAxis &a, int seg);

struct Check {
    enum Level { Error, Warning } level;
    QString where;  // "Project", "Master" or the axis name
    QString text;
};

struct AxisInfo {
    ecam::AxisStats stats;
    double w = 1.0;           // master units per second
    ecam::SpeedLimit limit;   // fastest master speed the axis's limits allow
    double motorPeakRpm = 0.0;
    double stepRateHz = 0.0;
    double worstAcc = 0.0;    // with the crank ramp, units/s^2
};

struct ProjectInfo {
    std::vector<AxisInfo> axes;
    double crankMaxRpm = std::numeric_limits<double>::infinity();  // what the project allows
    QString crankReason;
    double overrideMax = std::numeric_limits<double>::infinity();
    QString overrideReason;
    std::vector<Check> checks;
    int errors() const;
    int warnings() const;
};

// estopPin: the board's estop input (as the trigger index), -1 none or not known: a project that uses it is flagged.
// estopPin: the board's estop input (-1 none); opPins: its operator inputs' (5, -1 none), or nullptr.
ProjectInfo AnalyzeProject(const ecam::CamProject &project, int estopPin = -1, const int *opPins = nullptr);

// ---- Rows -----------------------------------------------------------------------------------------
// How a number is shown (Excel number format in the workbook, fixed decimals in the report).
enum class Fmt { Int, One, Three, Deg, Sec, Pos, Rate, Pct };
QString FormatValue(double v, Fmt f);  // Pct: a fraction shown as percent

struct Row {
    QString key;
    QString text;  // used when the value is text
    double value = std::numeric_limits<double>::quiet_NaN();
    Fmt fmt = Fmt::Three;
    QString unit;
    QString note;
    bool isText() const { return !text.isEmpty() || !(value == value); }
    QString shown() const;  // the text, or the formatted value
};
Row TextRow(const QString &k, const QString &v, const QString &note = {});
Row NumRow(const QString &k, double v, Fmt fmt, const QString &unit = {}, const QString &note = {});  // NaN: "not set"

std::vector<Row> MasterRows(const ecam::CamProject &project, const ProjectInfo &pi);
std::vector<Row> AxisSettingsRows(const ecam::CamAxis &a);

struct StatRow {
    QString key;
    double v;
    Fmt fmt;
    QString unit;
    QString at;  // where on the profile ("120.0°", "0.250 s")
};
std::vector<StatRow> AxisStatRows(const ecam::CamProject &project, int axis, const AxisInfo &ai);

}  // namespace review
