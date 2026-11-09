/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "WorkbookExport.h"

#include "Brand.h"
#include "CamGeometry.h"
#include "ProjectReview.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>

#include <xlsxwriter.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace ecam;
using namespace review;

namespace io {

namespace {

// Categorical series colors in fixed order (validated palette, light surface): axis i always wears
// slot i, on every chart in the workbook. Quantities on a motion sheet wear slots 1..4.
constexpr lxw_color_t kSeries[] = {0x2A78D6, 0xEB6834, 0x1BAF7A, 0xEDA100, 0xE87BA4, 0x008300, 0x6250D6, 0xE34948};
constexpr int kSeriesCount = static_cast<int>(sizeof(kSeries) / sizeof(kSeries[0]));
constexpr lxw_color_t kInk = 0x0B0B0B;
constexpr lxw_color_t kInk2 = 0x52514E;
constexpr lxw_color_t kMuted = 0x8A8985;
constexpr lxw_color_t kGrid = 0xE6E5E1;
constexpr lxw_color_t kRule = 0xC9C8C2;
constexpr lxw_color_t kError = 0xB3261E;
constexpr lxw_color_t kWarning = 0x8A5A00;
constexpr lxw_color_t kGood = 0x1E6B34;
constexpr lxw_color_t kTabSummary = 0x3A3A38;

constexpr double kPxPerRow = 20.0;  // Excel's default row height in pixels
constexpr double kChartW = 480.0;   // libxlsxwriter's default chart size in pixels
constexpr double kChartH = 288.0;

lxw_color_t SeriesColor(int i) { return kSeries[((i % kSeriesCount) + kSeriesCount) % kSeriesCount]; }

std::string U8(const QString &s) { return s.toStdString(); }
QString XHeader(const CamAxis &a) { return OnTimeline(a) ? QStringLiteral("Time (s)") : QStringLiteral("Crank (°)"); }

// ---- Workbook writer --------------------------------------------------------------------------

struct Col {
    QString header;
    const char *fmt = nullptr;  // number format of the data cells
    double width = 12.0;
};

class Book {
public:
    explicit Book(lxw_workbook *wb) : m_wb(wb) {
        auto font = [](lxw_format *f) { format_set_font_name(f, "Calibri"); };
        m_title = workbook_add_format(wb);
        font(m_title);
        format_set_font_size(m_title, 18);
        format_set_bold(m_title);
        format_set_font_color(m_title, kInk);
        m_subtitle = workbook_add_format(wb);
        font(m_subtitle);
        format_set_font_size(m_subtitle, 11);
        format_set_font_color(m_subtitle, kInk2);
        m_h2 = workbook_add_format(wb);
        font(m_h2);
        format_set_font_size(m_h2, 13);
        format_set_bold(m_h2);
        format_set_font_color(m_h2, kInk);
        format_set_bottom(m_h2, LXW_BORDER_THIN);
        format_set_bottom_color(m_h2, kRule);
        m_note = workbook_add_format(wb);
        font(m_note);
        format_set_italic(m_note);
        format_set_font_color(m_note, kMuted);
        m_link = workbook_add_format(wb);
        font(m_link);
        format_set_underline(m_link, LXW_UNDERLINE_SINGLE);
        format_set_font_color(m_link, 0x1F5FAD);
        m_error = workbook_add_format(wb);
        font(m_error);
        format_set_bold(m_error);
        format_set_font_color(m_error, kError);
        m_warning = workbook_add_format(wb);
        font(m_warning);
        format_set_bold(m_warning);
        format_set_font_color(m_warning, kWarning);
        m_good = workbook_add_format(wb);
        font(m_good);
        format_set_bold(m_good);
        format_set_font_color(m_good, kGood);
        m_text = workbook_add_format(wb);
        format_set_align(m_text, LXW_ALIGN_LEFT);
        m_wrap = workbook_add_format(wb);
        format_set_text_wrap(m_wrap);
        format_set_align(m_wrap, LXW_ALIGN_VERTICAL_TOP);
    }

    lxw_workbook *wb() const { return m_wb; }
    lxw_format *title() const { return m_title; }
    lxw_format *subtitle() const { return m_subtitle; }
    lxw_format *h2() const { return m_h2; }
    lxw_format *note() const { return m_note; }
    lxw_format *link() const { return m_link; }
    lxw_format *error() const { return m_error; }
    lxw_format *warning() const { return m_warning; }
    lxw_format *good() const { return m_good; }
    lxw_format *text() const { return m_text; }
    lxw_format *wrap() const { return m_wrap; }

    lxw_format *Num(const char *fmt) {
        if (!fmt) return nullptr;
        auto it = m_nums.find(fmt);
        if (it != m_nums.end()) return it->second;
        lxw_format *f = workbook_add_format(m_wb);
        format_set_num_format(f, fmt);
        m_nums[fmt] = f;
        return f;
    }

    // A unique, valid sheet name (31 characters, none of []:*?/\').
    lxw_worksheet *AddSheet(QString name, lxw_color_t tab) {
        name.remove(QRegularExpression(QStringLiteral("[\\[\\]:*?/\\\\']")));
        name = name.simplified();
        if (name.isEmpty()) name = QStringLiteral("Sheet");
        name = name.left(31);
        QString unique = name;
        for (int n = 2; m_names.count(unique.toLower().toStdString()); ++n) {
            const QString suffix = QStringLiteral(" (%1)").arg(n);
            unique = name.left(31 - suffix.size()) + suffix;
        }
        m_names.insert(unique.toLower().toStdString());
        lxw_worksheet *ws = workbook_add_worksheet(m_wb, U8(unique).c_str());
        m_sheetName[ws] = U8(unique);
        worksheet_set_tab_color(ws, tab);
        return ws;
    }
    const std::string &Name(lxw_worksheet *ws) { return m_sheetName[ws]; }
    // "internal:'Sheet name'!A1"
    std::string LinkTo(lxw_worksheet *ws) { return "internal:'" + Name(ws) + "'!A1"; }

    void Str(lxw_worksheet *ws, int r, int c, const QString &s, lxw_format *f = nullptr) {
        if (s.isEmpty()) {
            if (f) worksheet_write_blank(ws, r, c, f);
            return;
        }
        worksheet_write_string(ws, r, c, U8(s).c_str(), f);
    }
    // Numbers that are not finite (no limit set) are written as a dash.
    void Num(lxw_worksheet *ws, int r, int c, double v, const char *fmt = nullptr) {
        if (std::isfinite(v)) {
            worksheet_write_number(ws, r, c, v, Num(fmt));
        } else {
            worksheet_write_string(ws, r, c, "—", m_text);
        }
    }
    void Link(lxw_worksheet *ws, int r, int c, lxw_worksheet *target, const QString &text) {
        worksheet_write_url(ws, r, c, LinkTo(target).c_str(), m_link);
        worksheet_write_string(ws, r, c, U8(text).c_str(), m_link);
    }

    // An Excel table over [r0 .. r0 + rows] x [c0 .. c0 + cols - 1]: a header row then `rows` rows
    // already written (or written after). Sets the column widths. rows must be >= 1.
    void Table(lxw_worksheet *ws, int r0, int c0, int rows, const std::vector<Col> &cols, int style = 2) {
        std::vector<std::string> headers;
        headers.reserve(cols.size());
        std::vector<lxw_table_column> defs(cols.size());
        std::vector<lxw_table_column *> ptrs;
        for (size_t i = 0; i < cols.size(); ++i) headers.push_back(U8(cols[i].header));
        for (size_t i = 0; i < cols.size(); ++i) {
            defs[i] = lxw_table_column{};
            defs[i].header = headers[i].c_str();
            ptrs.push_back(&defs[i]);
            worksheet_set_column(ws, c0 + static_cast<int>(i), c0 + static_cast<int>(i), cols[i].width, nullptr);
        }
        ptrs.push_back(nullptr);
        lxw_table_options opt{};
        opt.style_type = LXW_TABLE_STYLE_TYPE_MEDIUM;
        opt.style_type_number = static_cast<uint8_t>(style);
        opt.columns = ptrs.data();
        worksheet_add_table(ws, r0, c0, r0 + std::max(rows, 1), c0 + static_cast<int>(cols.size()) - 1, &opt);
    }

    // A scatter chart with straight lines and no markers, Excel's way of drawing y(x).
    lxw_chart *Chart(const QString &title, const QString &xName, const QString &yName) {
        lxw_chart *ch = workbook_add_chart(m_wb, LXW_CHART_SCATTER_STRAIGHT);
        const std::string t = U8(title), xn = U8(xName), yn = U8(yName);
        lxw_chart_font titleFont{};
        titleFont.size = 12;
        titleFont.bold = LXW_EXPLICIT_FALSE;
        titleFont.color = kInk;
        chart_title_set_name(ch, t.c_str());
        chart_title_set_name_font(ch, &titleFont);
        lxw_chart_font axisFont{};
        axisFont.size = 9;
        axisFont.bold = LXW_EXPLICIT_FALSE;
        axisFont.color = kInk2;
        lxw_chart_font numFont{};
        numFont.size = 9;
        numFont.color = kInk2;
        lxw_chart_line gridLine{};
        gridLine.color = kGrid;
        gridLine.width = 0.75f;
        lxw_chart_line axisLine{};
        axisLine.color = kRule;
        for (lxw_chart_axis *ax : {ch->x_axis, ch->y_axis}) {
            chart_axis_set_name_font(ax, &axisFont);
            chart_axis_set_num_font(ax, &numFont);
            chart_axis_set_line(ax, &axisLine);
            chart_axis_major_gridlines_set_visible(ax, LXW_TRUE);
            chart_axis_major_gridlines_set_line(ax, &gridLine);
            chart_axis_set_label_position(ax, LXW_CHART_AXIS_LABEL_POSITION_LOW);
        }
        // Tick labels: compact numbers, not the cells' 4-decimal formats ("General" alone is taken as
        // "linked to the source cells").
        chart_axis_set_num_format(ch->x_axis, "General;-General");
        chart_axis_set_num_format(ch->y_axis, "General;-General");
        chart_axis_set_name(ch->x_axis, xn.c_str());
        chart_axis_set_name(ch->y_axis, yn.c_str());
        lxw_chart_line noLine{};
        noLine.none = LXW_TRUE;
        chart_chartarea_set_line(ch, &noLine);
        chart_legend_set_position(ch, LXW_CHART_LEGEND_NONE);
        return ch;
    }

    // y in column yc against x in column xc, rows r0..r1 of sheet ws.
    lxw_chart_series *Series(lxw_chart *ch, lxw_worksheet *ws, int xc, int yc, int r0, int r1, const QString &name,
                             lxw_color_t color, float width = 1.75f, uint8_t dash = LXW_CHART_LINE_DASH_SOLID) {
        lxw_chart_series *s = chart_add_series(ch, nullptr, nullptr);
        const std::string &sheet = Name(ws);
        chart_series_set_categories(s, sheet.c_str(), r0, xc, r1, xc);
        chart_series_set_values(s, sheet.c_str(), r0, yc, r1, yc);
        const std::string n = U8(name);
        chart_series_set_name(s, n.c_str());
        lxw_chart_line line{};
        line.color = color;
        line.width = width;
        line.dash_type = dash;
        chart_series_set_line(s, &line);
        chart_series_set_marker_type(s, LXW_CHART_MARKER_NONE);
        return s;
    }

    void Insert(lxw_worksheet *ws, int r, int c, lxw_chart *ch, double wPx, double hPx) {
        lxw_chart_options o{};
        o.x_scale = wPx / kChartW;
        o.y_scale = hPx / kChartH;
        o.object_position = LXW_OBJECT_MOVE_DONT_SIZE;
        worksheet_insert_chart_opt(ws, r, c, ch, &o);
    }

private:
    lxw_workbook *m_wb;
    lxw_format *m_title, *m_subtitle, *m_h2, *m_note, *m_link, *m_error, *m_warning, *m_good, *m_text, *m_wrap;
    std::map<std::string, lxw_format *> m_nums;
    std::set<std::string> m_names;
    std::map<lxw_worksheet *, std::string> m_sheetName;
};

int ChartRows(double hPx) { return static_cast<int>(std::ceil(hPx / kPxPerRow)) + 1; }

// Round a chart bound outward to a tidy number.
double NiceCeil(double v) {
    if (v <= 0.0) return 1.0;
    const double p = std::pow(10.0, std::floor(std::log10(v)));
    for (double m : {1.0, 1.2, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0}) {
        if (v <= m * p + 1e-12) return m * p;
    }
    return 10.0 * p;
}

constexpr const char *kFmtDeg = "0.00";
constexpr const char *kFmtSec = "0.0000";
constexpr const char *kFmtPos = "#,##0.0000";
constexpr const char *kFmtRate = "#,##0.000";
constexpr const char *kFmtInt = "#,##0";
constexpr const char *kFmt1 = "#,##0.0";
constexpr const char *kFmt3 = "#,##0.000";
constexpr const char *kFmtPct = "0.0%";
constexpr const char *kFmtGeneral = "General";

const char *XFmt(const CamAxis &a) { return OnTimeline(a) ? kFmtSec : kFmtDeg; }

// ---- Sheets -------------------------------------------------------------------------------------

struct AxisSheets {
    lxw_worksheet *profile = nullptr;
    lxw_worksheet *motion = nullptr;
    lxw_worksheet *cam = nullptr;
};

// Key / value / unit rows (review::Row); returns the next free row.
using KV = review::Row;

const char *ExcelFmt(Fmt f) {
    switch (f) {
        case Fmt::Int: return kFmtInt;
        case Fmt::One: return kFmt1;
        case Fmt::Three: return kFmt3;
        case Fmt::Deg: return kFmtDeg;
        case Fmt::Sec: return kFmtSec;
        case Fmt::Pos: return kFmtPos;
        case Fmt::Rate: return kFmtRate;
        case Fmt::Pct: return kFmtPct;
    }
    return kFmtGeneral;
}

int WriteKvTable(Book &b, lxw_worksheet *ws, int r0, int c0, const std::vector<KV> &rows, bool withNote) {
    for (size_t i = 0; i < rows.size(); ++i) {
        const int r = r0 + 1 + static_cast<int>(i);
        const KV &kv = rows[i];
        b.Str(ws, r, c0, kv.key);
        if (kv.text.isEmpty() && std::isfinite(kv.value)) {
            b.Num(ws, r, c0 + 1, kv.value, ExcelFmt(kv.fmt));
        } else {
            b.Str(ws, r, c0 + 1, kv.text, b.text());
        }
        b.Str(ws, r, c0 + 2, kv.unit);
        if (withNote) b.Str(ws, r, c0 + 3, kv.note);
    }
    std::vector<Col> cols = {{QStringLiteral("Parameter"), nullptr, 30},
                             {QStringLiteral("Value"), nullptr, 16},
                             {QStringLiteral("Unit"), nullptr, 10}};
    if (withNote) cols.push_back({QStringLiteral("Note"), nullptr, 60});
    b.Table(ws, r0, c0, static_cast<int>(rows.size()), cols);
    return r0 + static_cast<int>(rows.size()) + 2;
}

void WriteSummary(Book &b, lxw_worksheet *ws, const CamProject &project, const ProjectInfo &pi, const WorkbookOptions &o,
                  const std::vector<AxisSheets> &sheets, lxw_worksheet *master, lxw_worksheet *timing) {
    worksheet_gridlines(ws, LXW_HIDE_ALL_GRIDLINES);
    worksheet_set_column(ws, 0, 0, 2, nullptr);  // gutter
    const QString name = o.projectName.isEmpty() ? QStringLiteral("Untitled") : o.projectName;
    b.Str(ws, 0, 1, name, b.title());
    worksheet_set_row(ws, 0, 28, nullptr);
    b.Str(ws, 1, 1, QStringLiteral(ECAM_PRODUCT_NAME " project · exported %1")
                        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm"))),
          b.subtitle());
    if (!o.projectPath.isEmpty()) b.Str(ws, 2, 1, QDir::toNativeSeparators(o.projectPath), b.note());

    int r = 4;
    // Checks first: the thing to look at.
    b.Str(ws, r, 1, QStringLiteral("Checks"), b.h2());
    for (int c = 2; c <= 4; ++c) worksheet_write_blank(ws, r, c, b.h2());
    ++r;
    if (pi.checks.empty()) {
        b.Str(ws, r, 1, QStringLiteral("✓ No issues: no limit exceeded, velocity and acceleration continuous at every knot."), b.good());
        r += 2;
    } else {
        for (size_t i = 0; i < pi.checks.size(); ++i) {
            const Check &c = pi.checks[i];
            const int rr = r + 1 + static_cast<int>(i);
            b.Str(ws, rr, 1, c.level == Check::Error ? QStringLiteral("⚠ Error") : QStringLiteral("⚠ Warning"),
                  c.level == Check::Error ? b.error() : b.warning());
            b.Str(ws, rr, 2, c.where);
            b.Str(ws, rr, 3, c.text);
        }
        b.Table(ws, r, 1, static_cast<int>(pi.checks.size()),
                {{QStringLiteral("Level"), nullptr, 14}, {QStringLiteral("Where"), nullptr, 18}, {QStringLiteral("Check"), nullptr, 90}}, 3);
        r += static_cast<int>(pi.checks.size()) + 3;
    }

    // Axes at a glance.
    b.Str(ws, r, 1, QStringLiteral("Axes"), b.h2());
    for (int c = 2; c <= 4; ++c) worksheet_write_blank(ws, r, c, b.h2());
    ++r;
    if (project.axes.empty()) {
        b.Str(ws, r, 1, QStringLiteral("No axes."), b.note());
        r += 2;
    } else {
        const int r0 = r;
        for (size_t i = 0; i < project.axes.size(); ++i) {
            const CamAxis &a = project.axes[i];
            const AxisInfo &ai = pi.axes[i];
            const int rr = r0 + 1 + static_cast<int>(i);
            int c = 1;
            b.Link(ws, rr, c++, sheets[i].profile, Q(a.name));
            b.Str(ws, rr, c++, OnTimeline(a) ? QStringLiteral("Timeline") : QStringLiteral("Crank"));
            b.Str(ws, rr, c++, QStringLiteral("M-%1").arg(a.connector));
            b.Str(ws, rr, c++, Q(a.units));
            b.Str(ws, rr, c++, DriveText(a.drive));
            b.Num(ws, rr, c++, a.countsPerUnit, kFmt3);
            b.Num(ws, rr, c++, static_cast<double>(a.segments.size()), kFmtInt);
            b.Num(ws, rr, c++, ai.stats.maxPos - ai.stats.minPos, kFmtPos);
            b.Num(ws, rr, c++, ai.stats.maxVel * ai.w, kFmtRate);
            b.Num(ws, rr, c++, ai.stats.maxAcc * ai.w * ai.w, kFmtRate);
            b.Num(ws, rr, c++, ai.motorPeakRpm, kFmt1);
            b.Num(ws, rr, c++, ai.stepRateHz / 1000.0, kFmt3);
            if (OnTimeline(a)) {
                b.Num(ws, rr, c++, a.span, kFmtSec);
                b.Str(ws, rr, c++, QStringLiteral("s"));
            } else {
                b.Num(ws, rr, c++, 360.0, kFmtDeg);
                b.Str(ws, rr, c++, QStringLiteral("°"));
            }
            b.Str(ws, rr, c++, a.closed ? QStringLiteral("Closed") : QStringLiteral("Open"));
        }
        b.Table(ws, r0, 1, static_cast<int>(project.axes.size()),
                {{QStringLiteral("Axis"), nullptr, 18},
                 {QStringLiteral("Master"), nullptr, 18},
                 {QStringLiteral("Motor"), nullptr, 9},
                 {QStringLiteral("Units"), nullptr, 8},
                 {QStringLiteral("Drive"), nullptr, 13},
                 {QStringLiteral("Counts/unit"), nullptr, 14},
                 {QStringLiteral("Segments"), nullptr, 12},
                 {QStringLiteral("Stroke"), nullptr, 12},
                 {QStringLiteral("Peak vel (/s)"), nullptr, 16},
                 {QStringLiteral("Peak acc (/s²)"), nullptr, 17},
                 {QStringLiteral("Motor peak (rpm)"), nullptr, 19},
                 {QStringLiteral("Step rate (kHz)"), nullptr, 18},
                 {QStringLiteral("Cycle"), nullptr, 9},
                 {QStringLiteral("Cycle unit"), nullptr, 13},
                 {QStringLiteral("Profile"), nullptr, 11}});
        r = r0 + static_cast<int>(project.axes.size()) + 2;
        b.Str(ws, r++, 1, QStringLiteral("Velocity and acceleration are per second at the master speed (crank rpm, or the timeline "
                                         "speed override); units are each axis's own."),
              b.note());
        ++r;
    }

    // Master in brief, and where to look next.
    b.Str(ws, r, 1, QStringLiteral("Master"), b.h2());
    for (int c = 2; c <= 4; ++c) worksheet_write_blank(ws, r, c, b.h2());
    ++r;
    r = WriteKvTable(b, ws, r, 1, MasterRows(project, pi), false);

    b.Str(ws, r, 1, QStringLiteral("Sheets"), b.h2());
    for (int c = 2; c <= 4; ++c) worksheet_write_blank(ws, r, c, b.h2());
    ++r;
    b.Link(ws, r, 1, master, QStringLiteral("Master"));
    b.Str(ws, r++, 2, QStringLiteral("master settings and output pulses"));
    if (timing) {
        b.Link(ws, r, 1, timing, QStringLiteral("Timing"));
        b.Str(ws, r++, 2, QStringLiteral("every axis on one master coordinate, output lanes"));
    }
    for (size_t i = 0; i < project.axes.size(); ++i) {
        const QString n = Q(project.axes[i].name);
        b.Link(ws, r, 1, sheets[i].profile, n + QStringLiteral(" profile"));
        b.Str(ws, r++, 2, QStringLiteral("settings, segments, analysis"));
        b.Link(ws, r, 1, sheets[i].motion, n + QStringLiteral(" motion"));
        b.Str(ws, r++, 2, QStringLiteral("sampled motion table and charts"));
        if (sheets[i].cam) {
            b.Link(ws, r, 1, sheets[i].cam, n + QStringLiteral(" cam"));
            b.Str(ws, r++, 2, QStringLiteral("plate cam outline and pressure angle"));
        }
    }
}

void WriteMaster(Book &b, lxw_worksheet *ws, const CamProject &project, const ProjectInfo &pi) {
    worksheet_gridlines(ws, LXW_HIDE_ALL_GRIDLINES);
    b.Str(ws, 0, 0, QStringLiteral("Master"), b.title());
    worksheet_set_row(ws, 0, 28, nullptr);
    int r = 2;
    r = WriteKvTable(b, ws, r, 0, MasterRows(project, pi), true);
    ++r;
    b.Str(ws, r, 0, QStringLiteral("Output pulses"), b.h2());
    for (int c = 1; c <= 5; ++c) worksheet_write_blank(ws, r, c, b.h2());
    ++r;
    if (project.outputs.empty()) {
        b.Str(ws, r, 0, QStringLiteral("No output pulses."), b.note());
        return;
    }
    const int r0 = r;
    for (size_t i = 0; i < project.outputs.size(); ++i) {
        const OutputPulse &p = project.outputs[i];
        const int rr = r0 + 1 + static_cast<int>(i);
        const bool crank = p.master == AxisMaster::Crank;
        const char *f = crank ? kFmtDeg : kFmtSec;
        b.Str(ws, rr, 0, QStringLiteral("IO-%1").arg(p.output));
        b.Str(ws, rr, 1, crank ? QStringLiteral("Crank") : QStringLiteral("Timeline"));
        b.Num(ws, rr, 2, p.on, f);
        b.Num(ws, rr, 3, p.off, f);
        b.Num(ws, rr, 4, p.off - p.on, f);
        b.Str(ws, rr, 5, crank ? QStringLiteral("°") : QStringLiteral("s"));
    }
    b.Table(ws, r0, 0, static_cast<int>(project.outputs.size()),
            {{QStringLiteral("Output"), nullptr, 30},
             {QStringLiteral("Master"), nullptr, 16},
             {QStringLiteral("On"), nullptr, 10},
             {QStringLiteral("Off"), nullptr, 10},
             {QStringLiteral("Length"), nullptr, 10},
             {QStringLiteral("Unit "), nullptr, 8}});
}

void WriteProfile(Book &b, lxw_worksheet *ws, const CamProject &project, int axis, const AxisInfo &ai,
                  const ProjectInfo &pi, lxw_worksheet *summary) {
    const CamAxis &a = project.axes[axis];
    const QString u = Q(a.units), xu = XUnit(a);
    const char *xf = XFmt(a);
    const double w = ai.w;
    worksheet_gridlines(ws, LXW_HIDE_ALL_GRIDLINES);
    b.Str(ws, 0, 0, Q(a.name), b.title());
    worksheet_set_row(ws, 0, 28, nullptr);
    b.Link(ws, 1, 0, summary, QStringLiteral("← Summary"));

    // Settings
    int r = 3;
    b.Str(ws, r, 0, QStringLiteral("Settings"), b.h2());
    for (int c = 1; c <= 2; ++c) worksheet_write_blank(ws, r, c, b.h2());
    ++r;
    r = WriteKvTable(b, ws, r, 0, AxisSettingsRows(a), false);
    ++r;

    // Segments
    b.Str(ws, r, 0, QStringLiteral("Segments"), b.h2());
    for (int c = 1; c <= 10; ++c) worksheet_write_blank(ws, r, c, b.h2());
    ++r;
    const int n = static_cast<int>(a.segments.size());
    const int r0 = r;
    for (int s = 0; s < n; ++s) {
        const int rr = r0 + 1 + s;
        const Peaks pk = SegmentPeaks(a, s);
        const double x0 = SegmentStartDeg(a, s), x1 = a.segments[s].endDeg;
        const double p0 = SegmentStartPos(a, s), p1 = a.segments[s].endPos;
        b.Num(ws, rr, 0, s + 1, kFmtInt);
        b.Num(ws, rr, 1, x0, xf);
        b.Num(ws, rr, 2, x1, xf);
        b.Num(ws, rr, 3, x1 - x0, xf);
        b.Num(ws, rr, 4, p0, kFmtPos);
        b.Num(ws, rr, 5, p1, kFmtPos);
        b.Num(ws, rr, 6, p1 - p0, kFmtPos);
        b.Str(ws, rr, 7, LawText(a.segments[s].law));
        b.Num(ws, rr, 8, pk.vel * w, kFmtRate);
        b.Num(ws, rr, 9, pk.acc * w * w, kFmtRate);
        b.Num(ws, rr, 10, pk.jerk * w * w * w, kFmtRate);
    }
    b.Table(ws, r0, 0, std::max(n, 1),
            {{QStringLiteral("Segment"), nullptr, 30},
             {QStringLiteral("Start (%1)").arg(xu), nullptr, 16},
             {QStringLiteral("End (%1)").arg(xu), nullptr, 12},
             {QStringLiteral("Length (%1)").arg(xu), nullptr, 14},
             {QStringLiteral("From (%1)").arg(u), nullptr, 15},
             {QStringLiteral("To (%1)").arg(u), nullptr, 14},
             {QStringLiteral("Move (%1)").arg(u), nullptr, 15},
             {QStringLiteral("Motion law"), nullptr, 20},
             {QStringLiteral("Peak vel (%1/s)").arg(u), nullptr, 20},
             {QStringLiteral("Peak acc (%1/s²)").arg(u), nullptr, 21},
             {QStringLiteral("Peak jerk (%1/s³)").arg(u), nullptr, 21}});
    r = r0 + std::max(n, 1) + 2;
    b.Str(ws, r++, 0, OnTimeline(a) ? QStringLiteral("Per-second values at the speed override (%1).")
                                          .arg(QString::number(project.master.speedOverride * 100.0, 'f', 1) + QStringLiteral(" %"))
                                    : QStringLiteral("Per-second values at %1 rpm.").arg(Sig(project.master.rpm)),
          b.note());
    ++r;

    // Analysis
    b.Str(ws, r, 0, QStringLiteral("Analysis"), b.h2());
    for (int c = 1; c <= 3; ++c) worksheet_write_blank(ws, r, c, b.h2());
    ++r;
    const std::vector<StatRow> stats = AxisStatRows(project, axis, ai);
    const int s0 = r;
    for (size_t i = 0; i < stats.size(); ++i) {
        const int rr = s0 + 1 + static_cast<int>(i);
        b.Str(ws, rr, 0, stats[i].key);
        if (std::isfinite(stats[i].v)) {
            b.Num(ws, rr, 1, stats[i].v, ExcelFmt(stats[i].fmt));
        } else {
            b.Str(ws, rr, 1, QStringLiteral("no limit"), b.text());
        }
        b.Str(ws, rr, 2, stats[i].unit);
        b.Str(ws, rr, 3, stats[i].at);
    }
    b.Table(ws, s0, 0, static_cast<int>(stats.size()),
            {{QStringLiteral("Measure"), nullptr, 30}, {QStringLiteral("Value"), nullptr, 16}, {QStringLiteral("Unit"), nullptr, 12},
             {QStringLiteral("At"), nullptr, 12}});
    r = s0 + static_cast<int>(stats.size()) + 2;

    // This axis's checks
    std::vector<const Check *> mine;
    for (const Check &c : pi.checks) {
        if (c.where == Q(a.name)) mine.push_back(&c);
    }
    if (mine.empty()) {
        b.Str(ws, r, 0, QStringLiteral("✓ Smooth: velocity and acceleration are continuous at every knot."), b.good());
    } else {
        for (const Check *c : mine) {
            b.Str(ws, r, 0, c->level == Check::Error ? QStringLiteral("⚠ Error") : QStringLiteral("⚠ Warning"),
                  c->level == Check::Error ? b.error() : b.warning());
            b.Str(ws, r++, 1, c->text);
        }
    }
}

void WriteMotion(Book &b, lxw_worksheet *ws, const CamProject &project, int axis, const AxisInfo &ai, int samples,
                 lxw_worksheet *profile) {
    const CamAxis &a = project.axes[axis];
    const bool timeline = OnTimeline(a);
    const QString u = Q(a.units);
    const double w = ai.w;
    // Header block, then the table from row 3.
    b.Str(ws, 0, 0, Q(a.name) + QStringLiteral(" — motion"), b.title());
    worksheet_set_row(ws, 0, 28, nullptr);
    b.Link(ws, 1, 0, profile, QStringLiteral("← Profile"));
    b.Str(ws, 1, 2, timeline ? QStringLiteral("%1 samples over %2 s (as drawn); per second at the speed override %3 %.")
                                   .arg(samples).arg(Sig(a.span)).arg(QString::number(project.master.speedOverride * 100.0, 'f', 1))
                             : QStringLiteral("%1 samples per revolution; per second at %2 rpm.").arg(samples).arg(Sig(project.master.rpm)),
          b.note());
    const int h = 3;  // header row of the table
    const int first = h + 1, last = h + 1 + samples;
    for (int i = 0; i <= samples; ++i) {
        const CamPoint p = SampleAt(a, i, samples);
        const int r = first + i;
        const double x = a.span * i / samples;
        b.Num(ws, r, 0, x, XFmt(a));
        b.Num(ws, r, 1,
              timeline ? static_cast<double>(std::llround(x * 5000.0))
                       : static_cast<double>(std::llround(project.master.countsPerRev * i / samples)),
              kFmtInt);
        b.Num(ws, r, 2, p.pos, kFmtPos);
        b.Num(ws, r, 3, static_cast<double>(std::llround(p.pos * a.countsPerUnit)), kFmtInt);
        b.Num(ws, r, 4, p.vel * w, kFmtRate);
        b.Num(ws, r, 5, p.acc * w * w, kFmtRate);
        b.Num(ws, r, 6, p.jerk * w * w * w, kFmtRate);
        b.Num(ws, r, 7, p.segment + 1, kFmtInt);
    }
    b.Table(ws, h, 0, samples + 1,
            {{XHeader(a), nullptr, 11},
             {timeline ? QStringLiteral("Ticks (5 kHz)") : QStringLiteral("Master counts"), nullptr, 14},
             {QStringLiteral("Position (%1)").arg(u), nullptr, 14},
             {QStringLiteral("Motor counts"), nullptr, 13},
             {QStringLiteral("Velocity (%1/s)").arg(u), nullptr, 15},
             {QStringLiteral("Accel (%1/s²)").arg(u), nullptr, 15},
             {QStringLiteral("Jerk (%1/s³)").arg(u), nullptr, 15},
             {QStringLiteral("Segment"), nullptr, 9}},
            9);
    worksheet_freeze_panes(ws, h + 1, 0);

    // Four charts stacked to the right, one quantity each (one y scale per chart).
    const QString xName = timeline ? QStringLiteral("Time (s, as drawn)") : QStringLiteral("Crank angle (°)");
    struct Q4 {
        QString title, yName;
        int col;
    };
    const Q4 qs[] = {{QStringLiteral("Position"), QStringLiteral("Position (%1)").arg(u), 2},
                     {QStringLiteral("Velocity"), QStringLiteral("%1/s").arg(u), 4},
                     {QStringLiteral("Acceleration"), QStringLiteral("%1/s²").arg(u), 5},
                     {QStringLiteral("Jerk"), QStringLiteral("%1/s³").arg(u), 6}};
    const double cw = 760, chh = 260;
    int row = 1;
    for (int k = 0; k < 4; ++k) {
        lxw_chart *ch = b.Chart(Q(a.name) + QStringLiteral(" — ") + qs[k].title, xName, qs[k].yName);
        b.Series(ch, ws, 0, qs[k].col, first, last, qs[k].title, SeriesColor(k));
        chart_axis_set_min(ch->x_axis, 0.0);
        chart_axis_set_max(ch->x_axis, a.span);
        if (!timeline) chart_axis_set_major_unit(ch->x_axis, 45.0);
        b.Insert(ws, row, 9, ch, cw, chh);
        row += ChartRows(chh);
    }
}

void WriteCam(Book &b, lxw_worksheet *ws, const CamAxis &a, int samples, lxw_worksheet *profile, int axisIndex) {
    const QString u = Q(a.units);
    const CamShape shape = BuildCamShape(a, samples);
    b.Str(ws, 0, 0, Q(a.name) + QStringLiteral(" — plate cam"), b.title());
    worksheet_set_row(ws, 0, 28, nullptr);
    b.Link(ws, 1, 0, profile, QStringLiteral("← Profile"));
    b.Str(ws, 1, 2,
          QStringLiteral("Radial translating roller follower: base radius %1 %2, roller %3 %2. Follower at 12 o'clock at 0°, "
                         "cam turning clockwise.")
              .arg(Sig(a.baseRadius), u, Sig(a.rollerRadius)),
          b.note());
    const int h = 3, first = h + 1, last = h + 1 + samples;
    const double rp = a.baseRadius + a.rollerRadius;
    constexpr double kDegPerRad = 180.0 / 3.14159265358979323846;
    for (int i = 0; i <= samples; ++i) {
        const int r = first + i;
        const double deg = 360.0 * i / samples;
        const CamPoint p = SampleAt(a, i, samples);
        const double R = rp + (p.pos - shape.minPos);
        const double pa = std::atan2(p.vel * kDegPerRad, R) * kDegPerRad;
        const double th = (90.0 + deg) / kDegPerRad;
        b.Num(ws, r, 0, deg, kFmtDeg);
        b.Num(ws, r, 1, p.pos, kFmtPos);
        b.Num(ws, r, 2, R, kFmtPos);
        b.Num(ws, r, 3, pa, "0.00");
        const size_t k = static_cast<size_t>(std::min<int>(i, static_cast<int>(shape.pitch.size()) - 1));
        b.Num(ws, r, 4, shape.surface[k].x, kFmtPos);
        b.Num(ws, r, 5, shape.surface[k].y, kFmtPos);
        b.Num(ws, r, 6, shape.pitch[k].x, kFmtPos);
        b.Num(ws, r, 7, shape.pitch[k].y, kFmtPos);
        b.Num(ws, r, 8, a.baseRadius * std::cos(th), kFmtPos);
        b.Num(ws, r, 9, a.baseRadius * std::sin(th), kFmtPos);
    }
    b.Table(ws, h, 0, samples + 1,
            {{QStringLiteral("Crank (°)"), nullptr, 10},
             {QStringLiteral("Lift (%1)").arg(u), nullptr, 12},
             {QStringLiteral("Pitch radius (%1)").arg(u), nullptr, 17},
             {QStringLiteral("Pressure angle (°)"), nullptr, 18},
             {QStringLiteral("Cam X"), nullptr, 11},
             {QStringLiteral("Cam Y"), nullptr, 11},
             {QStringLiteral("Pitch X"), nullptr, 11},
             {QStringLiteral("Pitch Y"), nullptr, 11},
             {QStringLiteral("Base X"), nullptr, 11},
             {QStringLiteral("Base Y"), nullptr, 11}},
            9);
    worksheet_freeze_panes(ws, h + 1, 0);

    // Pressure angle limit lines (two points each), out of the way to the right of the table.
    const int lc = 11;
    b.Str(ws, h, lc, QStringLiteral("Limit (°)"), b.note());
    b.Str(ws, h, lc + 1, QStringLiteral("+"), b.note());
    b.Str(ws, h, lc + 2, QStringLiteral("−"), b.note());
    for (int i = 0; i < 2; ++i) {
        b.Num(ws, first + i, lc, 360.0 * i, kFmtDeg);
        b.Num(ws, first + i, lc + 1, kMaxPressureAngleDeg, kFmtDeg);
        b.Num(ws, first + i, lc + 2, -kMaxPressureAngleDeg, kFmtDeg);
    }

    // Outline: equal scales on both axes in a square chart so the cam is not distorted.
    double extent = 0.0;
    for (const Pt &p : shape.pitch) extent = std::max({extent, std::fabs(p.x), std::fabs(p.y)});
    extent = NiceCeil(extent * 1.05);
    lxw_chart *outline = b.Chart(Q(a.name) + QStringLiteral(" — cam outline"), QStringLiteral("X (%1)").arg(u),
                                 QStringLiteral("Y (%1)").arg(u));
    b.Series(outline, ws, 4, 5, first, last, QStringLiteral("Cam profile"), SeriesColor(axisIndex), 2.25f);
    b.Series(outline, ws, 6, 7, first, last, QStringLiteral("Pitch curve (roller centre)"), kMuted, 1.0f,
             LXW_CHART_LINE_DASH_DASH);
    b.Series(outline, ws, 8, 9, first, last, QStringLiteral("Base circle"), kRule, 1.0f, LXW_CHART_LINE_DASH_ROUND_DOT);
    for (lxw_chart_axis *ax : {outline->x_axis, outline->y_axis}) {
        chart_axis_set_min(ax, -extent);
        chart_axis_set_max(ax, extent);
        chart_axis_set_major_unit(ax, extent / 4.0);
    }
    chart_legend_set_position(outline, LXW_CHART_LEGEND_BOTTOM);
    b.Insert(ws, 1, 15, outline, 560, 600);

    lxw_chart *press = b.Chart(Q(a.name) + QStringLiteral(" — pressure angle"), QStringLiteral("Crank angle (°)"),
                               QStringLiteral("Pressure angle (°)"));
    b.Series(press, ws, 0, 3, first, last, QStringLiteral("Pressure angle"), SeriesColor(axisIndex));
    b.Series(press, ws, lc, lc + 1, first, first + 1, QStringLiteral("Limit ±%1°").arg(kMaxPressureAngleDeg, 0, 'f', 0), kInk2, 1.0f,
             LXW_CHART_LINE_DASH_DASH);
    b.Series(press, ws, lc, lc + 2, first, first + 1, QStringLiteral("Limit −"), kInk2, 1.0f, LXW_CHART_LINE_DASH_DASH);
    int16_t hidden[] = {2, -1};  // one legend entry for both limit lines
    chart_legend_delete_series(press, hidden);
    chart_legend_set_position(press, LXW_CHART_LEGEND_BOTTOM);
    chart_axis_set_min(press->x_axis, 0.0);
    chart_axis_set_max(press->x_axis, 360.0);
    chart_axis_set_major_unit(press->x_axis, 45.0);
    b.Insert(ws, 1 + ChartRows(600), 15, press, 560, 280);
}

// Step points for one output's pulses on [0, end]: (x, level) pairs at each edge.
std::vector<std::pair<double, double>> Lane(const std::vector<const OutputPulse *> &pulses, double end, double base) {
    std::vector<std::pair<double, double>> pts;
    const double hi = base + 0.8;
    base += 0.1;
    pts.push_back({0.0, base});
    for (const OutputPulse *p : pulses) {
        pts.push_back({p->on, pts.back().second});
        pts.push_back({p->on, hi});
        pts.push_back({p->off, hi});
        pts.push_back({p->off, base});
    }
    pts.push_back({std::max(end, pts.back().first), base});
    return pts;
}

lxw_worksheet *WriteTiming(Book &b, const CamProject &project, int samples, lxw_worksheet *summary) {
    std::vector<int> crankAxes, timeAxes;
    for (size_t i = 0; i < project.axes.size(); ++i) {
        (project.axes[i].master == AxisMaster::Crank ? crankAxes : timeAxes).push_back(static_cast<int>(i));
    }
    bool crankOut = false, timeOut = false;
    for (const OutputPulse &p : project.outputs) (p.master == AxisMaster::Crank ? crankOut : timeOut) = true;
    if (crankAxes.empty() && timeAxes.empty() && project.outputs.empty()) return nullptr;

    lxw_worksheet *ws = b.AddSheet(QStringLiteral("Timing"), 0x6250D6);
    b.Str(ws, 0, 0, QStringLiteral("Timing"), b.title());
    worksheet_set_row(ws, 0, 28, nullptr);
    b.Link(ws, 1, 0, summary, QStringLiteral("← Summary"));
    b.Str(ws, 1, 2, QStringLiteral("Every axis on one master coordinate, as the timing view draws them. Data below the charts."),
          b.note());

    // Charts on top, two per row; data starts below them.
    const double cw = 640, chh = 300;
    const int chartRow = 3;
    std::vector<lxw_chart *> charts;
    const int nCharts = (crankAxes.empty() ? 0 : 1) + (timeAxes.empty() ? 0 : 1) + (crankOut ? 1 : 0) + (timeOut ? 1 : 0);
    const int dataRow = chartRow + ((nCharts + 1) / 2) * ChartRows(chh) + 2;

    int col = 0;
    auto place = [&](lxw_chart *ch) {
        const int k = static_cast<int>(charts.size());
        charts.push_back(ch);
        // 640 px is about 10 default columns (64 px each).
        b.Insert(ws, chartRow + (k / 2) * ChartRows(chh), (k % 2) * 11, ch, cw, chh);
    };

    // Crank axes: one row per sample, a column per axis.
    if (!crankAxes.empty()) {
        const int h = dataRow, first = h + 1, last = h + 1 + samples;
        b.Str(ws, h - 1, col, QStringLiteral("Crank axes"), b.h2());
        std::vector<Col> cols = {{QStringLiteral("Crank (°)"), nullptr, 10}};
        for (int ai : crankAxes) cols.push_back({Q(project.axes[ai].name) + QStringLiteral(" (") + Q(project.axes[ai].units) + QStringLiteral(")"), nullptr, 14});
        for (int i = 0; i <= samples; ++i) {
            b.Num(ws, first + i, col, 360.0 * i / samples, kFmtDeg);
            for (size_t k = 0; k < crankAxes.size(); ++k) {
                b.Num(ws, first + i, col + 1 + static_cast<int>(k), SampleAt(project.axes[crankAxes[k]], i, samples).pos, kFmtPos);
            }
        }
        b.Table(ws, h, col, samples + 1, cols, 9);
        lxw_chart *ch = b.Chart(QStringLiteral("Crank axes"), QStringLiteral("Crank angle (°)"), QStringLiteral("Position (axis units)"));
        for (size_t k = 0; k < crankAxes.size(); ++k) {
            b.Series(ch, ws, col, col + 1 + static_cast<int>(k), first, last, Q(project.axes[crankAxes[k]].name),
                     SeriesColor(crankAxes[k]));
        }
        chart_axis_set_min(ch->x_axis, 0.0);
        chart_axis_set_max(ch->x_axis, 360.0);
        chart_axis_set_major_unit(ch->x_axis, 45.0);
        if (crankAxes.size() > 1) chart_legend_set_position(ch, LXW_CHART_LEGEND_BOTTOM);
        place(ch);
        col += static_cast<int>(cols.size()) + 1;
    }

    // Timeline axes on the shared timeline.
    const double length = std::max(TimelineLength(project), 1e-6);
    if (!timeAxes.empty()) {
        const int h = dataRow, first = h + 1, last = h + 1 + samples;
        b.Str(ws, h - 1, col, QStringLiteral("Timeline axes"), b.h2());
        std::vector<Col> cols = {{QStringLiteral("Time (s)"), nullptr, 10}};
        for (int ai : timeAxes) cols.push_back({Q(project.axes[ai].name) + QStringLiteral(" (") + Q(project.axes[ai].units) + QStringLiteral(") "), nullptr, 14});
        for (int i = 0; i <= samples; ++i) {
            const double t = length * i / samples;
            b.Num(ws, first + i, col, t, kFmtSec);
            for (size_t k = 0; k < timeAxes.size(); ++k) {
                b.Num(ws, first + i, col + 1 + static_cast<int>(k), TimelineAt(project.axes[timeAxes[k]], t).pos, kFmtPos);
            }
        }
        b.Table(ws, h, col, samples + 1, cols, 9);
        lxw_chart *ch = b.Chart(QStringLiteral("Timeline axes"), QStringLiteral("Time (s, as drawn)"), QStringLiteral("Position (axis units)"));
        for (size_t k = 0; k < timeAxes.size(); ++k) {
            b.Series(ch, ws, col, col + 1 + static_cast<int>(k), first, last, Q(project.axes[timeAxes[k]].name),
                     SeriesColor(timeAxes[k]));
        }
        chart_axis_set_min(ch->x_axis, 0.0);
        chart_axis_set_max(ch->x_axis, length);
        if (timeAxes.size() > 1) chart_legend_set_position(ch, LXW_CHART_LEGEND_BOTTOM);
        place(ch);
        col += static_cast<int>(cols.size()) + 1;
    }

    // Output lanes: IO-n drawn at height n, high while on.
    for (int pass = 0; pass < 2; ++pass) {
        const AxisMaster m = pass == 0 ? AxisMaster::Crank : AxisMaster::Timeline;
        if ((m == AxisMaster::Crank && !crankOut) || (m == AxisMaster::Timeline && !timeOut)) continue;
        const double end = m == AxisMaster::Crank ? 360.0 : length;
        const char *xf = m == AxisMaster::Crank ? kFmtDeg : kFmtSec;
        lxw_chart *ch = b.Chart(m == AxisMaster::Crank ? QStringLiteral("Crank outputs") : QStringLiteral("Timeline outputs"),
                                m == AxisMaster::Crank ? QStringLiteral("Crank angle (°)") : QStringLiteral("Time (s, as drawn)"),
                                QStringLiteral("Output"));
        const int h = dataRow;
        b.Str(ws, h - 1, col, m == AxisMaster::Crank ? QStringLiteral("Crank output lanes") : QStringLiteral("Timeline output lanes"),
              b.h2());
        int lanes = 0, top = 0;
        for (int o = 0; o < kOutputs; ++o) {
            std::vector<const OutputPulse *> mine;
            for (const OutputPulse &p : project.outputs) {
                if (p.output == o && p.master == m) mine.push_back(&p);
            }
            if (mine.empty()) continue;
            const auto pts = Lane(mine, end, o);
            b.Str(ws, h, col, QStringLiteral("IO-%1 x").arg(o));
            b.Str(ws, h, col + 1, QStringLiteral("IO-%1").arg(o));
            for (size_t i = 0; i < pts.size(); ++i) {
                b.Num(ws, h + 1 + static_cast<int>(i), col, pts[i].first, xf);
                b.Num(ws, h + 1 + static_cast<int>(i), col + 1, pts[i].second, "0.0");
            }
            b.Series(ch, ws, col, col + 1, h + 1, h + static_cast<int>(pts.size()), QStringLiteral("IO-%1").arg(o), SeriesColor(lanes), 2.0f);
            worksheet_set_column(ws, col, col + 1, 9, nullptr);
            col += 2;
            ++lanes;
            top = o;
        }
        ++col;
        chart_axis_set_min(ch->x_axis, 0.0);
        chart_axis_set_max(ch->x_axis, end);
        if (m == AxisMaster::Crank) chart_axis_set_major_unit(ch->x_axis, 45.0);
        // Lane n spans n .. n + 1, labelled at its bottom gridline.
        chart_axis_set_min(ch->y_axis, 0.0);
        chart_axis_set_max(ch->y_axis, top + 1.0);
        chart_axis_set_major_unit(ch->y_axis, 1.0);
        chart_axis_set_num_format(ch->y_axis, "\"IO-\"0");
        if (lanes > 1) chart_legend_set_position(ch, LXW_CHART_LEGEND_BOTTOM);
        place(ch);
    }
    return ws;
}

}  // namespace

bool ExportWorkbook(const CamProject &project, const WorkbookOptions &options, const QString &path, QString *error) {
    const int samples = std::clamp(options.samples, 8, 100000);
    // Write next to the target, then swap it in: a failed export never leaves half a workbook, and a
    // workbook open in Excel (locked) is reported instead of silently kept.
    const QString tmp = path + QStringLiteral(".part");
    QFile::remove(tmp);

    lxw_workbook_options wo{};
    wo.use_zip64 = LXW_TRUE;
    const std::string tmpPath = U8(QDir::toNativeSeparators(tmp));
    lxw_workbook *wb = workbook_new_opt(tmpPath.c_str(), &wo);
    if (!wb) {
        if (error) *error = QStringLiteral("could not create the workbook");
        return false;
    }
    Book b(wb);

    const QString title = options.projectName.isEmpty() ? QStringLiteral("Untitled") : options.projectName;
    const std::string titleU8 = U8(title);
    lxw_doc_properties props{};
    props.title = titleU8.c_str();
    props.subject = "Electronic cam project";
    props.comments = "Exported by the " ECAM_PRODUCT_NAME;
    props.keywords = "electronic cam, ClearCore, motion profile";
    props.created = std::time(nullptr);
    workbook_set_properties(wb, &props);

    const ProjectInfo pi = AnalyzeProject(project);

    // Sheet order: Summary, Master, Timing, then each axis's Profile / Motion / Cam.
    lxw_worksheet *summary = b.AddSheet(QStringLiteral("Summary"), kTabSummary);
    lxw_worksheet *master = b.AddSheet(QStringLiteral("Master"), kTabSummary);
    lxw_worksheet *timing = WriteTiming(b, project, samples, summary);
    std::vector<AxisSheets> sheets(project.axes.size());
    for (size_t i = 0; i < project.axes.size(); ++i) {
        const CamAxis &a = project.axes[i];
        const QString n = Q(a.name).left(20);
        const lxw_color_t tab = SeriesColor(static_cast<int>(i));
        sheets[i].profile = b.AddSheet(n + QStringLiteral(" Profile"), tab);
        sheets[i].motion = b.AddSheet(n + QStringLiteral(" Motion"), tab);
        if (a.master == AxisMaster::Crank && !a.segments.empty()) sheets[i].cam = b.AddSheet(n + QStringLiteral(" Cam"), tab);
    }

    WriteSummary(b, summary, project, pi, options, sheets, master, timing);
    WriteMaster(b, master, project, pi);
    for (size_t i = 0; i < project.axes.size(); ++i) {
        if (project.axes[i].segments.empty()) continue;
        WriteProfile(b, sheets[i].profile, project, static_cast<int>(i), pi.axes[i], pi, summary);
        WriteMotion(b, sheets[i].motion, project, static_cast<int>(i), pi.axes[i], samples, sheets[i].profile);
        if (sheets[i].cam) WriteCam(b, sheets[i].cam, project.axes[i], samples, sheets[i].profile, static_cast<int>(i));
    }
    worksheet_activate(summary);

    const lxw_error err = workbook_close(wb);
    if (err != LXW_NO_ERROR) {
        QFile::remove(tmp);
        if (error) *error = QString::fromUtf8(lxw_strerror(err));
        return false;
    }
    if (QFile::exists(path) && !QFile::remove(path)) {
        QFile::remove(tmp);
        if (error) *error = QStringLiteral("the file is in use (is it open in Excel?)");
        return false;
    }
    if (!QFile::rename(tmp, path)) {
        QFile::remove(tmp);
        if (error) *error = QStringLiteral("could not replace the file");
        return false;
    }
    return true;
}

}  // namespace io
