/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "ProjectReport.h"

#include "Brand.h"
#include "CamGeometry.h"
#include "ProjectReview.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFontMetricsF>
#include <QLocale>
#include <QPageSize>
#include <QPainter>
#include <QPainterPath>
#include <QPdfWriter>
#include <QSvgRenderer>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>

using namespace ecam;
using namespace review;

namespace io {

namespace {

// ---- Look ----------------------------------------------------------------------------------------
// Everything is in points (1/72 in): the PDF is written at 72 units per inch, and the preview images
// are scaled from the same coordinates, so both lay out identically.

const QColor kInk(0x1d, 0x1f, 0x23);
const QColor kInk2(0x4a, 0x4f, 0x57);
const QColor kMuted(0x7d, 0x84, 0x8d);
const QColor kRule(0xc8, 0xcd, 0xd4);
const QColor kGrid(0xe6, 0xe9, 0xed);
const QColor kAccent(0x17, 0x37, 0x5e);
const QColor kAccentSoft(0xe9, 0xef, 0xf6);
const QColor kZebra(0xf6, 0xf7, 0xf9);
const QColor kError(0xb3, 0x26, 0x1e);
const QColor kWarn(0x9a, 0x62, 0x00);
const QColor kGood(0x1e, 0x6b, 0x34);
const QColor kCrank(0x2a, 0x78, 0xd6);
const QColor kTimeline(0xc9, 0x6f, 0x00);
// Categorical colors, fixed order (the workbook's): axis i always wears slot i.
const QRgb kSeries[] = {0x2A78D6, 0xEB6834, 0x1BAF7A, 0xEDA100, 0xE87BA4, 0x008300, 0x6250D6, 0xE34948};
const QRgb kOutputColors[] = {0xD9534F, 0xE08E0B, 0x2E9E5B, 0x2A78D6, 0x7B5CC9, 0x8A6D3B};

QColor AxisColor(int i) { return QColor::fromRgb(kSeries[((i % 8) + 8) % 8]); }
QColor OutputColor(int k) { return QColor::fromRgb(kOutputColors[((k % 6) + 6) % 6]); }
QColor Tint(const QColor &c, double a) {  // c over white at alpha a
    return QColor::fromRgbF(1.0 - a * (1.0 - c.redF()), 1.0 - a * (1.0 - c.greenF()), 1.0 - a * (1.0 - c.blueF()));
}

QFont F(double pt, QFont::Weight weight = QFont::Normal, bool italic = false) {
    QFont f(QStringLiteral("Segoe UI"));
    f.setPointSizeF(pt);
    f.setWeight(weight);
    f.setItalic(italic);
    f.setHintingPreference(QFont::PreferNoHinting);  // the same metrics in the PDF and the preview
    f.setKerning(true);
    return f;
}

double NiceStep(double span, int target) {
    if (!(span > 0.0)) return 1.0;
    const double raw = span / target;
    const double mag = std::pow(10.0, std::floor(std::log10(raw)));
    const double n = raw / mag;
    return (n < 1.5 ? 1.0 : n < 3.0 ? 2.0 : n < 7.0 ? 5.0 : 10.0) * mag;
}

int StepDecimals(double step) {
    int d = 0;
    while (d < 6 && std::fabs(step * std::pow(10.0, d) - std::round(step * std::pow(10.0, d))) > 1e-6) ++d;
    return d;
}

QString Num(double v, int decimals) {
    if (!std::isfinite(v)) return QStringLiteral("—");
    if (std::fabs(v) < 0.5 * std::pow(10.0, -decimals)) v = 0.0;  // no "-0.00"
    QLocale c = QLocale::c();
    c.setNumberOptions(QLocale::DefaultNumberOptions);
    return c.toString(v, 'f', decimals);
}

QString XText(const CamAxis &a, double x) {
    return OnTimeline(a) ? Num(x, 3) + QStringLiteral(" s") : Num(x, 1) + QStringLiteral("°");
}

// ---- Pages ---------------------------------------------------------------------------------------

class Sink {
public:
    virtual ~Sink() = default;
    virtual QPainter *Begin() = 0;  // the next page
    virtual void End() = 0;         // that page is done
    virtual bool Finish() { return true; }
};

class PdfSink : public Sink {
public:
    PdfSink(const QString &path, const QPageSize &size, const QString &title) : m_writer(path) {
        m_writer.setResolution(72);
        m_writer.setPageSize(size);
        m_writer.setPageMargins(QMarginsF(0, 0, 0, 0));
        m_writer.setTitle(title);
        m_writer.setCreator(QStringLiteral(ECAM_PRODUCT_NAME));
    }
    QPainter *Begin() override {
        if (!m_started) {
            if (!m_painter.begin(&m_writer)) return nullptr;
            m_started = true;
        } else {
            m_writer.newPage();
        }
        m_painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing | QPainter::SmoothPixmapTransform);
        return &m_painter;
    }
    void End() override {}
    bool Finish() override { return m_started && m_painter.end(); }

private:
    QPdfWriter m_writer;
    QPainter m_painter;
    bool m_started = false;
};

// Pages as images: drawn in points, scaled to dpi. `keep` false lays out only (the first pass).
class ImageSink : public Sink {
public:
    ImageSink(QSizeF pagePt, double dpi, bool keep) : m_page(pagePt), m_dpi(dpi), m_keep(keep) {}
    QPainter *Begin() override {
        const QSize px = m_keep ? QSize(qRound(m_page.width() * m_dpi / 72.0), qRound(m_page.height() * m_dpi / 72.0)) : QSize(8, 8);
        m_current = QImage(px, QImage::Format_RGB32);
        // 72 logical dpi: a point is a unit, as on the PDF writer.
        m_current.setDotsPerMeterX(qRound(72.0 / 0.0254));
        m_current.setDotsPerMeterY(qRound(72.0 / 0.0254));
        m_current.fill(Qt::white);
        m_painter.begin(&m_current);
        m_painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing | QPainter::SmoothPixmapTransform);
        if (m_keep) m_painter.scale(m_dpi / 72.0, m_dpi / 72.0);
        return &m_painter;
    }
    void End() override {
        m_painter.end();
        if (m_keep) m_pages.push_back(m_current);
    }
    std::vector<QImage> &pages() { return m_pages; }

private:
    QSizeF m_page;
    double m_dpi;
    bool m_keep;
    QImage m_current;
    QPainter m_painter;
    std::vector<QImage> m_pages;
};

// ---- Charts --------------------------------------------------------------------------------------

struct Band {
    double x0, x1;
    QColor fill;
    QString label;
};
struct Series {
    std::vector<QPointF> pts;
    QColor color;
    double width = 1.2;
    Qt::PenStyle style = Qt::SolidLine;
};
struct Ref {
    double y;
    QString label;
    QColor color;
};
struct Mark {
    double x, y;
    QString label;
    QColor color;
};
struct Chart {
    QString title;
    QString unit;
    double x0 = 0.0, x1 = 360.0;
    bool crank = true;
    QString xTitle;
    std::vector<Band> bands;
    std::vector<Series> series;
    std::vector<Ref> refs;
    std::vector<Mark> marks;
    std::vector<QPointF> dots;  // knots
    QColor dotColor;
    std::vector<double> vlines;
    bool fixedY = false;
    double yLo = 0.0, yHi = 1.0;
};

// ---- The report ----------------------------------------------------------------------------------

struct TocEntry {
    QString title;
    int page;
    int level = 1;
};

struct Col {
    QString head;
    double w;  // relative
    Qt::Alignment align = Qt::AlignLeft;
};
struct Cell {
    QString text;
    QColor color;  // invalid: ink
    bool bold = false;
};

class Report {
public:
    Report(const CamProject &project, const ProjectInfo &pi, const ReportOptions &o, QSizeF page, const QString &stamp)
        : m_p(project), m_pi(pi), m_o(o), m_page(page), m_stamp(stamp) {
        m_body = QRectF(50, 62, page.width() - 100, page.height() - 62 - 52);
        m_name = o.projectName.isEmpty() ? QStringLiteral("Untitled") : o.projectName;
    }

    // Lays out and draws every page; returns false when a page could not be started.
    bool Render(Sink &sink, int totalPages, const std::vector<TocEntry> &toc) {
        m_sink = &sink;
        m_total = totalPages;
        m_tocKnown = toc;
        m_toc.clear();
        m_pageNo = 0;
        m_section = 0;
        m_sub = 0;
        m_painter = nullptr;
        if (!Cover()) return false;
        if (!m_painter) return false;
        Overview();
        Timing();
        Cycle();
        MasterSection();
        for (int i = 0; i < static_cast<int>(m_p.axes.size()); ++i) AxisSection(i);
        OutputsSection();
        ChecksSection();
        SignOff();
        if (m_painter) m_sink->End();
        m_painter = nullptr;
        return !m_failed;
    }

    int pages() const { return m_pageNo; }
    const std::vector<TocEntry> &toc() const { return m_toc; }

private:
    // ---- Page machinery
    void NewPage(bool cover = false) {
        if (m_painter) m_sink->End();
        m_painter = m_sink->Begin();
        if (!m_painter) {
            m_failed = true;
            static QImage dummy(8, 8, QImage::Format_RGB32);
            static QPainter fallback;  // keep drawing somewhere harmless
            if (!fallback.isActive()) fallback.begin(&dummy);
            m_painter = &fallback;
        }
        ++m_pageNo;
        m_y = m_body.top();
        if (!cover) HeaderFooter();
    }

    void HeaderFooter() {
        QPainter &p = *m_painter;
        p.setPen(kMuted);
        p.setFont(F(7.5, QFont::DemiBold));
        const double top = 30;
        p.drawText(QRectF(m_body.left(), top, m_body.width() / 2, 12), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral(ECAM_PRODUCT_NAME " · Project report"));
        p.setFont(F(7.5));
        p.drawText(QRectF(m_body.center().x(), top, m_body.width() / 2, 12), Qt::AlignRight | Qt::AlignVCenter,
                   Fm(F(7.5)).elidedText(m_name, Qt::ElideMiddle, m_body.width() / 2));
        p.setPen(QPen(kRule, 0.6));
        p.drawLine(QPointF(m_body.left(), top + 16), QPointF(m_body.right(), top + 16));
        const double bottom = m_page.height() - 34;
        p.drawLine(QPointF(m_body.left(), bottom - 6), QPointF(m_body.right(), bottom - 6));
        p.setPen(kMuted);
        p.drawText(QRectF(m_body.left(), bottom, m_body.width() * 0.7, 12), Qt::AlignLeft | Qt::AlignVCenter,
                   Fm(F(7.5)).elidedText(FileLine() + QStringLiteral("  ·  ") + m_stamp, Qt::ElideMiddle, m_body.width() * 0.7));
        p.setFont(F(7.5, QFont::DemiBold));
        p.drawText(QRectF(m_body.left(), bottom, m_body.width(), 12), Qt::AlignRight | Qt::AlignVCenter,
                   m_total > 0 ? QStringLiteral("Page %1 of %2").arg(m_pageNo).arg(m_total) : QStringLiteral("Page %1").arg(m_pageNo));
    }

    QString FileLine() const {
        return m_o.projectPath.isEmpty() ? QStringLiteral("not saved") : QFileInfo(m_o.projectPath).fileName();
    }

    QFontMetricsF Fm(const QFont &f) const { return QFontMetricsF(f, m_painter ? m_painter->device() : nullptr); }

    double TextH(const QFont &f, double w, const QString &t) const {
        if (t.isEmpty()) return 0.0;
        return Fm(f).boundingRect(QRectF(0, 0, w, 1e6), Qt::TextWordWrap, t).height();
    }

    bool Fits(double h) const { return m_y + h <= m_body.bottom() + 0.01; }
    void Need(double h) {
        if (!Fits(h)) NewPage();
    }

    // A numbered section: level 1 ("5"), level 2 ("5.1"). Starts a page unless `minSpace` > 0 and that
    // much is left on this one (short sections run on).
    void Section(const QString &title, int level = 1, double minSpace = 0) {
        if (minSpace > 0 && m_painter && m_y > m_body.top() + 1 && Fits(minSpace + 70)) {
            m_y += 22;
        } else {
            NewPage();
        }
        QString number;
        if (level == 1) {
            ++m_section;
            m_sub = 0;
            number = QString::number(m_section);
        } else {
            ++m_sub;
            number = QStringLiteral("%1.%2").arg(m_section).arg(m_sub);
        }
        m_toc.push_back({QStringLiteral("%1  %2").arg(number, title), m_pageNo, level});
        QPainter &p = *m_painter;
        const QFont numFont = F(level == 1 ? 10 : 9.5, QFont::DemiBold);
        const double nw = Fm(numFont).horizontalAdvance(number) + 12;
        p.setPen(kAccent);
        p.setFont(numFont);
        p.drawText(QRectF(m_body.left(), m_y, nw, 28), Qt::AlignLeft | Qt::AlignBottom, number);
        const QFont titleFont = F(level == 1 ? 19 : 16, QFont::DemiBold);
        p.setFont(titleFont);
        p.setPen(kInk);
        p.drawText(QRectF(m_body.left() + nw, m_y, m_body.width() - nw, 30), Qt::AlignLeft | Qt::AlignBottom,
                   Fm(titleFont).elidedText(title, Qt::ElideRight, m_body.width() - nw));
        m_y += 34;
        p.fillRect(QRectF(m_body.left(), m_y, 36, 2.2), kAccent);
        p.fillRect(QRectF(m_body.left() + 36, m_y + 0.8, m_body.width() - 36, 0.6), kRule);
        m_y += 16;
    }

    void H2(const QString &text, double keepWith = 60) {
        Need(22 + keepWith);
        QPainter &p = *m_painter;
        m_y += 4;
        p.fillRect(QRectF(m_body.left(), m_y + 3, 3, 12), kAccent);
        p.setPen(kInk);
        p.setFont(F(11.5, QFont::DemiBold));
        p.drawText(QRectF(m_body.left() + 9, m_y, m_body.width() - 9, 18), Qt::AlignLeft | Qt::AlignVCenter, text);
        m_y += 24;
    }

    void Para(const QString &text, const QFont &f = F(9), const QColor &color = kInk2, double gapAfter = 8) {
        const double h = TextH(f, m_body.width(), text);
        Need(h);
        m_painter->setFont(f);
        m_painter->setPen(color);
        m_painter->drawText(QRectF(m_body.left(), m_y, m_body.width(), h + 2), Qt::TextWordWrap, text);
        m_y += h + gapAfter;
    }

    void Note(const QString &text) { Para(text, F(7.8, QFont::Normal, true), kMuted, 10); }

    // A coloured box with a title and a line of text.
    void Callout(const QColor &color, const QString &title, const QString &text) {
        const double w = m_body.width() - 24;
        const double h = 14 + 14 + TextH(F(8.5), w, text) + 10;
        Need(h);
        QPainter &p = *m_painter;
        const QRectF r(m_body.left(), m_y, m_body.width(), h);
        p.fillRect(r, Tint(color, 0.08));
        p.fillRect(QRectF(r.left(), r.top(), 3.5, r.height()), color);
        p.setPen(color.darker(115));
        p.setFont(F(10, QFont::DemiBold));
        p.drawText(QRectF(r.left() + 14, r.top() + 8, w, 16), Qt::AlignLeft | Qt::AlignVCenter, title);
        p.setPen(kInk2);
        p.setFont(F(8.5));
        p.drawText(QRectF(r.left() + 14, r.top() + 26, w, h - 30), Qt::TextWordWrap, text);
        m_y += h + 12;
    }

    // ---- Tables
    void Table(const std::vector<Col> &cols, const std::vector<std::vector<Cell>> &rows, double fontPt = 8.3,
               double x = -1, double width = -1, double padY = 3.6) {
        if (x < 0) x = m_body.left();
        if (width < 0) width = m_body.width();
        double total = 0;
        for (const Col &c : cols) total += c.w;
        std::vector<double> ws;
        for (const Col &c : cols) ws.push_back(c.w / total * width);
        const QFont body = F(fontPt), bold = F(fontPt, QFont::DemiBold), head = F(fontPt - 0.4, QFont::DemiBold);
        const double padX = 5;
        auto rowH = [&](const std::vector<Cell> &r, bool header) {
            double h = 0;
            for (size_t c = 0; c < cols.size(); ++c) {
                const QString t = header ? cols[c].head : (c < r.size() ? r[c].text : QString());
                const QFont &f = header ? head : (c < r.size() && r[c].bold ? bold : body);
                h = std::max(h, TextH(f, ws[c] - 2 * padX, t.isEmpty() ? QStringLiteral(" ") : t));
            }
            return h + 2 * padY;
        };
        const double headH = rowH({}, true);
        QPainter *p = nullptr;
        auto header = [&]() {
            p = m_painter;
            double cx = x;
            p->fillRect(QRectF(x, m_y, width, headH), kAccentSoft);
            p->setFont(head);
            p->setPen(kAccent);
            for (size_t c = 0; c < cols.size(); ++c) {
                p->drawText(QRectF(cx + padX, m_y + padY, ws[c] - 2 * padX, headH - 2 * padY), static_cast<int>(cols[c].align) | Qt::TextWordWrap, cols[c].head);
                cx += ws[c];
            }
            p->setPen(QPen(kAccent, 0.8));
            p->drawLine(QPointF(x, m_y + headH), QPointF(x + width, m_y + headH));
            m_y += headH;
        };
        // A table that fits on a page is kept whole (no stray rows on the next page); a longer one
        // starts with at least its header and first row.
        double whole = headH;
        for (const auto &r : rows) whole += rowH(r, false);
        if (whole <= m_body.height() - 4) Need(whole);
        else Need(headH + (rows.empty() ? 0 : rowH(rows[0], false)));
        header();
        for (size_t r = 0; r < rows.size(); ++r) {
            const double h = rowH(rows[r], false);
            if (!Fits(h)) {
                NewPage();
                header();
            }
            if (r % 2 == 1) p->fillRect(QRectF(x, m_y, width, h), kZebra);
            double cx = x;
            for (size_t c = 0; c < cols.size() && c < rows[r].size(); ++c) {
                const Cell &cell = rows[r][c];
                p->setFont(cell.bold ? bold : body);
                p->setPen(cell.color.isValid() ? cell.color : kInk);
                p->drawText(QRectF(cx + padX, m_y + padY, ws[c] - 2 * padX, h - 2 * padY), static_cast<int>(cols[c].align) | Qt::TextWordWrap, cell.text);
                cx += ws[c];
            }
            m_y += h;
            p->setPen(QPen(kGrid, 0.5));
            p->drawLine(QPointF(x, m_y), QPointF(x + width, m_y));
        }
        p->setPen(QPen(kRule, 0.7));
        p->drawLine(QPointF(x, m_y), QPointF(x + width, m_y));
        m_y += 12;
    }

    std::vector<std::vector<Cell>> RowCells(const std::vector<Row> &rows, bool notes) const {
        std::vector<std::vector<Cell>> out;
        for (const Row &r : rows) {
            std::vector<Cell> c{{r.key, kInk2}, {r.shown(), {}, true}, {r.unit, kMuted}};
            if (notes) c.push_back({r.note, kMuted});
            out.push_back(c);
        }
        return out;
    }

    // ---- Charts
    void DrawChart(const QRectF &box, const Chart &c) {
        QPainter &p = *m_painter;
        const double titleH = c.title.isEmpty() ? 0.0 : 15.0;
        if (!c.title.isEmpty()) {
            p.setPen(kInk);
            p.setFont(F(8.8, QFont::DemiBold));
            const double tw = Fm(F(8.8, QFont::DemiBold)).horizontalAdvance(c.title);
            p.drawText(QRectF(box.left(), box.top(), box.width(), 13), Qt::AlignLeft | Qt::AlignVCenter, c.title);
            if (!c.unit.isEmpty()) {
                p.setPen(kMuted);
                p.setFont(F(8));
                p.drawText(QRectF(box.left() + tw + 6, box.top(), box.width(), 13), Qt::AlignLeft | Qt::AlignVCenter, c.unit);
            }
        }
        const double xLabH = c.xTitle.isEmpty() ? 13.0 : 24.0;
        const QRectF plot(box.left() + 46, box.top() + titleH + 3, box.width() - 46 - 4, box.height() - titleH - 3 - xLabH);

        double lo = std::numeric_limits<double>::infinity(), hi = -lo;
        for (const Series &s : c.series) {
            for (const QPointF &q : s.pts) {
                lo = std::min(lo, q.y());
                hi = std::max(hi, q.y());
            }
        }
        for (const Ref &r : c.refs) {
            lo = std::min(lo, r.y);
            hi = std::max(hi, r.y);
        }
        if (c.fixedY) {
            lo = c.yLo;
            hi = c.yHi;
        }
        if (!std::isfinite(lo)) {
            lo = 0;
            hi = 1;
        }
        if (hi - lo < 1e-12 * std::max(1.0, std::fabs(hi))) {
            lo -= 1;
            hi += 1;
        }
        if (!c.fixedY) {
            const double pad = (hi - lo) * 0.08;
            lo -= pad;
            hi += pad;
        }
        const double ys = NiceStep(hi - lo, 4);
        const double ylo = c.fixedY ? lo : std::floor(lo / ys) * ys, yhi = c.fixedY ? hi : std::ceil(hi / ys) * ys;
        auto X = [&](double x) { return plot.left() + (x - c.x0) / (c.x1 - c.x0) * plot.width(); };
        auto Y = [&](double y) { return plot.bottom() - (y - ylo) / (yhi - ylo) * plot.height(); };

        p.fillRect(plot, Qt::white);
        for (const Band &b : c.bands) {
            const QRectF br(X(b.x0), plot.top(), X(b.x1) - X(b.x0), plot.height());
            p.fillRect(br, b.fill);
            if (!b.label.isEmpty() && br.width() > 26) {
                p.setPen(kInk2);
                p.setFont(F(6.8));
                p.drawText(QRectF(br.left() + 3, plot.top() + 2, br.width() - 6, 10), Qt::AlignLeft | Qt::AlignVCenter,
                           Fm(F(6.8)).elidedText(b.label, Qt::ElideRight, br.width() - 6));
            }
        }
        // Grid and ticks
        const QFont tick = F(6.8);
        p.setFont(tick);
        const int yd = StepDecimals(ys);
        for (double y = ylo; y <= yhi + ys * 0.01; y += ys) {
            p.setPen(QPen(kGrid, 0.5));
            p.drawLine(QPointF(plot.left(), Y(y)), QPointF(plot.right(), Y(y)));
            p.setPen(kMuted);
            p.drawText(QRectF(box.left(), Y(y) - 5, 42, 10), Qt::AlignRight | Qt::AlignVCenter, Num(y, yd));
        }
        double xs = c.crank ? (plot.width() > 330 ? 30.0 : 60.0) : NiceStep(c.x1 - c.x0, plot.width() > 330 ? 10 : 6);
        if (c.crank && c.x1 - c.x0 < 300) xs = NiceStep(c.x1 - c.x0, 8);
        const int xd = StepDecimals(xs);
        for (double x = std::ceil(c.x0 / xs - 1e-9) * xs; x <= c.x1 + xs * 1e-6; x += xs) {
            p.setPen(QPen(kGrid, 0.5));
            p.drawLine(QPointF(X(x), plot.top()), QPointF(X(x), plot.bottom()));
            p.setPen(kMuted);
            p.drawText(QRectF(X(x) - 20, plot.bottom() + 2, 40, 10), Qt::AlignHCenter | Qt::AlignTop,
                       Num(x, xd) + (c.crank ? QStringLiteral("°") : QString()));
        }
        if (!c.xTitle.isEmpty()) {
            p.setFont(F(7.2));
            p.drawText(QRectF(plot.left(), plot.bottom() + 12, plot.width(), 11), Qt::AlignHCenter | Qt::AlignVCenter, c.xTitle);
        }
        p.save();
        p.setClipRect(plot.adjusted(-1, -1, 1, 1));
        p.setPen(QPen(kRule, 0.6, Qt::DotLine));
        for (double x : c.vlines) p.drawLine(QPointF(X(x), plot.top()), QPointF(X(x), plot.bottom()));
        if (ylo < 0 && yhi > 0) {
            p.setPen(QPen(kInk2, 0.6));
            p.drawLine(QPointF(plot.left(), Y(0)), QPointF(plot.right(), Y(0)));
        }
        for (const Ref &r : c.refs) {
            p.setPen(QPen(r.color, 0.9, Qt::DashLine));
            p.drawLine(QPointF(plot.left(), Y(r.y)), QPointF(plot.right(), Y(r.y)));
        }
        for (const Series &s : c.series) {
            if (s.pts.size() < 2) continue;
            QPainterPath path(QPointF(X(s.pts[0].x()), Y(s.pts[0].y())));
            for (size_t i = 1; i < s.pts.size(); ++i) path.lineTo(X(s.pts[i].x()), Y(s.pts[i].y()));
            QPen pen(s.color, s.width, s.style);
            pen.setJoinStyle(Qt::RoundJoin);
            pen.setCapStyle(Qt::RoundCap);
            p.setPen(pen);
            p.setBrush(Qt::NoBrush);
            p.drawPath(path);
        }
        p.setPen(QPen(c.dotColor.isValid() ? c.dotColor : kInk, 1.0));
        p.setBrush(Qt::white);
        for (const QPointF &d : c.dots) p.drawEllipse(QPointF(X(d.x()), Y(d.y())), 2.3, 2.3);
        p.restore();
        // Reference labels and marks (outside the clip so they are not cut)
        for (const Ref &r : c.refs) {
            if (r.label.isEmpty()) continue;
            p.setPen(r.color);
            p.setFont(F(6.6));
            p.drawText(QRectF(plot.right() - 160, Y(r.y) - (r.y >= 0 ? 10 : -1), 157, 9), Qt::AlignRight | Qt::AlignVCenter, r.label);
        }
        for (const Mark &m : c.marks) {
            const QPointF q(X(m.x), Y(m.y));
            p.setPen(Qt::NoPen);
            p.setBrush(m.color);
            p.drawEllipse(q, 2.4, 2.4);
            p.setPen(m.color.darker(120));
            p.setFont(F(6.8, QFont::DemiBold));
            const double tw = Fm(F(6.8, QFont::DemiBold)).horizontalAdvance(m.label) + 4;
            double lx = q.x() + 5;
            if (lx + tw > plot.right()) lx = q.x() - 5 - tw;
            const double ly = std::clamp(q.y() + (m.y >= 0 ? -11.0 : 2.0), plot.top(), plot.bottom() - 9);
            p.drawText(QRectF(lx, ly, tw, 9), Qt::AlignLeft | Qt::AlignVCenter, m.label);
        }
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(kRule, 0.7));
        p.drawRect(plot);
    }

    // ---- Shared drawing bits
    void Box(const QRectF &r, const QColor &edge, const QColor &fill, double radius = 4) {
        m_painter->setPen(QPen(edge, 0.9));
        m_painter->setBrush(fill);
        m_painter->drawRoundedRect(r, radius, radius);
        m_painter->setBrush(Qt::NoBrush);
    }

    void Arrow(QPointF a, QPointF b, const QColor &color, double width = 0.9) {
        QPainter &p = *m_painter;
        p.setPen(QPen(color, width));
        const QPointF d = b - a;
        const double len = std::hypot(d.x(), d.y());
        if (len < 1e-6) return;
        const QPointF u = d / len, n(-u.y(), u.x());
        const QPointF tip = b, base = b - u * 5.5;
        p.drawLine(a, base);
        p.setPen(Qt::NoPen);
        p.setBrush(color);
        p.drawPolygon(QPolygonF() << tip << base + n * 2.8 << base - n * 2.8);
        p.setBrush(Qt::NoBrush);
    }

    void Chip(double x, double y, const QString &text, const QColor &color) {
        QPainter &p = *m_painter;
        const QFont f = F(7.2, QFont::DemiBold);
        const double w = Fm(f).horizontalAdvance(text) + 12;
        p.setPen(Qt::NoPen);
        p.setBrush(Tint(color, 0.14));
        p.drawRoundedRect(QRectF(x, y, w, 14), 7, 7);
        p.setBrush(Qt::NoBrush);
        p.setPen(color.darker(125));
        p.setFont(f);
        p.drawText(QRectF(x, y, w, 14), Qt::AlignCenter, text);
        m_chipX = x + w + 5;
    }

    QString MasterLine() const {
        const CamMaster &m = m_p.master;
        QStringList parts;
        if (UsesCrank(m_p)) {
            if (m.source == MasterSource::Encoder) {
                parts << (m.lineMode ? QStringLiteral("Encoder on the line · %1 cuts/min").arg(Num(m.rpm, 1))
                                     : QStringLiteral("Encoder · %1 rpm").arg(Num(m.rpm, 1)));
            } else {
                parts << (m.lineMode ? QStringLiteral("Crank motor M-%1 feeds the line · %2 cuts/min").arg(m.crankConnector).arg(Num(m.rpm, 1))
                                     : QStringLiteral("Driven crank M-%1 · %2 rpm").arg(m.crankConnector).arg(Num(m.rpm, 1)));
            }
        }
        if (UsesTimeline(m_p)) parts << QStringLiteral("Timeline · %1").arg(TriggerText(m));
        return parts.isEmpty() ? QStringLiteral("No master in use") : parts.join(QStringLiteral("  +  "));
    }

    // ---- Cover
    bool Cover() {
        NewPage(true);
        if (m_failed) return false;
        QPainter &p = *m_painter;
        const double bandH = 196;
        p.fillRect(QRectF(0, 0, m_page.width(), bandH), kAccent);
        p.fillRect(QRectF(0, bandH, m_page.width(), 4), kCrank);
        QSvgRenderer icon(QStringLiteral(":/icons/ecam_app_icon.svg"));
        if (icon.isValid()) icon.render(&p, QRectF(m_page.width() - 50 - 58, 46, 58, 58));
        p.setPen(QColor(0x9f, 0xc3, 0xea));
        QFont small = F(8.5, QFont::DemiBold);
        small.setLetterSpacing(QFont::AbsoluteSpacing, 1.6);
        p.setFont(small);
        p.drawText(QRectF(50, 50, 400, 14), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("PROJECT REPORT"));
        p.setPen(Qt::white);
        p.setFont(F(28, QFont::DemiBold));
        p.drawText(QRectF(50, 68, m_page.width() - 180, 44), Qt::AlignLeft | Qt::AlignVCenter,
                   Fm(F(28, QFont::DemiBold)).elidedText(m_name, Qt::ElideRight, m_page.width() - 180));
        p.setPen(QColor(0xd5, 0xe0, 0xec));
        p.setFont(F(9.5));
        p.drawText(QRectF(50, 116, m_page.width() - 100, 14), Qt::AlignLeft | Qt::AlignVCenter, MasterLine());
        p.setPen(QColor(0xaf, 0xc2, 0xd8));
        p.setFont(F(8));
        const QString path = m_o.projectPath.isEmpty() ? QStringLiteral("Not saved to a file") : QDir::toNativeSeparators(m_o.projectPath);
        p.drawText(QRectF(50, 142, m_page.width() - 100, 12), Qt::AlignLeft | Qt::AlignVCenter,
                   Fm(F(8)).elidedText(path, Qt::ElideMiddle, m_page.width() - 100));
        p.drawText(QRectF(50, 156, m_page.width() - 100, 12), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("Generated %1 by " ECAM_PRODUCT_NAME).arg(m_stamp));

        m_y = bandH + 26;
        // Review status
        const int errors = m_pi.errors(), warnings = m_pi.warnings();
        if (errors == 0 && warnings == 0) {
            Callout(kGood, QStringLiteral("No issues found"),
                    QStringLiteral("No limit is exceeded, every motor connector has one owner, and velocity and acceleration are "
                                   "continuous at every knot. Review the details before running it on the machine."));
        } else {
            QStringList count;
            if (errors) count << QStringLiteral("%1 error%2").arg(errors).arg(errors == 1 ? "" : "s");
            if (warnings) count << QStringLiteral("%1 warning%2").arg(warnings).arg(warnings == 1 ? "" : "s");
            Callout(errors ? kError : kWarn, count.join(QStringLiteral(", ")),
                    QStringLiteral("Listed under Checks%1. Errors must be fixed before running; warnings deserve a look.")
                        .arg(TocPage(QStringLiteral("Checks"))));
        }

        // Key figures
        struct Tile {
            QString label, value, sub;
        };
        std::vector<Tile> tiles;
        const CamMaster &m = m_p.master;
        if (UsesCrank(m_p)) {
            tiles.push_back({QStringLiteral("CRANK"), m.source == MasterSource::Encoder ? QStringLiteral("%1 rpm").arg(Num(m.rpm, 1))
                                                                                        : QStringLiteral("%1 rpm").arg(Num(m.rpm, 1)),
                             m.source == MasterSource::Encoder ? QStringLiteral("external encoder, %1 counts/rev").arg(Num(m.crankPpr, 0))
                                                               : QStringLiteral("driven crank on M-%1").arg(m.crankConnector)});
            tiles.push_back({QStringLiteral("CYCLE TIME"), m.rpm > 0 ? QStringLiteral("%1 s").arg(Num(60.0 / m.rpm, 3)) : QStringLiteral("—"),
                             QStringLiteral("one crank revolution")});
            tiles.push_back({QStringLiteral("CRANK HEADROOM"),
                             std::isfinite(m_pi.crankMaxRpm) ? QStringLiteral("%1 rpm").arg(Num(m_pi.crankMaxRpm, 1)) : QStringLiteral("no limit"),
                             std::isfinite(m_pi.crankMaxRpm) ? QStringLiteral("max allowed: %1").arg(m_pi.crankReason)
                                                             : QStringLiteral("set axis limits to get one")});
        }
        if (UsesTimeline(m_p)) {
            tiles.push_back({QStringLiteral("TIMELINE"), QStringLiteral("%1 s").arg(Num(TimelineLength(m_p), 3)),
                             (m.timelineRepeat ? QStringLiteral("repeating, ") : QStringLiteral("once per trigger, ")) +
                                 QStringLiteral("%1 %").arg(Num(m.speedOverride * 100.0, 0))});
        }
        QStringList motors;
        for (const CamAxis &a : m_p.axes) motors << QStringLiteral("M-%1").arg(a.connector);
        tiles.push_back({QStringLiteral("AXES"), QString::number(m_p.axes.size()), motors.join(QStringLiteral(", "))});
        std::set<int> outs;
        for (const OutputPulse &o : m_p.outputs) outs.insert(o.output);
        QStringList outNames;
        for (int k : outs) outNames << QStringLiteral("IO-%1").arg(k);
        tiles.push_back({QStringLiteral("OUTPUT PULSES"), QString::number(m_p.outputs.size()),
                         outNames.isEmpty() ? QStringLiteral("no outputs switched") : outNames.join(QStringLiteral(", "))});
        double peakRpm = 0;
        QString peakAxis;
        for (size_t i = 0; i < m_p.axes.size(); ++i) {
            if (m_pi.axes[i].motorPeakRpm > peakRpm) {
                peakRpm = m_pi.axes[i].motorPeakRpm;
                peakAxis = Q(m_p.axes[i].name);
            }
        }
        tiles.push_back({QStringLiteral("PEAK MOTOR SPEED"), QStringLiteral("%1 rpm").arg(Num(peakRpm, 0)),
                         peakAxis.isEmpty() ? QString() : peakAxis});
        if (tiles.size() > 6) tiles.resize(6);

        const int cols = 3;
        const double gap = 10, tw = (m_body.width() - gap * (cols - 1)) / cols, th = 58;
        for (size_t i = 0; i < tiles.size(); ++i) {
            const double x = m_body.left() + (i % cols) * (tw + gap), y = m_y + (i / cols) * (th + gap);
            const QRectF r(x, y, tw, th);
            p.setPen(QPen(kGrid, 0.8));
            p.setBrush(QColor(0xfa, 0xfb, 0xfc));
            p.drawRoundedRect(r, 4, 4);
            p.setBrush(Qt::NoBrush);
            QFont lab = F(6.8, QFont::DemiBold);
            lab.setLetterSpacing(QFont::AbsoluteSpacing, 0.9);
            p.setFont(lab);
            p.setPen(kMuted);
            p.drawText(QRectF(x + 10, y + 7, tw - 20, 10), Qt::AlignLeft | Qt::AlignVCenter, tiles[i].label);
            p.setFont(F(15, QFont::DemiBold));
            p.setPen(kInk);
            p.drawText(QRectF(x + 10, y + 18, tw - 20, 22), Qt::AlignLeft | Qt::AlignVCenter, tiles[i].value);
            p.setFont(F(7.2));
            p.setPen(kInk2);
            p.drawText(QRectF(x + 10, y + 40, tw - 20, 12), Qt::AlignLeft | Qt::AlignVCenter,
                       Fm(F(7.2)).elidedText(tiles[i].sub, Qt::ElideRight, tw - 20));
        }
        m_y += ((tiles.size() + cols - 1) / cols) * (th + gap) + 14;

        // Contents (page numbers from the first pass)
        p.setPen(kInk);
        p.setFont(F(11.5, QFont::DemiBold));
        p.drawText(QRectF(m_body.left(), m_y, m_body.width(), 16), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("Contents"));
        m_y += 22;
        // Two columns when the list is long.
        const double rowH = 14.5;
        const int rowsFit = std::max(1, static_cast<int>((m_page.height() - 80 - m_y) / rowH));
        const int columns = static_cast<int>(m_tocKnown.size()) > rowsFit ? 2 : 1;
        const double colGap = 24, colW = (m_body.width() - (columns - 1) * colGap) / columns;
        const int perCol = (static_cast<int>(m_tocKnown.size()) + columns - 1) / columns;
        const double y0 = m_y;
        for (size_t idx = 0; idx < m_tocKnown.size(); ++idx) {
            const TocEntry &e = m_tocKnown[idx];
            const int col = static_cast<int>(idx) / perCol;
            const double x = m_body.left() + col * (colW + colGap);
            const double y = y0 + (static_cast<int>(idx) % perCol) * rowH;
            if (y > m_page.height() - 80) continue;
            const double indent = e.level == 1 ? 0 : 16;
            const QFont tf = e.level == 1 ? F(9, QFont::DemiBold) : F(8.5);
            p.setFont(tf);
            p.setPen(e.level == 1 ? kInk : kInk2);
            const QString t = Fm(tf).elidedText(e.title, Qt::ElideRight, colW - indent - 40);
            const double tw2 = Fm(tf).horizontalAdvance(t);
            p.drawText(QRectF(x + indent, y, colW - indent - 30, 13), Qt::AlignLeft | Qt::AlignVCenter, t);
            p.setPen(QPen(kRule, 0.8, Qt::DotLine));
            p.drawLine(QPointF(x + indent + tw2 + 6, y + 9.5), QPointF(x + colW - 24, y + 9.5));
            p.setPen(kInk2);
            p.setFont(F(8.5));
            p.drawText(QRectF(x + colW - 22, y, 22, 13), Qt::AlignRight | Qt::AlignVCenter, QString::number(e.page));
        }
        m_y = y0 + std::min(perCol, rowsFit) * rowH;

        // Foot of the cover
        p.setPen(kMuted);
        p.setFont(F(7.4, QFont::Normal, true));
        p.drawText(QRectF(m_body.left(), m_page.height() - 62, m_body.width(), 24), Qt::TextWordWrap,
                   QStringLiteral("Experimental software, not certified for production or safety-critical use. Check every value "
                                  "against the machine and keep a physical emergency stop in the circuit."));
        return true;
    }

    QString TocPage(const QString &title) const {
        for (const TocEntry &e : m_tocKnown) {
            if (e.title.endsWith(title)) return QStringLiteral(" (page %1)").arg(e.page);
        }
        return {};
    }

    // ---- Overview: block diagram and the motor / I/O map
    struct Node {
        QString title;
        QStringList lines;
        QColor color;
        QRectF r;
    };

    double NodeHeight(const Node &n, double w) const {
        double h = 22;
        for (const QString &l : n.lines) h += TextH(F(7.6), w - 16, l) + 1.5;
        return h + 6;
    }

    void DrawNode(const Node &n) {
        QPainter &p = *m_painter;
        Box(n.r, n.color, Tint(n.color, 0.06));
        p.fillRect(QRectF(n.r.left() + 0.5, n.r.top() + 0.5, 3, n.r.height() - 1), n.color);
        p.setPen(kInk);
        p.setFont(F(8.6, QFont::DemiBold));
        p.drawText(QRectF(n.r.left() + 10, n.r.top() + 5, n.r.width() - 16, 13), Qt::AlignLeft | Qt::AlignVCenter,
                   Fm(F(8.6, QFont::DemiBold)).elidedText(n.title, Qt::ElideRight, n.r.width() - 16));
        double y = n.r.top() + 21;
        p.setFont(F(7.6));
        p.setPen(kInk2);
        for (const QString &l : n.lines) {
            const double h = TextH(F(7.6), n.r.width() - 16, l);
            p.drawText(QRectF(n.r.left() + 10, y, n.r.width() - 16, h + 1), Qt::TextWordWrap, l);
            y += h + 1.5;
        }
    }

    void Overview() {
        Section(QStringLiteral("System overview"));
        const CamMaster &m = m_p.master;
        Para(QStringLiteral("How the parts of the machine connect: the master the ClearCore follows, each axis it drives and "
                            "the outputs it switches. Crank connections are blue, timeline connections amber."));

        std::vector<Node> left, right;
        if (UsesCrank(m_p)) {
            Node n{QStringLiteral("Crank master"), {}, kCrank, {}};
            if (m.source == MasterSource::Encoder) {
                n.lines << QStringLiteral("External encoder on the Encoder Adapter (DI-6..DI-8)")
                        << QStringLiteral("%1 counts/rev%2").arg(Num(m.crankPpr, 0), m.encoderReverse ? QStringLiteral(", reversed") : QString());
                if (m.lineMode) {
                    n.lines << QStringLiteral("Cut-length mode: %1 mm per cut, wheel %2 mm").arg(Num(m.designLength, 1), Num(m.wheelCircumference, 1));
                } else {
                    n.lines << QStringLiteral("Gear %1 : %2 · expected %3 rpm").arg(m.crankGearNum).arg(m.crankGearDen).arg(Num(m.rpm, 1));
                }
            } else {
                n.lines << QStringLiteral("Crank motor on M-%1, driven by the firmware").arg(m.crankConnector);
                if (m.lineMode) {
                    n.lines << QStringLiteral("Cut-length mode: %1 mm per cut, %2 mm of line per motor rev").arg(Num(m.designLength, 1), Num(m.wheelCircumference, 1))
                            << QStringLiteral("Line speed %1 m/min").arg(Num(m.rpm * m.designLength / 1000.0, 2));
                } else {
                    n.lines << QStringLiteral("%1 rpm, ramp %2 rpm/s").arg(Num(m.rpm, 1), Num(m.crankRampRpmPerSec, 0))
                            << QStringLiteral("Gear %1 : %2").arg(m.crankGearNum).arg(m.crankGearDen);
                }
            }
            left.push_back(n);
        }
        if (UsesTimeline(m_p)) {
            Node n{QStringLiteral("Timeline"), {}, kTimeline, {}};
            n.lines << QStringLiteral("Trigger: %1").arg(TriggerText(m))
                    << QStringLiteral("%1 s, %2").arg(Num(TimelineLength(m_p), 3),
                                                      m.timelineRepeat ? QStringLiteral("repeating") : QStringLiteral("once per trigger"))
                    << QStringLiteral("Speed override %1 %").arg(Num(m.speedOverride * 100.0, 0));
            left.push_back(n);
        }
        for (size_t i = 0; i < m_p.axes.size(); ++i) {
            const CamAxis &a = m_p.axes[i];
            Node n{Q(a.name), {}, OnTimeline(a) ? kTimeline : kCrank, {}};
            QString drive = DriveText(a.drive);
            if (a.drive == AxisDrive::Ballscrew) drive += QStringLiteral(" %1 %2/rev").arg(Num(a.screwPitch, 2), Q(a.units));
            if (a.drive == AxisDrive::Belt) drive += QStringLiteral(" Ø%1 %2").arg(Num(a.pulleyPd, 2), Q(a.units));
            n.lines << QStringLiteral("M-%1 · %2").arg(a.connector).arg(drive)
                    << QStringLiteral("Stroke %1 %2 · %3 segments").arg(Num(m_pi.axes[i].stats.maxPos - m_pi.axes[i].stats.minPos, 2), Q(a.units))
                           .arg(a.segments.size());
            right.push_back(n);
        }
        if (!m_p.outputs.empty()) {
            Node n{QStringLiteral("Outputs"), {}, kInk2, {}};
            std::map<int, std::pair<int, int>> per;  // output -> crank, timeline pulses
            for (const OutputPulse &o : m_p.outputs) (o.master == AxisMaster::Crank ? per[o.output].first : per[o.output].second)++;
            for (const auto &[k, c] : per) {
                QStringList what;
                if (c.first) what << QStringLiteral("%1 crank").arg(c.first);
                if (c.second) what << QStringLiteral("%1 timeline").arg(c.second);
                n.lines << QStringLiteral("IO-%1: %2 pulse%3").arg(k).arg(what.join(QStringLiteral(", "))).arg(c.first + c.second == 1 ? "" : "s");
            }
            right.push_back(n);
        }

        const double wL = 150, wC = 112, gap = 44;
        const double xL = m_body.left(), xC = xL + wL + gap, xR = xC + wC + gap, wR = m_body.right() - xR;
        double hl = 0, hr = 0;
        for (Node &n : left) hl += NodeHeight(n, wL) + 12;
        for (Node &n : right) hr += NodeHeight(n, wR) + 10;
        const double h = std::max({hl, hr, 120.0});
        Need(h + 10);
        const double top = m_y;
        double y = top + (h - hl) / 2;
        for (Node &n : left) {
            n.r = QRectF(xL, y, wL, NodeHeight(n, wL));
            y += n.r.height() + 12;
        }
        y = top + (h - hr) / 2;
        for (Node &n : right) {
            n.r = QRectF(xR, y, wR, NodeHeight(n, wR));
            y += n.r.height() + 10;
        }
        Node core{QStringLiteral("ClearCore"), {QStringLiteral("ECam firmware"), QStringLiteral("runs the profile from flash")}, kAccent,
                  QRectF(xC, top, wC, h - 10)};
        QPainter &p = *m_painter;
        for (const Node &n : left) {
            const double ay = n.r.center().y();
            Arrow(QPointF(n.r.right(), ay), QPointF(xC, ay), n.color, 1.1);
            p.setPen(n.color.darker(115));
            p.setFont(F(6.8));
            p.drawText(QRectF(n.r.right(), ay - 11, gap, 9), Qt::AlignCenter,
                       n.color == kCrank ? QStringLiteral("position") : QStringLiteral("trigger"));
        }
        for (const Node &n : right) {
            const double ay = n.r.center().y();
            Arrow(QPointF(xC + wC, ay), QPointF(n.r.left(), ay), n.color, 1.1);
            p.setPen(n.color.darker(115));
            p.setFont(F(6.8));
            p.drawText(QRectF(xC + wC, ay - 11, gap, 9), Qt::AlignCenter,
                       n.title == QStringLiteral("Outputs") ? QStringLiteral("switch") : QStringLiteral("steps"));
        }
        DrawNode(core);
        for (const Node &n : left) DrawNode(n);
        for (const Node &n : right) DrawNode(n);
        m_y = top + h + 12;

        // Motor connectors and I/O
        H2(QStringLiteral("Motor connectors"));
        std::vector<std::vector<Cell>> rows;
        for (int c = 0; c < kMotorConnectors; ++c) {
            QStringList who;
            QString master, detail;
            if (m.source == MasterSource::DrivenCrank && UsesCrank(m_p) && m.crankConnector == c) {
                who << QStringLiteral("Crank motor");
                master = QStringLiteral("—");
                detail = QStringLiteral("%1 steps/rev, gear %2 : %3").arg(Num(m.crankPpr, 0)).arg(m.crankGearNum).arg(m.crankGearDen);
            }
            for (const CamAxis &a : m_p.axes) {
                if (a.connector != c) continue;
                who << Q(a.name);
                master = OnTimeline(a) ? QStringLiteral("Timeline") : QStringLiteral("Crank");
                detail = QStringLiteral("%1, %2 counts/%3").arg(DriveText(a.drive), Num(a.countsPerUnit, 3), Q(a.units));
            }
            const bool clash = who.size() > 1;
            rows.push_back({{QStringLiteral("M-%1").arg(c), {}, true},
                            {who.isEmpty() ? QStringLiteral("not used") : who.join(QStringLiteral(" + ")), clash ? kError : (who.isEmpty() ? kMuted : QColor()), clash},
                            {master},
                            {detail, kInk2}});
        }
        Table({{QStringLiteral("Connector"), 1.0}, {QStringLiteral("Drives"), 2.0}, {QStringLiteral("Master"), 1.1}, {QStringLiteral("Details"), 3.4}}, rows);

        H2(QStringLiteral("Inputs and outputs"));
        std::vector<std::vector<Cell>> io;
        const bool encoder = UsesCrank(m_p) && m.source == MasterSource::Encoder;
        for (int k = 0; k < kTriggerInputs; ++k) {
            QStringList use;
            QColor color;
            if (k < kOutputs) {
                int n = 0;
                for (const OutputPulse &o : m_p.outputs) n += o.output == k;
                if (n) use << QStringLiteral("output, %1 pulse%2").arg(n).arg(n == 1 ? "" : "s");
            }
            if (encoder && k >= kEncoderInputFirst && k <= kEncoderInputLast) {
                use << QStringLiteral("Encoder Adapter (%1)").arg(k == 6 ? QStringLiteral("A") : k == 7 ? QStringLiteral("B") : QStringLiteral("index"));
            }
            if (UsesTimeline(m_p) && m.trigger == TriggerSource::DigitalInput && m.triggerInput == k) {
                use << QStringLiteral("timeline trigger, %1 edge").arg(m.triggerRising ? QStringLiteral("rising") : QStringLiteral("falling"));
            }
            if (use.isEmpty()) continue;
            if (use.size() > 1) color = kError;
            io.push_back({{InputName(k), {}, true}, {use.join(QStringLiteral(" + ")), color, use.size() > 1}});
        }
        if (io.empty()) {
            Note(QStringLiteral("No inputs or outputs used."));
        } else {
            Table({{QStringLiteral("Pin"), 1.0}, {QStringLiteral("Used for"), 6.5}}, io);
        }
    }

    // ---- Machine cycle flow charts
    struct FlowLine {
        QColor color;
        QString text;
    };
    struct FlowStep {
        QString head;
        std::vector<FlowLine> lines;
    };

    std::vector<FlowStep> Steps(AxisMaster master) const {
        const bool crank = master == AxisMaster::Crank;
        const double end = crank ? 360.0 : TimelineLength(m_p);
        std::set<double> ev{0.0};
        for (const CamAxis &a : m_p.axes) {
            if (a.master != master) continue;
            for (int k = 1; k < KnotCount(a); ++k) {
                if (KnotDeg(a, k) < end - 1e-9) ev.insert(KnotDeg(a, k));
            }
        }
        for (const OutputPulse &o : m_p.outputs) {
            if (o.master != master) continue;
            if (o.on > 1e-9 && o.on < end - 1e-9) ev.insert(o.on);
            if (o.off > 1e-9 && o.off < end - 1e-9) ev.insert(o.off);
        }
        // Merge events closer than a hair (rounding).
        std::vector<double> xs;
        for (double x : ev) {
            if (xs.empty() || x - xs.back() > 1e-6 * std::max(1.0, end)) xs.push_back(x);
        }
        xs.push_back(end);
        const double perSec = crank ? DegPerSecond(m_p.master) : m_p.master.speedOverride;
        std::vector<FlowStep> steps;
        for (size_t s = 0; s + 1 < xs.size(); ++s) {
            const double x0 = xs[s], x1 = xs[s + 1], mid = 0.5 * (x0 + x1);
            FlowStep st;
            QString range = crank ? QStringLiteral("%1° – %2°").arg(Num(x0, 1), Num(x1, 1))
                                  : QStringLiteral("%1 s – %2 s").arg(Num(x0, 3), Num(x1, 3));
            if (crank && m_p.master.lineMode) {  // cut-length mode: where on the line, as drawn
                const double mmPerDeg = m_p.master.designLength / 360.0;
                range += QStringLiteral("   ·   %1 – %2 mm of line").arg(Num(x0 * mmPerDeg, 1), Num(x1 * mmPerDeg, 1));
            }
            const QString len = perSec > 0 ? QStringLiteral("%1 s").arg(Num((x1 - x0) / perSec, 3)) : QString();
            st.head = QStringLiteral("Step %1   ·   %2%3").arg(s + 1).arg(range, len.isEmpty() ? QString() : QStringLiteral("   (%1)").arg(len));
            QStringList holds;
            for (size_t i = 0; i < m_p.axes.size(); ++i) {
                const CamAxis &a = m_p.axes[i];
                if (a.master != master) continue;
                const QString u = Q(a.units);
                auto posAt = [&](double x) {
                    if (crank) return x >= a.span - 1e-12 ? EvaluateSegment(a, static_cast<int>(a.segments.size()) - 1, 1.0).pos : Evaluate(a, x).pos;
                    return TimelineAt(a, x).pos;
                };
                const double p0 = posAt(x0), p1 = posAt(x1);
                const bool resting = !crank && mid >= a.span;
                const int seg = SegmentAt(a, std::min(mid, a.span * 0.999999));
                const MotionLaw law = a.segments[seg].law;
                if (resting || law == MotionLaw::Dwell || std::fabs(p1 - p0) < 1e-9) {
                    holds << QStringLiteral("%1 at %2 %3").arg(Q(a.name), Num(p0, 2), u);
                } else {
                    st.lines.push_back({AxisColor(static_cast<int>(i)),
                                        QStringLiteral("%1 %2 %3 → %4 %5   %6, segment %7")
                                            .arg(Q(a.name), p1 > p0 ? QStringLiteral("advances") : QStringLiteral("returns"), Num(p0, 2), Num(p1, 2), u,
                                                 LawText(law))
                                            .arg(seg + 1)});
                }
            }
            if (!holds.isEmpty()) st.lines.push_back({kMuted, QStringLiteral("Holding: ") + holds.join(QStringLiteral(" · "))});
            QStringList on;
            for (const OutputPulse &o : m_p.outputs) {
                if (o.master == master && mid >= o.on && mid < o.off) on << QStringLiteral("IO-%1").arg(o.output);
            }
            on.removeDuplicates();
            if (!on.isEmpty()) st.lines.push_back({kInk, QStringLiteral("Outputs on: ") + on.join(QStringLiteral(", "))});
            steps.push_back(st);
        }
        return steps;
    }

    double StepHeight(const FlowStep &s, double w) const {
        double h = 20;
        for (const FlowLine &l : s.lines) h += TextH(F(7.8), w - 26, l.text) + 2;
        return h + 6;
    }

    void Flow(AxisMaster master) {
        const bool crank = master == AxisMaster::Crank;
        const std::vector<FlowStep> steps = Steps(master);
        const CamMaster &m = m_p.master;
        const QColor color = crank ? kCrank : kTimeline;
        const double x = m_body.left() + 18, w = m_body.width() - 70;
        const QString startText = crank ? QStringLiteral("Start of the revolution  ·  θ = 0°")
                                        : (m.trigger == TriggerSource::Command ? QStringLiteral("Start command")
                                                                               : QStringLiteral("Trigger: %1").arg(TriggerText(m)));
        const QString endText = crank ? QStringLiteral("End of the revolution  ·  θ = 360°, repeats from 0°")
                                : m.timelineRepeat ? QStringLiteral("End of the cycle  ·  repeats from t = 0")
                                                   : QStringLiteral("Done  ·  axes rest at their end positions; waits for the next trigger");
        const double pill = 22, arrow = 14;
        double total = pill + arrow + pill;
        for (const FlowStep &s : steps) total += StepHeight(s, w) + arrow;
        const bool oneBlock = total <= m_body.height();
        if (oneBlock) Need(total);
        else Need(pill + arrow + (steps.empty() ? 0 : StepHeight(steps[0], w)));
        QPainter &p = *m_painter;
        auto drawPill = [&](double y, const QString &t, bool diamond) {
            const QRectF r(x, y, w, pill);
            if (diamond) {
                // A decision: wait here until it happens.
                p.setPen(QPen(color, 1.0));
                p.setBrush(Tint(color, 0.10));
                const double dx = 11;
                p.drawPolygon(QPolygonF() << QPointF(r.left() + dx, r.top()) << QPointF(r.right() - dx, r.top()) << QPointF(r.right(), r.center().y())
                                          << QPointF(r.right() - dx, r.bottom()) << QPointF(r.left() + dx, r.bottom()) << QPointF(r.left(), r.center().y()));
                p.setBrush(Qt::NoBrush);
            } else {
                p.setPen(Qt::NoPen);
                p.setBrush(color);
                p.drawRoundedRect(r, pill / 2, pill / 2);
                p.setBrush(Qt::NoBrush);
            }
            p.setPen(diamond ? color.darker(130) : Qt::white);
            p.setFont(F(8.4, QFont::DemiBold));
            p.drawText(r, Qt::AlignCenter, t);
            return r;
        };
        const QRectF startR = drawPill(m_y, startText, !crank);
        m_y += pill;
        const double startY = startR.center().y();
        for (const FlowStep &s : steps) {
            const double h = StepHeight(s, w);
            if (!Fits(arrow + h)) {
                Arrow(QPointF(x + w / 2, m_y), QPointF(x + w / 2, m_y + 10), color);
                p.setPen(kMuted);
                p.setFont(F(7, QFont::Normal, true));
                p.drawText(QRectF(x + w / 2 + 6, m_y, 200, 10), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("continued on the next page"));
                NewPage();
                m_y += 4;
            } else {
                Arrow(QPointF(x + w / 2, m_y), QPointF(x + w / 2, m_y + arrow), color);
                m_y += arrow;
            }
            const QRectF r(x, m_y, w, h);
            Box(r, kRule, Qt::white, 3);
            p.fillRect(QRectF(r.left() + 0.5, r.top() + 0.5, r.width() - 1, 16), Tint(color, 0.10));
            p.setPen(color.darker(135));
            p.setFont(F(7.8, QFont::DemiBold));
            p.drawText(QRectF(r.left() + 9, r.top() + 1, r.width() - 18, 15), Qt::AlignLeft | Qt::AlignVCenter, s.head);
            double ly = r.top() + 20;
            for (const FlowLine &l : s.lines) {
                const double lh = TextH(F(7.8), w - 26, l.text);
                p.setPen(Qt::NoPen);
                p.setBrush(l.color);
                p.drawEllipse(QPointF(r.left() + 12, ly + 5.5), 2.4, 2.4);
                p.setBrush(Qt::NoBrush);
                p.setPen(l.color == kMuted ? kMuted : kInk);
                p.setFont(F(7.8));
                p.drawText(QRectF(r.left() + 20, ly, w - 26, lh + 1), Qt::TextWordWrap, l.text);
                ly += lh + 2;
            }
            m_y += h;
        }
        if (!Fits(arrow + pill)) NewPage();
        Arrow(QPointF(x + w / 2, m_y), QPointF(x + w / 2, m_y + arrow), color);
        m_y += arrow;
        const QRectF endR = drawPill(m_y, endText, false);
        m_y += pill;
        // The loop back to the start (when it all fits on one page).
        if (oneBlock && (crank || m.timelineRepeat || m.trigger != TriggerSource::Command)) {
            const double lx = x + w + 22;
            p.setPen(QPen(color, 1.1));
            p.drawLine(QPointF(endR.right(), endR.center().y()), QPointF(lx, endR.center().y()));
            p.drawLine(QPointF(lx, endR.center().y()), QPointF(lx, startY));
            Arrow(QPointF(lx, startY), QPointF(startR.right() + 1, startY), color, 1.1);
            p.save();
            p.translate(lx + 10, (startY + endR.center().y()) / 2);
            p.rotate(-90);
            p.setPen(color.darker(120));
            p.setFont(F(7.2, QFont::DemiBold));
            p.drawText(QRectF(-80, -6, 160, 12), Qt::AlignCenter,
                       crank ? QStringLiteral("every revolution") : m.timelineRepeat ? QStringLiteral("repeats") : QStringLiteral("next trigger"));
            p.restore();
        }
        m_y += 16;
    }

    void Cycle() {
        if (!UsesCrank(m_p) && !UsesTimeline(m_p)) return;
        Section(QStringLiteral("Machine cycle"));
        Para(QStringLiteral("The cycle as a sequence of steps. A new step starts wherever any axis starts a new segment or an "
                            "output switches. Each step lists the axes that move, the ones that hold, and the outputs that are on. "
                            "Times are at the master speed."));
        if (UsesCrank(m_p)) {
            H2(QStringLiteral("One crank revolution"), 100);
            Flow(AxisMaster::Crank);
        }
        if (UsesTimeline(m_p)) {
            H2(QStringLiteral("One timeline run"), 100);
            Flow(AxisMaster::Timeline);
        }
    }

    // ---- Timing diagram (lanes)
    void Lanes(AxisMaster master) {
        const bool crank = master == AxisMaster::Crank;
        const double end = crank ? 360.0 : TimelineLength(m_p);
        if (!(end > 0)) return;
        std::vector<int> axes;
        for (size_t i = 0; i < m_p.axes.size(); ++i) {
            if (m_p.axes[i].master == master) axes.push_back(static_cast<int>(i));
        }
        std::set<int> outs;
        for (const OutputPulse &o : m_p.outputs) {
            if (o.master == master) outs.insert(o.output);
        }
        const double laneH = m_laneH, outH = 13, labelW = 108;
        const double h = axes.size() * laneH + (outs.empty() ? 0 : 8 + outs.size() * outH) + 26;
        Need(h);
        QPainter &p = *m_painter;
        const QRectF area(m_body.left() + labelW, m_y, m_body.width() - labelW, h - 26);
        auto X = [&](double x) { return area.left() + x / end * area.width(); };
        // Grid
        const double xs = crank ? 30.0 : NiceStep(end, 10);
        const int xd = StepDecimals(xs);
        p.setFont(F(6.8));
        for (double x = 0; x <= end + xs * 1e-6; x += xs) {
            p.setPen(QPen(kGrid, 0.5));
            p.drawLine(QPointF(X(x), area.top()), QPointF(X(x), area.bottom()));
            p.setPen(kMuted);
            p.drawText(QRectF(X(x) - 20, area.bottom() + 3, 40, 10), Qt::AlignHCenter | Qt::AlignTop,
                       Num(x, xd) + (crank ? QStringLiteral("°") : QString()));
        }
        p.setFont(F(7.2));
        p.drawText(QRectF(area.left(), area.bottom() + 13, area.width(), 11), Qt::AlignHCenter | Qt::AlignVCenter,
                   crank ? QStringLiteral("Crank angle θ") : QStringLiteral("Timeline t (s, as drawn)"));
        double y = area.top();
        for (int i : axes) {
            const CamAxis &a = m_p.axes[i];
            const AxisStats &st = m_pi.axes[i].stats;
            const QRectF lane(area.left(), y + 4, area.width(), laneH - 8);
            // Segments: motion tinted, dwell plain.
            for (int s = 0; s < static_cast<int>(a.segments.size()); ++s) {
                const double s0 = SegmentStartDeg(a, s), s1 = a.segments[s].endDeg;
                if (a.segments[s].law != MotionLaw::Dwell) p.fillRect(QRectF(X(s0), lane.top(), X(s1) - X(s0), lane.height()), Tint(AxisColor(i), 0.10));
                p.setPen(QPen(kRule, 0.5, Qt::DotLine));
                p.drawLine(QPointF(X(s1), lane.top()), QPointF(X(s1), lane.bottom()));
            }
            const double range = std::max(st.maxPos - st.minPos, 1e-12);
            QPainterPath path;
            const int n = 480;
            for (int k = 0; k <= n; ++k) {
                const double x = end * k / n;
                const double pos = crank ? SampleAt(a, k, n).pos : TimelineAt(a, x).pos;
                const QPointF q(X(x), lane.bottom() - (pos - st.minPos) / range * lane.height());
                if (k == 0) path.moveTo(q);
                else path.lineTo(q);
            }
            p.setPen(QPen(AxisColor(i), 1.4));
            p.drawPath(path);
            p.setPen(QPen(kRule, 0.6));
            p.drawRect(lane);
            p.setPen(kInk);
            p.setFont(F(8, QFont::DemiBold));
            p.drawText(QRectF(m_body.left(), lane.top(), labelW - 8, 13), Qt::AlignLeft | Qt::AlignVCenter,
                       Fm(F(8, QFont::DemiBold)).elidedText(Q(a.name), Qt::ElideRight, labelW - 8));
            p.setPen(kMuted);
            p.setFont(F(7));
            p.drawText(QRectF(m_body.left(), lane.top() + 13, labelW - 8, 22), Qt::TextWordWrap,
                       QStringLiteral("M-%1 · %2 – %3 %4").arg(a.connector).arg(Num(st.minPos, 1), Num(st.maxPos, 1), Q(a.units)));
            y += laneH;
        }
        if (!outs.empty()) {
            y += 8;
            for (int k : outs) {
                const QRectF lane(area.left(), y + 2, area.width(), outH - 4);
                p.fillRect(lane, QColor(0xf7, 0xf8, 0xf9));
                for (const OutputPulse &o : m_p.outputs) {
                    if (o.master != master || o.output != k) continue;
                    p.fillRect(QRectF(X(o.on), lane.top(), std::max(0.8, X(std::min(o.off, end)) - X(o.on)), lane.height()), OutputColor(k));
                }
                p.setPen(kInk2);
                p.setFont(F(7.4, QFont::DemiBold));
                p.drawText(QRectF(m_body.left(), y, labelW - 8, outH), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("IO-%1").arg(k));
                y += outH;
            }
        }
        m_y += h + 6;
    }

    void Timing() {
        if (!UsesCrank(m_p) && !UsesTimeline(m_p)) return;
        Section(QStringLiteral("Timing diagram"));
        Para(QStringLiteral("Every axis on its master, each scaled to its own stroke (labels give the range), with motion "
                            "segments shaded and the output pulses below. Use it to check the order of events and the clearances "
                            "between axes."));
        // Lanes as tall as the page allows (46..90 pt each).
        {
            int lanes = 0, outLanes = 0, groups = 0;
            for (AxisMaster mm : {AxisMaster::Crank, AxisMaster::Timeline}) {
                if (mm == AxisMaster::Crank ? !UsesCrank(m_p) : !UsesTimeline(m_p)) continue;
                ++groups;
                std::set<int> outs;
                for (const OutputPulse &o : m_p.outputs) {
                    if (o.master == mm) outs.insert(o.output);
                }
                outLanes += static_cast<int>(outs.size());
                for (const CamAxis &a : m_p.axes) lanes += a.master == mm;
            }
            const double fixed = groups * (28 + 26 + 8 + 6) + outLanes * 13;
            m_laneH = lanes ? std::clamp((m_body.bottom() - m_y - fixed - 50) / lanes, 46.0, 120.0) : 46.0;
        }
        if (UsesCrank(m_p)) {
            H2(QStringLiteral("Crank axes and outputs"), 120);
            Lanes(AxisMaster::Crank);
        }
        if (UsesTimeline(m_p)) {
            H2(QStringLiteral("Timeline axes and outputs"), 120);
            Lanes(AxisMaster::Timeline);
        }
    }

    // ---- Master
    void MasterSection() {
        Section(QStringLiteral("Master"), 1, 260);
        Para(QStringLiteral("What every axis follows: the crank (an external encoder or a crank motor the ClearCore drives) and "
                            "the timeline (seconds from a trigger)."));
        Table({{QStringLiteral("Parameter"), 2.4}, {QStringLiteral("Value"), 1.5, Qt::AlignRight}, {QStringLiteral("Unit"), 0.9},
               {QStringLiteral("Note"), 3.6}},
              RowCells(MasterRows(m_p, m_pi), true));
    }

    // ---- Axes
    void AxisSection(int i) {
        const CamAxis &a = m_p.axes[i];
        const AxisInfo &ai = m_pi.axes[i];
        const QString u = Q(a.units);
        const bool timeline = OnTimeline(a);
        Section(QStringLiteral("Axis: %1").arg(Q(a.name)));
        QPainter &p = *m_painter;
        m_chipX = m_body.left();
        Chip(m_chipX, m_y, QStringLiteral("M-%1").arg(a.connector), AxisColor(i));
        Chip(m_chipX, m_y, timeline ? QStringLiteral("Timeline") : QStringLiteral("Crank"), timeline ? kTimeline : kCrank);
        Chip(m_chipX, m_y, DriveText(a.drive), kInk2);
        Chip(m_chipX, m_y, a.closed ? QStringLiteral("Closed profile") : QStringLiteral("Open profile"), kInk2);
        int errs = 0, warns = 0;
        for (const Check &c : m_pi.checks) {
            if (c.where != Q(a.name)) continue;
            (c.level == Check::Error ? errs : warns)++;
        }
        Chip(m_chipX, m_y, errs ? QStringLiteral("%1 error%2").arg(errs).arg(errs == 1 ? "" : "s")
                           : warns ? QStringLiteral("%1 warning%2").arg(warns).arg(warns == 1 ? "" : "s")
                                   : QStringLiteral("No issues"),
             errs ? kError : warns ? kWarn : kGood);
        m_y += 24;

        // Settings | results, side by side.
        const double gap = 14, half = (m_body.width() - gap) / 2;
        const double top = m_y;
        p.setPen(kInk);
        p.setFont(F(10, QFont::DemiBold));
        p.drawText(QRectF(m_body.left(), m_y, half, 14), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("Settings"));
        p.drawText(QRectF(m_body.left() + half + gap, m_y, half, 14), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("Results"));
        m_y += 19;
        const double y0 = m_y;
        Table({{QStringLiteral("Parameter"), 2.0}, {QStringLiteral("Value"), 1.3, Qt::AlignRight}, {QStringLiteral("Unit"), 0.9}},
              RowCells(AxisSettingsRows(a), false), 7.6, m_body.left(), half, 2.6);
        const double yLeft = m_y;
        m_y = y0;
        std::vector<std::vector<Cell>> stats;
        for (const StatRow &s : AxisStatRows(m_p, i, ai)) {
            stats.push_back({{s.key, kInk2}, {std::isfinite(s.v) ? FormatValue(s.v, s.fmt) : QStringLiteral("no limit"), {}, true},
                             {s.unit, kMuted}, {s.at, kMuted}});
        }
        Table({{QStringLiteral("Measure"), 2.0}, {QStringLiteral("Value"), 1.3, Qt::AlignRight}, {QStringLiteral("Unit"), 0.8},
               {QStringLiteral("At"), 0.9}},
              stats, 7.6, m_body.left() + half + gap, half, 2.6);
        m_y = std::max(yLeft, m_y);
        (void)top;
        Note(timeline ? QStringLiteral("Per-second values at the speed override (%1 %).").arg(Num(m_p.master.speedOverride * 100.0, 1))
                      : QStringLiteral("Per-second values at %1 rpm (%2°/s).").arg(Num(m_p.master.rpm, 1), Num(DegPerSecond(m_p.master), 1)));

        // Checks for this axis
        std::vector<std::vector<Cell>> checks;
        for (const Check &c : m_pi.checks) {
            if (c.where != Q(a.name)) continue;
            checks.push_back({{c.level == Check::Error ? QStringLiteral("Error") : QStringLiteral("Warning"), c.level == Check::Error ? kError : kWarn, true},
                              {c.text}});
        }
        if (checks.empty()) {
            Callout(kGood, QStringLiteral("Smooth"), QStringLiteral("Velocity and acceleration are continuous at every knot, and no limit is exceeded."));
        } else {
            H2(QStringLiteral("Checks"), 40);
            Table({{QStringLiteral("Level"), 1.0}, {QStringLiteral("Check"), 7.0}}, checks);
        }

        // Segments
        H2(QStringLiteral("Segments"), 50);
        std::vector<std::vector<Cell>> segs;
        const int dx = timeline ? 3 : 2;
        for (int s = 0; s < static_cast<int>(a.segments.size()); ++s) {
            const Peaks pk = SegmentPeaks(a, s);
            const double x0 = SegmentStartDeg(a, s), x1 = a.segments[s].endDeg;
            const double p0 = SegmentStartPos(a, s), p1 = a.segments[s].endPos;
            const bool dwell = a.segments[s].law == MotionLaw::Dwell;
            segs.push_back({{QString::number(s + 1), AxisColor(i), true},
                            {LawText(a.segments[s].law), dwell ? kMuted : QColor()},
                            {Num(x0, dx)}, {Num(x1, dx)}, {Num(x1 - x0, dx)},
                            {Num(p0, 3)}, {Num(p1, 3)}, {dwell ? QStringLiteral("—") : Num(p1 - p0, 3), dwell ? kMuted : QColor()},
                            {Num(pk.vel * ai.w, 2)}, {Num(pk.acc * ai.w * ai.w, 1)}, {Num(pk.jerk * ai.w * ai.w * ai.w, 0)}});
        }
        const QString xun = timeline ? QStringLiteral("s") : QStringLiteral("°");
        Table({{QStringLiteral("#"), 0.45, Qt::AlignCenter}, {QStringLiteral("Law"), 1.9},
               {QStringLiteral("Start\n%1").arg(xun), 1.0, Qt::AlignRight}, {QStringLiteral("End\n%1").arg(xun), 1.0, Qt::AlignRight},
               {QStringLiteral("Length\n%1").arg(xun), 1.05, Qt::AlignRight}, {QStringLiteral("From\n%1").arg(u), 1.05, Qt::AlignRight},
               {QStringLiteral("To\n%1").arg(u), 1.05, Qt::AlignRight}, {QStringLiteral("Move\n%1").arg(u), 1.05, Qt::AlignRight},
               {QStringLiteral("Peak vel.\n%1/s").arg(u), 1.15, Qt::AlignRight}, {QStringLiteral("Peak acc.\n%1/s²").arg(u), 1.25, Qt::AlignRight},
               {QStringLiteral("Peak jerk\n%1/s³").arg(u), 1.25, Qt::AlignRight}},
              segs, 7.5);

        // Motion: displacement, velocity, acceleration, jerk
        Section(QStringLiteral("%1: displacement and motion").arg(Q(a.name)), 2);
        Motion(i);

        if (!timeline) {
            Section(QStringLiteral("%1: plate cam").arg(Q(a.name)), 2);
            Cam(i);
        }
    }

    void Motion(int i) {
        const CamAxis &a = m_p.axes[i];
        const AxisInfo &ai = m_pi.axes[i];
        const QString u = Q(a.units);
        const bool timeline = OnTimeline(a);
        const double w = ai.w;
        const int n = 720;
        Series pos{{}, AxisColor(i), 1.6}, vel{{}, AxisColor(i), 1.2}, acc{{}, AxisColor(i), 1.2}, jerk{{}, AxisColor(i), 1.0};
        for (int k = 0; k <= n; ++k) {
            const double x = a.span * k / n;
            const CamPoint c = SampleAt(a, k, n);
            pos.pts.emplace_back(x, c.pos);
            vel.pts.emplace_back(x, c.vel * w);
            acc.pts.emplace_back(x, c.acc * w * w);
            jerk.pts.emplace_back(x, c.jerk * w * w * w);
        }
        std::vector<double> knots;
        std::vector<QPointF> dots;
        std::vector<Band> bands, plain;
        for (int k = 0; k < KnotCount(a); ++k) {
            dots.emplace_back(KnotDeg(a, k), KnotPos(a, k));
            if (k > 0 && k < KnotCount(a) - 1) knots.push_back(KnotDeg(a, k));
        }
        for (int s = 0; s < static_cast<int>(a.segments.size()); ++s) {
            const bool dwell = a.segments[s].law == MotionLaw::Dwell;
            const QColor fill = dwell ? QColor(0xf3, 0xf4, 0xf6) : Tint(AxisColor(i), s % 2 ? 0.07 : 0.11);
            bands.push_back({SegmentStartDeg(a, s), a.segments[s].endDeg, fill, QStringLiteral("%1 · %2").arg(s + 1).arg(LawText(a.segments[s].law))});
        }
        const QString xTitle = timeline ? QStringLiteral("Time t (s, as drawn)") : QStringLiteral("Crank angle θ");
        auto base = [&](const QString &title, const QString &unit) {
            Chart c;
            c.title = title;
            c.unit = unit;
            c.x0 = 0;
            c.x1 = a.span;
            c.crank = !timeline;
            c.vlines = knots;
            return c;
        };
        auto peakMark = [&](const Series &s, const QString &unit, int decimals) {
            QPointF best(0, 0);
            for (const QPointF &q : s.pts) {
                if (std::fabs(q.y()) > std::fabs(best.y())) best = q;
            }
            return Mark{best.x(), best.y(), QStringLiteral("%1 %2").arg(Num(best.y(), decimals), unit), AxisColor(i)};
        };

        Chart d = base(QStringLiteral("Displacement"), QStringLiteral("position, %1").arg(u));
        d.bands = bands;
        d.series = {pos};
        d.dots = dots;
        d.dotColor = AxisColor(i).darker(120);
        d.xTitle = xTitle;
        DrawChart(QRectF(m_body.left(), m_y, m_body.width(), 214), d);
        m_y += 214 + 14;

        // Three charts and the note below them share what is left of the page.
        const QString note = QStringLiteral("Shaded: segments (number · motion law); dots: knots; dashed red: the axis's limits. "
                                            "Peaks are marked. Jerk spikes at a knot show an acceleration step.");
        const double noteH = TextH(F(7.8, QFont::Normal, true), m_body.width(), note) + 10;
        const double rest = m_body.bottom() - m_y - noteH - 8;
        const double hv = std::min(150.0, (rest - 2 * 12) / 3);
        Chart v = base(QStringLiteral("Velocity"), QStringLiteral("%1/s").arg(u));
        v.series = {vel};
        if (a.velMax > 0) v.refs = {{a.velMax, QStringLiteral("limit %1").arg(Num(a.velMax, 2)), kError}, {-a.velMax, QString(), kError}};
        v.marks = {peakMark(vel, QStringLiteral("%1/s").arg(u), 2)};
        DrawChart(QRectF(m_body.left(), m_y, m_body.width(), hv), v);
        m_y += hv + 12;
        Chart ac = base(QStringLiteral("Acceleration"), QStringLiteral("%1/s²").arg(u));
        ac.series = {acc};
        if (a.accMax > 0) ac.refs = {{a.accMax, QStringLiteral("limit %1").arg(Num(a.accMax, 1)), kError}, {-a.accMax, QString(), kError}};
        ac.marks = {peakMark(acc, QStringLiteral("%1/s²").arg(u), 1)};
        DrawChart(QRectF(m_body.left(), m_y, m_body.width(), hv), ac);
        m_y += hv + 12;
        Chart j = base(QStringLiteral("Jerk"), QStringLiteral("%1/s³").arg(u));
        j.series = {jerk};
        j.marks = {peakMark(jerk, QStringLiteral("%1/s³").arg(u), 0)};
        j.xTitle = xTitle;
        DrawChart(QRectF(m_body.left(), m_y, m_body.width(), hv), j);
        m_y += hv + 8;
        Note(note);
    }

    void Cam(int i) {
        const CamAxis &a = m_p.axes[i];
        const QString u = Q(a.units);
        const CamShape shape = BuildCamShape(a, 1440);
        QPainter &p = *m_painter;
        Para(QStringLiteral("The crank profile as the mechanical plate cam it replaces: a translating roller follower at 12 "
                            "o'clock, the cam turning clockwise. Base and roller radius only shape this drawing and the DXF "
                            "export; the ClearCore follows the profile itself."));
        const double size = std::min(m_body.width() * 0.62, 300.0);
        Need(size + 30);
        const QRectF box(m_body.left() + (m_body.width() - size) / 2, m_y, size, size);
        p.setPen(QPen(kGrid, 0.6));
        p.drawRect(box);
        const QPointF c = box.center() + QPointF(0, 10);
        const double reach = shape.maxRadius + 2.2 * std::max(shape.rollerRadius, 0.0) + 1e-9;
        const double k = (size / 2 - 22) / reach;
        auto S = [&](const Pt &q) { return QPointF(c.x() + q.x * k, c.y() - q.y * k); };
        // Centre lines
        p.setPen(QPen(kRule, 0.5, Qt::DashDotLine));
        p.drawLine(QPointF(box.left() + 8, c.y()), QPointF(box.right() - 8, c.y()));
        p.drawLine(QPointF(c.x(), box.top() + 8), QPointF(c.x(), box.bottom() - 8));
        // Surface, pitch curve, base circle
        QPainterPath surf, pitch;
        for (size_t j = 0; j < shape.surface.size(); ++j) {
            if (j == 0) surf.moveTo(S(shape.surface[j]));
            else surf.lineTo(S(shape.surface[j]));
        }
        surf.closeSubpath();
        for (size_t j = 0; j < shape.pitch.size(); ++j) {
            if (j == 0) pitch.moveTo(S(shape.pitch[j]));
            else pitch.lineTo(S(shape.pitch[j]));
        }
        p.setPen(QPen(kInk, 1.1));
        p.setBrush(QColor(0xe9, 0xed, 0xf2));
        p.drawPath(surf);
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(AxisColor(i), 0.8, Qt::DashLine));
        p.drawPath(pitch);
        p.setPen(QPen(kMuted, 0.6, Qt::DotLine));
        p.drawEllipse(c, shape.baseRadius * k, shape.baseRadius * k);
        // Bore and the follower at 0°
        p.setPen(QPen(kInk, 0.8));
        p.drawEllipse(c, 4, 4);
        Pt pt, nn;
        PitchPointAndNormal(a, shape.minPos, 0.0, false, &pt, &nn);
        const QPointF roller = S(pt);
        const double rr = std::max(shape.rollerRadius * k, 1.5);
        p.setPen(QPen(kInk2, 1.0));
        p.fillRect(QRectF(roller.x() - 3, box.top() + 6, 6, roller.y() - box.top() - 6), QColor(0xd6, 0xda, 0xe0));
        p.drawRect(QRectF(roller.x() - 3, box.top() + 6, 6, roller.y() - box.top() - 6));
        p.setBrush(Qt::white);
        p.drawEllipse(roller, rr, rr);
        p.setBrush(Qt::NoBrush);
        // Turning direction
        const double ar = std::min(18.0, shape.baseRadius * k * 0.5);
        QPainterPath turn;
        turn.arcMoveTo(QRectF(c.x() - ar, c.y() - ar, 2 * ar, 2 * ar), 120);
        turn.arcTo(QRectF(c.x() - ar, c.y() - ar, 2 * ar, 2 * ar), 120, -100);
        p.setPen(QPen(kAccent, 0.9));
        p.drawPath(turn);
        const QPointF tip = turn.currentPosition();
        Arrow(tip + QPointF(-3, -3), tip + QPointF(1, 1), kAccent, 0.9);
        m_y += size + 8;
        p.setPen(kInk2);
        p.setFont(F(8));
        p.drawText(QRectF(m_body.left(), m_y, m_body.width(), 12), Qt::AlignHCenter | Qt::AlignVCenter,
                   QStringLiteral("Base radius %1 %5 · roller radius %2 %5 · stroke %3 %5 · largest radius %4 %5")
                       .arg(Num(shape.baseRadius, 2), Num(shape.rollerRadius, 2), Num(shape.stroke, 2), Num(shape.maxRadius, 2), u));
        m_y += 22;

        // Pressure angle over the revolution
        Series pa{{}, AxisColor(i), 1.3};
        for (int kk = 0; kk <= 720; ++kk) {
            const double th = 360.0 * kk / 720;
            Pt q, nrm;
            PitchPointAndNormal(a, shape.minPos, th, kk == 720, &q, &nrm);
            const double r = std::hypot(q.x, q.y);
            if (r < 1e-12) continue;
            const double rx = q.x / r, ry = q.y / r;
            double phi = std::atan2(rx * nrm.y - ry * nrm.x, rx * nrm.x + ry * nrm.y) * 180.0 / 3.14159265358979323846;
            if (phi > 90) phi -= 180;
            if (phi < -90) phi += 180;
            pa.pts.emplace_back(th, phi);
        }
        Chart ch;
        ch.title = QStringLiteral("Pressure angle");
        ch.unit = QStringLiteral("degrees");
        ch.series = {pa};
        ch.refs = {{kMaxPressureAngleDeg, QStringLiteral("±%1° guideline").arg(Num(kMaxPressureAngleDeg, 0)), kWarn},
                   {-kMaxPressureAngleDeg, QString(), kWarn}};
        ch.xTitle = QStringLiteral("Crank angle θ");
        double peak = 0;
        QPointF at;
        for (const QPointF &q : pa.pts) {
            if (std::fabs(q.y()) > std::fabs(peak)) {
                peak = q.y();
                at = q;
            }
        }
        ch.marks = {{at.x(), at.y(), QStringLiteral("%1°").arg(Num(peak, 1)), AxisColor(i)}};
        Need(170);
        DrawChart(QRectF(m_body.left(), m_y, m_body.width(), 160), ch);
        m_y += 168;
        Note(QStringLiteral("Keep the pressure angle within about ±30° for a translating roller follower; a larger base radius lowers it."));
    }

    // ---- Outputs, checks, sign-off
    void OutputsSection() {
        Section(QStringLiteral("Outputs"), 1, 200);
        Para(QStringLiteral("IO-0..IO-5 switched by pulses, for the whole project. A crank pulse is on between two crank angles "
                            "every revolution; a timeline pulse between two times of each timeline run. Every output is off while "
                            "its master is not running."));
        if (m_p.outputs.empty()) {
            Note(QStringLiteral("No output pulses in this project."));
            return;
        }
        std::vector<std::vector<Cell>> rows;
        for (const OutputPulse &o : m_p.outputs) {
            const bool crank = o.master == AxisMaster::Crank;
            const int d = crank ? 1 : 3;
            const QString unit = crank ? QStringLiteral("°") : QStringLiteral(" s");
            const double perSec = crank ? DegPerSecond(m_p.master) : m_p.master.speedOverride;
            rows.push_back({{QStringLiteral("IO-%1").arg(o.output), OutputColor(o.output), true},
                            {crank ? QStringLiteral("Crank") : QStringLiteral("Timeline"), crank ? kCrank : kTimeline},
                            {Num(o.on, d) + unit}, {Num(o.off, d) + unit}, {Num(o.off - o.on, d) + unit},
                            {perSec > 0 ? Num((o.off - o.on) / perSec * 1000.0, 1) + QStringLiteral(" ms") : QStringLiteral("—")}});
        }
        Table({{QStringLiteral("Output"), 1.0}, {QStringLiteral("Master"), 1.2}, {QStringLiteral("On"), 1.2, Qt::AlignRight},
               {QStringLiteral("Off"), 1.2, Qt::AlignRight}, {QStringLiteral("Length"), 1.2, Qt::AlignRight},
               {QStringLiteral("On for"), 1.3, Qt::AlignRight}},
              rows);
        Note(QStringLiteral("On for: at the master speed (%1 rpm; timeline at %2 %).").arg(Num(m_p.master.rpm, 1), Num(m_p.master.speedOverride * 100.0, 0)));
    }

    void ChecksSection() {
        Section(QStringLiteral("Checks"), 1, 200);
        Para(QStringLiteral("Every check the app makes: motor connectors, master speed against what the axes allow, the trigger "
                            "input, each axis's limits, velocity and acceleration continuity at the knots, and the cam drawing."));
        if (m_pi.checks.empty()) {
            Callout(kGood, QStringLiteral("No issues found"),
                    QStringLiteral("No limit exceeded; velocity and acceleration are continuous at every knot."));
            return;
        }
        std::vector<std::vector<Cell>> rows;
        for (const Check &c : m_pi.checks) {
            rows.push_back({{c.level == Check::Error ? QStringLiteral("Error") : QStringLiteral("Warning"), c.level == Check::Error ? kError : kWarn, true},
                            {c.where, {}, true},
                            {c.text}});
        }
        Table({{QStringLiteral("Level"), 1.0}, {QStringLiteral("Where"), 1.5}, {QStringLiteral("Check"), 6.0}}, rows);
        Note(QStringLiteral("Errors: a velocity step (infinite acceleration), a limit exceeded, a connector or input used twice. "
                            "Warnings: an acceleration step (infinite jerk) or cam drawing geometry."));
    }

    void SignOff() {
        Section(QStringLiteral("Review and sign-off"), 1, 470);
        Para(QStringLiteral("For the engineer reviewing the project before it runs on the machine."));
        QPainter &p = *m_painter;
        const char *items[] = {"Master source, counts per revolution and direction match the machine",
                               "Each axis's motor connector, drive and counts per unit match the mechanics",
                               "Strokes and positions stay inside the machine's travel",
                               "Peak motor speeds and step rates are inside the motors' ratings",
                               "Velocity and acceleration limits are set and respected",
                               "The order of events and the clearances between axes are safe (timing diagram)",
                               "Output pulses switch the right devices at the right time",
                               "Errors fixed; warnings understood",
                               "First run on the machine at low speed with the emergency stop in reach"};
        H2(QStringLiteral("Checklist"), 120);
        for (const char *t : items) {
            Need(18);
            p.setPen(QPen(kInk2, 0.8));
            p.drawRect(QRectF(m_body.left() + 2, m_y + 2, 9, 9));
            p.setPen(kInk);
            p.setFont(F(9));
            p.drawText(QRectF(m_body.left() + 20, m_y, m_body.width() - 20, 13), Qt::AlignLeft | Qt::AlignVCenter, QString::fromUtf8(t));
            m_y += 18;
        }
        m_y += 8;
        H2(QStringLiteral("Notes"), 120);
        Need(120);
        for (int k = 0; k < 6; ++k) {
            p.setPen(QPen(kRule, 0.7));
            p.drawLine(QPointF(m_body.left(), m_y + 18), QPointF(m_body.right(), m_y + 18));
            m_y += 20;
        }
        m_y += 14;
        Need(110);
        const char *fields[] = {"Reviewed by", "Date", "Signature", "Approved by", "Date", "Signature"};
        const double colW = (m_body.width() - 20) / 3;
        for (int k = 0; k < 6; ++k) {
            const double x = m_body.left() + (k % 3) * (colW + 10), y = m_y + (k / 3) * 46;
            p.setPen(QPen(kInk2, 0.8));
            p.drawLine(QPointF(x, y + 24), QPointF(x + colW, y + 24));
            p.setPen(kMuted);
            p.setFont(F(7.5));
            p.drawText(QRectF(x, y + 27, colW, 10), Qt::AlignLeft | Qt::AlignVCenter, QString::fromUtf8(fields[k]));
        }
        m_y += 100;
    }

    const CamProject &m_p;
    const ProjectInfo &m_pi;
    ReportOptions m_o;
    QSizeF m_page;
    QString m_stamp;
    QString m_name;
    QRectF m_body;
    Sink *m_sink = nullptr;
    QPainter *m_painter = nullptr;
    int m_pageNo = 0;
    int m_total = 0;
    int m_section = 0;
    int m_sub = 0;
    double m_laneH = 46;
    double m_y = 0;
    double m_chipX = 0;
    bool m_failed = false;
    std::vector<TocEntry> m_toc, m_tocKnown;
};

QPageSize PaperFor(ReportOptions::Paper paper) {
    if (paper == ReportOptions::Paper::Letter) return QPageSize(QPageSize::Letter);
    if (paper == ReportOptions::Paper::A4) return QPageSize(QPageSize::A4);
    const QLocale::Country c = QLocale::system().territory();
    const bool letter = c == QLocale::UnitedStates || c == QLocale::Canada || c == QLocale::Mexico || c == QLocale::Philippines ||
                        c == QLocale::Chile || c == QLocale::Colombia;
    return QPageSize(letter ? QPageSize::Letter : QPageSize::A4);
}

QString Stamp() { return QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm")); }

// First pass: page count and section pages, for the footers and the contents.
std::pair<int, std::vector<TocEntry>> LayOut(const CamProject &project, const ProjectInfo &pi, const ReportOptions &o, QSizeF page,
                                             const QString &stamp) {
    ImageSink dry(page, 72, false);
    Report r(project, pi, o, page, stamp);
    r.Render(dry, 0, {});
    // Twice: the contents on the cover could, in principle, move the pages that follow.
    const std::vector<TocEntry> toc = r.toc();
    const int pages = r.pages();
    Report r2(project, pi, o, page, stamp);
    ImageSink dry2(page, 72, false);
    r2.Render(dry2, pages, toc);
    return {r2.pages(), r2.toc()};
}

}  // namespace

bool ExportReport(const CamProject &projectIn, const ReportOptions &options, const QString &path, QString *error) {
    CamProject project = projectIn;
    NormalizeProject(project);
    const ProjectInfo pi = AnalyzeProject(project);
    const QPageSize paper = PaperFor(options.paper);
    const QSizeF page = paper.size(QPageSize::Point);
    const QString stamp = Stamp();
    const auto [pages, toc] = LayOut(project, pi, options, page, stamp);

    const QString part = path + QStringLiteral(".part");
    QFile::remove(part);
    bool ok = false;
    {
        PdfSink pdf(part, paper, QStringLiteral("%1 — project report").arg(options.projectName.isEmpty() ? QStringLiteral("Untitled") : options.projectName));
        Report r(project, pi, options, page, stamp);
        ok = r.Render(pdf, pages, toc);
        ok = pdf.Finish() && ok;
    }
    if (!ok || QFileInfo(part).size() < 1000) {
        QFile::remove(part);
        if (error) *error = QStringLiteral("could not write the PDF (is the folder writable?)");
        return false;
    }
    if (QFile::exists(path) && !QFile::remove(path)) {
        QFile::remove(part);
        if (error) *error = QStringLiteral("the file is open in another program; close it and try again");
        return false;
    }
    if (!QFile::rename(part, path)) {
        QFile::remove(part);
        if (error) *error = QStringLiteral("could not rename the finished PDF into place");
        return false;
    }
    return true;
}

std::vector<QImage> RenderReportPages(const CamProject &projectIn, const ReportOptions &options, double dpi) {
    CamProject project = projectIn;
    NormalizeProject(project);
    const ProjectInfo pi = AnalyzeProject(project);
    const QSizeF page = PaperFor(options.paper).size(QPageSize::Point);
    const QString stamp = Stamp();
    const auto [pages, toc] = LayOut(project, pi, options, page, stamp);
    ImageSink images(page, dpi, true);
    Report r(project, pi, options, page, stamp);
    r.Render(images, pages, toc);
    return std::move(images.pages());
}

}  // namespace io
