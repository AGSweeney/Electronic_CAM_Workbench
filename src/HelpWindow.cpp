/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "HelpWindow.h"

#include "Brand.h"
#include "Theme.h"

#include <QApplication>
#include <QDate>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QImage>
#include <QLineEdit>
#include <QMessageBox>
#include <QAbstractTextDocumentLayout>
#include <QHash>
#include <QPageLayout>
#include <QPageSize>
#include <QPainter>
#include <QPointer>
#include <QPrintDialog>
#include <QPrinter>
#include <QSvgRenderer>
#include <QLocale>
#include <QPainterPath>
#include <QFontMetricsF>
#include <QMap>
#include <functional>

#include <cmath>
#include <QPushButton>
#include <QRegularExpression>
#include <QSet>
#include <QSettings>
#include <QSplitter>
#include <QTextBrowser>
#include <QTextBlock>
#include <QTextCursor>
#include <QTextDocument>
#include <QTextLayout>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <memory>

namespace {

QSettings &Settings() {
    static QSettings s(QStringLiteral("ECamUI"), QStringLiteral("ECamUI"));
    return s;
}

QString ReadResource(const QString &path) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return {};
    return QString::fromUtf8(f.readAll());
}

// On screen: the app's dark look.
const char *kScreenCss = R"CSS(
body { color: #d8dee6; font-size: 10.5pt; }
h1 { color: #f0f4f8; font-size: 20pt; font-weight: 700; margin-top: 4px; margin-bottom: 8px; }
h2 { color: #8fc6ff; font-size: 14pt; font-weight: 700; margin-top: 18px; margin-bottom: 6px; }
h3 { color: #e6ebf3; font-size: 11.5pt; font-weight: 700; margin-top: 12px; margin-bottom: 4px; }
p, li { color: #d8dee6; }
a { color: #8fc6ff; }
b { color: #f0f4f8; }
i { color: #c3cbd6; }
kbd { color: #f0f4f8; background-color: #2c333c; font-family: Consolas, monospace; }
code { color: #ffd68a; font-family: Consolas, monospace; }
th { color: #9fa6af; background-color: #262c34; text-align: left; padding: 4px 10px; border-bottom: 1px solid #3a424d; }
td { padding: 4px 10px; border-bottom: 1px solid #2c333c; vertical-align: top; }
p.note { background-color: #1e3048; color: #dbe8f7; padding: 8px 12px; }
p.warn { background-color: #3d2a1c; color: #ffd9b8; padding: 8px 12px; }
p.caption { color: #9fa6af; font-size: 9pt; margin-top: 2px; }
)CSS";

// On paper: dark ink on white, compact.
const char *kPrintCss = R"CSS(
body { color: #111111; font-size: 9.5pt; }
h1 { color: #17375e; font-size: 18pt; font-weight: 700; margin-top: 26px; margin-bottom: 6px; }
h2 { color: #17375e; font-size: 13pt; font-weight: 700; margin-top: 14px; margin-bottom: 4px; }
h3 { color: #222222; font-size: 11pt; font-weight: 700; margin-top: 10px; margin-bottom: 3px; }
a { color: #17375e; text-decoration: none; }
kbd { background-color: #e8ecf1; font-family: Consolas, monospace; }
code { color: #7a4b00; font-family: Consolas, monospace; }
th { background-color: #e6ebf2; text-align: left; padding: 3px 8px; border-bottom: 1px solid #9aa6b5; }
td { padding: 3px 8px; border-bottom: 1px solid #d5dbe3; vertical-align: top; }
p.note { background-color: #eef3f9; padding: 6px 10px; }
p.warn { background-color: #fbefe3; padding: 6px 10px; }
p.caption { color: #555555; font-size: 8.5pt; margin-top: 2px; }
p.cover { font-size: 12pt; color: #333333; }
)CSS";

QPointer<HelpWindow> g_help;

// The width each picture is shown at in the guide (its <img width>), from the chapters.
const QHash<QString, int> &ShownWidths() {
    static const QHash<QString, int> w = [] {
        QHash<QString, int> out;
        static const QRegularExpression img(QStringLiteral(R"re(<img src="images/([^"]+)"[^>]*width="(\d+)")re"));
        for (const HelpWindow::Chapter &c : HelpWindow::Chapters()) {
            for (auto it = img.globalMatch(ReadResource(QStringLiteral(":/help/") + c.file)); it.hasNext();) {
                const auto m = it.next();
                out.insert(m.captured(1), m.captured(2).toInt());
            }
        }
        return out;
    }();
    return w;
}

// The pictures are stored at up to twice their shown size. The text browser would shrink them with fast,
// unsmoothed scaling (text and thin lines break up), so hand it each picture already scaled, smoothly,
// to its shown width times the screen's pixel ratio: it then draws it 1:1.
class HelpBrowser : public QTextBrowser {
public:
    using QTextBrowser::QTextBrowser;
    QVariant loadResource(int type, const QUrl &name) override {
        if (type != QTextDocument::ImageResource) return QTextBrowser::loadResource(type, name);
        const QString file = name.fileName();
        QImage img(QStringLiteral(":/help/images/") + file);
        if (img.isNull()) return QTextBrowser::loadResource(type, name);
        const int shown = ShownWidths().value(file, 0);
        if (shown > 0) {
            const qreal dpr = devicePixelRatioF();
            const int target = qRound(shown * dpr);
            if (img.width() != target) img = img.scaledToWidth(target, Qt::SmoothTransformation);
            img.setDevicePixelRatio(dpr);
        }
        return img;
    }
};

// ---- On paper ------------------------------------------------------------------------------------------
// Laid out and paginated here rather than with QTextDocument::print, which adds its own 2 cm margin inside
// the printer's. Equal margins left and right (two-sided printing), the page number in a footer band.
const QMarginsF kPrintMargins(15, 13, 15, 11);  // mm: left, top, right, bottom

struct Pager {
    std::unique_ptr<QTextDocument> doc;
    double width = 0, pageHeight = 0, footer = 0;
};

// The headings (by their order in the document) that start on another page than the first line of what
// follows them: left at the foot of a page while their paragraph, list or picture moved on.
QList<int> Stranded(QTextDocument *doc, double pageHeight) {
    QAbstractTextDocumentLayout *layout = doc->documentLayout();
    auto page = [&](double y) { return static_cast<int>(std::floor(y / pageHeight)); };
    QList<int> out;
    int ordinal = -1;
    for (QTextBlock b = doc->begin(); b.isValid(); b = b.next()) {
        if (b.blockFormat().headingLevel() == 0) continue;
        ++ordinal;
        const QTextBlock n = b.next();
        if (!n.isValid()) continue;
        const QRectF nb = layout->blockBoundingRect(n);
        double firstLineBottom = nb.bottom();
        if (n.layout() && n.layout()->lineCount() > 0) {
            const QTextLine line = n.layout()->lineAt(0);
            firstLineBottom = nb.top() + line.y() + line.height();
        }
        if (page(layout->blockBoundingRect(b).top() + 1) != page(firstLineBottom - 1)) out << ordinal;
    }
    return out;
}

// The run of headings, one straight after another, that heading `k` (by its order in the document) is in: the first
// and the last, by the same order.
std::pair<int, int> HeadingRun(QTextDocument *doc, int k) {
    int ordinal = -1, start = -1;
    bool prevHeading = false;
    for (QTextBlock b = doc->begin(); b.isValid(); b = b.next()) {
        const bool heading = b.blockFormat().headingLevel() != 0;
        if (heading) {
            ++ordinal;
            if (!prevHeading) start = ordinal;
            if (ordinal >= k) {
                // Carry on to the end of the run.
                int end = ordinal;
                for (QTextBlock c = b.next(); c.isValid() && c.blockFormat().headingLevel() != 0; c = c.next()) ++end;
                return {start, end};
            }
        }
        prevHeading = heading;
    }
    return {k, k};
}

// The guide laid out on pages of `area` (device pixels of `device`), each heading kept with what follows
// it: the layout has no "keep with next", so a stranded heading gets a page break before it and the guide
// is built again from its HTML (editing a laid-out document in place leaves stale positions behind). One
// heading per pass, the first one, since moving it moves everything after it.
Pager LayOut(const QString &chapterFile, QPaintDevice *device, const QSizeF &area) {
    Pager pg;
    pg.width = area.width();
    pg.footer = 16.0 * device->logicalDpiY() / 72.0;  // 16 pt for the page number
    pg.pageHeight = area.height() - pg.footer;
    QSet<int> breaks;
    for (int pass = 0; pass < 60; ++pass) {
        pg.doc.reset(HelpWindow::PrintDocument(chapterFile, nullptr, breaks));
        pg.doc->documentLayout()->setPaintDevice(device);
        pg.doc->setDocumentMargin(0);
        pg.doc->setPageSize(QSizeF(pg.width, pg.pageHeight));
        bool added = false;
        for (int k : Stranded(pg.doc.get(), pg.pageHeight)) {
            // Headings one after another (a chapter's title and its first section) go to the new page together: the
            // break goes before the first of them, and the others lose theirs, or the first would stay behind alone.
            const auto run = HeadingRun(pg.doc.get(), k);
            if (breaks.contains(run.first)) continue;  // already starts a page (a heading taller than what is left)
            for (int j = run.first + 1; j <= run.second; ++j) breaks.remove(j);
            breaks.insert(run.first);
            added = true;
            break;
        }
        if (!added) break;
    }
    return pg;
}

// The whole guide's cover, in the page's printable area (device pixels at dpi): the project report's look (a navy band
// with the icon and the title, a blue rule), a cam's displacement curve as the one picture, the version and the date.
void DrawCover(QPainter &p, const QSizeF &area, double dpi) {
    const QColor accent(0x17, 0x37, 0x5e), blue(0x2a, 0x78, 0xd6), soft(0xe9, 0xef, 0xf6);
    const QColor ink2(0x4a, 0x4f, 0x57), muted(0x7d, 0x84, 0x8d), grid(0xe6, 0xe9, 0xed), rule(0xc8, 0xcd, 0xd4);
    // Drawn in points on a painter scaled to the page: a font's size is in points of the page too, so it is given as
    // pt / scale (the device would scale it again), and measured on the device it is drawn on.
    auto font = [dpi](double pt, QFont::Weight w = QFont::Normal) {
        QFont f(QStringLiteral("Segoe UI"));
        f.setPointSizeF(pt * 72.0 / dpi);
        f.setWeight(w);
        f.setHintingPreference(QFont::PreferNoHinting);
        return f;
    };
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::SmoothPixmapTransform);
    const double s = dpi / 72.0;  // drawn in points
    p.scale(s, s);
    const double w = area.width() / s, h = area.height() / s;
    // The band: what it is, and the icon. The title and the line under it measured, so a long title wraps cleanly and
    // the band grows to hold them.
    const QString title = QStringLiteral(ECAM_PRODUCT_NAME);
    const QString blurb = QStringLiteral("Electronic cams, timelines and their motion on the ClearCore: design, check, run and connect to a PLC.");
    const QFont titleFont = font(30, QFont::DemiBold), blurbFont = font(11.5);
    const double textW = w - 44 - 44 - 76 - 24;  // left of the icon
    const QRectF titleBox = QFontMetricsF(titleFont, p.device()).boundingRect(QRectF(0, 0, textW, 1000), Qt::AlignLeft | Qt::TextWordWrap, title);
    const QRectF blurbBox = QFontMetricsF(blurbFont, p.device()).boundingRect(QRectF(0, 0, w - 88, 1000), Qt::AlignLeft | Qt::TextWordWrap, blurb);
    const double titleTop = 74, blurbTop = titleTop + titleBox.height() + 18;
    const double band = std::max(h * 0.30, blurbTop + blurbBox.height() + 40);
    p.fillRect(QRectF(0, 0, w, band), accent);
    p.fillRect(QRectF(0, band, w, 4), blue);
    QSvgRenderer icon(QStringLiteral(":/icons/ecam_app_icon.svg"));
    if (icon.isValid()) icon.render(&p, QRectF(w - 44 - 76, 44, 76, 76));
    QFont caps = font(9, QFont::DemiBold);
    caps.setLetterSpacing(QFont::AbsoluteSpacing, 2.2);
    p.setFont(caps);
    p.setPen(QColor(0x9f, 0xc3, 0xea));
    p.drawText(QRectF(44, 52, textW, 16), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("USER GUIDE"));
    p.setPen(Qt::white);
    p.setFont(titleFont);
    p.drawText(QRectF(44, titleTop, textW, titleBox.height() + 2), Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap, title);
    p.setPen(QColor(0xd5, 0xe0, 0xec));
    p.setFont(blurbFont);
    p.drawText(QRectF(44, blurbTop, w - 88, blurbBox.height() + 2), Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap, blurb);
    // The picture: a rise, dwell, return, dwell on a light grid, the area under it shaded.
    const QRectF plot(44, band + 70, w - 88, h * 0.30);
    p.setPen(QPen(grid, 0.8));
    for (int i = 0; i <= 12; ++i) {
        const double x = plot.left() + plot.width() * i / 12.0;
        p.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
    }
    for (int i = 0; i <= 4; ++i) {
        const double y = plot.top() + plot.height() * i / 4.0;
        p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
    }
    const double pi = 3.14159265358979323846;
    auto cyc = [pi](double u) { return u - std::sin(2.0 * pi * u) / (2.0 * pi); };  // the cycloidal law, 0..1
    QPainterPath curve;
    const int n = 360;
    for (int i = 0; i <= n; ++i) {
        const double deg = 360.0 * i / n;
        double y = deg < 120 ? cyc(deg / 120.0) : deg < 180 ? 1.0 : deg < 300 ? 1.0 - cyc((deg - 180) / 120.0) : 0.0;
        const QPointF pt(plot.left() + plot.width() * deg / 360.0, plot.bottom() - 8 - (plot.height() - 16) * y);
        if (i == 0) curve.moveTo(pt);
        else curve.lineTo(pt);
    }
    QPainterPath fill = curve;
    fill.lineTo(plot.right(), plot.bottom());
    fill.lineTo(plot.left(), plot.bottom());
    fill.closeSubpath();
    p.fillPath(fill, soft);
    p.setPen(QPen(blue, 2.6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPath(curve);
    p.setPen(QPen(rule, 0.8));
    p.drawLine(QPointF(plot.left(), plot.bottom()), QPointF(plot.right(), plot.bottom()));
    p.setFont(font(7.5));
    p.setPen(muted);
    for (int deg : {0, 120, 180, 300, 360}) {
        const double x = plot.left() + plot.width() * deg / 360.0;
        p.drawText(QRectF(x - 20, plot.bottom() + 4, 40, 12), Qt::AlignHCenter | Qt::AlignTop, QStringLiteral("%1\u00b0").arg(deg));
    }
    // The version, the date, and at the foot who it is from.
    const double y0 = h - 120;
    p.setPen(QPen(rule, 0.8));
    p.drawLine(QPointF(44, y0), QPointF(w - 44, y0));
    p.setPen(ink2);
    p.setFont(font(13, QFont::DemiBold));
    p.drawText(QRectF(44, y0 + 14, w - 88, 20), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("Version " ECAM_PRODUCT_VERSION));
    p.setPen(muted);
    p.setFont(font(10));
    p.drawText(QRectF(44, y0 + 36, w - 88, 16), Qt::AlignLeft | Qt::AlignVCenter,
               QLocale(QLocale::English).toString(QDate::currentDate(), QStringLiteral("MMMM yyyy")));
    p.setFont(font(8));
    p.drawText(QRectF(44, h - 24, w - 88, 14), Qt::AlignLeft | Qt::AlignVCenter,
               QStringLiteral("\u00a9 2026 Adam G. Sweeney \u00b7 MIT License"));
    p.drawText(QRectF(44, h - 24, w - 88, 14), Qt::AlignRight | Qt::AlignVCenter, QStringLiteral(ECAM_PRODUCT_NAME));
    p.restore();
}

void DrawPage(QPainter &p, const Pager &pg, int page, int pages) {
    p.save();
    p.translate(0, -page * pg.pageHeight);
    // One device pixel short of the page's end: the next page's first picture starts exactly there.
    pg.doc->drawContents(&p, QRectF(0, page * pg.pageHeight, pg.width, pg.pageHeight - 1));
    p.restore();
    QFont f(QStringLiteral("Segoe UI"));
    f.setPointSizeF(7.5);
    p.setFont(f);
    p.setPen(QColor(0x70, 0x70, 0x70));
    p.drawText(QRectF(0, pg.pageHeight + pg.footer * 0.3, pg.width, pg.footer * 0.7), Qt::AlignHCenter | Qt::AlignVCenter,
               QStringLiteral(ECAM_PRODUCT_NAME " · User guide · %1 / %2").arg(page + 1).arg(pages));
}

// ---- Contents and bookmarks ------------------------------------------------------------------------------------------
// A chapter (level 1) or section (level 2) heading: where it is in the laid-out guide (the page from 0, the top in device
// pixels on that page).
struct TocEntry {
    int level = 1;
    QString title;
    int page = 0;
    double y = 0;
};

QVector<TocEntry> Headings(const Pager &pg) {
    QVector<TocEntry> out;
    QAbstractTextDocumentLayout *layout = pg.doc->documentLayout();
    for (QTextBlock b = pg.doc->begin(); b.isValid(); b = b.next()) {
        const int level = b.blockFormat().headingLevel();
        if (level != 1 && level != 2) continue;
        const QString title = b.text().simplified();
        if (title.isEmpty()) continue;
        const double top = layout->blockBoundingRect(b).top();
        TocEntry e;
        e.level = level;
        e.title = title;
        e.page = static_cast<int>(std::floor((top + 1) / pg.pageHeight));
        e.y = std::max(0.0, top - e.page * pg.pageHeight);
        out << e;
    }
    return out;
}

// The contents laid out on pages `height` points tall: each line's entry and its top, in points. A chapter is never the
// last line of a page.
struct TocLine {
    int entry;
    double y;
};
constexpr double kTocFirstTop = 70, kTocTop = 6, kTocChapter = 17, kTocChapterGap = 7, kTocSection = 13.2, kTocFoot = 14;

QVector<QVector<TocLine>> ContentsPages(const QVector<TocEntry> &entries, double height) {
    QVector<QVector<TocLine>> pages(1);
    double y = kTocFirstTop;
    for (int i = 0; i < entries.size(); ++i) {
        const bool chapter = entries[i].level == 1;
        const double gap = chapter && !pages.last().isEmpty() ? kTocChapterGap : 0;
        const double need = gap + (chapter ? kTocChapter + kTocSection : kTocSection);
        if (y + need > height - kTocFoot) {
            pages.append(QVector<TocLine>());
            y = kTocTop;
        } else {
            y += gap;
        }
        pages.last().append({i, y});
        y += chapter ? kTocChapter : kTocSection;
    }
    return pages;
}

// One contents page in the page's printable area (device pixels at dpi), drawn in points as the cover is: the chapters
// numbered and bold, their sections under them, dot leaders to the page numbers. `links` gets each line's entry and its
// box in points from the printable area's top left.
void DrawContents(QPainter &p, const QSizeF &area, double dpi, const QVector<TocEntry> &entries, const QVector<QVector<TocLine>> &pages,
                  int page, QVector<QPair<int, QRectF>> *links) {
    const QColor accent(0x17, 0x37, 0x5e), blue(0x2a, 0x78, 0xd6), ink(0x1d, 0x21, 0x27), ink2(0x4a, 0x4f, 0x57), muted(0x9a, 0xa1, 0xaa);
    auto font = [dpi](double pt, QFont::Weight w = QFont::Normal) {
        QFont f(QStringLiteral("Segoe UI"));
        f.setPointSizeF(pt * 72.0 / dpi);
        f.setWeight(w);
        f.setHintingPreference(QFont::PreferNoHinting);
        return f;
    };
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    const double s = dpi / 72.0;
    p.scale(s, s);
    const double w = area.width() / s, h = area.height() / s;
    const double left = 6, right = w - 6, numW = 30, textX = left + 26;
    if (page == 0) {
        QFont caps = font(9, QFont::DemiBold);
        caps.setLetterSpacing(QFont::AbsoluteSpacing, 2.2);
        p.setFont(caps);
        p.setPen(blue);
        p.drawText(QRectF(left, 0, w, 14), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("USER GUIDE"));
        p.setFont(font(24, QFont::DemiBold));
        p.setPen(accent);
        p.drawText(QRectF(left, 14, w, 34), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("Contents"));
        p.fillRect(QRectF(left, 52, right - left, 2.5), blue);
    }
    // Chapter numbers: the order of the level-1 entries.
    QHash<int, int> number;
    for (int i = 0, n = 0; i < entries.size(); ++i) {
        if (entries[i].level == 1) number.insert(i, ++n);
    }
    const double dot = 4.0;  // the leaders' pitch: dots line up from line to line
    for (const TocLine &l : pages[page]) {
        const TocEntry &e = entries[l.entry];
        const bool chapter = e.level == 1;
        const double lh = chapter ? kTocChapter : kTocSection;
        const QRectF line(left, l.y, right - left, lh);
        const QFont f = chapter ? font(11, QFont::DemiBold) : font(9.5);
        const QFontMetricsF fm(f, p.device());
        p.setFont(f);
        if (chapter) {
            p.setPen(blue);
            p.drawText(QRectF(left, l.y, textX - left, lh), Qt::AlignLeft | Qt::AlignVCenter, QString::number(number.value(l.entry)));
        }
        const double maxText = right - numW - 14 - textX;
        const QString title = fm.elidedText(e.title, Qt::ElideRight, maxText);
        p.setPen(chapter ? accent : ink);
        p.drawText(QRectF(textX, l.y, maxText + 4, lh), Qt::AlignLeft | Qt::AlignVCenter, title);
        p.setPen(chapter ? accent : ink2);
        p.drawText(QRectF(right - numW, l.y, numW, lh), Qt::AlignRight | Qt::AlignVCenter, QString::number(e.page + 1));
        // The leader: dots on a fixed pitch from the right, between the title and the number.
        const double from = textX + fm.horizontalAdvance(title) + 5, to = right - numW - 4;
        if (to - from > dot) {
            QFont df = font(chapter ? 9.5 : 9.5);
            const QFontMetricsF dm(df, p.device());
            df.setLetterSpacing(QFont::AbsoluteSpacing, dot - dm.horizontalAdvance(QLatin1Char('.')));
            const int n = static_cast<int>(std::floor((to - from) / dot));
            p.setFont(df);
            p.setPen(muted);
            p.drawText(QRectF(to - n * dot, l.y, n * dot + dot, lh), Qt::AlignLeft | Qt::AlignVCenter, QString(n, QLatin1Char('.')));
        }
        if (links) links->append({l.entry, line});
    }
    if (pages.size() > 1) {
        p.setFont(font(7.5));
        p.setPen(muted);
        p.drawText(QRectF(0, h - 12, w, 12), Qt::AlignHCenter | Qt::AlignVCenter,
                   QStringLiteral("Contents %1 / %2").arg(page + 1).arg(pages.size()));
    }
    p.restore();
}

// What the PDF's navigation needs from printing it: the headings, the pages before the guide's own (cover, contents),
// the contents' link boxes (per contents page, in points from the printable area's top left) and the page geometry.
struct PdfNav {
    QVector<TocEntry> entries;
    int frontPages = 0;
    QVector<QVector<QPair<int, QRectF>>> links;
    double dpi = 72, paperHeight = 0;
    QPointF margin;
};

// A PDF text string: UTF-16 with its byte order mark, in hex (nothing to escape).
QByteArray PdfText(const QString &s) {
    QByteArray out("<FEFF");
    for (QChar c : s) out += QByteArray::number(c.unicode(), 16).rightJustified(4, '0').toUpper();
    return out + '>';
}

// An object's text, "N 0 obj" to "endobj" (the last definition: an update redefines it), or empty.
QByteArray PdfObject(const QByteArray &pdf, int num) {
    const QByteArray head = QByteArray::number(num) + " 0 obj";
    qsizetype at = -1;
    for (qsizetype i = pdf.indexOf(head); i >= 0; i = pdf.indexOf(head, i + 1)) {
        if (i == 0 || pdf[i - 1] == '\n' || pdf[i - 1] == '\r') at = i;
    }
    if (at < 0) return {};
    const qsizetype end = pdf.indexOf("endobj", at);
    return end < 0 ? QByteArray() : pdf.mid(at + head.size(), end - at - head.size()).trimmed();
}

int PdfRef(const QByteArray &dict, const char *key) {
    const QRegularExpression re(QStringLiteral("/%1\\s+(\\d+)\\s+0\\s+R").arg(QLatin1String(key)));
    const auto m = re.match(QString::fromLatin1(dict));
    return m.hasMatch() ? m.captured(1).toInt() : -1;
}

// Bookmarks (chapters, their sections under them) and the contents pages' links, added to the PDF Qt wrote as an update
// at its end: new objects, the catalog and the contents pages' link lists redefined, a cross-reference section for them
// pointing back at Qt's. Every target is an explicit place: the page and the heading's top.
bool AddPdfNavigation(const QString &path, const PdfNav &nav, QString *why) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return (*why = QStringLiteral("can't read it")), false;
    const QByteArray pdf = f.readAll();
    f.close();
    const QString text = QString::fromLatin1(pdf);
    const auto sx = QRegularExpression(QStringLiteral(R"re(startxref\s+(\d+)\s+%%EOF\s*$)re")).match(text);
    const qsizetype tr = pdf.lastIndexOf("trailer");
    if (!sx.hasMatch() || tr < 0) return (*why = QStringLiteral("no trailer")), false;
    const QByteArray trailer = pdf.mid(tr, pdf.indexOf(">>", tr) - tr);
    const auto size = QRegularExpression(QStringLiteral(R"re(/Size\s+(\d+))re")).match(QString::fromLatin1(trailer));
    const int root = PdfRef(trailer, "Root"), info = PdfRef(trailer, "Info");
    const auto id = QRegularExpression(QStringLiteral(R"re(/ID\s*\[[^\]]*\])re")).match(QString::fromLatin1(trailer));
    if (!size.hasMatch() || root < 0) return (*why = QStringLiteral("no /Size or /Root")), false;
    const QByteArray catalog = PdfObject(pdf, root);
    const int pagesObj = PdfRef(catalog, "Pages");
    if (!catalog.startsWith("<<") || catalog.contains("/Outlines") || pagesObj < 0) return (*why = QStringLiteral("unexpected catalog")), false;
    // The pages, in order (Qt writes one flat page tree).
    const QByteArray pagesDict = PdfObject(pdf, pagesObj);
    const qsizetype kidsAt = pagesDict.indexOf("/Kids");
    const qsizetype kidsEnd = pagesDict.indexOf(']', kidsAt);
    if (kidsAt < 0 || kidsEnd < 0) return (*why = QStringLiteral("no page list")), false;
    QVector<int> pages;
    static const QRegularExpression ref(QStringLiteral(R"re((\d+)\s+0\s+R)re"));
    for (auto it = ref.globalMatch(QString::fromLatin1(pagesDict.mid(kidsAt, kidsEnd - kidsAt))); it.hasNext();) pages << it.next().captured(1).toInt();
    int last = 0;
    for (const TocEntry &e : nav.entries) last = std::max(last, e.page);
    if (nav.entries.isEmpty() || pages.size() < nav.frontPages + last + 1) return (*why = QStringLiteral("pages don't match")), false;

    int next = size.captured(1).toInt();
    QMap<int, QByteArray> objs;  // number -> body
    auto num = [](double v) { return QByteArray::number(v, 'f', 2); };
    auto dest = [&](const TocEntry &e) {
        const double top = nav.paperHeight - nav.margin.y() - e.y * 72.0 / nav.dpi + 4;
        return "[" + QByteArray::number(pages[nav.frontPages + e.page]) + " 0 R /XYZ 0 " + num(std::min(top, nav.paperHeight)) + " null]";
    };
    // The outline: chapters at the top, each one's sections under it (closed). A section before any chapter (one
    // chapter's PDF) goes at the top.
    const int outlines = next++;
    QVector<int> itemNum(nav.entries.size());
    for (int i = 0; i < nav.entries.size(); ++i) itemNum[i] = next++;
    QVector<int> parent(nav.entries.size(), -1);
    for (int i = 0, chapter = -1; i < nav.entries.size(); ++i) {
        if (nav.entries[i].level == 1) chapter = i;
        else parent[i] = chapter;
    }
    auto siblings = [&](int par) {
        QVector<int> out;
        for (int i = 0; i < nav.entries.size(); ++i) {
            if (parent[i] == par) out << i;
        }
        return out;
    };
    const QVector<int> top = siblings(-1);
    for (int i = 0; i < nav.entries.size(); ++i) {
        const QVector<int> sib = siblings(parent[i]);
        const int k = static_cast<int>(sib.indexOf(i));
        const QVector<int> kids = nav.entries[i].level == 1 ? siblings(i) : QVector<int>();
        QByteArray o = "<< /Title " + PdfText(nav.entries[i].title) + " /Parent " +
                       QByteArray::number(parent[i] < 0 ? outlines : itemNum[parent[i]]) + " 0 R";
        if (k > 0) o += " /Prev " + QByteArray::number(itemNum[sib[k - 1]]) + " 0 R";
        if (k + 1 < sib.size()) o += " /Next " + QByteArray::number(itemNum[sib[k + 1]]) + " 0 R";
        if (!kids.isEmpty()) {
            o += " /First " + QByteArray::number(itemNum[kids.first()]) + " 0 R /Last " + QByteArray::number(itemNum[kids.last()]) +
                 " 0 R /Count -" + QByteArray::number(kids.size());
        }
        o += " /Dest " + dest(nav.entries[i]) + " >>";
        objs.insert(itemNum[i], o);
    }
    objs.insert(outlines, "<< /Type /Outlines /First " + QByteArray::number(itemNum[top.first()]) + " 0 R /Last " +
                              QByteArray::number(itemNum[top.last()]) + " 0 R /Count " + QByteArray::number(top.size()) + " >>");
    // The contents' links: each line a link to its heading, added to its page's link list.
    for (int tp = 0; tp < nav.links.size(); ++tp) {
        const int pageNum = pages[1 + tp];
        const QByteArray page = PdfObject(pdf, pageNum);
        const int annots = PdfRef(page, "Annots");
        if (annots < 0) continue;
        QByteArray list = PdfObject(pdf, annots).trimmed();
        if (!list.startsWith('[') || !list.endsWith(']')) continue;
        list.chop(1);
        for (const auto &l : nav.links[tp]) {
            const QRectF r = l.second;
            const double x0 = nav.margin.x() + r.left(), x1 = nav.margin.x() + r.right();
            const double y1 = nav.paperHeight - nav.margin.y() - r.top(), y0 = nav.paperHeight - nav.margin.y() - r.bottom();
            const int a = next++;
            objs.insert(a, "<< /Type /Annot /Subtype /Link /Rect [" + num(x0) + " " + num(y0) + " " + num(x1) + " " + num(y1) +
                               "] /Border [0 0 0] /Dest " + dest(nav.entries[l.first]) + " >>");
            list += " " + QByteArray::number(a) + " 0 R";
        }
        objs.insert(annots, list + " ]");
    }
    // The catalog: the outline, shown when the file opens.
    objs.insert(root, "<<" + QByteArray(" /Outlines ") + QByteArray::number(outlines) + " 0 R /PageMode /UseOutlines" + catalog.mid(2));

    QByteArray add;
    if (!pdf.endsWith('\n')) add += '\n';
    QMap<int, qsizetype> offsets;
    for (auto it = objs.cbegin(); it != objs.cend(); ++it) {
        offsets.insert(it.key(), pdf.size() + add.size());
        add += QByteArray::number(it.key()) + " 0 obj\n" + it.value() + "\nendobj\n";
    }
    const qsizetype xref = pdf.size() + add.size();
    add += "xref\n";
    for (auto it = offsets.cbegin(); it != offsets.cend();) {
        auto run = it;
        int count = 0;
        for (int n = it.key(); run != offsets.cend() && run.key() == n; ++run, ++n) ++count;
        add += QByteArray::number(it.key()) + " " + QByteArray::number(count) + "\n";
        for (; it != run; ++it) add += QByteArray::number(it.value()).rightJustified(10, '0') + " 00000 n \n";
    }
    add += "trailer\n<< /Size " + QByteArray::number(next) + " /Root " + QByteArray::number(root) + " 0 R";
    if (info >= 0) add += " /Info " + QByteArray::number(info) + " 0 R";
    if (id.hasMatch()) add += " " + id.captured(0).toLatin1();
    add += " /Prev " + sx.captured(1).toLatin1() + " >>\nstartxref\n" + QByteArray::number(xref) + "\n%%EOF\n";
    if (!f.open(QIODevice::Append)) return (*why = QStringLiteral("can't write it")), false;
    const bool ok = f.write(add) == add.size();
    f.close();
    return ok || ((*why = QStringLiteral("write failed")), false);
}

// The bookmarks of a PDF written by AddPdfNavigation: (level, title) in order. For tests.
QVector<QPair<int, QString>> ReadPdfOutline(const QString &path) {
    QVector<QPair<int, QString>> out;
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return out;
    const QByteArray pdf = f.readAll();
    const qsizetype tr = pdf.lastIndexOf("trailer");
    if (tr < 0) return out;
    const int root = PdfRef(pdf.mid(tr, 400), "Root");
    const int outlines = PdfRef(PdfObject(pdf, root), "Outlines");
    if (outlines < 0) return out;
    auto title = [](const QByteArray &o) {
        const qsizetype a = o.indexOf("/Title <FEFF"), b = o.indexOf('>', a);
        QString s;
        if (a < 0 || b < 0) return s;
        const QByteArray hex = o.mid(a + 12, b - a - 12);
        for (qsizetype i = 0; i + 4 <= hex.size(); i += 4) s += QChar(static_cast<char16_t>(hex.mid(i, 4).toUShort(nullptr, 16)));
        return s;
    };
    std::function<void(int, int)> walk = [&](int first, int level) {
        for (int n = first, guard = 0; n >= 0 && guard < 10000; ++guard) {
            const QByteArray o = PdfObject(pdf, n);
            out.append({level, title(o)});
            const int kid = PdfRef(o, "First");
            if (kid >= 0) walk(kid, level + 1);
            n = PdfRef(o, "Next");
        }
    };
    walk(PdfRef(PdfObject(pdf, outlines), "First"), 1);
    return out;
}

// The guide on `printer`: for the whole guide the cover and the contents first (the guide's own pages keep their numbers
// from 1). `nav`, when given, gets what the PDF's bookmarks and the contents' links need.
void PrintGuide(QPrinter *printer, const QString &chapterFile, PdfNav *nav) {
    printer->setPageMargins(kPrintMargins, QPageLayout::Millimeter);
    QPainter p(printer);
    const QSizeF area = printer->pageRect(QPrinter::DevicePixel).size();
    const double dpi = printer->logicalDpiY();
    const Pager pg = LayOut(chapterFile, printer, area);
    const QVector<TocEntry> entries = Headings(pg);
    const bool front = chapterFile.isEmpty();
    const QVector<QVector<TocLine>> toc = front ? ContentsPages(entries, area.height() * 72.0 / dpi) : QVector<QVector<TocLine>>();
    if (nav) {
        nav->entries = entries;
        nav->frontPages = front ? 1 + static_cast<int>(toc.size()) : 0;
        nav->dpi = dpi;
        const QPageLayout layout = printer->pageLayout();
        nav->paperHeight = layout.fullRect(QPageLayout::Point).height();
        nav->margin = layout.paintRect(QPageLayout::Point).topLeft();
        nav->links.clear();
    }
    bool first = true;
    if (front) {
        DrawCover(p, area, printer->logicalDpiX());
        first = false;
        for (int i = 0; i < toc.size(); ++i) {
            printer->newPage();
            QVector<QPair<int, QRectF>> links;
            DrawContents(p, area, dpi, entries, toc, i, &links);
            if (nav) nav->links.append(links);
        }
    }
    const int pages = pg.doc->pageCount();
    for (int i = 0; i < pages; ++i) {
        if (!first) printer->newPage();
        first = false;
        DrawPage(p, pg, i, pages);
    }
}

}  // namespace

const QVector<HelpWindow::Chapter> &HelpWindow::Chapters() {
    static const QVector<Chapter> c = {
        {QStringLiteral("01-start.html"), QStringLiteral("Getting started")},
        {QStringLiteral("02-window.html"), QStringLiteral("The window")},
        {QStringLiteral("03-masters.html"), QStringLiteral("Masters: crank and timeline")},
        {QStringLiteral("04-profile.html"), QStringLiteral("Building a profile")},
        {QStringLiteral("05-laws.html"), QStringLiteral("Motion laws")},
        {QStringLiteral("06-axes.html"), QStringLiteral("Axes and mechanics")},
        {QStringLiteral("07-outputs.html"), QStringLiteral("Outputs")},
        {QStringLiteral("08-cutlength.html"), QStringLiteral("Flying saws: cut-length mode")},
        {QStringLiteral("09-board.html"), QStringLiteral("Running on the ClearCore")},
        {QStringLiteral("15-plc.html"), QStringLiteral("PLC (EtherNet/IP)")},
        {QStringLiteral("10-examples.html"), QStringLiteral("Worked examples")},
        {QStringLiteral("11-files.html"), QStringLiteral("Files and exports")},
        {QStringLiteral("12-scope.html"), QStringLiteral("ClearPath scope")},
        {QStringLiteral("13-keys.html"), QStringLiteral("Keyboard and mouse")},
        {QStringLiteral("14-trouble.html"), QStringLiteral("Troubleshooting and safety")},
    };
    return c;
}

HelpWindow *HelpWindow::Show(QWidget *parent, const QString &page) {
    if (!g_help) g_help = new HelpWindow(parent ? parent->window() : nullptr);
    if (!page.isEmpty()) {
        g_help->open(page);
    } else if (g_help->currentPage().isEmpty()) {
        g_help->open(Chapters().first().file);
    }
    g_help->show();
    g_help->raise();
    g_help->activateWindow();
    return g_help;
}

HelpWindow::HelpWindow(QWidget *parent) : QDialog(parent) {
    setWindowTitle(QStringLiteral(ECAM_PRODUCT_NAME " — User guide"));
    setWindowFlags(Qt::Window | Qt::WindowMinMaxButtonsHint | Qt::WindowCloseButtonHint);
    setStyleSheet(theme::DialogSheet());
    theme::ApplyAccent(this);

    auto *root = new QVBoxLayout(this);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    auto *bar = new QHBoxLayout();
    bar->setSpacing(6);
    auto *back = new QPushButton(QStringLiteral("‹ Back"), this);
    auto *forward = new QPushButton(QStringLiteral("Forward ›"), this);
    bar->addWidget(back);
    bar->addWidget(forward);
    bar->addSpacing(12);
    m_find = new QLineEdit(this);
    m_find->setPlaceholderText(QStringLiteral("Find in this page"));
    m_find->setClearButtonEnabled(true);
    m_find->setMinimumWidth(220);
    bar->addWidget(m_find);
    auto *prev = new QPushButton(QStringLiteral("▲"), this);
    auto *next = new QPushButton(QStringLiteral("▼"), this);
    prev->setToolTip(QStringLiteral("Previous match (Shift+Enter)"));
    next->setToolTip(QStringLiteral("Next match (Enter)"));
    prev->setMinimumWidth(36);
    next->setMinimumWidth(36);
    bar->addWidget(prev);
    bar->addWidget(next);
    bar->addStretch(1);
    auto *printChapter = new QPushButton(QStringLiteral("Print chapter…"), this);
    auto *printAll = new QPushButton(QStringLiteral("Print all…"), this);
    auto *pdf = new QPushButton(QStringLiteral("Save PDF…"), this);
    printAll->setToolTip(QStringLiteral("The whole guide."));
    pdf->setToolTip(QStringLiteral("The whole guide as a PDF."));
    bar->addWidget(printChapter);
    bar->addWidget(printAll);
    bar->addWidget(pdf);
    root->addLayout(bar);

    auto *split = new QSplitter(Qt::Horizontal, this);
    m_toc = new QTreeWidget(split);
    m_toc->setHeaderHidden(true);
    m_toc->setMinimumWidth(220);
    m_toc->setStyleSheet(QStringLiteral(
        "QTreeWidget { background: #1f242b; color: #d8dee6; border: 1px solid #1a1e24; border-radius: 3px; padding: 4px; }"
        "QTreeWidget::item { padding: 3px 2px; }"
        "QTreeWidget::item:selected { background: #2e4a70; color: #f0f4f8; }"));
    m_browser = new HelpBrowser(split);
    m_browser->setOpenExternalLinks(true);
    m_browser->setStyleSheet(QStringLiteral("QTextBrowser { background: #1f242b; border: 1px solid #1a1e24; border-radius: 3px; "
                                            "padding: 14px 22px; }"));
    m_browser->document()->setDefaultStyleSheet(QString::fromUtf8(kScreenCss));
    split->addWidget(m_toc);
    split->addWidget(m_browser);
    split->setStretchFactor(0, 0);
    split->setStretchFactor(1, 1);
    split->setSizes({260, 840});
    root->addWidget(split, 1);

    BuildToc();
    resize(1180, 860);
    restoreGeometry(Settings().value("help/geometry").toByteArray());

    connect(back, &QPushButton::clicked, m_browser, &QTextBrowser::backward);
    connect(forward, &QPushButton::clicked, m_browser, &QTextBrowser::forward);
    connect(m_browser, &QTextBrowser::backwardAvailable, back, &QPushButton::setEnabled);
    connect(m_browser, &QTextBrowser::forwardAvailable, forward, &QPushButton::setEnabled);
    back->setEnabled(false);
    forward->setEnabled(false);
    // The signal carries the new address: source() can still be the old one while it is emitted.
    connect(m_browser, &QTextBrowser::sourceChanged, this, [this](const QUrl &url) { SyncToc(url); });
    connect(m_toc, &QTreeWidget::itemClicked, this, [this](QTreeWidgetItem *it) { open(it->data(0, Qt::UserRole).toString()); });
    connect(m_find, &QLineEdit::returnPressed, this, [this]() { Find(QApplication::keyboardModifiers() & Qt::ShiftModifier); });
    connect(next, &QPushButton::clicked, this, [this]() { Find(false); });
    connect(prev, &QPushButton::clicked, this, [this]() { Find(true); });
    connect(printChapter, &QPushButton::clicked, this, [this]() { Print(false); });
    connect(printAll, &QPushButton::clicked, this, [this]() { Print(true); });
    connect(pdf, &QPushButton::clicked, this, &HelpWindow::ExportPdf);
    connect(this, &QDialog::finished, this, [this]() { Settings().setValue("help/geometry", saveGeometry()); });
}

void HelpWindow::BuildToc() {
    static const QRegularExpression h2(QStringLiteral(R"re(<h2><a name="([^"]+)"></a>(.*?)</h2>)re"));
    static const QRegularExpression tags(QStringLiteral("<[^>]+>"));
    m_toc->clear();
    for (const Chapter &c : Chapters()) {
        auto *item = new QTreeWidgetItem(m_toc, {c.title});
        item->setData(0, Qt::UserRole, c.file);
        QFont f = item->font(0);
        f.setBold(true);
        item->setFont(0, f);
        const QString html = ReadResource(QStringLiteral(":/help/") + c.file);
        for (auto it = h2.globalMatch(html); it.hasNext();) {
            const auto m = it.next();
            QString title = m.captured(2);
            title.remove(tags);
            auto *sub = new QTreeWidgetItem(item, {title.replace(QStringLiteral("&amp;"), QStringLiteral("&"))});
            sub->setData(0, Qt::UserRole, c.file + QLatin1Char('#') + m.captured(1));
        }
    }
}

void HelpWindow::open(const QString &page) { m_browser->setSource(QUrl(QStringLiteral("qrc:/help/") + page)); }

QString HelpWindow::currentPage() const { return m_browser->source().fileName(); }

QString HelpWindow::contentsCurrent() const { return m_toc->currentItem() ? m_toc->currentItem()->text(0) : QString(); }

void HelpWindow::SyncToc(const QUrl &url) {
    const QString file = url.fileName();
    const QString frag = url.fragment();
    const QSignalBlocker block(m_toc);
    for (int i = 0; i < m_toc->topLevelItemCount(); ++i) {
        QTreeWidgetItem *ch = m_toc->topLevelItem(i);
        if (ch->data(0, Qt::UserRole).toString() != file) {
            ch->setExpanded(false);
            continue;
        }
        ch->setExpanded(true);
        QTreeWidgetItem *pick = ch;
        for (int k = 0; k < ch->childCount(); ++k) {
            if (!frag.isEmpty() && ch->child(k)->data(0, Qt::UserRole).toString().endsWith(QLatin1Char('#') + frag)) pick = ch->child(k);
        }
        m_toc->setCurrentItem(pick);
    }
}

void HelpWindow::Find(bool backwards) {
    const QString text = m_find->text();
    if (text.isEmpty()) return;
    const QTextDocument::FindFlags flags = backwards ? QTextDocument::FindBackward : QTextDocument::FindFlags();
    if (m_browser->find(text, flags)) return;
    // Wrap round.
    QTextCursor c = m_browser->textCursor();
    c.movePosition(backwards ? QTextCursor::End : QTextCursor::Start);
    m_browser->setTextCursor(c);
    if (!m_browser->find(text, flags)) m_find->setStyleSheet(QStringLiteral("QLineEdit { border-color: #c0504d; }"));
    else m_find->setStyleSheet(QString());
}

QString HelpWindow::PrintHtml(const QString &chapterFile) {
    QString body;
    if (chapterFile.isEmpty()) {
        // The cover and the contents are drawn pages. Chapters run on (a new page for each would leave many half-empty
        // pages). One document: each chapter's anchors get its prefix ("c09-inputs": several chapters have a "setup"), its
        // title an anchor ("c09"), and a link to another chapter becomes a link inside the document, so it works in the PDF.
        static const QRegularExpression name(QStringLiteral(R"re(<a name="([^"]+)")re"));
        static const QRegularExpression local(QStringLiteral(R"re(href="#([^"]+)")re"));
        static const QRegularExpression other(QStringLiteral(R"re(href="(\d\d)-[a-z0-9_-]+\.html(?:#([^"]+))?")re"));
        for (const Chapter &c : Chapters()) {
            const QString id = QStringLiteral("c") + c.file.left(2);
            QString ch = ReadResource(QStringLiteral(":/help/") + c.file);
            ch.replace(name, QStringLiteral("<a name=\"%1-\\1\"").arg(id));
            ch.replace(local, QStringLiteral("href=\"#%1-\\1\"").arg(id));
            QString linked;
            qsizetype last = 0;
            for (auto it = other.globalMatch(ch); it.hasNext();) {
                const auto m = it.next();
                linked += ch.mid(last, m.capturedStart() - last);
                const QString to = QStringLiteral("c") + m.captured(1);
                linked += m.captured(2).isEmpty() ? QStringLiteral("href=\"#%1\"").arg(to) : QStringLiteral("href=\"#%1-%2\"").arg(to, m.captured(2));
                last = m.capturedEnd();
            }
            ch = linked + ch.mid(last);
            const qsizetype h1 = ch.indexOf(QStringLiteral("<h1>"));
            if (h1 >= 0) ch.insert(h1 + 4, QStringLiteral("<a name=\"%1\"></a>").arg(id));
            body += ch;
        }
    } else {
        body = ReadResource(QStringLiteral(":/help/") + QUrl(chapterFile).path());
    }
    // Pictures sized for paper: 72 % of their screen size, at most about 5.4 inches across. They are kept at
    // twice their screen size, so they stay sharp on paper; smaller, they fit the page and leave fewer gaps.
    static const QRegularExpression img(QStringLiteral(R"re(<img([^>]*?)width="(\d+)")re"));
    QString out;
    qsizetype last = 0;
    for (auto it = img.globalMatch(body); it.hasNext();) {
        const auto m = it.next();
        out += body.mid(last, m.capturedStart() - last);
        out += QStringLiteral("<img%1width=\"%2\"").arg(m.captured(1)).arg(std::min(static_cast<int>(std::lround(m.captured(2).toInt() * 0.72)), 520));
        last = m.capturedEnd();
    }
    out += body.mid(last);
    return out;
}

QTextDocument *HelpWindow::PrintDocument(const QString &chapterFile, QObject *parent, const QSet<int> &pageBreaks) {
    auto *doc = new QTextDocument(parent);
    doc->setDefaultStyleSheet(QString::fromUtf8(kPrintCss));
    QString html = PrintHtml(chapterFile);
    if (!pageBreaks.isEmpty()) {
        // Headings counted in document order, as the layout sees them.
        static const QRegularExpression heading(QStringLiteral(R"re(<h[1-6](?=[\s>]))re"));
        QString out;
        qsizetype last = 0;
        int ordinal = -1;
        for (auto it = heading.globalMatch(html); it.hasNext();) {
            const auto m = it.next();
            ++ordinal;
            out += html.mid(last, m.capturedEnd() - last);
            if (pageBreaks.contains(ordinal)) out += QStringLiteral(" style=\"page-break-before: always;\"");
            last = m.capturedEnd();
        }
        html = out + html.mid(last);
    }
    // Every picture handed to the document itself, so printing never depends on resolving paths.
    static const QRegularExpression src(QStringLiteral(R"re(src="(images/[^"]+)")re"));
    for (auto it = src.globalMatch(html); it.hasNext();) {
        const QString name = it.next().captured(1);
        if (doc->resource(QTextDocument::ImageResource, QUrl(name)).isNull()) {
            doc->addResource(QTextDocument::ImageResource, QUrl(name), QImage(QStringLiteral(":/help/") + name));
        }
    }
    doc->setHtml(html);
    return doc;
}

void HelpWindow::PrintTo(QPrinter *printer, const QString &chapterFile) { PrintGuide(printer, chapterFile, nullptr); }

namespace {

// A Letter page at `dpi` with the print margins, for laying out and previewing without a printer.
QImage PaperImage(double dpi, QRect *paint) {
    const QPageLayout layout(QPageSize(QPageSize::Letter), QPageLayout::Portrait, kPrintMargins, QPageLayout::Millimeter);
    *paint = layout.paintRectPixels(static_cast<int>(dpi));
    QImage img(layout.fullRectPixels(static_cast<int>(dpi)).size(), QImage::Format_RGB32);
    img.setDotsPerMeterX(static_cast<int>(std::lround(dpi / 0.0254)));
    img.setDotsPerMeterY(static_cast<int>(std::lround(dpi / 0.0254)));
    img.fill(Qt::white);
    return img;
}

}  // namespace

int HelpWindow::StrandedHeadings(const QString &chapterFile) {
    QRect paint;
    QImage img = PaperImage(110, &paint);
    const Pager pg = LayOut(chapterFile, &img, paint.size());
    return static_cast<int>(Stranded(pg.doc.get(), pg.pageHeight).size());
}

QImage HelpWindow::PreviewCover(double dpi) {
    QRect paint;
    QImage img = PaperImage(dpi, &paint);
    QPainter p(&img);
    p.translate(paint.topLeft());
    DrawCover(p, paint.size(), dpi);
    return img;
}

QImage HelpWindow::PreviewContents(int page, double dpi, int *pages) {
    QRect paint;
    QImage img = PaperImage(dpi, &paint);
    const Pager pg = LayOut(QString(), &img, paint.size());
    const QVector<TocEntry> entries = Headings(pg);
    const auto toc = ContentsPages(entries, paint.height() * 72.0 / dpi);
    if (pages) *pages = static_cast<int>(toc.size());
    if (page >= 0 && page < toc.size()) {
        QPainter p(&img);
        p.translate(paint.topLeft());
        DrawContents(p, paint.size(), dpi, entries, toc, page, nullptr);
    }
    return img;
}

QVector<QPair<int, QString>> HelpWindow::PdfOutlineForTest(const QString &path) { return ReadPdfOutline(path); }

QImage HelpWindow::PreviewPage(int page, double dpi, int *pages, const QString &chapterFile) {
    QRect paint;
    QImage img = PaperImage(dpi, &paint);
    const Pager pg = LayOut(chapterFile, &img, paint.size());
    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::SmoothPixmapTransform);
    p.translate(paint.topLeft());
    const int n = pg.doc->pageCount();
    if (pages) *pages = n;
    if (page >= 0 && page < n) DrawPage(p, pg, page, n);
    return img;
}

bool HelpWindow::SavePdf(const QString &path, QString *error, const QString &chapterFile) {
    QPrinter printer(QPrinter::HighResolution);
    printer.setOutputFormat(QPrinter::PdfFormat);
    printer.setOutputFileName(path);
    // US Letter, as the layout is checked on, whatever the PC's default paper (a printer keeps the paper picked for it).
    printer.setPageSize(QPageSize(QPageSize::Letter));
    printer.setPageOrientation(QPageLayout::Portrait);
    printer.setDocName(QStringLiteral(ECAM_PRODUCT_NAME " user guide"));
    PdfNav nav;
    PrintGuide(&printer, chapterFile, &nav);  // its painter ends with it: the file is complete
    const QFileInfo fi(path);
    if (!fi.exists() || fi.size() < 1024) {
        if (error) *error = QStringLiteral("the PDF was not written");
        return false;
    }
    // The bookmarks and the contents' links: Qt has no way to write them, so they are added as an update at the file's
    // end. Should that fail, the PDF is still whole and readable, only without them.
    QString why;
    if (!AddPdfNavigation(path, nav, &why)) qWarning("guide PDF: no bookmarks: %s", qPrintable(why));
    return true;
}

void HelpWindow::Print(bool all) {
    QPrinter printer(QPrinter::HighResolution);
    QPrintDialog dlg(&printer, this);
    dlg.setWindowTitle(all ? QStringLiteral("Print the user guide") : QStringLiteral("Print this chapter"));
    if (dlg.exec() != QDialog::Accepted) return;
    QApplication::setOverrideCursor(Qt::WaitCursor);  // laying out the whole guide takes a few seconds
    PrintTo(&printer, all ? QString() : currentPage());
    QApplication::restoreOverrideCursor();
}

void HelpWindow::ExportPdf() {
    const QString start = QDir(Settings().value("files/lastDir").toString()).filePath(QStringLiteral(ECAM_PRODUCT_NAME " user guide.pdf"));
    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("Save the user guide as PDF"), start, QStringLiteral("PDF (*.pdf)"));
    if (path.isEmpty()) return;
    if (QFileInfo(path).suffix().isEmpty()) path += QStringLiteral(".pdf");
    QString err;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    const bool saved = SavePdf(path, &err);
    QApplication::restoreOverrideCursor();
    if (!saved) QMessageBox::warning(this, QStringLiteral("Save PDF"), QStringLiteral("Could not save %1: %2").arg(path, err));
}
