/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "TimingView.h"

#include "Theme.h"

#include <QMouseEvent>
#include <QContextMenuEvent>
#include <QMenu>
#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cmath>

using namespace ecam;

namespace {

constexpr double kLabelW = 118.0;
constexpr double kRight = 14.0;
constexpr double kHeader = 22.0;  // above the plot: the time and repeat readout, clear of everything in the plot
constexpr double kTop = kHeader + 4.0;
constexpr double kBottom = 24.0;

double NiceStep(double span, int ticks) {
    if (span <= 0.0) return 1.0;
    const double raw = span / ticks;
    const double mag = std::pow(10.0, std::floor(std::log10(raw)));
    const double n = raw / mag;
    return (n < 1.5 ? 1.0 : n < 3.5 ? 2.0 : n < 7.5 ? 5.0 : 10.0) * mag;
}

}  // namespace

TimingView::TimingView(QWidget *parent) : QWidget(parent) {
    setCursor(Qt::CrossCursor);
    setMouseTracking(true);  // the resize cursor over a wait's line
}

int TimingView::WaitAt(double x) const {
    int best = -1;
    double gap = 5.0;
    for (size_t k = 0; k < m_project.waits.size(); ++k) {
        const double d = std::fabs(XFromT(m_project.waits[k].t) - x);
        if (d <= gap) {
            gap = d;
            best = static_cast<int>(k);
        }
    }
    return best;
}

void TimingView::setProject(const CamProject &project, int currentAxis) {
    m_project = project;
    m_current = currentAxis;
    m_rows.clear();
    for (int i = 0; i < static_cast<int>(project.axes.size()); ++i) {
        if (project.axes[i].master == AxisMaster::Timeline) m_rows.push_back(i);
    }
    m_length = std::max(TimelineLength(project), 1e-6);
    update();
}

void TimingView::setTime(double t) {
    m_t = t;
    update();
}

QRectF TimingView::Plot() const {
    return QRectF(kLabelW, kTop, std::max(10.0, width() - kLabelW - kRight), std::max(10.0, height() - kTop - kBottom));
}

double TimingView::XFromT(double t) const { return Plot().left() + t / m_length * Plot().width(); }
double TimingView::TFromX(double x) const {
    return std::clamp((x - Plot().left()) / Plot().width() * m_length, 0.0, m_length);
}

int TimingView::RowAt(double y) const {
    if (m_rows.isEmpty()) return -1;
    const QRectF r = Plot();
    const int i = static_cast<int>((y - r.top()) / (r.height() / m_rows.size()));
    return (i >= 0 && i < m_rows.size()) ? i : -1;
}

void TimingView::paintEvent(QPaintEvent *) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), theme::kCanvas);
    const QRectF r = Plot();
    if (m_rows.isEmpty()) {
        p.setPen(theme::kAxisText);
        p.drawText(rect(), Qt::AlignCenter, QStringLiteral("No axes on the timeline."));
        return;
    }

    // Time grid
    const double step = NiceStep(m_length, std::max(2, static_cast<int>(r.width() / 90)));
    p.setFont(QFont(font().family(), 8));
    for (int i = 0; i * step <= m_length + 1e-9; ++i) {
        const double x = XFromT(i * step);
        p.setPen(QPen(i % 2 ? theme::kGrid : theme::kGridMajor, 1));
        p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()));
        p.setPen(theme::kAxisText);
        p.drawText(QRectF(x - 30, r.bottom() + 3, 60, 16), Qt::AlignHCenter | Qt::AlignTop,
                   QStringLiteral("%1 s").arg(QString::number(i * step, 'g', 6)));
    }

    const double rowH = r.height() / m_rows.size();
    for (int row = 0; row < m_rows.size(); ++row) {
        const int ai = m_rows[row];
        const CamAxis &a = m_project.axes[ai];
        const QRectF band(r.left(), r.top() + row * rowH, r.width(), rowH);
        const bool cur = ai == m_current;
        if (cur) p.fillRect(band, theme::kSelectBand);
        // Past this axis's end: resting
        if (a.span < m_length) {
            const double x0 = XFromT(a.span);
            p.fillRect(QRectF(x0, band.top(), band.right() - x0, band.height()), QColor(0x17, 0x1b, 0x20, 200));
        }
        p.setPen(QPen(theme::kGridMajor, 1));
        p.drawLine(band.bottomLeft(), band.bottomRight());

        const AxisStats st = Analyze(a, 720);
        double lo = st.minPos, hi = st.maxPos;
        if (hi - lo < 1e-12) {
            lo -= 1;
            hi += 1;
        }
        const double pad = rowH * 0.18;
        auto y = [&](double pos) { return band.bottom() - pad - (pos - lo) / (hi - lo) * (band.height() - 2 * pad); };

        // Segment boundaries
        p.setPen(QPen(QColor(0xb8, 0xc4, 0xd6, 40), 1));
        for (int k = 1; k < KnotCount(a); ++k) {
            const double x = XFromT(KnotDeg(a, k));
            p.drawLine(QPointF(x, band.top() + 2), QPointF(x, band.bottom() - 2));
        }

        QPainterPath path;
        const int cols = std::max(2, static_cast<int>(r.width()));
        for (int i = 0; i <= cols; ++i) {
            const double t = m_length * i / cols;
            const CamPoint c = (a.cyclic && t >= a.span)
                                   ? EvaluateSegment(a, static_cast<int>(a.segments.size()) - 1, 1.0)
                                   : Evaluate(a, t);
            const QPointF pt(XFromT(t), y(c.pos));
            if (i == 0) {
                path.moveTo(pt);
            } else {
                path.lineTo(pt);
            }
        }
        p.setPen(QPen(theme::SegmentColor(ai), cur ? 2.2 : 1.6));
        p.setBrush(Qt::NoBrush);
        p.drawPath(path);

        // Row label
        const QString u = QString::fromStdString(a.units);
        p.setPen(cur ? theme::kText : theme::kAxisText);
        p.setFont(QFont(font().family(), 9, QFont::DemiBold));
        p.drawText(QRectF(8, band.top() + 4, kLabelW - 14, 18), Qt::AlignLeft | Qt::AlignVCenter,
                   p.fontMetrics().elidedText(QString::fromStdString(a.name), Qt::ElideRight, int(kLabelW - 14)));
        p.setFont(QFont(font().family(), 8));
        p.setPen(theme::kAxisText);
        p.drawText(QRectF(8, band.top() + 22, kLabelW - 14, 16), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("M-%1 · %2 s").arg(a.connector).arg(QString::number(a.span, 'g', 4)));
        p.drawText(QRectF(8, band.top() + 38, kLabelW - 14, 16), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("%1 … %2 %3").arg(QString::number(st.minPos, 'g', 4), QString::number(st.maxPos, 'g', 4), u));
        // Value at the cursor
        const CamPoint now = Evaluate(a, std::min(m_t, a.span));
        p.setPen(theme::kText);
        p.setBrush(theme::SegmentColor(ai));
        p.drawEllipse(QPointF(XFromT(std::min(m_t, m_length)), y(now.pos)), 3.5, 3.5);
    }

    p.setPen(QPen(theme::kGridMajor, 1));
    p.setBrush(Qt::NoBrush);
    p.drawRect(r);
    // The waits: a dashed line across the rows at each one's time, its name at the top; the one the board holds at, lit.
    p.setFont(QFont(font().family(), 8));
    QVector<QRectF> placed;  // the wait labels so far
    for (size_t k = 0; k < m_project.waits.size(); ++k) {
        const TimelineWait &w = m_project.waits[k];
        const bool dragged = static_cast<int>(k) == m_dragWait;
        const double t = dragged ? m_dragT : w.t;
        if (t > m_length) continue;
        const bool lit = static_cast<int>(k) + 1 == m_activeWait || dragged;
        // Dragged where a timeline axis moves: red (it will snap to the nearest rest).
        const bool bad = dragged && !TimelineAtRest(m_project, t);
        const double x = XFromT(t);
        const QColor line = bad ? QColor(0xff, 0x6b, 0x6b) : lit ? QColor(0xff, 0xcc, 0x40) : QColor(0xe0, 0xa8, 0x40, 170);
        p.setPen(QPen(line, lit ? 2.0 : 1.2, dragged ? Qt::SolidLine : Qt::DashLine));
        p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()));
        QString label = w.name.empty() ? QStringLiteral("Wait %1").arg(k + 1) : QString::fromStdString(w.name);
        if (dragged) label += QStringLiteral("  %1 s").arg(QString::number(t, 'f', 3));
        // To the right of its line, or the left near the plot's end; on the first line free of the labels before it.
        const double lw = p.fontMetrics().horizontalAdvance(label) + 2.0;
        const bool left = x + 3.0 + lw > r.right() - 2.0;
        QRectF box(left ? x - 3.0 - lw : x + 3.0, r.top() + 2.0, lw, 14.0);
        for (int line = 0; line < 3; ++line) {
            box.moveTop(r.top() + 2.0 + line * 14.0);
            bool clear = true;
            for (const QRectF &o : placed) clear = clear && !o.adjusted(-4, 0, 4, 0).intersects(box);
            if (clear) break;
        }
        placed << box;
        p.setPen(bad ? QColor(0xff, 0x9a, 0x9a) : lit ? QColor(0xff, 0xdd, 0x80) : QColor(0xe0, 0xb8, 0x70));
        p.drawText(box, (left ? Qt::AlignRight : Qt::AlignLeft) | Qt::AlignVCenter, label);
    }
    const double cx = XFromT(std::clamp(m_t, 0.0, m_length));
    p.setPen(QPen(theme::kCursor, 1));
    p.drawLine(QPointF(cx, r.top()), QPointF(cx, r.bottom()));

    QString head = QStringLiteral("t  %1 s").arg(QString::number(m_t, 'f', 3));
    head += m_project.master.timelineRepeat ? QStringLiteral("   repeat %1 s").arg(QString::number(m_length, 'g', 4))
                                            : QStringLiteral("   one-shot");
    // In the header above the plot, right-aligned with it: nothing inside the plot can run under it.
    p.setFont(QFont(QStringLiteral("Consolas"), 9));
    const QFontMetrics fm = p.fontMetrics();
    const double bh = std::min<double>(fm.height() + 6, kHeader - 2.0);
    const QRectF box(r.right() - fm.horizontalAdvance(head) - 14, (kHeader - bh) / 2.0 + 1.0, fm.horizontalAdvance(head) + 14, bh);
    p.setPen(QPen(QColor(0x3a, 0x41, 0x4c), 1));
    p.setBrush(QColor(20, 23, 28, 210));
    p.drawRoundedRect(box, 3, 3);
    p.setPen(theme::kText);
    p.drawText(box, Qt::AlignCenter, head);
}

void TimingView::mousePressEvent(QMouseEvent *e) {
    if (e->button() != Qt::LeftButton) return;
    // On a wait's line: drag the wait, not the cursor.
    const int wait = m_waitsEditable && Plot().contains(e->position()) ? WaitAt(e->position().x()) : -1;
    if (wait >= 0) {
        m_dragWait = wait;
        m_dragT = m_project.waits[wait].t;
        update();
        return;
    }
    const int row = RowAt(e->position().y());
    if (row >= 0 && m_rows[row] != m_current) emit axisClicked(m_rows[row]);
    if (Plot().contains(e->position())) {
        m_dragging = true;
        emit timeChanged(TFromX(e->position().x()));
    }
}

void TimingView::mouseMoveEvent(QMouseEvent *e) {
    if (m_dragWait >= 0) {
        m_dragT = TFromX(e->position().x());
        update();
        return;
    }
    if (m_dragging) {
        emit timeChanged(TFromX(e->position().x()));
        return;
    }
    const bool over = m_waitsEditable && Plot().contains(e->position()) && WaitAt(e->position().x()) >= 0;
    setCursor(over ? Qt::SizeHorCursor : Qt::CrossCursor);
}

void TimingView::mouseReleaseEvent(QMouseEvent *) {
    m_dragging = false;
    if (m_dragWait >= 0) {
        const int k = m_dragWait;
        const double t = m_dragT;
        m_dragWait = -1;
        update();
        if (std::fabs(t - m_project.waits[k].t) > 1e-9) emit waitMoved(k, t);
    }
}

void TimingView::contextMenuEvent(QContextMenuEvent *e) {
    if (m_rows.isEmpty() || !Plot().contains(e->pos())) return;
    const double t = TFromX(e->pos().x());
    QMenu menu(this);
    QAction *wait = menu.addAction(QStringLiteral("Wait for an input here\u2026"));
    if (menu.exec(e->globalPos()) == wait) emit waitRequested(t);
}

void TimingView::setActiveWait(int index) {
    if (index == m_activeWait) return;
    m_activeWait = index;
    update();
}
