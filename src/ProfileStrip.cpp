/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "ProfileStrip.h"

#include "Theme.h"

#include <QContextMenuEvent>
#include <QMenu>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QPainter>
#include <QWheelEvent>
#include <QPainterPath>

#include <algorithm>
#include <cmath>

using namespace ecam;

namespace {

constexpr double kLeft = 70.0;
constexpr double kRight = 14.0;
constexpr double kTop = 10.0;
constexpr double kBottom = 24.0;
constexpr double kGap = 14.0;
constexpr double kKnotHit = 9.0;
constexpr double kOutHead = 15.0;  // "Outputs" title row of the band
constexpr double kLaneH = 14.0;     // lane height when there is room; down to kLaneMin in a short diagram
constexpr double kLaneMin = 8.0;
constexpr double kEdgePx = 5.0;    // grab distance for a pulse's ends

const QColor kLit(0x7f, 0xe3, 0xa0);
const QColor kLampOff(0x3a, 0x41, 0x4c);

double NiceStep(double span, int targetTicks) {
    if (span <= 0.0) return 1.0;
    const double raw = span / targetTicks;
    const double mag = std::pow(10.0, std::floor(std::log10(raw)));
    const double n = raw / mag;
    const double nice = n < 1.5 ? 1.0 : n < 3.5 ? 2.0 : n < 7.5 ? 5.0 : 10.0;
    return nice * mag;
}

QString Num(double v, int sig = 4) {
    if (std::fabs(v) < 1e-12) return QStringLiteral("0");
    return QString::number(v, 'g', sig);
}

}  // namespace

ProfileStrip::ProfileStrip(QWidget *parent) : QWidget(parent) {
    setMouseTracking(true);
    setAutoFillBackground(false);
    setFocusPolicy(Qt::ClickFocus);
    m_axis = DefaultAxis("Axis 1");
    m_stats = Analyze(m_axis);
}

void ProfileStrip::setAxis(const CamAxis &axis, const CamMaster &master, double viewSpan) {
    m_axis = axis;
    m_master = master;
    const bool wasTime = m_time;
    const double oldView = m_view;
    m_time = axis.master == AxisMaster::Timeline;
    m_view = viewSpan > 0.0 ? viewSpan : axis.span;
    // A different master or length: start fitted; the same (another axis, an edit): keep the zoom.
    if (wasTime != m_time || std::fabs(oldView - m_view) > 1e-9 * std::max(1.0, m_view)) {
        const bool zoomed = m_zoom != 1.0;
        m_zoom = 1.0;
        m_x0 = 0.0;
        if (zoomed) emit zoomChanged(m_zoom);
    }
    ClampPan();
    m_stats = Analyze(m_axis, 1440);
    BuildMarkers();
    if (m_hoverKnot >= KnotCount(m_axis)) m_hoverKnot = -1;
    if (!m_rangeFrozen) FitRange();
    update();
}

void ProfileStrip::setSelectedSegment(int seg) {
    if (seg == m_selected) return;
    m_selected = seg;
    update();
}

void ProfileStrip::setCursorDeg(double deg) {
    m_cursorDeg = deg;
    update();
}

void ProfileStrip::setCurvesVisible(bool vel, bool acc, bool jerk) {
    m_showVel = vel;
    m_showAcc = acc;
    m_showJerk = jerk;
    Layout();
    update();
}

void ProfileStrip::setOutputs(const std::vector<OutputPulse> &pulses) {
    m_outputs = pulses;
    if (m_grab == Grab::None) RebuildLanes();
    update();
}

void ProfileStrip::RebuildLanes() {
    std::vector<int> lanes;
    for (int n = 0; n < kOutputs; ++n) {
        for (const OutputPulse &o : m_outputs) {
            if (o.output == n) {
                lanes.push_back(n);
                break;
            }
        }
    }
    if (lanes != m_lanes) {
        m_lanes = lanes;
        Layout();
    }
}

void ProfileStrip::setLiveOutputs(unsigned mask, bool valid) {
    if (mask == m_liveMask && valid == m_liveValid) return;
    m_liveMask = mask;
    m_liveValid = valid;
    update();
}

void ProfileStrip::setSyncWindow(double startDeg, double endDeg) {
    m_syncStart = startDeg;
    m_syncEnd = endDeg;
    update();
}

void ProfileStrip::setOutputsVisible(bool on) {
    m_showOutputs = on;
    Layout();
    update();
}

void ProfileStrip::setSnap(double deg, double pos) {
    m_snapDeg = deg;
    m_snapPos = pos;
}

void ProfileStrip::resizeEvent(QResizeEvent *) { Layout(); }

void ProfileStrip::Layout() {
    QRectF all(kLeft, kTop, std::max(10.0, width() - kLeft - kRight), std::max(10.0, height() - kTop - kBottom));
    // The Outputs band takes the bottom; the plots share the rest.
    // Only the outputs the project uses have a lane; with none, no band.
    if (m_showOutputs && !m_lanes.empty()) {
        const int lanes = static_cast<int>(m_lanes.size());
        // The band takes at most about a third of the height, so the curves keep most of it.
        m_laneH = std::clamp((all.height() * 0.32 - kOutHead) / lanes, kLaneMin, kLaneH);
        const double bandH = kOutHead + lanes * m_laneH;
        m_outBand = QRectF(all.left(), all.bottom() - bandH, all.width(), bandH);
        all.setBottom(std::max(all.top() + 20.0, m_outBand.top() - kGap));
    } else {
        m_outBand = QRectF();
    }
    if (m_showVel || m_showAcc || m_showJerk) {
        const double dh = std::floor((all.height() - kGap) * 0.62);
        m_disp = QRectF(all.left(), all.top(), all.width(), dh);
        m_deriv = QRectF(all.left(), m_disp.bottom() + kGap, all.width(), all.bottom() - m_disp.bottom() - kGap);
    } else {
        m_disp = all;
        m_deriv = QRectF();
    }
}

void ProfileStrip::FitRange() {
    double lo = m_stats.minPos, hi = m_stats.maxPos;
    if (hi - lo < 1e-9) {
        lo -= 1.0;
        hi += 1.0;
    }
    const double pad = (hi - lo) * 0.12;
    m_yMin = lo - pad;
    m_yMax = hi + pad;
}

double ProfileStrip::XFromDeg(double deg) const { return m_disp.left() + (deg - m_x0) / visibleSpan() * m_disp.width(); }
double ProfileStrip::DegFromX(double x) const {
    const double v = m_x0 + (x - m_disp.left()) / m_disp.width() * visibleSpan();
    return std::clamp(v, std::max(0.0, m_x0), std::min(m_view, m_x0 + visibleSpan()));
}

/* ---- Zoom and pan --------------------------------------------------------------------------- */

namespace {
constexpr double kMaxZoom = 200.0;
}

void ProfileStrip::ClampPan() {
    m_zoom = std::clamp(m_zoom, 1.0, kMaxZoom);
    m_x0 = std::clamp(m_x0, 0.0, std::max(0.0, m_view - visibleSpan()));
}

void ProfileStrip::ZoomAbout(double factor, double anchorDeg) {
    const double before = m_zoom;
    const double frac = (anchorDeg - m_x0) / visibleSpan();  // where the anchor sits across the plot
    m_zoom = std::clamp(m_zoom * factor, 1.0, kMaxZoom);
    m_x0 = anchorDeg - frac * visibleSpan();
    ClampPan();
    if (m_zoom != before) emit zoomChanged(m_zoom);
    update();
}

void ProfileStrip::zoomBy(double factor) {
    const bool cursorShown = m_cursorDeg >= m_x0 && m_cursorDeg <= m_x0 + visibleSpan();
    ZoomAbout(factor, cursorShown ? m_cursorDeg : m_x0 + visibleSpan() / 2.0);
}

void ProfileStrip::resetZoom() {
    if (m_zoom == 1.0 && m_x0 == 0.0) return;
    m_zoom = 1.0;
    m_x0 = 0.0;
    emit zoomChanged(m_zoom);
    update();
}

void ProfileStrip::panBy(double masterUnits) {
    const double before = m_x0;
    m_x0 += masterUnits;
    ClampPan();
    if (m_x0 != before) {
        emit zoomChanged(m_zoom);
        update();
    }
}

void ProfileStrip::wheelEvent(QWheelEvent *e) {
    const double steps = e->angleDelta().y() != 0 ? e->angleDelta().y() / 120.0 : e->angleDelta().x() / 120.0;
    if (e->modifiers() & Qt::ControlModifier) {
        const double x = e->position().x();
        const double anchor = (x >= m_disp.left() && x <= m_disp.right())
                                  ? m_x0 + (x - m_disp.left()) / m_disp.width() * visibleSpan()
                                  : m_x0 + visibleSpan() / 2.0;
        ZoomAbout(std::pow(1.25, steps), anchor);
        e->accept();
        return;
    }
    if ((e->modifiers() & Qt::ShiftModifier) || e->angleDelta().x() != 0) {
        if (m_zoom > 1.0) {
            panBy(-steps * visibleSpan() * 0.1);
            e->accept();
            return;
        }
    }
    QWidget::wheelEvent(e);
}

void ProfileStrip::keyPressEvent(QKeyEvent *e) {
    switch (e->key()) {
        case Qt::Key_Plus:
        case Qt::Key_Equal: zoomBy(1.25); return;
        case Qt::Key_Minus:
        case Qt::Key_Underscore: zoomBy(1.0 / 1.25); return;
        case Qt::Key_0: resetZoom(); return;
        default: break;
    }
    QWidget::keyPressEvent(e);
}

QString ProfileStrip::XText(double x, int decimals) const {
    if (!m_time && m_master.lineMode) {  // cut-length mode: line distance in the cycle as drawn
        return QStringLiteral("%1 mm").arg(QString::number(x * m_master.designLength / 360.0, 'f', decimals));
    }
    return m_time ? QStringLiteral("%1 s").arg(QString::number(x, 'f', decimals + 1))
                  : QStringLiteral("%1°").arg(QString::number(x, 'f', decimals));
}
double ProfileStrip::YFromPos(double pos) const {
    return m_disp.bottom() - (pos - m_yMin) / (m_yMax - m_yMin) * m_disp.height();
}
double ProfileStrip::PosFromY(double y) const {
    return m_yMin + (m_disp.bottom() - y) / m_disp.height() * (m_yMax - m_yMin);
}

QPointF ProfileStrip::KnotPoint(int k) const { return {XFromDeg(KnotDeg(m_axis, k)), YFromPos(KnotPos(m_axis, k))}; }

int ProfileStrip::HitKnot(const QPointF &p) const {
    int best = -1;
    double bestD = kKnotHit;
    for (int k = 0; k < KnotCount(m_axis); ++k) {
        const double d = std::hypot(p.x() - KnotPoint(k).x(), p.y() - KnotPoint(k).y());
        if (d <= bestD) {
            bestD = d;
            best = k;
        }
    }
    return best;
}

bool ProfileStrip::InPlot(const QPointF &p) const {
    return m_disp.adjusted(-4, -4, 4, 4).contains(p) || (!m_deriv.isNull() && m_deriv.contains(p)) ||
           (!m_outBand.isNull() && m_outBand.contains(p));
}

QRectF ProfileStrip::Lane(int output) const {
    const auto it = std::find(m_lanes.begin(), m_lanes.end(), output);
    if (it == m_lanes.end() || m_outBand.isNull()) return QRectF();
    const int row = static_cast<int>(it - m_lanes.begin());
    return QRectF(m_outBand.left(), m_outBand.top() + kOutHead + row * m_laneH, m_outBand.width(), m_laneH);
}

QRectF ProfileStrip::PulseRect(const OutputPulse &p) const {
    const QRectF lane = Lane(p.output);
    if (lane.isNull()) return QRectF();
    const double x0 = XFromDeg(p.on), x1 = std::max(XFromDeg(p.off), x0 + 2.0);
    const double inset = lane.height() >= 12.0 ? 2.0 : 1.0;
    return QRectF(x0, lane.top() + inset, x1 - x0, lane.height() - 2 * inset);
}

int ProfileStrip::LaneAt(double y) const {
    if (m_outBand.isNull()) return -1;
    const double top = m_outBand.top() + kOutHead;
    if (y < top || y >= m_outBand.bottom()) return -1;
    if (m_lanes.empty()) return -1;
    return m_lanes[std::clamp(static_cast<int>((y - top) / m_laneH), 0, static_cast<int>(m_lanes.size()) - 1)];
}

int ProfileStrip::PulseAt(QPointF pos, Grab *grab) const {
    *grab = Grab::None;
    const int lane = LaneAt(pos.y());
    if (lane < 0) return -1;
    for (int i = static_cast<int>(m_outputs.size()) - 1; i >= 0; --i) {
        const OutputPulse &p = m_outputs[i];
        if (p.output != lane || p.master != ViewMaster()) continue;
        const QRectF r = PulseRect(p);
        if (std::fabs(pos.x() - r.left()) <= kEdgePx) {
            *grab = Grab::On;
            return i;
        }
        if (std::fabs(pos.x() - r.right()) <= kEdgePx) {
            *grab = Grab::Off;
            return i;
        }
        if (pos.x() > r.left() && pos.x() < r.right()) {
            *grab = Grab::Move;
            return i;
        }
    }
    return -1;
}

double ProfileStrip::SnapDeg(double deg, Qt::KeyboardModifiers mods) const {
    if (m_snapDeg <= 0.0 || (mods & Qt::AltModifier)) return deg;
    return std::round(deg / m_snapDeg) * m_snapDeg;
}

double ProfileStrip::SnapPos(double pos, Qt::KeyboardModifiers mods) const {
    if (m_snapPos <= 0.0 || (mods & Qt::AltModifier)) return pos;
    return std::round(pos / m_snapPos) * m_snapPos;
}

CamPoint ProfileStrip::PointAt(double deg) const {
    if (m_axis.cyclic && deg >= m_axis.span) {
        return EvaluateSegment(m_axis, static_cast<int>(m_axis.segments.size()) - 1, 1.0);
    }
    return Evaluate(m_axis, deg);  // a timeline axis rests at its end past its span
}

/* ---- Problem markers ------------------------------------------------------------------------ */

void ProfileStrip::BuildMarkers() {
    m_markers.clear();
    m_hoverMarker = -1;
    const QString u = QString::fromStdString(m_axis.units);
    const double w = MasterRate(m_master, m_axis);
    const auto num = [](double v) { return QString::number(v, 'g', 4); };
    const auto at = [this](double x) {
        return m_time ? QStringLiteral("%1 s").arg(x, 0, 'f', 3) : QStringLiteral("%1°").arg(x, 0, 'f', 1);
    };
    const auto add = [this](double deg, double pos, bool error, const QString &text) {
        m_markers.push_back(Marker{deg, pos, error, text});
    };

    for (const CamIssue &is : m_stats.issues) {
        switch (is.kind) {
            case IssueKind::VelocityStep:
            case IssueKind::AccelStep: {
                const bool vel = is.kind == IssueKind::VelocityStep;
                const QString text =
                    vel ? QStringLiteral("<b>Velocity step</b> at %1: %2 → %3 %4/s.<br>Infinite acceleration, a hard "
                                         "knock. Use a law that ends at rest, or a Blend next to a Linear segment.")
                              .arg(at(is.deg), num(is.left * w), num(is.right * w), u)
                        : QStringLiteral("<b>Acceleration step</b> at %1: %2 → %3 %4/s².<br>Infinite jerk. Use "
                                         "Cycloidal, a polynomial or a modified law instead of Harmonic or Linear.")
                              .arg(at(is.deg), num(is.left * w * w), num(is.right * w * w), u);
                // Knot 0 of a repeating axis is also the end of its cycle: mark both ends.
                add(is.deg, KnotPos(m_axis, is.knot), vel, text);
                if (is.knot == 0 && m_axis.cyclic) add(m_axis.span, KnotPos(m_axis, KnotCount(m_axis) - 1), vel, text);
                break;
            }
            case IssueKind::SplineOvershoot:
                add(is.deg, is.left, false,
                    QStringLiteral("<b>Spline overshoot</b> at %1: the curve reaches %2 %3, past the knots' %4 %3.<br>"
                                   "Add a knot there, or move the knots next to it.")
                        .arg(at(is.deg), num(is.left), u, num(is.right)));
                break;
            case IssueKind::PressureAngle:
                add(is.deg, PointAt(is.deg).pos, false,
                    QStringLiteral("<b>Pressure angle</b> %1° at %2, above %3° (cam drawing).<br>A larger base "
                                   "radius or a longer segment lowers it.")
                        .arg(QString::number(is.left, 'f', 1), at(is.deg),
                             QString::number(kMaxPressureAngleDeg, 'f', 0)));
                break;
            case IssueKind::Undercut:
                add(is.deg, PointAt(is.deg).pos, false,
                    QStringLiteral("<b>Undercut</b> near %1: pitch radius %2 is smaller than the roller (cam drawing).<br>"
                                   "Use a smaller roller, a larger base radius or a gentler law.")
                        .arg(at(is.deg), num(is.left)));
                break;
        }
    }

    // The axis's limits at the master speed, where the profile passes them most.
    if (w > 0.0 && m_axis.velMax > 0.0 && m_stats.maxVel * w > m_axis.velMax * (1 + 1e-9)) {
        add(m_stats.maxVelDeg, PointAt(m_stats.maxVelDeg).pos, true,
            QStringLiteral("<b>Velocity over the limit</b> at %1: %2 %3/s, the limit is %4 %3/s.<br>Lower the speed, "
                           "lengthen the segment, or pick a law with a lower peak velocity (Modified sine).")
                .arg(at(m_stats.maxVelDeg), num(m_stats.maxVel * w), u, num(m_axis.velMax)));
    }
    if (w > 0.0 && m_axis.accMax > 0.0) {
        const double alpha = AxisRamp(m_master, m_axis);
        double worst = 0.0, worstDeg = 0.0;
        for (int i = 0; i < 1440; ++i) {
            const double x = m_axis.span * i / 1440;
            const CamPoint pt = Evaluate(m_axis, x);
            const double a = std::fabs(pt.acc) * w * w + std::fabs(pt.vel) * alpha;
            if (a > worst) {
                worst = a;
                worstDeg = x;
            }
        }
        if (worst > m_axis.accMax * (1 + 1e-9)) {
            add(worstDeg, PointAt(worstDeg).pos, true,
                QStringLiteral("<b>Acceleration over the limit</b> at %1: %2 %3/s²%4, the limit is %5 %3/s².<br>"
                               "Lower the speed, lengthen the segment, or pick a law with a lower peak acceleration "
                               "(Modified trapezoid).")
                    .arg(at(worstDeg), num(worst), u,
                         alpha > 0.0 ? QStringLiteral(" with the crank ramp") : QString(), num(m_axis.accMax)));
        }
    }
}

std::vector<QRectF> ProfileStrip::MarkerRects() const {
    // A sign 16 px wide sits 12 px over the point it marks (under it near the top of the plot). Signs at the same
    // place stack upward, so each stays readable and hoverable.
    std::vector<QRectF> rects(m_markers.size());
    const QRectF &r = m_disp;
    for (size_t i = 0; i < m_markers.size(); ++i) {
        const Marker &m = m_markers[i];
        const double x = XFromDeg(m.deg);
        if (x < r.left() - 1.0 || x > r.right() + 1.0) continue;
        const double y = YFromPos(m.pos);
        int stack = 0;
        for (size_t j = 0; j < i; ++j) {
            if (!rects[j].isNull() && std::fabs(rects[j].center().x() - x) < 14.0) ++stack;
        }
        double top = y - 12.0 - 15.0 - stack * 17.0;
        if (top < r.top() + 2.0) top = y + 12.0 + stack * 17.0;  // no room above: hang it under the point
        rects[i] = QRectF(x - 8.0, top, 16.0, 15.0);
    }
    return rects;
}

int ProfileStrip::HitMarker(const QPointF &p) const {
    const std::vector<QRectF> rects = MarkerRects();
    for (size_t i = 0; i < rects.size(); ++i) {
        if (!rects[i].isNull() && rects[i].adjusted(-3, -3, 3, 3).contains(p)) return static_cast<int>(i);
    }
    return -1;
}

void ProfileStrip::DrawMarkers(QPainter &p) const {
    if (m_markers.empty()) return;
    const std::vector<QRectF> rects = MarkerRects();
    const double bottom = m_deriv.isNull() ? m_disp.bottom() : m_deriv.bottom();
    p.save();
    p.setClipRect(QRectF(m_disp.left(), m_disp.top(), m_disp.width(), bottom - m_disp.top()));
    // Dashed lines first, through the displacement and the derivative plots, so the place reads in both.
    for (size_t i = 0; i < m_markers.size(); ++i) {
        if (rects[i].isNull()) continue;
        const QColor c = m_markers[i].error ? theme::kWarn : theme::kAccel;
        const double x = XFromDeg(m_markers[i].deg);
        QPen pen(QColor(c.red(), c.green(), c.blue(), 170), 1.2, Qt::DashLine);
        p.setPen(pen);
        p.drawLine(QPointF(x, m_disp.top()), QPointF(x, bottom));
    }
    p.setClipping(false);
    QFont f(font().family(), 9, QFont::Bold);
    p.setFont(f);
    for (size_t i = 0; i < m_markers.size(); ++i) {
        if (rects[i].isNull()) continue;
        const QRectF &s = rects[i];
        const QColor c = m_markers[i].error ? theme::kWarn : theme::kAccel;
        const bool hot = static_cast<int>(i) == m_hoverMarker;
        QPolygonF tri;
        tri << QPointF(s.center().x(), s.top()) << QPointF(s.right(), s.bottom()) << QPointF(s.left(), s.bottom());
        p.setPen(QPen(hot ? theme::kText : QColor(0x1b, 0x1e, 0x23), hot ? 1.6 : 1.2));
        p.setBrush(c);
        p.drawPolygon(tri);
        p.setPen(QColor(0x1b, 0x1e, 0x23));
        p.drawText(s.adjusted(0, 3, 0, 1), Qt::AlignCenter, QStringLiteral("!"));
    }
    p.restore();
}

/* ---- Painting ------------------------------------------------------------------------------- */

void ProfileStrip::paintEvent(QPaintEvent *) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), QColor(0x25, 0x29, 0x2f));
    DrawDisplacement(p);
    if (!m_deriv.isNull()) DrawDerivatives(p);
    if (!m_outBand.isNull()) DrawOutputs(p);
    DrawMarkers(p);
    DrawCursor(p);
    const QString readout = readoutHtml();
    if (readout != m_lastReadout) {
        m_lastReadout = readout;
        emit readoutChanged(readout);
    }
}

void ProfileStrip::DrawBands(QPainter &p, const QRectF &r) const {
    p.fillRect(r, theme::kCanvas);
    p.save();
    p.setClipRect(r);
    for (int s = 0; s < static_cast<int>(m_axis.segments.size()); ++s) {
        const double x0 = XFromDeg(SegmentStartDeg(m_axis, s));
        const double x1 = XFromDeg(m_axis.segments[s].endDeg);
        const QRectF band(x0, r.top(), x1 - x0, r.height());
        if (s % 2) p.fillRect(band, theme::kCanvasAlt);
        if (s == m_selected) p.fillRect(band, theme::kSelectBand);
    }
    // Timeline: past this axis's end it holds position while longer axes keep going.
    if (m_axis.span < m_view - 1e-12) {
        const double x0 = XFromDeg(m_axis.span);
        const QRectF hold(x0, r.top(), r.right() - x0, r.height());
        p.fillRect(hold, QColor(0x17, 0x1b, 0x20));
        p.setPen(QPen(QColor(0x2a, 0x30, 0x38), 1));
        for (double x = hold.left() - hold.height(); x < hold.right(); x += 10) {
            p.drawLine(QPointF(std::max(x, hold.left()), hold.bottom() - std::max(0.0, hold.left() - x)),
                       QPointF(std::min(x + hold.height(), hold.right()),
                               hold.top() + std::max(0.0, x + hold.height() - hold.right())));
        }
    }
    p.restore();
}

void ProfileStrip::DrawAngleGrid(QPainter &p, const QRectF &r, bool labels) const {
    const double vis = visibleSpan(), end = std::min(m_view, m_x0 + vis);
    if (m_time) {
        const double step = NiceStep(vis, std::max(2, static_cast<int>(r.width() / 90)));
        p.setFont(QFont(font().family(), 8));
        for (int i = static_cast<int>(std::ceil(m_x0 / step - 1e-9)); i * step <= end + 1e-9; ++i) {
            const double v = i * step;
            const double x = XFromDeg(v);
            p.setPen(QPen(i % 2 ? theme::kGrid : theme::kGridMajor, 1));
            p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()));
            if (labels) {
                p.setPen(theme::kAxisText);
                p.drawText(QRectF(x - 30, r.bottom() + 3, 60, 16), Qt::AlignHCenter | Qt::AlignTop,
                           QStringLiteral("%1 s").arg(Num(v, 6)));
            }
        }
        return;
    }
    // Labels at least ~60 px apart, two or three grid lines per label.
    const double pxPerDeg = r.width() / vis;
    static const double kLabelSteps[] = {0.1, 0.2, 0.5, 1, 2, 5, 10, 15, 30, 45, 90};
    double label = 90;
    int per = 2;
    if (m_master.lineMode) {
        // Cut-length mode labels line distance: round millimetres, not round degrees.
        const double mmPerDeg = m_master.designLength / 360.0;
        label = NiceStep(vis * mmPerDeg, std::max(2, static_cast<int>(r.width() / 90))) / mmPerDeg;
    } else {
        for (double st : kLabelSteps) {
            if (st * pxPerDeg >= 60) {
                label = st;
                break;
            }
        }
        if (label == 15 || label == 30 || label == 45 || label == 90) per = 3;
    }
    const double minor = label / per * pxPerDeg >= 14 ? label / per : label;
    const int ratio = static_cast<int>(std::lround(label / minor));
    p.setFont(QFont(font().family(), 8));
    for (long i = static_cast<long>(std::ceil(m_x0 / minor - 1e-9)); i * minor <= end + 1e-9; ++i) {
        const double d = i * minor;
        const double x = XFromDeg(d);
        const bool labelled = i % ratio == 0;
        const bool major = std::fabs(std::remainder(d, 90.0)) < 1e-9 || (m_zoom > 1.0 && labelled);
        p.setPen(QPen(major ? theme::kGridMajor : theme::kGrid, 1));
        p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()));
        if (labels && labelled) {
            p.setPen(theme::kAxisText);
            // Cut-length mode: line distance (mm) in the cycle as drawn.
            const QString text = m_master.lineMode ? QStringLiteral("%1").arg(Num(d * m_master.designLength / 360.0, 5))
                                                   : QStringLiteral("%1°").arg(Num(d, 6));
            p.drawText(QRectF(x - 30, r.bottom() + 3, 60, 16), Qt::AlignHCenter | Qt::AlignTop, text);
        }
    }
}

void ProfileStrip::DrawDisplacement(QPainter &p) const {
    const QRectF &r = m_disp;
    DrawBands(p, r);
    DrawAngleGrid(p, r, m_deriv.isNull() && m_outBand.isNull());

    // Position grid
    const double step = NiceStep(m_yMax - m_yMin, std::max(2, static_cast<int>(r.height() / 40)));
    p.setFont(QFont(font().family(), 8));
    for (double v = std::ceil(m_yMin / step) * step; v <= m_yMax + 1e-9; v += step) {
        const double y = YFromPos(v);
        p.setPen(QPen(std::fabs(v) < step * 1e-6 ? theme::kGridMajor : theme::kGrid, 1));
        p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
        p.setPen(theme::kAxisText);
        p.drawText(QRectF(0, y - 8, kLeft - 8, 16), Qt::AlignRight | Qt::AlignVCenter, Num(v, 6));
    }
    p.save();
    p.translate(12, r.center().y());
    p.rotate(-90);
    p.setPen(theme::kAxisText);
    p.drawText(QRectF(-r.height() / 2, -8, r.height(), 16), Qt::AlignCenter,
               QStringLiteral("Position (%1)").arg(QString::fromStdString(m_axis.units)));
    p.restore();

    // Law names along the top of each band
    p.setFont(QFont(font().family(), 8, QFont::DemiBold));
    for (int s = 0; s < static_cast<int>(m_axis.segments.size()); ++s) {
        const double x0 = std::max(XFromDeg(SegmentStartDeg(m_axis, s)), r.left());
        const double x1 = std::min(XFromDeg(m_axis.segments[s].endDeg), r.right());
        const QRectF tr(x0 + 10, r.top() + 2, x1 - x0 - 20, 14);
        if (tr.width() < 24) continue;
        p.setPen(s == m_selected ? theme::kText : theme::kAxisText);
        p.drawText(tr, Qt::AlignLeft | Qt::AlignTop,
                   p.fontMetrics().elidedText(QString::fromLatin1(LawName(m_axis.segments[s].law)), Qt::ElideRight,
                                              static_cast<int>(tr.width())));
    }

    // Curve, sampled per pixel plus every knot so corners stay sharp.
    p.save();
    p.setClipRect(r.adjusted(-1, -1, 1, 1));
    QPainterPath path;
    const int cols = std::max(2, static_cast<int>(r.width()));
    const double visD = visibleSpan();
    int k = 1;
    while (k < KnotCount(m_axis) && KnotDeg(m_axis, k) <= m_x0) ++k;
    for (int i = 0; i <= cols; ++i) {
        const double deg = std::min(m_view, m_x0 + visD * i / cols);
        while (k < KnotCount(m_axis) && KnotDeg(m_axis, k) < deg) {
            const double kd = KnotDeg(m_axis, k);
            // Left-hand limit at the knot.
            path.lineTo(XFromDeg(kd), YFromPos(EvaluateSegment(m_axis, k - 1, 1.0).pos));
            ++k;
        }
        const QPointF pt(XFromDeg(deg), YFromPos(PointAt(deg).pos));
        if (i == 0) {
            path.moveTo(pt);
        } else {
            path.lineTo(pt);
        }
    }
    p.setPen(QPen(theme::kDisplacement, 2.0));
    p.setBrush(Qt::NoBrush);
    p.drawPath(path);
    p.restore();

    // Knots: clipped to the plot plus a knot's width. The problems the analysis finds are drawn over everything
    // (DrawMarkers).
    p.save();
    p.setClipRect(QRectF(r.left() - 7, r.top() - 8, r.width() + 14, r.height() + 16));
    for (int kk = 0; kk < KnotCount(m_axis); ++kk) {
        const QPointF kp = KnotPoint(kk);
        const bool hot = kk == m_hoverKnot || kk == m_dragKnot;
        p.setPen(QPen(hot ? theme::kText : theme::kDisplacement, 2.0));
        p.setBrush(hot ? theme::kSelect : theme::kCanvas);
        p.drawEllipse(kp, hot ? 6.5 : 5.0, hot ? 6.5 : 5.0);
    }
    p.restore();

    p.setPen(QPen(theme::kGridMajor, 1));
    p.setBrush(Qt::NoBrush);
    p.drawRect(r);
}

void ProfileStrip::DrawDerivatives(QPainter &p) const {
    const QRectF &r = m_deriv;
    DrawBands(p, r);
    DrawAngleGrid(p, r, m_outBand.isNull());
    const double mid = r.center().y();
    p.setPen(QPen(theme::kGridMajor, 1));
    p.drawLine(QPointF(r.left(), mid), QPointF(r.right(), mid));

    const double w = MasterRate(m_master, m_axis);
    const QString u = QString::fromStdString(m_axis.units);
    struct Curve {
        bool on;
        QColor color;
        double peak;  // per-degree
        double (*get)(const CamPoint &);
        QString label;
    };
    const Curve curves[] = {
        {m_showVel, theme::kVelocity, m_stats.maxVel, [](const CamPoint &c) { return c.vel; },
         QStringLiteral("V %1 %2/s").arg(Num(m_stats.maxVel * w), u)},
        {m_showAcc, theme::kAccel, m_stats.maxAcc, [](const CamPoint &c) { return c.acc; },
         QStringLiteral("A %1 %2/s²").arg(Num(m_stats.maxAcc * w * w), u)},
        {m_showJerk, theme::kJerk, m_stats.maxJerk, [](const CamPoint &c) { return c.jerk; },
         QStringLiteral("J %1 %2/s³").arg(Num(m_stats.maxJerk * w * w * w), u)},
    };

    const int cols = std::max(2, static_cast<int>(r.width()));
    p.save();
    p.setClipRect(r);
    double legendX = r.left() + 8;
    p.setFont(QFont(font().family(), 8, QFont::DemiBold));
    for (const Curve &c : curves) {
        if (!c.on) continue;
        const double scale = c.peak > 0 ? (r.height() * 0.45) / c.peak : 0.0;
        QPainterPath path;
        for (int i = 0; i <= cols; ++i) {
            const double deg = std::min(m_view, m_x0 + visibleSpan() * i / cols);
            const QPointF pt(XFromDeg(deg), mid - c.get(PointAt(deg)) * scale);
            if (i == 0) {
                path.moveTo(pt);
            } else {
                path.lineTo(pt);
            }
        }
        p.setPen(QPen(c.color, 1.6));
        p.drawPath(path);
        const QString text = QStringLiteral("%1 peak").arg(c.label);
        const double tw = p.fontMetrics().horizontalAdvance(text) + 12;
        p.fillRect(QRectF(legendX - 4, r.top() + 3, tw, 16), QColor(20, 23, 28, 200));
        p.setPen(c.color);
        p.drawText(QRectF(legendX, r.top() + 3, tw, 16), Qt::AlignLeft | Qt::AlignVCenter, text);
        legendX += tw + 6;
    }
    p.restore();

    p.save();
    p.translate(12, r.center().y());
    p.rotate(-90);
    p.setPen(theme::kAxisText);
    p.setFont(QFont(font().family(), 8));
    p.drawText(QRectF(-r.height() / 2, -8, r.height(), 16), Qt::AlignCenter, QStringLiteral("Normalized"));
    p.restore();
    p.setPen(QPen(theme::kGridMajor, 1));
    p.drawRect(r);
}

void ProfileStrip::DrawOutputs(QPainter &p) const {
    const QRectF &band = m_outBand;
    p.fillRect(band, theme::kCanvas);
    DrawAngleGrid(p, band, true);
    p.setFont(QFont(font().family(), 8, QFont::DemiBold));
    p.setPen(theme::kText);
    p.drawText(QRectF(4, band.top(), kLeft - 8, kOutHead), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("Outputs"));
    const AxisMaster view = ViewMaster();
    p.setFont(QFont(font().family(), 8));
    p.setPen(theme::kAxisText);
    QString head = m_time ? QStringLiteral("timeline pulses (s)") : QStringLiteral("crank pulses (degrees, every revolution)");
    if (m_locked) head += QStringLiteral("  ·  locked");
    p.drawText(QRectF(band.left() + 6, band.top(), band.width() - 12, kOutHead), Qt::AlignLeft | Qt::AlignVCenter, head);
    for (const int n : m_lanes) {
        const QRectF lane = Lane(n);
        p.setPen(QPen(theme::kGrid, 1));
        p.drawLine(lane.bottomLeft(), lane.bottomRight());
        // Label with two lamps: this master's pulses at the cursor, and the board (when connected).
        bool atCursor = false;
        int other = 0;
        for (const OutputPulse &o : m_outputs) {
            if (o.output != n) continue;
            if (o.master != view) {
                ++other;
            } else if (m_cursorDeg >= o.on && m_cursorDeg < o.off) {
                atCursor = true;
            }
        }
        const QColor color = theme::OutputColor(n);
        p.setPen(color);
        QFont lf(font().family());
        lf.setPixelSize(static_cast<int>(std::clamp(m_laneH, 8.0, 11.0)));  // fits the lane
        lf.setBold(atCursor);
        p.setFont(lf);
        p.drawText(QRectF(4, lane.top(), 34, lane.height()), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("IO-%1").arg(n));
        p.setFont(QFont(font().family(), 8));
        p.setPen(Qt::NoPen);
        p.setBrush(atCursor ? color : kLampOff);
        const double lamp = std::min(3.5, m_laneH / 2.0 - 1.0);
        p.drawEllipse(QPointF(44, lane.center().y()), lamp, lamp);
        if (m_liveValid) {
            p.setBrush(((m_liveMask >> n) & 1u) ? kLit : kLampOff);
            p.drawRect(QRectF(52, lane.center().y() - lamp, 2 * lamp, 2 * lamp));
        }
        if (other > 0) {
            p.setPen(QColor(0x8a, 0x92, 0x9d, 150));
            const QString kind = m_time ? QStringLiteral("crank") : QStringLiteral("timeline");
            p.drawText(lane.adjusted(0, 0, -6, 0), Qt::AlignRight | Qt::AlignVCenter,
                       QStringLiteral("+%1 %2 pulse%3").arg(other).arg(kind, other > 1 ? QStringLiteral("s") : QString()));
        }
    }
    p.save();
    p.setClipRect(band);
    for (const OutputPulse &o : m_outputs) {
        if (o.master != view) continue;
        QColor fill = theme::OutputColor(o.output);
        fill.setAlpha(110);
        p.setPen(QPen(theme::OutputColor(o.output), 1.4));
        p.setBrush(fill);
        p.drawRoundedRect(PulseRect(o), 2, 2);
    }
    p.restore();
    p.setPen(QPen(theme::kGridMajor, 1));
    p.setBrush(Qt::NoBrush);
    p.drawRect(band);
}

void ProfileStrip::DrawCursor(QPainter &p) const {
    const double bottom = !m_outBand.isNull() ? m_outBand.bottom() : m_deriv.isNull() ? m_disp.bottom() : m_deriv.bottom();
    p.save();
    p.setClipRect(QRectF(m_disp.left() - 5, m_disp.top() - 5, m_disp.width() + 10, bottom - m_disp.top() + 10));
    // Cut-length mode: the sync, where the blade is on the cut line and moving with it.
    if (!m_time && m_master.lineMode && m_syncStart >= 0.0 && m_syncEnd > m_syncStart) {
        const double x0 = XFromDeg(m_syncStart), x1 = XFromDeg(m_syncEnd);
        p.fillRect(QRectF(x0, m_disp.top(), x1 - x0, bottom - m_disp.top()), QColor(0x7f, 0xe3, 0xa0, 28));
        p.setPen(QPen(QColor(0x7f, 0xe3, 0xa0, 170), 1.0, Qt::DashLine));
        p.drawLine(QPointF(x0, m_disp.top()), QPointF(x0, bottom));
        p.drawLine(QPointF(x1, m_disp.top()), QPointF(x1, bottom));
        p.setFont(QFont(font().family(), 8, QFont::DemiBold));
        p.setPen(QColor(0x7f, 0xe3, 0xa0));
        p.drawText(QRectF(x0 + 3, m_disp.top() + 16, x1 - x0 - 4, 14), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("sync / cut"));
    }
    auto line = [&](double deg, const QColor &c, Qt::PenStyle style) {
        const double x = XFromDeg(deg);
        p.setPen(QPen(c, 1, style));
        p.drawLine(QPointF(x, m_disp.top()), QPointF(x, bottom));
    };
    if (m_hoverDeg >= 0.0 && m_dragKnot < 0) line(m_hoverDeg, QColor(0xf0, 0xf3, 0xf7, 70), Qt::DashLine);
    line(m_cursorDeg, theme::kCursor, Qt::SolidLine);

    // The cursor's point on the curve (the readout itself is outside the plot: readoutHtml).
    if (m_dragKnot < 0) {
        const CamPoint c = PointAt(m_cursorDeg);
        p.setPen(QPen(theme::kText, 1.5));
        p.setBrush(theme::kDisplacement);
        p.drawEllipse(QPointF(XFromDeg(m_cursorDeg), YFromPos(c.pos)), 3.5, 3.5);
    }
    p.restore();
}

QString ProfileStrip::readoutHtml() const {
    if (m_axis.segments.empty()) return {};
    const double w = MasterRate(m_master, m_axis);
    const QString u = QString::fromStdString(m_axis.units).toHtmlEscaped();
    double deg = m_cursorDeg;
    CamPoint c = PointAt(deg);
    const bool knot = m_dragKnot >= 0;
    if (knot) {
        deg = KnotDeg(m_axis, m_dragKnot);
        c.pos = KnotPos(m_axis, m_dragKnot);
    }
    // Key in muted grey, value in its curve's colour, unit muted.
    auto field = [](const QString &key, const QString &value, const QString &unit, const QColor &color) {
        const QString gap = unit == QStringLiteral("°") ? QString() : QStringLiteral("&nbsp;");
        return QStringLiteral("<span style='color:#7d8693;'>%1</span>&nbsp;<span style='color:%2;'>%3</span>"
                              "<span style='color:#7d8693;'>%4%5</span>")
            .arg(key, color.name(), value.toHtmlEscaped(), gap, unit);
    };
    const QString sep = QStringLiteral("&nbsp;&nbsp;&nbsp;");
    QString x = XText(deg);
    QString xUnit;
    if (x.endsWith(QStringLiteral("°"))) {
        x.chop(1);
        xUnit = QStringLiteral("°");
    } else if (const int sp = x.lastIndexOf(QLatin1Char(' ')); sp > 0) {  // "1.25 s", "120.00 mm"
        xUnit = x.mid(sp + 1).toHtmlEscaped();
        x = x.left(sp);
    }
    QString html = field(m_time ? QStringLiteral("t") : m_master.lineMode ? QStringLiteral("x") : QStringLiteral("θ"), x, xUnit,
                         theme::kText) +
                   sep + field(QStringLiteral("s"), Num(c.pos, 6), u, theme::kDisplacement);
    if (knot) {
        html += sep + QStringLiteral("<span style='color:#7d8693;'>knot %1</span>").arg(m_dragKnot);
    } else {
        html += sep + field(QStringLiteral("v"), Num(c.vel * w), u + QStringLiteral("/s"), theme::kVelocity);
        html += sep + field(QStringLiteral("a"), Num(c.acc * w * w), u + QStringLiteral("/s²"), theme::kAccel);
    }
    return html;
}

/* ---- Mouse ---------------------------------------------------------------------------------- */

void ProfileStrip::setLocked(bool locked) {
    if (m_locked == locked) return;
    m_locked = locked;
    m_hoverKnot = -1;
    unsetCursor();
    setToolTip(QString());
    update();
}

void ProfileStrip::mousePressEvent(QMouseEvent *e) {
    if (e->button() == Qt::MiddleButton) {
        if (m_zoom > 1.0) {
            m_panning = true;
            m_panStartX = e->position().x();
            m_panStartX0 = m_x0;
            setCursor(Qt::ClosedHandCursor);
        }
        return;
    }
    if (e->button() != Qt::LeftButton) return;
    const QPointF pos = e->position();
    if (m_locked) {
        Grab grab;
        if (PulseAt(pos, &grab) >= 0 || HitKnot(pos) >= 0) emit lockedEditAttempted();
    } else {
        Grab grab;
        const int pulse = PulseAt(pos, &grab);
        if (pulse >= 0) {
            m_grab = grab;
            m_pulse = pulse;
            m_grabX = DegFromX(pos.x());
            m_pulseStart = m_outputs[pulse];
            emit pulseDragStarted();
            return;
        }
    }
    const int k = m_locked ? -1 : HitKnot(pos);
    if (k >= 0) {
        m_dragKnot = k;
        m_dragOrigin = pos;
        m_dragDeg0 = KnotDeg(m_axis, k);
        m_dragPos0 = KnotPos(m_axis, k);
        m_rangeFrozen = true;
        // Segment to the knot's left (the one it ends), or segment 0 for knot 0.
        emit segmentClicked(std::max(0, k - 1));
        emit dragStarted();
        update();
        return;
    }
    if (InPlot(pos)) {
        const double deg = DegFromX(pos.x());
        m_scrubbing = true;
        emit segmentClicked(SegmentAt(m_axis, std::min(deg, m_axis.span * 0.999999)));
        emit cursorMoved(deg);
    }
}

void ProfileStrip::mouseMoveEvent(QMouseEvent *e) {
    const QPointF pos = e->position();
    if (m_panning) {
        const double before = m_x0;
        m_x0 = m_panStartX0 - (pos.x() - m_panStartX) / m_disp.width() * visibleSpan();
        ClampPan();
        if (m_x0 != before) {
            emit zoomChanged(m_zoom);
            update();
        }
        return;
    }
    if (m_grab != Grab::None && m_pulse >= 0 && m_pulse < static_cast<int>(m_outputs.size())) {
        const Qt::KeyboardModifiers mods = e->modifiers();
        const double minW = m_time ? kMinPulseSeconds : kMinPulseDeg;
        const double maxX = m_time ? m_view : 360.0;
        OutputPulse q = m_pulseStart;
        const double dx = DegFromX(pos.x()) - m_grabX;
        const double width = m_pulseStart.off - m_pulseStart.on;
        if (m_grab == Grab::Move) {
            q.on = std::clamp(SnapDeg(m_pulseStart.on + dx, mods), 0.0, std::max(0.0, maxX - width));
            q.off = q.on + width;
            const int lane = LaneAt(pos.y());
            if (lane >= 0) q.output = lane;  // onto another output
        } else if (m_grab == Grab::On) {
            q.on = std::clamp(SnapDeg(m_pulseStart.on + dx, mods), 0.0, q.off - minW);
        } else {
            q.off = std::clamp(SnapDeg(m_pulseStart.off + dx, mods), q.on + minW, maxX);
        }
        std::vector<OutputPulse> pulses = m_outputs;
        pulses[m_pulse] = q;
        emit pulsesDragged(pulses);
        return;
    }
    if (m_dragKnot >= 0) {
        const Qt::KeyboardModifiers mods = e->modifiers();
        double deg = SnapDeg(DegFromX(pos.x()), mods);
        double val = SnapPos(PosFromY(pos.y()), mods);
        if (mods & Qt::ShiftModifier) deg = m_dragDeg0;  // vertical only
        if (mods & Qt::ControlModifier) val = m_dragPos0;  // horizontal only
        emit knotDragged(m_dragKnot, deg, val);
        return;
    }
    if (m_scrubbing) {
        emit cursorMoved(DegFromX(pos.x()));
        return;
    }
    {
        // A problem marker (a knot under it still drags when unlocked): say what it is and what to do.
        const int mk = (m_locked || HitKnot(pos) < 0) ? HitMarker(pos) : -1;
        const bool was = m_hoverMarker >= 0;
        if (mk != m_hoverMarker) {
            m_hoverMarker = mk;
            update();
        }
        if (mk >= 0) {
            setCursor(Qt::WhatsThisCursor);
            setToolTip(m_markers[mk].text);
            m_hoverKnot = -2;  // re-evaluate the knot cursor when leaving the marker
            m_hoverDeg = DegFromX(pos.x());
            return;
        }
        if (was) {
            setCursor(Qt::ArrowCursor);
            setToolTip(QString());
        }
    }
    if (m_locked) {
        m_hoverDeg = InPlot(pos) ? DegFromX(pos.x()) : -1.0;
        update();
        return;
    }
    const int k = HitKnot(pos);
    {
        Grab grab;
        if (k < 0 && PulseAt(pos, &grab) >= 0) {
            setCursor(grab == Grab::Move ? Qt::SizeAllCursor : Qt::SizeHorCursor);
            setToolTip(QStringLiteral("Drag to move (also onto another output), drag an end to change it.  "
                                      "Right-click to delete.  Alt: no snap."));
            m_hoverKnot = -2;  // re-evaluate the knot cursor when leaving the pulse
            m_hoverDeg = DegFromX(pos.x());
            update();
            return;
        }
    }
    if (k != m_hoverKnot) {
        m_hoverKnot = k;
        setCursor(k >= 0 ? Qt::SizeAllCursor : Qt::ArrowCursor);
        const bool endsTimeline = !m_axis.cyclic && k == KnotCount(m_axis) - 1;
        setToolTip(k < 0 ? QString()
                   : endsTimeline
                       ? QStringLiteral("Drag sideways to change this axis's duration.  Alt: no snap.")
                       : QStringLiteral("Drag to move.  Shift: vertical only.  Ctrl: horizontal only.  Alt: no snap."));
    }
    m_hoverDeg = InPlot(pos) ? DegFromX(pos.x()) : -1.0;
    update();
}

void ProfileStrip::mouseReleaseEvent(QMouseEvent *e) {
    if (e->button() == Qt::MiddleButton) {
        if (m_panning) {
            m_panning = false;
            unsetCursor();
        }
        return;
    }
    if (e->button() != Qt::LeftButton) return;
    m_scrubbing = false;
    if (m_grab != Grab::None) {
        m_grab = Grab::None;
        m_pulse = -1;
        emit pulseDragFinished();
        RebuildLanes();  // a lane whose last pulse moved away goes now
        update();
        return;
    }
    if (m_dragKnot >= 0) {
        m_dragKnot = -1;
        m_rangeFrozen = false;
        FitRange();
        emit dragFinished();
        update();
    }
}

void ProfileStrip::mouseDoubleClickEvent(QMouseEvent *e) {
    if (e->button() != Qt::LeftButton) return;
    if (m_locked) {
        if (LaneAt(e->position().y()) >= 0 || m_disp.contains(e->position())) emit lockedEditAttempted();
        return;
    }
    {
        const int lane = LaneAt(e->position().y());
        Grab grab;
        if (lane >= 0) {
            if (PulseAt(e->position(), &grab) < 0) {
                emit pulseAddRequested(lane, SnapDeg(DegFromX(e->position().x()), e->modifiers()));
            }
            return;
        }
    }
    if (HitKnot(e->position()) >= 0 || !m_disp.contains(e->position())) return;
    if (DegFromX(e->position().x()) >= m_axis.span) return;  // in the hold region
    emit splitRequested(SnapDeg(DegFromX(e->position().x()), e->modifiers()));
}

void ProfileStrip::leaveEvent(QEvent *) {
    m_hoverDeg = -1.0;
    m_hoverKnot = -1;
    m_hoverMarker = -1;
    update();
}

void ProfileStrip::contextMenuEvent(QContextMenuEvent *e) {
    if (m_locked) {
        if (InPlot(e->pos()) || LaneAt(e->pos().y()) >= 0) emit lockedEditAttempted();
        return;
    }
    if (LaneAt(e->pos().y()) >= 0) {
        Grab grab;
        const int pulse = PulseAt(e->pos(), &grab);
        if (pulse < 0) return;
        const OutputPulse &o = m_outputs[pulse];
        QMenu menu(this);
        menu.addSection(QStringLiteral("IO-%1  %2 to %3").arg(o.output).arg(XText(o.on), XText(o.off)));
        QAction *del = menu.addAction(QStringLiteral("Delete pulse"));
        if (menu.exec(e->globalPos()) == del) emit pulseDeleteRequested(pulse);
        return;
    }
    if (!InPlot(e->pos()) || (!m_outBand.isNull() && m_outBand.contains(e->pos()))) return;
    const double deg = SnapDeg(DegFromX(e->pos().x()), Qt::NoModifier);
    const int seg = SegmentAt(m_axis, std::min(deg, m_axis.span * 0.999999));
    emit segmentClicked(seg);

    QMenu menu(this);
    QAction *split = menu.addAction(QStringLiteral("Split segment at %1").arg(XText(deg, 1)));
    split->setEnabled(deg < m_axis.span);
    QAction *del = menu.addAction(QStringLiteral("Delete segment %1").arg(seg + 1));
    del->setEnabled(m_axis.segments.size() > 1);
    menu.addSeparator();
    QMenu *laws = menu.addMenu(QStringLiteral("Motion law"));
    for (int l = 0; l < static_cast<int>(MotionLaw::Count); ++l) {
        QAction *a = laws->addAction(QString::fromLatin1(LawName(static_cast<MotionLaw>(l))));
        a->setCheckable(true);
        a->setChecked(static_cast<int>(m_axis.segments[seg].law) == l);
        a->setData(l);
    }
    QAction *chosen = menu.exec(e->globalPos());
    if (!chosen) return;
    if (chosen == split) {
        emit splitRequested(deg);
    } else if (chosen == del) {
        emit deleteRequested(seg);
    } else if (chosen->data().isValid()) {
        emit lawRequested(seg, chosen->data().toInt());
    }
}
