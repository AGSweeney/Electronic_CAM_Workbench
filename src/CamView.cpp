/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "CamView.h"

#include "CamGeometry.h"

#include "Theme.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

using namespace ecam;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kSamples = 720;

double Wrap360(double d) {
    d = std::fmod(d, 360.0);
    return d < 0.0 ? d + 360.0 : d;
}

}  // namespace

CamView::CamView(QWidget *parent) : QWidget(parent) {
    setMouseTracking(false);
    setCursor(Qt::OpenHandCursor);
    m_axis = DefaultAxis("Axis 1");
    Rebuild();
}

void CamView::setAxis(const CamAxis &axis) {
    m_axis = axis;
    Rebuild();
    update();
}

void CamView::setAngle(double deg) {
    m_angle = deg;
    update();
}

void CamView::setSelectedSegment(int seg) {
    m_selected = seg;
    update();
}

double CamView::PitchRadius(double pos) const {
    return m_axis.baseRadius + m_axis.rollerRadius + (pos - m_stats.minPos);
}

void CamView::Rebuild() {
    m_stats = Analyze(m_axis, 1440);
    // Same geometry as the DXF export (core/CamGeometry).
    const CamShape shape = BuildCamShape(m_axis, kSamples);
    m_pitch.resize(kSamples + 1);
    m_surface.resize(kSamples + 1);
    m_segOf.resize(kSamples + 1);
    m_extentR = 0.0;
    for (int i = 0; i <= kSamples; ++i) {
        m_pitch[i] = QPointF(shape.pitch[i].x, shape.pitch[i].y);
        m_surface[i] = QPointF(shape.surface[i].x, shape.surface[i].y);
        m_segOf[i] = shape.segment[i];
        m_extentR = std::max(m_extentR, std::hypot(shape.pitch[i].x, shape.pitch[i].y));
    }
}

QPointF CamView::Center() const {
    // Leave room above the cam for the follower stem.
    const double stem = m_extentR * 0.55 + m_axis.rollerRadius;
    const double worldH = m_extentR * 2 + stem;
    const double worldW = m_extentR * 2;
    const double margin = 24.0;
    const double sx = (width() - 2 * margin) / worldW;
    const double sy = (height() - 2 * margin) / worldH;
    const_cast<CamView *>(this)->m_scale = std::max(1e-6, std::min(sx, sy));
    const double usedH = worldH * m_scale;
    const double top = (height() - usedH) / 2.0;
    return {width() / 2.0, top + (stem + m_extentR) * m_scale};
}

QPointF CamView::Rotate(const QPointF &c) const {
    const double a = -m_angle * kPi / 180.0;
    return {c.x() * std::cos(a) - c.y() * std::sin(a), c.x() * std::sin(a) + c.y() * std::cos(a)};
}

QPointF CamView::ToScreen(const QPointF &cam) const {
    const QPointF w = Rotate(cam);
    const QPointF c = Center();
    return {c.x() + w.x() * m_scale, c.y() - w.y() * m_scale};
}

void CamView::paintEvent(QPaintEvent *) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), theme::kCanvas);
    const QPointF ctr = Center();
    const double s = m_scale;

    // Fixed crosshair through the centre and the follower axis.
    p.setPen(QPen(theme::kGrid, 1, Qt::DashLine));
    p.drawLine(QPointF(ctr.x() - m_extentR * s * 1.08, ctr.y()), QPointF(ctr.x() + m_extentR * s * 1.08, ctr.y()));
    p.drawLine(QPointF(ctr.x(), ctr.y() + m_extentR * s * 1.08), QPointF(ctr.x(), 4));

    // Cam body
    QPolygonF body;
    for (const QPointF &pt : m_surface) body << ToScreen(pt);
    p.setPen(QPen(theme::kCamEdge, 2.0));
    p.setBrush(theme::kCamFill);
    p.drawPolygon(body);

    // Base circle
    p.setPen(QPen(theme::kGridMajor, 1, Qt::DashLine));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(ctr, m_axis.baseRadius * s, m_axis.baseRadius * s);

    // Segment boundaries (radial) and the pitch curve coloured by segment.
    for (int k = 0; k < KnotCount(m_axis) - 1; ++k) {
        const double a = (90.0 + KnotDeg(m_axis, k)) * kPi / 180.0;
        const double r = PitchRadius(KnotPos(m_axis, k));
        p.setPen(QPen(QColor(0xb8, 0xc4, 0xd6, 60), 1));
        p.drawLine(ToScreen(QPointF(0, 0)), ToScreen(QPointF(r * std::cos(a), r * std::sin(a))));
    }
    if (m_axis.rollerRadius > 0.0) {
        for (int i = 0; i < kSamples; ++i) {
            const int seg = m_segOf[i];
            QColor c = theme::SegmentColor(seg);
            const bool sel = seg == m_selected;
            c.setAlpha(sel ? 255 : 120);
            p.setPen(QPen(c, sel ? 2.4 : 1.2, sel ? Qt::SolidLine : Qt::DashLine));
            p.drawLine(ToScreen(m_pitch[i]), ToScreen(m_pitch[i + 1]));
        }
    } else if (m_selected >= 0) {
        for (int i = 0; i < kSamples; ++i) {
            if (m_segOf[i] != m_selected) continue;
            p.setPen(QPen(theme::kSelect, 3.0));
            p.drawLine(ToScreen(m_pitch[i]), ToScreen(m_pitch[i + 1]));
        }
    }

    // Hub, angle marks on the cam (they turn with it), rotation arrow.
    const double hubR = std::max(4.0, m_axis.baseRadius * 0.16 * s);
    p.setPen(QPen(theme::kCamEdge, 1.5));
    p.setBrush(QColor(0x2a, 0x2f, 0x36));
    p.drawEllipse(ctr, hubR, hubR);
    p.setBrush(theme::kCamEdge);
    p.drawEllipse(ctr, 2.5, 2.5);
    p.setFont(QFont(font().family(), 8, QFont::DemiBold));
    for (int d = 0; d < 360; d += 90) {
        const double a = (90.0 + d) * kPi / 180.0;
        const double r = m_axis.baseRadius * 0.62;
        const QPointF sp = ToScreen(QPointF(r * std::cos(a), r * std::sin(a)));
        const QPointF tick0 = ToScreen(QPointF(m_axis.baseRadius * 0.82 * std::cos(a), m_axis.baseRadius * 0.82 * std::sin(a)));
        const QPointF tick1 = ToScreen(QPointF(m_axis.baseRadius * 0.95 * std::cos(a), m_axis.baseRadius * 0.95 * std::sin(a)));
        p.setPen(QPen(theme::kAxisText, 1.5));
        p.drawLine(tick0, tick1);
        if (m_axis.baseRadius * s > 40) {
            p.drawText(QRectF(sp.x() - 20, sp.y() - 8, 40, 16), Qt::AlignCenter, QStringLiteral("%1°").arg(d));
        }
    }
    {
        const double ar = hubR + 10;
        QPainterPath arc;
        arc.arcMoveTo(QRectF(ctr.x() - ar, ctr.y() - ar, 2 * ar, 2 * ar), 60);
        arc.arcTo(QRectF(ctr.x() - ar, ctr.y() - ar, 2 * ar, 2 * ar), 60, -70);
        p.setPen(QPen(theme::kAxisText, 1.4));
        p.setBrush(Qt::NoBrush);
        p.drawPath(arc);
        const QPointF tip = arc.currentPosition();
        const double ta = -10.0 * kPi / 180.0;  // end of the arc, heading clockwise
        const QPointF dir(std::sin(ta), std::cos(ta));
        const QPointF side(-dir.y(), dir.x());
        QPolygonF head;
        head << tip + dir * 6 << tip - side * 4 << tip + side * 4;
        p.setBrush(theme::kAxisText);
        p.setPen(Qt::NoPen);
        p.drawPolygon(head);
    }

    // Follower at 12 o'clock.
    const CamPoint now = Evaluate(m_axis, m_angle);
    const double rNow = PitchRadius(now.pos);
    const QPointF rc(ctr.x(), ctr.y() - rNow * s);  // roller centre
    const double rrPx = m_axis.rollerRadius * s;
    const double stemW = std::max(6.0, std::min(rrPx * 0.9, m_extentR * s * 0.12));
    const double stemTop = 6.0;
    // Guide bushing (fixed)
    const double guideY = ctr.y() - (m_extentR + m_axis.rollerRadius) * s - 16;
    p.setPen(QPen(theme::kGridMajor, 1));
    p.setBrush(QColor(0x2c, 0x31, 0x38));
    p.drawRect(QRectF(ctr.x() - stemW / 2 - 9, guideY - 26, 7, 26));
    p.drawRect(QRectF(ctr.x() + stemW / 2 + 2, guideY - 26, 7, 26));
    // Stem
    p.setPen(QPen(theme::kFollowerDark, 1));
    p.setBrush(theme::kFollower);
    p.drawRect(QRectF(ctr.x() - stemW / 2, stemTop, stemW, rc.y() - stemTop));
    if (m_axis.rollerRadius > 0.0) {
        p.drawEllipse(rc, rrPx, rrPx);
        p.setBrush(theme::kFollowerDark);
        p.drawEllipse(rc, std::max(1.5, rrPx * 0.3), std::max(1.5, rrPx * 0.3));
    } else {
        QPolygonF knife;
        knife << QPointF(rc.x(), rc.y()) << QPointF(rc.x() - stemW / 2, rc.y() - stemW)
              << QPointF(rc.x() + stemW / 2, rc.y() - stemW);
        p.drawPolygon(knife);
    }

    // Common normal at the contact (its angle from the follower axis is the pressure angle).
    const double r1 = now.vel * 180.0 / kPi;  // dR/dtheta per radian
    const double phi = std::atan2(r1, rNow);
    {
        // The cam turns CW, so a rising profile tilts the normal toward +x at the follower.
        const QPointF dir(std::sin(phi), -std::cos(phi));  // screen: up is -y
        const double L = std::max(30.0, rrPx * 3.0);
        p.setPen(QPen(theme::kAccel, 1.2, Qt::DashLine));
        p.drawLine(rc - dir * L * 0.4, rc + dir * L);
    }

    // Readout
    const QString u = QString::fromStdString(m_axis.units);
    QStringList lines;
    lines << QStringLiteral("θ  %1°").arg(QString::number(Wrap360(m_angle), 'f', 2))
          << QStringLiteral("s  %1 %2").arg(QString::number(now.pos, 'f', 3), u)
          << QStringLiteral("φ  %1°").arg(QString::number(std::fabs(phi) * 180.0 / kPi, 'f', 1));
    p.setFont(QFont(QStringLiteral("Consolas"), 9));
    const QFontMetrics fm = p.fontMetrics();
    int tw = 0;
    for (const QString &l : lines) tw = std::max(tw, fm.horizontalAdvance(l));
    const QRectF box(10, 10, tw + 16, lines.size() * fm.height() + 10);
    p.setPen(QPen(QColor(0x3a, 0x41, 0x4c), 1));
    p.setBrush(QColor(20, 23, 28, 210));
    p.drawRoundedRect(box, 3, 3);
    p.setPen(theme::kText);
    for (int i = 0; i < lines.size(); ++i) {
        p.drawText(QPointF(box.left() + 8, box.top() + 5 + fm.ascent() + i * fm.height()), lines[i]);
    }

    // Geometry warnings
    QStringList warn;
    for (const CamIssue &is : m_stats.issues) {
        if (is.kind == IssueKind::Undercut) {
            warn << QStringLiteral("Undercut near %1° (pitch radius %2 < roller)")
                        .arg(QString::number(is.deg, 'f', 0), QString::number(is.left, 'g', 3));
        } else if (is.kind == IssueKind::PressureAngle) {
            warn << QStringLiteral("Pressure angle %1° at %2°")
                        .arg(QString::number(is.left, 'f', 1), QString::number(is.deg, 'f', 0));
        }
    }
    if (u == "deg" || u == QStringLiteral("°")) warn << QStringLiteral("Rotary axis: the cam shape is notional");
    p.setFont(QFont(font().family(), 8, QFont::DemiBold));
    for (int i = 0; i < warn.size(); ++i) {
        p.setPen(theme::kWarn);
        p.drawText(QRectF(10, height() - 20 - (warn.size() - 1 - i) * 16, width() - 20, 16),
                   Qt::AlignLeft | Qt::AlignVCenter, warn[i]);
    }
}

void CamView::mousePressEvent(QMouseEvent *e) {
    if (e->button() != Qt::LeftButton) return;
    const QPointF d = e->position() - Center();
    m_dragAngle = std::atan2(-d.y(), d.x()) * 180.0 / kPi;
    m_dragging = true;
    setCursor(Qt::ClosedHandCursor);
}

void CamView::mouseMoveEvent(QMouseEvent *e) {
    if (!m_dragging) return;
    const QPointF d = e->position() - Center();
    const double a = std::atan2(-d.y(), d.x()) * 180.0 / kPi;
    double delta = a - m_dragAngle;
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;
    m_dragAngle = a;
    // Dragging clockwise (negative math angle) advances the master.
    emit angleChanged(Wrap360(m_angle - delta));
}

void CamView::mouseReleaseEvent(QMouseEvent *) {
    m_dragging = false;
    setCursor(Qt::OpenHandCursor);
}

void CamView::wheelEvent(QWheelEvent *e) {
    const double step = (e->modifiers() & Qt::ShiftModifier) ? 10.0 : 1.0;
    const double dir = e->angleDelta().y() > 0 ? -1.0 : 1.0;
    emit angleChanged(Wrap360(std::round(m_angle) + dir * step));
}
