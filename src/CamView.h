/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Draws the profile as a physical plate cam with a translating radial roller follower.
// The cam turns clockwise with the master angle; the follower sits at 12 o'clock.
// Drag to turn the cam; the wheel steps 1 deg (Shift: 10 deg).

#include "CamProfile.h"

#include <QPointF>
#include <QVector>
#include <QWidget>

class CamView : public QWidget {
    Q_OBJECT
public:
    explicit CamView(QWidget *parent = nullptr);

    void setAxis(const ecam::CamAxis &axis);
    void setAngle(double deg);
    void setSelectedSegment(int seg);

    QSize sizeHint() const override { return {520, 520}; }
    QSize minimumSizeHint() const override { return {200, 120}; }

signals:
    void angleChanged(double deg);

protected:
    void paintEvent(QPaintEvent *) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;

private:
    void Rebuild();
    // Cam-frame point (units) -> screen, with the cam turned to m_angle.
    QPointF ToScreen(const QPointF &cam) const;
    QPointF Rotate(const QPointF &cam) const;  // cam frame -> world (units)
    QPointF Center() const;
    double PitchRadius(double pos) const;

    ecam::CamAxis m_axis;
    ecam::AxisStats m_stats;
    QVector<QPointF> m_pitch;    // cam frame, profile angle 0..360 inclusive
    QVector<QPointF> m_surface;  // pitch offset inward by the roller radius
    QVector<int> m_segOf;        // segment of each sample
    double m_angle = 0.0;
    int m_selected = -1;
    double m_scale = 1.0;    // px per unit
    double m_extentR = 1.0;  // largest pitch radius
    bool m_dragging = false;
    double m_dragAngle = 0.0;
};
