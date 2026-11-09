/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Timing chart of every timeline axis: one row per axis on the shared time axis (t = 0 common),
// each curve scaled to its own stroke. Past an axis's duration it rests at its end position.
// Click a row to edit that axis; click or drag to move the time cursor. Drag a wait's line to move the wait (red where a
// timeline axis moves: it snaps to the nearest rest when dropped there).

#include "CamProfile.h"

#include <QWidget>

class TimingView : public QWidget {
    Q_OBJECT
public:
    explicit TimingView(QWidget *parent = nullptr);

    void setProject(const ecam::CamProject &project, int currentAxis);
    void setTime(double t);
    void setActiveWait(int index);  // the board's wait holding now, 1-based (0 none): its marker lights
    void setWaitsEditable(bool on) { m_waitsEditable = on; }  // off while the profile is locked
    double xForTime(double t) const { return XFromT(t); }     // tests: where a time is drawn
    int draggedWait() const { return m_dragWait; }

    QSize sizeHint() const override { return {520, 520}; }
    QSize minimumSizeHint() const override { return {200, 120}; }

signals:
    void axisClicked(int axis);
    void timeChanged(double t);
    void waitRequested(double t);  // right-click: Wait for an input here
    void waitMoved(int index, double t);  // a wait's line dropped at t (0-based index, in time order)

protected:
    void paintEvent(QPaintEvent *) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void contextMenuEvent(QContextMenuEvent *e) override;

private:
    QRectF Plot() const;
    double XFromT(double t) const;
    double TFromX(double x) const;
    int RowAt(double y) const;  // index into m_rows
    int WaitAt(double x) const;  // the wait whose line is within a few pixels, -1 none

    ecam::CamProject m_project;
    QVector<int> m_rows;  // project axis index of each row
    int m_current = 0;
    double m_t = 0.0;
    double m_length = 1.0;
    bool m_dragging = false;
    int m_activeWait = 0;
    bool m_waitsEditable = true;
    int m_dragWait = -1;  // the wait being dragged
    double m_dragT = 0.0;
};
