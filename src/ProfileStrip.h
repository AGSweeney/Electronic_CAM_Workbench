/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// The displacement diagram: follower position against the master (crank angle 0..360 deg, or time
// on a timeline), with the knots (segment boundaries) draggable, an optional velocity /
// acceleration / jerk strip below sharing the same master axis, and the Outputs band at the bottom:
// one lane per IO-0..IO-5 with the project's output pulses on this axis's master (crank degrees or
// timeline seconds). The other master's pulses are counted, greyed, in their lane.
//
// Outputs: drag a pulse to move it (also onto another lane), drag an end to change its on / off,
// double-click a lane to add one, right-click a pulse to delete it. Snaps like the knots (Alt: none).
//
// Locked (setLocked): knots, segments and pulses can't be edited; scrubbing, zoom and pan still work, and an
// attempt to edit emits lockedEditAttempted.
//
// Zoom: the master axis (all three bands together). Ctrl+wheel zooms about the mouse, + / - (when
// focused) about the cursor line, 0 fits; middle-drag or Shift+wheel pans while zoomed.

#include "CamProfile.h"

#include <QWidget>

#include <vector>

class ProfileStrip : public QWidget {
    Q_OBJECT
public:
    explicit ProfileStrip(QWidget *parent = nullptr);

    // viewSpan: master length shown (360 on a crank; the longest axis on a timeline).
    void setAxis(const ecam::CamAxis &axis, const ecam::CamMaster &master, double viewSpan = 360.0);
    void setSelectedSegment(int seg);
    void setCursorDeg(double deg);
    void setCurvesVisible(bool vel, bool acc, bool jerk);
    void setSnap(double deg, double pos);  // 0 = off
    // Output pulses of the whole project; live = the board's outputs (bit n = IO-n) when liveValid.
    void setOutputs(const std::vector<ecam::OutputPulse> &pulses);
    void setLiveOutputs(unsigned mask, bool valid);
    void setOutputsVisible(bool on);
    // Cut-length mode: the sync window (deg as drawn), shaded in the diagram; start < 0 none.
    void setSyncWindow(double startDeg, double endDeg);
    void setLocked(bool locked);
    bool locked() const { return m_locked; }

    QSize sizeHint() const override { return {900, 320}; }
    QPointF knotPointForDemo(int k) const { return KnotPoint(k); }  // tools/ecam_demo: where a knot is drawn
    QSize minimumSizeHint() const override { return {480, 200}; }

    // The cursor readout (or the dragged knot's), as rich text for a label outside the plot.
    QString readoutHtml() const;

    // Master-axis zoom: 1 = the whole view span. zoomBy keeps the cursor line in place (or the
    // middle of the view when the cursor is off screen).
    double zoom() const { return m_zoom; }
    double viewStart() const { return m_x0; }
    double visibleSpan() const { return m_view / m_zoom; }
    void zoomBy(double factor);
    void resetZoom();
    void panBy(double masterUnits);

signals:
    // The zoom (1 = fit) or the visible window changed.
    void zoomChanged(double zoom);
    // The readout text changed (cursor moved, knot dragged, axis edited).
    void readoutChanged(const QString &html);
    void cursorMoved(double deg);
    void segmentClicked(int seg);
    void dragStarted();
    void knotDragged(int knot, double deg, double pos);
    void dragFinished();
    void splitRequested(double deg);
    void deleteRequested(int seg);
    void lawRequested(int seg, int law);  // ecam::MotionLaw
    // Output editing. A drag: pulseDragStarted, pulsesDragged (live, unsorted) ..., pulseDragFinished.
    // Indices are into the project's outputs; x is in this axis's master units.
    void pulseDragStarted();
    void pulsesDragged(const std::vector<ecam::OutputPulse> &pulses);
    void pulseDragFinished();
    void pulseAddRequested(int output, double x);
    void pulseDeleteRequested(int index);
    // A knot or pulse was grabbed, a lane or segment double-clicked or right-clicked while locked.
    void lockedEditAttempted();

protected:
    void paintEvent(QPaintEvent *) override;
    void resizeEvent(QResizeEvent *) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void mouseDoubleClickEvent(QMouseEvent *e) override;
    void leaveEvent(QEvent *) override;
    void wheelEvent(QWheelEvent *e) override;
    void keyPressEvent(QKeyEvent *e) override;
    void contextMenuEvent(QContextMenuEvent *e) override;

private:
    void Layout();
    void FitRange();
    void ZoomAbout(double factor, double anchorDeg);
    void ClampPan();
    double XFromDeg(double deg) const;
    double DegFromX(double x) const;
    double YFromPos(double pos) const;
    double PosFromY(double y) const;
    QPointF KnotPoint(int k) const;
    int HitKnot(const QPointF &p) const;
    bool InPlot(const QPointF &p) const;
    double SnapDeg(double deg, Qt::KeyboardModifiers mods) const;
    double SnapPos(double pos, Qt::KeyboardModifiers mods) const;
    ecam::CamPoint PointAt(double deg) const;  // 360 = end of the cycle

    void DrawBands(QPainter &p, const QRectF &r) const;
    void DrawAngleGrid(QPainter &p, const QRectF &r, bool labels) const;
    void DrawDisplacement(QPainter &p) const;
    void DrawDerivatives(QPainter &p) const;
    void DrawCursor(QPainter &p) const;
    void DrawOutputs(QPainter &p) const;

    // A problem the analysis found on this axis, marked where it happens: a warning sign over the curve and a dashed
    // line down through the plots. Hover the sign for what it is and what to do.
    struct Marker {
        double deg = 0.0;   // master position
        double pos = 0.0;   // axis position the sign points at
        bool error = false; // red (an error in the project review) or amber (a warning)
        QString text;       // the tooltip
    };
    void BuildMarkers();
    std::vector<QRectF> MarkerRects() const;  // the sign of each marker, in widget coordinates (empty if off-screen)
    int HitMarker(const QPointF &p) const;
    void DrawMarkers(QPainter &p) const;
    std::vector<Marker> m_markers;
    int m_hoverMarker = -1;

    enum class Grab { None, Move, On, Off };
    ecam::AxisMaster ViewMaster() const { return m_time ? ecam::AxisMaster::Timeline : ecam::AxisMaster::Crank; }
    QRectF Lane(int output) const;
    QRectF PulseRect(const ecam::OutputPulse &p) const;
    int LaneAt(double y) const;                 // output index, -1 outside the band
    int PulseAt(QPointF pos, Grab *grab) const;  // index into m_outputs (this master only), -1 for none

    ecam::CamAxis m_axis;
    ecam::CamMaster m_master;
    ecam::AxisStats m_stats;
    QRectF m_disp;
    QRectF m_deriv;
    double m_yMin = 0.0, m_yMax = 1.0;
    double m_view = 360.0;  // master length across the plot
    bool m_time = false;    // timeline: x is seconds
    QString XText(double x, int decimals = 2) const;
    bool m_rangeFrozen = false;

    QString m_lastReadout;
    double m_zoom = 1.0;  // master-axis zoom
    double m_x0 = 0.0;    // master position at the left edge of the plot
    bool m_panning = false;
    double m_panStartX = 0.0, m_panStartX0 = 0.0;
    int m_selected = -1;
    double m_cursorDeg = 0.0;
    bool m_showVel = true, m_showAcc = true, m_showJerk = false;
    double m_snapDeg = 1.0, m_snapPos = 0.0;

    bool m_locked = false;
    int m_hoverKnot = -1;
    int m_dragKnot = -1;
    bool m_scrubbing = false;
    QPointF m_dragOrigin;
    double m_dragDeg0 = 0.0, m_dragPos0 = 0.0;
    double m_hoverDeg = -1.0;

    // Outputs
    std::vector<ecam::OutputPulse> m_outputs;
    // The band's lanes: the outputs that have a pulse (either master), in order. Held while a pulse is dragged, so a
    // lane doesn't go from under the mouse when its last pulse moves to another.
    std::vector<int> m_lanes;
    void RebuildLanes();
    bool m_showOutputs = true;
    QRectF m_outBand;
    double m_laneH = 14.0;
    double m_syncStart = -1.0, m_syncEnd = -1.0;
    unsigned m_liveMask = 0;
    bool m_liveValid = false;
    Grab m_grab = Grab::None;
    int m_pulse = -1;
    double m_grabX = 0.0;
    ecam::OutputPulse m_pulseStart;
};
