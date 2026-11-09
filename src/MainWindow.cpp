/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "MainWindow.h"

#include "BoardLink.h"
#include "Brand.h"
#include "CamGeometry.h"
#include "CamPoints.h"
#include "CamView.h"
#include "CollapsibleGroup.h"
#include "SideDock.h"
#include "ProfileText.h"
#include "ProfileStrip.h"
#include "FirmwareDialog.h"
#include "JogDialog.h"
#include "SwitchWizard.h"
#include "OperatorDialog.h"
#include "FirmwareUpdate.h"
#include "HelpWindow.h"
#include "ProjectIo.h"
#include "ProjectReview.h"
#include "ScopeWindow.h"
#include "SetupWizard.h"
#include "StartupUi.h"
#include "ProjectReport.h"
#include "WorkbookExport.h"
#include "TimingView.h"
#include "Theme.h"

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QFileDialog>
#include <QDesktopServices>
#include <QDir>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QIcon>
#include <QInputDialog>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QMenuBar>
#include <QMessageBox>
#include <QPixmap>
#include <QPainter>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QScrollBar>
#include <QSerialPortInfo>
#include <QSettings>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QSplitter>
#include <QStandardItemModel>
#include <QStackedWidget>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>

#include <functional>
#include <map>
#include <memory>
#include <QStatusBar>
#include <QTableWidget>
#include <QTimer>
#include <QUrl>
#include <QUndoCommand>
#include <QUndoStack>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace ecam;

namespace {

enum Column { kColIndex, kColLaw, kColStartDeg, kColEndDeg, kColSpan, kColStartPos, kColEndPos, kColRise, kColCount };

QSettings &AppSettings() {
    static QSettings s(QStringLiteral("ECamUI"), QStringLiteral("ECamUI"));
    return s;
}

bool SameAxis(const CamAxis &a, const CamAxis &b) {
    if (a.name != b.name || a.units != b.units || a.countsPerUnit != b.countsPerUnit || a.startPos != b.startPos ||
        a.closed != b.closed || a.reverse != b.reverse || a.baseRadius != b.baseRadius || a.rollerRadius != b.rollerRadius ||
        a.master != b.master || a.span != b.span || a.connector != b.connector || a.velMax != b.velMax ||
        a.accMax != b.accMax || a.drive != b.drive || a.motorPpr != b.motorPpr || a.screwPitch != b.screwPitch ||
        a.pulleyPd != b.pulleyPd || a.gear != b.gear || a.gearNum != b.gearNum || a.gearDen != b.gearDen ||
        a.segments.size() != b.segments.size()) {
        return false;
    }
    const AxisHome &h = a.home, &k = b.home;
    if (h.required != k.required || h.dir != k.dir || h.fast != k.fast || h.slow != k.slow || h.back != k.back || h.at != k.at ||
        h.thenStart != k.thenStart || h.order != k.order || h.search != k.search || h.decel != k.decel) {
        return false;
    }
    for (size_t i = 0; i < a.segments.size(); ++i) {
        const CamSegment &x = a.segments[i], &y = b.segments[i];
        if (x.endDeg != y.endDeg || x.endPos != y.endPos || x.law != y.law) return false;
    }
    return true;
}

bool SameProject(const CamProject &a, const CamProject &b) {
    const CamMaster &x = a.master, &y = b.master;
    if (x.source != y.source || x.crankConnector != y.crankConnector || x.crankRampRpmPerSec != y.crankRampRpmPerSec ||
        x.crankMaxRpm != y.crankMaxRpm || x.timelineRepeat != y.timelineRepeat || x.speedOverride != y.speedOverride ||
        x.trigger != y.trigger || x.triggerInput != y.triggerInput || x.triggerRising != y.triggerRising ||
        x.triggerBit != y.triggerBit || x.crankPpr != y.crankPpr || x.crankGearNum != y.crankGearNum ||
        x.crankGearDen != y.crankGearDen || x.lineMode != y.lineMode || x.wheelCircumference != y.wheelCircumference ||
        x.designLength != y.designLength || x.kerf != y.kerf || x.encoderReverse != y.encoderReverse ||
        x.crankReverse != y.crankReverse) {
        return false;
    }
    if (a.master.countsPerRev != b.master.countsPerRev || a.master.rpm != b.master.rpm ||
        a.axes.size() != b.axes.size()) {
        return false;
    }
    for (size_t i = 0; i < a.axes.size(); ++i) {
        if (!SameAxis(a.axes[i], b.axes[i])) return false;
    }
    if (a.outputs.size() != b.outputs.size()) return false;
    for (size_t i = 0; i < a.outputs.size(); ++i) {
        const OutputPulse &p = a.outputs[i], &q = b.outputs[i];
        if (p.output != q.output || p.master != q.master || p.on != q.on || p.off != q.off) return false;
    }
    // The inputs too: an edit of only them is an edit (an undo step, the file changed).
    if (a.ccio.boards != b.ccio.boards || a.ccio.inputs.size() != b.ccio.inputs.size() || a.inputs.size() != b.inputs.size()) return false;
    for (size_t i = 0; i < a.ccio.inputs.size(); ++i) {
        const CcioInput &p = a.ccio.inputs[i], &q = b.ccio.inputs[i];
        if (p.board != q.board || p.pin != q.pin || p.name != q.name || p.role != q.role || p.axis != q.axis || p.normallyOpen != q.normallyOpen) return false;
    }
    for (size_t i = 0; i < a.inputs.size(); ++i) {
        const BoardInput &p = a.inputs[i], &q = b.inputs[i];
        if (p.pin != q.pin || p.name != q.name || p.role != q.role || p.axis != q.axis || p.normallyOpen != q.normallyOpen) return false;
    }
    if (a.master.limitStopSeconds != b.master.limitStopSeconds) return false;
    if (a.waits.size() != b.waits.size()) return false;
    for (size_t i = 0; i < a.waits.size(); ++i) {
        const TimelineWait &p = a.waits[i], &q = b.waits[i];
        if (p.t != q.t || p.name != q.name || p.source != q.source || p.board != q.board || p.pin != q.pin || p.on != q.on ||
            p.timeout != q.timeout || p.edge != q.edge) {
            return false;
        }
    }
    return true;
}

QString Fmt(double v, int dec = 3) { return QString::number(v, 'f', dec); }

QString Sig(double v) {
    if (std::fabs(v) < 1e-12) return QStringLiteral("0");
    return QString::number(v, 'g', 5);
}

class ProjectCommand : public QUndoCommand {
public:
    ProjectCommand(MainWindow *w, CamProject before, int axisBefore, CamProject after, int axisAfter,
                   const QString &text, int mergeId)
        : QUndoCommand(text),
          m_w(w),
          m_before(std::move(before)),
          m_after(std::move(after)),
          m_axisBefore(axisBefore),
          m_axisAfter(axisAfter),
          m_id(mergeId) {}

    void undo() override { m_w->Restore(m_before, m_axisBefore); }
    void redo() override {
        if (m_first) {  // already applied by the edit itself
            m_first = false;
            return;
        }
        m_w->Restore(m_after, m_axisAfter);
    }
    int id() const override { return m_id; }
    bool mergeWith(const QUndoCommand *other) override {
        const auto *o = static_cast<const ProjectCommand *>(other);
        if (o->m_id != m_id || o->m_axisBefore != m_axisAfter) return false;
        m_after = o->m_after;
        m_axisAfter = o->m_axisAfter;
        return true;
    }

private:
    MainWindow *m_w;
    CamProject m_before, m_after;
    int m_axisBefore, m_axisAfter;
    int m_id;
    bool m_first = true;
};

QDoubleSpinBox *MakeSpin(QWidget *parent, double lo, double hi, int decimals, const QString &suffix = {}) {
    auto *s = new QDoubleSpinBox(parent);
    s->setRange(lo, hi);
    s->setDecimals(decimals);
    s->setKeyboardTracking(false);
    s->setAccelerated(true);
    if (!suffix.isEmpty()) s->setSuffix(suffix);
    return s;
}

}  // namespace

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    m_project = DefaultProject();
    m_undo = new QUndoStack(this);
    m_anim = new QTimer(this);
    m_anim->setInterval(16);
    connect(m_anim, &QTimer::timeout, this, &MainWindow::AnimTick);
    connect(m_undo, &QUndoStack::cleanChanged, this, [this](bool) { RefreshTitle(); });
    // The board's estop input as last seen: the checks work offline too (the status corrects it when connected).
    m_estopPinKnown = AppSettings().value("board/estopPin", -1).toInt();
    {
        const QStringList ops = AppSettings().value("board/opPins").toString().split(QLatin1Char(','));
        for (int k = 0; k < 5; ++k) m_opPinsKnown[k] = k < ops.size() && !ops[k].isEmpty() ? ops[k].toInt() : -1;
    }

    BuildUi();
    BuildMenus();
    Refresh();
    SelectSegment(0);
    SetAngle(0.0);

    m_table->setFocus();
    restoreGeometry(AppSettings().value("window/geometry").toByteArray());
    m_hSplit->restoreState(AppSettings().value("window/projectSplit").toByteArray());
    m_projectSplit->restoreState(AppSettings().value("window/treeStatusSplit").toByteArray());
    m_diagramSplit->restoreState(AppSettings().value("window/topRowSplit").toByteArray());
    // A collapsed project tree gives its slot's width to the diagram (also once the window is laid out).
    m_settingsDock->SyncSplitter();
    QTimer::singleShot(0, m_settingsDock, &SideDock::SyncSplitter);
    m_camDock->SyncSplitter();
    QTimer::singleShot(0, m_camDock, &SideDock::SyncSplitter);
}

/* ------------------------------------------------------------------------- */
/* UI                                                                         */
/* ------------------------------------------------------------------------- */

namespace {
/** Calls back when the widget it watches changes size. */
class ResizeWatcher : public QObject {
public:
    ResizeWatcher(QObject *parent, std::function<void()> changed) : QObject(parent), m_changed(std::move(changed)) {}

protected:
    bool eventFilter(QObject *, QEvent *e) override {
        if (e->type() == QEvent::Resize) m_changed();
        return false;
    }

private:
    std::function<void()> m_changed;
};

// Pages of the detail pane, and what a tree node carries.
enum DetailPage { kPageCrank, kPageTimeline, kPageAxis, kPageSegments, kPageOutputs, kPageAnalysis, kPageBoard, kPageCcio, kPageInputs, kPageWaits, kPageCount };
constexpr int kRolePage = Qt::UserRole, kRoleAxis = Qt::UserRole + 1, kRoleCcioBoard = Qt::UserRole + 3;
}  // namespace

void MainWindow::BuildUi() {
    resize(1480, 940);
    setMinimumSize(1280, 720);
    setStyleSheet(theme::MainSheet());

    auto *central = new QWidget(this);
    auto *root = new QVBoxLayout(central);
    root->setContentsMargins(12, 4, 12, 12);
    root->setSpacing(10);

    // Left, the project tree. Right of it, the detail pane the tree drives beside the cam or timing view, over the
    // diagram, which takes the full width. Under everything, the board strip.
    m_hSplit = new QSplitter(Qt::Horizontal, central);
    m_hSplit->setHandleWidth(3);
    m_hSplit->setChildrenCollapsible(false);
    m_vSplit = new QSplitter(Qt::Vertical, m_hSplit);
    m_vSplit->setHandleWidth(3);
    m_vSplit->setChildrenCollapsible(false);
    m_diagramSplit = new QSplitter(Qt::Horizontal, m_vSplit);
    m_diagramSplit->setObjectName(QStringLiteral("ECamDiagramSplit"));
    m_diagramSplit->setHandleWidth(3);
    m_diagramSplit->setChildrenCollapsible(false);

    BuildSettingsPanels();

    auto *camGroup = m_viewGroup = new QGroupBox(m_diagramSplit);
    camGroup->setStyleSheet(QStringLiteral("QGroupBox { margin-top: 0; padding-top: 0; }"));  // the dock has the title
    auto *camLayout = new QVBoxLayout(camGroup);
    camLayout->setContentsMargins(10, 10, 10, 10);
    m_viewStack = new QStackedWidget(camGroup);
    m_camView = new CamView(m_viewStack);
    m_camView->setToolTip(QStringLiteral("Drag to turn the cam. Wheel: 1° (Shift: 10°)."));
    m_timing = new TimingView(m_viewStack);
    m_timing->setToolTip(QStringLiteral("Every timeline axis on the shared time axis. Click a row to edit it; drag to scrub."));
    m_viewStack->addWidget(m_camView);
    m_viewStack->addWidget(m_timing);
    camLayout->addWidget(m_viewStack);
    BuildSegmentPanel();
    QWidget *strip = BuildStripPanel();

    // The cam or timing view shares the top row with the detail pane, and folds away to give it all of it.
    camGroup->setMinimumWidth(200);
    m_camDock = new SideDock(QStringLiteral("Cam"), camGroup, QStringLiteral("cam"), SideDock::Edge::Right);
    m_camDock->setObjectName(QStringLiteral("ECamCamDock"));
    m_diagramSplit->addWidget(BuildDetailPane());
    m_diagramSplit->addWidget(m_camDock);
    m_diagramSplit->setStretchFactor(0, 1);
    m_diagramSplit->setStretchFactor(1, 1);
    m_diagramSplit->setSizes({900, 760});

    m_vSplit->addWidget(m_diagramSplit);
    m_vSplit->addWidget(strip);
    // The top row keeps the height of its tallest form, or wherever it's dragged to, and the diagram takes the rest.
    // A short window squeezes it, and it comes back when the window has the room again.
    m_vSplit->setStretchFactor(0, 0);
    m_vSplit->setStretchFactor(1, 1);
    m_topRowHeight = AppSettings().value("window/topRowHeight", 350).toInt();
    connect(m_vSplit, &QSplitter::splitterMoved, this, [this](int pos) { m_topRowHeight = pos; });
    m_vSplit->installEventFilter(new ResizeWatcher(m_vSplit, [this]() {
        const int room = m_vSplit->height() - m_vSplit->handleWidth();
        const int top = std::min(m_topRowHeight, room - m_vSplit->widget(1)->minimumSizeHint().height());
        m_vSplit->setSizes({top, room - top});
    }));

    // The project tree collapses to a strip like a docked tool window.
    m_settingsDock = new SideDock(QStringLiteral("Project"), BuildProjectPanel(), QStringLiteral("project"));
    m_settingsDock->setObjectName(QStringLiteral("ECamProjectDock"));
    m_hSplit->addWidget(m_settingsDock);
    m_hSplit->addWidget(m_vSplit);
    m_hSplit->setStretchFactor(0, 0);
    m_hSplit->setStretchFactor(1, 1);
    m_hSplit->setSizes({230, 1250});

    root->addWidget(m_hSplit, 1);
    root->addWidget(BuildStatusStrip());
    setCentralWidget(central);
    statusBar()->showMessage(
        QStringLiteral("Drag knots in the displacement diagram. Double-click a segment to split it. Right-click for more. "
                       "Ctrl+wheel zooms, middle-drag pans."));

    connect(m_camView, &CamView::angleChanged, this, &MainWindow::SetAngle);
    connect(m_timing, &TimingView::timeChanged, this, &MainWindow::SetTime);
    connect(m_timing, &TimingView::axisClicked, this, [this](int i) { m_axisCombo->setCurrentIndex(i); });
    connect(m_timing, &TimingView::waitRequested, this, &MainWindow::AddWait);
    // A wait's line dropped: to the millisecond where every timeline axis rests, or the nearest such time.
    connect(m_timing, &TimingView::waitMoved, this, [this](int k, double t) {
        if (k < 0 || k >= static_cast<int>(m_project.waits.size())) return;
        double at = std::round(t * 1000.0) / 1000.0;
        if (!TimelineAtRest(m_project, at)) at = NearestTimelineRest(m_project, t);
        if (at < 0.0) {
            statusBar()->showMessage(QStringLiteral("No time in the cycle where every timeline axis is at rest."), 5000);
            Refresh();
            return;
        }
        EditProject(QStringLiteral("Move wait"), [&](CamProject &p) { p.waits[k].t = at; });
        if (std::fabs(at - t) > 0.0015) {
            statusBar()->showMessage(QStringLiteral("Wait at %1 s, the nearest time every timeline axis rests.").arg(at, 0, 'g', 6), 5000);
        }
    });
    connect(m_strip, &ProfileStrip::cursorMoved, this, [this](double x) {
        if (OnTimeline()) {
            SetTime(x);
        } else {
            SetAngle(x);
        }
    });
    connect(m_strip, &ProfileStrip::segmentClicked, this, &MainWindow::SelectSegment);
    connect(m_strip, &ProfileStrip::segmentClicked, this, [this]() { ShowDetail(kPageSegments); });
    connect(m_strip, &ProfileStrip::lockedEditAttempted, this, [this]() {
        statusBar()->showMessage(QStringLiteral("The profile is locked against accidental edits. Unlock it (the lock in "
                                                "the diagram's bar, or Ctrl+L) to edit."),
                                 6000);
    });
    connect(m_strip, &ProfileStrip::dragStarted, this, [this]() { m_dragBefore = m_project; });
    connect(m_strip, &ProfileStrip::knotDragged, this, &MainWindow::OnKnotDragged);
    connect(m_strip, &ProfileStrip::dragFinished, this, [this]() {
        if (m_dragBefore && !SameProject(*m_dragBefore, m_project)) {
            m_undo->push(new ProjectCommand(this, *m_dragBefore, m_axis, m_project, m_axis,
                                            QStringLiteral("Move knot"), kNoMerge));
        }
        m_dragBefore.reset();
        Refresh();
    });
    connect(m_strip, &ProfileStrip::splitRequested, this, [this](double deg) {
        int newSeg = -1;
        EditAxis(QStringLiteral("Split segment"), [&](CamAxis &a) { newSeg = SplitAt(a, deg); });
        if (newSeg >= 0) SelectSegment(newSeg);
    });
    connect(m_strip, &ProfileStrip::deleteRequested, this, [this](int seg) {
        EditAxis(QStringLiteral("Delete segment"), [&](CamAxis &a) { DeleteSegment(a, seg); });
        SelectSegment(std::max(0, seg - 1));
    });
    connect(m_strip, &ProfileStrip::lawRequested, this, [this](int seg, int law) {
        EditAxis(QStringLiteral("Change motion law"),
                 [&](CamAxis &a) { SetSegmentLaw(a, seg, static_cast<MotionLaw>(law)); });
    });
    // Output pulses in the displacement diagram: a drag edits live and is one undo step when released.
    connect(m_strip, &ProfileStrip::pulseDragStarted, this, [this]() { m_dragBefore = m_project; });
    connect(m_strip, &ProfileStrip::pulsesDragged, this, [this](const std::vector<OutputPulse> &pulses) {
        m_project.outputs = pulses;  // unsorted while dragging, so the dragged index stays put
        m_strip->setOutputs(m_project.outputs);
    {
        // Cut-length mode: shade the sync window on the carriage's diagram.
        int sa = -1;
        double s0 = 0.0, s1 = 0.0;
        const bool sync = m_project.master.lineMode && LineSync(m_project, &sa, &s0, &s1) && Axis().master == AxisMaster::Crank;
        m_strip->setSyncWindow(sync ? s0 : -1.0, sync ? s1 : -1.0);
    }
        RefreshOutputs();
    });
    connect(m_strip, &ProfileStrip::pulseDragFinished, this, [this]() {
        NormalizeProject(m_project);
        if (m_dragBefore && !SameProject(*m_dragBefore, m_project)) {
            m_undo->push(new ProjectCommand(this, *m_dragBefore, m_axis, m_project, m_axis,
                                            QStringLiteral("Move output pulse"), kNoMerge));
        }
        m_dragBefore.reset();
        Refresh();
    });
    connect(m_strip, &ProfileStrip::pulseAddRequested, this, [this](int output, double x) { AddPulse(output, x); });
    connect(m_strip, &ProfileStrip::pulseDeleteRequested, this, [this](int index) {
        EditOutputs(QStringLiteral("Delete output pulse"), [&](std::vector<OutputPulse> &v) {
            if (index >= 0 && index < static_cast<int>(v.size())) v.erase(v.begin() + index);
        });
    });
}

void MainWindow::BuildSettingsPanels() {
    QWidget *panel = this;  // BuildDetailPane gives each its page

    // Master
    auto *masterGroup = m_crankGroup = new CollapsibleGroup(QStringLiteral("Crank"), QStringLiteral("crank"), panel);
    // Two columns, so the page fits the pane on a 1080p screen: what the master is, and how it runs.
    auto *masterCols = new QHBoxLayout(masterGroup->body());
    masterCols->setContentsMargins(14, 18, 14, 12);
    masterCols->setSpacing(28);
    auto *mf = m_masterForm = new QFormLayout();
    auto *mf2 = m_masterForm2 = new QFormLayout();
    mf->setSpacing(8);
    mf2->setSpacing(8);
    masterCols->addLayout(mf, 1);
    masterCols->addLayout(mf2, 1);
    m_masterSource = new QComboBox(masterGroup);
    m_masterSource->addItem(QStringLiteral("External encoder"), static_cast<int>(MasterSource::Encoder));
    m_masterSource->addItem(QStringLiteral("Driven crank"), static_cast<int>(MasterSource::DrivenCrank));
    m_masterSource->setToolTip(QStringLiteral("External encoder: the followers track a shaft encoder on the Encoder Adapter.\n"
                                              "Driven crank: the ClearCore drives the crank motor and the followers track it."));
    m_countsPerRev = MakeSpin(masterGroup, 1, 1e9, 0);  // the ppr; counts per crank rev = ppr x gear
    // Gear ratio as numerator : denominator (motor or encoder turns : crank turns), so a gearbox that
    // is not a whole ratio (50 : 7) is exact.
    auto *gearRow = m_gearRow = new QWidget(masterGroup);
    auto *gearLayout = new QHBoxLayout(gearRow);
    gearLayout->setContentsMargins(0, 0, 0, 0);
    gearLayout->setSpacing(6);
    m_gearNum = new QSpinBox(gearRow);
    m_gearDen = new QSpinBox(gearRow);
    for (QSpinBox *sb : {m_gearNum, m_gearDen}) {
        sb->setRange(1, 1000000);
        sb->setKeyboardTracking(false);
        sb->setAccelerated(true);
    }
    m_gearNum->setToolTip(QStringLiteral("Motor (or encoder) revolutions..."));
    m_gearDen->setToolTip(QStringLiteral("...per this many crank revolutions."));
    gearLayout->addWidget(m_gearNum, 1);
    gearLayout->addWidget(new QLabel(QStringLiteral(":"), gearRow));
    gearLayout->addWidget(m_gearDen, 1);
    m_rpm = MakeSpin(masterGroup, 0.01, 100000, 2, QStringLiteral(" rpm"));
    m_encoderReverse = new QCheckBox(QStringLiteral("Reverse encoder direction"), masterGroup);
    m_encoderReverse->setToolTip(QStringLiteral(
        "Count the encoder the other way. Tick it when the line (or crank) moving forward makes the board's\n"
        "encoder count (the Encoder row on the Board page) go down. Applied by the board at Upload."));
    m_lineMode = new QCheckBox(QStringLiteral("Cut-length mode (flying saw / shear)"), masterGroup);
    m_lineMode->setToolTip(QStringLiteral(
        "The encoder measures the line, or a driven crank's motor feeds it, and one crank revolution (360\u00b0 as\n"
        "drawn) is one cut of the design length.\n"
        "The cut length can be changed while running: the wait at the start of the cycle stretches, the\n"
        "accelerate / sync / decelerate / return part keeps its line distance."));
    m_wheel = MakeSpin(masterGroup, 0.001, 1e6, 3, QStringLiteral(" mm"));
    m_wheel->setToolTip(QStringLiteral("Line travel per encoder revolution (the measuring wheel's circumference)."));
    m_designLength = MakeSpin(masterGroup, 1, 1e6, 3, QStringLiteral(" mm"));
    m_designLength->setToolTip(QStringLiteral("Line distance from cut to cut as drawn: 360\u00b0 of the profile."));
    m_kerf = MakeSpin(masterGroup, 0, 1000, 3, QStringLiteral(" mm"));
    m_kerf->setToolTip(QStringLiteral("Material the cut removes (saw blade, laser; 0 for a shear). A finished piece is "
                                      "the cycle less the kerf."));
    m_crankConnector = new QComboBox(masterGroup);
    for (int i = 0; i < kMotorConnectors; ++i) m_crankConnector->addItem(QStringLiteral("M-%1").arg(i));
    m_crankRamp = MakeSpin(masterGroup, 0.1, 1e6, 1, QStringLiteral(" rpm/s"));
    m_crankRamp->setToolTip(QStringLiteral("Crank acceleration and deceleration. While ramping, followers also see s\u2032\u00b7\u03b1."));
    m_crankMax = MakeSpin(masterGroup, 0.1, 1e6, 1, QStringLiteral(" rpm"));
    m_crankMax->setToolTip(QStringLiteral("Crank motor speed limit, measured at the crank."));
    m_masterInfo = new QLabel(masterGroup);
    m_masterInfo->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));
    m_masterInfo->setWordWrap(true);
    m_masterInfo->setTextFormat(Qt::RichText);
    m_countsLabel = new QLabel(masterGroup);
    m_speedLabel = new QLabel(masterGroup);
    mf->addRow(QStringLiteral("Source"), m_masterSource);
    mf->addRow(m_encoderReverse);
    m_crankReverse = new QCheckBox(QStringLiteral("Reverse motor direction"), masterGroup);
    m_crankReverse->setObjectName(QStringLiteral("ECamCrankReverse"));
    m_crankReverse->setToolTip(QStringLiteral(
        "The crank motor turns the other way. The crank angle and the diagrams stay as they are. Applied at Upload."));
    mf->addRow(m_crankReverse);
    mf->addRow(m_lineMode);
    mf->addRow(m_countsLabel, m_countsPerRev);
    mf->addRow(QStringLiteral("Gear ratio"), gearRow);
    mf->addRow(QStringLiteral("Wheel"), m_wheel);
    mf->addRow(QStringLiteral("Design length"), m_designLength);
    mf->addRow(QStringLiteral("Kerf"), m_kerf);
    mf2->addRow(m_speedLabel, m_rpm);
    mf2->addRow(QStringLiteral("Crank motor"), m_crankConnector);
    mf2->addRow(QStringLiteral("Ramp"), m_crankRamp);
    mf2->addRow(QStringLiteral("Motor max"), m_crankMax);
    mf2->addRow(m_masterInfo);

    connect(m_masterSource, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (i < 0) return;
        const auto src = static_cast<MasterSource>(m_masterSource->itemData(i).toInt());
        EditProject(QStringLiteral("Master source"), [&](CamProject &p) { p.master.source = src; });
    });
    connect(m_countsPerRev, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        EditProject(QStringLiteral("Crank ppr"), [&](CamProject &p) { p.master.crankPpr = v; }, kMergeCountsPerRev);
    });
    connect(m_gearNum, &QSpinBox::valueChanged, this, [this](int v) {
        EditProject(QStringLiteral("Crank gear ratio"), [&](CamProject &p) { p.master.crankGearNum = v; }, kMergeGearNum);
    });
    connect(m_gearDen, &QSpinBox::valueChanged, this, [this](int v) {
        EditProject(QStringLiteral("Crank gear ratio"), [&](CamProject &p) { p.master.crankGearDen = v; }, kMergeGearDen);
    });
    connect(m_rpm, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        // Cut-length mode shows the line speed (m/min); rpm is cuts per minute at the design length.
        EditProject(QStringLiteral("Master speed"), [&](CamProject &p) {
            p.master.rpm = p.master.lineMode ? v * 1000.0 / std::max(p.master.designLength, 1e-9) : v;
        }, kMergeRpm);
    });
    connect(m_crankReverse, &QCheckBox::toggled, this, [this](bool on) {
        EditProject(QStringLiteral("Crank motor direction"), [&](CamProject &p) { p.master.crankReverse = on; });
    });
    connect(m_encoderReverse, &QCheckBox::toggled, this, [this](bool on) {
        EditProject(QStringLiteral("Encoder direction"), [&](CamProject &p) { p.master.encoderReverse = on; });
    });
    connect(m_lineMode, &QCheckBox::toggled, this, [this](bool on) {
        EditProject(on ? QStringLiteral("Cut-length mode on") : QStringLiteral("Cut-length mode off"),
                    [&](CamProject &p) { p.master.lineMode = on; });
    });
    connect(m_wheel, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        EditProject(QStringLiteral("Wheel circumference"), [&](CamProject &p) { p.master.wheelCircumference = v; },
                    kMergeWheel);
    });
    connect(m_designLength, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        // Keep the line speed: the cut rate changes with the length.
        EditProject(QStringLiteral("Design length"), [&](CamProject &p) {
            const double scale = p.master.designLength / std::max(v, 1e-9);
            p.master.designLength = v;
            p.master.rpm *= scale;
            if (p.master.source == MasterSource::DrivenCrank) {  // these are line speeds too
                p.master.crankRampRpmPerSec *= scale;
                p.master.crankMaxRpm *= scale;
            }
        }, kMergeDesignLength);
    });
    connect(m_kerf, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        EditProject(QStringLiteral("Kerf"), [&](CamProject &p) { p.master.kerf = v; }, kMergeKerf);
    });
    connect(m_crankConnector, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (i < 0) return;
        EditProject(QStringLiteral("Crank motor connector"), [&](CamProject &p) { p.master.crankConnector = i; });
    });
    connect(m_crankRamp, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        EditProject(QStringLiteral("Crank ramp"), [&](CamProject &p) {
            p.master.crankRampRpmPerSec = p.master.lineMode ? v * 1000.0 / std::max(p.master.designLength, 1e-9) : v;
        }, kMergeCrankRamp);
    });
    connect(m_crankMax, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        EditProject(QStringLiteral("Crank motor max"), [&](CamProject &p) {
            p.master.crankMaxRpm = p.master.lineMode ? v * 1000.0 / std::max(p.master.designLength, 1e-9) : v;
        }, kMergeCrankMax);
    });

    // Timeline
    m_timelineGroup = new CollapsibleGroup(QStringLiteral("Timeline"), QStringLiteral("timeline"), panel);
    auto *tf = m_timelineForm = new QFormLayout(m_timelineGroup->body());
    tf->setContentsMargins(14, 18, 14, 12);
    tf->setSpacing(8);
    m_repeat = new QCheckBox(QStringLiteral("Repeat (cycle = longest axis)"), m_timelineGroup);
    m_repeat->setToolTip(QStringLiteral("Off: the timeline runs once per trigger. On: it repeats; shorter axes rest at "
                                        "their end until the next cycle."));
    m_override = MakeSpin(m_timelineGroup, 1, 1000, 1, QStringLiteral(" %"));
    m_override->setToolTip(QStringLiteral("Speed override: scales the timeline's time (and every follower's speed)."));
    m_triggerSource = new QComboBox(m_timelineGroup);
    m_triggerSource->addItem(QStringLiteral("Command"), static_cast<int>(TriggerSource::Command));
    m_triggerSource->addItem(QStringLiteral("Digital input"), static_cast<int>(TriggerSource::DigitalInput));
    m_triggerSource->addItem(QStringLiteral("PLC (EtherNet/IP TRIGGER bit)"), static_cast<int>(TriggerSource::EtherNetIP));
    m_triggerInput = new QComboBox(m_timelineGroup);
    for (const char *n : {"IO-0", "IO-1", "IO-2", "IO-3", "IO-4", "IO-5", "DI-6", "DI-7", "DI-8", "A-9", "A-10", "A-11", "A-12"}) {
        m_triggerInput->addItem(QString::fromLatin1(n));
    }
    m_triggerEdge = new QComboBox(m_timelineGroup);
    m_triggerEdge->addItems({QStringLiteral("Rising edge"), QStringLiteral("Falling edge")});
    m_triggerBit = new QSpinBox(m_timelineGroup);
    m_triggerBit->setRange(0, 4095);
    m_triggerBit->setKeyboardTracking(false);
    m_triggerBit->setToolTip(QStringLiteral("Bit in the EtherNet/IP output assembly (reserved for later)."));
    m_timelineInfo = new QLabel(m_timelineGroup);
    m_timelineInfo->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));
    m_timelineInfo->setWordWrap(true);
    m_timelineInfo->setTextFormat(Qt::RichText);
    tf->addRow(m_repeat);
    tf->addRow(QStringLiteral("Speed"), m_override);
    tf->addRow(QStringLiteral("Trigger"), m_triggerSource);
    tf->addRow(QStringLiteral("Input"), m_triggerInput);
    tf->addRow(QStringLiteral("Edge"), m_triggerEdge);
    tf->addRow(QStringLiteral("Bit"), m_triggerBit);
    tf->addRow(m_timelineInfo);
    BuildOutputsPanel(panel);
    BuildCcioPanel(panel);
    BuildInputsPanel(panel);
    BuildWaitsPanel(panel);

    connect(m_repeat, &QCheckBox::toggled, this, [this](bool on) {
        EditProject(QStringLiteral("Timeline repeat"), [&](CamProject &p) { p.master.timelineRepeat = on; });
    });
    connect(m_override, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        EditProject(QStringLiteral("Speed override"), [&](CamProject &p) { p.master.speedOverride = v / 100.0; },
                    kMergeOverride);
    });
    connect(m_triggerSource, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (i < 0) return;
        const auto src = static_cast<TriggerSource>(m_triggerSource->itemData(i).toInt());
        EditProject(QStringLiteral("Trigger source"), [&](CamProject &p) { p.master.trigger = src; });
    });
    connect(m_triggerInput, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (i < 0) return;
        EditProject(QStringLiteral("Trigger input"), [&](CamProject &p) { p.master.triggerInput = i; });
    });
    connect(m_triggerEdge, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (i < 0) return;
        EditProject(QStringLiteral("Trigger edge"), [&](CamProject &p) { p.master.triggerRising = i == 0; });
    });
    connect(m_triggerBit, &QSpinBox::valueChanged, this, [this](int v) {
        EditProject(QStringLiteral("Trigger bit"), [&](CamProject &p) { p.master.triggerBit = v; }, kMergeTriggerBit);
    });

    // Axis
    auto *axisGroup = m_axisGroup = new CollapsibleGroup(QStringLiteral("Axis"), QStringLiteral("axis"), panel);
    // Two columns, so the page fits the pane on a 1080p screen: what the axis is and how it draws, and its mechanics and limits.
    auto *axisOuter = new QVBoxLayout(axisGroup->body());
    axisOuter->setContentsMargins(14, 18, 14, 12);
    axisOuter->setSpacing(12);
    auto *axisCols = new QHBoxLayout();
    axisCols->setSpacing(28);
    axisOuter->addLayout(axisCols);
    auto *af = m_axisForm = new QFormLayout();
    auto *af2 = m_axisForm2 = new QFormLayout();
    af->setSpacing(8);
    af2->setSpacing(8);
    axisCols->addLayout(af, 1);
    axisCols->addLayout(af2, 1);
    auto *axisRow = new QHBoxLayout();
    axisRow->setSpacing(6);
    m_axisCombo = new QComboBox(axisGroup);
    auto *addAxis = new QPushButton(QStringLiteral("+"), axisGroup);
    addAxis->setToolTip(QStringLiteral("Add a cam axis"));
    addAxis->setFixedWidth(34);
    addAxis->setStyleSheet(QStringLiteral("QPushButton { padding: 0; font-size: 15px; }"));
    m_removeAxisButton = new QPushButton(QStringLiteral("−"), axisGroup);
    m_removeAxisButton->setToolTip(QStringLiteral("Remove this cam axis"));
    m_removeAxisButton->setFixedWidth(34);
    m_removeAxisButton->setStyleSheet(QStringLiteral("QPushButton { padding: 0; font-size: 15px; }"));
    axisRow->addWidget(m_axisCombo, 1);
    axisRow->addWidget(addAxis);
    axisRow->addWidget(m_removeAxisButton);
    af->addRow(QStringLiteral("Axis"), axisRow);

    m_axisName = new QLineEdit(axisGroup);
    af->addRow(QStringLiteral("Name"), m_axisName);
    m_axisConnector = new QComboBox(axisGroup);
    for (int i = 0; i < kMotorConnectors; ++i) m_axisConnector->addItem(QStringLiteral("M-%1").arg(i));
    af->addRow(QStringLiteral("Motor"), m_axisConnector);
    m_axisMaster = new QComboBox(axisGroup);
    m_axisMaster->addItem(QStringLiteral("Crank"), static_cast<int>(AxisMaster::Crank));
    m_axisMaster->addItem(QStringLiteral("Timeline"), static_cast<int>(AxisMaster::Timeline));
    m_axisMaster->setToolTip(QStringLiteral("Crank: follows the crank angle every revolution.\n"
                                            "Timeline: its own diagram against time from the shared t = 0."));
    af->addRow(QStringLiteral("Master"), m_axisMaster);
    m_axisType = new QComboBox(axisGroup);
    m_axisType->setObjectName(QStringLiteral("axisType"));
    m_axisType->addItem(QStringLiteral("Cam profile"));
    m_axisType->addItem(QStringLiteral("Electronic gear"));
    m_axisType->setToolTip(QStringLiteral("Cam profile: follows its diagram.\n"
                                          "Electronic gear: no diagram, it follows the crank at a fixed ratio, exactly."));
    af->addRow(QStringLiteral("Type"), m_axisType);
    m_axisGearRow = new QWidget(axisGroup);
    {
        auto *gl = new QHBoxLayout(m_axisGearRow);
        gl->setContentsMargins(0, 0, 0, 0);
        gl->setSpacing(4);
        m_axisGearNum = new QSpinBox(m_axisGearRow);
        m_axisGearNum->setObjectName(QStringLiteral("axisGearNum"));
        m_axisGearNum->setRange(-1000000, 1000000);
        m_axisGearNum->setToolTip(QStringLiteral("Travel, in this axis's units, for the crank revolutions on the right. "
                                                 "Negative runs the axis the other way."));
        m_axisGearDen = new QSpinBox(m_axisGearRow);
        m_axisGearDen->setObjectName(QStringLiteral("axisGearDen"));
        m_axisGearDen->setRange(1, 1000000);
        m_axisGearDen->setToolTip(QStringLiteral("Crank revolutions for the travel on the left."));
        m_axisGearHint = new QLabel(m_axisGearRow);
        gl->addWidget(m_axisGearNum, 1);
        gl->addWidget(new QLabel(QStringLiteral(":"), m_axisGearRow));
        gl->addWidget(m_axisGearDen, 1);
        gl->addWidget(m_axisGearHint);
    }
    af->addRow(QStringLiteral("Gear ratio"), m_axisGearRow);
    m_duration = MakeSpin(axisGroup, 0.001, 1e6, 3, QStringLiteral(" s"));
    m_duration->setToolTip(QStringLiteral("Length of this axis's diagram; changing it stretches the whole diagram.\n"
                                          "Dragging the last knot changes only the last segment."));
    af->addRow(QStringLiteral("Duration"), m_duration);
    m_units = new QComboBox(axisGroup);
    m_units->setEditable(true);
    m_units->addItems({QStringLiteral("mm"), QStringLiteral("in"), QStringLiteral("deg"), QStringLiteral("rev")});
    af->addRow(QStringLiteral("Units"), m_units);
    m_axisDrive = new QComboBox(axisGroup);
    m_axisDrive->addItem(QStringLiteral("Direct (counts / unit)"), static_cast<int>(AxisDrive::Direct));
    m_axisDrive->addItem(QStringLiteral("Ballscrew"), static_cast<int>(AxisDrive::Ballscrew));
    m_axisDrive->addItem(QStringLiteral("Belt drive"), static_cast<int>(AxisDrive::Belt));
    m_axisDrive->setToolTip(QStringLiteral("How the motor moves the load. Ballscrew and belt drive work out counts / unit."));
    af2->addRow(QStringLiteral("Drive"), m_axisDrive);
    m_motorPpr = MakeSpin(axisGroup, 1, 1e9, 0);
    m_motorPpr->setToolTip(QStringLiteral("Motor steps per motor revolution (the drive's input resolution)."));
    af2->addRow(QStringLiteral("Motor ppr"), m_motorPpr);
    m_screwPitch = MakeSpin(axisGroup, 0.0001, 1e6, 4);
    m_screwPitch->setToolTip(QStringLiteral("Lead of the screw: travel per screw revolution."));
    af2->addRow(QStringLiteral("Screw pitch"), m_screwPitch);
    m_pulleyPd = MakeSpin(axisGroup, 0.0001, 1e6, 4);
    m_pulleyPd->setToolTip(QStringLiteral("Pitch diameter of the drive pulley: travel per motor rev is π × PD."));
    af2->addRow(QStringLiteral("Pulley PD"), m_pulleyPd);
    m_countsPerUnit = MakeSpin(axisGroup, 1e-6, 1e9, 4);
    m_countsPerUnit->setToolTip(QStringLiteral("Motor step counts per unit of slave travel."));
    af2->addRow(QStringLiteral("Counts / unit"), m_countsPerUnit);
    m_velMax = MakeSpin(axisGroup, 0.0, 1e9, 3);
    m_velMax->setSpecialValueText(QStringLiteral("not set"));
    m_velMax->setToolTip(QStringLiteral("Follower velocity limit. Sets the fastest safe crank speed."));
    af2->addRow(QStringLiteral("Vel max"), m_velMax);
    m_accMax = MakeSpin(axisGroup, 0.0, 1e12, 1);
    m_accMax->setSpecialValueText(QStringLiteral("not set"));
    m_accMax->setToolTip(QStringLiteral("Follower acceleration limit, including the crank ramp in driven mode."));
    af2->addRow(QStringLiteral("Acc max"), m_accMax);
    // Closed and the motor direction share a row, so the page fits the pane with the Homing row.
    auto *checks = new QHBoxLayout();
    checks->setSpacing(18);
    m_closed = new QCheckBox(QStringLiteral("Closed"), axisGroup);
    m_axisReverse = new QCheckBox(QStringLiteral("Reverse motor"), axisGroup);
    checks->addWidget(m_closed);
    checks->addWidget(m_axisReverse);
    checks->addStretch(1);
    m_axisReverse->setObjectName(QStringLiteral("ECamAxisReverse"));
    m_axisReverse->setToolTip(QStringLiteral(
        "This axis's motor turns the other way. The diagram and positions stay as drawn, so positive is still positive "
        "travel of the axis. Applied at Upload."));
    af2->addRow(checks);
    m_baseRadius = MakeSpin(axisGroup, 0.001, 1e6, 3);
    m_baseRadius->setToolTip(QStringLiteral("Cam drawing only: base circle radius."));
    af->addRow(QStringLiteral("Base radius"), m_baseRadius);
    m_rollerRadius = MakeSpin(axisGroup, 0.0, 1e6, 3);
    m_rollerRadius->setToolTip(QStringLiteral("Cam drawing only: roller follower radius (0 = knife edge)."));
    af->addRow(QStringLiteral("Roller radius"), m_rollerRadius);
    // Homing: a row on the page (what is set), the settings in their own window (the page fits the pane without them).
    {
        auto *row = new QHBoxLayout();
        row->setSpacing(8);
        m_homeSummary = new QLabel(axisGroup);
        m_homeSummary->setObjectName(QStringLiteral("ECamHomeSummary"));
        auto *open = new QPushButton(QStringLiteral("Homing\u2026"), axisGroup);
        open->setObjectName(QStringLiteral("ECamHomeOpen"));
        // No taller than the spin boxes' rows: the page fits the pane at 1080p.
        open->setMaximumHeight(m_accMax->sizeHint().height());
        row->addWidget(m_homeSummary, 1);
        row->addWidget(open);
        af2->addRow(QStringLiteral("Homing"), row);
        auto *dlg = m_homeDialog = new QDialog(this);
        dlg->setObjectName(QStringLiteral("ECamHomeDialog"));
        dlg->setStyleSheet(theme::DialogSheet());
        auto *dv = new QVBoxLayout(dlg);
        dv->setContentsMargins(16, 14, 16, 12);
        dv->setSpacing(12);
        auto *hc = new QHBoxLayout();
        hc->setSpacing(28);
        auto *hf = new QFormLayout();
        auto *hf2 = new QFormLayout();
        hf->setSpacing(8);
        hf2->setSpacing(8);
        hc->addLayout(hf, 1);
        hc->addLayout(hf2, 1);
        dv->addLayout(hc);
        auto *db = new QHBoxLayout();
        auto *help = new QPushButton(QStringLiteral("Help"), dlg);
        auto *close = new QPushButton(QStringLiteral("Close"), dlg);
        close->setDefault(true);
        db->addWidget(help);
        db->addStretch(1);
        db->addWidget(close);
        dv->addLayout(db);
        auto showHelp = [this]() { HelpWindow::Show(m_homeDialog, QStringLiteral("06-axes.html#homing")); };
        connect(help, &QPushButton::clicked, dlg, showHelp);
        auto *helpKey = new QAction(dlg);
        helpKey->setShortcut(QKeySequence(Qt::Key_F1));
        connect(helpKey, &QAction::triggered, dlg, showHelp);
        dlg->addAction(helpKey);
        connect(close, &QPushButton::clicked, dlg, &QDialog::accept);
        connect(open, &QPushButton::clicked, this, [this]() {
            RefreshAxisFields();
            m_homeDialog->exec();
        });
        m_homeSwitch = new QLabel(dlg);
        m_homeSwitch->setObjectName(QStringLiteral("ECamHomeSwitch"));
        hf->addRow(QStringLiteral("Home switch"), m_homeSwitch);
        m_homeDir = new QComboBox(dlg);
        m_homeDir->setObjectName(QStringLiteral("ECamHomeDir"));
        m_homeDir->addItem(QStringLiteral("− (negative)"));
        m_homeDir->addItem(QStringLiteral("+ (positive)"));
        m_homeDir->setToolTip(QStringLiteral("Which way the switch is from the axis's working travel."));
        hf->addRow(QStringLiteral("Toward"), m_homeDir);
        m_homeFast = MakeSpin(axisGroup, 0.001, 1e9, 3);
        m_homeFast->setObjectName(QStringLiteral("ECamHomeFast"));
        m_homeFast->setToolTip(QStringLiteral("The search for the switch, and the back-off."));
        hf->addRow(QStringLiteral("Search speed"), m_homeFast);
        m_homeSlow = MakeSpin(axisGroup, 0.001, 1e9, 3);
        m_homeSlow->setObjectName(QStringLiteral("ECamHomeSlow"));
        m_homeSlow->setToolTip(QStringLiteral("The last, slow move onto the switch, where its edge is caught. Slower repeats better."));
        hf->addRow(QStringLiteral("Latch speed"), m_homeSlow);
        m_homeBack = MakeSpin(axisGroup, 0.0, 1e9, 3);
        m_homeBack->setObjectName(QStringLiteral("ECamHomeBack"));
        m_homeBack->setToolTip(QStringLiteral("How far past the switch's release it backs off before the slow move."));
        hf->addRow(QStringLiteral("Back-off"), m_homeBack);
        m_homeDecel = MakeSpin(dlg, 0.0, 1e12, 1);
        m_homeDecel->setObjectName(QStringLiteral("ECamHomeDecel"));
        m_homeDecel->setSpecialValueText(QStringLiteral("as axis jog"));
        m_homeDecel->setToolTip(QStringLiteral("How hard it stops on reaching the switch. The run-on past the switch is speed\u00b2 / (2 \u00d7 decel)."));
        hf->addRow(QStringLiteral("Decel"), m_homeDecel);
        m_homeAt = MakeSpin(axisGroup, -1e9, 1e9, 3);
        m_homeAt->setObjectName(QStringLiteral("ECamHomeAt"));
        m_homeAt->setToolTip(QStringLiteral("The axis's position at the switch's edge, measured like the diagram (the profile's start is 0)."));
        hf2->addRow(QStringLiteral("Home position"), m_homeAt);
        m_homeSearch = MakeSpin(axisGroup, 0.0, 1e9, 3);
        m_homeSearch->setObjectName(QStringLiteral("ECamHomeSearch"));
        m_homeSearch->setSpecialValueText(QStringLiteral("until the switch"));
        m_homeSearch->setToolTip(QStringLiteral("The longest search: past it, homing fails. Set it a little over the axis's travel."));
        hf2->addRow(QStringLiteral("Search length"), m_homeSearch);
        m_homeThen = new QComboBox(dlg);
        m_homeThen->setObjectName(QStringLiteral("ECamHomeThen"));
        m_homeThen->addItem(QStringLiteral("Move to the cam's start"));
        m_homeThen->addItem(QStringLiteral("Stay at the switch"));
        hf2->addRow(QStringLiteral("Then"), m_homeThen);
        m_homeOrder = new QSpinBox(dlg);
        m_homeOrder->setObjectName(QStringLiteral("ECamHomeOrder"));
        m_homeOrder->setRange(1, kHomeOrderMax);
        m_homeOrder->setToolTip(QStringLiteral("Home all: lower numbers home first; axes with the same number home together."));
        hf2->addRow(QStringLiteral("Order"), m_homeOrder);
        m_homeRequired = new QCheckBox(QStringLiteral("Required before Start"), dlg);
        m_homeRequired->setObjectName(QStringLiteral("ECamHomeRequired"));
        m_homeRequired->setToolTip(QStringLiteral("The board refuses Start until this axis has homed since its motor was enabled."));
        hf2->addRow(m_homeRequired);
        connect(m_homeDir, &QComboBox::currentIndexChanged, this, [this](int i) {
            if (i < 0) return;
            EditAxis(QStringLiteral("Home direction"), [&](CamAxis &a) { a.home.dir = i == 1 ? 1 : -1; });
        });
        connect(m_homeFast, &QDoubleSpinBox::valueChanged, this, [this](double v) {
            EditAxis(QStringLiteral("Home search speed"), [&](CamAxis &a) { a.home.fast = v; }, kMergeHomeFast);
        });
        connect(m_homeSlow, &QDoubleSpinBox::valueChanged, this, [this](double v) {
            EditAxis(QStringLiteral("Home latch speed"), [&](CamAxis &a) { a.home.slow = v; }, kMergeHomeSlow);
        });
        connect(m_homeBack, &QDoubleSpinBox::valueChanged, this, [this](double v) {
            EditAxis(QStringLiteral("Home back-off"), [&](CamAxis &a) { a.home.back = v; }, kMergeHomeBack);
        });
        connect(m_homeAt, &QDoubleSpinBox::valueChanged, this, [this](double v) {
            EditAxis(QStringLiteral("Home position"), [&](CamAxis &a) { a.home.at = v; }, kMergeHomeAt);
        });
        connect(m_homeDecel, &QDoubleSpinBox::valueChanged, this, [this](double v) {
            EditAxis(QStringLiteral("Home decel"), [&](CamAxis &a) { a.home.decel = v; }, kMergeHomeDecel);
        });
        connect(m_homeSearch, &QDoubleSpinBox::valueChanged, this, [this](double v) {
            EditAxis(QStringLiteral("Home search length"), [&](CamAxis &a) { a.home.search = v; }, kMergeHomeSearch);
        });
        connect(m_homeThen, &QComboBox::currentIndexChanged, this, [this](int i) {
            if (i < 0) return;
            EditAxis(QStringLiteral("After homing"), [&](CamAxis &a) { a.home.thenStart = i == 0; });
        });
        connect(m_homeOrder, &QSpinBox::valueChanged, this, [this](int v) {
            EditAxis(QStringLiteral("Home order"), [&](CamAxis &a) { a.home.order = v; }, kMergeHomeOrder);
        });
        connect(m_homeRequired, &QCheckBox::toggled, this, [this](bool on) {
            EditAxis(QStringLiteral("Homing required"), [&](CamAxis &a) { a.home.required = on; });
        });
    }

    connect(m_axisCombo, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (m_updating || i < 0 || i == m_axis) return;
        m_axis = i;
        m_seg = 0;
        Refresh();
        SelectSegment(0);
    });
    connect(addAxis, &QPushButton::clicked, this, [this]() { AddAxis(false); });
    connect(m_removeAxisButton, &QPushButton::clicked, this, &MainWindow::RemoveAxis);
    connect(m_axisName, &QLineEdit::textEdited, this, [this](const QString &t) {
        EditAxis(QStringLiteral("Rename axis"), [&](CamAxis &a) { a.name = t.toStdString(); }, kMergeName);
    });
    connect(m_units, &QComboBox::currentTextChanged, this, [this](const QString &t) {
        if (t.trimmed().isEmpty()) return;
        EditAxis(QStringLiteral("Units"), [&](CamAxis &a) { a.units = t.trimmed().toStdString(); }, kMergeUnits);
    });
    connect(m_axisMaster, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (i < 0) return;
        const auto am = static_cast<AxisMaster>(m_axisMaster->itemData(i).toInt());
        const int idx = m_axis;
        EditProject(QStringLiteral("Axis master"), [&](CamProject &p) { SetAxisMaster(p, idx, am); });
    });
    connect(m_axisType, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (i < 0) return;
        // A gear can't be a timeline axis: back onto the crank first, keeping the shape for undo.
        const int idx = m_axis;
        EditProject(QStringLiteral("Axis type"), [&](CamProject &p) {
            if (i == 1) SetAxisMaster(p, idx, AxisMaster::Crank);
            p.axes[idx].gear = i == 1;
        });
    });
    connect(m_axisGearNum, &QSpinBox::valueChanged, this, [this](int v) {
        if (v == 0) return;  // no ratio: wait for the next digit
        EditAxis(QStringLiteral("Gear ratio"), [&](CamAxis &a) { a.gearNum = v; }, kMergeAxisGearNum);
    });
    connect(m_axisGearDen, &QSpinBox::valueChanged, this, [this](int v) {
        EditAxis(QStringLiteral("Gear ratio"), [&](CamAxis &a) { a.gearDen = v; }, kMergeAxisGearDen);
    });
    connect(m_duration, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        EditAxis(QStringLiteral("Axis duration"), [&](CamAxis &a) { ScaleSpan(a, v); }, kMergeDuration);
    });
    connect(m_axisConnector, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (i < 0) return;
        EditAxis(QStringLiteral("Axis motor connector"), [&](CamAxis &a) { a.connector = i; });
    });
    connect(m_velMax, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        EditAxis(QStringLiteral("Velocity limit"), [&](CamAxis &a) { a.velMax = v; }, kMergeVelMax);
    });
    connect(m_accMax, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        EditAxis(QStringLiteral("Acceleration limit"), [&](CamAxis &a) { a.accMax = v; }, kMergeAccMax);
    });
    connect(m_countsPerUnit, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        if (Axis().drive != AxisDrive::Direct) return;  // derived from the drive
        EditAxis(QStringLiteral("Counts / unit"), [&](CamAxis &a) { a.countsPerUnit = v; }, kMergeCountsPerUnit);
    });
    connect(m_axisDrive, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (i < 0) return;
        const auto d = static_cast<AxisDrive>(m_axisDrive->itemData(i).toInt());
        EditAxis(QStringLiteral("Axis drive"), [&](CamAxis &a) { a.drive = d; });
    });
    connect(m_motorPpr, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        EditAxis(QStringLiteral("Motor ppr"), [&](CamAxis &a) { a.motorPpr = v; }, kMergeMotorPpr);
    });
    connect(m_screwPitch, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        EditAxis(QStringLiteral("Screw pitch"), [&](CamAxis &a) { a.screwPitch = v; }, kMergeScrewPitch);
    });
    connect(m_pulleyPd, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        EditAxis(QStringLiteral("Pulley PD"), [&](CamAxis &a) { a.pulleyPd = v; }, kMergePulleyPd);
    });
    connect(m_axisReverse, &QCheckBox::toggled, this, [this](bool on) {
        EditAxis(QStringLiteral("Motor direction"), [&](CamAxis &a) { a.reverse = on; });
    });
    connect(m_closed, &QCheckBox::toggled, this, [this](bool on) {
        EditAxis(on ? QStringLiteral("Close profile") : QStringLiteral("Open profile"),
                 [&](CamAxis &a) { SetClosed(a, on); });
    });
    connect(m_baseRadius, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        EditAxis(QStringLiteral("Base radius"), [&](CamAxis &a) { a.baseRadius = v; }, kMergeBaseRadius);
    });
    connect(m_rollerRadius, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        EditAxis(QStringLiteral("Roller radius"), [&](CamAxis &a) { a.rollerRadius = v; }, kMergeRollerRadius);
    });
}

// Each panel is a page of the detail pane under the diagrams, in the order of the DetailPage enum.
QWidget *MainWindow::BuildDetailPane() {
    m_details = new QStackedWidget(this);
    m_details->setObjectName(QStringLiteral("ECamDetails"));
    for (CollapsibleGroup *g : {m_crankGroup, m_timelineGroup, m_axisGroup, m_segmentsGroup, m_outputsGroup, m_statsGroup, m_boardGroup, m_ccioGroup, m_inputsGroup, m_waitsGroup}) {
        g->setFoldable(false);
        auto *scroll = new QScrollArea(m_details);
        scroll->setWidgetResizable(true);
        scroll->setFrameShape(QFrame::NoFrame);
        scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        scroll->setStyleSheet(QStringLiteral("QScrollArea { background: transparent; }"
                                             "QScrollArea > QWidget > QWidget { background: transparent; }"));
        // The page takes the pane's width: fields shrink and text wraps instead of running under the edge. Not the
        // Board page: its buttons would go below their labels. It puts its settings under its controls when the pane
        // is narrow, and scrolls sideways when even that doesn't fit.
        if (g == m_boardGroup) {
            g->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
            scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
            scroll->viewport()->installEventFilter(new ResizeWatcher(scroll, [this, scroll]() { ReflowBoardPage(scroll->viewport()->width()); }));
        } else {
            g->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        }
        // A form reads badly stretched across a wide pane; the tables use all of it.
        if (g == m_crankGroup || g == m_axisGroup || g == m_timelineGroup || g == m_boardGroup) {
            g->setMaximumWidth(g == m_timelineGroup ? 640 : 1180);
            scroll->setAlignment(Qt::AlignLeft | Qt::AlignTop);
        }
        scroll->setWidget(g);
        m_details->addWidget(scroll);
    }
    m_details->setMinimumHeight(120);
    const int page = AppSettings().value("view/detailPage", kPageAxis).toInt();
    m_detailPage = page >= 0 && page < kPageCount ? page : kPageAxis;
    m_details->setCurrentIndex(m_detailPage);
    return m_details;
}

// Side by side needs both columns at their minimum, the gap and the margins; below that the settings go under the
// controls. A few pixels of hysteresis keep it from flipping back and forth at the edge.
void MainWindow::ReflowBoardPage(int width) {
    if (!m_boardCols || !m_boardControls || !m_boardSettings) return;
    const QMargins m = m_boardCols->contentsMargins();
    // The group's own frame around its body, whichever way the columns run.
    const int frame = m_boardGroup ? std::max(0, m_boardGroup->minimumSizeHint().width() - m_boardCols->minimumSize().width()) : 0;
    const int side = m_boardControls->minimumSize().width() + m_boardSettings->minimumSize().width() + m_boardCols->spacing() +
                     m.left() + m.right() + frame;
    const bool stacked = m_boardCols->direction() == QBoxLayout::TopToBottom;
    if (!stacked && width < side) {
        m_boardCols->setDirection(QBoxLayout::TopToBottom);
    } else if (stacked && width > side + 16) {
        m_boardCols->setDirection(QBoxLayout::LeftToRight);
    }
}

// The tree clears itself as it goes, which reports a new current item while the pages may already be gone.
MainWindow::~MainWindow() {
    if (m_tree) m_tree->blockSignals(true);
}

QWidget *MainWindow::BuildProjectPanel() {
    m_projectSplit = new QSplitter(Qt::Vertical, this);
    m_projectSplit->setObjectName(QStringLiteral("ECamProjectSplit"));
    m_projectSplit->setChildrenCollapsible(false);
    m_projectSplit->setHandleWidth(5);
    m_projectSplit->addWidget(BuildProjectTree());
    m_projectSplit->addWidget(BuildMachineStatus());
    m_projectSplit->setStretchFactor(0, 1);
    m_projectSplit->setStretchFactor(1, 0);
    m_projectSplit->setSizes({600, 1});  // the status at its own height, the tree the rest
    return m_projectSplit;
}

QWidget *MainWindow::BuildMachineStatus() {
    auto *panel = new QFrame(this);
    panel->setObjectName(QStringLiteral("ECamMachineStatus"));
    panel->setStyleSheet(QStringLiteral(
        "QFrame#ECamMachineStatus { background: #1f242b; border: 1px solid #1a1e24; border-radius: 2px; }"
        "QFrame#ECamMachineStatus QLabel { background: transparent; }"));
    auto *g = new QGridLayout(panel);
    g->setContentsMargins(10, 8, 10, 10);
    g->setHorizontalSpacing(8);
    g->setVerticalSpacing(5);
    auto muted = [panel](const QString &t) {
        auto *l = new QLabel(t, panel);
        l->setStyleSheet(QStringLiteral("QLabel { color: #8a929d; }"));
        return l;
    };
    auto *title = new QLabel(QStringLiteral("Status"), panel);
    title->setStyleSheet(QStringLiteral("QLabel { color: #d8dee6; font-weight: 600; }"));
    g->addWidget(title, 0, 0, 1, 3);
    m_statusBoard = new QLabel(panel);
    m_statusBoard->setObjectName(QStringLiteral("ECamStatusBoard"));
    m_statusEstop = new QLabel(panel);
    m_statusEstop->setObjectName(QStringLiteral("ECamStatusEstop"));
    g->addWidget(muted(QStringLiteral("Board")), 1, 0);
    g->addWidget(m_statusBoard, 1, 1, 1, 2);
    g->addWidget(muted(QStringLiteral("Estop")), 2, 0);
    g->addWidget(m_statusEstop, 2, 1, 1, 2);
    for (int c = 0; c < kMotorConnectors; ++c) {
        m_statusMotorName[c] = muted(QString());
        m_statusMotorName[c]->setObjectName(QStringLiteral("ECamStatusMotorName%1").arg(c));
        m_statusMotor[c] = new QLabel(panel);
        m_statusMotor[c]->setObjectName(QStringLiteral("ECamStatusMotor%1").arg(c));
        m_statusMotor[c]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        g->addWidget(muted(QStringLiteral("M-%1").arg(c)), 3 + c, 0);
        g->addWidget(m_statusMotorName[c], 3 + c, 1);
        g->addWidget(m_statusMotor[c], 3 + c, 2);
    }
    m_statusFault = new QLabel(panel);
    m_statusFault->setObjectName(QStringLiteral("ECamStatusFault"));
    m_statusFault->setWordWrap(true);
    m_statusFault->setStyleSheet(QStringLiteral("QLabel { color: #ff8a80; }"));
    g->addWidget(m_statusFault, 3 + kMotorConnectors, 0, 1, 3);
    g->setColumnStretch(1, 1);
    // A name too long for the column is cut, not the column widened: the tree sets the dock's width.
    for (QLabel *l : m_statusMotorName) l->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    m_statusBoard->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    panel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    return panel;
}

void MainWindow::RefreshMachineStatus() {
    if (!m_statusBoard || !m_board) return;
    const bool conn = m_board->isConnected();
    const BoardLink::State &s = m_board->state();
    const bool known = conn && s.valid;
    auto show = [](QLabel *l, const QString &text, const char *color) {
        l->setText(text);
        l->setStyleSheet(QStringLiteral("QLabel { color: %1; }").arg(QLatin1String(color)));
    };
    show(m_statusBoard, !conn ? QStringLiteral("Not connected") : !s.valid ? QStringLiteral("Connected") : s.state,
         !conn ? "#8a929d" : "#d8dee6");
    if (!known) {
        show(m_statusEstop, QStringLiteral("—"), "#8a929d");
    } else if (s.estop == QStringLiteral("ok")) {
        show(m_statusEstop, QStringLiteral("OK"), "#7fe3a0");
    } else if (s.estop == QStringLiteral("open")) {
        show(m_statusEstop, QStringLiteral("Open"), "#ff8a80");
    } else {
        show(m_statusEstop, QStringLiteral("Off"), "#ffcc80");
    }
    // Names from the open project when it is the one on the board.
    QString names[kMotorConnectors];
    if (known && s.profile != 0 && s.profile == ProjectHash()) {
        for (const CamAxis &a : m_project.axes) {
            if (a.connector >= 0 && a.connector < kMotorConnectors) names[a.connector] = QString::fromStdString(a.name);
        }
        if (m_project.master.source == MasterSource::DrivenCrank && m_project.master.crankConnector >= 0 &&
            m_project.master.crankConnector < kMotorConnectors) {
            names[m_project.master.crankConnector] = QStringLiteral("Crank");
        }
    }
    // Firmware before 0.8.9 reports only "every motor of the profile on" (enabled=).
    const bool perMotor = s.a9 >= 0;
    // A motor on a limit: the board reports by axis, so only with the open project on the board.
    int limitOf[kMotorConnectors] = {};
    // Homing, by axis too: 1 homed, -1 not (an axis with a home switch), 2 homing; 0 no switch.
    int homeOf[kMotorConnectors] = {};
    if (known && s.profile != 0 && s.profile == ProjectHash()) {
        for (size_t i = 0; i < m_project.axes.size() && i < 4; ++i) {
            const int c = m_project.axes[i].connector;
            if (c < 0 || c >= kMotorConnectors) continue;
            if ((s.limits >> (2 * i)) & 1u) limitOf[c] = 1;
            else if ((s.limits >> (2 * i + 1)) & 1u) limitOf[c] = -1;
            if (s.homed >= 0 && ((s.hasHome >> i) & 1)) {
                homeOf[c] = ((s.homing >> i) & 1) ? 2 : ((s.homed >> i) & 1) ? 1 : -1;
            }
        }
    }
    for (int c = 0; c < kMotorConnectors; ++c) {
        m_statusMotorName[c]->setText(names[c]);
        m_statusMotorName[c]->setToolTip(names[c]);
        const bool on = perMotor ? ((s.motorsOn >> c) & 1u) != 0 : false;
        if (!known || !perMotor) {
            show(m_statusMotor[c], QStringLiteral("—"), "#8a929d");
        } else if ((s.motorFault >> c) & 1u) {
            show(m_statusMotor[c], QStringLiteral("Fault"), "#ff8a80");
        } else if (limitOf[c] != 0) {
            show(m_statusMotor[c], limitOf[c] > 0 ? QStringLiteral("Limit +") : QStringLiteral("Limit \u2212"), "#ffcc80");
        } else if (on && homeOf[c] == 2) {
            show(m_statusMotor[c], QStringLiteral("Homing"), "#9ecbff");
        } else if (on && homeOf[c] == -1) {
            show(m_statusMotor[c], QStringLiteral("Not homed"), "#ffcc80");
        } else if (on && ((s.motorHlfb >> c) & 1u)) {
            show(m_statusMotor[c], homeOf[c] == 1 ? QStringLiteral("Homed") : QStringLiteral("Enabled"), "#7fe3a0");
        } else if (on) {
            show(m_statusMotor[c], QStringLiteral("Enabled, no HLFB"), "#ffcc80");
        } else {
            show(m_statusMotor[c], QStringLiteral("Disabled"), "#8a929d");
        }
    }
    const bool fault = known && !s.fault.isEmpty() && s.fault != QStringLiteral("none");
    m_statusFault->setText(fault ? s.fault : QString());
    m_statusFault->setVisible(fault);
}

QWidget *MainWindow::BuildProjectTree() {
    m_tree = new QTreeWidget(this);
    m_tree->setObjectName(QStringLiteral("ECamProjectTree"));
    m_tree->setHeaderHidden(true);
    m_tree->setIndentation(14);
    m_tree->setMinimumWidth(190);
    m_tree->setContextMenuPolicy(Qt::CustomContextMenu);
    m_tree->setStyleSheet(QStringLiteral(
        "QTreeWidget { background: #1f242b; color: #d8dee6; border: 1px solid #1a1e24; border-radius: 2px; }"
        "QTreeWidget::item { padding: 3px 2px; }"
        "QTreeWidget::item:selected { background: #2e4a70; color: #f0f4f8; }"));
    // Headings only fold; the rest carry the page they open, and an axis's its index.
    auto heading = [](QTreeWidgetItem *parent, const QString &text) {
        auto *it = new QTreeWidgetItem(parent, {text});
        it->setFlags(Qt::ItemIsEnabled);
        return it;
    };
    auto node = [](QTreeWidgetItem *parent, const QString &text, int page) {
        auto *it = new QTreeWidgetItem(parent, {text});
        it->setData(0, kRolePage, page);
        it->setData(0, kRoleAxis, -1);
        return it;
    };
    m_treeRoot = new QTreeWidgetItem(m_tree, {QStringLiteral("Untitled")});
    m_treeRoot->setFlags(Qt::ItemIsEnabled);
    m_treeMaster = heading(m_treeRoot, QStringLiteral("Master"));
    m_treeCrank = node(m_treeMaster, QStringLiteral("Crank"), kPageCrank);
    m_treeTimeline = node(m_treeMaster, QStringLiteral("Timeline"), kPageTimeline);
    m_treeWaits = node(m_treeTimeline, QStringLiteral("Waits"), kPageWaits);
    m_treeAxes = heading(m_treeRoot, QStringLiteral("Axes"));
    m_treeOutputs = node(m_treeRoot, QStringLiteral("Outputs"), kPageOutputs);
    m_treeInputs = node(m_treeRoot, QStringLiteral("Inputs"), kPageInputs);
    m_treeCcio = node(m_treeRoot, QStringLiteral("CCIO-8"), kPageCcio);
    m_treeAnalysis = node(m_treeRoot, QStringLiteral("Analysis"), kPageAnalysis);
    m_treeBoard = node(m_treeRoot, QStringLiteral("Board"), kPageBoard);
    m_tree->expandAll();
    m_tree->setIconSize(QSize(14, 14));
    m_markTimer = new QTimer(this);
    m_markTimer->setSingleShot(true);
    m_markTimer->setInterval(250);
    connect(m_markTimer, &QTimer::timeout, this, &MainWindow::RefreshTreeMarks);

    connect(m_tree, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem *it) {
        if (!it || !it->data(0, kRolePage).isValid()) return;
        const int axis = it->data(0, kRoleAxis).toInt();
        if (it->data(0, kRoleCcioBoard).isValid()) m_ccioBoard = it->data(0, kRoleCcioBoard).toInt();
        ShowDetail(it->data(0, kRolePage).toInt());
        if (it->data(0, kRolePage).toInt() == kPageCcio) RefreshCcio();
        if (it->data(0, kRolePage).toInt() == kPageInputs) RefreshInputs();
        if (axis >= 0 && axis != m_axis) m_axisCombo->setCurrentIndex(axis);  // one path for switching axes
    });
    connect(m_tree, &QTreeWidget::customContextMenuRequested, this, [this](const QPoint &at) {
        QTreeWidgetItem *it = m_tree->itemAt(at);
        if (!it) return;
        const bool axisNode = it->data(0, kRoleAxis).toInt() >= 0 && it->data(0, kRolePage).isValid();
        if (it != m_treeAxes && !axisNode) return;
        if (axisNode) m_tree->setCurrentItem(it);
        m_axisMenu->exec(m_tree->viewport()->mapToGlobal(at));
    });
    return m_tree;
}

// Keeps the tree on what the project holds now. Axes are added and removed here; everything else is fixed.
void MainWindow::RefreshTree() {
    if (!m_tree) return;
    QSignalBlocker block(m_tree);
    m_treeRoot->setText(0, m_path.isEmpty() ? QStringLiteral("Untitled") : QFileInfo(m_path).completeBaseName());
    m_treeCrank->setText(0, m_crankGroup->heading());
    m_treeTimeline->setText(0, m_timelineGroup->heading());
    m_treeOutputs->setText(0, m_outputsGroup->heading());
    m_treeInputs->setText(0, m_inputsGroup->heading());
    m_treeWaits->setText(0, m_waitsGroup->heading());
    m_treeWaits->setHidden(!UsesTimeline(m_project) && m_project.waits.empty());
    const int axes = static_cast<int>(m_project.axes.size());
    while (m_treeAxes->childCount() > axes) delete m_treeAxes->takeChild(m_treeAxes->childCount() - 1);
    while (m_treeAxes->childCount() < axes) {
        const int i = m_treeAxes->childCount();
        auto *it = new QTreeWidgetItem(m_treeAxes);
        it->setData(0, kRolePage, kPageAxis);
        it->setData(0, kRoleAxis, i);
        auto *segs = new QTreeWidgetItem(it, {QStringLiteral("Segments")});
        segs->setData(0, kRolePage, kPageSegments);
        segs->setData(0, kRoleAxis, i);
        it->setExpanded(true);
    }
    for (int i = 0; i < axes; ++i) m_treeAxes->child(i)->setText(0, QString::fromStdString(m_project.axes[i].name));
    const int boards = m_project.ccio.boards;
    while (m_treeCcio->childCount() > boards) delete m_treeCcio->takeChild(m_treeCcio->childCount() - 1);
    while (m_treeCcio->childCount() < boards) {
        const int i = m_treeCcio->childCount();
        auto *it = new QTreeWidgetItem(m_treeCcio, {QStringLiteral("Board %1").arg(i + 1)});
        it->setData(0, kRolePage, kPageCcio);
        it->setData(0, kRoleAxis, -1);
        it->setData(0, kRoleCcioBoard, i);
        m_treeCcio->setExpanded(true);
    }
    m_ccioBoard = std::clamp(m_ccioBoard, 0, std::max(boards - 1, 0));

    QTreeWidgetItem *axis = m_treeAxes->child(std::clamp(m_axis, 0, std::max(axes - 1, 0)));
    QTreeWidgetItem *cur = m_treeCrank;
    switch (m_detailPage) {
        case kPageTimeline: cur = m_treeTimeline; break;
        case kPageAxis: cur = axis; break;
        case kPageSegments: cur = axis ? axis->child(0) : nullptr; break;
        case kPageOutputs: cur = m_treeOutputs; break;
        case kPageAnalysis: cur = m_treeAnalysis; break;
        case kPageBoard: cur = m_treeBoard; break;
        case kPageCcio: cur = boards > 0 ? m_treeCcio->child(m_ccioBoard) : m_treeCcio; break;
        case kPageInputs: cur = m_treeInputs; break;
        case kPageWaits: cur = m_treeWaits; break;
        default: break;
    }
    if (cur) m_tree->setCurrentItem(cur);
    m_markTimer->start();
}

// The review's checks, on the nodes they belong to: a red triangle for an error, amber for a warning, and the text
// in the node's tooltip. A heading and the project show the worst thing under them, so folding one hides nothing.
void MainWindow::RefreshTreeMarks() {
    enum Mark { kNone, kWarning, kError };
    auto icon = [this](int mark) {
        const qreal dpr = m_tree->devicePixelRatioF();
        QPixmap pm(QSize(14, 14) * dpr);
        pm.setDevicePixelRatio(dpr);
        pm.fill(Qt::transparent);
        if (mark != kNone) {
            QPainter p(&pm);
            p.setRenderHint(QPainter::Antialiasing);
            p.setPen(Qt::NoPen);
            p.setBrush(QColor(mark == kError ? "#ff6b6b" : "#ffb347"));
            p.drawPolygon(QPolygonF({QPointF(7, 1), QPointF(13.5, 13), QPointF(0.5, 13)}));
            p.setPen(QPen(QColor("#1f242b"), 1.6));
            p.drawLine(QPointF(7, 5), QPointF(7, 9));
            p.drawPoint(QPointF(7, 11));
        }
        return QIcon(pm);
    };
    const review::ProjectInfo info = review::AnalyzeProject(m_project, m_estopPinKnown, m_opPinsKnown);
    std::map<QTreeWidgetItem *, int> marks;
    std::map<QTreeWidgetItem *, QStringList> texts;
    auto add = [&](QTreeWidgetItem *it, const review::Check &c) {
        const int mark = c.level == review::Check::Error ? kError : kWarning;
        texts[it] << QStringLiteral("%1 %2").arg(mark == kError ? QStringLiteral("Error:") : QStringLiteral("Warning:"), c.text);
        for (QTreeWidgetItem *up = it; up; up = up->parent()) marks[up] = std::max(marks[up], mark);
    };
    for (const review::Check &c : info.checks) {
        if (c.where == QStringLiteral("Project")) {
            add(m_treeRoot, c);
        } else if (c.where == QStringLiteral("Master")) {
            add(UsesCrank(m_project) ? m_treeCrank : m_treeTimeline, c);
        } else if (c.where == QStringLiteral("Inputs")) {
            add(m_treeInputs, c);
        } else if (c.where == QStringLiteral("Waits")) {
            add(m_treeWaits, c);
        } else {
            for (int i = 0; i < m_treeAxes->childCount(); ++i) {
                if (m_treeAxes->child(i)->text(0) == c.where) add(m_treeAxes->child(i), c);
            }
        }
    }
    // Every node has an icon, blank or not, so the names stay lined up.
    const QIcon none = icon(kNone), warning = icon(kWarning), error = icon(kError);
    for (QTreeWidgetItemIterator it(m_tree); *it; ++it) {
        const auto mark = marks.find(*it);
        const int m = mark == marks.end() ? kNone : mark->second;
        (*it)->setIcon(0, m == kError ? error : m == kWarning ? warning : none);
        (*it)->setData(0, Qt::UserRole + 2, m);
        const auto text = texts.find(*it);
        (*it)->setToolTip(0, text == texts.end() ? QString() : text->second.join(QLatin1Char('\n')));
    }
    if (info.errors() + info.warnings() > 0) {
        QStringList head;
        if (info.errors()) head << QStringLiteral("%1 error%2").arg(info.errors()).arg(info.errors() == 1 ? "" : "s");
        if (info.warnings()) head << QStringLiteral("%1 warning%2").arg(info.warnings()).arg(info.warnings() == 1 ? "" : "s");
        const QString own = m_treeRoot->toolTip(0);
        m_treeRoot->setToolTip(0, head.join(QStringLiteral(", ")) + (own.isEmpty() ? QString() : QStringLiteral("\n") + own));
    }
}

void MainWindow::ShowDetail(int page) {
    m_detailPage = page;
    m_details->setCurrentIndex(page);
    AppSettings().setValue("view/detailPage", page);
    RefreshTree();
}

void MainWindow::BuildSegmentPanel() {
    BuildBoardPanel();
    auto *group = m_segmentsGroup = new CollapsibleGroup(QStringLiteral("Segments"), QStringLiteral("segments"), this);
    auto *v = new QVBoxLayout(group->body());
    v->setContentsMargins(10, 16, 10, 10);
    v->setSpacing(8);

    m_table = new QTableWidget(group);
    m_table->setObjectName(QStringLiteral("ECamSegments"));
    m_table->setColumnCount(kColCount);
    m_table->setHorizontalHeaderLabels({QStringLiteral("#"), QStringLiteral("Law"), QStringLiteral("Start °"),
                                        QStringLiteral("End °"), QStringLiteral("Span °"), QStringLiteral("Start"),
                                        QStringLiteral("End"), QStringLiteral("Rise")});
    m_table->verticalHeader()->setVisible(false);
    m_table->verticalHeader()->setDefaultSectionSize(28);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setAlternatingRowColors(true);
    m_table->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed |
                             QAbstractItemView::AnyKeyPressed);
    auto *hh = m_table->horizontalHeader();
    hh->setSectionResizeMode(QHeaderView::Stretch);
    hh->setSectionResizeMode(kColIndex, QHeaderView::Fixed);
    hh->setSectionResizeMode(kColLaw, QHeaderView::Fixed);
    m_table->setColumnWidth(kColIndex, 30);
    m_table->setColumnWidth(kColLaw, 230);
    m_table->setMinimumHeight(150);
    v->addWidget(m_table, 1);

    auto *row = new QHBoxLayout();
    row->setSpacing(6);
    m_splitButton = new QPushButton(QStringLiteral("Split"), group);
    m_splitButton->setToolTip(QStringLiteral("Split the selected segment at its midpoint (or double-click the diagram)."));
    m_deleteButton = new QPushButton(QStringLiteral("Delete"), group);
    m_deleteButton->setToolTip(QStringLiteral("Delete the selected segment; the previous segment takes its span."));
    auto *hint = m_tableHint = new QLabel(group);
    hint->setStyleSheet(QStringLiteral("QLabel { color: #8a929d; }"));
    row->addWidget(m_splitButton);
    row->addWidget(m_deleteButton);
    row->addStretch(1);
    row->addWidget(hint);
    v->addLayout(row);

    connect(m_table, &QTableWidget::cellChanged, this, &MainWindow::OnTableItemChanged);
    connect(m_table, &QTableWidget::currentCellChanged, this, [this](int r, int, int, int) {
        if (!m_updating && r >= 0) SelectSegment(r);
    });
    connect(m_splitButton, &QPushButton::clicked, this, [this]() {
        const double mid = 0.5 * (SegmentStartDeg(Axis(), m_seg) + Axis().segments[m_seg].endDeg);
        int newSeg = -1;
        EditAxis(QStringLiteral("Split segment"), [&](CamAxis &a) { newSeg = SplitAt(a, std::round(mid * 2) / 2); });
        if (newSeg >= 0) SelectSegment(newSeg);
    });
    connect(m_deleteButton, &QPushButton::clicked, this, [this]() {
        const int seg = m_seg;
        EditAxis(QStringLiteral("Delete segment"), [&](CamAxis &a) { DeleteSegment(a, seg); });
        SelectSegment(std::max(0, seg - 1));
    });

    // Analysis (scrolls when the issue list is long, so the table keeps its rows)
    auto *statsGroup = m_statsGroup = new CollapsibleGroup(QStringLiteral("Analysis"), QStringLiteral("analysis"), this);
    auto *sl = new QVBoxLayout(statsGroup->body());
    sl->setContentsMargins(10, 16, 10, 10);
    auto *scroll = new QScrollArea(statsGroup);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setStyleSheet(QStringLiteral("QScrollArea { background: #1f242b; border: 1px solid #20242a; border-radius: 2px; }"
                                         "QScrollArea > QWidget > QWidget { background: #1f242b; }"));
    m_stats = new QLabel(scroll);
    m_stats->setObjectName(QStringLiteral("ECamStats"));
    m_stats->setStyleSheet(QStringLiteral("QLabel { border: none; }"));
    m_stats->setTextFormat(Qt::RichText);
    m_stats->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    m_stats->setWordWrap(true);
    m_stats->setTextInteractionFlags(Qt::TextSelectableByMouse);
    scroll->setWidget(m_stats);
    sl->addWidget(scroll);
}

// Under everything: is the board there, what is it doing, and the buttons a machine needs whatever else is open.
QWidget *MainWindow::BuildStatusStrip() {
    auto *strip = new QWidget(this);
    strip->setObjectName(QStringLiteral("ECamBoardStrip"));
    strip->setStyleSheet(QStringLiteral("#ECamBoardStrip { background: #1f242b; border: 1px solid #1a1e24; border-radius: 2px; }"));
    auto *row = new QHBoxLayout(strip);
    row->setContentsMargins(10, 6, 10, 6);
    row->setSpacing(6);
    m_boardLed = new QLabel(QStringLiteral("\u25cf"), strip);
    m_boardLed->setObjectName(QStringLiteral("ECamBoardLed"));
    row->addWidget(m_boardLed);
    m_boardStatus->setParent(strip);
    m_boardStatus->setMinimumHeight(fontMetrics().lineSpacing() * 2);
    row->addWidget(m_boardStatus, 1);
    for (QPushButton *b : {m_boardConnect, m_boardUpload, m_boardEnable, m_boardDisable, m_boardClearFault, m_boardHome, m_boardStart,
                           m_boardTrigger, m_boardStop, m_boardAbort}) {
        b->setParent(strip);
        b->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        b->setMinimumWidth(78);
        row->addWidget(b);
    }
    return strip;
}

void MainWindow::BuildBoardPanel() {
    m_board = new BoardLink(this);
    auto *group = m_boardGroup = new CollapsibleGroup(QStringLiteral("Board"), QStringLiteral("board"), this);
    // One row per job, buttons in shared columns, checkboxes right-aligned in their own column. Connect, Upload,
    // Enable, Disable, Start, Trigger, Stop and Abort are made here but live in the status strip.
    //   Connection  [transport] [address ............] [Discover]
    //   Profile     [Download] [Forget]                          [ ] Simulate
    //   Timeline    [ ] Trigger on Start                          (timeline axes only)
    //               armed: waiting for ...
    //   Hand        [Hand jog] [Crank -> 0]                     [ ] Follow board
    auto *boardCols = new QBoxLayout(QBoxLayout::LeftToRight, group->body());
    m_boardCols = boardCols;
    boardCols->setContentsMargins(10, 16, 10, 10);
    boardCols->setSpacing(20);
    auto *grid = new QGridLayout();
    m_boardControls = grid;
    boardCols->addLayout(grid, 1);
    grid->setHorizontalSpacing(6);
    grid->setVerticalSpacing(6);
    enum { kColLabel = 0, kColA, kColB, kColC, kColGap, kColCheck };
    auto rowLabel = [group](const QString &text) {
        auto *l = new QLabel(text, group);
        l->setStyleSheet(QStringLiteral("QLabel { color: #8a929d; }"));
        return l;
    };

    auto *row1 = new QHBoxLayout();
    row1->setSpacing(6);
    m_boardTransport = new QComboBox(group);
    m_boardTransport->addItems({QStringLiteral("Ethernet"), QStringLiteral("USB")});
    m_boardTransport->setCurrentIndex(AppSettings().value("board/transport", 0).toInt());
    m_boardTarget = new QComboBox(group);
    m_boardTarget->setEditable(true);
    m_boardTarget->setMinimumWidth(150);
    m_boardDiscover = new QPushButton(QStringLiteral("Discover"), group);
    m_boardConnect = new QPushButton(QStringLiteral("Connect"), group);
    m_boardConnect->setProperty("role", "primary");
    row1->addWidget(m_boardTransport);
    row1->addWidget(m_boardTarget, 1);
    row1->addWidget(m_boardDiscover);
    grid->addWidget(rowLabel(QStringLiteral("Connection")), 0, kColLabel);
    grid->addLayout(row1, 0, kColA, 1, kColCheck - kColA + 1);

    m_boardUpload = new QPushButton(QStringLiteral("Upload"), group);
    m_boardUpload->setToolTip(QStringLiteral(
        "Load this project onto the board (the board refuses while moving).\n"
        "The board keeps it in flash and loads it again at power-up, motors disabled."));
    m_boardDownload = new QPushButton(QStringLiteral("Download"), group);
    m_boardDownload->setToolTip(QStringLiteral("Open the board's profile as a new, unsaved project."));
    m_boardForget = new QPushButton(QStringLiteral("Forget"), group);
    m_boardEnable = new QPushButton(QStringLiteral("Enable"), group);
    m_boardEnable->setToolTip(QStringLiteral("Enable the motors the loaded profile uses."));
    m_boardDisable = new QPushButton(QStringLiteral("Disable"), group);
    m_boardHome = new QPushButton(QStringLiteral("Home"), group);
    m_boardHome->setObjectName(QStringLiteral("ECamBoardHome"));
    m_boardHome->setToolTip(QStringLiteral("Home every axis with a home switch, in their order (Stop stops it)."));
    m_boardClearFault = new QPushButton(QStringLiteral("Clear fault"), group);
    m_boardClearFault->setToolTip(QStringLiteral("Clear the latched fault without enabling the motors (e.g. to simulate)."));
    m_boardSim = new QCheckBox(QStringLiteral("Simulate"), group);
    m_boardSim->setToolTip(QStringLiteral("Run the profile on the board without stepping the motors."));
    m_boardFollow = new QCheckBox(QStringLiteral("Follow board"), group);
    m_boardFollow->setToolTip(QStringLiteral("Move the cam / timing cursor with the board's position."));
    m_boardFollow->setChecked(AppSettings().value("board/follow", true).toBool());
    // Encoder (encoder master): the live count, in revolutions / mm of line, and a zero for checking.
    m_boardEncLabel = rowLabel(QStringLiteral("Encoder"));
    m_boardEnc = new QLabel(group);
    m_boardEnc->setTextFormat(Qt::RichText);
    m_boardEnc->setToolTip(QStringLiteral(
        "The board's encoder count (DI-6 / DI-7, every edge counted), and the same in encoder revolutions\n"
        "(counts / rev from the Crank panel) or mm of line (cut-length mode). It should count up as the line\n"
        "or crank moves forward; otherwise tick Reverse encoder direction and Upload."));
    m_boardEncZero = new QPushButton(QStringLiteral("Zero"), group);
    m_boardEncZero->setToolTip(QStringLiteral("Show the count from here (in the app only; the board's count and the cam are not changed)."));
    grid->addWidget(m_boardEncLabel, 1, kColLabel);
    grid->addWidget(m_boardEnc, 1, kColA, 1, 2);
    grid->addWidget(m_boardEncZero, 1, kColC);
    connect(m_boardEncZero, &QPushButton::clicked, this, [this]() {
        m_encZero = m_board->state().encoder;
        RefreshBoard();
    });
    // Sim encoder (encoder master, simulate): the board advances a simulated count instead of the input.
    m_boardSimEncLabel = rowLabel(QStringLiteral("Sim encoder"));
    m_boardSimEncSpeed = MakeSpin(group, -1e6, 1e6, 2, QStringLiteral(" rpm"));
    m_boardSimEncSpeed->setToolTip(QStringLiteral("Speed of the simulated encoder (negative runs it backwards)."));
    m_boardSimEncRun = new QPushButton(QStringLiteral("Run"), group);
    m_boardSimEncRun->setCheckable(true);
    m_boardSimEncRun->setToolTip(QStringLiteral(
        "Simulate a moving line / crank: the board moves a simulated encoder count at this speed and the cam\n"
        "follows it as it would a real encoder (Start first). Stop holds the count. Simulate only."));
    m_boardSimEncState = new QLabel(group);
    m_boardSimEncState->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));
    grid->addWidget(m_boardSimEncLabel, 2, kColLabel);
    grid->addWidget(m_boardSimEncSpeed, 2, kColA);
    grid->addWidget(m_boardSimEncRun, 2, kColB);
    grid->addWidget(m_boardSimEncState, 2, kColC, 1, kColCheck - kColC + 1);
    auto sendSimEnc = [this]() {
        const double rate = m_boardSimEncRun->isChecked() ? SimEncoderCountsPerSecond(m_boardSimEncSpeed->value()) : 0.0;
        m_board->command(QStringLiteral("simenc %1").arg(rate, 0, 'f', 1), [this](bool ok, const QStringList &r) {
            if (!ok) {
                statusBar()->showMessage(QStringLiteral("simenc: %1").arg(r.isEmpty() ? QString() : r.last()), 6000);
                QSignalBlocker b(m_boardSimEncRun);
                m_boardSimEncRun->setChecked(false);
            }
            m_board->command(QStringLiteral("status"));
        });
        m_boardSimEncRun->setText(m_boardSimEncRun->isChecked() ? QStringLiteral("Stop") : QStringLiteral("Run"));
    };
    connect(m_boardSimEncRun, &QPushButton::toggled, this, sendSimEnc);
    connect(m_boardSimEncSpeed, &QDoubleSpinBox::valueChanged, this, [this, sendSimEnc](double) {
        if (m_boardSimEncRun->isChecked()) sendSimEnc();
    });
    grid->addWidget(rowLabel(QStringLiteral("Profile")), 3, kColLabel);
    grid->addWidget(m_boardDownload, 3, kColA);
    grid->addWidget(m_boardForget, 3, kColB);
    grid->addWidget(m_boardSim, 3, kColCheck);

    m_boardStart = new QPushButton(QStringLiteral("Start"), group);
    m_boardStart->setStyleSheet(QStringLiteral(
        "QPushButton { background: #1e4a28; color: #e8f8ec; border: 1px solid #3a9d55; font-weight: 700; }"
        "QPushButton:hover { background: #255a32; border-color: #4fc06e; }"
        "QPushButton:disabled { background: #2a332e; color: #7a9080; border-color: #3a4a3f; }"));
    m_boardStart->setToolTip(QStringLiteral("Crank axes engage (a driven crank ramps up); the timeline arms."));
    m_boardTrigger = new QPushButton(QStringLiteral("Trigger"), group);
    m_boardTrigger->setToolTip(QStringLiteral("Start the timeline now, instead of its input."));
    m_boardStop = new QPushButton(QStringLiteral("Stop"), group);
    m_boardStop->setToolTip(QStringLiteral("A timeline finishes its cycle, then stops (Stop again: at once). A crank ramps to rest; an "
                                           "encoder master lets go at once."));
    m_boardAbort = new QPushButton(QStringLiteral("Abort"), group);
    m_boardAbort->setStyleSheet(QStringLiteral(
        "QPushButton { background: #8e1d1d; color: #ffebee; border: 1px solid #e57373; font-weight: 800; }"
        "QPushButton:hover { background: #a82424; border-color: #ffcdd2; }"
        "QPushButton:disabled { background: #4a2a2a; color: #b09090; border-color: #5d3a3a; }"));
    m_boardTriggerOnStart = new QCheckBox(QStringLiteral("Trigger on Start"), group);
    m_boardTriggerOnStart->setToolTip(QStringLiteral("Start also fires the timeline, so testing needs no physical input."));
    m_boardTriggerOnStart->setChecked(AppSettings().value("board/triggerOnStart", true).toBool());

    // Cut length (cut-length mode): the finished piece from the next cut on, while running.
    m_boardCutLabel = rowLabel(QStringLiteral("Cut length"));
    m_boardCut = MakeSpin(group, 0.001, 1e6, 3, QStringLiteral(" mm"));
    m_boardCut->setToolTip(QStringLiteral("Finished piece length (the board adds the kerf). Takes effect with the next cut."));
    m_boardCut->setMinimumWidth(110);  // its range would ask for room for 1,000,000.000 mm; a length fits in this
    m_boardCutSet = new QPushButton(QStringLiteral("Set"), group);
    m_boardCutSet->setToolTip(QStringLiteral("Send the length to the board: it applies from the next cut on."));
    m_boardCutState = new QLabel(group);
    m_boardCutState->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));
    m_boardCutState->setWordWrap(true);
    grid->addWidget(m_boardCutLabel, 6, kColLabel);
    grid->addWidget(m_boardCut, 6, kColA);
    grid->addWidget(m_boardCutSet, 6, kColB);
    grid->addWidget(m_boardCutState, 6, kColC, 1, kColCheck - kColC + 1);
    connect(m_boardCutSet, &QPushButton::clicked, this, [this]() {
        const double mm = m_boardCut->value();
        m_board->command(QStringLiteral("length %1").arg(mm, 0, 'f', 3), [this, mm](bool ok, const QStringList &r) {
            statusBar()->showMessage(ok ? QStringLiteral("Board: cut length %1 mm from the next cut").arg(mm, 0, 'f', 3)
                                        : QStringLiteral("length: %1").arg(r.isEmpty() ? QString() : r.last()),
                                     6000);
            m_board->command(QStringLiteral("status"));
        });
    });

    // Timeline row (only when the project has timeline axes) and what the timeline is doing.
    m_boardTimelineLabel = rowLabel(QStringLiteral("Timeline"));
    m_boardTimelineState = new QLabel(group);
    m_boardTimelineState->setObjectName(QStringLiteral("ECamBoardTimelineState"));
    m_boardTimelineState->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));
    m_boardTimelineState->setWordWrap(true);
    grid->addWidget(m_boardTimelineLabel, 7, kColLabel);
    grid->addWidget(m_boardTriggerOnStart, 7, kColA, 1, 3);
    m_boardWaitSkip = new QPushButton(QStringLiteral("Skip wait"), group);
    m_boardWaitSkip->setObjectName(QStringLiteral("ECamWaitSkip"));
    m_boardWaitSkip->setToolTip(QStringLiteral("Simulate: pass the wait holding the timeline, as if its input had come."));
    m_boardWaitSkip->setVisible(false);
    grid->addWidget(m_boardWaitSkip, 7, kColCheck);
    connect(m_boardWaitSkip, &QPushButton::clicked, this, [this]() {
        m_board->command(QStringLiteral("wait_skip"), [this](bool ok, const QStringList &r) {
            if (!ok) statusBar()->showMessage(QStringLiteral("Skip wait: %1").arg(r.isEmpty() ? QString() : r.last()), 6000);
            m_board->command(QStringLiteral("status"));
        });
    });
    grid->addWidget(m_boardTimelineState, 8, kColA, 1, kColCheck - kColA + 1);

    m_boardJog = new QPushButton(QStringLiteral("Hand jog"), group);
    m_boardJog->setCheckable(true);
    m_boardJog->setToolTip(QStringLiteral(
        "Drive the motors by hand: drag the cam (or the wheel), or scrub the displacement / timing view,\n"
        "and the board's master goes there with every motor following its profile.\n"
        "Speed is limited to the profile's crank speed / timeline speed."));
    m_boardCrankZero = new QPushButton(QStringLiteral("Crank → 0°"), group);
    m_boardCrankZero->setToolTip(QStringLiteral("Turn the crank to 0° the short way (the motors follow their profiles)."));
    m_boardCrankZero->setVisible(false);  // only while hand jogging with the crank at rest (its cell stays)
    grid->addWidget(rowLabel(QStringLiteral("Hand")), 9, kColLabel);
    grid->addWidget(m_boardJog, 9, kColA);
    grid->addWidget(m_boardCrankZero, 9, kColB);
    m_boardAxisJog = new QPushButton(QStringLiteral("Axis jog…"), group);
    m_boardAxisJog->setObjectName(QStringLiteral("ECamAxisJog"));
    m_boardAxisJog->setToolTip(QStringLiteral(
        "Each axis on its own: enable its motor, jog it, step it, zero it, or move it back where the cam has it.\n"
        "Through the ClearCore, not the motors' USB. Not while the cam runs."));
    grid->addWidget(m_boardAxisJog, 9, kColC);
    connect(m_boardAxisJog, &QPushButton::clicked, this, &MainWindow::OpenAxisJog);
    grid->addWidget(m_boardFollow, 9, kColCheck);

    m_boardFirmware = new QLabel(group);
    m_boardFirmware->setObjectName(QStringLiteral("ECamFirmwareState"));
    m_boardFirmwareUpdate = new QPushButton(QStringLiteral("Update…"), group);
    m_boardFirmwareUpdate->setObjectName(QStringLiteral("ECamFwOpen"));
    m_boardFirmwareUpdate->setToolTip(QStringLiteral(
        "Put this app's firmware on a ClearCore over USB.\nThe board's profile is saved first and put back afterwards."));
    grid->addWidget(rowLabel(QStringLiteral("Firmware")), 10, kColLabel);
    grid->addWidget(m_boardFirmwareUpdate, 10, kColA);
    grid->addWidget(m_boardFirmware, 10, kColB, 1, kColCheck - kColB + 1);
    connect(m_boardCrankZero, &QPushButton::clicked, this, [this]() {
        if (!m_jogging) return;
        // The nearest whole turn from where the board's crank is: back if under 180 deg, else ahead.
        const BoardLink::State &s = m_board->state();
        const double abs = s.rev * 360.0 + s.deg;
        const double target = std::round(abs / 360.0) * 360.0;
        m_jogCrankDeg = target;
        SendJog(QStringLiteral("jog crank %1").arg(target, 0, 'f', 4));
        m_angle = 0.0;
        RefreshCursor();
        statusBar()->showMessage(QStringLiteral("Crank to 0°: %1° %2").arg(QString::number(std::fabs(target - abs), 'f', 1),
                                                                          target >= abs ? QStringLiteral("forward") : QStringLiteral("back")),
                                 4000);
    });
    m_boardStatus = new QLabel(group);
    m_boardStatus->setTextFormat(Qt::RichText);
    m_boardStatus->setWordWrap(true);
    m_boardStatus->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));

    // Live readouts must not resize the grid as their numbers change (the buttons would jump): they take
    // the width their cells give them instead of asking for their text's width.
    for (QLabel *l : {m_boardEnc, m_boardSimEncState, m_boardCutState, m_boardTimelineState, m_boardStatus}) {
        l->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        l->setMinimumWidth(0);
    }

    // Equal button columns; the gap column takes the spare width so the checkboxes sit right.
    for (int c : {kColA, kColB, kColC}) grid->setColumnMinimumWidth(c, 96);
    grid->setColumnStretch(kColGap, 1);
    grid->setRowStretch(11, 1);  // spare height goes under the rows, not between them
    for (QPushButton *b : {m_boardClearFault, m_boardEncZero, m_boardSimEncRun, m_boardUpload, m_boardDownload, m_boardForget, m_boardEnable, m_boardDisable, m_boardStart, m_boardStop, m_boardAbort, m_boardCutSet,
                           m_boardTrigger, m_boardJog, m_boardCrankZero, m_boardAxisJog}) {
        b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    RefreshBoardTargets();
    connect(m_boardTransport, &QComboBox::currentIndexChanged, this, [this](int) { RefreshBoardTargets(); });
    connect(m_boardDiscover, &QPushButton::clicked, this, [this]() {
        if (m_boardTransport->currentIndex() == 0) {
            m_board->discover();
            statusBar()->showMessage(QStringLiteral("Looking for boards..."), 2000);
        } else {
            RefreshBoardTargets();
        }
    });
    connect(m_board, &BoardLink::deviceDiscovered, this, [this](const QString &host, const QString &info) {
        if (m_boardTransport->currentIndex() != 0) return;
        if (m_boardTarget->findText(host) < 0) m_boardTarget->addItem(host);
        m_boardTarget->setCurrentText(host);
        statusBar()->showMessage(QStringLiteral("Found %1").arg(info), 5000);
    });
    connect(m_boardConnect, &QPushButton::clicked, this, &MainWindow::BoardConnect);
    connect(m_board, &BoardLink::connectedChanged, this, [this](bool on) {
        RefreshBoard();
        if (on) {
            LoadBoardNetwork();  // the estop setting too
            return;
        }
        m_estopLoaded = -1;
        m_estopPinLoaded = -1;
        m_estopPinLive = false;
        RefreshEstop();
        RefreshInputs();
        if (!m_netExpected) {
            m_netLoaded = net::Settings();
            m_netInUse->clear();
            RefreshNetwork();
        }
    });

    // Network settings. They change over USB only: it still reaches a board whatever its address, and an address
    // changed over Ethernet takes away the link it was changed on.
    auto *nf = new QFormLayout();
    m_boardSettings = nf;
    nf->setHorizontalSpacing(8);
    nf->setVerticalSpacing(7);  // with the Panel row it still fits the pane at 1080p
    boardCols->addLayout(nf, 1);
    m_netMode = new QComboBox(group);
    m_netMode->setObjectName(QStringLiteral("ECamNetMode"));
    m_netMode->addItems({QStringLiteral("DHCP"), QStringLiteral("Static")});
    auto field = [group](const char *name, const QString &tip) {
        auto *e = new QLineEdit(group);
        e->setObjectName(QLatin1String(name));
        e->setToolTip(tip);
        return e;
    };
    m_netIp = field("ECamNetIp", QStringLiteral("The board's address. On DHCP, the one it falls back to if no server answers."));
    m_netMask = field("ECamNetMask", QStringLiteral("Subnet mask, such as 255.255.255.0."));
    m_netGateway = field("ECamNetGateway", QStringLiteral("0.0.0.0 for none."));
    m_netHost = field("ECamNetHost", QStringLiteral("The EtherNet/IP host name. Empty for the default, ecam- and the last 3 MAC bytes."));
    m_netHost->setPlaceholderText(QStringLiteral("default"));
    m_netInUse = new QLabel(group);
    m_netInUse->setObjectName(QStringLiteral("ECamNetInUse"));
    m_netApply = new QPushButton(QStringLiteral("Apply and restart"), group);
    m_netApply->setObjectName(QStringLiteral("ECamNetApply"));
    m_netApply->setToolTip(QStringLiteral("Save these on the board and restart it, which is when they take effect."));
    m_netState = new QLabel(group);
    m_netState->setObjectName(QStringLiteral("ECamNetState"));
    m_netState->setWordWrap(true);
    m_netState->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));
    m_netState->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    m_netIpLabel = new QLabel(QStringLiteral("IP address"), group);
    // The address in use shares the first row, which keeps the page inside the pane on a 1080p screen.
    auto *netRow = new QHBoxLayout();
    netRow->setSpacing(10);
    netRow->addWidget(m_netMode, 1);
    netRow->addWidget(new QLabel(QStringLiteral("In use"), group));
    netRow->addWidget(m_netInUse, 1);
    m_estopMode = new QComboBox(group);
    m_estopMode->setObjectName(QStringLiteral("ECamEstopMode"));
    m_estopMode->addItems({QStringLiteral("Off"), QStringLiteral("Fault when low"), QStringLiteral("Fault when high")});
    m_estopMode->setToolTip(QStringLiteral("The board's estop (estop_mode). A normally closed estop holds its input high: fault when low."));
    m_estopPin = new QComboBox(group);
    m_estopPin->setObjectName(QStringLiteral("ECamEstopPin"));
    for (int i = 0; i < kTriggerInputs; ++i) m_estopPin->addItem(QString::fromStdString(BoardInputName(i)));
    m_estopPin->setToolTip(QStringLiteral("The estop's input on the board (estop_pin)."));
    m_estopState = new QLabel(group);
    m_estopState->setObjectName(QStringLiteral("ECamEstopState"));
    auto *estopRow = new QHBoxLayout();
    estopRow->setSpacing(8);
    estopRow->addWidget(m_estopMode, 1);
    estopRow->addWidget(m_estopPin);
    estopRow->addWidget(m_estopState, 1);
    nf->addRow(QStringLiteral("Estop"), estopRow);
    connect(m_estopMode, &QComboBox::activated, this, [this](int mode) { ApplyBoardEstop(mode); });
    {
        auto *opRow = new QHBoxLayout();
        opRow->setSpacing(8);
        m_opSummary = new QLabel(group);
        m_opSummary->setObjectName(QStringLiteral("ECamOpSummary"));
        m_opSummary->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        m_opButton = new QPushButton(QStringLiteral("Operator inputs\u2026"), group);
        m_opButton->setObjectName(QStringLiteral("ECamOpOpen"));
        m_opButton->setToolTip(QStringLiteral("A machine's own buttons: Enable, Disable, Fault reset, Start and Stop on the board's inputs."));
        opRow->addWidget(m_opSummary, 1);
        opRow->addWidget(m_opButton);
        nf->addRow(QStringLiteral("Panel"), opRow);
        connect(m_opButton, &QPushButton::clicked, this, &MainWindow::OpenOperatorInputs);
    }
    connect(m_estopPin, &QComboBox::activated, this, [this](int pin) { ApplyBoardEstopPin(pin); });
    nf->addRow(QStringLiteral("Network"), netRow);
    nf->addRow(m_netIpLabel, m_netIp);
    nf->addRow(QStringLiteral("Netmask"), m_netMask);
    nf->addRow(QStringLiteral("Gateway"), m_netGateway);
    nf->addRow(QStringLiteral("Host name"), m_netHost);
    nf->addRow(m_netApply, m_netState);
    connect(m_netMode, &QComboBox::currentIndexChanged, this, [this](int) { RefreshNetwork(); });
    for (QLineEdit *e : {m_netIp, m_netMask, m_netGateway, m_netHost}) {
        connect(e, &QLineEdit::textEdited, this, [this]() { RefreshNetwork(); });
        connect(e, &QLineEdit::textChanged, this, [this]() { RefreshNetwork(); });
    }
    connect(m_netApply, &QPushButton::clicked, this, [this]() { ApplyBoardNetwork(true); });
    // DHCP can take a while after a restart: keep the address in use current while the page is open.
    m_netPoll = new QTimer(this);
    m_netPoll->setInterval(3000);
    connect(m_netPoll, &QTimer::timeout, this, [this]() {
        if (!m_board->isConnected() || !m_boardGroup->isVisible()) return;
        m_board->command(QStringLiteral("eip"), [this](bool ok, const QStringList &lines) {
            if (ok) m_netInUse->setText(net::InUse(lines));
        });
    });
    m_netPoll->start();
    connect(m_board, &BoardLink::linkLost, this, [this](const QString &why) {
        statusBar()->showMessage(QStringLiteral("Board: %1").arg(why), 6000);
        RefreshBoard();
    });
    connect(m_board, &BoardLink::logLine, this, [this](const QString &line) {
        if (line.trimmed().startsWith(QStringLiteral("error:"))) statusBar()->showMessage(QStringLiteral("Board: ") + line.trimmed(), 6000);
    });
    connect(m_board, &BoardLink::stateUpdated, this, &MainWindow::RefreshCcioLive);
    connect(m_board, &BoardLink::stateUpdated, this, &MainWindow::RefreshInputsLive);
    connect(m_board, &BoardLink::connectedChanged, this, &MainWindow::RefreshInputsLive);
    connect(m_board, &BoardLink::connectedChanged, this, &MainWindow::RefreshCcioLive);
    connect(m_board, &BoardLink::stateUpdated, this, [this](const BoardLink::State &s) {
        // The board's estop input, from its status: remembered, and the pages that keep it free updated.
        if (s.valid && s.opKnown && !std::equal(s.opPins, s.opPins + 5, m_opPinsKnown)) {
            QStringList ops;
            for (int k = 0; k < 5; ++k) {
                m_opPinsKnown[k] = s.opPins[k];
                ops << QString::number(s.opPins[k]);
            }
            AppSettings().setValue("board/opPins", ops.join(QLatin1Char(',')));
            RefreshOutputs();
            RefreshInputs();
            RefreshWaits();
            RefreshEstop();
            m_markTimer->start();
        }
        if (s.valid && s.estopPin >= -1 && (s.estopPin != m_estopPinKnown || !m_estopPinLive)) {
            m_estopPinKnown = s.estopPin;
            m_estopPinLive = true;
            AppSettings().setValue("board/estopPin", m_estopPinKnown);
            RefreshOutputs();
            RefreshInputs();
            m_markTimer->start();
        }
        // Follow only while the board runs: when it is idle the cursor is the user's to drag, and
        // while jogging the cursor is the hand target the board is moving to.
        if (m_boardFollow->isChecked() && s.profile == ProjectHash() && !m_playButton->isChecked() &&
            s.state == QStringLiteral("running") && !m_jogging) {
            m_angle = s.deg;
            m_time = s.t;
            RefreshCursor();
        }
        m_strip->setLiveOutputs(s.outputs, m_board->isConnected() && s.profile == ProjectHash());
        // The board left hand jog (stop, abort, fault, disable).
        // (Not in the first moments: a status made just before the board took "jog on" still says ready.)
        if (m_jogging && s.state != QStringLiteral("jogging") && !m_jogInFlight && m_jogClock.elapsed() > 300) {
            SetJogging(false);
        }
        RefreshBoard();
    });

    auto simple = [this](QPushButton *b, const QString &cmd) {
        connect(b, &QPushButton::clicked, this, [this, cmd]() {
            m_board->command(cmd, [this, cmd](bool ok, const QStringList &r) {
                if (!ok) statusBar()->showMessage(QStringLiteral("%1: %2").arg(cmd, r.isEmpty() ? QString() : r.last()), 6000);
                m_board->command(QStringLiteral("status"));
            });
        });
    };
    simple(m_boardEnable, QStringLiteral("enable"));
    simple(m_boardDisable, QStringLiteral("disable"));
    simple(m_boardClearFault, QStringLiteral("clear"));
    simple(m_boardTrigger, QStringLiteral("trigger"));
    simple(m_boardStop, QStringLiteral("stop"));
    simple(m_boardAbort, QStringLiteral("abort"));
    connect(m_boardHome, &QPushButton::clicked, this, [this]() {
        m_board->command(QStringLiteral("axhome all"), [this](bool ok, const QStringList &r) {
            if (!ok) statusBar()->showMessage(QStringLiteral("Home: %1").arg(r.isEmpty() ? QString() : r.last()), 6000);
            m_board->command(QStringLiteral("status"));
        });
    });
    connect(m_boardStart, &QPushButton::clicked, this, [this]() {
        m_board->command(QStringLiteral("start"), [this](bool ok, const QStringList &r) {
            if (!ok) {
                const QString why = r.isEmpty() ? QString() : r.last();
                statusBar()->showMessage(QStringLiteral("start: %1").arg(why), 6000);
                if (why.contains(QLatin1String("off its cam position"))) OfferCamPositions();
                return;
            }
            if (m_boardTriggerOnStart->isChecked() && UsesTimeline(m_project)) {
                m_board->command(QStringLiteral("trigger"));
            }
            m_board->command(QStringLiteral("status"));
        });
    });
    connect(m_boardSim, &QCheckBox::clicked, this, [this](bool on) {
        m_board->command(QStringLiteral("sim %1").arg(on ? 1 : 0), [this](bool ok, const QStringList &r) {
            if (!ok) statusBar()->showMessage(QStringLiteral("sim: %1").arg(r.isEmpty() ? QString() : r.last()), 6000);
            m_board->command(QStringLiteral("status"));
        });
    });
    connect(m_boardUpload, &QPushButton::clicked, this, [this]() {
        m_boardBusy = true;
        RefreshBoard();
        statusBar()->showMessage(QStringLiteral("Uploading..."));
        m_board->uploadProfile(m_project, [this](bool ok, const QString &msg) {
            m_boardBusy = false;
            statusBar()->showMessage(ok ? QStringLiteral("Board: %1").arg(msg) : QStringLiteral("Upload failed: %1").arg(msg), 8000);
            m_board->command(QStringLiteral("status"));
            RefreshBoard();
        });
    });
    connect(m_boardDownload, &QPushButton::clicked, this, &MainWindow::BoardDownload);
    connect(m_boardForget, &QPushButton::clicked, this, [this]() {
        // Restart only with nothing moving (a restart drops the motors at once).
        const BoardLink::State &st = m_board->state();
        const bool idle = st.state != QStringLiteral("running") && st.state != QStringLiteral("jogging") &&
                          st.crank != QStringLiteral("running") && st.crank != QStringLiteral("stopping") &&
                          st.crank != QStringLiteral("following") && st.timeline != QStringLiteral("running") &&
                          st.timeline != QStringLiteral("stopping");
        QMessageBox box(QMessageBox::Warning, QStringLiteral("Forget"),
                        QStringLiteral("Erase the profile stored on the board?"), QMessageBox::NoButton, this);
        box.setInformativeText(
            idle ? QStringLiteral("This cannot be undone. The board is restarted so it comes up empty "
                                  "(the motors stay disabled); the app reconnects.\n\n"
                                  "To keep the profile, Download and save it first.")
                 : QStringLiteral("This cannot be undone. The board is moving, so it is not restarted: the profile "
                                  "keeps running until the next restart, then the board comes up empty.\n\n"
                                  "To keep the profile, Download and save it first."));
        QPushButton *yes = box.addButton(idle ? QStringLiteral("Forget and restart") : QStringLiteral("Forget"),
                                         QMessageBox::DestructiveRole);
        QPushButton *cancel = box.addButton(QMessageBox::Cancel);
        box.setDefaultButton(cancel);
        box.exec();
        if (box.clickedButton() != yes) return;
        m_board->command(QStringLiteral("forget"), [this, idle](bool ok, const QStringList &r) {
            if (!ok) {
                statusBar()->showMessage(QStringLiteral("forget: %1").arg(r.isEmpty() ? QString() : r.last()), 6000);
                m_board->command(QStringLiteral("status"));
                return;
            }
            if (!idle) {
                statusBar()->showMessage(QStringLiteral("Board: stored profile erased; it runs until the next restart."), 8000);
                m_board->command(QStringLiteral("status"));
                return;
            }
            RestartBoard(QStringLiteral("Board: stored profile erased, restarting..."));
        });
    });
    connect(m_boardFollow, &QCheckBox::toggled, this, [](bool on) { AppSettings().setValue("board/follow", on); });
    connect(m_boardFirmwareUpdate, &QPushButton::clicked, this, &MainWindow::OpenFirmwareUpdate);
    connect(m_boardJog, &QPushButton::clicked, this, [this](bool on) {
        if (!on) {
            m_board->command(QStringLiteral("jog off"));
            SetJogging(false);
            return;
        }
        m_board->command(QStringLiteral("jog on"), [this](bool ok, const QStringList &r) {
            if (!ok) {
                statusBar()->showMessage(QStringLiteral("jog: %1").arg(r.isEmpty() ? QString() : r.last()), 6000);
                SetJogging(false);
                return;
            }
            SetJogging(true);
            m_board->command(QStringLiteral("status"));
        });
    });
    connect(m_boardTriggerOnStart, &QCheckBox::toggled, this, [this](bool on) {
        AppSettings().setValue("board/triggerOnStart", on);
        RefreshBoard();  // the timeline line says what Start will do
    });
    RefreshBoard();
}

void MainWindow::RefreshBoardTargets() {
    const bool eth = m_boardTransport->currentIndex() == 0;
    m_boardTarget->clear();
    if (eth) {
        const QString last = AppSettings().value("board/host").toString();
        if (!last.isEmpty()) m_boardTarget->addItem(last);
        m_boardTarget->setToolTip(QStringLiteral("Board IP address (Discover finds it)."));
    } else {
        const QString last = AppSettings().value("board/port").toString();
        for (const QSerialPortInfo &p : QSerialPortInfo::availablePorts()) {
            const bool clearCore = fw::IsClearCore(p.vendorIdentifier(), p.productIdentifier());
            m_boardTarget->addItem(clearCore ? QStringLiteral("%1 (ClearCore)").arg(p.portName()) : p.portName(), p.portName());
            if (p.portName() == last || (last.isEmpty() && clearCore)) m_boardTarget->setCurrentIndex(m_boardTarget->count() - 1);
        }
        m_boardTarget->setToolTip(QStringLiteral("USB serial port of the ClearCore."));
    }
    m_boardDiscover->setText(eth ? QStringLiteral("Discover") : QStringLiteral("Refresh"));
}

void MainWindow::BoardConnect() {
    if (m_board->isConnected()) {
        m_board->disconnectLink();
        return;
    }
    QString err;
    bool ok;
    if (m_boardTransport->currentIndex() == 0) {
        const QString host = m_boardTarget->currentText().trimmed();
        ok = m_board->connectEthernet(host, &err);
        if (ok) AppSettings().setValue("board/host", host);
    } else {
        QString port = m_boardTarget->currentData().toString();
        if (port.isEmpty()) port = m_boardTarget->currentText().section(QLatin1Char(' '), 0, 0);
        ok = m_board->connectSerial(port, &err);
        if (ok) AppSettings().setValue("board/port", port);
    }
    AppSettings().setValue("board/transport", m_boardTransport->currentIndex());
    if (!ok) statusBar()->showMessage(QStringLiteral("Connect failed: %1").arg(err), 6000);
    RefreshBoard();
}

net::Settings MainWindow::NetFields() const {
    net::Settings s;
    s.valid = true;
    s.dhcp = m_netMode->currentIndex() == 0;
    s.ip = m_netIp->text().trimmed();
    s.netmask = m_netMask->text().trimmed();
    s.gateway = m_netGateway->text().trimmed();
    s.host = m_netHost->text().trimmed();
    return s;
}

// Reads what the board has into the fields. After a restart it also says whether the board kept what was sent.
void MainWindow::LoadBoardNetwork() {
    m_board->command(QStringLiteral("config"), [this](bool ok, const QStringList &lines) {
        if (!ok) return;
        for (const QString &l : lines) {
            const QStringList kv = l.trimmed().split(QLatin1Char(' '), Qt::SkipEmptyParts);
            if (kv.size() != 2) continue;
            if (kv[0] == QStringLiteral("estop_mode") || kv[0] == QStringLiteral("estop_a9")) m_estopLoaded = kv[1].toInt();
            if (kv[0] == QStringLiteral("estop_a9")) m_estopPinLoaded = kEstopInput;  // before the pin could be chosen
            if (kv[0] == QStringLiteral("estop_pin")) m_estopPinLoaded = m_estopPin->findText(kv[1]);
        }
        RefreshEstop();
        const net::Settings s = net::Parse(lines);
        if (!s.valid) return;
        m_netLoaded = s;
        {
            QSignalBlocker b(m_netMode);
            m_netMode->setCurrentIndex(s.dhcp ? 0 : 1);
        }
        m_netIp->setText(s.ip);
        m_netMask->setText(s.netmask);
        m_netGateway->setText(s.gateway);
        m_netHost->setText(s.host);
        RefreshNetwork();
        if (m_netExpected) {
            const bool same = *m_netExpected == s;
            m_netExpected.reset();
            m_netState->setText(same ? QStringLiteral("Applied.") : QStringLiteral("The board did not keep these settings: check them."));
            statusBar()->showMessage(same ? QStringLiteral("Board: network settings applied.")
                                          : QStringLiteral("Board: the network settings did not take."), 8000);
        }
    });
    m_board->command(QStringLiteral("eip"), [this](bool ok, const QStringList &lines) {
        if (ok) m_netInUse->setText(net::InUse(lines));
    });
}

void MainWindow::RefreshNetwork() {
    if (!m_netIp) return;
    const bool conn = m_board->isConnected();
    const bool usb = m_board->transport() == BoardLink::Transport::Serial;
    const BoardLink::State &s = m_board->state();
    const bool moving = s.state == QStringLiteral("running") || s.state == QStringLiteral("jogging");
    const bool plc = conn && s.valid && s.plcInControl();
    const bool motorsOn = s.enabled || s.motorsOn != 0;
    const bool editable = conn && usb && m_netLoaded.valid && !motorsOn && !moving && !plc && !m_boardBusy && !m_netExpected;
    for (QWidget *w : {static_cast<QWidget *>(m_netMode), static_cast<QWidget *>(m_netIp), static_cast<QWidget *>(m_netMask),
                       static_cast<QWidget *>(m_netGateway), static_cast<QWidget *>(m_netHost)}) {
        w->setEnabled(editable);
    }
    const net::Settings now = NetFields();
    const bool dirty = m_netLoaded.valid && now != m_netLoaded;
    const QString problem = dirty ? net::Check(now) : QString();
    m_netApply->setEnabled(editable && dirty && problem.isEmpty());
    m_netIpLabel->setText(now.dhcp ? QStringLiteral("Fallback IP") : QStringLiteral("IP address"));
    if (m_netExpected) return;  // the restart's outcome goes here
    QString why;
    if (!conn) {
        why = QStringLiteral("Connect over USB to change these.");
    } else if (!usb) {
        why = QStringLiteral("Read only over Ethernet: connect over USB to change these.");
    } else if (plc) {
        why = QStringLiteral("The PLC has control.");
    } else if (motorsOn || moving) {
        why = QStringLiteral("Motors enabled.");
    } else if (!problem.isEmpty()) {
        why = problem;
    } else if (dirty) {
        why = QStringLiteral("Takes effect when the board restarts.");
    }
    if (!why.isEmpty() || m_netState->text() != QStringLiteral("Applied.")) m_netState->setText(why);
}

void MainWindow::LoadBoardEstop() {
    m_board->command(QStringLiteral("config"), [this](bool ok, const QStringList &lines) {
        if (!ok) return;
        for (const QString &l : lines) {
            const QStringList kv = l.trimmed().split(QLatin1Char(' '), Qt::SkipEmptyParts);
            if (kv.size() != 2) continue;
            if (kv[0] == QStringLiteral("estop_mode") || kv[0] == QStringLiteral("estop_a9")) m_estopLoaded = kv[1].toInt();
            if (kv[0] == QStringLiteral("estop_a9")) m_estopPinLoaded = kEstopInput;
            if (kv[0] == QStringLiteral("estop_pin")) m_estopPinLoaded = m_estopPin->findText(kv[1]);
        }
        RefreshEstop();
    });
}

void MainWindow::ApplyBoardEstopPin(int pin) {
    if (pin == m_estopPinLoaded || pin < 0 || pin >= kTriggerInputs) return;
    m_board->command(QStringLiteral("set estop_pin %1").arg(QString::fromStdString(BoardInputName(pin))), [this](bool ok, const QStringList &r) {
        if (!ok) statusBar()->showMessage(QStringLiteral("Estop: %1").arg(r.isEmpty() ? QString() : r.last()), 8000);
        LoadBoardEstop();
        m_board->command(QStringLiteral("status"));
    });
}

// The board saves it to flash, so it waits for the motors to be disabled and nothing to move.
void MainWindow::ApplyBoardEstop(int mode) {
    if (mode == m_estopLoaded || mode < 0 || mode > 2) return;
    if (mode == 0 &&
        QMessageBox::question(this, QStringLiteral("Estop"),
                              QStringLiteral("Turn the estop off? The board will then ignore %1.").arg(m_estopPin->currentText())) != QMessageBox::Yes) {
        RefreshEstop();
        return;
    }
    m_board->command(QStringLiteral("set estop_mode %1").arg(mode), [this](bool ok, const QStringList &r) {
        if (!ok) statusBar()->showMessage(QStringLiteral("Estop: %1").arg(r.isEmpty() ? QString() : r.last()), 6000);
        LoadBoardEstop();
        m_board->command(QStringLiteral("status"));
    });
}

void MainWindow::RefreshEstop() {
    if (!m_estopMode) return;
    const bool conn = m_board->isConnected();
    const BoardLink::State &s = m_board->state();
    const bool moving = s.state == QStringLiteral("running") || s.state == QStringLiteral("jogging");
    const bool plc = conn && s.valid && s.plcInControl();
    const bool idle = !s.enabled && s.motorsOn == 0 && s.jogMoving == 0 && !moving;
    {
        QSignalBlocker b(m_estopMode);
        if (m_estopLoaded >= 0 && m_estopLoaded <= 2) m_estopMode->setCurrentIndex(m_estopLoaded);
        QSignalBlocker c(m_estopPin);
        if (m_estopPinLoaded >= 0) m_estopPin->setCurrentIndex(m_estopPinLoaded);
    }
    const bool editable = conn && m_estopLoaded >= 0 && idle && !plc && !m_boardBusy;
    m_estopMode->setEnabled(editable);
    m_estopPin->setEnabled(editable && m_estopPinLoaded >= 0);
    // Pins this project uses for something else: the board refuses them while it has this project; greyed either way.
    if (auto *model = qobject_cast<QStandardItemModel *>(m_estopPin->model())) {
        for (int i = 0; i < kTriggerInputs && i < model->rowCount(); ++i) {
            std::string why = BoardInputBlocked(m_project, i);
            const BoardInput in = BoardInputAt(m_project, i);
            if (why.empty() && (!in.name.empty() || in.role != CcioRole::None)) why = "input";
            if (why.empty() && WaitOnPin(m_project, i)) why = "wait";
            if (why.empty() && OpAt(i) >= 0) why = "operator";
            QStandardItem *item = model->item(i);
            const bool blocked = !why.empty() && i != m_estopPinLoaded;
            item->setFlags(blocked ? item->flags() & ~Qt::ItemIsEnabled : item->flags() | Qt::ItemIsEnabled);
            item->setToolTip(why == "output"    ? QStringLiteral("An output in this project.")
                             : why == "trigger" ? QStringLiteral("This project's timeline trigger.")
                             : why == "encoder" ? QStringLiteral("The Encoder Adapter's (an encoder master).")
                             : why == "input"   ? QStringLiteral("Set up as an input in this project (Inputs).")
                             : why == "wait"    ? QStringLiteral("A timeline wait's input in this project (Waits).")
                             : why == "operator" ? OpPinName(i) + QStringLiteral(" (Board, Panel).")
                                                : QString());
        }
    }
    if (m_opSummary) {
        // The panel's inputs, from the board's status (remembered when offline).
        static const char *const kOp[] = {"Enable", "Disable", "Fault reset", "Start", "Stop"};
        QStringList set;
        for (int k = 0; k < 5; ++k) {
            if (m_opPinsKnown[k] >= 0) set << QStringLiteral("%1 %2").arg(QLatin1String(kOp[k]), QString::fromStdString(BoardInputName(m_opPinsKnown[k])));
        }
        m_opSummary->setText(set.isEmpty() ? QStringLiteral("None") : set.join(QStringLiteral(", ")));
        m_opSummary->setToolTip(m_opSummary->text());
        m_opButton->setEnabled(conn && s.valid && s.opKnown);
    }
    QString text, color = QStringLiteral("#9fa6af");
    if (conn && s.valid) {
        // The chosen input's level: din= (0.8.9), else A-9's alone.
        const int pin = m_estopPinLoaded >= 0 ? m_estopPinLoaded : kEstopInput;
        const int bit = s.inputs >= 0 ? (s.inputs >> pin) & 1 : pin == kEstopInput ? s.a9 : -1;
        const QString name = QString::fromStdString(BoardInputName(pin));
        const QString level = bit < 0 ? QString() : bit ? name + QStringLiteral(" high") : name + QStringLiteral(" low");
        if (s.estop == QStringLiteral("ok")) {
            text = level.isEmpty() ? QStringLiteral("OK") : level + QStringLiteral(": OK");
            color = QStringLiteral("#7fe3a0");
        } else if (s.estop == QStringLiteral("open")) {
            text = level.isEmpty() ? QStringLiteral("Open") : level + QStringLiteral(": open");
            color = QStringLiteral("#ff8a80");
        } else {
            text = level;
        }
        if (!idle && !plc) text += text.isEmpty() ? QStringLiteral("Motors enabled") : QStringLiteral(" (motors enabled)");
    }
    m_estopState->setText(text);
    m_estopState->setStyleSheet(QStringLiteral("QLabel { color: %1; }").arg(color));
}

void MainWindow::ApplyBoardNetwork(bool ask) {
    const net::Settings to = NetFields();
    const QStringList commands = net::Commands(m_netLoaded, to);
    if (commands.isEmpty() || !net::Check(to).isEmpty()) return;
    if (ask) {
        const BoardLink::State &s = m_board->state();
        QString text = QStringLiteral("Save these settings and restart the board?\n\n%1\nIP address %2, netmask %3, gateway %4")
                           .arg(to.dhcp ? QStringLiteral("DHCP, falling back to:") : QStringLiteral("Static:"), to.ip, to.netmask, to.gateway);
        if (s.eip != QStringLiteral("none")) {
            text += QStringLiteral("\n\nThe PLC at %1 is connected: the restart drops its connection.").arg(s.eipFrom);
        }
        if (QMessageBox::question(this, QStringLiteral("Network settings"), text) != QMessageBox::Yes) return;
    }
    m_netExpected = to;
    RefreshNetwork();
    m_netState->setText(QStringLiteral("Saving..."));
    auto next = std::make_shared<std::function<void(int)>>();
    *next = [this, commands, next](int i) {
        if (i == commands.size()) {
            RestartBoard(QStringLiteral("Board: network settings saved, restarting..."));
            *next = nullptr;
            return;
        }
        m_board->command(commands[i], [this, commands, next, i](bool ok, const QStringList &lines) {
            if (ok) {
                (*next)(i + 1);
                return;
            }
            // Some may be saved already: read back what the board has now, so the fields don't mislead.
            m_netExpected.reset();
            m_netState->setText(QStringLiteral("%1: %2").arg(commands[i], lines.isEmpty() ? QStringLiteral("no reply") : lines.last()));
            *next = nullptr;
            LoadBoardNetwork();
        });
    };
    (*next)(0);
}

// Connects again once the board is back (Ethernet ~5 s; the USB port reappears a little sooner).
void MainWindow::ReconnectBoard(const QString &done) {
    auto *timer = new QTimer(this);
    auto *clock = new QElapsedTimer;
    clock->start();
    connect(timer, &QTimer::timeout, this, [this, timer, clock, done]() {
        if (m_board->isConnected() && m_board->state().valid) {
            statusBar()->showMessage(QStringLiteral("%1 and reconnected.").arg(done), 6000);
        } else if (clock->elapsed() > 30000) {
            statusBar()->showMessage(QStringLiteral("%1; press Connect to reconnect.").arg(done), 8000);
        } else {
            if (!m_board->isConnected()) BoardConnect();
            if (m_board->isConnected()) m_board->command(QStringLiteral("status"));
            return;
        }
        timer->stop();
        timer->deleteLater();
        delete clock;
        RefreshBoard();
    });
    timer->start(2500);
    RefreshBoard();
}

// The update owns the USB port while it runs, and the board goes away, so the board's link lets go and comes
// back after.
void MainWindow::OpenFirmwareUpdate() {
    if (findChild<FirmwareDialog *>()) return;
    const bool wasConnected = m_board->isConnected();
    if (wasConnected && m_boardTransport->currentIndex() == 1) m_board->disconnectLink();
    auto *dlg = new FirmwareDialog(this);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setWindowModality(Qt::WindowModal);
    auto again = std::make_shared<bool>(wasConnected);
    connect(dlg, &FirmwareDialog::aboutToUpdate, this, [this, again]() {
        *again = *again || m_board->isConnected();
        m_board->disconnectLink();
    });
    connect(dlg, &QDialog::finished, this, [this, again]() {
        if (*again) ReconnectBoard(QStringLiteral("Board back"));
    });
    dlg->show();
}

int MainWindow::OpAt(int pin) const {
    for (int k = 0; k < 5; ++k) {
        if (pin >= 0 && m_opPinsKnown[k] == pin) return k;
    }
    return -1;
}

QString MainWindow::OpPinName(int pin) const {
    static const char *const kOp[] = {"Enable", "Disable", "Fault reset", "Start", "Stop"};
    const int k = OpAt(pin);
    return k < 0 ? QString() : QStringLiteral("Operator: %1").arg(QLatin1String(kOp[k]));
}

// The board's operator inputs (its settings, like the estop): changes go to the board at once.
void MainWindow::OpenOperatorInputs() {
    OperatorDialog dlg(m_board, m_project, m_estopPinKnown, this);
    dlg.exec();
    m_board->command(QStringLiteral("status"));
}

// Modal, as Axis jog. Its upload puts the wizard's project into the app first (one undo step), then uploads it.
void MainWindow::OpenSwitchWizard() {
    if (m_project.axes.empty()) return;
    SwitchWizard dlg(m_board, m_project, m_estopPinKnown,
                     [this](const CamProject &p, std::function<void(bool, const QString &)> done) {
                         EditProject(QStringLiteral("Set up limits"), [&](CamProject &q) { q = p; });
                         m_boardBusy = true;
                         RefreshBoard();
                         m_board->uploadProfile(m_project, [this, done](bool ok, const QString &msg) {
                             m_boardBusy = false;
                             m_board->command(QStringLiteral("status"));
                             RefreshBoard();
                             done(ok, msg);
                         });
                     },
                     this);
    dlg.setOperatorPins(m_opPinsKnown);
    dlg.exec();
}

// Modal: the board panel's own buttons wait while it is open, so nothing else drives the motors meanwhile.
void MainWindow::OpenAxisJog() {
    if (!m_board->isConnected()) return;
    JogDialog dlg(m_board, this);
    dlg.exec();
}

// Start was refused because axes are off their cam positions (Axis jog moved them): list every one that is, and offer
// to move them there, one at a time in motor order, each at its Axis jog speed. Start stays the user's to press.
void MainWindow::OfferCamPositions() {
    m_board->command(QStringLiteral("axjog"), [this](bool ok, const QStringList &r) {
        if (!ok) return;
        QVector<JogDialog::MotorInfo> off;
        QStringList lines, disabled;
        const bool sim = m_board->state().simulate;
        for (const QString &l : r) {
            JogDialog::MotorInfo m;
            if (!JogDialog::ParseMotorLine(l, &m) || m.role != JogDialog::Role::Axis || !m.hasCam || m.off == 0) continue;
            off << m;
            lines << QStringLiteral("%1 (M-%2): %3 %4 off").arg(m.name).arg(m.motor).arg(m.off / m.cpu, 0, 'g', 4).arg(m.units);
            if (!m.on && !sim) disabled << QStringLiteral("M-%1").arg(m.motor);
        }
        if (off.isEmpty()) return;
        QMessageBox box(this);
        box.setObjectName(QStringLiteral("ECamCamPositions"));
        box.setIcon(QMessageBox::Question);
        box.setWindowTitle(QStringLiteral("Start"));
        box.setText(QStringLiteral("Start needs every axis at its cam position. Off it now:\n\n%1").arg(lines.join(QLatin1Char('\n'))));
        QPushButton *move = nullptr;
        if (disabled.isEmpty()) {
            box.setInformativeText(QStringLiteral("Move them there one at a time, M-0 first, at each motor's Axis jog speed?"));
            move = box.addButton(QStringLiteral("Move to cam positions"), QMessageBox::AcceptRole);
        } else {
            box.setInformativeText(QStringLiteral("Enable the motors to move them: %1 is disabled.").arg(disabled.join(QStringLiteral(", "))));
        }
        QPushButton *jog = box.addButton(QStringLiteral("Axis jog…"), QMessageBox::ActionRole);
        box.addButton(QMessageBox::Cancel);
        if (move) box.setDefaultButton(move);
        // Wide enough for its label and the style's padding (with a style sheet the box can size a long one short).
        for (QAbstractButton *b : box.buttons()) b->setMinimumWidth(b->fontMetrics().horizontalAdvance(b->text()) + 2 * 14 + 16);
        box.exec();
        if (move && box.clickedButton() == move) MoveToCamPositions(off, 0);
        else if (box.clickedButton() == jog) OpenAxisJog();
    });
}

// Move off[i]'s motor to its cam position, wait until it has stopped, then the next.
void MainWindow::MoveToCamPositions(const QVector<JogDialog::MotorInfo> &off, int i) {
    if (i >= off.size()) {
        statusBar()->showMessage(QStringLiteral("All at their cam positions: press Start."), 8000);
        m_board->command(QStringLiteral("status"));
        return;
    }
    const JogDialog::MotorInfo &m = off[i];
    // The speed the Axis jog window uses for this motor: its saved one, else 5 % of the limit (rounded down).
    QSettings s(QStringLiteral("ECamUI"), QStringLiteral("ECamUI"));
    const double vmax = std::max(m.vmax, 1e-3);
    double speed = s.value(QStringLiteral("axisJog/M%1/speed").arg(m.motor), 0.0).toDouble();
    if (!(speed > 0.0) || speed > vmax) {
        speed = vmax * 0.05;
        const double mag = std::pow(10.0, std::floor(std::log10(std::max(speed, 1e-6))));
        speed = std::floor(speed / mag) * mag;
    }
    statusBar()->showMessage(QStringLiteral("Moving %1 (M-%2) to its cam position...").arg(m.name).arg(m.motor));
    m_board->command(QStringLiteral("axjog %1 cam %2").arg(m.motor).arg(speed, 0, 'g', 12), [this, off, i](bool ok, const QStringList &r) {
        if (!ok) {
            QMessageBox::warning(this, QStringLiteral("Move to cam positions"),
                                 QStringLiteral("%1 (M-%2): %3").arg(off[i].name).arg(off[i].motor).arg(r.isEmpty() ? QString() : r.last()));
            return;
        }
        // Until it has stopped (or a minute has gone), then the next one.
        auto started = std::make_shared<QElapsedTimer>();
        started->start();
        auto poll = std::make_shared<std::function<void()>>();
        *poll = [this, off, i, started, poll]() {
            m_board->command(QStringLiteral("axjog"), [this, off, i, started, poll](bool ok2, const QStringList &r2) {
                bool moving = !ok2;
                for (const QString &l : r2) {
                    JogDialog::MotorInfo m2;
                    if (JogDialog::ParseMotorLine(l, &m2) && m2.motor == off[i].motor) moving = m2.moving;
                }
                if (!moving) {
                    MoveToCamPositions(off, i + 1);
                } else if (started->elapsed() > 60000) {
                    statusBar()->showMessage(QStringLiteral("%1 is still moving after a minute: stopped waiting.").arg(off[i].name), 8000);
                } else {
                    QTimer::singleShot(150, this, [poll]() { (*poll)(); });
                }
            });
        };
        QTimer::singleShot(150, this, [poll]() { (*poll)(); });
    });
}

void MainWindow::RestartBoard(const QString &message) {
    statusBar()->showMessage(message);
    m_board->command(QStringLiteral("restart"), [this](bool, const QStringList &) {
        m_board->disconnectLink();
        ReconnectBoard(QStringLiteral("Board restarted"));
    });
}

void MainWindow::ConnectBoard(const QString &target) {
    const bool usb = target.startsWith(QStringLiteral("COM"), Qt::CaseInsensitive);
    m_boardTransport->setCurrentIndex(usb ? 1 : 0);
    if (usb) {
        const int i = m_boardTarget->findData(target);
        if (i >= 0) m_boardTarget->setCurrentIndex(i);
        else m_boardTarget->setEditText(target);
    } else {
        m_boardTarget->setEditText(target);
    }
    BoardConnect();
}

quint32 MainWindow::ProjectHash() const { return ecam::ProfileHash(io::UploadLines(m_project)); }

void MainWindow::RefreshBoard() {
    if (!m_board) return;
    const bool conn = m_board->isConnected();
    const BoardLink::State &s = m_board->state();
    m_boardConnect->setText(conn ? QStringLiteral("Disconnect") : QStringLiteral("Connect"));
    RefreshNetwork();
    RefreshEstop();
    RefreshMachineStatus();
    m_boardTransport->setEnabled(!conn);
    m_boardTarget->setEnabled(!conn);
    m_boardDiscover->setEnabled(!conn);
    const bool loaded = conn && s.valid && s.profile != 0;
    const bool moving = s.state == QStringLiteral("running");
    const bool fault = s.fault.size() && s.fault != QStringLiteral("none");
    // A PLC with control (EtherNet/IP): the board refuses what would change the motion; Stop, Abort and
    // Disable still work.
    const bool plc = conn && s.valid && s.plcInControl();
    {
        const char *color = !conn ? "#6b7480" : fault ? "#e57373" : plc ? "#e6b450" : "#4fc06e";
        if (m_boardLed && m_boardLedColor != QLatin1String(color)) {
            m_boardLedColor = QLatin1String(color);
            m_boardLed->setStyleSheet(QStringLiteral("QLabel { color: %1; font-size: 15px; }").arg(m_boardLedColor));
        }
        if (m_boardLed) {
            m_boardLed->setToolTip(!conn ? QStringLiteral("Not connected") : fault ? QStringLiteral("Fault")
                                   : plc ? QStringLiteral("Connected; the PLC has control") : QStringLiteral("Connected"));
        }
    }
    m_boardUpload->setEnabled(conn && !m_boardBusy && !moving && !plc);
    m_boardDownload->setEnabled(loaded && !m_boardBusy);
    m_boardForget->setEnabled(conn && s.valid && s.stored && !m_boardBusy && !plc);
    m_boardForget->setToolTip(s.stored ? QStringLiteral("Erase the profile stored on the board: it keeps running until a restart, then the board starts empty.")
                                       : QStringLiteral("Nothing stored on the board."));
    m_boardEnable->setEnabled(loaded && !s.enabled && !m_boardBusy && !plc);
    m_boardDisable->setEnabled(conn && (s.enabled || s.motorsOn != 0 || s.jogMoving != 0));
    m_boardAxisJog->setEnabled(conn && s.valid && !m_boardBusy);  // any motor, with or without a profile
    m_boardClearFault->setVisible(conn && fault);
    m_boardSim->setEnabled(conn && !moving && !plc);
    m_boardSim->setChecked(s.simulate);
    m_boardStart->setEnabled(loaded && !fault && (s.enabled || s.simulate) && !m_boardBusy && !plc);
    m_boardTrigger->setEnabled(loaded && !plc && (s.timeline == QStringLiteral("armed") || s.timeline == QStringLiteral("done")));
    m_boardStop->setEnabled(conn && (moving || s.timeline == QStringLiteral("armed") || s.state == QStringLiteral("jogging") || s.homing != 0));
    m_boardHome->setVisible(conn && s.hasHome != 0);
    m_boardHome->setEnabled(loaded && !fault && s.enabled && !s.simulate && !moving && s.homing == 0 && !m_boardBusy && !plc);
    m_boardAbort->setEnabled(conn);
    // Hand jog needs this project on the board, and enabled motors (or simulate).
    const bool matches = loaded && s.profile == ProjectHash();
    m_boardJog->setEnabled(!plc && (m_jogging || (matches && !fault && (s.enabled || s.simulate) && !moving && !m_boardBusy)));
    // While the PLC has control these say why they are greyed; their own tips come back after. (Forget's tip
    // is set above on every refresh.)
    for (QWidget *w : {static_cast<QWidget *>(m_boardUpload), static_cast<QWidget *>(m_boardForget),
                       static_cast<QWidget *>(m_boardEnable), static_cast<QWidget *>(m_boardSim),
                       static_cast<QWidget *>(m_boardStart), static_cast<QWidget *>(m_boardJog)}) {
        const bool own = w != m_boardForget;
        if (own && !w->property("ecamTip").isValid()) w->setProperty("ecamTip", w->toolTip());
        if (plc) {
            w->setToolTip(QStringLiteral("The PLC has control (EtherNet/IP): the board refuses this until the PLC "
                                         "releases PLC_CONTROL. Stop, Abort and Disable still work."));
        } else if (own) {
            w->setToolTip(w->property("ecamTip").toString());
        }
    }
    // Crank to 0: hand jog on, crank jogging and at rest (no speed, no target on its way), not already at 0.
    const bool crankAtRest = m_jogging && s.crank == QStringLiteral("jogging") && std::fabs(s.rpm) < 1e-6 && !m_jogInFlight &&
                             m_jogPending.isEmpty();
    const bool atZero = std::fabs(std::fmod(s.deg, 360.0)) < 0.005 || std::fabs(std::fmod(s.deg, 360.0) - 360.0) < 0.005;
    m_boardCrankZero->setVisible(crankAtRest && UsesCrank(m_project));
    m_boardCrankZero->setEnabled(!atZero);

    // Encoder row: with an encoder master.
    {
        const CamMaster &em = m_project.master;
        const bool encRow = em.source == MasterSource::Encoder;
        for (QWidget *w : {static_cast<QWidget *>(m_boardEncLabel), static_cast<QWidget *>(m_boardEnc),
                           static_cast<QWidget *>(m_boardEncZero)}) {
            w->setVisible(encRow);
        }
        if (encRow) {
            const bool live = conn && s.valid;
            m_boardEncZero->setEnabled(live);
            if (!live) {
                m_boardEnc->setText(QStringLiteral("<span style='color:#8a929d;'>not connected</span>"));
            } else {
                const qint64 counts = s.encoder - m_encZero;
                QString text = QStringLiteral("<span style='font-size:13pt; font-weight:600;'>%1</span> counts").arg(counts);
                if (em.crankPpr > 0) text += QStringLiteral("  \u00b7  %1 rev").arg(QString::number(counts / em.crankPpr, 'f', 3));
                if (em.lineMode && em.lineCountsPerMm > 0) {
                    text += QStringLiteral("  \u00b7  %1 mm").arg(QString::number(counts / em.lineCountsPerMm, 'f', 2));
                }
                if (m_encZero != 0) text += QStringLiteral("  <span style='color:#8a929d;'>(from zero)</span>");
                if (em.encoderReverse) text += QStringLiteral("  <span style='color:#8a929d;'>reversed</span>");
                m_boardEnc->setText(text);
            }
        }
    }

    // Sim encoder row: encoder master, connected, simulating.
    {
        const CamMaster &em = m_project.master;
        const bool simRow = em.source == MasterSource::Encoder && conn && s.valid && s.simulate;
        for (QWidget *w : {static_cast<QWidget *>(m_boardSimEncLabel), static_cast<QWidget *>(m_boardSimEncSpeed),
                           static_cast<QWidget *>(m_boardSimEncRun), static_cast<QWidget *>(m_boardSimEncState)}) {
            w->setVisible(simRow);
        }
        const QString unit = em.lineMode ? QStringLiteral(" m/min") : QStringLiteral(" rpm");
        if (m_boardSimEncSpeed->suffix() != unit) {
            QSignalBlocker b(m_boardSimEncSpeed);
            m_boardSimEncSpeed->setSuffix(unit);
            // Start from the project's expected speed.
            m_boardSimEncSpeed->setValue(em.lineMode ? em.rpm * em.designLength / 1000.0 : em.rpm);
        }
        // Simulate off or disconnected: the board is back on the real encoder, so is the button.
        if (!simRow && m_boardSimEncRun->isChecked()) {
            QSignalBlocker b(m_boardSimEncRun);
            m_boardSimEncRun->setChecked(false);
            m_boardSimEncRun->setText(QStringLiteral("Run"));
        }
        if (simRow) {
            m_boardSimEncState->setText(s.simEncoder ? QStringLiteral("%1 counts/s").arg(QString::number(s.simEncoderRate, 'f', 0))
                                                     : QStringLiteral("real encoder input"));
        }
    }

    // Cut-length row: only in cut-length mode with crank axes.
    {
        const CamMaster &cm = m_project.master;
        const bool lineRow = cm.lineMode && UsesCrank(m_project);
        for (QWidget *w : {static_cast<QWidget *>(m_boardCutLabel), static_cast<QWidget *>(m_boardCut),
                           static_cast<QWidget *>(m_boardCutSet), static_cast<QWidget *>(m_boardCutState)}) {
            w->setVisible(lineRow);
        }
        if (lineRow) {
            const bool thisProfile = conn && s.valid && s.profile == ProjectHash() && s.cutLength > 0.0;
            m_boardCut->setMinimum(std::max(0.001, LineMinLength(m_project)));
            if (!m_boardCut->hasFocus() && !thisProfile && std::fabs(m_boardCut->value() - (cm.designLength - cm.kerf)) > 1e-9) {
                m_boardCut->setValue(cm.designLength - cm.kerf);
            }
            m_boardCutSet->setEnabled(thisProfile && !m_boardBusy && !plc);
            m_boardCutSet->setToolTip(thisProfile ? QStringLiteral("Send the length to the board: it applies from the next cut on.")
                                                  : QStringLiteral("Not available: upload this project first."));
            if (thisProfile) {
                const bool pending = std::fabs(s.cutNext - s.cutLength) > 1e-6;
                QString text;
                if (s.syncLeft >= 0.0) {
                    // Synchronized: the blade is on the cut line and moving with it.
                    text = QStringLiteral("<span style='font-size:13pt; font-weight:700; color:#7fe3a0;'>on the cut line</span> "
                                          "<span style='color:#7fe3a0;'>sync, %1 mm left</span><br>")
                               .arg(QString::number(s.syncLeft, 'f', 1));
                } else if (s.cutGap >= 0.0) {
                    text = QStringLiteral("blade to cut line <span style='font-size:14pt; font-weight:700; color:#e6ebf3;'>%1</span> mm<br>")
                               .arg(QString::number(s.cutGap, 'f', 1));
                } else {
                    text = QStringLiteral("<span style='font-size:14pt;'>&nbsp;</span><br>");  // same height without a readout
                }
                text += pending ? QStringLiteral("cutting %1 mm, %2 mm from the next cut").arg(Sig(s.cutLength), Sig(s.cutNext))
                                : QStringLiteral("cutting %1 mm (shortest %2 mm)").arg(Sig(s.cutLength), Sig(s.cutMin));
                m_boardCutState->setText(text);
            } else {
                m_boardCutState->setText(QStringLiteral("shortest %1 mm").arg(Sig(LineMinLength(m_project))));
            }
        }
    }

    // Timeline row: only with timeline axes; says what the timeline is doing and what starts it.
    const bool hasTimeline = UsesTimeline(m_project);
    for (QWidget *w : {static_cast<QWidget *>(m_boardTimelineLabel), static_cast<QWidget *>(m_boardTrigger),
                       static_cast<QWidget *>(m_boardTriggerOnStart), static_cast<QWidget *>(m_boardTimelineState)}) {
        w->setVisible(hasTimeline);
    }
    if (hasTimeline) {
        const CamMaster &tm = m_project.master;
        const QString source =
            tm.trigger == TriggerSource::DigitalInput
                ? QStringLiteral("%1 %2 edge").arg(m_triggerInput->itemText(tm.triggerInput),
                                                   tm.triggerRising ? QStringLiteral("rising") : QStringLiteral("falling"))
            : tm.trigger == TriggerSource::EtherNetIP ? QStringLiteral("the PLC's TRIGGER bit (EtherNet/IP)")
                                                      : QString();
        const QString orButton = source.isEmpty() ? QStringLiteral("the Trigger button")
                                                  : QStringLiteral("%1 or the Trigger button").arg(source);
        QString state, why;
        const QString &tl = s.timeline;
        if (!conn || !s.valid) {
            state = QStringLiteral("Not connected.");
            why = QStringLiteral("Connect to the board first.");
        } else if (!loaded) {
            state = QStringLiteral("No profile on the board.");
            why = QStringLiteral("Upload, then Start.");
        } else if (tl == QStringLiteral("armed")) {
            state = QStringLiteral("Armed: waiting for %1.").arg(orButton);
            why = QStringLiteral("Starts the timeline from t = 0.");
        } else if (tl == QStringLiteral("running") && s.endOfCycle) {
            state = QStringLiteral("Finishing the cycle, then stops.");
            why = QStringLiteral("Already running.");
        } else if (tl == QStringLiteral("running")) {
            state = QStringLiteral("Running%1.").arg(tm.timelineRepeat ? QStringLiteral(" (repeats until Stop)") : QString());
            why = QStringLiteral("Already running.");
        } else if (tl == QStringLiteral("stopping")) {
            state = QStringLiteral("Stopping.");
            why = QStringLiteral("Stopping.");
        } else if (tl == QStringLiteral("done")) {
            state = QStringLiteral("Done: %1 runs it again.").arg(orButton);
            why = QStringLiteral("Runs the timeline again from t = 0.");
        } else if (tl == QStringLiteral("jogging")) {
            state = QStringLiteral("Hand jog.");
            why = QStringLiteral("Hand jog is on.");
        } else if (tl == QStringLiteral("waiting")) {
            // Its name and input from the open project when that is the one on the board.
            const int k = s.waitIndex - 1;
            QString what = QStringLiteral("wait %1").arg(s.waitIndex);
            if (s.profile == ProjectHash() && k >= 0 && k < static_cast<int>(m_project.waits.size())) {
                const TimelineWait &w = m_project.waits[k];
                what = QStringLiteral("%1 (%2 %3)").arg(w.name.empty() ? QStringLiteral("wait %1").arg(s.waitIndex) : QString::fromStdString(w.name),
                                                        QString::fromStdString(WaitInputName(w)), w.on ? QStringLiteral("on") : QStringLiteral("off"));
            }
            state = QStringLiteral("Waiting for %1, %2 s.").arg(what).arg(s.waitedMs / 1000.0, 0, 'f', 1);
            why = QStringLiteral("Already running.");
        } else {
            state = m_boardTriggerOnStart->isChecked()
                        ? QStringLiteral("Idle: Start arms it and triggers it at once (Trigger on Start).")
                        : QStringLiteral("Idle: Start arms it; then %1 starts it.").arg(orButton);
            why = QStringLiteral("Start first: the timeline has to be armed.");
        }
        m_boardTimelineState->setText(state);
        m_boardWaitSkip->setVisible(conn && tl == QStringLiteral("waiting") && s.simulate);
        if (m_timing) m_timing->setActiveWait(conn && s.profile == ProjectHash() && tl == QStringLiteral("waiting") ? s.waitIndex : 0);
        m_boardTrigger->setToolTip(m_boardTrigger->isEnabled() ? why : QStringLiteral("Not available: %1").arg(why));
    }

    // Firmware: the app and the board have to match.
    const QString appFirmware = QStringLiteral(ECAM_PRODUCT_VERSION);
    const QString boardFirmware = conn ? m_board->firmwareVersion() : QString();
    const bool firmwareDiffers = !boardFirmware.isEmpty() && boardFirmware != appFirmware;
    m_boardFirmware->setText(firmwareDiffers ? QStringLiteral("<span style='color:#ffcc80;'>Board %1, this app needs %2</span>").arg(boardFirmware.toHtmlEscaped(), appFirmware)
                             : boardFirmware.isEmpty() ? QStringLiteral("This app carries %1").arg(appFirmware)
                                                       : QStringLiteral("Board %1, matches this app").arg(boardFirmware.toHtmlEscaped()));
    m_boardFirmwareUpdate->setEnabled(!(conn && s.valid && (s.enabled || moving || plc)) && !m_boardBusy);

    if (!conn) {
        m_boardStatus->setText(QStringLiteral("Not connected. Ethernet: TCP %1 (Discover finds the board). USB: the ClearCore's COM port.")
                                   .arg(BoardLink::kConsolePort));
        return;
    }
    if (!s.valid) {
        m_boardStatus->setText(QStringLiteral("Connected, waiting for status..."));
        return;
    }
    QStringList parts;
    const QString stateColor = fault ? QStringLiteral("#ff8a80") : moving ? QStringLiteral("#7fe3a0") : QStringLiteral("#e6ebf3");
    parts << QStringLiteral("<span style='color:%1; font-weight:600;'>%2</span>").arg(stateColor, s.state);
    if (fault) parts << QStringLiteral("<span style='color:#ff8a80;'>fault: %1 (Clear fault, or Enable; Simulate also clears it)</span>").arg(s.fault.toHtmlEscaped());
    if (s.estop == QStringLiteral("open")) parts << QStringLiteral("<span style='color:#ff8a80;'>estop open</span>");
    if (s.estop == QStringLiteral("off")) {
        parts << QStringLiteral("<span style='color:#ffcc80;' title='The Board page&#39;s Estop setting turns it on.'>"
                                "no estop</span>");
    }
    if (firmwareDiffers) {
        parts << QStringLiteral("<span style='color:#ffcc80;' title='Update it from the Board page.'>firmware %1, this app needs %2</span>")
                     .arg(boardFirmware.toHtmlEscaped(), appFirmware);
    }
    if (s.enabled) {
        parts << QStringLiteral("motors enabled");
    } else if (s.motorsOn != 0) {
        QStringList on;
        for (int c = 0; c < kMotorConnectors; ++c) {
            if ((s.motorsOn >> c) & 1u) on << QStringLiteral("M-%1").arg(c);
        }
        parts << QStringLiteral("%1 enabled").arg(on.join(QStringLiteral(", ")));
    } else {
        parts << QStringLiteral("motors disabled");
    }
    if (s.eip == QStringLiteral("control")) {
        parts << QStringLiteral("<span style='color:#80d8ff; font-weight:600;' title='EtherNet/IP: the PLC holds "
                                "PLC_CONTROL. Stop, Abort and Disable still work here.'>PLC %1 in control (%2 ms)</span>")
                     .arg(s.eipFrom.toHtmlEscaped())
                     .arg(s.eipRpi);
    } else if (s.eip == QStringLiteral("run") || s.eip == QStringLiteral("idle")) {
        parts << QStringLiteral("<span style='color:#8a929d;' title='EtherNet/IP: a PLC reads the status%1'>PLC %2 "
                                "connected%3</span>")
                     .arg(s.eip == QStringLiteral("idle") ? QStringLiteral(" (it is in Program)") : QString(),
                          s.eipFrom.toHtmlEscaped(),
                          s.eip == QStringLiteral("idle") ? QStringLiteral(", Program") : QString());
    }
    if (s.simulate) parts << QStringLiteral("<span style='color:#ffcc80;'>simulating</span>");
    if (s.profile == 0) {
        parts << QStringLiteral("no profile (Upload)");
    } else if (s.profile == ProjectHash()) {
        parts << QStringLiteral("<span style='color:#7fe3a0;'>profile %1 = this project</span>").arg(s.profile, 8, 16, QChar('0'));
    } else {
        parts << QStringLiteral("<span style='color:#ffcc80;'>profile %1 differs from this project (Upload or Download)</span>")
                     .arg(s.profile, 8, 16, QChar('0'));
    }
    if (s.profile != 0 && !s.stored) {
        parts << QStringLiteral("<span style='color:#ffcc80;'>not kept in flash: lost at reboot</span>");
    }
    QString motion;
    if (UsesCrank(m_project) || s.crank != QStringLiteral("off")) {
        motion += QStringLiteral("crank %1 %2° rev %3%4").arg(s.crank, QString::number(s.deg, 'f', 1)).arg(s.rev)
                      .arg(s.rpm <= 0 ? QString()
                           : m_project.master.lineMode
                               ? QStringLiteral(" %1 m/min").arg(QString::number(s.rpm * m_project.master.designLength / 1000.0, 'f', 2))
                               : QStringLiteral(" %1 rpm").arg(QString::number(s.rpm, 'f', 1)));
    }
    if (UsesTimeline(m_project) || s.timeline != QStringLiteral("idle")) {
        if (!motion.isEmpty()) motion += QStringLiteral("  ·  ");
        motion += QStringLiteral("timeline %1 %2 s at %3 %").arg(s.timeline, QString::number(s.t, 'f', 2),
                                                                  QString::number(s.override * 100.0, 'f', 0));
        if (!m_project.outputs.empty() || s.outputs != 0) {
            QStringList on;
            for (int n = 0; n < kOutputs; ++n) {
                if ((s.outputs >> n) & 1u) on << QStringLiteral("IO-%1").arg(n);
            }
            motion += QStringLiteral("  ·  outputs %1%2").arg(on.isEmpty() ? QStringLiteral("off") : on.join(QLatin1Char(' ')),
                                                             s.simulate ? QStringLiteral(" (simulated, pins off)") : QString());
        }
    }
    // This axis's commanded travel since the profile was loaded, in its units.
    if (s.profile == ProjectHash() && m_axis < s.axisCounts.size()) {
        const CamAxis &a = Axis();
        motion += QStringLiteral("  ·  %1 %2 %3").arg(QString::fromStdString(a.name).toHtmlEscaped(),
                                                      QString::number(s.axisCounts[m_axis] / a.countsPerUnit, 'f', 3),
                                                      QString::fromStdString(a.units).toHtmlEscaped());
    }
    m_boardStatus->setText(parts.join(QStringLiteral("  ·  ")) + QStringLiteral("<br>") + motion);
    m_boardStatus->setToolTip(m_boardStatus->text());  // the strip may not fit a long line
}

QWidget *MainWindow::BuildStripPanel() {
    auto *group = new QGroupBox(QStringLiteral("Displacement Diagram"), this);
    auto *v = new QVBoxLayout(group);
    v->setContentsMargins(10, 16, 10, 10);
    v->setSpacing(6);

    auto *bar = new QHBoxLayout();
    bar->setSpacing(8);
    // Ignored: Fit and Snap don't count towards the bar's minimum, or they'd hold the splitters open until they wrap.
    // Stretch gives them the row's spare width: the readout at its left, Fit and Snap at its right.
    auto *snapBox = new QWidget(group);
    snapBox->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    auto *snapRow = new QHBoxLayout(snapBox);
    snapRow->setContentsMargins(0, 0, 0, 0);
    snapRow->setSpacing(8);
    // Which axis the diagram shows (the same as the Axis selector in Settings).
    auto *axisLabel = new QLabel(QStringLiteral("Axis"), group);
    axisLabel->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));
    bar->addWidget(axisLabel);
    m_stripAxis = new QComboBox(group);
    m_stripAxis->setObjectName(QStringLiteral("ECamStripAxis"));
    m_stripAxis->setMinimumWidth(130);
    m_stripAxis->setSizeAdjustPolicy(QComboBox::AdjustToContents);
    m_stripAxis->setToolTip(QStringLiteral("The axis shown and edited in the diagram (Ctrl+PgUp / Ctrl+PgDown)."));
    bar->addWidget(m_stripAxis);
    m_lockButton = new QPushButton(group);
    m_lockButton->setObjectName(QStringLiteral("ECamLock"));
    m_lockButton->setCheckable(true);
    m_lockButton->setIconSize(QSize(16, 16));
    m_lockButton->setMinimumWidth(80);
    bar->addWidget(m_lockButton);
    auto *sep0 = new QFrame(group);
    sep0->setObjectName(QStringLiteral("ECamSep"));
    sep0->setFrameShape(QFrame::VLine);
    bar->addWidget(sep0);
    m_playButton = new QPushButton(QStringLiteral("▶  Run"), group);
    m_playButton->setCheckable(true);
    m_playButton->setToolTip(QStringLiteral("Turn the crank at the master speed (Space)."));
    m_playButton->setShortcut(QKeySequence(Qt::Key_Space));
    m_playButton->setMinimumWidth(80);
    bar->addWidget(m_playButton);
    auto *sep = new QFrame(group);
    sep->setObjectName(QStringLiteral("ECamSep"));
    sep->setFrameShape(QFrame::VLine);
    bar->addWidget(sep);
    m_showVel = new QCheckBox(QStringLiteral("Velocity"), group);
    m_showAcc = new QCheckBox(QStringLiteral("Acceleration"), group);
    m_showJerk = new QCheckBox(QStringLiteral("Jerk"), group);
    m_showVel->setStyleSheet(QStringLiteral("QCheckBox { color: %1; }").arg(theme::kVelocity.name()));
    m_showAcc->setStyleSheet(QStringLiteral("QCheckBox { color: %1; }").arg(theme::kAccel.name()));
    m_showJerk->setStyleSheet(QStringLiteral("QCheckBox { color: %1; }").arg(theme::kJerk.name()));
    m_showVel->setChecked(AppSettings().value("view/vel", true).toBool());
    m_showAcc->setChecked(AppSettings().value("view/acc", true).toBool());
    m_showJerk->setChecked(AppSettings().value("view/jerk", false).toBool());
    bar->addWidget(m_showVel);
    bar->addWidget(m_showAcc);
    bar->addWidget(m_showJerk);
    m_showOutputs = new QCheckBox(QStringLiteral("Outputs"), group);
    m_showOutputs->setStyleSheet(QStringLiteral("QCheckBox { color: #ffb347; }"));
    m_showOutputs->setToolTip(QStringLiteral("The Outputs band: IO-0..IO-5 pulses on this axis's master."));
    m_showOutputs->setChecked(AppSettings().value("view/outputs", true).toBool());
    bar->addWidget(m_showOutputs);
    // The cursor readout, out of the way of the curves.
    m_readout = new QLabel(group);
    m_readout->setObjectName(QStringLiteral("ECamReadout"));
    m_readout->setTextFormat(Qt::RichText);
    m_readout->setAlignment(Qt::AlignCenter);
    m_readout->setStyleSheet(QStringLiteral(
        "QLabel#ECamReadout { background: #121418; border: 1px solid #2c333c; border-radius: 3px; padding: 3px 12px;"
        " font-family: Consolas, 'Courier New', monospace; font-size: 12px; }"));
    m_readout->setToolTip(QStringLiteral("At the cursor: master position, follower position, velocity and acceleration "
                                         "(per second at the master speed). While dragging, the knot."));
    {
        const int full = QFontMetrics(QFont(QStringLiteral("Consolas"), 9))
                             .horizontalAdvance(QStringLiteral("θ 000.00°   s -0000.00 mm   v -0000.00 mm/s   a -00000.0 mm/s²")) + 30;
        // Fixed bounds: its text changes every sample, and that mustn't move the bar or wrap it. Only the largest
        // numbers need the upper end.
        m_readout->setMinimumWidth(full * 7 / 8);
        m_readout->setMaximumWidth(full);
        m_readout->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    }
    m_fitButton = new QPushButton(QStringLiteral("Fit"), group);
    m_fitButton->setObjectName(QStringLiteral("ECamFit"));
    m_fitButton->setEnabled(false);
    m_fitButton->setToolTip(QStringLiteral("Show the whole cycle (Ctrl+0).\n"
                                           "Zoom: Ctrl+wheel, or Ctrl + / Ctrl −.  Pan: middle-drag, or Shift+wheel."));
    snapRow->addWidget(m_readout);
    snapRow->addStretch(1);
    snapRow->addWidget(m_fitButton);
    m_snapLabel = new QLabel(QStringLiteral("Snap"), group);
    m_snapLabel->setToolTip(QStringLiteral("Snap for knots and pulses: along the master, then in position (Alt: no snap)."));
    snapRow->addWidget(m_snapLabel);
    m_snapDeg = MakeSpin(group, 0, 1000, 3, QStringLiteral("°"));
    m_snapDeg->setSpecialValueText(QStringLiteral("off"));
    m_snapDeg->setFixedWidth(84);
    m_snapCrank = AppSettings().value("view/snapDeg", 1.0).toDouble();
    m_snapTime = AppSettings().value("view/snapTime", 0.01).toDouble();
    m_snapDeg->setValue(m_snapCrank);
    snapRow->addWidget(m_snapDeg);
    snapRow->addWidget(new QLabel(QStringLiteral("/"), group));
    m_snapPos = MakeSpin(group, 0, 1e6, 4);
    m_snapPos->setSpecialValueText(QStringLiteral("off"));
    m_snapPos->setFixedWidth(84);
    m_snapPos->setValue(AppSettings().value("view/snapPos", 0.1).toDouble());
    snapRow->addWidget(m_snapPos);
    bar->addWidget(snapBox, 1);
    v->addLayout(bar);
    // On a window too narrow for one row, the readout, Fit and Snap wrap to a second.
    auto *bar2 = new QHBoxLayout();
    bar2->setSpacing(10);
    v->addLayout(bar2);
    auto wrapped = std::make_shared<bool>(false);
    auto fit = [group, bar, bar2, snapBox, wrapped]() {
        const bool wrap = group->width() < bar->sizeHint().width() + snapBox->minimumSizeHint().width() + bar->spacing() + 22;
        if (wrap == *wrapped) return;
        *wrapped = wrap;
        (wrap ? bar : bar2)->removeWidget(snapBox);
        (wrap ? bar2 : bar)->addWidget(snapBox, 1);
    };
    group->installEventFilter(new ResizeWatcher(group, fit));

    m_strip = new ProfileStrip(group);
    m_strip->setMinimumHeight(250);  // on a short window the top row gives way, and its forms scroll
    v->addWidget(m_strip, 1);
    connect(m_strip, &ProfileStrip::readoutChanged, m_readout, &QLabel::setText);
    connect(m_strip, &ProfileStrip::zoomChanged, this, [this](double z) {
        m_fitButton->setEnabled(z > 1.0);
        m_fitButton->setText(z > 1.0 ? QStringLiteral("Fit  ×%1").arg(QString::number(z, 'f', z < 10 ? 1 : 0)) : QStringLiteral("Fit"));
    });
    connect(m_fitButton, &QPushButton::clicked, m_strip, &ProfileStrip::resetZoom);
    {
        auto zoomAction = [this](const QList<QKeySequence> &keys, std::function<void()> fn) {
            auto *a = new QAction(this);
            a->setShortcuts(keys);
            a->setShortcutContext(Qt::WindowShortcut);
            connect(a, &QAction::triggered, this, fn);
            addAction(a);
        };
        zoomAction({QKeySequence(Qt::CTRL | Qt::Key_Plus), QKeySequence(Qt::CTRL | Qt::Key_Equal)},
                   [this]() { m_strip->zoomBy(1.25); });
        zoomAction({QKeySequence(Qt::CTRL | Qt::Key_Minus)}, [this]() { m_strip->zoomBy(1.0 / 1.25); });
        zoomAction({QKeySequence(Qt::CTRL | Qt::Key_0)}, [this]() { m_strip->resetZoom(); });
    }
    connect(m_stripAxis, &QComboBox::currentIndexChanged, this, [this](int i) {
        if (m_updating || i < 0 || i == m_axis) return;
        m_axisCombo->setCurrentIndex(i);  // one path for switching axes
    });
    // Step through the axes from the keyboard.
    auto *prevAxis = new QAction(this);
    prevAxis->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_PageUp));
    connect(prevAxis, &QAction::triggered, this, [this]() {
        if (m_axis > 0) m_axisCombo->setCurrentIndex(m_axis - 1);
    });
    auto *nextAxis = new QAction(this);
    nextAxis->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_PageDown));
    connect(nextAxis, &QAction::triggered, this, [this]() {
        if (m_axis + 1 < static_cast<int>(m_project.axes.size())) m_axisCombo->setCurrentIndex(m_axis + 1);
    });
    addAction(prevAxis);
    addAction(nextAxis);

    auto applyCurves = [this]() {
        m_strip->setCurvesVisible(m_showVel->isChecked(), m_showAcc->isChecked(), m_showJerk->isChecked());
    };
    applyCurves();
    for (QCheckBox *c : {m_showVel, m_showAcc, m_showJerk}) connect(c, &QCheckBox::toggled, this, applyCurves);
    m_strip->setOutputsVisible(m_showOutputs->isChecked());
    connect(m_showOutputs, &QCheckBox::toggled, this, [this](bool on) {
        AppSettings().setValue("view/outputs", on);
        m_strip->setOutputsVisible(on);
    });
    auto applySnap = [this]() {
        if (m_updating) return;
        (OnTimeline() ? m_snapTime : m_snapCrank) = m_snapDeg->value();
        m_strip->setSnap(m_snapDeg->value(), m_snapPos->value());
    };
    applySnap();
    connect(m_snapDeg, &QDoubleSpinBox::valueChanged, this, applySnap);
    connect(m_snapPos, &QDoubleSpinBox::valueChanged, this, applySnap);
    connect(m_playButton, &QPushButton::toggled, this, &MainWindow::TogglePlay);
    return group;
}

void MainWindow::BuildMenus() {
    auto *file = menuBar()->addMenu(QStringLiteral("&File"));
    file->addAction(QStringLiteral("&New"), QKeySequence::New, this, &MainWindow::NewProject);
    file->addAction(QStringLiteral("New from &guided setup…"), QKeySequence(QStringLiteral("Ctrl+Shift+N")), this,
                    [this]() { RunSetupWizard(); });
    file->addAction(QStringLiteral("&Open…"), QKeySequence::Open, this, [this]() { OpenProject(); });
    QMenu *recent = file->addMenu(QStringLiteral("Open &recent"));
    connect(recent, &QMenu::aboutToShow, this, [this, recent]() {
        recent->clear();
        const QStringList list = startup::RecentProjects();
        for (const QString &p : list) {
            recent->addAction(QDir::toNativeSeparators(p), this, [this, p]() {
                if (MaybeSave()) OpenFile(p);
            });
        }
        if (list.isEmpty()) recent->addAction(QStringLiteral("(none yet)"))->setEnabled(false);
    });
    QMenu *examples = file->addMenu(QStringLiteral("Open e&xample"));
    for (const startup::Example &e : startup::Examples()) {
        QAction *a = examples->addAction(e.title, this, [this, e]() { OpenExample(e.file, e.title); });
        a->setToolTip(e.about);
    }
    examples->setToolTipsVisible(true);
    file->addAction(QStringLiteral("Start page…"), this, &MainWindow::ShowStartPage);
    file->addAction(QStringLiteral("&Save"), QKeySequence::Save, this, [this]() { Save(); });
    file->addAction(QStringLiteral("Save &as…"), QKeySequence::SaveAs, this, [this]() { SaveAs(); });
    file->addSeparator();
    file->addAction(QStringLiteral("Export table (CSV)…"), this, [this]() { Export(false); });
    file->addAction(QStringLiteral("Export table (C header)…"), this, [this]() { Export(true); });
    m_exportDxfAction = file->addAction(QStringLiteral("Export cam DXF…"), this, &MainWindow::ExportCamDxf);
    m_exportDxfAction->setToolTip(QStringLiteral("The plate cam of the current crank axis, for CAD / CAM."));
    file->addAction(QStringLiteral("Export path DXF…"), this, &MainWindow::ExportPathDxf)
        ->setToolTip(QStringLiteral("The current axis's path (its position over one cycle) as DXF, in mm or inches."));
    auto *xlsx = file->addAction(QStringLiteral("Export workbook (Excel)…"), this, &MainWindow::ExportWorkbook);
    xlsx->setToolTip(QStringLiteral("The whole project as an Excel workbook: summary, master, timing, and per axis its "
                                    "segments, sampled motion and cam, with charts."));
    file->addSeparator();
    auto *report = file->addAction(QStringLiteral("Generate &report…"), this, &MainWindow::GenerateReport);
    report->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_R));
    report->setToolTip(QStringLiteral("A PDF for reviewing the project: system diagram, machine-cycle flow chart, timing, "
                                      "and per axis its settings, results, segments, motion charts and cam."));
    file->setToolTipsVisible(true);
    file->addSeparator();
    file->addAction(QStringLiteral("E&xit"), QKeySequence::Quit, this, &QWidget::close);

    auto *edit = menuBar()->addMenu(QStringLiteral("&Edit"));
    QAction *undo = m_undo->createUndoAction(this, QStringLiteral("&Undo"));
    undo->setShortcut(QKeySequence::Undo);
    QAction *redo = m_undo->createRedoAction(this, QStringLiteral("&Redo"));
    redo->setShortcut(QKeySequence::Redo);
    edit->addAction(undo);
    edit->addAction(redo);
    edit->addSeparator();
    m_lockAction = edit->addAction(QStringLiteral("&Lock profile"));
    m_lockAction->setCheckable(true);
    m_lockAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_L));
    m_lockAction->setToolTip(QStringLiteral("Lock the motion and the output pulses against accidental edits."));
    connect(m_lockAction, &QAction::toggled, this, &MainWindow::SetLocked);
    connect(m_lockButton, &QPushButton::toggled, this, &MainWindow::SetLocked);
    edit->addSeparator();
    m_splitSegAction = edit->addAction(QStringLiteral("Split segment"), this, [this]() { m_splitButton->click(); });
    m_deleteSegAction = edit->addAction(QStringLiteral("Delete segment"), this, [this]() { m_deleteButton->click(); });

    auto *axis = m_axisMenu = menuBar()->addMenu(QStringLiteral("&Axis"));
    axis->addAction(QStringLiteral("Add axis"), this, [this]() { AddAxis(false); });
    axis->addAction(QStringLiteral("Duplicate axis"), this, [this]() { AddAxis(true); });
    axis->addAction(QStringLiteral("Remove axis"), this, &MainWindow::RemoveAxis);
    axis->addSeparator();
    axis->addAction(QStringLiteral("Export path DXF…"), this, &MainWindow::ExportPathDxf)
        ->setToolTip(QStringLiteral("This axis's path (its position over one cycle) as DXF, in mm or inches."));
    m_importPointsAction = axis->addAction(QStringLiteral("Import points as spline…"), this, [this]() {
        const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("Import points"),
                                                          AppSettings().value("files/lastDir").toString(),
                                                          QStringLiteral("Points (*.csv *.txt);;All files (*)"));
        if (path.isEmpty()) return;
        QString text;
        if (ImportPoints(path, &text)) {
            statusBar()->showMessage(text, 10000);
        } else {
            QMessageBox::warning(this, QStringLiteral("Import points"), text);
        }
    });
    m_importPointsAction->setToolTip(QStringLiteral("Replaces this axis's segments with a cubic spline through the points in a file: "
                                                    "master value in the first column, position in the second."));

    auto *view = menuBar()->addMenu(QStringLiteral("&View"));
    QAction *dock = view->addAction(QStringLiteral("Project tree"));
    dock->setCheckable(true);
    dock->setChecked(!m_settingsDock->isCollapsed());
    dock->setShortcut(QKeySequence(Qt::Key_F9));
    connect(dock, &QAction::toggled, this, [this](bool on) { m_settingsDock->setCollapsed(!on); });
    connect(m_settingsDock, &SideDock::collapsedChanged, dock, [dock](bool collapsed) {
        QSignalBlocker b(dock);
        dock->setChecked(!collapsed);
    });
    // The pane holds the Cam view or the Timing view, whichever fits the selected axis: this shows or hides it.
    QAction *camDock = view->addAction(QStringLiteral("Cam / Timing view"));
    camDock->setObjectName(QStringLiteral("ECamViewPaneAction"));
    camDock->setCheckable(true);
    camDock->setChecked(!m_camDock->isCollapsed());
    camDock->setShortcut(QKeySequence(Qt::SHIFT | Qt::Key_F9));
    connect(camDock, &QAction::toggled, this, [this](bool on) { m_camDock->setCollapsed(!on); });
    connect(m_camDock, &SideDock::collapsedChanged, camDock, [camDock](bool collapsed) {
        QSignalBlocker b(camDock);
        camDock->setChecked(!collapsed);
    });
    // Only for a project with crank and timeline axes: the Timing view while a crank axis is selected.
    m_timingAction = view->addAction(QStringLiteral("Timing view for crank axes"));
    m_timingAction->setObjectName(QStringLiteral("ECamTimingAction"));
    m_timingAction->setCheckable(true);
    m_timingAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_T));
    m_timingAction->setToolTip(QStringLiteral("Show the Timing view (all timeline axes) also while a crank axis is selected."));
    connect(m_timingAction, &QAction::toggled, this, [this](bool on) {
        m_forceTiming = on;
        if (on && m_camDock->isCollapsed()) m_camDock->setCollapsed(false);  // nothing to see with the pane folded away
        Refresh();
    });
    view->addSeparator();

    auto *tools = menuBar()->addMenu(QStringLiteral("&Tools"));
    QAction *scope = tools->addAction(QStringLiteral("ClearPath &scope…"), this, [this]() {
        if (!m_scope) m_scope = new ScopeWindow(this);
        m_scope->show();
        m_scope->raise();
        m_scope->activateWindow();
    });
    scope->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_O));
    scope->setToolTip(QStringLiteral("Traces from a ClearPath motor on its own USB port: tracking error, velocity, "
                                     "bus voltage, temperature, HLFB, enable and alerts."));
    tools->addAction(QStringLiteral("Set up &limits and homing…"), this, &MainWindow::OpenSwitchWizard)
        ->setToolTip(QStringLiteral("One axis's switches: find each one, its contact, and its job proved by moving the axis."));
    tools->addAction(QStringLiteral("Update board &firmware…"), this, &MainWindow::OpenFirmwareUpdate)
        ->setToolTip(QStringLiteral("Put this app's firmware on a ClearCore over USB, keeping its profile."));
    tools->setToolTipsVisible(true);

    auto *help = menuBar()->addMenu(QStringLiteral("&Help"));
    help->addAction(QStringLiteral("User &guide"), QKeySequence::HelpContents, this, [this]() { HelpWindow::Show(this); });
    help->addAction(QStringLiteral("Getting &started"), this, [this]() { HelpWindow::Show(this, QStringLiteral("01-start.html")); });
    help->addAction(QStringLiteral("Worked &examples"), this, [this]() { HelpWindow::Show(this, QStringLiteral("10-examples.html")); });
    help->addAction(QStringLiteral("&Keyboard and mouse"), this, [this]() { HelpWindow::Show(this, QStringLiteral("13-keys.html")); });
    help->addAction(QStringLiteral("ClearPath s&cope"), this, [this]() { HelpWindow::Show(this, QStringLiteral("12-scope.html")); });
    help->addSeparator();
    help->addAction(QStringLiteral("&About " ECAM_PRODUCT_NAME "…"), this, [this]() {
        QMessageBox::about(this, QStringLiteral("About " ECAM_PRODUCT_NAME),
                           QStringLiteral("<h3>" ECAM_PRODUCT_NAME " " ECAM_PRODUCT_VERSION "</h3>"
                                          "<p>Design, analyse and run electronic cam profiles on a Teknic ClearCore.</p>"
                                          "<p><b>Experimental.</b> Not certified for production or safety-critical use; keep "
                                          "a physical estop in the circuit.</p>"
                                          "<p>MIT License. Copyright (c) 2026 Adam G. Sweeney.<br>Built with Qt (LGPL-3.0) and "
                                          "libxlsxwriter (BSD-2-Clause); see THIRD_PARTY_NOTICES.md.</p>"));
    });
}

/* ------------------------------------------------------------------------- */
/* Edits                                                                      */
/* ------------------------------------------------------------------------- */

void MainWindow::Restore(const CamProject &project, int axis) {
    m_project = project;
    m_axis = std::clamp(axis, 0, static_cast<int>(m_project.axes.size()) - 1);
    m_seg = std::clamp(m_seg, 0, static_cast<int>(Axis().segments.size()) - 1);
    Refresh();
}

void MainWindow::EditProject(const QString &text, const std::function<void(CamProject &)> &fn, int mergeId) {
    if (m_updating) return;
    const CamProject before = m_project;
    const int axisBefore = m_axis;
    fn(m_project);
    NormalizeProject(m_project);
    if (SameProject(before, m_project)) {
        Refresh();
        return;
    }
    m_undo->push(new ProjectCommand(this, before, axisBefore, m_project, m_axis, text, mergeId));
    m_seg = std::clamp(m_seg, 0, static_cast<int>(Axis().segments.size()) - 1);
    Refresh();
}

void MainWindow::EditOutputs(const QString &text, const std::function<void(std::vector<OutputPulse> &)> &fn) {
    EditProject(text, [&](CamProject &p) { fn(p.outputs); });
}

namespace {
enum CcioColumn { kCcioPin, kCcioLive, kCcioName, kCcioRole, kCcioAxis, kCcioContact, kCcioColumns };
}

// CCIO-8 inputs: how many boards the machine has, and on the board picked in the tree, what each input is for.
QWidget *MainWindow::BuildCcioPanel(QWidget *parent) {
    m_ccioGroup = new CollapsibleGroup(QStringLiteral("CCIO-8"), QStringLiteral("ccio"), parent);
    auto *v = new QVBoxLayout(m_ccioGroup->body());
    v->setContentsMargins(10, 12, 10, 6);
    v->setSpacing(4);
    auto *top = new QHBoxLayout();
    top->setSpacing(8);
    auto *label = new QLabel(QStringLiteral("Boards on COM-1"), m_ccioGroup);
    m_ccioBoards = new QSpinBox(m_ccioGroup);
    m_ccioBoards->setObjectName(QStringLiteral("ECamCcioBoards"));
    m_ccioBoards->setRange(0, kCcioMaxBoards);
    m_ccioBoards->setToolTip(QStringLiteral("How many CCIO-8 boards this machine has on COM-1, up to 8. Lowering it drops the settings of the boards removed."));
    m_ccioState = new QLabel(m_ccioGroup);
    m_ccioState->setObjectName(QStringLiteral("ECamCcioState"));
    m_ccioState->setTextFormat(Qt::RichText);
    m_ccioState->setWordWrap(true);
    top->addWidget(label);
    top->addWidget(m_ccioBoards);
    top->addSpacing(12);
    top->addWidget(m_ccioState, 1);
    v->addLayout(top);
    m_ccioHeading = new QLabel(m_ccioGroup);
    m_ccioHeading->setObjectName(QStringLiteral("ECamCcioHeading"));
    m_ccioHeading->setWordWrap(true);
    m_ccioHeading->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));
    v->addWidget(m_ccioHeading);

    m_ccioTable = new QTableWidget(kCcioPinsPerBoard, kCcioColumns, m_ccioGroup);
    m_ccioTable->setObjectName(QStringLiteral("ECamCcioTable"));
    m_ccioTable->setHorizontalHeaderLabels({QStringLiteral("Input"), QStringLiteral("State"), QStringLiteral("Name"),
                                            QStringLiteral("Function"), QStringLiteral("Axis"), QStringLiteral("Contact")});
    m_ccioTable->verticalHeader()->setVisible(false);
    // All eight inputs in the top row at 1080p.
    m_ccioTable->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    m_ccioTable->verticalHeader()->setDefaultSectionSize(26);
    m_ccioTable->setStyleSheet(QStringLiteral("QComboBox { min-height: 18px; padding-top: 0; padding-bottom: 0; }"));
    m_ccioTable->horizontalHeader()->setFixedHeight(24);
    m_ccioTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_ccioTable->horizontalHeader()->setSectionResizeMode(kCcioPin, QHeaderView::Fixed);
    m_ccioTable->horizontalHeader()->setSectionResizeMode(kCcioLive, QHeaderView::Fixed);
    m_ccioTable->horizontalHeader()->resizeSection(kCcioPin, 80);
    m_ccioTable->horizontalHeader()->resizeSection(kCcioLive, 60);
    m_ccioTable->setSelectionMode(QAbstractItemView::NoSelection);
    m_ccioTable->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    m_ccioTable->setMinimumHeight(110);
    v->addWidget(m_ccioTable, 1);

    for (int r = 0; r < kCcioPinsPerBoard; ++r) {
        for (int c : {kCcioPin, kCcioLive}) {
            auto *it = new QTableWidgetItem();
            it->setFlags(Qt::ItemIsEnabled);
            it->setTextAlignment(Qt::AlignCenter);
            m_ccioTable->setItem(r, c, it);
        }
        m_ccioTable->setItem(r, kCcioName, new QTableWidgetItem());
        auto *role = new QComboBox(m_ccioTable);
        role->addItems({QStringLiteral("Not used"), QStringLiteral("Home"), QStringLiteral("Limit +"), QStringLiteral("Limit \u2212")});
        m_ccioTable->setCellWidget(r, kCcioRole, role);
        auto *axis = new QComboBox(m_ccioTable);
        m_ccioTable->setCellWidget(r, kCcioAxis, axis);
        auto *contact = new QComboBox(m_ccioTable);
        contact->addItems({QStringLiteral("NC"), QStringLiteral("NO")});
        contact->setToolTip(QStringLiteral("Normally closed (tripped when the input is off) or normally open (tripped when on)."));
        m_ccioTable->setCellWidget(r, kCcioContact, contact);
        connect(contact, &QComboBox::currentIndexChanged, this, [this, r](int k) {
            if (m_updating || k < 0) return;
            EditCcioInput(r, QStringLiteral("CCIO-8 input contact"), [&](CcioInput &i) { i.normallyOpen = k == 1; });
        });
        connect(role, &QComboBox::currentIndexChanged, this, [this, r](int k) {
            if (m_updating || k < 0) return;
            EditCcioInput(r, QStringLiteral("CCIO-8 input function"), [&](CcioInput &i) {
                i.role = static_cast<CcioRole>(k);
                if (i.role == CcioRole::None) i.axis = -1;
                else if (i.axis < 0) i.axis = m_axis;
            });
        });
        connect(axis, &QComboBox::currentIndexChanged, this, [this, r](int k) {
            if (m_updating || k < 0) return;
            EditCcioInput(r, QStringLiteral("CCIO-8 input axis"), [&](CcioInput &i) { i.axis = k; });
        });
    }
    connect(m_ccioTable, &QTableWidget::itemChanged, this, [this](QTableWidgetItem *item) {
        if (m_updating || !item || item->column() != kCcioName) return;
        const std::string name = item->text().trimmed().toStdString();
        EditCcioInput(item->row(), QStringLiteral("CCIO-8 input name"), [&](CcioInput &i) { i.name = name; });
    });
    connect(m_ccioBoards, &QSpinBox::valueChanged, this, [this](int n) {
        if (m_updating) return;
        EditProject(QStringLiteral("CCIO-8 boards"), [&](CamProject &p) { p.ccio.boards = n; });
    });
    return m_ccioGroup;
}

QWidget *MainWindow::BuildInputsPanel(QWidget *parent) {
    m_inputsGroup = new CollapsibleGroup(QStringLiteral("Inputs"), QStringLiteral("inputs"), parent);
    auto *v = new QVBoxLayout(m_inputsGroup->body());
    v->setContentsMargins(10, 8, 10, 6);
    v->setSpacing(4);
    {
        auto *row = new QHBoxLayout();
        row->setSpacing(8);
        m_limitStop = new QDoubleSpinBox(m_inputsGroup);
        m_limitStop->setObjectName(QStringLiteral("ECamLimitStop"));
        m_limitStop->setRange(kLimitStopMin, kLimitStopMax);
        m_limitStop->setDecimals(2);
        m_limitStop->setSingleStep(0.05);
        m_limitStop->setSuffix(QStringLiteral(" s"));
        m_limitStop->setKeyboardTracking(false);
        m_limitStop->setToolTip(QStringLiteral("A tripped limit brings the machine to rest over this time (the controlled stop)."));
        m_overtravel = new QLabel(m_inputsGroup);
        m_overtravel->setObjectName(QStringLiteral("ECamOvertravel"));
        m_overtravel->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));
        m_overtravel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        auto *wizard = new QPushButton(QStringLiteral("Set up switches…"), m_inputsGroup);
        wizard->setObjectName(QStringLiteral("ECamSwitchWizard"));
        connect(wizard, &QPushButton::clicked, this, &MainWindow::OpenSwitchWizard);
        row->addWidget(new QLabel(QStringLiteral("Limit stop"), m_inputsGroup));
        row->addWidget(m_limitStop);
        row->addSpacing(10);
        row->addWidget(m_overtravel, 1);
        row->addWidget(wizard);
        v->addLayout(row);
        connect(m_limitStop, &QDoubleSpinBox::valueChanged, this, [this](double s) {
            if (m_updating) return;
            EditProject(QStringLiteral("Limit stop time"), [&](CamProject &p) { p.master.limitStopSeconds = s; });
        });
    }
    m_inputsTable = new QTableWidget(kTriggerInputs, kCcioColumns, m_inputsGroup);
    m_inputsTable->setObjectName(QStringLiteral("ECamInputsTable"));
    m_inputsTable->setHorizontalHeaderLabels({QStringLiteral("Input"), QStringLiteral("State"), QStringLiteral("Name"),
                                              QStringLiteral("Function"), QStringLiteral("Axis"), QStringLiteral("Contact")});
    m_inputsTable->verticalHeader()->setVisible(false);
    // All thirteen in the top row at 1080p: compact rows.
    m_inputsTable->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    m_inputsTable->verticalHeader()->setMinimumSectionSize(19);
    m_inputsTable->verticalHeader()->setDefaultSectionSize(19);
    m_inputsTable->setStyleSheet(QStringLiteral("QComboBox { min-height: 13px; padding-top: 0; padding-bottom: 0; }"));
    m_inputsTable->horizontalHeader()->setFixedHeight(20);
    m_inputsTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_inputsTable->horizontalHeader()->setSectionResizeMode(kCcioPin, QHeaderView::Fixed);
    m_inputsTable->horizontalHeader()->setSectionResizeMode(kCcioLive, QHeaderView::Fixed);
    m_inputsTable->horizontalHeader()->resizeSection(kCcioPin, 64);
    m_inputsTable->horizontalHeader()->resizeSection(kCcioLive, 52);
    m_inputsTable->setSelectionMode(QAbstractItemView::NoSelection);
    m_inputsTable->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    m_inputsTable->setMinimumHeight(110);
    v->addWidget(m_inputsTable, 1);
    for (int r = 0; r < kTriggerInputs; ++r) {
        for (int c : {kCcioPin, kCcioLive}) {
            auto *it = new QTableWidgetItem();
            it->setFlags(Qt::ItemIsEnabled);
            it->setTextAlignment(Qt::AlignCenter);
            m_inputsTable->setItem(r, c, it);
        }
        m_inputsTable->item(r, kCcioPin)->setText(QString::fromStdString(BoardInputName(r)));
        m_inputsTable->setItem(r, kCcioName, new QTableWidgetItem());
        auto *role = new QComboBox(m_inputsTable);
        role->addItems({QStringLiteral("Not used"), QStringLiteral("Home"), QStringLiteral("Limit +"), QStringLiteral("Limit \u2212")});
        m_inputsTable->setCellWidget(r, kCcioRole, role);
        auto *axis = new QComboBox(m_inputsTable);
        m_inputsTable->setCellWidget(r, kCcioAxis, axis);
        auto *contact = new QComboBox(m_inputsTable);
        contact->addItems({QStringLiteral("NC"), QStringLiteral("NO")});
        contact->setToolTip(QStringLiteral("Normally closed (tripped when the input is off) or normally open (tripped when on)."));
        m_inputsTable->setCellWidget(r, kCcioContact, contact);
        connect(contact, &QComboBox::currentIndexChanged, this, [this, r](int k) {
            if (m_updating || k < 0) return;
            EditBoardInput(r, QStringLiteral("Input contact"), [&](BoardInput &i) { i.normallyOpen = k == 1; });
        });
        connect(role, &QComboBox::currentIndexChanged, this, [this, r](int k) {
            if (m_updating || k < 0) return;
            EditBoardInput(r, QStringLiteral("Input function"), [&](BoardInput &i) {
                i.role = static_cast<CcioRole>(k);
                if (i.role == CcioRole::None) i.axis = -1;
                else if (i.axis < 0) i.axis = m_axis;
            });
        });
        connect(axis, &QComboBox::currentIndexChanged, this, [this, r](int k) {
            if (m_updating || k < 0) return;
            EditBoardInput(r, QStringLiteral("Input axis"), [&](BoardInput &i) { i.axis = k; });
        });
    }
    connect(m_inputsTable, &QTableWidget::itemChanged, this, [this](QTableWidgetItem *item) {
        if (m_updating || !item || item->column() != kCcioName) return;
        if (!(item->flags() & Qt::ItemIsEditable)) return;
        const std::string name = item->text().trimmed().toStdString();
        EditBoardInput(item->row(), QStringLiteral("Input name"), [&](BoardInput &i) { i.name = name; });
    });
    return m_inputsGroup;  // its live state is connected with the board's (BuildBoardPanel)
}

void MainWindow::EditBoardInput(int pin, const QString &text, const std::function<void(BoardInput &)> &fn) {
    EditProject(text, [&](CamProject &p) {
        BoardInput i = BoardInputAt(p, pin);
        fn(i);
        SetBoardInput(p, i);
    });
}

// A pin taken by something else shows what (an output, the timeline's trigger, the Encoder Adapter) and can't be set; one
// set anyway (a project from before, or changed since) keeps its fields so it can be cleared, and the review flags it.
void MainWindow::RefreshInputs() {
    if (!m_inputsTable) return;
    const bool was = m_updating;
    m_updating = true;
    int configured = 0;
    for (int r = 0; r < kTriggerInputs; ++r) {
        const BoardInput in = BoardInputAt(m_project, r);
        std::string blocked = BoardInputBlocked(m_project, r, m_estopPinKnown);
        if (blocked.empty() && WaitOnPin(m_project, r)) blocked = "wait";
        if (blocked.empty() && OpAt(r) >= 0) blocked = "operator";
        const bool set = !in.name.empty() || in.role != CcioRole::None;
        configured += set;
        const bool locked = m_locked || (!blocked.empty() && !set);
        QTableWidgetItem *pin = m_inputsTable->item(r, kCcioPin);
        pin->setForeground(!blocked.empty() && set ? QColor(0xff, 0x8a, 0x80) : !blocked.empty() ? QColor(0x6f, 0x78, 0x84) : QColor(0xe6, 0xeb, 0xf3));
        const QString why = blocked == "estop"     ? (m_estopPinLive ? QStringLiteral("Estop") : QStringLiteral("Estop (last seen)"))
                            : blocked == "output"  ? QStringLiteral("Output")
                            : blocked == "trigger" ? QStringLiteral("Timeline trigger")
                            : blocked == "encoder" ? QStringLiteral("Encoder Adapter")
                            : blocked == "wait"    ? QStringLiteral("Timeline wait")
                            : blocked == "operator" ? OpPinName(r)
                                                   : QString();
        pin->setToolTip(why.isEmpty() ? QString() : QStringLiteral("%1: not an input in this project.").arg(why));
        QTableWidgetItem *name = m_inputsTable->item(r, kCcioName);
        if (!blocked.empty() && !set) {
            name->setText(why);
            name->setForeground(QColor(0x6f, 0x78, 0x84));
            name->setFlags(Qt::ItemIsEnabled);
        } else {
            name->setText(QString::fromStdString(in.name));
            name->setForeground(QColor(0xe6, 0xeb, 0xf3));
            name->setFlags(m_locked ? Qt::ItemIsEnabled : Qt::ItemIsEnabled | Qt::ItemIsEditable);
        }
        auto *role = qobject_cast<QComboBox *>(m_inputsTable->cellWidget(r, kCcioRole));
        role->setCurrentIndex(static_cast<int>(in.role));
        role->setEnabled(!locked);
        auto *axis = qobject_cast<QComboBox *>(m_inputsTable->cellWidget(r, kCcioAxis));
        axis->clear();
        for (const CamAxis &a : m_project.axes) axis->addItem(QString::fromStdString(a.name));
        axis->setCurrentIndex(in.axis);
        axis->setEnabled(!locked && in.role != CcioRole::None);
        auto *contact = qobject_cast<QComboBox *>(m_inputsTable->cellWidget(r, kCcioContact));
        contact->setCurrentIndex(in.normallyOpen ? 1 : 0);
        contact->setEnabled(!locked && in.role != CcioRole::None);
    }
    m_limitStop->setValue(m_project.master.limitStopSeconds);
    m_limitStop->setEnabled(!m_locked);
    {
        // How far past a limit the controlled stop runs at the profile's peak speed: speed x time / 2.
        const review::ProjectInfo info = review::AnalyzeProject(m_project);
        QStringList over;
        for (size_t i = 0; i < m_project.axes.size() && i < info.axes.size(); ++i) {
            bool hasLimit = false;
            for (const BoardInput &b : m_project.inputs) hasLimit = hasLimit || (b.axis == int(i) && (b.role == CcioRole::LimitPositive || b.role == CcioRole::LimitNegative));
            for (const CcioInput &b : m_project.ccio.inputs) hasLimit = hasLimit || (b.axis == int(i) && (b.role == CcioRole::LimitPositive || b.role == CcioRole::LimitNegative));
            if (!hasLimit) continue;
            const double v = info.axes[i].stats.maxVel * info.axes[i].w;
            over << QStringLiteral("%1 %2 %3").arg(QString::fromStdString(m_project.axes[i].name))
                        .arg(v * m_project.master.limitStopSeconds / 2.0, 0, 'f', 1)
                        .arg(QString::fromStdString(m_project.axes[i].units));
        }
        m_overtravel->setText(over.isEmpty() ? QString() : QStringLiteral("Overtravel at peak speed: ") + over.join(QStringLiteral(", ")));
    }
    m_inputsGroup->setHeading(configured ? QStringLiteral("Inputs  (%1)").arg(configured) : QStringLiteral("Inputs"));
    m_updating = was;
    RefreshInputsLive();
}

void MainWindow::RefreshInputsLive() {
    if (!m_inputsTable) return;
    const BoardLink::State &s = m_board->state();
    const bool live = m_board->isConnected() && s.valid && s.inputs >= 0;
    for (int r = 0; r < kTriggerInputs; ++r) {
        QTableWidgetItem *it = m_inputsTable->item(r, kCcioLive);
        const bool on = live && ((s.inputs >> r) & 1);
        it->setText(!live ? QStringLiteral("\u2013") : on ? QStringLiteral("On") : QStringLiteral("Off"));
        it->setForeground(on ? QColor(0x7f, 0xe3, 0xa0) : QColor(0x8a, 0x92, 0x9d));
    }
}

void MainWindow::EditCcioInput(int pin, const QString &text, const std::function<void(CcioInput &)> &fn) {
    const int board = m_ccioBoard;
    EditProject(text, [&](CamProject &p) {
        CcioInput i = CcioInputAt(p.ccio, board, pin);
        fn(i);
        SetCcioInput(p.ccio, i);
    });
}

void MainWindow::RefreshCcio() {
    if (!m_ccioTable) return;
    const bool was = m_updating;
    m_updating = true;
    const int boards = m_project.ccio.boards;
    m_ccioBoards->setValue(boards);
    m_ccioBoards->setEnabled(!m_locked);
    m_ccioTable->setVisible(boards > 0);
    m_ccioHeading->setText(boards > 0 ? QStringLiteral("Board %1 of %2").arg(m_ccioBoard + 1).arg(boards)
                                      : QStringLiteral("No CCIO-8 boards in this project."));
    for (int r = 0; r < kCcioPinsPerBoard && boards > 0; ++r) {
        const CcioInput in = CcioInputAt(m_project.ccio, m_ccioBoard, r);
        m_ccioTable->item(r, kCcioPin)->setText(QString::fromStdString(CcioPinName(m_ccioBoard, r)));
        QTableWidgetItem *name = m_ccioTable->item(r, kCcioName);
        name->setText(QString::fromStdString(in.name));
        name->setFlags(m_locked ? Qt::ItemIsEnabled : Qt::ItemIsEnabled | Qt::ItemIsEditable);
        auto *role = qobject_cast<QComboBox *>(m_ccioTable->cellWidget(r, kCcioRole));
        role->setCurrentIndex(static_cast<int>(in.role));
        role->setEnabled(!m_locked);
        auto *axis = qobject_cast<QComboBox *>(m_ccioTable->cellWidget(r, kCcioAxis));
        axis->clear();
        for (const CamAxis &a : m_project.axes) axis->addItem(QString::fromStdString(a.name));
        axis->setCurrentIndex(in.axis);
        axis->setEnabled(!m_locked && in.role != CcioRole::None);
        auto *contact = qobject_cast<QComboBox *>(m_ccioTable->cellWidget(r, kCcioContact));
        contact->setCurrentIndex(in.normallyOpen ? 1 : 0);
        contact->setEnabled(!m_locked && in.role != CcioRole::None);
    }
    m_updating = was;
    RefreshCcioLive();
}

// What the board says now: it reports in its status, so about twice a second.
void MainWindow::RefreshCcioLive() {
    if (!m_ccioTable) return;
    const BoardLink::State &s = m_board->state();
    const bool live = m_board->isConnected() && s.valid;
    const int want = m_project.ccio.boards;
    QString text;
    if (!live) {
        text = QStringLiteral("Connect to the board to see the link and the inputs.");
    } else if (s.ccioBoards == 0) {
        text = want > 0 ? QStringLiteral("<span style='color:#ffcc80;'>The board found no CCIO-8. It looks when it starts: check COM-1 and "
                                         "the boards' 24 V, then restart it.</span>")
                        : QStringLiteral("The board found no CCIO-8.");
    } else if (s.ccioOff) {
        const QString found = QStringLiteral("The board found %1. The link is off: the profile on the board has no CCIO-8.").arg(s.ccioBoards);
        text = want > 0 ? QStringLiteral("<span style='color:#ffcc80;'>%1 Upload this project to turn it on.</span>").arg(found) : found;
    } else if (s.ccioBroken) {
        text = QStringLiteral("<span style='color:#ff8a80;'>CCIO-8 link broken: every input reads Off until it comes back.</span>");
    } else if (s.ccioBoards != want) {
        text = QStringLiteral("<span style='color:#ffcc80;'>The board found %1, this project has %2.</span>").arg(s.ccioBoards).arg(want);
    } else {
        text = QStringLiteral("<span style='color:#7fe3a0;'>Link OK, %1 found.</span>").arg(s.ccioBoards);
    }
    m_ccioState->setText(text);
    for (int r = 0; r < kCcioPinsPerBoard; ++r) {
        QTableWidgetItem *it = m_ccioTable->item(r, kCcioLive);
        const bool here = live && !s.ccioOff && m_ccioBoard < s.ccioBoards;
        const bool on = here && ((s.ccioInputs >> (m_ccioBoard * kCcioPinsPerBoard + r)) & 1u);
        it->setText(!here ? QStringLiteral("\u2013") : on ? QStringLiteral("On") : QStringLiteral("Off"));
        it->setForeground(on ? QColor(0x7f, 0xe3, 0xa0) : QColor(0x8a, 0x92, 0x9d));
    }
}

// The timeline's waits: a table, one row each. Times snap to where every timeline axis rests when added; the review flags
// one moved where an axis moves.
enum WaitColumn { kWaitTime, kWaitName, kWaitInput, kWaitLevel, kWaitChange, kWaitTimeout, kWaitColumns };

QWidget *MainWindow::BuildWaitsPanel(QWidget *parent) {
    m_waitsGroup = new CollapsibleGroup(QStringLiteral("Waits"), QStringLiteral("waits"), parent);
    auto *v = new QVBoxLayout(m_waitsGroup->body());
    v->setContentsMargins(14, 18, 14, 12);
    v->setSpacing(8);
    m_waitTable = new QTableWidget(0, kWaitColumns, m_waitsGroup);
    m_waitTable->setObjectName(QStringLiteral("ECamWaitTable"));
    m_waitTable->setHorizontalHeaderLabels({QStringLiteral("Time"), QStringLiteral("Name"), QStringLiteral("Input"), QStringLiteral("Wait for"),
                                            QStringLiteral("Change"), QStringLiteral("Timeout")});
    m_waitTable->horizontalHeaderItem(kWaitChange)->setToolTip(QStringLiteral(
        "Needs a change: it passes only if the input has been at the other level since this wait last passed, so a stuck\n"
        "sensor holds it. Off: the level alone, for a steady condition (a guard closed)."));
    m_waitTable->verticalHeader()->setVisible(false);
    m_waitTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    m_waitTable->horizontalHeader()->setSectionResizeMode(kWaitName, QHeaderView::Stretch);
    m_waitTable->horizontalHeader()->resizeSection(kWaitTime, 96);
    m_waitTable->horizontalHeader()->resizeSection(kWaitInput, 120);
    m_waitTable->horizontalHeader()->resizeSection(kWaitLevel, 70);
    m_waitTable->horizontalHeader()->resizeSection(kWaitChange, 64);
    m_waitTable->horizontalHeader()->resizeSection(kWaitTimeout, 110);
    m_waitTable->verticalHeader()->setMinimumSectionSize(19);
    m_waitTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_waitTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_waitTable->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    m_waitTable->setMinimumHeight(110);
    v->addWidget(m_waitTable, 1);
    auto *buttons = new QHBoxLayout();
    auto *add = new QPushButton(QStringLiteral("Add"), m_waitsGroup);
    add->setObjectName(QStringLiteral("ECamWaitAdd"));
    add->setToolTip(QStringLiteral("A wait at the cursor, moved to the nearest time every timeline axis is at rest."));
    auto *del = new QPushButton(QStringLiteral("Delete"), m_waitsGroup);
    del->setObjectName(QStringLiteral("ECamWaitDelete"));
    buttons->addWidget(add);
    buttons->addWidget(del);
    buttons->addStretch(1);
    v->addLayout(buttons);
    m_waitsInfo = new QLabel(m_waitsGroup);
    m_waitsInfo->setWordWrap(true);
    m_waitsInfo->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));
    v->addWidget(m_waitsInfo);
    connect(add, &QPushButton::clicked, this, [this]() { AddWait(m_time); });
    connect(del, &QPushButton::clicked, this, [this]() {
        const int r = m_waitTable->currentRow();
        if (r < 0 || r >= static_cast<int>(m_project.waits.size())) return;
        EditProject(QStringLiteral("Delete wait"), [&](CamProject &p) { p.waits.erase(p.waits.begin() + r); });
    });
    connect(m_waitTable, &QTableWidget::itemChanged, this, [this](QTableWidgetItem *item) {
        if (m_updating || !item || item->column() != kWaitName) return;
        const int r = item->row();
        const std::string name = item->text().trimmed().toStdString();
        EditProject(QStringLiteral("Wait name"), [&](CamProject &p) {
            if (r < static_cast<int>(p.waits.size())) p.waits[r].name = name;
        });
    });
    return m_waitsGroup;
}

void MainWindow::AddWait(double t) {
    if (!UsesTimeline(m_project)) {
        statusBar()->showMessage(QStringLiteral("Waits are for the timeline: the project has no timeline axes."), 5000);
        return;
    }
    if (static_cast<int>(m_project.waits.size()) >= kMaxWaits) {
        statusBar()->showMessage(QStringLiteral("At most %1 waits.").arg(kMaxWaits), 4000);
        return;
    }
    const double at = NearestTimelineRest(m_project, t);
    if (at < 0.0) {
        statusBar()->showMessage(QStringLiteral("No time in the cycle where every timeline axis is at rest: a wait needs one."), 6000);
        return;
    }
    // The first board input nothing else has.
    int pin = 0;
    for (int n = kTriggerInputs - 1; n >= 0; --n) {
        const BoardInput in = BoardInputAt(m_project, n);
        if (BoardInputBlocked(m_project, n, m_estopPinKnown).empty() && in.role == CcioRole::None && !WaitOnPin(m_project, n) && OpAt(n) < 0) pin = n;
    }
    TimelineWait w;
    w.t = at;
    w.pin = pin;
    EditProject(QStringLiteral("Add wait"), [&](CamProject &p) { p.waits.push_back(w); });
    ShowDetail(kPageWaits);
    if (std::fabs(at - t) > 0.0005) statusBar()->showMessage(QStringLiteral("Wait at %1 s, where every timeline axis rests.").arg(at, 0, 'g', 6), 5000);
}

void MainWindow::RefreshWaits() {
    if (!m_waitTable) return;
    const bool was = m_updating;
    m_updating = true;
    QSignalBlocker block(m_waitTable);
    const int n = static_cast<int>(m_project.waits.size());
    m_waitsGroup->setHeading(n ? QStringLiteral("Waits (%1)").arg(n) : QStringLiteral("Waits"));
    const int keep = m_waitTable->currentRow();
    m_waitTable->setRowCount(n);
    const double cycle = TimelineLength(m_project);
    for (int r = 0; r < n; ++r) {
        const TimelineWait &w = m_project.waits[r];
        auto *time = qobject_cast<QDoubleSpinBox *>(m_waitTable->cellWidget(r, kWaitTime));
        if (!time) {
            time = MakeSpin(m_waitTable, 0.0, 1e6, 3, QStringLiteral(" s"));
            time->setFrame(false);
            m_waitTable->setCellWidget(r, kWaitTime, time);
            connect(time, &QDoubleSpinBox::valueChanged, this, [this, time](double v) {
                if (m_updating) return;
                for (int k = 0; k < m_waitTable->rowCount(); ++k) {
                    if (m_waitTable->cellWidget(k, kWaitTime) != time) continue;
                    EditProject(QStringLiteral("Wait time"), [&](CamProject &p) {
                        if (k < static_cast<int>(p.waits.size())) p.waits[k].t = v;
                    });
                    break;
                }
            });
            auto *input = new QComboBox(m_waitTable);
            m_waitTable->setCellWidget(r, kWaitInput, input);
            connect(input, &QComboBox::currentIndexChanged, this, [this, input](int i) {
                if (m_updating || i < 0) return;
                const int code = input->itemData(i).toInt();
                for (int k = 0; k < m_waitTable->rowCount(); ++k) {
                    if (m_waitTable->cellWidget(k, kWaitInput) != input) continue;
                    EditProject(QStringLiteral("Wait input"), [&](CamProject &p) {
                        if (k >= static_cast<int>(p.waits.size())) return;
                        TimelineWait &x = p.waits[k];
                        x.source = code >= 200 ? WaitSource::Plc : code >= 100 ? WaitSource::Ccio : WaitSource::Board;
                        x.board = code >= 100 && code < 200 ? (code - 100) / kCcioPinsPerBoard : 0;
                        x.pin = code >= 200 ? code - 200 : code >= 100 ? (code - 100) % kCcioPinsPerBoard : code;
                    });
                    break;
                }
            });
            auto *level = new QComboBox(m_waitTable);
            level->addItems({QStringLiteral("On"), QStringLiteral("Off")});
            m_waitTable->setCellWidget(r, kWaitLevel, level);
            connect(level, &QComboBox::currentIndexChanged, this, [this, level](int i) {
                if (m_updating || i < 0) return;
                for (int k = 0; k < m_waitTable->rowCount(); ++k) {
                    if (m_waitTable->cellWidget(k, kWaitLevel) != level) continue;
                    EditProject(QStringLiteral("Wait level"), [&](CamProject &p) {
                        if (k < static_cast<int>(p.waits.size())) p.waits[k].on = i == 0;
                    });
                    break;
                }
            });
            // Needs a change: a check box centred in its cell.
            auto *changeCell = new QWidget(m_waitTable);
            auto *changeLayout = new QHBoxLayout(changeCell);
            changeLayout->setContentsMargins(0, 0, 0, 0);
            changeLayout->setAlignment(Qt::AlignCenter);
            auto *change = new QCheckBox(changeCell);
            change->setObjectName(QStringLiteral("ECamWaitChange"));
            changeLayout->addWidget(change);
            m_waitTable->setCellWidget(r, kWaitChange, changeCell);
            connect(change, &QCheckBox::toggled, this, [this, changeCell](bool on) {
                if (m_updating) return;
                for (int k = 0; k < m_waitTable->rowCount(); ++k) {
                    if (m_waitTable->cellWidget(k, kWaitChange) != changeCell) continue;
                    EditProject(QStringLiteral("Wait needs a change"), [&](CamProject &p) {
                        if (k < static_cast<int>(p.waits.size())) p.waits[k].edge = on;
                    });
                    break;
                }
            });
            auto *timeout = MakeSpin(m_waitTable, 0.0, 1e6, 2, QStringLiteral(" s"));
            timeout->setSpecialValueText(QStringLiteral("forever"));
            timeout->setFrame(false);
            m_waitTable->setCellWidget(r, kWaitTimeout, timeout);
            connect(timeout, &QDoubleSpinBox::valueChanged, this, [this, timeout](double v) {
                if (m_updating) return;
                for (int k = 0; k < m_waitTable->rowCount(); ++k) {
                    if (m_waitTable->cellWidget(k, kWaitTimeout) != timeout) continue;
                    EditProject(QStringLiteral("Wait timeout"), [&](CamProject &p) {
                        if (k < static_cast<int>(p.waits.size())) p.waits[k].timeout = v;
                    });
                    break;
                }
            });
        }
        time->setMaximum(std::max(cycle, w.t));
        time->setValue(w.t);
        time->setEnabled(!m_locked);
        QTableWidgetItem *name = m_waitTable->item(r, kWaitName);
        if (!name) {
            name = new QTableWidgetItem();
            m_waitTable->setItem(r, kWaitName, name);
        }
        name->setText(QString::fromStdString(w.name));
        name->setFlags(m_locked ? Qt::ItemIsEnabled | Qt::ItemIsSelectable : Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsEditable);
        // The input list: the board's inputs (one taken by something else greyed), the project's CCIO-8s, the PLC's bits.
        auto *input = qobject_cast<QComboBox *>(m_waitTable->cellWidget(r, kWaitInput));
        input->clear();
        auto *model = qobject_cast<QStandardItemModel *>(input->model());
        for (int p = 0; p < kTriggerInputs; ++p) {
            input->addItem(QString::fromStdString(BoardInputName(p)), p);
            const bool taken = !BoardInputBlocked(m_project, p, m_estopPinKnown).empty() || BoardInputAt(m_project, p).role != CcioRole::None ||
                               OpAt(p) >= 0;
            if (taken && model && !(w.source == WaitSource::Board && w.pin == p)) {
                model->item(input->count() - 1)->setFlags(model->item(input->count() - 1)->flags() & ~Qt::ItemIsEnabled);
            }
        }
        for (int b = 0; b < m_project.ccio.boards; ++b) {
            for (int p = 0; p < kCcioPinsPerBoard; ++p) {
                input->addItem(QString::fromStdString(CcioPinName(b, p)), 100 + b * kCcioPinsPerBoard + p);
                if (model && CcioInputAt(m_project.ccio, b, p).role != CcioRole::None && !(w.source == WaitSource::Ccio && w.board == b && w.pin == p)) {
                    model->item(input->count() - 1)->setFlags(model->item(input->count() - 1)->flags() & ~Qt::ItemIsEnabled);
                }
            }
        }
        for (int bit = 0; bit < kPlcWaitBits; ++bit) input->addItem(QStringLiteral("PLC bit %1").arg(bit), 200 + bit);
        const int code = w.source == WaitSource::Plc ? 200 + w.pin : w.source == WaitSource::Ccio ? 100 + w.board * kCcioPinsPerBoard + w.pin : w.pin;
        int at = input->findData(code);
        if (at < 0) {  // a CCIO-8 the project no longer has: keep it shown, the review flags it
            input->addItem(QString::fromStdString(WaitInputName(w)), code);
            at = input->count() - 1;
        }
        input->setCurrentIndex(at);
        input->setEnabled(!m_locked);
        auto *level = qobject_cast<QComboBox *>(m_waitTable->cellWidget(r, kWaitLevel));
        level->setCurrentIndex(w.on ? 0 : 1);
        level->setEnabled(!m_locked);
        if (QWidget *cell = m_waitTable->cellWidget(r, kWaitChange)) {
            if (auto *change = cell->findChild<QCheckBox *>()) {
                change->setChecked(w.edge);
                change->setEnabled(!m_locked);
            }
        }
        auto *timeout = qobject_cast<QDoubleSpinBox *>(m_waitTable->cellWidget(r, kWaitTimeout));
        timeout->setValue(w.timeout);
        timeout->setEnabled(!m_locked);
    }
    if (keep >= 0 && keep < n) m_waitTable->selectRow(keep);
    m_waitsInfo->setText(!UsesTimeline(m_project) ? QStringLiteral("The project has no timeline axes.")
                         : n ? QStringLiteral("The cycle is at least %1 s: each wait adds the time it holds.").arg(cycle, 0, 'g', 6)
                             : QString());
    m_updating = was;
}

QWidget *MainWindow::BuildOutputsPanel(QWidget *parent) {
    m_outputsGroup = new CollapsibleGroup(QStringLiteral("Outputs"), QStringLiteral("outputs"), parent);
    auto *v = new QVBoxLayout(m_outputsGroup->body());
    v->setContentsMargins(14, 18, 14, 12);
    v->setSpacing(8);
    m_outputTable = new QTableWidget(0, 4, m_outputsGroup);
    m_outputTable->setHorizontalHeaderLabels(
        {QStringLiteral("Output"), QStringLiteral("Master"), QStringLiteral("On"), QStringLiteral("Off")});
    m_outputTable->verticalHeader()->setVisible(false);
    m_outputTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    // The two drop-downs get their width; on and off share the rest.
    m_outputTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    m_outputTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
    m_outputTable->horizontalHeader()->resizeSection(0, 86);
    m_outputTable->horizontalHeader()->resizeSection(1, 96);
    m_outputTable->horizontalHeader()->setMinimumSectionSize(36);
    m_outputTable->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_outputTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_outputTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_outputTable->setMinimumHeight(110);
    // Take the column's width instead of asking for the columns' (the Settings column does not scroll
    // sideways, so a wider table would be cut off).
    m_outputTable->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    m_outputTable->setMinimumWidth(160);
    m_outputTable->setToolTip(QStringLiteral(
        "IO-0..IO-5 switched by pulses for the whole project, not one axis. A crank pulse is in degrees and\n"
        "repeats every revolution; a timeline pulse is in seconds from the trigger.\n"
        "Double-click On / Off to type it. In the displacement diagram's Outputs band: drag a pulse or its\n"
        "ends, double-click a lane to add one, right-click to delete."));
    auto *buttons = new QHBoxLayout();
    auto *add = m_addPulse = new QPushButton(QStringLiteral("Add"), m_outputsGroup);
    add->setToolTip(QStringLiteral("Add a pulse at the cursor on this axis's master (10\u00b0 on the crank, 0.1 s on the timeline)."));
    m_deletePulse = new QPushButton(QStringLiteral("Delete"), m_outputsGroup);
    buttons->addWidget(add);
    buttons->addWidget(m_deletePulse);
    buttons->addStretch(1);
    m_outputsInfo = new QLabel(m_outputsGroup);
    m_outputsInfo->setWordWrap(true);
    m_outputsInfo->setTextFormat(Qt::RichText);
    m_outputsInfo->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));
    v->addWidget(m_outputTable);
    v->addLayout(buttons);
    v->addWidget(m_outputsInfo);

    connect(add, &QPushButton::clicked, this, [this]() {
        if (static_cast<int>(m_project.outputs.size()) >= kMaxOutputPulses) {
            statusBar()->showMessage(QStringLiteral("At most %1 output pulses.").arg(kMaxOutputPulses), 4000);
            return;
        }
        // The first output not yet used and not the trigger input, else the first that isn't the trigger input.
        const CamMaster &tm = m_project.master;
        const int trig = tm.trigger == TriggerSource::DigitalInput && UsesTimeline(m_project) ? tm.triggerInput : -1;
        int output = trig == 0 ? 1 : 0;
        for (int n = kOutputs - 1; n >= 0; --n) {
            const BoardInput in = BoardInputAt(m_project, n);
            bool used = n == trig || n == m_estopPinKnown || !in.name.empty() || in.role != CcioRole::None || WaitOnPin(m_project, n) || OpAt(n) >= 0;
            for (const OutputPulse &o : m_project.outputs) used = used || o.output == n;
            if (!used) output = n;
        }
        AddPulse(output, OnTimeline() ? m_time : std::fmod(m_angle, 360.0));
    });
    connect(m_deletePulse, &QPushButton::clicked, this, [this]() {
        const int row = m_outputTable->currentRow();
        EditOutputs(QStringLiteral("Delete output pulse"), [&](std::vector<OutputPulse> &v) {
            if (row >= 0 && row < static_cast<int>(v.size())) v.erase(v.begin() + row);
        });
    });
    connect(m_outputTable, &QTableWidget::itemChanged, this, [this](QTableWidgetItem *item) {
        if (m_updating || !item) return;
        bool ok = false;
        QString text = item->text().trimmed();
        text.remove(QChar(0xB0));
        if (text.endsWith(QLatin1Char('s'))) text.chop(1);
        const double val = text.trimmed().toDouble(&ok);
        const int row = item->row();
        if (!ok || row < 0 || row >= static_cast<int>(m_project.outputs.size())) {
            RefreshOutputs();
            return;
        }
        const bool on = item->column() == 2;
        EditOutputs(on ? QStringLiteral("Output on time") : QStringLiteral("Output off time"),
                    [&](std::vector<OutputPulse> &v) {
                        OutputPulse &o = v[row];
                        if (on) {
                            const double width = o.off - o.on;
                            o.on = std::max(0.0, val);
                            const double minW = o.master == AxisMaster::Crank ? kMinPulseDeg : kMinPulseSeconds;
                            if (o.off < o.on + minW) o.off = o.on + width;
                        } else {
                            o.off = val;
                        }
                    });
    });
    connect(m_outputTable, &QTableWidget::itemSelectionChanged, this,
            [this]() { m_deletePulse->setEnabled(!m_locked && m_outputTable->currentRow() >= 0); });
    return m_outputsGroup;
}

void MainWindow::RefreshOutputs() {
    if (!m_outputTable) return;
    const bool was = m_updating;
    m_updating = true;
    const int keep = m_outputTable->currentRow();
    const int n = static_cast<int>(m_project.outputs.size());
    m_outputTable->setRowCount(n);
    for (int r = 0; r < n; ++r) {
        const OutputPulse &o = m_project.outputs[r];
        auto *combo = qobject_cast<QComboBox *>(m_outputTable->cellWidget(r, 0));
        if (!combo) {
            combo = new QComboBox(m_outputTable);
            // Compact, so the four columns fit the Settings column.
            combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
            combo->setMinimumContentsLength(4);
            combo->setIconSize(QSize(10, 10));
            for (int k = 0; k < kOutputs; ++k) {
                // A swatch in the output's color, as in the displacement diagram's Outputs band.
                QPixmap swatch(12, 12);
                swatch.fill(theme::OutputColor(k));
                combo->addItem(QIcon(swatch), QStringLiteral("IO-%1").arg(k));
            }
            connect(combo, &QComboBox::currentIndexChanged, this, [this, combo](int k) {
                if (m_updating || k < 0) return;
                for (int row = 0; row < m_outputTable->rowCount(); ++row) {
                    if (m_outputTable->cellWidget(row, 0) != combo) continue;
                    EditOutputs(QStringLiteral("Output"), [&](std::vector<OutputPulse> &v) {
                        if (row < static_cast<int>(v.size())) v[row].output = k;
                    });
                    break;
                }
            });
            m_outputTable->setCellWidget(r, 0, combo);
        }
        combo->setCurrentIndex(o.output);
        // The timeline's trigger input can't be an output too (the board refuses it); a pulse already on it keeps it
        // selectable, and the review flags it.
        if (auto *model = qobject_cast<QStandardItemModel *>(combo->model())) {
            const CamMaster &tm = m_project.master;
            const int trig = tm.trigger == TriggerSource::DigitalInput && UsesTimeline(m_project) ? tm.triggerInput : -1;
            for (int k = 0; k < kOutputs && k < model->rowCount(); ++k) {
                QStandardItem *item = model->item(k);
                const BoardInput in = BoardInputAt(m_project, k);
                const bool input = !in.name.empty() || in.role != CcioRole::None || WaitOnPin(m_project, k);
                const bool estop = k == m_estopPinKnown || OpAt(k) >= 0;
                const bool blocked = (k == trig || input || estop) && k != o.output;
                item->setFlags(blocked ? item->flags() & ~Qt::ItemIsEnabled : item->flags() | Qt::ItemIsEnabled);
                item->setToolTip(estop ? QStringLiteral("The board's estop input.")
                                 : k == trig ? QStringLiteral("The timeline's trigger input.")
                                 : input   ? QStringLiteral("Set up as an input (Inputs).")
                                           : QString());
            }
        }
        auto *mcombo = qobject_cast<QComboBox *>(m_outputTable->cellWidget(r, 1));
        if (!mcombo) {
            mcombo = new QComboBox(m_outputTable);
            mcombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
            mcombo->setMinimumContentsLength(5);
            mcombo->addItem(QStringLiteral("Crank"), static_cast<int>(AxisMaster::Crank));
            mcombo->addItem(QStringLiteral("Timeline"), static_cast<int>(AxisMaster::Timeline));
            connect(mcombo, &QComboBox::currentIndexChanged, this, [this, mcombo](int k) {
                if (m_updating || k < 0) return;
                for (int row = 0; row < m_outputTable->rowCount(); ++row) {
                    if (m_outputTable->cellWidget(row, 1) != mcombo) continue;
                    const auto master = static_cast<AxisMaster>(mcombo->itemData(k).toInt());
                    EditOutputs(QStringLiteral("Output master"), [&](std::vector<OutputPulse> &v) {
                        if (row >= static_cast<int>(v.size()) || v[row].master == master) return;
                        // Keep the pulse where it is in time at the expected crank speed.
                        OutputPulse &o = v[row];
                        const double degPerS = DegPerSecond(m_project.master);
                        if (master == AxisMaster::Crank && degPerS > 0.0) {
                            o.on = std::fmod(o.on * degPerS, 360.0);
                            o.off = std::min(360.0, o.on + (o.off - o.on) * degPerS);
                        } else if (master == AxisMaster::Timeline && degPerS > 0.0) {
                            o.on /= degPerS;
                            o.off /= degPerS;
                        }
                        o.master = master;
                    });
                    break;
                }
            });
            m_outputTable->setCellWidget(r, 1, mcombo);
        }
        mcombo->setCurrentIndex(o.master == AxisMaster::Crank ? 0 : 1);
        const bool crank = o.master == AxisMaster::Crank;
        for (int c = 2; c <= 3; ++c) {
            const double val = c == 2 ? o.on : o.off;
            const QString text = crank ? QStringLiteral("%1\u00b0").arg(QString::number(val, 'f', 1))
                                       : QStringLiteral("%1 s").arg(QString::number(val, 'f', 3));
            QTableWidgetItem *it = m_outputTable->item(r, c);
            if (!it) {
                it = new QTableWidgetItem;
                it->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
                m_outputTable->setItem(r, c, it);
            }
            it->setText(text);
        }
    }
    if (keep >= 0 && keep < n) m_outputTable->selectRow(keep);
    m_deletePulse->setEnabled(!m_locked && m_outputTable->currentRow() >= 0 && n > 0);
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < 2; ++c) {
            if (QWidget *cw = m_outputTable->cellWidget(r, c)) cw->setEnabled(!m_locked);
        }
    }
    m_outputsGroup->setHeading(n ? QStringLiteral("Outputs  (%1)").arg(n) : QStringLiteral("Outputs"));

    // What the outputs do, and inputs that clash with them.
    QStringList info;
    const CamMaster &m = m_project.master;
    if (m.trigger == TriggerSource::DigitalInput && m.triggerInput < kOutputs) {
        for (const OutputPulse &o : m_project.outputs) {
            if (o.output == m.triggerInput) {
                info << QStringLiteral("<span style='color:#ff8a80;'>IO-%1 is the timeline's trigger input: the board "
                                       "refuses the upload. Pick another output or trigger input.</span>")
                            .arg(m.triggerInput);
                break;
            }
        }
    }
    m_outputsInfo->setText(info.join(QStringLiteral("<br>")));

    // Trigger inputs that cannot be used: the Encoder Adapter's while an encoder is the master, and
    // the IO pins used as outputs.
    if (auto *model = qobject_cast<QStandardItemModel *>(m_triggerInput->model())) {
        const bool encoder = m.source == MasterSource::Encoder && UsesCrank(m_project);
        for (int i = 0; i < kTriggerInputs && i < model->rowCount(); ++i) {
            bool asOutput = false;
            for (const OutputPulse &o : m_project.outputs) asOutput = asOutput || o.output == i;
            const BoardInput in = BoardInputAt(m_project, i);
            const bool asInput = (!in.name.empty() || in.role != CcioRole::None || WaitOnPin(m_project, i)) && i != m.triggerInput;
            const bool asEstop = (i == m_estopPinKnown || OpAt(i) >= 0) && i != m.triggerInput;
            const bool blocked = (encoder && i >= kEncoderInputFirst && i <= kEncoderInputLast) || asOutput || asInput || asEstop;
            QStandardItem *item = model->item(i);
            item->setFlags(blocked ? item->flags() & ~Qt::ItemIsEnabled : item->flags() | Qt::ItemIsEnabled);
            item->setToolTip(encoder && i >= kEncoderInputFirst && i <= kEncoderInputLast
                                 ? QStringLiteral("The Encoder Adapter uses DI-6..DI-8.")
                             : asOutput ? QStringLiteral("Used as a timeline output.")
                             : asInput ? QStringLiteral("Set up as an input (Inputs).")
                             : asEstop ? QStringLiteral("The board's estop input.")

                                                : QString());
        }
    }
    m_updating = was;
}

double MainWindow::SimEncoderCountsPerSecond(double speed) const {
    const CamMaster &m = m_project.master;
    if (m.lineMode) return speed * 1000.0 / 60.0 * m.lineCountsPerMm;  // m/min of line
    return speed / 60.0 * m.countsPerRev;                               // crank rpm
}

void MainWindow::AddPulse(int output, double x) {
    if (static_cast<int>(m_project.outputs.size()) >= kMaxOutputPulses) {
        statusBar()->showMessage(QStringLiteral("At most %1 output pulses.").arg(kMaxOutputPulses), 4000);
        return;
    }
    const bool crank = !OnTimeline();
    EditOutputs(QStringLiteral("Add output pulse"), [&](std::vector<OutputPulse> &v) {
        OutputPulse o;
        o.output = output;
        o.master = crank ? AxisMaster::Crank : AxisMaster::Timeline;
        const double width = crank ? 10.0 : 0.1;
        o.on = std::max(0.0, crank ? std::min(x, 360.0 - width) : x);
        o.off = o.on + width;
        v.push_back(o);
    });
}

void MainWindow::EditAxis(const QString &text, const std::function<void(CamAxis &)> &fn, int mergeId) {
    EditProject(text, [&](CamProject &p) { fn(p.axes[m_axis]); }, mergeId);
}

void MainWindow::OnKnotDragged(int knot, double deg, double pos) {
    CamAxis &a = Axis();
    SetKnotDeg(a, knot, deg);
    SetKnotPos(a, knot, pos);
    NormalizeProject(m_project);
    m_camView->setAxis(Axis());
    m_strip->setAxis(Axis(), m_project.master, ViewSpan());
    m_strip->setOutputs(m_project.outputs);
    {
        // Cut-length mode: shade the sync window on the carriage's diagram.
        int sa = -1;
        double s0 = 0.0, s1 = 0.0;
        const bool sync = m_project.master.lineMode && LineSync(m_project, &sa, &s0, &s1) && Axis().master == AxisMaster::Crank;
        m_strip->setSyncWindow(sync ? s0 : -1.0, sync ? s1 : -1.0);
    }
    m_timing->setProject(m_project, m_axis);
    RefreshTable();
    RefreshStats();
}

void MainWindow::OnTableItemChanged(int row, int col) {
    if (m_updating) return;
    QTableWidgetItem *item = m_table->item(row, col);
    if (!item) return;
    bool ok = false;
    const double v = item->text().trimmed().remove(QChar(0xB0)).toDouble(&ok);
    if (!ok) {
        RefreshTable();
        return;
    }
    switch (col) {
        case kColStartDeg:
            EditAxis(QStringLiteral("Segment start"), [&](CamAxis &a) { SetKnotDeg(a, row, v); });
            break;
        case kColEndDeg:
            EditAxis(QStringLiteral("Segment end"), [&](CamAxis &a) { SetKnotDeg(a, row + 1, v); });
            break;
        case kColSpan:
            EditAxis(QStringLiteral("Segment span"),
                     [&](CamAxis &a) { SetKnotDeg(a, row + 1, SegmentStartDeg(a, row) + v); });
            break;
        case kColStartPos:
            EditAxis(QStringLiteral("Segment start position"), [&](CamAxis &a) { SetKnotPos(a, row, v); });
            break;
        case kColEndPos:
            EditAxis(QStringLiteral("Segment end position"), [&](CamAxis &a) { SetKnotPos(a, row + 1, v); });
            break;
        case kColRise:
            EditAxis(QStringLiteral("Segment rise"),
                     [&](CamAxis &a) { SetKnotPos(a, row + 1, SegmentStartPos(a, row) + v); });
            break;
        default:
            break;
    }
}

void MainWindow::AddAxis(bool duplicate) {
    if (m_project.axes.size() >= static_cast<size_t>(kMotorConnectors)) {
        statusBar()->showMessage(QStringLiteral("The ClearCore has %1 motor connectors.").arg(kMotorConnectors), 4000);
        return;
    }
    CamAxis a = duplicate ? Axis() : DefaultAxis("");
    if (!duplicate && OnTimeline()) {
        // A new timeline axis starts as long as the one being edited.
        a.master = AxisMaster::Timeline;
        ScaleSpan(a, Axis().span);
    }
    a.name = (duplicate ? a.name + " copy" : "Axis " + std::to_string(m_project.axes.size() + 1));
    {
        bool used[kMotorConnectors] = {};
        if (m_project.master.source == MasterSource::DrivenCrank) used[m_project.master.crankConnector] = true;
        for (const CamAxis &x : m_project.axes) used[x.connector] = true;
        for (int i = 0; i < kMotorConnectors; ++i) {
            if (!used[i]) {
                a.connector = i;
                break;
            }
        }
    }
    EditProject(duplicate ? QStringLiteral("Duplicate axis") : QStringLiteral("Add axis"), [&](CamProject &p) {
        p.axes.push_back(a);
        m_axis = static_cast<int>(p.axes.size()) - 1;
    });
    SelectSegment(0);
}

void MainWindow::RemoveAxis() {
    if (m_project.axes.size() <= 1) return;
    const int idx = m_axis;
    EditProject(QStringLiteral("Remove axis"), [&](CamProject &p) {
        p.axes.erase(p.axes.begin() + idx);
        CcioAxisRemoved(p.ccio, idx);
        BoardInputsAxisRemoved(p, idx);
        m_axis = std::min(idx, static_cast<int>(p.axes.size()) - 1);
    });
    SelectSegment(0);
}

/* ------------------------------------------------------------------------- */
/* Refresh                                                                    */
/* ------------------------------------------------------------------------- */

void MainWindow::Refresh() {
    m_updating = true;
    {
        QSignalBlocker b(m_axisCombo);
        m_axisCombo->clear();
        for (const CamAxis &a : m_project.axes) m_axisCombo->addItem(QString::fromStdString(a.name));
        m_axisCombo->setCurrentIndex(m_axis);
    }
    {
        QSignalBlocker b(m_stripAxis);
        m_stripAxis->clear();
        for (const CamAxis &a : m_project.axes) {
            m_stripAxis->addItem(QStringLiteral("%1  ·  M-%2%3")
                                     .arg(QString::fromStdString(a.name))
                                     .arg(a.connector)
                                     .arg(a.master == AxisMaster::Timeline ? QStringLiteral("  ·  timeline") : QString()));
        }
        m_stripAxis->setCurrentIndex(m_axis);
        m_stripAxis->setEnabled(m_project.axes.size() > 1);
    }
    m_removeAxisButton->setEnabled(m_project.axes.size() > 1);
    const CamMaster &m = m_project.master;
    m_crankGroup->setHeading(UsesCrank(m_project) ? QStringLiteral("Crank") : QStringLiteral("Crank  (no axes)"));
    m_timelineGroup->setHeading(UsesTimeline(m_project) ? QStringLiteral("Timeline") : QStringLiteral("Timeline  (not used)"));
    m_repeat->setChecked(m.timelineRepeat);
    if (std::fabs(m_override->value() - m.speedOverride * 100.0) > 1e-9) m_override->setValue(m.speedOverride * 100.0);
    m_triggerSource->setCurrentIndex(static_cast<int>(m.trigger));
    m_triggerInput->setCurrentIndex(m.triggerInput);
    m_triggerEdge->setCurrentIndex(m.triggerRising ? 0 : 1);
    m_triggerBit->setValue(m.triggerBit);
    m_timelineForm->setRowVisible(m_triggerInput, m.trigger == TriggerSource::DigitalInput);
    m_timelineForm->setRowVisible(m_triggerEdge, m.trigger == TriggerSource::DigitalInput);
    // The PLC's TRIGGER is one fixed bit (ControlFlags bit 8): nothing to choose. triggerBit stays in the file.
    m_timelineForm->setRowVisible(m_triggerBit, false);
    {
        const double len = TimelineLength(m_project);
        QString info = len > 0.0 ? QStringLiteral("Length %1 s, runs %2 s at this speed")
                                       .arg(Sig(len), Sig(len / std::max(m.speedOverride, 1e-9)))
                                 : QStringLiteral("Set an axis's master to Timeline, or add a timeline output pulse, to use it.");
        const MasterLimit lim = TimelineOverrideLimit();
        if (std::isfinite(lim.rpm)) {
            const bool over = m.speedOverride > lim.rpm;
            info += QStringLiteral("<br><span style='color:%1;'>Max %2 %</span> <span style='color:#8a929d;'>(%3)</span>")
                        .arg(over ? QStringLiteral("#ff8a80") : QStringLiteral("#7fe3a0"),
                             QString::number(lim.rpm * 100.0, 'f', 1), lim.reason.toHtmlEscaped());
        }
        m_timelineInfo->setText(info);
    }
    const bool driven = m.source == MasterSource::DrivenCrank;
    m_masterSource->setCurrentIndex(driven ? 1 : 0);
    if (auto *model = qobject_cast<QStandardItemModel *>(m_masterSource->model())) {
        // DI-6..DI-8 configured as inputs: the Encoder Adapter can't have them. Still selectable while it is the
        // master (a project from before), where the review flags it.
        QStandardItem *enc = model->item(0);
        const bool taken = BoardInputsOnEncoderPins(m_project);
        enc->setFlags(taken && driven ? enc->flags() & ~Qt::ItemIsEnabled : enc->flags() | Qt::ItemIsEnabled);
        enc->setToolTip(taken ? QStringLiteral("DI-6..DI-8 are set up as inputs (Inputs): the Encoder Adapter needs them.") : QString());
    }
    m_countsLabel->setText(driven ? QStringLiteral("Motor ppr") : QStringLiteral("Encoder counts / rev"));
    m_countsPerRev->setToolTip(driven ? QStringLiteral("Crank motor steps per motor revolution (its input resolution).")
                                      : QStringLiteral("Encoder counts per encoder revolution (quadrature, x4)."));
    m_gearNum->setToolTip(driven ? QStringLiteral("Motor revolutions...") : QStringLiteral("Encoder revolutions..."));
    m_gearDen->setToolTip(QStringLiteral("...per this many crank revolutions (10 : 1 is a 10x reduction; 1 : 1 on the "
                                         "crank shaft). Not a whole ratio? Enter it as a fraction, e.g. 50 : 7."));
    if (m_gearNum->value() != m.crankGearNum) m_gearNum->setValue(m.crankGearNum);
    if (m_gearDen->value() != m.crankGearDen) m_gearDen->setValue(m.crankGearDen);
    const bool line = m.lineMode;
    m_speedLabel->setText(line ? QStringLiteral("Line speed") : driven ? QStringLiteral("Run speed") : QStringLiteral("Expected speed"));
    m_lineMode->setChecked(m.lineMode);
    m_encoderReverse->setChecked(m.encoderReverse);
    m_masterForm->setRowVisible(m_encoderReverse, !driven);
    m_crankReverse->setChecked(m.crankReverse);
    m_masterForm->setRowVisible(m_crankReverse, driven);
    m_masterForm->setRowVisible(m_gearRow, !line);
    m_masterForm->setRowVisible(m_wheel, line);
    m_masterForm->setRowVisible(m_designLength, line);
    m_masterForm->setRowVisible(m_kerf, line);
    if (auto *wheelLabel = qobject_cast<QLabel *>(m_masterForm->labelForField(m_wheel))) {
        wheelLabel->setText(driven ? QStringLiteral("Line per motor rev") : QStringLiteral("Wheel"));
    }
    m_wheel->setToolTip(driven ? QStringLiteral("Line fed by one motor revolution, through its gearbox and feed rolls.")
                               : QStringLiteral("Line travel per encoder revolution (the measuring wheel's circumference)."));
    if (m_wheel->value() != m.wheelCircumference) m_wheel->setValue(m.wheelCircumference);
    if (m_designLength->value() != m.designLength) m_designLength->setValue(m.designLength);
    if (m_kerf->value() != m.kerf) m_kerf->setValue(m.kerf);
    m_rpm->setSuffix(line ? QStringLiteral(" m/min") : QStringLiteral(" rpm"));
    m_rpm->setToolTip(driven ? (line ? QStringLiteral("Line speed the crank motor feeds at, the same at every cut length.")
                                     : QStringLiteral("Crank speed the firmware runs at."))
                             : QStringLiteral("Crank speed for velocity / acceleration figures and the preview."));
    m_masterForm2->setRowVisible(m_crankConnector, driven);
    m_masterForm2->setRowVisible(m_crankRamp, driven);
    m_masterForm2->setRowVisible(m_crankMax, driven);
    if (m_countsPerRev->value() != m.crankPpr) m_countsPerRev->setValue(m.crankPpr);
    {
        const double shown = line ? m.rpm * m.designLength / 1000.0 : m.rpm;  // line speed in m/min
        if (std::fabs(m_rpm->value() - shown) > 1e-9) m_rpm->setValue(shown);
    }
    m_crankConnector->setCurrentIndex(m.crankConnector);
    {
        // Cut-length mode: the profile keeps cuts/min at the design length; these read as the line.
        const double k = line ? m.designLength / 1000.0 : 1.0;
        m_crankRamp->setDecimals(line ? 3 : 1);
        m_crankMax->setDecimals(line ? 3 : 1);
        m_crankRamp->setSuffix(line ? QStringLiteral(" m/min/s") : QStringLiteral(" rpm/s"));
        m_crankMax->setSuffix(line ? QStringLiteral(" m/min") : QStringLiteral(" rpm"));
        m_crankMax->setToolTip(line ? QStringLiteral("Line speed limit of the crank motor.")
                                    : QStringLiteral("Crank motor speed limit, measured at the crank."));
        if (std::fabs(m_crankRamp->value() - m.crankRampRpmPerSec * k) > 1e-9) m_crankRamp->setValue(m.crankRampRpmPerSec * k);
        if (std::fabs(m_crankMax->value() - m.crankMaxRpm * k) > 1e-9) m_crankMax->setValue(m.crankMaxRpm * k);
    }
    {
        const MasterLimit lim = ProjectSpeedLimit();
        // Counts per crank rev, exact as a fraction when the ratio is not whole (the board keeps it exact).
        const double units = CrankUnitsPerRev(m);
        const QString perRev = m.crankGearDen > 1 && std::fmod(units, m.crankGearDen) != 0.0
                                   ? QStringLiteral("%1 (= %2 / %3)").arg(Sig(m.countsPerRev), QString::number(units, 'f', 0))
                                         .arg(m.crankGearDen)
                                   : Sig(m.countsPerRev);
        QString info = QStringLiteral("%1 counts per crank rev  \u00b7  %2 counts/s  \u00b7  %3 \u00b0/s")
                           .arg(perRev, Sig(m.countsPerRev * m.rpm / 60.0), Sig(DegPerSecond(m)));
        if (line) {
            // Cut-length mode: the cycle in line terms, what may stretch, and the shortest piece.
            const double wait = LineWaitDeg(m_project);
            info = QStringLiteral("%1 counts/mm  \u00b7  %2 counts per cut  \u00b7  1\u00b0 = %3 mm  \u00b7  %4 cuts/min")
                       .arg(Sig(m.lineCountsPerMm), Sig(m.countsPerRev), Sig(m.designLength / 360.0), Sig(m.rpm));
            info += QStringLiteral("<br>Piece %1 mm as drawn (cycle %2 mm less %3 mm kerf)")
                        .arg(Sig(m.designLength - m.kerf), Sig(m.designLength), Sig(m.kerf));
            if (UsesCrank(m_project) && wait > 0.0) {
                info += QStringLiteral("<br>Wait 0\u2013%1\u00b0 stretches: shortest piece <b>%2 mm</b>, no longest")
                            .arg(QString::number(wait, 'f', 1), Sig(LineMinLength(m_project)));
            } else if (UsesCrank(m_project)) {
                info += QStringLiteral("<br><span style='color:#ff8a80;'>Every crank axis must start with a dwell (the wait "
                                       "that stretches) before the length can change.</span>");
            }
        }
        if (driven) {
            // The crank motor turns gear-ratio times faster than the crank.
            info += QStringLiteral("<br>Crank motor %1 rpm at the run speed")
                        .arg(QString::number(m.rpm * m.crankGearNum / m.crankGearDen, 'f', 1));
        }
        if (units > kMaxCrankUnits) {
            info += QStringLiteral("<br><span style='color:#ff8a80;'>ppr \u00d7 gear numerator is above %1: the board "
                                   "cannot count that finely. Reduce the fraction or the ppr.</span>")
                        .arg(Sig(kMaxCrankUnits));
        }
        if (std::isfinite(lim.rpm)) {
            const bool over = m.rpm > lim.rpm;
            // Cut-length mode: the limit as a line speed. It does not depend on the cut length (the
            // motion is fixed in line distance), so it holds for every length.
            const QString max = line ? QStringLiteral("Max line speed %1 m/min").arg(QString::number(lim.rpm * m.designLength / 1000.0, 'f', 2))
                                     : QStringLiteral("Max %1 rpm").arg(QString::number(lim.rpm, 'f', 1));
            info += QStringLiteral("<br><span style='color:%1;'>%2</span> <span style='color:#8a929d;'>(%3)</span>")
                        .arg(over ? QStringLiteral("#ff8a80") : QStringLiteral("#7fe3a0"), max, lim.reason.toHtmlEscaped());
        } else {
            info += QStringLiteral("<br><span style='color:#8a929d;'>Set axis limits to get a max crank speed.</span>");
        }
        m_masterInfo->setText(info);
    }
    RefreshAxisFields();
    m_updating = false;

    const bool timeline = OnTimeline();
    m_camDock->setTitle(ShowTiming() ? QStringLiteral("Timing") : QStringLiteral("Cam"));
    if (m_timingAction) {
        // It does something only with both kinds of axis: a timeline-only job shows the Timing view anyway, a crank-only
        // one has nothing on the timeline.
        bool crank = false, timeline = false;
        for (const CamAxis &a : m_project.axes) {
            crank = crank || a.master == AxisMaster::Crank;
            timeline = timeline || a.master == AxisMaster::Timeline;
        }
        const bool both = crank && timeline;
        m_timingAction->setEnabled(both);
        m_timingAction->setToolTip(both ? QStringLiteral("Show the Timing view (all timeline axes) also while a crank axis is selected.")
                                        : QStringLiteral("For a project with crank and timeline axes."));
        if (!both && m_forceTiming) {
            QSignalBlocker b(m_timingAction);
            m_timingAction->setChecked(false);
            m_forceTiming = false;
        }
    }
    m_viewStack->setCurrentWidget(ShowTiming() ? static_cast<QWidget *>(m_timing) : m_camView);
    RefreshOutputs();
    RefreshInputs();
    RefreshWaits();
    RefreshTree();
    RefreshCcio();
    m_camView->setAxis(Axis());
    m_camView->setSelectedSegment(m_seg);
    m_timing->setProject(m_project, m_axis);
    m_timing->setWaitsEditable(!m_locked);
    m_strip->setAxis(Axis(), m_project.master, ViewSpan());
    m_strip->setOutputs(m_project.outputs);
    {
        // Cut-length mode: shade the sync window on the carriage's diagram.
        int sa = -1;
        double s0 = 0.0, s1 = 0.0;
        const bool sync = m_project.master.lineMode && LineSync(m_project, &sa, &s0, &s1) && Axis().master == AxisMaster::Crank;
        m_strip->setSyncWindow(sync ? s0 : -1.0, sync ? s1 : -1.0);
    }
    m_strip->setSelectedSegment(m_seg);
    {
        const bool was = m_updating;
        m_updating = true;
        m_snapLabel->setText(QStringLiteral("Snap"));
        m_snapDeg->setSuffix(timeline ? QStringLiteral(" s") : QStringLiteral("°"));
        m_snapDeg->setValue(timeline ? m_snapTime : m_snapCrank);
        m_strip->setSnap(m_snapDeg->value(), m_snapPos->value());
        m_updating = was;
    }
    RefreshCursor();
    RefreshBoard();
    RefreshTable();
    RefreshStats();
    RefreshTitle();
    ApplyLock();  // the rows just rebuilt
}

void MainWindow::RefreshAxisFields() {
    const CamAxis &a = Axis();
    const QString name = QString::fromStdString(a.name);
    if (m_axisName->text() != name) m_axisName->setText(name);
    const QString units = QString::fromStdString(a.units);
    if (m_units->currentText() != units) m_units->setCurrentText(units);
    if (m_countsPerUnit->value() != a.countsPerUnit) m_countsPerUnit->setValue(a.countsPerUnit);
    // Mechanics: ballscrew / belt derive counts / unit, which is then shown read-only.
    m_axisDrive->setCurrentIndex(static_cast<int>(a.drive));
    const bool derived = a.drive != AxisDrive::Direct;
    // Motor ppr is always shown: ballscrew / belt use it for counts / unit, and every drive uses it
    // for the motor rpm in the analysis.
    m_motorPpr->setToolTip(derived ? QStringLiteral("Motor steps per motor revolution (the drive's input resolution).")
                                   : QStringLiteral("Motor steps per motor revolution. With a direct drive it is only "
                                                    "used for the peak motor rpm in the analysis."));
    m_axisForm2->setRowVisible(m_screwPitch, a.drive == AxisDrive::Ballscrew);
    m_axisForm2->setRowVisible(m_pulleyPd, a.drive == AxisDrive::Belt);
    if (m_motorPpr->value() != a.motorPpr) m_motorPpr->setValue(a.motorPpr);
    if (std::fabs(m_screwPitch->value() - a.screwPitch) > 1e-12) m_screwPitch->setValue(a.screwPitch);
    if (std::fabs(m_pulleyPd->value() - a.pulleyPd) > 1e-12) m_pulleyPd->setValue(a.pulleyPd);
    m_screwPitch->setSuffix(QStringLiteral(" %1/rev").arg(units));
    m_pulleyPd->setSuffix(QStringLiteral(" %1").arg(units));
    m_countsPerUnit->setReadOnly(derived);
    m_countsPerUnit->setButtonSymbols(derived ? QAbstractSpinBox::NoButtons : QAbstractSpinBox::UpDownArrows);
    m_countsPerUnit->setToolTip(derived ? QStringLiteral("Worked out from the drive: %1.")
                                              .arg(a.drive == AxisDrive::Ballscrew
                                                       ? QStringLiteral("motor ppr / screw pitch")
                                                       : QStringLiteral("motor ppr / (π × pulley PD)"))
                                        : QStringLiteral("Motor step counts per unit of slave travel."));
    m_axisConnector->setCurrentIndex(a.connector);
    const bool timeline = a.master == AxisMaster::Timeline;
    m_axisMaster->setCurrentIndex(timeline ? 1 : 0);
    m_axisForm->setRowVisible(m_duration, timeline);
    if (timeline && std::fabs(m_duration->value() - a.span) > 1e-12) m_duration->setValue(a.span);
    m_axisForm->setRowVisible(m_baseRadius, !timeline && !a.gear);
    m_axisForm->setRowVisible(m_rollerRadius, !timeline && !a.gear);
    {
        QSignalBlocker b1(m_axisType), b2(m_axisGearNum), b3(m_axisGearDen);
        m_axisType->setCurrentIndex(a.gear ? 1 : 0);
        m_axisGearNum->setValue(a.gearNum);
        m_axisGearDen->setValue(a.gearDen);
    }
    m_axisForm->setRowVisible(m_axisGearRow, a.gear);
    m_closed->setVisible(!a.gear);
    m_axisMaster->setEnabled(!a.gear);
    m_axisGearHint->setText(QStringLiteral("%1 : %2").arg(units, m_project.master.lineMode ? QStringLiteral("cycles") : QStringLiteral("crank revs")));
    m_closed->setToolTip(timeline ? QStringLiteral("Ends where it started. Off: the axis advances by (end \u2212 start) every cycle.")
                                  : QStringLiteral("Returns to its start every revolution. Off: it advances by (end \u2212 start) every "
                                                   "revolution, e.g. an index feed."));
    if (m_velMax->value() != a.velMax) m_velMax->setValue(a.velMax);
    if (m_accMax->value() != a.accMax) m_accMax->setValue(a.accMax);
    m_closed->setChecked(a.closed);
    m_axisReverse->setChecked(a.reverse);
    if (m_baseRadius->value() != a.baseRadius) m_baseRadius->setValue(a.baseRadius);
    if (m_rollerRadius->value() != a.rollerRadius) m_rollerRadius->setValue(a.rollerRadius);
    const QString u = QStringLiteral(" ") + units;
    m_baseRadius->setSuffix(u);
    m_rollerRadius->setSuffix(u);
    m_velMax->setSuffix(u + QStringLiteral("/s"));
    m_accMax->setSuffix(u + QStringLiteral("/s\u00b2"));
    {
        // Homing: its switch (or none), the settings, editable only with a switch.
        const AxisHome &h = a.home;
        QString sw;
        for (const BoardInput &in : m_project.inputs) {
            if (in.role == CcioRole::Home && in.axis == m_axis) {
                sw = QString::fromStdString(BoardInputName(in.pin)) + (in.normallyOpen ? QStringLiteral(", NO") : QStringLiteral(", NC"));
            }
        }
        for (const CcioInput &in : m_project.ccio.inputs) {
            if (in.role == CcioRole::Home && in.axis == m_axis) {
                sw = QString::fromStdString(CcioPinName(in.board, in.pin)) + (in.normallyOpen ? QStringLiteral(", NO") : QStringLiteral(", NC"));
            }
        }
        const bool has = !sw.isEmpty();
        m_homeSwitch->setText(has ? sw : QStringLiteral("None (Inputs or CCIO-8 page)"));
        m_homeDialog->setWindowTitle(QStringLiteral("Homing: %1").arg(QString::fromStdString(a.name)));
        m_homeSummary->setText(!has ? QStringLiteral("No home switch")
                                    : QStringLiteral("%1, toward %2%3").arg(sw, h.dir > 0 ? QStringLiteral("+") : QStringLiteral("\u2212"),
                                                                           h.required ? QStringLiteral(", required") : QString()));
        QSignalBlocker b1(m_homeDir), b2(m_homeFast), b3(m_homeSlow), b4(m_homeBack), b5(m_homeAt), b6(m_homeSearch), b7(m_homeThen),
            b8(m_homeOrder), b9(m_homeRequired), b10(m_homeDecel);
        const double vmax = AxisJogLimit(a);
        m_homeFast->setMaximum(vmax);
        m_homeSlow->setMaximum(vmax);
        m_homeDir->setCurrentIndex(h.dir > 0 ? 1 : 0);
        m_homeFast->setValue(h.fast);
        m_homeSlow->setValue(h.slow);
        m_homeBack->setValue(h.back);
        m_homeAt->setValue(h.at);
        m_homeSearch->setValue(h.search);
        m_homeDecel->setValue(h.decel);
        m_homeThen->setCurrentIndex(h.thenStart ? 0 : 1);
        m_homeOrder->setValue(h.order);
        m_homeRequired->setChecked(h.required);
        m_homeFast->setSuffix(u + QStringLiteral("/s"));
        m_homeSlow->setSuffix(u + QStringLiteral("/s"));
        m_homeBack->setSuffix(u);
        m_homeAt->setSuffix(u);
        m_homeSearch->setSuffix(u);
        m_homeDecel->setSuffix(u + QStringLiteral("/s\u00b2"));
        for (QWidget *w : std::initializer_list<QWidget *>{m_homeDir, m_homeFast, m_homeSlow, m_homeBack, m_homeAt, m_homeSearch, m_homeThen,
                                                            m_homeOrder, m_homeRequired, m_homeDecel}) {
            w->setEnabled(has);
        }
    }
    ApplyLock();  // a gear's diagram can't be edited
}

void MainWindow::RefreshTable() {
    const bool was = m_updating;
    m_updating = true;
    const CamAxis &a = Axis();
    const int n = static_cast<int>(a.segments.size());
    QSignalBlocker block(m_table);
    m_table->setRowCount(n);
    const QString u = QString::fromStdString(a.units);
    const bool timeline = a.master == AxisMaster::Timeline;
    const QString xu = timeline ? QStringLiteral("s") : QStringLiteral("°");
    m_table->horizontalHeaderItem(kColStartDeg)->setText(QStringLiteral("Start %1").arg(xu));
    m_table->horizontalHeaderItem(kColEndDeg)->setText(QStringLiteral("End %1").arg(xu));
    m_table->horizontalHeaderItem(kColSpan)->setText(QStringLiteral("Span %1").arg(xu));
    m_tableHint->setText(QStringLiteral("Positions in %1, %2.  Double-click a cell to edit.")
                             .arg(u, timeline ? QStringLiteral("times in seconds from t = 0")
                                              : QStringLiteral("angles in crank degrees")));
    const int xDec = timeline ? 3 : 2;

    auto setCell = [&](int r, int c, const QString &text, bool editable) {
        QTableWidgetItem *it = m_table->item(r, c);
        if (!it) {
            it = new QTableWidgetItem;
            m_table->setItem(r, c, it);
        }
        it->setText(text);
        it->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        Qt::ItemFlags f = Qt::ItemIsSelectable | Qt::ItemIsEnabled;
        if (editable) f |= Qt::ItemIsEditable;
        it->setFlags(f);
        it->setForeground(editable ? QColor(0xe6, 0xeb, 0xf3) : QColor(0x8a, 0x92, 0x9d));
    };

    for (int r = 0; r < n; ++r) {
        const CamSegment &s = a.segments[r];
        const double d0 = SegmentStartDeg(a, r), p0 = SegmentStartPos(a, r);
        const bool last = r == n - 1;
        const bool dwell = s.law == MotionLaw::Dwell;
        setCell(r, kColIndex, QString::number(r + 1), false);
        m_table->item(r, kColIndex)->setTextAlignment(Qt::AlignCenter);
        m_table->item(r, kColIndex)->setForeground(theme::SegmentColor(r));
        // On the timeline the last end is the axis's duration, so it can be edited too.
        const bool endEditable = !last || timeline;
        setCell(r, kColStartDeg, Fmt(d0, xDec), r > 0);
        setCell(r, kColEndDeg, Fmt(s.endDeg, xDec), endEditable);
        setCell(r, kColSpan, Fmt(s.endDeg - d0, xDec), endEditable);
        setCell(r, kColStartPos, Fmt(p0, 4), true);
        setCell(r, kColEndPos, Fmt(s.endPos, 4), true);
        setCell(r, kColRise, dwell ? QStringLiteral("—") : Fmt(s.endPos - p0, 4), !dwell);

        auto *combo = qobject_cast<QComboBox *>(m_table->cellWidget(r, kColLaw));
        if (!combo) {
            combo = new QComboBox(m_table);
            for (int l = 0; l < static_cast<int>(MotionLaw::Count); ++l) {
                combo->addItem(QString::fromLatin1(LawName(static_cast<MotionLaw>(l))), l);
            }
            combo->setProperty("row", r);
            connect(combo, &QComboBox::activated, this, [this, combo](int idx) {
                const int row = combo->property("row").toInt();
                const int law = combo->itemData(idx).toInt();
                // Queued: the edit rebuilds the table, which must not happen inside this combo's signal.
                QTimer::singleShot(0, this, [this, row, law]() {
                    SelectSegment(row);
                    EditAxis(QStringLiteral("Change motion law"),
                             [&](CamAxis &ax) { SetSegmentLaw(ax, row, static_cast<MotionLaw>(law)); });
                });
            });
            m_table->setCellWidget(r, kColLaw, combo);
        }
        combo->setProperty("row", r);
        QSignalBlocker cb(combo);
        combo->setCurrentIndex(static_cast<int>(s.law));
    }
    if (m_seg >= 0 && m_seg < n) m_table->selectRow(m_seg);
    m_deleteButton->setEnabled(!m_locked && n > 1);
    if (m_deleteSegAction) m_deleteSegAction->setEnabled(!m_locked && n > 1);
    m_updating = was;
}

void MainWindow::RefreshStats() {
    const CamAxis &a = Axis();
    const AxisStats st = Analyze(a);
    const double w = MasterRate(m_project.master, a);
    const bool timeline = a.master == AxisMaster::Timeline;
    const QString u = QString::fromStdString(a.units).toHtmlEscaped();
    auto row = [](const QString &k, const QString &v, const QString &extra = {}) {
        return QStringLiteral("<tr><td style='color:#9fa6af; padding-right:10px; white-space:nowrap;'>%1</td><td style='white-space:nowrap;'>%2</td>"
                              "<td style='color:#8a929d; padding-left:8px;'>%3</td></tr>")
            .arg(k, v, extra);
    };
    auto at = [&](double x) { return QStringLiteral("@ %1").arg(XText(a, x, timeline ? 3 : 1)); };

    QString html = QStringLiteral("<table cellspacing='0' cellpadding='1'>");
    html += row(QStringLiteral("Stroke"), QStringLiteral("%1 %2").arg(Sig(st.maxPos - st.minPos), u),
                QStringLiteral("%1 … %2").arg(Sig(st.minPos), Sig(st.maxPos)));
    html += row(QStringLiteral("Peak vel"), QStringLiteral("%1 %2/s").arg(Sig(st.maxVel * w), u), at(st.maxVelDeg));
    html += row(QStringLiteral("Peak acc"), QStringLiteral("%1 %2/s²").arg(Sig(st.maxAcc * w * w), u), at(st.maxAccDeg));
    html += row(QStringLiteral("Peak jerk"), QStringLiteral("%1 %2/s³").arg(Sig(st.maxJerk * w * w * w), u),
                at(st.maxJerkDeg));
    // Peak motor speed (sanity check against the motor's rating): steps/s over steps per motor rev.
    const double peakStepsPerSec = st.maxVel * w * a.countsPerUnit;
    html += row(QStringLiteral("Motor peak"),
                QStringLiteral("%1 rpm").arg(QString::number(peakStepsPerSec / std::max(a.motorPpr, 1.0) * 60.0, 'f', 0)),
                QStringLiteral("@ %1, %2 ppr").arg(XText(a, st.maxVelDeg, timeline ? 3 : 1), Sig(a.motorPpr)));
    html += row(QStringLiteral("Step rate"), QStringLiteral("%1 kHz").arg(Sig(peakStepsPerSec / 1000.0)),
                QStringLiteral("motor peak"));
    if (timeline) {
        html += row(QStringLiteral("Duration"), QStringLiteral("%1 s").arg(Sig(a.span)),
                    QStringLiteral("%1 s at %2 %").arg(Sig(a.span / std::max(w, 1e-9)), Sig(w * 100.0)));
        html += row(QStringLiteral("Net move"), a.closed ? QStringLiteral("closed")
                                                         : QStringLiteral("%1 %2").arg(Sig(st.advancePerRev), u));
        const SpeedLimit l = MaxMasterSpeed(a, 0.0, 1440);
        if (std::isfinite(l.maxDegPerSec)) {
            html += row(QStringLiteral("Max speed"), QStringLiteral("%1 %").arg(QString::number(l.maxDegPerSec * 100.0, 'f', 1)),
                        QStringLiteral("override this axis allows"));
        }
    } else {
        html += row(QStringLiteral("Ratio"),
                    QStringLiteral("%1").arg(Sig(st.maxVel * a.countsPerUnit * 360.0 / m_project.master.countsPerRev)),
                    QStringLiteral("peak motor/master counts"));
        const CamMaster &cm = m_project.master;
        if (cm.source == MasterSource::DrivenCrank) {
            html += row(QStringLiteral("Crank motor"),
                        QStringLiteral("%1 rpm").arg(QString::number(cm.rpm * cm.countsPerRev / cm.crankPpr, 'f', 1)),
                        cm.lineMode ? QStringLiteral("at %1 m/min").arg(Sig(cm.rpm * cm.designLength / 1000.0))
                                    : QStringLiteral("at %1 rpm, %2 : %3").arg(Sig(cm.rpm)).arg(cm.crankGearNum).arg(cm.crankGearDen));
        }
        html += row(QStringLiteral("Advance"),
                    a.closed ? QStringLiteral("closed") : QStringLiteral("%1 %2/rev").arg(Sig(st.advancePerRev), u));
        html += row(QStringLiteral("Pressure"), QStringLiteral("%1°").arg(QString::number(st.maxPressureDeg, 'f', 1)),
                    at(st.maxPressureAt));
        html += row(QStringLiteral("Min ρ"), QStringLiteral("%1 %2").arg(Sig(st.minCurvature), u), at(st.minCurvatureAt));
    }
    html += QStringLiteral("</table>");

    // Project-level checks: motor connectors, and crank speed against the axis / crank limits.
    QStringList projectIssues;
    {
        const CamMaster &m = m_project.master;
        QStringList owners[kMotorConnectors];
        if (m.source == MasterSource::DrivenCrank) owners[m.crankConnector] << QStringLiteral("crank");
        for (const CamAxis &x : m_project.axes) owners[x.connector] << QString::fromStdString(x.name).toHtmlEscaped();
        for (int i = 0; i < kMotorConnectors; ++i) {
            if (owners[i].size() > 1) {
                projectIssues << QStringLiteral("M-%1 is assigned to %2").arg(i).arg(owners[i].join(QStringLiteral(" and ")));
            }
        }
        const MasterLimit lim = ProjectSpeedLimit();
        if (UsesCrank(m_project) && m.rpm > lim.rpm) {
            projectIssues << QStringLiteral("%1 %2 rpm is above the max %3 rpm (%4)")
                                 .arg(m.source == MasterSource::DrivenCrank ? QStringLiteral("Run speed")
                                                                            : QStringLiteral("Expected speed"),
                                      QString::number(m.rpm, 'f', 1), QString::number(lim.rpm, 'f', 1),
                                      lim.reason.toHtmlEscaped());
        }
        const MasterLimit tlim = TimelineOverrideLimit();
        if (UsesTimeline(m_project) && m.speedOverride > tlim.rpm) {
            projectIssues << QStringLiteral("Timeline speed %1 % is above the max %2 % (%3)")
                                 .arg(QString::number(m.speedOverride * 100.0, 'f', 1),
                                      QString::number(tlim.rpm * 100.0, 'f', 1), tlim.reason.toHtmlEscaped());
        }
        if (a.velMax > 0.0 && st.maxVel * w > a.velMax * (1 + 1e-9)) {
            projectIssues << QStringLiteral("Peak velocity %1 > limit %2 %3/s").arg(Sig(st.maxVel * w), Sig(a.velMax), u);
        }
        if (a.accMax > 0.0) {
            // Worst follower acceleration at run speed while the crank ramps (driven) or at steady speed.
            const double alpha = AxisRamp(m, a);
            double worst = 0.0;
            for (int i = 0; i < 1440; ++i) {
                const CamPoint p = Evaluate(a, a.span * i / 1440);
                worst = std::max(worst, std::fabs(p.acc) * w * w + std::fabs(p.vel) * alpha);
            }
            if (worst > a.accMax * (1 + 1e-9)) {
                projectIssues << QStringLiteral("Peak acceleration %1 > limit %2 %3/s\u00b2%4")
                                     .arg(Sig(worst), Sig(a.accMax), u,
                                          alpha > 0 ? QStringLiteral(" (with crank ramp)") : QString());
            }
        }
    }
    if (!projectIssues.isEmpty()) {
        html += QStringLiteral("<p style='margin-top:8px; margin-bottom:2px; color:#c9d1dc;'>Limits</p>");
        for (const QString &pi : projectIssues) html += QStringLiteral("<div style='color:#ff8a80;'>\u26a0 %1</div>").arg(pi);
    }

    if (st.issues.empty() && projectIssues.isEmpty()) {
        html += QStringLiteral("<p style='color:#7fe3a0; margin-top:8px;'>✓ Smooth: velocity and acceleration are "
                               "continuous at every knot.</p>");
    } else if (!st.issues.empty()) {
        html += QStringLiteral("<p style='margin-top:8px; margin-bottom:2px; color:#c9d1dc;'>Profile</p>");
        for (const CamIssue &is : st.issues) {
            QString text;
            QString color = QStringLiteral("#ff8a80");
            switch (is.kind) {
                case IssueKind::VelocityStep:
                    text = QStringLiteral("Velocity step at %1 (%2 → %3 %4/s): infinite acceleration")
                               .arg(XText(a, is.deg, timeline ? 3 : 1), Sig(is.left * w), Sig(is.right * w), u);
                    break;
                case IssueKind::AccelStep:
                    color = QStringLiteral("#ffcc80");
                    text = QStringLiteral("Acceleration step at %1 (%2 → %3 %4/s²): infinite jerk")
                               .arg(XText(a, is.deg, timeline ? 3 : 1), Sig(is.left * w * w), Sig(is.right * w * w), u);
                    break;
                case IssueKind::PressureAngle:
                    color = QStringLiteral("#ffcc80");
                    text = QStringLiteral("Pressure angle %1° > %2° (cam drawing)")
                               .arg(QString::number(is.left, 'f', 1), QString::number(kMaxPressureAngleDeg, 'f', 0));
                    break;
                case IssueKind::Undercut:
                    color = QStringLiteral("#ffcc80");
                    text = QStringLiteral("Undercut near %1°: pitch radius %2 < roller (cam drawing)")
                               .arg(QString::number(is.deg, 'f', 1), Sig(is.left));
                    break;
                case IssueKind::SplineOvershoot:
                    color = QStringLiteral("#ffcc80");
                    text = QStringLiteral("Spline overshoot at %1: reaches %2, past the knots' %3 %4")
                               .arg(XText(a, is.deg, timeline ? 3 : 1), Sig(is.left), Sig(is.right), u);
                    break;
            }
            html += QStringLiteral("<div style='color:%1;'>⚠ %2</div>").arg(color, text);
        }
    }
    m_stats->setText(html);
}

MainWindow::MasterLimit MainWindow::ProjectSpeedLimit() const {
    const CamMaster &m = m_project.master;
    MasterLimit out{std::numeric_limits<double>::infinity(), QString()};
    if (m.source == MasterSource::DrivenCrank) {
        out.rpm = m.crankMaxRpm;
        out.reason = QStringLiteral("crank motor");
    }
    for (const CamAxis &a : m_project.axes) {
        if (a.master != AxisMaster::Crank) continue;
        const SpeedLimit l = MaxMasterSpeed(a, RampDegPerSecond2(m), 1440);
        const double rpm = l.maxDegPerSec / 6.0;
        if (rpm >= out.rpm) continue;
        out.rpm = rpm;
        const QString name = QString::fromStdString(a.name);
        switch (l.reason) {
            case SpeedLimit::Velocity:
                out.reason = QStringLiteral("%1 velocity @ %2\u00b0").arg(name, QString::number(l.atDeg, 'f', 0));
                break;
            case SpeedLimit::Accel:
                out.reason = QStringLiteral("%1 acceleration @ %2\u00b0").arg(name, QString::number(l.atDeg, 'f', 0));
                break;
            case SpeedLimit::Ramp:
                out.reason = QStringLiteral("%1: ramp too steep, max %2 rpm/s").arg(name, Sig(l.maxRampDegPerSec2 / 6.0));
                break;
            case SpeedLimit::None:
                break;
        }
    }
    return out;
}

MainWindow::MasterLimit MainWindow::TimelineOverrideLimit() const {
    MasterLimit out{std::numeric_limits<double>::infinity(), QString()};
    for (const CamAxis &a : m_project.axes) {
        if (a.master != AxisMaster::Timeline) continue;
        const SpeedLimit l = MaxMasterSpeed(a, 0.0, 1440);  // x is seconds, so the rate is the override
        if (l.maxDegPerSec >= out.rpm) continue;
        out.rpm = l.maxDegPerSec;
        out.reason = QStringLiteral("%1 %2 @ %3 s")
                         .arg(QString::fromStdString(a.name),
                              l.reason == SpeedLimit::Velocity ? QStringLiteral("velocity") : QStringLiteral("acceleration"),
                              QString::number(l.atDeg, 'f', 3));
    }
    return out;
}

QString MainWindow::XText(const CamAxis &a, double x, int decimals) const {
    return a.master == AxisMaster::Timeline ? QStringLiteral("%1 s").arg(QString::number(x, 'f', decimals))
                                            : QStringLiteral("%1°").arg(QString::number(x, 'f', decimals));
}

double MainWindow::ViewSpan() const { return OnTimeline() ? std::max(TimelineLength(m_project), Axis().span) : 360.0; }

void MainWindow::RefreshTitle() {
    const QString name = m_path.isEmpty() ? QStringLiteral("Untitled") : QFileInfo(m_path).fileName();
    if (m_treeRoot) m_treeRoot->setText(0, m_path.isEmpty() ? QStringLiteral("Untitled") : QFileInfo(m_path).completeBaseName());
    setWindowTitle(QStringLiteral(ECAM_PRODUCT_NAME " — %1%2").arg(name, m_undo->isClean() ? QString() : QStringLiteral("*")));
}

void MainWindow::SelectSegment(int seg) {
    seg = std::clamp(seg, 0, static_cast<int>(Axis().segments.size()) - 1);
    m_seg = seg;
    m_camView->setSelectedSegment(seg);
    m_strip->setSelectedSegment(seg);
    const bool was = m_updating;
    m_updating = true;
    if (m_table->currentRow() != seg) m_table->selectRow(seg);
    m_updating = was;
}

void MainWindow::SetAngle(double deg) {
    if (m_jogging && UsesCrank(m_project)) {
        // Unwrap: the shortest way from the last angle, so turning the cam round adds revolutions.
        double d = std::fmod(deg - m_angle, 360.0);
        if (d > 180.0) d -= 360.0;
        if (d < -180.0) d += 360.0;
        m_jogCrankDeg += d;
        SendJog(QStringLiteral("jog crank %1").arg(m_jogCrankDeg, 0, 'f', 4));
    }
    m_angle = std::fmod(std::fmod(deg, 360.0) + 360.0, 360.0);
    RefreshCursor();
}

void MainWindow::SetTime(double t) {
    m_time = t;
    if (m_jogging && UsesTimeline(m_project)) SendJog(QStringLiteral("jog time %1").arg(std::max(0.0, t), 0, 'f', 5));
    RefreshCursor();
}

void MainWindow::SendJog(const QString &cmd) {
    // One target in flight; a fast drag sends only the latest when the board has answered.
    if (m_jogInFlight) {
        m_jogPending = cmd;
        return;
    }
    m_jogInFlight = true;
    m_board->command(cmd, [this](bool ok, const QStringList &r) {
        m_jogInFlight = false;
        if (!ok) {
            statusBar()->showMessage(QStringLiteral("jog: %1").arg(r.isEmpty() ? QString() : r.last()), 4000);
        }
        if (!m_jogPending.isEmpty() && m_jogging) {
            const QString next = m_jogPending;
            m_jogPending.clear();
            SendJog(next);
        }
    });
}

void MainWindow::SetJogging(bool on) {
    m_jogPending.clear();
    if (on == m_jogging) {
        m_boardJog->setChecked(on);
        return;
    }
    m_jogging = on;
    m_boardJog->setChecked(on);
    if (on) {
        // Start from where the board's master is: the cursor jumps there, drags move it from there.
        const BoardLink::State &s = m_board->state();
        m_jogClock.restart();
        m_jogCrankDeg = s.rev * 360.0 + s.deg;
        m_angle = s.deg;
        m_time = s.t;
        if (m_playButton->isChecked()) m_playButton->click();
        RefreshCursor();
        statusBar()->showMessage(QStringLiteral("Hand jog: drag the cam or scrub the diagram to move the motors."));
    } else {
        statusBar()->showMessage(QStringLiteral("Hand jog off"), 3000);
    }
    m_playButton->setEnabled(!on);
    RefreshBoard();
}

void MainWindow::RefreshCursor() {
    m_camView->setAngle(m_angle);
    m_timing->setTime(m_time);
    m_strip->setCursorDeg(OnTimeline() ? m_time : std::fmod(m_angle, 360.0));
}

void MainWindow::TogglePlay(bool on) {
    m_playButton->setText(on ? QStringLiteral("\u275a\u275a  Stop") : QStringLiteral("\u25b6  Run"));
    // Crank: an encoder master is not ours to ramp; a driven crank ramps like the firmware will.
    m_animTarget = on && UsesCrank(m_project) ? DegPerSecond(m_project.master) : 0.0;
    if (m_project.master.source != MasterSource::DrivenCrank) m_animSpeed = m_animTarget;
    // Timeline: Run is the trigger. A one-shot that already finished starts again from t = 0.
    m_timeRunning = on && UsesTimeline(m_project);
    if (m_timeRunning && !m_project.master.timelineRepeat && m_time >= TimelineLength(m_project)) m_time = 0.0;
    if (on && !m_anim->isActive()) {
        m_animClock.restart();
        m_anim->start();
    }
}

void MainWindow::AnimTick() {
    const double dt = m_animClock.restart() / 1000.0;
    const CamMaster &m = m_project.master;
    QStringList status;

    // Crank
    if (m_playButton->isChecked()) m_animTarget = UsesCrank(m_project) ? DegPerSecond(m) : 0.0;  // follow edits
    if (m.source == MasterSource::DrivenCrank) {
        const double step = RampDegPerSecond2(m) * dt;
        m_animSpeed = m_animSpeed < m_animTarget ? std::min(m_animTarget, m_animSpeed + step)
                                                 : std::max(m_animTarget, m_animSpeed - step);
        if (UsesCrank(m_project)) status << QStringLiteral("Crank %1 rpm").arg(QString::number(m_animSpeed / 6.0, 'f', 1));
    } else {
        m_animSpeed = m_animTarget;
    }
    m_angle = std::fmod(m_angle + m_animSpeed * dt, 360.0);

    // Timeline
    if (m_timeRunning) {
        const double len = TimelineLength(m_project);
        m_time += m.speedOverride * dt;
        if (m.timelineRepeat && len > 0.0) {
            m_time = std::fmod(m_time, len);
        } else if (m_time >= len) {
            m_time = len;
            m_timeRunning = false;
        }
        status << QStringLiteral("t %1 s").arg(QString::number(m_time, 'f', 2));
    }
    RefreshCursor();
    if (!status.isEmpty()) statusBar()->showMessage(status.join(QStringLiteral("   ")));

    const bool crankMoving = m_animSpeed > 0.0 || m_animTarget > 0.0;
    if (!crankMoving && !m_timeRunning) {
        m_anim->stop();
        if (m_playButton->isChecked()) {
            QSignalBlocker b(m_playButton);  // a one-shot timeline finished: the button pops back up
            m_playButton->setChecked(false);
            m_playButton->setText(QStringLiteral("\u25b6  Run"));
        }
        statusBar()->showMessage(QStringLiteral("Stopped"), 2000);
    }
}

/* ------------------------------------------------------------------------- */
/* Files                                                                      */
/* ------------------------------------------------------------------------- */

void MainWindow::DiscardChangesForDemo() { m_undo->setClean(); }

bool MainWindow::MaybeSave() {
    if (m_undo->isClean()) return true;
    const auto r = QMessageBox::question(this, QStringLiteral(ECAM_PRODUCT_NAME), QStringLiteral("Save changes to the cam project?"),
                                         QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    if (r == QMessageBox::Save) return Save();
    return r == QMessageBox::Discard;
}

void MainWindow::SetLocked(bool locked) {
    if (m_locked != locked) {
        m_locked = locked;
        statusBar()->showMessage(locked ? QStringLiteral("Profile locked.") : QStringLiteral("Profile unlocked: editing."), 3000);
    }
    ApplyLock();
}

void MainWindow::ApplyLock() {
    if (m_lockButton) {
        QSignalBlocker b(m_lockButton);
        m_lockButton->setChecked(m_locked);
        m_lockButton->setIcon(QIcon(m_locked ? QStringLiteral(":/icons/lock.svg") : QStringLiteral(":/icons/unlock.svg")));
        m_lockButton->setText(m_locked ? QStringLiteral(" Locked") : QStringLiteral(" Unlocked"));
        m_lockButton->setToolTip(m_locked ? QStringLiteral("The motion and the output pulses can't be edited (scrub, zoom and "
                                                           "Run still work). Click or Ctrl+L to unlock.")
                                          : QStringLiteral("Editing. Click or Ctrl+L to lock against accidental edits."));
    }
    if (m_lockAction) {
        QSignalBlocker b(m_lockAction);
        m_lockAction->setChecked(m_locked);
    }
    // An electronic gear's one segment is its ratio: the diagram and the Segments table only show it.
    const bool gear = Axis().gear;
    if (m_strip) m_strip->setLocked(m_locked || gear);
    for (QTableWidget *t : {m_table, m_outputTable}) {
        if (!t) continue;
        const bool fixed = m_locked || (t == m_table && gear);
        // Each table's own edit triggers, kept the first time it is locked.
        if (!t->property("ecamEditTriggers").isValid()) t->setProperty("ecamEditTriggers", static_cast<int>(t->editTriggers()));
        t->setEditTriggers(fixed ? QAbstractItemView::NoEditTriggers
                                 : QAbstractItemView::EditTriggers(t->property("ecamEditTriggers").toInt()));
        // The drop-downs in the rows (law; output and master).
        for (int r = 0; r < t->rowCount(); ++r) {
            for (int c = 0; c < t->columnCount(); ++c) {
                if (QWidget *w = t->cellWidget(r, c)) w->setEnabled(!fixed);
            }
        }
    }
    if (m_splitButton) m_splitButton->setEnabled(!m_locked);
    if (m_splitSegAction) m_splitSegAction->setEnabled(!m_locked && !gear);
    if (m_splitButton) m_splitButton->setEnabled(!m_locked && !gear);
    if (m_importPointsAction) m_importPointsAction->setEnabled(!m_locked && !gear);
    const int segs = static_cast<int>(Axis().segments.size());
    if (m_deleteButton) m_deleteButton->setEnabled(!m_locked && segs > 1);
    if (m_deleteSegAction) m_deleteSegAction->setEnabled(!m_locked && segs > 1);
    if (m_addPulse) m_addPulse->setEnabled(!m_locked);
    if (m_timing) m_timing->setWaitsEditable(!m_locked);  // waits are dragged like pulses
    RefreshCcio();
    RefreshInputs();
    RefreshWaits();
    if (m_deletePulse) m_deletePulse->setEnabled(!m_locked && m_outputTable && m_outputTable->currentRow() >= 0);
}

void MainWindow::NewProject() {
    if (!MaybeSave()) return;
    m_project = DefaultProject();
    m_axis = 0;
    m_seg = 0;
    m_path.clear();
    m_undo->clear();
    SetLocked(false);  // a new project is there to be edited
    Refresh();
    SelectSegment(0);
}

bool MainWindow::OpenProject() {
    if (!MaybeSave()) return false;
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("Open cam project"),
                                                      AppSettings().value("files/lastDir").toString(),
                                                      QStringLiteral("ECam project (*.ecam);;All files (*)"));
    return !path.isEmpty() && OpenFile(path);
}

void MainWindow::UseProject(const CamProject &project, const QString &message, bool modified, bool locked) {
    m_project = project;
    m_axis = 0;
    m_seg = 0;
    m_path.clear();
    m_undo->clear();
    SetLocked(locked);
    if (modified) m_undo->resetClean();  // not saved anywhere yet
    Refresh();
    SelectSegment(0);
    RefreshTitle();
    if (!message.isEmpty()) statusBar()->showMessage(message, 10000);
}

void MainWindow::OpenExample(const QString &resource, const QString &title) {
    if (!MaybeSave()) return;
    CamProject p;
    QString err;
    if (!io::LoadProject(resource, &p, &err)) {
        QMessageBox::warning(this, QStringLiteral("Open example"), QStringLiteral("Could not open the example:\n%1").arg(err));
        return;
    }
    UseProject(p, QStringLiteral("Opened the example \"%1\" (locked: Ctrl+L to edit). Save it to keep your own copy.").arg(title),
               false, true);
}

bool MainWindow::RunSetupWizard() {
    if (!MaybeSave()) return false;
    SetupWizard wizard(this);
    if (wizard.exec() != QDialog::Accepted) return false;
    UseProject(wizard.project(),
               QStringLiteral("Created from the %1 template. Save it, then Upload it from the board strip.").arg(wizard.templateName()),
               true);
    return true;
}

void MainWindow::ShowStartPage() {
    // Cancelling the wizard or the open dialog comes back here.
    for (;;) {
        startup::StartDialog dlg(this);
        if (dlg.exec() != QDialog::Accepted) return;
        switch (dlg.choice()) {
            case startup::StartDialog::Choice::New: NewProject(); return;
            case startup::StartDialog::Choice::Wizard:
                if (RunSetupWizard()) return;
                break;
            case startup::StartDialog::Choice::Open:
                if (OpenProject()) return;
                break;
            case startup::StartDialog::Choice::Recent:
                if (MaybeSave() && OpenFile(dlg.path())) return;
                break;
            case startup::StartDialog::Choice::Example: OpenExample(dlg.path(), dlg.title()); return;
            case startup::StartDialog::Choice::None: return;
        }
    }
}

void MainWindow::BoardDownload() {
    m_boardBusy = true;
    RefreshBoard();
    statusBar()->showMessage(QStringLiteral("Downloading..."));
    m_board->downloadProfile([this](bool ok, const CamProject &p, bool exact, const QString &msg) {
        m_boardBusy = false;
        RefreshBoard();
        if (!ok) {
            statusBar()->showMessage(QStringLiteral("Download failed: %1").arg(msg), 8000);
            return;
        }
        // Same as the open project (the hash covers the project copy too): nothing to do.
        if (exact && msg == QStringLiteral("%1").arg(ProjectHash(), 8, 16, QChar('0'))) {
            statusBar()->showMessage(QStringLiteral("The board's profile is this project (hash %1).").arg(msg), 6000);
            return;
        }
        if (!MaybeSave()) return;
        m_project = p;
        m_axis = 0;
        m_seg = 0;
        m_path.clear();
        m_undo->clear();
        m_undo->resetClean();  // not saved anywhere yet
        SetLocked(true);       // what the board runs: look before changing it
        Refresh();
        SelectSegment(0);
        RefreshTitle();
        if (exact) {
            statusBar()->showMessage(QStringLiteral("Opened the board's project (hash %1). Save it to keep a copy.").arg(msg), 8000);
        } else {
            QMessageBox::information(
                this, QStringLiteral("Download"),
                QStringLiteral("The board's profile was uploaded by an older version, without the project copy.\n\n"
                               "The motion is exactly what the board runs, but the cam drawing (base / roller radius), "
                               "the drive mechanics, the crank gear ratio and the velocity / acceleration limits "
                               "are defaults. Upload it again to keep the whole project on the board."));
        }
    });
}

bool MainWindow::OpenFile(const QString &path) {
    CamProject p;
    QString err;
    if (!io::LoadProject(path, &p, &err)) {
        QMessageBox::warning(this, QStringLiteral("Open"), QStringLiteral("Could not open %1:\n%2").arg(path, err));
        startup::RemoveRecentProject(path);
        return false;
    }
    startup::AddRecentProject(path);
    m_project = p;
    m_axis = 0;
    m_seg = 0;
    m_path = path;
    m_undo->clear();
    SetLocked(true);  // an opened project is looked at first: unlock to edit
    AppSettings().setValue("files/lastDir", QFileInfo(path).absolutePath());
    Refresh();
    SelectSegment(0);
    statusBar()->showMessage(QStringLiteral("Opened %1, locked against accidental edits (Ctrl+L to edit).")
                                 .arg(QFileInfo(path).fileName()),
                             8000);
    return true;
}

void MainWindow::ShowAxis(int axis, double angleDeg) {
    m_axis = std::clamp(axis, 0, static_cast<int>(m_project.axes.size()) - 1);
    m_seg = 0;
    Refresh();
    SelectSegment(0);
    SetAngle(angleDeg);
}

bool MainWindow::Save() {
    if (m_path.isEmpty()) return SaveAs();
    QString err;
    if (!io::SaveProject(m_project, m_path, &err)) {
        QMessageBox::warning(this, QStringLiteral("Save"), QStringLiteral("Could not save %1:\n%2").arg(m_path, err));
        return false;
    }
    m_undo->setClean();
    startup::AddRecentProject(m_path);
    RefreshTitle();
    statusBar()->showMessage(QStringLiteral("Saved %1").arg(m_path), 4000);
    return true;
}

bool MainWindow::SaveAs() {
    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("Save cam project"),
                                                m_path.isEmpty() ? AppSettings().value("files/lastDir").toString() : m_path,
                                                QStringLiteral("ECam project (*.ecam)"));
    if (path.isEmpty()) return false;
    if (QFileInfo(path).suffix().isEmpty()) path += QStringLiteral(".ecam");
    m_path = path;
    AppSettings().setValue("files/lastDir", QFileInfo(path).absolutePath());
    return Save();
}

bool MainWindow::ImportPoints(const QString &path, QString *message) {
    if (Axis().gear) {
        *message = QStringLiteral("An electronic gear has no diagram: make the axis a cam profile first.");
        return false;
    }
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        *message = QStringLiteral("Could not read %1:\n%2").arg(path, f.errorString());
        return false;
    }
    const std::string text = f.readAll().toStdString();
    std::vector<CurvePoint> points;
    std::string err;
    if (!ParsePoints(text, &points, &err)) {
        *message = QStringLiteral("%1:\n%2").arg(QFileInfo(path).fileName(), QString::fromStdString(err));
        return false;
    }
    CamAxis fitted = Axis();
    PointsFit info;
    if (!FitAxisToPoints(fitted, points, kMaxSegments, &info, &err)) {
        *message = QString::fromStdString(err);
        return false;
    }
    EditAxis(QStringLiteral("Import points"), [&](CamAxis &a) { a = fitted; });
    AppSettings().setValue("files/lastDir", QFileInfo(path).absolutePath());
    *message = QStringLiteral("%1 points, %2 knots, %3 to %4 as the %5")
                   .arg(info.points)
                   .arg(info.knots)
                   .arg(Sig(info.firstX), Sig(info.lastX),
                        Axis().master == AxisMaster::Crank ? QStringLiteral("whole revolution") : QStringLiteral("duration"));
    if (info.knots < info.points) *message += QStringLiteral(", at most %1 %2 from the points").arg(Sig(info.maxError), QString::fromStdString(Axis().units));
    return true;
}

void MainWindow::Export(bool header) {
    bool ok = false;
    const int intervals = QInputDialog::getInt(this, QStringLiteral("Export"),
                                               QStringLiteral("Intervals per revolution (table has one more entry):"),
                                               AppSettings().value("files/exportIntervals", 360).toInt(), 4, 1000000, 1, &ok);
    if (!ok) return;
    AppSettings().setValue("files/exportIntervals", intervals);
    const QString base = QString::fromStdString(Axis().name).replace(QRegularExpression(QStringLiteral("\\W+")), "_");
    const QString suggested = AppSettings().value("files/lastDir").toString() + "/" + base + (header ? ".h" : ".csv");
    const QString path = QFileDialog::getSaveFileName(
        this, header ? QStringLiteral("Export C header") : QStringLiteral("Export CSV"), suggested,
        header ? QStringLiteral("C header (*.h)") : QStringLiteral("CSV (*.csv)"));
    if (path.isEmpty()) return;
    QString err;
    const bool done = header ? io::ExportHeader(m_project, m_axis, intervals, path, &err)
                             : io::ExportCsv(m_project, m_axis, intervals, path, &err);
    if (!done) {
        QMessageBox::warning(this, QStringLiteral("Export"), QStringLiteral("Could not write %1:\n%2").arg(path, err));
        return;
    }
    statusBar()->showMessage(QStringLiteral("Exported %1").arg(path), 4000);
}

void MainWindow::ExportWorkbook() {
    bool ok = false;
    const int samples = QInputDialog::getInt(this, QStringLiteral("Export workbook"),
                                             QStringLiteral("Samples per cycle (each motion table has one more row):"),
                                             AppSettings().value("files/workbookSamples", 360).toInt(), 36, 20000, 36, &ok);
    if (!ok) return;
    AppSettings().setValue("files/workbookSamples", samples);
    const QString base = m_path.isEmpty() ? QStringLiteral("Untitled") : QFileInfo(m_path).completeBaseName();
    const QString dir = m_path.isEmpty() ? AppSettings().value("files/lastDir").toString() : QFileInfo(m_path).absolutePath();
    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("Export workbook"), dir + "/" + base + ".xlsx",
                                                QStringLiteral("Excel workbook (*.xlsx)"));
    if (path.isEmpty()) return;
    if (QFileInfo(path).suffix().isEmpty()) path += QStringLiteral(".xlsx");
    io::WorkbookOptions o;
    o.samples = samples;
    o.projectName = base;
    o.projectPath = m_path;
    QString err;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    const bool done = io::ExportWorkbook(m_project, o, path, &err);
    QApplication::restoreOverrideCursor();
    if (!done) {
        QMessageBox::warning(this, QStringLiteral("Export workbook"), QStringLiteral("Could not write %1:\n%2").arg(path, err));
        return;
    }
    statusBar()->showMessage(QStringLiteral("Exported %1").arg(path), 5000);
}

void MainWindow::GenerateReport() {
    const QString base = m_path.isEmpty() ? QStringLiteral("Untitled") : QFileInfo(m_path).completeBaseName();
    const QString dir = m_path.isEmpty() ? AppSettings().value("files/lastDir").toString() : QFileInfo(m_path).absolutePath();
    QString path = QFileDialog::getSaveFileName(this, QStringLiteral("Generate report"), dir + "/" + base + " report.pdf",
                                                QStringLiteral("PDF (*.pdf)"));
    if (path.isEmpty()) return;
    if (QFileInfo(path).suffix().isEmpty()) path += QStringLiteral(".pdf");
    io::ReportOptions o;
    o.projectName = base;
    o.projectPath = m_path;
    QString err;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    const bool done = io::ExportReport(m_project, o, path, &err);
    QApplication::restoreOverrideCursor();
    if (!done) {
        QMessageBox::warning(this, QStringLiteral("Generate report"), QStringLiteral("Could not write %1:\n%2").arg(path, err));
        return;
    }
    statusBar()->showMessage(QStringLiteral("Report saved to %1").arg(QDir::toNativeSeparators(path)), 8000);
    QDesktopServices::openUrl(QUrl::fromLocalFile(path));
}

void MainWindow::ExportCamDxf() {
    const CamAxis &a = Axis();
    if (a.master != AxisMaster::Crank) {
        QMessageBox::information(this, QStringLiteral("Export cam DXF"),
                                 QStringLiteral("\"%1\" runs on the timeline; only crank axes have a cam shape.")
                                     .arg(QString::fromStdString(a.name)));
        return;
    }
    const AxisStats st = Analyze(a);
    for (const CamIssue &is : st.issues) {
        if (is.kind == IssueKind::Undercut) {
            const auto r = QMessageBox::warning(
                this, QStringLiteral("Export cam DXF"),
                QStringLiteral("The cam is undercut near %1° (the roller is larger than the pitch curve's radius of "
                               "curvature there), so its outline loops over itself and cannot be cut as drawn.\n\n"
                               "Increase the base radius or reduce the roller radius. Export anyway?")
                    .arg(QString::number(is.deg, 'f', 0)),
                QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel);
            if (r != QMessageBox::Yes) return;
            break;
        }
    }

    // Options
    QDialog dlg(this);
    dlg.setWindowTitle(QStringLiteral("Export cam DXF"));
    auto *form = new QFormLayout(&dlg);
    const QString u = QString::fromStdString(a.units);
    auto *points = new QSpinBox(&dlg);
    points->setRange(360, 36000);
    points->setSingleStep(360);
    points->setValue(AppSettings().value("dxf/points", 3600).toInt());
    points->setToolTip(QStringLiteral("Vertices of the cam outline per revolution."));
    auto *bore = new QDoubleSpinBox(&dlg);
    bore->setRange(0, 1e6);
    bore->setDecimals(3);
    bore->setSuffix(QStringLiteral(" ") + u);
    bore->setSpecialValueText(QStringLiteral("none"));
    bore->setValue(AppSettings().value("dxf/bore", 0.0).toDouble());
    auto *pitch = new QCheckBox(QStringLiteral("Pitch curve (roller centre path)"), &dlg);
    pitch->setChecked(AppSettings().value("dxf/pitch", true).toBool());
    pitch->setEnabled(a.rollerRadius > 0.0);
    auto *base = new QCheckBox(QStringLiteral("Base circle"), &dlg);
    base->setChecked(AppSettings().value("dxf/base", true).toBool());
    auto *marks = new QCheckBox(QStringLiteral("Centre cross and 0° mark"), &dlg);
    marks->setChecked(AppSettings().value("dxf/marks", true).toBool());
    auto *notes = new QCheckBox(QStringLiteral("Notes (name, rotation, radii)"), &dlg);
    notes->setChecked(AppSettings().value("dxf/notes", true).toBool());
    auto *units = new QComboBox(&dlg);
    units->addItems({QStringLiteral("mm"), QStringLiteral("in")});
    units->setCurrentIndex(AppSettings().value("dxf/units", 0).toInt());
    units->setToolTip(QStringLiteral("The drawing's units: the cam's lengths are converted from the axis's units."));
    form->addRow(QStringLiteral("Units"), units);
    form->addRow(QStringLiteral("Points / rev"), points);
    form->addRow(QStringLiteral("Bore diameter"), bore);
    form->addRow(pitch);
    form->addRow(base);
    form->addRow(marks);
    form->addRow(notes);
    auto *info = new QLabel(QStringLiteral("Units: %1. Follower at 12 o'clock at crank 0°, cam turns clockwise.\n"
                                           "Max radius %2 %1, base %3, roller %4.")
                                .arg(u, QString::number(BuildCamShape(a, 720).maxRadius, 'g', 5),
                                     QString::number(a.baseRadius, 'g', 5), QString::number(a.rollerRadius, 'g', 5)),
                            &dlg);
    info->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));
    form->addRow(info);
    // Where a physical roller cannot trace the profile exactly.
    QStringList caveats;
    int steps = 0;
    for (const CamIssue &is : st.issues) steps += is.kind == IssueKind::VelocityStep;
    if (steps > 0) {
        caveats << QStringLiteral("%1 velocity step(s): the outline is the roller's envelope, so the roller rounds "
                                  "those corners instead of following the profile into them.")
                       .arg(steps);
    }
    if (!a.closed) {
        caveats << QStringLiteral("Open profile: a physical cam must drop back at 0°. The outline includes the relief "
                                  "the roller rolls off by; the follower loses contact over the drop.");
    }
    if (!caveats.isEmpty()) {
        auto *warn = new QLabel(caveats.join(QStringLiteral("\n")), &dlg);
        warn->setWordWrap(true);
        warn->setStyleSheet(QStringLiteral("QLabel { color: #ffcc80; }"));
        form->addRow(warn);
    }
    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    form->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    if (dlg.exec() != QDialog::Accepted) return;
    AppSettings().setValue("dxf/points", points->value());
    AppSettings().setValue("dxf/units", units->currentIndex());
    AppSettings().setValue("dxf/bore", bore->value());
    AppSettings().setValue("dxf/pitch", pitch->isChecked());
    AppSettings().setValue("dxf/base", base->isChecked());
    AppSettings().setValue("dxf/marks", marks->isChecked());
    AppSettings().setValue("dxf/notes", notes->isChecked());

    const QString baseName = QString::fromStdString(a.name).replace(QRegularExpression(QStringLiteral("\\W+")), "_");
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("Export cam DXF"),
                                                      AppSettings().value("files/lastDir").toString() + "/" + baseName + ".dxf",
                                                      QStringLiteral("DXF (*.dxf)"));
    if (path.isEmpty()) return;
    io::CamDxfOptions o;
    o.pointsPerRev = points->value();
    o.boreDiameter = bore->value();
    o.pitchCurve = pitch->isChecked();
    o.baseCircle = base->isChecked();
    o.marks = marks->isChecked();
    o.notes = notes->isChecked();
    o.units = units->currentIndex() == 1 ? io::DxfUnits::Inch : io::DxfUnits::Millimetre;
    QString err;
    if (!io::ExportCamDxf(m_project, m_axis, o, path, &err)) {
        QMessageBox::warning(this, QStringLiteral("Export cam DXF"), QStringLiteral("Could not write %1:\n%2").arg(path, err));
        return;
    }
    statusBar()->showMessage(QStringLiteral("Exported %1").arg(path), 5000);
}

// The current axis's path (its position over one cycle) as DXF. The positions are in the axis's own units; the one
// choice is the drawing's units, and on a timeline axis how far a second is across. A crank axis's degrees across are
// scaled for a drawing about twice as wide as tall (written in the notes).
void MainWindow::ExportPathDxf() {
    if (m_project.axes.empty()) return;
    const CamAxis &a = Axis();
    QDialog dlg(this);
    dlg.setWindowTitle(QStringLiteral("Export path DXF"));
    auto *form = new QFormLayout(&dlg);
    auto *units = new QComboBox(&dlg);
    units->addItems({QStringLiteral("mm"), QStringLiteral("in")});
    units->setCurrentIndex(AppSettings().value("dxf/units", 0).toInt());
    units->setToolTip(QStringLiteral("The drawing's units: positions are converted from the axis's units; an axis in degrees or "
                                     "counts is drawn as it is."));
    form->addRow(QStringLiteral("Units"), units);
    // A timeline axis: time across, so much per second, remembered for each unit (40 mm/s, 10 in/s to begin with).
    QDoubleSpinBox *perSec = nullptr;
    if (a.master == AxisMaster::Timeline) {
        perSec = new QDoubleSpinBox(&dlg);
        perSec->setObjectName(QStringLiteral("ECamDxfPerSecond"));
        perSec->setRange(0.001, 1e6);
        perSec->setDecimals(3);
        perSec->setToolTip(QStringLiteral("Drawing units across for each second of the timeline."));
        auto show = [perSec](int unit) {
            perSec->setSuffix(unit == 1 ? QStringLiteral(" in/s") : QStringLiteral(" mm/s"));
            perSec->setValue(unit == 1 ? AppSettings().value("dxf/perSecondIn", 10.0).toDouble()
                                       : AppSettings().value("dxf/perSecondMm", 40.0).toDouble());
        };
        show(units->currentIndex());
        auto shown = std::make_shared<int>(units->currentIndex());
        connect(units, &QComboBox::currentIndexChanged, &dlg, [perSec, show, shown](int unit) {
            AppSettings().setValue(*shown == 1 ? "dxf/perSecondIn" : "dxf/perSecondMm", perSec->value());
            *shown = unit;
            show(unit);
        });
        form->addRow(QStringLiteral("Across"), perSec);
    }
    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    form->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    if (dlg.exec() != QDialog::Accepted) return;
    AppSettings().setValue("dxf/units", units->currentIndex());
    if (perSec) AppSettings().setValue(units->currentIndex() == 1 ? "dxf/perSecondIn" : "dxf/perSecondMm", perSec->value());

    io::PathDxfOptions o;
    o.units = units->currentIndex() == 1 ? io::DxfUnits::Inch : io::DxfUnits::Millimetre;
    if (perSec) o.perSecond = perSec->value();
    const QString name = QString::fromStdString(a.name).replace(QRegularExpression(QStringLiteral("\\W+")), "_") + ".dxf";
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("Export path DXF"),
                                                      AppSettings().value("files/lastDir").toString() + "/" + name,
                                                      QStringLiteral("DXF (*.dxf)"));
    if (path.isEmpty()) return;
    QString err;
    if (!io::ExportPathDxf(m_project, m_axis, o, path, &err)) {
        QMessageBox::warning(this, QStringLiteral("Export path DXF"), QStringLiteral("Could not write %1:\n%2").arg(path, err));
        return;
    }
    statusBar()->showMessage(QStringLiteral("Exported %1").arg(QDir::toNativeSeparators(path)), 6000);
}

void MainWindow::closeEvent(QCloseEvent *e) {
    if (!MaybeSave()) {
        e->ignore();
        return;
    }
    AppSettings().setValue("window/geometry", saveGeometry());
    AppSettings().setValue("window/topRowHeight", m_topRowHeight);
    AppSettings().setValue("window/projectSplit", m_hSplit->saveState());
    AppSettings().setValue("window/treeStatusSplit", m_projectSplit->saveState());
    AppSettings().setValue("window/topRowSplit", m_diagramSplit->saveState());
    AppSettings().setValue("view/vel", m_showVel->isChecked());
    AppSettings().setValue("view/acc", m_showAcc->isChecked());
    AppSettings().setValue("view/jerk", m_showJerk->isChecked());
    AppSettings().setValue("view/snapDeg", m_snapCrank);
    AppSettings().setValue("view/snapTime", m_snapTime);
    AppSettings().setValue("view/snapPos", m_snapPos->value());
    e->accept();
}
