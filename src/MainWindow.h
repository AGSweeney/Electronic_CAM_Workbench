/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

#include "BoardLink.h"
#include "CamProfile.h"
#include "JogDialog.h"

#include <QElapsedTimer>
#include <QMainWindow>

#include "NetSettings.h"

#include <optional>

#include <functional>
#include <optional>

class BoardLink;
class CamView;
class CollapsibleGroup;
class ScopeWindow;
class SideDock;
class QBoxLayout;
class QLayout;
class QMenu;
class QTreeWidget;
class QTreeWidgetItem;
class TimingView;
class QGroupBox;
class QSpinBox;
class QStackedWidget;
class ProfileStrip;
class QAction;
class QDialog;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QLineEdit;
class QPushButton;
class QSplitter;
class QTableWidget;
class QTimer;
class QUndoStack;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    // Axis > Import points as spline: replaces the axis with a spline through the file's points (undoable).
    // message: what was done, or why not.
    bool ImportPoints(const QString &path, QString *message);
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

    // Undo/redo entry point: replace the whole project (and the current axis).
    void Restore(const ecam::CamProject &project, int axis);

    bool OpenFile(const QString &path);
    // Replace the project with one that has no file yet (wizard, example); `modified` asks to save it on close.
    void UseProject(const ecam::CamProject &project, const QString &message, bool modified, bool locked = false);
    // The start page (new, guided setup, open, examples, recent); also File > Start page.
    void ShowStartPage();
    // The guided setup; false when cancelled.
    bool RunSetupWizard();
    void ShowAxis(int axis, double angleDeg);
    // Connect the Board panel to an IP address or a COM port (command line / tests).
    void ConnectBoard(const QString &target);
    // Sends the Network fields and restarts the board; ask: confirm with the user first.
    void ApplyBoardNetwork(bool ask = true);
    // The profile lock: while locked the motion (knots, segments, laws) and the output pulses can't be
    // edited in the diagram or the tables. Opened files and downloads start locked, new projects unlocked.
    void SetLocked(bool locked);
    bool IsLocked() const { return m_locked; }
    // The demo video tool (tools/ecam_demo): the open project, the board link, close without asking to save.
    const ecam::CamProject &project() const { return m_project; }
    BoardLink *board() const { return m_board; }
    void DiscardChangesForDemo();

protected:
    void closeEvent(QCloseEvent *e) override;

private:
    // Field ids for merging repeated edits of one control into a single undo step.
    enum MergeId {
        kNoMerge = -1,
        kMergeCountsPerRev = 1,
        kMergeRpm,
        kMergeName,
        kMergeUnits,
        kMergeCountsPerUnit,
        kMergeGearNum,
        kMergeGearDen,
        kMergeAxisGearNum,
        kMergeAxisGearDen,
        kMergeMotorPpr,
        kMergeScrewPitch,
        kMergePulleyPd,
        kMergeBaseRadius,
        kMergeRollerRadius,
        kMergeCrankRamp,
        kMergeCrankMax,
        kMergeVelMax,
        kMergeAccMax,
        kMergeOverride,
        kMergeDuration,
        kMergeTriggerBit,
        kMergeWheel,
        kMergeDesignLength,
        kMergeKerf,
        kMergeHomeFast,
        kMergeHomeSlow,
        kMergeHomeBack,
        kMergeHomeAt,
        kMergeHomeSearch,
        kMergeHomeOrder,
        kMergeHomeDecel,
    };

    // Fastest crank speed every axis (and the crank motor) allows, and what binds it.
    struct MasterLimit {
        double rpm;
        QString reason;
    };
    MasterLimit ProjectSpeedLimit() const;    // crank axes, rpm
    MasterLimit TimelineOverrideLimit() const;  // timeline axes, override as a fraction (1 = 100 %)
    QString XText(const ecam::CamAxis &a, double x, int decimals = 1) const;  // "120.0°" or "1.25 s"
    bool OnTimeline() const { return Axis().master == ecam::AxisMaster::Timeline; }
    // The centre shows the Timing view: on a timeline axis, or when View > Timing view is on.
    bool ShowTiming() const { return OnTimeline() || m_forceTiming; }
    double ViewSpan() const;  // strip width: 360 deg, or the timeline length

    void BuildUi();
    void BuildMenus();
    void BuildSettingsPanels();
    QWidget *BuildProjectTree();
    // The Project dock: the tree over the machine status (estop, each motor), split.
    QWidget *BuildProjectPanel();
    QWidget *BuildMachineStatus();
    void RefreshMachineStatus();
    QWidget *BuildDetailPane();
    void RefreshTree();
    void RefreshTreeMarks();
    void ShowDetail(int page);
    QWidget *BuildOutputsPanel(QWidget *parent);
    void RefreshOutputs();
    void RefreshCcio();
    void RefreshCcioLive();
    // Inputs: the board's own inputs (IO-0..A-12) for homes and limits, as the CCIO-8 inputs.
    QWidget *BuildInputsPanel(QWidget *parent);
    void RefreshInputs();
    QWidget *BuildWaitsPanel(QWidget *parent);
    void RefreshWaits();
    void AddWait(double t);  // at the nearest time every timeline axis is at rest
    void RefreshInputsLive();
    void EditBoardInput(int pin, const QString &text, const std::function<void(ecam::BoardInput &)> &fn);
    QWidget *BuildCcioPanel(QWidget *parent);
    void EditCcioInput(int pin, const QString &text, const std::function<void(ecam::CcioInput &)> &fn);
    // A pulse on the viewed axis's master at x (degrees or seconds): 10 deg / 0.1 s long.
    void AddPulse(int output, double x);
    void EditOutputs(const QString &text, const std::function<void(std::vector<ecam::OutputPulse> &)> &fn);
    void BuildSegmentPanel();
    QWidget *BuildStatusStrip();
    QWidget *BuildStripPanel();
    void BuildBoardPanel();
    void RefreshBoard();
    void RefreshBoardTargets();
    void BoardConnect();
    void OpenFirmwareUpdate();
    void ReconnectBoard(const QString &done);
    void BoardDownload();
    // "restart", then reconnect when the board is back.
    void RestartBoard(const QString &message);
    void LoadBoardNetwork();
    void RefreshNetwork();
    net::Settings NetFields() const;
    quint32 ProjectHash() const;

    ecam::CamAxis &Axis() { return m_project.axes[m_axis]; }
    const ecam::CamAxis &Axis() const { return m_project.axes[m_axis]; }

    // Applies fn to the project, normalizes, records an undo step, refreshes.
    void EditProject(const QString &text, const std::function<void(ecam::CamProject &)> &fn, int mergeId = kNoMerge);
    void EditAxis(const QString &text, const std::function<void(ecam::CamAxis &)> &fn, int mergeId = kNoMerge);

    void Refresh();
    void RefreshAxisFields();
    void RefreshTable();
    void RefreshStats();
    void RefreshTitle();
    void SelectSegment(int seg);
    void SetAngle(double deg);
    void SetTime(double t);
    void RefreshCursor();
    void ApplyLock();  // the lock onto the diagram, tables and edit buttons

    void OnTableItemChanged(int row, int col);
    void OnKnotDragged(int knot, double deg, double pos);

    void NewProject();
    bool OpenProject();
    void OpenExample(const QString &resource, const QString &title);
    bool Save();
    bool SaveAs();
    bool MaybeSave();
    void Export(bool header);
    void ExportCamDxf();
    void ExportPathDxf();
    void ExportWorkbook();
    void GenerateReport();
    void AddAxis(bool duplicate);
    void RemoveAxis();
    void TogglePlay(bool on);
    void AnimTick();

    ecam::CamProject m_project;
    int m_axis = 0;
    int m_seg = 0;
    double m_angle = 0.0;  // crank cursor, deg
    double m_time = 0.0;   // timeline cursor, s
    bool m_timeRunning = false;
    double m_snapCrank = 1.0, m_snapTime = 0.01;
    QString m_path;
    bool m_updating = false;
    std::optional<ecam::CamProject> m_dragBefore;

    QUndoStack *m_undo = nullptr;
    QTimer *m_anim = nullptr;
    QElapsedTimer m_animClock;
    double m_animSpeed = 0.0;  // preview crank speed, deg/s
    double m_animTarget = 0.0;

    // Master
    QDoubleSpinBox *m_countsPerRev = nullptr;
    QDoubleSpinBox *m_rpm = nullptr;  // rpm, or line speed in m/min in cut-length mode
    QCheckBox *m_lineMode = nullptr;   // cut-length mode
    QCheckBox *m_encoderReverse = nullptr;
    QCheckBox *m_crankReverse = nullptr;
    QDoubleSpinBox *m_wheel = nullptr;
    QDoubleSpinBox *m_designLength = nullptr;
    QDoubleSpinBox *m_kerf = nullptr;
    QWidget *m_gearRow = nullptr;
    // Board panel: encoder (encoder master)
    QLabel *m_boardEncLabel = nullptr;
    QLabel *m_boardEnc = nullptr;
    QPushButton *m_boardEncZero = nullptr;
    qint64 m_encZero = 0;  // the count Zero was pressed at (app only)
    // Board panel: simulated encoder (simulate)
    QLabel *m_boardSimEncLabel = nullptr;
    QDoubleSpinBox *m_boardSimEncSpeed = nullptr;
    QPushButton *m_boardSimEncRun = nullptr;
    QLabel *m_boardSimEncState = nullptr;
    // Encoder counts per second for a simulated-encoder speed (m/min of line, or crank rpm).
    double SimEncoderCountsPerSecond(double speed) const;
    // Board panel: cut length (cut-length mode)
    QLabel *m_boardCutLabel = nullptr;
    QDoubleSpinBox *m_boardCut = nullptr;
    QPushButton *m_boardCutSet = nullptr;
    QLabel *m_boardCutState = nullptr;
    QSpinBox *m_gearNum = nullptr;  // crank gear ratio numerator : denominator
    QSpinBox *m_gearDen = nullptr;
    QLabel *m_masterInfo = nullptr;
    QComboBox *m_masterSource = nullptr;
    QLabel *m_countsLabel = nullptr;
    QLabel *m_speedLabel = nullptr;
    QComboBox *m_crankConnector = nullptr;
    QDoubleSpinBox *m_crankRamp = nullptr;
    QDoubleSpinBox *m_crankMax = nullptr;
    QFormLayout *m_masterForm = nullptr;
    QFormLayout *m_masterForm2 = nullptr;  // its right column
    CollapsibleGroup *m_crankGroup = nullptr;
    // Timeline
    CollapsibleGroup *m_timelineGroup = nullptr;
    CollapsibleGroup *m_outputsGroup = nullptr;
    CollapsibleGroup *m_waitsGroup = nullptr;
    QTableWidget *m_waitTable = nullptr;
    QLabel *m_waitsInfo = nullptr;
    QTreeWidgetItem *m_treeWaits = nullptr;
    QPushButton *m_boardWaitSkip = nullptr;  // simulate: pass the wait holding the timeline
    CollapsibleGroup *m_ccioGroup = nullptr;
    QSpinBox *m_ccioBoards = nullptr;
    QLabel *m_ccioState = nullptr;
    QLabel *m_ccioHeading = nullptr;
    QTableWidget *m_ccioTable = nullptr;
    QTreeWidgetItem *m_treeCcio = nullptr;
    QTreeWidgetItem *m_treeInputs = nullptr;
    CollapsibleGroup *m_inputsGroup = nullptr;
    QTableWidget *m_inputsTable = nullptr;
    QDoubleSpinBox *m_limitStop = nullptr;  // the limit stop time (s)
    QLabel *m_overtravel = nullptr;         // each axis with a limit: how far past it the controlled stop runs
    int m_ccioBoard = 0;
    QTableWidget *m_outputTable = nullptr;
    QPushButton *m_deletePulse = nullptr;
    QPushButton *m_addPulse = nullptr;
    bool m_locked = false;
    QPushButton *m_lockButton = nullptr;
    QAction *m_lockAction = nullptr;
    QAction *m_splitSegAction = nullptr;
    QLabel *m_outputsInfo = nullptr;
    bool m_forceTiming = false;
    QAction *m_timingAction = nullptr;
    QFormLayout *m_timelineForm = nullptr;
    QCheckBox *m_repeat = nullptr;
    QDoubleSpinBox *m_override = nullptr;
    QComboBox *m_triggerSource = nullptr;
    QComboBox *m_triggerInput = nullptr;
    QComboBox *m_triggerEdge = nullptr;
    QSpinBox *m_triggerBit = nullptr;
    QLabel *m_timelineInfo = nullptr;
    // Axis
    QComboBox *m_axisCombo = nullptr;
    ScopeWindow *m_scope = nullptr;    // Tools > ClearPath scope (made on first use)
    QComboBox *m_stripAxis = nullptr;  // the same choice, in the displacement diagram's bar
    QLabel *m_readout = nullptr;       // cursor readout (DRO) in the diagram's bar
    QPushButton *m_fitButton = nullptr;  // zoom level; click to fit the whole cycle
    QPushButton *m_removeAxisButton = nullptr;
    QLineEdit *m_axisName = nullptr;
    QComboBox *m_axisConnector = nullptr;
    QComboBox *m_axisMaster = nullptr;
    QComboBox *m_axisDrive = nullptr;
    QDoubleSpinBox *m_motorPpr = nullptr;
    QDoubleSpinBox *m_screwPitch = nullptr;
    QDoubleSpinBox *m_pulleyPd = nullptr;
    QDoubleSpinBox *m_duration = nullptr;
    QFormLayout *m_axisForm = nullptr;
    QFormLayout *m_axisForm2 = nullptr;  // its right column
    QComboBox *m_axisType = nullptr;   // cam profile or electronic gear
    QWidget *m_axisGearRow = nullptr;
    QSpinBox *m_axisGearNum = nullptr;
    QSpinBox *m_axisGearDen = nullptr;
    QLabel *m_axisGearHint = nullptr;
    QDoubleSpinBox *m_velMax = nullptr;
    QDoubleSpinBox *m_accMax = nullptr;
    QComboBox *m_units = nullptr;
    QDoubleSpinBox *m_countsPerUnit = nullptr;
    QCheckBox *m_closed = nullptr;
    QCheckBox *m_axisReverse = nullptr;
    // Homing (the axis's home switch, its sequence)
    QLabel *m_homeSwitch = nullptr;
    QLabel *m_homeSummary = nullptr;
    QDialog *m_homeDialog = nullptr;
    QComboBox *m_homeDir = nullptr;
    QDoubleSpinBox *m_homeFast = nullptr;
    QDoubleSpinBox *m_homeSlow = nullptr;
    QDoubleSpinBox *m_homeBack = nullptr;
    QDoubleSpinBox *m_homeAt = nullptr;
    QDoubleSpinBox *m_homeSearch = nullptr;
    QDoubleSpinBox *m_homeDecel = nullptr;
    QComboBox *m_homeThen = nullptr;
    QSpinBox *m_homeOrder = nullptr;
    QCheckBox *m_homeRequired = nullptr;
    QDoubleSpinBox *m_baseRadius = nullptr;
    QDoubleSpinBox *m_rollerRadius = nullptr;
    QLabel *m_stats = nullptr;
    // Segments
    QTableWidget *m_table = nullptr;
    QLabel *m_tableHint = nullptr;
    QPushButton *m_splitButton = nullptr;
    QPushButton *m_deleteButton = nullptr;
    // Views
    CamView *m_camView = nullptr;
    TimingView *m_timing = nullptr;
    QStackedWidget *m_viewStack = nullptr;
    QGroupBox *m_viewGroup = nullptr;
    QLabel *m_snapLabel = nullptr;
    ProfileStrip *m_strip = nullptr;
    QCheckBox *m_showVel = nullptr;
    QCheckBox *m_showAcc = nullptr;
    QCheckBox *m_showJerk = nullptr;
    QCheckBox *m_showOutputs = nullptr;
    QDoubleSpinBox *m_snapDeg = nullptr;
    QDoubleSpinBox *m_snapPos = nullptr;
    QPushButton *m_playButton = nullptr;
    // The project tree picks what the detail pane under the diagram shows.
    QTreeWidget *m_tree = nullptr;
    QSplitter *m_projectSplit = nullptr;  // tree over status
    QLabel *m_statusBoard = nullptr, *m_statusEstop = nullptr, *m_statusFault = nullptr;
    QLabel *m_statusMotorName[4] = {}, *m_statusMotor[4] = {};
    QTreeWidgetItem *m_treeRoot = nullptr;
    QTreeWidgetItem *m_treeMaster = nullptr;
    QTreeWidgetItem *m_treeCrank = nullptr;
    QTreeWidgetItem *m_treeTimeline = nullptr;
    QTreeWidgetItem *m_treeAxes = nullptr;
    QTreeWidgetItem *m_treeOutputs = nullptr;
    QTreeWidgetItem *m_treeAnalysis = nullptr;
    QTreeWidgetItem *m_treeBoard = nullptr;
    QTimer *m_markTimer = nullptr;  // the review runs once things settle, not on every drag
    QStackedWidget *m_details = nullptr;
    int m_detailPage = 2;  // kPageAxis
    CollapsibleGroup *m_axisGroup = nullptr;
    CollapsibleGroup *m_segmentsGroup = nullptr;
    CollapsibleGroup *m_statsGroup = nullptr;
    CollapsibleGroup *m_boardGroup = nullptr;
    QLabel *m_boardLed = nullptr;
    QString m_boardLedColor;
    QMenu *m_axisMenu = nullptr;
    QSplitter *m_diagramSplit = nullptr;
    int m_topRowHeight = 350;  // what the top row returns to when the window has the room
    QSplitter *m_vSplit = nullptr;
    QSplitter *m_hSplit = nullptr;
    SideDock *m_settingsDock = nullptr;
    SideDock *m_camDock = nullptr;
    QAction *m_deleteSegAction = nullptr;
    QAction *m_exportDxfAction = nullptr;
    QAction *m_importPointsAction = nullptr;

    // Board
    BoardLink *m_board = nullptr;
    QComboBox *m_boardTransport = nullptr;
    QComboBox *m_boardTarget = nullptr;
    QPushButton *m_boardDiscover = nullptr;
    QPushButton *m_boardConnect = nullptr;
    QPushButton *m_boardUpload = nullptr;
    QPushButton *m_boardDownload = nullptr;
    QPushButton *m_boardForget = nullptr;
    QPushButton *m_boardEnable = nullptr;
    QPushButton *m_boardDisable = nullptr;
    QPushButton *m_boardClearFault = nullptr;
    QPushButton *m_boardStart = nullptr;
    QPushButton *m_boardTrigger = nullptr;
    QPushButton *m_boardStop = nullptr;
    QPushButton *m_boardAbort = nullptr;
    QPushButton *m_boardJog = nullptr;
    QPushButton *m_boardCrankZero = nullptr;
    QPushButton *m_boardAxisJog = nullptr;  // the Axis jog panel (each axis on its own)
    QPushButton *m_boardHome = nullptr;     // Home all (only with home switches on the board)
    void OpenAxisJog();
    void OfferCamPositions();  // Start refused off the cam positions: offer to move the axes there
    void MoveToCamPositions(const QVector<JogDialog::MotorInfo> &off, int i);
    void OpenSwitchWizard();  // hand jog, crank at rest: go to 0 deg the short way
    QLabel *m_boardTimelineLabel = nullptr;   // the Timeline row (hidden without timeline axes)
    QLabel *m_boardTimelineState = nullptr;   // armed / running / done, and what triggers it
    // Hand jog: drags drive the board's master.
    bool m_jogging = false;
    double m_jogCrankDeg = 0.0;     // absolute crank target since the load (rev * 360 + angle)
    bool m_jogInFlight = false;
    QElapsedTimer m_jogClock;
    QString m_jogPending;           // latest target not sent yet
    void SendJog(const QString &cmd);
    void SetJogging(bool on);
    QCheckBox *m_boardSim = nullptr;
    QCheckBox *m_boardFollow = nullptr;
    QLabel *m_boardFirmware = nullptr;
    QPushButton *m_boardFirmwareUpdate = nullptr;
    // Network settings: what the board has, and what the fields say.
    net::Settings m_netLoaded;
    std::optional<net::Settings> m_netExpected;  // what a restart should come back with
    QComboBox *m_netMode = nullptr;
    QLineEdit *m_netIp = nullptr;
    QLineEdit *m_netMask = nullptr;
    QLineEdit *m_netGateway = nullptr;
    QLineEdit *m_netHost = nullptr;
    // Estop: the board's estop_a9 (0 off, 1 fault when A-9 is low, 2 when high), applied at once over either link.
    // The Board page: the controls beside the settings (Estop, Network), or above them when the pane is narrow.
    QBoxLayout *m_boardCols = nullptr;
    QLayout *m_boardControls = nullptr, *m_boardSettings = nullptr;
    void ReflowBoardPage(int width);
    QComboBox *m_estopMode = nullptr;
    QComboBox *m_estopPin = nullptr;
    QLabel *m_estopState = nullptr;
    int m_estopLoaded = -1;     // what the board has, -1 not read
    int m_estopPinLoaded = -1;  // its input, -1 not read
    // The board's estop input as last seen (from its status), kept across sessions: the pin the project can't use. -1
    // none. EstopPinSeen: it came from the board this session, not a memory of the last one.
    int m_estopPinKnown = -1;
    // The board's operator inputs' pins (Enable, Disable, Fault reset, Start, Stop; -1 none), remembered like the estop's.
    int m_opPinsKnown[5] = {-1, -1, -1, -1, -1};
    int OpAt(int pin) const;           // the operator input on this pin, -1 none
    QString OpPinName(int pin) const;  // "Operator: Enable", empty none
    QLabel *m_opSummary = nullptr;
    QPushButton *m_opButton = nullptr;
    void OpenOperatorInputs();
    bool m_estopPinLive = false;
    void ApplyBoardEstopPin(int pin);
    void LoadBoardEstop();
    void ApplyBoardEstop(int mode);
    void RefreshEstop();
    QLabel *m_netInUse = nullptr;
    QLabel *m_netState = nullptr;
    QLabel *m_netIpLabel = nullptr;
    QPushButton *m_netApply = nullptr;
    QTimer *m_netPoll = nullptr;
    QCheckBox *m_boardTriggerOnStart = nullptr;
    QLabel *m_boardStatus = nullptr;
    bool m_boardBusy = false;
};
