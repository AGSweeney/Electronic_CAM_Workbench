/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Tools > Set up limits and homing: one axis's switches, from wiring to jobs proved by motion.
//   Axis       the axis, which switches it has, the setup speed
//   Find       trip each switch by hand: the input that changes is the switch's
//   Contact    each switch's level at rest says normally closed (on) or open (off); confirm it
//   Direction  hold Jog + / Jog -: the board stops the axis at the first switch it reaches (a stop on a switch jog),
//              and which switch that is proves the jobs. The - limit reached going + means the switches were swapped,
//              or the motor runs the other way: one question tells which. The home switch's side is seen on the way.
//   Limits     (an axis with limits) the switches go up to the board, and its own limits stop the axis: hold Jog until
//              each limit stops it, then off it again; how far past the switch it ran, the travel between them
//   Home       (an axis with a home switch) the homing settings, from what was seen; home it N times and see the spread
//              of the latched positions
//   Review     what was found and proved; Apply writes it into the project and uploads it
// Nothing is taken on trust from a hand: every limit is proved by driving the axis to it.

#include "BoardLink.h"
#include "CamProfile.h"

#include <QDialog>
#include <QString>
#include <QVector>

#include <functional>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QSpinBox;
class QStackedWidget;
class QTimer;

class SwitchWizard : public QDialog {
    Q_OBJECT
public:
    // upload(project, done): put this project into the app and upload it; done(ok, message) when the board has it.
    using Upload = std::function<void(const ecam::CamProject &, std::function<void(bool, const QString &)>)>;
    SwitchWizard(BoardLink *board, const ecam::CamProject &project, int estopPin, Upload upload, QWidget *parent = nullptr);
    ~SwitchWizard() override;

    enum Switch { kLimitPlus = 0, kLimitMinus, kHome, kSwitches };
    // An input: the board's (0..12) or a CCIO-8's (100 + board * 8 + pin), -1 none.
    struct Found {
        bool wanted = false;
        int input = -1;
        bool restOn = true;        // its level at rest
        bool normallyOpen = false;
        bool proved = false;       // reached by motion where its job says
    };

    // The project with what was found written in (the axis's switches, its motor direction).
    ecam::CamProject result() const;
    int step() const { return m_step; }
    const Found &found(int s) const { return m_found[s]; }
    int homeSide() const { return m_homeSide; }
    QString message() const;
    static QString InputName(int input);

    // What happened when a stop on a switch stopped the axis: the switch reached, going dir.
    enum class Verdict { Right, Ask, Home, Unknown };
    static Verdict Judge(int dir, int reached);  // reached: kLimitPlus / kLimitMinus / kHome, -1 none of them

    // For tests: drive it without a board.
    void setStepForTest(int step);
    void feedLevels(int board, quint64 ccio, int ccioBits);
    void feedStop(int dir, const QString &input);  // the axis stopped at this input going dir
    void answer(bool towardTrippedEnd);            // the question after a stop on the wrong limit
    void feedAxlimit(const QStringList &lines, bool moving);  // the board's axlimit lines (the Limits step)
    void feedAxhome(const QStringList &lines);     // the board's axhome lines (the Home step)
    void startHomeRunsForTest(int runs);           // as Home N times, after the upload, with no board
    bool limitStopped(int side) const { return m_limStopped[side]; }
    bool limitCleared(int side) const { return m_limCleared[side]; }
    qint64 overtravelCounts(int side) const { return m_overCounts[side]; }
    int homeRuns() const { return m_runsDone; }
    double homeSpreadCounts() const;
    void reject() override;                        // stops a jog first
    void setOperatorPins(const int pins[5]);       // the board's operator inputs: not offered as switches
    void setStateForTest(const BoardLink::State &state);  // the guide's pictures: draw as connected in this state

private:
    void build();
    void go(int step);
    bool applies(int step) const;  // the step is for this axis's switches
    int nextStep(int from, int dir) const;
    void uploadThen(const QString &ask, std::function<void()> then);
    void toggleEnable();
    void homeRun();
    void refresh();
    void poll();
    void onState(const BoardLink::State &s);
    void onAxjog(const QStringList &lines);
    void startJog(int dir);
    void stopJog();
    void jogTick();
    QString stopList() const;
    int motor() const;
    int axis() const;
    QString switchName(int s) const;
    QLabel *nameLabel(int s, QWidget *page);  // a switch's name on a page (hidden when the axis doesn't have it)
    int levelOf(int input, int board, quint64 ccio) const;
    QString blockedWhy(int input) const;

    BoardLink *m_board;
    ecam::CamProject m_project;  // the working copy: Apply hands it back
    ecam::CamProject m_original; // as the wizard opened (an axis's own homing settings, if it had any)
    int m_estopPin = -1;
    int m_opPins[5] = {-1, -1, -1, -1, -1};
    BoardLink::State m_testState;
    bool m_hasTestState = false;
    Upload m_upload;
    QTimer *m_poll = nullptr, *m_jogTimer = nullptr;
    bool m_pollBusy = false, m_jogBusy = false, m_uploading = false;

    int m_step = 0;
    Found m_found[kSwitches];
    // Find
    int m_finding = -1;       // the switch being found
    int m_baseBoard = -1;     // the levels when it started looking
    quint64 m_baseCcio = 0;
    int m_ccioBits = 0;
    int m_candidate = -1;     // the input that changed: waiting for it to come back
    QString m_findNote;
    int m_lastBoard = -1;
    quint64 m_lastCcio = 0;
    // Direction
    int m_jogDir = 0;
    int m_homeSide = 0;       // the side the home switch is on, from the motion: +1 / -1, 0 not seen
    int m_askDir = 0;         // a question is waiting: the direction the wrong limit was reached in
    bool m_reversed = false;  // the wizard set the axis's motor direction
    QString m_dirNote;
    bool m_motorOn = false;
    // Limits: the axis jog stops at each limit (0 +, 1 -), seen against where the counts were when the step began.
    bool m_tripBaseKnown = false;
    quint32 m_tripBase[2] = {0, 0};
    bool m_limStopped[2] = {false, false}, m_limCleared[2] = {false, false};
    qint64 m_tripAt[2] = {0, 0}, m_overCounts[2] = {-1, -1};
    QString m_limNote;
    // Home
    ecam::AxisHome m_home;
    bool m_homeSet = false;       // m_home holds the step's settings
    bool m_homeUp = false;        // they are on the board
    bool m_homeRunning = false, m_homeFailed = false, m_endsKnown = false, m_homeTest = false;
    quint32 m_endsBase = 0;
    int m_runsWanted = 5, m_runsDone = 0;
    QVector<qint64> m_devCounts;  // each homing's latch from the first one's (counts)
    QString m_homeNote;

    QLabel *m_title = nullptr, *m_note = nullptr;
    QVector<QLabel *> m_names[kSwitches];
    QStackedWidget *m_pages = nullptr;
    QComboBox *m_axis = nullptr;
    QCheckBox *m_want[kSwitches] = {};
    QDoubleSpinBox *m_speed = nullptr;
    QLabel *m_ready = nullptr;
    QLabel *m_findRows[kSwitches] = {};
    QLabel *m_contactRows[kSwitches] = {};
    QComboBox *m_contact[kSwitches] = {};
    QLabel *m_contactWarn = nullptr;
    QLabel *m_dirRows[kSwitches] = {};
    QPushButton *m_enable = nullptr, *m_jogMinus = nullptr, *m_jogPlus = nullptr, *m_yes = nullptr, *m_no = nullptr;
    QLabel *m_question = nullptr, *m_review = nullptr;
    QPushButton *m_limEnable = nullptr, *m_limMinus = nullptr, *m_limPlus = nullptr;
    QLabel *m_limRows[2] = {}, *m_limTravel = nullptr;
    QPushButton *m_homeEnable = nullptr, *m_homeGo = nullptr;
    QDoubleSpinBox *m_hFast = nullptr, *m_hSlow = nullptr, *m_hBack = nullptr, *m_hSearch = nullptr, *m_hAt = nullptr, *m_hDecel = nullptr;
    QComboBox *m_hThen = nullptr;
    QCheckBox *m_hRequired = nullptr;
    QSpinBox *m_hRuns = nullptr;
    QLabel *m_homeResults = nullptr;
    QPushButton *m_back = nullptr, *m_next = nullptr;
};
