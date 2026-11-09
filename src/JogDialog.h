/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Board > Axis jog: every motor connector, M-0..M-3, on its own, to set the machine up. A card per motor: enable or
// disable it, hold a button to jog it, step it a set distance, zero it, and for a profile axis move it back where the
// cam has it. Everything goes through the ClearCore (its "axjog" console command and the step outputs), not the
// motors' USB.
//
// A profile axis moves in its own units; a motor the profile doesn't use (any motor, with no profile) in revolutions,
// at the counts per revolution set on its card (kept on this PC). A driven crank's motor only enables: Hand jog
// turns it, with the axes following.
//
// The board refuses all of it while a cam runs (or waits on its trigger), while the PLC has control, and with a
// fault latched: the panel then offers what is allowed, Stop and Disable.

#include "BoardLink.h"

#include <QDialog>
#include <QElapsedTimer>
#include <QVector>

#include <functional>

class QDoubleSpinBox;
class QFrame;
class QLabel;
class QPushButton;
class QSpinBox;
class QTimer;

class JogDialog : public QDialog {
    Q_OBJECT
public:
    explicit JogDialog(BoardLink *board, QWidget *parent = nullptr);
    ~JogDialog() override;

    enum class Role { Free, Axis, Crank };
    // One motor as the board reports it ("axjog" command).
    struct MotorInfo {
        int motor = 0;           // connector, M-n
        Role role = Role::Free;
        int axis = -1;           // the profile's axis, -1 none
        bool on = false;
        bool moving = false;
        qint64 pos = 0;          // counts (an axis: from the profile's start)
        bool hasCam = false;
        qint64 cam = 0;
        qint64 off = 0;          // off its cam position since axis jog moved it (counts), 0 when on it
        double cpu = 1.0;        // counts per unit as the board has it (1 for counts)
        double vmax = 0.0;       // the board's limit, in its units per second
        bool limitPlus = false, limitMinus = false;  // on its + / - limit (0.9.0)
        QString stopped;         // the input a stop on a switch stopped at, empty none
        qint64 stoppedAt = 0;    // where (counts)
        QString units, name;
    };
    static bool ParseMotorLine(const QString &line, MotorInfo *out);

    // For tests.
    const QVector<MotorInfo> &motors() const { return m_motors; }
    bool camActive() const { return m_camActive; }
    QString message() const;
    void pressJog(int motor, int direction);  // as pressing and releasing a card's jog button
    void releaseJog();
    // Cards from these motors, with no board (layout checks).
    void showMotorsForTest(const QVector<MotorInfo> &motors);
    void setStateForTest(const BoardLink::State &state) { m_testState = state; m_hasTestState = true; refresh(); }
    void reject() override;  // stops every jog first

protected:
    void hideEvent(QHideEvent *event) override;

private:
    struct Card {
        QFrame *frame = nullptr;
        QLabel *title = nullptr, *role = nullptr, *pos = nullptr, *note = nullptr;
        QPushButton *enable = nullptr, *jogMinus = nullptr, *jogPlus = nullptr, *stepMinus = nullptr, *stepPlus = nullptr;
        QPushButton *zero = nullptr, *toCam = nullptr, *home = nullptr;
        QDoubleSpinBox *speed = nullptr, *step = nullptr;
        QSpinBox *cpr = nullptr;  // a motor the profile doesn't use: counts per revolution
        QWidget *moveRows = nullptr;
    };
    void poll();
    void build();
    void refresh();
    void send(const QString &line, std::function<void(bool, const QStringList &)> done = nullptr);
    void startJog(int motor, int direction);
    void stopJog();
    void jogTick();
    void saveSettings(int motor);
    // The card's units: the axis's, or revolutions for a motor the profile doesn't use.
    double UnitCounts(int motor) const;  // counts per shown unit
    QString Units(int motor) const;
    void applyRanges(int motor);

    BoardLink *m_board;
    QTimer *m_poll = nullptr;
    QTimer *m_jogTimer = nullptr;
    bool m_pollInFlight = false;
    bool m_jogInFlight = false;
    int m_jogMotor = -1, m_jogDir = 0;
    bool m_unsupported = false;
    bool m_camActive = false;
    bool m_test = false;
    BoardLink::State m_testState;
    bool m_hasTestState = false;
    QString m_lastError;
    QElapsedTimer m_errorClock;
    QVector<MotorInfo> m_motors;
    QVector<Card> m_cards;
    QString m_builtFor;  // the roles and names the cards were built for

    QLabel *m_banner = nullptr;
    QPushButton *m_stopCam = nullptr, *m_disableAll = nullptr, *m_clearFault = nullptr, *m_homeAll = nullptr;
    QWidget *m_grid = nullptr;
    QLabel *m_note = nullptr;
};
