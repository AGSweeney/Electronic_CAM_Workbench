/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Board > Operator inputs: the machine's own buttons on the ClearCore's inputs (Enable, Disable, Fault reset, Start,
// Stop), each an input or off, NC or NO, and whether Enable is a momentary button (with Disable) or a maintained switch.
// These are board settings, like the estop: each change goes to the board at once ("set op_...") and is read back from
// "config". A pin the loaded project uses, the estop's or another operator input's is greyed, as the board refuses it.

#include "BoardLink.h"
#include "CamProfile.h"

#include <QDialog>

class QComboBox;
class QLabel;

class OperatorDialog : public QDialog {
    Q_OBJECT
public:
    OperatorDialog(BoardLink *board, const ecam::CamProject &project, int estopPin, QWidget *parent = nullptr);

    static constexpr int kInputs = 5;  // Enable, Disable, Fault reset, Start, Stop
    static const char *KeyName(int input);  // op_enable ...
    static QString Label(int input);       // Enable ...

    // What "config" said (tests feed it): each input's pin (-1 off), the contacts (bit n NC), the mode.
    void applyConfig(const QStringList &lines);
    int pin(int input) const { return m_pin[input]; }
    bool nc(int input) const { return ((m_nc >> input) & 1) != 0; }
    bool maintained() const { return m_maintained; }
    QString message() const;
    // The guide's pictures and tests: draw as connected to a board in this state.
    void setStateForTest(const BoardLink::State &state) { m_testState = state; m_hasTestState = true; refresh(); }

private:
    void load();
    void set(const QString &key, const QString &value);
    void refresh();

    BoardLink *m_board;
    ecam::CamProject m_project;
    int m_estopPin = -1;
    int m_pin[kInputs] = {-1, -1, -1, -1, -1};
    int m_nc = (1 << 1) | (1 << 4);
    bool m_maintained = false;
    bool m_loaded = false;
    bool m_busy = false;
    BoardLink::State m_testState;
    bool m_hasTestState = false;
    QComboBox *m_pins[kInputs] = {};
    QComboBox *m_contacts[kInputs] = {};
    QLabel *m_states[kInputs] = {};
    QComboBox *m_mode = nullptr;
    QLabel *m_note = nullptr;
};
