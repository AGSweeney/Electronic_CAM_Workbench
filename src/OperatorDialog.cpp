/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "OperatorDialog.h"

#include "HelpWindow.h"
#include "Theme.h"

#include <QAction>
#include <QComboBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPointer>
#include <QPushButton>
#include <QStandardItemModel>
#include <QVBoxLayout>

using namespace ecam;

const char *OperatorDialog::KeyName(int input) {
    static const char *const kKeys[] = {"op_enable", "op_disable", "op_reset", "op_start", "op_stop"};
    return input >= 0 && input < kInputs ? kKeys[input] : "?";
}

QString OperatorDialog::Label(int input) {
    static const char *const kLabels[] = {"Enable", "Disable", "Fault reset", "Start", "Stop"};
    return input >= 0 && input < kInputs ? QString::fromLatin1(kLabels[input]) : QString();
}

OperatorDialog::OperatorDialog(BoardLink *board, const CamProject &project, int estopPin, QWidget *parent)
    : QDialog(parent), m_board(board), m_project(project), m_estopPin(estopPin) {
    setWindowTitle(QStringLiteral("Operator Inputs"));
    setObjectName(QStringLiteral("ECamOperatorDialog"));
    setStyleSheet(theme::DialogSheet());
    auto *v = new QVBoxLayout(this);
    v->setContentsMargins(16, 14, 16, 12);
    v->setSpacing(12);
    auto *g = new QGridLayout();
    g->setHorizontalSpacing(10);
    g->setVerticalSpacing(8);
    auto head = [this](const QString &t) {
        auto *l = new QLabel(t, this);
        l->setStyleSheet(QStringLiteral("QLabel { color: #8a929d; }"));
        return l;
    };
    g->addWidget(head(QStringLiteral("Input")), 0, 1);
    g->addWidget(head(QStringLiteral("Contact")), 0, 2);
    g->addWidget(head(QStringLiteral("Now")), 0, 3);
    for (int k = 0; k < kInputs; ++k) {
        g->addWidget(new QLabel(Label(k), this), k + 1, 0);
        auto *pin = m_pins[k] = new QComboBox(this);
        pin->setObjectName(QStringLiteral("ECamOpPin%1").arg(k));
        pin->addItem(QStringLiteral("Off"), -1);
        for (int p = 0; p < kTriggerInputs; ++p) pin->addItem(QString::fromStdString(BoardInputName(p)), p);
        auto *contact = m_contacts[k] = new QComboBox(this);
        contact->setObjectName(QStringLiteral("ECamOpContact%1").arg(k));
        contact->addItems({QStringLiteral("NC"), QStringLiteral("NO")});
        contact->setToolTip(QStringLiteral("Normally closed: pressed when the input opens (a broken wire reads as pressed). "
                                           "Normally open: pressed when it closes."));
        m_states[k] = new QLabel(this);
        m_states[k]->setObjectName(QStringLiteral("ECamOpState%1").arg(k));
        m_states[k]->setMinimumWidth(70);
        g->addWidget(pin, k + 1, 1);
        g->addWidget(contact, k + 1, 2);
        g->addWidget(m_states[k], k + 1, 3);
        connect(pin, &QComboBox::activated, this, [this, k](int i) {
            const int p = m_pins[k]->itemData(i).toInt();
            if (p == m_pin[k]) return;
            set(QString::fromLatin1(KeyName(k)), p < 0 ? QStringLiteral("off") : QString::fromStdString(BoardInputName(p)));
        });
        connect(contact, &QComboBox::activated, this, [this, k](int i) {
            if ((i == 0) == nc(k)) return;
            set(QStringLiteral("%1_contact").arg(QLatin1String(KeyName(k))), i == 0 ? QStringLiteral("nc") : QStringLiteral("no"));
        });
    }
    g->setColumnStretch(4, 1);
    v->addLayout(g);
    auto *modeRow = new QHBoxLayout();
    modeRow->addWidget(new QLabel(QStringLiteral("Enable"), this));
    m_mode = new QComboBox(this);
    m_mode->setObjectName(QStringLiteral("ECamOpMode"));
    m_mode->addItems({QStringLiteral("Momentary: Enable and Disable buttons"), QStringLiteral("Maintained: an Enable switch (off disables)")});
    modeRow->addWidget(m_mode, 1);
    v->addLayout(modeRow);
    connect(m_mode, &QComboBox::activated, this, [this](int i) {
        if ((i == 1) == m_maintained) return;
        set(QStringLiteral("op_enable_mode"), i == 1 ? QStringLiteral("maintained") : QStringLiteral("momentary"));
    });
    m_note = new QLabel(this);
    m_note->setObjectName(QStringLiteral("ECamOpNote"));
    m_note->setWordWrap(true);
    m_note->setStyleSheet(QStringLiteral("QLabel { color: #ffcc80; }"));
    v->addWidget(m_note);
    auto *buttons = new QHBoxLayout();
    auto *help = new QPushButton(QStringLiteral("Help"), this);
    auto *close = new QPushButton(QStringLiteral("Close"), this);
    close->setDefault(true);
    buttons->addWidget(help);
    buttons->addStretch(1);
    buttons->addWidget(close);
    v->addLayout(buttons);
    auto showHelp = [this]() { HelpWindow::Show(this, QStringLiteral("09-board.html#operator")); };
    connect(help, &QPushButton::clicked, this, showHelp);
    auto *helpKey = new QAction(this);
    helpKey->setShortcut(QKeySequence(Qt::Key_F1));
    connect(helpKey, &QAction::triggered, this, showHelp);
    addAction(helpKey);
    connect(close, &QPushButton::clicked, this, &QDialog::accept);
    connect(m_board, &BoardLink::stateUpdated, this, [this](const BoardLink::State &) { refresh(); });
    load();
    refresh();
}

QString OperatorDialog::message() const { return m_note->text(); }

void OperatorDialog::load() {
    if (!m_board->isConnected()) return;
    QPointer<OperatorDialog> self(this);
    m_board->command(QStringLiteral("config"), [self](bool ok, const QStringList &lines) {
        if (self && ok) self->applyConfig(lines);
    });
}

// "op_enable IO-3 no", "op_enable off nc", "op_enable_mode maintained"
void OperatorDialog::applyConfig(const QStringList &lines) {
    bool any = false;
    for (const QString &l : lines) {
        const QStringList kv = l.trimmed().split(QLatin1Char(' '), Qt::SkipEmptyParts);
        if (kv.size() == 2 && kv[0] == QStringLiteral("op_enable_mode")) {
            m_maintained = kv[1] == QStringLiteral("maintained");
            any = true;
            continue;
        }
        for (int k = 0; k < kInputs; ++k) {
            if (kv.size() != 3 || kv[0] != QLatin1String(KeyName(k))) continue;
            m_pin[k] = -1;
            for (int p = 0; p < kTriggerInputs; ++p) {
                if (kv[1] == QString::fromStdString(BoardInputName(p))) m_pin[k] = p;
            }
            m_nc = kv[2] == QStringLiteral("nc") ? (m_nc | (1 << k)) : (m_nc & ~(1 << k));
            any = true;
        }
    }
    m_loaded = m_loaded || any;
    refresh();
}

void OperatorDialog::set(const QString &key, const QString &value) {
    if (m_busy || !m_board->isConnected()) return;
    m_busy = true;
    m_note->clear();
    m_note->setProperty("ecamAuto", true);
    QPointer<OperatorDialog> self(this);
    m_board->command(QStringLiteral("set %1 %2").arg(key, value), [self, key](bool ok, const QStringList &r) {
        if (!self) return;
        self->m_busy = false;
        if (!ok) {
            self->m_note->setText(QStringLiteral("%1: %2").arg(key, r.isEmpty() ? QStringLiteral("refused") : r.last()));
            self->m_note->setProperty("ecamAuto", false);  // the board's reason stays up
        }
        self->load();
        self->m_board->command(QStringLiteral("status"));
    });
    refresh();
}

void OperatorDialog::refresh() {
    const BoardLink::State &s = m_hasTestState ? m_testState : m_board->state();
    const bool online = m_hasTestState || m_board->isConnected();
    const bool conn = online && s.valid;
    const bool moving = s.state == QLatin1String("running") || s.state == QLatin1String("jogging");
    const bool idle = conn && !s.enabled && s.motorsOn == 0 && s.jogMoving == 0 && !moving && !s.plcInControl();
    const bool editable = idle && m_loaded && !m_busy;
    for (int k = 0; k < kInputs; ++k) {
        QComboBox *pin = m_pins[k];
        {
            QSignalBlocker b1(pin), b2(m_contacts[k]);
            pin->setCurrentIndex(std::max(0, pin->findData(m_pin[k])));
            m_contacts[k]->setCurrentIndex(nc(k) ? 0 : 1);
        }
        // Pins taken: the estop's, another operator input's, or the project's (an output, the trigger, a home / limit,
        // a wait, the Encoder Adapter's): greyed, as the board refuses them.
        if (auto *model = qobject_cast<QStandardItemModel *>(pin->model())) {
            for (int i = 1; i < pin->count(); ++i) {
                const int p = pin->itemData(i).toInt();
                QString why;
                if (p == m_estopPin) why = QStringLiteral("The estop's input.");
                for (int j = 0; j < kInputs && why.isEmpty(); ++j) {
                    if (j != k && m_pin[j] == p) why = QStringLiteral("The %1 input.").arg(Label(j));
                }
                if (why.isEmpty() && (!BoardInputBlocked(m_project, p).empty() || BoardInputAt(m_project, p).role != CcioRole::None ||
                                      WaitOnPin(m_project, p))) {
                    why = QStringLiteral("Used by this project.");
                }
                QStandardItem *item = model->item(i);
                const bool blocked = !why.isEmpty() && p != m_pin[k];
                item->setFlags(blocked ? item->flags() & ~Qt::ItemIsEnabled : item->flags() | Qt::ItemIsEnabled);
                item->setToolTip(why);
            }
        }
        pin->setEnabled(editable);
        m_contacts[k]->setEnabled(editable && m_pin[k] >= 0);
        const bool pressed = conn && ((s.opPressed >> k) & 1);
        m_states[k]->setText(m_pin[k] < 0 || !conn ? QStringLiteral("–") : pressed ? QStringLiteral("Pressed") : QStringLiteral("Released"));
        m_states[k]->setStyleSheet(pressed ? QStringLiteral("QLabel { color: #ffcc80; }") : QStringLiteral("QLabel { color: #8a929d; }"));
    }
    {
        QSignalBlocker b(m_mode);
        m_mode->setCurrentIndex(m_maintained ? 1 : 0);
    }
    m_mode->setEnabled(editable);
    if (m_note->text().isEmpty() || m_note->property("ecamAuto").toBool()) {
        const QString why = !online ? QStringLiteral("Not connected to a board.")
                            : !s.valid           ? QString()
                            : s.plcInControl()   ? QStringLiteral("The PLC has control.")
                            : !idle              ? QStringLiteral("Disable the motors and stop the motion to change these.")
                                                 : QString();
        m_note->setText(why);
        m_note->setProperty("ecamAuto", true);
    }
}
