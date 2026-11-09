/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "JogDialog.h"

#include "HelpWindow.h"
#include "Theme.h"

#include <QAction>

#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHideEvent>
#include <QLabel>
#include <QPointer>
#include <QPushButton>
#include <QRegularExpression>
#include <QSettings>
#include <QSpinBox>
#include <QStyle>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace {

constexpr int kPollMs = 250;
constexpr int kJogRepeatMs = 150;  // well inside the board's 0.5 s deadman
constexpr int kMotors = 4;

int Decimals(double countsPerUnit) {
    const double c = std::fabs(countsPerUnit);
    if (c <= 1.0) return 0;
    return std::min(4, static_cast<int>(std::ceil(std::log10(c))));
}

QString Num(double v, int decimals) { return QString::number(v, 'f', decimals); }

QString Key(int motor, const char *what) { return QStringLiteral("axisJog/M%1/%2").arg(motor).arg(QLatin1String(what)); }

QLabel *Muted(const QString &text, QWidget *parent) {
    auto *l = new QLabel(text, parent);
    l->setStyleSheet(QStringLiteral("QLabel { color: #8a929d; }"));
    return l;
}

}  // namespace

bool JogDialog::ParseMotorLine(const QString &line, MotorInfo *out) {
    // axjog <m> role=<axis|free|crank> axis=<i|-> on= moving= pos= cam=<n|-> off= cpu= vmax= units= name=<rest>
    static const QRegularExpression re(QStringLiteral(
        "^axjog (\\d) role=(axis|free|crank) axis=(-|\\d+) on=([01]) moving=([01]) pos=(-?\\d+) cam=(-|-?\\d+) "
        "off=(-?\\d+) cpu=(\\S+) vmax=(\\S+)(?: lim=(\\S+) stopped=(\\S+) at=(-?\\d+))? units=(\\S*) name=(.*)$"));
    const QRegularExpressionMatch m = re.match(line.trimmed());
    if (!m.hasMatch()) return false;
    MotorInfo a;
    a.motor = m.captured(1).toInt();
    const QString role = m.captured(2);
    a.role = role == QLatin1String("axis") ? Role::Axis : role == QLatin1String("crank") ? Role::Crank : Role::Free;
    a.axis = m.captured(3) == QLatin1String("-") ? -1 : m.captured(3).toInt();
    a.on = m.captured(4) == QLatin1String("1");
    a.moving = m.captured(5) == QLatin1String("1");
    a.pos = m.captured(6).toLongLong();
    a.hasCam = m.captured(7) != QLatin1String("-");
    a.cam = a.hasCam ? m.captured(7).toLongLong() : 0;
    a.off = m.captured(8).toLongLong();
    a.cpu = m.captured(9).toDouble();
    if (a.cpu == 0.0) a.cpu = 1.0;
    a.vmax = m.captured(10).toDouble();
    const QString lim = m.captured(11);
    a.limitPlus = lim.contains(QLatin1Char('+'));
    a.limitMinus = lim.contains(QLatin1Char('-'));
    a.stopped = m.captured(12) == QLatin1String("-") ? QString() : m.captured(12);
    a.stoppedAt = m.captured(13).toLongLong();
    a.units = m.captured(14);
    a.name = m.captured(15);
    *out = a;
    return true;
}

JogDialog::JogDialog(BoardLink *board, QWidget *parent) : QDialog(parent), m_board(board) {
    setWindowTitle(QStringLiteral("Axis Jog"));
    setStyleSheet(theme::DialogSheet() + QStringLiteral(
        "QFrame#AxisJogCard { border: 1px solid #3a414b; border-radius: 6px; }"
        "QFrame#AxisJogCard[motorOn=\"true\"] { border: 1px solid #4f8f5f; }"));

    auto *outer = new QVBoxLayout(this);
    outer->setContentsMargins(14, 12, 14, 12);
    outer->setSpacing(10);

    // The banner: what stops the panel, and what may still be done about it.
    auto *bannerRow = new QHBoxLayout();
    m_banner = new QLabel(this);
    m_banner->setObjectName(QStringLiteral("AxisJogBanner"));
    m_banner->setWordWrap(true);
    m_banner->setStyleSheet(QStringLiteral("QLabel { color: #f0c36a; font-weight: 600; }"));
    m_stopCam = new QPushButton(QStringLiteral("Stop"), this);
    m_stopCam->setObjectName(QStringLiteral("AxisJogStopCam"));
    m_stopCam->setToolTip(QStringLiteral("Bring the cam to rest (as the board panel's Stop)."));
    m_clearFault = new QPushButton(QStringLiteral("Clear fault"), this);
    bannerRow->addWidget(m_banner, 1);
    bannerRow->addWidget(m_stopCam);
    bannerRow->addWidget(m_clearFault);
    outer->addLayout(bannerRow);

    m_grid = new QWidget(this);
    auto *grid = new QGridLayout(m_grid);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setSpacing(10);
    outer->addWidget(m_grid);

    m_note = new QLabel(this);
    m_note->setObjectName(QStringLiteral("AxisJogNote"));
    m_note->setWordWrap(true);
    m_note->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));
    outer->addWidget(m_note);

    auto *buttons = new QHBoxLayout();
    m_disableAll = new QPushButton(QStringLiteral("Disable all motors"), this);
    m_disableAll->setObjectName(QStringLiteral("AxisJogDisableAll"));
    m_disableAll->setToolTip(QStringLiteral("Disable every motor, and stop anything moving at once (as Disable)."));
    m_homeAll = new QPushButton(QStringLiteral("Home all"), this);
    m_homeAll->setObjectName(QStringLiteral("AxisJogHomeAll"));
    m_homeAll->setToolTip(QStringLiteral("Home every axis with a home switch, in their order."));
    auto *help = new QPushButton(QStringLiteral("Help"), this);
    help->setObjectName(QStringLiteral("AxisJogHelp"));
    auto *close = new QPushButton(QStringLiteral("Close"), this);
    buttons->addWidget(m_disableAll);
    buttons->addWidget(m_homeAll);
    buttons->addStretch(1);
    buttons->addWidget(help);
    buttons->addWidget(close);
    auto showHelp = [this]() { HelpWindow::Show(this, QStringLiteral("09-board.html#axisjog")); };
    connect(help, &QPushButton::clicked, this, showHelp);
    auto *helpKey = new QAction(this);
    helpKey->setShortcut(QKeySequence(Qt::Key_F1));
    helpKey->setShortcutContext(Qt::WindowShortcut);
    connect(helpKey, &QAction::triggered, this, showHelp);
    addAction(helpKey);
    outer->addLayout(buttons);

    connect(m_stopCam, &QPushButton::clicked, this, [this]() { send(QStringLiteral("stop")); });
    connect(m_clearFault, &QPushButton::clicked, this, [this]() { send(QStringLiteral("clear")); });
    connect(m_disableAll, &QPushButton::clicked, this, [this]() {
        stopJog();
        send(QStringLiteral("disable"));
    });
    connect(m_homeAll, &QPushButton::clicked, this, [this]() {
        const bool homing = m_board->state().homing != 0;
        send(homing ? QStringLiteral("axhome stop") : QStringLiteral("axhome all"), [this](bool, const QStringList &) { poll(); });
    });
    connect(close, &QPushButton::clicked, this, &JogDialog::reject);

    m_poll = new QTimer(this);
    m_poll->setInterval(kPollMs);
    connect(m_poll, &QTimer::timeout, this, &JogDialog::poll);
    m_jogTimer = new QTimer(this);
    m_jogTimer->setInterval(kJogRepeatMs);
    connect(m_jogTimer, &QTimer::timeout, this, &JogDialog::jogTick);
    connect(m_board, &BoardLink::stateUpdated, this, [this](const BoardLink::State &) { refresh(); });
    m_poll->start();
    poll();
    refresh();
}

JogDialog::~JogDialog() { stopJog(); }

QString JogDialog::message() const { return m_note->text(); }

double JogDialog::UnitCounts(int motor) const {
    if (motor < 0 || motor >= m_motors.size()) return 1.0;
    const MotorInfo &a = m_motors[motor];
    if (a.role == Role::Axis) return a.cpu;
    if (a.role == Role::Free && motor < m_cards.size() && m_cards[motor].cpr) return m_cards[motor].cpr->value();
    return 1.0;
}

QString JogDialog::Units(int motor) const {
    if (motor < 0 || motor >= m_motors.size()) return QString();
    const MotorInfo &a = m_motors[motor];
    return a.role == Role::Axis ? a.units : a.role == Role::Free ? QStringLiteral("rev") : QStringLiteral("counts");
}

void JogDialog::send(const QString &line, std::function<void(bool, const QStringList &)> done) {
    if (!m_board->isConnected()) return;
    QPointer<JogDialog> self(this);
    m_board->command(line, [self, line, done](bool ok, const QStringList &r) {
        if (self && !ok) {
            const QString why = r.isEmpty() ? QStringLiteral("no reply") : r.last();
            self->m_lastError = QStringLiteral("%1: %2").arg(line, why.startsWith(QLatin1String("error: ")) ? why.mid(7) : why);
            self->m_errorClock.restart();
            self->refresh();
        }
        if (done) done(ok, r);
    });
}

void JogDialog::poll() {
    if (m_test) return;
    if (m_pollInFlight || !m_board->isConnected()) {
        refresh();
        return;
    }
    m_pollInFlight = true;
    QPointer<JogDialog> self(this);
    m_board->command(QStringLiteral("axjog"), [self](bool ok, const QStringList &r) {
        if (!self) return;
        self->m_pollInFlight = false;
        QVector<MotorInfo> motors;
        if (!ok) {
            const QString last = r.isEmpty() ? QString() : r.last();
            self->m_unsupported = last.contains(QLatin1String("unknown command"));
        } else {
            self->m_unsupported = false;
            for (const QString &l : r) {
                MotorInfo a;
                if (ParseMotorLine(l, &a) && a.motor == motors.size()) motors << a;
            }
        }
        self->m_motors = motors;
        QString shape;
        for (const MotorInfo &a : motors) shape += QStringLiteral("%1:%2:%3:%4;").arg(int(a.role)).arg(a.axis).arg(a.units, a.name);
        if (shape != self->m_builtFor) {
            self->m_builtFor = shape;
            self->build();
        }
        self->refresh();
    });
}

void JogDialog::showMotorsForTest(const QVector<MotorInfo> &motors) {
    m_test = true;
    m_poll->stop();
    m_motors = motors;
    build();
    refresh();
}

void JogDialog::build() {
    stopJog();
    auto *grid = static_cast<QGridLayout *>(m_grid->layout());
    while (QLayoutItem *item = grid->takeAt(0)) {
        delete item->widget();
        delete item;
    }
    m_cards.clear();
    QSettings settings;
    for (int m = 0; m < m_motors.size() && m < kMotors; ++m) {
        const MotorInfo &a = m_motors[m];
        Card c;
        c.frame = new QFrame(m_grid);
        c.frame->setObjectName(QStringLiteral("AxisJogCard"));
        auto *g = new QGridLayout(c.frame);
        g->setContentsMargins(12, 10, 12, 10);
        g->setHorizontalSpacing(8);
        g->setVerticalSpacing(7);
        g->setColumnStretch(0, 1);

        // Title row: M-n and its name, Enable at the right; what it is under it.
        c.title = new QLabel(QStringLiteral("M-%1").arg(m) + (a.name.isEmpty() ? QString() : QStringLiteral("   ") + a.name.toHtmlEscaped()),
                             c.frame);
        c.title->setStyleSheet(QStringLiteral("QLabel { font-weight: 600; font-size: 14px; }"));
        c.role = Muted(a.role == Role::Axis    ? QStringLiteral("Profile axis %1, in %2").arg(a.axis).arg(a.units)
                       : a.role == Role::Crank ? QStringLiteral("Driven crank")
                                               : QStringLiteral("Not in the profile"),
                       c.frame);
        c.enable = new QPushButton(QStringLiteral("Enable"), c.frame);
        c.enable->setObjectName(QStringLiteral("AxisJogEnable%1").arg(m));
        c.enable->setMinimumWidth(84);
        c.pos = new QLabel(c.frame);
        c.pos->setObjectName(QStringLiteral("AxisJogPos%1").arg(m));
        c.pos->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        c.pos->setStyleSheet(QStringLiteral("QLabel { font-size: 17px; font-weight: 600; }"));
        c.pos->setToolTip(a.role == Role::Axis ? QStringLiteral("Commanded position from the profile's start (0 at the load, or where you zeroed it).")
                                               : QStringLiteral("Commanded position since the board started, or since you zeroed it."));
        g->addWidget(c.title, 0, 0);
        g->addWidget(c.enable, 0, 1);
        g->addWidget(c.role, 1, 0);
        g->addWidget(c.pos, 1, 1, 2, 1);

        c.moveRows = new QWidget(c.frame);
        auto *mv = new QGridLayout(c.moveRows);
        mv->setContentsMargins(0, 4, 0, 0);
        mv->setHorizontalSpacing(8);
        mv->setVerticalSpacing(7);
        c.jogMinus = new QPushButton(QStringLiteral("− Jog"), c.moveRows);
        c.jogPlus = new QPushButton(QStringLiteral("Jog +"), c.moveRows);
        c.jogMinus->setObjectName(QStringLiteral("AxisJogMinus%1").arg(m));
        c.jogPlus->setObjectName(QStringLiteral("AxisJogPlus%1").arg(m));
        c.jogMinus->setToolTip(QStringLiteral("Hold to move toward −; let go to stop."));
        c.jogPlus->setToolTip(QStringLiteral("Hold to move toward +; let go to stop."));
        c.speed = new QDoubleSpinBox(c.moveRows);
        c.speed->setObjectName(QStringLiteral("AxisJogSpeed%1").arg(m));
        c.step = new QDoubleSpinBox(c.moveRows);
        c.step->setObjectName(QStringLiteral("AxisJogStep%1").arg(m));
        c.stepMinus = new QPushButton(QStringLiteral("− Step"), c.moveRows);
        c.stepPlus = new QPushButton(QStringLiteral("Step +"), c.moveRows);
        c.stepMinus->setObjectName(QStringLiteral("AxisJogStepMinus%1").arg(m));
        c.stepPlus->setObjectName(QStringLiteral("AxisJogStepPlus%1").arg(m));
        c.zero = new QPushButton(QStringLiteral("Zero"), c.moveRows);
        c.zero->setObjectName(QStringLiteral("AxisJogZero%1").arg(m));
        c.zero->setToolTip(a.role == Role::Axis ? QStringLiteral("This position becomes 0: the profile's start. The cam then runs from it.")
                                                : QStringLiteral("This position becomes 0."));
        c.toCam = new QPushButton(QStringLiteral("To cam position"), c.moveRows);
        c.toCam->setObjectName(QStringLiteral("AxisJogToCam%1").arg(m));
        mv->addWidget(Muted(QStringLiteral("Speed"), c.moveRows), 0, 0);
        mv->addWidget(c.speed, 0, 1);
        mv->addWidget(c.jogMinus, 0, 2);
        mv->addWidget(c.jogPlus, 0, 3);
        mv->addWidget(Muted(QStringLiteral("Step"), c.moveRows), 1, 0);
        mv->addWidget(c.step, 1, 1);
        mv->addWidget(c.stepMinus, 1, 2);
        mv->addWidget(c.stepPlus, 1, 3);
        if (a.role == Role::Free) {
            c.cpr = new QSpinBox(c.moveRows);
            c.cpr->setObjectName(QStringLiteral("AxisJogCpr%1").arg(m));
            c.cpr->setRange(1, 10000000);
            c.cpr->setSuffix(QStringLiteral(" counts/rev"));
            c.cpr->setValue(settings.value(Key(m, "cpr"), 800).toInt());
            c.cpr->setToolTip(QStringLiteral("The motor's step input resolution, as set in ClearPath-MSP (kept on this PC)."));
            mv->addWidget(Muted(QStringLiteral("Motor"), c.moveRows), 2, 0);
            mv->addWidget(c.cpr, 2, 1);
            mv->addWidget(c.zero, 2, 3);
            c.toCam->setVisible(false);
        } else {
            mv->addWidget(c.toCam, 2, 1);
            c.home = new QPushButton(QStringLiteral("Home"), c.moveRows);
            c.home->setObjectName(QStringLiteral("AxisJogHome%1").arg(m));
            c.home->setMinimumWidth(72);
            mv->addWidget(c.home, 2, 2);
            mv->addWidget(c.zero, 2, 3);
            connect(c.home, &QPushButton::clicked, this, [this, m]() {
                if (m >= m_motors.size()) return;
                const MotorInfo &a = m_motors[m];
                const bool homing = a.axis >= 0 && ((m_board->state().homing >> a.axis) & 1);
                send(homing ? QStringLiteral("axhome stop") : QStringLiteral("axhome %1").arg(m), [this](bool, const QStringList &) { poll(); });
            });
        }
        mv->setColumnStretch(1, 1);
        for (QPushButton *b : {c.jogMinus, c.jogPlus, c.stepMinus, c.stepPlus, c.zero}) b->setMinimumWidth(72);
        g->addWidget(c.moveRows, 3, 0, 1, 2);
        c.moveRows->setVisible(a.role != Role::Crank);

        c.note = new QLabel(c.frame);
        c.note->setObjectName(QStringLiteral("AxisJogFlag%1").arg(m));
        c.note->setWordWrap(true);
        c.note->setStyleSheet(QStringLiteral("QLabel { color: #f0c36a; }"));
        g->addWidget(c.note, 4, 0, 1, 2);
        g->setRowStretch(5, 1);

        grid->addWidget(c.frame, m / 2, m % 2);
        m_cards << c;
        applyRanges(m);
        if (settings.contains(Key(m, "speed"))) c.speed->setValue(settings.value(Key(m, "speed")).toDouble());
        c.step->setValue(settings.value(Key(m, "step"), 1.0).toDouble());

        connect(c.enable, &QPushButton::clicked, this, [this, m]() {
            if (m >= m_motors.size()) return;
            send(QStringLiteral("axjog %1 %2").arg(m).arg(m_motors[m].on ? QStringLiteral("disable") : QStringLiteral("enable")),
                 [this](bool, const QStringList &) { poll(); });
        });
        connect(c.jogMinus, &QPushButton::pressed, this, [this, m]() { startJog(m, -1); });
        connect(c.jogPlus, &QPushButton::pressed, this, [this, m]() { startJog(m, +1); });
        connect(c.jogMinus, &QPushButton::released, this, &JogDialog::stopJog);
        connect(c.jogPlus, &QPushButton::released, this, &JogDialog::stopJog);
        auto step = [this, m](int dir) {
            const double k = m_motors[m].role == Role::Free ? UnitCounts(m) : 1.0;  // board units: counts for a free motor
            send(QStringLiteral("axjog %1 move %2 %3")
                     .arg(m)
                     .arg(dir * m_cards[m].step->value() * k, 0, 'g', 12)
                     .arg(m_cards[m].speed->value() * k, 0, 'g', 12));
            saveSettings(m);
        };
        connect(c.stepMinus, &QPushButton::clicked, this, [step]() { step(-1); });
        connect(c.stepPlus, &QPushButton::clicked, this, [step]() { step(+1); });
        connect(c.zero, &QPushButton::clicked, this, [this, m]() { send(QStringLiteral("axjog %1 zero").arg(m)); });
        connect(c.toCam, &QPushButton::clicked, this, [this, m]() {
            send(QStringLiteral("axjog %1 cam %2").arg(m).arg(m_cards[m].speed->value(), 0, 'g', 12));
            saveSettings(m);
        });
        connect(c.speed, &QDoubleSpinBox::editingFinished, this, [this, m]() { saveSettings(m); });
        connect(c.step, &QDoubleSpinBox::editingFinished, this, [this, m]() { saveSettings(m); });
        if (c.cpr) {
            connect(c.cpr, &QSpinBox::valueChanged, this, [this, m](int) {
                applyRanges(m);
                saveSettings(m);
                refresh();
            });
        }
    }
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);
    adjustSize();
}

// Speed and step limits in the card's units.
void JogDialog::applyRanges(int m) {
    if (m < 0 || m >= m_cards.size() || m >= m_motors.size()) return;
    const MotorInfo &a = m_motors[m];
    Card &c = m_cards[m];
    const double k = UnitCounts(m);
    const QString u = Units(m);
    const int dec = std::max(1, Decimals(k));
    // The board's limit is in its units: the axis's, or counts for a free motor.
    const double vmax = std::max(a.role == Role::Free ? a.vmax / k : a.vmax, 1e-3);
    const double keep = c.speed->value();
    c.speed->setDecimals(dec);
    c.speed->setRange(std::pow(10.0, -dec), vmax);
    c.speed->setSuffix(QStringLiteral(" %1/s").arg(u));
    c.speed->setToolTip(QStringLiteral("Jog and step speed. The limit: %1 %2/s.").arg(Num(vmax, dec), u));
    double def = vmax * 0.05;
    const double mag = std::pow(10.0, std::floor(std::log10(std::max(def, 1e-6))));
    def = std::floor(def / mag) * mag;
    c.speed->setValue(keep > std::pow(10.0, -dec) && keep <= vmax ? keep : def);
    c.step->setDecimals(dec);
    c.step->setRange(1.0 / std::fabs(k), 1.0e6);
    c.step->setSuffix(QStringLiteral(" %1").arg(u));
}

void JogDialog::refresh() {
    const BoardLink::State &s = m_hasTestState ? m_testState : m_board->state();
    const bool conn = m_board->isConnected() || m_test;
    const bool running = s.state == QLatin1String("running") || s.state == QLatin1String("jogging") ||
                         s.timeline == QLatin1String("armed") || (!s.crank.isEmpty() && s.crank != QLatin1String("off"));
    const bool plc = s.plcInControl();
    const bool fault = s.valid && !s.fault.isEmpty() && s.fault != QLatin1String("none");
    m_camActive = conn && running;

    QString banner;
    if (!conn) {
        banner = QStringLiteral("Not connected to a board.");
    } else if (m_unsupported) {
        banner = QStringLiteral("This board's firmware has no axis jog.");
    } else if (plc) {
        banner = QStringLiteral("The PLC has control.");
    } else if (m_camActive) {
        banner = QStringLiteral("The cam is running.");
    } else if (fault) {
        banner = QStringLiteral("Fault: %1").arg(s.fault);
    }
    m_banner->setText(banner);
    m_banner->setVisible(!banner.isEmpty());
    m_stopCam->setVisible(conn && m_camActive);
    m_stopCam->setEnabled(!plc);
    m_clearFault->setVisible(conn && fault && !m_camActive);
    m_clearFault->setEnabled(!plc);
    m_disableAll->setEnabled(conn);
    m_homeAll->setVisible(s.hasHome != 0);
    m_homeAll->setText(s.homing != 0 ? QStringLiteral("Stop homing") : QStringLiteral("Home all"));

    const bool locked = !conn || m_unsupported || plc || m_camActive || fault;
    m_homeAll->setEnabled(s.homing != 0 ? conn : !locked && !s.simulate);
    if (locked && m_jogMotor >= 0) stopJog();
    for (int m = 0; m < m_cards.size() && m < m_motors.size(); ++m) {
        const MotorInfo &a = m_motors[m];
        Card &c = m_cards[m];
        const double k = UnitCounts(m);
        const int dec = Decimals(k);
        c.pos->setText(QStringLiteral("%1 %2").arg(Num(a.pos / k, dec), Units(m).toHtmlEscaped()));
        if (c.frame->property("motorOn").toBool() != a.on) {
            c.frame->setProperty("motorOn", a.on);
            c.frame->style()->unpolish(c.frame);
            c.frame->style()->polish(c.frame);
        }
        const bool canMove = !locked && (a.on || s.simulate) && a.role != Role::Crank;
        c.enable->setText(a.on ? QStringLiteral("Disable") : QStringLiteral("Enable"));
        c.enable->setEnabled(conn && (a.on || !locked));  // disabling is never locked
        c.jogMinus->setEnabled(canMove && !a.limitMinus && (!a.moving || m_jogMotor == m));
        c.jogPlus->setEnabled(canMove && !a.limitPlus && (!a.moving || m_jogMotor == m));
        c.stepMinus->setEnabled(canMove && !a.moving);
        c.stepPlus->setEnabled(canMove && !a.moving);
        c.zero->setEnabled(!locked && !a.moving && a.role != Role::Crank);
        c.toCam->setEnabled(canMove && !a.moving && a.hasCam && a.pos != a.cam);
        if (c.home) {
            const bool hasHome = a.axis >= 0 && ((s.hasHome >> a.axis) & 1);
            const bool homing = a.axis >= 0 && ((s.homing >> a.axis) & 1);
            c.home->setVisible(a.role == Role::Axis);
            c.home->setText(homing ? QStringLiteral("Stop") : QStringLiteral("Home"));
            c.home->setEnabled(homing ? conn : canMove && hasHome && !a.moving && !s.simulate);
            c.home->setToolTip(!hasHome ? QStringLiteral("No home switch on this axis (an input with the Home job).")
                               : homing ? QStringLiteral("Stop homing.")
                                        : QStringLiteral("Find the home switch and set the position there."));
        }
        c.toCam->setToolTip(a.hasCam ? QStringLiteral("Move back where the cam has this axis now, %1 %2, so Start runs the profile where "
                                                      "it is drawn.")
                                           .arg(Num(a.cam / k, dec), Units(m))
                                     : QStringLiteral("A gear or an open profile has no fixed cam position."));
        QString note;
        const bool homed = a.axis >= 0 && s.homed >= 0 && ((s.homed >> a.axis) & 1);
        const bool homingNow = a.axis >= 0 && ((s.homing >> a.axis) & 1);
        if (a.role == Role::Axis) {
            const QString base = QStringLiteral("Profile axis %1, in %2").arg(a.axis).arg(a.units);
            const bool hasHome = a.axis >= 0 && ((s.hasHome >> a.axis) & 1);
            c.role->setText(homingNow ? base + QStringLiteral(" · homing")
                            : !hasHome ? base
                            : homed    ? base + QStringLiteral(" · homed")
                                       : base + QStringLiteral(" · not homed"));
        }
        if (a.limitPlus || a.limitMinus) {
            note = a.limitPlus && a.limitMinus ? QStringLiteral("On both limits") : a.limitPlus ? QStringLiteral("On its + limit") : QStringLiteral("On its \u2212 limit");
        } else if (a.off != 0) {
            note = QStringLiteral("%1 %2 off its cam position").arg(Num(a.off / k, dec), Units(m));
        }
        c.note->setText(note);
        c.note->setVisible(!note.isEmpty());
    }
    QString note;
    if (!m_lastError.isEmpty() && m_errorClock.isValid() && m_errorClock.elapsed() < 8000) {
        note = m_lastError;
    } else if (s.simulate && conn) {
        note = QStringLiteral("Simulate is on.");
    }
    m_note->setText(note);
    m_note->setVisible(!note.isEmpty());
}

void JogDialog::startJog(int motor, int direction) {
    if (motor < 0 || motor >= m_cards.size()) return;
    if (m_jogMotor >= 0 && m_jogMotor != motor) stopJog();
    m_jogMotor = motor;
    m_jogDir = direction;
    saveSettings(motor);
    jogTick();
    m_jogTimer->start();
}

void JogDialog::jogTick() {
    if (m_jogMotor < 0 || m_jogInFlight) return;
    m_jogInFlight = true;
    QPointer<JogDialog> self(this);
    const double k = m_motors[m_jogMotor].role == Role::Free ? UnitCounts(m_jogMotor) : 1.0;
    const double v = m_jogDir * m_cards[m_jogMotor].speed->value() * k;
    send(QStringLiteral("axjog %1 jog %2").arg(m_jogMotor).arg(v, 0, 'g', 12), [self](bool ok, const QStringList &) {
        if (!self) return;
        self->m_jogInFlight = false;
        if (!ok) self->stopJog();  // refused: holding the button on doesn't help
    });
}

void JogDialog::stopJog() {
    if (m_jogTimer) m_jogTimer->stop();
    if (m_jogMotor < 0) return;
    const int motor = m_jogMotor;
    m_jogMotor = -1;
    m_jogDir = 0;
    send(QStringLiteral("axjog %1 stop").arg(motor));
}

void JogDialog::pressJog(int motor, int direction) { startJog(motor, direction); }
void JogDialog::releaseJog() { stopJog(); }

void JogDialog::saveSettings(int m) {
    if (m < 0 || m >= m_cards.size() || m_test) return;
    QSettings settings;
    settings.setValue(Key(m, "speed"), m_cards[m].speed->value());
    settings.setValue(Key(m, "step"), m_cards[m].step->value());
    if (m_cards[m].cpr) settings.setValue(Key(m, "cpr"), m_cards[m].cpr->value());
}

void JogDialog::reject() {
    stopJog();
    send(QStringLiteral("axjog all stop"));
    QDialog::reject();
}

void JogDialog::hideEvent(QHideEvent *event) {
    stopJog();
    QDialog::hideEvent(event);
}
