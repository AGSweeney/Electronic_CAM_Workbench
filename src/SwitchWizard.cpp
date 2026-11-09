/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "SwitchWizard.h"

#include "HelpWindow.h"
#include "JogDialog.h"
#include "ProjectIo.h"
#include "ProfileText.h"
#include "ProjectReview.h"
#include "Theme.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPointer>
#include <QPushButton>
#include <QRegularExpression>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

using namespace ecam;

namespace {

constexpr int kPollMs = 200;        // the Workbench's rate while it watches: a hand on a switch shows within 0.2 s
constexpr int kJogRepeatMs = 150;   // inside the board's 0.5 s deadman
enum Step { kStepAxis, kStepFind, kStepContact, kStepDirection, kStepLimits, kStepHome, kStepReview, kSteps };
const char *const kStepNames[] = {"Axis", "Find each switch", "Contact", "Direction", "Limits", "Home", "Review"};

QLabel *Muted(const QString &text, QWidget *parent) {
    auto *l = new QLabel(text, parent);
    l->setStyleSheet(QStringLiteral("QLabel { color: #8a929d; }"));
    return l;
}

}  // namespace

QString SwitchWizard::InputName(int input) {
    if (input < 0) return QStringLiteral("-");
    if (input < 100) return QString::fromStdString(BoardInputName(input));
    const int b = input - 100;
    return QString::fromStdString(CcioPinName(b / kCcioPinsPerBoard, b % kCcioPinsPerBoard));
}

SwitchWizard::Verdict SwitchWizard::Judge(int dir, int reached) {
    if (reached == kHome) return Verdict::Home;
    if (reached == (dir > 0 ? kLimitPlus : kLimitMinus)) return Verdict::Right;
    if (reached == (dir > 0 ? kLimitMinus : kLimitPlus)) return Verdict::Ask;
    return Verdict::Unknown;
}

SwitchWizard::SwitchWizard(BoardLink *board, const CamProject &project, int estopPin, Upload upload, QWidget *parent)
    : QDialog(parent), m_board(board), m_project(project), m_original(project), m_estopPin(estopPin), m_upload(std::move(upload)) {
    setWindowTitle(QStringLiteral("Set Up Limits and Homing"));
    setStyleSheet(theme::DialogSheet());
    setMinimumWidth(640);
    build();
    m_poll = new QTimer(this);
    m_poll->setInterval(kPollMs);
    connect(m_poll, &QTimer::timeout, this, &SwitchWizard::poll);
    m_poll->start();
    m_jogTimer = new QTimer(this);
    m_jogTimer->setInterval(kJogRepeatMs);
    connect(m_jogTimer, &QTimer::timeout, this, &SwitchWizard::jogTick);
    connect(m_board, &BoardLink::stateUpdated, this, &SwitchWizard::onState);
    go(kStepAxis);
}

SwitchWizard::~SwitchWizard() {
    if (m_jogTimer) m_jogTimer->stop();
}

int SwitchWizard::axis() const { return m_axis ? std::max(0, m_axis->currentIndex()) : 0; }

int SwitchWizard::motor() const {
    const int a = axis();
    return a < static_cast<int>(m_project.axes.size()) ? m_project.axes[a].connector : 0;
}

QString SwitchWizard::switchName(int s) const {
    return s == kLimitPlus ? QStringLiteral("Limit +") : s == kLimitMinus ? QStringLiteral("Limit −") : QStringLiteral("Home");
}

QString SwitchWizard::message() const { return m_note ? m_note->text() : QString(); }

QLabel *SwitchWizard::nameLabel(int s, QWidget *page) {
    auto *l = new QLabel(switchName(s), page);
    m_names[s].push_back(l);
    return l;
}

void SwitchWizard::build() {
    auto *outer = new QVBoxLayout(this);
    outer->setContentsMargins(16, 14, 16, 14);
    outer->setSpacing(10);
    m_title = new QLabel(this);
    m_title->setObjectName(QStringLiteral("SwitchWizardTitle"));
    m_title->setStyleSheet(QStringLiteral("QLabel { font-size: 15px; font-weight: 600; }"));
    outer->addWidget(m_title);
    m_pages = new QStackedWidget(this);
    outer->addWidget(m_pages, 1);

    // Axis
    {
        auto *page = new QWidget(m_pages);
        auto *f = new QFormLayout(page);
        f->setSpacing(10);
        m_axis = new QComboBox(page);
        m_axis->setObjectName(QStringLiteral("SwitchWizardAxis"));
        for (const CamAxis &a : m_project.axes) m_axis->addItem(QStringLiteral("%1  (M-%2)").arg(QString::fromStdString(a.name)).arg(a.connector));
        f->addRow(QStringLiteral("Axis"), m_axis);
        auto *switches = new QHBoxLayout();
        for (int s = 0; s < kSwitches; ++s) {
            m_want[s] = new QCheckBox(switchName(s), page);
            m_want[s]->setObjectName(QStringLiteral("SwitchWizardWant%1").arg(s));
            m_want[s]->setChecked(s != kHome);
            switches->addWidget(m_want[s]);
        }
        switches->addStretch(1);
        f->addRow(QStringLiteral("Switches"), switches);
        m_speed = new QDoubleSpinBox(page);
        m_speed->setObjectName(QStringLiteral("SwitchWizardSpeed"));
        m_speed->setDecimals(3);
        f->addRow(QStringLiteral("Setup speed"), m_speed);
        m_ready = new QLabel(page);
        m_ready->setObjectName(QStringLiteral("SwitchWizardReady"));
        m_ready->setWordWrap(true);
        f->addRow(m_ready);
        auto setSpeed = [this]() {
            const CamAxis &a = m_project.axes[axis()];
            const double cpu = std::fabs(a.countsPerUnit) > 0 ? std::fabs(a.countsPerUnit) : 1.0;
            const double vmax = a.velMax > 0 ? std::min(a.velMax, 53333.0 / cpu) : 53333.0 / cpu;
            m_speed->setRange(0.001, vmax);
            m_speed->setSuffix(QStringLiteral(" %1/s").arg(QString::fromStdString(a.units)));
            m_speed->setValue(std::max(0.001, vmax * 0.1));
        };
        connect(m_axis, &QComboBox::currentIndexChanged, this, [this, setSpeed](int) {
            setSpeed();
            for (Found &f : m_found) f = Found();
            refresh();
        });
        for (QCheckBox *c : m_want) connect(c, &QCheckBox::toggled, this, [this](bool) { refresh(); });
        if (!m_project.axes.empty()) setSpeed();
        m_pages->addWidget(page);
    }
    // Find
    {
        auto *page = new QWidget(m_pages);
        auto *g = new QGridLayout(page);
        g->setVerticalSpacing(10);
        for (int s = 0; s < kSwitches; ++s) {
            g->addWidget(nameLabel(s, page), s, 0);
            m_findRows[s] = new QLabel(page);
            m_findRows[s]->setObjectName(QStringLiteral("SwitchWizardFind%1").arg(s));
            g->addWidget(m_findRows[s], s, 1);
        }
        auto *again = new QPushButton(QStringLiteral("Find again"), page);
        again->setObjectName(QStringLiteral("SwitchWizardFindAgain"));
        g->addWidget(again, kSwitches, 0);
        g->setColumnStretch(1, 1);
        g->setRowStretch(kSwitches + 1, 1);
        connect(again, &QPushButton::clicked, this, [this]() {
            for (Found &f : m_found) {
                f.input = -1;
                f.proved = false;
            }
            m_baseBoard = -1;
            m_candidate = -1;
            m_findNote.clear();
            refresh();
        });
        m_pages->addWidget(page);
    }
    // Contact
    {
        auto *page = new QWidget(m_pages);
        auto *g = new QGridLayout(page);
        g->setVerticalSpacing(10);
        g->addWidget(Muted(QStringLiteral("Switch"), page), 0, 0);
        g->addWidget(Muted(QStringLiteral("Input, at rest"), page), 0, 1);
        g->addWidget(Muted(QStringLiteral("Contact"), page), 0, 2);
        for (int s = 0; s < kSwitches; ++s) {
            g->addWidget(nameLabel(s, page), s + 1, 0);
            m_contactRows[s] = new QLabel(page);
            g->addWidget(m_contactRows[s], s + 1, 1);
            m_contact[s] = new QComboBox(page);
            m_contact[s]->setObjectName(QStringLiteral("SwitchWizardContact%1").arg(s));
            m_contact[s]->addItems({QStringLiteral("Normally closed"), QStringLiteral("Normally open")});
            g->addWidget(m_contact[s], s + 1, 2);
            connect(m_contact[s], &QComboBox::currentIndexChanged, this, [this, s](int k) {
                m_found[s].normallyOpen = k == 1;
                refresh();
            });
        }
        m_contactWarn = new QLabel(page);
        m_contactWarn->setWordWrap(true);
        m_contactWarn->setStyleSheet(QStringLiteral("QLabel { color: #ffcc80; }"));
        g->addWidget(m_contactWarn, kSwitches + 1, 0, 1, 3);
        g->setColumnStretch(1, 1);
        g->setRowStretch(kSwitches + 2, 1);
        m_pages->addWidget(page);
    }
    // Direction
    {
        auto *page = new QWidget(m_pages);
        auto *v = new QVBoxLayout(page);
        v->setSpacing(10);
        auto *row = new QHBoxLayout();
        m_enable = new QPushButton(QStringLiteral("Enable"), page);
        m_enable->setObjectName(QStringLiteral("SwitchWizardEnable"));
        m_jogMinus = new QPushButton(QStringLiteral("− Jog"), page);
        m_jogPlus = new QPushButton(QStringLiteral("Jog +"), page);
        m_jogMinus->setObjectName(QStringLiteral("SwitchWizardJogMinus"));
        m_jogPlus->setObjectName(QStringLiteral("SwitchWizardJogPlus"));
        for (QPushButton *b : {m_enable, m_jogMinus, m_jogPlus}) b->setMinimumWidth(90);
        row->addWidget(m_enable);
        row->addSpacing(16);
        row->addWidget(m_jogMinus);
        row->addWidget(m_jogPlus);
        row->addStretch(1);
        v->addLayout(row);
        auto *g = new QGridLayout();
        g->setVerticalSpacing(8);
        for (int s = 0; s < kSwitches; ++s) {
            g->addWidget(nameLabel(s, page), s, 0);
            m_dirRows[s] = new QLabel(page);
            m_dirRows[s]->setObjectName(QStringLiteral("SwitchWizardDir%1").arg(s));
            g->addWidget(m_dirRows[s], s, 1);
        }
        g->setColumnStretch(1, 1);
        v->addLayout(g);
        m_question = new QLabel(page);
        m_question->setObjectName(QStringLiteral("SwitchWizardQuestion"));
        m_question->setWordWrap(true);
        m_question->setStyleSheet(QStringLiteral("QLabel { color: #ffcc80; font-weight: 600; }"));
        auto *qrow = new QHBoxLayout();
        m_yes = new QPushButton(QStringLiteral("Yes"), page);
        m_no = new QPushButton(QStringLiteral("No"), page);
        m_yes->setObjectName(QStringLiteral("SwitchWizardYes"));
        m_no->setObjectName(QStringLiteral("SwitchWizardNo"));
        qrow->addWidget(m_question, 1);
        qrow->addWidget(m_yes);
        qrow->addWidget(m_no);
        v->addLayout(qrow);
        v->addStretch(1);
        connect(m_enable, &QPushButton::clicked, this, &SwitchWizard::toggleEnable);
        connect(m_jogMinus, &QPushButton::pressed, this, [this]() { startJog(-1); });
        connect(m_jogPlus, &QPushButton::pressed, this, [this]() { startJog(+1); });
        connect(m_jogMinus, &QPushButton::released, this, &SwitchWizard::stopJog);
        connect(m_jogPlus, &QPushButton::released, this, &SwitchWizard::stopJog);
        connect(m_yes, &QPushButton::clicked, this, [this]() { answer(true); });
        connect(m_no, &QPushButton::clicked, this, [this]() { answer(false); });
        m_pages->addWidget(page);
    }
    // Limits
    {
        auto *page = new QWidget(m_pages);
        auto *v = new QVBoxLayout(page);
        v->setSpacing(10);
        auto *row = new QHBoxLayout();
        m_limEnable = new QPushButton(QStringLiteral("Enable"), page);
        m_limEnable->setObjectName(QStringLiteral("SwitchWizardLimEnable"));
        m_limMinus = new QPushButton(QStringLiteral("\u2212 Jog"), page);
        m_limPlus = new QPushButton(QStringLiteral("Jog +"), page);
        m_limMinus->setObjectName(QStringLiteral("SwitchWizardLimMinus"));
        m_limPlus->setObjectName(QStringLiteral("SwitchWizardLimPlus"));
        for (QPushButton *b : {m_limEnable, m_limMinus, m_limPlus}) b->setMinimumWidth(90);
        row->addWidget(m_limEnable);
        row->addSpacing(16);
        row->addWidget(m_limMinus);
        row->addWidget(m_limPlus);
        row->addStretch(1);
        v->addLayout(row);
        auto *g = new QGridLayout();
        g->setVerticalSpacing(8);
        for (int s = 0; s < 2; ++s) {
            g->addWidget(nameLabel(s, page), s, 0);
            m_limRows[s] = new QLabel(page);
            m_limRows[s]->setObjectName(QStringLiteral("SwitchWizardLimit%1").arg(s));
            m_limRows[s]->setWordWrap(true);
            g->addWidget(m_limRows[s], s, 1);
        }
        g->setColumnStretch(1, 1);
        v->addLayout(g);
        m_limTravel = new QLabel(page);
        m_limTravel->setObjectName(QStringLiteral("SwitchWizardTravel"));
        m_limTravel->setWordWrap(true);
        v->addWidget(m_limTravel);
        v->addStretch(1);
        connect(m_limEnable, &QPushButton::clicked, this, &SwitchWizard::toggleEnable);
        connect(m_limMinus, &QPushButton::pressed, this, [this]() { startJog(-1); });
        connect(m_limPlus, &QPushButton::pressed, this, [this]() { startJog(+1); });
        connect(m_limMinus, &QPushButton::released, this, &SwitchWizard::stopJog);
        connect(m_limPlus, &QPushButton::released, this, &SwitchWizard::stopJog);
        m_pages->addWidget(page);
    }
    // Home
    {
        auto *page = new QWidget(m_pages);
        auto *v = new QVBoxLayout(page);
        v->setSpacing(10);
        auto *cols = new QHBoxLayout();
        cols->setSpacing(24);
        auto *f1 = new QFormLayout();
        auto *f2 = new QFormLayout();
        f1->setSpacing(8);
        f2->setSpacing(8);
        cols->addLayout(f1, 1);
        cols->addLayout(f2, 1);
        v->addLayout(cols);
        auto spin = [page](const char *name, double lo) {
            auto *s = new QDoubleSpinBox(page);
            s->setObjectName(QLatin1String(name));
            s->setRange(lo, 1e9);
            s->setDecimals(3);
            s->setKeyboardTracking(false);
            return s;
        };
        m_hFast = spin("SwitchWizardHomeFast", 0.001);
        m_hSlow = spin("SwitchWizardHomeSlow", 0.001);
        m_hBack = spin("SwitchWizardHomeBack", 0.0);
        m_hSearch = spin("SwitchWizardHomeSearch", 0.0);
        m_hSearch->setSpecialValueText(QStringLiteral("until the switch"));
        m_hAt = spin("SwitchWizardHomeAt", -1e9);
        m_hDecel = spin("SwitchWizardHomeDecel", 0.0);
        m_hDecel->setMaximum(1e12);
        m_hDecel->setDecimals(1);
        m_hDecel->setSpecialValueText(QStringLiteral("as axis jog"));
        m_hThen = new QComboBox(page);
        m_hThen->setObjectName(QStringLiteral("SwitchWizardHomeThen"));
        m_hThen->addItems({QStringLiteral("Move to the cam's start"), QStringLiteral("Stay at the switch")});
        m_hRequired = new QCheckBox(QStringLiteral("Required before Start"), page);
        m_hRequired->setObjectName(QStringLiteral("SwitchWizardHomeRequired"));
        m_hRuns = new QSpinBox(page);
        m_hRuns->setObjectName(QStringLiteral("SwitchWizardHomeRuns"));
        m_hRuns->setRange(1, 20);
        m_hRuns->setValue(5);
        f1->addRow(QStringLiteral("Search speed"), m_hFast);
        f1->addRow(QStringLiteral("Latch speed"), m_hSlow);
        f1->addRow(QStringLiteral("Back-off"), m_hBack);
        f1->addRow(QStringLiteral("Search length"), m_hSearch);
        f1->addRow(QStringLiteral("Decel"), m_hDecel);
        f2->addRow(QStringLiteral("Home position"), m_hAt);
        f2->addRow(QStringLiteral("Then"), m_hThen);
        f2->addRow(m_hRequired);
        f2->addRow(QStringLiteral("Homings"), m_hRuns);
        auto *row = new QHBoxLayout();
        m_homeEnable = new QPushButton(QStringLiteral("Enable"), page);
        m_homeEnable->setObjectName(QStringLiteral("SwitchWizardHomeEnable"));
        m_homeGo = new QPushButton(QStringLiteral("Home"), page);
        m_homeGo->setObjectName(QStringLiteral("SwitchWizardHomeGo"));
        for (QPushButton *b : {m_homeEnable, m_homeGo}) b->setMinimumWidth(90);
        row->addWidget(m_homeEnable);
        row->addWidget(m_homeGo);
        row->addStretch(1);
        v->addLayout(row);
        m_homeResults = new QLabel(page);
        m_homeResults->setObjectName(QStringLiteral("SwitchWizardHomeResults"));
        m_homeResults->setWordWrap(true);
        m_homeResults->setTextFormat(Qt::RichText);
        v->addWidget(m_homeResults);
        v->addStretch(1);
        // A changed setting goes up again before the next homing, and the results so far no longer count.
        auto changed = [this]() {
            if (!m_homeSet) return;
            m_home.fast = m_hFast->value();
            m_home.slow = std::min(m_hSlow->value(), m_home.fast);
            m_home.back = m_hBack->value();
            m_home.search = m_hSearch->value();
            m_home.decel = m_hDecel->value();
            m_home.at = m_hAt->value();
            m_home.thenStart = m_hThen->currentIndex() == 0;
            m_home.required = m_hRequired->isChecked();
            m_homeUp = false;
            m_runsDone = 0;
            m_devCounts.clear();
            m_homeFailed = false;
            refresh();
        };
        for (QDoubleSpinBox *s : {m_hFast, m_hSlow, m_hBack, m_hSearch, m_hAt, m_hDecel}) connect(s, &QDoubleSpinBox::valueChanged, this, changed);
        connect(m_hThen, &QComboBox::currentIndexChanged, this, changed);
        connect(m_hRequired, &QCheckBox::toggled, this, changed);
        connect(m_hRuns, &QSpinBox::valueChanged, this, [this](int v) {
            m_runsWanted = v;
            refresh();
        });
        connect(m_homeEnable, &QPushButton::clicked, this, &SwitchWizard::toggleEnable);
        connect(m_homeGo, &QPushButton::clicked, this, [this]() {
            if (m_homeRunning) {
                m_homeRunning = false;
                m_homeNote = QStringLiteral("Stopped.");
                m_board->command(QStringLiteral("axhome stop"));
                refresh();
                return;
            }
            auto start = [this]() {
                m_runsDone = 0;
                m_devCounts.clear();
                m_homeFailed = false;
                m_homeNote.clear();
                m_homeRunning = true;
                m_endsKnown = false;
                QPointer<SwitchWizard> self(this);
                // The homings so far, then the first one.
                m_board->command(QStringLiteral("axhome"), [self](bool ok, const QStringList &r) {
                    if (!self) return;
                    if (ok) self->feedAxhome(r);
                    self->homeRun();
                });
                refresh();
            };
            if (m_homeUp) {
                start();
            } else {
                uploadThen(QStringLiteral("Upload the switches and these homing settings to the board, then home the axis?"), [this, start]() {
                    m_homeUp = true;
                    start();
                });
            }
        });
        m_pages->addWidget(page);
    }
    // Review
    {
        auto *page = new QWidget(m_pages);
        auto *v = new QVBoxLayout(page);
        m_review = new QLabel(page);
        m_review->setObjectName(QStringLiteral("SwitchWizardReview"));
        m_review->setWordWrap(true);
        m_review->setTextFormat(Qt::RichText);
        v->addWidget(m_review);
        v->addStretch(1);
        m_pages->addWidget(page);
    }

    m_note = new QLabel(this);
    m_note->setObjectName(QStringLiteral("SwitchWizardNote"));
    m_note->setWordWrap(true);
    m_note->setStyleSheet(QStringLiteral("QLabel { color: #9fa6af; }"));
    outer->addWidget(m_note);
    auto *buttons = new QHBoxLayout();
    auto *help = new QPushButton(QStringLiteral("Help"), this);
    m_back = new QPushButton(QStringLiteral("Back"), this);
    m_next = new QPushButton(QStringLiteral("Next"), this);
    m_next->setObjectName(QStringLiteral("SwitchWizardNext"));
    auto *cancel = new QPushButton(QStringLiteral("Cancel"), this);
    buttons->addWidget(help);
    buttons->addStretch(1);
    buttons->addWidget(m_back);
    buttons->addWidget(m_next);
    buttons->addWidget(cancel);
    outer->addLayout(buttons);
    connect(help, &QPushButton::clicked, this, [this]() { HelpWindow::Show(this, QStringLiteral("09-board.html#switchwizard")); });
    connect(cancel, &QPushButton::clicked, this, &SwitchWizard::reject);
    connect(m_back, &QPushButton::clicked, this, [this]() { go(nextStep(m_step, -1)); });
    connect(m_next, &QPushButton::clicked, this, [this]() {
        if (m_step < kStepReview) {
            const int n = nextStep(m_step, +1);
            if (m_step == kStepDirection && (n == kStepLimits || n == kStepHome)) {
                // The board's own limits and homing need the switches: up they go first.
                uploadThen(QStringLiteral("Upload the switches found to the board, so its limits and homing can be tested?"),
                           [this, n]() { go(n); });
                return;
            }
            go(n);
            return;
        }
        if (QMessageBox::question(this, windowTitle(), QStringLiteral("Write these switches into the project and upload it?")) != QMessageBox::Yes) return;
        m_uploading = true;
        refresh();
        QPointer<SwitchWizard> self(this);
        m_upload(result(), [self](bool ok, const QString &msg) {
            if (!self) return;
            self->m_uploading = false;
            if (ok) {
                self->accept();
            } else {
                self->m_note->setText(QStringLiteral("Upload: %1").arg(msg));
                self->refresh();
            }
        });
    });
}

bool SwitchWizard::applies(int step) const {
    if (step == kStepLimits) return m_want[kLimitPlus]->isChecked() || m_want[kLimitMinus]->isChecked();
    if (step == kStepHome) return m_want[kHome]->isChecked();
    return step >= 0 && step < kSteps;
}

int SwitchWizard::nextStep(int from, int dir) const {
    int s = from + dir;
    while (s > 0 && s < kSteps - 1 && !applies(s)) s += dir;
    return std::clamp(s, 0, kSteps - 1);
}

void SwitchWizard::go(int step) {
    stopJog();
    m_step = std::clamp(step, 0, kSteps - 1);
    m_pages->setCurrentIndex(m_step);
    int index = 0, count = 0;
    for (int s = 0; s < kSteps; ++s) {
        if (!applies(s)) continue;
        ++count;
        if (s <= m_step) index = count;
    }
    m_title->setText(QStringLiteral("%1 of %2: %3").arg(index).arg(count).arg(QLatin1String(kStepNames[m_step])));
    for (int s = 0; s < kSwitches; ++s) m_found[s].wanted = m_want[s]->isChecked();
    if (m_step == kStepLimits) {
        // Counted from here: a trip already on the board isn't this test's.
        m_tripBaseKnown = false;
        for (int s = 0; s < 2; ++s) {
            m_limStopped[s] = m_limCleared[s] = false;
            m_overCounts[s] = -1;
        }
        m_limNote.clear();
    }
    if (m_step == kStepHome && !m_homeSet) {
        // The axis's own settings if it had a home switch; else from the setup speed. The direction is the one seen.
        const int ax = axis();
        AxisHome h = m_original.axes[ax].home;
        if (!HasHomeSwitch(m_original, ax)) {
            h = AxisHome();
            h.fast = m_speed->value();
            h.slow = h.fast / 10.0;
            h.back = h.fast * 0.25;
            h.thenStart = true;
        }
        if (m_homeSide) h.dir = m_homeSide;
        // The search: a little over the travel between the limits, when the Limits step measured it.
        if (h.search <= 0.0 && m_limStopped[0] && m_limStopped[1]) {
            const double cpu = std::fabs(m_project.axes[ax].countsPerUnit) > 0 ? std::fabs(m_project.axes[ax].countsPerUnit) : 1.0;
            h.search = std::ceil(std::llabs(m_tripAt[0] - m_tripAt[1]) / cpu * 1.1);
        }
        m_home = h;
        const QString u = QStringLiteral(" ") + QString::fromStdString(m_project.axes[ax].units);
        QSignalBlocker b1(m_hFast), b2(m_hSlow), b3(m_hBack), b4(m_hSearch), b5(m_hAt), b6(m_hThen), b7(m_hRequired), b8(m_hDecel);
        const double vmax = AxisJogLimit(m_project.axes[ax]);
        m_hFast->setMaximum(vmax);
        m_hSlow->setMaximum(vmax);
        m_hFast->setValue(h.fast);
        m_hSlow->setValue(h.slow);
        m_hBack->setValue(h.back);
        m_hSearch->setValue(h.search);
        m_hAt->setValue(h.at);
        m_hDecel->setValue(h.decel);
        m_hDecel->setSuffix(u + QStringLiteral("/s\u00b2"));
        m_hThen->setCurrentIndex(h.thenStart ? 0 : 1);
        m_hRequired->setChecked(h.required);
        m_hFast->setSuffix(u + QStringLiteral("/s"));
        m_hSlow->setSuffix(u + QStringLiteral("/s"));
        for (QDoubleSpinBox *s : {m_hBack, m_hSearch, m_hAt}) s->setSuffix(u);
        m_homeSet = true;
        m_homeUp = false;
    }
    if (m_step == kStepFind) {
        m_baseBoard = -1;
        m_candidate = -1;
        m_findNote.clear();
    }
    if (m_step == kStepContact) {
        // The contact from the level at rest: on is normally closed (the switch holds the input on until it trips).
        for (int s = 0; s < kSwitches; ++s) {
            if (!m_found[s].wanted || m_found[s].input < 0) continue;
            QSignalBlocker b(m_contact[s]);
            m_contact[s]->setCurrentIndex(m_found[s].normallyOpen ? 1 : 0);
        }
    }
    if (m_board->isConnected()) m_board->command(QStringLiteral("status"));
    refresh();
}

void SwitchWizard::setStepForTest(int step) { go(step); }

void SwitchWizard::poll() {
    if (!m_board->isConnected() || m_pollBusy) return;
    m_pollBusy = true;
    QPointer<SwitchWizard> self(this);
    m_board->command(QStringLiteral("status"), [self](bool, const QStringList &) {
        if (self) self->m_pollBusy = false;
    });
    if (m_step == kStepDirection) {
        m_board->command(QStringLiteral("axjog"), [self](bool ok, const QStringList &r) {
            if (self && ok) self->onAxjog(r);
        });
    } else if (m_step == kStepLimits) {
        m_board->command(QStringLiteral("axlimit"), [self](bool ok, const QStringList &r) {
            if (self && ok) self->feedAxlimit(r, ((self->m_board->state().jogMoving >> self->motor()) & 1u) != 0);
        });
    } else if (m_step == kStepHome && m_homeRunning) {
        m_board->command(QStringLiteral("axhome"), [self](bool ok, const QStringList &r) {
            if (self && ok) self->feedAxhome(r);
        });
    }
}

void SwitchWizard::toggleEnable() {
    m_board->command(QStringLiteral("axjog %1 %2").arg(motor()).arg(m_motorOn ? QStringLiteral("disable") : QStringLiteral("enable")),
                     [this](bool ok, const QStringList &r) {
                         const QString why = ok || r.isEmpty() ? QString() : r.last();
                         if (m_step == kStepDirection) m_dirNote = why;
                         else if (m_step == kStepLimits) m_limNote = why;
                         else m_homeNote = why;
                         m_board->command(QStringLiteral("status"));
                         refresh();
                     });
}

// Asks, then puts result() into the project and on the board; then() once the board has it. Stays put on No or a refusal.
void SwitchWizard::uploadThen(const QString &ask, std::function<void()> then) {
    if (QMessageBox::question(this, windowTitle(), ask) != QMessageBox::Yes) return;
    m_uploading = true;
    refresh();
    QPointer<SwitchWizard> self(this);
    const CamProject p = result();
    m_upload(p, [self, p, then](bool ok, const QString &msg) {
        if (!self) return;
        self->m_uploading = false;
        if (!ok) {
            self->m_note->setText(QStringLiteral("Upload: %1").arg(msg));
            self->refresh();
            return;
        }
        self->m_project = p;  // the board has it now: the next steps build on it
        then();
        self->refresh();
    });
}

// The Limits step: the board's axlimit line for this axis. A trip count past the step's start is a stop by that limit;
// at rest, the distance from where it tripped is the overtravel; the limit reading off again is it cleared.
void SwitchWizard::feedAxlimit(const QStringList &lines, bool moving) {
    static const QRegularExpression re(QStringLiteral(
        "^axlimit (\\d+) M-\\d+ lim=(\\d)(\\d) tripp=(\\d+) atp=(-?\\d+) tripm=(\\d+) atm=(-?\\d+) pos=(-?\\d+)"));
    for (const QString &l : lines) {
        const QRegularExpressionMatch m = re.match(l);
        if (!m.hasMatch() || m.captured(1).toInt() != axis()) continue;
        const bool lim[2] = {m.captured(2) == QLatin1String("1"), m.captured(3) == QLatin1String("1")};
        const quint32 trips[2] = {m.captured(4).toUInt(), m.captured(6).toUInt()};
        const qint64 at[2] = {m.captured(5).toLongLong(), m.captured(7).toLongLong()};
        const qint64 pos = m.captured(8).toLongLong();
        if (!m_tripBaseKnown) {
            m_tripBaseKnown = true;
            m_tripBase[0] = trips[0];
            m_tripBase[1] = trips[1];
        }
        for (int s = 0; s < 2; ++s) {
            if (trips[s] != m_tripBase[s]) {
                m_tripBase[s] = trips[s];
                m_limStopped[s] = true;
                m_limCleared[s] = false;
                m_tripAt[s] = at[s];
                m_overCounts[s] = -1;
            }
            if (m_limStopped[s] && m_overCounts[s] < 0 && !moving) m_overCounts[s] = std::llabs(pos - m_tripAt[s]);
            if (m_limStopped[s] && m_overCounts[s] >= 0 && !lim[s]) m_limCleared[s] = true;
        }
    }
    refresh();
}

// The Home step: the board's axhome line for this axis. A homing that ended since the last look is one result.
void SwitchWizard::feedAxhome(const QStringList &lines) {
    static const QRegularExpression re(QStringLiteral(
        "^axhome (\\d+) M-\\d+ has=(\\d) homed=(\\d) homing=(\\d) ends=(\\d+) latch=(-?\\d+) pos=(-?\\d+) last=(\\S+)"));
    for (const QString &l : lines) {
        const QRegularExpressionMatch m = re.match(l);
        if (!m.hasMatch() || m.captured(1).toInt() != axis()) continue;
        const quint32 ends = m.captured(5).toUInt();
        if (!m_endsKnown) {
            m_endsKnown = true;
            m_endsBase = ends;
            continue;
        }
        if (ends == m_endsBase || !m_homeRunning) continue;
        m_endsBase = ends;
        const QString last = m.captured(8);
        if (last != QLatin1String("ok")) {
            m_homeRunning = false;
            m_homeFailed = last != QLatin1String("stopped");
            const QString fault = m_board->state().fault;
            m_homeNote = last == QLatin1String("stopped") ? QStringLiteral("Stopped.")
                         : !fault.isEmpty() && fault != QLatin1String("none") ? fault
                                                                             : QStringLiteral("Homing failed (%1).").arg(last);
            continue;
        }
        // The first homing sets where the switch is (the home position); each one after latches against it.
        const double cpu = m_project.axes[axis()].countsPerUnit;
        const qint64 atCounts = std::llround(m_home.at * cpu);
        m_devCounts << (m_runsDone == 0 ? 0 : m.captured(6).toLongLong() - atCounts);
        ++m_runsDone;
        if (m_runsDone < m_runsWanted) {
            homeRun();
        } else {
            m_homeRunning = false;
        }
    }
    refresh();
}

void SwitchWizard::homeRun() {
    if (!m_homeRunning || m_homeTest) return;  // a test feeds the results itself
    QPointer<SwitchWizard> self(this);
    m_board->command(QStringLiteral("axhome %1").arg(motor()), [self](bool ok, const QStringList &r) {
        if (!self || ok) return;
        self->m_homeRunning = false;
        self->m_homeNote = r.isEmpty() ? QStringLiteral("refused") : r.last();
        self->refresh();
    });
}

void SwitchWizard::startHomeRunsForTest(int runs) {
    m_runsWanted = runs;
    m_runsDone = 0;
    m_devCounts.clear();
    m_homeFailed = false;
    m_homeRunning = true;
    m_endsKnown = false;
    m_homeUp = true;
    m_homeTest = true;
}

double SwitchWizard::homeSpreadCounts() const {
    if (m_devCounts.isEmpty()) return 0.0;
    const auto mm = std::minmax_element(m_devCounts.begin(), m_devCounts.end());
    return static_cast<double>(*mm.second - *mm.first);
}

void SwitchWizard::onState(const BoardLink::State &s) {
    m_motorOn = s.valid && ((s.motorsOn >> motor()) & 1u);
    if (s.valid && s.inputs >= 0) {
        const int bits = s.ccioOff || s.ccioBroken ? 0 : s.ccioBoards * kCcioPinsPerBoard;
        feedLevels(s.inputs, s.ccioInputs, bits);
    } else {
        refresh();
    }
}

int SwitchWizard::levelOf(int input, int board, quint64 ccio) const {
    if (input < 0) return -1;
    if (input < 100) return (board >> input) & 1;
    return static_cast<int>((ccio >> (input - 100)) & 1u);
}

// Why this input can't be one of this axis's switches, empty when it can.
QString SwitchWizard::blockedWhy(int input) const {
    const QString name = InputName(input);
    for (int s = 0; s < kSwitches; ++s) {
        if (m_found[s].input == input) return QStringLiteral("%1 is already the %2 switch").arg(name, switchName(s));
    }
    const int ax = axis();
    static const char *const kJobs[] = {"", "home", "+ limit", "− limit"};
    if (input < 100) {
        static const char *const kOp[] = {"Enable", "Disable", "Fault reset", "Start", "Stop"};
        for (int k = 0; k < 5; ++k) {
            if (m_opPins[k] == input) return QStringLiteral("%1 is the board's operator %2 input").arg(name, QLatin1String(kOp[k]));
        }
        const std::string why = BoardInputBlocked(m_project, input, m_estopPin);
        if (why == "estop") return QStringLiteral("%1 is the board's estop input").arg(name);
        if (why == "output") return QStringLiteral("%1 is an output in this project").arg(name);
        if (why == "trigger") return QStringLiteral("%1 is the timeline's trigger input").arg(name);
        if (why == "encoder") return QStringLiteral("%1 is the Encoder Adapter's").arg(name);
        const BoardInput in = BoardInputAt(m_project, input);
        if (in.role != CcioRole::None && in.axis != ax && in.axis >= 0) {
            return QStringLiteral("%1 is already %2's %3").arg(name, QString::fromStdString(m_project.axes[in.axis].name),
                                                                QString::fromUtf8(kJobs[static_cast<int>(in.role)]));
        }
    } else {
        const int b = input - 100;
        if (b / kCcioPinsPerBoard >= m_project.ccio.boards) {
            return QStringLiteral("%1 is on a CCIO-8 this project doesn't have (CCIO-8 page)").arg(name);
        }
        const CcioInput in = CcioInputAt(m_project.ccio, b / kCcioPinsPerBoard, b % kCcioPinsPerBoard);
        if (in.role != CcioRole::None && in.axis != ax && in.axis >= 0) {
            return QStringLiteral("%1 is already %2's %3").arg(name, QString::fromStdString(m_project.axes[in.axis].name),
                                                                QString::fromUtf8(kJobs[static_cast<int>(in.role)]));
        }
    }
    return QString();
}

// Find: the levels when it started looking; then one input that changes, and comes back, is the switch.
void SwitchWizard::feedLevels(int board, quint64 ccio, int ccioBits) {
    m_lastBoard = board;
    m_lastCcio = ccio;
    m_ccioBits = ccioBits;
    if (m_step != kStepFind) {
        refresh();
        return;
    }
    m_finding = -1;
    for (int s = 0; s < kSwitches; ++s) {
        if (m_found[s].wanted && m_found[s].input < 0) {
            m_finding = s;
            break;
        }
    }
    if (m_finding < 0) {
        refresh();
        return;
    }
    if (m_baseBoard < 0) {
        m_baseBoard = board;
        m_baseCcio = ccio;
        refresh();
        return;
    }
    QList<int> changed;
    for (int i = 0; i < kTriggerInputs; ++i) {
        if (((board ^ m_baseBoard) >> i) & 1) changed << i;
    }
    for (int i = 0; i < ccioBits && i < 64; ++i) {
        if (((ccio ^ m_baseCcio) >> i) & 1u) changed << 100 + i;
    }
    if (m_candidate < 0) {
        if (changed.size() == 1) {
            m_candidate = changed.first();
            m_findNote = QStringLiteral("%1 changed. Let go.").arg(InputName(m_candidate));
        } else if (changed.size() > 1) {
            QStringList names;
            for (int i : changed) names << InputName(i);
            m_findNote = QStringLiteral("More than one input changed (%1). Let go, and trip only the %2 switch.")
                             .arg(names.join(QStringLiteral(", ")), switchName(m_finding));
        }
    } else if (levelOf(m_candidate, board, ccio) == levelOf(m_candidate, m_baseBoard, m_baseCcio)) {
        // Back to its level at rest: that is the switch, if nothing else has it.
        const QString why = blockedWhy(m_candidate);
        if (!why.isEmpty()) {
            m_findNote = why + QStringLiteral(". Trip the %1 switch.").arg(switchName(m_finding));
        } else {
            Found &f = m_found[m_finding];
            f.input = m_candidate;
            f.restOn = levelOf(m_candidate, m_baseBoard, m_baseCcio) == 1;
            f.normallyOpen = !f.restOn;
            f.proved = false;
            m_findNote = QStringLiteral("The %1 switch is on %2.").arg(switchName(m_finding), InputName(m_candidate));
        }
        m_candidate = -1;
        m_baseBoard = -1;  // look again from here
    }
    refresh();
}

// The stop on a switch jog: the found switches that read as at rest now (one already tripped would stop it at once).
QString SwitchWizard::stopList() const {
    QStringList names;
    for (int s = 0; s < kSwitches; ++s) {
        const Found &f = m_found[s];
        if (!f.wanted || f.input < 0) continue;
        const int now = m_lastBoard >= 0 ? levelOf(f.input, m_lastBoard, m_lastCcio) : -1;
        if (now >= 0 && now != (f.restOn ? 1 : 0)) continue;
        names << InputName(f.input);
    }
    return names.join(QLatin1Char(','));
}

void SwitchWizard::startJog(int dir) {
    if (m_askDir != 0 || m_uploading) return;
    if (m_step == kStepDirection && stopList().isEmpty()) {
        m_dirNote = QStringLiteral("Every switch reads tripped: nothing could stop the axis.");
        refresh();
        return;
    }
    m_jogDir = dir;
    m_dirNote.clear();
    jogTick();
    m_jogTimer->start();
}

void SwitchWizard::jogTick() {
    if (m_jogDir == 0 || m_jogBusy) return;
    m_jogBusy = true;
    QPointer<SwitchWizard> self(this);
    const double v = m_jogDir * m_speed->value();
    // Direction: a stop on a switch (no limit is on the board yet). Limits: a plain jog, so the board's limit stops it.
    const QString line = m_step == kStepDirection ? QStringLiteral("axjog %1 jog %2 stopon %3").arg(motor()).arg(v, 0, 'g', 10).arg(stopList())
                                                  : QStringLiteral("axjog %1 jog %2").arg(motor()).arg(v, 0, 'g', 10);
    m_board->command(line,
                     [self](bool ok, const QStringList &r) {
                         if (!self) return;
                         self->m_jogBusy = false;
                         if (!ok && self->m_jogDir != 0) {
                             // Refused: it stopped on a switch (read it), or something else (say what).
                             const QString why = r.isEmpty() ? QString() : r.last();
                             if (self->m_step == kStepLimits) self->m_limNote = why;
                             else if (!why.contains(QLatin1String("stopped on a switch"))) self->m_dirNote = why;
                             self->m_jogTimer->stop();
                         }
                     });
}

// Let go: read where it stopped (a stop on a switch keeps it until "stop"), then stop.
void SwitchWizard::stopJog() {
    if (m_jogTimer) m_jogTimer->stop();
    if (m_jogDir == 0) return;
    QPointer<SwitchWizard> self(this);
    const int m = motor();
    if (m_step != kStepDirection) {
        m_jogDir = 0;
        m_board->command(QStringLiteral("axjog %1 stop").arg(m));
        return;
    }
    m_board->command(QStringLiteral("axjog"), [self, m](bool ok, const QStringList &r) {
        if (!self) return;
        if (ok) self->onAxjog(r);
        self->m_jogDir = 0;
        self->m_board->command(QStringLiteral("axjog %1 stop").arg(m));
    });
}

void SwitchWizard::onAxjog(const QStringList &lines) {
    for (const QString &l : lines) {
        JogDialog::MotorInfo info;
        if (!JogDialog::ParseMotorLine(l, &info) || info.motor != motor()) continue;
        if (!info.stopped.isEmpty() && m_jogDir != 0) {
            const int dir = m_jogDir;
            m_jogDir = 0;
            m_jogTimer->stop();
            feedStop(dir, info.stopped);
            m_board->command(QStringLiteral("axjog %1 stop").arg(motor()));
            return;
        }
    }
}

void SwitchWizard::feedStop(int dir, const QString &input) {
    int reached = -1;
    for (int s = 0; s < kSwitches; ++s) {
        if (m_found[s].wanted && InputName(m_found[s].input) == input) reached = s;
    }
    switch (Judge(dir, reached)) {
        case Verdict::Right:
            m_found[reached].proved = true;
            m_dirNote = QStringLiteral("%1 reached going %2: right.").arg(switchName(reached), dir > 0 ? QStringLiteral("+") : QStringLiteral("−"));
            break;
        case Verdict::Home:
            m_homeSide = dir;
            m_found[kHome].proved = true;
            m_dirNote = QStringLiteral("Home is on the %1 side. Hold the same jog again for the limit beyond it.")
                            .arg(dir > 0 ? QStringLiteral("+") : QStringLiteral("−"));
            break;
        case Verdict::Ask:
            m_askDir = dir;
            m_dirNote = QStringLiteral("%1 reached going %2.").arg(switchName(reached), dir > 0 ? QStringLiteral("+") : QStringLiteral("−"));
            break;
        case Verdict::Unknown:
            m_dirNote = QStringLiteral("Stopped at %1, which isn't one of this axis's switches.").arg(input);
            break;
    }
    refresh();
}

// Going dir, the axis reached the other limit. Toward the end where that end's switch was tripped by hand: the switches
// were swapped in Find. Away from it: the motor runs the other way.
void SwitchWizard::answer(bool towardTrippedEnd) {
    if (m_askDir == 0) return;
    const int dir = m_askDir;
    m_askDir = 0;
    if (towardTrippedEnd) {
        std::swap(m_found[kLimitPlus].input, m_found[kLimitMinus].input);
        std::swap(m_found[kLimitPlus].restOn, m_found[kLimitMinus].restOn);
        std::swap(m_found[kLimitPlus].normallyOpen, m_found[kLimitMinus].normallyOpen);
        m_found[dir > 0 ? kLimitPlus : kLimitMinus].proved = true;
        m_found[dir > 0 ? kLimitMinus : kLimitPlus].proved = false;
        m_dirNote = QStringLiteral("The + and − switches were the other way round: swapped.");
        refresh();
        return;
    }
    // The motor's + is the other way: reverse it, upload, prove again.
    CamAxis &a = m_project.axes[axis()];
    a.reverse = !a.reverse;
    m_reversed = !m_reversed;
    for (Found &f : m_found) f.proved = false;
    m_homeSide = 0;
    m_dirNote = QStringLiteral("The motor ran the other way: reversed. Uploading...");
    m_uploading = true;
    refresh();
    QPointer<SwitchWizard> self(this);
    m_upload(m_project, [self](bool ok, const QString &msg) {
        if (!self) return;
        self->m_uploading = false;
        self->m_dirNote = ok ? QStringLiteral("Reversed and uploaded. Enable, and jog each way again.") : QStringLiteral("Upload: %1").arg(msg);
        self->refresh();
    });
}

CamProject SwitchWizard::result() const {
    CamProject p = m_project;
    const int ax = axis();
    static const CcioRole kRole[] = {CcioRole::LimitPositive, CcioRole::LimitNegative, CcioRole::Home};
    static const char *const kName[] = {"limit +", "limit -", "home"};
    // The axis's jobs start again from what was found.
    for (BoardInput in : p.inputs) {
        if (in.axis == ax && in.role != CcioRole::None) {
            in.role = CcioRole::None;
            in.axis = -1;
            SetBoardInput(p, in);
        }
    }
    for (CcioInput in : p.ccio.inputs) {
        if (in.axis == ax && in.role != CcioRole::None) {
            in.role = CcioRole::None;
            in.axis = -1;
            SetCcioInput(p.ccio, in);
        }
    }
    for (int s = 0; s < kSwitches; ++s) {
        const Found &f = m_found[s];
        if (!f.wanted || f.input < 0) continue;
        const std::string name = p.axes[ax].name + " " + kName[s];
        if (f.input < 100) {
            BoardInput in = BoardInputAt(p, f.input);
            in.role = kRole[s];
            in.axis = ax;
            in.normallyOpen = f.normallyOpen;
            if (in.name.empty()) in.name = name;
            SetBoardInput(p, in);
        } else {
            const int b = f.input - 100;
            CcioInput in = CcioInputAt(p.ccio, b / kCcioPinsPerBoard, b % kCcioPinsPerBoard);
            in.role = kRole[s];
            in.axis = ax;
            in.normallyOpen = f.normallyOpen;
            if (in.name.empty()) in.name = name;
            SetCcioInput(p.ccio, in);
        }
    }
    // Homing: the Home step's settings, or at least the side the home switch was seen on.
    if (m_found[kHome].wanted && m_found[kHome].input >= 0) {
        AxisHome h = m_homeSet ? m_home : p.axes[ax].home;
        if (m_homeSide) h.dir = m_homeSide;
        p.axes[ax].home = h;
    }
    NormalizeProject(p);
    return p;
}

void SwitchWizard::refresh() {
    const BoardLink::State &s = m_hasTestState ? m_testState : m_board->state();
    const bool conn = (m_hasTestState || m_board->isConnected()) && s.valid;
    const bool onBoard = conn && s.profile != 0 && s.profile == ProfileHash(io::UploadLines(m_project));
    const bool running = s.state == QLatin1String("running") || s.state == QLatin1String("jogging") || s.timeline == QLatin1String("armed");
    bool next = false;
    switch (m_step) {
        case kStepAxis: {
            bool any = false;
            for (QCheckBox *c : m_want) any = any || c->isChecked();
            QString why;
            if (!conn) why = QStringLiteral("Not connected to a board.");
            else if (s.inputs < 0) why = QStringLiteral("This board's firmware doesn't report its inputs: update it.");
            else if (!onBoard) why = QStringLiteral("The board has another profile: upload this project first.");
            else if (running) why = QStringLiteral("The cam is running.");
            else if (s.plcInControl()) why = QStringLiteral("The PLC has control.");
            else if (!any) why = QStringLiteral("Tick the switches this axis has.");
            if (why.isEmpty() && s.estop == QLatin1String("off")) {
                m_ready->setText(QStringLiteral("<span style='color:#ffcc80;'>No estop on the board (Board page, Estop).</span>"));
            } else {
                m_ready->setText(why.isEmpty() ? QString() : QStringLiteral("<span style='color:#ffcc80;'>%1</span>").arg(why));
            }
            next = why.isEmpty();
            break;
        }
        case kStepFind: {
            bool all = true;
            for (int k = 0; k < kSwitches; ++k) {
                const Found &f = m_found[k];
                m_findRows[k]->setVisible(f.wanted);
                if (!f.wanted) continue;
                if (f.input >= 0) {
                    m_findRows[k]->setText(QStringLiteral("<span style='color:#7fe3a0;'>%1</span>").arg(InputName(f.input)));
                } else if (k == m_finding) {
                    m_findRows[k]->setText(m_candidate >= 0 ? QStringLiteral("%1 changed").arg(InputName(m_candidate))
                                                            : QStringLiteral("<b>Trip it by hand now</b>"));
                    all = false;
                } else {
                    m_findRows[k]->setText(QStringLiteral("–"));
                    all = false;
                }
            }
            m_note->setText(m_findNote);
            next = all;
            break;
        }
        case kStepContact: {
            QStringList warn;
            for (int k = 0; k < kSwitches; ++k) {
                const Found &f = m_found[k];
                m_contactRows[k]->setVisible(f.wanted);
                m_contact[k]->setVisible(f.wanted);
                if (!f.wanted) continue;
                const int now = m_lastBoard >= 0 ? levelOf(f.input, m_lastBoard, m_lastCcio) : -1;
                m_contactRows[k]->setText(QStringLiteral("%1, %2").arg(InputName(f.input), now < 0 ? QStringLiteral("–") : now ? QStringLiteral("On") : QStringLiteral("Off")));
                if (f.normallyOpen && k != kHome) warn << switchName(k);
            }
            m_contactWarn->setText(warn.isEmpty() ? QString()
                                                  : QStringLiteral("Normally open %1: a cut wire would read as not tripped, and the axis would lose that limit without a word.")
                                                        .arg(warn.join(QStringLiteral(" and "))));
            next = true;
            break;
        }
        case kStepDirection: {
            const bool motorReady = conn && !running && !m_uploading;
            m_enable->setText(m_motorOn ? QStringLiteral("Disable") : QStringLiteral("Enable"));
            m_enable->setEnabled(motorReady);
            const bool canJog = motorReady && (m_motorOn || s.simulate) && m_askDir == 0;
            m_jogMinus->setEnabled(canJog);
            m_jogPlus->setEnabled(canJog);
            bool all = true;
            for (int k = 0; k < kSwitches; ++k) {
                const Found &f = m_found[k];
                m_dirRows[k]->setVisible(f.wanted);
                if (!f.wanted) continue;
                QString text = InputName(f.input);
                if (k == kHome) {
                    text += m_homeSide ? QStringLiteral(": on the %1 side").arg(m_homeSide > 0 ? QStringLiteral("+") : QStringLiteral("−")) : QString();
                } else if (f.proved) {
                    text += QStringLiteral(": proved");
                }
                m_dirRows[k]->setText(f.proved ? QStringLiteral("<span style='color:#7fe3a0;'>%1</span>").arg(text) : text);
                all = all && f.proved;
            }
            const bool asking = m_askDir != 0;
            m_question->setVisible(asking);
            m_yes->setVisible(asking);
            m_no->setVisible(asking);
            if (asking) {
                m_question->setText(QStringLiteral("Did the axis move toward the end where you tripped the %1 limit?")
                                        .arg(m_askDir > 0 ? QStringLiteral("+") : QStringLiteral("−")));
            }
            m_note->setText(m_dirNote);
            next = all && !asking && !m_uploading;
            break;
        }
        case kStepLimits: {
            const bool motorReady = conn && !running && !m_uploading;
            m_limEnable->setText(m_motorOn ? QStringLiteral("Disable") : QStringLiteral("Enable"));
            m_limEnable->setEnabled(motorReady);
            const int ax = axis();
            const bool onPlus = (s.limits >> (2 * ax)) & 1u, onMinus = (s.limits >> (2 * ax + 1)) & 1u;
            m_limMinus->setEnabled(motorReady && m_motorOn && !onMinus);
            m_limPlus->setEnabled(motorReady && m_motorOn && !onPlus);
            const double cpu = std::fabs(m_project.axes[ax].countsPerUnit) > 0 ? std::fabs(m_project.axes[ax].countsPerUnit) : 1.0;
            const QString u = QString::fromStdString(m_project.axes[ax].units);
            bool all = true;
            for (int k = 0; k < 2; ++k) {
                const bool want = m_found[k].wanted && m_found[k].input >= 0;
                m_limRows[k]->setVisible(want);
                if (!want) continue;
                QString text;
                if (!m_limStopped[k]) {
                    text = QStringLiteral("%1: jog %2 until the board's limit stops the axis").arg(InputName(m_found[k].input), k == 0 ? QStringLiteral("+") : QStringLiteral("\u2212"));
                } else if (m_overCounts[k] < 0) {
                    text = QStringLiteral("Stopped by the limit\u2026");
                } else {
                    text = QStringLiteral("Stopped by the limit, %1 %2 past the switch").arg(m_overCounts[k] / cpu, 0, 'f', 2).arg(u);
                    text += m_limCleared[k] ? QStringLiteral("; cleared") : QStringLiteral("; now jog %1 off it").arg(k == 0 ? QStringLiteral("\u2212") : QStringLiteral("+"));
                }
                const bool done = m_limStopped[k] && m_limCleared[k];
                m_limRows[k]->setText(done ? QStringLiteral("<span style='color:#7fe3a0;'>%1</span>").arg(text) : text);
                all = all && done;
            }
            QStringList info;
            if (m_limStopped[0] && m_limStopped[1]) {
                info << QStringLiteral("Travel between the switches: %1 %2").arg(std::llabs(m_tripAt[0] - m_tripAt[1]) / cpu, 0, 'f', 2).arg(u);
            }
            {
                const review::ProjectInfo pi = review::AnalyzeProject(m_project);
                if (ax < static_cast<int>(pi.axes.size())) {
                    const double v = pi.axes[ax].stats.maxVel * pi.axes[ax].w;
                    info << QStringLiteral("At the profile's top speed a limit stop runs up to %1 %2 past the switch")
                                .arg(v * m_project.master.limitStopSeconds / 2.0, 0, 'f', 2).arg(u);
                }
            }
            m_limTravel->setText(info.join(QStringLiteral("<br>")));
            m_note->setText(m_limNote);
            next = all && !m_uploading;
            break;
        }
        case kStepHome: {
            const bool motorReady = conn && !running && !m_uploading;
            m_homeEnable->setText(m_motorOn ? QStringLiteral("Disable") : QStringLiteral("Enable"));
            m_homeEnable->setEnabled(motorReady && !m_homeRunning);
            m_homeGo->setText(m_homeRunning ? QStringLiteral("Stop") : QStringLiteral("Home %1 times").arg(m_runsWanted));
            m_homeGo->setEnabled(m_homeRunning ? conn : motorReady && (m_motorOn || !m_homeUp));
            for (QWidget *w : std::initializer_list<QWidget *>{m_hFast, m_hSlow, m_hBack, m_hSearch, m_hAt, m_hDecel, m_hThen, m_hRequired, m_hRuns}) {
                w->setEnabled(!m_homeRunning && !m_uploading);
            }
            const double cpu = std::fabs(m_project.axes[axis()].countsPerUnit) > 0 ? std::fabs(m_project.axes[axis()].countsPerUnit) : 1.0;
            const QString u = QString::fromStdString(m_project.axes[axis()].units);
            QStringList rows;
            rows << QStringLiteral("Toward %1 (the side the switch was seen on)").arg(m_home.dir > 0 ? QStringLiteral("+") : QStringLiteral("\u2212"));
            if (m_runsDone > 0) {
                rows << QStringLiteral("Homed %1 of %2").arg(m_runsDone).arg(m_runsWanted);
                if (m_runsDone > 1) {
                    rows << QStringLiteral("Spread of the latched positions: %1 %2 (%3 counts)")
                                .arg(homeSpreadCounts() / cpu, 0, 'f', 4).arg(u).arg(homeSpreadCounts(), 0, 'f', 0);
                }
            } else if (m_homeRunning) {
                rows << QStringLiteral("Homing\u2026");
            }
            m_homeResults->setText(rows.join(QStringLiteral("<br>")));
            m_note->setText(m_homeNote);
            next = m_runsDone >= m_runsWanted && !m_homeFailed && !m_homeRunning && !m_uploading;
            break;
        }
        case kStepReview: {
            QStringList rows;
            static const char *const kContact[] = {"normally closed", "normally open"};
            for (int k = 0; k < kSwitches; ++k) {
                const Found &f = m_found[k];
                if (!f.wanted) continue;
                QString row = QStringLiteral("<b>%1</b>: %2, %3").arg(switchName(k), InputName(f.input), QLatin1String(kContact[f.normallyOpen ? 1 : 0]));
                if (k == kHome && m_homeSide) row += QStringLiteral(", on the %1 side").arg(m_homeSide > 0 ? QStringLiteral("+") : QStringLiteral("−"));
                else if (f.proved) row += QStringLiteral(", proved by motion");
                rows << row;
            }
            if (m_reversed) rows << QStringLiteral("<b>Motor direction</b>: reversed");
            {
                const double cpu = std::fabs(m_project.axes[axis()].countsPerUnit) > 0 ? std::fabs(m_project.axes[axis()].countsPerUnit) : 1.0;
                const QString u = QString::fromStdString(m_project.axes[axis()].units);
                for (int k = 0; k < 2; ++k) {
                    if (m_limStopped[k] && m_overCounts[k] >= 0) {
                        rows << QStringLiteral("<b>%1 test</b>: stopped %2 %3 past the switch at %4 %3/s")
                                    .arg(switchName(k)).arg(m_overCounts[k] / cpu, 0, 'f', 2).arg(u).arg(m_speed->value(), 0, 'g', 4);
                    }
                }
                if (m_homeSet && m_found[kHome].wanted) {
                    rows << QStringLiteral("<b>Homing</b>: toward %1, search %2 %3/s, latch %4 %3/s, home position %5 %3%6")
                                .arg(m_home.dir > 0 ? QStringLiteral("+") : QStringLiteral("\u2212"))
                                .arg(m_home.fast, 0, 'g', 4).arg(u).arg(m_home.slow, 0, 'g', 4).arg(m_home.at, 0, 'g', 6)
                                .arg(m_home.required ? QStringLiteral(", required") : QString());
                    if (m_runsDone > 1) {
                        rows << QStringLiteral("<b>Homing test</b>: %1 homings, spread %2 %3").arg(m_runsDone).arg(homeSpreadCounts() / cpu, 0, 'f', 4).arg(u);
                    }
                }
            }
            m_review->setText(rows.join(QStringLiteral("<br>")));
            m_note->setText(QString());
            next = conn && !m_uploading;
            break;
        }
        default: break;
    }
    // A switch the axis doesn't have: its name goes with its row on every page.
    for (int k = 0; k < kSwitches; ++k) {
        for (QLabel *l : m_names[k]) l->setVisible(m_want[k]->isChecked());
    }
    if (m_step == kStepAxis) m_note->setText(QString());
    m_back->setEnabled(m_step > 0 && !m_uploading && m_jogDir == 0);
    m_next->setText(m_step == kStepReview ? QStringLiteral("Apply") : QStringLiteral("Next"));
    m_next->setEnabled(next);
}

void SwitchWizard::setStateForTest(const BoardLink::State &state) {
    m_testState = state;
    m_hasTestState = true;
    m_motorOn = state.valid && ((state.motorsOn >> motor()) & 1u);
    refresh();
}

void SwitchWizard::setOperatorPins(const int pins[5]) {
    for (int k = 0; k < 5; ++k) m_opPins[k] = pins[k];
}

void SwitchWizard::reject() {
    if (m_jogTimer) m_jogTimer->stop();
    if (m_jogDir != 0 && m_board->isConnected()) m_board->command(QStringLiteral("axjog %1 stop").arg(motor()));
    if (m_homeRunning && m_board->isConnected()) m_board->command(QStringLiteral("axhome stop"));
    m_homeRunning = false;
    m_jogDir = 0;
    QDialog::reject();
}
