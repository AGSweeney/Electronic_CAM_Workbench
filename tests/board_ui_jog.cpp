/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
// Bench check of hand jog through the real window, against a board in simulate mode (motors are
// never stepped). Opens a project, connects, uploads, then drags the cam view with synthetic mouse
// events and checks the board's master followed.
//
//   board_ui_jog <ip | COMn> <project.ecam>        exit code 0 = all passed

#include "BoardLink.h"
#include "CamView.h"
#include "MainWindow.h"

#include <QAbstractButton>
#include <QApplication>
#include <QTreeWidget>
#include <QCheckBox>
#include <QLabel>
#include <QElapsedTimer>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPushButton>
#include <QStatusBar>
#include <QTimer>

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>

static int g_fail = 0;
static void Check(bool ok, const QString &what) {
    std::printf("%s %s\n", ok ? "ok  " : "FAIL", qPrintable(what));
    if (!ok) ++g_fail;
}

template <typename F>
static bool WaitFor(F pred, int ms) {
    QElapsedTimer t;
    t.start();
    while (!pred() && t.elapsed() < ms) QApplication::processEvents(QEventLoop::AllEvents, 20);
    return pred();
}

static QPushButton *Button(QWidget *w, const QString &text) {
    for (QPushButton *b : w->findChildren<QPushButton *>()) {
        if (b->text() == text) return b;
    }
    return nullptr;
}

// Drag the cam view clockwise by `degrees` around its centre, in small steps.
static void DragCam(CamView *v, double degrees) {
    const QPointF c(v->width() / 2.0, v->height() * 0.6);
    const double r = std::min(v->width(), v->height()) * 0.3;
    auto at = [&](double a) { return QPointF(c.x() + r * std::cos(a), c.y() - r * std::sin(a)); };
    double a = 0.0;
    QMouseEvent press(QEvent::MouseButtonPress, at(a), v->mapToGlobal(at(a)), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(v, &press);
    const int steps = int(std::fabs(degrees) / 2.0);
    for (int i = 1; i <= steps; ++i) {
        a -= (degrees > 0 ? 2.0 : -2.0) * M_PI / 180.0;  // clockwise on screen = increasing crank angle
        QMouseEvent move(QEvent::MouseMove, at(a), v->mapToGlobal(at(a)), Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
        QApplication::sendEvent(v, &move);
        WaitFor([] { return false; }, 15);
    }
    QMouseEvent rel(QEvent::MouseButtonRelease, at(a), v->mapToGlobal(at(a)), Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
    QApplication::sendEvent(v, &rel);
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    if (argc < 3) {
        std::printf("usage: board_ui_jog <ip | COMn> <project.ecam>\n");
        return 2;
    }
    MainWindow win;
    win.resize(1480, 940);
    win.show();
    Check(win.OpenFile(argv[2]), "open project");
    win.ConnectBoard(argv[1]);
    // The Board page holds Hand jog and Crank -> 0, so have it open as a person would.
    if (auto *tree = win.findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"))) {
        const auto items = tree->findItems(QStringLiteral("Board"), Qt::MatchExactly | Qt::MatchRecursive);
        if (!items.isEmpty()) tree->setCurrentItem(items.first());
    }
    BoardLink *link = win.findChild<BoardLink *>();
    CamView *cam = win.findChild<CamView *>();
    const bool got = link && cam && WaitFor([&] { return link->state().valid; }, 4000);
    if (!got) {
        std::printf("     connected=%d, status bar: %s\n", link ? int(link->isConnected()) : -1,
                    qPrintable(win.statusBar()->currentMessage()));
    }
    Check(got, "connected, status received");

    link->command(QStringLiteral("abort"));
    link->command(QStringLiteral("disable"));
    link->command(QStringLiteral("sim 1"));
    Button(&win, QStringLiteral("Upload"))->click();
    Check(WaitFor([&] { return link->state().profile != 0 && Button(&win, QStringLiteral("Upload"))->isEnabled() &&
                                 link->state().simulate; }, 15000),
          "uploaded, simulate on");
    WaitFor([] { return false; }, 600);

    // 0. Timeline projects: Start arms, the Trigger button is enabled and starts it.
    QPushButton *trigger = Button(&win, QStringLiteral("Trigger"));
    if (trigger && trigger->isVisible()) {
        QCheckBox *onStart = nullptr;
        for (QCheckBox *c : win.findChildren<QCheckBox *>()) {
            if (c->text() == QStringLiteral("Trigger on Start")) onStart = c;
        }
        const bool wasChecked = onStart && onStart->isChecked();
        if (onStart) onStart->setChecked(false);
        Check(!trigger->isEnabled(), "Trigger greyed before Start (" + trigger->toolTip() + ")");
        Button(&win, QStringLiteral("Start"))->click();
        Check(WaitFor([&] { return link->state().timeline == QStringLiteral("armed") && trigger->isEnabled(); }, 3000),
              "Start arms the timeline and enables Trigger");
        QLabel *stateLine = nullptr;
        for (QLabel *l : win.findChildren<QLabel *>()) {
            if (l->text().startsWith(QStringLiteral("Armed"))) stateLine = l;
        }
        Check(stateLine != nullptr, "state line: " + (stateLine ? stateLine->text() : QStringLiteral("(none)")));
        trigger->click();
        Check(WaitFor([&] { return link->state().timeline == QStringLiteral("running"); }, 3000),
              "Trigger starts the timeline");
        Check(!trigger->isEnabled(), "Trigger greyed while running (" + trigger->toolTip() + ")");
        Button(&win, QStringLiteral("Stop"))->click();
        WaitFor([&] { return link->state().state != QStringLiteral("running"); }, 5000);
        if (onStart) onStart->setChecked(wasChecked);
        WaitFor([] { return false; }, 300);
    }

    // 1. Board idle, Follow board on: dragging the cam must stick (it used to snap back).
    DragCam(cam, 60);
    WaitFor([] { return false; }, 800);  // several telemetry lines
    const double idleAngle = std::fmod(link->state().deg, 360.0);
    Check(link->state().state != QStringLiteral("jogging"), "idle board does not move on its own");
    std::printf("     board master %.2f deg (idle, untouched)\n", idleAngle);

    // 2. Hand jog: the drag drives the board's master.
    QPushButton *jog = Button(&win, QStringLiteral("Hand jog"));
    Check(jog && jog->isEnabled(), "Hand jog available");
    jog->click();
    Check(WaitFor([&] { return link->state().state == QStringLiteral("jogging"); }, 3000), "board jogging");
    const double start = link->state().rev * 360.0 + link->state().deg;
    // The drag circles a fixed point, so the angle it sweeps depends on the layout: compare the board with
    // the angle the cam view itself reports, not with the nominal 200.
    double appAngle = -1.0;
    const QMetaObject::Connection angleConn =
        QObject::connect(cam, &CamView::angleChanged, [&](double a) { appAngle = std::fmod(std::fmod(a, 360.0) + 360.0, 360.0); });
    DragCam(cam, 200);
    QObject::disconnect(angleConn);
    WaitFor([&] { return false; }, 1500);  // let the board catch up (rate limited)
    const double end = link->state().rev * 360.0 + link->state().deg;
    const double boardMod = std::fmod(std::fmod(end, 360.0) + 360.0, 360.0);
    std::printf("     board master %.2f -> %.2f deg (dragged ~200 deg clockwise; app cursor %.2f deg)\n", start, end, appAngle);
    double diff = std::fabs(boardMod - appAngle);
    diff = std::min(diff, 360.0 - diff);
    Check(appAngle >= 0.0 && end - start > 150.0 && diff < 0.5, "board master followed the drag to the app's angle");
    Check(link->state().axisCounts.size() > 0, "axis positions reported");

    // 3. Crank -> 0 deg: surfaces once the crank is at rest, then goes to 0 the short way.
    QPushButton *zero = Button(&win, QStringLiteral("Crank → 0°"));
    Check(zero && WaitFor([&] { return zero->isVisible(); }, 3000), "Crank -> 0 shown with the crank at rest");
    if (zero && zero->isVisible()) {
        const double from = link->state().rev * 360.0 + link->state().deg;
        const double expect = std::round(from / 360.0) * 360.0;
        zero->click();
        WaitFor([&] { return false; }, 400);
        WaitFor([&] { return zero->isVisible(); }, 4000);  // at rest again
        const double at = link->state().rev * 360.0 + link->state().deg;
        std::printf("     crank %.2f -> %.2f deg (nearest 0: %.0f)\n", from, at, expect);
        Check(std::fabs(at - expect) < 0.05, "crank at 0 deg, the short way");
        Check(!zero->isEnabled(), "Crank -> 0 greyed out at 0");
    }

    jog->click();  // off
    Check(WaitFor([&] { return link->state().state != QStringLiteral("jogging"); }, 3000), "jog off");
    link->command(QStringLiteral("sim 0"));
    WaitFor([] { return false; }, 300);

    // Forget: enabled while a profile is stored; confirming erases it (the profile keeps running).
    if (QPushButton *forget = Button(&win, QStringLiteral("Forget"))) {
        link->command(QStringLiteral("status"));
        Check(WaitFor([&] { return link->state().stored && forget->isEnabled(); }, 3000), "Forget enabled (profile stored)");
        QTimer::singleShot(300, [] {
            auto *box = qobject_cast<QMessageBox *>(QApplication::activeModalWidget());
            for (QAbstractButton *b : box ? box->buttons() : QList<QAbstractButton *>()) {
                if (b->text() == QStringLiteral("Forget and restart")) b->click();
            }
        });
        forget->click();
        Check(WaitFor([&] { return !link->isConnected(); }, 4000), "Forget and restart: the board restarts");
        Check(WaitFor([&] { return link->isConnected() && link->state().state == QStringLiteral("empty"); }, 30000),
              "reconnected, board came up empty");
        Check(!forget->isEnabled(), "Forget greyed (nothing stored)");
    } else {
        Check(false, "Forget button");
    }
    std::printf(g_fail ? "%d check(s) FAILED\n" : "all checks passed\n", g_fail);
    return g_fail ? 1 : 0;
}
