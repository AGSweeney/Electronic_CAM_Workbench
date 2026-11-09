/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
// Bench check of the Board panel with a PLC on EtherNet/IP, through the real window. A PLC stand-in
// (ecam_bench uitest) drives the board; the two hand over on stdout / the board's state:
//
//   1. the app connects and uploads (simulate), prints "ready"
//   2. the PLC connects without PLC_CONTROL: the status line says "PLC ... connected"
//   3. the PLC takes control: Upload, Forget, Enable, Simulate, Start, Hand jog greyed with the reason in
//      their tips; Stop, Abort enabled; "PLC ... in control"; prints "in-control"
//   4. the PLC releases control: the buttons and their own tips come back
//
//   board_ui_plc <ip> <project.ecam>        exit code 0 = all passed

#include "BoardLink.h"
#include "MainWindow.h"

#include <QAbstractButton>
#include <QApplication>
#include <QCheckBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QPushButton>

#include <cstdio>

static int g_fail = 0;
static void Check(bool ok, const QString &what) {
    std::printf("%s %s\n", ok ? "ok  " : "FAIL", qPrintable(what));
    std::fflush(stdout);
    if (!ok) ++g_fail;
}

template <typename F>
static bool WaitFor(F pred, int ms) {
    QElapsedTimer t;
    t.start();
    while (!pred() && t.elapsed() < ms) QApplication::processEvents(QEventLoop::AllEvents, 20);
    return pred();
}

static QAbstractButton *Button(QWidget *w, const QString &text) {
    for (QAbstractButton *b : w->findChildren<QAbstractButton *>()) {
        if (b->text() == text) return b;
    }
    return nullptr;
}

static bool StatusSays(QWidget *w, const QString &text) {
    for (QLabel *l : w->findChildren<QLabel *>()) {
        if (l->text().contains(text)) return true;
    }
    return false;
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    if (argc < 3) {
        std::printf("usage: board_ui_plc <ip> <project.ecam>\n");
        return 2;
    }
    MainWindow win;
    win.resize(1480, 940);
    win.show();
    Check(win.OpenFile(argv[2]), "open project");
    win.ConnectBoard(argv[1]);
    BoardLink *link = win.findChild<BoardLink *>();
    Check(link && WaitFor([&] { return link->state().valid; }, 4000), "connected, status received");
    if (!link) return 1;
    link->command(QStringLiteral("abort"));
    link->command(QStringLiteral("disable"));
    link->command(QStringLiteral("clear"));
    link->command(QStringLiteral("sim 1"));
    Button(&win, QStringLiteral("Upload"))->click();
    Check(WaitFor([&] { return link->state().profile != 0 && Button(&win, QStringLiteral("Upload"))->isEnabled() &&
                                 link->state().simulate; }, 15000),
          "uploaded, simulate on");
    const QString uploadTip = Button(&win, QStringLiteral("Upload"))->toolTip();
    std::printf("ready\n");
    std::fflush(stdout);

    // 2. A PLC that only reads.
    Check(WaitFor([&] { return link->state().eip == QStringLiteral("run"); }, 20000), "status: eip=run (a PLC connected)");
    Check(WaitFor([&] { return StatusSays(&win, QStringLiteral("connected")) && StatusSays(&win, QStringLiteral("PLC ")); }, 3000),
          "status line: PLC ... connected");
    Check(Button(&win, QStringLiteral("Upload"))->isEnabled() && Button(&win, QStringLiteral("Start"))->isEnabled(),
          "without control: Upload and Start still enabled");

    // 3. The PLC takes control.
    Check(WaitFor([&] { return link->state().plcInControl(); }, 20000), "status: eip=control");
    WaitFor([] { return false; }, 400);
    if (const QString shot = qEnvironmentVariable("ECAM_UI_SHOT"); !shot.isEmpty()) win.grab().save(shot);
    const QString from = link->state().eipFrom;
    Check(StatusSays(&win, QStringLiteral("PLC %1 in control").arg(from)), QStringLiteral("status line: PLC %1 in control").arg(from));
    for (const QString &name : {QStringLiteral("Upload"), QStringLiteral("Enable"), QStringLiteral("Simulate"),
                                QStringLiteral("Start"), QStringLiteral("Hand jog"), QStringLiteral("Forget")}) {
        QAbstractButton *b = Button(&win, name);
        Check(b && !b->isEnabled() && b->toolTip().contains(QStringLiteral("The PLC has control")),
              name + QStringLiteral(" greyed, its tip says the PLC has control"));
    }
    Check(Button(&win, QStringLiteral("Abort"))->isEnabled(), "Abort still enabled");
    link->command(QStringLiteral("start"));  // the board refuses it too
    Check(WaitFor([&] { return link->state().state == QStringLiteral("ready"); }, 1000) &&
              link->state().crank == QStringLiteral("off"),
          "a console start while the PLC has control: refused, nothing moves");
    std::printf("in-control\n");
    std::fflush(stdout);

    // 4. Released.
    Check(WaitFor([&] { return !link->state().plcInControl(); }, 20000), "status: control released");
    WaitFor([] { return false; }, 400);
    Check(Button(&win, QStringLiteral("Upload"))->isEnabled() && Button(&win, QStringLiteral("Start"))->isEnabled() &&
              Button(&win, QStringLiteral("Simulate"))->isEnabled(),
          "after release: Upload, Start, Simulate enabled again");
    Check(Button(&win, QStringLiteral("Upload"))->toolTip() == uploadTip, "Upload's own tip is back");
    link->command(QStringLiteral("sim 0"));
    WaitFor([] { return false; }, 300);
    std::printf("%s\n", g_fail ? "some checks failed" : "all checks passed");
    return g_fail ? 1 : 0;
}
