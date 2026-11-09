/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
// Bench check of the Board page's Network settings, through the real window against a real board.
//
//   board_ui_net readonly <ip>                 over Ethernet the fields can be read, not changed
//   board_ui_net guards <COMn>                 over USB: editable, bad input refused, enabled motors block it
//   board_ui_net set <COMn> static <ip>        apply through the window; the board restarts and is read back
//   board_ui_net set <COMn> dhcp
//   board_ui_net set <COMn> host <name|->
//   board_ui_net estop <ip | COMn>            the Estop row: reads the board, trips (estop_test), changes and puts back 1
//   board_ui_net status <ip | COMn>           the Project dock's status: estop, trip, M-0 enabled alone (free shaft), off
//
// `set` changes what the board stores: the bench script checks the address from outside, and puts it back.

#include "BoardLink.h"
#include "MainWindow.h"

#include <QApplication>
#include <QComboBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTreeWidget>

#include <cstdio>
#include <functional>

static int g_fail = 0;

static void Check(bool ok, const QString &what) {
    std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.toUtf8().constData());
    std::fflush(stdout);
    if (!ok) ++g_fail;
}

static bool WaitFor(const std::function<bool()> &ready, int ms) {
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < ms) {
        if (ready()) return true;
        QApplication::processEvents(QEventLoop::AllEvents, 20);
    }
    return ready();
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    if (argc < 3) {
        std::printf("usage: see tests/board_ui_net.cpp\n");
        return 2;
    }
    const QString mode = QString::fromLatin1(argv[1]);
    MainWindow win;
    win.resize(1920, 1040);
    win.show();
    if (auto *tree = win.findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"))) {
        const auto items = tree->findItems(QStringLiteral("Board"), Qt::MatchExactly | Qt::MatchRecursive);
        if (!items.isEmpty()) tree->setCurrentItem(items.first());
    }
    auto *link = win.findChild<BoardLink *>();
    auto *ip = win.findChild<QLineEdit *>(QStringLiteral("ECamNetIp"));
    auto *host = win.findChild<QLineEdit *>(QStringLiteral("ECamNetHost"));
    auto *dhcp = win.findChild<QComboBox *>(QStringLiteral("ECamNetMode"));
    auto *apply = win.findChild<QPushButton *>(QStringLiteral("ECamNetApply"));
    auto *state = win.findChild<QLabel *>(QStringLiteral("ECamNetState"));
    auto *inUse = win.findChild<QLabel *>(QStringLiteral("ECamNetInUse"));
    Check(link && ip && host && dhcp && apply && state && inUse, "found the Network fields");
    if (!link || !ip || !host || !dhcp || !apply || !state || !inUse) return 1;

    win.ConnectBoard(QString::fromLatin1(argv[2]));
    Check(WaitFor([&] { return link->isConnected() && link->state().valid; }, 15000), "connected");
    Check(WaitFor([&] { return !ip->text().isEmpty(); }, 8000), "the board's settings read in: " + ip->text());
    Check(WaitFor([&] { return !inUse->text().isEmpty(); }, 8000), "the address in use: " + inUse->text());
    const QString shot = qEnvironmentVariable("ECAM_UI_SHOT");
    if (!shot.isEmpty()) win.grab().save(shot);

    if (mode == QStringLiteral("readonly")) {
        Check(!ip->isEnabled() && !host->isEnabled() && !dhcp->isEnabled() && !apply->isEnabled(), "over Ethernet nothing can be changed");
        Check(state->text().contains(QStringLiteral("USB")), "it says why: " + state->text());
    } else if (mode == QStringLiteral("guards")) {
        const QString was = ip->text();
        Check(ip->isEnabled() && host->isEnabled() && dhcp->isEnabled(), "over USB the fields can be edited");
        Check(!apply->isEnabled(), "nothing changed: nothing to apply");
        ip->setText(QStringLiteral("10.168.41.300"));
        QApplication::processEvents();
        Check(!apply->isEnabled() && state->text().contains(QStringLiteral("IP address")), "a bad address can't be applied: " + state->text());
        ip->setText(QStringLiteral("10.168.41.6"));
        QApplication::processEvents();
        Check(apply->isEnabled(), "a good one can");
        ip->setText(was);
        QApplication::processEvents();
        bool done = false;
        link->command(QStringLiteral("enable"), [&](bool, const QStringList &) { done = true; });
        WaitFor([&] { return done && link->state().enabled; }, 6000);
        Check(link->state().enabled, "motors enabled");
        WaitFor([&] { return !ip->isEnabled(); }, 3000);
        Check(!ip->isEnabled() && !apply->isEnabled(), "enabled motors lock the fields: " + state->text());
        done = false;
        link->command(QStringLiteral("disable"), [&](bool, const QStringList &) { done = true; });
        WaitFor([&] { return done && !link->state().enabled; }, 6000);
        Check(WaitFor([&] { return ip->isEnabled(); }, 3000), "disabled again: they unlock");
    } else if (mode == QStringLiteral("estop")) {
        auto *estop = win.findChild<QComboBox *>(QStringLiteral("ECamEstopMode"));
        auto *estopState = win.findChild<QLabel *>(QStringLiteral("ECamEstopState"));
        Check(estop && estopState, "found the Estop row");
        if (!estop || !estopState) return 1;
        auto ask = [&](const QString &cmd) {
            bool done = false;
            link->command(cmd, [&](bool, const QStringList &) { done = true; });
            WaitFor([&] { return done; }, 5000);
            link->command(QStringLiteral("status"));
        };
        Check(WaitFor([&] { return estop->isEnabled(); }, 8000), "editable with the motors disabled");
        Check(estop->currentIndex() == 1, "the board's setting read in: " + estop->currentText());
        Check(WaitFor([&] { return estopState->text() == QStringLiteral("A-9 high: OK"); }, 4000), "it reads: " + estopState->text());
        ask(QStringLiteral("estop_test 1"));
        Check(WaitFor([&] { return estopState->text().endsWith(QStringLiteral("open")); }, 4000), "tripped: " + estopState->text());
        ask(QStringLiteral("estop_test 0"));
        Check(WaitFor([&] { return estopState->text().endsWith(QStringLiteral("OK")); }, 4000), "back: " + estopState->text());
        // Through the combo: fault when high (A-9 is high, so open), then back to fault when low.
        estop->setCurrentIndex(2);
        emit estop->activated(2);
        Check(WaitFor([&] { return estopState->text() == QStringLiteral("A-9 high: open"); }, 6000), "fault when high: " + estopState->text());
        estop->setCurrentIndex(1);
        emit estop->activated(1);
        Check(WaitFor([&] { return estopState->text() == QStringLiteral("A-9 high: OK") && estop->currentIndex() == 1; }, 6000),
              "fault when low again: " + estopState->text());
        bool done = false;
        QString cfg;
        link->command(QStringLiteral("config"), [&](bool, const QStringList &r) {
            cfg = r.join(QLatin1Char(' '));
            done = true;
        });
        WaitFor([&] { return done; }, 5000);
        Check(cfg.contains(QStringLiteral("estop_mode 1")) && cfg.contains(QStringLiteral("estop_pin A-9")), "the board keeps estop_mode 1 on A-9");
        // The pin: A-11 (nothing on it: low, so open), then back to A-9 through the page.
        auto *pinBox = win.findChild<QComboBox *>(QStringLiteral("ECamEstopPin"));
        Check(pinBox && pinBox->currentText() == QStringLiteral("A-9") && pinBox->isEnabled(), "the pin reads A-9");
        if (pinBox) {
            pinBox->setCurrentIndex(11);
            emit pinBox->activated(11);
            Check(WaitFor([&] { return estopState->text() == QStringLiteral("A-11 low: open"); }, 6000), "on A-11: " + estopState->text());
            pinBox->setCurrentIndex(9);
            emit pinBox->activated(9);
            Check(WaitFor([&] { return estopState->text() == QStringLiteral("A-9 high: OK") && pinBox->currentIndex() == 9; }, 6000),
                  "back on A-9: " + estopState->text());
        }
        if (!shot.isEmpty()) win.grab().save(shot);
    } else if (mode == QStringLiteral("status")) {
        auto *board = win.findChild<QLabel *>(QStringLiteral("ECamStatusBoard"));
        auto *estop = win.findChild<QLabel *>(QStringLiteral("ECamStatusEstop"));
        auto *m0 = win.findChild<QLabel *>(QStringLiteral("ECamStatusMotor0"));
        auto *m1 = win.findChild<QLabel *>(QStringLiteral("ECamStatusMotor1"));
        auto *fault = win.findChild<QLabel *>(QStringLiteral("ECamStatusFault"));
        Check(board && estop && m0 && m1 && fault, "found the status panel");
        if (!board || !estop || !m0 || !m1 || !fault) return 1;
        auto ask = [&](const QString &cmd) {
            bool done = false;
            link->command(cmd, [&](bool, const QStringList &) { done = true; });
            WaitFor([&] { return done; }, 6000);
            link->command(QStringLiteral("status"));
        };
        Check(WaitFor([&] { return board->text() == QStringLiteral("disabled") && estop->text() == QStringLiteral("OK"); }, 6000),
              "board disabled, estop OK");
        Check(WaitFor([&] { return m0->text() == QStringLiteral("Disabled") && m1->text() == QStringLiteral("Disabled"); }, 4000), "motors disabled");
        // The Inputs page's State: A-9 (the estop jumper) On, the rest as the board reads them.
        auto *inputs = win.findChild<QTableWidget *>(QStringLiteral("ECamInputsTable"));
        Check(inputs != nullptr, "found the Inputs table");
        if (inputs) {
            Check(WaitFor([&] { return inputs->item(9, 1)->text() == QStringLiteral("On"); }, 4000),
                  "Inputs: A-9 reads On (the jumper): " + inputs->item(9, 1)->text());
            QStringList states;
            for (int r = 0; r < inputs->rowCount(); ++r) states << inputs->item(r, 0)->text() + QLatin1Char('=') + inputs->item(r, 1)->text();
            std::printf("     %s\n", states.join(QLatin1Char(' ')).toUtf8().constData());
        }
        ask(QStringLiteral("estop_test 1"));
        Check(WaitFor([&] { return estop->text() == QStringLiteral("Open"); }, 4000), "estop open shows");
        ask(QStringLiteral("estop_test 0"));
        Check(WaitFor([&] { return estop->text() == QStringLiteral("OK"); }, 4000), "and OK again");
        ask(QStringLiteral("axjog 0 enable"));
        Check(WaitFor([&] { return m0->text() == QStringLiteral("Enabled") && m1->text() == QStringLiteral("Disabled"); }, 4000),
              "M-0 enabled alone shows Enabled (HLFB up), M-1 Disabled: " + m0->text());
        if (!shot.isEmpty()) win.grab().save(shot);
        ask(QStringLiteral("axjog 0 disable"));
        Check(WaitFor([&] { return m0->text() == QStringLiteral("Disabled"); }, 4000), "M-0 disabled again");
        Check(!fault->isVisible(), "no fault shown");
    } else if (mode == QStringLiteral("set") && argc >= 4) {
        const QString what = QString::fromLatin1(argv[3]);
        if (what == QStringLiteral("static")) {
            dhcp->setCurrentIndex(1);
            if (argc >= 5) ip->setText(QString::fromLatin1(argv[4]));
        } else if (what == QStringLiteral("dhcp")) {
            dhcp->setCurrentIndex(0);
        } else if (what == QStringLiteral("host") && argc >= 5) {
            host->setText(QString::fromLatin1(argv[4]) == QStringLiteral("-") ? QString() : QString::fromLatin1(argv[4]));
        }
        QApplication::processEvents();
        Check(apply->isEnabled(), "there is something to apply");
        const QString want = ip->text();
        win.ApplyBoardNetwork(false);
        const bool back = WaitFor([&] { return state->text().contains(QStringLiteral("Applied")) || state->text().contains(QStringLiteral("did not")); }, 40000);
        Check(back && state->text().contains(QStringLiteral("Applied")), "read back after the restart: " + state->text());
        Check(link->isConnected() && ip->text() == want, "the board holds it: " + ip->text());
        WaitFor([&] { return false; }, 300);
    } else {
        Check(false, "unknown mode");
    }
    link->disconnectLink();
    WaitFor([] { return false; }, 300);
    std::printf(g_fail ? "%d check(s) FAILED\n" : "all checks passed\n", g_fail);
    return g_fail ? 1 : 0;
}
