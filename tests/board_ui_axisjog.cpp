/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
// The Axis jog panel against a real board, through its own buttons. It enables and moves only the two motors given:
// one a profile axis, the other one the profile doesn't use (free shafts only); it never enables the others. The
// running-cam checks use simulate, so nothing else moves. It leaves every motor disabled and simulate off, and
// doesn't change the board's profile.
//
//   board_ui_axisjog <ip | COMn> <axis motor> <free motor> [shot.png]      motors as connectors, 0..3
//   exit code 0 = all passed

#include "BoardLink.h"
#include "JogDialog.h"

#include <QApplication>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QPushButton>
#include <QSpinBox>

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
    while (!pred() && t.elapsed() < ms) QApplication::processEvents(QEventLoop::AllEvents, 10);
    return pred();
}

static void Pump(int ms) {
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < ms) QApplication::processEvents(QEventLoop::AllEvents, 10);
}

// One command, waited for: the reply's last line.
static QString Ask(BoardLink &b, const QString &cmd) {
    bool done = false;
    QString last;
    b.command(cmd, [&](bool, const QStringList &r) {
        last = r.isEmpty() ? QString() : r.last();
        done = true;
    });
    WaitFor([&] { return done; }, 5000);
    return last;
}

template <typename T>
static T *Find(JogDialog &d, const QString &name) { return d.findChild<T *>(name); }

int main(int argc, char **argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    QApplication app(argc, argv);
    QCoreApplication::setOrganizationName(QStringLiteral("ECamBenchTest"));
    QCoreApplication::setApplicationName(QStringLiteral("board_ui_axisjog"));
    const QStringList a = app.arguments();
    if (a.size() < 4) {
        std::printf("usage: board_ui_axisjog <ip | COMn> <axis motor> <free motor> [shot.png]\n");
        return 2;
    }
    const QString target = a[1];
    const int ax = a[2].toInt(), fr = a[3].toInt();
    const QString shot = a.size() > 4 ? a[4] : QString();

    BoardLink board;
    QString err;
    const bool serial = target.startsWith(QLatin1String("COM"), Qt::CaseInsensitive);
    if (!(serial ? board.connectSerial(target, &err) : board.connectEthernet(target, &err))) {
        std::printf("connect: %s\n", qPrintable(err));
        return 2;
    }
    Check(WaitFor([&] { return board.isConnected() && board.state().valid; }, 8000), "connected, status read");
    Ask(board, QStringLiteral("status"));
    Pump(200);
    const quint32 hash = board.state().profile;
    std::printf("     board profile %08x, firmware %s\n", hash, qPrintable(board.firmwareVersion()));
    Ask(board, QStringLiteral("disable"));
    Ask(board, QStringLiteral("sim 0"));

    JogDialog dlg(&board);
    dlg.show();
    Check(WaitFor([&] { return dlg.motors().size() == 4; }, 5000), "a card for every motor");
    if (dlg.motors().size() != 4) return 1;
    Pump(300);
    for (const JogDialog::MotorInfo &i : dlg.motors()) {
        static const char *const kRole[] = {"free", "axis", "crank"};
        std::printf("     M-%d %s %s: %.6g counts/%s, limit %.6g, pos %lld\n", i.motor, kRole[int(i.role)], qPrintable(i.name), i.cpu,
                    qPrintable(i.units), i.vmax, (long long)i.pos);
    }
    Check(dlg.motors()[ax].role == JogDialog::Role::Axis, QStringLiteral("M-%1 is a profile axis").arg(ax));
    Check(dlg.motors()[fr].role == JogDialog::Role::Free, QStringLiteral("M-%1 is not in the profile").arg(fr));
    std::printf("     panel %dx%d\n", dlg.width(), dlg.height());
    Check(dlg.width() <= 1000 && dlg.height() <= 900, "the panel fits a 1080p screen");
    auto info = [&](int m) { return dlg.motors().value(m); };
    auto pumpPoll = [&]() { Pump(400); };

    // ---- a running cam locks the panel (simulate: no motor moves)
    Check(Ask(board, QStringLiteral("sim 1")).startsWith(QLatin1String("ok")), "simulate on");
    Check(Ask(board, QStringLiteral("start")).startsWith(QLatin1String("ok")), "cam started (simulated)");
    Check(WaitFor([&] { return dlg.camActive(); }, 3000), "the panel sees the cam running");
    pumpPoll();
    for (int m : {ax, fr}) {
        auto *plus = Find<QPushButton>(dlg, QStringLiteral("AxisJogPlus%1").arg(m));
        auto *zero = Find<QPushButton>(dlg, QStringLiteral("AxisJogZero%1").arg(m));
        auto *en = Find<QPushButton>(dlg, QStringLiteral("AxisJogEnable%1").arg(m));
        Check(plus && !plus->isEnabled() && zero && !zero->isEnabled() && en && !en->isEnabled(),
              QStringLiteral("M-%1: jog, zero and enable locked while the cam runs").arg(m));
        const QString refused = Ask(board, QStringLiteral("axjog %1 jog 1").arg(m));
        Check(refused.contains(QLatin1String("cam is running")), QStringLiteral("M-%1: the board refuses a jog: %2").arg(m).arg(refused));
    }
    auto *stop = Find<QPushButton>(dlg, QStringLiteral("AxisJogStopCam"));
    Check(stop && stop->isVisible() && stop->isEnabled(), "Stop offered");
    if (stop) stop->click();
    Check(WaitFor([&] { return !dlg.camActive(); }, 8000), "Stop brings it to rest");
    Ask(board, QStringLiteral("start"));
    WaitFor([&] { return dlg.camActive(); }, 3000);
    auto *all = Find<QPushButton>(dlg, QStringLiteral("AxisJogDisableAll"));
    Check(all && all->isEnabled(), "Disable all offered while it runs");
    if (all) all->click();
    Check(WaitFor([&] { return !dlg.camActive(); }, 5000), "Disable all stops the cam at once");
    Check(Ask(board, QStringLiteral("sim 0")).startsWith(QLatin1String("ok")), "simulate off");
    pumpPoll();

    // ---- enable the two motors, one at a time
    for (int m : {ax, fr}) {
        auto *plus = Find<QPushButton>(dlg, QStringLiteral("AxisJogPlus%1").arg(m));
        Check(plus && !plus->isEnabled(), QStringLiteral("M-%1: jog needs the motor on").arg(m));
        Find<QPushButton>(dlg, QStringLiteral("AxisJogEnable%1").arg(m))->click();
        Check(WaitFor([&] { return info(m).on; }, 4000), QStringLiteral("M-%1 enabled").arg(m));
    }
    Ask(board, QStringLiteral("status"));
    Pump(200);
    Check(board.state().motorsOn == ((1u << ax) | (1u << fr)), QStringLiteral("only those two (mon=%1)").arg(board.state().motorsOn, 0, 16));

    // ---- hold to jog, the axis then the free motor
    for (int m : {ax, fr}) {
        auto *speed = Find<QDoubleSpinBox>(dlg, QStringLiteral("AxisJogSpeed%1").arg(m));
        const double k = m == fr ? Find<QSpinBox>(dlg, QStringLiteral("AxisJogCpr%1").arg(m))->value() : info(m).cpu;
        speed->setValue(2400.0 / k);  // 2400 counts/s: 3 rev/s of an 800-count motor
        const qint64 p0 = info(m).pos;
        QElapsedTimer held;
        held.start();
        dlg.pressJog(m, +1);
        Pump(1500);
        Check(info(m).moving, QStringLiteral("M-%1 moving while held").arg(m));
        dlg.releaseJog();
        const double heldS = held.elapsed() / 1000.0;
        Check(WaitFor([&] { return !info(m).moving; }, 3000), QStringLiteral("M-%1 stops when let go").arg(m));
        Pump(300);
        const double moved = (info(m).pos - p0) / 2400.0;  // seconds at full speed
        std::printf("     M-%d held %.2f s: moved %.3f s worth\n", m, heldS, moved);
        Check(moved > heldS - 0.6 && moved < heldS + 0.3, QStringLiteral("M-%1 moved about the speed times the hold").arg(m));
    }
    Check(info(ax).off != 0, "the axis is off its cam position");
    Check(info(fr).off == 0, "a free motor has no cam position to be off");

    // ---- steps on both at once: exact
    for (int m : {ax, fr}) Find<QDoubleSpinBox>(dlg, QStringLiteral("AxisJogStep%1").arg(m))->setValue(m == fr ? 2.0 : 10.0);
    const qint64 a1 = info(ax).pos, f1 = info(fr).pos;
    Find<QPushButton>(dlg, QStringLiteral("AxisJogStepMinus%1").arg(ax))->click();
    Find<QPushButton>(dlg, QStringLiteral("AxisJogStepPlus%1").arg(fr))->click();
    Check(WaitFor([&] { return info(ax).moving && info(fr).moving; }, 1500), "both move at once");
    Check(WaitFor([&] { return !info(ax).moving && !info(fr).moving; }, 15000), "both steps done");
    Pump(300);
    Check(info(ax).pos - a1 == -llround(10.0 * info(ax).cpu), QStringLiteral("the axis's step of 10 is exact (%1 counts)").arg(info(ax).pos - a1));
    Check(info(fr).pos - f1 == 1600, QStringLiteral("the free motor's 2 rev at 800 counts/rev is exact (%1 counts)").arg(info(fr).pos - f1));

    // ---- Start and Hand jog wait for the axis (simulate, so Start needs no motor)
    Ask(board, QStringLiteral("disable"));
    Ask(board, QStringLiteral("sim 1"));
    const QString st = Ask(board, QStringLiteral("start"));
    Check(st.contains(QLatin1String("off its cam position")), QStringLiteral("Start refused off cam: %1").arg(st));
    const QString hj = Ask(board, QStringLiteral("jog on"));
    Check(hj.contains(QLatin1String("off its cam position")), QStringLiteral("Hand jog refused off cam: %1").arg(hj));
    Ask(board, QStringLiteral("sim 0"));
    pumpPoll();
    for (int m : {ax, fr}) {
        Find<QPushButton>(dlg, QStringLiteral("AxisJogEnable%1").arg(m))->click();
        WaitFor([&] { return info(m).on; }, 4000);
    }
    pumpPoll();

    // ---- To cam position; zero on both
    auto *toCam = Find<QPushButton>(dlg, QStringLiteral("AxisJogToCam%1").arg(ax));
    Check(toCam && toCam->isEnabled(), "To cam position offered");
    if (toCam) toCam->click();
    Pump(300);
    Check(WaitFor([&] { return !info(ax).moving && info(ax).off == 0 && info(ax).pos == info(ax).cam; }, 15000), "the axis is back on its cam position");
    Find<QPushButton>(dlg, QStringLiteral("AxisJogZero%1").arg(fr))->click();
    Check(WaitFor([&] { return info(fr).pos == 0; }, 3000), "Zero: the free motor reads 0");
    Find<QPushButton>(dlg, QStringLiteral("AxisJogZero%1").arg(ax))->click();
    Check(WaitFor([&] { return info(ax).pos == 0; }, 3000), "Zero: the axis reads 0");
    Pump(300);
    std::printf("     after zero: axis cam %lld, off %lld\n", (long long)info(ax).cam, (long long)info(ax).off);
    if (info(ax).off != 0 && toCam) {
        toCam->click();
        Pump(300);
        WaitFor([&] { return !info(ax).moving && info(ax).off == 0; }, 15000);
    }
    Check(info(ax).off == 0, "the axis is left on its cam position");

    // ---- the deadman, on the free motor: a jog sent once stops by itself
    Check(Ask(board, QStringLiteral("axjog %1 jog 2400").arg(fr)).startsWith(QLatin1String("ok")), "a jog sent once");
    QElapsedTimer dm;
    dm.start();
    Check(WaitFor([&] { return info(fr).moving; }, 800), "it moves");
    Check(WaitFor([&] { return !info(fr).moving; }, 3000), "and stops by itself");
    std::printf("     deadman: stopped %lld ms after the only command\n", (long long)dm.elapsed());
    Check(dm.elapsed() > 450 && dm.elapsed() < 1600, "about half a second after it (plus the ramp and the poll)");

    if (!shot.isEmpty()) {
        Pump(400);
        Check(dlg.grab().save(shot), "screenshot");
    }

    // ---- disable from the cards; closing stops everything
    for (int m : {ax, fr}) {
        Find<QPushButton>(dlg, QStringLiteral("AxisJogEnable%1").arg(m))->click();
        Check(WaitFor([&] { return !info(m).on; }, 4000), QStringLiteral("M-%1 disabled from its card").arg(m));
    }
    dlg.reject();
    Pump(300);
    Ask(board, QStringLiteral("disable"));
    Ask(board, QStringLiteral("sim 0"));
    Ask(board, QStringLiteral("status"));
    Pump(200);
    Check(board.state().profile == hash, "the board's profile is unchanged");
    Check(!board.state().enabled && board.state().motorsOn == 0 && !board.state().simulate, "left disabled, simulate off");
    board.disconnectLink();
    std::printf(g_fail ? "%d check(s) FAILED\n" : "all checks passed\n", g_fail);
    return g_fail ? 1 : 0;
}
