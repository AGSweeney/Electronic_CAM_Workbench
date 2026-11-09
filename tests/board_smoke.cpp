/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
// Bench check of BoardLink against a ClearCore running ECam_Firmware. Never enables motors: the
// profile runs in simulate mode and only telemetry is checked.
//
//   board_smoke <ip | COMn> <project.ecam>
//
// Exit code 0 = all passed.

#include "BoardLink.h"
#include "CamProfile.h"
#include "ProjectIo.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QTimer>

#include <cmath>
#include <cstdio>

static int g_fail = 0;
static void Check(bool ok, const QString &what) {
    std::printf("%s %s\n", ok ? "ok  " : "FAIL", qPrintable(what));
    if (!ok) ++g_fail;
}

// Run the event loop until pred() or the timeout.
template <typename F>
static bool WaitFor(F pred, int ms) {
    QElapsedTimer t;
    t.start();
    while (!pred() && t.elapsed() < ms) QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    return pred();
}

static bool Run(BoardLink &b, const QString &cmd, QStringList *reply = nullptr) {
    bool done = false, ok = false;
    b.command(cmd, [&](bool k, const QStringList &r) {
        ok = k;
        done = true;
        if (reply) *reply = r;
    });
    WaitFor([&] { return done; }, 5000);
    return ok;
}

int main(int argc, char *argv[]) {
    QCoreApplication app(argc, argv);
    if (argc < 3) {
        std::printf("usage: board_smoke <ip | COMn> <project.ecam>\n");
        return 2;
    }
    const QString target = argv[1];
    ecam::CamProject project;
    QString err;
    if (!io::LoadProject(argv[2], &project, &err)) {
        std::printf("cannot load %s: %s\n", argv[2], qPrintable(err));
        return 2;
    }

    BoardLink b;
    const bool usb = target.startsWith(QStringLiteral("COM"), Qt::CaseInsensitive);
    Check(usb ? b.connectSerial(target, &err) : b.connectEthernet(target, &err),
          QStringLiteral("connect %1 %2").arg(target, err));
    if (!b.isConnected()) return 1;
    Check(WaitFor([&] { return b.state().valid; }, 3000), QStringLiteral("status received"));

    Run(b, QStringLiteral("abort"));
    Run(b, QStringLiteral("disable"));
    bool upDone = false, upOk = false;
    QString upMsg;
    QElapsedTimer t;
    t.start();
    b.uploadProfile(project, [&](bool ok, const QString &m) {
        upOk = ok;
        upMsg = m;
        upDone = true;
    });
    WaitFor([&] { return upDone; }, 15000);
    Check(upOk, QStringLiteral("upload: %1 (%2 ms)").arg(upMsg).arg(t.elapsed()));

    Check(Run(b, QStringLiteral("sim 1")), QStringLiteral("simulate on"));
    Check(Run(b, QStringLiteral("start")), QStringLiteral("start"));
    Check(Run(b, QStringLiteral("trigger")), QStringLiteral("trigger"));

    // Watch telemetry for a second. Positions are counts since the load, so each timeline axis's
    // change since the first telemetry line must match the profile's change over the same time.
    QVector<qint64> first;
    double t0 = -1;
    int tel = 0;
    double worstCounts = 0;
    QObject::connect(&b, &BoardLink::stateUpdated, [&](const BoardLink::State &s) {
        if (s.timeline != QStringLiteral("running") || s.axisCounts.size() != int(project.axes.size())) return;
        ++tel;
        if (t0 < 0) {
            t0 = s.t;
            first = s.axisCounts;
            return;
        }
        for (int i = 0; i < int(project.axes.size()); ++i) {
            const ecam::CamAxis &a = project.axes[i];
            if (a.master != ecam::AxisMaster::Timeline) continue;
            const double want = (ecam::Evaluate(a, std::min(s.t, a.span)).pos - ecam::Evaluate(a, std::min(t0, a.span)).pos);
            const double got = (s.axisCounts[i] - first[i]) / a.countsPerUnit;
            worstCounts = std::max(worstCounts, std::fabs(got - want) * a.countsPerUnit);
        }
    });
    WaitFor([] { return false; }, 1200);
    Check(tel > 10, QStringLiteral("telemetry while running: %1 lines in 1.2 s").arg(tel));
    // Both ends are whole motor counts, each within half a count of the exact profile.
    Check(worstCounts <= 1.5, QStringLiteral("board positions match the profile (worst %1 motor counts)").arg(worstCounts, 0, 'f', 2));

    Check(Run(b, QStringLiteral("stop")), QStringLiteral("stop"));
    WaitFor([&] { return b.state().timeline == QStringLiteral("idle"); }, 3000);
    Check(b.state().timeline == QStringLiteral("idle"), QStringLiteral("timeline back to idle"));
    QStringList diag;
    Run(b, QStringLiteral("diag"), &diag);
    if (!diag.isEmpty()) std::printf("     %s\n", qPrintable(diag.first()));
    b.disconnectLink();

    std::printf(g_fail ? "%d check(s) FAILED\n" : "all checks passed\n", g_fail);
    return g_fail ? 1 : 0;
}
