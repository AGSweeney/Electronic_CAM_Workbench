/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
// The real scope window on the real drives: Connect all, Start, record, Stop, Disconnect, through its own
// buttons. Run the motors yourself if the lanes should show anything; this never enables or moves one.
//
//   board_ui_scope <motors expected> [seconds] [shot.png] [name name ...]
//
// Close ClearPath-MSP first (one program owns a port).

#include "ScopeWindow.h"

#include <QApplication>
#include <QElapsedTimer>
#include <QMap>
#include <QPushButton>
#include <QSettings>
#include <QStringList>
#include <QVariant>

#include <cmath>
#include <cstdio>

using scope::Lane;

static int g_fail = 0;

static void Check(bool ok, const QString &what) {
    std::printf("%s %s\n", ok ? "ok  " : "FAIL", qPrintable(what));
    if (!ok) ++g_fail;
}

static QPushButton *Button(ScopeWindow &w, const QString &text) {
    for (QPushButton *b : w.findChildren<QPushButton *>()) {
        if (b->text().contains(text)) return b;
    }
    return nullptr;
}

template <typename F>
static bool WaitFor(F done, int ms) {
    QElapsedTimer t;
    t.start();
    while (!done() && t.elapsed() < ms) QApplication::processEvents(QEventLoop::AllEvents, 20);
    return done();
}

int main(int argc, char **argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    QApplication app(argc, argv);
    const QStringList a = app.arguments();
    if (a.size() < 2) {
        std::printf("usage: board_ui_scope <motors expected> [seconds] [shot.png] [name name ...]\n");
        return 2;
    }
    const int want = a[1].toInt();
    const int seconds = a.size() > 2 ? a[2].toInt() : 10;
    const QString shot = a.size() > 3 ? a[3] : QString();

    // The window saves to the app's own settings: put the user's back afterwards.
    QSettings user(QStringLiteral("ECamUI"), QStringLiteral("ECamUI"));
    user.beginGroup(QStringLiteral("clearpathScope"));
    QMap<QString, QVariant> saved;
    for (const QString &k : user.allKeys()) saved.insert(k, user.value(k));
    user.endGroup();

    int result = 0;
    {
    ScopeWindow w;
    w.resize(1480, 940);
    w.show();
    QApplication::processEvents();

    QPushButton *all = Button(w, QStringLiteral("Connect all"));
    Check(all && all->isEnabled(), QStringLiteral("Connect all is there"));
    if (all) all->click();
    const bool connected = WaitFor([&] { return w.motorCount() == want; }, 15000);
    Check(connected, QStringLiteral("%1 motors connected (%2): %3").arg(want).arg(w.motorCount()).arg(w.statusText()));
    for (int i = 4; i < a.size(); ++i) w.setMotorName(i - 4, a[i]);

    QPushButton *run = Button(w, QStringLiteral("Start"));
    Check(run && run->isEnabled(), QStringLiteral("Start is enabled"));
    if (run) run->click();
    Check(WaitFor([&] { return w.recording(); }, 8000), QStringLiteral("recording"));
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < seconds * 1000) QApplication::processEvents(QEventLoop::AllEvents, 10);
    std::printf("     %s\n", qPrintable(w.statusText()));
    std::printf("     slowest frame %.1f ms, %d frames\n", w.maxPaintMs(), w.plotFrames());

    double newest[scope::kMaxMotors] = {};
    int fed = 0;
    for (const Lane &l : w.lanes()) {
        const scope::SignalDef *d = scope::Find(l.key);
        if (!l.on || !d || d->source != scope::Source::Stream) continue;
        const scope::Series *s = w.series(l.key);
        const int n = s ? static_cast<int>(s->t.size()) : 0;
        bool ordered = true;
        for (int i = 1; i < n; ++i) ordered = ordered && s->t[i] >= s->t[i - 1];
        const double span = n > 1 ? s->t.back() - s->t.front() : 0.0;
        const double rate = span > 0 ? (n - 1) / span : 0.0;
        std::printf("     %s: %d samples, %.1f/s\n", qPrintable(l.key), n, rate);
        Check(n > 0 && std::fabs(rate - 844.35) < 5.0, QStringLiteral("%1 streams at the drive's rate").arg(l.key));
        Check(ordered, QStringLiteral("%1 is in time order").arg(l.key));
        if (n) newest[scope::MotorOf(l.key)] = s->t.back();
        ++fed;
    }
    Check(fed == want, QStringLiteral("a fast lane from every motor (%1)").arg(fed));
    double lo = 1e18, hi = -1e18;
    for (int m = 0; m < want; ++m) {
        lo = std::min(lo, newest[m]);
        hi = std::max(hi, newest[m]);
    }
    Check(hi - lo < 0.2, QStringLiteral("motors end together on one clock (%1 ms apart)").arg((hi - lo) * 1000.0, 0, 'f', 1));

    if (!shot.isEmpty()) {
        Check(w.grab().save(shot), QStringLiteral("screenshot"));
    }
    QPushButton *stop = Button(w, QStringLiteral("Stop"));
    if (stop) stop->click();
    const bool stopped = WaitFor([&] { return !w.recording(); }, 8000);
    Check(stopped, QStringLiteral("stopped: %1").arg(w.statusText()));
    Check(w.statusText().contains(QStringLiteral("channel 1 restored")), QStringLiteral("channel 1 put back"));
    QPushButton *off = Button(w, QStringLiteral("Disconnect"));
    if (off) off->click();
    Check(WaitFor([&] { return w.motorCount() == 0; }, 8000), QStringLiteral("disconnected"));
    w.close();
    result = g_fail ? 1 : 0;
    }
    QApplication::processEvents();
    user.remove(QStringLiteral("clearpathScope"));
    for (auto it = saved.cbegin(); it != saved.cend(); ++it) {
        user.setValue(QStringLiteral("clearpathScope/") + it.key(), it.value());
    }
    user.sync();
    std::printf(g_fail ? "%d check(s) FAILED\n" : "all checks passed\n", g_fail);
    return result;
}
