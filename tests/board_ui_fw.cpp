/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
// Bench check of the firmware update, through the real window against a real ClearCore on USB.
//
//   board_ui_fw update <COMn> <expected kind> <expected version> [<hash>]
//       Connects over USB when the board runs ECam firmware, opens Update Board Firmware from the Board page,
//       presses Update and waits. kind is ecam, other or boot; hash is the profile that must be on the board
//       afterwards, or - for none.
//   board_ui_fw refused <COMn>
//       The motors are enabled (the bench script does that): Update stays off.
//
// ECAM_FIRMWARE_DIR picks the image, ECAM_FIRMWARE_BACKUPS the backup folder, ECAM_UI_SHOT saves the window.

#include "BoardLink.h"
#include "FirmwareDialog.h"
#include "FirmwareUpdate.h"
#include "MainWindow.h"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
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

static QString Plain(QLabel *l) {
    QString s = l->text();
    s.remove(QRegularExpression(QStringLiteral("<[^>]*>")));
    return s;
}

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("ECamBench"));
    if (argc < 3) {
        std::puts("board_ui_fw update <COMn> <ecam|other|boot> <version> [hash|-]\nboard_ui_fw refused <COMn>");
        return 2;
    }
    const QString mode = QString::fromLocal8Bit(argv[1]);
    const QString port = QString::fromLocal8Bit(argv[2]);
    const QString kind = argc > 3 ? QString::fromLocal8Bit(argv[3]) : QString();
    const QString version = argc > 4 ? QString::fromLocal8Bit(argv[4]) : QString();
    const QString hash = argc > 5 ? QString::fromLocal8Bit(argv[5]) : QString();

    MainWindow mw;
    mw.resize(1920, 1040);
    mw.show();
    auto *tree = mw.findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"));
    const auto nodes = tree->findItems(QStringLiteral("Board"), Qt::MatchExactly | Qt::MatchRecursive);
    if (!nodes.isEmpty()) tree->setCurrentItem(nodes.first());

    // ECAM_FW_CONNECT: start connected over USB, as someone would, and read the version row.
    const bool connected = qEnvironmentVariableIsSet("ECAM_FW_CONNECT");
    auto *state = mw.findChild<QLabel *>(QStringLiteral("ECamFirmwareState"));
    auto *link = mw.findChild<BoardLink *>();
    if (connected) {
        for (QComboBox *b : mw.findChildren<QComboBox *>()) {
            if (b->count() == 2 && b->itemText(0) == QStringLiteral("Ethernet")) b->setCurrentIndex(1);
        }
        for (QComboBox *b : mw.findChildren<QComboBox *>()) {
            if (b->isEditable() && b->findData(port) >= 0) b->setCurrentIndex(b->findData(port));
        }
        for (QPushButton *b : mw.findChildren<QPushButton *>()) {
            if (b->text() == QStringLiteral("Connect")) {
                b->click();
                break;
            }
        }
        Check(WaitFor([&]() { return link->isConnected() && !link->firmwareVersion().isEmpty(); }, 6000), QStringLiteral("connected over USB"));
        WaitFor([&]() { return Plain(state).startsWith(QStringLiteral("Board ")); }, 2000);
        std::printf("     before: %s\n", Plain(state).toUtf8().constData());
    }

    auto *open = mw.findChild<QPushButton *>(QStringLiteral("ECamFwOpen"));
    Check(open != nullptr, QStringLiteral("the Board page has the firmware button"));
    if (!open) return 1;
    open->click();
    FirmwareDialog *dlg = nullptr;
    WaitFor([&]() { return (dlg = mw.findChild<FirmwareDialog *>()) != nullptr; }, 3000);
    Check(dlg != nullptr, QStringLiteral("the update window opens"));
    if (!dlg) return 1;
    auto *board = dlg->findChild<QLabel *>(QStringLiteral("ECamFwBoard"));
    auto *update = dlg->findChild<QPushButton *>(QStringLiteral("ECamFwUpdate"));
    auto *ports = dlg->findChild<QComboBox *>(QStringLiteral("ECamFwPort"));
    auto *stepLabel = dlg->findChild<QLabel *>(QStringLiteral("ECamFwStep"));
    auto *log = dlg->findChild<QPlainTextEdit *>(QStringLiteral("ECamFwLog"));

    // Drives are on USB here with the same vendor id: only the ClearCore may be offered.
    for (int i = 0; i < ports->count(); ++i) {
        Check(ports->itemText(i).contains(QStringLiteral("ClearCore")), QStringLiteral("port offered: %1").arg(ports->itemText(i)));
    }
    const int at = ports->findData(port);
    if (kind != QStringLiteral("boot")) {
        Check(at >= 0, QStringLiteral("%1 is offered").arg(port));
        if (at >= 0) ports->setCurrentIndex(at);
    }
    WaitFor([&]() { return !Plain(board).startsWith(QStringLiteral("Reading")); }, 8000);
    std::printf("     board: %s\n", Plain(board).toUtf8().constData());

    if (mode == QStringLiteral("refused")) {
        Check(!update->isEnabled(), QStringLiteral("Update is off"));
        Check(Plain(board).contains(QStringLiteral("Can't update now")), QStringLiteral("it says why"));
        dlg->reject();
        std::puts(g_fail ? "FAILED" : "all checks passed");
        return g_fail ? 1 : 0;
    }

    Check(update->isEnabled(), QStringLiteral("Update is on: %1").arg(update->text()));
    if (kind == QStringLiteral("other")) Check(Plain(board).contains(QStringLiteral("Not running ECam")), QStringLiteral("seen as not ECam"));
    if (kind == QStringLiteral("boot")) Check(Plain(board).contains(QStringLiteral("bootloader")), QStringLiteral("seen in its bootloader"));
    if (kind == QStringLiteral("ecam")) Check(Plain(board).contains(QStringLiteral("ECam firmware")), QStringLiteral("seen as ECam"));
    if (auto *restore = dlg->findChild<QCheckBox *>(QStringLiteral("ECamFwRestore")); restore && !restore->isHidden()) {
        std::printf("     offered: %s (ticked %d)\n", restore->text().toUtf8().constData(), restore->isChecked() ? 1 : 0);
        if (qEnvironmentVariableIsSet("ECAM_FW_RESTORE")) restore->setChecked(true);
    }
    if (qEnvironmentVariableIsSet("ECAM_UI_SHOT")) {
        QApplication::processEvents();
        dlg->grab().save(qEnvironmentVariable("ECAM_UI_SHOT"));
    }

    QElapsedTimer t;
    t.start();
    update->click();
    Check(dlg->busy(), QStringLiteral("running"));
    dlg->reject();
    Check(dlg->isVisible(), QStringLiteral("the window won't close while it runs"));
    WaitFor([&]() { return !dlg->busy(); }, 200000);
    std::printf("     took %.1f s\n     %s\n", t.elapsed() / 1000.0, Plain(stepLabel).toUtf8().constData());
    Check(dlg->finishedOk(), QStringLiteral("the update finished"));
    if (!dlg->finishedOk()) std::printf("--- log\n%s\n---\n", log->toPlainText().toUtf8().constData());
    if (!hash.isEmpty() && hash != QStringLiteral("-")) {
        Check(dlg->finishedOk() && Plain(stepLabel).contains(hash + QStringLiteral(" restored")), QStringLiteral("profile %1 restored").arg(hash));
        Check(dlg->finishedOk() && dlg->result().backupKept.isEmpty(), QStringLiteral("the backup file is gone"));
    }
    if (!version.isEmpty() && version != QStringLiteral("-")) {
        Check(dlg->finishedOk() && Plain(stepLabel).contains(version), QStringLiteral("firmware %1 reported").arg(version));
    }
    if (qEnvironmentVariableIsSet("ECAM_UI_SHOT")) {
        QApplication::processEvents();
        dlg->grab().save(qEnvironmentVariable("ECAM_UI_SHOT").replace(QStringLiteral(".png"), QStringLiteral("_done.png")));
    }
    if (connected) {
        dlg->reject();
        Check(WaitFor([&]() { return link->isConnected() && link->state().valid && !link->firmwareVersion().isEmpty(); }, 35000),
              QStringLiteral("the window connects again by itself"));
        WaitFor([&]() { return Plain(state).contains(QStringLiteral("matches")); }, 3000);
        std::printf("     after: %s\n", Plain(state).toUtf8().constData());
        Check(Plain(state).contains(QStringLiteral("matches this app")), QStringLiteral("the version row says it matches"));
    }
    std::puts(g_fail ? "FAILED" : "all checks passed");
    return g_fail ? 1 : 0;
}
