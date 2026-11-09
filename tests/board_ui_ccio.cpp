/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
// Bench check of the CCIO-8 page, through the real window against a real board.
//
//   board_ui_ccio <ip> <boards expected on the link>
//
// Connects, opens the CCIO-8 node, sets the count and reads back what the page says. ECAM_UI_SHOT saves the page.

#include "BoardLink.h"
#include "MainWindow.h"

#include <QApplication>
#include <QComboBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QPushButton>
#include <QRegularExpression>
#include <QSpinBox>
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

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("ECamBench"));
    if (argc < 3) {
        std::puts("board_ui_ccio <ip> <boards>");
        return 2;
    }
    const QString ip = QString::fromLocal8Bit(argv[1]);
    const int want = QString::fromLocal8Bit(argv[2]).toInt();

    MainWindow w;
    w.resize(1920, 1040);
    w.show();
    auto *link = w.findChild<BoardLink *>();
    QString error;
    const bool connected = link->connectEthernet(ip, &error);
    if (!connected) std::printf("     %s\n", error.toUtf8().constData());
    Check(connected, QStringLiteral("connected to %1").arg(ip));
    Check(WaitFor([&]() { return link->state().valid; }, 5000), QStringLiteral("status read"));
    std::printf("     board reports %d board(s), link %s, inputs %016llx\n", link->state().ccioBoards,
                link->state().ccioBroken ? "broken" : "ok", static_cast<unsigned long long>(link->state().ccioInputs));
    Check(link->state().ccioBoards == want, QStringLiteral("the board found %1").arg(want));

    auto *tree = w.findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"));
    const auto nodes = tree->findItems(QStringLiteral("CCIO-8"), Qt::MatchExactly | Qt::MatchRecursive);
    Check(nodes.size() == 1, QStringLiteral("the tree has CCIO-8"));
    if (nodes.isEmpty()) return 1;
    tree->setCurrentItem(nodes.first());
    auto *boards = w.findChild<QSpinBox *>(QStringLiteral("ECamCcioBoards"));
    auto *state = w.findChild<QLabel *>(QStringLiteral("ECamCcioState"));
    auto *table = w.findChild<QTableWidget *>(QStringLiteral("ECamCcioTable"));
    auto plain = [&]() { return state->text().remove(QRegularExpression(QStringLiteral("<[^>]*>"))); };

    boards->setValue(want + 1);
    WaitFor([&]() { return plain().contains(QStringLiteral("this project has")); }, 2000);
    std::printf("     one too many: %s\n", plain().toUtf8().constData());
    Check(plain().contains(QStringLiteral("this project has")), QStringLiteral("a count that doesn't match is called out"));
    boards->setValue(want);
    WaitFor([&]() { return plain().startsWith(QStringLiteral("Link OK")); }, 2000);
    std::printf("     matching: %s\n", plain().toUtf8().constData());
    Check(plain().startsWith(QStringLiteral("Link OK")), QStringLiteral("the page says the link is good"));
    if (want > 0) {
        QStringList states;
        for (int r = 0; r < table->rowCount(); ++r) {
            states << table->item(r, 0)->text() + QLatin1Char('=') + table->item(r, 1)->text();
        }
        std::printf("     %s\n", states.join(QLatin1Char(' ')).toUtf8().constData());
        Check(table->item(0, 1)->text() == QStringLiteral("On") || table->item(0, 1)->text() == QStringLiteral("Off"), QStringLiteral("inputs are live"));
    }
    if (qEnvironmentVariableIsSet("ECAM_UI_SHOT")) {
        QApplication::processEvents();
        w.grab().save(qEnvironmentVariable("ECAM_UI_SHOT"));
    }
    link->disconnectLink();
    std::puts(g_fail ? "FAILED" : "all checks passed");
    return g_fail ? 1 : 0;
}
