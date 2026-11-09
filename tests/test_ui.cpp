/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
// Drives the real main window with synthetic input: drag a knot in the displacement diagram,
// edit a table cell, change a law, then undo / redo. Exit code 0 = all passed.
// Opens a real window for a moment (the deployed build has only the windows platform plugin).

#include <QPlainTextEdit>
#include <QPlainTextEdit>
#include <QTemporaryDir>

#include "Brand.h"
#include "CollapsibleGroup.h"
#include "FirmwareDialog.h"
#include "FirmwareUpdate.h"
#include "JogDialog.h"
#include "SwitchWizard.h"
#include "OperatorDialog.h"
#include "MainWindow.h"
#include "ProfileStrip.h"
#include "ProjectIo.h"
#include "ProjectReview.h"
#include "ProfileText.h"
#include "SideDock.h"
#include "TimingView.h"
#include "ProjectReport.h"
#include "WorkbookExport.h"
#include "ClearPathLink.h"
#include "ScopeWindow.h"
#include "SetupWizard.h"
#include "StartupUi.h"
#include "CamTemplates.h"

#include <QAction>
#include "NetSettings.h"

#include <QApplication>
#include <QEventLoop>
#include <QTcpServer>
#include <QTcpSocket>
#include <QGroupBox>
#include <QStackedWidget>
#include <QSplitter>
#include <QStandardItemModel>
#include <QMap>
#include <QFrame>
#include <QScrollArea>
#include <QScrollBar>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QKeyEvent>
#include <QLineEdit>
#include <QPushButton>
#include <QWheelEvent>
#include <QLabel>
#include <QDir>
#include <QFile>
#include <QFormLayout>
#include <QMessageBox>
#include <QStatusBar>
#include <QAbstractButton>
#include <QMouseEvent>
#include <QSpinBox>
#include <QTableWidget>
#include <QTextBrowser>
#include <QRegularExpression>
#include <QFileInfo>
#include <QImage>
#include <map>
#include "HelpWindow.h"
#include <QPainter>
#include <QTextDocument>
#include <memory>
#include <QTimer>
#include <QScreen>
#include <QSettings>
#include <QSplashScreen>
#include <QUndoStack>

#include <algorithm>
#include <cmath>
#include <cstdio>

static int g_fail = 0;

static void Check(bool ok, const char *what) {
    std::printf("%s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) ++g_fail;
}

static double Cell(QTableWidget *t, int r, int c) { return t->item(r, c)->text().toDouble(); }

static bool Near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

static void Mouse(QWidget *w, QEvent::Type type, QPointF p, Qt::MouseButtons buttons, Qt::KeyboardModifiers mods = {}) {
    const Qt::MouseButton b = type == QEvent::MouseMove ? Qt::NoButton : Qt::LeftButton;
    QMouseEvent e(type, p, w->mapToGlobal(p), b, buttons, mods);
    QApplication::sendEvent(w, &e);
}

// Upload lines with the project copy: the board's reader accepts them with the same hash, and the
// project comes back exactly; without the copy (older uploads) the motion still comes back.
static void TestUploadRoundTrip() {
    ecam::CamProject p;
    QString err;
    if (!io::LoadProject(QStringLiteral(ECAM_SOURCE_DIR "/samples/mixed_timeline.ecam"), &p, &err)) {
        Check(false, "load samples/mixed_timeline.ecam");
        return;
    }
    p.axes[0].baseRadius = 41.5;  // editing-only fields the motion lines do not carry
    p.axes[0].velMax = 123.0;
    const std::vector<std::string> lines = io::UploadLines(p);
    ecam::ProfileReader reader;
    reader.Begin();
    bool accepted = true;
    QStringList text;
    for (const std::string &l : lines) {
        accepted = accepted && reader.Line(l.c_str()) == nullptr && l.size() < static_cast<size_t>(ecam::kProfileMaxLine);
        text << QString::fromStdString(l);
    }
    ecam::CamProject loaded;
    Check(accepted && reader.Finish(&loaded) == nullptr, "board reader accepts the upload lines (with json lines)");
    Check(reader.Hash() == ecam::ProfileHash(lines), "board hash = app hash, project copy included");
    std::printf("     %zu lines, %zu bytes on the board\n", lines.size(), reader.Text().size());

    ecam::CamProject back;
    bool exact = false;
    Check(io::ProjectFromUpload(text, &back, &exact, &err) && exact, "download: project copy found");
    Check(io::ProjectToJson(back, false) == io::ProjectToJson(p, false), "download: identical project (drawing, limits, mechanics)");

    QStringList motionOnly;
    for (const QString &l : text) {
        if (!l.startsWith(QStringLiteral("json "))) motionOnly << l;
    }
    Check(io::ProjectFromUpload(motionOnly, &back, &exact, &err) && !exact, "download without copy: rebuilt from motion lines");
    Check(ecam::ProfileHash(ecam::ProfileLines(back)) == ecam::ProfileHash(ecam::ProfileLines(p)), "download without copy: same motion");
}

int main(int argc, char *argv[]) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // every line out before a crash
    QApplication app(argc, argv);
    // The windows here save to the app's own settings as they close (window layout, splits, the guide's geometry,
    // the panels' last values): put every one of the user's back when the tests end. The window layout starts from
    // the defaults, so the checks don't depend on how the user last left the app.
    QSettings userAll(QStringLiteral("ECamUI"), QStringLiteral("ECamUI"));
    struct SettingsRestore {
        QSettings &s;
        QMap<QString, QVariant> saved;
        ~SettingsRestore() {
            s.clear();
            for (auto it = saved.cbegin(); it != saved.cend(); ++it) s.setValue(it.key(), it.value());
            s.sync();
        }
    } settingsRestore{userAll, {}};
    for (const QString &k : userAll.allKeys()) settingsRestore.saved.insert(k, userAll.value(k));
    userAll.remove(QStringLiteral("window"));
    userAll.remove(QStringLiteral("board/estopPin"));  // no board's estop remembered: the checks start from none
    userAll.sync();
    // The ClearPath scope and the user guide are the slow sections and aren't changing just now: ECAM_UI_FULL=1
    // runs them too.
    const bool full = qEnvironmentVariableIsSet("ECAM_UI_FULL");
    if (!full) std::printf("     skipping the scope and the guide (ECAM_UI_FULL=1 runs them)\n");
    TestUploadRoundTrip();
    MainWindow win;
    win.resize(1480, 940);
    win.show();
    // Twice: the diagram's bar may wrap on the first pass, which changes the diagram's height on the next.
    QApplication::processEvents();
    QApplication::processEvents();

    auto *strip = win.findChild<ProfileStrip *>();
    auto *table = win.findChild<QTableWidget *>(QStringLiteral("ECamSegments"));
    auto *undo = win.findChild<QUndoStack *>();
    Check(strip && table && undo, "found strip, table, undo stack");
    if (!strip || !table || !undo) return 1;

    // Default profile: knot 1 at (120 deg, 20). Mirror ProfileStrip's layout to find it on screen.
    const double left = 70, right = 14, top = 10, bottom = 24, gap = 14;
    const double w = strip->width() - left - right;
    // The Outputs band has a lane only for an output with a pulse: the default project has none, so no band.
    const bool outputsBand = false;
    // Lanes 8..14 px, the band at most ~32 % of the height (ProfileStrip::Layout).
    const double laneH = std::clamp(((strip->height() - top - bottom) * 0.32 - 15.0) / 6.0, 8.0, 14.0);
    const double h = strip->height() - top - bottom - (outputsBand ? 15.0 + 6.0 * laneH + gap : 0.0);
    const double dh = std::floor((h - gap) * 0.62);  // velocity + acceleration shown by default
    const double yMin = -2.4, yMax = 22.4;            // 0..20 with 12% padding
    auto toScreen = [&](double deg, double pos) {
        return QPointF(left + deg / 360.0 * w, top + dh - (pos - yMin) / (yMax - yMin) * dh);
    };

    // Locked: knots, splits and the tables can't be edited; scrubbing still works. A new project starts unlocked.
    {
        Check(!win.IsLocked(), "lock: a new project starts unlocked");
        QAction *lockAction = nullptr;
        for (QAction *a : win.findChildren<QAction *>()) {
            if (a->shortcut() == QKeySequence(Qt::CTRL | Qt::Key_L)) lockAction = a;
        }
        Check(lockAction != nullptr, "lock: Edit > Lock profile on Ctrl+L");
        if (lockAction) lockAction->trigger();
        QApplication::processEvents();
        QPushButton *lockButton = win.findChild<QPushButton *>(QStringLiteral("ECamLock"));
        Check(win.IsLocked() && lockButton && lockButton->isChecked() && lockButton->text().contains(QStringLiteral("Locked")),
              "lock: Ctrl+L locks, the bar's lock shows it");
        const double end0 = Cell(table, 0, 3);
        const int rows0 = table->rowCount();
        Mouse(strip, QEvent::MouseButtonPress, toScreen(120, 20), Qt::LeftButton);
        Mouse(strip, QEvent::MouseMove, toScreen(100, 15), Qt::LeftButton);
        Mouse(strip, QEvent::MouseButtonRelease, toScreen(100, 15), Qt::NoButton);
        QMouseEvent dbl(QEvent::MouseButtonDblClick, toScreen(40, 5), strip->mapToGlobal(toScreen(40, 5)), Qt::LeftButton,
                        Qt::LeftButton, Qt::NoModifier);
        QApplication::sendEvent(strip, &dbl);
        QApplication::processEvents();
        Check(Cell(table, 0, 3) == end0 && table->rowCount() == rows0 && undo->count() == 0,
              "lock: knot drag and double-click split do nothing");
        auto *law = qobject_cast<QComboBox *>(table->cellWidget(0, 1));
        Check(table->editTriggers() == QAbstractItemView::NoEditTriggers && law && !law->isEnabled(),
              "lock: the segments table and its laws can't be edited");
        bool splitOff = false;
        for (QPushButton *b : win.findChildren<QPushButton *>()) {
            if (b->text() == QStringLiteral("Split")) splitOff = !b->isEnabled();
        }
        Check(splitOff, "lock: Split is off");
        if (lockButton) lockButton->click();
        QApplication::processEvents();
        law = qobject_cast<QComboBox *>(table->cellWidget(0, 1));
        Check(!win.IsLocked() && lockAction && !lockAction->isChecked() && table->editTriggers() != QAbstractItemView::NoEditTriggers &&
                  law && law->isEnabled(),
              "lock: the bar's lock unlocks; editing is back");
    }

    // Drag knot 1 to about (100 deg, 15).
    Mouse(strip, QEvent::MouseButtonPress, toScreen(120, 20), Qt::LeftButton);
    Mouse(strip, QEvent::MouseMove, toScreen(110, 18), Qt::LeftButton);
    Mouse(strip, QEvent::MouseMove, toScreen(100, 15), Qt::LeftButton);
    Mouse(strip, QEvent::MouseButtonRelease, toScreen(100, 15), Qt::NoButton);
    QApplication::processEvents();
    std::printf("     after drag: seg1 end %.2f deg, %.4f; seg2 %.4f -> %.4f\n", Cell(table, 0, 3), Cell(table, 0, 6),
                Cell(table, 1, 5), Cell(table, 1, 6));
    Check(std::fabs(Cell(table, 0, 3) - 100.0) <= 1.0, "drag moved knot 1 to ~100 deg (1 deg snap)");
    Check(std::fabs(Cell(table, 0, 6) - 15.0) <= 0.3, "drag moved knot 1 to ~15");
    Check(Cell(table, 1, 6) == Cell(table, 0, 6), "dwell after knot 1 followed it");
    Check(undo->count() == 1, "one undo step for the whole drag");

    // Table edit: the return (segment 3) now starts at 15; a rise of -10 ends it at 5. Its end is tied
    // to the start of the profile (closed, through the last dwell), so the start moves to 5 too.
    table->item(2, 7)->setText(QStringLiteral("-10"));
    QApplication::processEvents();
    std::printf("     after rise edit: seg3 end %.4f, start pos %.4f\n", Cell(table, 2, 6), Cell(table, 0, 5));
    Check(std::fabs(Cell(table, 2, 6) - Cell(table, 0, 5)) < 1e-9, "closed: end of return tied to start");
    Check(undo->count() == 2, "table edit is an undo step");

    // Law change through the row's combo (queued).
    auto *combo = qobject_cast<QComboBox *>(table->cellWidget(0, 1));
    Check(combo != nullptr, "law combo present");
    if (combo) {
        combo->setCurrentIndex(6);  // Poly345
        emit combo->activated(6);
        QApplication::processEvents();
        QApplication::processEvents();
        combo = qobject_cast<QComboBox *>(table->cellWidget(0, 1));
        Check(combo && combo->currentText() == QStringLiteral("Polynomial 3-4-5"), "law changed to 3-4-5");
        Check(undo->count() == 3, "law change is an undo step");
    }

    // Undo all, back to the default profile.
    undo->undo();
    undo->undo();
    undo->undo();
    QApplication::processEvents();
    Check(Cell(table, 0, 3) == 120.0 && Cell(table, 0, 6) == 20.0, "undo restores knot 1");
    Check(qobject_cast<QComboBox *>(table->cellWidget(0, 1))->currentText() == QStringLiteral("Cycloidal"),
          "undo restores the law");
    undo->redo();
    QApplication::processEvents();
    Check(std::fabs(Cell(table, 0, 3) - 100.0) <= 1.0, "redo re-applies the drag");

    // Double-click splits.
    const int rows = table->rowCount();
    QMouseEvent dbl(QEvent::MouseButtonDblClick, toScreen(40, 5), strip->mapToGlobal(toScreen(40, 5)), Qt::LeftButton,
                    Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(strip, &dbl);
    QApplication::processEvents();
    Check(table->rowCount() == rows + 1, "double-click split adds a segment");
    Check(Cell(table, 0, 3) == 40.0, "split at the snapped angle");

    // Every page of the sample projects at 1920x1040: none has to scroll. ECAM_UI_TOUR=<dir> also saves each page.
    {
        const QString tour = qEnvironmentVariable("ECAM_UI_TOUR");
        MainWindow t;
        t.resize(1920, 1040);
        t.show();
        auto *tree = t.findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"));
        auto *details = t.findChild<QStackedWidget *>(QStringLiteral("ECamDetails"));
        const char *names[] = {"variable_length_cutoff", "packaging_machine", "bench_gear", "mixed_timeline", "pick_and_place", "flying_cutoff"};
        int pages = 0, scrolling = 0;
        for (const char *name : names) {
            t.OpenFile(QStringLiteral(ECAM_SOURCE_DIR "/samples/%1.ecam").arg(QLatin1String(name)));
            t.resize(1920, 1040);
            QApplication::processEvents();
            QApplication::processEvents();
            std::vector<QTreeWidgetItem *> nodes;
            for (QTreeWidgetItemIterator it(tree); *it; ++it) {
                if ((*it)->data(0, Qt::UserRole).isValid()) nodes.push_back(*it);
            }
            int n = 0;
            for (QTreeWidgetItem *node : nodes) {
                tree->setCurrentItem(node);
                QApplication::processEvents();
                QApplication::processEvents();
                auto *page = qobject_cast<QScrollArea *>(details->currentWidget());
                const bool scrolls = page && page->verticalScrollBar()->isVisible();
                pages++;
                scrolling += scrolls;
                if (scrolls) {
                    std::printf("     scrolls: %s, %s (pane %d x %d, page wants %d x %d)\n", name, node->text(0).toUtf8().constData(),
                                details->width(), details->height(), page->widget()->minimumSizeHint().width(),
                                page->widget()->sizeHint().height());
                }
                if (!tour.isEmpty()) {
                    t.grab(QRect(details->mapTo(&t, QPoint(0, 0)), details->size())).save(QStringLiteral("%1/%2_%3.png").arg(tour, QLatin1String(name)).arg(n++));
                }
            }
            // The review's checks land on the tree a moment after the project changes.
            QElapsedTimer settle;
            settle.start();
            while (settle.elapsed() < 400) QApplication::processEvents(QEventLoop::AllEvents, 20);
            const QString rootTip = tree->topLevelItem(0)->toolTip(0);
            std::printf("     marks %s: %s\n", name, rootTip.isEmpty() ? "none" : rootTip.split(QLatin1Char('\n')).first().toUtf8().constData());
            if (!tour.isEmpty()) t.grab().save(QStringLiteral("%1/%2_window.png").arg(tour, QLatin1String(name)));
        }
        Check(pages > 50 && scrolling == 0, (std::to_string(pages) + " pages at 1920x1040, none scrolls").c_str());
    }

    // The project tree drives the detail pane, and the diagram drives the tree.
    {
        auto *tree = win.findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"));
        auto *details = win.findChild<QStackedWidget *>(QStringLiteral("ECamDetails"));
        Check(tree && details, "found the project tree and the detail pane");
        if (tree && details) {
            auto find = [&](const QString &text) {
                const auto items = tree->findItems(text, Qt::MatchStartsWith | Qt::MatchRecursive);
                return items.isEmpty() ? nullptr : items.first();
            };
            QTreeWidgetItem *crank = find(QStringLiteral("Crank"));
            QTreeWidgetItem *outputs = find(QStringLiteral("Outputs"));
            Check(crank && outputs, "the tree has Crank and Outputs");
            if (crank && outputs) {
                tree->setCurrentItem(crank);
                QApplication::processEvents();
                Check(details->currentIndex() == 0, "Crank opens its page");
                tree->setCurrentItem(outputs);
                QApplication::processEvents();
                Check(details->currentIndex() == 4, "Outputs opens its page");
                Check(strip->isVisible(), "the diagram stays in view whatever the tree shows");
                // The first knot's segment: a click in the diagram opens Segments, and the tree follows.
                const QPointF at = QPointF(left + 0.1 * w, top + 20);
                QMouseEvent press(QEvent::MouseButtonPress, at, strip->mapToGlobal(at), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
                QMouseEvent release(QEvent::MouseButtonRelease, at, strip->mapToGlobal(at), Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
                QApplication::sendEvent(strip, &press);
                QApplication::sendEvent(strip, &release);
                QApplication::processEvents();
                Check(details->currentIndex() == 3 && tree->currentItem() && tree->currentItem()->text(0) == QStringLiteral("Segments"),
                      "a segment clicked in the diagram opens Segments in the tree");
            }
            const QString shot = qEnvironmentVariable("ECAM_UI_SHOT");
            if (!shot.isEmpty()) {
                // Sized for a 1080p monitor, not whatever this PC's saved geometry says: 1080p less the taskbar,
                // the default window, and the smallest it goes.
                tree->setCurrentItem(tree->topLevelItem(0)->child(1)->child(0));
                const QSize sizes[] = {{1920, 1040}, {1480, 940}, {1280, 720}};
                for (const QSize &size : sizes) {
                    win.resize(size);
                    QApplication::processEvents();
                    win.grab().save(QStringLiteral("%1_%2.png").arg(shot).arg(size.width()));
                }
                if (auto *camDock = win.findChild<SideDock *>(QStringLiteral("ECamCamDock"))) {
                    win.resize(1920, 1040);
                    camDock->setCollapsed(true);
                    QApplication::processEvents();
                    QApplication::processEvents();
                    win.grab().save(QStringLiteral("%1_1920_nocam.png").arg(shot));
                    camDock->setCollapsed(false);
                }
                win.resize(1480, 940);
                QApplication::processEvents();
            }
            // The diagram's bar never overlaps itself, from a 1080p screen down to the smallest window.
            {
                const QSize sizes[] = {{1920, 1040}, {1480, 940}, {1280, 720}};
                QGroupBox *diagram = qobject_cast<QGroupBox *>(strip->parentWidget());
                for (const QSize &size : sizes) {
                    win.resize(size);
                    QApplication::processEvents();
                    QApplication::processEvents();
                    bool clear = diagram != nullptr;
                    if (diagram) {
                        std::vector<QRect> boxes;
                        for (QWidget *child : diagram->findChildren<QWidget *>(Qt::FindDirectChildrenOnly)) {
                            if (child != strip && child->isVisible()) boxes.push_back(child->geometry());
                        }
                        for (size_t i = 0; i < boxes.size(); ++i) {
                            for (size_t j = i + 1; j < boxes.size(); ++j) clear = clear && !boxes[i].intersects(boxes[j]);
                        }
                    }
                    // The whole diagram is on screen: it ends above the board strip, at a usable height.
                    auto *boardBar = win.findChild<QWidget *>(QStringLiteral("ECamBoardStrip"));
                    Check(boardBar && strip->height() >= 250 &&
                              strip->mapTo(&win, QPoint(0, strip->height())).y() <= boardBar->mapTo(&win, QPoint(0, 0)).y(),
                          (std::string("the diagram fits above the strip at ") + std::to_string(size.width()) + "x" + std::to_string(size.height())).c_str());
                    Check(clear, (std::string("the diagram's bar doesn't overlap at ") + std::to_string(size.width()) + "x" + std::to_string(size.height())).c_str());
                    // Everything the bar holds is there, at a usable size, and not pushed out of the panel.
                    bool shown = diagram != nullptr;
                    if (diagram) {
                        bool fit = false;
                        for (QPushButton *b : diagram->findChildren<QPushButton *>()) {
                            if (b->text() != QStringLiteral("Fit")) continue;
                            const QRect at(b->mapTo(diagram, QPoint(0, 0)), b->size());
                            fit = b->isVisible() && b->width() >= 24 && diagram->rect().contains(at);
                        }
                        shown = fit;
                        auto *dro = diagram->findChild<QLabel *>(QStringLiteral("ECamReadout"));
                        shown = shown && dro && dro->isVisible() && dro->width() > 300 &&
                                diagram->rect().contains(QRect(dro->mapTo(diagram, QPoint(0, 0)), dro->size())) &&
                                dro->mapTo(diagram, QPoint(0, 0)).y() < strip->y();
                        // ... and clear of what shares its row.
                        if (dro) {
                            for (QWidget *sib : dro->parentWidget()->findChildren<QWidget *>(Qt::FindDirectChildrenOnly)) {
                                if (sib != dro && sib->isVisible()) shown = shown && !sib->geometry().intersects(dro->geometry());
                            }
                            // A 1080p screen keeps the bar on one row, with the diagram getting the height.
                            if (size.width() == 1920) {
                                Check(dro->mapTo(diagram, QPoint(0, 0)).y() < 60, "at 1920x1040 the DRO shares the bar's one row");
                            }
                        }
                        for (QDoubleSpinBox *sb : diagram->findChildren<QDoubleSpinBox *>()) {
                            const QRect at(sb->mapTo(diagram, QPoint(0, 0)), sb->size());
                            shown = shown && sb->isVisible() && sb->width() >= 60 && diagram->rect().contains(at);
                        }
                    }
                    Check(shown, (std::string("the DRO, Fit and Snap are in view above the diagram at ") + std::to_string(size.width()) + "x" + std::to_string(size.height())).c_str());
                }
                win.resize(1480, 940);
                QApplication::processEvents();
            }
        }
    }

    // The Cam folds away, giving the diagram its width, and comes back.
    {
        auto *camDock = win.findChild<SideDock *>(QStringLiteral("ECamCamDock"));
        Check(camDock != nullptr, "cam dock present");
        if (camDock) {
            camDock->setCollapsed(false);
            QApplication::processEvents();
            auto *pane = win.findChild<QStackedWidget *>(QStringLiteral("ECamDetails"));
            const int open = camDock->width(), diagram = pane->width();
            camDock->setCollapsed(true);
            QApplication::processEvents();
            QApplication::processEvents();
            std::printf("     cam dock: %d px open, %d px collapsed; detail pane %d -> %d px\n", open, camDock->width(), diagram, pane->width());
            Check(camDock->width() == SideDock::kStripWidth, "the Cam collapses to its strip");
            Check(pane->width() > diagram + open / 2, "the detail pane takes the Cam's width");
            camDock->setCollapsed(false);
            QApplication::processEvents();
            QApplication::processEvents();
            Check(std::abs(camDock->width() - open) <= 2 && std::abs(pane->width() - diagram) <= 2, "the Cam reopens at its width");
        }
    }

    // A short window squeezes the top row, and it comes back when there's room.
    {
        auto *pane = win.findChild<QStackedWidget *>(QStringLiteral("ECamDetails"));
        win.resize(1920, 1040);
        QApplication::processEvents();
        QApplication::processEvents();
        const int tall = pane->height();
        win.resize(1280, 720);
        QApplication::processEvents();
        QApplication::processEvents();
        const int squeezed = pane->height();
        win.resize(1920, 1040);
        QApplication::processEvents();
        QApplication::processEvents();
        std::printf("     top row: %d px at 1080p, %d at the smallest, %d after\n", tall, squeezed, pane->height());
        Check(squeezed < tall && pane->height() == tall, "the top row gives way on a short window and comes back");
        win.resize(1480, 940);
        QApplication::processEvents();
        QApplication::processEvents();
    }

    // The Project dock: the tree over the status (estop, each motor), at every window size.
    {
        auto *split = win.findChild<QSplitter *>(QStringLiteral("ECamProjectSplit"));
        auto *tree = win.findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"));
        auto *status = win.findChild<QFrame *>(QStringLiteral("ECamMachineStatus"));
        Check(split && tree && status && split->widget(0) == tree && split->widget(1) == status, "status: the Project dock is the tree over the status");
        auto *board = win.findChild<QLabel *>(QStringLiteral("ECamStatusBoard"));
        auto *estop = win.findChild<QLabel *>(QStringLiteral("ECamStatusEstop"));
        auto *m0 = win.findChild<QLabel *>(QStringLiteral("ECamStatusMotor0"));
        auto *m3 = win.findChild<QLabel *>(QStringLiteral("ECamStatusMotor3"));
        Check(board && board->text() == QStringLiteral("Not connected") && estop && estop->text() == QStringLiteral("—") && m0 &&
                  m0->text() == QStringLiteral("—") && m3,
              "status: no board, nothing claimed");
        for (const QSize size : {QSize(1920, 1040), QSize(1480, 940), QSize(1280, 720)}) {
            win.resize(size);
            QApplication::processEvents();
            QApplication::processEvents();
            const bool whole = status && status->height() >= status->sizeHint().height() - 2;
            std::printf("     %dx%d: tree %d px, status %d px (wants %d)\n", size.width(), size.height(), tree ? tree->height() : 0,
                        status ? status->height() : 0, status ? status->sizeHint().height() : 0);
            Check(tree && tree->height() >= 150 && whole,
                  QStringLiteral("status: at %1x%2 the status shows whole and the tree keeps room").arg(size.width()).arg(size.height()).toUtf8().constData());
        }
        win.resize(1480, 940);
        QApplication::processEvents();
    }

    // The Board page's network settings: what is checked before anything goes to a board.
    {
        net::Settings good;
        good.ip = QStringLiteral("10.168.41.5");
        good.netmask = QStringLiteral("255.255.255.0");
        good.gateway = QStringLiteral("0.0.0.0");
        Check(net::Check(good).isEmpty(), "net: a static address with no gateway is fine");
        auto bad = [&](const char *key, const char *value) {
            net::Settings s = good;
            const QString v = QLatin1String(value);
            if (!strcmp(key, "ip")) s.ip = v;
            if (!strcmp(key, "mask")) s.netmask = v;
            if (!strcmp(key, "gw")) s.gateway = v;
            if (!strcmp(key, "host")) s.host = v;
            return !net::Check(s).isEmpty();
        };
        Check(bad("ip", "10.168.41") && bad("ip", "10.168.41.256") && bad("ip", "a.b.c.d") && bad("ip", ""), "net: a malformed address is refused");
        Check(bad("ip", "0.0.0.1") && bad("ip", "127.0.0.1") && bad("ip", "224.0.0.1"), "net: an address a device can't use is refused");
        Check(bad("ip", "10.168.41.0") && bad("ip", "10.168.41.255"), "net: the network and broadcast addresses are refused");
        Check(bad("mask", "255.0.255.0") && bad("mask", "0.0.0.0") && bad("mask", "255.255.255.255"), "net: a mask with holes is refused");
        Check(bad("gw", "10.168.42.1") && bad("gw", "10.168.41.5") && !bad("gw", "10.168.41.254"), "net: the gateway must be on the subnet");
        Check(bad("host", "has space") && bad("host", "under_score") && !bad("host", "ecam-1.line2") && !bad("host", ""), "net: host name characters");
        const QStringList config = {QStringLiteral("estop_a9 0"), QStringLiteral("net_mode 1"), QStringLiteral("ip 10.168.41.5"), QStringLiteral("netmask 255.255.255.0"),
                                    QStringLiteral("gateway 0.0.0.0"), QStringLiteral("host - (default)"), QStringLiteral("ok")};
        const net::Settings read = net::Parse(config);
        Check(read.valid && !read.dhcp && read.ip == good.ip && read.gateway == good.gateway && read.host.isEmpty(), "net: reads config");
        Check(!net::Parse({QStringLiteral("ok")}).valid, "net: a reply without the settings isn't taken for them");
        Check(net::InUse({QStringLiteral("eip network stored=dhcp (no DHCP server: on the stored address) ip=10.168.41.5 (stored 10.168.41.5/255.255.255.0 gw 0.0.0.0)")}) ==
                  QStringLiteral("10.168.41.5"), "net: reads the address in use");
        net::Settings to = read;
        Check(net::Commands(read, to).isEmpty(), "net: nothing changed, nothing sent");
        to.ip = QStringLiteral("10.168.41.6");
        to.host = QStringLiteral("line2");
        Check(net::Commands(read, to) == QStringList({QStringLiteral("set ip 10.168.41.6"), QStringLiteral("set host line2")}), "net: only what changed is sent");
        to = read;
        to.dhcp = true;
        Check(net::Commands(read, to) == QStringList({QStringLiteral("set net_mode 0")}), "net: DHCP is net_mode 0");
        net::Settings cleared = to;
        cleared.host = QStringLiteral("x");
        Check(net::Commands(cleared, to) == QStringList({QStringLiteral("set host -")}), "net: an empty host name goes back to the default");
        auto *apply = win.findChild<QPushButton *>(QStringLiteral("ECamNetApply"));
        auto *ip = win.findChild<QLineEdit *>(QStringLiteral("ECamNetIp"));
        Check(apply && ip && !apply->isEnabled() && !ip->isEnabled(), "net: with no board nothing can be changed");
    }

    // Problems show on the tree, on the node they belong to, and go when they're fixed.
    {
        MainWindow m;
        m.resize(1920, 1040);
        m.show();
        auto *tree = m.findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"));
        auto settle = [] {
            QElapsedTimer et;
            et.start();
            while (et.elapsed() < 400) QApplication::processEvents(QEventLoop::AllEvents, 20);
        };
        auto mark = [](QTreeWidgetItem *it) { return it->data(0, Qt::UserRole + 2).toInt(); };
        QTreeWidgetItem *root = tree->topLevelItem(0), *axes = root->child(1);

        // An axis's error: on that axis, its heading and the project, and not on its neighbour.
        m.OpenFile(QStringLiteral(ECAM_SOURCE_DIR "/samples/mixed_timeline.ecam"));
        settle();
        int errorAxis = -1, cleanAxis = -1;
        for (int i = 0; i < axes->childCount(); ++i) {
            if (mark(axes->child(i)) == 2) errorAxis = i;
            if (mark(axes->child(i)) == 0) cleanAxis = i;
        }
        Check(errorAxis >= 0 && cleanAxis >= 0, "marks: an axis over its limit is marked, its neighbour isn't");
        if (errorAxis >= 0) {
            Check(axes->child(errorAxis)->toolTip(0).contains(QStringLiteral("Error:")), "marks: the node's tooltip says why");
            Check(mark(axes) == 2 && mark(root) == 2, "marks: the heading and the project show the worst under them");
            Check(root->toolTip(0).startsWith(QStringLiteral("2 errors")), "marks: the project counts them");
        }

        // A project's error: two axes on one motor, marked on the project, and gone when it's fixed.
        m.OpenFile(QStringLiteral(ECAM_SOURCE_DIR "/samples/pick_and_place.ecam"));
        settle();
        Check(mark(root) == 0 && root->toolTip(0).isEmpty(), "marks: a clean project has none");
        tree->setCurrentItem(axes->child(0));
        settle();
        QComboBox *motor = nullptr;
        for (QComboBox *cb : m.findChildren<QComboBox *>()) {
            if (cb->isVisible() && cb->count() >= 4 && cb->itemText(0) == QStringLiteral("M-0")) motor = cb;
        }
        Check(motor != nullptr, "marks: found the axis's motor");
        if (motor) {
            const int first = motor->currentIndex();
            tree->setCurrentItem(axes->child(1));
            settle();
            const int was = motor->currentIndex();
            motor->setCurrentIndex(first);
            settle();
            Check(mark(root) == 2, "marks: two axes on one motor is an error on the project");
            std::printf("     clash: %s\n", root->toolTip(0).replace(QLatin1Char('\n'), QStringLiteral(" | ")).toUtf8().constData());
            motor->setCurrentIndex(was);
            settle();
            Check(mark(root) == 0, "marks: fixing it clears the mark");
        }
        m.findChild<QUndoStack *>()->setClean();
    }

    // Every panel is a page now: its title doesn't fold it, and the machine's buttons are always in view.
    {
        CollapsibleGroup *board = nullptr;
        for (CollapsibleGroup *g : win.findChildren<CollapsibleGroup *>()) {
            if (g->title().contains(QStringLiteral("Board"))) board = g;
        }
        Check(board != nullptr && win.findChildren<CollapsibleGroup *>().size() == 10, "ten panels (Waits, 0.10.0)");
        auto *tree = win.findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"));
        auto *details = win.findChild<QStackedWidget *>(QStringLiteral("ECamDetails"));
        auto *stripBar = win.findChild<QWidget *>(QStringLiteral("ECamBoardStrip"));
        Check(tree && details && stripBar, "found the board strip");
        if (board && tree && details && stripBar) {
            const auto items = tree->findItems(QStringLiteral("Board"), Qt::MatchExactly | Qt::MatchRecursive);
            Check(items.size() == 1, "the tree has Board");
            if (!items.isEmpty()) {
                tree->setCurrentItem(items.first());
                QApplication::processEvents();
                Check(details->currentIndex() == 6 && board->isVisible(), "Board opens its page");
            }
            QMouseEvent press(QEvent::MouseButtonPress, QPointF(40, 8), board->mapToGlobal(QPointF(40, 8)),
                              Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
            QApplication::sendEvent(board, &press);
            QApplication::processEvents();
            Check(!board->isCollapsed() && board->body()->isVisible(), "a page's title doesn't fold it");
            // Whatever page is open, the run buttons stay in view.
            tree->setCurrentItem(tree->topLevelItem(0)->child(0)->child(0));
            QApplication::processEvents();
            for (const char *name : {"Connect", "Upload", "Enable", "Disable", "Start", "Stop", "Abort"}) {
                bool seen = false;
                for (QPushButton *b : stripBar->findChildren<QPushButton *>()) {
                    seen = seen || (b->text() == QLatin1String(name) && b->isVisible());
                }
                Check(seen, (std::string("the strip keeps ") + name + " in view").c_str());
            }
            // Squeezed (a narrow window, the cam view dragged wide): every button and check box keeps its label readable.
            if (!items.isEmpty()) {
                tree->setCurrentItem(items.first());
                QApplication::processEvents();
            }
            auto *split = win.findChild<QSplitter *>(QStringLiteral("ECamDiagramSplit"));
            Check(split != nullptr, "found the top row's split");
            const QByteArray splitWas = split ? split->saveState() : QByteArray();
            for (const QSize size : {QSize(1920, 1040), QSize(1280, 720)}) {
                win.resize(size);
                QApplication::processEvents();
                for (int pane : {1100, 800, 600, 420}) {
                    if (!split) break;
                    const int total = split->width() - split->handleWidth();
                    split->setSizes({pane, std::max(0, total - pane)});
                    QApplication::processEvents();
                    QApplication::processEvents();
                    QStringList unreadable;
                    QList<QAbstractButton *> buttons = board->findChildren<QAbstractButton *>();
                    buttons += stripBar->findChildren<QAbstractButton *>();
                    for (QAbstractButton *b : buttons) {
                        if (!b->isVisible() || b->text().isEmpty()) continue;
                        const int need = b->fontMetrics().horizontalAdvance(b->text()) + (qobject_cast<QCheckBox *>(b) ? 18 : 10);
                        if (b->width() < need) unreadable << QStringLiteral("%1 (%2 of %3)").arg(b->text()).arg(b->width()).arg(need);
                    }
                    const QString what = QStringLiteral("board page at %1x%2, pane %3 px: labels readable %4")
                                             .arg(size.width()).arg(size.height()).arg(details->width()).arg(unreadable.join(QStringLiteral(", ")));
                    Check(unreadable.isEmpty(), what.toUtf8().constData());
                    if (!unreadable.isEmpty()) win.grab().save(QDir::temp().filePath(QStringLiteral("ecam_board_squeezed.png")));
                }
            }
            win.resize(1480, 940);
            if (split) split->restoreState(splitWas);
            QApplication::processEvents();
        }
    }

    // Enter in a setting must not fold its panel (the key bubbles up from the spin box).
    {
        CollapsibleGroup *crank = nullptr;
        for (CollapsibleGroup *g : win.findChildren<CollapsibleGroup *>()) {
            if (g->title().contains(QStringLiteral("Crank"))) crank = g;
        }
        QDoubleSpinBox *spin = crank ? crank->body()->findChild<QDoubleSpinBox *>() : nullptr;
        if (crank && spin) {
            crank->setCollapsed(false);
            spin->setFocus();
            QApplication::processEvents();
            QKeyEvent enter(QEvent::KeyPress, Qt::Key_Return, Qt::NoModifier);
            QApplication::sendEvent(spin, &enter);
            QApplication::processEvents();
            Check(!crank->isCollapsed(), "Enter in a setting leaves its panel open");
        } else {
            Check(false, "found a setting in the Crank panel");
        }
    }

    // The settings column collapses to a strip like a docked tool window, and opens at its width.
    {
        SideDock *dock = win.findChild<SideDock *>(QStringLiteral("ECamProjectDock"));
        Check(dock != nullptr, "settings dock present");
        if (dock) {
            dock->setCollapsed(false);
            QApplication::processEvents();
            const int open = dock->width();
            dock->setCollapsed(true);
            QApplication::processEvents();
            const int closed = dock->width();
            dock->setCollapsed(false);
            QApplication::processEvents();
            std::printf("     settings dock: %d px open, %d px collapsed, %d px reopened\n", open, closed, dock->width());
            Check(closed == SideDock::kStripWidth, "dock collapses to its strip");
            Check(std::abs(dock->width() - open) <= 2, "dock reopens at its width");
        }
    }

    // Outputs in the displacement diagram (crank axis of the default project). Only outputs with a pulse have a lane:
    // none at first, so no band. Add one from the Outputs page, move it to IO-3 in the table (its lane follows), then in
    // IO-3's lane: double-click to add a second, drag it, drag its on end, undo. The Outputs table shows the result.
    {
        QTableWidget *outTable = nullptr;
        for (QTableWidget *tw : win.findChildren<QTableWidget *>()) {
            if (tw->horizontalHeaderItem(0) && tw->horizontalHeaderItem(0)->text() == QStringLiteral("Output")) outTable = tw;
        }
        QPushButton *addPulse = nullptr;
        for (CollapsibleGroup *g : win.findChildren<CollapsibleGroup *>()) {
            if (!g->title().startsWith(QStringLiteral("Outputs"))) continue;
            for (QPushButton *pb : g->findChildren<QPushButton *>()) {
                if (pb->text() == QStringLiteral("Add")) addPulse = pb;
            }
        }
        Check(outTable != nullptr && addPulse != nullptr, "Outputs table and its Add");
        if (outTable && addPulse) {
            auto cell = [&](int r, int c) {
                if (!outTable->item(r, c)) return -1.0;
                QString s = outTable->item(r, c)->text();
                s.remove(QChar(0xB0));
                return s.trimmed().toDouble();
            };
            auto combo = [&](int r, int c) {
                auto *cb = qobject_cast<QComboBox *>(outTable->cellWidget(r, c));
                return cb ? cb->currentText() : QString();
            };
            // Mirror ProfileStrip's layout: plot x from 70 to w - 14; the Outputs band (15 + lanes x lane height) above
            // the 24 px label row at the bottom. One lane: 14 px.
            const double pl = 70, pw = strip->width() - 84.0;
            const double oneLaneTop = strip->height() - 24.0 - (15.0 + 14.0);
            const double laneY = oneLaneTop + 15.0 + 7.0;
            auto xAt = [&](double deg) { return pl + deg / 360.0 * pw; };
            const int undo0 = undo->count();
            addPulse->click();
            QApplication::processEvents();
            Check(outTable->rowCount() == 1 && combo(0, 0) == QStringLiteral("IO-0"), "Add: a pulse on the first output, IO-0");
            auto *oc = qobject_cast<QComboBox *>(outTable->cellWidget(0, 0));
            if (oc) oc->setCurrentIndex(3);
            QApplication::processEvents();
            QApplication::processEvents();
            Check(combo(0, 0) == QStringLiteral("IO-3"), "the table moves it to IO-3");
            QMouseEvent dbl(QEvent::MouseButtonDblClick, QPointF(xAt(180), laneY), strip->mapToGlobal(QPointF(xAt(180), laneY)),
                            Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
            QApplication::sendEvent(strip, &dbl);
            QApplication::processEvents();
            int row = -1;
            for (int r = 0; r < outTable->rowCount(); ++r) {
                if (Near(cell(r, 2), 180.0, 1e-9)) row = r;
            }
            Check(outTable->rowCount() == 2 && row >= 0 && combo(row, 0) == QStringLiteral("IO-3") &&
                      combo(row, 1) == QStringLiteral("Crank") && Near(cell(row, 3), 190.0, 1e-9),
                  "the band has only IO-3's lane: double-click it at 180 deg adds a 10 deg crank pulse there");
            Mouse(strip, QEvent::MouseButtonPress, QPointF(xAt(185), laneY), Qt::LeftButton);
            Mouse(strip, QEvent::MouseMove, QPointF(xAt(205), laneY), Qt::LeftButton);
            Mouse(strip, QEvent::MouseButtonRelease, QPointF(xAt(205), laneY), Qt::NoButton);
            QApplication::processEvents();
            row = -1;
            for (int r = 0; r < outTable->rowCount(); ++r) {
                if (Near(cell(r, 2), 200.0, 1e-9)) row = r;
            }
            Check(row >= 0 && combo(row, 0) == QStringLiteral("IO-3") && Near(cell(row, 3), 210.0, 1e-9),
                  "drag moves the pulse 20 deg later");
            Mouse(strip, QEvent::MouseButtonPress, QPointF(xAt(200) + 1, laneY), Qt::LeftButton);
            Mouse(strip, QEvent::MouseMove, QPointF(xAt(190), laneY), Qt::LeftButton);
            Mouse(strip, QEvent::MouseButtonRelease, QPointF(xAt(190), laneY), Qt::NoButton);
            QApplication::processEvents();
            row = -1;
            for (int r = 0; r < outTable->rowCount(); ++r) {
                if (Near(cell(r, 3), 210.0, 1e-9)) row = r;
            }
            Check(row >= 0 && Near(cell(row, 2), 190.0, 1e-9), "drag the on end to 190 deg");
            undo->undo();
            QApplication::processEvents();
            bool back = false;
            for (int r = 0; r < outTable->rowCount(); ++r) back = back || Near(cell(r, 2), 200.0, 1e-9);
            Check(back, "undo: one drag is one step");
            while (undo->count() > undo0 && undo->canUndo()) undo->undo();
            QApplication::processEvents();
            Check(outTable->rowCount() == 0, "undo back to no outputs");
        }
    }

    // The timeline's trigger input and the outputs: the trigger list greys the outputs, the outputs' list greys the
    // trigger input, and the review flags a project that has both on one pin.
    {
        ecam::CamProject tp = ecam::DefaultProject();
        ecam::SetAxisMaster(tp, 0, ecam::AxisMaster::Timeline);
        tp.master.trigger = ecam::TriggerSource::DigitalInput;
        tp.master.triggerInput = 4;
        ecam::OutputPulse o;
        o.output = 1;
        o.master = ecam::AxisMaster::Timeline;
        o.on = 0.1;
        o.off = 0.3;
        tp.outputs.push_back(o);
        ecam::NormalizeProject(tp);
        bool clash = false;
        for (const auto &c : review::AnalyzeProject(tp).checks) clash = clash || c.text.contains(QStringLiteral("also an output"));
        Check(!clash, "review: trigger IO-4 and an output on IO-1 don't clash");
        tp.master.triggerInput = 1;
        bool flagged = false;
        for (const auto &c : review::AnalyzeProject(tp).checks) {
            flagged = flagged || (c.level == review::Check::Error && c.text.contains(QStringLiteral("IO-1 is also an output")));
        }
        Check(flagged, "review: the trigger input on an output's pin is an error");
    }
    // Inputs: the board's own inputs, as the CCIO-8 ones, with the pins something else has taken kept from them.
    {
        // The model: blocked pins, the encoder, the review, a save and a load.
        ecam::CamProject p = ecam::DefaultProject();  // an encoder master with a crank axis
        Check(ecam::BoardInputBlocked(p, 7) == "encoder" && ecam::BoardInputBlocked(p, 3).empty(), "inputs: DI-6..DI-8 are the encoder's");
        ecam::BoardInput home;
        home.pin = 3;
        home.name = "X home";
        home.role = ecam::CcioRole::Home;
        home.axis = 0;
        ecam::SetBoardInput(p, home);
        Check(p.inputs.size() == 1 && ecam::BoardInputAt(p, 3).name == "X home", "inputs: set one");
        ecam::OutputPulse o;
        o.output = 3;
        o.master = ecam::AxisMaster::Crank;
        o.on = 10;
        o.off = 20;
        ecam::CamProject clash = p;
        clash.outputs.push_back(o);
        bool flagged = false;
        for (const auto &c : review::AnalyzeProject(clash).checks) {
            flagged = flagged || (c.level == review::Check::Error && c.where == QStringLiteral("Inputs") && c.text.contains(QStringLiteral("IO-3")));
        }
        Check(flagged, "inputs: review: an input on an output's pin is an error");
        ecam::CamProject two = p;
        two.ccio.boards = 1;
        ecam::CcioInput ch;
        ch.board = 0;
        ch.pin = 2;
        ch.role = ecam::CcioRole::Home;
        ch.axis = 0;
        ecam::SetCcioInput(two.ccio, ch);
        flagged = false;
        for (const auto &c : review::AnalyzeProject(two).checks) {
            flagged = flagged || (c.level == review::Check::Error && c.text.contains(QStringLiteral("IO-3")) && c.text.contains(QStringLiteral("CCIOA2")));
        }
        Check(flagged, "inputs: review: one home on the board's input and a CCIO-8's is an error");
        ecam::CamProject enc = p;
        ecam::BoardInput di;
        di.pin = 6;
        di.name = "spare";
        ecam::SetBoardInput(enc, di);
        flagged = false;
        for (const auto &c : review::AnalyzeProject(enc).checks) flagged = flagged || c.text.contains(QStringLiteral("DI-6"));
        Check(flagged && ecam::BoardInputsOnEncoderPins(enc), "inputs: review: DI-6 with an encoder master is an error");
        QString err;
        ecam::CamProject back;
        Check(io::ProjectFromJson(io::ProjectToJson(p, false), &back, &err) && back.inputs.size() == 1 && back.inputs[0].pin == 3 &&
                  back.inputs[0].name == "X home" && back.inputs[0].role == ecam::CcioRole::Home && back.inputs[0].axis == 0,
              "inputs: saved and loaded");
        Check(!io::ProjectToJson(ecam::DefaultProject(), false).contains("\"inputs\""), "inputs: a project without them saves as before");

        // The page.
        MainWindow t;
        t.resize(1920, 1040);
        t.show();
        QApplication::processEvents();
        auto *tree = t.findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"));
        auto *details = t.findChild<QStackedWidget *>(QStringLiteral("ECamDetails"));
        auto *table = t.findChild<QTableWidget *>(QStringLiteral("ECamInputsTable"));
        const auto items = tree ? tree->findItems(QStringLiteral("Inputs"), Qt::MatchStartsWith | Qt::MatchRecursive) : QList<QTreeWidgetItem *>();
        Check(table && table->rowCount() == 13 && items.size() == 1, "inputs: the tree's Inputs, thirteen inputs");
        if (table && !items.isEmpty()) {
            tree->setCurrentItem(items.first());
            QApplication::processEvents();
            Check(details->currentIndex() == 8 && table->isVisible(), "inputs: Inputs opens its page");
            auto *role6 = qobject_cast<QComboBox *>(table->cellWidget(6, 3));
            Check(role6 && !role6->isEnabled() && table->item(6, 2)->text() == QStringLiteral("Encoder Adapter"),
                  "inputs: with an encoder master DI-6 can't be set (Encoder Adapter)");
            auto *undo = t.findChild<QUndoStack *>();
            const int steps = undo ? undo->count() : -1;
            auto *role2 = qobject_cast<QComboBox *>(table->cellWidget(2, 3));
            if (role2) role2->setCurrentIndex(1);  // Home
            QApplication::processEvents();
            table->item(2, 2)->setText(QStringLiteral("X home"));
            QApplication::processEvents();
            auto *axis2 = qobject_cast<QComboBox *>(table->cellWidget(2, 4));
            Check(axis2 && axis2->isEnabled() && axis2->currentIndex() == 0 && undo && undo->count() == steps + 2,
                  "inputs: IO-2 a home of the axis, named, two undo steps");
            Check(items.first()->text(0) == QStringLiteral("Inputs  (1)"), "inputs: the tree counts it");
            // The crank can't take the encoder while DI-6..DI-8 are inputs: driven, set DI-7, and Encoder is off.
            QComboBox *source = nullptr;
            for (QComboBox *c : t.findChildren<QComboBox *>()) {
                if (c->count() == 2 && c->itemText(0) == QStringLiteral("External encoder")) source = c;
            }
            Check(source != nullptr, "inputs: the crank's source");
            if (source) {
                source->setCurrentIndex(1);
                QApplication::processEvents();
                auto *role7 = qobject_cast<QComboBox *>(table->cellWidget(7, 3));
                Check(role7 && role7->isEnabled(), "inputs: a driven crank frees DI-7");
                table->item(7, 2)->setText(QStringLiteral("guard"));
                QApplication::processEvents();
                auto *model = qobject_cast<QStandardItemModel *>(source->model());
                Check(model && !(model->item(0)->flags() & Qt::ItemIsEnabled), "inputs: DI-7 set: the crank can't pick the encoder");
                table->item(7, 2)->setText(QString());
                QApplication::processEvents();
                Check(model && (model->item(0)->flags() & Qt::ItemIsEnabled), "inputs: DI-7 cleared: it can again");
            }
            // An input's pin is not offered as an output.
            QPushButton *addPulse = nullptr;
            for (CollapsibleGroup *g : t.findChildren<CollapsibleGroup *>()) {
                if (!g->title().startsWith(QStringLiteral("Outputs"))) continue;
                for (QPushButton *pb : g->findChildren<QPushButton *>()) {
                    if (pb->text() == QStringLiteral("Add")) addPulse = pb;
                }
            }
            for (int k = 0; k < 3 && addPulse; ++k) addPulse->click();
            QApplication::processEvents();
            QTableWidget *outTable = nullptr;
            for (QTableWidget *tw : t.findChildren<QTableWidget *>()) {
                if (tw->horizontalHeaderItem(0) && tw->horizontalHeaderItem(0)->text() == QStringLiteral("Output")) outTable = tw;
            }
            QStringList outs;
            for (int r = 0; outTable && r < outTable->rowCount(); ++r) {
                if (auto *cb = qobject_cast<QComboBox *>(outTable->cellWidget(r, 0))) outs << cb->currentText();
            }
            Check(outs.size() == 3 && !outs.contains(QStringLiteral("IO-2")), ("inputs: Add skips IO-2: " + outs.join(QStringLiteral(" ")).toStdString()).c_str());
            if (outTable && outTable->rowCount() > 0) {
                auto *cb = qobject_cast<QComboBox *>(outTable->cellWidget(0, 0));
                auto *model = cb ? qobject_cast<QStandardItemModel *>(cb->model()) : nullptr;
                Check(model && !(model->item(2)->flags() & Qt::ItemIsEnabled), "inputs: the Output column greys IO-2");
            }
        }
    }
    // A command's callback that sends another command keeps its reply: the next command starts at once and clears the
    // link's reply buffer (the switch wizard sent "axjog <m> stop" while reading an "axjog" reply: heap corruption).
    {
        QTcpServer server;
        if (!server.listen(QHostAddress::LocalHost, BoardLink::kConsolePort)) {
            Check(false, "fake board: listen on 127.0.0.1:9500");
        } else {
            QObject::connect(&server, &QTcpServer::newConnection, &server, [&server]() {
                QTcpSocket *s = server.nextPendingConnection();
                QObject::connect(s, &QTcpSocket::readyRead, s, [s]() {
                    while (s->canReadLine()) {
                        const QByteArray l = s->readLine().trimmed();
                        if (l.isEmpty()) continue;
                        if (l == "status") {
                            s->write("state=disabled enabled=0 sim=0 fault=none estop=ok a9=1 profile=0 stored=0 axes=2 crank=off tl=idle "
                                     "mon=1 lim=0 hashome=3 homed=1 homing=2 homesw=2 wait=2 waited=3500 ops=11 oppins=3,4,-1,10,11 opnc=12 opmode=1\r\nok\r\n");
                        } else {
                            s->write("line A\r\nline B\r\nok\r\n");
                        }
                    }
                });
            });
            BoardLink link;
            QString err;
            const bool up = link.connectEthernet(QStringLiteral("127.0.0.1"), &err);
            Check(up, "fake board: connected");
            int seen = -1;
            QStringList kept;
            QEventLoop loop;
            link.command(QStringLiteral("first"), [&](bool, const QStringList &r) {
                link.command(QStringLiteral("second"), [&](bool, const QStringList &) { loop.quit(); });
                seen = r.size();
                for (const QString &l : r) kept << l;
            });
            QTimer::singleShot(3000, &loop, &QEventLoop::quit);
            if (up) loop.exec();
            Check(seen == 3 && kept.value(0) == QStringLiteral("line A") && kept.value(2) == QStringLiteral("ok"),
                  "board link: a reply survives its callback sending the next command");
            QEventLoop wait;
            link.command(QStringLiteral("status"), [&](bool, const QStringList &) { wait.quit(); });
            QTimer::singleShot(3000, &wait, &QEventLoop::quit);
            if (up) wait.exec();
            const BoardLink::State &st = link.state();
            Check(st.hasHome == 3 && st.homed == 1 && st.homing == 2 && st.homeSwitch == 2, "board link: the homing tokens (hashome, homed, homing, homesw)");
            Check(st.waitIndex == 2 && st.waitedMs == 3500, "board link: the wait tokens (wait, waited)");
            Check(st.opKnown && st.opPins[0] == 3 && st.opPins[2] == -1 && st.opPins[4] == 11 && st.opPressed == 0x11 && st.opNc == 0x12 &&
                      st.opMaintained,
                  "board link: the operator tokens (ops, oppins, opnc, opmode)");
            link.disconnectLink();
        }
    }
    // Start refused off the cam positions: the popup lists every axis off its place (from the board's axjog lines),
    // and "Move to cam positions" moves them one at a time, each once the one before has stopped (a fake board).
    {
        QTcpServer server;
        QStringList got;   // the commands the board was sent
        int moving0 = 0, moving1 = 0;  // polls left that still say "moving"
        double off0 = 1250, off1 = -300;  // counts off (cpu 100: 12.5 mm, -3 mm)
        if (server.listen(QHostAddress::LocalHost, BoardLink::kConsolePort)) {
            QObject::connect(&server, &QTcpServer::newConnection, &server, [&]() {
                QTcpSocket *s = server.nextPendingConnection();
                QObject::connect(s, &QTcpSocket::readyRead, s, [&, s]() {
                    while (s->canReadLine()) {
                        const QString l = QString::fromLatin1(s->readLine()).trimmed();
                        if (l.isEmpty()) continue;
                        got << l;
                        if (l == "status") {
                            s->write("state=disabled enabled=0 sim=1 fault=none estop=ok profile=1a2b3c4d stored=1 axes=2 crank=off tl=idle mon=0\r\nok\r\n");
                        } else if (l == "start") {
                            s->write("error: X travel (M-0) is 12.5 mm off its cam position: move it there (axjog 0 cam) or zero it\r\n");
                        } else if (l == "axjog") {
                            const QString line0 = QStringLiteral("axjog 0 role=axis axis=0 on=0 moving=%1 pos=%2 cam=0 off=%2 cpu=100 vmax=500 lim=0 stopped=- at=0 units=mm name=X travel\r\n")
                                                      .arg(moving0 > 0 ? 1 : 0).arg(static_cast<qint64>(off0));
                            const QString line1 = QStringLiteral("axjog 1 role=axis axis=1 on=0 moving=%1 pos=%2 cam=0 off=%2 cpu=100 vmax=150 lim=0 stopped=- at=0 units=mm name=Z lift\r\n")
                                                      .arg(moving1 > 0 ? 1 : 0).arg(static_cast<qint64>(off1));
                            s->write((line0 + line1 + QStringLiteral("axjog 2 role=free axis=- on=0 moving=0 pos=0 cam=- off=0 cpu=1 vmax=53333 lim=0 stopped=- at=0 units=counts name=\r\nok\r\n")).toLatin1());
                            if (moving0 > 0 && --moving0 == 0) off0 = 0;
                            if (moving1 > 0 && --moving1 == 0) off1 = 0;
                        } else if (l.startsWith("axjog 0 cam")) {
                            moving0 = 2;
                            s->write("ok\r\n");
                        } else if (l.startsWith("axjog 1 cam")) {
                            moving1 = 2;
                            s->write("ok\r\n");
                        } else {
                            s->write("ok\r\n");
                        }
                    }
                });
            });
            MainWindow w;
            w.resize(1600, 900);
            w.show();
            w.ConnectBoard(QStringLiteral("127.0.0.1"));
            QElapsedTimer tm;
            tm.start();
            while (tm.elapsed() < 3000 && !(w.board()->isConnected() && w.board()->state().valid && w.board()->state().profile != 0))
                QApplication::processEvents(QEventLoop::AllEvents, 20);
            QString text;
            bool clicked = false, labelsFit = true;
            QTimer::singleShot(400, [&]() {
                auto *box = qobject_cast<QMessageBox *>(QApplication::activeModalWidget());
                if (!box || box->objectName() != QStringLiteral("ECamCamPositions")) return;
                text = box->text();
                box->grab().save(QDir::temp().filePath(QStringLiteral("ecam_cam_positions.png")));
                for (QAbstractButton *b : box->buttons()) {
                    labelsFit = labelsFit && b->width() >= b->fontMetrics().horizontalAdvance(b->text()) + 2 * 14;
                    if (b->text() == QStringLiteral("Move to cam positions")) {
                        b->click();
                        clicked = true;
                    }
                }
            });
            QPushButton *start = nullptr;
            for (QPushButton *b : w.findChildren<QPushButton *>()) {
                if (b->text() == QStringLiteral("Start") && b->isEnabled()) start = b;
            }
            if (start) start->click();
            tm.restart();
            while (tm.elapsed() < 6000 && !w.statusBar()->currentMessage().contains(QStringLiteral("press Start")))
                QApplication::processEvents(QEventLoop::AllEvents, 20);
            const int cam0 = static_cast<int>(got.indexOf(QRegularExpression(QStringLiteral("^axjog 0 cam \\d.*"))));
            const int cam1 = static_cast<int>(got.indexOf(QRegularExpression(QStringLiteral("^axjog 1 cam \\d.*"))));
            // Between the two moves, axjog polls until M-0 said it had stopped.
            int pollsBetween = 0;
            for (int k = cam0 + 1; cam0 >= 0 && k < cam1; ++k) pollsBetween += got[k] == QStringLiteral("axjog");
            std::printf("     off the cam positions: start %s, popup \"%s\", moves at %d and %d, %d polls between\n", start ? "clicked" : "disabled",
                        qPrintable(text.section(QLatin1Char('\n'), 2).replace(QLatin1Char('\n'), QStringLiteral(" | "))), cam0, cam1, pollsBetween);
            Check(text.contains(QStringLiteral("X travel (M-0): 12.5 mm off")) && text.contains(QStringLiteral("Z lift (M-1): -3 mm off")),
                  "off the cam positions: the popup lists every axis off, with how far");
            Check(clicked && cam0 >= 0 && cam1 > cam0 && pollsBetween >= 2, "off the cam positions: moved one at a time, the next once the first stopped");
            Check(w.statusBar()->currentMessage().contains(QStringLiteral("press Start")), "off the cam positions: then the user presses Start");
            Check(labelsFit, "off the cam positions: every button wide enough for its label");
            w.board()->disconnectLink();
            w.DiscardChangesForDemo();
        } else {
            Check(false, "off the cam positions: a fake board on 127.0.0.1:9500");
        }
    }

    // Homing: the settings saved with an axis that has a home switch (none without: the hash of every other project
    // stays), up to the board and back, the review's checks, the Axis page's Homing window.
    {
        ecam::CamProject p = ecam::DefaultProject();
        Check(!io::ProjectToJson(p, false).contains("\"home\""), "homing: an axis without a home switch saves nothing of it");
        ecam::BoardInput sw;
        sw.pin = 4;
        sw.role = ecam::CcioRole::Home;
        sw.axis = 0;
        ecam::SetBoardInput(p, sw);
        ecam::AxisHome &h = p.axes[0].home;
        h.dir = 1;
        h.fast = 5.0;
        h.slow = 0.25;
        h.back = 2.0;
        h.at = -1.5;
        h.search = 300.0;
        h.order = 2;
        h.thenStart = false;
        h.required = true;
        ecam::NormalizeProject(p);
        ecam::CamProject back;
        QString err;
        Check(io::ProjectFromJson(io::ProjectToJson(p, false), &back, &err) && back.axes[0].home.dir == 1 && back.axes[0].home.fast == 5.0 &&
                  back.axes[0].home.slow == 0.25 && back.axes[0].home.back == 2.0 && back.axes[0].home.at == -1.5 &&
                  back.axes[0].home.search == 300.0 && back.axes[0].home.order == 2 && !back.axes[0].home.thenStart && back.axes[0].home.required,
              "homing: the settings survive saving and loading");
        QStringList lines;
        for (const std::string &l : io::UploadLines(p)) lines << QString::fromStdString(l);
        bool exact = false;
        Check(lines.filter(QRegularExpression(QStringLiteral("^home 0 dir=\\+ fast=5 slow=0.25 back=2 at=-1.5 then=stay order=2 search=300 req=1$"))).size() == 1,
              "homing: the home line goes up");
        lines = lines.filter(QRegularExpression(QStringLiteral("^(?!json )")));
        Check(io::ProjectFromUpload(lines, &back, &exact, &err) && back.axes[0].home.at == -1.5 && back.axes[0].home.required,
              "homing: back from the board's motion lines alone");
        auto has = [](const ecam::CamProject &q, const char *text) {
            for (const auto &c : review::AnalyzeProject(q).checks) {
                if (c.text.contains(QLatin1String(text))) return true;
            }
            return false;
        };
        Check(!has(p, "homing"), "homing: set up with a search length: no review check");
        {
            ecam::CamProject d = p;
            d.axes[0].home.decel = 250.0;
            ecam::CamProject db;
            QString e2;
            Check(!io::ProjectToJson(p, false).contains("\"decel\"") && io::ProjectFromJson(io::ProjectToJson(d, false), &db, &e2) &&
                      db.axes[0].home.decel == 250.0,
                  "homing decel: saved only when set, and back");
        }
        ecam::CamProject q = p;
        q.axes[0].home.search = 0.0;
        Check(has(q, "searches without end"), "homing: no search length and no limit that way: a warning");
        ecam::BoardInput lim;
        lim.pin = 5;
        lim.role = ecam::CcioRole::LimitPositive;
        lim.axis = 0;
        ecam::SetBoardInput(q, lim);
        Check(!has(q, "searches without end"), "homing: ... none with a + limit to stop it");
        q = ecam::DefaultProject();
        q.axes[0].home.required = true;
        Check(has(q, "no home switch"), "homing: required with no home switch: an error");

        MainWindow w;
        w.resize(1920, 1040);
        w.show();
        QApplication::processEvents();
        auto *summary = w.findChild<QLabel *>(QStringLiteral("ECamHomeSummary"));
        auto *fast = w.findChild<QDoubleSpinBox *>(QStringLiteral("ECamHomeFast"));
        auto *req = w.findChild<QCheckBox *>(QStringLiteral("ECamHomeRequired"));
        Check(summary && fast && req && summary->text() == QStringLiteral("No home switch") && !fast->isEnabled(),
              "homing page: no switch, the settings greyed");
        w.UseProject(p, QString(), false);
        QApplication::processEvents();
        Check(summary && summary->text().contains(QStringLiteral("IO-4")) && summary->text().contains(QStringLiteral("required")) &&
                  fast->isEnabled() && fast->value() == 5.0 && req->isChecked(),
              "homing page: the switch, its settings");
        auto *decel = w.findChild<QDoubleSpinBox *>(QStringLiteral("ECamHomeDecel"));
        Check(decel && decel->isEnabled() && decel->value() == 0.0 && decel->specialValueText() == QStringLiteral("as axis jog"),
              "homing page: the Decel setting, as axis jog until set");
        if (req) {
            req->setChecked(false);
            QApplication::processEvents();
            Check(!summary->text().contains(QStringLiteral("required")), "homing page: an edit applies");
        }
    }
    // Timeline waits: saved and back, the review's checks, the Waits page (Add snaps to a dwell), the Inputs page's job.
    {
        ecam::CamProject p = ecam::DefaultProject();  // on the timeline: 2 s, the first dwell 0.667..1.0 s
        ecam::SetAxisMaster(p, 0, ecam::AxisMaster::Timeline);
        p.master.trigger = ecam::TriggerSource::Command;
        ecam::TimelineWait w;
        w.t = 0.8;
        w.name = "Gripper closed";
        w.pin = 2;
        w.timeout = 1.5;
        p.waits.push_back(w);
        ecam::TimelineWait b = w;
        b.t = 1.8;
        b.name.clear();
        b.source = ecam::WaitSource::Plc;
        b.pin = 5;
        b.on = false;
        b.timeout = 0.0;
        p.waits.push_back(b);
        ecam::NormalizeProject(p);
        ecam::CamProject back;
        QString err;
        {
            // A 0.10.0 project's wait (no "change"): the level alone; a new one needs a change.
            const QByteArray old = io::ProjectToJson(p, false).replace("\"change\":true", "\"x\":0");
            ecam::CamProject o;
            QString e2;
            Check(ecam::TimelineWait().edge && io::ProjectFromJson(old, &o, &e2) && !o.waits[0].edge && io::ProjectFromJson(io::ProjectToJson(p, false), &o, &e2) &&
                      o.waits[0].edge,
                  "wait change: new waits need a change; a 0.10.0 project's are the level alone");
        }
        Check(!io::ProjectToJson(ecam::DefaultProject(), false).contains("\"waits\"") && io::ProjectFromJson(io::ProjectToJson(p, false), &back, &err) &&
                  back.waits.size() == 2 && back.waits[0].name == "Gripper closed" && back.waits[0].pin == 2 && back.waits[0].timeout == 1.5 &&
                  back.waits[1].source == ecam::WaitSource::Plc && back.waits[1].pin == 5 && !back.waits[1].on,
              "waits: saved and back (none saved without)");
        auto checks = [](const ecam::CamProject &q, const char *text) {
            for (const auto &c : review::AnalyzeProject(q).checks) {
                if (c.where == QStringLiteral("Waits") && c.text.contains(QLatin1String(text))) return true;
            }
            return false;
        };
        Check(!checks(p, "Wait"), "waits: at rest, on free pins: no review check");
        ecam::CamProject q = p;
        q.waits[0].t = 0.3;  // in the rise
        Check(checks(q, "moving then"), "waits: one where an axis moves is an error");
        q = p;
        q.waits[0].pin = 9;  // A-9, the estop's by default
        Check(review::AnalyzeProject(q, 9).errors() > review::AnalyzeProject(p, 9).errors(), "waits: on the estop's pin, an error");
        Check(ecam::TimelineAtRest(p, 0.8) && !ecam::TimelineAtRest(p, 0.3) && std::fabs(ecam::NearestTimelineRest(p, 0.3) - 0.667) < 0.002,
              "waits: at rest at 0.8 s, not at 0.3 s; the nearest rest to 0.3 s is the dwell's start");

        MainWindow mw;
        mw.resize(1920, 1040);
        mw.show();
        mw.UseProject(p, QString(), false);
        QApplication::processEvents();
        auto *table = mw.findChild<QTableWidget *>(QStringLiteral("ECamWaitTable"));
        Check(table && table->rowCount() == 2, "waits page: a row per wait");
        if (auto *add = mw.findChild<QPushButton *>(QStringLiteral("ECamWaitAdd"))) {
            mw.ShowAxis(0, 1.2);  // the cursor in the return: the wait goes to the nearest rest
            add->click();
            QApplication::processEvents();
        }
        Check(table && table->rowCount() == 3, "waits page: Add adds one");
        if (table && table->rowCount() == 3) {
            bool atRest = true;
            for (int r = 0; r < 3; ++r) {
                auto *t = qobject_cast<QDoubleSpinBox *>(table->cellWidget(r, 0));
                atRest = atRest && t && ecam::TimelineAtRest(p, t->value());
            }
            Check(atRest, "waits page: the added wait is where every timeline axis rests");
        }
        // Dragging a wait's line in the Timing view: dropped at rest it stays there; where an axis moves, it snaps.
        if (auto *timing = mw.findChild<TimingView *>()) {
            mw.UseProject(p, QString(), false);  // the two waits again: 0.8 s and 1.8 s
            QApplication::processEvents();
            timing->resize(900, 400);
            auto drag = [&](double from, double to) {
                const QPointF a(timing->xForTime(from), 200), b(timing->xForTime(to), 200);
                QMouseEvent press(QEvent::MouseButtonPress, a, timing->mapToGlobal(a), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
                QApplication::sendEvent(timing, &press);
                const bool grabbed = timing->draggedWait() >= 0;
                QMouseEvent move(QEvent::MouseMove, b, timing->mapToGlobal(b), Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
                QApplication::sendEvent(timing, &move);
                QMouseEvent release(QEvent::MouseButtonRelease, b, timing->mapToGlobal(b), Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
                QApplication::sendEvent(timing, &release);
                QApplication::processEvents();
                return grabbed;
            };
            const bool grabbed = drag(0.8, 0.9);
            auto *t0 = table ? qobject_cast<QDoubleSpinBox *>(table->cellWidget(0, 0)) : nullptr;
            Check(grabbed && t0 && std::fabs(t0->value() - 0.9) < 0.006, "waits: drag a wait's line to 0.9 s (at rest): it moves there");
            drag(t0 ? t0->value() : 0.9, 0.3);  // into the rise
            t0 = table ? qobject_cast<QDoubleSpinBox *>(table->cellWidget(0, 0)) : nullptr;
            Check(t0 && std::fabs(t0->value() - 0.667) < 0.003, "waits: dropped where an axis moves: it snaps to the nearest rest (0.667 s)");
            mw.SetLocked(true);
            Check(!drag(t0 ? t0->value() : 0.667, 0.9), "waits: locked, the line doesn't drag");
            mw.SetLocked(false);
            // Labels: one near the end flips to the left of its line; two close together stagger; the readout is above.
            ecam::CamProject q = p;
            q.waits[0].t = 0.80;
            q.waits[0].name = "Gripper closed";
            q.waits[1].t = 0.82;
            q.waits[1].name = "Vacuum OK";
            ecam::TimelineWait end = q.waits[0];
            end.t = 1.98;
            end.name = "Part clear of the press";
            q.waits.push_back(end);
            ecam::NormalizeProject(q);
            mw.UseProject(q, QString(), false);
            QApplication::processEvents();
            const QString shot = QDir::temp().filePath(QStringLiteral("ecam_timing_waits.png"));
            timing->grab().save(shot);
            std::printf("     timing view with waits: %s\n", qPrintable(QDir::toNativeSeparators(shot)));
            Check(timing->grab().width() > 0, "waits: the Timing view draws the labels and the readout");
        }
        auto *inputs = mw.findChild<QTableWidget *>(QStringLiteral("ECamInputsTable"));
        Check(inputs && inputs->item(2, 2)->text() == QStringLiteral("Timeline wait"), "waits: the Inputs page shows IO-2's job");
    }
    // Operator inputs: the window reads the board's config; the project's pins and the estop's greyed; the review and the
    // Inputs page know the board's operator pins.
    {
        ecam::CamProject p = ecam::DefaultProject();
        ecam::OutputPulse o;
        o.output = 2;
        o.on = 10;
        o.off = 20;
        p.outputs.push_back(o);
        ecam::NormalizeProject(p);
        BoardLink none;
        OperatorDialog d(&none, p, 9);
        d.applyConfig({QStringLiteral("estop_mode 1"), QStringLiteral("op_enable IO-3 no"), QStringLiteral("op_disable IO-4 nc"),
                       QStringLiteral("op_reset off no"), QStringLiteral("op_start A-10 no"), QStringLiteral("op_stop A-11 nc"),
                       QStringLiteral("op_enable_mode maintained")});
        Check(d.pin(0) == 3 && d.pin(1) == 4 && d.pin(2) == -1 && d.pin(3) == 10 && d.pin(4) == 11 && !d.nc(0) && d.nc(1) && d.nc(4) && d.maintained(),
              "operator: the window reads the board's config");
        auto *reset = d.findChild<QComboBox *>(QStringLiteral("ECamOpPin2"));
        auto flags = [&](int pin) {
            auto *model = qobject_cast<QStandardItemModel *>(reset->model());
            return model->item(reset->findData(pin))->flags();
        };
        Check(reset && !(flags(2) & Qt::ItemIsEnabled) && !(flags(9) & Qt::ItemIsEnabled) && !(flags(3) & Qt::ItemIsEnabled) && (flags(5) & Qt::ItemIsEnabled),
              "operator: an output's pin, the estop's and another operator input's are greyed");
        Check(d.message().contains(QStringLiteral("Not connected")), "operator: offline, it says so (nothing to change)");
        const int ops[5] = {3, -1, -1, -1, -1};
        ecam::CamProject q = p;
        q.outputs[0].output = 3;
        ecam::NormalizeProject(q);
        bool flagged = false;
        for (const auto &c : review::AnalyzeProject(q, -1, ops).checks) flagged = flagged || c.text.contains(QStringLiteral("operator Enable"));
        Check(flagged, "operator: the review flags the project using an operator pin");
        QSettings opStore(QStringLiteral("ECamUI"), QStringLiteral("ECamUI"));
        opStore.setValue(QStringLiteral("board/opPins"), QStringLiteral("3,-1,-1,-1,-1"));
        opStore.sync();
        {
            MainWindow mw;
            mw.resize(1920, 1040);
            mw.show();
            QApplication::processEvents();
            auto *inputs = mw.findChild<QTableWidget *>(QStringLiteral("ECamInputsTable"));
            Check(inputs && inputs->item(3, 2)->text() == QStringLiteral("Operator: Enable"), "operator: the Inputs page shows IO-3's job (remembered from the board)");
            auto *summary = mw.findChild<QLabel *>(QStringLiteral("ECamOpSummary"));
            Check(summary && summary->text() == QStringLiteral("Enable IO-3"), "operator: the Board page's Panel row");
        }
        opStore.remove(QStringLiteral("board/opPins"));
        opStore.sync();
    }
    // The switch setup wizard, without a board: find, contact, direction by motion, the project it hands back.
    {
        using W = SwitchWizard;
        Check(W::Judge(+1, W::kLimitPlus) == W::Verdict::Right && W::Judge(-1, W::kLimitMinus) == W::Verdict::Right,
              "wizard: a limit reached the way its job says is right");
        Check(W::Judge(+1, W::kLimitMinus) == W::Verdict::Ask && W::Judge(-1, W::kLimitPlus) == W::Verdict::Ask,
              "wizard: the other limit reached asks which is wrong");
        Check(W::Judge(+1, W::kHome) == W::Verdict::Home && W::Judge(+1, -1) == W::Verdict::Unknown, "wizard: home, and none");
        ecam::CamProject p = ecam::DefaultProject();  // an encoder master, one crank axis on M-1
        ecam::OutputPulse o;
        o.output = 0;
        o.master = ecam::AxisMaster::Crank;
        o.on = 10;
        o.off = 20;
        p.outputs.push_back(o);
        ecam::NormalizeProject(p);
        BoardLink none;
        int uploads = 0;
        W w(&none, p, 9, [&](const ecam::CamProject &up, std::function<void(bool, const QString &)> done) {
            ++uploads;
            Check(up.axes[0].reverse, "wizard: the reverse goes up with the upload");
            done(true, QStringLiteral("ok"));
        });
        w.show();
        w.setStepForTest(1);
        // At rest: IO-2, IO-3 (NC) high, IO-4 (NO) low; IO-0 is an output.
        const int rest = (1 << 2) | (1 << 3);
        w.feedLevels(rest, 0, 0);
        w.feedLevels(rest & ~(1 << 2) & ~(1 << 3), 0, 0);  // two at once
        Check(w.message().contains(QStringLiteral("More than one")), "wizard: two inputs changing is turned down");
        w.feedLevels(rest, 0, 0);
        w.feedLevels(rest | 1, 0, 0);  // IO-0: an output
        w.feedLevels(rest, 0, 0);
        Check(w.found(W::kLimitPlus).input < 0 && w.message().contains(QStringLiteral("IO-0 is an output")), "wizard: an output's pin isn't taken");
        w.feedLevels(rest, 0, 0);              // the next poll, at rest
        w.feedLevels(rest & ~(1 << 3), 0, 0);  // trip IO-3
        Check(w.message().contains(QStringLiteral("IO-3 changed")), "wizard: the input that changed is seen");
        w.feedLevels(rest, 0, 0);  // let go
        Check(w.found(W::kLimitPlus).input == 3 && !w.found(W::kLimitPlus).normallyOpen, "wizard: Limit + is IO-3, normally closed");
        w.feedLevels(rest, 0, 0);
        w.feedLevels(rest | (1 << 4), 0, 0);  // IO-4, normally open: it goes on
        w.feedLevels(rest, 0, 0);
        Check(w.found(W::kLimitMinus).input == 4 && w.found(W::kLimitMinus).normallyOpen, "wizard: Limit - is IO-4, normally open (off at rest)");
        auto *next = w.findChild<QPushButton *>(QStringLiteral("SwitchWizardNext"));
        Check(next && next->isEnabled(), "wizard: every switch found: on to the contact");
        // Direction: going + it reached IO-4 (the - limit). The switches were swapped by hand: swap them.
        w.setStepForTest(3);
        w.feedStop(+1, QStringLiteral("IO-4"));
        auto *question = w.findChild<QLabel *>(QStringLiteral("SwitchWizardQuestion"));
        Check(question && question->isVisible() && question->text().contains(QStringLiteral("+ limit")), "wizard: the - limit reached going + asks");
        w.answer(true);
        Check(w.found(W::kLimitPlus).input == 4 && w.found(W::kLimitPlus).proved && w.found(W::kLimitMinus).input == 3,
              "wizard: 'toward where you tripped it': the switches swap, + proved");
        w.feedStop(-1, QStringLiteral("IO-3"));
        Check(w.found(W::kLimitMinus).proved && next->isEnabled(), "wizard: - reached going -: both proved");
        // The other answer: the motor runs the other way. It reverses, uploads, and both must be proved again.
        w.feedStop(+1, QStringLiteral("IO-3"));
        w.answer(false);
        Check(uploads == 1 && !w.found(W::kLimitPlus).proved && !w.found(W::kLimitMinus).proved && !next->isEnabled(),
              "wizard: 'away from it': reversed, uploaded, to prove again");
        w.feedStop(+1, QStringLiteral("IO-4"));
        w.feedStop(-1, QStringLiteral("IO-3"));
        const ecam::CamProject r = w.result();
        const ecam::BoardInput lp = ecam::BoardInputAt(r, 4), lm = ecam::BoardInputAt(r, 3);
        Check(r.axes[0].reverse && lp.role == ecam::CcioRole::LimitPositive && lp.axis == 0 && lp.normallyOpen &&
                  lm.role == ecam::CcioRole::LimitNegative && !lm.normallyOpen && !lm.name.empty(),
              "wizard: the project it hands back: the limits, their contacts, names, the motor reversed");
        bool reviewed = false;
        for (const auto &c : review::AnalyzeProject(r).checks) reviewed = reviewed || c.text.contains(QStringLiteral("normally open limit"));
        Check(reviewed, "wizard: and the review warns about the normally open limit");
        w.reject();

        // The Limits and Home steps (phase 3): the board's limits stop the axis; homing N times.
        W v(&none, p, 9, [&](const ecam::CamProject &, std::function<void(bool, const QString &)> done) { done(true, QStringLiteral("ok")); });
        v.show();
        if (auto *home = v.findChild<QCheckBox *>(QStringLiteral("SwitchWizardWant2"))) home->setChecked(true);
        v.setStepForTest(1);
        v.feedLevels(rest, 0, 0);
        v.feedLevels(rest & ~(1 << 3), 0, 0);  // IO-3: + limit, NC
        v.feedLevels(rest, 0, 0);
        v.feedLevels(rest, 0, 0);
        v.feedLevels(rest & ~(1 << 2), 0, 0);  // IO-2: - limit, NC
        v.feedLevels(rest, 0, 0);
        v.feedLevels(rest, 0, 0);
        v.feedLevels(rest | (1 << 5), 0, 0);   // IO-5: home, NO
        v.feedLevels(rest, 0, 0);
        Check(v.found(W::kLimitPlus).input == 3 && v.found(W::kLimitMinus).input == 2 && v.found(W::kHome).input == 5, "wizard 3: three switches found");
        v.setStepForTest(3);
        v.feedStop(+1, QStringLiteral("IO-3"));
        v.feedStop(-1, QStringLiteral("IO-5"));
        v.feedStop(-1, QStringLiteral("IO-2"));
        Check(v.homeSide() == -1 && v.found(W::kLimitMinus).proved && v.found(W::kLimitPlus).proved, "wizard 3: proved, home on the - side");
        // Limits: 800 counts/mm. The + limit trips at 10 mm and the axis rests 0.2 mm on; then off it.
        v.setStepForTest(4);
        v.feedAxlimit({QStringLiteral("axlimit 0 M-1 lim=00 tripp=0 atp=0 tripm=1 atm=-50 pos=0")}, false);
        v.feedAxlimit({QStringLiteral("axlimit 0 M-1 lim=10 tripp=1 atp=8000 tripm=1 atm=-50 pos=8100")}, true);
        Check(v.limitStopped(0) && v.overtravelCounts(0) < 0 && !v.limitStopped(1), "wizard 3: + stopped by the limit (an earlier - trip not counted)");
        v.feedAxlimit({QStringLiteral("axlimit 0 M-1 lim=10 tripp=1 atp=8000 tripm=1 atm=-50 pos=8160")}, false);
        Check(v.overtravelCounts(0) == 160 && !v.limitCleared(0), "wizard 3: at rest, 0.2 mm past the switch");
        v.feedAxlimit({QStringLiteral("axlimit 0 M-1 lim=00 tripp=1 atp=8000 tripm=1 atm=-50 pos=7000")}, false);
        auto *nextV = v.findChild<QPushButton *>(QStringLiteral("SwitchWizardNext"));
        Check(v.limitCleared(0) && nextV && !nextV->isEnabled(), "wizard 3: + cleared, - still to do");
        v.feedAxlimit({QStringLiteral("axlimit 0 M-1 lim=01 tripp=1 atp=8000 tripm=2 atm=-8000 pos=-8200")}, false);
        v.feedAxlimit({QStringLiteral("axlimit 0 M-1 lim=00 tripp=1 atp=8000 tripm=2 atm=-8000 pos=-7000")}, false);
        auto *travel = v.findChild<QLabel *>(QStringLiteral("SwitchWizardTravel"));
        Check(v.limitCleared(1) && v.overtravelCounts(1) == 200 && nextV->isEnabled() && travel && travel->text().contains(QStringLiteral("20.00 mm")),
              "wizard 3: both limits tested, 20 mm between the switches");
        // Home: toward - (seen), the setup speed, a search a little over the travel.
        v.setStepForTest(5);
        auto *fastH = v.findChild<QDoubleSpinBox *>(QStringLiteral("SwitchWizardHomeFast"));
        auto *searchH = v.findChild<QDoubleSpinBox *>(QStringLiteral("SwitchWizardHomeSearch"));
        Check(fastH && searchH && searchH->value() == 22.0 && fastH->value() > 0 && !nextV->isEnabled(), "wizard 3: home settings proposed (search 22 mm)");
        v.startHomeRunsForTest(3);
        v.feedAxhome({QStringLiteral("axhome 0 M-1 has=1 homed=0 homing=0 ends=4 latch=0 pos=0 last=ok")});  // where the count is
        v.feedAxhome({QStringLiteral("axhome 0 M-1 has=1 homed=1 homing=0 ends=5 latch=-5000 pos=0 last=ok")});
        v.feedAxhome({QStringLiteral("axhome 0 M-1 has=1 homed=1 homing=0 ends=6 latch=1 pos=0 last=ok")});
        v.feedAxhome({QStringLiteral("axhome 0 M-1 has=1 homed=1 homing=0 ends=7 latch=-2 pos=0 last=ok")});
        Check(v.homeRuns() == 3 && v.homeSpreadCounts() == 3.0 && nextV->isEnabled(), "wizard 3: three homings, a spread of 3 counts");
        const ecam::CamProject rh = v.result();
        Check(ecam::HasHomeSwitch(rh, 0) && rh.axes[0].home.dir == -1 && rh.axes[0].home.search == 22.0 &&
                  ecam::BoardInputAt(rh, 5).role == ecam::CcioRole::Home && ecam::BoardInputAt(rh, 5).normallyOpen,
              "wizard 3: the project: the home switch and its settings");
        v.startHomeRunsForTest(3);
        v.feedAxhome({QStringLiteral("axhome 0 M-1 has=1 homed=1 homing=0 ends=7 latch=-2 pos=0 last=ok")});
        v.feedAxhome({QStringLiteral("axhome 0 M-1 has=1 homed=0 homing=0 ends=8 latch=-2 pos=0 last=notfound")});
        Check(!nextV->isEnabled() && v.message().contains(QStringLiteral("failed")), "wizard 3: a failed homing holds the step");
        v.reject();
    }
    // The board's estop input, remembered from the last board: kept free offline. Pick and place triggers on A-10: put the
    // estop there and the review flags it, the Inputs page and the trigger list keep A-10 from anything else.
    {
        QSettings us(QStringLiteral("ECamUI"), QStringLiteral("ECamUI"));
        us.setValue(QStringLiteral("board/estopPin"), 10);
        us.sync();
        {
            MainWindow t;
            t.resize(1920, 1040);
            t.show();
            t.OpenFile(QStringLiteral(ECAM_SOURCE_DIR "/samples/pick_and_place.ecam"));
            QApplication::processEvents();
            bool flagged = false;
            ecam::CamProject pp;
            QString err;
            io::LoadProject(QStringLiteral(ECAM_SOURCE_DIR "/samples/pick_and_place.ecam"), &pp, &err);
            flagged = false;
            for (const auto &c : review::AnalyzeProject(pp, 10).checks) {
                flagged = flagged || (c.level == review::Check::Error && c.text.contains(QStringLiteral("Trigger input A-10 is the board's estop input")));
            }
            Check(flagged, "estop pin: review: the trigger on the board's estop input is an error");
            bool quiet = true;
            for (const auto &c : review::AnalyzeProject(pp, 9).checks) quiet = quiet && !c.text.contains(QStringLiteral("estop"));
            Check(quiet, "estop pin: review: the estop on A-9 doesn't touch pick and place");
            auto *table = t.findChild<QTableWidget *>(QStringLiteral("ECamInputsTable"));
            Check(table && table->item(10, 2)->text() == QStringLiteral("Estop (last seen)"), "estop pin: Inputs shows A-10 as the estop, last seen");
            auto *estopPin = t.findChild<QComboBox *>(QStringLiteral("ECamEstopPin"));
            Check(estopPin && estopPin->count() == 13 && !estopPin->isEnabled(), "estop pin: the Board page's pin list, waiting for a board");
        }
        us.setValue(QStringLiteral("board/estopPin"), 9);
        us.sync();
        {
            MainWindow t;
            t.resize(1920, 1040);
            t.show();
            QApplication::processEvents();
            QComboBox *trig = nullptr;
            for (QComboBox *c : t.findChildren<QComboBox *>()) {
                if (c->count() == 13 && c->itemText(12) == QStringLiteral("A-12") && c->objectName() != QStringLiteral("ECamEstopPin")) trig = c;
            }
            auto *model = trig ? qobject_cast<QStandardItemModel *>(trig->model()) : nullptr;
            Check(model && !(model->item(9)->flags() & Qt::ItemIsEnabled) && (model->item(10)->flags() & Qt::ItemIsEnabled),
                  "estop pin: the trigger list greys A-9, the estop's");
        }
        us.remove(QStringLiteral("board/estopPin"));
        us.sync();
    }
    // The pick and place sample: outputs IO-0 and IO-1, trigger A-10. The trigger list greys IO-0 and IO-1 (and they
    // look it), the outputs' lists grey A-10's... nothing (it isn't an IO pin), and the diagram has those two lanes.
    {
        MainWindow t;
        t.resize(1920, 1040);
        t.show();
        t.OpenFile(QStringLiteral(ECAM_SOURCE_DIR "/samples/pick_and_place.ecam"));
        QApplication::processEvents();
        QApplication::processEvents();
        QComboBox *trig = nullptr;
        for (QComboBox *c : t.findChildren<QComboBox *>()) {
            if (c->count() == 13 && c->itemText(0) == QStringLiteral("IO-0") && c->itemText(12) == QStringLiteral("A-12")) trig = c;
        }
        Check(trig != nullptr, "trigger list found");
        if (auto *model = trig ? qobject_cast<QStandardItemModel *>(trig->model()) : nullptr) {
            const auto on = [&](int i) { return (model->item(i)->flags() & Qt::ItemIsEnabled) != 0; };
            Check(!on(0) && !on(1) && on(2) && on(5) && on(10), "trigger list: IO-0 and IO-1 (outputs) can't be picked, IO-2..IO-5 can");
            trig->showPopup();
            QApplication::processEvents();
            QAbstractItemView *view = trig->view();
            const QImage pop = view->grab().toImage();
            auto inkOf = [&](int row) {  // the brightest text pixel in the row
                const QRect r = view->visualRect(model->index(row, 0));
                int best = 0;
                for (int y = r.top(); y <= r.bottom(); ++y) {
                    for (int x = r.left(); x < std::min(r.left() + 60, r.right()); ++x) best = std::max(best, pop.pixelColor(x, y).lightness());
                }
                return best;
            };
            const int grey = inkOf(1), bright = inkOf(3);  // IO-1: the list may open scrolled past IO-0
            std::printf("     trigger list text: IO-1 (an output) %d, IO-3 %d\n", grey, bright);
            Check(grey + 40 < bright, "trigger list: the outputs look greyed");
            if (qEnvironmentVariableIsSet("ECAM_UI_SHOTS")) pop.save(QDir::temp().filePath(QStringLiteral("ecam_trigger_list.png")));
            trig->hidePopup();
        }
        auto *strip = t.findChild<ProfileStrip *>();
        if (strip && qEnvironmentVariableIsSet("ECAM_UI_SHOTS")) strip->grab().save(QDir::temp().filePath(QStringLiteral("ecam_outputs_band.png")));
    }

    // Excel workbook of every sample project: written, a zip (an .xlsx), no temporary file left behind.
    for (const char *name : {"mixed_timeline", "flying_cutoff", "packaging_machine", "bench_driven_crank", "pick_and_place",
                             "true_flying_cutoff"}) {
        ecam::CamProject p;
        QString err;
        if (!io::LoadProject(QStringLiteral(ECAM_SOURCE_DIR "/samples/%1.ecam").arg(QLatin1String(name)), &p, &err)) {
            Check(false, "load a sample for the workbook");
            continue;
        }
        const QString path = QDir::temp().filePath(QStringLiteral("ecam_test_%1.xlsx").arg(QLatin1String(name)));
        io::WorkbookOptions o;
        o.samples = 180;
        o.projectName = QLatin1String(name);
        const bool done = io::ExportWorkbook(p, o, path, &err);
        if (!done) std::printf("  workbook: %s\n", qPrintable(err));
        Check(done, "workbook export");
        QFile f(path);
        Check(f.open(QIODevice::ReadOnly) && f.read(2) == "PK" && f.size() > 10000, "workbook is an xlsx (zip)");
        Check(!QFile::exists(path + QStringLiteral(".part")), "workbook: no temporary file left");
    }

    // Project report of sample projects: a PDF with every section; pages as images (ECAM_SHOTS) to look at.
    for (const char *name : {"packaging_machine", "mixed_timeline", "true_flying_cutoff", "bench_driven_crank", "flying_cutoff",
                             "variable_length_cutoff", "index_feed", "driven_crank", "pick_and_place", "bench_timeline"}) {
        ecam::CamProject p;
        QString err;
        if (!io::LoadProject(QStringLiteral(ECAM_SOURCE_DIR "/samples/%1.ecam").arg(QLatin1String(name)), &p, &err)) {
            Check(false, "load a sample for the report");
            continue;
        }
        const QString path = QDir::temp().filePath(QStringLiteral("ecam_test_report_%1.pdf").arg(QLatin1String(name)));
        io::ReportOptions o;
        o.projectName = QLatin1String(name);
        o.projectPath = QStringLiteral(ECAM_SOURCE_DIR "/samples/%1.ecam").arg(QLatin1String(name));
        o.paper = io::ReportOptions::Paper::Letter;
        QElapsedTimer t;
        t.start();
        const bool done = io::ExportReport(p, o, path, &err);
        const qint64 ms = t.elapsed();
        QFile f(path);
        const bool pdf = f.open(QIODevice::ReadOnly) && f.read(5) == "%PDF-";
        const std::vector<QImage> pages = io::RenderReportPages(p, o, 90);
        // At least: cover, overview, timing, cycle, sign-off, and per axis its section and motion (crank axes:
        // and the cam); master, outputs and checks may share pages.
        int expected = 5;
        for (const ecam::CamAxis &a : p.axes) expected += a.master == ecam::AxisMaster::Crank ? 3 : 2;
        std::printf("     report %s: %lld KB, %d pages (at least %d), %lld ms\n", name, static_cast<long long>(f.size() / 1024),
                    static_cast<int>(pages.size()), expected, static_cast<long long>(ms));
        Check(done && pdf && f.size() > 20000, "report: a PDF");
        Check(static_cast<int>(pages.size()) >= expected && static_cast<int>(pages.size()) < 60, "report: every section has its pages");
        Check(!QFile::exists(path + QStringLiteral(".part")), "report: no temporary file left");
        // No page left (nearly) empty: some ink between the header and the footer on every page.
        int empty = 0;
        for (size_t i = 0; i < pages.size(); ++i) {
            const QImage img = pages[i].convertToFormat(QImage::Format_RGB32);
            const int y0 = img.height() * 70 / 792, y1 = img.height() * 730 / 792;  // the body, in points of a Letter page
            qint64 ink = 0;
            for (int y = y0; y < y1; ++y) {
                const QRgb *line = reinterpret_cast<const QRgb *>(img.constScanLine(y));
                for (int x = 0; x < img.width(); ++x) ink += qGray(line[x]) < 235;
            }
            const double share = static_cast<double>(ink) / (static_cast<double>(img.width()) * (y1 - y0));
            if (share < 0.01) {
                ++empty;
                std::printf("     report %s: page %d is nearly empty (%.2f %% ink)\n", name, static_cast<int>(i + 1), share * 100.0);
            }
        }
        Check(empty == 0, "report: no nearly empty page");
        const QString shots = QString::fromLocal8Bit(qgetenv("ECAM_SHOTS"));
        if (!shots.isEmpty()) {
            for (size_t i = 0; i < pages.size(); ++i) {
                pages[i].save(QDir(shots).filePath(QStringLiteral("report_%1_%2.png").arg(QLatin1String(name)).arg(i + 1, 2, 10, QChar('0'))));
            }
        }
    }

    // The path DXF's window: every widget in it in its form (one left out is drawn at the top left, over the rest), for a
    // crank-only project and for one with both masters. Saved to look at.
    // A timeline axis also has its time scale: 40 mm/s to begin with, 10 in/s in inches.
    for (const char *sample : {"index_feed.ecam", "bench_timeline.ecam"}) {
        const bool timeline = std::string(sample) == "bench_timeline.ecam";
        {
            QSettings s(QStringLiteral("ECamUI"), QStringLiteral("ECamUI"));
            s.setValue(QStringLiteral("dxf/units"), 0);
            s.remove(QStringLiteral("dxf/perSecondMm"));
            s.remove(QStringLiteral("dxf/perSecondIn"));
        }
        MainWindow w;
        w.resize(1600, 900);
        w.show();
        w.OpenFile(QStringLiteral(ECAM_SOURCE_DIR "/samples/") + QString::fromLatin1(sample));
        QApplication::processEvents();
        QAction *act = nullptr;
        for (QAction *a2 : w.findChildren<QAction *>()) {
            if (a2->text().startsWith(QStringLiteral("Export path DXF"))) act = a2;
        }
        int stray = -1, rows = 0;
        double perSecMm = -1, perSecIn = -1;
        QString inSuffix;
        QString shot;
        QTimer::singleShot(300, [&]() {
            auto *dlg = qobject_cast<QDialog *>(QApplication::activeModalWidget());
            if (!dlg) return;
            QSet<QWidget *> placed;
            if (auto *form = qobject_cast<QFormLayout *>(dlg->layout())) {
                rows = form->rowCount();
                for (int i = 0; i < form->count(); ++i) {
                    if (QWidget *x = form->itemAt(i)->widget()) placed.insert(x);
                }
            }
            stray = 0;
            for (QWidget *x : dlg->findChildren<QWidget *>(QString(), Qt::FindDirectChildrenOnly)) {
                if (x->isVisible() && !placed.contains(x)) ++stray;
            }
            if (auto *ps = dlg->findChild<QDoubleSpinBox *>(QStringLiteral("ECamDxfPerSecond"))) {
                perSecMm = ps->value();
                if (auto *u = dlg->findChild<QComboBox *>()) u->setCurrentIndex(1);
                perSecIn = ps->value();
                inSuffix = ps->suffix();
            }
            shot = QDir::temp().filePath(QStringLiteral("ecam_path_dxf_dialog_%1.png").arg(QString::fromLatin1(sample).section('.', 0, 0)));
            dlg->grab().save(shot);
            dlg->reject();
        });
        if (act) act->trigger();
        std::printf("     path DXF window (%s): %d rows, %d stray, %s\n", sample, rows, stray, qPrintable(QDir::toNativeSeparators(shot)));
        Check(act && stray == 0 && rows == (timeline ? 3 : 2),
              (std::string("path DXF window: the units, and the time scale on a timeline axis, in its form (") + sample + ")").c_str());
        if (timeline) {
            Check(perSecMm == 40.0 && perSecIn == 10.0 && inSuffix == QStringLiteral(" in/s"),
                  "path DXF window: the time scale 40 mm/s, 10 in/s to begin with");
        }
        w.DiscardChangesForDemo();
    }

    // DXF (R2000) of the default cam (base 40, roller 8, stroke 20, mm), in mm and in inches; the paths of a crank axis
    // and of a timeline axis. Each read back: the polylines, the circles, the units, the scales.
    {
        struct DxfRead {
            QMap<QString, QVector<QVector<QPointF>>> polys;  // per layer, each polyline's vertices
            QMap<QString, QVector<double>> circles;           // per layer, radii
            QStringList texts;
            int units = -1, closedFlags = 0;
            QString version;
        };
        auto readDxf = [](const QString &path) {
            DxfRead r;
            QFile f(path);
            f.open(QIODevice::ReadOnly | QIODevice::Text);
            const QStringList lines = QString::fromUtf8(f.readAll()).split(QLatin1Char('\n'));
            QString entity, layer, var;
            double x = 0;
            for (int i = 0; i + 1 < lines.size(); i += 2) {
                const int code = lines[i].trimmed().toInt();
                const QString v = lines[i + 1].trimmed();
                if (code == 0) {
                    entity = v;
                    layer.clear();
                } else if (code == 9) {
                    var = v;
                } else if (code == 1 && var == "$ACADVER" && r.version.isEmpty()) {
                    r.version = v;
                } else if (code == 70 && var == "$INSUNITS" && entity == "SECTION") {
                    r.units = v.toInt();
                    var.clear();
                } else if (code == 8) {
                    layer = v;
                    if (entity == "LWPOLYLINE") r.polys[layer].append(QVector<QPointF>());
                } else if (entity == "LWPOLYLINE" && code == 70) {
                    r.closedFlags += v.toInt() & 1;
                } else if (entity == "LWPOLYLINE" && code == 10) {
                    x = v.toDouble();
                } else if (entity == "LWPOLYLINE" && code == 20) {
                    r.polys[layer].last().append(QPointF(x, v.toDouble()));
                } else if (entity == "CIRCLE" && code == 40) {
                    r.circles[layer].append(v.toDouble());
                } else if (entity == "TEXT" && code == 1) {
                    r.texts << v;
                }
            }
            return r;
        };
        auto radii = [](const QVector<QPointF> &pts, double *lo, double *hi) {
            *lo = 1e9, *hi = 0;
            for (const QPointF &q : pts) *lo = std::min(*lo, std::hypot(q.x(), q.y())), *hi = std::max(*hi, std::hypot(q.x(), q.y()));
        };
        ecam::CamProject p = ecam::DefaultProject();
        const QString path = QDir::temp().filePath(QStringLiteral("ecam_test_cam.dxf"));
        io::CamDxfOptions o;
        o.boreDiameter = 12.0;
        QString err;
        Check(io::ExportCamDxf(p, 0, o, path, &err), "DXF export");
        QFile::remove(QDir::temp().filePath(QStringLiteral("ecam_test_cam_mm.dxf")));
        QFile::copy(path, QDir::temp().filePath(QStringLiteral("ecam_test_cam_mm.dxf")));
        DxfRead r = readDxf(path);
        const QVector<QPointF> outline = r.polys.value("CAM_PROFILE").value(0);
        double rMin = 0, rMax = 0;
        radii(outline, &rMin, &rMax);
        std::printf("     DXF %s: %lld vertices, radius %.4f .. %.4f, $INSUNITS %d\n", qPrintable(r.version),
                    static_cast<long long>(outline.size()), rMin, rMax, r.units);
        Check(r.version == "AC1015", "DXF: R2000 (AC1015)");
        Check(outline.size() == 3600 && r.closedFlags >= 1, "cam outline: closed polyline, 3600 vertices");
        Check(std::fabs(rMin - 40.0) < 1e-3, "smallest outline radius = base circle (40)");
        Check(std::fabs(rMax - 60.0) < 1e-3, "largest outline radius = base + stroke (60)");
        Check(r.circles.value("BASE_CIRCLE").value(0) == 40.0 && std::fabs(r.circles.value("BORE").value(0) - 6.0) < 1e-9,
              "base circle and bore");
        Check(r.units == 4, "units: mm");
        // The same cam in inches: every length / 25.4, $INSUNITS 1.
        o.units = io::DxfUnits::Inch;
        Check(io::ExportCamDxf(p, 0, o, path, &err), "DXF export in inches");
        r = readDxf(path);
        radii(r.polys.value("CAM_PROFILE").value(0), &rMin, &rMax);
        Check(r.units == 1 && std::fabs(rMin - 40.0 / 25.4) < 1e-5 && std::fabs(rMax - 60.0 / 25.4) < 1e-5 &&
                  std::fabs(r.circles.value("BASE_CIRCLE").value(0) - 40.0 / 25.4) < 1e-5,
              "cam DXF in inches: radii / 25.4, $INSUNITS 1");

        // The crank axis's path: 0..360 deg across at 0.5 mm a degree, the position up; in inches, the position / 25.4.
        io::PathDxfOptions po;
        po.perDegree = 0.5;
        po.samples = 720;
        Check(io::ExportPathDxf(p, 0, po, path, &err), "path DXF export");
        r = readDxf(path);
        QVector<QPointF> curve = r.polys.value("PATH").value(0);
        double peak = 0;
        for (const QPointF &q : curve) peak = std::max(peak, q.y());
        const double at90 = ecam::Evaluate(p.axes[0], 90.0).pos;
        Check(curve.size() == 721 && std::fabs(curve.first().x()) < 1e-9 && std::fabs(curve.last().x() - 180.0) < 1e-6 &&
                  std::fabs(curve[180].x() - 45.0) < 1e-6 && std::fabs(curve[180].y() - at90) < 1e-6 && std::fabs(peak - 20.0) < 1e-3,
              "path DXF: 721 points, 0.5 mm a degree, the position up (peak = stroke 20)");
        Check(r.units == 4 && !r.texts.filter(QStringLiteral("ECW-PATH master=crank across=0.5 per=deg up=mm")).isEmpty(),
              "path DXF: mm, and the scales noted for reading it back");
        po.units = io::DxfUnits::Inch;
        po.perDegree = 0.02;
        Check(io::ExportPathDxf(p, 0, po, path, &err), "path DXF export in inches");
        r = readDxf(path);
        curve = r.polys.value("PATH").value(0);
        peak = 0;
        for (const QPointF &q : curve) peak = std::max(peak, q.y());
        Check(r.units == 1 && std::fabs(peak - 20.0 / 25.4) < 1e-5 && std::fabs(curve.last().x() - 7.2) < 1e-6,
              "path DXF in inches: position / 25.4, 0.02 in a degree");
        // Picked across: the default cam (20 mm stroke) is about twice as wide as tall, at an easy scale.
        io::PathDxfOptions ap;
        Check(io::ExportPathDxf(p, 0, ap, path, &err), "path DXF export, scale picked");
        r = readDxf(path);
        curve = r.polys.value("PATH").value(0);
        const double width = curve.isEmpty() ? 0 : curve.last().x();
        std::printf("     path DXF picked scale: %.1f mm across for a 20 mm stroke\n", width);
        Check(width >= 20.0 && width <= 80.0 && !r.texts.filter(QStringLiteral("1 deg = ")).isEmpty(),
              "path DXF: the scale across picked for a drawing about twice as wide as tall, and noted");
        // A timeline axis: seconds across.
        ecam::CamProject tl = ecam::BuildTemplate(ecam::DefaultTemplate(ecam::TemplateKind::TriggeredMove));
        int ti = -1;
        for (size_t i = 0; i < tl.axes.size(); ++i) {
            if (tl.axes[i].master == ecam::AxisMaster::Timeline) ti = static_cast<int>(i);
        }
        if (ti >= 0) {
            io::PathDxfOptions tp;
            tp.perSecond = 50.0;
            Check(io::ExportPathDxf(tl, ti, tp, path, &err), "timeline path DXF export");
            r = readDxf(path);
            curve = r.polys.value("PATH").value(0);
            Check(!curve.isEmpty() && std::fabs(curve.last().x() - tl.axes[ti].span * 50.0) < 1e-6 &&
                      !r.texts.filter(QStringLiteral("ECW-PATH master=timeline across=50 per=s")).isEmpty(),
                  "timeline path DXF: 50 mm a second across");
        } else {
            Check(false, "timeline path DXF: the triggered-move template has a timeline axis");
        }
        QFile::remove(QDir::temp().filePath(QStringLiteral("ecam_test_path_timeline.dxf")));
        QFile::copy(path, QDir::temp().filePath(QStringLiteral("ecam_test_path_timeline.dxf")));
    }

    // Guided setup: every template goes through the wizard to the project the template builder makes.
    // ECAM_SHOTS=<dir> also saves pictures of the splash, the start page and each wizard page.
    {
        const QString shots = QString::fromLocal8Bit(qgetenv("ECAM_SHOTS"));
        auto shot = [&](QWidget *w, const QString &name) {
            if (shots.isEmpty()) return;
            QApplication::processEvents();
            w->grab().save(QDir(shots).filePath(name + QStringLiteral(".png")));
        };
        if (!shots.isEmpty()) {
            QSplashScreen *splash = startup::ShowSplash();
            shot(splash, QStringLiteral("splash"));
            splash->close();
            delete splash;
            startup::StartDialog start;
            start.resize(880, 580);
            start.show();
            shot(&start, QStringLiteral("start"));
        }
        for (int k = 0; k < static_cast<int>(ecam::TemplateKind::Count); ++k) {
            const auto kind = static_cast<ecam::TemplateKind>(k);
            SetupWizard wz;
            wz.resize(1040, 680);
            wz.show();
            wz.selectTemplate(kind);
            QApplication::processEvents();
            const ecam::CamProject got = wz.project();
            const ecam::CamProject want = ecam::BuildTemplate(ecam::DefaultTemplate(kind));
            bool same = wz.spec().kind == kind && got.axes.size() == want.axes.size();
            for (size_t i = 0; same && i < got.axes.size(); ++i) same = got.axes[i].segments.size() == want.axes[i].segments.size();
            Check(same, "wizard: template builds its default project");
            // One motor step per motor: the flying cut-off adds the saw's.
            const bool flying = kind == ecam::TemplateKind::FlyingCutoff;
            Check(wz.pageCount() == (flying ? 6 : 5), "wizard: a motor step per motor");
            if (flying) {
                const ecam::TemplateSpec before = wz.spec();
                for (int i = 0; i < 2; ++i) wz.next();
                Check(wz.pageTitle() == QStringLiteral("Carriage motor"), "wizard: the carriage motor step");
                wz.next();
                Check(wz.pageTitle() == QStringLiteral("Saw motor"), "wizard: the saw motor step follows the carriage's");
                Check(wz.spec().sawConnector == before.sawConnector, "wizard: stepping keeps the saw settings");
            }
            const QString tag = QStringLiteral("wizard%1").arg(k);
            while (wz.page() < wz.pageCount() - 1) {
                shot(&wz, QStringLiteral("%1_p%2").arg(tag).arg(wz.page() + 1));
                wz.next();
            }
            shot(&wz, QStringLiteral("%1_p%2").arg(tag).arg(wz.page() + 1));
            Check(wz.page() == wz.pageCount() - 1, "wizard: walks to the review page");
            wz.next();
            Check(wz.result() == QDialog::Accepted, "wizard: Create project accepts");
        }
        // A wizard project in the main window: untitled and modified (asks to save on close).
        MainWindow w;
        w.UseProject(ecam::BuildTemplate(ecam::DefaultTemplate(ecam::TemplateKind::FlyingCutoff)), QString(), true);
        QApplication::processEvents();
        Check(w.windowTitle().contains(QStringLiteral("Untitled*")), "wizard project: untitled, modified");
        Check(!w.IsLocked(), "lock: a wizard project starts unlocked");
        {
            // An opened file starts locked (the recent list and last folder are put back afterwards).
            QSettings user(QStringLiteral("ECamUI"), QStringLiteral("ECamUI"));
            const QVariant recent = user.value(QStringLiteral("files/recent"));
            const QVariant lastDir = user.value(QStringLiteral("files/lastDir"));
            const QString path = QDir::temp().filePath(QStringLiteral("ecam_lock_test.ecam"));
            QString err;
            MainWindow o;
            Check(io::SaveProject(ecam::BuildTemplate(ecam::DefaultTemplate(ecam::TemplateKind::RiseDwellReturn)), path, &err) &&
                      o.OpenFile(path) && o.IsLocked(),
                  "lock: an opened file starts locked");
            startup::RemoveRecentProject(path);
            if (recent.isValid()) user.setValue(QStringLiteral("files/recent"), recent); else user.remove(QStringLiteral("files/recent"));
            if (lastDir.isValid()) user.setValue(QStringLiteral("files/lastDir"), lastDir); else user.remove(QStringLiteral("files/lastDir"));
            QFile::remove(path);
        }
        // The diagram's own axis dropdown switches the axis (and the Settings selector follows); the readout
        // sits in the diagram's bar.
        {
            w.resize(1480, 940);
            w.show();
            QApplication::processEvents();
            auto *stripAxis = w.findChild<QComboBox *>(QStringLiteral("ECamStripAxis"));
            auto *readout = w.findChild<QLabel *>(QStringLiteral("ECamReadout"));
            Check(stripAxis && stripAxis->count() == 2 && stripAxis->currentIndex() == 0, "diagram axis dropdown lists the axes");
            if (stripAxis) {
                stripAxis->setCurrentIndex(1);
                QApplication::processEvents();
                Check(stripAxis->itemText(1).startsWith(QStringLiteral("Saw plunge")), "dropdown: name and motor");
                auto *table = w.findChild<QTableWidget *>(QStringLiteral("ECamSegments"));
                Check(table && table->rowCount() == 5, "dropdown: the saw's 5 segments are in the table");
                // The tree and the dropdown are two views of one selection.
                auto *tree = w.findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"));
                auto *details = w.findChild<QStackedWidget *>(QStringLiteral("ECamDetails"));
                QTreeWidgetItem *axes = tree ? tree->topLevelItem(0)->child(1) : nullptr;
                Check(axes && axes->childCount() == 2, "tree: both axes listed");
                if (axes && details && axes->childCount() == 2) {
                    Check(axes->child(1)->text(0).startsWith(QStringLiteral("Saw plunge")), "tree: the axis's name");
                    tree->setCurrentItem(axes->child(1));
                    QApplication::processEvents();
                    Check(details->currentIndex() == 2 && tree->currentItem() == axes->child(1), "tree: the dropdown's axis is selected");
                    tree->setCurrentItem(axes->child(0)->child(0));
                    QApplication::processEvents();
                    Check(stripAxis->currentIndex() == 0 && details->currentIndex() == 3, "tree: another axis's Segments switches the axis");
                    stripAxis->setCurrentIndex(1);
                    QApplication::processEvents();
                    Check(tree->currentItem() == axes->child(1)->child(0) && details->currentIndex() == 3,
                          "tree: the dropdown moves the tree, on the same page");
                    Check(table && table->rowCount() == 5, "tree: the segment table shows that axis");
                }
            }
            QApplication::processEvents();
            // Cut-length mode: the master is line distance (x, mm).
            Check(readout && readout->text().contains(QStringLiteral(">x<")) && readout->text().contains(QStringLiteral("mm/s²")),
                  "readout in the diagram's bar");
            const QString shots = QString::fromLocal8Bit(qgetenv("ECAM_SHOTS"));
            if (!shots.isEmpty()) w.grab().save(QDir(shots).filePath(QStringLiteral("main_saw.png")));

            // Zoom and pan the master axis.
            auto *strip = w.findChild<ProfileStrip *>();
            auto *fit = w.findChild<QPushButton *>(QStringLiteral("ECamFit"));
            if (strip && fit) {
                Check(strip->zoom() == 1.0 && !fit->isEnabled(), "zoom: starts fitted");
                // Ctrl+wheel up two notches at the middle of the plot zooms 1.25^2 about that point.
                const QPointF at(strip->width() / 2.0, strip->height() / 4.0);
                QWheelEvent wheel(at, strip->mapToGlobal(at), QPoint(), QPoint(0, 240), Qt::NoButton, Qt::ControlModifier,
                                  Qt::NoScrollPhase, false);
                QApplication::sendEvent(strip, &wheel);
                QApplication::processEvents();
                Check(std::fabs(strip->zoom() - 1.5625) < 1e-9 && fit->isEnabled() && fit->text().contains(QStringLiteral("1.6")),
                      "zoom: Ctrl+wheel zooms and the Fit button shows it");
                Check(strip->viewStart() > 0.0 && strip->viewStart() + strip->visibleSpan() < 360.0, "zoom: about the mouse");
                // Pan to the far right: clamped at the end of the cycle.
                strip->panBy(1000.0);
                Check(std::fabs(strip->viewStart() + strip->visibleSpan() - 360.0) < 1e-9, "pan: stops at the end");
                strip->panBy(-1000.0);
                Check(strip->viewStart() == 0.0, "pan: stops at the start");
                strip->zoomBy(1000.0);
                Check(strip->zoom() == 200.0, "zoom: capped");
                strip->zoomBy(0.0001);
                Check(strip->zoom() == 1.0 && strip->viewStart() == 0.0, "zoom: never below fit");
                strip->zoomBy(4.0);
                if (!shots.isEmpty()) {
                    QApplication::processEvents();
                    w.grab().save(QDir(shots).filePath(QStringLiteral("main_zoom.png")));
                }
                fit->click();
                Check(strip->zoom() == 1.0 && !fit->isEnabled(), "zoom: Fit resets");
            } else {
                Check(false, "zoom: strip and Fit button found");
            }
        }
        // Bundled examples load from the resources.
        for (const startup::Example &e : startup::Examples()) {
            ecam::CamProject p;
            QString err;
            Check(io::LoadProject(e.file, &p, &err) && !p.axes.empty(), "bundled example loads");
        }
    }

    // ClearPath link: the framing against bytes seen on the wire (CLEARPATH_USB.md), then the scope
    // end to end against the simulated motor (the same protocol code a real motor gets).
    if (full) {
        using namespace clearpath;
        Check(Frame(5, 0, QByteArray()) == QByteArray::fromHex("d00030"), "clearpath: set-address packet d0 00 30");
        Check(Frame(1, 0, QByteArray()) == QByteArray::fromHex("900070"), "clearpath: empty reply 90 00 70");
        bool round = true;
        for (int n = 1; n <= 16; ++n) {
            QByteArray b;
            for (int i = 0; i < n; ++i) b.append(static_cast<char>((i * 73 + n * 31) & 0xFF));
            round = round && Unpack(Pack7(b), n) == b;
        }
        Check(round, "clearpath: 7-bit packing round trip");
        Check(Value(Pack7(QByteArray::fromHex("feff")), 2, true) == -2 && Value(Pack7(QByteArray::fromHex("feff")), 2, false) == 65534,
              "clearpath: signed and unsigned values");
        // A scope sample: 7 bytes in 8 characters, value in bytes 2-3.
        QByteArray smp = QByteArray::fromHex("0102e8fc0000f9");
        const QByteArray pkt = Frame(3, 0, smp, true);
        Check(pkt.size() == 11 && (static_cast<uint8_t>(pkt[1]) & 0x1F) == 8 && (static_cast<uint8_t>(pkt[1]) & 0x40),
              "clearpath: scope sample framing");
        // Position error scaling at MSP's gain: raw x 32768 / gain counts.
        Check(std::fabs(MonScale(*FindMonVar(27), 256) - 1.0) < 1e-12, "clearpath: position error scale");
        Check(std::fabs(MonScale(*FindMonVar(3), 3905) - 480.0) < 0.5, "clearpath: velocity 3905 raw = 480 RPM (capture)");

        // The link against the simulated node: connect, read, monitor channel save flag refused.
        try {
            Link link(OpenSimulated());
            Check(link.nodeCount() == 1, "clearpath sim: one node");
            Check(link.get(params::InputResolution) == 800, "clearpath sim: read a register");
            const MonState m1 = link.monitorGet(1);
            bool refused = false;
            try {
                link.monitorSet(1, MonState{m1.var | Link::kMonSaveNv, m1.gain, 0, 0});
            } catch (const std::exception &) {
                refused = true;
            }
            Check(refused && link.monitorGet(1) == m1, "clearpath: never saves the monitor channel to the motor");
        } catch (const std::exception &e) {
            std::printf("  %s\n", e.what());
            Check(false, "clearpath sim: link");
        }

        // The scope window recording from the simulated motor. It saves its lanes and layout as the user
        // changes them, so keep the user's own scope settings aside and put them back afterwards.
        QSettings userSettings(QStringLiteral("ECamUI"), QStringLiteral("ECamUI"));
        userSettings.beginGroup(QStringLiteral("clearpathScope"));
        QMap<QString, QVariant> savedScope;
        for (const QString &k : userSettings.allKeys()) savedScope.insert(k, userSettings.value(k));
        userSettings.endGroup();
        ScopeWindow sw;
        {
            // Every ClearPath on USB is listed, then the simulated motor; Refresh keeps what is picked.
            auto *ports = sw.findChild<QComboBox *>(QStringLiteral("ECamScopePort"));
            const QStringList usb = clearpath::FindPorts();
            bool listed = ports && ports->count() == usb.size() + 1;
            for (int i = 0; listed && i < usb.size(); ++i) listed = ports->itemData(i).toString() == usb[i];
            Check(listed, "scope: lists every ClearPath on USB and the simulated motor");
            std::printf("     %lld ClearPath(s) on USB: %s\n", static_cast<long long>(usb.size()), qPrintable(usb.join(QLatin1Char(' '))));
            QPushButton *refresh = nullptr;
            for (QPushButton *b : sw.findChildren<QPushButton *>()) {
                if (b->text() == QStringLiteral("Refresh")) refresh = b;
            }
            if (ports && refresh) {
                ports->setCurrentIndex(ports->count() - 1);
                const QString picked = ports->currentData().toString();
                refresh->click();
                Check(ports->currentData().toString() == picked, "scope: Refresh keeps the motor picked");
            } else {
                Check(false, "scope: the port list and Refresh are there");
            }
        }
        sw.resize(1240, 760);
        sw.setLanes(scope::DefaultLanes());
        sw.show();
        sw.startSimulated();
        QElapsedTimer et;
        et.start();
        while (et.elapsed() < 2500) QApplication::processEvents(QEventLoop::AllEvents, 20);
        const scope::Series *pe = sw.series(QStringLiteral("mon27"));
        const scope::Series *vel = sw.series(QStringLiteral("reg.velocity"));
        const scope::Series *hl = sw.series(QStringLiteral("dig.hlfb"));
        std::printf("     scope: %zu stream samples, %zu velocity reads in 2.5 s\n", pe ? pe->t.size() : size_t(0),
                    vel ? vel->t.size() : size_t(0));
        Check(sw.recording() && pe && pe->t.size() > 1200, "scope: streams about 845 samples/s");
        Check(vel && vel->t.size() > 15, "scope: reads the registers in turn while streaming");
        Check(hl && pe && hl->t.size() == pe->t.size(), "scope: digital lanes from every stream sample");
        bool monotonic = pe && !pe->t.empty();
        for (size_t i = 1; monotonic && i < pe->t.size(); ++i) monotonic = pe->t[i] > pe->t[i - 1];
        Check(monotonic, "scope: sample times increase");
        Check(sw.laneCount() == 7, "scope: seven default lanes (the critical concerns)");
        // Freeze while recording: a badge in the status row (not text over the time axis).
        {
            QPushButton *freeze = nullptr;
            for (QPushButton *b : sw.findChildren<QPushButton *>()) {
                if (b->text() == QLatin1String("Freeze")) freeze = b;
            }
            Check(freeze && !sw.frozenBadgeShown(), "scope: no frozen badge while live");
            if (freeze) {
                freeze->setChecked(true);
                QApplication::processEvents();
                Check(sw.frozenBadgeShown(), "scope: frozen badge while frozen and recording");
                const QString dir = QString::fromLocal8Bit(qgetenv("ECAM_SHOTS"));
                if (!dir.isEmpty()) sw.grab().save(QDir(dir).filePath(QStringLiteral("scope_frozen.png")));
                freeze->setChecked(false);
                QApplication::processEvents();
                Check(!sw.frozenBadgeShown(), "scope: badge gone when unfrozen");
            }
            // Help: its own window with the document.
            sw.ShowHelp();
            QApplication::processEvents();
            auto *doc = sw.helpDialog() ? sw.helpDialog()->findChild<QTextBrowser *>() : nullptr;
            Check(sw.helpDialog() && sw.helpDialog()->isVisible() && doc && doc->toPlainText().contains(QStringLiteral("Trigger")) &&
                      doc->toPlainText().contains(QStringLiteral("ClearPath-MSP")),
                  "scope: Help opens the help document");
            const QString dir = QString::fromLocal8Bit(qgetenv("ECAM_SHOTS"));
            if (!dir.isEmpty() && sw.helpDialog()) sw.helpDialog()->grab().save(QDir(dir).filePath(QStringLiteral("scope_help.png")));
            if (sw.helpDialog()) sw.helpDialog()->close();
        }
        for (int win : {5, 30}) {
            sw.setWindowSeconds(win);
            if (win == 30) {  // fill the longer window
                et.restart();
                while (et.elapsed() < 6000) QApplication::processEvents(QEventLoop::AllEvents, 20);
            }
            sw.resetPaintStats();
            et.restart();
            while (et.elapsed() < 1500) QApplication::processEvents(QEventLoop::AllEvents, 20);
            std::printf("     scope paint, %d s window: worst frame %.1f ms\n", win, sw.maxPaintMs());
            // The UI thread draws about 30 frames a second: a frame must stay well inside one.
            Check(sw.maxPaintMs() < 16.0, win == 5 ? "scope: a frame draws in under 16 ms (5 s window)"
                                                   : "scope: a frame draws in under 16 ms (30 s window)");
        }
        sw.setWindowSeconds(5);
        // Overlay: velocity drawn in the position-error lane (two units, so each keeps its own scale).
        {
            std::vector<scope::Lane> ov = scope::DefaultLanes();
            for (scope::Lane &l : ov) {
                if (l.key == QLatin1String("reg.velocity")) l.overlay = true;
            }
            sw.setLanes(ov);
            Check(sw.laneCount() == 7 && sw.shownLaneCount() == 6, "scope: an overlaid signal shares the lane above");
            sw.resetPaintStats();
            et.restart();
            while (et.elapsed() < 1500) QApplication::processEvents(QEventLoop::AllEvents, 20);
            std::printf("     scope paint with an overlay: worst frame %.1f ms\n", sw.maxPaintMs());
            Check(sw.maxPaintMs() < 16.0, "scope: a frame with an overlay draws in under 16 ms");
            const QString shotDir = QString::fromLocal8Bit(qgetenv("ECAM_SHOTS"));
            if (!shotDir.isEmpty()) sw.grab().save(QDir(shotDir).filePath(QStringLiteral("scope_overlay.png")));
            sw.setLanes(scope::DefaultLanes());
        }
        // Trigger on position error rising through 0 (twice a machine cycle in the simulation).
        {
            auto pump = [&](int ms) {
                et.restart();
                while (et.elapsed() < ms) QApplication::processEvents(QEventLoop::AllEvents, 10);
            };
            sw.setWindowSeconds(1);
            sw.setTrigger(ScopeWindow::TriggerMode::Normal, QStringLiteral("mon27"), ScopeWindow::TriggerEdge::Rising, 0.0, 0.2);
            pump(3000);
            const int n = sw.triggerCount();
            const double tt = sw.triggerTime();
            std::printf("     trigger (Normal): %d captures, last at %.4f s\n", n, tt);
            Check(n >= 2 && std::isfinite(tt), "trigger (Normal): captures each rising crossing");
            Check(std::fabs(sw.plotViewEnd() - (tt + 0.8 * 1.0)) < 1e-9, "trigger: the capture sits 20 % into the screen");
            const scope::Series *src = sw.series(QStringLiteral("mon27"));
            bool bracket = false;
            if (src && std::isfinite(tt)) {
                const size_t k = std::lower_bound(src->t.begin(), src->t.end(), tt) - src->t.begin();
                bracket = k > 0 && k < src->t.size() && src->v[k - 1] < 0.0 && src->v[k] >= 0.0;
            }
            Check(bracket, "trigger: the crossing lies between a sample below the level and one above");
            const QString shotDir = QString::fromLocal8Bit(qgetenv("ECAM_SHOTS"));
            if (!shotDir.isEmpty()) sw.grab().save(QDir(shotDir).filePath(QStringLiteral("scope_trigger.png")));

            sw.setTrigger(ScopeWindow::TriggerMode::Single, QStringLiteral("mon27"), ScopeWindow::TriggerEdge::Rising, 0.0, 0.2);
            pump(2500);
            Check(sw.triggerCount() == 1 && !sw.triggerArmed(), "trigger (Single): one capture, then waits for Arm");

            sw.setTrigger(ScopeWindow::TriggerMode::Auto, QStringLiteral("mon27"), ScopeWindow::TriggerEdge::Rising, 1e6, 0.2);
            pump(1500);
            Check(sw.triggerRolling() && sw.triggerCount() == 0, "trigger (Auto): rolls live when nothing triggers");

            sw.setTrigger(ScopeWindow::TriggerMode::Off, QStringLiteral("mon27"), ScopeWindow::TriggerEdge::Rising, 0.0, 0.2);
            sw.setWindowSeconds(5);
        }
        // Every register sample is that register's: no reply answers the wrong read.
        {
            const scope::Series *temp = sw.series(QStringLiteral("reg.temp"));
            const scope::Series *bus = sw.series(QStringLiteral("reg.bus"));
            bool sane = temp && bus && !temp->v.empty() && !bus->v.empty();
            for (double v : temp ? temp->v : std::vector<double>()) sane = sane && v >= 40 && v <= 50;
            for (double v : bus ? bus->v : std::vector<double>()) sane = sane && v >= 70 && v <= 80;
            Check(sane, "scope: each register read gets its own reply (keepalive awaited)");
        }
        const QString shots = QString::fromLocal8Bit(qgetenv("ECAM_SHOTS"));
        if (!shots.isEmpty()) sw.grab().save(QDir(shots).filePath(QStringLiteral("scope.png")));
        // Switch the streamed signal: one stream lane at a time.
        std::vector<scope::Lane> lanes = sw.lanes();
        for (scope::Lane &l : lanes) {
            if (l.key == QLatin1String("mon27")) l.on = false;
            if (l.key == QLatin1String("mon35")) l.on = true;
        }
        sw.setLanes(lanes);
        et.restart();
        while (et.elapsed() < 1200) QApplication::processEvents(QEventLoop::AllEvents, 20);
        const scope::Series *tq = sw.series(QStringLiteral("mon35"));
        Check(tq && tq->t.size() > 300, "scope: restarts the stream on the newly chosen signal");
        // Frames follow the screen, not the data (845 samples/s): about the refresh rate while recording,
        // none once nothing new arrives.
        {
            const int interval = sw.frameIntervalMs();
            const int before = sw.plotFrames();
            et.restart();
            while (et.elapsed() < 1000) QApplication::processEvents(QEventLoop::AllEvents, 5);
            const int fps = sw.plotFrames() - before;
            std::printf("     scope frames: %d in 1 s, timer %d ms (screen %.0f Hz)\n", fps, interval,
                        sw.screen() ? sw.screen()->refreshRate() : 0.0);
            Check(interval >= 4 && interval <= 42, "scope: frame timer from the screen's refresh rate");
            // Data lands ~30 times a second; frames must come well above that and not above the timer. (Windows
            // presents a window every refresh or every other one depending on the desktop, so 120 Hz shows as
            // 125 or 64 here.)
            Check(fps >= 45 && fps <= 1000 / interval + 5, "scope: draws at about the screen rate, not the data rate");
        }
        sw.stopAndDisconnect();
        Check(!sw.recording(), "scope: stops and disconnects");
        // Not connected: Start is disabled, and looks it (a primary button's blue would hide that).
        {
            QPushButton *start = nullptr;
            for (QPushButton *b : sw.findChildren<QPushButton *>()) {
                if (b->text().contains(QStringLiteral("Start"))) start = b;
            }
            QApplication::processEvents();
            Check(start && !start->isEnabled(), "scope: Start disabled while not connected");
            if (start) {
                const QImage img = start->grab().toImage();
                const QColor c = img.pixelColor(img.width() / 2, 3);  // the face, above the text
                std::printf("     Start face while disabled: %s\n", qPrintable(c.name()));
                Check(c.blue() < c.red() + 40, "scope: a disabled Start is not drawn in the primary blue");
            }
        }
        {
            QApplication::processEvents();
            const int before = sw.plotFrames();
            et.restart();
            while (et.elapsed() < 500) QApplication::processEvents(QEventLoop::AllEvents, 5);
            Check(sw.plotFrames() == before, "scope: no frames drawn when nothing changes");
        }
        // Stopped: zoom changes the timebase about the mouse, middle-drag pans back, 0 fits everything.
        {
            QWidget *plot = sw.plotWidget();
            sw.setWindowSeconds(5);
            QApplication::processEvents();
            const double plotW = plot->width() - 230.0 - 12.0;
            const double end0 = sw.plotViewEnd();
            const QPointF mid(230.0 + plotW / 2.0, plot->height() / 3.0);
            const double tMid = end0 - 5.0 + 0.5 * 5.0;
            QWheelEvent zoom(mid, plot->mapToGlobal(mid), QPoint(), QPoint(0, 240), Qt::NoButton, Qt::ControlModifier,
                             Qt::NoScrollPhase, false);
            QApplication::sendEvent(plot, &zoom);
            const double win = sw.windowSeconds();
            const double tMidAfter = sw.plotViewEnd() - win + 0.5 * win;
            std::printf("     scope zoom: 5 s -> %.3f s, time under the mouse %.4f -> %.4f\n", win, tMid, tMidAfter);
            Check(std::fabs(win - 3.2) < 0.01, "scope: Ctrl+wheel zooms the timebase (5 s -> 3.2 s)");
            Check(std::fabs(tMidAfter - tMid) < 1e-6, "scope: zoom keeps the time under the mouse");
            // Middle-drag 100 px right: the view moves back by 100 px worth of time.
            const double endBefore = sw.plotViewEnd();
            QMouseEvent press(QEvent::MouseButtonPress, mid, plot->mapToGlobal(mid), Qt::MiddleButton, Qt::MiddleButton, Qt::NoModifier);
            QApplication::sendEvent(plot, &press);
            const QPointF to = mid + QPointF(100, 0);
            QMouseEvent move(QEvent::MouseMove, to, plot->mapToGlobal(to), Qt::NoButton, Qt::MiddleButton, Qt::NoModifier);
            QApplication::sendEvent(plot, &move);
            QMouseEvent release(QEvent::MouseButtonRelease, to, plot->mapToGlobal(to), Qt::MiddleButton, Qt::NoButton, Qt::NoModifier);
            QApplication::sendEvent(plot, &release);
            const double moved = endBefore - sw.plotViewEnd();
            Check(std::fabs(moved - 100.0 / plotW * win) < 1e-6, "scope: middle-drag pans back in time");
            // 0 (Ctrl+0 in the window): everything recorded, right edge on the newest sample.
            plot->setFocus();
            QKeyEvent fit(QEvent::KeyPress, Qt::Key_0, Qt::NoModifier, QStringLiteral("0"));
            QApplication::sendEvent(plot, &fit);
            const scope::Series *all = sw.series(QStringLiteral("mon35"));
            const double span = all && !all->t.empty() ? all->t.back() - pe->t.front() : 0.0;
            std::printf("     scope fit: %.2f s window over %.2f s recorded\n", sw.windowSeconds(), span);
            Check(sw.windowSeconds() > win && sw.windowSeconds() <= 120.0 && sw.plotViewEnd() >= (all ? all->t.back() : 0.0),
                  "scope: 0 fits everything recorded");
            if (!QString::fromLocal8Bit(qgetenv("ECAM_SHOTS")).isEmpty()) {
                sw.grab().save(QDir(QString::fromLocal8Bit(qgetenv("ECAM_SHOTS"))).filePath(QStringLiteral("scope_fit.png")));
            }
        }
        sw.close();
        // Several motors in one scope: one time axis, lanes per motor, a trigger on any of them.
        {
            ScopeWindow multi;
            multi.resize(1240, 760);
            multi.show();
            const int lanesBefore = static_cast<int>(multi.lanes().size());
            multi.startSimulated(3);
            Check(multi.motorCount() == 3, "scope: three simulated motors connected");
            Check(static_cast<int>(multi.lanes().size()) > lanesBefore * 2, "scope: each motor has its lanes");
            QString fast;
            for (const scope::Lane &l : multi.lanes()) {
                if (l.on && scope::MotorOf(l.key) == 0 && scope::Find(l.key)->source == scope::Source::Stream) fast = l.key;
            }
            Check(!fast.isEmpty(), "scope: the first motor has a fast lane on");
            const QString second = scope::MotorKey(1, fast), third = scope::MotorKey(2, fast);
            int shown = 0, firstAt = -1, secondAt = -1;
            for (const scope::Lane &l : multi.lanes()) {
                if (l.key == fast) firstAt = shown;
                if (l.key == second) secondAt = shown;
                ++shown;
            }
            Check(secondAt == firstAt + 1, "scope: the same signal on the next motor sits beside it");
            QElapsedTimer multiClock;
            multiClock.start();
            while (multiClock.elapsed() < 1500) QApplication::processEvents(QEventLoop::AllEvents, 5);
            const scope::Series *a = multi.series(fast), *b = multi.series(second), *c = multi.series(third);
            Check(a && b && c && !a->t.empty() && !b->t.empty() && !c->t.empty(), "scope: every motor records its own lane");
            if (a && b && c && !a->t.empty() && !b->t.empty() && !c->t.empty()) {
                const double newest = std::max({a->t.back(), b->t.back(), c->t.back()});
                const double oldest = std::min({a->t.back(), b->t.back(), c->t.back()});
                Check(newest - oldest < 0.2, "scope: the motors are on one clock");
                bool ordered = true;
                for (const scope::Series *s : {a, b, c}) {
                    for (size_t i = 1; i < s->t.size(); ++i) ordered = ordered && s->t[i] >= s->t[i - 1];
                }
                Check(ordered, "scope: samples stay in time order");
            }
            Check(multi.statusText().contains(QStringLiteral("3 motors")), "scope: the status says how many are recording");
            multi.setMotorName(1, QStringLiteral("Carriage"));
            QApplication::processEvents();
            const auto trigBoxes = multi.findChildren<QComboBox *>();
            bool named = false;
            for (QComboBox *box : trigBoxes) {
                for (int i = 0; i < box->count(); ++i) named = named || box->itemText(i).startsWith(QStringLiteral("Carriage · "));
            }
            Check(named, "scope: a motor's name shows on its lanes");
            double lo = 1e18, hi = -1e18;
            if (b) {
                for (double v : b->v) {
                    lo = std::min(lo, v);
                    hi = std::max(hi, v);
                }
            }
            multi.setTrigger(ScopeWindow::TriggerMode::Normal, second, ScopeWindow::TriggerEdge::Either, (lo + hi) / 2.0, 0.2);
            multi.setWindowSeconds(1);
            multiClock.restart();
            while (multiClock.elapsed() < 4000 && multi.triggerCount() == 0) QApplication::processEvents(QEventLoop::AllEvents, 5);
            Check(multi.triggerCount() > 0, "scope: triggers on the second motor's lane");
            multi.setTrigger(ScopeWindow::TriggerMode::Off, fast, ScopeWindow::TriggerEdge::Rising, 0.0, 0.2);
            multi.stopAndDisconnect();
            Check(multi.motorCount() == 0, "scope: all motors disconnect");
            Check(multi.series(second) != nullptr, "scope: the recording stays to study after Disconnect");
            multi.startSimulated(1);
            bool extra = false;
            for (const scope::Lane &l : multi.lanes()) extra = extra || scope::MotorOf(l.key) > 0;
            Check(multi.motorCount() == 1 && !extra, "scope: a fresh connect starts from one motor");
            multi.stopAndDisconnect();
        }
        Check(scope::MotorOf(QStringLiteral("mon27")) == 0 && scope::BaseKey(QStringLiteral("m2/mon27")) == QStringLiteral("mon27") &&
                  scope::Find(QStringLiteral("m2/mon27")) == scope::Find(QStringLiteral("mon27")),
              "scope: a lane's key names its motor and its signal");
        userSettings.remove(QStringLiteral("clearpathScope"));
        for (auto it = savedScope.cbegin(); it != savedScope.cend(); ++it) {
            userSettings.setValue(QStringLiteral("clearpathScope/") + it.key(), it.value());
        }
        userSettings.sync();
    }

    // Shortcuts: no two actions in the main window share one (an ambiguous shortcut fires neither).
    {
        MainWindow mw;
        std::map<QString, QString> seen;
        bool clash = false;
        for (QAction *a : mw.findChildren<QAction *>()) {
            // A dialog's own (its F1) acts in its window only.
            const QWidget *owner = qobject_cast<QWidget *>(a->parent());
            if (owner && owner->window() != &mw) continue;
            for (const QKeySequence &k : a->shortcuts()) {
                const QString key = k.toString(QKeySequence::PortableText);
                if (key.isEmpty()) continue;
                const QString what = a->text().isEmpty() ? QStringLiteral("(unnamed)") : a->text();
                if (seen.count(key) && seen[key] != what) {
                    std::printf("     shortcut %s: %s and %s\n", qPrintable(key), qPrintable(seen[key]), qPrintable(what));
                    clash = true;
                }
                seen[key] = what;
            }
        }
        Check(!clash, "main window: every shortcut is used once");
    }
    // (ECAM_UI_FULL) The user guide: every chapter loads with its pictures, the contents lists them, the whole guide prints.
    if (full) {
        HelpWindow *help = HelpWindow::Show(nullptr);
        bool pictures = true, chapters = true;
        for (const HelpWindow::Chapter &c : HelpWindow::Chapters()) {
            help->open(c.file);
            QApplication::processEvents();
            const QString text = help->browser()->toPlainText();
            chapters = chapters && help->currentPage() == c.file && text.size() > 400;
            QFile f(QStringLiteral(":/help/") + c.file);
            f.open(QIODevice::ReadOnly);
            static const QRegularExpression img(QStringLiteral(R"re(src="(images/[^"]+)")re"));
            for (auto it = img.globalMatch(QString::fromUtf8(f.readAll())); it.hasNext();) {
                const QString pic = QStringLiteral(":/help/") + it.next().captured(1);
                if (QImage(pic).isNull()) {
                    std::printf("     missing picture %s in %s\n", qPrintable(pic), qPrintable(c.file));
                    pictures = false;
                }
            }
        }
        Check(chapters, "guide: every chapter opens");
        Check(pictures, "guide: every picture is in the resources");
        help->open(QStringLiteral("04-profile.html#knots"));
        QApplication::processEvents();
        Check(help->currentPage() == QLatin1String("04-profile.html") && help->contentsCurrent() == QLatin1String("Knots"),
              "guide: opens a section, the contents follow");
        help->open(QStringLiteral("10-examples.html"));
        QApplication::processEvents();
        std::printf("     contents after opening the examples: %s\n", qPrintable(help->contentsCurrent()));
        Check(help->contentsCurrent() == QLatin1String("Worked examples"), "guide: the contents follow the page");
        const QString pdf = QDir::temp().filePath(QStringLiteral("ecam_guide_test.pdf"));
        QString err;
        QElapsedTimer pdfTimer;
        pdfTimer.start();
        const bool printed = HelpWindow::SavePdf(pdf, &err);
        std::printf("     guide PDF: %lld KB in %lld ms\n", static_cast<long long>(QFileInfo(pdf).size() / 1024),
                    static_cast<long long>(pdfTimer.elapsed()));
        Check(printed && QFileInfo(pdf).size() > 500 * 1024, "guide: the whole guide prints to PDF, pictures included");
        {
            // Bookmarks: every chapter, its sections under it.
            const auto outline = HelpWindow::PdfOutlineForTest(pdf);
            int chapters = 0, sections = 0;
            for (const auto &o : outline) (o.first == 1 ? chapters : sections) += 1;
            std::printf("     PDF bookmarks: %d chapters, %d sections\n", chapters, sections);
            Check(chapters == HelpWindow::Chapters().size() && sections > 40 && !outline.isEmpty() &&
                      outline.first().second == QLatin1String("Getting started"),
                  "guide: the PDF has bookmarks, chapters with their sections");
            // Links between chapters go to their place in the PDF, not to a chapter's file.
            QFile f(pdf);
            const QByteArray bytes = f.open(QIODevice::ReadOnly) ? f.readAll() : QByteArray();
            Check(!bytes.isEmpty() && !bytes.contains(".html"), "guide: no link in the PDF points at a chapter file");
            Check(bytes.contains("/MediaBox [0 0 612.000000 792.000000]") && !bytes.contains("/MediaBox [0 0 595"),
                  "guide: the PDF is US Letter");
            int tocPages = 0;
            const QImage toc = HelpWindow::PreviewContents(0, 110, &tocPages);
            const QString shot = QDir::temp().filePath(QStringLiteral("ecam_guide_contents.png"));
            toc.save(shot);
            std::printf("     contents: %d page(s), %s\n", tocPages, qPrintable(QDir::toNativeSeparators(shot)));
            Check(tocPages >= 1 && tocPages <= 4, "guide: contents pages after the cover");
        }
        // The print document: pictures attached, laid out on paper-sized pages.
        {
            std::unique_ptr<QTextDocument> doc(HelpWindow::PrintDocument());
            const bool attached = !doc->resource(QTextDocument::ImageResource, QUrl(QStringLiteral("images/overview.png"))).isNull() &&
                                  !doc->resource(QTextDocument::ImageResource, QUrl(QStringLiteral("images/laws.png"))).isNull();
            Check(attached, "guide: pictures attached to the print document");
            // The printed pages as laid out for paper (Letter, the print margins).
            int pages = 0;
            const QImage first = HelpWindow::PreviewPage(0, 110, &pages);
            std::printf("     guide on paper: %d pages\n", pages);
            {
                // The whole guide's cover (the PDF and a printout of it start with it); saved to look at.
                const QImage cover = HelpWindow::PreviewCover(110);
                const QString shot = QDir::temp().filePath(QStringLiteral("ecam_guide_cover.png"));
                cover.save(shot);
                std::printf("     guide cover: %s\n", qPrintable(QDir::toNativeSeparators(shot)));
                // The band at the top is the report's navy; the page below it white.
                Check(!cover.isNull() && cover.pixelColor(cover.width() / 2, cover.height() / 8) == QColor(0x17, 0x37, 0x5e) &&
                          cover.pixelColor(10, cover.height() - 10) == QColor(Qt::white),
                      "guide: a cover page (the navy band, the page white)");
            }
            // A sanity bound against a layout gone wrong; the guide grows with the app (37 pages at 0.9.2).
            Check(pages >= 10 && pages <= 45 && !first.isNull(), "guide: the whole guide lays out on paper");
            const int stranded = HelpWindow::StrandedHeadings();
            std::printf("     headings stranded at a page foot: %d\n", stranded);
            Check(stranded == 0, "guide: no heading left at the foot of a page");
            const QString shotDir = QString::fromLocal8Bit(qgetenv("ECAM_SHOTS"));
            if (!shotDir.isEmpty()) {
                for (int pg : {0, 1, 2, 5, 9, pages - 2, pages - 1}) {
                    HelpWindow::PreviewPage(pg, 110, nullptr).save(QDir(shotDir).filePath(QStringLiteral("print_page%1.png").arg(pg + 1)));
                }
            }
        }
        // On screen: each picture handed over at exactly its shown width times the screen's pixel ratio (no
        // unsmoothed shrinking by the browser).
        {
            const QImage overview = help->browser()->loadResource(QTextDocument::ImageResource, QUrl(QStringLiteral("images/overview.png"))).value<QImage>();
            const QImage panel = help->browser()->loadResource(QTextDocument::ImageResource, QUrl(QStringLiteral("images/crank_panel.png"))).value<QImage>();
            const qreal dpr = help->browser()->devicePixelRatioF();
            std::printf("     on screen: overview %dx%d at dpr %.2f\n", overview.width(), overview.height(), dpr);
            Check(overview.width() == qRound(820 * dpr) && overview.devicePixelRatio() == dpr &&
                      panel.width() == qRound(760 * dpr),
                  "guide: pictures scaled smoothly to their shown size on screen");
        }
        const QString shotDir = QString::fromLocal8Bit(qgetenv("ECAM_SHOTS"));
        if (!shotDir.isEmpty()) {
            help->resize(1180, 860);
            help->open(QStringLiteral("10-examples.html"));
            QApplication::processEvents();
            help->grab().save(QDir(shotDir).filePath(QStringLiteral("guide.png")));
            QFile::copy(pdf, QDir(shotDir).filePath(QStringLiteral("guide.pdf")));
        }
        help->close();
    }

    // Axis > Import points as spline: the table fills, one undo step puts the axis back.
    {
        const int rows = table->rowCount();
        const int steps = undo->index();
        QString msg;
        Check(win.ImportPoints(QStringLiteral(ECAM_SOURCE_DIR "/samples/spline_points.csv"), &msg), "import points");
        QApplication::processEvents();
        std::printf("     import: %s\n", qPrintable(msg));
        Check(table->rowCount() == 18, "import: a row per piece");
        auto *law = qobject_cast<QComboBox *>(table->cellWidget(9, 1));
        Check(law && law->currentText() == QStringLiteral("Cubic spline"), "import: the rows are Cubic spline");
        Check(undo->index() == steps + 1, "import: one undo step");
        undo->undo();
        QApplication::processEvents();
        Check(table->rowCount() == rows, "import: undone");
        Check(!win.ImportPoints(QStringLiteral(ECAM_SOURCE_DIR "/README.md"), &msg), "import: not a table of points");
        Check(!win.ImportPoints(QStringLiteral(ECAM_SOURCE_DIR "/no_such_file.csv"), &msg), "import: no such file");
    }

    // Axis type: an electronic gear in the panel.
    {
        auto *type = win.findChild<QComboBox *>(QStringLiteral("axisType"));
        auto *num = win.findChild<QSpinBox *>(QStringLiteral("axisGearNum"));
        auto *den = win.findChild<QSpinBox *>(QStringLiteral("axisGearDen"));
        Check(type && num && den, "gear: the panel's fields");
        if (type && num && den) {
            const int start = undo->index();
            const int rows = table->rowCount();
            type->setCurrentIndex(1);
            QApplication::processEvents();
            Check(table->rowCount() == 1, "gear: one row");
            Check(strip->locked(), "gear: the diagram can't be edited");
            Check(table->editTriggers() == QAbstractItemView::NoEditTriggers, "gear: nor the table");
            auto *law = qobject_cast<QComboBox *>(table->cellWidget(0, 1));
            Check(law && !law->isEnabled() && law->currentText() == QStringLiteral("Linear"), "gear: its law shown, fixed");
            num->setValue(3);
            den->setValue(2);
            QApplication::processEvents();
            Check(Near(Cell(table, 0, 6), 1.5, 1e-9), "gear: the table shows the ratio");
            QString msg;
            Check(!win.ImportPoints(QStringLiteral(ECAM_SOURCE_DIR "/samples/spline_points.csv"), &msg), "gear: no points imported");
            num->setValue(-3);
            QApplication::processEvents();
            Check(Near(Cell(table, 0, 6), -1.5, 1e-9), "gear: negative reverses");
            while (undo->index() > start) undo->undo();
            QApplication::processEvents();
            Check(table->rowCount() == rows && !strip->locked() && type->currentIndex() == 0, "gear: undone, the diagram back");
        }
        // Files and uploads.
        ecam::CamProject p;
        QString err;
        Check(io::LoadProject(QStringLiteral(ECAM_SOURCE_DIR "/samples/bench_gear.ecam"), &p, &err), "gear: sample loads");
        Check(p.axes[0].gear && p.axes[0].gearNum == 3 && p.axes[0].gearDen == 2 && p.axes[0].segments.size() == 1, "gear: sample's ratio");
        bool line = false;
        for (const std::string &l : io::UploadLines(p)) line = line || l.find("gear=3:2") != std::string::npos;
        Check(line, "gear: in the upload");
        const QString tmp = QDir::temp().filePath(QStringLiteral("ecam_gear_test.ecam"));
        Check(io::SaveProject(p, tmp, &err), "gear: saves");
        ecam::CamProject q;
        Check(io::LoadProject(tmp, &q, &err) && q.axes[0].gear && q.axes[0].gearNum == 3 && q.axes[0].gearDen == 2, "gear: round trip");
        QFile::remove(tmp);
        bool exact = false;
        ecam::CamProject up;
        QStringList lines;
        for (const std::string &l : io::UploadLines(p)) lines << QString::fromStdString(l);
        Check(io::ProjectFromUpload(lines, &up, &exact, &err) && exact && up.axes[0].gear && up.axes[0].gearNum == 3, "gear: back from the board");
        lines = lines.filter(QRegularExpression(QStringLiteral("^(?!json )")));
        Check(io::ProjectFromUpload(lines, &up, &exact, &err) && !exact && up.axes[0].gear && up.axes[0].gearDen == 2, "gear: from the motion lines alone");
    }

    // View menu: Shift+F9 shows or hides the Cam / Timing pane; Ctrl+T, the Timing view for a crank axis, only does
    // something (and is only enabled) with crank and timeline axes, and opens the pane if it is folded away.
    {
        MainWindow w;
        w.resize(1920, 1040);
        w.show();
        QApplication::processEvents();
        auto *pane = w.findChild<QAction *>(QStringLiteral("ECamViewPaneAction"));
        auto *force = w.findChild<QAction *>(QStringLiteral("ECamTimingAction"));
        auto *timing = w.findChild<TimingView *>();
        Check(pane && force && timing && pane->text() == QStringLiteral("Cam / Timing view"), "view menu: the pane's item is named for both views");
        w.OpenFile(QStringLiteral(ECAM_SOURCE_DIR "/samples/bench_timeline.ecam"));
        QApplication::processEvents();
        Check(force && !force->isEnabled() && timing->isVisible(), "view menu: a timeline-only job shows the Timing view, Ctrl+T greyed");
        w.OpenFile(QStringLiteral(ECAM_SOURCE_DIR "/samples/mixed_timeline.ecam"));
        QApplication::processEvents();
        Check(force && force->isEnabled(), "view menu: crank and timeline axes: Ctrl+T enabled");
        if (pane && force && timing) {
            w.ShowAxis(0, 0.0);  // mixed_timeline's first axis is on the crank
            pane->setChecked(false);
            QApplication::processEvents();
            const bool camFirst = !timing->isVisible();
            force->setChecked(true);
            QApplication::processEvents();
            Check(camFirst && pane->isChecked() && timing->isVisible(), "view menu: Ctrl+T on a crank axis opens the pane on the Timing view");
            force->setChecked(false);
            w.OpenFile(QStringLiteral(ECAM_SOURCE_DIR "/samples/bench_timeline.ecam"));
            QApplication::processEvents();
            Check(!force->isChecked(), "view menu: a job without both kinds unticks it");
        }
    }

    // Motor direction on the pages.
    {
        MainWindow w;
        w.resize(1920, 1040);
        w.show();
        QApplication::processEvents();
        auto *axisBox = w.findChild<QCheckBox *>(QStringLiteral("ECamAxisReverse"));
        auto *crankBox = w.findChild<QCheckBox *>(QStringLiteral("ECamCrankReverse"));
        Check(axisBox && crankBox && !axisBox->isChecked() && !crankBox->isChecked(), "direction: the boxes exist, off by default");
        if (axisBox) {
            axisBox->setChecked(true);
            QApplication::processEvents();
            Check(axisBox->isChecked(), "direction: an axis can be reversed");
        }
    }

    // CCIO-8 inputs, without a board.
    using namespace ecam;
    {
        CamProject p = DefaultProject();
        const QByteArray plain = io::ProjectToJson(p, false);
        Check(!plain.contains("ccio"), "ccio: a project without boards writes nothing, so its hash stays");
        p.axes.push_back(DefaultAxis("Axis 2"));
        p.axes.push_back(DefaultAxis("Axis 3"));
        p.ccio.boards = 2;
        SetCcioInput(p.ccio, {1, 3, "X home", CcioRole::Home, 2});
        SetCcioInput(p.ccio, {0, 7, "X limit", CcioRole::LimitNegative, 1});
        CamProject back;
        QString error;
        Check(io::ProjectFromJson(io::ProjectToJson(p, false), &back, &error) && back.ccio.boards == 2 && back.ccio.inputs.size() == 2 &&
                  CcioInputAt(back.ccio, 1, 3).name == "X home" && CcioInputAt(back.ccio, 1, 3).role == CcioRole::Home &&
                  CcioInputAt(back.ccio, 1, 3).axis == 2 && CcioInputAt(back.ccio, 0, 7).role == CcioRole::LimitNegative,
              "ccio: the settings survive saving and loading");
        Check(CcioPinName(0, 0) == "CCIOA0" && CcioPinName(7, 7) == "CCIOH7", "ccio: connectors named as in the library, up to the eighth board");
        CcioConfig c = p.ccio;
        CcioAxisRemoved(c, 1);
        Check(CcioInputAt(c, 0, 7).axis == -1 && CcioInputAt(c, 1, 3).axis == 1, "ccio: removing an axis keeps inputs on the right axes");
        SetCcioInput(c, {1, 3, "", CcioRole::None, -1});
        Check(c.inputs.size() == 1, "ccio: a blank input isn't stored");
        CamProject fewer = p;
        fewer.ccio.boards = 1;
        NormalizeProject(fewer);
        Check(fewer.ccio.inputs.size() == 1 && fewer.ccio.inputs[0].board == 0, "ccio: fewer boards drops the settings of the ones removed");
        fewer.ccio.boards = 99;
        NormalizeProject(fewer);
        Check(fewer.ccio.boards == kCcioMaxBoards, "ccio: at most 8 boards");
        CamProject dup = p;
        SetCcioInput(dup.ccio, {0, 0, "", CcioRole::Home, 2});
        bool found = false;
        for (const auto &k : review::AnalyzeProject(dup).checks) found = found || (k.level == review::Check::Error && k.text.contains(QStringLiteral("Home is on both")));
        Check(found, "ccio: review: one job on two inputs is an error");
        CamProject none = p;
        SetCcioInput(none.ccio, {0, 1, "", CcioRole::LimitPositive, -1});
        found = false;
        for (const auto &k : review::AnalyzeProject(none).checks) found = found || (k.level == review::Check::Warning && k.text.contains(QStringLiteral("of no axis")));
        Check(found, "ccio: review: a job with no axis is a warning");

        MainWindow w;
        w.resize(1920, 1040);
        w.show();
        QApplication::processEvents();
        auto *tree = w.findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"));
        const auto nodes = tree->findItems(QStringLiteral("CCIO-8"), Qt::MatchExactly | Qt::MatchRecursive);
        Check(nodes.size() == 1 && nodes.first()->childCount() == 0, "ccio: the tree has its node, no boards at first");
        if (!nodes.isEmpty()) {
            tree->setCurrentItem(nodes.first());
            auto *boards = w.findChild<QSpinBox *>(QStringLiteral("ECamCcioBoards"));
            auto *table = w.findChild<QTableWidget *>(QStringLiteral("ECamCcioTable"));
            auto *heading = w.findChild<QLabel *>(QStringLiteral("ECamCcioHeading"));
            Check(boards && table && boards->isVisible() && !table->isVisible(), "ccio: the page opens, no table without boards");
            boards->setValue(8);
            QApplication::processEvents();
            Check(nodes.first()->childCount() == 8 && table->isVisible() && table->rowCount() == 8, "ccio: eight boards, eight nodes, eight inputs each");
            tree->setCurrentItem(nodes.first()->child(5));
            QApplication::processEvents();
            Check(heading->text().startsWith(QStringLiteral("Board 6 of 8")) && table->item(0, 0)->text() == QStringLiteral("CCIOF0"), "ccio: a board's node shows its inputs");
            table->item(2, 2)->setText(QStringLiteral("Y home"));
            qobject_cast<QComboBox *>(table->cellWidget(2, 3))->setCurrentIndex(1);
            QApplication::processEvents();
            tree->setCurrentItem(nodes.first()->child(0));
            QApplication::processEvents();
            Check(table->item(2, 2)->text().isEmpty(), "ccio: another board has its own inputs");
            tree->setCurrentItem(nodes.first()->child(5));
            QApplication::processEvents();
            Check(table->item(2, 2)->text() == QStringLiteral("Y home") && qobject_cast<QComboBox *>(table->cellWidget(2, 3))->currentIndex() == 1 &&
                      qobject_cast<QComboBox *>(table->cellWidget(2, 4))->currentIndex() == 0,
                  "ccio: edits stay with their input, a function takes the axis");
            boards->setValue(2);
            QApplication::processEvents();
            Check(nodes.first()->childCount() == 2, "ccio: lowering the count removes the nodes");
            w.setWindowModified(false);
        }
    }

    // Firmware update, without a board.
    {
        Check(fw::IsClearCore(0x2890, 0x8022) && fw::IsClearCore(0x2890, 0x0022), "firmware: a ClearCore is known by both its USB ids");
        Check(!fw::IsClearCore(0x2890, 0x0201) && !fw::IsClearCore(0x1234, 0x8022), "firmware: a ClearPath motor on USB is not a ClearCore");
        QByteArray image(4096, '\0');
        image[3] = 0x20;
        image.replace(2000, 19, "ECam_Firmware 1.2.3");
        Check(fw::CheckImage(image).isEmpty() && fw::VersionInImage(image) == QStringLiteral("1.2.3"), "firmware: reads the version from an image");
        Check(!fw::CheckImage(QByteArray(100, 'x')).isEmpty(), "firmware: refuses a short file");
        Check(!fw::CheckImage(QByteArray(4096, 'x')).isEmpty(), "firmware: refuses a file that isn't an image");
        QByteArray big(0x74001, '\0');
        big[3] = 0x20;
        Check(!fw::CheckImage(big).isEmpty(), "firmware: refuses an image that reaches the stored profile");
        const fw::Package pkg = fw::FindPackage();
        Check(pkg.ok && pkg.version == QStringLiteral(ECAM_PRODUCT_VERSION), "firmware: the app finds the firmware it was built with");

        QTemporaryDir dir;
        qputenv("ECAM_FIRMWARE_BACKUPS", dir.path().toUtf8());
        fw::Backup b;
        b.lines = {QStringLiteral("axes 1"), QStringLiteral("json {\"a\":1}")};
        b.hash = ecam::ProfileHash({"axes 1", "json {\"a\":1}"});
        b.serial = QStringLiteral("ABC");
        b.firmware = QStringLiteral("0.8.4");
        QString error;
        Check(fw::SaveBackup(&b, &error), "firmware: saves a backup");
        fw::Backup back;
        Check(fw::LoadBackup(b.file, &back, &error) && back.lines == b.lines && back.hash == b.hash && back.serial == b.serial,
              "firmware: a backup reads back exactly");
        Check(fw::PendingBackups().size() == 1, "firmware: an unfinished update's backup is found");
        QFile f(b.file);
        f.open(QIODevice::Append);
        f.write("extra line\n");
        f.close();
        Check(!fw::LoadBackup(b.file, &back, &error) && fw::PendingBackups().isEmpty(), "firmware: a damaged backup is refused");
        qunsetenv("ECAM_FIRMWARE_BACKUPS");

        FirmwareDialog d;
        d.show();
        QApplication::processEvents();
        Check(!d.busy() && !d.finishedOk(), "firmware: the update window opens without doing anything");
        auto *update = d.findChild<QPushButton *>(QStringLiteral("ECamFwUpdate"));
        Check(update && !update->isEnabled(), "firmware: Update waits until the board has been read");
        // Themed by itself, like the other windows: no light background or white log.
        const QImage shot = d.grab().toImage();
        auto *log = d.findChild<QPlainTextEdit *>(QStringLiteral("ECamFwLog"));
        const QPoint inLog = log->mapTo(&d, log->rect().center());
        Check(shot.pixelColor(4, 4).lightness() < 70 && shot.pixelColor(inLog).lightness() < 70, "firmware: the update window has the dark theme");
    }

    // Axis jog: the board's "axjog" lines, the panel with no board, and its cards for all four motors at 1080p.
    {
        JogDialog::MotorInfo a;
        Check(JogDialog::ParseMotorLine(QStringLiteral("axjog 1 role=axis axis=0 on=1 moving=0 pos=-800 cam=1600 off=-2400 cpu=80 vmax=666.66 units=mm name=Saw plunge"), &a) &&
                  a.motor == 1 && a.role == JogDialog::Role::Axis && a.axis == 0 && a.on && !a.moving && a.pos == -800 && a.hasCam &&
                  a.cam == 1600 && a.off == -2400 && a.cpu == 80.0 && a.units == QStringLiteral("mm") && a.name == QStringLiteral("Saw plunge"),
              "axis jog: an axis's line, a name with spaces");
        Check(JogDialog::ParseMotorLine(QStringLiteral("axjog 3 role=free axis=- on=0 moving=1 pos=-5 cam=- off=0 cpu=1 vmax=53333 units=counts name="), &a) &&
                  a.role == JogDialog::Role::Free && a.axis == -1 && !a.hasCam && a.moving && a.motor == 3 && a.name.isEmpty(),
              "axis jog: a motor the profile doesn't use");
        Check(JogDialog::ParseMotorLine(QStringLiteral("axjog 2 role=crank axis=- on=1 moving=0 pos=123 cam=- off=0 cpu=1 vmax=0 units=counts name=Crank"), &a) &&
                  a.role == JogDialog::Role::Crank,
              "axis jog: the crank motor");
        Check(!JogDialog::ParseMotorLine(QStringLiteral("ok"), &a) && !JogDialog::ParseMotorLine(QStringLiteral("axjog 0 role=axis"), &a) &&
                  !JogDialog::ParseMotorLine(QStringLiteral("axis 0 master=crank m=1 cpu=800"), &a),
              "axis jog: other lines (a profile's axis line too) are not motors");
        BoardLink none;
        {
            JogDialog d(&none);
            d.show();
            QApplication::processEvents();
            auto *banner = d.findChild<QLabel *>(QStringLiteral("AxisJogBanner"));
            Check(banner && banner->isVisible() && banner->text().contains(QStringLiteral("Not connected")), "axis jog: says when there is no board");
            Check(d.motors().isEmpty() && !d.camActive(), "axis jog: no cards without a board");
            d.pressJog(0, 1);
            d.releaseJog();
            Check(d.findChildren<QPushButton *>(QStringLiteral("AxisJogPlus0")).isEmpty(), "axis jog: nothing to press without a board");
            d.reject();
        }
        {
            // Two axes, a driven crank and a free motor: what a flying saw with a spare connector reports.
            QVector<JogDialog::MotorInfo> ms;
            const QStringList lines = {
                QStringLiteral("axjog 0 role=axis axis=0 on=1 moving=0 pos=123456 cam=0 off=123456 cpu=80 vmax=666.66 units=mm name=Carriage"),
                QStringLiteral("axjog 1 role=axis axis=1 on=0 moving=0 pos=0 cam=0 off=0 cpu=160 vmax=333.33 units=mm name=Saw plunge"),
                QStringLiteral("axjog 2 role=crank axis=- on=0 moving=0 pos=0 cam=- off=0 cpu=1 vmax=0 units=counts name=Crank"),
                QStringLiteral("axjog 3 role=free axis=- on=0 moving=0 pos=-4000 cam=- off=0 cpu=1 vmax=53333 units=counts name=")};
            for (const QString &l : lines) {
                JogDialog::MotorInfo m;
                if (JogDialog::ParseMotorLine(l, &m)) ms << m;
            }
            JogDialog d(&none);
            d.showMotorsForTest(ms);
            d.show();
            QApplication::processEvents();
            d.adjustSize();
            QApplication::processEvents();
            const QSize size = d.size();
            std::printf("     axis jog panel %dx%d\n", size.width(), size.height());
            Check(size.width() <= 1000 && size.height() <= 900, "axis jog: four cards fit a 1080p screen with room");
            Check(d.findChildren<QFrame *>(QStringLiteral("AxisJogCard")).size() == 4, "axis jog: a card for every motor");
            auto *pos3 = d.findChild<QLabel *>(QStringLiteral("AxisJogPos3"));
            Check(pos3 && pos3->text().startsWith(QStringLiteral("-5.000 rev")), "axis jog: a free motor reads in revolutions (800 counts/rev)");
            auto *plus2 = d.findChild<QPushButton *>(QStringLiteral("AxisJogPlus2"));
            Check(plus2 && !plus2->isVisibleTo(&d), "axis jog: the crank motor has no jog buttons");
            auto *cam3 = d.findChild<QPushButton *>(QStringLiteral("AxisJogToCam3"));
            Check(cam3 && !cam3->isVisibleTo(&d), "axis jog: a free motor has no cam position");
            auto *flag0 = d.findChild<QLabel *>(QStringLiteral("AxisJogFlag0"));
            Check(flag0 && flag0->isVisibleTo(&d) && flag0->text().contains(QStringLiteral("off its cam position")), "axis jog: says an axis is off its cam position");
            // Nothing cut off: every button and readout gets the width its text needs.
            QStringList clipped;
            for (QPushButton *b : d.findChildren<QPushButton *>()) {
                if (b->isVisibleTo(&d) && b->width() < b->fontMetrics().horizontalAdvance(b->text()) + 12) clipped << b->text();
            }
            for (QAbstractSpinBox *sb : d.findChildren<QAbstractSpinBox *>()) {
                if (sb->isVisibleTo(&d) && sb->width() < sb->sizeHint().width()) clipped << sb->objectName();
            }
            for (QLabel *l : d.findChildren<QLabel *>()) {
                if (l->isVisibleTo(&d) && !l->wordWrap() && l->width() < l->sizeHint().width()) clipped << l->text();
            }
            Check(clipped.isEmpty(), QStringLiteral("axis jog: nothing cut off (%1)").arg(clipped.join(QStringLiteral(", "))).toUtf8().constData());
            // Help is in the guide, not on the panel: no paragraph of instructions, and Help opens the guide there.
            bool paragraph = false;
            for (QLabel *l : d.findChildren<QLabel *>()) {
                paragraph = paragraph || (l->isVisibleTo(&d) && l->text().size() > 60);
            }
            Check(!paragraph, "axis jog: no paragraphs of instructions on the panel");
            auto *helpButton = d.findChild<QPushButton *>(QStringLiteral("AxisJogHelp"));
            Check(helpButton != nullptr, "axis jog: a Help button");
            if (helpButton) {
                helpButton->click();
                QApplication::processEvents();
                HelpWindow *hw = nullptr;
                for (QWidget *tw : QApplication::topLevelWidgets()) {
                    if (auto *h = qobject_cast<HelpWindow *>(tw); h && h->isVisible()) hw = h;
                }
                Check(hw && hw->currentPage() == QStringLiteral("09-board.html"), "axis jog: Help opens the Board chapter");
                if (hw) hw->close();
            }
            const QString shotPath = QDir::temp().filePath(QStringLiteral("ecam_axisjog_cards.png"));
            d.grab().save(shotPath);
            std::printf("     %s\n", qPrintable(shotPath));
            const QImage shot = d.grab().toImage();
            Check(shot.pixelColor(4, 4).lightness() < 70, "axis jog: the panel has the dark theme");
            d.reject();
        }
        MainWindow w;
        w.show();
        QApplication::processEvents();
        auto *open = w.findChild<QPushButton *>(QStringLiteral("ECamAxisJog"));
        Check(open && !open->isEnabled(), "axis jog: the Board page's Axis jog waits for a board");
        auto *estop = w.findChild<QComboBox *>(QStringLiteral("ECamEstopMode"));
        Check(estop && estop->count() == 3 && !estop->isEnabled(), "estop: the Board page's Estop setting, waiting for a board");
    }

    std::printf(g_fail ? "%d check(s) FAILED\n" : "all checks passed\n", g_fail);
    return g_fail ? 1 : 0;
}
