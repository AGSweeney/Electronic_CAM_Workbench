/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#include "Brand.h"
#include "MainWindow.h"
#include "ProfileText.h"
#include "ProjectIo.h"
#include "StartupUi.h"
#include "Theme.h"
#include "ProjectReport.h"
#include "WorkbookExport.h"

#include <QApplication>
#include <QFile>
#include <QFileInfo>
#include <QIcon>
#include <QPixmap>
#include <QSplashScreen>
#include <QTimer>
#include <QTreeWidget>
#include <QElapsedTimer>

#include <algorithm>

// ECamWorkbench [project.ecam] [--axis n] [--angle deg] [--board ip|COMn] [--screenshot out.png] [--size WxH] [--page node]
// ECamWorkbench --profile-lines project.ecam out.txt   writes the board upload (load ... commit) and exits
// ECamWorkbench --cam-dxf project.ecam <axis> out.dxf   writes the plate cam of a crank axis and exits
// ECamWorkbench --workbook project.ecam out.xlsx [samples]   writes the Excel workbook and exits
// --screenshot renders the window once (--size WxH, default 1480x940), saves it and exits; --page opens that tree node first.
// The splash shows while loading; with no project on the command line the start page follows
// (unless turned off on it, or --no-start).
int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    app.setStyleSheet(theme::AppSheet());
    app.setPalette(theme::AccentPalette(app.palette()));  // the app's blue, not the Windows accent
    // Settings are read with explicit QSettings("ECamUI", "ECamUI") names, so renaming the
    // application here does not move them.
    // No setApplicationDisplayName: Qt would append it to window titles that do not end with it, and
    // the window title already leads with the product name.
    app.setApplicationName(QStringLiteral(ECAM_PRODUCT_NAME));
    app.setOrganizationName("ECamUI");
    app.setWindowIcon(QIcon(":/icons/ecam_app_icon.svg"));

    {
        const QStringList a = app.arguments();
        const int i = a.indexOf("--profile-lines");
        if (i >= 0 && i + 2 < a.size()) {
            ecam::CamProject p;
            QString err;
            if (!io::LoadProject(a[i + 1], &p, &err)) return 2;
            QFile f(a[i + 2]);
            if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return 3;
            const auto lines = io::UploadLines(p);
            f.write("load\n");
            for (const std::string &l : lines) f.write((l + "\n").c_str());
            f.write("commit\n");
            f.write(QStringLiteral("# hash %1\n").arg(ecam::ProfileHash(lines), 8, 16, QChar('0')).toUtf8());
            return 0;
        }
    }

    {
        // ECamWorkbench --cam-dxf project.ecam <axis> out.dxf   (default export options)
        const QStringList a = app.arguments();
        const int i = a.indexOf("--cam-dxf");
        if (i >= 0 && i + 3 < a.size()) {
            ecam::CamProject p;
            QString err;
            if (!io::LoadProject(a[i + 1], &p, &err)) return 2;
            const int axis = a[i + 2].toInt();
            if (axis < 0 || axis >= int(p.axes.size())) return 3;
            return io::ExportCamDxf(p, axis, io::CamDxfOptions(), a[i + 3], &err) ? 0 : 4;
        }
    }

    {
        // ECamWorkbench --workbook project.ecam out.xlsx [samples per cycle, default 360]
        const QStringList a = app.arguments();
        const int i = a.indexOf("--workbook");
        if (i >= 0 && i + 2 < a.size()) {
            ecam::CamProject p;
            QString err;
            if (!io::LoadProject(a[i + 1], &p, &err)) return 2;
            io::WorkbookOptions o;
            o.projectName = QFileInfo(a[i + 1]).completeBaseName();
            o.projectPath = QFileInfo(a[i + 1]).absoluteFilePath();
            if (i + 3 < a.size() && a[i + 3].toInt() > 0) o.samples = a[i + 3].toInt();
            return io::ExportWorkbook(p, o, a[i + 2], &err) ? 0 : 4;
        }
        const int r = a.indexOf("--report");
        if (r >= 0 && r + 2 < a.size()) {
            ecam::CamProject p;
            QString err;
            if (!io::LoadProject(a[r + 1], &p, &err)) return 2;
            io::ReportOptions o;
            o.projectName = QFileInfo(a[r + 1]).completeBaseName();
            o.projectPath = QFileInfo(a[r + 1]).absoluteFilePath();
            return io::ExportReport(p, o, a[r + 2], &err) ? 0 : 4;
        }
    }

    QString project, screenshot, board, page;
    QSize size(1480, 940);  // screenshot window size
    int axis = 0;
    double angle = 0.0;
    bool noStart = false;
    const QStringList args = app.arguments();
    for (int i = 1; i < args.size(); ++i) {
        if (args[i] == "--no-start") {
            noStart = true;
        } else if (args[i] == "--screenshot" && i + 1 < args.size()) {
            screenshot = args[++i];
        } else if (args[i] == "--board" && i + 1 < args.size()) {
            board = args[++i];
        } else if (args[i] == "--page" && i + 1 < args.size()) {
            page = args[++i];
        } else if (args[i] == "--axis" && i + 1 < args.size()) {
            axis = args[++i].toInt();
        } else if (args[i] == "--angle" && i + 1 < args.size()) {
            angle = args[++i].toDouble();
        } else if (args[i] == "--size" && i + 1 < args.size()) {
            const QStringList wh = args[++i].split(QLatin1Char('x'));
            if (wh.size() == 2 && wh[0].toInt() > 0 && wh[1].toInt() > 0) size = QSize(wh[0].toInt(), wh[1].toInt());
        } else {
            project = args[i];
        }
    }

    if (!screenshot.isEmpty()) {  // scripted: no splash, no start page
        MainWindow window;
        window.resize(size);
        window.show();
        if (!project.isEmpty()) window.OpenFile(project);
        if (axis > 0 || angle != 0.0) window.ShowAxis(axis, angle);
        if (!board.isEmpty()) window.ConnectBoard(board);
        if (!page.isEmpty()) {
            auto *tree = window.findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"));
            const auto nodes = tree->findItems(page, Qt::MatchStartsWith | Qt::MatchRecursive);
            if (!nodes.isEmpty()) tree->setCurrentItem(nodes.first());
        }
        QTimer::singleShot(board.isEmpty() ? 400 : 1500, &window, [&]() {
            window.grab().save(screenshot);
            app.exit(0);
        });
        return app.exec();
    }

    // The splash stays up while the window is built and for at least kSplashMs in all.
    QElapsedTimer shown;
    shown.start();
    QSplashScreen *splash = startup::ShowSplash();
    MainWindow window;
    const bool startPage = project.isEmpty() && !noStart && startup::ShowStartPageAtStartup();
    QTimer::singleShot(std::max<qint64>(0, startup::kSplashMs - shown.elapsed()), &window, [&]() {
        window.show();
        splash->finish(&window);
        splash->deleteLater();
        if (!project.isEmpty()) window.OpenFile(project);
        if (axis > 0 || angle != 0.0) window.ShowAxis(axis, angle);
        if (!board.isEmpty()) window.ConnectBoard(board);
        if (startPage) QTimer::singleShot(0, &window, [&]() { window.ShowStartPage(); });
    });
    return app.exec();
}
