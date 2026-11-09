/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
// Makes the user guide's pictures (resources/help/images) from the real windows: the start page, the
// setup wizard, the main window on the bundled samples and templates (whole, with numbered callouts, and
// each panel cropped), the motion laws compared, and the ClearPath scope on its simulated motor.
//
//   help_images <output dir>      (ECAM_SAMPLES_DIR: the samples' folder, else the source tree's)
//
// Run it again whenever the UI changes. It keeps the user's own settings aside and puts them back.
// Each picture is saved at up to twice its size in the guide, so it stays sharp on high-DPI screens.

#include "CamLaws.h"
#include "JogDialog.h"
#include "OperatorDialog.h"
#include "ProjectIo.h"
#include "SwitchWizard.h"
#include "CamTemplates.h"
#include "CollapsibleGroup.h"
#include "MainWindow.h"
#include "ProfileStrip.h"
#include "ScopeWindow.h"
#include "SetupWizard.h"
#include "SideDock.h"
#include "TimingView.h"
#include "StartupUi.h"
#include "Theme.h"

#include <QApplication>
#include <QDir>
#include <QFile>
#include <QElapsedTimer>
#include <QGroupBox>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QSettings>
#include <QSplitter>
#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>

#include <cmath>
#include <cstdio>
#include <functional>
#include <map>

namespace {

QDir g_out;

void Pump(int ms) {
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < ms) QApplication::processEvents(QEventLoop::AllEvents, 10);
}

// Save at up to twice the width it is shown at in the guide.
void Save(const QPixmap &pm, const QString &name, int shownWidth) {
    QImage img = pm.toImage();
    const int maxW = shownWidth * 2;
    if (img.width() > maxW) img = img.scaledToWidth(maxW, Qt::SmoothTransformation);
    img.setDevicePixelRatio(1.0);
    img.save(g_out.filePath(name));
    std::printf("%-28s shown %4d px, saved %dx%d\n", qPrintable(name), shownWidth, img.width(), img.height());
}

void Shot(QWidget *w, const QString &name, int shownWidth) {
    if (!w) {
        std::printf("MISSING %s\n", qPrintable(name));
        return;
    }
    // From its window, for the dark background a widget doesn't paint itself.
    QWidget *top = w->window();
    Save(top == w ? w->grab() : top->grab(QRect(w->mapTo(top, QPoint(0, 0)), w->size())), name, shownWidth);
}

CollapsibleGroup *Group(QWidget *root, const QString &heading) {
    for (CollapsibleGroup *g : root->findChildren<CollapsibleGroup *>()) {
        if (g->heading().startsWith(heading)) return g;
    }
    return nullptr;
}

QGroupBox *Box(QWidget *root, const QString &title) {
    for (QGroupBox *g : root->findChildren<QGroupBox *>()) {
        if (g->title().startsWith(title)) return g;
    }
    return nullptr;
}

// Give the displacement diagram `height` px of the main window's vertical split.
void DiagramHeight(QWidget *mw, int height) {
    QGroupBox *diagram = nullptr;
    for (QGroupBox *g : mw->findChildren<QGroupBox *>()) {
        if (g->title().startsWith(QStringLiteral("Displacement"))) diagram = g;
    }
    for (QSplitter *sp : mw->findChildren<QSplitter *>()) {
        if (sp->orientation() != Qt::Vertical || !diagram) continue;
        const int i = sp->indexOf(diagram);
        if (i < 0) continue;
        QList<int> sizes = sp->sizes();
        int total = 0;
        for (int v : sizes) total += v;
        for (int k = 0; k < sizes.size(); ++k) sizes[k] = k == i ? height : (total - height) / std::max(1, static_cast<int>(sizes.size()) - 1);
        sp->setSizes(sizes);
    }
}

// Opens a page of the detail pane from the project tree (a page only draws while it's open).
void OpenPage(QWidget *mw, const QString &node) {
    auto *tree = mw->findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"));
    const auto items = tree->findItems(node, Qt::MatchStartsWith | Qt::MatchRecursive);
    for (QTreeWidgetItem *it : items) {
        if (!it->data(0, Qt::UserRole).isValid()) continue;
        tree->setCurrentItem(it);
        break;
    }
    Pump(200);
}

// The page of the selected axis.
void OpenAxisPage(QWidget *mw) {
    auto *tree = mw->findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"));
    tree->setCurrentItem(tree->topLevelItem(0)->child(1)->child(0));
    Pump(200);
}

QWidget *CamDock(QWidget *mw) { return mw->findChild<QWidget *>(QStringLiteral("ECamCamDock")); }

void OpenAllPanels(QWidget *root) {
    for (CollapsibleGroup *g : root->findChildren<CollapsibleGroup *>()) g->setCollapsed(false);
    for (SideDock *d : root->findChildren<SideDock *>()) d->setCollapsed(false);
}

// The main window with numbered callouts on its parts.
void Overview(MainWindow &mw, const QString &name) {
    QPixmap pm = mw.grab();
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    struct Part {
        QWidget *w;
        int n;
    };
    const Part parts[] = {
        {mw.findChild<QWidget *>(QStringLiteral("ECamProjectDock")), 1}, {mw.findChild<QWidget *>(QStringLiteral("ECamDetails")), 2},
        {CamDock(&mw), 3}, {Box(&mw, QStringLiteral("Displacement")), 4}, {mw.findChild<QWidget *>(QStringLiteral("ECamBoardStrip")), 5},
    };
    const QColor orange(0xff, 0x9f, 0x43);
    for (const Part &part : parts) {
        if (!part.w || !part.w->isVisible()) continue;
        const QRect vis = part.w->visibleRegion().boundingRect();
        if (vis.isEmpty()) continue;
        const QRect r(part.w->mapTo(&mw, vis.topLeft()), vis.size());
        p.setPen(QPen(orange, 2.5));
        p.setBrush(Qt::NoBrush);
        p.drawRoundedRect(QRectF(r).adjusted(2, 2, -2, -2), 5, 5);
        // The strip's right end is its buttons: its badge goes where there's nothing to cover.
        const QRectF badge(part.n == 5 ? r.center().x() : r.right() - 42, part.n == 5 ? r.top() + 7 : r.top() + 8, 30, 30);
        p.setPen(QPen(QColor(0x1f, 0x24, 0x2b), 2));
        p.setBrush(orange);
        p.drawEllipse(badge);
        QFont f = p.font();
        f.setPixelSize(17);
        f.setBold(true);
        p.setFont(f);
        p.setPen(QColor(0x1f, 0x24, 0x2b));
        p.drawText(badge, Qt::AlignCenter, QString::number(part.n));
    }
    p.end();
    Save(pm, name, 920);
}

// The motion laws side by side: position, velocity and acceleration over one move (u from 0 to 1).
void Laws(const QString &name) {
    const int W = 920, H = 520;
    QImage img(W * 2, H * 2, QImage::Format_ARGB32_Premultiplied);
    img.setDevicePixelRatio(2.0);
    img.fill(QColor(0x25, 0x29, 0x2f));
    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing);
    struct L {
        ecam::MotionLaw law;
        QColor color;
    };
    const L laws[] = {
        {ecam::MotionLaw::Cycloidal, theme::kDisplacement},
        {ecam::MotionLaw::ModifiedSine, theme::kVelocity},
        {ecam::MotionLaw::ModifiedTrapezoid, theme::kAccel},
        {ecam::MotionLaw::Poly345, theme::kJerk},
        {ecam::MotionLaw::Harmonic, theme::kWarn},
    };
    QFont small = p.font();
    small.setPixelSize(12);
    QFont head = small;
    head.setBold(true);
    // Legend.
    double lx = 70;
    p.setFont(head);
    for (const L &l : laws) {
        p.setPen(QPen(l.color, 3));
        p.drawLine(QPointF(lx, 18), QPointF(lx + 22, 18));
        p.setPen(theme::kText);
        const QString t = QString::fromLatin1(ecam::LawName(l.law));
        p.drawText(QPointF(lx + 28, 22), t);
        lx += 40 + p.fontMetrics().horizontalAdvance(t);
    }
    struct Panel {
        const char *title;
        double lo, hi;
        double (*get)(const ecam::LawValue &);
    };
    const Panel panels[] = {
        {"position (share of the rise)", -0.05, 1.05, [](const ecam::LawValue &v) { return v.s; }},
        {"velocity (rise per move)", -0.1, 2.3, [](const ecam::LawValue &v) { return v.v; }},
        {"acceleration", -7.0, 7.0, [](const ecam::LawValue &v) { return v.a; }},
    };
    const double top = 36, gap = 14, ph = (H - top - 30 - 2 * gap) / 3.0, left = 70, right = W - 16;
    for (int k = 0; k < 3; ++k) {
        const Panel &pn = panels[k];
        const QRectF r(left, top + k * (ph + gap), right - left, ph);
        p.fillRect(r, theme::kCanvas);
        p.setPen(QPen(theme::kGrid, 1));
        for (int i = 1; i < 4; ++i) p.drawLine(QPointF(r.left() + r.width() * i / 4, r.top()), QPointF(r.left() + r.width() * i / 4, r.bottom()));
        auto Y = [&](double v) { return r.bottom() - (v - pn.lo) / (pn.hi - pn.lo) * r.height(); };
        if (pn.lo < 0 && pn.hi > 0) {
            p.setPen(QPen(theme::kGridMajor, 1));
            p.drawLine(QPointF(r.left(), Y(0)), QPointF(r.right(), Y(0)));
        }
        for (const L &l : laws) {
            QPainterPath path;
            for (int i = 0; i <= 400; ++i) {
                const double u = i / 400.0;
                const QPointF pt(r.left() + u * r.width(), Y(pn.get(ecam::EvaluateLaw(l.law, u))));
                if (i == 0) path.moveTo(pt); else path.lineTo(pt);
            }
            p.setPen(QPen(l.color, 1.8));
            p.drawPath(path);
        }
        p.setPen(QPen(theme::kGridMajor, 1));
        p.drawRect(r);
        p.setFont(small);
        p.setPen(theme::kAxisText);
        p.save();
        p.translate(r.left() - 14, r.center().y());
        p.rotate(-90);
        p.drawText(QRectF(-r.height() / 2, -10, r.height(), 20), Qt::AlignCenter, QString::fromLatin1(pn.title));
        p.restore();
    }
    p.setFont(small);
    p.setPen(theme::kAxisText);
    const double yb = H - 22;
    for (int i = 0; i <= 4; ++i) {
        const double x = left + (right - left) * i / 4;
        p.drawText(QRectF(x - 40, yb, 80, 16), Qt::AlignHCenter | Qt::AlignTop, QStringLiteral("%1").arg(i * 25) + QStringLiteral(" %"));
    }
    p.end();
    Save(QPixmap::fromImage(img), name, W);
}

// The homing sequence drawn: the axis's position over time, toward the switch (here toward -), the run-on past it, the
// back-off, the slow latch, then to the cam's start; each phase numbered as the guide's list.
void HomingDiagram(const QString &name) {
    const int W = 900, H = 380;
    QImage img(W * 2, H * 2, QImage::Format_ARGB32_Premultiplied);
    img.setDevicePixelRatio(2.0);
    img.fill(QColor(0x25, 0x29, 0x2f));
    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF r(70, 24, W - 96, H - 70);
    p.fillRect(r, theme::kCanvas);
    p.setPen(QPen(theme::kGridMajor, 1));
    p.drawRect(r);
    // Position (top +) against time. The switch at y = 0.2 of the range; the axis starts at 1.0.
    auto X = [&](double t) { return r.left() + t * r.width(); };
    auto Y = [&](double v) { return r.bottom() - v * r.height(); };
    const double sw = 0.22;
    p.setPen(QPen(theme::kWarn, 1.2, Qt::DashLine));
    p.drawLine(QPointF(r.left(), Y(sw)), QPointF(r.right(), Y(sw)));
    QFont small = p.font();
    small.setPixelSize(12);
    p.setFont(small);
    p.setPen(theme::kWarn);
    p.drawText(QRectF(r.right() - 240, Y(sw) + 4, 234, 16), Qt::AlignRight, QStringLiteral("home switch: trips below this line"));
    // The path: piecewise, each a list of (t, position).
    QPainterPath path;
    struct Pt {
        double t, v;
    };
    std::vector<Pt> pts;
    auto line = [&](double t0, double v0, double t1, double v1, int n = 20) {
        for (int i = 0; i <= n; ++i) pts.push_back({t0 + (t1 - t0) * i / n, v0 + (v1 - v0) * i / n});
    };
    auto ease = [&](double t0, double v0, double t1, double v1, int n = 30) {  // decelerating to rest
        for (int i = 0; i <= n; ++i) {
            const double u = double(i) / n;
            pts.push_back({t0 + (t1 - t0) * u, v0 + (v1 - v0) * (1 - (1 - u) * (1 - u))});
        }
    };
    line(0.03, 0.90, 0.30, sw);          // 1 search, fast
    ease(0.30, sw, 0.36, sw - 0.09);     // 2 runs on past the switch while it stops (the homing decel)
    line(0.36, sw - 0.09, 0.47, sw);     // 3 back off until it releases
    ease(0.47, sw, 0.54, sw + 0.10);     // ... and the back-off distance more
    line(0.54, sw + 0.10, 0.80, sw);     // 4 toward it again, slow: the latch where it trips
    ease(0.80, sw, 0.83, sw - 0.012);
    ease(0.86, sw - 0.012, 0.98, 0.55);  // 5 to the cam's start
    for (size_t i = 0; i < pts.size(); ++i) {
        const QPointF q(X(pts[i].t), Y(pts[i].v));
        if (i == 0) path.moveTo(q); else path.lineTo(q);
    }
    p.setPen(QPen(theme::kDisplacement, 2.4));
    p.drawPath(path);
    // The latch: a dot where the slow move trips the switch.
    p.setPen(QPen(QColor(0x1f, 0x24, 0x2b), 1.5));
    p.setBrush(theme::kVelocity);
    p.drawEllipse(QPointF(X(0.80), Y(sw)), 6, 6);
    p.setPen(theme::kText);
    p.drawText(QRectF(X(0.80) - 40, Y(sw) - 28, 80, 16), Qt::AlignHCenter, QStringLiteral("latch"));
    // Phase numbers.
    struct Mark {
        double t, v;
        int n;
        const char *text;
    };
    const Mark marks[] = {{0.17, 0.62, 1, "search (fast)"}, {0.33, 0.06, 2, "run-on (decel)"}, {0.50, 0.42, 3, "back off"},
                          {0.67, 0.30, 4, "latch (slow)"}, {0.92, 0.47, 5, "to the start"}};
    QFont bold = small;
    bold.setBold(true);
    for (const Mark &m : marks) {
        const QRectF badge(X(m.t) - 11, Y(m.v) - 11, 22, 22);
        p.setPen(QPen(QColor(0x1f, 0x24, 0x2b), 1.5));
        p.setBrush(QColor(0xff, 0x9f, 0x43));
        p.drawEllipse(badge);
        p.setFont(bold);
        p.setPen(QColor(0x1f, 0x24, 0x2b));
        p.drawText(badge, Qt::AlignCenter, QString::number(m.n));
        p.setFont(small);
        p.setPen(theme::kText);
        p.drawText(QRectF(badge.right() + 4, badge.top() + 3, 140, 16), Qt::AlignLeft, QString::fromLatin1(m.text));
    }
    p.setPen(theme::kAxisText);
    p.drawText(QRectF(r.left(), r.bottom() + 8, r.width(), 16), Qt::AlignCenter, QStringLiteral("time"));
    p.save();
    p.translate(r.left() - 24, r.center().y());
    p.rotate(-90);
    p.drawText(QRectF(-100, -10, 200, 20), Qt::AlignCenter, QStringLiteral("position (+ up)"));
    p.restore();
    p.end();
    Save(QPixmap::fromImage(img), name, W);
}

}  // namespace

int main(int argc, char *argv[]) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    QApplication app(argc, argv);
    app.setStyleSheet(theme::AppSheet());
    app.setPalette(theme::AccentPalette(app.palette()));
    if (argc < 2) {
        std::printf("usage: help_images <output dir>\n");
        return 2;
    }
    g_out = QDir(QString::fromLocal8Bit(argv[1]));
    g_out.mkpath(QStringLiteral("."));

    // The windows below save their layout and scope settings as they go: keep the user's aside.
    QSettings settings(QStringLiteral("ECamUI"), QStringLiteral("ECamUI"));
    std::map<QString, QVariant> saved;
    for (const QString &k : settings.allKeys()) saved[k] = settings.value(k);
    settings.remove(QStringLiteral("clearpathScope"));

    // ECAM_SAMPLES_DIR: the samples from elsewhere (as committed, say), so a sample edited in the working tree isn't shot.
    const QString samples = qEnvironmentVariableIsSet("ECAM_SAMPLES_DIR")
                                ? QDir(qEnvironmentVariable("ECAM_SAMPLES_DIR")).absolutePath() + QLatin1Char('/')
                                : QStringLiteral(ECAM_SOURCE_DIR "/samples/");

    // ---- Start page and wizard
    {
        startup::StartDialog d;
        d.resize(880, 580);
        d.show();
        Pump(200);
        Shot(&d, QStringLiteral("start.png"), 720);
    }
    {
        SetupWizard wz;
        wz.resize(1040, 680);
        wz.show();
        wz.selectTemplate(ecam::TemplateKind::RiseDwellReturn);
        Pump(150);
        const char *names[] = {"wizard_template.png", nullptr, nullptr, "wizard_motion.png", "wizard_review.png"};  // the ones the guide shows
        for (int i = 0; i < wz.pageCount(); ++i) {
            if (names[i]) Shot(&wz, QString::fromLatin1(names[i]), 760);
            wz.next();
            Pump(80);
        }
    }
    {
        SetupWizard wz;
        wz.resize(1040, 680);
        wz.show();
        wz.selectTemplate(ecam::TemplateKind::FlyingCutoff);
        Pump(100);
        for (int i = 0; i < 3; ++i) wz.next();  // template, line and cut, carriage motor -> saw motor
        Pump(80);
        Shot(&wz, QStringLiteral("wizard_saw.png"), 760);
    }
    {
        SetupWizard wz;
        wz.resize(1040, 680);
        wz.show();
        wz.selectTemplate(ecam::TemplateKind::TriggeredMove);
        Pump(100);
        wz.next();
        Pump(80);
        Shot(&wz, QStringLiteral("wizard_trigger.png"), 760);
    }

    // ---- Main window
    {
        MainWindow mw;
        mw.resize(1920, 1040);  // a 1080p screen, less the taskbar
        mw.show();
        OpenAllPanels(&mw);
        mw.OpenFile(samples + QStringLiteral("variable_length_cutoff.ecam"));
        mw.ShowAxis(0, 222.0);
        mw.resize(1920, 1040);
        Pump(400);
        OpenAxisPage(&mw);
        Overview(mw, QStringLiteral("overview.png"));
        Shot(Group(&mw, QStringLiteral("Axis")), QStringLiteral("axis_panel.png"), 760);
        OpenPage(&mw, QStringLiteral("Crank"));
        Shot(Group(&mw, QStringLiteral("Crank")), QStringLiteral("crank_panel.png"), 760);
        mw.OpenFile(samples + QStringLiteral("bench_gear.ecam"));
        mw.ShowAxis(0, 90.0);
        Pump(400);
        OpenAxisPage(&mw);
        Shot(Group(&mw, QStringLiteral("Axis")), QStringLiteral("axis_gear.png"), 760);
        mw.OpenFile(samples + QStringLiteral("variable_length_cutoff.ecam"));
        mw.ShowAxis(0, 222.0);
        Pump(400);
        OpenPage(&mw, QStringLiteral("Board"));
        // No board here, so the Network fields get an example to show, as over USB.
        {
            const std::pair<const char *, const char *> example[] = {{"ECamNetIp", "192.168.1.50"}, {"ECamNetMask", "255.255.255.0"},
                                                                     {"ECamNetGateway", "192.168.1.1"}};
            for (const auto &e : example) {
                auto *f = mw.findChild<QLineEdit *>(QLatin1String(e.first));
                f->setText(QLatin1String(e.second));
                f->setEnabled(true);
            }
            mw.findChild<QLineEdit *>(QStringLiteral("ECamNetHost"))->setEnabled(true);
            auto *mode = mw.findChild<QComboBox *>(QStringLiteral("ECamNetMode"));
            mode->setCurrentIndex(1);
            mode->setEnabled(true);
            mw.findChild<QLabel *>(QStringLiteral("ECamNetInUse"))->setText(QStringLiteral("192.168.1.50"));
            mw.findChild<QLabel *>(QStringLiteral("ECamNetState"))->clear();
            for (QWidget *w : {static_cast<QWidget *>(mode), static_cast<QWidget *>(mw.findChild<QLineEdit *>(QStringLiteral("ECamNetIp"))),
                               static_cast<QWidget *>(mw.findChild<QLineEdit *>(QStringLiteral("ECamNetMask"))),
                               static_cast<QWidget *>(mw.findChild<QLineEdit *>(QStringLiteral("ECamNetGateway"))),
                               static_cast<QWidget *>(mw.findChild<QLineEdit *>(QStringLiteral("ECamNetHost")))}) {
                w->setEnabled(true);
            }
            Pump(100);
        }
        Shot(Group(&mw, QStringLiteral("Board")), QStringLiteral("board_panel.png"), 820);
        OpenPage(&mw, QStringLiteral("Segments"));
        Shot(Group(&mw, QStringLiteral("Segments")), QStringLiteral("segments.png"), 820);
        OpenPage(&mw, QStringLiteral("Analysis"));
        Shot(Group(&mw, QStringLiteral("Analysis")), QStringLiteral("analysis.png"), 760);
        OpenAxisPage(&mw);
        Shot(CamDock(&mw), QStringLiteral("cam_view.png"), 560);
        // The tree with something for it to mark.
        mw.OpenFile(samples + QStringLiteral("mixed_timeline.ecam"));
        Pump(600);
        OpenAxisPage(&mw);
        // Down to its last row: the rest of the tree is empty.
        {
            auto *tree = mw.findChild<QTreeWidget *>(QStringLiteral("ECamProjectTree"));
            QTreeWidgetItem *last = tree->topLevelItem(0);
            for (QTreeWidgetItemIterator it(tree); *it; ++it) last = *it;
            const QPoint at = tree->mapTo(&mw, QPoint(0, 0));
            Save(mw.grab(QRect(at, QSize(tree->width(), tree->visualItemRect(last).bottom() + 10))), QStringLiteral("project_tree.png"), 240);
        }
        mw.OpenFile(samples + QStringLiteral("variable_length_cutoff.ecam"));
        mw.ShowAxis(0, 222.0);
        Pump(300);
        // Diagrams: a narrower window and a taller diagram, so they read at the guide's width.
        mw.resize(1280, 1000);
        DiagramHeight(&mw, 560);
        Pump(300);
        Shot(mw.findChild<QWidget *>(QStringLiteral("ECamBoardStrip")), QStringLiteral("board_strip.png"), 820);
        Shot(Box(&mw, QStringLiteral("Displacement")), QStringLiteral("diagram.png"), 820);
        if (auto *strip = mw.findChild<ProfileStrip *>()) {
            strip->zoomBy(3.0);
            Pump(150);
            Shot(Box(&mw, QStringLiteral("Displacement")), QStringLiteral("diagram_zoom.png"), 820);
            strip->resetZoom();
        }
        // The diagram's problem markers: the packaging machine's pusher with a Linear rise (velocity steps at both
        // ends), a Harmonic return (acceleration steps) and a velocity limit the return passes.
        {
            ecam::CamProject bad;
            QString err;
            if (io::LoadProject(samples + QStringLiteral("packaging_machine.ecam"), &bad, &err)) {
                ecam::CamAxis &a = bad.axes[0];
                a.segments[1].law = ecam::MotionLaw::Linear;
                a.segments[3].law = ecam::MotionLaw::Harmonic;
                a.velMax = 300.0;
                const QString path = QDir::temp().filePath(QStringLiteral("ecw_help_problems.ecam"));
                if (io::SaveProject(bad, path, &err)) {
                    mw.OpenFile(path);
                    mw.ShowAxis(0, 0.0);
                    Pump(400);
                    Shot(Box(&mw, QStringLiteral("Displacement")), QStringLiteral("diagram_problems.png"), 820);
                    QFile::remove(path);
                }
            }
        }

        // A timeline project in a fresh window at full size (the Timing view in the centre).
        {
            MainWindow tw;
            tw.resize(1920, 1040);
            tw.show();
            OpenAllPanels(&tw);
            tw.OpenFile(samples + QStringLiteral("pick_and_place.ecam"));
            tw.ShowAxis(0, 0.9);
            tw.resize(1920, 1040);
            Pump(400);
            OpenPage(&tw, QStringLiteral("Timeline"));
            Shot(Group(&tw, QStringLiteral("Timeline")), QStringLiteral("timeline_panel.png"), 560);
            OpenPage(&tw, QStringLiteral("Outputs"));
            Shot(Group(&tw, QStringLiteral("Outputs")), QStringLiteral("outputs_panel.png"), 760);
            Shot(CamDock(&tw), QStringLiteral("timing_view.png"), 700);
        }
        mw.OpenFile(samples + QStringLiteral("pick_and_place.ecam"));
        mw.ShowAxis(0, 0.9);
        Pump(300);
        Shot(Box(&mw, QStringLiteral("Displacement")), QStringLiteral("diagram_timeline.png"), 820);

        // ---- Timeline waits, on the pick and place: the Waits page, and the Timing view with their lines.
        {
            ecam::CamProject pp;
            QString err;
            if (io::LoadProject(samples + QStringLiteral("pick_and_place.ecam"), &pp, &err)) {
                ecam::TimelineWait grip;
                grip.t = ecam::NearestTimelineRest(pp, 0.25);
                grip.name = "Gripper closed";
                grip.pin = 2;
                grip.timeout = 2.0;
                ecam::TimelineWait clear = grip;
                clear.t = ecam::NearestTimelineRest(pp, 1.55);
                clear.name = "PLC: place clear";
                clear.source = ecam::WaitSource::Plc;
                clear.pin = 0;
                clear.timeout = 0.0;
                pp.waits = {grip, clear};
                ecam::NormalizeProject(pp);
                MainWindow ww;
                ww.resize(1920, 1040);
                ww.show();
                OpenAllPanels(&ww);
                ww.UseProject(pp, QString(), false);
                ww.ShowAxis(0, 0.9);
                Pump(400);
                OpenPage(&ww, QStringLiteral("Waits"));
                Shot(Group(&ww, QStringLiteral("Waits")), QStringLiteral("waits_page.png"), 760);
                Shot(CamDock(&ww), QStringLiteral("timing_waits.png"), 700);
            }
        }

        // ---- Homing: an axis with its switches; the Homing window, and the sequence drawn.
        {
            ecam::CamProject hp = ecam::BuildTemplate(ecam::DefaultTemplate(ecam::TemplateKind::RiseDwellReturn));
            ecam::BoardInput home;
            home.pin = 3;
            home.name = "Slide home";
            home.role = ecam::CcioRole::Home;
            home.axis = 0;
            ecam::SetBoardInput(hp, home);
            ecam::BoardInput lp = home;
            lp.pin = 4;
            lp.name = "Slide limit +";
            lp.role = ecam::CcioRole::LimitPositive;
            ecam::SetBoardInput(hp, lp);
            ecam::BoardInput lm = home;
            lm.pin = 5;
            lm.name = "Slide limit -";
            lm.role = ecam::CcioRole::LimitNegative;
            ecam::SetBoardInput(hp, lm);
            ecam::AxisHome &h = hp.axes[0].home;
            h.dir = -1;
            h.fast = 50.0;
            h.slow = 5.0;
            h.back = 10.0;
            h.at = 0.0;
            h.search = 400.0;
            h.decel = 2000.0;
            h.required = true;
            ecam::NormalizeProject(hp);
            MainWindow hw;
            hw.resize(1920, 1040);
            hw.show();
            OpenAllPanels(&hw);
            hw.UseProject(hp, QString(), false);
            hw.ShowAxis(0, 60);
            Pump(300);
            OpenAxisPage(&hw);
            if (auto *dlg = hw.findChild<QDialog *>(QStringLiteral("ECamHomeDialog"))) {
                dlg->adjustSize();
                dlg->show();
                Pump(250);
                Shot(dlg, QStringLiteral("homing_window.png"), 620);
                dlg->hide();
            }
            // The switch wizard at Direction, the three switches found and proved.
            BoardLink none;
            SwitchWizard wz(&none, hp, 9, [](const ecam::CamProject &, std::function<void(bool, const QString &)> done) { done(true, QString()); });
            wz.resize(700, 420);
            wz.show();
            BoardLink::State ws;
            ws.valid = true;
            ws.state = QStringLiteral("ready");
            ws.inputs = (1 << 3) | (1 << 4) | (1 << 5);
            ws.motorsOn = 1u << hp.axes[0].connector;
            if (auto *homeBox = wz.findChild<QCheckBox *>(QStringLiteral("SwitchWizardWant2"))) homeBox->setChecked(true);
            wz.setStateForTest(ws);
            wz.setStepForTest(1);
            const int rest = (1 << 3) | (1 << 4) | (1 << 5);  // the three NC switches closed at rest
            for (int pin : {4, 5, 3}) {
                wz.feedLevels(rest, 0, 0);
                wz.feedLevels(rest & ~(1 << pin), 0, 0);
                wz.feedLevels(rest, 0, 0);
            }
            wz.setStepForTest(3);
            wz.feedStop(+1, QStringLiteral("IO-4"));
            wz.feedStop(-1, QStringLiteral("IO-3"));
            wz.feedStop(-1, QStringLiteral("IO-5"));
            wz.setStateForTest(ws);
            Pump(200);
            Shot(&wz, QStringLiteral("switch_wizard.png"), 620);
            wz.close();
        }
        HomingDiagram(QStringLiteral("homing_sequence.png"));

        // ---- Axis jog: the pick and place's three axes and a spare motor, X travel homed.
        {
            BoardLink none;
            JogDialog jd(&none);
            auto motor = [](int m, JogDialog::Role role, int axis, const char *name, const char *units, double cpu, double pos, double vmax, bool on) {
                JogDialog::MotorInfo a;
                a.motor = m;
                a.role = role;
                a.axis = axis;
                a.on = on;
                a.pos = static_cast<qint64>(pos * cpu);
                a.hasCam = role == JogDialog::Role::Axis;
                a.cam = 0;
                a.off = a.hasCam ? a.pos : 0;
                a.cpu = cpu;
                a.vmax = vmax;
                a.units = QString::fromLatin1(units);
                a.name = QString::fromLatin1(name);
                return a;
            };
            QVector<JogDialog::MotorInfo> ms = {motor(0, JogDialog::Role::Axis, 0, "X travel", "mm", 10.0255, 0.0, 500, true),
                                                motor(1, JogDialog::Role::Axis, 1, "Z lift", "mm", 53.3333, -12.5, 150, true),
                                                motor(2, JogDialog::Role::Axis, 2, "Vacuum rotate", "deg", 2.2222, 0.0, 720, false),
                                                motor(3, JogDialog::Role::Free, -1, "", "counts", 1.0, 0.0, 53333, false)};
            jd.showMotorsForTest(ms);
            BoardLink::State js;
            js.valid = true;
            js.state = QStringLiteral("ready");
            js.fault = QStringLiteral("none");
            js.motorsOn = 0x3;
            js.hasHome = 0x1;
            js.homed = 0x1;
            jd.setStateForTest(js);
            jd.show();
            Pump(250);
            Shot(&jd, QStringLiteral("axis_jog.png"), 760);
            jd.close();
        }

        // ---- Operator inputs: a panel with all five, Stop wired NC; Start pressed.
        {
            BoardLink none;
            ecam::CamProject op = ecam::DefaultProject();
            OperatorDialog od(&none, op, 9);
            od.applyConfig({QStringLiteral("op_enable IO-2 no"), QStringLiteral("op_disable A-11 nc"), QStringLiteral("op_reset IO-3 no"),
                            QStringLiteral("op_start IO-4 no"), QStringLiteral("op_stop IO-5 nc"), QStringLiteral("op_enable_mode momentary")});
            BoardLink::State os;
            os.valid = true;
            os.state = QStringLiteral("disabled");
            os.fault = QStringLiteral("none");
            os.opKnown = true;
            os.opPressed = 1 << 3;
            od.setStateForTest(os);
            od.show();
            Pump(250);
            Shot(&od, QStringLiteral("operator_inputs.png"), 520);
            od.close();
        }

        // The worked examples, built from the templates as the guided setup makes them.
        auto example = [&](ecam::TemplateKind kind, int axis, double at, const QString &diagram, const QString &cam) {
            mw.UseProject(ecam::BuildTemplate(ecam::DefaultTemplate(kind)), QString(), false);
            mw.ShowAxis(axis, at);
            Pump(300);
            if (!diagram.isEmpty()) Shot(Box(&mw, QStringLiteral("Displacement")), diagram, 820);
            if (!cam.isEmpty()) Shot(CamDock(&mw), cam, 520);
        };
        example(ecam::TemplateKind::RiseDwellReturn, 0, 60, QStringLiteral("example_pusher.png"), QString());
        example(ecam::TemplateKind::IndexFeed, 0, 90, QStringLiteral("example_index.png"), QString());
        example(ecam::TemplateKind::FlyingCutoff, 0, 230, QStringLiteral("example_flying.png"), QString());
        OpenPage(&mw, QStringLiteral("Crank"));
        Shot(Group(&mw, QStringLiteral("Crank")), QStringLiteral("example_flying_crank.png"), 760);
        example(ecam::TemplateKind::TriggeredMove, 0, 0.5, QStringLiteral("example_triggered.png"), QString());
    }

    // The pusher as a plate cam, in a fresh full-size window (the cam view is in the centre).
    {
        MainWindow cw;
        cw.resize(1920, 1040);
        cw.show();
        OpenAllPanels(&cw);
        cw.UseProject(ecam::BuildTemplate(ecam::DefaultTemplate(ecam::TemplateKind::RiseDwellReturn)), QString(), false);
        cw.ShowAxis(0, 60);
        Pump(400);
        Shot(CamDock(&cw), QStringLiteral("example_pusher_cam.png"), 520);
    }

    // ---- Motion laws
    Laws(QStringLiteral("laws.png"));

    // ---- ClearPath scope on the simulated motor
    {
        ScopeWindow sw;
        sw.resize(1240, 760);
        sw.setLanes(scope::DefaultLanes());
        sw.setWindowSeconds(5);
        sw.show();
        sw.startSimulated();
        Pump(5200);
        Shot(&sw, QStringLiteral("scope_overview.png"), 900);

        std::vector<scope::Lane> ov = scope::DefaultLanes();
        for (scope::Lane &l : ov) {
            if (l.key == QLatin1String("reg.velocity")) l.overlay = true;
        }
        sw.setLanes(ov);
        Pump(1500);
        Shot(&sw, QStringLiteral("scope_overlay.png"), 900);
        for (QPushButton *b : sw.findChildren<QPushButton *>()) {
            if (b->text() == QLatin1String("Lanes")) b->setChecked(true);
        }
        Pump(400);
        Shot(&sw, QStringLiteral("scope_lanes.png"), 900);
        for (QPushButton *b : sw.findChildren<QPushButton *>()) {
            if (b->text() == QLatin1String("Lanes")) b->setChecked(false);
        }
        sw.setLanes(scope::DefaultLanes());
        sw.setWindowSeconds(1);
        sw.setTrigger(ScopeWindow::TriggerMode::Normal, QStringLiteral("mon27"), ScopeWindow::TriggerEdge::Rising, 0.0, 0.2);
        Pump(3500);
        Shot(&sw, QStringLiteral("scope_trigger.png"), 900);
        sw.setTrigger(ScopeWindow::TriggerMode::Off, QStringLiteral("mon27"), ScopeWindow::TriggerEdge::Rising, 0.0, 0.2);
        sw.stopAndDisconnect();
        sw.close();
    }

    // Put the user's settings back exactly.
    settings.clear();
    for (const auto &kv : saved) settings.setValue(kv.first, kv.second);
    settings.sync();
    std::printf("done\n");
    return 0;
}
