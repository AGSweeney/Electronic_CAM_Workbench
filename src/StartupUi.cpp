/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "StartupUi.h"

#include "Brand.h"
#include "CamLaws.h"
#include "Theme.h"

#include <QApplication>
#include <QCheckBox>
#include <QDir>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLinearGradient>
#include <QListWidget>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QScreen>
#include <QSettings>
#include <QSplashScreen>
#include <QSvgRenderer>
#include <QVBoxLayout>

#include <cmath>

namespace startup {

namespace {

QSettings &Settings() {
    static QSettings s(QStringLiteral("ECamUI"), QStringLiteral("ECamUI"));
    return s;
}

constexpr int kMaxRecent = 10;

// A rise / dwell / return / dwell drawn with the cycloidal law: the app's own subject as decoration.
QPainterPath CamCurve(const QRectF &r) {
    QPainterPath path;
    const int n = 240;
    for (int i = 0; i <= n; ++i) {
        const double deg = 360.0 * i / n;
        double s = 0.0;
        if (deg < 120.0) {
            s = ecam::EvaluateLaw(ecam::MotionLaw::Cycloidal, deg / 120.0).s;
        } else if (deg < 180.0) {
            s = 1.0;
        } else if (deg < 300.0) {
            s = 1.0 - ecam::EvaluateLaw(ecam::MotionLaw::Cycloidal, (deg - 180.0) / 120.0).s;
        }
        const QPointF p(r.left() + r.width() * deg / 360.0, r.bottom() - r.height() * s);
        if (i == 0) {
            path.moveTo(p);
        } else {
            path.lineTo(p);
        }
    }
    return path;
}

}  // namespace

QSplashScreen *ShowSplash() {
    const QSize size(600, 340);
    const qreal dpr = qApp->primaryScreen() ? qApp->primaryScreen()->devicePixelRatio() : 1.0;
    QPixmap pm(size * dpr);
    pm.setDevicePixelRatio(dpr);
    {
        QPainter p(&pm);
        p.setRenderHint(QPainter::Antialiasing);
        QLinearGradient bg(0, 0, size.width(), size.height());
        bg.setColorAt(0.0, QColor(0x2a, 0x30, 0x38));
        bg.setColorAt(1.0, QColor(0x1b, 0x1f, 0x25));
        p.fillRect(QRect(QPoint(0, 0), size), bg);

        // Faint grid and the cam curve along the bottom.
        p.setPen(QPen(QColor(0x2e, 0x35, 0x3e), 1));
        const QRectF plot(36, 214, size.width() - 72, 70);
        for (int i = 0; i <= 8; ++i) {
            const double x = plot.left() + plot.width() * i / 8.0;
            p.drawLine(QPointF(x, plot.top() - 6), QPointF(x, plot.bottom() + 6));
        }
        QPen curve(theme::kDisplacement, 2.4);
        curve.setCapStyle(Qt::RoundCap);
        p.setPen(curve);
        p.drawPath(CamCurve(plot));

        QSvgRenderer icon(QStringLiteral(":/icons/ecam_app_icon.svg"));
        icon.render(&p, QRectF(36, 42, 96, 96));

        QFont title = p.font();
        title.setPixelSize(30);
        title.setBold(true);
        p.setFont(title);
        p.setPen(QColor(0xf0, 0xf4, 0xf8));
        p.drawText(QRectF(152, 54, size.width() - 170, 40), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral(ECAM_PRODUCT_NAME));
        QFont sub = p.font();
        sub.setPixelSize(15);
        sub.setBold(false);
        p.setFont(sub);
        p.setPen(QColor(0x9f, 0xa6, 0xaf));
        p.drawText(QRectF(154, 96, size.width() - 170, 24), Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("Electronic cam profiles for the Teknic ClearCore"));

        p.setPen(QPen(QColor(0x3a, 0x42, 0x4d), 1));
        p.drawRect(QRectF(0.5, 0.5, size.width() - 1, size.height() - 1));
    }
    auto *splash = new QSplashScreen(pm);
    splash->show();
    splash->showMessage(QStringLiteral("Loading…"), Qt::AlignLeft | Qt::AlignBottom, QColor(0x8a, 0x92, 0x9d));
    qApp->processEvents();
    return splash;
}

QStringList RecentProjects() {
    QStringList out;
    for (const QString &p : Settings().value("files/recent").toStringList()) {
        if (QFileInfo::exists(p) && !out.contains(p, Qt::CaseInsensitive)) out << p;
    }
    return out;
}

void AddRecentProject(const QString &path) {
    if (path.isEmpty() || path.startsWith(QLatin1Char(':'))) return;
    const QString abs = QFileInfo(path).absoluteFilePath();
    QStringList list = Settings().value("files/recent").toStringList();
    list.removeIf([&](const QString &p) { return QString::compare(p, abs, Qt::CaseInsensitive) == 0; });
    list.prepend(abs);
    while (list.size() > kMaxRecent) list.removeLast();
    Settings().setValue("files/recent", list);
}

void RemoveRecentProject(const QString &path) {
    QStringList list = Settings().value("files/recent").toStringList();
    list.removeIf([&](const QString &p) { return QString::compare(p, path, Qt::CaseInsensitive) == 0; });
    Settings().setValue("files/recent", list);
}

bool ShowStartPageAtStartup() { return Settings().value("start/showAtStartup", true).toBool(); }

const std::vector<Example> &Examples() {
    static const std::vector<Example> kExamples = {
        {QStringLiteral(":/samples/index_feed.ecam"), QStringLiteral("Index feed"),
         QStringLiteral("Crank axes from an encoder: a feed that advances every revolution.")},
        {QStringLiteral(":/samples/driven_crank.ecam"), QStringLiteral("Driven crank"),
         QStringLiteral("The ClearCore drives the crank motor itself; the followers track it.")},
        {QStringLiteral(":/samples/packaging_machine.ecam"), QStringLiteral("Packaging machine"),
         QStringLiteral("Pusher, gripper and index feed on one driven crank.")},
        {QStringLiteral(":/samples/pick_and_place.ecam"), QStringLiteral("Pick and place"),
         QStringLiteral("Three timeline axes and two outputs, started by an input.")},
        {QStringLiteral(":/samples/mixed_timeline.ecam"), QStringLiteral("Crank and timeline together"),
         QStringLiteral("A crank pusher alongside triggered timeline axes.")},
        {QStringLiteral(":/samples/flying_cutoff.ecam"), QStringLiteral("Flying shear"),
         QStringLiteral("A carriage synchronised to the line while a knife cuts.")},
        {QStringLiteral(":/samples/true_flying_cutoff.ecam"), QStringLiteral("Flying saw"),
         QStringLiteral("Carriage, saw plunge and outputs for a flying cut-off saw.")},
        {QStringLiteral(":/samples/variable_length_cutoff.ecam"), QStringLiteral("Variable-length cut-off"),
         QStringLiteral("Cut-length mode: the piece length changes while running.")},
        {QStringLiteral(":/samples/bench_spline.ecam"), QStringLiteral("Cubic spline"),
         QStringLiteral("A closed curve through 18 points, one follower on a driven crank.")},
        {QStringLiteral(":/samples/bench_gear.ecam"), QStringLiteral("Electronic gear"),
         QStringLiteral("A follower at 3 : 2 of a driven crank, exact over any number of revolutions.")},
    };
    return kExamples;
}

// ---- Start page --------------------------------------------------------------------------------

namespace {

QPushButton *Tile(const QString &title, const QString &text, bool primary = false) {
    auto *b = new QPushButton(title + QStringLiteral("\n") + text);
    b->setObjectName(QStringLiteral("ECamTile"));
    if (primary) b->setProperty("role", QStringLiteral("primary"));
    b->setCursor(Qt::PointingHandCursor);
    b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    return b;
}

QLabel *Heading(const QString &text) {
    auto *l = new QLabel(text);
    l->setObjectName(QStringLiteral("ECamHeading"));
    return l;
}

}  // namespace

StartDialog::StartDialog(QWidget *parent) : QDialog(parent) {
    setWindowTitle(QStringLiteral(ECAM_PRODUCT_NAME));
    setStyleSheet(theme::DialogSheet());
    theme::ApplyAccent(this);
    setMinimumSize(820, 540);

    auto *root = new QVBoxLayout(this);
    root->setContentsMargins(24, 20, 24, 16);
    root->setSpacing(14);

    // Header: logo, name, one line on what it is.
    {
        auto *row = new QHBoxLayout();
        auto *logo = new QLabel();
        logo->setPixmap(QIcon(QStringLiteral(":/icons/ecam_app_icon.svg")).pixmap(56, 56));
        row->addWidget(logo);
        auto *col = new QVBoxLayout();
        col->setSpacing(2);
        auto *title = new QLabel(QStringLiteral(ECAM_PRODUCT_NAME));
        title->setObjectName(QStringLiteral("ECamTitle"));
        auto *sub = new QLabel(QStringLiteral("Design electronic cam profiles, check them, and run them on a ClearCore."));
        sub->setObjectName(QStringLiteral("ECamHelp"));
        col->addWidget(title);
        col->addWidget(sub);
        row->addLayout(col, 1);
        root->addLayout(row);
    }

    auto *body = new QHBoxLayout();
    body->setSpacing(20);

    // Left: what to do.
    auto *left = new QVBoxLayout();
    left->setSpacing(8);
    left->addWidget(Heading(QStringLiteral("Start")));
    QPushButton *wizard = Tile(QStringLiteral("Guided setup…"),
                               QStringLiteral("Pick a machine motion and answer a few questions"), true);
    QPushButton *blank = Tile(QStringLiteral("New project"), QStringLiteral("One axis with a basic rise, dwell and return to edit"));
    QPushButton *open = Tile(QStringLiteral("Open project…"), QStringLiteral("An .ecam file saved earlier"));
    left->addWidget(wizard);
    left->addWidget(blank);
    left->addWidget(open);
    left->addSpacing(8);
    left->addWidget(Heading(QStringLiteral("Examples")));
    m_examples = new QListWidget();
    for (const Example &e : Examples()) {
        auto *it = new QListWidgetItem(e.title, m_examples);
        it->setToolTip(e.about);
        it->setData(Qt::UserRole, e.file);
    }
    left->addWidget(m_examples, 1);
    body->addLayout(left, 1);

    // Right: recent projects.
    auto *right = new QVBoxLayout();
    right->setSpacing(8);
    right->addWidget(Heading(QStringLiteral("Recent projects")));
    m_recent = new QListWidget();
    const QStringList recent = RecentProjects();
    for (const QString &p : recent) {
        const QFileInfo fi(p);
        auto *it = new QListWidgetItem(fi.completeBaseName() + QStringLiteral("\n") + QDir::toNativeSeparators(fi.absolutePath()),
                                       m_recent);
        it->setToolTip(QDir::toNativeSeparators(p));
        it->setData(Qt::UserRole, p);
    }
    if (recent.isEmpty()) {
        auto *it = new QListWidgetItem(QStringLiteral("No recent projects yet.\nProjects you open or save appear here."), m_recent);
        it->setFlags(Qt::NoItemFlags);
    }
    right->addWidget(m_recent, 1);
    auto *hint = new QLabel(QStringLiteral("Double-click a project or an example to open it."));
    hint->setObjectName(QStringLiteral("ECamHelp"));
    right->addWidget(hint);
    body->addLayout(right, 1);
    root->addLayout(body, 1);

    // Footer.
    auto *foot = new QHBoxLayout();
    m_atStartup = new QCheckBox(QStringLiteral("Show this window when the program starts"));
    m_atStartup->setChecked(ShowStartPageAtStartup());
    connect(m_atStartup, &QCheckBox::toggled, this, [](bool on) { Settings().setValue("start/showAtStartup", on); });
    foot->addWidget(m_atStartup);
    foot->addStretch(1);
    auto *close = new QPushButton(QStringLiteral("Close"));
    close->setToolTip(QStringLiteral("Keep the current project"));
    foot->addWidget(close);
    root->addLayout(foot);

    connect(wizard, &QPushButton::clicked, this, [this]() { Pick(Choice::Wizard); });
    connect(blank, &QPushButton::clicked, this, [this]() { Pick(Choice::New); });
    connect(open, &QPushButton::clicked, this, [this]() { Pick(Choice::Open); });
    connect(close, &QPushButton::clicked, this, &QDialog::reject);
    connect(m_recent, &QListWidget::itemActivated, this, [this](QListWidgetItem *it) {
        const QString p = it->data(Qt::UserRole).toString();
        if (!p.isEmpty()) Pick(Choice::Recent, p);
    });
    connect(m_examples, &QListWidget::itemActivated, this,
            [this](QListWidgetItem *it) { Pick(Choice::Example, it->data(Qt::UserRole).toString(), it->text()); });
    wizard->setFocus();
}

void StartDialog::Pick(Choice c, const QString &path, const QString &title) {
    m_choice = c;
    m_path = path;
    m_title = title;
    accept();
}

}  // namespace startup
