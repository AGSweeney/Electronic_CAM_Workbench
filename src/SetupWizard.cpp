/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "SetupWizard.h"

#include "Brand.h"
#include "Theme.h"

#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QStackedWidget>
#include <QStyle>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

using namespace ecam;

// ---- Preview: every axis's position in its own band, the first axis's velocity under them -----------

class MotionPreview : public QWidget {
public:
    explicit MotionPreview(bool compact, QWidget *parent = nullptr) : QWidget(parent), m_compact(compact) {
        setMinimumHeight(compact ? 120 : 220);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }
    void setProject(const CamProject &p) {
        m_project = p;
        update();
    }
    // A thumbnail of a template's default motion.
    static QPixmap Thumb(TemplateKind kind, QSize size, qreal dpr) {
        MotionPreview w(true);
        w.m_project = BuildTemplate(DefaultTemplate(kind));
        w.m_thumb = true;
        QPixmap pm(size * dpr);
        pm.setDevicePixelRatio(dpr);
        pm.fill(theme::kCanvas);
        QPainter p(&pm);
        w.Draw(p, QRectF(QPointF(0, 0), QSizeF(size)));
        return pm;
    }

protected:
    void paintEvent(QPaintEvent *) override {
        QPainter p(this);
        Draw(p, QRectF(rect()));
    }

private:
    void Draw(QPainter &p, const QRectF &r) {
        p.setRenderHint(QPainter::Antialiasing);
        p.fillRect(r, theme::kCanvas);
        if (m_project.axes.empty()) return;
        const CamAxis &first = m_project.axes[0];
        const bool timeline = first.master == AxisMaster::Timeline;
        const double span = first.span;
        const double padL = m_thumb ? 4 : 44, padR = m_thumb ? 4 : 12, padT = m_thumb ? 4 : 10, padB = m_thumb ? 4 : 22;
        const QRectF plot(r.left() + padL, r.top() + padT, r.width() - padL - padR, r.height() - padT - padB);
        const int axes = static_cast<int>(m_project.axes.size());
        const int bands = m_thumb ? axes : axes + 1;  // + velocity
        const double bandH = plot.height() / bands;
        auto X = [&](double x) { return plot.left() + plot.width() * x / span; };

        QFont f = p.font();
        f.setPixelSize(10);
        p.setFont(f);
        if (!m_thumb) {
            // Grid and master ticks.
            p.setPen(QPen(theme::kGrid, 1));
            const int ticks = timeline ? 4 : 8;
            for (int i = 0; i <= ticks; ++i) {
                const double x = plot.left() + plot.width() * i / ticks;
                p.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
                p.setPen(theme::kAxisText);
                const double v = span * i / ticks;
                const QString t = timeline ? QString::number(v, 'f', 2) + QStringLiteral(" s") : QString::number(v, 'f', 0) + QStringLiteral("°");
                p.drawText(QRectF(x - 30, plot.bottom() + 4, 60, 14), Qt::AlignHCenter | Qt::AlignTop, t);
                p.setPen(QPen(theme::kGrid, 1));
            }
            // Sync window of a flying cut-off.
            int sa = -1;
            double s0 = 0, s1 = 0;
            if (m_project.master.lineMode && LineSync(m_project, &sa, &s0, &s1)) {
                p.fillRect(QRectF(QPointF(X(s0), plot.top()), QPointF(X(s1), plot.bottom())), QColor(0x2e, 0x53, 0x85, 60));
            }
        }
        for (int ai = 0; ai < axes; ++ai) {
            const CamAxis &a = m_project.axes[ai];
            const AxisStats st = Analyze(a, 720);
            double lo = std::min(st.minPos, 0.0), hi = std::max(st.maxPos, lo + 1e-9);
            const QRectF band(plot.left(), plot.top() + bandH * ai, plot.width(), bandH);
            const double pad = band.height() * 0.14;
            QPainterPath path;
            const int n = std::max(60, static_cast<int>(plot.width()));
            for (int i = 0; i <= n; ++i) {
                const double x = a.span * i / n;
                const CamPoint c = i == n ? EvaluateSegment(a, static_cast<int>(a.segments.size()) - 1, 1.0) : Evaluate(a, x);
                const QPointF pt(X(x), band.bottom() - pad - (c.pos - lo) / (hi - lo) * (band.height() - 2 * pad));
                if (i == 0) path.moveTo(pt); else path.lineTo(pt);
            }
            p.setPen(QPen(ai == 0 ? theme::kDisplacement : theme::SegmentColor(ai + 2), m_thumb ? 1.6 : 2.0));
            p.drawPath(path);
            if (!m_thumb) {
                p.setPen(theme::kAxisText);
                p.drawText(QRectF(r.left() + 2, band.top() + pad - 7, padL - 6, 14), Qt::AlignRight | Qt::AlignVCenter,
                           QString::number(hi, 'g', 4));
                p.drawText(QRectF(r.left() + 2, band.bottom() - pad - 7, padL - 6, 14), Qt::AlignRight | Qt::AlignVCenter,
                           QString::number(lo, 'g', 4));
                p.drawText(QRectF(band.left() + 6, band.top() + 2, band.width() - 12, 14), Qt::AlignLeft | Qt::AlignTop,
                           QStringLiteral("%1 position (%2)").arg(QString::fromStdString(a.name), QString::fromStdString(a.units)));
            }
        }
        if (!m_thumb) {
            // Velocity of the first axis, per second.
            const CamAxis &a = first;
            const double w = MasterRate(m_project.master, a);
            const QRectF band(plot.left(), plot.top() + bandH * axes, plot.width(), bandH);
            const int n = std::max(60, static_cast<int>(plot.width()));
            std::vector<double> v(n + 1);
            double vmax = 1e-12;
            for (int i = 0; i <= n; ++i) {
                v[i] = Evaluate(a, std::min(a.span * i / n, a.span - 1e-9)).vel * w;
                vmax = std::max(vmax, std::fabs(v[i]));
            }
            const double mid = band.center().y(), half = band.height() * 0.4;
            p.setPen(QPen(theme::kGridMajor, 1));
            p.drawLine(QPointF(band.left(), mid), QPointF(band.right(), mid));
            QPainterPath path;
            for (int i = 0; i <= n; ++i) {
                const QPointF pt(X(a.span * i / n), mid - v[i] / vmax * half);
                if (i == 0) path.moveTo(pt); else path.lineTo(pt);
            }
            p.setPen(QPen(theme::kVelocity, 1.6));
            p.drawPath(path);
            p.setPen(theme::kAxisText);
            p.drawText(QRectF(r.left() + 2, mid - half - 7, padL - 6, 14), Qt::AlignRight | Qt::AlignVCenter, QString::number(vmax, 'g', 4));
            p.drawText(QRectF(r.left() + 2, mid - 7, padL - 6, 14), Qt::AlignRight | Qt::AlignVCenter, QStringLiteral("0"));
            p.drawText(QRectF(r.left() + 2, mid + half - 7, padL - 6, 14), Qt::AlignRight | Qt::AlignVCenter, QString::number(-vmax, 'g', 4));
            p.drawText(QRectF(band.left() + 6, band.top() + 2, band.width() - 12, 14), Qt::AlignLeft | Qt::AlignTop,
                       QStringLiteral("%1 velocity (%2/s)").arg(QString::fromStdString(a.name), QString::fromStdString(a.units)));
        }
    }

    CamProject m_project;
    bool m_compact = false;
    bool m_thumb = false;
};

// ---- Helpers -----------------------------------------------------------------------------------------

namespace {

QString InputName(int i) {
    if (i < 6) return QStringLiteral("IO-%1").arg(i);
    if (i < 9) return QStringLiteral("DI-%1").arg(i);
    return QStringLiteral("A-%1").arg(i);
}

QLabel *Help(const QString &text) {
    auto *l = new QLabel(text);
    l->setObjectName(QStringLiteral("ECamHelp"));
    l->setWordWrap(true);
    return l;
}

QLabel *Heading(const QString &text) {
    auto *l = new QLabel(text);
    l->setObjectName(QStringLiteral("ECamHeading"));
    return l;
}

QLabel *Derived() {
    auto *l = new QLabel();
    l->setObjectName(QStringLiteral("ECamDerived"));
    l->setWordWrap(true);
    return l;
}

QFormLayout *Form() {
    auto *f = new QFormLayout();
    f->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    f->setHorizontalSpacing(14);
    f->setVerticalSpacing(8);
    f->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
    return f;
}

QWidget *Wrap(QLayout *l) {
    auto *w = new QWidget();
    w->setLayout(l);
    return w;
}

QString SetupList(TemplateKind k) {
    switch (k) {
        case TemplateKind::RiseDwellReturn:
            return QStringLiteral("• What drives the machine cycle (an encoder, or a crank motor the ClearCore drives)<br>"
                                  "• The motor and its mechanics<br>• The stroke, and where in the cycle it rises and returns");
        case TemplateKind::IndexFeed:
            return QStringLiteral("• What drives the machine cycle<br>• The motor and its mechanics<br>"
                                  "• The feed length per cycle, and when in the cycle it feeds");
        case TemplateKind::Oscillate:
            return QStringLiteral("• What drives the machine cycle<br>• The motor and its mechanics<br>"
                                  "• The stroke, and where it turns around");
        case TemplateKind::TriggeredMove:
            return QStringLiteral("• What starts the move (an input or a command)<br>• The motor and its mechanics<br>"
                                  "• The distance and the time for each part of the move");
        case TemplateKind::FlyingCutoff:
            return QStringLiteral("• The encoder wheel on the line, the cut length and the line speed<br>"
                                  "• The carriage motor, and optionally a saw plunge motor and a saw output<br>"
                                  "• How long the carriage takes to catch up, ride along and return");
        case TemplateKind::Count: break;
    }
    return {};
}

QString LawHelp(MotionLaw law) {
    switch (law) {
        case MotionLaw::Cycloidal: return QStringLiteral("Smooth start and stop (zero acceleration at both ends). A safe default.");
        case MotionLaw::ModifiedSine: return QStringLiteral("Smooth, with a lower peak velocity. Good for heavier loads and index drives.");
        case MotionLaw::ModifiedTrapezoid: return QStringLiteral("Smooth, with the lowest peak acceleration. Good for fast, light moves.");
        case MotionLaw::Poly345: return QStringLiteral("Smooth polynomial, a little gentler on velocity than cycloidal.");
        case MotionLaw::Poly4567: return QStringLiteral("The smoothest (no jerk at the ends), for slightly higher peaks.");
        case MotionLaw::Harmonic:
            return QStringLiteral("Gentle velocity, but the acceleration jumps where it starts and stops. Fine for oscillating.");
        case MotionLaw::Linear:
            return QStringLiteral("Constant speed: the velocity jumps at both ends (a hard knock). Rarely what you want here.");
        default: return {};
    }
}

constexpr MotionLaw kLaws[] = {MotionLaw::Cycloidal, MotionLaw::ModifiedSine, MotionLaw::ModifiedTrapezoid, MotionLaw::Poly345,
                               MotionLaw::Poly4567,  MotionLaw::Harmonic,     MotionLaw::Linear};

QString Sig(double v) { return QString::number(v, 'g', 4); }

}  // namespace

// ---- Wizard ------------------------------------------------------------------------------------------

SetupWizard::SetupWizard(QWidget *parent) : QDialog(parent) {
    setWindowTitle(QStringLiteral("Guided setup — " ECAM_PRODUCT_NAME));
    setStyleSheet(theme::DialogSheet());
    theme::ApplyAccent(this);
    setMinimumSize(980, 640);
    m_spec = DefaultTemplate(TemplateKind::RiseDwellReturn);

    auto *root = new QHBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    // Step list on the left.
    auto *side = new QWidget();
    side->setObjectName(QStringLiteral("ECamSide"));
    side->setAttribute(Qt::WA_StyledBackground);
    side->setFixedWidth(200);
    auto *sl = new QVBoxLayout(side);
    sl->setContentsMargins(0, 24, 0, 16);
    sl->setSpacing(2);
    auto *st = new QLabel(QStringLiteral("  Guided setup"));
    st->setObjectName(QStringLiteral("ECamHeading"));
    sl->addWidget(st);
    sl->addSpacing(12);
    for (int i = 0; i < kPages; ++i) {  // numbered and hidden per template in Refresh
        auto *l = new QLabel();
        l->setObjectName(QStringLiteral("ECamStep"));
        m_steps.push_back(l);
        sl->addWidget(l);
    }
    sl->addStretch(1);
    root->addWidget(side);

    // Pages and buttons.
    auto *main = new QVBoxLayout();
    main->setContentsMargins(28, 22, 28, 16);
    main->setSpacing(12);
    m_pages = new QStackedWidget();
    m_pages->addWidget(BuildTemplatePage());
    m_pages->addWidget(BuildMasterPage());
    m_pages->addWidget(BuildMotorPage(0));
    m_pages->addWidget(BuildMotorPage(1));
    m_pages->addWidget(BuildMotionPage());
    m_pages->addWidget(BuildReviewPage());
    main->addWidget(m_pages, 1);
    auto *buttons = new QHBoxLayout();
    auto *cancel = new QPushButton(QStringLiteral("Cancel"));
    m_back = new QPushButton(QStringLiteral("‹ Back"));
    m_next = new QPushButton(QStringLiteral("Next ›"));
    m_next->setProperty("role", QStringLiteral("primary"));
    m_next->setDefault(true);
    buttons->addWidget(cancel);
    buttons->addStretch(1);
    buttons->addWidget(m_back);
    buttons->addWidget(m_next);
    main->addLayout(buttons);
    root->addLayout(main, 1);

    connect(cancel, &QPushButton::clicked, this, &QDialog::reject);
    connect(m_back, &QPushButton::clicked, this, [this]() { GoTo(m_page - 1); });
    connect(m_next, &QPushButton::clicked, this, &SetupWizard::next);

    m_templates->setCurrentRow(0);
    Load();
    GoTo(0);
}

ecam::CamProject SetupWizard::project() const { return BuildTemplate(m_spec); }

QString SetupWizard::templateName() const { return QString::fromLatin1(TemplateName(m_spec.kind)); }

void SetupWizard::selectTemplate(TemplateKind kind) { m_templates->setCurrentRow(static_cast<int>(kind)); }

std::vector<int> SetupWizard::Visible() const {
    std::vector<int> v;
    for (int p = 0; p < kPages; ++p) {
        if (p == kSawMotorPage && m_spec.kind != TemplateKind::FlyingCutoff) continue;
        v.push_back(p);
    }
    return v;
}

QString SetupWizard::pageTitle() const {
    const auto v = Visible();
    return m_steps[v[std::clamp(m_page, 0, static_cast<int>(v.size()) - 1)]]->text().section(QLatin1Char(' '), 1).trimmed();
}

void SetupWizard::next() {
    if (m_page + 1 < pageCount()) {
        GoTo(m_page + 1);
    } else {
        accept();
    }
}

void SetupWizard::GoTo(int page) {
    const auto v = Visible();
    m_page = std::clamp(page, 0, static_cast<int>(v.size()) - 1);
    m_pages->setCurrentIndex(v[m_page]);
    m_back->setEnabled(m_page > 0);
    m_next->setText(m_page == static_cast<int>(v.size()) - 1 ? QStringLiteral("Create project") : QStringLiteral("Next ›"));
    Refresh();
}

// Bound widgets ---------------------------------------------------------------------------------------

QDoubleSpinBox *SetupWizard::Num(double TemplateSpec::*field, double min, double max, int decimals, const QString &suffix) {
    auto *w = new QDoubleSpinBox();
    w->setRange(min, max);
    w->setDecimals(decimals);
    w->setSuffix(suffix);
    w->setMinimumWidth(130);
    w->setAccelerated(true);
    m_loaders.push_back([this, w, field]() { w->setValue(m_spec.*field); });
    connect(w, &QDoubleSpinBox::valueChanged, this, [this, field](double v) {
        if (m_loading) return;
        m_spec.*field = v;
        Changed();
    });
    return w;
}

QSpinBox *SetupWizard::Int(int TemplateSpec::*field, int min, int max) {
    auto *w = new QSpinBox();
    w->setRange(min, max);
    w->setFixedWidth(90);
    m_loaders.push_back([this, w, field]() { w->setValue(m_spec.*field); });
    connect(w, &QSpinBox::valueChanged, this, [this, field](int v) {
        if (m_loading) return;
        m_spec.*field = v;
        Changed();
    });
    return w;
}

QComboBox *SetupWizard::Connector(int TemplateSpec::*field) {
    auto *w = new QComboBox();
    for (int i = 0; i < kMotorConnectors; ++i) w->addItem(QStringLiteral("M-%1").arg(i));
    m_loaders.push_back([this, w, field]() { w->setCurrentIndex(m_spec.*field); });
    connect(w, &QComboBox::currentIndexChanged, this, [this, field](int i) {
        if (m_loading || i < 0) return;
        m_spec.*field = i;
        Changed();
    });
    return w;
}

QCheckBox *SetupWizard::Flag(bool TemplateSpec::*field, const QString &text) {
    auto *w = new QCheckBox(text);
    m_loaders.push_back([this, w, field]() { w->setChecked(m_spec.*field); });
    connect(w, &QCheckBox::toggled, this, [this, field](bool on) {
        if (m_loading) return;
        m_spec.*field = on;
        Changed();
    });
    return w;
}

// Pages -----------------------------------------------------------------------------------------------

QWidget *SetupWizard::BuildTemplatePage() {
    auto *v = new QVBoxLayout();
    v->setContentsMargins(0, 0, 0, 0);
    v->addWidget(Heading(QStringLiteral("What should the motor do?")));
    v->addWidget(Help(QStringLiteral("Pick the motion closest to your machine. Everything it sets up can be changed later in "
                                     "the editor; this only gives you a good, working start.")));
    auto *row = new QHBoxLayout();
    row->setSpacing(18);
    m_templates = new QListWidget();
    m_templates->setIconSize(QSize(120, 46));
    m_templates->setFixedWidth(330);
    const qreal dpr = devicePixelRatioF();
    for (int k = 0; k < static_cast<int>(TemplateKind::Count); ++k) {
        const auto kind = static_cast<TemplateKind>(k);
        const QPixmap thumb = MotionPreview::Thumb(kind, QSize(120, 46), dpr);
        QIcon icon(thumb);
        icon.addPixmap(thumb, QIcon::Selected);
        auto *it = new QListWidgetItem(icon, QString::fromLatin1(TemplateName(kind)), m_templates);
        it->setSizeHint(QSize(300, 60));
    }
    row->addWidget(m_templates);
    auto *detail = new QVBoxLayout();
    detail->setSpacing(10);
    m_tplName = Heading(QString());
    m_tplAbout = Help(QString());
    m_tplAbout->setStyleSheet(QStringLiteral("color: #d4dae2; font-size: 13px;"));
    m_tplSetup = new QLabel();
    m_tplSetup->setObjectName(QStringLiteral("ECamCard"));
    m_tplSetup->setWordWrap(true);
    m_tplSetup->setTextFormat(Qt::RichText);
    m_tplPreview = new MotionPreview(false);
    detail->addWidget(m_tplName);
    detail->addWidget(m_tplAbout);
    detail->addWidget(m_tplSetup);
    detail->addWidget(m_tplPreview, 1);
    row->addLayout(detail, 1);
    v->addLayout(row, 1);

    connect(m_templates, &QListWidget::currentRowChanged, this, [this](int row) {
        if (row < 0) return;
        const auto kind = static_cast<TemplateKind>(row);
        if (kind == m_spec.kind) return;
        m_spec = DefaultTemplate(kind);
        Load();
    });
    connect(m_templates, &QListWidget::itemDoubleClicked, this, [this]() { next(); });
    return Wrap(v);
}

QWidget *SetupWizard::BuildMasterPage() {
    auto *v = new QVBoxLayout();
    v->setContentsMargins(0, 0, 0, 0);
    m_masterKind = new QStackedWidget();

    // Crank: encoder or driven crank.
    {
        auto *c = new QVBoxLayout();
        c->setContentsMargins(0, 0, 0, 0);
        c->setSpacing(10);
        c->addWidget(Heading(QStringLiteral("What drives the machine cycle?")));
        c->addWidget(Help(QStringLiteral(
            "Your motion is drawn over one machine cycle, 0 to 360°, like a mechanical cam on a camshaft. Something has "
            "to say where in the cycle the machine is: the \"crank\".")));
        m_srcEncoder = new QRadioButton(QStringLiteral("An encoder on the machine's shaft. The machine runs on its own; the "
                                                       "ClearCore follows it (Encoder Adapter on DI-6..DI-8)."));
        m_srcDriven = new QRadioButton(QStringLiteral("The ClearCore drives a crank motor itself, at a speed you set."));
        auto *grp = new QButtonGroup(this);
        grp->addButton(m_srcEncoder);
        grp->addButton(m_srcDriven);
        c->addWidget(m_srcEncoder);
        c->addWidget(m_srcDriven);
        connect(m_srcEncoder, &QRadioButton::toggled, this, [this](bool on) {
            if (m_loading || !on) return;
            m_spec.source = MasterSource::Encoder;
            Changed();
        });
        connect(m_srcDriven, &QRadioButton::toggled, this, [this](bool on) {
            if (m_loading || !on) return;
            m_spec.source = MasterSource::DrivenCrank;
            // The crank motor takes a connector: move the axis off it.
            if (m_spec.connector == m_spec.crankConnector) m_spec.connector = (m_spec.crankConnector + 1) % kMotorConnectors;
            Load();
        });

        auto *ef = Form();
        ef->addRow(QStringLiteral("Encoder counts per rev"), Num(&TemplateSpec::encoderPpr, 1, 1e7, 0, QStringLiteral(" counts")));
        auto *gear = new QHBoxLayout();
        gear->addWidget(Int(&TemplateSpec::gearNum, 1, 100000));
        gear->addWidget(new QLabel(QStringLiteral(":")));
        gear->addWidget(Int(&TemplateSpec::gearDen, 1, 100000));
        gear->addStretch(1);
        ef->addRow(QStringLiteral("Encoder revs : machine revs"), gear);
        ef->addRow(QStringLiteral("Expected machine speed"), Num(&TemplateSpec::rpm, 0.1, 5000, 1, QStringLiteral(" rpm")));
        QLabel *encHelp = Help(QStringLiteral("Counts are quadrature edges (4 per line: a 1000-line encoder gives 4000). "
                                              "The speed is only used to show velocities and check limits."));
        encHelp->setMinimumWidth(420);
        ef->addRow(QString(), encHelp);
        m_encoderBox = Wrap(ef);

        auto *df = Form();
        df->addRow(QStringLiteral("Crank motor"), Connector(&TemplateSpec::crankConnector));
        df->addRow(QStringLiteral("Motor steps per rev"), Num(&TemplateSpec::encoderPpr, 1, 1e7, 0, QStringLiteral(" steps")));
        auto *g2 = new QHBoxLayout();
        g2->addWidget(Int(&TemplateSpec::gearNum, 1, 100000));
        g2->addWidget(new QLabel(QStringLiteral(":")));
        g2->addWidget(Int(&TemplateSpec::gearDen, 1, 100000));
        g2->addStretch(1);
        df->addRow(QStringLiteral("Motor revs : machine revs"), g2);
        df->addRow(QStringLiteral("Run speed"), Num(&TemplateSpec::rpm, 0.1, 5000, 1, QStringLiteral(" rpm")));
        df->addRow(QStringLiteral("Ramp up / down"), Num(&TemplateSpec::crankRampRpmPerSec, 1, 100000, 0, QStringLiteral(" rpm/s")));
        df->addRow(QStringLiteral("Top speed allowed"), Num(&TemplateSpec::crankMaxRpm, 1, 10000, 0, QStringLiteral(" rpm")));
        m_drivenBox = Wrap(df);
        c->addWidget(m_encoderBox);
        c->addWidget(m_drivenBox);
        m_crankDerived = Derived();
        c->addWidget(m_crankDerived);
        c->addStretch(1);
        m_masterKind->addWidget(Wrap(c));
    }

    // Timeline: the trigger.
    {
        auto *c = new QVBoxLayout();
        c->setContentsMargins(0, 0, 0, 0);
        c->setSpacing(10);
        c->addWidget(Heading(QStringLiteral("What starts the move?")));
        c->addWidget(Help(QStringLiteral("This motion runs on time, not on a machine shaft: each trigger runs it once from the "
                                         "start, then it waits for the next.")));
        auto *f = Form();
        m_trigger = new QComboBox();
        m_trigger->addItem(QStringLiteral("A digital input"));
        m_trigger->addItem(QStringLiteral("A command (the Trigger button or a host)"));
        f->addRow(QStringLiteral("Start on"), m_trigger);
        m_triggerInput = new QComboBox();
        for (int i = 0; i < kTriggerInputs; ++i) m_triggerInput->addItem(InputName(i));
        f->addRow(QStringLiteral("Input"), m_triggerInput);
        m_triggerEdge = new QComboBox();
        m_triggerEdge->addItem(QStringLiteral("Rising (off → on)"));
        m_triggerEdge->addItem(QStringLiteral("Falling (on → off)"));
        f->addRow(QStringLiteral("Edge"), m_triggerEdge);
        f->addRow(QString(), Flag(&TemplateSpec::repeat, QStringLiteral("Keep repeating after the first trigger")));
        c->addLayout(f);
        c->addStretch(1);
        connect(m_trigger, &QComboBox::currentIndexChanged, this, [this](int i) {
            if (m_loading) return;
            m_spec.trigger = i == 0 ? TriggerSource::DigitalInput : TriggerSource::Command;
            Changed();
        });
        connect(m_triggerInput, &QComboBox::currentIndexChanged, this, [this](int i) {
            if (m_loading || i < 0) return;
            m_spec.triggerInput = i;
            Changed();
        });
        connect(m_triggerEdge, &QComboBox::currentIndexChanged, this, [this](int i) {
            if (m_loading) return;
            m_spec.triggerRising = i == 0;
            Changed();
        });
        m_masterKind->addWidget(Wrap(c));
    }

    // Line: flying cut-off.
    {
        auto *c = new QVBoxLayout();
        c->setContentsMargins(0, 0, 0, 0);
        c->setSpacing(10);
        c->addWidget(Heading(QStringLiteral("The line and the cut")));
        c->addWidget(Help(QStringLiteral(
            "An encoder wheel rides on the material and measures the line. One machine cycle is one piece: the drawn cycle "
            "is one cut length of line, and the cut length can be changed later while running.")));
        auto *f = Form();
        f->addRow(QStringLiteral("Encoder counts per rev"), Num(&TemplateSpec::encoderPpr, 1, 1e7, 0, QStringLiteral(" counts")));
        f->addRow(QStringLiteral("Wheel circumference"), Num(&TemplateSpec::wheelCircumference, 1, 100000, 2, QStringLiteral(" mm")));
        f->addRow(QStringLiteral("Cut length"), Num(&TemplateSpec::cutLength, 10, 1e6, 1, QStringLiteral(" mm")));
        f->addRow(QStringLiteral("Kerf (blade width)"), Num(&TemplateSpec::kerf, 0, 100, 2, QStringLiteral(" mm")));
        f->addRow(QStringLiteral("Line speed"), Num(&TemplateSpec::lineSpeed, 0.01, 1000, 2, QStringLiteral(" m/min")));
        c->addLayout(f);
        m_lineDerived = Derived();
        c->addWidget(m_lineDerived);
        c->addStretch(1);
        m_masterKind->addWidget(Wrap(c));
    }
    v->addWidget(m_masterKind, 1);
    return Wrap(v);
}

QWidget *SetupWizard::BuildMotorPage(int which) {
    MotorUi &m = m_motor[which];
    m.f = which == 0 ? MotorFields{&TemplateSpec::name, &TemplateSpec::units, &TemplateSpec::connector, &TemplateSpec::drive,
                                   &TemplateSpec::motorPpr, &TemplateSpec::screwPitch, &TemplateSpec::pulleyPd,
                                   &TemplateSpec::countsPerUnit, &TemplateSpec::velMax, &TemplateSpec::accMax}
                     : MotorFields{&TemplateSpec::sawName, &TemplateSpec::sawUnits, &TemplateSpec::sawConnector,
                                   &TemplateSpec::sawDrive, &TemplateSpec::sawMotorPpr, &TemplateSpec::sawPitch,
                                   &TemplateSpec::sawPulleyPd, &TemplateSpec::sawCountsPerUnit, &TemplateSpec::sawVelMax,
                                   &TemplateSpec::sawAccMax};
    const MotorFields f = m.f;
    auto *v = new QVBoxLayout();
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(10);
    m.heading = Heading(QString());
    m.help = Help(QString());
    v->addWidget(m.heading);
    v->addWidget(m.help);
    if (which == 1) {
        m.enable = Flag(&TemplateSpec::saw, QStringLiteral("This machine has a saw plunge motor"));
        v->addWidget(m.enable);
    }

    auto *form = Form();
    auto *name = new QLineEdit();
    name->setMinimumWidth(220);
    m_loaders.push_back([this, name, f]() { name->setText(QString::fromStdString(m_spec.*f.name)); });
    connect(name, &QLineEdit::textChanged, this, [this, f](const QString &t) {
        if (m_loading) return;
        m_spec.*f.name = t.trimmed().toStdString();
        Changed();
    });
    form->addRow(QStringLiteral("Axis name"), name);
    form->addRow(QStringLiteral("Motor connector"), Connector(f.connector));
    m.units = new QComboBox();
    m.units->setEditable(true);
    m.units->setMinimumWidth(110);
    m.units->addItems({QStringLiteral("mm"), QStringLiteral("in"), QStringLiteral("deg")});
    connect(m.units, &QComboBox::currentTextChanged, this, [this, f](const QString &t) {
        if (m_loading || t.trimmed().isEmpty()) return;
        m_spec.*f.units = t.trimmed().toStdString();
        Changed();
    });
    form->addRow(QStringLiteral("Units"), m.units);
    m.drive = new QComboBox();
    m.drive->addItem(QStringLiteral("Ballscrew / lead screw"), static_cast<int>(AxisDrive::Ballscrew));
    m.drive->addItem(QStringLiteral("Belt and pulley / rack and pinion"), static_cast<int>(AxisDrive::Belt));
    m.drive->addItem(QStringLiteral("Direct (I know the counts per unit)"), static_cast<int>(AxisDrive::Direct));
    QComboBox *drive = m.drive;
    connect(drive, &QComboBox::currentIndexChanged, this, [this, f, drive](int) {
        if (m_loading) return;
        m_spec.*f.drive = static_cast<AxisDrive>(drive->currentData().toInt());
        Changed();
    });
    form->addRow(QStringLiteral("Drive"), m.drive);
    m.motorPpr = Num(f.motorPpr, 1, 1e7, 0, QStringLiteral(" steps"));
    form->addRow(QStringLiteral("Motor steps per rev"), m.motorPpr);
    m.pitch = Num(f.pitch, 0.001, 10000, 3);
    form->addRow(QStringLiteral("Screw lead (travel per rev)"), m.pitch);
    m.pulley = Num(f.pulley, 0.001, 10000, 3);
    form->addRow(QStringLiteral("Pulley pitch diameter"), m.pulley);
    m.direct = Num(f.countsPerUnit, 0.0001, 1e9, 4);
    form->addRow(QStringLiteral("Counts per unit"), m.direct);
    m.derived = Derived();
    m.derived->setMinimumWidth(420);
    form->addRow(QString(), m.derived);
    m.velMax = Num(f.velMax, 0, 1e9, 1);
    m.velMax->setSpecialValueText(QStringLiteral("not set"));
    m.accMax = Num(f.accMax, 0, 1e12, 0);
    m.accMax->setSpecialValueText(QStringLiteral("not set"));
    form->addRow(QStringLiteral("Velocity limit"), m.velMax);
    form->addRow(QStringLiteral("Acceleration limit"), m.accMax);
    QLabel *limHelp = Help(QStringLiteral("Optional. With limits set, the program tells you how fast the machine can run."));
    limHelp->setMinimumWidth(420);
    form->addRow(QString(), limHelp);
    m.form = form;
    m.fields = Wrap(form);
    v->addWidget(m.fields);

    if (which == 1) {
        m_sawOutput = Flag(&TemplateSpec::sawOutput, QStringLiteral("Switch output IO-0 on while riding the line (saw motor, clamp)"));
        v->addSpacing(6);
        v->addWidget(m_sawOutput);
    }
    m.warn = new QLabel();
    m.warn->setObjectName(QStringLiteral("ECamWarn"));
    m.warn->setWordWrap(true);
    v->addWidget(m.warn);
    v->addStretch(1);

    auto *scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setWidget(Wrap(v));
    return scroll;
}

QWidget *SetupWizard::BuildMotionPage() {
    auto *v = new QVBoxLayout();
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(10);
    v->addWidget(Heading(QStringLiteral("The motion")));
    auto *row = new QHBoxLayout();
    row->setSpacing(20);

    auto *left = new QVBoxLayout();
    left->setSpacing(8);
    auto *f = Form();
    m_law = new QComboBox();
    for (MotionLaw l : kLaws) m_law->addItem(QString::fromLatin1(LawName(l)), static_cast<int>(l));
    connect(m_law, &QComboBox::currentIndexChanged, this, [this](int) {
        if (m_loading) return;
        m_spec.law = static_cast<MotionLaw>(m_law->currentData().toInt());
        Changed();
    });
    f->addRow(QStringLiteral("Motion law"), m_law);
    m_motionForm = f;
    m_stroke = Num(&TemplateSpec::stroke, -1e6, 1e6, 3);
    m_strokeLabel = new QLabel(QStringLiteral("Stroke"));
    f->addRow(m_strokeLabel, m_stroke);
    left->addLayout(f);
    m_lawHelp = Help(QString());
    left->addWidget(m_lawHelp);

    m_phases = new QStackedWidget();
    {   // RiseDwellReturn
        auto *p = Form();
        p->addRow(QStringLiteral("Rise starts at"), Num(&TemplateSpec::a0, 0, 350, 1, QStringLiteral("°")));
        p->addRow(QStringLiteral("Rise ends at"), Num(&TemplateSpec::a1, 1, 359, 1, QStringLiteral("°")));
        p->addRow(QStringLiteral("Return starts at"), Num(&TemplateSpec::a2, 1, 359, 1, QStringLiteral("°")));
        p->addRow(QStringLiteral("Return ends at"), Num(&TemplateSpec::a3, 10, 360, 1, QStringLiteral("°")));
        m_phases->addWidget(Wrap(p));
    }
    {   // IndexFeed
        auto *p = Form();
        p->addRow(QStringLiteral("Feed starts at"), Num(&TemplateSpec::a0, 0, 350, 1, QStringLiteral("°")));
        p->addRow(QStringLiteral("Feed ends at"), Num(&TemplateSpec::a1, 10, 360, 1, QStringLiteral("°")));
        m_phases->addWidget(Wrap(p));
    }
    {   // Oscillate
        auto *p = Form();
        p->addRow(QStringLiteral("Turn around at"), Num(&TemplateSpec::a1, 30, 330, 1, QStringLiteral("°")));
        m_phases->addWidget(Wrap(p));
    }
    {   // TriggeredMove
        auto *p = Form();
        p->addRow(QStringLiteral("Wait after the trigger"), Num(&TemplateSpec::delay, 0, 3600, 3, QStringLiteral(" s")));
        p->addRow(QStringLiteral("Move out in"), Num(&TemplateSpec::moveTime, 0.005, 3600, 3, QStringLiteral(" s")));
        p->addRow(QStringLiteral("Hold for"), Num(&TemplateSpec::holdTime, 0, 3600, 3, QStringLiteral(" s")));
        p->addRow(QStringLiteral("Come back in"), Num(&TemplateSpec::returnTime, 0.005, 3600, 3, QStringLiteral(" s")));
        m_phases->addWidget(Wrap(p));
    }
    {   // FlyingCutoff
        auto *p = Form();
        p->addRow(QStringLiteral("Catch up with the line in"), Num(&TemplateSpec::rampDeg, 6, 120, 1, QStringLiteral("°")));
        p->addRow(QStringLiteral("Ride the line (cut) for"), Num(&TemplateSpec::syncDeg, 6, 200, 1, QStringLiteral("°")));
        p->addRow(QStringLiteral("Return in"), Num(&TemplateSpec::returnDeg, 12, 250, 1, QStringLiteral("°")));
        m_sawDepth = Num(&TemplateSpec::sawStroke, 0.1, 1000, 2);
        p->addRow(QStringLiteral("Saw plunge depth"), m_sawDepth);
        m_flyForm = p;
        m_phases->addWidget(Wrap(p));
    }
    left->addWidget(m_phases);
    m_flyDerived = Derived();
    left->addWidget(m_flyDerived);
    left->addStretch(1);
    auto *leftW = Wrap(left);
    leftW->setFixedWidth(360);
    row->addWidget(leftW);

    auto *right = new QVBoxLayout();
    m_preview = new MotionPreview(false);
    right->addWidget(m_preview, 1);
    m_peaks = new QLabel();
    m_peaks->setObjectName(QStringLiteral("ECamCard"));
    m_peaks->setTextFormat(Qt::RichText);
    m_peaks->setWordWrap(true);
    right->addWidget(m_peaks);
    row->addLayout(right, 1);
    v->addLayout(row, 1);
    return Wrap(v);
}

QWidget *SetupWizard::BuildReviewPage() {
    auto *v = new QVBoxLayout();
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(10);
    v->addWidget(Heading(QStringLiteral("Review")));
    m_review = new QLabel();
    m_review->setTextFormat(Qt::RichText);
    m_review->setWordWrap(true);
    m_review->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    m_review->setOpenExternalLinks(false);
    auto *scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setWidget(m_review);
    v->addWidget(scroll, 1);
    return Wrap(v);
}

// State -----------------------------------------------------------------------------------------------

void SetupWizard::Load() {
    m_loading = true;
    for (auto &l : m_loaders) l();
    m_srcEncoder->setChecked(m_spec.source == MasterSource::Encoder);
    m_srcDriven->setChecked(m_spec.source == MasterSource::DrivenCrank);
    m_trigger->setCurrentIndex(m_spec.trigger == TriggerSource::DigitalInput ? 0 : 1);
    m_triggerInput->setCurrentIndex(m_spec.triggerInput);
    m_triggerEdge->setCurrentIndex(m_spec.triggerRising ? 0 : 1);
    for (MotorUi &m : m_motor) {
        m.units->setCurrentText(QString::fromStdString(m_spec.*m.f.units));
        m.drive->setCurrentIndex(m.drive->findData(static_cast<int>(m_spec.*m.f.drive)));
    }
    m_law->setCurrentIndex(m_law->findData(static_cast<int>(m_spec.law)));
    m_loading = false;
    Refresh();
}

void SetupWizard::Changed() { Refresh(); }

QStringList SetupWizard::Problems(QStringList *warnings) const {
    QStringList errors;
    const CamProject p = project();
    const CamMaster &m = p.master;
    // Connectors.
    QStringList owners[kMotorConnectors];
    if (UsesCrank(p) && m.source == MasterSource::DrivenCrank) owners[m.crankConnector] << QStringLiteral("the crank motor");
    for (const CamAxis &a : p.axes) owners[a.connector] << QString::fromStdString(a.name);
    for (int i = 0; i < kMotorConnectors; ++i) {
        if (owners[i].size() > 1) errors << QStringLiteral("M-%1 is used by both %2.").arg(i).arg(owners[i].join(QStringLiteral(" and ")));
    }
    for (const CamAxis &a : p.axes) {
        const AxisStats st = Analyze(a);
        const double w = MasterRate(m, a);
        const QString n = QString::fromStdString(a.name), u = QString::fromStdString(a.units);
        for (const CamIssue &is : st.issues) {
            if (is.kind == IssueKind::VelocityStep) {
                errors << QStringLiteral("%1: the velocity jumps at %2 (a hard knock). Pick a smooth motion law.")
                              .arg(n, a.master == AxisMaster::Timeline ? Sig(is.deg) + QStringLiteral(" s") : Sig(is.deg) + QStringLiteral("°"));
                break;
            }
        }
        if (a.velMax > 0 && st.maxVel * w > a.velMax) {
            errors << QStringLiteral("%1: peak velocity %2 %3/s is above the limit %4.").arg(n, Sig(st.maxVel * w), u, Sig(a.velMax));
        }
        if (a.accMax > 0 && st.maxAcc * w * w > a.accMax) {
            errors << QStringLiteral("%1: peak acceleration %2 %3/s² is above the limit %4.").arg(n, Sig(st.maxAcc * w * w), u, Sig(a.accMax));
        }
        const double rpm = st.maxVel * w * a.countsPerUnit / std::max(a.motorPpr, 1.0) * 60.0;
        if (warnings && rpm > 3000.0) {
            *warnings << QStringLiteral("%1: the motor peaks at %2 rpm. Check the motor's rated speed, or slow the machine "
                                        "down or give the move more time.").arg(n, QString::number(rpm, 'f', 0));
        }
    }
    if (warnings && UsesCrank(p) && m.source == MasterSource::DrivenCrank && m.rpm > m.crankMaxRpm) {
        *warnings << QStringLiteral("The run speed %1 rpm is above the crank's top speed %2 rpm.").arg(Sig(m.rpm), Sig(m.crankMaxRpm));
    }
    if (warnings && m_spec.kind == TemplateKind::FlyingCutoff && FlyingWaitDeg(m_spec) < 6.0) {
        *warnings << QStringLiteral("The catch-up, ride and return do not fit in one cycle; they were shortened to fit.");
    }
    if (warnings && m_spec.kind == TemplateKind::TriggeredMove && m_spec.trigger == TriggerSource::DigitalInput &&
        m_spec.triggerInput >= 6 && m_spec.triggerInput <= 8) {
        *warnings << QStringLiteral("%1 is one of the Encoder Adapter's inputs; fine while no encoder is fitted.")
                         .arg(InputName(m_spec.triggerInput));
    }
    return errors;
}

void SetupWizard::Refresh() {
    const TemplateKind k = m_spec.kind;
    const bool timeline = TemplateOnTimeline(k), line = TemplateUsesLine(k);
    const QString u = QString::fromStdString(m_spec.units);
    const CamProject p = project();

    // Step list.
    const QString masterStep = timeline ? QStringLiteral("Trigger") : line ? QStringLiteral("Line and cut") : QStringLiteral("Machine cycle");
    const QString names[kPages] = {QStringLiteral("Template"),
                                   masterStep,
                                   line ? QStringLiteral("Carriage motor") : QStringLiteral("Motor"),
                                   QStringLiteral("Saw motor"),
                                   QStringLiteral("Motion"),
                                   QStringLiteral("Review")};
    {
        const auto vis = Visible();
        for (QLabel *l : m_steps) l->setVisible(false);
        for (int i = 0; i < static_cast<int>(vis.size()); ++i) {
            QLabel *l = m_steps[vis[i]];
            l->setVisible(true);
            l->setText(QStringLiteral("%1   %2").arg(i + 1).arg(names[vis[i]]));
            l->setProperty("current", i == m_page);
            l->setProperty("done", i < m_page);
            l->style()->unpolish(l);
            l->style()->polish(l);
        }
    }

    // 1 Template
    m_tplName->setText(QString::fromLatin1(TemplateName(k)));
    m_tplAbout->setText(QString::fromLatin1(TemplateSummary(k)));
    m_tplSetup->setText(QStringLiteral("<b>You'll set up</b><br>") + SetupList(k));
    m_tplPreview->setProject(p);

    // 2 Master
    m_masterKind->setCurrentIndex(timeline ? 1 : line ? 2 : 0);
    m_encoderBox->setVisible(m_spec.source == MasterSource::Encoder);
    m_drivenBox->setVisible(m_spec.source == MasterSource::DrivenCrank);
    if (!timeline && !line) {
        m_crankDerived->setText(QStringLiteral("= %1 counts per machine revolution, one cycle every %2 s at %3 rpm.")
                                    .arg(Sig(p.master.countsPerRev), Sig(60.0 / std::max(p.master.rpm, 1e-9)), Sig(p.master.rpm)));
    }
    m_triggerInput->setEnabled(m_spec.trigger == TriggerSource::DigitalInput);
    m_triggerEdge->setEnabled(m_spec.trigger == TriggerSource::DigitalInput);
    if (line) {
        m_lineDerived->setText(QStringLiteral("= %1 cuts per minute; %2 encoder counts per mm of line.")
                                   .arg(Sig(p.master.rpm), Sig(p.master.lineCountsPerMm)));
    }

    // Motors
    QStringList warns;
    const QStringList errs = Problems(&warns);
    QStringList conn;
    for (const QString &e : errs) {
        if (e.startsWith(QStringLiteral("M-"))) conn << QStringLiteral("⚠ ") + e;
    }
    for (int i = 0; i < 2; ++i) {
        MotorUi &m = m_motor[i];
        const AxisDrive d = m_spec.*m.f.drive;
        const QString mu = QString::fromStdString(m_spec.*m.f.units);
        m.form->setRowVisible(m.motorPpr, d != AxisDrive::Direct);
        m.form->setRowVisible(m.pitch, d == AxisDrive::Ballscrew);
        m.form->setRowVisible(m.pulley, d == AxisDrive::Belt);
        m.form->setRowVisible(m.direct, d == AxisDrive::Direct);
        m.pitch->setSuffix(QStringLiteral(" %1/rev").arg(mu));
        m.pulley->setSuffix(QStringLiteral(" %1").arg(mu));
        m.direct->setSuffix(QStringLiteral(" counts/%1").arg(mu));
        m.velMax->setSuffix(QStringLiteral(" %1/s").arg(mu));
        m.accMax->setSuffix(QStringLiteral(" %1/s²").arg(mu));
        const int ax = i;  // the saw is axis 1 when there is one
        if (ax < static_cast<int>(p.axes.size())) {
            m.derived->setText(QStringLiteral("= %1 motor counts per %2").arg(Sig(p.axes[ax].countsPerUnit), mu));
        }
        m.warn->setText(conn.join(QStringLiteral("<br>")));
    }
    if (line) {
        m_motor[0].heading->setText(QStringLiteral("The carriage motor"));
        m_motor[0].help->setText(QStringLiteral("The carriage rides along with the material during the cut. Which ClearCore motor "
                                                "connector its motor is on, and what lies between the motor and the carriage."));
    } else {
        m_motor[0].heading->setText(QStringLiteral("The motor and its mechanics"));
        m_motor[0].help->setText(QStringLiteral("Which ClearCore motor connector the follower motor is on, and what lies between "
                                                "the motor and the load, so positions can be given in real units."));
    }
    m_motor[1].heading->setText(QStringLiteral("The saw plunge motor"));
    m_motor[1].help->setText(QStringLiteral("A second motor lowers the saw into the material while the carriage rides the line, "
                                            "and lifts it before the carriage returns. Its own connector, mechanics and limits."));
    m_motor[1].fields->setEnabled(m_spec.saw);

    // 4 Motion
    m_phases->setCurrentIndex(static_cast<int>(k));
    m_sawDepth->setSuffix(QStringLiteral(" ") + QString::fromStdString(m_spec.sawUnits));
    m_flyForm->setRowVisible(m_sawDepth, m_spec.saw);
    m_lawHelp->setText(LawHelp(m_spec.law));
    m_motionForm->setRowVisible(m_law, !line);
    m_lawHelp->setVisible(!line);
    m_stroke->setSuffix(QStringLiteral(" ") + u);
    m_stroke->setVisible(!line);
    m_strokeLabel->setVisible(!line);
    m_strokeLabel->setText(k == TemplateKind::IndexFeed      ? QStringLiteral("Feed per cycle")
                           : k == TemplateKind::TriggeredMove ? QStringLiteral("Move distance")
                                                              : QStringLiteral("Stroke"));
    if (line) {
        m_flyDerived->setText(QStringLiteral("Carriage stroke %1 mm. The cycle opens with a %2° wait, which stretches or shrinks "
                                             "when the cut length changes; the shortest piece is %3 mm.")
                                  .arg(Sig(FlyingCarriageStroke(m_spec)), Sig(std::max(0.0, LineWaitDeg(p))), Sig(LineMinLength(p))));
    } else if (timeline && !p.axes.empty()) {
        m_flyDerived->setText(QStringLiteral("The whole move takes %1 s.").arg(Sig(p.axes[0].span)));
    } else {
        m_flyDerived->clear();
    }
    m_preview->setProject(p);
    if (!p.axes.empty()) {
        const CamAxis &a = p.axes[0];
        const AxisStats st = Analyze(a);
        const double w = MasterRate(p.master, a);
        const double rpm = st.maxVel * w * a.countsPerUnit / std::max(a.motorPpr, 1.0) * 60.0;
        QString html = QStringLiteral("<b>%1 at %2</b><br>").arg(QString::fromStdString(a.name).toHtmlEscaped(),
                                                                 timeline ? QStringLiteral("full speed")
                                                                 : line   ? QStringLiteral("%1 m/min").arg(Sig(m_spec.lineSpeed))
                                                                          : QStringLiteral("%1 rpm").arg(Sig(p.master.rpm)));
        html += QStringLiteral("Peak velocity <b>%1 %2/s</b> &nbsp;·&nbsp; peak acceleration <b>%3 %2/s²</b> &nbsp;·&nbsp; motor "
                               "<b>%4 rpm</b>")
                    .arg(Sig(st.maxVel * w), u.toHtmlEscaped(), Sig(st.maxAcc * w * w), QString::number(rpm, 'f', 0));
        for (const QString &e : errs) html += QStringLiteral("<br><span style='color:#ff8a80;'>⚠ %1</span>").arg(e.toHtmlEscaped());
        for (const QString &wn : warns) html += QStringLiteral("<br><span style='color:#ffcc80;'>⚠ %1</span>").arg(wn.toHtmlEscaped());
        m_peaks->setText(html);
    }

    // 5 Review
    {
        QString h;
        auto row = [](const QString &k2, const QString &v) {
            return QStringLiteral("<tr><td style='color:#9fa6af; padding:2px 16px 2px 0;'>%1</td><td style='padding:2px 0;'>%2</td></tr>").arg(k2, v);
        };
        h += QStringLiteral("<p style='font-size:13px;'>A <b>%1</b> project. Check it below, then <b>Create project</b> to open it "
                            "in the editor.</p>").arg(QString::fromLatin1(TemplateName(k)));
        h += QStringLiteral("<table cellspacing='0'>");
        const CamMaster &m = p.master;
        if (timeline) {
            h += row(QStringLiteral("Starts on"), m.trigger == TriggerSource::DigitalInput
                                                      ? QStringLiteral("%1, %2 edge").arg(InputName(m.triggerInput), m.triggerRising ? QStringLiteral("rising") : QStringLiteral("falling"))
                                                      : QStringLiteral("a command"));
            h += row(QStringLiteral("Runs"), m.timelineRepeat ? QStringLiteral("repeating") : QStringLiteral("once per trigger"));
        } else if (line) {
            h += row(QStringLiteral("Master"), QStringLiteral("encoder wheel on the line, %1 counts per rev, %2 mm circumference")
                                                   .arg(Sig(m.crankPpr), Sig(m.wheelCircumference)));
            h += row(QStringLiteral("Cut"), QStringLiteral("%1 mm pieces, %2 mm kerf, %3 cuts/min at %4 m/min")
                                                .arg(Sig(m.designLength), Sig(m.kerf), Sig(m.rpm), Sig(m_spec.lineSpeed)));
        } else if (m.source == MasterSource::Encoder) {
            h += row(QStringLiteral("Master"), QStringLiteral("encoder on the machine shaft, %1 counts per machine rev, expected %2 rpm")
                                                   .arg(Sig(m.countsPerRev), Sig(m.rpm)));
        } else {
            h += row(QStringLiteral("Master"), QStringLiteral("crank motor on M-%1, %2 rpm, ramp %3 rpm/s")
                                                   .arg(m.crankConnector).arg(Sig(m.rpm), Sig(m.crankRampRpmPerSec)));
        }
        for (const CamAxis &a : p.axes) {
            const AxisStats st = Analyze(a);
            const double w = MasterRate(m, a);
            const QString au = QString::fromStdString(a.units).toHtmlEscaped();
            h += row(QString::fromStdString(a.name).toHtmlEscaped(),
                     QStringLiteral("M-%1 · %2 segments · stroke %3 %4 · peak %5 %4/s, %6 %4/s² · %7 counts/%4")
                         .arg(a.connector)
                         .arg(a.segments.size())
                         .arg(Sig(st.maxPos - st.minPos), au, Sig(st.maxVel * w), Sig(st.maxAcc * w * w), Sig(a.countsPerUnit)));
        }
        for (const OutputPulse &o : p.outputs) {
            h += row(QStringLiteral("IO-%1").arg(o.output), QStringLiteral("on %1° to %2°").arg(Sig(o.on), Sig(o.off)));
        }
        h += QStringLiteral("</table>");
        if (errs.isEmpty() && warns.isEmpty()) {
            h += QStringLiteral("<p style='color:#7fe3a0;'>✓ No problems found.</p>");
        } else {
            h += QStringLiteral("<p>");
            for (const QString &e : errs) h += QStringLiteral("<span style='color:#ff8a80;'>⚠ %1</span><br>").arg(e.toHtmlEscaped());
            for (const QString &wn : warns) h += QStringLiteral("<span style='color:#ffcc80;'>⚠ %1</span><br>").arg(wn.toHtmlEscaped());
            h += QStringLiteral("</p>");
        }
        h += QStringLiteral(
            "<p style='margin-top:14px;'><b>Next steps</b></p><ol style='margin-left:-20px;'>"
            "<li>Fine-tune it in the editor: drag the knots, change a segment's law, add segments.</li>"
            "<li><b>File → Save</b> the project.</li>"
            "<li>In the board strip at the bottom, connect to the ClearCore and <b>Upload</b> the profile.</li>"
            "<li>Run it in <b>Simulate</b> first (motors stay disabled), then enable the motors.</li></ol>");
        m_review->setText(h);
    }
}
