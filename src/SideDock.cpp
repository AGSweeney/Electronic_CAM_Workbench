/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "SideDock.h"

#include <algorithm>

#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QFontMetricsF>
#include <QPainter>
#include <QSettings>
#include <QSplitter>
#include <QToolButton>
#include <QVBoxLayout>

namespace {

QSettings &Settings() {
    static QSettings s(QStringLiteral("ECamUI"), QStringLiteral("ECamUI"));
    return s;
}

// The collapsed strip: the title written bottom-to-top; a click anywhere opens the dock.
class Strip : public QWidget {
public:
    Strip(const QString &title, SideDock *dock) : QWidget(dock), m_title(title), m_dock(dock) {
        setCursor(Qt::PointingHandCursor);
        setToolTip(QStringLiteral("Open the %1 panel").arg(title.toLower()));
        setAttribute(Qt::WA_StyledBackground, true);
        setObjectName(QStringLiteral("SideDockStrip"));
    }

    void setTitle(const QString &title) {
        m_title = title;
        setToolTip(QStringLiteral("Open the %1 panel").arg(title.toLower()));
        update();
    }

protected:
    void paintEvent(QPaintEvent *) override {
        QPainter p(this);
        p.setRenderHint(QPainter::TextAntialiasing, true);
        p.fillRect(rect(), QColor(0x2c, 0x30, 0x36));
        p.setPen(QColor(0x20, 0x24, 0x2a));
        p.drawLine(width() - 1, 0, width() - 1, height());
        QFont f = font();
        f.setBold(true);
        f.setLetterSpacing(QFont::AbsoluteSpacing, 1.5);
        p.setFont(f);
        const QFontMetricsF fm(f);
        // Rotated 90 deg clockwise, the text runs down the strip and a letter's height (above the
        // baseline) points right on screen. Capitals stand capHeight above the baseline with nothing
        // below it, so the baseline sits half a cap height left of the centre line to centre them
        // across the strip. The text starts a little below the open button.
        const QToolButton *open = findChild<QToolButton *>();
        const double top = (open ? open->geometry().bottom() : 28) + 10.0;
        const double usable = width() - 1.0;  // the last column is the border line
        p.translate(usable / 2.0 - fm.capHeight() / 2.0, top);
        p.rotate(90);
        p.setPen(QColor(0xc9, 0xd1, 0xdc));
        p.drawText(QPointF(0, 0), m_title.toUpper());
    }
    void mousePressEvent(QMouseEvent *e) override {
        if (e->button() == Qt::LeftButton) m_dock->setCollapsed(false);
    }

private:
    QString m_title;
    SideDock *m_dock;
};

QToolButton *DockButton(const QString &text, const QString &tip, QWidget *parent) {
    auto *b = new QToolButton(parent);
    b->setText(text);
    b->setToolTip(tip);
    b->setAutoRaise(true);
    b->setStyleSheet(QStringLiteral(
        "QToolButton { background: transparent; border: none; color: #c9d1dc; font-size: 15px; font-weight: 700;"
        " min-height: 20px; padding: 0 6px; }"
        "QToolButton:hover { background: #3a414c; border-radius: 2px; }"));
    return b;
}

}  // namespace

SideDock::SideDock(const QString &title, QWidget *content, const QString &settingsKey, Edge edge, QWidget *parent)
    : QWidget(parent), m_edge(edge), m_content(content), m_key(settingsKey) {
    // The arrows point the way the dock goes.
    const QString toEdge = edge == Edge::Left ? QStringLiteral("«") : QStringLiteral("»");
    const QString fromEdge = edge == Edge::Left ? QStringLiteral("»") : QStringLiteral("«");
    auto *outer = new QHBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(0);

    // Open: header + content.
    auto *open = new QWidget(this);
    auto *v = new QVBoxLayout(open);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(4);
    m_header = new QWidget(open);
    m_header->setObjectName(QStringLiteral("SideDockHeader"));
    m_header->setAttribute(Qt::WA_StyledBackground, true);
    m_header->setStyleSheet(QStringLiteral(
        "QWidget#SideDockHeader { background: #2c3036; border: 1px solid #20242a; border-radius: 2px; }"));
    auto *h = new QHBoxLayout(m_header);
    h->setContentsMargins(10, 2, 2, 2);
    auto *label = m_label = new QLabel(title, m_header);
    label->setStyleSheet(QStringLiteral("QLabel { color: #c9d1dc; font-weight: 700; background: transparent; border: none; }"));
    h->addWidget(label);
    h->addStretch(1);
    QToolButton *collapse = DockButton(toEdge, QStringLiteral("Collapse the %1 panel").arg(title.toLower()), m_header);
    h->addWidget(collapse);
    v->addWidget(m_header);
    content->setParent(open);
    v->addWidget(content, 1);
    outer->addWidget(open, 1);

    // Collapsed: the strip, with an open button at the top.
    m_strip = new Strip(title, this);
    auto *sv = new QVBoxLayout(m_strip);
    sv->setContentsMargins(2, 4, 2, 4);
    QToolButton *expand = DockButton(fromEdge, QStringLiteral("Open the %1 panel").arg(title.toLower()), m_strip);
    sv->addWidget(expand, 0, Qt::AlignHCenter | Qt::AlignTop);
    sv->addStretch(1);
    outer->addWidget(m_strip);

    connect(collapse, &QToolButton::clicked, this, [this]() { setCollapsed(true); });
    connect(expand, &QToolButton::clicked, this, [this]() { setCollapsed(false); });

    m_collapsed = Settings().value(QStringLiteral("dock/%1/collapsed").arg(m_key), false).toBool();
    m_expandedWidth = Settings().value(QStringLiteral("dock/%1/width").arg(m_key), 360).toInt();
    Apply();
}

void SideDock::setTitle(const QString &title) {
    m_label->setText(title);
    static_cast<Strip *>(m_strip)->setTitle(title);
}

void SideDock::setCollapsed(bool collapsed) {
    if (collapsed == m_collapsed) return;
    if (collapsed) m_expandedWidth = width();  // reopen at this width
    m_collapsed = collapsed;
    Settings().setValue(QStringLiteral("dock/%1/collapsed").arg(m_key), collapsed);
    Settings().setValue(QStringLiteral("dock/%1/width").arg(m_key), m_expandedWidth);
    Apply();
    emit collapsedChanged(collapsed);
}

void SideDock::Apply() {
    QWidget *open = m_header->parentWidget();
    auto *splitter = qobject_cast<QSplitter *>(parentWidget());
    if (m_collapsed) {
        open->hide();
        m_strip->show();
        setFixedWidth(kStripWidth);
        SyncSplitter();
    } else {
        m_strip->hide();
        open->show();
        setMinimumWidth(0);
        setMaximumWidth(QWIDGETSIZE_MAX);
        if (splitter) {
            // Take the width back from the neighbour.
            QList<int> sizes = splitter->sizes();
            const int i = splitter->indexOf(this);
            const int n = Neighbour(i);
            if (i >= 0 && n >= 0 && n < sizes.size()) {
                // Never narrower than the content needs.
                const int want = std::max(m_expandedWidth, open->minimumSizeHint().width());
                const int delta = want - sizes[i];
                sizes[i] += delta;
                sizes[n] = std::max(100, sizes[n] - delta);
                splitter->setSizes(sizes);
            }
        }
    }
    updateGeometry();
}

void SideDock::SyncSplitter() {
    // A collapsed dock's slot in the splitter can still hold its old width (e.g. after the splitter
    // restored a saved layout): hand the space to the neighbour.
    auto *splitter = qobject_cast<QSplitter *>(parentWidget());
    if (!m_collapsed || !splitter) return;
    QList<int> sizes = splitter->sizes();
    const int i = splitter->indexOf(this);
    const int n = Neighbour(i);
    if (i < 0 || n < 0 || n >= sizes.size() || sizes[i] == kStripWidth) return;
    sizes[n] += sizes[i] - kStripWidth;
    sizes[i] = kStripWidth;
    splitter->setSizes(sizes);
}
