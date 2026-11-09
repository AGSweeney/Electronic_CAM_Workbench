/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "CollapsibleGroup.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QSettings>
#include <QVBoxLayout>

namespace {

// The group box style sheet reserves 20 px above the frame for the title (margin-top: 20px).
constexpr int kTitleBand = 22;

QSettings &Settings() {
    static QSettings s(QStringLiteral("ECamUI"), QStringLiteral("ECamUI"));
    return s;
}

}  // namespace

CollapsibleGroup::CollapsibleGroup(const QString &heading, const QString &settingsKey, QWidget *parent)
    : QGroupBox(parent), m_heading(heading), m_key(settingsKey) {
    auto *outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(0);
    m_body = new QWidget(this);
    m_body->setAttribute(Qt::WA_TranslucentBackground);
    outer->addWidget(m_body);
    setMouseTracking(true);
    setFocusPolicy(Qt::TabFocus);
    m_expandedPolicy = sizePolicy();
    m_collapsed = !m_key.isEmpty() && Settings().value(QStringLiteral("collapse/") + m_key, false).toBool();
    Apply();
}

void CollapsibleGroup::setHeading(const QString &heading) {
    if (heading == m_heading) return;
    m_heading = heading;
    Apply();
}

void CollapsibleGroup::setCollapsed(bool collapsed) {
    if (collapsed == m_collapsed) return;
    m_collapsed = collapsed;
    if (!m_key.isEmpty()) Settings().setValue(QStringLiteral("collapse/") + m_key, collapsed);
    Apply();
    emit collapsedChanged(collapsed);
}

void CollapsibleGroup::setFoldable(bool foldable) {
    m_foldable = foldable;
    if (!foldable) {
        m_collapsed = false;
        setFocusPolicy(Qt::NoFocus);
        setCursor(Qt::ArrowCursor);
    }
    Apply();
}

void CollapsibleGroup::Apply() {
    setTitle(m_foldable ? QStringLiteral("%1  %2").arg(m_collapsed ? QStringLiteral("▸") : QStringLiteral("▾"), m_heading)
                        : m_heading);
    setToolTip(m_collapsed ? QStringLiteral("Click the title to open") : QString());
    if (m_collapsed) {
        if (!m_body->isHidden()) m_expandedPolicy = sizePolicy();
        m_body->hide();
        // Only the title and the frame's top edge stay; the parent layout gets the rest.
        setFixedHeight(kTitleBand + fontMetrics().height() / 2 + 6);
        setSizePolicy(sizePolicy().horizontalPolicy(), QSizePolicy::Fixed);
    } else {
        setMinimumHeight(0);
        setMaximumHeight(QWIDGETSIZE_MAX);
        if (m_body->isHidden()) setSizePolicy(m_expandedPolicy);
        m_body->show();
    }
    updateGeometry();
}

bool CollapsibleGroup::InTitle(const QPoint &p) const { return p.y() >= 0 && p.y() < kTitleBand; }

void CollapsibleGroup::mousePressEvent(QMouseEvent *e) {
    if (m_foldable && e->button() == Qt::LeftButton && InTitle(e->position().toPoint())) {
        setCollapsed(!m_collapsed);
        e->accept();
        return;
    }
    QGroupBox::mousePressEvent(e);
}

void CollapsibleGroup::mouseMoveEvent(QMouseEvent *e) {
    setCursor(m_foldable && InTitle(e->position().toPoint()) ? Qt::PointingHandCursor : Qt::ArrowCursor);
    QGroupBox::mouseMoveEvent(e);
}

void CollapsibleGroup::keyPressEvent(QKeyEvent *e) {
    // Only when the group itself has the focus: Enter in a spin box inside it is not taken by the box
    // and bubbles up here, and must not fold the group away.
    if (m_foldable && hasFocus() &&
        (e->key() == Qt::Key_Space || e->key() == Qt::Key_Return || e->key() == Qt::Key_Enter)) {
        setCollapsed(!m_collapsed);
        return;
    }
    QGroupBox::keyPressEvent(e);
}
