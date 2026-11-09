/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// A group box whose title folds it away: click the title (or press Space/Enter on it) to collapse
// the group to its title bar, again to open it. The contents live in body(), so rows hidden on
// purpose inside it stay hidden when the group opens again. The state is saved under the key.

#include <QGroupBox>

class CollapsibleGroup : public QGroupBox {
    Q_OBJECT
public:
    CollapsibleGroup(const QString &heading, const QString &settingsKey, QWidget *parent = nullptr);

    // Put the group's layout on this widget.
    QWidget *body() const { return m_body; }

    void setHeading(const QString &heading);
    QString heading() const { return m_heading; }
    bool isCollapsed() const { return m_collapsed; }
    void setCollapsed(bool collapsed);
    // Off: always open with a plain title, and the saved state is left alone. For a group that is a page.
    void setFoldable(bool foldable);

signals:
    void collapsedChanged(bool collapsed);

protected:
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void keyPressEvent(QKeyEvent *e) override;

private:
    bool InTitle(const QPoint &p) const;
    void Apply();

    QWidget *m_body = nullptr;
    QString m_heading;
    QString m_key;
    bool m_collapsed = false;
    bool m_foldable = true;
    QSizePolicy m_expandedPolicy;
};
