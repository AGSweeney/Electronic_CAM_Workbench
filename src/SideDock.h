/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// A side panel that collapses like a docked tool window: open, it has a slim header with its title
// and a collapse button; collapsed, it is a narrow strip with the title written vertically, and a
// click anywhere on the strip opens it again at its previous width. Lives in a QSplitter, at the left
// of its neighbour or (Edge::Right) at the right. The state and width are saved under the key.

#include <QWidget>

class QLabel;
class QToolButton;

class SideDock : public QWidget {
    Q_OBJECT
public:
    enum class Edge { Left, Right };

    SideDock(const QString &title, QWidget *content, const QString &settingsKey, Edge edge = Edge::Left,
             QWidget *parent = nullptr);

    void setTitle(const QString &title);

    bool isCollapsed() const { return m_collapsed; }
    void setCollapsed(bool collapsed);
    // Collapsed: give the splitter slot's spare width to the neighbour (call after restoring the
    // splitter's saved state).
    void SyncSplitter();

    static constexpr int kStripWidth = 28;

signals:
    void collapsedChanged(bool collapsed);

private:
    void Apply();
    int Neighbour(int self) const { return m_edge == Edge::Left ? self + 1 : self - 1; }

    Edge m_edge = Edge::Left;
    QLabel *m_label = nullptr;
    QWidget *m_content = nullptr;
    QWidget *m_header = nullptr;
    QWidget *m_strip = nullptr;
    QString m_key;
    bool m_collapsed = false;
    int m_expandedWidth = 310;
};
