/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// What the user sees first: the splash screen while the program loads, then the start page
// (new, guided setup, open, an example, or a recent project).

#include <QDialog>
#include <QString>
#include <QStringList>

#include <vector>

class QCheckBox;
class QListWidget;
class QListWidgetItem;
class QSplashScreen;

namespace startup {

// Minimum time the splash stays up, so it reads as a splash and not a flicker.
constexpr int kSplashMs = 2000;

// The splash screen (logo, product name), shown and painted; the caller finishes it.
QSplashScreen *ShowSplash();

// Recent projects, newest first, only files that still exist.
QStringList RecentProjects();
void AddRecentProject(const QString &path);
void RemoveRecentProject(const QString &path);

// Show the start page when the program starts (the start page's own checkbox).
bool ShowStartPageAtStartup();

// Example projects bundled in the resources (":/samples/<file>").
struct Example {
    QString file;   // resource path
    QString title;
    QString about;
};
const std::vector<Example> &Examples();

class StartDialog : public QDialog {
    Q_OBJECT
public:
    enum class Choice { None, New, Wizard, Open, Recent, Example };

    explicit StartDialog(QWidget *parent = nullptr);

    Choice choice() const { return m_choice; }
    QString path() const { return m_path; }  // Recent: the file; Example: the resource
    QString title() const { return m_title; }  // Example: its title

private:
    void Pick(Choice c, const QString &path = {}, const QString &title = {});
    QListWidget *m_recent = nullptr;
    QListWidget *m_examples = nullptr;
    QCheckBox *m_atStartup = nullptr;
    Choice m_choice = Choice::None;
    QString m_path, m_title;
};

}  // namespace startup
