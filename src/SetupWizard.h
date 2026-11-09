/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Guided setup: walks a new user from a machine motion (template) to a working project:
//   Template   which motion the machine needs
//   Master     what times the cycle: an encoder or driven crank, a trigger, or the line (cut-off)
//   Motor      which motor connector and the mechanics between motor and load, one step per motor
//              (the flying cut-off has two: the carriage and the saw plunge)
//   Motion     stroke and where in the cycle it moves, with a live preview and the peaks
//   Review     the settings, any problems, and what to do next
// The result is an ordinary project (core/CamTemplates.h builds it) to edit and upload.

#include "CamTemplates.h"

#include <QDialog>

#include <functional>
#include <string>
#include <vector>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QListWidget;
class QPushButton;
class QRadioButton;
class QSpinBox;
class QStackedWidget;
class QWidget;
class MotionPreview;

class SetupWizard : public QDialog {
    Q_OBJECT
public:
    explicit SetupWizard(QWidget *parent = nullptr);

    ecam::CamProject project() const;
    QString templateName() const;
    const ecam::TemplateSpec &spec() const { return m_spec; }

    // Driving it (tests): pick a template, step forward.
    void selectTemplate(ecam::TemplateKind kind);
    int page() const { return m_page; }  // position among the steps this template shows
    int pageCount() const { return static_cast<int>(Visible().size()); }
    QString pageTitle() const;           // the current step's name
    void next();

private:
    // Every page, in order; a template shows a subset (Visible).
    enum Page { kTemplatePage, kMasterPage, kMotorPage, kSawMotorPage, kMotionPage, kReviewPage, kPages };
    std::vector<int> Visible() const;

    // The spec fields one motor page edits.
    struct MotorFields {
        std::string ecam::TemplateSpec::*name;
        std::string ecam::TemplateSpec::*units;
        int ecam::TemplateSpec::*connector;
        ecam::AxisDrive ecam::TemplateSpec::*drive;
        double ecam::TemplateSpec::*motorPpr;
        double ecam::TemplateSpec::*pitch;
        double ecam::TemplateSpec::*pulley;
        double ecam::TemplateSpec::*countsPerUnit;
        double ecam::TemplateSpec::*velMax;
        double ecam::TemplateSpec::*accMax;
    };
    struct MotorUi {
        MotorFields f{};
        QLabel *heading = nullptr;
        QLabel *help = nullptr;
        QCheckBox *enable = nullptr;  // the saw: whether there is one
        QWidget *fields = nullptr;
        QComboBox *units = nullptr;
        QComboBox *drive = nullptr;
        QFormLayout *form = nullptr;
        QDoubleSpinBox *motorPpr = nullptr;
        QDoubleSpinBox *pitch = nullptr;
        QDoubleSpinBox *pulley = nullptr;
        QDoubleSpinBox *direct = nullptr;
        QDoubleSpinBox *velMax = nullptr;
        QDoubleSpinBox *accMax = nullptr;
        QLabel *derived = nullptr;
        QLabel *warn = nullptr;
    };

    QWidget *BuildTemplatePage();
    QWidget *BuildMasterPage();
    QWidget *BuildMotorPage(int which);  // 0 the follower, 1 the saw plunge
    QWidget *BuildMotionPage();
    QWidget *BuildReviewPage();

    // Widgets bound to a spec field: loaded by Load(), written back on every change.
    QDoubleSpinBox *Num(double ecam::TemplateSpec::*field, double min, double max, int decimals, const QString &suffix = {});
    QSpinBox *Int(int ecam::TemplateSpec::*field, int min, int max);
    QComboBox *Connector(int ecam::TemplateSpec::*field);
    QCheckBox *Flag(bool ecam::TemplateSpec::*field, const QString &text);

    void Load();     // spec -> widgets
    void Changed();  // a widget changed the spec
    void Refresh();  // visibility, derived values, preview, review
    void GoTo(int page);
    QStringList Problems(QStringList *warnings) const;

    ecam::TemplateSpec m_spec;
    bool m_loading = false;
    int m_page = 0;
    std::vector<std::function<void()>> m_loaders;

    // Frame
    std::vector<QLabel *> m_steps;
    QStackedWidget *m_pages = nullptr;
    QPushButton *m_back = nullptr;
    QPushButton *m_next = nullptr;

    // 1 Template
    QListWidget *m_templates = nullptr;
    QLabel *m_tplName = nullptr;
    QLabel *m_tplAbout = nullptr;
    QLabel *m_tplSetup = nullptr;
    MotionPreview *m_tplPreview = nullptr;

    // 2 Master
    QStackedWidget *m_masterKind = nullptr;  // crank / timeline / line
    QRadioButton *m_srcEncoder = nullptr;
    QRadioButton *m_srcDriven = nullptr;
    QWidget *m_encoderBox = nullptr;
    QWidget *m_drivenBox = nullptr;
    QLabel *m_crankDerived = nullptr;
    QComboBox *m_trigger = nullptr;
    QComboBox *m_triggerInput = nullptr;
    QComboBox *m_triggerEdge = nullptr;
    QLabel *m_lineDerived = nullptr;

    // Motors: the follower, the saw plunge
    MotorUi m_motor[2];
    QCheckBox *m_sawOutput = nullptr;

    // 4 Motion
    QFormLayout *m_motionForm = nullptr;
    QComboBox *m_law = nullptr;
    QLabel *m_lawHelp = nullptr;
    QLabel *m_strokeLabel = nullptr;
    QDoubleSpinBox *m_stroke = nullptr;
    QStackedWidget *m_phases = nullptr;  // per template
    QLabel *m_flyDerived = nullptr;
    QFormLayout *m_flyForm = nullptr;
    QDoubleSpinBox *m_sawDepth = nullptr;
    MotionPreview *m_preview = nullptr;
    QLabel *m_peaks = nullptr;

    // 5 Review
    QLabel *m_review = nullptr;
};
