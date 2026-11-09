/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "FirmwareDialog.h"

#include "Theme.h"

#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDir>
#include <QFontDatabase>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPointer>
#include <QProgressBar>
#include <QPushButton>
#include <QThread>
#include <QVBoxLayout>

namespace {
const char *const kWarn = "#ffcc80";
const char *const kBad = "#ff8a80";
const char *const kGood = "#7fe3a0";

QString Colored(const char *color, const QString &text) {
    return QStringLiteral("<span style='color:%1;'>%2</span>").arg(QLatin1String(color), text.toHtmlEscaped());
}
QString Hex(quint32 v) { return QStringLiteral("%1").arg(v, 8, 16, QChar('0')); }
}  // namespace

FirmwareDialog::FirmwareDialog(QWidget *parent) : QDialog(parent) {
    setObjectName(QStringLiteral("ECamFirmwareDialog"));
    setWindowTitle(QStringLiteral("Update Board Firmware"));
    setMinimumWidth(620);
    setStyleSheet(theme::DialogSheet() + QStringLiteral(
        "QLabel { color: #e6ebf3; background: transparent; }"
        "QPlainTextEdit#ECamFwLog { background: #121418; color: #c8d0da; border: 1px solid #3d4650; border-radius: 2px; }"
        "QProgressBar { background: #1b1f25; border: 1px solid #3d4650; border-radius: 2px; }"
        "QProgressBar::chunk { background: #5a8fd4; }"));
    theme::ApplyAccent(this);
    m_package = fw::FindPackage();

    auto *v = new QVBoxLayout(this);
    v->setSpacing(10);
    m_carried = new QLabel(this);
    m_carried->setWordWrap(true);
    v->addWidget(m_carried);

    auto *grid = new QGridLayout();
    grid->setColumnStretch(1, 1);
    grid->addWidget(new QLabel(QStringLiteral("USB port"), this), 0, 0);
    m_port = new QComboBox(this);
    m_port->setObjectName(QStringLiteral("ECamFwPort"));
    grid->addWidget(m_port, 0, 1);
    m_refresh = new QPushButton(QStringLiteral("Refresh"), this);
    grid->addWidget(m_refresh, 0, 2);
    grid->addWidget(new QLabel(QStringLiteral("Board"), this), 1, 0, Qt::AlignTop);
    m_board = new QLabel(this);
    m_board->setObjectName(QStringLiteral("ECamFwBoard"));
    m_board->setWordWrap(true);
    m_board->setMinimumHeight(56);
    m_board->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    grid->addWidget(m_board, 1, 1, 1, 2);
    v->addLayout(grid);

    m_restore = new QCheckBox(this);
    m_restore->setObjectName(QStringLiteral("ECamFwRestore"));
    m_restore->hide();
    v->addWidget(m_restore);

    m_step = new QLabel(this);
    m_step->setObjectName(QStringLiteral("ECamFwStep"));
    m_step->setWordWrap(true);
    m_step->setTextFormat(Qt::RichText);
    v->addWidget(m_step);
    m_progress = new QProgressBar(this);
    m_progress->setRange(0, 100);
    m_progress->setValue(0);
    m_progress->setTextVisible(false);
    m_progress->setMaximumHeight(8);
    m_progress->hide();
    v->addWidget(m_progress);
    m_log = new QPlainTextEdit(this);
    m_log->setObjectName(QStringLiteral("ECamFwLog"));
    m_log->setReadOnly(true);
    m_log->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    m_log->setMinimumHeight(150);
    v->addWidget(m_log, 1);

    auto *buttons = new QHBoxLayout();
    buttons->addStretch(1);
    m_update = new QPushButton(QStringLiteral("Update"), this);
    m_update->setObjectName(QStringLiteral("ECamFwUpdate"));
    m_update->setProperty("role", "primary");
    m_update->setEnabled(false);
    m_update->setAutoDefault(false);
    m_close = new QPushButton(QStringLiteral("Close"), this);
    m_close->setAutoDefault(false);
    buttons->addWidget(m_update);
    buttons->addWidget(m_close);
    v->addLayout(buttons);

    connect(m_refresh, &QPushButton::clicked, this, &FirmwareDialog::refreshPorts);
    connect(m_port, &QComboBox::currentIndexChanged, this, [this]() { survey(); });
    connect(m_update, &QPushButton::clicked, this, &FirmwareDialog::startUpdate);
    connect(m_close, &QPushButton::clicked, this, &QDialog::reject);

    if (m_package.ok) {
        m_carried->setText(QStringLiteral("This app carries ClearCore firmware <b>%1</b>. It goes on over USB; a board's "
                                          "network and other settings stay.").arg(m_package.version.toHtmlEscaped()));
    } else {
        m_carried->setText(Colored(kBad, m_package.error));
    }
    refreshPorts();
}

void FirmwareDialog::refreshPorts() {
    const QString was = m_port->currentData().toString();
    m_ports = fw::ClearCorePorts();
    {
        const QSignalBlocker block(m_port);
        m_port->clear();
        for (const fw::Port &p : m_ports) {
            m_port->addItem(p.bootloader ? QStringLiteral("%1 (ClearCore, in its bootloader)").arg(p.name)
                                         : QStringLiteral("%1 (ClearCore)").arg(p.name),
                            p.name);
            if (p.name == was) m_port->setCurrentIndex(m_port->count() - 1);
        }
    }
    survey();
}

void FirmwareDialog::survey() {
    m_surveyed = false;
    m_update->setEnabled(false);
    m_restore->hide();
    const int i = m_port->currentIndex();
    if (i < 0 || i >= m_ports.size()) {
        m_board->setText(QStringLiteral("No ClearCore on USB. Connect its USB cable and press Refresh. A board that won't "
                                        "show up: double-tap its reset button (the IO-0 to IO-5 lights walk) and press Refresh."));
        return;
    }
    m_board->setText(QStringLiteral("Reading the board..."));
    const int id = ++m_surveyId;
    const fw::Port port = m_ports[i];
    QPointer<FirmwareDialog> self(this);
    QThread *t = QThread::create([self, id, port]() {
        const fw::Survey s = fw::Inspect(port);
        const QList<fw::Backup> pending = fw::PendingBackups();
        if (!self) return;
        QMetaObject::invokeMethod(self.data(), [self, id, s, pending]() {
            if (!self || id != self->m_surveyId) return;
            self->m_survey = s;
            self->m_pending = pending;
            self->m_surveyed = true;
            self->showSurvey();
        }, Qt::QueuedConnection);
    });
    connect(t, &QThread::finished, t, &QObject::deleteLater);
    t->start();
}

void FirmwareDialog::showSurvey() {
    const fw::Survey &s = m_survey;
    QStringList text;
    bool can = m_package.ok;
    switch (s.kind) {
    case fw::Kind::Unreachable:
        text << Colored(kBad, QStringLiteral("Can't open the port: %1. Is another program using it?").arg(s.detail));
        can = false;
        break;
    case fw::Kind::Bootloader:
        text << QStringLiteral("In its bootloader, with no firmware running. Nothing can be read from it to save.");
        break;
    case fw::Kind::Other:
        text << QStringLiteral("Not running ECam firmware, so there's no profile to save. Updating replaces what's on it.");
        break;
    case fw::Kind::ECam:
        text << (s.version == m_package.version ? QStringLiteral("ECam firmware %1, the same as this app's.").arg(s.version.toHtmlEscaped())
                                                : Colored(kWarn, QStringLiteral("ECam firmware %1, this app needs %2.").arg(s.version, m_package.version)));
        text << (s.profile != 0 ? QStringLiteral("Profile %1 will be saved to this PC first and put back afterwards.").arg(Hex(s.profile))
                                : QStringLiteral("No profile on it, so nothing to save."));
        if (s.plcConnected) text << Colored(kWarn, QStringLiteral("A PLC is connected: its connection drops while the board restarts."));
        break;
    }
    if (!s.blocked.isEmpty()) {
        text << Colored(kBad, QStringLiteral("Can't update now: %1.").arg(s.blocked));
        can = false;
    }
    m_board->setText(text.join(QStringLiteral("<br>")));

    // A profile an earlier update didn't put back, for a board with none of its own.
    const bool noProfile = s.kind != fw::Kind::ECam || s.profile == 0;
    if (noProfile && !m_pending.isEmpty() && s.kind != fw::Kind::Unreachable) {
        const fw::Backup &b = m_pending.first();
        const int i = m_port->currentIndex();
        const QString serial = i >= 0 && i < m_ports.size() ? m_ports[i].serial : QString();
        m_restore->setText(QStringLiteral("Put back profile %1, saved %2 by an update that didn't finish")
                               .arg(Hex(b.hash), b.when.toString(QStringLiteral("yyyy-MM-dd HH:mm"))));
        // Ticked only when it's surely this board's.
        m_restore->setChecked(!serial.isEmpty() && serial == b.serial);
        m_restore->show();
    }
    m_update->setText(s.kind == fw::Kind::ECam && s.version == m_package.version ? QStringLiteral("Reinstall %1").arg(m_package.version)
                                                                                 : QStringLiteral("Update to %1").arg(m_package.version));
    m_update->setEnabled(can && !m_busy);
}

void FirmwareDialog::setBusy(bool on) {
    m_busy = on;
    m_port->setEnabled(!on);
    m_refresh->setEnabled(!on);
    m_update->setEnabled(!on);
    m_close->setEnabled(!on);
    m_restore->setEnabled(!on);
    m_progress->setVisible(on);
}

void FirmwareDialog::startUpdate() {
    const int i = m_port->currentIndex();
    if (i < 0 || i >= m_ports.size() || !m_surveyed) return;
    ++m_surveyId;  // an answer still on its way doesn't count
    m_done = false;
    m_log->clear();
    m_progress->setValue(0);
    setBusy(true);
    emit aboutToUpdate();

    const QString restore = m_restore->isVisible() && m_restore->isChecked() && !m_pending.isEmpty() ? m_pending.first().file : QString();
    auto *updater = new fw::Updater(m_package, m_ports[i], restore);
    connect(updater, &fw::Updater::step, this, [this](const QString &text) {
        m_step->setText(text.toHtmlEscaped());
        m_log->appendPlainText(text);
    });
    connect(updater, &fw::Updater::percent, m_progress, &QProgressBar::setValue);
    connect(updater, &fw::Updater::log, m_log, &QPlainTextEdit::appendPlainText);
    QPointer<FirmwareDialog> self(this);
    QThread *t = QThread::create([updater, self]() {
        const fw::Result r = updater->run();
        if (!self) return;
        QMetaObject::invokeMethod(self.data(), [self, r]() {
            if (!self) return;
            self->m_result = r;
            self->m_done = true;
            self->setBusy(false);
            self->m_step->setText(Colored(r.ok ? kGood : kBad, r.message));
            self->m_log->appendPlainText(r.message);
            if (!r.backupKept.isEmpty()) {
                self->m_log->appendPlainText(QStringLiteral("The profile is still in %1").arg(QDir::toNativeSeparators(r.backupKept)));
            }
            self->refreshPorts();
            emit self->updated();
        }, Qt::QueuedConnection);
    });
    connect(t, &QThread::finished, updater, &QObject::deleteLater);
    connect(t, &QThread::finished, t, &QObject::deleteLater);
    t->start();
}

// Not while it's writing: the window has the board's only way back.
void FirmwareDialog::closeEvent(QCloseEvent *event) {
    if (m_busy) {
        event->ignore();
        return;
    }
    QDialog::closeEvent(event);
}

void FirmwareDialog::reject() {
    if (!m_busy) QDialog::reject();
}
