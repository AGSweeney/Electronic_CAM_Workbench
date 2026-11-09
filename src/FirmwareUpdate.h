/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Puts the firmware the app was built with on a ClearCore over USB, keeping the board's profile.
//
// Flashing erases the chip and the stored profile with it (the network and other settings are in NVM and
// stay). So the profile goes to a file first, checked against the board's hash, and that file only goes
// away once the new firmware holds the same hash in flash. If anything stops the update, the file is
// still there and the next update offers it.
//
// Everything here blocks: run it off the UI thread.

#include <QDateTime>
#include <QList>
#include <QObject>
#include <QString>
#include <QStringList>

namespace fw {

// A ClearCore's two USB identities. Vendor 0x2890 alone is not enough: ClearPath motors have it too.
constexpr quint16 kVendorId = 0x2890;
constexpr quint16 kAppProductId = 0x8022;   // running firmware
constexpr quint16 kBootProductId = 0x0022;  // in its bootloader
bool IsClearCore(quint16 vendor, quint16 product);

// What the app carries: firmware\ next to the exe when installed, the build folders when developing.
// ECAM_FIRMWARE_DIR in the environment points somewhere else (the bench test flashes other images).
struct Package {
    bool ok = false;
    QString error;
    QString bin;
    QString uf2;
    QString bossac;
    QString version;
};
Package FindPackage();
// The version in a firmware image, empty if it isn't an ECam image.
QString VersionInImage(const QByteArray &image);
// Why an image can't go on the board, empty if it can.
QString CheckImage(const QByteArray &image);

struct Port {
    QString name;
    QString serial;
    bool bootloader = false;
};
QList<Port> ClearCorePorts();

enum class Kind { Unreachable, Bootloader, ECam, Other };
struct Survey {
    Kind kind = Kind::Unreachable;
    QString version;  // ECam firmware's version
    QString detail;   // why it can't be reached, or what other firmware said
    quint32 profile = 0;
    bool stored = false;
    QString blocked;  // why it can't be updated now, empty if it can
    bool plcConnected = false;
};
Survey Inspect(const Port &port);

// A saved profile. The file is the upload's lines with a header of # lines.
struct Backup {
    QString file;
    QStringList lines;
    quint32 hash = 0;
    QString serial;
    QString firmware;
    QDateTime when;
};
QString BackupDir();
bool SaveBackup(Backup *backup, QString *error);
bool LoadBackup(const QString &file, Backup *backup, QString *error);
// Saved profiles an update didn't finish putting back, newest first.
QList<Backup> PendingBackups();

struct Result {
    bool ok = false;
    bool flashed = false;
    QString message;
    QString backupKept;  // the profile's file, if it's not on the board
};

class Updater : public QObject {
    Q_OBJECT
public:
    // restoreFile: a backup to put on after flashing, when the board has no profile of its own to save.
    Updater(const Package &package, const Port &port, const QString &restoreFile, QObject *parent = nullptr);
    Result run();

signals:
    void step(const QString &text);
    void percent(int value);
    void log(const QString &line);

private:
    bool bossac(const QStringList &args, int timeoutMs, bool quiet = false);
    bool enterBootloader(Port *boot, QString *error);

    Package m_package;
    Port m_port;
    QString m_restoreFile;
};

}  // namespace fw
