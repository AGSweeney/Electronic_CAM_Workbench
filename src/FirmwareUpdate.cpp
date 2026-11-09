/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "FirmwareUpdate.h"

#include "ProfileText.h"

#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QProcess>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QStandardPaths>
#include <QThread>

#include <algorithm>

#ifdef Q_OS_WIN
#define NOMINMAX
#include <windows.h>
#endif

namespace fw {
namespace {

constexpr qint64 kAppOffset = 0x4000;      // after the bootloader
constexpr qint64 kStoreAddress = 0x78000;  // Firmware/ECamFirmware/FlashStore.h: the image ends below it
constexpr int kBootloaderWaitMs = 15000;
constexpr int kBootWaitMs = 40000;  // a DHCP board with no server takes a while to settle

// The board's console over USB, one command at a time.
class Console {
public:
    bool open(const QString &name, int waitMs, QString *error) {
        m_port.setPortName(name);
        m_port.setBaudRate(115200);
        QElapsedTimer t;
        t.start();
        // The port is there a moment before Windows will let it be opened.
        while (!m_port.open(QIODevice::ReadWrite)) {
            if (t.elapsed() >= waitMs) {
                if (error) *error = m_port.errorString();
                return false;
            }
            QThread::msleep(250);
        }
        m_port.setDataTerminalReady(true);
        // A line ending flushes anything half-typed. Telemetry goes off first: the Workbench leaves it running on
        // USB, and part of a TEL line left in the buffer would land in the next reply.
        m_port.write("\r\ntel 0\r\n");
        readUntilQuiet();
        return true;
    }

    // True on "ok"; the lines before it, or the "error:" line, are in *lines.
    bool ask(const QString &line, int timeoutMs, QStringList *lines) {
        lines->clear();
        m_rx.clear();
        m_port.write(line.toUtf8() + "\r\n");
        m_port.flush();
        QElapsedTimer t;
        t.start();
        while (t.elapsed() < timeoutMs) {
            if (m_port.bytesAvailable() == 0 && !m_port.waitForReadyRead(100)) {
                if (m_port.error() != QSerialPort::NoError && m_port.error() != QSerialPort::TimeoutError) return false;
                continue;
            }
            m_rx += m_port.readAll();
            int nl;
            while ((nl = m_rx.indexOf('\n')) >= 0) {
                const QString l = QString::fromUtf8(m_rx.left(nl)).trimmed();
                m_rx.remove(0, nl + 1);
                if (l.isEmpty() || l.startsWith(QStringLiteral("TEL "))) continue;
                if (l == QStringLiteral("ok") || l.startsWith(QStringLiteral("ok "))) {
                    if (l.size() > 2) lines->append(l);
                    return true;
                }
                lines->append(l);
                if (l.startsWith(QStringLiteral("error:"))) return false;
            }
        }
        return false;
    }

private:
    void readUntilQuiet() {
        QElapsedTimer t;
        t.start();
        while (t.elapsed() < 1500 && m_port.waitForReadyRead(250)) m_port.readAll();
        m_port.clear(QSerialPort::Input);
    }

    QSerialPort m_port;
    QByteArray m_rx;
};

QString VersionFromReply(const QStringList &lines) {
    static const QRegularExpression re(QStringLiteral("^ECam_Firmware (\\S+)$"));
    for (const QString &l : lines) {
        const auto m = re.match(l);
        if (m.hasMatch()) return m.captured(1);
    }
    return {};
}

QHash<QString, QString> Fields(const QStringList &lines) {
    QHash<QString, QString> kv;
    for (const QString &l : lines) {
        if (!l.startsWith(QStringLiteral("state="))) continue;
        for (const QString &tok : l.split(QLatin1Char(' '), Qt::SkipEmptyParts)) {
            const int eq = tok.indexOf(QLatin1Char('='));
            if (eq > 0) kv.insert(tok.left(eq), tok.mid(eq + 1));
        }
    }
    return kv;
}

quint32 HashOf(const QStringList &lines) {
    quint32 h = ecam::ProfileHashBegin();
    for (const QString &l : lines) h = ecam::ProfileHashLine(h, l.toLatin1().constData());
    return h;
}

QString Hex(quint32 v) { return QStringLiteral("%1").arg(v, 8, 16, QChar('0')); }

// The port of one board, by its USB serial number when both sides have one.
bool SameBoard(const Port &p, const QString &serial) { return serial.isEmpty() || p.serial.isEmpty() || p.serial == serial; }

bool Restore(Console &c, const Backup &b, QString *error) {
    QStringList r;
    if (!c.ask(QStringLiteral("load"), 3000, &r)) {
        *error = QStringLiteral("load: %1").arg(r.isEmpty() ? QStringLiteral("no reply") : r.last());
        return false;
    }
    for (const QString &l : b.lines) {
        if (!c.ask(l, 3000, &r)) {
            *error = QStringLiteral("the new firmware refused \"%1\": %2").arg(l.left(40), r.isEmpty() ? QStringLiteral("no reply") : r.last());
            return false;
        }
    }
    if (!c.ask(QStringLiteral("commit"), 10000, &r) && r.isEmpty()) {
        *error = QStringLiteral("commit: no reply");
        return false;
    }
    const QString reply = r.join(QLatin1Char(' '));
    if (!reply.contains(QStringLiteral("hash=") + Hex(b.hash))) {
        *error = QStringLiteral("the board's hash doesn't match (%1)").arg(reply.left(80));
        return false;
    }
    if (!reply.contains(QStringLiteral("stored=1"))) {
        *error = QStringLiteral("the profile loaded but wasn't kept in flash (%1)").arg(reply.left(80));
        return false;
    }
    return true;
}

}  // namespace

bool IsClearCore(quint16 vendor, quint16 product) {
    return vendor == kVendorId && (product == kAppProductId || product == kBootProductId);
}

QString VersionInImage(const QByteArray &image) {
    static const QRegularExpression re(QStringLiteral("ECam_Firmware (\\d+\\.\\d+\\.\\d+)"));
    const auto m = re.match(QString::fromLatin1(image));
    return m.hasMatch() ? m.captured(1) : QString();
}

QString CheckImage(const QByteArray &image) {
    if (image.size() < 1024) return QStringLiteral("the firmware image is empty or cut short");
    if (image.size() > kStoreAddress - kAppOffset) return QStringLiteral("the firmware image is too big for the board");
    // A Cortex-M image starts with its stack pointer, in RAM at 0x2000_0000.
    if (static_cast<quint8>(image[3]) != 0x20) return QStringLiteral("the file is not a ClearCore firmware image");
    return {};
}

Package FindPackage() {
    Package p;
    const QString appDir = QCoreApplication::applicationDirPath();
    QStringList binDirs, bossacs;
    const QString override = qEnvironmentVariable("ECAM_FIRMWARE_DIR");
    if (!override.isEmpty()) binDirs << override;
    binDirs << appDir + QStringLiteral("/firmware");
    bossacs << appDir + QStringLiteral("/firmware/bossac/bossac.exe");
    // From a build tree: up to the repository.
    QDir up(appDir);
    for (int i = 0; i < 5 && up.cdUp(); ++i) {
        binDirs << up.filePath(QStringLiteral("Firmware/ECamFirmware/Release"));
        bossacs << up.filePath(QStringLiteral("tools/bossac/bossac.exe"));
    }
    for (const QString &d : binDirs) {
        const QString bin = QDir(d).filePath(QStringLiteral("ECam_Firmware.bin"));
        if (!QFileInfo::exists(bin)) continue;
        p.bin = bin;
        const QString uf2 = QDir(d).filePath(QStringLiteral("ECam_Firmware.uf2"));
        if (QFileInfo::exists(uf2)) p.uf2 = uf2;
        break;
    }
    for (const QString &b : bossacs) {
        if (QFileInfo::exists(b)) {
            p.bossac = b;
            break;
        }
    }
    if (p.bin.isEmpty()) {
        p.error = QStringLiteral("The firmware isn't with this copy of the app (no firmware\\ECam_Firmware.bin).");
        return p;
    }
    if (p.bossac.isEmpty()) {
        p.error = QStringLiteral("The flasher isn't with this copy of the app (no firmware\\bossac\\bossac.exe).");
        return p;
    }
    QFile f(p.bin);
    if (!f.open(QIODevice::ReadOnly)) {
        p.error = QStringLiteral("Can't read %1: %2").arg(p.bin, f.errorString());
        return p;
    }
    const QByteArray image = f.readAll();
    p.error = CheckImage(image);
    if (!p.error.isEmpty()) return p;
    p.version = VersionInImage(image);
    p.ok = true;  // an image with another name still flashes (the bench test does)
    return p;
}

QList<Port> ClearCorePorts() {
    QList<Port> ports;
    for (const QSerialPortInfo &i : QSerialPortInfo::availablePorts()) {
        if (!IsClearCore(i.vendorIdentifier(), i.productIdentifier())) continue;
        ports.append({i.portName(), i.serialNumber(), i.productIdentifier() == kBootProductId});
    }
    return ports;
}

Survey Inspect(const Port &port) {
    Survey s;
    if (port.bootloader) {
        s.kind = Kind::Bootloader;
        return s;
    }
    Console c;
    if (!c.open(port.name, 2000, &s.detail)) return s;
    QStringList r;
    const bool ok = c.ask(QStringLiteral("version"), 1500, &r);
    s.version = ok ? VersionFromReply(r) : QString();
    if (s.version.isEmpty()) {
        s.kind = Kind::Other;
        s.detail = r.join(QLatin1Char(' ')).left(80);
        return s;
    }
    s.kind = Kind::ECam;
    if (!c.ask(QStringLiteral("status"), 2000, &r)) {
        s.blocked = QStringLiteral("the board didn't answer status");
        return s;
    }
    const auto kv = Fields(r);
    s.profile = kv.value(QStringLiteral("profile")).toUInt(nullptr, 16);
    s.stored = kv.value(QStringLiteral("stored")) == QStringLiteral("1");
    const QString eip = kv.value(QStringLiteral("eip"));
    s.plcConnected = eip == QStringLiteral("run") || eip == QStringLiteral("idle");
    const QString tl = kv.value(QStringLiteral("tl"));
    const bool moving = kv.value(QStringLiteral("crank"), QStringLiteral("off")) != QStringLiteral("off") ||
                        tl == QStringLiteral("running") || tl == QStringLiteral("stopping") || tl == QStringLiteral("jogging");
    if (eip == QStringLiteral("control")) {
        s.blocked = QStringLiteral("a PLC is in control of the board");
    } else if (kv.value(QStringLiteral("enabled")) == QStringLiteral("1")) {
        s.blocked = QStringLiteral("the motors are enabled");
    } else if (moving) {
        s.blocked = QStringLiteral("the board is running");
    }
    return s;
}

QString BackupDir() {
    const QString override = qEnvironmentVariable("ECAM_FIRMWARE_BACKUPS");
    if (!override.isEmpty()) return override;
    return QDir(QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation)).filePath(QStringLiteral("firmware_backups"));
}

bool SaveBackup(Backup *b, QString *error) {
    QDir dir(BackupDir());
    if (!dir.mkpath(QStringLiteral("."))) {
        *error = QStringLiteral("can't make %1").arg(dir.path());
        return false;
    }
    b->when = QDateTime::currentDateTime();
    b->file = dir.filePath(QStringLiteral("profile_%1_%2.txt").arg(Hex(b->hash), b->when.toString(QStringLiteral("yyyyMMdd_HHmmss"))));
    QSaveFile f(b->file);
    if (!f.open(QIODevice::WriteOnly)) {
        *error = f.errorString();
        return false;
    }
    QByteArray text = QStringLiteral("# ECam profile backup\n# hash=%1\n# serial=%2\n# firmware=%3\n# date=%4\n")
                          .arg(Hex(b->hash), b->serial, b->firmware, b->when.toString(Qt::ISODate))
                          .toUtf8();
    for (const QString &l : b->lines) text += l.toLatin1() + '\n';
    f.write(text);
    if (!f.commit()) {
        *error = f.errorString();
        return false;
    }
    return true;
}

bool LoadBackup(const QString &file, Backup *b, QString *error) {
    QFile f(file);
    if (!f.open(QIODevice::ReadOnly)) {
        *error = f.errorString();
        return false;
    }
    *b = Backup();
    b->file = file;
    bool haveHash = false;
    for (const QByteArray &raw : f.readAll().split('\n')) {
        const QString l = QString::fromLatin1(raw).trimmed();
        if (l.isEmpty()) continue;
        if (!l.startsWith(QLatin1Char('#'))) {
            b->lines << l;
            continue;
        }
        const QString field = l.mid(1).trimmed();
        const QString value = field.section(QLatin1Char('='), 1);
        if (field.startsWith(QStringLiteral("hash="))) {
            b->hash = value.toUInt(nullptr, 16);
            haveHash = true;
        } else if (field.startsWith(QStringLiteral("serial="))) {
            b->serial = value;
        } else if (field.startsWith(QStringLiteral("firmware="))) {
            b->firmware = value;
        } else if (field.startsWith(QStringLiteral("date="))) {
            b->when = QDateTime::fromString(value, Qt::ISODate);
        }
    }
    if (!haveHash || b->lines.isEmpty() || HashOf(b->lines) != b->hash) {
        *error = QStringLiteral("the backup is damaged (its lines don't match its hash)");
        return false;
    }
    return true;
}

QList<Backup> PendingBackups() {
    QList<Backup> all;
    const QDir dir(BackupDir());
    for (const QString &name : dir.entryList({QStringLiteral("profile_*.txt")}, QDir::Files)) {
        Backup b;
        QString error;
        if (LoadBackup(dir.filePath(name), &b, &error)) all << b;
    }
    std::sort(all.begin(), all.end(), [](const Backup &a, const Backup &b) { return a.when > b.when; });
    return all;
}

Updater::Updater(const Package &package, const Port &port, const QString &restoreFile, QObject *parent)
    : QObject(parent), m_package(package), m_port(port), m_restoreFile(restoreFile) {}

bool Updater::bossac(const QStringList &args, int timeoutMs, bool quiet) {
    QProcess p;
    p.setProcessChannelMode(QProcess::MergedChannels);
#ifdef Q_OS_WIN
    p.setCreateProcessArgumentsModifier([](QProcess::CreateProcessArguments *a) { a->flags |= CREATE_NO_WINDOW; });
#endif
    p.start(m_package.bossac, args);
    if (!p.waitForStarted(5000)) {
        emit log(QStringLiteral("bossac didn't start: %1").arg(p.errorString()));
        return false;
    }
    static const QRegularExpression bar(QStringLiteral("\\((\\d+)/(\\d+) pages\\)|(\\d+)%"));
    QElapsedTimer t;
    t.start();
    QByteArray rx;
    auto drain = [&]() {
        rx += p.readAll();
        int end;
        // Its progress bar rewrites one line with carriage returns.
        while ((end = rx.indexOf('\n')) >= 0 || (end = rx.indexOf('\r')) >= 0) {
            const QString l = QString::fromLocal8Bit(rx.left(end)).trimmed();
            rx.remove(0, end + 1);
            if (l.isEmpty()) continue;
            const auto m = bar.match(l);
            if (m.hasMatch() && l.startsWith(QLatin1Char('['))) {
                const int pct = m.captured(3).isEmpty() ? m.captured(1).toInt() * 100 / std::max(1, m.captured(2).toInt()) : m.captured(3).toInt();
                emit percent(pct);
            } else if (!quiet) {
                emit log(l);
            }
        }
    };
    while (p.state() != QProcess::NotRunning) {
        p.waitForReadyRead(200);
        drain();
        if (t.elapsed() > timeoutMs) {
            p.kill();
            p.waitForFinished(2000);
            emit log(QStringLiteral("bossac timed out"));
            return false;
        }
    }
    drain();
    return p.exitStatus() == QProcess::NormalExit && p.exitCode() == 0;
}

bool Updater::enterBootloader(Port *boot, QString *error) {
    QStringList before;
    for (const Port &p : ClearCorePorts()) {
        if (p.bootloader) before << p.name;
    }
    emit step(QStringLiteral("Starting the bootloader..."));
    // Opening at 1200 baud is the request. The board leaves at once, so bossac reports that it can't reach the
    // port: that isn't a fault, and whether it worked is whether the bootloader's port appears.
    bossac({QStringLiteral("--info"), QStringLiteral("--arduino-erase"), QStringLiteral("--port=") + m_port.name}, 20000, true);
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < kBootloaderWaitMs) {
        for (const Port &p : ClearCorePorts()) {
            if (p.bootloader && !before.contains(p.name) && SameBoard(p, m_port.serial)) {
                *boot = p;
                return true;
            }
        }
        QThread::msleep(250);
    }
    *error = QStringLiteral("The board didn't start its bootloader. Double-tap the reset button on the ClearCore "
                            "(the IO-0 to IO-5 lights walk), then update again.");
    return false;
}

Result Updater::run() {
    Result res;
    QStringList others;  // other ClearCores: never the one to talk to afterwards
    for (const Port &p : ClearCorePorts()) {
        if (p.name != m_port.name && !p.bootloader) others << p.name;
    }

    // 1. Look again, it may have changed since the window showed it.
    emit step(QStringLiteral("Reading the board..."));
    const Survey s = Inspect(m_port);
    if (s.kind == Kind::Unreachable) {
        res.message = QStringLiteral("Can't open %1: %2").arg(m_port.name, s.detail);
        return res;
    }
    if (!s.blocked.isEmpty()) {
        res.message = QStringLiteral("Not updated: %1.").arg(s.blocked);
        return res;
    }

    // 2. The profile, to a file, before anything is erased.
    Backup backup;
    bool haveBackup = false;
    if (s.kind == Kind::ECam && s.profile != 0) {
        emit step(QStringLiteral("Saving the board's profile..."));
        Console c;
        QString error;
        QStringList lines;
        if (!c.open(m_port.name, 2000, &error)) {
            res.message = QStringLiteral("Can't open %1: %2").arg(m_port.name, error);
            return res;
        }
        if (!c.ask(QStringLiteral("download"), 15000, &lines)) {
            res.message = QStringLiteral("Not updated: the board's profile could not be read (%1). Nothing was erased.")
                              .arg(lines.isEmpty() ? QStringLiteral("no reply") : lines.last());
            return res;
        }
        backup.lines = lines;
        backup.hash = HashOf(lines);
        backup.serial = m_port.serial;
        backup.firmware = s.version;
        if (backup.hash != s.profile) {
            res.message = QStringLiteral("Not updated: the profile read back as %1, the board says %2. Nothing was erased.")
                              .arg(Hex(backup.hash), Hex(s.profile));
            return res;
        }
        if (!SaveBackup(&backup, &error)) {
            res.message = QStringLiteral("Not updated: the backup could not be saved (%1). Nothing was erased.").arg(error);
            return res;
        }
        haveBackup = true;
        res.backupKept = backup.file;
        emit log(QStringLiteral("Profile %1 saved to %2").arg(Hex(backup.hash), QDir::toNativeSeparators(backup.file)));
    } else if (!m_restoreFile.isEmpty()) {
        QString error;
        if (!LoadBackup(m_restoreFile, &backup, &error)) {
            res.message = QStringLiteral("Not updated: %1.").arg(error);
            return res;
        }
        haveBackup = true;
        res.backupKept = backup.file;
    }

    // 3. Flash.
    Port boot = m_port;
    if (!m_port.bootloader) {
        QString error;
        if (!enterBootloader(&boot, &error)) {
            res.message = error;
            return res;
        }
    }
    emit log(QStringLiteral("Bootloader on %1").arg(boot.name));
    emit step(QStringLiteral("Writing firmware %1...").arg(m_package.version));
    res.flashed = true;  // from here the chip may be erased
    if (!bossac({QStringLiteral("--info"), QStringLiteral("--port=") + boot.name, QStringLiteral("--usb-port"), QStringLiteral("--write"),
                 QStringLiteral("--erase"), QStringLiteral("--verify"), QStringLiteral("--offset=0x4000"), QStringLiteral("--reset"),
                 QDir::toNativeSeparators(m_package.bin)},
                120000)) {
        res.message = QStringLiteral("Flashing failed and the board has no working firmware. Double-tap its reset button "
                                     "(the IO-0 to IO-5 lights walk) and press Refresh, then update again%1.")
                          .arg(haveBackup ? QStringLiteral(": the saved profile will be offered") : QString());
        return res;
    }

    // 4. The new firmware, on the same board.
    emit step(QStringLiteral("Waiting for the board to start..."));
    Console c;
    QStringList r;
    QString running, error;
    QElapsedTimer t;
    t.start();
    bool opened = false;
    while (!opened && t.elapsed() < kBootWaitMs) {
        for (const Port &p : ClearCorePorts()) {
            if (p.bootloader || others.contains(p.name) || !SameBoard(p, m_port.serial)) continue;
            if (c.open(p.name, 3000, &error)) {
                opened = true;
                break;
            }
        }
        if (!opened) QThread::msleep(500);
    }
    if (!opened) {
        res.message = QStringLiteral("The firmware was written, but the board's port didn't come back%1.").arg(error.isEmpty() ? QString() : QStringLiteral(" (%1)").arg(error));
        return res;
    }
    for (int i = 0; i < 6 && running.isEmpty(); ++i) {
        if (c.ask(QStringLiteral("version"), 2000, &r)) running = VersionFromReply(r);
    }
    if (m_package.version.isEmpty()) {
        // An image that isn't ECam firmware: only the bench test does this, and there's nothing to restore to.
        res.ok = true;
        res.message = QStringLiteral("Flashed a firmware that isn't ECam.");
        return res;
    }
    if (running != m_package.version) {
        res.message = QStringLiteral("The board reports firmware %1, not %2.").arg(running.isEmpty() ? QStringLiteral("nothing") : running, m_package.version);
        return res;
    }

    // 5. The profile back, and only then the file gone.
    QString done = QStringLiteral("Firmware %1 is running.").arg(running);
    if (haveBackup) {
        emit step(QStringLiteral("Restoring the profile..."));
        if (!Restore(c, backup, &error)) {
            res.message = QStringLiteral("%1 The profile was not restored: %2. It is saved in %3.")
                              .arg(done, error, QDir::toNativeSeparators(backup.file));
            return res;
        }
        QFile::remove(backup.file);
        res.backupKept.clear();
        done += QStringLiteral(" Profile %1 restored and kept in flash.").arg(Hex(backup.hash));
    } else {
        done += QStringLiteral(" No profile on the board yet.");
    }
    res.ok = true;
    res.message = done;
    return res;
}

}  // namespace fw
