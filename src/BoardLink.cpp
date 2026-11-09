/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "BoardLink.h"

#include "ProjectIo.h"

#include "ProfileText.h"

#include <QNetworkInterface>
#include <QRegularExpression>

#include <memory>

BoardLink::BoardLink(QObject *parent) : QObject(parent) {
    qRegisterMetaType<BoardLink::State>();
    connect(&m_tcp, &QTcpSocket::readyRead, this, [this]() { onReadyRead(&m_tcp); });
    connect(&m_tcp, &QTcpSocket::disconnected, this, [this]() {
        if (m_transport == Transport::Ethernet) {
            disconnectLink();
            emit linkLost(QStringLiteral("board closed the connection"));
        }
    });
    connect(&m_serial, &QSerialPort::readyRead, this, [this]() { onReadyRead(&m_serial); });
    connect(&m_serial, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError e) {
        if (e == QSerialPort::ResourceError && m_transport == Transport::Serial) {
            disconnectLink();
            emit linkLost(QStringLiteral("USB port lost (board reset or unplugged?)"));
        }
    });
    connect(&m_udp, &QUdpSocket::readyRead, this, [this]() {
        while (m_udp.hasPendingDatagrams()) {
            QByteArray d;
            d.resize(int(m_udp.pendingDatagramSize()));
            QHostAddress from;
            m_udp.readDatagram(d.data(), d.size(), &from);
            const QString text = QString::fromUtf8(d).trimmed();
            if (!text.startsWith(QStringLiteral("ECAM "))) continue;
            const QRegularExpression ip(QStringLiteral("IP=(\\S+)"));
            const auto m = ip.match(text);
            emit deviceDiscovered(m.hasMatch() ? m.captured(1) : QHostAddress(from.toIPv4Address()).toString(), text);
        }
    });
    m_pollTimer.setInterval(500);
    connect(&m_pollTimer, &QTimer::timeout, this, [this]() {
        if (isConnected() && !busy()) command(QStringLiteral("status"));
    });
    m_timeoutTimer.setSingleShot(true);
    m_timeoutTimer.setInterval(4000);
    connect(&m_timeoutTimer, &QTimer::timeout, this, [this]() {
        if (!m_active) return;
        m_lines << QStringLiteral("error: no reply from the board");
        finish(false);
    });
}

BoardLink::~BoardLink() {
    // Closing the socket emits signals; nothing may reach a window that is being destroyed.
    blockSignals(true);
    m_tcp.disconnect(this);
    m_serial.disconnect(this);
    m_queue.clear();
    m_active = false;
    m_tcp.abort();
    if (m_serial.isOpen()) m_serial.close();
}

bool BoardLink::connectEthernet(const QString &host, QString *error) {
    disconnectLink();
    m_tcp.connectToHost(host, kConsolePort);
    if (!m_tcp.waitForConnected(3000)) {
        if (error) *error = m_tcp.errorString();
        m_tcp.abort();
        return false;
    }
    m_transport = Transport::Ethernet;
    onConnected();
    return true;
}

bool BoardLink::connectSerial(const QString &port, QString *error) {
    disconnectLink();
    m_serial.setPortName(port);
    m_serial.setBaudRate(115200);
    if (!m_serial.open(QIODevice::ReadWrite)) {
        if (error) *error = m_serial.errorString();
        return false;
    }
    m_serial.setDataTerminalReady(true);
    m_transport = Transport::Serial;
    onConnected();
    return true;
}

void BoardLink::onConnected() {
    m_rx.clear();
    m_queue.clear();
    m_active = false;
    m_state = State();
    emit connectedChanged(true);
    // A line ending first flushes anything half-typed on the board's console.
    write("\r\n");
    command(QStringLiteral("tel 50"));
    command(QStringLiteral("status"));
    m_firmware.clear();
    command(QStringLiteral("version"), [this](bool ok, const QStringList &r) {
        for (const QString &l : r) {
            if (ok && l.startsWith(QStringLiteral("ECam_Firmware "))) m_firmware = l.section(QLatin1Char(' '), 1, 1);
        }
        emit stateUpdated(m_state);
    });
    m_pollTimer.start();
}

void BoardLink::disconnectLink() {
    const bool was = isConnected();
    m_pollTimer.stop();
    m_timeoutTimer.stop();
    // Cleared first, so our own close is not reported as a lost link.
    const Transport was_ = m_transport;
    m_transport = Transport::None;
    if (was_ == Transport::Ethernet) m_tcp.abort();
    if (m_serial.isOpen()) m_serial.close();
    // Fail what was waiting so callers are not left hanging.
    QQueue<Pending> q;
    q.swap(m_queue);
    if (m_active && m_current.done) m_current.done(false, {QStringLiteral("error: disconnected")});
    m_active = false;
    for (const Pending &p : q) {
        if (p.done) p.done(false, {QStringLiteral("error: disconnected")});
    }
    m_state = State();
    if (was) emit connectedChanged(false);
}

bool BoardLink::isConnected() const { return m_transport != Transport::None; }

void BoardLink::discover() {
    if (m_udp.state() != QAbstractSocket::BoundState) m_udp.bind(QHostAddress::AnyIPv4, 0);
    const QByteArray req("ECAM_DISCOVER?");
    m_udp.writeDatagram(req, QHostAddress::Broadcast, kDiscoveryPort);
    // Directed broadcasts per interface: Windows sends 255.255.255.255 out of one adapter only.
    for (const QNetworkInterface &nif : QNetworkInterface::allInterfaces()) {
        if (!(nif.flags() & QNetworkInterface::IsUp) || !(nif.flags() & QNetworkInterface::CanBroadcast)) continue;
        for (const QNetworkAddressEntry &e : nif.addressEntries()) {
            if (e.ip().protocol() == QAbstractSocket::IPv4Protocol && !e.broadcast().isNull()) {
                m_udp.writeDatagram(req, e.broadcast(), kDiscoveryPort);
            }
        }
    }
}

void BoardLink::write(const QByteArray &bytes) {
    if (m_transport == Transport::Ethernet) {
        m_tcp.write(bytes);
    } else if (m_transport == Transport::Serial) {
        m_serial.write(bytes);
    }
}

void BoardLink::command(const QString &line, Done done) {
    if (!isConnected()) {
        if (done) done(false, {QStringLiteral("error: not connected")});
        return;
    }
    m_queue.enqueue({line, std::move(done)});
    pump();
}

void BoardLink::pump() {
    if (m_active || m_queue.isEmpty() || !isConnected()) return;
    m_current = m_queue.dequeue();
    m_active = true;
    m_lines.clear();
    if (m_current.line != QStringLiteral("status")) emit logLine(QStringLiteral("> ") + m_current.line);
    write(m_current.line.toUtf8() + "\r\n");
    m_timeoutTimer.start();
}

void BoardLink::finish(bool ok) {
    m_timeoutTimer.stop();
    m_active = false;
    Pending p = m_current;
    m_current = Pending();
    // The reply is the callback's own copy: a command the callback sends starts at once, and pump() clears m_lines for it.
    const QStringList lines = m_lines;
    if (p.done) p.done(ok, lines);
    pump();
}

void BoardLink::onReadyRead(QIODevice *dev) {
    m_rx += dev->readAll();
    int nl;
    while ((nl = m_rx.indexOf('\n')) >= 0) {
        const QString line = QString::fromUtf8(m_rx.left(nl)).trimmed();
        m_rx.remove(0, nl + 1);
        if (!line.isEmpty()) handleLine(line);
    }
}

void BoardLink::handleLine(const QString &line) {
    if (line.startsWith(QStringLiteral("TEL "))) {
        parseFields(line, false);
        return;
    }
    if (!m_active) {
        emit logLine(line);  // banner, boot messages
        return;
    }
    m_lines << line;
    if (line.startsWith(QStringLiteral("state="))) parseFields(line, true);
    const bool ok = line == QStringLiteral("ok") || line.startsWith(QStringLiteral("ok "));
    if (ok || line.startsWith(QStringLiteral("error:"))) {
        if (m_current.line == QStringLiteral("download") && ok) {
            emit logLine(QStringLiteral("  %1 profile lines").arg(m_lines.size() - 1));
        } else if (!ok || m_current.line != QStringLiteral("status")) {
            for (const QString &l : m_lines) emit logLine(QStringLiteral("  ") + l);
        }
        finish(ok);
    }
}

void BoardLink::parseFields(const QString &line, bool full) {
    QHash<QString, QString> kv;
    for (const QString &tok : line.split(QLatin1Char(' '), Qt::SkipEmptyParts)) {
        const int eq = tok.indexOf(QLatin1Char('='));
        if (eq > 0) kv.insert(tok.left(eq), tok.mid(eq + 1));
    }
    State &s = m_state;
    s.valid = true;
    s.state = kv.value(QStringLiteral("state"), s.state);
    s.crank = kv.value(QStringLiteral("crank"), s.crank);
    s.timeline = kv.value(QStringLiteral("tl"), s.timeline);
    s.deg = kv.value(QStringLiteral("deg")).toDouble();
    s.rev = kv.value(QStringLiteral("rev")).toLong();
    s.rpm = kv.value(QStringLiteral("rpm")).toDouble();
    s.t = kv.value(QStringLiteral("t")).toDouble();
    s.cycle = kv.value(QStringLiteral("cycle")).toLong();
    s.override = kv.value(QStringLiteral("ovr"), QStringLiteral("1")).toDouble();
    s.outputs = kv.value(QStringLiteral("out"), QStringLiteral("0")).toUInt();
    if (kv.contains(QStringLiteral("enc"))) s.encoder = kv.value(QStringLiteral("enc")).toLongLong();
    s.cutGap = kv.contains(QStringLiteral("gap")) ? kv.value(QStringLiteral("gap")).toDouble() : -1.0;
    s.syncLeft = kv.contains(QStringLiteral("sync")) ? kv.value(QStringLiteral("sync")).toDouble() : -1.0;
    if (full) {
        s.enabled = kv.value(QStringLiteral("enabled")) == QStringLiteral("1");
        s.simulate = kv.value(QStringLiteral("sim")) == QStringLiteral("1");
        // The fault message has spaces ("motor fault on M-0"): it runs up to " estop=".
        {
            const int f = line.indexOf(QStringLiteral(" fault="));
            const int e = line.indexOf(QStringLiteral(" estop="));
            s.fault = (f >= 0 && e > f) ? line.mid(f + 7, e - f - 7) : kv.value(QStringLiteral("fault"));
        }
        s.estop = kv.value(QStringLiteral("estop"));
        s.a9 = kv.contains(QStringLiteral("a9")) ? kv.value(QStringLiteral("a9")).toInt() : -1;
        s.estopPin = kv.contains(QStringLiteral("epin")) ? kv.value(QStringLiteral("epin")).toInt() : -2;
        s.inputs = kv.contains(QStringLiteral("din")) ? static_cast<int>(kv.value(QStringLiteral("din")).toUInt(nullptr, 16)) : -1;
        s.profile = kv.value(QStringLiteral("profile")).toUInt(nullptr, 16);
        s.stored = kv.value(QStringLiteral("stored")) == QStringLiteral("1");
        s.cutLength = kv.value(QStringLiteral("len")).toDouble();
        s.cutNext = kv.value(QStringLiteral("next")).toDouble();
        s.cutMin = kv.value(QStringLiteral("lmin")).toDouble();
        s.kerf = kv.value(QStringLiteral("kerf")).toDouble();
        s.simEncoder = kv.contains(QStringLiteral("simenc"));
        s.endOfCycle = kv.value(QStringLiteral("endcycle")) == QStringLiteral("1");
        s.ccioBoards = kv.value(QStringLiteral("ccio")).toInt();
        s.ccioBroken = kv.value(QStringLiteral("ccio_link")) == QStringLiteral("broken");
        s.ccioOff = kv.value(QStringLiteral("ccio_link")) == QStringLiteral("off");
        s.ccioInputs = kv.value(QStringLiteral("ccio_in")).toULongLong(nullptr, 16);
        s.simEncoderRate = kv.value(QStringLiteral("simenc")).toDouble();
        s.eip = kv.value(QStringLiteral("eip"), QStringLiteral("none"));
        s.eipFrom = kv.value(QStringLiteral("eip_from"));
        s.eipRpi = kv.value(QStringLiteral("eip_rpi")).toInt();
        s.motorsOn = kv.value(QStringLiteral("mon")).toUInt(nullptr, 16);
        s.jogMoving = kv.value(QStringLiteral("jogmove")).toUInt(nullptr, 16);
        s.motorFault = kv.value(QStringLiteral("mfault")).toUInt(nullptr, 16);
        s.motorHlfb = kv.value(QStringLiteral("mhlfb")).toUInt(nullptr, 16);
        s.limits = kv.value(QStringLiteral("lim")).toUInt(nullptr, 16);
        s.homed = kv.contains(QStringLiteral("homed")) ? static_cast<int>(kv.value(QStringLiteral("homed")).toUInt(nullptr, 16)) : -1;
        s.homing = static_cast<int>(kv.value(QStringLiteral("homing")).toUInt(nullptr, 16));
        s.homeSwitch = static_cast<int>(kv.value(QStringLiteral("homesw")).toUInt(nullptr, 16));
        s.hasHome = static_cast<int>(kv.value(QStringLiteral("hashome")).toUInt(nullptr, 16));
        s.waitIndex = kv.value(QStringLiteral("wait")).toInt();
        s.opKnown = kv.contains(QStringLiteral("oppins"));
        if (s.opKnown) {
            const QStringList pins = kv.value(QStringLiteral("oppins")).split(QLatin1Char(','));
            for (int k = 0; k < 5; ++k) s.opPins[k] = k < pins.size() ? pins[k].toInt() : -1;
            s.opPressed = static_cast<int>(kv.value(QStringLiteral("ops")).toUInt(nullptr, 16));
            s.opNc = static_cast<int>(kv.value(QStringLiteral("opnc")).toUInt(nullptr, 16));
            s.opMaintained = kv.value(QStringLiteral("opmode")).toInt() == 1;
        }
        s.waitedMs = kv.value(QStringLiteral("waited")).toLongLong();
        s.offCam = kv.value(QStringLiteral("offcam")).toUInt(nullptr, 16);
    }
    s.axisCounts.clear();
    for (int i = 0; kv.contains(QStringLiteral("a%1").arg(i)); ++i) {
        s.axisCounts << kv.value(QStringLiteral("a%1").arg(i)).toLongLong();
    }
    emit stateUpdated(s);
}

void BoardLink::uploadProfile(const ecam::CamProject &project, std::function<void(bool, const QString &)> done) {
    const std::vector<std::string> lines = io::UploadLines(project);
    const quint32 hash = ecam::ProfileHash(lines);
    // Stop at the first error; the board drops the upload when "load" is sent again.
    auto failed = std::make_shared<bool>(false);
    auto step = [failed, done](bool ok, const QStringList &reply) {
        if (!ok && !*failed) {
            *failed = true;
            done(false, reply.isEmpty() ? QStringLiteral("no reply") : reply.last());
        }
    };
    command(QStringLiteral("load"), step);
    for (const std::string &l : lines) {
        command(QString::fromStdString(l), [failed, step](bool ok, const QStringList &r) {
            if (!*failed) step(ok, r);
        });
    }
    command(QStringLiteral("commit"), [failed, done, hash, step](bool ok, const QStringList &r) {
        if (*failed) return;
        if (!ok) {
            step(ok, r);
            return;
        }
        const QString reply = r.isEmpty() ? QString() : r.last();
        const QRegularExpression re(QStringLiteral("hash=([0-9a-fA-F]+)"));
        const auto m = re.match(reply);
        const quint32 board = m.hasMatch() ? m.captured(1).toUInt(nullptr, 16) : 0;
        if (board != hash) {
            done(false, QStringLiteral("board hash %1 does not match %2").arg(board, 8, 16, QChar('0')).arg(hash, 8, 16, QChar('0')));
            return;
        }
        // stored=1: kept in flash for the next boot (older firmware does not say).
        const int se = reply.indexOf(QStringLiteral(" store_error="));
        QString where;
        if (reply.contains(QStringLiteral("stored=1"))) {
            where = QStringLiteral(", kept in the board's flash");
        } else if (se >= 0) {
            where = QStringLiteral(", NOT kept in flash (%1): lost at reboot").arg(reply.mid(se + 13));
        } else if (reply.contains(QStringLiteral("stored=0"))) {
            where = QStringLiteral(", not kept in flash: lost at reboot");
        }
        done(true, QStringLiteral("loaded, hash %1%2").arg(hash, 8, 16, QChar('0')).arg(where));
    });
}

void BoardLink::downloadProfile(std::function<void(bool, const ecam::CamProject &, bool, const QString &)> done) {
    command(QStringLiteral("download"), [done](bool ok, const QStringList &r) {
        ecam::CamProject p;
        if (!ok) {
            const QString why = r.isEmpty() ? QStringLiteral("no reply") : r.last();
            done(false, p, false,
                 why.contains(QStringLiteral("unknown command")) ? QStringLiteral("the board's firmware cannot download (update it)") : why);
            return;
        }
        QStringList lines = r;
        lines.removeLast();  // "ok"
        // The board's hash covers exactly these lines.
        quint32 h = ecam::ProfileHashBegin();
        for (const QString &l : lines) h = ecam::ProfileHashLine(h, l.toLatin1().constData());
        bool exact = false;
        QString error;
        if (!io::ProjectFromUpload(lines, &p, &exact, &error)) {
            done(false, p, false, error);
            return;
        }
        done(true, p, exact, QStringLiteral("%1").arg(h, 8, 16, QChar('0')));
    });
}
