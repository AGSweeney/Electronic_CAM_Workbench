/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Host side of ECam_Firmware (Firmware/PROTOCOL.md): the line console over TCP 9500 or the USB
// serial port, UDP 9502 discovery, telemetry, and profile upload.

#include "CamProfile.h"

#include <QHash>
#include <QObject>
#include <QQueue>
#include <QSerialPort>
#include <QStringList>
#include <QTcpSocket>
#include <QTimer>
#include <QUdpSocket>

#include <functional>

class BoardLink : public QObject {
    Q_OBJECT
public:
    enum class Transport { None, Ethernet, Serial };

    // From "status" (all fields) or a TEL line (motion fields only).
    struct State {
        bool valid = false;
        QString state;      // empty, disabled, ready, armed, running, fault
        bool enabled = false;
        bool simulate = false;
        QString fault;      // "none" when clear
        QString estop;      // ok / open / off (no estop configured, A-9)
        int a9 = -1;        // the A-9 input's level, 1 high, whatever estop_a9 says (-1: firmware before 0.8.9)
        int inputs = -1;    // the board's own inputs, bit n = IO-0..A-12 as the trigger index (-1: not reported)
        int estopPin = -2;  // the estop's input as the trigger index, -1 while the estop is off (-2: not reported)
        quint32 profile = 0;
        bool stored = false;  // the loaded profile is also the one kept in the board's flash
        QString crank;      // off, following, running, stopping
        QString timeline;   // idle, armed, running, stopping, done, jogging
        bool endOfCycle = false;  // running its last cycle: Stop asked, it rests at the cycle's end
        unsigned outputs = 0;        // outputs, bit n = IO-n (what the profile has on; pins stay off in sim)
        qint64 encoder = 0;          // encoder count (after the profile's direction; simulated in simenc)
        bool simEncoder = false;     // simulate: the board follows a simulated encoder (simenc=)
        double simEncoderRate = 0.0; // its counts/s
        // Cut-length mode (0 otherwise): finished piece now / from the next cut, shortest piece, kerf (mm).
        double cutLength = 0.0, cutNext = 0.0, cutMin = 0.0, kerf = 0.0;
        // Blade to the next cut line on the material (mm of line, 0 in the sync; -1 when not reported),
        // and the line left in the sync (mm, -1 when not in it).
        double cutGap = -1.0, syncLeft = -1.0;
        double deg = 0.0;
        long rev = 0;
        double rpm = 0.0;
        double t = 0.0;
        long cycle = 0;
        double override = 1.0;
        QVector<qint64> axisCounts;  // commanded motor counts per axis from the profile's start (0 at the load)
        // Axis jog (firmware 0.8.9+): bit c = M-c, the motor is on (enabled alone or with the rest), an axis jog moves
        // it; bit i = axis i, axis jog left it off its cam position.
        unsigned motorsOn = 0, jogMoving = 0, offCam = 0;
        unsigned motorFault = 0, motorHlfb = 0;  // bit c = M-c: in fault, HLFB asserted
        unsigned limits = 0;  // bit 2i: axis i on its + limit, bit 2i+1: on its - limit
        // Homing (0.9.1), bit i = axis i: homed, homing (or waiting in a Home all), its home switch tripped. -1: the
        // firmware doesn't home.
        int homed = -1, homing = 0, homeSwitch = 0, hasHome = 0;
        // Timeline waits (0.10.0): the wait holding, 1-based (0 none), and for how long.
        int waitIndex = 0;
        qint64 waitedMs = 0;
        // Operator inputs (0.11.0): pressed now (bit n: Enable, Disable, Fault reset, Start, Stop), each one's input
        // (-1 none; opKnown false before 0.11.0), the contacts (bit n NC) and the Enable mode (maintained).
        bool opKnown = false;
        int opPressed = 0, opNc = 0;
        int opPins[5] = {-1, -1, -1, -1, -1};
        bool opMaintained = false;
        // EtherNet/IP (firmware 0.4+): none (no connection), idle (PLC in Program), run, control (the PLC
        // has control: the board refuses enable, start, upload ...); the PLC's address and RPI (ms).
        QString eip = QStringLiteral("none");
        QString eipFrom;
        int eipRpi = 0;
        bool plcInControl() const { return eip == QStringLiteral("control"); }
        int ccioBoards = 0;
        bool ccioBroken = false;
        bool ccioOff = false;  // the profile on the board has no boards, so it doesn't talk to them
        quint64 ccioInputs = 0;  // bit 0: the first board's first input
    };

    static constexpr quint16 kConsolePort = 9500;
    static constexpr quint16 kDiscoveryPort = 9502;

    explicit BoardLink(QObject *parent = nullptr);
    ~BoardLink() override;

    bool connectEthernet(const QString &host, QString *error);
    bool connectSerial(const QString &port, QString *error);
    void disconnectLink();
    bool isConnected() const;
    // The firmware's version, empty until the board has said.
    QString firmwareVersion() const { return m_firmware; }
    Transport transport() const { return m_transport; }

    void discover();

    using Done = std::function<void(bool ok, const QStringList &lines)>;
    // Queued; done runs with the reply lines (the last one is "ok..." or "error: ...").
    void command(const QString &line, Done done = nullptr);
    bool busy() const { return m_active || !m_queue.isEmpty(); }

    // load, every profile line, commit; reports the board's hash against ours.
    void uploadProfile(const ecam::CamProject &project, std::function<void(bool ok, const QString &message)> done);
    // "download": the board's loaded profile. exact = it carried the project copy (see io::ProjectFromUpload).
    void downloadProfile(std::function<void(bool ok, const ecam::CamProject &project, bool exact, const QString &message)> done);

    const State &state() const { return m_state; }

signals:
    void connectedChanged(bool connected);
    void deviceDiscovered(const QString &host, const QString &info);
    void stateUpdated(const BoardLink::State &state);
    void logLine(const QString &line);
    void linkLost(const QString &reason);

private:
    struct Pending {
        QString line;
        Done done;
    };
    void write(const QByteArray &bytes);
    void onReadyRead(QIODevice *dev);
    void handleLine(const QString &line);
    void pump();
    void finish(bool ok);
    void parseFields(const QString &line, bool full);
    void onConnected();

    QString m_firmware;

    Transport m_transport = Transport::None;
    QTcpSocket m_tcp;
    QSerialPort m_serial;
    QUdpSocket m_udp;
    QTimer m_pollTimer;
    QTimer m_timeoutTimer;
    QByteArray m_rx;

    QQueue<Pending> m_queue;
    bool m_active = false;
    Pending m_current;
    QStringList m_lines;

    State m_state;
};

Q_DECLARE_METATYPE(BoardLink::State)
