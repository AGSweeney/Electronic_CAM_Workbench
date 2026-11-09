/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "ProjectIo.h"

#include "Brand.h"

#include "CamGeometry.h"
#include "ProfileText.h"

#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QSaveFile>
#include <QTextStream>
#include <QVector>

#include <algorithm>
#include <cmath>
#include <tuple>

using namespace ecam;

namespace io {

namespace {

bool WriteText(const QString &path, const QString &text, QString *error) {
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (error) *error = f.errorString();
        return false;
    }
    f.write(text.toUtf8());
    if (!f.commit()) {
        if (error) *error = f.errorString();
        return false;
    }
    return true;
}

QString CIdentifier(const std::string &name) {
    QString id = QString::fromStdString(name);
    id.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9]+")), QStringLiteral("_"));
    id.remove(QRegularExpression(QStringLiteral("^_+|_+$")));
    if (id.isEmpty() || id.at(0).isDigit()) id.prepend(QStringLiteral("Axis_"));
    return id;
}

}  // namespace

bool SaveProject(const CamProject &project, const QString &path, QString *error) {
    return WriteText(path, QString::fromUtf8(ProjectToJson(project, true)), error);
}

bool LoadProject(const QString &path, CamProject *project, QString *error) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        if (error) *error = f.errorString();
        return false;
    }
    return ProjectFromJson(f.readAll(), project, error);
}

QByteArray ProjectToJson(const CamProject &project, bool indented) {
    QJsonObject master;
    master["countsPerRev"] = project.master.countsPerRev;
    master["crankPpr"] = project.master.crankPpr;
    master["crankGearNum"] = project.master.crankGearNum;
    master["crankGearDen"] = project.master.crankGearDen;
    master["rpm"] = project.master.rpm;
    master["source"] = project.master.source == MasterSource::DrivenCrank ? "drivenCrank" : "encoder";
    master["crankConnector"] = project.master.crankConnector;
    master["crankRampRpmPerSec"] = project.master.crankRampRpmPerSec;
    master["crankMaxRpm"] = project.master.crankMaxRpm;
    if (project.master.encoderReverse) master["encoderReverse"] = true;
    if (project.master.crankReverse) master["crankReverse"] = true;
    if (project.master.lineMode) {
        QJsonObject line;
        line["wheelCircumference"] = project.master.wheelCircumference;
        line["designLength"] = project.master.designLength;
        line["kerf"] = project.master.kerf;
        master["cutLength"] = line;
    }
    QJsonObject timeline;
    timeline["repeat"] = project.master.timelineRepeat;
    timeline["speedOverride"] = project.master.speedOverride;
    static const char *kTriggers[] = {"command", "digitalInput", "ethernetIp"};
    timeline["trigger"] = kTriggers[static_cast<int>(project.master.trigger)];
    timeline["triggerInput"] = project.master.triggerInput;
    timeline["triggerRising"] = project.master.triggerRising;
    timeline["triggerBit"] = project.master.triggerBit;
    master["timeline"] = timeline;

    QJsonArray axes;
    for (size_t ai = 0; ai < project.axes.size(); ++ai) {
        const CamAxis &a = project.axes[ai];
        QJsonObject o;
        o["name"] = QString::fromStdString(a.name);
        o["units"] = QString::fromStdString(a.units);
        o["master"] = a.master == AxisMaster::Timeline ? "timeline" : "crank";
        if (a.master == AxisMaster::Timeline) o["duration"] = a.span;
        o["connector"] = a.connector;
        o["countsPerUnit"] = a.countsPerUnit;
        static const char *kDrives[] = {"direct", "ballscrew", "belt"};
        o["drive"] = kDrives[static_cast<int>(a.drive)];
        o["motorPpr"] = a.motorPpr;
        o["screwPitch"] = a.screwPitch;
        o["pulleyPd"] = a.pulleyPd;
        o["velMax"] = a.velMax;
        o["accMax"] = a.accMax;
        o["startPos"] = a.startPos;
        o["closed"] = a.closed;
        if (a.reverse) o["reverse"] = true;
        o["baseRadius"] = a.baseRadius;
        o["rollerRadius"] = a.rollerRadius;
        if (a.gear) {
            o["gear"] = true;
            o["gearNum"] = a.gearNum;
            o["gearDen"] = a.gearDen;
        }
        // Homing, only for an axis with a home switch (or homing required): a project without keeps its text and hash.
        if (HasHomeSwitch(project, static_cast<int>(ai)) || a.home.required) {
            const AxisHome &h = a.home;
            QJsonObject ho;
            if (h.required) ho["required"] = true;
            ho["dir"] = h.dir > 0 ? QStringLiteral("+") : QStringLiteral("-");
            ho["fast"] = h.fast;
            ho["slow"] = h.slow;
            ho["back"] = h.back;
            ho["at"] = h.at;
            ho["then"] = h.thenStart ? QStringLiteral("start") : QStringLiteral("stay");
            ho["order"] = h.order;
            ho["search"] = h.search;
            if (h.decel > 0.0) ho["decel"] = h.decel;
            o["home"] = ho;
        }
        QJsonArray segs;
        for (const CamSegment &s : a.segments) {
            QJsonObject so;
            so["endDeg"] = s.endDeg;
            so["endPos"] = s.endPos;
            so["law"] = QString::fromLatin1(LawKey(s.law));
            segs.append(so);
        }
        o["segments"] = segs;
        axes.append(o);
    }

    QJsonObject root;
    root["format"] = "ECamUI";
    root["version"] = kFormatVersion;
    root["master"] = master;
    root["axes"] = axes;
    if (!project.outputs.empty()) {
        QJsonArray outs;
        for (const OutputPulse &o : project.outputs) {
            QJsonObject oo;
            oo["output"] = o.output;
            oo["master"] = o.master == AxisMaster::Crank ? "crank" : "timeline";
            oo["on"] = o.on;
            oo["off"] = o.off;
            outs.append(oo);
        }
        root["outputs"] = outs;
    }
    // Only when used: a project without it stays the same, hash included.
    if (project.ccio.boards > 0) {
        static const char *const kRoles[] = {"none", "home", "limit+", "limit-"};
        QJsonObject co;
        co["boards"] = project.ccio.boards;
        QJsonArray ins;
        for (const CcioInput &i : project.ccio.inputs) {
            QJsonObject io;
            io["board"] = i.board;
            io["pin"] = i.pin;
            if (!i.name.empty()) io["name"] = QString::fromStdString(i.name);
            io["role"] = kRoles[static_cast<int>(i.role)];
            if (i.axis >= 0) io["axis"] = i.axis;
            if (i.normallyOpen) io["contact"] = QStringLiteral("no");
            ins.append(io);
        }
        co["inputs"] = ins;
        root["ccio"] = co;
    }
    if (!project.inputs.empty()) {
        static const char *const kRoles[] = {"none", "home", "limit+", "limit-"};
        QJsonArray ins;
        for (const BoardInput &i : project.inputs) {
            QJsonObject io;
            io["pin"] = QString::fromStdString(BoardInputName(i.pin));
            if (!i.name.empty()) io["name"] = QString::fromStdString(i.name);
            io["role"] = kRoles[static_cast<int>(i.role)];
            if (i.axis >= 0) io["axis"] = i.axis;
            if (i.normallyOpen) io["contact"] = QStringLiteral("no");
            ins.append(io);
        }
        root["inputs"] = ins;
    }
    if (project.master.limitStopSeconds != 0.1) root["limitStop"] = project.master.limitStopSeconds;
    if (!project.waits.empty()) {
        static const char *const kSources[] = {"board", "ccio", "plc"};
        QJsonArray ws;
        for (const TimelineWait &w : project.waits) {
            QJsonObject wo;
            wo["t"] = w.t;
            if (!w.name.empty()) wo["name"] = QString::fromStdString(w.name);
            wo["source"] = kSources[static_cast<int>(w.source)];
            if (w.source == WaitSource::Ccio) wo["board"] = w.board;
            wo["pin"] = w.pin;
            wo["level"] = w.on ? QStringLiteral("on") : QStringLiteral("off");
            if (w.timeout > 0.0) wo["timeout"] = w.timeout;
            if (w.edge) wo["change"] = true;
            ws.append(wo);
        }
        root["waits"] = ws;
    }
    return QJsonDocument(root).toJson(indented ? QJsonDocument::Indented : QJsonDocument::Compact);
}

bool ProjectFromJson(const QByteArray &json, CamProject *project, QString *error) {
    QJsonParseError pe;
    const QJsonDocument doc = QJsonDocument::fromJson(json, &pe);
    if (doc.isNull() || !doc.isObject()) {
        if (error) *error = QStringLiteral("Not a JSON project: %1").arg(pe.errorString());
        return false;
    }
    const QJsonObject root = doc.object();
    // The format tag stays "ECamUI" so every project file keeps opening.
    if (root["format"].toString() != "ECamUI") {
        if (error) *error = QStringLiteral("Not an " ECAM_PRODUCT_NAME " project.");
        return false;
    }
    if (root["version"].toInt() > kFormatVersion) {
        if (error) *error = QStringLiteral("Made by a newer " ECAM_PRODUCT_NAME " (format version %1).").arg(root["version"].toInt());
        return false;
    }

    CamProject p;
    const QJsonObject master = root["master"].toObject();
    p.master.countsPerRev = master["countsPerRev"].toDouble(p.master.countsPerRev);
    // Files from before the crank mechanics: counts per rev becomes the ppr at 1:1.
    p.master.crankPpr = master["crankPpr"].toDouble(p.master.countsPerRev);
    p.master.crankGearNum = std::max(1, master["crankGearNum"].toInt(1));
    p.master.crankGearDen = std::max(1, master["crankGearDen"].toInt(1));
    p.master.rpm = master["rpm"].toDouble(p.master.rpm);
    const QString source = master["source"].toString("encoder");
    if (source == "drivenCrank") {
        p.master.source = MasterSource::DrivenCrank;
    } else if (source != "encoder") {
        if (error) *error = QStringLiteral("Unknown master source \"%1\".").arg(source);
        return false;
    }
    p.master.crankConnector = std::clamp(master["crankConnector"].toInt(p.master.crankConnector), 0, kMotorConnectors - 1);
    p.master.crankRampRpmPerSec = master["crankRampRpmPerSec"].toDouble(p.master.crankRampRpmPerSec);
    p.master.crankMaxRpm = master["crankMaxRpm"].toDouble(p.master.crankMaxRpm);
    p.master.encoderReverse = master["encoderReverse"].toBool(false);
    p.master.crankReverse = master["crankReverse"].toBool(false);
    if (master.contains("cutLength")) {
        const QJsonObject line = master["cutLength"].toObject();
        p.master.lineMode = true;
        p.master.wheelCircumference = line["wheelCircumference"].toDouble(p.master.wheelCircumference);
        p.master.designLength = line["designLength"].toDouble(p.master.designLength);
        p.master.kerf = line["kerf"].toDouble(0.0);
    }
    {
        const QJsonObject tl = master["timeline"].toObject();
        p.master.timelineRepeat = tl["repeat"].toBool(false);
        p.master.speedOverride = std::max(0.01, tl["speedOverride"].toDouble(1.0));
        const QString trig = tl["trigger"].toString("digitalInput");
        p.master.trigger = trig == "command"      ? TriggerSource::Command
                           : trig == "ethernetIp" ? TriggerSource::EtherNetIP
                                                  : TriggerSource::DigitalInput;
        p.master.triggerInput = std::clamp(tl["triggerInput"].toInt(0), 0, kTriggerInputs - 1);
        p.master.triggerRising = tl["triggerRising"].toBool(true);
        p.master.triggerBit = std::max(0, tl["triggerBit"].toInt(0));
    }

    for (const QJsonValue &av : root["axes"].toArray()) {
        const QJsonObject o = av.toObject();
        CamAxis a;
        a.name = o["name"].toString(QStringLiteral("Axis %1").arg(p.axes.size() + 1)).toStdString();
        a.units = o["units"].toString("mm").toStdString();
        a.master = o["master"].toString("crank") == "timeline" ? AxisMaster::Timeline : AxisMaster::Crank;
        a.span = a.master == AxisMaster::Timeline ? o["duration"].toDouble(1.0) : 360.0;
        a.connector = o["connector"].toInt(static_cast<int>(p.axes.size()) + 1);
        a.countsPerUnit = o["countsPerUnit"].toDouble(a.countsPerUnit);
        const QString drive = o["drive"].toString(QStringLiteral("direct"));
        a.drive = drive == QStringLiteral("ballscrew") ? AxisDrive::Ballscrew
                  : drive == QStringLiteral("belt")    ? AxisDrive::Belt
                                                       : AxisDrive::Direct;
        a.motorPpr = o["motorPpr"].toDouble(a.motorPpr);
        a.screwPitch = o["screwPitch"].toDouble(a.screwPitch);
        a.pulleyPd = o["pulleyPd"].toDouble(a.pulleyPd);
        a.velMax = o["velMax"].toDouble(0.0);
        a.accMax = o["accMax"].toDouble(0.0);
        a.startPos = o["startPos"].toDouble(0.0);
        a.closed = o["closed"].toBool(true);
        a.reverse = o["reverse"].toBool(false);
        a.baseRadius = o["baseRadius"].toDouble(a.baseRadius);
        a.rollerRadius = o["rollerRadius"].toDouble(a.rollerRadius);
        a.gear = o["gear"].toBool(false);
        a.gearNum = o["gearNum"].toInt(1);
        a.gearDen = o["gearDen"].toInt(1);
        if (o.contains("home")) {
            const QJsonObject ho = o["home"].toObject();
            AxisHome &h = a.home;
            h.required = ho["required"].toBool(false);
            h.dir = ho["dir"].toString() == QStringLiteral("+") ? 1 : -1;
            h.fast = ho["fast"].toDouble(0.0);
            h.slow = ho["slow"].toDouble(0.0);
            h.back = ho["back"].toDouble(0.0);
            h.at = ho["at"].toDouble(0.0);
            h.thenStart = ho["then"].toString() != QStringLiteral("stay");
            h.order = ho["order"].toInt(1);
            h.search = ho["search"].toDouble(0.0);
            h.decel = ho["decel"].toDouble(0.0);
        }
        for (const QJsonValue &sv : o["segments"].toArray()) {
            const QJsonObject so = sv.toObject();
            CamSegment s;
            s.endDeg = so["endDeg"].toDouble();
            s.endPos = so["endPos"].toDouble();
            const QByteArray key = so["law"].toString().toLatin1();
            if (!LawFromKey(key.constData(), &s.law)) {
                if (error) *error = QStringLiteral("Unknown motion law \"%1\".").arg(QString::fromLatin1(key));
                return false;
            }
            a.segments.push_back(s);
        }
        p.axes.push_back(a);
    }
    if (p.axes.empty()) p.axes.push_back(DefaultAxis("Axis 1"));
    for (const QJsonValue &ov : root["outputs"].toArray()) {
        const QJsonObject oo = ov.toObject();
        OutputPulse o;
        o.output = oo["output"].toInt(0);
        o.master = oo["master"].toString(QStringLiteral("timeline")) == QStringLiteral("crank") ? AxisMaster::Crank
                                                                                             : AxisMaster::Timeline;
        o.on = oo["on"].toDouble(0.0);
        o.off = oo["off"].toDouble(o.on + 0.1);
        p.outputs.push_back(o);
    }
    if (static_cast<int>(p.outputs.size()) > kMaxOutputPulses) p.outputs.resize(kMaxOutputPulses);
    const QJsonObject co = root["ccio"].toObject();
    p.ccio.boards = co["boards"].toInt(0);
    for (const QJsonValue &iv : co["inputs"].toArray()) {
        const QJsonObject io = iv.toObject();
        CcioInput i;
        i.board = io["board"].toInt(-1);
        i.pin = io["pin"].toInt(-1);
        i.name = io["name"].toString().toStdString();
        const QString role = io["role"].toString();
        i.role = role == QStringLiteral("home") ? CcioRole::Home
                 : role == QStringLiteral("limit+") ? CcioRole::LimitPositive
                 : role == QStringLiteral("limit-") ? CcioRole::LimitNegative
                                                    : CcioRole::None;
        i.axis = io["axis"].toInt(-1);
        i.normallyOpen = io["contact"].toString() == QStringLiteral("no");
        SetCcioInput(p.ccio, i);
    }
    for (const QJsonValue &iv : root["inputs"].toArray()) {
        const QJsonObject io = iv.toObject();
        BoardInput i;
        i.pin = -1;
        const QString pin = io["pin"].toString();
        for (int n = 0; n < kTriggerInputs; ++n) {
            if (pin == QString::fromStdString(BoardInputName(n))) i.pin = n;
        }
        i.name = io["name"].toString().toStdString();
        const QString role = io["role"].toString();
        i.role = role == QStringLiteral("home") ? CcioRole::Home
                 : role == QStringLiteral("limit+") ? CcioRole::LimitPositive
                 : role == QStringLiteral("limit-") ? CcioRole::LimitNegative
                                                    : CcioRole::None;
        i.axis = io["axis"].toInt(-1);
        i.normallyOpen = io["contact"].toString() == QStringLiteral("no");
        if (i.pin >= 0) SetBoardInput(p, i);
    }
    p.master.limitStopSeconds = root["limitStop"].toDouble(0.1);
    for (const QJsonValue &wv : root["waits"].toArray()) {
        const QJsonObject wo = wv.toObject();
        TimelineWait w;
        w.t = wo["t"].toDouble(0.0);
        w.name = wo["name"].toString().toStdString();
        const QString src = wo["source"].toString();
        w.source = src == QStringLiteral("ccio") ? WaitSource::Ccio : src == QStringLiteral("plc") ? WaitSource::Plc : WaitSource::Board;
        w.board = wo["board"].toInt(0);
        w.pin = wo["pin"].toInt(0);
        w.on = wo["level"].toString() != QStringLiteral("off");
        w.timeout = wo["timeout"].toDouble(0.0);
        w.edge = wo["change"].toBool(false);  // left out (0.10.0): the level alone
        p.waits.push_back(w);
    }
    NormalizeProject(p);
    *project = p;
    return true;
}

std::vector<std::string> UploadLines(const CamProject &project) {
    std::vector<std::string> lines = ProfileLines(project);
    // The project copy: compact JSON, zlib, base64, in chunks that fit a profile line.
    const QByteArray packed = qCompress(ProjectToJson(project, false), 9).toBase64();
    const int chunk = kProfileMaxLine - 16;
    const std::string prefix = std::string(kProfileProjectLine) + " ";
    for (int i = 0; i < packed.size(); i += chunk) {
        lines.push_back(prefix + packed.mid(i, chunk).toStdString());
    }
    return lines;
}

bool ProjectFromUpload(const QStringList &lines, CamProject *project, bool *exact, QString *error) {
    const QString prefix = QString::fromLatin1(kProfileProjectLine) + QLatin1Char(' ');
    QByteArray packed;
    for (const QString &l : lines) {
        if (l.startsWith(prefix)) packed += l.mid(prefix.size()).toLatin1();
    }
    if (!packed.isEmpty()) {
        const QByteArray json = qUncompress(QByteArray::fromBase64(packed));
        if (!json.isEmpty() && ProjectFromJson(json, project, error)) {
            *exact = true;
            return true;
        }
    }
    // No (usable) copy: rebuild from what the board runs.
    ProfileReader reader;
    reader.Begin();
    for (const QString &l : lines) {
        if (l.startsWith(prefix)) continue;
        const QByteArray text = l.toLatin1();
        if (const char *err = reader.Line(text.constData())) {
            if (error) *error = QStringLiteral("Board profile line \"%1\": %2").arg(l, QString::fromLatin1(err));
            return false;
        }
    }
    CamProject p;
    if (const char *err = reader.Finish(&p)) {
        if (error) *error = QStringLiteral("Board profile: %1").arg(QString::fromLatin1(err));
        return false;
    }
    *project = p;
    *exact = false;
    return true;
}

bool ExportCsv(const CamProject &project, int axis, int intervals, const QString &path, QString *error) {
    const CamAxis &a = project.axes.at(axis);
    const double w = MasterRate(project.master, a);
    const bool timeline = a.master == AxisMaster::Timeline;
    QString out;
    QTextStream ts(&out);
    const QString u = QString::fromStdString(a.units);
    ts << (timeline ? "t_s,ticks_5khz,pos_" : "master_deg,master_counts,pos_") << u << ",motor_counts,vel_" << u << "_per_s,acc_" << u << "_per_s2\n";
    for (int i = 0; i <= intervals; ++i) {
        const double deg = a.span * i / intervals;
        // 360 evaluates the end of the cycle, not the wrapped start (matters for open profiles).
        const CamPoint p = (i == intervals) ? EvaluateSegment(a, static_cast<int>(a.segments.size()) - 1, 1.0)
                                            : Evaluate(a, deg);
        ts << QString::number(deg, 'f', 4) << ','
           << QString::number(timeline ? std::llround(deg * 5000.0)
                                       : std::llround(project.master.countsPerRev * i / intervals)) << ','
           << QString::number(p.pos, 'f', 6) << ','
           << QString::number(std::llround(p.pos * a.countsPerUnit)) << ','
           << QString::number(p.vel * w, 'f', 6) << ','
           << QString::number(p.acc * w * w, 'f', 6) << '\n';
    }
    ts.flush();
    return WriteText(path, out, error);
}

bool ExportHeader(const CamProject &project, int axis, int intervals, const QString &path, QString *error) {
    const CamAxis &a = project.axes.at(axis);
    const QString id = CIdentifier(a.name);
    const QString up = id.toUpper();
    QString out;
    QTextStream ts(&out);
    const bool timeline = a.master == AxisMaster::Timeline;
    const QString xu = timeline ? QStringLiteral("s") : QStringLiteral("deg");
    ts << "// Generated by the " ECAM_PRODUCT_NAME ": motion table for axis \"" << QString::fromStdString(a.name) << "\".\n";
    if (timeline) {
        ts << "// Timeline axis. Entry i is the motor position (counts) at t = i * DURATION_US / " << intervals
           << " (as drawn; the speed override scales time).\n";
    } else {
        ts << "// Crank axis. Entry i is the motor position (counts) at crank angle i * 360 / " << intervals << " deg,\n"
           << "// i.e. at master count i * MASTER_COUNTS_PER_REV / " << intervals << ".\n";
    }
    ts << "// Segments (start " << xu << " -> end " << xu << ": law, start -> end " << QString::fromStdString(a.units) << "):\n";
    for (int s = 0; s < static_cast<int>(a.segments.size()); ++s) {
        ts << "//   " << QString::number(SegmentStartDeg(a, s), 'f', 2) << " -> "
           << QString::number(a.segments[s].endDeg, 'f', 2) << ": " << LawName(a.segments[s].law) << ", "
           << QString::number(SegmentStartPos(a, s), 'g', 8) << " -> " << QString::number(a.segments[s].endPos, 'g', 8)
           << '\n';
    }
    ts << "#pragma once\n#include <stdint.h>\n\n"
       << "#define ECAM_" << up << "_TIMELINE " << (timeline ? 1 : 0) << "   // 0 = crank axis, 1 = timeline axis\n";
    if (timeline) {
        ts << "#define ECAM_" << up << "_DURATION_US " << std::llround(a.span * 1e6) << "\n";
    } else {
        ts << "#define ECAM_" << up << "_MASTER_COUNTS_PER_REV " << std::llround(project.master.countsPerRev) << "\n";
    }
    ts
       << "#define ECAM_" << up << "_INTERVALS " << intervals << "\n"
       << "#define ECAM_" << up << "_CLOSED " << (a.closed ? 1 : 0) << "\n"
       << "#define ECAM_" << up << "_CONNECTOR " << a.connector << "   // M-n\n"
       << "#define ECAM_" << up << "_MASTER_DRIVEN " << (project.master.source == MasterSource::DrivenCrank ? 1 : 0)
       << "   // 0 = external encoder, 1 = driven crank\n\n"
       << "static const int32_t kECam_" << id << "[ECAM_" << up << "_INTERVALS + 1] = {";
    for (int i = 0; i <= intervals; ++i) {
        const CamPoint p = (i == intervals) ? EvaluateSegment(a, static_cast<int>(a.segments.size()) - 1, 1.0)
                                            : Evaluate(a, a.span * i / intervals);
        if (i % 10 == 0) ts << "\n   ";
        ts << ' ' << std::llround(p.pos * a.countsPerUnit) << (i < intervals ? "," : "");
    }
    ts << "\n};\n";
    ts.flush();
    return WriteText(path, out, error);
}

/* ---- DXF ------------------------------------------------------------------------------------- */

namespace {

// A DXF R2000 (AC1015) writer: entities are gathered first, then the file is put together with every table,
// block record and dictionary an R2000 reader expects, each object with its handle and owner.
class Dxf {
public:
    explicit Dxf(QStringList layers, QVector<int> colors) : m_layers(std::move(layers)), m_colors(std::move(colors)) {}

    void Polyline(const QString &layer, const std::vector<Pt> &pts, bool closed) {
        Entity(QStringLiteral("LWPOLYLINE"), layer);
        Pair(100, QStringLiteral("AcDbPolyline"));
        Pair(90, static_cast<int>(pts.size()));
        Pair(70, closed ? 1 : 0);
        for (const Pt &p : pts) {
            Num(10, p.x);
            Num(20, p.y);
            Grow(p.x, p.y);
        }
    }
    void Circle(const QString &layer, double x, double y, double r) {
        Entity(QStringLiteral("CIRCLE"), layer);
        Pair(100, QStringLiteral("AcDbCircle"));
        Point(10, x, y);
        Num(40, r);
        Grow(x - r, y - r);
        Grow(x + r, y + r);
    }
    void Line(const QString &layer, double x0, double y0, double x1, double y1) {
        Entity(QStringLiteral("LINE"), layer);
        Pair(100, QStringLiteral("AcDbLine"));
        Point(10, x0, y0);
        Point(11, x1, y1);
        Grow(x0, y0);
        Grow(x1, y1);
    }
    void Text(const QString &layer, double x, double y, double h, const QString &s) {
        Entity(QStringLiteral("TEXT"), layer);
        Pair(100, QStringLiteral("AcDbText"));
        Point(10, x, y);
        Num(40, h);
        Pair(1, s);
        Pair(100, QStringLiteral("AcDbText"));
        Grow(x, y);
        Grow(x + h * 0.6 * s.size(), y + h);
    }

    // The whole file; `insUnits`: 1 in, 4 mm, 0 unitless.
    QString Finish(int insUnits) {
        QString out;
        QTextStream ts(&out);
        auto pair = [&](int code, const QString &v) { ts << code << '\n' << v << '\n'; };
        auto num = [&](int code, double v) { pair(code, QString::number(v, 'f', 6)); };
        auto hex = [](int h) { return QString::number(h, 16).toUpper(); };
        // Fixed handles for the tables and the blocks; the entities' handles follow them.
        enum : int {
            hVport = 1, hLtype, hLayer, hStyle, hView, hUcs, hAppid, hDimstyle, hBlockRec,
            hVportActive, hByBlock, hByLayer, hContinuous, hLayer0, hStyleStd, hAppAcad, hDimStd,
            hMsRec, hPsRec, hMsBlock, hMsEnd, hPsBlock, hPsEnd, hRootDict, hGroupDict, hLayers  // layers from here
        };
        const int firstEntity = hLayers + static_cast<int>(m_layers.size());
        const int seed = firstEntity + m_entities;
        const double pad = std::max({m_x1 - m_x0, m_y1 - m_y0, 1.0}) * 0.05;

        pair(0, QStringLiteral("SECTION"));
        pair(2, QStringLiteral("HEADER"));
        pair(9, QStringLiteral("$ACADVER"));
        pair(1, QStringLiteral("AC1015"));
        pair(9, QStringLiteral("$HANDSEED"));
        pair(5, hex(seed));
        pair(9, QStringLiteral("$INSUNITS"));
        pair(70, QString::number(insUnits));
        pair(9, QStringLiteral("$MEASUREMENT"));
        pair(70, insUnits == 1 ? QStringLiteral("0") : QStringLiteral("1"));
        pair(9, QStringLiteral("$EXTMIN"));
        num(10, m_x0 - pad), num(20, m_y0 - pad), num(30, 0.0);
        pair(9, QStringLiteral("$EXTMAX"));
        num(10, m_x1 + pad), num(20, m_y1 + pad), num(30, 0.0);
        pair(0, QStringLiteral("ENDSEC"));
        pair(0, QStringLiteral("SECTION"));
        pair(2, QStringLiteral("CLASSES"));
        pair(0, QStringLiteral("ENDSEC"));

        pair(0, QStringLiteral("SECTION"));
        pair(2, QStringLiteral("TABLES"));
        auto table = [&](const char *name, int handle, int count) {
            pair(0, QStringLiteral("TABLE"));
            pair(2, QString::fromLatin1(name));
            pair(5, hex(handle));
            pair(330, QStringLiteral("0"));
            pair(100, QStringLiteral("AcDbSymbolTable"));
            pair(70, QString::number(count));
        };
        auto record = [&](const char *type, int handle, int owner, const char *subclass, const QString &name) {
            pair(0, QString::fromLatin1(type));
            pair(5, hex(handle));
            pair(330, hex(owner));
            pair(100, QStringLiteral("AcDbSymbolTableRecord"));
            pair(100, QString::fromLatin1(subclass));
            pair(2, name);
            pair(70, QStringLiteral("0"));
        };
        // VPORT: the active viewport, looking at the drawing.
        table("VPORT", hVport, 1);
        record("VPORT", hVportActive, hVport, "AcDbViewportTableRecord", QStringLiteral("*ACTIVE"));
        num(10, 0.0), num(20, 0.0), num(11, 1.0), num(21, 1.0);
        num(12, (m_x0 + m_x1) / 2), num(22, (m_y0 + m_y1) / 2);
        num(13, 0.0), num(23, 0.0), num(14, 1.0), num(24, 1.0), num(15, 1.0), num(25, 1.0);
        num(16, 0.0), num(26, 0.0), num(36, 1.0), num(17, 0.0), num(27, 0.0), num(37, 0.0);
        num(40, (m_y1 - m_y0) + 4 * pad), num(41, std::max(1.0, (m_x1 - m_x0) / std::max(1e-9, m_y1 - m_y0))), num(42, 50.0);
        num(43, 0.0), num(44, 0.0), num(50, 0.0), num(51, 0.0);
        pair(71, QStringLiteral("0")), pair(72, QStringLiteral("100")), pair(73, QStringLiteral("1")), pair(74, QStringLiteral("3"));
        pair(75, QStringLiteral("0")), pair(76, QStringLiteral("0")), pair(77, QStringLiteral("0")), pair(78, QStringLiteral("0"));
        pair(0, QStringLiteral("ENDTAB"));
        // LTYPE: ByBlock, ByLayer, Continuous.
        table("LTYPE", hLtype, 3);
        for (const auto &[h, name] : {std::pair<int, QString>{hByBlock, QStringLiteral("ByBlock")}, {hByLayer, QStringLiteral("ByLayer")},
                                      {hContinuous, QStringLiteral("Continuous")}}) {
            record("LTYPE", h, hLtype, "AcDbLinetypeTableRecord", name);
            pair(3, h == hContinuous ? QStringLiteral("Solid line") : QString());
            pair(72, QStringLiteral("65"));
            pair(73, QStringLiteral("0"));
            num(40, 0.0);
        }
        pair(0, QStringLiteral("ENDTAB"));
        // LAYER: 0 and ours.
        table("LAYER", hLayer, 1 + static_cast<int>(m_layers.size()));
        record("LAYER", hLayer0, hLayer, "AcDbLayerTableRecord", QStringLiteral("0"));
        pair(62, QStringLiteral("7"));
        pair(6, QStringLiteral("Continuous"));
        pair(370, QStringLiteral("-3"));
        pair(390, QStringLiteral("0"));
        for (int i = 0; i < m_layers.size(); ++i) {
            record("LAYER", hLayers + i, hLayer, "AcDbLayerTableRecord", m_layers[i]);
            pair(62, QString::number(m_colors.value(i, 7)));
            pair(6, QStringLiteral("Continuous"));
            pair(370, QStringLiteral("-3"));
            pair(390, QStringLiteral("0"));
        }
        pair(0, QStringLiteral("ENDTAB"));
        // STYLE: Standard.
        table("STYLE", hStyle, 1);
        record("STYLE", hStyleStd, hStyle, "AcDbTextStyleTableRecord", QStringLiteral("Standard"));
        num(40, 0.0), num(41, 1.0), num(50, 0.0);
        pair(71, QStringLiteral("0"));
        num(42, 2.5);
        pair(3, QStringLiteral("txt"));
        pair(4, QString());
        pair(0, QStringLiteral("ENDTAB"));
        // VIEW, UCS: empty.
        table("VIEW", hView, 0);
        pair(0, QStringLiteral("ENDTAB"));
        table("UCS", hUcs, 0);
        pair(0, QStringLiteral("ENDTAB"));
        // APPID: ACAD.
        table("APPID", hAppid, 1);
        record("APPID", hAppAcad, hAppid, "AcDbRegAppTableRecord", QStringLiteral("ACAD"));
        pair(0, QStringLiteral("ENDTAB"));
        // DIMSTYLE: Standard (its table has a subclass of its own; its records use code 105 for the handle).
        pair(0, QStringLiteral("TABLE"));
        pair(2, QStringLiteral("DIMSTYLE"));
        pair(5, hex(hDimstyle));
        pair(330, QStringLiteral("0"));
        pair(100, QStringLiteral("AcDbSymbolTable"));
        pair(70, QStringLiteral("1"));
        pair(100, QStringLiteral("AcDbDimStyleTable"));
        pair(0, QStringLiteral("DIMSTYLE"));
        pair(105, hex(hDimStd));
        pair(330, hex(hDimstyle));
        pair(100, QStringLiteral("AcDbSymbolTableRecord"));
        pair(100, QStringLiteral("AcDbDimStyleTableRecord"));
        pair(2, QStringLiteral("Standard"));
        pair(70, QStringLiteral("0"));
        pair(0, QStringLiteral("ENDTAB"));
        // BLOCK_RECORD: model space and paper space.
        table("BLOCK_RECORD", hBlockRec, 2);
        for (const auto &[h, name] : {std::pair<int, QString>{hMsRec, QStringLiteral("*Model_Space")}, {hPsRec, QStringLiteral("*Paper_Space")}}) {
            pair(0, QStringLiteral("BLOCK_RECORD"));
            pair(5, hex(h));
            pair(330, hex(hBlockRec));
            pair(100, QStringLiteral("AcDbSymbolTableRecord"));
            pair(100, QStringLiteral("AcDbBlockTableRecord"));
            pair(2, name);
        }
        pair(0, QStringLiteral("ENDTAB"));
        pair(0, QStringLiteral("ENDSEC"));

        // BLOCKS: the two layout blocks, empty.
        pair(0, QStringLiteral("SECTION"));
        pair(2, QStringLiteral("BLOCKS"));
        for (const auto &[b, e, rec, name] : {std::tuple<int, int, int, QString>{hMsBlock, hMsEnd, hMsRec, QStringLiteral("*Model_Space")},
                                              {hPsBlock, hPsEnd, hPsRec, QStringLiteral("*Paper_Space")}}) {
            pair(0, QStringLiteral("BLOCK"));
            pair(5, hex(b));
            pair(330, hex(rec));
            pair(100, QStringLiteral("AcDbEntity"));
            pair(8, QStringLiteral("0"));
            pair(100, QStringLiteral("AcDbBlockBegin"));
            pair(2, name);
            pair(70, QStringLiteral("0"));
            num(10, 0.0), num(20, 0.0), num(30, 0.0);
            pair(3, name);
            pair(1, QString());
            pair(0, QStringLiteral("ENDBLK"));
            pair(5, hex(e));
            pair(330, hex(rec));
            pair(100, QStringLiteral("AcDbEntity"));
            pair(8, QStringLiteral("0"));
            pair(100, QStringLiteral("AcDbBlockEnd"));
        }
        pair(0, QStringLiteral("ENDSEC"));

        // ENTITIES: ours, in model space (their handles and owner filled in here).
        pair(0, QStringLiteral("SECTION"));
        pair(2, QStringLiteral("ENTITIES"));
        int handle = firstEntity;
        for (const QString &line : m_body.split(QLatin1Char('\n'))) {
            if (line == QStringLiteral("@HANDLE")) {
                pair(5, hex(handle++));
                pair(330, hex(hMsRec));
            } else if (!line.isEmpty()) {
                ts << line << '\n';
            }
        }
        pair(0, QStringLiteral("ENDSEC"));

        // OBJECTS: the root dictionary and its (empty) group dictionary.
        pair(0, QStringLiteral("SECTION"));
        pair(2, QStringLiteral("OBJECTS"));
        pair(0, QStringLiteral("DICTIONARY"));
        pair(5, hex(hRootDict));
        pair(330, QStringLiteral("0"));
        pair(100, QStringLiteral("AcDbDictionary"));
        pair(281, QStringLiteral("1"));
        pair(3, QStringLiteral("ACAD_GROUP"));
        pair(350, hex(hGroupDict));
        pair(0, QStringLiteral("DICTIONARY"));
        pair(5, hex(hGroupDict));
        pair(330, hex(hRootDict));
        pair(100, QStringLiteral("AcDbDictionary"));
        pair(281, QStringLiteral("1"));
        pair(0, QStringLiteral("ENDSEC"));
        pair(0, QStringLiteral("EOF"));
        ts.flush();
        return out;
    }

private:
    void Entity(const QString &type, const QString &layer) {
        Pair(0, type);
        m_ts << "@HANDLE\n";  // the handle and the owner, filled in by Finish
        Pair(100, QStringLiteral("AcDbEntity"));
        Pair(8, layer);
        ++m_entities;
    }
    void Pair(int code, const QString &v) { m_ts << code << '\n' << v << '\n'; }
    void Pair(int code, int v) { Pair(code, QString::number(v)); }
    void Num(int code, double v) { Pair(code, QString::number(v, 'f', 6)); }
    void Point(int base, double x, double y) {
        Num(base, x);
        Num(base + 10, y);
        Num(base + 20, 0.0);
    }
    void Grow(double x, double y) {
        m_x0 = std::min(m_x0, x), m_x1 = std::max(m_x1, x);
        m_y0 = std::min(m_y0, y), m_y1 = std::max(m_y1, y);
    }
    QStringList m_layers;
    QVector<int> m_colors;
    QString m_body;
    QTextStream m_ts{&m_body};
    int m_entities = 0;
    double m_x0 = 1e300, m_y0 = 1e300, m_x1 = -1e300, m_y1 = -1e300;
};

// A length in the axis's units, in the drawing's: the factor, or 0 when the axis's units are not a length (deg,
// rev, counts): those are drawn as they are, unitless.
double LengthFactor(const std::string &axisUnits, DxfUnits to) {
    const QString u = QString::fromStdString(axisUnits).trimmed().toLower();
    double mm = 0.0;  // the axis's unit in mm
    if (u == QStringLiteral("mm")) mm = 1.0;
    else if (u == QStringLiteral("cm")) mm = 10.0;
    else if (u == QStringLiteral("m")) mm = 1000.0;
    else if (u == QStringLiteral("in") || u == QStringLiteral("inch") || u == QStringLiteral("\"")) mm = 25.4;
    if (mm == 0.0) return 0.0;
    return to == DxfUnits::Inch ? mm / 25.4 : mm;
}

QString UnitName(DxfUnits u) { return u == DxfUnits::Inch ? QStringLiteral("in") : QStringLiteral("mm"); }

// Drop the closing duplicate so a closed polyline does not have a zero-length edge.
std::vector<Pt> WithoutClosingPoint(std::vector<Pt> pts) {
    if (pts.size() > 2) {
        const Pt &a = pts.front(), &b = pts.back();
        if (std::hypot(a.x - b.x, a.y - b.y) < 1e-9) pts.pop_back();
    }
    return pts;
}

}  // namespace

bool ExportCamDxf(const CamProject &project, int axis, const CamDxfOptions &o, const QString &path, QString *error) {
    const CamAxis &a = project.axes.at(axis);
    if (a.master != AxisMaster::Crank) {
        if (error) *error = QStringLiteral("only a crank axis has a cam shape");
        return false;
    }
    // The drawing's units: the cam is a length drawing, in the axis's units converted (or as it is, unitless).
    const double f = LengthFactor(a.units, o.units);
    const double k = f > 0.0 ? f : 1.0;
    const QString u = f > 0.0 ? UnitName(o.units) : QString::fromStdString(a.units);
    auto scaled = [k](std::vector<Pt> pts) {
        for (Pt &p : pts) p.x *= k, p.y *= k;
        return pts;
    };
    const CamShape shape = BuildCamShape(a, std::max(36, o.pointsPerRev));
    const std::vector<Pt> surface = scaled(WithoutClosingPoint(shape.surface));
    const std::vector<Pt> pitch = scaled(WithoutClosingPoint(shape.pitch));
    double ext = shape.maxRadius * k;
    for (const Pt &p : pitch) ext = std::max(ext, std::hypot(p.x, p.y));
    ext = std::max(ext, o.boreDiameter * k / 2.0);

    Dxf d({QStringLiteral("CAM_PROFILE"), QStringLiteral("PITCH_CURVE"), QStringLiteral("BASE_CIRCLE"), QStringLiteral("BORE"),
           QStringLiteral("MARKS"), QStringLiteral("NOTES")},
          {7, 4, 8, 1, 2, 3});
    // An open profile (end above start) is a snail cam: closing the outline draws its radial step.
    d.Polyline(QStringLiteral("CAM_PROFILE"), surface, true);
    if (o.pitchCurve && a.rollerRadius > 0.0) d.Polyline(QStringLiteral("PITCH_CURVE"), pitch, true);
    if (o.baseCircle) d.Circle(QStringLiteral("BASE_CIRCLE"), 0, 0, a.baseRadius * k);
    if (o.boreDiameter > 0.0) d.Circle(QStringLiteral("BORE"), 0, 0, o.boreDiameter * k / 2.0);
    const double h = std::max(ext * 0.04, 1e-3);  // text height
    if (o.marks) {
        const double c = std::max(a.baseRadius * k * 0.15, h);
        d.Line(QStringLiteral("MARKS"), -c, 0, c, 0);
        d.Line(QStringLiteral("MARKS"), 0, -c, 0, c);
        // 0 deg: the contact direction at crank 0, from the base circle out past the profile.
        d.Line(QStringLiteral("MARKS"), 0, a.baseRadius * k * 0.6, 0, ext * 1.08);
        d.Text(QStringLiteral("MARKS"), h * 0.4, ext * 1.08, h, QStringLiteral("0 deg"));
    }
    if (o.notes) {
        double y = -ext * 1.12;
        const QStringList lines = {
            QStringLiteral("Cam: %1 (" ECAM_PRODUCT_NAME ")").arg(QString::fromStdString(a.name)),
            QStringLiteral("Follower at 0 deg mark when the crank is at 0 deg; cam turns clockwise"),
            QStringLiteral("Base radius %1 %3, roller radius %2 %3 (radial translating follower)")
                .arg(a.baseRadius * k, 0, 'g', 6).arg(a.rollerRadius * k, 0, 'g', 6).arg(u),
            QStringLiteral("Stroke %1 %2, %3 points per rev").arg(shape.stroke * k, 0, 'g', 6).arg(u).arg(surface.size()),
        };
        for (const QString &l : lines) {
            d.Text(QStringLiteral("NOTES"), -ext * 1.15, y, h * 0.8, l);
            y -= h * 1.4;
        }
    }
    return WriteText(path, d.Finish(f > 0.0 ? (o.units == DxfUnits::Inch ? 1 : 4) : 0), error);
}

bool ExportPathDxf(const CamProject &project, int axis, const PathDxfOptions &o, const QString &path, QString *error) {
    const CamAxis &a = project.axes.at(axis);
    if (a.segments.empty()) {
        if (error) *error = QStringLiteral("the axis has no profile");
        return false;
    }
    const bool timeline = a.master == AxisMaster::Timeline;
    const double span = timeline ? a.span : 360.0;  // the master's cycle: s or deg
    if (!(span > 0.0)) {
        if (error) *error = QStringLiteral("the axis's cycle has no length");
        return false;
    }
    const double f = LengthFactor(a.units, o.units);
    const double k = f > 0.0 ? f : 1.0;
    const QString yu = f > 0.0 ? UnitName(o.units) : QString::fromStdString(a.units);
    const QString xu = f > 0.0 ? UnitName(o.units) : QStringLiteral("unit");
    const int n = std::max(36, o.samples);
    std::vector<double> ms(n + 1), ps(n + 1);
    double lo = 1e300, hi = -1e300;
    for (int i = 0; i <= n; ++i) {
        ms[i] = span * i / n;
        ps[i] = Evaluate(a, ms[i]).pos * k;
        lo = std::min(lo, ps[i]), hi = std::max(hi, ps[i]);
    }
    // Across: the given scale, or (0) one picked so the drawing is about twice as wide as it is tall, rounded to 1, 2,
    // 2.5 or 5 times a power of ten per degree / second so the values still read off.
    double xs = timeline ? o.perSecond : o.perDegree;
    if (!(xs > 0.0)) {
        const double raw = 2.0 * std::max(hi - lo, 1e-9) / span;
        const double decade = std::pow(10.0, std::floor(std::log10(raw)));
        double best = decade;
        for (double m : {1.0, 2.0, 2.5, 5.0, 10.0}) {
            if (std::fabs(std::log(m * decade / raw)) < std::fabs(std::log(best / raw))) best = m * decade;
        }
        xs = best;
    }
    std::vector<Pt> pts;
    pts.reserve(n + 1);
    for (int i = 0; i <= n; ++i) pts.push_back({ms[i] * xs, ps[i]});
    const double w = span * xs, hgt = std::max(hi - lo, w * 0.05);
    const double th = std::max(std::min(w, hgt) * 0.035, 1e-3);  // text height

    Dxf d({QStringLiteral("PATH"), QStringLiteral("AXES"), QStringLiteral("NOTES")}, {7, 8, 3});
    d.Polyline(QStringLiteral("PATH"), pts, false);
    if (o.axes) {
        // The master axis at position 0 (or the lowest), the position axis at the cycle's start; a tick at each step.
        const double y0 = (lo <= 0.0 && hi >= 0.0) ? 0.0 : lo;
        d.Line(QStringLiteral("AXES"), 0, y0, w, y0);
        d.Line(QStringLiteral("AXES"), 0, lo, 0, hi);
        const double step = timeline ? std::pow(10.0, std::floor(std::log10(span / 4.0))) : 30.0;
        for (double m = 0.0; m <= span + 1e-9; m += step) {
            d.Line(QStringLiteral("AXES"), m * xs, y0, m * xs, y0 - th * 0.8);
            d.Text(QStringLiteral("AXES"), m * xs - th * 0.6, y0 - th * 2.2, th * 0.8,
                   timeline ? QStringLiteral("%1 s").arg(m, 0, 'g', 4) : QStringLiteral("%1 deg").arg(m, 0, 'f', 0));
        }
    }
    if (o.notes) {
        double y = lo - th * 4.5;
        const QStringList lines = {
            QStringLiteral("Path: %1 (" ECAM_PRODUCT_NAME ")").arg(QString::fromStdString(a.name)),
            timeline ? QStringLiteral("Across: time, 1 s = %1 %2 (cycle %3 s)").arg(xs, 0, 'g', 6).arg(xu).arg(span, 0, 'g', 6)
                     : QStringLiteral("Across: crank angle, 1 deg = %1 %2 (one revolution)").arg(xs, 0, 'g', 6).arg(xu),
            QStringLiteral("Up: position, in %1%2").arg(yu, f > 0.0 ? QString() : QStringLiteral(" (drawn as they are)")),
            // For reading it back (the import): what the drawing's coordinates mean.
            QStringLiteral("ECW-PATH master=%1 across=%2 per=%3 up=%4 units=%5")
                .arg(timeline ? QStringLiteral("timeline") : QStringLiteral("crank"))
                .arg(xs, 0, 'g', 10)
                .arg(timeline ? QStringLiteral("s") : QStringLiteral("deg"))
                .arg(yu, f > 0.0 ? UnitName(o.units) : QStringLiteral("none")),
        };
        for (const QString &l : lines) {
            d.Text(QStringLiteral("NOTES"), 0, y, th * 0.8, l);
            y -= th * 1.4;
        }
    }
    return WriteText(path, d.Finish(f > 0.0 ? (o.units == DxfUnits::Inch ? 1 : 4) : 0), error);
}

}  // namespace io
