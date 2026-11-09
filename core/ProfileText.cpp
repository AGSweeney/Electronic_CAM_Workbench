/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#include "ProfileText.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace ecam {

namespace {

std::string Num(double v) {
    char buf[40];
    snprintf(buf, sizeof buf, "%.17g", v);
    return buf;
}

// Units and names travel as single tokens / the rest of the line: keep them printable.
std::string Clean(const std::string &s, bool spacesOk) {
    std::string out;
    for (char c : s) {
        if (c == ' ' && !spacesOk) {
            out += '_';
        } else if (c >= 0x20 && c < 0x7f) {
            out += c;
        }
    }
    // The board trims trailing spaces before hashing a line; never send any.
    while (!out.empty() && out.back() == ' ') out.pop_back();
    return out.empty() ? std::string("-") : out;
}

const char *kTriggerKeys[] = {"cmd", "input", "eip"};

// "key=value" lookup in a tokenized line; returns nullptr when absent.
const char *Find(char *const *tok, int n, const char *key) {
    const size_t len = strlen(key);
    for (int i = 0; i < n; ++i) {
        if (strncmp(tok[i], key, len) == 0 && tok[i][len] == '=') return tok[i] + len + 1;
    }
    return nullptr;
}

bool ParseDouble(const char *s, double *out) {
    if (!s || !*s) return false;
    char *end = nullptr;
    const double v = strtod(s, &end);
    if (end == s || *end != '\0') return false;
    *out = v;
    return true;
}

bool ParseInt(const char *s, int *out) {
    if (!s || !*s) return false;
    char *end = nullptr;
    const long v = strtol(s, &end, 10);
    if (end == s || *end != '\0') return false;
    *out = static_cast<int>(v);
    return true;
}

}  // namespace

std::vector<std::string> ProfileLines(const CamProject &p) {
    std::vector<std::string> lines;
    const CamMaster &m = p.master;
    lines.push_back("master src=" + std::string(m.source == MasterSource::DrivenCrank ? "driven" : "encoder") +
                    " cpr=" + Num(CrankUnitsPerRev(m)) + " den=" + Num(m.crankGearDen) + " rpm=" + Num(m.rpm) +
                    " crank=" + Num(m.crankConnector) +
                    " ramp=" + Num(m.crankRampRpmPerSec) + " max=" + Num(m.crankMaxRpm) +
                    (m.lineMode ? " line=" + Num(m.lineCountsPerMm) + " kerf=" + Num(m.kerf) : std::string()) +
                    (m.source == MasterSource::Encoder && m.encoderReverse ? " rev=1" : "") +
                    (m.source == MasterSource::DrivenCrank && m.crankReverse ? " mrev=1" : ""));
    lines.push_back("timeline repeat=" + std::string(m.timelineRepeat ? "1" : "0") + " ovr=" + Num(m.speedOverride) +
                    " trig=" + kTriggerKeys[static_cast<int>(m.trigger)] + " in=" + Num(m.triggerInput) +
                    " rise=" + std::string(m.triggerRising ? "1" : "0") + " bit=" + Num(m.triggerBit));
    for (size_t i = 0; i < p.axes.size(); ++i) {
        const CamAxis &a = p.axes[i];
        const std::string idx = Num(static_cast<double>(i));
        lines.push_back("axis " + idx + " master=" + (a.master == AxisMaster::Timeline ? "timeline" : "crank") +
                        " m=" + Num(a.connector) + " cpu=" + Num(a.countsPerUnit) + " start=" + Num(a.startPos) +
                        " closed=" + (a.closed ? "1" : "0") + " span=" + Num(a.span) + " units=" +
                        Clean(a.units, false) +
                        (a.gear ? " gear=" + Num(a.gearNum) + ":" + Num(a.gearDen) : std::string()) +
                        (a.reverse ? " mrev=1" : "") +
                        " name=" + Clean(Clean(a.name, true).substr(0, 40), true));
        for (const CamSegment &s : a.segments) {
            lines.push_back("seg " + idx + " " + Num(s.endDeg) + " " + Num(s.endPos) + " " + LawKey(s.law));
        }
    }
    for (const OutputPulse &o : p.outputs) {
        lines.push_back("out " + Num(o.output) + " " + Num(o.on) + " " + Num(o.off) +
                        (o.master == AxisMaster::Crank ? " crank" : ""));
    }
    // Timeline waits, only with some (a profile without keeps its text and hash).
    for (const TimelineWait &w : p.waits) {
        std::string src = w.source == WaitSource::Board ? "board " + Num(w.pin)
                          : w.source == WaitSource::Ccio ? "ccio " + Num(w.board) + " " + Num(w.pin)
                                                         : "plc " + Num(w.pin);
        lines.push_back("wait " + Num(w.t) + " " + src + (w.on ? " on" : " off") + (w.edge ? " edge" : "") +
                        (w.timeout > 0.0 ? " timeout=" + Num(w.timeout) : std::string()));
    }
    // Only with boards, so a profile without any keeps its text and hash.
    if (p.ccio.boards > 0) lines.push_back("ccio_boards " + Num(p.ccio.boards));
    // The inputs with a job (a name alone stays with the project copy), and the limit stop: only when there are any,
    // so a profile without them keeps its text and hash.
    static const char *const kRoles[] = {"", "home", "limit+", "limit-"};
    for (const BoardInput &i : p.inputs) {
        if (i.role == CcioRole::None || i.axis < 0) continue;
        lines.push_back("input board " + Num(i.pin) + " " + kRoles[static_cast<int>(i.role)] + " " + Num(i.axis) +
                        (i.normallyOpen ? " no" : " nc"));
    }
    for (const CcioInput &i : p.ccio.inputs) {
        if (i.role == CcioRole::None || i.axis < 0) continue;
        lines.push_back("input ccio " + Num(i.board) + " " + Num(i.pin) + " " + kRoles[static_cast<int>(i.role)] + " " +
                        Num(i.axis) + (i.normallyOpen ? " no" : " nc"));
    }
    // Homing, for each axis with a home switch (the settings of one without stay with the project copy).
    for (size_t a = 0; a < p.axes.size(); ++a) {
        if (!HasHomeSwitch(p, static_cast<int>(a))) continue;
        const AxisHome &h = p.axes[a].home;
        lines.push_back("home " + Num(static_cast<double>(a)) + " dir=" + (h.dir > 0 ? "+" : "-") + " fast=" + Num(h.fast) +
                        " slow=" + Num(h.slow) + " back=" + Num(h.back) + " at=" + Num(h.at) +
                        " then=" + (h.thenStart ? "start" : "stay") + " order=" + Num(h.order) + " search=" + Num(h.search) +
                        " req=" + (h.required ? "1" : "0") + (h.decel > 0.0 ? " dec=" + Num(h.decel) : std::string()));
    }
    if (HasLimits(p)) lines.push_back("limits stop=" + Num(p.master.limitStopSeconds));
    return lines;
}

uint32_t ProfileHashBegin() { return 2166136261u; }

uint32_t ProfileHashLine(uint32_t h, const char *line) {
    for (const char *p = line; *p; ++p) {
        h ^= static_cast<uint8_t>(*p);
        h *= 16777619u;
    }
    h ^= static_cast<uint8_t>('\n');
    h *= 16777619u;
    return h;
}

uint32_t ProfileHash(const std::vector<std::string> &lines) {
    uint32_t h = ProfileHashBegin();
    for (const std::string &l : lines) h = ProfileHashLine(h, l.c_str());
    return h;
}

void ProfileReader::Begin() {
    m_project = CamProject();
    m_hash = ProfileHashBegin();
    m_text.clear();
    m_active = true;
    m_haveMaster = false;
    for (bool &h : m_homeLine) h = false;
}

const char *ProfileReader::Line(const char *text) {
    if (!m_active) return "no load in progress";
    const size_t len = strlen(text);
    if (len >= static_cast<size_t>(kProfileMaxLine)) return "line too long";
    if (m_text.size() + len + 1 > kProfileMaxText) return "profile too large (32 KB)";
    m_hash = ProfileHashLine(m_hash, text);
    m_text.append(text, len);
    m_text += '\n';
    // The project copy is stored and hashed, not used for motion.
    const size_t jl = strlen(kProfileProjectLine);
    if (strncmp(text, kProfileProjectLine, jl) == 0 && text[jl] == ' ') return nullptr;

    char buf[kProfileMaxLine];
    strncpy(buf, text, sizeof buf);
    buf[sizeof buf - 1] = '\0';
    // The axis name is the rest of the line (it may contain spaces).
    char *name = strstr(buf, " name=");
    if (name) {
        *name = '\0';
        name += 6;
    }
    char *tok[16];
    int n = 0;
    for (char *p = strtok(buf, " "); p && n < 16; p = strtok(nullptr, " ")) tok[n++] = p;
    if (n == 0) return "empty line";

    CamMaster &m = m_project.master;
    if (strcmp(tok[0], "master") == 0) {
        const char *src = Find(tok, n, "src");
        if (!src) return "master: src missing";
        if (strcmp(src, "driven") == 0) {
            m.source = MasterSource::DrivenCrank;
        } else if (strcmp(src, "encoder") == 0) {
            m.source = MasterSource::Encoder;
        } else {
            return "master: src must be encoder or driven";
        }
        double v;
        // cpr: master units per crank revolution (ppr x gear numerator); den: units per count (the gear
        // denominator, 1 when absent). The board needs only these; the ratio is kept exact.
        if (!ParseDouble(Find(tok, n, "cpr"), &v) || v < 1 || v > kMaxCrankUnits || v != static_cast<double>(static_cast<long>(v))) {
            return "master: bad cpr (whole units per crank rev)";
        }
        int den = 1;
        if (Find(tok, n, "den") && (!ParseInt(Find(tok, n, "den"), &den) || den < 1)) return "master: bad den";
        m.crankPpr = v;
        m.crankGearNum = 1;
        m.crankGearDen = den;
        m.countsPerRev = v / den;
        if (!ParseDouble(Find(tok, n, "rpm"), &v) || v <= 0) return "master: bad rpm";
        m.rpm = v;
        int c;
        if (!ParseInt(Find(tok, n, "crank"), &c) || c < 0 || c >= kMotorConnectors) return "master: bad crank";
        m.crankConnector = c;
        if (!ParseDouble(Find(tok, n, "ramp"), &v) || v <= 0) return "master: bad ramp";
        m.crankRampRpmPerSec = v;
        if (!ParseDouble(Find(tok, n, "max"), &v) || v <= 0) return "master: bad max";
        m.crankMaxRpm = v;
        // Encoder direction: rev=1 counts the other way.
        {
            int r = 0;
            if (Find(tok, n, "rev") && !ParseInt(Find(tok, n, "rev"), &r)) return "master: bad rev";
            m.encoderReverse = r != 0;
            r = 0;
            if (Find(tok, n, "mrev") && !ParseInt(Find(tok, n, "mrev"), &r)) return "master: bad mrev";
            m.crankReverse = r != 0;
        }
        // Cut-length mode: line=<encoder or crank motor counts per mm>; cpr is the design cut in counts.
        if (Find(tok, n, "line")) {
            if (!ParseDouble(Find(tok, n, "line"), &v) || v <= 0) return "master: bad line (counts per mm)";
            if (den != 1) return "master: cut-length mode has no gear ratio (den 1)";
            m.lineMode = true;
            m.lineCountsPerMm = v;
            m.wheelCircumference = 0.0;  // the scale is given directly
            m.designLength = m.crankPpr / v;
            m.kerf = 0.0;
            if (Find(tok, n, "kerf") && (!ParseDouble(Find(tok, n, "kerf"), &v) || v < 0)) return "master: bad kerf";
            if (Find(tok, n, "kerf")) m.kerf = v;

        }
        m_haveMaster = true;
        return nullptr;
    }
    if (strcmp(tok[0], "timeline") == 0) {
        int i;
        double v;
        if (!ParseInt(Find(tok, n, "repeat"), &i)) return "timeline: bad repeat";
        m.timelineRepeat = i != 0;
        if (!ParseDouble(Find(tok, n, "ovr"), &v) || v <= 0 || v > 10) return "timeline: bad ovr";
        m.speedOverride = v;
        const char *trig = Find(tok, n, "trig");
        bool found = false;
        for (int k = 0; k < 3 && trig; ++k) {
            if (strcmp(trig, kTriggerKeys[k]) == 0) {
                m.trigger = static_cast<TriggerSource>(k);
                found = true;
            }
        }
        if (!found) return "timeline: bad trig";
        if (!ParseInt(Find(tok, n, "in"), &i) || i < 0 || i >= kTriggerInputs) return "timeline: bad in";
        m.triggerInput = i;
        if (!ParseInt(Find(tok, n, "rise"), &i)) return "timeline: bad rise";
        m.triggerRising = i != 0;
        if (!ParseInt(Find(tok, n, "bit"), &i) || i < 0) return "timeline: bad bit";
        m.triggerBit = i;
        return nullptr;
    }
    if (strcmp(tok[0], "axis") == 0) {
        int idx;
        if (n < 2 || !ParseInt(tok[1], &idx) || idx != static_cast<int>(m_project.axes.size())) {
            return "axis: index out of order";
        }
        if (idx >= kMaxAxes) return "axis: too many axes";
        CamAxis a;
        a.segments.clear();
        const char *mt = Find(tok, n, "master");
        if (mt && strcmp(mt, "timeline") == 0) {
            a.master = AxisMaster::Timeline;
        } else if (mt && strcmp(mt, "crank") == 0) {
            a.master = AxisMaster::Crank;
        } else {
            return "axis: master must be crank or timeline";
        }
        int c;
        double v;
        if (!ParseInt(Find(tok, n, "m"), &c) || c < 0 || c >= kMotorConnectors) return "axis: bad m";
        a.connector = c;
        if (!ParseDouble(Find(tok, n, "cpu"), &v) || v == 0) return "axis: bad cpu";
        a.countsPerUnit = v;
        if (!ParseDouble(Find(tok, n, "start"), &v)) return "axis: bad start";
        a.startPos = v;
        if (!ParseInt(Find(tok, n, "closed"), &c)) return "axis: bad closed";
        a.closed = c != 0;
        c = 0;
        if (Find(tok, n, "mrev") && !ParseInt(Find(tok, n, "mrev"), &c)) return "axis: bad mrev";
        a.reverse = c != 0;
        if (!ParseDouble(Find(tok, n, "span"), &v) || v <= 0) return "axis: bad span";
        a.span = v;
        const char *units = Find(tok, n, "units");
        a.units = units ? units : "-";
        // Electronic gear: gear=<num>:<den>, whole numbers, num not 0, den above 0.
        if (const char *gear = Find(tok, n, "gear")) {
            char *end = nullptr;
            const long num = strtol(gear, &end, 10);
            if (end == gear || *end != ':') return "axis: bad gear (num:den)";
            const char *d = end + 1;
            const long den = strtol(d, &end, 10);
            if (end == d || *end != '\0' || num == 0 || den < 1) return "axis: bad gear (num:den, num not 0, den above 0)";
            if (a.master != AxisMaster::Crank) return "axis: a gear follows the crank";
            a.gear = true;
            a.gearNum = static_cast<int>(num);
            a.gearDen = static_cast<int>(den);
        }
        a.name = name ? name : "";
        m_project.axes.push_back(a);
        return nullptr;
    }
    if (strcmp(tok[0], "seg") == 0) {
        int idx;
        double end, pos;
        if (n != 5 || !ParseInt(tok[1], &idx) || !ParseDouble(tok[2], &end) || !ParseDouble(tok[3], &pos)) {
            return "seg: expected seg <axis> <end> <pos> <law>";
        }
        if (idx < 0 || idx >= static_cast<int>(m_project.axes.size())) return "seg: no such axis";
        CamAxis &a = m_project.axes[idx];
        if (static_cast<int>(a.segments.size()) >= kMaxSegments) return "seg: too many segments";
        CamSegment s;
        s.endDeg = end;
        s.endPos = pos;
        if (!LawFromKey(tok[4], &s.law)) return "seg: unknown law";
        const double prev = a.segments.empty() ? 0.0 : a.segments.back().endDeg;
        if (!(end > prev)) return "seg: ends must increase";
        a.segments.push_back(s);
        return nullptr;
    }
    if (strcmp(tok[0], "out") == 0) {
        int o;
        double on, off;
        const bool crank = n == 5 && strcmp(tok[4], "crank") == 0;
        if ((n != 4 && !crank) || !ParseInt(tok[1], &o) || !ParseDouble(tok[2], &on) || !ParseDouble(tok[3], &off)) {
            return "out: expected out <IO 0-5> <on> <off> [crank] (seconds, or crank degrees)";
        }
        if (o < 0 || o >= kOutputs) return "out: output must be 0..5 (IO-0..IO-5)";
        if (crank) {
            if (on < 0.0 || off > 360.0 || off < on + kMinPulseDeg * 0.999) return "out: crank pulse needs 0 <= on < off <= 360";
        } else if (on < 0.0 || off < on + kMinPulseSeconds * 0.999) {
            return "out: need 0 <= on and off >= on + 1 ms";
        }
        if (static_cast<int>(m_project.outputs.size()) >= kMaxOutputPulses) return "out: too many pulses (32)";
        OutputPulse p;
        p.output = o;
        p.master = crank ? AxisMaster::Crank : AxisMaster::Timeline;
        p.on = on;
        p.off = off;
        m_project.outputs.push_back(p);
        return nullptr;
    }
    if (strcmp(tok[0], "input") == 0) {
        // input board <pin> <role> <axis> <nc|no>   |   input ccio <board> <pin> <role> <axis> <nc|no>
        const bool board = n == 6 && strcmp(tok[1], "board") == 0;
        const bool ccio = n == 7 && strcmp(tok[1], "ccio") == 0;
        if (!board && !ccio) return "input: expected input board <pin> <role> <axis> <nc|no> or input ccio <board> <pin> <role> <axis> <nc|no>";
        int k = 2, cb = 0, pin, axis;
        if (ccio && (!ParseInt(tok[k++], &cb) || cb < 0 || cb >= kCcioMaxBoards)) return "input: bad CCIO-8 board";
        if (!ParseInt(tok[k++], &pin) || pin < 0 || pin >= (board ? kTriggerInputs : kCcioPinsPerBoard)) return "input: bad pin";
        const char *role = tok[k++];
        const CcioRole r = strcmp(role, "home") == 0     ? CcioRole::Home
                           : strcmp(role, "limit+") == 0 ? CcioRole::LimitPositive
                           : strcmp(role, "limit-") == 0 ? CcioRole::LimitNegative
                                                         : CcioRole::None;
        if (r == CcioRole::None) return "input: the job is home, limit+ or limit-";
        if (!ParseInt(tok[k++], &axis) || axis < 0) return "input: bad axis";
        const char *contact = tok[k++];
        if (strcmp(contact, "nc") != 0 && strcmp(contact, "no") != 0) return "input: the contact is nc or no";
        if (board) {
            BoardInput i;
            i.pin = pin;
            i.role = r;
            i.axis = axis;
            i.normallyOpen = strcmp(contact, "no") == 0;
            for (const BoardInput &o : m_project.inputs) {
                if (o.pin == pin) return "input: the same input twice";
            }
            m_project.inputs.push_back(i);
        } else {
            CcioInput i;
            i.board = cb;
            i.pin = pin;
            i.role = r;
            i.axis = axis;
            i.normallyOpen = strcmp(contact, "no") == 0;
            for (const CcioInput &o : m_project.ccio.inputs) {
                if (o.board == cb && o.pin == pin) return "input: the same input twice";
            }
            m_project.ccio.inputs.push_back(i);
        }
        return nullptr;
    }
    if (strcmp(tok[0], "wait") == 0) {
        // wait <t> board <pin> on|off [edge] [timeout=<s>] | wait <t> ccio <board> <pin> on|off [...] | wait <t> plc <bit> on|off [...]
        static const char *const kUsage = "wait: expected wait <t s> board <pin>|ccio <board> <pin>|plc <bit> on|off [edge] [timeout=<s>]";
        if (n < 5) return kUsage;
        TimelineWait w;
        w.edge = false;  // left out (0.10.0): the level alone
        if (!ParseDouble(tok[1], &w.t) || !(w.t > 0.0)) return "wait: the time must be above 0";
        int k = 3;
        if (strcmp(tok[2], "board") == 0) {
            w.source = WaitSource::Board;
            if (!ParseInt(tok[k++], &w.pin) || w.pin < 0 || w.pin >= kTriggerInputs) return "wait: bad board input";
        } else if (strcmp(tok[2], "ccio") == 0) {
            w.source = WaitSource::Ccio;
            if (n < 6 || !ParseInt(tok[k++], &w.board) || w.board < 0 || w.board >= kCcioMaxBoards) return "wait: bad CCIO-8 board";
            if (!ParseInt(tok[k++], &w.pin) || w.pin < 0 || w.pin >= kCcioPinsPerBoard) return "wait: bad CCIO-8 pin";
        } else if (strcmp(tok[2], "plc") == 0) {
            w.source = WaitSource::Plc;
            if (!ParseInt(tok[k++], &w.pin) || w.pin < 0 || w.pin >= kPlcWaitBits) return "wait: the PLC bit is 0-31";
        } else {
            return kUsage;
        }
        if (k >= n || (strcmp(tok[k], "on") != 0 && strcmp(tok[k], "off") != 0)) return "wait: on or off";
        w.on = strcmp(tok[k], "on") == 0;
        for (++k; k < n; ++k) {
            if (strcmp(tok[k], "edge") == 0) {
                w.edge = true;
            } else if (strncmp(tok[k], "timeout=", 8) == 0) {
                if (!ParseDouble(tok[k] + 8, &w.timeout) || w.timeout < 0) return "wait: timeout must be 0 or more";
            } else {
                return kUsage;
            }
        }
        if (m_project.waits.size() >= static_cast<size_t>(kMaxWaits)) return "wait: at most 16";
        m_project.waits.push_back(w);
        return nullptr;
    }
    if (strcmp(tok[0], "home") == 0) {
        // home <axis> dir=-|+ fast=<u/s> slow=<u/s> back=<u> at=<u> then=stay|start order=<1-9> search=<u> req=0|1 [dec=<u/s2>]
        static const char *const kUsage =
            "home: expected home <axis> dir=-|+ fast=<u/s> slow=<u/s> back=<u> at=<u> then=stay|start order=<1-9> search=<u> req=0|1 [dec=<u/s2>]";
        int axis, order, req;
        double fast, slow, back, at, search, decel = 0.0;
        if ((n != 11 && n != 12) || !ParseInt(tok[1], &axis)) return kUsage;
        if (n == 12 && (!ParseDouble(Find(tok, n, "dec"), &decel) || decel < 0)) return "home: dec must be 0 or more";
        if (axis < 0 || axis >= static_cast<int>(m_project.axes.size())) return "home: no such axis (home lines come after the axes)";
        if (m_homeLine[axis]) return "home: two lines for one axis";
        const char *dir = Find(tok, n, "dir");
        const char *then = Find(tok, n, "then");
        if (!dir || (strcmp(dir, "-") != 0 && strcmp(dir, "+") != 0)) return "home: dir is - or +";
        if (!then || (strcmp(then, "stay") != 0 && strcmp(then, "start") != 0)) return "home: then is stay or start";
        if (!ParseDouble(Find(tok, n, "fast"), &fast) || !(fast > 0)) return "home: fast must be above 0";
        if (!ParseDouble(Find(tok, n, "slow"), &slow) || !(slow > 0) || slow > fast) return "home: slow must be above 0 and at most fast";
        if (!ParseDouble(Find(tok, n, "back"), &back) || back < 0) return "home: back must be 0 or more";
        if (!ParseDouble(Find(tok, n, "at"), &at)) return kUsage;
        if (!ParseDouble(Find(tok, n, "search"), &search) || search < 0) return "home: search must be 0 or more";
        if (!ParseInt(Find(tok, n, "order"), &order) || order < 1 || order > kHomeOrderMax) return "home: order is 1-9";
        if (!ParseInt(Find(tok, n, "req"), &req) || (req != 0 && req != 1)) return "home: req is 0 or 1";
        AxisHome &h = m_project.axes[axis].home;
        h.dir = strcmp(dir, "+") == 0 ? 1 : -1;
        h.fast = fast;
        h.slow = slow;
        h.back = back;
        h.at = at;
        h.thenStart = strcmp(then, "start") == 0;
        h.order = order;
        h.search = search;
        h.required = req != 0;
        h.decel = decel;
        m_homeLine[axis] = true;
        return nullptr;
    }
    if (strcmp(tok[0], "limits") == 0) {
        double s;
        if (n != 2 || !ParseDouble(Find(tok, n, "stop"), &s) || s < kLimitStopMin || s > kLimitStopMax) {
            return "limits: expected limits stop=<seconds 0.01-2>";
        }
        m_project.master.limitStopSeconds = s;
        return nullptr;
    }
    if (strcmp(tok[0], "ccio_boards") == 0) {
        int boards;
        if (n != 2 || !ParseInt(tok[1], &boards) || boards < 1 || boards > kCcioMaxBoards) {
            return "ccio_boards: expected ccio_boards <boards 1-8>";
        }
        m_project.ccio.boards = boards;
        return nullptr;
    }
    return "unknown profile line";
}

const char *ProfileReader::Finish(CamProject *out) {
    if (!m_active) return "no load in progress";
    m_active = false;
    if (!m_haveMaster) return "master line missing";
    if (m_project.axes.empty()) return "no axes";
    bool used[kMotorConnectors] = {};
    for (const CamAxis &a : m_project.axes) {
        if (a.segments.empty()) return "an axis has no segments";
        const double last = a.segments.back().endDeg;
        if (a.master == AxisMaster::Crank && (last < 359.999 || last > 360.001)) return "a crank axis must end at 360";
        if (a.master == AxisMaster::Timeline && (last < a.span * 0.99999 || last > a.span * 1.00001)) {
            return "a timeline axis must end at its span";
        }
        if (used[a.connector]) return "two axes on one motor connector";
        used[a.connector] = true;
    }
    if (m_project.master.source == MasterSource::DrivenCrank && UsesCrank(m_project) &&
        used[m_project.master.crankConnector]) {
        return "the crank motor's connector is used by an axis";
    }
    if (m_project.master.lineMode && UsesCrank(m_project) && LineWaitDeg(m_project) <= 0.0) {
        return "cut-length mode: every crank axis must start with a dwell (the wait that stretches)";
    }
    const CamMaster &tm = m_project.master;
    if (UsesTimeline(m_project) && tm.trigger == TriggerSource::DigitalInput) {
        if (TriggerOnEncoderInput(tm, UsesCrank(m_project))) {
            return "the trigger input is on the Encoder Adapter (DI-6..DI-8) while an encoder is the master";
        }
        for (const OutputPulse &o : m_project.outputs) {
            if (o.output == tm.triggerInput) return "the trigger input is also a timeline output";
        }
    }
    // The inputs' jobs: each on an axis there is, one job per axis, on a pin nothing else has, on a CCIO-8 there is.
    {
        struct Job {
            CcioRole role;
            int axis;
        };
        std::vector<Job> jobs;
        const int axes = static_cast<int>(m_project.axes.size());
        for (const BoardInput &i : m_project.inputs) {
            if (i.axis >= axes) return "input: no such axis";
            if (!BoardInputBlocked(m_project, i.pin).empty()) return "input: the pin is an output, the trigger or the Encoder Adapter's";
            jobs.push_back({i.role, i.axis});
        }
        for (const CcioInput &i : m_project.ccio.inputs) {
            if (i.axis >= axes) return "input: no such axis";
            if (i.board >= m_project.ccio.boards) return "input: on a CCIO-8 the profile doesn't have (ccio_boards)";
            jobs.push_back({i.role, i.axis});
        }
        for (size_t a = 0; a < jobs.size(); ++a) {
            for (size_t b = a + 1; b < jobs.size(); ++b) {
                if (jobs[a].role == jobs[b].role && jobs[a].axis == jobs[b].axis) return "input: one job on two inputs";
            }
        }
    }
    // Waits: on the timeline, inside its cycle, on a pin nothing else has (one job per pin), on a CCIO-8 there is.
    if (!m_project.waits.empty()) {
        if (!UsesTimeline(m_project)) return "wait: the profile has no timeline";
        const double cycle = TimelineLength(m_project);
        for (const TimelineWait &w : m_project.waits) {
            if (!(w.t < cycle)) return "wait: past the end of the timeline's cycle";
            if (w.source == WaitSource::Board) {
                if (!BoardInputBlocked(m_project, w.pin).empty()) return "wait: the pin is an output, the trigger or the Encoder Adapter's";
                if (BoardInputAt(m_project, w.pin).role != CcioRole::None) return "wait: the pin is a home or limit input";
            } else if (w.source == WaitSource::Ccio) {
                if (w.board >= m_project.ccio.boards) return "wait: on a CCIO-8 the profile doesn't have (ccio_boards)";
                if (CcioInputAt(m_project.ccio, w.board, w.pin).role != CcioRole::None) return "wait: the pin is a home or limit input";
            }
        }
    }
    // Homing needs its switch. A home switch without a line (a 0.9.0 profile) homes with the defaults.
    for (int a = 0; a < static_cast<int>(m_project.axes.size()) && a < kMotorConnectors; ++a) {
        if (m_homeLine[a] && !HasHomeSwitch(m_project, a)) return "home: the axis has no home input";
    }
    NormalizeProject(m_project);
    *out = m_project;
    return nullptr;
}

}  // namespace ecam
