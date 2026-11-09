// MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
#include "suites.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

namespace bench {

namespace {

uint16_t U16(const Bytes &b, size_t o) { return static_cast<uint16_t>(b[o] | b[o + 1] << 8); }
uint32_t U32(const Bytes &b, size_t o) { return U16(b, o) | static_cast<uint32_t>(U16(b, o + 2)) << 16; }
int32_t I32(const Bytes &b, size_t o) { return static_cast<int32_t>(U32(b, o)); }

std::string IpAt(const Bytes &d, size_t at) {  // CIP carries an address as a UDINT, so its bytes arrive reversed
    char t[20];
    snprintf(t, sizeof(t), "%u.%u.%u.%u", d[at + 3], d[at + 2], d[at + 1], d[at]);
    return t;
}

Bytes Attr(Link &l, uint8_t attr) {
    const CipReply r = l.Explicit(0x0E, Path(0xF5, 1, attr));
    if (r.gen != 0) {
        throw std::runtime_error("get F5 attr " + std::to_string(attr) + ": status " + Hex(r.gen));
    }
    return r.data;
}

}  // namespace

const char *const kEngineStates[7] = {"empty", "disabled", "ready", "armed", "running", "jogging", "fault"};
const char *const kCrankStates[5] = {"off", "following", "running", "stopping", "jogging"};
const char *const kTlStates[6] = {"idle", "armed", "running", "stopping", "done", "jogging"};

const char *Name(const char *const *names, size_t count, unsigned v) { return v < count ? names[v] : "?"; }

Sts DecodeStatus(const Bytes &b) {
    Sts s;
    if (b.size() < kStsSize) {
        return s;
    }
    s.valid = true;
    s.fw = U16(b, 0);
    s.flags = U16(b, 2);
    s.fault = U16(b, 4);
    s.reject = U16(b, 6);
    s.state = U16(b, 8);
    s.crank = U16(b, 10);
    s.tl = U16(b, 12);
    s.outputs = U16(b, 14);
    s.hash = U32(b, 16);
    s.heartbeat = U32(b, 20);
    s.angle = I32(b, 24);
    s.rev = I32(b, 28);
    s.rpm = I32(b, 32);
    s.tUs = I32(b, 36);
    s.cycle = I32(b, 40);
    s.ovr = I32(b, 44);
    s.enc = I32(b, 48);
    s.axes = U16(b, 52);
    s.inputs = U16(b, 54);
    s.len = I32(b, 56);
    s.next = I32(b, 60);
    s.lmin = I32(b, 64);
    s.gap = I32(b, 68);
    s.sync = I32(b, 72);
    s.commloss = U32(b, 76);
    s.waitIndex = U16(b, 80);
    s.waitedMs = U32(b, 84);
    s.reservedZero = true;
    for (size_t i = 80; i < 96; i++) {
        s.reservedZero = s.reservedZero && b[i] == 0;
    }
    for (size_t i = 0; i < 4; i++) {
        const size_t o = 96 + 16 * i;
        s.axis[i].counts = I32(b, o);
        s.axis[i].travel = I32(b, o + 4);
        s.axis[i].flags = U16(b, o + 8);
        s.axis[i].conn = U16(b, o + 10);
        for (size_t k = 12; k < 16; k++) {
            s.reservedZero = s.reservedZero && b[o + k] == 0;
        }
    }
    return s;
}

Sts CurrentStatus(Link &l) { return DecodeStatus(l.Status()); }

Sts WaitStatus(Link &l, const std::function<bool(const Sts &)> &pred, double timeout) {
    const double end = PreciseSeconds() + timeout;
    while (PreciseSeconds() < end) {
        const Sts s = CurrentStatus(l);
        if (s.valid && pred(s)) {
            return s;
        }
        Sleep(10);
    }
    return CurrentStatus(l);
}

std::string FormatStatus(const Sts &s) {
    char t[256];
    snprintf(t, sizeof(t), "fw %u.%u  %s  flags 0x%04x  fault %u  reject %u  hash %08x  hb %u ms  crank %s rev %d rpm %.2f  tl %s  commloss %u",
             s.fw >> 8, s.fw & 0xFF, Name(kEngineStates, 7, s.state), s.flags, s.fault, s.reject, s.hash, s.heartbeat,
             Name(kCrankStates, 5, s.crank), s.rev, s.rpm / 100.0, Name(kTlStates, 6, s.tl), s.commloss);
    return t;
}

void Pulse(Link &l, uint16_t bit, int holdMs) {
    l.SetFlags(l.Flags() | bit);
    Sleep(static_cast<DWORD>(holdMs));
    l.SetFlags(l.Flags() & static_cast<uint16_t>(~bit));
    Sleep(static_cast<DWORD>(holdMs));
}

void TakeControl(Link &l, uint16_t extra) {
    l.SetFlags(C_CONTROL | extra);
    Sleep(150);
}

Net TcpIpRead(Link &l) {
    Net n;
    const Bytes a2 = Attr(l, 2), a3 = Attr(l, 3), a5 = Attr(l, 5), a6 = Attr(l, 6);
    n.capability = U32(a2, 0);
    n.settable = (n.capability & 0x10) != 0;
    n.control = U32(a3, 0) & 0x0F;
    n.ip = IpAt(a5, 0);
    n.netmask = IpAt(a5, 4);
    n.gateway = IpAt(a5, 8);
    n.dns1 = IpAt(a5, 12);
    n.dns2 = IpAt(a5, 16);
    const uint16_t dlen = U16(a5, 20);
    n.domain.assign(reinterpret_cast<const char *>(a5.data()) + 22, dlen);
    const uint16_t hlen = U16(a6, 0);
    n.host.assign(reinterpret_cast<const char *>(a6.data()) + 2, hlen);
    return n;
}

std::map<std::string, std::string> ConsoleStatus(Console &c) {
    std::map<std::string, std::string> m;
    std::istringstream in(Find(c.Cmd("status"), "state="));
    std::string kv;
    while (in >> kv) {
        const size_t eq = kv.find('=');
        if (eq != std::string::npos) {
            m[kv.substr(0, eq)] = kv.substr(eq + 1);
        }
    }
    return m;
}

std::string Upload(Console &c, const std::string &text) {
    c.Cmd("load");
    std::istringstream in(text);
    std::string l;
    while (std::getline(in, l)) {
        if (!l.empty() && l.back() == '\r') {
            l.pop_back();
        }
        if (l.find_first_not_of(" \t") == std::string::npos) {
            continue;
        }
        const auto r = c.Cmd(l);
        if (r.back().rfind("ok", 0) != 0) {
            throw std::runtime_error("upload: '" + l + "' -> " + r.back());
        }
    }
    const auto r = c.Cmd("commit", 10);
    if (r.back().rfind("ok", 0) != 0) {
        throw std::runtime_error("commit -> " + r.back());
    }
    return r.back();
}

std::string ReadFile(const std::string &path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("cannot read " + path);
    }
    std::stringstream t;
    t << in.rdbuf();
    return t.str();
}

namespace {
std::string g_plcIp;
}

void SetPlc(const std::string &ip) {
    g_plcIp = ip;
    UsePlc(ip);
}

void ClearPlc() {
    if (!g_plcIp.empty()) {
        PlcClearOutput(g_plcIp);
    }
}

bool Summary(const Results &r) {
    printf("\n%d/%d passed\n", r.passed, r.total);
    fflush(stdout);
    return r.passed == r.total && r.total > 0;
}

const char *const kBenchDriven =
    "master src=driven cpr=4000 den=1 rpm=20 crank=0 ramp=60 max=600\n"
    "timeline repeat=1 ovr=1 trig=cmd in=0 rise=1 bit=0\n"
    "axis 0 master=crank m=1 cpu=800 start=0 closed=1 span=360 units=mm name=Slide\n"
    "seg 0 120 10 cycloidal\n"
    "seg 0 180 10 dwell\n"
    "seg 0 300 0 cycloidal\n"
    "seg 0 360 0 dwell\n";

const char *const kBenchTimeline =
    "master src=encoder cpr=4000 den=1 rpm=20 crank=0 ramp=60 max=600\n"
    "timeline repeat=1 ovr=1 trig=cmd in=0 rise=1 bit=0\n"
    "axis 0 master=crank m=0 cpu=800 start=0 closed=1 span=360 units=deg name=Crank\n"
    "seg 0 180 90 cycloidal\n"
    "seg 0 360 0 cycloidal\n"
    "axis 1 master=timeline m=1 cpu=800 start=0 closed=1 span=2 units=mm name=Slide\n"
    "seg 1 1 20 cycloidal\n"
    "seg 1 2 0 cycloidal\n"
    "out 1 0.2 0.6\n";

/** Polls the console's status; the values of key seen, in order, without repeats. */
std::vector<std::string> WatchConsole(Console &c, double seconds, const char *key) {
    std::vector<std::string> seen;
    const double t0 = PreciseSeconds();
    while (PreciseSeconds() - t0 < seconds) {
        const std::string v = ConsoleStatus(c)[key];
        if (seen.empty() || seen.back() != v) {
            seen.push_back(v);
        }
    }
    return seen;
}

std::unique_ptr<Link> FreshLink(const std::string &ip, uint16_t flags) {
    auto lk = std::make_unique<Link>(ip, 20);
    lk->SetFlags(flags);
    lk->Open();
    Sleep(300);
    return lk;
}

// ---------------------------------------------------------------------------------------------------------------------
// Phase 1: identity, CIP objects, Multiple Service, Class 1 connection types
// ---------------------------------------------------------------------------------------------------------------------

namespace {

bool Contains(const Bytes &d, const char *text) {
    const std::string s(d.begin(), d.end());
    return s.find(text) != std::string::npos;
}

long long BoardTx(Console &c) { return IntAfter(Find(c.Cmd("eip"), "eip counters"), "tx="); }

void Phase1(const std::string &ip, Results &r) {
    printf("== discovery and identity\n");
    const Identity me = ListIdentity(ip);
    char t[256];
    snprintf(t, sizeof(t), "vendor=%u type=%u product=%u rev=%u.%u name='%s' serial=0x%08x status=0x%04x", me.vendor, me.type,
             me.product, me.revMajor, me.revMinor, me.name.c_str(), me.serial, me.status);
    r.Check("ListIdentity (UDP unicast) answers", true, t);
    r.Check("identity: vendor 65000, generic device, product 3, 'ECam ClearCore', rev " + std::to_string(kEipRevMajor) + "." +
                std::to_string(kEipRevMinor),
            me.vendor == 65000 && me.type == 43 && me.product == 3 && me.name == "ECam ClearCore" &&
                me.revMajor == kEipRevMajor && me.revMinor == kEipRevMinor);
    const auto found = Scan(Broadcast());
    bool seen = false;
    for (const Identity &f : found) {
        seen = seen || f.ip == ip;
    }
    r.Check("ListIdentity broadcast finds the board", seen, std::to_string(found.size()) + " answered");

    printf("== encapsulation and CIP objects\n");
    Link l(ip);
    l.Connect();
    r.Check("RegisterSession", l.Session() != 0, Hex(l.Session(), 8));
    const auto svc = l.ListServices();
    const uint16_t cap = !svc.empty() && svc[0].data.size() >= 4 ? U16(svc[0].data, 2) : 0;
    r.Check("ListServices: TCP encapsulation and Class 1 I/O", (cap & 0x0120) == 0x0120, "capability " + Hex(cap, 4));
    CipReply c = l.Explicit(0x01, Path(0x01, 1));
    r.Check("Identity Get_Attributes_All", c.gen == 0 && c.data.size() >= 2 && U16(c.data, 0) == 65000 && Contains(c.data, "ECam ClearCore"));
    c = l.Explicit(0x0E, Path(0x01, 1, 7));
    r.Check("Identity product name (attr 7)",
            c.gen == 0 && !c.data.empty() && std::string(c.data.begin() + 1, c.data.begin() + 1 + c.data[0]) == "ECam ClearCore");
    c = l.Explicit(0x0E, Path(0x04, 0, 3));
    r.Check("Assembly class: 3 instances", c.gen == 0 && U16(c.data, 0) == 3);
    const CipReply s150 = l.Explicit(0x0E, Path(0x04, kAsmCmd, 4)), s100 = l.Explicit(0x0E, Path(0x04, kAsmSts, 4));
    r.Check("Assembly sizes: 150 is 64 bytes, 100 is 160 bytes",
            s150.gen == 0 && s100.gen == 0 && U16(s150.data, 0) == 64 && U16(s100.data, 0) == 160);
    c = l.Explicit(0x0E, Path(0x04, kAsmSts, 3));
    r.Check("Status assembly readable by explicit Get (160 bytes)", c.gen == 0 && c.data.size() == 160);
    c = l.Explicit(0x0E, Path(0xF6, 1, 3));
    r.Check("Ethernet Link: MAC address", c.gen == 0 && c.data.size() == 6);
    Net n = TcpIpRead(l);
    r.Check("TCP/IP object: settable, static, this address", n.settable && n.control == 0 && n.ip == ip,
            "ip=" + n.ip + " mask=" + n.netmask + " gw=" + n.gateway + " host='" + n.host + "'");
    r.Check("default host name is 'ecam-' + 6 hex digits", n.host.rfind("ecam-", 0) == 0 && n.host.size() == 11, n.host);
    const std::string defaultHost = n.host;
    {
        Console con;
        con.Open(ip.c_str());
        con.Cmd("set host console-name");
        Sleep(300);  // the adapter picks the name up within 100 ms
        const std::string live = TcpIpRead(l).host;
        con.Cmd("set host -");
        Sleep(300);
        const std::string back = TcpIpRead(l).host;
        r.Check("a host name set on the console applies at once, '-' restores the default",
                live == "console-name" && back == defaultHost, "'" + live + "', then '" + back + "'");
    }

    printf("== Multiple Service Packet\n");
    std::vector<std::pair<uint8_t, Bytes>> ent;
    uint8_t gen = Multiple(l, {CipRequest(0x0E, Path(0x01, 1, 1)), CipRequest(0x0E, Path(0x01, 1, 7)),
                               CipRequest(0x0E, Path(0x04, kAsmSts, 4)), CipRequest(0x0E, Path(0xF5, 1, 6))}, &ent);
    bool ok = gen == 0 && ent.size() == 4;
    for (const auto &e : ent) {
        ok = ok && e.first == 0;
    }
    ok = ok && U16(ent[0].second, 0) == 65000 && Contains(ent[1].second, "ECam ClearCore") && U16(ent[2].second, 0) == 160;
    r.Check("four Gets in one packet, every answer right", ok);
    gen = Multiple(l, {CipRequest(0x0E, Path(0x01, 1, 1)), CipRequest(0x0E, Path(0x01, 1, 99)),
                       CipRequest(0x54, Path(0x06, 1), Bytes(4, 0))}, &ent);
    r.Check("a bad attribute and a Forward_Open inside: 0x1E, each entry its own status",
            gen == 0x1E && ent.size() == 3 && ent[0].first == 0 && ent[1].first == 0x14 && ent[2].first == 0x08,
            "general " + Hex(gen));
    c = l.Explicit(0x0A, Path(0x02, 1), Bytes{1, 0, 40, 0});
    r.Check("malformed packet (offset past the end): 0x13", c.gen == 0x13, Hex(c.gen));
    l.Close();

    printf("== Class 1 connections\n");
    Console con;
    con.Open(ip.c_str());
    struct Kind {
        const char *name;
        OpenOptions o;
    };
    OpenOptions large, multi, input;
    large.large = true;
    multi.multicast = true;
    input.inputOnly = input.multicast = true;
    const Kind kinds[] = {{"Forward_Open, point-to-point, 20 ms", {}},
                          {"Large_Forward_Open, point-to-point, 20 ms", large},
                          {"multicast T->O, 20 ms", multi},
                          {"input-only (null O->T), multicast T->O", input}};
    for (const Kind &k : kinds) {
        Link link(ip, 20);
        try {
            link.Open(k.o);
            Sleep(2000);
            const uint64_t n1 = link.RxCount();
            const auto times = link.RxTimes();
            const double mean = times.size() > 1 ? (times.back() - times.front()) / (times.size() - 1) * 1000 : 0;
            snprintf(t, sizeof(t), "%llu status packets in 2 s, mean %.1f ms", (unsigned long long)n1, mean);
            r.Check(k.name, n1 > 70 && mean > 15 && mean < 30, t);
        } catch (const std::exception &x) {
            r.Check(k.name, false, x.what());
        }
        link.Close();
        Sleep(300);
    }
    {
        Link link(ip, 20);
        link.Open();
        Link other(ip, 20, 0x4321);
        other.Connect();
        const OpenReply o = other.ForwardOpen();
        r.Check("a second originator is refused while one owns the connection (0x01 / 0x0106)",
                o.gen == 0x01 && !o.ext.empty() && o.ext[0] == 0x0106, Hex(o.gen) + " " + HexList(o.ext));
    }
    Sleep(300);
    {
        Link bad(ip);
        bad.Connect();
        OpenOptions wrong;
        wrong.o2tSize = 2 + 4 + 32;
        const OpenReply o = bad.ForwardOpen(wrong);
        r.Check("wrong O->T size refused (0x01 / 0x0127)", o.gen == 0x01 && !o.ext.empty() && o.ext[0] == 0x0127,
                Hex(o.gen) + " " + HexList(o.ext));
    }
    {
        Link bad(ip);
        bad.Connect();
        OpenOptions c3;
        c3.transport = 0x03;
        const OpenReply o = bad.ForwardOpen(c3);
        r.Check("Class 3 transport refused (not supported in this version)", o.gen == 0x01, Hex(o.gen) + " " + HexList(o.ext));
    }

    // At the PLC's 20 ms. The Python suite ran 10 ms here, faster than any machine this drives.
    printf("== the console and discovery alongside\n");
    Link link(ip, 20);
    link.Open();
    Sleep(300);
    const long long tx0 = BoardTx(con);
    const double t0 = PreciseSeconds();
    int replies = 0;
    while (PreciseSeconds() - t0 < 3) {  // a stress: the next command as soon as the reply is in
        replies += con.Cmd("status").back().rfind("ok", 0) == 0;
    }
    const double dt = PreciseSeconds() - t0;
    const long long sent = BoardTx(con) - tx0;
    const auto d = Discover();
    bool answered = false;
    for (const auto &e : d) {
        answered = answered || e.first == ip;
    }
    r.Check("console answers while Class 1 runs at 20 ms", replies > 1000, std::to_string(replies) + " status replies in 3 s");
    r.Check("UDP discovery (9502) still answers", answered, d.empty() ? "nothing" : d[0].second);
    link.Close();
    snprintf(t, sizeof(t), "%lld sent in %.2f s (%.1f/s of 50)", sent, dt, sent / dt);
    r.Check("Class 1 at 20 ms keeps its rate under a console flood (board side)", sent / dt > 48.5, t);
    const auto lines = con.Cmd("eip");
    for (const std::string &x : lines) {
        if (x.rfind("eip", 0) == 0) {
            printf("    %s\n", x.c_str());
        }
    }
    r.Check("'eip' console command reports the connections", Find(lines, "eip counters").find("opens=") != std::string::npos);
    printf("    %s\n", con.Cmd("diag")[0].c_str());
}

}  // namespace

int NetTool(const std::string &mode, const std::string &ip, const std::vector<std::string> &args) {
    auto ipBytes = [](Bytes &b, const std::string &text) {
        unsigned p[4] = {};
        sscanf(text.c_str(), "%u.%u.%u.%u", &p[0], &p[1], &p[2], &p[3]);
        b.insert(b.end(), {static_cast<uint8_t>(p[3]), static_cast<uint8_t>(p[2]), static_cast<uint8_t>(p[1]), static_cast<uint8_t>(p[0])});
    };
    try {
        if (mode == "identity") {
            const Identity id = ListIdentity(ip);
            printf("%s: vendor %u type %u product %u rev %u.%u serial 0x%08x status 0x%04x state %u '%s'\n", id.ip.c_str(), id.vendor, id.type,
                   id.product, id.revMajor, id.revMinor, id.serial, id.status, id.state, id.name.c_str());
        } else if (mode == "scan") {
            for (const Identity &id : Scan(ip.substr(0, ip.rfind('.')) + ".255")) {
                printf("%s  %s  rev %u.%u\n", id.ip.c_str(), id.name.c_str(), id.revMajor, id.revMinor);
            }
        } else if (mode == "net") {
            Link l(ip);
            l.Connect();
            const Net n = TcpIpRead(l);
            printf("%s: %s (in use) ip=%s mask=%s gw=%s host='%s' settable=%d\n", ip.c_str(), n.control == 2 ? "dhcp" : "static", n.ip.c_str(),
                   n.netmask.c_str(), n.gateway.c_str(), n.host.c_str(), n.settable ? 1 : 0);
        } else if (mode == "io" || mode == "status") {
            const double seconds = args.empty() ? 5 : atof(args[0].c_str());
            Link l(ip, 20);
            l.Open();
            const double t0 = PreciseSeconds();
            while (PreciseSeconds() - t0 < seconds) {
                if (mode == "status") {
                    printf("%s\n", FormatStatus(CurrentStatus(l)).c_str());
                }
                Sleep(mode == "status" ? 1000 : 50);
            }
            printf("%llu status packets\n", (unsigned long long)l.RxCount());
        } else {
            Link l(ip);
            l.Connect();
            uint8_t g = 0xFF;
            if (mode == "setip" && args.size() >= 3) {
                Bytes d;
                for (const std::string &s : {args[0], args[1], args[2], std::string("0.0.0.0"), std::string("0.0.0.0")}) {
                    ipBytes(d, s);
                }
                d.push_back(0);
                d.push_back(0);
                g = l.Explicit(0x10, Path(0xF5, 1, 5), d).gen;
            } else if (mode == "sethost" && !args.empty()) {
                Bytes d = {static_cast<uint8_t>(args[0].size()), 0};
                d.insert(d.end(), args[0].begin(), args[0].end());
                if (args[0].size() & 1) {
                    d.push_back(0);
                }
                g = l.Explicit(0x10, Path(0xF5, 1, 6), d).gen;
            } else if (mode == "setdhcp" && !args.empty()) {
                g = l.Explicit(0x10, Path(0xF5, 1, 3), Bytes{static_cast<uint8_t>(args[0] == "on" ? 2 : 0), 0, 0, 0}).gen;
            } else if (mode == "reset" && !args.empty()) {
                g = l.Explicit(0x05, Path(0x01, 1), Bytes{static_cast<uint8_t>(atoi(args[0].c_str()))}).gen;
            } else {
                printf("usage: setip IP NEW MASK GW | sethost IP NAME | setdhcp IP on|off | reset IP 0|1\n");
                return 2;
            }
            printf("general status %s\n", Hex(g).c_str());
        }
    } catch (const std::exception &x) {
        printf("STOPPED: %s\n", x.what());
        return 1;
    }
    return 0;
}

int SuiteTest(const std::string &ip) {
    Results r;
    try {
        Phase1(ip, r);
    } catch (const std::exception &x) {
        r.Check("the suite ran to the end", false, x.what());
    }
    return Summary(r) ? 0 : 1;
}

}  // namespace bench
