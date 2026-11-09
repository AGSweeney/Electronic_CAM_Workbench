// MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
#include "bench_link.h"

#include <cstdio>
#include <memory>
#include <cstring>
#include <random>

#include "bench_common.h"
#include "eip.h"

namespace bench {

namespace {

std::string g_localIp = "10.168.41.1", g_broadcast = "10.168.41.255", g_plc;
const char *const kPlcOut = "ECam_O";  // the only PLC variable this writes
const char *const kPlcIn = "ECam_I";

bool PlcOpen(EipSession *s, const std::string &ip) {
    EipOpenConfig open;
    memset(&open, 0, sizeof(open));
    open.ip = ip.c_str();
    open.slot = -1;
    open.timeout_ms = 3000;
    return eip_session_open(s, &open) == EIP_OK;
}

std::mt19937 &Rng() {
    static std::mt19937 r{std::random_device{}()};
    return r;
}

void Put16(Bytes &b, uint16_t v) {
    b.push_back(static_cast<uint8_t>(v));
    b.push_back(static_cast<uint8_t>(v >> 8));
}

void Put32(Bytes &b, uint32_t v) {
    Put16(b, static_cast<uint16_t>(v));
    Put16(b, static_cast<uint16_t>(v >> 16));
}

uint16_t Get16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | p[1] << 8); }

uint32_t Get32(const uint8_t *p) { return Get16(p) | static_cast<uint32_t>(Get16(p + 2)) << 16; }

Bytes Cpf(const std::vector<Item> &items) {
    Bytes out;
    Put16(out, static_cast<uint16_t>(items.size()));
    for (const Item &i : items) {
        Put16(out, i.type);
        Put16(out, static_cast<uint16_t>(i.data.size()));
        out.insert(out.end(), i.data.begin(), i.data.end());
    }
    return out;
}

std::vector<Item> ParseCpf(const uint8_t *b, size_t len) {
    std::vector<Item> items;
    if (len < 2) {
        return items;
    }
    const uint16_t n = Get16(b);
    size_t off = 2;
    for (uint16_t i = 0; i < n && off + 4 <= len; i++) {
        const uint16_t t = Get16(b + off), l = Get16(b + off + 2);
        if (off + 4 + l > len) {
            break;
        }
        items.push_back({t, Bytes(b + off + 4, b + off + 4 + l)});
        off += 4u + l;
    }
    return items;
}

Bytes EncapHeader(uint16_t command, uint16_t length, uint32_t session) {
    Bytes h;
    Put16(h, command);
    Put16(h, length);
    Put32(h, session);
    Put32(h, 0);
    const char *ctx = "ecamexer";
    h.insert(h.end(), ctx, ctx + 8);
    Put32(h, 0);
    return h;
}

CipReply ParseReply(const Bytes &r) {
    CipReply c;
    if (r.size() < 4) {
        throw std::runtime_error("short CIP reply");
    }
    c.gen = r[2];
    const size_t words = r[3];
    for (size_t i = 0; i < words && 4 + 2 * i + 1 < r.size(); i++) {
        c.ext.push_back(Get16(r.data() + 4 + 2 * i));
    }
    if (4 + 2 * words < r.size()) {
        c.data.assign(r.begin() + static_cast<long>(4 + 2 * words), r.end());
    }
    return c;
}

sockaddr_in Addr(const std::string &ip, uint16_t port) {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &a.sin_addr);
    return a;
}

void Timeout(SOCKET s, double seconds) {
    const DWORD ms = static_cast<DWORD>(seconds * 1000);
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&ms), sizeof(ms));
}

Identity ParseIdentity(const uint8_t *packet, size_t len) {
    if (len < 24) {
        throw std::runtime_error("short ListIdentity reply");
    }
    const auto items = ParseCpf(packet + 24, len - 24);
    if (items.empty() || items[0].data.size() < 33) {
        throw std::runtime_error("bad ListIdentity reply");
    }
    const Bytes &d = items[0].data;
    Identity id;
    char ip[20];
    snprintf(ip, sizeof(ip), "%u.%u.%u.%u", d[6], d[7], d[8], d[9]);
    id.ip = ip;
    id.vendor = Get16(&d[18]);
    id.type = Get16(&d[20]);
    id.product = Get16(&d[22]);
    id.revMajor = d[24];
    id.revMinor = d[25];
    id.status = Get16(&d[26]);
    id.serial = Get32(&d[28]);
    const size_t n = d[32];
    id.name.assign(reinterpret_cast<const char *>(&d[33]), n <= d.size() - 33 ? n : d.size() - 33);
    if (d.size() > 33 + n) {
        id.state = d[33 + n];
    }
    return id;
}

struct Winsock {
    Winsock() {
        WSADATA w;
        WSAStartup(MAKEWORD(2, 2), &w);
    }
} g_winsock;

}  // namespace

Bytes Path(uint8_t cls, uint8_t inst, int attr) {
    Bytes p = {0x20, cls, 0x24, inst};
    if (attr >= 0) {
        p.push_back(0x30);
        p.push_back(static_cast<uint8_t>(attr));
    }
    return p;
}

Bytes CipRequest(uint8_t service, const Bytes &path, const Bytes &data) {
    Bytes r = {service, static_cast<uint8_t>(path.size() / 2)};
    r.insert(r.end(), path.begin(), path.end());
    r.insert(r.end(), data.begin(), data.end());
    return r;
}

std::string Hex(unsigned v, int width) {
    char t[16];
    snprintf(t, sizeof(t), "0x%0*x", width, v);
    return t;
}

std::string HexList(const std::vector<uint16_t> &v) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); i++) {
        s += (i ? " " : "") + Hex(v[i], 4);
    }
    return s + "]";
}

void UseNetworkOf(const std::string &ip) {
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in a = Addr(ip, kEnipPort);
    if (connect(s, reinterpret_cast<sockaddr *>(&a), sizeof(a)) == 0) {  // UDP: picks the route, sends nothing
        sockaddr_in me{};
        int n = sizeof(me);
        if (getsockname(s, reinterpret_cast<sockaddr *>(&me), &n) == 0) {
            char t[20];
            inet_ntop(AF_INET, &me.sin_addr, t, sizeof(t));
            g_localIp = t;
            g_broadcast = g_localIp.substr(0, g_localIp.rfind('.')) + ".255";
        }
    }
    closesocket(s);
}

const std::string &LocalIp() { return g_localIp; }
const std::string &Broadcast() { return g_broadcast; }

void UsePlc(const std::string &plcIp) { g_plc = plcIp; }

bool UsingPlc() { return !g_plc.empty(); }

bool PlcClearOutput(const std::string &plcIp) {
    auto s = std::make_unique<EipSession>();
    const uint8_t zero[kCmdSize] = {};
    for (int i = 0; i < 5; i++) {
        memset(s.get(), 0, sizeof(EipSession));
        if (PlcOpen(s.get(), plcIp)) {
            uint8_t back[kCmdSize];
            uint16_t len = sizeof(back);
            const int w = eip_tag_write(s.get(), kPlcOut, EIP_SINT, kCmdSize, zero, kCmdSize, nullptr);
            const int r = eip_tag_read(s.get(), kPlcOut, kCmdSize, back, &len, nullptr, nullptr);
            eip_session_close(s.get());
            if (w == EIP_OK && r == EIP_OK && len == kCmdSize && memcmp(back, zero, kCmdSize) == 0) {
                return true;
            }
        }
        Sleep(200);
    }
    printf("WARNING: could not confirm ECam_O is clear\n");
    return false;
}

Link::Link(std::string ip, int rpiMs, uint16_t vendor, uint32_t origSerial)
    : m_ip(std::move(ip)), m_rpi(rpiMs / 1000.0), m_vendor(vendor),
      m_serial(static_cast<uint16_t>(1 + Rng()() % 0xFFFF)), m_origSerial(origSerial ? origSerial : 1 + Rng()() % 0xFFFFFFFEu) {}

Link::~Link() { Close(); }

void Link::RecvExact(uint8_t *to, size_t n) {
    size_t got = 0;
    while (got < n) {
        const int r = recv(m_tcp, reinterpret_cast<char *>(to) + got, static_cast<int>(n - got), 0);
        if (r <= 0) {
            throw std::runtime_error("connection closed by adapter");
        }
        got += static_cast<size_t>(r);
    }
}

void Link::Encap(uint16_t command, const Bytes &data, uint32_t *session, Bytes *body) {
    Bytes out = EncapHeader(command, static_cast<uint16_t>(data.size()), m_session);
    out.insert(out.end(), data.begin(), data.end());
    if (send(m_tcp, reinterpret_cast<const char *>(out.data()), static_cast<int>(out.size()), 0) <= 0) {
        throw std::runtime_error("send failed");
    }
    uint8_t h[24];
    RecvExact(h, sizeof(h));
    const uint16_t length = Get16(h + 2);
    const uint32_t status = Get32(h + 8);
    Bytes b(length);
    if (length) {
        RecvExact(b.data(), length);
    }
    if (status != 0) {
        throw std::runtime_error("encapsulation status " + Hex(status));
    }
    if (session) {
        *session = Get32(h + 4);
    }
    if (body) {
        *body = std::move(b);
    }
}

Link &Link::Connect() {
    m_tcp = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    sockaddr_in a = Addr(m_ip, kEnipPort);
    Timeout(m_tcp, 3);
    const BOOL one = TRUE;
    setsockopt(m_tcp, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char *>(&one), sizeof(one));
    if (connect(m_tcp, reinterpret_cast<sockaddr *>(&a), sizeof(a)) != 0) {
        closesocket(m_tcp);
        m_tcp = INVALID_SOCKET;
        throw std::runtime_error("no connection to " + m_ip);
    }
    Bytes reg;
    Put16(reg, 1);
    Put16(reg, 0);
    Encap(0x65, reg, &m_session, nullptr);
    return *this;
}

Bytes Link::Ucmm(const Bytes &mr, const std::vector<Item> &extra, std::vector<Item> *itemsOut) {
    Bytes data;
    Put32(data, 0);
    Put16(data, 0);
    std::vector<Item> items = {{0x0000, {}}, {0x00B2, mr}};
    items.insert(items.end(), extra.begin(), extra.end());
    const Bytes cpf = Cpf(items);
    data.insert(data.end(), cpf.begin(), cpf.end());
    Bytes body;
    Encap(0x6F, data, nullptr, &body);
    if (body.size() < 6) {
        throw std::runtime_error("short reply");
    }
    auto reply = ParseCpf(body.data() + 6, body.size() - 6);
    if (itemsOut) {
        *itemsOut = reply;
    }
    for (const Item &i : reply) {
        if (i.type == 0x00B2) {
            return i.data;
        }
    }
    throw std::runtime_error("no unconnected data item in reply");
}

CipReply Link::Explicit(uint8_t service, const Bytes &path, const Bytes &data) {
    return ParseReply(Ucmm(CipRequest(service, path, data), {}, nullptr));
}

std::vector<Item> Link::ListServices() {
    Bytes body;
    Encap(0x04, {}, nullptr, &body);
    return ParseCpf(body.data(), body.size());
}

OpenReply Link::ForwardOpen(const OpenOptions &o) {
    m_udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    const BOOL one = TRUE;
    setsockopt(m_udp, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&one), sizeof(one));
    // Multicast, and input-only (which never sends from its port, so a PC firewall would drop the board's
    // point-to-point replies to an ephemeral port): receive on 2222.
    sockaddr_in me = Addr(g_localIp, (o.multicast || o.inputOnly) ? kIoPort : 0);
    if (bind(m_udp, reinterpret_cast<sockaddr *>(&me), sizeof(me)) != 0) {
        throw std::runtime_error("cannot bind UDP on " + g_localIp);
    }
    int n = sizeof(me);
    getsockname(m_udp, reinterpret_cast<sockaddr *>(&me), &n);
    const uint16_t localPort = ntohs(me.sin_port);
    m_o2tNull = o.inputOnly;
    m_connPath = o.connPath;
    if (m_connPath.empty()) {
        m_connPath = o.inputOnly ? Bytes{0x20, 0x04, 0x24, kAsmCfg, 0x2C, kAsmSts}
                                 : Bytes{0x20, 0x04, 0x24, kAsmCfg, 0x2C, kAsmCmd, 0x2C, kAsmSts};
    }
    const uint32_t rpiUs = static_cast<uint32_t>(m_rpi * 1e6 + 0.5);
    const uint32_t t2oType = o.multicast ? 1 : 2, o2tType = o.inputOnly ? 0 : 2;
    const uint32_t o2tSize = o.inputOnly ? 0 : static_cast<uint32_t>(o.o2tSize), t2oSize = static_cast<uint32_t>(o.t2oSize);
    Bytes p = {0x0A, 0x0E};
    Put32(p, 0);
    Put32(p, 1 + Rng()() % 0x7FFFFFFEu);
    Put16(p, m_serial);
    Put16(p, m_vendor);
    Put32(p, m_origSerial);
    p.insert(p.end(), {static_cast<uint8_t>(m_timeoutMult), 0, 0, 0});
    Put32(p, rpiUs);
    if (o.large) {
        Put32(p, (o2tType << 29) | (1u << 26) | o2tSize);
        Put32(p, rpiUs);
        Put32(p, (t2oType << 29) | (1u << 26) | t2oSize);
    } else {
        Put16(p, static_cast<uint16_t>((o2tType << 13) | 0x0800 | o2tSize));
        Put32(p, rpiUs);
        Put16(p, static_cast<uint16_t>((t2oType << 13) | 0x0800 | t2oSize));
    }
    p.push_back(o.transport);
    p.push_back(static_cast<uint8_t>(m_connPath.size() / 2));
    p.insert(p.end(), m_connPath.begin(), m_connPath.end());
    Bytes mr = {static_cast<uint8_t>(o.large ? 0x5B : 0x54), 0x02, 0x20, 0x06, 0x24, 0x01};
    mr.insert(mr.end(), p.begin(), p.end());
    std::vector<Item> extra;
    if (!o.multicast) {
        Bytes sock = {0, 2, static_cast<uint8_t>(localPort >> 8), static_cast<uint8_t>(localPort)};
        sock.resize(16, 0);
        extra.push_back({0x8001, sock});
    }
    OpenReply out;
    const Bytes reply = Ucmm(mr, extra, &out.items);
    static_cast<CipReply &>(out) = ParseReply(reply);
    if (out.gen != 0) {
        closesocket(m_udp);
        m_udp = INVALID_SOCKET;
        return out;
    }
    m_o2tId = Get32(out.data.data());
    m_t2oId = Get32(out.data.data() + 4);
    m_dest = Addr(m_ip, kIoPort);
    std::string group;
    for (const Item &i : out.items) {
        if (i.type == 0x8000 && i.data.size() >= 8) {
            m_dest.sin_port = htons(static_cast<uint16_t>(i.data[2] << 8 | i.data[3]));
        }
        if (i.type == 0x8001 && i.data.size() >= 8) {  // T->O socket: the multicast group to join
            char t[20];
            snprintf(t, sizeof(t), "%u.%u.%u.%u", i.data[4], i.data[5], i.data[6], i.data[7]);
            group = t;
        }
    }
    if (o.multicast && !group.empty()) {
        ip_mreq m{};
        inet_pton(AF_INET, group.c_str(), &m.imr_multiaddr);
        inet_pton(AF_INET, g_localIp.c_str(), &m.imr_interface);
        setsockopt(m_udp, IPPROTO_IP, IP_ADD_MEMBERSHIP, reinterpret_cast<const char *>(&m), sizeof(m));
    }
    if (o.inputOnly && !o.multicast) {
        // A PC firewall drops unicast to a port that has sent nothing, and an input-only connection never sends:
        // one datagram opens its pinhole (the board counts it as one bad packet). A PLC needs nothing of the kind.
        const char two[2] = {0, 0};
        sockaddr_in d = Addr(m_ip, kIoPort);
        sendto(m_udp, two, 2, 0, reinterpret_cast<sockaddr *>(&d), sizeof(d));
    }
    m_seq32 = 0;
    m_stop = false;
    m_thread = std::thread([this] { IoLoop(); });
    return out;
}

Link &Link::Open(const OpenOptions &o) {
    if (UsingPlc()) {
        // No session to the board: it takes two, and the PLC already holds one.
        m_bridge = true;
        m_stop = false;
        m_thread = std::thread([this] { PlcLoop(); });
    } else {
        if (m_tcp == INVALID_SOCKET) {
            Connect();
        }
        const OpenReply r = ForwardOpen(o);
        if (r.gen != 0) {
            throw std::runtime_error("Forward_Open failed: general " + Hex(r.gen) + " extended " + HexList(r.ext));
        }
    }
    const double end = Seconds() + 3;
    while (!HaveStatus()) {
        if (Seconds() > end) {
            throw std::runtime_error("Forward_Open ok but no T->O data (firewall on UDP?)");
        }
        Sleep(10);
    }
    return *this;
}

uint8_t Link::ForwardClose() {
    if (m_connPath.empty()) {
        return 0xFF;
    }
    Bytes mr = {0x4E, 0x02, 0x20, 0x06, 0x24, 0x01, 0x0A, 0x0E};
    Put16(mr, m_serial);
    Put16(mr, m_vendor);
    Put32(mr, m_origSerial);
    mr.push_back(static_cast<uint8_t>(m_connPath.size() / 2));
    mr.push_back(0);
    mr.insert(mr.end(), m_connPath.begin(), m_connPath.end());
    return ParseReply(Ucmm(mr, {}, nullptr)).gen;
}

void Link::StopThread() {
    m_stop = true;
    if (m_thread.joinable()) {
        m_thread.join();
    }
    if (m_bridge) {
        m_bridge = false;
        PlcClearOutput(g_plc);
    }
}

// Through a PLC no cable can be pulled from here. The PLC lets go instead: safe, but a release and not a connection loss,
// so the checks that follow expect something this path can't do.
void Link::StopIo() { StopThread(); }

void Link::Drop() {
    StopIo();
    if (m_udp != INVALID_SOCKET) {
        closesocket(m_udp);
        m_udp = INVALID_SOCKET;
    }
}

void Link::Close() {
    StopThread();
    if (m_udp != INVALID_SOCKET && m_tcp != INVALID_SOCKET) {
        try {
            ForwardClose();
        } catch (const std::exception &) {
        }
    }
    if (m_tcp != INVALID_SOCKET) {
        const Bytes h = EncapHeader(0x66, 0, m_session);  // UnRegisterSession: no reply
        send(m_tcp, reinterpret_cast<const char *>(h.data()), static_cast<int>(h.size()), 0);
        closesocket(m_tcp);
        m_tcp = INVALID_SOCKET;
    }
    if (m_udp != INVALID_SOCKET) {
        closesocket(m_udp);
        m_udp = INVALID_SOCKET;
    }
}

void Link::SetFlags(uint16_t f) {
    std::lock_guard<std::mutex> g(m_lock);
    m_cmd[0] = static_cast<uint8_t>(f);
    m_cmd[1] = static_cast<uint8_t>(f >> 8);
}

uint16_t Link::Flags() {
    std::lock_guard<std::mutex> g(m_lock);
    return Get16(m_cmd);
}

void Link::SetValue(size_t offset, int32_t v) {
    std::lock_guard<std::mutex> g(m_lock);
    for (int i = 0; i < 4; i++) {
        m_cmd[offset + static_cast<size_t>(i)] = static_cast<uint8_t>(static_cast<uint32_t>(v) >> (8 * i));
    }
}

void Link::SetByte(size_t offset, uint8_t v) {
    std::lock_guard<std::mutex> g(m_lock);
    m_cmd[offset] = v;
}

bool Link::HaveStatus() {
    std::lock_guard<std::mutex> g(m_lock);
    return !m_status.empty();
}

Bytes Link::Status() {
    std::lock_guard<std::mutex> g(m_lock);
    return m_status;
}

double Link::StatusAge() {
    std::lock_guard<std::mutex> g(m_lock);
    return PreciseSeconds() - m_statusTime;
}

double Link::StatusTime() {
    std::lock_guard<std::mutex> g(m_lock);
    return m_statusTime;
}

void Link::ClearTxLog() {
    std::lock_guard<std::mutex> g(m_lock);
    m_txLog.clear();
    m_lastTxFlags = -1;
}

std::vector<double> Link::RxTimes() {
    std::lock_guard<std::mutex> g(m_lock);
    return m_rxTimes;
}

std::vector<std::pair<double, uint16_t>> Link::TxLog() {
    std::lock_guard<std::mutex> g(m_lock);
    return m_txLog;
}

void Link::PlcLoop() {
    auto s = std::make_unique<EipSession>();
    memset(s.get(), 0, sizeof(EipSession));
    bool open = false, sentAny = false;
    uint8_t sent[kCmdSize] = {};
    while (!m_stop) {
        if (!open) {
            open = PlcOpen(s.get(), g_plc);
            if (!open) {
                printf("bridge: no session to the PLC\n");
                Sleep(200);
                continue;
            }
        }
        uint8_t out[kCmdSize], in[kStsSize];
        {
            std::lock_guard<std::mutex> g(m_lock);
            memcpy(out, m_cmd, sizeof(out));
        }
        int rc = EIP_OK;
        if (!sentAny || memcmp(out, sent, sizeof(out)) != 0) {
            rc = eip_tag_write(s.get(), kPlcOut, EIP_SINT, kCmdSize, out, kCmdSize, nullptr);
            if (rc == EIP_OK) {
                memcpy(sent, out, sizeof(sent));
                sentAny = true;
                std::lock_guard<std::mutex> g(m_lock);
                m_txLog.emplace_back(PreciseSeconds(), Get16(out));
            }
        }
        uint16_t len = sizeof(in);
        if (rc == EIP_OK) {
            rc = eip_tag_read(s.get(), kPlcIn, kStsSize, in, &len, nullptr, nullptr);
        }
        if (rc != EIP_OK || len != kStsSize) {
            printf("bridge: %s\n", eip_status_text(rc));
            eip_session_close(s.get());
            open = false;
            Sleep(50);
            continue;
        }
        {
            std::lock_guard<std::mutex> g(m_lock);
            m_status.assign(in, in + kStsSize);
            m_statusTime = PreciseSeconds();
            m_rxTimes.push_back(m_statusTime);
            m_rxCount++;
        }
        Sleep(10);
    }
    if (open) {
        eip_session_close(s.get());
    }
}

void Link::IoLoop() {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
    double next = PreciseSeconds();
    uint8_t pkt[2048];
    while (!m_stop) {
        const double now = PreciseSeconds();
        if (m_o2tNull || m_hold) {
            next = now + m_rpi;
        } else if (now >= next) {
            m_seq32++;
            Bytes payload;
            Put16(payload, static_cast<uint16_t>(m_seq32));
            Put32(payload, m_run ? 1 : 0);
            {
                std::lock_guard<std::mutex> g(m_lock);
                payload.insert(payload.end(), m_cmd, m_cmd + kCmdSize);
                const uint16_t f = Get16(m_cmd);
                if (f != m_lastTxFlags) {
                    m_lastTxFlags = f;
                    m_txLog.emplace_back(now, f);
                }
            }
            Bytes addr;
            Put32(addr, m_o2tId);
            Put32(addr, m_seq32);
            const Bytes out = Cpf({{0x8002, addr}, {0x00B1, payload}});
            sendto(m_udp, reinterpret_cast<const char *>(out.data()), static_cast<int>(out.size()), 0,
                   reinterpret_cast<sockaddr *>(&m_dest), sizeof(m_dest));
            next += m_rpi;
            if (next < now) {
                next = now + m_rpi;
            }
        }
        fd_set r;
        FD_ZERO(&r);
        FD_SET(m_udp, &r);
        timeval t = {0, 0};
        if (select(0, &r, nullptr, nullptr, &t) <= 0) {
            continue;
        }
        const int n = recv(m_udp, reinterpret_cast<char *>(pkt), sizeof(pkt), 0);
        if (n <= 0) {
            continue;
        }
        const double at = PreciseSeconds();
        const auto items = ParseCpf(pkt, static_cast<size_t>(n));
        const Bytes *conn = nullptr, *data = nullptr;
        for (const Item &i : items) {
            if (i.type == 0x8002) conn = &i.data;
            if (i.type == 0x00B1) data = &i.data;
        }
        if (conn && data && conn->size() >= 4 && Get32(conn->data()) == m_t2oId && data->size() == 2u + kStsSize) {
            std::lock_guard<std::mutex> g(m_lock);
            m_status.assign(data->begin() + 2, data->end());
            m_statusTime = at;
            m_rxTimes.push_back(at);
            m_rxCount++;
        }
    }
}

Identity ListIdentity(const std::string &ip, double timeout) {
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    Timeout(s, timeout);
    const Bytes h = EncapHeader(0x63, 0, 0);
    sockaddr_in a = Addr(ip, kEnipPort);
    sendto(s, reinterpret_cast<const char *>(h.data()), static_cast<int>(h.size()), 0, reinterpret_cast<sockaddr *>(&a), sizeof(a));
    uint8_t buf[1024];
    const int n = recv(s, reinterpret_cast<char *>(buf), sizeof(buf), 0);
    closesocket(s);
    if (n <= 0) {
        throw std::runtime_error("no ListIdentity reply from " + ip);
    }
    return ParseIdentity(buf, static_cast<size_t>(n));
}

namespace {

/** Broadcasts one datagram from this PC's bench address and hands each reply to onReply until timeout. */
template <typename F>
void BroadcastAsk(const Bytes &ask, const std::string &to, uint16_t port, double timeout, F onReply) {
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    const BOOL one = TRUE;
    setsockopt(s, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char *>(&one), sizeof(one));
    sockaddr_in me = Addr(g_localIp, 0);
    bind(s, reinterpret_cast<sockaddr *>(&me), sizeof(me));
    sockaddr_in a = Addr(to, port);
    sendto(s, reinterpret_cast<const char *>(ask.data()), static_cast<int>(ask.size()), 0, reinterpret_cast<sockaddr *>(&a), sizeof(a));
    Timeout(s, timeout);
    uint8_t buf[1024];
    for (;;) {
        sockaddr_in from{};
        int fl = sizeof(from);
        const int n = recvfrom(s, reinterpret_cast<char *>(buf), sizeof(buf), 0, reinterpret_cast<sockaddr *>(&from), &fl);
        if (n <= 0) {
            break;
        }
        onReply(from, buf, static_cast<size_t>(n));
    }
    closesocket(s);
}

}  // namespace

std::vector<Identity> Scan(const std::string &broadcast, double timeout) {
    std::vector<Identity> found;
    BroadcastAsk(EncapHeader(0x63, 0, 0), broadcast, kEnipPort, timeout, [&](const sockaddr_in &, const uint8_t *b, size_t n) {
        try {
            found.push_back(ParseIdentity(b, n));
        } catch (const std::exception &) {
        }
    });
    return found;
}

std::vector<std::pair<std::string, std::string>> Discover(double timeout) {
    std::vector<std::pair<std::string, std::string>> found;
    const char *ask = "ECAM_DISCOVER?";
    BroadcastAsk(Bytes(ask, ask + 14), g_broadcast, kDiscoveryPort, timeout, [&](const sockaddr_in &from, const uint8_t *b, size_t n) {
        char t[20];
        inet_ntop(AF_INET, &from.sin_addr, t, sizeof(t));
        std::string text(reinterpret_cast<const char *>(b), n);
        while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) {
            text.pop_back();
        }
        found.emplace_back(t, text);
    });
    return found;
}

uint8_t Multiple(Link &l, const std::vector<Bytes> &entries, std::vector<std::pair<uint8_t, Bytes>> *out) {
    const size_t head = 2 + 2 * entries.size();
    Bytes body, data;
    Put16(data, static_cast<uint16_t>(entries.size()));
    for (const Bytes &e : entries) {
        Put16(data, static_cast<uint16_t>(head + body.size()));
        body.insert(body.end(), e.begin(), e.end());
    }
    data.insert(data.end(), body.begin(), body.end());
    const CipReply r = l.Explicit(0x0A, Path(0x02, 1), data);
    out->clear();
    if ((r.gen == 0 || r.gen == 0x1E) && r.data.size() >= 2) {
        const uint16_t count = Get16(r.data.data());
        for (uint16_t i = 0; i < count; i++) {
            const size_t from = Get16(r.data.data() + 2 + 2 * i);
            const size_t to = i + 1 < count ? Get16(r.data.data() + 4 + 2 * i) : r.data.size();
            const CipReply e = ParseReply(Bytes(r.data.begin() + static_cast<long>(from), r.data.begin() + static_cast<long>(to)));
            out->emplace_back(e.gen, e.data);
        }
    }
    return r.gen;
}

}  // namespace bench
