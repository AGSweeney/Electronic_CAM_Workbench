// MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
// A PC stand-in for a PLC, with control over the bytes on the wire. The suites need what QuickEIP's scanner rightly
// won't send: wrong sizes, Class 3, multicast and input-only variants, a connection dropped without a Forward_Close.
// Everything that keeps to the rules uses QuickEIP itself (see scanner, soak and nettest).
#ifndef ECAM_BENCH_LINK_H_
#define ECAM_BENCH_LINK_H_

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace bench {

using Bytes = std::vector<uint8_t>;

constexpr uint16_t kEnipPort = 44818, kIoPort = 2222, kDiscoveryPort = 9502;
constexpr uint8_t kAsmCmd = 150, kAsmSts = 100, kAsmCfg = 151;
constexpr uint16_t kCmdSize = 64, kStsSize = 160;

struct Item {
    uint16_t type;
    Bytes data;
};

struct CipReply {
    uint8_t gen = 0xFF;
    std::vector<uint16_t> ext;
    Bytes data;
};

struct OpenOptions {
    int o2tSize = 2 + 4 + kCmdSize;
    int t2oSize = 2 + kStsSize;
    bool multicast = false, inputOnly = false, large = false;
    Bytes connPath;  // empty: the normal path
    uint8_t transport = 0x01;
};

struct OpenReply : CipReply {
    std::vector<Item> items;
};

Bytes Path(uint8_t cls, uint8_t inst, int attr = -1);
Bytes CipRequest(uint8_t service, const Bytes &path, const Bytes &data = {});
std::string Hex(unsigned v, int width = 2);
std::string HexList(const std::vector<uint16_t> &v);

/** This PC's address on the way to ip, and that /24's broadcast. Sockets bind to it, so replies come back. */
void UseNetworkOf(const std::string &ip);
const std::string &LocalIp();
const std::string &Broadcast();

/** From now on Link::Open goes through this PLC: commands by tag write to ECam_O, the only tag this writes, status by tag
 *  read from ECam_I, and the PLC's own Class 1 connection carries them. Console and explicit traffic to the board stay as
 *  they are. Tests that need to own the connection can't apply on this path, and their failures say nothing about the board. */
void UsePlc(const std::string &plcIp);
bool UsingPlc();
/** On a session of its own, so it works whatever else is happening: the PLC goes on holding what was written last. */
bool PlcClearOutput(const std::string &plcIp);

class Link {
public:
    explicit Link(std::string ip, int rpiMs = 20, uint16_t vendor = 0x1234, uint32_t origSerial = 0);
    ~Link();
    Link(const Link &) = delete;
    Link &operator=(const Link &) = delete;

    Link &Connect();
    CipReply Explicit(uint8_t service, const Bytes &path, const Bytes &data = {});
    std::vector<Item> ListServices();
    OpenReply ForwardOpen(const OpenOptions &o = {});
    Link &Open(const OpenOptions &o = {});
    uint8_t ForwardClose();
    void Close();
    /** Stops sending and receiving at once, with no Forward_Close: a pulled cable, seen from the board. */
    void StopIo();
    /** StopIo, and the UDP socket goes too. */
    void Drop();

    uint32_t Session() const { return m_session; }
    void SetFlags(uint16_t f);
    uint16_t Flags();
    void SetValue(size_t offset, int32_t v);
    void SetByte(size_t offset, uint8_t v);
    void SetRun(bool run) { m_run = run; }
    void SetTimeoutMult(int m) { m_timeoutMult = m; }
    /** While true, O->T stops but the connection is kept: the board sees packets go missing. */
    void HoldSending(bool hold) { m_hold = hold; }

    bool HaveStatus();
    Bytes Status();
    double StatusAge();  // seconds since the last status packet
    double StatusTime();
    void ClearTxLog();
    uint64_t RxCount() const { return m_rxCount; }
    std::vector<double> RxTimes();
    std::vector<std::pair<double, uint16_t>> TxLog();

private:
    void Encap(uint16_t command, const Bytes &data, uint32_t *session, Bytes *body);
    void RecvExact(uint8_t *to, size_t n);
    Bytes Ucmm(const Bytes &mr, const std::vector<Item> &extra, std::vector<Item> *items);
    void IoLoop();
    void PlcLoop();
    void StopThread();

    std::string m_ip;
    double m_rpi;
    uint16_t m_vendor, m_serial;
    uint32_t m_origSerial;
    int m_timeoutMult = 1;  // Forward_Open: timeout = RPI x 4 << this. At 20 ms, the PLC's 160 ms
    SOCKET m_tcp = INVALID_SOCKET, m_udp = INVALID_SOCKET;
    uint32_t m_session = 0, m_o2tId = 0, m_t2oId = 0, m_seq32 = 0;
    sockaddr_in m_dest{};
    Bytes m_connPath;
    bool m_o2tNull = false;
    bool m_bridge = false;  // Open went through the PLC

    std::thread m_thread;
    std::atomic<bool> m_stop{false}, m_run{true}, m_hold{false};
    std::atomic<uint64_t> m_rxCount{0};
    std::mutex m_lock;
    uint8_t m_cmd[kCmdSize] = {};
    Bytes m_status;
    double m_statusTime = 0;
    std::vector<double> m_rxTimes;
    std::vector<std::pair<double, uint16_t>> m_txLog;
    int m_lastTxFlags = -1;
};

struct Identity {
    std::string ip, name;
    uint16_t vendor = 0, type = 0, product = 0, status = 0;
    uint8_t revMajor = 0, revMinor = 0, state = 0;
    uint32_t serial = 0;
};

Identity ListIdentity(const std::string &ip, double timeout = 2.0);
std::vector<Identity> Scan(const std::string &broadcast, double timeout = 1.5);
/** Each answer to ECAM_DISCOVER?: (address, text). */
std::vector<std::pair<std::string, std::string>> Discover(double timeout = 1.5);

/** Multiple Service Packet to the Message Router: the general status, and each entry's (status, data). */
uint8_t Multiple(Link &l, const std::vector<Bytes> &entries, std::vector<std::pair<uint8_t, Bytes>> *out);

}  // namespace bench

#endif  // ECAM_BENCH_LINK_H_
