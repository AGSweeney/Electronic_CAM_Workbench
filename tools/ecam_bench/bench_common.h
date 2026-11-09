// MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
// What the bench tools share: the board's text console, check results and a clock.
#ifndef ECAM_BENCH_COMMON_H_
#define ECAM_BENCH_COMMON_H_

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include "eip.h"
#include "eip_hal.h"

namespace bench {

constexpr uint16_t kConsolePort = 9500;

/** Seconds on the performance counter, for anything timed finer than the 16 ms system tick. */
inline double PreciseSeconds() {
    static const double freq = [] {
        LARGE_INTEGER f;
        QueryPerformanceFrequency(&f);
        return static_cast<double>(f.QuadPart);
    }();
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    return static_cast<double>(c.QuadPart) / freq;
}

inline double Seconds() {
    static const ULONGLONG start = GetTickCount64();
    return (GetTickCount64() - start) / 1000.0;
}

inline long long IntAfter(const std::string &s, const char *key) {
    const size_t at = s.find(key);
    return at == std::string::npos ? -1 : atoll(s.c_str() + at + strlen(key));
}

class Console {
public:
    ~Console() { Close(); }
    void Open(const char *ip) {
        Close();
        uint8_t a[4];
        inet_pton(AF_INET, ip, a);
        for (int i = 0; i < 25 && m_sock == EIP_SOCK_INVALID; i++) {
            m_sock = eip_hal_tcp_connect(a, kConsolePort, 2000);
            if (m_sock == EIP_SOCK_INVALID) {
                Sleep(200);
            }
        }
        if (m_sock == EIP_SOCK_INVALID) {
            throw std::runtime_error("no console connection");
        }
        Drain(500);
    }
    void Close() {
        if (m_sock != EIP_SOCK_INVALID) {
            Send("exit\n");
            eip_hal_tcp_close(m_sock);
            m_sock = EIP_SOCK_INVALID;
        }
        m_pending.clear();
    }
    void Send(const std::string &t) {
        eip_hal_tcp_send(m_sock, reinterpret_cast<const uint8_t *>(t.data()), static_cast<uint16_t>(t.size()));
    }
    /** The reply's lines, ending at ok or error. TEL lines are counted and left out. */
    std::vector<std::string> Cmd(const std::string &c, double timeout = 5.0) {
        Send(c + "\n");
        std::vector<std::string> lines;
        const double end = Seconds() + timeout;
        while (Seconds() < end) {
            std::string l;
            if (!NextLine(l, 50)) {
                continue;
            }
            lines.push_back(l);
            if (l.rfind("ok", 0) == 0 || l.rfind("error", 0) == 0) {
                return lines;
            }
        }
        throw std::runtime_error("console: no reply to '" + c + "'");
    }
    void Drain(uint32_t ms) {
        const double end = Seconds() + ms / 1000.0;
        std::string l;
        while (Seconds() < end) {
            NextLine(l, 20);
        }
    }
    uint64_t TelLines() const { return m_tel; }

private:
    bool NextLine(std::string &out, uint32_t ms) {
        for (;;) {
            const size_t at = m_pending.find('\n');
            if (at != std::string::npos) {
                out = m_pending.substr(0, at);
                m_pending.erase(0, at + 1);
                if (!out.empty() && out.back() == '\r') {
                    out.pop_back();
                }
                if (out.rfind("TEL ", 0) == 0) {
                    m_tel++;
                    continue;
                }
                return true;
            }
            uint8_t buf[2048];
            const int n = eip_hal_tcp_recv(m_sock, buf, sizeof(buf), ms);
            if (n < 0) {
                throw std::runtime_error("console connection closed");
            }
            if (n == 0) {
                return false;
            }
            m_pending.append(reinterpret_cast<char *>(buf), static_cast<size_t>(n));
        }
    }

    eip_sock_t m_sock = EIP_SOCK_INVALID;
    std::string m_pending;
    uint64_t m_tel = 0;
};

inline std::string Find(const std::vector<std::string> &lines, const char *prefix) {
    for (const std::string &l : lines) {
        if (l.rfind(prefix, 0) == 0) {
            return l;
        }
    }
    return "";
}

struct Results {
    int passed = 0, total = 0;
    void Check(const std::string &what, bool ok, const std::string &detail = "") {
        total++;
        passed += ok;
        printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(), detail.empty() ? "" : " - ", detail.c_str());
        fflush(stdout);
    }
};

/** A program's stdout. Empty if it fails to start; it is killed at the time limit. */
inline std::string Exec(const std::string &command, DWORD timeoutMs, int *exitCode = nullptr,
                        const std::function<void(const std::string &)> &onLine = nullptr) {
    SECURITY_ATTRIBUTES sa = {sizeof(sa), nullptr, TRUE};
    HANDLE rd = nullptr, wr = nullptr;
    if (!CreatePipe(&rd, &wr, &sa, 0)) {
        return "";
    }
    SetHandleInformation(rd, HANDLE_FLAG_INHERIT, 0);
    STARTUPINFOA si;
    memset(&si, 0, sizeof(si));
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdOutput = si.hStdError = wr;
    PROCESS_INFORMATION pi;
    std::string cmd = command;
    const BOOL ok = CreateProcessA(nullptr, cmd.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi);
    const DWORD startError = ok ? 0 : GetLastError();
    CloseHandle(wr);
    std::string out;
    size_t shown = 0;
    if (!ok) {
        printf("    could not start %s: Windows error %lu\n", command.c_str(), startError);
    } else {
        const ULONGLONG end = GetTickCount64() + timeoutMs;
        for (;;) {
            DWORD avail = 0;
            if (!PeekNamedPipe(rd, nullptr, 0, nullptr, &avail, nullptr)) {
                break;  // the program has gone and the pipe is empty
            }
            if (avail) {
                char buf[1024];
                DWORD n = 0;
                if (ReadFile(rd, buf, sizeof(buf), &n, nullptr) && n) {
                    out.append(buf, n);
                    if (onLine) {
                        size_t at;
                        while ((at = out.find('\n', shown)) != std::string::npos) {
                            std::string line = out.substr(shown, at - shown);
                            if (!line.empty() && line.back() == '\r') {
                                line.pop_back();
                            }
                            onLine(line);
                            shown = at + 1;
                        }
                    }
                }
                continue;
            }
            if (GetTickCount64() > end) {
                TerminateProcess(pi.hProcess, 1);
                break;
            }
            Sleep(10);
        }
        if (exitCode) {
            WaitForSingleObject(pi.hProcess, 2000);
            DWORD code = 1;
            GetExitCodeProcess(pi.hProcess, &code);
            *exitCode = static_cast<int>(code);
        }
        CloseHandle(pi.hThread);
        CloseHandle(pi.hProcess);
    }
    CloseHandle(rd);
    return out;
}

}  // namespace bench

#endif  // ECAM_BENCH_COMMON_H_
