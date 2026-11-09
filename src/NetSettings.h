/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// The board's network settings as the console shows them (Firmware/PROTOCOL.md: config, set), and what the Board
// page checks before it sends any. Network keys apply after a restart.

#include <QRegularExpression>
#include <QString>
#include <QStringList>

#include <array>
#include <optional>

namespace net {

struct Settings {
    bool valid = false;  // read from a board
    bool dhcp = false;
    QString ip, netmask, gateway;
    QString host;  // empty: the default from the MAC

    bool operator==(const Settings &o) const {
        return dhcp == o.dhcp && ip == o.ip && netmask == o.netmask && gateway == o.gateway && host == o.host;
    }
    bool operator!=(const Settings &o) const { return !(*this == o); }
};

/** a.b.c.d as four bytes, or nothing. */
inline std::optional<std::array<int, 4>> ParseIp(const QString &text) {
    const QStringList parts = text.trimmed().split(QLatin1Char('.'));
    if (parts.size() != 4) return std::nullopt;
    std::array<int, 4> out{};
    for (int i = 0; i < 4; ++i) {
        bool ok = false;
        const int v = parts[i].toInt(&ok);
        if (!ok || v < 0 || v > 255 || parts[i].isEmpty()) return std::nullopt;
        out[static_cast<size_t>(i)] = v;
    }
    return out;
}

inline quint32 ToWord(const std::array<int, 4> &a) {
    return static_cast<quint32>(a[0]) << 24 | static_cast<quint32>(a[1]) << 16 | static_cast<quint32>(a[2]) << 8 |
           static_cast<quint32>(a[3]);
}

/** What's wrong with these settings, or empty. Addresses are checked on DHCP too: they are its fallback. */
inline QString Check(const Settings &s) {
    const auto ip = ParseIp(s.ip), mask = ParseIp(s.netmask), gw = ParseIp(s.gateway);
    if (!ip) return QStringLiteral("IP address: four numbers 0 to 255, such as 192.168.1.10.");
    if (!mask) return QStringLiteral("Netmask: four numbers 0 to 255, such as 255.255.255.0.");
    if (!gw) return QStringLiteral("Gateway: four numbers 0 to 255; 0.0.0.0 for none.");
    const quint32 a = ToWord(*ip), m = ToWord(*mask), g = ToWord(*gw);
    // Ones then zeros, with at least one of each.
    if (m == 0 || m == 0xFFFFFFFFu || (~m & (~m + 1)) != 0) return QStringLiteral("Netmask: not a valid mask.");
    if ((*ip)[0] == 0 || (*ip)[0] == 127 || (*ip)[0] >= 224) return QStringLiteral("IP address: not one a device can use.");
    if ((a & ~m) == 0 || (a & ~m) == ~m) return QStringLiteral("IP address: that is the subnet's network or broadcast address.");
    if (g != 0 && (g & m) != (a & m)) return QStringLiteral("Gateway: not on the same subnet as the IP address.");
    if (g == a) return QStringLiteral("Gateway: the same as the IP address.");
    if (s.host.size() > 63) return QStringLiteral("Host name: 63 characters at most.");
    static const QRegularExpression name(QStringLiteral("^[A-Za-z0-9.-]*$"));
    if (!name.match(s.host).hasMatch()) return QStringLiteral("Host name: letters, digits, - and . only.");
    return {};
}

/** Settings from the lines of `config`. */
inline Settings Parse(const QStringList &lines) {
    Settings s;
    int seen = 0;
    for (const QString &l : lines) {
        if (l.startsWith(QStringLiteral("net_mode "))) {
            s.dhcp = l.mid(9).trimmed() == QStringLiteral("0");
            ++seen;
        } else if (l.startsWith(QStringLiteral("ip "))) {
            s.ip = l.mid(3).trimmed();
            ++seen;
        } else if (l.startsWith(QStringLiteral("netmask "))) {
            s.netmask = l.mid(8).trimmed();
            ++seen;
        } else if (l.startsWith(QStringLiteral("gateway "))) {
            s.gateway = l.mid(8).trimmed();
            ++seen;
        } else if (l.startsWith(QStringLiteral("host "))) {
            const QString h = l.mid(5).trimmed();
            s.host = h.startsWith(QLatin1Char('-')) ? QString() : h;  // "- (default)"
            ++seen;
        }
    }
    s.valid = seen == 5;
    return s;
}

/** The address in use, from the `eip` command's network line (ip=...), or empty. */
inline QString InUse(const QStringList &lines) {
    for (const QString &l : lines) {
        if (!l.startsWith(QStringLiteral("eip network"))) continue;
        const int at = l.indexOf(QStringLiteral(" ip="));
        return at < 0 ? QString() : l.mid(at + 4).section(QLatin1Char(' '), 0, 0);
    }
    return {};
}

/** The set commands that take a board from one to the other, and nothing it already has. */
inline QStringList Commands(const Settings &from, const Settings &to) {
    QStringList out;
    if (to.dhcp != from.dhcp) out << QStringLiteral("set net_mode %1").arg(to.dhcp ? 0 : 1);
    if (to.ip != from.ip) out << QStringLiteral("set ip %1").arg(to.ip);
    if (to.netmask != from.netmask) out << QStringLiteral("set netmask %1").arg(to.netmask);
    if (to.gateway != from.gateway) out << QStringLiteral("set gateway %1").arg(to.gateway);
    if (to.host != from.host) out << QStringLiteral("set host %1").arg(to.host.isEmpty() ? QStringLiteral("-") : to.host);
    return out;
}

}  // namespace net
