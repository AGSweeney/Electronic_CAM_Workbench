/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#include "EipLink.h"

#include "CamEngine.h"
#include "ClearCore.h"
#include "ECamConfig.h"
#include "EipProtocol.h"
#include "Settings.h"
#include "Transport.h"

#include "eip.h"
#include "eip_clearcore.h"

#include <stdio.h>
#include <string.h>

static const EipAssemblyConfig kAssemblies[] = {
    /* instance,       direction,        size,              name,      initial, implicit_only */
    {ECAM_EIP_ASM_CMD, EIP_ASM_CONSUMED, ECAM_EIP_CMD_SIZE, "ECamCmd", 0, 0},
    {ECAM_EIP_ASM_STS, EIP_ASM_PRODUCED, ECAM_EIP_STS_SIZE, "ECamSts", 0, 0},
    {ECAM_EIP_ASM_CFG, EIP_ASM_CONFIG, 0, "ECamCfg", 0, 0},
};

static_assert(eipp::kStsSize == ECAM_EIP_STS_SIZE, "ECamSts size: EipProtocol.h and ECamConfig.h differ");
static_assert(eipp::kCmdSize == ECAM_EIP_CMD_SIZE, "ECamCmd size: EipProtocol.h and ECamConfig.h differ");

static bool g_started = false;
static char g_host[65];                 /* the host name in use */
static volatile bool g_restartPending = false;
static const char *g_restartWhy = "";

/* The command assembly (PROTOCOL.md, Command assembly 150). */
static bool g_active = false;       /* the connection is open and the PLC in Run */
static bool g_armed = false;        /* data seen since it became active: edges count from here */
static uint32_t g_activeRx = 0;     /* rx_packets when it became active */
static uint16_t g_prevFlags = 0;    /* ControlFlags last pass (0 while inactive) */
static bool g_inControl = false;
static uint16_t g_reject = 0;       /* RejectCode */
static uint32_t g_lossCount = 0;    /* control lost to a close or Program (the adapter counts timeouts) */
static uint32_t g_seenTimeouts = 0;

/* The host name in use: the stored one, or "ecam-" + the last 3 MAC bytes. */
static void EffectiveHost(char *out, size_t len) {
    if (g_settings.host[0]) {
        snprintf(out, len, "%s", g_settings.host);
        return;
    }
    const uint8_t *mac = EthernetMgr.MacAddress();
    snprintf(out, len, "%s%02x%02x%02x", ECAM_EIP_HOST_PREFIX, mac ? mac[3] : 0, mac ? mac[4] : 0, mac ? mac[5] : 0);
}

/* ---- CIP callbacks (from eip_adapter_poll) ---------------------------------------------------- */

/* TCP/IP Interface sets and Identity Reset only while the motors are off and nothing moves:
 * 0x10 (device state conflict) otherwise, and nothing changes. */
static uint8_t AllowChange(void *, uint8_t what) {
    if (EngineSafeForNetworkChange()) {
        return 0;
    }
    TransportUsbSendLine(what == EIP_CHANGE_RESET ? "eip: identity reset refused (motors enabled or moving)"
                                                  : "eip: network change refused (motors enabled or moving)");
    return 0x10;
}

static void ApplyInterface(void *, const EipInterfaceConfig *c) {
    const uint8_t mode = c->control == 2 ? 0 : 1; /* CIP: 0 static, 2 DHCP; settings: 0 DHCP, 1 static */
    const bool restart = mode != g_settings.netMode || memcmp(c->ip, g_settings.ip, 4) != 0 ||
                         memcmp(c->netmask, g_settings.netmask, 4) != 0 || memcmp(c->gateway, g_settings.gateway, 4) != 0;
    g_settings.netMode = mode;
    memcpy(g_settings.ip, c->ip, 4);
    memcpy(g_settings.netmask, c->netmask, 4);
    memcpy(g_settings.gateway, c->gateway, 4);
    memcpy(g_settings.dns1, c->dns1, 4);
    memcpy(g_settings.dns2, c->dns2, 4);
    snprintf(g_settings.domain, sizeof(g_settings.domain), "%s", c->domain);
    /* The request carries the name in use; store it only when it is a new one, so the default
     * (from the MAC) stays the default. */
    if (strcmp(c->host_name, g_host) != 0) {
        snprintf(g_settings.host, sizeof(g_settings.host), "%s", c->host_name);
    }
    const bool saved = SettingsSave();
    EffectiveHost(g_host, sizeof(g_host));
    eip_clearcore_set_host_name(g_host); /* the host name applies at once */

    char line[160];
    snprintf(line, sizeof(line), "eip: network settings %s: %s %u.%u.%u.%u/%u.%u.%u.%u gw %u.%u.%u.%u host %s%s",
             saved ? "saved" : "SAVE FAILED", mode ? "static" : "dhcp", c->ip[0], c->ip[1], c->ip[2], c->ip[3],
             c->netmask[0], c->netmask[1], c->netmask[2], c->netmask[3], c->gateway[0], c->gateway[1], c->gateway[2],
             c->gateway[3], g_host, saved && restart ? ", restarting" : "");
    TransportUsbSendLine(line);
    if (saved && restart) {
        g_restartWhy = "network settings changed over EtherNet/IP";
        g_restartPending = true;
    }
}

static void ResetDevice(void *, uint8_t type) {
    if (type == 1) {
        SettingsNetworkDefaults();
        TransportUsbSendLine(SettingsSave() ? "eip: identity reset 1: network settings back to defaults"
                                            : "eip: identity reset 1: SAVE FAILED");
        g_restartWhy = "identity reset (factory network settings)";
    } else {
        g_restartWhy = "identity reset";
    }
    g_restartPending = true;
}

/* ---- Running ------------------------------------------------------------------------------------ */

static void Yield();

static void Start() {
    EffectiveHost(g_host, sizeof(g_host));
    eip_clearcore_set_host_name(g_host);
    EipAdapterConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.vendor_id = ECAM_EIP_VENDOR_ID;
    cfg.device_type = ECAM_EIP_DEVICE_TYPE;
    cfg.product_code = ECAM_EIP_PRODUCT_CODE;
    cfg.revision_major = ECAM_EIP_REV_MAJOR;
    cfg.revision_minor = ECAM_EIP_REV_MINOR;
    cfg.product_name = ECAM_EIP_PRODUCT_NAME;
    cfg.serial_number = 0; /* from the MAC */
    cfg.assemblies = kAssemblies;
    cfg.assembly_count = sizeof(kAssemblies) / sizeof(kAssemblies[0]);
    cfg.apply_interface = ApplyInterface;
    cfg.reset = ResetDevice;
    cfg.allow_change = AllowChange;
    cfg.domain = g_settings.domain;
    const int rc = eip_adapter_start(&cfg); /* poll mode on the ClearCore: no task */
    char line[96];
    snprintf(line, sizeof(line), "eip: adapter %s on %s (vendor %u, product %u, host %s)", rc == EIP_OK ? "up" : "FAILED",
             TransportIpString(), ECAM_EIP_VENDOR_ID, ECAM_EIP_PRODUCT_CODE, g_host);
    TransportUsbSendLine(line);
    g_started = rc == EIP_OK;
    if (g_started) {
        EngineSetYield(Yield);
    }
}

/* LED: slow blink while waiting for a PLC connection, on while one is open in Run. */
static void UpdateLed(bool connected) {
    static uint32_t last = 0;
    static bool state = false;
    if (connected) {
        ConnectorLed.State(true);
        return;
    }
    if (Milliseconds() - last >= 500u) {
        last = Milliseconds();
        state = !state;
        ConnectorLed.State(state);
    }
}

/* Restart once nothing moves; keep answering for a moment so the last reply goes out. */
static void ServiceRestart() {
    if (!g_restartPending || !EngineSafeForNetworkChange()) {
        return;
    }
    char line[96];
    snprintf(line, sizeof(line), "eip: restarting (%s)", g_restartWhy);
    TransportUsbSendLine(line);
    const uint32_t t0 = Milliseconds();
    while (Milliseconds() - t0 < 300u) {
        eip_adapter_poll();
        EthernetMgr.Refresh();
    }
    TransportRestart();
}

/* ECamSts: refreshed every ECAM_EIP_STS_REFRESH_MS, so each produced packet (RPI >= 1 ms) carries a
 * status at most that old, and explicit Gets of assembly 100 read it too. */
static void UpdateStatus(const EipAdapterView &view) {
    static uint32_t last = 0;
    const uint32_t now = Milliseconds();
    if (now - last < ECAM_EIP_STS_REFRESH_MS) {
        return;
    }
    last = now;
    eipp::Status st;
    EngineEipStatus(&st);
    st.commLossCount = view.timeouts + g_lossCount;
    st.rejectCode = g_reject;
    if (g_inControl) {
        st.flags |= eipp::kFlagPlcInControl;
    }
    if (g_reject) {
        st.flags |= eipp::kFlagRejected;
    }
    uint8_t data[eipp::kStsSize];
    eipp::Pack(st, data);
    eip_adapter_write(ECAM_EIP_ASM_STS, data, sizeof(data));
}

/* While the engine commits a profile (a few hundred ms of table building and flash writing): keep the
 * network and the connection running, and the heartbeat ticking. The rest of the status holds its last
 * values (the runtime is being rebuilt). Network changes are refused meanwhile (EngineSafeForNetworkChange). */
static void Yield() {
    static uint32_t last = 0;
    const uint32_t now = Milliseconds();
    if (!g_started || now == last) {
        return;
    }
    last = now;
    EthernetMgr.Refresh();
    eip_adapter_poll();
    uint8_t data[eipp::kStsSize];
    uint16_t n = sizeof(data);
    if (eip_adapter_read(ECAM_EIP_ASM_STS, data, &n) == EIP_OK && n == sizeof(data)) {
        eipp::Put32(data + eipp::kStsHeartbeat, now);
        eip_adapter_write(ECAM_EIP_ASM_STS, data, sizeof(data));
    }
}

/* An action's result: accepted clears RejectCode, refused sets it (why, from the engine). */
static void Result(const char *err, const char *what) {
    if (!err) {
        g_reject = 0;
        return;
    }
    const uint16_t code = EngineRefusal();
    g_reject = code ? code : (uint16_t)eipp::kRejectBusy;
    char line[120];
    snprintf(line, sizeof(line), "eip: %s refused (%u): %s", what, (unsigned)g_reject, err);
    TransportUsbSendLine(line);
}

static void Do(const char *(*fn)(), const char *what) {
    (void)EngineRefusal();
    Result(fn(), what);
}

/* The command assembly's values this pass. */
struct Command {
    uint16_t flags;
    int32_t rpm, override, length, jogCrank, jogTime; /* 0.01 rpm, 0.01 %, um, mdeg, us */
    uint32_t waitBits;                                /* the timeline's PLC wait bits (levels) */
};

static Command ReadCommand() {
    Command c = {};
    uint8_t data[eipp::kCmdSize];
    uint16_t n = sizeof(data);
    if (eip_adapter_read(ECAM_EIP_ASM_CMD, data, &n) == EIP_OK && n == sizeof(data)) {
        c.flags = eipp::Get16(data + eipp::kCmdControlFlags);
        c.rpm = (int32_t)eipp::Get32(data + eipp::kCmdCrankRpm);
        c.override = (int32_t)eipp::Get32(data + eipp::kCmdTimelineOverride);
        c.length = (int32_t)eipp::Get32(data + eipp::kCmdCutLength);
        c.jogCrank = (int32_t)eipp::Get32(data + eipp::kCmdJogCrankTarget);
        c.jogTime = (int32_t)eipp::Get32(data + eipp::kCmdJogTimeTarget);
        c.waitBits = eipp::Get32(data + eipp::kCmdWaitBits);
    }
    return c;
}

static Command g_prev = {};          /* last pass (all zero while inactive) */
static bool g_jogOn = false;         /* JOG accepted and still held: its targets are followed */

/* Setpoints (plan 4.1): applied when they change and, the non-zero ones, when the PLC takes control.
 * 0 = the profile's speed / override; a cut length of 0 keeps the current one. */
static void ApplySetpoints(const Command &c, bool tookControl) {
    const EngineMasters m = EngineGetMasters();
    if (c.rpm != g_prev.rpm || (tookControl && c.rpm != 0)) {
        if (c.rpm != 0) {
            (void)EngineRefusal();
            Result(EngineRpm(c.rpm / 100.0), "CrankRpm");
        } else if (m.drivenCrank) {
            Result(EngineRpm(m.rpm), "CrankRpm 0 (the profile's)");
        }
    }
    if (c.override != g_prev.override || (tookControl && c.override != 0)) {
        if (c.override != 0) {
            (void)EngineRefusal();
            Result(EngineOverride(c.override / 10000.0), "TimelineOverride");
        } else if (m.timeline) {
            Result(EngineOverride(m.override), "TimelineOverride 0 (the profile's)");
        }
    }
    if (c.length != 0 && (c.length != g_prev.length || tookControl)) {
        (void)EngineRefusal();
        Result(EngineLength(c.length / 1000.0), "CutLength");
    }
}

/* Jog targets: followed while JOG is held, as they change (the PLC sets them to where the master is,
 * from the status, before raising JOG). */
static void ApplyJogTargets(const Command &c, bool force) {
    const EngineMasters m = EngineGetMasters();
    if (m.crank && (force || c.jogCrank != g_prev.jogCrank)) {
        (void)EngineRefusal();
        Result(EngineJogCrank(c.jogCrank / 1000.0), "JogCrankTarget");
    }
    if (m.timeline && (force || c.jogTime != g_prev.jogTime)) {
        (void)EngineRefusal();
        Result(EngineJogTime(c.jogTime / 1.0e6), "JogTimeTarget");
    }
}

/* ControlFlags: levels while held, edges when a bit turns on, through the console's Engine*() calls. */
static void ServiceCommands(const EipAdapterView &v) {
    const bool active = v.io_open && v.run;
    const Command c = active ? ReadCommand() : Command();
    /* The PLC wait bits are levels, read every pass with no control needed; with no connection in Run they hold
     * every PLC-bit wait (whichever level it wants). */
    EngineSetPlcWaitBits(c.waitBits, active);
    if (active && !g_active) {
        g_armed = false;
        g_activeRx = v.rx_packets;
    }
    g_active = active;
    if (active && !g_armed) {
        /* Bits and values already set when the connection comes up (or the PLC goes to Run) are not edges
         * or changes: wait for the first packet and start from it, so nothing starts by itself on
         * reconnect. Setpoints still apply when the PLC takes control. */
        if (v.rx_packets == g_activeRx) {
            return;
        }
        g_armed = true;
        g_prev = c;
        g_prev.flags = (uint16_t)(c.flags & ~eipp::kCtlPlcControl); /* taking control is still seen */
    }
    const bool timedOut = v.timeouts != g_seenTimeouts;
    g_seenTimeouts = v.timeouts;

    if (g_inControl && !active) {
        /* Closed, timed out, or Program while the PLC had control: controlled stop, COMM_LOSS (5.2). A jog
         * ends the same way (stop ends it). */
        g_inControl = false;
        g_jogOn = false;
        g_prev = Command();
        g_prevFlags = 0;
        if (!timedOut) {
            g_lossCount++;
        }
        const bool moving = EngineMoving();
        EngineCommLoss();
        TransportUsbSendLine(moving ? "eip: connection lost while the PLC had control: stopping, COMM_LOSS"
                                    : "eip: connection lost while the PLC had control: motors disabled");
        return;
    }

    const uint16_t flags = c.flags;
    const uint16_t rise = (uint16_t)(flags & ~g_prev.flags);
    const uint16_t fall = (uint16_t)(g_prev.flags & ~flags);
    g_prevFlags = flags;
    const bool control = active && (flags & eipp::kCtlPlcControl);
    const bool tookControl = control && !g_inControl;
    if (control != g_inControl) {
        g_inControl = control;
        TransportUsbSendLine(control ? "eip: the PLC has control" : "eip: the PLC released control");
    }

    /* Always, with or without control: stop, abort, dropping ENABLE or JOG. */
    if (rise & eipp::kCtlAbort) {
        Do(EngineAbort, "ABORT");
    }
    if (rise & eipp::kCtlStop) {
        Do(EngineStop, "STOP");
    }
    if (fall & eipp::kCtlJog) {
        /* Always ends a jog; it counts as an accepted action (clearing RejectCode) only if JOG had started one. */
        const char *err = EngineJog(false);
        if (g_jogOn) {
            Result(err, "JOG off");
        }
        g_jogOn = false;
    }
    if (fall & eipp::kCtlEnable) {
        Do(EngineDisable, "ENABLE off");
    }
    if (!control) {
        const uint16_t needControl = eipp::kCtlClearFault | eipp::kCtlSimulate | eipp::kCtlEnable | eipp::kCtlStart |
                                     eipp::kCtlJog | eipp::kCtlTrigger | eipp::kCtlHome;
        const bool setpoint = (c.rpm != g_prev.rpm && c.rpm) || (c.override != g_prev.override && c.override) ||
                              (c.length != g_prev.length && c.length);
        if ((rise & needControl) || setpoint) {
            g_reject = eipp::kRejectNotInControl;
        }
        g_prev = c;
        return;
    }
    if (rise & eipp::kCtlClearFault) {
        Do(EngineClearFault, "CLEAR_FAULT");
    }
    if (rise & eipp::kCtlSimulate) {
        (void)EngineRefusal();
        Result(EngineSimulate(true), "SIMULATE on");
    } else if (fall & eipp::kCtlSimulate) {
        (void)EngineRefusal();
        Result(EngineSimulate(false), "SIMULATE off");
    }
    if (rise & eipp::kCtlEnable) {
        Do(EngineEnable, "ENABLE");
    }
    ApplySetpoints(c, tookControl);
    if (rise & eipp::kCtlStart) {
        Do(EngineStart, "START");
    }
    if (rise & eipp::kCtlTrigger) {
        Do(EngineEipTrigger, "TRIGGER");
    }
    if (rise & eipp::kCtlHome) {
        Do(EngineHomeAll, "HOME");
    }
    if (rise & eipp::kCtlJog) {
        (void)EngineRefusal();
        const char *err = EngineJog(true);
        g_jogOn = err == nullptr;
        Result(err, "JOG");
    } else if (g_jogOn && (flags & eipp::kCtlJog)) {
        ApplyJogTargets(c, false);
    }
    g_prev = c;
}

bool EipLinkPlcInControl() {
    return g_inControl;
}

void EipLinkStatusTokens(char *buf, size_t len) {
    if (!g_started) {
        snprintf(buf, len, " eip=none");
        return;
    }
    EipAdapterView v;
    eip_adapter_snapshot(&v);
    if (!v.io_open) {
        snprintf(buf, len, " eip=none");
        return;
    }
    snprintf(buf, len, " eip=%s eip_from=%u.%u.%u.%u eip_rpi=%lu", g_inControl ? "control" : v.run ? "run" : "idle",
             v.peer_ip[0], v.peer_ip[1], v.peer_ip[2], v.peer_ip[3],
             (unsigned long)(v.rpi_us / 1000u));
}

void EipLinkHostChanged() {
    if (!g_started) {
        return; /* picked up at start */
    }
    EffectiveHost(g_host, sizeof(g_host));
    eip_clearcore_set_host_name(g_host);
}

void EipLinkPoll() {
    if (!g_started) {
        if (TransportEthernetReady()) {
            Start();
        }
        return;
    }
    eip_adapter_poll();
    EipAdapterView view;
    eip_adapter_snapshot(&view);
    UpdateLed(view.io_open && view.run);
    ServiceCommands(view);
    UpdateStatus(view);
    ServiceRestart();
}

static void Ip(char *out, size_t len, const uint8_t a[4]) {
    snprintf(out, len, "%u.%u.%u.%u", a[0], a[1], a[2], a[3]);
}

void EipLinkPrint(void (*out)(const char *line)) {
    char line[200];
    if (!g_started) {
        out("eip not started (the network is not up yet)");
        return;
    }
    EipAdapterView v;
    eip_adapter_snapshot(&v);
    snprintf(line, sizeof(line), "eip identity vendor=%u type=%u product=%u rev=%u.%u serial=%08lx name=\"%s\" host=%s",
             v.vendor_id, v.device_type, v.product_code, v.revision_major, v.revision_minor,
             (unsigned long)v.serial_number, v.name, g_host);
    out(line);
    char ip[16], nm[16], gw[16];
    Ip(ip, sizeof(ip), g_settings.ip);
    Ip(nm, sizeof(nm), g_settings.netmask);
    Ip(gw, sizeof(gw), g_settings.gateway);
    snprintf(line, sizeof(line), "eip network stored=%s%s ip=%s (stored %s/%s gw %s)",
             g_settings.netMode ? "static" : "dhcp", TransportDhcpFellBack() ? " (no DHCP server: on the stored address)" : "",
             TransportIpString(), ip, nm, gw);
    out(line);
    char from[16];
    Ip(from, sizeof(from), v.peer_ip);
    snprintf(line, sizeof(line),
             "eip connection open=%u run=%u from=%s o2t=%u t2o=%u rpi_ms=%lu timeout_ms=%lu multicast=%u tcp_clients=%u",
             v.io_open, v.run, v.io_open ? from : "-", v.o2t_instance, v.t2o_instance, (unsigned long)(v.rpi_us / 1000u),
             (unsigned long)v.timeout_ms, v.multicast, v.tcp_clients);
    out(line);
    snprintf(line, sizeof(line),
             "eip counters rx=%lu tx=%lu opens=%lu timeouts=%lu drop_seq=%lu drop_src=%lu drop_bad=%lu idle=%lu udp_dropped=%lu",
             (unsigned long)v.rx_packets, (unsigned long)v.tx_packets, (unsigned long)v.opens, (unsigned long)v.timeouts,
             (unsigned long)v.rx_dropped_seq, (unsigned long)v.rx_dropped_source, (unsigned long)v.rx_dropped_bad,
             (unsigned long)v.rx_idle, (unsigned long)v.udp_replies_dropped);
    out(line);
    snprintf(line, sizeof(line), "eip control in_control=%u flags=0x%04x reject=%u comm_losses=%lu", g_inControl ? 1u : 0u,
             (unsigned)g_prevFlags, (unsigned)g_reject, (unsigned long)(v.timeouts + g_lossCount));
    out(line);
    if (g_restartPending) {
        snprintf(line, sizeof(line), "eip restart pending: %s (waits until nothing moves)", g_restartWhy);
        out(line);
    }
}
