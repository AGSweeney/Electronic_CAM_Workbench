/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#include "Settings.h"

#include "NvmManager.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static_assert(sizeof(ECamSettings) <= ClearCore::NvmManager::NVM_LOC_RESERVED_TEKNIC,
              "settings must fit the NVM user area");

static const uint32_t ECAM_NVM_MAGIC = 0x4D414345u; /* 'ECAM' */
static const uint16_t ECAM_NVM_VERSION = 5;  /* 2: estop on A-9; 3: host name, DNS, domain; 4: the estop's input; 5: operator inputs */
static const uint32_t CHPGL_NVM_MAGIC = 0x47504843u; /* 'CHPG' (ClearHPGL) */

ECamSettings g_settings;
static bool g_fromNvm = false;
static const char *g_importedFrom = nullptr;
static bool g_estopMoved = false;

static ClearCore::NvmManager &Nvm() {
    return ClearCore::NvmManager::Instance();
}

static uint32_t Fnv(const void *data, size_t len) {
    const uint8_t *p = static_cast<const uint8_t *>(data);
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

static uint32_t Checksum(const ECamSettings *s) {
    return Fnv(s, offsetof(ECamSettings, checksum));
}

/* Versions 1 and 2 (the same layout; version 1 had the estop on DI-6). */
#pragma pack(push, 1)
struct ECamSettingsV2 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint8_t estop;
    uint8_t hlfbCheck;
    uint16_t stopMs;
    uint8_t netMode;
    uint8_t ip[4];
    uint8_t netmask[4];
    uint8_t gateway[4];
    uint32_t checksum;
};
#pragma pack(pop)

/* Version 4: version 5 without the operator inputs. */
#pragma pack(push, 1)
struct ECamSettingsV4 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint8_t estop;
    uint8_t hlfbCheck;
    uint16_t stopMs;
    uint8_t netMode;
    uint8_t ip[4];
    uint8_t netmask[4];
    uint8_t gateway[4];
    uint8_t dns1[4];
    uint8_t dns2[4];
    char domain[49];
    char host[65];
    uint8_t estopPin;
    uint32_t checksum;
};
#pragma pack(pop)

/* Version 3: version 4 without the estop's input (A-9). */
#pragma pack(push, 1)
struct ECamSettingsV3 {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint8_t estop;
    uint8_t hlfbCheck;
    uint16_t stopMs;
    uint8_t netMode;
    uint8_t ip[4];
    uint8_t netmask[4];
    uint8_t gateway[4];
    uint8_t dns1[4];
    uint8_t dns2[4];
    char domain[49];
    char host[65];
    uint32_t checksum;
};
#pragma pack(pop)

static const char *const kInputNames[] = {"IO-0", "IO-1", "IO-2", "IO-3", "IO-4", "IO-5", "DI-6",
                                          "DI-7", "DI-8", "A-9",  "A-10", "A-11", "A-12"};
static const int kInputCount = (int)(sizeof(kInputNames) / sizeof(kInputNames[0]));

const char *SettingsInputName(int pin) {
    return pin >= 0 && pin < kInputCount ? kInputNames[pin] : "-";
}

int SettingsParseInput(const char *s) {
    /* A name with or without its dash, any case; or the index. */
    char norm[8] = {};
    size_t n = 0;
    for (const char *p = s; *p && n < sizeof(norm) - 1; ++p) {
        if (*p == '-') continue;
        norm[n++] = (char)(*p >= 'a' && *p <= 'z' ? *p - 32 : *p);
    }
    for (int i = 0; i < kInputCount; ++i) {
        char want[8] = {};
        size_t k = 0;
        for (const char *p = kInputNames[i]; *p; ++p) {
            if (*p != '-') want[k++] = *p;
        }
        if (strcmp(norm, want) == 0) return i;
    }
    unsigned long v;
    char *end = nullptr;
    v = strtoul(s, &end, 10);
    if (end != s && *end == '\0' && v < (unsigned long)kInputCount) return (int)v;
    return -1;
}

void SettingsNetworkDefaults() {
    g_settings.netMode = 0;
    const uint8_t ip[4] = {192, 168, 0, 110}, nm[4] = {255, 255, 255, 0}, gw[4] = {192, 168, 0, 1};
    memcpy(g_settings.ip, ip, 4);
    memcpy(g_settings.netmask, nm, 4);
    memcpy(g_settings.gateway, gw, 4);
    memset(g_settings.dns1, 0, 4);
    memset(g_settings.dns2, 0, 4);
    memset(g_settings.domain, 0, sizeof(g_settings.domain));
    memset(g_settings.host, 0, sizeof(g_settings.host));
    g_settings.checksum = Checksum(&g_settings);
}

void SettingsDefaults() {
    memset(&g_settings, 0, sizeof(g_settings));
    g_settings.magic = ECAM_NVM_MAGIC;
    g_settings.version = ECAM_NVM_VERSION;
    g_settings.size = sizeof(ECamSettings);
    g_settings.estopMode = 0;  /* off until an estop is wired: set estop_pin, then estop_mode */
    g_settings.estopPin = 9;   /* A-9 */
    g_settings.hlfbCheck = 1;
    g_settings.stopMs = 500;
    /* No operator inputs; Disable and Stop normally closed (a broken wire disables / stops) when one is set. */
    memset(g_settings.opPin, 0xFF, sizeof(g_settings.opPin));
    g_settings.opNc = (1u << 1) | (1u << 4);
    g_settings.opMode = 0;
    SettingsNetworkDefaults();
}

/* ClearHPGL (PlotterSettings, versions 1..3 share this prefix): estopDi6 at 77, hlfbCheck at 79,
 * netMode at 87, ip at 88, netmask at 92, gateway at 96. */
static bool ImportClearHpgl(const uint8_t *blob, size_t len) {
    uint32_t magic;
    uint16_t version;
    memcpy(&magic, blob, 4);
    memcpy(&version, blob + 4, 2);
    if (magic != CHPGL_NVM_MAGIC || version < 1 || version > 3 || len < 100) {
        return false;
    }
    const uint8_t estop = blob[77], hlfb = blob[79], mode = blob[87];
    if (estop > 2 || hlfb > 1 || mode > 1) {
        return false;
    }
    /* ClearHPGL's estop is on DI-6, which is the Encoder Adapter here: not imported (stays off). */
    g_settings.hlfbCheck = hlfb;
    g_settings.netMode = mode;
    memcpy(g_settings.ip, blob + 88, 4);
    memcpy(g_settings.netmask, blob + 92, 4);
    memcpy(g_settings.gateway, blob + 96, 4);
    g_settings.checksum = Checksum(&g_settings);
    g_importedFrom = "ClearHPGL";
    return true;
}

void SettingsLoad() {
    uint8_t raw[ClearCore::NvmManager::NVM_LOC_RESERVED_TEKNIC];
    Nvm().BlockRead(ClearCore::NvmManager::NVM_LOC_USER_START, (int)sizeof(raw), raw);
    ECamSettings s;
    memcpy(&s, raw, sizeof(s));
    ECamSettingsV2 old;
    memcpy(&old, raw, sizeof(old));
    if (old.magic == ECAM_NVM_MAGIC && (old.version == 1 || old.version == 2) && old.size == sizeof(ECamSettingsV2) &&
        old.checksum == Fnv(&old, offsetof(ECamSettingsV2, checksum))) {
        /* Versions 1 and 2: keep everything; the new fields start empty. Version 1 had the estop on
         * DI-6, which comes up off. Written back as version 3 at the next save. */
        SettingsDefaults();
        g_settings.estopMode = old.version == 1 ? 0 : old.estop;
        g_settings.hlfbCheck = old.hlfbCheck;
        g_settings.stopMs = old.stopMs;
        g_settings.netMode = old.netMode;
        memcpy(g_settings.ip, old.ip, 4);
        memcpy(g_settings.netmask, old.netmask, 4);
        memcpy(g_settings.gateway, old.gateway, 4);
        g_settings.checksum = Checksum(&g_settings);
        g_estopMoved = old.version == 1;
        g_fromNvm = true;
        return;
    }
    ECamSettingsV4 v4;
    memcpy(&v4, raw, sizeof(v4));
    if (v4.magic == ECAM_NVM_MAGIC && v4.version == 4 && v4.size == sizeof(ECamSettingsV4) &&
        v4.checksum == Fnv(&v4, offsetof(ECamSettingsV4, checksum))) {
        /* Version 4: everything kept, no operator inputs. Written back as version 5 at the next save. */
        SettingsDefaults();
        memcpy(&g_settings.estopMode, &v4.estop, offsetof(ECamSettingsV4, checksum) - offsetof(ECamSettingsV4, estop));
        g_settings.checksum = Checksum(&g_settings);
        g_fromNvm = true;
        return;
    }
    ECamSettingsV3 v3;
    memcpy(&v3, raw, sizeof(v3));
    if (v3.magic == ECAM_NVM_MAGIC && v3.version == 3 && v3.size == sizeof(ECamSettingsV3) &&
        v3.checksum == Fnv(&v3, offsetof(ECamSettingsV3, checksum))) {
        /* Version 3: everything kept, the estop on A-9 as it was. Written back as version 4 at the next save. */
        SettingsDefaults();
        memcpy(&g_settings.estopMode, &v3.estop, offsetof(ECamSettingsV3, checksum) - offsetof(ECamSettingsV3, estop));
        g_settings.estopPin = 9;
        g_settings.checksum = Checksum(&g_settings);
        g_fromNvm = true;
        return;
    }
    if (s.magic != ECAM_NVM_MAGIC || s.version != ECAM_NVM_VERSION || s.size != sizeof(ECamSettings) ||
        s.checksum != Checksum(&s)) {
        SettingsDefaults();
        g_fromNvm = false;
        /* Not written back: the other firmware's blob stays until the first "set". */
        ImportClearHpgl(raw, sizeof(raw));
        return;
    }
    g_settings = s;
    if (g_settings.estopPin >= kInputCount) g_settings.estopPin = 9;
    for (uint8_t &p : g_settings.opPin) {
        if (p >= kInputCount) p = 0xFF;
    }
    if (g_settings.opMode > 1) g_settings.opMode = 0;
    g_fromNvm = true;
}

bool SettingsSave() {
    g_settings.magic = ECAM_NVM_MAGIC;
    g_settings.version = ECAM_NVM_VERSION;
    g_settings.size = sizeof(ECamSettings);
    g_settings.checksum = Checksum(&g_settings);
    ECamSettings existing;
    Nvm().BlockRead(ClearCore::NvmManager::NVM_LOC_USER_START, (int)sizeof(existing), (uint8_t *)&existing);
    /* BlockWrite reports "unchanged" and "write failed" the same way: an unchanged cache only
     * counts once the page write has finished (see ClearHPGL Settings.cpp). */
    if (memcmp(&existing, &g_settings, sizeof(existing)) == 0) {
        if (Nvm().Synchonized()) {
            g_fromNvm = true;
            return true;
        }
        ECamSettings nudge;
        memset(&nudge, 0, sizeof(nudge));
        nudge.magic = 1;
        if (!Nvm().BlockWrite(ClearCore::NvmManager::NVM_LOC_USER_START, (int)sizeof(nudge),
                              (const uint8_t *)&nudge)) {
            return false;
        }
    }
    if (!Nvm().BlockWrite(ClearCore::NvmManager::NVM_LOC_USER_START, (int)sizeof(g_settings),
                          (const uint8_t *)&g_settings)) {
        return false;
    }
    g_fromNvm = Nvm().Synchonized();
    return g_fromNvm;
}

bool SettingsFromNvm() {
    return g_fromNvm;
}

const char *SettingsImportedFrom() {
    return g_importedFrom;
}

bool SettingsEstopMoved() {
    return g_estopMoved;
}

static bool ParseIp(const char *s, uint8_t out[4]) {
    unsigned a, b, c, d;
    char tail;
    if (sscanf(s, "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) != 4 || a > 255 || b > 255 || c > 255 || d > 255) {
        return false;
    }
    out[0] = (uint8_t)a;
    out[1] = (uint8_t)b;
    out[2] = (uint8_t)c;
    out[3] = (uint8_t)d;
    return true;
}

static bool ParseUint(const char *s, unsigned long lo, unsigned long hi, unsigned long *out) {
    char *end = nullptr;
    const unsigned long v = strtoul(s, &end, 10);
    if (end == s || *end != '\0' || v < lo || v > hi) {
        return false;
    }
    *out = v;
    return true;
}

const char *SettingsSet(const char *key, const char *value, bool *needsRestart) {
    *needsRestart = false;
    unsigned long v = 0;
    if (strcmp(key, "estop_di6") == 0) {
        return "the estop moved: set estop_pin and estop_mode";
    }
    if (strcmp(key, "estop_a9") == 0) {
        /* Before 0.8.9: the estop on A-9. */
        if (!ParseUint(value, 0, 2, &v)) return "estop_a9: 0 off, 1 fault when A-9 is low, 2 fault when high";
        g_settings.estopMode = (uint8_t)v;
        g_settings.estopPin = 9;
        g_estopMoved = false;
    } else if (strcmp(key, "estop_mode") == 0) {
        if (!ParseUint(value, 0, 2, &v)) return "estop_mode: 0 off, 1 fault when the input is low, 2 fault when high";
        g_settings.estopMode = (uint8_t)v;
        g_estopMoved = false;
    } else if (strcmp(key, "estop_pin") == 0) {
        const int pin = SettingsParseInput(value);
        if (pin < 0) return "estop_pin: IO-0..IO-5, DI-6..DI-8, A-9..A-12";
        g_settings.estopPin = (uint8_t)pin;
    } else if (SettingsOpIndex(key) >= 0) {
        /* op_enable IO-3 | op_enable off (the caller has checked the pin against the estop, the others and the profile) */
        const int k = SettingsOpIndex(key);
        if (strcmp(value, "off") == 0) {
            g_settings.opPin[k] = 0xFF;
        } else {
            const int pin = SettingsParseInput(value);
            if (pin < 0) return "an input, IO-0..IO-5, DI-6..DI-8, A-9..A-12, or off";
            g_settings.opPin[k] = (uint8_t)pin;
        }
    } else if (strncmp(key, "op_", 3) == 0 && strlen(key) > 11 && strcmp(key + strlen(key) - 8, "_contact") == 0) {
        /* op_enable_contact nc|no */
        char name[24];
        const size_t n = strlen(key) - 8;
        if (n >= sizeof(name)) return "unknown key (see config)";
        memcpy(name, key, n);
        name[n] = '\0';
        const int k = SettingsOpIndex(name);
        if (k < 0) return "unknown key (see config)";
        if (strcmp(value, "nc") != 0 && strcmp(value, "no") != 0) return "nc or no";
        g_settings.opNc = (uint8_t)(strcmp(value, "nc") == 0 ? (g_settings.opNc | (1u << k)) : (g_settings.opNc & ~(1u << k)));
    } else if (strcmp(key, "op_enable_mode") == 0) {
        if (strcmp(value, "momentary") == 0) g_settings.opMode = 0;
        else if (strcmp(value, "maintained") == 0) g_settings.opMode = 1;
        else return "op_enable_mode: momentary (Enable and Disable buttons) or maintained (an Enable switch)";
    } else if (strcmp(key, "hlfb_check") == 0) {
        if (!ParseUint(value, 0, 1, &v)) return "hlfb_check: 0 or 1";
        g_settings.hlfbCheck = (uint8_t)v;
    } else if (strcmp(key, "stop_ms") == 0) {
        if (!ParseUint(value, 10, 60000, &v)) return "stop_ms: 10..60000";
        g_settings.stopMs = (uint16_t)v;
    } else if (strcmp(key, "net_mode") == 0) {
        if (!ParseUint(value, 0, 1, &v)) return "net_mode: 0 DHCP, 1 static";
        g_settings.netMode = (uint8_t)v;
        *needsRestart = true;
    } else if (strcmp(key, "ip") == 0) {
        if (!ParseIp(value, g_settings.ip)) return "ip: a.b.c.d";
        *needsRestart = true;
    } else if (strcmp(key, "netmask") == 0) {
        if (!ParseIp(value, g_settings.netmask)) return "netmask: a.b.c.d";
        *needsRestart = true;
    } else if (strcmp(key, "gateway") == 0) {
        if (!ParseIp(value, g_settings.gateway)) return "gateway: a.b.c.d";
        *needsRestart = true;
    } else if (strcmp(key, "host") == 0) {
        /* "-" clears it (back to the default name). Letters, digits, '-' and '.', as EtherNet/IP allows. */
        const bool clear = strcmp(value, "-") == 0;
        if (!clear) {
            const size_t n = strlen(value);
            if (n == 0 || n >= sizeof(g_settings.host)) return "host: 1..64 characters (- for the default)";
            for (size_t i = 0; i < n; i++) {
                const char c = value[i];
                if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '.')) {
                    return "host: letters, digits, '-' and '.' only";
                }
            }
        }
        memset(g_settings.host, 0, sizeof(g_settings.host));
        if (!clear) strncpy(g_settings.host, value, sizeof(g_settings.host) - 1);
    } else {
        return "unknown key (see config)";
    }
    if (!SettingsSave()) {
        return "applied, but saving to NVM failed (supply voltage?)";
    }
    return nullptr;
}

const char *SettingsOpName(int input) {
    static const char *const kNames[] = {"op_enable", "op_disable", "op_reset", "op_start", "op_stop"};
    return input >= 0 && input < 5 ? kNames[input] : "?";
}

int SettingsOpIndex(const char *key) {
    for (int i = 0; i < 5; ++i) {
        if (strcmp(key, SettingsOpName(i)) == 0) return i;
    }
    return -1;
}

void SettingsPrint(void (*out)(const char *line)) {
    char buf[96];
    snprintf(buf, sizeof(buf), "estop_mode %u", g_settings.estopMode);
    out(buf);
    snprintf(buf, sizeof(buf), "estop_pin %s", SettingsInputName(g_settings.estopPin));
    out(buf);
    for (int i = 0; i < 5; ++i) {
        snprintf(buf, sizeof(buf), "%s %s %s", SettingsOpName(i), g_settings.opPin[i] == 0xFF ? "off" : SettingsInputName(g_settings.opPin[i]),
                 (g_settings.opNc >> i) & 1u ? "nc" : "no");
        out(buf);
    }
    snprintf(buf, sizeof(buf), "op_enable_mode %s", g_settings.opMode ? "maintained" : "momentary");
    out(buf);
    snprintf(buf, sizeof(buf), "hlfb_check %u", g_settings.hlfbCheck);
    out(buf);
    snprintf(buf, sizeof(buf), "stop_ms %u", g_settings.stopMs);
    out(buf);
    snprintf(buf, sizeof(buf), "net_mode %u", g_settings.netMode);
    out(buf);
    const uint8_t *a = g_settings.ip, *n = g_settings.netmask, *g = g_settings.gateway;
    snprintf(buf, sizeof(buf), "ip %u.%u.%u.%u", a[0], a[1], a[2], a[3]);
    out(buf);
    snprintf(buf, sizeof(buf), "netmask %u.%u.%u.%u", n[0], n[1], n[2], n[3]);
    out(buf);
    snprintf(buf, sizeof(buf), "gateway %u.%u.%u.%u", g[0], g[1], g[2], g[3]);
    out(buf);
    snprintf(buf, sizeof(buf), "host %s", g_settings.host[0] ? g_settings.host : "- (default)");
    out(buf);
    if (g_settings.dns1[0] || g_settings.dns2[0] || g_settings.domain[0]) {
        const uint8_t *d1 = g_settings.dns1, *d2 = g_settings.dns2;
        snprintf(buf, sizeof(buf), "dns %u.%u.%u.%u %u.%u.%u.%u domain %s (set over EtherNet/IP)", d1[0], d1[1], d1[2],
                 d1[3], d2[0], d2[1], d2[2], d2[3], g_settings.domain[0] ? g_settings.domain : "-");
        out(buf);
    }
}
