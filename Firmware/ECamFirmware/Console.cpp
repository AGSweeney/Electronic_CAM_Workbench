/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#include "Console.h"

#include "CamEngine.h"
#include "ECamConfig.h"
#include "EipLink.h"
#include "Settings.h"
#include "Transport.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *const kHelp[] = {
    "  status               state, master position, axis positions (motor counts)",
    "  ccio                 CCIO-8 boards on COM-1, the link, the inputs (bit 0 = first board's first input)",
    "  load                 begin a profile upload: master/timeline/axis/seg lines, then commit",
    "  commit               check and load the uploaded profile (disables the motors)",
    "  profile              the loaded profile",
    "  download             the loaded profile's upload lines (to read it back)",
    "  forget               clear the profile kept in flash (it loads at boot otherwise)",
    "  enable | disable     the profile's motors",
    "  start                engage: crank axes follow (driven crank ramps up); timeline arms",
    "  trigger              start the timeline now (instead of its input)",
    "  stop                 ramp to rest (an encoder master lets go at once)",
    "  abort                stop at once",
    "  override <percent>   timeline speed",
    "  rpm <value>          driven crank speed",
    "  length <mm>          finished piece length from the next cut on (cut-length mode; plus the kerf)",
    "  jog on|off           hand mode: the master goes where 'jog crank'/'jog time' say",
    "  jog crank <deg>      crank target, absolute degrees since the load (rev * 360 + angle)",
    "  jog time <s>         timeline target in seconds",
    "  axjog                each motor M-0..M-3: role, on, moving, position, cam position, counts/unit, limit",
    "  axjog <m> enable|disable  one motor (not while the cam runs; disable always). <m>: 0..3 or M-0..M-3",
    "  axjog <m> jog <u/s>  velocity jog; send again within 0.5 s or it ramps to rest (0 stops)",
    "  axjog <m> jog <u/s> stopon all|IO-3,CCIOA2   ... and ramps to rest at the first of those inputs to change",
    "  axjog <m> move <u> <u/s>  relative move    axjog <m> goto <u> <u/s>  to a position",
    "  axjog <m> cam <u/s>  a profile axis: to where the cam has it at the next start",
    "  axjog <m> zero       its position here is 0   axjog <m|all> stop     (u: the axis's units, else counts)",
    "  axhome <m>|all       home one motor's axis, or every axis with a home switch in order   axhome stop",
    "  axhome               each axis's homing: switch, homed, homing, ends, latch, last result",
    "  axlimit              each axis: its limits now, the axis jog stops at them (count, where it tripped)",
    "  waits                the timeline's waits: input, level, timeout, met now, reached, timed out; the PLC bits",
    "  wait_skip            simulate only: pass the wait holding the timeline",
    "  op_test enable|disable|reset|start|stop press|release|off   bench: an operator input forced (RAM only)",
    "  waitbits_test <hex>|off   bench: these PLC wait bits instead of the PLC's, as if connected (RAM only)",
    "  limit_test <axis> +|-|off   home_test <axis> <u>|off   (bench: a limit tripped; a home switch at a position)",
    "  sim 0|1              simulate: run the profile without stepping the motors",
    "  simenc <counts/s>    simulate only: a simulated encoder moving at this rate (0 holds it)",
    "  clear                clear a latched fault",
    "  tel <ms>             push 'TEL ...' lines every <ms> on this link (0 = off)",
    "  config               settings      set <key> <value>   change and save one",
    "  diag                 interrupt timing",
    "  motors               each motor connector: enable, HLFB, ready state, status / alert registers",
    "  eip                  EtherNet/IP adapter: identity, connection, counters",
    "  restart | version | exit",
};

static void Reply(ConsoleOut out, const char *err) {
    if (err) {
        char buf[ECAM_MAX_LINE];
        snprintf(buf, sizeof(buf), "error: %s", err);
        out(buf);
    } else {
        out("ok");
    }
}

void ConsoleBanner(ConsoleOut out) {
    char buf[ECAM_MAX_LINE];
    snprintf(buf, sizeof(buf), "=== %s %s (electronic cam) ===", ECAM_FIRMWARE_NAME, ECAM_FIRMWARE_VERSION);
    out(buf);
    snprintf(buf, sizeof(buf), "IP %s   console on USB and TCP %u   settings: %s", TransportIpString(),
             (unsigned)ECAM_TCP_CONSOLE_PORT,
             SettingsFromNvm()          ? "NVM"
             : SettingsImportedFrom()   ? "defaults, network from ClearHPGL"
                                        : "defaults");
    out(buf);
    if (g_settings.estopMode == 0) {
        out(SettingsEstopMoved() ? "estop OFF: it moved from DI-6 to A-9 (the Encoder Adapter uses DI-6..DI-8); "
                                   "wire it, then set estop_pin and estop_mode"
                                 : "estop off (set estop_pin and estop_mode 1 or 2 with an estop wired)");
    }
}

static uint8_t Tokenize(char *line, char *tok[], uint8_t max) {
    uint8_t n = 0;
    char *p = line;
    while (*p && n < max) {
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        if (!*p) {
            break;
        }
        tok[n++] = p;
        while (*p && *p != ' ' && *p != '\t') {
            p++;
        }
        if (*p) {
            *p++ = '\0';
        }
    }
    return n;
}

static bool Eq(const char *a, const char *b) {
    return strcmp(a, b) == 0;
}

static bool ParseNumber(const char *s, double *out) {
    char *end = nullptr;
    const double v = strtod(s, &end);
    if (end == s || *end != '\0') {
        return false;
    }
    *out = v;
    return true;
}

static bool IsProfileLine(const char *word) {
    return Eq(word, "master") || Eq(word, "timeline") || Eq(word, "axis") || Eq(word, "seg") || Eq(word, "out") ||
           Eq(word, "ccio_boards") || Eq(word, "input") || Eq(word, "limits") || Eq(word, "home") || Eq(word, "wait") ||
           Eq(word, "json");
}

bool ConsoleHandleLine(const char *raw, ConsoleOut out) {
    char line[ECAM_MAX_LINE];
    strncpy(line, raw, sizeof(line));
    line[sizeof(line) - 1] = '\0';
    /* Trim trailing spaces (the hash is over the exact text the host hashed). */
    for (size_t n = strlen(line); n > 0 && (line[n - 1] == ' ' || line[n - 1] == '\t'); --n) {
        line[n - 1] = '\0';
    }
    const char *start = line;
    while (*start == ' ' || *start == '\t') {
        start++;
    }
    if (!*start) {
        return false;
    }

    /* Profile lines go to the reader untouched. */
    char first[16] = {};
    sscanf(start, "%15s", first);
    if (IsProfileLine(first)) {
        if (!EngineLoading()) {
            Reply(out, "send 'load' first");
        } else {
            Reply(out, EngineLoadLine(start));
        }
        return false;
    }

    char work[ECAM_MAX_LINE];
    strncpy(work, start, sizeof(work));
    work[sizeof(work) - 1] = '\0';
    char *tok[6];
    const uint8_t n = Tokenize(work, tok, 6);
    const char *cmd = tok[0];
    char buf[512];  /* status: up to 4 axes, the cut-length fields, axis jog */

    /* While the PLC has control the console may stop, abort and disable, not change the motion. */
    static const char *const kPlcOwned[] = {"enable", "start", "trigger", "jog",  "rpm",    "override",
                                            "length", "sim",   "simenc",  "load", "commit", "forget", "wait_skip"};
    if (EipLinkPlcInControl()) {
        /* Axis jog too, but its disable and stop stay the console's. */
        if ((Eq(cmd, "axjog") && n >= 3 && !Eq(tok[2], "disable") && !Eq(tok[2], "stop")) ||
            (Eq(cmd, "axhome") && n >= 2 && !Eq(tok[1], "stop"))) {
            Reply(out, "the PLC has control");
            return false;
        }
        for (const char *k : kPlcOwned) {
            if (Eq(cmd, k)) {
                Reply(out, "the PLC has control");
                return false;
            }
        }
    }

    if (Eq(cmd, "help") || Eq(cmd, "?")) {
        for (const char *h : kHelp) {
            out(h);
        }
        Reply(out, nullptr);
    } else if (Eq(cmd, "version")) {
        out(ECAM_FIRMWARE_NAME " " ECAM_FIRMWARE_VERSION);
        Reply(out, nullptr);
    } else if (Eq(cmd, "status")) {
        EngineStatus(buf, sizeof(buf));
        const size_t used = strlen(buf);
        EipLinkStatusTokens(buf + used, sizeof(buf) - used);
        out(buf);
        Reply(out, nullptr);
    } else if (Eq(cmd, "load")) {
        Reply(out, EngineLoadBegin());
    } else if (Eq(cmd, "commit")) {
        char info[64];
        const char *err = EngineCommit(info, sizeof(info));
        if (err) {
            Reply(out, err);
        } else {
            snprintf(buf, sizeof(buf), "ok %s", info);
            out(buf);
        }
    } else if (Eq(cmd, "download")) {
        Reply(out, EngineDownload(out));
    } else if (Eq(cmd, "forget")) {
        Reply(out, EngineForget());
    } else if (Eq(cmd, "profile")) {
        EngineProfile(out);
        Reply(out, nullptr);
    } else if (Eq(cmd, "enable")) {
        Reply(out, EngineEnable());
    } else if (Eq(cmd, "disable")) {
        Reply(out, EngineDisable());
    } else if (Eq(cmd, "start")) {
        Reply(out, EngineStart());
    } else if (Eq(cmd, "trigger")) {
        Reply(out, EngineTrigger());
    } else if (Eq(cmd, "stop")) {
        Reply(out, EngineStop());
    } else if (Eq(cmd, "abort")) {
        Reply(out, EngineAbort());
    } else if (Eq(cmd, "clear")) {
        Reply(out, EngineClearFault());
    } else if (Eq(cmd, "length")) {
        double v;
        if (n < 2 || !ParseNumber(tok[1], &v)) {
            Reply(out, "length <mm>");
        } else {
            Reply(out, EngineLength(v));
        }
    } else if (Eq(cmd, "override") || Eq(cmd, "rpm")) {
        double v;
        if (n < 2 || !ParseNumber(tok[1], &v)) {
            Reply(out, Eq(cmd, "rpm") ? "rpm <value>" : "override <percent>");
        } else {
            Reply(out, Eq(cmd, "rpm") ? EngineRpm(v) : EngineOverride(v / 100.0));
        }
    } else if (Eq(cmd, "jog")) {
        double v;
        if (n >= 2 && Eq(tok[1], "on")) {
            Reply(out, EngineJog(true));
        } else if (n >= 2 && Eq(tok[1], "off")) {
            Reply(out, EngineJog(false));
        } else if (n >= 3 && Eq(tok[1], "crank") && ParseNumber(tok[2], &v)) {
            Reply(out, EngineJogCrank(v));
        } else if (n >= 3 && Eq(tok[1], "time") && ParseNumber(tok[2], &v)) {
            Reply(out, EngineJogTime(v));
        } else {
            Reply(out, "jog on | jog off | jog crank <deg> | jog time <s>");
        }
    } else if (Eq(cmd, "axjog")) {
        double v = 0.0, w = 0.0;
        const bool all = n >= 2 && Eq(tok[1], "all");
        /* The motor: 0..3, or M-0..M-3 as on the board. */
        const char *m = n >= 2 ? tok[1] : "";
        if ((m[0] == 'M' || m[0] == 'm') && m[1] == '-') {
            m += 2;
        }
        char *end = nullptr;
        const long i = n >= 2 ? strtol(m, &end, 10) : -1;
        const bool idx = n >= 2 && end != m && *end == '\0';
        const char *verb = n >= 3 ? tok[2] : "";
        if (n == 1) {
            EngineJogList(out);
            Reply(out, nullptr);
        } else if (all && Eq(verb, "stop")) {
            Reply(out, EngineJogStop(-1));
        } else if (!idx) {
            Reply(out, "axjog | axjog <m> enable|disable|jog|move|goto|cam|zero|stop (help)");
        } else if (Eq(verb, "enable") || Eq(verb, "disable")) {
            Reply(out, EngineJogEnable((int)i, Eq(verb, "enable")));
        } else if (Eq(verb, "jog") && n >= 6 && ParseNumber(tok[3], &v) && Eq(tok[4], "stopon")) {
            Reply(out, EngineJogStopOn((int)i, v, tok[5]));
        } else if (Eq(verb, "jog") && n >= 4 && ParseNumber(tok[3], &v)) {
            Reply(out, EngineJogVelocity((int)i, v));
        } else if ((Eq(verb, "move") || Eq(verb, "goto")) && n >= 5 && ParseNumber(tok[3], &v) && ParseNumber(tok[4], &w)) {
            Reply(out, EngineJogMove((int)i, v, Eq(verb, "goto"), w));
        } else if (Eq(verb, "cam") && n >= 4 && ParseNumber(tok[3], &w)) {
            Reply(out, EngineJogToCam((int)i, w));
        } else if (Eq(verb, "zero")) {
            Reply(out, EngineJogZero((int)i));
        } else if (Eq(verb, "stop")) {
            Reply(out, EngineJogStop((int)i));
        } else {
            Reply(out, "axjog <m> enable | disable | jog <u/s> | move <u> <u/s> | goto <u> <u/s> | cam <u/s> | zero | stop");
        }
    } else if (Eq(cmd, "sim")) {
        if (n < 2 || (!Eq(tok[1], "0") && !Eq(tok[1], "1"))) {
            Reply(out, "sim 0|1");
        } else {
            Reply(out, EngineSimulate(Eq(tok[1], "1")));
        }
    } else if (Eq(cmd, "simenc")) {
        double v;
        if (n < 2 || !ParseNumber(tok[1], &v)) {
            Reply(out, "simenc <counts/s>");
        } else {
            Reply(out, EngineSimEncoder(v));
        }
    } else if (Eq(cmd, "tel")) {
        double v;
        if (n < 2 || !ParseNumber(tok[1], &v) || v < 0) {
            Reply(out, "tel <ms>");
        } else {
            Reply(out, TransportSetTelemetry(out, (uint32_t)v));
        }
    } else if (Eq(cmd, "config")) {
        SettingsPrint(out);
        Reply(out, nullptr);
    } else if (Eq(cmd, "set")) {
        if (n < 3) {
            Reply(out, "set <key> <value>");
        } else if (EngineBusy()) {
            Reply(out, "disable the motors and stop the motion first");
        } else if (const char *clash = SettingsOpIndex(tok[1]) >= 0 && !Eq(tok[2], "off")
                                           ? (SettingsParseInput(tok[2]) < 0 ? nullptr : EngineOpPinCheck(SettingsOpIndex(tok[1]), SettingsParseInput(tok[2])))
                                       : Eq(tok[1], "estop_pin")   ? EngineEstopCheck(SettingsParseInput(tok[2]), g_settings.estopMode)
                                       : Eq(tok[1], "estop_mode") ? EngineEstopCheck(g_settings.estopPin, atoi(tok[2]))
                                       : Eq(tok[1], "estop_a9")   ? EngineEstopCheck(9, atoi(tok[2]))
                                                                   : nullptr) {
            Reply(out, clash);
        } else {
            bool restart = false;
            const char *err = SettingsSet(tok[1], tok[2], &restart);
            EngineApplySettings();
            if (!err && Eq(tok[1], "host")) {
                EipLinkHostChanged();
            }
            if (!err && restart) {
                out("saved; applies after restart");
            }
            Reply(out, err);
        }
    } else if (Eq(cmd, "op_test")) {
        static const char *const kNames[] = {"enable", "disable", "reset", "start", "stop"};
        int input = -1;
        for (int k = 0; k < 5 && n >= 2; ++k) {
            if (Eq(tok[1], kNames[k])) input = k;
        }
        const int state = n >= 3 ? (Eq(tok[2], "press") ? 1 : Eq(tok[2], "release") ? 0 : Eq(tok[2], "off") ? -1 : 9) : 9;
        Reply(out, state == 9 ? "op_test enable|disable|reset|start|stop press|release|off" : EngineOpTest(input, state));
    } else if (Eq(cmd, "waits")) {
        EngineWaitList(out);
        Reply(out, nullptr);
    } else if (Eq(cmd, "wait_skip")) {
        Reply(out, EngineWaitSkip());
    } else if (Eq(cmd, "waitbits_test")) {
        char *end = nullptr;
        const unsigned long bits = n >= 2 ? strtoul(tok[1], &end, 16) : 0;
        if (n >= 2 && Eq(tok[1], "off")) {
            EngineWaitBitsTest(false, 0);
            Reply(out, nullptr);
        } else if (n >= 2 && end != tok[1] && *end == '\0') {
            EngineWaitBitsTest(true, (uint32_t)bits);
            Reply(out, nullptr);
        } else {
            Reply(out, "waitbits_test <hex>|off");
        }
    } else if (Eq(cmd, "axhome") && n == 1) {
        EngineHomeList(out);
        Reply(out, nullptr);
    } else if (Eq(cmd, "axlimit")) {
        EngineLimitList(out);
        Reply(out, nullptr);
    } else if (Eq(cmd, "axhome")) {
        const char *m = n >= 2 ? tok[1] : "";
        if ((m[0] == 'M' || m[0] == 'm') && m[1] == '-') {
            m += 2;
        }
        char *end = nullptr;
        const long i = n >= 2 ? strtol(m, &end, 10) : -1;
        if (n >= 2 && Eq(tok[1], "all")) {
            Reply(out, EngineHomeAll());
        } else if (n >= 2 && Eq(tok[1], "stop")) {
            Reply(out, EngineHomeStop());
        } else if (n >= 2 && end != m && *end == '\0') {
            Reply(out, EngineHome((int)i));
        } else {
            Reply(out, "axhome <m> | axhome all | axhome stop");
        }
    } else if (Eq(cmd, "home_test") && n == 1) {
        EngineHomeTestList(out);
        Reply(out, nullptr);
    } else if (Eq(cmd, "home_test")) {
        char *end = nullptr;
        const long ax = n >= 3 ? strtol(tok[1], &end, 10) : -1;
        double v = 0.0;
        const bool off = n >= 3 && Eq(tok[2], "off");
        if (n < 3 || end == tok[1] || (!off && !ParseNumber(tok[2], &v))) {
            Reply(out, "home_test <axis> <position>|off");
        } else {
            Reply(out, EngineHomeTest((int)ax, !off, v));
        }
    } else if (Eq(cmd, "limit_test")) {
        char *end = nullptr;
        const long ax = n >= 3 ? strtol(tok[1], &end, 10) : -1;
        const int side = n >= 3 ? (Eq(tok[2], "+") ? 1 : Eq(tok[2], "-") ? -1 : Eq(tok[2], "off") ? 0 : 9) : 9;
        if (n < 3 || end == tok[1] || side == 9) {
            Reply(out, "limit_test <axis> +|-|off");
        } else {
            Reply(out, EngineLimitTest((int)ax, side));
        }
    } else if (Eq(cmd, "estop_test")) {
        if (n < 2 || (!Eq(tok[1], "0") && !Eq(tok[1], "1"))) {
            Reply(out, "estop_test 0|1");
        } else {
            EngineEstopTest(Eq(tok[1], "1"));
            Reply(out, nullptr);
        }
    } else if (Eq(cmd, "diag")) {
        EngineDiag(buf, sizeof(buf));
        out(buf);
        TransportStats(buf, sizeof(buf));
        out(buf);
        TransportLwipStats(buf, sizeof(buf));
        out(buf);
        TransportLoopStats(buf, sizeof(buf));
        out(buf);
        Reply(out, nullptr);
    } else if (Eq(cmd, "motors")) {
        EngineMotors(out);
        Reply(out, nullptr);
    } else if (Eq(cmd, "ccio")) {
        EngineCcio(out);
        Reply(out, nullptr);
    } else if (Eq(cmd, "eip")) {
        EipLinkPrint(out);
        Reply(out, nullptr);
    } else if (Eq(cmd, "restart")) {
        Reply(out, nullptr);
        TransportRestart();
    } else if (Eq(cmd, "exit")) {
        Reply(out, nullptr);
        return true;
    } else {
        Reply(out, "unknown command (help)");
    }
    return false;
}
