/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#include "CamEngine.h"

#include "EipProtocol.h"

#include "CamRuntime.h"
#include "OperatorPanel.h"
#include "ClearCore.h"
#include "ECamConfig.h"
#include "ProfileText.h"
#include "FlashStore.h"
#include "Settings.h"
#include "EipLink.h"
#include "Transport.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <string>

using namespace ecam;

static CamRuntime g_rt;
static CamProject g_project;  /* the loaded profile (names, counts per unit) */
static ProfileReader g_reader;
static uint32_t g_hash = 0;
static bool g_loaded = false;
static std::string g_text;       /* the loaded profile's upload text ("download" returns it) */
static bool g_stored = false;    /* it is also the one in the flash store */
static bool g_restoring = false; /* replaying the stored profile at boot: do not store it again */
static bool g_committing = false; /* EngineCommit is building or storing, or enable waits on HLFB (the yield runs) */
static void (*g_yield)() = nullptr;
static uint16_t g_refusal = 0;      /* eipp::RejectCode of the last refused call */
static bool g_commLossStop = false; /* COMM_LOSS: stopping; disable once at rest and settled */
static bool g_commLossAtRest = false;   /* the stop's commanded motion has ended: the settle wait is on */
static uint32_t g_commLossRestMs = 0;   /* when it ended */

/* A refused call: remember why for the PLC, return the console's message. */
static const char *Refuse(uint16_t code, const char *msg) {
    g_refusal = code;
    return msg;
}
static uint32_t g_commitUs[3] = {}; /* the last commit: check, build, store (diag) */

static MotorDriver *const kMotors[kMotorConnectors] = {&ConnectorM0, &ConnectorM1, &ConnectorM2, &ConnectorM3};
static bool g_used[kMotorConnectors] = {};
/* g_on: each motor enabled (all of them by enable, one by axis enable); g_enabled: every one the profile uses. */
static bool g_on[kMotorConnectors] = {};
/* Axis jog of a motor the profile doesn't use (any motor, with no profile): its own generator, ticked by the sample
 * interrupt, and its position in counts since the boot (or its zero). */
static ManualMove g_free[kMotorConnectors];
static volatile int64_t g_freePos[kMotorConnectors] = {};

/* Limits: the profile's limit inputs (set at commit), read by the sample interrupt every sample. A limit is tripped
 * once its input has read tripped for 3 samples (0.6 ms) in a row, the contact applied (NC: low; NO: high); a CCIO-8
 * limit is tripped while the link is down, whatever its contact. */
struct LimitIn {
    uint8_t ccio;          /* 0: the board's input (index 0..12), 1: a CCIO-8's (bit 0..63) */
    uint8_t index;
    uint8_t axis;
    int8_t side;           /* +1 / -1 a limit, 0 a home switch */
    uint8_t normallyOpen;
    uint8_t count;         /* samples the reading has disagreed with the state */
    uint8_t tripped;
};
static const int kMaxLimits = 16;
static LimitIn g_limits[kMaxLimits];
static volatile int g_limitCount = 0;
static volatile uint8_t g_limPlus = 0, g_limMinus = 0;          /* tripped now, bit = axis */
static volatile uint8_t g_limTestPlus = 0, g_limTestMinus = 0;  /* limit_test: forced tripped (RAM) */
static volatile uint8_t g_homeSw = 0;                           /* home switches tripped now, bit = axis */
/* home_test: a simulated home switch fixed to the motor (homing moves the axis's position, not the switch), tripped
 * beyond it toward the home direction (RAM). Positions in the axis's sense: the steps sent, times the motor's sign. */
static volatile uint8_t g_homeTest = 0;
static volatile int64_t g_homeTestAt[kRtAxes] = {};
static volatile int8_t g_homeTestDir[kRtAxes] = {};
static volatile int8_t g_homeTestSign[kRtAxes] = {};
static volatile int64_t g_motorSteps[kMotorConnectors] = {};  /* every step sent to each motor since the boot */
static uint32_t g_homeEventsSeen = 0;

/* Operator inputs: the panel's buttons, filtered in the sample interrupt, acted on in the main loop (OpPoll). */
static OperatorPanel g_panel;

/* Timeline waits: each one's input, read by the sample interrupt (board / CCIO-8 levels filtered over 3 samples as the
 * limits are; a PLC bit as last received, and only with a connection in Run). */
struct WaitIn {
    uint8_t source;   /* 0 board, 1 CCIO-8, 2 PLC */
    uint8_t index;    /* board input, CCIO-8 bit (board * 8 + pin), PLC bit */
    uint8_t on;       /* the level it waits for */
    uint8_t count;    /* samples the raw level has disagreed with the filtered one */
    uint8_t level;    /* the filtered level (a PLC bit: as received) */
};
static WaitIn g_waits[kMaxWaits];
static volatile int g_waitCount = 0;
static volatile uint32_t g_plcWaitBits = 0;
static volatile bool g_plcLive = false;
static volatile bool g_plcTestOn = false;     /* waitbits_test: these bits instead of the PLC's, live (RAM, bench) */
static volatile uint32_t g_plcTestBits = 0;
static volatile uint16_t g_waitMet = 0;
static uint32_t g_waitEventsSeen = 0;
static uint32_t g_limitEventsSeen = 0;

/* Stop on a switch: an axis jog that ramps to rest the moment one of the watched inputs changes from its level when
 * the jog started (the wizard's discovery moves, before any limit exists). */
struct StopOn {
    bool active;
    int axis;              /* the profile's axis, -1 a motor it doesn't use */
    uint16_t board;        /* watched: the board's inputs, bit = index */
    uint64_t ccio;         /* the CCIO-8s', bit = board * 8 + pin */
    uint16_t boardAt;
    uint64_t ccioAt;
    int hit;               /* the input that stopped it: 0..12 the board's, 100 + bit a CCIO-8's; -1 none */
    int dir;               /* the direction it was going when it stopped */
    int64_t at;            /* the position (counts) where it changed */
};
static StopOn g_stopOn[kMotorConnectors] = {};
static volatile bool g_stopOnAny = false;
static bool g_enabled = false;
static bool g_simulate = false;
static const char *g_fault = nullptr;
static uint16_t g_faultCode = 0; /* eipp::FaultCode beside the message */

/* Read by the sample interrupt. */
static volatile bool g_outputSteps = false;
static volatile bool g_tick = true;  /* false while a new profile is being built */
static Connector *volatile g_triggerInput = nullptr;

/* Interrupt timing (DWT cycle counter). */
static volatile uint32_t g_isrMaxCycles = 0;
static volatile uint32_t g_carryEvents = 0;

static Connector *TriggerConnector(int index) {
    static Connector *const kInputs[kTriggerInputs] = {
        &ConnectorIO0, &ConnectorIO1, &ConnectorIO2, &ConnectorIO3, &ConnectorIO4, &ConnectorIO5, &ConnectorDI6,
        &ConnectorDI7, &ConnectorDI8, &ConnectorA9,  &ConnectorA10, &ConnectorA11, &ConnectorA12,
    };
    return (index >= 0 && index < kTriggerInputs) ? kInputs[index] : nullptr;
}

/* Timeline outputs IO-0..IO-5. g_outputPins: the profile's outputs (set to output mode at commit);
 * g_outputsOn: what was last written to them. */
static Connector *const kOutputConnectors[kOutputs] = {&ConnectorIO0, &ConnectorIO1, &ConnectorIO2,
                                              &ConnectorIO3, &ConnectorIO4, &ConnectorIO5};
static volatile uint8_t g_outputPins = 0;

/* Simulated encoder (simulate only): the cam follows a count the firmware advances itself, so a moving
 * line can be simulated without an encoder. Q16 counts; the rate is counts per sample (Q16). */
static volatile bool g_simEncOn = false;
static volatile int64_t g_simEncQ16 = 0;
static volatile int32_t g_simEncRateQ16 = 0;
static double g_simEncRate = 0.0;  /* counts/s, for status */
static volatile int32_t g_simEncCount = 0;  /* whole counts, 32-bit so the main loop reads it whole */

/* The encoder count the cam follows and status reports. */
static int32_t EncoderCount() {
    return g_simEncOn ? g_simEncCount : EncoderIn.Position();
}
static uint8_t g_outputsOn = 0;

static const char *kTriggerNames[kTriggerInputs] = {"IO-0", "IO-1", "IO-2", "IO-3", "IO-4", "IO-5", "DI-6",
                                                    "DI-7", "DI-8", "A-9",  "A-10", "A-11", "A-12"};

/* ------------------------------------------------------------------------- */
/* Sample interrupt                                                           */
/* ------------------------------------------------------------------------- */

/* Sample-to-sample interval: a late sample means the step-count register was not updated before the
 * step timer rolled over, so the drive got the previous sample's pulses again. */
static volatile uint32_t g_maxGapCycles = 0;
static volatile uint32_t g_lateSamples = 0;
static uint32_t g_lastSampleCycles = 0;

static void OnSample() {
    const uint32_t start = DWT->CYCCNT;
    if (g_lastSampleCycles != 0) {
        const uint32_t gap = start - g_lastSampleCycles;
        if (gap > g_maxGapCycles) {
            g_maxGapCycles = gap;
        }
        /* 1.5 sample periods: the callback would have missed a step-timer rollover. */
        if (gap > (SystemCoreClock / kRtSampleHz) * 3u / 2u) {
            g_lateSamples = g_lateSamples + 1;
        }
    }
    g_lastSampleCycles = start;
    CamRuntime::Input in;
    if (g_simEncOn) {
        g_simEncQ16 = g_simEncQ16 + g_simEncRateQ16;
        g_simEncCount = (int32_t)(g_simEncQ16 >> 16);
    }
    in.encoder = EncoderCount();
    Connector *const trig = g_triggerInput;
    in.trigger = trig ? trig->State() != 0 : false;
    int32_t steps[kMotorConnectors];
    if (!g_tick) {
        return;
    }
    /* The inputs, when anything watches them: limits, or a stop on a switch. */
    uint16_t board = 0;
    uint64_t ccio = 0;
    bool link = true;
    const int limits = g_limitCount;
    const int waits = g_waitCount;
    if (limits > 0 || g_stopOnAny || waits > 0 || g_panel.Any()) {  /* limits, home switches, waits, the panel */
        for (int i = 0; i < kTriggerInputs; ++i) {
            if (TriggerConnector(i)->State()) board |= (uint16_t)(1u << i);
        }
        link = ConnectorCOM1.PortIsOpen() && !CcioMgr.LinkBroken();
        ccio = link ? CcioMgr.InputState() : 0;
    }
    uint8_t plus = g_limTestPlus, minus = g_limTestMinus, home = 0;
    for (int k = 0; k < limits; ++k) {
        LimitIn &l = g_limits[k];
        const bool level = l.ccio ? ((ccio >> l.index) & 1u) != 0 : ((board >> l.index) & 1u) != 0;
        const bool active = (l.ccio && !link) || (l.normallyOpen ? level : !level);
        if (active != (l.tripped != 0)) {
            if (++l.count >= 3) {
                l.tripped = active ? 1 : 0;
                l.count = 0;
            }
        } else {
            l.count = 0;
        }
        if (l.tripped) {
            if (l.side > 0) plus |= (uint8_t)(1u << l.axis);
            else if (l.side < 0) minus |= (uint8_t)(1u << l.axis);
            else home |= (uint8_t)(1u << l.axis);
        }
    }
    if (g_homeTest) {
        for (int i = 0; i < kRtAxes && i < g_rt.AxisCount(); ++i) {
            if (!((g_homeTest >> i) & 1u)) continue;
            const int64_t pos = g_homeTestSign[i] * g_motorSteps[g_rt.AxisConnector(i)];
            if (g_homeTestDir[i] < 0 ? pos <= g_homeTestAt[i] : pos >= g_homeTestAt[i]) home |= (uint8_t)(1u << i);
        }
    }
    /* Each wait's input: at the level it wants (met), or at the other one (which arms a wait that needs a change). Neither
     * while it can't be read: a CCIO-8 with the link down, a PLC bit with no connection in Run. */
    uint16_t met = 0, opposite = 0;
    for (int k = 0; k < waits; ++k) {
        WaitIn &w = g_waits[k];
        bool valid = true;
        if (w.source == 2) {
            valid = g_plcTestOn || g_plcLive;
            const uint32_t bits = g_plcTestOn ? g_plcTestBits : g_plcWaitBits;
            w.level = ((bits >> w.index) & 1u) ? 1 : 0;
        } else {
            const uint8_t raw = (w.source == 1 ? ((ccio >> w.index) & 1u) : ((board >> w.index) & 1u)) ? 1 : 0;
            valid = w.source != 1 || link;
            if (raw != w.level) {
                if (++w.count >= 3) {
                    w.level = raw;
                    w.count = 0;
                }
            } else {
                w.count = 0;
            }
        }
        if (!valid) continue;
        if (w.level == w.on) met = (uint16_t)(met | (1u << k));
        else opposite = (uint16_t)(opposite | (1u << k));
    }
    in.waitMet = met;
    in.waitOpposite = opposite;
    g_waitMet = met;
    if (g_panel.Any()) g_panel.Sample(board);
    g_limPlus = plus;
    g_limMinus = minus;
    g_homeSw = home;
    if (!g_simulate) {  /* in simulate nothing moves: limits are reported, not enforced */
        in.limitPlus = plus;
        in.limitMinus = minus;
        in.home = home;
    }
    g_rt.Tick(in, steps);
    for (int c = 0; c < kMotorConnectors; ++c) {
        if (!g_used[c] && g_free[c].Active()) {
            const int64_t was = g_freePos[c];
            const int64_t now = g_free[c].Tick(was);
            g_freePos[c] = now;
            steps[c] += (int32_t)(now - was);
        }
    }
    if (g_stopOnAny) {
        bool any = false;
        for (int c = 0; c < kMotorConnectors; ++c) {
            StopOn &s = g_stopOn[c];
            if (!s.active) continue;
            const bool moving = s.axis >= 0 ? g_rt.AxisMoving(s.axis) : g_free[c].Active();
            if (!moving) {
                s.active = false;  /* the jog ended first (let go, the deadman) */
                continue;
            }
            const uint16_t b = (uint16_t)((board ^ s.boardAt) & s.board);
            const uint64_t x = (ccio ^ s.ccioAt) & s.ccio;
            if (b || x) {
                int hit = -1;
                for (int i = 0; i < kTriggerInputs && hit < 0; ++i) {
                    if (b & (1u << i)) hit = i;
                }
                for (int i = 0; i < 64 && hit < 0; ++i) {
                    if ((x >> i) & 1u) hit = 100 + i;
                }
                s.hit = hit;
                const int64_t v = s.axis >= 0 ? 0 : g_free[c].VelocityQ24();
                if (s.axis >= 0) {
                    s.at = g_rt.AxisPosition(s.axis);
                    g_rt.AxisStopRapid(s.axis);
                } else {
                    s.at = g_freePos[c];
                    g_free[c].StopRapid((v < 0 ? -v : v) / g_rt.LimitStopSamples() + 1);
                }
                s.active = false;
                continue;
            }
            any = true;
        }
        g_stopOnAny = any;
    }
    /* Outputs: only the profile's pins, only on a change; never in simulate (a simulation must not
     * fire real actuators; status reports what they would be). */
    const uint8_t want = g_simulate ? 0 : (uint8_t)(g_rt.Outputs() & g_outputPins);
    const uint8_t changed = (uint8_t)(want ^ g_outputsOn);
    if (changed) {
        for (int n = 0; n < kOutputs; ++n) {
            if (changed & (1u << n)) {
                kOutputConnectors[n]->State((want >> n) & 1u);
            }
        }
        g_outputsOn = want;
    }
    if (g_outputSteps) {
        for (int c = 0; c < kMotorConnectors; ++c) {
            if (g_used[c] || g_on[c]) {
                g_motorSteps[c] = g_motorSteps[c] + steps[c];
                kMotors[c]->SetExternalSteps(steps[c]);
                if (kMotors[c]->ExternalStepsCarry() != 0) {
                    g_carryEvents = g_carryEvents + 1;
                }
            }
        }
    }
    const uint32_t took = DWT->CYCCNT - start;
    if (took > g_isrMaxCycles) {
        g_isrMaxCycles = took;
    }
}

/* ------------------------------------------------------------------------- */
/* Setup and faults                                                           */
/* ------------------------------------------------------------------------- */

void EngineEstopInputMode();

static void OpConfigure();

void EngineApplySettings() {
    g_rt.SetStopSeconds(g_settings.stopMs / 1000.0);
    EngineEstopInputMode();
    OpConfigure();
}

/* The boards the search found. Closing the port zeroes the library's count, and the Workbench still shows them. */
static uint8_t g_ccioFound = 0;

static uint8_t CcioBoards() {
    return ConnectorCOM1.PortIsOpen() ? CcioMgr.CcioCount() : g_ccioFound;
}

/* The sample interrupt polls the link with a DMA transfer: let the one in flight finish (a few hundred us at most,
 * and only here, with nothing moving) and stop the polling before the port goes. */
static void CcioClose() {
    if (CcioMgr.CcioCount() != 0) {
        g_ccioFound = CcioMgr.CcioCount();
    }
    __disable_irq();
    ConnectorCOM1.SpiAsyncWaitComplete();
    CcioMgr.LinkClose();
    __enable_irq();
    ConnectorCOM1.PortClose();
}

/* A loaded profile decides the link: without boards in it, nothing talks to them, so a break can't fault. With no
 * profile the link stays up and the Workbench can show the inputs. Boards found at boot and not at this reopen are
 * gone until a restart, as at boot. */
static void CcioApplyProfile(int boards) {
    const bool open = ConnectorCOM1.PortIsOpen();
    if (boards == 0 && open) {
        CcioClose();
    } else if (boards > 0 && !open && g_ccioFound != 0) {
        ConnectorCOM1.PortOpen();
        if (CcioMgr.CcioCount() == 0) {
            CcioClose();
            g_ccioFound = 0;
        }
    }
}

void EngineInit() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
    Delay_ms(50);
    for (MotorDriver *m : kMotors) {
        m->HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
        m->HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
        m->EnableRequest(false);
        m->ExternalStepMode(true);  /* axis jog steps any motor before a profile is loaded */
    }
    ConnectorA9.Mode(Connector::INPUT_DIGITAL);  /* the estop's input by default (estop_pin); DI-6..DI-8 are the Encoder Adapter */
    /* A-10..A-12 as digital inputs too, so status reports them as levels (din=): a trigger or a home / limit input. */
    ConnectorA10.Mode(Connector::INPUT_DIGITAL);
    ConnectorA11.Mode(Connector::INPUT_DIGITAL);
    ConnectorA12.Mode(Connector::INPUT_DIGITAL);
    EncoderIn.Enable(true);
    /* CCIO-8 inputs on COM-1. With the port open and no board, the library runs a blocking discovery from the
     * SysTick interrupt every second, so with none found the port closes: a board added later needs a restart.
     * The wait covers a board still powering up: the library's own retry comes after a second. */
    ConnectorCOM1.Mode(Connector::CCIO);
    ConnectorCOM1.PortOpen();
    if (CcioMgr.CcioCount() == 0) {
        Delay_ms(1200);
    }
    g_ccioFound = CcioMgr.CcioCount();
    if (g_ccioFound == 0) {
        ConnectorCOM1.PortClose();
    }
    EngineApplySettings();
    SysMgr.SampleCallbackSet(OnSample);
}

/* All the inputs, read until two reads agree: 64 bits written by the sample interrupt can tear. The sensors are
 * normally closed, so a broken link reads as every input open, not the library's last values. The link is looked at
 * after the read, so a break during it still reads open. */
static uint64_t CcioInputs() {
    uint64_t a, b;
    do {
        a = CcioMgr.InputState();
        b = CcioMgr.InputState();
    } while (a != b);
    return CcioMgr.LinkBroken() || !ConnectorCOM1.PortIsOpen() ? 0 : a;
}

static const char *CcioLinkName() {
    return !ConnectorCOM1.PortIsOpen() ? "off" : CcioMgr.LinkBroken() ? "broken" : "ok";
}

void EngineCcio(void (*out)(const char *line)) {
    char buf[96];
    const uint64_t in = CcioInputs();
    snprintf(buf, sizeof(buf), "ccio boards=%u link=%s inputs=%08lx%08lx", (unsigned)CcioBoards(),
             CcioBoards() == 0 ? "none" : CcioLinkName(), (unsigned long)(in >> 32),
             (unsigned long)(in & 0xFFFFFFFFu));
    out(buf);
}

/* What the profile uses this input for, or nullptr: an output, the trigger, a timeline wait, a home or limit, the Encoder
 * Adapter's DI-6..DI-8 with an encoder master. */
static const char *ProfileUse(const CamProject &p, int pin) {
    if (pin < kOutputs) {
        for (const OutputPulse &o : p.outputs) {
            if (o.output == pin) return "an output";
        }
    }
    if (UsesTimeline(p) && p.master.trigger == TriggerSource::DigitalInput && p.master.triggerInput == pin) return "the trigger";
    if (WaitOnPin(p, pin)) return "a timeline wait's input";
    for (const BoardInput &i : p.inputs) {
        if (i.pin == pin && i.role != CcioRole::None) return "a home or limit input";
    }
    if (p.master.source == MasterSource::Encoder && UsesCrank(p) && pin >= kEncoderInputFirst && pin <= kEncoderInputLast) {
        return "the Encoder Adapter's";
    }
    return nullptr;
}

static const char *const kOpLabel[] = {"Enable", "Disable", "Fault reset", "Start", "Stop"};

/* An operator input on a pin the profile uses: the reason, or nullptr. */
static const char *OpPinClash(const CamProject &p) {
    static char msg[112];
    for (int k = 0; k < kOpInputs; ++k) {
        const int pin = g_settings.opPin[k] == 0xFF ? -1 : g_settings.opPin[k];
        if (pin < 0) continue;
        if (const char *use = ProfileUse(p, pin)) {
            snprintf(msg, sizeof(msg), "%s is the operator %s input: the profile can't use it as %s (set %s off, or another input)",
                     SettingsInputName(pin), kOpLabel[k], use, SettingsOpName(k));
            return msg;
        }
    }
    return nullptr;
}

/* The estop on this input clashes with the profile: an output, the trigger, or the Encoder Adapter's DI-6..DI-8 with an
 * encoder master. The reason, or nullptr. Only while the estop is on. */
static const char *EstopPinClash(const CamProject &p, int pin) {
    if (g_settings.estopMode == 0) {
        return nullptr;
    }
    static char msg[112];
    const char *name = SettingsInputName(pin);
    if (pin < kOutputs) {
        for (const OutputPulse &o : p.outputs) {
            if (o.output == pin) {
                snprintf(msg, sizeof(msg), "%s is the estop's input: it can't be an output (set estop_pin, or estop_mode 0)", name);
                return msg;
            }
        }
    }
    if (UsesTimeline(p) && p.master.trigger == TriggerSource::DigitalInput && p.master.triggerInput == pin) {
        snprintf(msg, sizeof(msg), "%s is the estop's input: it can't be the trigger (set estop_pin, or estop_mode 0)", name);
        return msg;
    }
    if (WaitOnPin(p, pin)) {
        snprintf(msg, sizeof(msg), "%s is the estop's input: it can't be a timeline wait's (set estop_pin, or estop_mode 0)", name);
        return msg;
    }
    for (const BoardInput &i : p.inputs) {
        if (i.pin == pin) {
            snprintf(msg, sizeof(msg), "%s is the estop's input: it can't be a home or limit input (set estop_pin, or estop_mode 0)", name);
            return msg;
        }
    }
    if (p.master.source == MasterSource::Encoder && UsesCrank(p) && pin >= kEncoderInputFirst && pin <= kEncoderInputLast) {
        snprintf(msg, sizeof(msg), "%s is the estop's input: the encoder master's Encoder Adapter needs DI-6..DI-8", name);
        return msg;
    }
    return nullptr;
}

static const char *EstopOpenText() {
    static char msg[80];
    snprintf(msg, sizeof(msg), "estop open (%s); wire it or set estop_mode 0", SettingsInputName(g_settings.estopPin));
    return msg;
}

const char *EngineEstopCheck(int pin, int mode) {
    if (mode != 0) {
        for (int k = 0; k < kOpInputs; ++k) {
            if (g_settings.opPin[k] == pin) {
                static char msg[96];
                snprintf(msg, sizeof(msg), "%s is the operator %s input (%s off first)", SettingsInputName(pin), kOpLabel[k], SettingsOpName(k));
                return msg;
            }
        }
    }
    if (mode == 0 || !g_loaded) {
        return nullptr;
    }
    const uint8_t was = g_settings.estopMode;
    g_settings.estopMode = (uint8_t)mode;
    const char *err = EstopPinClash(g_project, pin);
    g_settings.estopMode = was;
    return err;
}

void EngineEstopInputMode() {
    /* The estop's input reads as a digital input, unless the loaded profile drives it (which commit refuses). */
    Connector *const in = TriggerConnector(g_settings.estopPin);
    if (in && in->Mode() != Connector::OUTPUT_DIGITAL) {
        in->Mode(Connector::INPUT_DIGITAL);
    }
}

static bool g_estopTest = false;  /* estop_test: reads as open until cleared or restarted. It can only stop a machine */

static bool EstopTripped() {
    if (g_estopTest) {
        return true;
    }
    if (g_settings.estopMode == 0) {
        return false;
    }
    Connector *const in = TriggerConnector(g_settings.estopPin);
    if (!in) {
        return true;  /* no such input: as open, it can only stop a machine */
    }
    const bool high = in->State() != 0;
    return g_settings.estopMode == 1 ? !high : high;
}

static void Snapshot(RtStatus *s) {
    __disable_irq();
    g_rt.Status(s);
    __enable_irq();
}

static bool AllHlfbAsserted();

static bool Moving() {
    RtStatus s;
    Snapshot(&s);
    return s.crank != CrankState::Off || s.timeline == TimelineState::Running ||
           s.timeline == TimelineState::Stopping || s.timeline == TimelineState::Jogging || s.timeline == TimelineState::Waiting;
}

/* A used motor is in fault now: enabled (past the enable delay) with HLFB deasserted. The library's
 * MotorInFault bit is not used: it stays set across a disable until the motor next reaches READY, so it
 * would fault the next enable during its enable delay. */
static bool MotorFaulted(MotorDriver *m) {
    return m->StatusReg().bit.ReadyState == MotorDriver::MOTOR_FAULTED;
}

static bool AnyOn() {
    for (int c = 0; c < kMotorConnectors; ++c) {
        if (g_on[c]) {
            return true;
        }
    }
    return false;
}

/* After g_on changes: steps go out while any motor is on (a disabled one gets none: nothing moves it). */
static void UpdateEnabled() {
    bool all = false;
    for (int c = 0; c < kMotorConnectors; ++c) {
        if (g_used[c]) {
            all = true;
            if (!g_on[c]) {
                all = false;
                break;
            }
        }
    }
    g_enabled = all;
    g_outputSteps = AnyOn() && !g_simulate;
}

static void DisableMotors() {
    g_outputSteps = false;
    __disable_irq();
    g_rt.AxisHalt();
    g_rt.ClearHomed(0xFF);  /* a disabled ClearPath can be turned by hand: the position is no longer known */
    for (ManualMove &f : g_free) {
        f.Halt();
    }
    __enable_irq();
    for (int c = 0; c < kMotorConnectors; ++c) {
        kMotors[c]->EnableRequest(false);
        g_on[c] = false;
    }
    g_enabled = false;
}

static void Fault(uint16_t code, const char *why) {
    g_rt.RequestAbort();
    DisableMotors();
    g_fault = why;
    g_faultCode = code;
}

static bool AxisJogMoving() {
    __disable_irq();
    bool m = g_rt.AnyAxisMoving();
    for (const ManualMove &f : g_free) {
        m = m || f.Active();
    }
    __enable_irq();
    return m;
}

bool EngineBusy() { return AnyOn() || Moving() || AxisJogMoving(); }

bool EngineMoving() {
    return Moving() || AxisJogMoving();
}

uint16_t EngineRefusal() {
    return g_refusal;
}

void EngineCommLoss() {
    if (Moving()) {
        g_rt.RequestStop();
        g_commLossStop = true;
        g_commLossAtRest = false;
        /* Latched now (start refuses); the motors disable once at rest and settled (EnginePoll). */
        g_fault = "EtherNet/IP connection lost";
        g_faultCode = eipp::kFaultCommLoss;
    } else if (AnyOn() || AxisJogMoving()) {
        DisableMotors();
    }
}

/* The end of the commanded stop is not the end of the motion: the motors are still closing their following
 * error, and disabling then lets them coast. Hold them until HLFB says they have settled, for at most
 * ECAM_STOP_SETTLE_MS (the whole of it with hlfb_check 0: nothing says when). */
static bool CommLossSettled() {
    if (!g_commLossAtRest) {
        g_commLossAtRest = true;
        g_commLossRestMs = Milliseconds();
    }
    return !AnyOn() || (g_settings.hlfbCheck && AllHlfbAsserted()) ||
           Milliseconds() - g_commLossRestMs >= ECAM_STOP_SETTLE_MS;
}

/* The library latches blink code 3-4 (CCIO-8 error) and leaves it blinking after the link is back, so it is cleared
 * when the link comes back. Starting "down" also clears the code a failed search at power-up left behind: closing
 * the port drops LinkBroken but not the code. While the link is broken the library sets the code again every sample,
 * so a break during this call still shows. The interrupts are held off because the sample interrupt writes the same
 * byte. */
static bool g_ccioDown = true;

static void CcioBlinkPoll() {
    const bool down = CcioMgr.LinkBroken();
    if (g_ccioDown && !down) {
        __disable_irq();
        StatusMgr.BlinkCodeClear(3, 4);
        __enable_irq();
    }
    g_ccioDown = down;
}

static void AxisJogPoll();

static void LimitPoll();
static void HomePoll();
static void WaitPoll();
static void OpPoll();

void EnginePoll() {
    CcioBlinkPoll();
    if (g_commLossStop && !Moving() && CommLossSettled()) {
        g_commLossStop = false;
        DisableMotors();
    }
    if (EstopTripped() && (AnyOn() || Moving() || AxisJogMoving())) {
        static char msg[32];
        snprintf(msg, sizeof(msg), "estop (%s)", SettingsInputName(g_settings.estopPin));
        Fault(eipp::kFaultEstop, msg);
    }
    AxisJogPoll();
    LimitPoll();
    HomePoll();
    WaitPoll();
    OpPoll();
    {
        for (int c = 0; c < kMotorConnectors; ++c) {
            if (g_on[c] && MotorFaulted(kMotors[c])) {
                static char msg[32];
                snprintf(msg, sizeof(msg), "motor fault on M-%d", c);
                Fault(eipp::kFaultMotor, msg);
                break;
            }
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Upload                                                                     */
/* ------------------------------------------------------------------------- */

const char *EngineLoadBegin() {
    if (Moving()) {
        return "stop the motion before loading";
    }
    g_reader.Begin();
    return nullptr;
}

bool EngineLoading() {
    return g_reader.Active();
}

const char *EngineLoadLine(const char *line) {
    return g_reader.Line(line);
}

static uint32_t UsSince(uint32_t cycles) {
    return (DWT->CYCCNT - cycles) / (SystemCoreClock / 1000000u);
}

void EngineSetYield(void (*fn)()) {
    g_yield = fn;
    g_rt.SetBuildYield(fn);
    FlashStoreSetYield(fn);
}

static const char *Commit(char *info, size_t len);
static bool g_axMoved[kRtAxes] = {};  /* axis jog or zero moved it since the cam last placed it */
static bool JogMoving(int c);

const char *EngineCommit(char *info, size_t len) {
    g_committing = true;
    const char *err = Commit(info, len);
    g_committing = false;
    return err;
}

static const char *Commit(char *info, size_t len) {
    uint32_t t = DWT->CYCCNT;
    CamProject p;
    const char *err = g_reader.Finish(&p);
    g_commitUs[0] = UsSince(t);
    if (err) {
        return err;
    }
    if (Moving()) {
        return "stop the motion before loading";
    }
    if (const char *estop = EstopPinClash(p, g_settings.estopPin)) {
        return estop;
    }
    if (const char *op = OpPinClash(p)) {
        return op;
    }
    /* A new profile can use other motors: start from disabled. */
    DisableMotors();
    g_triggerInput = nullptr;
    /* The interrupt must not tick a half-built runtime. Building takes milliseconds of double-precision
     * work, so the interrupt skips the runtime instead of interrupts being turned off. */
    g_tick = false;
    __DSB();
    t = DWT->CYCCNT;
    err = g_rt.Build(p);
    g_commitUs[1] = UsSince(t);
    g_tick = true;
    if (err) {
        g_loaded = false;
        return err;
    }
    g_project = p;
    g_hash = g_reader.Hash();
    g_loaded = true;
    CcioApplyProfile(p.ccio.boards);

    for (int c = 0; c < kMotorConnectors; ++c) {
        g_used[c] = false;
    }
    for (int i = 0; i < kRtAxes; ++i) {
        g_axMoved[i] = false;
    }
    /* The limits, for the sample interrupt: off while they change. */
    {
        g_limitCount = 0;
        __DSB();
        int k = 0;
        auto side = [](CcioRole r) -> int8_t { return r == CcioRole::LimitPositive ? 1 : r == CcioRole::LimitNegative ? -1 : 0; };
        for (const BoardInput &i : p.inputs) {
            if (i.role != CcioRole::None && i.axis >= 0 && k < kMaxLimits) {
                g_limits[k++] = {0, (uint8_t)i.pin, (uint8_t)i.axis, side(i.role), (uint8_t)(i.normallyOpen ? 1 : 0), 0, 0};
            }
        }
        for (const CcioInput &i : p.ccio.inputs) {
            if (i.role != CcioRole::None && i.axis >= 0 && k < kMaxLimits) {
                g_limits[k++] = {1, (uint8_t)(i.board * kCcioPinsPerBoard + i.pin), (uint8_t)i.axis, side(i.role),
                                 (uint8_t)(i.normallyOpen ? 1 : 0), 0, 0};
            }
        }
        g_limTestPlus = 0;
        g_limTestMinus = 0;
        g_homeTest = 0;
        g_homeEventsSeen = g_rt.HomeEvents();
        g_waitCount = 0;
        __DSB();
        int nw = 0;
        for (const TimelineWait &w : p.waits) {
            if (nw >= kMaxWaits) break;
            const uint8_t src = w.source == WaitSource::Board ? 0 : w.source == WaitSource::Ccio ? 1 : 2;
            const uint8_t idx = (uint8_t)(w.source == WaitSource::Ccio ? w.board * kCcioPinsPerBoard + w.pin : w.pin);
            g_waits[nw++] = {src, idx, (uint8_t)(w.on ? 1 : 0), 0, (uint8_t)(w.on ? 0 : 1)};  /* starts at the other level */
        }
        __DSB();
        g_waitCount = nw;
        g_waitEventsSeen = g_rt.WaitEvents();
        __DSB();
        g_limitCount = k;
        g_limitEventsSeen = g_rt.LimitEvents();
        for (StopOn &s : g_stopOn) {
            s.active = false;
            s.hit = -1;
        }
        g_stopOnAny = false;
    }
    for (int i = 0; i < g_rt.AxisCount(); ++i) {
        g_used[g_rt.AxisConnector(i)] = true;
    }
    if (g_rt.CrankConnector() >= 0) {
        g_used[g_rt.CrankConnector()] = true;
    }
    /* Every motor takes the sample interrupt's steps: the profile's from the cam, the others from axis jog. */
    for (int c = 0; c < kMotorConnectors; ++c) {
        kMotors[c]->ExternalStepMode(true);
    }
    /* Encoder direction (the follower engages from wherever the count is, so flipping it is safe here:
     * nothing is moving). */
    EncoderIn.SwapDirection(p.master.source == MasterSource::Encoder && p.master.encoderReverse);
    /* Outputs: the profile's IO pins become outputs, off; pins the last profile used go back to inputs. */
    {
        const uint8_t used = g_rt.OutputsUsed();
        g_outputPins = 0;
        for (int n = 0; n < kOutputs; ++n) {
            const uint8_t bit = (uint8_t)(1u << n);
            if (used & bit) {
                kOutputConnectors[n]->Mode(Connector::OUTPUT_DIGITAL);
                kOutputConnectors[n]->State(0);
            } else if (g_outputsOn & bit || kOutputConnectors[n]->Mode() == Connector::OUTPUT_DIGITAL) {
                kOutputConnectors[n]->State(0);
                kOutputConnectors[n]->Mode(Connector::INPUT_DIGITAL);
            }
        }
        g_outputsOn = 0;
        g_outputPins = used;
    }
    if (g_rt.UsesTimeline() && p.master.trigger == TriggerSource::DigitalInput) {
        Connector *in = TriggerConnector(p.master.triggerInput);
        if (in) {
            in->Mode(Connector::INPUT_DIGITAL);
            g_triggerInput = in;
        }
    }
    g_text = g_reader.TakeText();
    /* Keep it in flash for the next boot (nothing is moving: commit refuses otherwise). */
    const char *storeErr = nullptr;
    if (g_restoring) {
        g_stored = true;
    } else {
        t = DWT->CYCCNT;
        storeErr = FlashStoreSave(g_text.data(), (uint32_t)g_text.size());
        g_commitUs[2] = UsSince(t);
        g_stored = storeErr == nullptr;
    }
    snprintf(info, len, "axes=%d hash=%08lx stored=%d%s%s", g_rt.AxisCount(), (unsigned long)g_hash, g_stored ? 1 : 0,
             storeErr ? " store_error=" : "", storeErr ? storeErr : "");
    return nullptr;
}

const char *EngineRestore(char *info, size_t len) {
    uint32_t n = 0;
    const char *text = FlashStoreText(&n);
    if (!text) {
        return "nothing stored";
    }
    /* Replay the stored upload through the same path as the console. */
    const char *err = EngineLoadBegin();
    const char *p = text;
    const char *end = text + n;
    char line[kProfileMaxLine + 1];
    while (!err && p < end) {
        const char *nl = static_cast<const char *>(memchr(p, '\n', end - p));
        const size_t l = (size_t)((nl ? nl : end) - p);
        if (l >= sizeof(line)) {
            err = "stored line too long";
            break;
        }
        memcpy(line, p, l);
        line[l] = '\0';
        err = EngineLoadLine(line);
        p = nl ? nl + 1 : end;
    }
    if (err) {
        g_reader.Begin();  /* drop the partial load */
        return err;
    }
    g_restoring = true;
    err = EngineCommit(info, len);
    g_restoring = false;
    return err;
}

const char *EngineDownload(void (*out)(const char *line)) {
    if (!g_loaded) {
        return "no profile loaded";
    }
    size_t start = 0;
    while (start < g_text.size()) {
        const size_t nl = g_text.find('\n', start);
        const size_t end = nl == std::string::npos ? g_text.size() : nl;
        out(g_text.substr(start, end - start).c_str());
        start = end + 1;
    }
    return nullptr;
}

void EngineEstopTest(bool open) { g_estopTest = open; }

const char *EngineForget() {
    if (EngineBusy()) {
        return "disable the motors and stop the motion first";
    }
    const char *err = FlashStoreErase();
    if (!err) {
        g_stored = false;
    }
    return err;
}

/* ------------------------------------------------------------------------- */
/* Control                                                                    */
/* ------------------------------------------------------------------------- */

static bool AllHlfbAsserted() {
    for (int c = 0; c < kMotorConnectors; ++c) {
        if (g_used[c] && g_on[c] && kMotors[c]->HlfbState() != MotorDriver::HLFB_ASSERTED) {
            return false;
        }
    }
    return true;
}

const char *EngineEnable() {
    if (!g_loaded) {
        return Refuse(eipp::kRejectNoProfile, "load a profile first (it says which motors to enable)");
    }
    if (EstopTripped()) {
        return Refuse(eipp::kRejectEstopOpen, EstopOpenText());
    }
    if (g_enabled) {
        return nullptr;
    }
    g_fault = nullptr;
    g_faultCode = 0;
    g_commLossStop = false;
    for (int c = 0; c < kMotorConnectors; ++c) {
        if (g_used[c]) {
            kMotors[c]->ClearAlerts();
            kMotors[c]->EnableRequest(true);
            g_on[c] = true;
        }
    }
    if (g_settings.hlfbCheck) {
        /* Up to a second: the network keeps running meanwhile (and refuses network changes). */
        g_committing = true;
        const uint32_t start = Milliseconds();
        while (!AllHlfbAsserted()) {
            if (Milliseconds() - start > ECAM_ENABLE_HLFB_WAIT_MS) {
                g_committing = false;
                /* Which motors held it up, before the disable: "M-2", "M-0 M-2". */
                static char which[24];
                static char fault[64];
                static char why[96];
                which[0] = '\0';
                for (int m = 0; m < kMotorConnectors; ++m) {
                    if (g_used[m] && g_on[m] && kMotors[m]->HlfbState() != MotorDriver::HLFB_ASSERTED) {
                        const size_t n = strlen(which);
                        snprintf(which + n, sizeof(which) - n, "%sM-%d", n ? " " : "", m);
                    }
                }
                DisableMotors();
                snprintf(fault, sizeof(fault), "HLFB not asserted on enable (%s)", which);
                snprintf(why, sizeof(why), "HLFB not asserted on %s (check the drive and its power, or set hlfb_check 0)", which);
                g_fault = fault;
                g_faultCode = eipp::kFaultHlfbTimeout;
                return Refuse(eipp::kRejectFaulted, why);
            }
            if (g_yield) {
                g_yield();
            }
        }
        g_committing = false;
    }
    UpdateEnabled();
    return nullptr;
}

const char *EngineDisable() {
    g_rt.RequestAbort();
    DisableMotors();
    return nullptr;
}

const char *EngineClearFault() {
    if (g_commLossStop) {
        return Refuse(eipp::kRejectBusy, "still stopping after the connection loss");
    }
    g_fault = nullptr;
    g_faultCode = 0;
    for (int c = 0; c < kMotorConnectors; ++c) {
        if (g_used[c]) {
            kMotors[c]->ClearAlerts();
        }
    }
    return nullptr;
}

static const char *AxisJogClear(bool atStart);
static const char *OnLimit();
static const char *TowardLimit(int axis, int dir);
static const char *NotHomed();

const char *EngineStart() {
    if (!g_loaded) {
        return Refuse(eipp::kRejectNoProfile, "no profile loaded");
    }
    if (g_fault) {
        return Refuse(eipp::kRejectFaulted, "fault latched (clear)");
    }
    if (EngineLoading()) {
        return Refuse(eipp::kRejectBusy, "a profile upload is in progress");
    }
    if (!g_simulate && !g_enabled) {
        return Refuse(eipp::kRejectNotEnabled, "motors disabled (enable, or sim 1 to run without motors)");
    }
    const char *err = AxisJogClear(true);
    if (err) {
        return err;
    }
    if ((err = NotHomed()) != nullptr) {
        return err;
    }
    g_rt.RequestStart();
    return nullptr;
}

/* Stop: a running timeline finishes its cycle (Stop again: at once, ramped); a crank ramps to rest. The connection
 * loss stops at once (EngineCommLoss). */
const char *EngineStop() {
    g_rt.RequestCycleStop();
    EngineHomeStop();  /* Stop also stops a homing (the PLC's STOP) */
    return nullptr;
}

const char *EngineAbort() {
    g_rt.RequestAbort();
    return nullptr;
}

const char *EngineEipTrigger() {
    if (!g_loaded) {
        return Refuse(eipp::kRejectNoProfile, "no profile loaded");
    }
    if (!g_rt.UsesTimeline() || g_project.master.trigger != TriggerSource::EtherNetIP) {
        return Refuse(eipp::kRejectNotEipTrigger, "the profile's timeline trigger is not EtherNet/IP");
    }
    if (AxisJogMoving()) {
        return Refuse(eipp::kRejectBusy, "an axis jog is moving");
    }
    if (const char *lim = OnLimit()) {
        return lim;
    }
    if (const char *h = NotHomed()) {
        return h;
    }
    g_rt.RequestTrigger();
    return nullptr;
}

EngineMasters EngineGetMasters() {
    EngineMasters m = {};
    if (!g_loaded) {
        return m;
    }
    m.loaded = true;
    m.crank = g_rt.UsesCrank();
    m.drivenCrank = g_rt.DrivenCrank();
    m.timeline = g_rt.UsesTimeline();
    m.lineMode = g_rt.LineMode();
    m.rpm = g_project.master.rpm;
    m.override = g_project.master.speedOverride;
    return m;
}

const char *EngineTrigger() {
    if (!g_loaded || !g_rt.UsesTimeline()) {
        return "no timeline axes loaded";
    }
    if (AxisJogMoving()) {
        return Refuse(eipp::kRejectBusy, "an axis jog is moving");
    }
    if (const char *lim = OnLimit()) {
        return lim;
    }
    if (const char *h = NotHomed()) {
        return h;
    }
    g_rt.RequestTrigger();
    return nullptr;
}

const char *EngineOverride(double fraction) {
    if (fraction < 0.0) {
        return Refuse(eipp::kRejectOutOfRange, "override: 0..1000 %");
    }
    if (fraction > 10.0) {
        return Refuse(eipp::kRejectOverLimit, "override: 0..1000 %");
    }
    g_rt.SetOverride(fraction);
    return nullptr;
}

const char *EngineRpm(double rpm) {
    if (!g_loaded) {
        return Refuse(eipp::kRejectNoProfile, "no driven crank loaded");
    }
    if (!g_rt.DrivenCrank()) {
        return Refuse(eipp::kRejectNotDrivenCrank, "no driven crank loaded");
    }
    if (rpm < 0.0) {
        return Refuse(eipp::kRejectOutOfRange, "rpm: 0 .. the crank motor max");
    }
    if (rpm > g_project.master.crankMaxRpm) {
        return Refuse(eipp::kRejectOverLimit, "rpm: 0 .. the crank motor max");
    }
    /* The target is 64-bit: keep the interrupt from reading half of it. */
    __disable_irq();
    g_rt.SetRpm(rpm);
    __enable_irq();
    return nullptr;
}

const char *EngineJog(bool on) {
    if (!on) {
        g_rt.RequestJog(false);
        return nullptr;
    }
    if (!g_loaded) {
        return Refuse(eipp::kRejectNoProfile, "no profile loaded");
    }
    if (g_fault) {
        return Refuse(eipp::kRejectFaulted, "fault latched (clear)");
    }
    if (!g_simulate && !g_enabled) {
        return Refuse(eipp::kRejectNotEnabled, "motors disabled (enable, or sim 1)");
    }
    if (Moving()) {
        return Refuse(eipp::kRejectBusy, "stop the motion first");
    }
    const char *err = AxisJogClear(false);
    if (err) {
        return err;
    }
    g_rt.RequestJog(true);
    return nullptr;
}

const char *EngineJogCrank(double degrees) {
    if (!g_loaded || !g_rt.UsesCrank()) {
        return Refuse(g_loaded ? eipp::kRejectOutOfRange : eipp::kRejectNoProfile, "no crank axes loaded");
    }
    double counts = degrees * g_rt.CountsPerRev() / 360.0;
    if (g_rt.LineMode()) {
        /* Degrees are of the cycle as drawn: whole turns are cuts of the current length, and within a
         * cut the drawn position sits after the stretched (or shortened) wait. */
        const double turns = floor(degrees / 360.0);
        const double within = (degrees - turns * 360.0) / 360.0 * g_rt.DesignCounts();
        double phase = within + (double)(g_rt.CountsPerRev() - g_rt.DesignCounts());
        if (phase < 0.0) {
            phase = 0.0;
        }
        counts = turns * g_rt.CountsPerRev() + phase;
    }
    if (counts > 2.0e9 || counts < -2.0e9) {
        return Refuse(eipp::kRejectOutOfRange, "jog target out of range");
    }
    g_rt.SetJogCrankCounts((int32_t)(counts < 0 ? counts - 0.5 : counts + 0.5));
    return nullptr;
}

const char *EngineJogTime(double seconds) {
    if (!g_loaded || !g_rt.UsesTimeline()) {
        return Refuse(g_loaded ? eipp::kRejectOutOfRange : eipp::kRejectNoProfile, "no timeline axes loaded");
    }
    if (seconds < 0.0) seconds = 0.0;
    if (seconds > 400.0) {
        return Refuse(eipp::kRejectOutOfRange, "jog time out of range");
    }
    g_rt.SetJogTimeQ4((int32_t)(seconds * kRtSampleHz * 16.0 + 0.5));
    return nullptr;
}

const char *EngineSimulate(bool on) {
    if (Moving()) {
        return Refuse(eipp::kRejectBusy, "stop the motion first");
    }
    if (on && g_fault) {
        /* Simulate steps no motor: a latched fault (estop, motor, HLFB on enable) does not stop it. The
         * motors stay disabled; Enable checks them again. */
        EngineClearFault();
    }
    if (!on) {
        g_simEncOn = false;  /* back to the real encoder (nothing is moving, so nothing jumps) */
        g_simEncRateQ16 = 0;
        g_simEncRate = 0.0;
    }
    if (AxisJogMoving()) {
        return Refuse(eipp::kRejectBusy, "an axis jog is moving");
    }
    g_simulate = on;
    UpdateEnabled();
    return nullptr;
}

const char *EngineSimEncoder(double countsPerSecond) {
    if (!g_simulate) {
        return "simulate only (sim 1)";
    }
    if (countsPerSecond > 2.0e6 || countsPerSecond < -2.0e6) {
        return "simenc: at most 2000000 counts/s";
    }
    if (!g_simEncOn) {
        /* Start from the real count: the follower works on changes, so nothing jumps. */
        __disable_irq();
        g_simEncQ16 = (int64_t)EncoderIn.Position() << 16;
        g_simEncCount = EncoderIn.Position();
        g_simEncOn = true;
        __enable_irq();
    }
    g_simEncRateQ16 = (int32_t)llround(countsPerSecond / kRtSampleHz * 65536.0);
    g_simEncRate = countsPerSecond;
    return nullptr;
}

/* ------------------------------------------------------------------------- */
/* Reports                                                                    */
/* ------------------------------------------------------------------------- */

static uint16_t State(const RtStatus &s) {
    if (g_fault) return eipp::kStateFault;
    if (!g_loaded) return eipp::kStateEmpty;
    if (s.crank == CrankState::Jogging || s.timeline == TimelineState::Jogging) return eipp::kStateJogging;
    if (s.crank != CrankState::Off || s.timeline == TimelineState::Running || s.timeline == TimelineState::Stopping ||
        s.timeline == TimelineState::Waiting) {
        return eipp::kStateRunning;
    }
    if (s.timeline == TimelineState::Armed) return eipp::kStateArmed;
    return (g_enabled || g_simulate) ? eipp::kStateReady : eipp::kStateDisabled;
}

/* ------------------------------------------------------------------------- */
/* Axis jog                                                                   */
/* ------------------------------------------------------------------------- */

/* By motor connector. A profile axis moves through the runtime (its position is the cam's, in its units); a motor the
 * profile doesn't use, or any motor with no profile, moves on its own generator here, in counts; a driven crank's
 * motor only enables and disables here (turning it alone would put every axis out of step with the crank: Hand jog
 * moves it). */
enum class Role { Free, Axis, Crank };

static Role RoleOf(int c, int *axis) {
    *axis = -1;
    if (!g_loaded) {
        return Role::Free;
    }
    for (int i = 0; i < g_rt.AxisCount(); ++i) {
        if (g_rt.AxisConnector(i) == c) {
            *axis = i;
            return Role::Axis;
        }
    }
    return g_rt.CrankConnector() == c ? Role::Crank : Role::Free;
}

static uint32_t g_jogMs[kMotorConnectors] = {};  /* the last velocity jog command: the deadman */

static const char *JogCheck(int c) {
    if (c < 0 || c >= kMotorConnectors) {
        return Refuse(eipp::kRejectOutOfRange, "no such motor (0..3, as M-0..M-3)");
    }
    return nullptr;
}

/* A setup move may start on this motor: the cam has nothing engaged and nothing waits to engage. */
static const char *JogMayMove(int c, bool needMotor) {
    const char *err = JogCheck(c);
    if (err) {
        return err;
    }
    if (g_fault) {
        return Refuse(eipp::kRejectFaulted, "fault latched (clear)");
    }
    if (EngineLoading() || g_committing) {
        return Refuse(eipp::kRejectBusy, "a profile upload is in progress");
    }
    RtStatus s;
    Snapshot(&s);
    if (Moving() || s.timeline == TimelineState::Armed) {
        return Refuse(eipp::kRejectBusy, "the cam is running: stop it first");
    }
    if (needMotor && !g_simulate && !g_on[c]) {
        return Refuse(eipp::kRejectNotEnabled, "the motor is disabled");
    }
    return nullptr;
}

/* A move needs a motor that moves on its own: not the crank's. */
static const char *JogMayMoveMotor(int c, int *axis) {
    const char *err = JogMayMove(c, true);
    if (err) {
        return err;
    }
    if (RoleOf(c, axis) == Role::Crank) {
        return Refuse(eipp::kRejectOutOfRange, "the crank motor moves with Hand jog (the axes follow it)");
    }
    return nullptr;
}

/* Counts per unit: the axis's, or 1 (counts) for a motor the profile doesn't use. */
static double JogCpu(int axis) { return axis >= 0 ? g_project.axes[axis].countsPerUnit : 1.0; }

/* Speeds it may jog at, in counts: the profile's limit where it has one, never past 4000 rpm of an 800-count motor. */
static double JogMaxCps(int axis) {
    if (axis < 0) {
        return ECAM_AXJOG_MAX_CPS;
    }
    const CamAxis &a = g_project.axes[axis];
    const double v = a.velMax > 0.0 ? a.velMax * fabs(a.countsPerUnit) : ECAM_AXJOG_MAX_CPS;
    return v > ECAM_AXJOG_MAX_CPS ? ECAM_AXJOG_MAX_CPS : v;
}

static double JogAccel(int axis, double cps) {
    /* The profile's acceleration limit; without one, full speed in a quarter second. */
    const double acc = axis >= 0 && g_project.axes[axis].accMax > 0.0 ? g_project.axes[axis].accMax * fabs(JogCpu(axis))
                                                                       : fabs(cps) * 4.0;
    return acc < 100.0 ? 100.0 : acc;
}

static const char *JogSpeed(int axis, double unitsPerSec, double *cps) {
    *cps = unitsPerSec * JogCpu(axis);
    if (fabs(*cps) > JogMaxCps(axis) + 1e-6) {
        static char msg[96];
        snprintf(msg, sizeof(msg), "faster than the limit (%.6g %s/s)", JogMaxCps(axis) / fabs(JogCpu(axis)),
                 axis >= 0 ? g_project.axes[axis].units.c_str() : "counts");
        return Refuse(eipp::kRejectOverLimit, msg);
    }
    return nullptr;
}

/* The motor is moving by axis jog. */
static bool JogMoving(int c) {
    int axis;
    const Role r = RoleOf(c, &axis);
    __disable_irq();
    const bool m = r == Role::Axis ? g_rt.AxisMoving(axis) : r == Role::Free ? g_free[c].Active() : false;
    __enable_irq();
    return m;
}

static int64_t JogPos(int c) {
    int axis;
    const Role r = RoleOf(c, &axis);
    RtStatus s;
    if (r == Role::Crank) {
        Snapshot(&s);
        return s.crankMotorPos;
    }
    __disable_irq();
    const int64_t p = r == Role::Axis ? g_rt.AxisPosition(axis) : g_freePos[c];
    __enable_irq();
    return p;
}

const char *EngineJogEnable(int c, bool on) {
    const char *err = JogCheck(c);
    if (err) {
        return err;
    }
    if (!on) {
        /* Disable is never refused. A motor the cam is moving: every motor, as disable (the cam can't run without it). */
        int axis;
        const Role r = RoleOf(c, &axis);
        if (Moving() && r != Role::Free) {
            g_rt.RequestAbort();
            DisableMotors();
            return nullptr;
        }
        __disable_irq();
        if (r == Role::Axis) {
            g_rt.AxisStop(axis);
            g_rt.AxisHalt();  /* its motor stops at once: the other jogs stop too, so none is left moving unseen */
            g_rt.ClearHomed((uint8_t)(1u << axis));
        }
        g_free[c].Halt();
        __enable_irq();
        kMotors[c]->EnableRequest(false);
        g_on[c] = false;
        UpdateEnabled();
        return nullptr;
    }
    if ((err = JogMayMove(c, false)) != nullptr) {
        return err;
    }
    if (EstopTripped()) {
        return Refuse(eipp::kRejectEstopOpen, EstopOpenText());
    }
    if (g_on[c]) {
        return nullptr;
    }
    kMotors[c]->ClearAlerts();
    kMotors[c]->EnableRequest(true);
    g_on[c] = true;
    if (g_settings.hlfbCheck) {
        g_committing = true;
        const uint32_t start = Milliseconds();
        while (kMotors[c]->HlfbState() != MotorDriver::HLFB_ASSERTED) {
            if (Milliseconds() - start > ECAM_ENABLE_HLFB_WAIT_MS) {
                g_committing = false;
                kMotors[c]->EnableRequest(false);
                g_on[c] = false;
                UpdateEnabled();
                static char why[96];
                snprintf(why, sizeof(why), "HLFB not asserted on M-%d (check the drive and its power, or set hlfb_check 0)", c);
                return Refuse(eipp::kRejectFaulted, why);
            }
            if (g_yield) {
                g_yield();
            }
        }
        g_committing = false;
    }
    UpdateEnabled();
    return nullptr;
}

const char *EngineJogVelocity(int c, double unitsPerSec) {
    int axis = -1;
    const char *err = JogMayMoveMotor(c, &axis);
    if (err) {
        /* A refused jog still stops one that moves (the deadman's refresh after a fault, say). */
        if (c >= 0 && c < kMotorConnectors) {
            EngineJogStop(c);
        }
        return err;
    }
    double cps = 0.0;
    if ((err = JogSpeed(axis, unitsPerSec, &cps)) != nullptr) {
        return err;
    }
    if ((err = TowardLimit(axis, cps > 0 ? 1 : cps < 0 ? -1 : 0)) != nullptr) {
        EngineJogStop(c);
        return err;
    }
    g_jogMs[c] = Milliseconds();
    __disable_irq();
    if (cps == 0.0) {
        if (axis >= 0) {
            g_rt.AxisStop(axis);
        } else {
            g_free[c].Stop();
        }
    } else if (axis >= 0) {
        g_rt.AxisJog(axis, cps, JogAccel(axis, cps));
    } else {
        g_free[c].Jog(g_freePos[c], cps, JogAccel(axis, cps));
    }
    __enable_irq();
    if (cps != 0.0 && axis >= 0) {
        g_axMoved[axis] = true;
    }
    return nullptr;
}

static const char *JogMoveCounts(int c, int axis, int64_t counts, double unitsPerSec) {
    double cps = 0.0;
    const char *err = JogSpeed(axis, unitsPerSec, &cps);
    if (err) {
        return err;
    }
    if (!(fabs(cps) > 0.0)) {
        return Refuse(eipp::kRejectOutOfRange, "speed must be above 0");
    }
    {
        const int64_t from = JogPos(c);
        if ((err = TowardLimit(axis, counts > from ? 1 : counts < from ? -1 : 0)) != nullptr) {
            return err;
        }
    }
    __disable_irq();
    if (axis >= 0) {
        g_rt.AxisMoveTo(axis, counts, fabs(cps), JogAccel(axis, cps));
    } else {
        g_free[c].MoveTo(g_freePos[c], counts, fabs(cps), JogAccel(axis, cps));
    }
    __enable_irq();
    if (axis >= 0) {
        g_axMoved[axis] = true;
    }
    return nullptr;
}

const char *EngineJogMove(int c, double units, bool absolute, double unitsPerSec) {
    int axis = -1;
    const char *err = JogMayMoveMotor(c, &axis);
    if (err) {
        return err;
    }
    if (JogMoving(c)) {
        return Refuse(eipp::kRejectBusy, "the motor is moving");
    }
    const double target = (absolute ? 0.0 : (double)JogPos(c)) + units * JogCpu(axis);
    if (target > 2.0e12 || target < -2.0e12) {
        return Refuse(eipp::kRejectOutOfRange, "move out of range");
    }
    return JogMoveCounts(c, axis, (int64_t)llround(target), unitsPerSec);
}

const char *EngineJogToCam(int c, double unitsPerSec) {
    int axis = -1;
    const char *err = JogMayMoveMotor(c, &axis);
    if (err) {
        return err;
    }
    int64_t cam = 0;
    bool fixed = false;
    if (axis >= 0) {
        __disable_irq();
        fixed = g_rt.CamPosition(axis, true, &cam);
        __enable_irq();
    }
    if (!fixed) {
        return Refuse(eipp::kRejectOutOfRange, axis < 0 ? "the profile doesn't use this motor: no cam position"
                                                        : "a gear or an open profile has no fixed cam position");
    }
    if (JogMoving(c)) {
        return Refuse(eipp::kRejectBusy, "the motor is moving");
    }
    return JogMoveCounts(c, axis, cam, unitsPerSec);
}

const char *EngineJogStop(int c) {
    if (c < 0) {
        for (int m = 0; m < kMotorConnectors; ++m) {
            EngineJogStop(m);
        }
        return nullptr;
    }
    const char *err = JogCheck(c);
    if (err) {
        return err;
    }
    int axis;
    const Role r = RoleOf(c, &axis);
    __disable_irq();
    if (r == Role::Axis) {
        g_rt.AxisStop(axis);
    }
    g_free[c].Stop();
    g_stopOn[c].active = false;
    g_stopOn[c].hit = -1;
    __enable_irq();
    return nullptr;
}

const char *EngineJogZero(int c) {
    const char *err = JogMayMove(c, false);
    if (err) {
        return err;
    }
    int axis;
    const Role r = RoleOf(c, &axis);
    if (r == Role::Crank) {
        return Refuse(eipp::kRejectOutOfRange, "the crank motor's position is the crank's (Hand jog, Crank to 0)");
    }
    if (JogMoving(c)) {
        return Refuse(eipp::kRejectBusy, "the motor is moving");
    }
    __disable_irq();
    if (r == Role::Axis) {
        g_rt.ZeroAxis(axis);
    } else {
        g_freePos[c] = 0;
    }
    __enable_irq();
    if (axis >= 0) {
        g_axMoved[axis] = true;
    }
    return nullptr;
}

/* Off its cam position by this many counts (0 when on it, or with no fixed place). An axis axis jog moved; and a
 * timeline axis whenever the timeline is at rest, since an abort, a fault or a connection loss can leave it part-way
 * through the cycle while the next Start begins at t = 0. */
static int64_t AxisOffCam(int axis, bool atStart) {
    bool check = g_axMoved[axis];
    if (!check && g_project.axes[axis].master == AxisMaster::Timeline) {
        RtStatus s;
        Snapshot(&s);
        check = s.timeline == TimelineState::Idle || s.timeline == TimelineState::Done || s.timeline == TimelineState::Armed;
    }
    if (!check) {
        return 0;
    }
    int64_t cam = 0;
    __disable_irq();
    const bool fixed = g_rt.CamPosition(axis, atStart, &cam);
    const int64_t pos = g_rt.AxisPosition(axis);
    __enable_irq();
    if (!fixed) {
        return 0;
    }
    const int64_t off = pos - cam;
    return off >= -1 && off <= 1 ? 0 : off;
}

/* Start and hand jog: no axis jog moving, and every axis axis jog moved back where the cam has it. The cam
 * engages from wherever the motors are, so an axis left elsewhere would run its whole profile shifted. */
/* The input a limit is on, by name (IO-3, CCIOA2). */
static const char *LimitInputName(int axis, int side) {
    static char name[12];
    for (int k = 0; k < g_limitCount; ++k) {
        const LimitIn &l = g_limits[k];
        if (l.axis == axis && l.side == side) {
            if (l.ccio) {
                snprintf(name, sizeof(name), "CCIO%c%d", 'A' + l.index / kCcioPinsPerBoard, l.index % kCcioPinsPerBoard);
            } else {
                snprintf(name, sizeof(name), "%s", SettingsInputName(l.index));
            }
            return name;
        }
    }
    return "limit_test";
}

/* An axis of the profile is on a limit: why Start, Hand jog and Trigger wait (not in simulate: limits only report). */
static const char *OnLimit() {
    if (g_simulate) {
        return nullptr;
    }
    const uint8_t plus = g_limPlus, minus = g_limMinus;
    for (int i = 0; i < g_rt.AxisCount() && i < (int)g_project.axes.size(); ++i) {
        const int side = (plus >> i) & 1u ? 1 : (minus >> i) & 1u ? -1 : 0;
        if (side == 0) continue;
        static char msg[128];
        const CamAxis &a = g_project.axes[i];
        snprintf(msg, sizeof(msg), "%s (M-%d) is on its %c limit (%s): jog it off first (axjog %d jog %s)", a.name.c_str(),
                 a.connector, side > 0 ? '+' : '-', LimitInputName(i, side), a.connector, side > 0 ? "-<u/s>" : "<u/s>");
        return Refuse(eipp::kRejectOnLimit, msg);
    }
    return nullptr;
}

/* An axis jog toward a tripped limit is refused (away from it is how it clears). dir: the move's direction. */
static const char *TowardLimit(int axis, int dir) {
    if (axis < 0 || g_simulate || dir == 0) {
        return nullptr;
    }
    const uint8_t bit = (uint8_t)(1u << axis);
    if ((dir > 0 && (g_limPlus & bit)) || (dir < 0 && (g_limMinus & bit))) {
        static char msg[96];
        snprintf(msg, sizeof(msg), "on its %c limit (%s): it can only move %s", dir > 0 ? '+' : '-',
                 LimitInputName(axis, dir), dir > 0 ? "-" : "+");
        return Refuse(eipp::kRejectOnLimit, msg);
    }
    return nullptr;
}

/* A limit stopped the cam: latch the fault. The motors stay enabled, holding (a vertical axis mustn't drop). */
static void LimitPoll() {
    __disable_irq();
    const uint32_t events = g_rt.LimitEvents();
    const int axis = g_rt.LimitEventAxis(), side = g_rt.LimitEventSide();
    __enable_irq();
    if (events == g_limitEventsSeen) {
        return;
    }
    g_limitEventsSeen = events;
    static char msg[96];
    const char *name = axis >= 0 && axis < (int)g_project.axes.size() ? g_project.axes[axis].name.c_str() : "?";
    snprintf(msg, sizeof(msg), "limit %c on %s (%s)", side > 0 ? '+' : '-', name, LimitInputName(axis, side));
    g_fault = msg;
    g_faultCode = eipp::kFaultLimit;
}

/* "stopon": all, or a comma list of inputs (IO-3, A-10, CCIOA2). */
static bool ParseStopOn(const char *list, uint16_t *board, uint64_t *ccio) {
    *board = 0;
    *ccio = 0;
    if (strcmp(list, "all") == 0) {
        *board = (uint16_t)((1u << kTriggerInputs) - 1u);
        const int n = CcioBoards() * kCcioPinsPerBoard;
        *ccio = n >= 64 ? ~0ull : ((1ull << n) - 1ull);
        return true;
    }
    char buf[96];
    strncpy(buf, list, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    for (char *tok = strtok(buf, ","); tok; tok = strtok(nullptr, ",")) {
        if ((tok[0] == 'C' || tok[0] == 'c') && strlen(tok) == 6 && (tok[4] >= 'A' && tok[4] <= 'H') && tok[5] >= '0' && tok[5] <= '7') {
            *ccio |= 1ull << ((tok[4] - 'A') * kCcioPinsPerBoard + (tok[5] - '0'));
            continue;
        }
        const int pin = SettingsParseInput(tok);
        if (pin < 0) {
            return false;
        }
        *board |= (uint16_t)(1u << pin);
    }
    return *board != 0 || *ccio != 0;
}

static uint16_t BoardLevelsNow() {
    uint16_t v = 0;
    for (int i = 0; i < kTriggerInputs; ++i) {
        if (TriggerConnector(i)->State()) v |= (uint16_t)(1u << i);
    }
    return v;
}

const char *EngineJogStopOn(int c, double unitsPerSec, const char *list) {
    if (c < 0 || c >= kMotorConnectors) {
        return EngineJogVelocity(c, unitsPerSec);
    }
    uint16_t board;
    uint64_t ccio;
    if (!ParseStopOn(list, &board, &ccio)) {
        return Refuse(eipp::kRejectOutOfRange, "stopon: all, or inputs such as IO-3,A-10,CCIOA2");
    }
    StopOn &s = g_stopOn[c];
    const int dir = unitsPerSec > 0 ? 1 : unitsPerSec < 0 ? -1 : 0;
    if (!s.active && s.hit >= 0 && dir == s.dir && dir != 0) {
        /* It stopped on a switch going this way: a late repeat of the held button mustn't drive it on (stop clears it). */
        return Refuse(eipp::kRejectBusy, "stopped on a switch: axjog <m> stop, or go the other way");
    }
    const char *err = EngineJogVelocity(c, unitsPerSec);
    if (err || dir == 0) {
        return err;
    }
    if (!s.active) {
        int axis;
        RoleOf(c, &axis);
        const uint16_t levels = BoardLevelsNow();
        const uint64_t ccioNow = CcioInputs();
        __disable_irq();
        s.axis = axis;
        s.board = board;
        s.ccio = ccio;
        s.boardAt = levels;
        s.ccioAt = ccioNow;
        s.hit = -1;
        s.dir = dir;
        s.active = true;
        g_stopOnAny = true;
        __enable_irq();
    }
    return nullptr;
}

const char *EngineLimitTest(int axis, int side) {
    if (axis < 0 || axis >= kRtAxes) {
        return "limit_test <axis> +|-|off";
    }
    const uint8_t bit = (uint8_t)(1u << axis);
    if (side == 0) {
        g_limTestPlus = (uint8_t)(g_limTestPlus & ~bit);
        g_limTestMinus = (uint8_t)(g_limTestMinus & ~bit);
    } else if (side > 0) {
        g_limTestPlus = (uint8_t)(g_limTestPlus | bit);
    } else {
        g_limTestMinus = (uint8_t)(g_limTestMinus | bit);
    }
    return nullptr;
}

/* Homing. */
static const char *HomeGate() {
    if (g_simulate) {
        return Refuse(eipp::kRejectBusy, "homing needs the motors and the switches: sim 0");
    }
    return nullptr;
}

const char *EngineHome(int c) {
    if (c < 0) {
        return EngineHomeAll();
    }
    int axis = -1;
    const char *err = JogMayMoveMotor(c, &axis);
    if (err) {
        return err;
    }
    if ((err = HomeGate()) != nullptr) {
        return err;
    }
    if (axis < 0) {
        return Refuse(eipp::kRejectOutOfRange, "the profile doesn't use this motor: nothing to home");
    }
    if (JogMoving(c)) {
        return Refuse(eipp::kRejectBusy, "the motor is moving");
    }
    __disable_irq();
    const bool ok = g_rt.AxisHome(axis);
    __enable_irq();
    if (!ok) {
        static char msg[96];
        snprintf(msg, sizeof(msg), "%s has no home switch (an input with the Home job)", g_project.axes[axis].name.c_str());
        return Refuse(eipp::kRejectOutOfRange, msg);
    }
    g_axMoved[axis] = true;
    return nullptr;
}

const char *EngineHomeAll() {
    if (!g_loaded) {
        return Refuse(eipp::kRejectNoProfile, "no profile loaded");
    }
    const char *err = HomeGate();
    if (err) {
        return err;
    }
    bool any = false;
    for (int i = 0; i < g_rt.AxisCount(); ++i) {
        if (!g_rt.HasHome(i)) continue;
        any = true;
        if ((err = JogMayMove(g_rt.AxisConnector(i), true)) != nullptr) {
            return err;
        }
        if (JogMoving(g_rt.AxisConnector(i))) {
            return Refuse(eipp::kRejectBusy, "an axis is moving");
        }
    }
    if (!any) {
        return Refuse(eipp::kRejectOutOfRange, "no axis has a home switch (an input with the Home job)");
    }
    __disable_irq();
    g_rt.AxisHomeAll();
    __enable_irq();
    for (int i = 0; i < g_rt.AxisCount(); ++i) {
        if (g_rt.HasHome(i)) g_axMoved[i] = true;
    }
    return nullptr;
}

const char *EngineHomeStop() {
    if (!g_loaded) {
        return nullptr;
    }
    __disable_irq();
    for (int i = 0; i < g_rt.AxisCount(); ++i) {
        if (g_rt.Homing(i) || ((g_rt.HomingMask() >> i) & 1u)) {
            g_rt.AxisStop(i);
        }
    }
    __enable_irq();
    return nullptr;
}

const char *EngineHomeTest(int axis, bool on, double units) {
    if (!g_loaded || axis < 0 || axis >= g_rt.AxisCount()) {
        return "home_test <axis> <position>|off";
    }
    const uint8_t bit = (uint8_t)(1u << axis);
    if (!on) {
        g_homeTest = (uint8_t)(g_homeTest & ~bit);
        return nullptr;
    }
    const CamAxis &a = g_project.axes[axis];
    const int c = g_rt.AxisConnector(axis);
    const int8_t sign = a.reverse ? -1 : 1;
    __disable_irq();
    /* The switch where the axis's position will read units now, fixed to the motor from here on. */
    const int64_t here = sign * g_motorSteps[c];
    g_homeTestAt[axis] = here + ((int64_t)llround(units * a.countsPerUnit) - g_rt.AxisPosition(axis));
    g_homeTestSign[axis] = sign;
    g_homeTestDir[axis] = (int8_t)((a.home.dir > 0) == (a.countsPerUnit >= 0.0) ? 1 : -1);
    g_homeTest = (uint8_t)(g_homeTest | bit);
    __enable_irq();
    return nullptr;
}

/* axhome: each profile axis's homing; axlimit: the axis jog stops at its limits (the switch wizard reads both). */
void EngineHomeList(void (*out)(const char *line)) {
    char buf[ECAM_MAX_LINE];
    for (int i = 0; g_loaded && i < g_rt.AxisCount(); ++i) {
        __disable_irq();
        const bool has = g_rt.HasHome(i), homing = g_rt.Homing(i) || ((g_rt.HomingMask() >> i) & 1u);
        const bool homed = ((g_rt.HomedMask() >> i) & 1u) != 0;
        const uint32_t ends = g_rt.AxisHomeEnds(i);
        const int64_t latch = g_rt.AxisLatch(i), pos = g_rt.AxisPosition(i);
        const CamRuntime::HomeFail r = g_rt.AxisHomeResult(i);
        __enable_irq();
        snprintf(buf, sizeof(buf), "axhome %d M-%d has=%d homed=%d homing=%d ends=%lu latch=%ld pos=%ld last=%s", i,
                 g_rt.AxisConnector(i), has ? 1 : 0, homed ? 1 : 0, homing ? 1 : 0, (unsigned long)ends, (long)latch, (long)pos,
                 r == CamRuntime::HomeFail::None ? "ok" : r == CamRuntime::HomeFail::Stopped ? "stopped"
                 : r == CamRuntime::HomeFail::Limit ? "limit" : r == CamRuntime::HomeFail::NotFound ? "notfound" : "norelease");
        out(buf);
    }
}

void EngineLimitList(void (*out)(const char *line)) {
    char buf[ECAM_MAX_LINE];
    for (int i = 0; g_loaded && i < g_rt.AxisCount(); ++i) {
        __disable_irq();
        const uint32_t tp = g_rt.AxisLimitTrips(i, 0), tm = g_rt.AxisLimitTrips(i, 1);
        const int64_t ap = g_rt.AxisLimitTripAt(i, 0), am = g_rt.AxisLimitTripAt(i, 1), pos = g_rt.AxisPosition(i);
        __enable_irq();
        snprintf(buf, sizeof(buf), "axlimit %d M-%d lim=%d%d tripp=%lu atp=%ld tripm=%lu atm=%ld pos=%ld", i, g_rt.AxisConnector(i),
                 (g_limPlus >> i) & 1u ? 1 : 0, (g_limMinus >> i) & 1u ? 1 : 0, (unsigned long)tp, (long)ap, (unsigned long)tm,
                 (long)am, (long)pos);
        out(buf);
    }
}

void EngineHomeTestList(void (*out)(const char *line)) {
    char buf[ECAM_MAX_LINE];
    for (int i = 0; g_loaded && i < g_rt.AxisCount(); ++i) {
        const int c = g_rt.AxisConnector(i);
        __disable_irq();
        const int64_t steps = g_motorSteps[c], at = g_homeTestAt[i];
        const int64_t pos = g_rt.AxisPosition(i), latch = g_rt.AxisLatch(i);
        const CamRuntime::HomeFail r = g_rt.AxisHomeResult(i);
        __enable_irq();
        snprintf(buf, sizeof(buf), "home_test %d M-%d on=%d switch=%ld motor=%ld pos=%ld latch=%ld last=%s", i, c,
                 (g_homeTest >> i) & 1u ? 1 : 0, (long)at, (long)((g_homeTestSign[i] ? g_homeTestSign[i] : 1) * steps), (long)pos,
                 (long)latch, r == CamRuntime::HomeFail::None ? "ok" : CamRuntime::HomeFailText(r));
        out(buf);
    }
}

/* Start and Trigger: every axis that requires homing has homed (not in simulate: nothing moves there). */
static const char *NotHomed() {
    if (g_simulate) {
        return nullptr;
    }
    __disable_irq();
    const uint8_t homed = g_rt.HomedMask();
    __enable_irq();
    for (int i = 0; i < g_rt.AxisCount() && i < (int)g_project.axes.size(); ++i) {
        const CamAxis &a = g_project.axes[i];
        if (!a.home.required || !g_rt.HasHome(i) || ((homed >> i) & 1u)) continue;
        static char msg[112];
        snprintf(msg, sizeof(msg), "%s (M-%d) isn't homed: home it (axhome %d, or axhome all)", a.name.c_str(), a.connector,
                 a.connector);
        return Refuse(eipp::kRejectNotHomed, msg);
    }
    return nullptr;
}

/* Operator inputs. */
static void OpConfigure() {
    int8_t pins[kOpInputs];
    for (int k = 0; k < kOpInputs; ++k) pins[k] = g_settings.opPin[k] == 0xFF ? -1 : (int8_t)g_settings.opPin[k];
    /* The panel's IO pins as inputs (a profile that used one as an output is refused while it is an operator input). */
    for (int k = 0; k < kOpInputs; ++k) {
        if (pins[k] >= 0 && pins[k] < kOutputs) kOutputConnectors[pins[k]]->Mode(Connector::INPUT_DIGITAL);
        if (pins[k] >= 0 && pins[k] >= 9) TriggerConnector(pins[k])->Mode(Connector::INPUT_DIGITAL);
    }
    __disable_irq();
    g_panel.Configure(pins, g_settings.opNc, g_settings.opMode == 1);
    __enable_irq();
}

/* A setting moving an operator input onto this pin: the estop's, another operator input's, or the loaded profile's. */
const char *EngineOpPinCheck(int input, int pin) {
    static char msg[112];
    if (pin < 0) return nullptr;
    if (g_settings.estopMode != 0 && pin == g_settings.estopPin) {
        snprintf(msg, sizeof(msg), "%s is the estop's input", SettingsInputName(pin));
        return msg;
    }
    for (int k = 0; k < kOpInputs; ++k) {
        if (k != input && g_settings.opPin[k] == pin) {
            snprintf(msg, sizeof(msg), "%s is already the operator %s input", SettingsInputName(pin), kOpLabel[k]);
            return msg;
        }
    }
    if (g_loaded) {
        if (const char *use = ProfileUse(g_project, pin)) {
            snprintf(msg, sizeof(msg), "%s is %s in the loaded profile", SettingsInputName(pin), use);
            return msg;
        }
    }
    return nullptr;
}

const char *EngineOpTest(int input, int state) {
    if (input < 0 || input >= kOpInputs) return "op_test enable|disable|reset|start|stop press|release|off";
    g_panel.Test(input, state);
    return nullptr;
}

/* The panel's buttons, as the console's commands. While the PLC has control, Enable, Fault reset and Start are its
 * (ignored here); Disable and Stop always act. Each action and any refusal goes to the USB console's log. */
static void OpPoll() {
    const uint8_t act = g_panel.Poll();
    if (act == 0) return;
    char line[120];
    auto log = [&line](const char *what, const char *err) {
        snprintf(line, sizeof(line), "op: %s%s%s", what, err ? ": refused: " : "", err ? err : "");
        TransportUsbSendLine(line);
    };
    const bool plc = EipLinkPlcInControl();
    if (act & kActStop) log("Stop", EngineStop());
    if (act & kActDisable) log("Disable", EngineDisable());
    if (act & kActReset) log("Fault reset", plc ? "the PLC has control" : EngineClearFault());
    if (act & kActEnable) log("Enable", plc ? "the PLC has control" : EngineEnable());
    if (act & kActStart) {
        const char *err = plc ? "the PLC has control" : EngineStart();
        log("Start", err);
        /* A timeline with a command trigger: a panel has no Trigger button, so Start triggers it too. */
        if (!err && g_loaded && g_rt.UsesTimeline() && g_project.master.trigger == TriggerSource::Command) {
            log("Start: trigger", EngineTrigger());
        }
    }
}

/* Timeline waits. */
void EngineSetPlcWaitBits(uint32_t bits, bool live) {
    g_plcWaitBits = bits;
    g_plcLive = live;
}

static const char *WaitText(int k) {
    static char text[40];
    if (k < 0 || k >= (int)g_project.waits.size()) {
        return "?";
    }
    const TimelineWait &w = g_project.waits[k];
    snprintf(text, sizeof(text), "%s %s", WaitInputName(w).c_str(), w.on ? "on" : "off");
    return text;
}

/* A wait timed out: latch the fault; the timeline is stopped at it, the motors stay enabled. */
static void WaitPoll() {
    __disable_irq();
    const uint32_t events = g_rt.WaitEvents();
    const int k = g_rt.WaitEventIndex();
    __enable_irq();
    if (events == g_waitEventsSeen) {
        return;
    }
    g_waitEventsSeen = events;
    static char msg[96];
    const double t = k >= 0 && k < (int)g_project.waits.size() ? g_project.waits[k].t : 0.0;
    snprintf(msg, sizeof(msg), "wait %d (%s) timed out at %.3f s", k + 1, WaitText(k), t);
    g_fault = msg;
    g_faultCode = eipp::kFaultWaitTimeout;
}

void EngineWaitList(void (*out)(const char *line)) {
    char buf[ECAM_MAX_LINE];
    const uint16_t met = g_waitMet;
    for (int k = 0; g_loaded && k < (int)g_project.waits.size() && k < g_rt.WaitCount(); ++k) {
        const TimelineWait &w = g_project.waits[k];
        __disable_irq();
        const uint32_t holds = g_rt.WaitHolds(k), timeouts = g_rt.WaitTimeouts(k);
        const bool armed = g_rt.WaitArmed(k);
        __enable_irq();
        snprintf(buf, sizeof(buf), "wait %d t=%.4f input=%s want=%s mode=%s timeout=%.3f met=%d armed=%d reached=%lu timeouts=%lu", k + 1,
                 w.t, WaitInputName(w).c_str(), w.on ? "on" : "off", w.edge ? "change" : "level", w.timeout, (met >> k) & 1u ? 1 : 0,
                 armed ? 1 : 0, (unsigned long)holds, (unsigned long)timeouts);
        out(buf);
    }
    snprintf(buf, sizeof(buf), "plc wait bits %08lx %s%s", (unsigned long)(g_plcTestOn ? g_plcTestBits : g_plcWaitBits),
             g_plcTestOn ? "live" : g_plcLive ? "live" : "no connection", g_plcTestOn ? " (waitbits_test)" : "");
    out(buf);
}

void EngineWaitBitsTest(bool on, uint32_t bits) {
    g_plcTestBits = bits;
    g_plcTestOn = on;
}

const char *EngineWaitSkip() {
    if (!g_simulate) {
        return "wait_skip works only in simulate";
    }
    RtStatus s;
    Snapshot(&s);
    if (s.timeline != TimelineState::Waiting) {
        return "no wait is holding";
    }
    g_rt.RequestWaitSkip();
    return nullptr;
}

/* A homing failed: latch the fault. The motors stay enabled, as for a limit. */
static void HomePoll() {
    __disable_irq();
    const uint32_t events = g_rt.HomeEvents();
    const int axis = g_rt.HomeEventAxis();
    const CamRuntime::HomeFail why = g_rt.HomeEventFail();
    __enable_irq();
    if (events == g_homeEventsSeen) {
        return;
    }
    g_homeEventsSeen = events;
    static char msg[112];
    const char *name = axis >= 0 && axis < (int)g_project.axes.size() ? g_project.axes[axis].name.c_str() : "?";
    snprintf(msg, sizeof(msg), "home failed on %s: %s", name, CamRuntime::HomeFailText(why));
    g_fault = msg;
    g_faultCode = eipp::kFaultHome;
}

static const char *AxisJogClear(bool atStart) {
    if (AxisJogMoving()) {
        return Refuse(eipp::kRejectBusy, "an axis jog is moving");
    }
    if (const char *lim = OnLimit()) {
        return lim;
    }
    for (int i = 0; i < g_rt.AxisCount(); ++i) {
        const int64_t off = AxisOffCam(i, atStart);
        if (off != 0) {
            static char msg[128];
            const CamAxis &a = g_project.axes[i];
            snprintf(msg, sizeof(msg), "%s (M-%d) is %.4g %s off its cam position: move it there (axjog %d cam) or zero it",
                     a.name.c_str(), a.connector, (double)off / a.countsPerUnit, a.units.c_str(), a.connector);
            return Refuse(eipp::kRejectAxisOffCam, msg);
        }
    }
    for (int i = 0; i < kRtAxes; ++i) {
        g_axMoved[i] = false;  /* the cam places them from here on */
    }
    return nullptr;
}

/* The deadman: a velocity jog needs its command again within ECAM_AXJOG_DEADMAN_MS, or it ramps to rest. */
static void AxisJogPoll() {
    for (int c = 0; c < kMotorConnectors; ++c) {
        int axis;
        const Role r = RoleOf(c, &axis);
        __disable_irq();
        const bool jog = r == Role::Axis ? g_rt.AxisVelocityJog(axis) : r == Role::Free ? g_free[c].VelocityJog() : false;
        __enable_irq();
        if (jog && Milliseconds() - g_jogMs[c] > ECAM_AXJOG_DEADMAN_MS) {
            EngineJogStop(c);
        }
    }
}

void EngineJogList(void (*out)(const char *line)) {
    char buf[ECAM_MAX_LINE];
    static const char *const kRole[] = {"free", "axis", "crank"};
    for (int c = 0; c < kMotorConnectors; ++c) {
        int axis;
        const Role r = RoleOf(c, &axis);
        int64_t cam = 0;
        bool fixed = false;
        if (r == Role::Axis) {
            __disable_irq();
            fixed = g_rt.CamPosition(axis, true, &cam);
            __enable_irq();
        }
        /* cam=- : not a profile axis, or a gear / open profile (no fixed place). Counts fit a long: the cam limits
         * them to 2^31, and a free motor's would take days at full speed to pass it. */
        char camText[16] = "-";
        if (fixed) {
            snprintf(camText, sizeof(camText), "%ld", (long)cam);
        }
        char axisText[8] = "-";
        if (axis >= 0) {
            snprintf(axisText, sizeof(axisText), "%d", axis);
        }
        const double cpu = r == Role::Crank ? 1.0 : JogCpu(axis);
        const char *units = r == Role::Axis ? g_project.axes[axis].units.c_str() : "counts";
        const char *name = r == Role::Axis ? g_project.axes[axis].name.c_str() : r == Role::Crank ? "Crank" : "";
        /* lim: + / - / +- / - (none); stopped: the input a stop on a switch stopped at, and where. */
        char limText[4] = "-";
        if (axis >= 0) {
            const bool p = ((g_limPlus >> axis) & 1u) != 0, m = ((g_limMinus >> axis) & 1u) != 0;
            snprintf(limText, sizeof(limText), "%s%s", p ? "+" : "", m ? "-" : "");
            if (!p && !m) snprintf(limText, sizeof(limText), "0");
        } else {
            snprintf(limText, sizeof(limText), "0");
        }
        char hitText[12] = "-";
        const StopOn &so = g_stopOn[c];
        if (so.hit >= 100) {
            snprintf(hitText, sizeof(hitText), "CCIO%c%d", 'A' + (so.hit - 100) / kCcioPinsPerBoard, (so.hit - 100) % kCcioPinsPerBoard);
        } else if (so.hit >= 0) {
            snprintf(hitText, sizeof(hitText), "%s", SettingsInputName(so.hit));
        }
        snprintf(buf, sizeof(buf), "axjog %d role=%s axis=%s on=%d moving=%d pos=%ld cam=%s off=%ld cpu=%.9g vmax=%.6g lim=%s stopped=%s at=%ld units=%s name=%s",
                 c, kRole[(int)r], axisText, g_on[c] ? 1 : 0, JogMoving(c) ? 1 : 0, (long)JogPos(c), camText,
                 axis >= 0 ? (long)AxisOffCam(axis, true) : 0l, cpu, r == Role::Crank ? 0.0 : JogMaxCps(axis) / fabs(cpu), limText,
                 hitText, (long)so.at, units, name);
        out(buf);
    }
}

/* Status tokens, bit c = M-c: motors on, moving by axis jog, in fault (enabled with HLFB down), HLFB asserted; and
 * bit i = axis i: off its cam position. */
static int AxisStatusTokens(char *buf, size_t len) {
    unsigned on = 0, moving = 0, off = 0, fault = 0, hlfb = 0;
    for (int c = 0; c < kMotorConnectors; ++c) {
        if (g_on[c]) on |= 1u << c;
        if (JogMoving(c)) moving |= 1u << c;
        if (MotorFaulted(kMotors[c])) fault |= 1u << c;
        /* Bipolar PWM HLFB: asserted at rest, a PWM measurement (torque) while it moves; either is up. */
        const MotorDriver::HlfbStates hs = kMotors[c]->HlfbState();
        if (hs == MotorDriver::HLFB_ASSERTED || hs == MotorDriver::HLFB_HAS_MEASUREMENT) hlfb |= 1u << c;
    }
    for (int i = 0; g_loaded && i < g_rt.AxisCount(); ++i) {
        if (AxisOffCam(i, true) != 0) off |= 1u << i;
    }
    unsigned lim = 0;
    for (int i = 0; i < kRtAxes; ++i) {
        if ((g_limPlus >> i) & 1u) lim |= 1u << (2 * i);
        if ((g_limMinus >> i) & 1u) lim |= 1u << (2 * i + 1);
    }
    unsigned homed = 0, homing = 0, hsw = 0, has = 0;
    if (g_loaded) {
        __disable_irq();
        homed = g_rt.HomedMask();
        homing = g_rt.HomingMask();
        __enable_irq();
        hsw = g_homeSw;
        for (int i = 0; i < g_rt.AxisCount(); ++i) {
            if (g_rt.HasHome(i)) has |= 1u << i;
        }
    }
    return snprintf(buf, len, " mon=%x jogmove=%x offcam=%x mfault=%x mhlfb=%x lim=%x hashome=%x homed=%x homing=%x homesw=%x", on,
                    moving, off, fault, hlfb, lim, has, homed, homing, hsw);
}

static const char *StateName(const RtStatus &s) {
    static const char *const kNames[] = {"empty", "disabled", "ready", "armed", "running", "jogging", "fault"};
    return kNames[State(s)];
}

/* The crank angle in the cycle as drawn (cut-length mode: past the stretched wait). */
static double CrankDeg(const RtStatus &s) {
    if (g_rt.LineMode()) {
        return (double)s.designPhase * 360.0 / (double)g_rt.DesignCounts();
    }
    return (double)(s.crankPhaseQ24 >> 24) * 360.0 / (double)g_rt.CountsPerRev();
}

/* mm is the finished piece: the cycle (cut to cut) is the piece plus the kerf. */
const char *EngineLength(double mm) {
    if (!g_loaded) {
        return Refuse(eipp::kRejectNoProfile, "no cut-length profile loaded");
    }
    if (!g_rt.LineMode()) {
        return Refuse(eipp::kRejectNotCutLengthMode, "no cut-length profile loaded");
    }
    const double cpm = g_rt.LineCountsPerMm();
    const double kerf = g_rt.Kerf();
    if (!(mm > 0.0) || (mm + kerf) * cpm > 2.0e9) {
        return Refuse(eipp::kRejectOutOfRange, "length out of range");
    }
    const int64_t counts = (int64_t)llround((mm + kerf) * cpm);
    if (counts < g_rt.MinCutCounts()) {
        static char msg[80];
        snprintf(msg, sizeof(msg), "shortest piece is %.3f mm (the cycle's fixed part less the kerf)",
                 g_rt.MinCutCounts() / cpm - kerf);
        return Refuse(eipp::kRejectLengthTooShort, msg);
    }
    return g_rt.SetCutCounts(counts);
}

static double CrankRpm(const RtStatus &s) {
    /* The cycle as drawn, as `rpm` is set: in cut-length mode that is the line speed at any cut length. */
    return (double)s.crankVelQ24 / 16777216.0 * kRtSampleHz * 60.0 / (double)g_rt.DesignCounts();
}

bool EngineSafeForNetworkChange() {
    if (AnyOn() || AxisJogMoving() || EngineLoading() || g_committing) {
        return false;
    }
    if (!g_loaded) {
        return true;
    }
    RtStatus s;
    Snapshot(&s);
    return s.crank == CrankState::Off && (s.timeline == TimelineState::Idle || s.timeline == TimelineState::Done);
}

void EngineStatus(char *buf, size_t len) {
    RtStatus s;
    Snapshot(&s);
    int n = snprintf(buf, len,
                     "state=%s enabled=%d sim=%d fault=%s estop=%s a9=%d profile=%08lx stored=%d axes=%d crank=%s tl=%s "
                     "deg=%.2f rev=%ld rpm=%.2f t=%.4f cycle=%ld ovr=%.3f enc=%ld out=%u",
                     StateName(s), g_enabled ? 1 : 0, g_simulate ? 1 : 0, g_fault ? g_fault : "none",
                     g_settings.estopMode == 0 ? "off" : EstopTripped() ? "open" : "ok", ConnectorA9.State() ? 1 : 0,
                     g_loaded ? (unsigned long)g_hash : 0ul, g_stored ? 1 : 0,
                     g_loaded ? g_rt.AxisCount() : 0, CrankStateName(s.crank), TimelineStateName(s.timeline),
                     CrankDeg(s), (long)s.crankRev, CrankRpm(s), (double)s.timeQ16 / 65536.0 / kRtSampleHz,
                     (long)s.cycle, s.rateQ16 / 65536.0, (long)EncoderCount(), (unsigned)s.outputs);
    if (s.endOfCycle && n > 0 && (size_t)n < len) {
        n += snprintf(buf + n, len - n, " endcycle=1");
    }
    if (s.waitIndex > 0 && n > 0 && (size_t)n < len) {
        n += snprintf(buf + n, len - n, " wait=%d waited=%lu", s.waitIndex, (unsigned long)(s.waitedSamples / (kRtSampleHz / 1000)));
    }
    if (n > 0 && (size_t)n < len) {
        /* The estop's input, -1 while the estop is off. */
        n += snprintf(buf + n, len - n, " epin=%d", g_settings.estopMode == 0 ? -1 : (int)g_settings.estopPin);
    }
    if (n > 0 && (size_t)n < len) {
        /* Operator inputs: pressed now (bit n: Enable, Disable, Fault reset, Start, Stop), each one's input (-1 none), the
         * contacts (bit n NC) and the Enable mode. */
        int p[kOpInputs];
        for (int k = 0; k < kOpInputs; ++k) p[k] = g_settings.opPin[k] == 0xFF ? -1 : g_settings.opPin[k];
        n += snprintf(buf + n, len - n, " ops=%x oppins=%d,%d,%d,%d,%d opnc=%x opmode=%d", (unsigned)g_panel.Pressed(), p[0], p[1], p[2], p[3],
                      p[4], (unsigned)g_settings.opNc, (int)g_settings.opMode);
    }
    if (n > 0 && (size_t)n < len) {
        /* The board's own inputs, bit n = IO-0..IO-5, DI-6..DI-8, A-9..A-12 (an IO pin the profile drives reads as its
         * output). */
        unsigned din = 0;
        for (int i = 0; i < kTriggerInputs; ++i) {
            if (TriggerConnector(i)->State()) din |= 1u << i;
        }
        n += snprintf(buf + n, len - n, " din=%x", din);
    }
    if (g_simEncOn && n > 0 && (size_t)n < len) {
        n += snprintf(buf + n, len - n, " simenc=%.0f", g_simEncRate);
    }
    if (g_loaded && g_rt.LineMode() && n > 0 && (size_t)n < len) {
        const double cpm = g_rt.LineCountsPerMm();
        const double kerf = g_rt.Kerf();  /* lengths are finished pieces: cycle less kerf */
        n += snprintf(buf + n, len - n, " len=%.3f next=%.3f lmin=%.3f kerf=%.3f", s.cycleUnits / cpm - kerf,
                      s.pendingUnits / cpm - kerf, g_rt.MinCutCounts() / cpm - kerf, kerf);
        if (s.hasCutGap && n > 0 && (size_t)n < len) {
            /* gap: blade to the next cut line (mm of line, 0 in the sync); sync: line left in the sync */
            n += snprintf(buf + n, len - n, " gap=%.2f sync=%.2f", s.cutGapUnits / cpm, s.inSync ? s.syncLeftUnits / cpm : -1.0);
        }
    }
    for (int i = 0; g_loaded && i < g_rt.AxisCount() && n > 0 && (size_t)n < len; ++i) {
        n += snprintf(buf + n, len - n, " a%d=%ld", i, (long)s.axisPos[i]);
    }
    if (n > 0 && (size_t)n < len) {
        n += AxisStatusTokens(buf + n, len - n);
    }
    if (CcioBoards() != 0 && n > 0 && (size_t)n < len) {
        const uint64_t in = CcioInputs();
        n += snprintf(buf + n, len - n, " ccio=%u ccio_link=%s ccio_in=%08lx%08lx", (unsigned)CcioBoards(),
                      CcioLinkName(), (unsigned long)(in >> 32), (unsigned long)(in & 0xFFFFFFFFu));
    }
}

void EngineTelemetry(char *buf, size_t len) {
    RtStatus s;
    Snapshot(&s);
    int n = snprintf(buf, len, "TEL ms=%lu state=%s crank=%s tl=%s deg=%.3f rev=%ld rpm=%.2f t=%.4f cycle=%ld ovr=%.3f out=%u enc=%ld",
                     (unsigned long)Milliseconds(), StateName(s), CrankStateName(s.crank),
                     TimelineStateName(s.timeline), CrankDeg(s), (long)s.crankRev, CrankRpm(s),
                     (double)s.timeQ16 / 65536.0 / kRtSampleHz, (long)s.cycle, s.rateQ16 / 65536.0,
                     (unsigned)s.outputs, (long)EncoderCount());
    if (g_loaded && s.hasCutGap && n > 0 && (size_t)n < len) {
        const double cpm = g_rt.LineCountsPerMm();
        n += snprintf(buf + n, len - n, " gap=%.2f sync=%.2f", s.cutGapUnits / cpm, s.inSync ? s.syncLeftUnits / cpm : -1.0);
    }
    if (s.waitIndex > 0 && n > 0 && (size_t)n < len) {
        n += snprintf(buf + n, len - n, " wait=%d waited=%lu", s.waitIndex, (unsigned long)(s.waitedSamples / (kRtSampleHz / 1000)));
    }
    for (int i = 0; g_loaded && i < g_rt.AxisCount() && n > 0 && (size_t)n < len; ++i) {
        n += snprintf(buf + n, len - n, " a%d=%ld", i, (long)s.axisPos[i]);
    }
}

void EngineEipStatus(eipp::Status *st) {
    RtStatus s;
    Snapshot(&s);
    *st = eipp::Status();
    const uint16_t state = State(s);
    const bool estopOpen = EstopTripped();
    uint16_t f = 0;
    if (g_loaded) f |= eipp::kFlagProfileLoaded;
    if (g_stored) f |= eipp::kFlagProfileStored;
    if (g_enabled) f |= eipp::kFlagEnabled;
    if (g_loaded && !g_fault && (g_enabled || g_simulate)) f |= eipp::kFlagReady;
    if (s.crank == CrankState::Following || s.crank == CrankState::Running || s.crank == CrankState::Stopping ||
        s.timeline == TimelineState::Running || s.timeline == TimelineState::Stopping || s.timeline == TimelineState::Waiting) {
        f |= eipp::kFlagRunning;
    }
    if (s.timeline == TimelineState::Waiting) f |= eipp::kFlagWaiting;
    if (s.timeline == TimelineState::Armed) f |= eipp::kFlagTimelineArmed;
    if (s.timeline == TimelineState::Done) f |= eipp::kFlagTimelineDone;
    if (s.crank == CrankState::Stopping || s.timeline == TimelineState::Stopping || s.endOfCycle) f |= eipp::kFlagStopping;
    if (state == eipp::kStateJogging) f |= eipp::kFlagJogging;
    if (g_fault) f |= eipp::kFlagFaulted;
    if (estopOpen) f |= eipp::kFlagEstopOpen;
    if (g_simulate) f |= eipp::kFlagSimulate;
    st->flags = f;
    st->firmwareRev = (uint16_t)(ECAM_FIRMWARE_MAJOR * 256u + ECAM_FIRMWARE_MINOR);
    st->faultCode = g_fault ? g_faultCode : 0;
    st->engineState = state;
    st->crankState = (uint16_t)s.crank;
    st->timelineState = (uint16_t)s.timeline;
    st->outputs = s.outputs;
    st->profileHash = g_loaded ? g_hash : 0;
    st->heartbeat = Milliseconds();
    st->encoderCount = EncoderCount();
    Connector *const trig = g_triggerInput;
    st->inputs = (uint16_t)((trig && trig->State() ? eipp::kInTrigger : 0) | (estopOpen ? 0 : eipp::kInEstopOk));
    if (!g_loaded) {
        return;
    }
    st->crankAngle = eipp::Scale(CrankDeg(s), 1000.0);
    st->crankRev = s.crankRev;
    st->crankRpm = eipp::Scale(CrankRpm(s), 100.0);
    st->timelineTime = eipp::Scale((double)s.timeQ16 / 65536.0, 1.0e6 / kRtSampleHz);
    st->timelineCycle = s.cycle;
    st->timelineOverride = eipp::Scale(s.rateQ16 / 65536.0, 10000.0);
    st->waitIndex = (uint16_t)s.waitIndex;
    st->waitedMs = s.waitedSamples / (kRtSampleHz / 1000);
    st->axisCount = (uint16_t)g_rt.AxisCount();
    if (g_rt.LineMode()) {
        const double cpm = g_rt.LineCountsPerMm();
        const double kerf = g_rt.Kerf();
        st->cutLength = eipp::Scale(s.cycleUnits / cpm - kerf, 1000.0);
        st->cutLengthNext = eipp::Scale(s.pendingUnits / cpm - kerf, 1000.0);
        st->cutLengthMin = eipp::Scale(g_rt.MinCutCounts() / cpm - kerf, 1000.0);
        if (s.hasCutGap) {
            st->cutGap = eipp::Scale(s.cutGapUnits / cpm, 1000.0);
            st->syncLeft = s.inSync ? eipp::Scale(s.syncLeftUnits / cpm, 1000.0) : -1;
        }
    }
    __disable_irq();
    const uint8_t homedMask = g_rt.HomedMask(), homingMask = g_rt.HomingMask();
    __enable_irq();
    for (int i = 0; i < g_rt.AxisCount() && i < eipp::kStsAxes && i < (int)g_project.axes.size(); ++i) {
        const CamAxis &a = g_project.axes[i];
        const int c = g_rt.AxisConnector(i);
        eipp::AxisStatus &ax = st->axes[i];
        ax.commandedCounts = (int32_t)s.axisPos[i];
        ax.commandedTravel = a.countsPerUnit != 0.0 ? eipp::Scale((double)s.axisPos[i] / a.countsPerUnit, 1000.0) : 0;
        ax.connector = (uint16_t)c;
        uint16_t af = eipp::kAxUsed;
        if (a.master == AxisMaster::Timeline) af |= eipp::kAxTimeline;
        if (a.gear) af |= eipp::kAxGear;
        if (c >= 0 && c < kMotorConnectors) {
            MotorDriver *m = kMotors[c];
            if (g_on[c] && m->EnableRequest()) af |= eipp::kAxEnabled;
            if (m->HlfbState() == MotorDriver::HLFB_ASSERTED) af |= eipp::kAxHlfb;
            if (MotorFaulted(m)) af |= eipp::kAxMotorFault;
            if (m->AlertReg().reg != 0) af |= eipp::kAxAlerts;
        }
        if ((g_limPlus >> i) & 1u) af |= eipp::kAxLimitPlus;
        if ((g_limMinus >> i) & 1u) af |= eipp::kAxLimitMinus;
        if ((homedMask >> i) & 1u) af |= eipp::kAxHomed;
        if ((homingMask >> i) & 1u) af |= eipp::kAxHoming;
        ax.flags = af;
    }
}

void EngineProfile(void (*out)(const char *line)) {
    char buf[ECAM_MAX_LINE];
    if (!g_loaded) {
        out("no profile loaded");
        return;
    }
    const CamMaster &m = g_project.master;
    snprintf(buf, sizeof(buf), "profile %08lx: crank %s %.0f counts/rev %.2f rpm; timeline %s %.0f %% trigger %s",
             (unsigned long)g_hash, m.source == MasterSource::DrivenCrank ? "driven" : "encoder", m.countsPerRev,
             m.rpm, m.timelineRepeat ? "repeat" : "one-shot", m.speedOverride * 100.0,
             m.trigger == TriggerSource::Command        ? "command"
             : m.trigger == TriggerSource::EtherNetIP   ? "ethernet/ip (TRIGGER bit)"
                                                        : kTriggerNames[m.triggerInput]);
    out(buf);
    for (size_t i = 0; i < g_project.axes.size(); ++i) {
        const CamAxis &a = g_project.axes[i];
        if (a.gear) {
            snprintf(buf, sizeof(buf), "a%u M-%d crank %s: electronic gear %d:%d %s per crank rev, %.6g counts/%s", (unsigned)i,
                     a.connector, a.name.c_str(), a.gearNum, a.gearDen, a.units.c_str(), a.countsPerUnit, a.units.c_str());
            out(buf);
            continue;
        }
        snprintf(buf, sizeof(buf), "a%u M-%d %s %s: %u segments, span %.4g, %.6g counts/%s%s", (unsigned)i,
                 a.connector, a.master == AxisMaster::Timeline ? "timeline" : "crank", a.name.c_str(),
                 (unsigned)a.segments.size(), a.span, a.countsPerUnit, a.units.c_str(), a.closed ? "" : ", open");
        out(buf);
    }
    for (const OutputPulse &o : g_project.outputs) {
        const bool crank = o.master == AxisMaster::Crank;
        snprintf(buf, sizeof(buf), "out IO-%d %s on %.4g%s, off %.4g%s", o.output, crank ? "crank" : "timeline", o.on,
                 crank ? " deg" : " s", o.off, crank ? " deg" : " s");
        out(buf);
    }
}

void EngineMotors(void (*out)(const char *line)) {
    static const char *const kHlfb[] = {"deasserted", "asserted", "pwm", "unknown"};
    static const char *const kReady[] = {"disabled", "enabling", "faulted", "ready", "moving", "?", "?", "?"};
    char buf[ECAM_MAX_LINE];
    for (int c = 0; c < kMotorConnectors; ++c) {
        MotorDriver *m = kMotors[c];
        const unsigned hs = (unsigned)m->HlfbState() & 3u;
        snprintf(buf, sizeof(buf), "M-%d used=%d enable_req=%d hlfb=%s duty=%.1f%% ready=%s in_fault=%u status=0x%08lx alerts=0x%08lx",
                 c, g_used[c] ? 1 : 0, m->EnableRequest() ? 1 : 0, kHlfb[hs], (double)m->HlfbPercent(),
                 kReady[(unsigned)m->StatusReg().bit.ReadyState & 7u], (unsigned)m->StatusReg().bit.MotorInFault,
                 (unsigned long)m->StatusReg().reg, (unsigned long)m->AlertReg().reg);
        out(buf);
    }
}

void EngineDiag(char *buf, size_t len) {
    const uint32_t perUs = SystemCoreClock / 1000000u;
    unsigned long waits = 0;
    for (int c = 0; c < kMotorConnectors; ++c) {
        waits += kMotors[c]->ReversalWaits();
    }
    snprintf(buf, len,
             "sample callback max %lu us (of 200), carried-step samples %lu, sample interval max %lu us, "
             "late samples %lu, reversals held one sample %lu, last commit: check %lu build %lu store %lu ms",
             (unsigned long)(g_isrMaxCycles / perUs), (unsigned long)g_carryEvents,
             (unsigned long)(g_maxGapCycles / perUs), (unsigned long)g_lateSamples, waits,
             (unsigned long)(g_commitUs[0] / 1000u), (unsigned long)(g_commitUs[1] / 1000u),
             (unsigned long)(g_commitUs[2] / 1000u));
}
