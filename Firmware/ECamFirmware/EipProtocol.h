/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef __ECAM_EIP_PROTOCOL_H__
#define __ECAM_EIP_PROTOCOL_H__

/*
 * The EtherNet/IP assembly layout: the source of truth for the tables in Firmware/PROTOCOL.md.
 * Pure C++ (no ClearCore headers), so the host test (tests/test_eip_protocol.cpp) packs
 * the same bytes as the firmware.
 *
 * Little-endian; every DINT on a 4-byte boundary; scaled integers only; fixed sizes whatever the profile.
 */

#include <stddef.h>
#include <stdint.h>

namespace eipp {

/* ---- ECamCmd, instance 150 (O->T, 64 bytes) ------------------------------------------------------- */

constexpr uint16_t kCmdSize = 64;

enum CmdOffset : uint16_t {
    kCmdControlFlags = 0,     /* UINT  CtlFlag bits */
    /* 2 UINT reserved */
    kCmdCrankRpm = 4,         /* DINT  0.01 rpm, 0 = the profile's */
    kCmdTimelineOverride = 8, /* DINT  0.01 % (10000 = as drawn), 0 = the profile's */
    kCmdCutLength = 12,       /* DINT  um, finished piece, 0 = keep the current length */
    kCmdJogCrankTarget = 16,  /* DINT  millidegrees since the load (rev * 360000 + mdeg), followed while JOG */
    kCmdJogTimeTarget = 20,   /* DINT  us of timeline time, followed while JOG */
    kCmdWaitBits = 24,        /* UDINT bit n = PLC wait bit n: levels for the timeline's waits (0.10.0) */
    /* 28..63 reserved */
};

enum CtlFlag : uint16_t {
    kCtlEnable = 1u << 0,      /* level: rising enables, falling disables (falling always works) */
    kCtlPlcControl = 1u << 1,  /* level: the PLC takes control */
    kCtlStart = 1u << 2,       /* rising edge */
    kCtlStop = 1u << 3,        /* rising edge, always works */
    kCtlAbort = 1u << 4,       /* rising edge, always works */
    kCtlClearFault = 1u << 5,  /* rising edge */
    kCtlSimulate = 1u << 6,    /* level: rising sim 1, falling sim 0 */
    kCtlJog = 1u << 7,         /* level: rising jog on, falling jog off (always); targets followed as they change */
    kCtlTrigger = 1u << 8,     /* rising edge: start the timeline (profile trigger EtherNet/IP) */
    kCtlHome = 1u << 9,        /* rising edge: home every axis with a home switch, in order (0.9.1) */
};

/* ---- ECamSts, instance 100 (T->O, 160 bytes) ------------------------------------------------------ */

constexpr uint16_t kStsSize = 160;
constexpr int kStsAxes = 4;
constexpr uint16_t kStsAxisBase = 96;
constexpr uint16_t kStsAxisSize = 16;

/* Byte offsets. */
enum StsOffset : uint16_t {
    kStsFirmwareRev = 0,     /* UINT  major * 256 + minor */
    kStsStatusFlags = 2,     /* UINT  StsFlag bits */
    kStsFaultCode = 4,       /* UINT  FaultCode */
    kStsRejectCode = 6,      /* UINT  RejectCode */
    kStsEngineState = 8,     /* UINT  EngineState */
    kStsCrankState = 10,     /* UINT  0 off, 1 following, 2 running, 3 stopping, 4 jogging */
    kStsTimelineState = 12,  /* UINT  0 idle, 1 armed, 2 running, 3 stopping, 4 done, 5 jogging */
    kStsOutputs = 14,        /* UINT  IO-0..IO-5 the profile has on */
    kStsProfileHash = 16,    /* UDINT 0 = none */
    kStsHeartbeat = 20,      /* UDINT ms since boot */
    kStsCrankAngle = 24,     /* DINT  millidegrees in the cycle as drawn */
    kStsCrankRev = 28,       /* DINT  whole revolutions since the load */
    kStsCrankRpm = 32,       /* DINT  0.01 rpm */
    kStsTimelineTime = 36,   /* DINT  us into the cycle */
    kStsTimelineCycle = 40,  /* DINT  completed cycles */
    kStsTimelineOverride = 44, /* DINT 0.01 % (10000 = as drawn) */
    kStsEncoderCount = 48,   /* DINT  raw, or the simulated encoder */
    kStsAxisCount = 52,      /* UINT */
    kStsInputs = 54,         /* UINT  StsInput bits */
    kStsCutLength = 56,      /* DINT  um, finished piece now (cut-length mode, else 0) */
    kStsCutLengthNext = 60,  /* DINT  um, from the next cut */
    kStsCutLengthMin = 64,   /* DINT  um, shortest piece */
    kStsCutGap = 68,         /* DINT  um of line from the blade to the next cut line (0 in the sync) */
    kStsSyncLeft = 72,       /* DINT  um of line left in the sync, -1 outside it */
    kStsCommLossCount = 76,  /* UDINT connection losses since boot */
    kStsWaitIndex = 80,      /* UINT  the timeline wait holding, 1-based, 0 none (0.10.0) */
    /* 82 UINT reserved */
    kStsWaitedMs = 84,       /* UDINT how long it has held, ms */
    /* 88..95 reserved */
};

/* AxisStatus[i] at kStsAxisBase + kStsAxisSize * i. */
enum StsAxisOffset : uint16_t {
    kAxCommandedCounts = 0,  /* DINT  motor counts since the load (as a<i>= in status) */
    kAxCommandedTravel = 4,  /* DINT  commanded travel since the load, milli-units (mm or deg x 1000) */
    kAxFlags = 8,            /* UINT  AxFlag bits */
    kAxConnector = 10,       /* UINT  M-n */
    /* +12 DINT reserved */
};

enum StsFlag : uint16_t {
    kFlagProfileLoaded = 1u << 0,
    kFlagProfileStored = 1u << 1,
    kFlagEnabled = 1u << 2,
    kFlagReady = 1u << 3,          /* enabled or simulating, profile loaded, no fault */
    kFlagRunning = 1u << 4,        /* crank engaged or timeline running / stopping */
    kFlagTimelineArmed = 1u << 5,
    kFlagTimelineDone = 1u << 6,
    kFlagStopping = 1u << 7,
    kFlagJogging = 1u << 8,
    kFlagFaulted = 1u << 9,
    kFlagEstopOpen = 1u << 10,
    kFlagSimulate = 1u << 11,
    kFlagPlcInControl = 1u << 12,
    kFlagRejected = 1u << 13,
    kFlagWaiting = 1u << 14,       /* the timeline holds at a wait (0.10.0) */
};

enum StsInput : uint16_t {
    kInTrigger = 1u << 0,  /* the profile's trigger input is on */
    kInEstopOk = 1u << 1,  /* the estop input is closed (or estop_a9 0) */
};

enum AxFlag : uint16_t {
    kAxUsed = 1u << 0,
    kAxTimeline = 1u << 1,  /* else the crank */
    kAxEnabled = 1u << 2,
    kAxHlfb = 1u << 3,      /* HLFB asserted */
    kAxMotorFault = 1u << 4,
    kAxAlerts = 1u << 5,
    kAxGear = 1u << 6,      /* an electronic gear */
    kAxLimitPlus = 1u << 7,  /* its + limit is tripped */
    kAxLimitMinus = 1u << 8, /* its - limit is tripped */
    kAxHomed = 1u << 9,      /* homed since it was last enabled */
    kAxHoming = 1u << 10,    /* homing, or waiting its turn in a Home all */
};

enum EngineState : uint16_t {
    kStateEmpty = 0,
    kStateDisabled = 1,
    kStateReady = 2,
    kStateArmed = 3,
    kStateRunning = 4,
    kStateJogging = 5,
    kStateFault = 6,
};

/* Latched: the motion stops and the motors disable until CLEAR_FAULT. */
enum FaultCode : uint16_t {
    kFaultNone = 0,
    kFaultEstop = 1,
    kFaultMotor = 2,
    kFaultHlfbTimeout = 3,
    kFaultCommLoss = 4,
    kFaultLimit = 5,        /* a limit stopped the cam (a controlled stop; the motors stay enabled) */
    kFaultHome = 6,         /* homing failed (a limit, the switch not found, not released); the motors stay enabled */
    kFaultWaitTimeout = 7,  /* a timeline wait timed out: stopped at it, the motors enabled (0.10.0) */
};

/* Why the last edge or setpoint was refused (nothing happened); 0 once one is accepted. */
enum RejectCode : uint16_t {
    kRejectNone = 0,
    kRejectNoProfile = 10,
    kRejectNotInControl = 11,  /* the edge needs PLC_CONTROL */
    kRejectNotEnabled = 12,
    kRejectBusy = 13,
    kRejectOutOfRange = 14,
    kRejectNotDrivenCrank = 15,
    kRejectNotCutLengthMode = 16,
    kRejectLengthTooShort = 17,
    kRejectOverLimit = 18,
    kRejectFaulted = 19,       /* a fault is latched (CLEAR_FAULT), or enabling latched one (HLFB) */
    kRejectEstopOpen = 20,     /* enable refused: the estop input is open */
    kRejectNotEipTrigger = 21, /* TRIGGER: the profile's timeline trigger is not EtherNet/IP */
    kRejectAxisOffCam = 22,    /* START: an axis moved by axis jog is not where the cam has it */
    kRejectNotHomed = 23,      /* START, TRIGGER: an axis that requires homing isn't homed */
    kRejectOnLimit = 24,       /* START, JOG, TRIGGER: an axis is on a limit (jog it off first) */
};

struct AxisStatus {
    int32_t commandedCounts = 0;
    int32_t commandedTravel = 0;
    uint16_t flags = 0;
    uint16_t connector = 0;
};

/* The status in assembly units; Pack() lays it out. */
struct Status {
    uint16_t firmwareRev = 0;
    uint16_t flags = 0;
    uint16_t faultCode = 0;
    uint16_t rejectCode = 0;
    uint16_t engineState = 0;
    uint16_t crankState = 0;
    uint16_t timelineState = 0;
    uint16_t outputs = 0;
    uint32_t profileHash = 0;
    uint32_t heartbeat = 0;
    int32_t crankAngle = 0;
    int32_t crankRev = 0;
    int32_t crankRpm = 0;
    int32_t timelineTime = 0;
    int32_t timelineCycle = 0;
    int32_t timelineOverride = 0;
    int32_t encoderCount = 0;
    uint16_t axisCount = 0;
    uint16_t inputs = 0;
    int32_t cutLength = 0;
    int32_t cutLengthNext = 0;
    int32_t cutLengthMin = 0;
    int32_t cutGap = 0;
    int32_t syncLeft = -1;
    uint32_t commLossCount = 0;
    uint16_t waitIndex = 0;
    uint32_t waitedMs = 0;
    AxisStatus axes[kStsAxes];
};

inline void Put16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}
inline void Put32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
inline uint16_t Get16(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}
inline uint32_t Get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* out: kStsSize bytes. Reserved bytes are zero. */
inline void Pack(const Status &s, uint8_t *out) {
    for (uint16_t i = 0; i < kStsSize; ++i) {
        out[i] = 0;
    }
    Put16(out + kStsFirmwareRev, s.firmwareRev);
    Put16(out + kStsStatusFlags, s.flags);
    Put16(out + kStsFaultCode, s.faultCode);
    Put16(out + kStsRejectCode, s.rejectCode);
    Put16(out + kStsEngineState, s.engineState);
    Put16(out + kStsCrankState, s.crankState);
    Put16(out + kStsTimelineState, s.timelineState);
    Put16(out + kStsOutputs, s.outputs);
    Put32(out + kStsProfileHash, s.profileHash);
    Put32(out + kStsHeartbeat, s.heartbeat);
    Put32(out + kStsCrankAngle, (uint32_t)s.crankAngle);
    Put32(out + kStsCrankRev, (uint32_t)s.crankRev);
    Put32(out + kStsCrankRpm, (uint32_t)s.crankRpm);
    Put32(out + kStsTimelineTime, (uint32_t)s.timelineTime);
    Put32(out + kStsTimelineCycle, (uint32_t)s.timelineCycle);
    Put32(out + kStsTimelineOverride, (uint32_t)s.timelineOverride);
    Put32(out + kStsEncoderCount, (uint32_t)s.encoderCount);
    Put16(out + kStsAxisCount, s.axisCount);
    Put16(out + kStsInputs, s.inputs);
    Put32(out + kStsCutLength, (uint32_t)s.cutLength);
    Put32(out + kStsCutLengthNext, (uint32_t)s.cutLengthNext);
    Put32(out + kStsCutLengthMin, (uint32_t)s.cutLengthMin);
    Put32(out + kStsCutGap, (uint32_t)s.cutGap);
    Put32(out + kStsSyncLeft, (uint32_t)s.syncLeft);
    Put32(out + kStsCommLossCount, s.commLossCount);
    Put16(out + kStsWaitIndex, s.waitIndex);
    Put32(out + kStsWaitedMs, s.waitedMs);
    for (int i = 0; i < kStsAxes; ++i) {
        uint8_t *a = out + kStsAxisBase + kStsAxisSize * i;
        Put32(a + kAxCommandedCounts, (uint32_t)s.axes[i].commandedCounts);
        Put32(a + kAxCommandedTravel, (uint32_t)s.axes[i].commandedTravel);
        Put16(a + kAxFlags, s.axes[i].flags);
        Put16(a + kAxConnector, s.axes[i].connector);
    }
}

/* Scaling helpers: round to the nearest unit and saturate at the DINT range. */
inline int32_t Scale(double v, double unitsPerValue) {
    const double x = v * unitsPerValue;
    if (!(x == x)) {
        return 0; /* NaN */
    }
    if (x >= 2147483647.0) {
        return 2147483647;
    }
    if (x <= -2147483648.0) {
        return (int32_t)(-2147483647 - 1);
    }
    return (int32_t)(x < 0 ? x - 0.5 : x + 0.5);
}

} // namespace eipp

static_assert(eipp::kStsCommLossCount + 4 <= 80, "status header overlaps the reserved bytes");
static_assert(eipp::kStsAxisBase + eipp::kStsAxes * eipp::kStsAxisSize == eipp::kStsSize, "status size");

#endif
