// MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
//
// The EtherNet/IP status assembly (Firmware/ECamFirmware/EipProtocol.h) packed on the PC: offsets as the
// table in Firmware/PROTOCOL.md (Status assembly 100) gives them, little-endian, reserved bytes zero, the scaling.

#include "EipProtocol.h"

#include <cstdio>
#include <cstring>

static int g_fail = 0;
static int g_pass = 0;

static void Check(bool ok, const char *what) {
    if (ok) {
        ++g_pass;
    } else {
        ++g_fail;
        std::printf("FAIL %s\n", what);
    }
}

static uint32_t U32(const uint8_t *b, int off) {
    return (uint32_t)b[off] | ((uint32_t)b[off + 1] << 8) | ((uint32_t)b[off + 2] << 16) | ((uint32_t)b[off + 3] << 24);
}
static uint16_t U16(const uint8_t *b, int off) {
    return (uint16_t)(b[off] | (b[off + 1] << 8));
}

int main() {
    using namespace eipp;

    // Offsets written out as numbers, as in the plan, so a change to the header shows here.
    Check(kStsSize == 160, "size 160");
    Check(kStsFirmwareRev == 0 && kStsStatusFlags == 2 && kStsFaultCode == 4 && kStsRejectCode == 6, "header UINTs");
    Check(kStsEngineState == 8 && kStsCrankState == 10 && kStsTimelineState == 12 && kStsOutputs == 14, "state UINTs");
    Check(kStsProfileHash == 16 && kStsHeartbeat == 20 && kStsCrankAngle == 24 && kStsCrankRev == 28, "16..31");
    Check(kStsCrankRpm == 32 && kStsTimelineTime == 36 && kStsTimelineCycle == 40 && kStsTimelineOverride == 44, "32..47");
    Check(kStsEncoderCount == 48 && kStsAxisCount == 52 && kStsInputs == 54, "48..55");
    Check(kStsCutLength == 56 && kStsCutLengthNext == 60 && kStsCutLengthMin == 64 && kStsCutGap == 68, "56..71");
    Check(kStsSyncLeft == 72 && kStsCommLossCount == 76, "72..79");
    Check(kStsAxisBase == 96 && kStsAxisSize == 16, "axis blocks at 96, 16 bytes");
    Check(kAxCommandedCounts == 0 && kAxCommandedTravel == 4 && kAxFlags == 8 && kAxConnector == 10, "axis offsets");
    // Every DINT on a 4-byte boundary.
    const int dints[] = {kStsProfileHash, kStsHeartbeat, kStsCrankAngle, kStsCrankRev, kStsCrankRpm,
                         kStsTimelineTime, kStsTimelineCycle, kStsTimelineOverride, kStsEncoderCount, kStsCutLength,
                         kStsCutLengthNext, kStsCutLengthMin, kStsCutGap, kStsSyncLeft, kStsCommLossCount};
    bool aligned = true;
    for (int d : dints) aligned = aligned && d % 4 == 0;
    Check(aligned, "DINTs aligned");

    // ECamCmd (plan 4.1).
    Check(kCmdSize == 64, "command size 64");
    Check(kCmdControlFlags == 0 && kCmdCrankRpm == 4 && kCmdTimelineOverride == 8 && kCmdCutLength == 12 &&
              kCmdJogCrankTarget == 16 && kCmdJogTimeTarget == 20,
          "command offsets");
    Check(kCtlEnable == 1 && kCtlPlcControl == 2 && kCtlStart == 4 && kCtlStop == 8 && kCtlAbort == 16 &&
              kCtlClearFault == 32 && kCtlSimulate == 64 && kCtlJog == 128 && kCtlTrigger == 256,
          "ControlFlags bits");
    Check(kRejectNotInControl == 11 && kRejectFaulted == 19 && kRejectEstopOpen == 20 && kRejectNotEipTrigger == 21 && kRejectAxisOffCam == 22 && kRejectOnLimit == 24 && kFaultLimit == 5 && kAxLimitPlus == 0x80 && kAxLimitMinus == 0x100 &&
              kCtlHome == 512 && kAxHomed == 0x200 && kAxHoming == 0x400 && kFaultHome == 6 && kRejectNotHomed == 23 &&
              kFlagWaiting == 0x4000 && kFaultWaitTimeout == 7 && kCmdWaitBits == 24 && kStsWaitIndex == 80 && kStsWaitedMs == 84,
          "reject codes");
    {
        const uint8_t cmd[4] = {0x45, 0x01, 0, 0};  // ENABLE | START | SIMULATE | TRIGGER
        const uint16_t f = Get16(cmd + kCmdControlFlags);
        Check((f & kCtlEnable) && (f & kCtlStart) && (f & kCtlSimulate) && (f & kCtlTrigger) && !(f & kCtlPlcControl),
              "ControlFlags little-endian");
    }

    Status s;
    s.firmwareRev = 0x0003;
    s.flags = kFlagProfileLoaded | kFlagRunning | kFlagSimulate;
    s.faultCode = kFaultEstop;
    s.rejectCode = kRejectBusy;
    s.engineState = kStateRunning;
    s.crankState = 2;
    s.timelineState = 5;
    s.outputs = 0x21;
    s.profileHash = 0x4399fd04u;
    s.heartbeat = 0xA1B2C3D4u;
    s.crankAngle = 359999;
    s.crankRev = -3;
    s.crankRpm = 2000;
    s.timelineTime = 1234567;
    s.timelineCycle = 42;
    s.timelineOverride = 10000;
    s.encoderCount = -123456;
    s.axisCount = 2;
    s.inputs = kInTrigger | kInEstopOk;
    s.cutLength = 500000;
    s.cutLengthNext = 600000;
    s.cutLengthMin = 120000;
    s.cutGap = 0;
    s.syncLeft = -1;
    s.commLossCount = 7;
    s.axes[1].commandedCounts = -800;
    s.axes[1].commandedTravel = -1000;
    s.axes[1].flags = kAxUsed | kAxTimeline | kAxHlfb;
    s.axes[1].connector = 1;
    s.axes[3].commandedCounts = 0x7FFFFFFF;
    s.axes[3].connector = 3;

    uint8_t b[kStsSize];
    std::memset(b, 0xEE, sizeof(b));
    Pack(s, b);
    Check(U16(b, 0) == 3 && b[0] == 3 && b[1] == 0, "FirmwareRev little-endian");
    Check(U16(b, 2) == (1 | 16 | 2048), "StatusFlags");
    Check(U16(b, 4) == 1 && U16(b, 6) == 13 && U16(b, 8) == 4, "codes and EngineState");
    Check(U16(b, 10) == 2 && U16(b, 12) == 5 && U16(b, 14) == 0x21, "crank / timeline state, outputs");
    Check(U32(b, 16) == 0x4399fd04u && b[16] == 0x04, "ProfileHash");
    Check(U32(b, 20) == 0xA1B2C3D4u, "Heartbeat");
    Check((int32_t)U32(b, 24) == 359999 && (int32_t)U32(b, 28) == -3 && (int32_t)U32(b, 32) == 2000, "crank");
    Check((int32_t)U32(b, 36) == 1234567 && (int32_t)U32(b, 40) == 42 && (int32_t)U32(b, 44) == 10000, "timeline");
    Check((int32_t)U32(b, 48) == -123456 && U16(b, 52) == 2 && U16(b, 54) == 3, "encoder, axis count, inputs");
    Check((int32_t)U32(b, 56) == 500000 && (int32_t)U32(b, 60) == 600000 && (int32_t)U32(b, 64) == 120000, "cut");
    Check((int32_t)U32(b, 68) == 0 && (int32_t)U32(b, 72) == -1 && U32(b, 76) == 7, "gap, sync, comm loss");
    bool reserved = true;
    reserved = reserved && U16(b, 80) == 0 && U32(b, 84) == 0;  // no wait holding (s's waitIndex / waitedMs are 0)
    for (int i = 82; i < 84; ++i) reserved = reserved && b[i] == 0;
    for (int i = 88; i < 96; ++i) reserved = reserved && b[i] == 0;
    for (int a = 0; a < 4; ++a) {
        for (int i = 12; i < 16; ++i) reserved = reserved && b[96 + 16 * a + i] == 0;
    }
    Check(reserved, "reserved bytes zero");
    {
        Status w;
        w.waitIndex = 3;
        w.waitedMs = 70000;
        uint8_t wb[kStsSize];
        Pack(w, wb);
        Check(U16(wb, 80) == 3 && U32(wb, 84) == 70000u && wb[82] == 0 && wb[83] == 0, "WaitIndex at 80, WaitedMs at 84 (0.10.0)");
    }
    Check(U32(b, 96) == 0 && U16(b, 104) == 0, "axis 0 unused: zero");
    Check((int32_t)U32(b, 112) == -800 && (int32_t)U32(b, 116) == -1000, "axis 1 counts, travel");
    Check(U16(b, 120) == (1 | 2 | 8) && U16(b, 122) == 1, "axis 1 flags, connector");
    Check(U32(b, 144) == 0x7FFFFFFFu && U16(b, 154) == 3, "axis 3");
    Check(Get32(b + 16) == 0x4399fd04u && Get16(b + 52) == 2, "Get16 / Get32");

    // A default Status: SyncLeft -1, the rest zero.
    Status z;
    Pack(z, b);
    bool zero = true;
    for (int i = 0; i < kStsSize; ++i) zero = zero && (i >= 72 && i < 76 ? b[i] == 0xFF : b[i] == 0);
    Check(zero, "default status: zero, SyncLeft -1");

    // Scaling: round half away from zero, saturate, NaN 0.
    Check(Scale(12.3456, 1000.0) == 12346 && Scale(-12.3456, 1000.0) == -12346, "Scale rounds");
    Check(Scale(0.4, 1.0) == 0 && Scale(0.5, 1.0) == 1 && Scale(-0.5, 1.0) == -1, "Scale half");
    Check(Scale(1.0e12, 1.0) == 2147483647 && Scale(-1.0e12, 1.0) == (int32_t)0x80000000, "Scale saturates");
    double nan = 0.0;
    nan = nan / nan;
    Check(Scale(nan, 1.0) == 0, "Scale NaN");
    // Timeline time: samples (Q16) to us at 5 kHz.
    Check(Scale(5000.0, 1.0e6 / 5000.0) == 1000000, "1 s of samples = 1e6 us");

    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
