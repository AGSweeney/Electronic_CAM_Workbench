/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef __ECAM_ENGINE_H__
#define __ECAM_ENGINE_H__

/*
 * The cam engine: a loaded profile (ecam::CamRuntime) driven from the ClearCore's 5 kHz sample
 * interrupt through SysMgr.SampleCallbackSet(), with the profile's motors in external step mode.
 *
 * Every action returns nullptr on success or an error message for the console.
 */

#include <stddef.h>
#include <stdint.h>

namespace eipp {
struct Status;
}

void EngineInit();
/* Main loop: estop and motor faults. */
void EnginePoll();

/* Upload: "load", then the profile lines, then "commit". */
const char *EngineLoadBegin();
const char *EngineLoadLine(const char *line);
bool EngineLoading();
/* On success writes "axes=.. hash=.. stored=.." into info. Motors are disabled by a new load. The
 * profile is also saved to the flash store (stored=1), so it is loaded again at the next boot. */
const char *EngineCommit(char *info, size_t len);
/* Boot: load the profile kept in flash (motors stay disabled, nothing starts). */
const char *EngineRestore(char *info, size_t len);
/* "download": the loaded profile's upload text, one line per call. */
const char *EngineDownload(void (*out)(const char *line));
/* "forget": clear the flash store (the loaded profile stays until a restart). */
const char *EngineForget();

const char *EngineEnable();
const char *EngineDisable();
const char *EngineClearFault();
const char *EngineStart();
const char *EngineStop();
const char *EngineAbort();
const char *EngineTrigger();
const char *EngineOverride(double fraction);
const char *EngineRpm(double rpm);
/* Cut-length mode: the finished piece length in mm (the cycle is that plus the kerf), from the next cut
 * on (at once while the crank is off). */
const char *EngineLength(double mm);
/* Hand jog: the master goes where the host says (EngineJogCrank: absolute crank degrees since the
 * load, rev * 360 + angle; EngineJogTime: timeline seconds) and the motors follow their profiles. */
const char *EngineJog(bool on);
const char *EngineJogCrank(double degrees);
const char *EngineJogTime(double seconds);
/* Simulate: the profile runs, positions are reported, no steps go to the motors. Leaving simulate
 * also leaves the simulated encoder. */
const char *EngineSimulate(bool on);
/* Simulate only: the cam follows a simulated encoder count moving at countsPerSecond (0 holds it)
 * instead of the encoder input; status / TEL report it as enc= (and simenc= in status). */
const char *EngineSimEncoder(double countsPerSecond);
void EngineApplySettings();
/* "set estop_pin" / "estop_mode": nullptr, or why the estop can't be on this input with the loaded profile (an output, the
 * trigger, the Encoder Adapter's DI-6..DI-8 with an encoder master). */
const char *EngineEstopCheck(int pin, int mode);
/* Operator inputs (0.11.0): a setting moving operator input n (0..4) onto pin refused while the estop, another operator
 * input or the loaded profile uses it; op_test (bench, RAM): an input forced pressed (1), released (0), or not (-1). */
const char *EngineOpPinCheck(int input, int pin);
const char *EngineOpTest(int input, int state);
/* Axis jog: one motor (connector c, M-c) on its own, to set the machine up, through the step outputs (no ClearPath
 * USB). A profile axis moves in its units, keeping the cam's position; a motor the profile doesn't use (any motor,
 * with no profile) in counts; a driven crank's motor only enables. Refused while the cam runs, stops, hand jogs or
 * is armed, during an upload and with a fault latched; disable never is (a motor the cam is moving: every motor).
 * EngineJogVelocity must be sent again within ECAM_AXJOG_DEADMAN_MS (0 ramps to rest); EngineJogToCam moves an axis
 * where the cam has it at the next start; EngineJogZero makes its position here 0 (an axis: the profile's start).
 * Start and hand jog are refused while an axis jog moves, or while an axis axis jog moved is off its cam position. */
const char *EngineJogEnable(int c, bool on);
const char *EngineJogVelocity(int c, double unitsPerSec);
const char *EngineJogMove(int c, double units, bool absolute, double unitsPerSec);
const char *EngineJogToCam(int c, double unitsPerSec);
const char *EngineJogStop(int c);  /* -1: every motor */
/* A velocity jog that ramps to rest (over the limit stop time) the moment one of the listed inputs changes from its level
 * when the jog started ("all", or IO-3,A-10,CCIOA2): the wizard's discovery moves. axjog reports the input and where. A
 * repeat going the same way after it stopped is refused until "axjog <m> stop". */
const char *EngineJogStopOn(int c, double unitsPerSec, const char *list);
/* "limit_test <axis> +|-|off": the limit reads as tripped (RAM, the bench). It can only stop a machine. side 0: off. */
const char *EngineLimitTest(int axis, int side);
/* Homing (0.9.1): one motor's axis (c 0..3, -1 all), every axis with a home switch in order, stop; home_test. */
const char *EngineHome(int c);
const char *EngineHomeAll();
const char *EngineHomeStop();
const char *EngineHomeTest(int axis, bool on, double units);
void EngineHomeTestList(void (*out)(const char *line));
void EngineHomeList(void (*out)(const char *line));   /* axhome alone */
void EngineLimitList(void (*out)(const char *line));  /* axlimit */
/* Timeline waits (0.10.0): the PLC's wait bits and whether a connection is live; the waits now; skip (simulate). */
void EngineSetPlcWaitBits(uint32_t bits, bool live);
void EngineWaitList(void (*out)(const char *line));
const char *EngineWaitSkip();
void EngineWaitBitsTest(bool on, uint32_t bits);  /* waitbits_test: bench, RAM only */
const char *EngineJogZero(int c);
/* "axjog": one line per motor: role (axis, free, crank), on, moving, position, cam position, off, counts/unit, limit,
 * units, name. */
void EngineJogList(void (*out)(const char *line));
/* Motors disabled, nothing moving, no upload in progress: network settings may change and the board may
 * restart (EtherNet/IP TCP/IP Interface sets, Identity Reset). */
bool EngineSafeForNetworkChange();
/* Called every few ms while a commit builds the tables and writes the flash (a few hundred ms), so the
 * network keeps running. It must not call back into the engine; EngineSafeForNetworkChange() is false
 * meanwhile. */
void EngineSetYield(void (*fn)());
/* Crank engaged, the timeline running, stopping or jogging, or an axis jog moving. Flash and NVM writes must wait for the end of it:
 * they hold the sample interrupt off, the step timer repeats the last sample's pulses meanwhile, and the drive
 * ends up ahead of the board's count. */
bool EngineMoving();
/* The motors enabled, or anything moving. A flash or NVM write must wait for this to clear. */
bool EngineBusy();
/* The CCIO-8 link and its inputs, for the console. */
void EngineCcio(void (*out)(const char *line));

/* Reads the estop as open while true, whatever A-9 and estop_a9 say. RAM only, for the bench: A-9 needn't be wired. */
void EngineEstopTest(bool open);
/* The eipp::RejectCode of the last call refused (an action returned an error), for the PLC's RejectCode. */
uint16_t EngineRefusal();
/* The PLC lost its connection (closed, timed out, Program) while it had control: a controlled stop if
 * anything moves, then the motors disable and COMM_LOSS latches; idle but enabled: the motors disable. */
void EngineCommLoss();
/* The PLC's TRIGGER bit: starts the timeline (as "trigger") when the profile's trigger is EtherNet/IP. */
const char *EngineEipTrigger();
/* The loaded profile: which masters it uses, and its own crank speed (rpm) and timeline override (1 = as
 * drawn), for the PLC's "0 = the profile's" setpoints. All false / 0 with no profile. */
struct EngineMasters {
    bool loaded, crank, drivenCrank, timeline, lineMode;
    double rpm, override;
};
EngineMasters EngineGetMasters();

void EngineStatus(char *buf, size_t len);
void EngineTelemetry(char *buf, size_t len);
/* The EtherNet/IP status assembly in its units (EipProtocol.h); PLC control, RejectCode and
 * CommLossCount are EipLink's and left zero. */
void EngineEipStatus(eipp::Status *st);
/* "profile" command: one line per axis. */
void EngineProfile(void (*out)(const char *line));
/* "motors": per connector, used, enable, HLFB (state and duty), ready state, status and alert registers. */
void EngineMotors(void (*out)(const char *line));
/* "diag": interrupt timing, carry. */
void EngineDiag(char *buf, size_t len);

#endif
