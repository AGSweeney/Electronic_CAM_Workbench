# ECam_Firmware protocol

**Experimental.** Not certified for production or safety-critical use. Keep a physical estop in the circuit.

The ClearCore runs cam profiles made in the Electronic CAM Workbench. A profile is **loaded** onto the
board as text, kept in flash, and run from the 5 kHz sample interrupt; nothing is streamed while it runs.
Firmware `0.8.7`.

## Links

| Link | Role |
|------|------|
| USB CDC (any baud) | Console |
| TCP **9500** | Console, one client (a new connection is accepted once the previous one has closed) |
| UDP **9502** | `ECAM_DISCOVER?` → `ECAM ECam_Firmware IP=… CONSOLE=9500 FW=…` |
| TCP/UDP **44818**, UDP **2222** | EtherNet/IP adapter (QuickEIP): identity, assemblies 150 / 100 / 151, network settings (below) |

Clear of ClearCNC (8888, 8889, 10040), ClearCoreROS (9200–9203) and ClearHPGL (9100, 9300, 9302).
DHCP by default; when no DHCP server answers (about 10 s) the board uses its **stored static address**
(`ip`, `netmask`, `gateway`; 192.168.0.110 / 24 until set), so a board switched to DHCP stays where it was.
`set net_mode 1` plus `ip`, `netmask`, `gateway` for a static address after `restart`. On first boot the
network and HLFB-check settings are taken from a ClearHPGL settings blob left on the board. Once its
address is set the board announces it (gratuitous ARP at 0, 1 and 3 s), so a PC or PLC that tried it while
it restarted reaches it at once.

**One command per main-loop pass.** USB and TCP each run at most one command per pass, so a host that
sends without pause can't starve the rest of the loop (EtherNet/IP, discovery). Throughput is unchanged
(about 1,500 `status` a second over TCP).

The console is the same on every link. Every command ends with a line starting `ok` or `error:`. Lines
starting `TEL ` are telemetry and may arrive between replies. There is no echo and no prompt; a terminal
works with local echo on. A TCP client gets a banner (name, version, IP, where the settings came from, and
a line when no estop is set).

## Loading a profile

```
load
master src=encoder cpr=4000 den=1 rpm=60 crank=0 ramp=60 max=600
timeline repeat=0 ovr=1 trig=input in=7 rise=1 bit=0
axis 0 master=crank m=0 cpu=800 start=0 closed=1 span=360 units=mm name=Pusher
seg 0 120 20 cycloidal
seg 0 180 20 dwell
seg 0 300 0 cycloidal
seg 0 360 0 dwell
axis 1 master=timeline m=1 cpu=800 start=0 closed=1 span=2.5 units=mm name=Slide
seg 1 0.8 150 modtrap
...
out 0 0.3 1.4
out 1 1.4 1.5
out 2 95 165 crank
json AAAF8XjarZRLb9swDMe/yqBzYDgPe5mPbYchh21Gip6GHVSJdYXpYVByHgjy3UfZedhI+gJ6E8m/...
json ...
commit
ok axes=2 hash=1a2b3c4d stored=1
```

Each line is answered with `ok` or `error: …`. `commit` checks the whole profile, builds the tables and
replies with the axis count, the hash and whether it was kept in flash. The hash is FNV-1a over every line
between `load` and `commit`, each followed by `\n`, `json` lines included; the Workbench computes the same
hash and compares (`core/ProfileText.cpp` is shared by both sides; `ECamWorkbench --profile-lines` prints
an upload and its hash). A new profile disables the motors. `load` and `commit` are refused while anything
moves.

| Line | Fields |
|------|--------|
| `master` | `src` encoder / driven; optional `rev=1` to count the encoder the other way (applied at `commit`); optional `line=<counts per mm>` and `kerf=<mm>` for cut-length mode (`den` 1: `cpr` is then the design cut in counts; counts of the encoder, or of the crank motor on a driven crank, which feeds the line); `cpr` master units per crank rev and `den` units per count (optional, default 1): ppr × gear numerator and the gear denominator, so a ratio that is not whole (50 : 7) stays exact, the board counting in 1/den of an encoder count or crank-motor step; `rpm`; `crank` crank motor connector; `ramp` rpm/s; `max` rpm |
| `timeline` | `repeat` 0/1, `ovr` speed override (1 = as drawn), `trig` cmd / input / eip, `in` input index (0–5 IO-0..5, 6–8 DI-6..8, 9–12 A-9..12), `rise` 1/0, `bit` EtherNet/IP bit (reserved) |
| `axis <i>` | in order from 0; `master` crank / timeline, `m` motor connector, `cpu` motor counts per unit, `start` position at x = 0, `closed` 0/1, `span` 360 or the duration in s, `units`, `gear` (optional, `num:den`: an electronic gear on the crank, `num` units per `den` crank revolutions, `num` negative to reverse; its `seg` is one `linear` segment, for the record), `name` (rest of the line) |
| `seg <i> <end> <pos> <law>` | `end` in degrees (crank) or seconds (timeline), `pos` in units; laws `dwell linear harmonic cycloidal modtrap modsine poly345 poly4567 blend spline` (`blend` takes its neighbours' velocities at its ends; consecutive `spline` segments are one cubic spline through their knots; older firmware refuses them) |
| `out <n> <on> <off> [crank]` | Output IO-*n* (0–5). Without `crank`: on from `on` to `off` seconds of timeline time (`off` ≥ `on` + 1 ms); the cycle is at least as long as the last `off`, so a profile with timeline pulses and no timeline axis still has a timeline. With `crank`: on from `on` to `off` crank degrees (0 ≤ `on` < `off` ≤ 360) every revolution. Up to 32 in all. |
| `ccio_boards <n>` | CCIO-8 boards the project uses, 1-8; left out with none. See CCIO-8 inputs. |
| `json <base64>` | Optional, any number, after the motion lines: the Workbench's whole `.ecam` project (compact JSON, zlib via Qt's `qCompress`, base64, split into chunks), so `download` gives it back exactly. Stored and hashed, never used for motion. |

Limits: 4 axes (one per connector), 32 segments per axis, 32 output pulses, 160 characters per line,
32 KB per upload. A crank axis ends at 360, a timeline axis at its span. Refused trigger inputs: DI-6..DI-8
while an encoder is the master of a crank axis (the Encoder Adapter uses them), an IO pin that is also an
output (crank or timeline pulse), and the estop's input (`estop_pin`) while the estop is on.

### Outputs

The IO pins in `out` lines become digital outputs at `commit` (off); pins the previous profile used go
back to inputs. In the sample interrupt an output is on while one of its pulses is active: a crank pulse
while the crank is engaged (following, running, stopping, hand jog) and its phase is inside the pulse, a
timeline pulse while the timeline runs, stops or hand jogs and its time is inside the pulse. **Every
output is off** otherwise (idle, armed, done, after a stop, abort, fault, estop). Pins are written only when they change. **In simulate
the pins stay off**: `out=` in `status` and `TEL` still reports what the profile has on.

### Kept in flash

A committed profile is also written, exactly as uploaded, to the top 32 KB of the processor's flash
(`FlashStore.cpp`, 0x78000–0x7FFFF; the linker scripts end the firmware below it) and read back to
verify: `stored=1` in the `commit` reply and in `status`. If the write fails, the profile still runs and
the reply says `stored=0 store_error=<why>`. The CPU cache is off while the flash is erased and written.

At power-up the board loads the stored profile through the same path as an upload (USB prints
`stored profile loaded: axes=… hash=… stored=1`, or `no stored profile (…)`), with the motors disabled and
nothing started.

Flashing new firmware erases the chip, and the stored profile with it. The Workbench's firmware update saves the
profile with `download` first and loads it back; `src/FirmwareUpdate.cpp`.

| Command | Effect |
|---------|--------|
| `download` | The loaded profile's lines exactly as uploaded (motion lines and `json` lines), then `ok`. Their FNV-1a is the `profile` hash. `error: no profile loaded` when there is none. |
| `forget` | Erase the stored profile (cannot be undone). The loaded profile keeps running until a restart; the board then comes up empty. The Workbench follows it with `restart` when nothing moves. Refused while the motors are enabled or anything moves. |

### CCIO-8 inputs

CCIO-8 boards chain from COM-1, up to 8. They're found at power-up; with none, the port closes (the library would
otherwise repeat a blocking search every second), so a board added later needs a restart. With boards found,
`status` ends with `ccio=<boards> ccio_link=ok|broken|off ccio_in=<16 hex digits>`, bit 0 the first board's first input,
and `ccio` prints the same. The sensors are normally closed, so on a broken link every input reads 0, as if tripped.
The board blinks code 3-4 while the link is broken; the library retries every second and the code clears when the link
is back.

The profile decides the link. The profile line `ccio_boards <boards>` (1-8, left out with none) keeps it up. A loaded profile
without one closes the port, so a board that drops out can't fault: `ccio_link=off`, the inputs read 0, and `ccio=` is
still the boards found. With no profile loaded the link stays up, so the inputs can be seen before a project is made.
Loading a profile with the line reopens it; if the boards have gone, they're gone until a restart.

A CCIO-8 input with a limit job (below) trips while the link is broken, whatever its contact.

### Limits (0.9.0)

```
input board <pin 0-12> <home|limit+|limit-> <axis> <nc|no>
input ccio <board 0-7> <pin 0-7> <home|limit+|limit-> <axis> <nc|no>
limits stop=<seconds>
```

`input` gives one of the board's own inputs (IO-0..IO-5, DI-6..DI-8, A-9..A-12 as 0-12) or a CCIO-8's a job for an axis.
`nc`: normally closed, the input is on at rest and off when tripped (and when a wire is cut); `no`: the other way round.
Refused: an axis the profile doesn't have, a pin that is an output, the trigger, the Encoder Adapter's (DI-6..DI-8 with
an encoder master) or the estop's, a CCIO-8 past `ccio_boards`, and a second input with the same job on one axis.
`limits stop=` (0.01-2 s, 0.1 when left out) is the limit stop time; it is written only with a limit in the profile.
A `home` input homes its axis (0.9.1):

```
home <axis> dir=-|+ fast=<u/s> slow=<u/s> back=<u> at=<u> then=stay|start order=<1-9> search=<u> req=0|1 [dec=<u/s2>]
```

One per axis with a home input (refused for an axis without one; a home input with no line, as in a 0.9.0 profile,
homes with the defaults: toward -, fast 10 % of the axis's jog limit, slow fast / 10, back fast x 0.25 s, at 0, then
start, order 1, search 0, not required). The sequence runs on axis jog's generator: off the switch if it starts on it
(and `back` further), toward it at `fast` until it trips, ramp to rest, away at `fast` until it releases, `back` further,
toward it at `slow`; the sample it trips is the latch, and the axis's position there becomes `at`. `then=start` moves it
to its cam start at `fast`. `search` (0: none) bounds the search and the back-offs. `dec` (0.9.3, left out or 0: as axis jog, the axis's
acceleration limit or `fast` in a quarter second) is how hard it stops on tripping the switch, after the search and after
the latch: the run-on past the switch is `fast`² / (2 `dec`). Acceleration and the back-offs stay as axis jog. A limit, a search past `search`, or a
switch that doesn't release fails it: the fault `home failed on <axis>: <why>` (FaultCode 6), the motors stay enabled.
Homed holds until that motor is disabled, a fault that disables, `axjog <m> zero`, a new profile, a restart. `start` and
`trigger` are refused while an axis with `req=1` isn't homed (`error: Slide (M-1) isn't homed: home it`, RejectCode 23),
not in simulate. The latch is taken after the input's 3-sample filter: 0.6 ms late, the same every time, so `at` takes it up.

A limit is read every sample and filtered over 3 samples (0.6 ms), so a bounce doesn't trip it. It acts when an axis
moves toward it (a + limit on + travel), not when it moves away, so the axis can always be driven off:

- **The cam running.** The master ramps to rest over the limit stop time: a driven crank's speed, or the timeline's
  rate, and every axis follows it, so the cam stays in step. With an encoder master, which can't be ramped, each crank
  axis lets go and ramps to rest itself over the stop time. The fault `limit + on <axis> (<input>)` latches
  (FaultCode 5) and the motors stay enabled. `clear` works on the limit; `start`, `trigger` and Hand jog are refused
  while an axis is on one (`error: <axis> (M-1) is on its + limit (IO-3): jog it off first`), and so is axis jog toward it.
- **Axis jog.** A jog or move toward a tripped limit ramps that axis to rest at its own limit, no fault; jogging the
  other way works.
- **Simulate** only reports a limit: nothing stops.

The stop takes the speed × stop time / 2 past the switch (shown as "Overtravel at peak speed" in the Workbench), so
the switch wants at least that much room before the hard end.

| Command | Effect |
|---------|--------|
| `axjog <m> jog <u/s> stopon <list>` | A jog that stops at the first input in the list to change from its level when the jog started, `all` for every input, or names such as `IO-3,A-10,CCIOA2`. It ramps to rest at the axis's limit and `axjog` shows `stopped=<input> at=<position>` until `axjog <m> stop`; a jog the same way is refused until then. For setting switches up. |
| `axhome <m>`, `axhome all`, `axhome stop` | Homing (0.9.1): the axis on motor `<m>`, or every axis with a home switch by its order; `stop` stops it (so do `axjog <m> stop`, `stop`, `abort`, `disable`). Needs the motor on, nothing running, no fault, not simulate. |
| `axhome` (alone), `axlimit` | For the switch wizard (0.9.2), one line per profile axis. `axhome <i> M-<c> has= homed= homing= ends=<homings ended since the load> latch=<counts> pos=<counts> last=ok\|stopped\|limit\|notfound\|norelease`. `axlimit <i> M-<c> lim=<+><-> tripp=<n> atp=<counts> tripm=<n> atm=<counts> pos=<counts>`: the axis jog stops by each limit since the load, and where the limit tripped the last time. |
| `home_test <axis> <u>\|off`, `home_test` | For the bench: a home switch where the axis's position reads `<u>` now, fixed to the motor (homing moves the position, not the switch), tripped beyond it toward the home direction. RAM only. Alone: each axis's `switch`, `motor` (steps sent, in the axis's sense), `pos`, the last `latch` and result. |
| `limit_test <axis> +\|-\|off` | For the bench: the axis's + or − limit reads tripped, whatever its input says, until `off` or a restart. RAM only. |

### Timeline waits (0.10.0)

```
wait <t s> board <pin 0-12> on|off [edge] [timeout=<s>]
wait <t s> ccio <board 0-7> <pin 0-7> on|off [edge] [timeout=<s>]
wait <t s> plc <bit 0-31> on|off [edge] [timeout=<s>]
```

`edge` (0.10.1), *needs a change*: the wait passes only if its input has read the other level since the wait last
passed (since Start, the first time), and reads the level now. A stuck sensor holds it; a change any time before the
timeline reaches it counts. Left out (as in 0.10.0): the level alone. An input that can't be read (a CCIO-8 with the
link down, a PLC bit with no connection in Run) is neither level and arms nothing.

Up to 16, after the axes. At `t` (into the cycle, as drawn) the timeline holds until the input reads the level: the
time stops exactly on `t` (every timeline axis and output where it is, a pulse that starts at `t` not yet), and runs on
from there at the rate it had. A board or CCIO-8 input is filtered over 3 samples; a PLC bit (command byte 24) is as
last received and only counts with a connection open and in Run. `timeout` (0 or left out: none) faults: `wait <n>
(<input> <level>) timed out at <t> s`, FaultCode 7, the timeline stopped there, the motors enabled. Each wait holds once
a cycle. Refused: `t` not inside the timeline's cycle, a profile with no timeline, a pin that is an output, the
trigger, the Encoder Adapter's, the estop's or a home / limit input, a CCIO-8 past `ccio_boards`.

`stop` (or `abort`, `disable`, a fault) while waiting stops there, without finishing the cycle. Hand jog ignores waits.
In simulate the waits hold on the real inputs and bits.

| Command | Effect |
|---------|--------|
| `waits` | Each wait: time, input, level, `mode=change\|level`, timeout, met now, `armed` (the other level seen since it last passed), times reached, timeouts; and the PLC wait bits (live or not). |
| `wait_skip` | Simulate only: pass the wait holding the timeline. |
| `waitbits_test <hex>\|off` | For the bench: these PLC wait bits instead of the PLC's, as if connected. RAM only. |

### Operator inputs (0.11.0)

A panel on the board's inputs, for a machine run with no Workbench and no PLC: `op_enable`, `op_disable`, `op_reset`,
`op_start`, `op_stop`, each an input (IO-0..IO-5, DI-6..DI-8, A-9..A-12) or `off`, `op_<name>_contact nc|no` (NC:
pressed when the input is low; Disable and Stop default to NC), and `op_enable_mode momentary` (Enable and Disable
buttons) or `maintained` (an Enable switch: on enables, off disables). Board settings, like the estop.

- A press is 10 ms of steady level. Enable, Fault reset and Start act on becoming pressed, never on being pressed when
  the board starts or the setting is made. Disable and Stop act on becoming pressed; held, they keep Enable / Start from
  acting.
- The actions are the console's: `enable`, `disable`, `clear`, `start` (plus `trigger` for a timeline whose trigger is
  `cmd`), `stop`. While the PLC has control, Enable, Fault reset and Start are ignored; Disable and Stop always act.
  Each one is logged on USB: `op: Start`, or `op: Enable: refused: <why>`.
- One job per pin: an operator input can't share a pin with another one or the estop, the board refuses a profile that
  uses one (an output, the trigger, a home / limit, a wait, the Encoder Adapter's), and refuses moving an operator input
  onto a pin the loaded profile uses.
- `status`: `ops=<hex>` pressed now (bit 0 Enable .. 4 Stop), `oppins=<e>,<d>,<r>,<s>,<t>` (-1 none), `opnc=<hex>`,
  `opmode=0|1`.

### Motor direction

`mrev=1` on an `axis` line, or on `master` for a driven crank, turns that motor the other way. Only the steps sent to
the motor change: `status`, positions and the diagram are as drawn, so positive stays positive travel of the axis.

## Running

| Command | Effect |
|---------|--------|
| `enable` / `disable` | The loaded profile's motors (followers, and the crank motor when driven). Enable checks HLFB (`hlfb_check`) and the estop, and clears a latched fault. |
| `simenc <counts/s>` | Simulate only: the cam follows a simulated encoder count the firmware advances at this rate each sample (negative runs backwards, 0 holds it), starting from the real count so nothing jumps. `status` and `TEL` report it as `enc=`, and `status` adds `simenc=<counts/s>`. `sim 0` returns to the real encoder input. |
| `sim 0\|1` | Simulate: the profile runs and positions are reported, but no steps go to the motors. Start and hand jog do not need enable (nor the motors connected); `sim 1` also clears a latched fault, the motors staying disabled. |
| `start` | Crank axes engage: they follow the encoder, or the crank motor ramps up to `rpm` and they follow it. The timeline arms. |
| `trigger` | Starts the timeline from t = 0 (as its input edge would). Works when armed, or after a one-shot finished. |
| `stop` | A running timeline finishes the cycle it is in and comes to rest at its end (`endcycle=1` in `status` meanwhile; a one-shot then waits for Start, not the trigger). `stop` again while it finishes: its speed ramps to zero over `stop_ms`, part-way through the cycle. A driven crank ramps down at its ramp; an encoder master cannot be ramped: its followers let go at once. A lost PLC connection stops as the second `stop` does. |
| `abort` | Everything stops at once. |
| `jog on` / `jog off` | Hand mode: the master goes where `jog crank` / `jog time` say instead of following the encoder, the crank speed or the clock; every axis follows its profile and a driven crank's motor turns with it. Needs nothing moving and enabled motors (or `sim 1`). Speed is limited to the profile's crank speed / timeline speed, reached in a quarter second (a driven crank uses its ramp). `jog off` and `stop` come to rest, then release; `abort` stops at once. |
| `jog crank <deg>` | Crank target in absolute degrees since the load (`rev * 360 + deg` from `status`), so whole turns count. |
| `jog time <s>` | Timeline target in seconds, clamped to the cycle. |
| `axjog` | Axis jog (0.8.9): one line per motor connector, `axjog <m> role=<axis\|free\|crank> axis=<i\|-> on=<0\|1> moving=<0\|1> pos=<counts> cam=<counts\|-> off=<counts> cpu=<counts/unit> vmax=<units/s> lim=<+\|-\|+-\|0> stopped=<input\|-> at=<pos> units=<u> name=<name>`. `axis`: a profile axis (`axis` its index), in the axis's units; `pos` is its commanded position from the profile's start, as `a<i>`; `cam` where the cam has it at the next start (`-` for a gear or an open profile: no fixed place); `off` how far axis jog left it from there (0 when on it, or not moved by axis jog). `free`: a motor the profile doesn't use (every motor, with no profile), in counts since the boot or its zero. `crank`: a driven crank's motor, enable and disable only (Hand jog turns it with the axes following). `vmax` is the speed limit (the profile's, at most 4000 rpm of an 800-count motor). |
| `axjog <m> enable` / `disable` | One motor, `<m>` 0..3 or `M-0`..`M-3` (its HLFB is waited for as `enable` does). `disable` is never refused; a motor the cam is moving: every motor, as `disable`. |
| `axjog <m> jog <u/s>` | Velocity jog, signed, in the motor's units (counts for a free motor). It must be sent again within 0.5 s or the motor ramps to rest by itself (the deadman); `0` ramps to rest. |
| `axjog <m> move <u> <u/s>` / `goto <u> <u/s>` | A relative move / a move to a position, landing on it to the count. Moves on different motors run at once. |
| `axjog <m> cam <u/s>` | A profile axis: moves it to its `cam` position. |
| `axjog <m> zero` | Its position here becomes 0 (an axis: the profile's start). |
| `axjog <m> stop`, `axjog all stop` | Ramp to rest. `abort`, `disable`, a fault and the estop stop at once. |

**Axis jog** moves a motor on its own through the step outputs, to set the machine up; the ClearPaths' USB isn't
used. Acceleration is the axis's limit, or full speed in a quarter second. It is refused (`error: the cam is
running: stop it first`) while the crank is engaged, the timeline runs, stops, hand jogs or is armed, and also during
an upload and with a fault latched; moves need the motor on (or `sim 1`). The PLC's control takes it too, but not
`axjog <m> disable` and `stop`. Start, Hand jog (`jog on`) and `trigger` are refused while an axis jog moves, and
Start and Hand jog while an axis that axis jog moved is off its cam position: the cam engages from wherever the motors
are, so it would run that axis's whole profile shifted. Move it back with `axjog <m> cam`, or zero it where it belongs.
A new profile forgets all of it and disables every motor. The board can't tell where a motor really is: after a power
cycle or a new profile it takes every axis to be at the profile's start, as before.
| `override <percent>` | Timeline speed (ramped). |
| `rpm <value>` | Driven crank speed (ramped at the crank ramp). In cut-length mode: cuts per minute at the design length, so the line speed stays the same at any cut length; `rpm=` reads the same way. |
| `length <mm>` | Cut-length mode: the finished piece length; the cycle is that plus the kerf, in whole encoder counts. It applies at the next cut (at once while the crank is off; while hand jogging, when the jog ends). Refused below the shortest piece (the cycle's fixed part less the kerf). |
| `clear` | Clear a latched fault. |
| `status` | `state= enabled= sim= fault= estop= a9= profile=<hash> stored=0\|1 axes= crank= tl= deg= rev= rpm= t= cycle= ovr= enc= out= [len= next= lmin= kerf= gap= sync=] a0= a1= …`; `estop` is `ok`, `open` or `off` (not configured); `out` is the outputs mask, bit *n* = IO-*n*; in cut-length mode `len` / `next` are the finished piece now and from the next cut, `lmin` the shortest piece, `gap` the line distance from the blade to the next cut line on the material (0 while synchronized on it) and `sync` the line left in the sync, -1 outside it (mm; both also in `TEL`; only when the carriage has a Linear sync segment), and `deg` is the position in the cycle as drawn |
| `tel <ms>` | Push `TEL ms= state= crank= tl= deg= rev= rpm= t= cycle= ovr= out= enc= a0= …` every `<ms>` (min 20, 0 off) on this link. For commissioning: leave it off on a running machine. Over TCP a line is sent only when it fits the send buffer. USB writes block the main loop, so a USB host that stops reading stalls the board, EtherNet/IP included, until it reads or closes the port. |
| `profile` | The loaded profile, one line per axis. |
| `diag` | Longest sample-callback run, steps carried over the per-sample limit, link statistics, how long the last `commit` took (check, build, store in ms), lwIP's heap and pools (use, high water, allocation failures), and the longest main-loop pass since the last `diag` (a commit or an enable's HLFB wait is one long pass). Saving settings or a profile writes NVM / flash, which stalls the processor briefly: one late sample per settings write, a few per profile save. |
| `config`, `set <key> <value>` | `estop_mode` (0 off, 1 fault when its input is low, 2 when high), `estop_pin` (IO-0..A-12, A-9 by default; `estop_a9 <mode>` sets both, on A-9), `op_enable`, `op_disable`, `op_reset`, `op_start`, `op_stop` (an input or `off`), `op_<name>_contact nc|no`, `op_enable_mode momentary|maintained` (operator inputs, 0.11.0), `hlfb_check`, `stop_ms`, `net_mode`, `ip`, `netmask`, `gateway`, `host` (EtherNet/IP host name, letters, digits, `-` and `.`; `-` for the default `ecam-` + the last 3 MAC bytes; applies at once); saved to the NVM user page (magic `ECAM`, version 5; versions 1 to 4 still load, with no operator inputs). Network keys apply after `restart`. Settings also set over EtherNet/IP (DNS servers, domain) are listed. `set` and `forget` are refused while the motors are enabled or anything moves: a write to flash holds the sample interrupt off for milliseconds, and in motion the drives would receive steps the board never counted. |
| `op_test enable\|disable\|reset\|start\|stop press\|release\|off` | For the bench: an operator input forced pressed or released, or back to its wire. RAM only. |
| `estop_test 0\|1` | For the bench: `1` makes the estop read as open, whatever its input and `estop_mode` say, until `estop_test 0` or a restart. RAM only. It can only stop a machine, never hide a real estop. |
| `eip` | The EtherNet/IP adapter: identity, the network settings stored and in use, the open connection (originator, assemblies, RPI, timeout, multicast) and its counters, and the PLC control state (ControlFlags, RejectCode, connection losses). |
| `motors` | Each motor connector: used by the profile, enable request, HLFB state and duty, ready state, status and alert registers. |
| `restart` | Replies `ok`, then resets the board (the TCP link drops; reconnect after about 5 s). |
| `version`, `help`, `exit` (closes the TCP link) | |

**States.** `state` is `empty` (no profile), `disabled`, `ready` (enabled or simulating), `armed`,
`running`, `jogging` or `fault`. `crank` is `off`, `following` (encoder master engaged), `running` (driven
crank), `stopping` or `jogging`; `tl` is `idle`, `armed`, `running`, `stopping`, `done` (one-shot finished),
`jogging` or `waiting` (with `wait=<n> waited=<ms>`, n from 1 in time order; in `TEL` too). `a<i>` are each axis's commanded motor counts from the profile's start (0 at the load), moved by the
cam and by axis jog. `epin` the estop's input (as the trigger index, -1 while it is off); `a9` A-9's level (1 high). `din` (hex) the board's own inputs, bit n = IO-0..IO-5, DI-6..DI-8, A-9..A-12 (an IO pin the profile drives reads as its output). `lim` (hex, 2 bits an axis: bit 2i its + limit tripped, bit 2i+1 its −). `hashome`, `homed`, `homing`, `homesw` (hex, bit i = axis i): a home switch, homed, homing (or waiting in a Home all), the switch tripped now. `mon` and `jogmove` (hex, bit c = M-c): the motor on, an axis jog moving it; `mfault` and `mhlfb` (bit c = M-c): the motor in fault (enabled, HLFB down past the enable), its HLFB up (asserted, or PWM while it moves);
`offcam` (bit i = axis i): axis
jog left it off its cam position. `enabled=1` means every motor of the profile is on.

**Engaging.** Steps are the change of each axis's target, so an axis engages at its current position:
whatever it is when `start` (crank) or the trigger (timeline) arrives is taken as the profile position at
that moment; homing (Limits, `home`) sets where that is. Open profiles add their advance every revolution or cycle.

**Faults.** An open estop or a motor fault on a used motor aborts the motion (all outputs off), disables
the motors and latches until `clear` / `enable`. A limit is a controlled stop and keeps the motors enabled (Limits).

**The estop.** `estop_pin` picks its input, any of IO-0..IO-5, DI-6..DI-8 and A-9..A-12 (A-9 by default), and
`estop_mode` how it trips. The board refuses a profile that uses the estop's input (an output, the trigger, or DI-6..DI-8
with an encoder master), and refuses moving the estop onto an input the loaded profile uses. Settings version 4 holds
the input; a board on version 2 or 3 keeps its estop, on A-9. Version 1 had it on DI-6, which comes up off, and the
banner says so until it is set. `set estop_di6` is refused with a pointer to these. New boards, and settings imported
from ClearHPGL, start with the estop off.

## How it runs

(The fixed-point formats, interpolation and limits in detail: [docs/MATH.md](../docs/MATH.md#10-the-runtime-on-the-clearcore).)

`commit` samples every axis into a table of motor counts at 2049 evenly spaced master positions (double
precision, in the main loop). The sample callback (`SysMgr.SampleCallbackSet`, an ECL addition) reads the
encoder and the trigger input, advances the master, interpolates each axis's table with integer
arithmetic, and hands each motor its steps (`MotorDriver::SetExternalSteps`, external step mode, also an
ECL addition). Measured on a ClearCore with three axes: under 25 µs of the 200 µs sample.

- Crank: phase within the revolution (in 1/den counts) plus a revolution count, so long runs never
  overflow or creep. Encoder: the phase moves by the encoder's change (the callback sees the previous
  sample's encoder reading). Driven: velocity ramped at the crank ramp; the crank motor gets the master's
  steps.
- Timeline: time in samples × the speed override, within one cycle plus a cycle count. The cycle is the
  longest timeline axis or the last timeline pulse's `off`, whichever is later; one-shot stops there,
  repeat wraps. Shorter axes rest at their end.
- Cut-length mode: the cycle is the cut length in encoder counts; the profile is looked up at the
  phase less (cycle − design), clamped to the wait at 0, so only the wait stretches. A new length is
  taken at the cut boundary. `jog crank <deg>` is in degrees of the cycle as drawn.
- Outputs: each sample, a crank pulse is compared with the crank phase (whole master units) and a
  timeline pulse with the timeline time; the pins change only when the result does.
- Step direction changes only with no pulses in flight (ECL `ApplySampleSteps`), so a drive that samples
  DIR slightly after the step edge never counts a step the wrong way.

## EtherNet/IP

The board is an EtherNet/IP adapter (QuickEIP, vendored in `QuickEIP/`): discovery, identity, the objects and the connections
(phase 1), the status assembly, live (phase 2), the command assembly's control flags, PLC control and
connection-loss handling (phase 3), and its setpoints, JOG and TRIGGER (phase 4).

| Item | Value |
|---|---|
| Identity | Vendor 65000, device type 43 (generic), product code 3, revision 1.8 (1 = this assembly layout; minor = firmware major × 100 + minor), "ECam ClearCore", serial from the MAC |
| EDS | [`eds/ECam_ClearCore.eds`](eds/ECam_ClearCore.eds): connections "Exclusive Owner" (150 / 100 / 151, 64 / 160 / 0 bytes) and "Input Only" (heartbeat 254, 100); RPI 10–100 ms, default 20 |
| Assemblies | 150 command (O→T, 64 bytes, run/idle header), 100 status (T→O, 160 bytes), 151 config (0 bytes) |
| Connections | Class 1, cyclic or change of state; O→T point-to-point or null (input-only); T→O point-to-point or multicast; one owner at a time |
| Explicit | Identity, Assembly, Connection Manager, TCP/IP Interface, Ethernet Link, Message Router (Multiple Service Packet); up to 2 sessions |
| Network settings | TCP/IP Interface attr 3 (static / DHCP), 5 (address; static only), 6 (host name) from any EtherNet/IP tool. Stored in the same settings as `set net_mode / ip / ...`; the address and mode apply after an automatic restart, the host name at once. Refused (CIP 0x10) while the motors are enabled or anything moves |
| Identity Reset | Type 0 restarts; type 1 sets the network settings back to their defaults (DHCP, 192.168.0.110, default host name) and restarts. Refused like the settings |
| LED | Slow blink until a Class 1 connection is open in Run, then on |

Attribute 3 reports the mode **in use**: DHCP (2) only when a DHCP server supplied the address. A board set
to DHCP that found no server reports static (0); `eip` shows the stored mode.

### Status assembly 100 (`ECamSts`, 160 bytes)

Refreshed every 1 ms, so each packet carries a status at most that old; an explicit Get of assembly 100
attribute 3 reads it too. Little-endian, DINTs on 4-byte boundaries, scaled integers. The layout is
`Firmware/ECamFirmware/EipProtocol.h`.

| Byte | Type | Field |
|---|---|---|
| 0 | UINT | FirmwareRev (major × 256 + minor) |
| 2 | UINT | StatusFlags: 0 loaded, 1 stored, 2 enabled, 3 ready, 4 running, 5 timeline armed, 6 timeline done, 7 stopping, 8 jogging, 9 faulted, 10 estop open, 11 simulate, 12 PLC in control, 13 rejected, 14 waiting (0.10.0) |
| 4 | UINT | FaultCode: 0 none, 1 estop, 2 motor fault, 3 HLFB not asserted on enable (`fault=` names the motors: `HLFB not asserted on enable (M-2)`), 4 connection lost while the PLC had control, 5 a limit (0.9.0), 6 homing failed (0.9.1), 7 a timeline wait timed out (0.10.0) |
| 6 | UINT | RejectCode: why the last edge was refused, 0 once one is accepted (below) |
| 8 | UINT | EngineState: 0 empty, 1 disabled, 2 ready, 3 armed, 4 running, 5 jogging, 6 fault (as `state=`) |
| 10, 12 | UINT | CrankState (0 off, 1 following, 2 running, 3 stopping, 4 jogging), TimelineState (0 idle, 1 armed, 2 running, 3 stopping, 4 done, 5 jogging, 6 waiting) |
| 14 | UINT | Outputs IO-0..IO-5 the profile has on (in simulate: what they would be; the pins stay off) |
| 16, 20 | UDINT | ProfileHash (as `profile=`), Heartbeat (ms since boot) |
| 24, 28, 32 | DINT | CrankAngle (millidegrees, as `deg=`), CrankRev, CrankRpm (0.01 rpm) |
| 36, 40, 44 | DINT | TimelineTime (µs), TimelineCycle, TimelineOverride (0.01 %, ramped) |
| 48 | DINT | EncoderCount (or the simulated encoder) |
| 52, 54 | UINT | AxisCount, Inputs (bit 0 the trigger input, bit 1 estop OK) |
| 56..72 | DINT | Cut-length mode, µm: CutLength, CutLengthNext, CutLengthMin, CutGap, SyncLeft (−1 outside the sync) |
| 76 | UDINT | CommLossCount: connections the board timed out since boot, plus closes and Program while the PLC had control |
| 80, 84 | UINT, UDINT | WaitIndex (the timeline wait holding, 1-based, 0 none), WaitedMs (how long it has held) (0.10.0) |
| 96 + 16·i | 16 B | Axis *i*: CommandedCounts (DINT, as `a<i>=`), CommandedTravel (DINT, milli-units since the load: counts / counts-per-unit × 1000), AxisFlags (UINT: 0 used, 1 timeline, 2 enabled, 3 HLFB, 4 motor fault, 5 alerts, 6 electronic gear, 7 + limit tripped, 8 − limit tripped, 9 homed, 10 homing), Connector (UINT, M-*n*) |

### Command assembly 150 (`ECamCmd`, 64 bytes)

| Byte | Type | Field |
|---|---|---|
| 0 | UINT | ControlFlags (below) |
| 4 | DINT | CrankRpm, 0.01 rpm (driven crank). 0 = the profile's speed |
| 8 | DINT | TimelineOverride, 0.01 % (10000 = as drawn). 0 = the profile's |
| 12 | DINT | CutLength, µm, finished piece (cut-length mode). 0 = keep the current length |
| 16 | DINT | JogCrankTarget, millidegrees since the load (rev × 360000 + mdeg), followed while JOG is held |
| 20 | DINT | JogTimeTarget, µs of timeline time, followed while JOG is held |
| 24 | UDINT | WaitBits: bit n = PLC wait bit n, levels for the timeline's `plc` waits. No control needed (0.10.0) |
| 2, 28–63 | | reserved, 0 |

| Bit | Name | Acts |
|---|---|---|
| 0 | ENABLE | rising: `enable`; falling: `disable`, **always** |
| 1 | PLC_CONTROL | level: the PLC takes control (below) |
| 2 | START | rising: `start` |
| 3 | STOP | rising: `stop`, **always**: a timeline finishes its cycle (a second edge: ramp to rest at once) |
| 4 | ABORT | rising: `abort`, **always** |
| 5 | CLEAR_FAULT | rising: `clear` |
| 6 | SIMULATE | rising: `sim 1`; falling: `sim 0` |
| 7 | JOG | rising: `jog on` (the master stays put); held: follows the jog targets as they change; falling: `jog off`, **always** |
| 8 | TRIGGER | rising: starts the timeline, for a profile with `trig=eip` |
| 9 | HOME | rising: `axhome all` (0.9.1); STOP stops it |

- **Control.** The PLC is in control while its connection is open, it is in Run and it holds PLC_CONTROL. CLEAR_FAULT, SIMULATE, ENABLE, START, TRIGGER, JOG and HOME need control (RejectCode 11 otherwise) and act in that order within a packet. The console then answers `error: the PLC has control` to `enable`, `start`, `trigger`, `jog`, `rpm`, `override`, `length`, `sim`, `simenc`, `load`, `commit`, `forget`, axis jog and `axhome` (not `axhome stop`); `stop`, `abort`, `disable`, `axjog <i> disable` / `stop` and the rest still work. Releasing PLC_CONTROL hands over without stopping.
- **Hold a command bit for at least two RPIs.** The board sees only what a packet carries, so a shorter bit can fall between two packets and never arrive. Through a Micro850 at a 20 ms RPI, an ABORT held 5 ms was acted on 9 times in 30, held 20 ms 27 in 30, and held 30 ms or longer 30 in 30.
- **Edges count from the first packet.** Bits already set when the connection opens or the PLC goes to Run are not edges: nothing starts by itself on reconnect.
- **Setpoints** apply when they change, and the non-zero ones when the PLC takes control; CrankRpm or TimelineOverride changed back to 0 returns to the profile's value. Changed without control: RejectCode 11, not applied.
- **JOG:** set the targets to where the master is (CrankRev × 360000 + CrankAngle; TimelineTime) before raising JOG, then move them. A stop, an abort or a connection loss ends the jog too.
- **TRIGGER** latency: 0.2–1.6 ms from the packet reaching the board's network to the timeline at t = 0 (bench); the PLC's RPI comes on top. Use a wired trigger input where the timing matters (200 µs).
- **Connection loss** (closed, timed out, Program) while the PLC has control: a controlled stop as `stop` (an encoder master's followers let go at once), then the motors disable once they have settled (HLFB asserted on each, at most 500 ms; the whole 500 ms with `hlfb_check 0`); FaultCode 4 latches at once. Idle but enabled: the motors disable. Without control it is not a fault, but a held ENABLE dropping disables.
- **RejectCode:** 10 no profile, 11 not in control, 12 motors disabled, 13 busy (moving, or an upload in progress), 14 out of range (negative, or a jog target), 15 CrankRpm on an encoder master, 16 CutLength without cut-length mode, 17 CutLength under the shortest piece, 18 over the limit (rpm above the crank motor's max, override above 1000 %), 19 a fault is latched (or enabling latched HLFB, FaultCode 3), 20 estop open, 21 TRIGGER on a profile whose trigger is not EtherNet/IP, 22 START with an axis off its cam position (0.8.9; 13 while an axis jog moves): moved by axis jog, or a timeline axis left part-way through the cycle by a second STOP, ABORT, a fault or a connection loss. JOG the timeline to 0 (or `axjog <m> cam`) and START again. 23 START or TRIGGER with an axis that requires homing not homed (0.9.1). 24 START, TRIGGER or JOG with an axis on a limit (0.9.0).
- `status` adds `eip=none|idle|run|control`, and while a connection is open `eip_from=<ip> eip_rpi=<ms>`.

**A profile upload pauses the connection about 50 ms.** `commit` takes a few hundred ms (the tables, then
the flash). The adapter keeps running through the tables, but the flash erase stalls the processor for
about 40 ms. A connection timeout of 80 ms or more rides through it (RPI 10 ms × 8, or 20 ms × 4); RPI
10 ms × 4 does not. The heartbeat keeps counting during a commit; the rest of the status holds its last
values until the commit ends. `forget` pauses it the same way. A settings write stalls the processor about 9 ms.

`tools/ecam_bench` stands in for a PLC and for EtherNet/IP tools (`identity`, `scan`, `net`, `io`,
`status`, `setip`, `sethost`, `setdhcp`, `reset`) and runs the bench tests (`test`, `nettest`, `statustest`,
`cmdtest`, `motiontest`, `livetest`, `loadtest`, `edstest`, `uitest`, `soak`, `plcloss`). Add `--plc IP` to run a suite
through a real PLC.

## Not implemented yet

- Absolute engage, phase offset, engaging a crank axis at a chosen angle.
