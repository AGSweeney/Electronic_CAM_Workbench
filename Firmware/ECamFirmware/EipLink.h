/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef __ECAM_EIP_LINK_H__
#define __ECAM_EIP_LINK_H__

/*
 * The EtherNet/IP adapter (QuickEIP, vendored in ../../QuickEIP; see Firmware/PROTOCOL.md, EtherNet/IP).
 *
 *   Identity      vendor 65000, generic device (43), product code 3, "ECam ClearCore", rev = firmware major.minor
 *   Assemblies    150 command (O->T, 64 bytes), 100 status (T->O, 160 bytes), 151 config (empty)
 *   TCP/IP        attrs 3 (static / DHCP), 5 (address), 6 (host name) settable while nothing moves; stored in
 *                 the board settings (the same ones as "set net_mode / ip / ..."), applied by a restart once
 *                 idle (the host name at once)
 *   Identity Reset  type 0 restarts, type 1 restores the network settings to their defaults and restarts
 *
 * The adapter starts once the network is up and runs in poll mode from the main loop. The ClearCore LED
 * blinks slowly until a PLC connection is open in Run, then stays on.
 */

#include <stddef.h>

/* Every main-loop pass: starts the adapter when the network is up, polls it, restarts when asked. */
void EipLinkPoll();
/* The host name in the settings changed ("set host"): it applies at once, as when set over CIP. */
void EipLinkHostChanged();
/* The "eip" console command: identity, connection, counters, network settings. */
void EipLinkPrint(void (*out)(const char *line));
/* The PLC has control: its Class 1 connection is open, it is in Run, and it holds PLC_CONTROL. The console
 * then refuses what would change the motion (start, enable, rpm, load ...); stop, abort, disable still work. */
bool EipLinkPlcInControl();
/* " eip=none|idle|run|control [eip_from=ip eip_rpi=ms]" for the console's status line. */
void EipLinkStatusTokens(char *buf, size_t len);

#endif
