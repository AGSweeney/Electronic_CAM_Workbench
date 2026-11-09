/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef __ECAM_TRANSPORT_H__
#define __ECAM_TRANSPORT_H__

/*
 * Host links, all running the same console (Console.h):
 *
 *   USB CDC     console
 *   TCP 9500    console, one client
 *   UDP 9502    "ECAM_DISCOVER?" -> "ECAM ECam_Firmware IP=... CONSOLE=9500 FW=..."
 */

#include <stddef.h>
#include <stdint.h>

void TransportInitUsb();
void TransportInitEthernet();
void TransportPoll();

void TransportUsbSendLine(const char *line);
const char *TransportIpString();
bool TransportEthernetReady();
/* DHCP mode found no server and runs on the stored static address. */
bool TransportDhcpFellBack();
void TransportStats(char *buf, size_t len);
/* lwIP's heap and pools: use, high water and allocation failures since boot ("diag"). */
void TransportLwipStats(char *buf, size_t len);
/* Once per main-loop pass: the longest pass since the last TransportLoopStats() ("diag" reads and resets
 * it; a commit or an enable's HLFB wait is one long pass, by design). */
void TransportLoopTick();
void TransportLoopStats(char *buf, size_t len);
void TransportRestart();

/* "tel <ms>" on the link that asked (0 = off). Lines go out only when they fit in that link's send
 * buffer, so a host that stops reading never stalls the board. */
const char *TransportSetTelemetry(void (*out)(const char *), uint32_t periodMs);

#endif
