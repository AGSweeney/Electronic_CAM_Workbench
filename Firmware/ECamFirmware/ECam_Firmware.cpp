/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
/*
 * ECam_Firmware
 *
 * Electronic cam for the ClearCore. Profiles made in the Electronic CAM Workbench (ECamUI) are loaded over the console and run
 * from the 5 kHz sample interrupt:
 *
 *   crank axes      follow an encoder on the Encoder Adapter, or a crank motor this board drives
 *   timeline axes   run their own diagram against time from a shared t = 0, started by a trigger
 *                   (digital input, or the "trigger" command)
 *
 *   USB CDC     console
 *   TCP 9500    console
 *   UDP 9502    "ECAM_DISCOVER?"
 *   EtherNet/IP adapter (QuickEIP): TCP/UDP 44818, UDP 2222 (EipLink.h)
 *
 * A-9 is an optional estop (estop_a9); DI-6..DI-8 are the Encoder Adapter. IO-0..IO-5 can be
 * timeline outputs. See Firmware/PROTOCOL.md.
 */

#include "CamEngine.h"
#include "ClearCore.h"
#include "Console.h"
#include "ECamConfig.h"
#include "EipLink.h"
#include "Settings.h"
#include "Transport.h"

#include <stdio.h>

int main(void) {
    Delay_ms(300);
    TransportInitUsb();
    SettingsLoad();
    EngineInit();
    TransportUsbSendLine(ECAM_FIRMWARE_NAME " " ECAM_FIRMWARE_VERSION " ready");
    {
        /* The last uploaded profile, from flash. Motors stay disabled and nothing starts. */
        char info[96];
        char line[160];
        const char *err = EngineRestore(info, sizeof(info));
        snprintf(line, sizeof(line), err ? "no stored profile (%s)" : "stored profile loaded: %s", err ? err : info);
        TransportUsbSendLine(line);
    }
    TransportInitEthernet();

    while (true) {
        TransportLoopTick();
        TransportPoll();
        EipLinkPoll();
        EnginePoll();
    }
}
