/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef __ECAM_SETTINGS_H__
#define __ECAM_SETTINGS_H__

/*
 * Board settings kept in the NVM user page (magic 'ECAM'). Cam profiles are not stored here:
 * they are loaded over the console and live in RAM until the next load or restart.
 *
 * On first boot the network, estop and HLFB-check settings are taken from a ClearHPGL blob
 * left on the board, so a bench that already works keeps working.
 */

#include <stddef.h>
#include <stdint.h>

#pragma pack(push, 1)
struct ECamSettings {
    uint32_t magic;
    uint16_t version;
    uint16_t size;

    uint8_t estopMode;  /* 0 off, 1 fault when the estop input is low, 2 fault when high (v1: DI-6, read as off) */
    uint8_t hlfbCheck;  /* wait for HLFB when enabling */
    uint16_t stopMs;    /* timeline stop ramp */

    uint8_t netMode;    /* 0 DHCP, 1 static (a DHCP failure falls back to the static address below) */
    uint8_t ip[4];
    uint8_t netmask[4];
    uint8_t gateway[4];

    /* Version 3: what EtherNet/IP tools set through the TCP/IP Interface object (attrs 5 and 6). */
    uint8_t dns1[4];
    uint8_t dns2[4];
    char domain[49];
    char host[65];      /* empty: "ecam-" + the last 3 MAC bytes */

    /* Version 4: the estop's input, as the trigger index (IO-0..IO-5, DI-6..DI-8, A-9..A-12). Versions 2 and 3: A-9. */
    uint8_t estopPin;

    /* Version 5: operator inputs (Enable, Disable, Fault reset, Start, Stop): each one's input, 0xFF none; bit n of opNc,
     * input n normally closed; opMode 0 momentary (Enable and Disable buttons), 1 maintained (an Enable switch). */
    uint8_t opPin[5];
    uint8_t opNc;
    uint8_t opMode;

    uint32_t checksum;
};
#pragma pack(pop)

extern ECamSettings g_settings;

void SettingsDefaults();
void SettingsLoad();
bool SettingsSave();
/* The network settings back to their defaults (EtherNet/IP Identity Reset type 1). Not saved. */
void SettingsNetworkDefaults();
bool SettingsFromNvm();
const char *SettingsImportedFrom();
/* The estop was turned off when version 1 settings (estop on DI-6) were loaded. */
bool SettingsEstopMoved();

/* The estop's input index, and its name (IO-0 .. A-12). */
const char *SettingsInputName(int pin);
/* Parses "A-9", "a9", "9": the input index, -1 when it isn't one. */
int SettingsParseInput(const char *s);
/* Console "set <key> <value>": nullptr on success or an error. Saves to NVM. The caller checks an estop_pin against the
 * loaded profile first (EngineEstopPinCheck). */
const char *SettingsSet(const char *key, const char *value, bool *needsRestart);
/* Operator inputs: the setting's name (op_enable...) for input n, and n for a name, -1 none. */
const char *SettingsOpName(int input);
int SettingsOpIndex(const char *key);
/* One "key value" line per setting. */
void SettingsPrint(void (*out)(const char *line));

#endif
