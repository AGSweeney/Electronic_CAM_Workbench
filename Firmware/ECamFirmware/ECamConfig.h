/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef __ECAM_CONFIG_H__
#define __ECAM_CONFIG_H__

#define ECAM_FIRMWARE_NAME "ECam_Firmware"
#define ECAM_FIRMWARE_VERSION "0.11.4"
#define ECAM_FIRMWARE_MAJOR 0u         /* the same, as numbers (the status assembly's FirmwareRev) */
#define ECAM_FIRMWARE_MINOR 11u

#define ECAM_USB_BAUD 115200

/* Clear of ClearCNC (8888/8889/10040), ClearCoreROS (9200-9203) and ClearHPGL (9100/9300/9302). */
#define ECAM_TCP_CONSOLE_PORT 9500
#define ECAM_UDP_DISCOVERY_PORT 9502
#define ECAM_DISCOVERY_REQUEST "ECAM_DISCOVER?"
/* EtherNet/IP adapter (QuickEIP): identity and assemblies. See Firmware/PROTOCOL.md, EtherNet/IP. */
#define ECAM_EIP_VENDOR_ID 65000u
#define ECAM_EIP_DEVICE_TYPE 43u       /* generic device */
#define ECAM_EIP_PRODUCT_CODE 3u
/* CIP revision (identity, electronic key, EDS MajRev / MinRev): major 1 = this assembly layout (it changes
 * only if the layout does, incompatibly); minor = firmware major * 100 + minor. CIP majors are 1..127. Keep
 * Firmware/eds/ECam_ClearCore.eds in step. */
#define ECAM_EIP_REV_MAJOR 1u
#define ECAM_EIP_REV_MINOR (ECAM_FIRMWARE_MAJOR * 100u + ECAM_FIRMWARE_MINOR)
#define ECAM_EIP_PRODUCT_NAME "ECam ClearCore"
#define ECAM_EIP_HOST_PREFIX "ecam-"
#define ECAM_EIP_ASM_CMD 150u          /* O->T, PLC -> board */
#define ECAM_EIP_ASM_STS 100u          /* T->O, board -> PLC */
#define ECAM_EIP_ASM_CFG 151u          /* config, empty */
#define ECAM_EIP_CMD_SIZE 64u
#define ECAM_EIP_STS_SIZE 160u
#define ECAM_EIP_STS_REFRESH_MS 1u /* the status assembly is refreshed this often */

#define ECAM_MAX_LINE 192
#define ECAM_WAIT_USB_MS 3000u
#define ECAM_ENABLE_HLFB_WAIT_MS 1000u
#define ECAM_AXJOG_DEADMAN_MS 500u     /* a velocity axis jog not sent again in this long ramps to rest */
#define ECAM_AXJOG_MAX_CPS 53333.0     /* axis jog never faster: 4000 rpm of an 800-count motor */
#define ECAM_STOP_SETTLE_MS 500u       /* COMM_LOSS: the longest the motors stay enabled, at rest, to settle */

#endif
