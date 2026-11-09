/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// The product name shown to users; the executable is ECamWorkbench.exe. Internal names stay "ECamUI"
// (the repository, the .ecam file format tag and the saved-settings location), so existing project
// files and saved layouts keep working.

#define ECAM_PRODUCT_NAME "Electronic CAM Workbench"
#define ECAM_PRODUCT_SHORT "CAM Workbench"
// The release (the app and the firmware it ships with, Firmware/ECamFirmware/ECamConfig.h); the installer
// (installer/build_installer.ps1) reads it from here.
#define ECAM_PRODUCT_VERSION "0.11.4"
