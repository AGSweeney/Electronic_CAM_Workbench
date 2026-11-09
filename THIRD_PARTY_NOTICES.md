# Third-party notices

The Electronic CAM Workbench's own source code (`src/`, `core/`, `tests/`, `Firmware/ECamFirmware/`
except `Device_Startup/`, and the build scripts) is released under the [MIT License](LICENSE), Copyright
(c) 2026 Adam G. Sweeney. This repository also contains code and tools by others, which remain under their
own copyrights and licences. Changes made to them in this project stay under the licence of the file they
were made in.

| Component | Path | Copyright | Licence |
|-----------|------|-----------|---------|
| ClearCore library | `libClearCore/` (files with a Teknic header) | Teknic, Inc. | MIT: [`libClearCore/license.txt`](libClearCore/license.txt) |
| ClearCore library additions | `libClearCore/` (files with an Adam G. Sweeney header, e.g. `CoordinatedMotionController`, `LinearInterpolator`), and the sample callback / external step mode added to Teknic's `SysManager` and `MotorDriver` | Adam G. Sweeney | MIT |
| Microchip Advanced Software Framework (ASF) | `libClearCore/hal/`, `hpl/`, `hri/`, `usb/` | Microchip Technology Inc. and its subsidiaries | Microchip ASF licence, in each file's header (use with Microchip products) |
| Microchip device startup and linker scripts | `Firmware/ECamFirmware/Device_Startup/startup_same53.c`, `system_same53.c`, `same53n19a_flash.ld`, `same53n19a_sram.ld` | Microchip Technology Inc. | Apache-2.0 (`SPDX-License-Identifier` in each file) |
| Bootloader-aware linker scripts | `Firmware/ECamFirmware/Device_Startup/flash_with_bootloader.ld`, `flash_without_bootloader.ld` | Arduino LLC | LGPL-2.1 or later, in each file's header |
| lwIP TCP/IP stack | `LwIP/` | Swedish Institute of Computer Science and contributors | BSD-style, in each file's header |
| QuickEIP EtherNet/IP stack | `QuickEIP/` (see `QuickEIP/VENDOR.txt`) | Adam G. Sweeney | MIT: [`QuickEIP/LICENSE`](QuickEIP/LICENSE) |
| Qt 6 (linked, not included) | Installed by vcpkg; `windeployqt` copies its DLLs next to the exe | The Qt Company Ltd. and other contributors | LGPL-3.0 (dynamically linked) |
| libxlsxwriter (linked, not included) | Installed by vcpkg (`libxlsxwriter`); `xlsxwriter.dll` is copied next to the exe. Excel workbook export | John McNamara | BSD-2-Clause |
| zlib and minizip (linked, not included) | Installed by vcpkg as libxlsxwriter's dependencies; `zlib1.dll`, `minizip.dll` next to the exe | Jean-loup Gailly, Mark Adler; Gilles Vollant and contributors | zlib licence |
| ClearCore flash scripts | `tools/flash_clearcore.cmd`, `tools/flash_clearcore_loop.cmd` | Teknic, Inc. (authors Zachary Lebold, Nick Dawson) | Distributed with Teknic's ClearCore library (MIT) |
| BOSSA flash utility | `tools/bossac/bossac.exe` | ShumaTech (BOSSA), <https://github.com/shumatech/BOSSA> | BSD-3-Clause: [`tools/bossac/LICENSE`](tools/bossac/LICENSE), installed next to `bossac.exe` |
| UF2 conversion tool | `tools/uf2-builder/` | Distributed with Teknic's ClearCore library; UF2 format by Microsoft, <https://github.com/microsoft/uf2> | Upstream licence applies |

## Changes made here to third-party files

- **ClearCore library and lwIP:** `libClearCore/` and `LwIP/` are copies of the
  [Enhanced ClearCore Library](https://github.com/AGSweeney/EnhancedClearCoreLibrary) (branch
  `external-step-mode`) and match it file for file. Its `MODIFICATIONS.md` and `NEW_FILES.md` list the
  changes to Teknic files (USB serial, TCP, motor driver, system manager: sections 8 and 9 are the sample
  callback, external step mode and step-direction rule this firmware runs on) and the one change to a
  Microchip ASF file (`libClearCore/hpl/usb/hpl_usb.c`, endpoint direction), which remains under
  Microchip's licence.
- **Linker scripts:** `Firmware/ECamFirmware/Device_Startup/flash_with_bootloader.ld` and
  `flash_without_bootloader.ld` end the `FLASH` region 32 KB early, reserving 0x78000–0x7FFFF for the
  stored cam profile (`FlashStore.h`). They remain under the LGPL-2.1 (Arduino LLC); the change is in
  the files themselves, so their source travels with the firmware source.
