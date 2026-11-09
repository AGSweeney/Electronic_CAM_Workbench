#!/bin/bash
# Builds TestMaster.bin against the libraries from Firmware/build.ps1, which must have run first.
# Flash it only to the second ClearCore: tools/flash_clearcore.cmd takes the .bin, and flashing the ECam board with it
# would erase its firmware and profile.
set -e
cd "$(dirname "$0")"
ROOT=../..
TC="/c/Program Files (x86)/Atmel/Studio/7.0/toolchain/arm/arm-gnu-toolchain/bin"
PACKS="C:/Program Files (x86)/Atmel/Studio/7.0/Packs"
OUT=build
mkdir -p "$OUT"

COMMON=(-mthumb -D__SAME53N19A__ -DNDEBUG -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O2 -ffunction-sections -fdata-sections -Wall
        -I"$PACKS/atmel/SAME53_DFP/1.1.118/include" -I"$PACKS/arm/CMSIS/5.4.0/CMSIS/Core/Include" -I"$PACKS/arm/CMSIS/4.5.0/CMSIS/Include"
        -I"$ROOT/libClearCore/inc" -I"$ROOT/LwIP/LwIP/src/include" -I"$ROOT/LwIP/LwIP/port/include"
        -I"$ROOT/QuickEIP/include" -I"$ROOT/QuickEIP/src" -I"$ROOT/QuickEIP/ports/clearcore")
STARTUP=$ROOT/Firmware/ECamFirmware/Device_Startup

objs=()
for f in "$ROOT"/QuickEIP/src/*.c "$ROOT"/QuickEIP/ports/lwip_raw/eip_hal_lwip_raw.c; do
    o="$OUT/$(basename "${f%.c}").o"
    "$TC/arm-none-eabi-gcc.exe" -x c -std=gnu99 "${COMMON[@]}" -c "$f" -o "$o"
    objs+=("$o")
done
for f in TestMaster.cpp "$ROOT"/QuickEIP/ports/clearcore/eip_hal_clearcore.cpp; do
    o="$OUT/$(basename "${f%.cpp}").o"
    "$TC/arm-none-eabi-g++.exe" -std=gnu++11 -fno-rtti -fno-exceptions "${COMMON[@]}" -c "$f" -o "$o"
    objs+=("$o")
done
"$TC/arm-none-eabi-g++.exe" -o "$OUT/TestMaster.elf" "${objs[@]}" -mthumb -Wl,-Map="$OUT/TestMaster.map" --specs=rdimon.specs \
    -Wl,--start-group -lm -larm_cortexM4lf_math -lClearCore -lLwIP -Wl,--end-group \
    -L"$STARTUP" -L"$PACKS/arm/CMSIS/4.5.0/CMSIS/Lib/GCC" -L"$ROOT/libClearCore/Release" -L"$ROOT/LwIP/Release" \
    -Wl,--gc-sections -mcpu=cortex-m4 -T"$STARTUP/flash_with_bootloader.ld" -mfloat-abi=hard -mfpu=fpv4-sp-d16
"$TC/arm-none-eabi-objcopy.exe" -O binary "$OUT/TestMaster.elf" "$OUT/TestMaster.bin"
"$TC/arm-none-eabi-size.exe" "$OUT/TestMaster.elf"
