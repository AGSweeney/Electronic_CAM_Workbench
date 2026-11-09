@echo off
rem Flash ECam_Firmware.bin (next to this file) to a ClearCore on USB.
rem Flashing erases the profile stored on the board. Tools > Update board firmware in the Workbench keeps it:
rem use this only if the Workbench can't be run.
echo Electronic CAM Workbench: flash the ClearCore firmware over USB.
echo Connect the ClearCore by USB. Flashing from here ERASES the profile stored on the board.
echo To keep it, close this and use Tools, Update board firmware in the Workbench.
echo.
call "%~dp0flash_clearcore.cmd" "%~dp0ECam_Firmware.bin"
echo.
pause
