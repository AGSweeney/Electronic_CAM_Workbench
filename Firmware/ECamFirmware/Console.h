/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef __ECAM_CONSOLE_H__
#define __ECAM_CONSOLE_H__

/*
 * Line console, the same on USB and on TCP 9500. A command's last reply line starts with "ok" or
 * "error:". Lines starting "TEL " are telemetry, pushed between replies. No echo and no prompt:
 * it is meant for programs (the Electronic CAM Workbench, scripts); a terminal works with local echo on.
 */

typedef void (*ConsoleOut)(const char *line);

void ConsoleBanner(ConsoleOut out);
/* Returns true when the session should end ("exit", TCP only). */
bool ConsoleHandleLine(const char *line, ConsoleOut out);

#endif
