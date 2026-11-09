/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// The whole project as an Excel workbook (.xlsx), with native Excel tables and charts:
//   Summary         project, master and axes at a glance, every check (limits, steps, cam drawing)
//   Master          the master settings and the output pulses
//   <axis> Profile  axis settings, the segment table (with per-segment peaks) and the analysis
//   <axis> Motion   sampled position / velocity / acceleration / jerk, with a chart of each
//   Timing          every axis on one master coordinate (crank and timeline apart), output lanes
//   <axis> Cam      crank axes: plate cam outline, pitch curve and pressure angle
// Written with libxlsxwriter (BSD-2-Clause).

#include "CamProfile.h"

#include <QString>

namespace io {

struct WorkbookOptions {
    int samples = 360;      // samples per cycle (crank revolution or timeline axis duration), plus the end point
    QString projectName;    // shown in the title; "Untitled" when empty
    QString projectPath;    // the .ecam file, if saved
};

bool ExportWorkbook(const ecam::CamProject &project, const WorkbookOptions &options, const QString &path,
                    QString *error);

}  // namespace io
