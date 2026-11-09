/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// The project report (File > Generate report): a PDF for an engineer to review the project in detail.
// Drawn as vector graphics, Letter or A4 by locale:
//   cover        project, review status, key figures, contents
//   overview     system block diagram, motor and I/O map
//   cycle        machine-cycle flow chart (crank revolution, timeline run)
//   timing       every axis on one master coordinate, the output lanes
//   master       every master setting, with notes
//   per axis     settings and results, checks, segments, displacement / velocity / acceleration / jerk,
//                the plate cam and its pressure angle (crank axes)
//   outputs, checks, review and sign-off
// The figures and checks are the ones the app and the Excel workbook show (ProjectReview).

#include "CamProfile.h"

#include <QImage>
#include <QString>

#include <vector>

namespace io {

struct ReportOptions {
    QString projectName;  // the title; "Untitled" when empty
    QString projectPath;  // the .ecam file, if saved
    enum class Paper { Auto, Letter, A4 } paper = Paper::Auto;  // Auto: Letter in the US, else A4
};

bool ExportReport(const ecam::CamProject &project, const ReportOptions &options, const QString &path, QString *error);

// The report's pages as images at `dpi` (preview, tests), laid out exactly as in the PDF.
std::vector<QImage> RenderReportPages(const ecam::CamProject &project, const ReportOptions &options, double dpi);

}  // namespace io
