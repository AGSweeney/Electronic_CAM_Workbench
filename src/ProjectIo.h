/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Files: the .ecam project (JSON) and the exports for the firmware side.

#include "CamProfile.h"

#include <QString>
#include <QStringList>

#include <string>
#include <vector>

namespace io {

constexpr int kFormatVersion = 1;

bool SaveProject(const ecam::CamProject &project, const QString &path, QString *error);
bool LoadProject(const QString &path, ecam::CamProject *project, QString *error);
// The .ecam contents in memory (also what an upload carries so it can be read back from the board).
QByteArray ProjectToJson(const ecam::CamProject &project, bool indented);
bool ProjectFromJson(const QByteArray &json, ecam::CamProject *project, QString *error);

// Upload lines: the board's profile lines followed by "json <base64>" lines carrying the whole
// project (compressed), so Download gives back exactly what was uploaded.
std::vector<std::string> UploadLines(const ecam::CamProject &project);
// From the downloaded lines: the project copy when present (*exact = true), else a project rebuilt
// from the motion lines (*exact = false: drawing, mechanics and limits come back as defaults).
bool ProjectFromUpload(const QStringList &lines, ecam::CamProject *project, bool *exact, QString *error);

// One row per sample over one master revolution: angle, encoder counts, position, motor counts,
// and velocity / acceleration at the project's crank speed. `intervals` + 1 rows (0..360 deg).
bool ExportCsv(const ecam::CamProject &project, int axis, int intervals, const QString &path, QString *error);

// A C header with the motor-count table for one axis, for a first firmware experiment.
bool ExportHeader(const ecam::CamProject &project, int axis, int intervals, const QString &path, QString *error);

// The DXF drawings' units: lengths are converted from the axis's units (mm, cm, m, in); an axis in other units (deg,
// rev, counts) is drawn as it is, unitless.
enum class DxfUnits { Millimetre, Inch };

// Plate cam of a crank axis as DXF (R2000). Layers:
//   CAM_PROFILE  cam outline (closed polyline)     PITCH_CURVE  roller centre path
//   BASE_CIRCLE  base circle                       BORE         bore circle (if set)
//   MARKS        centre cross, 0 deg mark          NOTES        name, rotation, radii
// Cam frame as in core/CamGeometry.h: follower at 12 o'clock at crank 0 deg, cam turns clockwise.
struct CamDxfOptions {
    int pointsPerRev = 3600;
    bool pitchCurve = true;
    bool baseCircle = true;
    bool marks = true;
    bool notes = true;
    double boreDiameter = 0.0;  // 0 = none; in the axis's units
    DxfUnits units = DxfUnits::Millimetre;
};
bool ExportCamDxf(const ecam::CamProject &project, int axis, const CamDxfOptions &options, const QString &path,
                  QString *error);

// One axis's path (its displacement over one master cycle) as DXF (R2000): the master across (crank degrees or
// timeline seconds, at `perDegree` / `perSecond` drawing units each; 0: picked so the drawing is about twice as wide
// as it is tall, rounded to 1, 2, 2.5 or 5 times a power of ten), the position up. Layers: PATH (open polyline), AXES (the axes, ticks), NOTES (the name, the scales, and an
// "ECW-PATH ..." line for reading it back).
struct PathDxfOptions {
    DxfUnits units = DxfUnits::Millimetre;
    double perDegree = 0.0;  // drawing units across per crank degree (0: picked)
    double perSecond = 0.0;  // drawing units across per timeline second (0: picked)
    int samples = 3600;
    bool axes = true;
    bool notes = true;
};
bool ExportPathDxf(const ecam::CamProject &project, int axis, const PathDxfOptions &options, const QString &path,
                   QString *error);

}  // namespace io
