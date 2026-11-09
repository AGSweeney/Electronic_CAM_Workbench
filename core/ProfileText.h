/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Text form of a CamProject for loading it onto the ClearCore over the console (USB or TCP).
// Portable C++14 (the ClearCore toolchain is GCC 6); shared by ECamUI (writer) and the firmware
// (reader), so both sides agree on the format and on the hash of what was sent.
//
//   load
//   master src=encoder cpr=4000 rpm=60 crank=0 ramp=60 max=600
//   timeline repeat=0 ovr=1 trig=input in=6 rise=1 bit=0
//   axis 0 master=crank m=1 cpu=800 start=0 closed=1 span=360 units=mm name=Pusher arm
//   seg 0 120 20 cycloidal
//   ...
//   out 2 0.5 0.75        output IO-2 on from 0.5 s to 0.75 s of timeline time
//   out 3 40 100 crank    output IO-3 on from 40 to 100 crank degrees, every revolution
//   commit
//
// Lines are at most kProfileMaxLine characters. Numbers use %.17g so doubles round-trip.
// The hash is FNV-1a over every line between "load" and "commit", each followed by '\n'.

#include "CamProfile.h"

#include <stdint.h>

#include <string>
#include <vector>

namespace ecam {

constexpr int kProfileMaxLine = 160;
// The whole upload text (it is kept in the board's flash, 32 KB less a header).
constexpr size_t kProfileMaxText = 32768 - 16;
// "json <base64>" lines carry a copy of the whole project (the app's .ecam, compressed) so it can be
// read back from the board exactly; the board stores and hashes them but does not use them.
constexpr const char *kProfileProjectLine = "json";
constexpr int kMaxAxes = kMotorConnectors;
constexpr int kMaxSegments = 64;  // per axis

// The lines between "load" and "commit".
std::vector<std::string> ProfileLines(const CamProject &project);

uint32_t ProfileHashBegin();
uint32_t ProfileHashLine(uint32_t hash, const char *line);
uint32_t ProfileHash(const std::vector<std::string> &lines);

// Reads the lines of one upload. Line() returns nullptr or an error message.
class ProfileReader {
public:
    void Begin();
    const char *Line(const char *line);
    // Checks and normalizes the project; nullptr or an error message.
    const char *Finish(CamProject *out);
    uint32_t Hash() const { return m_hash; }
    bool Active() const { return m_active; }
    // The exact text read (every line followed by '\n'), for storing and reading back.
    const std::string &Text() const { return m_text; }
    std::string TakeText() { return std::move(m_text); }

private:
    CamProject m_project;
    std::string m_text;
    uint32_t m_hash = 0;
    bool m_active = false;
    bool m_haveMaster = false;
    bool m_homeLine[kMotorConnectors] = {};  // a home line per axis
};

}  // namespace ecam
