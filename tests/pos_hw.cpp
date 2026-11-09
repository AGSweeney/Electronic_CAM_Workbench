/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
// Reads a ClearPath's own position registers over its USB port. Never enables or moves the motor.
// "commanded" counts the step pulses the drive actually received, so comparing its change with the
// board's own count shows whether a pulse was added or lost on the way.
//
//   pos_hw <COMn>
//
// One line: serial, commanded, measured, error. Close ClearPath-MSP first (one program owns the port).

#include "ClearPathLink.h"

#include <QCoreApplication>
#include <QStringList>

#include <cstdio>

using namespace clearpath;

int main(int argc, char *argv[]) {
    QCoreApplication app(argc, argv);
    const QStringList a = app.arguments();
    if (a.size() < 2) {
        std::printf("usage: pos_hw <COMn>\n");
        return 2;
    }
    QString err;
    auto t = OpenSerial(a[1], &err);
    if (!t) {
        std::printf("could not open %s: %s\n", qPrintable(a[1]), qPrintable(err));
        return 2;
    }
    try {
        Link link(std::move(t));
        std::printf("serial %lld commanded %lld measured %lld error %lld\n", static_cast<long long>(link.get(params::SerialNumber)),
                    static_cast<long long>(link.get(params::PositionCommanded)), static_cast<long long>(link.get(params::PositionMeasured)),
                    static_cast<long long>(link.get(params::PositionError)));
    } catch (const std::exception &x) {
        std::printf("error: %s\n", x.what());
        return 1;
    }
    return 0;
}
