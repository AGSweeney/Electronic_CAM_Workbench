/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
// How well can scope samples from different ClearPaths be lined up? Streams from several drives at once on
// one clock, and finds where each velocity changes sign. Give the drives the same cam, with sharp reversals,
// and those are the same instant on the machine, so their spread is the error of the time axis.
//
//   scope_sync <seconds> <COMn> [COMn ...] [--mon id]
//
// Two times for each sample: "arrival" is when it reached the PC, "spaced" evens them out as the scope does.
// Never enables or moves a motor. Close ClearPath-MSP and the scope first (one program owns a port).

#include "ClearPathLink.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QStringList>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace clearpath;

namespace {

struct Event {
    double arrival, spaced;
    int dir;
};

struct Drive {
    QString port;
    long long serial = 0;
    std::vector<double> arrival, spaced, value;
    long long steps = 0;  // sample periods by the 2-bit counter, lost ones included
    int lost = 0;
    double worstJump = 0.0;  // the furthest "spaced" ran from "arrival"
    int anchors = 0;
    std::string error;
    std::vector<Event> events;
};

void Record(Drive &d, const QElapsedTimer &clock, double seconds, int monId) {
    try {
        const MonVar *mv = FindMonVar(monId);
        QString err;
        std::unique_ptr<Transport> t = OpenSerial(d.port, &err);
        if (!t) throw std::runtime_error("could not open: " + err.toStdString());
        Link link(std::move(t));
        d.serial = link.get(params::SerialNumber);
        const MonState orig = link.monitorGet(1);
        link.setRate(Link::kStreamBaud);
        link.monitorSet(1, MonState{mv->id, mv->gain, 0, orig.sync});
        link.takeStream();
        link.scope(3);
        const double off = clock.elapsed() / 1000.0 - link.now();  // this link's clock onto the shared one

        StreamClock placed;  // as the scope
        double lastKeep = 0.0;
        const double end = clock.elapsed() / 1000.0 + seconds;
        while (clock.elapsed() / 1000.0 < end) {
            link.pump(3);
            const double now = link.now();
            if (now - lastKeep > 0.3) {
                link.keepalive();
                lastKeep = now;
            }
            for (const StreamSample &s : link.takeStream()) {
                const double arrival = s.t + off;
                const double tt = placed.place(arrival, s.counter);
                d.steps = static_cast<long long>(d.arrival.size()) + placed.lost();
                d.lost = placed.lost();
                d.anchors = placed.anchors();
                d.worstJump = std::max(d.worstJump, std::fabs(tt - arrival));
                d.arrival.push_back(arrival);
                d.spaced.push_back(tt);
                d.value.push_back(MonScale(*mv, s.raw));
            }
        }
        link.scope(0);
        link.scope(0);
        MonState back = orig;
        back.var &= ~Link::kMonSaveNv;
        link.monitorSet(1, back);
        link.setRate(Link::kBaud);
    } catch (const std::exception &e) {
        d.error = e.what();
    }
}

// Sign changes, with a dead band of half the usual speed so ripple near zero doesn't count. A slow drive's
// commanded velocity is steps, with spikes on it: the band comes from the median, not the peak, and each
// sample is the median of those around it, so a spike can't make or hide a reversal.
void FindEvents(Drive &d) {
    std::vector<double> mag;
    mag.reserve(d.value.size());
    for (double v : d.value) mag.push_back(std::fabs(v));
    if (mag.empty()) return;
    std::nth_element(mag.begin(), mag.begin() + mag.size() / 2, mag.end());
    const double band = mag[mag.size() / 2] * 0.5;
    if (band <= 0.0) return;
    constexpr size_t kHalf = 12;  // about 14 ms each side
    std::vector<double> near;
    int state = 0;
    size_t lastOut = 0;
    for (size_t i = kHalf; i + kHalf < d.value.size(); ++i) {
        near.assign(d.value.begin() + static_cast<std::ptrdiff_t>(i - kHalf), d.value.begin() + static_cast<std::ptrdiff_t>(i + kHalf + 1));
        std::nth_element(near.begin(), near.begin() + kHalf, near.end());
        const double v = near[kHalf];
        const int now = v > band ? 1 : v < -band ? -1 : 0;
        if (now == 0) continue;
        if (state != 0 && now != state && i > lastOut) {
            // The middle of the swing from the last sample on the old side to this one.
            d.events.push_back({(d.arrival[lastOut] + d.arrival[i]) / 2.0, (d.spaced[lastOut] + d.spaced[i]) / 2.0, now});
        }
        state = now;
        lastOut = i;
    }
}

struct Stats {
    double mean = 0, sd = 0, lo = 0, hi = 0;
    int n = 0;
};

Stats Measure(const std::vector<double> &x) {
    Stats s;
    s.n = static_cast<int>(x.size());
    if (x.empty()) return s;
    s.lo = *std::min_element(x.begin(), x.end());
    s.hi = *std::max_element(x.begin(), x.end());
    for (double v : x) s.mean += v;
    s.mean /= s.n;
    for (double v : x) s.sd += (v - s.mean) * (v - s.mean);
    s.sd = std::sqrt(s.sd / s.n);
    return s;
}

void Compare(const Drive &a, const Drive &b) {
    std::vector<double> da, ds;
    for (const Event &e : a.events) {
        const Event *best = nullptr;
        for (const Event &f : b.events) {
            if (f.dir == 0) continue;
            if (!best || std::fabs(f.arrival - e.arrival) < std::fabs(best->arrival - e.arrival)) best = &f;
        }
        if (!best || std::fabs(best->arrival - e.arrival) > 0.5) continue;
        da.push_back((best->arrival - e.arrival) * 1000.0);
        ds.push_back((best->spaced - e.spaced) * 1000.0);
    }
    const Stats sa = Measure(da), ss = Measure(ds);
    std::printf("%s vs %s: %d events\n", qPrintable(a.port), qPrintable(b.port), sa.n);
    std::printf("   arrival: mean %+.2f ms, sd %.2f, range %+.2f .. %+.2f\n", sa.mean, sa.sd, sa.lo, sa.hi);
    std::printf("   spaced:  mean %+.2f ms, sd %.2f, range %+.2f .. %+.2f\n", ss.mean, ss.sd, ss.lo, ss.hi);
    if (ds.size() >= 6) {  // does it wander through the run?
        const size_t third = ds.size() / 3;
        const std::vector<double> first(ds.begin(), ds.begin() + third), last(ds.end() - third, ds.end());
        std::printf("   spaced, first third %+.2f ms, last third %+.2f ms\n", Measure(first).mean, Measure(last).mean);
    }
}

}  // namespace

int main(int argc, char *argv[]) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    QCoreApplication app(argc, argv);
    QStringList a = app.arguments();
    int monId = 3;
    const int m = a.indexOf(QStringLiteral("--mon"));
    if (m >= 0 && m + 1 < a.size()) {
        monId = a[m + 1].toInt();
        a.remove(m, 2);
    }
    if (a.size() < 3 || !FindMonVar(monId)) {
        std::printf("usage: scope_sync <seconds> <COMn> [COMn ...] [--mon id]\n");
        return 2;
    }
    const double seconds = a[1].toDouble();
    std::vector<Drive> drives(a.size() - 2);
    for (size_t i = 0; i < drives.size(); ++i) drives[i].port = a[static_cast<int>(i) + 2];

    QElapsedTimer clock;
    clock.start();
    std::vector<std::thread> threads;
    for (Drive &d : drives) threads.emplace_back([&d, &clock, seconds, monId] { Record(d, clock, seconds, monId); });
    for (std::thread &t : threads) t.join();

    int failed = 0;
    for (Drive &d : drives) {
        if (!d.error.empty()) {
            std::printf("%s: FAILED %s\n", qPrintable(d.port), d.error.c_str());
            ++failed;
            continue;
        }
        FindEvents(d);
        const double span = d.arrival.empty() ? 0.0 : d.arrival.back() - d.arrival.front();
        std::printf("%s serial %lld: %zu samples, %d lost, %.3f samples/s by the counter, re-anchored %d times (furthest from arrival %.1f ms), "
                    "%zu reversals\n",
                    qPrintable(d.port), d.serial, d.arrival.size(), d.lost, span > 0 ? d.steps / span : 0.0, d.anchors,
                    d.worstJump * 1000.0, d.events.size());
    }
    for (size_t i = 0; i < drives.size(); ++i) {
        for (size_t j = i + 1; j < drives.size(); ++j) {
            if (drives[i].events.size() >= 2 && drives[j].events.size() >= 2) Compare(drives[i], drives[j]);
        }
    }
    return failed ? 1 : 0;
}
