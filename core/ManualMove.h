/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// One motor moving on its own (axis jog): a velocity jog, or a move to a position, each with its own speed and
// acceleration, in motor counts. Integer fixed point (counts << 24 per sample), so it runs in the 5 kHz sample
// interrupt; one float square root per sample while a move decelerates. Header only: the firmware's build and the
// PC tests both take it as is. Portable C++14.
//
// The caller owns the position (whole counts): Tick() takes it and returns the new one; the difference is the
// sample's steps. Calls other than Tick() come from the main thread with the interrupt held off.

#include <math.h>
#include <stdint.h>

namespace ecam {

class ManualMove {
public:
    static constexpr int32_t kSampleHz = 5000;

    bool Active() const { return m_mode != 0; }
    bool VelocityJog() const { return m_mode == 1; }

    // Velocity jog toward countsPerSec (signed). 0 ramps to rest.
    void Jog(int64_t pos, double countsPerSec, double accel) {
        if (m_mode == 0 && countsPerSec == 0.0) return;
        Begin(pos, accel);
        m_vTarget = PerSample(countsPerSec);
        m_mode = 1;
    }
    // To a position, landing on it to the count without passing it.
    void MoveTo(int64_t pos, int64_t target, double countsPerSec, double accel) {
        if (m_mode == 0 && target == pos) return;
        Begin(pos, accel);
        m_target = target * kOne;
        m_vmax = PerSample(countsPerSec < 0 ? -countsPerSec : countsPerSec);
        if (m_vmax < 1) m_vmax = 1;
        m_mode = 2;
    }
    // Ramp to rest from a speed the motor already has (counts << 24 per sample, signed): an axis the cam let go of. The
    // deceleration is acc (counts << 24 per sample per sample).
    void StopFrom(int64_t pos, int64_t velQ24, int64_t accQ24) {
        m_pos = pos * kOne;
        m_vel = velQ24;
        m_vTarget = 0;
        m_acc = accQ24 < 1 ? 1 : accQ24;
        m_mode = 1;
    }
    // Ramp to rest at this deceleration (counts << 24 per sample per sample), at least as quick as the one it has.
    void StopRapid(int64_t accQ24) {
        if (m_mode == 0) return;
        m_vTarget = 0;
        if (accQ24 > m_acc) m_acc = accQ24;
        m_mode = 1;
    }
    // Ramp to rest at exactly this deceleration (counts/s^2), gentler or harder than the one it has.
    void StopAt(double accel) {
        if (m_mode == 0) return;
        m_vTarget = 0;
        m_acc = PerSample2(accel);
        m_mode = 1;
    }
    int64_t VelocityQ24() const { return m_vel; }
    // Ramp to rest.
    void Stop() {
        if (m_mode == 0) return;
        m_vTarget = 0;
        m_mode = 1;
    }
    // At once.
    void Halt() {
        m_mode = 0;
        m_vel = 0;
    }

    // One sample: the new position in whole counts.
    int64_t Tick(int64_t pos) {
        if (m_mode == 0) return pos;
        int64_t v;
        bool arrived = false;
        if (m_mode == 1) {
            v = StepToward(m_vel, m_vTarget, m_acc);
            if (v == 0 && m_vTarget == 0) m_mode = 0;
        } else {
            v = MoveVelocity(m_pos, m_target, m_vel, m_vmax, m_acc, &arrived);
            if (arrived) m_mode = 0;
        }
        m_pos += v;
        m_vel = arrived ? 0 : v;
        (void)pos;
        return m_pos >> 24;  // floor
    }

private:
    static constexpr int64_t kOne = int64_t(1) << 24;

    static int64_t PerSample(double perSec) { return static_cast<int64_t>(llround(perSec / kSampleHz * 16777216.0)); }
    static int64_t PerSample2(double perSec2) {
        const int64_t v = static_cast<int64_t>(llround(perSec2 / kSampleHz / kSampleHz * 16777216.0));
        return v < 1 ? 1 : v;
    }
    static int64_t StepToward(int64_t v, int64_t target, int64_t step) {
        if (v < target) return v + step < target ? v + step : target;
        if (v > target) return v - step > target ? v - step : target;
        return v;
    }
    // The fastest speed that can still stop on the target in whole samples (v + (v - acc) + ... <= the distance
    // left), capped at vmax, changed by at most acc per sample; the last step lands on the target.
    static int64_t MoveVelocity(int64_t pos, int64_t target, int64_t vel, int64_t vmax, int64_t acc, bool *arrived) {
        const int64_t err = target - pos;
        const int64_t mag = err < 0 ? -err : err;
        const int64_t vabs = vel < 0 ? -vel : vel;
        if (mag <= acc && vabs <= acc) {
            *arrived = true;
            return err;
        }
        *arrived = false;
        // Single precision: the sample interrupt's FPU has no double. The result only sets a speed.
        const float a = static_cast<float>(acc);
        float vd = a * (sqrtf(2.0f * static_cast<float>(mag) / a + 0.25f) - 0.5f);
        if (vd > static_cast<float>(mag)) vd = static_cast<float>(mag);
        if (vd > static_cast<float>(vmax)) vd = static_cast<float>(vmax);
        const int64_t want = err < 0 ? -static_cast<int64_t>(vd) : static_cast<int64_t>(vd);
        const int64_t v = StepToward(vel, want, acc);
        // A speed that is not a whole number of acc steps can't quite stop in them: land on the target.
        if ((err > 0 && v > err) || (err < 0 && v < err)) {
            *arrived = true;
            return err;
        }
        return v;
    }
    void Begin(int64_t pos, double accel) {
        if (m_mode == 0) {
            m_pos = pos * kOne;
            m_vel = 0;
        }
        m_acc = PerSample2(accel);
    }

    uint8_t m_mode = 0;  // 0 none, 1 velocity, 2 to a position
    int64_t m_pos = 0, m_vel = 0, m_vTarget = 0, m_target = 0, m_vmax = 1, m_acc = 1;
};

}  // namespace ecam
