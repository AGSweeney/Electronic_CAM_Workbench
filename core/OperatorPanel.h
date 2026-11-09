/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
#pragma once

// Operator inputs: a machine's own buttons on the ClearCore's inputs (Enable, Disable, Fault reset, Start, Stop), for a
// machine run with no Workbench and no PLC. The board's settings say which input is which, its contact (NC / NO) and
// whether Enable is a momentary button (with a Disable button) or a maintained switch.
//
// Sample() runs in the sample interrupt with the board's input levels: each input's "pressed" (closed for NO, open for
// NC) is filtered over 10 ms, as buttons bounce. Poll() runs in the main loop and returns what to do since the last call.
// Enable, Fault reset and Start act on becoming pressed, never on being pressed when the inputs are configured or the
// board starts: after Configure() nothing acts until the filter has settled and a change is seen. Disable and Stop act
// on becoming pressed, and while held they keep Enable and Start from acting; a maintained Enable switch disables on
// turning off. Header only, so the firmware and the PC tests share it. Portable C++14.

#include <stdint.h>

namespace ecam {

enum OpInput { kOpEnable = 0, kOpDisable, kOpReset, kOpStart, kOpStop, kOpInputs };
enum OpAction : uint8_t { kActEnable = 1, kActDisable = 2, kActReset = 4, kActStart = 8, kActStop = 16 };

class OperatorPanel {
public:
    static constexpr int kFilterSamples = 50;  // 10 ms at 5 kHz

    // pins: the board's input (0..12) for each OpInput, -1 none. ncMask: bit n, input n is normally closed. The caller
    // holds the sample interrupt off.
    void Configure(const int8_t pins[kOpInputs], uint8_t ncMask, bool maintained) {
        m_any = false;
        for (int i = 0; i < kOpInputs; ++i) {
            m_pin[i] = pins[i];
            m_count[i] = 0;
            m_any = m_any || pins[i] >= 0;
        }
        m_nc = ncMask;
        m_maintained = maintained;
        m_pressed = 0;
        m_settle = 2 * kFilterSamples;  // the filter reaches the real levels before anything counts as a change
        m_known = false;
    }
    bool Any() const { return m_any; }
    bool Maintained() const { return m_maintained; }
    int Pin(int input) const { return input >= 0 && input < kOpInputs ? m_pin[input] : -1; }

    // Sample interrupt: levels, bit n = the board's input n high.
    void Sample(uint16_t levels) {
        if (m_settle > 0) --m_settle;
        uint8_t pressed = m_pressed;
        for (int i = 0; i < kOpInputs; ++i) {
            if (m_pin[i] < 0) continue;
            const bool high = ((levels >> m_pin[i]) & 1u) != 0;
            const bool now = ((m_nc >> i) & 1u) ? !high : high;  // NC: pressed when open (low)
            const bool was = ((pressed >> i) & 1u) != 0;
            if (now != was) {
                if (++m_count[i] >= kFilterSamples) {
                    pressed = static_cast<uint8_t>(pressed ^ (1u << i));
                    m_count[i] = 0;
                }
            } else {
                m_count[i] = 0;
            }
        }
        m_pressed = pressed;
    }

    // Pressed now (filtered; op_test's forcing applied), bit n = OpInput n.
    uint8_t Pressed() const {
        uint8_t p = 0;
        for (int i = 0; i < kOpInputs; ++i) {
            if (m_pin[i] >= 0 && ((m_pressed >> i) & 1u)) p = static_cast<uint8_t>(p | (1u << i));
        }
        return static_cast<uint8_t>((p & ~m_testMask) | (m_testValue & m_testMask));
    }

    // Main loop: the actions since the last call (OpAction bits).
    uint8_t Poll() {
        const uint8_t p = Pressed();
        if (m_settle > 0 || !m_known) {
            m_prev = p;
            m_known = m_settle == 0;
            return 0;
        }
        const uint8_t rise = static_cast<uint8_t>(p & ~m_prev), fall = static_cast<uint8_t>(m_prev & ~p);
        m_prev = p;
        uint8_t act = 0;
        if (rise & (1u << kOpEnable)) act |= kActEnable;
        if (m_maintained && (fall & (1u << kOpEnable))) act |= kActDisable;
        if (rise & (1u << kOpDisable)) act |= kActDisable;
        if (rise & (1u << kOpReset)) act |= kActReset;
        if (rise & (1u << kOpStart)) act |= kActStart;
        if (rise & (1u << kOpStop)) act |= kActStop;
        if (p & (1u << kOpDisable)) act = static_cast<uint8_t>(act & ~kActEnable);  // Disable held: no enable
        if (p & (1u << kOpStop)) act = static_cast<uint8_t>(act & ~kActStart);      // Stop held: no start
        return act;
    }

    // Bench (op_test): force an input pressed (1) or released (0), or back to its wire (-1).
    void Test(int input, int state) {
        if (input < 0 || input >= kOpInputs) return;
        const uint8_t bit = static_cast<uint8_t>(1u << input);
        if (state < 0) {
            m_testMask = static_cast<uint8_t>(m_testMask & ~bit);
        } else {
            m_testMask = static_cast<uint8_t>(m_testMask | bit);
            m_testValue = static_cast<uint8_t>(state ? (m_testValue | bit) : (m_testValue & ~bit));
        }
    }

private:
    int8_t m_pin[kOpInputs] = {-1, -1, -1, -1, -1};
    uint8_t m_nc = 0;
    bool m_maintained = false;
    bool m_any = false;
    volatile uint8_t m_pressed = 0;
    uint8_t m_count[kOpInputs] = {};
    volatile int m_settle = 0;
    uint8_t m_prev = 0;
    bool m_known = false;
    volatile uint8_t m_testMask = 0, m_testValue = 0;
};

}  // namespace ecam
