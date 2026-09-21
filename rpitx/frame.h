#pragma once
#include <vector>
#include <cstdint>
#include <cmath>
#include <cstddef>
#include <algorithm>
#include <string>

// =====================================================================
// Shared OOK frame synthesis for the rpitx/rpitx-ui port.
//
// Same protocol/timing as ook_tx.cpp / hotas_drive.cpp (16-bit word
// MSB-first, pause HIGH + 16 bit-pulses, back-to-back with no inter-frame
// gap). The HackRF tools mix this envelope onto a subcarrier (FIF_HZ)
// because they dial the HackRF off-frequency and rely on that IF offset
// to land on the real 27,144,449 Hz carrier. rpitx/sendiq instead tunes
// its DMA clock generator directly to the target RF frequency, so here
// the baseband is just the envelope on I with Q=0 (no NCO needed).
//
// Output is raw interleaved float32 (I,Q) samples in the [-1,1] range
// sendiq expects with `-t float` (see sendiq.cpp: values are used as-is,
// no additional normalization applied).
// =====================================================================

static const double T0_US   = 311.0, T1_US = 809.0, TLOW_US = 193.0, IDLE_US = 1813.0;
static const double EDGE_US = 10.0;

// Word layout (MSB-first): A3 A2 A1 A0 N S E W A B X Y ?12 ?13 SL ?15
static const uint16_t BIT_N = 1u<<11, BIT_S = 1u<<10, BIT_E = 1u<<9, BIT_W = 1u<<8;
static const uint16_t BIT_A = 1u<<7,  BIT_B = 1u<<6,  BIT_X = 1u<<5, BIT_Y = 1u<<4;

struct Osc { float env = 0.0f; };

inline size_t us2n(double us, double fs){
    long v = std::lround(us * 1e-6 * fs);
    return static_cast<size_t>(std::max<long>(1, v));
}

inline void write_env_segment_f(std::vector<float>& iq, size_t n, double fs, Osc& osc,
                                 float env_target, double edge_us)
{
    const double tau = (edge_us > 0.0) ? (edge_us * 1e-6) : 0.0;
    const double k   = (tau > 0.0) ? (1.0 - std::exp(-1.0 / (fs * tau))) : 1.0;
    iq.reserve(iq.size() + n * 2);
    for (size_t i = 0; i < n; ++i) {
        osc.env += static_cast<float>(k * (env_target - osc.env));
        iq.push_back(osc.env); // I
        iq.push_back(0.0f);    // Q
    }
}

inline void write_high_then_low_f(std::vector<float>& iq, size_t s_high, size_t s_low,
                                   double fs, double edge_us, Osc& osc)
{
    write_env_segment_f(iq, s_high, fs, osc, 1.0f, edge_us);
    if (s_low) write_env_segment_f(iq, s_low, fs, osc, 0.0f, edge_us);
}

inline void build_frame_f(std::vector<float>& out, uint16_t word, double fs, Osc& osc){
    out.clear();
    const size_t s_t0 = us2n(T0_US, fs), s_t1 = us2n(T1_US, fs);
    const size_t s_tL = us2n(TLOW_US, fs), s_idle = us2n(IDLE_US, fs);
    out.reserve((s_idle + (s_t1 + s_tL) * 16) * 2);
    write_high_then_low_f(out, s_idle, s_tL, fs, EDGE_US, osc);
    for (int i = 0; i < 16; ++i) {
        const int bit = (word >> (15 - i)) & 1;
        const size_t sH = bit ? s_t1 : s_t0;
        write_high_then_low_f(out, sH, s_tL, fs, EDGE_US, osc);
    }
}

inline std::string decode_word(uint16_t w){
    std::string s = "addr=" + std::to_string((w>>12)&0xF) + " [";
    bool first = true;
    auto add = [&](uint16_t bit, const char* name){
        if (w & bit) { if(!first) s += ","; s += name; first = false; }
    };
    add(BIT_N,"N"); add(BIT_S,"S"); add(BIT_E,"E"); add(BIT_W,"W");
    add(BIT_A,"A"); add(BIT_B,"B"); add(BIT_X,"X"); add(BIT_Y,"Y");
    if (first) s += "idle";
    s += "]";
    return s;
}
