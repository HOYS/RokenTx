#include <libhackrf/hackrf.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <atomic>
#include <csignal>
#include <thread>
#include <cmath>
#include <algorithm>
#include <getopt.h>

// =====================================================================
// Rokenbok-style 27 MHz OOK/PWM remote transmitter (HackRF, libhackrf).
//
// Protocol (reverse-engineered from BBD_0000/0001.C16 SDR captures AND
// cross-checked against a direct logic-analyzer capture of the real
// remote's keying line, key_order.jpg / timing.png):
//
//   - Real carrier sits at ~27.145 MHz, one of the six standard 27 MHz
//     R/C channels (26.995 / 27.045 / 27.095 / 27.145 / 27.195 / 27.255).
//     Earlier attempts in this repo defaulted to 27.125 MHz -- 20 kHz
//     off the real channel.
//   - 16-bit word, MSB first, sent as a "pause" (long HIGH) followed by
//     16 bit-pulses, each bit-pulse = HIGH (short=0 / long=1) + fixed LOW
//     gap, then the next frame's pause HIGH begins immediately (no extra
//     silence). While a button is held, the remote repeats this frame
//     continuously.
//   - Word layout (MSB first): A3 A2 A1 A0 N S E W A B X Y ?12 ?13 SL ?15
//       A3..A0 = 4-bit paired address/remote-ID nibble
//       N/S/E/W/A/B/X/Y/SL = one bit per button, set independently
//         (so diagonals / multi-button combos are just multiple bits set)
//       ?12/?13/?15 = bits seen in captures whose function is unconfirmed
//
// Timing constants below are the average of two independent
// logic-analyzer calibration runs (timing.png) and this project's own
// SDR envelope measurements -- all three agreed to within ~1%.
// =====================================================================

static std::atomic<bool> g_run(true);
static void sigint_handler(int){ g_run = false; }

// -------------------------- button bit masks --------------------------
// Bit position counted from the numeric 16-bit word (bit15 = first bit sent).
static const uint16_t BIT_N   = 1u << 11; // word 0x0800
static const uint16_t BIT_S   = 1u << 10; // 0x0400
static const uint16_t BIT_E   = 1u << 9;  // 0x0200
static const uint16_t BIT_W   = 1u << 8;  // 0x0100
static const uint16_t BIT_A   = 1u << 7;  // 0x0080
static const uint16_t BIT_B   = 1u << 6;  // 0x0040
static const uint16_t BIT_X   = 1u << 5;  // 0x0020
static const uint16_t BIT_Y   = 1u << 4;  // 0x0010
static const uint16_t BIT_R12 = 1u << 3;  // 0x0008 (unconfirmed)
static const uint16_t BIT_R13 = 1u << 2;  // 0x0004 (unconfirmed)
static const uint16_t BIT_SL  = 1u << 1;  // 0x0002
static const uint16_t BIT_R15 = 1u << 0;  // 0x0001 (unconfirmed)

struct ButtonDef { const char* name; uint16_t mask; };
static const ButtonDef BUTTONS[] = {
    {"N", BIT_N}, {"S", BIT_S}, {"E", BIT_E}, {"W", BIT_W},
    {"A", BIT_A}, {"B", BIT_B}, {"X", BIT_X}, {"Y", BIT_Y},
    {"SL", BIT_SL},
    {"R12", BIT_R12}, {"R13", BIT_R13}, {"R15", BIT_R15},
};

static std::string decode_word(uint16_t w){
    std::ostringstream out;
    uint8_t addr = (w >> 12) & 0xF;
    out << "addr=" << (int)addr << " buttons=[";
    bool first = true;
    for (const auto& b : BUTTONS) {
        if (w & b.mask) {
            if (!first) out << ",";
            out << b.name;
            first = false;
        }
    }
    out << "]";
    return out.str();
}

static std::vector<std::string> split(const std::string& s, char sep1, char sep2){
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == sep1 || c == sep2) {
            if (!cur.empty()) out.push_back(cur);
            cur.clear();
        } else cur += c;
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

static uint16_t parse_buttons(const std::string& spec){
    uint16_t mask = 0;
    for (auto tok : split(spec, ',', '+')) {
        for (auto& c : tok) c = std::toupper((unsigned char)c);
        bool found = false;
        for (const auto& b : BUTTONS) {
            if (tok == b.name) { mask |= b.mask; found = true; break; }
        }
        if (!found) {
            std::cerr << "[WARN] Unknown button name: " << tok << "\n";
        }
    }
    return mask;
}

// -------------------------- parameters --------------------------
struct Params {
    // NOTE: the baseband IQ carries a +fif_hz tone (see below), which the
    // HackRF up-converts to fc + fif_hz. The real measured channel is
    // ~27,145,000 Hz, so fc is deliberately set 20 kHz LOW of that --
    // effective RF = fc + fif_hz = 27,145,000 Hz. This also keeps the
    // desired signal off of HackRF's LO-leakage spur (which sits at fc).
    // Do not "correct" fc to 27145000 without also zeroing fif_hz, or the
    // real output ends up 20 kHz off-channel (learned the hard way).
    uint64_t fc = 27125000;     // dial frequency (Hz); real RF = fc + fif_hz
    uint64_t fs = 10000000;     // sample rate (Hz)

    uint16_t bits = 16;
    uint16_t addr = 3;          // 4-bit paired address (default: value seen in captures)
    uint16_t btn_mask = 0;      // OR of BIT_* button masks
    int64_t  word_override = -1; // if >=0, used verbatim instead of addr/btn_mask

    double t0_us   = 311.0;     // 0 -> short HIGH   (avg of logic + SDR measurements)
    double t1_us   = 809.0;     // 1 -> long  HIGH
    double tlow_us = 193.0;     // LOW between bits
    double idle_us = 1813.0;    // "pause" long HIGH marking start of frame
    double frame_us= 0.0;       // extra pad-to-fixed-period (0 = natural, back-to-back)

    int8_t  amp   = 110;        // HIGH peak amplitude (signed 8-bit)
    int8_t  floor = 0;          // LOW amplitude (0 => true OOK)
    int     vga   = 20;         // TX VGA 0..47
    int     amp_on= 0;          // 1 to enable RF amp (usually leave off)

    double fif_hz = 20000.0;    // tiny tone freq during HIGH (0 = DC)
    double edge_us= 10.0;       // envelope ramp time on transitions

    double warm_ms    = 2.0;    // steady carrier before first frame
    int    burst_n    = 1;      // how many frames per burst (>=1)
    double inter_ms   = 0.0;    // LOW pause between bursts (0 = back-to-back)

    bool dry_run = false;       // build + report, don't touch hardware
    std::string write_raw;      // if non-empty, dump interleaved int8 IQ here

    double dc_i = 0.0;          // baseband DC bias correction (cancels HackRF LO leakage)
    double dc_q = 0.0;
    bool   tone_only = false;   // if true, transmit a constant (dc_i,dc_q) tone forever
                                 // (no OOK framing) -- used for LO-leakage calibration

} P;

// ------------------------ sample helpers ------------------------
static inline int8_t clip_i8(double x){
    if(x > 127) x = 127;
    if(x < -128) x = -128;
    return static_cast<int8_t>(std::lrint(x));
}


// --- Slewed-envelope writer with continuous NCO (ASK) -----------------
static void write_env_segment(std::vector<int8_t>& iq,
                              size_t n, double fs, double fif,
                              double &phase,
                              double &env, double env_target,
                              int8_t amp_hi, int8_t amp_floor,
                              double edge_us)
{
    iq.reserve(iq.size() + n*2);

    const double TWO_PI = 6.283185307179586;
    const double w   = (fif == 0.0) ? 0.0 : TWO_PI * (fif / fs);
    const double tau = (edge_us > 0.0) ? (edge_us * 1e-6) : 0.0;
    const double k   = (tau > 0.0) ? (1.0 - std::exp(-1.0 / (fs * tau))) : 1.0;

    const double Ahi    = std::min(127, std::max(0, (int)amp_hi));
    const double Afloor = std::min(127, std::max(0, (int)amp_floor));
    const double depth  = std::max(0.0, Ahi - Afloor);

    for (size_t i = 0; i < n; ++i) {
        env += k * (env_target - env);                 // RC-slew envelope
        if (w != 0.0) {                                // continuous IF phase
            phase += w;
            if (phase >= TWO_PI) phase -= TWO_PI;
            else if (phase <= -TWO_PI) phase += TWO_PI;
        }
        const double mag = Afloor + depth * env;       // ASK magnitude
        const double ci = (w == 0.0) ? 1.0 : std::cos(phase);
        const double sq = (w == 0.0) ? 0.0 : std::sin(phase);
        iq.push_back( clip_i8(mag * ci + P.dc_i) );
        iq.push_back( clip_i8(mag * sq + P.dc_q) );
    }
}

// HIGH pulse then fixed LOW gap, using the slewed writer
static void write_high_then_low(std::vector<int8_t>& iq,
                                size_t s_high, size_t s_low,
                                double fs, double fif,
                                double edge_us,
                                double &phase,
                                double &env,
                                int8_t amp_high, int8_t amp_low)
{
    write_env_segment(iq, s_high, fs, fif, phase, env, 1.0, amp_high, amp_low, edge_us);
    if (s_low)
        write_env_segment(iq, s_low,  fs, fif, phase, env, 0.0, amp_high, amp_low, edge_us);
}


static void build_cyclic_buffer(std::vector<int8_t>& out, size_t target_seconds = 1)
{
    out.clear();

    auto us2s = [&](double us)->size_t {
        long v = std::llround(us * 1e-6 * double(P.fs));
        return size_t(std::max<long>(1, v));
    };

    const size_t s_t0   = us2s(P.t0_us);
    const size_t s_t1   = us2s(P.t1_us);
    const size_t s_tL   = us2s(P.tlow_us);
    const size_t s_idle = us2s(P.idle_us);
    const size_t s_frame_target = (P.frame_us > 0.0) ? us2s(P.frame_us) : 0;

    uint16_t code = (P.word_override >= 0) ? (uint16_t)P.word_override
                                            : (uint16_t)(((P.addr & 0xF) << 12) | P.btn_mask);

    // Build ONE frame (pause-high + bits + optional pad)
    std::vector<int8_t> frame;
    frame.reserve( (s_idle + (s_t1+s_tL)*P.bits + 4*s_tL) * 2 );

    double phase = 0.0;
    double env   = 0.0;

    // Pause HIGH marking start of frame, followed by the same fixed LOW
    // gap used between bits (confirmed present in real captures -- without
    // it the pause fuses with the first bit and shifts every frame's bits).
    write_high_then_low(frame, s_idle, s_tL, P.fs, P.fif_hz, P.edge_us, phase, env, P.amp, P.floor);

    // Bits MSB-first: 1 = long HIGH, 0 = short HIGH
    for (int i = 0; i < P.bits; ++i) {
        const int bit = ((code >> (P.bits-1-i)) & 1);
        const size_t sH = bit ? s_t1 : s_t0;
        write_high_then_low(frame, sH, s_tL, P.fs, P.fif_hz, P.edge_us, phase, env, P.amp, P.floor);
    }

    // Optional pad to a fixed total frame period, added as HIGH so the
    // frame still ends in an idle-HIGH region (off by default: real
    // remote does not do this, period is naturally code-dependent).
    if (s_frame_target > 0) {
        const size_t used = frame.size() / 2;
        if (used < s_frame_target) {
            const size_t pad = s_frame_target - used;
            write_high_then_low(frame, /*s_high=*/pad, /*s_low=*/0,
                                 P.fs, P.fif_hz, P.edge_us, phase, env,
                                 P.amp, /*amp_low=*/P.floor);
        }
    }

    // --- Build the full cyclic buffer: warm-up (once) + repeated frames ----
    // NOTE: the warm-up must NOT be inside the repeated unit -- earlier code
    // wrote it into `burst` and then repeated `burst` wholesale, which
    // injected a ~warm_ms silent gap before EVERY frame instead of once at
    // the start. That broke the real remote's expectation of back-to-back
    // frames with zero extra silence while a button is held (confirmed via
    // RTL-SDR: synth showed a bogus ~2167us LOW gap once per frame that
    // doesn't exist in real captures/replays).
    std::vector<int8_t> burst;
    burst.reserve(frame.size() * std::max(1, P.burst_n));

    const size_t s_inter = size_t(std::llround(P.inter_ms * 1e-3 * P.fs));
    for (int k = 0; k < std::max(1, P.burst_n); ++k) {
        burst.insert(burst.end(), frame.begin(), frame.end());
        if (s_inter)
            write_env_segment(burst, s_inter, P.fs, P.fif_hz, phase, env, 0.0, P.amp, P.floor, P.edge_us);
    }

    const size_t samples_total = std::max<uint64_t>(P.fs * target_seconds, 1);
    const size_t reps = std::max<size_t>(1, samples_total / (burst.size()/2));

    std::vector<int8_t> warm;
    const size_t s_warm = size_t(std::llround(P.warm_ms * 1e-3 * P.fs));
    if (s_warm > 0)
        write_env_segment(warm, s_warm, P.fs, P.fif_hz, phase, env, 0.0, P.amp, P.floor, P.edge_us);

    out.reserve(warm.size() + reps * burst.size());
    out.insert(out.end(), warm.begin(), warm.end());
    for (size_t r = 0; r < reps; ++r) out.insert(out.end(), burst.begin(), burst.end());
}


// -------------- HackRF TX callback over cyclic buffer --------------
static const std::vector<int8_t>* g_buf = nullptr;
static std::atomic<size_t>        g_ofs{0};

static int tx_callback(hackrf_transfer* xfer){
    const auto* B = g_buf;
    if(!B || B->empty()){
        std::memset(xfer->buffer, 0, xfer->valid_length);
        return 0;
    }
    const size_t N = B->size();
    size_t pos = g_ofs.load(std::memory_order_relaxed);
    size_t need = xfer->valid_length;

    size_t first = std::min(need, N - pos);
    std::memcpy(xfer->buffer, B->data() + pos, first);
    if(need > first){
        std::memcpy(xfer->buffer + first, B->data(), need - first);
    }
    g_ofs.store((pos + need) % N, std::memory_order_release);
    return 0;
}

// ----------------------------- CLI -----------------------------
static void print_buttons(){
    std::cerr << "Known button names: ";
    for (const auto& b : BUTTONS) std::cerr << b.name << " ";
    std::cerr << "\n(R12/R13/R15 are bits seen in captures whose real function is unconfirmed)\n";
}

static void usage(const char* prog){
    std::cerr <<
    "Usage: " << prog << " [options]\n"
    "  --addr N        4-bit paired address/remote-ID, 0..15 (default 3, from captures)\n"
    "  --btn LIST      Comma/plus separated button names, e.g. --btn N or --btn N+E\n"
    "  --word 0xHEX    Raw 16-bit word override (ignores --addr/--btn); also decoded & printed\n"
    "  --list-buttons  Print known button names and exit\n"
    "  --fc HZ         Center frequency (default 27145000 -- measured real channel)\n"
    "  --fs HZ         Sample rate (default 10000000)\n"
    "  --t0 US         Bit-0 HIGH (short) microseconds (default 311)\n"
    "  --t1 US         Bit-1 HIGH (long) microseconds (default 809)\n"
    "  --tlow US       LOW gap microseconds (fixed) (default 193)\n"
    "  --idle US       Pause HIGH marking frame start (default 1813)\n"
    "  --frame_us US   Force fixed frame period via HIGH padding (0 = natural, default)\n"
    "  --amp N         HIGH amplitude (signed 8-bit peak, default 110)\n"
    "  --floor N       LOW amplitude (0=true OOK, else ASK floor)\n"
    "  --fif HZ        Small in-band IF tone during HIGH (default 20000)\n"
    "  --edge_us US    Ramp time on edges (default 10)\n"
    "  --vga N         TX VGA gain 0..47 (default 20)\n"
    "  --amp_on 0|1    RF amp on/off (default 0)\n"
    "  --dry_run 0|1   Build the frame and print the decoded word, skip HackRF (default 0)\n"
    "  --write_raw F   Also dump interleaved int8 I/Q to file F (for hackrf_transfer or offline decode)\n"
    "  --dc_i N --dc_q N  Baseband DC bias correction added to every sample (cancels HackRF LO leakage)\n"
    "  --tone_only 0|1 Transmit constant (dc_i,dc_q) at the dial frequency forever -- for LO-leakage calibration\n";
}

int main(int argc, char** argv){
    static struct option opts[] = {
        {"addr", required_argument, 0, 0},
        {"btn", required_argument, 0, 0},
        {"word", required_argument, 0, 0},
        {"list-buttons", no_argument, 0, 0},
        {"fc", required_argument, 0, 0},
        {"fs", required_argument, 0, 0},
        {"t0", required_argument, 0, 0},
        {"t1", required_argument, 0, 0},
        {"tlow", required_argument,0, 0},
        {"idle", required_argument,0, 0},
        {"frame_us", required_argument,0,0},
        {"amp", required_argument,0,0},
        {"floor", required_argument,0,0},
        {"fif", required_argument,0,0},
        {"edge_us", required_argument,0,0},
        {"vga", required_argument,0,0},
        {"amp_on", required_argument,0,0},
        {"warm_ms", required_argument,0,0},
        {"burst_n", required_argument,0,0},
        {"inter_ms", required_argument,0,0},
        {"dry_run", required_argument,0,0},
        {"write_raw", required_argument,0,0},
        {"dc_i", required_argument,0,0},
        {"dc_q", required_argument,0,0},
        {"tone_only", required_argument,0,0},
        {0,0,0,0},
    };
    int idx=0;
    while(true){
        int c = getopt_long(argc, argv, "", opts, &idx);
        if(c==-1) break;
        if(c==0){
            std::string k = opts[idx].name;
            if(k=="addr")        P.addr = (uint16_t) std::stoul(optarg) & 0xF;
            else if(k=="btn")    P.btn_mask = parse_buttons(optarg);
            else if(k=="word")   P.word_override = std::stol(optarg, nullptr, 16);
            else if(k=="list-buttons"){ print_buttons(); return 0; }
            else if(k=="fc")     P.fc = std::stoull(optarg);
            else if(k=="fs")     P.fs = std::stoull(optarg);
            else if(k=="t0")     P.t0_us   = std::stod(optarg);
            else if(k=="t1")     P.t1_us   = std::stod(optarg);
            else if(k=="tlow")   P.tlow_us = std::stod(optarg);
            else if(k=="idle")   P.idle_us = std::stod(optarg);
            else if(k=="frame_us") P.frame_us = std::stod(optarg);
            else if(k=="amp")    P.amp   = (int8_t) std::stoi(optarg);
            else if(k=="floor")  P.floor = (int8_t) std::stoi(optarg);
            else if(k=="fif")    P.fif_hz = std::stod(optarg);
            else if(k=="edge_us")P.edge_us= std::stod(optarg);
            else if(k=="vga")    P.vga    = std::stoi(optarg);
            else if(k=="amp_on") P.amp_on = std::stoi(optarg);
            else if(k=="warm_ms") P.warm_ms = std::stod(optarg);
            else if(k=="burst_n") P.burst_n = std::stoi(optarg);
            else if(k=="inter_ms") P.inter_ms = std::stod(optarg);
            else if(k=="dry_run") P.dry_run = std::stoi(optarg) != 0;
            else if(k=="write_raw") P.write_raw = optarg;
            else if(k=="dc_i") P.dc_i = std::stod(optarg);
            else if(k=="dc_q") P.dc_q = std::stod(optarg);
            else if(k=="tone_only") P.tone_only = std::stoi(optarg) != 0;
        }
    }
    if(P.amp < 0) P.amp = 0;
    if(P.amp > 127) P.amp = 127;
    if(P.floor < 0) P.floor = 0;
    if(P.floor > P.amp) P.floor = P.amp;
    if(P.vga < 0) P.vga = 0;
    if(P.vga > 47) P.vga = 47;

    uint16_t code = (P.word_override >= 0) ? (uint16_t)P.word_override
                                            : (uint16_t)(((P.addr & 0xF) << 12) | P.btn_mask);

    std::cout << "[CFG] fc(dial)="<<P.fc<<" fif="<<P.fif_hz
              <<" -> effective RF="<<(uint64_t)(P.fc+P.fif_hz)<<" Hz"
              <<" fs="<<P.fs
              <<" word=0x"<<std::hex<<code<<std::dec
              <<"  (" << decode_word(code) << ")\n"
              <<"[CFG] t0="<<P.t0_us<<"us t1="<<P.t1_us<<"us tlow="<<P.tlow_us<<"us"
              <<" idle="<<P.idle_us<<"us frame_us="<<P.frame_us
              <<" amp="<<int(P.amp)<<" floor="<<int(P.floor)
              <<" fif="<<P.fif_hz<<"Hz edge_us="<<P.edge_us
              <<" vga="<<P.vga<<" amp_on="<<P.amp_on<<"\n";

    std::vector<int8_t> buf;
    if (P.tone_only) {
        // Calibration mode: transmit a constant (dc_i, dc_q) baseband value
        // forever, at the DIAL frequency (no fif rotation). If dc_i=dc_q=0,
        // this reveals HackRF's own LO-leakage carrier in isolation; sweeping
        // dc_i/dc_q lets you find the correction that cancels it.
        const size_t n = P.fs; // 1 second
        buf.resize(n*2);
        for (size_t i = 0; i < n; ++i) {
            buf[2*i]   = clip_i8(P.dc_i);
            buf[2*i+1] = clip_i8(P.dc_q);
        }
        std::cout << "[TONE_ONLY] dc_i=" << P.dc_i << " dc_q=" << P.dc_q
                  << " at dial freq " << P.fc << " Hz\n";
    } else {
        build_cyclic_buffer(buf, /*target_seconds=*/1);
    }
    if(buf.empty()){
        std::cerr << "Build failed: empty buffer\n";
        return 1;
    }
    std::cout << "[CFG] cyclic buffer built: " << buf.size()/2 << " complex samples\n";

    if (!P.write_raw.empty()) {
        std::ofstream f(P.write_raw, std::ios::binary);
        f.write(reinterpret_cast<const char*>(buf.data()), buf.size());
        f.close();
        std::cout << "[FILE] wrote " << P.write_raw << " (" << buf.size() << " bytes, interleaved int8 I/Q)\n";
        std::cout << "[TIP] hackrf_transfer -t " << P.write_raw << " -f " << P.fc
                  << " -s " << P.fs << " -x " << P.vga << " -a " << P.amp_on << " -R\n";
    }

    if (P.dry_run) {
        std::cout << "[DRY_RUN] Skipping HackRF; nothing transmitted.\n";
        return 0;
    }

    g_buf = &buf;
    g_ofs.store(0);

    if(hackrf_init()!=HACKRF_SUCCESS){ std::cerr<<"hackrf_init failed\n"; return 1; }
    hackrf_device* dev=nullptr;
    if(hackrf_open(&dev)!=HACKRF_SUCCESS){ std::cerr<<"hackrf_open failed\n"; hackrf_exit(); return 1; }

    if(hackrf_set_sample_rate(dev,(double)P.fs)!=HACKRF_SUCCESS){ std::cerr<<"set_sample_rate failed\n"; return 1; }
    uint32_t bw = hackrf_compute_baseband_filter_bw((uint32_t)P.fs);
    hackrf_set_baseband_filter_bandwidth(dev, bw);

    if(hackrf_set_freq(dev, P.fc)!=HACKRF_SUCCESS){ std::cerr<<"set_freq failed\n"; return 1; }
    hackrf_set_txvga_gain(dev, P.vga);
    hackrf_set_amp_enable(dev, P.amp_on ? 1 : 0);

    signal(SIGINT,  sigint_handler);
    signal(SIGTERM, sigint_handler);

    if(hackrf_start_tx(dev, tx_callback, nullptr)!=HACKRF_SUCCESS){
        std::cerr<<"hackrf_start_tx failed\n";
        hackrf_close(dev); hackrf_exit();
        return 1;
    }
    std::cout << "[TX] Streaming… Ctrl+C to stop.\n";

    while(g_run) std::this_thread::sleep_for(std::chrono::milliseconds(200));

    hackrf_stop_tx(dev);
    hackrf_close(dev);
    hackrf_exit();
    return 0;
}
