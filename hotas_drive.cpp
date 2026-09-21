#include <libhackrf/hackrf.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <csignal>
#include <cmath>
#include <algorithm>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>

// =====================================================================
// Live drive for the Rokenbok-style 27MHz toy (address 3) using a
// Thrustmaster T.A320 Pilot HOTAS stick, read via the Linux joystick API
// (/dev/input/js0), streamed continuously to a HackRF.
//
// Same protocol/timing as ook_tx.cpp (16-bit word MSB-first, pause HIGH +
// 16 bit-pulses, back-to-back with NO extra silence while a button is
// held -- see ook_tx.cpp for the derivation and the warm-up-per-frame bug
// that broke the original synth path). This program builds ONE frame at a
// time into a double-buffer; the joystick-polling thread rebuilds the
// currently-inactive buffer whenever the controller state changes and
// atomically flips which one the TX callback reads from, so button/axis
// changes take effect at the next frame boundary with no inter-frame gap.
// =====================================================================

static std::atomic<bool> g_run(true);
static void sigint_handler(int){ g_run = false; }

// -------------------------- protocol/RF params (matches ook_tx.cpp; confirmed working) --------------------------
static const uint64_t FC_DIAL = 27125000;   // dial freq; real RF = FC_DIAL + FIF_HZ
static const uint64_t FS      = 10000000;
static const double   FIF_HZ  = 19449.0;    // -> effective RF 27,144,449 Hz
static const double   T0_US   = 311.0, T1_US = 809.0, TLOW_US = 193.0, IDLE_US = 1813.0;
static const double   EDGE_US = 10.0;
static const int8_t   AMP     = 110;
static const int8_t   FLOOR   = 0;
static const int      VGA     = 30;
static const int      AMP_ON  = 1;

// -------------------------- button bit masks (matches ook_tx.cpp) --------------------------
static const uint16_t BIT_N = 1u<<11, BIT_S = 1u<<10, BIT_E = 1u<<9, BIT_W = 1u<<8;
static const uint16_t BIT_A = 1u<<7,  BIT_B = 1u<<6,  BIT_X = 1u<<5, BIT_Y = 1u<<4;

static inline int8_t clip_i8(double x){
    if(x > 127) x = 127;
    if(x < -128) x = -128;
    return static_cast<int8_t>(std::lrint(x));
}

// --- Slewed-envelope writer with continuous NCO (ASK), same as ook_tx.cpp ---
struct Osc { double phase = 0.0; double env = 0.0; };

static void write_env_segment(std::vector<int8_t>& iq, size_t n, double fs, double fif,
                               Osc& osc, double env_target,
                               int8_t amp_hi, int8_t amp_floor, double edge_us)
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
        osc.env += k * (env_target - osc.env);
        if (w != 0.0) {
            osc.phase += w;
            if (osc.phase >= TWO_PI) osc.phase -= TWO_PI;
        }
        const double mag = Afloor + depth * osc.env;
        const double ci = (w == 0.0) ? 1.0 : std::cos(osc.phase);
        const double sq = (w == 0.0) ? 0.0 : std::sin(osc.phase);
        iq.push_back(clip_i8(mag * ci));
        iq.push_back(clip_i8(mag * sq));
    }
}

static void write_high_then_low(std::vector<int8_t>& iq, size_t s_high, size_t s_low,
                                 double fs, double fif, double edge_us, Osc& osc,
                                 int8_t amp_high, int8_t amp_low)
{
    write_env_segment(iq, s_high, fs, fif, osc, 1.0, amp_high, amp_low, edge_us);
    if (s_low) write_env_segment(iq, s_low, fs, fif, osc, 0.0, amp_high, amp_low, edge_us);
}

static inline size_t us2n(double us, double fs){
    long v = std::llround(us * 1e-6 * fs);
    return size_t(std::max<long>(1, v));
}

static void build_frame(std::vector<int8_t>& out, uint16_t word, Osc& osc){
    out.clear();
    const size_t s_t0 = us2n(T0_US, FS), s_t1 = us2n(T1_US, FS);
    const size_t s_tL = us2n(TLOW_US, FS), s_idle = us2n(IDLE_US, FS);
    out.reserve((s_idle + (s_t1+s_tL)*16) * 2);
    write_high_then_low(out, s_idle, s_tL, FS, FIF_HZ, EDGE_US, osc, AMP, FLOOR);
    for (int i = 0; i < 16; ++i) {
        const int bit = (word >> (15-i)) & 1;
        const size_t sH = bit ? s_t1 : s_t0;
        write_high_then_low(out, sH, s_tL, FS, FIF_HZ, EDGE_US, osc, AMP, FLOOR);
    }
}

// -------------------------- joystick reading (Linux js API) --------------------------
#pragma pack(push,1)
struct js_event { uint32_t time; int16_t value; uint8_t type; uint8_t number; };
#pragma pack(pop)
static const uint8_t JS_EVENT_BUTTON = 0x01;
static const uint8_t JS_EVENT_AXIS   = 0x02;
static const uint8_t JS_EVENT_INIT   = 0x80;
static const int DEADZONE = 10000;

struct Stick {
    int fd = -1;
    int16_t axes[16] = {0};
    uint8_t buttons[32] = {0};

    bool open_dev(const char* path){
        fd = ::open(path, O_RDONLY | O_NONBLOCK);
        return fd >= 0;
    }
    void poll(){
        js_event e;
        while (true) {
            ssize_t n = ::read(fd, &e, sizeof(e));
            if (n != (ssize_t)sizeof(e)) break;
            uint8_t kind = e.type & ~JS_EVENT_INIT;
            if (kind == JS_EVENT_AXIS && e.number < 16) axes[e.number] = e.value;
            else if (kind == JS_EVENT_BUTTON && e.number < 32) buttons[e.number] = (uint8_t)e.value;
        }
    }
    uint16_t word(int addr, bool invert_y) const {
        uint16_t w = (uint16_t)((addr & 0xF) << 12);
        int16_t x = axes[0];
        int16_t y = invert_y ? (int16_t)(-axes[1]) : axes[1];
        if (x > DEADZONE) w |= BIT_E; else if (x < -DEADZONE) w |= BIT_W;
        if (y < -DEADZONE) w |= BIT_N; else if (y > DEADZONE) w |= BIT_S;
        if (buttons[2]) w |= BIT_A;
        if (buttons[3]) w |= BIT_B;
        if (buttons[0]) w |= BIT_X;
        if (buttons[1]) w |= BIT_Y;
        return w;
    }
};

static std::string decode_word(uint16_t w){
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

// -------------------------- double-buffered TX callback --------------------------
static std::vector<int8_t> g_bufA, g_bufB;
static std::atomic<std::vector<int8_t>*> g_active{nullptr};
static std::vector<int8_t>* g_cb_last = nullptr;
static size_t g_ofs = 0;

static int tx_cb(hackrf_transfer* xfer){
    auto* buf = g_active.load(std::memory_order_acquire);
    if (!buf || buf->empty()) { std::memset(xfer->buffer, 0, xfer->valid_length); return 0; }
    if (buf != g_cb_last) { g_ofs = 0; g_cb_last = buf; }
    const size_t N = buf->size();
    size_t need = xfer->valid_length;
    size_t first = std::min(need, N - g_ofs);
    std::memcpy(xfer->buffer, buf->data() + g_ofs, first);
    if (need > first) std::memcpy(xfer->buffer + first, buf->data(), need - first);
    g_ofs = (g_ofs + need) % N;
    return 0;
}

int main(int argc, char** argv){
    int addr = 3;
    bool invert_y = false;
    bool dry_run = false;
    const char* js_path = "/dev/input/js0";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--addr" && i+1 < argc) addr = std::atoi(argv[++i]);
        else if (a == "--invert-y") invert_y = true;
        else if (a == "--dry-run") dry_run = true;
        else if (a == "--js" && i+1 < argc) js_path = argv[++i];
    }

    std::cout.setf(std::ios_base::unitbuf);
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    Stick stick;
    if (!stick.open_dev(js_path)) {
        std::cerr << "Failed to open " << js_path << "\n";
        return 1;
    }
    std::cout << "[+] Reading " << js_path << ". Ctrl+C to stop.\n";

    if (dry_run) {
        uint16_t last = 0xFFFF;
        while (g_run) {
            stick.poll();
            uint16_t w = stick.word(addr, invert_y);
            if (w != last) { std::cout << "word=0x" << std::hex << w << std::dec << "  " << decode_word(w) << "\n"; last = w; }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return 0;
    }

    if (hackrf_init() != HACKRF_SUCCESS) { std::cerr << "hackrf_init failed\n"; return 1; }
    hackrf_device* dev = nullptr;
    if (hackrf_open(&dev) != HACKRF_SUCCESS) { std::cerr << "hackrf_open failed\n"; return 1; }
    hackrf_set_sample_rate(dev, (double)FS);
    uint32_t bw = hackrf_compute_baseband_filter_bw((uint32_t)FS);
    hackrf_set_baseband_filter_bandwidth(dev, bw);
    hackrf_set_freq(dev, FC_DIAL);
    hackrf_set_txvga_gain(dev, VGA);
    hackrf_set_amp_enable(dev, AMP_ON ? 1 : 0);

    Osc osc;
    uint16_t init_word = (uint16_t)((addr & 0xF) << 12);
    build_frame(g_bufA, init_word, osc);
    g_active.store(&g_bufA, std::memory_order_release);
    bool active_is_A = true;

    if (hackrf_start_tx(dev, tx_cb, nullptr) != HACKRF_SUCCESS) {
        std::cerr << "hackrf_start_tx failed\n";
        hackrf_close(dev); hackrf_exit();
        return 1;
    }
    std::cout << "[TX] fc=" << FC_DIAL << " fif=" << FIF_HZ << " -> RF=" << (uint64_t)(FC_DIAL+FIF_HZ)
              << " Hz  fs=" << FS << "  vga=" << VGA << "dB amp=" << (AMP_ON?"on":"off") << "\n";

    uint16_t last_word = init_word;
    while (g_run) {
        stick.poll();
        uint16_t w = stick.word(addr, invert_y);
        if (w != last_word) {
            std::cout << "word=0x" << std::hex << w << std::dec << "  " << decode_word(w) << "\n";
            last_word = w;
            // Only rebuild+swap the TX buffer when the word actually changes --
            // one frame (pause+16 bits) takes ~14ms to transmit, which is longer
            // than the joystick poll interval below. Swapping on every poll tick
            // regardless of change was truncating every frame mid-flight before
            // it ever completed (the actual bug behind "not working"). While the
            // word is unchanged, the TX callback keeps looping the same buffer
            // via modulo wraparound, repeating complete frames back-to-back.
            std::vector<int8_t>& target = active_is_A ? g_bufB : g_bufA;
            build_frame(target, w, osc);
            g_active.store(&target, std::memory_order_release);
            active_is_A = !active_is_A;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "\n[TX] stopping...\n";
    hackrf_stop_tx(dev);
    hackrf_close(dev);
    hackrf_exit();
    return 0;
}
