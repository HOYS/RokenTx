#include "frame.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <atomic>
#include <chrono>
#include <thread>
#include <csignal>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>

// =====================================================================
// Live drive for the Rokenbok-style 27MHz toy (address 3) using a
// joystick read via the Linux joystick API (/dev/input/js0), but
// transmitting through rpitx-ui's `sendiq` instead of a HackRF.
//
// This program does NOT talk to any radio hardware itself -- it just
// streams raw float32 I/Q samples to stdout, meant to be piped into
// sendiq (which owns the DMA/GPIO transmit path and needs root):
//
//   ./hotas_drive_rpitx | sudo sendiq -i - -s 200000 -f 27144449 -t float -p 7
//
// --dry-run prints decoded word changes instead of writing binary, so
// the joystick reading/mapping can be verified on any machine (no rpitx
// or Pi hardware needed for that part).
// =====================================================================

static std::atomic<bool> g_run(true);
static void sigint_handler(int){ g_run = false; }

static const double FS = 200000.0; // sendiq's MAX_SAMPLERATE is 200000; stay at/under it

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
    // Same mapping as hotas_drive.cpp: buttons[2]=A, [3]=B, [0]=X, [1]=Y.
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

static bool write_all(int fd, const void* buf, size_t n){
    const char* p = static_cast<const char*>(buf);
    while (n > 0) {
        ssize_t w = ::write(fd, p, n);
        if (w < 0) return false;
        p += w; n -= (size_t)w;
    }
    return true;
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

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    Stick stick;
    if (!stick.open_dev(js_path)) {
        std::cerr << "Failed to open " << js_path << "\n";
        return 1;
    }
    std::cerr << "[+] Reading " << js_path << ". Ctrl+C to stop.\n";
    if (!dry_run) {
        std::cerr << "[+] Streaming float32 I/Q @ " << (uint64_t)FS
                   << " sps to stdout for sendiq.\n";
    }

    Osc osc;
    std::vector<float> frame;
    uint16_t last_word = 0xFFFF;

    while (g_run) {
        stick.poll();
        uint16_t w = stick.word(addr, invert_y);

        if (dry_run) {
            if (w != last_word) {
                std::cout << "word=0x" << std::hex << w << std::dec
                           << "  " << decode_word(w) << "\n";
                last_word = w;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        if (w != last_word) {
            std::cerr << "word=0x" << std::hex << w << std::dec
                       << "  " << decode_word(w) << "\n";
            last_word = w;
        }
        build_frame_f(frame, w, FS, osc);
        if (!write_all(1, frame.data(), frame.size() * sizeof(float))) {
            std::cerr << "stdout write failed (pipe closed?)\n";
            break;
        }
    }

    std::cerr << "\n[+] stopping...\n";
    return 0;
}
