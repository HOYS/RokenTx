#include "frame.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <sstream>
#include <atomic>
#include <csignal>
#include <iostream>
#include <unistd.h>

// =====================================================================
// One-shot / held-button OOK transmitter, rpitx port of ook_tx.cpp.
//
// Builds the word from --addr/--btn (or a raw --word override) and
// streams the resulting frame on a loop to stdout as float32 I/Q,
// meant to be piped into sendiq:
//
//   ./ook_tx_rpitx --btn A | sudo sendiq -i - -s 200000 -f 27144449 -t float -p 7
//
// --dry-run prints the decoded word once and exits without writing
// binary, so it can be sanity-checked without rpitx/a Pi.
// =====================================================================

static std::atomic<bool> g_run(true);
static void sigint_handler(int){ g_run = false; }
static const double FS = 200000.0;

struct ButtonDef { const char* name; uint16_t mask; };
static const ButtonDef BUTTONS[] = {
    {"N", BIT_N}, {"S", BIT_S}, {"E", BIT_E}, {"W", BIT_W},
    {"A", BIT_A}, {"B", BIT_B}, {"X", BIT_X}, {"Y", BIT_Y},
};

static std::vector<std::string> split(const std::string& s, char sep1, char sep2){
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == sep1 || c == sep2) { if (!cur.empty()) out.push_back(cur); cur.clear(); }
        else cur += c;
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

static bool write_all(int fd, const void* buf, size_t n){
    const char* p = static_cast<const char*>(buf);
    while (n > 0) {
        ssize_t w = ::write(fd, p, n);
        if (w < 0) return false;
        p += w; n -= (size_t)w;
    }
    return true;
}

static void usage(const char* prog){
    std::cerr <<
        "Usage: " << prog << " [options]\n"
        "  --addr N        4-bit paired address, 0..15 (default 3)\n"
        "  --btn LIST      Comma/plus separated button names, e.g. --btn A or --btn N+E\n"
        "  --word 0xHEX    Raw 16-bit word override (ignores --addr/--btn)\n"
        "  --dry-run       Print the decoded word and exit, no stdout stream\n";
}

int main(int argc, char** argv){
    uint16_t addr = 3;
    uint16_t btn_mask = 0;
    int64_t word_override = -1;
    bool dry_run = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--addr" && i+1 < argc) addr = (uint16_t)(std::stoul(argv[++i]) & 0xF);
        else if (a == "--btn" && i+1 < argc) {
            for (const auto& name : split(argv[++i], ',', '+')) {
                bool found = false;
                for (const auto& b : BUTTONS) {
                    if (name == b.name) { btn_mask |= b.mask; found = true; break; }
                }
                if (!found) { std::cerr << "Unknown button: " << name << "\n"; return 1; }
            }
        }
        else if (a == "--word" && i+1 < argc) word_override = std::stoll(argv[++i], nullptr, 0);
        else if (a == "--dry-run") dry_run = true;
        else if (a == "--help" || a == "-h") { usage(argv[0]); return 0; }
        else { std::cerr << "Unknown arg: " << a << "\n"; usage(argv[0]); return 1; }
    }

    uint16_t word = (word_override >= 0)
                  ? (uint16_t)word_override
                  : (uint16_t)(((addr & 0xF) << 12) | btn_mask);

    std::cerr << "word=0x" << std::hex << word << std::dec << "  " << decode_word(word) << "\n";
    if (dry_run) return 0;

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);
    std::cerr << "[+] Streaming float32 I/Q @ " << (uint64_t)FS << " sps to stdout for sendiq. Ctrl+C to stop.\n";

    Osc osc;
    std::vector<float> frame;
    build_frame_f(frame, word, FS, osc);
    while (g_run) {
        if (!write_all(1, frame.data(), frame.size() * sizeof(float))) {
            std::cerr << "stdout write failed (pipe closed?)\n";
            break;
        }
    }
    std::cerr << "\n[+] stopping...\n";
    return 0;
}
