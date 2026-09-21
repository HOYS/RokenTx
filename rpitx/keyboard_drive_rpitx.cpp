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

#include <termios.h>
#include <unistd.h>

// =====================================================================
// Live drive for the Rokenbok-style 27MHz toy (address 3), controlled
// from an interactive SSH session's keyboard instead of a joystick.
// Meant for a Pi with no joystick attached -- run this over SSH and
// press keys directly; it puts the terminal into raw mode to read
// keystrokes one at a time, with no Enter needed.
//
// Because a terminal only tells you when a key is PRESSED, not when
// it's released (unlike the joystick API), there's no "held" state to
// read each poll. Instead each bit is a TOGGLE: press a key once to set
// that bit (as if held down), press it again to release it. The current
// word is echoed to stderr every time it changes.
//
// Key bindings:
//   w/a/s/d  -> N/W/S/E (movement)
//   1/2/3/4  -> A/B/X/Y (buttons)
//   space    -> release everything (idle)
//   q        -> quit
//
// Streams raw float32 I/Q to stdout for rpitx-ui's sendiq, same as
// hotas_drive_rpitx.cpp:
//   ./keyboard_drive_rpitx | sudo sendiq -i - -s 200000 -f 27144449 -t float -p 7
//
// --dry-run prints decoded word changes instead of writing binary, so
// the key mapping can be verified without rpitx/a Pi (just needs a
// real terminal, since raw mode requires a tty on stdin).
// =====================================================================

static std::atomic<bool> g_run(true);
static void sigint_handler(int){ g_run = false; }
static const double FS = 200000.0; // sendiq's MAX_SAMPLERATE is 200000; stay at/under it

static struct termios g_saved_termios;
static bool g_termios_saved = false;

static void restore_termios(){
    if (g_termios_saved) tcsetattr(STDIN_FILENO, TCSANOW, &g_saved_termios);
}

static bool enable_raw_mode(){
    if (!isatty(STDIN_FILENO)) return false;
    if (tcgetattr(STDIN_FILENO, &g_saved_termios) != 0) return false;
    g_termios_saved = true;
    struct termios raw = g_saved_termios;
    raw.c_lflag &= ~(ICANON | ECHO); // no line buffering, no local echo
    raw.c_cc[VMIN]  = 0;             // read() returns immediately...
    raw.c_cc[VTIME] = 0;             // ...even with zero bytes available (polling)
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    return true;
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

int main(int argc, char** argv){
    int addr = 3;
    bool dry_run = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--addr" && i+1 < argc) addr = std::atoi(argv[++i]);
        else if (a == "--dry-run") dry_run = true;
    }

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    if (!enable_raw_mode()) {
        std::cerr << "stdin is not a tty -- run this interactively over SSH, "
                     "not with input piped/redirected.\n";
        return 1;
    }
    std::atexit(restore_termios);

    std::cerr << "[+] Keyboard control ready. w/a/s/d=N/W/S/E  1/2/3/4=A/B/X/Y  "
                 "space=release all  q=quit\n";
    if (!dry_run) {
        std::cerr << "[+] Streaming float32 I/Q @ " << (uint64_t)FS
                   << " sps to stdout for sendiq.\n";
    }

    uint16_t held_mask = 0;
    uint16_t last_word = 0xFFFF;
    Osc osc;
    std::vector<float> frame;

    while (g_run) {
        char c;
        ssize_t n = ::read(STDIN_FILENO, &c, 1);
        if (n == 1) {
            switch (c) {
                case 'w': held_mask ^= BIT_N; break;
                case 's': held_mask ^= BIT_S; break;
                case 'a': held_mask ^= BIT_W; break;
                case 'd': held_mask ^= BIT_E; break;
                case '1': held_mask ^= BIT_A; break;
                case '2': held_mask ^= BIT_B; break;
                case '3': held_mask ^= BIT_X; break;
                case '4': held_mask ^= BIT_Y; break;
                case ' ': held_mask = 0; break;
                case 'q': g_run = false; break;
                default: break;
            }
        }

        uint16_t w = (uint16_t)((addr & 0xF) << 12) | held_mask;

        if (dry_run) {
            if (w != last_word) {
                std::cout << "word=0x" << std::hex << w << std::dec
                           << "  " << decode_word(w) << "\r\n";
                std::cout.flush();
                last_word = w;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        if (w != last_word) {
            std::cerr << "word=0x" << std::hex << w << std::dec
                       << "  " << decode_word(w) << "\r\n";
            last_word = w;
        }
        build_frame_f(frame, w, FS, osc);
        if (!write_all(1, frame.data(), frame.size() * sizeof(float))) {
            std::cerr << "stdout write failed (pipe closed?)\r\n";
            break;
        }
    }

    std::cerr << "\r\n[+] stopping...\r\n";
    return 0;
}
