#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <atomic>
#include <csignal>
#include <unistd.h>
#include <algorithm>

// =====================================================================
// Replays a raw .C16 capture (interleaved int16 I/Q) through rpitx-ui's
// `sendiq` instead of a HackRF.
//
// The HackRF path (replay_hackrf.cpp) replays the capture's raw IQ
// verbatim at the capture's own *dial* frequency (27,125,000 Hz), because
// the real carrier (27,144,449 Hz) only appears as a residual offset tone
// baked into that IQ. rpitx/sendiq instead tunes its DMA clock generator
// directly to whatever frequency you give it -- so here we throw away
// that offset tone and just extract the OOK envelope (per-sample
// magnitude) from the capture, to be transmitted with the real carrier
// frequency dialed directly. Sample rate is left at the capture's native
// rate (e.g. 500000); sendiq auto-decimates internally if it's above its
// own 200000 sps ceiling, so no resampling is done here.
//
// Usage:
//   ./replay_rpitx <metadata.txt> <samples.c16> [--once]
//   pipe its stdout into: sudo sendiq -i - -s <sample_rate printed below> -f 27144449 -t float -p 7
// =====================================================================

static std::atomic<bool> g_run(true);
static void on_signal(int){ g_run = false; }

static const double REAL_CARRIER_HZ = 27144449.0; // see README: measured real carrier, independent of the capture's dial freq

struct Meta {
    double sample_rate = 0.0;
    double center_freq = 0.0;
};

static bool parse_meta(const std::string& txt, Meta& m){
    std::ifstream in(txt);
    if(!in) return false;
    std::string line;
    while(std::getline(in, line)){
        if(line.rfind("sample_rate=",0)==0){
            m.sample_rate = std::stod(line.substr(12));
        } else if(line.rfind("center_frequency=",0)==0){
            m.center_freq = std::stod(line.substr(17));
        }
    }
    return m.sample_rate > 0 && m.center_freq > 0;
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
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <metadata.txt> <samples.c16> [--once]\n";
        return 1;
    }
    std::string meta_path = argv[1];
    std::string c16_path  = argv[2];
    bool once = false;
    for (int i = 3; i < argc; ++i) if (std::string(argv[i]) == "--once") once = true;

    Meta m;
    if (!parse_meta(meta_path, m)) {
        std::cerr << "Failed to parse metadata\n";
        return 1;
    }
    std::cerr << "Input        : " << m.sample_rate << " sps @ " << m.center_freq
               << " Hz (dial freq; ignored -- envelope only, no offset tone)\n";
    std::cerr << "Recommended sendiq invocation:\n"
               << "  sudo sendiq -i - -s " << (uint64_t)m.sample_rate
               << " -f " << (uint64_t)REAL_CARRIER_HZ << " -t float -p 7"
               << (once ? "" : " -l") << "\n";

    std::ifstream f(c16_path, std::ios::binary);
    if (!f) { std::cerr << "Failed to open " << c16_path << "\n"; return 1; }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    const size_t CHUNK = 64 * 1024; // bytes, multiple of 4 (int16 I + int16 Q)
    std::vector<int16_t> raw(CHUNK / 2);
    std::vector<float>   out;
    out.reserve(CHUNK / 4 * 2);

    while (g_run) {
        f.read(reinterpret_cast<char*>(raw.data()), CHUNK);
        std::streamsize got = f.gcount();
        if (got <= 0 || (got % 4) != 0) {
            if (once) break;
            f.clear();
            f.seekg(0, std::ios::beg);
            continue;
        }
        size_t samples = static_cast<size_t>(got) / 4; // (I16,Q16) pairs
        out.clear();
        for (size_t s = 0; s < samples; ++s) {
            float i = raw[2*s] / 32768.0f;
            float q = raw[2*s + 1] / 32768.0f;
            float mag = std::min(1.0f, std::sqrt(i*i + q*q));
            out.push_back(mag); // I
            out.push_back(0.0f); // Q
        }
        if (!write_all(1, out.data(), out.size() * sizeof(float))) {
            std::cerr << "stdout write failed (pipe closed?)\n";
            break;
        }
    }
    return 0;
}
