#include <libhackrf/hackrf.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <atomic>
#include <csignal>
#include <thread>

// =====================================================================
// Replays a raw .C16 capture (interleaved int16 I/Q) directly out the
// HackRF, looping until Ctrl+C. Center frequency and sample rate are read
// from a companion metadata .TXT file (see BBD_0000.TXT / BBD_0001.TXT for
// the format: "center_frequency=..." / "sample_rate=..." lines).
//
// If the capture's sample rate is below TARGET_RATE, samples are
// upsampled by simple repetition (not interpolation) to reach it -- this
// is a crude but sufficient way to hit a HackRF-friendly output rate for
// an OOK signal, since it doesn't change the pulse shape, only how many
// samples represent it.
//
// Usage: ./replay_hackrf <metadata.txt> <samples.c16>
// =====================================================================

// ---------- Config knobs ----------
static const uint64_t TARGET_RATE = 2000000ULL; // 2 MS/s output if original < 2 MS/s
static const size_t   RING_BYTES  = 16 * 1024 * 1024; // 16 MiB ring buffer
static const int      TXVGA_GAIN  = 30;               // 0..47 dB; 30 is moderate
static const bool     RF_AMP_ON   = true;             // enable HackRF RF amp
// ----------------------------------

static std::atomic<bool> running(true);
static void on_signal(int){ running = false; }

// --- simple lock-free ring buffer for producer (file thread) -> consumer (TX callback)
struct Ring {
    std::vector<uint8_t> b;
    std::atomic<size_t>  w{0};
    std::atomic<size_t>  r{0};

    Ring(size_t n): b(n) {}
    size_t size() const { return b.size(); }

    // return free space (bytes we can write without overwriting unread data)
    size_t free_space() const {
        size_t rp = r.load(std::memory_order_acquire);
        size_t wp = w.load(std::memory_order_acquire);
        if (wp >= rp) return b.size() - (wp - rp) - 1;
        return rp - wp - 1;
    }
    size_t used() const {
        size_t rp = r.load(std::memory_order_acquire);
        size_t wp = w.load(std::memory_order_acquire);
        if (wp >= rp) return wp - rp;
        return b.size() - (rp - wp);
    }

    // write "n" bytes (assume caller checked free_space)
    void write(const uint8_t* src, size_t n){
        size_t wp = w.load(std::memory_order_relaxed);
        size_t N  = b.size();
        size_t first = std::min(n, N - wp);
        std::memcpy(&b[wp], src, first);
        if (n > first) std::memcpy(&b[0], src + first, n - first);
        w.store((wp + n) % N, std::memory_order_release);
    }

    // read exactly n bytes if available; return false if underrun
    bool read(uint8_t* dst, size_t n){
        if (used() < n) return false;
        size_t rp = r.load(std::memory_order_relaxed);
        size_t N  = b.size();
        size_t first = std::min(n, N - rp);
        std::memcpy(dst, &b[rp], first);
        if (n > first) std::memcpy(dst + first, &b[0], n - first);
        r.store((rp + n) % N, std::memory_order_release);
        return true;
    }
};

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

struct FeederCfg {
    std::string path_c16;
    int interp = 1; // repetition factor (e.g., 4 when going 500k -> 2M)
};

struct Feeder {
    Ring& ring;
    FeederCfg cfg;
    std::thread th;
    std::atomic<bool> ok{true};

    Feeder(Ring& r, const FeederCfg& c): ring(r), cfg(c) {}

    void start(){
        th = std::thread([this]{
            FILE* f = std::fopen(cfg.path_c16.c_str(), "rb");
            if(!f){ std::perror("fopen"); ok=false; return; }

            const size_t CHUNK_ORIG = 64*1024; // bytes from file (multiple of 4)
            std::vector<uint8_t>  raw(CHUNK_ORIG);
            std::vector<int16_t>  as16(CHUNK_ORIG/2);
            // worst-case upsample size
            std::vector<uint8_t>  up((CHUNK_ORIG/4) * (2 * cfg.interp)); 
            // Explanation: each orig "sample" = 4 bytes (I16,Q16) -> 2 bytes (I8,Q8)
            // then multiplied by interp.

            while(running){
                // if ring has little free space, sleep briefly
                if (ring.free_space() < up.size()){
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    continue;
                }

                size_t rd = std::fread(raw.data(), 1, raw.size(), f);
                if(rd == 0 || (rd % 4) != 0){
                    // rewind if EOF or misaligned read
                    std::clearerr(f);
                    std::fseek(f, 0, SEEK_SET);
                    continue;
                }
                // convert bytes->int16
                std::memcpy(as16.data(), raw.data(), rd);
                size_t samples = rd / 4; // (I16,Q16)
                // upsample into "up"
                size_t out_bytes = 0;
                for(size_t s=0; s<samples; ++s){
                    int16_t i16 = as16[2*s + 0];
                    int16_t q16 = as16[2*s + 1];
                    int8_t  i8  = static_cast<int8_t>(i16 >> 8);
                    int8_t  q8  = static_cast<int8_t>(q16 >> 8);
                    for(int r=0; r<cfg.interp; ++r){
                        if(out_bytes + 2 > up.size()){
                            // grow if needed (shouldn't happen with our sizing, but be safe)
                            up.resize(up.size() + 65536);
                        }
                        up[out_bytes++] = static_cast<uint8_t>(i8);
                        up[out_bytes++] = static_cast<uint8_t>(q8);
                    }
                }
                ring.write(up.data(), out_bytes);
            }
            std::fclose(f);
        });
    }
    void join(){ if(th.joinable()) th.join(); }
};

// Global playback ring & last sample for underrun padding
static Ring* g_ring = nullptr;
static uint8_t g_lastIQ[2] = {0,0};

static int tx_cb(hackrf_transfer* xfer){
    if(!g_ring) return 0;
    // Try to read exactly valid_length bytes. If not enough, pad with last IQ.
    if(!g_ring->read(xfer->buffer, xfer->valid_length)){
        // underrun: fill with last known IQ to avoid silence
        for(int i=0;i<xfer->valid_length;i+=2){
            xfer->buffer[i]   = g_lastIQ[0];
            xfer->buffer[i+1] = g_lastIQ[1];
        }
    } else {
        // store last IQ from end of buffer
        if(xfer->valid_length >= 2){
            g_lastIQ[0] = xfer->buffer[xfer->valid_length-2];
            g_lastIQ[1] = xfer->buffer[xfer->valid_length-1];
        }
    }
    return 0;
}

int main(int argc, char** argv){
    if(argc < 3){
        std::cerr << "Usage: " << argv[0] << " <metadata.txt> <samples.c16>\n";
        return 1;
    }
    std::string meta_path = argv[1];
    std::string c16_path  = argv[2];

    Meta m;
    if(!parse_meta(meta_path, m)){
        std::cerr << "Failed to parse metadata\n";
        return 1;
    }

    // Decide output rate & interpolation factor
    uint64_t out_rate = (m.sample_rate < TARGET_RATE) ? TARGET_RATE : static_cast<uint64_t>(m.sample_rate);
    int interp = (m.sample_rate < TARGET_RATE)
               ? static_cast<int>( (TARGET_RATE + m.sample_rate/2) / m.sample_rate ) // round
               : 1;
    if(interp < 1) interp = 1;
    out_rate = static_cast<uint64_t>(m.sample_rate * interp);

    std::cout << "Input  : " << m.sample_rate << " sps @ " << m.center_freq << " Hz\n";
    std::cout << "Output : " << out_rate       << " sps (interp x" << interp << ")\n";

    // Init HackRF
    if(hackrf_init() != HACKRF_SUCCESS){ std::cerr << "hackrf_init failed\n"; return 1; }
    hackrf_device* dev = nullptr;
    if(hackrf_open(&dev) != HACKRF_SUCCESS){ std::cerr << "hackrf_open failed\n"; return 1; }

    // Configure device
    if(hackrf_set_sample_rate(dev, out_rate) != HACKRF_SUCCESS){ std::cerr << "set_sample_rate failed\n"; return 1; }
    if(hackrf_set_freq(dev, static_cast<uint64_t>(m.center_freq)) != HACKRF_SUCCESS){ std::cerr << "set_freq failed\n"; return 1; }

    uint32_t bw = hackrf_compute_baseband_filter_bw(static_cast<uint32_t>(out_rate));
    hackrf_set_baseband_filter_bandwidth(dev, bw);

    hackrf_set_txvga_gain(dev, TXVGA_GAIN);
    hackrf_set_amp_enable(dev, RF_AMP_ON ? 1 : 0);

    // Prepare ring + feeder
    Ring ring(RING_BYTES);
    g_ring = &ring;
    FeederCfg cfg{c16_path, interp};
    Feeder feeder(ring, cfg);
    feeder.start();

    // Start TX
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);
    if(hackrf_start_tx(dev, tx_cb, nullptr) != HACKRF_SUCCESS){
        std::cerr << "hackrf_start_tx failed\n";
        running = false;
    }

    // Run until Ctrl-C
    while(running) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Cleanup
    hackrf_stop_tx(dev);
    feeder.join();
    hackrf_close(dev);
    hackrf_exit();
    return 0;
}