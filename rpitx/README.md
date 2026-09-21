# rpitx-ui port

Transmits the same Rokenbok-style 27 MHz OOK protocol as the top-level
HackRF tools (`ook_tx` / `replay_hackrf` / `hotas_drive`), but using a
Raspberry Pi's GPIO4 as the RF output via
[rpitx-ui](https://github.com/IgrikXD/rpitx-ui) instead of a HackRF.

**This only runs on an actual Raspberry Pi** -- `rpitx-ui`'s `sendiq`
drives the Broadcom SoC's clock generator/DMA peripherals directly, which
don't exist on a normal PC. The three tools here were written and
compiled/tested (word encoding, joystick reading, envelope extraction)
on the dev PC since they're plain POSIX/C++ with no rpitx dependency --
but the actual RF hop through `sendiq` has not been tested on real
hardware yet. Try it and report back.

## How it fits together

Each tool here does **not** talk to any radio hardware itself. It just
streams raw `float32` interleaved I/Q samples (`I,Q,I,Q,...`, range
[-1,1]) to stdout. You pipe that into rpitx-ui's `sendiq`, which is the
part that actually owns the DMA/GPIO transmit path and needs root:

```
./some_tool | sudo sendiq -i - -s <rate> -f 27144449 -t float -p 7
```

This mirrors the double-buffer/callback design in `hotas_drive.cpp` --
there it's a HackRF TX callback pulling from a buffer; here it's a Unix
pipe, with `sendiq`'s own read-from-stdin loop providing the same
backpressure/pacing.

Why float I/Q with Q=0, instead of the HackRF tools' NCO-mixed-onto-an-
IF-offset approach? The HackRF tools dial off-frequency (27,125,000 Hz)
and rely on a 19,449 Hz IF offset baked into the signal to land on the
real carrier (27,144,449 Hz) -- a quirk of that hardware/setup. rpitx's
`sendiq` instead tunes its own clock generator directly to whatever
frequency you give it, so there's no need for that trick: the baseband
is just the OOK envelope on the I channel, Q always 0.

## Setup (on the Pi)

1. Connect a wire (or the [coax PCB](https://github.com/IgrikXD/rpitx-coax-pcb)) to GPIO4 as the antenna.
2. Install rpitx-ui:
   ```sh
   git clone https://github.com/IgrikXD/rpitx-ui
   cd rpitx-ui
   ./install.sh --skip-optional   # sendiq is a core target, no need for ft8_lib etc.
   ```
3. Copy this `rpitx/` directory to the Pi and build the tools here:
   ```sh
   g++ -O2 -std=c++17 -o hotas_drive_rpitx hotas_drive_rpitx.cpp -lpthread
   g++ -O2 -std=c++17 -o ook_tx_rpitx ook_tx_rpitx.cpp
   g++ -O2 -std=c++17 -o replay_rpitx replay_rpitx.cpp
   ```
   (No libhackrf/librpitx headers needed for these three -- only
   `sendiq`, installed system-wide by `install.sh`, needs `librpitx`.)

## Usage

**Live joystick control** (needs the joystick plugged into the Pi):
```sh
./hotas_drive_rpitx --dry-run          # sanity-check button/axis mapping, no TX
./hotas_drive_rpitx | sudo sendiq -i - -s 200000 -f 27144449 -t float -p 7
```
Same button mapping as `hotas_drive.cpp`: buttons[2]=A, [3]=B, [0]=X, [1]=Y
(axis 0 = roll -> E/W, axis 1 = pitch -> N/S). Add `--invert-y` /
`--addr N` / `--js /dev/input/jsN` same as the HackRF version.

**Single command / hold a button:**
```sh
./ook_tx_rpitx --dry-run --btn A       # sanity-check the word, no TX
./ook_tx_rpitx --btn A | sudo sendiq -i - -s 200000 -f 27144449 -t float -p 7
./ook_tx_rpitx --word 0x3080 | sudo sendiq -i - -s 200000 -f 27144449 -t float -p 7
```

**Replay a real capture:**
```sh
./replay_rpitx ../BBD_0001.TXT ../BBD_0001.C16 | sudo sendiq -i - -s 500000 -f 27144449 -t float -p 7
```
`replay_rpitx` prints its own recommended `sendiq` invocation (sample
rate comes from the capture's metadata `.TXT`; `sendiq` auto-decimates
internally if that's above its 200000 sps ceiling, so no resampling is
done on this end). Add `--once` to stop at end-of-file instead of
looping, and drop the recommended `-l` flag from the sendiq command
accordingly.

## Tuning notes / open questions for real-hardware testing

- `-p 7` (max drive level) is a starting guess -- rpitx's amplitude
  control on GPIO is a coarse 3-bit pad-drive quantizer, not a true
  linear power level like a HackRF's VGA gain. May need tuning for
  range vs. spurious emissions.
- `-s 200000` for the live-control tools is the highest rate under
  sendiq's `MAX_SAMPLERATE` (200000); this gives 5 us/sample resolution
  against bit timings of 193-809 us, which should be plenty, but hasn't
  been verified against a real receive/decode of the Pi's own TX yet
  (the way `BBD_0001.C16` was cross-checked via RTL-SDR for the HackRF
  path).
- No IF offset / dial-frequency trick is used here (see above) --
  `sendiq -f 27144449` is assumed to tune precisely enough on its own.
  If the toy doesn't respond, checking the actual TX frequency with the
  RTL-SDR (same way the real carrier was originally measured) is the
  first thing to verify.
