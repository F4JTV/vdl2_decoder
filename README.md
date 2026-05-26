# VDL Mode 2 Decoder for SDR++

A self-contained SDR++ plugin that decodes **VDL Mode 2** (VHF Data Link Mode 2),
the digital data link used in civil aviation for ACARS-over-AVLC and ATN/X.25
messaging. It demodulates the D8PSK signal directly from complex baseband,
performs forward error correction, reassembles AVLC frames and decodes embedded
ACARS messages, displaying them in a detachable message window with optional TSV
logging.

The decoder is a clean C++ port of the physical- and link-layer chain of
[dumpvdl2](https://github.com/szpajder/dumpvdl2) (GPLv3) and has **no external
runtime dependencies** — the Reed–Solomon codec is vendored, so no `libfec`,
`libacars` or `glib` are required.

## What is VDL Mode 2?

VDL2 is a 31.5 kbit/s digital link in the 136 MHz aeronautical band. Key
parameters:

| Parameter | Value |
|-----------|-------|
| Modulation | Differential 8-PSK (D8PSK), Gray-coded |
| Symbol rate | 10 500 symbols/s (3 bits/symbol → 31 500 bit/s) |
| Forward error correction | Reed–Solomon RS(255, 249) over GF(256) |
| Link layer | AVLC (Aviation VHF Link Control), HDLC-framed |
| Payload | ACARS, X.25/CLNP, XID |
| Common Signalling Channel | 136.975 MHz |

Other channels frequently in use include 136.700, 136.725, 136.775, 136.800,
136.825, 136.875 and 136.925 MHz (regional allocations vary).

## Features

- Direct complex-baseband D8PSK demodulation with preamble synchronisation.
- LFSR descrambling, block de-interleaving and Reed–Solomon error correction
  (with erasure handling for short final blocks).
- HDLC de-framing and AVLC parsing: source/destination DLC addresses and types,
  air/ground status, command/response, and link control field classification
  (I/S/U frames).
- ACARS decoding: registration, label, block id, technical acknowledgement,
  downlink message sequence and flight id, and free text. The ACARS Block Check
  Sequence is verified and structurally-malformed sub-blocks are rejected, so
  only well-formed messages are shown (matching dumpvdl2's behaviour).
- Detachable message window with a sortable table, auto-scroll, a clear button,
  a live noise-floor readout and a one-click channel selector.
- Optional TSV logging to a folder of your choice.
- Multiple independent instances (decode several channels at once).
- Settings are persisted across sessions.

## Building (Ubuntu 24.04)

The module builds as part of the SDR++ tree. From a checkout of
[SDR++](https://github.com/AlexandreRouma/SDRPlusPlus):

1. Place this folder at `decoder_modules/vdl2_decoder/` (the included
   `apply_to_sdrpp.sh` does this and wires up the build for you — see below).
2. Configure and build SDR++ as usual:

   ```sh
   cd SDRPlusPlus
   mkdir -p build && cd build
   cmake .. -DOPT_BUILD_VDL2_DECODER=ON
   make -j$(nproc)
   sudo make install
   ```

No extra system packages are needed beyond SDR++'s own build dependencies
(`build-essential`, `cmake`, `libfftw3-dev`, `libvolk-dev`, `libglfw3-dev`, …).

### Automated integration

Run the helper script from anywhere, passing the path to your SDR++ source tree:

```sh
./apply_to_sdrpp.sh /path/to/SDRPlusPlus
```

It is idempotent and performs three edits:

- copies the module into `decoder_modules/vdl2_decoder/`;
- adds the `OPT_BUILD_VDL2_DECODER` option and `add_subdirectory(...)` to the
  root `CMakeLists.txt`;
- registers `vdl2_decoder.so` in the default module list in
  `core/src/core.cpp` so it loads automatically.

Re-run `cmake`/`make` afterwards.

## Usage

1. Start SDR++ and start your SDR device.
2. Enable the **vdl2_decoder** module (Module Manager) if it is not already
   loaded; a new entry appears in the menu.
3. Pick a channel from the **Channel** dropdown (defaults to 136.975 MHz, the
   Common Signalling Channel). This tunes the radio for you. The VFO is locked
   to a 14 kHz bandwidth and produces the 105 kHz baseband the decoder needs.
4. Click **Show Messages** to open the message window. Decoded frames appear as
   aircraft transmit; the **Noise floor** readout helps you confirm the channel
   is centred.
5. To save traffic, tick **Log to file**, choose a folder, and a
   `vdl2_log.tsv` file will be appended to.

### Reading the table

| Column | Meaning |
|--------|---------|
| Time | Local time the frame was decoded |
| Src / Dst | 24-bit DLC addresses (hex) |
| Type | Source → destination address types (Aircraft / Ground station / …) |
| A/G | Air or Ground status of the addressed station |
| Proto | Higher-layer protocol (ACARS, X.25, XID, …) |
| Label | ACARS message label |
| Flight | Flight id (downlink ACARS) |
| Text | ACARS free-text payload |

## Notes & limitations

- The module needs a clean, correctly tuned signal; VDL2 bursts are short, so a
  good antenna and front-end help considerably.
- Higher network-layer PDUs (X.25 / CLNP) are identified and framed but their
  full contents are not decoded — the focus is ACARS-over-AVLC.
- Only FCS-valid AVLC frames are displayed; frames that Reed–Solomon cannot
  repair, and ACARS sub-blocks that fail structural/BCS validation, are dropped.

## License

This module reuses algorithms from dumpvdl2 and is distributed under the
**GPLv3**, consistent with that project.
