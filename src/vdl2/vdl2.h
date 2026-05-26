/*
 * vdl2.h - Self-contained VHF Data Link Mode 2 (VDL2) demodulator/decoder.
 *
 * Faithful C++ port of the physical / link-layer decoding chain of dumpvdl2
 * (Copyright (c) Tomasz Lemiech, GPLv3): D8PSK demodulation, preamble
 * synchronisation, LFSR descrambling, block deinterleaving, Reed-Solomon FEC
 * (RS(255,249), GF(256)), HDLC bit-destuffing and AVLC frame extraction.
 *
 * The chain operates directly on complex baseband I/Q. The expected input
 * sample rate is SPS * SYMBOL_RATE = 10 * 10500 = 105000 Hz; the host is
 * responsible for channelising/resampling the channel of interest to that
 * rate before handing samples to Demodulator::process().
 *
 * Each successfully de-framed AVLC octet string (FCS still attached) is handed
 * to a user callback. Higher-layer parsing (AVLC addresses, ACARS, ...) lives
 * in avlc.h / avlc.cpp.
 */
#pragma once
#include <cstdint>
#include <functional>
#include <vector>

namespace vdl2 {

// VDL2 physical-layer constants (ICAO Doc 9776).
constexpr int   SPS         = 10;       // samples per symbol
constexpr int   BPS         = 3;        // bits per D8PSK symbol
constexpr int   SYMBOL_RATE = 10500;    // symbols/second
constexpr int   SAMPLE_RATE = SPS * SYMBOL_RATE; // 105000 Hz expected input
constexpr int   PREAMBLE_SYMS = 16;
constexpr int   SYNC_BUFLEN = PREAMBLE_SYMS * SPS; // 160

constexpr int   RS_K        = 249;
constexpr int   RS_N        = 255;
constexpr int   TRLEN       = 17;       // transmission length field (bits)
constexpr int   HDRFECLEN   = 5;        // header FEC field (bits)
constexpr int   HEADER_LEN  = 3 + TRLEN + HDRFECLEN; // 25

// A de-framed AVLC octet string plus reception metadata.
struct RawFrame {
    std::vector<uint8_t> data;  // AVLC frame octets (FCS still attached)
    int   frame_index = 0;      // index within the burst
    float frame_pwr_dbfs = 0.f; // average symbol power, dBFS
    float nf_dbfs = 0.f;        // noise-floor estimate, dBFS
    int   fec_corrections = 0;  // number of RS symbols corrected
    int   header_syndrome = 0;  // header FEC syndrome (0 == clean header)
    float ppm_error = 0.f;      // estimated tx/rx clock error, ppm
};

using FrameCallback = std::function<void(const RawFrame&)>;

class Demodulator {
public:
    // centerFreqHz / channelFreqHz are informational only here (offset tuning
    // is handled by the host VFO); inputSampleRate must equal SAMPLE_RATE.
    explicit Demodulator(FrameCallback cb, uint32_t inputSampleRate = SAMPLE_RATE);
    ~Demodulator();

    // Feed a block of complex baseband samples (interleaved re/im as floats).
    void process(const float* iq, int numSamples);

    // Convenience overload for re/im split is not provided; use process().
    void reset();

    // Live status for the GUI.
    float noiseFloorDbfs() const { return 20.f * log10f_(mag_nf + 0.001f); }

private:
    // ---- DSP / framing state ----
    enum DemodState { DM_INIT, DM_SYNC };
    enum DecoderState { DEC_IDLE, DEC_HEADER, DEC_DATA };

    FrameCallback callback;
    uint32_t srate;

    // Chebyshev 2-pole input LPF (cutoff 8 kHz).
    float A[3] = {0,0,0};
    float B[3] = {0,0,0};
    float fre[3] = {0,0,0}, fim[3] = {0,0,0};
    float lpre[3] = {0,0,0}, lpim[3] = {0,0,0};
    void initLPF(float cutoffNorm, float ripplePct);

    // Sync / symbol clock.
    float syncbuf[SYNC_BUFLEN] = {0};
    int   syncbufidx = 0;
    float pherr[3] = {0,0,0};
    int   sclk = 0;
    DemodState demod_state = DM_INIT;
    float prev_phi = 0.f, dphi = 0.f, prev_dphi = 0.f, ppm_error = 0.f;
    float mag_lp = 0.f, mag_nf = 2.f;
    int   nfcnt = 0;
    float frame_pwr = 0.f;
    int   frame_pwr_cnt = 0;
    float lr_X[PREAMBLE_SYMS];
    float lr_denom = 0.f;

    // Bitstream (raw 1-bit-per-byte buffers).
    std::vector<uint8_t> bs;   // main symbol bitstream
    uint32_t bs_start = 0, bs_end = 0, descrambler_pos = 0;
    std::vector<uint8_t> fbs;  // de-framed (unstuffed) bitstream
    uint32_t fbs_start = 0, fbs_end = 0;

    DecoderState decoder_state = DEC_IDLE;
    uint32_t requested_bits = 0;
    uint16_t lfsr = 0;
    uint32_t datalen = 0, datalen_octets = 0;
    uint32_t num_blocks = 0, fec_octets = 0, last_block_len_octets = 0;
    int syndrome = 0;
    int num_fec_corrections = 0;

    class ReedSolomon* rs = nullptr;

    // ---- internal helpers ----
    void demodSample(float re, float im);
    int  gotSync();
    void decoderReset();
    void demodReset();
    void decodeBurst();

    // bitstream ops
    void bsReset();
    int  bsAppendMsbFirst(const uint8_t* bytes, uint32_t numbytes, uint32_t numbits);
    int  bsAppendLsbFirst(const uint8_t* bytes, uint32_t numbytes, uint32_t numbits);
    int  bsReadLsbFirst(uint8_t* out, uint32_t numbytes, uint32_t numbits);
    int  bsReadWordMsbFirst(uint32_t* ret, uint32_t numbits);
    void bsDescramble();
    int  copyNextFrame();   // HDLC unstuff into fbs; returns 1 more / 0 last / -1 err

    static float log10f_(float x);
};

uint32_t reverse_bits(uint32_t v, int numbits);

} // namespace vdl2
