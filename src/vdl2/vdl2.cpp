/*
 * vdl2.cpp - VDL Mode 2 demodulator/decoder implementation.
 *
 * Faithful C++ port of the dumpvdl2 physical/link decoding chain
 * (Copyright (c) Tomasz Lemiech, GPLv3).
 */
#include "vdl2.h"
#include "rs.h"

#include <cmath>
#include <cstring>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_4
#define M_PI_4 0.78539816339744830962
#endif

namespace vdl2 {

// ---------------------------------------------------------------------------
// Constants from dumpvdl2
// ---------------------------------------------------------------------------
static constexpr uint16_t LFSR_IV = 0x6959u;
static constexpr int   ARITY = 8;
static constexpr float MAG_LP = 0.9f;
static constexpr float NF_LP  = 0.85f;
static constexpr int   SYNC_SKIP = 3;
static constexpr float SYNC_THRESHOLD = 4.f;
static constexpr float PHERR_MAX = 1000.f;
static constexpr uint32_t MAX_FRAME_LENGTH = 0x3FFF;
static constexpr uint32_t MAX_FRAME_LENGTH_CORRECTED = 0x1FFF;
static constexpr float INP_LPF_CUTOFF = 8000.f;
static constexpr float INP_LPF_RIPPLE = 0.5f;
static constexpr uint32_t BSLEN = 32768u;

#define ONES(x) (~(~0u << (x)))

// Header (22,17) FEC parity check matrix and syndrome decode table.
static const uint32_t H[HDRFECLEN] = {
    0b0000000011111111111110000,
    0b0011111100001111111101000,
    0b1100011100110000111100100,
    0b1101101101010011001100010,
    0b0110100111100101010100001
};
static const uint32_t syndtable[1 << HDRFECLEN] = {
    0b0000000000000000000000000, 0b0000000000000000000000001,
    0b0000000000000000000000010, 0b0100000000000000000000100,
    0b0000000000000000000000100, 0b0100000000000000000000010,
    0b1000000000000000000000000, 0b0100000000000000000000000,
    0b0000000000000000000001000, 0b0010000000000000000000000,
    0b0001000000000000000000000, 0b0000100000000000000000000,
    0b0000010000000000000000000, 0b1000100000000000000000000,
    0b0000001000000000000000000, 0b0000000100000000000000000,
    0b0000000000000000000010000, 0b0000000010000000000000000,
    0b0100000000100000000000000, 0b0000000001000000000000000,
    0b0100000001000000000000000, 0b0000000000100000000000000,
    0b0000000000010000000000000, 0b1000000010000000000000000,
    0b0000000000001000000000000, 0b0000000000000100000000000,
    0b0000000000000010000000000, 0b0000000000000001000000000,
    0b0000000000000000100000000, 0b0000000000000000010000000,
    0b0000000000000000001000000, 0b0000000000000000000100000,
};

static int32_t parity(uint32_t v) {
    int32_t p = 0;
    while (v) { p = !p; v &= (v - 1); }
    return p;
}

static uint32_t decode_header(uint32_t* r) {
    uint32_t syndrome = 0u;
    for (int i = 0; i < HDRFECLEN; i++) {
        uint32_t row = *r & H[i];
        syndrome |= (parity(row)) << (HDRFECLEN - 1 - i);
    }
    *r ^= syndtable[syndrome];
    return syndrome;
}

static int get_fec_octetcount(uint32_t len) {
    if (len < 3)       return 0;
    else if (len < 31) return 2;
    else if (len < 68) return 4;
    else               return 6;
}

uint32_t reverse_bits(uint32_t v, int numbits) {
    uint32_t r = v;
    int s = (int)(sizeof(v) * 8) - 1;
    for (v >>= 1; v; v >>= 1) { r <<= 1; r |= v & 1; s--; }
    r <<= s;
    r >>= 32 - numbits;
    return r;
}

float Demodulator::log10f_(float x) { return std::log10(x); }

// ---------------------------------------------------------------------------
// Chebyshev 2-pole low-pass (port of dumpvdl2 chebyshev.c, 2 poles)
// ---------------------------------------------------------------------------
void Demodulator::initLPF(float cutoffNorm, float ripplePct) {
    const int npoles = 2;
    const int LP_BSIZE = npoles + 3;
    float Aa[8] = {0}, Bb[8] = {0};
    float TA[8] = {0}, TB[8] = {0};
    Aa[2] = 1.f; Bb[2] = 1.f;

    for (int p = 1; p <= npoles / 2; p++) {
        float rp, ip;
        float ang = (float)(M_PI / (2 * npoles) + (p - 1) * M_PI / npoles);
        ip = std::sin(ang);
        rp = -std::cos(ang);
        if (ripplePct != 0.f) {
            float es = std::sqrt(std::pow(100.f / (100.f - ripplePct), 2.f) - 1.f);
            float vx = (1.f / npoles) * std::log((1.f / es) + std::sqrt(1.f / (es * es) + 1.f));
            float kx = (1.f / npoles) * std::log((1.f / es) + std::sqrt(1.f / (es * es) - 1.f));
            kx = (std::exp(kx) + std::exp(-kx)) / 2.f;
            rp *= ((std::exp(vx) - std::exp(-vx)) / 2.f) / kx;
            ip *= ((std::exp(vx) + std::exp(-vx)) / 2.f) / kx;
        }
        float t = 2.f * std::tan(0.5f);
        float w = 2.f * (float)M_PI * cutoffNorm;
        float m = rp * rp + ip * ip;
        float d = 4.f - 4.f * rp * t + m * t * t;
        float x0 = t * t / d;
        float x1 = 2.f * x0;
        float x2 = x0;
        float y1 = (8.f - 2.f * m * t * t) / d;
        float y2 = (-4.f - 4.f * rp * t - m * t * t) / d;
        float k = std::sin(0.5f - w / 2.f) / std::sin(0.5f + w / 2.f);
        d = 1 + y1 * k - y2 * k * k;
        float AA0 = (x0 - x1 * k + x2 * k * k) / d;
        float AA1 = (-2.f * x0 * k + x1 + x1 * k * k - 2.f * x2 * k) / d;
        float AA2 = (x0 * k * k - x1 * k + x2) / d;
        float BB1 = (2.f * k + y1 + y1 * k * k - 2.f * y2 * k) / d;
        float BB2 = (-(k * k) - y1 * k + y2) / d;

        memcpy(TA, Aa, LP_BSIZE * sizeof(float));
        memcpy(TB, Bb, LP_BSIZE * sizeof(float));
        for (int i = 2; i < LP_BSIZE; i++) {
            Aa[i] = AA0 * TA[i] + AA1 * TA[i - 1] + AA2 * TA[i - 2];
            Bb[i] =        TB[i] - BB1 * TB[i - 1] - BB2 * TB[i - 2];
        }
    }
    Bb[2] = 0.f;
    for (int i = 0; i < LP_BSIZE - 2; i++) { Aa[i] = Aa[i + 2]; Bb[i] = -Bb[i + 2]; }
    float sa = 0.f, sb = 0.f;
    for (int i = 0; i < LP_BSIZE - 2; i++) { sa += Aa[i]; sb += Bb[i]; }
    float gain = sa / (1.f - sb);
    for (int i = 0; i < LP_BSIZE - 2; i++) Aa[i] /= gain;

    A[0] = Aa[0]; A[1] = Aa[1]; A[2] = Aa[2];
    B[0] = Bb[0]; B[1] = Bb[1]; B[2] = Bb[2];
}

static inline float cheb2(const float* A, const float* B, const float* in, const float* out) {
    float r = A[0] * in[0];
    r += A[1] * in[1] + A[2] * in[2];
    r += B[1] * out[1] + B[2] * out[2];
    return r;
}

// ---------------------------------------------------------------------------
// Construction / lifecycle
// ---------------------------------------------------------------------------
Demodulator::Demodulator(FrameCallback cb, uint32_t inputSampleRate)
    : callback(std::move(cb)), srate(inputSampleRate) {
    bs.assign(BSLEN, 0);
    fbs.assign(BSLEN, 0);
    rs = new ReedSolomon();

    // pre-compute linear-regression constants for sync
    float mean_X = 0.f;
    for (int i = 0; i < PREAMBLE_SYMS; i++) mean_X += i;
    mean_X /= PREAMBLE_SYMS;
    lr_denom = 0.f;
    for (int i = 0; i < PREAMBLE_SYMS; i++) {
        lr_X[i] = i - mean_X;
        lr_denom += (i - mean_X) * (i - mean_X);
    }

    initLPF(INP_LPF_CUTOFF / (float)srate, INP_LPF_RIPPLE);
    demodReset();
}

Demodulator::~Demodulator() { delete rs; }

void Demodulator::reset() { demodReset(); }

void Demodulator::bsReset() { bs_start = bs_end = descrambler_pos = 0; }

void Demodulator::decoderReset() {
    decoder_state = DEC_HEADER;
    requested_bits = HEADER_LEN;
    num_fec_corrections = 0;
    bsReset();
    fbs_start = fbs_end = 0;
}

void Demodulator::demodReset() {
    decoderReset();
    sclk = 0;
    demod_state = DM_INIT;
    pherr[1] = pherr[2] = PHERR_MAX;
    frame_pwr = 0.f;
    frame_pwr_cnt = 0;
}

// ---------------------------------------------------------------------------
// Bitstream operations (1 bit per byte)
// ---------------------------------------------------------------------------
int Demodulator::bsAppendMsbFirst(const uint8_t* bytes, uint32_t numbytes, uint32_t numbits) {
    if (bs_end + numbits * numbytes > bs.size()) return -1;
    for (uint32_t i = 0; i < numbytes; i++) {
        uint8_t t = bytes[i];
        for (int j = (int)numbits - 1; j >= 0; j--) bs[bs_end++] = (t >> j) & 0x01;
    }
    return 0;
}
int Demodulator::bsAppendLsbFirst(const uint8_t* bytes, uint32_t numbytes, uint32_t numbits) {
    if (bs_end + numbits * numbytes > bs.size()) return -1;
    for (uint32_t i = 0; i < numbytes; i++) {
        uint8_t t = bytes[i];
        for (uint32_t j = 0; j < numbits; j++) bs[bs_end++] = (t >> j) & 0x01;
    }
    return 0;
}
int Demodulator::bsReadLsbFirst(uint8_t* out, uint32_t numbytes, uint32_t numbits) {
    if (bs_start + numbits * numbytes > bs_end) return -1;
    for (uint32_t i = 0; i < numbytes; i++) {
        out[i] = 0;
        for (uint32_t j = 0; j < numbits; j++) out[i] |= (0x01 & bs[bs_start++]) << j;
    }
    return 0;
}
int Demodulator::bsReadWordMsbFirst(uint32_t* ret, uint32_t numbits) {
    if (bs_start + numbits > bs_end) return -1;
    *ret = 0;
    for (uint32_t i = 0; i < numbits; i++)
        *ret |= (0x01 & bs[bs_start++]) << (numbits - i - 1);
    return 0;
}
void Demodulator::bsDescramble() {
    if (descrambler_pos < bs_start) descrambler_pos = bs_start;
    for (uint32_t i = descrambler_pos; i < bs_end; i++) {
        // LFSR length 15, feedback polynomial x^15 + x + 1
        uint8_t bit = ((lfsr >> 0) ^ (lfsr >> 14)) & 1;
        lfsr = (lfsr >> 1) | (bit << 14);
        bs[i] ^= bit;
    }
    descrambler_pos = bs_end;
}

// HDLC bit-destuffing + flag splitting (src=bs, dst=fbs). 1=more, 0=last, -1=err.
int Demodulator::copyNextFrame() {
    int ones;
    uint32_t i, j;
restart:
    ones = 0;
    fbs_start = fbs_end = 0;
    for (i = bs_start, j = 0; i < bs_end; i++, bs_start++) {
        if (bs[i] == 0x0 && ones == 5) { ones = 0; continue; } // stuffed 0
        else if (bs[i] == 0x1) {
            ones++;
            if (ones > 6) return -1; // 7 ones: invalid
        }
        fbs[j] = bs[i];
        if (bs[i] == 0x0) {
            if (ones == 6) {              // 0x7e flag
                if (j == 7) { bs_start++; goto restart; } // leading flag
                else {
                    if (j < 7) return -1;
                    fbs_end = j - 7;      // drop trailing flag
                    bs_start++;
                    break;
                }
            }
            ones = 0;
        }
        j++; fbs_end++;
    }
    return (bs_start < bs_end ? 1 : 0);
}

// Deinterleave 'in' (len octets) column-major into rs_tab[rows][RS_N], filling
// 'fillwidth' columns starting at 'offset'. Flat-buffer port of dumpvdl2's VLA.
static int deinterleave(const uint8_t* in, uint32_t len, uint32_t rows,
                        uint8_t* rs_tab /* rows*RS_N */, uint32_t fillwidth, uint32_t offset) {
    const uint32_t cols = RS_N;
    if (rows == 0 || cols == 0 || fillwidth == 0) return -1;
    uint32_t last_row_len = len % fillwidth;
    if (last_row_len == 0) last_row_len = fillwidth;
    if (fillwidth + offset > cols) return -2;
    if (len > rows * fillwidth) return -3;
    if (rows > 1 && len - last_row_len < (rows - 1) * fillwidth) return -4;
    if (last_row_len == 0 && len / fillwidth < rows) return -5;
    uint32_t row = 0, col = offset;
    last_row_len += offset;
    for (uint32_t i = 0; i < len; i++) {
        if (row == rows - 1 && col >= last_row_len) {
            rs_tab[row * cols + col] = 0x00;
            row = 0; col++;
        }
        rs_tab[(row++) * cols + col] = in[i];
        if (row == rows) { row = 0; col++; }
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Burst decoder state machine (port of decode_vdl2_burst)
// ---------------------------------------------------------------------------
void Demodulator::decodeBurst() {
    switch (decoder_state) {
    case DEC_HEADER: {
        lfsr = LFSR_IV;
        bsDescramble();
        uint32_t header;
        if (bsReadWordMsbFirst(&header, HEADER_LEN) < 0) { decoder_state = DEC_IDLE; return; }
        header &= ONES(TRLEN + HDRFECLEN);
        syndrome = (int)decode_header(&header);
        if ((header & ONES(TRLEN + HDRFECLEN)) != header) { decoder_state = DEC_IDLE; return; }
        header >>= HDRFECLEN;
        datalen = reverse_bits(header & ONES(TRLEN), TRLEN);
        if ((syndrome != 0 && datalen > MAX_FRAME_LENGTH_CORRECTED) || datalen > MAX_FRAME_LENGTH) {
            decoder_state = DEC_IDLE; return;
        }
        datalen_octets = datalen / 8;
        if (datalen % 8 != 0) datalen_octets++;
        num_blocks = datalen_octets / RS_K;
        fec_octets = num_blocks * (RS_N - RS_K);
        last_block_len_octets = datalen_octets % RS_K;
        if (last_block_len_octets != 0) num_blocks++;
        fec_octets += get_fec_octetcount(last_block_len_octets);
        if (last_block_len_octets == 0) last_block_len_octets = 249;
        if (fec_octets == 0) { decoder_state = DEC_IDLE; return; }
        requested_bits = 8 * (datalen_octets + fec_octets);
        decoder_state = DEC_DATA;
        return;
    }
    case DEC_DATA: {
        bsDescramble();
        std::vector<uint8_t> data(datalen_octets ? datalen_octets : 1, 0);
        std::vector<uint8_t> fec(fec_octets ? fec_octets : 1, 0);
        if (bsReadLsbFirst(data.data(), datalen_octets, 8) < 0) { decoder_state = DEC_IDLE; return; }
        if (bsReadLsbFirst(fec.data(), fec_octets, 8) < 0) { decoder_state = DEC_IDLE; return; }

        std::vector<uint8_t> rs_tab((size_t)num_blocks * RS_N, 0);
        if (deinterleave(data.data(), datalen_octets, num_blocks, rs_tab.data(), RS_K, 0) < 0) {
            decoder_state = DEC_IDLE; return;
        }
        uint32_t fec_rows = num_blocks;
        if (get_fec_octetcount(last_block_len_octets) == 0) fec_rows--;
        if (deinterleave(fec.data(), fec_octets, fec_rows, rs_tab.data(), RS_N - RS_K, RS_K) < 0) {
            decoder_state = DEC_IDLE; return;
        }

        bsReset();
        for (uint32_t r = 0; r < num_blocks; r++) {
            int num_fec_octets = RS_N - RS_K;
            if (r == num_blocks - 1) num_fec_octets = get_fec_octetcount(last_block_len_octets);
            uint8_t* block = &rs_tab[(size_t)r * RS_N];
            int ret;
            if (num_fec_octets == 0) {
                ret = 0; // no FEC on tiny last block
            } else {
                int erasure_cnt = (RS_N - RS_K) - num_fec_octets;
                if (erasure_cnt > 0) {
                    int eras[RS_N - RS_K];
                    for (int i = 0; i < erasure_cnt; i++) eras[i] = RS_K + num_fec_octets + i;
                    ret = rs->decode(block, eras, erasure_cnt);
                } else {
                    ret = rs->decode(block, nullptr, 0);
                }
            }
            if (ret < 0) { decoder_state = DEC_IDLE; return; }
            if (ret > 0) num_fec_corrections += ret - ((RS_N - RS_K) - num_fec_octets);

            uint32_t take = (r != num_blocks - 1) ? (uint32_t)RS_K : last_block_len_octets;
            if (bsAppendLsbFirst(block, take, 8) < 0) { decoder_state = DEC_IDLE; return; }
        }
        // Truncate padding bits added by whole-byte append.
        if (datalen < bs_end - bs_start) bs_end = datalen;

        int ret, frame_cnt = 0;
        while ((ret = copyNextFrame()) >= 0) {
            if ((fbs_end - fbs_start) % 8 != 0) { decoder_state = DEC_IDLE; return; }
            uint32_t frame_len_octets = (fbs_end - fbs_start) / 8;
            RawFrame rf;
            rf.data.resize(frame_len_octets);
            // read fbs LSB-first into bytes
            uint32_t fp = fbs_start;
            for (uint32_t b = 0; b < frame_len_octets; b++) {
                uint8_t v = 0;
                for (uint32_t j = 0; j < 8; j++) v |= (0x01 & fbs[fp++]) << j;
                rf.data[b] = v;
            }
            rf.frame_index = frame_cnt;
            rf.frame_pwr_dbfs = 10.f * std::log10(frame_pwr + 1e-9f);
            rf.nf_dbfs = noiseFloorDbfs();
            rf.fec_corrections = num_fec_corrections;
            rf.header_syndrome = syndrome;
            rf.ppm_error = ppm_error;
            if (callback) callback(rf);
            frame_cnt++;
            if (ret == 0) break;
        }
        decoder_state = DEC_IDLE;
        return;
    }
    case DEC_IDLE:
    default:
        return;
    }
}

// ---------------------------------------------------------------------------
// Preamble synchronisation (port of got_sync)
// ---------------------------------------------------------------------------
int Demodulator::gotSync() {
    static const float pr_phase[PREAMBLE_SYMS] = {
        0.f * (float)M_PI / 4,  3.f * (float)M_PI / 4, -3.f * (float)M_PI / 4,  1.f * (float)M_PI / 4,
        1.f * (float)M_PI / 4,  2.f * (float)M_PI / 4,  0.f * (float)M_PI / 4,  4.f * (float)M_PI / 4,
       -3.f * (float)M_PI / 4,  4.f * (float)M_PI / 4, -2.f * (float)M_PI / 4,  3.f * (float)M_PI / 4,
        1.f * (float)M_PI / 4, -2.f * (float)M_PI / 4, -3.f * (float)M_PI / 4,  0.f * (float)M_PI / 4
    };
    float errvec[PREAMBLE_SYMS];
    float errvec_mean, unwrap = 0.f, prev_err;
    prev_err = errvec_mean = errvec[0] = syncbuf[(syncbufidx + SPS) % SYNC_BUFLEN] - pr_phase[0];
    for (int i = 1; i < PREAMBLE_SYMS; i++) {
        float cur_err = syncbuf[(syncbufidx + (i + 1) * SPS) % SYNC_BUFLEN] - pr_phase[i];
        float errdiff = cur_err - prev_err;
        prev_err = cur_err;
        if (errdiff > M_PI) unwrap -= 2.f * (float)M_PI;
        else if (errdiff < -M_PI) unwrap += 2.f * (float)M_PI;
        errvec[i] = cur_err + unwrap;
        errvec_mean += errvec[i];
    }
    errvec_mean /= PREAMBLE_SYMS;
    for (int i = 0; i < PREAMBLE_SYMS; i++) errvec[i] -= errvec_mean;

    float freq_err = 0.f;
    for (int i = 0; i < PREAMBLE_SYMS; i++) freq_err += lr_X[i] * errvec[i];
    freq_err /= lr_denom;

    float err = 0.f;
    pherr[0] = 0.f;
    for (int i = 0; i < PREAMBLE_SYMS; i++) {
        err = errvec[i] - freq_err * lr_X[i];
        pherr[0] += err * err;
    }

    if (pherr[1] < SYNC_THRESHOLD && pherr[0] > pherr[1]) {
        // parabola vertex over (sclk, pherr[2..0]) spaced by SYNC_SKIP
        float x = (float)sclk, d = (float)SYNC_SKIP;
        float y1 = pherr[2], y2 = pherr[1], y3 = pherr[0];
        float denom = (float)(d * 2 * d * (-d));
        float aA = (x * (y2 - y1) + (x - d) * (y1 - y3) + (x - 2 * d) * (y3 - y2)) / denom;
        float bB = (x * x * (y1 - y2) + (x - d) * (x - d) * (y3 - y1) + (x - 2 * d) * (x - 2 * d) * (y2 - y3)) / denom;
        float vertex_x = -bB / (2 * aA);
        sclk = -(int)std::lround(vertex_x);
        int sp = syncbufidx - sclk;
        if (sp < 0) sp += SYNC_BUFLEN;
        prev_phi = syncbuf[sp];
        dphi = prev_dphi;
        ppm_error = SYMBOL_RATE * dphi / (2.f * (float)M_PI * 1.0e8f) * 1e6f; // info only
        pherr[1] = pherr[2] = PHERR_MAX;
        return 1;
    }
    pherr[2] = pherr[1];
    pherr[1] = pherr[0];
    prev_dphi = freq_err;
    return 0;
}

// ---------------------------------------------------------------------------
// Per-sample D8PSK demod (port of demod())
// ---------------------------------------------------------------------------
void Demodulator::demodSample(float re, float im) {
    static const uint8_t graycode[ARITY] = { 0, 1, 3, 2, 6, 7, 5, 4 };

    if (decoder_state == DEC_IDLE) demodReset();

    switch (demod_state) {
    case DM_INIT:
        syncbufidx = (syncbufidx + 1) % SYNC_BUFLEN;
        syncbuf[syncbufidx] = std::atan2(im, re);
        if (++sclk < SYNC_SKIP) return;
        sclk = 0;
        {
            float mag = std::hypot(re, im);
            mag_lp = mag_lp * MAG_LP + mag * (1.f - MAG_LP);
            if (++nfcnt == 1000) {
                nfcnt = 0;
                mag_nf = NF_LP * mag_nf + (1.f - NF_LP) * std::min(mag_lp, mag_nf) + 0.0001f;
            }
        }
        if (gotSync()) demod_state = DM_SYNC;
        return;
    case DM_SYNC:
        if (++sclk < SPS) return;
        sclk = 0;
        {
            float phi = std::atan2(im, re);
            float d = phi - prev_phi - dphi;
            if (d < 0) d += 2.f * (float)M_PI;
            else if (d > 2.f * (float)M_PI) d -= 2.f * (float)M_PI;
            d /= (float)M_PI_4;
            int idx = ((int)std::lround(d)) % ARITY;
            if (idx < 0) idx += ARITY;
            float symbol_pwr = re * re + im * im;
            frame_pwr = (frame_pwr * frame_pwr_cnt + symbol_pwr) / (frame_pwr_cnt + 1);
            frame_pwr_cnt++;
            prev_phi = phi;
            uint8_t g = graycode[idx];
            if (bsAppendMsbFirst(&g, 1, BPS) < 0) { demodReset(); return; }
            if (bs_end - bs_start >= requested_bits) decodeBurst();
        }
        return;
    }
}

// ---------------------------------------------------------------------------
// Public process(): LPF + per-sample demod
// ---------------------------------------------------------------------------
void Demodulator::process(const float* iq, int numSamples) {
    for (int n = 0; n < numSamples; n++) {
        for (int k = 2; k > 0; k--) {
            fre[k] = fre[k - 1]; fim[k] = fim[k - 1];
            lpre[k] = lpre[k - 1]; lpim[k] = lpim[k - 1];
        }
        fre[0] = iq[2 * n];
        fim[0] = iq[2 * n + 1];
        lpre[0] = cheb2(A, B, fre, lpre);
        lpim[0] = cheb2(A, B, fim, lpim);
        demodSample(lpre[0], lpim[0]);
    }
}

} // namespace vdl2
