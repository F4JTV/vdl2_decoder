/*
 * rs.h - Self-contained Reed-Solomon codec over GF(2^8) for VDL Mode 2.
 *
 * This is a clean-room style port of the generic char Reed-Solomon codec
 * (the classic "RS char" algorithm by Phil Karn, KA9Q, libfec, LGPL),
 * specialised for the VDL2 parameters used by dumpvdl2:
 *
 *     init_rs_char(symsize=8, gfpoly=0x187, fcr=120, prim=1, nroots=6, pad=0)
 *
 * i.e. RS(255, 249) over GF(256) with field generator polynomial 0x187,
 * first consecutive root = 120, primitive element = 1, 6 parity symbols.
 *
 * Only the pieces needed by the decoder are kept. An encoder is also provided
 * so the codec can be self-tested without a live signal.
 */
#pragma once
#include <cstdint>

namespace vdl2 {

// Reed-Solomon codec state. Allocated once and reused for every block.
class ReedSolomon {
public:
    ReedSolomon();

    // Decode one RS codeword in place.
    // data:      RS_N (=255) symbols (data + parity).
    // eras_pos:  optional list of erasure positions (may be nullptr).
    // no_eras:   number of erasures.
    // Returns the number of corrected symbols, or -1 if decoding failed.
    int decode(uint8_t* data, int* eras_pos, int no_eras) const;

    // Encode: compute RS_N-RS_K parity symbols for the first RS_K data symbols.
    // data[0..RS_K-1] are read, parity[0..NROOTS-1] are written. (test helper)
    void encode(const uint8_t* data, uint8_t* parity) const;

    static const int SYMSIZE = 8;
    static const int GFPOLY  = 0x187;
    static const int FCR     = 120;
    static const int PRIM    = 1;
    static const int NN      = 255;          // 2^SYMSIZE - 1
    static const int NROOTS  = 6;            // RS_N - RS_K
    static const int PAD     = 0;
    static const int A0      = 255;          // index of the zero element

private:
    uint8_t alpha_to[NN + 1];   // log -> poly
    uint8_t index_of[NN + 1];   // poly -> log
    uint8_t genpoly[NROOTS + 1];
    int iprim;                  // prim-th root of 1, index form

    int mod(int x) const;       // x mod NN
};

} // namespace vdl2
