/*
 * rs.cpp - Reed-Solomon GF(2^8) codec implementation for VDL Mode 2.
 *
 * Clean-room style port of the generic char Reed-Solomon decoder
 * (algorithm by Phil Karn, KA9Q; libfec, LGPL), specialised to the VDL2
 * field parameters. See rs.h for the parameter set.
 */
#include "rs.h"
#include <cstring>

namespace vdl2 {

static inline int min_int(int a, int b) { return a < b ? a : b; }

int ReedSolomon::mod(int x) const {
    while (x >= NN) {
        x -= NN;
        x = (x >> SYMSIZE) + (x & NN);
    }
    return x;
}

ReedSolomon::ReedSolomon() {
    // Generate Galois field log/antilog tables for GF(2^SYMSIZE)
    // with field generator polynomial GFPOLY.
    int sr = 1;
    index_of[0] = A0; // log(0) = -inf
    alpha_to[A0] = 0; // alpha^(-inf) = 0
    for (int i = 0; i < NN; i++) {
        index_of[sr] = (uint8_t)i;
        alpha_to[i] = (uint8_t)sr;
        sr <<= 1;
        if (sr & (1 << SYMSIZE)) {
            sr ^= GFPOLY;
        }
        sr &= NN;
    }

    // Find prim-th root of 1, used in decoding.
    int ip;
    for (iprim = 1; (iprim % PRIM) != 0; iprim += NN) {}
    iprim = iprim / PRIM;
    (void)ip;

    // Form the RS code generator polynomial:
    //   g(x) = product (x - alpha^(fcr + i*prim)), i = 0 .. NROOTS-1
    genpoly[0] = 1;
    int root = FCR * PRIM;
    for (int i = 0; i < NROOTS; i++, root += PRIM) {
        genpoly[i + 1] = 1;
        // multiply genpoly[0..i] by (x - alpha^root)
        for (int j = i; j > 0; j--) {
            if (genpoly[j] != 0) {
                genpoly[j] = genpoly[j - 1] ^
                    alpha_to[mod((int)index_of[genpoly[j]] + root)];
            } else {
                genpoly[j] = genpoly[j - 1];
            }
        }
        // genpoly[0] can never be zero
        genpoly[0] = alpha_to[mod((int)index_of[genpoly[0]] + root)];
    }
    // convert genpoly[] to index form for quicker encoding
    for (int i = 0; i <= NROOTS; i++) {
        genpoly[i] = index_of[genpoly[i]];
    }
}

void ReedSolomon::encode(const uint8_t* data, uint8_t* parity) const {
    memset(parity, 0, NROOTS);
    for (int i = 0; i < NN - NROOTS - PAD; i++) {
        uint8_t feedback = index_of[data[i] ^ parity[0]];
        if (feedback != A0) {
            for (int j = 1; j < NROOTS; j++) {
                parity[j] ^= alpha_to[mod((int)feedback + genpoly[NROOTS - j])];
            }
        }
        memmove(&parity[0], &parity[1], NROOTS - 1);
        if (feedback != A0) {
            parity[NROOTS - 1] = alpha_to[mod((int)feedback + genpoly[0])];
        } else {
            parity[NROOTS - 1] = 0;
        }
    }
}

int ReedSolomon::decode(uint8_t* data, int* eras_pos, int no_eras) const {
    const int NROOTS_ = NROOTS;
    uint8_t lambda[NROOTS + 1];
    uint8_t s[NROOTS];
    uint8_t b[NROOTS + 1];
    uint8_t t[NROOTS + 1];
    uint8_t omega[NROOTS + 1];
    uint8_t root[NROOTS];
    uint8_t reg[NROOTS + 1];
    uint8_t loc[NROOTS];
    int count = 0;
    int deg_lambda, el, deg_omega;
    int i, j, r, k;
    uint8_t q, tmp, num1, num2, den, discr_r;

    // form the syndromes; evaluate data(x) at roots of g(x)
    for (i = 0; i < NROOTS_; i++) {
        s[i] = data[0];
    }
    for (j = 1; j < NN - PAD; j++) {
        for (i = 0; i < NROOTS_; i++) {
            if (s[i] == 0) {
                s[i] = data[j];
            } else {
                s[i] = data[j] ^ alpha_to[mod((int)index_of[s[i]] + (FCR + i) * PRIM)];
            }
        }
    }

    // convert syndromes to index form, check for nonzero
    int syn_error = 0;
    for (i = 0; i < NROOTS_; i++) {
        syn_error |= s[i];
        s[i] = index_of[s[i]];
    }

    if (!syn_error) {
        // no errors: syndromes are zero
        return 0;
    }

    memset(&lambda[1], 0, NROOTS_ * sizeof(lambda[0]));
    lambda[0] = 1;

    if (no_eras > 0) {
        // init lambda to be the erasure locator polynomial
        lambda[1] = alpha_to[mod(PRIM * (NN - 1 - eras_pos[0]))];
        for (i = 1; i < no_eras; i++) {
            uint8_t u = mod(PRIM * (NN - 1 - eras_pos[i]));
            for (j = i + 1; j > 0; j--) {
                tmp = index_of[lambda[j - 1]];
                if (tmp != A0) {
                    lambda[j] ^= alpha_to[mod((int)u + tmp)];
                }
            }
        }
    }
    for (i = 0; i < NROOTS_ + 1; i++) {
        b[i] = index_of[lambda[i]];
    }

    // Berlekamp-Massey to determine error+erasure locator polynomial
    r = no_eras;
    el = no_eras;
    while (++r <= NROOTS_) {
        // compute discrepancy at the r-th step in poly form
        discr_r = 0;
        for (i = 0; i < r; i++) {
            if ((lambda[i] != 0) && (s[r - i - 1] != A0)) {
                discr_r ^= alpha_to[mod((int)index_of[lambda[i]] + s[r - i - 1])];
            }
        }
        discr_r = index_of[discr_r];
        if (discr_r == A0) {
            // shift b
            memmove(&b[1], b, NROOTS_ * sizeof(b[0]));
            b[0] = A0;
        } else {
            // T(x) = lambda(x) - discr_r * x * b(x)
            t[0] = lambda[0];
            for (i = 0; i < NROOTS_; i++) {
                if (b[i] != A0) {
                    t[i + 1] = lambda[i + 1] ^ alpha_to[mod((int)discr_r + b[i])];
                } else {
                    t[i + 1] = lambda[i + 1];
                }
            }
            if (2 * el <= r + no_eras - 1) {
                el = r + no_eras - el;
                // b(x) = lambda(x) / discr_r
                for (i = 0; i <= NROOTS_; i++) {
                    b[i] = (lambda[i] == 0) ? A0
                        : mod((int)index_of[lambda[i]] - discr_r + NN);
                }
            } else {
                // shift b
                memmove(&b[1], b, NROOTS_ * sizeof(b[0]));
                b[0] = A0;
            }
            memcpy(lambda, t, (NROOTS_ + 1) * sizeof(t[0]));
        }
    }

    // convert lambda to index form and compute deg(lambda)
    deg_lambda = 0;
    for (i = 0; i < NROOTS_ + 1; i++) {
        lambda[i] = index_of[lambda[i]];
        if (lambda[i] != A0) {
            deg_lambda = i;
        }
    }

    // find roots of error+erasure locator poly by Chien search
    memcpy(&reg[1], &lambda[1], NROOTS_ * sizeof(reg[0]));
    count = 0;
    for (i = 1, k = iprim - 1; i <= NN; i++, k = mod(k + iprim)) {
        q = 1; // lambda[0] is always 0
        for (j = deg_lambda; j > 0; j--) {
            if (reg[j] != A0) {
                reg[j] = mod((int)reg[j] + j);
                q ^= alpha_to[reg[j]];
            }
        }
        if (q != 0) {
            continue; // not a root
        }
        // store root (index-form) and error location number
        root[count] = (uint8_t)i;
        loc[count] = (uint8_t)k;
        if (++count == deg_lambda) {
            break;
        }
    }
    if (deg_lambda != count) {
        // deg(lambda) != number of roots => uncorrectable
        return -1;
    }

    // compute err+eras evaluator poly omega(x) = s(x)*lambda(x) mod x^NROOTS
    deg_omega = deg_lambda - 1;
    for (i = 0; i <= deg_omega; i++) {
        tmp = 0;
        for (j = i; j >= 0; j--) {
            if ((s[i - j] != A0) && (lambda[j] != A0)) {
                tmp ^= alpha_to[mod((int)s[i - j] + lambda[j])];
            }
        }
        omega[i] = index_of[tmp];
    }

    // compute error values via Forney
    for (j = count - 1; j >= 0; j--) {
        num1 = 0;
        for (i = deg_omega; i >= 0; i--) {
            if (omega[i] != A0) {
                num1 ^= alpha_to[mod((int)omega[i] + i * root[j])];
            }
        }
        num2 = alpha_to[mod((int)root[j] * (FCR - 1) + NN)];
        den = 0;
        // lambda'(z), even terms (formal derivative)
        for (i = min_int(deg_lambda, NROOTS_ - 1) & ~1; i >= 0; i -= 2) {
            if (lambda[i + 1] != A0) {
                den ^= alpha_to[mod((int)lambda[i + 1] + i * root[j])];
            }
        }
        // apply error to data
        if (num1 != 0 && loc[j] >= (uint8_t)PAD) {
            data[loc[j] - PAD] ^= alpha_to[mod((int)index_of[num1] +
                index_of[num2] + NN - index_of[den])];
        }
    }

    // write the corrected erasure/error positions to eras_pos
    if (eras_pos != nullptr) {
        for (i = 0; i < count; i++) {
            eras_pos[i] = loc[i];
        }
    }
    (void)NROOTS_;
    return count;
}

} // namespace vdl2
