// test_acars.cpp - verify ACARS parity stripping, BCS validation and
// sanitization, including rejection of structurally-malformed sub-blocks.
//
//   g++ -std=c++17 -O2 -I src test/test_acars.cpp src/vdl2/*.cpp -o /tmp/test_acars
//   /tmp/test_acars        # expect "*** ALL ACARS TESTS OK ***"
#include "vdl2/avlc.h"
#include "vdl2/vdl2.h"
#include <cstdio>
#include <vector>
#include <string>

using namespace vdl2;

// Set odd parity in bit 7 of a 7-bit char (so the low 7 bits survive masking).
static uint8_t odd_par(uint8_t c) {
    c &= 0x7f;
    int ones = 0;
    for (int i = 0; i < 7; i++) ones += (c >> i) & 1;
    if ((ones & 1) == 0) c |= 0x80;
    return c;
}

enum Corruption { GOOD, NO_DEL, BAD_BCS };

// Build a complete AVLC frame carrying a downlink ACARS message.
static std::vector<uint8_t> build_frame(Corruption corr) {
    std::vector<uint8_t> fr;
    uint8_t dst[4] = {0x10,0x20,0x30,0x40};
    uint8_t src[4] = {0x10,0x05,0x07,0x09};   // aircraft
    for (int i=0;i<4;i++) fr.push_back(dst[i]);
    for (int i=0;i<4;i++) fr.push_back(src[i]);
    fr.push_back(0x00);                          // LCF: I-frame
    fr.push_back(0xff); fr.push_back(0xff); fr.push_back(0x01);
    size_t a = fr.size();
    auto pp = [&](const char* s){ for (const char* c=s;*c;c++) fr.push_back(odd_par((uint8_t)*c)); };

    fr.push_back(odd_par('2'));                  // mode
    pp(".F-GXYZ");                               // reg
    fr.push_back(odd_par(0x06));                 // ACK
    pp("H1");                                    // label
    fr.push_back(odd_par('5'));                  // block id (digit -> downlink)
    fr.push_back(0x02);                          // STX
    pp("M42B");                                  // seq (4)
    pp("AF1234");                                // flight (6)
    pp("POS N4321.0\nE00736.4");                 // text (with newline)
    fr.push_back(0x03);                          // ETX

    uint16_t bcs = crc16_ccitt(&fr[a], (uint32_t)(fr.size()-a), 0);
    if (corr == BAD_BCS) bcs ^= 0x0001;          // flip a bit
    fr.push_back(bcs & 0xff);
    fr.push_back((bcs >> 8) & 0xff);
    if (corr != NO_DEL) fr.push_back(0x7f);      // DEL (omitted in NO_DEL case)

    uint16_t fcs = ~crc16_ccitt(fr.data(), fr.size(), 0xFFFF);
    fr.push_back(fcs & 0xff);
    fr.push_back((fcs >> 8) & 0xff);
    return fr;
}

static Frame parse(Corruption c) {
    RawFrame rf; rf.data = build_frame(c);
    return parse_avlc(rf);
}

int main() {
    bool all = true;

    // 1) Good frame: fully decoded, BCS verified, fields clean.
    {
        Frame f = parse(GOOD);
        auto clean = [](const std::string& s){
            for (unsigned char c : s) if (c < 0x20 || c >= 0x7f) return false; return true; };
        bool ok = f.fcs_ok && f.acars_crc_ok && !f.acars_malformed
            && f.acars_block_id=='5' && f.acars_reg=="F-GXYZ" && f.acars_label=="H1"
            && f.acars_msg_seq=="M42B" && f.acars_flight=="AF1234"
            && f.acars_text=="POS N4321.0 E00736.4"   // newline -> space
            && clean(f.acars_reg) && clean(f.acars_flight)
            && clean(f.acars_msg_seq) && clean(f.acars_text) && clean(f.acars_label);
        printf("[good]    reg='%s' label='%s' seq='%s' flight='%s' text='%s' bcs_ok=%d malformed=%d -> %s\n",
               f.acars_reg.c_str(), f.acars_label.c_str(), f.acars_msg_seq.c_str(),
               f.acars_flight.c_str(), f.acars_text.c_str(), f.acars_crc_ok, f.acars_malformed,
               ok ? "OK" : "FAIL");
        all = all && ok;
    }

    // 2) No DEL terminator: structurally malformed -> flagged (module drops it).
    {
        Frame f = parse(NO_DEL);
        bool ok = f.fcs_ok && f.acars_malformed;
        printf("[no DEL]  fcs_ok=%d malformed=%d -> %s\n", f.fcs_ok, f.acars_malformed, ok?"OK":"FAIL");
        all = all && ok;
    }

    // 3) Corrupted BCS: structurally valid but CRC fails (kept, flagged).
    {
        Frame f = parse(BAD_BCS);
        bool ok = f.fcs_ok && !f.acars_malformed && !f.acars_crc_ok;
        printf("[bad BCS] fcs_ok=%d malformed=%d bcs_ok=%d -> %s\n",
               f.fcs_ok, f.acars_malformed, f.acars_crc_ok, ok?"OK":"FAIL");
        all = all && ok;
    }

    printf("\n%s\n", all ? "*** ALL ACARS TESTS OK ***" : "*** FAIL ***");
    return all ? 0 : 1;
}
