// test_chain.cpp - build a VDL2 burst from a known ACARS frame, modulate to
// D8PSK I/Q, run it through the demodulator, and verify the decoded frame.
//
// Standalone self-test (no SDR++ required). From this module's root:
//   g++ -std=c++17 -O2 -I src test/test_chain.cpp src/vdl2/*.cpp -o /tmp/test_chain
//   /tmp/test_chain        # expect "*** ROUND-TRIP OK ***"
#include "vdl2/vdl2.h"
#include "vdl2/rs.h"
#include "vdl2/avlc.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include <cmath>
using namespace vdl2;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ONES(x) (~(~0u << (x)))

// --- header (22,17) code: same H as decoder ---
static const uint32_t H[5] = {
    0b0000000011111111111110000,
    0b0011111100001111111101000,
    0b1100011100110000111100100,
    0b1101101101010011001100010,
    0b0110100111100101010100001
};
static int parity(uint32_t v){int p=0;while(v){p=!p;v&=v-1;}return p;}
static uint32_t hdr_syndrome(uint32_t cw){
    uint32_t s=0; for(int i=0;i<5;i++) s |= (parity(cw&H[i]))<<(5-1-i); return s;
}

static int get_fec_octetcount(uint32_t len){
    if(len<3) return 0; else if(len<31) return 2; else if(len<68) return 4; else return 6;
}

// forward interleave: inverse index mapping of decoder's deinterleave().
// writes rs_tab[rows][255] columns [offset, offset+fillwidth) into out[len].
static void interleave(uint8_t* out, uint32_t len, uint32_t rows,
                       const uint8_t* rs_tab, uint32_t fillwidth, uint32_t offset){
    const uint32_t cols=255;
    uint32_t last_row_len = len % fillwidth; if(last_row_len==0) last_row_len=fillwidth;
    uint32_t row=0, col=offset; last_row_len += offset;
    for(uint32_t i=0;i<len;i++){
        if(row==rows-1 && col>=last_row_len){ row=0; col++; }
        out[i] = rs_tab[(row++)*cols + col];
        if(row==rows){ row=0; col++; }
    }
}

static uint8_t bits_buf[300000];  // 1 bit per byte (post-scramble symbol bits)

int main(){
    ReedSolomon rs;

    // ---- 1. Build a known AVLC frame with an ACARS I-frame payload ----
    // dst(4) + src(4) + lcf(1) + FFFF01 + ACARS body, then FCS(2).
    std::vector<uint8_t> fr;
    uint8_t dst[4] = {0x10,0x20,0x30,0x40};
    uint8_t src[4] = {0x10,0x05,0x07,0x09}; // aircraft type (top addr bits -> 1)
    for(int i=0;i<4;i++) fr.push_back(dst[i]);
    for(int i=0;i<4;i++) fr.push_back(src[i]);
    fr.push_back(0x00);                  // LCF: I-frame (bit0=0)
    fr.push_back(0xff); fr.push_back(0xff); fr.push_back(0x01); // ACARS prefix
    size_t acars_start = fr.size();
    // ACARS body: mode '2', reg ".F-ABCD", ack ACK(0x06), label "H1", blkid '1'
    // (block id is a digit -> downlink, so seq+flight are extracted)
    const char* body = "2.F-ABCD\x06H11";
    for(const char* c=body;*c;c++) fr.push_back((uint8_t)*c);
    fr.push_back(0x02);                  // STX
    // downlink seq + flight
    const char* seqflt = "M01AAF1234";
    for(const char* c=seqflt;*c;c++) fr.push_back((uint8_t)*c);
    const char* text = "HELLO VDL2 WORLD";
    for(const char* c=text;*c;c++) fr.push_back((uint8_t)*c);
    fr.push_back(0x03);                  // ETX

    // ACARS Block Check Sequence (CRC-16/CCITT, init 0) over mode..ETX,
    // appended low byte first (receiver residue 0), then the DEL terminator.
    uint16_t bcs = crc16_ccitt(&fr[acars_start], (uint32_t)(fr.size() - acars_start), 0);
    fr.push_back(bcs & 0xff);
    fr.push_back((bcs >> 8) & 0xff);
    fr.push_back(0x7f);                  // DEL

    // FCS: append ~crc, low byte first, so receiver residue == 0xF0B8
    uint16_t c = crc16_ccitt(fr.data(), fr.size(), 0xFFFF);
    uint16_t fcs = ~c;
    fr.push_back(fcs & 0xff);
    fr.push_back((fcs >> 8) & 0xff);
    if(crc16_ccitt(fr.data(), fr.size(), 0xFFFF) != 0xF0B8){
        printf("FCS construction FAILED (residue=%04x)\n", crc16_ccitt(fr.data(), fr.size(), 0xFFFF));
        return 1;
    }
    printf("AVLC frame built: %zu octets (incl FCS)\n", fr.size());

    // ---- 2. HDLC: bit-stuff + flags into a raw bitstream (LSB-first octets) ----
    std::vector<uint8_t> dbits; // 1 bit per byte
    auto push_flag=[&](){ uint8_t fl=0x7e; for(int j=0;j<8;j++) dbits.push_back((fl>>j)&1); };
    push_flag();
    int ones=0;
    for(uint8_t by : fr){
        for(int j=0;j<8;j++){
            uint8_t bit=(by>>j)&1;
            dbits.push_back(bit);
            if(bit==1){ ones++; if(ones==5){ dbits.push_back(0); ones=0; } }
            else ones=0;
        }
    }
    push_flag();

    // pad to whole octets
    while(dbits.size()%8) dbits.push_back(0);
    uint32_t datalen = (uint32_t)dbits.size();      // length in bits (the TRLEN value)
    uint32_t datalen_octets = datalen/8;

    // pack dbits into data octets LSB-first
    std::vector<uint8_t> data(datalen_octets,0);
    for(uint32_t i=0;i<datalen;i++) data[i/8] |= (dbits[i]&1) << (i%8);

    // ---- 3. RS-encode blocks ----
    uint32_t num_blocks = datalen_octets/RS_K;
    uint32_t fec_octets = num_blocks*(RS_N-RS_K);
    uint32_t last_block_len = datalen_octets % RS_K;
    if(last_block_len!=0) num_blocks++;
    fec_octets += get_fec_octetcount(last_block_len);
    if(last_block_len==0) last_block_len=249;

    std::vector<uint8_t> rs_tab((size_t)num_blocks*RS_N,0);
    // fill data into rows
    {
        uint32_t off=0;
        for(uint32_t r=0;r<num_blocks;r++){
            uint32_t take=(r!=num_blocks-1)?RS_K:last_block_len;
            memcpy(&rs_tab[(size_t)r*RS_N], &data[off], take);
            off+=take;
        }
    }
    // compute parity for each block
    for(uint32_t r=0;r<num_blocks;r++){
        int nf=(r==num_blocks-1)?get_fec_octetcount(last_block_len):(RS_N-RS_K);
        if(nf==0) continue;
        uint8_t parity[6];
        rs.encode(&rs_tab[(size_t)r*RS_N], parity);   // full 6 parity over 249 data
        // place 'nf' parity octets right after RS_K (rest are erasures => 0)
        for(int i=0;i<nf;i++) rs_tab[(size_t)r*RS_N + RS_K + i] = parity[i];
    }

    // ---- 4. interleave back into data[] and fec[] streams ----
    std::vector<uint8_t> idata(datalen_octets,0), ifec(fec_octets,0);
    interleave(idata.data(), datalen_octets, num_blocks, rs_tab.data(), RS_K, 0);
    uint32_t fec_rows=num_blocks; if(get_fec_octetcount(last_block_len)==0) fec_rows--;
    interleave(ifec.data(), fec_octets, fec_rows, rs_tab.data(), RS_N-RS_K, RS_K);

    // ---- 5. assemble symbol bitstream: header(25 MSB-first) + data + fec ----
    // header codeword: (reverse(datalen,17) << 5) | parity5, top 3 reserved=0
    uint32_t data17 = reverse_bits(datalen & ONES(17), 17);
    uint32_t cw = (data17 << 5);
    uint32_t syn = hdr_syndrome(cw);
    cw |= syn;                                   // identity parity columns
    if(hdr_syndrome(cw)!=0){ printf("header parity build FAILED\n"); return 1; }
    uint32_t header25 = cw & ONES(22);           // top 3 reserved bits = 0

    std::vector<uint8_t> symbits; // pre-scramble, 1 bit/byte
    // header: 25 bits MSB-first
    for(int i=0;i<HEADER_LEN;i++) symbits.push_back((header25>>(HEADER_LEN-1-i))&1);
    // data octets LSB-first
    for(uint8_t b: idata) for(int j=0;j<8;j++) symbits.push_back((b>>j)&1);
    // fec octets LSB-first
    for(uint8_t b: ifec) for(int j=0;j<8;j++) symbits.push_back((b>>j)&1);

    // ---- 6. scramble (same LFSR as decoder) ----
    uint16_t lfsr=0x6959;
    for(size_t i=0;i<symbits.size();i++){
        uint8_t bit=((lfsr>>0)^(lfsr>>14))&1;
        lfsr=(lfsr>>1)|(bit<<14);
        symbits[i]^=bit;
    }
    // pad symbits to multiple of BPS=3
    while(symbits.size()%3) symbits.push_back(0);

    // ---- 7. map 3-bit groups (MSB-first) -> symbol idx via inverse Gray ----
    static const uint8_t graycode[8]={0,1,3,2,6,7,5,4};
    uint8_t degray[8]; for(int i=0;i<8;i++) degray[graycode[i]]=i;
    std::vector<int> dsym;
    for(size_t i=0;i+2<symbits.size()+1 && i+3<=symbits.size(); i+=3){
        uint8_t v=(symbits[i]<<2)|(symbits[i+1]<<1)|symbits[i+2];
        dsym.push_back(degray[v]);
    }

    // ---- 8. build absolute symbol phases: preamble then differential data ----
    const double pr[16]={0,3,-3,1,1,2,0,4,-3,4,-2,3,1,-2,-3,0};
    std::vector<double> phase;
    for(int i=0;i<16;i++) phase.push_back(pr[i]*M_PI/4.0);
    double cur = pr[15]*M_PI/4.0;                // prev_phi after preamble (=0)
    for(int idx : dsym){ cur += idx*M_PI/4.0; phase.push_back(cur); }

    // ---- 9. render to I/Q at SPS=10, with lead-in/lead-out silence-ish carrier ----
    std::vector<float> iq;
    auto emit=[&](double ph,int n,float amp){ for(int k=0;k<n;k++){ iq.push_back(amp*cosf(ph)); iq.push_back(amp*sinf(ph)); } };
    // lead-in: 300 samples at phase 0 to settle LPF + noise floor
    emit(0.0, 60, 0.0f);                         // a little dead air
    for(double ph : phase) emit(ph, SPS, 1.0f);  // 10 samples/symbol, full amplitude
    emit(phase.back(), 40, 1.0f);                // tail

    printf("Generated %zu symbols (16 preamble + %zu data), %zu IQ samples\n",
           phase.size(), dsym.size(), iq.size()/2);

    // ---- 10. run through the demodulator ----
    int got=0;
    Frame decoded;
    Demodulator demod([&](const RawFrame& rf){
        Frame f = parse_avlc(rf);
        if(f.valid){ got++; decoded=f; }
    });
    // feed a bunch of dead air first so noise-floor logic initialises
    std::vector<float> warm(2000,0.0f);
    demod.process(warm.data(), warm.size()/2);
    demod.process(iq.data(), iq.size()/2);

    printf("\nFrames decoded with valid FCS: %d\n", got);
    if(got>0){
        printf("  proto      : %s\n", decoded.proto.c_str());
        printf("  src/dst    : %06X (%s) -> %06X (%s)\n",
               decoded.src_addr, decoded.srcTypeStr().c_str(),
               decoded.dst_addr, decoded.dstTypeStr().c_str());
        printf("  lcf        : %c %s\n", decoded.lcf_class, decoded.lcf_desc.c_str());
        printf("  fec corr   : %d   hdr synd: %d\n", decoded.fec_corrections, decoded.header_syndrome);
        if(decoded.has_acars){
            printf("  ACARS mode : %c\n", decoded.acars_mode);
            printf("  ACARS reg  : %s\n", decoded.acars_reg.c_str());
            printf("  ACARS label: %s  blk: %c  nak: %d\n",
                   decoded.acars_label.c_str(), decoded.acars_block_id, decoded.acars_nak);
            printf("  ACARS seq  : %s  flight: %s\n",
                   decoded.acars_msg_seq.c_str(), decoded.acars_flight.c_str());
            printf("  ACARS text : '%s'\n", decoded.acars_text.c_str());
        }
        bool textok = (decoded.acars_text == "HELLO VDL2 WORLD");
        printf("\n%s\n", textok ? "*** ROUND-TRIP OK ***" : "*** TEXT MISMATCH ***");
        return textok ? 0 : 2;
    }
    return 3;
}
