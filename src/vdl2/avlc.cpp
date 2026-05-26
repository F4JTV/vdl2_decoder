/*
 * avlc.cpp - AVLC frame + ACARS message parser for VDL Mode 2.
 *
 * Frame structure / address decoding ported from dumpvdl2 (avlc.c, GPLv3).
 * ACARS message layout follows ARINC 618.
 */
#include "avlc.h"
#include <cstring>
#include <cctype>
#include <vector>

namespace vdl2 {

#define ONES(x) (~(~0u << (x)))

static constexpr uint32_t MIN_AVLC_LEN = 11;
static constexpr uint16_t GOOD_FCS = 0xF0B8u;

// AVLC U-frame modifier function codes
static constexpr uint8_t UF_UI   = 0x00;
static constexpr uint8_t UF_DM   = 0x03;
static constexpr uint8_t UF_DISC = 0x10;
static constexpr uint8_t UF_UA   = 0x18;
static constexpr uint8_t UF_FRMR = 0x21;
static constexpr uint8_t UF_XID  = 0x2b;
static constexpr uint8_t UF_TEST = 0x38;

uint16_t crc16_ccitt(const uint8_t* data, uint32_t len, uint16_t crc_init) {
    static const uint16_t t[256] = {
        0x0000,0x1189,0x2312,0x329B,0x4624,0x57AD,0x6536,0x74BF,0x8C48,0x9DC1,0xAF5A,0xBED3,0xCA6C,0xDBE5,0xE97E,0xF8F7,
        0x1081,0x0108,0x3393,0x221A,0x56A5,0x472C,0x75B7,0x643E,0x9CC9,0x8D40,0xBFDB,0xAE52,0xDAED,0xCB64,0xF9FF,0xE876,
        0x2102,0x308B,0x0210,0x1399,0x6726,0x76AF,0x4434,0x55BD,0xAD4A,0xBCC3,0x8E58,0x9FD1,0xEB6E,0xFAE7,0xC87C,0xD9F5,
        0x3183,0x200A,0x1291,0x0318,0x77A7,0x662E,0x54B5,0x453C,0xBDCB,0xAC42,0x9ED9,0x8F50,0xFBEF,0xEA66,0xD8FD,0xC974,
        0x4204,0x538D,0x6116,0x709F,0x0420,0x15A9,0x2732,0x36BB,0xCE4C,0xDFC5,0xED5E,0xFCD7,0x8868,0x99E1,0xAB7A,0xBAF3,
        0x5285,0x430C,0x7197,0x601E,0x14A1,0x0528,0x37B3,0x263A,0xDECD,0xCF44,0xFDDF,0xEC56,0x98E9,0x8960,0xBBFB,0xAA72,
        0x6306,0x728F,0x4014,0x519D,0x2522,0x34AB,0x0630,0x17B9,0xEF4E,0xFEC7,0xCC5C,0xDDD5,0xA96A,0xB8E3,0x8A78,0x9BF1,
        0x7387,0x620E,0x5095,0x411C,0x35A3,0x242A,0x16B1,0x0738,0xFFCF,0xEE46,0xDCDD,0xCD54,0xB9EB,0xA862,0x9AF9,0x8B70,
        0x8408,0x9581,0xA71A,0xB693,0xC22C,0xD3A5,0xE13E,0xF0B7,0x0840,0x19C9,0x2B52,0x3ADB,0x4E64,0x5FED,0x6D76,0x7CFF,
        0x9489,0x8500,0xB79B,0xA612,0xD2AD,0xC324,0xF1BF,0xE036,0x18C1,0x0948,0x3BD3,0x2A5A,0x5EE5,0x4F6C,0x7DF7,0x6C7E,
        0xA50A,0xB483,0x8618,0x9791,0xE32E,0xF2A7,0xC03C,0xD1B5,0x2942,0x38CB,0x0A50,0x1BD9,0x6F66,0x7EEF,0x4C74,0x5DFD,
        0xB58B,0xA402,0x9699,0x8710,0xF3AF,0xE226,0xD0BD,0xC134,0x39C3,0x284A,0x1AD1,0x0B58,0x7FE7,0x6E6E,0x5CF5,0x4D7C,
        0xC60C,0xD785,0xE51E,0xF497,0x8028,0x91A1,0xA33A,0xB2B3,0x4A44,0x5BCD,0x6956,0x78DF,0x0C60,0x1DE9,0x2F72,0x3EFB,
        0xD68D,0xC704,0xF59F,0xE416,0x90A9,0x8120,0xB3BB,0xA232,0x5AC5,0x4B4C,0x79D7,0x685E,0x1CE1,0x0D68,0x3FF3,0x2E7A,
        0xE70E,0xF687,0xC41C,0xD595,0xA12A,0xB0A3,0x8238,0x93B1,0x6B46,0x7ACF,0x4854,0x59DD,0x2D62,0x3CEB,0x0E70,0x1FF9,
        0xF78F,0xE606,0xD49D,0xC514,0xB1AB,0xA022,0x92B9,0x8330,0x7BC7,0x6A4E,0x58D5,0x495C,0x3DE3,0x2C6A,0x1EF1,0x0F78
    };
    uint16_t crc = crc_init;
    while (len-- > 0) crc = (crc >> 8) ^ t[(crc ^ *data++) & 0xff];
    return crc;
}

// 28-bit DLC address: 24-bit address, 3-bit type, 1-bit status.
static uint32_t parse_dlc_addr(const uint8_t* buf) {
    return reverse_bits((buf[0] >> 1) | (buf[1] << 6) | (buf[2] << 13) |
                        ((buf[3] & 0xfe) << 20), 28) & ONES(28);
}

std::string Frame::srcTypeStr() const {
    switch (src_type) {
        case ADDRTYPE_AIRCRAFT: return "Aircraft";
        case ADDRTYPE_GS_ADM:
        case ADDRTYPE_GS_DEL:   return "Ground station";
        case ADDRTYPE_ALL:      return "All stations";
        default:                return "reserved";
    }
}
std::string Frame::dstTypeStr() const {
    switch (dst_type) {
        case ADDRTYPE_AIRCRAFT: return "Aircraft";
        case ADDRTYPE_GS_ADM:
        case ADDRTYPE_GS_DEL:   return "Ground station";
        case ADDRTYPE_ALL:      return "All stations";
        default:                return "reserved";
    }
}

static const char* U_cmd_name(uint8_t mf) {
    switch (mf) {
        case UF_UI:   return "UI";
        case UF_DM:   return "DM";
        case UF_DISC: return "DISC";
        case UF_UA:   return "UA";
        case UF_FRMR: return "FRMR";
        case UF_XID:  return "XID";
        case UF_TEST: return "TEST";
        default:      return "U";
    }
}
static const char* S_cmd_name(uint8_t sf) {
    static const char* s[] = { "Receive Ready", "Receive not Ready", "Reject", "Selective Reject" };
    return s[sf & 0x3];
}

// ACARS characters are 7-bit ISO-5 with bit 7 used as an odd-parity bit.
// Keep printable content, trimming the leading/trailing padding (dots/spaces)
// used in fixed-width fields such as registration, sequence and flight id.
static std::string trim_pad(const uint8_t* p, int n) {
    std::string s;
    for (int i = 0; i < n; i++) {
        uint8_t c = p[i] & 0x7f;             // drop parity bit
        if (c < 0x20 || c == 0x7f) continue; // drop control / DEL
        s += (char)c;
    }
    size_t a = s.find_first_not_of(" .");
    size_t b = s.find_last_not_of(" .");
    if (a == std::string::npos) return "";
    return s.substr(a, b - a + 1);
}

// Single ACARS character: strip parity, map non-printables to space.
static inline char acars_char(uint8_t c) {
    c &= 0x7f;
    return (c >= 0x20 && c < 0x7f) ? (char)c : ' ';
}

// Parse the ACARS message body that follows the 0xFF 0xFF 0x01 prefix.
// Follows libacars (la_acars.c): the buffer must end with a DEL byte, carry a
// valid Block Check Sequence (CRC-16/CCITT over the parity-bearing octets,
// init 0, residue 0) and an ETX/ETB terminator. Returns false if the block is
// structurally invalid (the caller should then drop the frame, as dumpvdl2
// does). The downlink-specific sequence/flight fields are extracted when the
// block id is a digit (IS_DOWNLINK_BLK), independent of the DLC address type.
static bool parse_acars(Frame& f, const uint8_t* p, int len) {
    // LA_ACARS_PREAMBLE_LEN = 16 (mode..ETX + 2-byte BCS + DEL, no SOH).
    if (len < 16) return false;
    if (p[len - 1] != 0x7f) return false;            // must end with DEL
    int n = len - 1;                                  // drop DEL

    // Block Check Sequence over the raw (parity-bearing) octets, init 0.
    f.acars_crc_ok = (crc16_ccitt(p, (uint32_t)n, 0) == 0);
    n -= 2;                                           // drop the 2 BCS bytes
    if (n < 12) return false;

    // Work on a parity-stripped copy.
    std::vector<uint8_t> b((size_t)n);
    for (int i = 0; i < n; i++) b[i] = p[i] & 0x7f;

    uint8_t term = b[n - 1];
    if (term != 0x03 && term != 0x17) return false;   // ETX or ETB expected
    n--;                                              // drop terminator

    f.has_acars = true;
    int idx = 0;
    f.acars_mode = acars_char(b[idx]); idx++;
    f.acars_reg  = trim_pad(&b[idx], 7); idx += 7;
    uint8_t ack  = b[idx]; idx++;
    f.acars_nak  = (ack == 0x15);                     // NAK
    char l0 = (char)b[idx], l1 = (char)b[idx + 1]; idx += 2;
    if ((uint8_t)l1 == 0x7f) l1 = 'd';                // "_d" squitter label
    f.acars_label = std::string(1, acars_char((uint8_t)l0)) + std::string(1, acars_char((uint8_t)l1));
    uint8_t blk = b[idx]; idx++;
    f.acars_block_id = (blk == 0) ? ' ' : acars_char(blk);
    bool downlink = (blk >= '0' && blk <= '9');       // IS_DOWNLINK_BLK

    int remaining = n - idx;
    if (remaining < 1) {
        // Empty text: valid for an uplink (incl. _d squitter ACKs); a downlink
        // with no text field is malformed.
        if (!downlink) { f.acars_text.clear(); return true; }
        return false;
    }
    if (b[idx] != 0x02) return false;                 // STX expected
    idx++; remaining--;

    if (downlink) {
        if (remaining < 10) return false;
        f.acars_msg_seq = trim_pad(&b[idx], 4); idx += 4; remaining -= 4;
        f.acars_flight  = trim_pad(&b[idx], 6); idx += 6; remaining -= 6;
    }

    std::string txt;
    for (int i = 0; i < remaining; i++) {
        uint8_t c = b[idx + i];
        if (c == 0) { txt += '.'; continue; }                 // NUL -> '.'
        if (c == '\n' || c == '\r' || c == '\t') { txt += ' '; continue; }
        if (c < 0x20 || c == 0x7f) continue;                  // other control
        txt += (char)c;
    }
    while (!txt.empty() && txt.back() == ' ') txt.pop_back();
    f.acars_text = txt;
    return true;
}

Frame parse_avlc(const RawFrame& raw) {
    Frame f;
    f.frame_pwr_dbfs = raw.frame_pwr_dbfs;
    f.nf_dbfs = raw.nf_dbfs;
    f.fec_corrections = raw.fec_corrections;
    f.header_syndrome = raw.header_syndrome;

    const uint8_t* buf = raw.data.data();
    uint32_t len = (uint32_t)raw.data.size();
    if (len < MIN_AVLC_LEN) return f;

    if (crc16_ccitt(buf, len, 0xFFFFu) != GOOD_FCS) return f; // FCS bad
    f.fcs_ok = true;
    len -= 2; // strip FCS

    const uint8_t* ptr = buf;
    uint32_t dst = parse_dlc_addr(ptr); ptr += 4; len -= 4;
    uint32_t src = parse_dlc_addr(ptr); ptr += 4; len -= 4;

    f.dst_addr = dst & 0xFFFFFF;
    f.dst_type = (dst >> 24) & 0x7;
    f.airborne = !((dst >> 27) & 0x1);       // status bit: 0 = airborne (A/G)
    f.src_addr = src & 0xFFFFFF;
    f.src_type = (src >> 24) & 0x7;
    f.is_command = ((src >> 27) & 0x1) == 0; // status bit: 0 = command

    uint8_t lcf = *ptr++; len--;

    if ((lcf & 0x1) == 0x0) {            // I-frame
        f.lcf_class = 'I';
        f.lcf_desc = "Information";
        if (len > 3 && ptr[0] == 0xff && ptr[1] == 0xff && ptr[2] == 0x01) {
            f.proto = "ACARS";
            if (!parse_acars(f, ptr + 3, (int)len - 3)) {
                f.acars_malformed = true; // structurally invalid -> caller drops
            }
        } else {
            f.proto = "X.25/ISO 8208";
        }
    } else if ((lcf & 0x3) == 0x1) {     // S-frame
        f.lcf_class = 'S';
        uint8_t sf = (lcf >> 2) & 0x3;
        f.lcf_desc = std::string("Supervisory: ") + S_cmd_name(sf);
        f.proto = "AVLC S-frame";
    } else {                             // U-frame
        f.lcf_class = 'U';
        uint8_t mf = lcf & 0x3b;         // U_MFUNC mask
        f.lcf_desc = std::string("Unnumbered: ") + U_cmd_name(mf);
        if (mf == UF_XID) f.proto = "XID";
        else              f.proto = "AVLC U-frame";
    }

    f.valid = true;
    return f;
}

} // namespace vdl2
