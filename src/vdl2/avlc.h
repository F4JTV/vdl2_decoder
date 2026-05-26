/*
 * avlc.h - AVLC (Aviation VHF Link Control) frame parser for VDL Mode 2.
 *
 * Takes a de-framed AVLC octet string (as produced by vdl2::Demodulator),
 * verifies the CRC-16/CCITT FCS, decodes the DLC source/destination addresses
 * and the link control field, and extracts an embedded ACARS message when one
 * is present. Higher network-layer PDUs (X.25 / CLNP / XID) are identified but
 * left as raw payload for compactness.
 *
 * Address decoding and frame structure follow dumpvdl2 (avlc.c, GPLv3).
 */
#pragma once
#include <cstdint>
#include <string>
#include "vdl2.h"

namespace vdl2 {

enum AddrType {
    ADDRTYPE_RESERVED  = 0,
    ADDRTYPE_AIRCRAFT  = 1,
    ADDRTYPE_GS_ADM    = 4,
    ADDRTYPE_GS_DEL    = 5,
    ADDRTYPE_ALL       = 7
};

struct Frame {
    bool valid = false;          // FCS check passed and frame structurally sane
    bool fcs_ok = false;

    // DLC addresses
    uint32_t src_addr = 0;       // 24-bit
    uint32_t dst_addr = 0;
    int src_type = 0;            // AddrType
    int dst_type = 0;
    bool airborne = false;       // from dst A/G status bit
    bool is_command = false;     // from src C/R status bit

    // Link control field
    char lcf_class = 'U';        // 'I', 'S', or 'U'
    std::string lcf_desc;        // human-readable control description

    // Higher layer
    std::string proto = "Unknown";

    // ACARS payload (if present)
    bool has_acars = false;
    bool acars_crc_ok = false;   // ACARS Block Check Sequence verified
    bool acars_malformed = false; // structurally invalid ACARS (should be dropped)
    char acars_mode = ' ';
    std::string acars_reg;       // aircraft registration
    bool acars_nak = false;      // technical acknowledgement was NAK
    std::string acars_label;
    char acars_block_id = ' ';
    std::string acars_msg_seq;   // downlink message sequence number
    std::string acars_flight;    // downlink flight id
    std::string acars_text;      // free text

    // Reception metadata (copied from RawFrame)
    float frame_pwr_dbfs = 0.f;
    float nf_dbfs = 0.f;
    int   fec_corrections = 0;
    int   header_syndrome = 0;

    std::string srcTypeStr() const;
    std::string dstTypeStr() const;
};

// Parse one de-framed AVLC octet string. The returned Frame has .valid==false
// if the FCS check fails or the frame is malformed.
Frame parse_avlc(const RawFrame& raw);

uint16_t crc16_ccitt(const uint8_t* data, uint32_t len, uint16_t crc_init);

} // namespace vdl2
