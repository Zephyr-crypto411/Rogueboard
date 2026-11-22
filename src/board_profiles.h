#pragma once

#include <stdint.h>

/*
 * board_profiles.h
 *
 * Central database of known e-skateboard nRF24 protocols, built from
 * everything reverse-engineered so far:
 *
 *   - Exway:
 *       Detection = PROMISCUOUS. Board+remote are already paired on an
 *       address we don't know. We capture raw air traffic in illegal-
 *       address mode and search for a 2-byte manufacturer "magic ID"
 *       (0x39 0x32) to find candidate addresses, then validate by
 *       listening on each candidate and checking for sane gear values.
 *
 *   - Acton Blink QU4TR0
 *       Detection = FIXED PAIRING ADDRESS. An unpaired/pairing-mode
 *       board listens on a universal, brand-wide address
 *       (0xE3 0xE3 0xE3 0xE3 0xE3). We address it directly and look
 *       for an ACK. The *paired* operating address is then derived as
 *       [byte0, 0xFF-byte0, 0xE3, 0xE3, 0xE3] -- only one byte is
 *       actually random.
 *
 *   - Meepo M6C:SOON. No protocol details yet (see areas/
 *     rogueboard.md). Placeholder profile below, flagged as such --
 *     fill in once a real capture or FCC/teardown is in hand.
 *
 * Each profile says which detection method applies so the scan loop
 * can branch accordingly. A board could in principle support both
 * (sniff it while paired, OR catch it brand-new in pairing mode) --
 * the enum is a bitmask-able style for that reason, though today's
 * profiles each only populate one path.
 */

typedef enum {
    DetectionNone        = 0,
    DetectionPromiscuous  = 1 << 0,  // search raw traffic for magic ID
    DetectionFixedPairing = 1 << 1,  // probe known universal pairing address
} DetectionMethod;

typedef struct {
    const char* brand_name;

    DetectionMethod detection;

    // --- Radio config ---
    uint16_t start_frequency_mhz;   // e.g. 2402
    uint16_t end_frequency_mhz;     // e.g. 2480
    uint8_t  channel_increment;     // MHz step between channels (1 or 2)
    uint8_t  data_rate_2mbps;       // 1 = 2Mbps, 0 = 1Mbps (250kbps unhandled for now)

    // --- Promiscuous-mode fields (used if DetectionPromiscuous set) ---
    uint8_t magic_id[2];            // manufacturer ID bytes to search for; {0,0} = none

    // --- Fixed-pairing-mode fields (used if DetectionFixedPairing set) ---
    uint8_t pairing_address[5];     // universal address board listens on pre-pairing
    uint8_t pairing_cmd_byte;       // first byte of the binding message (e.g. Acton's 0xC0)

    // --- Payload layout (nibble/byte offsets into the data packet) ---
    // Set to 0xFF for "unknown/not yet reverse engineered" fields.
    uint8_t gear_lower_byte;
    uint8_t gear_higher_byte;
    uint8_t throttle_lower_byte;
    uint8_t throttle_middle_byte;
    uint8_t throttle_higher_byte;

    // Free-text note for anything that doesn't fit the struct yet --
    // confidence level, open questions, source citation.
    const char* notes;
} BoardProfile;

#define PROFILE_COUNT 3

static const BoardProfile board_profiles[PROFILE_COUNT] = {
    {
        .brand_name = "Exway",
        .detection = DetectionPromiscuous,
        .start_frequency_mhz = 2402,
        .end_frequency_mhz = 2480,
        .channel_increment = 2,
        .data_rate_2mbps = 1,
        .magic_id = {0x39, 0x32},
        .pairing_address = {0, 0, 0, 0, 0},   // n/a for this detection mode
        .pairing_cmd_byte = 0,
        .gear_lower_byte = 5,
        .gear_higher_byte = 4,
        .throttle_lower_byte = 7,
        .throttle_middle_byte = 6,
        .throttle_higher_byte = 5,            // overlaps gear_lower_byte - see sniffer.h
        .notes = "Confirmed from sieveandsand/Hacking_X1_Pro project_sniffer. "
                 "Gear: forward1-4 = 0x00/0x08/0x10/0x18, backward1-4 = 0x40/0x48/0x50/0x58. "
                 "Detection now does the REAL promiscuous magic-ID search + gear-decode "
                 "validation (see promiscuous_scan.c) - no longer an RSSI guess.",
    },
    {
        .brand_name = "Acton Blink QU4TRO",
        .detection = DetectionFixedPairing,
        .start_frequency_mhz = 2402,
        .end_frequency_mhz = 2480,             // CONFIRMED from Hardware.cpp - real table is
                                                // interleaved across the full band, NOT linear;
                                                // see acton_channel_table[] in the app for the
                                                // actual 16 values (this struct can't express a
                                                // non-sequential hop pattern, so the app uses its
                                                // own hardcoded table rather than these two fields)
        .channel_increment = 2,                // not really applicable - real hops aren't evenly spaced
        .data_rate_2mbps = 1,
        .magic_id = {0, 0},                   // n/a for this detection mode
        .pairing_address = {0xE3, 0xE3, 0xE3, 0xE3, 0xE3},
        .pairing_cmd_byte = 0xC0,
        .gear_lower_byte = 0xFF,               // not applicable - Acton encodes direction
        .gear_higher_byte = 0xFF,               // differently (drive message byte[1], not a gear nibble)
        .throttle_lower_byte = 1,              // quatroMessage[1] is the throttle byte directly
        .throttle_middle_byte = 0xFF,
        .throttle_higher_byte = 0xFF,
        .notes = "Confirmed Throttle is a single byte: "
                 "boardMin=32 (max accel), boardCenter=128 (neutral), boardMax=230 (max brake). "
                 "Paired address = [b0, 0xFF-b0, 0xE3, 0xE3, 0xE3], only b0 is truly random. "
                 "Channel table (frequencies[16]) and initial quatroAddress[] not yet obtained "
                 "(live in Hardware.cpp).",
    },
    {
        .brand_name = "Meepo (M6C remote) - UNCONFIRMED",
        .detection = DetectionNone,            // no protocol details yet - placeholder only
        .start_frequency_mhz = 2410,            // from FCC test report antenna measurements
        .end_frequency_mhz = 2480,               // same source - NOT confirmed as actual channel usage
        .channel_increment = 10,                 // FCC test points were every 10MHz - likely NOT the real hop step
        .data_rate_2mbps = 0,                    // unknown
        .magic_id = {0, 0},                      // unknown - chip/modulation unconfirmed
        .pairing_address = {0, 0, 0, 0, 0},       // unknown
        .pairing_cmd_byte = 0,
        .gear_lower_byte = 0xFF,
        .gear_higher_byte = 0xFF,
        .throttle_lower_byte = 0xFF,
        .throttle_middle_byte = 0xFF,
        .throttle_higher_byte = 0xFF,
        .notes = "PLACEHOLDER ONLY. FCC filing 2BOHH-MB2251 confirms 2.4GHz PCB antenna, "
                 "2400-2500MHz test range - that's it. Chipset, modulation, hopping, and "
                 "addressing all unconfirmed. Needs teardown/chip ID or an SPI/promiscuous "
                 "capture from the actual M6C before this profile means anything. "
                 "Do not trust these radio-config values for a real scan.",
    },
};
