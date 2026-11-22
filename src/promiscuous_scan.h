#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "board_profiles.h"

/*
 * promiscuous_scan.h / promiscuous_scan.c
 *
 * Real magic-ID promiscuous discovery, ported from
 * sieveandsand/Hacking_X1_Pro's sniffer.cpp -- replaces the earlier
 * RSSI-threshold guess, which could never actually confirm a brand,
 * only suggest "something's transmitting here."
 *
 * THE TECHNIQUE (confirmed from the real source, not guessed):
 *   SETUP_AW's "00" encoding is marked RESERVED/illegal in the
 *   nRF24L01+ datasheet. Writing it anyway, combined with a 2-byte
 *   RX address and auto-ack/CRC/dynamic-payload all disabled, puts
 *   the chip into a raw promiscuous capture mode: it will shift out
 *   bytes for ANY nearby packet-like RF energy, not just ones
 *   matching a real configured address. This is the same class of
 *   technique public nRF24 promiscuous-sniffing tools use. It is not
 *   something this file invents -- it's a faithful port of a working
 *   implementation.
 *
 * TWO-PASS PROCESS (matches the original exactly):
 *   Pass 1 (scan): sweep the profile's channel range in illegal-width
 *     mode. Whenever RPD (signal present) fires, grab 5 raw bytes.
 *     If the first 2 match the brand's magic_id, treat all 5 as a
 *     CANDIDATE address and tally which channels it showed up on.
 *   Pass 2 (validate): switch back to legal 5-byte addressing, and
 *     for each candidate (byte-order REVERSED -- confirmed in the
 *     original, not a guess) x each channel it appeared on, listen
 *     normally and try to decode a real gear value from the payload.
 *     Only a successful gear decode counts as CONFIRMED. Anything
 *     that doesn't validate is NOT reported as this brand -- it's
 *     left for the generic unidentified sweep to pick up instead,
 *     per the instruction not to show unconfirmed guesses as if
 *     they were identified.
 *
 * This only works for profiles with DetectionPromiscuous set and a
 * real magic_id. Calling it on a profile without one is a no-op that
 * returns false.
 */

typedef struct {
    bool confirmed;
    uint8_t address[5];      // little-endian order, ready for RX_ADDR_P0/TX_ADDR
    uint8_t channel;         // RF_CH value it validated on
    int8_t direction;        // -1 backward, 0 unknown, 1 forward (from the first decoded packet)
} PromiscuousScanResult;

/**
 * Runs the full scan+validate pipeline for one profile. This is slow
 * by nature (the original sweeps the whole band with a multi-second
 * dwell per channel) -- SNIFF_DURATION_MS below is tuned down from
 * the original Arduino default for a snappier Flipper UI, at the
 * cost of possibly missing a weak/infrequent transmitter. If real
 * hardware testing shows too many missed detections, raise it first
 * before suspecting the decode logic itself.
 */
bool promiscuous_scan_and_validate(const BoardProfile* profile, PromiscuousScanResult* out);
