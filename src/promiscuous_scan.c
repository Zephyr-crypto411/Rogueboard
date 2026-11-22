#include "promiscuous_scan.h"
#include "nrf24_hal.h"
#include <furi.h>
#include <string.h>

// Gear values, matching sniffer.h's GearLevel_t exactly - these are
// Exway-specific constants from the real source, not generic.
#define GEAR_FORWARD1  0x00
#define GEAR_FORWARD2  0x08
#define GEAR_FORWARD3  0x10
#define GEAR_FORWARD4  0x18
#define GEAR_BACKWARD1 0x40
#define GEAR_BACKWARD2 0x48
#define GEAR_BACKWARD3 0x50
#define GEAR_BACKWARD4 0x58

#define LOWER_MASK  0xF0
#define HIGHER_MASK 0x0F

#define MAX_CANDIDATES 10
#define MAX_CHANNELS_PER_CANDIDATE 6

// Shortened from the original's SNIFF_DURATION=2000ms per channel --
// that totals well over a minute across the full band, which doesn't
// suit an interactive Flipper scan. If real testing shows missed
// detections, raise these first before suspecting the decode logic.
#define SCAN_DWELL_MS 100
#define VALIDATE_DWELL_MS 300

static bool is_valid_gear(uint8_t gear) {
    return gear == GEAR_FORWARD1 || gear == GEAR_FORWARD2 || gear == GEAR_FORWARD3 || gear == GEAR_FORWARD4 ||
           gear == GEAR_BACKWARD1 || gear == GEAR_BACKWARD2 || gear == GEAR_BACKWARD3 || gear == GEAR_BACKWARD4;
}

static bool rx_data_ready(void) {
    uint8_t status = nrf24_read_register(NRF24_REG_STATUS);
    return (status & 0x40) != 0; // RX_DR
}

static void enter_promiscuous_mode(void) {
    nrf24_write_register(NRF24_REG_CONFIG, 0x03);     // power up, PRIM_RX, CRC disabled
    nrf24_write_register(NRF24_REG_EN_AA, 0x00);       // auto-ack off
    nrf24_write_register(NRF24_REG_RF_SETUP, 0x0F);    // max power, 2Mbps
    nrf24_write_register(NRF24_REG_DYNPD, 0x00);       // dynamic payload off
    nrf24_write_register(NRF24_REG_FEATURE, 0x00);

    // The exploit: SETUP_AW's "00" encoding is reserved/illegal per the
    // nRF24L01+ datasheet. Writing it anyway puts the chip into a raw
    // capture mode that shifts out bytes for any nearby packet-like RF
    // energy, not just ones matching a configured address.
    nrf24_write_register(NRF24_REG_SETUP_AW, 0x00);

    uint8_t illegal_address[2] = {0x55, 0x00};
    nrf24_write_register_bytes(NRF24_REG_RX_ADDR_P0, illegal_address, 2);
}

static void enter_legal_mode(void) {
    nrf24_write_register(NRF24_REG_CONFIG, 0x0F);     // power up, PRIM_RX, CRC enabled (2-byte)
    nrf24_write_register(NRF24_REG_SETUP_AW, 0x03);    // legal 5-byte address width
    nrf24_write_register(NRF24_REG_RF_SETUP, 0x0F);
}

bool promiscuous_scan_and_validate(const BoardProfile* profile, PromiscuousScanResult* out) {
    out->confirmed = false;

    if(!(profile->detection & DetectionPromiscuous)) return false;
    if(profile->magic_id[0] == 0 && profile->magic_id[1] == 0) return false; // no real magic ID set

    uint8_t candidates[MAX_CANDIDATES][5];
    uint8_t candidate_channels[MAX_CANDIDATES][MAX_CHANNELS_PER_CANDIDATE];
    uint8_t candidate_channel_count[MAX_CANDIDATES];
    uint8_t candidate_count = 0;
    memset(candidate_channel_count, 0, sizeof(candidate_channel_count));

    uint16_t channel_count =
        (profile->end_frequency_mhz - profile->start_frequency_mhz) / profile->channel_increment + 1;

    // --- Pass 1: promiscuous scan for the magic ID ---
    enter_promiscuous_mode();

    for(uint16_t i = 0; i < channel_count; i++) {
        uint8_t rf_ch = profile->start_frequency_mhz + i * profile->channel_increment - 2400;
        nrf24_write_register(NRF24_REG_RF_CH, rf_ch);
        nrf24_ce_high();

        uint32_t start = furi_get_tick();
        while(furi_get_tick() - start < furi_ms_to_ticks(SCAN_DWELL_MS)) {
            if(rx_data_ready() && nrf24_read_rpd()) {
                uint8_t raw[5];
                nrf24_read_rx_payload(raw, 5);

                if(raw[0] == profile->magic_id[0] && raw[1] == profile->magic_id[1]) {
                    bool already_known = false;
                    for(uint8_t c = 0; c < candidate_count && !already_known; c++) {
                        if(memcmp(raw, candidates[c], 5) == 0) {
                            already_known = true;
                            if(candidate_channel_count[c] < MAX_CHANNELS_PER_CANDIDATE) {
                                candidate_channels[c][candidate_channel_count[c]++] = rf_ch;
                            }
                        }
                    }
                    if(!already_known && candidate_count < MAX_CANDIDATES) {
                        memcpy(candidates[candidate_count], raw, 5);
                        candidate_channels[candidate_count][0] = rf_ch;
                        candidate_channel_count[candidate_count] = 1;
                        candidate_count++;
                    }
                }
                nrf24_exec_command(NRF24_CMD_FLUSH_RX);
            }
        }
        nrf24_ce_low();
    }

    if(candidate_count == 0) return false;

    // --- Pass 2: validate each candidate by trying to decode a real gear value ---
    enter_legal_mode();

    for(uint8_t c = 0; c < candidate_count; c++) {
        // CONFIRMED in the original source: the raw promiscuous-mode
        // bytes need to be byte-order REVERSED to form the real address.
        uint8_t little_endian_address[5];
        for(uint8_t k = 0; k < 5; k++) little_endian_address[k] = candidates[c][4 - k];

        for(uint8_t ch_i = 0; ch_i < candidate_channel_count[c]; ch_i++) {
            nrf24_write_register_bytes(NRF24_REG_RX_ADDR_P0, little_endian_address, 5);
            nrf24_write_register(NRF24_REG_RF_CH, candidate_channels[c][ch_i]);
            nrf24_ce_high();

            uint32_t start = furi_get_tick();
            while(furi_get_tick() - start < furi_ms_to_ticks(VALIDATE_DWELL_MS)) {
                if(rx_data_ready()) {
                    uint8_t payload[32];
                    nrf24_read_rx_payload(payload, 32);

                    uint8_t gear_lower = payload[profile->gear_lower_byte];
                    uint8_t gear_higher = payload[profile->gear_higher_byte];
                    uint8_t gear = ((gear_higher & HIGHER_MASK) << 4) | ((gear_lower & LOWER_MASK) >> 4);

                    if(is_valid_gear(gear)) {
                        nrf24_ce_low();
                        memcpy(out->address, little_endian_address, 5);
                        out->channel = candidate_channels[c][ch_i];
                        out->direction = (gear <= GEAR_FORWARD4) ? 1 : -1;
                        out->confirmed = true;
                        return true;
                    }
                    nrf24_exec_command(NRF24_CMD_FLUSH_RX);
                }
            }
            nrf24_ce_low();
        }
    }

    return false; // candidates existed but none decoded a valid gear value
}
