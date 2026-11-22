#include "acton_protocol.h"
#include "nrf24_hal.h"
#include <furi.h>

static const uint8_t PAIRING_ADDRESS[5] = {0xE3, 0xE3, 0xE3, 0xE3, 0xE3};

static uint8_t drive_tx_buf[5];
static uint8_t drive_rx_buf[8];

// Sets both TX_ADDR and RX_ADDR_P0 to the same 5-byte address, as the
// original does for every stage (it always listens on its own TX
// address for the ACK payload, since ShockBurst ACKs ride back on
// pipe 0 matching TX_ADDR).
static void set_address(const uint8_t addr[5]) {
    nrf24_write_register_bytes(NRF24_REG_TX_ADDR, addr, 5);
    nrf24_write_register_bytes(NRF24_REG_RX_ADDR_P0, addr, 5);
}

static void clear_irqs(void) {
    nrf24_write_register(NRF24_REG_STATUS, 0x70);
}

static void send_two_byte_binding(uint8_t our_identity_byte) {
    uint8_t binding_msg[2] = {0xC0, our_identity_byte};
    nrf24_write_tx_payload(binding_msg, 2);
    nrf24_pulse_ce();
}

ActonPairResult acton_pair(const uint8_t channel_table[16], uint8_t our_identity_byte, ActonPairOutcome* out) {
    out->channel_mhz_offset = 0;

    // --- Stage 1: broadcast on universal pairing address, hop channels ---
    nrf24_write_register(NRF24_REG_CONFIG, 0x0E);    // CRC on, 2-byte, power up, PTX
    nrf24_write_register(NRF24_REG_RF_SETUP, 0x0B);  // low power, 2Mbps

    bool stage1_acked = false;
    uint8_t found_channel_index = 0;

    for(uint8_t ch = 0; ch < 16 && !stage1_acked; ch++) {
        nrf24_write_register(NRF24_REG_RF_CH, channel_table[ch]);
        set_address(PAIRING_ADDRESS);
        clear_irqs();
        nrf24_exec_command(NRF24_CMD_FLUSH_TX);
        clear_irqs();
        send_two_byte_binding(our_identity_byte);

        // Poll STATUS for up to ~10 tries, matching rtr_counter > 10 in original
        for(uint8_t attempt = 0; attempt < 11; attempt++) {
            furi_delay_ms(2);
            uint8_t status1 = nrf24_exec_command(NRF24_CMD_NOP);
            uint8_t status2 = nrf24_exec_command(NRF24_CMD_NOP);
            if(status1 != status2) continue;

            if(status1 & 0x20) { // TX_DS - ACK received
                stage1_acked = true;
                found_channel_index = ch;
                break;
            }
            if(status1 & 0x10) { // MAX_RTR - resend once on same channel
                clear_irqs();
                nrf24_exec_command(NRF24_CMD_FLUSH_TX);
                clear_irqs();
                send_two_byte_binding(our_identity_byte);
            }
            // status1 == 0x0E: still waiting, loop again
        }
    }

    if(!stage1_acked) {
        return ActonPairFailed_NoBoardFound;
    }

    // --- Stage 2: switch to negotiated intermediate address, confirm ---
    uint8_t negotiated_address[5] = {
        our_identity_byte,
        (uint8_t)(0xFF - our_identity_byte),
        0xE3, 0xE3, 0xE3
    };

    set_address(negotiated_address);
    nrf24_exec_command(NRF24_CMD_FLUSH_TX);
    nrf24_write_register(NRF24_REG_RF_SETUP, 0x0D); // medium power, 2Mbps
    clear_irqs();
    clear_irqs();
    send_two_byte_binding(our_identity_byte);

    bool stage2_acked = false;
    for(uint8_t attempt = 0; attempt < 11; attempt++) {
        furi_delay_ms(2);
        uint8_t status1 = nrf24_exec_command(NRF24_CMD_NOP);
        uint8_t status2 = nrf24_exec_command(NRF24_CMD_NOP);
        if(status1 != status2) continue;

        if(status1 & 0x20) {
            stage2_acked = true;
            break;
        }
        if(status1 & 0x10) {
            clear_irqs();
            nrf24_exec_command(NRF24_CMD_FLUSH_TX);
            clear_irqs();
            send_two_byte_binding(our_identity_byte);
        }
    }

    if(!stage2_acked) {
        return ActonPairFailed_NegotiationLost;
    }

    // --- Stage 3: switch to receiver, wait for the board's final address ---
    nrf24_write_register(NRF24_REG_CONFIG, 0x0F); // CRC on, power up, PRIM_RX=1 now
    nrf24_exec_command(NRF24_CMD_FLUSH_RX);
    clear_irqs();
    nrf24_write_register(NRF24_REG_RF_SETUP, 0x0F); // full power, 2Mbps

    // CRITICAL: CE must be held continuously HIGH in RX mode, not pulsed.
    nrf24_ce_high();

    bool got_address = false;
    uint8_t pairing_return[3] = {0, 0, 0};

    for(uint16_t attempt = 0; attempt < 101; attempt++) { // matches rtr_counter > 100 in original
        furi_delay_ms(2);
        uint8_t status1 = nrf24_exec_command(NRF24_CMD_NOP);
        uint8_t status2 = nrf24_exec_command(NRF24_CMD_NOP);
        if(status1 != status2) continue;

        if(status1 & 0x40) { // RX_DR - data arrived
            nrf24_read_rx_payload(pairing_return, 3);
            got_address = true;
            break;
        }
        // status1 == 0x0E: keep waiting
    }

    nrf24_ce_low(); // back to standby before caller does anything else

    if(!got_address) {
        return ActonPairFailed_Timeout;
    }

    out->operating_address[0] = pairing_return[1];
    out->operating_address[1] = pairing_return[2];
    // CONFIRMED from Hardware.cpp: quatroAddress[5] default = {0x51,0xFC,0xC9,0xC9,0xC9}.
    // Only bytes [0] and [1] are ever overwritten (by EEPROM or the pairing
    // reply); bytes [2:4] stay fixed at 0xC9. Since this was reverse-engineered
    // from a real Acton remote's SPI traffic, 0xC9 is very likely the real
    // board's own expectation, not an arbitrary choice.
    out->operating_address[2] = 0xC9;
    out->operating_address[3] = 0xC9;
    out->operating_address[4] = 0xC9;
    out->channel_mhz_offset = found_channel_index;

    return ActonPairSuccess;
}

static const uint8_t* drive_channel_table;
static uint8_t drive_channel_index;
static uint8_t drive_operating_address[5];

void acton_drive_init(const uint8_t operating_address[5], const uint8_t channel_table[16], uint8_t starting_channel_index) {
    for(int i = 0; i < 5; i++) drive_operating_address[i] = operating_address[i];
    drive_channel_table = channel_table;
    drive_channel_index = starting_channel_index;

    nrf24_write_register(NRF24_REG_CONFIG, 0x0E);   // CRC on, power up, PTX
    nrf24_write_register(NRF24_REG_RF_SETUP, 0x0F); // full power, 2Mbps
    nrf24_write_register(NRF24_REG_RF_CH, drive_channel_table[drive_channel_index]);
    set_address(drive_operating_address);
    clear_irqs();
    nrf24_exec_command(NRF24_CMD_FLUSH_TX);
}

bool acton_send_drive_frame(uint8_t throttle_byte, uint8_t* out_board_battery_state) {
    nrf24_exec_command(NRF24_CMD_FLUSH_TX);
    nrf24_exec_command(NRF24_CMD_FLUSH_RX);
    clear_irqs();

    drive_tx_buf[0] = 0x00;
    drive_tx_buf[1] = throttle_byte;
    drive_tx_buf[2] = 0x00;
    drive_tx_buf[3] = 0x00;
    drive_tx_buf[4] = 0x00;

    nrf24_write_tx_payload(drive_tx_buf, 5);
    nrf24_pulse_ce();

    for(uint8_t attempt = 0; attempt < 3; attempt++) {
        furi_delay_ms(3);
        uint8_t status1 = nrf24_exec_command(NRF24_CMD_NOP);
        uint8_t status2 = nrf24_exec_command(NRF24_CMD_NOP);
        if(status1 != status2) continue;

        if(status1 & 0x60) { // both TX_DS and RX_DR - ACK with payload received
            nrf24_read_rx_payload(drive_rx_buf, 8);
            if(out_board_battery_state) *out_board_battery_state = drive_rx_buf[0] & 0x0F;
            return true;
        }
        if(status1 & 0x20) { // TX_DS only, no ACK payload this time - still "connected"
            return true;
        }
        if(status1 & 0x10) {
            // MAX_RTR - connection likely lost.
            return false;
        }
    }
    return false;
}
