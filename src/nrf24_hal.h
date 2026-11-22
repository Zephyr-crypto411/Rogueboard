#pragma once

#include <furi_hal_spi.h>
#include <furi_hal_gpio.h>
#include <furi_hal_resources.h>
#include <stdint.h>

/*
 * nrf24_hal.h / nrf24_hal.c
 *
 * Minimal register-level nRF24L01+ driver for the Flipper Zero.
 * This REPLACES the Arduino RF24 library calls that sniffer.cpp relies
 * on (SPI.transfer(), digitalWrite(), etc. don't exist on Flipper).
 *
 * Porting plan:
 *   v0.1 (this stub): confirm wiring + SPI bus by reading a known
 *                      register (STATUS) and showing it on screen.
 *   v0.2: implement the same register writes sniffer.cpp's init()
 *         performs (illegal RX address, PA level, 2Mbps, CRC/AutoACK
 *         off, dynamic payload off) using the functions below.
 *   v0.3: port scanWithMagicID() / _testIfValidExway() /
 *         _listenWithChannelHopping() logic, swapping RF24 library
 *         calls for these furi_hal-based ones.
 *
 * Wiring: this follows the de facto community-standard nRF24 pinout
 * used by essentially every Flipper nRF24 app (Channel Scanner,
 * mousejack tools, etc.) -- it's not Momentum-specific, but it works
 * on Momentum because it matches what furi_hal_spi_bus_handle_external
 * already wires MOSI/MISO/SCK to in firmware.
 *
 *   nRF24 VCC  -> Flipper pin 9  (3V3)  -- NOT 5V
 *   nRF24 GND  -> Flipper pin 8  (GND)
 *   nRF24 SCK  -> Flipper pin 5  (B3)   -- fixed by furi_hal external SPI bus
 *   nRF24 MOSI -> Flipper pin 2  (A7)   -- fixed by furi_hal external SPI bus
 *   nRF24 MISO -> Flipper pin 3  (A6)   -- fixed by furi_hal external SPI bus
 *   nRF24 CSN  -> Flipper pin 4  (A4)
 *   nRF24 CE   -> Flipper pin 6  (B2)
 *   nRF24 IRQ  -> not connected
 *
 * The Flipper's external SPI bus is shared with other peripherals
 * (e.g. the SD card uses the internal bus, not this one, so that's
 * fine) -- always acquire/release the handle around each transaction,
 * which the furi_hal_spi_bus_* calls below handle for you.
 */

// --- nRF24L01+ command bytes (same as RF_Comm.h from the Acton repo --
// these are straight from the datasheet, not device-specific) ---
#define NRF24_CMD_R_REGISTER       0x00
#define NRF24_CMD_W_REGISTER       0x20
#define NRF24_CMD_R_RX_PAYLOAD     0x61
#define NRF24_CMD_W_TX_PAYLOAD     0xA0
#define NRF24_CMD_FLUSH_TX         0xE1
#define NRF24_CMD_FLUSH_RX         0xE2
#define NRF24_CMD_NOP              0xFF

// --- Registers we need for v0.1/v0.2 ---
#define NRF24_REG_CONFIG           0x00
#define NRF24_REG_EN_AA            0x01
#define NRF24_REG_EN_RXADDR        0x02
#define NRF24_REG_SETUP_AW         0x03
#define NRF24_REG_RF_CH            0x05
#define NRF24_REG_RF_SETUP         0x06
#define NRF24_REG_STATUS           0x07
#define NRF24_REG_RX_ADDR_P0       0x0A
#define NRF24_REG_TX_ADDR          0x10
#define NRF24_REG_RX_PW_P0         0x11
#define NRF24_REG_DYNPD            0x1C
#define NRF24_REG_FEATURE          0x1D
#define NRF24_REG_RPD              0x09   // Received Power Detector - crude RSSI

// --- Pin assignment: matches the de facto community nRF24 pinout
// (pin 6/B2 = CE, pin 4/A4 = CSN) -- see wiring table above.
#define NRF24_CE_PIN    &gpio_ext_pb2
#define NRF24_CSN_PIN   &gpio_ext_pa4

/**
 * Initializes the SPI bus handle and CE/CSN GPIO pins.
 * Call once at app startup.
 */
void nrf24_hal_init(void);

/**
 * Releases GPIO pins. Call on app exit.
 */
void nrf24_hal_deinit(void);

/**
 * Reads a single register. Mirrors RF_Comm.h's read_FromAddress().
 */
uint8_t nrf24_read_register(uint8_t reg);

/**
 * Writes a single byte to a register. Mirrors write_ToAddress().
 */
void nrf24_write_register(uint8_t reg, uint8_t value);

/**
 * Reads `length` bytes from a register (e.g. a 5-byte address field).
 */
void nrf24_read_register_bytes(uint8_t reg, uint8_t* buffer, uint8_t length);

/**
 * Writes `length` bytes to a register.
 */
void nrf24_write_register_bytes(uint8_t reg, const uint8_t* values, uint8_t length);

/**
 * Reads the RPD (Received Power Detector) register.
 *
 * This is NOT a true dBm RSSI value -- the nRF24L01+ only exposes a
 * single bit: whether received power exceeded roughly -64dBm at any
 * point since the register was last read. Treat it as "signal
 * present / strong nearby" vs "weak or absent", not a precise meter.
 * Useful for a rough signal-strength bar (e.g. 1-2 bars if set,
 * 0 bars if clear over several samples) but don't present it to the
 * user as a specific dBm number -- that would overstate precision
 * the hardware doesn't have.
 *
 * @return true if power was detected above threshold, false otherwise
 */
bool nrf24_read_rpd(void);

/**
 * Executes a single-byte command (FLUSH_TX, FLUSH_RX, STATUS, NOP) and
 * returns the STATUS byte the chip always shifts out on the first
 * byte of any SPI transaction. This is how the Acton protocol reads
 * STATUS without a separate register read -- it just sends NOP or
 * re-sends STATUS and reads what comes back on that same byte.
 */
uint8_t nrf24_exec_command(uint8_t cmd);

/**
 * Writes a TX payload (does NOT transmit by itself -- call
 * nrf24_pulse_ce() afterward, matching the original trigger_CS()).
 */
void nrf24_write_tx_payload(const uint8_t* data, uint8_t len);

/**
 * Reads a received payload out of the RX FIFO.
 */
void nrf24_read_rx_payload(uint8_t* buffer, uint8_t len);

/**
 * Pulses CE high briefly (>10us per datasheet) then low -- this is
 * what actually triggers a queued TX payload to transmit. Matches
 * RF_Comm.h's trigger_CS(). Do NOT use this for entering RX mode --
 * see nrf24_ce_high(), which must stay high continuously in RX mode
 * (the original author's hard-won note: "this pin has cost me 4h of
 * my life" when it wasn't held high).
 */
void nrf24_pulse_ce(void);

/**
 * Sets CE high (enters RX or TX mode depending on CONFIG register).
 */
void nrf24_ce_high(void);

/**
 * Sets CE low (standby mode).
 */
void nrf24_ce_low(void);
