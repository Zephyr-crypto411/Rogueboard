#include "nrf24_hal.h"

static FuriHalSpiBusHandle* spi_handle = NULL;

void nrf24_hal_init(void) {
    // External GPIO SPI bus - NOT the internal bus used by the SD card.
    // Cast needed: furi_hal_spi_bus_handle_external is declared const in
    // this firmware, but furi_hal_spi_acquire/release/tx/rx take a
    // non-const handle pointer. We never modify the handle ourselves,
    // only pass it to those API calls, so this is safe.
    spi_handle = (FuriHalSpiBusHandle*)&furi_hal_spi_bus_handle_external;

    furi_hal_gpio_init_simple(NRF24_CE_PIN, GpioModeOutputPushPull);
    furi_hal_gpio_init_simple(NRF24_CSN_PIN, GpioModeOutputPushPull);

    furi_hal_gpio_write(NRF24_CSN_PIN, true);  // CSN idle high
    furi_hal_gpio_write(NRF24_CE_PIN, false);  // CE idle low (standby)
}

void nrf24_hal_deinit(void) {
    furi_hal_gpio_write(NRF24_CE_PIN, false);
    furi_hal_gpio_write(NRF24_CSN_PIN, true);
}

static void csn_low(void) {
    furi_hal_gpio_write(NRF24_CSN_PIN, false);
}

static void csn_high(void) {
    furi_hal_gpio_write(NRF24_CSN_PIN, true);
}

uint8_t nrf24_read_register(uint8_t reg) {
    uint8_t value = 0;
    nrf24_read_register_bytes(reg, &value, 1);
    return value;
}

void nrf24_write_register(uint8_t reg, uint8_t value) {
    nrf24_write_register_bytes(reg, &value, 1);
}

void nrf24_read_register_bytes(uint8_t reg, uint8_t* buffer, uint8_t length) {
    furi_hal_spi_acquire(spi_handle);
    csn_low();

    uint8_t cmd = NRF24_CMD_R_REGISTER | (reg & 0x1F);
    furi_hal_spi_bus_tx(spi_handle, &cmd, 1, FuriWaitForever);
    // STATUS byte is clocked out alongside the command; ignore it here.
    furi_hal_spi_bus_rx(spi_handle, buffer, length, FuriWaitForever);

    csn_high();
    furi_hal_spi_release(spi_handle);
}

void nrf24_write_register_bytes(uint8_t reg, const uint8_t* values, uint8_t length) {
    furi_hal_spi_acquire(spi_handle);
    csn_low();

    uint8_t cmd = NRF24_CMD_W_REGISTER | (reg & 0x1F);
    furi_hal_spi_bus_tx(spi_handle, &cmd, 1, FuriWaitForever);
    furi_hal_spi_bus_tx(spi_handle, (uint8_t*)values, length, FuriWaitForever);

    csn_high();
    furi_hal_spi_release(spi_handle);
}

bool nrf24_read_rpd(void) {
    // RPD is bit 0 of its register; all other bits are reserved/zero.
    uint8_t rpd = nrf24_read_register(NRF24_REG_RPD);
    return (rpd & 0x01) != 0;
}

uint8_t nrf24_exec_command(uint8_t cmd) {
    uint8_t tx = cmd;
    uint8_t rx = 0;
    furi_hal_spi_acquire(spi_handle);
    csn_low();
    furi_hal_spi_bus_trx(spi_handle, &tx, &rx, 1, FuriWaitForever);
    csn_high();
    furi_hal_spi_release(spi_handle);
    return rx; // STATUS is always shifted out during the command byte
}

void nrf24_write_tx_payload(const uint8_t* data, uint8_t len) {
    furi_hal_spi_acquire(spi_handle);
    csn_low();
    uint8_t cmd = NRF24_CMD_W_TX_PAYLOAD;
    furi_hal_spi_bus_tx(spi_handle, &cmd, 1, FuriWaitForever);
    furi_hal_spi_bus_tx(spi_handle, (uint8_t*)data, len, FuriWaitForever);
    csn_high();
    furi_hal_spi_release(spi_handle);
}

void nrf24_read_rx_payload(uint8_t* buffer, uint8_t len) {
    furi_hal_spi_acquire(spi_handle);
    csn_low();
    uint8_t cmd = NRF24_CMD_R_RX_PAYLOAD;
    furi_hal_spi_bus_tx(spi_handle, &cmd, 1, FuriWaitForever);
    furi_hal_spi_bus_rx(spi_handle, buffer, len, FuriWaitForever);
    csn_high();
    furi_hal_spi_release(spi_handle);
}

void nrf24_pulse_ce(void) {
    furi_hal_gpio_write(NRF24_CE_PIN, true);
    furi_delay_us(15); // datasheet minimum is 10us
    furi_hal_gpio_write(NRF24_CE_PIN, false);
}

void nrf24_ce_high(void) {
    furi_hal_gpio_write(NRF24_CE_PIN, true);
}

void nrf24_ce_low(void) {
    furi_hal_gpio_write(NRF24_CE_PIN, false);
}
