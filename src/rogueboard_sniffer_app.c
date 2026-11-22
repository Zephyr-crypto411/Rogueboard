#include <furi.h>
#include <gui/gui.h>
#include <input/input.h>
#include "nrf24_hal.h"
#include "board_profiles.h"
#include "acton_protocol.h"
#include "promiscuous_scan.h"

/*
 * RogueBoard Sniffer - v0.4
 *
 * Adds REAL Acton pairing and basic drive control on top of v0.3's
 * scanner. Exway is still scan-only (no drive protocol known yet).
 *
 * SAFETY DESIGN (read before testing):
 *   - ARM is explicit: long-press OK to toggle armed/disarmed. Until
 *     armed, Up/Down do nothing and the board is held at neutral.
 *   - Throttle is RAMPED, never snapped: holding Up/Down moves a
 *     *target*, and the actual sent value steps toward it a little
 *     each tick (THROTTLE_STEP_PER_TICK). Letting go ramps back to
 *     neutral the same way, it doesn't cut instantly.
 *   - Releasing both buttons, disarming, or losing the connection
 *     (MAX_RTR from acton_send_drive_frame) all force the target
 *     back to neutral immediately.
 *   - Exiting (Back) always sends one final neutral frame before
 *     leaving drive mode, whether armed or not.
 *   - TEST WITH WHEELS OFF THE GROUND. This sends real commands to a
 *     real board once paired and armed.
 */

#define ACTON_BOARD_MIN     32   // max acceleration
#define ACTON_BOARD_CENTER  128  // neutral
#define ACTON_BOARD_MAX     230  // max brake
#define THROTTLE_STEP_PER_TICK 4 // ramp speed - higher = more aggressive
#define DRIVE_TICK_MS       40

typedef enum {
    ScreenMenu,
    ScreenPairing,
    ScreenDrive,
} Screen;

#define MAX_FOUND_DEVICES       8

/*
 * Found-devices list model (replaces the static per-brand menu).
 *
 * A row is either:
 *   - a KNOWN match: some profile's presence check (RSSI sweep for
 *     promiscuous brands, or a lightweight ACK probe for fixed-pairing
 *     brands) found something, so we label it by brand.
 *   - an UNKNOWN hit: the generic full-spectrum RSSI sweep found energy
 *     on a channel that no known profile's range/signature explains.
 *     Labeled "UNIDENTIFIED" rather than silently dropped, same as
 *     your mockup.
 *
 * mph is NOT live data yet for any brand - no profile has a working
 * speed decode ported. It always shows "??" right now. The field
 * exists so a future speed estimator (e.g. porting Acton's dead-
 * reckoning model from QuatroRemote.ino) has somewhere to plug in
 * without another UI rework.
 */
typedef struct {
    char label[16];
    uint8_t rssi_bars;
    bool mph_known;
    uint8_t mph;
    int profile_index; // -1 if unidentified (no profile match)
} FoundDevice;

static FoundDevice found_devices[MAX_FOUND_DEVICES];
static uint8_t found_device_count;
static int8_t selected_device = -1;

// --- Acton channel table: CONFIRMED from Hardware.cpp ---
// Hex values 2.4GHz
// band (2402-2480MHz), not a simple linear sequence. In MHz:
// 2402, 2425, 2448, 2471, 2405, 2428, 2451, 2474,
// 2408, 2431, 2454, 2477, 2411, 2434, 2457, 2480
// Moved up here (was previously further down the file) because
// quick_probe_fixed_pairing() below needs it too.
static const uint8_t acton_channel_table[16] = {
    0x02, 0x19, 0x30, 0x47, 0x05, 0x1C, 0x33, 0x4A,
    0x08, 0x1F, 0x36, 0x4D, 0x0B, 0x22, 0x39, 0x50
};

// Lightweight presence probe for fixed-pairing brands (Acton): a
// real stage-1-only check -- transmit the binding command on the
// universal pairing address across each channel, and see if anything
// ACKs. Deliberately stops there rather than completing the rest of
// the handshake in acton_protocol.c's acton_pair() (negotiation +
// address exchange), since a menu-level "is one here?" scan shouldn't
// silently also bind to it.
//
// This only detects a board that's currently IN PAIRING MODE (e.g. a
// button held on the board) -- it canNOT see an already-paired board
// quietly driving, since that board is listening on its own private
// address, not the universal one. A "not found" here does not mean
// "no Acton board nearby", just "none in pairing mode right now".
static bool quick_probe_fixed_pairing(const BoardProfile* profile, uint8_t* out_bars) {
    nrf24_write_register(NRF24_REG_CONFIG, 0x0E);    // CRC on, 2-byte, power up, PTX
    nrf24_write_register(NRF24_REG_RF_SETUP, 0x0B);  // low power, 2Mbps
    nrf24_write_register(NRF24_REG_SETUP_AW, 0x03);  // 5-byte addresses

    for(uint8_t ch = 0; ch < 16; ch++) {
        nrf24_write_register(NRF24_REG_RF_CH, acton_channel_table[ch]);
        nrf24_write_register_bytes(NRF24_REG_TX_ADDR, profile->pairing_address, 5);
        nrf24_write_register_bytes(NRF24_REG_RX_ADDR_P0, profile->pairing_address, 5);
        nrf24_write_register(NRF24_REG_STATUS, 0x70); // clear IRQs
        nrf24_exec_command(NRF24_CMD_FLUSH_TX);

        uint8_t binding_msg[2] = {profile->pairing_cmd_byte, 0x00};
        nrf24_write_tx_payload(binding_msg, 2);
        nrf24_pulse_ce();

        for(uint8_t attempt = 0; attempt < 11; attempt++) {
            furi_delay_ms(2);
            uint8_t status1 = nrf24_exec_command(NRF24_CMD_NOP);
            uint8_t status2 = nrf24_exec_command(NRF24_CMD_NOP);
            if(status1 != status2) continue;

            if(status1 & 0x20) { // TX_DS - ACK received, something's listening
                *out_bars = 3;
                return true;
            }
            if(status1 & 0x10) { // MAX_RTR - try again on this channel
                nrf24_write_register(NRF24_REG_STATUS, 0x70);
                nrf24_exec_command(NRF24_CMD_FLUSH_TX);
                nrf24_write_tx_payload(binding_msg, 2);
                nrf24_pulse_ce();
            }
        }
    }
    return false;
}

// Generic full-spectrum sweep for anything that doesn't match a known
// profile. Scans the whole legal 2400-2483MHz range and reports up to
// two distinct unexplained hot spots as "UNIDENTIFIED" rows, matching
// the mockup. This cannot tell whether two unidentified hits are the
// same device seen twice or two different devices -- it only has
// channel-level RSSI to go on, no address/identity info.
static void scan_unidentified(FoundDevice* out, uint8_t* out_count, uint8_t max_count) {
    *out_count = 0;
    uint8_t hit_counts[84] = {0};

    for(uint8_t ch = 0; ch < 84; ch += 2) {
        nrf24_write_register(NRF24_REG_RF_CH, ch);
        nrf24_ce_high();
        uint8_t hits = 0;
        for(uint8_t s = 0; s < 8; s++) { // lighter sampling than per-brand sweep - this covers 2x the channels
            if(nrf24_read_rpd()) hits++;
            furi_delay_ms(1);
        }
        nrf24_ce_low();
        hit_counts[ch] = hits;
    }

    for(uint8_t pass = 0; pass < max_count; pass++) {
        uint8_t best_ch = 0, best_hits = 0;
        for(uint8_t ch = 0; ch < 84; ch++) {
            if(hit_counts[ch] > best_hits) { best_hits = hit_counts[ch]; best_ch = ch; }
        }
        if(best_hits < 3) break; // nothing else worth reporting

        FoundDevice* d = &out[*out_count];
        snprintf(d->label, sizeof(d->label), "UNIDENTIFIED");
        d->rssi_bars = (best_hits >= 6) ? 3 : (best_hits >= 4) ? 2 : 1;
        d->mph_known = false;
        d->profile_index = -1;
        (*out_count)++;

        // Mask out this channel and its neighbors so the next pass finds
        // a genuinely different hotspot rather than the same one again.
        for(int8_t m = (int8_t)best_ch - 6; m <= (int8_t)best_ch + 6; m++) {
            if(m >= 0 && m < 84) hit_counts[m] = 0;
        }
    }
}

static void run_full_scan(void) {
    found_device_count = 0;
    selected_device = -1;

    for(int i = 0; i < PROFILE_COUNT && found_device_count < MAX_FOUND_DEVICES; i++) {
        const BoardProfile* profile = &board_profiles[i];
        if(profile->detection == DetectionNone) continue;

        bool found = false;
        uint8_t bars = 0;

        if(profile->detection & DetectionPromiscuous) {
            // Real validation only: a gear value must actually decode.
            // No decode = no row for this brand, ever - an unexplained
            // hit falls through to the generic unidentified sweep below
            // instead of being mislabeled with a brand name.
            PromiscuousScanResult result;
            found = promiscuous_scan_and_validate(profile, &result);
            if(found) bars = 3; // a validated gear decode is as strong a signal as it gets
        } else if(profile->detection & DetectionFixedPairing) {
            found = quick_probe_fixed_pairing(profile, &bars);
        }

        if(found) {
            FoundDevice* d = &found_devices[found_device_count];
            snprintf(d->label, sizeof(d->label), "%s", profile->brand_name);
            d->rssi_bars = bars;
            d->mph_known = false; // no brand has a working speed decode yet
            d->mph = 0;
            d->profile_index = i;
            found_device_count++;
        }
    }

    if(found_device_count < MAX_FOUND_DEVICES) {
        uint8_t unident_count = 0;
        scan_unidentified(&found_devices[found_device_count], &unident_count,
                           MAX_FOUND_DEVICES - found_device_count);
        found_device_count += unident_count;
    }
}

typedef struct {
    FuriMutex* mutex;
    bool running;
    Screen screen;

    // Pairing screen state
    bool pairing_in_progress;
    ActonPairResult pair_result;
    ActonPairOutcome pair_outcome;

    // Drive screen state
    bool armed;
    bool up_held, down_held;
    float current_throttle;
    float target_throttle;
    bool link_ok;
    uint8_t board_battery;
} AppState;

static void draw_rssi_bars(Canvas* canvas, int x, int y, uint8_t bars) {
    for(uint8_t b = 0; b < 3; b++) {
        int h = (b + 1) * 3;
        if(b < bars) canvas_draw_box(canvas, x + b * 5, y - h, 3, h);
        else canvas_draw_frame(canvas, x + b * 5, y - h, 3, h);
    }
}

static void draw_callback(Canvas* canvas, void* ctx) {
    AppState* state = ctx;
    furi_mutex_acquire(state->mutex, FuriWaitForever);
    canvas_clear(canvas);
    canvas_set_font(canvas, FontPrimary);

    if(state->screen == ScreenMenu) {
        canvas_draw_str(canvas, 2, 10, "RogueBoard Sniffer");
        canvas_set_font(canvas, FontSecondary);

        if(found_device_count == 0) {
            canvas_draw_str(canvas, 2, 26, "No devices found.");
            canvas_draw_str(canvas, 2, 38, "OK: scan");
        } else {
            int y = 22;
            for(uint8_t i = 0; i < found_device_count; i++) {
                FoundDevice* d = &found_devices[i];
                bool is_selected = (selected_device == (int8_t)i);

                if(is_selected) canvas_draw_str(canvas, 2, y, ">");
                canvas_draw_str(canvas, 10, y, d->label);

                char mph_str[8];
                snprintf(mph_str, sizeof(mph_str), d->mph_known ? "%dMPH" : "??MPH", d->mph);
                canvas_draw_str(canvas, 82, y, mph_str);

                draw_rssi_bars(canvas, 112, y, d->rssi_bars);
                y += 11;
            }
        }
        canvas_draw_str(canvas, 2, 63, "OK:scan U/D:select Rt:pair");
// If you are reading this, you have found an easter egg!🐣
    } else if(state->screen == ScreenPairing) {
        canvas_draw_str(canvas, 2, 10, "Acton Pairing");
        canvas_set_font(canvas, FontSecondary);
        if(state->pairing_in_progress) {
            canvas_draw_str(canvas, 2, 28, "Put board in pairing mode");
            canvas_draw_str(canvas, 2, 40, "now. Searching...");
        } else {
            switch(state->pair_result) {
                case ActonPairSuccess:
                    canvas_draw_str(canvas, 2, 28, "Paired! Entering drive...");
                    break;
                case ActonPairFailed_NoBoardFound:
                    canvas_draw_str(canvas, 2, 28, "No board found.");
                    canvas_draw_str(canvas, 2, 40, "Is it in pairing mode?");
                    break;
                case ActonPairFailed_NegotiationLost:
                    canvas_draw_str(canvas, 2, 28, "Lost signal mid-pair.");
                    canvas_draw_str(canvas, 2, 40, "Try moving closer.");
                    break;
                case ActonPairFailed_Timeout:
                    canvas_draw_str(canvas, 2, 28, "Board never replied.");
                    canvas_draw_str(canvas, 2, 40, "Channel table may be wrong.");
                    break;
                default: break;
            }
            canvas_draw_str(canvas, 2, 56, "Back: menu");
        }

    } else if(state->screen == ScreenDrive) {
        canvas_draw_str(canvas, 2, 10, "Acton Drive");
        canvas_set_font(canvas, FontSecondary);

        char line[48];
        snprintf(line, sizeof(line), "Armed: %s", state->armed ? "YES" : "no (hold OK)");
        canvas_draw_str(canvas, 2, 24, line);

        snprintf(line, sizeof(line), "Throttle: %d", (int)state->current_throttle);
        canvas_draw_str(canvas, 2, 36, line);

        snprintf(line, sizeof(line), "Link: %s  Batt: %d",
                 state->link_ok ? "OK" : "LOST", state->board_battery);
        canvas_draw_str(canvas, 2, 48, line);

        canvas_draw_str(canvas, 2, 63, "Up/Dn:throttle Bk:stop+exit");
    }

    furi_mutex_release(state->mutex);
}

static void input_callback(InputEvent* event, void* ctx) {
    AppState* state = ctx;
    furi_mutex_acquire(state->mutex, FuriWaitForever);

    if(state->screen == ScreenMenu) {
        if(event->type == InputTypeShort && event->key == InputKeyBack) {
            state->running = false;
        } else if(event->type == InputTypeShort && event->key == InputKeyOk) {
            furi_mutex_release(state->mutex);
            run_full_scan();
            furi_mutex_acquire(state->mutex, FuriWaitForever);
        } else if(event->type == InputTypeShort && event->key == InputKeyUp) {
            if(found_device_count > 0) {
                selected_device = (selected_device <= 0) ? (int8_t)found_device_count - 1 : selected_device - 1;
            }
        } else if(event->type == InputTypeShort && event->key == InputKeyDown) {
            if(found_device_count > 0) {
                selected_device = (selected_device + 1 >= (int8_t)found_device_count) ? 0 : selected_device + 1;
            }
        } else if(event->type == InputTypeShort && event->key == InputKeyRight) {
            // Only brands with a known drive protocol (DetectionFixedPairing
            // today = Acton only) can actually be paired/driven. Selecting an
            // UNIDENTIFIED row or a scan-only brand (Exway) does nothing yet.
            if(selected_device >= 0) {
                int pi = found_devices[selected_device].profile_index;
                if(pi >= 0 && (board_profiles[pi].detection & DetectionFixedPairing)) {
                    state->screen = ScreenPairing;
                    state->pairing_in_progress = true;
                }
            }
        }
    } else if(state->screen == ScreenPairing) {
        if(event->type == InputTypeShort && event->key == InputKeyBack) {
            state->screen = ScreenMenu;
        }
    } else if(state->screen == ScreenDrive) {
        if(event->type == InputTypeLong && event->key == InputKeyOk) {
            state->armed = !state->armed;
            if(!state->armed) state->target_throttle = ACTON_BOARD_CENTER;
        } else if(event->type == InputTypePress && event->key == InputKeyUp) {
            state->up_held = true;
        } else if(event->type == InputTypeRelease && event->key == InputKeyUp) {
            state->up_held = false;
        } else if(event->type == InputTypePress && event->key == InputKeyDown) {
            state->down_held = true;
        } else if(event->type == InputTypeRelease && event->key == InputKeyDown) {
            state->down_held = false;
        } else if(event->type == InputTypeShort && event->key == InputKeyBack) {
            // Force one neutral frame before leaving - handled in main loop
            state->armed = false;
            state->target_throttle = ACTON_BOARD_CENTER;
            state->screen = ScreenMenu;
        }
    }

    furi_mutex_release(state->mutex);
}

int32_t rogueboard_sniffer_app(void* p) {
    UNUSED(p);

    AppState state = {
        .mutex = furi_mutex_alloc(FuriMutexTypeNormal),
        .running = true,
        .screen = ScreenMenu,
        .current_throttle = ACTON_BOARD_CENTER,
        .target_throttle = ACTON_BOARD_CENTER,
        .link_ok = true,
    };

    ViewPort* view_port = view_port_alloc();
    view_port_draw_callback_set(view_port, draw_callback, &state);
    view_port_input_callback_set(view_port, input_callback, &state);

    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, view_port, GuiLayerFullscreen);

    nrf24_hal_init();
    nrf24_write_register(NRF24_REG_SETUP_AW, 0x03);

    while(state.running) {
        furi_mutex_acquire(state.mutex, FuriWaitForever);
        Screen current_screen = state.screen;
        furi_mutex_release(state.mutex);

        if(current_screen == ScreenPairing) {
            ActonPairOutcome outcome;
           
            // 0x42 avoids the reserved 0xE3/0xFF values mentioned in acton_protocol.h.
            ActonPairResult result = acton_pair(acton_channel_table, 0x42, &outcome);

            furi_mutex_acquire(state.mutex, FuriWaitForever);
            state.pairing_in_progress = false;
            state.pair_result = result;
            state.pair_outcome = outcome;
            if(result == ActonPairSuccess) {
                acton_drive_init(outcome.operating_address, acton_channel_table, outcome.channel_mhz_offset);
                state.current_throttle = ACTON_BOARD_CENTER;
                state.target_throttle = ACTON_BOARD_CENTER;
                state.armed = false;
                state.link_ok = true;
                state.screen = ScreenDrive;
            }
            furi_mutex_release(state.mutex);

            view_port_update(view_port);
            furi_delay_ms(1500); // let the result message show before continuing

        } else if(current_screen == ScreenDrive) {
            furi_mutex_acquire(state.mutex, FuriWaitForever);

            if(state.armed) {
                if(state.up_held && !state.down_held) state.target_throttle = ACTON_BOARD_MIN;
                else if(state.down_held && !state.up_held) state.target_throttle = ACTON_BOARD_MAX;
                else state.target_throttle = ACTON_BOARD_CENTER;
            } else {
                state.target_throttle = ACTON_BOARD_CENTER;
            }

            // Ramp current toward target, never snap
            if(state.current_throttle < state.target_throttle) {
                state.current_throttle += THROTTLE_STEP_PER_TICK;
                if(state.current_throttle > state.target_throttle) state.current_throttle = state.target_throttle;
            } else if(state.current_throttle > state.target_throttle) {
                state.current_throttle -= THROTTLE_STEP_PER_TICK;
                if(state.current_throttle < state.target_throttle) state.current_throttle = state.target_throttle;
            }

            uint8_t throttle_byte = (uint8_t)state.current_throttle;
            if(throttle_byte < ACTON_BOARD_MIN) throttle_byte = ACTON_BOARD_MIN;
            if(throttle_byte > ACTON_BOARD_MAX) throttle_byte = ACTON_BOARD_MAX;

            furi_mutex_release(state.mutex);

            uint8_t battery = 0;
            bool ok = acton_send_drive_frame(throttle_byte, &battery);

            furi_mutex_acquire(state.mutex, FuriWaitForever);
            state.link_ok = ok;
            if(ok) state.board_battery = battery;
            if(!ok) {
                // Link appears lost - force neutral and stop acting on input
                // until the connection recovers or the user re-pairs.
                state.armed = false;
                state.target_throttle = ACTON_BOARD_CENTER;
            }
            furi_mutex_release(state.mutex);

            view_port_update(view_port);
            furi_delay_ms(DRIVE_TICK_MS);

        } else {
            view_port_update(view_port);
            furi_delay_ms(100);
        }
    }

    // Final safety frame on exit, if we ever paired - best effort, ignore result.
    uint8_t unused_battery;
    acton_send_drive_frame(ACTON_BOARD_CENTER, &unused_battery);

    nrf24_hal_deinit();
    gui_remove_view_port(gui, view_port);
    furi_record_close(RECORD_GUI);
    view_port_free(view_port);
    furi_mutex_free(state.mutex);

    return 0;
}
