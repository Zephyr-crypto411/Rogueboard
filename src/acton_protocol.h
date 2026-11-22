#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * acton_protocol.h / acton_protocol.c
 *
 * Full pairing + drive protocol for the Acton Blink QU4TRO, 
 *
 * THREE ADDRESSES ARE INVOLVED (correcting an earlier oversimplification):
 *   1. Universal pairing address 0xE3 x5 -- broadcast-style, used only
 *      to find a board that's in pairing mode on SOME channel.
 *   2. Negotiated intermediate address [our_id, 0xFF-our_id, 0xE3,0xE3,0xE3]
 *      -- both sides derive this the same way from a byte WE choose.
 *      This isn't the final driving address, just a narrower channel
 *      to avoid cross-talk with other units during negotiation.
 *   3. Final operating address -- NOT derived by formula. The board
 *      sends back its own 3-byte message; bytes 1 and 2 of that
 *      become the real address used for all actual drive traffic.
 *
 * There is no cryptographic check anywhere in this handshake. "our_id"
 * is simply a byte we pick (0x00-0xFF, avoid 0xE3 and 0xFF to reduce
 * ambiguity with the fixed/complement bytes). The board doesn't
 * authenticate who's claiming it -- it only requires a board that's
 * actually in pairing mode (usually a physical button-press on the
 * board itself) and protocol-correct timing.
 *
 * IMPORTANT HARDWARE NOTE: CE must be held
 * CONTINUOUSLY HIGH while in RX mode (CONFIG bit PRIM_RX set), not
 * pulsed like for TX. nrf24_ce_high()/nrf24_ce_low() handle this;
 * nrf24_pulse_ce() is for TX only. Mixing these up silently breaks
 * receiving.
 *
 * SAFETY: acton_pair() only negotiates an address -- it never sends
 * a throttle value. acton_send_drive_frame() is the only function
 * that can make the board move, and it always sends a real throttle
 * byte you provide -- there is no "neutral-only" no-op mode baked in
 * here. The CALLER (app.c) is responsible for arming/disarming,
 * ramping, and defaulting to neutral -- see its comments. Test only
 * with wheels off the ground.
 */

typedef enum {
    ActonPairInProgress,
    ActonPairSuccess,
    ActonPairFailed_NoBoardFound,   // exhausted all channels at stage 1
    ActonPairFailed_NegotiationLost, // ACKed stage 1 but lost at stage 2
    ActonPairFailed_Timeout,         // got to RX wait but board never replied
} ActonPairResult;

typedef struct {
    uint8_t operating_address[5]; // valid only if result == ActonPairSuccess
    uint8_t channel_mhz_offset;    // which channel (0-15 index into Acton's table) it paired on
} ActonPairOutcome;

/**
 * Runs the full 3-stage pairing handshake against a board that is
 * currently in pairing mode (user must put the physical board into
 * pairing mode first -- this code cannot do that remotely).
 *
 * @param channel_table     16-entry channel table (RF_CH values, 0-83).
 *                          Acton's real table lives in Hardware.cpp,
 *                          which we don't have -- pass your best guess
 *                          or a captured one. Using the wrong table
 *                          just means pairing will fail/timeout, not
 *                          anything unsafe.
 * @param our_identity_byte byte we claim as our address identity.
 *                          Avoid 0xE3 (collides with the fixed bytes)
 *                          and 0xFF (collides with its own complement).
 * @param out               filled with the result and, on success,
 *                          the operating address to use for driving.
 * @return the same value as out->result would show, for convenience
 */
ActonPairResult acton_pair(const uint8_t channel_table[16], uint8_t our_identity_byte, ActonPairOutcome* out);

/**
 * Configures the radio for drive mode using a previously paired
 * operating_address (either from acton_pair(), or one you captured/
 * already knew). Call once before the first acton_send_drive_frame().
 */
void acton_drive_init(const uint8_t operating_address[5], const uint8_t channel_table[16], uint8_t starting_channel_index);

/**
 * Sends one drive frame and waits briefly for the board's ACK reply
 * (which carries its battery state).
 *
 * @param throttle_byte  0-255. From QuatroRemote.ino: boardMin=32 is
 *                       max acceleration, boardCenter=128 is neutral,
 *                       boardMax=230 is max brake. CLAMP before
 *                       calling this -- this function does not clamp
 *                       for you.
 * @param out_board_battery_state  filled with the board's reported
 *                       battery nibble if the ACK was received.
 * @return true if the board ACKed (connection alive), false if it
 *         appears lost (caller should stop sending full-range values
 *         and/or re-pair).
 *
 * NOTE: the original alternates between two message buffers
 * (quatroMessage_1 / quatroMessage_2) where message_2's exact extra
 * content comes from Modes.cpp, which we don't have. This port always
 * sends a single message style carrying the throttle byte in position
 * [1] and zeros elsewhere. Basic throttle control should still work
 * since [1] is the byte the board actually reads for torque, but gear/
 * mode-switching behavior that may live in message_2 is NOT replicated.
 */
bool acton_send_drive_frame(uint8_t throttle_byte, uint8_t* out_board_battery_state);
