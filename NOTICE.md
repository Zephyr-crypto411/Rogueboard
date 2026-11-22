Notice

RogueBoard incorporates protocol knowledge and, in places, ported logic from other open-source reverse-engineering projects. This file credits those sources and notes the licensing situation for each, as best as could be determined at the time of writing.

Before publishing this repo, re-check both source repos' current license files yourself. Licenses and repo states can change, and the notes below reflect what was found during development, not a legal review.

Acton Blink QU4TRO driver — McNugget6750/Open-esk8-Remote
Source: https://github.com/McNugget6750/Open-esk8-Remote
Original author: Timo Birnschein (timo.birnschein@microforge.de)
License (per file headers): LGPL

acton_protocol.c and acton_protocol.h in this repo are a port of the pairing handshake and drive-frame logic originally implemented in that project's Pairing_StateMachine.cpp, RF_Comm.h/.cpp, Hardware.h/.cpp, and QuatroRemote.ino. The radio register sequences, the three-stage pairing handshake (universal pairing address, negotiated intermediate address, board-assigned final address), the confirmed 16-channel hop table, and the throttle byte range (32/128/230) are all derived directly from that source.

Because the original is LGPL-licensed, this repo should make clear which files are a derivative of it (the two listed above) and should not relicense those files under stricter terms than LGPL permits. Confirm the current LICENSE file in that repo before publishing, and consider linking to it directly from the affected files' headers.

Exway promiscuous sniffer — sieveandsand/Hacking_X1_Pro
Source: https://github.com/sieveandsand/Hacking_X1_Pro (project_sniffer/ — sniffer.h, sniffer.cpp, sniffer_config.h)
License: at the time this project was developed, the repo-level license and the LGPL text in sniffer.cpp's own file header did not match — the repo appeared to carry a CC BY-NC-SA (non-commercial) license at the top level. This was not resolved before publishing this notice.

promiscuous_scan.c and promiscuous_scan.h in this repo are a port of the magic-ID promiscuous discovery technique from that project: the illegal/reserved SETUP_AW encoding used to force raw promiscuous capture, the two-byte manufacturer magic ID search (0x39 0x32 for Exway), the candidate-address harvesting and byte-reversal, and the gear-value decode used to validate a candidate address.

Some protocol patterns observed here (e.g. a neutral-centered throttle byte with opposite-direction extremes, and an address-handoff scheme using a byte and its 0xFF-complement during pairing negotiation) were also independently described in a public write-up about a different, unrelated board (an Acton-family board documented on Hackaday — see project history/commit messages for the exact reference). No code was copied from that source; it's noted here only because it corroborated patterns also found in the repos above.

Everything else

nrf24_hal.c/.h, board_profiles.h, and rogueboard_sniffer_app.c are original to this project, written against the public nRF24L01+ datasheet and the Flipper Zero furi_hal API, not ported from either source repo above.
