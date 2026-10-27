// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// ce fichier est inclus par mc6809_instructions.cpp (MC6809_INCLUDE_ADDRESSING défini) pour que les
// modes d'adressage soient inclus. Compilé seul (IDE Arduino), il est vide.
// Update Luc Dion 2026-10-26

#ifdef MC6809_INCLUDE_ADDRESSING
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
/*
 * mc6809_addressing_modes.cpp  -  part of MC6809
 *
 * (C)2021-2025 elmerucr
 */

// l'IDE Arduino compile le reste en -Os. En -Os, rd8/wr8, push/pull et les tests de drapeaux n'étaient
// PAS mis en ligne : chaque accès mémoire devenait un appel vers une copie de la fonction
#pragma GCC optimize ("O2", "no-jump-tables")   // pas de tables de sauts en .rodata (flash)
#include "mc6809.hpp"
#include <Arduino.h>

IRAM_ATTR uint16_t mc6809::a_dir(bool *legal)
{
	*legal = true;
	return (dp << 8) | rd8(pc++);
}

IRAM_ATTR uint16_t mc6809::a_ih(bool *legal)
{
	// Inherent, instruction contains all information.
	*legal = true;
	return 0;
}

IRAM_ATTR uint16_t mc6809::a_imb(bool *legal)
{
	*legal = true;
	return pc++;
}

IRAM_ATTR uint16_t mc6809::a_imw(bool *legal)
{
	uint16_t address = pc++;
	pc++;
	*legal = true;
	return address;
}

IRAM_ATTR uint16_t mc6809::a_reb(bool *legal)
{
	// sign extend the 8 bit value
	uint16_t offset = (uint16_t)((int8_t)rd8(pc++));
	*legal = true;
	return (uint16_t)(pc + offset);
}

IRAM_ATTR uint16_t mc6809::a_rew(bool *legal)
{
	uint16_t offset = rd8(pc++);
	offset = (offset << 8) | rd8(pc++);
	*legal = true;
	return pc + offset;
}

// cycles en moins en mode natif 6309, par post-octet (référence HD63B09EP)
static const uint8_t DRAM_ATTR k_idx_nat63[256] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 2, 0, 2, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 2, 0, 2, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 2, 0, 2, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 2, 0, 2, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
};

// sans table de sauts (elle serait en .rodata = flash) : arbre de comparaisons en IRAM
IRAM_ATTR uint16_t mc6809::a_idx(bool *legal)
{
	// First, assume the addressing mode is legal. If not, this flag will be changed in the code below.
	*legal = true;

	uint16_t address = 0;
	uint16_t offset;
	uint8_t byte;
	uint16_t word;

	// read postbyte
	uint8_t postbyte = rd8(pc++);

	// HD6309 en mode natif : certains modes coûtent moins de cycles.
	if (__builtin_expect(nat, 0)) cycles -= k_idx_nat63[postbyte];

	if (postbyte == 0b10011111) {
		// indirect extended
		cycles += 5;

		word = rd8(pc++) << 8;
		word |= rd8(pc++);
		address = rd8(word++) << 8;
		address |= rd8(word);
	} else {
		switch (postbyte & 0b10000000) {
		case 0b00000000:
			// non-indirect, constant 5-bit signed offset
			cycles += 1;

			if (postbyte & 0b00010000) {
				offset = 0xffe0 | (postbyte & 0x1f);
			} else {
				offset = postbyte & 0x0f; // bit 4 always 0
			}
			address = *index_regs[(postbyte & 0b01100000) >> 5]
				+ offset;
			break;
		case 0b10000000:
			// all other modes

			switch (postbyte & 0b00010000) {
			case 0b00000000:
				// non-indirect

				switch (postbyte & 0b00001111) {
				case 0b0100:
					// no offset uses no extra cycles

					address = *index_regs[(postbyte & 0b01100000) >> 5];
					break;
				case 0b1000:
					// 8-bit offset
					cycles += 1;

					byte = rd8(pc++);
					if (byte & 0b10000000) {
						offset = 0xff00 | byte;
					} else {
						offset = byte;
					}
					address = *index_regs[(postbyte & 0b01100000) >> 5]
						+ offset;
					break;
				case 0b1001:
					// 16 bit offset
					cycles += 4;

					offset = rd8(pc++) << 8;
					offset |= rd8(pc++);
					address = *index_regs[(postbyte & 0b01100000) >> 5]
						+ offset;
					break;
				case 0b0110:
					// accumulator a offset
					cycles += 1;

					if (ac & 0b10000000) {
						offset = 0xff00 | ac;
					} else {
						offset = ac;
					}
					address = *index_regs[(postbyte & 0b01100000) >> 5]
						+ offset;
					break;
				case 0b0101:
					// accumulator b offset
					cycles += 1;

					if (br & 0b10000000) {
						offset = 0xff00 | br;
					} else {
						offset = br;
					}
					address = *index_regs[(postbyte & 0b01100000) >> 5]
						+ offset;
					break;
				case 0b1011:
					// accumulator d offset
					cycles += 4;

					offset = (ac << 8) | br;
					address = *index_regs[(postbyte & 0b01100000) >> 5]
						+ offset;
					break;
				case 0b0000:
					// auto increment by 1
					cycles += 2;

					address = *index_regs[(postbyte & 0b01100000) >> 5];
					(*index_regs[(postbyte & 0b01100000) >> 5])++;
					break;
				case 0b0001:
					// auto increment by 2
					cycles += 3;

					address = *index_regs[(postbyte & 0b01100000) >> 5];
					(*index_regs[(postbyte & 0b01100000) >> 5]) += 2;
					break;
				case 0b0010:
					// auto decrement by 1
					cycles += 2;

					(*index_regs[(postbyte & 0b01100000) >> 5])--;
					address = *index_regs[(postbyte & 0b01100000) >> 5];
					break;
				case 0b0011:
					// auto decrement by 2
					cycles += 3;

					(*index_regs[(postbyte & 0b01100000) >> 5]) -= 2;
					address = *index_regs[(postbyte & 0b01100000) >> 5];
					break;
				case 0b1100:
					// constant offset pc 8bit, reads extra byte
					cycles += 1;

					byte = rd8(pc++);
					if (byte & 0b10000000) {
						offset = 0xff00 | byte;
					} else {
						offset = byte;
					}
					address = pc + offset;
					break;
				case 0b1101:
					// constant offset pc 16bit, reads 2 extra bytes
					cycles += 5;

					offset = rd8(pc++) << 8;
					offset |= rd8(pc++);
					address = pc + offset;
					break;
				default:
					// sur 6309, modes W / E / F (sinon illégal)
					if (__builtin_expect(h63, 0)) return a_idx63(postbyte, nullptr);
					*legal = false;
					break;
				}
				break;
			case 0b00010000:
				// indirect

				switch (postbyte & 0b00001111) {
				case 0b0100:
					// indirect, no offset
					cycles += 3;

					word = *index_regs[(postbyte & 0b01100000) >> 5];
					address = rd8(word++) << 8;
					address |= rd8(word);
					break;
				case 0b1000:
					// indirect, 8-bit offset
					cycles += 4;

					byte = rd8(pc++);
					if (byte & 0b10000000) {
						offset = 0xff00 | byte;
					} else {
						offset = byte;
					}
					word = *index_regs[(postbyte & 0b01100000) >> 5]
						+ offset;
					address = rd8(word++) << 8;
					address |= rd8(word);
					break;
				case 0b1001:
					// indirect, 16-bit offset
					cycles += 7;

					offset = rd8(pc++) << 8;
					offset |= rd8(pc++);
					word = *index_regs[(postbyte & 0b01100000) >> 5]
						+ offset;
					address = rd8(word++) << 8;
					address |= rd8(word);
					break;
				case 0b0110:
					// indirect accumulator a offset
					cycles += 4;

					if (ac & 0b10000000) {
						offset = 0xff00 | ac;
					} else {
						offset = ac;
					}
					word = *index_regs[(postbyte & 0b01100000) >> 5]
						+ offset;
					address = rd8(word++) << 8;
					address |= rd8(word);
					break;
				case 0b0101:
					// indirect accumulator b offset
					cycles += 4;

					if (br & 0b10000000) {
						offset = 0xff00 | br;
					} else {
						offset = br;
					}
					word = *index_regs[(postbyte & 0b01100000) >> 5]
						+ offset;
					address = rd8(word++) << 8;
					address |= rd8(word);
					break;
				case 0b1011:
					// accumulator d offset
					cycles += 7;

					offset = (ac << 8) | br;
					word = *index_regs[(postbyte & 0b01100000) >> 5]
						+ offset;
					address = rd8(word++) << 8;
					address |= rd8(word);
					break;
				case 0b0001:
					// indirect auto increment by 2
					cycles += 6;

					word = *index_regs[(postbyte & 0b01100000) >> 5];
					(*index_regs[(postbyte & 0b01100000) >> 5]) += 2;
					address = rd8(word++) << 8;
					address |= rd8(word);
					break;
				case 0b0011:
					// indirect auto decrement by 2
					cycles += 6;

					(*index_regs[(postbyte & 0b01100000) >> 5]) -= 2;
					word = *index_regs[(postbyte & 0b01100000) >> 5];
					address = rd8(word++) << 8;
					address |= rd8(word);
					break;
				case 0b1100:
					// indirect constant offset pc 8bit, reads extra byte
					cycles += 4;

					byte = rd8(pc++);
					if (byte & 0b10000000) {
						offset = 0xff00 | byte;
					} else {
						offset = byte;
					}
					word = pc + offset;
					address = rd8(word++) << 8;
					address |= rd8(word);
					break;
				case 0b1101:
					// indirect constant offset pc 16bit, reads 2 extra bytes
					cycles += 8;

					offset = rd8(pc++) << 8;
					offset |= rd8(pc++);
					word = pc + offset;
					address = rd8(word++) << 8;
					address |= rd8(word);
					break;
				default:
					// sur 6309, modes W / E / F (sinon illégal)
					if (__builtin_expect(h63, 0)) return a_idx63(postbyte, nullptr);
					*legal = false;
					break;
				}
				break;
			}
			break;
		};
	}
	return address;
}

IRAM_ATTR uint16_t mc6809::a_ext(bool *legal)
{
	uint16_t word = (rd8(pc++)) << 8;
	word |= rd8(pc++);
	*legal = true;
	return word;
}

IRAM_ATTR uint16_t mc6809::a_no(bool *legal)
{
	// no mode @ illegal instruction
	*legal = false;
	return 0;
}

#pragma GCC diagnostic pop
#endif // MC6809_INCLUDE_ADDRESSING
