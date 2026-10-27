// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

/*
 * mc6809_opcodes.cpp  -  part of MC6809
 *
 * (C)2021-2025 elmerucr
 */

// Update Luc Dion 2026-10-26

 #include <Arduino.h>
// l'IDE Arduino compile le reste en -Os. En -Os, rd8/wr8, push/pull et les tests de drapeaux n'étaient
// PAS mis en ligne : chaque accès mémoire devenait un appel vers une copie de la fonction
#pragma GCC optimize ("O2", "no-jump-tables")   // pas de tables de sauts en .rodata (flash)
#pragma GCC diagnostic ignored "-Wattributes"   // always_inline voulu sur des fonctions non « inline »
 #include "mc6809.hpp"
#ifndef DRAM_ATTR
#define DRAM_ATTR
#endif
// modes d'adressage dans la même unité de compilation (mise en ligne)
#define MC6809_INCLUDE_ADDRESSING
#include "mc6809_addressing_modes.cpp"
#include <cstdio>

// tables de la page 1 sorties de l'objet CPU (4 Ko de RAM interne rendus)
const mc6809::execute_instruction mc6809::opcodes_page1[256] = {
		&mc6809::neg,	&mc6809::neg,	&mc6809::negcom,	&mc6809::com,	&mc6809::lsr,	&mc6809::lsr,	&mc6809::ror,	&mc6809::asr,	// 0x00
		&mc6809::asl,	&mc6809::rol,	&mc6809::dec,	&mc6809::dec,	&mc6809::inc,	&mc6809::tst,	&mc6809::jmp,	&mc6809::clr,
		&mc6809::page2,	&mc6809::page3,	&mc6809::nop,	&mc6809::sync,	&mc6809::ill,	&mc6809::ill,	&mc6809::lbra,	&mc6809::lbsr,	// 0x10
		&mc6809::nop,	&mc6809::daa,	&mc6809::orcc,	&mc6809::nop,	&mc6809::andcc,	&mc6809::sex,	&mc6809::exg,	&mc6809::tfr,
		&mc6809::bra,	&mc6809::brn,	&mc6809::bhi,	&mc6809::bls,	&mc6809::bhs,	&mc6809::blo,	&mc6809::bne,	&mc6809::beq,	// 0x20
		&mc6809::bvc,	&mc6809::bvs,	&mc6809::bpl,	&mc6809::bmi,	&mc6809::bge,	&mc6809::blt,	&mc6809::bgt,	&mc6809::ble,
		&mc6809::leax,	&mc6809::leay,	&mc6809::leas,	&mc6809::leau,	&mc6809::pshs,	&mc6809::puls,	&mc6809::pshu,	&mc6809::pulu,	// 0x30
		&mc6809::andcc,	&mc6809::rts,	&mc6809::abx,	&mc6809::rti,	&mc6809::cwai,	&mc6809::mul,	&mc6809::xres,	&mc6809::swi,
		&mc6809::nega,	&mc6809::nega,	&mc6809::negcoma,	&mc6809::coma,	&mc6809::lsra,	&mc6809::lsra,	&mc6809::rora,	&mc6809::asra,	// 0x40
		&mc6809::asla,	&mc6809::rola,	&mc6809::deca,	&mc6809::deca,	&mc6809::inca,	&mc6809::tsta,	&mc6809::clra,	&mc6809::clra,
		&mc6809::negb,	&mc6809::negb,	&mc6809::negcomb,	&mc6809::comb,	&mc6809::lsrb,	&mc6809::lsrb,	&mc6809::rorb,	&mc6809::asrb,	// 0x50
		&mc6809::aslb,	&mc6809::rolb,	&mc6809::decb,	&mc6809::decb,	&mc6809::incb,	&mc6809::tstb,	&mc6809::clrb,	&mc6809::clrb,
		&mc6809::neg,	&mc6809::neg,	&mc6809::negcom,	&mc6809::com,	&mc6809::lsr,	&mc6809::lsr,	&mc6809::ror,	&mc6809::asr,	// 0x60
		&mc6809::asl,	&mc6809::rol,	&mc6809::dec,	&mc6809::dec,	&mc6809::inc,	&mc6809::tst,	&mc6809::jmp,	&mc6809::clr,
		&mc6809::neg,	&mc6809::neg,	&mc6809::negcom,	&mc6809::com,	&mc6809::lsr,	&mc6809::lsr,	&mc6809::ror,	&mc6809::asr,	// 0x70
		&mc6809::asl,	&mc6809::rol,	&mc6809::dec,	&mc6809::dec,	&mc6809::inc,	&mc6809::tst,	&mc6809::jmp,	&mc6809::clr,
		&mc6809::suba,	&mc6809::cmpa,	&mc6809::sbca,	&mc6809::subd,	&mc6809::anda,	&mc6809::bita,	&mc6809::lda,	&mc6809::st_imm_a,	// 0x80
		&mc6809::eora,	&mc6809::adca,	&mc6809::ora,	&mc6809::adda,	&mc6809::cmpx,	&mc6809::bsr,	&mc6809::ldx,	&mc6809::st_imm_x,
		&mc6809::suba,	&mc6809::cmpa,	&mc6809::sbca,	&mc6809::subd,	&mc6809::anda,	&mc6809::bita,	&mc6809::lda,	&mc6809::sta,	// 0x90
		&mc6809::eora,	&mc6809::adca,	&mc6809::ora,	&mc6809::adda,	&mc6809::cmpx,	&mc6809::jsr,	&mc6809::ldx,	&mc6809::stx,
		&mc6809::suba,	&mc6809::cmpa,	&mc6809::sbca,	&mc6809::subd,	&mc6809::anda,	&mc6809::bita,	&mc6809::lda,	&mc6809::sta,	// 0xa0
		&mc6809::eora,	&mc6809::adca,	&mc6809::ora,	&mc6809::adda,	&mc6809::cmpx,	&mc6809::jsr,	&mc6809::ldx,	&mc6809::stx,
		&mc6809::suba,	&mc6809::cmpa,	&mc6809::sbca,	&mc6809::subd,	&mc6809::anda,	&mc6809::bita,	&mc6809::lda,	&mc6809::sta,	// 0xb0
		&mc6809::eora,	&mc6809::adca,	&mc6809::ora,	&mc6809::adda,	&mc6809::cmpx,	&mc6809::jsr,	&mc6809::ldx,	&mc6809::stx,
		&mc6809::subb,	&mc6809::cmpb,	&mc6809::sbcb,	&mc6809::addd,	&mc6809::andb,	&mc6809::bitb,	&mc6809::ldb,	&mc6809::st_imm_b,	// 0xc0
		&mc6809::eorb,	&mc6809::adcb,	&mc6809::orb,	&mc6809::addb,	&mc6809::ldd,	&mc6809::ill,	&mc6809::ldu,	&mc6809::st_imm_u,
		&mc6809::subb,	&mc6809::cmpb,	&mc6809::sbcb,	&mc6809::addd,	&mc6809::andb,	&mc6809::bitb,	&mc6809::ldb,	&mc6809::stb,	// 0xd0
		&mc6809::eorb,	&mc6809::adcb,	&mc6809::orb,	&mc6809::addb,	&mc6809::ldd,	&mc6809::std,	&mc6809::ldu,	&mc6809::stu,
		&mc6809::subb,	&mc6809::cmpb,	&mc6809::sbcb,	&mc6809::addd,	&mc6809::andb,	&mc6809::bitb,	&mc6809::ldb,	&mc6809::stb,	// 0xe0
		&mc6809::eorb,	&mc6809::adcb,	&mc6809::orb,	&mc6809::addb,	&mc6809::ldd,	&mc6809::std,	&mc6809::ldu,	&mc6809::stu,
		&mc6809::subb,	&mc6809::cmpb,	&mc6809::sbcb,	&mc6809::addd,	&mc6809::andb,	&mc6809::bitb,	&mc6809::ldb,	&mc6809::stb,	// 0xf0
		&mc6809::eorb,	&mc6809::adcb,	&mc6809::orb,	&mc6809::addb,	&mc6809::ldd,	&mc6809::std,	&mc6809::ldu,	&mc6809::stu
	};

const mc6809::addressing_mode mc6809::addressing_modes_page1[256] = {
		&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	// 0x00
		&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,
		&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_rew,	&mc6809::a_rew,	// 0x10
		&mc6809::a_no,	&mc6809::a_ih,	&mc6809::a_imb,	&mc6809::a_no,	&mc6809::a_imb,	&mc6809::a_ih,	&mc6809::a_imb,	&mc6809::a_imb,
		&mc6809::a_reb,	&mc6809::a_reb,	&mc6809::a_reb,	&mc6809::a_reb,	&mc6809::a_reb,	&mc6809::a_reb,	&mc6809::a_reb,	&mc6809::a_reb,	// 0x20
		&mc6809::a_reb,	&mc6809::a_reb,	&mc6809::a_reb,	&mc6809::a_reb,	&mc6809::a_reb,	&mc6809::a_reb,	&mc6809::a_reb,	&mc6809::a_reb,
		&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imb,	// 0x30
		&mc6809::a_imb,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_imb,	&mc6809::a_ih,	&mc6809::a_no,	&mc6809::a_ih,	// 0x3C CWAI = immédiat (était a_ih)
		&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	// 0x40
		&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,
		&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	// 0x50
		&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,	&mc6809::a_ih,
		&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	// 0x60
		&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,
		&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	// 0x70
		&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,
		&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imw,	&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imb,	// 0x80
		&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imw,	&mc6809::a_reb,	&mc6809::a_imw,	&mc6809::a_imw,
		&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	// 0x90
		&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,
		&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	// 0xa0
		&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,
		&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	// 0xb0
		&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,
		&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imw,	&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imb,	// 0xc0
		&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imb,	&mc6809::a_imw,	&mc6809::a_no,	&mc6809::a_imw,	&mc6809::a_imw,
		&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	// 0xd0
		&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,	&mc6809::a_dir,
		&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	// 0xe0
		&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,	&mc6809::a_idx,
		&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	// 0xf0
		&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext,	&mc6809::a_ext
	};

uint32_t (*mc6809_nop_budget)(void) = nullptr;   // voir L12 dans execute
uint8_t  Byte;
uint16_t Word;
uint32_t Dword;
extern bool CPU_in_WAIT_STATE;
// d_reg is a stand-in temporary variable to ease calculations during individual instructions that deal
// with the d register
uint16_t d_reg;

IRAM_ATTR void mc6809::ill(uint16_t ea)
{
	// Restent sans effet : $14/$15/$CD (HCF : bloquent le vrai 6809).
}

// ---------------------------------------------------------------------------
// opcodes non documentés du 6809 réel (comportement de MAME m6809, BSD-3). Des logiciels s'en servent
// : détection 6309 (moteur AGD $10 $86 = LDA # sur 6809, puis TFR W,X qui lit $FFFF), protections.
// ---------------------------------------------------------------------------
IRAM_ATTR void mc6809::negcom(uint16_t ea)  { if (is_c_flag_set()) com(ea);  else neg(ea);  }
IRAM_ATTR void mc6809::negcoma(uint16_t ea) { if (is_c_flag_set()) coma(ea); else nega(ea); }
IRAM_ATTR void mc6809::negcomb(uint16_t ea) { if (is_c_flag_set()) comb(ea); else negb(ea); }
IRAM_ATTR void mc6809::st_imm_a(uint16_t ea) { clear_v_flag(); test_nz_flags(ac); }
IRAM_ATTR void mc6809::st_imm_b(uint16_t ea) { clear_v_flag(); test_nz_flags(br); }
IRAM_ATTR void mc6809::st_imm_x(uint16_t ea)
{
	clear_v_flag();
	if (xr & 0x8000) set_n_flag(); else clear_n_flag();
	if (xr == 0) set_z_flag(); else clear_z_flag();
}
IRAM_ATTR void mc6809::st_imm_u(uint16_t ea)
{
	clear_v_flag();
	if (us & 0x8000) set_n_flag(); else clear_n_flag();
	if (us == 0) set_z_flag(); else clear_z_flag();
}
IRAM_ATTR void mc6809::xres(uint16_t ea)
{
	set_e_flag();
	push_all();	// + F, E en mode natif 6309
	set_i_flag();
	set_f_flag();
	pc = (rd8(VECTOR_RESET)) << 8;
	pc |= rd8(VECTOR_RESET+1);
}

// TFR/EXG : registre non défini = $FF (8 bits) / $FFFF (16 bits) ; 8 -> 16 bits = $FF:valeur ; 16 -> 8
// bits = octet bas (comme MAME).
IRAM_ATTR uint16_t mc6809::read_tfr_reg(uint8_t r)
{
	if (__builtin_expect(h63, 0)) return read_tfr63(r);
	switch (r & 0x0f) {
		case 0x0: return (ac << 8) | br;
		case 0x1: return xr;
		case 0x2: return yr;
		case 0x3: return us;
		case 0x4: return sp;
		case 0x5: return pc;
		case 0x8: return 0xff00 | ac;
		case 0x9: return 0xff00 | br;
		case 0xa: return 0xff00 | cc;
		case 0xb: return 0xff00 | dp;
		default:  return 0xffff;
	}
}

IRAM_ATTR void mc6809::write_tfr_reg(uint8_t r, uint16_t v)
{
	if (__builtin_expect(h63, 0)) { write_tfr63(r, v); return; }
	switch (r & 0x0f) {
		case 0x0: ac = v >> 8; br = v & 0xff; break;
		case 0x1: xr = v; break;
		case 0x2: yr = v; break;
		case 0x3: us = v; break;
		case 0x4: sp = v; nmi_enabled = true; break;
		case 0x5: pc = v; break;
		case 0x8: ac = v & 0xff; break;
		case 0x9: br = v & 0xff; break;
		case 0xa: cc = v & 0xff; break;
		case 0xb: dp = v & 0xff; break;
		default: break;
	}
}

IRAM_ATTR void mc6809::abx(uint16_t ea)
{
	xr += br;
}

IRAM_ATTR void mc6809::adca(uint16_t ea)
{
	// See: Osborne, A. 1976. An introduction to microcomputers Volume I Basic Concepts. SYBEX. pages 4-12
	// to 4-16.

	uint8_t old_carry = (is_c_flag_set() ? 1 : 0);

	Byte = rd8(ea);

	// Half carry
	if (((ac & 0x0f) + (Byte & 0x0f) + old_carry) & 0b00010000) {
		set_h_flag();
	} else {
		clear_h_flag();
	}

	bool bit_7_carry_in = (((ac & 0x7f) + (Byte & 0x7f) + old_carry) & 0x80) ? true : false;

	Word = ac + Byte + old_carry;
	ac = Word & 0x00ff;

	bool carry = (Word & 0x0100) ? true : false;

	if (carry != bit_7_carry_in) set_v_flag(); else clear_v_flag();
	if (carry) set_c_flag(); else clear_c_flag();
	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::adcb(uint16_t ea)
{
	uint8_t old_carry = (is_c_flag_set() ? 1 : 0);

	Byte = rd8(ea);

	// Half carry
	if (((br & 0x0f) + (Byte & 0x0f) + old_carry) & 0b00010000) {
		set_h_flag();
	} else {
		clear_h_flag();
	}

	bool bit_7_carry_in = (((br & 0x7f) + (Byte & 0x7f) + old_carry) & 0x80) ? true : false;

	Word = br + Byte + old_carry;
	br = Word & 0x00ff;

	bool carry = (Word & 0x0100) ? true : false;

	if (carry != bit_7_carry_in) set_v_flag(); else clear_v_flag();
	if (carry) set_c_flag(); else clear_c_flag();
	test_nz_flags(br);
}

IRAM_ATTR void mc6809::adda(uint16_t ea)
{
	Byte = rd8(ea);

	// Half carry
	if (((ac & 0x0f) + (Byte & 0x0f)) & 0b00010000) {
		set_h_flag();
	} else {
		clear_h_flag();
	}

	bool bit_7_carry_in = (((ac & 0x7f) + (Byte & 0x7f)) & 0x80) ? true : false;

	Word = ac + Byte;
	ac = Word & 0x00ff;

	bool carry = (Word & 0x0100) ? true : false;

	if (carry != bit_7_carry_in) set_v_flag(); else clear_v_flag();
	if (carry) set_c_flag(); else clear_c_flag();
	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::addb(uint16_t ea)
{
	Byte = rd8(ea);

	// Half carry
	if (((br & 0x0f) + (Byte & 0x0f)) & 0b00010000) {
		set_h_flag();
	} else {
		clear_h_flag();
	}

	bool bit_7_carry_in = (((br & 0x7f) + (Byte & 0x7f)) & 0x80) ? true : false;

	Word = br + Byte;
	br = Word & 0x00ff;

	bool carry = (Word & 0x0100) ? true : false;

	if (carry != bit_7_carry_in) set_v_flag(); else clear_v_flag();
	if (carry) set_c_flag(); else clear_c_flag();
	test_nz_flags(br);
}

IRAM_ATTR void mc6809::addd(uint16_t ea)
{
	Word = (rd8(ea++)) << 8;
	Word |= rd8(ea);

	d_reg = (ac << 8) | br;

	// no need for half-carry here

	bool bit_15_carry_in = (((d_reg & 0x7fff) + (Word & 0x7fff)) & 0x8000) ? true : false;

	Dword = d_reg + Word;
	d_reg = Dword & 0xffff;
	ac = (d_reg & 0xff00) >> 8;
	br = d_reg & 0xff;

	bool carry = (Dword & 0x00010000) ? true : false;

	if(carry != bit_15_carry_in) set_v_flag(); else clear_v_flag();
	if(carry) set_c_flag(); else clear_c_flag();
	test_nz_flags_16(d_reg);
}

IRAM_ATTR void mc6809::anda(uint16_t ea)
{
	Byte = ac & rd8(ea);
	clear_v_flag();
	test_nz_flags(Byte);
	ac = Byte;
}

IRAM_ATTR void mc6809::andb(uint16_t ea)
{
	Byte = br & rd8(ea);
	clear_v_flag();
	test_nz_flags(Byte);
	br = Byte;
}

IRAM_ATTR void mc6809::andcc(uint16_t ea)
{
	cc &= rd8(ea);
}

IRAM_ATTR void mc6809::asl(uint16_t ea)
{
	Byte = rd8(ea);

	if (Byte & 0x80) set_c_flag(); else clear_c_flag();
	if (((Byte & 0xc0) == 0x80) || ((Byte & 0xc0) == 0x40))
		set_v_flag(); else clear_v_flag();

	Byte <<= 1;

	test_nz_flags(Byte);
	wr8(ea, Byte);
}

IRAM_ATTR void mc6809::asla(uint16_t ea)
{
	if (ac & 0x80) set_c_flag(); else clear_c_flag();
	if (((ac & 0xc0) == 0x80) || ((ac & 0xc0) == 0x40))
		set_v_flag(); else clear_v_flag();

	ac <<= 1;

	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::aslb(uint16_t ea)
{
	if (br & 0x80) set_c_flag(); else clear_c_flag();
	if (((br & 0xc0) == 0x80) || ((br & 0xc0) == 0x40))
		set_v_flag(); else clear_v_flag();

	br <<= 1;

	test_nz_flags(br);
}

IRAM_ATTR void mc6809::asr(uint16_t ea)
{
	Byte = rd8(ea);

	if (Byte & 0x01) set_c_flag(); else clear_c_flag();
	bool bit7 = (Byte & 0x80) ? true : false;

	Byte >>= 1;
	if (bit7) Byte |= 0x80; else Byte &= 0x7f;

	test_nz_flags(Byte);
	wr8(ea, Byte);
}

IRAM_ATTR void mc6809::asra(uint16_t ea)
{
	if (ac & 0x01) set_c_flag(); else clear_c_flag();
	bool bit7 = (ac & 0x80) ? true : false;

	ac >>= 1;
	if (bit7) ac |= 0x80; else ac &= 0x7f;

	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::asrb(uint16_t ea)
{
	if (br & 0x01) set_c_flag(); else clear_c_flag();
	bool bit7 = (br & 0x80) ? true : false;

	br >>= 1;
	if (bit7) br |= 0x80; else br &= 0x7f;

	test_nz_flags(br);
}

IRAM_ATTR void mc6809::beq(uint16_t ea)
{
	if (is_z_flag_set()) pc = ea;
}

IRAM_ATTR void mc6809::bge(uint16_t ea)
{
	// both n and v set OR both n and v clear
	if ((is_n_flag_set() && is_v_flag_set()) || (is_n_flag_clear() && is_v_flag_clear())) {
		pc = ea;
	}
}

IRAM_ATTR void mc6809::bgt(uint16_t ea)
{
	// (both n and v set OR both n and v clear) AND (z clear)
	if (((is_n_flag_set() && is_v_flag_set()) || (is_n_flag_clear() && is_v_flag_clear())) && is_z_flag_clear()) {
		pc = ea;
	}
}

IRAM_ATTR void mc6809::bhi(uint16_t ea)
{
	if (is_z_flag_clear() && is_c_flag_clear()) {
		pc = ea;
	}
}

// bhs - Branch if Higher or Same bcc - Branch if Carry Clear E.g. if no borrow was needed (carry
// clear) after a comparison, then value in register must be higher than or the same as the compared
// value.
IRAM_ATTR void mc6809::bhs(uint16_t ea)
{
	if (is_c_flag_clear()) {
		pc = ea;
	}
}

IRAM_ATTR void mc6809::bita(uint16_t ea)
{
	Byte = ac & rd8(ea);
	clear_v_flag();
	test_nz_flags(Byte);
}

IRAM_ATTR void mc6809::bitb(uint16_t ea)
{
	Byte = br & rd8(ea);
	clear_v_flag();
	test_nz_flags(Byte);
}

IRAM_ATTR void mc6809::ble(uint16_t ea)
{
	if (is_z_flag_set() || (is_n_flag_set() && is_v_flag_clear()) || (is_n_flag_clear() && is_v_flag_set())) {
		pc = ea;
	}
}

// blo - Branch if Lower bcs - Branch if Carry Set E.g. if a borrow was needed (carry set) after a
// comparison, the value in the register must be lower than the compared value.
IRAM_ATTR void mc6809::blo(uint16_t ea)
{
	if (is_c_flag_set()) {
		pc = ea;
	}
}

// bls - Branch if Lower or Same
IRAM_ATTR void mc6809::bls(uint16_t ea)
{
	if (is_c_flag_set() || is_z_flag_set()) {
		pc = ea;
	}
}

IRAM_ATTR void mc6809::blt(uint16_t ea)
{
	if ((is_n_flag_set() && is_v_flag_clear()) || (is_n_flag_clear() && is_v_flag_set())) {
		pc = ea;
	}
}

IRAM_ATTR void mc6809::bmi(uint16_t ea)
{
	if (is_n_flag_set()) {
		pc = ea;
	}
}

IRAM_ATTR void mc6809::bne(uint16_t ea)
{
	if (is_z_flag_clear()) {
		pc = ea;
	}
}

IRAM_ATTR void mc6809::bpl(uint16_t ea)
{
	if (is_n_flag_clear()) {
		pc = ea;
	}
}

IRAM_ATTR void mc6809::bra(uint16_t ea)
{
	pc = ea;
}

IRAM_ATTR void mc6809::brn(uint16_t ea)
{
	// does essentially nothing
}

IRAM_ATTR void mc6809::bsr(uint16_t ea)
{
	push_sp(pc & 0x00ff);
	push_sp((pc & 0xff00) >> 8);
	pc = ea;
}

IRAM_ATTR void mc6809::bvc(uint16_t ea)
{
	if (is_v_flag_clear()) {
		pc = ea;
	}
}

IRAM_ATTR void mc6809::bvs(uint16_t ea)
{
	if (is_v_flag_set()) {
		pc = ea;
	}
}

IRAM_ATTR void mc6809::clr(uint16_t ea)
{
	wr8(ea, 0x00);
	clear_n_flag();
	set_z_flag();
	clear_v_flag();
	clear_c_flag();
}

IRAM_ATTR void mc6809::clra(uint16_t ea)
{
	ac = 0x00;
	clear_n_flag();
	set_z_flag();
	clear_v_flag();
	clear_c_flag();
}

IRAM_ATTR void mc6809::clrb(uint16_t ea)
{
	br = 0x00;
	clear_n_flag();
	set_z_flag();
	clear_v_flag();
	clear_c_flag();
}

IRAM_ATTR void mc6809::cmpa(uint16_t ea)
{
	// code inspired by virtualc64
	Byte = rd8(ea);
	Word = ac - Byte;

	if (Word > 255) set_c_flag(); else clear_c_flag();

	if (((ac ^ Word) & 0x80) && ((ac ^ Byte) & 0x80)) set_v_flag(); else clear_v_flag();

	Byte = Word & 0xff;
	test_nz_flags(Byte);
}

IRAM_ATTR void mc6809::cmpb(uint16_t ea)
{
	// code inspired by virtualc64
	Byte = rd8(ea);
	Word = br - Byte;

	if (Word > 255) set_c_flag(); else clear_c_flag();

	if (((br ^ Word) & 0x80) && ((br ^ Byte) & 0x80)) set_v_flag(); else clear_v_flag();

	Byte = Word & 0xff;
	test_nz_flags(Byte);
}

IRAM_ATTR void mc6809::cmpd(uint16_t ea)
{
	// code inspired by virtualc64
	Word = rd8(ea++) << 8;
	Word |= rd8((uint16_t)ea);
	d_reg = (ac << 8) | br;
	Dword = d_reg - Word;

	if (Dword > 65535) set_c_flag(); else clear_c_flag();

	if (((d_reg ^ Dword) & 0x8000) && ((d_reg ^ Word) & 0x8000)) set_v_flag(); else clear_v_flag();

	Word = Dword & 0xffff;
	test_nz_flags_16(Word);
}

IRAM_ATTR void mc6809::cmpu(uint16_t ea)
{
	// code inspired by virtualc64
	Word = rd8(ea++) << 8;
	Word |= rd8((uint16_t)ea);
	Dword = us - Word;

	if (Dword > 65535) set_c_flag(); else clear_c_flag();

	if (((us ^ Dword) & 0x8000) && ((us ^ Word) & 0x8000)) set_v_flag(); else clear_v_flag();

	Word = Dword & 0xffff;
	test_nz_flags_16(Word);
}

IRAM_ATTR void mc6809::cmps(uint16_t ea)
{
	// code inspired by virtualc64
	Word = rd8(ea++) << 8;
	Word |= rd8((uint16_t)ea);
	Dword = sp - Word;

	if (Dword > 65535) set_c_flag(); else clear_c_flag();

	if (((sp ^ Dword) & 0x8000) && ((sp ^ Word) & 0x8000)) set_v_flag(); else clear_v_flag();

	Word = Dword & 0xffff;
	test_nz_flags_16(Word);
}

IRAM_ATTR void mc6809::cmpx(uint16_t ea)
{
	// code inspired by virtualc64
	Word = rd8(ea++) << 8;
	Word |= rd8((uint16_t)ea);
	Dword = xr - Word;

	if (Dword > 65535) set_c_flag(); else clear_c_flag();

	if (((xr ^ Dword) & 0x8000) && ((xr ^ Word) & 0x8000)) set_v_flag(); else clear_v_flag();

	Word = Dword & 0xffff;
	test_nz_flags_16(Word);
}

IRAM_ATTR void mc6809::cmpy(uint16_t ea)
{
	// code inspired by virtualc64
	Word = rd8(ea++) << 8;
	Word |= rd8((uint16_t)ea);
	Dword = yr - Word;

	if (Dword > 65535) set_c_flag(); else clear_c_flag();

	if (((yr ^ Dword) & 0x8000) && ((yr ^ Word) & 0x8000)) set_v_flag(); else clear_v_flag();

	Word = Dword & 0xffff;
	test_nz_flags_16(Word);
}

IRAM_ATTR void mc6809::com(uint16_t ea)
{
	Byte = rd8(ea);
	Byte = ~Byte;
	wr8(ea, Byte);
	test_nz_flags(Byte);
	clear_v_flag();
	set_c_flag();
}

IRAM_ATTR void mc6809::coma(uint16_t ea)
{
	ac = ~ac;
	test_nz_flags(ac);
	clear_v_flag();
	set_c_flag();
}

IRAM_ATTR void mc6809::comb(uint16_t ea)
{
	br = ~br;
	test_nz_flags(br);
	clear_v_flag();
	set_c_flag();
}

IRAM_ATTR void mc6809::cwai(uint16_t ea)
{
	// CWAI complet (fiche technique MC6809) : CC &= immédiat E = 1, empilement de TOUT l'état, puis
	// attente d'une interruption AUTORISÉE ; celle-ci ne réempile pas (cwai_stacked, voir execute)
	cc &= rd8(ea);
	set_e_flag();
	push_all();	// + F, E en mode natif 6309
	cpu_state = CPU_CWAI;
}

IRAM_ATTR void mc6809::daa(uint16_t ea)
{
	if (is_h_flag_set() || ((ac & 0x0f) > 9))
		Byte = 0x06; else Byte = 0;
	if (is_c_flag_set() || (((ac & 0xf0) >> 4) > 9) ||
		((((ac & 0xf0) >> 4) > 8) && ((ac & 0x0f) > 9)))
		Byte |= 0x60;
	Word = ac + Byte;
	ac = Word & 0xff;
	// C reste à 1 s'il l'était (fiche MC6809 : « set if a carry is generated or if the carry bit was set
	// before »), V = 0 (comme MAME).
	if (Word & 0x0100) set_c_flag();
	clear_v_flag();
	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::dec(uint16_t ea)
{
	Byte = rd8(ea);

	bool bit_7_carry_in = (((Byte & 0x7f) + 0x7f) & 0x80) ? true : false;

	Word = Byte + 0xff;
	Byte = Word & 0x00ff;

	bool carry = (Word & 0x0100) ? true : false;

	if (carry != bit_7_carry_in) set_v_flag(); else clear_v_flag();
	test_nz_flags(Byte);

	wr8(ea, Byte);
}

IRAM_ATTR void mc6809::deca(uint16_t ea)
{
	bool bit_7_carry_in = (((ac & 0x7f) + 0x7f) & 0x80) ? true : false;

	Word = ac + 0xff;		// do an addition
	ac = Word & 0x00ff;

	bool carry = (Word & 0x0100) ? true : false;

	if (carry != bit_7_carry_in) set_v_flag(); else clear_v_flag();
	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::decb(uint16_t ea)
{
	bool bit_7_carry_in = (((br & 0x7f) + 0x7f) & 0x80) ? true : false;

	Word = br + 0xff;		// do an addition
	br = Word & 0x00ff;

	bool carry = (Word & 0x0100) ? true : false;

	if (carry != bit_7_carry_in) set_v_flag(); else clear_v_flag();
	test_nz_flags(br);
}

IRAM_ATTR void mc6809::eora(uint16_t ea)
{
	ac ^= rd8(ea);
	clear_v_flag();
	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::eorb(uint16_t ea)
{
	br ^= rd8(ea);
	clear_v_flag();
	test_nz_flags(br);
}

IRAM_ATTR void mc6809::exg(uint16_t ea)
{
	// toutes les combinaisons, comme le vrai 6809 (MAME)
	const uint8_t post = rd8(ea);
	const uint16_t a = read_tfr_reg(post >> 4);
	const uint16_t b = read_tfr_reg(post & 0x0f);
	write_tfr_reg(post >> 4, b);
	write_tfr_reg(post & 0x0f, a);
}

IRAM_ATTR void mc6809::inc(uint16_t ea)
{
	Byte = rd8(ea);

	bool bit_7_carry_in = (((Byte & 0x7f) + 0x01) & 0x80) ? true : false;

	Word = Byte + 0x01;
	Byte = Word & 0x00ff;

	bool carry = (Word & 0x0100) ? true : false;

	if (carry != bit_7_carry_in) set_v_flag(); else clear_v_flag();
	test_nz_flags(Byte);

	wr8(ea, Byte);
}

IRAM_ATTR void mc6809::inca(uint16_t ea)
{
	bool bit_7_carry_in = (((ac & 0x7f) + 0x01) & 0x80) ? true : false;

	Word = ac + 0x01;		// do an addition
	ac = Word & 0x00ff;

	bool carry = (Word & 0x0100) ? true : false;

	if (carry != bit_7_carry_in) set_v_flag(); else clear_v_flag();
	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::incb(uint16_t ea)
{
	bool bit_7_carry_in = (((br & 0x7f) + 0x01) & 0x80) ? true : false;

	Word = br + 0x01;		// do an addition
	br = Word & 0x00ff;

	bool carry = (Word & 0x0100) ? true : false;

	if (carry != bit_7_carry_in) set_v_flag(); else clear_v_flag();
	test_nz_flags(br);
}

IRAM_ATTR void mc6809::jmp(uint16_t ea)
{
	pc = ea;
}

IRAM_ATTR void mc6809::jsr(uint16_t ea)
{
	push_sp(pc & 0x00ff);
	push_sp((pc & 0xff00) >> 8);
	pc = ea;
}

IRAM_ATTR void mc6809::lbeq(uint16_t ea)
{
	if (is_z_flag_set()) {
		pc = ea;
		cycles += lbx;	// branche longue prise = +1 cycle

	}
}

IRAM_ATTR void mc6809::lbge(uint16_t ea)
{
	// both n and v set OR both n and v clear
	if ((is_n_flag_set() && is_v_flag_set()) || (is_n_flag_clear() && is_v_flag_clear())) {
		pc = ea;
		cycles += lbx;	// branche longue prise = +1 cycle

	}
}

IRAM_ATTR void mc6809::lbgt(uint16_t ea)
{
	// (both n and v set OR both n and v clear) AND (z clear)
	if (((is_n_flag_set() && is_v_flag_set()) || (is_n_flag_clear() && is_v_flag_clear())) && is_z_flag_clear()) {
		pc = ea;
		cycles += lbx;	// branche longue prise = +1 cycle

	}
}

IRAM_ATTR void mc6809::lbhi(uint16_t ea)
{
	if (is_z_flag_clear() && is_c_flag_clear()) {
		pc = ea;
		cycles += lbx;	// branche longue prise = +1 cycle

	}
}

// lbhs - Branch if Higher or Same lbcc - Branch if Carry Clear E.g. if no borrow was needed (carry
// clear) after a comparison, then value in register must be higher or the same as the compared value.
IRAM_ATTR void mc6809::lbhs(uint16_t ea)
{
	if (is_c_flag_clear()) {
		pc = ea;
		cycles += lbx;	// branche longue prise = +1 cycle

	}
}

IRAM_ATTR void mc6809::lble(uint16_t ea)
{
	if (is_z_flag_set() || (is_n_flag_set() && is_v_flag_clear()) || (is_n_flag_clear() && is_v_flag_set())) {
		pc = ea;
		cycles += lbx;	// branche longue prise = +1 cycle

	}
}

// lblo - Branch if Lower lbcs - Branch if Carry Set E.g. if a borrow was needed (carry set) after a
// comparison, the value in the register must be lower than the compared value.
IRAM_ATTR void mc6809::lblo(uint16_t ea)
{
	if (is_c_flag_set()) {
		pc = ea;
		cycles += lbx;	// branche longue prise = +1 cycle

	}
}

// bls - Branch if Lower or Same
IRAM_ATTR void mc6809::lbls(uint16_t ea)
{
	if (is_c_flag_set() || is_z_flag_set()) {
		pc = ea;
		cycles += lbx;	// branche longue prise = +1 cycle

	}
}

IRAM_ATTR void mc6809::lblt(uint16_t ea)
{
	if ((is_n_flag_set() && is_v_flag_clear()) || (is_n_flag_clear() && is_v_flag_set())) {
		pc = ea;
		cycles += lbx;	// branche longue prise = +1 cycle

	}
}

IRAM_ATTR void mc6809::lbmi(uint16_t ea)
{
	if (is_n_flag_set()) {
		pc = ea;
		cycles += lbx;	// branche longue prise = +1 cycle

	}
}

IRAM_ATTR void mc6809::lbne(uint16_t ea)
{
	if (is_z_flag_clear()) {
		pc = ea;
		cycles += lbx;	// branche longue prise = +1 cycle

	}
}

IRAM_ATTR void mc6809::lbpl(uint16_t ea)
{
	if (is_n_flag_clear()) {
		pc = ea;
		cycles += lbx;	// branche longue prise = +1 cycle

	}
}

IRAM_ATTR void mc6809::lbra(uint16_t ea)
{
	pc = ea;
}

IRAM_ATTR void mc6809::lbrn(uint16_t ea)
{
	// does essentially nothing
}

IRAM_ATTR void mc6809::lbsr(uint16_t ea)
{
	push_sp(pc & 0x00ff);
	push_sp((pc & 0xff00) >> 8);
	pc = ea;
}

IRAM_ATTR void mc6809::lbvc(uint16_t ea)
{
	if (is_v_flag_clear()) {
		pc = ea;
		cycles += lbx;	// branche longue prise = +1 cycle

	}
}

IRAM_ATTR void mc6809::lbvs(uint16_t ea)
{
	if (is_v_flag_set()) {
		pc = ea;
		cycles += lbx;	// branche longue prise = +1 cycle

	}
}

IRAM_ATTR void mc6809::lda(uint16_t ea)
{
	ac = rd8(ea);
	clear_v_flag();
	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::ldb(uint16_t ea)
{
	br = rd8(ea);
	clear_v_flag();
	test_nz_flags(br);
}

IRAM_ATTR void mc6809::ldd(uint16_t ea)
{
	ac = rd8(ea++);
	br = rd8((uint16_t)ea);
	d_reg = (ac << 8) | br;
	clear_v_flag();
	test_nz_flags_16(d_reg);
}

IRAM_ATTR void mc6809::lds(uint16_t ea)
{
	sp = rd8(ea++) << 8;
	sp |= rd8((uint16_t)ea);
	clear_v_flag();
	test_nz_flags_16(sp);

	// a write to system stackpointer enables nmi's
	nmi_enabled = true;
}

IRAM_ATTR void mc6809::ldu(uint16_t ea)
{
	us = rd8(ea++) << 8;
	us |= rd8((uint16_t)ea);
	clear_v_flag();
	test_nz_flags_16(us);
}

IRAM_ATTR void mc6809::ldx(uint16_t ea)
{
	xr = rd8(ea++) << 8;
	xr |= rd8((uint16_t)ea);
	clear_v_flag();
	test_nz_flags_16(xr);
}

IRAM_ATTR void mc6809::ldy(uint16_t ea)
{
	yr = rd8(ea++) << 8;
	yr |= rd8((uint16_t)ea);
	clear_v_flag();
	test_nz_flags_16(yr);
}

IRAM_ATTR void mc6809::leax(uint16_t ea)
{
	test_z_flag_16(ea);
	xr = ea;
}

IRAM_ATTR void mc6809::leay(uint16_t ea)
{
	test_z_flag_16(ea);
	yr = ea;
}

IRAM_ATTR void mc6809::leas(uint16_t ea)
{

	sp = ea;

	// a write to system stackpointer enables nmi's
	nmi_enabled = true;
}

IRAM_ATTR void mc6809::leau(uint16_t ea)
{

	us = ea;
}

IRAM_ATTR void mc6809::lsr(uint16_t ea)
{
	Byte = rd8(ea);
	if (Byte & 0x01) set_c_flag(); else clear_c_flag();
	Byte >>= 1;
	test_z_flag(Byte);
	clear_n_flag();
	wr8(ea, Byte);
}

IRAM_ATTR void mc6809::lsra(uint16_t ea)
{
	if (ac & 0x01) set_c_flag(); else clear_c_flag();
	ac >>= 1;
	test_z_flag(ac);
	clear_n_flag();
}

IRAM_ATTR void mc6809::lsrb(uint16_t ea)
{
	if (br & 0x01) set_c_flag(); else clear_c_flag();
	br >>= 1;
	test_z_flag(br);
	clear_n_flag();
}

IRAM_ATTR void mc6809::mul(uint16_t ea)
{
	d_reg = ac * br;
	test_z_flag_16(d_reg);
	ac = (d_reg & 0xff00) >> 8;
	br = d_reg & 0xff;
	if (br & 0x80) set_c_flag(); else clear_c_flag();
}

IRAM_ATTR void mc6809::neg(uint16_t ea)
{
	Byte = rd8(ea);
	if (Byte == 0x80) set_v_flag(); else clear_v_flag();
	if (Byte == 0x00) clear_c_flag(); else set_c_flag();
	Byte = ~Byte;
	Byte++;
	test_nz_flags(Byte);
	wr8(ea, Byte);
}

IRAM_ATTR void mc6809::nega(uint16_t ea)
{
	if (ac == 0x80) set_v_flag(); else clear_v_flag();
	if (ac == 0x00) clear_c_flag(); else set_c_flag();
	ac = ~ac;
	ac++;
	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::negb(uint16_t ea)
{
	if (br == 0x80) set_v_flag(); else clear_v_flag();
	if (br == 0x00) clear_c_flag(); else set_c_flag();
	br = ~br;
	br++;
	test_nz_flags(br);
}

IRAM_ATTR void mc6809::nop(uint16_t ea)
{
	// does nothing
}

IRAM_ATTR void mc6809::ora(uint16_t ea)
{
	Byte = ac | rd8(ea);
	clear_v_flag();
	test_nz_flags(Byte);
	ac = Byte;
}

IRAM_ATTR void mc6809::orb(uint16_t ea)
{
	Byte = br | rd8(ea);
	clear_v_flag();
	test_nz_flags(Byte);
	br = Byte;
}

IRAM_ATTR void mc6809::orcc(uint16_t ea)
{
	cc |= rd8(ea);
}

IRAM_ATTR void mc6809::page2(uint16_t ea)
{
	uint8_t opcode = rd8(pc++);

	bool am_legal;

	if (opcodes_page2[opcode] == &mc6809::ill)
	{
		// vrai 6809, préfixe $10 suivi d'un opcode inconnu = instruction de la page 1 (+1 cycle). Ex. $10
		// $86 = LDA #.
		cycles += cycles_page1[opcode] + 1;
		bool am_legal1;
		uint16_t ea1 = (this->*addressing_modes_page1[opcode])(&am_legal1);
		(this->*opcodes_page1[opcode])(ea1);
		return;
	}
	cycles += cycles_page2[opcode];	// cycles des instructions $10 xx
	uint16_t effective_address = (this->*addressing_modes_page2[opcode])(&am_legal);
	(this->*opcodes_page2[opcode])(effective_address);
}

IRAM_ATTR void mc6809::page3(uint16_t ea)
{
	uint8_t opcode = rd8(pc++);

	bool am_legal;

	if (opcodes_page3[opcode] == &mc6809::ill)
	{
		// vrai 6809, préfixe $11 suivi d'un opcode inconnu = instruction de la page 1 (+1 cycle). Ex. $10
		// $86 = LDA #.
		cycles += cycles_page1[opcode] + 1;
		bool am_legal1;
		uint16_t ea1 = (this->*addressing_modes_page1[opcode])(&am_legal1);
		(this->*opcodes_page1[opcode])(ea1);
		return;
	}
	cycles += cycles_page3[opcode];	// cycles des instructions $11 xx
	uint16_t effective_address = (this->*addressing_modes_page3[opcode])(&am_legal);
	(this->*opcodes_page3[opcode])(effective_address);
}

IRAM_ATTR void mc6809::pshs(uint16_t ea)
{
	Byte = rd8(ea);
	// +1 cycle par octet transféré (registres 16 bits = 2)
	cycles += __builtin_popcount(Byte & 0xF0) * 2 + __builtin_popcount(Byte & 0x0F);

	if (Byte & 0x80) { push_sp(pc & 0x00ff); push_sp((pc & 0xff00) >> 8); }
	if (Byte & 0x40) { push_sp(us & 0x00ff); push_sp((us & 0xff00) >> 8); }
	if (Byte & 0x20) { push_sp(yr & 0x00ff); push_sp((yr & 0xff00) >> 8); }
	if (Byte & 0x10) { push_sp(xr & 0x00ff); push_sp((xr & 0xff00) >> 8); }
	if (Byte & 0x08) { push_sp(dp);                                       }
	if (Byte & 0x04) { push_sp(br);                                       }
	if (Byte & 0x02) { push_sp(ac);                                       }
	if (Byte & 0x01) { push_sp(cc);                                       }
}

IRAM_ATTR void mc6809::pshu(uint16_t ea)
{
	Byte = rd8(ea);
	// +1 cycle par octet transféré (registres 16 bits = 2)
	cycles += __builtin_popcount(Byte & 0xF0) * 2 + __builtin_popcount(Byte & 0x0F);

	if (Byte & 0x80) { push_us(pc & 0x00ff); push_us((pc & 0xff00) >> 8); }
	if (Byte & 0x40) { push_us(sp & 0x00ff); push_us((sp & 0xff00) >> 8); }
	if (Byte & 0x20) { push_us(yr & 0x00ff); push_us((yr & 0xff00) >> 8); }
	if (Byte & 0x10) { push_us(xr & 0x00ff); push_us((xr & 0xff00) >> 8); }
	if (Byte & 0x08) { push_us(dp);                                       }
	if (Byte & 0x04) { push_us(br);                                       }
	if (Byte & 0x02) { push_us(ac);                                       }
	if (Byte & 0x01) { push_us(cc);                                       }
}

IRAM_ATTR void mc6809::puls(uint16_t ea)
{
	Byte = rd8(ea);
	// +1 cycle par octet transféré (registres 16 bits = 2)
	cycles += __builtin_popcount(Byte & 0xF0) * 2 + __builtin_popcount(Byte & 0x0F);

	if (Byte & 0x01) { cc   = pull_sp();                                    }
	if (Byte & 0x02) { ac   = pull_sp();                                    }
	if (Byte & 0x04) { br   = pull_sp();                                    }
	if (Byte & 0x08) { dp   = pull_sp();                                    }
	if (Byte & 0x10) { Word = pull_sp() << 8; Word |= pull_sp(); xr = Word; }
	if (Byte & 0x20) { Word = pull_sp() << 8; Word |= pull_sp(); yr = Word; }
	if (Byte & 0x40) { Word = pull_sp() << 8; Word |= pull_sp(); us = Word; }
	if (Byte & 0x80) { Word = pull_sp() << 8; Word |= pull_sp(); pc = Word; }
}

IRAM_ATTR void mc6809::pulu(uint16_t ea)
{
	Byte = rd8(ea);
	// +1 cycle par octet transféré (registres 16 bits = 2)
	cycles += __builtin_popcount(Byte & 0xF0) * 2 + __builtin_popcount(Byte & 0x0F);

	if (Byte & 0x01) { cc   = pull_us();                                    }
	if (Byte & 0x02) { ac   = pull_us();                                    }
	if (Byte & 0x04) { br   = pull_us();                                    }
	if (Byte & 0x08) { dp   = pull_us();                                    }
	if (Byte & 0x10) { Word = pull_us() << 8; Word |= pull_us(); xr = Word; }
	if (Byte & 0x20) { Word = pull_us() << 8; Word |= pull_us(); yr = Word; }
	if (Byte & 0x40) { Word = pull_us() << 8; Word |= pull_us(); sp = Word; }
	if (Byte & 0x80) { Word = pull_us() << 8; Word |= pull_us(); pc = Word; }
}

IRAM_ATTR void mc6809::rol(uint16_t ea)
{
	Byte = rd8(ea);
	uint8_t old_carry = cc & C_FLAG;
	if (((Byte & 0b11000000) == 0b01000000) || ((Byte & 0b11000000) == 0b10000000))
		set_v_flag(); else clear_v_flag();
	if (Byte & 0x80) set_c_flag(); else clear_c_flag();
	Byte <<= 1;
	Byte |= old_carry;
	test_nz_flags(Byte);
	wr8(ea, Byte);
}

IRAM_ATTR void mc6809::rola(uint16_t ea)
{
	uint8_t old_carry = cc & C_FLAG;
	if (((ac & 0b11000000) == 0b01000000) || ((ac & 0b11000000) == 0b10000000))
		set_v_flag(); else clear_v_flag();
	if (ac & 0x80) set_c_flag(); else clear_c_flag();
	ac <<= 1;
	ac |= old_carry;
	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::rolb(uint16_t ea)
{
	uint8_t old_carry = cc & C_FLAG;
	if (((br & 0b11000000) == 0b01000000) || ((br & 0b11000000) == 0b10000000))
		set_v_flag(); else clear_v_flag();
	if (br & 0x80) set_c_flag(); else clear_c_flag();
	br <<= 1;
	br |= old_carry;
	test_nz_flags(br);
}

IRAM_ATTR void mc6809::ror(uint16_t ea)
{
	Byte = rd8(ea);
	bool old_carry = is_c_flag_set();
	if (Byte & 0x01) set_c_flag(); else clear_c_flag();
	Byte >>= 1;
	if (old_carry) Byte |= 0x80;
	test_nz_flags(Byte);
	wr8(ea, Byte);
}

IRAM_ATTR void mc6809::rora(uint16_t ea)
{
	bool old_carry = is_c_flag_set();
	if (ac & 0x01) set_c_flag(); else clear_c_flag();
	ac >>= 1;
	if (old_carry) ac |= 0x80;
	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::rorb(uint16_t ea)
{
	bool old_carry = is_c_flag_set();
	if (br & 0x01) set_c_flag(); else clear_c_flag();
	br >>= 1;
	if (old_carry) br |= 0x80;
	test_nz_flags(br);
}

IRAM_ATTR void mc6809::rti(uint16_t ea)
{
	cc = pull_sp();
	if (is_e_flag_set()) {
		cycles += 9;	// état complet = 15 cycles au lieu de 6
		ac = pull_sp();
		br = pull_sp();
		if (__builtin_expect(nat, 0)) { er = pull_sp(); fr = pull_sp(); cycles += 2; }
		dp = pull_sp();
		Word = pull_sp() << 8;
		Word |= pull_sp();
		xr = Word;
		Word = pull_sp() << 8;
		Word |= pull_sp();
		yr = Word;
		Word = pull_sp() << 8;
		Word |= pull_sp();
		us = Word;

	}
	Word = pull_sp() << 8;
	Word |= pull_sp();
	pc = Word;
}

IRAM_ATTR void mc6809::rts(uint16_t ea)
{
	Word = pull_sp() << 8;
	Word |= pull_sp();
	pc = Word;
}

IRAM_ATTR void mc6809::sbca(uint16_t ea)
{
	// code inspired by virtualc64
	Byte = rd8(ea);
	Word = ac - Byte - (is_c_flag_set() ? 1 : 0);

	if (Word > 255) set_c_flag(); else clear_c_flag();

	if (((ac ^ Word) & 0x80) && ((ac ^ Byte) & 0x80)) set_v_flag(); else clear_v_flag();

	ac = Word & 0xff;
	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::sbcb(uint16_t ea)
{
	// code inspired by virtualc64
	Byte = rd8(ea);
	Word = br - Byte - (is_c_flag_set() ? 1 : 0);

	if (Word > 255) set_c_flag(); else clear_c_flag();

	if (((br ^ Word) & 0x80) && ((br ^ Byte) & 0x80)) set_v_flag(); else clear_v_flag();

	br = Word & 0xff;
	test_nz_flags(br);
}

IRAM_ATTR void mc6809::sex(uint16_t ea)
{
	if (br & 0x80) ac = 0xff; else ac = 0x00;
	test_nz_flags(br);
}

IRAM_ATTR void mc6809::sta(uint16_t ea)
{
	wr8(ea, ac);
	clear_v_flag();
	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::stb(uint16_t ea)
{
	wr8(ea, br);
	clear_v_flag();
	test_nz_flags(br);
}

IRAM_ATTR void mc6809::std(uint16_t ea)
{
	wr8(ea++, ac);
	wr8(ea, br);
	d_reg = (ac << 8) | br;
	clear_v_flag();
	test_nz_flags_16(d_reg);
}

IRAM_ATTR void mc6809::stu(uint16_t ea)
{
	wr8(ea++, us >> 8);
	wr8(ea, us & 0xff);
	clear_v_flag();
	test_nz_flags_16(us);
}

IRAM_ATTR void mc6809::sts(uint16_t ea)
{
	wr8(ea++, sp >> 8);
	wr8(ea, sp & 0xff);
	clear_v_flag();
	test_nz_flags_16(sp);
}

IRAM_ATTR void mc6809::stx(uint16_t ea)
{
	wr8(ea++, xr >> 8);
	wr8(ea, xr & 0xff);
	clear_v_flag();
	test_nz_flags_16(xr);
}

IRAM_ATTR void mc6809::sty(uint16_t ea)
{
	wr8(ea++, yr >> 8);
	wr8(ea, yr & 0xff);
	clear_v_flag();
	test_nz_flags_16(yr);
}

IRAM_ATTR void mc6809::suba(uint16_t ea)
{
	// code inspired by virtualc64
	Byte = rd8(ea);
	Word = ac - Byte;

	if (Word > 255) set_c_flag(); else clear_c_flag();

	if (((ac ^ Word) & 0x80) && ((ac ^ Byte) & 0x80)) set_v_flag(); else clear_v_flag();

	ac = Word & 0xff;
	test_nz_flags(ac);
}

IRAM_ATTR void mc6809::subb(uint16_t ea)
{
	// code inspired by virtualc64
	Byte = rd8(ea);
	Word = br - Byte;

	if (Word > 255) set_c_flag(); else clear_c_flag();

	if (((br ^ Word) & 0x80) && ((br ^ Byte) & 0x80)) set_v_flag(); else clear_v_flag();

	br = Word & 0xff;
	test_nz_flags(br);
}

IRAM_ATTR void mc6809::subd(uint16_t ea)
{
	// code inspired by virtualc64
	Word = rd8(ea++) << 8;
	Word |= rd8((uint16_t)ea);

	d_reg = (ac << 8) | br;

	Dword = d_reg - Word;

	if (Dword > 65535) set_c_flag(); else clear_c_flag();

	if (((d_reg ^ Dword) & 0x8000) && ((d_reg ^ Word) & 0x8000)) set_v_flag(); else clear_v_flag();

	d_reg = Dword & 0xffff;
	ac = (d_reg & 0xff00) >> 8;
	br = d_reg & 0xff;
	test_nz_flags_16(d_reg);
}

IRAM_ATTR void mc6809::swi(uint16_t ea)
{
	set_e_flag();
	push_all();	// + F, E en mode natif 6309
	set_i_flag();
	set_f_flag();
	pc = 0;
	pc = (rd8(VECTOR_SWI)) << 8;
	pc |= rd8(VECTOR_SWI+1);
}

IRAM_ATTR void mc6809::swi2(uint16_t ea)
{
	set_e_flag();
	push_all();	// + F, E en mode natif 6309
	pc = 0;
	pc = (rd8(VECTOR_SWI2)) << 8;
	pc |= rd8(VECTOR_SWI2+1);
}

IRAM_ATTR void mc6809::swi3(uint16_t ea)
{
	set_e_flag();
	push_all();	// + F, E en mode natif 6309
	pc = 0;
	pc = (rd8(VECTOR_SWI3)) << 8;
	pc |= rd8(VECTOR_SWI3+1);
}

IRAM_ATTR void mc6809::sync(uint16_t ea)
{
	// SYNC géré dans le coeur (voir execute) : le CPU attend qu'une ligne d'interruption soit active,
	// pendant que le temps émulé (GIME, PIA) continue d'avancer via StepCPU.
	cpu_state = CPU_SYNC;

}

IRAM_ATTR void mc6809::tfr(uint16_t ea)
{
	// toutes les combinaisons, comme le vrai 6809 (MAME)
	const uint8_t post = rd8(ea);
	write_tfr_reg(post & 0x0f, read_tfr_reg(post >> 4));
}

IRAM_ATTR void mc6809::tst(uint16_t ea)
{
	test_nz_flags(rd8(ea));
	clear_v_flag();
}

IRAM_ATTR void mc6809::tsta(uint16_t ea)
{
	test_nz_flags(ac);
	clear_v_flag();
}

IRAM_ATTR void mc6809::tstb(uint16_t ea)
{
	test_nz_flags(br);
	clear_v_flag();
}

// ===========================================================================
// Généré depuis les tables opcodes_page1 / addressing_modes_page1 de mc6809.hpp (mêmes fonctions, même
// ordre : mode d'adressage puis opcode)
// ===========================================================================
// l'aiguillage (ancien dispatch1) est dans execute, plus bas.

// déplacé depuis mc6809.cpp pour mettre dispatch1 en ligne.
IRAM_ATTR uint16_t mc6809::execute()
{	// compteur de trace de débogage retiré (inutile, coûtait à chaque…)
	uint32_t old_cycles = cycles;
	static void * const DRAM_ATTR k_tab[256] = {
		&&L00, &&L01, &&L02, &&L03, &&L04, &&L05, &&L06, &&L07,
		&&L08, &&L09, &&L0A, &&L0B, &&L0C, &&L0D, &&L0E, &&L0F,
		&&L10, &&L11, &&L12, &&L13, &&L14, &&L15, &&L16, &&L17,
		&&L18, &&L19, &&L1A, &&L1B, &&L1C, &&L1D, &&L1E, &&L1F,
		&&L20, &&L21, &&L22, &&L23, &&L24, &&L25, &&L26, &&L27,
		&&L28, &&L29, &&L2A, &&L2B, &&L2C, &&L2D, &&L2E, &&L2F,
		&&L30, &&L31, &&L32, &&L33, &&L34, &&L35, &&L36, &&L37,
		&&L38, &&L39, &&L3A, &&L3B, &&L3C, &&L3D, &&L3E, &&L3F,
		&&L40, &&L41, &&L42, &&L43, &&L44, &&L45, &&L46, &&L47,
		&&L48, &&L49, &&L4A, &&L4B, &&L4C, &&L4D, &&L4E, &&L4F,
		&&L50, &&L51, &&L52, &&L53, &&L54, &&L55, &&L56, &&L57,
		&&L58, &&L59, &&L5A, &&L5B, &&L5C, &&L5D, &&L5E, &&L5F,
		&&L60, &&L61, &&L62, &&L63, &&L64, &&L65, &&L66, &&L67,
		&&L68, &&L69, &&L6A, &&L6B, &&L6C, &&L6D, &&L6E, &&L6F,
		&&L70, &&L71, &&L72, &&L73, &&L74, &&L75, &&L76, &&L77,
		&&L78, &&L79, &&L7A, &&L7B, &&L7C, &&L7D, &&L7E, &&L7F,
		&&L80, &&L81, &&L82, &&L83, &&L84, &&L85, &&L86, &&L87,
		&&L88, &&L89, &&L8A, &&L8B, &&L8C, &&L8D, &&L8E, &&L8F,
		&&L90, &&L91, &&L92, &&L93, &&L94, &&L95, &&L96, &&L97,
		&&L98, &&L99, &&L9A, &&L9B, &&L9C, &&L9D, &&L9E, &&L9F,
		&&LA0, &&LA1, &&LA2, &&LA3, &&LA4, &&LA5, &&LA6, &&LA7,
		&&LA8, &&LA9, &&LAA, &&LAB, &&LAC, &&LAD, &&LAE, &&LAF,
		&&LB0, &&LB1, &&LB2, &&LB3, &&LB4, &&LB5, &&LB6, &&LB7,
		&&LB8, &&LB9, &&LBA, &&LBB, &&LBC, &&LBD, &&LBE, &&LBF,
		&&LC0, &&LC1, &&LC2, &&LC3, &&LC4, &&LC5, &&LC6, &&LC7,
		&&LC8, &&LC9, &&LCA, &&LCB, &&LCC, &&LCD, &&LCE, &&LCF,
		&&LD0, &&LD1, &&LD2, &&LD3, &&LD4, &&LD5, &&LD6, &&LD7,
		&&LD8, &&LD9, &&LDA, &&LDB, &&LDC, &&LDD, &&LDE, &&LDF,
		&&LE0, &&LE1, &&LE2, &&LE3, &&LE4, &&LE5, &&LE6, &&LE7,
		&&LE8, &&LE9, &&LEA, &&LEB, &&LEC, &&LED, &&LEE, &&LEF,
		&&LF0, &&LF1, &&LF2, &&LF3, &&LF4, &&LF5, &&LF6, &&LF7,
		&&LF8, &&LF9, &&LFA, &&LFB, &&LFC, &&LFD, &&LFE, &&LFF,
	};
	// aiguillage de la page 1 du HD6309 (mêmes étiquettes sauf opcodes propres au 6309)
	static void * const DRAM_ATTR k_tab63[256] = {
		&&L00, &&L01_63, &&L02_63, &&L03, &&L04, &&L05_63, &&L06, &&L07,
		&&L08, &&L09, &&L0A, &&L0B_63, &&L0C, &&L0D, &&L0E, &&L0F,
		&&L10_63, &&L11_63, &&L12, &&L13, &&L14_63, &&LILL63, &&L16, &&L17,
		&&LILL63, &&L19, &&L1A, &&LILL63, &&L1C, &&L1D, &&L1E, &&L1F,
		&&L20, &&L21, &&L22, &&L23, &&L24, &&L25, &&L26, &&L27,
		&&L28, &&L29, &&L2A, &&L2B, &&L2C, &&L2D, &&L2E, &&L2F,
		&&L30, &&L31, &&L32, &&L33, &&L34, &&L35, &&L36, &&L37,
		&&LILL63, &&L39, &&L3A, &&L3B, &&L3C, &&L3D, &&LILL63, &&L3F,
		&&L40, &&LILL63, &&LILL63, &&L43, &&L44, &&LILL63, &&L46, &&L47,
		&&L48, &&L49, &&L4A, &&LILL63, &&L4C, &&L4D, &&LILL63, &&L4F,
		&&L50, &&LILL63, &&LILL63, &&L53, &&L54, &&LILL63, &&L56, &&L57,
		&&L58, &&L59, &&L5A, &&LILL63, &&L5C, &&L5D, &&LILL63, &&L5F,
		&&L60, &&L61_63, &&L62_63, &&L63, &&L64, &&L65_63, &&L66, &&L67,
		&&L68, &&L69, &&L6A, &&L6B_63, &&L6C, &&L6D, &&L6E, &&L6F,
		&&L70, &&L71_63, &&L72_63, &&L73, &&L74, &&L75_63, &&L76, &&L77,
		&&L78, &&L79, &&L7A, &&L7B_63, &&L7C, &&L7D, &&L7E, &&L7F,
		&&L80, &&L81, &&L82, &&L83, &&L84, &&L85, &&L86, &&LILL63,
		&&L88, &&L89, &&L8A, &&L8B, &&L8C, &&L8D, &&L8E, &&LILL63,
		&&L90, &&L91, &&L92, &&L93, &&L94, &&L95, &&L96, &&L97,
		&&L98, &&L99, &&L9A, &&L9B, &&L9C, &&L9D, &&L9E, &&L9F,
		&&LA0, &&LA1, &&LA2, &&LA3, &&LA4, &&LA5, &&LA6, &&LA7,
		&&LA8, &&LA9, &&LAA, &&LAB, &&LAC, &&LAD, &&LAE, &&LAF,
		&&LB0, &&LB1, &&LB2, &&LB3, &&LB4, &&LB5, &&LB6, &&LB7,
		&&LB8, &&LB9, &&LBA, &&LBB, &&LBC, &&LBD, &&LBE, &&LBF,
		&&LC0, &&LC1, &&LC2, &&LC3, &&LC4, &&LC5, &&LC6, &&LILL63,
		&&LC8, &&LC9, &&LCA, &&LCB, &&LCC, &&LCD_63, &&LCE, &&LILL63,
		&&LD0, &&LD1, &&LD2, &&LD3, &&LD4, &&LD5, &&LD6, &&LD7,
		&&LD8, &&LD9, &&LDA, &&LDB, &&LDC, &&LDD, &&LDE, &&LDF,
		&&LE0, &&LE1, &&LE2, &&LE3, &&LE4, &&LE5, &&LE6, &&LE7,
		&&LE8, &&LE9, &&LEA, &&LEB, &&LEC, &&LED, &&LEE, &&LEF,
		&&LF0, &&LF1, &&LF2, &&LF3, &&LF4, &&LF5, &&LF6, &&LF7,
		&&LF8, &&LF9, &&LFA, &&LFB, &&LFC, &&LFD, &&LFE, &&LFF,
	};

	uint8_t opcode;
	bool slow = h63;	// 6309 = toujours la fin « lente » (piège indexé, voir fin_lente)
	void * const *tab = h63 ? k_tab63 : k_tab;

	// chemin rapide, le cas de loin le plus fréquent état normal, aucune ligne d'interruption active, pas
	// de front NMI en attente
	if (__builtin_expect(cpu_state == CPU_NORMAL && *irq_line && *firq_line && *nmi_line && old_nmi_line, 1))
	{
		opcode = rd8(pc++);
		cycles += cyc1[opcode];   // table 6809, 6309 émulation ou 6309 natif
		goto *tab[opcode];   // aiguillage en ligne (retour par « fin_rapide »)
	}

	// SYNC -- attente de N'IMPORTE QUELLE ligne d'interruption active, masquée ou non. Masquée : on
	// reprend simplement à l'instruction suivante ; autorisée : elle est servie normalement ci-dessous.
	if (cpu_state == CPU_SYNC &&
	    ((*irq_line == false) || (*firq_line == false) ||
	     ((*nmi_line == false) && (old_nmi_line == true) && nmi_enabled)))
	{
		cpu_state = CPU_NORMAL;
	}
	// CWAI a déjà empilé l'état complet (E=1) : l'interruption qui réveille le CPU ne doit pas le
	// réempiler (voir nmi/firq/irq).
	cwai_stacked = (cpu_state == CPU_CWAI);

	if ((*nmi_line == false) && (old_nmi_line == true) && nmi_enabled)
	{
		cpu_state = CPU_NORMAL;
		tfm_run = false;	// un TFM interrompu repart de zéro
		nmi();
	}

	else if ((*firq_line == false) && is_f_flag_clear())
	{
		cpu_state = CPU_NORMAL;
		tfm_run = false;	// un TFM interrompu repart de zéro
		firq();
	}
	else if ((*irq_line == false) && is_i_flag_clear())
	{
		cpu_state = CPU_NORMAL;
		tfm_run = false;	// un TFM interrompu repart de zéro
		irq();
	}
	else
	{
		if (cpu_state == CPU_NORMAL)
		{
			opcode = rd8(pc++);
			// TODO: check for illegal opcode and start exception
			cycles += cyc1[opcode]; // enlevé d'en commentaires = activé
			slow = true;
			goto *tab[opcode];
		}
	}

fin_lente:
	old_nmi_line = *nmi_line;
	if (__builtin_expect(trap63, 0))
	{
		// post-octet indexé illégal du 6309 : on annule l'effet de l'instruction sur les registres, puis
		// piège $FFF0 (MD bit 6).
		trap63 = false;
		pc = snap63.pc; xr = snap63.xr; yr = snap63.yr; us = snap63.us; sp = snap63.sp;
		ac = snap63.ac; br = snap63.br; er = snap63.er; fr = snap63.fr; cc = snap63.cc; dp = snap63.dp;
		trap63_take(0x40);
	}
	return cycles - old_cycles;

	// ---- aiguillage de la page 1 (goto calculé, table en DRAM)
	{
		bool l;
	L00: neg(a_dir(&l)); goto fin;
	L01: neg(a_dir(&l)); goto fin;
	L02: negcom(a_dir(&l)); goto fin;
	L03: com(a_dir(&l)); goto fin;
	L04: lsr(a_dir(&l)); goto fin;
	L05: lsr(a_dir(&l)); goto fin;
	L06: ror(a_dir(&l)); goto fin;
	L07: asr(a_dir(&l)); goto fin;
	L08: asl(a_dir(&l)); goto fin;
	L09: rol(a_dir(&l)); goto fin;
	L0A: dec(a_dir(&l)); goto fin;
	L0B: dec(a_dir(&l)); goto fin;
	L0C: inc(a_dir(&l)); goto fin;
	L0D: tst(a_dir(&l)); goto fin;
	L0E: jmp(a_dir(&l)); goto fin;
	L0F: clr(a_dir(&l)); goto fin;
	L10: page2(a_ih(&l)); goto fin;
	L11: page3(a_ih(&l)); goto fin;
	L12:
		// suite de NOP exécutée d'un bloc. Les NOP n'ont aucun effet sauf le temps : on en enchaîne autant
		// que le permet le budget donné par l'émulateur (cycles avant le prochain évènement GIME)
		if (mc6809_nop_budget)
		{
			const uint32_t b = mc6809_nop_budget();       // cycles jusqu'au prochain évènement
			const uint32_t c = cyc1[0x12];                   // 2 cycles (1 en natif 6309)
			uint32_t k = c;                                  // cycles déjà comptés (ce NOP)
			while (k + c <= b && k < 256 && rd8(pc) == 0x12) { pc++; k += c; }
			cycles += k - c;
		}
		goto fin;
	L13: sync(a_ih(&l)); goto fin;
	L14: ill(a_no(&l)); goto fin;
	L15: ill(a_no(&l)); goto fin;
	L16: lbra(a_rew(&l)); goto fin;
	L17: lbsr(a_rew(&l)); goto fin;
	L18: nop(a_no(&l)); goto fin;
	L19: daa(a_ih(&l)); goto fin;
	L1A: orcc(a_imb(&l)); goto fin;
	L1B: nop(a_no(&l)); goto fin;
	L1C: andcc(a_imb(&l)); goto fin;
	L1D: sex(a_ih(&l)); goto fin;
	L1E: exg(a_imb(&l)); goto fin;
	L1F: tfr(a_imb(&l)); goto fin;
	L20: bra(a_reb(&l)); goto fin;
	L21: brn(a_reb(&l)); goto fin;
	L22: bhi(a_reb(&l)); goto fin;
	L23: bls(a_reb(&l)); goto fin;
	L24: bhs(a_reb(&l)); goto fin;
	L25: blo(a_reb(&l)); goto fin;
	L26: bne(a_reb(&l)); goto fin;
	L27: beq(a_reb(&l)); goto fin;
	L28: bvc(a_reb(&l)); goto fin;
	L29: bvs(a_reb(&l)); goto fin;
	L2A: bpl(a_reb(&l)); goto fin;
	L2B: bmi(a_reb(&l)); goto fin;
	L2C: bge(a_reb(&l)); goto fin;
	L2D: blt(a_reb(&l)); goto fin;
	L2E: bgt(a_reb(&l)); goto fin;
	L2F: ble(a_reb(&l)); goto fin;
	L30: leax(a_idx(&l)); goto fin;
	L31: leay(a_idx(&l)); goto fin;
	L32: leas(a_idx(&l)); goto fin;
	L33: leau(a_idx(&l)); goto fin;
	L34: pshs(a_imb(&l)); goto fin;
	L35: puls(a_imb(&l)); goto fin;
	L36: pshu(a_imb(&l)); goto fin;
	L37: pulu(a_imb(&l)); goto fin;
	L38: andcc(a_imb(&l)); goto fin;
	L39: rts(a_ih(&l)); goto fin;
	L3A: abx(a_ih(&l)); goto fin;
	L3B: rti(a_ih(&l)); goto fin;
	L3C: cwai(a_imb(&l)); goto fin;
	L3D: mul(a_ih(&l)); goto fin;
	L3E: xres(a_no(&l)); goto fin;
	L3F: swi(a_ih(&l)); goto fin;
	L40: nega(a_ih(&l)); goto fin;
	L41: nega(a_ih(&l)); goto fin;
	L42: negcoma(a_ih(&l)); goto fin;
	L43: coma(a_ih(&l)); goto fin;
	L44: lsra(a_ih(&l)); goto fin;
	L45: lsra(a_ih(&l)); goto fin;
	L46: rora(a_ih(&l)); goto fin;
	L47: asra(a_ih(&l)); goto fin;
	L48: asla(a_ih(&l)); goto fin;
	L49: rola(a_ih(&l)); goto fin;
	L4A: deca(a_ih(&l)); goto fin;
	L4B: deca(a_ih(&l)); goto fin;
	L4C: inca(a_ih(&l)); goto fin;
	L4D: tsta(a_ih(&l)); goto fin;
	L4E: clra(a_ih(&l)); goto fin;
	L4F: clra(a_ih(&l)); goto fin;
	L50: negb(a_ih(&l)); goto fin;
	L51: negb(a_ih(&l)); goto fin;
	L52: negcomb(a_ih(&l)); goto fin;
	L53: comb(a_ih(&l)); goto fin;
	L54: lsrb(a_ih(&l)); goto fin;
	L55: lsrb(a_ih(&l)); goto fin;
	L56: rorb(a_ih(&l)); goto fin;
	L57: asrb(a_ih(&l)); goto fin;
	L58: aslb(a_ih(&l)); goto fin;
	L59: rolb(a_ih(&l)); goto fin;
	L5A: decb(a_ih(&l)); goto fin;
	L5B: decb(a_ih(&l)); goto fin;
	L5C: incb(a_ih(&l)); goto fin;
	L5D: tstb(a_ih(&l)); goto fin;
	L5E: clrb(a_ih(&l)); goto fin;
	L5F: clrb(a_ih(&l)); goto fin;
	L60: neg(a_idx(&l)); goto fin;
	L61: neg(a_idx(&l)); goto fin;
	L62: negcom(a_idx(&l)); goto fin;
	L63: com(a_idx(&l)); goto fin;
	L64: lsr(a_idx(&l)); goto fin;
	L65: lsr(a_idx(&l)); goto fin;
	L66: ror(a_idx(&l)); goto fin;
	L67: asr(a_idx(&l)); goto fin;
	L68: asl(a_idx(&l)); goto fin;
	L69: rol(a_idx(&l)); goto fin;
	L6A: dec(a_idx(&l)); goto fin;
	L6B: dec(a_idx(&l)); goto fin;
	L6C: inc(a_idx(&l)); goto fin;
	L6D: tst(a_idx(&l)); goto fin;
	L6E: jmp(a_idx(&l)); goto fin;
	L6F: clr(a_idx(&l)); goto fin;
	L70: neg(a_ext(&l)); goto fin;
	L71: neg(a_ext(&l)); goto fin;
	L72: negcom(a_ext(&l)); goto fin;
	L73: com(a_ext(&l)); goto fin;
	L74: lsr(a_ext(&l)); goto fin;
	L75: lsr(a_ext(&l)); goto fin;
	L76: ror(a_ext(&l)); goto fin;
	L77: asr(a_ext(&l)); goto fin;
	L78: asl(a_ext(&l)); goto fin;
	L79: rol(a_ext(&l)); goto fin;
	L7A: dec(a_ext(&l)); goto fin;
	L7B: dec(a_ext(&l)); goto fin;
	L7C: inc(a_ext(&l)); goto fin;
	L7D: tst(a_ext(&l)); goto fin;
	L7E: jmp(a_ext(&l)); goto fin;
	L7F: clr(a_ext(&l)); goto fin;
	L80: suba(a_imb(&l)); goto fin;
	L81: cmpa(a_imb(&l)); goto fin;
	L82: sbca(a_imb(&l)); goto fin;
	L83: subd(a_imw(&l)); goto fin;
	L84: anda(a_imb(&l)); goto fin;
	L85: bita(a_imb(&l)); goto fin;
	L86: lda(a_imb(&l)); goto fin;
	L87: st_imm_a(a_imb(&l)); goto fin;
	L88: eora(a_imb(&l)); goto fin;
	L89: adca(a_imb(&l)); goto fin;
	L8A: ora(a_imb(&l)); goto fin;
	L8B: adda(a_imb(&l)); goto fin;
	L8C: cmpx(a_imw(&l)); goto fin;
	L8D: bsr(a_reb(&l)); goto fin;
	L8E: ldx(a_imw(&l)); goto fin;
	L8F: st_imm_x(a_imw(&l)); goto fin;
	L90: suba(a_dir(&l)); goto fin;
	L91: cmpa(a_dir(&l)); goto fin;
	L92: sbca(a_dir(&l)); goto fin;
	L93: subd(a_dir(&l)); goto fin;
	L94: anda(a_dir(&l)); goto fin;
	L95: bita(a_dir(&l)); goto fin;
	L96: lda(a_dir(&l)); goto fin;
	L97: sta(a_dir(&l)); goto fin;
	L98: eora(a_dir(&l)); goto fin;
	L99: adca(a_dir(&l)); goto fin;
	L9A: ora(a_dir(&l)); goto fin;
	L9B: adda(a_dir(&l)); goto fin;
	L9C: cmpx(a_dir(&l)); goto fin;
	L9D: jsr(a_dir(&l)); goto fin;
	L9E: ldx(a_dir(&l)); goto fin;
	L9F: stx(a_dir(&l)); goto fin;
	LA0: suba(a_idx(&l)); goto fin;
	LA1: cmpa(a_idx(&l)); goto fin;
	LA2: sbca(a_idx(&l)); goto fin;
	LA3: subd(a_idx(&l)); goto fin;
	LA4: anda(a_idx(&l)); goto fin;
	LA5: bita(a_idx(&l)); goto fin;
	LA6: lda(a_idx(&l)); goto fin;
	LA7: sta(a_idx(&l)); goto fin;
	LA8: eora(a_idx(&l)); goto fin;
	LA9: adca(a_idx(&l)); goto fin;
	LAA: ora(a_idx(&l)); goto fin;
	LAB: adda(a_idx(&l)); goto fin;
	LAC: cmpx(a_idx(&l)); goto fin;
	LAD: jsr(a_idx(&l)); goto fin;
	LAE: ldx(a_idx(&l)); goto fin;
	LAF: stx(a_idx(&l)); goto fin;
	LB0: suba(a_ext(&l)); goto fin;
	LB1: cmpa(a_ext(&l)); goto fin;
	LB2: sbca(a_ext(&l)); goto fin;
	LB3: subd(a_ext(&l)); goto fin;
	LB4: anda(a_ext(&l)); goto fin;
	LB5: bita(a_ext(&l)); goto fin;
	LB6: lda(a_ext(&l)); goto fin;
	LB7: sta(a_ext(&l)); goto fin;
	LB8: eora(a_ext(&l)); goto fin;
	LB9: adca(a_ext(&l)); goto fin;
	LBA: ora(a_ext(&l)); goto fin;
	LBB: adda(a_ext(&l)); goto fin;
	LBC: cmpx(a_ext(&l)); goto fin;
	LBD: jsr(a_ext(&l)); goto fin;
	LBE: ldx(a_ext(&l)); goto fin;
	LBF: stx(a_ext(&l)); goto fin;
	LC0: subb(a_imb(&l)); goto fin;
	LC1: cmpb(a_imb(&l)); goto fin;
	LC2: sbcb(a_imb(&l)); goto fin;
	LC3: addd(a_imw(&l)); goto fin;
	LC4: andb(a_imb(&l)); goto fin;
	LC5: bitb(a_imb(&l)); goto fin;
	LC6: ldb(a_imb(&l)); goto fin;
	LC7: st_imm_b(a_imb(&l)); goto fin;
	LC8: eorb(a_imb(&l)); goto fin;
	LC9: adcb(a_imb(&l)); goto fin;
	LCA: orb(a_imb(&l)); goto fin;
	LCB: addb(a_imb(&l)); goto fin;
	LCC: ldd(a_imw(&l)); goto fin;
	LCD: ill(a_no(&l)); goto fin;
	LCE: ldu(a_imw(&l)); goto fin;
	LCF: st_imm_u(a_imw(&l)); goto fin;
	LD0: subb(a_dir(&l)); goto fin;
	LD1: cmpb(a_dir(&l)); goto fin;
	LD2: sbcb(a_dir(&l)); goto fin;
	LD3: addd(a_dir(&l)); goto fin;
	LD4: andb(a_dir(&l)); goto fin;
	LD5: bitb(a_dir(&l)); goto fin;
	LD6: ldb(a_dir(&l)); goto fin;
	LD7: stb(a_dir(&l)); goto fin;
	LD8: eorb(a_dir(&l)); goto fin;
	LD9: adcb(a_dir(&l)); goto fin;
	LDA: orb(a_dir(&l)); goto fin;
	LDB: addb(a_dir(&l)); goto fin;
	LDC: ldd(a_dir(&l)); goto fin;
	LDD: std(a_dir(&l)); goto fin;
	LDE: ldu(a_dir(&l)); goto fin;
	LDF: stu(a_dir(&l)); goto fin;
	LE0: subb(a_idx(&l)); goto fin;
	LE1: cmpb(a_idx(&l)); goto fin;
	LE2: sbcb(a_idx(&l)); goto fin;
	LE3: addd(a_idx(&l)); goto fin;
	LE4: andb(a_idx(&l)); goto fin;
	LE5: bitb(a_idx(&l)); goto fin;
	LE6: ldb(a_idx(&l)); goto fin;
	LE7: stb(a_idx(&l)); goto fin;
	LE8: eorb(a_idx(&l)); goto fin;
	LE9: adcb(a_idx(&l)); goto fin;
	LEA: orb(a_idx(&l)); goto fin;
	LEB: addb(a_idx(&l)); goto fin;
	LEC: ldd(a_idx(&l)); goto fin;
	LED: std(a_idx(&l)); goto fin;
	LEE: ldu(a_idx(&l)); goto fin;
	LEF: stu(a_idx(&l)); goto fin;
	LF0: subb(a_ext(&l)); goto fin;
	LF1: cmpb(a_ext(&l)); goto fin;
	LF2: sbcb(a_ext(&l)); goto fin;
	LF3: addd(a_ext(&l)); goto fin;
	LF4: andb(a_ext(&l)); goto fin;
	LF5: bitb(a_ext(&l)); goto fin;
	LF6: ldb(a_ext(&l)); goto fin;
	LF7: stb(a_ext(&l)); goto fin;
	LF8: eorb(a_ext(&l)); goto fin;
	LF9: adcb(a_ext(&l)); goto fin;
	LFA: orb(a_ext(&l)); goto fin;
	LFB: addb(a_ext(&l)); goto fin;
	LFC: ldd(a_ext(&l)); goto fin;
	LFD: std(a_ext(&l)); goto fin;
	LFE: ldu(a_ext(&l)); goto fin;
	LFF: stu(a_ext(&l)); goto fin;

	// ---- opcodes propres au HD6309 (page 1)
	LILL63: trap63_take(0x40); goto fin;
	L01_63: { const uint8_t im = rd8(pc++); im63(0, im, a_dir(&l)); } goto fin;	// OIM
	L02_63: { const uint8_t im = rd8(pc++); im63(1, im, a_dir(&l)); } goto fin;	// AIM
	L05_63: { const uint8_t im = rd8(pc++); im63(2, im, a_dir(&l)); } goto fin;	// EIM
	L0B_63: { const uint8_t im = rd8(pc++); im63(3, im, a_dir(&l)); } goto fin;	// TIM
	L61_63: { const uint8_t im = rd8(pc++); im63(0, im, a_idx(&l)); } goto fin;
	L62_63: { const uint8_t im = rd8(pc++); im63(1, im, a_idx(&l)); } goto fin;
	L65_63: { const uint8_t im = rd8(pc++); im63(2, im, a_idx(&l)); } goto fin;
	L6B_63: { const uint8_t im = rd8(pc++); im63(3, im, a_idx(&l)); } goto fin;
	L71_63: { const uint8_t im = rd8(pc++); im63(0, im, a_ext(&l)); } goto fin;
	L72_63: { const uint8_t im = rd8(pc++); im63(1, im, a_ext(&l)); } goto fin;
	L75_63: { const uint8_t im = rd8(pc++); im63(2, im, a_ext(&l)); } goto fin;
	L7B_63: { const uint8_t im = rd8(pc++); im63(3, im, a_ext(&l)); } goto fin;
	L10_63: page2_63(); goto fin;
	L11_63: page3_63(); goto fin;
	L14_63: sexw63(); goto fin;
	LCD_63: ldq_imm63(); goto fin;
	fin:
		(void)l;
		if (slow) goto fin_lente;
		return cycles - old_cycles;
	}
	// return 0
}

// extension HD6309 (code en flash, hors IRAM)
#define MC6809_INCLUDE_HD6309
#include "hd6309.cpp"
