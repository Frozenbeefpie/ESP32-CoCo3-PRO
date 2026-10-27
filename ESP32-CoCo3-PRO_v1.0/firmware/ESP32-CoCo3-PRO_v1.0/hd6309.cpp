#include <string.h>
// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Extension HD6309 du coeur MC6809 (elmerucr, licence MIT).
// Sémantique des instructions d'après le coeur hd6309 de MAME (licence BSD-3-Clause, copyright Nathan Woods
// et contributeurs), réécrite pour ce coeur ; LDMD, BITMD, DECD/INCD, ROLD et cycles selon la doc Hitachi.

#ifdef MC6809_INCLUDE_HD6309
#include "mc6809.hpp"
#ifndef DRAM_ATTR
#define DRAM_ATTR
#endif

// Tables de cycles générées (h6309/gentab.py) : page 1 lue depuis execute en IRAM -> RAM interne
// (DRAM_ATTR), pas en flash.
#include "hd6309_cycles.h"

static inline uint8_t f8(uint8_t &cc, uint8_t mask, uint8_t a, uint8_t b, uint32_t r)
{
	cc &= ~mask;
	if (mask & H_FLAG) cc |= ((a ^ b ^ r) & 0x10) ? H_FLAG : 0;
	if (mask & N_FLAG) cc |= (r & 0x80) ? N_FLAG : 0;
	if (mask & Z_FLAG) cc |= ((uint8_t)r == 0) ? Z_FLAG : 0;
	if (mask & V_FLAG) cc |= ((a ^ b ^ r ^ (r >> 1)) & 0x80) ? V_FLAG : 0;
	if (mask & C_FLAG) cc |= (r & 0x100) ? C_FLAG : 0;
	return (uint8_t)r;
}

static inline uint16_t f16(uint8_t &cc, uint8_t mask, uint16_t a, uint16_t b, uint32_t r)
{
	cc &= ~mask;
	if (mask & N_FLAG) cc |= (r & 0x8000) ? N_FLAG : 0;
	if (mask & Z_FLAG) cc |= ((uint16_t)r == 0) ? Z_FLAG : 0;
	if (mask & V_FLAG) cc |= ((a ^ b ^ r ^ (r >> 1)) & 0x8000) ? V_FLAG : 0;
	if (mask & C_FLAG) cc |= (r & 0x10000) ? C_FLAG : 0;
	return (uint16_t)r;
}

static inline void fq(uint8_t &cc, uint32_t q)	// LDQ/STQ : N Z, V = 0
{
	cc &= ~(N_FLAG | Z_FLAG | V_FLAG);
	if (q & 0x80000000u) cc |= N_FLAG;
	if (q == 0) cc |= Z_FLAG;
}

// chargement / test : N Z selon la valeur, V = 0 (comme set_flags(mask, r) de MAME)
static inline uint16_t nzv16(uint8_t &cc, uint16_t v)
{
	cc &= ~(N_FLAG | Z_FLAG | V_FLAG);
	if (v & 0x8000) cc |= N_FLAG;
	if (v == 0) cc |= Z_FLAG;
	return v;
}
static inline uint8_t nzv8(uint8_t &cc, uint8_t v)
{
	cc &= ~(N_FLAG | Z_FLAG | V_FLAG);
	if (v & 0x80) cc |= N_FLAG;
	if (v == 0) cc |= Z_FLAG;
	return v;
}

#define NZVC (N_FLAG | Z_FLAG | V_FLAG | C_FLAG)
#define NZV  (N_FLAG | Z_FLAG | V_FLAG)
#define NZ   (N_FLAG | Z_FLAG)

// ---------------------------------------------------------------------------
// Type de CPU, registre MD
// ---------------------------------------------------------------------------
void mc6809::set_cpu_6309(bool on)
{
	h63 = on;
	md = 0;
	set_md(0);
	if (on && BOOT_LOG_VERBOSE) printf("[MC6809] mode HD6309\n");
}

// Save State
void mc6809::save_state(snapshot_t &s) const
{
	static_assert(sizeof(snap63) <= sizeof(s.snap63), "snap63 trop grand");
	memset(&s, 0, sizeof s);
	s.pc = pc; s.xr = xr; s.yr = yr; s.us = us; s.sp = sp; s.vr = vr;
	s.dp = dp; s.ac = ac; s.br = br; s.cc = cc; s.er = er; s.fr = fr; s.md = md;
	s.state = (uint8_t)cpu_state;
	s.cwai_stacked = cwai_stacked; s.h63 = h63; s.trap63 = trap63; s.tfm_run = tfm_run;
	s.nmi_enabled = nmi_enabled; s.old_nmi = old_nmi_line; s.old_firq = old_firq_line; s.old_irq = old_irq_line;
	s.cycle_saldo = cycle_saldo; s.cycles = cycles;
	memcpy(s.snap63, &snap63, sizeof(snap63));
}
void mc6809::load_state(const snapshot_t &s)
{
	h63 = s.h63 != 0;
	md = s.md;
	set_md(s.md);			// nat, cyc1, lbx
	md = h63 ? s.md : 0;		// bits 7-6 (pièges) compris
	pc = s.pc; xr = s.xr; yr = s.yr; us = s.us; sp = s.sp; vr = s.vr;
	dp = s.dp; ac = s.ac; br = s.br; cc = s.cc; er = s.er; fr = s.fr;
	cpu_state = (cpu_state_t)s.state;
	cwai_stacked = s.cwai_stacked; trap63 = s.trap63; tfm_run = s.tfm_run;
	nmi_enabled = s.nmi_enabled; old_nmi_line = s.old_nmi; old_firq_line = s.old_firq; old_irq_line = s.old_irq;
	cycle_saldo = s.cycle_saldo; cycles = s.cycles;
	memcpy(&snap63, s.snap63, sizeof(snap63));
}

void mc6809::set_md(uint8_t v)
{
	md = h63 ? (uint8_t)((md & 0xC0) | (v & 0x03)) : 0;
	nat = h63 && (md & 0x01);
	cyc1 = !h63 ? cycles_page1 : (nat ? k_cyc63n_p1 : k_cyc63e_p1);
	lbx = nat ? 0 : 1;
}

// Piège $FFF0 : opcode illégal (MD b6) ou division par zéro (MD b7). État complet empilé (avec E, F en
// natif) ; I et F inchangés (Hitachi).
void mc6809::trap63_take(uint8_t mdbit)
{
	md |= mdbit;
	tfm_run = false;
	traps63++;
	trap_pc63 = pc;
	set_e_flag();
	push_all();
	pc = rd16(VECTOR_ILL_OPC);
	cycles += 18;
}

// ---------------------------------------------------------------------------
// Modes indexés propres au 6309 (appelé par a_idx pour les post-octets que le 6809 n'a pas). Cycles en
// plus de la base de l'opcode, émulation/natif.
// ---------------------------------------------------------------------------
uint16_t mc6809::a_idx63(uint8_t pb, uint16_t *unused)
{
	(void)unused;
	uint16_t &r = *index_regs[(pb >> 5) & 3];
	uint16_t a = 0, w = wr();
	const bool ind = pb & 0x10;
	switch (pb & 0x0F)
	{
		case 0x7: a = r + (int8_t)er; cycles += 1; break;		// E,R
		case 0xA: a = r + (int8_t)fr; cycles += 1; break;		// F,R
		case 0xE: a = r + w; cycles += (nat && !ind) ? 1 : 4; break;	// W,R
		case 0xF:
			if (ind) {						// [n16]
				a = rd16(pc); pc += 2; cycles += 2;
				break;
			}
			switch (pb & 0x60) {
				case 0x00: a = w; break;					// W
				case 0x20: a = w + rd16(pc); pc += 2; cycles += nat ? 2 : 6; break;	// n16,W
				case 0x40: a = w; set_wr(w + 2); cycles += nat ? 1 : 3; break;	// W++
				default:   set_wr(w - 2); a = wr(); cycles += nat ? 1 : 3; break;	// W
			}
			break;
		case 0x0:
			if (ind) {
				switch (pb & 0x60) {
					case 0x00: a = w; break;					// [,W]
					case 0x20: a = w + rd16(pc); pc += 2; cycles += nat ? 2 : 6; break;	// [n16,W]
					case 0x40: a = w; set_wr(w + 2); cycles += 3; break;		// [,W++]
					default:   set_wr(w - 2); a = wr(); cycles += 3; break;	// [,--W]
				}
				break;
			}
			// pas d'autre cas : ,R+ est traité par a_idx fall through
		default:
			// [,-R] et autres : opcode illégal -> piège à la fin de l'instruction
			if (!trap63) {
				trap63 = true;
				snap63.pc = pc; snap63.xr = xr; snap63.yr = yr; snap63.us = us; snap63.sp = sp;
				snap63.ac = ac; snap63.br = br; snap63.er = er; snap63.fr = fr; snap63.cc = cc; snap63.dp = dp;
			}
			return r;
	}
	if (ind) { a = rd16(a); cycles += 3; }
	return a;
}

// ---------------------------------------------------------------------------
// TFR / EXG du 6309 : 0 D, 1 X, 2 Y, 3 U, 4 S, 5 PC, 6 W, 7 V, 8 A, 9 B A CC, B DP, C/D zéro, E E, F
// F. Lecture 8 bits = octet doublé (A:A) écriture 8 bits = octet haut (A, DP, E) ou bas (B, CC, F).
// ---------------------------------------------------------------------------
uint16_t mc6809::read_tfr63(uint8_t r)
{
	switch (r & 0x0f) {
		case 0x0: return (uint16_t)((ac << 8) | br);
		case 0x1: return xr;
		case 0x2: return yr;
		case 0x3: return us;
		case 0x4: return sp;
		case 0x5: return pc;
		case 0x6: return wr();
		case 0x7: return vr;
		case 0x8: return (uint16_t)((ac << 8) | ac);
		case 0x9: return (uint16_t)((br << 8) | br);
		case 0xa: return (uint16_t)((cc << 8) | cc);
		case 0xb: return (uint16_t)((dp << 8) | dp);
		case 0xe: return (uint16_t)((er << 8) | er);
		case 0xf: return (uint16_t)((fr << 8) | fr);
		default:  return 0;
	}
}

void mc6809::write_tfr63(uint8_t r, uint16_t v)
{
	switch (r & 0x0f) {
		case 0x0: ac = v >> 8; br = v & 0xff; break;
		case 0x1: xr = v; break;
		case 0x2: yr = v; break;
		case 0x3: us = v; break;
		case 0x4: sp = v; nmi_enabled = true; break;
		case 0x5: pc = v; break;
		case 0x6: set_wr(v); break;
		case 0x7: vr = v; break;
		case 0x8: ac = v >> 8; break;
		case 0x9: br = v & 0xff; break;
		case 0xa: cc = v & 0xff; break;
		case 0xb: dp = v >> 8; break;
		case 0xe: er = v >> 8; break;
		case 0xf: fr = v & 0xff; break;
		default: break;
	}
}

// ---------------------------------------------------------------------------
// Page 1 : OIM AIM EIM TIM, SEXW, LDQ #
// ---------------------------------------------------------------------------
void mc6809::im63(uint8_t kind, uint8_t im, uint16_t ea)
{
	const uint8_t m = rd8(ea);
	uint8_t r;
	switch (kind) {
		case 0:  r = m | im; break;
		case 2:  r = m ^ im; break;
		default: r = m & im; break;	// AIM, TIM
	}
	cc &= ~V_FLAG;
	f8(cc, NZ, 0, m, r);
	if (kind != 3) wr8(ea, r);
}

void mc6809::sexw63()
{
	const uint16_t w = wr();
	const uint16_t d = (w & 0x8000) ? 0xFFFF : 0x0000;
	ac = d >> 8; br = d & 0xff;
	cc &= ~(N_FLAG | Z_FLAG);
	if (d & 0x8000) cc |= N_FLAG;
	if (d == 0 && w == 0) cc |= Z_FLAG;
}

void mc6809::ldq_imm63()
{
	ac = rd8(pc++); br = rd8(pc++); er = rd8(pc++); fr = rd8(pc++);
	fq(cc, ((uint32_t)ac << 24) | ((uint32_t)br << 16) | wr());
}

// ---------------------------------------------------------------------------
// Opérations registre-registre : ADDR ADCR SUBR SBCR ANDR ORR EORR CMPR
// ---------------------------------------------------------------------------
void mc6809::rr63(uint8_t op)
{
	const uint8_t pb = rd8(pc++);
	const uint8_t s = pb >> 4, d = pb & 0x0f;
	const bool promote = ((pb & 0x80) != 0) != ((pb & 0x08) != 0);
	const uint8_t k = op & 7;
	const uint8_t c = cc & C_FLAG ? 1 : 0;

	if (d < 8 || promote)
	{
		// ---- 16 bits
		uint16_t sv;
		if (s < 8) sv = read_tfr63(s);
		else switch (s) {
			case 0x8: case 0x9: sv = promote ? (uint16_t)((ac << 8) | br) : 0; break;
			case 0xe: case 0xf: sv = promote ? wr() : 0; break;
			default: sv = 0; break;		// CC, DP (promus), 0
		}
		int dr;					// registre destination (TFR), -1 = aucun
		if (d < 8) dr = d;
		else if (d == 8 || d == 9) dr = 0;	// A, B promus -> D
		else if (d >= 0xe) dr = 6;		// E, F promus -> W
		else dr = -1;				// CC, DP, 0 promus : résultat perdu
		const uint16_t dv = dr >= 0 ? read_tfr63(dr) : 0;
		uint16_t r;
		switch (k) {
			case 0: r = f16(cc, NZVC, dv, sv, (uint32_t)dv + sv); break;
			case 1: r = f16(cc, NZVC, dv, sv, (uint32_t)dv + sv + c); break;
			case 2: case 7: r = f16(cc, NZVC, dv, sv, (uint32_t)dv - sv); break;
			case 3: r = f16(cc, NZVC, dv, sv, (uint32_t)dv - sv - c); break;
			case 4: cc &= ~V_FLAG; r = f16(cc, NZ, 0, dv, dv & sv); break;
			case 5: cc &= ~V_FLAG; r = f16(cc, NZ, 0, dv, dv | sv); break;
			default: cc &= ~V_FLAG; r = f16(cc, NZ, 0, dv, dv ^ sv); break;
		}
		if (k != 7 && dr >= 0) write_tfr63(dr, r);
	}
	else
	{
		// ---- 8 bits
		if (k >= 4 && k <= 6) cc &= ~V_FLAG;	// AND/OR/EOR : V effacé avant de lire CC (MAME)
		uint8_t sv;
		switch (s) {
			case 0x8: sv = ac; break;
			case 0x9: sv = br; break;
			case 0xa: sv = cc; break;
			case 0xb: sv = dp; break;
			case 0xe: sv = er; break;
			case 0xf: sv = fr; break;
			default:  sv = 0; break;
		}
		uint8_t *dp8;
		uint8_t dummy = 0;
		switch (d) {
			case 0x8: dp8 = &ac; break;
			case 0x9: dp8 = &br; break;
			case 0xa: dp8 = &cc; break;
			case 0xb: dp8 = &dp; break;
			case 0xe: dp8 = &er; break;
			case 0xf: dp8 = &fr; break;
			default:  dp8 = &dummy; break;
		}
		const uint8_t dv = *dp8;
		uint8_t r;
		switch (k) {
			case 0: r = f8(cc, NZVC, dv, sv, (uint32_t)dv + sv); break;		// pas de H (ADDR)
			case 1: r = f8(cc, NZVC, dv, sv, (uint32_t)dv + sv + c); break;
			case 2: case 7: r = f8(cc, NZVC, dv, sv, (uint32_t)dv - sv); break;
			case 3: r = f8(cc, NZVC, dv, sv, (uint32_t)dv - sv - c); break;
			case 4: cc &= ~V_FLAG; r = f8(cc, NZ, 0, dv, dv & sv); break;
			case 5: cc &= ~V_FLAG; r = f8(cc, NZ, 0, dv, dv | sv); break;
			default: cc &= ~V_FLAG; r = f8(cc, NZ, 0, dv, dv ^ sv); break;
		}
		if (k != 7) *dp8 = r;
	}
}

// ---------------------------------------------------------------------------
// TFM : transfert de bloc, W octets. Exécuté par tranches pour que le temps émulé (GIME,
// interruptions) avance
// ---------------------------------------------------------------------------
void mc6809::tfm63(uint8_t op)
{
	const uint8_t pb = rd8(pc++);
	const uint8_t s = pb >> 4, d = pb & 0x0f;
	if (tfm_run) cycles -= 6;		// reprise : base déjà comptée
	tfm_run = false;
	uint16_t w = wr();
	if (w == 0) return;
	if (s > 4) { trap63_take(0x40); return; }
	if (d > 4) {				// la source est lue (et avance) avant le piège
		uint16_t a = read_tfr63(s);
		(void)rd8(a);
		if (op == 0x38 || op == 0x3A) a++; else if (op == 0x39) a--;
		write_tfr63(s, a);
		trap63_take(0x40);
		return;
	}

	// octets par tranche : jusqu'au prochain évènement de l'émulateur (fin de ligne GIME) si connu ;
	// budget 0 = inconnu (timer rapide, overclock FDC...) -> petites tranches de 16 octets (48 cycles).
	uint32_t n = 16;
	if (mc6809_nop_budget) {
		const uint32_t b = mc6809_nop_budget();
		if (b) n = b > 12 ? (b - 12) / 3 : 1;
		if (n > 1024) n = 1024;
	}
	uint16_t sa = read_tfr63(s), da = read_tfr63(d);
	const int ds = (op == 0x38 || op == 0x3A) ? 1 : (op == 0x39 ? -1 : 0);
	const int dd = (op == 0x38 || op == 0x3B) ? 1 : (op == 0x39 ? -1 : 0);
	uint32_t k = 0;
	while (w != 0 && k < n) {
		const uint8_t v = rd8(sa);
		sa += ds;
		if (s == d) da = sa;		// même registre : la source a bougé
		wr8(da, v);
		da += dd;
		if (s == d) sa = da;
		w--; k++;
	}
	write_tfr63(s, sa);
	write_tfr63(d, da);
	set_wr(w);
	cycles += 3 * k;
	if (w != 0) { pc -= 3; tfm_run = true; }
}

// ---------------------------------------------------------------------------
// Opérations sur bits : BAND BIAND BOR BIOR BEOR BIEOR LDBT STBT post-octet : b7-6 registre (CC, A,
// B), b5-3 bit mémoire, b2-0 bit registre
// ---------------------------------------------------------------------------
void mc6809::bitop63(uint8_t op)
{
	bool l;
	const uint8_t im = rd8(pc++);
	const uint16_t ea = a_dir(&l);
	uint8_t m = rd8(ea);
	uint8_t dummy = m;
	uint8_t *r;
	switch (im & 0xC0) {
		case 0x00: r = &cc; break;
		case 0x40: r = &ac; break;
		case 0x80: r = &br; break;
		default:   r = &dummy; break;
	}
	const uint8_t sb = (im >> 3) & 7, db = im & 7;
	if (op == 0x37) {				// STBT : bit du registre -> mémoire
		if (*r & (1 << sb)) m |= (1 << db); else m &= ~(1 << db);
		wr8(ea, m);
		return;
	}
	const bool src = (m >> sb) & 1;
	const bool dst = (*r >> db) & 1;
	bool res;
	switch (op) {
		case 0x30: res = dst && src; break;	// BAND
		case 0x31: res = dst && !src; break;	// BIAND
		case 0x32: res = dst || src; break;	// BOR
		case 0x33: res = dst || !src; break;	// BIOR
		case 0x34: res = dst != src; break;	// BEOR
		case 0x35: res = dst == src; break;	// BIEOR
		default:   res = src; break;		// LDBT
	}
	if (res) *r |= (1 << db); else *r &= ~(1 << db);
}

// ---------------------------------------------------------------------------
// DIVD (D / octet -> B quotient, A reste) et DIVQ (Q / mot -> W quotient D reste). Drapeaux et cycles
// variables d'après MAME (rév. TJL 2026 d'après des mesures sur le vrai 6309)
// ---------------------------------------------------------------------------
bool mc6809::div63(bool quad, uint16_t divisor)
{
	int extra = 0;
	if (!quad)
	{
		const uint8_t dv8 = divisor & 0xff;
		if (dv8 == 0) { cycles -= nat ? 0 : 2; return false; }
		uint16_t t1 = (uint16_t)((ac << 8) | br);
		uint8_t t2 = dv8;
		bool ns = false, vs = false;
		if (t1 & 0x8000) { t1 = ~t1 + 1; ac = t1 >> 8; br = t1 & 0xff; ns = true; extra++; }
		if (t2 & 0x80)   { t2 = ~t2 + 1; vs = true; extra++; }
		uint16_t q = t1 / t2;
		uint8_t rm = t1 % t2;
		cc &= ~(N_FLAG | Z_FLAG | V_FLAG | C_FLAG);
		if (q >> 8) {				// débordement : registres inchangés
			cc |= V_FLAG; if (ns) cc |= N_FLAG;
			cycles += extra - 13;
			return true;
		}
		if (ns) rm = ~rm + 1;
		if (ns != vs) {
			const uint16_t nq = ~q + 1;
			if ((nq & 0x80) && !(q & 0x80)) q = nq;
		}
		ac = rm; br = q & 0xff;
		if (br & 1) cc |= C_FLAG;
		if ((((q >> 15) ^ (br >> 7)) & 1) == 0) {
			if (br == 0) cc |= Z_FLAG;
			if (br & 0x80) cc |= N_FLAG;
		} else {
			cc |= N_FLAG | V_FLAG; extra--;
		}
		cycles += extra;
		return true;
	}
	if (divisor == 0) { cycles -= nat ? 0 : 2; return false; }
	uint32_t t1 = ((uint32_t)ac << 24) | ((uint32_t)br << 16) | wr();
	uint16_t t2 = divisor;
	bool ns = false, vs = false;
	if (t1 & 0x80000000u) {
		t1 = ~t1 + 1;
		ac = t1 >> 24; br = (t1 >> 16) & 0xff; set_wr(t1 & 0xffff);
		ns = true; extra++;
	}
	if (t2 & 0x8000) { t2 = ~t2 + 1; vs = true; extra++; }
	uint32_t q = t1 / t2;
	uint16_t rm = t1 % t2;
	cc &= ~(N_FLAG | Z_FLAG | V_FLAG | C_FLAG);
	if (q >> 16) {
		cc |= V_FLAG; if (ns) cc |= N_FLAG;
		cycles += extra - 13;
		return true;
	}
	if (ns) rm = ~rm + 1;
	if (ns != vs) {
		const uint32_t nq = ~q + 1;
		if ((nq & 0x8000) && !(q & 0x8000)) q = nq;
	}
	ac = rm >> 8; br = rm & 0xff;
	set_wr(q & 0xffff);
	const uint16_t w = wr();
	if (w & 1) cc |= C_FLAG;
	if ((((q >> 31) ^ (w >> 15)) & 1) == 0) {
		if (w == 0) cc |= Z_FLAG;
		if (w & 0x8000) cc |= N_FLAG;
	} else {
		cc |= N_FLAG | V_FLAG; extra--;
	}
	cycles += extra;
	return true;
}

// ---------------------------------------------------------------------------
// Opérations 16 bits inhérentes (D, W) et 8 bits (E, F) des pages 2 et 3
// ---------------------------------------------------------------------------
static uint16_t inh16(uint8_t &cc, uint8_t lo, uint16_t t)
{
	const uint8_t c = cc & C_FLAG;
	switch (lo) {
		case 0x0: return f16(cc, NZVC, 0, t, (uint32_t)0 - t);			// NEG
		case 0x3: cc &= ~V_FLAG; cc |= C_FLAG; return f16(cc, NZ, 0, 0, (uint16_t)~t);	// COM
		case 0x4: cc = (cc & ~C_FLAG) | (t & 1); return f16(cc, NZ, 0, 0, t >> 1);	// LSR
		case 0x6: { uint16_t r = (t >> 1) | (c ? 0x8000 : 0);		// ROR
			cc = (cc & ~C_FLAG) | (t & 1); return f16(cc, NZ, 0, 0, r); }
		case 0x7: cc = (cc & ~C_FLAG) | (t & 1); return f16(cc, NZ, 0, 0, (uint16_t)((int16_t)t >> 1));	// ASR
		case 0x8: return f16(cc, NZVC, t, t, (uint32_t)t << 1);		// ASL
		case 0x9: { uint32_t r = ((uint32_t)t << 1) | (c ? 1 : 0);	// ROL (V = b15 ^ b14)
			return f16(cc, NZVC, t, t, r); }
		case 0xA: { uint8_t k = cc & C_FLAG; uint16_t r = f16(cc, NZV, t, 1, (uint32_t)t - 1); cc |= k; return r; }	// DEC
		case 0xC: { uint8_t k = cc & C_FLAG; uint16_t r = f16(cc, NZV, t, 1, (uint32_t)t + 1); cc |= k; return r; }	// INC
		case 0xD: nzv16(cc, t); return t;			// TST
		default:  cc = (cc & ~NZVC) | Z_FLAG; return 0;		// CLR
	}
}

static uint8_t inh8(uint8_t &cc, uint8_t lo, uint8_t t)
{
	switch (lo) {
		case 0x3: cc &= ~V_FLAG; cc |= C_FLAG; return f8(cc, NZ, 0, 0, (uint8_t)~t);	// COM
		case 0xA: return f8(cc, NZV, t, 1, (uint32_t)t - 1);			// DEC
		case 0xC: return f8(cc, NZV, t, 1, (uint32_t)t + 1);			// INC
		case 0xD: nzv8(cc, t); return t;				// TST
		default:  cc = (cc & ~NZVC) | Z_FLAG; return 0;			// CLR
	}
}

// opération ALU 16 bits ; k = quartet bas de l'opcode ; rend le résultat
static uint16_t alu16(uint8_t &cc, uint8_t k, uint16_t r, uint16_t m)
{
	const uint8_t c = cc & C_FLAG ? 1 : 0;
	switch (k) {
		case 0x0: case 0x1: case 0x3: case 0xC:
			return f16(cc, NZVC, r, m, (uint32_t)r - m);		// SUB / CMP
		case 0x2: return f16(cc, NZVC, r, m, (uint32_t)r - m - c);	// SBC
		case 0x4: case 0x5: cc &= ~V_FLAG; return f16(cc, NZ, 0, r, r & m);	// AND / BIT
		case 0x8: cc &= ~V_FLAG; return f16(cc, NZ, 0, r, r ^ m);	// EOR
		case 0x9: return f16(cc, NZVC, r, m, (uint32_t)r + m + c);	// ADC
		case 0xA: cc &= ~V_FLAG; return f16(cc, NZ, 0, r, r | m);	// OR
		default:  return f16(cc, NZVC, r, m, (uint32_t)r + m);		// ADD
	}
}

// ---------------------------------------------------------------------------
// Page 2 ($10 xx)
// ---------------------------------------------------------------------------
void mc6809::page2_63()
{
	const uint8_t op = rd8(pc++);
	cycles += (nat ? k_cyc63n_p2 : k_cyc63e_p2)[op];
	bool l;

	if (op >= 0x21 && op <= 0x2F) {
		const uint16_t ea = a_rew(&l);
		switch (op) {
			case 0x21: lbrn(ea); break;
			case 0x22: lbhi(ea); break;
			case 0x23: lbls(ea); break;
			case 0x24: lbhs(ea); break;
			case 0x25: lblo(ea); break;
			case 0x26: lbne(ea); break;
			case 0x27: lbeq(ea); break;
			case 0x28: lbvc(ea); break;
			case 0x29: lbvs(ea); break;
			case 0x2A: lbpl(ea); break;
			case 0x2B: lbmi(ea); break;
			case 0x2C: lbge(ea); break;
			case 0x2D: lblt(ea); break;
			case 0x2E: lbgt(ea); break;
			default:   lble(ea); break;
		}
		return;
	}
	if (op >= 0x30 && op <= 0x37) { rr63(op); return; }
	switch (op) {
		case 0x38: push_sp(fr); push_sp(er); return;			// PSHSW
		case 0x39: er = pull_sp(); fr = pull_sp(); return;		// PULSW
		case 0x3A: push_us(fr); push_us(er); return;			// PSHUW
		case 0x3B: er = pull_us(); fr = pull_us(); return;		// PULUW
		case 0x3F: swi2(0); return;
		default: break;
	}
	if (op >= 0x40 && op <= 0x5F) {
		const uint8_t lo = op & 0x0F;
		if (lo == 0x1 || lo == 0x2 || lo == 0x5 || lo == 0xB || lo == 0xE) { trap63_take(0x40); return; }
		if (op < 0x50) {
			const uint16_t r = inh16(cc, lo, (uint16_t)((ac << 8) | br));
			ac = r >> 8; br = r & 0xff;
		} else {
			set_wr(inh16(cc, lo, wr()));
		}
		return;
	}
	if (op < 0x80) { trap63_take(0x40); return; }

	const uint8_t k = op & 0x0F;
	const uint8_t row = op & 0x30;
	// opcodes valides de $80-$FF
	bool ok;
	if (op < 0xC0) ok = !(k == 0xD || (k == 0x7 && row == 0) || (k == 0xF && row == 0));
	else ok = (op == 0xCE) || (row != 0 && k >= 0xC);
	if (!ok) { trap63_take(0x40); return; }

	uint16_t ea;
	switch (row) {
		case 0x00: ea = pc; pc += 2; break;
		case 0x10: ea = a_dir(&l); break;
		case 0x20: ea = a_idx(&l); break;
		default:   ea = a_ext(&l); break;
	}

	if (op >= 0xC0) {
		switch (k) {
			case 0xC: {							// LDQ
				ac = rd8(ea); br = rd8(ea + 1); er = rd8(ea + 2); fr = rd8(ea + 3);
				fq(cc, ((uint32_t)ac << 24) | ((uint32_t)br << 16) | wr());
				return; }
			case 0xD:							// STQ
				wr8(ea, ac); wr8(ea + 1, br); wr8(ea + 2, er); wr8(ea + 3, fr);
				fq(cc, ((uint32_t)ac << 24) | ((uint32_t)br << 16) | wr());
				return;
			case 0xE: sp = nzv16(cc, rd16(ea)); nmi_enabled = true; return;	// LDS
			default:  wr16(ea, sp); nzv16(cc, sp); return;		// STS
		}
	}
	switch (k) {
		case 0x6: set_wr(nzv16(cc, rd16(ea))); return;		// LDW
		case 0x7: wr16(ea, wr()); nzv16(cc, wr()); return;	// STW
		case 0xE: yr = nzv16(cc, rd16(ea)); return;		// LDY
		case 0xF: wr16(ea, yr); nzv16(cc, yr); return;		// STY
		default: break;
	}
	const uint16_t m = rd16(ea);
	switch (k) {
		case 0x0: case 0xB: set_wr(alu16(cc, k, wr(), m)); return;	// SUBW ADDW
		case 0x1: alu16(cc, k, wr(), m); return;				// CMPW
		case 0x3: alu16(cc, k, (uint16_t)((ac << 8) | br), m); return;	// CMPD
		case 0x5: alu16(cc, k, (uint16_t)((ac << 8) | br), m); return;	// BITD
		case 0xC: alu16(cc, k, yr, m); return;				// CMPY
		default: {							// SBCD ANDD EORD ADCD ORD
			const uint16_t r = alu16(cc, k, (uint16_t)((ac << 8) | br), m);
			ac = r >> 8; br = r & 0xff;
			return; }
	}
}

// ---------------------------------------------------------------------------
// Page 3 ($11 xx)
// ---------------------------------------------------------------------------
void mc6809::page3_63()
{
	const uint8_t op = rd8(pc++);
	cycles += (nat ? k_cyc63n_p3 : k_cyc63e_p3)[op];
	bool l;

	if (op >= 0x30 && op <= 0x37) { bitop63(op); return; }
	if (op >= 0x38 && op <= 0x3B) { tfm63(op); return; }
	switch (op) {
		case 0x3C: {							// BITMD
			const uint8_t im = rd8(pc++) & 0xC0;
			if (md & im) cc &= ~Z_FLAG; else cc |= Z_FLAG;
			md &= ~im;
			return; }
		case 0x3D: set_md(rd8(pc++)); return;				// LDMD
		case 0x3F: swi3(0); return;
		default: break;
	}
	if (op >= 0x40 && op <= 0x5F) {
		const uint8_t lo = op & 0x0F;
		if (!(lo == 0x3 || lo == 0xA || lo == 0xC || lo == 0xD || lo == 0xF)) { trap63_take(0x40); return; }
		if (op < 0x50) er = inh8(cc, lo, er); else fr = inh8(cc, lo, fr);
		return;
	}
	if (op < 0x80) { trap63_take(0x40); return; }

	const uint8_t k = op & 0x0F;
	const uint8_t row = op & 0x30;
	bool ok;
	if (op < 0xC0) ok = (k <= 1 || k == 3 || k == 6 || k == 0xB || k >= 0xC) || (k == 7 && row != 0);
	else ok = (k <= 1 || k == 6 || k == 0xB) || (k == 7 && row != 0);
	if (!ok) { trap63_take(0x40); return; }

	const bool w16 = (op < 0xC0) && (k == 3 || k == 0xC || k == 0xE || k == 0xF);
	uint16_t ea;
	switch (row) {
		case 0x00: ea = pc; pc += w16 ? 2 : 1; break;
		case 0x10: ea = a_dir(&l); break;
		case 0x20: ea = a_idx(&l); break;
		default:   ea = a_ext(&l); break;
	}

	if (w16) {
		const uint16_t m = rd16(ea);
		switch (k) {
			case 0x3: alu16(cc, 0x3, us, m); return;			// CMPU
			case 0xC: alu16(cc, 0x3, sp, m); return;			// CMPS
			case 0xE: if (!div63(true, m)) trap63_take(0x80); return;	// DIVQ
			default: {							// MULD
				const uint32_t r = (uint32_t)((int32_t)(int16_t)((ac << 8) | br) * (int32_t)(int16_t)m);
				ac = r >> 24; br = (r >> 16) & 0xff; set_wr(r & 0xffff);
				cc &= ~(N_FLAG | Z_FLAG | V_FLAG | C_FLAG);
				if (r & 0x80000000u) cc |= N_FLAG;
				if (r == 0) cc |= Z_FLAG;
				return; }
		}
	}
	if (k == 0xD) { if (!div63(false, rd8(ea))) trap63_take(0x80); return; }	// DIVD

	uint8_t &r = (op < 0xC0) ? er : fr;
	switch (k) {
		case 0x6: r = nzv8(cc, rd8(ea)); return;			// LDE / LDF
		case 0x7: wr8(ea, r); nzv8(cc, r); return;			// STE / STF
		default: break;
	}
	const uint8_t m = rd8(ea);
	switch (k) {
		case 0x0: r = f8(cc, NZVC, r, m, (uint32_t)r - m); return;		// SUBE/F
		case 0x1: f8(cc, NZVC, r, m, (uint32_t)r - m); return;		// CMPE/F
		default:  r = f8(cc, H_FLAG | NZVC, r, m, (uint32_t)r + m); return;	// ADDE/F
	}
}

#endif // MC6809_INCLUDE_HD6309
