/*
 * mc6809.hpp  -  part of MC6809
 *
 * (C)2021-2025 elmerucr
 */

// Update: Luc Dion 2026-10-26.

#ifndef MC6809_HPP
#define MC6809_HPP

// messages [MC6809] du démarrage masqués en version bêta (voir main.h)
#include "boot_log.h"   // réglage BOOT_LOG_VERBOSE
#include <cstdint>
#include <cstddef>

#define MC6809_MAJOR_VERSION	0
#define MC6809_MINOR_VERSION	16
#define MC6809_BUILD		20250405
#define MC6809_YEAR		2025

#define	C_FLAG	0x01	// carry
#define	V_FLAG	0x02	// overflow
#define	Z_FLAG	0x04	// zero
#define	N_FLAG	0x08	// negative
#define	I_FLAG	0x10	// irq
#define	H_FLAG	0x20	// half carry
#define	F_FLAG	0x40	// firq
#define	E_FLAG	0x80	// entire state on stack

#define	VECTOR_ILL_OPC	0xfff0	// originally from 6309
#define	VECTOR_SWI3	0xfff2
#define	VECTOR_SWI2	0xfff4
#define	VECTOR_FIRQ	0xfff6
#define	VECTOR_IRQ	0xfff8
#define	VECTOR_SWI	0xfffa
#define	VECTOR_NMI	0xfffc
#define	VECTOR_RESET	0xfffe

#define SYNC_CYCLES	50
#define CWAI_CYCLES	50

enum cpu_state_t {
	CPU_NORMAL = 0,
	CPU_CWAI,
	CPU_SYNC
};

const char cpu_state_description[3][5] = {
	"run",
	"halt",
	"halt"
};

#ifndef MC6809_NO_FAST_BUS
// Tables de pages du GIME (gime.cpp) : 8 tranches de 8 Ko ($0000-$FFFF).
extern uint8_t *gime_rd[9];
extern uint8_t *gime_wr[9];
#define mc6809_page_rd gime_rd
#define mc6809_page_wr gime_wr
#endif

// budget en cycles avant le prochain évènement de l'émulateur (nullptr ou 0 = pas d'enchaînement de
// NOP).
extern uint32_t (*mc6809_nop_budget)(void);

class mc6809 {
public:
	mc6809();
	virtual ~mc6809();

	virtual uint8_t read8(uint16_t address) const = 0;
	virtual void write8(uint16_t address, uint8_t value) const = 0;

#ifndef MC6809_NO_FAST_BUS
	inline __attribute__((always_inline)) uint8_t rd8(uint16_t a) const
	{
		if (a < 0xFE00) return mc6809_page_rd[a >> 13][a & 0x1FFF];
		return read8(a);
	}
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"   // petits opcodes mis en ligne dans execute
	inline __attribute__((always_inline)) void wr8(uint16_t a, uint8_t v) const
	{
		if (a < 0xFE00) { mc6809_page_wr[a >> 13][a & 0x1FFF] = v; return; }
		write8(a, v);
	}
#else
	inline uint8_t rd8(uint16_t a) const { return read8(a); }
	inline void wr8(uint16_t a, uint8_t v) const { write8(a, v); }
#endif

	// Assignment of the different interrupt lines. The constructor of the cpu class creates true values
	// (level up) by default - so if none is assigned, the cpu will still work.
	public:
	void assign_nmi_line(bool *line) { nmi_line = line; }
	void assign_firq_line(bool *line) { firq_line = line; }
	void assign_irq_line(bool *line) { irq_line = line; }

	// Reset exception. Doesn't emulate the number of cycles taken.
	void reset();

	// Main execute function. Runs one instruction, and returns the number of cycles consumed. Checking
	// for breakpoints must be done with the breakpoint member function that returns true or false.
	uint16_t execute();

	void status(char *text_buffer, int n);
	void stacks(char *text_buffer, int n, int no);
	uint16_t disassemble_instruction(char *buffer, size_t n, uint16_t address);
	bool disassemble_successfull() { return disassemble_success; }

	inline bool is_e_flag_set()   { return (cc & E_FLAG) ? true  : false; }
	inline bool is_e_flag_clear() { return (cc & E_FLAG) ? false : true ; }
	inline bool is_f_flag_set()   { return (cc & F_FLAG) ? true  : false; }
	inline bool is_f_flag_clear() { return (cc & F_FLAG) ? false : true ; }
	inline bool is_h_flag_set()   { return (cc & H_FLAG) ? true  : false; }
	inline bool is_h_flag_clear() { return (cc & H_FLAG) ? false : true ; }
	inline bool is_i_flag_set()   { return (cc & I_FLAG) ? true  : false; }
	inline bool is_i_flag_clear() { return (cc & I_FLAG) ? false : true ; }
	inline bool is_n_flag_set()   { return (cc & N_FLAG) ? true  : false; }
	inline bool is_n_flag_clear() { return (cc & N_FLAG) ? false : true ; }
	inline bool is_z_flag_set()   { return (cc & Z_FLAG) ? true  : false; }
	inline bool is_z_flag_clear() { return (cc & Z_FLAG) ? false : true ; }
	inline bool is_v_flag_set()   { return (cc & V_FLAG) ? true  : false; }
	inline bool is_v_flag_clear() { return (cc & V_FLAG) ? false : true ; }
	inline bool is_c_flag_set()   { return (cc & C_FLAG) ? true  : false; }
	inline bool is_c_flag_clear() { return (cc & C_FLAG) ? false : true ; }

	inline void set_e_flag()   { cc |= E_FLAG; }
	inline void clear_e_flag() { cc &= (0xff - E_FLAG); }
	inline void set_f_flag()   { cc |= F_FLAG; }
	inline void clear_f_flag() { cc &= (0xff - F_FLAG); }
	inline void set_h_flag()   { cc |= H_FLAG; }
	inline void clear_h_flag() { cc &= (0xff - H_FLAG); }
	inline void set_i_flag()   { cc |= I_FLAG; }
	inline void clear_i_flag() { cc &= (0xff - I_FLAG); }
	inline void set_n_flag()   { cc |= N_FLAG; }
	inline void clear_n_flag() { cc &= (0xff - N_FLAG); }
	inline void set_z_flag()   { cc |= Z_FLAG; }
	inline void clear_z_flag() { cc &= (0xff - Z_FLAG); }
	inline void set_v_flag()   { cc |= V_FLAG; }
	inline void clear_v_flag() { cc &= (0xff - V_FLAG); }
	inline void set_c_flag()   { cc |= C_FLAG; }
	inline void clear_c_flag() { cc &= (0xff - C_FLAG); }

	inline __attribute__((always_inline)) void test_n_flag(uint8_t byte) { if (byte &  0x80) set_n_flag(); else clear_n_flag(); }
	inline __attribute__((always_inline)) void test_z_flag(uint8_t byte) { if (byte == 0x00) set_z_flag(); else clear_z_flag(); }
	inline __attribute__((always_inline)) void test_nz_flags(uint8_t byte) { test_n_flag(byte); test_z_flag(byte); }
	inline __attribute__((always_inline)) void test_n_flag_16(uint16_t word) { if (word &  0x8000) set_n_flag(); else clear_n_flag(); }
	inline __attribute__((always_inline)) void test_z_flag_16(uint16_t word) { if (word == 0x0000) set_z_flag(); else clear_z_flag(); }
	inline __attribute__((always_inline)) void test_nz_flags_16(uint16_t word) { test_n_flag_16(word); test_z_flag_16(word); }

	// getters and setters, useful while debugging TODO: what about flags here?
	uint16_t get_pc()              { return pc; }
	void     set_pc(uint16_t word) { pc = word; }
	uint8_t  get_dp()              { return dp; }
	void     set_dp(uint8_t  byte) { dp = byte; }
	uint8_t  get_ac()              { return ac; }
	void     set_ac(uint8_t  byte) { ac = byte; }
	uint8_t  get_br()              { return br; }
	void     set_br(uint8_t  byte) { br = byte; }
	uint16_t get_dr()              { return (ac << 8) | br; }
	void     set_dr(uint16_t word) { ac = (word & 0xff00) >> 8; br = word & 0x00; }
	uint16_t get_xr()              { return xr; }
	void     set_xr(uint16_t word) { xr = word; }
	uint16_t get_yr()              { return yr; }
	void     set_yr(uint16_t word) { yr = word; }
	uint16_t get_us()              { return us; }
	void     set_us(uint16_t word) { us = word; }
	uint16_t get_sp()              { return sp; }
	void     set_sp(uint16_t word) { sp = word; }
	uint8_t  get_cc()              { return cc; }
	void     set_cc(uint8_t  byte) { cc = byte; }

	bool *breakpoint_array;
	inline bool breakpoint() { return breakpoint_array[pc] ? true : false; }
	void toggle_breakpoint(uint16_t address);
	void clear_breakpoints();

	inline uint32_t clock_ticks() { return cycles; }

	void set_cpu_6309(bool on);
	bool is_6309() const { return h63; }
	bool is_native() const { return nat; }
	uint8_t  get_er() const { return er; }
	uint8_t  get_fr() const { return fr; }
	uint16_t get_vr() const { return vr; }
	uint8_t  get_md() const { return md; }
	void set_er(uint8_t v) { er = v; }		// accès de test / débogage
	void set_fr(uint8_t v) { fr = v; }
	void set_vr(uint16_t v) { vr = v; }
	void set_md_full(uint8_t v) { md = v; set_md(v); }
	bool tfm_pending() const { return tfm_run; }
	// Save State (registres et état interne du processeur)
	struct snapshot_t {
		uint16_t pc, xr, yr, us, sp, vr;
		uint8_t  dp, ac, br, cc, er, fr, md, state;
		uint8_t  cwai_stacked, h63, trap63, tfm_run, nmi_enabled, old_nmi, old_firq, old_irq;
		int32_t  cycle_saldo;
		uint32_t cycles;
		uint8_t  snap63[24];
	};
	void save_state(snapshot_t &s) const;
	void load_state(const snapshot_t &s);
	uint32_t traps63 = 0;		// pièges $FFF0 pris (diagnostic, W)
	uint16_t trap_pc63 = 0;		// PC empilé du dernier piège

private:
	uint16_t pc;	// program counter
	uint8_t	 dp;	// direct page register
	uint8_t  ac;	// accumulator a, msb of d
	uint8_t  br;	// accumulator b, lsb of d
	// uint16_t dr; // non-existing = virtual as a combi of ac and br
	uint16_t xr;	// x index register
	uint16_t yr;	// y index register
	uint16_t us;	// user stack pointer
	uint16_t sp;	// hardware stack pointer
	uint8_t  cc;	// condition code register

	enum cpu_state_t cpu_state;
	bool cwai_stacked;	// CWAI a déjà empilé l'état complet

	// ---- HD6309
	bool     h63;		// jeu d'instructions 6309 actif
	bool     nat;		// mode natif (MD bit 0), toujours faux si !h63
	uint8_t  lbx;		// cycle de plus d'une branche longue prise (1 ; 0 en natif)
	uint8_t  er, fr;	// E, F (W = E:F ; Q = D:W)
	uint16_t vr;		// V
	uint8_t  md;		// MD : b0 natif, b1 FIRQ = IRQ, b6 opcode illégal, b7 division par 0
	bool     trap63;	// post-octet indexé illégal : piège à la fin de l'instruction
	bool     tfm_run;	// TFM interrompu en cours de route (reprise sans cycles de base)
	const uint8_t *cyc1;	// cycles de la page 1 (6809, 6309 émulation ou natif)
	struct { uint16_t pc, xr, yr, us, sp; uint8_t ac, br, er, fr, cc, dp; } snap63;
	inline uint16_t wr() const { return (uint16_t)((er << 8) | fr); }
	inline void set_wr(uint16_t v) { er = v >> 8; fr = v & 0xff; }
	void set_md(uint8_t v);
	void trap63_take(uint8_t mdbit);	// piège $FFF0 (b6 illégal, b7 div. par 0)
	uint16_t a_idx63(uint8_t postbyte, uint16_t *ireg);
	uint16_t read_tfr63(uint8_t r);
	void write_tfr63(uint8_t r, uint16_t v);
	void im63(uint8_t kind, uint8_t im, uint16_t ea);	// OIM AIM EIM TIM
	void sexw63();
	void ldq_imm63();
	void page2_63();
	void page3_63();
	void rr63(uint8_t op);		// ADDR ... CMPR
	void tfm63(uint8_t op);
	void bitop63(uint8_t op);
	bool div63(bool quad, uint16_t divisor);	// false = division par 0
	uint16_t rd16(uint16_t a) { return (uint16_t)((rd8(a) << 8) | rd8((uint16_t)(a + 1))); }
	void wr16(uint16_t a, uint16_t v) { wr8(a, v >> 8); wr8((uint16_t)(a + 1), v & 0xff); }

	uint16_t *index_regs[4];

	bool nmi_enabled;
	bool default_pin;

	bool *nmi_line;
	bool old_nmi_line;
	bool *firq_line;
	bool old_firq_line;
	bool *irq_line;
	bool old_irq_line;

	int32_t cycle_saldo;
	uint32_t cycles;

	typedef uint16_t (mc6809::*addressing_mode)(bool *legal);
	typedef void (mc6809::*execute_instruction)(uint16_t);

	bool disassemble_success;

	// Exception functions are private. Exceptions are triggered indirectly by the execute function that
	// polls the different interrupt lines or detects an illegal opcode.
	void nmi();
	void firq();
	void irq();
	void illegal_opcode();

	// Internal stackpointer functionality
	inline __attribute__((always_inline)) void    push_sp(uint8_t byte) { wr8(--sp, byte); }
	inline __attribute__((always_inline)) uint8_t pull_sp()             { return rd8(sp++); }
	inline __attribute__((always_inline)) void    push_us(uint8_t byte) { wr8(--us, byte); }
	inline __attribute__((always_inline)) uint8_t pull_us()             { return rd8(us++); }

	// état complet (E déjà réglé par l'appelant) ; en mode natif 6309 F et E s'ajoutent entre DP et B (2
	// cycles de plus).
	inline __attribute__((always_inline)) void push_all()
	{
		push_sp(pc & 0x00ff);
		push_sp((pc & 0xff00) >> 8);
		push_sp(us & 0x00ff);
		push_sp((us & 0xff00) >> 8);
		push_sp(yr & 0x00ff);
		push_sp((yr & 0xff00) >> 8);
		push_sp(xr & 0x00ff);
		push_sp((xr & 0xff00) >> 8);
		push_sp(dp);
		if (__builtin_expect(nat, 0)) { push_sp(fr); push_sp(er); cycles += 2; }
		push_sp(br);
		push_sp(ac);
		push_sp(cc);
	}

	// addressing modes
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"   // always_inline sur fonctions non « inline » (voulu)
	__attribute__((always_inline)) uint16_t a_dir(bool *legal);   // mis en ligne dans execute // direct page (base page)
	__attribute__((always_inline)) uint16_t a_imb(bool *legal);   // mis en ligne dans execute // immediate byte
	__attribute__((always_inline)) uint16_t a_imw(bool *legal);   // mis en ligne dans execute // immediate word
	__attribute__((always_inline)) uint16_t a_ih(bool *legal);   // mis en ligne dans execute // inherent
	__attribute__((always_inline)) uint16_t a_reb(bool *legal);   // mis en ligne dans execute // relative byte (8 bit signed)
	__attribute__((always_inline)) uint16_t a_rew(bool *legal);   // mis en ligne dans execute // relative word (16 bit signed)
	uint16_t a_idx(bool *legal);	// indexed
	__attribute__((always_inline)) uint16_t a_ext(bool *legal);   // mis en ligne dans execute // extended
	__attribute__((always_inline)) uint16_t a_no(bool *legal);   // mis en ligne dans execute // no mode (@ illegal instructions)
#pragma GCC diagnostic pop

	// instructions
	__attribute__((always_inline)) void abx(uint16_t ea);
	void adca(uint16_t ea);
	void adcb(uint16_t ea);
	void adda(uint16_t ea);
	void addb(uint16_t ea);
	void addd(uint16_t ea);
	__attribute__((always_inline)) void anda(uint16_t ea);
	__attribute__((always_inline)) void andb(uint16_t ea);

	__attribute__((always_inline)) void andcc(uint16_t ea);
	void asl(uint16_t ea);		// also lsl
	__attribute__((always_inline)) void asla(uint16_t ea);
	__attribute__((always_inline)) void aslb(uint16_t ea);
	void asr(uint16_t ea);
	__attribute__((always_inline)) void asra(uint16_t ea);
	__attribute__((always_inline)) void asrb(uint16_t ea);
	__attribute__((always_inline)) void beq(uint16_t ea);

	__attribute__((always_inline)) void bge(uint16_t ea);
	__attribute__((always_inline)) void bgt(uint16_t ea);
	__attribute__((always_inline)) void bhi(uint16_t ea);
	__attribute__((always_inline)) void bhs(uint16_t ea);		// bcc
	__attribute__((always_inline)) void bita(uint16_t ea);
	__attribute__((always_inline)) void bitb(uint16_t ea);
	__attribute__((always_inline)) void bmi(uint16_t ea);
	__attribute__((always_inline)) void ble(uint16_t ea);

	__attribute__((always_inline)) void blo(uint16_t ea);		// bcs
	__attribute__((always_inline)) void bls(uint16_t ea);
	__attribute__((always_inline)) void blt(uint16_t ea);
	__attribute__((always_inline)) void bne(uint16_t ea);
	__attribute__((always_inline)) void bpl(uint16_t ea);
	__attribute__((always_inline)) void bra(uint16_t ea);
	__attribute__((always_inline)) void brn(uint16_t ea);
	void bsr(uint16_t ea);

	__attribute__((always_inline)) void bvc(uint16_t ea);
	__attribute__((always_inline)) void bvs(uint16_t ea);
	__attribute__((always_inline)) void clr(uint16_t ea);
	__attribute__((always_inline)) void clra(uint16_t ea);
	__attribute__((always_inline)) void clrb(uint16_t ea);
	void cmpa(uint16_t ea);
	void cmpb(uint16_t ea);
	void cmpd(uint16_t ea);

	void cmps(uint16_t ea);
	void cmpu(uint16_t ea);
	void cmpx(uint16_t ea);
	void cmpy(uint16_t ea);
	void com(uint16_t ea);
	__attribute__((always_inline)) void coma(uint16_t ea);
	__attribute__((always_inline)) void comb(uint16_t ea);
	void cwai(uint16_t ea);

	void daa(uint16_t ea);
	void dec(uint16_t ea);
	__attribute__((always_inline)) void deca(uint16_t ea);
	__attribute__((always_inline)) void decb(uint16_t ea);
	__attribute__((always_inline)) void eora(uint16_t ea);
	__attribute__((always_inline)) void eorb(uint16_t ea);
	__attribute__((always_inline)) void exg(uint16_t ea);
	__attribute__((always_inline)) void ill(uint16_t ea);		// illegal opcode (from 6309)
	// opcodes non documentés du vrai 6809 (comme MAME)
	__attribute__((always_inline)) void negcom(uint16_t ea);	// $02/$62/$72 : C=1 -> COM, sinon NEG
	void negcoma(uint16_t ea);	// $42
	void negcomb(uint16_t ea);	// $52
	__attribute__((always_inline)) void st_imm_a(uint16_t ea);	// $87 « STA # » : drapeaux seulement
	__attribute__((always_inline)) void st_imm_b(uint16_t ea);	// $C7
	__attribute__((always_inline)) void st_imm_x(uint16_t ea);	// $8F
	__attribute__((always_inline)) void st_imm_u(uint16_t ea);	// $CF
	void xres(uint16_t ea);		// $3E : « RESET » logiciel (vecteur $FFFE)
	uint16_t read_tfr_reg(uint8_t r);
	void write_tfr_reg(uint8_t r, uint16_t v);

	void inc(uint16_t ea);
	__attribute__((always_inline)) void inca(uint16_t ea);
	__attribute__((always_inline)) void incb(uint16_t ea);
	__attribute__((always_inline)) void jmp(uint16_t ea);
	__attribute__((always_inline)) void jsr(uint16_t ea);
	void lbeq(uint16_t ea);
	void lbge(uint16_t ea);
	void lbgt(uint16_t ea);

	void lbhi(uint16_t ea);
	void lbhs(uint16_t ea);		// lbcc
	void lble(uint16_t ea);
	void lblo(uint16_t ea);		// lbcs
	void lbls(uint16_t ea);
	void lblt(uint16_t ea);
	void lbmi(uint16_t ea);
	void lbne(uint16_t ea);

	void lbpl(uint16_t ea);
	__attribute__((always_inline)) void lbra(uint16_t ea);
	void lbrn(uint16_t ea);
	__attribute__((always_inline)) void lbsr(uint16_t ea);
	void lbvc(uint16_t ea);
	void lbvs(uint16_t ea);
	__attribute__((always_inline)) void lda(uint16_t ea);
	__attribute__((always_inline)) void ldb(uint16_t ea);

	void ldd(uint16_t ea);
	void lds(uint16_t ea);
	void ldu(uint16_t ea);
	void ldx(uint16_t ea);
	void ldy(uint16_t ea);
	__attribute__((always_inline)) void leas(uint16_t ea);
	__attribute__((always_inline)) void leau(uint16_t ea);
	__attribute__((always_inline)) void leax(uint16_t ea);

	__attribute__((always_inline)) void leay(uint16_t ea);
	void lsr(uint16_t ea);
	__attribute__((always_inline)) void lsra(uint16_t ea);
	__attribute__((always_inline)) void lsrb(uint16_t ea);
	__attribute__((always_inline)) void mul(uint16_t ea);
	void neg(uint16_t ea);
	__attribute__((always_inline)) void nega(uint16_t ea);
	__attribute__((always_inline)) void negb(uint16_t ea);

	__attribute__((always_inline)) void nop(uint16_t ea);
	__attribute__((always_inline)) void ora(uint16_t ea);
	__attribute__((always_inline)) void orb(uint16_t ea);
	__attribute__((always_inline)) void orcc(uint16_t ea);

	void page2(uint16_t ea);
	void page3(uint16_t ea);

	void pshs(uint16_t ea);
	void pshu(uint16_t ea);
	void puls(uint16_t ea);
	void pulu(uint16_t ea);

	void rol(uint16_t ea);
	__attribute__((always_inline)) void rola(uint16_t ea);
	__attribute__((always_inline)) void rolb(uint16_t ea);
	void ror(uint16_t ea);
	__attribute__((always_inline)) void rora(uint16_t ea);
	__attribute__((always_inline)) void rorb(uint16_t ea);
	void rti(uint16_t ea);
	void rts(uint16_t ea);

	void sbca(uint16_t ea);
	void sbcb(uint16_t ea);
	__attribute__((always_inline)) void sex(uint16_t ea);
	__attribute__((always_inline)) void sta(uint16_t ea);
	__attribute__((always_inline)) void stb(uint16_t ea);
	void std(uint16_t ea);
	void sts(uint16_t ea);
	void stu(uint16_t ea);

	void stx(uint16_t ea);
	void sty(uint16_t ea);
	void suba(uint16_t ea);
	void subb(uint16_t ea);
	void subd(uint16_t ea);
	void swi(uint16_t ea);
	void swi2(uint16_t ea);
	void swi3(uint16_t ea);

	__attribute__((always_inline)) void sync(uint16_t ea);
	__attribute__((always_inline)) void tfr(uint16_t ea);
	__attribute__((always_inline)) void tst(uint16_t ea);
	__attribute__((always_inline)) void tsta(uint16_t ea);
	__attribute__((always_inline)) void tstb(uint16_t ea);
#pragma GCC diagnostic pop

private:
	// table statique (flash), plus dans l'objet CPU en RAM interne utilisée seulement par le repli
	// $10/$11 + opcode inconnu du 6809.
	static const execute_instruction opcodes_page1[256];

	const execute_instruction opcodes_page2[256] = {
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x00
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x10
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::lbrn,	&mc6809::lbhi,	&mc6809::lbls,	&mc6809::lbhs,	&mc6809::lblo,	&mc6809::lbne,	&mc6809::lbeq,	// 0x20
		&mc6809::lbvc,	&mc6809::lbvs,	&mc6809::lbpl,	&mc6809::lbmi,	&mc6809::lbge,	&mc6809::lblt,	&mc6809::lbgt,	&mc6809::lble,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x30
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::swi2,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x40
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x50
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x60
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x70
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmpd,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x80
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmpy,	&mc6809::ill,	&mc6809::ldy,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmpd,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x90
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmpy,	&mc6809::ill,	&mc6809::ldy,	&mc6809::sty,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmpd,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0xa0
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmpy,	&mc6809::ill,	&mc6809::ldy,	&mc6809::sty,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmpd,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0xb0
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmpy,	&mc6809::ill,	&mc6809::ldy,	&mc6809::sty,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0xc0
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::lds,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0xd0
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::lds,	&mc6809::sts,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0xe0
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::lds,	&mc6809::sts,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0xf0
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::lds,	&mc6809::sts
	};

	const execute_instruction opcodes_page3[256] = {
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x00
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x10
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x20
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x30
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::swi3,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x40
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x50
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x60
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x70
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmpu,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x80
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmps,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmpu,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0x90
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmps,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmpu,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0xa0
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmps,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmpu,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0xb0
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::cmps,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0xc0
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0xd0
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0xe0
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	// 0xf0
		&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill,	&mc6809::ill
	};

	// table statique (flash), plus dans l'objet CPU en RAM interne utilisée seulement par le repli
	// $10/$11 + opcode inconnu du 6809.
	static const addressing_mode addressing_modes_page1[256];

	const addressing_mode addressing_modes_page2[256] = {
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x00
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x10
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_rew,	&mc6809::a_rew,	&mc6809::a_rew,	&mc6809::a_rew,	&mc6809::a_rew,	&mc6809::a_rew,	&mc6809::a_rew,	// 0x20
		&mc6809::a_rew,	&mc6809::a_rew,	&mc6809::a_rew,	&mc6809::a_rew,	&mc6809::a_rew,	&mc6809::a_rew,	&mc6809::a_rew,	&mc6809::a_rew,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x30
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_ih,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x40
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x50
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x60
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x70
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_imw,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x80
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_imw,	&mc6809::a_no,	&mc6809::a_imw,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_dir,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x90
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_dir,	&mc6809::a_no,	&mc6809::a_dir,	&mc6809::a_dir,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_idx,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0xa0
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_idx,	&mc6809::a_no,	&mc6809::a_idx,	&mc6809::a_idx,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_ext,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0xb0
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_ext,	&mc6809::a_no,	&mc6809::a_ext,	&mc6809::a_ext,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0xc0
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_imw,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0xd0
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_dir,	&mc6809::a_dir,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0xe0
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_idx,	&mc6809::a_idx,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0xf0
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_ext,	&mc6809::a_ext
	};

	const addressing_mode addressing_modes_page3[256] = {
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x00
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x10
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x20
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x30
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_ih,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x40
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x50
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x60
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x70
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_imw,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x80
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_imw,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_dir,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0x90
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_dir,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_idx,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0xa0
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_idx,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_ext,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0xb0
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_ext,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0xc0
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0xd0
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0xe0
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	// 0xf0
		&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no,	&mc6809::a_no
	};

	const uint8_t cycles_page1[256] = {
		 6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  3,  6,	// 0x00
		 0,  0,  2,  4,  0,  0,  5,  9,  3,  2,  3,  2,  3,  2,  8,  6,	// 0x10
		 3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,	// 0x20
		 4,  4,  4,  4,  5,  5,  5,  5,  3,  5,  3,  6, 20, 11,  19, 19,	// 0x30
		 2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,	// 0x40
		 2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,	// 0x50
		 6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  3,  6,	// 0x60
		 7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  4,  7,	// 0x70
		 2,  2,  2,  4,  2,  2,  2,  2,  2,  2,  2,  2,  4,  7,  3,  3,	// 0x80
		 4,  4,  4,  6,  4,  4,  4,  4,  4,  4,  4,  4,  6,  7,  5,  5,	// 0x90
		 4,  4,  4,  6,  4,  4,  4,  4,  4,  4,  4,  4,  6,  7,  5,  5,	// 0xa0
		 5,  5,  5,  7,  5,  5,  5,  5,  5,  5,  5,  5,  7,  8,  6,  6,	// 0xb0
		 2,  2,  2,  4,  2,  2,  2,  2,  2,  2,  2,  2,  3,  0,  3,  3,	// 0xc0
		 4,  4,  4,  6,  4,  4,  4,  4,  4,  4,  4,  4,  5,  5,  5,  5,	// 0xd0
		 4,  4,  4,  6,  4,  4,  4,  4,  4,  4,  4,  4,  5,  5,  5,  5,	// 0xe0
		 5,  5,  5,  7,  5,  5,  5,  5,  5,  5,  5,  5,  6,  6,  6,  6	// 0xf0
	};

	const uint8_t cycles_page2[256] = {
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0x00
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0x10
		 0,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,	// 0x20
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 20,	// 0x30
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0x40
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0x50
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0x60
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0x70
		 0,  0,  0,  5,  0,  0,  0,  0,  0,  0,  0,  0,  5,  0,  4,  0,	// 0x80
		 0,  0,  0,  7,  0,  0,  0,  0,  0,  0,  0,  0,  7,  0,  6,  6,	// 0x90
		 0,  0,  0,  7,  0,  0,  0,  0,  0,  0,  0,  0,  7,  0,  6,  6,	// 0xa0
		 0,  0,  0,  8,  0,  0,  0,  0,  0,  0,  0,  0,  8,  0,  7,  7,	// 0xb0
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  4,  0,	// 0xc0
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  6,  6,	// 0xd0
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  6,  6,	// 0xe0
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  7,  7	// 0xf0
	};

	const uint8_t cycles_page3[256] = {
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0x00
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0x10
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0x20
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 20,	// 0x30
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0x40
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0x50
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0x60
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0x70
		 0,  0,  0,  5,  0,  0,  0,  0,  0,  0,  0,  0,  5,  0,  0,  0,	// 0x80
		 0,  0,  0,  7,  0,  0,  0,  0,  0,  0,  0,  0,  7,  0,  0,  0,	// 0x90
		 0,  0,  0,  7,  0,  0,  0,  0,  0,  0,  0,  0,  7,  0,  0,  0,	// 0xa0
		 0,  0,  0,  8,  0,  0,  0,  0,  0,  0,  0,  0,  8,  0,  0,  0,	// 0xb0
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0xc0
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0xd0
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,	// 0xe0
		 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0	// 0xf0
	};
};

#endif
