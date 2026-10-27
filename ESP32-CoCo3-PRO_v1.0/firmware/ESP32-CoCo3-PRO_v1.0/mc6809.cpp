// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

/*
 * mc6809.cpp  -  part of MC6809
 *
 * (C)2021-2025 elmerucr
 *
 *
MIT License

Copyright (c) 2021-2025 elmerucr

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include <Arduino.h>
// l'IDE Arduino compile le reste en -Os. En -Os, rd8/wr8, push/pull et les tests de drapeaux n'étaient
// PAS mis en ligne : chaque accès mémoire devenait un appel vers une copie de la fonction
#pragma GCC optimize ("O2")
 #include "mc6809.hpp"
#include <cstdio>

mc6809::mc6809()
{
	cc = 0b00000000;

	// When NFI pins are not (yet) assigned, there needs to be a decent starting value (true).
	default_pin = true;
	nmi_line = &default_pin;
	firq_line = &default_pin;
	irq_line = &default_pin;

	cycles = 0;

	// MC6809 par défaut
	h63 = false; nat = false; lbx = 1;
	er = fr = 0; vr = 0xffff; md = 0;
	trap63 = false; tfm_run = false;
	cyc1 = cycles_page1;

	index_regs[0b00] = &xr;
	index_regs[0b01] = &yr;
	index_regs[0b10] = &us;
	index_regs[0b11] = &sp;

	breakpoint_array = NULL;
	breakpoint_array = new bool[65536];
	clear_breakpoints();

	if (BOOT_LOG_VERBOSE) printf("[MC6809] version %i.%i.%i (C)%i elmerucr\n",
	       MC6809_MAJOR_VERSION,
	       MC6809_MINOR_VERSION,
	       MC6809_BUILD,
	       MC6809_YEAR);
}

mc6809::~mc6809()
{
	if (BOOT_LOG_VERBOSE) printf("[MC6809] cleaning up\n");
	delete breakpoint_array;
}

void mc6809::reset()
{
	if (BOOT_LOG_VERBOSE) printf("[MC6809] resetting cpu\n");
	// For 6800 compatibility, direct page register defaults to zero after a reset.
	dp = 0x00;
	cycles = 0;	// après un reset/reboot

	// firq and irq masked after reset
	cc |= (I_FLAG | F_FLAG);

	// After reset, nmi is fully disabled. Only after a first write to the system stackpointer enabled.
	nmi_enabled = false;
	old_nmi_line = *nmi_line;

	// set cpu status
	cpu_state = CPU_NORMAL;
	cwai_stacked = false;

	// 6309 en mode émulation après un reset (MD = 0)
	md = 0;
	set_md(0);
	trap63 = false;
	tfm_run = false;

	// Load program counter from vector
	pc = 0;
	pc = rd8(VECTOR_RESET) << 8;
	pc |= rd8(VECTOR_RESET+1);
}

// execute est maintenant dans mc6809_instructions.cpp avec l'aiguillage dispatch1 mis en ligne (un
// appel de moins par instruction).

void mc6809::toggle_breakpoint(uint16_t address)
{
	breakpoint_array[address] = !breakpoint_array[address];
}

void mc6809::clear_breakpoints()
{
	for (int i=0; i<65536; i++) {
		breakpoint_array[i] = false;
	}
}

IRAM_ATTR void mc6809::nmi()
{
	if (!cwai_stacked)	// déjà empilé par CWAI
	{
		set_e_flag();
		push_all();	// + F, E en mode natif 6309
	}
	cwai_stacked = false;
	set_i_flag();
	set_f_flag();
	pc = 0;
	pc = rd8(VECTOR_NMI) << 8;
	pc |= rd8(VECTOR_NMI+1);

	// TODO: Can't find this in the documentation
	cycles += 19;
}

IRAM_ATTR void mc6809::firq()
{
	if (!cwai_stacked)	// après CWAI, état complet déjà empilé (E=1)
	{
		if (__builtin_expect(md & 0x02, 0))	// 6309, MD bit 1 : FIRQ empile tout
		{
			set_e_flag();
			push_all();
			cycles += 9;
		}
		else
		{
			push_sp(pc & 0x00ff);
			push_sp((pc & 0xff00) >> 8);
			clear_e_flag();
			push_sp(cc);
		}
	}
	cwai_stacked = false;
	set_f_flag();
	set_i_flag();
	pc = 0;
	pc = rd8(VECTOR_FIRQ) << 8;
	pc |= rd8(VECTOR_FIRQ+1);

	// can't find this in the documentation
	cycles += 10;
}

IRAM_ATTR void mc6809::irq()
{
	if (!cwai_stacked)	// déjà empilé par CWAI
	{
		set_e_flag();
		push_all();	// + F, E en mode natif 6309
	}
	cwai_stacked = false;
	set_i_flag();
	pc = 0;
	pc = rd8(VECTOR_IRQ) << 8;
	pc |= rd8(VECTOR_IRQ+1);

	// can't find this in the documentation
	cycles += 19;
}

void mc6809::illegal_opcode()
{
	set_e_flag();
	push_all();
	set_i_flag();
	set_f_flag();
	pc = 0;
	pc = rd8(VECTOR_ILL_OPC) << 8;
	pc |= rd8(VECTOR_ILL_OPC+1);

	// same as nmi number of cycles
	cycles += 19;
}

// pc dp ac br xr yr us sp efhinzvc N F I NMI enabled/blocked c000 00 01:ae 0000 d0d0 0000 0ffc
// -*-*---- 11 1 1 state normal/cwai/sync
// TODO: "state normal" --> make it real after implementation of cwai/sync
void mc6809::status(char *text_buffer, int n)
{
	snprintf(text_buffer, n, " pc  dp ac br  xr   yr   us   sp  efhinzvc  N F I  cpu\n"
			"%04x %02x %02x:%02x "
			"%04x %04x %04x %04x "
			"%c%c%c%c%c%c%c%c "
			"%c%c %c %c  "
			"%s",
			pc, dp, ac, br,
			xr, yr, us, sp,
			cc & E_FLAG ? '*' : '-',
			cc & F_FLAG ? '*' : '-',
			cc & H_FLAG ? '*' : '-',
			cc & I_FLAG ? '*' : '-',
			cc & N_FLAG ? '*' : '-',
			cc & Z_FLAG ? '*' : '-',
			cc & V_FLAG ? '*' : '-',
			cc & C_FLAG ? '*' : '-',
			nmi_enabled ? old_nmi_line ? '1' : '0' : '-',
			nmi_enabled ? *nmi_line ? '1' : '0' : '-',
			*firq_line ? '1' : '0',
			*irq_line ? '1' : '0',
			cpu_state_description[cpu_state]);
}

void mc6809::stacks(char *text_buffer, int n, int no)
{
	// display top of both stacks as 8 and 16 bit values
	int bytes = snprintf(text_buffer, n, "  usp      ssp\n");
	text_buffer += bytes;
	n -= bytes;
	for (int i=0; i<no; i++) {
		bytes = snprintf(text_buffer, n, "%04x %02x  %04x %02x",
			get_us() + i,
			rd8((uint16_t)(get_us() + i)),
			get_sp() + i,
			rd8((uint16_t)(get_sp() + i)));
		text_buffer += bytes;
		n -= bytes;
		if (i < no-1) {
			bytes = snprintf(text_buffer, n, "\n");
			text_buffer += bytes;
			n -= bytes;
		}
	}
}
