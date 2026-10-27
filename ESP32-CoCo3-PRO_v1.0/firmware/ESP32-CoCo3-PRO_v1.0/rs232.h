// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// RS-232 Pak (26-2226) : ACIA 6551 en $FF68-$FF6B.
// D'après MAME mos6551.cpp / coco_rs232.cpp (license:BSD-3-Clause, copyright-holders:smf, Nathan Woods).

#pragma once
#include <stdint.h>
#include <stddef.h>

// Liaison (menu F12 S, L) : où vont les octets de l'ACIA.
#define RS_LINK_NONE  0     // rien de branché (DCD inactif, octets émis perdus)
#define RS_LINK_UART  1     // UART1 de l'ESP32 (TTL 3,3 V : TX GPIO 41, RX GPIO 42)
#define RS_LINK_LOOP  2     // bouclage TX -> RX (essais)
#define RS_LINK_MODEM 3     // modem Hayes par le Wi-Fi (modem.cpp : BBS, telnet)
#define RS_LINK_COUNT 4

// Vitesse réelle (menu F12 S, B) : 0 = celle programmée dans le 6551 sinon fixe (le logiciel reçoit
// alors les octets à cette cadence).
#define RS_SPEED_COUNT 9
extern const uint32_t k_rs_speed[RS_SPEED_COUNT];   // [0] = 0 (auto)

extern volatile bool g_rs_present;   // un RS-232 Pak est en service
extern uint8_t g_rs_slot;            // son slot (0-3)
extern uint8_t g_rs_link;            // RS_LINK_*
extern uint8_t g_rs_speed;           // index dans k_rs_speed

void     Rs_Reset(void);                       // remise à zéro matérielle
uint8_t  Rs_Read(uint16_t a);                  // $FF68-$FF6B (coeur CPU)
void     Rs_Write(uint16_t a, uint8_t v);
void     Rs_Poll(void);                        // coeur CPU : temps, liaison, IRQ
void     Rs_ApplyLink(void);                   // après un changement de liaison / vitesse
uint32_t Rs_ProgrammedBaud(void);
void     Rs_Report(char *buf, size_t n);       // ligne [RS232] de W

// Fourni par main.cpp : la sortie IRQ (niveau, 1 = active) a changé.
void     Rs_IrqChanged(bool active);
