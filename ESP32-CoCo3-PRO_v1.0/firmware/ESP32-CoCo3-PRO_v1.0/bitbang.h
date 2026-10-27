// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Port série du CoCo (« bit banger ») : sortie $FF20 bit 1, entrée $FF22 bit 0.

#pragma once
#include <stdint.h>
#include <stddef.h>

#define BB_OFF       0
#define BB_PRINTER   1
#define BB_DRIVEWIRE 2
#define BB_UART      3
#define BB_NETDW     4   // serveur DriveWire du PC par le Wi-Fi (menu N)
#define BB_MODEM     5   // modem Hayes par le Wi-Fi (BBS, telnet)
#define BB_MODE_COUNT 6

#define BB_SPEED_COUNT 11
extern const uint32_t k_bb_speed[BB_SPEED_COUNT];   // [0] = 0 (auto)

extern uint8_t g_bb_mode;      // BB_*
extern uint8_t g_bb_speed;     // index dans k_bb_speed
extern volatile uint64_t g_bb_ticks;   // main.cpp : temps CPU (tics 14,31818 MHz)

void    Bb_Reset(void);                      // remise à zéro (reset du CoCo, changement de mode)
void    Bb_Write(uint8_t ff20, uint8_t ddr); // écriture de $FF20 (valeur, DDR)
uint8_t Bb_InBit(void);                      // niveau de $FF22 bit 0
void    Bb_Poll(void);                       // coeur CPU (<= 1 fois / ms) : liaisons
bool    Bb_UartBlocked(void);                // UART pris par le RS-232 Pak
bool    Bb_ModemBlocked(void);               // modem pris par le RS-232 Pak
uint32_t Bb_CurrentBaud(void);               // vitesse en service (0 = inconnue)
void    Bb_Report(char *buf, size_t n);      // ligne [BITBANG] de W

// File de l'imprimante (coeur CPU -> loop) : lue par main.cpp qui écrit le fichier.
int     Bb_PrnGet(void);                     // 1 si vide
uint32_t Bb_PrnLastMs(void);                 // dernier octet imprimé (ms)
