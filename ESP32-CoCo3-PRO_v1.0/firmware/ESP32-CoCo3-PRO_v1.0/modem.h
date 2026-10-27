// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Modem Hayes par le Wi-Fi : commandes AT, « composer » = connexion TCP (telnet, BBS).

#pragma once
#include <stdint.h>
#include <stddef.h>

// Coeur CPU (RS-232 Pak, bit banger)
void     Modem_Put(uint8_t b);          // octet émis par le CoCo
bool     Modem_Ready(void);             // un octet attend pour le CoCo
int      Modem_Get(void);               // 1 si rien
bool     Modem_Carrier(void);           // DCD (selon AT&C)
void     Modem_SetDtr(bool on);         // DTR du CoCo (AT&D2 : chute = raccrocher)
void     Modem_Use(void);               // met le modem en service au premier usage
void     Modem_Step(void);              // un pas (tâche réseau de cc3net.cpp)

// Menus / journal
void     Modem_StatusLine(char *buf, size_t n);   // « IDLE », « CONNECTED bbs:23 »...
void     Modem_Report(char *buf, size_t n);       // ligne [MODEM] de W
void     Modem_Shutdown(void);                    // avant un redémarrage : raccroche
void     Modem_MainPoll(void);                    // loop : AT&W (écriture en flash hors de la tâche du modem)
