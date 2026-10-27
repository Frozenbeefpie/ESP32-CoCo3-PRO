// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// AUTORUN : l'ESP32 tape une ou plusieurs lignes dans le BASIC au démarrage (F12 > SYSTEM > AUTORUN).

#pragma once
#include <stdint.h>
#include <stddef.h>

enum { AR_OFF = 0, AR_COMMAND = 1, AR_FILE = 2, AR_MODES };

void        Autorun_Init(void);           // setup : réglages (NVS « cc3b ») et lignes à taper
void        Autorun_Poll(void);           // loop
uint8_t     Autorun_Mode(void);
void        Autorun_SetMode(uint8_t m);   // sauvé
const char *Autorun_Command(void);
void        Autorun_SetCommand(const char *c);   // sauvé
void        Autorun_StatusLine(char *b, size_t n);   // menu / W
bool        Autorun_FileExists(void);
void        Autorun_UserKey(void);        // tâche USB : une touche réelle (annule la frappe en cours)
