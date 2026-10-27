// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur : Luc Dion 2026-10-26.

// Page web (port 80) : fichiers de la carte SD, état de l'ESP32.

#pragma once
#include <stdint.h>
#include <stddef.h>

void        Web_Init(void);                 // setup : réglages (NVS « cc3w »)
void        Web_Poll(void);                 // loop
void        Web_SetEnabled(bool on);        // sauvé
bool        Web_Enabled(void);
const char *Web_Pin(void);
void        Web_NewPin(void);               // nouveau PIN au hasard (sauvé)
void        Web_StatusLine(char *buf, size_t n);   // menu : « http://192.168.1.20/ » ou raison
void        Web_Report(void);               // ligne [WEB] de W
