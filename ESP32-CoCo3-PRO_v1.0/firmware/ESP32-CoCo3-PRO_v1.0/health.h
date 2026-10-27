// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// État matériel de l'ESP32 : charge des coeurs, température, mémoires, SD (F12 > SYSTEM > HARDWARE HEALTH).

#pragma once
#include <stdint.h>
#include <stddef.h>

enum { HL_OK = 0, HL_WARN = 1, HL_BAD = 2, HL_INFO = 3 };
#define HEALTH_ROWS 16

void Health_Sample(void);        // loop : mesures (au plus une fois par seconde)
void Health_RefreshSd(void);     // loop : place libre de la SD à recalculer (lent : à la demande)
// Ligne i (0..HEALTH_ROWS-1) : nom, valeur (avec barre), niveau HL_*
void Health_Row(int i, const char **label, char *val, size_t n, uint8_t *level);
void Health_Json(char *b, size_t n);   // page web (/api/health)
void Health_Report(void);        // ligne [HEALTH] de W
