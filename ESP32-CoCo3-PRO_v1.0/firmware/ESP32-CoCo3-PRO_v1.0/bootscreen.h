// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Écran d'accueil (logo + musique, ou film / images de la carte SD) et écran de veille.

#pragma once
#include <stdint.h>
#include <stddef.h>

enum { BS_OFF = 0, BS_LOGO = 1, BS_SD = 2, BS_MODES };
enum { BS_END_KEY = 0, BS_END_5, BS_END_10, BS_END_30, BS_ENDS };

void    BootScreen_Init(void);          // setup : réglages (NVS « cc3s »)
bool    BootScreen_Wanted(void);        // à montrer à ce démarrage
void    BootScreen_Run(void);           // coeur vidéo : affiche jusqu'à la touche (bloquant)
uint8_t BootScreen_Mode(void);
void    BootScreen_SetMode(uint8_t m);  // sauvé
uint8_t BootScreen_End(void);
void    BootScreen_SetEnd(uint8_t e);   // sauvé
void    BootScreen_Describe(char *b, size_t n);   // menu : ce que SD CARD montrera

// écran de veille (même contenu que l'écran d'accueil) après N minutes sans action au clavier, à la
// souris ou à la manette ; le CoCo est arrêté pendant la veille et reprend où il était
enum { SV_OFF = 0, SV_5, SV_10, SV_20, SV_30, SV_COUNT };
enum { SVK_BLANK = 0, SVK_LOGO, SVK_SD, SVK_COUNT };
uint8_t Saver_Delay(void);   void Saver_SetDelay(uint8_t d);
uint8_t Saver_Kind(void);    void Saver_SetKind(uint8_t k);
bool    Saver_Sound(void);   void Saver_SetSound(bool on);
bool    Saver_Due(bool emu_running);    // coeur vidéo, à chaque tour
void    Saver_Run(void);                // bloquant jusqu'au réveil
void    Saver_Touch(void);              // repart de zéro (après le menu, etc.)
