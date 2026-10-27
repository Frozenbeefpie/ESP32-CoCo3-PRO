// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Sortie son horodatée : les changements du CNA sont datés en temps émulé puis rendus à cadence fixe.

#pragma once
#include <stdint.h>
#include <stddef.h>

#define AUDIO_RATE        48000u
#define AUDIO_TICKS_HZ    14318180u               // tics émulés (horloge GIME)

bool   Audio_Init(int gpio);                        // true = I2S PDM actif
bool   Audio_UsingI2S(void);
// sortie choisie à chaud (diagnostic du bruit de fond, commande A) 0 = I2S PDM, 1 = PWM LEDC 40 kHz, 2
// = broche à 0, 3 = broche à 1.
void    Audio_SetOutputMode(uint8_t m);
uint8_t Audio_GetOutputMode(void);
bool    Audio_UsingPWM(void);
void    Audio_PwmWrite(uint8_t v);
// plancher (silence = broche à 0, contre le bruit d'alimentation).
void    Audio_SetFloor(bool on);
void   Audio_Push(uint32_t t_ticks, uint8_t level); // coeur CPU : nouveau niveau (0-255)
// stéréo. Gauche = GPIO 47 (broche de Audio_Init), droite = AUDIO_R_GPIO. Mono (défaut) : une ligne,
// gauche et droite moyennées
#define AUDIO_R_GPIO 12
void   Audio_PushLR(uint32_t t_ticks, uint8_t left, uint8_t right);
void   Audio_SetStereo(bool on);
bool   Audio_GetStereo(void);                       // réglage demandé
bool   Audio_IsStereo(void);                        // réellement en service
// Génère n échantillons 16 bits signés ; now_ticks = temps émulé actuel. Utilisé par la tâche audio
// (ESP) et par le banc PC.
void   Audio_Render(int16_t *out, size_t n, uint32_t now_ticks);
void   Audio_Flush(void);                           // vider les files (changement de base de temps)
void   Audio_Resync(void);                          // après une pause (menu F12)
struct audio_stats_t { uint32_t pushed, dropped, late, resyncs, flushes; };   // flushes (Turbo / Overclock)
const audio_stats_t *Audio_GetStats(void);
// signal de test (0 = son du CoCo, 1 sinus 1 kHz, 2 sinus -20 dB 3 carré 1 kHz, 4 silence milieu
// d'échelle). I2S seulement
void    Audio_PsgEnable(bool on);                   // + remise à zéro (voies muettes)
void    Audio_PsgWrite(uint32_t t_ticks, uint8_t v);
// YM2149 de la carte CoCo PSG (registre r 0-15 ; r = 0x10 : contrôle de la carte, bit 0 = horloge /
// 2). Porté de MAME ay8910.cpp (BSD-3-Clause).
void    Audio_AyEnable(bool on);
void    Audio_AyWrite(uint32_t t_ticks, uint8_t r, uint8_t v);
void    Audio_SetPaused(bool on);               // menu F12 : générateurs continus (PSG) muets
void    Audio_SetTestTone(uint8_t mode);
// flux d'échantillons signés 16 bits à 48 kHz (écran d'accueil) : remplace le son du CoCo tant qu'il
// est ouvert. Un seul producteur.
bool     Audio_StreamBegin(void);
void     Audio_StreamEnd(void);
uint32_t Audio_StreamRoom(void);
void     Audio_StreamWrite(const int16_t *p, uint32_t n);
uint8_t Audio_GetTestTone(void);
