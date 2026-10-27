// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Lecteur de cassette virtuel : $FF20 bit 0 suit une bande virtuelle qui n'avance que moteur en marche ($FF21) ;
// ce que le CoCo écrit par son CNA moteur en marche est enregistré en .CAS.

#pragma once
#include <stdint.h>
#include <stddef.h>

enum { CAS_EMPTY = 0, CAS_PLAY = 1, CAS_RECORD = 2 };
enum { CAS_FMT_CAS = 0, CAS_FMT_WAV = 1 };

// Coeur CPU (accès aux registres) ; t = tics émulés
void    Cas_Motor(bool on, uint32_t t);
uint8_t Cas_InBit(uint32_t t);                   // entrée cassette : 0 / 1
void    Cas_DacWrite(uint8_t level8, uint32_t t);// sortie du CNA (bits 7-2 de $FF20)
extern bool g_cas_motor;                         // moteur en marche (lu à chaque ligne : en ligne)
static inline bool Cas_MotorOn(void) { return g_cas_motor; }
uint8_t Cas_Level(uint32_t t);                   // son de la bande (AUDIO ON) : 0 / 255

// loop (coeur 1, basse priorité) : lecture anticipée du .WAV, écriture des enregistrements
void    Cas_Service(void);

// Menu (émulation suspendue)
bool    Cas_Insert(const char *path);            // CAS / .WAV : bande en lecture, au début
void    Cas_Eject(void);                         // ferme (un enregistrement est finalisé)
void    Cas_Rewind(void);
bool    Cas_RecordNew(void);                     // nouvelle bande vierge dans /CASSETTE
uint8_t Cas_Mode(void);
const char *Cas_Name(void);                      // chemin de la bande ("" si aucune)
void    Cas_Position(char *b, size_t n);         // « 12 % (3.4 s) », « 5.2 s, 1834 bytes »
bool    Cas_GetFast(void);
void    Cas_SetFast(bool on);
uint8_t Cas_GetFormat(void);                     // format des nouvelles bandes
void    Cas_SetFormat(uint8_t f);
void    Cas_Report(char *b, size_t n);           // commande W
// fichiers d'une bande .CAS / .WAV (blocs d'en-tête justes) : nom (8 car.), type 0/1/2, ASCII
int     Cas_ListTape(const char *path, void (*cb)(const char *name8, uint8_t type, uint8_t ascii, void *u), void *u);
