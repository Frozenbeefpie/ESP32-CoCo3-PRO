// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Dossier partagé d'un PC (tools/coco_share.py) : images .DSK du PC montées dans le menu D.

#pragma once
#include <stdint.h>
#include <stddef.h>

enum
{
  NS_OK = 0,
  NS_NOWIFI = -1,     // Wi-Fi non connecté
  NS_NOSERVER = -2,   // aucun serveur trouvé / injoignable
  NS_PIN = -3,        // code PIN refusé (ou absent)
  NS_ERR = -4,        // erreur de transfert / réponse inattendue
  NS_NOMEM = -5,      // PSRAM insuffisante
  NS_NOTFOUND = -6,   // fichier ou dossier absent
  NS_READONLY = -7,   // partage en lecture seule
};

#define NS_MAX_ENTRIES 400

int         NetShare_Connect(void);                 // trouve le serveur (mémorisé ou diffusion) et le salue
const char *NetShare_ServerName(void);              // nom du PC (après Connect)
const char *NetShare_Host(void);                    // « ip:port » ("" = aucun)
bool        NetShare_HasPin(void);
void        NetShare_SetPin(const char *pin);       // sauvé en flash
const char *NetShare_ErrorText(int code);           // texte anglais court pour le menu

int  NetShare_List(const char *path);               // lit un dossier : nombre d'entrées ou code < 0
int  NetShare_Count(void);
bool NetShare_Entry(int i, bool *dir, uint32_t *size, const char **name);
int  NetShare_Read(const char *path, uint32_t off, uint32_t len, uint8_t *buf, uint32_t *got);   // morceau
int  NetShare_Get(const char *path, uint8_t **buf, uint32_t *size);  // fichier entier en PSRAM (free par l'appelant)
int  NetShare_Put(const char *path, const uint8_t *data, uint32_t size);

void NetShare_QueueMount(uint8_t drv, const char *path);   // à monter dès que possible (démarrage)
void NetShare_Poll(void);                           // loop : renvoi des disquettes modifiées, montages différés
void NetShare_Report(void);                         // ligne [SHARE] de W
