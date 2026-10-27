// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Wi-Fi (station) et heure NTP ; F12 > NETWORK.

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <time.h>

#define NET_TZ_COUNT 9
extern const char *const k_net_tz_label[NET_TZ_COUNT];

struct NetAp { char ssid[33]; int8_t rssi; bool open; };

void        Net_Init(void);                       // setup : réglages ; démarre si activé
void        Net_Poll(void);                       // loop : journal des changements d'état, heure
void        Net_SetEnabled(bool on);              // marche / arrêt (sauvé)
bool        Net_Enabled(void);
void        Net_SetCredentials(const char *ssid, const char *pass);   // sauve et (re)connecte
const char *Net_Ssid(void);
int         Net_Scan(NetAp *list, int max);       // bloquant (2-4 s) ; trié par signal
void        Net_SetTz(uint8_t idx);               // fuseau (sauvé)
uint8_t     Net_Tz(void);
bool        Net_Connected(void);
void        Net_StatusLine(char *buf, size_t n);  // menu : « CONNECTED 192.168.1.20 -58 dBm »
bool        Net_TimeValid(void);                  // heure NTP reçue
bool        Net_LocalTime(struct tm *t);          // heure locale (faux si inconnue)
uint32_t    Net_TodayYmd(void);                   // AAAAMMJJ (0 si inconnue)
void        Net_Report(void);                     // ligne [WIFI] de W

// ---------------------------------------------------------------------------
// DriveWire par le réseau (client TCP vers un serveur DriveWire sur un PC DriveWire 4, pyDriveWire,
// lwwire... port 65504 par défaut)
// ---------------------------------------------------------------------------
void        NetDw_SetEnabled(bool on);            // sauvé
bool        NetDw_Enabled(void);
void        NetDw_SetServer(const char *hostport);// « 192.168.1.10 » ou « pc.local:65504 » (sauvé)
const char *NetDw_Server(void);                   // tel que tapé (« » = aucun)
bool        NetDw_Linked(void);                   // connexion TCP ouverte
void        NetDw_StatusLine(char *buf, size_t n);
// Coeur CPU
bool        NetDw_Active(void);                   // en marche (le port Becker va au PC)
void        NetDw_Put(uint8_t b);                 // CoCo -> serveur
bool        NetDw_Ready(void);                    // un octet du serveur attend
int         NetDw_Get(void);                      // 1 si rien
void        NetDw_Flush(void);                    // reset du CoCo : octets reçus jetés
void        NetDw_Shutdown(void);                 // avant un redémarrage : ferme la connexion proprement
