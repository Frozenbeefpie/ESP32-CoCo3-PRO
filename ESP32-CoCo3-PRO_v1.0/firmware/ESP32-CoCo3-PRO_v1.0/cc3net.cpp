// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Wi-Fi (station) et heure NTP (voir cc3net.h).

#include "cc3net.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

void Log_Line(const char *s);   // main.cpp : journal (série + anneau du F9)

const char *const k_net_tz_label[NET_TZ_COUNT] = {
  "Eastern (Montreal, Toronto, New York)", "Atlantic (Halifax)", "Newfoundland",
  "Central (Winnipeg, Chicago)", "Mountain (Calgary, Denver)", "Pacific (Vancouver, LA)",
  "UTC", "UK (London)", "Central Europe (Paris)" };
static const char *const k_net_tz_posix[NET_TZ_COUNT] = {
  "EST5EDT,M3.2.0,M11.1.0", "AST4ADT,M3.2.0,M11.1.0", "NST3:30NDT,M3.2.0,M11.1.0",
  "CST6CDT,M3.2.0,M11.1.0", "MST7MDT,M3.2.0,M11.1.0", "PST8PDT,M3.2.0,M11.1.0",
  "UTC0", "GMT0BST,M3.5.0/1,M10.5.0", "CET-1CEST,M3.5.0,M10.5.0/3" };

static bool    s_enabled = false;
static char    s_ssid[33] = "";
static char    s_pass[65] = "";
static uint8_t s_tz = 0;

#ifdef ESP_PLATFORM
#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"   // tâche avec pile en PSRAM
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_sntp.h"

static volatile bool    s_ev_up = false, s_ev_down = false, s_ev_ip = false;
static volatile uint8_t s_ev_reason = 0;
static volatile uint8_t s_auth_fail = 0;   // échecs d'authentification de suite (remis à 0 à l'obtention de l'IP)
static volatile bool    s_connected = false;
static bool     s_started = false, s_time_logged = false;
static bool     s_boot_wait = false;   // démarrage du Wi-Fi après les réclamations USB
static bool     s_radio_off = false;   // pilote chargé, radio arrêtée (esp_wifi_stop)
static uint32_t s_mem_before = 0, s_dma_before = 0, s_dmablk_before = 0;
static uint32_t s_mem_after = 0, s_dma_after = 0, s_dmablk_after = 0;
static uint32_t s_connects = 0, s_drops = 0;
// le pilote Wi-Fi a besoin d'environ 36 Ko de RAM interne DMA
#define NET_NEED_DMA   (36u * 1024u)
#define NET_NEED_BLOCK ( 8u * 1024u)
static void Net_Mem(uint32_t &in, uint32_t &dma, uint32_t &blk);
static bool s_nomem = false;      // pas assez de RAM (vérifié avant de démarrer)
static bool s_broken = false;     // démarrage raté : plus aucun appel au Wi-Fi
static bool Net_MemoryOk(const char *what)
{
  uint32_t in, dma, blk;
  Net_Mem(in, dma, blk);
  if (dma >= NET_NEED_DMA && blk >= NET_NEED_BLOCK) { s_nomem = false; return true; }
  s_nomem = true;
  char b[200];
  snprintf(b, sizeof b, "[WIFI] %s refuse : RAM interne DMA %lu Ko (bloc %lu o), il en faut %u Ko (bloc %u o) ; debrancher un peripherique USB et redemarrer",
           what, (unsigned long)(dma / 1024), (unsigned long)blk, NET_NEED_DMA / 1024, NET_NEED_BLOCK);
  Log_Line(b);
  return false;
}

static void Net_Save(void)
{
  Preferences p;
  if (!p.begin("cc3n", false)) return;
  p.putBool("on", s_enabled);
  p.putString("ssid", s_ssid);
  p.putString("pass", s_pass);
  p.putUChar("tz", s_tz);
  extern bool g_netdw_on; extern char g_netdw_host[];
  p.putBool("dwon", g_netdw_on);
  p.putString("dwhost", g_netdw_host);
  p.end();
}

static void Net_Event(arduino_event_id_t ev, arduino_event_info_t info)
{
  switch (ev)
  {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:    s_ev_up = true; break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:       s_connected = true; s_ev_ip = true; s_auth_fail = 0; break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      s_connected = false; s_ev_down = true; s_ev_reason = info.wifi_sta_disconnected.reason;
      { const uint8_t r = s_ev_reason; if ((r == 15 || r == 202 || r == 204 || r == 2) && s_auth_fail < 255) s_auth_fail++; }
      break;
    default: break;
  }
}

static void Net_Mem(uint32_t &in, uint32_t &dma, uint32_t &blk)
{
  in  = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  dma = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  blk = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
}

// arrête proprement une tentative en cours (sinon « sta is connecting, return error »)
static void Net_Halt(void)
{
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false);
  s_connected = false;
  for (int i = 0; i < 20 && WiFi.status() == WL_CONNECTED; i++) vTaskDelay(pdMS_TO_TICKS(10));
  vTaskDelay(pdMS_TO_TICKS(150));          // laisse le pilote quitter l'état « connecting »
}

static void Net_Start(void)
{
  if (s_started || s_broken) return;
  { extern void Usb_DmaReserveTake(void); if (!s_radio_off) Usb_DmaReserveTake(); }   // avant que le pilote prenne sa RAM
  if (!s_radio_off && !Net_MemoryOk("demarrage")) return;   // pilote déjà chargé : rien à allouer
  char b[160];
  Net_Mem(s_mem_before, s_dma_before, s_dmablk_before);
  WiFi.persistent(false);                  // réglages gardés par nous (NVS « cc3n »)
  static bool s_ev_reg = false;
  if (!s_ev_reg) { s_ev_reg = true; WiFi.onEvent(Net_Event); }
  WiFi.setHostname("ESP32-CoCo3");
  if (!WiFi.mode(WIFI_STA) || WiFi.getMode() != WIFI_STA)   // démarrage raté (RAM)
  {
    s_broken = true;
    Log_Line("[WIFI] ECHEC du demarrage du pilote (RAM interne) : Wi-Fi desactive jusqu'au redemarrage");
    return;
  }
  if (s_radio_off)                          // on rallume la radio (pilote resté chargé)
  {
    const esp_err_t e = esp_wifi_start();
    if (e != ESP_OK) { snprintf(b, sizeof b, "[WIFI] radio : esp_wifi_start %s", esp_err_to_name(e)); Log_Line(b); }
    s_radio_off = false;
  }
  WiFi.setAutoReconnect(true);
  // pas de veille modem : avec elle, un paquet reçu attend la balise suivante du routeur (100-300 ms)
  // ; DriveWire (NitrOS-9 interroge le serveur sans cesse) et le partage de disques deviennent très
  // lents
  WiFi.setSleep(false);
  configTzTime(k_net_tz_posix[s_tz < NET_TZ_COUNT ? s_tz : 0], "pool.ntp.org", "time.nist.gov");
  s_started = true;
  s_time_logged = false;
  s_auth_fail = 0;
  if (s_ssid[0]) WiFi.begin(s_ssid, s_pass);
  Net_Mem(s_mem_after, s_dma_after, s_dmablk_after);
  snprintf(b, sizeof b, "[WIFI] demarre (%s) : RAM interne %lu -> %lu Ko, DMA %lu -> %lu Ko (plus grand bloc %lu -> %lu)",
           s_ssid[0] ? s_ssid : "aucun reseau choisi",
           (unsigned long)(s_mem_before / 1024), (unsigned long)(s_mem_after / 1024),
           (unsigned long)(s_dma_before / 1024), (unsigned long)(s_dma_after / 1024),
           (unsigned long)s_dmablk_before, (unsigned long)s_dmablk_after);
  Log_Line(b);
}

// arrêt sans plantage. Avec le core 3.2.1, WiFi.mode(WIFI_OFF) retire les gestionnaires d'évènements
// d'Arduino (et WiFi.mode(WIFI_STA) les rajoute) sans verrou, pendant que la tâche des évènements…
static void Net_SafeOff(void)
{
  s_connected = false;
  WiFi.setAutoReconnect(false);
  esp_sntp_stop();                          // plus de requêtes NTP
  if (WiFi.status() == WL_CONNECTED)
  {
    WiFi.disconnect(false);
    for (int i = 0; i < 50 && WiFi.status() == WL_CONNECTED; i++) vTaskDelay(pdMS_TO_TICKS(10));
  }
  vTaskDelay(pdMS_TO_TICKS(100));
  esp_wifi_stop();                          // radio arrêtée ; pilote et gestionnaires restent
  s_radio_off = true;
  vTaskDelay(pdMS_TO_TICKS(100));
}

static void Net_Stop(void)
{
  if (!s_started || s_broken) { s_started = false; s_connected = false; return; }
  s_started = false;                        // DriveWire réseau et partage : plus de Wi-Fi (Net_Connected)
  Net_SafeOff();
  s_connected = false;
  uint32_t in, dma, blk; Net_Mem(in, dma, blk);
  char b[180];
  snprintf(b, sizeof b, "[WIFI] arrete (radio coupee ; toute la RAM au redemarrage) : RAM interne %lu Ko, DMA %lu Ko (plus grand bloc %lu)",
           (unsigned long)(in / 1024), (unsigned long)(dma / 1024), (unsigned long)blk);
  Log_Line(b);
}

void Net_Init(void)
{
  Preferences p;
  if (p.begin("cc3n", true))
  {
    s_enabled = p.getBool("on", false);
    String a = p.getString("ssid", ""), b = p.getString("pass", "");
    strncpy(s_ssid, a.c_str(), sizeof s_ssid - 1);
    strncpy(s_pass, b.c_str(), sizeof s_pass - 1);
    s_tz = p.getUChar("tz", 0);
    if (s_tz >= NET_TZ_COUNT) s_tz = 0;
    extern bool g_netdw_on; extern char g_netdw_host[];
    g_netdw_on = p.getBool("dwon", false);
    String h = p.getString("dwhost", "");
    strncpy(g_netdw_host, h.c_str(), 63); g_netdw_host[63] = 0;
    p.end();
  }
  // Le fuseau sert aussi sans Wi-Fi (heure inconnue, mais réglage prêt).
  setenv("TZ", k_net_tz_posix[s_tz], 1);
  tzset();
  // au démarrage, le Wi-Fi attend que l'USB ait réclamé clavier, souris et manettes (Net_Poll)
  if (s_enabled) s_boot_wait = true;
  extern void NetDw_Boot(void); NetDw_Boot();
}

extern bool Usb_BootSettled(void);   // main.cpp
void Net_Poll(void)
{
  char b[160];
  if (s_boot_wait && Usb_BootSettled())
  {
    s_boot_wait = false;
    if (s_enabled) { Log_Line("[WIFI] demarrage apres les reclamations USB (clavier d'abord)"); Net_Start(); }
  }
  if (s_ev_up)   { s_ev_up = false; snprintf(b, sizeof b, "[WIFI] associe a %s", s_ssid); Log_Line(b); }
  if (s_ev_ip)
  {
    s_ev_ip = false; s_connects++;
    { extern void Usb_DmaReserveArm(void); Usb_DmaReserveArm(); }   // réserve DMA rendue à l USB seulement après la connexion
    Net_Mem(s_mem_after, s_dma_after, s_dmablk_after);
    snprintf(b, sizeof b, "[WIFI] connecte : IP %s, signal %d dBm, canal %d ; RAM interne %lu Ko, DMA %lu Ko (plus grand bloc %lu)",
             WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(), (int)WiFi.channel(),
             (unsigned long)(s_mem_after / 1024), (unsigned long)(s_dma_after / 1024), (unsigned long)s_dmablk_after);
    Log_Line(b);
  }
  if (s_ev_down)
  {
    s_ev_down = false;
    if (s_started) { s_drops++; snprintf(b, sizeof b, "[WIFI] deconnecte (raison %u)", (unsigned)s_ev_reason); Log_Line(b); }
  }
  if (s_started && !s_time_logged && Net_TimeValid())
  {
    s_time_logged = true;
    struct tm t; Net_LocalTime(&t);
    snprintf(b, sizeof b, "[WIFI] heure NTP : %04d-%02d-%02d %02d:%02d:%02d (%s)", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec, k_net_tz_posix[s_tz]);
    Log_Line(b);
  }
}

void Net_SetEnabled(bool on)
{
  s_enabled = on;
  Net_Save();
  s_boot_wait = false;
  if (on) Net_Start(); else Net_Stop();
}

void Net_SetCredentials(const char *ssid, const char *pass)
{
  strncpy(s_ssid, ssid ? ssid : "", sizeof s_ssid - 1); s_ssid[sizeof s_ssid - 1] = 0;
  strncpy(s_pass, pass ? pass : "", sizeof s_pass - 1); s_pass[sizeof s_pass - 1] = 0;
  s_enabled = true;
  Net_Save();
  s_boot_wait = false;
  if (s_broken) return;
  if (!s_started) Net_Start();
  else { Net_Halt(); WiFi.setAutoReconnect(true); WiFi.begin(s_ssid, s_pass); }
}

int Net_Scan(NetAp *list, int max)
{
  const bool was = s_started;
  if (s_broken) { Log_Line("[WIFI] recherche impossible : pilote en echec (redemarrer)"); return 0; }
  if (!was && s_radio_off)                  // pilote chargé, radio arrêtée
  {
    esp_wifi_start();
    s_radio_off = false;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  else if (!was)
  {
    if (!Net_MemoryOk("recherche")) return 0;
    WiFi.persistent(false);
    if (!WiFi.mode(WIFI_STA) || WiFi.getMode() != WIFI_STA)
    {
      s_broken = true;
      Log_Line("[WIFI] ECHEC du demarrage du pilote (RAM interne) : Wi-Fi desactive jusqu'au redemarrage");
      return 0;
    }
  }
  // pendant une tentative de connexion (réseau introuvable, reconnexion auto) le pilote refuse la
  // recherche (« sta is connecting ») -> on suspend la tentative.
  const bool resume = was && !s_connected;
  if (resume) Net_Halt();
  int n = WiFi.scanNetworks(false, false);
  if (n < 0) { vTaskDelay(pdMS_TO_TICKS(500)); n = WiFi.scanNetworks(false, false); }
  if (n < 0) { char e[64]; snprintf(e, sizeof e, "[WIFI] recherche refusee par le pilote (code %d)", n); Log_Line(e); }
  int k = 0;
  for (int i = 0; i < n && k < max; i++)
  {
    const String s = WiFi.SSID(i);
    if (!s.length()) continue;                           // réseau caché
    bool dup = false;
    for (int j = 0; j < k; j++) if (!strcmp(list[j].ssid, s.c_str())) { dup = true; if (WiFi.RSSI(i) > list[j].rssi) list[j].rssi = (int8_t)WiFi.RSSI(i); }
    if (dup) continue;
    strncpy(list[k].ssid, s.c_str(), 32); list[k].ssid[32] = 0;
    list[k].rssi = (int8_t)WiFi.RSSI(i);
    list[k].open = WiFi.encryptionType(i) == WIFI_AUTH_OPEN;
    { char e[80]; snprintf(e, sizeof e, "[WIFI]   %-32s %4d dBm, canal %d%s", list[k].ssid, (int)WiFi.RSSI(i), (int)WiFi.channel(i), list[k].open ? ", ouvert" : ""); Log_Line(e); }
    k++;
  }
  WiFi.scanDelete();
  if (!was) Net_SafeOff();                  // radio arrêtée, pilote gardé (pas de WiFi.mode(WIFI_OFF))
  else if (resume && s_ssid[0]) { WiFi.setAutoReconnect(true); WiFi.begin(s_ssid, s_pass); }
  for (int i = 1; i < k; i++)                            // tri par signal décroissant
    for (int j = i; j > 0 && list[j].rssi > list[j - 1].rssi; j--) { NetAp t = list[j]; list[j] = list[j - 1]; list[j - 1] = t; }
  char b[64]; snprintf(b, sizeof b, "[WIFI] recherche : %d reseau(x)", k); Log_Line(b);
  return k;
}

void Net_SetTz(uint8_t idx)
{
  s_tz = (idx < NET_TZ_COUNT) ? idx : 0;
  setenv("TZ", k_net_tz_posix[s_tz], 1);
  tzset();
  Net_Save();
}

bool Net_Connected(void) { return s_started && s_connected; }

void Net_StatusLine(char *buf, size_t n)
{
  if (s_broken)                 { snprintf(buf, n, "FAILED (memory): reboot the ESP32-CoCo3"); return; }
  if (s_enabled && s_nomem)     { snprintf(buf, n, "NOT ENOUGH MEMORY: unplug a USB device, reboot"); return; }
  if (s_enabled && s_boot_wait) { snprintf(buf, n, "STARTING (after USB devices)"); return; }
  if (!s_enabled || !s_started) { snprintf(buf, n, "OFF"); return; }
  if (!s_ssid[0])               { snprintf(buf, n, "ON, no network chosen"); return; }
  if (s_connected) snprintf(buf, n, "CONNECTED %s %d dBm", WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  else if (s_drops && (s_ev_reason == 201))
                   snprintf(buf, n, "NETWORK NOT FOUND (2.4 GHz only), retrying");
  else if (s_drops && s_auth_fail >= 3)   // 3 échecs d'authentification de suite
                   snprintf(buf, n, "WRONG PASSWORD? retrying");
  else             snprintf(buf, n, "CONNECTING to %.20s...", s_ssid);
}

void Net_Report(void)
{
  char b[400];
  if (!s_started) { snprintf(b, sizeof b, "[WIFI] arrete%s%s", s_ssid[0] ? ", reseau memorise " : "", s_ssid); Log_Line(b); return; }
  uint32_t in, dma, blk; Net_Mem(in, dma, blk);
  char tbuf[80] = "inconnue";
  struct tm t;
  if (Net_LocalTime(&t)) snprintf(tbuf, sizeof tbuf, "%04d-%02d-%02d %02d:%02d:%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
  snprintf(b, sizeof b, "[WIFI] %s, reseau %s, IP %s, signal %d dBm, canal %d ; connexions %lu, pertes %lu ; heure %s ; RAM interne %lu Ko, DMA %lu Ko (plus grand bloc %lu) ; avant le Wi-Fi : %lu / %lu Ko (bloc %lu)",
           s_connected ? "connecte" : "non connecte", s_ssid, s_connected ? WiFi.localIP().toString().c_str() : "-",
           s_connected ? (int)WiFi.RSSI() : 0, s_connected ? (int)WiFi.channel() : 0,
           (unsigned long)s_connects, (unsigned long)s_drops, tbuf,
           (unsigned long)(in / 1024), (unsigned long)(dma / 1024), (unsigned long)blk,
           (unsigned long)(s_mem_before / 1024), (unsigned long)(s_dma_before / 1024), (unsigned long)s_dmablk_before);
  Log_Line(b);
  extern void NetDw_Report(void); NetDw_Report();
}

#else
// Banc (PC) : pas de Wi-Fi.
void Net_Init(void) {}
void Net_Poll(void) {}
void Net_SetEnabled(bool on) { s_enabled = on; }
void Net_SetCredentials(const char *ssid, const char *pass)
{ strncpy(s_ssid, ssid ? ssid : "", sizeof s_ssid - 1); strncpy(s_pass, pass ? pass : "", sizeof s_pass - 1); s_enabled = true; }
int  Net_Scan(NetAp *, int) { return 0; }
void Net_SetTz(uint8_t idx) { s_tz = (idx < NET_TZ_COUNT) ? idx : 0; }
bool Net_Connected(void) { return false; }
void Net_StatusLine(char *buf, size_t n) { snprintf(buf, n, s_enabled ? "ON (bench: no Wi-Fi)" : "OFF"); }
void Net_Report(void) { Log_Line("[WIFI] banc : pas de Wi-Fi"); }
#endif

bool        Net_Enabled(void) { return s_enabled; }
const char *Net_Ssid(void)    { return s_ssid; }
uint8_t     Net_Tz(void)      { return s_tz; }

bool Net_TimeValid(void)
{
#ifdef ESP_PLATFORM
  return time(nullptr) > 1704067200;   // après le 1er janvier 2024 : heure NTP reçue
#else
  return false;
#endif
}

bool Net_LocalTime(struct tm *t)
{
  if (!Net_TimeValid()) return false;
  const time_t now = time(nullptr);
  localtime_r(&now, t);
  return true;
}

uint32_t Net_TodayYmd(void)
{
  struct tm t;
  if (!Net_LocalTime(&t)) return 0;
  return (uint32_t)((t.tm_year + 1900) * 10000 + (t.tm_mon + 1) * 100 + t.tm_mday);
}

// ===========================================================================
// DriveWire par le réseau (voir cc3net.h)
// ===========================================================================
bool g_netdw_on = false;            // réglage (NVS « cc3n » : dwon / dwhost)
char g_netdw_host[64] = "";

// Files à un producteur / un consommateur (coeur CPU <-> tâche réseau), en PSRAM.
#define NDW_TXQ 2048u               // CoCo -> serveur
#define NDW_RXQ 8192u               // serveur -> CoCo
static uint8_t *s_ndw_tx = nullptr, *s_ndw_rx = nullptr;
static volatile uint16_t s_ndw_tx_h = 0, s_ndw_tx_t = 0;   // h : coeur CPU, t : tâche
static volatile uint16_t s_ndw_rx_h = 0, s_ndw_rx_t = 0;   // h : tâche, t : coeur CPU
static volatile bool     s_ndw_rx_flush = false;           // tâche -> CPU : jeter ce qui précède s_ndw_flush_to
static volatile uint16_t s_ndw_flush_to = 0;
static volatile bool     s_ndw_linked = false;
static volatile uint32_t s_ndw_in = 0, s_ndw_out = 0, s_ndw_lost = 0, s_ndw_links = 0;
static volatile uint8_t  s_ndw_state = 0;                  // NDW_*
static volatile uint32_t s_ndw_unlinked_ms = 0;
static uint32_t s_rtt_t0 = 0, s_rtt_sum = 0, s_rtt_n = 0, s_rtt_max = 0;   // aller-retour CoCo -> PC -> CoCo (us) // premier octet gardé sans…
enum { NDW_OFF = 0, NDW_NOWIFI, NDW_NOSERVER, NDW_CONNECTING, NDW_LINKED, NDW_FAILED, NDW_BADNAME };

#ifndef ESP_PLATFORM
static uint32_t millis(void) { return 0; }
#endif

bool NetDw_Active(void) { return g_netdw_on && s_ndw_tx && s_ndw_rx; }

void NetDw_Put(uint8_t b)
{
  if (!s_ndw_tx) return;
  // sans connexion (démarrage : le CoCo parle avant que le Wi-Fi et le serveur soient prêts), l'octet
  // est gardé
  if (!s_ndw_linked && s_ndw_tx_h == s_ndw_tx_t) s_ndw_unlinked_ms = millis();
  const uint16_t n = (uint16_t)((s_ndw_tx_h + 1u) % NDW_TXQ);
  if (n == s_ndw_tx_t) { s_ndw_lost++; return; }
  s_ndw_tx[s_ndw_tx_h] = b;
  __sync_synchronize();
  s_ndw_tx_h = n;
  { extern void Net_WorkerWake(void); Net_WorkerWake(); }   // tâche réseau réveillée tout de suite
}

bool NetDw_Ready(void)
{
  if (!s_ndw_rx) return false;
  if (s_ndw_rx_flush) { s_ndw_rx_t = s_ndw_flush_to; s_ndw_rx_flush = false; }
  return s_ndw_rx_t != s_ndw_rx_h;
}

int NetDw_Get(void)
{
  if (!NetDw_Ready()) return -1;
  const uint8_t b = s_ndw_rx[s_ndw_rx_t];
  __sync_synchronize();
  s_ndw_rx_t = (uint16_t)((s_ndw_rx_t + 1u) % NDW_RXQ);
  return b;
}

void NetDw_Flush(void) { if (s_ndw_rx) s_ndw_rx_t = s_ndw_rx_h; }

bool        NetDw_Enabled(void) { return g_netdw_on; }
const char *NetDw_Server(void)  { return g_netdw_host; }
bool        NetDw_Linked(void)  { return s_ndw_linked; }

static void NetDw_Split(char *host, size_t hn, uint16_t *port)
{
  strncpy(host, g_netdw_host, hn - 1); host[hn - 1] = 0;
  *port = 65504;
  char *c = strrchr(host, ':');
  if (c) { const long v = strtol(c + 1, nullptr, 10); *c = 0; if (v > 0 && v < 65536) *port = (uint16_t)v; }
}

void NetDw_StatusLine(char *buf, size_t n)
{
  char host[64]; uint16_t port;
  NetDw_Split(host, sizeof host, &port);
  switch (g_netdw_on ? s_ndw_state : NDW_OFF)
  {
    case NDW_OFF:        snprintf(buf, n, "OFF (Becker port = SD disks /x0-/x3)"); break;
    case NDW_NOWIFI:     snprintf(buf, n, "ON, waiting for Wi-Fi"); break;
    case NDW_NOSERVER:   snprintf(buf, n, "ON, no server address (press D)"); break;
    case NDW_CONNECTING: snprintf(buf, n, "CONNECTING to %.24s:%u...", host, (unsigned)port); break;
    case NDW_LINKED:     snprintf(buf, n, "CONNECTED %.24s:%u", host, (unsigned)port); break;
    case NDW_BADNAME:    snprintf(buf, n, "UNKNOWN NAME %.24s, retrying", host); break;
    default:             snprintf(buf, n, "NO SERVER at %.20s:%u, retrying", host, (unsigned)port); break;
  }
}

#ifdef ESP_PLATFORM
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#define NDW_LOCAL_PORT 50604u                 // port local fixe (voir NetDw_Connect)
static TaskHandle_t s_ndw_task = nullptr;
static volatile bool s_ndw_close_req = false, s_ndw_closed_ack = false;   // fermeture propre avant redémarrage
static volatile bool s_ndw_restart = false;    // adresse changée : refaire la connexion

static void NetDw_Log(const char *fmt, ...)
{
  char b[160]; va_list a; va_start(a, fmt); vsnprintf(b, sizeof b, fmt, a); va_end(a); Log_Line(b);
}

// connexion non bloquante (DriveWire et modem partagent une seule tâche réseau une attente de 3 s ici
// bloquerait le modem). Rend le socket en cours de connexion, ou -1.
static int NetDw_ConnectStart(void)
{
  char host[64]; uint16_t port;
  NetDw_Split(host, sizeof host, &port);
  struct addrinfo hints; memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET; hints.ai_socktype = SOCK_STREAM;
  struct addrinfo *res = nullptr;
  char ps[8]; snprintf(ps, sizeof ps, "%u", (unsigned)port);
  if (getaddrinfo(host, ps, &hints, &res) != 0 || !res)
  {
    if (s_ndw_state != NDW_BADNAME) NetDw_Log("[DWNET] nom inconnu : %s", host);
    s_ndw_state = NDW_BADNAME;
    return -1;
  }
  const int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s < 0) { freeaddrinfo(res); s_ndw_state = NDW_FAILED; return -1; }
  // port local fixe. DriveWire 4 ne sert qu'un client à la fois et reste bloqué en lecture sur
  // l'ancienne connexion si la carte a été éteinte ou redémarrée sans la fermer : il ne prend jamais
  // la nouvelle
  {
    const int one = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in la; memset(&la, 0, sizeof la);
    la.sin_family = AF_INET; la.sin_port = htons(NDW_LOCAL_PORT); la.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (struct sockaddr *)&la, sizeof la) < 0) NetDw_Log("[DWNET] port local %u occupe : port au hasard", (unsigned)NDW_LOCAL_PORT);
  }
  fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
  const int r = connect(s, res->ai_addr, res->ai_addrlen);
  freeaddrinfo(res);
  if (r < 0 && errno != EINPROGRESS) { close(s); return -1; }
  return s;
}

// État de la partie DriveWire de la tâche réseau
static int      s_w_sock = -1, s_w_pend = -1;
static uint32_t s_w_next = 0, s_w_fails = 0, s_w_t0 = 0;
static uint8_t  s_w_buf[512];
static uint32_t s_w_busy_ms = 0;                     // dernier octet échangé (tâche réseau : 1 ms tant qu'il y a du trafic)

static void NetDw_Fail(void)
{
  if (s_ndw_state != NDW_BADNAME)
  {
    if (s_w_fails == 0) { char h[64]; uint16_t p; NetDw_Split(h, sizeof h, &p); NetDw_Log("[DWNET] pas de serveur DriveWire a %s:%u (nouvel essai chaque seconde, puis toutes les 5 s)", h, (unsigned)p); }
    s_ndw_state = NDW_FAILED;
  }
  s_w_fails++;
  s_w_next = millis() + (s_w_fails < 20 ? 1000 : 5000);   // 1 s au début (démarrage du CoCo), puis 5 s
}

static void NetDw_OnLinked(void)
{
  if (s_ndw_tx_t != s_ndw_tx_h && (uint32_t)(millis() - s_ndw_unlinked_ms) > 120000u)
  {                                                 // 2 min (HDB-DOS attend sans limite)
    const unsigned k = (unsigned)((s_ndw_tx_h + NDW_TXQ - s_ndw_tx_t) % NDW_TXQ);
    s_ndw_lost += k; s_ndw_tx_t = s_ndw_tx_h;
    NetDw_Log("[DWNET] %u octet(s) du CoCo trop anciens (plus de 2 min) : jetes", k);
  }
  else if (s_ndw_tx_t != s_ndw_tx_h)
    NetDw_Log("[DWNET] %u octet(s) du CoCo en attente envoyes au serveur", (unsigned)((s_ndw_tx_h + NDW_TXQ - s_ndw_tx_t) % NDW_TXQ));
  s_ndw_flush_to = s_ndw_rx_h; __sync_synchronize(); s_ndw_rx_flush = true;
  s_ndw_linked = true; s_ndw_links++; s_w_fails = 0;
  s_ndw_state = NDW_LINKED;
  { char h[64]; uint16_t p; NetDw_Split(h, sizeof h, &p); NetDw_Log("[DWNET] connecte au serveur DriveWire %s:%u", h, (unsigned)p); }
}

static void NetDw_CloseLinked(void)
{
  if (s_w_sock >= 0) { close(s_w_sock); s_w_sock = -1; }
  s_ndw_linked = false; s_ndw_flush_to = s_ndw_rx_h; __sync_synchronize(); s_ndw_rx_flush = true; s_ndw_tx_t = s_ndw_tx_h;
}

// Un pas de DriveWire par le réseau (jamais bloquant).
static void NetDw_Step(void)
{
  const uint32_t now = millis();
  if (s_ndw_close_req)                                // redémarrage demandé : FIN au serveur
  {
    if (s_w_sock >= 0) { shutdown(s_w_sock, SHUT_RDWR); close(s_w_sock); s_w_sock = -1; s_ndw_linked = false; }
    if (s_w_pend >= 0) { close(s_w_pend); s_w_pend = -1; }
    s_ndw_closed_ack = true;
    return;
  }
  const bool want = g_netdw_on && g_netdw_host[0] && Net_Connected() && !s_ndw_restart;
  if (!want || (s_w_sock >= 0 && s_ndw_restart))
  {
    if (s_w_sock >= 0)
    {
      NetDw_CloseLinked();
      NetDw_Log("[DWNET] connexion fermee (entree %lu o, sortie %lu o)", (unsigned long)s_ndw_in, (unsigned long)s_ndw_out);
    }
    if (s_w_pend >= 0) { close(s_w_pend); s_w_pend = -1; }
    s_ndw_restart = false; s_w_fails = 0; s_w_next = 0;
    s_ndw_state = !g_netdw_on ? NDW_OFF : !g_netdw_host[0] ? NDW_NOSERVER : !Net_Connected() ? NDW_NOWIFI : NDW_CONNECTING;
    return;
  }
  if (s_w_sock < 0)
  {
    if (s_w_pend < 0)
    {
      static bool s_wait_logged = false;               // diagnostic
      if (s_ndw_tx_t != s_ndw_tx_h && !s_wait_logged) { s_wait_logged = true; NetDw_Log("[DWNET] le CoCo a commence a parler au serveur : octets gardes jusqu a la connexion"); }
      if ((int32_t)(now - s_w_next) < 0) return;
      if (s_ndw_state != NDW_BADNAME && s_ndw_state != NDW_FAILED) s_ndw_state = NDW_CONNECTING;
      s_w_pend = NetDw_ConnectStart();
      if (s_w_pend < 0) { NetDw_Fail(); return; }
      s_w_t0 = now;
      return;
    }
    fd_set wf; FD_ZERO(&wf); FD_SET(s_w_pend, &wf);
    struct timeval tv = { 0, 0 };
    const int r = select(s_w_pend + 1, nullptr, &wf, nullptr, &tv);
    if (r == 0)
    {
      if (now - s_w_t0 >= 3000u) { close(s_w_pend); s_w_pend = -1; NetDw_Fail(); }   // 3 s pour répondre
      return;
    }
    int err = 0; socklen_t el = sizeof err;
    if (r < 0 || getsockopt(s_w_pend, SOL_SOCKET, SO_ERROR, &err, &el) < 0 || err)
    {
      close(s_w_pend); s_w_pend = -1; NetDw_Fail();
      return;
    }
    const int one = 1;
    setsockopt(s_w_pend, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);   // pas d'attente de Nagle : un secteur = un aller-retour
    setsockopt(s_w_pend, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof one);
    s_w_sock = s_w_pend; s_w_pend = -1;
    NetDw_OnLinked();
  }
  // Serveur -> CoCo
  const int sock = s_w_sock;
  fd_set rf; FD_ZERO(&rf); FD_SET(sock, &rf);
  struct timeval tv = { 0, 0 };
  const int r = select(sock + 1, &rf, nullptr, nullptr, &tv);
  bool drop = false;
  if (r > 0 && FD_ISSET(sock, &rf))
  {
    const uint16_t h = s_ndw_rx_h, t = s_ndw_rx_t;
    const uint32_t room = (uint32_t)((t + NDW_RXQ - h - 1u) % NDW_RXQ);
    if (room)
    {
      const int k = recv(sock, s_w_buf, room < sizeof s_w_buf ? room : sizeof s_w_buf, MSG_DONTWAIT);
      if (k == 0 || (k < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) drop = true;
      for (int i = 0; i < k; i++) { s_ndw_rx[(h + i) % NDW_RXQ] = s_w_buf[i]; }
      if (k > 0) { __sync_synchronize(); s_ndw_rx_h = (uint16_t)((h + k) % NDW_RXQ); s_ndw_in += (uint32_t)k; s_w_busy_ms = millis(); }
      if (k > 0 && s_rtt_t0)                          // première réponse après un envoi
      {
        const uint32_t us = (uint32_t)(esp_timer_get_time() - s_rtt_t0);
        s_rtt_t0 = 0; s_rtt_sum += us; s_rtt_n++; if (us > s_rtt_max) s_rtt_max = us;
      }
    }
  }
  else if (r < 0) drop = true;
  // CoCo -> serveur
  while (!drop && s_ndw_tx_t != s_ndw_tx_h)
  {
    const uint16_t t = s_ndw_tx_t, h = s_ndw_tx_h;
    const uint16_t n = (h > t) ? (uint16_t)(h - t) : (uint16_t)(NDW_TXQ - t);   // morceau contigu
    const int k = send(sock, s_ndw_tx + t, n, MSG_DONTWAIT);
    if (k < 0) { if (errno == EAGAIN || errno == EWOULDBLOCK) break; drop = true; break; }
    s_ndw_out += (uint32_t)k; s_w_busy_ms = millis();
    if (!s_rtt_t0) s_rtt_t0 = (uint32_t)esp_timer_get_time();
    s_ndw_tx_t = (uint16_t)((t + k) % NDW_TXQ);
    if (k < n) break;
  }
  if (drop)
  {
    NetDw_CloseLinked();
    s_ndw_state = NDW_FAILED;
    NetDw_Log("[DWNET] connexion perdue (serveur arrete ?) ; nouvel essai dans 2 s");
    s_w_next = millis() + 2000;
  }
}

// UNE tâche réseau pour DriveWire par le Wi-Fi et le modem de RAM interne ; la RAM DMA manquait au
// clavier USB). Pile en RAM interne obligatoire (lwIP).
extern void Modem_Step(void);   // modem.cpp : un pas du modem (rien s'il n'est pas en service)
// au repos (aucun trafic depuis 200 ms, modem en attente de commandes), la tâche ne tourne que toutes
// les 10 ms au lieu de chaque milliseconde : le coeur 0 (vidéo, USB Wi-Fi) était à 98 % dans Pinball
static volatile bool s_wk_sleep = false;
void Net_WorkerWake(void) { if (s_wk_sleep && s_ndw_task) { s_wk_sleep = false; xTaskNotifyGive(s_ndw_task); } }
extern bool Modem_Busy(void);
static void Net_Worker(void *)
{
  for (;;)
  {
    if (s_ndw_tx) NetDw_Step();
    Modem_Step();
    const bool busy = Modem_Busy() || s_w_pend >= 0 || (s_ndw_tx && s_ndw_tx_t != s_ndw_tx_h && s_w_sock >= 0)
                   || (s_w_sock >= 0 && millis() - s_w_busy_ms < 200u);
    if (busy) vTaskDelay(1);                            // 1 ms : délai maximal d'un octet vers le réseau
    else { s_wk_sleep = true; ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10)); s_wk_sleep = false; }
  }
}
void Net_WorkerEnsure(void)
{
  if (s_ndw_task) return;
  if (xTaskCreatePinnedToCore(Net_Worker, "NetWk", 4608, nullptr, 2, &s_ndw_task, 0) != pdPASS) s_ndw_task = nullptr;   // coeur 0, au-dessus du VideoCore
  else Log_Line("[NET] tache reseau demarree (DriveWire Wi-Fi + modem, pile 4,5 Ko en RAM interne)");
}
unsigned Net_WorkerStackFree(void) { return s_ndw_task ? (unsigned)uxTaskGetStackHighWaterMark(s_ndw_task) : 0u; }

static void NetDw_EnsureTask(void)
{
  if (!s_ndw_tx)
  {
    s_ndw_tx = (uint8_t *)heap_caps_malloc(NDW_TXQ, MALLOC_CAP_SPIRAM);
    s_ndw_rx = (uint8_t *)heap_caps_malloc(NDW_RXQ, MALLOC_CAP_SPIRAM);
    if (!s_ndw_tx || !s_ndw_rx) { free(s_ndw_tx); free(s_ndw_rx); s_ndw_tx = s_ndw_rx = nullptr; Log_Line("[DWNET] PSRAM insuffisante"); return; }
  }
  Net_WorkerEnsure();
}

void NetDw_Boot(void) { if (g_netdw_on) NetDw_EnsureTask(); }

// appelée par Sys_Restart : ferme la connexion (le serveur la libère)
void NetDw_Shutdown(void)
{
  if (!s_ndw_task) return;
  const bool was = s_ndw_linked;
  s_ndw_close_req = true;
  for (int i = 0; i < 30 && !s_ndw_closed_ack; i++) vTaskDelay(pdMS_TO_TICKS(10));
  if (was) vTaskDelay(pdMS_TO_TICKS(100));            // laisse partir le FIN avant la coupure du Wi-Fi
}

void NetDw_SetEnabled(bool on)
{
  g_netdw_on = on;
  Net_Save();
  if (on) NetDw_EnsureTask();
  NetDw_Log("[DWNET] DriveWire par le reseau %s (serveur %s)", on ? "en marche" : "arrete", g_netdw_host[0] ? g_netdw_host : "aucun");
}

void NetDw_SetServer(const char *hp)
{
  strncpy(g_netdw_host, hp ? hp : "", sizeof g_netdw_host - 1); g_netdw_host[sizeof g_netdw_host - 1] = 0;
  Net_Save();
  s_ndw_restart = true;
  NetDw_Log("[DWNET] serveur : %s", g_netdw_host[0] ? g_netdw_host : "aucun");
}

void NetDw_Report(void)
{
  char st[64]; NetDw_StatusLine(st, sizeof st);
  NetDw_Log("[DWNET] aller-retour moyen %lu us, maximum %lu us (%lu mesures) ; Wi-Fi sans veille : %s",
            (unsigned long)(s_rtt_n ? s_rtt_sum / s_rtt_n : 0), (unsigned long)s_rtt_max, (unsigned long)s_rtt_n,
            WiFi.getSleep() ? "non" : "oui");
  s_rtt_sum = s_rtt_n = s_rtt_max = 0;
  NetDw_Log("[DWNET] %s ; connexions %lu, recus %lu o, envoyes %lu o, perdus %lu", st,
            (unsigned long)s_ndw_links, (unsigned long)s_ndw_in, (unsigned long)s_ndw_out, (unsigned long)s_ndw_lost);
}

#else
// Banc : pas de réseau ; DWNETLOOP = le serveur intégré joue le PC (essai du chemin CPU).
void Net_WorkerWake(void) {}
void NetDw_Boot(void) {}
void NetDw_Shutdown(void) {}
void NetDw_SetEnabled(bool on) { g_netdw_on = on; }
void NetDw_SetServer(const char *hp) { strncpy(g_netdw_host, hp ? hp : "", sizeof g_netdw_host - 1); }
void NetDw_Report(void) {}
#endif
