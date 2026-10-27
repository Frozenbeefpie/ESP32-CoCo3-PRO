// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Modem Hayes par le Wi-Fi (voir modem.h).

#include "modem.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

void Log_Line(const char *s);   // main.cpp

#ifdef ESP_PLATFORM
#include <Arduino.h>
#include <Preferences.h>
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"
#include "cc3net.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
static uint32_t Mdm_Ms(void) { return (uint32_t)millis(); }
static void *Mdm_Mem(size_t n) { void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM); return p ? p : malloc(n); }
#else
// Banc PC : temps émulé, réseau simulé (voir plus bas)
extern volatile uint64_t g_emu_ticks;
static uint32_t Mdm_Ms(void) { return (uint32_t)(g_emu_ticks / 14318ull); }
static void *Mdm_Mem(size_t n) { return malloc(n); }
#endif
extern const char *const g_fw_version_short;   // main.cpp (FW_VERSION_SHORT)

// ---------------------------------------------------------------- files (un producteur, un consommateur)
#define MQ_IN  1024u                  // CoCo -> modem
#define MQ_OUT 8192u                  // modem -> CoCo (le CoCo lit à la vitesse du port série)
static uint8_t *s_in, *s_out;
static volatile uint16_t s_in_h, s_in_t, s_out_h, s_out_t;
static volatile uint32_t s_in_drop, s_out_drop;

static bool Mdm_Alloc(void)
{
  if (!s_in)  s_in  = (uint8_t *)Mdm_Mem(MQ_IN);
  if (!s_out) s_out = (uint8_t *)Mdm_Mem(MQ_OUT);
  return s_in && s_out;
}

void Modem_Put(uint8_t b)                       // coeur CPU
{
  if (!s_in) return;
  const uint16_t h = s_in_h, n = (uint16_t)((h + 1u) % MQ_IN);
  if (n == s_in_t) { s_in_drop++; return; }
  s_in[h] = b;
  __sync_synchronize();
  s_in_h = n;
#ifdef ESP_PLATFORM
  extern void Net_WorkerWake(void); Net_WorkerWake();
#endif
}
static bool in_get(uint8_t *b)                  // tâche du modem
{
  const uint16_t t = s_in_t;
  if (t == s_in_h) return false;
  *b = s_in[t];
  __sync_synchronize();
  s_in_t = (uint16_t)((t + 1u) % MQ_IN);
  return true;
}
static uint32_t out_room(void) { return (uint32_t)((s_out_t + MQ_OUT - s_out_h - 1u) % MQ_OUT); }
static void out_put(uint8_t b)                  // tâche du modem
{
  if (!s_out) return;
  const uint16_t h = s_out_h, n = (uint16_t)((h + 1u) % MQ_OUT);
  if (n == s_out_t) { s_out_drop++; return; }
  s_out[h] = b;
  __sync_synchronize();
  s_out_h = n;
}
static void out_str(const char *s) { while (*s) out_put((uint8_t)*s++); }

#ifndef ESP_PLATFORM
static void Mdm_Service(uint32_t now);
#endif
bool Modem_Ready(void)                          // coeur CPU
{
#ifndef ESP_PLATFORM
  Mdm_Service(Mdm_Ms());                        // banc : pas de tâche
#endif
  return s_out && s_out_t != s_out_h;
}
int Modem_Get(void)
{
  if (!s_out || s_out_t == s_out_h) return -1;
  const uint16_t t = s_out_t;
  const uint8_t b = s_out[t];
  __sync_synchronize();
  s_out_t = (uint16_t)((t + 1u) % MQ_OUT);
  return b;
}

// ---------------------------------------------------------------- réglages
struct MdmCfg
{
  uint8_t  echo, verbose, quiet, s0, dcd, dtr, telnet;
  uint16_t port;
  char     book[4][64];
};
static MdmCfg s_cfg;
static void Cfg_Factory(MdmCfg &c)
{
  memset(&c, 0, sizeof c);
  c.echo = 1; c.verbose = 1; c.quiet = 0; c.s0 = 0;
  c.dcd = 1;       // &C1 : DCD = porteuse
  c.dtr = 0;       // &D0 : DTR ignoré (les terminaux le basculent au démarrage)
  c.telnet = 1;
  c.port = 6400;
}
#ifdef ESP_PLATFORM
// la tâche du modem ne touche jamais à la flash (NVS) (une pile en PSRAM l'interdirait la pile est
// revenue en RAM interne, lwIP l'exige, mais la règle est gardée)
static MdmCfg s_saved;
static volatile bool s_save_req = false;
static void Cfg_Load(void) { s_cfg = s_saved; }
static bool Cfg_Save(void) { s_saved = s_cfg; s_save_req = true; return true; }
static void Cfg_ReadNvs(void)
{
  Cfg_Factory(s_cfg);
  s_saved = s_cfg;
  Preferences p;
  if (!p.begin("cc3m", true)) return;
  s_cfg.echo = p.getUChar("e", s_cfg.echo);   s_cfg.verbose = p.getUChar("v", s_cfg.verbose);
  s_cfg.quiet = p.getUChar("q", s_cfg.quiet); s_cfg.s0 = p.getUChar("s0", s_cfg.s0);
  s_cfg.dcd = p.getUChar("c", s_cfg.dcd);     s_cfg.dtr = p.getUChar("d", s_cfg.dtr);
  s_cfg.telnet = p.getUChar("t", s_cfg.telnet); s_cfg.port = p.getUShort("p", s_cfg.port);
  for (int i = 0; i < 4; i++)
  {
    char k[4] = { 'z', (char)('0' + i), 0, 0 };
    String v = p.getString(k, "");
    strncpy(s_cfg.book[i], v.c_str(), sizeof s_cfg.book[i] - 1);
  }
  p.end();
  s_saved = s_cfg;
}
static bool Cfg_WriteNvs(void)
{
  Preferences p;
  if (!p.begin("cc3m", false)) return false;
  p.putUChar("e", s_saved.echo); p.putUChar("v", s_saved.verbose); p.putUChar("q", s_saved.quiet);
  p.putUChar("s0", s_saved.s0); p.putUChar("c", s_saved.dcd); p.putUChar("d", s_saved.dtr);
  p.putUChar("t", s_saved.telnet); p.putUShort("p", s_saved.port);
  for (int i = 0; i < 4; i++) { char k[4] = { 'z', (char)('0' + i), 0, 0 }; p.putString(k, s_saved.book[i]); }
  p.end();
  return true;
}
#else
static MdmCfg s_saved; static bool s_saved_ok = false;
static void Cfg_Load(void) { if (s_saved_ok) s_cfg = s_saved; else Cfg_Factory(s_cfg); }
static bool Cfg_Save(void) { s_saved = s_cfg; s_saved_ok = true; return true; }
#endif

// ---------------------------------------------------------------- couche réseau
// nl_dial : >= 0 = connexion ; -1 pas de réponse, -2 occupé (refusé), -3 nom inconnu, -4 abandon
// (touche)
static bool nl_wifi(void);
static int  nl_dial(const char *host, uint16_t port, bool (*abort_cb)(void));
static int  nl_send(int s, const uint8_t *p, int n);   // octets envoyés (>= 0) ou -1 (fermée)
static int  nl_recv(int s, uint8_t *p, int n);         // > 0 octets, 0 rien, -1 fermée
static bool nl_closed(int s);                          // l'autre bout a fermé (sans lire les données)
static void nl_close(int s);
static int  nl_accept(uint16_t port);                  // appel entrant (>= 0) ou -1

// ---------------------------------------------------------------- état
enum { ST_CMD = 0, ST_ONLINE, ST_RINGING };
static uint8_t  s_st = ST_CMD;
static int      s_sock = -1, s_ring_sock = -1;
static char     s_peer[72];
static char     s_acc_ip[24] = "";       // adresse de l'appel entrant
static char     s_line[96], s_last[96];
static uint8_t  s_len = 0;
static bool     s_prev_a = false;            // pour « A/ »
static uint32_t s_last_data = 0, s_plus_t = 0;
static uint8_t  s_plus = 0;
static uint8_t  s_rings = 0;
static uint32_t s_next_ring = 0;
static volatile bool s_dtr = true, s_dtr_seen_off = false;
static volatile bool s_shutdown = false;
static uint8_t  s_tn = 0, s_tn_cmd = 0;      // telnet : état de l'analyseur
static uint32_t st_out = 0, st_in = 0, st_calls = 0, st_answers = 0;
static bool     s_started = false;

bool Modem_Carrier(void) { return s_cfg.dcd == 0 || s_sock >= 0; }
void Modem_SetDtr(bool on) { if (!on && s_dtr) s_dtr_seen_off = true; s_dtr = on; }
void Modem_Shutdown(void) { s_shutdown = true; }

static void Mdm_Log(const char *fmt, const char *a = "", unsigned long n = 0)
{
  char b[160]; snprintf(b, sizeof b, fmt, a, n); Log_Line(b);
}

enum { MR_OK = 0, MR_CONNECT = 1, MR_RING = 2, MR_NOCARRIER = 3, MR_ERROR = 4, MR_NODIALTONE = 6, MR_BUSY = 7, MR_NOANSWER = 8 };
static void Result(int code)
{
  if (s_cfg.quiet) return;
  static const char *const k_txt[9] = { "OK", "CONNECT", "RING", "NO CARRIER", "ERROR", "", "NO DIALTONE", "BUSY", "NO ANSWER" };
  char b[24];
  if (s_cfg.verbose) snprintf(b, sizeof b, "\r\n%s\r\n", k_txt[code]);
  else               snprintf(b, sizeof b, "%d\r", code);
  out_str(b);
}
static void Info(const char *s) { out_str("\r\n"); out_str(s); }

static void Hangup(void)
{
  if (s_sock >= 0) { nl_close(s_sock); s_sock = -1; Mdm_Log("[MODEM] raccroche (%s)", s_peer); }
  s_st = ST_CMD;
  s_plus = 0;
  s_tn = 0;
}

static bool Abort_Key(void)                   // pendant la composition : une touche abandonne
{
  uint8_t b;
  if (in_get(&b)) return true;
  return s_shutdown;
}

static void Telnet_Greet(void)               // appel entrant : le CoCo fait l'écho et le mode caractère
{
  if (!s_cfg.telnet || s_sock < 0) return;
  static const uint8_t g[] = { 255, 251, 1, 255, 251, 3 };   // IAC WILL ECHO, IAC WILL SGA
  nl_send(s_sock, g, sizeof g);
}

static void Answer(void)
{
  if (s_ring_sock < 0) { Result(MR_NOCARRIER); return; }
  s_sock = s_ring_sock; s_ring_sock = -1;
  s_st = ST_ONLINE; s_plus = 0; s_tn = 0; s_last_data = Mdm_Ms();
  st_answers++;
  Mdm_Log("[MODEM] appel entrant decroche (%s)", s_peer);
  Telnet_Greet();
  Result(MR_CONNECT);
}

static void Dial(const char *arg)
{
  char host[72];
  while (*arg == ' ' || *arg == 'T' || *arg == 't' || *arg == 'P' || *arg == 'p') arg++;
  if ((*arg == 'S' || *arg == 's') && (arg[1] == 0 || (isdigit((unsigned char)arg[1]) && (arg[2] == 0 || arg[2] == ' '))))
  {
    int n = 0;
    if (isdigit((unsigned char)arg[1])) n = arg[1] - '0';
    if (n < 0 || n > 3 || !s_cfg.book[n][0]) { Result(MR_ERROR); return; }
    strncpy(host, s_cfg.book[n], sizeof host - 1); host[sizeof host - 1] = 0;
  }
  else
  {
    size_t k = 0;
    for (; *arg && k < sizeof host - 1; arg++) if (*arg != ' ' && *arg != ',' && *arg != ';' && *arg != '"') host[k++] = *arg;
    host[k] = 0;
  }
  if (!host[0]) { Result(MR_ERROR); return; }
  if (s_sock >= 0) Hangup();
  if (!nl_wifi()) { Result(MR_NODIALTONE); return; }
  uint16_t port = 23;
  char *c = strrchr(host, ':');
  if (c) { const long v = strtol(c + 1, nullptr, 10); *c = 0; if (v > 0 && v < 65536) port = (uint16_t)v; }
  snprintf(s_peer, sizeof s_peer, "%.60s:%u", host, (unsigned)port);
  Mdm_Log("[MODEM] compose %s", s_peer);
  const int s = nl_dial(host, port, Abort_Key);
  if (s < 0)
  {
    Mdm_Log(s == -2 ? "[MODEM] %s : occupe (connexion refusee)" : s == -3 ? "[MODEM] %s : nom inconnu" :
            s == -4 ? "[MODEM] %s : abandon" : "[MODEM] %s : pas de reponse", s_peer);
    Result(s == -2 ? MR_BUSY : s == -4 ? MR_NOCARRIER : MR_NOANSWER);
    return;
  }
  s_sock = s; s_st = ST_ONLINE; s_plus = 0; s_tn = 0; s_last_data = Mdm_Ms();
  st_calls++;
  Mdm_Log("[MODEM] connecte a %s", s_peer);
  Result(MR_CONNECT);
}

static void View(void)
{
  char b[100];
  snprintf(b, sizeof b, "E%u V%u Q%u S0=%u &C%u &D%u NET%u *P%u",
           s_cfg.echo, s_cfg.verbose, s_cfg.quiet, s_cfg.s0, s_cfg.dcd, s_cfg.dtr ? 2 : 0, s_cfg.telnet, (unsigned)s_cfg.port);
  Info(b);
  for (int i = 0; i < 4; i++) { snprintf(b, sizeof b, "&Z%d=%s", i, s_cfg.book[i]); Info(b); }
}

static int Num(const char *&p)
{
  int v = 0;
  while (isdigit((unsigned char)*p)) v = v * 10 + (*p++ - '0');
  return v;
}

// Une ligne de commandes (le résultat est envoyé ici).
static void Command(const char *line)
{
  while (*line == ' ') line++;
  if (!((line[0] == 'A' || line[0] == 'a') && (line[1] == 'T' || line[1] == 't'))) return;   // pas « AT » : ignoré
  strncpy(s_last, line, sizeof s_last - 1); s_last[sizeof s_last - 1] = 0;
  const char *p = line + 2;
  for (;;)
  {
    while (*p == ' ') p++;
    const char c = (char)toupper((unsigned char)*p);
    if (!c) break;
    p++;
    switch (c)
    {
      case 'Z': Hangup(); Cfg_Load(); break;
      case 'E': s_cfg.echo = Num(p) ? 1 : 0; break;
      case 'V': s_cfg.verbose = Num(p) ? 1 : 0; break;
      case 'Q': s_cfg.quiet = Num(p) ? 1 : 0; break;
      case 'L': case 'M': case 'X': case 'B': case 'W': (void)Num(p); break;   // sans objet
      case 'H': (void)Num(p); Hangup(); break;
      case 'I':
      {
        (void)Num(p);
        char b[100];
        snprintf(b, sizeof b, "ESP32-CoCo3 Wi-Fi modem (v%s)", g_fw_version_short); Info(b);
#ifdef ESP_PLATFORM
        char w[64]; Net_StatusLine(w, sizeof w); snprintf(b, sizeof b, "Wi-Fi: %s", w); Info(b);
#endif
        if (s_cfg.port) { snprintf(b, sizeof b, "Incoming calls: TCP port %u", (unsigned)s_cfg.port); Info(b); }
        break;
      }
      case 'O':
        (void)Num(p);
        if (s_sock >= 0) { s_st = ST_ONLINE; s_plus = 0; s_last_data = Mdm_Ms(); Result(MR_CONNECT); }
        else Result(MR_NOCARRIER);
        return;
      case 'A':
        if (s_st == ST_RINGING) Answer(); else Result(MR_NOCARRIER);
        return;
      case 'D':
        Dial(p);
        return;
      case 'S':
      {
        const int r = Num(p);
        if (*p == '=') { p++; const int v = Num(p); if (r == 0) s_cfg.s0 = (uint8_t)(v > 255 ? 255 : v); }
        else if (*p == '?') { p++; char b[8]; snprintf(b, sizeof b, "%03u", r == 0 ? s_cfg.s0 : 0u); Info(b); }
        break;
      }
      case 'N':
        if (toupper((unsigned char)p[0]) == 'E' && toupper((unsigned char)p[1]) == 'T') { p += 2; s_cfg.telnet = Num(p) ? 1 : 0; break; }
        Result(MR_ERROR); return;
      case '*':
        if (toupper((unsigned char)*p) == 'P') { p++; const int v = Num(p); s_cfg.port = (uint16_t)(v > 65535 ? 0 : v); break; }
        Result(MR_ERROR); return;
      case '&':
      {
        const char d = (char)toupper((unsigned char)*p);
        if (!d) { Result(MR_ERROR); return; }
        p++;
        if (d == 'F') { Cfg_Factory(s_cfg); }
        else if (d == 'W') { (void)Num(p); if (!Cfg_Save()) { Result(MR_ERROR); return; } }
        else if (d == 'V') { (void)Num(p); View(); }
        else if (d == 'C') { s_cfg.dcd = Num(p) ? 1 : 0; }
        else if (d == 'D') { const int v = Num(p); s_cfg.dtr = (v >= 2) ? 1 : 0; }
        else if (d == 'Z')
        {
          const int n = Num(p);
          if (*p != '=' || n < 0 || n > 3) { Result(MR_ERROR); return; }
          p++;
          size_t k = 0;
          for (; *p && k < sizeof s_cfg.book[n] - 1; p++) if (*p != ' ') s_cfg.book[n][k++] = *p;
          s_cfg.book[n][k] = 0;
          Result(MR_OK); return;                  // le reste de la ligne est l'adresse
        }
        else { (void)Num(p); }                   // &K, &S... : sans objet
        break;
      }
      default:
        Result(MR_ERROR); return;
    }
  }
  Result(MR_OK);
}

static void Cmd_Char(uint8_t b)
{
  b &= 0x7F;
  if (s_len == 1 && s_prev_a && b == '/')        // « A/ » : répéter la dernière commande
  {
    if (s_cfg.echo) out_put(b);
    s_prev_a = false; s_len = 0;
    out_str("\r\n");
    char l[96]; strncpy(l, s_last, sizeof l); l[sizeof l - 1] = 0;
    Command(l);
    return;
  }
  if (b == '\r')
  {
    if (s_cfg.echo) out_put('\r');
    s_line[s_len] = 0;
    s_len = 0; s_prev_a = false;
    Command(s_line);
    return;
  }
  if (b == '\n') return;
  if (b == 8 || b == 127)
  {
    if (s_len) { s_len--; if (s_cfg.echo) out_str("\b \b"); }
    return;
  }
  if (b < 32) return;
  if (s_cfg.echo) out_put(b);
  if (s_len == 0 && (b == 'A' || b == 'a')) { s_prev_a = true; s_line[s_len++] = (char)b; return; }
  s_prev_a = false;
  if (s_len < sizeof s_line - 1) s_line[s_len++] = (char)b;
}

// Données du serveur vers le CoCo : négociation telnet retirée.
static uint8_t s_tn_reply[48]; static int s_tn_rn = 0;
static void Tn_Reply(uint8_t cmd, uint8_t opt)
{
  if (s_tn_rn + 3 > (int)sizeof s_tn_reply) return;
  s_tn_reply[s_tn_rn++] = 255; s_tn_reply[s_tn_rn++] = cmd; s_tn_reply[s_tn_rn++] = opt;
}
static void From_Net(const uint8_t *d, int n)
{
  for (int i = 0; i < n; i++)
  {
    const uint8_t b = d[i];
    if (!s_cfg.telnet) { out_put(b); continue; }
    switch (s_tn)
    {
      case 0: if (b == 255) s_tn = 1; else out_put(b); break;
      case 1:                                   // après IAC
        if (b == 255) { out_put(255); s_tn = 0; }
        else if (b >= 251 && b <= 254) { s_tn_cmd = b; s_tn = 2; }
        else if (b == 250) s_tn = 3;            // SB ... IAC SE
        else s_tn = 0;                          // NOP, GA, etc.
        break;
      case 2:                                   // option de WILL / WONT / DO / DONT
        // On accepte que le serveur fasse l'écho et supprime GA ; on refuse le reste.
        if (s_tn_cmd == 251) Tn_Reply((b == 1 || b == 3) ? 253 : 254, b);        // WILL -> DO / DONT
        else if (s_tn_cmd == 253) Tn_Reply(b == 3 ? 251 : 252, b);                // DO -> WILL (SGA) / WONT
        s_tn = 0;
        break;
      case 3: if (b == 255) s_tn = 4; break;    // dans SB
      case 4: s_tn = (b == 240) ? 0 : 3; break; // IAC SE
    }
  }
}

static void Online_Char(uint8_t b, uint32_t now, uint8_t *tx, int &tn)
{
  if (b == '+' && (s_plus > 0 ? (now - s_plus_t) < 1000u : (now - s_last_data) >= 1000u) && s_plus < 3)
  {
    if (s_plus == 0) s_plus_t = now;
    s_plus++;
  }
  else s_plus = 0;
  s_last_data = now;
  if (s_cfg.telnet && b == 255) tx[tn++] = 255;   // $FF doublé
  tx[tn++] = b;
}

static void Mdm_Service(uint32_t now)
{
  if (s_shutdown) { if (s_sock >= 0) Hangup(); if (s_ring_sock >= 0) { nl_close(s_ring_sock); s_ring_sock = -1; } return; }

  // DTR (AT&D2) : le CoCo coupe DTR -> raccrocher
  if (s_dtr_seen_off) { s_dtr_seen_off = false; if (s_cfg.dtr && s_sock >= 0) { Hangup(); Result(MR_NOCARRIER); } }

  // Octets du CoCo
  uint8_t tx[256 + 256]; int tn = 0;
  uint8_t b;
  while (tn < 256 && in_get(&b))
  {
    if (s_st == ST_ONLINE) Online_Char(b, now, tx, tn);
    else Cmd_Char(b);
  }
  if (tn && s_sock >= 0)
  {
    int k = 0;
    while (k < tn) { const int r = nl_send(s_sock, tx + k, tn - k); if (r < 0) { k = -1; break; } if (r == 0) break; k += r; }
    if (k > 0) st_out += (uint32_t)k;
    if (k < 0) { Hangup(); Result(MR_NOCARRIER); }
  }
  // +++ suivi d'une seconde de silence : mode commande, la connexion reste
  if (s_st == ST_ONLINE && s_plus == 3 && (now - s_last_data) >= 1000u)
  {
    s_plus = 0; s_st = ST_CMD; s_len = 0;
    Result(MR_OK);
  }
  // Serveur -> CoCo (en ligne seulement ; en mode commande les données attendent dans TCP)
  if (s_st == ST_ONLINE && s_sock >= 0)
  {
    uint32_t room = out_room();
    if (room > 64)
    {
      uint8_t rb[512];
      const int r = nl_recv(s_sock, rb, (int)(room - 32 < sizeof rb ? room - 32 : sizeof rb));
      if (r > 0) { st_in += (uint32_t)r; From_Net(rb, r); }
      else if (r < 0) { Hangup(); Result(MR_NOCARRIER); }
    }
    if (s_tn_rn && s_sock >= 0) { nl_send(s_sock, s_tn_reply, s_tn_rn); s_tn_rn = 0; }
  }
  else if (s_st == ST_CMD && s_sock >= 0 && nl_closed(s_sock))
  {
    Hangup();                                    // l'autre bout a raccroché pendant le mode commande
  }
  // Appels entrants
  const int a = nl_accept(s_cfg.port);
  if (a >= 0)
  {
    if (s_st == ST_CMD && s_sock < 0 && s_ring_sock < 0)
    {
      s_ring_sock = a; s_st = ST_RINGING; s_rings = 0; s_next_ring = now;
      snprintf(s_peer, sizeof s_peer, "%s", s_acc_ip[0] ? s_acc_ip : "appel entrant");
      Mdm_Log("[MODEM] appel entrant de %s (port %lu)", s_peer, (unsigned long)s_cfg.port);
    }
    else
    {
      static const char k_busy[] = "\r\nBUSY - the CoCo is on another call.\r\n";
      nl_send(a, (const uint8_t *)k_busy, (int)sizeof k_busy - 1);
      nl_close(a);
    }
  }
  if (s_st == ST_RINGING)
  {
    if (s_ring_sock >= 0 && nl_closed(s_ring_sock)) { nl_close(s_ring_sock); s_ring_sock = -1; s_st = ST_CMD; }
    else if ((int32_t)(now - s_next_ring) >= 0)
    {
      if (s_rings >= 10) { nl_close(s_ring_sock); s_ring_sock = -1; s_st = ST_CMD; }
      else
      {
        s_rings++; s_next_ring = now + 3000u;
        Result(MR_RING);
        if (s_cfg.s0 && s_rings >= s_cfg.s0) Answer();
      }
    }
  }
}

void Modem_StatusLine(char *buf, size_t n)
{
  if (s_sock >= 0) snprintf(buf, n, "%s %.40s", s_st == ST_ONLINE ? "CONNECTED" : "ON HOLD (+++)", s_peer);
  else if (s_st == ST_RINGING) snprintf(buf, n, "RINGING (ATA to answer)");
  else snprintf(buf, n, "IDLE (AT commands)");
}

static unsigned Mdm_StackFree(void);
void Modem_Report(char *buf, size_t n)
{
  char st[64]; Modem_StatusLine(st, sizeof st);
  snprintf(buf, n, "[MODEM] %s ; appels %lu sortants / %lu entrants ; octets vers le reseau %lu, du reseau %lu ; perdus %lu/%lu ; ecoute port %u, telnet %s, DCD %s",
           s_in ? st : "inactif", (unsigned long)st_calls, (unsigned long)st_answers, (unsigned long)st_out, (unsigned long)st_in,
           (unsigned long)s_in_drop, (unsigned long)s_out_drop, (unsigned)s_cfg.port, s_cfg.telnet ? "oui" : "non",
           s_cfg.dcd ? "porteuse" : "toujours");
  const size_t k = strlen(buf);
  if (k + 40 < n) snprintf(buf + k, n - k, " ; pile libre min %u o", Mdm_StackFree());   // réglage de la pile
}

// ================================================================ ESP32 : tâche et sockets
#ifdef ESP_PLATFORM
static int s_lsock = -1;
static uint16_t s_lport = 0;

static bool nl_wifi(void) { return Net_Connected(); }

static int nl_dial(const char *host, uint16_t port, bool (*abort_cb)(void))
{
  struct addrinfo hints; memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET; hints.ai_socktype = SOCK_STREAM;
  struct addrinfo *res = nullptr;
  char ps[8]; snprintf(ps, sizeof ps, "%u", (unsigned)port);
  if (getaddrinfo(host, ps, &hints, &res) != 0 || !res) return -3;
  const int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s < 0) { freeaddrinfo(res); return -1; }
  fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
  int r = connect(s, res->ai_addr, res->ai_addrlen);
  freeaddrinfo(res);
  if (r < 0 && errno != EINPROGRESS) { close(s); return -1; }
  for (int i = 0; i < 150; i++)                 // 15 s
  {
    if (abort_cb && abort_cb()) { close(s); return -4; }
    fd_set wf; FD_ZERO(&wf); FD_SET(s, &wf);
    struct timeval tv = { 0, 100000 };
    r = select(s + 1, nullptr, &wf, nullptr, &tv);
    if (r > 0)
    {
      int err = 0; socklen_t el = sizeof err;
      getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &el);
      if (err) { close(s); return err == ECONNREFUSED ? -2 : -1; }
      const int one = 1;
      setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
      setsockopt(s, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof one);
      return s;
    }
    if (r < 0) { close(s); return -1; }
  }
  close(s);
  return -1;
}

static int nl_send(int s, const uint8_t *p, int n)
{
  const int r = send(s, p, n, MSG_DONTWAIT);
  if (r >= 0) return r;
  return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
}
static int nl_recv(int s, uint8_t *p, int n)
{
  const int r = recv(s, p, n, MSG_DONTWAIT);
  if (r > 0) return r;
  if (r == 0) return -1;
  return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
}
static bool nl_closed(int s)
{
  uint8_t b;
  const int r = recv(s, &b, 1, MSG_DONTWAIT | MSG_PEEK);
  return r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK);
}
static void nl_close(int s) { if (s >= 0) { shutdown(s, SHUT_RDWR); close(s); } }

static int nl_accept(uint16_t port)
{
  const bool want = port && Net_Connected();
  if (s_lsock >= 0 && (!want || port != s_lport)) { close(s_lsock); s_lsock = -1; }
  if (!want) return -1;
  if (s_lsock < 0)
  {
    static uint32_t s_retry = 0;
    if ((int32_t)(millis() - s_retry) < 0) return -1;
    s_retry = millis() + 5000u;
    const int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s < 0) return -1;
    const int one = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in la; memset(&la, 0, sizeof la);
    la.sin_family = AF_INET; la.sin_port = htons(port); la.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (struct sockaddr *)&la, sizeof la) < 0 || listen(s, 1) < 0) { close(s); return -1; }
    fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
    s_lsock = s; s_lport = port;
    Mdm_Log("[MODEM] appels entrants : ecoute sur le port %s%lu", "", (unsigned long)port);
  }
  struct sockaddr_in ra; socklen_t rl = sizeof ra;
  const int c = accept(s_lsock, (struct sockaddr *)&ra, &rl);
  if (c < 0) return -1;
  inet_ntoa_r(ra.sin_addr, s_acc_ip, sizeof s_acc_ip);
  fcntl(c, F_SETFL, fcntl(c, F_GETFL, 0) | O_NONBLOCK);
  const int one = 1;
  setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
  setsockopt(c, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof one);
  return c;
}

// plus de tâche à lui : le modem tourne dans la tâche réseau partagée avec DriveWire par le Wi-Fi
// (cc3net.cpp), pour rendre 3,5 Ko de RAM interne au clavier USB.
extern void     Net_WorkerEnsure(void);
extern unsigned Net_WorkerStackFree(void);
static unsigned Mdm_StackFree(void) { return Net_WorkerStackFree(); }
static volatile bool s_run = false;
void Modem_Step(void) { if (s_run) Mdm_Service(Mdm_Ms()); }
// la tâche réseau reste à 1 ms : en ligne, sonnerie, octets en attente, sortie non vidée
bool Modem_Busy(void) { return s_run && (s_sock >= 0 || s_ring_sock >= 0 || s_st != ST_CMD || s_in_t != s_in_h || (s_out && s_out_t != s_out_h)); }

void Modem_Use(void)
{
  if (s_run || !Mdm_Alloc()) return;
  if (!s_started) { s_started = true; Cfg_ReadNvs(); }
  __sync_synchronize(); s_run = true;
  Net_WorkerEnsure();
  { char b[120]; snprintf(b, sizeof b, "[MODEM] en service (tache reseau partagee) ; RAM interne DMA libre %u o",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)); Log_Line(b); }
}
void Modem_MainPoll(void)                       // loop : écriture AT&W en flash
{
  if (s_save_req) { s_save_req = false; Cfg_WriteNvs(); }
}

#else
// ================================================================ banc PC : réseau simulé
// « Serveur » intégré : envoie une bannière (avec une négociation telnet) puis renvoie en écho ce
// qu'il reçoit. ATDT busy -> occupé, ATDT nowhere -> pas de réponse.
static bool    s_fake_up = false, s_fake_closed = false;
static uint8_t s_fake_q[4096]; static int s_fake_n = 0;
static bool nl_wifi(void) { return getenv("MDMNOWIFI") == nullptr; }
static int  nl_dial(const char *host, uint16_t, bool (*)(void))
{
  if (!strcmp(host, "busy")) return -2;
  if (!strcmp(host, "nowhere")) return -1;
  s_fake_up = true; s_fake_closed = false; s_fake_n = 0;
  static const uint8_t ban[] = { 255, 253, 24, 255, 251, 1, 'W', 'E', 'L', 'C', 'O', 'M', 'E', ' ', 255, 255, '\r', '\n' };
  memcpy(s_fake_q, ban, sizeof ban); s_fake_n = sizeof ban;
  return 7;
}
static int  nl_send(int, const uint8_t *p, int n)
{
  if (s_fake_closed) return -1;
  for (int i = 0; i < n; i++)
  {
    if (getenv("MDMDBG")) fprintf(stderr, "[NET TX] $%02X\n", p[i]);
    if (p[i] == 'Q' && !getenv("MDMNOQUIT")) { s_fake_closed = true; return i + 1; }   // « Q » : le serveur raccroche
    if (s_fake_n < (int)sizeof s_fake_q) s_fake_q[s_fake_n++] = p[i];
  }
  return n;
}
static int  nl_recv(int, uint8_t *p, int n)
{
  if (s_fake_n == 0) return s_fake_closed ? -1 : 0;
  const int k = n < s_fake_n ? n : s_fake_n;
  memcpy(p, s_fake_q, k); memmove(s_fake_q, s_fake_q + k, s_fake_n - k); s_fake_n -= k;
  return k;
}
static bool nl_closed(int) { return s_fake_closed && s_fake_n == 0; }
static void nl_close(int) { s_fake_up = false; s_fake_n = 0; }
static int  nl_accept(uint16_t)
{
  static bool once = false;
  if (!once && getenv("MDMRING") && Mdm_Ms() > (uint32_t)atoi(getenv("MDMRING"))) { once = true; s_fake_up = true; s_fake_closed = false; s_fake_n = 0; return 9; }
  return -1;
}
static unsigned Mdm_StackFree(void) { return 0; }
void Modem_Use(void) { Mdm_Alloc(); if (!s_started) { s_started = true; Cfg_Load(); } }
void Modem_Step(void) {}
bool Modem_Busy(void) { return false; }
void Modem_MainPoll(void) {}
#endif
