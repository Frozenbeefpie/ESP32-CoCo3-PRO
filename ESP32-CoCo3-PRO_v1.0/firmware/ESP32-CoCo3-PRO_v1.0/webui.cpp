// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur : Luc Dion 2026-10-26.

// Page web : la carte SD vue d'un navigateur (voir webui.h).

#include "webui.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <strings.h>

#ifdef ESP_PLATFORM
#include <Arduino.h>
#include <Preferences.h>
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "lwip/sockets.h"
#include "cc3net.h"
extern bool SD_MMC_MountOK;
extern void Sd_Lock(void);                     // main.cpp : verrou de la carte SD
extern void Sd_Unlock(void);
extern int  Sd_MountedPaths(const char **out, int max);   // main.cpp : disques montés
extern void Log_Line(const char *s);
extern const char *const g_fw_version_short;
static uint32_t W_Ms(void) { return (uint32_t)millis(); }
extern bool Emu_Running(void);                 // main.cpp : jeu en marche (ni menu F12, ni arrêt)
static bool W_EmuRun(void) { return Emu_Running(); }
static void *W_Mem(size_t n) { void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM); return p ? p : malloc(n); }
static void *W_Realloc(void *o, size_t n) { void *p = heap_caps_realloc(o, n, MALLOC_CAP_SPIRAM); return p ? p : realloc(o, n); }
#define W_ROOT "/sdcard"
static bool W_SdOk(void) { return SD_MMC_MountOK; }
static bool W_NetOk(void) { return Net_Connected(); }
static int  W_Mounted(const char **out, int max) { return Sd_MountedPaths(out, max); }
static void W_Ram(char *b, size_t n)
{
  snprintf(b, n, "\"int\":%u,\"dma\":%u,\"dmablk\":%u,\"psram\":%u",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}
static void W_Ip(char *b, size_t n);
#else
// ---------------------------------------------------------------- PC (essais)
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <time.h>
static uint32_t W_Ms(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (uint32_t)(t.tv_sec * 1000u + t.tv_nsec / 1000000u); }
static void *W_Mem(size_t n) { return malloc(n); }
static void *W_Realloc(void *o, size_t n) { return realloc(o, n); }
static const char *W_RootPc(void) { const char *r = getenv("WEBROOT"); return r ? r : "."; }
#define W_ROOT W_RootPc()
static void Sd_Lock(void) {}
static void Sd_Unlock(void) {}
static bool W_EmuRun(void) { return getenv("WEBSLOW") != nullptr; }   // essai du débit réduit
static void Log_Line(const char *s) { fprintf(stderr, "%s\n", s); }
static const char *const g_fw_version_short = "PC";
static bool W_SdOk(void) { return true; }
static bool W_NetOk(void) { return true; }
static int  W_Mounted(const char **out, int max)
{
  static const char *m = getenv("WEBMOUNT");   // un chemin monté (essai du verrou)
  if (m && max > 0) { out[0] = m; return 1; }
  return 0;
}
static void W_Ram(char *b, size_t n) { snprintf(b, n, "\"int\":0,\"dma\":0,\"dmablk\":0,\"psram\":0"); }
static void W_Ip(char *b, size_t n) { snprintf(b, n, "127.0.0.1"); }
#endif

#ifndef WEB_PORT
#define WEB_PORT 80
#endif
#define W_HDR   2048u
#define W_IO    4096u
#define W_SLOW_MS 30u     // pendant un jeu, un morceau de W_IO toutes les 30 ms
#define W_OUTMAX (256u * 1024u)       // plus grande réponse en mémoire (répertoire, BASIC en texte)
#define W_DSKMAX (1536u * 1024u)      // plus grande image .DSK lue

static bool     s_on = false;
static char     s_pin[8] = "";
static int      s_ls = -1, s_cs = -1;
static uint16_t s_port = WEB_PORT;
static uint8_t *s_hdr, *s_io;
static char    *s_out;                // réponse en mémoire (en-tête + corps)
static uint32_t s_out_cap = 0, s_olen = 0, s_opos = 0;
enum { C_IDLE = 0, C_HDR, C_SEND, C_FILE, C_RECV };
static uint8_t  s_st = C_IDLE;
static uint32_t s_hlen = 0, s_t0 = 0, s_last = 0;
static int      s_fd = -1;
static uint32_t s_rem = 0, s_iolen = 0, s_iopos = 0;
static char     s_tmp[300], s_dst[300];
static uint32_t s_bad_pin = 0, s_lock_until = 0;
static uint32_t st_req = 0, st_up = 0, st_down = 0, st_err = 0, st_upb = 0, st_downb = 0;
static bool     s_logged_listen = false;

// ---------------------------------------------------------------- réglages
#ifdef ESP_PLATFORM
static void W_Save(void)
{
  Preferences p;
  if (!p.begin("cc3w", false)) return;
  p.putUChar("on", s_on ? 1 : 0);
  p.putString("pin", s_pin);
  p.end();
}
static void W_NewPinRaw(void) { snprintf(s_pin, sizeof s_pin, "%06lu", (unsigned long)(esp_random() % 1000000u)); }
void Web_Init(void)
{
  Preferences p;
  if (p.begin("cc3w", true))
  {
    s_on = p.getUChar("on", 0) != 0;
    String v = p.getString("pin", "");
    strncpy(s_pin, v.c_str(), sizeof s_pin - 1);
    p.end();
  }
  if (strlen(s_pin) != 6) { W_NewPinRaw(); W_Save(); }
}
static void W_Ip(char *b, size_t n)
{
  char st[64]; Net_StatusLine(st, sizeof st);       // « CONNECTED 192.168.1.20 -58 dBm »
  const char *p = strchr(st, ' ');
  if (!p) { b[0] = 0; return; }
  p++;
  size_t k = 0; while (p[k] && p[k] != ' ' && k + 1 < n) { b[k] = p[k]; k++; }
  b[k] = 0;
}
#else
static void W_Save(void) {}
static void W_NewPinRaw(void) { snprintf(s_pin, sizeof s_pin, "%06u", (unsigned)(rand() % 1000000)); }
void Web_Init(void)
{
  s_on = getenv("WEBPORT") != nullptr;          // banc de l'émulateur : pas de serveur sans WEBPORT
  const char *p = getenv("WEBPIN"); snprintf(s_pin, sizeof s_pin, "%s", p ? p : "123456");
  const char *pt = getenv("WEBPORT"); if (pt) s_port = (uint16_t)atoi(pt);
}
#endif
void Web_SetEnabled(bool on) { s_on = on; W_Save(); }
bool Web_Enabled(void) { return s_on; }
const char *Web_Pin(void) { return s_pin; }
void Web_NewPin(void) { W_NewPinRaw(); s_bad_pin = 0; W_Save(); }

// ---------------------------------------------------------------- outils
static bool W_Alloc(void)
{
  if (!s_hdr) s_hdr = (uint8_t *)W_Mem(W_HDR + 1);
  if (!s_io)  s_io  = (uint8_t *)W_Mem(W_IO);
  if (!s_out) { s_out = (char *)W_Mem(16384); s_out_cap = s_out ? 16384u : 0u; }
  return s_hdr && s_io && s_out;
}
static bool W_Grow(uint32_t need)
{
  if (need <= s_out_cap) return true;
  if (need > W_OUTMAX) return false;
  uint32_t c = s_out_cap * 2; while (c < need) c *= 2; if (c > W_OUTMAX) c = W_OUTMAX;
  char *p = (char *)W_Realloc(s_out, c);
  if (!p) return false;
  s_out = p; s_out_cap = c; return true;
}
static bool s_ovf = false;
static void O_Mem(const void *d, uint32_t n)
{
  if (s_ovf) return;
  if (!W_Grow(s_olen + n + 1)) { s_ovf = true; return; }
  memcpy(s_out + s_olen, d, n); s_olen += n; s_out[s_olen] = 0;
}
static void O_Str(const char *s) { O_Mem(s, (uint32_t)strlen(s)); }
static void O_Fmt(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void O_Fmt(const char *fmt, ...)
{
  char b[400]; va_list a; va_start(a, fmt); int k = vsnprintf(b, sizeof b, fmt, a); va_end(a);
  if (k > 0) O_Mem(b, (uint32_t)(k < (int)sizeof b ? k : (int)sizeof b - 1));
}
static void O_Json(const char *s)                // chaîne JSON entre guillemets
{
  O_Str("\"");
  for (; *s; s++)
  {
    const unsigned char c = (unsigned char)*s;
    if (c == '"' || c == '\\') { char e[3] = { '\\', (char)c, 0 }; O_Str(e); }
    else if (c < 32) O_Fmt("\\u%04x", c);
    else if (c >= 128) O_Fmt("\\u%04x", c);     // noms de la SD : octets hors ASCII gardés tels quels
    else O_Mem(&c, 1);
  }
  O_Str("\"");
}
static void W_CloseFd(void) { if (s_fd >= 0) { Sd_Lock(); close(s_fd); Sd_Unlock(); s_fd = -1; } }
static void W_CloseClient(void)
{
  W_CloseFd();
  if (s_st == C_RECV && s_tmp[0]) { Sd_Lock(); char f[340]; snprintf(f, sizeof f, "%s%s", W_ROOT, s_tmp); unlink(f); Sd_Unlock(); }
  s_tmp[0] = 0;
  if (s_cs >= 0) { close(s_cs); s_cs = -1; }
  s_st = C_IDLE;
}
static void W_Full(char *out, size_t n, const char *p) { snprintf(out, n, "%s%s", W_ROOT, p); }

static int W_HexVal(char c) { return c >= '0' && c <= '9' ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10 : (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1; }
static void W_UrlDecode(char *s)
{
  char *o = s;
  for (; *s; s++)
  {
    if (*s == '+') *o++ = ' ';
    else if (*s == '%' && W_HexVal(s[1]) >= 0 && W_HexVal(s[2]) >= 0) { *o++ = (char)(W_HexVal(s[1]) * 16 + W_HexVal(s[2])); s += 2; }
    else *o++ = *s;
  }
  *o = 0;
}
// valeur d'un paramètre de la requête (décodée) ; faux s'il manque
static bool W_Param(const char *q, const char *name, char *out, size_t n)
{
  const size_t ln = strlen(name);
  while (q && *q)
  {
    if (!strncmp(q, name, ln) && q[ln] == '=')
    {
      q += ln + 1;
      size_t k = 0; while (q[k] && q[k] != '&' && k + 1 < n) { out[k] = q[k]; k++; }
      out[k] = 0; W_UrlDecode(out); return true;
    }
    q = strchr(q, '&'); if (q) q++;
  }
  return false;
}
// chemin sûr : commence par « / », sans « .. », sans « \ », sans double « / » final
static bool W_SafePath(char *p)
{
  if (p[0] != '/') return false;
  if (strstr(p, "/..") || strchr(p, '\\')) return false;
  size_t n = strlen(p);
  while (n > 1 && p[n - 1] == '/') p[--n] = 0;
  return n < 250;
}
static bool W_IEq(const char *a, const char *b) { while (*a && *b) { if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) return false; a++; b++; } return *a == *b; }
static bool W_IPrefix(const char *s, const char *pre)   // s commence par pre (dossier)
{
  size_t n = strlen(pre);
  for (size_t i = 0; i < n; i++) if (toupper((unsigned char)s[i]) != toupper((unsigned char)pre[i])) return false;
  return s[n] == 0 || s[n] == '/';
}
// Chemin protégé contre l'écriture : disque monté, dossier qui en contient un, /ESP-CONFIG.
static const char *W_Locked(const char *p)
{
  if (W_IPrefix(p, "/ESP-CONFIG")) return "settings folder (read only)";
  const char *m[10]; const int n = W_Mounted(m, 10);
  for (int i = 0; i < n; i++)
  {
    if (W_IEq(p, m[i])) return "disk is mounted (eject it first)";
    if (W_IPrefix(m[i], p) && strlen(p) > 1) return "folder holds a mounted disk";
  }
  return nullptr;
}

// ---------------------------------------------------------------- réponses
static void W_Head(int code, const char *type, long len, const char *extra = nullptr)
{
  const char *txt = code == 200 ? "OK" : code == 400 ? "Bad Request" : code == 403 ? "Forbidden" : code == 404 ? "Not Found"
                  : code == 409 ? "Conflict" : code == 413 ? "Too Large" : code == 423 ? "Locked" : code == 429 ? "Too Many" : code == 503 ? "Unavailable" : "Error";
  s_olen = 0; s_opos = 0; s_ovf = false;
  O_Fmt("HTTP/1.1 %d %s\r\nContent-Type: %s\r\nConnection: close\r\nCache-Control: no-store\r\n", code, txt, type);
  if (len >= 0) O_Fmt("Content-Length: %ld\r\n", len);
  if (extra) O_Str(extra);
  O_Str("\r\n");
}
static void W_SendBody(int code, const char *type, const char *body, uint32_t n)   // réponse complète en mémoire
{
  W_Head(code, type, (long)n);
  O_Mem(body, n);
  s_st = C_SEND;
}
static void W_Json(int code, const char *json) { W_SendBody(code, "application/json", json, (uint32_t)strlen(json)); }
static void W_Err(int code, const char *msg)
{
  char b[200]; snprintf(b, sizeof b, "{\"ok\":false,\"err\":\"%s\"}", msg);
  st_err++;
  W_Json(code, b);
}
// Corps déjà écrit dans s_out après un en-tête provisoire : on refait l'en-tête avec la longueur.
static uint32_t s_body0 = 0;
static void W_BeginBody(void) { s_olen = 0; s_opos = 0; s_ovf = false; s_body0 = 0; }
static void W_FinishBody(int code, const char *type, const char *extra = nullptr)
{
  if (s_ovf) { W_Err(413, "answer too large"); return; }
  const uint32_t blen = s_olen;
  char head[400];
  const char *txt = code == 200 ? "OK" : "Error";
  int hl = snprintf(head, sizeof head, "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nConnection: close\r\nCache-Control: no-store\r\nContent-Length: %lu\r\n%s\r\n",
                    code, txt, type, (unsigned long)blen, extra ? extra : "");
  if (!W_Grow(blen + hl + 1)) { W_Err(413, "answer too large"); return; }
  memmove(s_out + hl, s_out, blen);
  memcpy(s_out, head, hl);
  s_olen = blen + hl; s_opos = 0;
  s_st = C_SEND;
}

// ---------------------------------------------------------------- la page
extern const char k_web_page[];                       // plus bas
static void W_Page(void)
{
  const uint32_t n = (uint32_t)strlen(k_web_page);
  W_Head(200, "text/html; charset=utf-8", (long)n);
  O_Mem(k_web_page, n);
  s_st = C_SEND;
}

// ---------------------------------------------------------------- API
static void A_Info(void)
{
  char ram[160]; W_Ram(ram, sizeof ram);
  char ip[24]; W_Ip(ip, sizeof ip);
  W_BeginBody();
  O_Fmt("{\"ok\":true,\"ver\":\"%s\",\"ip\":\"%s\",%s,\"mounted\":[", g_fw_version_short, ip, ram);
  const char *m[10]; const int n = W_Mounted(m, 10);
  for (int i = 0; i < n; i++) { if (i) O_Str(","); O_Json(m[i]); }
  O_Str("]}");
  W_FinishBody(200, "application/json");
}

static void A_List(const char *p)
{
  char full[300]; W_Full(full, sizeof full, p);
  Sd_Lock();
  DIR *d = opendir(full);
  Sd_Unlock();
  if (!d) { W_Err(404, "folder not found"); return; }
  W_BeginBody();
  O_Str("{\"ok\":true,\"p\":"); O_Json(p); O_Fmt(",\"ro\":%s,\"e\":[", W_IPrefix(p, "/ESP-CONFIG") ? "true" : "false");   // dossier où l'on ne peut rien écrire
  int n = 0;
  for (;;)
  {
    Sd_Lock();
    struct dirent *e = readdir(d);
    char name[260] = ""; bool dir = false; long size = 0; bool ok = e != nullptr;
    if (ok)
    {
      snprintf(name, sizeof name, "%s", e->d_name);
      char f[600]; snprintf(f, sizeof f, "%s/%s", full, name);
      struct stat st;
      if (stat(f, &st) == 0) { dir = S_ISDIR(st.st_mode); size = dir ? 0 : (long)st.st_size; }
      else dir = e->d_type == DT_DIR;
    }
    Sd_Unlock();
    if (!ok) break;
    if (!strcmp(name, ".") || !strcmp(name, "..")) continue;
    if (n >= 2000) break;
    char child[600]; snprintf(child, sizeof child, "%s/%s", strcmp(p, "/") ? p : "", name);
    if (n++) O_Str(",");
    O_Str("["); O_Json(name); O_Fmt(",%d,%ld,%d]", dir ? 1 : 0, size, W_Locked(child) ? (W_IPrefix(child, "/ESP-CONFIG") ? 2 : 1) : 0);
    if ((n & 15) == 0) { /* le verrou a été rendu entre deux entrées */ }
  }
  Sd_Lock(); closedir(d); Sd_Unlock();
  O_Str("]}");
  W_FinishBody(200, "application/json");
}

static void A_Get(const char *p)
{
  char full[300]; W_Full(full, sizeof full, p);
  struct stat st;
  Sd_Lock();
  const bool ok = stat(full, &st) == 0 && !S_ISDIR(st.st_mode);
  s_fd = ok ? open(full, O_RDONLY) : -1;
  Sd_Unlock();
  if (s_fd < 0) { W_Err(404, "file not found"); return; }
  const char *base = strrchr(p, '/'); base = base ? base + 1 : p;
  char extra[320]; snprintf(extra, sizeof extra, "Content-Disposition: attachment; filename=\"%s\"\r\n", base);
  W_Head(200, "application/octet-stream", (long)st.st_size, extra);
  s_rem = (uint32_t)st.st_size; s_iolen = s_iopos = 0;
  s_st = C_FILE; st_down++;
}

static bool W_MkDirs(const char *p)          // crée les dossiers du chemin p (sans le dernier élément)
{
  char t[300]; snprintf(t, sizeof t, "%s", p);
  for (char *s = t + 1; *s; s++)
  {
    if (*s != '/') continue;
    *s = 0;
    char f[340]; W_Full(f, sizeof f, t);
    struct stat st;
    if (stat(f, &st) != 0 && mkdir(f, 0777) != 0) { *s = '/'; return false; }
    *s = '/';
  }
  return true;
}

static void A_Put(const char *p, long clen, bool overwrite, uint32_t body_have)
{
  const char *why = W_Locked(p);
  if (why) { W_Err(423, why); return; }
  if (clen < 0) { W_Err(400, "Content-Length needed"); return; }
  char full[300]; W_Full(full, sizeof full, p);
  struct stat st;
  Sd_Lock();
  const bool exists = stat(full, &st) == 0;
  bool dirs = W_MkDirs(p);
  Sd_Unlock();
  if (exists && S_ISDIR(st.st_mode)) { W_Err(409, "a folder has that name"); return; }
  if (exists && !overwrite) { W_Err(409, "exists"); return; }
  if (!dirs) { W_Err(500, "cannot create folder"); return; }
  snprintf(s_dst, sizeof s_dst, "%s", p);
  snprintf(s_tmp, sizeof s_tmp, "%s.UP~", p);
  char ft[340]; W_Full(ft, sizeof ft, s_tmp);
  Sd_Lock();
  s_fd = open(ft, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  Sd_Unlock();
  if (s_fd < 0) { s_tmp[0] = 0; W_Err(500, "cannot write on the SD card"); return; }
  s_rem = (uint32_t)clen;
  // octets du corps déjà reçus avec l'en-tête : s_hdr[s_hlen - body_have .. s_hlen)
  s_iolen = 0;
  if (body_have)
  {
    const uint32_t k = body_have > s_rem ? s_rem : body_have;
    memcpy(s_io, s_hdr + s_hlen - body_have, k);
    s_iolen = k;
  }
  s_st = C_RECV; st_up++;
}

static void W_Done(bool ok, const char *msg)
{
  if (ok) { char b[120]; snprintf(b, sizeof b, "{\"ok\":true%s%s%s}", msg ? ",\"msg\":\"" : "", msg ? msg : "", msg ? "\"" : ""); W_Json(200, b); }
  else W_Err(500, msg ? msg : "failed");
}

static void A_Del(const char *p)
{
  const char *why = W_Locked(p); if (why) { W_Err(423, why); return; }
  if (!strcmp(p, "/")) { W_Err(400, "no"); return; }
  char full[300]; W_Full(full, sizeof full, p);
  struct stat st;
  Sd_Lock();
  bool ok = stat(full, &st) == 0;
  const bool dir = ok && S_ISDIR(st.st_mode);
  if (ok) ok = dir ? rmdir(full) == 0 : unlink(full) == 0;
  Sd_Unlock();
  char lg[340]; snprintf(lg, sizeof lg, "[WEB] efface %s : %s", p, ok ? "OK" : "echec"); Log_Line(lg);
  W_Done(ok, ok ? nullptr : (dir ? "folder not empty?" : "cannot delete"));
}
static void A_Mkdir(const char *p)
{
  if (W_IPrefix(p, "/ESP-CONFIG")) { W_Err(423, "settings folder (read only)"); return; }
  char full[300]; W_Full(full, sizeof full, p);
  Sd_Lock(); const bool ok = mkdir(full, 0777) == 0; Sd_Unlock();
  W_Done(ok, ok ? nullptr : "cannot create folder");
}
static void A_Ren(const char *p, const char *nn)
{
  const char *why = W_Locked(p); if (why) { W_Err(423, why); return; }
  if (!nn[0] || strchr(nn, '/') || strchr(nn, '\\') || !strcmp(nn, "..") || !strcmp(nn, ".")) { W_Err(400, "bad name"); return; }
  char np[300]; snprintf(np, sizeof np, "%s", p);
  char *sl = strrchr(np, '/'); if (!sl) { W_Err(400, "bad path"); return; }
  snprintf(sl + 1, sizeof np - (sl + 1 - np), "%s", nn);
  if (W_Locked(np)) { W_Err(423, "name is protected"); return; }
  char f1[340], f2[340]; W_Full(f1, sizeof f1, p); W_Full(f2, sizeof f2, np);
  struct stat st;
  Sd_Lock();
  const bool clash = stat(f2, &st) == 0 && !W_IEq(p, np);
  const bool ok = !clash && rename(f1, f2) == 0;
  Sd_Unlock();
  W_Done(ok, ok ? nullptr : (clash ? "name already used" : "cannot rename"));
}

// ---------------------------------------------------------------- disquettes RS-DOS
struct DskImg { uint8_t *d; uint32_t size, hdr; uint8_t tracks, sides; };
static bool Dsk_Load(const char *p, DskImg &g)
{
  char full[300]; W_Full(full, sizeof full, p);
  struct stat st;
  Sd_Lock();
  int fd = (stat(full, &st) == 0 && !S_ISDIR(st.st_mode) && st.st_size <= (off_t)W_DSKMAX && st.st_size >= 256 * 18 * 18) ? open(full, O_RDONLY) : -1;
  Sd_Unlock();
  if (fd < 0) return false;
  g.size = (uint32_t)st.st_size;
  g.d = (uint8_t *)W_Mem(g.size);
  if (!g.d) { Sd_Lock(); close(fd); Sd_Unlock(); return false; }
  uint32_t got = 0;
  while (got < g.size)                                // morceaux de 4 Ko : le CPU peut lire ses disquettes entre deux
  {
    Sd_Lock();
    const ssize_t r = read(fd, g.d + got, g.size - got < W_IO ? g.size - got : W_IO);
    Sd_Unlock();
    if (r <= 0) break;
    got += (uint32_t)r;
  }
  Sd_Lock(); close(fd); Sd_Unlock();
  if (got != g.size) { free(g.d); g.d = nullptr; return false; }
  g.hdr = g.size % 256u;                              // en-tête JVC
  g.sides = (g.hdr >= 2 && g.d[1] == 2) ? 2 : 1;
  g.tracks = (uint8_t)((g.size - g.hdr) / (18u * 256u * g.sides));
  return g.tracks >= 18;
}
static const uint8_t *Dsk_Sec(const DskImg &g, int track, int sector)   // face 0, secteur 1-18
{
  const uint32_t off = g.hdr + ((uint32_t)(track * g.sides) * 18u + (uint32_t)(sector - 1)) * 256u;
  return off + 256u <= g.size ? g.d + off : nullptr;
}
static int Dsk_Grans(const DskImg &g) { int n = (g.tracks - 1) * 2; return n > 68 + 10 ? 78 : n; }   // 35 pistes : 68
static const uint8_t *Dsk_Gran(const DskImg &g, int gr) { int t = gr / 2; if (t >= 17) t++; return Dsk_Sec(g, t, (gr & 1) * 9 + 1); }
// taille d'un fichier (chaîne de granules) ; -1 si la chaîne est abîmée
static long Dsk_FileSize(const DskImg &g, const uint8_t *fat, int first, uint16_t lastb)
{
  long size = 0; int gr = first, guard = 0;
  const int ng = Dsk_Grans(g);
  while (guard++ < 90)
  {
    if (gr < 0 || gr >= ng) return -1;
    const uint8_t v = fat[gr];
    if (v >= 0xC0)
    {
      int secs = v & 0x1F; if (secs < 1) secs = 1; if (secs > 9) secs = 9;
      size += (long)(secs - 1) * 256 + (lastb > 256 ? 256 : lastb);
      return size;
    }
    size += 9 * 256; gr = v;
  }
  return -1;
}
static bool Dsk_Extract(const DskImg &g, const uint8_t *fat, int first, uint16_t lastb, uint8_t **out, uint32_t *n)
{
  const long sz = Dsk_FileSize(g, fat, first, lastb);
  if (sz < 0) return false;
  uint8_t *b = (uint8_t *)W_Mem((size_t)sz + 1);
  if (!b) return false;
  uint32_t k = 0; int gr = first, guard = 0;
  while (guard++ < 90)
  {
    const uint8_t *s = Dsk_Gran(g, gr);
    const uint8_t v = fat[gr];
    const uint32_t take = v >= 0xC0 ? (uint32_t)(sz - (long)k) : 9u * 256u;
    if (!s || k + take > (uint32_t)sz) { free(b); return false; }
    memcpy(b + k, s, take); k += take;
    if (v >= 0xC0) break;
    gr = v;
  }
  *out = b; *n = k; return true;
}
// entrée de répertoire n (0-71) ; nullptr si fin / effacée gérées par l'appelant
static const uint8_t *Dsk_Dir(const DskImg &g, int i) { const uint8_t *s = Dsk_Sec(g, 17, 3 + i / 8); return s ? s + (i % 8) * 32 : nullptr; }
static void Dsk_Name(const uint8_t *e, char *nm, char *ext)
{
  int k = 0; for (int i = 0; i < 8; i++) nm[k++] = (char)(e[i] >= 32 && e[i] < 127 ? e[i] : '?');
  while (k && nm[k - 1] == ' ') k--;
  nm[k] = 0;
  k = 0; for (int i = 0; i < 3; i++) ext[k++] = (char)(e[8 + i] >= 32 && e[8 + i] < 127 ? e[8 + i] : '?');
  while (k && ext[k - 1] == ' ') k--;
  ext[k] = 0;
}
static void A_Dsk(const char *p)
{
  DskImg g = {};
  if (!Dsk_Load(p, g)) { W_Err(400, "not a disk image (or too big)"); return; }
  const uint8_t *fat = Dsk_Sec(g, 17, 2);
  W_BeginBody();
  int freeg = 0; const int ng = Dsk_Grans(g);
  for (int i = 0; i < ng; i++) if (fat && fat[i] == 0xFF) freeg++;
  O_Fmt("{\"ok\":true,\"tracks\":%u,\"sides\":%u,\"free\":%d,\"f\":[", g.tracks, g.sides, freeg * 2304);
  int n = 0;
  for (int i = 0; i < 72 && fat; i++)
  {
    const uint8_t *e = Dsk_Dir(g, i);
    if (!e || e[0] == 0xFF) break;                  // fin du répertoire
    if (e[0] == 0x00) continue;                     // effacé
    char nm[12], ext[6]; Dsk_Name(e, nm, ext);
    const uint16_t lastb = (uint16_t)(e[14] << 8 | e[15]);
    const long sz = Dsk_FileSize(g, fat, e[13], lastb);
    if (n++) O_Str(",");
    O_Str("["); O_Json(nm); O_Str(","); O_Json(ext); O_Fmt(",%u,%u,%ld]", e[11], e[12] ? 1u : 0u, sz);
  }
  O_Str("]}");
  free(g.d);
  W_FinishBody(200, "application/json");
}

// Détokenisation : Color BASIC, Extended, Disk BASIC et Super Extended (CoCo 3). Fonctions du Super
// Extended : premier numéro $29 (+$80 = $A9)
static const char *const k_tok[] = {
  "FOR","GO","REM","'","ELSE","IF","DATA","PRINT","ON","INPUT","END","NEXT","DIM","READ","RUN","RESTORE",
  "RETURN","STOP","POKE","CONT","LIST","CLEAR","NEW","CLOAD","CSAVE","OPEN","CLOSE","LLIST","SET","RESET","CLS","MOTOR",
  "SOUND","AUDIO","EXEC","SKIPF","TAB(","TO","SUB","THEN","NOT","STEP","OFF","+","-","*","/","^",
  "AND","OR",">","=","<","DEL","EDIT","TRON","TROFF","DEF","LET","LINE","PCLS","PSET","PRESET","SCREEN",
  "PCLEAR","COLOR","CIRCLE","PAINT","GET","PUT","DRAW","PCOPY","PMODE","PLAY","DLOAD","RENUM","FN","USING","DIR","DRIVE",
  "FIELD","FILES","KILL","LOAD","LSET","MERGE","RENAME","RSET","SAVE","WRITE","VERIFY","UNLOAD","DSKINI","BACKUP","COPY","DSKI$",
  "DSKO$","DOS","WIDTH","PALETTE","HSCREEN","LPOKE","HCLS","HCOLOR","HPAINT","HCIRCLE","HLINE","HGET","HPUT","HBUFF","HPRINT","ERR",
  "BRK","LOCATE","HSTAT","HSET","HRESET","HDRAW","CMP","RGB","ATTR" };
static const char *const k_fn[] = {
  "SGN","INT","ABS","USR","RND","SIN","PEEK","LEN","STR$","VAL","ASC","CHR$","EOF","JOYSTK","LEFT$","RIGHT$",
  "MID$","POINT","INKEY$","MEM","ATN","COS","TAN","EXP","FIX","LOG","POS","SQR","HEX$","VARPTR","INSTR","TIMER",
  "PPOINT","STRING$","CVN","FREE","LOC","LOF","MKN$","AS",nullptr,"LPEEK","BUTTON","HPOINT","ERNO","ERLIN" };
#define NTOK ((int)(sizeof k_tok / sizeof k_tok[0]))
#define NFN  ((int)(sizeof k_fn / sizeof k_fn[0]))
static bool Bas_Detok(const uint8_t *b, uint32_t n)   // écrit le texte dans s_out
{
  if (n < 3 || b[0] != 0xFF) return false;
  uint32_t i = 3;                                     // $FF, longueur sur 2 octets
  int lines = 0;
  while (i + 4 <= n)
  {
    const uint16_t next = (uint16_t)(b[i] << 8 | b[i + 1]);
    if (next == 0) break;
    const uint16_t ln = (uint16_t)(b[i + 2] << 8 | b[i + 3]);
    i += 4;
    O_Fmt("%u ", ln);
    bool quote = false, literal = false, data = false;
    while (i < n && b[i])
    {
      const uint8_t c = b[i++];
      if (literal || quote) { if (c == '"' && quote) quote = false; if (c >= 32 && c < 128) O_Mem(&c, 1); else O_Fmt("{%02X}", c); continue; }
      if (data) { if (c == ':') data = false; if (c == '"') quote = true; if (c >= 32 && c < 128) O_Mem(&c, 1); else O_Fmt("{%02X}", c); continue; }
      if (c == '"') { quote = true; O_Mem(&c, 1); continue; }
      if (c == ':' && i < n && (b[i] == 0x83 || b[i] == 0x84)) continue;   // « ' » et ELSE sont rangés ':' + jeton
      if (c == 0xFF && i < n)
      {
        const uint8_t f = b[i++];
        if (f >= 0x80 && f - 0x80 < NFN && k_fn[f - 0x80]) O_Str(k_fn[f - 0x80]); else O_Fmt("{FF%02X}", f);
        continue;
      }
      if (c >= 0x80)
      {
        if (c - 0x80 < NTOK) O_Str(k_tok[c - 0x80]); else O_Fmt("{%02X}", c);
        if (c == 0x82 || c == 0x83) literal = true;   // REM, '
        if (c == 0x86) data = true;                   // DATA
        continue;
      }
      O_Mem(&c, 1);
    }
    i++;                                              // 0 de fin de ligne
    O_Str("\r\n");
    if (++lines > 20000) break;
  }
  return lines > 0;
}
static void A_DskGet(const char *p, const char *fname, bool text)
{
  DskImg g = {};
  if (!Dsk_Load(p, g)) { W_Err(400, "not a disk image"); return; }
  const uint8_t *fat = Dsk_Sec(g, 17, 2);
  const uint8_t *found = nullptr;
  for (int i = 0; i < 72 && fat; i++)
  {
    const uint8_t *e = Dsk_Dir(g, i);
    if (!e || e[0] == 0xFF) break;
    if (e[0] == 0) continue;
    char nm[12], ext[6], full[20]; Dsk_Name(e, nm, ext);
    snprintf(full, sizeof full, "%s.%s", nm, ext);
    if (W_IEq(full, fname)) { found = e; break; }
  }
  uint8_t *data = nullptr; uint32_t n = 0;
  if (!found || !Dsk_Extract(g, fat, found[13], (uint16_t)(found[14] << 8 | found[15]), &data, &n))
  { free(g.d); W_Err(404, "file not found in the disk"); return; }
  const uint8_t ftype = found[11], fascii = found[12];   // (found pointe dans l'image : copié avant free)
  free(g.d);
  W_BeginBody();
  char extra[200];
  if (text && ftype == 2 && !fascii) { free(data); W_Err(400, "machine language: use Raw"); return; }
  if (text)
  {
    bool ok = true;
    if (ftype == 0 && fascii == 0) ok = Bas_Detok(data, n);          // BASIC rangé en jetons
    else for (uint32_t i = 0; i < n; i++)                                  // ASCII : CR -> CR LF
    {
      const uint8_t c = data[i];
      if (c == 13) O_Str("\r\n"); else if (c == 10) {} else if (c >= 32 && c < 128) O_Mem(&c, 1); else if (c == 9) O_Mem(&c, 1); else O_Fmt("{%02X}", c);
    }
    free(data);
    if (!ok) { W_Err(400, "not a BASIC program"); return; }
    snprintf(extra, sizeof extra, "Content-Disposition: inline; filename=\"%s.txt\"\r\n", fname);
    W_FinishBody(200, "text/plain; charset=us-ascii", extra);
    return;
  }
  O_Mem(data, n); free(data);
  snprintf(extra, sizeof extra, "Content-Disposition: attachment; filename=\"%s\"\r\n", fname);
  W_FinishBody(200, "application/octet-stream", extra);
}

// ---------------------------------------------------------------- requête
static bool W_PinOk(const char *hdrs, const char *q)
{
  char k[16] = "";
  const char *h = strstr(hdrs, "\nX-Pin:"); if (!h) h = strstr(hdrs, "\nx-pin:");
  if (h) { h += 7; while (*h == ' ') h++; size_t i = 0; while (h[i] && h[i] != '\r' && i < 15) { k[i] = h[i]; i++; } k[i] = 0; }
  else W_Param(q, "k", k, sizeof k);
  return s_pin[0] && strcmp(k, s_pin) == 0;
}
static long W_ContentLength(const char *hdrs)
{
  const char *h = hdrs;
  while ((h = strchr(h, '\n')) != nullptr)
  {
    h++;
    if (!strncasecmp(h, "Content-Length:", 15)) return atol(h + 15);
  }
  return -1;
}
static void W_Handle(uint32_t hend)                   // en-tête complet : s_hdr[0..hend) (avec la ligne vide)
{
  st_req++;
  char *h = (char *)s_hdr;
  const char saved = h[hend]; h[hend] = 0;
  char method[8] = "", target[400] = "";
  sscanf(h, "%7s %399s", method, target);
  const bool post = !strcmp(method, "POST") || !strcmp(method, "PUT");
  char *q = strchr(target, '?'); if (q) *q++ = 0; else q = (char *)"";
  const long clen = W_ContentLength(h);
  const uint32_t body_have = s_hlen - hend;
  h[hend] = saved;
  char hdrcopy[W_HDR + 1];                           // pile de loop : 32 Ko
  memcpy(hdrcopy, h, hend); hdrcopy[hend] = 0;

  if (!strcmp(target, "/") || !strcmp(target, "/index.html")) { W_Page(); return; }
  if (!strcmp(target, "/favicon.ico")) { W_SendBody(404, "text/plain", "", 0); return; }
  if (strncmp(target, "/api/", 5)) { W_Err(404, "not found"); return; }
  const char *api = target + 5;
  if ((int32_t)(W_Ms() - s_lock_until) < 0) { W_Err(429, "too many wrong PINs: wait 30 s"); return; }
  if (!W_PinOk(hdrcopy, q))
  {
    if (++s_bad_pin >= 5) { s_bad_pin = 0; s_lock_until = W_Ms() + 30000u; Log_Line("[WEB] 5 PIN refuses : pause de 30 s"); }
    W_Err(403, "PIN"); return;
  }
  s_bad_pin = 0;
  if (!W_SdOk()) { W_Err(503, "no SD card"); return; }
  if (!strcmp(api, "info")) { A_Info(); return; }
  if (!strcmp(api, "health"))                       // état matériel (health.cpp)
  {
    extern void Health_Json(char *b, size_t n);
    static char hb[3072];
    Health_Json(hb, sizeof hb);
    W_Json(200, hb); return;
  }
  char p[300] = "";
  if (!W_Param(q, "p", p, sizeof p) || !W_SafePath(p)) { W_Err(400, "bad path"); return; }
  if (!strcmp(api, "ls"))  { A_List(p); return; }
  if (!strcmp(api, "get")) { A_Get(p); return; }
  if (!strcmp(api, "dsk")) { A_Dsk(p); return; }
  if (!strcmp(api, "dskget")) { char f[20] = "", m[8] = ""; W_Param(q, "f", f, sizeof f); W_Param(q, "m", m, sizeof m); A_DskGet(p, f, !strcmp(m, "txt")); return; }
  if (!post) { W_Err(400, "use POST"); return; }
  if (!strcmp(api, "put")) { char ow[4] = ""; W_Param(q, "ow", ow, sizeof ow); A_Put(p, clen, ow[0] == '1', body_have); return; }
  if (!strcmp(api, "del")) { A_Del(p); return; }
  if (!strcmp(api, "mkdir")) { A_Mkdir(p); return; }
  if (!strcmp(api, "ren")) { char nn[120] = ""; W_Param(q, "n", nn, sizeof nn); A_Ren(p, nn); return; }
  W_Err(404, "unknown");
}

// ---------------------------------------------------------------- boucle
static void W_StopAll(void)
{
  W_CloseClient();
  if (s_ls >= 0) { close(s_ls); s_ls = -1; }
  s_logged_listen = false;
}
static bool W_Listen(void)
{
  if (s_ls >= 0) return true;
  const int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s < 0) return false;
  const int one = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  struct sockaddr_in a; memset(&a, 0, sizeof a);
  a.sin_family = AF_INET; a.sin_port = htons(s_port); a.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (struct sockaddr *)&a, sizeof a) < 0 || listen(s, 2) < 0) { close(s); return false; }
  fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
  s_ls = s;
  if (!s_logged_listen)
  {
    s_logged_listen = true;
    char ip[24]; W_Ip(ip, sizeof ip);
    char b[120]; snprintf(b, sizeof b, "[WEB] page web : http://%s/ (port %u)", ip, (unsigned)s_port); Log_Line(b);
  }
  return true;
}
// Envoie ce qui reste de s_out ; vrai quand tout est parti
static bool W_Flush(void)
{
  while (s_opos < s_olen)
  {
    const int k = send(s_cs, s_out + s_opos, s_olen - s_opos, MSG_DONTWAIT);
    if (k < 0) { if (errno == EAGAIN || errno == EWOULDBLOCK) return false; W_CloseClient(); return false; }
    s_opos += (uint32_t)k; s_last = W_Ms();
  }
  return true;
}

void Web_Poll(void)
{
  if (!s_on || !W_NetOk() || !W_SdOk()) { if (s_ls >= 0 || s_cs >= 0) W_StopAll(); return; }
  if (!W_Alloc()) return;
  if (!W_Listen()) return;
  const uint32_t now = W_Ms();
  if (s_cs < 0)
  {
    struct sockaddr_in ra; socklen_t rl = sizeof ra;
    const int c = accept(s_ls, (struct sockaddr *)&ra, &rl);
    if (c < 0) return;
    fcntl(c, F_SETFL, fcntl(c, F_GETFL, 0) | O_NONBLOCK);
    const int one = 1; setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    s_cs = c; s_st = C_HDR; s_hlen = 0; s_t0 = s_last = now; s_tmp[0] = 0;
  }
  if ((uint32_t)(now - s_last) > 15000u) { if (s_st == C_RECV) Log_Line("[WEB] envoi interrompu (15 s sans donnees) : fichier jete"); W_CloseClient(); return; }

  // pendant un jeu, un fichier passe par petits morceaux (4 Ko toutes les W_SLOW_MS, environ 130 Ko/s)
  // : loop partage le coeur 1 avec le 6809 (même priorité) et la SD est lente (~4 ms par accès)
  const bool slow = (s_st == C_FILE || s_st == C_RECV) && W_EmuRun();
  if (slow) { static uint32_t s_slow_t = 0; if ((uint32_t)(now - s_slow_t) < W_SLOW_MS) return; s_slow_t = now; }
  for (int step = 0; step < (slow ? 1 : 8) && s_cs >= 0; step++)    // au plus ~32 Ko par passage de loop
  {
    switch (s_st)
    {
      case C_HDR:
      {
        const int k = recv(s_cs, s_hdr + s_hlen, W_HDR - s_hlen, MSG_DONTWAIT);
        if (k == 0 || (k < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) { W_CloseClient(); return; }
        if (k < 0) return;
        s_hlen += (uint32_t)k; s_last = now; s_hdr[s_hlen] = 0;
        const char *e = strstr((const char *)s_hdr, "\r\n\r\n");
        if (!e) { if (s_hlen >= W_HDR) { W_Err(431, "header too large"); } continue; }
        W_Handle((uint32_t)(e + 4 - (const char *)s_hdr));
        break;
      }
      case C_SEND:
        if (W_Flush()) { W_CloseClient(); return; }
        return;
      case C_FILE:                                     // en-tête, puis le fichier par morceaux de 4 Ko
      {
        if (!W_Flush()) return;
        if (s_iopos >= s_iolen)
        {
          if (s_rem == 0) { W_CloseClient(); return; }
          Sd_Lock();
          const ssize_t r = read(s_fd, s_io, s_rem < W_IO ? s_rem : W_IO);
          Sd_Unlock();
          if (r <= 0) { W_CloseClient(); return; }
          s_iolen = (uint32_t)r; s_iopos = 0; s_rem -= (uint32_t)r;
        }
        const int k = send(s_cs, s_io + s_iopos, s_iolen - s_iopos, MSG_DONTWAIT);
        if (k < 0) { if (errno == EAGAIN || errno == EWOULDBLOCK) return; W_CloseClient(); return; }
        s_iopos += (uint32_t)k; st_downb += (uint32_t)k; s_last = now;
        break;
      }
      case C_RECV:                                     // corps de l'envoi -> fichier .UP~ puis renommé
      {
        if (s_iolen)
        {
          Sd_Lock();
          const ssize_t w = write(s_fd, s_io, s_iolen);
          Sd_Unlock();
          if (w != (ssize_t)s_iolen) { W_CloseFd(); char ft[340]; W_Full(ft, sizeof ft, s_tmp); Sd_Lock(); unlink(ft); Sd_Unlock(); s_tmp[0] = 0; W_Err(507, "SD card full or write error"); break; }
          s_rem -= s_iolen; st_upb += s_iolen; s_iolen = 0;
        }
        if (s_rem == 0)
        {
          W_CloseFd();
          char ft[340], fd[340]; W_Full(ft, sizeof ft, s_tmp); W_Full(fd, sizeof fd, s_dst);
          Sd_Lock(); unlink(fd); const bool ok = rename(ft, fd) == 0; Sd_Unlock();
          s_tmp[0] = 0;
          char lg[360]; snprintf(lg, sizeof lg, "[WEB] recu %s : %s", s_dst, ok ? "OK" : "echec du renommage"); Log_Line(lg);
          W_Done(ok, ok ? (W_IPrefix(s_dst, "/FLASHNOW") ? "Firmware will be flashed at the next start." : nullptr) : "rename failed");
          break;
        }
        const int k = recv(s_cs, s_io, s_rem < W_IO ? s_rem : W_IO, MSG_DONTWAIT);
        if (k == 0 || (k < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) { W_CloseClient(); return; }
        if (k < 0) return;
        s_iolen = (uint32_t)k; s_last = now;
        break;
      }
      default: W_CloseClient(); return;
    }
  }
}

void Web_StatusLine(char *buf, size_t n)
{
  if (!s_on) { snprintf(buf, n, "OFF"); return; }
  if (!W_NetOk()) { snprintf(buf, n, "needs Wi-Fi"); return; }
  if (!W_SdOk()) { snprintf(buf, n, "no SD card"); return; }
  char ip[24]; W_Ip(ip, sizeof ip);
  if (s_ls < 0) snprintf(buf, n, "starting...");
  else if (s_port == 80) snprintf(buf, n, "http://%s/", ip);
  else snprintf(buf, n, "http://%s:%u/", ip, (unsigned)s_port);
}
void Web_Report(void)
{
  char st[64]; Web_StatusLine(st, sizeof st);
  char b[240];
  snprintf(b, sizeof b, "[WEB] %s ; requetes %lu, envois %lu (%lu Ko), telechargements %lu (%lu Ko), erreurs %lu",
           st, (unsigned long)st_req, (unsigned long)st_up, (unsigned long)(st_upb / 1024u),
           (unsigned long)st_down, (unsigned long)(st_downb / 1024u), (unsigned long)st_err);
  Log_Line(b);
}

// ---------------------------------------------------------------- page (HTML + JS, en flash)
const char k_web_page[] = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CoCo3 SD card</title>
<style>
:root{--bg:#f4f5f7;--fg:#1d2330;--mut:#667;--card:#fff;--line:#dde1e7;--acc:#1a5fb4;--bad:#b42318;--ok:#137333}
@media (prefers-color-scheme:dark){:root{--bg:#14171c;--fg:#e6e9ef;--mut:#9aa3b2;--card:#1d2129;--line:#2d333d;--acc:#7cb3ff;--bad:#ff8a80;--ok:#7ddc95}}
*{box-sizing:border-box}body{margin:0;font:15px/1.4 system-ui,sans-serif;background:var(--bg);color:var(--fg)}
header{padding:12px 16px;border-bottom:1px solid var(--line);display:flex;gap:12px;align-items:baseline;flex-wrap:wrap}
h1{font-size:18px;margin:0}#info{color:var(--mut);font-size:13px}
main{padding:12px 16px;max-width:1000px;margin:auto}
#crumbs a,#crumbs span{margin-right:4px}a{color:var(--acc);text-decoration:none;cursor:pointer}
.bar{display:flex;gap:8px;flex-wrap:wrap;margin:10px 0}
button{font:inherit;padding:5px 10px;border:1px solid var(--line);background:var(--card);color:var(--fg);border-radius:6px;cursor:pointer}
button:hover{border-color:var(--acc)}
table{width:100%;border-collapse:collapse;background:var(--card);border:1px solid var(--line);border-radius:8px;overflow:hidden}
td,th{padding:6px 10px;border-bottom:1px solid var(--line);text-align:left;font-size:14px}
th{color:var(--mut);font-weight:600}td.n{text-align:right;font-variant-numeric:tabular-nums;white-space:nowrap}
td.act{white-space:nowrap;text-align:right}td.act a{margin-left:10px}
.lock{color:var(--mut);font-size:12px;margin-left:6px}
#drop{border:2px dashed var(--line);border-radius:8px;padding:14px;text-align:center;color:var(--mut);margin:10px 0}
#drop.on{border-color:var(--acc);color:var(--acc)}
#msg{min-height:20px;margin:6px 0}.bad{color:var(--bad)}.ok{color:var(--ok)}
#dsk{display:none;margin-top:14px}#dsk h2{font-size:16px;margin:4px 0}
pre{background:var(--card);border:1px solid var(--line);padding:10px;border-radius:8px;overflow:auto;max-height:60vh;font:13px/1.35 ui-monospace,monospace}
#pinbox{display:none;max-width:320px;margin:60px auto;text-align:center}#pinbox input{font:20px monospace;width:9em;text-align:center;padding:6px}
</style></head><body>
<header><h1>ESP32-CoCo3 &mdash; SD card</h1><span id="info"></span></header>
<div id="pinbox"><p>PIN shown in F12 &gt; NETWORK &gt; WEB PAGE</p><input id="pin" maxlength="6" inputmode="numeric" autofocus><p><button onclick="setPin()">Open</button></p><p id="pinmsg" class="bad"></p></div>
<main id="app" style="display:none">
<div id="crumbs"></div>
<div class="bar"><button onclick="up()">Up</button><button onclick="mk()">New folder</button>
<button onclick="document.getElementById('f').click()">Send files...</button><input type="file" id="f" multiple hidden onchange="send(this.files)">
<button onclick="ls(cur)">Refresh</button><button onclick="hl()">ESP32 health</button></div>
<div id="hlp" style="display:none;margin:8px 0"><table><tbody id="hlt"></tbody></table></div>
<div id="drop">Drop files here to send them to this folder</div>
<div id="msg"></div>
<table><thead><tr><th>Name</th><th class="n">Size</th><th></th></tr></thead><tbody id="list"></tbody></table>
<div id="dsk"><h2 id="dskt"></h2><table><thead><tr><th>File</th><th>Type</th><th class="n">Size</th><th></th></tr></thead><tbody id="dskl"></tbody></table><pre id="view" style="display:none"></pre></div>
</main>
<script>
let pin=localStorage.getItem('cc3pin')||'',cur=localStorage.getItem('cc3dir')||'/';
const $=id=>document.getElementById(id);
function msg(t,c){$('msg').textContent=t;$('msg').className=c||''}
function esc(s){return s.replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]))}
function sz(n){return n<1024?n+' B':n<1048576?(n/1024).toFixed(1)+' KB':(n/1048576).toFixed(1)+' MB'}
function join(d,n){return (d=='/'?'':d)+'/'+n}
async function api(p,opt){opt=opt||{};opt.headers=Object.assign({'X-Pin':pin},opt.headers||{});
 const r=await fetch('/api/'+p,opt);if(r.status==403){showPin('Wrong PIN');throw 'pin'}
 const j=await r.json().catch(()=>({ok:false,err:'HTTP '+r.status}));if(!j.ok)throw j.err||'error';return j}
function showPin(t){$('app').style.display='none';$('pinbox').style.display='block';$('pinmsg').textContent=t||''}
function setPin(){pin=$('pin').value.trim();localStorage.setItem('cc3pin',pin);start()}
$('pin').addEventListener('keydown',e=>{if(e.key=='Enter')setPin()});
async function start(){try{const i=await api('info');$('pinbox').style.display='none';$('app').style.display='block';
 $('info').textContent='v'+i.ver+' · '+i.ip+' · free RAM: internal '+(i.int>>10)+' KB, DMA '+(i.dma>>10)+' KB, PSRAM '+(i.psram>>10)+' KB';
 ls(cur)}catch(e){if(e!='pin')showPin(String(e))}}
async function ls(d,keep){try{const j=await api('ls?p='+encodeURIComponent(d));cur=j.p;localStorage.setItem('cc3dir',cur);
 const parts=cur.split('/').filter(x=>x);let h='<a onclick="ls(\'/\')">SD</a>',acc='';
 for(const p of parts){acc+='/'+p;h+=' / <a data-p="'+esc(acc)+'" onclick="ls(this.dataset.p)">'+esc(p)+'</a>'}$('crumbs').innerHTML=h;
 j.e.sort((a,b)=>b[1]-a[1]||a[0].localeCompare(b[0],undefined,{sensitivity:'base'}));
 let t='';for(const [n,dir,s,lk] of j.e){const p=esc(join(cur,n));
  const dsk=/\.dsk$/i.test(n);
  t+='<tr><td>'+(dir?'<a data-p="'+p+'" onclick="ls(this.dataset.p)">📁 '+esc(n)+'</a>':esc(n))+(lk?'<span class="lock">🔒 '+(lk==2?'read only':'in use')+'</span>':'')+'</td><td class="n">'+(dir?'':sz(s))+'</td><td class="act">'
  +(dsk?'<a data-p="'+p+'" onclick="dsk(this.dataset.p)">Contents</a>':'')
  +(dir?'':'<a href="/api/get?k='+pin+'&p='+encodeURIComponent(join(cur,n))+'">Download</a>')
  +(lk||j.ro?'':'<a data-p="'+p+'" onclick="ren(this.dataset.p)">Rename</a><a data-p="'+p+'" onclick="del(this.dataset.p)">Delete</a>')+'</td></tr>'}
 $('list').innerHTML=t||'<tr><td colspan="3" style="color:var(--mut)">(empty folder)</td></tr>';$('dsk').style.display='none';if(!keep)msg('')}catch(e){if(e!='pin')msg(String(e),'bad')}}
function up(){if(cur!='/')ls(cur.substring(0,cur.lastIndexOf('/'))||'/')}
async function mk(){const n=prompt('New folder name:');if(!n)return;try{await api('mkdir?p='+encodeURIComponent(join(cur,n)),{method:'POST'});ls(cur)}catch(e){msg(String(e),'bad')}}
async function del(p){if(!confirm('Delete '+p+' ?'))return;try{await api('del?p='+encodeURIComponent(p),{method:'POST'});ls(cur)}catch(e){msg(String(e),'bad')}}
async function ren(p){const o=p.substring(p.lastIndexOf('/')+1);const n=prompt('New name:',o);if(!n||n==o)return;
 try{await api('ren?p='+encodeURIComponent(p)+'&n='+encodeURIComponent(n),{method:'POST'});ls(cur)}catch(e){msg(String(e),'bad')}}
function put(path,file,ow){return new Promise((ok,ko)=>{const x=new XMLHttpRequest();
 x.open('POST','/api/put?p='+encodeURIComponent(path)+(ow?'&ow=1':''));x.setRequestHeader('X-Pin',pin);
 x.upload.onprogress=e=>{if(e.lengthComputable)msg('Sending '+file.name+' : '+Math.round(e.loaded*100/e.total)+' %')};
 x.onload=()=>{let j={};try{j=JSON.parse(x.responseText)}catch(e){}if(j.ok)ok(j);else ko(j.err||('HTTP '+x.status))};
 x.onerror=()=>ko('network error');x.send(file)})}
async function send(files){for(const f of files){const p=join(cur,f.name);
 try{let j;try{j=await put(p,f,false)}catch(e){if(e=='exists'&&confirm(f.name+' exists. Replace it?'))j=await put(p,f,true);else throw e}
  msg(f.name+' sent.'+(j.msg?' '+j.msg:''),'ok')}catch(e){msg(f.name+' : '+e,'bad');break}}
 $('f').value='';ls(cur,true)}
const dr=$('drop');['dragenter','dragover'].forEach(e=>dr.addEventListener(e,ev=>{ev.preventDefault();dr.classList.add('on')}));
['dragleave','drop'].forEach(e=>dr.addEventListener(e,ev=>{ev.preventDefault();dr.classList.remove('on')}));
dr.addEventListener('drop',ev=>send(ev.dataTransfer.files));
const TY=['BASIC','DATA','ML','TEXT'];
async function dsk(p){try{const j=await api('dsk?p='+encodeURIComponent(p));
 $('dskt').textContent=p+' — '+j.tracks+' tracks, '+sz(j.free)+' free';let t='';
 for(const [n,e,ty,asc,s] of j.f){const fn=n+'.'+e,q='p='+encodeURIComponent(p)+'&f='+encodeURIComponent(fn)+'&k='+pin;
  const txt=(ty==0||asc||ty==3);
  t+='<tr><td>'+esc(fn)+'</td><td>'+(TY[ty]||ty)+(asc?' (ASCII)':'')+'</td><td class="n">'+(s<0?'?':sz(s))+'</td><td class="act">'
  +(txt?'<a data-q="'+esc(q)+'" onclick="view(this.dataset.q)">View</a><a href="/api/dskget?m=txt&'+esc(q)+'">Text</a>':'')
  +'<a href="/api/dskget?'+esc(q)+'">Raw</a></td></tr>'}
 $('dskl').innerHTML=t||'<tr><td colspan="4" style="color:var(--mut)">(no files)</td></tr>';$('view').style.display='none';$('dsk').style.display='block';
 $('dsk').scrollIntoView({behavior:'smooth'})}catch(e){msg(String(e),'bad')}}
async function view(q){const r=await fetch('/api/dskget?m=txt&'+q);const t=await r.text();$('view').textContent=t;$('view').style.display='block'}
let hlTimer=null;const HC=['var(--ok)','#b8860b','var(--bad)','var(--fg)'];
async function hlLoad(){try{const j=await api('health');$('hlt').innerHTML=j.rows.map(r=>'<tr><td style="white-space:nowrap">'+esc(r[0])+'</td><td style="font-family:ui-monospace,monospace;color:'+HC[r[2]]+'">'+esc(r[1])+'</td></tr>').join('')}catch(e){}}
function hl(){const p=$('hlp');if(p.style.display=='none'){p.style.display='block';hlLoad();hlTimer=setInterval(hlLoad,5000)}else{p.style.display='none';clearInterval(hlTimer)}}
if(pin)start();else showPin('');
</script></body></html>
)HTML";
