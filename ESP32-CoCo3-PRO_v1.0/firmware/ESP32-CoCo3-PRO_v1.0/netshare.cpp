// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Dossier partagé d'un PC par le Wi-Fi (voir netshare.h).

#include "netshare.h"
#include "cc3net.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

void Log_Line(const char *s);   // main.cpp

// Accès aux lecteurs (main.cpp) pour le renvoi des disquettes modifiées
bool Disk_NetDirty(uint8_t drv, const uint8_t **data, uint32_t *size, const char **path, uint32_t *stamp, uint32_t *last_ms);
void Disk_NetSaved(uint8_t drv, uint32_t stamp, bool ok);
bool Disk_AdoptNet(uint8_t drv, const char *path, uint8_t *img, uint32_t size);

// montages différés (démarrage, Save State)
static char     s_pend[4][192];
static uint8_t  s_pend_tries[4];
void NetShare_QueueMount(uint8_t drv, const char *path)
{
  if (drv > 3 || !path) return;
  snprintf(s_pend[drv], sizeof s_pend[drv], "%s", path);
  s_pend_tries[drv] = 0;
}

struct NsEntry { uint32_t size; uint8_t dir; char name[103]; };
static NsEntry *s_ent = nullptr;
static int      s_count = 0;
static char     s_host[48] = "";          // « ip:port »
static char     s_name[28] = "";
static char     s_pin[8] = "";
static bool     s_loaded = false;
static uint32_t s_saves = 0, s_save_fail = 0, s_gets = 0;

const char *NetShare_ErrorText(int code)
{
  switch (code)
  {
    case NS_NOWIFI:   return "Wi-Fi is not connected (menu N).";
    case NS_NOSERVER: return "No PC share found: run coco_share.py on the PC.";
    case NS_PIN:      return "PIN refused: type the PIN shown by coco_share.py.";
    case NS_NOMEM:    return "Not enough memory for this disk.";
    case NS_NOTFOUND: return "File or folder not found on the PC.";
    case NS_READONLY: return "The PC share refused (read-only or not local).";
    case NS_OK:       return "OK";
    default:          return "Transfer error with the PC share.";
  }
}

const char *NetShare_ServerName(void) { return s_name; }
const char *NetShare_Host(void)       { return s_host; }
int  NetShare_Count(void)             { return s_count; }

bool NetShare_Entry(int i, bool *dir, uint32_t *size, const char **name)
{
  if (!s_ent || i < 0 || i >= s_count) return false;
  if (dir) *dir = s_ent[i].dir != 0;
  if (size) *size = s_ent[i].size;
  if (name) *name = s_ent[i].name;
  return true;
}

#ifdef ESP_PLATFORM
#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include "esp_heap_caps.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#define NS_PORT 6809

static void Ns_Log(const char *fmt, ...)
{
  char b[200]; va_list a; va_start(a, fmt); vsnprintf(b, sizeof b, fmt, a); va_end(a); Log_Line(b);
}

static void Ns_Load(void)
{
  if (s_loaded) return;
  s_loaded = true;
  Preferences p;
  if (p.begin("cc3n", true))
  {
    String a = p.getString("shpin", ""), b = p.getString("shhost", "");
    strncpy(s_pin, a.c_str(), sizeof s_pin - 1);
    strncpy(s_host, b.c_str(), sizeof s_host - 1);
    p.end();
  }
}

static void Ns_Save(void)
{
  Preferences p;
  if (!p.begin("cc3n", false)) return;
  p.putString("shpin", s_pin);
  p.putString("shhost", s_host);
  p.end();
}

bool NetShare_HasPin(void) { Ns_Load(); return s_pin[0] != 0; }

void NetShare_SetPin(const char *pin)
{
  Ns_Load();
  strncpy(s_pin, pin ? pin : "", sizeof s_pin - 1); s_pin[sizeof s_pin - 1] = 0;
  Ns_Save();
}

// « /A B/x.dsk » -> « /A%20B/x.dsk »
static void Ns_Encode(const char *in, char *out, size_t n)
{
  static const char *hx = "0123456789ABCDEF";
  size_t k = 0;
  for (; *in && k + 4 < n; in++)
  {
    const unsigned char c = (unsigned char)*in;
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '/' || c == '-' || c == '_' || c == '.' || c == '~')
      out[k++] = (char)c;
    else { out[k++] = '%'; out[k++] = hx[c >> 4]; out[k++] = hx[c & 15]; }
  }
  out[k] = 0;
}

static int Ns_Open(void)
{
  char host[48]; strncpy(host, s_host, sizeof host - 1); host[sizeof host - 1] = 0;
  uint16_t port = NS_PORT;
  char *c = strrchr(host, ':');
  if (c) { *c = 0; port = (uint16_t)atoi(c + 1); }
  struct sockaddr_in sa; memset(&sa, 0, sizeof sa);
  sa.sin_family = AF_INET; sa.sin_port = htons(port);
  if (inet_pton(AF_INET, host, &sa.sin_addr) != 1) return -1;
  const int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s < 0) return -1;
  fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
  int r = connect(s, (struct sockaddr *)&sa, sizeof sa);
  if (r < 0 && errno != EINPROGRESS) { close(s); return -1; }
  fd_set wf; FD_ZERO(&wf); FD_SET(s, &wf);
  struct timeval tv = { 2, 0 };
  r = select(s + 1, nullptr, &wf, nullptr, &tv);
  int err = 0; socklen_t el = sizeof err;
  if (r <= 0 || getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &el) < 0 || err) { close(s); return -1; }
  fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) & ~O_NONBLOCK);
  struct timeval to = { 6, 0 };
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof to);
  setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &to, sizeof to);
  const int one = 1;
  setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
  return s;
}

static bool Ns_SendAll(int s, const void *p, size_t n)
{
  const uint8_t *b = (const uint8_t *)p;
  while (n)
  {
    const int k = send(s, b, n > 4096 ? 4096 : n, 0);
    if (k <= 0) return false;
    b += k; n -= (size_t)k;
  }
  return true;
}

// Requête HTTP/1.0. Corps reçu dans « out » (cap octets) ou, si alloc, dans un tampon PSRAM de la
// taille annoncée (*alloc, à libérer par l'appelant).
static int Ns_Http(const char *method, const char *api, const char *path, const char *extra,
                   const uint8_t *body, uint32_t blen,
                   uint8_t *out, uint32_t cap, uint32_t *outlen, uint8_t **alloc)
{
  if (outlen) *outlen = 0;
  if (alloc) *alloc = nullptr;
  if (!Net_Connected()) return NS_NOWIFI;
  if (!s_host[0]) return NS_NOSERVER;
  const int s = Ns_Open();
  if (s < 0) return NS_NOSERVER;
  char enc[300]; Ns_Encode(path ? path : "/", enc, sizeof enc);
  char req[512];
  int n = snprintf(req, sizeof req, "%s /api/%s?p=%s%s HTTP/1.0\r\nHost: %s\r\nX-CoCo-Pin: %s\r\n", method, api, enc, extra ? extra : "", s_host, s_pin);
  if (body) n += snprintf(req + n, sizeof req - n, "Content-Length: %lu\r\nContent-Type: application/octet-stream\r\n", (unsigned long)blen);
  n += snprintf(req + n, sizeof req - n, "\r\n");
  if (!Ns_SendAll(s, req, (size_t)n) || (body && !Ns_SendAll(s, body, blen))) { close(s); return NS_ERR; }
  // En-têtes
  char hdr[1024]; size_t h = 0; int status = 0; long clen = -1;
  bool done = false;
  while (h < sizeof hdr - 1)
  {
    const int k = recv(s, hdr + h, 1, 0);     // octet par octet jusqu'à la fin des en-têtes (petit)
    if (k <= 0) break;
    h++;
    if (h >= 4 && memcmp(hdr + h - 4, "\r\n\r\n", 4) == 0) { done = true; break; }
  }
  hdr[h] = 0;
  if (!done) { close(s); return NS_ERR; }
  sscanf(hdr, "HTTP/%*s %d", &status);
  const char *cl = strcasestr(hdr, "\r\nContent-Length:");
  if (cl) clen = atol(cl + 17);
  if (status == 401) { close(s); return NS_PIN; }
  if (status == 404) { close(s); return NS_NOTFOUND; }
  if (status == 403) { close(s); return NS_READONLY; }
  if (status != 200) { close(s); return NS_ERR; }
  uint8_t *dst = out; uint32_t room = cap;
  if (alloc)
  {
    if (clen < 0 || clen > 4L * 1024 * 1024) { close(s); return NS_ERR; }
    dst = (uint8_t *)heap_caps_malloc((size_t)clen + 1, MALLOC_CAP_SPIRAM);   // + 1 : fin de texte possible
    if (!dst) { close(s); return NS_NOMEM; }
    room = (uint32_t)clen;
  }
  uint32_t got = 0;
  while (got < room && (clen < 0 || got < (uint32_t)clen))
  {
    const uint32_t want = room - got;
    const int k = recv(s, dst + got, want > 8192 ? 8192 : want, 0);
    if (k <= 0) break;
    got += (uint32_t)k;
  }
  close(s);
  if (clen >= 0 && got < (uint32_t)clen && got < room)
  {
    if (alloc) free(dst);
    return NS_ERR;
  }
  if (outlen) *outlen = got;
  if (alloc) *alloc = dst;
  return NS_OK;
}

// Diffusion UDP « COCO3SHARE? » -> « COCO3SHARE <port> <nom> » du serveur
static bool Ns_Discover(void)
{
  const int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) return false;
  const int one = 1;
  setsockopt(s, SOL_SOCKET, SO_BROADCAST, &one, sizeof one);
  struct timeval to = { 0, 400000 };
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof to);
  struct sockaddr_in b; memset(&b, 0, sizeof b);
  b.sin_family = AF_INET; b.sin_port = htons(NS_PORT);
  const IPAddress sub = WiFi.broadcastIP();
  const uint32_t targets[2] = { 0xFFFFFFFFu, (uint32_t)sub };
  bool found = false;
  for (int tries = 0; tries < 3 && !found; tries++)
  {
    for (int t = 0; t < 2; t++)
    {
      b.sin_addr.s_addr = (t == 0) ? htonl(targets[0]) : targets[1];
      sendto(s, "COCO3SHARE?", 11, 0, (struct sockaddr *)&b, sizeof b);
    }
    char r[64]; struct sockaddr_in from; socklen_t fl = sizeof from;
    const int k = recvfrom(s, r, sizeof r - 1, 0, (struct sockaddr *)&from, &fl);
    if (k > 11 && memcmp(r, "COCO3SHARE ", 11) == 0)
    {
      r[k] = 0;
      const int port = atoi(r + 11);
      const char *nm = strchr(r + 11, ' ');
      char ip[16]; inet_ntop(AF_INET, &from.sin_addr, ip, sizeof ip);
      snprintf(s_host, sizeof s_host, "%s:%d", ip, port > 0 ? port : NS_PORT);
      snprintf(s_name, sizeof s_name, "%s", nm ? nm + 1 : ip);
      found = true;
    }
  }
  close(s);
  if (found) Ns_Log("[SHARE] partage trouve : %s (%s)", s_host, s_name);
  return found;
}

int NetShare_Connect(void)
{
  Ns_Load();
  if (!Net_Connected()) return NS_NOWIFI;
  char buf[96]; uint32_t n = 0;
  int r = NS_NOSERVER;
  if (s_host[0]) r = Ns_Http("GET", "hello", "/", nullptr, nullptr, 0, (uint8_t *)buf, sizeof buf - 1, &n, nullptr);
  if (r == NS_NOSERVER || r == NS_ERR)
  {
    if (!Ns_Discover()) return NS_NOSERVER;
    r = Ns_Http("GET", "hello", "/", nullptr, nullptr, 0, (uint8_t *)buf, sizeof buf - 1, &n, nullptr);
  }
  if (r != NS_OK) return r;
  buf[n] = 0;
  if (strncmp(buf, "COCO3SHARE", 10) != 0) return NS_ERR;
  const char *nl = strchr(buf, '\n');
  if (nl && nl[1]) { snprintf(s_name, sizeof s_name, "%s", nl + 1); char *e = strchr(s_name, '\n'); if (e) *e = 0; }
  Ns_Save();
  return NS_OK;
}

int NetShare_List(const char *path)
{
  if (!s_ent) s_ent = (NsEntry *)heap_caps_calloc(NS_MAX_ENTRIES, sizeof(NsEntry), MALLOC_CAP_SPIRAM);
  if (!s_ent) return NS_NOMEM;
  s_count = 0;
  uint8_t *txt = nullptr; uint32_t n = 0;
  const int r = Ns_Http("GET", "list", path, nullptr, nullptr, 0, nullptr, 0, &n, &txt);
  if (r != NS_OK) return r;
  char *p = (char *)txt, *end = (char *)txt + n;
  while (p < end && s_count < NS_MAX_ENTRIES)
  {
    char *nl = (char *)memchr(p, '\n', (size_t)(end - p));
    char *le = nl ? nl : end;
    if (le > p && le[-1] == '\r') le[-1] = 0;
    *le = 0;                                    // le tampon a de la place : '\n' remplacé
    NsEntry &e = s_ent[s_count];
    if (p[0] == 'D' && p[1] == ' ') { e.dir = 1; e.size = 0; snprintf(e.name, sizeof e.name, "%s", p + 2); s_count++; }
    else if (p[0] == 'F' && p[1] == ' ')
    {
      char *sp = strchr(p + 2, ' ');
      if (sp) { e.dir = 0; e.size = (uint32_t)strtoul(p + 2, nullptr, 10); snprintf(e.name, sizeof e.name, "%s", sp + 1); s_count++; }
    }
    if (!nl) break;
    p = nl + 1;
  }
  free(txt);
  return s_count;
}

int NetShare_Read(const char *path, uint32_t off, uint32_t len, uint8_t *buf, uint32_t *got)
{
  char extra[48]; snprintf(extra, sizeof extra, "&off=%lu&len=%lu", (unsigned long)off, (unsigned long)len);
  return Ns_Http("GET", "get", path, extra, nullptr, 0, buf, len, got, nullptr);
}

int NetShare_Get(const char *path, uint8_t **buf, uint32_t *size)
{
  uint32_t n = 0;
  const int r = Ns_Http("GET", "get", path, nullptr, nullptr, 0, nullptr, 0, &n, buf);
  if (r == NS_OK) { *size = n; s_gets++; Ns_Log("[SHARE] %s recu du PC (%lu o)", path, (unsigned long)n); }
  return r;
}

int NetShare_Put(const char *path, const uint8_t *data, uint32_t size)
{
  char ok[16]; uint32_t n = 0;
  const int r = Ns_Http("PUT", "put", path, nullptr, data, size, (uint8_t *)ok, sizeof ok, &n, nullptr);
  if (r == NS_OK) s_saves++; else s_save_fail++;
  return r;
}

static void Ns_PendingMounts(void)
{
  static uint32_t next = 0;
  if ((int32_t)(millis() - next) < 0) return;
  bool any = false;
  for (uint8_t d = 0; d < 4; d++) any = any || s_pend[d][0];
  if (!any) return;
  next = millis() + 5000;                         // un essai toutes les 5 s
  if (!Net_Connected()) return;                   // le Wi-Fi d'abord (essais non comptés)
  const int c = NetShare_Connect();
  for (uint8_t d = 0; d < 4; d++)
  {
    if (!s_pend[d][0]) continue;
    int r = c;
    uint8_t *img = nullptr; uint32_t size = 0;
    if (r == NS_OK) r = NetShare_Get(s_pend[d], &img, &size);
    if (r == NS_OK)
    {
      if (!Disk_AdoptNet(d, s_pend[d], img, size)) { free(img); Ns_Log("[SHARE] lecteur %u : %s non monte (un autre disque a ete choisi)", d, s_pend[d]); }
      s_pend[d][0] = 0;
      continue;
    }
    if (r == NS_NOTFOUND || r == NS_PIN || ++s_pend_tries[d] >= 24)   // 24 x 5 s = 2 min
    {
      Ns_Log("[SHARE] lecteur %u : %s non monte : %s", d, s_pend[d], NetShare_ErrorText(r));
      s_pend[d][0] = 0;
    }
  }
}

void NetShare_Poll(void)
{
  static uint32_t last = 0;
  if (millis() - last < 500) return;
  last = millis();
  Ns_PendingMounts();
  for (uint8_t d = 0; d < 4; d++)
  {
    const uint8_t *data; uint32_t size, stamp, ms; const char *path;
    if (!Disk_NetDirty(d, &data, &size, &path, &stamp, &ms)) continue;
    if (millis() - ms < 2000) continue;          // le CoCo écrit encore
    if (!Net_Connected()) continue;
    const int r = NetShare_Put(path, data, size);
    Disk_NetSaved(d, stamp, r == NS_OK);
    Ns_Log(r == NS_OK ? "[SHARE] lecteur %u : %s renvoye au PC (%lu o)" : "[SHARE] lecteur %u : echec du renvoi de %s (%lu o) : nouvel essai", d, path, (unsigned long)size);
    if (r != NS_OK) last = millis() + 5000;      // attente avant le prochain essai
    break;                                       // un disque par passage
  }
}

void NetShare_Report(void)
{
  Ns_Load();
  Ns_Log("[SHARE] serveur %s (%s), PIN %s ; disques recus %lu, renvois %lu, echecs %lu",
         s_host[0] ? s_host : "aucun", s_name[0] ? s_name : "-", s_pin[0] ? "memorise" : "absent",
         (unsigned long)s_gets, (unsigned long)s_saves, (unsigned long)s_save_fail);
}

#else
// Banc : pas de réseau
bool NetShare_HasPin(void) { return s_pin[0] != 0; }
void NetShare_SetPin(const char *pin) { strncpy(s_pin, pin ? pin : "", sizeof s_pin - 1); }
int  NetShare_Connect(void) { return NS_NOWIFI; }
int  NetShare_List(const char *) { s_count = 0; return NS_NOWIFI; }
int  NetShare_Read(const char *, uint32_t, uint32_t, uint8_t *, uint32_t *got) { if (got) *got = 0; return NS_NOWIFI; }
int  NetShare_Get(const char *, uint8_t **, uint32_t *) { return NS_NOWIFI; }
int  NetShare_Put(const char *, const uint8_t *, uint32_t) { return NS_NOWIFI; }
void NetShare_Poll(void) {}
void NetShare_Report(void) {}
#endif
