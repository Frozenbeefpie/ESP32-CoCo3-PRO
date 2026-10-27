// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// AUTORUN : frappe automatique d'une commande au démarrage (voir autorun.h).

#include "autorun.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include "gime.h"

#ifdef ESP_PLATFORM
#include <Arduino.h>
#include <Preferences.h>
#endif

extern void     Kbd_Inject(uint8_t mod, uint8_t key);   // main.cpp : instantané clavier (0, 0 = tout relâché)
extern uint16_t Cpu_PC(void);                           // main.cpp
extern uint16_t Cpu_SP(void);                           // main.cpp
extern bool     Emu_Running(void);                      // main.cpp : ni menu F12, ni arrêt
extern bool     SD_MMC_MountOK;
extern void     Sd_Lock(void);
extern void     Sd_Unlock(void);
extern void     Log_Line(const char *s);

#ifdef ESP_PLATFORM
#define AR_PATH "/sdcard/ESP-BOOT/AUTORUN.TXT"
static uint32_t AR_Ms(void) { return (uint32_t)millis(); }
#else
static const char *AR_PathPc(void) { const char *p = getenv("AUTORUNFILE"); return p ? p : "AUTORUN.TXT"; }
#define AR_PATH AR_PathPc()
extern volatile uint64_t g_emu_ticks;
static uint32_t AR_Ms(void) { return (uint32_t)(g_emu_ticks / 14318ull); }   // banc : temps émulé
#endif

#define AR_LINES  32
#define AR_LEN    128
#define AR_PRESS_MS   60      // touche enfoncée
#define AR_RELEASE_MS 60      // touche relâchée avant la suivante
#define AR_IDLE_MS    500     // BASIC au repos depuis
#define AR_START_MAX  30000u  // délai maximal pour la première invite

static uint8_t  s_mode = AR_OFF;
static char     s_cmd[AR_LEN] = "";
static char   (*s_lines)[AR_LEN] = nullptr;   // lignes à taper (PSRAM / tas)
static int      s_nlines = 0, s_line = 0, s_pos = 0;
enum { S_NONE = 0, S_WAIT, S_DOWN, S_UP, S_DONE, S_CANCEL, S_TIMEOUT };
static uint8_t  s_st = S_NONE;
static uint32_t s_t = 0, s_t0 = 0, s_idle_since = 0, s_samples = 0, s_hits = 0;
static volatile bool s_user_key = false;
static bool     s_injecting = false;
static void AR_Release(void);
static void AR_Log(const char *t)
{
  Log_Line(t);
#ifndef ESP_PLATFORM
  fprintf(stderr, "%s\n", t);                     // banc
#endif
}

// ---------------------------------------------------------------- réglages
#ifdef ESP_PLATFORM
static void AR_Save(void)
{
  Preferences p;
  if (!p.begin("cc3b", false)) return;
  p.putUChar("mode", s_mode);
  p.putString("cmd", s_cmd);
  p.end();
}
static void AR_Load(void)
{
  Preferences p;
  if (!p.begin("cc3b", true)) return;
  s_mode = p.getUChar("mode", AR_OFF); if (s_mode >= AR_MODES) s_mode = AR_OFF;
  String c = p.getString("cmd", "");
  strncpy(s_cmd, c.c_str(), sizeof s_cmd - 1);
  p.end();
}
#else
static void AR_Save(void) {}
static void AR_Load(void)
{
  const char *m = getenv("AUTORUN");               // banc : AUTORUN=1 (commande AUTORUNCMD) ou 2 (fichier)
  s_mode = m ? (uint8_t)atoi(m) : AR_OFF;
  const char *c = getenv("AUTORUNCMD"); snprintf(s_cmd, sizeof s_cmd, "%s", c ? c : "");
}
#endif
uint8_t     Autorun_Mode(void) { return s_mode; }
void        Autorun_SetMode(uint8_t m) { s_mode = m < AR_MODES ? m : AR_OFF; AR_Save(); }
const char *Autorun_Command(void) { return s_cmd; }
void        Autorun_SetCommand(const char *c) { snprintf(s_cmd, sizeof s_cmd, "%s", c ? c : ""); AR_Save(); }
void        Autorun_UserKey(void) { if (!s_injecting && Emu_Running() && (s_st == S_WAIT || s_st == S_DOWN || s_st == S_UP)) s_user_key = true; }   // pas pendant l'écran d'accueil
void        Autorun_Cancel(void) { if (s_st == S_WAIT || s_st == S_DOWN || s_st == S_UP) { if (s_st == S_DOWN) AR_Release(); s_st = S_CANCEL; AR_Log("[AUTORUN] annule (ESC a l ecran d accueil)"); } }

bool Autorun_FileExists(void)
{
  if (!SD_MMC_MountOK) return false;
  Sd_Lock();
  const int fd = open(AR_PATH, O_RDONLY);
  if (fd >= 0) close(fd);
  Sd_Unlock();
  return fd >= 0;
}

static bool AR_Alloc(void)
{
  if (!s_lines) s_lines = (char (*)[AR_LEN])calloc(AR_LINES, AR_LEN);
  return s_lines != nullptr;
}
static void AR_AddLine(const char *l)
{
  if (s_nlines >= AR_LINES) return;
  while (*l == ' ' || *l == '\t') l++;
  if (!*l || *l == '#') return;                    // ligne vide ou commentaire « # »
  char *o = s_lines[s_nlines];
  size_t k = 0;
  for (; *l && k + 1 < AR_LEN; l++) if ((unsigned char)*l >= 32 && (unsigned char)*l < 127) o[k++] = (char)toupper((unsigned char)*l);
  while (k && o[k - 1] == ' ') k--;
  o[k] = 0;
  if (k) s_nlines++;
}
static void AR_ReadFile(void)
{
  if (!SD_MMC_MountOK) return;
  char buf[2048];
  Sd_Lock();
  const int fd = open(AR_PATH, O_RDONLY);
  int n = fd >= 0 ? (int)read(fd, buf, sizeof buf - 1) : -1;
  if (fd >= 0) close(fd);
  Sd_Unlock();
  if (n <= 0) return;
  buf[n] = 0;
  char *p = buf;
  if ((unsigned char)p[0] == 0xEF && (unsigned char)p[1] == 0xBB && (unsigned char)p[2] == 0xBF) p += 3;   // BOM UTF-8 (Notepad)
  while (*p)
  {
    char *e = p; while (*e && *e != '\r' && *e != '\n') e++;
    const char c = *e; *e = 0;
    AR_AddLine(p);
    if (!c) break;
    p = e + 1;
  }
}

void Autorun_Init(void)
{
  AR_Load();
  s_nlines = 0; s_line = 0; s_pos = 0; s_st = S_NONE;
  if (s_mode == AR_OFF || !AR_Alloc()) return;
  if (s_mode == AR_COMMAND) AR_AddLine(s_cmd);
  else AR_ReadFile();
  char b[200];
  if (!s_nlines)
  {
    snprintf(b, sizeof b, "[AUTORUN] %s : rien a taper", s_mode == AR_COMMAND ? "commande vide" : "fichier /ESP-BOOT/AUTORUN.TXT absent ou vide");
    AR_Log(b); s_st = S_DONE; return;
  }
  snprintf(b, sizeof b, "[AUTORUN] %d ligne(s) a taper (%s), premiere : %s", s_nlines, s_mode == AR_COMMAND ? "commande" : "AUTORUN.TXT", s_lines[0]);
  AR_Log(b);
  s_st = S_WAIT; s_t0 = AR_Ms(); s_idle_since = 0; s_samples = s_hits = 0;
}

// ---------------------------------------------------------------- frappe
// ASCII -> code de touche USB (clavier américain, comme le reste de l'émulateur) ; bit 8 = SHIFT
static uint16_t AR_Key(char c)
{
  if (c >= 'A' && c <= 'Z') return (uint16_t)(0x04 + (c - 'A'));
  if (c >= '1' && c <= '9') return (uint16_t)(0x1E + (c - '1'));
  switch (c)
  {
    case '0': return 0x27;  case '\r': return 0x28; case ' ': return 0x2C;
    case '-': return 0x2D;  case '=': return 0x2E;  case '[': return 0x2F; case ']': return 0x30;
    case '\\': return 0x31; case ';': return 0x33;  case '\'': return 0x34; case '`': return 0x35;
    case ',': return 0x36;  case '.': return 0x37;  case '/': return 0x38;
    case '!': return 0x11E; case '@': return 0x11F; case '#': return 0x120; case '$': return 0x121;
    case '%': return 0x122; case '^': return 0x123; case '&': return 0x124; case '*': return 0x125;
    case '(': return 0x126; case ')': return 0x127; case '_': return 0x12D; case '+': return 0x12E;
    case '{': return 0x12F; case '}': return 0x130; case '|': return 0x131; case ':': return 0x133;
    case '"': return 0x134; case '~': return 0x135; case '<': return 0x136; case '>': return 0x137;
    case '?': return 0x138;
  }
  return 0;
}
static void AR_Press(uint16_t k)
{
  s_injecting = true;
  Kbd_Inject((k & 0x100) ? 0x02 : 0x00, (uint8_t)(k & 0xFF));
  s_injecting = false;
}
static void AR_Release(void) { s_injecting = true; Kbd_Inject(0, 0); s_injecting = false; }

// Le processeur attend-il une touche ? (boucles de lecture du clavier du BASIC)
static bool AR_InKeyWait(uint16_t pc)
{
  return (pc >= 0xA1B1 && pc <= 0xA250)     // CONSOLE IN : clignotement, KEYIN
      || (pc >= 0xA7D1 && pc <= 0xA7DB)     // délai anti-rebond
      || (pc >= 0xA0C0 && pc <= 0xA0F0)     // attente de touche (CoCo 3)
      || (pc >= 0xF77E && pc <= 0xF7AD);    // curseur de l'écran 40 / 80 colonnes
}

static bool AR_AtPrompt(void)
{
  const uint16_t curlin = (uint16_t)(gime_peek(0x68) << 8 | gime_peek(0x69));
  if (curlin != 0xFFFF && curlin != 0) return false;   // un programme tourne (INPUT, LINE INPUT : même pile)
  const uint16_t sp = Cpu_SP();
  for (int k = 0; k < 40; k++)                    // « A3 9D AC 7F » : la saisie appelée directement par la boucle…
    if (gime_peek((uint16_t)(sp + k)) == 0xA3 && gime_peek((uint16_t)(sp + k + 1)) == 0x9D
     && gime_peek((uint16_t)(sp + k + 2)) == 0xAC && gime_peek((uint16_t)(sp + k + 3)) == 0x7F) return true;
  return false;
}

void Autorun_Poll(void)
{
  if (s_st == S_NONE || s_st >= S_DONE) return;
  const uint32_t now = AR_Ms();
  if (s_user_key)
  {
    if (s_st == S_DOWN) AR_Release();
    s_st = S_CANCEL;
    AR_Log("[AUTORUN] annule : une touche du clavier a ete tapee");
    return;
  }
  if (!Emu_Running()) { s_idle_since = 0; s_samples = s_hits = 0; if (s_st == S_WAIT && s_line == 0) s_t0 = now; return; }   // écran d accueil, menu : le délai de 30 s attend
  switch (s_st)
  {
    case S_WAIT:
    {
      const bool direct = AR_AtPrompt();
      const bool wait = AR_InKeyWait(Cpu_PC());
      if (!direct) { s_idle_since = 0; s_samples = s_hits = 0; }
      else
      {
        if (!s_idle_since) { s_idle_since = now; s_samples = s_hits = 0; }
        s_samples++; if (wait) s_hits++;
        if (now - s_idle_since >= AR_IDLE_MS)
        {
          if (s_samples >= 8 && s_hits * 10u >= s_samples * 8u)   // 80 % des relevés dans l'attente d'une touche
          {
            char b[200]; snprintf(b, sizeof b, "[AUTORUN] ligne %d : %s", s_line + 1, s_lines[s_line]); AR_Log(b);
            s_pos = 0; s_st = S_UP; s_t = now; return;
          }
          s_idle_since = 0;                                       // occupé : on recommence la fenêtre
        }
      }
      if (s_line == 0 && now - s_t0 > AR_START_MAX)
      {
        s_st = S_TIMEOUT;
        AR_Log("[AUTORUN] abandon : pas d invite du BASIC apres 30 s (cartouche, OS-9 ?)");
      }
      return;
    }
    case S_UP:                                     // touche relâchée : la suivante après le délai
      if (now - s_t < AR_RELEASE_MS) return;
      {
        const char *l = s_lines[s_line];
        const char c = l[s_pos] ? l[s_pos] : '\r';
        const uint16_t k = AR_Key(c);
        if (!k && c != '\r') { s_pos++; return; } // caractère sans touche : sauté
        AR_Press(k); s_st = S_DOWN; s_t = now;
      }
      return;
    case S_DOWN:
      if (now - s_t < AR_PRESS_MS) return;
      AR_Release(); s_t = now;
      if (!s_lines[s_line][s_pos])                // c'était ENTER
      {
        s_line++;
        if (s_line >= s_nlines) { s_st = S_DONE; AR_Log("[AUTORUN] termine"); return; }
        s_st = S_WAIT; s_idle_since = 0; s_samples = s_hits = 0;
        return;
      }
      s_pos++; s_st = S_UP;
      return;
  }
}

void Autorun_StatusLine(char *b, size_t n)
{
  switch (s_st)
  {
    case S_WAIT:    snprintf(b, n, "waiting for BASIC (line %d/%d)", s_line + 1, s_nlines); break;
    case S_DOWN: case S_UP: snprintf(b, n, "typing line %d/%d", s_line + 1, s_nlines); break;
    case S_DONE:    snprintf(b, n, s_nlines ? "done (%d line%s)" : "nothing to type", s_nlines, s_nlines > 1 ? "s" : ""); break;
    case S_CANCEL:  snprintf(b, n, "cancelled by a key"); break;
    case S_TIMEOUT: snprintf(b, n, "no BASIC prompt (30 s)"); break;
    default:        snprintf(b, n, "%s", s_mode == AR_OFF ? "off" : "at next start"); break;
  }
}
