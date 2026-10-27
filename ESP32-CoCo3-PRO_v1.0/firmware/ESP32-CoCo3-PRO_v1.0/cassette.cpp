// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Magnétophone virtuel : lecture .CAS / .WAV, enregistrement .CAS (voir cassette.h).

#include "cassette.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#ifdef ESP_PLATFORM
#pragma GCC optimize ("O2")
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static void Cas_Yield(void) { vTaskDelay(1); }
static void *Cas_Mem(size_t n) { void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); return p ? p : malloc(n); }
static void  Cas_Free(void *p) { heap_caps_free(p); }
static const char *Cas_Root(void) { return "/sdcard"; }
#else
static void *Cas_Mem(size_t n) { return malloc(n); }
static void  Cas_Free(void *p) { free(p); }
static const char *Cas_Root(void) { const char *r = getenv("CAS_ROOT"); return r ? r : "sdroot"; }
static void Cas_Yield(void) {}
#endif

extern void Sd_Lock(void);
extern void Sd_Unlock(void);
extern void Cas_Notify(const char *l1, const char *l2);      // main.cpp : message à l'écran
extern void Cas_SetTurbo(bool on);                           // main.cpp : Overclock MACHINE (chargement rapide)

bool g_cas_motor = false;
namespace {
// ---------------------------------------------------------------- constantes
const uint32_t TICKS_HZ = 14318180u;
const uint32_t T_ONE = 5966u, T_ZERO = 11932u;               // durée d'un bit : 2400 Hz / 1200 Hz
const uint32_t REC_RATE = 22050u;
const uint32_t REC_STEP_Q8 = (uint32_t)(((uint64_t)TICKS_HZ << 8) / REC_RATE);   // tics par échantillon, Q8
const uint32_t CAS_MAX = 2u * 1024u * 1024u;                 // CAS : 2 Mo au plus (chargé en PSRAM)
const uint32_t WR_BYTES = 128u * 1024u, WR_BITS = WR_BYTES * 8u;   // anneau du .WAV lu (1 Mbit)
const uint32_t RR_BYTES = 256u * 1024u;                      // file de l'enregistrement
const uint32_t DEC_GAP = 34000u, DEC_MIN = 1500u;            // cycle trop long pour l'amorce (> 2,4 ms) / parasite
const uint64_t DEC_SILENCE = 14318180u;                        // 1 s sans front : calage perdu

// ---------------------------------------------------------------- état
volatile uint8_t s_mode = CAS_EMPTY;
char     s_path[200] = "";
bool     s_motor = false;
// bande immobile après le démarrage du moteur jusqu'à la 1re lecture de $FF20 (la ROM attend ~0,6 s,
// LA7D1, avant de lire)
bool     s_hold = false;
uint32_t s_hold_t = 0;                                          // tics passés en attente (garde : 3 s)
uint32_t s_lastt = 0;
uint64_t s_pos = 0;                                           // position de la bande (tics émulés, moteur en marche)
bool     s_fast = true, s_turbo = false;
uint8_t  s_fmt = CAS_FMT_CAS, s_rec_fmt = CAS_FMT_CAS;
uint8_t  s_level = 128;                                       // dernier niveau du CNA
volatile uint32_t s_underruns = 0, s_rec_drop = 0;
volatile bool s_note_play_save = false, s_note_auto = false, s_auto_open = false;
volatile bool s_close_play = false;                            // CSAVE pendant une lecture : la bande lue est fermée par loop

// lecture .CAS
uint8_t *s_cas = nullptr; uint32_t s_cas_len = 0;
uint32_t s_bit = 0; uint64_t s_bit_t0 = 0; uint32_t s_bit_len = T_ONE;
bool     s_cas_end = false;

// lecture .WAV
bool     s_is_wav = false;
int      s_wfd = -1;
long     s_wdata = 0; uint32_t s_wlen = 0, s_wread = 0;       // début et taille des données, octets lus
uint16_t s_wch = 1, s_wbits = 16; uint32_t s_wrate = 44100, s_wtotal = 0;
uint32_t s_wup = 1;                                             // sur-échantillonnage du comparateur (≥ 44 kHz : fronts à ±23 µs)
int32_t  s_wprev = 0;
// un seul remplissage du .WAV à la fois (menu : insertion / retour au début ; loop : lecture
// anticipée).
int      s_wlock = 0;
void WLock(void) { while (__atomic_exchange_n(&s_wlock, 1, __ATOMIC_ACQUIRE)) Cas_Yield(); }
bool WTryLock(void) { return __atomic_exchange_n(&s_wlock, 1, __ATOMIC_ACQUIRE) == 0; }
void WUnlock(void) { __atomic_store_n(&s_wlock, 0, __ATOMIC_RELEASE); }
// analyse de la file (commande W) : blocs reconnus, sommes fausses
bool     s_an_todo = false; uint16_t s_an_blk = 0, s_an_bad = 0; bool s_an_done = false;
uint32_t s_inband = 0, s_bandmax = 2; bool s_tog = false;           // silence juste après un demi-cycle // échantillon filtré précédent…
uint8_t *s_wr = nullptr;                                      // anneau de bits (comparateur)
volatile uint32_t s_wfill = 0;                                // échantillons prêts (index absolu)
volatile uint32_t s_wcpu = 0;                                 // échantillon lu par le CPU
volatile bool s_weof = false;
int32_t  s_dc = 0, s_env = 2000; uint8_t s_cmp = 0;           // suppression du continu, enveloppe, sortie
int32_t  s_lp = 0, s_lpk = 256;                                // passe-bas ~6 kHz avant le comparateur (souffle)
uint8_t  s_wlast = 0;
uint8_t *s_wio = nullptr;                                     // tampon de lecture SD (8 Ko)

// enregistrement
uint8_t *s_rr = nullptr; volatile uint32_t s_rh = 0, s_rt = 0;
uint64_t s_rnext_q8 = 0;                                      // prochain échantillon .WAV (position Q8)
bool     s_rec_on = false;                                    // la file reçoit (bande en enregistrement, calée)
int      s_rfd = -1; uint32_t s_rbytes = 0, s_rlast_hdr_ms = 0;

// décodeurs du signal du CNA (fronts montants et descendants, en parallèle)
struct Dec { uint64_t last; bool has, sync; uint16_t sr; uint8_t nb, wn, wi; uint32_t win[8], thr, lead; };
Dec s_dr, s_df; int8_t s_owner = -1;

// ---------------------------------------------------------------- outils
void Path(char *o, size_t n, const char *rel) { snprintf(o, n, "%s%s%s", Cas_Root(), rel[0] == '/' ? "" : "/", rel); }
int  Rd(int fd, void *p, int n) { int g = 0; while (g < n) { const int k = n - g < 8192 ? n - g : 8192; Sd_Lock(); const int r = (int)read(fd, (uint8_t *)p + g, k); Sd_Unlock(); if (r <= 0) break; g += r; } return g; }
int  Wr(int fd, const void *p, int n) { Sd_Lock(); const int r = (int)write(fd, p, n); Sd_Unlock(); return r; }
void Seek(int fd, long o) { Sd_Lock(); lseek(fd, o, SEEK_SET); Sd_Unlock(); }
void Close(int &fd) { if (fd >= 0) { Sd_Lock(); close(fd); Sd_Unlock(); fd = -1; } }
bool HasExt(const char *p, const char *e) { const size_t a = strlen(p), b = strlen(e); if (a < b) return false; for (size_t i = 0; i < b; i++) { char c = p[a - b + i]; if (c >= 'a' && c <= 'z') c -= 32; if (c != e[i]) return false; } return true; }
void Put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }

// ---------------------------------------------------------------- file d'enregistrement (coeur CPU -> loop)
inline void RPush(uint8_t v)
{
  const uint32_t h = s_rh, nh = (h + 1u) & (RR_BYTES - 1u);
  if (nh == s_rt) { s_rec_drop++; return; }
  s_rr[h] = v;
  __atomic_store_n(&s_rh, nh, __ATOMIC_RELEASE);
}

// ---------------------------------------------------------------- décodage du CNA (enregistrement .CAS)
void Emit(uint8_t b);
void Dec_Reset(Dec &d) { memset(&d, 0, sizeof d); }
void Dec_Tail(Dec &d) { if (d.sync && d.nb == 7) Emit((uint8_t)((d.sr >> 9) & 0x7F)); }   // dernier octet : son dernier cycle n'a pas de front suivant (bit 7…)
void Dec_All(void) { Dec_Tail(s_dr); Dec_Tail(s_df); Dec_Reset(s_dr); Dec_Reset(s_df); s_owner = -1; }
void OnSync(void)
{
  // CSAVE avec une bande en lecture : comme sans bande, une NOUVELLE bande est créée (la bande lue
  // n'est jamais effacée)
  if (s_mode == CAS_PLAY) { s_close_play = true; s_mode = CAS_EMPTY; s_hold = false; }
  if (s_mode == CAS_EMPTY)                                    // CSAVE sans bande : bande automatique
  {
    s_mode = CAS_RECORD; s_rec_fmt = s_fmt; s_auto_open = true; s_note_auto = true;
    s_rnext_q8 = s_pos << 8;
    s_rec_on = true;
    if (s_fast && !s_turbo) { s_turbo = true; Cas_SetTurbo(true); }
    return;
  }
  s_rec_on = true;
}
void Emit(uint8_t b) { if (s_mode == CAS_RECORD && s_rec_fmt == CAS_FMT_CAS && s_rec_on && s_rr) RPush(b); }
void Dec_Edge(Dec &d, int id, uint64_t p)
{
  if (!d.has) { d.has = true; d.last = p; return; }
  const uint64_t per64 = p - d.last; d.last = p;
  if (s_owner >= 0 && s_owner != id) return;                  // l'autre décodeur a trouvé la phase
  // la ROM s'arrête entre deux octets $55 et le bloc suivant (somme de contrôle du bloc ~11 cycles par
  // octet, jusqu'à 45 000 tics pour 255 octets)
  if (per64 < DEC_MIN) return;                                // parasite
  if (per64 > DEC_SILENCE)                                    // longue coupure : calage perdu
  {
    const bool was = d.sync;
    Dec_Tail(d);
    d.sync = false; d.nb = 0; d.wn = 0; d.wi = 0; d.thr = 0; d.lead = 0;
    if (was && s_owner == id) s_owner = -1;
    return;
  }
  const uint32_t per = (uint32_t)per64;
  if (!d.sync && per <= DEC_GAP)
  {
    d.win[d.wi] = per; d.wi = (uint8_t)((d.wi + 1) & 7); if (d.wn < 8) d.wn++;
    uint32_t mn = 0xFFFFFFFFu, mx = 0;
    for (int i = 0; i < d.wn; i++) { if (d.win[i] < mn) mn = d.win[i]; if (d.win[i] > mx) mx = d.win[i]; }
    d.thr = (d.wn >= 6 && (uint64_t)mx * 10u > (uint64_t)mn * 15u) ? mn + mn / 2u : 0u;   // cycles courts ET longs vus
  }
  const uint16_t bit = (d.thr && per < d.thr) ? 1 : 0;
  d.sr = (uint16_t)((d.sr >> 1) | (bit << 15));
  if (!d.sync)
  {
    // L'amorce ($55 = 1010...) se répète tous les 2 bits : la limite des octets n'est connue qu'à
    // l'octet de synchronisation $3C qui la suit (dans sr : $55 en bits 0-7, $3C en bits 8-15).
    if (d.nb < 16) d.nb++;
    d.lead++;
    if (d.nb >= 16 && d.thr && d.sr == 0x3C55)
    {
      d.sync = true; d.nb = 0; s_owner = (int8_t)id;
      OnSync();
      uint32_t n = d.lead > 8 ? (d.lead - 8) / 8 : 1; if (n > 128) n = 128; if (!n) n = 1;
      for (uint32_t i = 0; i < n; i++) Emit(0x55);           // amorce (longueur reçue, 128 au plus)
      Emit(0x3C);
    }
    return;
  }
  if (++d.nb == 8) { Emit((uint8_t)(d.sr >> 8)); d.nb = 0; }
}

// ---------------------------------------------------------------- avance du temps (toutes les entrées)
void Adv(uint32_t t)
{
  const uint32_t d = t - s_lastt;
  s_lastt = t;
  if (!s_motor) return;
  if (s_hold)
  {
    s_hold_t += d;
    if (s_hold_t < 3u * TICKS_HZ) return;                        // garde : un chargeur qui ne lit jamais $FF20
    s_hold = false;
  }
  s_pos += d;
  if (s_mode == CAS_RECORD && s_rec_fmt == CAS_FMT_WAV && s_rec_on && s_rr)   // échantillons 22 050 Hz du niveau tenu
  {
    const uint64_t now_q8 = s_pos << 8;
    uint32_t guard = 0;
    while (s_rnext_q8 <= now_q8 && guard++ < 400000u) { RPush(s_level); s_rnext_q8 += REC_STEP_Q8; }
    if (s_rnext_q8 <= now_q8) s_rnext_q8 = now_q8 + REC_STEP_Q8;
  }
}

// ---------------------------------------------------------------- niveau de la bande en lecture
uint8_t Cas_Bit(void)
{
  if (s_mode != CAS_PLAY) return 0;
  if (!s_is_wav)
  {
    if (!s_cas || s_cas_end) return 0;
    if (s_pos < s_bit_t0) { s_bit = 0; s_bit_t0 = 0; s_bit_len = (s_cas[0] & 1) ? T_ONE : T_ZERO; }
    while (s_pos >= s_bit_t0 + s_bit_len)
    {
      s_bit_t0 += s_bit_len;
      // après le dernier octet, 16 octets $55 virtuels (le bruit d'une vraie bande) la ROM mesure le
      // dernier bit jusqu'au front suivant
      if (++s_bit >= s_cas_len * 8u + 128u) { s_cas_end = true; return 0; }
      s_bit_len = (s_bit >= s_cas_len * 8u) ? (((0x55u >> (s_bit & 7)) & 1) ? T_ONE : T_ZERO)
                                            : (((s_cas[s_bit >> 3] >> (s_bit & 7)) & 1) ? T_ONE : T_ZERO);
    }
    return (s_pos - s_bit_t0) < (s_bit_len >> 1) ? 1 : 0;   // une période : moitié haute puis moitié basse
  }
  if (!s_wr) return 0;
  const uint32_t s = (uint32_t)((s_pos * (uint64_t)(s_wrate * s_wup)) / TICKS_HZ);
  s_wcpu = s;
  if (s >= s_wfill) { if (!s_weof) s_underruns++; return s_weof ? 0 : s_wlast; }
  s_wlast = (uint8_t)((s_wr[(s >> 3) & (WR_BYTES - 1u)] >> (s & 7)) & 1);
  return s_wlast;
}

// ---------------------------------------------------------------- WAV : en-tête, lecture anticipée
bool Wav_Header(int fd)
{
  uint8_t h[12];
  if (Rd(fd, h, 12) != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4)) return false;
  long pos = 12; bool fmt = false;
  for (int g = 0; g < 32; g++)
  {
    uint8_t c[8];
    if (Rd(fd, c, 8) != 8) break;
    const uint32_t len = (uint32_t)(c[4] | c[5] << 8 | c[6] << 16 | (uint32_t)c[7] << 24);
    pos += 8;
    if (!memcmp(c, "fmt ", 4))
    {
      uint8_t f[16]; if (len < 16 || Rd(fd, f, 16) != 16) break;
      const uint16_t tag = (uint16_t)(f[0] | f[1] << 8);
      s_wch = (uint16_t)(f[2] | f[3] << 8);
      s_wrate = (uint32_t)(f[4] | f[5] << 8 | f[6] << 16 | (uint32_t)f[7] << 24);
      s_wbits = (uint16_t)(f[14] | f[15] << 8);
      fmt = (tag == 1 || tag == 0xFFFE) && (s_wbits == 8 || s_wbits == 16) && s_wch >= 1 && s_wch <= 2 && s_wrate >= 4000 && s_wrate <= 96000;
      pos += (long)len + (long)(len & 1); Seek(fd, pos);
    }
    else if (!memcmp(c, "data", 4)) { s_wdata = pos; s_wlen = len; break; }
    else { pos += (long)len + (long)(len & 1); Seek(fd, pos); }
  }
  if (!fmt || !s_wlen) return false;
  s_wtotal = s_wlen / (uint32_t)((s_wbits / 8) * s_wch);
  return true;
}
void Wav_ResetRing(void)
{
  s_wfill = 0; s_wcpu = 0; s_weof = false; s_wread = 0; s_dc = 0; s_env = 2000; s_cmp = 0; s_wlast = 0; s_lp = 0; s_wprev = 0;
  s_wup = (44100u + s_wrate - 1u) / s_wrate; if (s_wup < 1u) s_wup = 1u; if (s_wup > 8u) s_wup = 8u;   // 11 025 Hz : x4
  s_inband = 0; s_tog = false; s_bandmax = (s_wrate * s_wup * 90u + 999999u) / 1000000u; if (!s_bandmax) s_bandmax = 1;   // ~90 µs
  s_lpk = (s_wrate <= 11025u) ? 250 : (s_wrate <= 16000u) ? 232 : (s_wrate <= 22050u) ? 210 : (s_wrate <= 32000u) ? 177
        : (s_wrate <= 44100u) ? 147 : (s_wrate <= 48000u) ? 139 : (s_wrate <= 88200u) ? 89 : 83;   // 256·(1 − exp(−2π·6000/fréq))
  if (s_wfd >= 0) Seek(s_wfd, s_wdata);
}
// Lit et convertit jusqu'à max_raw octets ; rend le nombre d'octets lus
uint32_t Wav_Fill(uint32_t max_raw)
{
  if (s_wfd < 0 || !s_wr || !s_wio || s_weof) return 0;
  const uint32_t fsz = (uint32_t)((s_wbits / 8) * s_wch);
  uint32_t done = 0;
  {
    const uint32_t cpu = s_wcpu, fill = s_wfill;
    if ((int32_t)(cpu - fill) > 0)                            // le CPU a dépassé la lecture (Turbo) : on saute
    {
      const uint32_t skip = (cpu - fill + s_wup - 1u) / s_wup;  // en échantillons du fichier
      if ((uint64_t)skip * fsz >= (uint64_t)(s_wlen - s_wread)) { s_weof = true; return 0; }
      s_wread += skip * fsz; Seek(s_wfd, s_wdata + (long)s_wread);
      __atomic_store_n(&s_wfill, fill + skip * s_wup, __ATOMIC_RELEASE);
    }
  }
  while (done < max_raw && !s_weof)
  {
    const int32_t ahead = (int32_t)(s_wfill - s_wcpu);
    const int32_t room = (int32_t)WR_BITS - 65536 - (ahead > 0 ? ahead : 0);   // jamais par-dessus ce que le CPU lit
    if (room <= 0) break;
    uint32_t want = 8192u - (8192u % fsz);
    if (want / fsz * s_wup > (uint32_t)room) want = ((uint32_t)room / s_wup / 8u * 8u) * fsz;
    if (!want) break;                                         // file presque pleine
                                                              // la lecture anticipée s'arrêtait après ~20 s : longs .WAV en
                                                              // erreur)
    if (want > s_wlen - s_wread) want = (s_wlen - s_wread) / fsz * fsz;
    if (!want) { s_weof = true; break; }
    const int got = Rd(s_wfd, s_wio, (int)want);
    if (got < (int)fsz) { s_weof = true; break; }
    s_wread += (uint32_t)got; done += (uint32_t)got;
    uint32_t f = s_wfill;
    for (int i = 0; i + (int)fsz <= got; i += (int)fsz)
    {
      const uint8_t *p = s_wio + i;
      int32_t x;
      if (s_wbits == 8) { x = ((int32_t)p[0] - 128) << 8; if (s_wch == 2) x = (x + (((int32_t)p[1] - 128) << 8)) / 2; }
      else { x = (int16_t)(p[0] | p[1] << 8); if (s_wch == 2) x = (x + (int16_t)(p[2] | p[3] << 8)) / 2; }
      s_lp += ((x - s_lp) * s_lpk) >> 8;                      // passe-bas (un vrai CoCo filtre aussi l'entrée)
      x = s_lp;
      s_dc += (x * 16 - s_dc) >> 9;                           // continu (Q4), ~15 Hz à 44 kHz
      const int32_t v = x - (s_dc >> 4);
      const int32_t a = v < 0 ? -v : v;
      if (a > s_env) s_env = a; else s_env -= s_env >> 11;
      int32_t hy = s_env >> 2; if (hy < 96) hy = 96;             // hystérésis 25 % de la crête (était 12,5 %)
      for (uint32_t u = 1; u <= s_wup; u++)                    // interpolation linéaire entre deux échantillons
      {
        const int32_t vi = s_wprev + (int32_t)(((int64_t)(v - s_wprev) * (int32_t)u) / (int32_t)s_wup);
        if (vi > hy) { s_cmp = 1; s_inband = 0; s_tog = false; }   // comparateur à hystérésis
        else if (vi < -hy) { s_cmp = 0; s_inband = 0; s_tog = false; }
        else if (++s_inband == s_bandmax && !s_tog) { s_cmp ^= 1; s_tog = true; }
        // un signal qui revient au repos (silence) juste après un demi-cycle donne quand même le front
        // qui termine le bit (comme un comparateur sans hystérésis) des .WAV fabriqués s'arrêtent sur
        // la somme de…
        uint8_t &byte = s_wr[(f >> 3) & (WR_BYTES - 1u)];
        if (s_cmp) byte |= (uint8_t)(1u << (f & 7)); else byte &= (uint8_t)~(1u << (f & 7));
        f++;
      }
      s_wprev = v;
    }
    __atomic_store_n(&s_wfill, f, __ATOMIC_RELEASE);
  }
  return done;
}

// décode la file du comparateur comme la ROM (période entre fronts montants) : diagnostic
void Wav_Analyse(void)
{
  s_an_blk = 0; s_an_bad = 0; s_an_done = true;
  if (!s_wr || s_wfill > WR_BITS) return;                    // fichier plus long que la file : pas d'analyse
  const uint32_t thr = (s_wrate * s_wup) / 1600u;            // entre 2400 Hz et 1200 Hz
  uint32_t last = 0; bool has = false; int prev = 0;
  uint8_t a = 0; int st = 0, nb = 0; uint8_t cur = 0, len = 0, ck = 0; int left = 0;
  for (uint32_t f = 0; f < s_wfill; f++)
  {
    const int b = (s_wr[f >> 3] >> (f & 7)) & 1;
    if (b && !prev)
    {
      if (has)
      {
        const uint32_t per = f - last;
        const int bit = (per < thr) ? 1 : 0;
        if (per > 4u * thr) { st = 0; a = 0; }               // silence : plus de bloc en cours
        else if (st == 0) { a = (uint8_t)((a >> 1) | (bit << 7)); if (a == 0x3C) { st = 1; nb = 0; cur = 0; } }
        else
        {
          cur = (uint8_t)((cur >> 1) | (bit << 7));
          if (++nb == 8)
          {
            nb = 0;
            if (st == 1) { ck = cur; st = 2; }               // type
            else if (st == 2) { len = cur; ck = (uint8_t)(ck + cur); left = len; st = left ? 3 : 4; }
            else if (st == 3) { ck = (uint8_t)(ck + cur); if (--left == 0) st = 4; }
            else { s_an_blk++; if (cur != ck) s_an_bad++; st = 0; a = 0; }
          }
        }
      }
      has = true; last = f;
    }
    prev = b;
  }
}

// ---------------------------------------------------------------- enregistrement : fichier
void Rec_WriteHeader(void)
{
  if (s_rfd < 0 || s_rec_fmt != CAS_FMT_WAV) return;
  uint8_t h[44];
  memcpy(h, "RIFF", 4); Put32(h + 4, 36u + s_rbytes); memcpy(h + 8, "WAVEfmt ", 8); Put32(h + 16, 16);
  h[20] = 1; h[21] = 0; h[22] = 1; h[23] = 0; Put32(h + 24, REC_RATE); Put32(h + 28, REC_RATE); h[32] = 1; h[33] = 0; h[34] = 8; h[35] = 0;
  memcpy(h + 36, "data", 4); Put32(h + 40, s_rbytes);
  Sd_Lock(); lseek(s_rfd, 0, SEEK_SET); const ssize_t w = write(s_rfd, h, 44); (void)w; lseek(s_rfd, 0, SEEK_END); Sd_Unlock();
}
bool Rec_Open(void)                                           // nouvelle bande : /CASSETTE/TAPEnnnn.CAS|WAV
{
  char dir[64]; Path(dir, sizeof dir, "/CASSETTE");
  Sd_Lock(); mkdir(dir, 0777); Sd_Unlock();
  char rel[64], full[260];
  for (int i = 1; i < 10000; i++)
  {
    snprintf(rel, sizeof rel, "/CASSETTE/TAPE%04d.%s", i, s_rec_fmt == CAS_FMT_WAV ? "WAV" : "CAS");
    Path(full, sizeof full, rel);
    struct stat st; Sd_Lock(); const bool ex = stat(full, &st) == 0; Sd_Unlock();
    if (ex) continue;
    Sd_Lock(); s_rfd = open(full, O_WRONLY | O_CREAT | O_TRUNC, 0666); Sd_Unlock();
    if (s_rfd < 0) return false;
    snprintf(s_path, sizeof s_path, "%s", rel);
    s_rbytes = 0;
    if (s_rec_fmt == CAS_FMT_WAV) { uint8_t z[44]; memset(z, 0, sizeof z); Wr(s_rfd, z, 44); Rec_WriteHeader(); }
    return true;
  }
  return false;
}
uint32_t Rec_Flush(uint32_t max)                              // file -> fichier
{
  if (s_rfd < 0 || !s_rr) return 0;
  uint32_t done = 0;
  while (done < max)
  {
    const uint32_t h = __atomic_load_n(&s_rh, __ATOMIC_ACQUIRE), t = s_rt;
    if (h == t) break;
    uint32_t n = (h > t) ? h - t : RR_BYTES - t;
    if (n > 8192u) n = 8192u;
    const int w = Wr(s_rfd, s_rr + t, (int)n);
    if (w <= 0) break;
    s_rbytes += (uint32_t)w; done += (uint32_t)w;
    __atomic_store_n(&s_rt, (t + (uint32_t)w) & (RR_BYTES - 1u), __ATOMIC_RELEASE);
  }
  return done;
}
void Rec_Close(void)
{
  if (s_rfd < 0) return;
  while (Rec_Flush(65536u)) {}
  Rec_WriteHeader();
  Sd_Lock(); fsync(s_rfd); Sd_Unlock();
  Close(s_rfd);
}
void Turbo_Off(void) { if (s_turbo) { s_turbo = false; Cas_SetTurbo(false); } }
void Free_Play(void)
{
  if (s_cas) { Cas_Free(s_cas); s_cas = nullptr; }
  s_cas_len = 0; s_cas_end = false;
  WLock(); Close(s_wfd); s_is_wav = false; WUnlock();
}
bool Buffers(void)
{
  if (!s_rr) s_rr = (uint8_t *)Cas_Mem(RR_BYTES);
  if (!s_wr) s_wr = (uint8_t *)Cas_Mem(WR_BYTES);
  if (!s_wio) s_wio = (uint8_t *)Cas_Mem(8192);
  return s_rr && s_wr && s_wio;
}
}  // namespace

// ---------------------------------------------------------------- liste des fichiers d'une bande (menu)
namespace {
struct BitParse
{
  uint8_t a = 0, cur = 0, typ = 0, len = 0, ck = 0; int st = 0, nb = 0, left = 0, pos = 0; uint8_t buf[16];
  void (*cb)(const char *name8, uint8_t type, uint8_t ascii, void *u) = nullptr; void *u = nullptr; int found = 0;
  void Reset(void)                                            // silence : le dernier bit d'un bloc n'a pas de front suivant
  {
    if (st == 4 && nb == 7)
    {
      const uint8_t c0 = (uint8_t)(cur >> 1), c1 = (uint8_t)(c0 | 0x80);
      if ((c0 == ck || c1 == ck) && typ == 0 && len >= 11 && cb && found < 68) { cb((const char *)buf, buf[8], buf[9], u); found++; }
    }
    a = 0; st = 0;
  }
  void Bit(int bit)
  {
    if (st == 0) { a = (uint8_t)((a >> 1) | (bit << 7)); if (a == 0x3C) { st = 1; nb = 0; } return; }
    cur = (uint8_t)((cur >> 1) | (bit << 7));
    if (++nb < 8) return;
    nb = 0;
    if (st == 1) { typ = cur; ck = cur; st = 2; }
    else if (st == 2) { len = cur; ck = (uint8_t)(ck + cur); left = len; pos = 0; st = left ? 3 : 4; }
    else if (st == 3) { ck = (uint8_t)(ck + cur); if (pos < 16) buf[pos] = cur; pos++; if (--left == 0) st = 4; }
    else
    {
      if (cur == ck && typ == 0 && len >= 11 && cb && found < 68) { cb((const char *)buf, buf[8], buf[9], u); found++; }
      st = 0; a = 0;
    }
  }
};
}
int Cas_ListTape(const char *path, void (*cb)(const char *name8, uint8_t type, uint8_t ascii, void *u), void *u)
{
  char full[260]; Path(full, sizeof full, path);
  Sd_Lock(); const int fd = open(full, O_RDONLY); Sd_Unlock();
  if (fd < 0) return 0;
  uint8_t *io = (uint8_t *)Cas_Mem(4096);
  if (!io) { int f2 = fd; Close(f2); return 0; }
  BitParse bp; bp.cb = cb; bp.u = u;
  if (!HasExt(path, ".WAV"))
  {
    uint32_t tot = 0; int g;
    while (tot < CAS_MAX && (g = Rd(fd, io, 4096)) > 0)
    {
      for (int i = 0; i < g; i++) for (int k = 0; k < 8; k++) bp.Bit((io[i] >> k) & 1);
      tot += (uint32_t)g;
    }
  }
  else
  {
    // même chaîne que la lecture : en-tête, continu retiré, comparateur à hystérésis ; temps des
    // fronts montants interpolé (1/16 d'échantillon) ; 1 = période < 1/1600 s
    WLock();                                                  // loop ne lit pas le .WAV en cours pendant ce temps
    const uint16_t sch = s_wch, sbits = s_wbits; const uint32_t srate = s_wrate, stot = s_wtotal, slen = s_wlen; const long sdata = s_wdata;
    if (Wav_Header(fd))
    {
      const uint32_t fsz = (uint32_t)((s_wbits / 8) * s_wch), rate = s_wrate;
      const uint32_t thr16 = rate * 16u / 1600u;
      Seek(fd, s_wdata);
      uint32_t left = s_wlen; int32_t dc = 0, env = 2000, prev = 0; int cmp = 0; uint64_t i16 = 0, last16 = 0; bool has = false;
      while (left >= fsz)
      {
        uint32_t want = 4096u - 4096u % fsz; if (want > left) want = left / fsz * fsz;
        const int g = Rd(fd, io, (int)want); if (g < (int)fsz) break;
        left -= (uint32_t)g;
        for (int i = 0; i + (int)fsz <= g; i += (int)fsz, i16 += 16)
        {
          const uint8_t *q = io + i; int32_t x;
          if (s_wbits == 8) { x = ((int32_t)q[0] - 128) << 8; if (s_wch == 2) x = (x + (((int32_t)q[1] - 128) << 8)) / 2; }
          else { x = (int16_t)(q[0] | q[1] << 8); if (s_wch == 2) x = (x + (int16_t)(q[2] | q[3] << 8)) / 2; }
          dc += (x * 16 - dc) >> 9; const int32_t v = x - (dc >> 4);
          const int32_t av = v < 0 ? -v : v; if (av > env) env = av; else env -= env >> 11;
          int32_t hy = env >> 2; if (hy < 96) hy = 96;
          if (!cmp && v > hy)
          {
            cmp = 1;
            const int32_t span = v - prev; const uint64_t fr = span > 0 ? (uint64_t)((int64_t)(v - hy) * 16 / span) : 0;
            const uint64_t t16 = i16 - (fr > 16 ? 16 : fr);
            if (has)
            {
              const uint64_t per = t16 - last16;
              if (per > 4u * thr16) bp.Reset(); else bp.Bit(per < thr16 ? 1 : 0);
            }
            has = true; last16 = t16;
          }
          else if (cmp && v < -hy) cmp = 0;
          prev = v;
        }
      }
      (void)rate;
    }
    s_wch = sch; s_wbits = sbits; s_wrate = srate; s_wtotal = stot; s_wlen = slen; s_wdata = sdata;   // la bande en lecture n'est pas touchée
    WUnlock();
  }
  Cas_Free(io);
  { int f2 = fd; Close(f2); }
  return bp.found;
}

// ---------------------------------------------------------------- coeur CPU
void Cas_Motor(bool on, uint32_t t)
{
  Adv(t);
  if (on == s_motor) return;
  if (on && !Buffers()) return;                               // 1er démarrage du moteur : files en PSRAM (~392 Ko)
  s_motor = on; g_cas_motor = on;
  Dec_All();
  if (on)
  {
    if (s_fast && s_mode != CAS_EMPTY && !s_turbo) { s_turbo = true; Cas_SetTurbo(true); }
    if (s_mode == CAS_RECORD) s_rnext_q8 = s_pos << 8;
    s_hold = (s_mode == CAS_PLAY); s_hold_t = 0;
  }
  else { s_hold = false; Turbo_Off(); }
}
uint32_t s_rd_t = 0, s_rd_min = 0xFFFFFFFFu, s_rd_max = 0, s_rd_n = 0; uint64_t s_rd_sum = 0;   // écart entre lectures de $FF20 (W)
uint8_t Cas_InBit(uint32_t t)                                 // lecture CPU : la bande part
{
  Adv(t); s_hold = false;
  if (!s_motor) return 0;
  const uint32_t d = t - s_rd_t; s_rd_t = t;
  if (d < 20000u) { if (d < s_rd_min) s_rd_min = d; if (d > s_rd_max) s_rd_max = d; s_rd_sum += d; s_rd_n++; }
  return Cas_Bit();
}
uint8_t Cas_Level(uint32_t t) { Adv(t); return (s_motor && Cas_Bit()) ? 255 : 0; }          // AUDIO ON : ne fait pas partir la bande
void Cas_DacWrite(uint8_t level8, uint32_t t)
{
  Adv(t);
  const uint8_t prev = s_level;
  s_level = level8;
  if (!s_motor) return;
  if (s_hold && level8 != prev) s_hold = false;               // le CoCo écrit (CSAVE) : la bande part aussi
  if (s_mode == CAS_RECORD && s_rec_fmt == CAS_FMT_WAV) return;   // WAV : échantillons seulement (Adv)
  if (prev < 128 && level8 >= 128) Dec_Edge(s_dr, 0, s_pos);
  else if (prev >= 128 && level8 < 128) Dec_Edge(s_df, 1, s_pos);
}

// ---------------------------------------------------------------- loop
void Cas_Service(void)
{
  if (s_note_auto) { s_note_auto = false; }
  if (s_close_play) { s_close_play = false; Free_Play(); }
  if (s_auto_open)
  {
    s_auto_open = false;
    if (Rec_Open()) { char b[64]; snprintf(b, sizeof b, "%s", s_path); Cas_Notify("TAPE: RECORDING", b); }
    else { Cas_Notify("TAPE: CANNOT CREATE A FILE", "CHECK THE SD CARD"); s_mode = CAS_EMPTY; s_rec_on = false; s_rh = s_rt = 0; }
  }
  if (s_note_play_save) { s_note_play_save = false; Cas_Notify("TAPE IS IN PLAY MODE", "F12 > CASSETTE: EJECT OR RECORD NEW TAPE"); }
  if (s_mode == CAS_PLAY && s_is_wav && WTryLock())
  {
    Wav_Fill(64u * 1024u);
    if (s_an_todo && s_weof) { s_an_todo = false; Wav_Analyse(); }
    WUnlock();
  }
  if (s_mode == CAS_RECORD && s_rfd >= 0)
  {
    Rec_Flush(32u * 1024u);
#ifdef ESP_PLATFORM
    const uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (s_rec_fmt == CAS_FMT_WAV && ms - s_rlast_hdr_ms > 2000u) { s_rlast_hdr_ms = ms; Rec_WriteHeader(); }   // fichier toujours lisible
#endif
  }
}

// ---------------------------------------------------------------- menu
bool Cas_Insert(const char *path)
{
  Cas_Eject();
  if (!Buffers()) return false;
  char full[260]; Path(full, sizeof full, path);
  if (HasExt(path, ".WAV"))
  {
    Sd_Lock(); s_wfd = open(full, O_RDONLY); Sd_Unlock();
    if (s_wfd < 0) return false;
    if (!Wav_Header(s_wfd)) { Close(s_wfd); return false; }
    s_is_wav = true;                                          // remplissage : Cas_Rewind (une seule fois)
  }
  else
  {
    Sd_Lock(); const int fd = open(full, O_RDONLY); Sd_Unlock();
    if (fd < 0) return false;
    Sd_Lock(); const long sz = lseek(fd, 0, SEEK_END); lseek(fd, 0, SEEK_SET); Sd_Unlock();
    if (sz <= 0 || sz > (long)CAS_MAX) { int f2 = fd; Close(f2); return false; }
    s_cas = (uint8_t *)Cas_Mem((size_t)sz);
    if (!s_cas) { int f2 = fd; Close(f2); return false; }
    const int got = Rd(fd, s_cas, (int)sz);
    { int f2 = fd; Close(f2); }
    if (got != (int)sz) { Cas_Free(s_cas); s_cas = nullptr; return false; }
    s_cas_len = (uint32_t)sz; s_is_wav = false;
  }
  snprintf(s_path, sizeof s_path, "%s", path);
  s_mode = CAS_PLAY;
  Cas_Rewind();
  return true;
}
void Cas_Eject(void)
{
  Turbo_Off();
  if (s_mode == CAS_RECORD) Rec_Close();
  Free_Play();
  s_mode = CAS_EMPTY; s_rec_on = false; s_path[0] = 0;
  s_rh = s_rt = 0; s_pos = 0; Dec_All();
}
void Cas_Rewind(void)
{
  if (s_mode != CAS_PLAY) return;
  s_pos = 0; s_bit = 0; s_bit_t0 = 0; s_cas_end = false;
  if (s_cas && s_cas_len) s_bit_len = (s_cas[0] & 1) ? T_ONE : T_ZERO;
  if (s_is_wav) { WLock(); Wav_ResetRing(); Wav_Fill(512u * 1024u); s_an_todo = true; s_an_done = false; WUnlock(); }
}
bool Cas_RecordNew(void)
{
  Cas_Eject();
  if (!Buffers()) return false;
  s_rec_fmt = s_fmt;
  if (!Rec_Open()) return false;
  s_mode = CAS_RECORD; s_rec_on = (s_rec_fmt == CAS_FMT_WAV);  // CAS : la file reçoit dès le calage sur l'amorce
  s_rnext_q8 = s_pos << 8;
  return true;
}
uint8_t Cas_Mode(void) { return s_mode; }
const char *Cas_Name(void) { return s_path; }
bool Cas_GetFast(void) { return s_fast; }
void Cas_SetFast(bool on) { s_fast = on; if (!on) Turbo_Off(); }
uint8_t Cas_GetFormat(void) { return s_fmt; }
void Cas_SetFormat(uint8_t f) { (void)f; s_fmt = CAS_FMT_CAS; }   // enregistrement toujours en .CAS (le .WAV se lit seulement)
void Cas_Position(char *b, size_t n)
{
  const double sec = (double)s_pos / TICKS_HZ;
  if (s_mode == CAS_PLAY && !s_is_wav)
    snprintf(b, n, "%u %% (%.1f s)%s", s_cas_len ? (unsigned)((uint64_t)((s_bit / 8u) < s_cas_len ? (s_bit / 8u) : s_cas_len) * 100u / s_cas_len) : 0u, sec, s_cas_end ? " END" : "");
  else if (s_mode == CAS_PLAY)
  {
    const uint32_t s = (uint32_t)((s_pos * (uint64_t)s_wrate) / TICKS_HZ);
    snprintf(b, n, "%u %% (%.1f s)%s", s_wtotal ? (unsigned)((uint64_t)(s > s_wtotal ? s_wtotal : s) * 100u / s_wtotal) : 0u, sec, s >= s_wtotal ? " END" : "");
  }
  else if (s_mode == CAS_RECORD) snprintf(b, n, "%.1f s, %lu bytes", sec, (unsigned long)(s_rbytes + ((s_rh - s_rt) & (RR_BYTES - 1u))));
  else snprintf(b, n, "-");
}
void Cas_Report(char *b, size_t n)
{
  char pos[48]; Cas_Position(pos, sizeof pos);
  snprintf(b, n, "[CAS] %s %s ; position %s ; moteur %s ; chargement rapide %s ; retards du .WAV %lu, octets perdus %lu",
           s_mode == CAS_PLAY ? "lecture" : (s_mode == CAS_RECORD ? "enregistrement" : "aucune bande"), s_path, pos,
           s_motor ? "EN MARCHE" : "arrete", s_fast ? "oui" : "non",
           (unsigned long)s_underruns, (unsigned long)s_rec_drop);
  if (s_rd_n)
  {
    const size_t l = strlen(b);
    snprintf(b + l, n - l, " ; lectures $FF20 %lu, ecart min/moy/max %lu/%lu/%lu tics",
             (unsigned long)s_rd_n, (unsigned long)s_rd_min, (unsigned long)(s_rd_sum / s_rd_n), (unsigned long)s_rd_max);
  }
  if (s_mode == CAS_PLAY && s_is_wav)
  {
    const size_t l = strlen(b);
    if (s_an_done) snprintf(b + l, n - l, " ; .WAV %lu Hz x%lu : %u bloc(s) reconnu(s), %u somme(s) fausse(s)",
                            (unsigned long)s_wrate, (unsigned long)s_wup, (unsigned)s_an_blk, (unsigned)s_an_bad);
    else snprintf(b + l, n - l, " ; .WAV %lu Hz x%lu : analyse en attente", (unsigned long)s_wrate, (unsigned long)s_wup);
  }
}
