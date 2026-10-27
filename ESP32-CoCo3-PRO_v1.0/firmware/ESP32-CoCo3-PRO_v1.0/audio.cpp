// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Sortie son horodatée (PDM), sources du CoCo et des cartouches son (voir audio.h).

#include "audio.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#include "esp_heap_caps.h"   // file du PSG en PSRAM
#else
#define IRAM_ATTR
#endif

#define Q_SIZE   8192u                               // puissance de 2
#define Q_MASK   (Q_SIZE - 1u)
#define LATENCY  (AUDIO_TICKS_HZ / 50u)              // 20 ms de retard fixe
#define MAXERR   (AUDIO_TICKS_HZ / 10u)              // > 100 ms d'écart : resynchronisation
#define STEP_Q8  ((uint32_t)(((uint64_t)AUDIO_TICKS_HZ << 8) / AUDIO_RATE))   // tics par échantillon (Q8)

struct AudioEv { uint32_t t; uint8_t v; uint8_t r; };   // v = gauche (ou mono), r = droite
#ifdef ESP_PLATFORM
static AudioEv *s_q = nullptr;                      // 64 Ko en PSRAM, alloués par Audio_Init
#else
static AudioEv s_qbuf[Q_SIZE];                      // banc (PC)
static AudioEv *s_q = s_qbuf;
#endif
static volatile uint32_t s_head = 0, s_tail = 0;     // producteur coeur 1 / consommateur coeur 0
static audio_stats_t s_st;
static volatile bool s_resync = true;
static bool s_i2s = false;
static volatile uint8_t s_level = 128;              // dernier niveau (pour le PWM)

IRAM_ATTR void Audio_Push(uint32_t t, uint8_t v) { Audio_PushLR(t, v, v); }
IRAM_ATTR void Audio_PushLR(uint32_t t, uint8_t v, uint8_t r)   // gauche / droite
{
  const uint32_t h = s_head;
  if (!s_q) return;                                                 // avant Audio_Init
  if (((h + 1u) & Q_MASK) == s_tail) { s_st.dropped++; return; }   // file pleine
  s_q[h & Q_MASK].t = t;
  s_q[h & Q_MASK].v = v;
  s_q[h & Q_MASK].r = r;
  s_level = (uint8_t)(((uint16_t)v + r) >> 1);
  __atomic_store_n(&s_head, (h + 1u) & Q_MASK, __ATOMIC_RELEASE);
  s_st.pushed++;
}

// ---------------------------------------------------------------------------
// SN76489A (Game Master Cartridge). Porté de MAME sn76496.cpp
// (license:BSD-3-Clause, copyright-holders:Nicola Salmoria) : variante
// SN76489A = LFSR 16 bits (feedback 0x10000, prises 0x04 / 0x08), sans
// inversion, horloge / 16 (= 250 kHz pour 4 MHz), volumes par pas de 2 dB.
// Différence voulue : à la remise à zéro, les 4 voies sont muettes (MAME et
// le vrai circuit démarrent au volume maximal jusqu'à ce que le logiciel
// les coupe : un bourdonnement avec un logiciel qui ignore la carte).
// ---------------------------------------------------------------------------
#define PSG_Q_SIZE 1024u
#define PSG_Q_MASK (PSG_Q_SIZE - 1u)
#define PSG_TICK_Q8 ((uint32_t)(((uint64_t)AUDIO_TICKS_HZ << 8) / 250000u))   // tics émulés par pas du PSG (Q8)
// file en PSRAM, allouée seulement si une GMC est en service (16 Ko de RAM interne en tableau statique
// -> RAM DMA de l'USB épuisée, clavier et souris non énumérés derrière le hub).
static AudioEv *s_pq = nullptr;
static volatile uint32_t s_phead = 0, s_ptail = 0;
static volatile bool s_psg_on = false;
static volatile bool s_psg_reset = false;
struct Psg
{
  uint16_t reg[8];
  uint8_t  last;
  uint16_t vol[4];
  int32_t  period[4], count[4];
  uint8_t  output[4];
  uint32_t rng;
};
static Psg s_psg;
static uint16_t s_psg_voltab[16];
static void Psg_Reset(void)
{
  if (!s_psg_voltab[0])
  {
    float out = 32767.0f / 4.0f;                    // 4 voies, chacune 1/4 de l'échelle
    for (int i = 0; i < 15; i++) { s_psg_voltab[i] = (uint16_t)out; out /= 1.258925412f; }
    s_psg_voltab[15] = 0;
  }
  memset(&s_psg, 0, sizeof(s_psg));
  for (int i = 0; i < 8; i += 2) s_psg.reg[i + 1] = 0x0F;   // muettes (voir plus haut)
  for (int i = 0; i < 4; i++) { s_psg.period[i] = (i == 3) ? 0 : 0x400; s_psg.vol[i] = 0; }
  s_psg.rng = 0x10000;
  s_psg.output[3] = s_psg.rng & 1;
}
static void Psg_Write(uint8_t data)
{
  Psg &p = s_psg;
  int r;
  if (data & 0x80) { r = (data & 0x70) >> 4; p.last = (uint8_t)r; p.reg[r] = (uint16_t)((p.reg[r] & 0x3F0) | (data & 0x0F)); }
  else r = p.last;
  const int c = r >> 1;
  switch (r)
  {
    case 0: case 2: case 4:
      if ((data & 0x80) == 0) p.reg[r] = (uint16_t)((p.reg[r] & 0x0F) | ((data & 0x3F) << 4));
      p.period[c] = p.reg[r] ? p.reg[r] : 0x400;
      if (r == 4 && (p.reg[6] & 3) == 3) p.period[3] = p.period[2] << 1;
      break;
    case 1: case 3: case 5: case 7:
      p.vol[c] = s_psg_voltab[data & 0x0F];
      if ((data & 0x80) == 0) p.reg[r] = (uint16_t)((p.reg[r] & 0x3F0) | (data & 0x0F));
      break;
    case 6:
    {
      if ((data & 0x80) == 0) p.reg[r] = (uint16_t)((p.reg[r] & 0x3F0) | (data & 0x0F));
      const int n = p.reg[6];
      p.period[3] = ((n & 3) == 3) ? (p.period[2] << 1) : (1 << (5 + (n & 3)));
      p.rng = 0x10000;
      break;
    }
  }
}
static inline uint32_t Psg_Tick(void)                // un pas à 250 kHz ; rend la somme des voies (0-32767)
{
  Psg &p = s_psg;
  for (int i = 0; i < 3; i++)
    if (--p.count[i] <= 0) { p.output[i] ^= 1; p.count[i] = p.period[i]; }
  if (--p.count[3] <= 0)
  {
    const bool t1 = (p.rng & 0x04) != 0;
    const bool t2 = ((p.rng & 0x08) != 0) && (p.reg[6] & 0x04);
    p.rng >>= 1;
    if (t1 != t2) p.rng |= 0x10000;
    p.output[3] = p.rng & 1;
    p.count[3] = p.period[3];
  }
  return (p.output[0] ? p.vol[0] : 0) + (p.output[1] ? p.vol[1] : 0) +
         (p.output[2] ? p.vol[2] : 0) + (p.output[3] ? p.vol[3] : 0);
}
static volatile bool s_paused = false;   // émulation suspendue (menu F12, superposition)
void Audio_SetPaused(bool on) { s_paused = on; }
void Audio_PsgEnable(bool on)
{
  if (on && !s_pq)
  {
#ifdef ESP_PLATFORM
    s_pq = (AudioEv *)heap_caps_malloc(PSG_Q_SIZE * sizeof(AudioEv), MALLOC_CAP_SPIRAM);
#else
    s_pq = (AudioEv *)malloc(PSG_Q_SIZE * sizeof(AudioEv));
#endif
  }
  s_psg_reset = true;
  s_psg_on = on && s_pq;
}
IRAM_ATTR void Audio_PsgWrite(uint32_t t, uint8_t v)
{
  const uint32_t h = s_phead;
  if (!s_psg_on) return;
  if (((h + 1u) & PSG_Q_MASK) == s_ptail) { s_st.dropped++; return; }
  s_pq[h & PSG_Q_MASK].t = t;
  s_pq[h & PSG_Q_MASK].v = v;
  __atomic_store_n(&s_phead, (h + 1u) & PSG_Q_MASK, __ATOMIC_RELEASE);
}

// ---------------------------------------------------------------------------
// YM2149 de la carte CoCo PSG (Ed Snider) : registre choisi en $FF5E donnée en $FF5F, horloge 1 MHz
// (bit 0 du contrôle $FF5D : 0 = 1 MHz 1 = 0,5 MHz)
// ---------------------------------------------------------------------------
struct AyEv { uint32_t t; uint8_t r, v; };
#define AY_Q_SIZE 1024u
#define AY_Q_MASK (AY_Q_SIZE - 1u)
static AyEv *s_aq = nullptr;
static volatile uint32_t s_ahead = 0, s_atail = 0;
static volatile bool s_ay_on = false, s_ay_reset = false;
static const uint16_t k_ym_lvl[32] = {   // YM2149, 0..10922 (1/3 de l'échelle par voie)
  0, 0, 51, 84, 120, 152, 186, 219, 266, 324, 383, 441, 530, 637, 743, 849,
  1010, 1213, 1417, 1622, 1930, 2311, 2691, 3070, 3645, 4373, 5105, 5837, 6937, 8279, 9611, 10922 };
struct Ay
{
  uint8_t  reg[16];
  uint16_t tper[3]; uint16_t tcnt[3]; uint8_t tout[3];
  uint8_t  ncnt, npre; uint32_t rng;
  uint32_t eper, ecnt; int8_t estep; uint8_t eattack, ehold, ealt, eholding;
  bool     half;                                     // contrôle bit 0 : horloge / 2
  uint32_t sub;                                      // pas à 125 kHz sautés (mode 0,5 MHz)
};
static Ay s_ay;
static void Ay_Reset(void)
{
  memset(&s_ay, 0, sizeof(s_ay));
  s_ay.rng = 1;
}
static void Ay_SetShape(uint8_t shape)
{
  Ay &a = s_ay;
  a.eattack = (shape & 0x04) ? 0x1F : 0x00;
  if ((shape & 0x08) == 0) { a.ehold = 1; a.ealt = a.eattack; }
  else { a.ehold = shape & 0x01; a.ealt = shape & 0x02; }
  a.estep = 0x1F; a.eholding = 0;
}
static void Ay_Write(uint8_t r, uint8_t v)
{
  Ay &a = s_ay;
  if (r == 0x10) { a.half = (v & 1) != 0; return; }  // contrôle de la carte ($FF5D)
  r &= 0x0F;
  a.reg[r] = v;
  switch (r)
  {
    case 0: case 1: a.tper[0] = (uint16_t)(a.reg[0] | ((a.reg[1] & 0x0F) << 8)); break;
    case 2: case 3: a.tper[1] = (uint16_t)(a.reg[2] | ((a.reg[3] & 0x0F) << 8)); break;
    case 4: case 5: a.tper[2] = (uint16_t)(a.reg[4] | ((a.reg[5] & 0x0F) << 8)); break;
    case 11: case 12: a.eper = (uint32_t)a.reg[11] | ((uint32_t)a.reg[12] << 8); break;
    case 13: Ay_SetShape(v); break;
    default: break;
  }
}
static inline uint32_t Ay_Tick(void)                 // un pas à 125 kHz ; somme des 3 voies (0-32766)
{
  Ay &a = s_ay;
  for (int c = 0; c < 3; c++)
  {
    const uint16_t per = a.tper[c] ? a.tper[c] : 1;
    if (++a.tcnt[c] >= per) { a.tcnt[c] = 0; a.tout[c] ^= 1; }
  }
  if (++a.ncnt >= (a.reg[6] & 0x1F))
  {
    a.ncnt = 0;
    a.npre ^= 1;
    if (!a.npre) a.rng = (a.rng >> 1) | (((a.rng & 1) ^ ((a.rng >> 3) & 1)) << 16);
  }
  if (!a.eholding)
  {
    if (++a.ecnt >= a.eper)
    {
      a.ecnt = 0;
      a.estep--;
      if (a.estep < 0)
      {
        if (a.ehold) { if (a.ealt) a.eattack ^= 0x1F; a.eholding = 1; a.estep = 0; }
        else { if (a.ealt && (a.estep & 0x20)) a.eattack ^= 0x1F; a.estep &= 0x1F; }
      }
    }
  }
  const uint8_t evol = (uint8_t)((a.estep ^ a.eattack) & 0x1F);
  const uint8_t en = a.reg[7];
  const uint8_t nout = a.rng & 1;
  uint32_t sum = 0;
  for (int c = 0; c < 3; c++)
  {
    const bool on = (a.tout[c] | ((en >> c) & 1)) & (nout | ((en >> (3 + c)) & 1));
    if (!on) continue;
    const uint8_t vr = a.reg[8 + c];
    sum += k_ym_lvl[(vr & 0x10) ? evol : ((vr & 0x0F) ? (((vr & 0x0F) << 1) | 1) : 0)];
  }
  return sum;
}
void Audio_AyEnable(bool on)
{
  if (on && !s_aq)
  {
#ifdef ESP_PLATFORM
    s_aq = (AyEv *)heap_caps_malloc(AY_Q_SIZE * sizeof(AyEv), MALLOC_CAP_SPIRAM);
#else
    s_aq = (AyEv *)malloc(AY_Q_SIZE * sizeof(AyEv));
#endif
  }
  s_ay_reset = true;
  s_ay_on = on && s_aq;
}
IRAM_ATTR void Audio_AyWrite(uint32_t t, uint8_t r, uint8_t v)
{
  if (!s_ay_on) return;
  const uint32_t h = s_ahead;
  if (((h + 1u) & AY_Q_MASK) == s_atail) { s_st.dropped++; return; }
  s_aq[h & AY_Q_MASK].t = t; s_aq[h & AY_Q_MASK].r = r; s_aq[h & AY_Q_MASK].v = v;
  __atomic_store_n(&s_ahead, (h + 1u) & AY_Q_MASK, __ATOMIC_RELEASE);
}
#define AY_TICK_Q8 ((uint32_t)(((uint64_t)AUDIO_TICKS_HZ << 8) / 125000u))
// Avance le YM2149 d'un échantillon de sortie ; rend le niveau 0-255.
static inline uint8_t Ay_Sample(void)
{
  static uint32_t acc = 0, held = 0;
  static uint8_t last8 = 0;
  acc += STEP_Q8;
  uint32_t sum = 0, k = 0;
  while (acc >= AY_TICK_Q8)
  {
    acc -= AY_TICK_Q8;
    if (!(s_ay.half && (s_ay.sub ^= 1))) held = Ay_Tick();   // 0,5 MHz : un pas sur deux (sortie tenue)
    sum += held; k++;
  }
  if (k) last8 = (uint8_t)((sum / k) * 255u / 32767u);
  return last8;
}

void Audio_Resync(void) { s_resync = true; }
const audio_stats_t *Audio_GetStats(void) { return &s_st; }
bool Audio_UsingI2S(void) { return s_i2s; }

// signaux de test (bip du menu).
static volatile uint8_t s_test = 0;
void Audio_SetTestTone(uint8_t mode) { s_test = mode; }
uint8_t Audio_GetTestTone(void) { return s_test; }
static void Audio_RenderTest(int16_t *out, size_t n)
{
  // 1 = sinus 1 kHz pleine échelle, 2 = sinus 1 kHz -20 dB, 3 = carré 1 kHz ; 7 = 10 kHz 0 dB 8 = 10
  // kHz -10 dB, 9 = 20 kHz 0 dB (2,4 échantillons par période)
  static uint32_t ph = 0;                              // phase Q16 sur une période de 48 échantillons
  static const int16_t k_sin[48] = {
    0, 4277, 8481, 12539, 16383, 19947, 23170, 25996, 28377, 30273, 31650, 32487,
    32767, 32487, 31650, 30273, 28377, 25996, 23170, 19947, 16383, 12539, 8481, 4277,
    0, -4277, -8481, -12539, -16383, -19947, -23170, -25996, -28377, -30273, -31650, -32487,
    -32767, -32487, -31650, -30273, -28377, -25996, -23170, -19947, -16383, -12539, -8481, -4277 };
  for (size_t i = 0; i < n; i++)
  {
    const uint32_t k = ph; ph = (ph + 1u) % 48u;         // 48 kHz / 48 = 1 kHz exactement
    int32_t v = 0;
    switch (s_test)
    {
      case 1: v = k_sin[k]; break;
      case 2: v = k_sin[k] / 10; break;
      case 3: v = (k < 24) ? 32767 : -32767; break;
      case 5: v = -32768; break;                         // broche au plus bas (densité PDM ~0 %)
      case 6: v = 32767; break;                          // broche au plus haut (densité ~100 %)
      case 7: case 8: case 9:                            // 10 kHz 0 dB, 10 kHz -10 dB, 20 kHz 0 dB
      {
        static float hph = 0.0f;
        const float f = (s_test == 9) ? 20000.0f : 10000.0f;
        hph += f / (float)AUDIO_RATE; if (hph >= 1.0f) hph -= 1.0f;
        const float a = (s_test == 8) ? 0.31623f * 32767.0f : 32767.0f;   // 10 dB = x 0,316
        v = (int32_t)(a * sinf(6.2831853f * hph));
        break;
      }
      default: v = 0; break;
    }
    out[i] = (int16_t)v;
  }
}

// « plancher » de la sortie
#define FLOOR_IDLE   (AUDIO_RATE / 20u)              // 50 ms sans changement
#define FLOOR_REL    3                               // remontée de d (Q8 par échantillon)
static volatile bool s_floor_on = true;
void Audio_SetFloor(bool on) { s_floor_on = on; }
// une DESCENTE après un repos était perdue. Au repos, d a rejoint le niveau (sortie à 0) ; la descente
// suivante faisait suivre d instantanément et la sortie restait à 0
struct FloorSt { int32_t d = 128 << 8; uint32_t idle = 0; uint8_t prev = 128; bool raised = false; };
__attribute__((unused)) static FloorSt s_fl_m, s_fl_l, s_fl_r;               // mono, gauche, droite (états séparés)
__attribute__((unused)) static inline int16_t Audio_Floor(FloorSt &F, uint8_t x)   // remplacé par Post_Run (gardé pour référence)
{
  int32_t &d = F.d;                                  // décalage (Q8)
  uint32_t &idle = F.idle;
  uint8_t &prev = F.prev;
  bool &raised = F.raised;                           // d remonté depuis la dernière attaque
  if (!s_floor_on)
  {
    int32_t sm = ((int32_t)x - 128) * 256;
    return (int16_t)(sm > 32767 ? 32767 : sm);
  }
  if (x != prev) { prev = x; idle = 0; } else if (idle < FLOOR_IDLE) idle++;
  const int32_t xq = (int32_t)x << 8;
  if (xq < d)
  {
    if (raised) { d = xq - (d - xq); raised = false; }   // descente après repos -> coup en miroir
    else d = xq;
  }
  else if (idle >= FLOOR_IDLE && d < xq) { d += FLOOR_REL; if (d > xq) d = xq; raised = true; }
  int32_t o = xq - d;
  if (o > 65535) o = 65535;
  return (int16_t)(o - 32768);                       // 0 -> -32768 (densité 0 %)
}

// sortie stéréo (I2S PDM sur 2 lignes : gauche GPIO 47, droite AUDIO_R_GPIO).
static volatile bool s_stereo = false;
// ---------------------------------------------------------------------------
// nouvelle chaîne de sortie (remplace le « plancher » Audio_Floor de 4t / 7a1). superpose, bruit
// métallique qui apparaît et disparaît. Causes
#define POST_N      128u
#define POST_HP_R   0.99804f        // 1 - 2*pi*15/48000
#define POST_LP_A   0.7275f         // 1 - exp(-2*pi*10000/48000)
#define POST_REL    0.99145f        // décroissance du décalage par bloc : exp(-128 / (0,4 s * 48000))
struct PostSt { float hx = 0, hy = 0, lp = 0, off = 0, need_p = 0; float prev[POST_N]; uint32_t pn = 0; bool init = false; };
static PostSt s_po_m, s_po_l, s_po_r;
static uint16_t s_lv[POST_N], s_rv[POST_N];          // niveaux Q8 (0..65280) du bloc rendu
static inline void Lv_Set(size_t i, uint32_t l, uint32_t r) { s_lv[i] = (uint16_t)l; s_rv[i] = (uint16_t)r; }
static void Post_Reset(void) { s_po_m.init = s_po_l.init = s_po_r.init = false; s_po_m.pn = s_po_l.pn = s_po_r.pn = 0; }
// q : niveaux Q8 du bloc courant ; sort le bloc PRÉCÉDENT (pas = 1 mono, 2 stéréo)
static void Post_Run(PostSt &P, const uint16_t *q, size_t n, int16_t *out, size_t stride)
{
  float cur[POST_N];
  if (!P.init) { P.init = true; P.hx = q[0]; P.hy = 0; P.lp = 0; P.off = 0; P.need_p = 0; P.pn = 0; }
  float mn = 0;
  for (size_t i = 0; i < n; i++)
  {
    const float x = (float)q[i];
    P.hy = x - P.hx + POST_HP_R * P.hy; P.hx = x;          // couplage alternatif
    P.lp += POST_LP_A * (P.hy - P.lp);                      // passe-bas
    cur[i] = P.lp;
    if (P.lp < mn) mn = P.lp;
  }
  const float need_c = -mn;                                 // décalage qui garde ce bloc >= 0
  if (!s_floor_on)
  {
    for (size_t i = 0; i < n; i++)
    {
      const float v = i < P.pn ? P.prev[i] : 0.0f;
      int32_t o = (int32_t)v; if (o > 32767) o = 32767; else if (o < -32768) o = -32768;
      out[i * stride] = (int16_t)o;
    }
  }
  else
  {
    float tgt = P.off * POST_REL - 48.0f;                   // exponentielle + petite pente : atteint vraiment 0
    if (tgt < 0) tgt = 0;
    if (tgt < P.need_p) tgt = P.need_p;
    if (tgt < need_c) tgt = need_c;
    const float a = P.off, d = (tgt - a) / (float)n;
    for (size_t i = 0; i < n; i++)
    {
      const float v = i < P.pn ? P.prev[i] : 0.0f;
      int32_t o = (int32_t)(v + a + d * (float)(i + 1));
      if (o < 0) o = 0; else if (o > 65535) o = 65535;
      out[i * stride] = (int16_t)(o - 32768);              // 0 -> densité 0 %
    }
    P.off = tgt;
  }
  P.need_p = need_c;
  memcpy(P.prev, cur, n * sizeof(float)); P.pn = (uint32_t)n;
}
static void Post_Block(int16_t *out, size_t n)
{
  if (n > POST_N) n = POST_N;
  if (s_stereo) { Post_Run(s_po_l, s_lv, n, out, 2); Post_Run(s_po_r, s_rv, n, out + 1, 2); }
  else
  {
    for (size_t i = 0; i < n; i++) s_lv[i] = (uint16_t)(((uint32_t)s_lv[i] + s_rv[i]) >> 1);
    Post_Run(s_po_m, s_lv, n, out, 1);
  }
}
// vidage complet des files (changement de base de temps : Turbo / Overclock MACHINE).
static volatile bool s_flush = false;
void Audio_Flush(void) { s_flush = true; s_resync = true; }
// flux d'échantillons 48 kHz (écran d'accueil : musique WAV, jingle du logo). Un producteur (coeur
// vidéo), un consommateur (tâche audio) ; file en PSRAM.
#define STREAM_Q 16384u
static int16_t *s_stream_q = nullptr;
static volatile uint32_t s_stream_h = 0, s_stream_t = 0;
static volatile bool s_stream_on = false;
bool Audio_StreamBegin(void)
{
  if (!s_stream_q)
  {
#ifdef ESP_PLATFORM
    s_stream_q = (int16_t *)heap_caps_malloc(STREAM_Q * sizeof(int16_t), MALLOC_CAP_SPIRAM);
#endif
    if (!s_stream_q) s_stream_q = (int16_t *)malloc(STREAM_Q * sizeof(int16_t));
    if (!s_stream_q) return false;
  }
  s_stream_h = s_stream_t = 0;
  __sync_synchronize();
  s_stream_on = true;
  return true;
}
void Audio_StreamEnd(void) { s_stream_on = false; __sync_synchronize(); s_resync = true; }
uint32_t Audio_StreamRoom(void) { return (STREAM_Q - 1u) - ((s_stream_h - s_stream_t) & (STREAM_Q - 1u)); }
void Audio_StreamWrite(const int16_t *p, uint32_t n)
{
  uint32_t h = s_stream_h;
  for (uint32_t i = 0; i < n; i++) { s_stream_q[h] = p[i]; h = (h + 1u) & (STREAM_Q - 1u); }
  __sync_synchronize();
  s_stream_h = h;
}
static void Audio_RenderStream(int16_t *out, size_t n)
{
  uint32_t t = s_stream_t;
  const uint32_t h = s_stream_h;
  for (size_t i = 0; i < n; i++)
  {
    int16_t v = 0;
    if (t != h) { v = s_stream_q[t]; t = (t + 1u) & (STREAM_Q - 1u); }
    out[i] = v;
  }
  __sync_synchronize();
  s_stream_t = t;
}

void Audio_Render(int16_t *out, size_t n, uint32_t now)
{
  if (s_stream_on && s_stream_q)
  {
    Audio_RenderStream(out, n);
    if (s_stereo) for (size_t i = n; i-- > 0;) { out[2 * i] = out[i]; out[2 * i + 1] = out[i]; }
    s_resync = true; Post_Reset(); return;
  }
  if (s_test)
  {
    Audio_RenderTest(out, n);
    if (s_stereo) for (size_t i = n; i-- > 0;) { out[2 * i] = out[i]; out[2 * i + 1] = out[i]; }   // même signal des deux côtés
    s_resync = true; Post_Reset(); return;
  }
  static uint32_t play = 0, frac = 0;                // tête de lecture (tics émulés, + fraction Q8)
  static uint8_t cur = 128, curR = 128;              // cur = gauche, curR = droite
  // émulation figée (menu F12, superposition) : le temps émulé n'avance plus. resynchronisation
  // faisaient monter et descendre en boucle la hauteur d'un son tenu (la 440 Hz du SN76489 dans le
  // menu)
  static uint32_t last_now = 0, still = 0;
  if (now == last_now) { if (still < 1000u) still++; } else { still = 0; last_now = now; }
  if (still >= 8u)
  {
    static uint32_t facc = 0;
    static uint8_t f8 = 0;
    for (size_t i = 0; i < n; i++)
    {
      uint16_t mix = 0, nmix = 1;                    // partie commune (PSG), ajoutée à chaque voie
      if (s_psg_on && !s_paused)                     // menu F12 : PSG muet (Audio_SetPaused)
      {
        facc += STEP_Q8;
        uint32_t sum = 0, k = 0;
        while (facc >= PSG_TICK_Q8) { facc -= PSG_TICK_Q8; sum += Psg_Tick(); k++; }
        if (k) f8 = (uint8_t)((sum / k) * 255u / 32767u);
        mix += f8; nmix++;
      }
      if (s_ay_on && !s_paused) { mix += Ay_Sample(); nmix++; }
      Lv_Set(i, (((uint32_t)cur + mix) << 8) / nmix, (((uint32_t)curR + mix) << 8) / nmix);
    }
    Post_Block(out, n);
    s_resync = true;                                 // à la reprise : retard nominal
    return;
  }
  if (s_flush)                                       // Turbo relâché / mode Overclock changé
  {
    s_flush = false;
    uint32_t t = s_tail; const uint32_t h = __atomic_load_n(&s_head, __ATOMIC_ACQUIRE);
    while (t != h) { cur = s_q[t].v; curR = s_q[t].r; t = (t + 1u) & Q_MASK; }
    s_tail = t;
    uint32_t pt = s_ptail; const uint32_t ph = __atomic_load_n(&s_phead, __ATOMIC_ACQUIRE);
    while (pt != ph) { if (s_psg_on) Psg_Write(s_pq[pt].v); pt = (pt + 1u) & PSG_Q_MASK; }
    s_ptail = pt;
    uint32_t at = s_atail; const uint32_t ah = __atomic_load_n(&s_ahead, __ATOMIC_ACQUIRE);
    while (at != ah) { if (s_ay_on) Ay_Write(s_aq[at].r, s_aq[at].v); at = (at + 1u) & AY_Q_MASK; }
    s_atail = at;
    s_st.flushes++;
  }
  const uint32_t target = now - LATENCY;
  int32_t err = (int32_t)(target - play);
  if (s_resync || err > (int32_t)MAXERR || err < -(int32_t)MAXERR)
  {
    // Démarrage, retour du menu, ou émulation trop rapide (Overclock MACHINE) : on repart au retard
    // nominal et on jette le passé.
    play = target; frac = 0; err = 0;
    if (!s_resync) s_st.resyncs++;
    s_resync = false;
    uint32_t t = s_tail;
    const uint32_t h = __atomic_load_n(&s_head, __ATOMIC_ACQUIRE);
    while (t != h && ((int32_t)(s_q[t].t - play) <= 0 || (int32_t)(s_q[t].t - play) > (int32_t)MAXERR))   // + horodatages d'une autre base de temps
    { cur = s_q[t].v; curR = s_q[t].r; t = (t + 1u) & Q_MASK; }
    s_tail = t;
    uint32_t pt = s_ptail;                           // écritures PSG du passé appliquées d'un coup
    const uint32_t ph = __atomic_load_n(&s_phead, __ATOMIC_ACQUIRE);
    while (s_psg_on && pt != ph && (int32_t)(s_pq[pt].t - play) <= 0) { Psg_Write(s_pq[pt].v); pt = (pt + 1u) & PSG_Q_MASK; }
    s_ptail = pt;
    uint32_t at = s_atail;                           // idem pour le YM2149
    const uint32_t ah = __atomic_load_n(&s_ahead, __ATOMIC_ACQUIRE);
    while (s_ay_on && at != ah && (int32_t)(s_aq[at].t - play) <= 0) { Ay_Write(s_aq[at].r, s_aq[at].v); at = (at + 1u) & AY_Q_MASK; }
    s_atail = at;
  }
  if (s_psg_reset) { s_psg_reset = false; Psg_Reset(); }
  if (s_ay_reset) { s_ay_reset = false; Ay_Reset(); }
  static uint32_t psg_acc = 0;                       // tics émulés en attente pour le PSG (Q8)
  // (horloges VGA / I2S / émulation légèrement différentes).
  int32_t adj = err / 64;
  const int32_t lim = (int32_t)(STEP_Q8 / 16u);
  if (adj > lim) adj = lim; else if (adj < -lim) adj = -lim;
  const uint32_t step = (uint32_t)((int32_t)STEP_Q8 + adj);
  uint32_t t = s_tail;
  const uint32_t h = __atomic_load_n(&s_head, __ATOMIC_ACQUIRE);
  uint32_t pt = s_ptail;
  const uint32_t ph = __atomic_load_n(&s_phead, __ATOMIC_ACQUIRE);
  const bool psg = s_psg_on;
  uint32_t at = s_atail;
  const uint32_t ah = __atomic_load_n(&s_ahead, __ATOMIC_ACQUIRE);
  const bool ay = s_ay_on;
  // un évènement en tête de file très loin dans le futur (autre base de temps : Turbo) bloquait toute
  // la file : le son ne revenait pas. On resynchronise au bloc suivant.
  if (t != h && (int32_t)(s_q[t].t - play) > (int32_t)(2u * MAXERR)) s_resync = true;
  if (n > POST_N) n = POST_N;
  for (size_t i = 0; i < n; i++)
  {
    const uint32_t p0 = play;
    frac += step;
    play += frac >> 8;
    frac &= 0xFF;
    // moyenne du CNA sur la durée de l'échantillon (p0 .. play)
    uint32_t accL = 0, accR = 0, tp = p0;
    while (t != h && (int32_t)(s_q[t].t - play) <= 0)
    {
      const uint32_t te = s_q[t].t;
      if ((int32_t)(te - tp) > 0) { const uint32_t dt = te - tp; accL += (uint32_t)cur * dt; accR += (uint32_t)curR * dt; tp = te; }
      cur = s_q[t].v;
      curR = s_q[t].r;
      t = (t + 1u) & Q_MASK;
    }
    { const uint32_t dt = play - tp; accL += (uint32_t)cur * dt; accR += (uint32_t)curR * dt; }
    const uint32_t len = play - p0;
    const uint32_t avL = len ? (uint32_t)(((uint64_t)accL << 8) / len) : ((uint32_t)cur << 8);
    const uint32_t avR = len ? (uint32_t)(((uint64_t)accR << 8) / len) : ((uint32_t)curR << 8);
    uint16_t mix = 0, nmix = 1;                      // CoCo + PSG (GMC) + YM2149 (CoCo PSG), moyenne ; par voie
    if (psg && !s_paused)                            // SN76489A, pas de 250 kHz moyennés sur l'échantillon
    {
      while (pt != ph && (int32_t)(s_pq[pt].t - play) <= 0) { Psg_Write(s_pq[pt].v); pt = (pt + 1u) & PSG_Q_MASK; }
      // le PSG avance d'un pas FIXE par échantillon (horloge de sortie). tenue montait et descendait
      // (vibrato) en vitesse réelle et au menu.
      psg_acc += STEP_Q8;
      uint32_t sum = 0, k = 0;
      while (psg_acc >= PSG_TICK_Q8) { psg_acc -= PSG_TICK_Q8; sum += Psg_Tick(); k++; }
      static uint8_t last8 = 0;
      if (k) last8 = (uint8_t)((sum / k) * 255u / 32767u);
      mix += last8; nmix++;
    }
    if (ay && !s_paused)                             // YM2149, pas de 125 kHz moyennés sur l'échantillon
    {
      while (at != ah && (int32_t)(s_aq[at].t - play) <= 0) { Ay_Write(s_aq[at].r, s_aq[at].v); at = (at + 1u) & AY_Q_MASK; }
      mix += Ay_Sample(); nmix++;
    }
    Lv_Set(i, (avL + ((uint32_t)mix << 8)) / nmix, (avR + ((uint32_t)mix << 8)) / nmix);
  }
  Post_Block(out, n);
  if (psg) s_ptail = pt;
  else s_ptail = ph;                                 // PSG absent : file vidée
  if (ay) s_atail = at;
  else s_atail = ah;
  // Évènements déjà « en retard » (arrivés après leur heure) : compte seulement
  if (t != h && (int32_t)(s_q[t].t - play) < -(int32_t)(AUDIO_TICKS_HZ / 1000u)) s_st.late++;
  s_tail = t;
}

#ifdef ESP_PLATFORM
#include <Arduino.h>
#include "driver/i2s_pdm.h"
#include "driver/gpio.h"     // gpio_reset_pin (voie droite rendue en mono)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern volatile uint64_t g_emu_ticks;               // main.cpp : temps émulé (14,31818 MHz)
extern volatile bool g_audio_realtime;              // main.cpp : Overclock MACHINE -> horodatage en temps réel
static i2s_chan_handle_t s_tx = nullptr;
static int s_gpio = -1;

// sortie choisie à chaud (diagnostic)
static volatile uint8_t s_mode_req = 0, s_mode = 0;
static volatile int16_t s_pwm_d = 128;
static volatile bool s_pwm_raised = false;
static volatile uint32_t s_pwm_last_ms = 0;

static bool Pdm_Create(void)
{
  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  cc.dma_desc_num = 4;
  cc.dma_frame_num = 128;                           // 4 x 128 échantillons = 10,7 ms de DMA
  if (i2s_new_channel(&cc, &s_tx, nullptr) != ESP_OK) { s_tx = nullptr; return false; }
  i2s_pdm_tx_config_t pc = {
#ifdef I2S_PDM_TX_CLK_DAC_DEFAULT_CONFIG
    .clk_cfg  = I2S_PDM_TX_CLK_DAC_DEFAULT_CONFIG(AUDIO_RATE),
    .slot_cfg = I2S_PDM_TX_SLOT_DAC_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, s_stereo ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO),
#else
    .clk_cfg  = I2S_PDM_TX_CLK_DEFAULT_CONFIG(AUDIO_RATE),
    .slot_cfg = I2S_PDM_TX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, s_stereo ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO),
#endif
    .gpio_cfg = {
      .clk  = I2S_GPIO_UNUSED,
      .dout = (gpio_num_t)s_gpio,
#if SOC_I2S_PDM_MAX_TX_LINES > 1
      .dout2 = s_stereo ? (gpio_num_t)AUDIO_R_GPIO : I2S_GPIO_UNUSED,   // voie droite
#endif
      .invert_flags = { .clk_inv = false },
    },
  };
#if SOC_I2S_HW_VERSION_2
  pc.slot_cfg.hp_en = false;                        // garder le continu (plancher à 0 %)
  if (s_stereo) pc.slot_cfg.line_mode = I2S_PDM_TX_TWO_LINE_DAC;   // gauche sur dout, droite sur dout2
#endif
  if (i2s_channel_init_pdm_tx_mode(s_tx, &pc) != ESP_OK || i2s_channel_enable(s_tx) != ESP_OK)
  {
    i2s_del_channel(s_tx);
    s_tx = nullptr;
    return false;
  }
  return true;
}

static void Pdm_Destroy(void)
{
  if (!s_tx) return;
  i2s_channel_disable(s_tx);
  i2s_del_channel(s_tx);
  s_tx = nullptr;
}

// Exécuté par la tâche audio elle-même (jamais pendant un i2s_channel_write).
static void Audio_ApplyMode(uint8_t m)
{
  const uint8_t was = s_mode;
  s_mode = 0xFF;                                     // plus d'écriture PWM pendant la bascule
  if (was == 1) ledcDetach(s_gpio);
  if (was == 0) { s_i2s = false; Pdm_Destroy(); }
  switch (m)
  {
    case 0:
      if (Pdm_Create()) { s_resync = true; s_i2s = true; }
      else m = 1;                                    // repli
      if (m == 0) break;
      // fallthrough
    case 1:
      ledcAttach(s_gpio, 40000, 8);
      s_pwm_d = s_floor_on ? s_level : 0;
      ledcWrite(s_gpio, (uint32_t)(s_level - s_pwm_d));
      break;
    case 2: pinMode(s_gpio, OUTPUT); digitalWrite(s_gpio, LOW);  break;
    default: pinMode(s_gpio, OUTPUT); digitalWrite(s_gpio, HIGH); m = 3; break;
  }
  s_mode = m;
}

void Audio_SetOutputMode(uint8_t m) { s_mode_req = m; }
uint8_t Audio_GetOutputMode(void) { return s_mode; }
bool Audio_UsingPWM(void) { return s_mode == 1; }
// PWM : même plancher, en niveaux entiers ; la remontée au repos est faite par la tâche audio
// (Pwm_IdleStep, toutes les ~3 ms).
void Audio_PwmWrite(uint8_t v)
{
  s_level = v;
  if (s_mode != 1) return;
  s_pwm_last_ms = millis();
  if (!s_floor_on) { ledcWrite(s_gpio, v); return; }
  if (v < s_pwm_d)
  {
    if (s_pwm_raised) { s_pwm_d = (int16_t)(2 * v - s_pwm_d); s_pwm_raised = false; }   // coup en miroir
    else s_pwm_d = v;
  }
  int32_t o = (int32_t)v - s_pwm_d;
  ledcWrite(s_gpio, (uint32_t)(o > 255 ? 255 : o));
}
static void Pwm_IdleStep(void)
{
  if (!s_floor_on || millis() - s_pwm_last_ms < 50u) return;
  const int16_t v = s_level;
  if (s_pwm_d < v) { s_pwm_d++; s_pwm_raised = true; const int32_t o = (int32_t)v - s_pwm_d; ledcWrite(s_gpio, (uint32_t)(o > 255 ? 255 : o)); }
}

// mono / stéréo, appliqué par la tâche audio (canal I2S recréé).
static volatile bool s_stereo_req = false;
void Audio_SetStereo(bool on) { s_stereo_req = on; }
bool Audio_GetStereo(void)    { return s_stereo_req; }
bool Audio_IsStereo(void)     { return s_stereo && s_mode == 0; }
static void Audio_ApplyStereo(void)
{
  s_i2s = false;
  Pdm_Destroy();
  s_stereo = s_stereo_req;
  if (!Pdm_Create()) { s_stereo = false; Pdm_Create(); }    // repli : mono
  if (!s_stereo) gpio_reset_pin((gpio_num_t)AUDIO_R_GPIO);  // GPIO 12 rendu
  s_i2s = (s_tx != nullptr);
  s_resync = true;
}

static void Audio_Task(void *)
{
  static int16_t buf[256];                           // 128 trames (x 2 voies en stéréo)
  for (;;)
  {
    if (s_mode_req != s_mode) Audio_ApplyMode(s_mode_req);
    if (s_mode == 0 && s_stereo_req != s_stereo) Audio_ApplyStereo();
    if (s_mode == 1) { Pwm_IdleStep(); vTaskDelay(3); continue; }   // ~0,4 s pour 128 niveaux
    if (s_mode != 0) { vTaskDelay(10); continue; }
    // Temps émulé (normal) ou réel (Overclock MACHINE : l'émulation va plus vite que le temps réel, on
    // rejoue alors les évènements au fil de l'eau).
    const uint32_t now = g_audio_realtime ? (uint32_t)((uint64_t)esp_timer_get_time() * 14318180ull / 1000000ull)
                                          : (uint32_t)g_emu_ticks;   // 32 bits bas : lecture atomique sur Xtensa
    if (!s_tx) { vTaskDelay(10); continue; }
    Audio_Render(buf, 128, now);
    size_t wr = 0;
    i2s_channel_write(s_tx, buf, (s_stereo ? 256u : 128u) * sizeof(int16_t), &wr, portMAX_DELAY);   // bloque au rythme du DMA
  }
}

bool Audio_Init(int gpio)
{
  s_gpio = gpio;
  // file principale en PSRAM (64 Ko de RAM interne / DMA rendus pour l'USB et le Wi-Fi). Consommée par
  // Audio_Task (tâche, pas une interruption).
  if (!s_q) s_q = (AudioEv *)heap_caps_malloc(Q_SIZE * sizeof(AudioEv), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!s_q) s_q = (AudioEv *)malloc(Q_SIZE * sizeof(AudioEv));
  s_stereo = s_stereo_req;                          // réglage lu dans config.cfg avant Audio_Init
  if (!Pdm_Create() && (s_stereo = false, !Pdm_Create()))
  {
    ledcAttach(gpio, 40000, 8);                     // repli : PWM LEDC direct
    ledcWrite(gpio, s_level);
    s_mode = s_mode_req = 1;
    return false;
  }
  s_i2s = true;
  xTaskCreatePinnedToCore(Audio_Task, "Audio", 3072, nullptr, 3, nullptr, 0);   // coeur 0, au-dessus du VideoCore
  return true;
}
#else
static bool s_stereo_bench = false;
void Audio_SetStereo(bool on) { s_stereo_bench = on; s_stereo = on; }   // banc : rendu stéréo testable
bool Audio_GetStereo(void)    { return s_stereo_bench; }
bool Audio_IsStereo(void)     { return false; }
bool Audio_Init(int) { return false; }
void Audio_SetOutputMode(uint8_t) {}
uint8_t Audio_GetOutputMode(void) { return 0; }
bool Audio_UsingPWM(void) { return false; }
void Audio_PwmWrite(uint8_t) {}
#endif
