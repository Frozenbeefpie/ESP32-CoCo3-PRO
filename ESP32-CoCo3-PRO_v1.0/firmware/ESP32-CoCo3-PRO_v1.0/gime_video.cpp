// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Rendu vidéo des modes CoCo 3 (voir gime_video.h).

#include "gime_video.h"
#include "gime.h"
#ifdef ESP_PLATFORM
#include "esp_attr.h"   // IRAM_ATTR
#else
#define IRAM_ATTR
#define DRAM_ATTR
#endif
#pragma GCC optimize ("no-jump-tables")   // tables de sauts en flash évitées (code en IRAM)
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// Généré : palette GIME 6 bits -> octet VGA (R bits 7-6, G bits 4-3, B bits 1-0) RGB : décodage direct
// R1R0 G1G0 B1B0 (bits 5/2, 4/1, 3/0).
static const uint8_t lut_rgb[64] = {
  0x00, 0x01, 0x08, 0x09, 0x40, 0x41, 0x48, 0x49, 0x02, 0x03, 0x0A, 0x0B, 0x42, 0x43, 0x4A, 0x4B,
  0x10, 0x11, 0x18, 0x19, 0x50, 0x51, 0x58, 0x59, 0x12, 0x13, 0x1A, 0x1B, 0x52, 0x53, 0x5A, 0x5B,
  0x80, 0x81, 0x88, 0x89, 0xC0, 0xC1, 0xC8, 0xC9, 0x82, 0x83, 0x8A, 0x8B, 0xC2, 0xC3, 0xCA, 0xCB,
  0x90, 0x91, 0x98, 0x99, 0xD0, 0xD1, 0xD8, 0xD9, 0x92, 0x93, 0x9A, 0x9B, 0xD2, 0xD3, 0xDA, 0xDB
};
// Composite : table de conversion RGB->CMP d'OS-9 Level Two (COCO3 COLORS.txt manuel Tandy) inversée
static const uint8_t lut_cmp[64] = {
  0x00, 0x51, 0x08, 0x50, 0x91, 0x48, 0x89, 0x40, 0x81, 0x41, 0x4A, 0x42, 0x01, 0x0A, 0x09, 0x52,
  0x49, 0x11, 0x18, 0x58, 0x90, 0x88, 0x88, 0x80, 0xC1, 0x82, 0x8A, 0x43, 0x02, 0x0B, 0x12, 0x1A,
  0x92, 0x19, 0x10, 0x98, 0xD8, 0xD0, 0xC8, 0xC0, 0xCA, 0xC3, 0x83, 0x4B, 0x03, 0x13, 0x1B, 0x5A,
  0xDB, 0x9A, 0x59, 0x99, 0xD9, 0xD1, 0xC9, 0xD2, 0xCB, 0xD3, 0x8B, 0x93, 0x53, 0x5B, 0x9B, 0xDA
};
// Composite avec inversion de phase ($FF98 bit 5, BPI) : teinte tournée de 180° (6 des 12 teintes),
// intensité inchangée.
static const uint8_t lut_cmp180[64] = {
  0x00, 0x40, 0x81, 0x41, 0x4A, 0x42, 0x01, 0x51, 0x08, 0x50, 0x91, 0x48, 0x89, 0x0A, 0x09, 0x52,
  0x49, 0x80, 0xC1, 0x82, 0x8A, 0x43, 0x02, 0x11, 0x18, 0x58, 0x90, 0x88, 0x88, 0x0B, 0x12, 0x1A,
  0x92, 0xC0, 0xCA, 0xC3, 0x83, 0x4B, 0x03, 0x19, 0x10, 0x98, 0xD8, 0xD0, 0xC8, 0x13, 0x1B, 0x5A,
  0xDB, 0xD2, 0xCB, 0xD3, 0x8B, 0x93, 0x53, 0x9A, 0x59, 0x99, 0xD9, 0xD1, 0xC9, 0x5B, 0x9B, 0xDA
};

static const uint8_t *s_font = nullptr;
static bool           s_rgb  = false;

// état des registres vidéo « au fil des lignes » pour la trame en cours de rendu (copie de la dernière
// trame émulée publiée par gime.cpp puis mis à jour par le journal au fur et à mesure des lignes).
static gime_vframe_t s_fr;          // trame rejouée (état de départ + journal)
static uint8_t       R[16];         // $FF90-$FF9F courants
static uint16_t      s_ei;          // prochaine entrée du journal à appliquer
static int           s_line_base;   // ligne émulée de la ligne de sortie 0

// capture de la VRAM (voir gime_video.h).
static uint8_t          *s_cap_buf[3] = { nullptr, nullptr, nullptr };
static volatile uint8_t  s_cap_ready   = 0xFF;   // dernier tampon complet
static volatile uint8_t  s_cap_reading = 0xFF;   // tampon lu par le rendu
static uint16_t          s_cap_rows[3];          // lignes actives capturées
static uint32_t          s_cap_frame[3];         // trame émulée de la capture
static bool              s_cap_enabled = true;
// État côté CPU pendant la capture d'une trame
static int               s_cw      = -1;         // tampon en remplissage
static int               s_ck      = -1;         // ligne active suivante (-1 = attente)
static int               s_cfirst  = 0;          // ligne émulée de la 1re ligne active
static int               s_crows   = 0;
static uint32_t          s_cvpos   = 0;
static uint16_t          s_clir    = 0;

// file de lignes capturées en RAM INTERNE (coeur CPU -> coeur vidéo)
struct cap_q_t { uint8_t *dst; uint8_t *lir_dst; uint16_t n; uint8_t lir; int8_t done_w; uint16_t rows; uint32_t frame; };
#define CAPQ_MAX 32u
static uint8_t          *s_q_mem   = nullptr;        // CAPQ_MAX x 256 octets (RAM interne)
static uint32_t          s_q_slots = 0;              // puissance de 2, 0 = file désactivée
static cap_q_t           s_q[CAPQ_MAX];
static volatile uint32_t s_q_head = 0, s_q_tail = 0; // producteur coeur CPU / consommateur coeur vidéo
static volatile uint8_t  s_cap_pending = 0xFF;      // tampon complet, pas encore publié (file)
// statistiques (commande W) et bascule de la file (commande Q)
static volatile uint32_t s_cst_pub = 0, s_cst_drop = 0, s_cst_wait = 0, s_cst_nobuf = 0;
static uint32_t          s_q_slots_cfg = 0;             // taille configurée (0 = pas de file)
static volatile int8_t   s_q_want = -1;                 // 1 rien, 0 couper, 1 activer (appliqué en début de trame)
void gime_video_capture_stats(uint32_t *pub, uint32_t *drop, uint32_t *wait, uint32_t *nobuf, bool *queue_on)
{ *pub = s_cst_pub; *drop = s_cst_drop; *wait = s_cst_wait; *nobuf = s_cst_nobuf; *queue_on = s_q_slots != 0; }
void gime_video_capture_queue_enable(bool on) { s_q_want = on ? 1 : 0; }

// État latché pour la trame en cours
static struct
{
  const uint8_t *ram;
  int      out_w, out_h;
  bool     graphics;      // FF98 bit 7
  bool     attributes;    // texte : FF99 bit 0
  bool     hven;          // FF9F bit 7 : écran virtuel 256 octets
  int      xres;          // octets (graphique) ou caractères (texte) par rangée
  int      bpp;           // graphique : 1, 2 ou 4 bits par pixel
  int      logical_w;     // pixels logiques par ligne
  int      body_w;        // pixels de sortie occupés par la zone active
  int      left;          // colonne de sortie du début de la zone active
  int      body_lines;    // 192, 200 ou 225
  int      top;           // première ligne de sortie de la zone active
  uint16_t lpr;           // lignes par rangée
  int      underline_line;
  uint32_t vpos;          // adresse physique de la rangée courante
  uint16_t line_in_row;
  uint32_t hoff;          // FF9F x 2
  uint32_t step;          // pas 16.16 : pixels logiques par pixel de sortie
  uint8_t  pal6[16];      // palette GIME (6 bits) courante
  uint8_t  pal[16];       // palette résolue en octets VGA
  uint8_t  border;
  bool     blink;
  const uint8_t *cap;     // image capturée (nullptr = RAM vivante)
  const uint8_t *cap_lir; // ligne dans la rangée, par ligne active
} F;

static uint8_t s_row[640];   // pixels de la rangée courante (octets VGA), largeur logique

void gime_video_init(const uint8_t *font) { s_font = font; }
void gime_video_set_monitor_rgb(bool rgb) { s_rgb = rgb; }
bool gime_video_monitor_rgb(void)         { return s_rgb; }
bool gime_video_active(void)              { return (gime.reg[GIME_INIT0] & GIME_INIT0_COCO) == 0; }

static inline uint8_t color_for(uint8_t c, uint8_t vmode)
{
  c &= 0x3F;
  if (s_rgb) return lut_rgb[c];
  return (vmode & 0x20) ? lut_cmp180[c] : lut_cmp[c];
}

uint8_t gime_video_color(uint8_t c)
{
  return color_for(c, gime.reg[GIME_VMODE]);
}

static const uint8_t DRAM_ATTR k_gfx_bytes[8] = {   // en RAM (lu à chaque ligne)
  16, 20, 32, 40, 64, 80, 128, 160 };

// Largeur logique et paramètres de rangée selon FF98/FF99 (MAME record_full_body_scanline).
static IRAM_ATTR void decode_mode(uint8_t vmode, uint8_t vres, bool &graphics, bool &attr, int &xres, int &bpp, int &logical_w)
{
  graphics = (vmode & 0x80) != 0;
  if (graphics)
  {
    xres = k_gfx_bytes[(vres >> 2) & 7];
    switch (vres & 3)
    {
      case 0:  bpp = 1; break;   // 2 couleurs, 8 pixels/octet
      case 1:  bpp = 2; break;   // 4 couleurs, 4 pixels/octet
      default: bpp = 4; break;   // 16 couleurs, 2 pixels/octet (11 = indéfini)
    }
    attr = false;
    logical_w = xres * (8 / bpp);
  }
  else
  {
    switch (vres & 0x14)
    {
      case 0x00: xres = 32; break;
      case 0x04: xres = 40; break;
      case 0x10: xres = 64; break;
      default:   xres = 80; break;
    }
    bpp  = 0;
    attr = (vres & 0x01) != 0;
    logical_w = xres * 8;
  }
  if (logical_w > 640) logical_w = 640;
}

int gime_video_wanted_width(void)
{
  bool g, a; int x, b, w;
  decode_mode(gime.reg[GIME_VMODE], gime.reg[GIME_VRES], g, a, x, b, w);
  return (w > 320) ? 640 : 320;
}

// Paramètres recalculables en cours de trame -------------------------- Horizontal : mode
// texte/graphique, octets par rangée, largeur active.
static void setup_horizontal(void)
{
  decode_mode(R[GIME_VMODE], R[GIME_VRES], F.graphics, F.attributes, F.xres, F.bpp, F.logical_w);
  const bool wide = (R[GIME_VRES] & 0x04) != 0;   // 640 (sinon 512) à l'échelle 640
  int body = wide ? 640 : 512;
  if (F.out_w < 640) body >>= 1;
  if (body > F.out_w) body = F.out_w;
  F.body_w = body;
  F.left   = (F.out_w - body) / 2;
  F.step   = (uint32_t)(((uint64_t)F.logical_w << 16) / (uint32_t)body);
}

// Lignes par rangée et ligne de soulignement (FF98 bits 2-0).
static void setup_lpr(void)
{
  static const uint16_t DRAM_ATTR k_lpr[8] = { 1, 1, 2, 8, 9, 10, 11, 0xFFFF };
  F.lpr = k_lpr[R[GIME_VMODE] & 7];
  switch (R[GIME_VMODE] & 7)            // souligné (SockMaster, via MAME)
  {
    case 3:  F.underline_line = 7; break;
    case 4:
    case 5:  F.underline_line = 8; break;
    case 6:  F.underline_line = 9; break;
    default: F.underline_line = -1; break;
  }
  if (F.line_in_row >= F.lpr) F.line_in_row = 0;
}

static void setup_hoffset(void)
{
  F.hoff = (uint32_t)(R[GIME_HOFS] & 0x7F) * 2;
  F.hven = (R[GIME_HOFS] & 0x80) != 0;
}

static void resolve_palette(void)
{
  for (int i = 0; i < 16; i++) F.pal[i] = color_for(F.pal6[i], R[GIME_VMODE]);
  F.border = color_for(R[GIME_BRDR], R[GIME_VMODE]);
}

// Applique les changements du journal survenus jusqu'à la ligne émulée « line » incluse
static void apply_log_until(int line)
{
  while (s_ei < s_fr.n && (int)s_fr.e[s_ei].line <= line)
  {
    const gime_vlog_entry_t &e = s_fr.e[s_ei++];
    if (e.idx >= 0x10)
    {
      const uint8_t i = e.idx & 0x0F;
      F.pal6[i] = e.val & 0x3F;
      F.pal[i]  = color_for(e.val, R[GIME_VMODE]);
      continue;
    }
    R[e.idx] = e.val;
    switch (e.idx)
    {
      case GIME_VMODE:               // mode, LPR, inversion de phase (BPI)
        setup_horizontal();
        setup_lpr();
        resolve_palette();
        break;
      case GIME_VRES:                // résolution horizontale / couleurs
        setup_horizontal();
        break;
      case GIME_BRDR:
        F.border = color_for(e.val, R[GIME_VMODE]);
        break;
      case GIME_HOFS:
        setup_hoffset();
        break;
      default:                       // $FF9B, $FF9C-$FF9E : trame suivante
        break;
    }
  }
}

// banque vidéo de 512 Ko ($FF9B bits 3-0, cartes 2 Mo et plus comme MAME : bank_512k = (FF9B & $0F) *
// $80000). Avec 512 Ko : toujours 0.
static inline uint32_t video_bank(uint8_t ff9b)
{
  return ((uint32_t)(ff9b & 0x0F) * GIME_RAM_SIZE) & (gime_ram_size() - 1);
}

void gime_video_begin_frame(const uint8_t *ram, int out_w, int out_h)
{
  F.ram   = ram;
  F.out_w = out_w;
  F.out_h = out_h;

  // Dernière trame émulée complète ; à défaut (démarrage, CPU arrêté avant la première trame), l'état
  // courant des registres, sans journal. journal et capture de la MÊME trame
  {
    const uint8_t r = s_cap_ready;
    if (s_cap_enabled && r < 3 && (int32_t)(s_cap_frame[r] - gime_vlog_published_frame) == 1)
    {
      for (uint32_t i = 0; i < 2000000u && (int32_t)(s_cap_frame[r] - gime_vlog_published_frame) == 1; i++)
        __asm__ __volatile__("" ::: "memory");
    }
  }
  if (!gime_vlog_get_frame(&s_fr))
  {
    memcpy(s_fr.reg, gime.reg, sizeof(s_fr.reg));
    memcpy(s_fr.palette, gime.palette, sizeof(s_fr.palette));
    s_fr.n = 0;
  }
  memcpy(R, s_fr.reg, sizeof(R));
  memcpy(F.pal6, s_fr.palette, sizeof(F.pal6));
  F.ram = ram + video_bank(R[0x0B]);           // banque vidéo
  s_ei = 0;

  // Vertical : fixé pour toute la trame.
  switch (R[GIME_VRES] & 0x60)
  {
    case 0x20: F.body_lines = 200; break;
    case 0x60: F.body_lines = 225; break;
    default:   F.body_lines = 192; break;   // 00 (et 10 = indéfini)
  }
  if (F.body_lines > out_h) F.body_lines = out_h;
  F.top = (out_h - F.body_lines) / 2;
  s_line_base = (int)GIME_LINES_PER_FRAME - F.body_lines - F.top;

  F.vpos = (((uint32_t)R[GIME_VOFS_HI] << 11) | ((uint32_t)R[GIME_VOFS_LO] << 3)) & (GIME_RAM_SIZE - 1);
  F.line_in_row = R[GIME_VSC] & 0x0F;          // défilement vertical fin

  setup_horizontal();
  setup_lpr();
  setup_hoffset();
  resolve_palette();
  F.blink  = gime_raster.blink;

  // image capturée au fil du balayage, si elle est récente et a la même hauteur que la trame rendue.
  F.cap = nullptr;
  F.cap_lir = nullptr;
  s_cap_reading = 0xFF;
  const uint8_t r = s_cap_ready;
  if (s_cap_enabled && r < 3 && s_cap_rows[r] == (uint16_t)F.body_lines &&
      (uint32_t)(gime_raster.frame - s_cap_frame[r]) <= 2 &&
      (s_fr.n == 0 || s_cap_frame[r] == s_fr.frame))   // même trame que le journal
  {
    s_cap_reading = r;          // réservé : le CPU choisit un autre tampon
    __sync_synchronize();
    F.cap     = s_cap_buf[r];
    F.cap_lir = s_cap_buf[r] + GIME_CAP_ROWS * GIME_CAP_ROW_BYTES;
  }
}

void gime_video_set_capture_queue(uint8_t *mem, uint32_t slots)
{
  uint32_t n = 1; while (n * 2 <= slots && n * 2 <= CAPQ_MAX) n *= 2;
  s_q_mem = mem; s_q_slots = (mem && slots >= 2) ? n : 0;
  s_q_slots_cfg = s_q_slots;
}

// Coeur vidéo : vide la file (copie vers les tampons de capture en PSRAM et publication des trames
// complètes). Sans effet si la file est vide.
IRAM_ATTR void gime_video_capture_drain(void)
{
  if (!s_q_slots) return;
  uint32_t t = s_q_tail;
  const uint32_t h = __atomic_load_n(&s_q_head, __ATOMIC_ACQUIRE);
  while (t != h)
  {
    const uint32_t k = t & (s_q_slots - 1);
    const cap_q_t &e = s_q[k];
    if (e.n) memcpy(e.dst, s_q_mem + k * 256u, e.n);
    if (e.lir_dst) *e.lir_dst = e.lir;
    if (e.done_w >= 0)
    {
      s_cap_rows[e.done_w]  = e.rows;
      s_cap_frame[e.done_w] = e.frame;
      __sync_synchronize();
      s_cap_ready   = (uint8_t)e.done_w;
      s_cap_pending = 0xFF;
      s_cst_pub++;
    }
    t++;
    __atomic_store_n(&s_q_tail, t, __ATOMIC_RELEASE);
  }
}

// des entrées attendent (le coeur vidéo ne compte comme travail que les vidages utiles).
IRAM_ATTR bool gime_video_capture_has_work(void)
{
  return s_q_slots && s_q_tail != __atomic_load_n(&s_q_head, __ATOMIC_ACQUIRE);
}

// Coeur CPU : réserve une entrée de la file (nullptr = pleine).
static inline cap_q_t *capq_reserve(uint32_t *slot)
{
  if (!s_q_slots) return nullptr;
  const uint32_t h = s_q_head;
  if (h - __atomic_load_n(&s_q_tail, __ATOMIC_ACQUIRE) >= s_q_slots) return nullptr;
  *slot = h & (s_q_slots - 1);
  return &s_q[*slot];
}
static inline void capq_publish(void)
{
  __atomic_store_n(&s_q_head, s_q_head + 1, __ATOMIC_RELEASE);
#if !defined(ESP_PLATFORM) && !defined(CAPQ_THREAD_TEST)
  gime_video_capture_drain();   // banc PC (un seul fil) : vidé tout de suite
#endif
}

void gime_video_set_capture_buffers(uint8_t *b0, uint8_t *b1, uint8_t *b2)
{
  s_cap_buf[0] = b0; s_cap_buf[1] = b1; s_cap_buf[2] = b2;
  if (!b0 || !b1 || !b2) s_cap_buf[0] = nullptr;
}

void gime_video_use_capture(bool on) { s_cap_enabled = on; }

static inline int body_lines_of(uint8_t vres)
{
  switch (vres & 0x60)
  {
    case 0x20: return 200;
    case 0x60: return 225;
    default:   return 192;
  }
}

// Coeur CPU, à chaque fin de ligne émulée
IRAM_ATTR void gime_video_capture_line(const uint8_t *ram, uint16_t line)
{
  if (s_cap_buf[0] == nullptr) return;
  if (line == 0 || (gime.reg[GIME_INIT0] & GIME_INIT0_COCO))   // hors zone / mode CoCo 1/2
  {
    s_ck = -1;
    return;
  }
  if (s_ck == -1)                                   // en attente du début de zone active
  {
    const int n = body_lines_of(gime_line_reg[GIME_VRES]);
    if ((int)line != (int)GIME_LINES_PER_FRAME - n) return;
    // bascule de la file demandée (commande Q), appliquée file vide
    if (s_q_want >= 0 && __atomic_load_n(&s_q_tail, __ATOMIC_ACQUIRE) == s_q_head && s_cap_pending == 0xFF)
    {
      s_q_slots = s_q_want ? s_q_slots_cfg : 0;
      s_q_want = -1;
    }
    uint8_t w = 0;
    while (w == s_cap_ready || w == s_cap_reading || w == s_cap_pending) w++;
    if (w > 2) { s_ck = -2; s_cst_nobuf++; return; }   // aucun tampon libre (rare) : trame non capturée
    s_cw     = w;
    s_crows  = n;
    s_cfirst = line;
    s_cvpos  = (((uint32_t)gime.reg[GIME_VOFS_HI] << 11) | ((uint32_t)gime.reg[GIME_VOFS_LO] << 3)) & (GIME_RAM_SIZE - 1);
    s_clir   = gime.reg[GIME_VSC] & 0x0F;
    s_ck     = 0;
  }
  if (s_ck < 0) return;                             // trame déjà complète
  if ((int)line - s_cfirst != s_ck) { s_ck = -2; return; }   // désynchronisé : abandon

  // Paramètres de la ligne (registres vivants = ceux que le rendu rejoue à cette même ligne via le
  // journal). mode, résolution et décalage horizontal pris au début de la ligne (gime_line_reg)
  bool graphics, attr; int xres, bpp, lw;
  decode_mode(gime_line_reg[GIME_VMODE], gime_line_reg[GIME_VRES], graphics, attr, xres, bpp, lw);
  const bool     hven  = (gime_line_reg[GIME_HOFS] & 0x80) != 0;
  const uint32_t hoff  = (uint32_t)(gime_line_reg[GIME_HOFS] & 0x7F) * 2;
  const uint32_t used  = graphics ? (uint32_t)xres : (uint32_t)xres * (attr ? 2u : 1u);
  const uint32_t pitch = hven ? 256u : used;
  uint32_t n = hoff + used;
  if (hven || n > 256u) n = 256u;
  static const uint16_t DRAM_ATTR k_lpr[8] = { 1, 1, 2, 8, 9, 10, 11, 0xFFFF };
  uint16_t lpr = k_lpr[gime_line_reg[GIME_VMODE] & 7];
  if (s_clir >= lpr) s_clir = 0;

  uint8_t *dst = s_cap_buf[s_cw] + (uint32_t)s_ck * GIME_CAP_ROW_BYTES;
  uint8_t *lir_dst = s_cap_buf[s_cw] + GIME_CAP_ROWS * GIME_CAP_ROW_BYTES + s_ck;
  ram += video_bank(gime.reg[0x0B]);                // banque vidéo
  const uint32_t first = GIME_RAM_SIZE - s_cvpos;   // octets avant le bouclage à $00000
  uint32_t slot;
  cap_q_t *e = capq_reserve(&slot);                 // file en RAM interne
#ifdef ESP_PLATFORM
  if (!e && s_q_slots)
  {
    // File pleine : le coeur vidéo est momentanément bloqué (vTaskDelay tâche audio...). On l'attend
    // au plus 3 ms ; l'émulation rattrape ensuite ce retard (cadence continue), l'image reste exacte.
    uint32_t c0; __asm__ __volatile__("rsr %0, ccount" : "=a"(c0));
    s_cst_wait++;
    for (;;)
    {
      e = capq_reserve(&slot);
      if (e) break;
      uint32_t c1; __asm__ __volatile__("rsr %0, ccount" : "=a"(c1));
      if (c1 - c0 > 720000u) break;               // ~3 ms à 240 MHz
    }
  }
#elif defined(CAPQ_THREAD_TEST)
  for (uint32_t i = 0; !e && s_q_slots && i < 20000000u; i++) e = capq_reserve(&slot);
#endif
  if (!e && s_q_slots)
  {
    // Toujours pleine : on abandonne la capture de cette trame plutôt que d'écrire directement
    // (l'ordre FIFO garantit qu'une entrée ancienne ne peut pas écraser une plus récente)
    s_ck = -2;
    s_cst_drop++;
    return;
  }
  uint8_t *cd = e ? s_q_mem + slot * 256u : dst;
  if (n <= first)
  {
    memcpy(cd, ram + s_cvpos, n);
  }
  else
  {
    memcpy(cd, ram + s_cvpos, first);
    memcpy(cd + first, ram, n - first);
  }
  const bool last = (s_ck + 1 == s_crows);
  if (e)
  {
    e->dst = dst; e->n = (uint16_t)n; e->lir_dst = lir_dst; e->lir = (uint8_t)s_clir;
    e->done_w = last ? (int8_t)s_cw : (int8_t)-1;
    e->rows = (uint16_t)s_crows; e->frame = gime_raster.frame;
    if (last) s_cap_pending = (uint8_t)s_cw;
  }
  else
  {
    *lir_dst = (uint8_t)s_clir;
  }

  if (++s_clir >= lpr)
  {
    s_clir  = 0;
    s_cvpos = (s_cvpos + pitch) & (GIME_RAM_SIZE - 1);
  }
  if (++s_ck == s_crows)                            // dernière ligne active
  {
    if (!e)                                         // sans file (copie directe, comme avant 4k)
    {
      s_cap_rows[s_cw]  = (uint16_t)s_crows;
      s_cap_frame[s_cw] = gime_raster.frame;
      __sync_synchronize();
      s_cap_ready = (uint8_t)s_cw;
      s_cst_pub++;
    }
    else
    {
      capq_publish();                               // marqueur de fin de trame inclus
      s_ck = -2;
      return;
    }
    s_ck = -2;
    return;
  }
  if (e) capq_publish();
}

// Décode une rangée (ligne de balayage) dans s_row ; retourne le nombre d'octets lus en mémoire vidéo
// (pas de la rangée). row : fenêtre capturée de la ligne ou nullptr = RAM vivante.
#define VBYTE(i) (row ? row[((F.hoff + (i)) & 0xFF)] : ram[(vp + ((F.hoff + (i)) & 0xFF)) & (GIME_RAM_SIZE - 1)])
static uint32_t fetch_row(const uint8_t *row, uint16_t lir_cap)
{
  const uint8_t *ram = F.ram;
  const uint32_t vp  = F.vpos;
  uint32_t off = 0;
  uint8_t *o = s_row;

  if (F.graphics)
  {
    for (int i = 0; i < F.xres; i++)
    {
      const uint8_t d = VBYTE(off);
      off++;
      switch (F.bpp)
      {
        case 1:
          for (int b = 7; b >= 0; b--) *o++ = F.pal[(d >> b) & 1];
          break;
        case 2:
          *o++ = F.pal[(d >> 6) & 3]; *o++ = F.pal[(d >> 4) & 3];
          *o++ = F.pal[(d >> 2) & 3]; *o++ = F.pal[d & 3];
          break;
        default:
          *o++ = F.pal[d >> 4]; *o++ = F.pal[d & 15];
          break;
      }
    }
  }
  else
  {
    const uint16_t lir = row ? lir_cap : F.line_in_row;
    for (int i = 0; i < F.xres; i++)
    {
      uint8_t ch = VBYTE(off);
      off++;
      uint8_t fg = F.pal[1], bg = F.pal[0];
      bool underline = false;
      if (F.attributes)
      {
        uint8_t at = VBYTE(off);
        off++;
        if ((at & 0x80) && F.blink)      // clignotant : caractère effacé
        {
          ch = 0x20;
          at &= ~0x40;
        }
        fg = F.pal[8 + ((at >> 3) & 7)];
        bg = F.pal[at & 7];
        underline = (at & 0x40) && (lir == F.underline_line);
      }
      uint8_t bits = (lir < 12 && s_font) ? s_font[(ch & 0x7F) * 12 + lir] : 0;
      if (underline) bits = 0xFF;
      for (int b = 7; b >= 0; b--) *o++ = ((bits >> b) & 1) ? fg : bg;
    }
  }
  return F.hven ? 256 : off;
}

void gime_video_render_line(int y, uint8_t *out)
{
  apply_log_until(s_line_base + y);   // effets en cours de trame

  if (y < F.top || y >= F.top + F.body_lines)
  {
    memset(out, F.border, F.out_w);
    return;
  }

  const int b = y - F.top;                         // ligne de la zone active
  const uint8_t *row = F.cap ? F.cap + (uint32_t)b * GIME_CAP_ROW_BYTES : nullptr;
  const uint32_t pitch = fetch_row(row, F.cap ? F.cap_lir[b] : 0);

  memset(out, F.border, F.left);
  uint8_t *o = out + F.left;
  if (F.step == 0x10000)
  {
    memcpy(o, s_row, F.body_w);
  }
  else
  {
    uint32_t acc = 0;
    for (int x = 0; x < F.body_w; x++, acc += F.step)
    {
      o[x] = s_row[acc >> 16];
    }
  }
  memset(o + F.body_w, F.border, F.out_w - F.left - F.body_w);

  if (++F.line_in_row >= F.lpr)
  {
    F.line_in_row = 0;
    F.vpos = (F.vpos + pitch) & (GIME_RAM_SIZE - 1);
  }
}

#ifdef CAPQ_THREAD_TEST
// Banc de test de la file : tampon publié, sa trame et ses lignes.
int gime_video_test_ready(uint32_t *frame, uint16_t *rows)
{
  const uint8_t r = s_cap_ready;
  if (r > 2) return -1;
  *frame = s_cap_frame[r]; *rows = s_cap_rows[r];
  s_cap_reading = r;
  return r;
}
void gime_video_test_release(void) { s_cap_reading = 0xFF; }
#endif
