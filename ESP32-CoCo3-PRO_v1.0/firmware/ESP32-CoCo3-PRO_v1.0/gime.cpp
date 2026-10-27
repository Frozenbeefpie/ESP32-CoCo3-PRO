// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// GIME : MMU et cartographie mémoire (voir gime.h).

#include "gime.h"
#include <stdlib.h>
#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#endif
#ifdef ESP_PLATFORM
#include "esp_attr.h"   // IRAM_ATTR
#else
#define IRAM_ATTR
#endif
#ifndef DRAM_ATTR
#define DRAM_ATTR
#endif
#pragma GCC optimize ("no-jump-tables")   // tables de sauts en flash évitées (code en IRAM)
#include <string.h>
#include <stddef.h>

gime_t        gime;
gime_raster_t gime_raster;
uint8_t      *gime_rd[9];
uint8_t       gime_timer_adjust = 2;   // 2 = GIME 1986, 1 = GIME 1987
void (*gime_field_sync_cb)(void) = nullptr;
void (*gime_field_sync_rise_cb)(void) = nullptr;   // front montant de FS
void (*gime_hsync_cb)(void)      = nullptr;

// Journal vidéo par ligne : 2 trames (en cours d'écriture / publiée).
static gime_vframe_t     s_vf[2];
static uint8_t           s_vf_w = 0;          // trame en cours d'écriture
static volatile uint32_t s_vf_seq = 0;        // impair = publication en cours
static volatile bool     s_vf_valid = false;
volatile uint32_t gime_vlog_published_frame = 0;   // n° de la dernière trame publiée
static uint8_t           s_ext_lines = 0;    // état courant de EI0-EI2

static IRAM_ATTR void vlog_start_frame(void)
{
  gime_vframe_t &f = s_vf[s_vf_w];
  memcpy(f.reg, gime.reg, sizeof(f.reg));
  memcpy(f.palette, gime.palette, sizeof(f.palette));
  f.n        = 0;
  f.overflow = false;
  f.frame    = gime_raster.frame;
}

static IRAM_ATTR void vlog_publish(void)
{
  s_vf_seq++;                 // impair : le lecteur doit recommencer
  __sync_synchronize();       // (2 coeurs : ordre des écritures garanti)
  s_vf_w ^= 1;
  s_vf_valid = true;
  gime_vlog_published_frame = s_vf[s_vf_w ^ 1].frame;
  __sync_synchronize();
  s_vf_seq++;                 // pair : stable
  vlog_start_frame();         // nouvelle trame dans l'autre tampon
}

static inline IRAM_ATTR void vlog_note_at(uint8_t idx, uint8_t val, uint16_t line)
{
  gime_vframe_t &f = s_vf[s_vf_w];
  if (f.n < GIME_VLOG_MAX)
  {
    gime_vlog_entry_t &e = f.e[f.n++];
    e.line = line;
    e.idx  = idx;
    e.val  = val;
  }
  else
  {
    f.overflow = true;
  }
}
static inline IRAM_ATTR void vlog_note(uint8_t idx, uint8_t val)
{
  vlog_note_at(idx, val, gime_raster.line);
}

// registres vidéo « de la ligne ». Le GIME prend le mode ($FF98), la résolution ($FF99) et le décalage
// horizontal ($FF9F) au début de chaque ligne
uint8_t gime_line_reg[16];                 // lus par la capture des lignes (gime_video.cpp)
static inline IRAM_ATTR uint32_t lr_latch_tick(void)
{
  return ((gime_line_reg[GIME_VRES] >> 2) & 1u) ? 250u : 380u;   // HRES bit 0 : 40 / 32 octets
}
#define GIME_ACTIVE_START lr_latch_tick()
bool    gime_hs_done = false;       // synchro horizontale de la ligne en cours déjà passée (gime.h)
static uint8_t s_lr_pend = 0;              // bit 0 : $FF98, bit 1 : $FF99, bit 2 : $FF9F
static uint8_t s_lr_val[3];
static inline IRAM_ATTR int lr_slot(uint8_t idx)
{
  return idx == GIME_VMODE ? 0 : (idx == GIME_VRES ? 1 : (idx == GIME_HOFS ? 2 : -1));
}
static inline IRAM_ATTR void lr_apply_pending(void)
{
  if (!s_lr_pend) return;
  if (s_lr_pend & 1) gime_line_reg[GIME_VMODE] = s_lr_val[0];
  if (s_lr_pend & 2) gime_line_reg[GIME_VRES]  = s_lr_val[1];
  if (s_lr_pend & 4) gime_line_reg[GIME_HOFS]  = s_lr_val[2];
  s_lr_pend = 0;
}
void gime_line_reg_sync(void)
{
  memcpy(gime_line_reg, gime.reg, sizeof(gime_line_reg));
  s_lr_pend = 0;
}

bool gime_vlog_get_frame(gime_vframe_t *dst)
{
  for (int tries = 0; tries < 4; tries++)
  {
    const uint32_t s0 = s_vf_seq;
    if (!s_vf_valid) return false;
    if (s0 & 1) continue;
    const gime_vframe_t &src = s_vf[s_vf_w ^ 1];
    memcpy(dst, &src, offsetof(gime_vframe_t, e));
    const uint16_t n = (src.n <= GIME_VLOG_MAX) ? src.n : GIME_VLOG_MAX;
    memcpy(dst->e, src.e, n * sizeof(gime_vlog_entry_t));
    dst->n = n;
    __sync_synchronize();
    if (s_vf_seq == s0) return true;   // pas de publication pendant la copie
  }
  return false;
}
uint8_t *gime_wr[9];

static uint8_t       *s_ram     = nullptr;   // 512 Ko physiques (+ garde)
static const uint8_t *s_rom_int = nullptr;   // 32 Ko internes ($8000-$FFFF)
static const uint8_t *s_cart    = nullptr;   // 16 Ko cartouche
static uint32_t       s_cart_mask = GIME_CART_ROM_SIZE - 1;   // fenêtre de la ROM (0x3FFF = banque de 16 Ko en miroir)
static const uint8_t *s_cart_b8[2] = { nullptr, nullptr };   // banques de 8 Ko ($C000 / $E000), nullptr = inutilisées
// Destination des écritures en ROM (ignorées) et des lectures sans ROM (cartouche absente) : un bloc
// complet pour qu'aucun index ne déborde
static uint8_t       *s_sink = nullptr;
static uint8_t       *s_open_bus = nullptr;
static void gime_alloc_pages(void)
{
  if (s_sink) return;
#ifdef ESP_PLATFORM
  s_sink     = (uint8_t *)heap_caps_malloc(GIME_BLOCK_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  s_open_bus = (uint8_t *)heap_caps_malloc(GIME_BLOCK_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#endif
  if (!s_sink)     s_sink     = (uint8_t *)malloc(GIME_BLOCK_SIZE);
  if (!s_open_bus) s_open_bus = (uint8_t *)malloc(GIME_BLOCK_SIZE);
  memset(s_open_bus, 0xFF, GIME_BLOCK_SIZE);
}

static uint32_t s_ram_size = GIME_RAM_SIZE;

void gime_set_ram_size(uint32_t size)
{
  s_ram_size = size;
  gime_update_memory();
}
uint32_t gime_ram_size(void) { return s_ram_size; }

void gime_init(uint8_t *ram, const uint8_t *rom_int, const uint8_t *cart)
{
  s_ram     = ram;
  s_rom_int = rom_int;
  s_cart    = cart;
  gime_alloc_pages();
  gime_reset();
}

void gime_reset(void)
{
  // Tepolt (cité par MAME) : tous les registres GIME sont à 0 à l'allumage. INIT0 = 0 -> MMU inactive,
  // mode ROM "16K interne + 16K externe".
  memset(gime.reg, 0, sizeof(gime.reg));
  gime_line_reg_sync();
  for (int i = 0; i < 8; i++)
  {
    gime.mmu[i] = gime.mmu[i + 8] = (uint8_t)(0x38 + i);
    gime.mmu_ext[i] = gime.mmu_ext[i + 8] = 0;
  }
  memset(gime.palette, 0, sizeof(gime.palette));
  gime.ty           = false;   // SAM remis à zéro : mode ROM
  gime.irq_pending  = 0;
  gime.firq_pending = 0;
  s_ext_lines       = 0;
  memset(&gime_raster, 0, sizeof(gime_raster));   // timer arrêté (valeur 0)
  s_vf_valid = false;
  vlog_start_frame();
  gime_update_memory();
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
IRAM_ATTR void gime_signal(uint8_t source)
{
  if (gime.reg[GIME_IRQENR]  & source) gime.irq_pending  |= source;
  if (gime.reg[GIME_FIRQENR] & source) gime.firq_pending |= source;
}

static bool s_cart_clock = false;   // CART = horloge Q
void gime_set_cart_clock(bool on)
{
  s_cart_clock = on;
  if (on) gime_signal(GIME_INT_EI0);
}

void gime_set_ext_line(uint8_t source, bool active)
{
  source &= (GIME_INT_EI0 | GIME_INT_EI1 | GIME_INT_EI2);
  const uint8_t old = s_ext_lines;
  if (active) s_ext_lines |= source; else s_ext_lines &= ~source;
  const uint8_t rising = s_ext_lines & ~old;
  if (rising) gime_signal(rising);
}

// Écriture de $FF94 (ou changement de TINS) : le timer repart de la valeur programmée. la valeur 0
// n'arrête le timer que sur le GIME 1987. Le GIME 1986 la traite comme les autres (0 + 2 = 2
// impulsions)
static inline IRAM_ATTR uint32_t timer_period(void)
{
  const uint32_t v = ((uint32_t)(gime.reg[GIME_TMR_MSB] & 0x0F) << 8) | gime.reg[GIME_TMR_LSB];
  return (v || GIME_TIMER_ADJUST == 2u) ? (v + GIME_TIMER_ADJUST) : 0;
}
static IRAM_ATTR void timer_restart(void)
{
  gime_raster.timer_count = timer_period();
  gime_raster.timer_ticks = 0;
}

// Décompte de n impulsions du timer ; à 0 : interruption + rechargement. changement d'option 1986/1987
// (menu F12, config)
void gime_set_1987(bool on)
{
  gime_timer_adjust = on ? 1u : 2u;
  if (!gime_raster.timer_count) timer_restart();
}

// diagnostic des FIRQ du timer (Pop Star : une FIRQ toutes les 3 lignes transition du bas décalée de 3
// lignes d'une image à l'autre). Commande G. échéances fusionnées
uint32_t gime_dbg_tmr_merged = 0, gime_dbg_tmr_lat_max = 0, gime_dbg_tmr_acks = 0;
uint16_t gime_dbg_tmr_pc = 0;
uint8_t  gime_dbg_tmr_cc = 0;
static bool     s_tmr_wait = false;
static uint64_t s_tmr_set_ticks = 0;
static uint16_t s_tmr_set_pc = 0;
static uint8_t  s_tmr_set_cc = 0;
extern uint16_t gime_dbg_cpu_pc(void);
extern uint8_t  gime_dbg_cpu_cc(void);
static inline IRAM_ATTR uint64_t gime_dbg_now(void)
{
  return ((uint64_t)gime_raster.frame * GIME_LINES_PER_FRAME + gime_raster.line) * GIME_TICKS_PER_LINE + gime_raster.line_ticks;
}
static inline IRAM_ATTR void gime_dbg_tmr_expired(uint32_t k)
{
  if (!(gime.reg[GIME_FIRQENR] & GIME_INT_TMR)) return;
  if (gime.firq_pending & GIME_INT_TMR) { gime_dbg_tmr_merged += k; return; }
  gime_dbg_tmr_merged += k - 1;
  s_tmr_wait = true;
  s_tmr_set_ticks = gime_dbg_now();
  s_tmr_set_pc = gime_dbg_cpu_pc();
  s_tmr_set_cc = gime_dbg_cpu_cc();
}

static inline IRAM_ATTR void timer_count_down(uint32_t n)
{
  if (!n || !gime_raster.timer_count) return;
  if (n < gime_raster.timer_count)
  {
    gime_raster.timer_count -= n;
    return;
  }
  // Une ou plusieurs échéances dans ces n impulsions. calcul direct (période 2 possible avec le timer
  // à 0 sur un GIME 1986 : plus de boucle par échéance). L'interruption est mémorisée
  n -= gime_raster.timer_count;
  uint32_t k = 1;
  const uint32_t p = timer_period();
  if (p)
  {
    k += n / p;
    gime_raster.timer_count = p - (n % p);
  }
  else gime_raster.timer_count = 0;
  gime_raster.n_tmr += k;
  if (k & 1u) gime_raster.blink = !gime_raster.blink;   // cadence du clignotement du texte
  gime_dbg_tmr_expired(k);                              // avant gime_signal (état du drapeau)
  gime_signal(GIME_INT_TMR);
}

static IRAM_ATTR void end_of_line(void)
{
  gime_raster.n_hbord++;
  gime_signal(GIME_INT_HBORD);
  if (gime_hsync_cb) gime_hsync_cb();
  lr_apply_pending();                      // écritures de la partie visible -> ligne suivante
  // impulsion du timer TINS=0 déplacée au front de HS (gime_tick)
  if (++gime_raster.line >= GIME_LINES_PER_FRAME)
  {
    gime_raster.line = 0;
    gime_raster.frame++;
  }
  if (gime_raster.line == GIME_VBORD_LINE)
  {
    gime_raster.n_vbord++;
    gime_signal(GIME_INT_VBORD);
    vlog_publish();
  }
  // FS (PIA0 CB1) n'est PAS au début de la bordure du bas comme VBORD, mais après elle (MAME gime.cpp
  // : « the GIME trailing edges field sync after the bottom border »)
  {
    static const uint8_t DRAM_ATTR k_bb[4] = { 28, 23, 28, 12 };   // $FF99 bits 6-5 : 192, 200, (infini), 225 ; XRoar (263 lignes)
    const uint16_t fs_fall = k_bb[(gime.reg[GIME_VRES] >> 5) & 3];
    if (gime_raster.line == fs_fall)
    {
      if (gime_field_sync_cb) gime_field_sync_cb();
    }
    else if (gime_raster.line == fs_fall + 6u)
    {
      if (gime_field_sync_rise_cb) gime_field_sync_rise_cb();
    }
  }
}

IRAM_ATTR void gime_tick(uint32_t cpu_cycles, bool fast)
{
  const uint32_t t = cpu_cycles << (fast ? 3 : 4);   // x8 ou x16 ticks maîtres

  if (gime.reg[GIME_INIT1] & 0x20)                    // TINS=1 : 279 ns
  {
    gime_raster.timer_ticks += t;
    const uint32_t n = gime_raster.timer_ticks >> 2;
    gime_raster.timer_ticks &= 3;
    if (n) timer_count_down(n);
  }

  // deux évènements par ligne : la synchro horizontale (impulsion du timer en TINS=0, comme XRoar : au
  // front de HS, gime_hs_tick après HBORD) puis la fin de ligne (HBORD). ~87 ticks trop tôt
  uint32_t rem = t;
  for (;;)
  {
    const uint32_t lt = gime_raster.line_ticks;
    if (!gime_hs_done)
    {
      const uint32_t hs = gime_hs_tick();
      if (lt + rem < hs) { gime_raster.line_ticks = lt + rem; return; }
      if (lt < hs) { rem -= hs - lt; gime_raster.line_ticks = hs; }
      gime_hs_done = true;
      if (!(gime.reg[GIME_INIT1] & 0x20)) timer_count_down(1);   // TINS=0 : horloge = HSYNC
      continue;
    }
    if (lt + rem < GIME_TICKS_PER_LINE) { gime_raster.line_ticks = lt + rem; return; }
    rem -= GIME_TICKS_PER_LINE - lt;
    gime_raster.line_ticks = 0;
    gime_hs_done = false;
    end_of_line();
  }
}
bool gime_hs_pending(void)
{
  return !gime_hs_done && gime_raster.line_ticks < gime_hs_tick();
}

// Portage direct de gime_device::update_memory(int bank) (MAME).
static void update_slot(int slot_index)
{
  int      slot   = slot_index;
  uint32_t offset = 0;
  bool     force_ram  = false;
  bool     enable_mmu = (gime.reg[GIME_INIT0] & GIME_INIT0_MMUEN) != 0;

  if (slot_index == 8)          // page $FE00-$FEFF
  {
    slot       = 7;
    offset     = 0x1E00;
    force_ram  = (gime.reg[GIME_INIT0] & GIME_INIT0_MC3) != 0;
    enable_mmu = enable_mmu && !force_ram;   // MC3 : toujours bloc $3F
  }

  // numéro de bloc sur 10 bits (MMU 8 bits + $FF9B latché), la RAM physique se répétant modulo sa
  // taille (512 Ko : seuls les 6 bits du GIME d'origine comptent). La ROM reste décidée sur les 6 bits
  // du GIME
  uint32_t block;
  if (enable_mmu)
  {
    int task = (gime.reg[GIME_INIT1] & 0x01) ? 8 : 0;
    block = gime.mmu[slot + task] | ((uint32_t)gime.mmu_ext[slot + task] << 8);
  }
  else
  {
    block = (uint32_t)(slot + 0x38);
  }

  const uint8_t *rd;
  uint8_t       *wr;
  if ((block & 0x3F) >= 0x3C && !gime.ty && !force_ram)
  {
    // En ROM. La page ROM est liée à la TRANCHE logique (slot & 3), pas au bloc : c'est le
    // comportement du GIME réel (et de MAME).
    const uint8_t rom_mode = gime.reg[GIME_INIT0] & GIME_INIT0_MCMASK;
    if (rom_mode == 3 || (rom_mode < 2 && (block & 0x3F) >= 0x3E))
    {
      if (s_cart)
      {
        const uint32_t coff = (((slot & 3) ^ 2) * GIME_BLOCK_SIZE) & s_cart_mask;   // 16 Ko (banques) ou 32 Ko
        rd = s_cart_b8[0] ? s_cart_b8[(coff >> 13) & 1] : (s_cart + coff);           // 2 fenêtres de 8 Ko (CoCo PSG)
      }
      else
      {
        rd = s_open_bus;
      }
    }
    else
    {
      rd = s_rom_int + (slot & 3) * GIME_BLOCK_SIZE;
    }
    wr = s_sink;
  }
  else
  {
    uint8_t *mem = s_ram + ((block * GIME_BLOCK_SIZE) & (s_ram_size - 1));
    rd = mem;
    wr = mem;
  }

  // Les pointeurs de lecture ROM sont const ; la table est commune lecture/écriture pour la vitesse,
  // on ne réécrit jamais via gime_rd[].
  gime_rd[slot_index] = (uint8_t *)(rd + offset);
  gime_wr[slot_index] = (wr == s_sink) ? s_sink : (wr + offset);
}

void gime_update_memory(void)
{
  gime_alloc_pages();
  for (int i = 0; i <= 8; i++)
  {
    update_slot(i);
  }
}

// Multi-Pak : la ROM de cartouche visible (slot choisi par CTS, $FF7F)
void gime_set_cart(const uint8_t *cart, uint32_t mask)
{
  if (s_cart == cart && s_cart_mask == mask && !s_cart_b8[0]) return;
  s_cart = cart;
  s_cart_mask = mask;
  s_cart_b8[0] = s_cart_b8[1] = nullptr;
  gime_update_memory();
}
void gime_set_cart_banks8(const uint8_t *lo, const uint8_t *hi)
{
  if (s_cart_b8[0] == lo && s_cart_b8[1] == hi) return;
  s_cart_b8[0] = lo; s_cart_b8[1] = hi;
  s_cart_mask = 0x3FFF;                              // 16 Ko vus en miroir dans le mode 32 Ko externe
  gime_update_memory();
}

void gime_set_ty(bool all_ram)
{
  if (gime.ty != all_ram)
  {
    gime.ty = all_ram;
    gime_update_memory();
  }
}

IRAM_ATTR bool gime_write_register(uint16_t address, uint8_t value)
{
  if (address < 0xFF90 || address > 0xFFBF)
  {
    return false;
  }

  if (address <= 0xFF9F)
  {
    const uint8_t idx    = address & 0x0F;
    const uint8_t xorval = gime.reg[idx] ^ value;
    gime.reg[idx] = value;
    if (idx >= GIME_VMODE && xorval)   // $FF98-$FF9F : registres vidéo
    {
      const int sl = lr_slot(idx);
      if (sl >= 0 && gime_raster.line_ticks >= GIME_ACTIVE_START)   // pendant la partie visible -> ligne suivante
      {
        s_lr_pend |= (uint8_t)(1u << sl);
        s_lr_val[sl] = value;
        const uint16_t nl = (uint16_t)(gime_raster.line + 1u);
        vlog_note_at(idx, value, nl >= GIME_LINES_PER_FRAME ? 0 : nl);
      }
      else
      {
        gime_line_reg[idx] = value;
        if (sl >= 0) s_lr_pend &= (uint8_t)~(1u << sl);
        vlog_note(idx, value);
      }
    }
    switch (idx)
    {
      case GIME_INIT0:
        if (xorval & (GIME_INIT0_MMUEN | GIME_INIT0_MC3 | GIME_INIT0_MCMASK))
        {
          gime_update_memory();
        }
        break;
      case GIME_INIT1:
        if (xorval & 0x01)   // TR : changement de tâche
        {
          gime_update_memory();
        }
        if (xorval & 0x20)   // TINS : MAME relance le timer (valeur d'origine)
        {
          timer_restart();
        }
        break;
      case GIME_TMR_MSB:     // $FF94 : relance le timer
        timer_restart();
        break;
      case GIME_IRQENR:
        gime.irq_pending &= value;    // désactiver une source l'acquitte (MAME)
        if (s_cart_clock) gime_signal(GIME_INT_EI0);
        break;
      case GIME_FIRQENR:
        gime.firq_pending &= value;
        if (s_cart_clock) gime_signal(GIME_INT_EI0);
        break;
      default:                       // $FF95 (sans relance), vidéo : mémorisés
        break;
    }
    return true;
  }

  if (address <= 0xFFAF)
  {
    const uint8_t idx = address & 0x0F;
    // 8 bits mémorisés (cartes 2 Mo) et $FF9B bits 5-4 latchés (cartes 4/8 Mo). En 512 Ko, le modulo
    // sur la taille ne garde que les 6 bits du GIME d'origine : comportement inchangé.
    const uint8_t ext = (gime.reg[0x0B] >> 4) & 0x03;
    if (gime.mmu[idx] != value || gime.mmu_ext[idx] != ext)
    {
      gime.mmu[idx]     = value;
      gime.mmu_ext[idx] = ext;
      // Seule la tâche active change la carte courante, mais recalculer les 9 entrées coûte moins cher
      // que de tester (écritures rares).
      gime_update_memory();
    }
    return true;
  }

  const uint8_t pi = address & 0x0F;              // $FFB0-$FFBF
  if (gime.palette[pi] != (value & 0x3F))
  {
    gime.palette[pi] = value & 0x3F;
    vlog_note(0x10 | pi, value & 0x3F);
  }
  return true;
}

IRAM_ATTR bool gime_read_register(uint16_t address, uint8_t *value)
{
  if (address < 0xFF90 || address > 0xFFBF)
  {
    return false;
  }
  if (address == 0xFF92)
  {
    // Lecture = sources IRQ actives, et acquittement (doc GIME + MAME).
    *value = gime.irq_pending;
    gime.irq_pending = 0;
    if (s_cart_clock) gime_signal(GIME_INT_EI0);   // prochain front de Q
  }
  else if (address == 0xFF93)
  {
    if (s_tmr_wait && (gime.firq_pending & GIME_INT_TMR))   // délai de service de la FIRQ
    {
      const uint64_t lat = gime_dbg_now() - s_tmr_set_ticks;
      gime_dbg_tmr_acks++;
      if (lat > gime_dbg_tmr_lat_max)
      {
        gime_dbg_tmr_lat_max = (uint32_t)lat;
        gime_dbg_tmr_pc = s_tmr_set_pc;
        gime_dbg_tmr_cc = s_tmr_set_cc;
      }
      s_tmr_wait = false;
    }
    *value = gime.firq_pending;
    gime.firq_pending = 0;
    if (s_cart_clock) gime_signal(GIME_INT_EI0);
  }
  else if (address <= 0xFF9F)
  {
    // Registres en écriture seule sur le vrai GIME ; renvoyer la valeur écrite est sans danger (aucun
    // logiciel connu n'en dépend).
    *value = gime.reg[address & 0x0F];
  }
  else if (address <= 0xFFAF)
  {
    *value = gime.mmu[address & 0x0F] & 0x3F;   // relu sur 6 bits (bits 7-6 flottants, comme MAME)
  }
  else
  {
    *value = gime.palette[address & 0x0F];
  }
  return true;
}

// Save State. RAM physique, registres, MMU, palette, balayage et timer, lignes d'interruption
// externes, version du GIME.
uint8_t *gime_ram_ptr(void) { return s_ram; }
size_t gime_state_size(void) { return sizeof(gime) + sizeof(gime_raster) + 2; }
void gime_state_save(uint8_t *p)
{
  memcpy(p, &gime, sizeof(gime)); p += sizeof(gime);
  memcpy(p, &gime_raster, sizeof(gime_raster)); p += sizeof(gime_raster);
  p[0] = s_ext_lines; p[1] = gime_timer_adjust;
}
void gime_state_load(const uint8_t *p)
{
  memcpy(&gime, p, sizeof(gime)); p += sizeof(gime);
  memcpy(&gime_raster, p, sizeof(gime_raster)); p += sizeof(gime_raster);
  s_ext_lines = p[0];
  gime_timer_adjust = (p[1] == 1u) ? 1u : 2u;
  s_vf_valid = false;
  vlog_start_frame();
  gime_update_memory();
  gime_line_reg_sync();                     // registres de la ligne = état rechargé
  gime_hs_done = gime_raster.line_ticks >= gime_hs_tick();
}

uint8_t gime_peek(uint16_t address)
{
  if (address < 0xFF00)
  {
    return gime_read_mem(address);
  }
  // FF92/FF93 exclus : leur lecture acquitte les interruptions, un simple coup d'oeil de diagnostic ne
  // doit pas avoir cet effet de bord.
  if (address >= 0xFF90 && address <= 0xFFBF && address != 0xFF92 && address != 0xFF93)
  {
    uint8_t v;
    gime_read_register(address, &v);
    return v;
  }
  return s_rom_int[address - 0x8000]; // ombres I/O + vecteurs (tableau rom[])
}

void gime_poke(uint16_t address, uint8_t value)
{
  if (address < 0xFF00)
  {
    gime_write_mem(address, value);
  }
}

uint32_t gime_legacy_video_base(void)
{
  // + banque vidéo de 512 Ko ($FF9B bits 3-0, comme MAME).
  const uint32_t bank = ((uint32_t)(gime.reg[0x0B] & 0x0F) * GIME_RAM_SIZE) & (s_ram_size - 1);
  return bank |
         ((uint32_t)(gime.reg[GIME_VOFS_HI] & 0xE0) << 11) |
         ((uint32_t)(gime.reg[GIME_VOFS_LO] & 0x3F) << 3);
}

uint32_t gime_logical_to_physical(uint16_t address)
{
  if (address >= 0xFF00)
  {
    return 0xFFFFFFFFu;
  }
  const int      slot   = (address >= 0xFE00) ? 8 : (address >> 13);
  const uint8_t *p      = gime_rd[slot];
  const uint32_t within = (slot == 8) ? (address & 0xFF) : (address & 0x1FFF);
  if (p >= s_ram && p < s_ram + s_ram_size)
  {
    return (uint32_t)(p - s_ram) + within;
  }
  return 0xFFFFFFFFu;
}
