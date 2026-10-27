// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// GIME (TCC1014) : MMU 2 tâches x 8 pages, registres $FF90-$FFBF, interruptions, timer.

#ifndef __GIME_H
#define __GIME_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Dimensions
// ---------------------------------------------------------------------------
#define GIME_RAM_SIZE        (512u * 1024u)   // 64 blocs de 8 Ko ($00-$3F) : RAM
                                             // de base, ET espace adressable par la VIDÉO (les 512 premiers
                                             // Ko, même avec l'extension 2/4 Mo, comme MAME) extension
                                             // mémoire (cartes « 2 Mo / 8 Mo »)
#define GIME_RAM_MAX         (4u * 1024u * 1024u)   // limite de la PSRAM disponible
// Garde après les 512 Ko : les rendus vidéo « legacy » (CoCo 1/2) indexent jusqu'à offset SAM max
// (127*512) + 6 Ko à partir de la base vidéo
#define GIME_RAM_GUARD       (80u * 1024u)
#define GIME_RAM_ALLOC       (GIME_RAM_SIZE + GIME_RAM_GUARD)      // 512 Ko
#define GIME_RAM_ALLOC_FOR(sz) ((uint32_t)(sz) + GIME_RAM_GUARD)
#define GIME_CART_ROM_SIZE   (32u * 1024u)    // ROM cartouche 32 Ko (disk21 = 16128 o)
#define GIME_BLOCK_SIZE      0x2000u

// ---------------------------------------------------------------------------
// Index des registres $FF90-$FF9F dans gime_t::reg[]
// ---------------------------------------------------------------------------
enum GimeReg : uint8_t
{
  GIME_INIT0   = 0x0,   // $FF90 COCO MMUEN IEN FEN MC3 MC2 MC1 MC0
  GIME_INIT1   = 0x1,   // $FF91 TINS(b5) TR(b0)
  GIME_IRQENR  = 0x2,   // $FF92
  GIME_FIRQENR = 0x3,   // $FF93
  GIME_TMR_MSB = 0x4,   // $FF94
  GIME_TMR_LSB = 0x5,   // $FF95
  GIME_VMODE   = 0x8,   // $FF98
  GIME_VRES    = 0x9,   // $FF99
  GIME_BRDR    = 0xA,   // $FF9A
  GIME_VSC     = 0xC,   // $FF9C
  GIME_VOFS_HI = 0xD,   // $FF9D
  GIME_VOFS_LO = 0xE,   // $FF9E
  GIME_HOFS    = 0xF,   // $FF9F
};

// Bits de INIT0 ($FF90)
#define GIME_INIT0_COCO   0x80
#define GIME_INIT0_MMUEN  0x40
#define GIME_INIT0_IEN    0x20
#define GIME_INIT0_FEN    0x10
#define GIME_INIT0_MC3    0x08   // $FE00-$FEFF = RAM constante (bloc $3F)
#define GIME_INIT0_MC2    0x04   // SCS standard ($FF40-$FF5F)
#define GIME_INIT0_MCMASK 0x03   // 0x=16K int+16K ext, 10=32K int, 11=32K ext

struct gime_t
{
  uint8_t reg[16];      // $FF90-$FF9F (valeurs écrites)
  uint8_t mmu[16];      // $FFA0-$FFA7 tâche 0, $FFA8-$FFAF tâche 1 (8 bits mémorisés)
                        // 6 utiles en 512 Ko, relus sur 6 bits comme le GIME)
  uint8_t mmu_ext[16];  // bits 9-8 du bloc ($FF9B bits 5-4 latchés à l'écriture)
  uint8_t palette[16];  // $FFB0-$FFBF (6 bits utiles)
  bool    ty;           // SAM TY : false = mode ROM ($FFDE), true = tout-RAM ($FFDF)
  uint8_t irq_pending;
  uint8_t firq_pending;
};

extern gime_t gime;

// Tables de pages : [0..7] = tranches logiques de 8 Ko, [8] = page $FE00-$FEFF (pointeur déjà décalé à
// l'octet $FE00 de son bloc)
extern uint8_t *gime_rd[9];
extern uint8_t *gime_wr[9];

// Initialisation : ram = GIME_RAM_ALLOC octets, rom_int = 32 Ko (image interne complète, vecteurs
// $FFF0-$FFFF inclus), cart = GIME_CART_ROM_SIZE octets (ou nullptr si aucune cartouche).
void gime_init(uint8_t *ram, const uint8_t *rom_int, const uint8_t *cart);
// taille de la RAM physique (512 Ko, 2 Mo ou 4 Mo ; puissance de 2). ram doit alors faire
// GIME_RAM_ALLOC_FOR(size) octets. Défaut : 512 Ko.
void     gime_set_ram_size(uint32_t size);
uint32_t gime_ram_size(void);
void gime_reset(void);
void gime_update_memory(void);

// Registres $FF90-$FFBF. true si l'adresse appartient au GIME (traitée).
bool gime_write_register(uint16_t address, uint8_t value);
bool gime_read_register(uint16_t address, uint8_t *value);
void gime_set_ty(bool all_ram);  // $FFDE (false) / $FFDF (true)

// Accès CPU rapides pour $0000-$FEFF (l'appelant traite $FF00-$FFFF).
static inline uint8_t gime_read_mem(uint16_t a)
{
  if (a < 0xFE00) return gime_rd[a >> 13][a & 0x1FFF];
  return gime_rd[8][a & 0x00FF];
}
static inline void gime_write_mem(uint16_t a, uint8_t v)
{
  if (a < 0xFE00) gime_wr[a >> 13][a & 0x1FFF] = v;
  else            gime_wr[8][a & 0x00FF] = v;
}

// Accès « émulateur » (sans effet de bord périphérique) à une adresse LOGIQUE du CPU, via la carte
// mémoire courante
uint8_t gime_peek(uint16_t address);
// Save State (voir gime.cpp)
#include <stddef.h>
uint8_t *gime_ram_ptr(void);
size_t   gime_state_size(void);
void     gime_state_save(uint8_t *p);
void     gime_state_load(const uint8_t *p);
void    gime_poke(uint16_t address, uint8_t value);

// Adresse physique de la base vidéo en mode compatible CoCo 1/2 (FF9D & $E0) << 11 | (FF9E & $3F) << 3
// ; le décalage SAM (F0-F6 x 512) s'y ajoute (comme MAME get_video_base en legacy_video).
uint32_t gime_legacy_video_base(void);

// Traduction logique -> physique (diagnostic ; 0xFFFFFFFF si ROM/I-O).
uint32_t gime_logical_to_physical(uint16_t address);

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Le temps est compté en « ticks maîtres » de 14,31818 MHz (quartz NTSC du CoCo3) : 1 cycle CPU = 16
// ticks à 0,89 MHz, 8 ticks à 1,79 MHz. Ligne = 912 ticks (63,69 µs)
#define GIME_TICKS_PER_LINE    912u
// 263 lignes (XRoar, mesuré sur un vrai CoCo 3, 60 Hz) : synchro 4 + 3 bordure du haut (36 / 34 / 19
// lignes en 192 / 200 / 225), zone visible VBORD, bordure du bas (28 / 23 / 12)
#define GIME_LINES_PER_FRAME   263u
#define GIME_VBORD_LINE        0u     // ligne (du compteur émulé) où VBORD est signalé
// Le timer décompte valeur+2 (GIME 1986) ou valeur+1 (GIME 1987). Le CoCo3 grand public a surtout le
// GIME 1986 ; MAME permet les deux. choix à l'exécution (menu F12, touche G) ; défaut 1986.
extern uint8_t gime_timer_adjust;               // 2 = GIME 1986, 1 = GIME 1987
extern uint8_t gime_line_reg[16];               // $FF98 / $FF99 / $FF9F pris au début de la ligne (capture)
void gime_line_reg_sync(void);
bool gime_hs_pending(void);                     // synchro horizontale pas encore passée dans cette ligne
void gime_set_1987(bool on);   // relance un timer arrêté (0 tourne en 1986)
static inline bool gime_is_1987(void) { return gime_timer_adjust == 1u; }
#define GIME_TIMER_ADJUST      ((uint32_t)gime_timer_adjust)

// Bits des sources (identiques dans $FF92 et $FF93)
#define GIME_INT_TMR    0x20
#define GIME_INT_HBORD  0x10
#define GIME_INT_VBORD  0x08
#define GIME_INT_EI2    0x04   // série (non émulé)
#define GIME_INT_EI1    0x02   // clavier (réservé)
#define GIME_INT_EI0    0x01   // cartouche (ligne CART, 7a)

struct gime_raster_t
{
  uint32_t line_ticks;     // ticks écoulés dans la ligne courante
  uint16_t line;           // 0..261
  uint32_t frame;          // compteur de trames émulées (diagnostic)
  uint32_t timer_count;    // décompte restant (0 = timer arrêté)
  uint32_t timer_ticks;    // ticks accumulés en mode TINS=1
  uint32_t n_hbord, n_vbord, n_tmr;   // compteurs de diagnostic
  bool     blink;           // bascule à chaque échéance du timer (attribut clignotant, comme MAME)
};
extern gime_raster_t gime_raster;

// Faire avancer le GIME de cpu_cycles cycles CPU. Appelé après CHAQUE instruction exécutée. fast =
// $FFD9 (1,79 MHz).
void gime_tick(uint32_t cpu_cycles, bool fast);
// chemin rapide en ligne (appelé à chaque instruction) : sans timer 279 ns et sans fin de ligne, on ne
// fait qu'avancer le compteur de ticks ; sinon gime_tick complet. Résultat identique à gime_tick
static inline __attribute__((always_inline)) uint32_t gime_hs_tick(void)
{
  return ((gime.reg[GIME_VRES] >> 2) & 1u) ? 87u : 153u;
}
extern bool gime_hs_done;
static inline __attribute__((always_inline)) void gime_tick_inline(uint32_t cpu_cycles, bool fast)
{
  if (!(gime.reg[GIME_INIT1] & 0x20))
  {
    const uint32_t lt = gime_raster.line_ticks + (cpu_cycles << (fast ? 3 : 4));
    const uint32_t lim = gime_hs_done ? GIME_TICKS_PER_LINE : gime_hs_tick();
    if (lt < lim) { gime_raster.line_ticks = lt; return; }
  }
  gime_tick(cpu_cycles, fast);
}

// Signaler une source d'interruption (front) : latchée dans irq_pending / firq_pending selon $FF92 /
// $FF93.
void gime_signal(uint8_t source);

// Entrées d'interruption externes du GIME (niveau) : EI0 = CART (cartouche) EI1 = clavier (une rangée
// PA0-PA6 du PIA0 à 0), EI2 = série
void gime_set_ext_line(uint8_t source, bool active);

// ligne CART reliée à l'horloge Q (cartouches de jeu à démarrage automatique). EI0 voit un front à
// chaque cycle : la source est relatchée dès qu'elle est acquittée (lecture de $FF92/$FF93) ou
// autorisée.
void gime_set_cart_clock(bool on);

// ROM de cartouche visible en $C000 (slot CTS du Multi-Pak). mask : 0x7FFF (32 Ko) ou 0x3FFF.
void gime_set_cart(const uint8_t *cart, uint32_t mask = GIME_CART_ROM_SIZE - 1);
// deux fenêtres de 8 Ko indépendantes ($C000-$DFFF, $E000-$FEFF) : CoCo PSG.
void gime_set_cart_banks8(const uint8_t *lo, const uint8_t *hi);

// État des sorties IRQ/FIRQ du GIME (true = demande active). INIT0 IEN/FEN sont les interrupteurs
// généraux.
static inline bool gime_irq_active(void)
{
  return (gime.reg[GIME_INIT0] & GIME_INIT0_IEN) && gime.irq_pending;
}
static inline bool gime_firq_active(void)
{
  return (gime.reg[GIME_INIT0] & GIME_INIT0_FEN) && gime.firq_pending;
}

// ---------------------------------------------------------------------------
// journal des registres vidéo par ligne (effets en cours de trame)
// ---------------------------------------------------------------------------
// Chaque écriture de $FF98-$FF9F ou de la palette $FFB0-$FFBF est notée avec la ligne du balayage
// émulé
#define GIME_VLOG_MAX 1024u   // changements notés par trame (au-delà : ignorés)

struct gime_vlog_entry_t
{
  uint16_t line;    // ligne du balayage émulé (0..261)
  uint8_t  idx;     // 0x08-0x0F = registre $FF98-$FF9F, 0x10-0x1F = palette 0-15
  uint8_t  val;
};

struct gime_vframe_t
{
  uint8_t  reg[16];       // état de $FF90-$FF9F au début de la trame (VBORD)
  uint8_t  palette[16];   // palette au début de la trame
  uint16_t n;             // nombre de changements notés
  bool     overflow;      // plus de GIME_VLOG_MAX changements dans la trame
  uint32_t frame;         // numéro de trame émulée
  gime_vlog_entry_t e[GIME_VLOG_MAX];
};

// Copie la dernière trame complète publiée dans *dst (lecture sans verrou protégée par un compteur de
// séquence). false si aucune trame publiée.
bool gime_vlog_get_frame(gime_vframe_t *dst);
extern volatile uint32_t gime_vlog_published_frame;   // appariement capture / journal

// Appelé à chaque trame émulée au début de VBORD (signal FS du VDG/GIME vers le PIA0 CB1) et à chaque
// fin de ligne (HS vers PIA0 CA1). nullptr = rien.
extern void (*gime_field_sync_cb)(void);        // FS : front descendant (fin de la bordure du bas)
extern void (*gime_field_sync_rise_cb)(void);   // FS front montant (6 lignes plus tard)
extern void (*gime_hsync_cb)(void);

#endif
