// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// RS-232 Pak : ACIA 6551 (voir rs232.h). D'après MAME mos6551.cpp (license:BSD-3-Clause).

#include "rs232.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef ESP_PLATFORM
#include <Arduino.h>
#include "esp_attr.h"
#include "esp_timer.h"
#else
#define IRAM_ATTR
#include <stdlib.h>
#endif

extern volatile uint64_t g_emu_ticks;   // main.cpp : temps émulé (14,31818 MHz)
#include "modem.h"
#define TICKS_HZ 14318180ull

const uint32_t k_rs_speed[RS_SPEED_COUNT] = { 0, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };

volatile bool g_rs_present = false;
uint8_t g_rs_slot  = 0;
uint8_t g_rs_link  = RS_LINK_UART;
uint8_t g_rs_speed = 0;

enum { SR_PE = 0x01, SR_FE = 0x02, SR_OVR = 0x04, SR_RDRF = 0x08, SR_TDRE = 0x10, SR_DCD = 0x20, SR_DSR = 0x40, SR_IRQ = 0x80 };
enum { IRQ_RDRF = 1, IRQ_TDRE = 2 };

// Diviseurs du générateur interne (quartz 1,8432 MHz / 16 / diviseur). Code 0 (horloge externe x16) :
// 115 200 bauds, comme MAME (quartz / 16).
static const uint16_t k_div[16] = { 1, 2304, 1536, 1048, 856, 768, 384, 192, 96, 64, 48, 32, 24, 16, 12, 6 };

static struct
{
  uint8_t  control, command, status;
  uint8_t  rdr, tdr;
  uint8_t  irq_state;
  bool     irq_out;
  bool     tx_shift_busy;       // octet en cours d'émission
  uint8_t  tx_shift;
  uint64_t tx_done;             // fin de l'émission en cours (tics)
  uint64_t rx_ready;            // prochain octet reçu au plus tôt (tics)
  uint32_t char_ticks;          // durée d'un caractère
  // statistiques
  uint32_t n_tx, n_rx, n_drop;
} s;

// File de réception (liaison -> ACIA) et d'émission (ACIA -> liaison).
#define RQ 1024u
static uint8_t  *s_rq;                  // files en PSRAM (Rs_Alloc), RAM interne gardée pour l'USB
static uint16_t s_rq_h = 0, s_rq_t = 0;
#define TQ 512u
static uint8_t  *s_tq;
static uint16_t s_tq_h = 0, s_tq_t = 0;
static uint32_t s_applied_baud = 0;     // vitesse réglée sur l'UART

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
static void *Rs_Mem(size_t n) { void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM); return p ? p : malloc(n); }
#else
static void *Rs_Mem(size_t n) { return malloc(n); }
#endif
static bool Rs_Alloc(void)
{
  if (!s_rq) s_rq = (uint8_t *)Rs_Mem(RQ);
  if (!s_tq) s_tq = (uint8_t *)Rs_Mem(TQ);
  return s_rq && s_tq;
}

static inline bool rq_empty(void) { return s_rq_h == s_rq_t; }
static inline void rq_put(uint8_t b)
{
  const uint16_t n = (uint16_t)((s_rq_h + 1u) % RQ);
  if (!s_rq || n == s_rq_t) { s.n_drop++; return; }
  s_rq[s_rq_h] = b; s_rq_h = n;
}
static inline uint8_t rq_get(void) { const uint8_t b = s_rq[s_rq_t]; s_rq_t = (uint16_t)((s_rq_t + 1u) % RQ); return b; }
static inline void tq_put(uint8_t b)
{
  const uint16_t n = (uint16_t)((s_tq_h + 1u) % TQ);
  if (!s_tq || n == s_tq_t) { s.n_drop++; return; }
  s_tq[s_tq_h] = b; s_tq_h = n;
}

uint32_t Rs_ProgrammedBaud(void)
{
  return 1843200u / (16u * k_div[s.control & 0x0F]);
}
static uint32_t Rs_EffectiveBaud(void)
{
  return g_rs_speed ? k_rs_speed[g_rs_speed] : Rs_ProgrammedBaud();
}
static void Rs_UpdateCharTime(void)
{
  const uint32_t wl = 8u - ((s.control >> 5) & 3u);
  const uint32_t par = ((s.command >> 5) & 1u) ? 1u : 0u;
  const uint32_t stop = (s.control & 0x80) ? 2u : 1u;
  const uint32_t bits = 1u + wl + par + stop;
  const uint32_t baud = Rs_EffectiveBaud();
  s.char_ticks = (uint32_t)((TICKS_HZ * bits) / (baud ? baud : 9600u));
}

static void Rs_UpdateIrq(void)
{
  const bool on = s.irq_state != 0;
  if (on) s.status |= SR_IRQ; else s.status &= (uint8_t)~SR_IRQ;
  if (on != s.irq_out) { s.irq_out = on; Rs_IrqChanged(on); }
}

static inline bool dtr_on(void)     { return (s.command & 0x01) != 0; }
static inline bool rx_irq_en(void)  { return dtr_on() && !(s.command & 0x02); }
static inline bool tx_irq_en(void)  { return dtr_on() && ((s.command >> 2) & 3) == 1; }

static void Rs_StatusLines(void)
{
  // DCD / DSR : 0 = actifs. Rien de branché : porteuse absente.
  if (g_rs_link == RS_LINK_NONE) s.status |= (SR_DCD | SR_DSR);
  else s.status &= (uint8_t)~(SR_DCD | SR_DSR);
  if (g_rs_link == RS_LINK_MODEM && !Modem_Carrier()) s.status |= SR_DCD;   // DCD = porteuse du modem (AT&C1)
}

void Rs_Reset(void)
{
  if (!Rs_Alloc()) g_rs_present = false;   // (manque de mémoire : Pak inactif)
  memset(&s, 0, sizeof s);
  s.status = SR_TDRE;
  Rs_StatusLines();
  s_rq_h = s_rq_t = 0;
  s_tq_h = s_tq_t = 0;
  Rs_UpdateCharTime();
  if (s_applied_baud == 0) s_applied_baud = 1;   // forcer le réglage de l'UART au premier passage
  Rs_UpdateIrq();
}

// Avance l'émetteur et le récepteur jusqu'à « now ».
static void Rs_Advance(uint64_t now)
{
  // Émetteur : registre à décalage -> liaison, puis TDR -> registre à décalage.
  if (s.tx_shift_busy && now >= s.tx_done)
  {
    s.tx_shift_busy = false;
    s.n_tx++;
    if (g_rs_link == RS_LINK_LOOP) rq_put(s.tx_shift);
    else if (g_rs_link == RS_LINK_UART) tq_put(s.tx_shift);
    else if (g_rs_link == RS_LINK_MODEM) Modem_Put(s.tx_shift);
  }
  if (!s.tx_shift_busy && !(s.status & SR_TDRE))
  {
    s.tx_shift = s.tdr;
    s.tx_shift_busy = true;
    s.tx_done = now + s.char_ticks;
    s.status |= SR_TDRE;
    if (tx_irq_en()) { s.irq_state |= IRQ_TDRE; Rs_UpdateIrq(); }
  }
  // Récepteur : un octet de la file quand RDR est libre et un caractère écoulé.
  if (!dtr_on()) return;                          // récepteur arrêté (DTR inactif) : les octets attendent
  if (!(s.status & SR_RDRF) && !rq_empty())
  {
    if (s.rx_ready < now - s.char_ticks) s.rx_ready = now;   // ligne au repos : pas de retard accumulé
    if (now >= s.rx_ready)
    {
      uint8_t b = rq_get();
      if (((s.control >> 5) & 3) != 0) b &= (uint8_t)(0xFF >> ((s.control >> 5) & 3));   // 7, 6 ou 5 bits
      s.rdr = b;
      s.status |= SR_RDRF;
      s.n_rx++;
      s.rx_ready = now + s.char_ticks;
      if (s.command & 0x10) { if (g_rs_link == RS_LINK_MODEM) Modem_Put(b); else tq_put(b); }   // mode écho
      if (rx_irq_en()) { s.irq_state |= IRQ_RDRF; Rs_UpdateIrq(); }
    }
  }
}

uint8_t Rs_Read(uint16_t a)
{
  Rs_Advance(g_emu_ticks);
#ifndef ESP_PLATFORM
  if (getenv("RSDBG")) fprintf(stderr, "[RSDBG] lecture $%04X etat $%02X irq %u rdr $%02X\n", a, s.status, s.irq_state, s.rdr);
#endif
  switch (a & 3)
  {
    case 0:
      s.status &= (uint8_t)~(SR_PE | SR_FE | SR_OVR | SR_RDRF);
      return s.rdr;
    case 1:
    {
      const uint8_t st = s.status;
      if (s.irq_state) { s.irq_state = 0; Rs_UpdateIrq(); }
      return st;
    }
    case 2:  return s.command;
    default: return s.control;
  }
}

void Rs_Write(uint16_t a, uint8_t v)
{
  const uint64_t now = g_emu_ticks;
  Rs_Advance(now);
  switch (a & 3)
  {
    case 0:
      s.tdr = v;
      s.status &= (uint8_t)~SR_TDRE;
      Rs_Advance(now);                                    // registre à décalage libre : départ immédiat
      break;
    case 1:                                               // remise à zéro programmée
      s.status &= (uint8_t)~SR_OVR;
      s.command &= 0xE0;
      s.irq_state = 0;
      Rs_UpdateIrq();
      break;
    case 2:
      s.command = v;
      if (!rx_irq_en()) s.irq_state &= (uint8_t)~IRQ_RDRF;
      if (!tx_irq_en()) s.irq_state &= (uint8_t)~IRQ_TDRE;   // (comme MAME : l'IRQ TDRE naît au passage TDR -> registre à décalage)
      Rs_UpdateCharTime();
      Rs_UpdateIrq();
      break;
    default:
      s.control = v;
      Rs_UpdateCharTime();
      break;
  }
}

// ---------------------------------------------------------------------------
// Liaisons
// ---------------------------------------------------------------------------
#ifdef ESP_PLATFORM
#define RS_TX_PIN 41
#define RS_RX_PIN 42
static void Uart_SetBaud(uint32_t baud)
{
  Serial1.updateBaudRate(baud);
}
static int  Uart_Avail(void)          { return Serial1.available(); }
static int  Uart_Read(void)           { return Serial1.read(); }
static int  Uart_Room(void)           { return Serial1.availableForWrite(); }
static void Uart_Write(uint8_t b)     { Serial1.write(b); }
static uint32_t Now_Ms(void)          { return (uint32_t)(esp_timer_get_time() / 1000); }
#else
// Banc (PC) : octets émis -> stderr « [RS232 TX] », reçus <- variable RSIN.
static const char *s_bench_in = nullptr;
static void Uart_SetBaud(uint32_t) {}
static int  Uart_Avail(void)          { if (!s_bench_in) { s_bench_in = getenv("RSIN"); if (!s_bench_in) s_bench_in = ""; } return *s_bench_in ? 1 : 0; }
static int  Uart_Read(void)           { return *s_bench_in ? (uint8_t)*s_bench_in++ : -1; }
static int  Uart_Room(void)           { return 64; }
static void Uart_Write(uint8_t b)     { fprintf(stderr, (b >= 32 && b < 127) ? "[RS232 TX] '%c'\n" : "[RS232 TX] $%02X\n", b); }
static uint32_t Now_Ms(void)          { static uint32_t t = 0; return t += 2; }
#endif

void Rs_ApplyLink(void)
{
  s_applied_baud = 1;     // réglage de l'UART au prochain Rs_Poll
  if (g_rs_link == RS_LINK_MODEM) Modem_Use();
  Rs_StatusLines();
  Rs_UpdateCharTime();
}

// Coeur CPU, à chaque tour de la boucle d'émulation.
void Rs_Poll(void)
{
  if (!g_rs_present) return;
  Rs_Advance(g_emu_ticks);
  if (g_rs_link == RS_LINK_MODEM)               // modem Hayes par le Wi-Fi
  {
    Modem_SetDtr(dtr_on());
    Rs_StatusLines();                            // DCD suit la porteuse
    if (!dtr_on()) return;                       // DTR inactif : les octets attendent dans le modem
    while (((s_rq_h + 1u) % RQ) != s_rq_t && Modem_Ready()) { const int b = Modem_Get(); if (b < 0) break; rq_put((uint8_t)b); }
    return;
  }
  if (g_rs_link != RS_LINK_UART) return;
  static uint32_t last = 0;
  const uint32_t ms = Now_Ms();
  if (ms == last) return;                       // liaison servie au plus une fois par ms
  last = ms;
  const uint32_t baud = Rs_EffectiveBaud();
  if (baud != s_applied_baud) { Uart_SetBaud(baud); s_applied_baud = baud; }
  // ACIA -> UART
  int room = Uart_Room();
  while (s_tq_t != s_tq_h && room-- > 0) { Uart_Write(s_tq[s_tq_t]); s_tq_t = (uint16_t)((s_tq_t + 1u) % TQ); }
  // UART -> file de réception (tant qu'il y a de la place ; DTR inactif ils restent dans le tampon de
  // l'UART, comme un terminal qui attend)
  if (!dtr_on()) return;
  int n = Uart_Avail();
  while (n-- > 0 && ((s_rq_h + 1u) % RQ) != s_rq_t)
  {
    const int b = Uart_Read();
    if (b < 0) break;
    rq_put((uint8_t)b);
  }
}

void Rs_Report(char *buf, size_t n)
{
  static const char *const k_link[RS_LINK_COUNT] = { "aucune", "UART1 (TX 41 / RX 42)", "bouclage", "modem Wi-Fi" };
  snprintf(buf, n, "[RS232] %s, slot %u, liaison %s, %lu bauds %s (programme %lu) ; ctrl $%02X cmd $%02X etat $%02X ; emis %lu, recus %lu, perdus %lu, en attente %u",
           g_rs_present ? "RS-232 Pak present" : "absent", (unsigned)g_rs_slot + 1,
           k_link[g_rs_link < RS_LINK_COUNT ? g_rs_link : 0], (unsigned long)Rs_EffectiveBaud(),
           g_rs_speed ? "(fixe)" : "(auto)", (unsigned long)Rs_ProgrammedBaud(),
           s.control, s.command, s.status, (unsigned long)s.n_tx, (unsigned long)s.n_rx, (unsigned long)s.n_drop,
           (unsigned)((s_rq_h + RQ - s_rq_t) % RQ));
}
