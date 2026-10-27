// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Port série du CoCo (« bit banger ») : décodage UART en temps émulé (voir bitbang.h).

#include "bitbang.h"
#include "drivewire.h"
#include "cc3net.h"     // DriveWire par le Wi-Fi
#include "rs232.h"
#include "modem.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef ESP_PLATFORM
#include <Arduino.h>
#include "esp_timer.h"
#else
#include <stdlib.h>
#endif

#define TICKS_HZ 14318180u

const uint32_t k_bb_speed[BB_SPEED_COUNT] = { 0, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 };

uint8_t g_bb_mode  = BB_PRINTER;
uint8_t g_bb_speed = 0;

static inline int32_t dt(uint32_t a, uint32_t b) { return (int32_t)(a - b); }   // a - b signé
static inline uint32_t now32(void) { return (uint32_t)g_bb_ticks; }

// ---------------------------------------------------------------------------
// Émission (CoCo -> extérieur) : fronts de $FF20 bit 1
// ---------------------------------------------------------------------------
#define EQ 512u                      // fronts en attente de décodage
static uint32_t *s_edge;            // files en PSRAM (Bb_Alloc) : RAM interne gardée pour l'USB et le Wi-Fi
static uint16_t s_e_rd = 0, s_e_wr = 0;
static uint8_t  s_lvl_rd = 1;        // niveau avant le front s_e_rd
static uint8_t  s_line = 1;          // niveau actuel de la sortie
static uint32_t s_last_edge = 0;
static bool     s_in_frame = false;
static uint32_t s_t0 = 0;            // front de départ de la trame en cours

// Vitesse mesurée : 16 dernières impulsions (durées entre fronts).
#define PW 16u
static uint32_t s_pulse[PW];
static uint8_t  s_pulse_n = 0, s_pulse_i = 0;
static uint32_t s_est_q8 = 0;        // durée d'un bit mesurée (1/256 tic), 0 = inconnue

// Réception (extérieur -> CoCo)
#define RQ 512u
static uint8_t  *s_rq;
static uint32_t *s_rq_time;       // arrivée de chaque octet (un octet ne part pas avant)
static uint16_t s_rq_h = 0, s_rq_t = 0;
static bool     s_rx_active = false;
static uint8_t  s_rx_byte = 0;
static uint32_t s_rx_t0 = 0, s_rx_end = 0, s_rx_prev_end = 0;
static bool     s_rx_idle_seen = true;   // lecture « repos » depuis la fin du dernier octet
static uint32_t s_rx_idle_t = 0;         // dernière lecture « repos »
static uint32_t s_rx_bit_q8 = 0;

// Vers l'UART / l'imprimante
#define TQ 512u
static uint8_t  *s_tq;
static uint16_t s_tq_h = 0, s_tq_t = 0;
#define PQ 2048u
static uint8_t  *s_pq;
static volatile uint16_t s_pq_h = 0, s_pq_t = 0;
static volatile uint32_t s_prn_ms = 0;

static struct { uint32_t tx, rx, ferr, glitch, drop; } st;

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
static void *Bb_Mem(size_t n) { void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM); return p ? p : malloc(n); }
#else
static void *Bb_Mem(size_t n) { return malloc(n); }
#endif
static bool Bb_Alloc(void)
{
  if (s_edge) return true;
  s_edge    = (uint32_t *)Bb_Mem(EQ * sizeof(uint32_t));
  s_rq      = (uint8_t *)Bb_Mem(RQ);
  s_rq_time = (uint32_t *)Bb_Mem(RQ * sizeof(uint32_t));
  s_tq      = (uint8_t *)Bb_Mem(TQ);
  s_pq      = (uint8_t *)Bb_Mem(PQ);
  if (s_edge && s_rq && s_rq_time && s_tq && s_pq) return true;
  s_edge = nullptr;                     // (manque de mémoire : port série inactif)
  return false;
}

#ifdef ESP_PLATFORM
static uint32_t Now_Ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
#else
extern int64_t esp_timer_get_time();   // banc : même horloge que main.cpp
static uint32_t Now_Ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
#endif

// ---------------------------------------------------------------------------
static uint32_t Bb_FixedBitQ8(void)
{
  return g_bb_speed ? (uint32_t)(((uint64_t)TICKS_HZ << 8) / k_bb_speed[g_bb_speed]) : 0;
}

// Durée d'un bit : impulsion la plus courte confirmée (une autre à ±1/6) puis moyenne de toutes les
// impulsions qui en sont un multiple (1-9 bits).
static void Bb_Estimate(void)
{
  uint32_t m = 0;
  for (uint8_t i = 0; i < s_pulse_n; i++)
  {
    const uint32_t p = s_pulse[i];
    if (m && p >= m) continue;
    for (uint8_t j = 0; j < s_pulse_n; j++)
      if (j != i && s_pulse[j] + p / 6 >= p && s_pulse[j] <= p + p / 6) { m = p; break; }
  }
  if (!m || m > TICKS_HZ / 250u) return;     // plus lent que 250 bauds : du son, pas une liaison série
  uint64_t S = 0; uint32_t K = 0;
  for (uint8_t i = 0; i < s_pulse_n; i++)
  {
    const uint32_t p = s_pulse[i];
    const uint32_t k = (p + m / 2) / m;
    if (k < 1 || k > 9) continue;
    const uint32_t e = (p > k * m) ? p - k * m : k * m - p;
    if (e > m / 4) continue;
    S += p; K += k;
  }
  if (K) s_est_q8 = (uint32_t)((S << 8) / K);
}

static uint32_t Bb_BitQ8(void)
{
  const uint32_t f = Bb_FixedBitQ8();
  return f ? f : s_est_q8;
}

uint32_t Bb_CurrentBaud(void)
{
  const uint32_t q = Bb_BitQ8();
  return q ? (uint32_t)(((uint64_t)TICKS_HZ << 8) / q) : 0;
}

// Vitesse normalisée la plus proche (pour l'UART)
static uint32_t Bb_NearestStd(uint32_t baud)
{
  uint32_t best = 115200, be = 0xFFFFFFFFu;
  for (uint8_t i = 1; i < BB_SPEED_COUNT; i++)
  {
    const uint32_t s = k_bb_speed[i];
    const uint32_t e = (s > baud) ? s - baud : baud - s;
    if ((uint64_t)e * 1000 / s < (uint64_t)be) { be = (uint32_t)((uint64_t)e * 1000 / s); best = s; }
  }
  return best;
}

static inline bool rq_full(void) { return ((s_rq_h + 1u) % RQ) == s_rq_t; }
static inline void rq_put(uint8_t b) { if (rq_full()) { st.drop++; return; } s_rq[s_rq_h] = b; s_rq_time[s_rq_h] = now32(); s_rq_h = (uint16_t)((s_rq_h + 1u) % RQ); }
static inline void tq_put(uint8_t b)
{
  const uint16_t n = (uint16_t)((s_tq_h + 1u) % TQ);
  if (n == s_tq_t) { st.drop++; return; }
  s_tq[s_tq_h] = b; s_tq_h = n;
}

static void Dw_Drain(void)
{
  while ((dw_becker_read(0xFF41) & 0x02) && !rq_full()) rq_put(dw_becker_read(0xFF42));
}

// Un octet décodé
static void Bb_Deliver(uint8_t b)
{
  st.tx++;
  switch (g_bb_mode)
  {
    case BB_PRINTER:
    {
      const uint16_t n = (uint16_t)((s_pq_h + 1u) % PQ);
      if (n == s_pq_t) { st.drop++; break; }
      s_pq[s_pq_h] = b; s_pq_h = n;
      s_prn_ms = Now_Ms();
      break;
    }
    case BB_DRIVEWIRE:
      dw_becker_write(0xFF42, b);
      Dw_Drain();
      break;
    case BB_UART:
      if (!Bb_UartBlocked()) tq_put(b);
      break;
    case BB_NETDW:
      NetDw_Put(b);
      break;
    case BB_MODEM:
      if (!Bb_ModemBlocked()) Modem_Put(b);
      break;
    default: break;
  }
#ifndef ESP_PLATFORM
  if (getenv("BBDBG")) fprintf(stderr, (b >= 32 && b < 127) ? "[BB TX] '%c'\n" : "[BB TX] $%02X\n", b);
#endif
}

// Niveau de la ligne à l'instant t (fronts non encore consommés)
static uint8_t Bb_LevelAt(uint32_t t)
{
  uint8_t l = s_lvl_rd;
  for (uint16_t j = s_e_rd; j != s_e_wr; j = (uint16_t)((j + 1u) % EQ))
  {
    if (dt(s_edge[j], t) > 0) break;
    l ^= 1;
  }
  return l;
}

static void Bb_Consume(uint32_t t)   // jette les fronts jusqu'à t compris
{
  while (s_e_rd != s_e_wr && dt(s_edge[s_e_rd], t) <= 0)
  {
    s_lvl_rd ^= 1;
    s_e_rd = (uint16_t)((s_e_rd + 1u) % EQ);
  }
}

// Décode les trames complètes à l'instant « now ».
static void Bb_Process(uint32_t now)
{
  for (;;)
  {
    if (!s_in_frame)
    {
      // prochain front descendant (1 -> 0)
      while (s_e_rd != s_e_wr && s_lvl_rd == 0) { s_lvl_rd = 1; s_e_rd = (uint16_t)((s_e_rd + 1u) % EQ); }
      if (s_e_rd == s_e_wr) return;
      s_t0 = s_edge[s_e_rd];
      s_in_frame = true;
    }
    uint32_t q = Bb_BitQ8();
    if (!q)
    {
      // vitesse encore inconnue : on attend d'autres impulsions (50 ms au plus)
      if (dt(now, s_t0) < (int32_t)(TICKS_HZ / 20)) return;
      uint32_t m = 0;
      for (uint8_t i = 0; i < s_pulse_n; i++) if (!m || s_pulse[i] < m) m = s_pulse[i];
      if (!m || m > TICKS_HZ / 250u) { s_in_frame = false; Bb_Consume(s_t0); continue; }
      q = m << 8;
    }
    const uint32_t t_stop = s_t0 + (uint32_t)(((uint64_t)q * 19u) >> 9);   // 9,5 bits
    if (dt(now, t_stop) < 0) return;                                     // trame pas finie
    if (Bb_LevelAt(s_t0 + (q >> 9)) != 0)                                // bit de départ trop court
    {
      st.glitch++;
      s_in_frame = false;
      Bb_Consume(s_t0);
      continue;
    }
    uint8_t b = 0;
    for (uint8_t i = 0; i < 8; i++)
      if (Bb_LevelAt(s_t0 + (uint32_t)(((uint64_t)q * (2u * i + 3u)) >> 9))) b |= (uint8_t)(1u << i);
    const uint8_t stop = Bb_LevelAt(t_stop);
    Bb_Consume(t_stop);
    s_in_frame = false;
    if (stop) Bb_Deliver(b); else st.ferr++;
  }
}

// ---------------------------------------------------------------------------
void Bb_Reset(void)
{
  if (!Bb_Alloc()) g_bb_mode = BB_OFF;
  s_e_rd = s_e_wr = 0;
  s_lvl_rd = s_line;
  s_in_frame = false;
  s_pulse_n = s_pulse_i = 0;
  s_est_q8 = 0;
  s_rq_h = s_rq_t = 0;
  s_tq_h = s_tq_t = 0;
  s_rx_active = false;
  s_rx_prev_end = s_rx_idle_t = now32();
  s_rx_idle_seen = true;
  s_rx_bit_q8 = 0;
}

void Bb_Write(uint8_t ff20, uint8_t ddr)
{
  const uint8_t lvl = (ddr & 0x02) ? (uint8_t)((ff20 >> 1) & 1) : 1;
  if (lvl == s_line) return;
  s_line = lvl;
  if (g_bb_mode == BB_OFF || !s_edge) return;
  const uint32_t now = now32();
  // impulsion qui se termine (largeur d'un ou plusieurs bits)
  const uint32_t p = now - s_last_edge;
  s_last_edge = now;
  if (p >= 8 && p < TICKS_HZ / 20)
  {
    s_pulse[s_pulse_i] = p;
    s_pulse_i = (uint8_t)((s_pulse_i + 1u) % PW);
    if (s_pulse_n < PW) s_pulse_n++;
    // mesure : tant qu'inconnue, à chaque impulsion plus courte qu'un bit (5/6), puis toutes les 16
    if (!g_bb_speed && (!s_est_q8 || s_pulse_i == 0 || ((uint64_t)p << 8) * 6 < (uint64_t)s_est_q8 * 5)) Bb_Estimate();
  }
  else if (p >= TICKS_HZ / 20 && !g_bb_speed && p > TICKS_HZ)   // > 1 s de silence : nouvelle mesure
  {
    s_pulse_n = s_pulse_i = 0;
    s_est_q8 = 0;
  }
  const uint16_t n = (uint16_t)((s_e_wr + 1u) % EQ);
  if (n == s_e_rd)                    // file pleine : on décode d'abord, sinon on jette le plus vieux
  {
    Bb_Process(now);
    if (n == s_e_rd) { s_lvl_rd ^= 1; s_e_rd = (uint16_t)((s_e_rd + 1u) % EQ); s_in_frame = false; st.drop++; }
  }
  s_edge[s_e_wr] = now;
  s_e_wr = n;
  if (((s_e_wr + EQ - s_e_rd) % EQ) > EQ / 2) Bb_Process(now);
}

uint8_t Bb_InBit(void)
{
  if (g_bb_mode == BB_OFF || g_bb_mode == BB_PRINTER || !s_edge) return 0;   // imprimante prête
  const uint32_t now = now32();
  if (g_bb_mode == BB_DRIVEWIRE && !s_rx_active && s_rq_h == s_rq_t) Bb_Process(now);
  for (;;)
  {
    if (s_rx_active)
    {
      if (dt(now, s_rx_end) >= 0) { s_rx_active = false; s_rx_prev_end = s_rx_end; continue; }
      const int32_t d = dt(now, s_rx_t0);
      if (d < 0) return 1;
      const uint32_t idx = (uint32_t)(((uint64_t)d << 8) / s_rx_bit_q8);
#ifndef ESP_PLATFORM
      { static int dbg = getenv("BBRXDBG") ? atoi(getenv("BBRXDBG")) : 0; if (dbg > 0) { dbg--; fprintf(stderr, "[BBRX] octet $%02X d=%ld idx=%u\n", s_rx_byte, (long)d, (unsigned)idx); } }
#endif
      if (idx == 0) return 0;
      if (idx <= 8) return (uint8_t)((s_rx_byte >> (idx - 1)) & 1);
      return 1;
    }
    if (s_rq_h == s_rq_t) { s_rx_idle_seen = true; s_rx_idle_t = now; return 1; }
    // Départ du prochain octet flot continu (octet déjà là à la fin du précédent, aucune lecture «
    // repos » entre les deux)
    const uint32_t avail = s_rq_time[s_rq_t];
    uint32_t t0;
    if (!s_rx_idle_seen && dt(avail, s_rx_prev_end) <= 0) t0 = s_rx_prev_end;
    else if (s_rx_idle_seen && dt(s_rx_idle_t, avail) >= 0) t0 = now;
    else { s_rx_idle_seen = true; s_rx_idle_t = now; return 1; }
    s_rx_bit_q8 = Bb_BitQ8();
    if (!s_rx_bit_q8) s_rx_bit_q8 = (uint32_t)(((uint64_t)TICKS_HZ << 8) / 115200u);
    s_rx_byte = s_rq[s_rq_t];
    s_rq_t = (uint16_t)((s_rq_t + 1u) % RQ);
    st.rx++;
    s_rx_t0 = t0;
    s_rx_idle_seen = false;
#ifndef ESP_PLATFORM
    if (getenv("BBSTART")) fprintf(stderr, "[BBRX] depart $%02X t0=%u now=%u\n", s_rx_byte, (unsigned)s_rx_t0, (unsigned)now);
#endif
    s_rx_end = s_rx_t0 + (uint32_t)(((uint64_t)s_rx_bit_q8 * 10u) >> 8);
    s_rx_active = true;
  }
}

// ---------------------------------------------------------------------------
// UART
// ---------------------------------------------------------------------------
bool Bb_UartBlocked(void) { return g_rs_present && g_rs_link == RS_LINK_UART; }
bool Bb_ModemBlocked(void) { return g_rs_present && g_rs_link == RS_LINK_MODEM; }

#ifdef ESP_PLATFORM
static void Uart_SetBaud(uint32_t b) { Serial1.updateBaudRate(b); }
static int  Uart_Avail(void)         { return Serial1.available(); }
static int  Uart_Read(void)          { return Serial1.read(); }
static int  Uart_Room(void)          { return Serial1.availableForWrite(); }
static void Uart_Write(uint8_t b)    { Serial1.write(b); }
#else
// Banc : BBIN = octets reçus ; BBUARTDW = « PC » DriveWire au bout du câble (le serveur intégré, servi
// à la cadence de Bb_Poll : latence réaliste).
static const char *s_bench_in = nullptr;
static bool s_bench_dw = getenv("BBUARTDW") != nullptr;
static void Uart_SetBaud(uint32_t) {}
static int  Uart_Avail(void)
{
  if (s_bench_dw) return (dw_becker_read(0xFF41) & 2) ? 1 : 0;
  if (!s_bench_in) { s_bench_in = getenv("BBIN"); if (!s_bench_in) s_bench_in = ""; }
  return *s_bench_in ? 1 : 0;
}
static int  Uart_Read(void)          { if (s_bench_dw) return dw_becker_read(0xFF42); return *s_bench_in ? (uint8_t)*s_bench_in++ : -1; }
static int  Uart_Room(void)          { return 64; }
static void Uart_Write(uint8_t b)
{
  if (s_bench_dw) { dw_becker_write(0xFF42, b); return; }
  fprintf(stderr, (b >= 32 && b < 127) ? "[BB UART] '%c'\n" : "[BB UART] $%02X\n", b);
}
#endif

void Bb_Poll(void)
{
  if (g_bb_mode == BB_OFF || !s_edge) return;
#ifdef ESP_PLATFORM
  static uint32_t last = 0;
  const uint32_t ms = Now_Ms();
  if (ms == last) return;                    // liaisons servies au plus une fois par ms
  last = ms;
#endif
  Bb_Process(now32());
  if (g_bb_mode == BB_DRIVEWIRE) { Dw_Drain(); return; }
  if (g_bb_mode == BB_NETDW) { while (!rq_full() && NetDw_Ready()) { const int b = NetDw_Get(); if (b < 0) break; rq_put((uint8_t)b); } return; }
  if (g_bb_mode == BB_MODEM)                   // modem Hayes par le Wi-Fi
  {
    if (!Bb_ModemBlocked()) while (!rq_full() && Modem_Ready()) { const int b = Modem_Get(); if (b < 0) break; rq_put((uint8_t)b); }
    return;
  }
  if (g_bb_mode != BB_UART || Bb_UartBlocked()) return;
  static uint32_t s_applied = 0;
  uint32_t baud = g_bb_speed ? k_bb_speed[g_bb_speed] : (s_est_q8 ? Bb_NearestStd(Bb_CurrentBaud()) : 115200u);
  if (baud != s_applied) { Uart_SetBaud(baud); s_applied = baud; }
  int room = Uart_Room();
  while (s_tq_t != s_tq_h && room-- > 0) { Uart_Write(s_tq[s_tq_t]); s_tq_t = (uint16_t)((s_tq_t + 1u) % TQ); }
  int n = Uart_Avail();
  while (n-- > 0 && !rq_full())
  {
    const int b = Uart_Read();
    if (b < 0) break;
    rq_put((uint8_t)b);
  }
}

int Bb_PrnGet(void)
{
  if (!s_pq || s_pq_t == s_pq_h) return -1;
  const uint8_t b = s_pq[s_pq_t];
  s_pq_t = (uint16_t)((s_pq_t + 1u) % PQ);
  return b;
}
uint32_t Bb_PrnLastMs(void) { return s_prn_ms; }

void Bb_Report(char *buf, size_t n)
{
  static const char *const k_mode[BB_MODE_COUNT] = { "arret", "imprimante -> SD", "DriveWire integre", "UART1 (TX 41 / RX 42)", "DriveWire par le Wi-Fi", "modem Wi-Fi" };
  snprintf(buf, n, "[BITBANG] %s%s, %lu bauds %s ; emis %lu, recus %lu, erreurs de trame %lu, parasites %lu, perdus %lu",
           k_mode[g_bb_mode < BB_MODE_COUNT ? g_bb_mode : 0],
           (g_bb_mode == BB_UART && Bb_UartBlocked()) ? " (UART pris par le RS-232 Pak)" : "",
           (unsigned long)Bb_CurrentBaud(), g_bb_speed ? "(fixe)" : "(auto)",
           (unsigned long)st.tx, (unsigned long)st.rx, (unsigned long)st.ferr, (unsigned long)st.glitch, (unsigned long)st.drop);
}
