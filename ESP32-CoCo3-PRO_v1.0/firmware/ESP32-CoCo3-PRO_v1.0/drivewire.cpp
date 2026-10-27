// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Serveur DriveWire 4 intégré (voir drivewire.h).

#include "drivewire.h"
#include <string.h>

static const dw_backend_t *s_be = nullptr;
static bool s_enabled = false;
static dw_stats_t s_stats;

// Réponse du serveur vers le CoCo (file circulaire)
static uint8_t  s_tx[512];
static uint16_t s_tx_head = 0, s_tx_tail = 0;

// Commande en cours de réception
static uint8_t  s_rx[300];
static uint16_t s_rx_len = 0;

// Après READEX : attente de la somme envoyée par le CoCo
static bool     s_wait_sum = false;
static uint16_t s_expected_sum = 0;
static uint8_t  s_readex_err = 0;

static uint8_t  s_sector[256];

static void tx_put(uint8_t b)
{
  const uint16_t n = (uint16_t)((s_tx_head + 1) % sizeof s_tx);
  if (n == s_tx_tail) return;                 // plein (ne devrait pas arriver)
  s_tx[s_tx_head] = b;
  s_tx_head = n;
}

static uint16_t checksum(const uint8_t *b)
{
  uint16_t s = 0;
  for (int i = 0; i < 256; i++) s = (uint16_t)(s + b[i]);
  return s;
}

void dw_init(const dw_backend_t *be) { s_be = be; dw_reset(); memset(&s_stats, 0, sizeof s_stats); }

void dw_reset(void)
{
  s_tx_head = s_tx_tail = 0;
  s_rx_len = 0;
  s_wait_sum = false;
}

bool dw_port_enabled(void)        { return s_enabled; }
void dw_set_port_enabled(bool on) { s_enabled = on; if (!on) dw_reset(); }
const dw_stats_t *dw_get_stats(void) { return &s_stats; }

static bool drive_ok(uint8_t d) { return s_be && d < DW_DRIVES && s_be->present(d); }

// Longueur totale de la commande reçue (0 = pas encore connue)
static uint16_t cmd_length(const uint8_t *b, uint16_t len)
{
  const uint8_t op = b[0];
  if (op >= 0x80 && op <= 0x8F) return 2;     // FASTWRITE canal n + octet
  switch (op)
  {
    case 0x00: case 0x49: case 0x54: case 0xF8: case 0xFE: case 0xFF:
    case 0x23: case 0x46: case 0x43:
      return 1;
    case 0x5A: case 0x50: case 0x45: case 0xC5:
      return 2;
    case 0x47: case 0x53: case 0xC3: case 0x44:
      return 3;
    case 0x63:                                  // SERREADM canal, nombre
      return 3;
    case 0xD2: case 0xF2: case 0x52: case 0x72:
      return 5;
    case 0x57: case 0x77:
      return 5 + 256 + 2;
    case 0x64:                                  // SERWRITEM canal, nombre, octets
      return (len >= 3) ? (uint16_t)(3 + b[2]) : 0;
    case 0xC4:                                  // SERSETSTAT canal, code (+26 si SS.ComSt)
      return (len >= 3) ? (uint16_t)(b[2] == 0x28 ? 3 + 26 : 3) : 0;
    case 0x01: case 0x02:                       // NAMEOBJ : longueur, nom
      return (len >= 2) ? (uint16_t)(2 + b[1]) : 0;
    default:
      return 1;                                 // inconnu : ignoré (compté)
  }
}

static void execute(const uint8_t *b, uint16_t len)
{
  const uint8_t op = b[0];
  s_stats.last_op = op;
  switch (op)
  {
    case 0x5A:                                  // DWINIT : capacités du serveur
      tx_put(0x00);
      break;
    case 0x23:                                  // TIME
    {
      uint8_t t[6] = { 126, 1, 1, 0, 0, 0 };
      if (s_be && s_be->get_time) s_be->get_time(t);
      for (int i = 0; i < 6; i++) tx_put(t[i]);
      break;
    }
    case 0xD2: case 0xF2: case 0x52: case 0x72:
    {
      const uint8_t d = b[1];
      const uint32_t lsn = ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 8) | b[4];
      uint8_t err = 0;
      if (!drive_ok(d)) { memset(s_sector, 0, 256); err = 0xF6; }          // E$NotRdy
      else if (!s_be->read(d, lsn, s_sector)) { memset(s_sector, 0, 256); err = 0xF4; }  // E$Read
      for (int i = 0; i < 256; i++) tx_put(s_sector[i]);
      s_stats.reads++;
      if (err) s_stats.errors++;
      if (op == 0xD2 || op == 0xF2)
      {
        s_wait_sum = true;
        s_expected_sum = checksum(s_sector);
        s_readex_err = err;
      }
      else
      {
        const uint16_t s = checksum(s_sector);
        tx_put((uint8_t)(s >> 8)); tx_put((uint8_t)s);
      }
      break;
    }
    case 0x57: case 0x77:
    {
      const uint8_t d = b[1];
      const uint32_t lsn = ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 8) | b[4];
      const uint16_t sum = (uint16_t)((b[261] << 8) | b[262]);
      uint8_t err = 0;
      if (checksum(b + 5) != sum) err = 0xF3;                                // E$CRC
      else if (!drive_ok(d)) err = 0xF6;
      else if (!s_be->write(d, lsn, b + 5)) err = 0xF5;                      // E$Write
      tx_put(err);
      s_stats.writes++;
      if (err) s_stats.errors++;
      break;
    }
    case 0x43:                                  // SERREAD : aucune donnée série
      tx_put(0); tx_put(0);
      break;
    case 0x63:                                  // SERREADM : n octets (vides)
      for (int i = 0; i < b[2]; i++) tx_put(0);
      break;
    case 0x01: case 0x02:                       // NAMEOBJ : non géré
      tx_put(0);
      break;
    case 0x00: case 0x49: case 0x54: case 0xF8: case 0xFE: case 0xFF:
    case 0x46: case 0x50: case 0x45: case 0xC5: case 0x47: case 0x53:
    case 0xC3: case 0x44: case 0x64: case 0xC4:
      break;
    default:
      if (op >= 0x80 && op <= 0x8F) break;
      s_stats.unknown++;
      break;
  }
  (void)len;
}

void dw_becker_write(uint16_t address, uint8_t value)
{
  if (address != 0xFF42) return;
  if (s_wait_sum)                               // somme de contrôle après READEX
  {
    s_rx[s_rx_len++] = value;
    if (s_rx_len == 2)
    {
      const uint16_t sum = (uint16_t)((s_rx[0] << 8) | s_rx[1]);
      uint8_t err = s_readex_err;
      if (!err && sum != s_expected_sum) err = 0xF3;
      tx_put(err);
      s_wait_sum = false;
      s_rx_len = 0;
    }
    return;
  }
  if (s_rx_len < sizeof s_rx) s_rx[s_rx_len++] = value;
  const uint16_t need = cmd_length(s_rx, s_rx_len);
  if (need && s_rx_len >= need)
  {
    s_rx_len = 0;          // avant execute : READEX peut passer en attente de somme
    execute(s_rx, need);
  }
}

uint8_t dw_becker_read(uint16_t address)
{
  if (address == 0xFF41) return (s_tx_head != s_tx_tail) ? 0x02 : 0x00;
  if (address == 0xFF42)
  {
    if (s_tx_head == s_tx_tail) return 0;
    const uint8_t b = s_tx[s_tx_tail];
    s_tx_tail = (uint16_t)((s_tx_tail + 1) % sizeof s_tx);
    return b;
  }
  return 0;
}
