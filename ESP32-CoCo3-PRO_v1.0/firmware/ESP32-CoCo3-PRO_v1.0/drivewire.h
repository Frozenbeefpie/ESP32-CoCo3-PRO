// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Serveur DriveWire 4 intégré et port Becker ($FF41/$FF42) : disques virtuels sur la carte SD.

#pragma once
#include <stdint.h>
#include <stdbool.h>

#define DW_DRIVES 4

struct dw_backend_t
{
  bool (*present)(uint8_t drive);                                   // disque monté ?
  bool (*read)(uint8_t drive, uint32_t lsn, uint8_t *buf256);       // secteur absent du fichier = 0
  bool (*write)(uint8_t drive, uint32_t lsn, const uint8_t *buf256);
  void (*get_time)(uint8_t t[6]);                                   // an-1900, mois, jour, h, min, s
};

void    dw_init(const dw_backend_t *be);
void    dw_reset(void);                    // vide les files (reset CoCo)
bool    dw_port_enabled(void);             // port Becker actif (au moins un disque DW monté)
void    dw_set_port_enabled(bool on);
uint8_t dw_becker_read(uint16_t address);  // $FF41 / $FF42
void    dw_becker_write(uint16_t address, uint8_t value);

// Statistiques (journal [DW])
struct dw_stats_t { uint32_t reads, writes, errors, unknown; uint8_t last_op; };
const dw_stats_t *dw_get_stats(void);
