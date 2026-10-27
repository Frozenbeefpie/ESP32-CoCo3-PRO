// ESP32-CoCo3-PRO 1.0 — Tandy Color Computer 3 emulator on ESP32-S3 (MIT license).
// Auteur: Luc Dion 2026-10-26

// Mise à jour du firmware depuis la carte SD (version publique, fichiers .FLH non chiffrés).
// Format .FLH : somme décimale, « - », texte libre, « ~ », image ESP32 ; la somme est celle de tous
// les octets à partir du « - » inclus.
// Auteur : Luc Dion, inspiré de ESP32-CoCo2 de Cedric Beaudoin.

#include "main.h"
#include <Update.h>

// Image binaire : écrite dans la partition OTA inactive ; la carte n'y bascule que si l'image est
// complète et vérifiée par ESP-IDF. Fichier effacé puis redémarrage en cas de succès.
void flashFromSD(const char *filename)
{
  if (!SD_MMC.exists(filename)) return;
  File f = SD_MMC.open(filename, FILE_READ);
  if (!f) return;
  const size_t size = f.size();
  if (!Update.begin(size)) { f.close(); return; }
  const size_t written = Update.writeStream(f);
  f.close();
  if (written != size) { Update.abort(); return; }
  if (Update.end(true))
  {
    SD_MMC.remove(filename);
    esp_restart();
  }
}

// Copie la partie binaire d'un .FLH (tout ce qui suit le premier « ~ »).
bool copyFile(const char *srcFilename, const char *destFilename)
{
  File src = SD_MMC.open(srcFilename, FILE_READ);
  if (!src) return false;
  File dst = SD_MMC.open(destFilename, FILE_WRITE);
  uint8_t *buf = (uint8_t *)malloc(4096);   // > 256 o : en PSRAM
  if (!dst || !buf) { src.close(); if (dst) dst.close(); free(buf); return false; }
  bool body = false, ok = true;
  size_t n;
  while (ok && (n = src.read(buf, 4096)) > 0)
  {
    size_t start = 0;
    if (!body)
    {
      while (start < n && buf[start] != '~') start++;
      if (start == n) continue;          // pas encore de « ~ »
      start++;
      body = true;
    }
    ok = dst.write(buf + start, n - start) == n - start;
  }
  src.close();
  dst.close();
  free(buf);
  return ok && body;
}

// Vérifie la somme d'un .FLH (1 à 14 chiffres, puis « - »).
bool ValidFirmwareFile(const char *filename)
{
  File f = SD_MMC.open(filename, FILE_READ);
  uint8_t *buf = (uint8_t *)malloc(4096);
  if (!f || !buf) { if (f) f.close(); free(buf); return false; }
  uint32_t want = 0, sum = 0;
  int digits = 0;
  bool dash = false, ok = true;
  size_t n;
  while (ok && (n = f.read(buf, 4096)) > 0)
  {
    for (size_t i = 0; i < n; i++)
    {
      const uint8_t c = buf[i];
      if (!dash)
      {
        if (c == '-' && digits > 0) { dash = true; sum += c; }
        else if (c >= '0' && c <= '9' && digits < 14) { want = want * 10u + (uint32_t)(c - '0'); digits++; }
        else { ok = false; break; }
      }
      else sum += c;
    }
  }
  f.close();
  free(buf);
  return ok && dash && sum == want;
}

// Gardée pour compatibilité (aucun système de fichiers interne n'est utilisé).
void InitFilesystem(void) {}
