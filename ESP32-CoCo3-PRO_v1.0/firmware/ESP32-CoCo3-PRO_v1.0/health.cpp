// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// État matériel de l'ESP32 (voir health.h).

#include "health.h"
#include <string.h>
#include <stdio.h>
#include "audio.h"

extern void Log_Line(const char *s);
extern const char *const g_fw_version_short;
extern volatile uint32_t g_perf_load_x10, g_perf_hz, g_perf_target_hz, g_perf_lost_ps;
extern volatile uint32_t g_vid_frames;
extern bool Usb_Summary(char *b, size_t n);
extern void Net_StatusLine(char *buf, size_t n);
extern bool Net_Connected(void);

#ifdef ESP_PLATFORM
#include <Arduino.h>
#include <SD_MMC.h>
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "gime.h"   // version du GIME
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"   // ulTaskGetIdleRunTimeCounterForCore
extern bool SD_MMC_MountOK;
extern void Sd_Lock(void);
extern void Sd_Unlock(void);

static uint32_t s_t_last = 0;                 // ms
static uint64_t s_us_last = 0;
static uint32_t s_idle_last[2] = { 0, 0 };
static uint8_t  s_load[2] = { 0, 0 };         // % par coeur
static uint8_t  s_vidwork = 0;                // travail réel du coeur vidéo (%)
static uint32_t s_vidle_last = 0;
static uint64_t s_us_last_v = 0;
extern volatile uint32_t g_vid_idle_cc;        // main.cpp : cycles d'attente du coeur vidéo
static float    s_temp = 0;
static uint32_t s_fps = 0, s_vf_last = 0;
static uint64_t s_sd_total = 0, s_sd_used = 0; static uint8_t s_sd_type = 0;
static volatile bool s_sd_req = true;
static uint32_t s_sketch = 0;                 // taille du firmware (lue une fois : lecture complète de la flash)

void Health_RefreshSd(void) { s_sd_req = true; }
void Health_Sample(void)
{
  const uint32_t now = millis();
  if (s_t_last && now - s_t_last < 1000u) return;
  if (!s_sketch) s_sketch = ESP.getSketchSize() | 1u;   // une fois, au démarrage
  const uint64_t us = esp_timer_get_time();
  for (int c = 0; c < 2; c++)
  {
    const uint32_t v = (uint32_t)ulTaskGetIdleRunTimeCounterForCore(c);   // µs passées dans la tâche IDLE du coeur
    if (s_us_last)
    {
      const uint32_t dt = (uint32_t)(us - s_us_last), di = v - s_idle_last[c];
      const uint32_t idle_pc = dt ? (uint32_t)((uint64_t)di * 100u / dt) : 0;
      s_load[c] = (uint8_t)(idle_pc >= 100 ? 0 : 100 - idle_pc);
    }
    s_idle_last[c] = v;
  }
  {                                              // VIDEO WORK
    const uint32_t vi = g_vid_idle_cc;
    if (s_us_last_v)
    {
      const uint64_t tot = (uint64_t)(us - s_us_last_v) * getCpuFrequencyMhz();
      const uint32_t di = vi - s_vidle_last;
      const uint32_t idle_pc = tot ? (uint32_t)((uint64_t)di * 100u / tot) : 0;
      s_vidwork = (uint8_t)(idle_pc >= 100 ? 0 : 100 - idle_pc);
    }
    s_vidle_last = vi; s_us_last_v = us;
  }
  const uint32_t vf = g_vid_frames;
  if (s_t_last) s_fps = (vf - s_vf_last) * 1000u / (now - s_t_last);
  s_vf_last = vf;
  s_us_last = us; s_t_last = now;
  s_temp = temperatureRead();
  if (s_sd_req && SD_MMC_MountOK)               // f_getfree : peut prendre une seconde sur une grosse carte (une fois)
  {
    s_sd_req = false;
    Sd_Lock();
    s_sd_type = (uint8_t)SD_MMC.cardType();
    s_sd_total = SD_MMC.totalBytes();
    s_sd_used = SD_MMC.usedBytes();
    Sd_Unlock();
  }
}

static void Bar(char *b, size_t n, uint32_t pc)  // « ######.... »
{
  if (pc > 100) pc = 100;
  char bar[11]; const uint32_t k = (pc + 5) / 10;
  for (uint32_t i = 0; i < 10; i++) bar[i] = i < k ? '#' : '.';
  bar[10] = 0;
  snprintf(b, n, "%s", bar);
}
static const char *Reason(void)
{
  switch (esp_reset_reason())
  {
    case ESP_RST_POWERON:  return "power on";
    case ESP_RST_SW:       return "restart";
    case ESP_RST_PANIC:    return "CRASH";
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "WATCHDOG";
    case ESP_RST_BROWNOUT: return "LOW VOLTAGE";
    case ESP_RST_EXT:      return "reset pin";
    case ESP_RST_DEEPSLEEP:return "sleep";
    default:               return "other";
  }
}
static void Kb(char *b, size_t n, uint32_t v) { if (v >= 10u * 1024u * 1024u) snprintf(b, n, "%lu MB", (unsigned long)(v >> 20)); else if (v >= 10240u) snprintf(b, n, "%lu KB", (unsigned long)(v >> 10)); else snprintf(b, n, "%lu B", (unsigned long)v); }

extern bool g_cpu6309_run;   // main.cpp : HD6309 en service
extern bool Emu_Running(void);   // main.cpp
void Health_Row(int i, const char **label, char *v, size_t n, uint8_t *lv)
{
  char a[24], bb[24], cc[24], bar[16];
  *lv = HL_OK;
  switch (i)
  {
    case 0:
    {
      *label = "FIRMWARE";
      // ESP.getSketchSize relit TOUT le firmware en flash (vérification, ~1,6 Mo) : les deux coeurs
      // s'arrêtent pendant la lecture
      const esp_partition_t *p = esp_ota_get_running_partition();
      snprintf(v, n, "v%s  (%s, %lu of %lu KB)", g_fw_version_short, p ? p->label : "?",
               (unsigned long)(s_sketch / 1024u), (unsigned long)(p ? p->size / 1024u : 0u));
      *lv = HL_INFO; break;
    }
    case 1:
    {
      *label = "UPTIME";
      const uint32_t s = (uint32_t)(esp_timer_get_time() / 1000000ull);
      const char *r = Reason();
      snprintf(v, n, "%lu h %02lu min %02lu s  (start: %s)", (unsigned long)(s / 3600u), (unsigned long)(s / 60u % 60u), (unsigned long)(s % 60u), r);
      *lv = (!strcmp(r, "CRASH") || !strcmp(r, "WATCHDOG") || !strcmp(r, "LOW VOLTAGE")) ? HL_WARN : HL_INFO; break;
    }
    case 2:
      *label = "CHIP";
      snprintf(v, n, "%s rev %u.%u, %lu MHz, flash %lu MB", ESP.getChipModel(), (unsigned)(ESP.getChipRevision() / 100), (unsigned)(ESP.getChipRevision() % 100),
               (unsigned long)getCpuFrequencyMhz(), (unsigned long)(ESP.getFlashChipSize() >> 20));
      *lv = HL_INFO; break;
    case 3:
      *label = "CORE 0 LOAD";
      Bar(bar, sizeof bar, s_load[0]); snprintf(v, n, "%s %3u %%  (video, USB, Wi-Fi)", bar, s_load[0]);
      *lv = HL_INFO; break;   // le coeur vidéo ne s'arrête presque jamais (98 % même Wi-Fi coupé)
    case 4:
      *label = "CORE 1 LOAD";
      Bar(bar, sizeof bar, s_load[1]); snprintf(v, n, "%s %3u %%  (%s emulation)", bar, s_load[1], g_cpu6309_run ? "HD6309" : "MC6809");
      *lv = HL_INFO; break;   // la tâche du 6809 ne cède presque jamais le coeur
    case 5:
    {
      *label = g_cpu6309_run ? "6309 WORK" : "6809 WORK";   // processeur en service
      const uint32_t l = g_perf_load_x10 / 10;
      Bar(bar, sizeof bar, l); snprintf(v, n, "%s %3lu %%  of real time", bar, (unsigned long)l);
      *lv = l > 95 ? HL_BAD : l > 85 ? HL_WARN : HL_OK; break;
    }
    case 6:
    {
      *label = "VIDEO WORK";   // coeur 0, temps hors des attentes de la vidéo (VSYNC, vTaskDelay)
      if (!Emu_Running()) { snprintf(v, n, "-- (emulation paused)"); *lv = HL_INFO; break; }   // le menu ne passe pas par l'attente mesurée
      Bar(bar, sizeof bar, s_vidwork); snprintf(v, n, "%s %3u %%  of real time (core 0)", bar, s_vidwork);
      *lv = s_vidwork > 95 ? HL_WARN : HL_OK; break;
    }
    case 7:
    {
      *label = "CHIP TEMP";
      const int t = (int)(s_temp + 0.5f);
      Bar(bar, sizeof bar, t > 0 ? (uint32_t)t : 0u); snprintf(v, n, "%s %3d C  (inside the chip)", bar, t);
      *lv = t >= 80 ? HL_BAD : t >= 65 ? HL_WARN : HL_OK; break;
    }
    case 8:
    {
      *label = "INTERNAL RAM";
      const uint32_t f = heap_caps_get_free_size(MALLOC_CAP_INTERNAL), m = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
                     l = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
      Kb(a, sizeof a, f); Kb(bb, sizeof bb, l); Kb(cc, sizeof cc, m);
      snprintf(v, n, "%s free, block %s, min %s", a, bb, cc);
      *lv = f < 6000 ? HL_BAD : f < 12000 ? HL_WARN : HL_OK; break;
    }
    case 9:
    {
      *label = "DMA RAM (USB)";
      const uint32_t c = MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL;
      const uint32_t f = heap_caps_get_free_size(c), m = heap_caps_get_minimum_free_size(c), l = heap_caps_get_largest_free_block(c);
      Kb(a, sizeof a, f); Kb(bb, sizeof bb, l); Kb(cc, sizeof cc, m);
      snprintf(v, n, "%s free, block %s, min %s", a, bb, cc);
      *lv = l < 1024 ? HL_BAD : l < 3000 ? HL_WARN : HL_OK; break;
    }
    case 10:
    {
      *label = "PSRAM";
      const uint32_t t = ESP.getPsramSize(), f = ESP.getFreePsram();
      Kb(a, sizeof a, f); Kb(bb, sizeof bb, t);
      snprintf(v, n, "%s free of %s", a, bb);
      *lv = f < 256u * 1024u ? HL_WARN : HL_OK; break;
    }
    case 11:
    {
      *label = "SD CARD";
      if (!SD_MMC_MountOK || !s_sd_total) { snprintf(v, n, "%s", SD_MMC_MountOK ? "measuring..." : "NO CARD"); *lv = SD_MMC_MountOK ? HL_INFO : HL_BAD; break; }
      const uint32_t free_mb = (uint32_t)((s_sd_total - s_sd_used) >> 20), tot_mb = (uint32_t)(s_sd_total >> 20);
      const uint32_t pc = tot_mb ? free_mb * 100u / tot_mb : 0;
      static const char *const k_t[] = { "?", "MMC", "SD", "SDHC", "?" };
      snprintf(v, n, "%lu.%lu GB free of %lu.%lu GB (%lu %%) %s", (unsigned long)(free_mb / 1024u), (unsigned long)(free_mb % 1024u * 10u / 1024u),
               (unsigned long)(tot_mb / 1024u), (unsigned long)(tot_mb % 1024u * 10u / 1024u), (unsigned long)pc, k_t[s_sd_type < 4 ? s_sd_type : 4]);
      *lv = pc < 5 ? HL_BAD : pc < 15 ? HL_WARN : HL_OK; break;
    }
    case 12:
      *label = "Wi-Fi";
      Net_StatusLine(v, n);
      *lv = Net_Connected() ? HL_OK : HL_INFO; break;
    case 13:
      *label = "USB";
      *lv = Usb_Summary(v, n) ? HL_WARN : HL_OK; break;
    case 14:
    {
      *label = "EMULATED CPU";
      const uint32_t hz = g_perf_hz, t = g_perf_target_hz;
      snprintf(v, n, "%lu.%02lu / %lu.%02lu MHz, lost %lu cyc/s, GIME %s", (unsigned long)(hz / 1000000u), (unsigned long)(hz / 10000u % 100u),
               (unsigned long)(t / 1000000u), (unsigned long)(t / 10000u % 100u), (unsigned long)g_perf_lost_ps, gime_is_1987() ? "1987" : "1986");   // GIME
      *lv = g_perf_lost_ps > 20000 ? HL_WARN : HL_OK; break;
    }
    case 15:
    {
      *label = "VIDEO / SOUND";
      const audio_stats_t *a2 = Audio_GetStats();
      snprintf(v, n, "%lu CoCo frames/s, sound lost %lu, late %lu", (unsigned long)s_fps, (unsigned long)(a2 ? a2->dropped : 0), (unsigned long)(a2 ? a2->late : 0));
      *lv = HL_INFO; break;
    }
    default: *label = ""; v[0] = 0; *lv = HL_INFO; break;
  }
}
#else
// Banc PC : rien à mesurer
void Health_RefreshSd(void) {}
void Health_Sample(void) {}
void Health_Row(int i, const char **label, char *v, size_t n, uint8_t *lv) { (void)i; *label = "BENCH"; snprintf(v, n, "-"); *lv = HL_INFO; }
#endif

void Health_Json(char *b, size_t n)
{
  size_t k = (size_t)snprintf(b, n, "{\"ok\":true,\"rows\":[");
  for (int i = 0; i < HEALTH_ROWS && k + 8 < n; i++)
  {
    const char *l; char v[96]; uint8_t lv;
    Health_Row(i, &l, v, sizeof v, &lv);
    for (char *p = v; *p; p++) if (*p == '"' || *p == '\\') *p = '\'';
    k += (size_t)snprintf(b + k, n - k, "%s[\"%s\",\"%s\",%u]", i ? "," : "", l, v, lv);
  }
  if (k + 3 < n) snprintf(b + k, n - k, "]}");
}

void Health_Report(void)
{
  char line[200];
  for (int i = 1; i < HEALTH_ROWS; i++)
  {
    if (i == 2 || i == 11 || i == 12 || i == 13) continue;   // déjà au démarrage, dans [WIFI], [USB], [PERF]
    const char *l; char v[120]; uint8_t lv;
    Health_Row(i, &l, v, sizeof v, &lv);
    snprintf(line, sizeof line, "[HEALTH] %-14s %s%s", l, v, lv == HL_BAD ? "  <-- !" : lv == HL_WARN ? "  <-- ?" : "");
    Log_Line(line);
  }
}
