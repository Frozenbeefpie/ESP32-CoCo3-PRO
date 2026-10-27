// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Rendu vidéo GIME (modes CoCo 3 : texte 40/80 colonnes, graphiques 2/4/16 couleurs, palette).

#ifndef __GIME_VIDEO_H
#define __GIME_VIDEO_H

#include <stdint.h>

// font : police GIME 128 caractères x 12 lignes (font_gimeChip de ROMS_Source.h).
void gime_video_init(const uint8_t *font);

// false = moniteur composite (défaut, comme SECB au démarrage : ses valeurs de palette par défaut sont
// des codes CMP) ; true = moniteur RGB (CM-8).
void gime_video_set_monitor_rgb(bool rgb);
bool gime_video_monitor_rgb(void);

// true si le GIME est en mode vidéo CoCo3 (INIT0 bit 7 COCO = 0).
bool gime_video_active(void);

// Largeur VGA souhaitée pour le mode courant : 640 si le mode a plus de 320 pixels logiques (texte
// 64/80 col., graphiques 512/640), sinon 320.
int gime_video_wanted_width(void);

// Prépare une trame, puis rend chaque ligne 0..out_h-1 de sortie, dans l'ordre. la trame rendue est la
// dernière trame émulée complète (gime_vlog_get_frame)
void gime_video_begin_frame(const uint8_t *ram, int out_w, int out_h);
void gime_video_render_line(int y, uint8_t *out);

// Octet VGA d'une couleur GIME 6 bits selon le moniteur choisi (utile au menu).
uint8_t gime_video_color(uint8_t gime_color);

// ---------------------------------------------------------------------------
// capture de la VRAM des modes CoCo3 au fil du balayage émulé
// ---------------------------------------------------------------------------
#define GIME_CAP_ROWS      225u                      // zone active max.
#define GIME_CAP_ROW_BYTES 256u                      // fenêtre lue par ligne
#define GIME_CAP_BUF_SIZE  (GIME_CAP_ROWS * GIME_CAP_ROW_BYTES + GIME_CAP_ROWS)

// 3 tampons de GIME_CAP_BUF_SIZE octets (PSRAM), fournis par l'appelant. nullptr = pas de capture
// (rendu sur la RAM vivante, comme avant).
void gime_video_set_capture_buffers(uint8_t *b0, uint8_t *b1, uint8_t *b2);

// Coeur CPU, fin de ligne émulée « line » (0..261, VBORD = 0).
void gime_video_capture_line(const uint8_t *ram, uint16_t line);
// file de lignes en RAM interne (slots x 256 octets), vidée par le coeur vidéo avec
// gime_video_capture_drain (au fil du rendu).
void gime_video_set_capture_queue(uint8_t *mem, uint32_t slots);
void gime_video_capture_drain(void);
bool gime_video_capture_has_work(void);   // des lignes attendent dans la file (mesure VIDEO WORK)
// statistiques (trames publiées, abandonnées, attentes, sans tampon) et bascule de la file (commande
// série Q).
void gime_video_capture_stats(uint32_t *pub, uint32_t *drop, uint32_t *wait, uint32_t *nobuf, bool *queue_on);
void gime_video_capture_queue_enable(bool on);

// Diagnostic / banc d'essai : false = ignorer la capture (RAM vivante).
void gime_video_use_capture(bool on);

#endif
