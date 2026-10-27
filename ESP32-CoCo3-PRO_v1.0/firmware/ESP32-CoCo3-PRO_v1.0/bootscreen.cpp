// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Écran d'accueil et écran de veille (voir bootscreen.h).

#include "bootscreen.h"
#include "audio.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <strings.h>

extern const char *const g_fw_version_short;
extern void Log_Line(const char *s);
extern uint8_t Font_Row(uint8_t ch, uint8_t row);   // EmuMenu.cpp : police du GIME (8 x 8)

#ifdef ESP_PLATFORM
#include <Arduino.h>
#include <Preferences.h>
#include "esp_heap_caps.h"
#include "VGA.h"
extern VGA *vga;
extern bool SD_MMC_MountOK;
extern void Sd_Lock(void);
extern void Sd_Unlock(void);
extern uint8_t Input_AnyPressed(void);               // main.cpp : 0 = rien ; code de touche, ou $FF = bouton
extern uint32_t Input_Activity(void);                // main.cpp : change à chaque action (touche, souris, manette)
#define BS_ROOT "/sdcard/ESP-BOOT"
static uint32_t BS_Ms(void) { return (uint32_t)millis(); }
static void BS_Wait(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1)); }
static void *BS_Mem(size_t n) { void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM); return p ? p : malloc(n); }
static bool BS_SdOk(void) { return SD_MMC_MountOK; }
static void BS_Present(const uint8_t *fb)
{
  // le mode 320x240 4:3 a 326 points par ligne (mode1) : l'image est centrée, bords en noir
  if (!vga || !vga->dmaBuffer || vga->mode.hRes < 320 || vga->mode.hRes > 400 || vga->mode.vRes < 240) return;
  const int w = vga->mode.hRes, ox = (w - 320) / 2;
  for (int y = 0; y < 240; y++)
  {
    uint8_t *d = vga->dmaBuffer->getLineAddr8(y, vga->backBuffer);
    if (ox) memset(d, 0, ox);
    memcpy(d + ox, fb + y * 320, 320);
    if (w - 320 - ox > 0) memset(d + ox + 320, 0, w - 320 - ox);
  }
  vga->show();
}
#else
// Banc PC : BSROOT = dossier de la « SD », BSDUMP = dossier des images PPM, BSKEY = trame de la touche
static void Sd_Lock(void) {}
static void Sd_Unlock(void) {}
static uint32_t s_pc_ms = 0;
static uint32_t BS_Ms(void) { return s_pc_ms; }
static void BS_Wait(uint32_t ms) { s_pc_ms += ms ? ms : 1; }
static void *BS_Mem(size_t n) { return malloc(n); }
static bool BS_SdOk(void) { return true; }
static const char *BS_RootPc(void) { const char *r = getenv("BSROOT"); return r ? r : "."; }
#define BS_ROOT BS_RootPc()
static int s_pc_frames = 0;
static void BS_Present(const uint8_t *fb)
{
  s_pc_frames++;
  const char *d = getenv("BSDUMP"); const int every = getenv("BSEVERY") ? atoi(getenv("BSEVERY")) : 15;
  if (!d || s_pc_frames % every) return;
  char f[300]; snprintf(f, sizeof f, "%s/f%04d.ppm", d, s_pc_frames);
  FILE *o = fopen(f, "wb"); if (!o) return;
  fprintf(o, "P6 320 240 255\n");
  for (int i = 0; i < 320 * 240; i++)
  { const uint8_t v = fb[i]; const uint8_t px[3] = { (uint8_t)((v & 7) * 255 / 7), (uint8_t)(((v >> 3) & 7) * 255 / 7), (uint8_t)((v >> 6) * 255 / 3) }; fwrite(px, 1, 3, o); }
  fclose(o);
}
static uint8_t Input_AnyPressed(void) { const int k = getenv("BSKEY") ? atoi(getenv("BSKEY")) : 300; return s_pc_frames >= k && s_pc_frames < k + 3 ? 0x2C : 0; }
static uint32_t Input_Activity(void) { return Input_AnyPressed(); }
#endif

// ---------------------------------------------------------------- réglages
static uint8_t s_mode = BS_OFF, s_end = BS_END_KEY;
static uint8_t s_sv = SV_OFF, s_svk = SVK_LOGO, s_svs = 0;     // écran de veille (délai, contenu, son)
#ifdef ESP_PLATFORM
static void BS_Save(void)
{
  Preferences p; if (!p.begin("cc3s", false)) return;
  p.putUChar("mode", s_mode); p.putUChar("end", s_end);
  p.putUChar("sv", s_sv); p.putUChar("svk", s_svk); p.putUChar("svs", s_svs);
  p.end();
}
void BootScreen_Init(void)
{
  Preferences p;
  if (p.begin("cc3s", true))
  {
    s_mode = p.getUChar("mode", BS_OFF); s_end = p.getUChar("end", BS_END_KEY);
    s_sv = p.getUChar("sv", SV_OFF); s_svk = p.getUChar("svk", SVK_LOGO); s_svs = p.getUChar("svs", 0);
    p.end();
  }
  if (s_sv >= SV_COUNT) s_sv = SV_OFF;
  if (s_svk >= SVK_COUNT) s_svk = SVK_LOGO;
  if (SD_MMC_MountOK)                                // dossiers de l'écran d'accueil créés s'ils manquent
  {
    struct stat st;
    Sd_Lock();
    if (stat(BS_ROOT, &st) != 0) mkdir(BS_ROOT, 0777);
    if (stat(BS_ROOT "/SLIDES", &st) != 0) mkdir(BS_ROOT "/SLIDES", 0777);
    Sd_Unlock();
  }
  if (s_mode >= BS_MODES) s_mode = BS_OFF;
  if (s_end >= BS_ENDS) s_end = BS_END_KEY;
}
#else
static void BS_Save(void) {}
void BootScreen_Init(void) { const char *m = getenv("BSMODE"); s_mode = m ? (uint8_t)atoi(m) : BS_OFF; const char *e = getenv("BSEND"); s_end = e ? (uint8_t)atoi(e) : BS_END_KEY;
  const char *k = getenv("SVK"); s_svk = k ? (uint8_t)atoi(k) : SVK_LOGO; s_svs = getenv("SVS") != nullptr; }
#endif
bool    BootScreen_Wanted(void) { return s_mode != BS_OFF; }
uint8_t BootScreen_Mode(void) { return s_mode; }
void    BootScreen_SetMode(uint8_t m) { s_mode = m < BS_MODES ? m : BS_OFF; BS_Save(); }
uint8_t BootScreen_End(void) { return s_end; }
void    BootScreen_SetEnd(uint8_t e) { s_end = e < BS_ENDS ? e : BS_END_KEY; BS_Save(); }
uint8_t Saver_Delay(void) { return s_sv; }
void    Saver_SetDelay(uint8_t d) { s_sv = d < SV_COUNT ? d : SV_OFF; BS_Save(); }
uint8_t Saver_Kind(void) { return s_svk; }
void    Saver_SetKind(uint8_t k) { s_svk = k < SVK_COUNT ? k : SVK_LOGO; BS_Save(); }
bool    Saver_Sound(void) { return s_svs != 0; }
void    Saver_SetSound(bool on) { s_svs = on ? 1 : 0; BS_Save(); }

// ---------------------------------------------------------------- écran de veille : minuterie
// Appelé par le coeur vidéo à chaque tour : vrai quand rien n'a bougé (clavier, souris manette) depuis
// le délai choisi ; la minuterie repart à chaque action.
static uint32_t s_sv_last = 0, s_sv_act = 0;
bool Saver_Due(bool emu_running)
{
  const uint32_t now = BS_Ms(), a = Input_Activity();
  if (a != s_sv_act || !s_sv_last || !emu_running) { s_sv_act = a; s_sv_last = now ? now : 1; return false; }
  static const uint32_t k_min[SV_COUNT] = { 0, 5, 10, 20, 30 };
  return s_sv != SV_OFF && now - s_sv_last >= k_min[s_sv] * 60000u;
}
void Saver_Touch(void) { s_sv_last = BS_Ms() | 1u; s_sv_act = Input_Activity(); }

// ---------------------------------------------------------------- fichiers
static void BS_Path(char *out, size_t n, const char *rel) { snprintf(out, n, "%s/%s", BS_ROOT, rel); }
static bool BS_Exists(const char *rel)
{
  if (!BS_SdOk()) return false;
  char f[200]; BS_Path(f, sizeof f, rel);
  struct stat st;
  Sd_Lock(); const bool ok = stat(f, &st) == 0 && !S_ISDIR(st.st_mode) && st.st_size > 16; Sd_Unlock();
  return ok;
}
static int BS_Open(const char *rel) { char f[200]; BS_Path(f, sizeof f, rel); Sd_Lock(); const int fd = open(f, O_RDONLY); Sd_Unlock(); return fd; }
static int BS_Read(int fd, void *p, int n)            // par morceaux de 8 Ko sous le verrou de la SD
{
  int got = 0;
  while (got < n)
  {
    const int k = n - got < 8192 ? n - got : 8192;
    Sd_Lock(); const int r = (int)read(fd, (uint8_t *)p + got, k); Sd_Unlock();
    if (r <= 0) break;
    got += r;
  }
  return got;
}
static void BS_Seek(int fd, long off) { Sd_Lock(); lseek(fd, off, SEEK_SET); Sd_Unlock(); }
static void BS_Close(int fd) { if (fd >= 0) { Sd_Lock(); close(fd); Sd_Unlock(); } }

#define SLIDES_MAX 64
typedef char BsName[40];
static BsName *s_slides = nullptr, *s_gifs = nullptr;
static int s_nslides = 0, s_ngifs = 0;
// fichiers « *ext » du dossier sub (« » = /ESP-BOOT), triés par nom
static int BS_ListDir(const char *sub, const char *ext, BsName *&list)
{
  if (!BS_SdOk()) return 0;
  if (!list) list = (BsName *)BS_Mem(SLIDES_MAX * sizeof(BsName));
  if (!list) return 0;
  int n = 0;
  char d[200]; if (sub[0]) BS_Path(d, sizeof d, sub); else snprintf(d, sizeof d, "%s", BS_ROOT);
  Sd_Lock(); DIR *dir = opendir(d); Sd_Unlock();
  if (!dir) return 0;
  for (;;)
  {
    Sd_Lock(); struct dirent *e = readdir(dir); Sd_Unlock();
    if (!e) break;
    const size_t l = strlen(e->d_name);
    if (l < 5 || l >= 40 || e->d_name[0] == '.' || strcasecmp(e->d_name + l - 4, ext)) continue;
    if (n < SLIDES_MAX) strcpy(list[n++], e->d_name);
  }
  Sd_Lock(); closedir(dir); Sd_Unlock();
  for (int i = 1; i < n; i++)                        // tri par nom
    for (int j = i; j > 0 && strcasecmp(list[j - 1], list[j]) > 0; j--)
    { BsName t; strcpy(t, list[j]); strcpy(list[j], list[j - 1]); strcpy(list[j - 1], t); }
  return n;
}
static int BS_ListSlides(void) { return s_nslides = BS_ListDir("SLIDES", ".BMP", s_slides); }
static int BS_ListGifs(void)   { return s_ngifs = BS_ListDir("", ".GIF", s_gifs); }
void BootScreen_Describe(char *b, size_t n)
{
  const bool v = BS_Exists("SPLASH.CCV"), w = BS_Exists("SPLASH.WAV");
  const int g = v ? 0 : BS_ListGifs();
  const int sl = (v || g) ? 0 : BS_ListSlides();
  if (v) snprintf(b, n, "SPLASH.CCV%s", w ? " + SPLASH.WAV" : "");
  else if (g) snprintf(b, n, "%d GIF%s%s", g, g > 1 ? "s" : "", w ? " + SPLASH.WAV" : "");
  else if (sl) snprintf(b, n, "%d slide%s%s", sl, sl > 1 ? "s" : "", w ? " + SPLASH.WAV" : "");
  else snprintf(b, n, "nothing in /ESP-BOOT: logo%s", w ? " + WAV" : "");
}

// ---------------------------------------------------------------- image
static uint8_t *s_fb = nullptr;                    // 320 x 240
static inline uint8_t RGB(uint8_t r, uint8_t g, uint8_t b) { return (uint8_t)((r >> 5) | ((g >> 5) << 3) | (b & 0xC0)); }
static inline void Px(int x, int y, uint8_t c) { if ((unsigned)x < 320u && (unsigned)y < 240u) s_fb[y * 320 + x] = c; }
// Texte avec la police du GIME, agrandi s fois ; couleur par point (pour les effets)
static void Text(const char *t, int x, int y, int s, uint8_t (*col)(int px, int py, int ci), int ci0 = 0)
{
  for (int ci = 0; t[ci]; ci++)
    for (int r = 0; r < 8; r++)
    {
      const uint8_t bits = Font_Row((uint8_t)t[ci], (uint8_t)r);
      for (int b = 0; b < 8; b++)
        if (bits & (0x80 >> b))
          for (int dy = 0; dy < s; dy++) for (int dx = 0; dx < s; dx++)
          {
            const int px = x + (ci * 8 + b) * s + dx, py = y + r * s + dy;
            Px(px, py, col(px, py, ci0 + ci));
          }
    }
}
static int TextW(const char *t, int s) { return (int)strlen(t) * 8 * s; }

// ---------------------------------------------------------------- logo animé
static uint32_t s_anim_t = 0;                       // ms depuis le début
static const uint8_t k_rainbow[7] = { 0x07, 0x17, 0x3F, 0x38, 0xF8, 0xC0, 0xC7 };   // rouge orange jaune vert cyan bleu magenta
static uint8_t Col_Title(int px, int py, int ci)
{
  (void)ci;
  // dégradé or -> blanc, et un reflet qui balaie le titre toutes les 3 s
  const int sweep = (int)((s_anim_t % 3000u) * 480u / 3000u) - 80;
  const int d = px - py / 2 - sweep;
  if (d >= 0 && d < 14) return 0xFF;
  return py < 48 ? RGB(255, 255, 160) : (py < 56 ? RGB(255, 220, 64) : RGB(255, 160, 0));
}
static uint8_t Col_Pro(int px, int py, int ci) { (void)py; return k_rainbow[(unsigned)(ci * 2 + px / 6 + (int)(s_anim_t / 90)) % 7u]; }
static uint8_t Col_Sub(int, int, int) { return RGB(96, 224, 255); }
static uint8_t Col_Hint(int, int, int) { return RGB(200, 200, 200); }
static uint8_t Col_Ver(int, int, int) { return RGB(128, 128, 128); }

struct Star { int16_t x, y; uint8_t sp; };
static Star s_stars[70];
static void Logo_Frame(const char *hint)
{
  const uint32_t t = s_anim_t;
  memset(s_fb, 0, 320 * 240);
  // étoiles qui défilent
  for (int i = 0; i < 70; i++)
  {
    Star &s = s_stars[i];
    s.x -= s.sp; if (s.x < 0) { s.x += 320; s.y = (int16_t)((s.y * 73 + 41) % 240); }
    Px(s.x, s.y, s.sp > 2 ? 0xFF : (s.sp > 1 ? RGB(160, 160, 160) : RGB(96, 96, 96)));
  }
  // bandes arc-en-ciel en biais qui entrent par la gauche puis ondulent
  const int band_w = 7, y0 = 146;
  const int reach = t < 1200 ? (int)(t * 360u / 1200u) : 360;
  for (int x = 0; x < reach && x < 320; x++)
  {
    const int wave = (int)(3.0f * sinf((x + (int)(t / 8)) * 0.045f));
    for (int k = 0; k < 7; k++)
    {
      const int yy = y0 + k * band_w - x / 12 + wave;
      for (int j = 0; j < band_w - 1; j++) Px(x, yy + j, k_rainbow[k]);
    }
  }
  // titre : lettres qui tombent une par une (rebond), puis restent
  static const char *const k_t1 = "ESP32-CoCo3";
  const int s1 = 3, w1 = TextW(k_t1, s1), x1 = (320 - w1) / 2, ybase = 40;
  for (int i = 0; k_t1[i]; i++)
  {
    const int32_t ti = (int32_t)t - 300 - i * 90;
    if (ti < 0) continue;
    int dy = 0;
    if (ti < 600) { const float f = ti / 600.0f; dy = (int)(-90.0f * (1.0f - f) * fabsf(cosf(f * 7.0f))); }
    char c[2] = { k_t1[i], 0 };
    // ombre puis lettre
    Text(c, x1 + i * 8 * s1 + 2, ybase + dy + 2, s1, [](int, int, int) -> uint8_t { return 0x40; });
    Text(c, x1 + i * 8 * s1, ybase + dy, s1, Col_Title);
  }
  if (t > 1600)
  {
    static const char *const k_t2 = "PRO";
    const int s2 = 4, w2 = TextW(k_t2, s2), xp = (320 - w2) / 2, yp = 80;
    for (int oy = -2; oy <= 2; oy += 2) for (int ox = -2; ox <= 2; ox += 2)   // contour blanc
      if (ox || oy) Text(k_t2, xp + ox, yp + oy, s2, [](int, int, int) -> uint8_t { return 0xFF; });
    Text(k_t2, xp, yp, s2, Col_Pro);
  }
  if (t > 2200)
  {
    static const char *const k_sub = "COLOR COMPUTER 3 EMULATOR";
    Text(k_sub, (320 - TextW(k_sub, 1)) / 2, 202, 1, Col_Sub);
  }
  if (hint && t > 2600 && ((t / 600) & 1) == 0) Text(hint, (320 - TextW(hint, 1)) / 2, 216, 1, Col_Hint);
  char v[24]; snprintf(v, sizeof v, "v%s", g_fw_version_short);
  Text(v, 320 - TextW(v, 1) - 4, 230, 1, Col_Ver);
}

// ---------------------------------------------------------------- mélodie du logo (composée pour ce projet)
// Voix 1 : carré 25 % ; voix 2 : triangle (basse). Notes MIDI, 0 = silence. Croche = 105 ms (≈ 143
// bpm).
static const uint8_t k_mel[] = {
  72, 76, 79, 84, 83, 79, 76, 79,   77, 81, 84, 89, 88, 84, 81, 84,
  79, 83, 86, 91, 89, 86, 83, 86,   84, 0,  88, 0,  91, 91, 96, 0,
  0, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0, 0, 0 };
static const uint8_t k_bass[] = {
  48, 0, 55, 0, 48, 0, 55, 0,       53, 0, 60, 0, 53, 0, 60, 0,
  55, 0, 62, 0, 55, 0, 62, 0,       48, 0, 55, 0, 60, 0, 48, 0,
  0, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0, 0, 0 };
#define MEL_STEPS ((uint32_t)sizeof k_mel)
#define MEL_STEP_SMP 5040u                          // 105 ms à 48 kHz
static uint32_t s_syn_pos = 0;                      // échantillon courant
static uint32_t s_ph1 = 0, s_ph2 = 0;
static uint32_t s_note_inc[128];                    // pas de phase par note (calculé une fois : pow est lent)
static uint32_t Note_Inc(uint8_t n)
{
  if (!s_note_inc[69]) for (int k = 1; k < 128; k++) s_note_inc[k] = (uint32_t)(440.0 * pow(2.0, (k - 69) / 12.0) * 4294967296.0 / 48000.0);
  return n < 128 ? s_note_inc[n] : 0;
}
static void Synth_Fill(void)
{
  int16_t buf[256];
  uint32_t room = Audio_StreamRoom();
  while (room >= 256)
  {
    for (int i = 0; i < 256; i++)
    {
      const uint32_t step = (s_syn_pos / MEL_STEP_SMP) % MEL_STEPS, in = s_syn_pos % MEL_STEP_SMP;
      const uint8_t n1 = k_mel[step], n2 = k_bass[step];
      int32_t v = 0;
      if (n1)
      {
        s_ph1 += Note_Inc(n1);
        const int32_t env = in < 200 ? (int32_t)in * 120 : (int32_t)(24000 - (int32_t)in * 24000 / (int32_t)MEL_STEP_SMP / 2);   // volume x3
        v += (s_ph1 < 0x40000000u ? env * 3 : -env) / 4;   // carré 25 % sans composante continue
      }
      if (n2)
      {
        s_ph2 += Note_Inc(n2);
        const int32_t tri = (int32_t)((s_ph2 >> 16) & 0xFFFF);
        const int32_t tv = (tri < 32768 ? tri : 65535 - tri) - 16384;   // 16384..16383
        const int32_t env2 = in < 300 ? (int32_t)in * 60 : 18000;   // volume x3
        v += tv * env2 / 16384 / 2;
      }
      buf[i] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
      s_syn_pos++;
    }
    Audio_StreamWrite(buf, 256);
    room -= 256;
  }
}

// ---------------------------------------------------------------- musique WAV
static int s_wav = -1;
static long s_wav_data = 0, s_wav_len = 0, s_wav_left = 0;
static uint16_t s_wav_ch = 1, s_wav_bits = 16;
static uint32_t s_wav_rate = 22050, s_wav_frac = 0, s_wav_step = 0;
static int16_t s_wav_prev = 0, s_wav_cur = 0;
static uint8_t *s_wav_buf = nullptr; static int s_wav_bn = 0, s_wav_bp = 0;
static bool Wav_Open(void)
{
  s_wav = BS_Open("SPLASH.WAV");
  if (s_wav < 0) return false;
  uint8_t h[12];
  if (BS_Read(s_wav, h, 12) != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4)) { BS_Close(s_wav); s_wav = -1; return false; }
  long pos = 12; bool fmt = false;
  for (int guard = 0; guard < 32; guard++)
  {
    uint8_t c[8];
    if (BS_Read(s_wav, c, 8) != 8) break;
    const long len = (long)(c[4] | c[5] << 8 | c[6] << 16 | (uint32_t)c[7] << 24);
    pos += 8;
    if (!memcmp(c, "fmt ", 4))
    {
      uint8_t f[16]; if (len < 16 || BS_Read(s_wav, f, 16) != 16) break;
      const uint16_t tag = (uint16_t)(f[0] | f[1] << 8);
      s_wav_ch = (uint16_t)(f[2] | f[3] << 8);
      s_wav_rate = (uint32_t)(f[4] | f[5] << 8 | f[6] << 16 | (uint32_t)f[7] << 24);
      s_wav_bits = (uint16_t)(f[14] | f[15] << 8);
      fmt = tag == 1 && (s_wav_bits == 8 || s_wav_bits == 16) && s_wav_ch >= 1 && s_wav_ch <= 2 && s_wav_rate >= 4000 && s_wav_rate <= 96000;
      pos += 16; if (len > 16) { pos += len - 16 + (len & 1); BS_Seek(s_wav, pos); }
      else if (len & 1) { pos++; BS_Seek(s_wav, pos); }
    }
    else if (!memcmp(c, "data", 4)) { s_wav_data = pos; s_wav_len = len; break; }
    else { pos += len + (len & 1); BS_Seek(s_wav, pos); }
  }
  if (!fmt || !s_wav_len) { BS_Close(s_wav); s_wav = -1; Log_Line("[BOOTSCREEN] SPLASH.WAV : format non pris en charge (PCM 8/16 bits, mono/stereo)"); return false; }
  if (!s_wav_buf) s_wav_buf = (uint8_t *)BS_Mem(4096);
  if (!s_wav_buf) { BS_Close(s_wav); s_wav = -1; return false; }
  s_wav_left = s_wav_len; s_wav_bn = s_wav_bp = 0;
  s_wav_step = (uint32_t)(((uint64_t)s_wav_rate << 16) / 48000u);
  s_wav_frac = 0x10000;                              // force la lecture du premier échantillon
  char b[120]; snprintf(b, sizeof b, "[BOOTSCREEN] musique SPLASH.WAV : %lu Hz, %u bits, %s", (unsigned long)s_wav_rate, s_wav_bits, s_wav_ch == 1 ? "mono" : "stereo"); Log_Line(b);
  return true;
}
static bool Wav_Next(int16_t *v)                    // un échantillon d'origine (mono)
{
  const int fsz = (s_wav_bits / 8) * s_wav_ch;
  if (s_wav_bp + fsz > s_wav_bn)
  {
    if (s_wav_left < fsz) { s_wav_left = s_wav_len; BS_Seek(s_wav, s_wav_data); }   // en boucle
    int want = 4096 - (4096 % fsz); if (want > s_wav_left) want = (int)(s_wav_left - s_wav_left % fsz);
    s_wav_bn = BS_Read(s_wav, s_wav_buf, want); s_wav_bp = 0;
    if (s_wav_bn < fsz) return false;
    s_wav_left -= s_wav_bn;
  }
  const uint8_t *p = s_wav_buf + s_wav_bp; s_wav_bp += fsz;
  int32_t a, b;
  if (s_wav_bits == 8) { a = ((int32_t)p[0] - 128) << 8; b = s_wav_ch == 2 ? ((int32_t)p[1] - 128) << 8 : a; }
  else { a = (int16_t)(p[0] | p[1] << 8); b = s_wav_ch == 2 ? (int16_t)(p[2] | p[3] << 8) : a; }
  *v = (int16_t)((a + b) / 2);
  return true;
}
static void Wav_Fill(void)
{
  if (s_wav < 0) return;
  int16_t buf[256];
  uint32_t room = Audio_StreamRoom();
  while (room >= 256)
  {
    for (int i = 0; i < 256; i++)
    {
      while (s_wav_frac >= 0x10000) { s_wav_prev = s_wav_cur; if (!Wav_Next(&s_wav_cur)) { s_wav_cur = 0; } s_wav_frac -= 0x10000; }
      buf[i] = (int16_t)(s_wav_prev + (((int32_t)s_wav_cur - s_wav_prev) * (int32_t)s_wav_frac >> 16));   // interpolation linéaire
      s_wav_frac += s_wav_step;
    }
    Audio_StreamWrite(buf, 256);
    room -= 256;
  }
}

// ---------------------------------------------------------------- film CCV
static int s_ccv = -1; static uint16_t s_cw, s_chh, s_cfps10, s_cn; static uint8_t *s_cframe = nullptr; static uint32_t s_cidx = 0;
static bool Ccv_Open(void)
{
  s_ccv = BS_Open("SPLASH.CCV");
  if (s_ccv < 0) return false;
  uint8_t h[16];
  if (BS_Read(s_ccv, h, 16) != 16 || memcmp(h, "CCV1", 4)) { BS_Close(s_ccv); s_ccv = -1; return false; }
  s_cw = (uint16_t)(h[4] | h[5] << 8); s_chh = (uint16_t)(h[6] | h[7] << 8); s_cfps10 = (uint16_t)(h[8] | h[9] << 8); s_cn = (uint16_t)(h[10] | h[11] << 8);
  if (!s_cw || !s_chh || s_cw > 320 || s_chh > 240 || !s_cn || s_cfps10 < 10 || s_cfps10 > 600) { BS_Close(s_ccv); s_ccv = -1; Log_Line("[BOOTSCREEN] SPLASH.CCV : en-tete invalide"); return false; }
  if (!s_cframe) s_cframe = (uint8_t *)BS_Mem(320 * 240);
  if (!s_cframe) { BS_Close(s_ccv); s_ccv = -1; return false; }
  s_cidx = 0;
  char b[120]; snprintf(b, sizeof b, "[BOOTSCREEN] film SPLASH.CCV : %ux%u, %u images, %u.%u images/s", s_cw, s_chh, s_cn, s_cfps10 / 10, s_cfps10 % 10); Log_Line(b);
  return true;
}
static void Blit_Fit(const uint8_t *src, int w, int h)   // agrandi d'un facteur entier, centré, fond noir
{
  int s = 320 / w; if (240 / h < s) s = 240 / h; if (s < 1) s = 1;
  const int ox = (320 - w * s) / 2, oy = (240 - h * s) / 2;
  memset(s_fb, 0, 320 * 240);
  for (int y = 0; y < h; y++)
    for (int r = 0; r < s; r++)
    {
      uint8_t *d = s_fb + (oy + y * s + r) * 320 + ox;
      if (oy + y * s + r >= 240) break;
      const uint8_t *p = src + y * w;
      if (s == 1) memcpy(d, p, w); else for (int x = 0; x < w; x++) memset(d + x * s, p[x], s);
    }
}
static bool Ccv_Frame(void)
{
  const uint32_t n = (uint32_t)s_cw * s_chh;
  if (s_cidx >= s_cn) { s_cidx = 0; BS_Seek(s_ccv, 16); }
  if ((uint32_t)BS_Read(s_ccv, s_cframe, (int)n) != n) { s_cidx = s_cn; return false; }
  s_cidx++;
  Blit_Fit(s_cframe, s_cw, s_chh);
  return true;
}

// ---------------------------------------------------------------- diaporama BMP
static bool Bmp_Show(const char *name)
{
  char rel[60]; snprintf(rel, sizeof rel, "SLIDES/%s", name);
  const int fd = BS_Open(rel);
  if (fd < 0) return false;
  uint8_t h[54];
  bool ok = BS_Read(fd, h, 54) == 54 && h[0] == 'B' && h[1] == 'M';
  const uint32_t off = (uint32_t)(h[10] | h[11] << 8 | h[12] << 16 | (uint32_t)h[13] << 24);
  int32_t w = (int32_t)(h[18] | h[19] << 8 | h[20] << 16 | (uint32_t)h[21] << 24);
  int32_t hh = (int32_t)(h[22] | h[23] << 8 | h[24] << 16 | (uint32_t)h[25] << 24);
  const uint16_t bpp = (uint16_t)(h[28] | h[29] << 8);
  const uint32_t comp = (uint32_t)(h[30] | h[31] << 8 | h[32] << 16 | (uint32_t)h[33] << 24);
  const uint32_t hdrsz = (uint32_t)(h[14] | h[15] << 8);
  const bool topdown = hh < 0; if (hh < 0) hh = -hh;
  ok = ok && w > 0 && hh > 0 && w <= 4096 && hh <= 4096 && (bpp == 8 || bpp == 24 || bpp == 32) && (comp == 0 || (comp == 3 && bpp == 32));
  uint8_t pal[256][4];
  if (ok && bpp == 8) { BS_Seek(fd, 14 + hdrsz); BS_Read(fd, pal, 1024); }
  if (!ok) { BS_Close(fd); char b[120]; snprintf(b, sizeof b, "[BOOTSCREEN] %s : BMP non pris en charge (8, 24 ou 32 bits non compresse)", name); Log_Line(b); return false; }
  // mise à l'échelle : réduire pour tenir dans 320 x 240 (au plus proche), ou agrandir d'un facteur
  // entier
  int dw, dh;
  if (w <= 320 && hh <= 240) { int s = 320 / w; if (240 / hh < s) s = 240 / hh; dw = w * s; dh = hh * s; }
  else if (w * 240 >= hh * 320) { dw = 320; dh = (int)((int64_t)hh * 320 / w); }
  else { dh = 240; dw = (int)((int64_t)w * 240 / hh); }
  if (dw < 1) dw = 1;
  if (dh < 1) dh = 1;
  const int ox = (320 - dw) / 2, oy = (240 - dh) / 2;
  const uint32_t stride = ((uint32_t)w * bpp / 8 + 3u) & ~3u;
  uint8_t *row = (uint8_t *)BS_Mem(stride);
  if (!row) { BS_Close(fd); return false; }
  memset(s_fb, 0, 320 * 240);
  int last_sy = -1;
  for (int dy = 0; dy < dh; dy++)
  {
    const int sy = (int)((int64_t)dy * hh / dh);
    if (sy != last_sy)
    {
      const int fy = topdown ? sy : hh - 1 - sy;
      BS_Seek(fd, (long)off + (long)fy * stride);
      if ((uint32_t)BS_Read(fd, row, (int)stride) != stride) break;
      last_sy = sy;
    }
    uint8_t *d = s_fb + (oy + dy) * 320 + ox;
    for (int dx = 0; dx < dw; dx++)
    {
      const int sx = (int)((int64_t)dx * w / dw);
      uint8_t r, g, b;
      if (bpp == 8) { const uint8_t *q = pal[row[sx]]; b = q[0]; g = q[1]; r = q[2]; }
      else { const uint8_t *q = row + sx * (bpp / 8); b = q[0]; g = q[1]; r = q[2]; }
      d[dx] = RGB(r, g, b);
    }
  }
  free(row);
  BS_Close(fd);
  return true;
}

// ---------------------------------------------------------------- GIF animés
// Décodeur écrit pour le projet : GIF87a / GIF89a, palettes globale et locales LZW, entrelacement,
// transparence, effacement (disposal) 1 / 2 / 3, délais
#define GIF_MAXW 640
#define GIF_MAXH 480
struct GifRd { int fd; uint8_t *buf; int n, p; bool eof; };
static GifRd s_gr = { -1, nullptr, 0, 0, false };
static inline int Gr_Byte(void)
{
  if (s_gr.p >= s_gr.n)
  {
    if (s_gr.eof) return -1;
    s_gr.n = BS_Read(s_gr.fd, s_gr.buf, 4096); s_gr.p = 0;
    if (s_gr.n <= 0) { s_gr.eof = true; s_gr.n = 0; return -1; }
  }
  return s_gr.buf[s_gr.p++];
}
static bool Gr_Skip(int n) { while (n-- > 0) if (Gr_Byte() < 0) return false; return true; }
static bool Gr_SkipBlocks(void) { for (;;) { const int l = Gr_Byte(); if (l <= 0) return l == 0; if (!Gr_Skip(l)) return false; } }
static uint16_t *s_lzw_pre = nullptr; static uint8_t *s_lzw_suf = nullptr, *s_lzw_stk = nullptr;
static uint8_t *s_gcanvas = nullptr, *s_gprev = nullptr;          // image du GIF (BBGGGRRR), sauvegarde (disposal 3)
static uint8_t s_gpal[256], s_lpal[256];
static uint16_t s_gw, s_gh; static uint8_t s_gbg; static bool s_ggct;
static long s_gfirst = 0;                                          // position du premier bloc (pour reboucler)
static uint8_t s_gdisp = 0; static uint16_t s_gdelay = 10; static int s_gtrans = -1;
static uint16_t s_px, s_py, s_pw, s_ph; static uint8_t s_pdisp = 0; // image précédente (pour son effacement)
static int s_gfile = -1;                                           // fichier en cours dans s_gifs
static uint32_t s_gfile_t0 = 0, s_gnext = 0; static int s_gframes = 0;
static void Gif_Close(void) { BS_Close(s_gr.fd); s_gr.fd = -1; }
static bool Gif_Open(const char *name)
{
  Gif_Close();
  if (!s_gr.buf) s_gr.buf = (uint8_t *)BS_Mem(4096);
  if (!s_lzw_pre) { s_lzw_pre = (uint16_t *)BS_Mem(4096 * 2); s_lzw_suf = (uint8_t *)BS_Mem(4096); s_lzw_stk = (uint8_t *)BS_Mem(4097); }
  if (!s_gcanvas) { s_gcanvas = (uint8_t *)BS_Mem(GIF_MAXW * GIF_MAXH); s_gprev = (uint8_t *)BS_Mem(GIF_MAXW * GIF_MAXH); }
  if (!s_gr.buf || !s_lzw_pre || !s_lzw_suf || !s_lzw_stk || !s_gcanvas || !s_gprev) return false;
  s_gr.fd = BS_Open(name); s_gr.n = s_gr.p = 0; s_gr.eof = false;
  if (s_gr.fd < 0) return false;
  uint8_t h[13];
  for (int i = 0; i < 13; i++) { const int c = Gr_Byte(); if (c < 0) { Gif_Close(); return false; } h[i] = (uint8_t)c; }
  if (memcmp(h, "GIF8", 4)) { Gif_Close(); return false; }
  s_gw = (uint16_t)(h[6] | h[7] << 8); s_gh = (uint16_t)(h[8] | h[9] << 8);
  char b[120];
  if (!s_gw || !s_gh || s_gw > GIF_MAXW || s_gh > GIF_MAXH)
  { snprintf(b, sizeof b, "[BOOTSCREEN] %s : %ux%u, trop grand (640x480 au plus)", name, s_gw, s_gh); Log_Line(b); Gif_Close(); return false; }
  s_ggct = (h[10] & 0x80) != 0; s_gbg = h[11];
  memset(s_gpal, 0, sizeof s_gpal);
  if (s_ggct)
  {
    const int n = 2 << (h[10] & 7);
    for (int i = 0; i < n; i++) { const int r = Gr_Byte(), g = Gr_Byte(), bl = Gr_Byte(); if (bl < 0) { Gif_Close(); return false; } s_gpal[i] = RGB((uint8_t)r, (uint8_t)g, (uint8_t)bl); }
  }
  s_gfirst = 13 + (s_ggct ? 3 * (2 << (h[10] & 7)) : 0);
  memset(s_gcanvas, 0, (size_t)s_gw * s_gh);
  s_pdisp = 0; s_pw = 0; s_gdisp = 0; s_gdelay = 10; s_gtrans = -1;
  snprintf(b, sizeof b, "[BOOTSCREEN] GIF %s : %ux%u", name, s_gw, s_gh); Log_Line(b);
  return true;
}
static void Gif_Rewind(void)
{
  BS_Seek(s_gr.fd, s_gfirst); s_gr.n = s_gr.p = 0; s_gr.eof = false;
  memset(s_gcanvas, 0, (size_t)s_gw * s_gh); s_pdisp = 0; s_pw = 0; s_gtrans = -1; s_gdisp = 0;
}
// LZW d'une image ; écrit dans le canevas (palette pal, transparence, entrelacement)
static bool Gif_Lzw(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, bool inter, const uint8_t *pal)
{
  const int mcs = Gr_Byte();
  if (mcs < 2 || mcs > 8) return false;
  const int clear = 1 << mcs, eoi = clear + 1;
  int csize = mcs + 1, next = clear + 2, prev = -1;
  uint32_t bits = 0; int nb = 0, blk = 0;
  uint8_t first = 0;
  uint32_t pix = 0; const uint32_t total = (uint32_t)w * h;
  int row = 0, pass = 0, col = 0;
  static const uint8_t k_start[4] = { 0, 4, 2, 1 }, k_step[4] = { 8, 8, 4, 2 };
  for (int i = 0; i < clear; i++) { s_lzw_pre[i] = 0xFFFF; s_lzw_suf[i] = (uint8_t)i; }
  for (;;)
  {
    while (nb < csize)
    {
      if (blk == 0) { blk = Gr_Byte(); if (blk <= 0) return pix >= total; }
      const int c = Gr_Byte(); if (c < 0) return false;
      bits |= (uint32_t)c << nb; nb += 8; blk--;
    }
    int code = (int)(bits & ((1u << csize) - 1)); bits >>= csize; nb -= csize;
    if (code == clear) { csize = mcs + 1; next = clear + 2; prev = -1; continue; }
    if (code == eoi) break;
    int sp = 0, c = code;
    if (prev < 0) { if (code >= clear) return false; first = (uint8_t)code; s_lzw_stk[sp++] = first; }
    else
    {
      if (code > next || code >= 4096) return false;
      if (code == next) { s_lzw_stk[sp++] = first; c = prev; }   // cas KwKwK
      while (c >= clear && sp < 4096) { s_lzw_stk[sp++] = s_lzw_suf[c]; c = s_lzw_pre[c]; }
      if (c >= clear) return false;
      first = s_lzw_suf[c]; s_lzw_stk[sp++] = first;
      if (next < 4096) { s_lzw_pre[next] = (uint16_t)prev; s_lzw_suf[next] = first; next++; if (next == (1 << csize) && csize < 12) csize++; }
    }
    prev = code;
    while (sp > 0)                                   // points de la pile -> canevas
    {
      const uint8_t idx = s_lzw_stk[--sp];
      if (pix < total)
      {
        const int yy = y0 + row, xx = x0 + col;
        if ((int)idx != s_gtrans && xx < s_gw && yy < s_gh) s_gcanvas[yy * s_gw + xx] = pal[idx];
        pix++;
        if (++col >= w)
        {
          col = 0;
          if (!inter) row++;
          else { row += k_step[pass]; while (row >= h && pass < 3) { pass++; row = k_start[pass]; } }
        }
      }
    }
  }
  return Gr_SkipBlocks();
}
// Image suivante du GIF dans le canevas ; faux à la fin du fichier
static bool Gif_NextFrame(void)
{
  // effacement de l'image précédente
  if (s_pw && s_pdisp == 2) for (int y = s_py; y < s_py + s_ph && y < s_gh; y++) memset(s_gcanvas + y * s_gw + s_px, 0, (s_px + s_pw <= s_gw) ? s_pw : s_gw - s_px);
  else if (s_pw && s_pdisp == 3) memcpy(s_gcanvas, s_gprev, (size_t)s_gw * s_gh);
  for (int guard = 0; guard < 4096; guard++)
  {
    const int c = Gr_Byte();
    if (c < 0 || c == 0x3B) return false;
    if (c == 0x21)
    {
      const int lab = Gr_Byte();
      if (lab == 0xF9)
      {
        const int len = Gr_Byte(); if (len < 4) return false;
        const int fl = Gr_Byte(), d0 = Gr_Byte(), d1 = Gr_Byte(), ti = Gr_Byte();
        if (ti < 0 || !Gr_Skip(len - 4)) return false;
        s_gdisp = (uint8_t)((fl >> 2) & 7); s_gdelay = (uint16_t)(d0 | d1 << 8); s_gtrans = (fl & 1) ? ti : -1;
        if (!Gr_SkipBlocks()) return false;
      }
      else if (!Gr_SkipBlocks()) return false;
      continue;
    }
    if (c != 0x2C) return false;
    uint8_t d[9];
    for (int i = 0; i < 9; i++) { const int v = Gr_Byte(); if (v < 0) return false; d[i] = (uint8_t)v; }
    const uint16_t x = (uint16_t)(d[0] | d[1] << 8), y = (uint16_t)(d[2] | d[3] << 8), w = (uint16_t)(d[4] | d[5] << 8), h = (uint16_t)(d[6] | d[7] << 8);
    const uint8_t *pal = s_gpal;
    if (d[8] & 0x80)
    {
      const int n = 2 << (d[8] & 7);
      for (int i = 0; i < n; i++) { const int r = Gr_Byte(), g = Gr_Byte(), b = Gr_Byte(); if (b < 0) return false; s_lpal[i] = RGB((uint8_t)r, (uint8_t)g, (uint8_t)b); }
      pal = s_lpal;
    }
    if (s_gdisp == 3) memcpy(s_gprev, s_gcanvas, (size_t)s_gw * s_gh);
    if (!Gif_Lzw(x, y, w, h, (d[8] & 0x40) != 0, pal)) return false;
    s_px = x; s_py = y; s_pw = (x < s_gw) ? w : 0; s_ph = h; s_pdisp = s_gdisp;
    s_gtrans = -1; s_gdisp = 0;                      // l'extension ne vaut que pour une image
    return true;
  }
  return false;
}
// canevas -> image de l'écran (agrandi d'un facteur entier ou réduit, centré)
static void Gif_Blit(void)
{
  if (s_gw <= 320 && s_gh <= 240) { Blit_Fit(s_gcanvas, s_gw, s_gh); return; }
  int dw, dh;
  if ((int)s_gw * 240 >= (int)s_gh * 320) { dw = 320; dh = s_gh * 320 / s_gw; } else { dh = 240; dw = s_gw * 240 / s_gh; }
  const int ox = (320 - dw) / 2, oy = (240 - dh) / 2;
  memset(s_fb, 0, 320 * 240);
  for (int y = 0; y < dh; y++)
  {
    const uint8_t *src = s_gcanvas + (y * s_gh / dh) * s_gw;
    uint8_t *dst = s_fb + (oy + y) * 320 + ox;
    for (int x = 0; x < dw; x++) dst[x] = src[x * s_gw / dw];
  }
}
// Tour de la boucle : image suivante quand son délai est écoulé ; chaque GIF joue au moins 6 s (repris
// depuis le début s'il est court), puis le suivant. Faux si aucun GIF n'est lisible.
static bool Gif_Step(uint32_t now)
{
  if (s_gfile < 0 || (s_gr.fd < 0))
  {
    for (int tries = 0; tries < s_ngifs; tries++)
    {
      s_gfile = (s_gfile + 1) % s_ngifs;
      if (Gif_Open(s_gifs[s_gfile])) { s_gfile_t0 = now; s_gnext = now; s_gframes = 0; break; }
    }
    if (s_gr.fd < 0) return false;
  }
  if ((int32_t)(now - s_gnext) < 0) return true;
  if (!Gif_NextFrame())
  {
    if (s_gframes > 0 && (now - s_gfile_t0 < 6000u || s_ngifs == 1)) { Gif_Rewind(); s_gframes = 0; if (!Gif_NextFrame()) { Gif_Close(); return true; } }
    else { Gif_Close(); return true; }               // fichier suivant au prochain tour
  }
  s_gframes++;
  Gif_Blit();
  const uint16_t d = s_gdelay < 2 ? 10 : s_gdelay;   // délai 0 ou 1 : 100 ms (comme les navigateurs)
  s_gnext = now + d * 10u;
  return true;
}

// ---------------------------------------------------------------- boucle
static void BS_Show(bool saver);
void BootScreen_Run(void) { BS_Show(false); }
void Saver_Run(void) { BS_Show(true); }
static void BS_Show(bool saver)
{
  if (!s_fb) s_fb = (uint8_t *)BS_Mem(320 * 240);
  if (!s_fb) return;
  uint8_t kind = 0;                                 // 0 logo, 1 film, 2 diaporama, 3 GIF, 4 écran noir
  bool wav = false;
  const bool sd = saver ? s_svk == SVK_SD : s_mode == BS_SD;
  if (saver && s_svk == SVK_BLANK) kind = 4;
  else if (sd)
  {
    if (Ccv_Open()) kind = 1;
    else if (BS_ListGifs()) { kind = 3; s_gfile = -1; }
    else if (BS_ListSlides()) kind = 2;
    wav = BS_Exists("SPLASH.WAV") && Wav_Open();
  }
  if (saver && !s_svs) wav = false;
  const bool music = (wav || kind == 0) && (!saver || s_svs);
  if (music && !Audio_StreamBegin()) { wav = false; }
  for (int i = 0; i < 70; i++) { s_stars[i].x = (int16_t)((i * 97) % 320); s_stars[i].y = (int16_t)((i * 53 + 17) % 240); s_stars[i].sp = (uint8_t)(1 + (i % 3)); }
  s_syn_pos = 0; s_ph1 = s_ph2 = 0;
  {
    char b[120]; snprintf(b, sizeof b, "[%s] %s%s ; fin : %s", saver ? "VEILLE" : "BOOTSCREEN", kind == 1 ? "film" : kind == 2 ? "diaporama" : kind == 3 ? "GIF" : kind == 4 ? "ecran noir" : "logo",
                          wav ? " + SPLASH.WAV" : (music ? " + melodie" : ""),
                          saver ? "touche, souris ou manette" : s_end == BS_END_KEY ? "touche" : s_end == BS_END_5 ? "5 s" : s_end == BS_END_10 ? "10 s" : "30 s");
    Log_Line(b);
  }
  static const uint32_t k_end_ms[BS_ENDS] = { 0, 5000, 10000, 30000 };
  const uint32_t t0 = BS_Ms();
  uint32_t next = t0, slide_t = 0; int slide = -1;
  const uint32_t period = kind == 1 ? 10000u / s_cfps10 : (kind == 3 ? 10u : 33u);
  uint8_t key = 0;
  // une touche tenue au démarrage (ex. SHIFT) ne compte pas : il faut un nouvel appui
  if (!saver) for (int i = 0; i < 300 && Input_AnyPressed(); i++) BS_Wait(10);
  const uint32_t act0 = Input_Activity();
  bool blank_done = false;
  for (;;)
  {
    const uint32_t now = BS_Ms();
    s_anim_t = now - t0;
    if (kind == 4)                                  // écran noir : dessiné une fois, puis on attend
    {
      if (!blank_done) { memset(s_fb, 0, 320 * 240); BS_Present(s_fb); BS_Present(s_fb); blank_done = true; }
      if (Input_Activity() != act0) { key = Input_AnyPressed(); if (!key) key = 0xFE; break; }
      BS_Wait(50);
      continue;
    }
    bool show = true;
    if (kind == 3) { const uint32_t was = s_gnext; if (!Gif_Step(now)) kind = 0; else show = s_gnext != was; }
    if (kind == 1) { if (!Ccv_Frame()) kind = 0; }
    else if (kind == 2)
    {
      if (slide < 0 || now - slide_t >= 4000u)
      {
        slide = (slide + 1) % s_nslides; slide_t = now;
        if (!Bmp_Show(s_slides[slide]) && s_nslides == 1) kind = 0;
      }
    }
    if (kind == 0) Logo_Frame(saver ? nullptr : (s_end == BS_END_KEY ? "PRESS ANY KEY OR BUTTON" : nullptr));
    if (show) BS_Present(s_fb);
    if (wav) Wav_Fill(); else if (music) Synth_Fill();
    key = Input_AnyPressed();
    if (key) break;
    if (saver && Input_Activity() != act0) { key = 0xFE; break; }   // la souris a bougé, etc.
    if (!saver && s_end != BS_END_KEY && now - t0 >= k_end_ms[s_end]) break;
    next += period;
    const uint32_t after = BS_Ms();
    if ((int32_t)(next - after) > 0) BS_Wait(next - after); else { next = after; BS_Wait(1); }
  }
  // attendre que la touche soit relâchée (elle n'arrive pas au CoCo) ; le son s'éteint
  for (int i = 0; i < 300 && Input_AnyPressed(); i++) { if (wav) Wav_Fill(); else if (music) Synth_Fill(); BS_Wait(10); }
  if (music) Audio_StreamEnd();
  BS_Close(s_wav); s_wav = -1;
  BS_Close(s_ccv); s_ccv = -1;
  Gif_Close();
  memset(s_fb, 0, 320 * 240);
  BS_Present(s_fb); BS_Present(s_fb);               // les deux tampons en noir
  if (!saver && key == 0x29) { extern void Autorun_Cancel(void); Autorun_Cancel(); }   // ESC : sans AUTORUN
  char b[80]; snprintf(b, sizeof b, "[%s] fin apres %lu s (%s)", saver ? "VEILLE" : "BOOTSCREEN", (unsigned long)((BS_Ms() - t0) / 1000u),
                       saver ? "reveil" : key == 0x29 ? "ESC : AUTORUN annule" : key ? "touche" : "delai");
  Log_Line(b);
}
