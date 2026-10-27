// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Boucle principale : coeur CPU, E/S $FF00-$FFFF, PIA, son, disquettes, réglages, commandes série.
// Auteur : Luc Dion 2026, inspiré de ESP32-CoCo2 de Cedric Beaudoin.

#include <Arduino.h>
#include <cstring>

// Correctif : sur ce package core/libs (esp32-arduino-libs) liblwip.a est précompilé

#include "lwip/netif.h"
#include "lwip/ip6.h"
extern "C" int lwip_hook_ip6_input(struct pbuf *p, struct netif *inp) __attribute__((weak));
extern "C" int lwip_hook_ip6_input(struct pbuf *p, struct netif *inp)
{
  if (ip6_addr_isany_val(inp->ip6_addr[0].u_addr.ip6))
  {
    // Pas d'adresse Link-Local -> on vide le buffer pour qu'il ne soit pas accepté sur l'interface
    // d'entrée.
    pbuf_free(p);
    return 1;
  }
  return 0;
}

// Core ESP32-Arduino: setup/loop ("loopTask") n'a que 8 Ko de stack par défaut
SET_LOOP_TASK_STACK_SIZE(32 * 1024);  // 32 Ko au lieu de 8 Ko par défaut
#include "mc6809.hpp"
#include <new>   // new de placement (objet CPU en RAM interne)
#include "esp_timer.h"
#include "esp_system.h"

#include "main.h"
#include "modem.h"   // modem Hayes par le Wi-Fi
#include "webui.h"   // page web (carte SD vue du PC)
#include "autorun.h" // AUTORUN (frappe automatique au démarrage)
#include "bootscreen.h" // écran d'accueil
#include "health.h"  // état matériel de l'ESP32
#include "gime.h"
#include "gime_video.h"
#include "drivewire.h" // disques virtuels DriveWire (port Becker)
#include "audio.h"     // sortie son horodatée (I2S PDM)
#include "cassette.h"  // magnétophone virtuel (.CAS / .WAV sur la SD)
#include "rs232.h"     // RS-232 Pak (ACIA 6551)
#include "bitbang.h"   // port série du CoCo (bit banger)
#include "cc3net.h"    // Wi-Fi et heure NTP
#include "netshare.h"  // dossier partagé d'un PC
// le port Becker va au PC sauf si le bit banger (mode 5) utilise déjà la connexion : un seul des deux
// à la fois (sinon le bit banger vole les réponses).
static inline bool Becker_Net(void) { return NetDw_Active() && g_bb_mode != BB_NETDW; }

// ---------------------------------------------------------------------------
// journal en mémoire pour les testeurs sans port série
// ---------------------------------------------------------------------------
static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;   // verrou de l'anneau
// les 2 derniers Ko du journal sont aussi écrits en mémoire RTC, qui survit à un plantage / watchdog
// watchdog / redémarrage logiciel (pas une coupure de courant)
#define LOG_RTC_TAIL 2048
#define LOG_RTC_MAGIC 0xC0C03A11u
RTC_NOINIT_ATTR static char s_rtc_tail[LOG_RTC_TAIL];
RTC_NOINIT_ATTR static uint32_t s_rtc_head;
RTC_NOINIT_ATTR static uint32_t s_rtc_magic;
// dernière étape « à risque » en cours (mémoire RTC, survit au chien de garde) : affichée au démarrage
// suivant après un plantage sans message.
RTC_NOINIT_ATTR volatile uint32_t g_rtc_step;
#define STEP_IDLE        0
#define STEP_MENU        1   // menu F12 ouvert
#define STEP_MENU_EXIT   2   // sortie du menu F12 (retour à l'émulation)
#define STEP_STATE_SAVE  5
#define STEP_STATE_LOAD  6
#define STEP_HOTKEY      7   // F5 / F8
#define STEP_REBOOT      8   // redémarrage demandé (menu R, Ctrl+Alt+Suppr, mise à jour)
static bool g_boot_after_requested_reboot = false;
static const char *Step_Name(uint32_t s)
{
  switch (s)
  {
    case STEP_IDLE: return "emulation";
    case STEP_MENU: return "menu F12";
    case STEP_MENU_EXIT: return "sortie du menu F12";
    case STEP_STATE_SAVE: return "Save State : sauvegarde";
    case STEP_STATE_LOAD: return "Save State : chargement";
    case STEP_HOTKEY: return "F5 / F8";
    case STEP_REBOOT: return "redemarrage demande";
    default: return "?";
  }
}
static char *g_prev_tail = nullptr;      // copie (PSRAM) de la queue d'avant le redémarrage
static size_t g_prev_tail_len = 0;
// affichage « par morceaux » des rapports (U, W, F9...) : la tâche loop partage le coeur 1 avec le
// coeur CPU du CoCo (même priorité)
static volatile bool s_log_pace = false;
static volatile TaskHandle_t s_log_pace_task = nullptr;
struct LogPace
{
  LogPace() { s_log_pace_task = xTaskGetCurrentTaskHandle(); s_log_pace = true; }
  ~LogPace() { s_log_pace = false; }
};
class LogTee : public Print
{
public:
  static constexpr size_t RING = 64 * 1024;
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override
  {
    Serial.write(b, n);
    if (!ring_ || !n) return n;
    char ts[16];
    portENTER_CRITICAL(&s_log_mux);
    for (size_t i = 0; i < n; i++)
    {
      if (lineStart_)
      {
        const uint32_t sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        const int k = snprintf(ts, sizeof ts, "[%02lu:%02lu:%02lu] ", (unsigned long)(sec / 3600), (unsigned long)(sec / 60 % 60), (unsigned long)(sec % 60));
        for (int j = 0; j < k; j++) put_((uint8_t)ts[j]);
        lineStart_ = false;
      }
      if (b[i] == '\r') continue;
      put_(b[i]);
      if (b[i] == '\n') lineStart_ = true;
    }
    portEXIT_CRITICAL(&s_log_mux);
    if (s_log_pace && xTaskGetCurrentTaskHandle() == s_log_pace_task)
    {
      for (size_t i = 0; i < n; i++) if (b[i] == '\n') paceLines_++;
      if (paceLines_ >= 2) { paceLines_ = 0; vTaskDelay(1); }
    }
    return n;
  }
  void begin(void)
  {
    if (!ring_) ring_ = (uint8_t *)heap_caps_malloc(RING, MALLOC_CAP_SPIRAM);
    // récupère la queue RTC du démarrage précédent (si elle est valide) puis la remet à zéro pour ce
    // démarrage.
    if (s_rtc_magic == LOG_RTC_MAGIC && esp_reset_reason() != ESP_RST_POWERON)
    {
      const uint32_t h = s_rtc_head;
      const size_t n = h < LOG_RTC_TAIL ? h : LOG_RTC_TAIL;
      g_prev_tail = (char *)heap_caps_malloc(n + 1, MALLOC_CAP_SPIRAM);
      if (g_prev_tail)
      {
        const uint32_t start = h < LOG_RTC_TAIL ? 0 : h % LOG_RTC_TAIL;
        for (size_t i = 0; i < n; i++) g_prev_tail[i] = s_rtc_tail[(start + i) % LOG_RTC_TAIL];
        g_prev_tail[n] = 0;
        g_prev_tail_len = n;
      }
    }
    s_rtc_head = 0;
    s_rtc_magic = LOG_RTC_MAGIC;
  }
  // Copie le contenu de l'anneau, dans l'ordre, dans dst (au plus RING octets).
  size_t snapshot(uint8_t *dst)
  {
    if (!ring_ || !dst) return 0;
    portENTER_CRITICAL(&s_log_mux);
    const size_t n = full_ ? RING : head_;
    const size_t start = full_ ? head_ : 0;
    for (size_t i = 0; i < n; i++) dst[i] = ring_[(start + i) % RING];
    portEXIT_CRITICAL(&s_log_mux);
    return n;
  }
private:
  inline void put_(uint8_t c)
  {
    ring_[head_++] = c; if (head_ == RING) { head_ = 0; full_ = true; }
    if (s_rtc_magic == LOG_RTC_MAGIC) s_rtc_tail[(s_rtc_head++) % LOG_RTC_TAIL] = (char)c;
  }
  uint8_t *ring_ = nullptr;
  size_t head_ = 0;
  bool full_ = false;
  bool lineStart_ = true;
  uint8_t paceLines_ = 0;
};
LogTee LogOut;
extern char g_dw_name[DW_DRIVES + 2][256];
bool DW_Mount(uint8_t d, const char *path);
void DW_Unmount(uint8_t d);
extern volatile bool g_dw_boot_request;
bool VHD_Enabled(void);
uint8_t VHD_Read(uint16_t a);
void VHD_Write(uint16_t a, uint8_t v);
uint8_t g_pia_ddr[4] = { 0x00, 0xFF, 0xF8, 0xFE };   // DDR $FF00, $FF02, $FF22 ; [3] = $FF20
// sortie réelle du port A du PIA1 : une broche en entrée (DDR = 0) est tirée au niveau haut (comme
// pia6821 de MAME) : CNA, comparateur, bit banger.
static inline uint8_t Pia1A_Out(void);
// souris USB -> port joystick du CoCo (voir Mouse_*)
volatile uint64_t g_bb_ticks = 0;           // temps CPU (tics 14,31818 MHz, sans l Overclock CPU) : bit banger
volatile uint64_t g_emu_ticks = 0;          // temps émulé en tics GIME (14,31818 MHz)
volatile bool g_audio_realtime = false;     // Overclock MACHINE -> son horodaté en temps réel
uint8_t g_mouse_mode = 0;                   // 0 OFF, 1 droit basse rés., 2 droit haute rés
                                            // 5 deux ports basse rés., 6 deux ports haute rés.
static inline uint8_t Mouse_Mask(void)      // ports de la souris (bit 0 droit, bit 1 gauche)
{
  return g_mouse_mode == 0 ? 0 : (g_mouse_mode <= 2 ? 1 : (g_mouse_mode <= 4 ? 2 : 3));
}
static inline uint8_t Mouse_ModeFor(uint8_t mask, bool hi)
{
  return mask == 0 ? 0 : mask == 1 ? (hi ? 2 : 1) : mask == 2 ? (hi ? 4 : 3) : (hi ? 6 : 5);
}
uint8_t g_mouse_sens = 4;                   // sensibilité 1-8 (4 = 2 unités par point de souris)
bool    g_mouse_hires_pref = false;         // résolution gardée quand le port est OFF
uint8_t g_kbjoy_port = 0;                   // manette au clavier : 0 OFF, 1 droite, 2 gauche
uint8_t g_kbjoy_set  = 0;                   // 0 flèches+Espace/Ctrl droit, 1 pavé numérique, 2 WASD+Espace/F
uint8_t g_kbjoy_set2 = 0;                   // jeu de touches du port gauche (g_kbjoy_set : port droit)
static void Mouse_DacWrite(uint8_t ff20);
static void Mouse_MuxChanged(void);
static bool Mouse_Comparator(uint8_t mux, bool *result);
#include "EmuMenu.h"  // struct JoystickDeviceConfig + API JoyWizard_* partagées avec le menu…

#include "ESP32S3vga.h"
#include <GfxWrapper.h>
#include "FreeMonoBoldOblique24pt7b.h"
#include "FreeSerif24pt7b.h"
#include "FreeMono9pt7b.h"

//---------------------------- USB STACK
// ESP32-Coco2 : prise en charge du hub USB natif on-board (librairie EspUsbHost)
// en remplacement de Soft-Host logiciel (ESP32-USB-Soft-Host) qui
// simulait le clavier et les 2 manettes via des GPIOs.
// A faire dans Arduino IDE avant de compiler
// - 6c : la librairie EspUsbHost 2.7.0 (TANAKA Masayuki, licence MIT,
// https://github.com/tanakamasayuki/EspUsbHost) est maintenant FOURNIE dans
// le croquis (dossier src/EspUsbHost), avec un filtre des interfaces HID
// (voir « CoCo3 6c » dans src/EspUsbHost/EspUsbHost.cpp). La version
// installée par le Library manager n'est plus utilisée (inutile de la
// désinstaller).
// Activer l'option "USB Host Hub Support" dans la configuration du board
// (sdkconfig), pour permettre au hub USB 3-en-1 externe (clavier + 2
// manettes) de fonctionner sur le port USB natif de l'ESP32-S3.
// Utiliser un hub USB 3-en-1 self-powered, les bus-powered risquent
// d'êtreu instables à énumérer 3 périphériques simultanément.
// Le port USB natif de l'ESP32-S3 est hardwired (GPIO 19 / GPIO 20)
// aucune définition de pins nécessaire.

#include "src/EspUsbHost/EspUsbHost.h"   // copie corrigée dans le croquis
static void Mem_DmaStats(uint32_t &freeB, uint32_t &largestB, uint32_t &minB);

// Structure conservée pour ne pas toucher au reste du fichier (mapping clavier CoCo, manettes, etc.)
// qui référence USB_DEV_CONTROL. Etait auparavant fournie par ESP32-USB-Soft-Host.h.
typedef struct
{
  uint8_t ScanArray[256];
  uint8_t USB_CoCo_Key_Array[8];
  uint8_t JOY1_X_AXIS, JOY1_Y_AXIS, JOY1_BUTT1, JOY1_BUTT2;
  uint8_t JOY2_X_AXIS, JOY2_Y_AXIS, JOY2_BUTT1, JOY2_BUTT2;
  uint8_t PORT_KEYBOARD, PORT_JOY1, PORT_JOY2;  // conservés par compatibilité (non utilisés avec EspUsbHost)
} USB_DEVICES_CTRL;

USB_DEVICES_CTRL USB_DEV_CONTROL;

// EspUsbHost *UsbHost* est maintenant un pointeur (voir Setup_USB) plutôt qu'un objet global
EspUsbHost *UsbHost = nullptr;

// un clavier USB est en ligne (menu F12, ligne USB DEVICES : KBD vert / rouge)
bool Usb_KeyboardOnline(void) { return UsbHost && UsbHost->cocoKeyboardOnline(); }

// résumé USB pour le menu F12 (USB DEVICES > USB STATUS) « HUB + 3 DEVICES, 7/8 CH » (canaux du
// contrôleur hôte occupés / disponibles)
int Usb_StalledPorts(char *desc, size_t n)
{
  static uint32_t s_last = 0;
  static uint8_t s_prevAddr[8], s_prevMask[8];   // masques des ports suspects au relevé précédent
  static int s_count = 0;
  static char s_desc[96] = "";
  if (!UsbHost) return 0;
  if (s_last && millis() - s_last < 2000) { if (desc && n) snprintf(desc, n, "%s", s_desc); return s_count; }
  s_last = millis() | 1u;
  uint8_t curAddr[8] = {}, curMask[8] = {};
  int count = 0;
  size_t pos = 0;
  s_desc[0] = 0;
  EspUsbHostDeviceInfo dev[8];
  const size_t nd = UsbHost->getDevices(dev, 8);
  for (size_t i = 0, h = 0; i < nd && i < 8; i++)
  {
    if (!dev[i].isHub) continue;
    EspUsbHostHubInfo hub;
    if (!UsbHost->getHubInfo(dev[i].address, hub)) continue;
    uint8_t mask = 0;
    for (uint8_t port = 1; port <= hub.portCount && port <= 8; port++)
    {
      uint16_t st = 0, ch = 0;
      if (UsbHost->getHubPortStatus(dev[i].address, port, st, ch) && (st & 0x0001) && !(st & 0x0002)) mask |= (uint8_t)(1u << (port - 1));
    }
    uint8_t prev = 0;
    for (int k = 0; k < 8; k++) if (s_prevAddr[k] == dev[i].address) prev = s_prevMask[k];
    const uint8_t both = mask & prev;
    for (uint8_t port = 1; port <= 8; port++)
      if ((both >> (port - 1)) & 1)
      {
        count++;
        if (pos < sizeof s_desc - 24) pos += snprintf(s_desc + pos, sizeof s_desc - pos, "%shub adr %u port %u", pos ? " ; " : "", dev[i].address, port);
      }
    if (h < 8) { curAddr[h] = dev[i].address; curMask[h] = mask; h++; }
  }
  memcpy(s_prevAddr, curAddr, sizeof curAddr);
  memcpy(s_prevMask, curMask, sizeof curMask);
  s_count = count;
  if (desc && n) snprintf(desc, n, "%s", s_desc);
  return count;
}

// rend true s'il y a un appareil sans canal (ligne en jaune dans le menu)
bool Usb_Summary(char *b, size_t n)
{
  if (!UsbHost) { snprintf(b, n, "USB host not started"); return false; }
  EspUsbHostDeviceInfo dev[8];
  const size_t cnt = UsbHost->getDevices(dev, 8);
  unsigned hubs = 0, others = 0;
  for (size_t i = 0; i < cnt && i < 8; i++) { if (dev[i].isHub) hubs++; else others++; }
  const unsigned ch = (unsigned)UsbHost->estimatedHcdChannelCount();
  const unsigned mx = (unsigned)UsbHost->maxEndpointChannelCount();
  if (!cnt) { snprintf(b, n, "NO DEVICE"); return false; }
  // appareils branchés qui n'ont pas pu démarrer, interfaces sans canal
  int nc = Usb_StalledPorts(nullptr, 0);
  for (size_t i = 0; i < cnt && i < 8; i++)
  {
    if (dev[i].isHub) continue;
    EspUsbHostInterfaceInfo itf[8];
    const size_t ni = UsbHost->getInterfaces(dev[i].address, itf, 8);
    bool miss = false;
    for (size_t k = 0; k < ni && k < 8; k++)
      if (!itf[k].claimed && itf[k].claimAttempted && (itf[k].claimResult == ESP_ERR_NOT_SUPPORTED || itf[k].claimResult == ESP_ERR_NO_MEM)) miss = true;
    if (miss) nc++;
  }
  if (nc) snprintf(b, n, "%s%u DEV, %u/%u CH, %d NOT STARTED", hubs ? "HUB + " : "", others, ch, mx, nc);   // canal ou RAM
  else snprintf(b, n, "%s%u DEVICE%s, %u/%u CH", hubs ? "HUB + " : "", others, others == 1 ? "" : "S", ch, mx);
  return nc != 0;
}

// Adresses USB assignées dynamiquement aux 2 manettes lors de leur connexion au hub (0 = aucune
// manette assignée à ce port pour l'instant)
uint8_t USB_JOY1_ADDRESS = 0;
uint8_t USB_JOY2_ADDRESS = 0;

extern USB_DEVICES_CTRL USB_DEV_CONTROL;

extern void Setup_USB(void);

// Prototypes (ces fonctions sont définies plus bas dans le fichier, mais sont appelées par les
// callbacks EspUsbHost déclarées avant elles).
void fillKeysStruct(void);
void UpdateKeyMap(uint8_t * Data);
void UpdateJoyMap(uint8_t * Data, uint8_t usbNum);

//-----------------------------------------------------------

// Core ESP32-Arduino 3.x (ESP-IDF 5.x) : l'objet global "GPIO" (accès direct GPIO.out1_w1ts.val, etc.)
// n'est plus disponible tel quel
#include "soc/gpio_reg.h"

#define LED_PORT_ANODE 45
#define LED_PORT_CATHODE 46

#define LED_PORT_CATHODE_HI() REG_WRITE(GPIO_OUT1_W1TS_REG, (1 << (LED_PORT_CATHODE - 32)));   // Set HIGH
#define LED_PORT_CATHODE_LOW() REG_WRITE(GPIO_OUT1_W1TC_REG, (1 << (LED_PORT_CATHODE - 32)));  // Set LOW

#define LED_PORT_ANODE_HI() REG_WRITE(GPIO_OUT1_W1TS_REG, (1 << (LED_PORT_ANODE - 32)));   // Set HIGH
#define LED_PORT_ANODE_LOW() REG_WRITE(GPIO_OUT1_W1TC_REG, (1 << (LED_PORT_ANODE - 32)));  // Set LOW

void inline DISKETTE_LED_ON(void)
{
  LED_PORT_CATHODE_HI();
  LED_PORT_ANODE_LOW();
}
void inline DISKETTE_LED_OFF(void)
{
  LED_PORT_CATHODE_LOW();
  LED_PORT_ANODE_HI();
}
void inline DISKETTE_LED_FLOAT(void)
{
  LED_PORT_CATHODE_LOW();
  LED_PORT_ANODE_LOW();
}

uint8_t uint8_t_VarGlobal;
uint8_t DEBUGloop1, DEBUGloop2;
void Debug1Toggle(void)
{
  if ((DEBUGloop1 & 0b00000001) == 0)
  {
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << DEBUG1));
  }
  else
  {
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << DEBUG1));
  }
  DEBUGloop1++;
}
void Debug2Toggle(void)
{
  if ((DEBUGloop2 & 0b00000001) == 0)
  {
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << DEBUG2));
  }
  else
  {
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << DEBUG2));
  }
  DEBUGloop2++;
}

void Debug2Hi(void)
{
  REG_WRITE(GPIO_OUT_W1TS_REG, (1 << DEBUG2));
}

void Debug2Low(void)
{
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << DEBUG2));
}

uint8_t ResetVectors[16] = {0xA6, 0x81, 0x01, 0x00, 0x01, 0x03, 0x01, 0x0F,
	0x01, 0x0C, 0x01, 0x06, 0x01, 0x09, 0xA0, 0x27};

bool CPU_in_WAIT_STATE = false;
volatile bool g_cpu_parked = false;   // la tâche CPU est arrêtée entre deux rafales (menus)

volatile bool g_resync_cpu_clock = true;

volatile uint32_t g_vsync_frame_count = 0;// V-SYNC
static uint16_t g_gime_div_q8 = 256;    // Overclock CPU, cycles GIME par cycle CPU (Q8, 256 = 1:1)
static uint16_t g_gime_div_max = 256;   // plafond du multiplicateur (~5 MHz)
static constexpr bool g_batch_enabled = true;   // exécution par lots
// phase des couleurs d'artefact (PMODE 4 sur composite). Sur un vrai CoCo, elle dépend de l'instant du
// démarrage (on faisait RESET jusqu'à avoir les bonnes couleurs)
uint8_t g_artifact_swap = 0;
volatile uint32_t g_vga_frame_us = 16667;  // période de trame VGA mesurée (µs)
volatile int64_t  g_vsync_us = 0;          // instant (esp_timer) du dernier VSYNC VGA
volatile int32_t  g_phase_err_lines = 0;   // écart de phase trame émulée / trame VGA (lignes, commande W)

File file;

#ifndef ISR_CORE
char text_buffer[150];
#endif

// Core ESP32-Arduino 3.x (ESP-IDF 5.5)
uint8_t *MENU_Backup = nullptr;
uint8_t *MENU_BackupPage2 = nullptr;
#define DISK_RAM_CAP_ALLOC (40u * 18u * 256u + 256u)   // voir couche disque
extern uint8_t *RAM_Disk0, *RAM_Disk1, *RAM_Disk2, *RAM_Disk3;
#ifdef PSRAM_EMU
// rom[] : ROM interne 32 Ko ($8000-$FFFF, image coco3[])
uint8_t *rom = nullptr;
static inline uint8_t Pia1A_Out(void) { return (uint8_t)(rom[ROM_FF20] | (uint8_t)~g_pia_ddr[3]); }
uint8_t *memory = nullptr;
uint32_t g_ram_capacity = GIME_RAM_SIZE;   // RAM physique allouée (512 Ko..4 Mo)
uint32_t g_ram_selected = GIME_RAM_SIZE;   // taille choisie (menu E / config.ccc)
// images de la VRAM CoCo 2 figées au FS (voir VDG_Snapshot).
#define VDG_SNAP_SIZE 6144u          // PMODE 4 = 6 Ko, le plus grand mode VDG
static uint8_t          *s_vdg_snap[3] = { nullptr, nullptr, nullptr };
uint8_t *cart_rom = nullptr;
// port d'extension. Sans Multi-Pak : une cartouche (slot 0). Avec le Multi-Pak : 4 slots, $FF7F bits
// 1-0 = SCS (slot des E/S $FF40-$FF5F) bits 5-4 = CTS (slot de la ROM en $C000 et de la ligne CART)
SlotConfig g_slotcfg     = { 0, 0, { SLOT_FDC, SLOT_EMPTY, SLOT_EMPTY, SLOT_EMPTY }, { 1, 1, 1, 1 }, { "", "", "", "" } };
SlotConfig g_slotcfg_run = { 0, 0, { SLOT_FDC, SLOT_EMPTY, SLOT_EMPTY, SLOT_EMPTY }, { 1, 1, 1, 1 }, { "", "", "", "" } };
bool    g_cart_clock = false;          // CART = Q en service (PIA1 CB1 + GIME EI0)
bool    g_ff40_fdc   = true;           // $FF40-$FF5F -> contrôleur de disquette
uint8_t g_mpi_reg    = 0;              // $FF7F (Multi-Pak)
bool    g_o90_present = false;         // Orchestra-90 en service
static uint8_t *g_slot_rom[MPI_SLOTS] = { nullptr, nullptr, nullptr, nullptr };   // ROM de chaque slot (32 Ko, PSRAM)
// cartouches à banques (RoboCop, Predator, Game Master Cartridge) image entière en PSRAM, fenêtre de
// 16 Ko en $C000 (en miroir), banque choisie par une écriture en $FF40 (slot SCS)
static uint8_t *g_slot_big[MPI_SLOTS]   = { nullptr, nullptr, nullptr, nullptr };
static uint32_t g_slot_bsize[MPI_SLOTS] = { 0, 0, 0, 0 };   // 0 = pas de banques
static uint8_t  g_slot_bank[MPI_SLOTS]  = { 0, 0, 0, 0 };
static uint8_t  g_scs_slot = 0;
static bool     g_scs_banked = false, g_scs_gmc = false;
bool            g_gmc_present = false;
// CoCo PSG : deux banques de 8 Ko ($FF5A -> $C000, $FF5B -> $E000 ; bit 7 = SRAM de la carte, non
// émulée)
bool            g_cpsg_present = false;
static bool     g_scs_cpsg = false;
static uint8_t  g_cpsg_bank[2] = { 0, 1 };   // « jeu déjà lancé » : image linéaire en $C000-$FFFF
static uint8_t  g_cpsg_ctrl = 0, g_cpsg_latch = 0, g_cpsg_reg[16];
#define CART_BANKED_MAX (512u * 1024u)
#else
#error "La phase GIME exige PSRAM_EMU (512 Ko de RAM physique en PSRAM)"
#endif

void Disk_Init(void);
static bool g_rom_internal = false;
void AllocatePSRAMBuffers(void)
{
  MENU_Backup      = (uint8_t *)heap_caps_malloc(640*250, MALLOC_CAP_SPIRAM);
  MENU_BackupPage2 = (uint8_t *)heap_caps_malloc(640*250, MALLOC_CAP_SPIRAM);
  // tampons fixes des lecteurs (simple face jusqu'à 40 pistes) les images plus grandes : tampon
  // dynamique ou lecture à la volée.
  RAM_Disk0        = (uint8_t *)heap_caps_malloc(DISK_RAM_CAP_ALLOC, MALLOC_CAP_SPIRAM);
  RAM_Disk1        = (uint8_t *)heap_caps_malloc(DISK_RAM_CAP_ALLOC, MALLOC_CAP_SPIRAM);
  RAM_Disk2        = (uint8_t *)heap_caps_malloc(DISK_RAM_CAP_ALLOC, MALLOC_CAP_SPIRAM);
  RAM_Disk3        = (uint8_t *)heap_caps_malloc(DISK_RAM_CAP_ALLOC, MALLOC_CAP_SPIRAM);
  Disk_Init();
#ifdef PSRAM_EMU
  // ROM interne 32 Ko (BASIC, DECB, vecteurs, ombre E/S) en RAM interne s'il reste de la place (> 96
  // Ko contigus), sinon PSRAM.
  rom = nullptr;
  BOOTLOG("[DIAG] RAM interne : plus grand bloc libre %u o avant la ROM\n", (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  // ROM remise en PSRAM. Mesuré sur la carte (4c -> 4d, MATRIX en BASIC) : aucun gain en RAM interne ;
  // ces 32 Ko servent mieux à l'IRAM du coeur 6809 compilé en -O2 (voir mc6809*.cpp).
  if (false)
    rom = (uint8_t *)heap_caps_malloc(32769, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  g_rom_internal = (rom != nullptr);
  if (!rom) rom = (uint8_t *)heap_caps_malloc(32769, MALLOC_CAP_SPIRAM);
  // RAM physique allouée à la capacité maximale possible (4 Mo sinon 2 Mo, sinon 512 Ko)
  {
    static const uint32_t k_try[3] = { GIME_RAM_MAX, 2u * 1024u * 1024u, GIME_RAM_SIZE };
    for (int i = 0; i < 3 && memory == nullptr; i++)
    {
      memory = (uint8_t *)heap_caps_malloc(GIME_RAM_ALLOC_FOR(k_try[i]), MALLOC_CAP_SPIRAM);
      if (memory) g_ram_capacity = k_try[i];
    }
  }
  cart_rom = (uint8_t *)heap_caps_malloc(GIME_CART_ROM_SIZE, MALLOC_CAP_SPIRAM);
  g_slot_rom[0] = cart_rom;                         // slots 1-3 du Multi-Pak
  for (int i = 1; i < MPI_SLOTS; i++)
  {
    g_slot_rom[i] = (uint8_t *)heap_caps_malloc(GIME_CART_ROM_SIZE, MALLOC_CAP_SPIRAM);
    if (g_slot_rom[i]) memset(g_slot_rom[i], 0xFF, GIME_CART_ROM_SIZE);
  }
#endif
  for (int i = 0; i < 3; i++)          // images VRAM CoCo 2 (3 x 6 Ko)
  {
    s_vdg_snap[i] = (uint8_t *)heap_caps_malloc(VDG_SNAP_SIZE, MALLOC_CAP_SPIRAM);
  }
  if (!s_vdg_snap[0] || !s_vdg_snap[1] || !s_vdg_snap[2]) s_vdg_snap[0] = nullptr;
  // images de la VRAM des modes CoCo3 (3 x ~57 Ko en PSRAM). file de 32 lignes en RAM interne pour la
  // capture (voir gime_video.cpp)
  gime_video_set_capture_queue((uint8_t *)heap_caps_malloc(32 * 256, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT), 32);
  gime_video_set_capture_buffers((uint8_t *)heap_caps_malloc(GIME_CAP_BUF_SIZE, MALLOC_CAP_SPIRAM),
                                 (uint8_t *)heap_caps_malloc(GIME_CAP_BUF_SIZE, MALLOC_CAP_SPIRAM),
                                 (uint8_t *)heap_caps_malloc(GIME_CAP_BUF_SIZE, MALLOC_CAP_SPIRAM));

  const char *names[]    = {"MENU_Backup", "MENU_BackupPage2", "RAM_Disk0", "RAM_Disk1", "RAM_Disk2", "RAM_Disk3", "rom", "memory", "cart_rom"};
  uint8_t   *ptrs[]      = {MENU_Backup, MENU_BackupPage2, RAM_Disk0, RAM_Disk1, RAM_Disk2, RAM_Disk3, rom, memory, cart_rom};
  bool        allOk = true;
  for (uint8_t i = 0; i != 9; i++)
  {
    if (ptrs[i] == nullptr)
    {
      allOk = false;
    }
  }
  if (allOk)
  {
    memset(memory, 0xFF, GIME_RAM_ALLOC_FOR(g_ram_capacity));
    memset(rom, 0x00, 32769);
    memset(cart_rom, 0xFF, GIME_CART_ROM_SIZE);
    gime_init(memory, rom, cart_rom);
  }
  else
  {
    while(1) delay(1);
  }
}

SpecialFunctionStruct sf;
// les modes CoCo 1/2 passent par la palette du GIME, comme sur un vrai CoCo3 (BASIC la charge avec les
// couleurs du VDG ; les versions « CoCo3 » de jeux CoCo 2 la reprogramment, ex. Spidercide)
#include "gime_video.h"
// Registre resté à sa valeur BASIC (table composite chargée au démarrage, ou table RGB de la commande
// RGB)
static const uint8_t k_pal_basic_cmp[16] = { 0x12,0x24,0x0B,0x07,0x3F,0x1F,0x09,0x26, 0x00,0x12,0x00,0x3F, 0x00,0x12,0x00,0x26 };
static const uint8_t k_pal_basic_rgb[16] = { 0x12,0x36,0x09,0x24,0x3F,0x10,0x2D,0x26, 0x00,0x12,0x00,0x3F, 0x00,0x12,0x00,0x26 };
static const uint8_t k_pal_vdg[16] = { 0x1C,0xFC,0x03,0xE0, 0xFF,0x5B,0xE3,0xE8,    // 4 couleurs CSS 0 / 1 (et SG)
                                       0x00,0x1C,0x00,0xFF,                        // 2 couleurs CSS 0 / 1
                                       0x00,0x1C,0x00,0xE8 };                      // texte CSS 0 / 1
// moniteur RGB. Un vrai CoCo3 décode alors TOUS les registres en RGB
static inline bool LegacyPalUntouched(void)
{
  bool cmp = true, rgb = true;
  for (int i = 0; i < 16; i++)
  {
    const uint8_t p = gime.palette[i] & 0x3F;
    if (p != k_pal_basic_cmp[i]) cmp = false;
    if (p != k_pal_basic_rgb[i]) rgb = false;
  }
  return cmp || rgb;
}
static inline uint8_t LegacyPal(uint8_t idx)
{
  idx &= 15;
  const uint8_t p = gime.palette[idx] & 0x3F;
  if (gime_video_monitor_rgb())
    return LegacyPalUntouched() ? k_pal_vdg[idx] : gime_video_color(p);
  if (p == k_pal_basic_cmp[idx] || p == k_pal_basic_rgb[idx]) return k_pal_vdg[idx];
  return gime_video_color(p);
}
// Blocs semi-graphiques : teintes de l'ancienne table (cc2_VDG_MAP), sauf la couleur 5 (vert foncé par
// erreur -> turquoise des modes graphiques).
static const uint8_t k_pal_vdg_sg[8] = { 0x1C,0xFC,0x03,0xE0,0xFF,0x5B,0xA3,0xF1 };
static inline uint8_t LegacyPalSG(uint8_t idx)
{
  idx &= 7;
  const uint8_t p = gime.palette[idx] & 0x3F;
  if (gime_video_monitor_rgb())   // même règle que LegacyPal (ci-dessous)
    return LegacyPalUntouched() ? k_pal_vdg_sg[idx] : gime_video_color(p);
  if (p == k_pal_basic_cmp[idx] || p == k_pal_basic_rgb[idx]) return k_pal_vdg_sg[idx];
  return gime_video_color(p);
}
static inline void LegacyPal4(uint8_t *c)
{
  const uint8_t b = (sf.Coco2GraphicMode & 0x08) ? 4 : 0;
  for (int i = 0; i < 4; i++) c[i] = LegacyPal(b + i);
}
static inline void LegacyPal2(uint8_t *c)
{
  const uint8_t b = (sf.Coco2GraphicMode & 0x08) ? 10 : 8;
  c[0] = LegacyPal(b); c[1] = LegacyPal(b + 1);
}
static uint8_t s_txt_dark = 0x00, s_txt_light = 0x1C;   // texte : couleurs de la trame en cours

// Commandes série (W, U, J) et commandes injectées par F9. Appelée depuis loop, non bloquante.
static constexpr bool g_pia_sync_emulated = true;   // FS/HS du PIA0 = balayage GIME émulé
static void PIA0_FieldSync(void);
static void PIA0_FieldSyncRise(void);
static void PIA0_HSync(void);

void PollDebugCommands(void)
{
  static char cmdBuf[16];
  static uint8_t cmdLen = 0;
  extern const char *volatile g_cmd_inject;   // commandes injectées (F9)
  if (!Serial.available() && !(g_cmd_inject && *g_cmd_inject)) return;
  LogPace pace;   // rapports affichés par morceaux (le coeur CPU garde son temps)
  while (Serial.available() || (g_cmd_inject && *g_cmd_inject))
  {
    char c = (g_cmd_inject && *g_cmd_inject) ? *g_cmd_inject++ : (char)Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (cmdLen > 0 && (cmdBuf[0] == 'J' || cmdBuf[0] == 'j'))
      {
        // « J » : état des manettes / souris
        extern volatile int16_t g_mouse_x, g_mouse_y; extern volatile uint8_t g_mouse_buttons;
        LogOut.printf("[JOY] mode souris %u ; droite X=%u Y=%u B1=%u B2=%u ; gauche X=%u Y=%u B1=%u B2=%u ; souris x=%d y=%d boutons=%u\n",
                      g_mouse_mode, USB_DEV_CONTROL.JOY1_X_AXIS, USB_DEV_CONTROL.JOY1_Y_AXIS, USB_DEV_CONTROL.JOY1_BUTT1, USB_DEV_CONTROL.JOY1_BUTT2,
                      USB_DEV_CONTROL.JOY2_X_AXIS, USB_DEV_CONTROL.JOY2_Y_AXIS, USB_DEV_CONTROL.JOY2_BUTT1, USB_DEV_CONTROL.JOY2_BUTT2,
                      g_mouse_x, g_mouse_y, g_mouse_buttons);
      }
      else if (cmdLen > 0 && (cmdBuf[0] == 'U' || cmdBuf[0] == 'u'))
      {
        // « U » : détail USB (canaux du contrôleur, interfaces de chaque périphérique)
        extern void Usb_Report(bool); Usb_Report(true);
      }
      else if (cmdLen > 0 && (cmdBuf[0] == 'W' || cmdBuf[0] == 'w'))
      {
        // « W » : état complet (aussi dans le journal F9)
        const dw_stats_t *st = dw_get_stats();
        LogOut.printf("[DW] port Becker %s, secteurs lus %lu, ecrits %lu, erreurs %lu, commandes inconnues %lu (derniere $%02X)\n",
                      dw_port_enabled() ? "actif" : "inactif", (unsigned long)st->reads, (unsigned long)st->writes,
                      (unsigned long)st->errors, (unsigned long)st->unknown, st->last_op);
        { extern uint32_t g_vd_stat_reads, g_vd_stat_hits, g_vd_stat_sd; extern uint64_t g_vd_stat_sd_us;
          extern uint32_t g_vd_stat_direct;
          LogOut.printf("[VD] lectures %lu, dans le cache %lu, acces SD %lu (dont %lu directs), moyenne %lu us par acces SD\n",
                        (unsigned long)g_vd_stat_reads, (unsigned long)g_vd_stat_hits, (unsigned long)g_vd_stat_sd, (unsigned long)g_vd_stat_direct,
                        (unsigned long)(g_vd_stat_sd ? g_vd_stat_sd_us / g_vd_stat_sd : 0)); }
        extern volatile uint32_t g_perf_load_x10, g_perf_peak_x10, g_perf_lost_ps, g_perf_capped_ps, g_perf_hz, g_perf_target_hz,
                                 g_perf_line_x10, g_perf_io_x10, g_perf_ips, g_perf_cpi, g_perf_wait_x10;
        LogOut.printf("[PERF] coeur CPU : charge %lu.%lu %% (pic %lu.%lu %%), %lu Hz atteints / %lu Hz vises, cycles perdus %lu/s, rafales plafonnees %lu/s, ROM en %s\n",
                      (unsigned long)(g_perf_load_x10 / 10), (unsigned long)(g_perf_load_x10 % 10),
                      (unsigned long)(g_perf_peak_x10 / 10), (unsigned long)(g_perf_peak_x10 % 10),
                      (unsigned long)g_perf_hz, (unsigned long)g_perf_target_hz,
                      (unsigned long)g_perf_lost_ps, (unsigned long)g_perf_capped_ps, g_rom_internal ? "RAM interne" : "PSRAM");
        LogOut.printf("[PERF] dont copies de lignes %lu.%lu %%, E/S %lu.%lu %% ; %lu instructions/s, %lu cycles ESP32 par instruction (hors lignes/E-S) ; RAM interne libre %u o (plus grand bloc %u o)\n",
                      (unsigned long)(g_perf_line_x10 / 10), (unsigned long)(g_perf_line_x10 % 10),
                      (unsigned long)(g_perf_io_x10 / 10), (unsigned long)(g_perf_io_x10 % 10),
                      (unsigned long)g_perf_ips, (unsigned long)g_perf_cpi,
                      (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        if (g_oc_mode == OC_CPU)
          LogOut.printf("[PERF] Overclock CPU adaptatif : multiplicateur x%.2f (plafond x%.2f)\n", g_gime_div_q8 / 256.0, g_gime_div_max / 256.0);
        LogOut.printf("[PERF] 6809 en attente (SYNC/CWAI) %lu.%lu %% de ses cycles\n",
                      (unsigned long)(g_perf_wait_x10 / 10), (unsigned long)(g_perf_wait_x10 % 10));
        { // version du GIME et cadence du timer (un jeu peut s'y caler)
          extern volatile uint32_t g_cpu_sleeps, g_cpu_forced_sleeps;
          static uint32_t s_gt0 = 0, s_gn0 = 0, s_gs0 = 0, s_gf0 = 0;
          const uint32_t ms = millis(), nt = gime_raster.n_tmr, ns = g_cpu_sleeps, nf = g_cpu_forced_sleeps, dt = ms - s_gt0;
          const uint32_t tv = ((uint32_t)(gime.reg[GIME_TMR_MSB] & 0x0F) << 8) | gime.reg[GIME_TMR_LSB];
          const bool ok = s_gt0 && dt;
          LogOut.printf("[GIME] version %s (le timer compte valeur + %u) ; timer : valeur %lu, horloge %s, %lu echeances/s ; FIRQ timer %s\n",
                        gime_is_1987() ? "1987" : "1986", (unsigned)gime_timer_adjust, (unsigned long)tv,
                        (gime.reg[GIME_INIT1] & 0x20) ? "279 ns" : "lignes (HSYNC)",
                        ok ? (unsigned long)((uint64_t)(nt - s_gn0) * 1000u / dt) : 0ul,
                        (gime.reg[GIME_FIRQENR] & GIME_INT_TMR) ? "permise" : "non");
          LogOut.printf("[PERF] sommeil du 6809 a l heure : %lu/s, forces (garde-fou) %lu/s (mesure entre deux W)\n",
                        ok ? (unsigned long)((uint64_t)(ns - s_gs0) * 1000u / dt) : 0ul, ok ? (unsigned long)((uint64_t)(nf - s_gf0) * 1000u / dt) : 0ul);
          s_gt0 = ms; s_gn0 = nt; s_gs0 = ns; s_gf0 = nf; }
        g_perf_peak_x10 = 0;
        { extern volatile uint32_t g_vid_frames, g_vid_render_cc, g_vid_render_max, g_vid_drain_cc, g_vsync_frame_count;
          static uint32_t s_vf0 = 0, s_vs0 = 0, s_rc0 = 0, s_dc0 = 0; static uint64_t s_t0 = 0;
          const uint64_t t = esp_timer_get_time(); const uint32_t vf = g_vid_frames, vs = g_vsync_frame_count, rc = g_vid_render_cc, dc = g_vid_drain_cc;
          if (s_t0 && vf != s_vf0)
          {
            const double sec = (double)(t - s_t0) / 1e6, n = (double)(vf - s_vf0);
            LogOut.printf("[VIDEO] coeur video : %.1f images GIME rendues/s pour %.1f VSYNC/s ; rendu moyen %.2f ms, max %.2f ms (trame VGA 16,6 ms) ; dont vidage file %.2f ms/image\n",
                          n / sec, (vs - s_vs0) / sec, (rc - s_rc0) / n / 240000.0, g_vid_render_max / 240000.0, (dc - s_dc0) / n / 240000.0);
          }
          else LogOut.println("[VIDEO] (retaper W : mesure entre deux W, mode GIME seulement)");
          LogOut.printf("[PHASE] ecart trame emulee / trame VGA : %ld ligne(s) (0 = verrouille) ; trame VGA %lu us\n",
                        (long)g_phase_err_lines, (unsigned long)g_vga_frame_us);
          s_t0 = t; s_vf0 = vf; s_vs0 = vs; s_rc0 = rc; s_dc0 = dc; g_vid_render_max = 0; }
        { uint32_t cp, cd, cw, cn; bool qon; gime_video_capture_stats(&cp, &cd, &cw, &cn, &qon);
          LogOut.printf("[CAPT] file %s ; trames capturees %lu, abandonnees %lu, attentes du coeur video %lu, sans tampon libre %lu\n",
                        qon ? "ACTIVE" : "coupee (copie directe)", (unsigned long)cp, (unsigned long)cd, (unsigned long)cw, (unsigned long)cn); }
        { const audio_stats_t *as = Audio_GetStats();
          LogOut.printf("[AUDIO] %s, evenements %lu, perdus %lu, en retard %lu, resynchros %lu, vidages (Turbo) %lu ; sortie : moyenne par echantillon, couplage alternatif, decalage anticipe (7t35)\n", Audio_UsingI2S() ? (Audio_IsStereo() ? "I2S PDM stereo (GPIO 47 G + 12 D)" : "I2S PDM mono (GPIO 47)") : (Audio_UsingPWM() ? "PWM LEDC" : "broche forcee"),
                        (unsigned long)as->pushed, (unsigned long)as->dropped, (unsigned long)as->late, (unsigned long)as->resyncs, (unsigned long)as->flushes); }
        { extern void Sdc_Report(void); Sdc_Report(); }
        if (g_rs_present) { char rb[256]; Rs_Report(rb, sizeof rb); LogOut.println(rb); }
        { char rb[256]; Bb_Report(rb, sizeof rb); LogOut.println(rb); }
        if ((g_rs_present && g_rs_link == RS_LINK_MODEM) || g_bb_mode == BB_MODEM) { char rb[340]; Modem_Report(rb, sizeof rb); LogOut.println(rb); }
        Net_Report();
        NetShare_Report();
        Web_Report();
        Health_Report();
        { char cb[600]; Cas_Report(cb, sizeof cb); LogOut.println(cb); }
        { extern void Cpu_Report(void); Cpu_Report(); }
        { extern void Usb_Report(bool); Usb_Report(false); }
        for (int i = 0; i < DW_DRIVES + 2; i++) if (g_dw_name[i][0]) LogOut.printf(i < DW_DRIVES ? "[DW]   /x%d = %s\n" : "[VHD]  disque %d = %s\n", i < DW_DRIVES ? i : i - DW_DRIVES, g_dw_name[i]);
      }
      cmdLen = 0;
    }
    else if (cmdLen < sizeof(cmdBuf) - 1)
    {
      cmdBuf[cmdLen++] = c;
    }
  }
}

struct DiskAccessStruct
{
  uint8_t TrackPos;
  uint8_t DriveSelected;
  uint8_t MotorOnOff;
  uint8_t SectorPos;
  uint8_t DRIVE_COMMAND;
  uint8_t DataRegisterValue;
  uint32_t DSK_FILE_DataPTR;
  bool IsinReadProcess;
  bool IsInWriteProcess;
  bool IsInAddressProcess;  // READ_ADDRESS ($C0) — utilisé par BACKUP pour lire l'en-tête…
  uint8_t AddressField[6];  // Track, Side, Sector, Length code, CRC hi, CRC lo
  // Vrai après une commande Type I (RESTORE/SEEK/STEP*) ; faux après une commande Type II/III
  // (READ/WRITE SECTOR, READ ADDRESS) ou FORCE_INT
  bool LastCommandWasTypeI;
  // WRITE_TRACK ($F0, utilisé par DSKINI pour le formatage bas niveau) le ROM streame ~6250 octets
  // bruts (gaps/sync/ID/CRC/données MFM) sans que notre "disquette" (un tableau de secteurs à plat)
  // ait…
  bool IsInTrackFormat;
  // Direction du dernier STEP_IN/STEP_OUT effectué (true=IN/avance false=OUT/recul)
  bool LastStepDirectionWasIn;
  bool NMI_Int_Started;
  uint8_t NMI_Delay;    // To Allow the CoCo to read the last byte from the FD502 before the NMI.
  uint16_t RW_Process_ByteRemainingCounter;
};

DiskAccessStruct DiskAccess;

DriveStruct Disk_Drive;

// ---------------------------------------------------------------------------
// Bus CPU. $0000-$FEFF : table de pages GIME (un seul test + indirection voir gime.h). $FF00-$FFFF :
// I/O -> ManagePeripherals_* puis registres GIME ($FF90-$FFBF) ou ombre I/O / vecteurs dans rom[]
// ---------------------------------------------------------------------------
// compteur de cycles ESP32 (CCOUNT) et cumuls de la mesure de charge.
struct perf_t;
extern perf_t g_perf;
static inline uint32_t Perf_CCount(void)
{
#ifdef ESP_PLATFORM
  uint32_t c; __asm__ __volatile__("rsr %0, ccount" : "=a"(c)); return c;
#else
  return 0;
#endif
}
static inline void Perf_AddIo(uint32_t cc);

class cpu_t : public mc6809
{
  public:
      uint8_t read8(uint16_t address) const;
      uint8_t read8_io(uint16_t address) const;
      void write8(uint16_t address, uint8_t value) const;
      void write8_io(uint16_t address, uint8_t value) const;
};

// EXÉCUTION PAR LOTS (ordonnancement par échéances)
static uint32_t g_batch_acc = 0;     // cycles CPU du lot pas encore appliqués
static bool     g_batch_io  = false; // E/S pendant le lot -> fin du lot
static void Batch_Sync(void);
// CoCo SDC (voir Sdc_* plus bas).
static bool Sdc_Read(uint16_t a, uint8_t *v);
static bool Sdc_Write(uint16_t a, uint8_t v);
// méthodes définies HORS de la classe. Une méthode définie dans la classe est « inline » (section
// COMDAT)
IRAM_ATTR uint8_t cpu_t::read8(uint16_t address) const
{
  if (address < 0xFF00)
  {
    return gime_read_mem(address);
  }
  if (g_batch_acc) Batch_Sync();       // E/S datée au début de son instruction
  g_batch_io = true;
  const uint32_t cc0 = Perf_CCount();   // temps passé en E/S
  const uint8_t r = read8_io(address);
  Perf_AddIo(Perf_CCount() - cc0);
  return r;
}

IRAM_ATTR __attribute__((optimize("no-jump-tables"))) uint8_t cpu_t::read8_io(uint16_t address) const
{
  if (address >= 0xFF80 && address <= 0xFF86 && VHD_Enabled()) return VHD_Read(address);
  if (address == 0xFF7F && g_slotcfg_run.mpi) return g_mpi_reg;                               // Multi-Pak
  if (g_scs_cpsg && (address & 0xFFF0) == 0xFF50) { extern uint8_t Cpsg_Read(uint16_t); return Cpsg_Read(address); }
  if (g_rs_present && (address & 0xFFFC) == 0xFF68) return Rs_Read(address);                 // RS-232 Pak
  // miroirs des PIA (décodage partiel du CoCo) $FF04-$FF1F = PIA0 ($FF00-$FF03), $FF24-$FF3F = PIA1
  // ($FF20-$FF23).
  if (address < 0xFF40) address &= 0xFF23;
  else if (address < 0xFF50)
  {
    // port Becker DriveWire ($FF41/$FF42) prioritaire sur les miroirs du FDC quand un disque virtuel
    // est monté.
    if ((address == 0xFF41 || address == 0xFF42) && Becker_Net())   // port Becker -> serveur DriveWire du PC
    {
      if (address == 0xFF41) return NetDw_Ready() ? 0x02 : 0x00;
      const int b = NetDw_Get(); return b < 0 ? 0 : (uint8_t)b;
    }
    if ((address == 0xFF41 || address == 0xFF42) && dw_port_enabled()) return dw_becker_read(address);
    if (!g_ff40_fdc) return 0xFF;                               // slot SCS sans contrôleur de disquette
    { uint8_t sv; if (Sdc_Read(address, &sv)) return sv; }     // CoCo SDC
    address = (address < 0xFF48) ? 0xFF40 : (address & 0xFF4B);  // miroirs FDC
  }
  // décale la colonne active par ASL/ROL directement sur le DDR de $FF02.
  if ((address == 0xFF00 || address == 0xFF02 || address == 0xFF20 || address == 0xFF22) &&
      (rom[(uint16_t)(address + 1) - ROM_OFFSET] & 0x04) == 0)
  {
    return g_pia_ddr[(address == 0xFF00) ? 0 : (address == 0xFF02 ? 1 : (address == 0xFF22 ? 2 : 3))];   // + $FF20
  }
  ManagePeripherals_Read(address);
  uint8_t v;
  if (gime_read_register(address, &v))
  {
    return v;
  }
  if (address == 0xFF22) return (uint8_t)((rom[ROM_FF22] & 0xFE) | Bb_InBit());   // bit 0 = entrée série (bit banger)
  if (address == 0xFF20 && !(g_pia_ddr[3] & 0x01))                                   // bit 0 = entrée cassette (comparateur)
    return (uint8_t)((rom[ROM_FF20] & 0xFE) | Cas_InBit((uint32_t)g_emu_ticks));
  return rom[address - ROM_OFFSET];
}

IRAM_ATTR void cpu_t::write8(uint16_t address, uint8_t value) const
{
  if (address < 0xFF00)
  {
    gime_write_mem(address, value);
    return;
  }
  if (g_batch_acc) Batch_Sync();
  g_batch_io = true;
  const uint32_t cc0 = Perf_CCount();
  write8_io(address, value);
  Perf_AddIo(Perf_CCount() - cc0);
}

IRAM_ATTR __attribute__((optimize("no-jump-tables"))) void cpu_t::write8_io(uint16_t address, uint8_t value) const
{
  if (address >= 0xFF80 && address <= 0xFF86 && VHD_Enabled()) { VHD_Write(address, value); return; }
  if (address == 0xFF7F && g_slotcfg_run.mpi) { extern void Mpi_Write(uint8_t); Mpi_Write(value); return; }
  if (g_scs_cpsg && (address & 0xFFF0) == 0xFF50) { extern void Cpsg_Write(uint16_t, uint8_t); Cpsg_Write(address, value); return; }
  if ((address == 0xFF7A || address == 0xFF7B) && g_o90_present) { extern void O90_Write(uint16_t, uint8_t); O90_Write(address, value); return; }
  if (g_rs_present && (address & 0xFFFC) == 0xFF68) { Rs_Write(address, value); return; }   // RS-232 Pak
  if (address < 0xFF40) address &= 0xFF23;   // miroirs PIA
  else if (address < 0xFF50)
  {
    if (g_scs_banked && address <= 0xFF41)                     // banque ($FF40) / PSG du GMC ($FF41)
    {
      extern void Cart_BankWrite(uint8_t); extern void Gmc_PsgWrite(uint8_t);
      if (address == 0xFF40) { Cart_BankWrite(value); return; }
      if (g_scs_gmc) { Gmc_PsgWrite(value); return; }
    }
    if ((address == 0xFF41 || address == 0xFF42) && Becker_Net()) { if (address == 0xFF42) NetDw_Put(value); return; }
    if ((address == 0xFF41 || address == 0xFF42) && dw_port_enabled()) { dw_becker_write(address, value); return; }
    if (!g_ff40_fdc) return;                                    // pas de contrôleur de disquette
    if (Sdc_Write(address, value)) return;                      // CoCo SDC
    address = (address < 0xFF48) ? 0xFF40 : (address & 0xFF4B);  // miroirs FDC
  }
  ManagePeripherals_Write(address, value);
  gime_write_register(address, value);   // sans effet hors $FF90-$FFBF
}

// ===========================================================================
// couche disque — images 35/40/80 pistes, simple ou double face en-tête JVC ; chargement en PSRAM ou
// lecture « à la volée » depuis la SD. Géométrie : en-tête JVC (taille % 256)

// en lecture à la volée, écriture immédiate du secteur.
// ===========================================================================
#define DISK_SIZE      161280u                       // image 35 pistes simple face (bancs)
#define DISK_RAM_CAP   (40u * 18u * 256u + 256u)     // 184 576 : SF 40 pistes + en-tête
#define DISK_MAX_LBA   4096u                         // 80 pistes x 2 faces x 25 secteurs max.

struct DiskUnit
{
  uint8_t *ram;          // image en PSRAM (nullptr = lecture à la volée)
  uint8_t *fixed;        // tampon préalloué du lecteur (DISK_RAM_CAP)
  uint8_t *dyn;          // tampon alloué pour une grande image
  uint32_t size;         // taille du fichier (avec en-tête)
  uint32_t hdr;          // octets d'en-tête JVC
  uint8_t  tracks, sides, spt, first;
  bool     loaded;
  bool     dirty_any;
  uint8_t  dirty[DISK_MAX_LBA / 8];
  // disquette du dossier partagé d'un PC (netshare) : image en PSRAM renvoyée au PC 2 s après la
  // dernière écriture (loop -> NetShare_Poll)
  bool     net;
  char     net_path[192];
  volatile uint32_t net_stamp, net_saved, net_ms;
#ifdef ESP_PLATFORM
  File     f;
  bool     f_open;
  bool     f_rw;         // fichier ouvert en écriture
#endif
};
static DiskUnit g_disk[4];

uint8_t *RAM_Disk0 = nullptr;    // = g_disk[i].fixed (compatibilité : bancs d'essai)
uint8_t *RAM_Disk1 = nullptr;
uint8_t *RAM_Disk2 = nullptr;
uint8_t *RAM_Disk3 = nullptr;

#ifdef ESP_PLATFORM
#include <freertos/semphr.h>
static SemaphoreHandle_t s_sd_mutex = nullptr;
#define SD_LOCK()   do { if (s_sd_mutex) xSemaphoreTake(s_sd_mutex, portMAX_DELAY); } while (0)
#define SD_UNLOCK() do { if (s_sd_mutex) xSemaphoreGive(s_sd_mutex); } while (0)
#else
#define SD_LOCK()   do { } while (0)
#define SD_UNLOCK() do { } while (0)
#endif

static void Disk_SetGeometry(DiskUnit &d, uint32_t size, const uint8_t *h)
{
  d.size = size;
  d.hdr = size % 256u;
  d.spt = 18; d.sides = 1; d.first = 1;
  if (d.hdr >= 1 && h[0] >= 1)  d.spt = h[0];
  if (d.hdr >= 2 && h[1] >= 1 && h[1] <= 2) d.sides = h[1];
  if (d.hdr >= 4) d.first = h[3];
  // Arrondi à la piste supérieure : certaines images sont tronquées (ex. 159 232 o = 34 pistes + 10
  // secteurs) ; les secteurs absents se lisent $FF.
  const uint32_t trk_bytes = (uint32_t)d.spt * 256u * d.sides;
  uint32_t t = (size - d.hdr + trk_bytes - 1) / trk_bytes;
  if (d.hdr < 2 && d.sides == 1 && t > 42) { d.sides = 2; t = (t + 1) / 2; }
  if (t > 255) t = 255;
  d.tracks = (uint8_t)t;
  d.dirty_any = false;
  memset(d.dirty, 0, sizeof d.dirty);
}

static bool Disk_Lba(const DiskUnit &d, uint8_t track, uint8_t side, uint8_t sector, uint32_t &lba)
{
  if (!d.loaded || track >= d.tracks || side >= d.sides) return false;
  if (sector < d.first || sector >= d.first + d.spt) return false;
  lba = ((uint32_t)track * d.sides + side) * d.spt + (sector - d.first);
  return lba < DISK_MAX_LBA;
}

uint8_t Disk_Sides(uint8_t drv) { return (drv < 4 && g_disk[drv].loaded) ? g_disk[drv].sides : 1; }
uint8_t Disk_Tracks(uint8_t drv) { return (drv < 4 && g_disk[drv].loaded) ? g_disk[drv].tracks : 0; }
uint8_t Disk_Spt(uint8_t drv)    { return (drv < 4 && g_disk[drv].loaded) ? g_disk[drv].spt : 18; }
uint8_t Disk_First(uint8_t drv)  { return (drv < 4 && g_disk[drv].loaded) ? g_disk[drv].first : 1; }

// dernier disque lu par le CoCo (nom du Save State) : 0-3 = disquettes 4 + n = unité DriveWire / VHD n
// ; -1 = aucun depuis le démarrage.
volatile int8_t g_state_last_disk = -1;

bool Disk_ReadSector(uint8_t drv, uint8_t track, uint8_t side, uint8_t sector, uint8_t *buf)
{
  if (drv > 3) return false;
  g_state_last_disk = (int8_t)drv;
  DiskUnit &d = g_disk[drv];
  uint32_t lba;
  if (!Disk_Lba(d, track, side, sector, lba)) return false;
  const uint32_t off = d.hdr + lba * 256u;
  if (d.ram)
  {
    if (off + 256u <= d.size) memcpy(buf, d.ram + off, 256);
    else
    {
      memset(buf, 0xFF, 256);                           // image tronquée
      if (off < d.size) memcpy(buf, d.ram + off, d.size - off);
    }
    return true;
  }
#ifdef ESP_PLATFORM
  bool ok = false;
  SD_LOCK();
  if (d.f_open && d.f.seek(off))
  {
    const size_t r = d.f.read(buf, 256);
    if (r < 256) memset(buf + r, 0xFF, 256 - r);        // image tronquée
    ok = true;
  }
  SD_UNLOCK();
  return ok;
#else
  return false;
#endif
}

bool Disk_WriteSector(uint8_t drv, uint8_t track, uint8_t side, uint8_t sector, const uint8_t *buf)
{
  if (drv > 3) return false;
  DiskUnit &d = g_disk[drv];
  uint32_t lba;
  if (!Disk_Lba(d, track, side, sector, lba)) return false;
  const uint32_t off = d.hdr + lba * 256u;
  if (d.ram)
  {
    if (off + 256u > d.size) return false;             // hors de l'image (tronquée)
    memcpy(d.ram + off, buf, 256);
    d.dirty[lba >> 3] |= (uint8_t)(1u << (lba & 7));
    d.dirty_any = true;
    sf.PHYSICAL_Drive_Must_Be_Saved = true;      // écriture SD par le coeur vidéo
    return true;
  }
#ifdef ESP_PLATFORM
  bool ok = false;
  SD_LOCK();
  if (d.f_open && d.f_rw && d.f.seek(off)) { ok = (d.f.write(buf, 256) == 256); d.f.flush(); }
  SD_UNLOCK();
  return ok;
#else
  return false;
#endif
}

// accès par numéro de secteur logique (LSN = ordre linéaire de l'image, comme OS-9 et la CoCo SDC),
// pour la SDC émulée.
static bool Disk_ReadLSN(uint8_t drv, uint32_t lsn, uint8_t *buf)
{
  if (drv > 3 || !g_disk[drv].loaded || lsn >= DISK_MAX_LBA) return false;
  g_state_last_disk = (int8_t)drv;
  DiskUnit &d = g_disk[drv];
  const uint32_t off = d.hdr + lsn * 256u;
  if (off >= d.size) return false;
  if (d.ram)
  {
    memset(buf, 0xFF, 256);
    memcpy(buf, d.ram + off, (off + 256u <= d.size) ? 256u : d.size - off);
    return true;
  }
#ifdef ESP_PLATFORM
  bool ok = false;
  SD_LOCK();
  if (d.f_open && d.f.seek(off))
  {
    const size_t r = d.f.read(buf, 256);
    if (r < 256) memset(buf + r, 0xFF, 256 - r);
    ok = true;
  }
  SD_UNLOCK();
  return ok;
#else
  return false;
#endif
}
static bool Disk_WriteLSN(uint8_t drv, uint32_t lsn, const uint8_t *buf)
{
  if (drv > 3 || !g_disk[drv].loaded || lsn >= DISK_MAX_LBA) return false;
  DiskUnit &d = g_disk[drv];
  const uint32_t off = d.hdr + lsn * 256u;
  if (off + 256u > d.size) return false;
  if (d.ram)
  {
    memcpy(d.ram + off, buf, 256);
    d.dirty[lsn >> 3] |= (uint8_t)(1u << (lsn & 7));
    d.dirty_any = true;
    sf.PHYSICAL_Drive_Must_Be_Saved = true;
    return true;
  }
#ifdef ESP_PLATFORM
  bool ok = false;
  SD_LOCK();
  if (d.f_open && d.f_rw && d.f.seek(off)) { ok = (d.f.write(buf, 256) == 256); d.f.flush(); }
  SD_UNLOCK();
  return ok;
#else
  return false;
#endif
}

// ===========================================================================
// émulation de la CoCo SDC (interface de commandes). Protocole tiré du pilote NitrOS-9 llcocosdc
// (disque CRIBBOS9) détection
// ===========================================================================
static struct
{
  bool     cmd_mode;        // $43 écrit en $FF40
  uint8_t  flash42;         // registre $FF42 (détection)
  uint8_t  p[3];            // $FF49-$FF4B
  uint8_t  status;
  uint8_t  cmd;
  uint8_t  drv;
  uint16_t pos;             // octets transférés
  bool     rd, wr;          // transfert de données en cours
  bool     result;          // paramètres rendus (lisibles aussi hors du mode commandes)
  uint8_t  buf[256];
  uint32_t n_rd, n_wr, n_cmd, n_det, n_mode, n_fail;   // diagnostic (W)
  uint32_t last_lsn;
  uint8_t  last_cmd, last_status;
} s_sdc;
#define SDC_BUSY   0x01
#define SDC_READY  0x02
#define SDC_FAILED 0x80
volatile bool g_sdc_enabled = true;

void Sdc_Report(void)   // ligne [SDC] de W
{
  LogOut.printf("[SDC] %s ; detections %lu, entrees en mode commandes %lu, commandes %lu (lectures %lu, ecritures %lu, echecs %lu) ; derniere : cmd $%02X LSN %lu etat $%02X ; lecteur 0 %s, lecteur 1 %s%s\n",
                g_sdc_enabled ? (dw_port_enabled() ? "ABSENTE (port DriveWire actif)" : "presente") : "desactivee",
                (unsigned long)s_sdc.n_det, (unsigned long)s_sdc.n_mode, (unsigned long)s_sdc.n_cmd, (unsigned long)s_sdc.n_rd,
                (unsigned long)s_sdc.n_wr, (unsigned long)s_sdc.n_fail, s_sdc.last_cmd, (unsigned long)s_sdc.last_lsn, s_sdc.last_status,
                g_disk[0].loaded ? "monte" : "vide", g_disk[1].loaded ? "monte" : "vide", s_sdc.cmd_mode ? " (en mode commandes)" : "");
}
static uint32_t Sdc_Lsn(void) { return ((uint32_t)s_sdc.p[0] << 16) | ((uint32_t)s_sdc.p[1] << 8) | s_sdc.p[2]; }

static void Sdc_Command(uint8_t v)
{
  s_sdc.cmd = v; s_sdc.drv = v & 1; s_sdc.pos = 0; s_sdc.rd = s_sdc.wr = false; s_sdc.result = false;
  s_sdc.n_cmd++; s_sdc.last_cmd = v; s_sdc.last_lsn = Sdc_Lsn();
  const uint8_t kind = v & 0xE0;
  if (kind == 0x80)                                    // lecture d'un secteur
  {
    if (Disk_ReadLSN(s_sdc.drv, Sdc_Lsn(), s_sdc.buf)) { s_sdc.rd = true; s_sdc.status = SDC_BUSY | SDC_READY; s_sdc.n_rd++; }
    else s_sdc.status = SDC_FAILED | 0x10;             // secteur ou image absents
  }
  else if (kind == 0xA0 || kind == 0xE0)               // écriture d'un secteur / commande étendue avec données
  {
    if (kind == 0xA0 && !g_disk[s_sdc.drv].loaded) { s_sdc.status = SDC_FAILED | 0x10; return; }
    s_sdc.wr = true; s_sdc.status = SDC_BUSY | SDC_READY;
  }
  else if (kind == 0xC0)                               // commande étendue sans données
  {
    s_sdc.status = 0;
    if (s_sdc.p[0] == 'Q')                             // taille de l'image (secteurs)
    {
      const DiskUnit &d = g_disk[s_sdc.drv];
      if (!d.loaded) { s_sdc.status = SDC_FAILED | 0x10; return; }
      const uint32_t n = (d.size - d.hdr) / 256u;
      s_sdc.p[0] = (uint8_t)(n >> 16); s_sdc.p[1] = (uint8_t)(n >> 8); s_sdc.p[2] = (uint8_t)n;
      s_sdc.result = true;
    }
  }
  else s_sdc.status = 0;                               // autres : sans effet
}

static bool Sdc_Read(uint16_t a, uint8_t *v)
{
  if (!g_sdc_enabled) return false;
  if (a == 0xFF42 || a == 0xFF43)                      // registres flash (détection)
  {
    *v = (a == 0xFF43) ? (uint8_t)(s_sdc.flash42 & 0x60) : s_sdc.flash42;
    if (a == 0xFF43) s_sdc.n_det++;
    return true;
  }
  if (!s_sdc.cmd_mode)
  {
    // Résultat d'une commande « Q » relu après la sortie du mode commandes (le pilote lit $FF49-$FF4B
    // après avoir remis $FF40 à 0).
    if (s_sdc.result && a >= 0xFF49 && a <= 0xFF4B) { *v = s_sdc.p[a - 0xFF49]; return true; }
    return false;
  }
  switch (a)
  {
    case 0xFF48: *v = s_sdc.status; return true;
    case 0xFF49: *v = s_sdc.p[0]; return true;
    case 0xFF4A:
    case 0xFF4B:
      if (s_sdc.rd)
      {
        *v = s_sdc.buf[s_sdc.pos++];
        if (s_sdc.pos >= 256) { s_sdc.rd = false; s_sdc.status = 0; }
      }
      else *v = s_sdc.p[a - 0xFF49];
      return true;
    default: return a >= 0xFF40 && a <= 0xFF4F;        // reste de la plage : sans effet en mode commandes
  }
}

static bool Sdc_Write(uint16_t a, uint8_t v)
{
  if (!g_sdc_enabled) return false;
  if (a == 0xFF42) { s_sdc.flash42 = v; return true; }
  if (a == 0xFF43) return true;
  if (a == 0xFF40)
  {
    if (v == 0x43) { s_sdc.cmd_mode = true; s_sdc.result = false; s_sdc.n_mode++; return true; }   // entrée en mode commandes
    s_sdc.cmd_mode = false;                            // sortie : la valeur va aussi au contrôleur ($FF40)
    return false;
  }
  if (!s_sdc.cmd_mode) { if (a >= 0xFF48 && a <= 0xFF4B) s_sdc.result = false; return false; }
  switch (a)
  {
    case 0xFF48:
      Sdc_Command(v); s_sdc.last_status = s_sdc.status;
      if (s_sdc.status & SDC_FAILED)
      {
        if (++s_sdc.n_fail <= 5)   // premiers échecs signalés sur le port série
          LogOut.printf("[SDC] echec : cmd $%02X LSN %lu (lecteur %u %s)\n", v, (unsigned long)Sdc_Lsn(), v & 1, g_disk[v & 1].loaded ? "monte" : "vide");
      }
      return true;
    case 0xFF49: s_sdc.p[0] = v; return true;
    case 0xFF4A:
    case 0xFF4B:
      if (s_sdc.wr)
      {
        s_sdc.buf[s_sdc.pos++] = v;
        if (s_sdc.pos >= 256)
        {
          s_sdc.wr = false;
          if ((s_sdc.cmd & 0xE0) == 0xA0)
          {
            s_sdc.status = Disk_WriteLSN(s_sdc.drv, Sdc_Lsn(), s_sdc.buf) ? 0 : (SDC_FAILED | 0x20);
            s_sdc.n_wr++;
          }
          else s_sdc.status = 0;                       // commande étendue avec données : acceptée
        }
      }
      else s_sdc.p[a - 0xFF49] = v;
      return true;
    default: return true;
  }
}

// Écrit sur la SD les secteurs modifiés (coeur vidéo, ou avant un changement de disquette). Rend le
// nombre de secteurs écrits.
uint32_t Disk_Flush(uint8_t drv)
{
  if (drv > 3) return 0;
  DiskUnit &d = g_disk[drv];
  if (!d.loaded || !d.ram || !d.dirty_any) return 0;
  d.dirty_any = false;
  if (d.net)                                        // disque du PC : renvoi par loop
  {
    memset(d.dirty, 0, sizeof d.dirty);
    d.net_ms = millis();
    d.net_stamp++;
    return 0;
  }
  uint32_t n = 0;
#ifdef ESP_PLATFORM
  if (!d.f_open || !d.f_rw) return 0;
  SD_LOCK();
  for (uint32_t i = 0; i < DISK_MAX_LBA / 8; i++)
  {
    if (!d.dirty[i]) continue;
    const uint8_t m = __atomic_exchange_n(&d.dirty[i], (uint8_t)0, __ATOMIC_ACQ_REL);   // le coeur CPU peut marquer en parallèle
    for (uint32_t b = 0; b < 8; b++)
    {
      if (!(m & (1u << b))) continue;
      const uint32_t off = d.hdr + (i * 8u + b) * 256u;
      if (d.f.seek(off)) d.f.write(d.ram + off, 256);
      n++;
    }
  }
  d.f.flush();
  SD_UNLOCK();
#else
  memset(d.dirty, 0, sizeof d.dirty);
#endif
  return n;
}

void Disk_FlushAll(void)
{
  for (uint8_t i = 0; i < 4; i++) Disk_Flush(i);
}

static void Disk_Unmount(uint8_t drv)
{
  DiskUnit &d = g_disk[drv];
  Disk_Flush(drv);
  if (d.net)                                        // disque du PC modifié : renvoyé avant de le quitter
  {
    if (d.net_stamp != d.net_saved && d.ram)
    {
      const int r = NetShare_Put(d.net_path, d.ram, d.size);
      LogOut.printf("[SHARE] lecteur %u : %s %s avant le changement de disque\n", drv, d.net_path, r == NS_OK ? "renvoye au PC" : "NON renvoye (PC injoignable)");
    }
    d.net = false;
  }
#ifdef ESP_PLATFORM
  SD_LOCK();
  if (d.f_open) { d.f.close(); d.f_open = false; }
  SD_UNLOCK();
#endif
  if (d.dyn) { free(d.dyn); d.dyn = nullptr; }
  d.ram = nullptr;
  d.loaded = false;
}

// Banc d'essai / images en mémoire : copie « data » (taille « size »).
bool Disk_MountMemory(uint8_t drv, const uint8_t *data, uint32_t size)
{
  if (drv > 3) return false;
  Disk_Unmount(drv);
  DiskUnit &d = g_disk[drv];
  uint8_t *dst = d.fixed;
  if (size > DISK_RAM_CAP)
  {
    d.dyn = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (!d.dyn) return false;
    dst = d.dyn;
  }
  memcpy(dst, data, size);
  d.ram = dst;
  d.loaded = true;
  Disk_SetGeometry(d, size, dst);
  return true;
}

// disquette du PC (dossier partagé) : copie montée en PSRAM, renvoyée au PC
bool Disk_MountNet(uint8_t drv, const char *path, const uint8_t *data, uint32_t size)
{
  if (drv > 3 || !Disk_MountMemory(drv, data, size)) return false;
  NetShare_QueueMount(drv, "");
  DiskUnit &d = g_disk[drv];
  d.net = true;
  snprintf(d.net_path, sizeof d.net_path, "%s", path);
  d.net_stamp = d.net_saved = 0;
  d.net_ms = 0;
  LogOut.printf("[DISK] lecteur %u : %s du PC, %u o, %u pistes x %u face(s) x %u sect., PSRAM (renvoi au PC)\n",
                drv, path, (unsigned)size, d.tracks, d.sides, d.spt);
  return true;
}

// montage différé d'un disque du PC (démarrage, Save State), pendant que le CoCo tourne : l'image
// reçue (PSRAM) est adoptée telle quelle
bool Disk_AdoptNet(uint8_t drv, const char *path, uint8_t *img, uint32_t size)
{
  if (drv > 3 || !img || size < 256) return false;
  DiskUnit &d = g_disk[drv];
#ifdef ESP_PLATFORM
  if (d.f_open) return false;
#endif
  if (d.net || d.dyn) return false;                 // un autre disque a été monté entre-temps
  d.loaded = false;
  __sync_synchronize();
  d.ram = img;
  d.dyn = img;                                      // libéré au prochain démontage
  Disk_SetGeometry(d, size, img);
  d.net = true;
  snprintf(d.net_path, sizeof d.net_path, "%s", path);
  d.net_stamp = d.net_saved = 0;
  d.net_ms = 0;
  __sync_synchronize();
  d.loaded = true;
  LogOut.printf("[DISK] lecteur %u : %s du PC monte (%u o, %u pistes x %u face(s))\n", drv, path, (unsigned)size, d.tracks, d.sides);
  return true;
}

bool Disk_NetDirty(uint8_t drv, const uint8_t **data, uint32_t *size, const char **path, uint32_t *stamp, uint32_t *last_ms)
{
  if (drv > 3) return false;
  DiskUnit &d = g_disk[drv];
  if (!d.loaded || !d.net || !d.ram || d.net_stamp == d.net_saved) return false;
  *data = d.ram; *size = d.size; *path = d.net_path; *stamp = d.net_stamp; *last_ms = d.net_ms;
  return true;
}

void Disk_NetSaved(uint8_t drv, uint32_t stamp, bool ok)
{
  if (drv > 3 || !ok) return;
  g_disk[drv].net_saved = stamp;                    // une écriture pendant l'envoi change net_stamp : nouvel envoi
}

void Disk_Init(void)
{
#ifdef ESP_PLATFORM
  if (!s_sd_mutex) s_sd_mutex = xSemaphoreCreateMutex();
#endif
  uint8_t *fx[4] = { RAM_Disk0, RAM_Disk1, RAM_Disk2, RAM_Disk3 };
  for (int i = 0; i < 4; i++)
  {
    DiskUnit &d = g_disk[i];
    d.fixed = fx[i];
    d.dyn = nullptr;
#ifdef ESP_PLATFORM
    d.f_open = false; d.f_rw = false;
#endif
    // État par défaut (compatibilité bancs) : 35 pistes SF vierges en PSRAM.
    if (d.fixed) memset(d.fixed, 0xFF, DISK_SIZE);
    d.ram = d.fixed;
    d.loaded = (d.fixed != nullptr);
    Disk_SetGeometry(d, DISK_SIZE, d.fixed);
  }
}

bool ReadCoCoFile(const char* filename, uint8_t DriveNumber)
{
  if (DriveNumber > 3) return false;
  NetShare_QueueMount(DriveNumber, "");             // un montage différé en attente est annulé
  DiskUnit &d = g_disk[DriveNumber];
#ifdef ESP_PLATFORM
  Disk_Unmount(DriveNumber);
  SD_LOCK();
  File f = SD_MMC.open(filename, "r+");
  bool rw = true;
  if (!f) { f = SD_MMC.open(filename, FILE_READ); rw = false; }
  if (!f || f.isDirectory())
  {
    if (f) f.close();
    SD_UNLOCK();
    // Comme avant la lecteur « vide » = disquette vierge de 35 pistes en PSRAM (non sauvegardée),
    // plutôt qu'un lecteur absent.
    if (d.fixed)
    {
      memset(d.fixed, 0xFF, DISK_SIZE);
      d.ram = d.fixed;
      d.loaded = true;
      Disk_SetGeometry(d, DISK_SIZE, d.fixed);
    }
    if (strncmp(filename, "/PC:", 4) == 0)   // disque du PC : pas encore de Wi-Fi
    {
      LogOut.printf("[DISK] lecteur %u : %s est sur le PC : monte des que le Wi-Fi et le partage repondent\n", DriveNumber, filename + 1);
      NetShare_QueueMount(DriveNumber, filename + 4);   // montage différé (loop)
    }
    else if (!filename[0])                    // aucun disque choisi pour ce lecteur
    LogOut.printf("[DISK] lecteur %u : vide (disquette vierge en memoire)\n", DriveNumber);
    else
    LogOut.printf("[DISK] lecteur %u : impossible d'ouvrir %s (disquette vierge en memoire)\n", DriveNumber, filename);
    return false;
  }
  const uint32_t size = f.size();
  uint8_t hdrb[8] = { 0 };
  const uint32_t hdr = size % 256u;
  if (hdr) { f.seek(0); f.read(hdrb, hdr < 8 ? hdr : 8); }
  // Mémoire : tampon fixe, sinon tampon dynamique, sinon lecture à la volée
  uint8_t *dst = nullptr;
  if (size <= DISK_RAM_CAP) dst = d.fixed;
  else
  {
    d.dyn = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    dst = d.dyn;
  }
  if (dst)
  {
    f.seek(0);
    uint32_t got = 0;
    while (got < size)
    {
      const size_t r = f.read(dst + got, (size - got) > 32768u ? 32768u : (size - got));
      if (r == 0) break;
      got += r;
    }
    if (got != size && d.dyn) { free(d.dyn); d.dyn = nullptr; dst = nullptr; }
  }
  d.f = f; d.f_open = true; d.f_rw = rw;
  SD_UNLOCK();
  d.ram = dst;
  d.loaded = true;
  Disk_SetGeometry(d, size, hdrb);
  BOOTLOG("[DISK] lecteur %u : %s, %u o, %u pistes x %u face(s) x %u sect., %s%s\n",
                DriveNumber, filename, (unsigned)size, d.tracks, d.sides, d.spt,
                dst ? "PSRAM" : "lecture a la volee (SD)", rw ? "" : ", lecture seule");
  return true;
#else
  (void)filename;
  return d.loaded;
#endif
}

// Compatibilité : l'ancienne sauvegarde de l'image entière devient l'écriture des secteurs modifiés.
bool WriteCoCoFile(const char* filename, uint8_t DriveNumber)
{
  (void)filename;
  Disk_Flush(DriveNumber);
  return true;
}

// ===========================================================================
// disques virtuels DriveWire (/x0-/x3) servis par l'ESP32. Images « LSN » (secteur n à l'octet n x
// 256, sans en-tête ni géométrie) comme les images NitrOS-9 DriveWire
// ===========================================================================
// unités 0-3 = DriveWire /x0-/x3, unités 4-5 = disques durs virtuels « VHD » (interface $FF80-$FF86 de
// VCC/MAME, pilote EmuDsk).
#define VD_UNITS (DW_DRIVES + 2)
#define VHD_UNIT0 DW_DRIVES
char g_dw_name[VD_UNITS][256];             // chemin monté ("" = aucun)
struct DwUnit
{
  bool     mounted;
  uint32_t size;
#ifdef ESP_PLATFORM
  File     f;
  bool     rw;
#else
  FILE    *fp;
#endif
};
static DwUnit g_dw[VD_UNITS];
volatile bool g_dw_boot_request = false;   // menu F12 -> amorçage NitrOS-9 depuis /x0

static bool Dw_Present(uint8_t d) { return d < DW_DRIVES && g_dw[d].mounted; }

// cache de lecture par blocs de 4 Ko (16 secteurs), 48 blocs (192 Ko) en PSRAM, remplacement LRU. Sur
// la carte, chaque accès SD coûte cher (seek + lecture dans un fichier de 128 Mo)
#define VD_CACHE_LINES  48
#define VD_CACHE_SECT   16
struct VdCacheLine { uint8_t unit; bool valid; uint32_t block; uint32_t stamp; uint8_t *data; };
static VdCacheLine s_vdc[VD_CACHE_LINES];
static uint32_t s_vdc_clock = 0;
uint32_t g_vd_stat_reads = 0, g_vd_stat_hits = 0, g_vd_stat_sd = 0;
uint64_t g_vd_stat_sd_us = 0;
uint32_t g_vd_stat_direct = 0;              // lectures servies par la carte des clusters
volatile uint64_t g_vd_led_off_at = 0;       // LED disque : éteinte par le coeur vidéo après l'accès

static inline void Vd_LedPulse(void)
{
  DISKETTE_LED_ON();
  g_vd_led_off_at = esp_timer_get_time() + 40000;
}

static void Vd_CacheInvalidate(uint8_t d)
{
  for (int i = 0; i < VD_CACHE_LINES; i++) if (s_vdc[i].unit == d) s_vdc[i].valid = false;
}

// ---------------------------------------------------------------------------
// lecture directe des disques virtuels (VHD, DriveWire) par carte des clusters. Le core 3.2.1 n'a pas
// le « fast seek » de FatFs
// ---------------------------------------------------------------------------
#ifdef ESP_PLATFORM
#include "ff.h"
#include "diskio.h"
struct VdExtent { uint32_t fclu, lclu, n; };            // cluster du fichier, du volume, nombre
struct VdMap
{
  bool ok; uint8_t pdrv;
  uint32_t database, csize, ssize, clubytes, covered;   // covered : octets couverts par la carte
  uint32_t n_ext, hint;
  VdExtent *ext;
};
static VdMap s_vdmap[VD_UNITS];

static void Vd_MapFree(uint8_t d)
{
  VdMap &m = s_vdmap[d];
  if (m.ext) free(m.ext);
  memset(&m, 0, sizeof m);
}

// Appelé sous SD_LOCK, fichier déjà ouvert par l'API Arduino (taille connue).
static void Vd_MapBuild(uint8_t d, const char *path, uint32_t size)
{
  Vd_MapFree(d);
  if (!size || !path || path[0] != '/') return;
  // FIL (~4 Ko avec son tampon) alloué le temps du montage, en PSRAM en lecture ne passent pas par le
  // tampon du FIL.
  FIL *filp = (FIL *)heap_caps_malloc(sizeof(FIL), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!filp) { LogOut.printf("[VD] carte des clusters : memoire insuffisante, acces par le fichier\n"); return; }
  FIL &fil = *filp;
  char fp[300];
  FRESULT r = FR_NO_FILE;
  for (int drv = 0; drv < FF_VOLUMES && r != FR_OK; drv++)
  {
    snprintf(fp, sizeof fp, "%d:%s", drv, path);
    r = f_open(&fil, fp, FA_READ);
    if (r == FR_OK && f_size(&fil) != size) { f_close(&fil); r = FR_NO_FILE; }
  }
  if (r != FR_OK) { free(filp); LogOut.printf("[VD] carte des clusters impossible (%d) : acces par le fichier\n", (int)r); return; }
  FATFS *fs = fil.obj.fs;
  VdMap &m = s_vdmap[d];
  m.pdrv = fs->pdrv;
  m.database = (uint32_t)fs->database;
  m.csize = fs->csize;
#if FF_MAX_SS != FF_MIN_SS
  m.ssize = fs->ssize;
#else
  m.ssize = FF_MAX_SS;
#endif
  m.clubytes = m.csize * m.ssize;
  const uint8_t type = fs->fs_type;
  const uint32_t nfat = fs->n_fatent, fatbase = (uint32_t)fs->fatbase;
  uint32_t c = fil.obj.sclust;
  f_close(&fil);
  free(filp); filp = nullptr;
  if ((type != FS_FAT16 && type != FS_FAT32) || !m.clubytes || c < 2) { Vd_MapFree(d); return; }
  const uint32_t ncl = (uint32_t)(((uint64_t)size + m.clubytes - 1) / m.clubytes);
  uint32_t cap = 64;
  m.ext = (VdExtent *)heap_caps_malloc(cap * sizeof(VdExtent), MALLOC_CAP_SPIRAM);
  uint8_t *sec = (uint8_t *)heap_caps_malloc(m.ssize, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  bool ok = m.ext && sec;
  uint32_t cur_sec = 0xFFFFFFFFu;
  const uint32_t esz = (type == FS_FAT32) ? 4u : 2u;
  const uint32_t per = m.ssize / esz;                    // entrées de FAT par secteur
  for (uint32_t i = 0; ok && i < ncl; i++)
  {
    if (c < 2 || c >= nfat) { ok = false; break; }
    if (m.n_ext && m.ext[m.n_ext - 1].lclu + m.ext[m.n_ext - 1].n == c) m.ext[m.n_ext - 1].n++;
    else
    {
      if (m.n_ext == cap)
      {
        if (cap >= 8192) { ok = false; break; }           // fichier trop fragmenté
        VdExtent *ne = (VdExtent *)heap_caps_realloc(m.ext, cap * 2 * sizeof(VdExtent), MALLOC_CAP_SPIRAM);
        if (!ne) { ok = false; break; }
        m.ext = ne; cap *= 2;
      }
      m.ext[m.n_ext].fclu = i; m.ext[m.n_ext].lclu = c; m.ext[m.n_ext].n = 1;
      m.n_ext++;
    }
    if (i + 1 == ncl) break;
    const uint32_t s = fatbase + c / per;                  // entrée suivante dans la FAT
    if (s != cur_sec)
    {
      if (disk_read(m.pdrv, sec, s, 1) != RES_OK) { ok = false; break; }
      cur_sec = s;
    }
    const uint32_t o = (c % per) * esz;
    c = (type == FS_FAT32) ? (((uint32_t)sec[o] | ((uint32_t)sec[o + 1] << 8) | ((uint32_t)sec[o + 2] << 16) | ((uint32_t)sec[o + 3] << 24)) & 0x0FFFFFFFu)
                           : ((uint32_t)sec[o] | ((uint32_t)sec[o + 1] << 8));
  }
  if (sec) free(sec);
  if (!ok)
  {
    LogOut.printf("[VD] carte des clusters impossible (chaine) : acces par le fichier\n");
    Vd_MapFree(d);
    return;
  }
  m.covered = size;
  m.ok = true;
  LogOut.printf("[VD] carte des clusters : %lu plage(s), %lu clusters de %lu o (lecture directe)\n",
                (unsigned long)m.n_ext, (unsigned long)ncl, (unsigned long)m.clubytes);
}

// Lecture directe ; rend false si hors de la carte (l'appelant prend l'ancien chemin).
static bool Vd_ReadMapped(uint8_t d, uint32_t off, uint8_t *buf, uint32_t len)
{
  if (d >= VD_UNITS) return false;
  VdMap &m = s_vdmap[d];
  if (!m.ok || (off % m.ssize) || (len % m.ssize) || (uint64_t)off + len > m.covered) return false;
  while (len)
  {
    const uint32_t ci = off / m.clubytes;
    uint32_t k = m.hint < m.n_ext ? m.hint : 0;
    if (!(m.ext[k].fclu <= ci && ci < m.ext[k].fclu + m.ext[k].n))
    {
      uint32_t lo = 0, hi = m.n_ext;                       // recherche dichotomique
      while (hi - lo > 1) { const uint32_t mid = (lo + hi) / 2; if (m.ext[mid].fclu <= ci) lo = mid; else hi = mid; }
      k = lo;
      if (!(m.ext[k].fclu <= ci && ci < m.ext[k].fclu + m.ext[k].n)) return false;
    }
    m.hint = k;
    const VdExtent &e = m.ext[k];
    const uint32_t in = off % m.clubytes;
    const uint32_t lba = m.database + (e.lclu + (ci - e.fclu) - 2u) * m.csize + in / m.ssize;
    const uint32_t avail = (e.fclu + e.n - ci) * m.clubytes - in;
    const uint32_t n = len < avail ? len : avail;
    if (disk_read(m.pdrv, buf, lba, n / m.ssize) != RES_OK) return false;
    buf += n; off += n; len -= n;
  }
  g_vd_stat_direct++;
  return true;
}
#endif

// état de la carte SD (journal de démarrage et W)
extern bool SD_MMC_MountOK;
void Sd_Report(void)
{
#ifdef ESP_PLATFORM
  if (!SD_MMC_MountOK) { LogOut.println("[SD] carte non montee"); return; }
  const uint8_t ct = SD_MMC.cardType();
  FATFS *fs = nullptr; DWORD fre = 0;
  const char *fst = "?";
  uint32_t clu = 0;
  for (int drv = 0; drv < FF_VOLUMES; drv++)   // volume de la SD : celui dont la taille correspond
  {
    char dn[4] = { (char)('0' + drv), ':', 0, 0 };
    FATFS *f2 = nullptr; DWORD fr2 = 0;
    if (f_getfree(dn, &fr2, &f2) != FR_OK || !f2) continue;
    const uint64_t tot = (uint64_t)(f2->n_fatent - 2) * f2->csize * 512u;
    if (!fs || (tot + (64ull << 20) >= SD_MMC.totalBytes() && tot <= SD_MMC.totalBytes() + (64ull << 20))) { fs = f2; fre = fr2; }
  }
  if (fs)
  {
    fst = fs->fs_type == FS_FAT32 ? "FAT32" : fs->fs_type == FS_FAT16 ? "FAT16" : fs->fs_type == FS_FAT12 ? "FAT12" : "exFAT";
    clu = (uint32_t)fs->csize * 512u;
  }
  struct stat st;
  const bool dir = stat("/sdcard/ESP-CONFIG", &st) == 0 && S_ISDIR(st.st_mode);
  long c1 = -1, c2 = -1;
  if (stat("/sdcard/ESP-CONFIG" "/config.cfg", &st) == 0) c1 = (long)st.st_size;
  if (stat("/sdcard/ESP-CONFIG" "/joyconfig.cfg", &st) == 0) c2 = (long)st.st_size;
  LogOut.printf("[SD] carte %s, %llu Mo, %s, clusters %lu o, libre %lu Mo ; %s %s ; config.cfg %ld o, joyconfig.cfg %ld o (-1 = absent)\n",
                ct == CARD_SDHC ? "SDHC/SDXC" : ct == CARD_SD ? "SDSC" : ct == CARD_MMC ? "MMC" : "?",
                (unsigned long long)(SD_MMC.cardSize() / (1024ull * 1024ull)), fst, (unsigned long)clu,
                (unsigned long)((uint64_t)fre * clu / (1024ull * 1024ull)), "/ESP-CONFIG", dir ? "present" : "ABSENT", c1, c2);
#endif
}

static bool Vd_ReadRaw(DwUnit &u, uint32_t off, uint8_t *buf, uint32_t len)
{
  memset(buf, 0, len);
  if (off >= u.size) return true;                      // zone « creuse »
  if (off + len > u.size) len = u.size - off;
  bool ok = false;
  const uint64_t t0 = esp_timer_get_time();
  SD_LOCK();
#ifdef ESP_PLATFORM
  if (Vd_ReadMapped((uint8_t)(&u - g_dw), off, buf, len)) ok = true;   // lecture directe
  else if (u.f.seek(off)) { u.f.read(buf, len); ok = true; }
#else
  if (u.fp && fseek(u.fp, off, SEEK_SET) == 0) { size_t r = fread(buf, 1, len, u.fp); (void)r; ok = true; }
#endif
  SD_UNLOCK();
  g_vd_stat_sd++;
  g_vd_stat_sd_us += esp_timer_get_time() - t0;
  return ok;
}

static bool Dw_Read(uint8_t d, uint32_t lsn, uint8_t *buf)
{
  g_state_last_disk = (int8_t)(4 + d);
  DwUnit &u = g_dw[d];
  g_vd_stat_reads++;
  Vd_LedPulse();
  const uint32_t block = lsn / VD_CACHE_SECT;
  int victim = -1;
  for (int i = 0; i < VD_CACHE_LINES; i++)
  {
    VdCacheLine &c = s_vdc[i];
    if (c.valid && c.unit == d && c.block == block)
    {
      c.stamp = ++s_vdc_clock;
      memcpy(buf, c.data + (lsn % VD_CACHE_SECT) * 256u, 256);
      g_vd_stat_hits++;
      return true;
    }
    if (victim < 0) victim = i;
    else if (s_vdc[victim].valid && (!c.valid || c.stamp < s_vdc[victim].stamp)) victim = i;   // libre, sinon le plus ancien
  }
  VdCacheLine &c = s_vdc[victim];
  if (!c.data)
  {
#ifdef ESP_PLATFORM
    c.data = (uint8_t *)heap_caps_malloc(VD_CACHE_SECT * 256u, MALLOC_CAP_SPIRAM);
#else
    c.data = (uint8_t *)malloc(VD_CACHE_SECT * 256u);
#endif
  }
  if (!c.data)                                        // pas de mémoire : lecture directe
    return Vd_ReadRaw(u, lsn * 256u, buf, 256);
  c.valid = false;
  if (!Vd_ReadRaw(u, block * VD_CACHE_SECT * 256u, c.data, VD_CACHE_SECT * 256u)) return false;
  c.unit = d; c.block = block; c.valid = true; c.stamp = ++s_vdc_clock;
  memcpy(buf, c.data + (lsn % VD_CACHE_SECT) * 256u, 256);
  return true;
}

static bool Dw_Write(uint8_t d, uint32_t lsn, const uint8_t *buf)
{
  DwUnit &u = g_dw[d];
  const uint32_t off = lsn * 256u;
  bool ok = false;
  static const uint8_t zero[256] = { 0 };
  SD_LOCK();
#ifdef ESP_PLATFORM
  if (u.rw)
  {
    ok = true;
    if (off > u.size && u.f.seek(u.size))              // combler le trou par des zéros
    {
      uint32_t gap = off - u.size;
      while (gap && ok) { const uint32_t n = gap > 256u ? 256u : gap; ok = (u.f.write(zero, n) == n); gap -= n; }
    }
    ok = ok && u.f.seek(off) && (u.f.write(buf, 256) == 256);
    u.f.flush();
  }
#else
  if (u.fp)
  {
    ok = true;
    if (off > u.size) { fseek(u.fp, u.size, SEEK_SET); uint32_t gap = off - u.size; while (gap) { const uint32_t n = gap > 256u ? 256u : gap; fwrite(zero, 1, n, u.fp); gap -= n; } }
    ok = (fseek(u.fp, off, SEEK_SET) == 0) && (fwrite(buf, 1, 256, u.fp) == 256);
    fflush(u.fp);
  }
#endif
  if (ok && off + 256u > u.size) u.size = off + 256u;
  SD_UNLOCK();
  Vd_LedPulse();
  if (ok)                                              // garder le cache à jour
  {
    const uint32_t block = lsn / VD_CACHE_SECT;
    for (int i = 0; i < VD_CACHE_LINES; i++)
      if (s_vdc[i].valid && s_vdc[i].unit == d && s_vdc[i].block == block)
        memcpy(s_vdc[i].data + (lsn % VD_CACHE_SECT) * 256u, buf, 256);
  }
  return ok;
}

static void Dw_Time(uint8_t t[6])
{
  struct tm lt;                                        // heure NTP si le Wi-Fi l'a reçue
  if (Net_LocalTime(&lt))
  {
    t[0] = (uint8_t)lt.tm_year; t[1] = (uint8_t)(lt.tm_mon + 1); t[2] = (uint8_t)lt.tm_mday;
    t[3] = (uint8_t)lt.tm_hour; t[4] = (uint8_t)lt.tm_min; t[5] = (uint8_t)lt.tm_sec;
    return;
  }
  // Pas d'horloge temps réel sur la carte : date de compilation + temps écoulé.
  static const char *mon = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char *d = __DATE__, *tm = __TIME__;           // "Sep 28 2026", "11:35:00"
  int month = 1;
  for (int i = 0; i < 12; i++) if (!strncmp(d, mon + i * 3, 3)) month = i + 1;
  int day = atoi(d + 4), year = atoi(d + 7);
  uint32_t secs = (uint32_t)atoi(tm) * 3600u + (uint32_t)atoi(tm + 3) * 60u + (uint32_t)atoi(tm + 6)
                + (uint32_t)(esp_timer_get_time() / 1000000);
  static const uint8_t mdays[12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
  while (secs >= 86400u)
  {
    secs -= 86400u;
    int ml = mdays[month - 1] + ((month == 2 && (year % 4) == 0) ? 1 : 0);
    if (++day > ml) { day = 1; if (++month > 12) { month = 1; year++; } }
  }
  t[0] = (uint8_t)(year - 1900); t[1] = (uint8_t)month; t[2] = (uint8_t)day;
  t[3] = (uint8_t)(secs / 3600u); t[4] = (uint8_t)((secs / 60u) % 60u); t[5] = (uint8_t)(secs % 60u);
}

static const dw_backend_t s_dw_backend = { Dw_Present, Dw_Read, Dw_Write, Dw_Time };

static void Dw_UpdatePort(void)
{
  bool any = false;
  for (int i = 0; i < DW_DRIVES; i++) any = any || g_dw[i].mounted;
  dw_set_port_enabled(any);
}

void DW_Unmount(uint8_t d)
{
  if (d >= VD_UNITS) return;
  DwUnit &u = g_dw[d];
  SD_LOCK();
#ifdef ESP_PLATFORM
  if (u.mounted) u.f.close();
  Vd_MapFree(d);
#else
  if (u.fp) { fclose(u.fp); u.fp = nullptr; }
#endif
  u.mounted = false;
  u.size = 0;
  SD_UNLOCK();
  Vd_CacheInvalidate(d);
  g_dw_name[d][0] = 0;
  Dw_UpdatePort();
}

bool DW_Mount(uint8_t d, const char *path)
{
  static bool s_init = false;
  if (!s_init) { dw_init(&s_dw_backend); s_init = true; }
  if (d >= VD_UNITS) return false;
  DW_Unmount(d);
  if (!path || !path[0]) return false;
  DwUnit &u = g_dw[d];
  bool rw = true;
  SD_LOCK();
#ifdef ESP_PLATFORM
  u.f = SD_MMC.open(path, "r+");
  if (!u.f) { u.f = SD_MMC.open(path, FILE_READ); rw = false; }
  if (u.f && u.f.isDirectory()) u.f.close();
  u.mounted = (bool)u.f;
  u.rw = rw;
  u.size = u.mounted ? (uint32_t)u.f.size() : 0;
  if (u.mounted) Vd_MapBuild(d, path, u.size);         // lecture directe
#else
  u.fp = fopen(path, "r+b");
  if (!u.fp) { u.fp = fopen(path, "rb"); rw = false; }
  u.mounted = (u.fp != nullptr);
  if (u.mounted) { fseek(u.fp, 0, SEEK_END); u.size = (uint32_t)ftell(u.fp); }
#endif
  SD_UNLOCK();
  if (!u.mounted)
  {
    if (d >= VHD_UNIT0) LogOut.printf("[VHD] disque %u : impossible d'ouvrir %s\n", d - VHD_UNIT0, path);
    else LogOut.printf("[DW] /x%u : impossible d'ouvrir %s\n", d, path);
    return false;
  }
  snprintf(g_dw_name[d], sizeof g_dw_name[d], "%s", path);
  uint8_t l0[256];
  const int8_t keep_last = g_state_last_disk;       // lecture du menu, pas du CoCo
  Dw_Read(d, 0, l0);
  g_state_last_disk = keep_last;
  const uint32_t tot = ((uint32_t)l0[0] << 16) | ((uint32_t)l0[1] << 8) | l0[2];
  BOOTLOG("[%s] %s%u <- %s : fichier %u o, volume OS-9 de %u secteurs (%u Ko)%s\n",
                d >= VHD_UNIT0 ? "VHD" : "DW", d >= VHD_UNIT0 ? "disque " : "/x", d >= VHD_UNIT0 ? d - VHD_UNIT0 : d,
                path, (unsigned)u.size, (unsigned)tot, (unsigned)(tot / 4u), rw ? "" : ", lecture seule");
  Dw_UpdatePort();
  return true;
}

// Amorçage NitrOS-9 depuis /x0, comme la commande DOS de HDB-DOS : piste « noyau » (LSN 612-629, 18
// secteurs) chargée en $2600, contrôle de la signature « OS », saut en $2602
// ---------------------------------------------------------------------------
// disque dur virtuel « VHD » (VCC / MAME coco_vhd.cpp, BSD-3) $FF80-$FF82 numéro de secteur (24 bits,
// poids fort d'abord) $FF83 écriture = commande (0 lire, 1 écrire, 2 vider le cache) lecture = état…
// ---------------------------------------------------------------------------
static uint32_t s_vhd_lsn = 0;
static uint16_t s_vhd_buf = 0;
static uint8_t  s_vhd_status = 0xFF;
static uint8_t  s_vhd_sel = 0;
bool VHD_Enabled(void) { return g_dw[VHD_UNIT0].mounted || g_dw[VHD_UNIT0 + 1].mounted; }
uint8_t VHD_Read(uint16_t a)
{
  return (a == 0xFF83) ? s_vhd_status : 0x00;
}
void VHD_Write(uint16_t a, uint8_t v)
{
  switch (a)
  {
    case 0xFF80: s_vhd_lsn = (s_vhd_lsn & 0x00FFFF) | ((uint32_t)v << 16); break;
    case 0xFF81: s_vhd_lsn = (s_vhd_lsn & 0xFF00FF) | ((uint32_t)v << 8);  break;
    case 0xFF82: s_vhd_lsn = (s_vhd_lsn & 0xFFFF00) | v;                   break;
    case 0xFF84: s_vhd_buf = (uint16_t)((s_vhd_buf & 0x00FF) | (v << 8));  break;
    case 0xFF85: s_vhd_buf = (uint16_t)((s_vhd_buf & 0xFF00) | v);         break;
    case 0xFF86: s_vhd_sel = v;                                            break;
    case 0xFF83:
    {
      const uint8_t unit = VHD_UNIT0 + s_vhd_sel;
      if (s_vhd_sel > 1 || !g_dw[unit].mounted) { s_vhd_status = 0x02; break; }
      uint8_t b[256];
      if (v == 0)
      {
        if (!Dw_Read(unit, s_vhd_lsn, b)) { s_vhd_status = 0x05; break; }
        for (uint16_t i = 0; i < 256; i++) gime_poke((uint16_t)(s_vhd_buf + i), b[i]);
        s_vhd_status = 0x00;
      }
      else if (v == 1)
      {
        for (uint16_t i = 0; i < 256; i++) b[i] = gime_peek((uint16_t)(s_vhd_buf + i));
        s_vhd_status = Dw_Write(unit, s_vhd_lsn, b) ? 0x00 : 0x05;
      }
      else if (v == 2) s_vhd_status = 0x00;
      else             s_vhd_status = 0xFE;
      break;
    }
    default: break;
  }
}

extern cpu_t *cpu;
static void Dw_BootNow(void)
{
  g_dw_boot_request = false;
  if (!Dw_Present(0)) { LogOut.println("[DW] amorcage : aucun disque en /x0"); return; }
  uint8_t sec[256];
  Dw_Read(0, 612, sec);
  if (sec[0] != 'O' || sec[1] != 'S')
  {
    LogOut.println("[DW] amorcage : pas de piste noyau (signature OS absente au LSN 612)");
    return;
  }
  gime_write_register(0xFF90, 0xCC);
  gime_write_register(0xFF91, 0x00);
  for (uint16_t i = 0; i < 8; i++)
  {
    gime_write_register(0xFFA0 + i, 0x38 + i);
    gime_write_register(0xFFA8 + i, 0x38 + i);
  }
  gime_set_ty(true);                       // SAM TY = 1 : tout-RAM ($FFDF)
  for (uint16_t s = 0; s < 18; s++)
  {
    Dw_Read(0, 612u + s, sec);
    for (uint16_t i = 0; i < 256; i++) gime_poke(0x2600 + s * 256 + i, sec[i]);
  }
  dw_reset();
  cpu->set_cc(cpu->get_cc() | 0x50);
  cpu->set_pc(0x2602);
  LogOut.println("[DW] amorcage NitrOS-9 : piste noyau chargee en $2600, saut en $2602");
}

// Réglages vidéo lus dans config.ccc (appliqués après les valeurs par défaut de
// InitPeripherals_and_Others, qui passe APRÈS la lecture).
static bool g_cfg_video_loaded = false;
static bool g_cfg_monitor_rgb  = false;
static bool g_cfg_artefact     = true;
static bool g_cfg_artifact_swap = false;
static uint8_t g_cfg_overclock = OC_OFF;  // 0 OFF, 1 Machine (anciens fichiers), 2 CPU
static uint32_t g_cfg_ram      = 0;       // RAM choisie (0 = défaut 512 Ko)
static char g_cfg_dw_name[4][256];         // /x0, /x1, VHD 0, VHD 1 sauvegardés
bool g_pc_caps = false;
volatile uint32_t g_caps_save_ms = 0;      // Verr. Maj. changé -> sauvegarde différée (0 = rien)
bool g_lowercase_t1 = false;
bool g_cpu6309_sel = false;                // processeur choisi (menu C), appliqué au démarrage
bool g_cpu6309_run = false;                // processeur en service (HD6309 si vrai)

// configuration rangée dans /ESP-CONFIG (à part des disques des utilisateurs). Lecture :
// /ESP-CONFIG/x, sinon sa copie de secours x.bak sinon l'ancien emplacement à la racine (migration)
#define CFG_DIR "/ESP-CONFIG"
// réglages renommés .cfg (.CCC = image de cartouche dans le monde CoCo, ex. VCC). Lecture : nouveau
// nom (puis sa copie .bak), sinon l'ancien nom .ccc (dans /ESP-CONFIG, sa copie .bak, puis la racine)
static bool s_cfg_legacy_used = false;
static File Cfg_TryOpen(const char *path)
{
  File f = SD_MMC.open(path, FILE_READ);
  if (f && f.size() > 0) return f;
  if (f) f.close();
  return File();
}
static File Cfg_OpenRead(const char *name, const char *legacy)
{
  char p[48];
  snprintf(p, sizeof p, CFG_DIR "/%s", name);
  File f = Cfg_TryOpen(p);
  if (f) return f;
  snprintf(p, sizeof p, CFG_DIR "/%s.tmp", name);   // écriture sûre interrompue avant le renommage
  f = Cfg_TryOpen(p);
  if (f) { LogOut.printf("[CONFIG] %s absent : fichier .tmp utilise\n", name); return f; }
  snprintf(p, sizeof p, CFG_DIR "/%s.bak", name);
  f = Cfg_TryOpen(p);
  if (f) { LogOut.printf("[CONFIG] %s illisible : copie de secours utilisee\n", name); return f; }
  const char *k_fmt[3] = { CFG_DIR "/%s", CFG_DIR "/%s.bak", "/%s" };
  for (int i = 0; i < 3; i++)
  {
    snprintf(p, sizeof p, k_fmt[i], legacy);
    f = Cfg_TryOpen(p);
    if (f)
    {
      LogOut.printf("[CONFIG] ancien fichier %s lu : converti en %s\n", p, name);
      s_cfg_legacy_used = true;
      return f;
    }
  }
  return File();
}
// fichier existant réécrit sur place ("r+", même taille) : aucun cluster à allouer. recréé ; la
// première création après le démarrage faisait chercher un cluster libre dans toute la FAT
static File Cfg_OpenWrite(const char *name)
{
  char p[48];
  if (!SD_MMC.exists(CFG_DIR)) SD_MMC.mkdir(CFG_DIR);
  snprintf(p, sizeof p, CFG_DIR "/%s", name);
  File f = SD_MMC.open(p, "r+");
  if (f && !f.isDirectory()) { f.seek(0); return f; }
  if (f) f.close();
  return SD_MMC.open(p, FILE_WRITE);
}
// copie de secours <nom>.bak du fichier lu au démarrage (sur place si la copie existe déjà). Rien si
// le fichier principal est absent ou vide.
static bool Cfg_SafeWrite(const char *name, const uint8_t *buf, size_t len);
static void Cfg_Backup(const char *name)
{
  char p[48], b[48];
  snprintf(p, sizeof p, CFG_DIR "/%s", name);
  snprintf(b, sizeof b, CFG_DIR "/%s.bak", name);
  File src = Cfg_TryOpen(p);
  if (!src) return;
  const size_t n = src.size();
  uint8_t *buf = (n <= 16384) ? (uint8_t *)heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : nullptr;
  const bool ok = buf && src.read(buf, n) == n;
  src.close();
  if (ok)
  {
    char nb[40];
    snprintf(nb, sizeof nb, "%s.bak", name);
    Cfg_SafeWrite(nb, buf, n);                 // copie de secours en écriture sûre aussi
  }
  free(buf);
}

// écriture sûre d'un petit fichier de réglages. gardait son ancien contenu (relu différent), même
// effacé puis recréé
#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>
#ifdef ESP_PLATFORM
static const char *Cfg_VfsRoot(void) { return "/sdcard"; }
#else
static const char *Cfg_VfsRoot(void) { const char *r = getenv("SDROOT"); return r ? r : "/tmp/claude-0/sdroot"; }   // banc d'essai
#endif
static bool Cfg_ReadBackEq(const char *vpath, const uint8_t *buf, size_t len)
{
  FILE *r = fopen(vpath, "rb");
  if (!r) return false;
  uint8_t tmp[64];
  size_t done = 0; bool same = true;
  while (done < len && same)
  {
    const size_t k = (len - done) < sizeof tmp ? (len - done) : sizeof tmp;
    if (fread(tmp, 1, k, r) != k || memcmp(tmp, buf + done, k) != 0) same = false;
    done += k;
  }
  if (same && fgetc(r) != EOF) same = false;   // pas plus long
  fclose(r);
  return same;
}
static bool Cfg_ReadBackPath(const char *name, const uint8_t *buf, size_t len)
{
  char vp[192];
  snprintf(vp, sizeof vp, "%s" CFG_DIR "/%s", Cfg_VfsRoot(), name);
  return Cfg_ReadBackEq(vp, buf, len);
}
static bool Cfg_SafeWrite(const char *name, const uint8_t *buf, size_t len)
{
  char vp[192], vt[192], vd[160];
  snprintf(vd, sizeof vd, "%s" CFG_DIR, Cfg_VfsRoot());
  snprintf(vp, sizeof vp, "%s" CFG_DIR "/%s", Cfg_VfsRoot(), name);
  snprintf(vt, sizeof vt, "%s" CFG_DIR "/%s.tmp", Cfg_VfsRoot(), name);
  // dossier créé en POSIX (errno visible) s'il manque
  struct stat st;
  if (stat(vd, &st) != 0 || !S_ISDIR(st.st_mode))
  {
    errno = 0;
    const int r = mkdir(vd, 0777);
    LogOut.printf("[CONFIG] dossier %s absent : creation %s (errno %d)\n", CFG_DIR, r == 0 ? "OK" : "ECHEC", r == 0 ? 0 : errno);
  }
  bool ok = false;
  for (int attempt = 1; attempt <= 3 && !ok; attempt++)   // jusqu'à 3 essais
  {
    int e_open = 0, e_wr = 0, e_fl = 0, e_sy = 0, e_cl = 0, e_rn = 0;
    unlink(vt);
    errno = 0;
    FILE *f = fopen(vt, "wb");
    if (!f) e_open = errno ? errno : -1;
    else
    {
      if (fwrite(buf, 1, len, f) != len) e_wr = errno ? errno : -1;
      if (fflush(f) != 0) e_fl = errno ? errno : -1;
      if (fsync(fileno(f)) != 0) e_sy = errno ? errno : -1;
      if (fclose(f) != 0) e_cl = errno ? errno : -1;
    }
    const bool tmpOk = f && !e_wr && !e_fl && !e_sy && !e_cl && Cfg_ReadBackEq(vt, buf, len);
    if (tmpOk)
    {
      unlink(vp);
      if (rename(vt, vp) != 0) e_rn = errno ? errno : -1;
      ok = !e_rn && Cfg_ReadBackEq(vp, buf, len);
    }
    if (!ok) LogOut.printf("[CONFIG] %s : ecriture sure %s, essai %d (ouverture %d, ecriture %d, vidage %d, fsync %d, fermeture %d, relu .tmp %s, renommage %d)\n",
                  name, ok ? "OK" : "ECHEC", attempt, e_open, e_wr, e_fl, e_sy, e_cl, tmpOk ? "OK" : "NON", e_rn);
    if (!ok) unlink(vt);
    if (e_open) break;                          // ouverture impossible : inutile de réessayer
  }
  return ok;
}

// tampon d'écriture de config.cfg (même interface que File::write)
struct CfgBuf
{
  uint8_t *buf; size_t len = 0, cap = 4096; bool over = false;
  CfgBuf() { buf = (uint8_t *)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); }
  ~CfgBuf() { free(buf); }
  bool ok() const { return buf != nullptr; }
  size_t write(const uint8_t *d, size_t n) { if (!buf || len + n > cap) { over = true; return 0; } memcpy(buf + len, d, n); len += n; return n; }
  size_t write(const char *d, size_t n) { return write((const uint8_t *)d, n); }
  void close() {}
};

bool SaveConfigToSD(void)
{
    const uint32_t t0 = millis();           // durée (O lent juste après l'ouverture du menu ?)
    CfgBuf f;                               // contenu bâti en mémoire, puis écriture sûre
    if (!f.ok()) {
        LogOut.println("config.cfg : memoire insuffisante, non sauvegarde");
        return false;
    }

    for (int i = 0; i < 4; i++)
    {
        size_t written = f.write(Disk_Drive.Name_Disk[i], 256);
        if (written != 256)
        {
            LogOut.printf("Write failed for disk %d\n", i);
            f.close();
            return false;
        }
    }

    // bloc vidéo ajouté après les 4 noms de disque (un ancien config.ccc de 1024 octets reste lisible,
    // voir LoadConfigFromSD).
    uint8_t video[16] = { 'C', 'C', '3', 'V', 2,
                         (uint8_t)(gime_video_monitor_rgb() ? 1 : 0),
                         (uint8_t)(sf.Artefact ? (g_artifact_swap ? 2 : 1) : 0),   // 2 = phase inversée
                         (uint8_t)(g_oc_mode == OC_MACHINE ? 1 : (g_oc_mode == OC_CPU ? 2 : 0)),  // octet 7 : 0 OFF, 1 Machine, 2 CPU
                         (uint8_t)(g_ram_selected / (512u * 1024u)),
                         (uint8_t)(g_lowercase_t1 ? 1 : 0),           // octet 9 : minuscules 32 col.
                         g_mouse_mode,                                // octet 10 : souris
                         (uint8_t)(g_mouse_sens | (g_mouse_hires_pref ? 0x10 : 0)),   // octet 11 : sensibilité + résolution préférée
                         (uint8_t)(0xC0 | (g_kbjoy_port & 3) | ((g_kbjoy_set2 & 3) << 2) | ((g_kbjoy_set & 3) << 4)),   // octet 12 : manette au clavier
                         (uint8_t)(0x80 | (gime_is_1987() ? 1 : 0)),                 // octet 13 : version du GIME
                         (uint8_t)(0x80 | (g_pc_caps ? 1 : 0)),                      // octet 14 : Verr. Maj. interne, option L
                         (uint8_t)(0x80 | (g_cpu6309_sel ? 1 : 0)) };                // octet 15 : processeur 6809 / 6309
    f.write(video, sizeof(video));

    // disques virtuels DriveWire /x0 et /x1
    const uint8_t tag[4] = { 'C', 'C', '3', 'W' };
    f.write(tag, sizeof(tag));
    f.write((const uint8_t *)g_dw_name[0], 256);
    f.write((const uint8_t *)g_dw_name[1], 256);
    // disques durs virtuels VHD 0 et 1
    const uint8_t tagh[4] = { 'C', 'C', '3', 'H' };
    f.write(tagh, sizeof(tagh));
    f.write((const uint8_t *)g_dw_name[VHD_UNIT0], 256);
    f.write((const uint8_t *)g_dw_name[VHD_UNIT0 + 1], 256);
    // port d'extension (mode, démarrage automatique, cartouche)
    const uint8_t tags[8] = { 'C', 'C', '3', 'S', g_slotcfg.mode[0], (uint8_t)(g_slotcfg.autostart[0] ? 1 : 0), g_rs_link, g_rs_speed };   // octets 6-7 = liaison / vitesse RS-232
    f.write(tags, sizeof(tags));
    f.write((const uint8_t *)g_slotcfg.name[0], 256);
    // Multi-Pak (actif, commutateur, slots 1-3)
    const uint8_t tagm[12] = { 'C', 'C', '3', 'M', g_slotcfg.mpi, g_slotcfg.sw,
                               g_slotcfg.mode[1], g_slotcfg.mode[2], g_slotcfg.mode[3],
                               g_slotcfg.autostart[1], g_slotcfg.autostart[2], g_slotcfg.autostart[3] };
    f.write(tagm, sizeof(tagm));
    for (int i = 1; i < MPI_SLOTS; i++) f.write((const uint8_t *)g_slotcfg.name[i], 256);
    // sortie son (0 = mono, 1 = stéréo GPIO 47 + 12)
    const uint8_t taga[8] = { 'C', 'C', '3', 'A', (uint8_t)(Audio_GetStereo() ? 1 : 0), (uint8_t)(g_bb_mode + 1), g_bb_speed, 0 };   // octets 5-6 = bit banger (mode + 1, vitesse)
    f.write(taga, sizeof(taga));
    // cassette (chargement rapide, format des nouvelles bandes, bande en lecture)
    const uint8_t tagk[8] = { 'C', 'C', '3', 'K', (uint8_t)(Cas_GetFast() ? 1 : 0), Cas_GetFormat(), (uint8_t)(Cas_Mode() == CAS_PLAY ? 1 : 0), 0 };
    f.write(tagk, sizeof(tagk));
    { char nm[256]; memset(nm, 0, sizeof nm); if (Cas_Mode() == CAS_PLAY) snprintf(nm, sizeof nm, "%s", Cas_Name()); f.write((const uint8_t *)nm, 256); }

    // écriture sûre (.tmp sur un autre cluster, erreurs vérifiées, relu renommé) ; repli : ancienne
    // écriture sur place, relue.
    bool ok = !f.over && Cfg_SafeWrite("config.cfg", f.buf, f.len);
    if (!ok && !f.over)
    {
      File w = Cfg_OpenWrite("config.cfg");
      if (w) { w.write(f.buf, f.len); w.close(); }
      ok = w && Cfg_ReadBackPath("config.cfg", f.buf, f.len);
    }
    if (!ok) LogOut.println("Configuration NOT SAVED : SD write lost (check the card on a PC)");
    return ok;
}

bool LoadConfigFromSD(void)
{
    File f = Cfg_OpenRead("config.cfg", "config.ccc");
    if (!f)
    {
        LogOut.println("No " CFG_DIR "/config.cfg yet, using defaults");
        return false;
    }

    for (int i = 0; i < 4; i++)
    {
        size_t readBytes = f.read(Disk_Drive.Name_Disk[i], 256);
        if (readBytes != 256)
        {
            LogOut.printf("Read failed for disk %d\n", i);
            f.close();
            return false;
        }
    }

    uint8_t video[8];
    if (f.read(video, sizeof(video)) == sizeof(video) &&
        video[0] == 'C' && video[1] == 'C' && video[2] == '3' && video[3] == 'V')
    {
      g_cfg_video_loaded = true;
      g_cfg_monitor_rgb  = video[5] != 0;
      g_cfg_artefact     = video[6] != 0;
      g_cfg_artifact_swap = video[6] == 2;
      g_cfg_overclock    = (video[7] == 1) ? OC_MACHINE : (video[7] == 2 ? OC_CPU : OC_OFF);
      uint8_t v2[8] = { 0 };
      if (video[4] >= 2 && f.read(v2, sizeof(v2)) == sizeof(v2) &&
          (v2[0] == 1 || v2[0] == 4 || v2[0] == 8))
      {
        g_cfg_ram = (uint32_t)v2[0] * 512u * 1024u;
      }
      if (video[4] >= 2) g_lowercase_t1 = (v2[1] == 1);
      if (video[4] >= 2 && v2[2] <= 6) g_mouse_mode = v2[2];
      if (video[4] >= 2 && (v2[3] & 0x0F) >= 1 && (v2[3] & 0x0F) <= 8)
      {
        g_mouse_sens = v2[3] & 0x0F;
        g_mouse_hires_pref = (v2[3] & 0x10) != 0;
      }
      if (video[4] >= 2 && (v2[4] & 0xC0) == 0xC0)                    // port(s) et jeu de chaque port
      {
        g_kbjoy_port = v2[4] & 3;
        g_kbjoy_set2 = (v2[4] >> 2) & 3; if (g_kbjoy_set2 > 2) g_kbjoy_set2 = 0;
        g_kbjoy_set  = (v2[4] >> 4) & 3; if (g_kbjoy_set > 2) g_kbjoy_set = 0;
      }
      else if (video[4] >= 2 && (v2[4] & 0x80) && (v2[4] & 0x0F) <= 2 && ((v2[4] >> 4) & 0x07) <= 2)
      {
        g_kbjoy_port = v2[4] & 0x0F;
        g_kbjoy_set  = (v2[4] >> 4) & 0x07;
        g_kbjoy_set2 = g_kbjoy_set;
      }
      if (video[4] >= 2 && (v2[5] & 0x80)) gime_set_1987((v2[5] & 1) != 0);   // GIME 1986 / 1987
      if (video[4] >= 2 && (v2[6] & 0x80)) g_pc_caps = (v2[6] & 1) != 0;       // Verr. Maj. (option L)
      if (video[4] >= 2 && (v2[7] & 0x80)) g_cpu6309_sel = (v2[7] & 1) != 0;   // MC6809 / HD6309
      uint8_t tag[4];
      if (video[4] >= 2 && f.read(tag, 4) == 4 && tag[0] == 'C' && tag[1] == 'C' && tag[2] == '3' && tag[3] == 'W')
      {
        f.read((uint8_t *)g_cfg_dw_name[0], 256);
        f.read((uint8_t *)g_cfg_dw_name[1], 256);
        g_cfg_dw_name[0][255] = g_cfg_dw_name[1][255] = 0;
        uint8_t tagh[4];
        if (f.read(tagh, 4) == 4 && tagh[0] == 'C' && tagh[1] == 'C' && tagh[2] == '3' && tagh[3] == 'H')
        {
          f.read((uint8_t *)g_cfg_dw_name[2], 256);    // VHD 0 / 1
          f.read((uint8_t *)g_cfg_dw_name[3], 256);
          g_cfg_dw_name[2][255] = g_cfg_dw_name[3][255] = 0;
          uint8_t tags[8];
          if (f.read(tags, 8) == 8 && tags[0] == 'C' && tags[1] == 'C' && tags[2] == '3' && tags[3] == 'S' &&
              tags[4] <= SLOT_LAST && f.read((uint8_t *)g_slotcfg.name[0], 256) == 256)
          {
            g_slotcfg.name[0][255] = 0;
            g_slotcfg.mode[0] = tags[4];
            LogOut.printf("[CONFIG] lu : port d'extension %u%s%s\n", tags[4], g_slotcfg.name[0][0] ? ", " : "", g_slotcfg.name[0]);
            g_slotcfg.autostart[0] = (tags[5] & 1);
            g_rs_link  = (tags[6] < RS_LINK_COUNT) ? tags[6] : RS_LINK_UART;
            g_rs_speed = (tags[7] < RS_SPEED_COUNT) ? tags[7] : 0;
            if (tags[6] == 0 && tags[7] == 0) g_rs_link = RS_LINK_UART;       // ancien format : défaut UART
            uint8_t tagm[12];
            if (f.read(tagm, 12) == 12 && tagm[0] == 'C' && tagm[1] == 'C' && tagm[2] == '3' && tagm[3] == 'M')
            {
              g_slotcfg.mpi = tagm[4] & 1;
              g_slotcfg.sw  = tagm[5] & 3;
              for (int i = 1; i < MPI_SLOTS; i++)
              {
                g_slotcfg.mode[i] = (tagm[5 + i] <= SLOT_LAST) ? tagm[5 + i] : SLOT_EMPTY;
                g_slotcfg.autostart[i] = tagm[8 + i] & 1;
                if (f.read((uint8_t *)g_slotcfg.name[i], 256) != 256) g_slotcfg.name[i][0] = 0;
                g_slotcfg.name[i][255] = 0;
              }
              uint8_t taga[8];
              if (f.read(taga, 8) == 8 && taga[0] == 'C' && taga[1] == 'C' && taga[2] == '3' && taga[3] == 'A')
                Audio_SetStereo(taga[4] & 1);
                if (taga[5] >= 1 && taga[5] <= BB_MODE_COUNT) g_bb_mode = (uint8_t)(taga[5] - 1);
                if (taga[6] < BB_SPEED_COUNT) g_bb_speed = taga[6];
              uint8_t tagk[8] = { 0 }; char knm[256];                 // cassette
              if (f.read(tagk, 8) == 8 && tagk[0] == 'C' && tagk[1] == 'C' && tagk[2] == '3' && tagk[3] == 'K' && f.read((uint8_t *)knm, 256) == 256)
              {
                knm[255] = 0;
                Cas_SetFast(tagk[4] & 1); Cas_SetFormat(tagk[5]);
                if ((tagk[6] & 1) && knm[0] == '/') { const bool ok = Cas_Insert(knm); BOOTLOG("[CAS] bande %s : %s\n", knm, ok ? "remise en place (au debut)" : "introuvable"); }
              }
            }
          }
        }
      }
    }

    f.close();
    BOOTLOG("Configuration loaded from SD (video: %s, overclock: %s)\n",
                  g_cfg_video_loaded ? (g_cfg_monitor_rgb ? "RGB" : "CMP") : "defaut",
                  g_cfg_video_loaded ? (g_cfg_overclock == OC_MACHINE ? "MACHINE" : (g_cfg_overclock == OC_CPU ? "CPU" : "OFF")) : "defaut");
    return true;
}

// =====================================================================
// CONFIGURATION DES MANETTES USB (Right/Left) -- voir EmuMenu.h pour la définition de
// JoystickDeviceConfig et l'explication du partage entre main.cpp (propriétaire des données + lecture
// USB) et…
// =====================================================================
JoystickDeviceConfig JoystickCfg[2] = {}; // [JOY_SLOT_RIGHT], [JOY_SLOT_LEFT]

// Valeurs "factory" identifiées manuellement sont utilisées par défaut tant que l'utilisateur n'a pas
// configuré explicitement CETTE manette via le menu J.
struct JoystickBuiltinDefault
{
  uint16_t vid, pid;
  uint8_t xByte, yByte, btnByte, fire1Mask, fire2Mask;
};
static const JoystickBuiltinDefault JoystickBuiltins[] = {
  {0x0810, 0x0001, 3, 4, 5, 0x10, 0x20}, // manette générique (mode Analog Off testé)
  {0x081f, 0xe401, 0, 1, 5, 0x10, 0x20}, // joypad générique
};
#define JOYSTICK_BUILTIN_COUNT (sizeof(JoystickBuiltins)/sizeof(JoystickBuiltins[0]))

// réglages des deux ports manette (journal série)
static void Joy_LogConfig(const char *what)
{
  for (uint8_t s = 0; s != 2; s++)
  {
    const JoystickDeviceConfig &c = JoystickCfg[s];
    if (!c.configured) { LogOut.printf("[JOY] %s : port %s non regle\n", what, s == JOY_SLOT_RIGHT ? "droit (R)" : "gauche (L)"); continue; }
    const uint8_t n = c.mode >> 4;
    char pad[16];
    if (n) snprintf(pad, sizeof pad, "manette %u", n); else snprintf(pad, sizeof pad, "toute interface");
    LogOut.printf("[JOY] %s : port %s = %04x:%04x %s, %s, X octet %u, Y octet %u, tir 1 octet %u/$%02X, tir 2 octet %u/$%02X\n", what,
                  s == JOY_SLOT_RIGHT ? "droit (R)" : "gauche (L)", c.vid, c.pid, pad, (c.mode & 0x0F) ? "analogique" : "numerique",
                  c.xByte, c.yByte, c.fire1Byte, c.fire1Mask, c.fire2Byte, c.fire2Mask);
  }
}

bool SaveJoystickConfigToSD(void)
{
    // écriture sûre (.tmp sur un autre cluster, erreurs vérifiées, relecture)
    Joy_LogConfig("sauvegarde");
    if (Cfg_SafeWrite("joyconfig.cfg", (const uint8_t *)&JoystickCfg[0], sizeof(JoystickCfg)))
    {
        LogOut.println("Joystick configuration saved to SD");
        return true;
    }
    // Repli : ancienne méthode (sur place), relue.
    File f = Cfg_OpenWrite("joyconfig.cfg");
    if (!f)
    {
        LogOut.println("Failed to open " CFG_DIR "/joyconfig.cfg for writing");
        return false;
    }
    size_t written = f.write((uint8_t*)&JoystickCfg[0], sizeof(JoystickCfg));
    f.close();
    const bool ok = written == sizeof(JoystickCfg) &&
                    Cfg_ReadBackPath("joyconfig.cfg", (const uint8_t *)&JoystickCfg[0], sizeof(JoystickCfg));
    if (!ok) LogOut.println("Joystick configuration NOT SAVED : SD write lost (check the card on a PC)");
    return ok;
}

bool LoadJoystickConfigFromSD(void)
{
    File f = Cfg_OpenRead("joyconfig.cfg", "joyconfig.ccc");
    if (!f)
    {
        // Pas encore de fichier (premier démarrage / pas encore configuré) Pas une erreur, on reste
        // sur JoystickCfg[] à zéro/ configured=false, ce qui fait retomber sur les valeurs factory.
        memset(JoystickCfg, 0, sizeof(JoystickCfg));
        LogOut.println("No joyconfig.cfg yet, using builtin joystick defaults");
        return false;
    }
    size_t readBytes = f.read((uint8_t*)&JoystickCfg[0], sizeof(JoystickCfg));
    f.close();
    if (readBytes != sizeof(JoystickCfg))
    {
        LogOut.println("Read failed for joyconfig.cfg, using builtin joystick defaults");
        memset(JoystickCfg, 0, sizeof(JoystickCfg));
        return false;
    }
    BOOTLOGLN("Joystick configuration loaded from SD");
    Joy_LogConfig("lu");
    return true;
}

// cpu_t::read8/write8 accède directement à memory[]/rom[] (alloués en PSRAM)
cpu_t *cpu = nullptr;

void Cpu_Report(void)   // ligne [CPU] de W
{
  if (!cpu) return;
  if (!cpu->is_6309())
  {
    LogOut.printf("[CPU] MC6809%s\n", g_cpu6309_sel ? " (HD6309 choisi : R pour redemarrer)" : "");
    return;
  }
  LogOut.printf("[CPU] HD6309 mode %s, MD=$%02X ; pieges $FFF0 (opcode illegal / division par 0) : %lu, dernier PC empile $%04X%s\n",
                cpu->is_native() ? "NATIF" : "emulation", cpu->get_md(), (unsigned long)cpu->traps63, cpu->trap_pc63,
                g_cpu6309_sel ? "" : " (MC6809 choisi : R pour redemarrer)");
}

// ---------------------------------------------------------------------------
// ce sont des OU câblés (actifs bas) entre le PIA (VSYNC, etc. : sf.irq_pin / sf.firq_pin) et les
// sorties IRQ/FIRQ du GIME (timer, HBORD, VBORD).
// ---------------------------------------------------------------------------
bool cpu_irq_line  = true;   // true = inactive
bool cpu_firq_line = true;

// Exécute UNE instruction, fait avancer le GIME du nombre de cycles consommés (balayage émulé, timer,
// HBORD/VBORD) puis recalcule les lignes combinées
uint8_t  g_oc_mode = OC_OFF;
static uint16_t g_last_gime_cycles = 0;
// ~5 MHz. Mesuré sur la carte : MACHINE atteint 7,1-7,7 MHz CPU à ~2,5 MHz tenait 60 Hz sans effort ->
// marge suffisante à 5 MHz.
#define OC_CPU_DIV_SLOW_Q8  1430  // 0,89 MHz x 5,6 = ~5 MHz
#define OC_CPU_DIV_FAST_Q8  715   // 1,79 MHz x 2,8 = ~5 MHz
static void Oc_UpdateDivider(void)
{
  if (g_oc_mode == OC_CPU)
    g_gime_div_max = (sf.CPU_Speed == CPU_FAST) ? OC_CPU_DIV_FAST_Q8 : OC_CPU_DIV_SLOW_Q8;
  else
    g_gime_div_max = 256;
  g_gime_div_q8 = g_gime_div_max;
}
// Overclock CPU ADAPTATIF. Le multiplicateur fixe (x5,6 en SLOW x2,8 en FAST) supposait que l'ESP32
// tiendrait ~5 MHz. Sinon le temps du GIME (avancé de cycles CPU / multiplicateur) prenait du retard
static volatile uint32_t g_oc_busy_cc = 0;   // cycles ESP32 passés à émuler (cumul)
static void Oc_Adapt(int32_t carry)
{
  static uint32_t s_b0 = 0, s_t0 = 0;
  const uint32_t b = g_oc_busy_cc, t = Perf_CCount();
  const uint32_t db = b - s_b0, dt = t - s_t0;
  s_b0 = b; s_t0 = t;
  if (g_oc_mode != OC_CPU || dt < 1000000u || dt > 40000000u) return;   // 1re trame, pause
  const uint32_t load_q8 = (uint32_t)(((uint64_t)db << 8) / dt);
  uint32_t d = g_gime_div_q8;
  if (carry > 2000)        d -= d / 8;
  else if (load_q8 > 235)  d -= d / 32;          // > 92 %
  else if (load_q8 < 210)  d += d / 64 + 1;      // < 82 %
  if (d < 256) d = 256;
  if (d > g_gime_div_max) d = g_gime_div_max;
  g_gime_div_q8 = (uint16_t)d;
}
void SetOverclockMode(uint8_t mode)
{
  g_oc_mode = (mode > OC_MACHINE) ? OC_OFF : mode;
  sf.CycleAccurateMode = (g_oc_mode != OC_MACHINE);
  { const bool rt = (g_oc_mode == OC_MACHINE);
    if (rt != g_audio_realtime) Audio_Flush();      // autre base de temps : la file du son est vidée (Turbo)
    g_audio_realtime = rt; }
  Oc_UpdateDivider();
  g_resync_cpu_clock = true;
}
static inline void Fdc_Tick(uint16_t c);   // contrôleur de disquette
static void Fdc_Reset(void);
static inline bool Fdc_HaltLine(void);
volatile uint32_t g_cpu_wait_cycles = 0;   // cycles 6809 passés en SYNC/CWAI
static inline __attribute__((always_inline)) uint16_t StepCPU(void)
{
  // ligne HALT du contrôleur de disquette (DSKREG bit 7 sans DRQ). Le 6809 reconnaît HALT avec une
  // instruction de retard : c'est ce qui laisse Disk BASIC ranger le dernier octet (STB ,X+) avant
  // l'arrêt.
  static bool s_halt_prev = false;
  if (g_dw_boot_request) Dw_BootNow();
  const bool halt_now = Fdc_HaltLine();
  uint16_t c;
  if (halt_now && s_halt_prev)
  {
    c = 4;   // CPU arrêté : le temps avance, rien n'est exécuté
  }
  else
  {
    c = cpu->execute();
    if (c == 0)
    {
      c = 8;   // CPU en SYNC/CWAI (execute rend 0) : le temps émulé avance quand même
      g_cpu_wait_cycles += 8;   // cycles d'attente (SYNC/CWAI) = CPU 6809 disponible
    }
  }
  s_halt_prev = halt_now;
  Fdc_Tick(c);
  if (g_gime_div_q8 == 256)
  {
    const bool fast = (sf.CPU_Speed == CPU_FAST);   // lu une seule fois
    gime_tick_inline(c, fast);
    g_last_gime_cycles = c;
    g_emu_ticks += (uint64_t)c << (fast ? 3 : 4);   // temps émulé (14,318 MHz)
    g_bb_ticks  += (uint64_t)c << (fast ? 3 : 4);
  }
  else
  {
    // le temps GIME avance de c / facteur.
    static uint32_t s_acc = 0;
    s_acc += (uint32_t)c << 8;
    const uint32_t gc = s_acc / g_gime_div_q8;
    s_acc -= gc * g_gime_div_q8;
    if (gc) gime_tick(gc, sf.CPU_Speed == CPU_FAST);
    g_last_gime_cycles = (uint16_t)gc;
    g_emu_ticks += (uint64_t)gc << ((sf.CPU_Speed == CPU_FAST) ? 3 : 4);
    g_bb_ticks  += (uint64_t)c << ((sf.CPU_Speed == CPU_FAST) ? 3 : 4);   // boucles comptées du CoCo = cycles CPU
  }
  cpu_irq_line  = sf.irq_pin  && !gime_irq_active();
  cpu_firq_line = sf.firq_pin && !gime_firq_active();
  return c;
}

// applique les cycles accumulés du lot (sans franchir d'échéance pendant le lot : appelé avant une
// E/S)
static inline void Fdc_AddIndex(uint32_t c);
static inline bool Fdc_IsBusy(void);
static IRAM_ATTR void Batch_Sync(void)
{
  const uint32_t a = g_batch_acc;
  g_batch_acc = 0;
  const bool fast = (sf.CPU_Speed == CPU_FAST);
  // Avant une E/S, le contrôleur de disquette est au repos (sinon pas de lot) : seule l'impulsion
  // d'index avance. En fin de lot, la dernière instruction a pu lancer une commande
  if (Fdc_IsBusy()) Fdc_Tick((uint16_t)a); else Fdc_AddIndex(a);
  gime_tick(a, fast);
  g_emu_ticks += (uint64_t)a << (fast ? 3 : 4);
  g_bb_ticks  += (uint64_t)a << (fast ? 3 : 4);
}

// un lot d'instructions, jusqu'à la prochaine échéance du GIME une E/S, un SYNC/CWAI, ou « limit »
// cycles. Rend les cycles CPU exécutés (= cycles GIME : jamais utilisé en Overclock CPU)
static inline __attribute__((always_inline)) uint32_t Cpu_Batch(uint32_t limit, uint32_t *n)
{
  if (!g_batch_enabled || g_gime_div_q8 != 256 || Fdc_IsBusy() || g_dw_boot_request)
  {
    *n = 1;
    return StepCPU();
  }
  const bool fast = (sf.CPU_Speed == CPU_FAST);
  const uint32_t sh = fast ? 3u : 4u, rnd = (1u << sh) - 1u;
  // Échéances : fin de ligne, puis timer 279 ns (TINS=1). Le lot s'arrête après l'instruction qui
  // l'atteint, exactement comme StepCPU.
  uint32_t budget = (GIME_TICKS_PER_LINE - gime_raster.line_ticks + rnd) >> sh;
  if ((gime.reg[GIME_INIT1] & 0x20) && gime_raster.timer_count)
  {
    const uint32_t need = (gime_raster.timer_count << 2) - gime_raster.timer_ticks;
    const uint32_t b2 = (need + rnd) >> sh;
    if (b2 < budget) budget = b2;
  }
  // timer TINS=0 : échéance possible au front de HS, au milieu de la période
  if (!(gime.reg[GIME_INIT1] & 0x20) && gime_raster.timer_count == 1u && !gime_hs_done && gime_raster.line_ticks < gime_hs_tick())
  {
    const uint32_t b3 = (gime_hs_tick() - gime_raster.line_ticks + rnd) >> sh;
    if (b3 < budget) budget = b3;
  }
  if (budget > limit) budget = limit;
  if (budget == 0) budget = 1;
  g_batch_io = false;
  uint32_t total = 0, k = 0;
  do
  {
    uint32_t c = cpu->execute();
    k++;
    if (c == 0)                       // SYNC/CWAI : comme StepCPU
    {
      c = 8;
      g_cpu_wait_cycles += 8;
      g_batch_acc += c; total += c;
      break;
    }
    g_batch_acc += c; total += c;
  } while (total < budget && !g_batch_io);
  Batch_Sync();
  g_last_gime_cycles = (uint16_t)total;
  cpu_irq_line  = sf.irq_pin  && !gime_irq_active();
  cpu_firq_line = sf.firq_pin && !gime_firq_active();
  *n = k;
  return total;
}

// PC / CC du CPU pour les compteurs de diagnostic du GIME
uint16_t gime_dbg_cpu_pc(void) { return cpu ? cpu->get_pc() : 0; }
uint8_t  gime_dbg_cpu_cc(void) { return cpu ? cpu->get_cc() : 0; }


// CPU en SYNC/CWAI : le coeur ne fait plus rien tant que CPU_in_WAIT_STATE est vrai
static inline bool WaitStateIdleTick(void)
{
  gime_tick(8, sf.CPU_Speed == CPU_FAST);
  cpu_irq_line  = sf.irq_pin  && !gime_irq_active();
  cpu_firq_line = sf.firq_pin && !gime_firq_active();
  return (!cpu_irq_line || !cpu_firq_line || !sf.nmi_pin ||
          gpio_get_level(VSYNC_PORT) == 0);
}

void DoCPU(void)
{

  if (CPU_in_WAIT_STATE)
  {
    if (sf.CPU_HALTED_BY_EMULATOR) // If Emulator is in Menus
    {
      return;
    }

    if (!WaitStateIdleTick())  // Manage CWAI/SYNC instruction
    {
      return;
    }
    else
    {
      CPU_in_WAIT_STATE = false;
    }
  }

  StepCPU();
  StepCPU();
  if (sf.CPU_Speed == CPU_FAST)
  {
    StepCPU();
    StepCPU();
  }

  return;
}

void CopyDiskToRamDisk(void)
{

  ReadCoCoFile((const char*)Disk_Drive.Name_Disk[0], 0);
  ReadCoCoFile((const char*)Disk_Drive.Name_Disk[1], 1);
  ReadCoCoFile((const char*)Disk_Drive.Name_Disk[2], 2);
  ReadCoCoFile((const char*)Disk_Drive.Name_Disk[3], 3);
  // disques virtuels DriveWire sauvegardés dans config.ccc
  for (uint8_t i = 0; i < 2; i++) if (g_cfg_dw_name[i][0]) DW_Mount(i, g_cfg_dw_name[i]);
  for (uint8_t i = 0; i < 2; i++) if (g_cfg_dw_name[2 + i][0]) DW_Mount(VHD_UNIT0 + i, g_cfg_dw_name[2 + i]);
}

// Les positions marquées "X" (non utilisées en mode 6-bit CoCo3, voir commentaire ci-dessous) doivent
// être à -1 ("pas de pin", déjà testé par `if (pins[i] >= 0)` dans VGA::init), PAS à 0 (0 = GPIO0…)
    const PinConfig pins(-1,-1,8,9,-1,  -1,-1,-1,6, 7, -1,  -1,-1,-1,4,5,  2,1);
// B B X G G X R R 6 bit mode for Coco 3 0 1 X 0 1 X 0 1

VGA* vga;
GfxWrapper<VGA> *gfx;

// Core ESP32-Arduino 3.x : Mode::MODE_320x240x60_4_3 etc. sont probablement des objets statiques
// définis dans un autre fichier (lib VGA)
Mode mode0;
Mode mode1;
Mode mode2;
Mode mode3;

uint8_t SCAN_Keyboard_Matrix[8][7];

uint8_t ReadCoCoButtons(void)
{
  uint8_t val1, val2;
  val1 = USB_DEV_CONTROL.JOY1_BUTT1;
  val2 = USB_DEV_CONTROL.JOY2_BUTT1<<1;

  return val1 | val2;

}

void InitSD_Card1(void)
{
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  if (!SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT, 12))
  {
    LogOut.println("Card Mount Failed");
    return;
  }

 file = SD_MMC.open("/GAMES01.DSK", FILE_READ);
  if (!file)
  {
    LogOut.println("Failed to open /GAMES01.DSK");
    return;
  }
  else
  {
    LogOut.println("File Correct");
  }
  while(1)
  {

  }
}
extern bool SD_MMC_MountOK;  // défini dans InitSD_Card plus bas dans ce fichier

extern void flashFromSD(const char* filename);
static void FlashNow_Check(void);
void CheckFirmwareUpdate(void)
{
  if (!SD_MMC_MountOK)
  {
    debugln("[CheckFirmwareUpdate] SD_MMC non montée : mise à jour ignorée (pas de blocage).");
    return;
  }
  debugln("Before flash");
  flashFromSD("/qprcx.rty");  // Random name for flash file (Created from the Flash menu)
  // FlashNow_Check est appelé à la fin d'InitPeripherals_and_Others, une fois la VGA en marche
  // (message à l'écran)
}

// ===========================================================================
// dossier /FLASHNOW (créé au démarrage s'il manque) Un firmware .FLH valide qu'on y dépose est flashé
// tout de suite au démarrage, avant la vidéo et l'USB (donc même sans clavier ni menu F12) puis…

// retour automatique à la version précédente si une nouvelle version plante ou est éteinte avant 15 s
// de marche (rollback d'ESP-IDF)
// ===========================================================================
static void Overlay_Request(const char *l1, const char *l2, uint32_t ms, const char *l3, const char *l4);   // (plus bas)
#ifdef ESP_PLATFORM
#include <Update.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_flash.h"
#include <Preferences.h>
extern "C" bool verifyRollbackLater() { return true; }   // core Arduino (fonction C faible)

static void FlashNow_Rename(const char *src, const char *suffix)
{
  char dst[160];
  snprintf(dst, sizeof dst, "%s%s", src, suffix);
  if (SD_MMC.exists(dst)) SD_MMC.remove(dst);       // ancien fichier du même nom
  SD_MMC.rename(src, dst);
}

// écran pendant FLASHNOW (VGA déjà démarrée, coeur vidéo pas encore lancé dessin direct sur les deux
// tampons). Pendant l'écriture en flash, la PSRAM de l'image est momentanément inaccessible
extern void DrawText(const char *text, uint8_t Xpos, uint8_t Ypos, uint8_t XpixOffset, uint8_t YpixOffset, uint8_t ForeColor, uint8_t BackColor);
// pendant l'écriture en flash, la PSRAM (même bus MSPI que la flash) est inaccessible au DMA de la VGA
static bool s_fn_internal = false;
static void FlashNow_VideoInternal(bool on)
{
  if (!vga || on == s_fn_internal) return;
  vga->stop();
  vga->usePsram = !on;
  vga->bufferCount = on ? 1 : 2;
  bool ok = vga->Reinit(pins, mode1, 8);
  if (!ok && on)                                           // pas assez de RAM interne : PSRAM comme avant
  {
    vga->usePsram = true; vga->bufferCount = 2;
    ok = vga->Reinit(pins, mode1, 8);
    on = false;
  }
  vga->start();
  s_fn_internal = on;
  LogOut.printf("[FLASHNOW] ecran en %s\n", on ? "RAM interne (stable pendant l'ecriture)" : "PSRAM");
  if (!on) SetVideoMode(VIDEO_MODE_320X240_4_3);           // mode normal complet
}

static void FlashNow_Screen(const char *name, const char *status, uint8_t status_color)
{
  if (!vga) return;
  for (int pass = 0; pass < 2; pass++)
  {
    vga->clear(0);
    DrawText("FIRMWARE UPDATE (FLASHNOW FOLDER)", 0, 2, 0, 0, 255, 0);
    char nb[54]; snprintf(nb, sizeof nb, "%s", name);   // 53 colonnes au plus
    DrawText(nb, 0, 4, 0, 0, 28, 0);
    DrawText("DO NOT POWER OFF!", 0, 7, 0, 0, 224, 0);
    DrawText("This takes about 30 seconds. The LED blinks", 0, 9, 0, 0, 255, 0);
    DrawText("while writing; the screen may flicker or go", 0, 10, 0, 0, 255, 0);
    DrawText("dark. The ESP32 restarts by itself when done.", 0, 11, 0, 0, 255, 0);
    if (status) DrawText(status, 0, 14, 0, 0, status_color, 0);
    vga->show();
  }
}

static void FlashNow_Check(void)
{
  if (!SD_MMC.exists("/FLASHNOW")) { SD_MMC.mkdir("/FLASHNOW"); return; }
  File dir = SD_MMC.open("/FLASHNOW");
  if (!dir || !dir.isDirectory()) return;
  char name[112] = "";
  for (File f = dir.openNextFile(); f; f = dir.openNextFile())
  {
    if (f.isDirectory()) continue;
    const char *n = f.name();
    const char *b = strrchr(n, '/'); b = b ? b + 1 : n;
    const size_t l = strlen(b);
    if (l < 5 || b[0] == '.') continue;                 // fichiers cachés (._xxx du Mac)
    if (strcasecmp(b + l - 4, ".FLH") != 0 && strcasecmp(b + l - 4, ".BIN") != 0) continue;   // FLH ou .BIN (.done / .bad / .failed ignorés)
    snprintf(name, sizeof name, "%s", b);
    break;
  }
  dir.close();
  if (!name[0]) return;
  char src[128];
  snprintf(src, sizeof src, "/FLASHNOW/%s", name);
  LogOut.printf("[FLASHNOW] firmware trouve : %s\n", src);
  // une seule lecture du fichier, par blocs de 4 Ko : somme de contrôle (comme ValidFirmwareFile :
  // octets après le '-') et écriture directe en flash de ce qui suit le '~' (comme copyFile)
  File f = SD_MMC.open(src, FILE_READ);
  const size_t fsize = f ? f.size() : 0;
  uint8_t *buf = (uint8_t *)malloc(4096);
  bool ok = f && buf && fsize > 1024;
  // En-tête : chiffres, '-', texte libre, '~'
  uint32_t want = 0, sum = 0;
  size_t hdr = 0;
  int n = ok ? f.read(buf, 4096) : 0;
  const bool is_bin = strcasecmp(name + strlen(name) - 4, ".BIN") == 0;   // image brute
  if (ok && is_bin)
    ok = fsize > 65536 && n > 0 && buf[0] == 0xE9;        // en-tête d'image ESP32, pas de somme
  else if (ok)
  {
    size_t i = 0;
    while (i < (size_t)n && buf[i] >= '0' && buf[i] <= '9' && i < 14) want = want * 10u + (uint32_t)(buf[i++] - '0');
    ok = i > 0 && i < (size_t)n && buf[i] == '-';
    size_t j = i + 1;
    while (ok && j < (size_t)n && buf[j] != '~') j++;
    ok = ok && j < (size_t)n;
    hdr = j + 1;                                           // premier octet du binaire
    for (size_t k = i; ok && k < (size_t)n; k++) sum += buf[k];   // le '-' compte aussi (comme ValidFirmwareFile)
    ok = ok && hdr < (size_t)n && buf[hdr] == 0xE9;        // en-tête d'image ESP32
  }
  if (!ok)
  {
    if (f) f.close();
    free(buf);
    FlashNow_Rename(src, ".bad");
    FlashNow_Screen(name, is_bin ? "NOT A VALID .BIN FILE: ignored." : "NOT A VALID .FLH FILE: ignored.", 224); delay(4000);
    LogOut.printf("[FLASHNOW] fichier invalide : renomme %s.bad (demarrage normal)\n", src);
    return;
  }
  const size_t img = fsize - hdr;
  FlashNow_VideoInternal(true);                            // image stable pendant l'écriture
  FlashNow_Screen(name, "Checking and writing...", 252);
  delay(1500);                                             // le temps de lire le message
  pinMode(LED_PORT_ANODE, OUTPUT); pinMode(LED_PORT_CATHODE, OUTPUT);
  digitalWrite(LED_PORT_CATHODE, LOW); digitalWrite(LED_PORT_ANODE, HIGH);
  if (!Update.begin(img))
  {
    f.close(); free(buf);
    digitalWrite(LED_PORT_ANODE, LOW);
    FlashNow_Rename(src, ".failed");
    FlashNow_Screen(name, "FIRMWARE TOO BIG for this board (USB needed).", 224); delay(4000);
    FlashNow_VideoInternal(false);
    LogOut.printf("[FLASHNOW] firmware de %u octets trop gros pour la partition : renomme %s.failed\n", (unsigned)img, src);
    return;
  }
  LogOut.printf("[FLASHNOW] ecriture de %u octets en flash (ecran noir ~20 s, DEL clignotante : NE PAS COUPER LE COURANT)\n", (unsigned)img);
  ok = Update.write(buf + hdr, (size_t)n - hdr) == (size_t)n - hdr;
  size_t done = (size_t)n;
  uint32_t blink = 0;
  int pct_shown = 0;
  while (ok && done < fsize)
  {
    n = f.read(buf, 4096);
    if (n <= 0) { ok = false; break; }
    for (int k = 0; k < n; k++) sum += buf[k];
    ok = Update.write(buf, (size_t)n) == (size_t)n;
    done += (size_t)n;
    if (++blink % 8 == 0) digitalWrite(LED_PORT_ANODE, (blink / 8) & 1 ? LOW : HIGH);   // ~32 Ko par clignotement
    const int pct = (int)((uint64_t)done * 100u / fsize);
    if (pct >= pct_shown + 10)
    {
      pct_shown = pct - pct % 10;
      LogOut.printf("[FLASHNOW] %d %%\n", pct_shown);
      char st[40]; snprintf(st, sizeof st, "Writing... %d %%   ", pct_shown);
      FlashNow_Screen(name, st, 252);
    }
  }
  f.close();
  free(buf);
  if (ok && !is_bin && sum != want)
  {
    Update.abort();
    digitalWrite(LED_PORT_ANODE, LOW);
    FlashNow_Rename(src, ".bad");
    FlashNow_Screen(name, "BAD FILE (checksum): not installed. Starting.", 224); delay(4000);
    FlashNow_VideoInternal(false);
    LogOut.printf("[FLASHNOW] somme de controle fausse (%lu / %lu) : renomme %s.bad, version actuelle conservee\n", (unsigned long)sum, (unsigned long)want, src);
    return;
  }
  if (!ok || !Update.end(true))
  {
    Update.abort();
    digitalWrite(LED_PORT_ANODE, LOW);
    FlashNow_Rename(src, ".failed");
    FlashNow_Screen(name, "WRITE FAILED: not installed. Starting.", 224); delay(4000);
    FlashNow_VideoInternal(false);
    LogOut.printf("[FLASHNOW] ECHEC de l'ecriture en flash : renomme %s.failed, version actuelle conservee\n", src);
    return;
  }
  FlashNow_Rename(src, ".done");                           // gardé comme référence
  LogOut.printf("[FLASHNOW] termine : %s.done ; redemarrage sur la nouvelle version\n", src);
  FlashNow_Screen(name, "DONE! Restarting with the new firmware...", 28);
  delay(1500);
  digitalWrite(LED_PORT_ANODE, LOW);
  delay(100);
  esp_restart();
}

// Rapport au démarrage (après le journal) : version refusée, état de la partition.
void Fw_BootReport(void)
{
  const esp_partition_t *run = esp_ota_get_running_partition();
  esp_ota_img_states_t st;
  const bool pending = run && esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY;
  const esp_partition_t *bad = esp_ota_get_last_invalid_partition();
  LogOut.printf("[FIRMWARE] partition %s%s\n", run ? run->label : "?", pending ? " (nouvelle version, validee apres 15 s de marche)" : "");
  { // table de partitions réelle de la carte (schéma choisi dans Arduino)
    uint32_t fsz = 0; esp_flash_get_size(NULL, &fsz);
    char t[200]; size_t k = (size_t)snprintf(t, sizeof t, "[FIRMWARE] flash %lu Ko ;", (unsigned long)(fsz / 1024));
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    for (; it && k < sizeof t - 32; it = esp_partition_next(it))
    {
      const esp_partition_t *p = esp_partition_get(it);
      k += (size_t)snprintf(t + k, sizeof t - k, " %s %lu Ko", p->label, (unsigned long)(p->size / 1024));
    }
    if (it) esp_partition_iterator_release(it);
    LogOut.println(t);
  }
  if (bad)
  {
    LogOut.printf("[FIRMWARE] la version de la partition %s n'a pas demarre correctement : retour a la version precedente\n", bad->label);
    Overlay_Request("NEW FIRMWARE FAILED", "Back to the previous version", 6000, nullptr, nullptr);
  }
  // Version de chaque partition (pour le menu F, touche P)
  if (run)
  {
    Preferences p;
    if (p.begin("cc3f", false))
    {
      if (p.getString(run->label, "") != FW_VERSION_SHORT) p.putString(run->label, FW_VERSION_SHORT);
      p.end();
    }
  }
}

// loop : valide la version en cours après 15 s de marche (ou menu F12 ouvert).
void Fw_ValidateIfDue(bool now)
{
  static bool done = false;
  if (done || (!now && millis() < 15000)) return;
  done = true;
  const esp_partition_t *run = esp_ota_get_running_partition();
  esp_ota_img_states_t st;
  if (run && esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY)
  {
    esp_ota_mark_app_valid_cancel_rollback();
    LogOut.println("[FIRMWARE] nouvelle version validee (plus de retour automatique)");
  }
}

// Menu F, touche P : version de l'autre partition ("" si aucune version valide).
bool Fw_OtherVersion(char *out, size_t n)
{
  out[0] = 0;
  const esp_partition_t *other = esp_ota_get_next_update_partition(nullptr);
  if (!other) return false;
  esp_app_desc_t d;
  if (esp_ota_get_partition_description(other, &d) != ESP_OK) return false;   // vide ou effacée
  esp_ota_img_states_t st;
  if (esp_ota_get_state_partition(other, &st) == ESP_OK && (st == ESP_OTA_IMG_INVALID || st == ESP_OTA_IMG_ABORTED)) return false;
  Preferences p;
  String v;
  if (p.begin("cc3f", true)) { v = p.getString(other->label, ""); p.end(); }
  snprintf(out, n, "%s", v.length() ? (String(strchr(v.c_str(), '.') ? "v" : "v0.") + v).c_str() : "(unknown version)");
  return true;
}
bool Fw_BootOther(void)
{
  const esp_partition_t *other = esp_ota_get_next_update_partition(nullptr);
  if (!other || esp_ota_set_boot_partition(other) != ESP_OK) return false;
  LogOut.printf("[FIRMWARE] redemarrage sur la partition %s (version precedente)\n", other->label);
  extern void Sys_Restart(void);
  Sys_Restart();                                           // arrêt propre
  return true;
}
#else
static void FlashNow_Check(void) {}
void Fw_BootReport(void) {}
void Fw_ValidateIfDue(bool) {}
bool Fw_OtherVersion(char *out, size_t n) { if (n) out[0] = 0; return false; }
bool Fw_BootOther(void) { return false; }
#endif

bool SD_MMC_MountOK = false;  // vrai seulement si SD_MMC.begin a réussi ET qu'une carte est détectée

void InitSD_Card(void)
{

  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);

  if (!SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT, 12)) {
    debugln("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    debugln("No SD_MMC card attached");
    return;
  }

  SD_MMC_MountOK = true;

  BOOTLOG("SD_MMC Card Type: ");
  if(cardType == CARD_MMC){
    debugln("MMC");
  } else if(cardType == CARD_SD){
    debugln("SDSC");
  } else if(cardType == CARD_SDHC){
    debugln("SDHC");
  } else {
    debugln("UNKNOWN");
  }

}

#define BOARD_CASSETTE_OUT 13
#define BOARD_CASSETTE_IN 14
#define BOARD_CASSETTE_RELAY 10

#define ROM_OFFSET 0x8000
#define RAM_MODE 0x0000
void InitPeripherals_and_Others(void)
{

  // RAM physique 512 Ko déjà remplie à $FF et GIME initialisé dans AllocatePSRAMBuffers ; on remet le
  // GIME à l'état d'allumage.
  memset(memory, 0xFF, GIME_RAM_ALLOC_FOR(g_ram_capacity));
  gime_reset();

  Serial.begin(921600);
  delay(10);

  InitSD_Card();

  CheckFirmwareUpdate();

  InitDisks();

  cpu->assign_nmi_line(&sf.nmi_pin);
  cpu->assign_firq_line(&cpu_firq_line);   // PIA + GIME (voir StepCPU)
  cpu->assign_irq_line(&cpu_irq_line);

  //------ Special registers
  sf.CPU_HALTED_BY_EMULATOR = false;
  sf.DIRECT_Key_Code = 0;  // Nothing
  sf.PHYSICAL_Drive_Must_Be_Saved = false;
  sf.V_Synch_Int_Enabled = false;
  sf.nmi_pin = true;  // True = disabled
  sf.firq_pin = true; // True = disabled
  sf.irq_pin = true; // True = disabled
  sf.V_Synch = false;
  sf.DiskROM_Present = true;       // à rendre optionnel via menu plus tard
  sf.ROM_Offset = ROM_OFFSET;
  sf.AnyKeypress = false; // Used for Keyboard Scan.

  sf.CPU_Speed = CPU_SLOW;
  sf.Coco2VideoGenMODE = 255;    // To reset in first execution
  sf.Coco2VideoPageOffset_Registers = 0b00000010; // Init to be at address 400 (Even if Basic set it at boot)
  sf.Coco2ColorMode = 0;
  sf.VideoEmulatorXpixels = 320;
  sf.Artefact = true; // Artefact mode by default.
  sf.CycleAccurateMode = true;  // Overclock OFF par défaut = vitesse CoCo3 réelle
                                // (0,89 MHz, 1,79 MHz après POKE 65497) ; ON = ~2,7 MHz Valeurs par
                                // défaut ci-dessus SEULEMENT sans sauvegarde : config.ccc (lu plus
                                // haut par InitDisks) a priorité
  if (g_cfg_video_loaded)          // choix sauvegardé (menu F12 M/A/O)
  {
    sf.Artefact = g_cfg_artefact;
    g_artifact_swap = g_cfg_artifact_swap ? 1 : 0;
    gime_video_set_monitor_rgb(g_cfg_monitor_rgb);
    SetOverclockMode(g_cfg_overclock);
  }
  // taille de RAM (512 Ko par défaut ; limitée à ce qui a pu être alloué).
  g_ram_selected = g_cfg_ram ? g_cfg_ram : GIME_RAM_SIZE;
  if (g_ram_selected > g_ram_capacity) g_ram_selected = g_ram_capacity;
  gime_set_ram_size(g_ram_selected);
  BOOTLOG("[RAM] %u Ko utilises (capacite allouee %u Ko)\n",
                (unsigned)(g_ram_selected / 1024), (unsigned)(g_ram_capacity / 1024));
  sf.Coco2GraphicMode = 0;

  sf.is_JOY1_B1_WasPressed = false;
  sf.is_JOY1_B2_WasPressed = false;
  sf.is_JOY2_B1_WasPressed = false;
  sf.is_JOY2_B2_WasPressed = false;
  sf.is_LastKeyboardScanned = false;
//------------ Init of Disk Registers

  DiskAccess.IsinReadProcess = false;
  DiskAccess.IsInWriteProcess = false;
  DiskAccess.IsInAddressProcess = false;
  DiskAccess.LastCommandWasTypeI = true;  // état initial neutre (pas de transfert en cours)
  DiskAccess.IsInTrackFormat = false;
  DiskAccess.LastStepDirectionWasIn = true;  // convention neutre par defaut (avance)
  DiskAccess.NMI_Int_Started = false;
  DiskAccess.NMI_Delay = 0;
  Fdc_Reset();

  InitPorts();

  CopyCoCo3ROMS(); // ROM interne coco3[] -> rom[], disk21[] -> cart_rom
  gime_video_init(font_gimeChip);   // police texte GIME 128 car. x 12 lignes
  gime_field_sync_cb = PIA0_FieldSync;   // FS/HS du balayage émulé
  gime_field_sync_rise_cb = PIA0_FieldSyncRise;
  gime_hsync_cb      = PIA0_HSync;
  gime_reset();    // carte mémoire d'allumage : ROM 16K int + 16K cartouche
  gime_set_cart_clock(g_cart_clock);   // CART = Q (EI0 du GIME)
	CopyDiskToRamDisk();

  vga = new VGA();
  gfx = new GfxWrapper<VGA>(*vga, mode1.hRes, mode1.vRes);
  vga->bufferCount = 2;
  bool vgaOk = vga->init(pins, mode1, 8);
  if (!vgaOk)
  {
    while(1) delay(1);
  }
  // vga->start; DEBUG TEST ************************

  SetVideoMode(VIDEO_MODE_320X240_4_3);

  if (SD_MMC_MountOK) FlashNow_Check();   // /FLASHNOW, avec message à l'écran (VGA prête, avant l'USB et…)
}

void InitPorts(void)
{
  // Core ESP32-Arduino 3.x : ledcAttach remplace ledcSetup+ledcAttachPin (les canaux sont assignés
  // automatiquement) et ledcWrite prend maintenant le numéro de pin, plus le canal
  if (Audio_Init(47))
  {
    BOOTLOGLN("[AUDIO] sortie I2S PDM 48 kHz, horodatee (retard 20 ms)");
  }
  else
  {
    LogOut.println("[AUDIO] I2S indisponible : PWM LEDC direct");
  }

  // int initval = analogRead(BOARD_CASSETTE_IN) adc1_config_channel_atten(ADC1_CHANNEL_6,
  // ADC_ATTEN_DB_0); // 0-1.1V

  // digitalWrite(BOARD_CASSETTE_RELAY, LOW); //Disable Relay.

  return;
}
// Mode VGA 640 pour les modes GIME 64/80 colonnes et 512/640 pixels. Balayage IDENTIQUE à mode1
// (MODE_320x240x60_4_3)
#define VIDEO_MODE_GIME_640 4
static const Mode MODE_GIME_640x240
(
    17,
    80,        // h_sync (2 x 40)
    48,        // h_back_porch (2 x 24)
    652,       // h_active (2 x 326)
    16,        // v_front_porch (= mode1 : 527 lignes, 60,07 Hz, voir MODE_COCO_320x240)
    2,         // v_sync
    29,        // v_back_porch
    240,       // v_active
    25263158,  // 2 x 240 MHz / 19
    0, 0, 2
);

// mode 320 4:3 de l'émulateur. Identique à Mode::MODE_320x240x60_4_3 (Mode.cpp) sauf le palier avant
// vertical : 16 lignes au lieu de 10 (phase ne change pas (le palier avant suit la zone active))
static const Mode MODE_COCO_320x240
(
    8, 40, 24, 320 + 6,
    16, 2, 29, 240,
    12587500, 0, 0, 2
);

// Efface les tampons vidéo juste après un changement de mode : le nouveau tampon DMA est alloué sans
// être initialisé (d'où le « garbage » visible avant le premier rendu).
static void ClearVGABuffers(void)
{
  for (int b = 0; b < vga->bufferCount; b++)
  {
    for (int y = 0; y < vga->mode.vRes; y++)
    {
      memset(vga->dmaBuffer->getLineAddr8(y, b), 0, vga->mode.hRes);
    }
    vga->dmaBuffer->flush(b);   // cache PSRAM -> mémoire lue par le DMA
  }
}

static uint8_t s_currentVideoMode = 0xFF;

// Rend la main au DÉBUT (front descendant) d'une impulsion VSYNC (polarité négative, lue sur
// VSYNC_PORT). Attente max. ~1 trame (16,4 ms).
static void WaitVSyncFallingEdge(void)
{
  while (gpio_get_level(VSYNC_PORT) == 0) { }   // finir une impulsion en cours
  while (gpio_get_level(VSYNC_PORT) != 0) { }   // attendre la suivante
}

void SetVideoMode(uint8_t VideoMode)
{
  // Bascule rapide 320 4:3 <-> 640 GIME
  const bool pair_now    = (s_currentVideoMode == VIDEO_MODE_320X240_4_3 || s_currentVideoMode == VIDEO_MODE_GIME_640);
  const bool pair_target = (VideoMode == VIDEO_MODE_320X240_4_3 || VideoMode == VIDEO_MODE_GIME_640);
  if (pair_now && pair_target)
  {
    if (VideoMode == s_currentVideoMode)
    {
      return;
    }
    const Mode &target = (VideoMode == VIDEO_MODE_GIME_640) ? MODE_GIME_640x240 : mode1;
    const uint64_t t0 = esp_timer_get_time();
    if (vga->hasAlternate(target) && vga->switchToAlternate(WaitVSyncFallingEdge))
    {
      sf.VideoEmulatorXpixels = (VideoMode == VIDEO_MODE_GIME_640) ? 652 : 380;
      s_currentVideoMode = VideoMode;
      BOOTLOG("[VGA] bascule rapide -> %s en %u us\n",
                    (VideoMode == VIDEO_MODE_GIME_640) ? "640" : "320",
                    (unsigned)(esp_timer_get_time() - t0));
      return;
    }
    BOOTLOG("[VGA] bascule rapide impossible (jumeau pret=%d) : Reinit complet\n",
                  vga->hasAlternate(target));
    // sinon : chemin complet ci-dessous
  }
  const uint64_t t_full = esp_timer_get_time();

  vga->stop();
  switch (VideoMode)
  {
  case VIDEO_MODE_320X240_16_9:
    sf.VideoEmulatorXpixels = 320;
    if(!vga->Reinit(pins, mode0, 8)) while(1) delay(1);
    break;
    case VIDEO_MODE_320X240_4_3:
      sf.VideoEmulatorXpixels = 380;
      if(!vga->Reinit(pins, mode1, 8)) while(1) delay(1);
    break;
    case VIDEO_MODE_640X240_16_9:
      sf.VideoEmulatorXpixels = 640;
      if(!vga->Reinit(pins, mode2, 8)) while(1) delay(1);
    break;
    case VIDEO_MODE_640X240_4_3:
      sf.VideoEmulatorXpixels = 640;
      if(!vga->Reinit(pins, mode3, 8)) while(1) delay(1);
    break;
    case VIDEO_MODE_GIME_640:
      sf.VideoEmulatorXpixels = 652;
      if(!vga->Reinit(pins, MODE_GIME_640x240, 8)) while(1) delay(1);
    break;

  default:
    break;
  }
  ClearVGABuffers();
  vga->start();
  s_currentVideoMode = VideoMode;
  // Prépare le mode jumeau pour les bascules rapides suivantes (Reinit vient de libérer l'ancien
  // tampon alterné).
  bool altOk = false;
  if (VideoMode == VIDEO_MODE_320X240_4_3)
  {
    altOk = vga->prepareAlternate(MODE_GIME_640x240);
  }
  else if (VideoMode == VIDEO_MODE_GIME_640)
  {
    altOk = vga->prepareAlternate(mode1);
  }
  BOOTLOG("[VGA] Reinit complet mode %u en %u us, jumeau prepare=%d\n",
                VideoMode, (unsigned)(esp_timer_get_time() - t_full), altOk);

}

const int Field_Synch_Interrupt_PIN = VSYNC_PORT;  // VSynch pin

// ---------------------------------------------------------------------------
// PIA0 ($FF00-$FF03) piloté par le balayage émulé du GIME. FS (VSYNC du VDG/GIME) -> CB1 : drapeau
// $FF03 bit 7, IRQ si $FF03 bit 0. HS (HSYNC) -> CA1 : drapeau $FF01 bit 7, IRQ si $FF01 bit 0
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// image de la VRAM CoCo 2 prise au fil du balayage émulé. Les jeux CoCo 1/2 redessinent l'écran juste
// après l'interruption de trame (FS)
// ---------------------------------------------------------------------------
static uint32_t          s_vdg_snap_base[3], s_vdg_snap_off[3];
static volatile uint8_t  s_vdg_ready   = 0xFF;   // dernier tampon complet
static volatile uint8_t  s_vdg_reading = 0xFF;   // tampon lu par le rendu

// Copie progressive, au rythme du balayage émulé (comme le VDG qui lit la VRAM ligne par ligne)
static uint8_t  s_vdg_w   = 0xFF;       // tampon en cours de remplissage
static uint32_t s_vdg_src = 0;          // adresse physique de départ (trame)

static IRAM_ATTR void VDG_CopyLine(uint16_t line) // coeur CPU, fin de ligne émulée
{
  if (s_vdg_snap[0] == nullptr) return;
  const int k = (int)line - (int)(GIME_LINES_PER_FRAME - 192);
  if (k < 0 || k >= 192) return;
  if (k == 0)                            // début de la zone active
  {
    if (gime_video_active()) { s_vdg_w = 0xFF; return; }
    uint8_t w = 0;
    while (w == s_vdg_ready || w == s_vdg_reading) w++;
    s_vdg_w = w;
    const uint32_t base = gime_legacy_video_base();
    const uint32_t off  = (uint32_t)sf.Coco2VideoPageOffset_Registers * COCO2_GRAPH_OFFSET_LEN;
    s_vdg_snap_base[w] = base;
    s_vdg_snap_off[w]  = off;
    s_vdg_src = base + off;              // garde de 80 Ko après la RAM
  }
  if (s_vdg_w > 2) return;
  memcpy(s_vdg_snap[s_vdg_w] + k * 32, memory + s_vdg_src + k * 32, 32);
  if (k == 191)                          // dernière ligne : image complète
  {
    __sync_synchronize();
    s_vdg_ready = s_vdg_w;
    s_vdg_w = 0xFF;
  }
}

// Pointeur « vram » des rendus CoCo 2
static uint8_t *LegacyVRAM(uint32_t memloop_off)
{
  const uint8_t r = s_vdg_ready;
  if (g_pia_sync_emulated && r < 3 &&
      s_vdg_snap_base[r] == gime_legacy_video_base() && s_vdg_snap_off[r] == memloop_off)
  {
    s_vdg_reading = r;
    return s_vdg_snap[r] - memloop_off;   // vram[MemLoop] = s_vdg_snap[r][0...]
  }
  s_vdg_reading = 0xFF;
  return memory + gime_legacy_video_base();
}

static inline void PIA0_UpdateIRQ(void)
{
  const bool fs = (rom[ROM_FF03] & 0x81) == 0x81;
  const bool hs = (rom[ROM_FF01] & 0x81) == 0x81;
  sf.irq_pin = !(fs || hs);
}

// Entrée clavier EI1 du GIME : active quand au moins une rangée PA0-PA6 du PIA0 est tirée à 0 pour les
// colonnes actuellement sélectionnées par $FF02 (touche enfoncée, ou bouton de manette sur PA0/PA1)
static void GIME_PollKeyboardLine(void)
{
  const uint8_t cols = (uint8_t)~rom[ROM_FF02];   // colonnes actives (à 0)
  bool any = (ReadCoCoButtons() != 3);             // boutons sur PA0/PA1
  for (uint8_t c = 0; c < 8 && !any; c++)
  {
    if (cols & (1u << c))
    {
      for (uint8_t r = 0; r < 7; r++)
      {
        if (SCAN_Keyboard_Matrix[c][r] != 0) { any = true; break; }
      }
    }
  }
  gime_set_ext_line(GIME_INT_EI1, any);
}

// FS au bon moment (fin de la bordure du bas) et front choisi par $FF03 bit 1 (PIA CB1 : 0 = front
// descendant, 1 = front montant).
static void PIA0_FieldSyncEdge(void)
{
  if (!g_pia_sync_emulated) return;
  rom[ROM_FF03] |= 0x80;
  sf.V_Synch = true;
  PIA0_UpdateIRQ();
}
static void PIA0_FieldSync(void)        // rappel GIME : FS descend
{
  GIME_PollKeyboardLine();              // EI1, indépendant de la source FS
  if (!(rom[ROM_FF03] & 0x02)) PIA0_FieldSyncEdge();
}
static void PIA0_FieldSyncRise(void)    // rappel GIME : FS remonte (6 lignes plus tard)
{
  if (rom[ROM_FF03] & 0x02) PIA0_FieldSyncEdge();
}

// ---------------------------------------------------------------------------
// Sortie son (PWM 8 bits, broche 47), comme le CoCo CNA 6 bits = $FF20 bits 7-2 (bits 1-0)
// ---------------------------------------------------------------------------
#ifndef ESP_PLATFORM
void (*g_snd_hook)(uint8_t) = nullptr;
void (*g_psg_hook)(uint8_t) = nullptr;   // banc, écritures du SN76489A
void (*g_ay_hook)(uint8_t, uint8_t) = nullptr;   // banc, écritures du YM2149
#endif
static uint8_t g_o90_l = 128, g_o90_r = 128;   // CNA de l'Orchestra-90
static void UpdateSoundOutput(void)
{
  static int32_t s_last = -1;   // clé gauche << 8 | droite
  static uint16_t s_dac = 0x80;   // dernier niveau CNA audible
  const bool snden  = (rom[ROM_FF23] & 0x08) != 0;
  const bool mux_dac = ((rom[ROM_FF01] | rom[ROM_FF03]) & 0x08) == 0;
  // son coupé (SNDEN = 0) ou sélecteur ailleurs que sur le CNA la source est simplement débranchée, la
  // sortie (couplée en alternatif) ne saute pas
  if (snden && mux_dac)
  {
    s_dac = Pia1A_Out() & 0xFC;   // sortie réelle (DDR)
  }
  else if (snden && (rom[ROM_FF01] & 0x08) && !(rom[ROM_FF03] & 0x08) && Cas_MotorOn())
  {
    s_dac = Cas_Level((uint32_t)g_emu_ticks) ? 0xC0 : 0x40;   // sélecteur sur la cassette (AUDIO ON) : la bande s'entend
  }
  uint16_t out = s_dac;
  if (rom[ROM_FF22] & 0x02)
  {
    out += 0x80;
  }
  if (out > 255) out = 255;
  // Orchestra-90 : ses 2 CNA 8 bits (gauche $FF7A, droite $FF7B) ont leurs propres sorties sur la
  // vraie carte
  uint16_t outL = out, outR = out;
  if (g_o90_present) { outL = (uint16_t)((out + g_o90_l) >> 1); outR = (uint16_t)((out + g_o90_r) >> 1); }
  const int32_t key = (int32_t)((outL << 8) | outR);
  if (key != s_last)
  {
    s_last = key;
    const uint8_t mono = (uint8_t)((outL + outR) >> 1);
#ifndef ESP_PLATFORM
    if (g_snd_hook) g_snd_hook(mono);           // banc : journal horodaté du son
#endif
    if (Audio_UsingI2S())                       // rejoué à cadence fixe
      Audio_PushLR(g_audio_realtime ? (uint32_t)((uint64_t)esp_timer_get_time() * 14318180ull / 1000000ull) : (uint32_t)g_emu_ticks,
                   (uint8_t)outL, (uint8_t)outR);
    else                  Audio_PwmWrite(mono);   // PWM (ou rien si broche forcée)
  }
}

// écriture d'un CNA de l'Orchestra-90 (le logiciel fait STD $FF7A gauche puis droite).
void O90_Write(uint16_t a, uint8_t v)
{
  if (a == 0xFF7A) g_o90_l = v; else g_o90_r = v;
  UpdateSoundOutput();
}

static inline void Perf_AddLine(uint32_t cc);
static IRAM_ATTR void PIA0_HSync(void)            // rappel GIME, chaque fin de ligne
{
  if (Cas_MotorOn() && (rom[ROM_FF01] & 0x08) && !(rom[ROM_FF03] & 0x08) && (rom[ROM_FF23] & 0x08)) UpdateSoundOutput();   // AUDIO ON
  if (!g_pia_sync_emulated) return;
  const uint32_t cc0 = Perf_CCount();   // coût des copies de ligne
  VDG_CopyLine(gime_raster.line);       // VRAM CoCo 2 au fil du balayage
  gime_video_capture_line(memory, gime_raster.line);   // VRAM modes CoCo3
  Perf_AddLine(Perf_CCount() - cc0);
  rom[ROM_FF01] |= 0x80;
  if (rom[ROM_FF01] & 0x01)
  {
    PIA0_UpdateIRQ();
  }
}

void IRAM_ATTR Field_Synch_Interrupt_Flag()
{

  if (!g_pia_sync_emulated)     // sinon FS vient du GIME émulé
  {
    rom[ROM_FF03] |= 0b10000000;  // Set bit 7

    if (sf.V_Synch_Int_Enabled)
    {
      sf.irq_pin = false;  // Interrupt enabled.
    }
    sf.V_Synch = true;
  }

    g_vsync_frame_count++; // V-SYNC COUNT
    // période mesurée de la trame VGA (moyenne glissante), pour caler le cycle-accurate sur une trame
    // émulée par trame VGA.
    {
      static int64_t s_last_us = 0;
      const int64_t now = esp_timer_get_time();
      const int64_t d = now - s_last_us;
      s_last_us = now;
      if (d > 14000 && d < 20000)
      {
        // moyenne glissante en 1/16 µs, arrondie. L'ancienne division entière tronquait vers le bas
        // (jusqu'à -15 µs) : émulation un peu trop rapide, la trame émulée glissait contre la trame
        // VGA.
        static uint32_t s_acc16 = 16667u * 16u;
        s_acc16 = s_acc16 - (s_acc16 >> 4) + (uint32_t)d;
        g_vga_frame_us = (s_acc16 + 8u) >> 4;
      }
      g_vsync_us = now;   // instant du dernier VSYNC (verrouillage de phase)
    }

}

// FF22: Resolution Selection F8 = Pmode 4 256x192 2 colors 11111XXX E8 = Pmode 3 128x192 4 colors
// 11101XXX D8 = pmode 2 128x192 2 colors 11011XXX C8 = pmode 1 128x96 4 colors 11001XXX B8 = pmode 0…

bool IsButtonPressed(void)
{
  if (digitalRead(21) == true)
  {
    return false;
  }
  else
  {
    return true;
  }
}
uint8_t debugloop = 0;

hw_timer_t* cpuTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

uint32_t testloop1 = 0;

void IRAM_ATTR onCPUTimer()
{

  Debug1Toggle();
  StepCPU();
  StepCPU();
  testloop1++;
  if (testloop1 == 20000)
  {
    testloop1 = 0;
  }

}

void setTimerInterval(uint32_t newInterval) // To set Slow/Fast CPU speed
{
  // Core ESP32-Arduino 3.x (ESP-IDF 5.x)
  timerAlarm(cpuTimer, newInterval, true, 0);  // newInterval est le nombre de ticks
}

// ===========================================================================
// souris USB branchée sur un port joystick du CoCo. Position absolue 0-1023 par axe, intégrée à partir
// des déplacements relatifs de la souris USB ; boutons gauche/droit = boutons 1/2 du port
// ===========================================================================
volatile int16_t g_mouse_x = 512, g_mouse_y = 512;   // 0-1023
volatile uint8_t g_mouse_buttons = 0;                // bit 0 gauche, bit 1 droit
static bool     s_hr_low = false;                    // CNA à 0 (charge en cours)
static uint64_t s_hr_start = 0;                      // début de la charge (tics)

static inline bool Mouse_Hires(void)    { return g_mouse_mode == 2 || g_mouse_mode == 4 || g_mouse_mode == 6; }
static inline uint64_t Mouse_ChargeTicks(uint8_t axis)
{
  const int32_t v = axis ? g_mouse_y : g_mouse_x;
  const uint32_t us = (uint32_t)v * 4856u / 1023u + 560u;
  return (uint64_t)us * 14318u / 1000u;
}
static inline uint8_t Mouse_CurrentAxis(void)
{
  return (uint8_t)(((rom[ROM_FF01] & 0x08) >> 3));   // bit 0 du mux = axe X/Y
}
static void Mouse_Retrigger(void)
{
  // Même règle que MAME : CNA toujours à 0 et charge précédente terminée > nouvelle charge (lecture de
  // l'autre axe par le logiciel).
  if (s_hr_low && g_emu_ticks >= s_hr_start + Mouse_ChargeTicks(Mouse_CurrentAxis()))
    s_hr_start = g_emu_ticks;
}
static void Mouse_DacWrite(uint8_t ff20)
{
  if (!Mouse_Hires()) return;
  const bool low = (ff20 & 0xFC) == 0;
  if (low && !s_hr_low) s_hr_start = g_emu_ticks;
  s_hr_low = low;
  if (low) Mouse_Retrigger();
}
static void Mouse_MuxChanged(void)
{
  if (Mouse_Hires()) Mouse_Retrigger();
}
// Rend true si la souris haute résolution fournit le comparateur pour ce mux.
static bool Mouse_Comparator(uint8_t mux, bool *result)
{
  if (!Mouse_Hires() || !((Mouse_Mask() >> ((mux >> 1) & 1)) & 1)) return false;   // un port ou les deux
  if (!s_hr_low) { *result = false; return true; }                 // pas de charge : 0
  *result = g_emu_ticks >= s_hr_start + Mouse_ChargeTicks(mux & 1);
  return true;
}
// Appelé par la souris USB (coeur 0) : déplacement relatif + boutons.
volatile uint32_t g_input_events = 0;   // touches et souris (écran de veille)
void Mouse_UsbEvent(int16_t dx, int16_t dy, uint8_t buttons)
{
  if (dx || dy || buttons) g_input_events++;
  if (g_mouse_mode == 0) return;
  // sensibilité 1-8 -> dx x sens / 2 (4 = x2), reste gardé pour les petits mouvements
  static int32_t s_fx = 0, s_fy = 0;
  s_fx += (int32_t)dx * g_mouse_sens; s_fy += (int32_t)dy * g_mouse_sens;
  const int32_t mx = s_fx / 2, my = s_fy / 2;
  s_fx -= mx * 2; s_fy -= my * 2;
  int32_t x = g_mouse_x + mx, y = g_mouse_y + my;
  if (x < 0) x = 0; if (x > 1023) x = 1023;
  if (y < 0) y = 0; if (y > 1023) y = 1023;
  g_mouse_x = (int16_t)x; g_mouse_y = (int16_t)y;
  g_mouse_buttons = buttons & 3;
  const uint8_t ax = (uint8_t)(x >> 2), ay = (uint8_t)(y >> 2);    // basse résolution 0-255
  const uint8_t b1 = (buttons & 1) ? 0 : 1, b2 = (buttons & 2) ? 0 : 1;   // 0 = enfoncé
  if (Mouse_Mask() & 1)
  {
    USB_DEV_CONTROL.JOY1_X_AXIS = ax; USB_DEV_CONTROL.JOY1_Y_AXIS = ay;
    USB_DEV_CONTROL.JOY1_BUTT1 = b1;  USB_DEV_CONTROL.JOY1_BUTT2 = b2;
  }
  if (Mouse_Mask() & 2)
  {
    USB_DEV_CONTROL.JOY2_X_AXIS = ax; USB_DEV_CONTROL.JOY2_Y_AXIS = ay;
    USB_DEV_CONTROL.JOY2_BUTT1 = b1;  USB_DEV_CONTROL.JOY2_BUTT2 = b2;
  }
}

uint8_t ReadJoysticks(uint8_t JoyNum)
{

    switch (JoyNum)
  {
  case JOY_RX:
    return USB_DEV_CONTROL.JOY1_X_AXIS;
    break;
  case JOY_RY:
    return USB_DEV_CONTROL.JOY1_Y_AXIS;
    break;
  case JOY_LX:
    return USB_DEV_CONTROL.JOY2_X_AXIS;
    break;
  case JOY_LY:
    return USB_DEV_CONTROL.JOY2_Y_AXIS;
    break;

  default:
    break;
  }
  return 255;
}

// initial setup

// Fréquences nominales du 6809 sur CoCo2 (approximatif, à ajuster si une valeur plus précise est
// documentée pour le matériel visé)
constexpr uint32_t CPU_HZ_SLOW = 894886;   // ~0.895 MHz
constexpr uint32_t CPU_HZ_FAST = 1789772;  // ~1.79 MHz

// ===========================================================================
// touches spéciales pour les testeurs (sans port série) F9 : journal daté -> /LOGS/LOGnnnn.TXT (config
// + W + U + J + tout le journal depuis la mise sous tension) Impr. écran
// ===========================================================================
const char *volatile g_cmd_inject = nullptr;
volatile bool g_req_log = false, g_req_snap = false, g_req_reboot = false, g_req_secret = false;
// Superposition demandée au coeur vidéo : 2 lignes, durée en ms. jusqu'à 4 lignes de 30 caractères au
// plus
#define OVL_LINES 4
#define OVL_COLS  30
volatile bool g_overlay_req = false;
volatile bool g_menu_open = false;   // vrai seulement pendant EMULATOR_Menu
char g_overlay_l[OVL_LINES][OVL_COLS + 1];
uint32_t g_overlay_ms = 0;
static void Overlay_Set(int i, const char *t)
{
  strncpy(g_overlay_l[i], t ? t : "", OVL_COLS);
  g_overlay_l[i][OVL_COLS] = 0;
}
static void Overlay_Request(const char *l1, const char *l2, uint32_t ms, const char *l3, const char *l4);
// cassette : message à l'écran ; chargement rapide (Overclock MACHINE tant que le moteur tourne)
void Cas_Notify(const char *l1, const char *l2) { Overlay_Request(l1, l2, 3000, nullptr, nullptr); }
void Cas_SetTurbo(bool on)
{
  static uint8_t s_saved = OC_OFF; static bool s_act = false;
  if (on) { if (!s_act && g_oc_mode != OC_MACHINE) { s_saved = g_oc_mode; s_act = true; SetOverclockMode(OC_MACHINE); } }
  else if (s_act) { s_act = false; SetOverclockMode(s_saved); }
}
static void Overlay_Request(const char *l1, const char *l2, uint32_t ms, const char *l3 = nullptr, const char *l4 = nullptr)
{
  // menu F12 ouvert : la boucle vidéo (qui affiche la superposition) est occupée par le menu ->
  // message écrit directement au bas de l'écran du menu.
  if (g_menu_open) { extern void Menu_Toast(const char *, const char *); Menu_Toast(l1, l3 ? l3 : l2); return; }   // menu F12 ouvert : message au bas du menu
  Overlay_Set(0, l1); Overlay_Set(1, l2); Overlay_Set(2, l3); Overlay_Set(3, l4);
  g_overlay_ms = ms;
  g_overlay_req = true;
}


// Prochain nom libre « /DIR/PREFnnnn.EXT » (nnnn de 0001 à 9999).
#define DEBUG_DIR "/ESP-DEBUG"            // journaux, captures, aide des testeurs
static bool Sd_NextName(const char *dir, const char *pref, const char *ext, char *out, size_t outsz, uint16_t &last)
{
  if (!SD_MMC_MountOK) return false;
  if (!SD_MMC.exists(DEBUG_DIR)) SD_MMC.mkdir(DEBUG_DIR);
  if (!SD_MMC.exists(dir)) SD_MMC.mkdir(dir);
  for (uint16_t n = (uint16_t)(last + 1); n != 0 && n <= 9999; n++)
  {
    snprintf(out, outsz, "%s/%s%04u.%s", dir, pref, (unsigned)n, ext);
    if (!SD_MMC.exists(out)) { last = n; return true; }
  }
  return false;
}

static void Report_Config(void)
{
  extern uint32_t g_ram_selected; extern uint8_t g_mouse_mode, g_kbjoy_port, g_oc_mode;
  extern bool g_cpu6309_run, g_lowercase_t1; extern uint32_t g_fdc_rnf[4];
  static const char *k_oc[] = { "OFF (au cycle pres)", "CPU", "MACHINE" };
  static const char *k_mouse[] = { "aucune", "droite basse res.", "droite haute res.", "gauche basse res.", "gauche haute res." };
  LogOut.printf("[CONFIG] CPU %s ; RAM %lu Ko ; Overclock %s ; GIME %s ; moniteur %s ; souris %s ; manette au clavier %s ; minuscules (L) %s\n",
                g_cpu6309_run ? "HD6309" : "MC6809", (unsigned long)(g_ram_selected / 1024),
                k_oc[g_oc_mode <= 2 ? g_oc_mode : 0], gime_is_1987() ? "1987" : "1986",
                gime_video_monitor_rgb() ? "RGB" : "composite", (g_mouse_mode == 5 ? "deux ports, basse res." : g_mouse_mode == 6 ? "deux ports, haute res." : k_mouse[g_mouse_mode <= 4 ? g_mouse_mode : 0]),
                g_kbjoy_port == 1 ? "droite" : (g_kbjoy_port == 2 ? "gauche" : (g_kbjoy_port == 3 ? "les deux" : "non")), g_lowercase_t1 ? "oui" : "non");
  { extern void Slots_Report(const char *); Slots_Report("[CONFIG]"); }
  for (int i = 0; i < 4; i++)
    LogOut.printf("[CONFIG] lecteur %d : %s ; RECORD NOT FOUND : %lu\n", i,
                  Disk_Drive.Name_Disk[i][0] ? (const char *)Disk_Drive.Name_Disk[i] : "(vide)", (unsigned long)g_fdc_rnf[i]);
  for (int i = 0; i < DW_DRIVES + 2; i++)
    if (g_dw_name[i][0]) LogOut.printf(i < DW_DRIVES ? "[CONFIG] DriveWire /x%d : %s\n" : "[CONFIG] VHD %d : %s\n", i < DW_DRIVES ? i : i - DW_DRIVES, g_dw_name[i]);
}

// ---- Capture d'écran (BMP 8 bits, palette 256 couleurs)
// Octet VGA : R = bits 7-6, G = bits 4-3, B = bits 1-0 (comme la palette du GIME, gime_video.cpp). Les
// lignes 640 de large sont doublées (640 x 480) pour garder les proportions ; 320 x 240 sinon.
static uint8_t *Snap_Capture(int &w, int &h)
{
  if (!vga || !vga->dmaBuffer) return nullptr;
  w = vga->mode.hRes; h = vga->mode.vRes;
  const int front = (vga->backBuffer + vga->bufferCount - 1) % (vga->bufferCount > 0 ? vga->bufferCount : 1);
  uint8_t *img = (uint8_t *)heap_caps_malloc((size_t)w * h, MALLOC_CAP_SPIRAM);
  if (!img) return nullptr;
  for (int y = 0; y < h; y++) memcpy(img + (size_t)y * w, vga->dmaBuffer->getLineAddr8(y, front), w);
  return img;
}
static bool Snap_WriteBMP(const char *path, const uint8_t *img, int w, int h)
{
  const int rep = (w >= 640) ? 2 : 1;
  const int oh = h * rep;
  const uint32_t rowsz = (uint32_t)((w + 3) & ~3);
  const uint32_t imgsz = rowsz * oh, off = 14 + 40 + 1024;
  uint8_t hdr[54] = {0};
  auto put32 = [&](int o, uint32_t v) { hdr[o] = v; hdr[o + 1] = v >> 8; hdr[o + 2] = v >> 16; hdr[o + 3] = v >> 24; };
  hdr[0] = 'B'; hdr[1] = 'M'; put32(2, off + imgsz); put32(10, off);
  put32(14, 40); put32(18, (uint32_t)w); put32(22, (uint32_t)oh);
  hdr[26] = 1; hdr[28] = 8; put32(34, imgsz); put32(38, 2835); put32(42, 2835); put32(46, 256); put32(50, 0);
  File f = SD_MMC.open(path, FILE_WRITE);
  if (!f) return false;
  f.write(hdr, sizeof hdr);
  uint8_t pal[1024];
  for (int i = 0; i < 256; i++)
  {
    pal[i * 4 + 0] = (uint8_t)((i & 3) * 85);          // B
    pal[i * 4 + 1] = (uint8_t)(((i >> 3) & 3) * 85);   // G
    pal[i * 4 + 2] = (uint8_t)(((i >> 6) & 3) * 85);   // R
    pal[i * 4 + 3] = 0;
  }
  f.write(pal, sizeof pal);
  uint8_t pad[4] = {0};
  for (int y = h - 1; y >= 0; y--)                    // BMP : de bas en haut
    for (int r = 0; r < rep; r++)
    {
      f.write(img + (size_t)y * w, w);
      if (rowsz != (uint32_t)w) f.write(pad, rowsz - w);
    }
  f.close();
  return true;
}

// ---- Journal F9
char g_log_note[96] = "";
volatile bool g_log_note_ready = false;
// redémarrage propre. esp_restart arme un chien de garde de secours puis arrête l'autre coeur
extern bool State_WaitParkedPublic(void);
void Sys_Restart(void)
{
  g_rtc_step = STEP_REBOOT;
  CPU_in_WAIT_STATE = true;
  sf.CPU_HALTED_BY_EMULATOR = true;
  State_WaitParkedPublic();
  Audio_SetPaused(true);
  NetDw_Shutdown();                                  // DriveWire par le Wi-Fi : libère le serveur du PC
  if (vga) vga->stop();
  if (SD_MMC_MountOK) { SD_LOCK(); SD_MMC.end(); SD_MMC_MountOK = false; }
  Serial.flush();
  delay(50);
  esp_restart();
}
static bool s_crash_boot = false;         // ce démarrage suit un plantage

static void Log_WriteText(File &f, const uint8_t *buf, size_t n, size_t lineStart)
{
  for (size_t i = lineStart; i < n; i++)               // fins de ligne Windows (Bloc-notes)
    if (buf[i] == '\n')
    {
      f.write(buf + lineStart, i - lineStart);
      f.write((const uint8_t *)"\r\n", 2);
      lineStart = i + 1;
    }
  if (lineStart < n) f.write(buf + lineStart, n - lineStart);
}

// kind : 0 = F9, 1 = Maj+F9 (avec note), 2 = automatique après un plantage
struct SdGuard { SdGuard() { SD_LOCK(); } ~SdGuard() { SD_UNLOCK(); } };
static void Tester_SaveLog(int kind)
{
  static uint16_t s_last = 0;
  char path[48], bmp[48];
  int w = 0, h = 0;
  uint8_t *img = Snap_Capture(w, h);                  // capture d'abord (écran tel quel)
  const uint32_t sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
  LogOut.printf("\n===== %s : journal ESP32-CoCo3-Pro v" FW_VERSION_SHORT " (" FW_VERSION_DATE "), en marche depuis %02lu:%02lu:%02lu =====\n",
                kind == 2 ? "AUTO (apres plantage)" : (kind == 1 ? "Maj+F9" : "F9"),
                (unsigned long)(sec / 3600), (unsigned long)(sec / 60 % 60), (unsigned long)(sec % 60));
  if (kind == 1 && g_log_note[0]) LogOut.printf("[NOTE] %s\n", g_log_note);
  Report_Config();
  g_cmd_inject = "W\nU\nJ\n";
  PollDebugCommands();
  SdGuard sd_guard;                                   // SD sous verrou (menu, disques, Save State)
  if (!Sd_NextName(DEBUG_DIR "/LOGS", "LOG", "TXT", path, sizeof path, s_last))
  {
    if (img) free(img);
    LogOut.println("[LOG] carte SD absente ou pleine : journal non enregistre");
    Overlay_Request("LOG NOT SAVED", "SD card missing or full", 2500);
    return;
  }
  uint8_t *buf = (uint8_t *)heap_caps_malloc(LogTee::RING, MALLOC_CAP_SPIRAM);
  File f = SD_MMC.open(path, FILE_WRITE);
  if (!buf || !f)
  {
    if (buf) free(buf);
    if (img) free(img);
    if (f) f.close();
    LogOut.printf("[LOG] impossible d'ecrire %s\n", path);
    Overlay_Request("LOG NOT SAVED", path, 2500);
    return;
  }
  f.printf("ESP32-CoCo3-Pro firmware v%s (%s) - log %s - uptime %02lu:%02lu:%02lu\r\n",
           FW_VERSION_SHORT, FW_VERSION_DATE, path, (unsigned long)(sec / 3600), (unsigned long)(sec / 60 % 60), (unsigned long)(sec % 60));
  { struct tm lt; if (Net_LocalTime(&lt)) f.printf("Date : %04d-%02d-%02d %02d:%02d:%02d\r\n", lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec); }
  if (kind == 1 && g_log_note[0]) f.printf("NOTE DU TESTEUR / TESTER NOTE : %s\r\n", g_log_note);
  if (kind == 2) f.print("JOURNAL AUTOMATIQUE APRES UN PLANTAGE / AUTOMATIC LOG AFTER A CRASH\r\n");
  f.print("\r\n");
  if (g_prev_tail && g_prev_tail_len)                 // ce qui précédait le redémarrage
  {
    f.print("===== Dernieres lignes avant le redemarrage / Last lines before the restart =====\r\n");
    Log_WriteText(f, (const uint8_t *)g_prev_tail, g_prev_tail_len, 0);
    f.print("\r\n===== Demarrage actuel / Current boot =====\r\n");
  }
  const size_t n = LogOut.snapshot(buf);
  size_t lineStart = 0;
  if (n == LogTee::RING)                    // anneau plein : 1re ligne coupée, on la saute
  {
    while (lineStart < n && buf[lineStart] != '\n') lineStart++;
    if (lineStart < n) lineStart++;
    f.print("[...] (debut du journal ecrase : anneau de 64 Ko plein)\r\n");
  }
  Log_WriteText(f, buf, n, lineStart);
  f.close();
  free(buf);
  // capture jumelée, même numéro (LOGnnnn.BMP)
  bool bmpOk = false;
  if (img)
  {
    strncpy(bmp, path, sizeof bmp); bmp[sizeof bmp - 1] = 0;
    char *dot = strrchr(bmp, '.');
    if (dot) strcpy(dot, ".BMP");
    bmpOk = Snap_WriteBMP(bmp, img, w, h);
    free(img);
  }
  LogOut.printf("[LOG] journal enregistre : %s%s\n", path, bmpOk ? " (+ capture .BMP)" : "");
  if (kind == 2) Overlay_Request("CRASH LOG SAVED", "Please send this file:", 5000, path);
  else Overlay_Request("LOG SAVED", path, 2000);
}

static void Tester_Snapshot(void)
{
  static uint16_t s_last = 0;
  char path[48];
  int w = 0, h = 0;
  uint8_t *img = Snap_Capture(w, h);
  if (!img) return;
  SdGuard sd_guard;                                   // SD sous verrou
  if (!Sd_NextName(DEBUG_DIR "/SNAPS", "SNAP", "BMP", path, sizeof path, s_last) || !Snap_WriteBMP(path, img, w, h))
  {
    free(img);
    LogOut.println("[SNAP] carte SD absente ou pleine : capture non enregistree");
    Overlay_Request("SNAPSHOT NOT SAVED", "SD card missing or full", 2500);
    return;
  }
  free(img);
  LogOut.printf("[SNAP] capture enregistree : %s (%d x %d)\n", path, w, w >= 640 ? h * 2 : h);
  Overlay_Request("SNAPSHOT SAVED", path, 1500);
}

// fichiers d'aide pour les testeurs à la racine de la carte SD (créés s'ils manquent)
static void Tester_WriteReadme(void)
{
  if (!SD_MMC_MountOK) return;
  if (!SD_MMC.exists(DEBUG_DIR)) SD_MMC.mkdir(DEBUG_DIR);
  if (!SD_MMC.exists(DEBUG_DIR "/LISEZMOI-TESTEURS.TXT"))
  {
    File f = SD_MMC.open(DEBUG_DIR "/LISEZMOI-TESTEURS.TXT", FILE_WRITE);
    if (f)
    {
      f.print("ESP32-CoCo3-Pro - touches pour les testeurs\r\n\r\n"
              "F12                 Menu de l'emulateur (Echap pour revenir)\r\n"
              "F10                 Aide : touches et options (2 pages)\r\n"
              "F9                  Journal + capture -> /ESP-DEBUG/LOGS/LOGnnnn.TXT et .BMP\r\n"
              "Maj+F9              Idem, avec une note tapee a l'ecran (3 lignes ; Entree = OK, Echap = annuler)\r\n"
              "Impr. ecran         Capture d'ecran -> /ESP-DEBUG/SNAPS/SNAPnnnn.BMP\r\n"
              "Insert (maintenu)   Turbo : Overclock Machine tant que la touche est enfoncee\r\n"
              "Ctrl+Alt+Suppr      Redemarrage de l'ESP32\r\n\r\n"
              "Apres un plantage, un journal est enregistre automatiquement au redemarrage.\r\n"
              "Pour signaler un probleme : envoyez le fichier LOGnnnn.TXT (et .BMP),\r\n"
              "le nom de l'image disque et ce que vous avez fait.\r\n");
      f.close();
    }
  }
  if (!SD_MMC.exists(DEBUG_DIR "/README-TESTERS.TXT"))
  {
    File f = SD_MMC.open(DEBUG_DIR "/README-TESTERS.TXT", FILE_WRITE);
    if (f)
    {
      f.print("ESP32-CoCo3-Pro - keys for testers\r\n\r\n"
              "F12                 Emulator menu (Esc to go back)\r\n"
              "F10                 Help: keys and options (2 pages)\r\n"
              "F9                  Log + screenshot -> /ESP-DEBUG/LOGS/LOGnnnn.TXT and .BMP\r\n"
              "Shift+F9            Same, with a note typed on screen (3 lines; Enter = OK, Esc = cancel)\r\n"
              "Print Screen        Screenshot -> /ESP-DEBUG/SNAPS/SNAPnnnn.BMP\r\n"
              "Insert (held)       Turbo: Machine overclock while the key is held\r\n"
              "Ctrl+Alt+Delete     Reboot the ESP32\r\n\r\n"
              "After a crash, a log is saved automatically at the next boot.\r\n"
              "To report a problem: send the LOGnnnn.TXT file (and .BMP),\r\n"
              "the disk image name and what you did.\r\n");
      f.close();
    }
  }
}

// Appelée par loop (coeur 1, tâche loop) : travaux demandés par le clavier.
volatile bool g_req_note = false;
static void Tester_Poll(void)
{
  static bool s_init = false, s_auto_done = false;
  if (!s_init)
  {
    s_init = true;
    const esp_reset_reason_t rr = esp_reset_reason();
    s_crash_boot = !g_boot_after_requested_reboot && (rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT ||
                    rr == ESP_RST_WDT || rr == ESP_RST_BROWNOUT);
    Tester_WriteReadme();
  }
  if (g_req_reboot)
  {
    LogOut.println("[BOOT] Ctrl+Alt+Suppr : redemarrage");
    Sys_Restart();
  }
  // journal automatique 10 s après un démarrage qui suit un plantage
  if (s_crash_boot && !s_auto_done && millis() > 10000)
  {
    s_auto_done = true;
    Tester_SaveLog(2);
  }
  if (g_req_snap) { g_req_snap = false; Tester_Snapshot(); }
  if (g_req_log)  { g_req_log = false;  Tester_SaveLog(0); }
  if (g_log_note_ready)
  {
    g_log_note_ready = false;
    vTaskDelay(pdMS_TO_TICKS(150));   // laisse le coeur vidéo effacer le cadre de saisie avant la capture
    Tester_SaveLog(1);
  }
  if (g_req_secret)
  {
    g_req_secret = false;
    Overlay_Request("Programmed by Luc Dion 2026", "ESP32-CoCo3-Pro v" FW_VERSION_SHORT, 5000);
  }
}

// Touches spéciales, lues dans l'instantané clavier (tâche USB). Les touches traitées sont retirées de
// l'instantané : le CoCo ne les voit pas.
static void Tester_Keys(uint8_t *Data)
{
  extern uint8_t g_oc_mode; extern void SetOverclockMode(uint8_t);
  static bool s_f9 = false, s_prt = false, s_ins = false, s_cad = false, s_cab = false;
  static uint8_t s_oc_saved = 0;
  const bool ctrl = (Data[0] & 0x11) != 0, alt = (Data[0] & 0x44) != 0;
  bool f9 = false, prt = false, ins = false, del = false, bks = false;
  for (uint8_t k = 2; k != 7; k++)
  {
    switch (Data[k])
    {
    case 0x42: f9 = true;  Data[k] = 0; break;           // F9
    case 0x46: prt = true; Data[k] = 0; break;           // Impr. écran
    case 0x49: ins = true; Data[k] = 0; break;           // Insert
    case 0x4C: del = true; if (ctrl && alt) Data[k] = 0; break;   // Suppr
    case 0x2A: bks = true; if (ctrl && alt) Data[k] = 0; break;   // Retour arrière
    default: break;
    }
  }
  extern volatile bool g_req_note;
  if (f9 && !s_f9)
  {
    if (Data[0] & 0x22) g_req_note = true;
    else g_req_log = true;
  }
  if (prt && !s_prt) g_req_snap = true;
  if (ins != s_ins)
  {
    if (ins) { s_oc_saved = g_oc_mode; SetOverclockMode(OC_MACHINE); }
    else SetOverclockMode(s_oc_saved);
  }
  const bool cad = ctrl && alt && del, cab = ctrl && alt && bks;
  if (cad && !s_cad) g_req_reboot = true;
  if (cab && !s_cab) g_req_secret = true;
  s_f9 = f9; s_prt = prt; s_ins = ins; s_cad = cad; s_cab = cab;
}

// Superposition dessinée par le coeur vidéo : émulation suspendue, cadre au centre de l'image affichée
// (sur les deux tampons), puis reprise. L'image suivante efface le cadre.
extern void DrawText(const char *text, uint8_t Xpos, uint8_t Ypos, uint8_t XpixOffset, uint8_t YpixOffset, uint8_t ForeColor, uint8_t BackColor);
static void Overlay_Draw(void)
{
  const int w = vga->mode.hRes, h = vga->mode.vRes;
  int nl = 0, maxc = 0;
  for (int i = 0; i < OVL_LINES; i++)
  {
    const int l = (int)strlen(g_overlay_l[i]);
    if (l) nl = i + 1;
    if (l > maxc) maxc = l;
  }
  if (!nl) return;
  const int bw = 6 * maxc + 16, bh = 10 * nl + 12;
  const int bx = (w - bw) / 2, by = (h - bh) / 2;
  for (int pass = 0; pass < 2; pass++)                 // les deux tampons
  {
    for (int y = by; y < by + bh; y++)
      for (int x = bx; x < bx + bw; x++)
      {
        const bool border = (y == by || y == by + bh - 1 || x == bx || x == bx + bw - 1);
        vga->dot(x, y, border ? 0xDB : 0x01);             // cadre blanc, fond bleu
      }
    for (int i = 0; i < nl; i++)
    {
      const int l = (int)strlen(g_overlay_l[i]);
      if (!l) continue;
      const int x = bx + (bw - 6 * l) / 2, y = by + 7 + 10 * i;
      DrawText(g_overlay_l[i], (uint8_t)(x / 6), (uint8_t)(y / 8), (uint8_t)(x % 6), (uint8_t)(y % 8), i == 0 ? 0xD8 : 0xDB, 0x01);
    }
    vga->show();
  }
}

// émulation suspendue. Entrée = enregistrer, Échap = annuler, Retour arrière.
char Note_HidToChar(uint8_t k, bool shift)   // aussi pour le mot de passe Wi-Fi (menu N)
{
  if (k >= 0x04 && k <= 0x1D) return (char)((shift ? 'A' : 'a') + (k - 0x04));
  if (k >= 0x1E && k <= 0x27)
  {
    static const char n[] = "1234567890", sn[] = "!@#$%^&*()";
    return shift ? sn[k - 0x1E] : n[k - 0x1E];
  }
  switch (k)
  {
  case 0x2C: return ' ';
  case 0x2D: return shift ? '_' : '-';
  case 0x2E: return shift ? '+' : '=';
  case 0x2F: return shift ? '{' : '[';
  case 0x30: return shift ? '}' : ']';
  case 0x31: return shift ? '|' : '\\';
  case 0x35: return shift ? '~' : '`';
  case 0x33: return shift ? ':' : ';';
  case 0x34: return shift ? '"' : '\'';
  case 0x36: return shift ? '<' : ',';
  case 0x37: return shift ? '>' : '.';
  case 0x38: return shift ? '?' : '/';
  default: return 0;
  }
}
static void Note_Input(void)
{
  extern uint8_t HID_ModifierByte;
  char txt[90] = "";                        // 3 lignes de 30 (curseur compris)
  size_t len = 0;
  uint8_t prev = sf.DIRECT_Key_Code;
  const uint32_t t0 = millis();
  bool dirty = true;
  while (millis() - t0 < 120000)            // 2 min au plus
  {
    if (dirty)
    {
      dirty = false;
      char tmp[92];
      snprintf(tmp, sizeof tmp, "%s_", txt);
      const size_t tl = strlen(tmp);
      Overlay_Set(0, "TESTER NOTE  ENTER:OK ESC:EXIT");
      for (int i = 0; i < 3; i++)
      {
        char line[OVL_COLS + 1];
        const size_t st = (size_t)i * OVL_COLS;
        size_t n = st < tl ? tl - st : 0;
        if (n > OVL_COLS) n = OVL_COLS;
        memcpy(line, tmp + (st < tl ? st : tl), n);
        memset(line + n, ' ', OVL_COLS - n);   // largeur fixe : le cadre ne change pas
        line[OVL_COLS] = 0;
        Overlay_Set(i + 1, line);
      }
      Overlay_Draw();
    }
    const uint8_t k = sf.DIRECT_Key_Code;
    if (k != prev && k != 0)
    {
      if (k == MENU_ENTER)
      {
        strncpy(g_log_note, txt, sizeof g_log_note - 1);
        g_log_note[sizeof g_log_note - 1] = 0;
        g_log_note_ready = true;
        break;
      }
      if (k == MENU_ESC) break;
      if (k == 0x2A) { if (len) txt[--len] = 0; dirty = true; }
      else
      {
        const char c = Note_HidToChar(k, (HID_ModifierByte & 0x22) != 0);
        if (c && len < sizeof txt - 1) { txt[len++] = c; txt[len] = 0; dirty = true; }
      }
    }
    prev = k;
    vTaskDelay(pdMS_TO_TICKS(15));
  }
  vTaskDelay(pdMS_TO_TICKS(150));
}

// ---------------------------------------------------------------------------
// imprimante du port série (bit banger, mode PRINTER) -> fichier texte /PRINTS/PRTnnnn.TXT
// ---------------------------------------------------------------------------
static void Prn_Service(bool force_close = false)
{
  static File s_f;
  static bool s_open = false, s_cr = false, s_err_shown = false;
  static char s_path[48];
  static uint16_t s_last = 0;
  static uint8_t s_pre[4];
  static uint8_t s_npre = 0;
  static uint32_t s_bytes = 0;
  uint8_t out[300];
  size_t n = 0;
  int c;
  while (n < sizeof out - 2 && (c = Bb_PrnGet()) >= 0)
  {
    const uint8_t b = (uint8_t)c;
    if (!s_open)
    {
      const bool text = (b >= 0x20 && b < 0x7F) || b == 13 || b == 10 || b == 9 || b == 12;
      if (!text) { s_npre = 0; continue; }
      s_pre[s_npre++] = b;
      if (s_npre < sizeof s_pre) continue;
      if (s_pre[0] == s_pre[1] && s_pre[1] == s_pre[2] && s_pre[2] == s_pre[3])   // « UUUU » : onde carrée du CNA
      { s_pre[0] = s_pre[1]; s_pre[1] = s_pre[2]; s_pre[2] = s_pre[3]; s_npre = 3; continue; }
      bool ok;
      SD_LOCK();
      ok = Sd_NextName("/PRINTS", "PRT", "TXT", s_path, sizeof s_path, s_last);
      if (ok) { s_f = SD_MMC.open(s_path, FILE_WRITE); ok = (bool)s_f; }
      SD_UNLOCK();
#ifndef ESP_PLATFORM
      if (getenv("BBDBG")) fprintf(stderr, "[PRINT] ouverture %s : %d\n", s_path, (int)ok);
#endif
      if (!ok)
      {
        s_npre = 0;
        if (!s_err_shown) { s_err_shown = true; LogOut.println("[PRINT] carte SD absente ou pleine : impression perdue"); Overlay_Request("PRINT NOT SAVED", "SD card missing or full", 2500); }
        continue;
      }
      s_open = true; s_cr = false; s_bytes = 0;
      LogOut.printf("[PRINT] impression -> %s\n", s_path);
      for (uint8_t i = 0; i < s_npre; i++)
      {
        const uint8_t p = s_pre[i];
        if (p == 13) { out[n++] = 13; out[n++] = 10; s_cr = true; }
        else if (p == 10 && s_cr) s_cr = false;
        else { out[n++] = p; s_cr = false; }
      }
      s_npre = 0;
      continue;
    }
    if (b == 13) { out[n++] = 13; out[n++] = 10; s_cr = true; }
    else if (b == 10 && s_cr) s_cr = false;          // CR LF envoyés par le programme : pas de ligne vide
    else { out[n++] = b; s_cr = false; }
  }
  if (n && s_open)
  {
    SD_LOCK();
    s_f.write(out, n);
    SD_UNLOCK();
    s_bytes += n;
  }
  const uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000);
  if (force_close || (uint32_t)(ms - Bb_PrnLastMs()) > 5000)
  {
    s_npre = 0;
    if (s_open)
    {
      SD_LOCK();
      s_f.close();
      SD_UNLOCK();
      s_open = false;
      s_err_shown = false;
      LogOut.printf("[PRINT] %s ferme (%lu octets)\n", s_path, (unsigned long)s_bytes);
      Overlay_Request("PRINTED TO SD CARD", s_path, 2000);
    }
  }
}

void Log_Line(const char *s) { LogOut.println(s); }
extern const char *const g_fw_version_short = FW_VERSION_SHORT;   // modem (ATI) // journal pour les modules (cc3net)

void loop()
{
  PollDebugCommands();
  { extern void Mem_PrintFailures(void); Mem_PrintFailures(); }
  Tester_Poll();   // F9, Impr. écran, Ctrl+Alt+Suppr, message
  Prn_Service();   // imprimante -> fichier sur la SD
  Net_Poll();      // Wi-Fi (journal des changements d'état, heure NTP)
  Modem_MainPoll(); // AT&W (flash) hors de la tâche du modem
  NetShare_Poll(); // renvoi au PC des disquettes modifiées
  Web_Poll();      // page web (un morceau par passage, jamais bloquant)
  Autorun_Poll();  // AUTORUN
  Health_Sample(); // charge des coeurs, température, SD (une fois par seconde)
  Cas_Service();   // cassette (lecture anticipée du .WAV, écriture des enregistrements)
  Fw_ValidateIfDue(false);   // nouvelle version validée après 15 s
  vTaskDelay(1);   // loopTask (framework Arduino, Core 1) doit exister
                    // mais ne fait rien: la job CPU est dans CPUCore_Task
}

// ---------------------------------------------------------------------------
// mesure de charge du coeur CPU (commande série « W »). busy_cc : cycles ESP32 (CCOUNT, 240 MHz)
// passés dans les rafales d'émulation ; charge = busy_cc / durée de la fenêtre (~2 s). lost
// ---------------------------------------------------------------------------
struct perf_t { uint32_t busy_cc, bursts, capped, lost, line_cc, io_cc, instr; };
perf_t g_perf;   // aussi incrémenté par PIA0_HSync et le bus E/S
volatile uint32_t g_perf_load_x10 = 0, g_perf_peak_x10 = 0, g_perf_lost_ps = 0, g_perf_capped_ps = 0,
                  g_perf_hz = 0, g_perf_target_hz = 0,
                  g_perf_line_x10 = 0, g_perf_io_x10 = 0, g_perf_ips = 0, g_perf_cpi = 0,   // détail
                  g_perf_wait_x10 = 0;   // 6809 en SYNC/CWAI
static inline void Perf_AddIo(uint32_t cc) { g_perf.io_cc += cc; }
static inline void Perf_AddLine(uint32_t cc) { g_perf.line_cc += cc; }
static void Perf_CloseWindow(uint32_t us, uint32_t cpu_cycles, uint32_t target_hz)
{
  if (!us) return;
#ifdef ESP_PLATFORM
  const uint32_t mhz = getCpuFrequencyMhz();
#else
  const uint32_t mhz = 240;
#endif
  const uint32_t load = (uint32_t)((uint64_t)g_perf.busy_cc * 1000ull / ((uint64_t)us * mhz));
  g_perf_load_x10 = load;
  if (load > g_perf_peak_x10) g_perf_peak_x10 = load;
  g_perf_lost_ps = (uint32_t)((uint64_t)g_perf.lost * 1000000ull / us);
  g_perf_capped_ps = (uint32_t)((uint64_t)g_perf.capped * 1000000ull / us);
  g_perf_hz = (uint32_t)((uint64_t)cpu_cycles * 1000000ull / us);
  g_perf_target_hz = target_hz;
  g_perf_line_x10 = (uint32_t)((uint64_t)g_perf.line_cc * 1000ull / ((uint64_t)us * mhz));
  g_perf_io_x10   = (uint32_t)((uint64_t)g_perf.io_cc * 1000ull / ((uint64_t)us * mhz));
  g_perf_ips      = (uint32_t)((uint64_t)g_perf.instr * 1000000ull / us);
  { extern volatile uint32_t g_cpu_wait_cycles; static uint32_t s_w0 = 0;   // part du temps 6809 en SYNC/CWAI
    const uint32_t w = g_cpu_wait_cycles; g_perf_wait_x10 = cpu_cycles ? (uint32_t)((uint64_t)(w - s_w0) * 1000ull / cpu_cycles) : 0; s_w0 = w; }
  g_perf_cpi      = g_perf.instr ? (uint32_t)(((uint64_t)g_perf.busy_cc - g_perf.line_cc - g_perf.io_cc) / g_perf.instr) : 0;
  memset(&g_perf, 0, sizeof g_perf);
}
// Rafale d'émulation : jusqu'à `need` cycles GIME (ou 12 000 cycles CPU). En IRAM avec StepCPU /
// Fdc_Tick incorporés
static IRAM_ATTR __attribute__((noinline)) uint32_t Cpu_RunBurst(int32_t need, uint32_t *cpu_cyc_out)
{
  uint32_t burst = 0, cpu_cyc = 0, n = 0;
  do
  {
    uint32_t k;
    const int32_t left = need - (int32_t)burst;
    cpu_cyc += Cpu_Batch(left > 0 ? (uint32_t)left : 1u, &k);   // par lots
    burst += g_last_gime_cycles;   // temps GIME (= cycles CPU hors mode OC_CPU)
    n += k;
  } while ((int32_t)burst < need && cpu_cyc < 12000);
  g_perf.instr += n;
  *cpu_cyc_out = cpu_cyc;
  return burst;
}


// ligne émulée visée au VSYNC VGA. Le rendu démarre ~1,5 ms (~23 lignes) après le VSYNC ; la trame
// émulée est publiée à la ligne 0. 108 + 23 = 131 = demi-trame
#define PHASE_TARGET_LINE 108
// le 6809 DORT quand il est à l'heure (au lieu de céder la main en boucle avec taskYIELD) le coeur 1
// devient libre pour loop (page web, SD, commandes) et la charge mesurée devient la vraie
#define CPU_SLEEP_WHEN_AHEAD 1
#define CPU_SLEEP_GUARD_US   1300
#define CPU_SLEEP_MAX_MS     20
volatile uint32_t g_cpu_sleeps = 0, g_cpu_forced_sleeps = 0;   // W : sommeils par seconde
void CPUCore_Task(void *pvParameters)
{
    uint64_t t_ref_us = esp_timer_get_time();
    int32_t  total_cycles_since_ref = 0;   // signé : report de trame
    uint32_t last_seen_frame = 0;
    uint32_t diag_cumulative_cycles = 0;
#if CPU_SLEEP_WHEN_AHEAD
    UBaseType_t prio_now = 1;
    uint32_t last_sleep_ms = millis();
#endif

    for (;;)
    {
#if CPU_SLEEP_WHEN_AHEAD
        {
            const UBaseType_t want = sf.CycleAccurateMode ? 2 : 1;
            if (want != prio_now) { prio_now = want; vTaskPrioritySet(NULL, want); }
            const uint32_t ms = millis();
            if (prio_now == 2 && (uint32_t)(ms - last_sleep_ms) >= CPU_SLEEP_MAX_MS)
            {
                vTaskDelay(1); g_cpu_forced_sleeps++; last_sleep_ms = millis();
            }
        }
#endif
        if (g_resync_cpu_clock || g_vsync_frame_count != last_seen_frame)
        {
            const uint64_t now_us = esp_timer_get_time();
            // le retard (ou l'avance) accumulé à la fin de la trame VGA est REPORTÉ sur la suivante au
            // lieu d'être abandonné. ~35 cycles étaient perdus à chaque trame (-0,23 %)
            int32_t carry = 0;
            if (!g_resync_cpu_clock && sf.CycleAccurateMode)
            {
                const uint32_t shift_c = (sf.CPU_Speed == CPU_FAST) ? 3 : 4;
                const uint32_t k_c = (uint32_t)((((uint64_t)GIME_TICKS_PER_LINE * GIME_LINES_PER_FRAME) << 16) /
                                                ((uint64_t)g_vga_frame_us << shift_c));
                const int32_t target_now = (int32_t)(((uint64_t)(uint32_t)(now_us - t_ref_us) * k_c) >> 16);
                carry = target_now - total_cycles_since_ref;
                Oc_Adapt(carry);   // multiplicateur de l'Overclock CPU
                if (carry >  2000) { g_perf.lost += (uint32_t)(carry - 2000); carry =  2000; }   // cycles abandonnés (ESP32 trop lent)
                if (carry < -2000) carry = -2000;

                // VERROUILLAGE DE PHASE trame émulée / trame VGA. Le cadencement donnait une trame
                // émulée par trame VGA en MOYENNE, mais rien ne fixait leur position relative
                {
                    const int32_t cpl   = (int32_t)(GIME_TICKS_PER_LINE >> shift_c);        // cycles par ligne
                    const int32_t frame = cpl * (int32_t)GIME_LINES_PER_FRAME;
                    int32_t since_vs = (int32_t)(((uint64_t)(uint32_t)(now_us - g_vsync_us) * k_c) >> 16);
                    if (since_vs < 0 || since_vs > frame) since_vs = 0;   // VSYNC arrivé entre-temps
                    // position (en cycles) que l'émulation DEVRAIT avoir au VSYNC
                    int32_t pos = (int32_t)gime_raster.line * cpl + (int32_t)(gime_raster.line_ticks >> shift_c)
                                  + carry - since_vs;
                    int32_t e = pos - PHASE_TARGET_LINE * cpl;
                    e %= frame; if (e < -frame / 2) e += frame; else if (e >= frame / 2) e -= frame;
                    g_phase_err_lines = e / cpl;
                    int32_t corr = e / 16;                           // constante de temps ~16 trames
                    if (corr >  2 * cpl) corr =  2 * cpl;            // au plus 2 lignes par trame (0,8 %)
                    if (corr < -2 * cpl) corr = -2 * cpl;
                    carry -= corr;                                   // en avance -> on ralentit
                }
            }
            last_seen_frame = g_vsync_frame_count;
            t_ref_us = now_us;
            total_cycles_since_ref = -carry;
            g_resync_cpu_clock = false;
        }
                static uint64_t last_report_us = 0;
        static uint32_t last_report_cycles = 0;
        static uint32_t last_report_frame = 0;
        uint64_t now_report_us = esp_timer_get_time();
        if (now_report_us - last_report_us > 2000000ULL)
        {
            // cible réelle = une trame émulée par trame VGA (238 944 ticks / 16 ou 8, x fréquence VGA
            // mesurée ; 912 ticks par ligne).
            uint32_t target_hz_dbg = (uint32_t)((((uint64_t)GIME_TICKS_PER_LINE * GIME_LINES_PER_FRAME) * 1000000ULL) /
                                                ((uint64_t)g_vga_frame_us << ((sf.CPU_Speed == CPU_FAST) ? 3 : 4)));
            Perf_CloseWindow((uint32_t)(now_report_us - last_report_us), diag_cumulative_cycles - last_report_cycles, target_hz_dbg);
            last_report_us = now_report_us;
            last_report_cycles = diag_cumulative_cycles;
            last_report_frame = g_vsync_frame_count;

        }
        if (g_rs_present && !sf.CPU_HALTED_BY_EMULATOR) Rs_Poll();   // RS-232 Pak (aussi pendant SYNC/CWAI)
        if (g_bb_mode != BB_OFF && !sf.CPU_HALTED_BY_EMULATOR) Bb_Poll();   // bit banger
        // Gestion CWAI / attente : reprise de la logique DoCPU actuelle ---
        if (CPU_in_WAIT_STATE)
        {
            if (sf.CPU_HALTED_BY_EMULATOR)
            {
                g_cpu_parked = true;    // entre deux rafales, état cohérent (Save State)
#if CPU_SLEEP_WHEN_AHEAD
                vTaskDelay(1); last_sleep_ms = millis();   // menu F12 / arrêt : le coeur 1 est libre
#else
                                taskYIELD();
#endif
                continue;
            }
            if (!WaitStateIdleTick())   // SYNC/CWAI : le temps GIME avance
            {
                                taskYIELD();
                continue;
            }
            else
            {
                CPU_in_WAIT_STATE = false;
            }
        }

         if (sf.CycleAccurateMode)
        {
            // exactement UNE trame émulée (262 x 912 ticks de 14,318 MHz = 14 901,25 cycles à 0,89
            // MHz, le double à 1,79 MHz) par trame VGA, répartie uniformément sur la trame mesurée
            const uint32_t shift = (sf.CPU_Speed == CPU_FAST) ? 3 : 4;   // ticks par cycle = 8 / 16
            const uint32_t k_q16 = (uint32_t)((((uint64_t)GIME_TICKS_PER_LINE * GIME_LINES_PER_FRAME) << 16) /
                                              ((uint64_t)g_vga_frame_us << shift));
            const uint32_t elapsed_us = (uint32_t)(esp_timer_get_time() - t_ref_us);
            const int32_t target_cycles = (int32_t)(((uint64_t)elapsed_us * k_q16) >> 16);
            if (total_cycles_since_ref < target_cycles)
            {
                // En retard sur l'horloge réelle -> rattraper d'un coup rafale dans une fonction en
                // IRAM (StepCPU, FDC et GIME compris) + mesure de charge (compteur CCOUNT).
                uint32_t cpu_cyc = 0;
                const uint32_t cc0 = Perf_CCount();
                const uint32_t burst = Cpu_RunBurst(target_cycles - total_cycles_since_ref, &cpu_cyc);
                { const uint32_t dcc = Perf_CCount() - cc0; g_perf.busy_cc += dcc; g_oc_busy_cc += dcc; }
                g_perf.bursts++;
                if (cpu_cyc >= 12000) g_perf.capped++;
                total_cycles_since_ref += (int32_t)burst;
                diag_cumulative_cycles += cpu_cyc;
            }
            else
            {
                // À l'heure ou en avance -> céder la main sans bloquer le watchdog IDLE du coeur.
                // Éviter une boucle vide pure.
#if CPU_SLEEP_WHEN_AHEAD
                // dormir jusqu'au tic suivant si le VSYNC n'est pas proche
                const uint32_t since_vs_us = (uint32_t)(esp_timer_get_time() - g_vsync_us);
                if (since_vs_us + CPU_SLEEP_GUARD_US < g_vga_frame_us)
                {
                    vTaskDelay(1); g_cpu_sleeps++; last_sleep_ms = millis();
                }
                else taskYIELD();
#else
                taskYIELD();
#endif
            }
        }
         else

        {
            // overclock : rafale en IRAM, comme le mode cycle-accurate
            // même rafale en IRAM que le mode cycle-accurate
            uint32_t cpu_cyc = 0;
            const uint32_t cc0 = Perf_CCount();
            Cpu_RunBurst(2000, &cpu_cyc);
            g_perf.busy_cc += Perf_CCount() - cc0;
            g_perf.bursts++;
            diag_cumulative_cycles += cpu_cyc;
        }
    }
}

// les petites allocations (malloc / new / String) de plus de MALLOC_EXT_LIMIT octets vont en PSRAM
#define MALLOC_EXT_LIMIT 256
void setup()
{
#if MALLOC_EXT_LIMIT
  heap_caps_malloc_extmem_enable(MALLOC_EXT_LIMIT);
#endif
  Serial.begin(921600);
  LogOut.begin();   // journal en mémoire (F9)
  delay(200);
  // ligne courte pour la bêta ; historique complet si BOOT_LOG_VERBOSE.
  LogOut.println("[VERSION] ESP32-CoCo3-Pro firmware v" FW_VERSION_SHORT " (" FW_VERSION_DATE ")");
  {
    // cause du dernier démarrage (plantage, chien de garde, chute de tension...)
    const esp_reset_reason_t rr = esp_reset_reason();
    const char *why = rr == ESP_RST_POWERON ? "mise sous tension" : rr == ESP_RST_SW ? "redemarrage logiciel" :
                      rr == ESP_RST_PANIC ? "PLANTAGE (exception)" : rr == ESP_RST_INT_WDT ? "PLANTAGE (chien de garde interruptions)" :
                      rr == ESP_RST_TASK_WDT ? "PLANTAGE (chien de garde taches)" : rr == ESP_RST_WDT ? "PLANTAGE (chien de garde)" :
                      rr == ESP_RST_BROWNOUT ? "CHUTE DE TENSION (alimentation)" : rr == ESP_RST_EXT ? "bouton reset" :
                      rr == ESP_RST_DEEPSLEEP ? "reveil" : "inconnue";
    LogOut.printf("[BOOT] cause du demarrage : %s (%d)\n", why, (int)rr);
    if (rr != ESP_RST_POWERON && rr != ESP_RST_SW && rr != ESP_RST_EXT && g_prev_tail)
    {
      if (g_rtc_step == STEP_REBOOT)   // redémarrage demandé, fini par le chien de garde
      {
        g_boot_after_requested_reboot = true;
        LogOut.println("[BOOT] redemarrage demande, termine par le chien de garde (pas un plantage du CoCo)");
      }
      else LogOut.printf("[BOOT] etape en cours au plantage : %s (%lu)\n", Step_Name(g_rtc_step), (unsigned long)g_rtc_step);
    }
    g_rtc_step = STEP_IDLE;
  }

  AllocatePSRAMBuffers();
BOOTLOG("[DIAG] Heap interne libre = %u octets\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  // l'objet CPU (registres 6809 + tables de décodage, ~14 Ko) en RAM INTERNE. instruction lisait ses
  // registres et ses tables à travers le cache PSRAM partagé avec la vidéo.
  {
    void *mem = heap_caps_malloc(sizeof(cpu_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    cpu = mem ? new (mem) cpu_t() : new cpu_t();
    BOOTLOG("[DIAG] objet CPU (%u o) en %s\n", (unsigned)sizeof(cpu_t), mem ? "RAM interne" : "PSRAM");
  }

  mode0 = Mode::MODE_320x240x60;
  mode1 = MODE_COCO_320x240;   // 60,00 Hz (Mode::MODE_320x240x60_4_3 à 529 lignes)
  mode2 = Mode::MODE_640x240x60;
  mode3 = Mode::MODE_640x240x60_4_3;

#ifdef DEBUG_ALL
  pinMode(DEBUG1,OUTPUT);   // GPIO 13 = sortie cassette sur la carte
  pinMode(DEBUG2,OUTPUT);
#endif
  pinMode(LED_PORT_ANODE,OUTPUT);
  pinMode(LED_PORT_CATHODE,OUTPUT);

  InitPeripherals_and_Others();

  DISKETTE_LED_OFF();

#define SERIAL1_TX 41
#define SERIAL1_RX 42
#define SERIAL1_BAUD 115200

  Serial1.setRxBufferSize(2048);   // RS-232 Pak (liaison UART)
  Serial1.begin(SERIAL1_BAUD, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);

  g_cpu6309_run = g_cpu6309_sel;            // processeur choisi (menu C)
  cpu->set_cpu_6309(g_cpu6309_run);
  LogOut.printf("[CPU] %s\n", g_cpu6309_run ? "HD6309 (mode emulation au reset ; LDMD #1 = natif)" : "MC6809");
  cpu->reset();
  { extern void Cfg_MigrateLegacy(void); Cfg_MigrateLegacy(); }   // config.ccc / joyconfig.ccc -> .cfg (réglages déjà appliqués)
  { extern void State_InitName(void); State_InitName(); }   // dernier logiciel du Save State

  Setup_USB();
  LogOut.printf("[BOOT] Setup_USB() returned\n");
  BootScreen_Init();   // écran d'accueil : le CoCo attend (arrêté) qu'il soit fini
  if (BootScreen_Wanted()) { CPU_in_WAIT_STATE = true; sf.CPU_HALTED_BY_EMULATOR = true; }
  Autorun_Init(); // AUTORUN (lignes lues sur la SD ; tapées par loop à l'invite du BASIC)
  Web_Init();   // page web (réglages ; servie par loop quand le Wi-Fi est connecté)
  Net_Init();   // Wi-Fi (coupé par défaut ; après l'USB, qui prend sa RAM DMA d'abord)
  Fw_BootReport();   // partition en cours, retour automatique éventuel

  attachInterrupt(digitalPinToInterrupt(Field_Synch_Interrupt_PIN), Field_Synch_Interrupt_Flag, FALLING);
  LogOut.printf("[BOOT] attachInterrupt() done, creating VideoCore task ...\n");

  xTaskCreatePinnedToCore
  (
    VideoCore,        // Fonction
    "SystemCore",      // Name
    6048,              // stack
    NULL,              // Parameter
    1,
                       // touches manquées / réponse clavier lente)
    NULL,              // Handle
    0                  // Core 0 (Videocore)
  );
  xTaskCreatePinnedToCore
  (
    CPUCore_Task,       // Fonction
    "CPUCore",          // Name
    4096,               // Stack
    NULL,               // Parametre
    1,                  // Priorité
    NULL,               // Handle
    1   // coeur 1 // Core 1 CPUCore
  );
  LogOut.printf("[BOOT] setup() end (VideoCore task created)\n");
}

// DoCPU remplacé par CPUCore_Task dans setup

#define COCO2_GRAPHMODE_32X16_8X12 0b0 // Text 32x16
#define COCO2_GRAPHMODE_256X192X2 0b11110110  // PMODE 4
#define COCO2_GRAPHMODE_128X192X4 0b11100110  // PMODE 3 OK

#define COCO2_GRAPHMODE_128X192X2 0b11010101  // PMODE 2
#define COCO2_GRAPHMODE_128X96X4 0b11000100  // PMODE 1
#define COCO2_GRAPHMODE_128X96X4_1 0b11110100  // PMODE 1 Other mode??

#define COCO2_GRAPHMODE_128X96X2 0b10110011  // PMODE 0

//--------- Other modes in assembly language only, not officialy supported in Basic Coco 2
#define COCO2_GRAPHMODE_128X64X4 0b10100010
#define COCO2_GRAPHMODE_128X64X2 0b10010001

#define COCO2_GRAPHMODE_64X64X4 0b10000001

// ---------------------------------------------------------------------------
// choix du rendu CoCo 1/2 comme le GIME (MAME gime.cpp, BSD-3). (V2-V0) étaient reconnues
// ---------------------------------------------------------------------------
void Do_G6R(uint8_t lpr);
static uint8_t LegacyRendererFor(uint8_t m)   // m = Coco2GraphicMode sans CSS
{
  if ((m & 0x80) == 0) return COCO2_GRAPHMODE_32X16_8X12;
  static const uint8_t lpr_tab[8] = { 3, 3, 3, 2, 2, 1, 1, 1 };
  const uint8_t lpr = lpr_tab[m & 0x07];
  switch ((m >> 4) & 0x07)
  {
    case 0:                                   // G1C : 64 px, 4 couleurs
      return COCO2_GRAPHMODE_64X64X4;         // (seul rendu disponible)
    case 1: case 3: case 5:                   // G1R/G2R/G3R : 128 px, 2 coul.
      return (lpr == 1) ? COCO2_GRAPHMODE_128X192X2 :
             (lpr == 2) ? COCO2_GRAPHMODE_128X96X2  : COCO2_GRAPHMODE_128X64X2;
    case 2: case 4: case 6:                   // G2C/G3C/G6C : 128 px, 4 coul.
      return (lpr == 1) ? COCO2_GRAPHMODE_128X192X4 :
             (lpr == 2) ? COCO2_GRAPHMODE_128X96X4  : COCO2_GRAPHMODE_128X64X4;
    default:                                  // G6R : 256 px, 2 couleurs
      return (lpr == 1) ? COCO2_GRAPHMODE_256X192X2 :   // PMODE 4
             (lpr == 2) ? COCO2_GRAPHMODE_128X96X4_1 :  // Megabug (artefacts)
                          COCO2_GRAPHMODE_128X64X4;     // approximation
  }
}

#ifdef DEBUG_ALL
#define WAIT_VSYNCH_LOW()  while(gpio_get_level(VSYNC_PORT) != 0) {DEBUG1_SET; gime_video_capture_drain();}DEBUG1_CLR
#else
// temps d'attente du coeur vidéo (VSYNC sans ligne à vider, vTaskDelay de la boucle) : VIDEO WORK
volatile uint32_t g_vid_idle_cc = 0;
#define WAIT_VSYNCH_LOW()  do { const uint32_t w0_ = Perf_CCount(); uint32_t wk_ = 0; \
    while (gpio_get_level(VSYNC_PORT) != 0) { if (gime_video_capture_has_work()) { const uint32_t c0_ = Perf_CCount(); gime_video_capture_drain(); wk_ += Perf_CCount() - c0_; } } \
    g_vid_idle_cc += (Perf_CCount() - w0_) - wk_; } while (0)   // attente mesurée
#endif
#define X_OFFSET_32 312
// ---------------------------------------------------------------------------
// Les modes de plus de 320 pixels logiques (texte 64/80 col., graphiques 512/640) demandent la sortie
// VGA 640x240 (Mode 2, timings VGA standard) les autres restent en 320x240 4:3 (Mode 1), comme les…
// ---------------------------------------------------------------------------
#ifndef GIME_VGA_ALLOW_640
#define GIME_VGA_ALLOW_640 1
#endif

static bool VGAIs640(void)
{
  return vga->mode.hRes >= 640;
}

static void EnsureVGA320(void)
{
  if (VGAIs640())
  {
    SetVideoMode(VIDEO_MODE_320X240_4_3);
    sf.Coco2VideoGenMODE = 255;   // les rendus legacy effaceront l'écran
  }
}

// mesure du coeur vidéo (commande W) : images GIME rendues durée du rendu (moyenne / max) et part
// passée à vider la file de capture.
volatile uint32_t g_vid_frames = 0, g_vid_render_cc = 0, g_vid_render_max = 0, g_vid_drain_cc = 0;
static inline void VidDrain(void)
{
  const uint32_t c0 = Perf_CCount();
  gime_video_capture_drain();
  g_vid_drain_cc += Perf_CCount() - c0;
}
static void RenderGIMEFrame(void)
{
  const uint32_t rc0 = Perf_CCount();
#if GIME_VGA_ALLOW_640
  // Hystérésis : on ne change de mode VGA que si le besoin dure 8 trames (évite de basculer sur un
  // changement de mode fugitif d'un programme).
  static uint8_t s_switchCount = 0;
  const bool want640 = (gime_video_wanted_width() == 640);
  if (want640 != VGAIs640())
  {
    if (++s_switchCount >= 8)
    {
      s_switchCount = 0;
      SetVideoMode(want640 ? VIDEO_MODE_GIME_640 : VIDEO_MODE_320X240_4_3);
    }
  }
  else
  {
    s_switchCount = 0;
  }
#endif
  const int w = vga->mode.hRes;
  const int h = vga->mode.vRes;
  VidDrain();   // lignes en attente -> tampon publié avant de choisir
  gime_video_begin_frame(memory, w, h);
  for (int y = 0; y < h; y++)
  {
    gime_video_render_line(y, vga->dmaBuffer->getLineAddr8(y, vga->backBuffer));
    VidDrain();   // file de capture vidée au fil du rendu
  }
  const uint32_t rc = Perf_CCount() - rc0;
  g_vid_render_cc += rc;
  if (rc > g_vid_render_max) g_vid_render_max = rc;
  g_vid_frames++;
  sf.Coco2VideoGenMODE = 254;     // au retour en mode legacy : effacement forcé
}

void IRAM_ATTR VideoCore(void *pvParameters)
{
  uint8_t ModeValue;
  if (BootScreen_Wanted())   // écran d'accueil avant le BASIC (le CoCo est arrêté depuis setup)
  {
    vTaskDelay(pdMS_TO_TICKS(50));
    EnsureVGA320();
    BootScreen_Run();
    CPU_in_WAIT_STATE = false;
    g_resync_cpu_clock = true;
    sf.CPU_HALTED_BY_EMULATOR = false;
  }
  while (true)
  {
    gime_video_capture_drain();
    { const uint32_t d0_ = Perf_CCount(); vTaskDelay(1); g_vid_idle_cc += Perf_CCount() - d0_; }   // attente mesurée ; CORRECTIF
                     // complètement setup/loop/DoCPU sur le même cœur qui n'obtenaient donc jamais de
                     // temps CPU. Verr. Maj
    if (g_caps_save_ms && (uint32_t)((uint32_t)(esp_timer_get_time() / 1000) - g_caps_save_ms) > 2000u)
    {
      g_caps_save_ms = 0;
      SaveConfigToSD();
    }
    if (g_req_note)
    {
      g_req_note = false;
      CPU_in_WAIT_STATE = true;
      sf.CPU_HALTED_BY_EMULATOR = true;
      vTaskDelay(1);
      Note_Input();
      CPU_in_WAIT_STATE = false;
      sf.CPU_HALTED_BY_EMULATOR = false;
      g_resync_cpu_clock = true;
    }
    if (g_overlay_req)   // message en superposition (F9, capture, message de l'auteur)
    {
      g_overlay_req = false;
      CPU_in_WAIT_STATE = true;
      sf.CPU_HALTED_BY_EMULATOR = true;
      vTaskDelay(1);
      Overlay_Draw();
      vTaskDelay(pdMS_TO_TICKS(g_overlay_ms));
      CPU_in_WAIT_STATE = false;
      sf.CPU_HALTED_BY_EMULATOR = false;
      g_resync_cpu_clock = true;
    }
    if (Saver_Due(!sf.CPU_HALTED_BY_EMULATOR))   // écran de veille (rien touché depuis N minutes)
    {
      const bool was_wait = CPU_in_WAIT_STATE;
      CPU_in_WAIT_STATE = true;
      sf.CPU_HALTED_BY_EMULATOR = true;
      vTaskDelay(1);
      EnsureVGA320();
      Audio_SetPaused(true);
      Saver_Run();
      Audio_SetPaused(false);
      sf.Coco2VideoGenMODE = 254;                     // image du CoCo redessinée (modes texte / VDG)
      CPU_in_WAIT_STATE = was_wait;
      g_resync_cpu_clock = true;
      sf.CPU_HALTED_BY_EMULATOR = false;
      Saver_Touch();
    }
    if (sf.DIRECT_Key_Code == 0x43)   // F10 = aide (touches et options), émulation suspendue
    {
      extern void EMULATOR_Help(void);
      CPU_in_WAIT_STATE = true;
      sf.CPU_HALTED_BY_EMULATOR = true;
      vTaskDelay(1);
      EnsureVGA320();
      Audio_SetPaused(true);
      EMULATOR_Help();
      Audio_SetPaused(false);
      CPU_in_WAIT_STATE = false;
      sf.CPU_HALTED_BY_EMULATOR = false;
      vTaskDelay(200);
    }
    if (sf.DIRECT_Key_Code == 0x3E || sf.DIRECT_Key_Code == 0x41)   // F5 = sauvegarder, F8 = charger l'état
    {
      extern void State_HotkeyPublic(bool save);
      State_HotkeyPublic(sf.DIRECT_Key_Code == 0x3E);
    }
    if (sf.DIRECT_Key_Code == MENU_F12) // Emulator menu entrance
    {
      extern bool g_wait_before_menu;
      g_wait_before_menu = CPU_in_WAIT_STATE;   // SYNC/CWAI réel (le menu force l'attente)
      CPU_in_WAIT_STATE = true;
      sf.CPU_HALTED_BY_EMULATOR = true;
      vTaskDelay(1);

      EnsureVGA320();   // le menu est dessiné en 320x240
      Audio_SetPaused(true);    // le PSG (GMC) se tait pendant le menu
      g_rtc_step = STEP_MENU;
      g_menu_open = true;
      EMULATOR_Menu();
      g_menu_open = false;
      { extern void Menu_ToastClear(void); Menu_ToastClear(); }   // message du bas : effacé
      g_rtc_step = STEP_MENU_EXIT;
      Audio_SetPaused(false);
      CPU_in_WAIT_STATE = g_wait_before_menu;
      g_resync_cpu_clock = true;
      sf.CPU_HALTED_BY_EMULATOR = false;
      vTaskDelay(200);
      g_rtc_step = STEP_IDLE;
    }

    if (gime_video_active())
    {
      RenderGIMEFrame();
      ModeValue = 0xFF;     // aucun cas du switch : pas de rendu legacy
    }
    else
    {
      EnsureVGA320();
      ModeValue = LegacyRendererFor(sf.Coco2GraphicMode & 0b11110111);
      if ((sf.Coco2GraphicMode & 0xF0) == 0xF0)    // G6R (256 points), toutes hauteurs, artefacts NTSC
      {
        static const uint8_t lpr_tab[8] = { 3, 3, 3, 2, 2, 1, 1, 1 };
        Do_G6R(lpr_tab[sf.Coco2GraphicMode & 0x07]);
        ModeValue = 0xFF;
      }
    }

    switch (ModeValue)
    {

      case COCO2_GRAPHMODE_32X16_8X12:
        Do_COCO2_GRAPHMODE_32X16_8X12();
      break;

      case COCO2_GRAPHMODE_256X192X2:
        if (sf.Artefact)
        {
          Do_COCO2_GRAPHMODE_256X192X2_ARTEFACT();
        }
        else
        {
          Do_COCO2_GRAPHMODE_256X192X2();
        }
      break;

      case COCO2_GRAPHMODE_128X192X4:
        Do_COCO2_GRAPHMODE_128X192X4();
      break;

      case COCO2_GRAPHMODE_128X192X2:
        Do_COCO2_GRAPHMODE_128X192X2();
      break;

      case COCO2_GRAPHMODE_128X96X4:
      case COCO2_GRAPHMODE_128X96X4_1:
      Do_COCO2_GRAPHMODE_128X96X4();
      break;

      case COCO2_GRAPHMODE_128X96X2:
        Do_COCO2_GRAPHMODE_128X96X2();
      break;

      case COCO2_GRAPHMODE_128X64X4:
        Do_COCO2_GRAPHMODE_128X64X4();
      break;

      case COCO2_GRAPHMODE_128X64X2:
        Do_COCO2_GRAPHMODE_128X64X2();
    break;

    case COCO2_GRAPHMODE_64X64X4:
      Do_COCO2_GRAPHMODE_64X64X4();
break;

      default:
      break;
    }

    // while (!sf.V_Synch) //While no interrupt { //vTaskDelay(1); NOP; }


    WAIT_VSYNCH_LOW();

    vga->show();

    if (sf.PHYSICAL_Drive_Must_Be_Saved)
    {
      DISKETTE_LED_ON();
      sf.PHYSICAL_Drive_Must_Be_Saved = false;
      Disk_FlushAll();   // secteurs modifiés seulement
      DISKETTE_LED_OFF();
    }

    // LED disque des disques virtuels (DriveWire / VHD) éteinte 40 ms après le dernier accès (les
    // disquettes gèrent la leur).
    if (g_vd_led_off_at && esp_timer_get_time() > g_vd_led_off_at)
    {
      g_vd_led_off_at = 0;
      DISKETTE_LED_OFF();
    }

    FillKeyboardMatrix();
  }
}

void Do_COCO2_GRAPHMODE_64X64X4(void)
{

  if (sf.Coco2VideoGenMODE != COCO2_GRAPHMODE_64X64X4)
  {
    vga->clear(0b11111111);
    vga->show();
    vga->clear(0b11111111);
    vga->show();

    sf.Coco2VideoGenMODE = COCO2_GRAPHMODE_64X64X4;
  }

  uint32_t MemLoop = sf.Coco2VideoPageOffset_Registers * COCO2_GRAPH_OFFSET_LEN;
  uint8_t *vram = LegacyVRAM(MemLoop);  // VRAM figée au FS émulé
  uint16_t Xloop, Yloop, Yloop1;
  uint8_t Dot1bit[8];
  uint8_t Color[4];

  if ((sf.Coco2GraphicMode & 0b00001000) == 0)
  {
    Color[0] = 0b00011100;  // Green
    Color[1] = 0b11111100;  // Yellow
    Color[2] = 0b00000011;  // Blue
    Color[3] = 0b11100000;  // Red
  }
  else
  {
    Color[0] = 0xff; // blanc
    Color[1] = 0b01011011;  // Turquoise
    Color[2] = 0b11100011;  // lilas
    Color[3] = 0b11101000;  // orange(buff)
  }

  LegacyPal4(Color);   // palette GIME
  for (Yloop = 24; Yloop <216; Yloop+=3)
  {
        for (Xloop = 32; Xloop <288 ; Xloop+=16)
    {

      Dot1bit[3] = (vram[MemLoop] >> 6) & 0b11;  // Bits 6-7
      Dot1bit[2] = (vram[MemLoop] >> 4) & 0b11;  // Bits 4-5
      Dot1bit[1] = (vram[MemLoop] >> 2) & 0b11;  // Bits 2-3
      Dot1bit[0] = (vram[MemLoop++] >> 0) & 0b11;  // Bits 0-1

      // Line one
      vga->dot(Xloop,Yloop, Color[Dot1bit[3]]);
      vga->dot(Xloop+1,Yloop, Color[Dot1bit[3]]);
      vga->dot(Xloop+2,Yloop, Color[Dot1bit[3]]);
      vga->dot(Xloop+3,Yloop, Color[Dot1bit[3]]);

      vga->dot(Xloop+4,Yloop, Color[Dot1bit[2]]);
      vga->dot(Xloop+5,Yloop, Color[Dot1bit[2]]);
      vga->dot(Xloop+6,Yloop, Color[Dot1bit[2]]);
      vga->dot(Xloop+7,Yloop, Color[Dot1bit[2]]);

      vga->dot(Xloop+8,Yloop, Color[Dot1bit[1]]);
      vga->dot(Xloop+9,Yloop, Color[Dot1bit[1]]);
      vga->dot(Xloop+10,Yloop, Color[Dot1bit[1]]);
      vga->dot(Xloop+11,Yloop, Color[Dot1bit[1]]);

      vga->dot(Xloop+12,Yloop, Color[Dot1bit[0]]);
      vga->dot(Xloop+13,Yloop, Color[Dot1bit[0]]);
      vga->dot(Xloop+14,Yloop, Color[Dot1bit[0]]);
      vga->dot(Xloop+15,Yloop, Color[Dot1bit[0]]);

      Yloop1 = Yloop+1;
      // Line 2

      vga->dot(Xloop,Yloop1, Color[Dot1bit[3]]);
      vga->dot(Xloop+1,Yloop1, Color[Dot1bit[3]]);
      vga->dot(Xloop+2,Yloop1, Color[Dot1bit[3]]);
      vga->dot(Xloop+3,Yloop1, Color[Dot1bit[3]]);

      vga->dot(Xloop+4,Yloop1, Color[Dot1bit[2]]);
      vga->dot(Xloop+5,Yloop1, Color[Dot1bit[2]]);
      vga->dot(Xloop+6,Yloop1, Color[Dot1bit[2]]);
      vga->dot(Xloop+7,Yloop1, Color[Dot1bit[2]]);

      vga->dot(Xloop+8,Yloop1, Color[Dot1bit[1]]);
      vga->dot(Xloop+9,Yloop1, Color[Dot1bit[1]]);
      vga->dot(Xloop+10,Yloop1, Color[Dot1bit[1]]);
      vga->dot(Xloop+11,Yloop1, Color[Dot1bit[1]]);

      vga->dot(Xloop+12,Yloop1, Color[Dot1bit[0]]);
      vga->dot(Xloop+13,Yloop1, Color[Dot1bit[0]]);
      vga->dot(Xloop+14,Yloop1, Color[Dot1bit[0]]);
      vga->dot(Xloop+15,Yloop1, Color[Dot1bit[0]]);
      // Line 3
      Yloop1++;

      vga->dot(Xloop,Yloop1, Color[Dot1bit[3]]);
      vga->dot(Xloop+1,Yloop1, Color[Dot1bit[3]]);
      vga->dot(Xloop+2,Yloop1, Color[Dot1bit[3]]);
      vga->dot(Xloop+3,Yloop1, Color[Dot1bit[3]]);

      vga->dot(Xloop+4,Yloop1, Color[Dot1bit[2]]);
      vga->dot(Xloop+5,Yloop1, Color[Dot1bit[2]]);
      vga->dot(Xloop+6,Yloop1, Color[Dot1bit[2]]);
      vga->dot(Xloop+7,Yloop1, Color[Dot1bit[2]]);

      vga->dot(Xloop+8,Yloop1, Color[Dot1bit[1]]);
      vga->dot(Xloop+9,Yloop1, Color[Dot1bit[1]]);
      vga->dot(Xloop+10,Yloop1, Color[Dot1bit[1]]);
      vga->dot(Xloop+11,Yloop1, Color[Dot1bit[1]]);

      vga->dot(Xloop+12,Yloop1, Color[Dot1bit[0]]);
      vga->dot(Xloop+13,Yloop1, Color[Dot1bit[0]]);
      vga->dot(Xloop+14,Yloop1, Color[Dot1bit[0]]);
      vga->dot(Xloop+15,Yloop1, Color[Dot1bit[0]]);

    }
  }

}

void Do_COCO2_GRAPHMODE_128X64X2(void)
{

  if (sf.Coco2VideoGenMODE != COCO2_GRAPHMODE_128X64X2)
  {
    vga->clear(0b11111111);
    vga->show();
    vga->clear(0b11111111);
    vga->show();

    sf.Coco2VideoGenMODE = COCO2_GRAPHMODE_128X64X2;
  }

  uint32_t MemLoop = sf.Coco2VideoPageOffset_Registers * COCO2_GRAPH_OFFSET_LEN;
  uint8_t *vram = LegacyVRAM(MemLoop);  // VRAM figée au FS émulé
  uint16_t Xloop, Yloop, Yloop1;
  uint8_t Dot1bit[8];
  uint8_t Color[2];

  if ((sf.Coco2GraphicMode & 0b00001000) == 0)
  {
    Color[0] = 0b00000000;  // black
    Color[1] = 0b00011100; // green
  }
  else
  {
    Color[0] = 0x00;
    Color[1] = 0xff;
  }

  LegacyPal2(Color);   // palette GIME
  for (Yloop = 24; Yloop <216; Yloop+=3)
  {
    for (Xloop = 32; Xloop <288 ; Xloop+=16)
    {
      Dot1bit[7] = ((vram[MemLoop]>>7) & 0b00000001);
      Dot1bit[6] = ((vram[MemLoop]>>6) & 0b00000001);
      Dot1bit[5] = ((vram[MemLoop]>>5) & 0b00000001);
      Dot1bit[4] = ((vram[MemLoop]>>4) & 0b00000001);
      Dot1bit[3] = ((vram[MemLoop]>>3) & 0b00000001);
      Dot1bit[2] = ((vram[MemLoop]>>2) & 0b00000001);
      Dot1bit[1] = ((vram[MemLoop]>>1) & 0b00000001);
      Dot1bit[0] = (vram[MemLoop++] & 0b00000001);

      vga->dot(Xloop,Yloop, Color[Dot1bit[7]]);
      vga->dot(Xloop+1,Yloop, Color[Dot1bit[7]]);

      vga->dot(Xloop+2,Yloop, Color[Dot1bit[6]]);
      vga->dot(Xloop+3,Yloop, Color[Dot1bit[6]]);

      vga->dot(Xloop+4,Yloop, Color[Dot1bit[5]]);
      vga->dot(Xloop+5,Yloop, Color[Dot1bit[5]]);

      vga->dot(Xloop+6,Yloop, Color[Dot1bit[4]]);
      vga->dot(Xloop+7,Yloop, Color[Dot1bit[4]]);

      vga->dot(Xloop+8,Yloop, Color[Dot1bit[3]]);
      vga->dot(Xloop+9,Yloop, Color[Dot1bit[3]]);

      vga->dot(Xloop+10,Yloop, Color[Dot1bit[2]]);
      vga->dot(Xloop+11,Yloop, Color[Dot1bit[2]]);

      vga->dot(Xloop+12,Yloop, Color[Dot1bit[1]]);
      vga->dot(Xloop+13,Yloop, Color[Dot1bit[1]]);

      vga->dot(Xloop+14,Yloop, Color[Dot1bit[0]]);
      vga->dot(Xloop+15,Yloop, Color[Dot1bit[0]]);

      //------------------- Second Line
      Yloop1 = Yloop+1;
      vga->dot(Xloop,Yloop1, Color[Dot1bit[7]]);
      vga->dot(Xloop+1,Yloop1, Color[Dot1bit[7]]);

      vga->dot(Xloop+2,Yloop1, Color[Dot1bit[6]]);
      vga->dot(Xloop+3,Yloop1, Color[Dot1bit[6]]);

      vga->dot(Xloop+4,Yloop1, Color[Dot1bit[5]]);
      vga->dot(Xloop+5,Yloop1, Color[Dot1bit[5]]);

      vga->dot(Xloop+6,Yloop1, Color[Dot1bit[4]]);
      vga->dot(Xloop+7,Yloop1, Color[Dot1bit[4]]);

      vga->dot(Xloop+8,Yloop1, Color[Dot1bit[3]]);
      vga->dot(Xloop+9,Yloop1, Color[Dot1bit[3]]);

      vga->dot(Xloop+10,Yloop1, Color[Dot1bit[2]]);
      vga->dot(Xloop+11,Yloop1, Color[Dot1bit[2]]);

      vga->dot(Xloop+12,Yloop1, Color[Dot1bit[1]]);
      vga->dot(Xloop+13,Yloop1, Color[Dot1bit[1]]);

      vga->dot(Xloop+14,Yloop1, Color[Dot1bit[0]]);
      vga->dot(Xloop+15,Yloop1, Color[Dot1bit[0]]);
      //---- Line 3

      Yloop1++;
      vga->dot(Xloop,Yloop1, Color[Dot1bit[7]]);
      vga->dot(Xloop+1,Yloop1, Color[Dot1bit[7]]);

      vga->dot(Xloop+2,Yloop1, Color[Dot1bit[6]]);
      vga->dot(Xloop+3,Yloop1, Color[Dot1bit[6]]);

      vga->dot(Xloop+4,Yloop1, Color[Dot1bit[5]]);
      vga->dot(Xloop+5,Yloop1, Color[Dot1bit[5]]);

      vga->dot(Xloop+6,Yloop1, Color[Dot1bit[4]]);
      vga->dot(Xloop+7,Yloop1, Color[Dot1bit[4]]);

      vga->dot(Xloop+8,Yloop1, Color[Dot1bit[3]]);
      vga->dot(Xloop+9,Yloop1, Color[Dot1bit[3]]);

      vga->dot(Xloop+10,Yloop1, Color[Dot1bit[2]]);
      vga->dot(Xloop+11,Yloop1, Color[Dot1bit[2]]);

      vga->dot(Xloop+12,Yloop1, Color[Dot1bit[1]]);
      vga->dot(Xloop+13,Yloop1, Color[Dot1bit[1]]);

      vga->dot(Xloop+14,Yloop1, Color[Dot1bit[0]]);
      vga->dot(Xloop+15,Yloop1, Color[Dot1bit[0]]);

    }
  }
}

void Do_COCO2_GRAPHMODE_128X64X4(void)
{

  if (sf.Coco2VideoGenMODE != COCO2_GRAPHMODE_128X64X4)
  {
    vga->clear(0b11111111);
    vga->show();
    vga->clear(0b11111111);
    vga->show();

    sf.Coco2VideoGenMODE = COCO2_GRAPHMODE_128X64X4;
  }

  uint32_t MemLoop = sf.Coco2VideoPageOffset_Registers * COCO2_GRAPH_OFFSET_LEN;
  uint8_t *vram = LegacyVRAM(MemLoop);  // VRAM figée au FS émulé
  uint16_t Xloop, Yloop, Yloop1;
  uint8_t Dot1bit[8];
  uint8_t Color[4];

  if ((sf.Coco2GraphicMode & 0b00001000) == 0)
  {
    Color[0] = 0b00011100;  // Green
    Color[1] = 0b11111100;  // Yellow
    Color[2] = 0b00000011;  // Blue
    Color[3] = 0b11100000;  // Red
  }
  else
  {
    Color[0] = 0xff; // blanc
    Color[1] = 0b01011011;  // Turquoise
    Color[2] = 0b11100011;  // lilas
    Color[3] = 0b11101000;  // orange(buff)
  }

  LegacyPal4(Color);   // palette GIME
  for (Yloop = 24; Yloop <216; Yloop+=3)
  {
    for (Xloop = 32; Xloop <288 ; Xloop+=8)
    {

      Dot1bit[3] = (vram[MemLoop] >> 6) & 0b11;  // Bits 6-7
      Dot1bit[2] = (vram[MemLoop] >> 4) & 0b11;  // Bits 4-5
      Dot1bit[1] = (vram[MemLoop] >> 2) & 0b11;  // Bits 2-3
      Dot1bit[0] = (vram[MemLoop++] >> 0) & 0b11;  // Bits 0-1

      // Line one
      vga->dot(Xloop,Yloop, Color[Dot1bit[3]]);
      vga->dot(Xloop+1,Yloop, Color[Dot1bit[3]]);

      vga->dot(Xloop+2,Yloop, Color[Dot1bit[2]]);
      vga->dot(Xloop+3,Yloop, Color[Dot1bit[2]]);

      vga->dot(Xloop+4,Yloop, Color[Dot1bit[1]]);
      vga->dot(Xloop+5,Yloop, Color[Dot1bit[1]]);

      vga->dot(Xloop+6,Yloop, Color[Dot1bit[0]]);
      vga->dot(Xloop+7,Yloop, Color[Dot1bit[0]]);

      Yloop1 = Yloop+1;
      // Line 2

      vga->dot(Xloop,Yloop1, Color[Dot1bit[3]]);
      vga->dot(Xloop+1,Yloop1, Color[Dot1bit[3]]);

      vga->dot(Xloop+2,Yloop1, Color[Dot1bit[2]]);
      vga->dot(Xloop+3,Yloop1, Color[Dot1bit[2]]);

      vga->dot(Xloop+4,Yloop1, Color[Dot1bit[1]]);
      vga->dot(Xloop+5,Yloop1, Color[Dot1bit[1]]);

      vga->dot(Xloop+6,Yloop1, Color[Dot1bit[0]]);
      vga->dot(Xloop+7,Yloop1, Color[Dot1bit[0]]);
      Yloop1++;
      // Line 3

      vga->dot(Xloop,Yloop1, Color[Dot1bit[3]]);
      vga->dot(Xloop+1,Yloop1, Color[Dot1bit[3]]);

      vga->dot(Xloop+2,Yloop1, Color[Dot1bit[2]]);
      vga->dot(Xloop+3,Yloop1, Color[Dot1bit[2]]);

      vga->dot(Xloop+4,Yloop1, Color[Dot1bit[1]]);
      vga->dot(Xloop+5,Yloop1, Color[Dot1bit[1]]);

      vga->dot(Xloop+6,Yloop1, Color[Dot1bit[0]]);
      vga->dot(Xloop+7,Yloop1, Color[Dot1bit[0]]);

    }
  }

}

void Do_COCO2_GRAPHMODE_128X96X2(void)
{

  if (sf.Coco2VideoGenMODE != COCO2_GRAPHMODE_128X96X2)
  {
    vga->clear(0b11111111);
    vga->show();
    vga->clear(0b11111111);
    vga->show();

    sf.Coco2VideoGenMODE = COCO2_GRAPHMODE_128X96X2;
  }

  uint32_t MemLoop = sf.Coco2VideoPageOffset_Registers * COCO2_GRAPH_OFFSET_LEN;
  uint8_t *vram = LegacyVRAM(MemLoop);  // VRAM figée au FS émulé
  uint16_t Xloop, Yloop, Yloop1;
  uint8_t Dot1bit[8];
  uint8_t Color[2];

  if ((sf.Coco2GraphicMode & 0b00001000) == 0)
  {
    Color[0] = 0b00000000;  // black
    Color[1] = 0b00011100; // green
  }
  else
  {
    Color[0] = 0x00;
    Color[1] = 0xff;
  }

  LegacyPal2(Color);   // palette GIME
  for (Yloop = 24; Yloop <216; Yloop+=2)
  {
        for (Xloop = 32; Xloop <288 ; Xloop+=16)
    {
      Dot1bit[7] = ((vram[MemLoop]>>7) & 0b00000001);
      Dot1bit[6] = ((vram[MemLoop]>>6) & 0b00000001);
      Dot1bit[5] = ((vram[MemLoop]>>5) & 0b00000001);
      Dot1bit[4] = ((vram[MemLoop]>>4) & 0b00000001);
      Dot1bit[3] = ((vram[MemLoop]>>3) & 0b00000001);
      Dot1bit[2] = ((vram[MemLoop]>>2) & 0b00000001);
      Dot1bit[1] = ((vram[MemLoop]>>1) & 0b00000001);
      Dot1bit[0] = (vram[MemLoop++] & 0b00000001);

      vga->dot(Xloop,Yloop, Color[Dot1bit[7]]);
      vga->dot(Xloop+1,Yloop, Color[Dot1bit[7]]);

      vga->dot(Xloop+2,Yloop, Color[Dot1bit[6]]);
      vga->dot(Xloop+3,Yloop, Color[Dot1bit[6]]);

      vga->dot(Xloop+4,Yloop, Color[Dot1bit[5]]);
      vga->dot(Xloop+5,Yloop, Color[Dot1bit[5]]);

      vga->dot(Xloop+6,Yloop, Color[Dot1bit[4]]);
      vga->dot(Xloop+7,Yloop, Color[Dot1bit[4]]);

      vga->dot(Xloop+8,Yloop, Color[Dot1bit[3]]);
      vga->dot(Xloop+9,Yloop, Color[Dot1bit[3]]);

      vga->dot(Xloop+10,Yloop, Color[Dot1bit[2]]);
      vga->dot(Xloop+11,Yloop, Color[Dot1bit[2]]);

      vga->dot(Xloop+12,Yloop, Color[Dot1bit[1]]);
      vga->dot(Xloop+13,Yloop, Color[Dot1bit[1]]);

      vga->dot(Xloop+14,Yloop, Color[Dot1bit[0]]);
      vga->dot(Xloop+15,Yloop, Color[Dot1bit[0]]);

      //------------------- Second Line
      Yloop1 = Yloop+1;
      vga->dot(Xloop,Yloop1, Color[Dot1bit[7]]);
      vga->dot(Xloop+1,Yloop1, Color[Dot1bit[7]]);

      vga->dot(Xloop+2,Yloop1, Color[Dot1bit[6]]);
      vga->dot(Xloop+3,Yloop1, Color[Dot1bit[6]]);

      vga->dot(Xloop+4,Yloop1, Color[Dot1bit[5]]);
      vga->dot(Xloop+5,Yloop1, Color[Dot1bit[5]]);

      vga->dot(Xloop+6,Yloop1, Color[Dot1bit[4]]);
      vga->dot(Xloop+7,Yloop1, Color[Dot1bit[4]]);

      vga->dot(Xloop+8,Yloop1, Color[Dot1bit[3]]);
      vga->dot(Xloop+9,Yloop1, Color[Dot1bit[3]]);

      vga->dot(Xloop+10,Yloop1, Color[Dot1bit[2]]);
      vga->dot(Xloop+11,Yloop1, Color[Dot1bit[2]]);

      vga->dot(Xloop+12,Yloop1, Color[Dot1bit[1]]);
      vga->dot(Xloop+13,Yloop1, Color[Dot1bit[1]]);

      vga->dot(Xloop+14,Yloop1, Color[Dot1bit[0]]);
      vga->dot(Xloop+15,Yloop1, Color[Dot1bit[0]]);

    }
  }

}

void Do_COCO2_GRAPHMODE_128X96X4(void)
{

  if (sf.Coco2VideoGenMODE != COCO2_GRAPHMODE_128X96X4)
  {
    vga->clear(0b11111111);
    vga->show();
    vga->clear(0b11111111);
    vga->show();

    sf.Coco2VideoGenMODE = COCO2_GRAPHMODE_128X96X4;
  }

  uint32_t MemLoop = sf.Coco2VideoPageOffset_Registers * COCO2_GRAPH_OFFSET_LEN;
  uint8_t *vram = LegacyVRAM(MemLoop);  // VRAM figée au FS émulé
  uint16_t Xloop, Yloop;
  uint8_t Dot1bit[8];
  uint8_t Color[4];

  if ((sf.Coco2GraphicMode & 0b00001000) == 0)
  {
    Color[0] = 0b00011100;  // Green
    Color[1] = 0b11111100;  // Yellow
    Color[2] = 0b00000011;  // Blue
    Color[3] = 0b11100000;  // Red
  }
  else
  {
    Color[0] = 0b00000000;  // Black
    Color[2] = 0b11100000;  // Red
    Color[1] = 0b00000011;  // blue
    Color[3] = 0b11111111;  // White
    // Color[0] = 0xff; //blanc Color[1] = 0b01011011; //Turquoise Color[2] = 0b11100011; //lilas
    // Color[3] = 0b11101000; //orange(buff)
  }

  if ((sf.Coco2GraphicMode & 0x08) == 0) LegacyPal4(Color);
  for (Yloop = 24; Yloop <216; Yloop+=2)
  {
    for (Xloop = 32; Xloop <288 ; Xloop+=8)
    {

      Dot1bit[3] = (vram[MemLoop] >> 6) & 0b11;  // Bits 6-7
      Dot1bit[2] = (vram[MemLoop] >> 4) & 0b11;  // Bits 4-5
      Dot1bit[1] = (vram[MemLoop] >> 2) & 0b11;  // Bits 2-3
      Dot1bit[0] = (vram[MemLoop++] >> 0) & 0b11;  // Bits 0-1

      // Line one
      vga->dot(Xloop,Yloop, Color[Dot1bit[3]]);
      vga->dot(Xloop+1,Yloop, Color[Dot1bit[3]]);

      vga->dot(Xloop+2,Yloop, Color[Dot1bit[2]]);
      vga->dot(Xloop+3,Yloop, Color[Dot1bit[2]]);

      vga->dot(Xloop+4,Yloop, Color[Dot1bit[1]]);
      vga->dot(Xloop+5,Yloop, Color[Dot1bit[1]]);

      vga->dot(Xloop+6,Yloop, Color[Dot1bit[0]]);
      vga->dot(Xloop+7,Yloop, Color[Dot1bit[0]]);

      // Line 2

      vga->dot(Xloop,Yloop+1, Color[Dot1bit[3]]);
      vga->dot(Xloop+1,Yloop+1, Color[Dot1bit[3]]);

      vga->dot(Xloop+2,Yloop+1, Color[Dot1bit[2]]);
      vga->dot(Xloop+3,Yloop+1, Color[Dot1bit[2]]);

      vga->dot(Xloop+4,Yloop+1, Color[Dot1bit[1]]);
      vga->dot(Xloop+5,Yloop+1, Color[Dot1bit[1]]);

      vga->dot(Xloop+6,Yloop+1, Color[Dot1bit[0]]);
      vga->dot(Xloop+7,Yloop+1, Color[Dot1bit[0]]);

    }
  }

}

void Do_COCO2_GRAPHMODE_128X192X2(void)
{

  if (sf.Coco2VideoGenMODE != COCO2_GRAPHMODE_128X192X2)
  {
    vga->clear(0b11111111);
    vga->show();
    vga->clear(0b11111111);
    vga->show();

    sf.Coco2VideoGenMODE = COCO2_GRAPHMODE_128X192X2;
  }

  uint32_t MemLoop = sf.Coco2VideoPageOffset_Registers * COCO2_GRAPH_OFFSET_LEN;
  uint8_t *vram = LegacyVRAM(MemLoop);  // VRAM figée au FS émulé
  uint16_t Xloop, Yloop;
  uint8_t Dot1bit[8];
  uint8_t Color[2];

  if ((sf.Coco2GraphicMode & 0b00001000) == 0)
  {
    Color[0] = 0b00000000;  // black
    Color[1] = 0b00011100; // green
  }
  else
  {
    Color[0] = 0x00;
    Color[1] = 0xff;
  }

  LegacyPal2(Color);   // palette GIME
  for (Yloop = 24; Yloop <216; Yloop++)
  {
    for (Xloop = (sf.VideoEmulatorXpixels-256)>>1; Xloop <sf.VideoEmulatorXpixels - ((sf.VideoEmulatorXpixels-256)>>1) ; Xloop+=16)
    {
      Dot1bit[7] = ((vram[MemLoop]>>7) & 0b00000001);
      Dot1bit[6] = ((vram[MemLoop]>>6) & 0b00000001);
      Dot1bit[5] = ((vram[MemLoop]>>5) & 0b00000001);
      Dot1bit[4] = ((vram[MemLoop]>>4) & 0b00000001);
      Dot1bit[3] = ((vram[MemLoop]>>3) & 0b00000001);
      Dot1bit[2] = ((vram[MemLoop]>>2) & 0b00000001);
      Dot1bit[1] = ((vram[MemLoop]>>1) & 0b00000001);
      Dot1bit[0] = (vram[MemLoop++] & 0b00000001);

      vga->dot(Xloop,Yloop, Color[Dot1bit[7]]);
      vga->dot(Xloop+1,Yloop, Color[Dot1bit[7]]);

      vga->dot(Xloop+2,Yloop, Color[Dot1bit[6]]);
      vga->dot(Xloop+3,Yloop, Color[Dot1bit[6]]);

      vga->dot(Xloop+4,Yloop, Color[Dot1bit[5]]);
      vga->dot(Xloop+5,Yloop, Color[Dot1bit[5]]);

      vga->dot(Xloop+6,Yloop, Color[Dot1bit[4]]);
      vga->dot(Xloop+7,Yloop, Color[Dot1bit[4]]);

      vga->dot(Xloop+8,Yloop, Color[Dot1bit[3]]);
      vga->dot(Xloop+9,Yloop, Color[Dot1bit[3]]);

      vga->dot(Xloop+10,Yloop, Color[Dot1bit[2]]);
      vga->dot(Xloop+11,Yloop, Color[Dot1bit[2]]);

      vga->dot(Xloop+12,Yloop, Color[Dot1bit[1]]);
      vga->dot(Xloop+13,Yloop, Color[Dot1bit[1]]);

      vga->dot(Xloop+14,Yloop, Color[Dot1bit[0]]);
      vga->dot(Xloop+15,Yloop, Color[Dot1bit[0]]);

    }
  }
}

void Do_COCO2_GRAPHMODE_128X192X4(void)
{

  if (sf.Coco2VideoGenMODE != COCO2_GRAPHMODE_128X192X4)
  {
    vga->clear(0b11111111);
    vga->show();
    vga->clear(0b11111111);
    vga->show();

    sf.Coco2VideoGenMODE = COCO2_GRAPHMODE_128X192X4;
    LogOut.println("MODE");
  }

  uint32_t MemLoop = sf.Coco2VideoPageOffset_Registers * COCO2_GRAPH_OFFSET_LEN;
  uint8_t *vram = LegacyVRAM(MemLoop);  // VRAM figée au FS émulé
  uint16_t Xloop, Yloop;
  uint8_t Dot1bit[8];
  uint8_t Color[4];

  if ((sf.Coco2GraphicMode & 0b00001000) == 0)
  {
    Color[0] = 0b00011100;  // Green
    Color[1] = 0b11111100;  // Yellow
    Color[2] = 0b00000011;  // Blue
    Color[3] = 0b11100000;  // Red
  }
  else
  {
    Color[0] = 0xff; // blanc
    Color[1] = 0b01011011;  // Turquoise
    Color[2] = 0b11100011;  // lilas
    Color[3] = 0b11101000;  // orange(buff)
  }

  LegacyPal4(Color);   // palette GIME
  for (Yloop = 24; Yloop <216; Yloop++)
  {
    for (Xloop = 32; Xloop < 288 ; Xloop+=8)
    {
      Dot1bit[3] = (vram[MemLoop] >> 6) & 0b11;  // Bits 6-7
      Dot1bit[2] = (vram[MemLoop] >> 4) & 0b11;  // Bits 4-5
      Dot1bit[1] = (vram[MemLoop] >> 2) & 0b11;  // Bits 2-3
      Dot1bit[0] = (vram[MemLoop++] >> 0) & 0b11;  // Bits 0-1

      vga->dot(Xloop,Yloop, Color[Dot1bit[3]]);
      vga->dot(Xloop+1,Yloop, Color[Dot1bit[3]]);

      vga->dot(Xloop+2,Yloop, Color[Dot1bit[2]]);
      vga->dot(Xloop+3,Yloop, Color[Dot1bit[2]]);

      vga->dot(Xloop+4,Yloop, Color[Dot1bit[1]]);
      vga->dot(Xloop+5,Yloop, Color[Dot1bit[1]]);

      vga->dot(Xloop+6,Yloop, Color[Dot1bit[0]]);
      vga->dot(Xloop+7,Yloop, Color[Dot1bit[0]]);

    }
  }

}

// ---------------------------------------------------------------------------
// mode G6R du VDG (256 points, 1 bit par point) quel que soit le nombre de lignes par rangée (SAM V)
// ---------------------------------------------------------------------------
static uint8_t Vga_FromRGB(uint8_t r, uint8_t g, uint8_t b)
{
  const uint8_t R = (uint8_t)((r + 42) / 85), G = (uint8_t)((g + 42) / 85), B = (uint8_t)((b + 42) / 85);   // 0..3
  return (uint8_t)((R << 6) | ((R >> 1) << 5) | (G << 3) | ((G >> 1) << 2) | B);
}
static void Vga_ToRGB(uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b)
{
  r = (uint8_t)(((v >> 6) & 3) * 85); g = (uint8_t)(((v >> 3) & 3) * 85); b = (uint8_t)((v & 3) * 85);
}
static uint8_t s_art_tab[128];
static void Art_Build(uint8_t c0, uint8_t c1, bool swap)
{
  static int16_t s_k = -1;
  const int16_t k = (int16_t)((c0 << 8) ^ c1 ^ (swap ? 0x4000 : 0));
  if (k == s_k) return;
  s_k = k;
  static const float f[14 * 3] = {
    0.157f, 0.000f, 0.157f,  0.000f, 0.157f, 0.000f,  1.000f, 0.824f, 1.000f,  0.824f, 1.000f, 0.824f,
    0.706f, 0.236f, 0.118f,  0.000f, 0.197f, 0.471f,  1.000f, 0.550f, 0.393f,  0.275f, 0.785f, 1.000f,
    0.000f, 0.500f, 1.000f,  1.000f, 0.500f, 0.000f,  1.000f, 0.942f, 0.785f,  0.393f, 0.942f, 1.000f,
    0.236f, 0.000f, 0.000f,  0.000f, 0.000f, 0.236f };
  static const uint8_t corr[128] = {
    0,0, 0,0, 0,6, 0,2,  5,7, 5,7, 1,3, 1,11,  8,6, 8,14, 8,9, 8,9,  4,4, 4,15, 12,12, 12,15,
    5,13, 5,13, 13,0, 13,2,  10,10, 10,10, 10,15, 10,11,  3,1, 3,1, 15,9, 15,9,  11,11, 11,11, 15,15, 15,15,
    14,0, 14,0, 14,6, 14,2,  0,7, 0,7, 1,3, 1,11,  9,6, 9,14, 9,9, 9,9,  15,4, 15,15, 12,12, 12,15,
    2,13, 2,13, 2,0, 2,2,  10,10, 10,10, 10,15, 10,11,  12,1, 12,1, 12,9, 12,9,  15,11, 15,11, 15,15, 15,15 };
  uint8_t r0, g0, b0, r1, g1, b1;
  Vga_ToRGB(c0, r0, g0, b0); Vga_ToRGB(c1, r1, g1, b1);
  uint8_t col[16];
  col[0] = c0; col[15] = c1;
  for (int i = 1; i <= 14; i++)
  {
    const float *ff = &f[((i - 1) ^ (swap ? 1 : 0)) * 3];
    col[i] = Vga_FromRGB((uint8_t)(r0 * (1.0f - ff[0]) + r1 * ff[0] + 0.5f),
                         (uint8_t)(g0 * (1.0f - ff[1]) + g1 * ff[1] + 0.5f),
                         (uint8_t)(b0 * (1.0f - ff[2]) + b1 * ff[2] + 0.5f));
  }
  for (int i = 0; i < 128; i++) s_art_tab[i] = col[corr[i]];
}
#define COCO2_GRAPHMODE_G6R_ANY 0xF7
void Do_G6R(uint8_t lpr)
{
  if (sf.Coco2VideoGenMODE != COCO2_GRAPHMODE_G6R_ANY)
  {
    for (int pass = 0; pass < 2; pass++) { vga->clear(0b11111111); vga->show(); }
    sf.Coco2VideoGenMODE = COCO2_GRAPHMODE_G6R_ANY;
  }
  uint32_t MemLoop = sf.Coco2VideoPageOffset_Registers * COCO2_GRAPH_OFFSET_LEN;
  const uint8_t *vram = LegacyVRAM(MemLoop);
  uint8_t c2[2];
  LegacyPal2(c2);                                    // fond / point (palette GIME, CSS)
  const bool art = sf.Artefact;
  if (art) Art_Build(c2[0], c2[1], g_artifact_swap == 0);   // phase par défaut = celle de MAME (CoCo 3) ; menu A : inversée
  const uint16_t rows = (uint16_t)(192 / lpr);
  uint8_t line[256];
  uint8_t bits[256 + 6];                             // 2 points de garde de chaque côté
  for (uint16_t row = 0; row < rows; row++)
  {
    const uint8_t *src = vram + MemLoop + (uint32_t)row * 32u;
    if (!art)
    {
      for (int x = 0; x < 256; x++) line[x] = c2[(src[x >> 3] >> (7 - (x & 7))) & 1];
    }
    else
    {
      // points de garde = bordure. En mode graphique 2 couleurs, la bordure du VDG a la couleur du
      // point allumé (vert ou blanc selon CSS) et MAME calcule les artefacts sur toute la ligne,
      // bordure comprise
      bits[0] = bits[1] = 1;
      for (int x = 0; x < 256; x++) bits[x + 2] = (src[x >> 3] >> (7 - (x & 7))) & 1;
      bits[258] = bits[259] = bits[260] = bits[261] = 1;
      for (int x = 0; x < 256; x += 2)
      {
        const uint8_t *b = &bits[x];                 // b[0] = point x-2 ... b[5] = point x+3
        const uint8_t val = (uint8_t)((b[0] << 5) | (b[1] << 4) | (b[2] << 3) | (b[3] << 2) | (b[4] << 1) | b[5]);
        line[x] = s_art_tab[val * 2];
        line[x + 1] = s_art_tab[val * 2 + 1];
      }
    }
    for (uint8_t k = 0; k < lpr; k++) vga->drawLineFromMemory256(32, 24 + row * lpr + k, line);
  }
}

void Do_COCO2_GRAPHMODE_256X192X2(void)
{
  if (sf.Coco2VideoGenMODE != COCO2_GRAPHMODE_256X192X2)
  {
    vga->clear(0b11111111);
    vga->show();
    vga->clear(0b11111111);
    vga->show();
    sf.Coco2VideoGenMODE = COCO2_GRAPHMODE_256X192X2;
  }

  uint32_t MemLoop = (sf.Coco2VideoPageOffset_Registers * COCO2_GRAPH_OFFSET_LEN);
  uint8_t *vram = LegacyVRAM(MemLoop);  // VRAM figée au FS émulé
  uint16_t Xloop, Yloop;
  uint8_t Dot1bit[16];
  uint8_t Color[2];

  if ((sf.Coco2GraphicMode & 0b00001000) == 0)
  {
    Color[0] = 0b00000000;  // black
    Color[1] = 0b00011100; // green
  }
  else
  {
    Color[0] = 0x00;
    Color[1] = 0xff;
  }

  // Correctif 25-09 : partait de 23 -> 193 lignes au lieu de 192 : image décalée d'une ligne vers le
  // haut et 32 octets lus APRÈS la page graphique de 6 Ko (ligne de garbage en bas)
  LegacyPal2(Color);   // palette GIME
  for (Yloop = 24; Yloop <216; Yloop++)

  {
    for (Xloop = 32; Xloop <288 ; Xloop+=16)
    {

      Dot1bit[0] = ((vram[MemLoop]>>7) & 0b00000001);
      Dot1bit[1] = ((vram[MemLoop]>>6) & 0b00000001);
      Dot1bit[2] = ((vram[MemLoop]>>5) & 0b00000001);
      Dot1bit[3] = ((vram[MemLoop]>>4) & 0b00000001);
      Dot1bit[4] = ((vram[MemLoop]>>3) & 0b00000001);
      Dot1bit[5] = ((vram[MemLoop]>>2) & 0b00000001);
      Dot1bit[6] = ((vram[MemLoop]>>1) & 0b00000001);
      Dot1bit[7] = (vram[MemLoop++] & 0b00000001);

      Dot1bit[7] = Color[Dot1bit[7]];
      Dot1bit[6] = Color[Dot1bit[6]];
      Dot1bit[5] = Color[Dot1bit[5]];
      Dot1bit[4] = Color[Dot1bit[4]];
      Dot1bit[3] = Color[Dot1bit[3]];
      Dot1bit[2] = Color[Dot1bit[2]];
      Dot1bit[1] = Color[Dot1bit[1]];
      Dot1bit[0] = Color[Dot1bit[0]];

      Dot1bit[8] = ((vram[MemLoop]>>7) & 0b00000001);
      Dot1bit[9] = ((vram[MemLoop]>>6) & 0b00000001);
      Dot1bit[10] = ((vram[MemLoop]>>5) & 0b00000001);
      Dot1bit[11] = ((vram[MemLoop]>>4) & 0b00000001);
      Dot1bit[12] = ((vram[MemLoop]>>3) & 0b00000001);
      Dot1bit[13] = ((vram[MemLoop]>>2) & 0b00000001);
      Dot1bit[14] = ((vram[MemLoop]>>1) & 0b00000001);
      Dot1bit[15] = (vram[MemLoop++] & 0b00000001);

      Dot1bit[15] = Color[Dot1bit[15]];
      Dot1bit[14] = Color[Dot1bit[14]];
      Dot1bit[13] = Color[Dot1bit[13]];
      Dot1bit[12] = Color[Dot1bit[12]];
      Dot1bit[11] = Color[Dot1bit[11]];
      Dot1bit[10] = Color[Dot1bit[10]];
      Dot1bit[9] = Color[Dot1bit[9]];
      Dot1bit[8] = Color[Dot1bit[8]];

      vga->drawLineFromMemory16(Xloop, Yloop,&Dot1bit[0]);

    }
  }
}

void IRAM_ATTR Do_COCO2_GRAPHMODE_256X192X2_ARTEFACT(void)
{
  if (sf.Coco2VideoGenMODE != COCO2_GRAPHMODE_256X192X2)
  {
    vga->clear(0b11111111);
    vga->show();
    vga->clear(0b11111111);
    vga->show();
    sf.Coco2VideoGenMODE = COCO2_GRAPHMODE_256X192X2;

  }
  uint32_t MemLoop = sf.Coco2VideoPageOffset_Registers * COCO2_GRAPH_OFFSET_LEN;
  uint8_t *vram = LegacyVRAM(MemLoop);  // VRAM figée au FS émulé
  uint16_t Xloop, Yloop;
  uint8_t Dot1bit[8];
  uint8_t Color[4];
  uint16_t DotCount = 0;
  uint16_t Xcount;
  uint8_t BitCount;
  bool NoBlackFlag = false;
  Color[0] = 0b00000000;  // Black
  Color[2] = 0b11100000;  // Red
  Color[1] = 0b00000011;  // blue
  Color[3] = 0b11111111;  // White
  if (g_artifact_swap) { Color[1] = 0b11100000; Color[2] = 0b00000011; }   // phase inversée (bleu/rouge)
  // Correctif 25-09 : au 1er pixel de chaque ligne, le code écrit LinePrep[-1] (pixel noir ou 2e point
  // allumé de suite) : écriture hors tableau sur la pile
  uint8_t LinePrepBuf[1 + 266];
  uint8_t *LinePrep = LinePrepBuf + 1;
  uint16_t LinePrepLoop;
  for (Yloop = 24; Yloop <216; Yloop++)
  {
    Xcount = 0;
    DotCount = 0;
    LinePrepLoop = 0;

    for (Xloop = 32; Xloop <288 ; Xloop+=8)
    {
      Dot1bit[0] = ((vram[MemLoop]>>7) & 0b00000001);
      Dot1bit[1] = ((vram[MemLoop]>>6) & 0b00000001);
      Dot1bit[2] = ((vram[MemLoop]>>5) & 0b00000001);
      Dot1bit[3] = ((vram[MemLoop]>>4) & 0b00000001);
      Dot1bit[4] = ((vram[MemLoop]>>3) & 0b00000001);
      Dot1bit[5] = ((vram[MemLoop]>>2) & 0b00000001);
      Dot1bit[6] = ((vram[MemLoop]>>1) & 0b00000001);
      Dot1bit[7] = (vram[MemLoop++] & 0b00000001);

      // The following 8 ittération used without loop for optimisation
      BitCount = 0;
      if (Dot1bit[BitCount]==1)
        {
          DotCount++;
          if ((Xloop + BitCount & 0b00000001))    // Check if dot is pair or impair
          {
            LinePrep[LinePrepLoop + BitCount] = Color[1];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[1];

            NoBlackFlag = true;
          }
          else
          {
            LinePrep[LinePrepLoop + BitCount] = Color[2];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[2];
            NoBlackFlag = true;
          }
          if (DotCount >1)
          {
            LinePrep[LinePrepLoop + BitCount - 1] = Color[3];
            LinePrep[LinePrepLoop + BitCount] = Color[3];
            NoBlackFlag = true;
          }
        }
        else
        {
          if (NoBlackFlag ==false)
          {
            LinePrep[LinePrepLoop + BitCount] = Color[0];
            LinePrep[LinePrepLoop + BitCount - 1] = Color[0];
          }
          DotCount = 0;  // Reset the DotCount That make Artefact possible if value is 2 or more
          NoBlackFlag = false;
        }

      BitCount = 1;
      if (Dot1bit[BitCount]==1)
        {
          DotCount++;
          if ((Xloop + BitCount & 0b00000001))    // Check if dot is pair or impair
          {
            LinePrep[LinePrepLoop + BitCount] = Color[1];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[1];

            NoBlackFlag = true;
          }
          else
          {
            LinePrep[LinePrepLoop + BitCount] = Color[2];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[2];
            NoBlackFlag = true;
          }
          if (DotCount >1)
          {
            LinePrep[LinePrepLoop + BitCount - 1] = Color[3];
            LinePrep[LinePrepLoop + BitCount] = Color[3];
            NoBlackFlag = true;
          }
        }
        else
        {
          if (NoBlackFlag ==false)
          {
            LinePrep[LinePrepLoop + BitCount] = Color[0];
            LinePrep[LinePrepLoop + BitCount - 1] = Color[0];
          }
          DotCount = 0;  // Reset the DotCount That make Artefact possible if value is 2 or more
          NoBlackFlag = false;
        }

      BitCount = 2;
      if (Dot1bit[BitCount]==1)
        {
          DotCount++;
          if ((Xloop + BitCount & 0b00000001))    // Check if dot is pair or impair
          {
            LinePrep[LinePrepLoop + BitCount] = Color[1];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[1];

            NoBlackFlag = true;
          }
          else
          {
            LinePrep[LinePrepLoop + BitCount] = Color[2];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[2];
            NoBlackFlag = true;
          }
          if (DotCount >1)
          {
            LinePrep[LinePrepLoop + BitCount - 1] = Color[3];
            LinePrep[LinePrepLoop + BitCount] = Color[3];
            NoBlackFlag = true;
          }
        }
        else
        {
          if (NoBlackFlag ==false)
          {
            LinePrep[LinePrepLoop + BitCount] = Color[0];
            LinePrep[LinePrepLoop + BitCount - 1] = Color[0];
          }
          DotCount = 0;  // Reset the DotCount That make Artefact possible if value is 2 or more
          NoBlackFlag = false;
        }

      BitCount = 3;
      if (Dot1bit[BitCount]==1)
        {
          DotCount++;
          if ((Xloop + BitCount & 0b00000001))    // Check if dot is pair or impair
          {
            LinePrep[LinePrepLoop + BitCount] = Color[1];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[1];

            NoBlackFlag = true;
          }
          else
          {
            LinePrep[LinePrepLoop + BitCount] = Color[2];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[2];
            NoBlackFlag = true;
          }
          if (DotCount >1)
          {
            LinePrep[LinePrepLoop + BitCount - 1] = Color[3];
            LinePrep[LinePrepLoop + BitCount] = Color[3];
            NoBlackFlag = true;
          }
        }
        else
        {
          if (NoBlackFlag ==false)
          {
            LinePrep[LinePrepLoop + BitCount] = Color[0];
            LinePrep[LinePrepLoop + BitCount - 1] = Color[0];
          }
          DotCount = 0;  // Reset the DotCount That make Artefact possible if value is 2 or more
          NoBlackFlag = false;
        }

      BitCount = 4;
      if (Dot1bit[BitCount]==1)
        {
          DotCount++;
          if ((Xloop + BitCount & 0b00000001))    // Check if dot is pair or impair
          {
            LinePrep[LinePrepLoop + BitCount] = Color[1];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[1];

            NoBlackFlag = true;
          }
          else
          {
            LinePrep[LinePrepLoop + BitCount] = Color[2];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[2];
            NoBlackFlag = true;
          }
          if (DotCount >1)
          {
            LinePrep[LinePrepLoop + BitCount - 1] = Color[3];
            LinePrep[LinePrepLoop + BitCount] = Color[3];
            NoBlackFlag = true;
          }
        }
        else
        {
          if (NoBlackFlag ==false)
          {
            LinePrep[LinePrepLoop + BitCount] = Color[0];
            LinePrep[LinePrepLoop + BitCount - 1] = Color[0];
          }
          DotCount = 0;  // Reset the DotCount That make Artefact possible if value is 2 or more
          NoBlackFlag = false;
        }

      BitCount = 5;
      if (Dot1bit[BitCount]==1)
        {
          DotCount++;
          if ((Xloop + BitCount & 0b00000001))    // Check if dot is pair or impair
          {
            LinePrep[LinePrepLoop + BitCount] = Color[1];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[1];

            NoBlackFlag = true;
          }
          else
          {
            LinePrep[LinePrepLoop + BitCount] = Color[2];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[2];
            NoBlackFlag = true;
          }
          if (DotCount >1)
          {
            LinePrep[LinePrepLoop + BitCount - 1] = Color[3];
            LinePrep[LinePrepLoop + BitCount] = Color[3];
            NoBlackFlag = true;
          }
        }
        else
        {
          if (NoBlackFlag ==false)
          {
            LinePrep[LinePrepLoop + BitCount] = Color[0];
            LinePrep[LinePrepLoop + BitCount - 1] = Color[0];
          }
          DotCount = 0;  // Reset the DotCount That make Artefact possible if value is 2 or more
          NoBlackFlag = false;
        }

      BitCount = 6;
      if (Dot1bit[BitCount]==1)
        {
          DotCount++;
          if ((Xloop + BitCount & 0b00000001))    // Check if dot is pair or impair
          {
            LinePrep[LinePrepLoop + BitCount] = Color[1];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[1];

            NoBlackFlag = true;
          }
          else
          {
            LinePrep[LinePrepLoop + BitCount] = Color[2];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[2];
            NoBlackFlag = true;
          }
          if (DotCount >1)
          {
            LinePrep[LinePrepLoop + BitCount - 1] = Color[3];
            LinePrep[LinePrepLoop + BitCount] = Color[3];
            NoBlackFlag = true;
          }
        }
        else
        {
          if (NoBlackFlag ==false)
          {
            LinePrep[LinePrepLoop + BitCount] = Color[0];
            LinePrep[LinePrepLoop + BitCount - 1] = Color[0];
          }
          DotCount = 0;  // Reset the DotCount That make Artefact possible if value is 2 or more
          NoBlackFlag = false;
        }

      BitCount = 7;
      if (Dot1bit[BitCount]==1)
        {
          DotCount++;
          if ((Xloop + BitCount & 0b00000001))    // Check if dot is pair or impair
          {
            LinePrep[LinePrepLoop + BitCount] = Color[1];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[1];

            NoBlackFlag = true;
          }
          else
          {
            LinePrep[LinePrepLoop + BitCount] = Color[2];
            LinePrep[LinePrepLoop + BitCount + 1] = Color[2];
            NoBlackFlag = true;
          }
          if (DotCount >1)
          {
            LinePrep[LinePrepLoop + BitCount - 1] = Color[3];
            LinePrep[LinePrepLoop + BitCount] = Color[3];
            NoBlackFlag = true;
          }
        }
        else
        {
          if (NoBlackFlag ==false)
          {
            LinePrep[LinePrepLoop + BitCount] = Color[0];
            LinePrep[LinePrepLoop + BitCount - 1] = Color[0];
          }
          DotCount = 0;  // Reset the DotCount That make Artefact possible if value is 2 or more
          NoBlackFlag = false;
        }

      //--------------------------------------
      LinePrepLoop+=8;
    }
    vga->drawLineFromMemory256(32, Yloop,&LinePrep[0]);
  }
}

void Do_COCO2_GRAPHMODE_256X192X2_ARTEFACT_BACK(void) // (almost)
{
     vga->clear(0b11111111);
    uint32_t MemLoop = sf.Coco2VideoPageOffset_Registers * COCO2_GRAPH_OFFSET_LEN;
    uint8_t *vram = LegacyVRAM(MemLoop);  // VRAM figée au FS émulé
    uint16_t Xloop, Yloop;
    uint8_t Dot1bit[8];
    uint8_t Color[4];

    // Définition des couleurs artefact NTSC
    Color[0] = 0b00000000;  // noir
    Color[1] = 0b11100000;  // rouge
    Color[2] = 0b00000011;  // bleu
    Color[3] = 0b11111111;  // blanc

    for (Yloop = 24; Yloop < 216; Yloop++)
    {
        for (Xloop = (sf.VideoEmulatorXpixels - X_OFFSET_32) >> 1;
             Xloop < sf.VideoEmulatorXpixels - ((sf.VideoEmulatorXpixels - X_OFFSET_32) >> 1);
             Xloop += 8)
        {
            uint8_t byte = vram[MemLoop++];

            // 8 Bit decoding for this line
            for (int i = 0; i < 8; i += 2)
            {
                // Group pixels in pairs for artefacts NTSC
                uint8_t pair = (byte >> (6 - i)) & 0b11;
                vga->dot(Xloop + i, Yloop, Color[pair]);
                vga->dot(Xloop + i + 1, Yloop, Color[pair]);
            }
        }
    }
}

void Do_COCO2_GRAPHMODE_32X16_8X12(void)
{
  if (sf.Coco2VideoGenMODE != COCO2_GRAPHMODE_32X16_8X12)
  {
    vga->clear(0b00000000);
    vga->show();
    vga->clear(0b00000000);
    vga->show();
    sf.Coco2VideoGenMODE = COCO2_GRAPHMODE_32X16_8X12;
  }
  // couleurs du texte depuis la palette GIME (12-13, ou 14-15 si CSS)
  { const uint8_t b = (sf.Coco2GraphicMode & 0x08) ? 14 : 12; s_txt_dark = LegacyPal(b); s_txt_light = LegacyPal(b + 1); }
  gfx->fillRect(((sf.VideoEmulatorXpixels-X_OFFSET_32)>>1),24-1,256,192,VDG_GREEN);
  gfx->setTextColor(0);
  uint32_t MemLoop = sf.Coco2VideoPageOffset_Registers * COCO2_GRAPH_OFFSET_LEN;
  uint8_t *vram = LegacyVRAM(MemLoop);  // VRAM figée au FS émulé
  uint8_t tmpchar;
  // sur le CoCo 3, en mode alphanumérique CoCo 1/2, le nombre de lignes par rangée vient de $FF9C (pas
  // des bits V du SAM)
  {
    static const uint8_t k_lpr[16] = { 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 4, 3, 2, 1, 12 };
    const uint8_t lpr = k_lpr[gime.reg[GIME_VSC] & 0x0F];
    const uint8_t pitch = (sf.Coco2GraphicMode & 0x01) ? 16 : 32;
    if (lpr != 12 || pitch != 32)
    {
      const uint16_t x0 = (uint16_t)((sf.VideoEmulatorXpixels - X_OFFSET_32) >> 1);
      uint8_t lir = 0;
      uint32_t row = MemLoop;
      for (uint16_t y = 0; y != 192; y++)
      {
        for (uint8_t Xloop = 0; Xloop != 32; Xloop++)
          DisplayVDGcharLine(vram[row + (pitch == 32 ? Xloop : (Xloop >> 1))], lir, x0 + Xloop * 8, y + 23);   // pas de 16 : chaque octet sur 2 cases (approximation)
        if (++lir >= lpr) { lir = 0; row += pitch; }
        if (row - MemLoop + 32 > 6144u) row = MemLoop + 6144u - 32;   // reste dans l'instantané
      }
      return;
    }
  }
  for (uint8_t Yloop = 0; Yloop !=16; Yloop++)
  {
    for (uint8_t Xloop = 0; Xloop != 32; Xloop++)
    {
      tmpchar = vram[MemLoop++];
      DisplayVDGchar(tmpchar,Xloop*8+ ((sf.VideoEmulatorXpixels-X_OFFSET_32)>>1), Yloop*12+23);
    }
  }
}

// Image CoCo 2 (Extended + Color BASIC 1.3 + Disk 1.1) dans rom[]. Conservée pour un éventuel mode
// CoCo 2 ; non appelée en configuration CoCo3.
void CopyCoCo2ROMS(void)
{
  memcpy(&rom[0x0000], extbas11, 8192);   // $8000
  memcpy(&rom[0x2000], bas13,    8192);   // $A000
  memcpy(&rom[0x4000], disk11,   8192);   // $C000
  memcpy(&rom[0xFFF0 - 0x8000], ResetVectors, 16);
}

// ROM interne CoCo3 (Super Extended Color BASIC, 32 Ko, vecteurs $FFF0-$FFFF inclus) -> rom[] ; ROM
// cartouche Disk Extended Color BASIC 2.1 -> cart_rom
static uint32_t Cart_Load(const char *path, uint8_t *dst)
{
  if (!path || path[0] != '/') return 0;
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) { LogOut.printf("[SLOT] cartouche introuvable : %s\n", path); return 0; }
  const uint32_t fsize = (uint32_t)f.size();
  memset(dst, 0xFF, GIME_CART_ROM_SIZE);
  const uint32_t n = (fsize < GIME_CART_ROM_SIZE) ? fsize : GIME_CART_ROM_SIZE;
  const uint32_t got = (n > 0) ? (uint32_t)f.read(dst, n) : 0;
  f.close();
  if (got == 0 || got != n) { LogOut.printf("[SLOT] lecture impossible : %s\n", path); return 0; }
  uint32_t p2 = 1;
  while (p2 < got) p2 <<= 1;
  for (uint32_t o = p2; o < GIME_CART_ROM_SIZE; o += p2) memcpy(dst + o, dst, p2);
  if (fsize > GIME_CART_ROM_SIZE)
    LogOut.printf("[SLOT] %s : %lu octets, seuls les 32 premiers Ko sont utilises (cartouches a banques : plus tard)\n",
                  path, (unsigned long)fsize);
  return got;
}

// cartouche à banques : toute l'image (jusqu'à 512 Ko) en PSRAM arrondie à la puissance de 2 suivante.
// Rend la taille lue (0 = échec).
static uint32_t Cart_LoadBanked(const char *path, int i)
{
  if (!path || path[0] != '/') return 0;
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) { LogOut.printf("[SLOT] cartouche introuvable : %s\n", path); return 0; }
  uint32_t fsize = (uint32_t)f.size();
  if (fsize > CART_BANKED_MAX) { LogOut.printf("[SLOT] %s : %lu octets, limite a 512 Ko\n", path, (unsigned long)fsize); fsize = CART_BANKED_MAX; }
  uint32_t p2 = 16384;
  while (p2 < fsize) p2 <<= 1;
  if (g_slot_big[i]) { heap_caps_free(g_slot_big[i]); g_slot_big[i] = nullptr; }
  g_slot_big[i] = (uint8_t *)heap_caps_malloc(p2, MALLOC_CAP_SPIRAM);
  if (!g_slot_big[i]) { f.close(); LogOut.printf("[SLOT] PSRAM insuffisante pour %s\n", path); return 0; }
  memset(g_slot_big[i], 0xFF, p2);
  const uint32_t got = fsize ? (uint32_t)f.read(g_slot_big[i], fsize) : 0;
  f.close();
  if (got == 0 || got != fsize) { LogOut.printf("[SLOT] lecture impossible : %s\n", path); return 0; }
  g_slot_bsize[i] = p2;
  g_slot_bank[i] = 0;
  LogOut.printf("[SLOT] slot %d : cartouche a banques, %lu octets (%lu banques de 16 Ko)\n", i + 1, (unsigned long)got, (unsigned long)(p2 / 16384));
  return got;
}

// description d'un slot pour les journaux ROM de disque (signature « DK » en $C000, ex. HDB-DOS, Disk
// BASIC) le BASIC l'appelle lui-même au démarrage
static bool Slot_IsDiskRom(uint8_t i)
{
  const uint8_t *r = (i < MPI_SLOTS && g_slot_rom[i]) ? g_slot_rom[i] : (i == 0 ? cart_rom : nullptr);
  return r && r[0] == 'D' && r[1] == 'K';
}
static void Slot_Text(char *b, size_t n, const SlotConfig &c, int i)
{
  if (c.mode[i] == SLOT_FDC)      snprintf(b, n, "controleur de disquette (Disk BASIC 2.1)");
  else if (SLOT_HAS_ROM(c.mode[i])) snprintf(b, n, "%s %s%s", c.mode[i] == SLOT_O90 ? "Orchestra-90" : (c.mode[i] == SLOT_GMC ? "Game Master Cartridge" : (c.mode[i] == SLOT_CPSG ? "CoCo PSG" : "cartouche")), c.name[i],
                                       (c.mode[i] == SLOT_ROM && !g_slot_big[i] && Slot_IsDiskRom((uint8_t)i)) ? " (ROM de disque DK : lancee par le BASIC, ligne CART inactive)" : (c.autostart[i] ? " (demarrage automatique)" : ""));
  else if (c.mode[i] == SLOT_RS232) snprintf(b, n, "RS-232 Pak (ACIA 6551 $FF68)");
  else                            snprintf(b, n, "vide");
}
void Slots_Report(const char *tag)
{
  char b[320];
  const SlotConfig &c = g_slotcfg_run;
  if (!c.mpi) { Slot_Text(b, sizeof b, c, 0); LogOut.printf("%s port d'extension : %s\n", tag, b);
                if (g_rs_present) { Rs_Report(b, sizeof b); LogOut.printf("%s %s\n", tag, b); } return; }
  LogOut.printf("%s Multi-Pak : commutateur slot %u, $FF7F = $%02X (CTS slot %u, SCS slot %u)\n", tag,
                c.sw + 1, g_mpi_reg, ((g_mpi_reg >> 4) & 3) + 1, (g_mpi_reg & 3) + 1);
  for (int i = 0; i < MPI_SLOTS; i++) { Slot_Text(b, sizeof b, c, i); LogOut.printf("%s   slot %d : %s\n", tag, i + 1, b); }
  if (g_rs_present) { Rs_Report(b, sizeof b); LogOut.printf("%s %s\n", tag, b); }
}

// ligne CART en niveau (IRQ de l'ACIA du RS-232 Pak), en plus de l'horloge Q des cartouches de jeu.
// Elle va à EI0 du GIME (front d'activation, gime_set_ext_line) au PIA1 CB1
#ifndef RS_CART_TIED
#define RS_CART_TIED 1
#endif
static bool s_rs_irq = false, s_cart_line = false, s_cb1_flag = false;
static void Cart_Firq(void)
{
  const bool flag = g_cart_clock || s_cb1_flag;
  rom[ROM_FF23] = (uint8_t)((rom[ROM_FF23] & 0x3F) | (flag ? 0x80 : 0x00));
  sf.firq_pin = !(flag && (rom[ROM_FF23] & 0x01));
}
static void Cart_LineUpdate(void)
{
  const bool cts_ok = RS_CART_TIED || !g_slotcfg_run.mpi || (((g_mpi_reg >> 4) & 3) == g_rs_slot);
  const bool act = g_rs_present && s_rs_irq && cts_ok;
  if (act == s_cart_line) return;
  s_cart_line = act;
  gime_set_ext_line(GIME_INT_EI0, act);
  // Comme MAME (cococart.cpp : ASSERT = niveau 1 sur CB1) : l'activation est un front montant pour le
  // PIA ; $FF23 bit 1 = 1 (valeur du BASIC, $37) le retient.
  const bool rising_sel = (rom[ROM_FF23] & 0x02) != 0;
  if (act == rising_sel) s_cb1_flag = true;
  Cart_Firq();
#ifndef ESP_PLATFORM
  if (getenv("RSDBG")) fprintf(stderr, "[RSDBG] CART %u, drapeau CB1 %u, $FF23 $%02X, firq_pin %u, PC $%04X\n", act, s_cb1_flag, rom[ROM_FF23], sf.firq_pin, cpu->get_pc());
#endif
}
void Cart_FirqPublic(void) { Cart_Firq(); }
void Rs_IrqChanged(bool active)   // appelée par rs232.cpp (coeur CPU)
{
  s_rs_irq = active;
  Cart_LineUpdate();
}
void Cart_Cb1Ack(void)             // lecture de $FF22
{
#ifndef ESP_PLATFORM
  if (getenv("RSDBG") && s_cb1_flag) fprintf(stderr, "[RSDBG] lecture $FF22 : drapeau CB1 efface, PC $%04X\n", cpu->get_pc());
#endif
  if (s_cb1_flag) { s_cb1_flag = false; Cart_Firq(); }
}

// effets du registre $FF7F (ou du slot unique) : ROM visible en $C000 et ligne CART (slot CTS), E/S
// $FF40-$FF5F (slot SCS).
static void Slots_Apply(void)
{
  const SlotConfig &c = g_slotcfg_run;
  const uint8_t cts = c.mpi ? ((g_mpi_reg >> 4) & 3) : 0;
  const uint8_t scs = c.mpi ? (g_mpi_reg & 3) : 0;
  if (c.mode[cts] == SLOT_CPSG && g_slot_big[cts])     // deux banques de 8 Ko
    gime_set_cart_banks8(g_slot_big[cts] + (((uint32_t)(g_cpsg_bank[0] & 0x3F) * 8192u) & (g_slot_bsize[cts] - 1)),
                         g_slot_big[cts] + (((uint32_t)(g_cpsg_bank[1] & 0x3F) * 8192u) & (g_slot_bsize[cts] - 1)));
  else if (g_slot_bsize[cts] && g_slot_big[cts])       // banque de 16 Ko en miroir
    gime_set_cart(g_slot_big[cts] + (((uint32_t)g_slot_bank[cts] * 16384u) & (g_slot_bsize[cts] - 1)), 0x3FFF);
  else
    gime_set_cart(g_slot_rom[cts] ? g_slot_rom[cts] : cart_rom);
  g_ff40_fdc   = (c.mode[scs] == SLOT_FDC);
  g_scs_slot   = scs;
  g_scs_banked = g_slot_bsize[scs] != 0 && c.mode[scs] != SLOT_CPSG;
  g_scs_gmc    = (c.mode[scs] == SLOT_GMC);
  g_scs_cpsg   = (c.mode[scs] == SLOT_CPSG);
  g_cart_clock = SLOT_HAS_ROM(c.mode[cts]) && c.autostart[cts] && !(c.mode[cts] == SLOT_CPSG && (g_cpsg_ctrl & 0x10)) &&
                 !(c.mode[cts] == SLOT_ROM && !g_slot_big[cts] && Slot_IsDiskRom(cts));
  gime_set_cart_clock(g_cart_clock);
  Cart_LineUpdate();   // le slot CTS a pu changer
  Cart_Firq();
}
// écriture en $FF40 : banque du slot SCS ; $FF41 : SN76489A (GMC)
void Cart_BankWrite(uint8_t v)
{
  g_slot_bank[g_scs_slot] = v;
  Slots_Apply();
}
void Gmc_PsgWrite(uint8_t v)
{
#ifndef ESP_PLATFORM
  if (g_psg_hook) g_psg_hook(v);   // banc : journal horodaté des écritures PSG
#endif
  Audio_PsgWrite(g_audio_realtime ? (uint32_t)((uint64_t)esp_timer_get_time() * 14318180ull / 1000000ull) : (uint32_t)g_emu_ticks, v);
}

// registres de la CoCo PSG ($FF50-$FF5F du slot SCS)
static inline uint32_t Snd_Stamp(void)
{
  return g_audio_realtime ? (uint32_t)((uint64_t)esp_timer_get_time() * 14318180ull / 1000000ull) : (uint32_t)g_emu_ticks;
}
void Cpsg_Write(uint16_t a, uint8_t v)
{
  switch (a & 0x0F)
  {
    case 0x0A: case 0x0B: g_cpsg_bank[a & 1] = v; Slots_Apply(); break;
    case 0x0D:
      g_cpsg_ctrl = v;
#ifndef ESP_PLATFORM
      if (g_ay_hook) g_ay_hook(0x10, v);
#endif
      Audio_AyWrite(Snd_Stamp(), 0x10, v);
      Slots_Apply();                                  // bit 4 : démarrage automatique
      break;
    case 0x0E: g_cpsg_latch = v & 0x0F; break;
    case 0x0F:
      g_cpsg_reg[g_cpsg_latch] = v;
#ifndef ESP_PLATFORM
      if (g_ay_hook) g_ay_hook(g_cpsg_latch, v);
#endif
      Audio_AyWrite(Snd_Stamp(), g_cpsg_latch, v);
      break;
    default: break;
  }
}
uint8_t Cpsg_Read(uint16_t a)
{
  switch (a & 0x0F)
  {
    case 0x0A: case 0x0B: return g_cpsg_bank[a & 1];
    case 0x0F: return (g_cpsg_latch >= 14) ? 0xFF : g_cpsg_reg[g_cpsg_latch];   // ports A/B : manettes numériques (rien d'appuyé)
    default: return 0x00;
  }
}

void Mpi_Write(uint8_t v)
{
  g_mpi_reg = v;
  Slots_Apply();
}

void CopyCoCo3ROMS(void)
{
  memcpy(rom, coco3, 32768);
  // contenu du port d'extension selon le choix du menu F12 S
  g_slotcfg_run = g_slotcfg;
  SlotConfig &c = g_slotcfg_run;
  const int n = c.mpi ? MPI_SLOTS : 1;
  bool fdc_seen = false;
  for (int i = 0; i < MPI_SLOTS; i++)
  {
    if (!g_slot_rom[i]) { c.mode[i] = SLOT_EMPTY; continue; }
    memset(g_slot_rom[i], 0xFF, GIME_CART_ROM_SIZE);
    if (i >= n) continue;
    g_slot_bsize[i] = 0;
    bool loaded = true;
    if (SLOT_HAS_ROM(c.mode[i]))
    {
      // Game Master Cartridge, ou toute image de plus de 32 Ko : à banques
      File fz = SD_MMC.open(c.name[i], FILE_READ);
      const uint32_t fsz = fz ? (uint32_t)fz.size() : 0;
      if (fz) fz.close();
      if (c.mode[i] == SLOT_GMC || c.mode[i] == SLOT_CPSG || fsz > GIME_CART_ROM_SIZE) loaded = Cart_LoadBanked(c.name[i], i) != 0;
      else loaded = Cart_Load(c.name[i], g_slot_rom[i]) != 0;
    }
    if (!loaded)
    {
      LogOut.printf("[SLOT] slot %d : cartouche absente%s\n", i + 1, c.mpi ? " : slot vide" : " : retour au controleur de disquette (Disk BASIC)");
      c.mode[i] = c.mpi ? SLOT_EMPTY : SLOT_FDC;
      Overlay_Request("CARTRIDGE NOT FOUND", c.mpi ? "SLOT LEFT EMPTY" : "DISK BASIC LOADED", 4000);
    }
    if (c.mode[i] == SLOT_FDC)
    {
      if (fdc_seen) { c.mode[i] = SLOT_EMPTY; continue; }   // un seul contrôleur de disquette
      fdc_seen = true;
      memcpy(g_slot_rom[i], disk21, sizeof(disk21));
      memcpy(g_slot_rom[i] + 16384, g_slot_rom[i], 16384);   // miroir (16 Ko, A14 non reliée)
    }
  }
  if (!c.mpi) for (int i = 1; i < MPI_SLOTS; i++) c.mode[i] = SLOT_EMPTY;
  g_o90_present = false;                                   // une seule Orchestra-90 (ses CNA répondent quel que soit SCS)
  for (int i = 0; i < MPI_SLOTS; i++)
  {
    if (c.mode[i] != SLOT_O90) continue;
    if (g_o90_present) c.mode[i] = SLOT_EMPTY; else g_o90_present = true;
  }
  g_gmc_present = false;                                   // une seule Game Master Cartridge
  for (int i = 0; i < MPI_SLOTS; i++)
  {
    if (c.mode[i] != SLOT_GMC) continue;
    if (g_gmc_present) c.mode[i] = SLOT_EMPTY; else g_gmc_present = true;
  }
  Audio_PsgEnable(g_gmc_present);
  g_cpsg_present = false;                                  // une seule CoCo PSG
  for (int i = 0; i < MPI_SLOTS; i++)
  {
    if (c.mode[i] != SLOT_CPSG) continue;
    if (g_cpsg_present) c.mode[i] = SLOT_EMPTY; else g_cpsg_present = true;
  }
  g_cpsg_bank[0] = 0; g_cpsg_bank[1] = 1; g_cpsg_ctrl = 0; g_cpsg_latch = 0;
  memset(g_cpsg_reg, 0, sizeof g_cpsg_reg);
  Audio_AyEnable(g_cpsg_present);
  g_rs_present = false;                                    // un seul RS-232 Pak
  for (int i = 0; i < MPI_SLOTS; i++)
  {
    if (c.mode[i] != SLOT_RS232) continue;
    if (g_rs_present) c.mode[i] = SLOT_EMPTY; else { g_rs_present = true; g_rs_slot = (uint8_t)i; }
  }
  s_rs_irq = false; s_cart_line = false; s_cb1_flag = false;
  Rs_Reset();
  Rs_ApplyLink();
  Bb_Reset();
  if ((g_rs_present && g_rs_link == RS_LINK_MODEM) || g_bb_mode == BB_MODEM) Modem_Use();   // au démarrage (tâche du modem)
  NetDw_Flush();
  g_mpi_reg = c.mpi ? (uint8_t)((c.sw << 4) | c.sw) : 0;   // à l'allumage : CTS = SCS = commutateur
  Slots_Apply();
  Slots_Report("[SLOT]");
}

// DRQ = 1 when the Computer can read data or write data to the register. DRQ is 0 when is busy.

#define MACRO_TRACK_R DiskAccess.DSK_FILE_DataPTR = (DiskAccess.TrackPos * 18 * 256) + ((DiskAccess.SectorPos - 1) * 256)
#define MACRO_TRACK_W DiskAccess.DSK_FILE_DataPTR = (DiskAccess.TrackPos * 18 * 256) + ((DiskAccess.SectorPos - 1) * 256)
#define M_DRQ_BIT_READY 0b00000010

// ===========================================================================
// contrôleur de disquette CoCo (FD-502 / Disk BASIC) émulé comme le vrai matériel
// ===========================================================================
// géométrie par lecteur (Disk_Tracks/Sides/Spt/First, couche disque)
uint8_t Disk_Sides(uint8_t drv); uint8_t Disk_Tracks(uint8_t drv);
uint8_t Disk_Spt(uint8_t drv);   uint8_t Disk_First(uint8_t drv);
bool Disk_ReadSector(uint8_t drv, uint8_t track, uint8_t side, uint8_t sector, uint8_t *buf);
bool Disk_WriteSector(uint8_t drv, uint8_t track, uint8_t side, uint8_t sector, const uint8_t *buf);
#define FDC_END_DELAY     64      // cycles entre le dernier octet et INTRQ (CRC)
#define FDC_TYPE1_DELAY   200     // cycles d'une commande de type I (déplacement)
#define FDC_WTRACK_BYTES  6250    // octets acceptés par WRITE TRACK
#define FDC_LOST_SECTOR   4000    // cycles sans lecture/écriture d'octet -> LOST DATA
#define FDC_LOST_RADDR    300     // READ ADDRESS : le champ ID défile vite
// Un vrai WD1793 n'attend pas le CPU : si l'octet n'est pas pris à temps, la commande se termine quand
// même avec LOST DATA

enum FdcMode : uint8_t { FDC_IDLE, FDC_READ, FDC_WRITE, FDC_RADDR, FDC_WTRACK };

struct FdcState
{
  uint8_t  dskreg;        // $FF40
  uint8_t  track;         // $FF49 registre de piste
  uint8_t  sector;        // $FF4A registre de secteur
  uint8_t  data;          // $FF4B registre de données
  uint8_t  cmd;           // dernière commande
  uint8_t  head;          // piste physique sous la tête
  uint8_t  err;           // bits d'erreur du statut (RNF $10, ...)
  bool     typeI;         // statut à présenter : type I ou II/III
  bool     busy, drq, intrq;
  bool     dir_in;        // direction du dernier STEP
  uint8_t  mode;
  uint16_t remain;        // octets restant à transférer
  uint32_t ptr;           // position dans l'image
  int32_t  end_cd;        // compte à rebours (cycles) avant INTRQ, <0 = rien
  uint8_t  addr[6];       // champ ID pour READ ADDRESS
  uint8_t  next_id;       // secteur « sous la tête » pour READ ADDRESS
  uint32_t index_cyc;     // pour simuler l'impulsion d'index
  int32_t  lost_cd;       // délai max. pour servir un octet (sinon LOST DATA)
  uint8_t  side;          // face ($FF40 bit 6 si disque double face)
  uint16_t pos;           // position dans le tampon secteur
  uint8_t  buf[256];      // tampon du secteur en cours (lecture / écriture)
  bool     buf_loaded;    // lecture : secteur chargé (au 1er octet lu, voir Fdc_Read)
};
static FdcState fdc;
static inline void Fdc_AddIndex(uint32_t c) { fdc.index_cyc += c; }
static inline bool Fdc_IsBusy(void) { return fdc.busy; }

// ---------------------------------------------------------------------------
// budget d'enchaînement des NOP (voir L12 dans execute). Cycles CPU avant le prochain évènement GIME
// (fin de ligne : HBORD, HS du PIA, FS, VBORD, capture vidéo, timer à 63,5 µs)
// ---------------------------------------------------------------------------
static constexpr bool g_nop_batch = true;   // suites de NOP d'un bloc
static uint32_t Nop_Budget(void)
{
  if (!g_nop_batch || g_gime_div_q8 != 256 || fdc.busy) return 0;
  if ((gime.reg[GIME_INIT1] & 0x20) && gime_raster.timer_count) return 0;
  const uint32_t sh = (sf.CPU_Speed == CPU_FAST) ? 3 : 4;
  const uint32_t used = gime_raster.line_ticks + (g_batch_acc << sh);   // cycles du lot pas encore appliqués
  if (used >= GIME_TICKS_PER_LINE) return 0;
  // arrêt aussi au front de HS si le timer (TINS=0) y arrive à échéance
  if (gime_raster.timer_count == 1u && !gime_hs_done && used < gime_hs_tick()) return (gime_hs_tick() - used) >> sh;
  return (GIME_TICKS_PER_LINE - used) >> sh;   // ticks jusqu'à la fin de ligne
}
static struct NopBudgetInit { NopBudgetInit() { mc6809_nop_budget = Nop_Budget; } } s_nop_budget_init;

static void Fdc_UpdateLines(void)
{
  if (fdc.intrq && (fdc.dskreg & 0x20)) fdc.dskreg &= 0x7F;   // INTRQ efface HALT enable
  sf.nmi_pin = !(fdc.intrq && (fdc.dskreg & 0x20));             // actif à false
}

// Ligne HALT du 6809 (utilisée par StepCPU)
static inline bool Fdc_HaltLine(void)
{
  return fdc.busy && (fdc.dskreg & 0x80) && !fdc.drq;   // busy testé d'abord (le plus souvent faux)
}

static void Fdc_Reset(void)
{
  memset(&fdc, 0, sizeof fdc);
  fdc.typeI = true; fdc.end_cd = -1; fdc.dir_in = true; fdc.sector = 1; fdc.next_id = 1;
  Fdc_UpdateLines();
}

static void Fdc_SelectDrive(void)
{
  DiskAccess.DriveSelected = GetDriveNumber(fdc.dskreg);
  // $FF40 bit 6 : face 1 pour un disque double face (sinon sélection du lecteur 3 de Disk BASIC, déjà
  // traitée par GetDriveNumber).
  fdc.side = ((fdc.dskreg & 0x40) && Disk_Sides(DiskAccess.DriveSelected) > 1) ? 1 : 0;
}

static void Fdc_ScheduleEnd(int32_t cycles)
{
  fdc.drq = false;
  fdc.busy = true;
  fdc.end_cd = cycles;
}

static void Fdc_Finish(void)          // fin de commande : INTRQ
{
  fdc.mode = FDC_IDLE;
  fdc.busy = false;
  fdc.drq = false;
  fdc.intrq = true;
  fdc.end_cd = -1;
  DISKETTE_LED_OFF();
  Fdc_UpdateLines();
}

// Appelée à chaque instruction avec le nombre de cycles écoulés
static inline __attribute__((always_inline)) void Fdc_Tick(uint16_t c)
{
  fdc.index_cyc += c;
  if (!fdc.busy) return;   // contrôleur au repos (end_cd >= 0 implique busy)
  if (fdc.drq)
  {
    fdc.lost_cd -= c;
    if (fdc.lost_cd <= 0) { fdc.err |= 0x04; Fdc_Finish(); return; }   // LOST DATA
  }
  if (fdc.end_cd >= 0)
  {
    fdc.end_cd -= c;
    if (fdc.end_cd <= 0) Fdc_Finish();
  }
}

// RECORD NOT FOUND par lecteur (rapport F9). Normal en petit nombre (protections, formatage) ; en
// grand nombre = image abîmée ou incompatible.
uint32_t g_fdc_rnf[4] = {0, 0, 0, 0};
static inline void Fdc_CountRnf(void)
{
  if (DiskAccess.DriveSelected < 4) g_fdc_rnf[DiskAccess.DriveSelected]++;
}
static bool Fdc_SectorValid(uint8_t s)
{
  const uint8_t d = DiskAccess.DriveSelected;
  const uint8_t first = Disk_First(d);
  if (fdc.head >= Disk_Tracks(d) || fdc.side >= Disk_Sides(d) || s < first) return false;
  // comme le vrai WD1793 (et VCC, MAME), le champ ID du secteur doit porter le numéro de piste du
  // registre de piste : registre écrit sans SEEK (tête ailleurs) -> RECORD NOT FOUND
  if (fdc.track != fdc.head) return false;
  return s < first + Disk_Spt(d);
}

// Prépare le transfert du secteur fdc.sector (lecture : charge le tampon). false = secteur introuvable
// (RNF).
static bool Fdc_StartSector(void)
{
  // en lecture, le secteur n'est chargé qu'au premier octet lu
  if (fdc.mode == FDC_READ && !Fdc_SectorValid(fdc.sector)) return false;
  fdc.buf_loaded = false;
  fdc.pos = 0;
  fdc.remain = 256;
  fdc.drq = true;
  fdc.busy = true;
  fdc.lost_cd = FDC_LOST_SECTOR;
  return true;
}

static void Fdc_Command(uint8_t v)
{
  Fdc_SelectDrive();
  if ((v & 0xF0) == 0xD0)             // type IV : FORCE INTERRUPT
  {
    fdc.mode = FDC_IDLE; fdc.busy = false; fdc.drq = false; fdc.end_cd = -1;
    fdc.typeI = true;
    fdc.intrq = (v & 0x0F) != 0;      // $D8 etc. : interruption immédiate
    DISKETTE_LED_OFF();
    Fdc_UpdateLines();
    return;
  }
  fdc.cmd = v;
  fdc.intrq = false;
  fdc.err = 0;
  if ((v & 0x80) == 0)                // type I : RESTORE/SEEK/STEP
  {
    fdc.typeI = true;
    switch (v & 0xF0)
    {
      case 0x00: fdc.head = 0; fdc.track = 0; break;                        // RESTORE
      case 0x10: fdc.head = fdc.data; fdc.track = fdc.data; break;           // SEEK
      case 0x20: case 0x30:                                                  // STEP
        if (fdc.dir_in) fdc.head++; else if (fdc.head) fdc.head--;
        if (v & 0x10) fdc.track = fdc.head;
        break;
      case 0x40: case 0x50:                                                  // STEP IN
        fdc.dir_in = true; fdc.head++;
        if (v & 0x10) fdc.track++;
        break;
      default:                                                               // STEP OUT
        fdc.dir_in = false; if (fdc.head) fdc.head--;
        if ((v & 0x10) && fdc.track) fdc.track--;
        break;
    }
    if ((v & 0x04) && fdc.head >= Disk_Tracks(DiskAccess.DriveSelected)) fdc.err = 0x10;   // vérif. : SEEK ERROR
    fdc.mode = FDC_IDLE;
    Fdc_ScheduleEnd(FDC_TYPE1_DELAY);
    Fdc_UpdateLines();
    return;
  }
  fdc.typeI = false;
  DISKETTE_LED_ON();
  switch (v & 0xE0)
  {
    case 0x80:                        // READ SECTOR (bit 4 = multiple)
    case 0xA0:                        // WRITE SECTOR
      if (!Fdc_SectorValid(fdc.sector))
      {
        fdc.err = 0x10;               // RECORD NOT FOUND
        Fdc_CountRnf();
        fdc.mode = FDC_IDLE;
        Fdc_ScheduleEnd(FDC_END_DELAY);
        break;
      }
      fdc.mode = ((v & 0xE0) == 0x80) ? FDC_READ : FDC_WRITE;
      if (!Fdc_StartSector())
      {
        fdc.err = 0x10;               // RECORD NOT FOUND
        Fdc_CountRnf();
        fdc.mode = FDC_IDLE;
        Fdc_ScheduleEnd(FDC_END_DELAY);
      }
      break;
    case 0xC0:                        // READ ADDRESS ($C0) / FORCE INT traité plus haut
    {
      const uint8_t spt = Disk_Spt(DiskAccess.DriveSelected), first = Disk_First(DiskAccess.DriveSelected);
      if (fdc.next_id < first || fdc.next_id >= first + spt) fdc.next_id = first;
      uint8_t id = fdc.next_id;
      fdc.next_id = (uint8_t)(first + ((id - first + 1) % spt));
      fdc.addr[0] = fdc.head; fdc.addr[1] = fdc.side; fdc.addr[2] = id; fdc.addr[3] = 1;
      fdc.addr[4] = 0; fdc.addr[5] = 0;
      fdc.mode = FDC_RADDR; fdc.remain = 6; fdc.drq = true; fdc.busy = true;
      fdc.lost_cd = FDC_LOST_RADDR;
      break;
    }
    default:                          // $E0 READ TRACK / $F0 WRITE TRACK
      if ((v & 0xF0) == 0xF0 && fdc.head < Disk_Tracks(DiskAccess.DriveSelected) &&
          fdc.side < Disk_Sides(DiskAccess.DriveSelected))
      {
        // Formatage : les secteurs de la piste (face courante) sont remplis de $FF (la GAT de Disk
        // BASIC lit $FF = granule libre).
        const uint8_t spt = Disk_Spt(DiskAccess.DriveSelected), first = Disk_First(DiskAccess.DriveSelected);
        memset(fdc.buf, 0xFF, 256);
        for (uint8_t k = 0; k < spt; k++) Disk_WriteSector(DiskAccess.DriveSelected, fdc.head, fdc.side, first + k, fdc.buf);
        fdc.mode = FDC_WTRACK; fdc.remain = FDC_WTRACK_BYTES; fdc.drq = true; fdc.busy = true;
        fdc.lost_cd = FDC_LOST_SECTOR;
      }
      else
      {
        fdc.mode = FDC_IDLE;          // READ TRACK non émulé : fin immédiate
        Fdc_ScheduleEnd(FDC_END_DELAY);
      }
      break;
  }
  Fdc_UpdateLines();
}

static uint8_t Fdc_Status(void)
{
  uint8_t s = fdc.err;
  if (fdc.busy) s |= 0x01;
  if (fdc.typeI)
  {
    if (fdc.head == 0) s |= 0x04;                                   // TRK00
    if ((fdc.index_cyc % 178977u) < 2000u) s |= 0x02;               // index (~5 tr/s)
    s |= 0x20;                                                      // tête chargée
  }
  else if (fdc.drq) s |= 0x02;
  return s;
}

// Lecture $FF40-$FF4F (déjà ramenée à $FF40 / $FF48-$FF4B) : met la valeur dans rom[] (read8 la
// renvoie ensuite).
static void Fdc_Read(uint16_t a)
{
  switch (a)
  {
    case 0xFF48:
      rom[ROM_FF48] = Fdc_Status();
      if (fdc.intrq) { fdc.intrq = false; Fdc_UpdateLines(); }      // lecture = acquittement
      break;
    case 0xFF49: rom[ROM_FF49] = fdc.track;  break;
    case 0xFF4A: rom[ROM_FF4A] = fdc.sector; break;
    case 0xFF4B:
      if (fdc.mode == FDC_READ && fdc.drq)
      {
        if (!fdc.buf_loaded)
        {
          Fdc_SelectDrive();                                        // face / lecteur actuels
          if (!Disk_ReadSector(DiskAccess.DriveSelected, fdc.head, fdc.side, fdc.sector, fdc.buf))
          {
            fdc.err = 0x10;                                         // RECORD NOT FOUND
            Fdc_CountRnf();
            Fdc_ScheduleEnd(FDC_END_DELAY);
            rom[ROM_FF4B] = fdc.data;
            break;
          }
          fdc.buf_loaded = true;
        }
        fdc.data = fdc.buf[fdc.pos++];
        fdc.lost_cd = FDC_LOST_SECTOR;
        if (--fdc.remain == 0)
        {
          if ((fdc.cmd & 0x10) && Fdc_SectorValid(fdc.sector + 1))
          {
            fdc.sector++;                                           // multi-secteur
            if (!Fdc_StartSector()) { fdc.err = 0x10; Fdc_ScheduleEnd(FDC_END_DELAY); }
          }
          else
          {
            if (fdc.cmd & 0x10) { fdc.sector++; fdc.err = 0x10; }   // fin de piste : RNF
            Fdc_ScheduleEnd(FDC_END_DELAY);
          }
        }
      }
      else if (fdc.mode == FDC_RADDR && fdc.drq)
      {
        fdc.data = fdc.addr[6 - fdc.remain];
        if (--fdc.remain == 0)
        {
          fdc.sector = fdc.addr[0];                                 // le WD copie la piste lue ici
          Fdc_ScheduleEnd(FDC_END_DELAY);
        }
      }
      rom[ROM_FF4B] = fdc.data;
      break;
    default:
      rom[ROM_FF40] = fdc.dskreg;
      break;
  }
}

static void Fdc_Write(uint16_t a, uint8_t v)
{
  switch (a)
  {
    case 0xFF40:
      fdc.dskreg = v;
      rom[ROM_FF40] = v;
      Fdc_SelectDrive();
      Fdc_UpdateLines();
      break;
    case 0xFF48: Fdc_Command(v); break;
    case 0xFF49: fdc.track = v;  break;
    case 0xFF4A: fdc.sector = v; break;
    case 0xFF4B:
      fdc.data = v;
      if (fdc.mode == FDC_WRITE && fdc.drq)
      {
        fdc.buf[fdc.pos++] = v;
        fdc.lost_cd = FDC_LOST_SECTOR;
        if (--fdc.remain == 0)
        {
          Fdc_SelectDrive();                                        // face actuelle ($FF40 bit 6)
          if (!Disk_WriteSector(DiskAccess.DriveSelected, fdc.head, fdc.side, fdc.sector, fdc.buf))
          {
            fdc.err = 0x10;
            Fdc_ScheduleEnd(FDC_END_DELAY);
          }
          else if ((fdc.cmd & 0x10) && Fdc_SectorValid(fdc.sector + 1))
          {
            fdc.sector++;
            Fdc_StartSector();
          }
          else
          {
            Fdc_ScheduleEnd(FDC_END_DELAY);
          }
        }
      }
      else if (fdc.mode == FDC_WTRACK && fdc.drq)
      {
        fdc.lost_cd = FDC_LOST_SECTOR;
        if (--fdc.remain == 0) Fdc_ScheduleEnd(FDC_END_DELAY);
      }
      break;
  }
}

IRAM_ATTR __attribute__((optimize("no-jump-tables"))) void ManagePeripherals_Read(uint16_t address)   // pas de table de sauts en flash
{
  // Manage all the read operation of memory addresses
  switch (address)
  {
     // et overlay disk21[]. Le contrôleur disque réel n'active pas la ligne CART ; Extended BASIC
     // détecte « DK » en $C000 et SECB a déjà recopié la ROM cartouche en RAM

    case M_FF22:   // lecture de $FF22 = acquittement du drapeau CB1 (ligne CART)
      { extern void Cart_Cb1Ack(void); Cart_Cb1Ack(); }
      break;

    case M_FF02: // ROM_FF02
      rom[ROM_FF03] &= 0b01111111;  // If ff02 Address is read, it reset the Vsynch interrupt flag AT ff03.
      // La lecture du registre de données acquitte TOUJOURS la demande d'interruption sur un vrai PIA
      // 6821, peu importe l'état courant du bit d'activation — sinon irq_pin reste bloqué à false pour
      // de bon si…
      sf.V_Synch = false;
      PIA0_UpdateIRQ();   // la source HS peut rester active

      break;
      // All next are drive related--------------------------

    case M_FF00: // ROM_FF02
    rom[ROM_FF01] &= 0x7F;   // lecture de $FF00 = acquittement HS
    PIA0_UpdateIRQ();

    uint8_t val1, buttons, tmp1;
    uint8_t MultiplexerCB2_CA2;
    if (true)
    {
      ManageKeyboardScan(rom[ROM_FF02]);
      // Do the joystick stuff first
      buttons = ReadCoCoButtons();

      if ((buttons & 0b00000001) == 0)
      {

        sf.is_JOY1_B1_WasPressed=true;
        rom[ROM_FF00] &=0b11111110;
      }
      else
      {

        if (!sf.is_LastKeyboardScanned)
        {
          rom[ROM_FF00] |=0b00000001;
        }
        else
        {
          sf.is_LastKeyboardScanned = false;
        }
      }

      if ((buttons & 0b00000010) == 0)
      {

        rom[ROM_FF00] &=0b11111101;
      }

      MultiplexerCB2_CA2 = (((rom[ROM_FF03] & 0b00001000)>>2) | ((rom[ROM_FF01] & 0b00001000)>>3));   // Get the two bits Comparator Selection
      val1 = ReadJoysticks(MultiplexerCB2_CA2);   // Sample Requested joystick

      // 2e bouton des manettes (droite = PA2, gauche = PA3), comme le CoCo3
      if (USB_DEV_CONTROL.JOY1_BUTT2 == 0) rom[ROM_FF00] &= 0b11111011;
      if (USB_DEV_CONTROL.JOY2_BUTT2 == 0) rom[ROM_FF00] &= 0b11110111;
      bool hires_cmp;
      if (Mouse_Comparator(MultiplexerCB2_CA2, &hires_cmp))
      {
        if (hires_cmp) rom[ROM_FF00] |= 0b10000000; else rom[ROM_FF00] &= 0b01111111;
      }
      else if (val1 > (Pia1A_Out() & 0b11111100))   // sortie réelle du CNA (DDR)
      {
        rom[ROM_FF00] |= 0b10000000;

      }
      else
      {
        rom[ROM_FF00] &= 0b01111111;

      }

    }
    else // Manage Keyboard
    {
      uint8_t val1;
      ManageKeyboardScan(rom[ROM_FF02]);
    }

      break;

    // contrôleur de disquette réaliste (voir Fdc_*)
    case M_FF40:
    case M_FF48:
    case M_FF49:
    case M_FF4A:
    case M_FF4B:
      Fdc_Read(address);
      break;

    default:
    break;
  }

}

uint8_t ReadDiskByte(uint32_t BytePos)
{
  uint8_t ByteRead;
  switch (DiskAccess.DriveSelected)
  {
  case 0:
  ByteRead = RAM_Disk0[BytePos];
  break;
  case 1:
  ByteRead = RAM_Disk1[BytePos];
  break;
  case 2:
  ByteRead = RAM_Disk2[BytePos];
  break;
  case 3:
  ByteRead = RAM_Disk3[BytePos];
  break;

  default:
    break;
  }
  return ByteRead;
}

void WriteDiskByte(uint32_t BytePos, uint8_t ByteData)
{

  switch (DiskAccess.DriveSelected)
  {
  case 0:
    RAM_Disk0[BytePos] = ByteData;
  break;
  case 1:
    RAM_Disk1[BytePos] = ByteData;
  break;
  case 2:
    RAM_Disk2[BytePos] = ByteData;
  break;
  case 3:
    RAM_Disk3[BytePos] = ByteData;
  break;

  default:
    break;
  }

}

uint8_t MountFileSystem(void)
{
  if (!SD_MMC.begin())
    return 2; // No Card / init fail

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
    return 3; // No media

  return 0;
}

uint8_t GetDriveNumber(uint8_t Address)
{
  uint8_t tmpDrive = 0;
  if (Address & 0b00000001)
  {
    tmpDrive = 0;
  }
  else if (Address & 0b00000010)
  {
    tmpDrive = 1;
  }
  else if (Address & 0b00000100)
  {
    tmpDrive = 2;
  }
  else if (Address & 0b01000000)
  {
    tmpDrive = 3;
  }
  return tmpDrive;
}

// après une lecture sous l'ancien nom .ccc, réécrire sous le nouveau nom .cfg puis effacer les anciens
// fichiers (sinon /config.ccc apparaîtrait comme une « cartouche » dans le navigateur)
bool SaveConfigToSD(void);
bool SaveJoystickConfigToSD(void);
void Cfg_MigrateLegacy(void)
{
  if (!s_cfg_legacy_used) return;
  s_cfg_legacy_used = false;
  const bool okc = SaveConfigToSD();
  const bool okj = SD_MMC.exists(CFG_DIR "/joyconfig.cfg") || SaveJoystickConfigToSD();
  static const char *k_old[6] = { CFG_DIR "/config.ccc", CFG_DIR "/config.ccc.bak", "/config.ccc",
                                  CFG_DIR "/joyconfig.ccc", CFG_DIR "/joyconfig.ccc.bak", "/joyconfig.ccc" };
  for (int i = 0; i < 6; i++)
  {
    if (!(i < 3 ? okc : okj)) continue;
    if (SD_MMC.exists(k_old[i]) && SD_MMC.remove(k_old[i])) LogOut.printf("[CONFIG] ancien fichier %s efface\n", k_old[i]);
  }
}

void InitDisks(void)
{

  Sd_Report();
  if (LoadConfigFromSD()) Cfg_Backup("config.cfg");                 // copie de secours au démarrage
  if (LoadJoystickConfigFromSD()) Cfg_Backup("joyconfig.cfg");
  { extern void UsbDev_LoadPublic(void); UsbDev_LoadPublic(); }   // appareils USB mémorisés

  Disk_Drive.Name_Disk[0][255] = 0; // Just to be sure.
  Disk_Drive.Name_Disk[1][255] = 0; // Just to be sure.
  Disk_Drive.Name_Disk[2][255] = 0; // Just to be sure.
  Disk_Drive.Name_Disk[3][255] = 0; // Just to be sure.

  Disk_Drive.LastNumber_Accessed = 255; // No Last Accessed
  Disk_Drive.isFileAlreadyOpen = false;

  Disk_Drive.Error = false;
  Disk_Drive.LastAccessType = 255;

  return;
}


  IRAM_ATTR __attribute__((optimize("no-jump-tables"))) void ManagePeripherals_Write(uint16_t address, uint8_t value)
{
    if (address >= 0xFF90 && address <= 0xFFDF)
  {
  }


    uint8_t tmpvar8t;
    uint16_t tmpvar16t;

    // $FF02 / $FF22 adressent le DDR quand le bit 2 du registre de contrôle voisin ($FF01 / $FF03 /
    // $FF23) vaut 0. prises pour des données
    if ((address == 0xFF00 || address == 0xFF02 || address == 0xFF20 || address == 0xFF22) &&
        (rom[(uint16_t)(address + 1) - ROM_OFFSET] & 0x04) == 0)
    {
      g_pia_ddr[(address == 0xFF00) ? 0 : (address == 0xFF02 ? 1 : (address == 0xFF22 ? 2 : 3))] = value;
      if (address == 0xFF20)               // la sortie réelle change avec la direction
      {
        Bb_Write(rom[ROM_FF20], value);
        Mouse_DacWrite(Pia1A_Out());
        if (Cas_MotorOn()) Cas_DacWrite((uint8_t)(Pia1A_Out() & 0xFC), (uint32_t)g_emu_ticks);   // sortie cassette
        UpdateSoundOutput();
      }
      return;
    }

    switch (address)
    {
      case M_FF20:
        rom[ROM_FF20] = value;
        Bb_Write(value, g_pia_ddr[3]);   // bit 1 = sortie série (bit banger) ; selon le DDR
        Mouse_DacWrite(Pia1A_Out());     // interface haute résolution (charge au passage du CNA à 0)
        if (Cas_MotorOn()) Cas_DacWrite((uint8_t)(Pia1A_Out() & 0xFC), (uint32_t)g_emu_ticks);   // sortie cassette = CNA
        UpdateSoundOutput();
      break;
      case 0xFF21:
        // son bit 2 (DDR / données de $FF20) restait figé. Bits 7-6 = drapeaux en lecture seule
        // (6821). CA1 (détection de porteuse) / CA2 (moteur cassette) : pas d'effet émulé.
        rom[0xFF21 - ROM_OFFSET] = (uint8_t)((rom[0xFF21 - ROM_OFFSET] & 0xC0) | (value & 0x3F));
        // CA2 en sortie (bits 5-4 = 11) : bit 3 = relais du moteur cassette (virtuel)
        Cas_Motor((value & 0x38) == 0x38, (uint32_t)g_emu_ticks);
      break;
    case M_FF02:
        ManageKeyboardScan(value);
        rom[ROM_FF02] = value;
      break;
    case M_FF01:
        // FF01 tombaient dans le "default" et ne faisaient rien. FF01 (avec FF03) pilote
        // MultiplexerCB2_CA2, la sélection d'axe du comparateur joystick émulé (voir
        // ManagePeripherals_Read, case M_FF00)
        rom[ROM_FF01] = (rom[ROM_FF01] & 0b11000000) | (value & 0b00111111);
        PIA0_UpdateIRQ();
        Mouse_MuxChanged();
      break;
    case M_FF03:
        if ((value & 0b00000001) == 0)
        {
          sf.V_Synch_Int_Enabled = false;
        }
        else
        {
          sf.V_Synch_Int_Enabled = true;
        }
        // Bit7 (et bit6) sont des drapeaux matériels en LECTURE SEULE sur un vrai PIA 6821 — le CPU ne
        // peut jamais les modifier par écriture. Ne jamais écraser rom[ROM_FF03] au complet
        rom[ROM_FF03] = (rom[ROM_FF03] & 0b11000000) | (value & 0b00111111);
        Mouse_MuxChanged();
        if (g_pia_sync_emulated)
        {
          PIA0_UpdateIRQ();   // autoriser avec le drapeau déjà levé = IRQ (6821)
        }
      break;

            case M_FF23:
      // bits 7-6 = drapeaux en lecture seule (6821). CB1 = ligne CART reliée à l'horloge Q par une
      // cartouche de jeu, le drapeau (bit 7) est relevé à chaque cycle, donc toujours vu à 1
      rom[ROM_FF23] = (uint8_t)(value & 0x3F);
      { extern void Cart_FirqPublic(void); Cart_FirqPublic(); }   // drapeau CB1 (horloge Q ou IRQ du RS-232 Pak)
      UpdateSoundOutput();   // SNDEN (CB2) a pu changer
    break;

      // $FF92/$FF93 (et tout $FF90-$FFBF) : gime_write_register dans write8.

      // Vitesse CPU (SAM R1/R0). Sur le CoCo3, $FFD7 et $FFD9 donnent tous deux 1,79 MHz, $FFD6 et
      // $FFD8 reviennent à 0,89 MHz (POKE 65495,0 / 65497,0 des programmes CoCo 1/2). Retiré
      case 0xFFD7:
      case M_FFD9:
        sf.CPU_Speed = CPU_FAST;
        Oc_UpdateDivider();
      break;
      case 0xFFD6:
      case M_FFD8:
        sf.CPU_Speed = CPU_SLOW;
        Oc_UpdateDivider();
      break;
//---------------- Disk Related
   // contrôleur de disquette réaliste (voir Fdc_*)
   case M_FF40:
   case M_FF48:
   case M_FF49:
   case M_FF4A:
   case M_FF4B:
     Fdc_Write(address, value);
     break;

//------------ All the next logic is for videomodes
    case M_FF22:

      rom[ROM_FF22] = value;
      sf.Coco2GraphicMode = (sf.Coco2GraphicMode & 0b00000111) | (value & 0b11111000);
      UpdateSoundOutput();   // bit 1 = son 1 bit

      break;

      // In the next decode, no need to store data at address because it's only set reset. Address are
      // not readables
    case M_FFC0:
      sf.Coco2GraphicMode &=0b11111110;
    break;
    case M_FFC1:
      sf.Coco2GraphicMode |=0b00000001;
    break;
    case M_FFC2:
      sf.Coco2GraphicMode &=0b11111101;
    break;
    case M_FFC3:
      sf.Coco2GraphicMode |=0b00000010;
    break;

    case M_FFC4:
      sf.Coco2GraphicMode &=0b11111011;
    break;
    case M_FFC5:
      sf.Coco2GraphicMode |=0b00000100;
    break;
    //-------------------- Next are Address offset SET/RESET registers
    case M_FFC6:
      sf.Coco2VideoPageOffset_Registers &=0b11111110; // Clear the bit
    break;
    case M_FFC7:
    sf.Coco2VideoPageOffset_Registers |=0b00000001; // Set the bit
    break;
    case M_FFC8:
      sf.Coco2VideoPageOffset_Registers &=0b11111101; // Clear the bit
    break;
    case M_FFC9:
    sf.Coco2VideoPageOffset_Registers |=0b00000010; // Set the bit
    break;
    case M_FFCA:
      sf.Coco2VideoPageOffset_Registers &=0b11111011; // Clear the bit
    break;
    case M_FFCB:
    sf.Coco2VideoPageOffset_Registers |=0b00000100; // Set the bit
    break;
    case M_FFCC:
      sf.Coco2VideoPageOffset_Registers &=0b11110111; // Clear the bit
    break;
    case M_FFCD:
    sf.Coco2VideoPageOffset_Registers |=0b00001000; // Set the bit
    break;
    case M_FFCE:
      sf.Coco2VideoPageOffset_Registers &=0b11101111; // Clear the bit
    break;
    case M_FFCF:
    sf.Coco2VideoPageOffset_Registers |=0b00010000; // Set the bit
    break;
    case M_FFD0:
      sf.Coco2VideoPageOffset_Registers &=0b11011111; // Clear the bit
    break;
    case M_FFD1:
    sf.Coco2VideoPageOffset_Registers |=0b00100000; // Set the bit
    break;
    case M_FFD2:
      sf.Coco2VideoPageOffset_Registers &=0b10111111; // Clear the bit
    break;
    case M_FFD3:
    sf.Coco2VideoPageOffset_Registers |=0b01000000; // Set the bit
    break;

    case M_FFDE:
      gime_set_ty(false);   // mode ROM
    break;
    case M_FFDF:
      gime_set_ty(true);    // tout-RAM
    break;

      default:
      break;
    }

  }

void FillKeyboardMatrix(void)
{
  for (int loop1 = 0; loop1 != 8; loop1++)
  {
    for (int loop2 = 0; loop2 != 7; loop2++)
    {
      SCAN_Keyboard_Matrix[loop1][loop2] = 0;
    }
  }
  sf.AnyKeypress = false;

  for (uint8_t loop1 = 0; loop1 !=8; loop1++)
  {
    if (USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] & 0b01000000)
    SCAN_Keyboard_Matrix[loop1][6] = ((USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] & 0b01000000) ^ 0b01111111);

    if (USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] & 0b00100000)
        SCAN_Keyboard_Matrix[loop1][5] = ((USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] & 0b00100000) ^ 0b01111111);

    if (USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] & 0b00010000)
        SCAN_Keyboard_Matrix[loop1][4] = ((USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] & 0b00010000) ^ 0b01111111);

    if (USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] & 0b00001000)
        SCAN_Keyboard_Matrix[loop1][3] = ((USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] & 0b00001000) ^ 0b01111111);

    if (USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] & 0b00000100)
        SCAN_Keyboard_Matrix[loop1][2] = ((USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] & 0b00000100) ^ 0b01111111);

    if (USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] & 0b00000010)
        SCAN_Keyboard_Matrix[loop1][1] = ((USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] & 0b00000010) ^ 0b01111111);

    if (USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] & 0b00000001)
        SCAN_Keyboard_Matrix[loop1][0] = ((USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] & 0b00000001) ^ 0b01111111);
    if (USB_DEV_CONTROL.USB_CoCo_Key_Array[loop1] != 0)
    {
      sf.AnyKeypress = true;
    }
  }

}

  void ManageKeyboardScan(uint8_t value)
  {
    // TOUTES les colonnes sélectionnées (à 0). colonne trouvée (dans l'ordre 7..0) comptait
    const uint8_t Val1 = (uint8_t)(~value) & g_pia_ddr[1];
    rom[ROM_FF00] |= 0b01111111;   // aucune rangée active

    if (ReadCoCoButtons()==3 && !sf.is_JOY1_B1_WasPressed) // boutons relâchés
    {
      sf.is_LastKeyboardScanned = true;
      for (uint8_t col = 0; col != 8; col++)
      {
        if (!(Val1 & (1u << col))) continue;
        for (uint8_t row = 0; row != 7; row++)
        {
          if (SCAN_Keyboard_Matrix[col][row] != 0)
          {
            rom[ROM_FF00] &= SCAN_Keyboard_Matrix[col][row];
          }
        }
      }
    }
    else
    {
      sf.is_JOY1_B1_WasPressed=false;
      rom[ROM_FF00] &= 0x7f;
    }
  }

  //-------------------------------- VDG MAP

  // une seule ligne (lir = ligne du glyphe, 0-11) d'un caractère VDG.
  void DisplayVDGcharLine(uint8_t charNum, uint8_t lir, uint16_t Xpos, uint16_t Ypos)
  {
    uint8_t tmp[8];
    if (lir > 11) lir = 11;
    if (charNum >= 0x80)
    {
      const uint8_t on = LegacyPalSG((charNum >> 4) & 7);
      const uint8_t q = (lir < 6) ? (charNum >> 2) : charNum;
      for (int x = 0; x < 8; x++) tmp[x] = (q & ((x < 4) ? 2 : 1)) ? on : 0x00;
      vga->drawLineFromMemory8(Xpos, Ypos, tmp);
      return;
    }
    const bool lower  = ((sf.Coco2GraphicMode & 0x10) != 0) || g_lowercase_t1;
    const bool invert = (sf.Coco2GraphicMode & 0x20) != 0;
    const uint8_t light = invert ? s_txt_dark : s_txt_light, dark = invert ? s_txt_light : s_txt_dark;
    if (lower && charNum < 0x20)
    {
      const uint8_t bits = font_gimeChip[(0x60 + charNum) * 12 + lir];
      for (int x = 0; x < 8; x++) tmp[x] = (bits & (0x80 >> x)) ? dark : light;
    }
    else
    {
      const uint8_t *src = &cc2_VDG_MAP[charNum * 96 + lir * 8];
      for (int x = 0; x < 8; x++) tmp[x] = (src[x] == 0x1C) ? light : dark;
    }
    vga->drawLineFromMemory8(Xpos, Ypos, tmp);
  }

  void DisplayVDGchar(uint8_t charNum, uint16_t Xpos, uint16_t Ypos)
  {
    // particularités du GIME en texte 32 colonnes ($FF22, A/G=0) bit 4 : codes $00-$1F = vraies
    // minuscules (au lieu des majuscules en vidéo inverse du VDG) — OS-9 et BASIC (POKE) s'en servent
    // bit 5
    static uint8_t *s_lc = nullptr;              // 3 Ko en PSRAM
    static bool s_lc_ready = false;
    if (!s_lc_ready)
    {
#ifdef ESP_PLATFORM
      s_lc = (uint8_t *)heap_caps_malloc(32 * 96, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#endif
      if (!s_lc) s_lc = (uint8_t *)malloc(32 * 96);
      for (int c = 0; c < 32; c++)
        for (int r = 0; r < 12; r++)
        {
          const uint8_t bits = font_gimeChip[(0x60 + c) * 12 + r];
          for (int x = 0; x < 8; x++)
            s_lc[c * 96 + r * 8 + x] = (bits & (0x80 >> x)) ? 0x00 : 0x1C;
        }
      s_lc_ready = true;
    }
    const bool lower  = ((sf.Coco2GraphicMode & 0x10) != 0) || g_lowercase_t1;   // 3x : option menu L
    const bool invert = (sf.Coco2GraphicMode & 0x20) != 0;
    const uint8_t *src = (lower && charNum < 0x20) ? &s_lc[charNum * 96] : &cc2_VDG_MAP[charNum * 96];
    uint16_t YposLoop;
    uint8_t tmp[8];
    if (charNum >= 0x80)
    {
      // bloc semi-graphique SG4 (2 x 2 blocs de 4 x 6 points) : couleur = registre de palette (code >>
      // 4) & 7, fond noir.
      const uint8_t on = LegacyPalSG((charNum >> 4) & 7);
      for (YposLoop = 0; YposLoop != 12; YposLoop++)
      {
        const uint8_t q = (YposLoop < 6) ? (charNum >> 2) : charNum;   // bits 3-2 en haut, 1-0 en bas
        for (int x = 0; x < 8; x++) tmp[x] = (q & ((x < 4) ? 2 : 1)) ? on : 0x00;
        vga->drawLineFromMemory8(Xpos, Ypos + YposLoop, tmp);
      }
      return;
    }
    // texte, couleurs de la palette GIME (fond clair 0x1C de la table caractère 0x00), inversées si
    // $FF22 bit 5.
    const uint8_t light = invert ? s_txt_dark : s_txt_light, dark = invert ? s_txt_light : s_txt_dark;
    for (YposLoop = Ypos; YposLoop != Ypos + 12; YposLoop++, src += 8)
    {
      for (int x = 0; x < 8; x++) tmp[x] = (src[x] == 0x1C) ? light : dark;
      vga->drawLineFromMemory8(Xpos, YposLoop, tmp);
    }
  }

void line(int x0, int y0, int x1, int y1, int rgb)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;

    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;

    int err = dx + dy;

    while (true)
    {
        vga->dot(x0, y0, rgb);

        if (x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

  //--------------------------------------- USB STACK FOR COCO KEYBOARD MAPPING AND JOYSTICKS

// EspUsbHost envoie un évènement par touche pressée/relâchée, alors que UpdateKeyMap (inchangée plus
// bas) attend un "instantané" façon rapport USB HID Boot Keyboard
uint8_t HID_ModifierByte = 0;   // non static (lu par la saisie de note Maj+F9)
static uint8_t HID_KeysHeld[5] = {0, 0, 0, 0, 0};

static void AddHeldKey(uint8_t keycode)
{
  for (uint8_t i = 0; i != 5; i++)
  {
    if (HID_KeysHeld[i] == keycode) return;  // déjà enfoncée
  }
  for (uint8_t i = 0; i != 5; i++)
  {
    if (HID_KeysHeld[i] == 0)
    {
      HID_KeysHeld[i] = keycode;
      return;
    }
  }
  // 5 touches déjà enfoncées simultanément (rollover) : on ignore, comme le ferait un clavier USB Boot
  // Protocol en cas de dépassement.
}

static void RemoveHeldKey(uint8_t keycode)
{
  for (uint8_t i = 0; i != 5; i++)
  {
    if (HID_KeysHeld[i] == keycode) HID_KeysHeld[i] = 0;
  }
}

// Callback natif clavier EspUsbHost : remplace la lecture logicielle du Soft-Host (my_USB_PrintCB /
// USB_DEV_CONTROL.PORT_KEYBOARD).
static void my_USB_KeyboardCB(const EspUsbHostKeyboardEvent &event)
{
  HID_ModifierByte = event.modifiers;
  g_input_events++;                                             // écran de veille
  if (event.pressed && event.keycode != 0) Autorun_UserKey();   // une vraie touche annule l'AUTORUN

  if (event.keycode != 0)  // keycode == 0 : évènement modificateur seul (Shift/Ctrl/Alt/Gui)
  {
    if (event.pressed)
      AddHeldKey(event.keycode);
    else
      RemoveHeldKey(event.keycode);
  }

  uint8_t Snapshot[8] = {0};
  Snapshot[0] = HID_ModifierByte;
  for (uint8_t i = 0; i != 5; i++)
  {
    Snapshot[2 + i] = HID_KeysHeld[i];
  }

  UpdateKeyMap(Snapshot);

#ifdef DEBUG_ALL
// LogOut.printf("KBD addr=%u keycode=0x%02x pressed=%d mod=0x%02x ascii=0x%02x\n" event.address,
// event.keycode, event.pressed, event.modifiers, event.ascii)
#endif
}

// Fonctions utilitaires du dispatch manettes -- déclarées avant leur premier usage (Setup_USB les
// enregistre plus bas, my_USB_HIDInputCB les appelle juste après)
static inline bool Joy_PortAuto(uint8_t s)
{
  return !(g_kbjoy_port & (1u << s)) && !((Mouse_Mask() >> s) & 1) && !JoystickCfg[s].configured;
}
static uint8_t AssignJoystickAddress(uint8_t address)
{
  if (Joy_PortAuto(0) && (USB_JOY1_ADDRESS == 0 || USB_JOY1_ADDRESS == address)) { USB_JOY1_ADDRESS = address; return 1; }
  if (Joy_PortAuto(1) && (USB_JOY2_ADDRESS == 0 || USB_JOY2_ADDRESS == address)) { USB_JOY2_ADDRESS = address; return 2; }
  return 0; // aucun slot libre (2 manettes déjà associées)
}

// Construit le tableau 5 octets attendu par UpdateJoyMap (Data[0]=X Data[1]=Y, Data[4]
// bit4/bit5=Bouton1/Bouton2) à partir d'une config générique.
static void ApplyJoystickReport(const JoystickDeviceConfig &cfg, const EspUsbHostHIDInput &input, uint8_t usbNum)
{
  if (usbNum == 0) return;
  if (cfg.xByte >= input.length || cfg.yByte >= input.length ||
      cfg.fire1Byte >= input.length || cfg.fire2Byte >= input.length)
  {
    return;
  }

  uint8_t xVal = input.data[cfg.xByte];
  uint8_t yVal = input.data[cfg.yByte];
  if (cfg.xInvert) xVal = 0xFF - xVal;
  if (cfg.yInvert) yVal = 0xFF - yVal;

  uint8_t Data[5] = {0};
  Data[0] = xVal;
  Data[1] = yVal;
  if (input.data[cfg.fire1Byte] & cfg.fire1Mask) Data[4] |= 0x10;
  if (input.data[cfg.fire2Byte] & cfg.fire2Mask) Data[4] |= 0x20;

  UpdateJoyMap(Data, usbNum);
}

// ---- Suivi des périphériques HID connectés, pour l'assistant de
// configuration (menu F12 -> J)
#define HID_TRACK_MAX 8
#define HID_TRACK_MAX_BYTES 16
// une entrée par INTERFACE (adaptateurs à 2 manettes, ex. « Twin USB Joystick » 5500 : une interface
// par joueur)
static inline uint8_t Joy_Key(uint8_t addr, uint8_t iface)
{
  return iface == 0 ? addr : (uint8_t)(0x80 | ((addr & 0x1F) << 2) | (iface & 3));
}
// Interface retenue par le menu J dans les 4 bits hauts de « mode »
static inline bool Joy_IfaceOk(const JoystickDeviceConfig &c, uint8_t iface)
{
  const uint8_t n = (uint8_t)(c.mode >> 4);
  return n == 0 || (uint8_t)(n - 1) == iface;
}
struct HidTrackedDevice
{
  bool present;
  uint8_t address;     // clé (Joy_Key)
  uint8_t real;        // adresse USB réelle
  uint16_t vid, pid;
  uint8_t subclass; // 0x01 = Boot Interface (clavier/souris) -> exclu de la liste manettes
  uint8_t length;
  uint8_t data[HID_TRACK_MAX_BYTES];
  uint8_t lastChangedByte; // 0xFF = aucun changement depuis JoyWizard_ResetChangeDetector
};
static HidTrackedDevice HidTracked[HID_TRACK_MAX] = {};

uint8_t JoyWizard_GetDeviceCount(void)
{
  uint8_t n = 0;
  for (uint8_t i = 0; i != HID_TRACK_MAX; i++)
    if (HidTracked[i].present && HidTracked[i].subclass != 0x01) n++;
  return n;
}

bool JoyWizard_GetDeviceInfo(uint8_t index, uint16_t &vid, uint16_t &pid, uint8_t &address)
{
  uint8_t n = 0;
  for (uint8_t i = 0; i != HID_TRACK_MAX; i++)
  {
    if (HidTracked[i].present && HidTracked[i].subclass != 0x01)
    {
      if (n == index)
      {
        vid = HidTracked[i].vid;
        pid = HidTracked[i].pid;
        address = HidTracked[i].address;
        return true;
      }
      n++;
    }
  }
  return false;
}

bool JoyWizard_GetSnapshot(uint8_t address, uint8_t *outData, uint8_t *outLen, uint8_t *outChangedByte)
{
  for (uint8_t i = 0; i != HID_TRACK_MAX; i++)
  {
    if (HidTracked[i].present && HidTracked[i].address == address)
    {
      memcpy(outData, HidTracked[i].data, HidTracked[i].length);
      *outLen = HidTracked[i].length;
      *outChangedByte = HidTracked[i].lastChangedByte;
      return true;
    }
  }
  return false;
}

void JoyWizard_ResetChangeDetector(uint8_t address)
{
  for (uint8_t i = 0; i != HID_TRACK_MAX; i++)
    if (HidTracked[i].present && HidTracked[i].address == address)
      HidTracked[i].lastChangedByte = 0xFF;
}

// Libère l'association port Joystick1/Joystick2 <-> adresse USB à la déconnexion d'une manette, pour
// permettre son remplacement à chaud.
static void my_USB_DeviceDisconnectedCB(const EspUsbHostDeviceInfo &device)
{
  LogOut.printf("[USB] *** DEVICE DISCONNECTED *** addr=%u vid=0x%04x pid=0x%04x\n", device.address, device.vid, device.pid);
  // manette débranchée : son port revient au repos (centre, boutons relâchés)
  if (device.address == USB_JOY1_ADDRESS && !(g_kbjoy_port & 1) && !(Mouse_Mask() & 1))
  { USB_DEV_CONTROL.JOY1_X_AXIS = 128; USB_DEV_CONTROL.JOY1_Y_AXIS = 128; USB_DEV_CONTROL.JOY1_BUTT1 = 1; USB_DEV_CONTROL.JOY1_BUTT2 = 1; }
  if (device.address == USB_JOY2_ADDRESS && !(g_kbjoy_port & 2) && !(Mouse_Mask() & 2))
  { USB_DEV_CONTROL.JOY2_X_AXIS = 128; USB_DEV_CONTROL.JOY2_Y_AXIS = 128; USB_DEV_CONTROL.JOY2_BUTT1 = 1; USB_DEV_CONTROL.JOY2_BUTT2 = 1; }
  if (device.address == USB_JOY1_ADDRESS) USB_JOY1_ADDRESS = 0;
  if (device.address == USB_JOY2_ADDRESS) USB_JOY2_ADDRESS = 0;

  for (uint8_t i = 0; i != HID_TRACK_MAX; i++)
    if (HidTracked[i].present && HidTracked[i].real == device.address)
      HidTracked[i].present = false;
}

// Log connexion/déconnexion USB (basse fréquence, sans impact perf). dernier branchement (Wi-Fi
// démarré seulement quand l'USB est installé, cc3net.cpp)
static volatile uint32_t s_usb_last_conn_ms = 0;
bool Usb_BootSettled(void)
{
  const uint32_t now = millis();
  if (now > 15000u) return true;                     // au plus 15 s après le démarrage
  if (now < 2000u) return false;
  const uint32_t last = s_usb_last_conn_ms;
  if (!last) return now > 6000u;                     // aucun appareil vu : 6 s
  return now - last > 3000u;                         // 1,5 s d'attente des manettes + leur réclamation
}
static void my_USB_DeviceConnectedCB(const EspUsbHostDeviceInfo &device)
{
  s_usb_last_conn_ms = millis() | 1u;
  uint32_t f, l, m; Mem_DmaStats(f, l, m);
  LogOut.printf("[USB] *** DEVICE CONNECTED *** addr=%u vid=0x%04x pid=0x%04x ; RAM interne DMA libre %lu o (bloc %lu o, minimum %lu o)\n",
         device.address, device.vid, device.pid, (unsigned long)f, (unsigned long)l, (unsigned long)m);
  // option L : le voyant Verr. Maj. du clavier suit l'état mémorisé (g_pc_caps, sauvegardé sur SD). Le
  // clavier démarre voyant éteint
  extern bool g_lowercase_t1;
  if (g_lowercase_t1 && UsbHost)
  {
    EspUsbHostInterfaceInfo itf[8];
    const size_t ni = UsbHost->getInterfaces(device.address, itf, 8);
    for (size_t k = 0; k < ni && k < 8; k++)
    {
      if (itf[k].claimed && itf[k].interfaceClass == 3 && itf[k].interfaceSubClass == 1 && itf[k].interfaceProtocol == 1)
      {
        UsbHost->setKeyboardLeds(false, g_pc_caps, false, device.address);
        break;
      }
    }
  }
}

// onHIDInput se déclenche pour toute interface HID (clavier, souris manette), avant la classification
// interne de la lib -- utilisé ici pour contourner onGamepad, qui ne reconnaît pas les manettes…
static bool Mouse_FromRaw(uint8_t address, uint8_t iface, const uint8_t *d, size_t len);
static void my_USB_HIDInputCB(const EspUsbHostHIDInput &input)
{
  if (input.protocol == 2 && g_mouse_mode != 0)     // interface souris
  {
    Mouse_FromRaw(input.address, input.interfaceNumber, input.data, input.length);
    return;
  }
  // ---- Lecture réelle des manettes (contourne onGamepad, qui ne classe
  // pas ces modèles comme "gamepad" -- voir diagnostic ci-dessus) -------- Priorité à une config
  // utilisateur explicite (menu F12 -> J, persistée sur SD) associée à ce vid/pid
  {
    // première correspondance vid/pid trouvée -- donc si le MÊME device est configuré à la fois pour
    // Right ET Left (un seul joystick USB disponible, assigné aux deux slots), seul Right (premier
    // testé index…)
    bool anyMatched = false;
    for (uint8_t s = 0; s != 2; s++)
    {
      if (JoystickCfg[s].configured && JoystickCfg[s].vid == input.vid && JoystickCfg[s].pid == input.pid &&
          Joy_IfaceOk(JoystickCfg[s], input.interfaceNumber) &&
          !(g_kbjoy_port & (1u << s)) && !((Mouse_Mask() >> s) & 1))   // port pris par le clavier / la souris // bonne interface (manette 1 ou…)
      {
        anyMatched = true;
        // Slot explicitement configuré -> Right(0)=Joystick1, Left(1)=Joystick2.
        uint8_t usbNum = (s == JOY_SLOT_RIGHT) ? 1 : 2;
        ApplyJoystickReport(JoystickCfg[s], input, usbNum);
      }
    }

    // manette Xbox (360 / One / Series et compatibles) : rapport déjà converti par la pile USB (X, Y,
    // boutons A = tir 1, B = tir 2) ; le menu J peut le redéfinir.
    if (!anyMatched && input.protocol == 0xFF && (input.subclass == 0x47 || input.subclass == 0x5D) && input.length >= 8)
    {
      anyMatched = true;
      static uint8_t s_xbox_logged = 0;
      if (s_xbox_logged != input.address)
      {
        s_xbox_logged = input.address;
        LogOut.printf("[JOY] manette Xbox %s (%04x:%04x, adresse %u) : stick gauche ou croix = manette, A = tir 1, B = tir 2\n",
                      input.subclass == 0x47 ? "One / Series" : "360", input.vid, input.pid, input.address);
      }
      const uint8_t usbNum = AssignJoystickAddress(input.address);
      if (usbNum != 0)
      {
        JoystickDeviceConfig tmp = {};
        tmp.xByte = 0; tmp.yByte = 1;
        tmp.fire1Byte = 2; tmp.fire1Mask = 0x01;
        tmp.fire2Byte = 2; tmp.fire2Mask = 0x02;
        ApplyJoystickReport(tmp, input, usbNum);
      }
    }

    if (!anyMatched)
    {
      for (uint8_t b = 0; b != JOYSTICK_BUILTIN_COUNT; b++)
      {
        if (input.vid == JoystickBuiltins[b].vid && input.pid == JoystickBuiltins[b].pid && input.length >= 6)
        {
          uint8_t usbNum = AssignJoystickAddress(input.address);
          if (usbNum != 0)
          {
            JoystickDeviceConfig tmp = {};
            tmp.xByte = JoystickBuiltins[b].xByte;
            tmp.yByte = JoystickBuiltins[b].yByte;
            tmp.fire1Byte = JoystickBuiltins[b].btnByte;
            tmp.fire1Mask = JoystickBuiltins[b].fire1Mask;
            tmp.fire2Byte = JoystickBuiltins[b].btnByte;
            tmp.fire2Mask = JoystickBuiltins[b].fire2Mask;
            ApplyJoystickReport(tmp, input, usbNum);
          }
          break;
        }
      }
    }
  }

  // ---- Suivi pour l'assistant de configuration (liste + snapshot live)
  {
    HidTrackedDevice *t = nullptr;
    for (uint8_t i = 0; i != HID_TRACK_MAX; i++)
    {
      if (HidTracked[i].present && HidTracked[i].address == Joy_Key(input.address, input.interfaceNumber)) { t = &HidTracked[i]; break; }
    }
    if (t == nullptr)
    {
      for (uint8_t i = 0; i != HID_TRACK_MAX; i++)
      {
        if (!HidTracked[i].present)
        {
          t = &HidTracked[i];
          t->present = true;
          t->address = Joy_Key(input.address, input.interfaceNumber);
          t->real = input.address;
          t->vid = input.vid;
          t->pid = input.pid;
          t->subclass = input.subclass;
          t->length = 0;
          t->lastChangedByte = 0xFF;
          memset(t->data, 0, sizeof(t->data));
          break;
        }
      }
    }
    if (t != nullptr)
    {
      uint8_t tlen = (input.length < sizeof(t->data)) ? input.length : sizeof(t->data);

      // La détection "quel octet a changé" (utilisée par l'assistant de configuration pour identifier
      // un bouton/axe pressé) est limitée aux 14 premiers octets
      #define HID_CHANGE_DETECT_MAX_BYTE 14
      uint8_t detectLen = (tlen < HID_CHANGE_DETECT_MAX_BYTE) ? tlen : HID_CHANGE_DETECT_MAX_BYTE;
      for (uint8_t i = 0; i != detectLen; i++)
      {
        if (t->data[i] != input.data[i])
        {
          // NOTE : si plusieurs octets changent dans le même rapport, seul le dernier est retenu --
          // suffisant tant que l'utilisateur n'actionne qu'UNE chose à la fois, comme demandé par
          // l'assistant.
          t->lastChangedByte = i;
        }
      }
      t->length = tlen;
      memcpy(t->data, input.data, tlen);
    }
  }
}

// =====================================================================
// dernier fichier choisi pour chaque type de cartouche à ROM (ROM, Orchestra-90, Game Master, PSG) :
// ESP-CONFIG/cartlast.cfg.
// =====================================================================
static char s_cart_last[4][256];
static bool s_cart_last_dirty = false, s_cart_last_loaded = false;
static int CartLast_Index(uint8_t mode) { return mode == SLOT_ROM ? 0 : mode == SLOT_O90 ? 1 : mode == SLOT_GMC ? 2 : mode == SLOT_CPSG ? 3 : -1; }
static void CartLast_Load(void)
{
  if (s_cart_last_loaded) return;
  s_cart_last_loaded = true;
  memset(s_cart_last, 0, sizeof s_cart_last);
  File f = Cfg_OpenRead("cartlast.cfg", "cartlast.ccc");
  if (f) { if (f.read((uint8_t *)s_cart_last, sizeof s_cart_last) != sizeof s_cart_last) memset(s_cart_last, 0, sizeof s_cart_last); f.close(); }
  for (int i = 0; i < MPI_SLOTS; i++)                 // cartouches déjà en place
  {
    const int k = CartLast_Index(g_slotcfg.mode[i]);
    if (k >= 0 && !s_cart_last[k][0] && g_slotcfg.name[i][0]) snprintf(s_cart_last[k], 256, "%s", g_slotcfg.name[i]);
  }
}
const char *CartLast_Get(uint8_t mode)
{
  CartLast_Load();
  const int k = CartLast_Index(mode);
  return k >= 0 ? s_cart_last[k] : "";
}
void CartLast_Remember(uint8_t mode, const char *path)
{
  CartLast_Load();
  const int k = CartLast_Index(mode);
  if (k < 0 || !path || strcmp(s_cart_last[k], path) == 0) return;
  snprintf(s_cart_last[k], 256, "%s", path);
  s_cart_last_dirty = true;
}

// =====================================================================
// MENU USB DEVICES : appareils vus (en ligne + 8 derniers hors ligne, ESP-CONFIG/usbseen.cfg), clavier
// du CoCo, ports JOY (R) / (L)
// =====================================================================

struct UsbSeenRec
{
  uint8_t  used, type, pad, set;                   // set : réglages de l'assistant valides (map)
  uint8_t  autok, rsv[3];                          // autok : reconnue sans réglage (Xbox, manette connue)
  uint16_t vid, pid;
  uint32_t seq;                                    // ordre de dernière apparition
  char     name[26];
  uint8_t  rsv2[2];
  JoystickDeviceConfig map;
};
#define USEEN_MAX     16
#define USEEN_OFFLINE 8
struct UsbSeenFile
{
  char       magic[4];                             // "USB1"
  uint32_t   seq;
  uint16_t   kvid, kpid;                           // clavier du CoCo
  uint8_t    kset, rsv[3];
  char       kname[26];
  uint8_t    rsv2[2];
  UsbSeenRec rec[USEEN_MAX];
};
static UsbSeenFile s_us = {};
static bool s_us_dirty = false;
static bool s_joy_dirty = false;
static uint32_t s_us_online[USEEN_MAX / 32 + 1] = {};   // en ligne au dernier relevé (RAM)

struct UsbOnlineDev { uint8_t type, pad, npads, key, autok, nochan; uint16_t vid, pid; char name[26]; };   // nochan

static bool Usb_IsBuiltinPad(uint16_t vid, uint16_t pid)
{
  for (uint8_t b = 0; b != JOYSTICK_BUILTIN_COUNT; b++)
    if (JoystickBuiltins[b].vid == vid && JoystickBuiltins[b].pid == pid) return true;
  return false;
}

// Appareils en ligne (claviers, souris, manettes ; une entrée par manette d'un adaptateur double).
static int Usb_ScanOnline(UsbOnlineDev *o, int max)
{
  int k = 0;
  if (!UsbHost) return 0;
  EspUsbHostDeviceInfo dev[8];
  const size_t nd = UsbHost->getDevices(dev, 8);
  for (size_t i = 0; i < nd && i < 8; i++)
  {
    if (dev[i].isHub) continue;
    char nm[26];
    const char *pr = dev[i].product, *mf = dev[i].manufacturer;
    if (pr && pr[0]) snprintf(nm, sizeof nm, "%s", pr);
    else if (mf && mf[0]) snprintf(nm, sizeof nm, "%s", mf);
    else snprintf(nm, sizeof nm, "%04X:%04X", dev[i].vid, dev[i].pid);
    for (char *c = nm; *c; c++) { if (*c >= 'a' && *c <= 'z') *c = (char)(*c - 32); else if ((uint8_t)*c < 32 || (uint8_t)*c > 126) *c = ' '; }
    { // espaces du début et de la fin enlevés (" USB Gamepad ")
      char *b = nm; while (*b == ' ') b++;
      memmove(nm, b, strlen(b) + 1);
      for (size_t L = strlen(nm); L && nm[L - 1] == ' '; L--) nm[L - 1] = 0;
      if (!nm[0]) snprintf(nm, sizeof nm, "%04X:%04X", dev[i].vid, dev[i].pid);
    }
    EspUsbHostInterfaceInfo itf[8];
    const size_t ni = UsbHost->getInterfaces(dev[i].address, itf, 8);
    const uint8_t drain = UsbHost->cocoDrainMask(dev[i].address);
    bool kb = false, ms = false;
    bool msNc = false; uint8_t padNc[4]; uint8_t nNc = 0;   // interfaces en attente d'un canal
    bool noMem = false;                                       // ou abandonnées faute de RAM DMA
    uint8_t pads[4]; uint8_t np = 0; bool xbox = false;
    uint8_t hidPad[4]; uint8_t nh = 0;               // interfaces HID « manette »
    uint8_t hidUnk[4]; uint8_t nu = 0;               // interfaces HID sans collection connue
    for (size_t j = 0; j < ni && j < 8; j++)
    {
      const EspUsbHostInterfaceInfo &f = itf[j];
      if (!f.claimed)
      {
        // manette / souris sans canal (USB plein) : listée « NO CHANNEL »
        if (f.interfaceClass == 3 && f.claimAttempted && (f.claimResult == ESP_ERR_NOT_SUPPORTED || f.claimResult == ESP_ERR_NO_MEM))
        {
          const uint8_t mm = UsbHost->cocoIfaceMask(dev[i].address, f.number);
          if (f.claimResult == ESP_ERR_NO_MEM) noMem = true;   // manque de RAM DMA (pas de canal)
          if (f.interfaceProtocol == 2 || ((mm & 0x02) && !(mm & 0x04))) msNc = true;
          else if (((mm & 0x04) || f.interfaceProtocol == 0) && f.interfaceProtocol != 1 && nNc < 4) padNc[nNc++] = f.number;
        }
        continue;
      }
      const uint8_t m = UsbHost->cocoIfaceMask(dev[i].address, f.number);
      if (f.interfaceClass == 3)
      {
        if (f.number < 8 && ((drain >> f.number) & 1)) continue;          // lue seulement pour débloquer l'adaptateur
        if (f.interfaceProtocol == 1 || ((m & 0x01) && !(m & 0x04))) kb = true;        // manette + clavier = manette
        else if (f.interfaceProtocol == 2 || ((m & 0x02) && !(m & 0x04))) ms = true;
        else if (m & 0x04) { if (nh < 4) hidPad[nh++] = f.number; }      // collection joystick / gamepad
        else if (m == 0 && nu < 4) hidUnk[nu++] = f.number;               // filtre HID coupé : inconnue
      }
      else if (f.interfaceClass == 0xFF && (f.interfaceSubClass == 0x5D || f.interfaceSubClass == 0x47) && np < 4)
      {
        pads[np++] = f.number;                                            // Xbox 360 / One / Series
        xbox = true;
      }
    }
    // une manette Xbox (ou en mode X) n'est comptée qu'une fois, même si elle présente aussi une
    // interface HID (8BitDo : 2 manettes affichées pour une).
    if (!xbox)
    {
      for (uint8_t j = 0; j < nh && np < 4; j++) pads[np++] = hidPad[j];
      if (!np && !kb && !ms) for (uint8_t j = 0; j < nu && np < 1; j++) pads[np++] = hidUnk[j];
    }
    if (kb && k < max)
    {
      UsbOnlineDev &e = o[k++];
      e = {}; e.type = UDEV_KBD; e.vid = dev[i].vid; e.pid = dev[i].pid; memcpy(e.name, nm, sizeof nm);
    }
    if ((ms || msNc) && k < max)
    {
      UsbOnlineDev &e = o[k++];
      e = {}; e.type = UDEV_MOUSE; e.vid = dev[i].vid; e.pid = dev[i].pid; e.autok = 1; e.nochan = ms ? 0 : (noMem ? 2 : 1); memcpy(e.name, nm, sizeof nm);
    }
    if (!np && !xbox) for (uint8_t j = 0; j < nNc && k < max; j++)   // manette sans canal
    {
      UsbOnlineDev &e = o[k++];
      e = {}; e.type = UDEV_PAD; e.vid = dev[i].vid; e.pid = dev[i].pid; e.pad = (uint8_t)(padNc[j] + 1); e.npads = nNc;
      e.autok = Usb_IsBuiltinPad(dev[i].vid, dev[i].pid) ? 1 : 0; e.nochan = noMem ? 2 : 1;
      memcpy(e.name, nm, sizeof nm);
    }
    for (uint8_t p = 0; p < np && k < max; p++)
    {
      UsbOnlineDev &e = o[k++];
      e = {}; e.type = UDEV_PAD; e.vid = dev[i].vid; e.pid = dev[i].pid; e.pad = (uint8_t)(pads[p] + 1); e.npads = np;
      e.key = Joy_Key(dev[i].address, pads[p]);
      e.autok = (xbox || Usb_IsBuiltinPad(dev[i].vid, dev[i].pid)) ? 1 : 0;
      memcpy(e.name, nm, sizeof nm);
    }
  }
  return k;
}

static int Usb_FindRec(uint8_t type, uint16_t vid, uint16_t pid, uint8_t pad)
{
  for (int i = 0; i < USEEN_MAX; i++)
  {
    const UsbSeenRec &r = s_us.rec[i];
    if (r.used && r.type == type && r.vid == vid && r.pid == pid && (type != UDEV_PAD || r.pad == pad)) return i;
  }
  return -1;
}
static bool Usb_RecOnline(int i) { return (s_us_online[i >> 5] >> (i & 31)) & 1u; }

// Relevé : les appareils en ligne sont ajoutés / mis à jour dans la mémoire.
static int Usb_Refresh(UsbOnlineDev *on, int max)
{
  const int n = Usb_ScanOnline(on, max);
  uint32_t now[USEEN_MAX / 32 + 1] = {};
  for (int k = 0; k < n; k++)
  {
    int i = Usb_FindRec(on[k].type, on[k].vid, on[k].pid, on[k].pad);
    if (i < 0)
    {
      for (int j = 0; j < USEEN_MAX && i < 0; j++) if (!s_us.rec[j].used) i = j;
      if (i < 0)                                     // plein : le plus ancien hors ligne laisse sa place
      {
        uint32_t best = 0xFFFFFFFFu;
        for (int j = 0; j < USEEN_MAX; j++)
          if (!Usb_RecOnline(j) && !((now[j >> 5] >> (j & 31)) & 1u) && s_us.rec[j].seq < best) { best = s_us.rec[j].seq; i = j; }
      }
      if (i < 0) continue;
      UsbSeenRec &r = s_us.rec[i];
      r = {};
      r.used = 1; r.type = on[k].type; r.vid = on[k].vid; r.pid = on[k].pid; r.pad = on[k].pad;
      s_us_dirty = true;
    }
    UsbSeenRec &r = s_us.rec[i];
    if (!Usb_RecOnline(i) || strcmp(r.name, on[k].name) != 0 || r.autok != on[k].autok)
    {
      r.seq = ++s_us.seq;
      memcpy(r.name, on[k].name, sizeof r.name);
      r.autok = on[k].autok;
      s_us_dirty = true;
    }
    now[i >> 5] |= 1u << (i & 31);
  }
  memcpy(s_us_online, now, sizeof now);
  // clavier du CoCo : le premier clavier vu devient celui du CoCo
  if (!s_us.kset)
    for (int k = 0; k < n; k++)
      if (on[k].type == UDEV_KBD)
      {
        s_us.kset = 1; s_us.kvid = on[k].vid; s_us.kpid = on[k].pid;
        memcpy(s_us.kname, on[k].name, sizeof s_us.kname);
        s_us_dirty = true;
        break;
      }
  return n;
}

static void Usb_ViewFromRec(int i, UsbDevView &d, const UsbOnlineDev *on, int non)
{
  const UsbSeenRec &r = s_us.rec[i];
  d = {};
  d.type = r.type; d.vid = r.vid; d.pid = r.pid; d.pad = r.pad;
  memcpy(d.name, r.name, sizeof d.name);
  d.online = Usb_RecOnline(i);
  d.tag = r.type == UDEV_KBD ? UTAG_KBD : (r.set ? UTAG_SET : ((r.autok || r.type == UDEV_MOUSE) ? UTAG_AUTO : UTAG_NEW));
  for (int k = 0; k < non; k++)
    if (on[k].type == r.type && on[k].vid == r.vid && on[k].pid == r.pid && on[k].pad == r.pad)
    { d.key = on[k].key; d.npads = on[k].npads; d.nochan = on[k].nochan; break; }
  if (!d.npads && r.type == UDEV_PAD && r.pad > 1) d.npads = 2;
}

bool UsbDev_SameDev(const UsbDevView &a, const UsbDevView &b)
{
  return a.type == b.type && a.vid == b.vid && a.pid == b.pid && (a.type != UDEV_PAD || a.pad == b.pad);
}

int UsbDev_List(UsbDevView *out, int max, int *nOnline)
{
  UsbOnlineDev on[12];
  const int non = Usb_Refresh(on, 12);
  int k = 0;
  // en ligne, groupés par type (clavier, manette, souris)
  static const uint8_t k_order[3] = { UDEV_KBD, UDEV_PAD, UDEV_MOUSE };
  for (int t = 0; t < 3; t++)
    for (int i = 0; i < USEEN_MAX && k < max; i++)
      if (s_us.rec[i].used && s_us.rec[i].type == k_order[t] && Usb_RecOnline(i)) Usb_ViewFromRec(i, out[k++], on, non);
  if (nOnline) *nOnline = k;
  // hors ligne : les 8 plus récents, groupés par type
  int idx[USEEN_MAX], m = 0;
  for (int i = 0; i < USEEN_MAX; i++) if (s_us.rec[i].used && !Usb_RecOnline(i)) idx[m++] = i;
  for (int a = 0; a < m; a++) for (int b = a + 1; b < m; b++)
    if (s_us.rec[idx[b]].seq > s_us.rec[idx[a]].seq) { const int t = idx[a]; idx[a] = idx[b]; idx[b] = t; }
  if (m > USEEN_OFFLINE) m = USEEN_OFFLINE;
  for (int t = 0; t < 3; t++)
    for (int a = 0; a < m && k < max; a++)
      if (s_us.rec[idx[a]].type == k_order[t]) Usb_ViewFromRec(idx[a], out[k++], on, non);
  return k;
}

void UsbDev_Forget(const UsbDevView &d)
{
  const int i = Usb_FindRec(d.type, d.vid, d.pid, d.pad);
  if (i < 0 || Usb_RecOnline(i)) return;
  s_us.rec[i] = {};
  s_us_dirty = true;
}

// ---- ports JOY : 0 = droit (R), 1 = gauche (L)
uint8_t UsbPort_Kind(uint8_t p)
{
  if (g_kbjoy_port & (1u << p)) return UPORT_KBD;
  if ((Mouse_Mask() >> p) & 1) return UPORT_MOUSE;
  if (JoystickCfg[p].configured) return UPORT_PAD;
  return UPORT_AUTO;
}
static bool Usb_FirstOnline(uint8_t type, UsbDevView &d)
{
  UsbOnlineDev on[12];
  const int non = Usb_Refresh(on, 12);
  for (int i = 0; i < USEEN_MAX; i++)
    if (s_us.rec[i].used && s_us.rec[i].type == type && Usb_RecOnline(i)) { Usb_ViewFromRec(i, d, on, non); return true; }
  return false;
}
bool UsbKbd_Get(UsbDevView &d)
{
  UsbOnlineDev on[12];
  const int non = Usb_Refresh(on, 12);
  if (!s_us.kset) return false;
  const int i = Usb_FindRec(UDEV_KBD, s_us.kvid, s_us.kpid, 0);
  if (i >= 0) { Usb_ViewFromRec(i, d, on, non); return true; }
  d = {}; d.type = UDEV_KBD; d.tag = UTAG_KBD; d.vid = s_us.kvid; d.pid = s_us.kpid;
  memcpy(d.name, s_us.kname, sizeof d.name);
  return true;
}
bool UsbPort_Device(uint8_t p, UsbDevView &d)
{
  d = {};
  switch (UsbPort_Kind(p))
  {
  case UPORT_KBD:   return UsbKbd_Get(d);
  case UPORT_MOUSE:
    if (Usb_FirstOnline(UDEV_MOUSE, d)) return true;
    d.type = UDEV_MOUSE; snprintf(d.name, sizeof d.name, "USB MOUSE"); return true;
  case UPORT_PAD:
  {
    UsbOnlineDev on[12];
    const int non = Usb_Refresh(on, 12);
    const JoystickDeviceConfig &c = JoystickCfg[p];
    uint8_t pad = (uint8_t)(c.mode >> 4);
    int i = Usb_FindRec(UDEV_PAD, c.vid, c.pid, pad ? pad : 1);
    if (i < 0 && !pad)
      for (int j = 0; j < USEEN_MAX && i < 0; j++)
        if (s_us.rec[j].used && s_us.rec[j].type == UDEV_PAD && s_us.rec[j].vid == c.vid && s_us.rec[j].pid == c.pid) i = j;
    if (i >= 0) { Usb_ViewFromRec(i, d, on, non); return true; }
    d.type = UDEV_PAD; d.vid = c.vid; d.pid = c.pid; d.pad = pad; d.tag = UTAG_SET;
    snprintf(d.name, sizeof d.name, "%04X:%04X", c.vid, c.pid);
    return true;
  }
  default: return false;
  }
}

// Choix pour un port : AUTO, le clavier, la souris, chaque manette en ligne et l'appareil affecté s'il
// est hors ligne.
int UsbPort_Choices(uint8_t p, UsbDevView *out, int max)
{
  int k = 0;
  out[k] = {}; snprintf(out[k].name, sizeof out[k].name, "ANY GAMEPAD (AUTO)"); out[k].online = true; k++;
  UsbDevView cur;
  const bool hasCur = UsbPort_Device(p, cur);
  if (hasCur && !cur.online && k < max) out[k++] = cur;
  UsbDevView all[24];
  int non = 0;
  const int n = UsbDev_List(all, 24, &non);
  (void)n;
  bool kbDone = false, msDone = false;
  for (int i = 0; i < non && k < max; i++)
  {
    if (all[i].type == UDEV_KBD)
    {
      if (kbDone) continue;
      kbDone = true;
      UsbDevView kd;
      if (UsbKbd_Get(kd) && kd.online) out[k++] = kd; else out[k++] = all[i];   // le clavier du CoCo s'il est là
    }
    else if (all[i].type == UDEV_MOUSE)
    {
      if (msDone) continue;
      msDone = true;
      out[k++] = all[i];
    }
    else out[k++] = all[i];
  }
  return k;
}

static void Usb_DefaultMap(const UsbDevView &d, JoystickDeviceConfig &c)
{
  c = {};
  c.xByte = c.yByte = c.fire1Byte = c.fire2Byte = 0xFF;   // NEW : rien tant que l'assistant n'a pas réglé la manette
  for (uint8_t b = 0; b != JOYSTICK_BUILTIN_COUNT; b++)
    if (JoystickBuiltins[b].vid == d.vid && JoystickBuiltins[b].pid == d.pid)
    {
      c.xByte = JoystickBuiltins[b].xByte; c.yByte = JoystickBuiltins[b].yByte;
      c.fire1Byte = c.fire2Byte = JoystickBuiltins[b].btnByte;
      c.fire1Mask = JoystickBuiltins[b].fire1Mask; c.fire2Mask = JoystickBuiltins[b].fire2Mask;
      return;
    }
  if (d.tag == UTAG_AUTO)                                  // Xbox : rapport déjà converti (X, Y, A, B)
  {
    c.xByte = 0; c.yByte = 1; c.fire1Byte = 2; c.fire1Mask = 0x01; c.fire2Byte = 2; c.fire2Mask = 0x02;
  }
}

void UsbPort_Set(uint8_t p, const UsbDevView &d)
{
  // le port redevient AUTO, puis reçoit l'appareil choisi
  g_kbjoy_port &= (uint8_t)~(1u << p);
  if ((Mouse_Mask() >> p) & 1) g_mouse_mode = Mouse_ModeFor((uint8_t)(Mouse_Mask() & ~(1u << p)), g_mouse_hires_pref);   // l'autre port garde la souris
  if (JoystickCfg[p].configured) { JoystickCfg[p].configured = false; s_joy_dirty = true; }
  if (p == 0) USB_JOY1_ADDRESS = 0; else USB_JOY2_ADDRESS = 0;
  switch (d.type)
  {
  case UDEV_KBD:
    g_kbjoy_port |= (uint8_t)(1u << p);
    break;
  case UDEV_MOUSE:
    if (Mouse_Mask() == 0) g_mouse_hires_pref = false;   // nouvelle souris en basse résolution (souris Tandy, OS-9 par défaut)
    g_mouse_mode = Mouse_ModeFor((uint8_t)(Mouse_Mask() | (1u << p)), g_mouse_hires_pref);   // souris possible sur les deux ports
    break;
  case UDEV_PAD:
  {
    JoystickDeviceConfig c;
    const int i = Usb_FindRec(UDEV_PAD, d.vid, d.pid, d.pad);
    if (i >= 0 && s_us.rec[i].set) c = s_us.rec[i].map; else Usb_DefaultMap(d, c);
    c.configured = true;
    c.vid = d.vid; c.pid = d.pid;
    c.mode = (uint8_t)((c.mode & 0x0F) | ((d.pad & 0x0F) << 4));
    JoystickCfg[p] = c;
    s_joy_dirty = true;
    break;
  }
  default: break;
  }
  LogOut.printf("[USB] port %s : %s %s\n", p == 0 ? "JOY (R)" : "JOY (L)",
                d.type == UDEV_KBD ? "clavier" : d.type == UDEV_MOUSE ? "souris" : d.type == UDEV_PAD ? "manette" : "AUTO", d.name);
}

// Après l'assistant : réglages du port -> appareil (SET), et aussi l'autre port s'il a le même
// appareil.
void UsbPort_StoreMap(uint8_t p)
{
  const JoystickDeviceConfig &c = JoystickCfg[p];
  if (!c.configured) return;
  const uint8_t pad = (uint8_t)(c.mode >> 4);
  int i = Usb_FindRec(UDEV_PAD, c.vid, c.pid, pad ? pad : 1);
  if (i >= 0)
  {
    s_us.rec[i].set = 1;
    s_us.rec[i].map = c;
    s_us_dirty = true;
  }
  const uint8_t o = (uint8_t)(1 - p);
  if (JoystickCfg[o].configured && JoystickCfg[o].vid == c.vid && JoystickCfg[o].pid == c.pid && (JoystickCfg[o].mode >> 4) == pad)
    JoystickCfg[o] = c;
  s_joy_dirty = true;
}

int UsbKbd_Choices(UsbDevView *out, int max)
{
  UsbDevView all[24];
  int non = 0;
  UsbDev_List(all, 24, &non);
  int k = 0;
  for (int i = 0; i < non && k < max; i++) if (all[i].type == UDEV_KBD) out[k++] = all[i];
  return k;
}
void UsbKbd_Set(const UsbDevView &d)
{
  if (d.type != UDEV_KBD) return;
  s_us.kset = 1; s_us.kvid = d.vid; s_us.kpid = d.pid;
  memcpy(s_us.kname, d.name, sizeof s_us.kname);
  s_us_dirty = true;
}
bool UsbKbd_Changed(void)
{
  UsbDevView d;
  if (!UsbKbd_Get(d) || d.online) return false;
  UsbDevView c[4];
  return UsbKbd_Choices(c, 4) > 0;
}

// ---- fichier ESP-CONFIG/usbseen.cfg
static void UsbDev_Load(void)
{
  memset(&s_us, 0, sizeof s_us);
  File f = Cfg_OpenRead("usbseen.cfg", "usbseen.ccc");
  if (f)
  {
    UsbSeenFile t;
    if (f.read((uint8_t *)&t, sizeof t) == sizeof t && memcmp(t.magic, "USB1", 4) == 0) s_us = t;
    f.close();
  }
  memcpy(s_us.magic, "USB1", 4);
  for (uint8_t p = 0; p < 2; p++)
  {
    const JoystickDeviceConfig &c = JoystickCfg[p];
    if (!c.configured || c.xByte == 0xFF) continue;
    const uint8_t pad = (uint8_t)(c.mode >> 4) ? (uint8_t)(c.mode >> 4) : 1;
    if (Usb_FindRec(UDEV_PAD, c.vid, c.pid, pad) >= 0) continue;
    for (int j = 0; j < USEEN_MAX; j++)
      if (!s_us.rec[j].used)
      {
        UsbSeenRec &r = s_us.rec[j];
        r = {};
        r.used = 1; r.type = UDEV_PAD; r.vid = c.vid; r.pid = c.pid; r.pad = pad; r.set = 1; r.map = c;
        r.seq = ++s_us.seq;
        snprintf(r.name, sizeof r.name, "%04X:%04X", c.vid, c.pid);
        s_us_dirty = true;
        break;
      }
  }
  int n = 0;
  for (int j = 0; j < USEEN_MAX; j++) if (s_us.rec[j].used) n++;
  LogOut.printf("[USB] usbseen.cfg : %d appareil(s) memorise(s), clavier du CoCo %s\n", n, s_us.kset ? s_us.kname : "(aucun)");
}
void UsbDev_LoadPublic(void) { UsbDev_Load(); }
void UsbDev_SaveIfDirty(void)
{
  if (s_us_dirty)
  {
    s_us_dirty = false;
    // seulement les 8 plus récents hors ligne
    int idx[USEEN_MAX], m = 0;
    for (int i = 0; i < USEEN_MAX; i++) if (s_us.rec[i].used && !Usb_RecOnline(i)) idx[m++] = i;
    for (int a = 0; a < m; a++) for (int b = a + 1; b < m; b++)
      if (s_us.rec[idx[b]].seq > s_us.rec[idx[a]].seq) { const int t = idx[a]; idx[a] = idx[b]; idx[b] = t; }
    for (int a = USEEN_OFFLINE; a < m; a++) s_us.rec[idx[a]] = {};
    const bool ok = Cfg_SafeWrite("usbseen.cfg", (const uint8_t *)&s_us, sizeof s_us);
    if (!ok) LogOut.println("[USB] usbseen.cfg : ECHEC de la sauvegarde");
  }
  if (s_joy_dirty)
  {
    s_joy_dirty = false;
    SaveJoystickConfigToSD();
  }
  if (s_cart_last_dirty)
  {
    s_cart_last_dirty = false;
    Cfg_SafeWrite("cartlast.cfg", (const uint8_t *)s_cart_last, sizeof s_cart_last);
  }
}

// ---------------------------------------------------------------------------
// lecture de la souris par NOTRE analyse du descripteur de rapport HID. La lib retombe sur le format «
// boot » (boutons, X, Y en octets) quand elle ne reconnaît pas la disposition
// ---------------------------------------------------------------------------
void Mouse_UsbEvent(int16_t dx, int16_t dy, uint8_t buttons);
struct MouseField { uint16_t off; uint8_t size; bool sgn; bool ok; };
struct MouseLayout
{
  bool     valid;
  uint8_t  address, iface;
  uint8_t  reportId;            // 0 = pas de n° de rapport
  MouseField x, y, btn[3];
};
static MouseLayout s_mlayout[4];

static int32_t Hid_GetField(const uint8_t *d, size_t len, const MouseField &f)
{
  if (!f.ok || f.size == 0 || f.size > 16) return 0;
  uint32_t v = 0;
  for (uint8_t i = 0; i < f.size; i++)
  {
    const uint32_t bit = f.off + i;
    if ((bit >> 3) >= len) break;
    if (d[bit >> 3] & (1u << (bit & 7))) v |= (1u << i);
  }
  if (f.sgn && (v & (1u << (f.size - 1)))) return (int32_t)v - (int32_t)(1u << f.size);
  return (int32_t)v;
}

// Analyse minimale d'un descripteur de rapport HID : champs d'entrée X/Y relatifs (page 1, usages
// $30/$31) et boutons 1-3 (page 9).
bool Mouse_ParseDescriptor(uint8_t address, uint8_t iface, const uint8_t *d, uint16_t len)
{
  MouseLayout L = {};
  L.address = address; L.iface = iface;
  uint16_t upage = 0, rsize = 0, rcount = 0; int32_t lmin = 0; uint8_t rid = 0;
  static uint16_t bitpos[256];                      // position de bit par n° de rapport
  memset(bitpos, 0, sizeof bitpos);
  uint32_t usages[16]; uint8_t nus = 0; uint32_t umin = 0, umax = 0; bool hasRange = false;
  struct { uint16_t upage, rsize, rcount; int32_t lmin; uint8_t rid; } stack[4]; uint8_t sp = 0;
  bool anyId = false, found = false;
  for (uint16_t i = 0; i < len; )
  {
    const uint8_t b = d[i];
    if (b == 0xFE) { i += 3 + (i + 1 < len ? d[i + 1] : 0); continue; }   // élément long
    uint8_t sz = b & 3; if (sz == 3) sz = 4;
    uint32_t val = 0;
    for (uint8_t k = 0; k < sz && i + 1 + k < len; k++) val |= (uint32_t)d[i + 1 + k] << (8 * k);
    int32_t sval = (sz == 1) ? (int8_t)val : (sz == 2) ? (int16_t)val : (int32_t)val;
    const uint8_t type = (b >> 2) & 3, tag = b >> 4;
    i += 1 + sz;
    if (type == 1)                                  // global
    {
      switch (tag)
      {
        case 0: upage = (uint16_t)val; break;
        case 1: lmin = sval; break;
        case 7: rsize = (uint16_t)val; break;
        case 8: rid = (uint8_t)val; anyId = true; break;
        case 9: rcount = (uint16_t)val; break;
        case 10: if (sp < 4) { stack[sp].upage = upage; stack[sp].rsize = rsize; stack[sp].rcount = rcount; stack[sp].lmin = lmin; stack[sp].rid = rid; sp++; } break;
        case 11: if (sp) { sp--; upage = stack[sp].upage; rsize = stack[sp].rsize; rcount = stack[sp].rcount; lmin = stack[sp].lmin; rid = stack[sp].rid; } break;
        default: break;
      }
    }
    else if (type == 2)                             // local
    {
      if (tag == 0 && nus < 16) usages[nus++] = (sz == 4) ? val : ((uint32_t)upage << 16) | val;
      else if (tag == 1) { umin = (sz == 4) ? val : ((uint32_t)upage << 16) | val; hasRange = true; }
      else if (tag == 2) { umax = (sz == 4) ? val : ((uint32_t)upage << 16) | val; hasRange = true; }
    }
    else if (type == 0)                             // main
    {
      if (tag == 8)                                 // Input
      {
        const bool constant = (val & 1) != 0, relative = (val & 4) != 0;
        for (uint16_t f = 0; f < rcount; f++)
        {
          uint32_t u = 0;
          if (hasRange) u = umin + f;
          else if (nus) u = usages[f < nus ? f : nus - 1];
          const uint16_t off = bitpos[rid];
          if (!constant && !found)
          {
            MouseField fld = { off, (uint8_t)rsize, lmin < 0, true };
            const uint16_t pg = (uint16_t)(u >> 16), us = (uint16_t)u;
            if (pg == 0x01 && us == 0x30 && relative) { L.x = fld; L.reportId = rid; }
            else if (pg == 0x01 && us == 0x31 && relative && L.x.ok && L.reportId == rid) L.y = fld;
            else if (pg == 0x09 && us >= 1 && us <= 3) { L.btn[us - 1] = fld; L.btn[us - 1].sgn = false; }
          }
          bitpos[rid] = (uint16_t)(bitpos[rid] + rsize);
        }
        if (L.x.ok && L.y.ok) found = true;
      }
      else if (tag == 9 || tag == 11) { /* Output / Feature : pas dans les rapports d'entrée */ }
      nus = 0; hasRange = false; umin = umax = 0;    // les locaux s'effacent après un élément main
    }
  }
  if (!found) return false;
  (void)anyId;
  // Décalage du n° de rapport (1 octet en tête du rapport)
  const uint16_t idbits = L.reportId ? 8 : 0;
  L.x.off += idbits; L.y.off += idbits;
  for (int k = 0; k < 3; k++) if (L.btn[k].ok) L.btn[k].off += idbits;
  L.valid = true;
  for (int k = 0; k < 4; k++)
    if (!s_mlayout[k].valid || (s_mlayout[k].address == address && s_mlayout[k].iface == iface))
    { s_mlayout[k] = L; break; }
  LogOut.printf("[MOUSE] descripteur addr=%u iface=%u : rapport %u, X bit %u/%u, Y bit %u/%u, boutons bits %u,%u,%u\n",
                address, iface, L.reportId, L.x.off, L.x.size, L.y.off, L.y.size, L.btn[0].off, L.btn[1].off, L.btn[2].off);
  return true;
}

static const MouseLayout *Mouse_FindLayout(uint8_t address, uint8_t iface)
{
  for (int k = 0; k < 4; k++)
    if (s_mlayout[k].valid && s_mlayout[k].address == address && s_mlayout[k].iface == iface) return &s_mlayout[k];
  return nullptr;
}

static uint8_t s_mouse_self_addr = 0;     // adresse décodée par nous (onMouse ignoré)

// Rend true si le rapport a été traité ici.
static bool Mouse_FromRaw(uint8_t address, uint8_t iface, const uint8_t *d, size_t len)
{
  const MouseLayout *L = Mouse_FindLayout(address, iface);
  if (L)
  {
    if (L->reportId && (len == 0 || d[0] != L->reportId)) return true;   // autre rapport du même appareil
    uint8_t b = 0;
    for (int k = 0; k < 3; k++) if (Hid_GetField(d, len, L->btn[k])) b |= (uint8_t)(1u << k);
    s_mouse_self_addr = address;
    Mouse_UsbEvent((int16_t)Hid_GetField(d, len, L->x), (int16_t)Hid_GetField(d, len, L->y), b);
    return true;
  }
  // Repli sans descripteur : [ID][boutons][X 12 bits][Y 12 bits][molette]...
  if (len == 7 && d[0] >= 1 && d[0] <= 15 && (d[1] & 0xF8) == 0)
  {
    int16_t x = (int16_t)(d[2] | ((d[3] & 0x0F) << 8)); if (x & 0x800) x -= 0x1000;
    int16_t y = (int16_t)((d[3] >> 4) | (d[4] << 4));   if (y & 0x800) y -= 0x1000;
    s_mouse_self_addr = address;
    Mouse_UsbEvent(x, y, d[1] & 7);
    return true;
  }
  return false;
}

static void my_USB_HIDReportDescCB(const EspUsbHostHIDReportDescriptor &desc)
{
  Mouse_ParseDescriptor(desc.address, desc.interfaceNumber, desc.data, desc.length);
}

static void my_USB_MouseCB(const EspUsbHostMouseEvent &event)
{
  if (event.address == s_mouse_self_addr) return;    // déjà décodé à partir du rapport brut
  Mouse_UsbEvent(event.x, event.y, event.buttons);
}

// RAM interne utilisable par le DMA. ESP-IDF y alloue, pour CHAQUE canal USB ouvert (EP0 d'un nouveau
// périphérique, point d'accès d'une interface) deux listes de descripteurs alignées sur 512 octets
static void Mem_DmaStats(uint32_t &freeB, uint32_t &largestB, uint32_t &minB)
{
#ifdef ESP_PLATFORM
  freeB    = heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  largestB = heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  minB     = heap_caps_get_minimum_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
#else
  freeB = largestB = minB = 0;
#endif
}

// Échecs d'allocation (toute l'application) notés par le rappel d'ESP-IDF affichés ensuite par loop :
// le rappel peut être appelé dans un contexte où l'on ne doit rien imprimer.
struct MemFail { uint32_t size, caps; const char *func; };
static MemFail s_memFail[8];
static volatile uint32_t s_memFailCount = 0;
static uint32_t s_memFailShown = 0;
static void IRAM_ATTR Mem_FailedAllocCB(size_t size, uint32_t caps, const char *func)
{
  const uint32_t n = s_memFailCount;
  s_memFail[n & 7].size = (uint32_t)size;
  s_memFail[n & 7].caps = caps;
  s_memFail[n & 7].func = func;
  s_memFailCount = n + 1;
}
// réserve de RAM interne DMA pour l'USB, prise juste avant le démarrage du Wi-Fi (qui prend ~40 Ko et
// laisse la RAM DMA morcelée)
#define USB_DMA_RESERVE 4096
static void *s_dma_reserve = nullptr;
static bool s_dma_reserve_used = false;
// démarrage du pilote lui-même (ses allocations ratées comptaient comme un manque de l'USB).
static volatile bool s_dma_reserve_armed = false;
static uint32_t s_dma_arm_at = 0;      // échecs comptés avant la connexion : ignorés
static uint32_t s_dma_take_ms = 0;
void Usb_DmaReserveArm(void)
{
  if (s_dma_reserve_armed) return;
  s_dma_arm_at = s_memFailCount;
  s_dma_reserve_armed = true;
}
void Usb_DmaReserveTake(void)
{
  if (s_dma_reserve || s_dma_reserve_used) return;
  s_dma_reserve = heap_caps_malloc(USB_DMA_RESERVE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  s_dma_take_ms = millis();   // sans connexion au bout de 20 s, réserve disponible quand même
  LogOut.printf("[USB] reserve de RAM DMA pour un branchement ulterieur : %s (%u o)\n", s_dma_reserve ? "prise" : "impossible", (unsigned)USB_DMA_RESERVE);
}
void Mem_PrintFailures(void)
{
  const uint32_t n = s_memFailCount;
  if (n == s_memFailShown) return;
  if (!s_dma_reserve_armed && s_dma_reserve && (uint32_t)(millis() - s_dma_take_ms) > 20000u) Usb_DmaReserveArm();   // Wi-Fi jamais connecté
  if (s_dma_reserve && s_dma_reserve_armed && n > s_dma_arm_at && (s_memFail[(n - 1) & 7].caps & MALLOC_CAP_DMA))
  {
    heap_caps_free(s_dma_reserve); s_dma_reserve = nullptr; s_dma_reserve_used = true;
    LogOut.printf("[USB] reserve de RAM DMA rendue a l'USB (%u o) : nouvel essai de l'appareil dans ~1 s\n", (unsigned)USB_DMA_RESERVE);
  }
  if (n - s_memFailShown > 8) s_memFailShown = n - 8;
  uint32_t f, l, m; Mem_DmaStats(f, l, m);
  while (s_memFailShown != n)
  {
    const MemFail &e = s_memFail[s_memFailShown & 7];
    LogOut.printf("[MEM] echec d'allocation no %lu : %lu o, caps $%08lX (%s) ; RAM interne DMA libre %lu o, plus grand bloc %lu o, minimum %lu o\n",
                  (unsigned long)(s_memFailShown + 1), (unsigned long)e.size, (unsigned long)e.caps, e.func ? e.func : "?",
                  (unsigned long)f, (unsigned long)l, (unsigned long)m);
    s_memFailShown++;
  }
}

// état des canaux du contrôleur USB hôte (8 sur ESP32-S3). Chaque périphérique prend 1 canal pour son
// EP0, plus 1 par point d'accès de chaque interface réclamée
void Usb_Report(bool detail)
{
  if (!UsbHost) { LogOut.println("[USB] hote non demarre"); return; }
  {
    uint32_t f, l, m; Mem_DmaStats(f, l, m);
    LogOut.printf("[USB] RAM interne DMA libre %lu o (plus grand bloc %lu o, minimum atteint %lu o) ; piles : hote %lu/%lu o libres, client %lu/%lu o libres ; echecs d'allocation : %lu\n",
                  (unsigned long)f, (unsigned long)l, (unsigned long)m,
                  (unsigned long)UsbHost->hostTaskStackFree(), (unsigned long)UsbHost->hostTaskStackSize(),
                  (unsigned long)UsbHost->clientTaskStackFree(), (unsigned long)UsbHost->clientTaskStackSize(),
                  (unsigned long)s_memFailCount);
  }
  if (detail)   // rapports reçus par point d'accès (manette muette ?)
  {
    static char sb[1500];
    if (UsbHost->cocoEndpointStats(sb, sizeof sb)) LogOut.print(sb);
    LogOut.printf("[USB] filtre des interfaces HID : %s (commande H pour basculer, puis rebrancher l'appareil)\n", UsbHost->hidInterfaceFilter() ? "actif" : "COUPE");
  }
  EspUsbHostDeviceInfo dev[8];
  const size_t n = UsbHost->getDevices(dev, 8);
  LogOut.printf("[USB] filtre HID %s ; canaux estimes %u/%u (EP0 %u, interfaces %u, hub %u) ; %u peripherique(s) ; derniere erreur : %s\n",
                UsbHost->hidInterfaceFilter() ? "actif" : "inactif",
                (unsigned)UsbHost->estimatedHcdChannelCount(), (unsigned)UsbHost->maxEndpointChannelCount(),
                (unsigned)UsbHost->ep0ChannelCount(), (unsigned)UsbHost->endpointChannelCount(),
                (unsigned)UsbHost->hubEndpointChannelCount(), (unsigned)n, UsbHost->lastErrorName());
  for (size_t i = 0; i < n && i < 8; i++)
  {
    EspUsbHostInterfaceInfo itf[8];
    const size_t ni = UsbHost->getInterfaces(dev[i].address, itf, 8);
    LogOut.printf("[USB]   adr %u %04x:%04x %s%s, canaux %u :", dev[i].address, dev[i].vid, dev[i].pid,
                  dev[i].isHub ? "HUB " : "", dev[i].product ? dev[i].product : "",
                  (unsigned)UsbHost->estimatedHcdChannelCount(dev[i].address));
    for (size_t k = 0; k < ni && k < 8; k++)
      LogOut.printf(" [if%u cl%02X/%02X/%02X ep%u %s]", itf[k].number, itf[k].interfaceClass, itf[k].interfaceSubClass,
                    itf[k].interfaceProtocol, itf[k].endpointCount,
                    (itf[k].claimed && itf[k].number < 8 && ((UsbHost->cocoDrainMask(dev[i].address) >> itf[k].number) & 1u)) ? "OK, lue pour debloquer l'adaptateur" :
                    itf[k].claimed ? "OK" :
                    !itf[k].claimAttempted ? "non reclamee" :
                    itf[k].claimResult == ESP_ERR_NOT_ALLOWED ? "ignoree (filtre)" :
                    itf[k].claimResult == ESP_ERR_NOT_FINISHED ? "en attente/reserve" :
                    itf[k].claimResult == ESP_ERR_NO_MEM ? "manque de RAM DMA" :
                    itf[k].claimResult == ESP_ERR_NOT_SUPPORTED ? "plus de canal" :
                    esp_err_to_name(itf[k].claimResult));
    LogOut.printf("\n");
  }
  { // appareils branchés qui n'ont pas pu démarrer
    char sd[96];
    const int ns = Usb_StalledPorts(sd, sizeof sd);
    if (ns) LogOut.printf("[USB] %d appareil(s) branche(s) mais non demarre(s) (canaux USB pleins ?) : %s\n", ns, sd);
    { // interface abandonnée faute de RAM interne DMA
      bool nm = false;
      for (size_t i = 0; i < n && i < 8; i++)
      {
        EspUsbHostInterfaceInfo it[8];
        const size_t ni = UsbHost->getInterfaces(dev[i].address, it, 8);
        for (size_t k = 0; k < ni && k < 8; k++) if (!it[k].claimed && it[k].claimAttempted && it[k].claimResult == ESP_ERR_NO_MEM) nm = true;
      }
      if (nm) LogOut.println("[USB] appareil sans RAM interne DMA : couper DriveWire par le Wi-Fi, le modem ou le Wi-Fi (puis redemarrer) libere de la memoire ; rebrancher l'appareil");
    }
    if (UsbHost->cocoPreemptCount())
      LogOut.printf("[USB] manette / souris ayant cede son canal au clavier : %lu fois\n", (unsigned long)UsbHost->cocoPreemptCount());
  }
  // manettes vues par l'assistant J et réglages des ports
  for (uint8_t i = 0; i != HID_TRACK_MAX; i++)
  {
    const HidTrackedDevice &t = HidTracked[i];
    if (!t.present || t.subclass == 0x01) continue;
    if (t.address & 0x80)
      LogOut.printf("[JOY] vue : adr %u manette %u (%04x:%04x)\n", (t.address >> 2) & 0x1F, (t.address & 3) + 1, t.vid, t.pid);
    else
      LogOut.printf("[JOY] vue : adr %u (%04x:%04x)\n", t.address, t.vid, t.pid);
  }
  Joy_LogConfig("regle");
  if (detail) UsbHost->printAllDeviceInfo(LogOut);
  Mem_PrintFailures();
}

void Setup_USB(void)
{
  fillKeysStruct();

  UsbHost = new EspUsbHost();

  UsbHost->onKeyboard(my_USB_KeyboardCB);
  UsbHost->onHIDInput(my_USB_HIDInputCB);
  UsbHost->onMouse(my_USB_MouseCB);   // souris USB -> port joystick
  UsbHost->onHIDReportDescriptor(my_USB_HIDReportDescCB);   // disposition réelle des champs
  UsbHost->onDeviceDisconnected(my_USB_DeviceDisconnectedCB);
  UsbHost->onDeviceConnected(my_USB_DeviceConnectedCB);

  // ne réclamer que les interfaces HID utiles (clavier, souris, manette) les 8 canaux du contrôleur
  // hôte suffisent alors pour hub + 3 périphériques.
  UsbHost->setHidInterfaceFilter(true);

    EspUsbHostConfig usbCfg;
  usbCfg.taskCore = 0;   // Dans Core0 avec VideoCore, pas Core1 avec CPU cycle-accurate
  // piles en RAM interne : 2 x 8 Ko par défaut. La tâche hôte (pilote de hub, énumération) se contente
  // de 4 Ko (exemples d'ESP-IDF) ; la tâche client exécute nos rappels clavier/souris : 6 Ko
  usbCfg.taskStackSize = 5120;
  usbCfg.hostTaskStackSize = 4608;
#ifdef ESP_PLATFORM
  heap_caps_register_failed_alloc_callback(Mem_FailedAllocCB);
#endif
  bool ok = UsbHost->begin(usbCfg);

  if (!ok)
  {
    LogOut.printf("[USB] begin() failed: %s\n", UsbHost->lastErrorName());
  }

}

#define K_ESC 27
#define K_F1 1
#define K_F2 2
#define K_BACKSPACE 8
#define K_APOSTROPHE 39
#define K_ENTER 13
#define K_QUOTE 34
#define K_SPACE 32
#define K_CLEAR 3
#define K_SHIFT 0x82
#define K_CTRL 0x81
#define K_ALT 0x84
#define K_CAPS 0x0a  // SHIFT 0 on CoCo

#define K_PAUSE_AT 0x0E   // touche PC Pause ou ` = SHIFT+@ du CoCo (pause de LIST/DIR)
#define K_UP 4
#define K_DOWN 5
#define K_LEFT 6
#define K_RIGHT 7

#define lookupOR_0 0b00000001
#define lookupOR_1 0b00000010
#define lookupOR_2 0b00000100
#define lookupOR_3 0b00001000
#define lookupOR_4 0b00010000
#define lookupOR_5 0b00100000
#define lookupOR_6 0b01000000
#define lookupOR_7 0b10000000

#define lookupAND_0 0b11111110
#define lookupAND_1 0b11111101
#define lookupAND_2 0b11111011
#define lookupAND_3 0b11110111
#define lookupAND_4 0b11101111
#define lookupAND_5 0b11011111
#define lookupAND_6 0b10111111
#define lookupAND_7 0b01111111
void fillKeysStruct(void)
{
    USB_DEV_CONTROL.JOY1_BUTT1 = 1;
    USB_DEV_CONTROL.JOY1_BUTT2 = 1;
    USB_DEV_CONTROL.JOY1_X_AXIS = 128;   // centre (échelle 0-255 ; 31 = ancienne échelle 0-63, JOYSTK donnait 7)
    USB_DEV_CONTROL.JOY1_Y_AXIS = 128;
    USB_DEV_CONTROL.JOY2_BUTT1 = 1;
    USB_DEV_CONTROL.JOY2_BUTT2 = 1;
    USB_DEV_CONTROL.JOY2_X_AXIS = 128;
    USB_DEV_CONTROL.JOY2_Y_AXIS = 128;

    USB_DEV_CONTROL.PORT_JOY1 = 1;
    USB_DEV_CONTROL.PORT_JOY2 = 2;
    USB_DEV_CONTROL.PORT_KEYBOARD = 0;
    for (uint16_t u=0; u!=256; u++)
    {
        USB_DEV_CONTROL.ScanArray[u] = 0;
    }

    USB_DEV_CONTROL.ScanArray[0x29] = K_ESC;
    USB_DEV_CONTROL.ScanArray[0x3a] = K_F1;
    USB_DEV_CONTROL.ScanArray[0x3b] = K_F2;
    USB_DEV_CONTROL.ScanArray[0x1e] = '1';
    USB_DEV_CONTROL.ScanArray[0x1f] = '2';
    USB_DEV_CONTROL.ScanArray[0x20] = '3';
    USB_DEV_CONTROL.ScanArray[0x21] = '4';
    USB_DEV_CONTROL.ScanArray[0x22] = '5';
    USB_DEV_CONTROL.ScanArray[0x23] = '6';
    USB_DEV_CONTROL.ScanArray[0x24] = '7';
    USB_DEV_CONTROL.ScanArray[0x25] = '8';
    USB_DEV_CONTROL.ScanArray[0x26] = '9';
    USB_DEV_CONTROL.ScanArray[0x27] = '0';
    USB_DEV_CONTROL.ScanArray[0x2d] = '-';
    USB_DEV_CONTROL.ScanArray[0x2e] = '=';
    USB_DEV_CONTROL.ScanArray[0x2a] = K_BACKSPACE;
    USB_DEV_CONTROL.ScanArray[0x14] = 'Q';
    USB_DEV_CONTROL.ScanArray[0x1a] = 'W';
    USB_DEV_CONTROL.ScanArray[0x08] = 'E';
    USB_DEV_CONTROL.ScanArray[0x15] = 'R';
    USB_DEV_CONTROL.ScanArray[0x17] = 'T';
    USB_DEV_CONTROL.ScanArray[0x1c] = 'Y';
    USB_DEV_CONTROL.ScanArray[0x18] = 'U';
    USB_DEV_CONTROL.ScanArray[0x0c] = 'I';
    USB_DEV_CONTROL.ScanArray[0x12] = 'O';
    USB_DEV_CONTROL.ScanArray[0x13] = 'P';
    USB_DEV_CONTROL.ScanArray[0x04] = 'A';
    USB_DEV_CONTROL.ScanArray[0x16] = 'S';
    USB_DEV_CONTROL.ScanArray[0x07] = 'D';
    USB_DEV_CONTROL.ScanArray[0x09] = 'F';
    USB_DEV_CONTROL.ScanArray[0x0a] = 'G';
    USB_DEV_CONTROL.ScanArray[0x0b] = 'H';
    USB_DEV_CONTROL.ScanArray[0x0d] = 'J';
    USB_DEV_CONTROL.ScanArray[0x0e] = 'K';
    USB_DEV_CONTROL.ScanArray[0x0f] = 'L';
    USB_DEV_CONTROL.ScanArray[0x33] = ';';
    USB_DEV_CONTROL.ScanArray[0x34] = K_APOSTROPHE;
    USB_DEV_CONTROL.ScanArray[0x28] = K_ENTER;
    USB_DEV_CONTROL.ScanArray[0x1d] = 'Z';
    USB_DEV_CONTROL.ScanArray[0x1b] = 'X';
    USB_DEV_CONTROL.ScanArray[0x06] = 'C';
    USB_DEV_CONTROL.ScanArray[0x19] = 'V';
    USB_DEV_CONTROL.ScanArray[0x05] = 'B';
    USB_DEV_CONTROL.ScanArray[0x11] = 'N';
    USB_DEV_CONTROL.ScanArray[0x10] = 'M';
    USB_DEV_CONTROL.ScanArray[0x36] = ',';
    USB_DEV_CONTROL.ScanArray[0x37] = '.';
    USB_DEV_CONTROL.ScanArray[0x38] = '/';
    USB_DEV_CONTROL.ScanArray[0x39] = K_CAPS;  // SHIFT 0 on CoCo

    USB_DEV_CONTROL.ScanArray[0x52] = K_UP;
    USB_DEV_CONTROL.ScanArray[0x48] = K_PAUSE_AT;   // Pause = SHIFT+@ (pause de l'affichage)
    USB_DEV_CONTROL.ScanArray[0x35] = K_PAUSE_AT;   // touche ` (sous Échap) = SHIFT+@
    USB_DEV_CONTROL.ScanArray[0x51] = K_DOWN;
    USB_DEV_CONTROL.ScanArray[0x50] = K_LEFT;
    USB_DEV_CONTROL.ScanArray[0x4f] = K_RIGHT;

    USB_DEV_CONTROL.ScanArray[0x80 | 0x1e] = '!';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x1f] = '@';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x20] = '#';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x21] = '$';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x22] = '%';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x23] = '^';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x24] = '&';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x25] = '*';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x26] = '(';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x27] = ')';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x2d] = '_';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x2e] = '+';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x34] = K_QUOTE;
    USB_DEV_CONTROL.ScanArray[0x80 | 0x33] = ':';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x36] = '<';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x37] = '>';
    USB_DEV_CONTROL.ScanArray[0x80 | 0x38] = '?';
    USB_DEV_CONTROL.ScanArray[0x2c] = K_SPACE;
    USB_DEV_CONTROL.ScanArray[0x4c] = K_CLEAR;

    // modificateurs (UpdateKeyMap). Ces entrées tombaient sur les index $81/$82/$84 = table « shiftée
    // » de $01/$02/$04 : SHIFT+A donnait ALT.

}

// ===========================================================================
// manette au clavier. Jeux qui ne se jouent qu'au joystick (DINOWARS, etc.) sans manette USB : 3 jeux
// de touches, port droit ou gauche. Axes numériques
// ===========================================================================
// clavier sur l'un ou les deux ports (g_kbjoy_port : bit 0 droit, bit 1 gauche) jeu de touches propre
// à chaque port (g_kbjoy_set : droit, g_kbjoy_set2 : gauche).
static void KbJoy_Detect(const uint8_t *d, uint8_t set, uint8_t &ax, uint8_t &ay, bool &f1, bool &f2, uint8_t &usedMask)
{
    bool up = false, down = false, left = false, right = false;
    f1 = f2 = false;
    for (uint8_t k = 2; k != 8; k++)
    {
        const uint8_t c = d[k];
        bool used = true;
        switch (set)
        {
        case 1:   // pavé numérique
            switch (c)
            {
            case 0x60: up = true; break;                    // 8
            case 0x5A: down = true; break;                  // 2
            case 0x5C: left = true; break;                  // 4
            case 0x5E: right = true; break;                 // 6
            case 0x5F: up = left = true; break;             // 7
            case 0x61: up = right = true; break;            // 9
            case 0x59: down = left = true; break;           // 1
            case 0x5B: down = right = true; break;          // 3
            case 0x62: f1 = true; break;                    // 0
            case 0x58: f2 = true; break;                    // Entrée du pavé
            default: used = false; break;
            }
            break;
        case 2:   // WASD
            switch (c)
            {
            case 0x1A: up = true; break;                    // W
            case 0x16: down = true; break;                  // S
            case 0x04: left = true; break;                  // A
            case 0x07: right = true; break;                 // D
            case 0x2C: f1 = true; break;                    // Espace
            case 0x09: f2 = true; break;                    // F
            default: used = false; break;
            }
            break;
        default:  // flèches
            switch (c)
            {
            case 0x52: up = true; break;
            case 0x51: down = true; break;
            case 0x50: left = true; break;
            case 0x4F: right = true; break;
            case 0x3A: f1 = true; break;                    // F1
            case 0x3B: f2 = true; break;                    // F2
            default: used = false; break;
            }
            break;
        }
        if (used) usedMask |= (uint8_t)(1u << k);
    }
    ax = left ? 0 : (right ? 255 : 128);
    ay = up ? 0 : (down ? 255 : 128);
}
static void KbJoy_Apply(uint8_t *d)
{
    uint8_t used = 0;
    for (uint8_t p = 0; p < 2; p++)
    {
        if (!(g_kbjoy_port & (1u << p))) continue;
        uint8_t ax, ay; bool f1, f2;
        KbJoy_Detect(d, p == 0 ? g_kbjoy_set : g_kbjoy_set2, ax, ay, f1, f2, used);
        if (p == 0)
        {
            USB_DEV_CONTROL.JOY1_X_AXIS = ax; USB_DEV_CONTROL.JOY1_Y_AXIS = ay;
            USB_DEV_CONTROL.JOY1_BUTT1 = f1 ? 0 : 1; USB_DEV_CONTROL.JOY1_BUTT2 = f2 ? 0 : 1;
        }
        else
        {
            USB_DEV_CONTROL.JOY2_X_AXIS = ax; USB_DEV_CONTROL.JOY2_Y_AXIS = ay;
            USB_DEV_CONTROL.JOY2_BUTT1 = f1 ? 0 : 1; USB_DEV_CONTROL.JOY2_BUTT2 = f2 ? 0 : 1;
        }
    }
    for (uint8_t k = 2; k != 8; k++) if ((used >> k) & 1) d[k] = 0;
}

void UpdateKeyMap(uint8_t * Data)
{
    bool ShiftDisabled = false;
    bool ForceShift = false;
    uint8_t SpecialKey;
    uint8_t KeyTranslated = 0;
    uint8_t loop1;
    // combinaison des bits donnait un seul code : SHIFT+CTRL, etc
    uint8_t kd[8];
    Tester_Keys(Data);   // F9, Impr. écran, Insert, Ctrl+Alt+Suppr/Retour arrière (retirées de…)
    const uint8_t rawDirectKey = Data[2];   // le menu F12 doit voir la touche réelle (flèches)
    if (g_kbjoy_port != 0)
    {
        memcpy(kd, Data, 8);
        Data = kd;
        KbJoy_Apply(kd);
    }
    const bool shiftDown = (Data[0] & 0x22) != 0;
    bool letterSeen = false;
    // Verr. Maj. : en mode option L, bascule interne (au front d'appui) au lieu de SHIFT+0 envoyé au
    // CoCo.
    static bool s_caps_prev = false;
    bool caps_now = false;
    for (uint8_t k = 2; k != 7; k++) if (Data[k] == 0x39) caps_now = true;
    if (g_lowercase_t1 && caps_now && !s_caps_prev)
    {
      g_pc_caps = !g_pc_caps;
      g_caps_save_ms = (uint32_t)(esp_timer_get_time() / 1000) | 1u;   // sauvegardé par le coeur vidéo (pas d'accès SD depuis la tâche USB)
    }
    s_caps_prev = caps_now;
    const bool ctrlDown  = (Data[0] & 0x11) != 0;
    const bool altDown   = (Data[0] & 0x44) != 0;
    SpecialKey = shiftDown ? K_SHIFT : 0;
    for (uint8_t u = 0; u!=8; u++)
    {
        USB_DEV_CONTROL.USB_CoCo_Key_Array[u] = 0;
    }
    if (shiftDown) USB_DEV_CONTROL.USB_CoCo_Key_Array[7] |= lookupOR_6;
    if (ctrlDown)  USB_DEV_CONTROL.USB_CoCo_Key_Array[4] |= lookupOR_6;
    if (altDown)   USB_DEV_CONTROL.USB_CoCo_Key_Array[3] |= lookupOR_6;

    sf.DIRECT_Key_Code = rawDirectKey;

    for (loop1=2; loop1 !=7; loop1++)
    {
        const uint8_t raw = Data[loop1];
        KeyTranslated = 0;
        if (shiftDown && raw != 0 && raw < 0x80)
        {
            KeyTranslated = USB_DEV_CONTROL.ScanArray[raw | 0x80];
        }
        if (KeyTranslated == 0)
        {
            KeyTranslated = USB_DEV_CONTROL.ScanArray[raw];
        }
        if (g_lowercase_t1 && raw == 0x39) KeyTranslated = 0;   // Verr. Maj. géré ci-dessus

        switch (KeyTranslated)
        {
        case 'H':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[0] |= lookupOR_1;
            break;
        case 'P':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[0] |= lookupOR_2;
            break;
        case 'X':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[0] |= lookupOR_3;
            break;
        case '0':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[0] |= lookupOR_4;
            break;
        case '8':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[0] |= lookupOR_5;
            break;
        case K_ENTER:
            USB_DEV_CONTROL.USB_CoCo_Key_Array[0] |= lookupOR_6;
            break;
        //-------------------------------------------
        case 'A':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[1] |= lookupOR_0;
            break;
        case 'I':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[1] |= lookupOR_1;
            break;
        case 'Q':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[1] |= lookupOR_2;
            break;
        case 'Y':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[1] |= lookupOR_3;
            break;
        case '1':
        case '!':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[1] |= lookupOR_4;
            break;
        case '9':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[1] |= lookupOR_5;
            break;
        case K_CLEAR:
            USB_DEV_CONTROL.USB_CoCo_Key_Array[1] |= lookupOR_6;
            break;
        //-------------------------------------------
        case 'B':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[2] |= lookupOR_0;
            break;
        case 'J':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[2] |= lookupOR_1;
            break;
        case 'R':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[2] |= lookupOR_2;
            break;
        case 'Z':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[2] |= lookupOR_3;
            break;
        case '2':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[2] |= lookupOR_4;
            break;
        case K_ESC:
            USB_DEV_CONTROL.USB_CoCo_Key_Array[2] |= lookupOR_6;
            break;
        //-------------------------------------------
        case 'C':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[3] |= lookupOR_0;
            break;
        case 'K':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[3] |= lookupOR_1;
            break;
        case 'S':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[3] |= lookupOR_2;
            break;
        case K_UP:
            USB_DEV_CONTROL.USB_CoCo_Key_Array[3] |= lookupOR_3;
            break;
        case '3':
        case '#':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[3] |= lookupOR_4;
            break;
        case ';':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[3] |= lookupOR_5;
            break;
        //-------------------------------------------
        case 'D':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[4] |= lookupOR_0;
            break;
        case 'L':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[4] |= lookupOR_1;
            break;
        case 'T':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[4] |= lookupOR_2;
            break;
        case K_DOWN:
            USB_DEV_CONTROL.USB_CoCo_Key_Array[4] |= lookupOR_3;
            break;
        case '4':
        case '$':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[4] |= lookupOR_4;
            break;
        case ',':
        case '<':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[4] |= lookupOR_5;
            break;
        //-------------------------------------------
        case 'E':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[5] |= lookupOR_0;
            break;
        case 'M':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[5] |= lookupOR_1;
            break;
        case 'U':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[5] |= lookupOR_2;
            break;
        case K_LEFT:
        case K_BACKSPACE:
            USB_DEV_CONTROL.USB_CoCo_Key_Array[5] |= lookupOR_3;
            break;
        case '5':
        case '%':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[5] |= lookupOR_4;
            break;
        case '-':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[5] |= lookupOR_5;
            break;
        case K_F1:
            USB_DEV_CONTROL.USB_CoCo_Key_Array[5] |= lookupOR_6;
            break;
        //-------------------------------------------
        case 'F':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[6] |= lookupOR_0;
            break;
        case 'N':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[6] |= lookupOR_1;
            break;
        case 'V':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[6] |= lookupOR_2;
            break;
        case K_RIGHT:
            USB_DEV_CONTROL.USB_CoCo_Key_Array[6] |= lookupOR_3;
            break;
        case '6':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[6] |= lookupOR_4;
            break;
        case '.':
        case '>':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[6] |= lookupOR_5;
            break;
        case K_F2:
            USB_DEV_CONTROL.USB_CoCo_Key_Array[6] |= lookupOR_6;
            break;
        //-------------------------------------------
        case 'G':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[7] |= lookupOR_0;
            break;
        case 'O':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[7] |= lookupOR_1;
            break;
        case 'W':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[7] |= lookupOR_2;
            break;
        case K_SPACE:
            USB_DEV_CONTROL.USB_CoCo_Key_Array[7] |= lookupOR_3;
            break;
        case '7':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[7] |= lookupOR_4;
            break;
        case '/':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[7] |= lookupOR_5;
            break;
        case '&':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[6] |= lookupOR_4;
            break;
        case '?':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[7] |= lookupOR_5;
            break;
        case '@':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[0] |= lookupOR_0;
            ShiftDisabled = true;
            break;
        case K_PAUSE_AT:   // SHIFT+@ du CoCo (un « @ » PC s'obtient avec SHIFT, qu'on retire)
            USB_DEV_CONTROL.USB_CoCo_Key_Array[0] |= lookupOR_0;
            ForceShift = true;
            break;
        case '*':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[2] |= lookupOR_5;
            break;
        case '(':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[0] |= lookupOR_5;
            break;
        case ')':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[1] |= lookupOR_5;
            break;
        case '=':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[5] |= lookupOR_5;
            ForceShift = true;
            break;
        case '+':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[3] |= lookupOR_5;
            ForceShift = true;
            break;
        case K_CAPS:
            USB_DEV_CONTROL.USB_CoCo_Key_Array[0] |= lookupOR_4;
            ForceShift = true;
            break;
        case ':':
            USB_DEV_CONTROL.USB_CoCo_Key_Array[2] |= lookupOR_5;
            ShiftDisabled = true;
            break;
        case K_APOSTROPHE:
            USB_DEV_CONTROL.USB_CoCo_Key_Array[7] |= lookupOR_4;
            ForceShift = true;
            break;
        case K_QUOTE:
            USB_DEV_CONTROL.USB_CoCo_Key_Array[2] |= lookupOR_4;
            ForceShift = true;
            break;

            //-------------------------------------------

        default:
            break;
        }
        if (ShiftDisabled == true)
        {
            USB_DEV_CONTROL.USB_CoCo_Key_Array[7] &= lookupAND_6;
        }
        if (ForceShift == true)
        {
            USB_DEV_CONTROL.USB_CoCo_Key_Array[7] |= lookupOR_6;
        }
        if (KeyTranslated >= 'A' && KeyTranslated <= 'Z') letterSeen = true;
    }

    // lettres : Verr. Maj. (bascule interne) ou SHIFT -> majuscule, sinon minuscule. On envoie donc
    // SHIFT au CoCo seulement pour une majuscule BASIC étant maintenu en mode minuscules (CASFLG $011A
    // = 0)
    if (g_lowercase_t1 && letterSeen)
    {
        if (g_pc_caps || shiftDown) USB_DEV_CONTROL.USB_CoCo_Key_Array[7] |= lookupOR_6;
        else                        USB_DEV_CONTROL.USB_CoCo_Key_Array[7] &= lookupAND_6;
        // BASIC seulement (vecteurs IRQ/FIRQ = JMP en $010C/$010F) : forcer CASFLG à 0
        if (gime_peek(0x010C) == 0x7E && gime_peek(0x010F) == 0x7E && gime_peek(0x011A) == 0xFF)
            gime_poke(0x011A, 0x00);
    }

}

void UpdateJoyMap(uint8_t * Data, uint8_t usbNum)
{
    if (usbNum == 1)    // Joystick1
    {
        USB_DEV_CONTROL.JOY1_X_AXIS = (Data[0]>>0);
        USB_DEV_CONTROL.JOY1_Y_AXIS = (Data[1]>>0);
        USB_DEV_CONTROL.JOY1_BUTT1 = ((Data[4]<<3) & 0b10000000);
        USB_DEV_CONTROL.JOY1_BUTT2 = ((Data[4]<<2) & 0b10000000);
        if (USB_DEV_CONTROL.JOY1_BUTT1 !=0)
        {
            USB_DEV_CONTROL.JOY1_BUTT1 = 0;
        }
        else
        {
            USB_DEV_CONTROL.JOY1_BUTT1 = 0x01;
        }
        if (USB_DEV_CONTROL.JOY1_BUTT2 !=0)
        {
            USB_DEV_CONTROL.JOY1_BUTT2 = 0;
        }
        else
        {
            USB_DEV_CONTROL.JOY1_BUTT2 = 0x01;
        }
    }
    else if (usbNum == 2)    // Joystick2
    {
        USB_DEV_CONTROL.JOY2_X_AXIS = (Data[0]>>0);
        USB_DEV_CONTROL.JOY2_Y_AXIS = (Data[1]>>0);
        USB_DEV_CONTROL.JOY2_BUTT1 = ((Data[4]<<3) & 0b10000000);
        USB_DEV_CONTROL.JOY2_BUTT2 = ((Data[4]<<2) & 0b10000000);
        if (USB_DEV_CONTROL.JOY2_BUTT1 !=0)
        {
            USB_DEV_CONTROL.JOY2_BUTT1 = 0;
        }
        else
        {
            USB_DEV_CONTROL.JOY2_BUTT1 = 0x01;
        }
        if (USB_DEV_CONTROL.JOY2_BUTT2 !=0)
        {
            USB_DEV_CONTROL.JOY2_BUTT2 = 0;
        }
        else
        {
            USB_DEV_CONTROL.JOY2_BUTT2 = 0x01;
        }
    }
}

//----------------- End USB COCO MAP

// ===========================================================================
// SAVE STATE — instantané complet de la machine sur la carte SD. Fichier
// /ESP-STATES/<logiciel>-<n>.STA (n = 1 à 4) : en-tête, configuration (slots, disques), puis sections
// étiquetées
// ===========================================================================
#define STATE_DIR      "/ESP-STATES"
#define STATE_MAGIC    "CC3STAT1"
#define STATE_VERSION  1
struct StateHdr
{
  char     magic[8];
  uint16_t version, hdr_size;
  char     fw[8];
  uint32_t ram_size;
  uint8_t  cpu6309, pad[3];
  uint32_t seq;          // numéro de sauvegarde (ordre chronologique, pas d'horloge)
  uint32_t frames;       // trames émulées depuis le démarrage du logiciel
  char     soft[48];
};
struct StateMisc
{
  uint8_t  wait_state, irq_line, firq_line, cb1_flag, rs_irq, cart_line, mpi_reg, scs_slot;
  uint8_t  slot_bank[MPI_SLOTS], cpsg_bank[2], cpsg_ctrl, cpsg_latch, cpsg_reg[16], o90_l, o90_r;
  uint8_t  pia_ddr[4];   // [3] = DDR de $FF20 (ancien octet de bourrage)
  uint32_t vhd_lsn;
  uint16_t vhd_buf;
  uint8_t  vhd_status, vhd_sel;
};
struct StateConf
{
  SlotConfig slots;
  char disk[4][256];
  char vd[VD_UNITS][256];
};
uint8_t g_state_slot = 0;                       // emplacement courant (0-3) : F5 / F8
bool    g_wait_before_menu = false;             // CPU_in_WAIT_STATE avant le menu F12

char g_state_name_loaded[48] = "";             // nom de l'état rechargé (F8 / menu V)

static void State_SoftName(char *out, size_t n)
{
  const char *src = nullptr;
  // d'abord le disque que le programme en cours lit réellement (ex. jeu lancé depuis le lecteur 1 :
  // son nom, pas celui du lecteur 0).
  const int8_t ld = g_state_last_disk;
  if (ld >= 0 && ld < 4 && Disk_Drive.Name_Disk[ld][0]) src = (const char *)Disk_Drive.Name_Disk[ld];
  else if (ld >= 4 && ld - 4 < VD_UNITS && g_dw_name[ld - 4][0]) src = g_dw_name[ld - 4];
  else if (g_state_name_loaded[0]) src = g_state_name_loaded;   // état rechargé, rien lu depuis
  else if (Disk_Drive.Name_Disk[0][0])  src = (const char *)Disk_Drive.Name_Disk[0];
  else if (g_dw_name[VHD_UNIT0][0]) src = g_dw_name[VHD_UNIT0];
  else if (g_dw_name[0][0])         src = g_dw_name[0];
  else
    for (int i = 0; i < MPI_SLOTS && !src; i++)
      if (SLOT_HAS_ROM(g_slotcfg_run.mode[i]) && g_slotcfg_run.name[i][0]) src = g_slotcfg_run.name[i];
  if (!src) { snprintf(out, n, "BASIC"); return; }
  const char *b = strrchr(src, '/');
  b = b ? b + 1 : src;
  size_t k = 0;
  for (; *b && *b != '.' && k + 1 < n && k < 40; b++)
  {
    const char c = *b;
    out[k++] = ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_') ? c : '_';
  }
  out[k] = 0;
  if (!k) snprintf(out, n, "SOFT");
}
void State_CurrentName(char *out, size_t n) { State_SoftName(out, n); }
static void State_Path(char *out, size_t n, uint8_t slot)
{
  char nm[48];
  State_SoftName(nm, sizeof nm);
  snprintf(out, n, STATE_DIR "/%s-%u.STA", nm, (unsigned)slot + 1);
}

// nom du dernier logiciel sauvegardé / rechargé, gardé sur la SD (/ESP-STATES/LAST.TXT) : au
// démarrage, avant toute lecture de disque, le menu V et F8 visent ce logiciel
#define STATE_LAST_FILE STATE_DIR "/LAST.TXT"
static void State_RememberName(const char *nm)      // appelant : sans le verrou SD
{
  if (!SD_MMC_MountOK || !nm || !nm[0]) return;
  SD_LOCK();
  if (!SD_MMC.exists(STATE_DIR)) SD_MMC.mkdir(STATE_DIR);
  File f = SD_MMC.open(STATE_LAST_FILE, FILE_WRITE);
  if (f) { f.write((const uint8_t *)nm, strlen(nm)); f.close(); }
  SD_UNLOCK();
}
void State_InitName(void)                            // setup, après le montage de la SD
{
  if (!SD_MMC_MountOK) return;
  SD_LOCK();
  File f = SD_MMC.open(STATE_LAST_FILE, FILE_READ);
  if (f)
  {
    char b[48] = "";
    const size_t n = f.read((uint8_t *)b, sizeof b - 1);
    b[n] = 0;
    for (size_t i = 0; i < n; i++) if (b[i] == '\r' || b[i] == '\n') { b[i] = 0; break; }
    snprintf(g_state_name_loaded, sizeof g_state_name_loaded, "%s", b);
    f.close();
  }
  SD_UNLOCK();
  if (g_state_name_loaded[0]) LogOut.printf("[STATE] dernier logiciel : %s\n", g_state_name_loaded);
}

// menu V, touche N : logiciel suivant (ordre alphabétique) parmi ceux qui ont des états dans
// /ESP-STATES. Il devient le nom courant jusqu'à la prochaine lecture de disque par le CoCo
int State_NextSoftware(void)
{
  if (!SD_MMC_MountOK) return 0;
  const int MAXN = 64;
  char (*names)[41] = (char (*)[41])heap_caps_malloc(MAXN * 41, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!names) return 0;
  int cnt = 0;
  SD_LOCK();
  File d = SD_MMC.open(STATE_DIR);
  if (d && d.isDirectory())
  {
    for (File e = d.openNextFile(); e; e = d.openNextFile())
    {
      const char *fn = e.name();
      const char *b = strrchr(fn, '/'); b = b ? b + 1 : fn;
      const size_t L = strlen(b);
      // <NOM>-n.STA
      if (!e.isDirectory() && L > 6 && L - 6 <= 40 && strcasecmp(b + L - 4, ".STA") == 0 && b[L - 6] == '-' && b[L - 5] >= '1' && b[L - 5] <= '4')
      {
        char nm[41];
        memcpy(nm, b, L - 6); nm[L - 6] = 0;
        bool dup = false;
        for (int i = 0; i < cnt && !dup; i++) dup = strcmp(names[i], nm) == 0;
        if (!dup && cnt < MAXN) strcpy(names[cnt++], nm);
      }
      e.close();
    }
  }
  if (d) d.close();
  SD_UNLOCK();
  if (cnt)
  {
    char cur[48];
    State_SoftName(cur, sizeof cur);
    // le suivant de « cur » dans l'ordre alphabétique (sinon le premier)
    int best = -1, first = 0;
    for (int i = 0; i < cnt; i++)
    {
      if (strcasecmp(names[i], names[first]) < 0) first = i;
      if (strcasecmp(names[i], cur) > 0 && (best < 0 || strcasecmp(names[i], names[best]) < 0)) best = i;
    }
    if (best < 0) best = first;
    snprintf(g_state_name_loaded, sizeof g_state_name_loaded, "%s", names[best]);
    g_state_last_disk = -1;
  }
  free(names);
  return cnt;
}

// Attendre que la tâche CPU soit arrêtée entre deux rafales (drapeaux de menu déjà posés par
// l'appelant). false = délai dépassé.
static bool State_WaitParked(void);
bool State_WaitParkedPublic(void) { return State_WaitParked(); }
static bool State_WaitParked(void)
{
#ifndef ESP_PLATFORM
  return true;                                       // banc : pas de tâche CPU séparée
#endif
  g_cpu_parked = false;
  for (int i = 0; i < 100 && !g_cpu_parked; i++) vTaskDelay(pdMS_TO_TICKS(5));
  return g_cpu_parked;
}

static bool St_W(File &f, const void *p, size_t n) { return f.write((const uint8_t *)p, n) == n; }
static bool St_Sec(File &f, const char *tag, const void *p, uint32_t n)
{
  return St_W(f, tag, 4) && St_W(f, &n, 4) && St_W(f, p, n);
}
static bool St_RSec(File &f, const char *tag, void *p, uint32_t n)
{
  char t[4]; uint32_t l = 0;
  if (f.read((uint8_t *)t, 4) != 4 || memcmp(t, tag, 4) != 0) return false;
  if (f.read((uint8_t *)&l, 4) != 4 || l != n) return false;
  return f.read((uint8_t *)p, n) == n;
}

// Résumé d'un emplacement pour le menu : "#12 MEGABUG" ou "(empty)".
void State_Describe(uint8_t slot, char *out, size_t n)
{
  char path[96];
  State_Path(path, sizeof path, slot);
  snprintf(out, n, "(empty)");
  if (!SD_MMC_MountOK) return;
  SD_LOCK();
  File f = SD_MMC.open(path, FILE_READ);
  StateHdr h;
  if (f && f.read((uint8_t *)&h, sizeof h) == sizeof h && memcmp(h.magic, STATE_MAGIC, 8) == 0)
  {
    h.soft[sizeof h.soft - 1] = 0;
    const uint32_t sec = h.frames / 60u;
    snprintf(out, n, "#%lu %.20s (uptime %lu:%02lu)", (unsigned long)h.seq, h.soft,
             (unsigned long)(sec / 60u), (unsigned long)(sec % 60u));
  }
  if (f) f.close();
  SD_UNLOCK();
}

// AUTORUN (autorun.cpp) : touches tapées comme un instantané de clavier USB
void Kbd_Inject(uint8_t mod, uint8_t key)
{
  uint8_t d[8] = { mod, 0, key, 0, 0, 0, 0, 0 };
  UpdateKeyMap(d);
}
uint16_t Cpu_PC(void) { return cpu ? cpu->get_pc() : 0; }
uint16_t Cpu_SP(void) { return cpu ? cpu->get_sp() : 0; }
bool Emu_Running(void) { return !sf.CPU_HALTED_BY_EMULATOR; }   // (menu F12, note, état...)

// écran de veille : valeur qui change à chaque action (touche, souris, bouton ou manche de manette
// poussé franchement ; le bruit d'un manche au repos est ignoré)
uint32_t Input_Activity(void)
{
  uint32_t h = g_input_events * 2654435761u;
  const uint8_t ax[4] = { USB_DEV_CONTROL.JOY1_X_AXIS, USB_DEV_CONTROL.JOY1_Y_AXIS, USB_DEV_CONTROL.JOY2_X_AXIS, USB_DEV_CONTROL.JOY2_Y_AXIS };
  for (int i = 0; i < 4; i++) { const int d = (int)ax[i] - 128; h = h * 31u + (d > 48 ? 1u : d < -48 ? 2u : 0u); }
  h = h * 31u + (USB_DEV_CONTROL.JOY1_BUTT1 ? 0u : 1u) + (USB_DEV_CONTROL.JOY1_BUTT2 ? 0u : 2u) + (USB_DEV_CONTROL.JOY2_BUTT1 ? 0u : 4u) + (USB_DEV_CONTROL.JOY2_BUTT2 ? 0u : 8u);
  return h;
}
// écran d'accueil : une touche (code USB) ou un bouton (manette, souris) -> $FF
uint8_t Input_AnyPressed(void)
{
  if (sf.DIRECT_Key_Code) return sf.DIRECT_Key_Code;
  if (!USB_DEV_CONTROL.JOY1_BUTT1 || !USB_DEV_CONTROL.JOY1_BUTT2 || !USB_DEV_CONTROL.JOY2_BUTT1 || !USB_DEV_CONTROL.JOY2_BUTT2) return 0xFF;
  extern volatile uint8_t g_mouse_buttons;
  if (g_mouse_buttons) return 0xFF;
  return 0;
}

// page web (webui.cpp) : verrou de la SD et disques montés (protégés contre l'écriture)
void Sd_Lock(void)   { SD_LOCK(); }
void Sd_Unlock(void) { SD_UNLOCK(); }
int Sd_MountedPaths(const char **out, int max)
{
  int n = 0;
  for (int i = 0; i < 4 && n < max; i++) if (Disk_Drive.Name_Disk[i][0]) out[n++] = (const char *)Disk_Drive.Name_Disk[i];
  for (int i = 0; i < VD_UNITS && n < max; i++) if (g_dw_name[i][0]) out[n++] = g_dw_name[i];
  return n;
}

// cartouche reçue du PC : copie sur la SD (dossier créé au besoin). Rend true si tout est écrit.
bool Sd_WriteFile(const char *path, const uint8_t *data, uint32_t size)
{
  if (!SD_MMC_MountOK || !path || path[0] != '/') return false;
  char dir[128];
  snprintf(dir, sizeof dir, "%s", path);
  char *sl = strrchr(dir, '/');
  if (sl && sl != dir) *sl = 0; else dir[0] = 0;
  SD_LOCK();
  if (dir[0] && !SD_MMC.exists(dir)) SD_MMC.mkdir(dir);
  if (SD_MMC.exists(path)) SD_MMC.remove(path);
  File f = SD_MMC.open(path, FILE_WRITE);
  bool ok = false;
  if (f) { ok = f.write(data, size) == size; f.close(); }
  SD_UNLOCK();
  LogOut.printf("[SLOT] %s : %lu octets %s\n", path, (unsigned long)size, ok ? "copies du PC sur la SD" : "NON ecrits (SD)");
  return ok;
}

// effacer un emplacement, ou tous les états du logiciel courant.
bool State_Delete(uint8_t slot)
{
  if (!SD_MMC_MountOK || slot > 3) return false;
  char path[96];
  State_Path(path, sizeof path, slot);
  SD_LOCK();
  const bool ok = SD_MMC.exists(path) && SD_MMC.remove(path);
  SD_UNLOCK();
  LogOut.printf("[STATE] %s : %s\n", path, ok ? "efface" : "rien a effacer");
  return ok;
}
int State_DeleteAll(void)
{
  int n = 0;
  for (uint8_t s = 0; s < 4; s++) if (State_Delete(s)) n++;
  return n;
}

// numéro de séquence d'un emplacement (0 = vide / illisible).
static uint32_t State_SlotSeq(uint8_t slot)
{
  char path[96];
  State_Path(path, sizeof path, slot);
  uint32_t seq = 0;
  File f = SD_MMC.open(path, FILE_READ);
  StateHdr h;
  if (f && f.read((uint8_t *)&h, sizeof h) == sizeof h && memcmp(h.magic, STATE_MAGIC, 8) == 0) seq = h.seq ? h.seq : 1;
  if (f) f.close();
  return seq;
}

// F5 tourne sur les 4 emplacements : un vide, sinon le plus ancien (plus petit numéro). F8 : le plus
// récent. -1 = aucun (F8).
static int State_PickSlot(bool for_save)
{
  if (!SD_MMC_MountOK) return for_save ? 0 : -1;
  int best = -1; uint32_t bseq = 0;
  SD_LOCK();
  for (uint8_t s = 0; s < 4; s++)
  {
    const uint32_t q = State_SlotSeq(s);
    if (for_save)
    {
      if (q == 0) { best = s; break; }
      if (best < 0 || q < bseq) { best = s; bseq = q; }
    }
    else if (q > bseq) { best = s; bseq = q; }
  }
  SD_UNLOCK();
  return best;
}

static uint32_t State_NextSeq(void)
{
  // Le plus grand numéro trouvé dans les 4 emplacements + 1.
  uint32_t best = 0;
  for (uint8_t s = 0; s < 4; s++)
  {
    char path[96];
    State_Path(path, sizeof path, s);
    File f = SD_MMC.open(path, FILE_READ);
    StateHdr h;
    if (f && f.read((uint8_t *)&h, sizeof h) == sizeof h && memcmp(h.magic, STATE_MAGIC, 8) == 0 && h.seq > best) best = h.seq;
    if (f) f.close();
  }
  return best + 1;
}

// Sauvegarde (coeur vidéo, CPU arrêté). wait_state = CPU_in_WAIT_STATE réel. tampons du Save State en
// PSRAM (alloués au premier usage) : ~7,8 Ko de RAM interne rendus
struct StateScratch { StateConf conf; FdcState fdcn; DiskAccessStruct dan; decltype(s_sdc) sdcn; };
static StateScratch *State_Scratch(void)
{
  static StateScratch *p = nullptr;
  if (!p) p = (StateScratch *)heap_caps_malloc(sizeof(StateScratch), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) p = (StateScratch *)malloc(sizeof(StateScratch));
  return p;
}

bool State_Save(uint8_t slot, bool wait_state)
{
  if (!SD_MMC_MountOK || slot > 3) return false;
  if (!State_WaitParked()) { LogOut.println("[STATE] processeur non arrete : sauvegarde annulee"); return false; }
  Disk_FlushAll();                                   // les disquettes de la carte SD suivent l'état
  char path[96], tmp[100];
  State_Path(path, sizeof path, slot);
  snprintf(tmp, sizeof tmp, "%s.TMP", path);
  const uint32_t t0 = millis();
  SD_LOCK();
  if (!SD_MMC.exists(STATE_DIR)) SD_MMC.mkdir(STATE_DIR);
  StateHdr h;
  memset(&h, 0, sizeof h);
  memcpy(h.magic, STATE_MAGIC, 8);
  h.version = STATE_VERSION; h.hdr_size = sizeof h;
  strncpy(h.fw, FW_VERSION_SHORT, sizeof h.fw);
  h.ram_size = gime_ram_size();
  h.cpu6309 = g_cpu6309_run ? 1 : 0;
  h.seq = State_NextSeq();
  h.frames = gime_raster.frame;
  State_SoftName(h.soft, sizeof h.soft);
  File f = SD_MMC.open(tmp, FILE_WRITE);
  bool ok = (bool)f;
  // Configuration
  StateScratch *sc = State_Scratch();
  if (!sc) { f.close(); SD_UNLOCK(); return false; }
  StateConf &conf = sc->conf;                        // 3,5 Ko : hors pile, en PSRAM
  memset(&conf, 0, sizeof conf);
  conf.slots = g_slotcfg_run;
  for (int i = 0; i < 4; i++) memcpy(conf.disk[i], Disk_Drive.Name_Disk[i], 256);
  for (int i = 0; i < VD_UNITS; i++) memcpy(conf.vd[i], g_dw_name[i], 256);
  // État divers
  StateMisc m;
  memset(&m, 0, sizeof m);
  m.wait_state = wait_state; m.irq_line = cpu_irq_line; m.firq_line = cpu_firq_line;
  m.cb1_flag = s_cb1_flag; m.rs_irq = s_rs_irq; m.cart_line = s_cart_line; m.mpi_reg = g_mpi_reg; m.scs_slot = g_scs_slot;
  memcpy(m.slot_bank, g_slot_bank, sizeof m.slot_bank); memcpy(m.cpsg_bank, g_cpsg_bank, 2);
  m.cpsg_ctrl = g_cpsg_ctrl; m.cpsg_latch = g_cpsg_latch; memcpy(m.cpsg_reg, g_cpsg_reg, 16);
  m.o90_l = g_o90_l; m.o90_r = g_o90_r;
  memcpy(m.pia_ddr, g_pia_ddr, 4);
  m.vhd_lsn = s_vhd_lsn; m.vhd_buf = s_vhd_buf; m.vhd_status = s_vhd_status; m.vhd_sel = s_vhd_sel;
  cpu_t::snapshot_t cs;
  cpu->save_state(cs);
  uint8_t gbuf[256];
  const uint32_t gsz = (uint32_t)gime_state_size();
  if (gsz > sizeof gbuf) ok = false; else gime_state_save(gbuf);
  ok = ok && St_W(f, &h, sizeof h)
          && St_Sec(f, "CONF", &conf, sizeof conf)
          && St_Sec(f, "CPU ", &cs, sizeof cs)
          && St_Sec(f, "GIME", gbuf, gsz)
          && St_Sec(f, "SF  ", &sf, sizeof sf)
          && St_Sec(f, "MISC", &m, sizeof m)
          && St_Sec(f, "FDC ", &fdc, sizeof fdc)
          && St_Sec(f, "DSKA", &DiskAccess, sizeof DiskAccess)
          && St_Sec(f, "SDC ", &s_sdc, sizeof s_sdc)
          && St_Sec(f, "ROMI", rom, 32768);
  // RAM : blocs de 1 Ko ; 0 + 1 024 octets bruts, ou 1 + l'octet répété.
  const uint32_t rsz = gime_ram_size();
  const uint8_t *ram = gime_ram_ptr();
  ok = ok && St_W(f, "RAM ", 4) && St_W(f, &rsz, 4);
  for (uint32_t o = 0; ok && o < rsz; o += 1024)
  {
    const uint8_t *b = ram + o;
    bool same = true;
    for (int i = 1; i < 1024 && same; i++) same = (b[i] == b[0]);
    const uint8_t hd[2] = { (uint8_t)(same ? 1 : 0), b[0] };
    ok = same ? St_W(f, hd, 2) : (St_W(f, hd, 1) && St_W(f, b, 1024));
  }
  ok = ok && St_W(f, "END ", 4);
  if (f) f.close();
  if (ok)
  {
    SD_MMC.remove(path);
    ok = SD_MMC.rename(tmp, path);
  }
  else SD_MMC.remove(tmp);
  SD_UNLOCK();
  LogOut.printf("[STATE] sauvegarde %s %s (#%lu, %lu ms)\n", path, ok ? "OK" : "ECHEC", (unsigned long)h.seq, (unsigned long)(millis() - t0));
  if (ok) State_RememberName(h.soft);
  return ok;
}

// Chargement (coeur vidéo, CPU arrêté). *wait_state reçoit l'état SYNC/CWAI sauvegardé. err : message
// court (anglais, affiché) en cas de refus.
bool State_Load(uint8_t slot, bool *wait_state, char *err, size_t en)
{
  snprintf(err, en, "LOAD FAILED");
  if (!SD_MMC_MountOK || slot > 3) { snprintf(err, en, "SD CARD MISSING"); return false; }
  if (!State_WaitParked()) { snprintf(err, en, "CPU BUSY, TRY AGAIN"); return false; }
  char path[96];
  State_Path(path, sizeof path, slot);
  StateScratch *sc = State_Scratch();
  if (!sc) { snprintf(err, en, "OUT OF MEMORY"); return false; }
  StateConf &conf = sc->conf;                        // en PSRAM
  StateHdr h;
  SD_LOCK();
  File f = SD_MMC.open(path, FILE_READ);
  bool ok = f && f.read((uint8_t *)&h, sizeof h) == sizeof h && memcmp(h.magic, STATE_MAGIC, 8) == 0 && h.version == STATE_VERSION;
  if (!f) snprintf(err, en, "SLOT %u IS EMPTY", (unsigned)slot + 1);
  else if (!ok) snprintf(err, en, "NOT A VALID STATE FILE");
  if (ok && h.ram_size != gime_ram_size())
  { snprintf(err, en, "NEEDS %lu KB RAM (MENU E)", (unsigned long)(h.ram_size / 1024)); ok = false; }
  if (ok && (h.cpu6309 != 0) != g_cpu6309_run)
  { snprintf(err, en, "NEEDS %s (MENU C)", h.cpu6309 ? "HD6309" : "MC6809"); ok = false; }
  if (ok) ok = St_RSec(f, "CONF", &conf, sizeof conf);
  if (ok)
  {
    // Les ROM de cartouche ne sont pas dans le fichier : slots identiques exigés.
    const SlotConfig &a = conf.slots, &r = g_slotcfg_run;
    bool same = (a.mpi == r.mpi);
    for (int i = 0; i < MPI_SLOTS && same; i++)
      same = (a.mode[i] == r.mode[i]) && (!SLOT_HAS_ROM(a.mode[i]) || strcmp(a.name[i], r.name[i]) == 0);
    if (!same) { snprintf(err, en, "CARTRIDGE SLOT DIFFERS"); ok = false; }
  }
  if (!ok) { if (f) f.close(); SD_UNLOCK(); return false; }
  // Lire tout ce qui est petit avant de toucher à la machine.
  cpu_t::snapshot_t cs;
  uint8_t gbuf[256];
  const uint32_t gsz = (uint32_t)gime_state_size();
  SpecialFunctionStruct sfn;
  StateMisc m;
  FdcState &fdcn = sc->fdcn;                         // en PSRAM
  DiskAccessStruct &dan = sc->dan;
  decltype(s_sdc) &sdcn = sc->sdcn;
  ok = gsz <= sizeof gbuf
       && St_RSec(f, "CPU ", &cs, sizeof cs)
       && St_RSec(f, "GIME", gbuf, gsz)
       && St_RSec(f, "SF  ", &sfn, sizeof sfn)
       && St_RSec(f, "MISC", &m, sizeof m)
       && St_RSec(f, "FDC ", &fdcn, sizeof fdcn)
       && St_RSec(f, "DSKA", &dan, sizeof dan)
       && St_RSec(f, "SDC ", &sdcn, sizeof sdcn);
  if (!ok) { f.close(); SD_UNLOCK(); snprintf(err, en, "DAMAGED STATE FILE"); return false; }
  const uint32_t pos_rom = (uint32_t)f.position();
  f.close();
  SD_UNLOCK();
  // Disques : remontés d'après le fichier s'ils diffèrent (hors verrou SD).
  for (int i = 0; i < 4; i++)
  {
    conf.disk[i][255] = 0;
    if (strcmp(conf.disk[i], (const char *)Disk_Drive.Name_Disk[i]) != 0 && conf.disk[i][0])
    {
      memcpy(Disk_Drive.Name_Disk[i], conf.disk[i], 256);
      if (!ReadCoCoFile(conf.disk[i], (uint8_t)i)) LogOut.printf("[STATE] disquette %d introuvable : %s\n", i, conf.disk[i]);
    }
  }
  for (int i = 0; i < VD_UNITS; i++)
  {
    conf.vd[i][255] = 0;
    if (strcmp(conf.vd[i], g_dw_name[i]) != 0 && conf.vd[i][0]) DW_Mount((uint8_t)i, conf.vd[i]);
  }
  // ROM interne + page des E/S, puis la RAM.
  SD_LOCK();
  f = SD_MMC.open(path, FILE_READ);
  ok = f && f.seek(pos_rom) && St_RSec(f, "ROMI", rom, 32768);
  uint32_t rsz = 0;
  char t[4];
  ok = ok && f.read((uint8_t *)t, 4) == 4 && memcmp(t, "RAM ", 4) == 0 && f.read((uint8_t *)&rsz, 4) == 4 && rsz == gime_ram_size();
  uint8_t *ram = gime_ram_ptr();
  for (uint32_t o = 0; ok && o < rsz; o += 1024)
  {
    uint8_t hd[2];
    if (f.read(hd, 1) != 1) { ok = false; break; }
    if (hd[0] == 1) { ok = f.read(hd + 1, 1) == 1; if (ok) memset(ram + o, hd[1], 1024); }
    else ok = f.read(ram + o, 1024) == 1024;
  }
  if (f) f.close();
  SD_UNLOCK();
  if (!ok) { snprintf(err, en, "DAMAGED STATE: PLEASE REBOOT"); return false; }
  // Appliquer.
  cpu->load_state(cs);
  gime_state_load(gbuf);
  {
    const SpecialFunctionStruct keep = sf;
    sf = sfn;
    sf.DIRECT_Key_Code = keep.DIRECT_Key_Code;
    sf.CPU_HALTED_BY_EMULATOR = keep.CPU_HALTED_BY_EMULATOR;
    sf.PHYSICAL_Drive_Must_Be_Saved = false;
    sf.Artefact = keep.Artefact;
    sf.CycleAccurateMode = keep.CycleAccurateMode;
    sf.VideoEmulatorXpixels = keep.VideoEmulatorXpixels;
    sf.DiskROM_Present = keep.DiskROM_Present;
    sf.Coco2VideoGenMODE = 255;                      // rendu legacy : effacement forcé
  }
  cpu_irq_line = m.irq_line; cpu_firq_line = m.firq_line;
  s_cb1_flag = m.cb1_flag; s_rs_irq = m.rs_irq; s_cart_line = m.cart_line;
  g_mpi_reg = m.mpi_reg;
  memcpy(g_slot_bank, m.slot_bank, sizeof g_slot_bank); memcpy(g_cpsg_bank, m.cpsg_bank, 2);
  g_cpsg_ctrl = m.cpsg_ctrl; g_cpsg_latch = m.cpsg_latch; memcpy(g_cpsg_reg, m.cpsg_reg, 16);
  g_o90_l = m.o90_l; g_o90_r = m.o90_r;
  memcpy(g_pia_ddr, m.pia_ddr, 4);
  if (g_pia_ddr[3] == 0)                    // état d'avant la DDR du BASIC, $FF20 = données
  {
    g_pia_ddr[3] = 0xFE;
    rom[0xFF21 - ROM_OFFSET] |= 0x04;
  }
  s_vhd_lsn = m.vhd_lsn; s_vhd_buf = m.vhd_buf; s_vhd_status = m.vhd_status; s_vhd_sel = m.vhd_sel;
  fdc = fdcn;
  DiskAccess = dan;
  s_sdc = sdcn;
  Slots_Apply();                                     // ROM visible, banques, ligne CART
  Audio_PsgEnable(g_gmc_present);                    // SN76489 : registres non sauvegardés -> muet
  if (g_cpsg_present)                                // YM2149 : registres rejoués
  {
    Audio_AyEnable(true);
    Audio_AyWrite(Snd_Stamp(), 0x10, g_cpsg_ctrl);
    for (uint8_t r = 0; r < 14; r++) Audio_AyWrite(Snd_Stamp(), r, g_cpsg_reg[r]);
  }
  if (g_rs_present) { Rs_Reset(); Rs_ApplyLink(); }
  Bb_Reset();
  NetDw_Flush();
  UpdateSoundOutput();
  Audio_Resync();
  g_resync_cpu_clock = true;
  *wait_state = m.wait_state != 0;
  // les sauvegardes suivantes gardent le nom de l'état rechargé jusqu'à la prochaine lecture de disque
  // par le CoCo.
  snprintf(g_state_name_loaded, sizeof g_state_name_loaded, "%.*s", (int)sizeof h.soft, h.soft);
  g_state_last_disk = -1;
  State_RememberName(g_state_name_loaded);
  LogOut.printf("[STATE] chargement %s OK (#%lu, %s)\n", path, (unsigned long)h.seq, h.soft);
  return true;
}

// Raccourcis F5 (sauvegarde) / F8 (chargement) de l'emplacement courant depuis le coeur vidéo.
// L'émulation est suspendue pendant l'opération.
static void State_Hotkey(bool save)
{
  g_rtc_step = STEP_HOTKEY;
  const uint8_t key = sf.DIRECT_Key_Code;
  bool w = CPU_in_WAIT_STATE;
  CPU_in_WAIT_STATE = true;
  sf.CPU_HALTED_BY_EMULATOR = true;
  Audio_SetPaused(true);
  char l1[OVL_COLS + 1], l2[OVL_COLS + 1];
  char nm[48];
  State_SoftName(nm, sizeof nm);
  const int pick = State_PickSlot(save);              // rotation (F5) / plus récent (F8)
  if (pick < 0)
  {
    snprintf(l1, sizeof l1, "NO SAVED STATE");
    snprintf(l2, sizeof l2, "for %.*s", OVL_COLS - 4, nm);
    Overlay_Request(l1, l2, 3000);
    Audio_SetPaused(false);
    CPU_in_WAIT_STATE = w;
    sf.CPU_HALTED_BY_EMULATOR = false;
    g_resync_cpu_clock = true;
    while (sf.DIRECT_Key_Code == key) vTaskDelay(10);
    g_rtc_step = STEP_IDLE;
    return;
  }
  g_state_slot = (uint8_t)pick;
  snprintf(l1, sizeof l1, "%s STATE %u...", save ? "SAVING" : "LOADING", (unsigned)g_state_slot + 1);
  Overlay_Set(0, l1); Overlay_Set(1, "Please wait"); Overlay_Set(2, nullptr); Overlay_Set(3, nullptr);
  Overlay_Draw();
  bool ok;
  char err[40];
  if (save) ok = State_Save(g_state_slot, w);
  else      ok = State_Load(g_state_slot, &w, err, sizeof err);
  snprintf(l1, sizeof l1, "STATE %u %s", (unsigned)g_state_slot + 1, ok ? (save ? "SAVED" : "LOADED") : (save ? "NOT SAVED" : "NOT LOADED"));
  if (ok) snprintf(l2, sizeof l2, "%.*s", OVL_COLS, nm);
  else     snprintf(l2, sizeof l2, "%s", save ? "SD card missing or full" : err);
  Overlay_Request(l1, l2, ok ? 1200 : 3000);
  Audio_SetPaused(false);
  CPU_in_WAIT_STATE = w;
  sf.CPU_HALTED_BY_EMULATOR = false;
  g_resync_cpu_clock = true;
  while (sf.DIRECT_Key_Code == key) vTaskDelay(10);
  g_rtc_step = STEP_IDLE;
}
void State_HotkeyPublic(bool save) { State_Hotkey(save); }
