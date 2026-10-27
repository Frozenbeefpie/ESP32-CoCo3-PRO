// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Définitions communes, version du firmware.
// Auteur : Luc Dion 2026, inspiré de ESP32-CoCo2 de Cedric Beaudoin.

#ifndef __MAIN_H
#define __MAIN_H

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "SD_MMC.h"
#include "EmuMenu.h"
#include "ROMS_Source.h"
#include <EEPROM.h>
//----------------------
#define VSYNC_PORT GPIO_NUM_1
#define HSYNC_PORT GPIO_NUM_2

#define PSRAM_EMU

//------------
#ifdef PRINT_DEBUG
#define debug Serial.print
#define debugln Serial.println
#define debugf Serial.printf
#else
#define debug
#define debugln
#define debugf
#endif

// Core ESP32-Arduino 3.x (ESP-IDF 5.x) --- Direct GPIO.out_w1ts / GPIO.out_w1tc) n'est plus disponible
#include "soc/gpio_reg.h"

#define DEBUG1_SET REG_WRITE(GPIO_OUT_W1TS_REG, (1 << DEBUG1))
#define DEBUG1_CLR REG_WRITE(GPIO_OUT_W1TC_REG, (1 << DEBUG1))

#define MACRO_DO_CASSETTE_BIT_OUTPUT \
if ((value & 0b00000010) == 0) \
{ \
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << BOARD_CASSETTE_OUT));  /* Output 0 */ \
} \
else \
{ \
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << BOARD_CASSETTE_OUT));  /* Output 1 */ \
}

struct SpecialFunctionStruct
{
  uint8_t DIRECT_Key_Code;
  bool CPU_HALTED_BY_EMULATOR; // CPU Halted because the Emulator is in menus.
  bool PHYSICAL_Drive_Must_Be_Saved; // To Save VIrtual Disk to Physical SD Card.
  bool nmi_pin; // Mapped to CPU pîn NMI
  bool firq_pin; // Mapped to CPU pîn FIRQ
  bool irq_pin; // Mapped to CPU pîn IRQ
  bool V_Synch;
  bool V_Synch_Int_Enabled;
  uint16_t ROM_Offset; // Rom address offset when executing from ROM address.
  bool AnyKeypress;   // Global VAR for any keypress check
  bool CoCo2Mode;   // True at boot
  bool CoCo2VideoMode;
  bool CPU_Speed;
  uint8_t Coco2VideoGenMODE;
  uint8_t Coco2GraphicMode;
  // DiskROMOverlay_Active retirés — mode ROM/RAM = gime.ty, ROM cartouche cartographiée par le GIME,
  // voir gime.h.)
  bool DiskROM_Present;        // Virtual Disk Controller connected (always true for now)
  uint8_t Coco2ColorMode;
  uint8_t Coco2VideoPageOffset_Registers;   // 7 bits data of SET/CLR only offset register to do mult to final…
  uint16_t VideoEmulatorXpixels;   // define the number of X pixel to center the Emulation screen.
  bool is_JOY1_B1_WasPressed;
  bool is_JOY1_B2_WasPressed;
  bool is_JOY2_B1_WasPressed;
  bool is_JOY2_B2_WasPressed;
  bool is_LastKeyboardScanned;
  bool Artefact;
  bool CycleAccurateMode;   // true = clock réel 6809 ; false = overclock (vitesse max)

};

struct DriveStruct
{
  uint8_t Name_Disk[4][256];
  uint8_t LastNumber_Accessed;
  uint8_t LastAccessType; // Read Write
  bool Error;
  bool isFileAlreadyOpen;
  uint8_t DriveNumber;
};

// version abrégée du firmware (menu F12, ligne [VERSION]).
#define FW_VERSION_SHORT "1.0"
#define FW_VERSION_DATE  "2026-10-26"


#define SD_MMC_CMD 38 // Please do not modify it.
#define SD_MMC_CLK 39 // Please do not modify it.
#define SD_MMC_D0  40 // Please do not modify it.

//------------- Keys definition for Emulator in Custom Menu

#define MENU_F12 69
#define MENU_UP 82
#define MENU_DOWN 81
#define MENU_LEFT 80
#define MENU_RIGHT 79   // flèche droite (HID 0x4F ; avant 70 = Impr. écran)
#define MENU_ESC 41
#define MENU_1 30
#define MENU_2 31
#define MENU_3 32
#define MENU_4 33
#define MENU_5 34
#define MENU_6 35
#define MENU_7 36
#define MENU_8 37
#define MENU_9 38
#define MENU_0 39
#define MENU_ENTER 40
#define MENU_PGDWN 78
#define MENU_PGUP 75
#define MENU_R 21   // Reboot
#define MENU_F 9    // Firmware Upgrade
#define MENU_D 7    // Disk Drive Menu
#define MENU_Y 28   // Yes
#define MENU_N 17   // No
#define MENU_A 4    // Artifact Colors On/Off taggle
#define MENU_O 18   // Overclock / Cycle Accurate toggle
#define MENU_C 6    // processeur MC6809 / HD6309

//---------------------------------------------------------------

void DoCPU(void);

#define JOY1_X_AN_PIN 4  // Joy Right X
#define JOY1_Y_AN_PIN 5  // Joy Right Y
#define JOY2_X_AN_PIN 6  // Joy Left X
#define JOY2_Y_AN_PIN 10 // Joy Left Y
#define JOY1_B1 46
#define JOY1_B2 17
#define JOY2_B1 3
#define JOY2_B2 45

#define JOY_RX 0
#define JOY_RY 1
#define JOY_LX 2
#define JOY_LY 3

//----------- Videos modes for the emulator
#define VIDEO_MODE_320X240_16_9 0
#define VIDEO_MODE_320X240_4_3 1
#define VIDEO_MODE_640X240_16_9 2
#define VIDEO_MODE_640X240_4_3 3

#define COCO2_GRAPH_OFFSET_LEN 512
// For Rom access
#define ROM_OFFSET 0x8000

#define ROM_FF00 0xff00 - ROM_OFFSET
#define ROM_FF01 0xff01 - ROM_OFFSET
#define ROM_FF03 0xff03 - ROM_OFFSET
#define ROM_FF02 0xff02 - ROM_OFFSET
#define ROM_FF03 0xff03 - ROM_OFFSET

#define ROM_FF20 0xff20 - ROM_OFFSET
#define ROM_FF22 0xff22 - ROM_OFFSET
#define ROM_FF23 0xff23 - ROM_OFFSET
// DISK ACCESS
#define ROM_FF40 0xff40 - ROM_OFFSET
#define ROM_FF48 0xff48 - ROM_OFFSET
#define ROM_FF49 0xff49 - ROM_OFFSET
#define ROM_FF4A 0xff4a - ROM_OFFSET
#define ROM_FF4B 0xff4b - ROM_OFFSET

#define ROM_FF92 0xff92 - ROM_OFFSET
#define ROM_FF93 0xff93 - ROM_OFFSET

// For Switch Case
#define M_FF00 0xff00
#define M_FF01 0xff01
#define M_FF02 0xff02
#define M_FF03 0xff03
#define M_FF20 0xff20

#define M_FF22 0xff22
#define M_FF23 0xff23

// Disk access
#define M_FF40 0xff40
#define M_FF48 0xff48
#define M_FF49 0xff49
#define M_FF4A 0xff4a
#define M_FF4B 0xff4b

#define M_FF92 0xff92
#define M_FF93 0xff93

#define M_FFC0 0xffc0
#define M_FFC1 0xffc1
#define M_FFC2 0xffc2
#define M_FFC3 0xffc3
#define M_FFC4 0xffc4
#define M_FFC5 0xffc5
#define M_FFC6 0xffc6
#define M_FFC7 0xffc7
#define M_FFC8 0xffc8
#define M_FFC9 0xffc9
#define M_FFCA 0xffca
#define M_FFCB 0xffcb
#define M_FFCC 0xffcc
#define M_FFCD 0xffcd
#define M_FFCE 0xffce
#define M_FFCF 0xffcf
#define M_FFD0 0xffd0
#define M_FFD1 0xffd1
#define M_FFD2 0xffd2
#define M_FFD3 0xffd3
#define M_FFD4 0xffd4
#define M_FFD5 0xffd5
#define M_FFD8 0xffd8
#define M_FFD9 0xffd9
#define M_FFDE 0xffde // ROM 32K
#define M_FFDF 0xffdf // RAM 32K UPPER

#define CPU_FAST true // Double Speed (~1.79 MHz)
#define CPU_SLOW false // Single Speed (~0.89 MHz)

#define VDG_BLACK       0x0000      /* 0, 0, 0 */
#define VDG_NAVY        0x000F      /* 0, 0, 128 */
#define VDG_DARKGREEN   0x03E0      /* 0, 128, 0 */
#define VDG_DARKCYAN    0x03EF      /* 0, 128, 128 */
#define VDG_MAROON      0x7800      /* 128, 0, 0 */
#define VDG_PURPLE      0x780F      /* 128, 0, 128 */
#define VDG_OLIVE       0x7BE0      /* 128, 128, 0 */
#define VDG_LIGHTGREY   0xD69A      /* 211, 211, 211 */
#define VDG_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define VDG_BLUE        0x001F      /* 0, 0, 255 */
#define VDG_GREEN       0x07E0      /* 0, 255, 0 */
#define VDG_CYAN        0x07FF      /* 0, 255, 255 */
#define VDG_RED         0xF800      /* 255, 0, 0 */
#define VDG_MAGENTA     0xF81F      /* 255, 0, 255 */
#define VDG_YELLOW      0xFFE0      /* 255, 255, 0 */
#define VDG_WHITE       0xFFFF      /* 255, 255, 255 */
#define VDG_ORANGE      0xFDA0      /* 255, 180, 0 */
#define VDG_GREENYELLOW 0xB7E0      /* 180, 255, 0 */
#define VDG_PINK        0xFE19      /* 255, 192, 203 */ // Lighter pink, was 0xFC9F
#define VDG_BROWN       0x9A60      /* 150, 75, 0 */
#define VDG_GOLD        0xFEA0      /* 255, 215, 0 */
#define VDG_SILVER      0xC618      /* 192, 192, 192 */
#define VDG_SKYBLUE     0x867D      /* 135, 206, 235 */
#define VDG_VIOLET      0x915C      /* 180, 46, 226 */

void CheckFirmwareUpdate(void);

void InitSD_Card(void);
void InitSD_Card1(void);
uint8_t GetDriveNumber(uint8_t Address);
void InitDisks(void);
uint8_t MountFileSystem(void);
void WriteDiskByte(uint32_t BytePos, uint8_t ByteData);
uint8_t ReadDiskByte(uint32_t BytePos);

bool ReadCoCoFile(const char* filename, uint8_t DriveNumber);
bool WriteCoCoFile(const char* filename, uint8_t DriveNumber);
bool SaveConfigToSD(void);
bool LoadConfigFromSD(void);

void InitPorts(void);
uint8_t ReadCoCoButtons(void);
uint8_t ReadJoysticks(uint8_t JoyNum);
void CopyDiskToRamDisk(void);
void DisplayVDGchar(uint8_t charNum, uint16_t Xpos, uint16_t Ypos);
void DisplayVDGcharLine(uint8_t charNum, uint8_t lir, uint16_t Xpos, uint16_t Ypos);
bool IsButtonPressed(void);
void ManagePeripherals_Write(uint16_t address, uint8_t value);
void ManagePeripherals_Read(uint16_t address);
void ManageKeyboardScan(uint8_t value);
void FillKeyboardMatrix(void);
void CopyCoCo2ROMS(void);
void CopyCoCo3ROMS(void);

void SetVideoMode(uint8_t VideoMode);
void VideoCore(void *pvParameters);
void line(int x0, int y0, int x1, int y1, int rgb);

void InitPeripherals_and_Others(void);

//--------------- Videos modes

void Do_COCO2_GRAPHMODE_32X16_8X12(void);
void Do_COCO2_GRAPHMODE_256X192X2(void);
void Do_COCO2_GRAPHMODE_256X192X2_ARTEFACT(void);
void Do_COCO2_GRAPHMODE_128X192X4(void);
void Do_COCO2_GRAPHMODE_128X192X2(void);
void Do_COCO2_GRAPHMODE_128X96X4(void);
void Do_COCO2_GRAPHMODE_128X96X2(void);
void Do_COCO2_GRAPHMODE_128X64X4(void);
void Do_COCO2_GRAPHMODE_128X64X2(void);
void Do_COCO2_GRAPHMODE_64X64X4(void);

#define DEBUG1 13
#define DEBUG2 13

// trois réglages d'Overclock (menu F12, touche O) OC_OFF : 0,89 / 1,79 MHz exacts, 60 images/s (cycle
// accurate) OC_CPU : 60 images/s, horloge juste
#define OC_OFF     0
#define OC_CPU     1
#define OC_MACHINE 2
extern uint8_t g_oc_mode;
void SetOverclockMode(uint8_t mode);

#endif

// port d'extension, Multi-Pak (voir main.cpp, g_slotcfg)
#define SLOT_FDC   0
#define SLOT_ROM   1
#define SLOT_EMPTY 2
#define SLOT_O90   3              // Orchestra-90 (ROM + 2 CNA 8 bits en $FF7A/$FF7B)
#define SLOT_GMC   4              // Game Master Cartridge (ROM à banques + SN76489A en $FF41)
#define SLOT_CPSG  5              // CoCo PSG (Ed Snider : YM2149 en $FF5E/$FF5F, banques de 8 Ko en…)
#define SLOT_RS232 6              // RS-232 Pak (ACIA 6551 en $FF68-$FF6B, IRQ sur la ligne CART)
#define SLOT_LAST  SLOT_RS232
#define SLOT_HAS_ROM(m) ((m) == SLOT_ROM || (m) == SLOT_O90 || (m) == SLOT_GMC || (m) == SLOT_CPSG)
#define MPI_SLOTS  4
#include <stdint.h>
struct SlotConfig
{
  uint8_t mpi;                    // 0 = une cartouche (slot 0), 1 = Multi-Pak (4 slots)
  uint8_t sw;                     // commutateur du Multi-Pak : slot 0-3 à l'allumage
  uint8_t mode[MPI_SLOTS];        // SLOT_FDC / SLOT_ROM / SLOT_EMPTY / SLOT_O90
  uint8_t autostart[MPI_SLOTS];   // cartouche ROM : CART reliée à Q
  char    name[MPI_SLOTS][256];   // fichier .ROM / .CCC
};
extern SlotConfig g_slotcfg;      // choix du menu F12 S (sauvegardé)
extern SlotConfig g_slotcfg_run;  // en service (appliqué au démarrage)
extern bool     g_cart_clock;     // CART = Q (slot CTS : cartouche à démarrage automatique)
extern bool     g_ff40_fdc;       // $FF40-$FF5F -> contrôleur de disquette (slot SCS)
extern uint8_t  g_mpi_reg;        // registre $FF7F du Multi-Pak
extern bool     g_o90_present;    // une carte Orchestra-90 en service
extern bool     g_gmc_present;    // une Game Master Cartridge en service
extern bool     g_cpsg_present;   // une CoCo PSG en service

// journal de démarrage. 0 = version bêta
#include "boot_log.h"   // réglage BOOT_LOG_VERBOSE (un seul endroit)
#define BOOTLOG(...)  do { if (BOOT_LOG_VERBOSE) LogOut.printf(__VA_ARGS__); } while (0)   // LogOut = série + journal F9
#define BOOTLOGLN(s)  do { if (BOOT_LOG_VERBOSE) LogOut.println(s); } while (0)
