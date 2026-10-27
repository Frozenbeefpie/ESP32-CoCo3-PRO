// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Menu F12 et navigateur de fichiers.
// Auteur : Luc Dion, inspiré de ESP32-CoCo2 de Cedric Beaudoin.

#ifndef __EMU_MENU_H
#define __EMU_MENU_H

#include <Arduino.h>
#include "VGA.h"

extern VGA* vga;
extern void line(int x0, int y0, int x1, int y1, int rgb);

void MENU_DisplayGIMEchar(uint8_t charNum, uint8_t ODDbyte, uint16_t Xpos, uint16_t Ypos, uint8_t ForeColor, uint8_t BackColor);
void DrawText(const char *text, uint8_t Xpos, uint8_t Ypos, uint8_t XpixOffset, uint8_t YpixOffset, uint8_t ForeColor, uint8_t BackColor);
void EMULATOR_Menu(void);
void EMU_Draw_Menu(void);
void BackDisplay(void);
void RestoreDisplay(void);
void PopulateDiskContent(const char *FileToLoad);
void PrintFileName(uint8_t LoopFile);
void DiskMenuChoose(void);
void DiskMenuChoose_1(uint8_t DriveNumber);
void ClearFileList(void);
void ClearFileBuffer(void);
uint8_t DrawFiles(uint16_t FileNumber, uint8_t ForeColor, uint8_t BackColor);
void DisplayDiskContent(void);
int8_t FillFileBuffer(uint16_t startIndex, int8_t MaxIndex, const char* fileExt);
void DrawFrames(void);
void DrawMenuDiskChoose(void);
void DrawDiskMenuChoose_1(void);
void DrawFirmwareUpdateMenuChoose(void);
char* Firmware_Choose(void);

// =====================================================================
// CONFIGURATION DES MANETTES USB (Right/Left, Analog/Digital, mapping axes/boutons) -- persistée sur
// SD (/joyconfig.ccc), configurable via menu F12 -> J
// =====================================================================
#define JOY_SLOT_RIGHT 0
#define JOY_SLOT_LEFT  1
#define JOY_MODE_DIGITAL 0
#define JOY_MODE_ANALOG  1

struct JoystickDeviceConfig
{
  bool configured;
  uint16_t vid;
  uint16_t pid;
  uint8_t mode;         // JOY_MODE_DIGITAL / JOY_MODE_ANALOG (provision future)
  uint8_t xByte;
  bool xInvert;
  uint8_t yByte;
  bool yInvert;
  uint8_t fire1Byte;
  uint8_t fire1Mask;    // 0xFF si le byte entier sert de valeur sinon bit isolé
  uint8_t fire2Byte;
  uint8_t fire2Mask;
};

// Défini/possédé par main.cpp (lecture USB), utilisé en lecture/écriture par l'assistant de
// configuration dans EmuMenu.cpp.
extern JoystickDeviceConfig JoystickCfg[2]; // [JOY_SLOT_RIGHT], [JOY_SLOT_LEFT]
extern bool SaveJoystickConfigToSD(void);

// API de lecture "live" des devices HID connectés, exposée par main.cpp pour l'assistant de
// configuration (liste des manettes disponibles, dernier rapport reçu, détection du dernier bytes
// changé).
extern uint8_t JoyWizard_GetDeviceCount(void);
extern bool JoyWizard_GetDeviceInfo(uint8_t index, uint16_t &vid, uint16_t &pid, uint8_t &address);
extern bool JoyWizard_GetSnapshot(uint8_t address, uint8_t *outData, uint8_t *outLen, uint8_t *outChangedByte);
extern void JoyWizard_ResetChangeDetector(uint8_t address);

void JoystickMenuChoose(void);

// =====================================================================
// menu USB DEVICES : appareils en ligne et mémorisés (usbseen.cfg) affectation des ports JOY (R) / JOY
// (L) et du clavier du CoCo
// =====================================================================
#define UDEV_KBD   1
#define UDEV_MOUSE 2
#define UDEV_PAD   3
#define UTAG_AUTO  0              // reconnu (Xbox, manette connue, souris)
#define UTAG_SET   1              // réglé par l'assistant
#define UTAG_NEW   2              // manette inconnue, jamais réglée
#define UTAG_KBD   3              // clavier
#define UPORT_AUTO  0             // première manette connue branchée (comme avant)
#define UPORT_KBD   1
#define UPORT_MOUSE 2
#define UPORT_PAD   3
struct UsbDevView
{
  uint8_t  type, tag, pad, npads;  // pad = interface + 1 (manettes) ; npads = manettes de l'appareil
  uint16_t vid, pid;
  bool     online;
  uint8_t  key;                    // clé de suivi HID (assistant), 0 hors ligne
  char     name[26];
  uint8_t  nochan;                 // branché, mais plus de canal USB (pas encore actif)
};
int  UsbDev_List(UsbDevView *out, int max, int *nOnline);        // en ligne, puis hors ligne (8 max)
void UsbDev_Forget(const UsbDevView &d);
bool UsbDev_SameDev(const UsbDevView &a, const UsbDevView &b);
uint8_t UsbPort_Kind(uint8_t port);                               // UPORT_*
bool UsbPort_Device(uint8_t port, UsbDevView &d);                 // appareil affecté (false : AUTO)
int  UsbPort_Choices(uint8_t port, UsbDevView *out, int max);    // [0] = AUTO (type 0)
void UsbPort_Set(uint8_t port, const UsbDevView &d);             // type 0 = AUTO
void UsbPort_StoreMap(uint8_t port);                             // après l'assistant : réglages -> appareil
bool UsbKbd_Get(UsbDevView &d);                                   // clavier du CoCo (false : aucun)
int  UsbKbd_Choices(UsbDevView *out, int max);                    // claviers en ligne
void UsbKbd_Set(const UsbDevView &d);
bool UsbKbd_Changed(void);                                        // clavier du CoCo absent, un autre en ligne
void UsbDev_SaveIfDirty(void);
extern uint8_t g_kbjoy_set2;                                      // jeu de touches du port gauche
bool JoyWizard_ForDevice(uint8_t slot, const UsbDevView &d);      // assistant sans choix du port / appareil

#endif
