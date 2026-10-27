// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur: Luc Dion 2026-10-26

// Menu F12 et navigateur de fichiers (disques, cartouches, bandes).
// Auteur : Luc Dion, inspiré de ESP32-CoCo2 de Cedric Beaudoin.

#include "EmuMenu.h"
#include "main.h"
#include "ROMS_Source.h"
#include <cstring>  // memcpy, utilisé par l'outil de configuration joystick
#include "gime_video.h"
#include "gime.h"        // gime_ram_size
#include "rs232.h"       // RS-232 Pak (liaison, vitesse)
#include "audio.h"       // sortie son mono / stéréo
#include "bitbang.h"     // port série du CoCo (bit banger)
#include "modem.h"       // modem Hayes par le Wi-Fi
#include "cc3net.h"      // Wi-Fi
#include "webui.h"       // page web
#include "autorun.h"     // AUTORUN
#include "bootscreen.h"  // écran d'accueil
#include "health.h"      // état matériel
#include "netshare.h"    // dossier partagé d'un PC
#include "cassette.h"    // magnétophone virtuel
// Save State (main.cpp)
#ifndef MENU_V
#define MENU_V 0x19
#endif
extern uint8_t g_state_slot;
extern bool    g_wait_before_menu;
void State_Describe(uint8_t slot, char *out, size_t n);
void State_CurrentName(char *out, size_t n);
int  State_NextSoftware(void);
bool State_Save(uint8_t slot, bool wait_state);
bool State_Load(uint8_t slot, bool *wait_state, char *err, size_t en);
static void StateMenuChoose(void);
static void BitBangMenuChoose(void);
static void NetMenuChoose(void);

// MENU_J n'existe pas dans main.h (les autres touches MENU_* -- D, A R, F, etc. -- y sont définies
// comme le code clavier USB HID brut de la lettre, ex: MENU_D=0x07)
#ifndef MENU_J
#define MENU_J 0x0D
#endif
// Mêmes codes USB HID bruts que ceux déjà utilisés dans fillKeysStruct (main.cpp) pour la traduction
// clavier CoCo -- Backspace=0x2A, Home=0x4A (USB HID Usage Tables)
#ifndef MENU_M
#define MENU_M 0x10
#ifndef MENU_Z
#define MENU_Z 0x1D    // code HID de 'Z'
#endif
#endif
// Touche E (extension mémoire 512 Ko / 2 Mo / 4 Mo) : code HID de 'E' = 0x08.
#ifndef MENU_E
#define MENU_E 0x08
#endif
extern uint32_t g_ram_capacity;
extern uint32_t g_ram_selected;
extern uint8_t g_artifact_swap;   // main.cpp, phase des couleurs d'artefact
#ifndef MENU_BACKSPACE
#define MENU_BACKSPACE 0x2A
#endif
#ifndef MENU_HOME
#define MENU_HOME 0x4A
#endif

extern uint8_t *MENU_Backup;
extern uint8_t *MENU_BackupPage2;
extern SpecialFunctionStruct sf;
extern volatile bool g_resync_cpu_clock;
extern DriveStruct Disk_Drive;
// disques virtuels DriveWire (main.cpp)
extern char g_dw_name[6][256];     // 0-3 DriveWire, 4-5 VHD
bool DW_Mount(uint8_t d, const char *path);
void DW_Unmount(uint8_t d);
extern volatile bool g_dw_boot_request;
bool State_Delete(uint8_t slot);
int  State_DeleteAll(void);
const char *CartLast_Get(uint8_t mode);
void CartLast_Remember(uint8_t mode, const char *path);
static bool s_menu_exit_all = false;      // B (amorçage DW) : quitte tout le menu F12
#ifndef MENU_B
#define MENU_B 0x05    // code HID de 'B'
#endif
#ifndef MENU_L
#define MENU_L 0x0F
#endif
extern bool g_lowercase_t1;
extern bool g_cpu6309_sel;        // processeur choisi (menu C)
extern bool g_cpu6309_run;        // processeur en service
#ifndef MENU_G
#define MENU_G 0x0A    // code HID de 'G'
#endif
#ifndef MENU_P
#define MENU_P 0x13    // code HID de 'P'
#endif
#define MENU_MINUS   0x2D   // '-'
#define MENU_EQUAL   0x2E   // '=' (+ sans SHIFT)
#define MENU_KP_MIN  0x56   // pavé numérique -
#define MENU_KP_PLUS 0x57   // pavé numérique +
extern uint8_t g_mouse_mode;
extern uint8_t g_kbjoy_port;     // manette au clavier
extern uint8_t g_kbjoy_set;
#ifndef MENU_K
#define MENU_K 0x0E    // code HID de 'K'
#endif
void KbJoyMenuChoose(void);
extern uint8_t g_mouse_sens;
extern bool    g_mouse_hires_pref;
void MouseMenuChoose(void);
#ifndef MENU_U
#define MENU_U 0x18    // code HID de 'U'
#endif
#ifndef MENU_S
#define MENU_S 0x16    // code HID de 'S'
#endif
void SlotMenuChoose(void);
void HelpMenuChoose(void);
#ifndef MENU_H
#define MENU_H 0x0B    // code HID de 'H'
#endif
#ifndef MENU_O
#define MENU_O 0x12    // code HID de 'O'
#endif
#ifndef MENU_W
#define MENU_W 0x1A    // code HID de 'W'
#endif
static void Menu_RebootNow(void);
static bool Slot_PendingReboot(void);
static bool Menu_RebootPending(void);
static void Menu_CancelPending(void);
static bool Menu_AskYN(const char *q);

extern void flashFromSD(const char* filename);
extern void InitFilesystem(void);
extern bool copyFile(const char* srcFilename, const char* destFilename);
extern bool ValidFirmwareFile(const char* filename);

#ifdef PSRAM_EMU
extern uint8_t *rom;
extern uint8_t *memory;
#else
extern uint8_t rom[32769];
extern uint8_t memory[65536];
#endif

struct FileArrayStruct
{
  uint8_t FileBuffer[31][255];
  int16_t FileStart;
  int16_t FileEnd;
  uint8_t FileList[69][11];
};
// tampon des listes de fichiers (8,7 Ko) en PSRAM, alloué au premier usage
static FileArrayStruct *FileArray_Get(void)
{
  static FileArrayStruct *p = nullptr;
  if (!p) p = (FileArrayStruct *)heap_caps_calloc(1, sizeof(FileArrayStruct), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) p = (FileArrayStruct *)calloc(1, sizeof(FileArrayStruct));
  return p;
}
#define FileArray (*FileArray_Get())

// =====================================================================
// touches des menus au front d'appui. Une pression = une action seules les flèches et PgUp / PgDn se
// répètent si on les tient (après 400 ms, puis toutes les 70 ms)
// =====================================================================
static uint8_t  s_key_prev = 0;
static uint32_t s_key_next_rep = 0;
static bool Menu_IsRepeatKey(uint8_t k)
{
    return k == MENU_PGUP || k == MENU_PGDWN;
}
static uint8_t Menu_GetKey(void)
{
    const uint8_t k = sf.DIRECT_Key_Code;
    const uint32_t now = millis();
    if (k != s_key_prev)
    {
        s_key_prev = k;
        s_key_next_rep = now + 400;
        return k;                                   // nouvel appui (0 = relâchée)
    }
    if (k && Menu_IsRepeatKey(k) && (int32_t)(now - s_key_next_rep) >= 0)
    {
        s_key_next_rep = now + 70;
        return k;                                   // répétition
    }
    return 0;
}
// Touche tenue actuelle = déjà vue (après un sous-menu qui lit les touches lui-même).
static void Menu_KeySync(void)
{
    s_key_prev = sf.DIRECT_Key_Code;
    s_key_next_rep = millis() + 400;
}

// =====================================================================
// NAVIGATION D'ARBORESCENCE -- menu Disk (F12 -> D -> 0..3)
// =====================================================================
// Chemin actuellement parcouru par le navigateur de fichiers de disque. Toujours "/" pour la root,
// sinon sans "/" final (ex: "/GAMES" ou "/GAMES/SHOOTERS")
static char CurrentDiskPath[128] = "/";
static bool s_browse_pc = false;     // navigateur sur le dossier partagé du PC (touche P)
static uint8_t s_browse_drive = 0;   // lecteur choisi (le PC seulement pour les disquettes 0-3)
static char s_pc_listed[128] = "";   // dossier du PC dont la liste est en cache
static bool s_browse_tape = false;   // navigateur ouvert pour la cassette (.CAS / .WAV)
static bool s_browse_cart = false;   // navigateur ouvert pour une cartouche (pas d'aperçu de disque)

static void DiskPath_GoToRoot(char *path, size_t pathSize)
{
    snprintf(path, pathSize, "/");
}

// dirName : nom du sous-dossier choisi, SANS le "/" de tête utilisé comme marqueur "ceci est un
// dossier" dans FileArray.FileBuffer[] (voir FillFileBuffer ci-dessous).
static void DiskPath_Descend(char *path, size_t pathSize, const char *dirName)
{
    char tmp[128];
    if (strcmp(path, "/") == 0)
        snprintf(tmp, sizeof(tmp), "/%s", dirName);
    else
        snprintf(tmp, sizeof(tmp), "%s/%s", path, dirName);
    strncpy(path, tmp, pathSize - 1);
    path[pathSize - 1] = '\0';
}

static void DiskPath_GoUp(char *path, size_t pathSize)
{
    if (strcmp(path, "/") == 0) return; // déjà à root
    char *lastSlash = strrchr(path, '/');
    if (lastSlash == nullptr || lastSlash == path)
    {
        // Le seul "/" restant est celui de tête -> retour à root.
        path[0] = '/';
        path[1] = '\0';
    }
    else
    {
        *lastSlash = '\0';
    }
}

// le nom finit-il par une des extensions « .ROM|.CCC » (sans tenir compte de la casse) ?
static bool Pc_ExtOk(const char *name, const char *exts)
{
    if (!exts || !exts[0]) return true;
    const size_t ln = strlen(name);
    const char *p = exts;
    while (*p)
    {
        const char *e = strchr(p, '|');
        const size_t le = e ? (size_t)(e - p) : strlen(p);
        if (le && ln >= le && strncasecmp(name + ln - le, p, le) == 0) return true;
        if (!e) break;
        p = e + 1;
    }
    return false;
}
// même rôle que FillFileBuffer, pour le dossier du PC (liste en cache) seuls les fichiers du bon type
// (disquettes ou cartouches) sont gardés.
static int8_t Pc_FillFileBuffer(uint16_t startIndex, int8_t MaxIndex, const char *path, const char *exts)
{
    if (strcmp(path, s_pc_listed) != 0)
    {
        if (NetShare_List(path) < 0) { s_pc_listed[0] = 0; return -1; }
        snprintf(s_pc_listed, sizeof s_pc_listed, "%s", path);
    }
    int8_t bufferIndex = 0;
    bool cleared = false;
    int seen = 0;                                  // rang parmi les entrées gardées
    for (int i = 0; i < NetShare_Count() && bufferIndex <= MaxIndex; i++)
    {
        bool dir; uint32_t size; const char *name;
        if (!NetShare_Entry(i, &dir, &size, &name)) break;
        if (!dir && !Pc_ExtOk(name, exts)) continue;
        if (seen++ < (int)startIndex) continue;
        if (!cleared)
        {
            for (int k = 0; k <= MaxIndex; k++) memset(FileArray.FileBuffer[k], 0, 255);
            cleared = true;
        }
        snprintf((char *)FileArray.FileBuffer[bufferIndex], 255, dir ? "/%s" : "%s", name);
        bufferIndex++;
    }
    return bufferIndex ? bufferIndex - 1 : -1;
}

int8_t FillFileBuffer(uint16_t startIndex, int8_t MaxIndex, const char* fileExt, const char* path)
{
    if (MaxIndex <= 0) return -1; // Pas de fichiers à charger
    if (s_browse_pc) return Pc_FillFileBuffer(startIndex, MaxIndex, path, fileExt);

    File root = SD_MMC.open(path);
    if (!root)
    {
#ifdef DEBUG_PRINT
        Serial.print("Failed to open directory: ");
        Serial.println(path);
#endif
        return -1;
    }

    if (!root.isDirectory())
    {
#ifdef DEBUG_PRINT
        Serial.print("Not a directory: ");
        Serial.println(path);
#endif
        root.close();
        return -1;
    }

    // getNextFileName lit seulement le dossier (readdir) openNextFile ouvrait chaque fichier (lent
    // dans un gros dossier : /SIERRA).
    uint16_t currentIndex = 0;
    int8_t bufferIndex = 0;
    bool bufferCleared = false; // Nouveau flag

    while (bufferIndex <= MaxIndex)
    {
        bool isDir = false;
        const String full = root.getNextFileName(&isDir);
        if (full.length() == 0) break;
        const char *name = strrchr(full.c_str(), '/');
        name = name ? name + 1 : full.c_str();
        bool validEntry = false;

        // === FILTRE DES ENTRÉES ===
        if (isDir)
        {
            // dossiers de l'émulateur masqués (réglages, journaux)
            const char *dn = strrchr(name, '/');
            dn = dn ? dn + 1 : name;
            validEntry = strcasecmp(dn, "ESP-CONFIG") != 0 && strcasecmp(dn, "ESP-DEBUG") != 0 && strcasecmp(dn, "ESP-STATES") != 0 &&
                         strcasecmp(dn, "System Volume Information") != 0;
        }
        else
        {
            // plusieurs extensions permises, séparées par « | » (".ROM|.CCC")
            const char* ext = strrchr(name, '.');
            if (ext)
            {
                const char *e = fileExt;
                while (*e)
                {
                    const char *bar = strchr(e, '|');
                    const size_t len = bar ? (size_t)(bar - e) : strlen(e);
                    if (strlen(ext) == len && strncasecmp(ext, e, len) == 0) { validEntry = true; break; }
                    if (!bar) break;
                    e = bar + 1;
                }
            }
        }
        // ==========================

        if (currentIndex >= startIndex && validEntry)
        {
            // Clear buffer une seule fois avant de remplir
            if (!bufferCleared)
            {
                for (int i = 0; i <= MaxIndex; i++)
                {
                    memset(FileArray.FileBuffer[i], 0, 255);
                }
                bufferCleared = true;
            }

            if (isDir)
            {
                snprintf((char*)FileArray.FileBuffer[bufferIndex],
                         255,
                         "/%s",
                         name);
            }
            else
            {
                snprintf((char*)FileArray.FileBuffer[bufferIndex],
                         255,
                         "%s",
                         name);
            }

            bufferIndex++;
        }

        // seules les entrées affichées comptent. du dossier (.BIN, .TXT...) étaient comptés, donc
        // FileStart + 11 ne tombait pas sur la page suivante ; une fenêtre sans fichier valide
        if (validEntry) currentIndex++;
    }

    root.close();

#ifdef DEBUG_PRINT
    Serial.print("Loaded ");
    Serial.print(bufferIndex);
    Serial.println(" entries into FileBuffer");
#endif

    if (bufferIndex == 0)
        return -1; // Aucun fichier chargé

    return bufferIndex - 1; // Dernier index rempli
}

// aperçu du répertoire RS-DOS (piste 17, secteurs 3-11) lu en une seule lecture de 2 304 octets Fin de
// répertoire ($FF) et entrées effacées ($00) respectées
static char s_preview_path[192] = "";   // disque dont l'aperçu est affiché ("" = aucun)
void PopulateDiskContent(const char *FileToLoad)
{
    char tmpBuf[192];
    if (strcmp(CurrentDiskPath, "/") == 0)
        snprintf(tmpBuf, sizeof(tmpBuf), "/%s", FileToLoad);
    else
        snprintf(tmpBuf, sizeof(tmpBuf), "%s/%s", CurrentDiskPath, FileToLoad);
    if (strcmp(tmpBuf, s_preview_path) == 0) return;   // déjà affiché
    ClearFileList();
    snprintf(s_preview_path, sizeof(s_preview_path), "%s", tmpBuf);

    static uint8_t *dir = nullptr;               // 2,3 Ko en PSRAM
    if (!dir) dir = (uint8_t *)heap_caps_malloc(9 * 256, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!dir) dir = (uint8_t *)malloc(9 * 256);
    if (!dir) return;
    if (s_browse_pc)                             // répertoire lu sur le PC (2 304 octets)
    {
        uint32_t size = 0;
        for (int i = 0; i < NetShare_Count(); i++)
        {
            bool isdir; uint32_t sz; const char *nm;
            if (NetShare_Entry(i, &isdir, &sz, &nm) && !isdir && strcmp(nm, FileToLoad) == 0) { size = sz; break; }
        }
        const uint32_t hdr = size % 256u;
        const uint32_t dirPos = hdr + 256u * 18u * 17u + 512u;
        uint32_t got = 0;
        if (size < dirPos + 9 * 256 || NetShare_Read(tmpBuf, dirPos, 9 * 256, dir, &got) != NS_OK || got != 9 * 256) return;
    }
    else
    {
    File f = SD_MMC.open(tmpBuf, FILE_READ);
    if (!f) return;
    const uint32_t size = f.size();
    const uint32_t hdr = size % 256u;                         // en-tête JVC éventuel
    const uint32_t dirPos = hdr + 256u * 18u * 17u + 512u;    // piste 17, secteur 3
    if (size < dirPos + 9 * 256 || !f.seek(dirPos) || f.read(dir, 9 * 256) != 9 * 256)
    {
        f.close();
        return;
    }
    f.close();
    }

    uint8_t n = 0;
    for (uint8_t e = 0; e < 72 && n < 68; e++)
    {
        const uint8_t *ent = &dir[e * 32];
        if (ent[0] == 0xFF) break;               // fin du répertoire
        if (ent[0] == 0x00) continue;            // entrée effacée
        bool ok = ent[11] <= 3;                  // type : BASIC, données, ML, texte
        for (uint8_t i = 0; i < 11 && ok; i++) ok = ent[i] >= 0x20 && ent[i] < 0x7F;
        if (!ok)                                 // pas un répertoire RS-DOS
        {
            memset(FileArray.FileList, 255, sizeof(FileArray.FileList));
            return;
        }
        memcpy(FileArray.FileList[n++], ent, 11);
    }
}

// aperçu d'une bande (.CAS / .WAV) : un fichier par bloc d'en-tête (nom, BAS / DAT / BIN)
static void Tape_Entry(const char *name8, uint8_t type, uint8_t ascii, void *u)
{
    (void)ascii;
    uint8_t &n = *(uint8_t *)u;
    if (n >= 68) return;
    for (int i = 0; i < 8; i++) { const char c = name8[i]; FileArray.FileList[n][i] = (c >= 0x20 && c < 0x7F) ? c : '?'; }
    const char *ext = type == 0 ? "BAS" : type == 1 ? "DAT" : type == 2 ? "BIN" : "???";
    memcpy(&FileArray.FileList[n][8], ext, 3);
    n++;
}
static void PopulateTapeContent(const char *FileToLoad)
{
    char tmpBuf[192];
    if (strcmp(CurrentDiskPath, "/") == 0) snprintf(tmpBuf, sizeof(tmpBuf), "/%s", FileToLoad);
    else snprintf(tmpBuf, sizeof(tmpBuf), "%s/%s", CurrentDiskPath, FileToLoad);
    if (strcmp(tmpBuf, s_preview_path) == 0) return;   // déjà affiché
    ClearFileList();
    snprintf(s_preview_path, sizeof(s_preview_path), "%s", tmpBuf);
    uint8_t n = 0;
    Cas_ListTape(tmpBuf, Tape_Entry, &n);
}

void ClearFileList(void)
{
    // effacées (noms d'un ancien disque au bas de l'écran). L'entrée 68 reste toujours à 255 : fin de
    // liste garantie pour DisplayDiskContent.
    memset(FileArray.FileList, 255, sizeof(FileArray.FileList));
    s_preview_path[0] = 0;
}
void ClearFileBuffer(void)
{
    for (uint8_t loop1 = 0; loop1 != 60; loop1++)
    {
        for (uint8_t loop2 = 0; loop2 != 30; loop2++)
        {
            FileArray.FileBuffer[loop2][loop1] = 0;
        }
    }
    return;
}

void PrintFileName(uint8_t LoopFile)
{

    for (int i = 0; i < 11; i++)
    {
       debug((char)FileArray.FileList[LoopFile][i]);
    }
    debugln("");
}

void MENU_DisplayGIMEchar(uint8_t charNum, uint8_t ODDbyte, uint16_t Xpos, uint16_t Ypos, uint8_t ForeColor, uint8_t BackColor)
{
    // uint32_t BlinkPhase = (sf.GIME_BLINK_COUTER_COPY & 0b00010000)
    uint32_t BlinkPhase = 0;

    uint32_t loop1, loop2;
    uint16_t XposLoop, YposLoop;
    uint8_t tmpChar, loopArray;
    uint8_t Underline, Blink;

    // caractère hors de l'écran = ignoré. VGA::drawLineFromMemory8 n'a pas de contrôle de bornes
    if (!vga || Xpos + 8 > (uint16_t)vga->mode.hRes || Ypos + 8 > (uint16_t)vga->mode.vRes) return;

    Blink     = (ODDbyte & 0b10000000);
    Underline = (ODDbyte & 0b01000000);

    // Precalculate an array of 8 pixels, it's faster in PSRAM
    uint8_t tmpArray[8];

    loop1 = charNum * 12 + 1;

    for (YposLoop = Ypos; YposLoop != Ypos + 8; YposLoop++)
    {
        loopArray = 0;
        tmpChar   = font_gimeChip[loop1++];

        for (uint8_t bit = 0; bit != 8; bit++)
        {
            if ((tmpChar & 0b10000000) != 0)
            {
                if (Blink == 0b10000000)
                {
                    if (BlinkPhase != 0)
                    {
                        tmpArray[loopArray++] = BackColor;
                    }
                    else
                    {
                        tmpArray[loopArray++] = ForeColor;
                    }
                }
                else
                {
                    tmpArray[loopArray++] = ForeColor;
                }
            }
            else
            {
                tmpArray[loopArray++] = BackColor;
            }

            tmpChar <<= 1;
        }

        if ((YposLoop == Ypos + 7) && (Underline != 0))
        {

            for (uint8_t loop3 = 0; loop3 !=8; loop3++)
            {
                tmpArray[loop3] = ForeColor;
            }

            vga->drawLineFromMemory8(Xpos, YposLoop, &tmpArray[0]);
        }
        else
        {
            vga->drawLineFromMemory8(Xpos, YposLoop, &tmpArray[0]);
        }
    }
}

// message (capture, journal F9, etc.) pendant le menu F12 : écrit au bas de l'écran dans tous les
// tampons vidéo (visible tout de suite, effacé au prochain redessin du menu)
static bool s_toast_shown = false;   // un message du bas a été dessiné
void Menu_Toast(const char *l1, const char *l2)
{
    if (!vga || !vga->dmaBuffer) return;
    char a[54], b[54];
    snprintf(a, sizeof a, "%-53.53s", l1 ? l1 : "");
    snprintf(b, sizeof b, "%-53.53s", l2 ? l2 : "");
    const int keep = vga->backBuffer;
    s_toast_shown = true;
    for (int i = 0; i < (vga->bufferCount > 0 ? vga->bufferCount : 1); i++)
    {
        vga->backBuffer = i;
        DrawText(a, 0, 28, 0, 0, 255, 3);
        DrawText(b, 0, 29, 0, 0, 255, 3);
        vga->dmaBuffer->flush(i);
    }
    vga->backBuffer = keep;
}

// Sortie du menu : efface le message du bas (lignes 224-239), que l'écran du CoCo ne redessine pas
// (bordure des modes texte / VDG). Couleur = celle de la bordure juste au-dessus.
void Menu_ToastClear(void)
{
    if (!s_toast_shown || !vga || !vga->dmaBuffer) return;
    s_toast_shown = false;
    const int w = vga->mode.hRes, h = vga->mode.vRes;
    if (h < 240) return;
    for (int i = 0; i < (vga->bufferCount > 0 ? vga->bufferCount : 1); i++)
    {
        const uint8_t c = vga->dmaBuffer->getLineAddr8(222, i)[0];
        for (int y = 224; y < 240; y++) memset(vga->dmaBuffer->getLineAddr8(y, i), c, w);
        vga->dmaBuffer->flush(i);
    }
}

uint8_t Font_Row(uint8_t ch, uint8_t row) { return (ch < 128 && row < 8) ? font_gimeChip[ch * 12 + 1 + row] : 0; }   // écran d'accueil

void DrawText(const char *text, uint8_t Xpos, uint8_t Ypos, uint8_t XpixOffset, uint8_t YpixOffset, uint8_t ForeColor, uint8_t BackColor)
{
    uint16_t X, Y;
    X = (uint16_t)Xpos * 6 + XpixOffset;
    Y = (uint16_t)Ypos * 8 + YpixOffset;

    while(*text != '\0')
    {
        MENU_DisplayGIMEchar(*text, 0, X, Y, ForeColor, BackColor);
        text++;
        X +=6;
    }
}

void EMULATOR_Menu(void)
{
    BackDisplay();
    EMU_Draw_Menu();
    // réglages du menu F12 sauvegardés une seule fois, à la sortie
    SaveConfigToSD();
    UsbDev_SaveIfDirty();    // appareils USB, manettes, dernières cartouches
    RestoreDisplay();
}

void BackDisplay(void)
{
    uint32_t loopBack = 0;
    uint16_t X, Y;
    vga->show();    // Swap Buffer
    for (Y=0; Y!=240;Y++)
    {
        for (X=0; X!=320;X++)
        {
            MENU_Backup[loopBack++] = vga->getPixel(X, Y);
        }

    }
    loopBack = 0;
    vga->show();    // Swap Buffer
    for (Y=0; Y!=240;Y++)
    {
        for (X=0; X!=320;X++)
        {
            MENU_BackupPage2[loopBack++] = vga->getPixel(X, Y);
        }

    }

    vga->clear(0);
    vga->show();
    vga->clear(0);
    vga->show();

}

void RestoreDisplay(void)
{
    uint32_t loopBack = 0;
    uint16_t X, Y;

    for (Y = 0; Y < 240; Y++)
    {
        for (X = 0; X < 320; X++)
        {
            vga->dot(X, Y, MENU_Backup[loopBack++]);
        }
    }
    vga->show();
    loopBack = 0;
    for (Y = 0; Y < 240; Y++)
    {
        for (X = 0; X < 320; X++)
        {
            vga->dot(X, Y, MENU_BackupPage2[loopBack++]);
        }
    }

}

extern void Fw_ValidateIfDue(bool now);
extern bool Fw_OtherVersion(char *out, size_t n);
extern bool Fw_BootOther(void);
// =====================================================================
// MAIN MENU : (H)ARDWARE, (M)ONITOR, (D)ATA STORAGE, (C)ART SLOT & COM (U)SB DEVICES, (N)ETWORK,
// (S)YSTEM ; résumé vert à droite
// =====================================================================
#define MN_TOP    16
#define MN_PITCH  12
#define MN_Y_RB   187           // ligne de la question / du rappel de redémarrage
#define MN_Y_KEYS 198           // ligne des touches
#define MN_WHITE  255
#define MN_YELLOW 252
#define MN_GREEN  28
#define MN_RED    224
#define MN_BLUE   3

static inline uint8_t Mn_Y(int i) { return (uint8_t)(MN_TOP + i * MN_PITCH); }

// Texte à la position (colonne, pixel y), tronqué à « room » caractères (~ au bout).
static void Mn_Text(const char *t, int col, int y, uint8_t fg, uint8_t bg, int room = 53)
{
    char b[60];
    const int n = (int)strlen(t);
    if (room > 57) room = 57;
    if (room < 1) return;
    if (n > room) { memcpy(b, t, room - 1); b[room - 1] = '~'; b[room] = 0; }
    else          { memcpy(b, t, n + 1); }
    DrawText(b, (uint8_t)col, 0, 0, (uint8_t)y, fg, bg);
}

static void Mn_Frame(const char *title, const char *keys)
{
    DrawText(title, 0, 0, 0, 0, MN_WHITE, 0);
    if (!strcmp(title, "MAIN MENU"))
        DrawText("Firmware v" FW_VERSION_SHORT " (" FW_VERSION_DATE ")", 24, 0, 0, 0, MN_GREEN, 0);
    else
    {
        const char *v = "v" FW_VERSION_SHORT;
        DrawText(v, (uint8_t)(53 - strlen(v)), 0, 0, 0, MN_GREEN, 0);
    }
    line(0, 10, 319, 10, MN_BLUE);
    DrawText(keys, 0, 0, 0, MN_Y_KEYS, MN_GREEN, 0);
    line(0, 211, 319, 211, MN_BLUE);                       // sous la ligne des touches
    DrawText("-- ESP32-COCO3-PRO ----- COLOR COMPUTER 3 EMULATOR --", 0, 27, 0, 0, MN_RED, 0);
    DrawText("  (C) 2026 DESIGNED AND DEVELOPED BY LUC DION", 0, 28, 0, 0, MN_BLUE, 0);
    DrawText("      In collaboration with Cedric Beaudoin", 0, 29, 0, 0, MN_BLUE, 0);
}

// Une ligne de menu : « > » + libellé + valeur + (R).
static void Mn_Row(int i, const char *label, const char *value, uint8_t vcolor, int vcol,
                   bool sel, bool arrows, bool rflag, bool hot, int yadd = 0, bool compact = false)
{
    const int y = Mn_Y(i) + yadd;
    const uint8_t bg = sel ? MN_BLUE : 0;
    if (sel) for (int yy = y - (compact ? 1 : 2); yy < y + (compact ? 9 : 10); yy++) line(0, yy, 319, yy, MN_BLUE);
    // « valeur\tXétiquette » : étiquette en colonne 47 (X = couleur : W blanc, Y jaune, G vert, R
    // rouge)
    char vbuf[64];
    const char *tag = nullptr;
    if (value && strchr(value, '\t'))
    {
        snprintf(vbuf, sizeof vbuf, "%s", value);
        char *t = strchr(vbuf, '\t');
        *t = 0;
        tag = t + 1;
        value = vbuf;
    }
    Mn_Text(sel ? ">" : " ", 0, y, MN_WHITE, bg);
    for (int k = 0; label[k] && k < 40; k++)
    {
        const char c[2] = { label[k], 0 };
        DrawText(c, (uint8_t)(1 + k), 0, 0, (uint8_t)y, (hot && k == 0) ? MN_WHITE : MN_YELLOW, bg);
    }
    if (value && value[0])
    {
        const int room = (tag ? 46 : 53) - vcol - (rflag ? 4 : 0);
        if (sel && arrows)
        {
            char b[64];
            snprintf(b, sizeof b, "< %.*s >", room > 4 ? room - 4 : 1, value);
            Mn_Text(b, vcol, y, MN_WHITE, bg, room);
        }
        else Mn_Text(value, vcol, y, sel ? MN_WHITE : vcolor, bg, room);
    }
    if (rflag) Mn_Text("(R)", 50, y, sel ? MN_WHITE : MN_RED, bg);
    if (tag && tag[0] && tag[1])
    {
        const uint8_t tc = tag[0] == 'Y' ? MN_YELLOW : tag[0] == 'G' ? MN_GREEN : tag[0] == 'R' ? MN_RED : MN_WHITE;
        Mn_Text(tag + 1, 47, y, sel ? MN_WHITE : tc, bg, 6);
    }
}

static void Mn_Base(const char *path, char *b, size_t n)   // nom de fichier sans le dossier
{
    if (!path || !path[0]) { snprintf(b, n, "(empty)"); return; }
    const char *p = strrchr(path, '/');
    snprintf(b, n, "%s", p ? p + 1 : path);
}

// ---------------------------------------------------------------- éléments
struct MnItem
{
    const char *label;
    int vcol;
    void (*value)(char *b, size_t n, uint8_t &color);   // nullptr : pas de valeur
    void (*change)(int dir);                            // GAUCHE / DROITE (nullptr : aucun)
    bool (*enter)(void);                                // ENTER ; rend true = retour à l'émulation
    bool (*reboot)(void);                               // (R)
    bool off;                                           // ligne inactive (à venir, information)
    void (*clear)(void);                                // touche Suppr (vider un lecteur)
};

static void Mn_Redraw2(void (*draw)(void))   // les deux tampons de l'écran
{
    for (int pass = 0; pass < 2; pass++) { vga->clear(0); draw(); vga->show(); }
}

// ---------------------------------------------------------------- moteur des pages (niveaux 2 et 3)
// une page = titre + liste (fixe, ou refaite à chaque dessin par build) + ligne d'aide verte + lignes
// d'information (extra) + rafraîchissement (poll, ~1 s).
struct MnPage
{
    const char *title;
    const MnItem *items; int count;          // liste fixe...
    int (*build)(MnItem *out, int max);      // ou liste refaite à chaque dessin (Multi-Pak, etc.)
    const char *hint;                        // ligne d'aide verte sous la liste
    int (*extra)(int y);                     // lignes d'information à partir du pixel y ; rend le y suivant
    bool (*poll)(void);                      // ~1 s : true = redessiner (état du Wi-Fi, heure...)
    uint8_t compactFrom;                     // rangées serrées (9 px) à partir de celle-ci (0 = aucune)
};
static const char k_mn_sep[] = "-";          // ligne de séparation (mince ligne bleue)
static uint8_t s_mn_dyn_compact = 0;         // compactFrom = 0xFF : rangée fixée par build
#define MN_DYN_MAX 18
#define MENU_DEL   0x4C                      // touche Suppr (HID) : vider le lecteur choisi
static const MnPage *s_mn_page = nullptr;
static MnItem s_mn_dyn[MN_DYN_MAX];
static const MnItem *s_mn_items = nullptr;
static int s_mn_count = 0, s_mn_sel = 0;
static const char *s_mn_msg = nullptr;       // message d'une action (State saved., etc.)
static uint8_t s_mn_msg_color = MN_GREEN;
static const char *const k_mn_sub_keys = "UP/DN select  LEFT/RIGHT change  ENTER  ESC back";

static void Mn_Load(const MnPage *p)
{
    s_mn_page = p;
    if (p->build) { s_mn_count = p->build(s_mn_dyn, MN_DYN_MAX); s_mn_items = s_mn_dyn; }
    else          { s_mn_items = p->items; s_mn_count = p->count; }
    if (s_mn_sel >= s_mn_count) s_mn_sel = s_mn_count - 1;
    if (s_mn_sel < 0) s_mn_sel = 0;
    int s = s_mn_sel;
    while (s < s_mn_count && s_mn_items[s].off) s++;               // ligne inactive : la suivante...
    if (s >= s_mn_count) { s = s_mn_sel; while (s > 0 && s_mn_items[s].off) s--; }   // ou la précédente
    s_mn_sel = s;
}

static void Mn_DrawSub(void)
{
    const MnPage *p = s_mn_page;
    Mn_Frame(p->title, k_mn_sub_keys);
    bool any_r = false;
    int y = MN_TOP;
    for (int i = 0; i < s_mn_count; i++)
    {
        const MnItem &it = s_mn_items[i];
        if (it.label == k_mn_sep) { line(0, y + 1, 319, y + 1, MN_BLUE); y += 5; continue; }
        const int cf = p->compactFrom == 0xFF ? s_mn_dyn_compact : p->compactFrom;
        const bool cmp = cf && i >= cf;
        char v[64] = "";
        uint8_t c = MN_GREEN;
        if (it.value) it.value(v, sizeof v, c);
        const bool r = it.reboot && it.reboot();
        any_r |= r;
        Mn_Row(i, it.label, v, c, it.vcol, i == s_mn_sel && !it.off, it.change != nullptr && !it.off, r, false, y - Mn_Y(i), cmp);
        y += cmp ? 9 : MN_PITCH;
    }
    y += 4;
    if (p->hint) { y += 4; Mn_Text(p->hint, 1, y, MN_GREEN, 0); y += 12; }
    if (p->extra) p->extra(y);
    if (s_mn_msg) Mn_Text(s_mn_msg, 1, any_r ? MN_Y_RB - 11 : MN_Y_RB, s_mn_msg_color, 0);
    if (any_r) DrawText("Reboot to apply changes", 1, 0, 0, MN_Y_RB, MN_RED, 0);
}

// Ligne des touches redessinée seule (les deux tampons) ; center = texte centré.
static void Mn_KeysLine(const char *t, uint8_t color, uint8_t bg = 0, bool center = false)
{
    const int x = center ? (320 - 6 * (int)strlen(t)) / 2 : 0;
    for (int pass = 0; pass < 2; pass++)
    {
        for (int yy = MN_Y_KEYS - 2; yy < MN_Y_KEYS + 10; yy++) line(0, yy, 319, yy, bg);
        DrawText(t, 0, 0, (uint8_t)x, MN_Y_KEYS, color, bg);
        vga->show();
    }
}

// question en rouge sur barre bleue, centrée, qui alterne (700 ms) avec la ligne des touches « keys ».
// Rend 1 = Y, 2 = N, 3 = ESC.
static void Menu_Beep(void)
{
    const uint8_t old = Audio_GetTestTone();
    Audio_SetTestTone(1);
    vTaskDelay(pdMS_TO_TICKS(120));
    Audio_SetTestTone(old);
}
static int Mn_Ask(const char *q, const char *keys)
{
    Menu_KeySync();
    if (strncmp(q, "REBOOT", 6) == 0) Menu_Beep();
    int answer = 0;
    bool phase = true;
    uint32_t t0 = millis() - 1000;
    while (!answer)
    {
        if (millis() - t0 >= 700)
        {
            t0 = millis();
            if (phase) Mn_KeysLine(q, MN_RED, MN_BLUE, true);
            else       Mn_KeysLine(keys, MN_GREEN);
            phase = !phase;
        }
        const uint8_t k = Menu_GetKey();
        if (k == MENU_Y) answer = 1;
        else if (k == MENU_N) answer = 2;
        else if (k == MENU_ESC) answer = 3;
        vTaskDelay(20);
    }
    Menu_KeySync();
    Mn_KeysLine(keys, MN_GREEN);
    return answer;
}
static bool Mn_AskYN(const char *q)          // pages de niveau 2 / 3 : Y = oui ; N, ESC = non
{
    char b[64];
    snprintf(b, sizeof b, "%s  Y / N", q);
    return Mn_Ask(b, k_mn_sub_keys) == 1;
}

// Page de menu : rend true s'il faut retourner directement à l'émulation.
static bool Mn_Page(const MnPage *p)
{
    s_mn_sel = 0;
    s_mn_msg = nullptr;
    Mn_Load(p);
    Mn_Redraw2(Mn_DrawSub);
    Menu_KeySync();
    uint32_t tpoll = millis();
    while (1)
    {
        const uint8_t k = Menu_GetKey();
        bool redraw = false;
        const MnItem it = s_mn_items[s_mn_sel];       // copie : la liste peut être refaite
        if (k == MENU_UP || k == MENU_DOWN)
        {
            int s = s_mn_sel;
            do { s += (k == MENU_UP) ? -1 : 1; } while (s >= 0 && s < s_mn_count && s_mn_items[s].off);
            if (s >= 0 && s < s_mn_count) { s_mn_sel = s; s_mn_msg = nullptr; redraw = true; }
        }
        else if ((k == MENU_LEFT || k == MENU_RIGHT) && it.change && !it.off)
        {
            it.change(k == MENU_LEFT ? -1 : 1);
            redraw = true;
        }
        else if (k == MENU_ENTER && !it.off)
        {
            if (it.enter)
            {
                const int sel = s_mn_sel;
                s_mn_msg = nullptr;                   // l'ancien message (ex. « press ENTER ») ne reste pas
                if (it.enter()) return true;
                s_mn_sel = sel;                       // une page appelée a pu changer la sélection
                Mn_Load(p);
                Menu_KeySync();
            }
            else if (it.change) it.change(1);
            redraw = true;
        }
        else if (k == MENU_DEL && it.clear && !it.off)
        {
            it.clear();
            redraw = true;
        }
        else if (k == MENU_ESC)
        {
            Menu_KeySync();
            s_mn_msg = nullptr;
            return false;
        }
        if (p->poll && millis() - tpoll >= 1000)
        {
            tpoll = millis();
            if (p->poll()) redraw = true;
        }
        if (redraw) { Mn_Load(p); Mn_Redraw2(Mn_DrawSub); }
        vTaskDelay(2);
    }
}

// HARDWARE
static void V_Cpu(char *b, size_t n, uint8_t &c)  { snprintf(b, n, "%s", g_cpu6309_sel ? "HD6309" : "MC6809"); c = MN_GREEN; }
static void C_Cpu(int)                            { g_cpu6309_sel = !g_cpu6309_sel; }
static bool R_Cpu(void)                           { return g_cpu6309_sel != g_cpu6309_run; }
static void V_Ram(char *b, size_t n, uint8_t &c)
{
    const unsigned kb = (unsigned)(g_ram_selected / 1024);
    if (kb >= 1024) snprintf(b, n, "%u MB", kb / 1024); else snprintf(b, n, "%u KB", kb);
    c = MN_GREEN;
}
static void C_Ram(int dir)
{
    static const uint32_t k[3] = { 512u * 1024u, 2u * 1024u * 1024u, 4u * 1024u * 1024u };
    int i = 0;
    for (int j = 0; j < 3; j++) if (g_ram_selected == k[j]) i = j;
    for (int t = 0; t < 3; t++)
    {
        i = (i + (dir < 0 ? 2 : 1)) % 3;
        if (k[i] <= g_ram_capacity) break;
    }
    g_ram_selected = k[i];
}
static bool R_Ram(void)                           { return g_ram_selected != gime_ram_size(); }
static void V_Gime(char *b, size_t n, uint8_t &c) { snprintf(b, n, "%s", gime_is_1987() ? "1987 (timer +1)" : "1986 (timer +2)"); c = MN_GREEN; }
static void C_Gime(int)                           { gime_set_1987(!gime_is_1987()); }
static void V_Oc(char *b, size_t n, uint8_t &c)
{
    snprintf(b, n, "%s", g_oc_mode == OC_OFF ? "OFF (cycle accurate)" : (g_oc_mode == OC_CPU ? "CPU" : "CPU + GIME (machine)"));
    c = g_oc_mode == OC_OFF ? MN_GREEN : MN_RED;
}
static void C_Oc(int dir)
{
    if (dir >= 0) SetOverclockMode(g_oc_mode == OC_OFF ? OC_CPU : (g_oc_mode == OC_CPU ? OC_MACHINE : OC_OFF));
    else          SetOverclockMode(g_oc_mode == OC_OFF ? OC_MACHINE : (g_oc_mode == OC_MACHINE ? OC_CPU : OC_OFF));
}
static const MnItem k_mn_hw[] = {
    { "CPU",               22, V_Cpu,  C_Cpu,  nullptr, R_Cpu,  false },
    { "EXPANSION RAM",     22, V_Ram,  C_Ram,  nullptr, R_Ram,  false },
    { "GIME CHIP VERSION", 22, V_Gime, C_Gime, nullptr, nullptr, false },
    { "OVERCLOCK",         22, V_Oc,   C_Oc,   nullptr, nullptr, false },
};

// MONITOR
static void V_Mon(char *b, size_t n, uint8_t &c)  { snprintf(b, n, "%s", gime_video_monitor_rgb() ? "RGB" : "COMPOSITE"); c = MN_GREEN; }
static void C_Mon(int)
{
    // Comme sur un vrai CoCo 3 : un moniteur RGB n'affiche pas les couleurs d'artefact, un composite
    // oui (on peut ensuite changer l'artefact).
    gime_video_set_monitor_rgb(!gime_video_monitor_rgb());
    sf.Artefact = !gime_video_monitor_rgb();
}
static void V_Art(char *b, size_t n, uint8_t &c)
{
    snprintf(b, n, "%s", !sf.Artefact ? "DISABLED" : (g_artifact_swap ? "BLUE-RED" : "RED-BLUE"));
    c = sf.Artefact ? MN_GREEN : MN_RED;
}
static void C_Art(int dir)                        // DISABLED -> RED-BLUE -> BLUE-RED
{
    int st = !sf.Artefact ? 0 : (g_artifact_swap ? 2 : 1);
    st = (st + (dir < 0 ? 2 : 1)) % 3;
    sf.Artefact = st != 0;
    g_artifact_swap = st == 2;
}
static void V_Aud(char *b, size_t n, uint8_t &c)  { snprintf(b, n, "%s", Audio_GetStereo() ? "STEREO" : "MONO"); c = MN_GREEN; }
static void C_Aud(int)                            { Audio_SetStereo(!Audio_GetStereo()); }
static const MnItem k_mn_mon[] = {
    { "COCO 3 MONITOR",           27, V_Mon, C_Mon, nullptr, nullptr, false },
    { "ARTIFACT COLORS (COCO 2)", 27, V_Art, C_Art, nullptr, nullptr, false },
    { "AUDIO OUTPUT",             27, V_Aud, C_Aud, nullptr, nullptr, false },
};

// DATA STORAGE
static void V_DrvN(int d, char *b, size_t n, uint8_t &c)
{
    Mn_Base((const char *)Disk_Drive.Name_Disk[d], b, n);
    c = Disk_Drive.Name_Disk[d][0] ? MN_GREEN : MN_RED;
}
static void V_Drv0(char *b, size_t n, uint8_t &c) { V_DrvN(0, b, n, c); }
static void V_Drv1(char *b, size_t n, uint8_t &c) { V_DrvN(1, b, n, c); }
static void V_Drv2(char *b, size_t n, uint8_t &c) { V_DrvN(2, b, n, c); }
static void V_Drv3(char *b, size_t n, uint8_t &c) { V_DrvN(3, b, n, c); }
static bool E_Drv0(void) { DiskMenuChoose_1(0); return false; }
static bool E_Drv1(void) { DiskMenuChoose_1(1); return false; }
static bool E_Drv2(void) { DiskMenuChoose_1(2); return false; }
static bool E_Drv3(void) { DiskMenuChoose_1(3); return false; }
static __attribute__((unused)) void V_Vd(char *b, size_t n, uint8_t &c)
{
    int k = 0;
    for (int i = 0; i < 6; i++) if (g_dw_name[i][0]) k++;
    if (!k) { snprintf(b, n, "none"); c = MN_RED; return; }
    char nm[40];
    const int first = g_dw_name[4][0] ? 4 : 0;
    int i0 = first;
    if (!g_dw_name[i0][0]) for (int i = 0; i < 6; i++) if (g_dw_name[i][0]) { i0 = i; break; }
    Mn_Base(g_dw_name[i0], nm, sizeof nm);
    if (k > 1) snprintf(b, n, "%s +%d", nm, k - 1); else snprintf(b, n, "%s", nm);
    c = MN_GREEN;
}
// /X0, /X1, VHD 0, VHD 1 ouvrent le navigateur (niveau 3) comme D0-D3 ; Suppr = vider
static bool Mn_Browse(uint8_t unit)
{
    DiskMenuChoose_1(unit);
    if (s_menu_exit_all) { s_menu_exit_all = false; return true; }
    return false;
}
static bool E_X0(void) { return Mn_Browse(4); }
static bool E_X1(void) { return Mn_Browse(5); }
static bool E_H0(void) { return Mn_Browse(6); }
static bool E_H1(void) { return Mn_Browse(7); }
static void Drv_Eject(uint8_t d) { Disk_Drive.Name_Disk[d][0] = 0; ReadCoCoFile("", d); }
static void K_Drv0(void) { Drv_Eject(0); }
static void K_Drv1(void) { Drv_Eject(1); }
static void K_Drv2(void) { Drv_Eject(2); }
static void K_Drv3(void) { Drv_Eject(3); }
static void K_X0(void) { DW_Unmount(0); }
static void K_X1(void) { DW_Unmount(1); }
static void K_H0(void) { DW_Unmount(4); }
static void K_H1(void) { DW_Unmount(5); }
static void V_Boot(char *b, size_t n, uint8_t &c)
{
    if (g_dw_name[0][0]) { snprintf(b, n, "ENTER: boot from /X0"); c = MN_GREEN; }
    else { snprintf(b, n, "no disk in /X0"); c = MN_RED; }
}
static bool E_Boot(void)
{
    if (!g_dw_name[0][0]) { s_mn_msg = "No disk in /X0."; s_mn_msg_color = MN_RED; return false; }
    g_dw_boot_request = true;            // amorçage au retour dans l'émulation
    vTaskDelay(200);
    return true;
}
static bool Pg_State(void);
// CASSETTE (page de niveau 3, plus bas)
static bool Pg_Cas(void);
static void V_Cas(char *b, size_t n, uint8_t &c)
{
    const uint8_t m = Cas_Mode();
    if (m == CAS_EMPTY) { snprintf(b, n, "no tape"); c = MN_RED; return; }
    char nm[40]; Mn_Base(Cas_Name(), nm, sizeof nm);
    snprintf(b, n, "%s%s", m == CAS_RECORD ? "REC " : "", nm); c = m == CAS_RECORD ? MN_YELLOW : MN_GREEN;
}
static bool E_Cas(void) { return Pg_Cas(); }
static void K_Cas(void) { Cas_Eject(); }
static bool Pg_Cart(void);
static bool Pg_Net(void);
static bool Pg_Mouse(void);
static void V_State(char *b, size_t n, uint8_t &c) { snprintf(b, n, "4 slots (F5 / F8)"); c = MN_GREEN; }
static bool E_State(void) { return Pg_State(); }
static void V_Soon(char *b, size_t n, uint8_t &c) { snprintf(b, n, "coming soon"); c = MN_BLUE; }
static void V_DwN(int u, char *b, size_t n, uint8_t &c) { Mn_Base(g_dw_name[u], b, n); c = g_dw_name[u][0] ? MN_GREEN : MN_RED; }
static void V_X0(char *b, size_t n, uint8_t &c) { V_DwN(0, b, n, c); }
static void V_X1(char *b, size_t n, uint8_t &c) { V_DwN(1, b, n, c); }
static void V_H0(char *b, size_t n, uint8_t &c) { V_DwN(4, b, n, c); }
static void V_H1(char *b, size_t n, uint8_t &c) { V_DwN(5, b, n, c); }
static const MnItem k_mn_sto[] = {
    { "DRIVE 0",            22, V_Drv0,  nullptr, E_Drv0,  nullptr, false, K_Drv0 },
    { "DRIVE 1",            22, V_Drv1,  nullptr, E_Drv1,  nullptr, false, K_Drv1 },
    { "DRIVE 2",            22, V_Drv2,  nullptr, E_Drv2,  nullptr, false, K_Drv2 },
    { "DRIVE 3",            22, V_Drv3,  nullptr, E_Drv3,  nullptr, false, K_Drv3 },
    { k_mn_sep,             22, nullptr, nullptr, nullptr, nullptr, true,  nullptr },
    { "DRIVEWIRE /X0",      22, V_X0,    nullptr, E_X0,    nullptr, false, K_X0 },
    { "DRIVEWIRE /X1",      22, V_X1,    nullptr, E_X1,    nullptr, false, K_X1 },
    { "HARD DISK VHD 0",    22, V_H0,    nullptr, E_H0,    nullptr, false, K_H0 },
    { "HARD DISK VHD 1",    22, V_H1,    nullptr, E_H1,    nullptr, false, K_H1 },
    { "BOOT NITROS-9",      22, V_Boot,  nullptr, E_Boot,  nullptr, false, nullptr },
    { k_mn_sep,             22, nullptr, nullptr, nullptr, nullptr, true,  nullptr },
    { "SAVE / LOAD STATE",  22, V_State, nullptr, E_State, nullptr, false, nullptr },
    { "CASSETTE",           22, V_Cas,   nullptr, E_Cas,   nullptr, false, K_Cas },
};

// CART SLOT & COM
static void V_Slot(char *b, size_t n, uint8_t &c)
{
    if (g_slotcfg.mpi)                         snprintf(b, n, "MULTI-PAK (4 slots)");
    else if (g_slotcfg.mode[0] == SLOT_FDC)    snprintf(b, n, "DISK BASIC (FD-502)");
    else if (g_slotcfg.mode[0] == SLOT_EMPTY)  snprintf(b, n, "EMPTY");
    else if (g_slotcfg.mode[0] == SLOT_RS232)  snprintf(b, n, "RS-232 PAK");
    else { char nm[40]; Mn_Base(g_slotcfg.name[0], nm, sizeof nm); snprintf(b, n, "%s", nm); }
    c = (!g_slotcfg.mpi && g_slotcfg.mode[0] == SLOT_EMPTY) ? MN_RED : MN_GREEN;
}
static void V_Bb(char *b, size_t n, uint8_t &c)
{
    static const char *const k_bb[BB_MODE_COUNT] = { "OFF", "PRINTER -> SD", "DRIVEWIRE (/x0-/x3)", "UART 41/42", "DRIVEWIRE Wi-Fi", "MODEM Wi-Fi" };
    snprintf(b, n, "%s", k_bb[g_bb_mode < BB_MODE_COUNT ? g_bb_mode : 0]);
    c = g_bb_mode ? MN_GREEN : MN_RED;
}
static void V_Rs(char *b, size_t n, uint8_t &c)
{
    int slot = -1;
    for (int i = 0; i < (g_slotcfg.mpi ? MPI_SLOTS : 1); i++) if (g_slotcfg.mode[i] == SLOT_RS232) slot = i;
    if (slot < 0) { snprintf(b, n, "OFF"); c = MN_RED; return; }
    if (g_slotcfg.mpi) snprintf(b, n, "MULTI-PAK SLOT %d", slot + 1); else snprintf(b, n, "IN CARTRIDGE SLOT");
    c = MN_GREEN;
}
static __attribute__((unused)) void V_Mpi(char *b, size_t n, uint8_t &c)
{
    if (!g_slotcfg.mpi) { snprintf(b, n, "OFF"); c = MN_RED; return; }
    snprintf(b, n, "ON, SWITCH ON SLOT %d", g_slotcfg.sw + 1);
    c = MN_GREEN;
}

// USB DEVICES (écrans existants ; affectation des ports : livraison suivante)
static void V_Lc(char *b, size_t n, uint8_t &c)   { snprintf(b, n, "%s", g_lowercase_t1 ? "ENABLED" : "DISABLED"); c = g_lowercase_t1 ? MN_GREEN : MN_RED; }
static void C_Lc(int)                             { g_lowercase_t1 = !g_lowercase_t1; }
static void V_UsbSt(char *b, size_t n, uint8_t &c) { extern bool Usb_Summary(char *, size_t); c = Usb_Summary(b, n) ? MN_YELLOW : MN_GREEN; }   // jaune = appareil sans canal
static bool Pg_Usb(void);                     // page USB DEVICES (fin du fichier)

// SYSTEM
static void Menu_FirmwareFlow(void);
static void V_Fw(char *b, size_t n, uint8_t &c)   { snprintf(b, n, ".FLH / .BIN on SD card"); c = MN_GREEN; }
static bool E_Fw(void)                            { Menu_FirmwareFlow(); return false; }
static __attribute__((unused)) void V_Ver(char *b, size_t n, uint8_t &c)  { snprintf(b, n, "v" FW_VERSION_SHORT " (" FW_VERSION_DATE ")"); c = MN_GREEN; }
static bool E_Reboot(void)
{
    if (Mn_AskYN(Menu_RebootPending() ? "REBOOT TO APPLY CHANGES?" : "REBOOT THE ESP32-COCO3?")) Menu_RebootNow();
    return false;
}
static void V_Help(char *b, size_t n, uint8_t &c) { snprintf(b, n, "keys and options"); c = MN_GREEN; }
static bool E_Help(void)                          { HelpMenuChoose(); return false; }
static void Menu_PrevVersion(void);
static void V_Prev(char *b, size_t n, uint8_t &c)
{
    char v[40];
    if (Fw_OtherVersion(v, sizeof v)) { snprintf(b, n, "%s", v); c = MN_GREEN; }
    else { snprintf(b, n, "none"); c = MN_RED; }
}
static bool E_Prev(void) { char v[40]; if (Fw_OtherVersion(v, sizeof v)) Menu_PrevVersion(); return false; }
static void V_Log(char *b, size_t n, uint8_t &c)  { snprintf(b, n, "log + screenshot -> SD"); c = MN_GREEN; }
static bool E_Log(void)
{
    extern volatile bool g_req_log;
    g_req_log = true;          // comme F9 : écrit au retour à l'émulation (capture de l'image du CoCo)
    return true;
}
// AUTORUN (autorun.cpp)
static bool Menu_TextInput(const char *prompt, char *out, size_t outsz, uint8_t y);
static const char *const k_ar_mode[AR_MODES] = { "OFF", "COMMAND", "FILE /ESP-BOOT/AUTORUN.TXT" };
static void V_ArMode(char *b, size_t n, uint8_t &c)
{
    const uint8_t m = Autorun_Mode();
    if (m == AR_FILE && !Autorun_FileExists()) { snprintf(b, n, "FILE (AUTORUN.TXT missing)"); c = MN_YELLOW; return; }
    snprintf(b, n, "%s", k_ar_mode[m < AR_MODES ? m : 0]); c = m ? MN_GREEN : MN_RED;
}
static void C_ArMode(int dir)  { Autorun_SetMode((uint8_t)((Autorun_Mode() + (dir < 0 ? AR_MODES - 1 : 1)) % AR_MODES)); }
static void V_ArCmd(char *b, size_t n, uint8_t &c)  { snprintf(b, n, "%s", Autorun_Command()[0] ? Autorun_Command() : "(none) ENTER to type"); c = Autorun_Command()[0] ? MN_GREEN : MN_RED; }
static bool E_ArCmd(void)
{
    char cmd[100];
    strncpy(cmd, Autorun_Command(), sizeof cmd - 1); cmd[sizeof cmd - 1] = 0;
    if (Menu_TextInput("Command typed at start (ex: RUN\"GAME\"):", cmd, sizeof cmd, 21))
    {
        Autorun_SetCommand(cmd);
        if (cmd[0] && Autorun_Mode() == AR_OFF) Autorun_SetMode(AR_COMMAND);
        s_mn_msg = "Saved. It will be typed at the next start."; s_mn_msg_color = MN_GREEN;
    }
    Menu_KeySync();
    return false;
}
static void V_ArSt(char *b, size_t n, uint8_t &c)   { Autorun_StatusLine(b, n); c = MN_GREEN; }
// écran d'accueil (bootscreen.cpp)
static const char *const k_bs_mode[BS_MODES] = { "OFF", "LOGO + MUSIC", "SD CARD (/ESP-BOOT)" };
static const char *const k_bs_end[BS_ENDS]   = { "KEY OR BUTTON", "5 s", "10 s", "30 s" };
static void V_BsMode(char *b, size_t n, uint8_t &c) { const uint8_t m = BootScreen_Mode(); snprintf(b, n, "%s", k_bs_mode[m < BS_MODES ? m : 0]); c = m ? MN_GREEN : MN_RED; }
static void C_BsMode(int dir)  { BootScreen_SetMode((uint8_t)((BootScreen_Mode() + (dir < 0 ? BS_MODES - 1 : 1)) % BS_MODES)); }
static void V_BsEnd(char *b, size_t n, uint8_t &c)  { const uint8_t e = BootScreen_End(); snprintf(b, n, "%s", k_bs_end[e < BS_ENDS ? e : 0]); c = MN_GREEN; }
static void C_BsEnd(int dir)   { BootScreen_SetEnd((uint8_t)((BootScreen_End() + (dir < 0 ? BS_ENDS - 1 : 1)) % BS_ENDS)); }
static void V_BsSd(char *b, size_t n, uint8_t &c)   { BootScreen_Describe(b, n); c = BootScreen_Mode() == BS_SD ? MN_GREEN : MN_BLUE; }
// HARDWARE HEALTH (health.cpp) : une ligne par mesure, redessinée chaque seconde
template <int I> static void V_Hl(char *b, size_t n, uint8_t &c)
{
    const char *l; uint8_t lv;
    Health_Row(I, &l, b, n, &lv);
    c = lv == HL_BAD ? MN_RED : lv == HL_WARN ? MN_YELLOW : lv == HL_INFO ? MN_WHITE : MN_GREEN;
}
static void (*const k_hl_v[HEALTH_ROWS])(char *, size_t, uint8_t &) = {
    V_Hl<0>, V_Hl<1>, V_Hl<2>, V_Hl<3>, V_Hl<4>, V_Hl<5>, V_Hl<6>, V_Hl<7>,
    V_Hl<8>, V_Hl<9>, V_Hl<10>, V_Hl<11>, V_Hl<12>, V_Hl<13>, V_Hl<14>, V_Hl<15> };
static int B_Health(MnItem *o, int max)
{
    int k = 0;
    for (int i = 0; i < HEALTH_ROWS && k < max; i++)
    {
        const char *l; char v[8]; uint8_t lv;
        Health_Row(i, &l, v, 1, &lv);                // nom seulement
        o[k++] = MnItem{ l, 15, k_hl_v[i], nullptr, nullptr, nullptr, true, nullptr };
    }
    return k;
}
static int X_Health(int y)
{
    y += 2;
    Mn_Text("Emulation is paused here; the web page shows live", 1, y, MN_GREEN, 0); y += 9;
    Mn_Text("values. Bars: each # = 10 %. No voltage sensor.", 1, y, MN_GREEN, 0); y += 9;
    return y;
}
static bool P_Health(void) { static uint32_t t = 0; if (millis() - t < 1000) return false; t = millis(); return true; }
static const MnPage k_pg_health = { "MENU > SYSTEM > HARDWARE HEALTH", nullptr, 0, B_Health, nullptr, X_Health, P_Health, 1 };
static bool Mn_Page(const MnPage *p);
static bool E_Health(void) { Health_RefreshSd(); Mn_Page(&k_pg_health); return false; }

// écran de veille
static const char *const k_sv_delay[SV_COUNT] = { "OFF", "5 min", "10 min", "20 min", "30 min" };
static const char *const k_sv_kind[SVK_COUNT] = { "BLACK SCREEN", "LOGO", "SD CARD (/ESP-BOOT)" };
static void V_SvDel(char *b, size_t n, uint8_t &c)  { const uint8_t d = Saver_Delay(); snprintf(b, n, "%s", k_sv_delay[d < SV_COUNT ? d : 0]); c = d ? MN_GREEN : MN_RED; }
static void C_SvDel(int dir)   { Saver_SetDelay((uint8_t)((Saver_Delay() + (dir < 0 ? SV_COUNT - 1 : 1)) % SV_COUNT)); }
static void V_SvKind(char *b, size_t n, uint8_t &c) { const uint8_t k = Saver_Kind(); snprintf(b, n, "%s", k_sv_kind[k < SVK_COUNT ? k : 0]); c = MN_GREEN; }
static void C_SvKind(int dir)  { Saver_SetKind((uint8_t)((Saver_Kind() + (dir < 0 ? SVK_COUNT - 1 : 1)) % SVK_COUNT)); }
static void V_SvSnd(char *b, size_t n, uint8_t &c)  { snprintf(b, n, "%s", Saver_Sound() ? "ON" : "OFF"); c = Saver_Sound() ? MN_GREEN : MN_RED; }
static void C_SvSnd(int)       { Saver_SetSound(!Saver_Sound()); }
static const MnItem k_mn_sys[] = {
    { "BOOT SCREEN",      22, V_BsMode, C_BsMode, nullptr, nullptr, false },
    { "  ENDS WITH",      22, V_BsEnd,  C_BsEnd,  nullptr, nullptr, false },
    { "  SD CARD HAS",    22, V_BsSd,   nullptr,  nullptr, nullptr, true  },
    { k_mn_sep,           22, nullptr,  nullptr,  nullptr, nullptr, true  },
    { "SCREEN SAVER",     22, V_SvDel,  C_SvDel,  nullptr, nullptr, false },
    { "  SHOWS",          22, V_SvKind, C_SvKind, nullptr, nullptr, false },
    { "  SOUND",          22, V_SvSnd,  C_SvSnd,  nullptr, nullptr, false },
    { k_mn_sep,           22, nullptr,  nullptr,  nullptr, nullptr, true  },
    { "AUTORUN",          22, V_ArMode, C_ArMode, nullptr, nullptr, false },
    { "  COMMAND",        22, V_ArCmd,  nullptr,  E_ArCmd, nullptr, false },
    { "  THIS START",     22, V_ArSt,   nullptr,  nullptr, nullptr, true  },
    { k_mn_sep,           22, nullptr,  nullptr,  nullptr, nullptr, true  },
    { "HARDWARE HEALTH",  22, nullptr, nullptr, E_Health, nullptr, false },
    { "FIRMWARE UPDATE",  22, V_Fw,   nullptr, E_Fw,     nullptr, false },
    { "PREVIOUS VERSION", 22, V_Prev, nullptr, E_Prev,   nullptr, false },
    { "REBOOT",           22, nullptr, nullptr, E_Reboot, Menu_RebootPending, false },
    { "HELP (F10)",       22, V_Help, nullptr, E_Help,   nullptr, false },
    { "TESTER LOG (F9)",  22, V_Log,  nullptr, E_Log,    nullptr, false },
};

// ---------------------------------------------------------------- sous-menu générique
// ---------------------------------------------------------------- MAIN MENU
static int s_mn_main_sel = 2;   // numéro de rubrique (2 = DATA STORAGE, en tête)
static void Mn_MainValue(int i, char *b, size_t n, uint8_t &c)
{
    c = MN_GREEN;
    switch (i)
    {
    case 0:
    {
        char ram[12];
        const unsigned kb = (unsigned)(g_ram_selected / 1024);
        if (kb >= 1024) snprintf(ram, sizeof ram, "%u MB", kb / 1024); else snprintf(ram, sizeof ram, "%u KB", kb);
        snprintf(b, n, "%s  %s  GIME %s", g_cpu6309_sel ? "HD6309" : "MC6809", ram, gime_is_1987() ? "1987" : "1986");
        break;
    }
    case 1:
        snprintf(b, n, "%s  %s  %s", gime_video_monitor_rgb() ? "RGB" : "COMPOSITE",
                 !sf.Artefact ? "NO ARTIFACT" : (g_artifact_swap ? "BLUE-RED" : "RED-BLUE"), Audio_GetStereo() ? "STEREO" : "MONO");
        break;
    case 2: { char nm[40]; Mn_Base((const char *)Disk_Drive.Name_Disk[0], nm, sizeof nm); snprintf(b, n, "D0    %s", nm); if (!Disk_Drive.Name_Disk[0][0]) c = MN_RED; break; }
    case 3: V_Slot(b, n, c); break;
    case 4:
    {
        static const char *const k_kind[4] = { "AUTO", "KEYBOARD", "MOUSE", "GAMEPAD" };
        const char *r = k_kind[UsbPort_Kind(0) & 3];
        const char *l = k_kind[UsbPort_Kind(1) & 3];
        snprintf(b, n, "R: %s  L: %s", r, l);
        break;
    }
    case 5: Net_StatusLine(b, n); if (!Net_Enabled()) c = MN_RED; break;
    case 6: snprintf(b, n, "UPDATE  REBOOT  HELP"); break;
    }
}
static const char *const k_mn_main[7] = { "HARDWARE", "MONITOR", "DATA STORAGE", "CART SLOT & COM", "USB DEVICES", "NETWORK", "SYSTEM" };
// DATA STORAGE en tête (D0-D3, /X0, /X1, VHD 0, VHD 1), mince ligne bleue, puis les autres rubriques
// (4 px plus bas pour laisser la place à la ligne).
static const uint8_t k_mn_order[7] = { 2, 0, 1, 3, 4, 5, 6 };   // ordre affiché (numéros de rubrique)
static const uint8_t k_mn_main_row[7] = { 8, 9, 0, 10, 11, 12, 13 };   // ligne de chaque rubrique
#define MN_SEP_Y  110            // mince ligne bleue sous les disques (rangée 7 finit à y=107)
#define MN_SEP_ADD 4             // décalage des rubriques sous la ligne
extern bool Usb_KeyboardOnline(void);

static const char *const k_mn_main_keys = "UP/DN select  ENTER open  ESC exit  D H M C U N S";
static void Mn_DrawMain(void)
{
    Mn_Frame("MAIN MENU", k_mn_main_keys);
    for (int i = 0; i < 7; i++)
    {
        char v[64]; uint8_t c;
        Mn_MainValue(i, v, sizeof v, c);
        const bool r = (i == 0 && (R_Cpu() || R_Ram())) || (i == 3 && Slot_PendingReboot());
        const int row = k_mn_main_row[i];
        const int yadd = row > 7 ? MN_SEP_ADD : 0;
        const bool sel = i == s_mn_main_sel;
        Mn_Row(row, k_mn_main[i], v, c, 20, sel, false, r, true, yadd);
        if (i == 4)   // USB DEVICES : clavier en ligne (vert) ou aucun (rouge)
        {
            const bool kb = Usb_KeyboardOnline();
            const bool chg = kb && UsbKbd_Changed();   // autre clavier que celui du CoCo : jaune
            Mn_Text("KBD", 47, Mn_Y(row) + yadd, sel ? MN_WHITE : (chg ? MN_YELLOW : (kb ? MN_GREEN : MN_RED)), sel ? MN_BLUE : 0);
        }
    }
    // Lignes 1 à 7 : D1-D3, puis disques virtuels DriveWire /X0 /X1 et VHD 0 / VHD 1
    static const char *const k_lbl[7] = { "D1", "D2", "D3", "/X0", "/X1", "VHD 0", "VHD 1" };
    for (int k = 0; k < 7; k++)
    {
        const char *path = k < 3 ? (const char *)Disk_Drive.Name_Disk[k + 1]
                                 : g_dw_name[k == 3 ? 0 : (k == 4 ? 1 : (k == 5 ? 4 : 5))];
        char nm[40], v[48];
        Mn_Base(path, nm, sizeof nm);
        snprintf(v, sizeof v, "%-6s%s", k_lbl[k], nm);
        Mn_Text(v, 20, Mn_Y(1 + k), path[0] ? MN_GREEN : MN_RED, 0, 33);
    }
    line(0, MN_SEP_Y, 319, MN_SEP_Y, MN_BLUE);
}

static int Mn_Pos(int id) { for (int p = 0; p < 7; p++) if (k_mn_order[p] == id) return p; return 0; }

#define MN_N(a) ((int)(sizeof a / sizeof a[0]))
static const MnPage k_pg_hw  = { "MENU > HARDWARE",     k_mn_hw,  MN_N(k_mn_hw),  nullptr, nullptr, nullptr, nullptr };
static const MnPage k_pg_mon = { "MENU > MONITOR",      k_mn_mon, MN_N(k_mn_mon), nullptr, nullptr, nullptr, nullptr };
static const MnPage k_pg_sto = { "MENU > DATA STORAGE", k_mn_sto, MN_N(k_mn_sto), nullptr, "ENTER: SD card or PC share (P)  DEL: eject", nullptr, nullptr };
static int X_Sys(int y)
{
    y += 4;
    Mn_Text("AUTORUN: typed at the BASIC prompt at start.", 1, y, MN_GREEN, 0); y += 9;
    Mn_Text("FILE: one command per line. Any key cancels.", 1, y, MN_GREEN, 0); y += 9;
    return y;
}
static const MnPage k_pg_sys = { "MENU > SYSTEM",       k_mn_sys, MN_N(k_mn_sys), nullptr, nullptr, X_Sys, nullptr, 1 };   // rangées serrées
// Ouvre la rubrique i ; rend true s'il faut retourner à l'émulation.
static bool Mn_Open(int i)
{
    switch (i)
    {
    case 0: return Mn_Page(&k_pg_hw);
    case 1: return Mn_Page(&k_pg_mon);
    case 2: return Mn_Page(&k_pg_sto);
    case 3: return Pg_Cart();
    case 4: return Pg_Usb();
    case 5: return Pg_Net();
    case 6: return Mn_Page(&k_pg_sys);
    }
    return false;
}

static void Mn_LeaveToEmulation(void)
{
    for (int pass = 0; pass < 2; pass++) { vga->clear(0); vga->show(); }
}

void EMU_Draw_Menu(void)
{
    Fw_ValidateIfDue(true);   // le menu F12 s'ouvre : la version en cours fonctionne
    Mn_Redraw2(Mn_DrawMain);
    Menu_KeySync();          // F12 encore tenue
    while (1)
    {
        const uint8_t k = Menu_GetKey();
        int open = -1;
        switch (k)
        {
        case MENU_UP:    if (Mn_Pos(s_mn_main_sel) > 0) { s_mn_main_sel = k_mn_order[Mn_Pos(s_mn_main_sel) - 1]; Mn_Redraw2(Mn_DrawMain); } break;
        case MENU_DOWN:  if (Mn_Pos(s_mn_main_sel) < 6) { s_mn_main_sel = k_mn_order[Mn_Pos(s_mn_main_sel) + 1]; Mn_Redraw2(Mn_DrawMain); } break;
        case MENU_ENTER: open = s_mn_main_sel; break;
        case MENU_H: open = 0; break;
        case MENU_M: open = 1; break;
        case MENU_D: open = 2; break;
        case MENU_C: open = 3; break;
        case MENU_U: open = 4; break;
        case MENU_N: open = 5; break;
        case MENU_S: open = 6; break;
        case MENU_ESC:
            if (Menu_RebootPending())         // CPU, RAM ou cartouches changés
            {
                // question en rouge qui clignote sur la ligne des touches. Y = sauvegarde et
                // redémarrage N = on reste dans le MAIN MENU (choix gardés, toujours marqués (R)) ESC
                // = sortie sans redémarrer
                const int answer = Mn_Ask("REBOOT TO APPLY CHANGES?  Y / N / ESC", k_mn_main_keys);
                if (answer == 1) { Menu_RebootNow(); return; }
                if (answer == 2) { Mn_Redraw2(Mn_DrawMain); break; }   // reste dans le MAIN MENU
                Menu_CancelPending();         // ESC : choix annulés (ni appliqués ni sauvegardés)
            }
            Mn_LeaveToEmulation();
            return;
        default: break;
        }
        if (open >= 0)
        {
            s_mn_main_sel = open;
            if (Mn_Open(open)) { Mn_LeaveToEmulation(); return; }
            Mn_Redraw2(Mn_DrawMain);
            Menu_KeySync();
        }
        vTaskDelay(2);
    }
}

// mise à jour du firmware (ex-touche F du menu principal) : choix d'un FLH / .BIN sur la SD, ou retour
// à la version précédente (P).
static void Menu_PrevVersion(void)   // extrait du flux firmware (aussi SYSTEM > PREVIOUS VERSION)
{
    char v[40], q[64];
    Fw_OtherVersion(v, sizeof v);
    snprintf(q, sizeof q, "Restart the previous firmware %s?", v);
    for (int pass = 0; pass < 2; pass++)
    {
        vga->clear(0);
        DrawText(q,0,27,0,0,255,0);
        DrawText(" Y/N",(uint8_t)strlen(q),27,0,0,0b11100000,0);
        vga->show();
    }
    while (sf.DIRECT_Key_Code == MENU_P) vTaskDelay(10);
    while (1)
    {
        const uint8_t k = sf.DIRECT_Key_Code;
        if (k == MENU_Y) { Fw_BootOther(); break; }
        if (k == MENU_N || k == MENU_ESC) break;
        vTaskDelay(20);
    }
    while (sf.DIRECT_Key_Code != 0) vTaskDelay(10);
}

static void Menu_FirmwareFlow(void)
{
    char *file;
    bool IsFileValid = false;
    (void)IsFileValid;
            file = Firmware_Choose();
            if (strcmp(file, "PREV") == 0)   // redémarrer sur la version précédente
                Menu_PrevVersion();
            else if (strcmp(file, "NF") == 0)
            {
                debugln("No File.");
            }
            else
            {
                vga->clear(0);
                vga->show();
                vga->clear(0);
                vga->show();
                DrawText("Install this firmware?",0,27,0,0,255,0);
                DrawText(" Y/N",23,27,0,0,0b11100000,0);
                vga->show();
                debugln(file);
                while(1)
                {
                    switch (sf.DIRECT_Key_Code)
                    {
                    case MENU_Y:

                        vga->clear(0);
                        vga->show();
                        vga->clear(0);
                        vga->show();
                        DrawText("Checking firmware validity...",0,27,0,0,255,0);
                        vga->show();
                        if (IsFileValid = ValidFirmwareFile(file))
                        {
                            debugln("File Valid");
                            vga->clear(0);
                            vga->show();
                            vga->clear(0);
                            vga->show();
                            DrawText("The firmware is valid.          ",0,26,0,0,0b00011100,0);
                            DrawText("Copying file...          ",0,27,0,0,255,0);
                            vga->show();
                            copyFile(file, "/qprcx.rty"); // random name, Will be removed at boot after flash (Because we can't…)
                            vga->clear(0);
                            vga->show();
                            vga->clear(0);
                            vga->show();
                            DrawText("Rebooting and installing firmware...          ",0,27,0,0,255,0);
                            vga->show();
                            vTaskDelay(2000);
                            esp_restart();
                        }
                        else
                        {
                            vga->clear(0);
                            vga->show();
                            vga->clear(0);
                            vga->show();
                            DrawText("Invalid firmware!          ",0,27,0,0,0b11100000,0);
                            DrawText("Checking firmware validity...",0,28,0,0,255,0);
                            vga->show();
                            debugln("File NOT Valid");
                            vTaskDelay(1500);
                            return;
                        }

                        break;
                    case MENU_N:
                        return;
                        break;
                    default:
                        break;
                    }
                    vTaskDelay(1);
                }
            }

}

// ---------------------------------------------------------------------------
// sous-menu souris USB (touche M du menu principal). P : port (OFF / RIGHT / LEFT) R : résolution (LOW
// / HIGH) + / - : sensibilité (1-8) ESC : retour
// ---------------------------------------------------------------------------
static void DrawMouseMenu(void)
{
    vga->clear(0);
    vga->show();
    vga->clear(0);
    vga->show();
    DrawText ("Mouse settings:",0,0,0,0,255,0);
    line(0,10,319,10,3);

    const uint8_t port = (g_mouse_mode == 0) ? 0 : (g_mouse_mode <= 2 ? 1 : 2);
    const bool hires = (g_mouse_mode == 2 || g_mouse_mode == 4) || (g_mouse_mode == 0 && g_mouse_hires_pref);

    DrawText ("P",0,2,0,0,255,0);
    DrawText ("ort",1,2,0,0,252,0);
    DrawText (port == 0 ? "OFF        " : (port == 1 ? "RIGHT joystick" : "LEFT joystick "),16,2,0,0,port ? 28 : 224,0);

    DrawText ("R",0,4,0,0,255,0);
    DrawText ("esolution",1,4,0,0,252,0);
    DrawText (hires ? "HIGH (Hi-Res interface)  " : "LOW (Tandy mouse, 64 steps)",16,4,0,0,28,0);

    DrawText ("+ / -",0,6,0,0,255,0);
    DrawText ("Sensitivity",6,6,0,0,252,0);
    {
        char bar[12];
        for (int i = 0; i < 8; i++) bar[i] = (i < g_mouse_sens) ? '#' : '.';
        bar[8] = 0;
        DrawText (bar,18,6,0,0,28,0);
        char n[4]; snprintf(n, sizeof n, "%u", g_mouse_sens);
        DrawText (n,28,6,0,0,255,0);
    }

    DrawText ("OS-9 / Multi-Vue / GShell default:",0,10,0,0,255,0);
    DrawText ("RIGHT joystick, LOW resolution.",0,11,0,0,255,0);
    DrawText ("HIGH needs the Hi-Res setting in OS-9.",0,12,0,0,255,0);
    DrawText ("Left button = fire 1, right button = fire 2.",0,14,0,0,255,0);
    DrawText ("ESC to go back.",0,16,0,0,255,0);
    vga->show();
}

void MouseMenuChoose(void)
{
    DrawMouseMenu();
    Menu_KeySync();
    while (1)
    {
        const uint8_t port = (g_mouse_mode == 0) ? 0 : (g_mouse_mode <= 2 ? 1 : 2);
        bool hires = (g_mouse_mode == 2 || g_mouse_mode == 4) || (g_mouse_mode == 0 && g_mouse_hires_pref);
        bool changed = false;
        switch (Menu_GetKey())
        {
        case MENU_P:                                  // OFF -> RIGHT -> LEFT -> OFF
        {
            const uint8_t np = (uint8_t)((port + 1) % 3);
            g_mouse_mode = (np == 0) ? 0 : (np == 1 ? (hires ? 2 : 1) : (hires ? 4 : 3));
            changed = true;
            break;
        }
        case 0x15:                                    // R : basse <-> haute résolution
            hires = !hires;
            g_mouse_hires_pref = hires;
            if (port) g_mouse_mode = (port == 1) ? (hires ? 2 : 1) : (hires ? 4 : 3);
            changed = true;
            break;
        case MENU_EQUAL:
        case MENU_KP_PLUS:
            if (g_mouse_sens < 8) { g_mouse_sens++; changed = true; }
            break;
        case MENU_MINUS:
        case MENU_KP_MIN:
            if (g_mouse_sens > 1) { g_mouse_sens--; changed = true; }
            break;
        case MENU_ESC:
            return;
        default:
            break;
        }
        if (changed)
        {
            DrawMouseMenu();
        }
        vTaskDelay(2);
    }
}

// ---------------------------------------------------------------------------
// sous-menu « Keyboard joystick » (touche K du menu principal). P : port OFF / RIGHT / LEFT K : jeu de
// touches ESC : retour
// ---------------------------------------------------------------------------
static void DrawKbJoyMenu(void)
{
    vga->clear(0);
    vga->show();
    vga->clear(0);
    vga->show();
    DrawText ("Keyboard joystick:",0,0,0,0,255,0);
    line(0,10,319,10,3);

    DrawText ("P",0,2,0,0,255,0);
    DrawText ("ort",1,2,0,0,252,0);
    DrawText (g_kbjoy_port == 0 ? "OFF           " : (g_kbjoy_port == 1 ? "RIGHT joystick" : "LEFT joystick "),16,2,0,0,g_kbjoy_port ? 28 : 224,0);

    DrawText ("K",0,4,0,0,255,0);
    DrawText ("eys",1,4,0,0,252,0);
    static const char *k_sets[3][3] = {
        { "ARROW keys    ", "Move : arrow keys", "Fire : F1 / F2" },
        { "NUMERIC KEYPAD", "Move : 8 2 4 6 (7 9 1 3 diagonals)", "Fire : 0 / keypad ENTER" },
        { "W A S D       ", "Move : W A S D", "Fire : SPACE / F" } };
    const uint8_t k = g_kbjoy_set % 3;
    DrawText (k_sets[k][0],16,4,0,0,28,0);
    DrawText (k_sets[k][1],2,6,0,0,255,0);
    DrawText (k_sets[k][2],2,7,0,0,255,0);

    DrawText ("While ON, these keys go to the joystick",0,10,0,0,255,0);
    DrawText ("only, not to the CoCo keyboard.",0,11,0,0,255,0);
    DrawText ("Do not use the same port as the mouse.",0,12,0,0,255,0);
    DrawText ("ESC to go back.",0,14,0,0,255,0);
    vga->show();
}

void KbJoyMenuChoose(void)
{
    DrawKbJoyMenu();
    Menu_KeySync();
    while (1)
    {
        bool changed = false;
        switch (Menu_GetKey())
        {
        case MENU_P:
            g_kbjoy_port = (uint8_t)((g_kbjoy_port + 1) % 3);
            changed = true;
            break;
        case MENU_K:
            g_kbjoy_set = (uint8_t)((g_kbjoy_set + 1) % 3);
            changed = true;
            break;
        case MENU_ESC:
            return;
        default:
            break;
        }
        if (changed)
        {
            DrawKbJoyMenu();
        }
        vTaskDelay(2);
    }
}

// =====================================================================
// PORT D'EXTENSION (menu F12 -> S)
// =====================================================================
// Une seule cartouche (le Multi-Pak viendra en 7b) : contrôleur de disquette FD-502 avec Disk BASIC
// 2.1 (défaut), cartouche ROM .ROM / .CCC ou port vide
static int s_cart_slot = 0;   // slot dont on choisit la cartouche (navigateur)
static uint8_t s_cart_newmode = SLOT_ROM;   // SLOT_ROM ou SLOT_O90 après le choix du fichier
// ROM installée dans un slot (une seule O90 / GMC / PSG) et retenue pour son type
static void Slot_InstallRom(int slot, uint8_t mode, const char *path)
{
    snprintf(g_slotcfg.name[slot], sizeof(g_slotcfg.name[0]), "%s", path);
    if (mode == SLOT_O90 || mode == SLOT_GMC || mode == SLOT_CPSG)   // une seule de chaque
        for (int i = 0; i < MPI_SLOTS; i++) if (g_slotcfg.mode[i] == mode) g_slotcfg.mode[i] = SLOT_EMPTY;
    g_slotcfg.mode[slot] = mode;
    CartLast_Remember(mode, path);
}

static void Menu_RebootNow(void)
{
    extern void Sys_Restart(void);
    SaveConfigToSD();   // choix du menu (CPU, RAM, GIME, slots...) gardés pour le redémarrage
    UsbDev_SaveIfDirty();
    Sys_Restart();   // arrêt propre (CPU, son, vidéo, SD) puis esp_restart
    vTaskDelay(200);
}

// choix appliqués seulement au redémarrage : CPU (C), RAM (E) cartouches / slots (S). Une seule
// question pour les trois, en quittant F12 (ESC) ou avec R : « Reboot to apply changes? Y/N »
static bool Menu_RebootPending(void)
{
    return g_cpu6309_sel != g_cpu6309_run || g_ram_selected != gime_ram_size() || Slot_PendingReboot();
}
static void Menu_CancelPending(void)
{
    g_cpu6309_sel = g_cpu6309_run;
    g_ram_selected = gime_ram_size();
    g_slotcfg = g_slotcfg_run;
}
// Question Y/N sur la ligne 27 ; ESC = N. Rend true sur Y.
static bool Menu_AskYN(const char *q)
{
    for (int pass = 0; pass < 2; pass++)
    {
        DrawText(q, 0, 27, 0, 0, 255, 0);
        DrawText(" Y/N", (uint8_t)strlen(q), 27, 0, 0, 0b11100000, 0);
        vga->show();
    }
    Menu_KeySync();                       // touche qui a ouvert la question
    while (1)
    {
        const uint8_t k = Menu_GetKey();
        if (k == MENU_Y) { Menu_KeySync(); return true; }
        if (k == MENU_N || k == MENU_ESC) { Menu_KeySync(); return false; }
        vTaskDelay(20);
    }
}

// Choix du menu différent de ce qui est en service (appliqué au redémarrage)
static bool Slot_PendingReboot(void)
{
    const SlotConfig &a = g_slotcfg, &r = g_slotcfg_run;
    if (a.mpi != r.mpi) return true;
    if (a.mpi && a.sw != r.sw) return true;
    const int n = a.mpi ? MPI_SLOTS : 1;
    for (int i = 0; i < n; i++)
    {
        if (a.mode[i] != r.mode[i]) return true;
        if (SLOT_HAS_ROM(a.mode[i]) && (a.autostart[i] != r.autostart[i] || strcmp(a.name[i], r.name[i]) != 0)) return true;
    }
    return false;
}

static void Slot_Describe(char *buf, size_t n, const SlotConfig &c, int i)
{
    if (c.mode[i] == SLOT_FDC)        snprintf(buf, n, "Disk controller (Disk BASIC 2.1)");
    else if (c.mode[i] == SLOT_EMPTY) snprintf(buf, n, "Empty");
    else if (c.mode[i] == SLOT_RS232) snprintf(buf, n, "RS-232 Pak (6551 serial port)");
    else
    {
        const char *b = strrchr(c.name[i], '/');
        snprintf(buf, n, "%s %.24s%s", c.mode[i] == SLOT_O90 ? "O90" : (c.mode[i] == SLOT_GMC ? "GMC" : (c.mode[i] == SLOT_CPSG ? "PSG" : "ROM")), b ? b + 1 : c.name[i], c.autostart[i] ? " (auto)" : "");
    }
}

void DrawSlotMenu(int sel)
{
    char buf[64];
    const SlotConfig &c = g_slotcfg;
    for (int pass = 0; pass < 2; pass++)
    {
    vga->clear(0);
    DrawText ("Cartridge slot (expansion port):",0,0,0,0,255,0);
    line(0,10,319,10,3);

    DrawText ("M",0,2,0,0,255,0);
    DrawText ("ulti-Pak interface",1,2,0,0,252,0);
    DrawText (c.mpi ? "ON (4 slots)" : "OFF",22,2,0,0,c.mpi ? 28 : 224,0);
    int y = 4;
    if (c.mpi)
    {
        DrawText ("W",0,3,0,0,255,0);
        DrawText ("Multi-Pak switch",2,3,0,0,252,0);
        snprintf(buf, sizeof buf, "slot %d (used at power-on)", c.sw + 1);
        DrawText (buf,22,3,0,0,28,0);
        y = 5;
        for (int i = 0; i < MPI_SLOTS; i++, y++)
        {
            char line_[64], d[56];
            Slot_Describe(d, sizeof d, c, i);
            snprintf(line_, sizeof line_, "%c%d %s", (i == sel) ? '>' : ' ', i + 1, d);
            DrawText (line_,0,y,0,0,(i == sel) ? 255 : 28,(i == sel) ? 3 : 0);
        }
        DrawText ("1-4: select a slot",0,y + 1,0,0,255,0);
        y += 3;
    }
    else
    {
        Slot_Describe(buf, sizeof buf, c, 0);
        DrawText ("Cartridge:",0,y,0,0,255,0);
        DrawText (buf,11,y,0,0,28,0);
        y += 2;
    }
    if (c.mpi) { snprintf(buf, sizeof buf, "For slot %d:", sel + 1); DrawText (buf,0,y++,0,0,255,0); }
    DrawText ("D",0,y,0,0,255,0);
    DrawText ("isk controller FD-502 (Disk BASIC 2.1)",1,y++,0,0,252,0);
    DrawText ("C",0,y,0,0,255,0);
    DrawText ("artridge ROM file (.ROM / .CCC)...",1,y++,0,0,252,0);
    DrawText ("O",0,y,0,0,255,0);
    DrawText ("rchestra-90 stereo music card (ROM)...",1,y++,0,0,252,0);
    DrawText ("G",0,y,0,0,255,0);
    DrawText ("ame Master Cartridge (ROM)...",1,y++,0,0,252,0);
    DrawText ("P",0,y,0,0,255,0);
    DrawText ("SG: CoCo PSG card, YM2149 (ROM)...",1,y++,0,0,252,0);
    DrawText ("R",0,y,0,0,255,0);
    DrawText ("S-232 Pak (6551 serial port, $FF68)",1,y++,0,0,252,0);
    DrawText ("N",0,y,0,0,255,0);
    DrawText ("othing (empty slot)",1,y++,0,0,252,0);
    DrawText ("A",0,y,0,0,255,0);
    DrawText ("utostart ROM cartridge",1,y,0,0,252,0);
    DrawText (c.autostart[sel] ? "ON " : "OFF",24,y,0,0,c.autostart[sel] ? 28 : 224,0);
    DrawText ("(game paks)",28,y++,0,0,255,0);
    bool rs = false;
    for (int i = 0; i < (c.mpi ? MPI_SLOTS : 1); i++) if (c.mode[i] == SLOT_RS232) rs = true;
    if (rs)                                              // liaison et vitesse (immédiates)
    {
        static const char *const k_lk[RS_LINK_COUNT] = { "none", "UART 41/42", "loopback", "modem Wi-Fi" };
        char sp[16];
        if (g_rs_speed) snprintf(sp, sizeof sp, "%lu", (unsigned long)k_rs_speed[g_rs_speed]); else snprintf(sp, sizeof sp, "auto");
        DrawText ("L",0,y,0,0,255,0);
        DrawText ("ink",1,y,0,0,252,0);
        DrawText (k_lk[g_rs_link < RS_LINK_COUNT ? g_rs_link : 0],5,y,0,0,28,0);
        DrawText ("B",20,y,0,0,255,0);
        DrawText ("aud",21,y,0,0,252,0);
        DrawText (sp,25,y,0,0,28,0);
        DrawText ("(RS-232)",33,y++,0,0,255,0);
    }
    else y++;
    DrawText ("One disk controller only. A ROM cartridge",0,y++,0,0,255,0);
    DrawText ("alone replaces it (DriveWire, VHD still work).",0,y++,0,0,255,0);
    DrawText ("ROM files: 32 KB, or up to 512 KB bank-switched.",0,y++,0,0,255,0);
    if (Slot_PendingReboot())
        DrawText ("(reboot to apply)",0,y + 1,0,0,224,0);
    DrawText ("ESC to go back.",0,27,0,0,255,0);
    vga->show();
    }
}

void SlotMenuChoose(void)
{
    int sel = g_slotcfg.mpi ? g_slotcfg.sw : 0;
    DrawSlotMenu(sel);
    Menu_KeySync();
    while (1)
    {
        const uint8_t k = Menu_GetKey();
        bool changed = false, save = true;
        SlotConfig &c = g_slotcfg;
        switch (k)
        {
        case MENU_M:
            c.mpi ^= 1;
            if (!c.mpi) sel = 0;
            changed = true;
            break;
        case MENU_W:
            if (c.mpi) { c.sw = (uint8_t)((c.sw + 1) & 3); changed = true; }
            break;
        case MENU_1: case MENU_2: case MENU_3: case MENU_4:
            if (c.mpi) { sel = (k == MENU_1) ? 0 : (k == MENU_2) ? 1 : (k == MENU_3) ? 2 : 3; changed = true; save = false; }
            break;
        case MENU_D:
            for (int i = 0; i < MPI_SLOTS; i++) if (c.mode[i] == SLOT_FDC) c.mode[i] = SLOT_EMPTY;   // un seul contrôleur
            c.mode[sel] = SLOT_FDC;
            changed = true;
            break;
        case MENU_N:
            c.mode[sel] = SLOT_EMPTY;
            changed = true;
            break;
        case MENU_R:                          // RS-232 Pak (un seul)
            for (int i = 0; i < MPI_SLOTS; i++) if (c.mode[i] == SLOT_RS232) c.mode[i] = SLOT_EMPTY;
            c.mode[sel] = SLOT_RS232;
            changed = true;
            break;
        case MENU_L:                          // liaison du RS-232 Pak (appliquée tout de suite)
            g_rs_link = (uint8_t)((g_rs_link + 1) % RS_LINK_COUNT);
            Rs_ApplyLink();
            changed = true;
            break;
        case MENU_B:                          // vitesse réelle (auto = celle programmée)
            g_rs_speed = (uint8_t)((g_rs_speed + 1) % RS_SPEED_COUNT);
            Rs_ApplyLink();
            changed = true;
            break;
        case MENU_C:
        case MENU_O:                          // Orchestra-90 = sa ROM + 2 CNA
        case MENU_G:                          // Game Master Cartridge
        case MENU_P:                          // CoCo PSG
            ClearFileList();                  // pas d'aperçu d'un ancien disque
            s_cart_slot = sel;
            s_cart_newmode = (k == MENU_O) ? SLOT_O90 : ((k == MENU_G) ? SLOT_GMC : ((k == MENU_P) ? SLOT_CPSG : SLOT_ROM));
            DiskMenuChoose_1(8);              // choisit name[sel], mode[sel] = SLOT_ROM (sauvegardé)
            s_browse_cart = false;
            ClearFileList();
            changed = true;
            save = false;
            break;
        case MENU_A:
            c.autostart[sel] ^= 1;
            changed = true;
            break;
        case MENU_ESC:                        // la question du redémarrage est posée en quittant F12
            return;
        default:
            break;
        }
        if (changed)
        {
            (void)save;                       // sauvegarde à la sortie du menu F12
            DrawSlotMenu(sel);
        }
        vTaskDelay(2);
    }
}

// =====================================================================
// SAVE STATE (menu F12 -> V) : 4 emplacements par logiciel
// =====================================================================
static void DrawStateMenu(const char *msg, uint8_t msg_color)
{
    char name[48], buf[64];
    State_CurrentName(name, sizeof name);
    for (int pass = 0; pass < 2; pass++)
    {
        vga->clear(0);
        DrawText ("Save / load state:",0,0,0,0,255,0);
        line(0,10,319,10,3);
        DrawText ("Software:",0,2,0,0,255,0);
        DrawText (name,10,2,0,0,28,0);
        for (uint8_t i = 0; i < 4; i++)
        {
            char d[48];
            State_Describe(i, d, sizeof d);
            snprintf(buf, sizeof buf, "%c%u %s", (i == g_state_slot) ? '>' : ' ', (unsigned)i + 1, d);
            DrawText (buf,0,4 + i,0,0,(i == g_state_slot) ? 255 : 28,(i == g_state_slot) ? 3 : 0);
        }
        DrawText ("1-4",0,9,0,0,255,0);
        DrawText ("select a slot",4,9,0,0,252,0);
        DrawText ("S",0,10,0,0,255,0);
        DrawText ("ave the CoCo to this slot",1,10,0,0,252,0);
        DrawText ("L",0,11,0,0,255,0);
        DrawText ("oad this slot (back to the CoCo)",1,11,0,0,252,0);
        DrawText ("N",0,12,0,0,255,0);
        DrawText ("ext software with saved states",1,12,0,0,252,0);
        DrawText ("While running: F5 saves over the oldest",0,13,0,0,28,0);
        DrawText ("slot, F8 loads the newest (highest #).",0,14,0,0,28,0);
        DrawText ("Older states: select 1-4 here, then L.",0,15,0,0,28,0);
        DrawText ("A state needs the same CPU, RAM size and",0,16,0,0,255,0);
        DrawText ("cartridges; disks are mounted from the state.",0,17,0,0,255,0);
        if (msg) DrawText (msg,0,20,0,0,msg_color,0);
        DrawText ("ESC to go back.",0,27,0,0,255,0);
        vga->show();
    }
}

static bool State_AskYN(const char *q)
{
    for (int pass = 0; pass < 2; pass++)
    {
        DrawText (q,0,22,0,0,255,0);
        DrawText (" Y/N",(uint8_t)strlen(q),22,0,0,0b11100000,0);
        vga->show();
    }
    while (sf.DIRECT_Key_Code != 0) vTaskDelay(10);
    while (1)
    {
        const uint8_t r = sf.DIRECT_Key_Code;
        if (r == MENU_Y) { while (sf.DIRECT_Key_Code == MENU_Y) vTaskDelay(10); return true; }
        if (r == MENU_N || r == MENU_ESC) { while (sf.DIRECT_Key_Code == r) vTaskDelay(10); return false; }
        vTaskDelay(20);
    }
}

static void StateMenuChoose(void)
{
    DrawStateMenu(nullptr, 0);
    Menu_KeySync();
    while (sf.DIRECT_Key_Code == MENU_V) vTaskDelay(10);
    while (1)
    {
        const uint8_t k = Menu_GetKey();
        switch (k)
        {
        case MENU_1: case MENU_2: case MENU_3: case MENU_4:
            g_state_slot = (k == MENU_1) ? 0 : (k == MENU_2) ? 1 : (k == MENU_3) ? 2 : 3;
            DrawStateMenu(nullptr, 0);
            break;
        case MENU_S:
        {
            char d[48];
            State_Describe(g_state_slot, d, sizeof d);
            while (sf.DIRECT_Key_Code == MENU_S) vTaskDelay(10);
            if (strcmp(d, "(empty)") != 0)
            {
                char q[40];
                snprintf(q, sizeof q, "Replace slot %u?", (unsigned)g_state_slot + 1);
                if (!State_AskYN(q)) { DrawStateMenu(nullptr, 0); break; }
            }
            DrawStateMenu("Saving, please wait...", 252);
            const bool ok = State_Save(g_state_slot, g_wait_before_menu);
            DrawStateMenu(ok ? "State saved." : "NOT SAVED: SD card missing or full.", ok ? 28 : 224);
            break;
        }
        case MENU_L:
        {
            while (sf.DIRECT_Key_Code == MENU_L) vTaskDelay(10);
            DrawStateMenu("Loading, please wait...", 252);
            char err[40];
            bool w = g_wait_before_menu;
            if (State_Load(g_state_slot, &w, err, sizeof err))
            {
                g_wait_before_menu = w;
                s_menu_exit_all = true;  // retour direct au CoCo
                return;
            }
            char msg[56];
            snprintf(msg, sizeof msg, "NOT LOADED: %s", err);
            DrawStateMenu(msg, 224);
            break;
        }
        case MENU_N:                                 // logiciel suivant
        {
            while (sf.DIRECT_Key_Code == MENU_N) vTaskDelay(10);
            const int n = State_NextSoftware();
            char msg[48];
            if (n) snprintf(msg, sizeof msg, "%d software(s) with saved states.", n);
            else   snprintf(msg, sizeof msg, "No saved state on the SD card.");
            DrawStateMenu(msg, 28);
            break;
        }
        case MENU_ESC:
            return;
        default:
            break;
        }
        vTaskDelay(2);
    }
}

// =====================================================================
// PORT SÉRIE DU COCO (bit banger) : menu F12 -> B 1-4 : destination (appliquée tout de suite), B :
// vitesse
// =====================================================================
static void DrawBitBangMenu(void)
{
    static const char *const k_items[BB_MODE_COUNT] = {
        "OFF (input reads 0)",
        "PRINTER -> text file on the SD card",
        "DRIVEWIRE built-in server (/x0-/x3)",
        "UART pins 41 / 42 (PC DriveWire, terminal)",
        "DRIVEWIRE server on a PC, by Wi-Fi (menu N)",
        "MODEM by Wi-Fi (BBS, telnet)" };
    char buf[64];
    for (int pass = 0; pass < 2; pass++)
    {
        vga->clear(0);
        DrawText ("Bit banger serial port ($FF20 / $FF22):",0,0,0,0,255,0);
        line(0,10,319,10,3);
        for (uint8_t i = 0; i < BB_MODE_COUNT; i++)
        {
            snprintf(buf, sizeof buf, "%c%u %s", (i == g_bb_mode) ? '>' : ' ', (unsigned)i + 1, k_items[i]);
            DrawText (buf,0,2 + i,0,0,(i == g_bb_mode) ? 255 : 28,(i == g_bb_mode) ? 3 : 0);
        }
        DrawText ("B",0,7,0,0,255,0);
        DrawText ("aud rate:",1,7,0,0,252,0);
        if (g_bb_speed) snprintf(buf, sizeof buf, "%lu (fixed)", (unsigned long)k_bb_speed[g_bb_speed]);
        else
        {
            const uint32_t b = Bb_CurrentBaud();
            if (b) snprintf(buf, sizeof buf, "AUTO (now %lu)    ", (unsigned long)b);
            else   snprintf(buf, sizeof buf, "AUTO              ");
        }
        DrawText (buf,11,7,0,0,28,0);
        DrawText ("PRINTER: LLIST, PRINT #-2 -> /PRINTS/PRTnnnn.TXT",0,9,0,0,28,0);
        DrawText ("  (file closed 5 s after the last character).",0,10,0,0,28,0);
        DrawText ("DRIVEWIRE: NitrOS-9 'dw' (bit banger) disks and",0,11,0,0,28,0);
        DrawText ("  DriveWire ROMs, without a PC (mount /x0 in D).",0,12,0,0,28,0);
        DrawText ("UART: 3.3 V TTL serial, e.g. a PC DriveWire",0,13,0,0,28,0);
        DrawText ("  server at 115200 (CoCo 3 at 1.79 MHz).",0,14,0,0,28,0);
        DrawText ("WI-FI: same as UART, through the network (the",0,15,0,0,28,0);
        DrawText ("  Becker port then stays on the SD disks).",0,16,0,0,28,0);
        if (g_bb_mode == BB_UART && Bb_UartBlocked())
            DrawText ("UART busy: used by the RS-232 Pak (menu S).",0,17,0,0,224,0);
        if (g_bb_mode == BB_NETDW && !NetDw_Enabled())
            DrawText ("Turn DriveWire ON in menu N (Wi-Fi) first.",0,17,0,0,224,0);
        DrawText ("AUTO measures the speed sent by the CoCo.",0,18,0,0,255,0);
        DrawText ("ESC to go back.",0,27,0,0,255,0);
        vga->show();
    }
}

static void BitBangMenuChoose(void)
{
    DrawBitBangMenu();
    Menu_KeySync();
    while (sf.DIRECT_Key_Code == MENU_B) vTaskDelay(10);
    while (1)
    {
        const uint8_t k = Menu_GetKey();
        switch (k)
        {
        case MENU_1: case MENU_2: case MENU_3: case MENU_4: case MENU_5:
            g_bb_mode = (k == MENU_1) ? BB_OFF : (k == MENU_2) ? BB_PRINTER : (k == MENU_3) ? BB_DRIVEWIRE : (k == MENU_4) ? BB_UART : BB_NETDW;
            Bb_Reset();
            DrawBitBangMenu();
            break;
        case MENU_B:
            g_bb_speed = (uint8_t)((g_bb_speed + 1) % BB_SPEED_COUNT);
            Bb_Reset();
            DrawBitBangMenu();
            break;
        case MENU_ESC:
            return;
        default:
            break;
        }
        vTaskDelay(2);
    }
}

// =====================================================================
// RÉSEAU WI-FI (menu F12 -> N) 1 : marche / arrêt S : chercher les réseaux Z : fuseau horaire
// =====================================================================
extern char Note_HidToChar(uint8_t k, bool shift);   // main.cpp
extern uint8_t HID_ModifierByte;

// Saisie d'un texte au clavier (ligne 'y'). Entrée = valider, Échap = annuler Verr. Maj. = majuscules.
// Rend false si annulé.
static bool Menu_TextInput(const char *prompt, char *out, size_t outsz, uint8_t y)
{
    size_t len = strlen(out);
    bool caps = false, dirty = true;
    uint8_t prev = sf.DIRECT_Key_Code;
    while (1)
    {
        if (dirty)
        {
            dirty = false;
            char line[80];
            snprintf(line, sizeof line, "%s%s_", out, "");
            for (int pass = 0; pass < 2; pass++)
            {
                DrawText ("                                                     ",0,y,0,0,0,0);
                DrawText ("                                                     ",0,y + 1,0,0,0,0);
                DrawText (prompt,0,y,0,0,255,0);
                DrawText (line,0,y + 1,0,0,252,0);
                DrawText (caps ? "CAPS LOCK ON " : "             ",40,y,0,0,224,0);
                vga->show();
            }
        }
        const uint8_t k = sf.DIRECT_Key_Code;
        if (k != prev && k != 0)
        {
            if (k == MENU_ENTER) { while (sf.DIRECT_Key_Code == MENU_ENTER) vTaskDelay(10); return true; }
            if (k == MENU_ESC)   { while (sf.DIRECT_Key_Code == MENU_ESC) vTaskDelay(10); return false; }
            if (k == 0x39) { caps = !caps; dirty = true; }
            else if (k == MENU_BACKSPACE) { if (len) out[--len] = 0; dirty = true; }
            else
            {
                const bool shift = (HID_ModifierByte & 0x22) != 0;
                char c = Note_HidToChar(k, shift);
                if (c && caps && c >= 'a' && c <= 'z') c = (char)(c - 32);
                else if (c && caps && c >= 'A' && c <= 'Z') c = (char)(c + 32);
                if (c && len < outsz - 1) { out[len++] = c; out[len] = 0; dirty = true; }
            }
        }
        prev = k;
        vTaskDelay(15);
    }
}

static void DrawNetMenu(const char *msg, uint8_t msg_color)
{
    char buf[64];
    for (int pass = 0; pass < 2; pass++)
    {
        vga->clear(0);
        DrawText ("Network (Wi-Fi):",0,0,0,0,255,0);
        line(0,10,319,10,3);
        Net_StatusLine(buf, sizeof buf);
        DrawText ("Status:",0,2,0,0,255,0);
        DrawText (buf,8,2,0,0,Net_Connected() ? 28 : 252,0);
        DrawText ("Network:",0,3,0,0,255,0);
        DrawText (Net_Ssid()[0] ? Net_Ssid() : "(none)",9,3,0,0,28,0);
        struct tm t;
        if (Net_LocalTime(&t)) snprintf(buf, sizeof buf, "%04d-%02d-%02d %02d:%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min);
        else snprintf(buf, sizeof buf, "unknown (needs Wi-Fi)");
        DrawText ("Time:",0,4,0,0,255,0);
        DrawText (buf,6,4,0,0,28,0);
        DrawText ("Zone:",0,5,0,0,255,0);
        DrawText (k_net_tz_label[Net_Tz() < NET_TZ_COUNT ? Net_Tz() : 0],6,5,0,0,28,0);

        DrawText ("1",0,7,0,0,255,0);
        DrawText (Net_Enabled() ? " Wi-Fi is ON: press to turn it OFF" : " Wi-Fi is OFF: press to turn it ON",1,7,0,0,252,0);
        DrawText ("S",0,8,0,0,255,0);
        DrawText ("earch the networks and connect",1,8,0,0,252,0);
        DrawText ("Z",0,9,0,0,255,0);
        DrawText ("one (time zone)",1,9,0,0,252,0);

        // DriveWire par le réseau
        DrawText ("DriveWire server on a PC (DriveWire 4, pyDriveWire):",0,11,0,0,255,0);
        NetDw_StatusLine(buf, sizeof buf);
        DrawText ("Status:",0,12,0,0,255,0);
        DrawText (buf,8,12,0,0,NetDw_Linked() ? 28 : (NetDw_Enabled() ? 252 : 224),0);
        DrawText ("Server:",0,13,0,0,255,0);
        DrawText (NetDw_Server()[0] ? NetDw_Server() : "(none)",8,13,0,0,28,0);
        DrawText ("2",0,14,0,0,255,0);
        DrawText (NetDw_Enabled() ? " DriveWire by Wi-Fi is ON: press to turn it OFF" : " DriveWire by Wi-Fi is OFF: press to turn it ON",1,14,0,0,252,0);
        DrawText ("D",0,15,0,0,255,0);
        DrawText (" server address (IP or name, :port if not 65504)",1,15,0,0,252,0);
        DrawText (g_bb_mode == BB_NETDW ? "Used by the bit banger (menu B, mode 5)." : "When ON, the Becker port ($FF41) goes to the PC.",0,16,0,0,g_bb_mode == BB_NETDW ? 224 : 28,0);

        DrawText ("Wi-Fi is OFF by default. When ON, it connects by",0,18,0,0,28,0);
        DrawText ("itself at power-up and sets the clock. Name and",0,19,0,0,28,0);
        DrawText ("password are kept in the ESP32 flash, not on the",0,20,0,0,28,0);
        DrawText ("SD card. Keys: US keyboard layout.",0,21,0,0,28,0);
        if (msg) DrawText (msg,0,23,0,0,msg_color,0);
        DrawText ("ESC to go back.",0,27,0,0,255,0);
        vga->show();
    }
}

static void NetScanAndConnect(void)
{
    static NetAp aps[16];
    for (int pass = 0; pass < 2; pass++)
    {
        vga->clear(0);
        DrawText ("Searching Wi-Fi networks...",0,0,0,0,255,0);
        vga->show();
    }
    const int n = Net_Scan(aps, 16);
    int sel = 0;
    while (1)
    {
        for (int pass = 0; pass < 2; pass++)
        {
            vga->clear(0);
            DrawText ("Choose the network (arrows + ENTER):",0,0,0,0,255,0);
            line(0,10,319,10,3);
            if (n <= 0) DrawText ("No network found.",0,2,0,0,224,0);
            for (int i = 0; i < n; i++)
            {
                char b[64];
                snprintf(b, sizeof b, "%c %-32s %4d dBm%s", i == sel ? '>' : ' ', aps[i].ssid, (int)aps[i].rssi, aps[i].open ? " open" : "");
                DrawText (b,0,(uint8_t)(2 + i),0,0,i == sel ? 255 : 28,i == sel ? 3 : 0);
            }
            DrawText ("M  type a hidden network name",0,20,0,0,252,0);
            DrawText ("ESC to go back.",0,27,0,0,255,0);
            vga->show();
        }
        while (sf.DIRECT_Key_Code != 0) vTaskDelay(10);
        uint8_t k = 0;
        while ((k = sf.DIRECT_Key_Code) == 0) vTaskDelay(10);
        if (k == MENU_ESC) { while (sf.DIRECT_Key_Code == MENU_ESC) vTaskDelay(10); return; }
        if (k == MENU_UP && sel > 0) sel--;
        else if (k == MENU_DOWN && sel < n - 1) sel++;
        else if (k == MENU_ENTER || k == MENU_M)
        {
            char ssid[33] = "", pass[65] = "";
            bool open = false;
            if (k == MENU_ENTER)
            {
                if (n <= 0) continue;
                strncpy(ssid, aps[sel].ssid, sizeof ssid - 1);
                open = aps[sel].open;
            }
            else
            {
                while (sf.DIRECT_Key_Code == MENU_M) vTaskDelay(10);
                if (!Menu_TextInput("Network name, then ENTER:", ssid, sizeof ssid, 22) || !ssid[0]) continue;
            }
            while (sf.DIRECT_Key_Code == MENU_ENTER) vTaskDelay(10);
            if (!open)
            {
                char q[64];
                snprintf(q, sizeof q, "Password for %.24s, then ENTER:", ssid);
                if (!Menu_TextInput(q, pass, sizeof pass, 24)) continue;
            }
            Net_SetCredentials(ssid, pass);
            return;
        }
    }
}

static void NetMenuChoose(void)
{
    DrawNetMenu(nullptr, 0);
    Menu_KeySync();          // touches au front d'appui (comme les autres menus 7m10)
    uint32_t last = millis();
    char shown[128] = "";
    while (1)
    {
        const uint8_t k = Menu_GetKey();
        switch (k)
        {
        case MENU_1:
            Net_SetEnabled(!Net_Enabled());
            DrawNetMenu(Net_Enabled() ? "Wi-Fi turned ON." : "Wi-Fi turned OFF.", 28);
            while (sf.DIRECT_Key_Code == MENU_1) vTaskDelay(10);
            break;
        case MENU_S:
            while (sf.DIRECT_Key_Code == MENU_S) vTaskDelay(10);
            NetScanAndConnect();
            Menu_KeySync();
            DrawNetMenu(Net_Enabled() && Net_Ssid()[0] ? "Connecting... (status updates every second)" : nullptr, 252);
            break;
        case MENU_2:
            NetDw_SetEnabled(!NetDw_Enabled());
            DrawNetMenu(NetDw_Enabled() ? (Net_Enabled() ? "DriveWire by Wi-Fi turned ON." : "DriveWire ON: turn Wi-Fi ON too (1).") : "DriveWire by Wi-Fi turned OFF.", 28);
            break;
        case MENU_D:
        {
            char hp[64];
            strncpy(hp, NetDw_Server(), sizeof hp - 1); hp[sizeof hp - 1] = 0;
            DrawNetMenu(nullptr, 0);
            if (Menu_TextInput("Server IP or name (:port if not 65504):", hp, sizeof hp, 24))
            {
                NetDw_SetServer(hp);
                DrawNetMenu("Server saved.", 28);
            }
            else DrawNetMenu(nullptr, 0);
            Menu_KeySync();
            break;
        }
        case MENU_Z:
            Net_SetTz((uint8_t)((Net_Tz() + 1) % NET_TZ_COUNT));
            DrawNetMenu(nullptr, 0);
            while (sf.DIRECT_Key_Code == MENU_Z) vTaskDelay(10);
            break;
        case MENU_ESC:
            return;
        default:
            break;
        }
        if (millis() - last > 1000)                   // état et heure (minute) à jour : redessin seulement s'ils changent
        {
            last = millis();
            char now[128], st[48];
            struct tm t;
            char dws[48];
            Net_StatusLine(st, sizeof st);
            NetDw_StatusLine(dws, sizeof dws);
            char ws[48]; Web_StatusLine(ws, sizeof ws);
            snprintf(now, sizeof now, "%s|%s|%s|%d", st, dws, ws, Net_LocalTime(&t) ? t.tm_hour * 60 + t.tm_min : -1);
            if (strcmp(now, shown) != 0) { if (shown[0]) DrawNetMenu(nullptr, 0); strcpy(shown, now); }
        }
        vTaskDelay(5);
    }
}

// =====================================================================
// AIDE (menu F12 -> H) : touches et options, en 2 pages
// =====================================================================
static void DrawHelpPage(int page)
{
    static const char *const k_p1[] = {
        "Keys while the CoCo is running:",
        "",
        "F12          Settings menu      F10  This help",
        "Esc          CoCo BREAK",
        "Delete       CoCo CLEAR",
        "Backspace    CoCo left arrow (erase)",
        "Pause or `   CoCo SHIFT+@ (pause a listing)",
        "F1 / F2      CoCo 3 F1 / F2 keys",
        "Alt / Ctrl   CoCo 3 ALT / CTRL keys",
        "Caps Lock    Upper / lower case (option L)",
        "Insert       Hold for turbo (maximum speed)",
        "F5 / F8      Save / load state (DATA STORAGE)",
        "",
        "For testers (files go to /ESP-DEBUG on the SD):",
        "F9           Save a log + a screenshot",
        "Shift+F9     Same, with a short note you type",
        "Print Screen Screenshot only (BMP)",
        "Ctrl+Alt+Del Reboot the ESP32",
        "",
        "Please send the LOGnnnn.TXT and .BMP files",
        "with every problem you report. Thank you!",
        nullptr };
    static const char *const k_p2[] = {
        "Settings menu (F12): shortcut keys",
        "",
        "D  DATA STORAGE:",
        "   Drives 0-3, DriveWire /X0 /X1, VHD 0 / 1,",
        "   Boot NitrOS-9, Save / load state, Cassette",
        "H  HARDWARE:",
        "   CPU, Expansion RAM, GIME version, Overclock",
        "M  MONITOR:",
        "   CoCo 3 monitor, Artifact colors, Audio output",
        "C  CART SLOT & COM:",
        "   Multi-Pak, cartridges (ROM, Orchestra-90, Game",
        "   Master, CoCo PSG, RS-232 Pak), Bit banger port",
        "U  USB DEVICES:",
        "   Keyboard, Joy (R) / (L), Mouse, gamepad wizard",
        "N  NETWORK:",
        "   Wi-Fi, Time zone, DriveWire server, Web page",
        "S  SYSTEM:",
        "   Boot screen, Screen saver, AUTORUN, Hardware",
        "   health, Firmware update, Previous version,",
        "   Reboot, Help, Tester log",
        "",
        "Tips:",
        "- SD card: FAT32, 4-32 GB. Keep /ESP-CONFIG.",
        "- 8BitDo gamepad: power it on with Power + B.",
        "- Cartridge ROMs: .ROM / .CCC files.",
        nullptr };
    const char *const *t = (page == 0) ? k_p1 : k_p2;
    for (int pass = 0; pass < 2; pass++)
    {
        vga->clear(0);
        char hdr[40];
        snprintf(hdr, sizeof hdr, "Help - page %d/2", page + 1);
        DrawText (hdr,0,0,0,0,255,0);
        DrawText ("Firmware v" FW_VERSION_SHORT,30,0,0,0,28,0);
        line(0,10,319,10,3);
        for (int i = 0; t[i]; i++)
        {
            const bool title = (i == 0) || (t[i][0] && t[i][strlen(t[i]) - 1] == ':');
            DrawText (t[i],0,2 + i,0,0,title ? 255 : 252,0);
        }
        DrawText ("Page Up / Page Down: other page  ESC: back",0,27,0,0,28,0);
        vga->show();
    }
}

// F10 pendant l'émulation : l'aide seule (sans passer par le menu F12)
void EMULATOR_Help(void)
{
    BackDisplay();
    HelpMenuChoose();
    RestoreDisplay();
}

void HelpMenuChoose(void)
{
    int page = 0;
    DrawHelpPage(page);
    Menu_KeySync();
    while (sf.DIRECT_Key_Code == 0x43 || sf.DIRECT_Key_Code == MENU_H) vTaskDelay(10);   // relâcher F10 / H
    while (1)
    {
        const uint8_t k = Menu_GetKey();
        if (k == MENU_ESC || k == 0x43)   // Échap ou F10
        {
            while (sf.DIRECT_Key_Code == k) vTaskDelay(10);   // relâcher (sinon F10 rouvrirait l'aide)
            return;
        }
        if (k == MENU_PGDWN || k == MENU_PGUP || k == MENU_ENTER || k == MENU_DOWN || k == MENU_UP)
        {
            page ^= 1;
            DrawHelpPage(page);
        }
        vTaskDelay(2);
    }
}

void DiskMenuChoose(void)
{

    DrawMenuDiskChoose();

    while(1)
    {

        switch (Menu_GetKey())
        {
        case MENU_0:
            DiskMenuChoose_1(0);
            DrawMenuDiskChoose();
            Menu_KeySync();
            break;
        case MENU_1:
            DiskMenuChoose_1(1);
            DrawMenuDiskChoose();
            Menu_KeySync();
        break;
        case MENU_2:
            DiskMenuChoose_1(2);
            DrawMenuDiskChoose();
            Menu_KeySync();
            break;
        case MENU_3:
            DiskMenuChoose_1(3);
            DrawMenuDiskChoose();
            Menu_KeySync();
            break;
        case MENU_4:            // DriveWire /x0
            DiskMenuChoose_1(4);
            DrawMenuDiskChoose();
            Menu_KeySync();
            break;
        case MENU_5:            // DriveWire /x1
            DiskMenuChoose_1(5);
            DrawMenuDiskChoose();
            Menu_KeySync();
            break;
        case MENU_6:            // VHD 0
            DiskMenuChoose_1(6);
            DrawMenuDiskChoose();
            Menu_KeySync();
            break;
        case MENU_7:            // VHD 1
            DiskMenuChoose_1(7);
            DrawMenuDiskChoose();
            Menu_KeySync();
            break;
        case MENU_U:            // démonter /x0, /x1, VHD 0 et VHD 1
            DW_Unmount(0);
            DW_Unmount(1);
            DW_Unmount(4);
            DW_Unmount(5);
            DrawMenuDiskChoose();
            break;
        case MENU_B:            // amorcer NitrOS-9 depuis /x0 (au retour dans l'émulation)
            if (g_dw_name[0][0])
            {
                g_dw_boot_request = true;
                s_menu_exit_all = true;
                vTaskDelay(200);
                return;
            }
            DrawText ("No disk in /x0 !",0,27,0,0,0b11100000,0);
            vga->show();
            break;
        case MENU_ESC:
            return;
            break;
        default:
            break;
        }
        vTaskDelay(2);
    }

    return;
}

void DrawMenuDiskChoose(void)
{
    vga->clear(0);
    vga->show();
    vga->clear(0);
    vga->show();
    DrawText ("Disk drive menu:",0,0,0,0,255,0);
    line(0,10,319,10,3);
    DrawText ("Disk drive 0:",0,2,0,0,255,0);
    DrawText ((char*)Disk_Drive.Name_Disk[0] + 1,1,3,0,0,0b11100000,0);

    DrawText ("Disk drive 1:",0,5,0,0,255,0);
    DrawText ((char*)Disk_Drive.Name_Disk[1] + 1,1,6,0,0,0b11100000,0);

    DrawText ("Disk drive 2:",0,8,0,0,255,0);
    DrawText ((char*)Disk_Drive.Name_Disk[2] + 1,1,9,0,0,0b11100000,0);

    DrawText ("Disk drive 3:",0,11,0,0,255,0);
    DrawText ((char*)Disk_Drive.Name_Disk[3] + 1,1,12,0,0,0b11100000,0);

    // disques virtuels DriveWire (port Becker, serveur dans l'ESP32)
    DrawText ("DriveWire /x0 (4):",0,14,0,0,255,0);
    DrawText (g_dw_name[0][0] ? g_dw_name[0] + 1 : "(none)",1,15,0,0,0b00011100,0);
    DrawText ("DriveWire /x1 (5):",0,16,0,0,255,0);
    DrawText (g_dw_name[1][0] ? g_dw_name[1] + 1 : "(none)",1,17,0,0,0b00011100,0);
    // disques durs virtuels (interface VHD $FF80, pilote EmuDsk)
    DrawText ("Hard disk VHD 0 (6):",0,18,0,0,255,0);
    DrawText (g_dw_name[4][0] ? g_dw_name[4] + 1 : "(none)",1,19,0,0,0b11111100,0);
    DrawText ("Hard disk VHD 1 (7):",0,20,0,0,255,0);
    DrawText (g_dw_name[5][0] ? g_dw_name[5] + 1 : "(none)",1,21,0,0,0b11111100,0);

    DrawText ("0-3: floppy  4-5: DriveWire  6-7: VHD",0,23,0,0,255,0);
    DrawText ("U: unmount 4-7   B: boot NitrOS-9 from /x0",0,24,0,0,255,0);
    DrawText ("ESC to exit menu.",0,25,0,0,255,0);
    if (!g_ff40_fdc)              // pas de contrôleur de disquette dans le slot actif
        DrawText ("No disk controller in the active slot (F12 S).",0,27,0,0,0b11100000,0);

    vga->show();

}

// Ajoute un sous-dossier au chemin courant (ex: "/" + "GAMES" -> "/GAMES" ou "/GAMES" + "SUB" ->
// "/GAMES/SUB").
static void DiskNav_Descend(char *path, size_t pathSize, const char *dirName)
{
    if (strcmp(path, "/") == 0)
    {
        snprintf(path, pathSize, "/%s", dirName);
    }
    else
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp), "%s/%s", path, dirName);
        strncpy(path, tmp, pathSize - 1);
        path[pathSize - 1] = '\0';
    }
}

// Retire le dernier segment du chemin courant (remonte d'un niveau). Ne fait rien si déjà à root.
static void DiskNav_GoUp(char *path)
{
    if (strcmp(path, "/") == 0) return;
    char *lastSlash = strrchr(path, '/');
    if (lastSlash == nullptr) { strcpy(path, "/"); return; }
    if (lastSlash == path)
    {
        path[1] = '\0'; // ex: "/GAMES" -> "/"
    }
    else
    {
        *lastSlash = '\0'; // ex: "/GAMES/SUB" -> "/GAMES"
    }
}

void DrawDiskMenuChoose_1(const char *currentPath)
{
    DrawText (s_browse_tape ? "Select the tape (.CAS / .WAV):" : s_browse_cart ? (s_cart_newmode == SLOT_O90 ? "Select the Orchestra-90 ROM (.ROM / .CCC):" : (s_cart_newmode == SLOT_GMC ? "Select the Game Master ROM (.ROM / .CCC):" : (s_cart_newmode == SLOT_CPSG ? "Select the CoCo PSG game (.ROM / .CCC):" : "Select the cartridge ROM (.ROM / .CCC):"))) : "Select the file to assign to disk drive:",0,0,0,0,255,0);
    // X=13 : évite la colonne des noms de fichiers (X=0-11, remplie par DrawFiles) sur les lignes 1 à
    // 13.
    DrawText (s_browse_pc ? "PC:" : "Path:",13,2,0,0,s_browse_pc ? 0b11111100 : 255,0);
    DrawText (currentPath,19,2,0,0,0b00011100,0);
    if (s_browse_pc && NetShare_ServerName()[0]) { char b[40]; snprintf(b, sizeof b, "(%.18s)", NetShare_ServerName()); DrawText (b,13,1,0,0,0b11111100,0); }
    if (s_browse_cart || s_browse_drive < 4) DrawText (s_browse_pc ? "P SD CARD" : "P PC SHARE",32,11,0,0,0b11111100,0);   // cartouches aussi
    // X=40, lignes 5-11 seulement : après (ligne 12+) c'est la zone du cadre/aperçu du contenu du
    // disque (DisplayDiskContent) Largeur dispo ~13 caractères (jusqu'à la bordure à 319px).
    DrawText ("Navigation:",32,3,0,0,255,0);
    DrawText ("Enter OPEN",32,4,0,0,255,0);
    DrawText ("Arrow DOWN",32,5,0,0,255,0);
    DrawText ("Arrow UP",32,6,0,0,255,0);
    DrawText ("Page DOWN",32,7,0,0,255,0);
    DrawText ("Page UP",32,8,0,0,255,0);
    DrawText ("Backspace BACK",32,9,0,0,255,0);
    DrawText ("Home ROOT",32,10,0,0,255,0);
    return;
}

void DrawFirmwareUpdateMenuChoose(void)
{
    DrawText ("Select firmware to flash for system upgrade:",0,0,0,0,255,0);
    DrawText ("Navigation:",40,5,0,0,255,0);
    DrawText ("Arrow DOWN",40,7,0,0,255,0);
    DrawText ("Arrow UP",40,8,0,0,255,0);
    // version précédente (autre partition) et dossier FLASHNOW
    {
        char v[40], b[64];
        if (Fw_OtherVersion(v, sizeof v))
        {
            snprintf(b, sizeof b, "P = restart the previous firmware %s", v);
            DrawText (b,0,22,0,0,252,0);
        }
        DrawText ("Emergency:  put working .FLH or .BIN file in SD card",0,24,0,0,28,0);
        DrawText ("FLASHNOW folder; it will be flashed at next power-up.",0,25,0,0,28,0);
    }
    return;
}

// passage au dossier partagé du PC : trouve le serveur, demande le PIN au besoin.
bool Disk_MountNet(uint8_t drv, const char *path, const uint8_t *data, uint32_t size);   // main.cpp
static int Pc_Open(void)
{
    Menu_Toast("Looking for the PC share (coco_share.py)...", "");
    int r = NetShare_Connect();
    while (r == NS_PIN)
    {
        char pin[8] = "";
        Menu_KeySync();
        if (!Menu_TextInput("PIN shown by coco_share.py on the PC:", pin, sizeof pin, 24) || !pin[0]) return NS_PIN;
        NetShare_SetPin(pin);
        Menu_Toast("Checking the PIN...", "");
        r = NetShare_Connect();
    }
    Menu_KeySync();
    return r;
}

void DiskMenuChoose_1(uint8_t DriveNumber)
{
    #define DELAY_MENU_SELECT 80
    int16_t FileStart = 0;
    int16_t MenuPick = 0;

    int8_t MenuMaxIndex;
    char buf[15];

    // les disques durs virtuels (6, 7) sont des fichiers .VHD DriveNumber 8 = cartouche du port
    // d'extension (.ROM / .CCC)
    s_browse_cart = (DriveNumber == 8);
    s_browse_tape = (DriveNumber == 9);   // cassette
    if (s_browse_tape) ClearFileList();   // pas d'aperçu d'un disque vu avant
    const char *fileExt = s_browse_tape ? ".CAS|.WAV" : (s_browse_cart ? ".ROM|.CCC" : ((DriveNumber >= 6) ? ".VHD" : ".DSK"));
    // le navigateur s'ouvre dans le dossier du dernier disque monté dans CE lecteur (chemin déjà
    // sauvegardé dans config.ccc), avec ce disque présélectionné
    const char *lastMounted = s_browse_tape ? Cas_Name() : s_browse_cart ? (const char *)g_slotcfg.name[s_cart_slot] :
                              ((DriveNumber < 4) ? (const char *)Disk_Drive.Name_Disk[DriveNumber]
                                                 : g_dw_name[DriveNumber < 6 ? DriveNumber - 4 : DriveNumber - 2]);
    const char *lastName = nullptr;
    s_browse_drive = DriveNumber;
    s_browse_pc = false;
    s_pc_listed[0] = 0;
    // dernier disque venu du PC (« /PC:/dossier/x.dsk ») : on rouvre le PC
    if (!s_browse_cart && DriveNumber < 4 && lastMounted && strncmp(lastMounted, "/PC:", 4) == 0)
    {
        vga->clear(0);
        if (Pc_Open() == NS_OK) { s_browse_pc = true; lastMounted += 4; }
        else lastMounted = nullptr;
    }
    DiskPath_GoToRoot(CurrentDiskPath, sizeof(CurrentDiskPath));
    if (lastMounted && lastMounted[0] == '/')
    {
        const char *slash = strrchr(lastMounted, '/');
        lastName = slash + 1;
        const size_t dirLen = (size_t)(slash - lastMounted);
        if (dirLen > 0 && dirLen < sizeof(CurrentDiskPath))
        {
            memcpy(CurrentDiskPath, lastMounted, dirLen);
            CurrentDiskPath[dirLen] = '\0';
        }
    }

    vga->clear(0);
    vga->show();
    vga->clear(0);
    vga->show();

    MenuMaxIndex =  FillFileBuffer(MenuPick, 10,fileExt, CurrentDiskPath);
    if (MenuMaxIndex == -1 && strcmp(CurrentDiskPath, "/") != 0)   // dossier disparu -> racine
    {
        DiskPath_GoToRoot(CurrentDiskPath, sizeof(CurrentDiskPath));
        lastName = nullptr;
        MenuMaxIndex = FillFileBuffer(MenuPick, 10, fileExt, CurrentDiskPath);
    }
    // Présélection du dernier disque : on parcourt les pages (11 entrées).
    if (lastName && lastName[0] && MenuMaxIndex >= 0)
    {
        bool found = false;
        for (int page = 0; page < 200 && !found; page++)
        {
            for (int i = 0; i <= MenuMaxIndex; i++)
            {
                if (strcasecmp((const char *)FileArray.FileBuffer[i], lastName) == 0) { MenuPick = i; found = true; break; }
            }
            if (found) break;
            const int8_t next = FillFileBuffer(FileStart + 11, 10, fileExt, CurrentDiskPath);
            if (next == -1) break;
            FileStart += 11;
            MenuMaxIndex = next;
        }
        if (!found)   // pas trouvé : retour à la 1re page
        {
            FileStart = 0;
            MenuPick = 0;
            MenuMaxIndex = FillFileBuffer(0, 10, fileExt, CurrentDiskPath);
        }
    }

    DrawDiskMenuChoose_1(CurrentDiskPath);
    DrawFiles(MenuPick,0b11100000, 0b00000011);
    DisplayDiskContent();
    DrawFrames();

    vga->show();
    Menu_KeySync();          // touche du lecteur encore tenue
    while(1)
    {

        switch (Menu_GetKey())
        {
        case MENU_DOWN:
            MenuPick++;

            if (MenuPick > MenuMaxIndex)
            {
                // fin de liste : la page affichée reste valide. MenuMaxIndex restait à -1 : flèche bas
                // et Entrée sans effet
                const int8_t next = FillFileBuffer(FileStart + 11, 10, fileExt, CurrentDiskPath);
                if (next == -1)
                {
                    MenuPick--;
                }
                else
                {
                    FileStart += 11;
                    MenuMaxIndex = next;
                    MenuPick = 0;
                }
            }
            vga->clear(0);

            DrawDiskMenuChoose_1(CurrentDiskPath);

            DrawFiles(MenuPick,0b11100000, 0b00000011);
            DisplayDiskContent();
            DrawFrames();
            vga->show();
            break;
        case MENU_PGDWN:

            for (uint8_t loop1 = 0; loop1 !=11; loop1++)
            {
                MenuPick++;

                if (MenuPick > MenuMaxIndex)
                {
                    const int8_t next = FillFileBuffer(FileStart + 11, 10, fileExt, CurrentDiskPath);
                    if (next == -1)
                    {
                        MenuPick--;
                        break;
                    }
                    FileStart += 11;
                    MenuMaxIndex = next;
                    MenuPick = 0;
                }
            }
            vga->clear(0);

            DrawDiskMenuChoose_1(CurrentDiskPath);

            DrawFiles(MenuPick,0b11100000, 0b00000011);
            DisplayDiskContent();
            DrawFrames();
            vga->show();
            break;
        case MENU_UP:

            MenuPick--;

            if (MenuPick < 0 && FileStart < 0)
            {
                MenuPick = 0;
                FileStart = 0;
            }
            else if (MenuPick < 0 && FileStart >= 0)
            {
                MenuPick+=11;
                FileStart-=11;
                if (FileStart <0)
                {
                    FileStart = 0;
                    MenuPick = 0;
                }
                MenuMaxIndex =  FillFileBuffer(FileStart, 10,fileExt, CurrentDiskPath);
            }
            vga->clear(0);

            DrawDiskMenuChoose_1(CurrentDiskPath);
            DrawFiles(MenuPick,0b11100000, 0b00000011);
            DisplayDiskContent();
            DrawFrames();
            vga->show();
            break;
        case MENU_PGUP:
            for (uint8_t loop1 = 0; loop1 !=11; loop1++)
            {
                MenuPick--;

                if (MenuPick < 0 && FileStart < 0)
                {
                    MenuPick = 0;
                    FileStart = 0;
                }
                else if (MenuPick < 0 && FileStart >= 0)
                {
                    MenuPick+=11;
                    FileStart-=11;
                    if (FileStart <0)
                    {
                        FileStart = 0;
                        MenuPick = 0;
                    }
                    MenuMaxIndex =  FillFileBuffer(FileStart, 10,fileExt, CurrentDiskPath);
                }
            }
                vga->clear(0);
                DrawDiskMenuChoose_1(CurrentDiskPath);
                DrawFiles(MenuPick,0b11100000, 0b00000011);
                DisplayDiskContent();
                DrawFrames();
                vga->show();
                break;
        case MENU_ENTER:
            // Un dossier est préfixé par "/" dans FileArray.FileBuffer[] (voir FillFileBuffer) --
            // distingue "descendre dedans" de "assigner ce fichier .DSK au lecteur". dossier vide
            // (aucune entrée)
            if (MenuMaxIndex < 0 || MenuPick > MenuMaxIndex || FileArray.FileBuffer[MenuPick][0] == 0)
            {
                break;
            }
            if (FileArray.FileBuffer[MenuPick][0] == '/')
            {
                DiskPath_Descend(CurrentDiskPath, sizeof(CurrentDiskPath),
                                  (const char*)&FileArray.FileBuffer[MenuPick][1]);
                FileStart = 0;
                MenuPick = 0;
                ClearFileBuffer();
                ClearFileList();  // efface l'aperçu du contenu du disque (sinon il affichait l'ancien…)
                MenuMaxIndex = FillFileBuffer(FileStart, 10, fileExt, CurrentDiskPath);
                vga->clear(0);
                DrawDiskMenuChoose_1(CurrentDiskPath);
                DrawFiles(MenuPick,0b11100000, 0b00000011);
                DisplayDiskContent();
                DrawFrames();
                vga->show();
            }
            else
            {
                // Fichier .DSK -> chemin complet = dossier courant + nom.
                char fullPath[256];
                if (strcmp(CurrentDiskPath, "/") == 0)
                    snprintf(fullPath, sizeof(fullPath), "/%s", (char*)FileArray.FileBuffer[MenuPick]);
                else
                    snprintf(fullPath, sizeof(fullPath), "%s/%s", CurrentDiskPath, (char*)FileArray.FileBuffer[MenuPick]);

                if (s_browse_tape)          // bande choisie : en lecture, au début
                {
                    s_browse_tape = false;
                    if (!Cas_Insert(fullPath)) Menu_Toast("Cannot read this tape (.CAS or PCM .WAV).", "");
                    else Serial.printf("[CAS] bande en lecture : %s\n", fullPath);
                    return;
                }
                if (s_browse_cart)          // cartouche choisie (insérée au redémarrage)
                {
                    if (s_browse_pc)        // cartouche du PC : copiée sur la SD (chargée au démarrage, avant le…)
                    {
                        Menu_Toast("Copying the cartridge from the PC...", "");
                        uint8_t *img = nullptr; uint32_t sz = 0;
                        const int r = NetShare_Get(fullPath, &img, &sz);
                        char sdPath[160];
                        const char *bn = strrchr(fullPath, '/');
                        snprintf(sdPath, sizeof sdPath, "/PC-ROMS/%s", bn ? bn + 1 : fullPath);
                        extern bool Sd_WriteFile(const char *path, const uint8_t *data, uint32_t size);
                        const bool ok = r == NS_OK && img && sz && Sd_WriteFile(sdPath, img, sz);
                        if (img) free(img);
                        if (!ok)
                        {
                            Menu_Toast(r == NS_OK ? "Could not copy it to the SD card." : NetShare_ErrorText(r), "");
                            break;
                        }
                        Slot_InstallRom(s_cart_slot, s_cart_newmode, sdPath);
                        s_browse_pc = false;
                        s_browse_cart = false;
                        return;
                    }
                    Slot_InstallRom(s_cart_slot, s_cart_newmode, fullPath);   // aussi retenu pour ce type
                    s_browse_cart = false;
                    return;
                }
                if (DriveNumber >= 4)       // disque virtuel DriveWire
                {
                    DW_Mount(DriveNumber < 6 ? DriveNumber - 4 : DriveNumber - 2, fullPath);   // 4,5 -> /x0,/x1 ; 6,7 -> VHD 0,1
                    return;
                }
                if (s_browse_pc)            // disquette du PC copiée en PSRAM
                {
                    Menu_Toast("Loading the disk from the PC...", "");
                    const uint32_t t0 = millis();
                    uint8_t *img = nullptr; uint32_t sz = 0;
                    int r = NetShare_Get(fullPath, &img, &sz);
                    bool ok = false;
                    if (r == NS_OK && img) { ok = Disk_MountNet(DriveNumber, fullPath, img, sz); free(img); }
                    if (!ok)
                    {
                        Menu_Toast(r == NS_OK ? "Disk image too large or invalid." : NetShare_ErrorText(r), "");
                        break;
                    }
                    snprintf((char*)Disk_Drive.Name_Disk[DriveNumber], sizeof(Disk_Drive.Name_Disk[DriveNumber]), "/PC:%s", fullPath);
                    Serial.printf("[MENU] lecteur %u monte du PC en %lu ms : %s\n", DriveNumber, (unsigned long)(millis() - t0), fullPath);
                    s_browse_pc = false;
                    return;
                }
                snprintf((char*)Disk_Drive.Name_Disk[DriveNumber], sizeof(Disk_Drive.Name_Disk[DriveNumber]), "%s", fullPath);
                const uint32_t t0 = millis();
                ReadCoCoFile((const char*)Disk_Drive.Name_Disk[DriveNumber], DriveNumber);        // Load into RAMDISK
                Serial.printf("[MENU] lecteur %u monte en %lu ms : %s\n", DriveNumber,
                              (unsigned long)(millis() - t0), fullPath);
                return;
            }
            break;

        case MENU_BACKSPACE:
        {
            // au retour dans le dossier parent, le dossier qu'on quitte est présélectionné
            char child[130] = "";
            if (strcmp(CurrentDiskPath, "/") != 0)
            {
                const char *slash = strrchr(CurrentDiskPath, '/');
                snprintf(child, sizeof(child), "/%s", slash ? slash + 1 : CurrentDiskPath);
            }
            DiskPath_GoUp(CurrentDiskPath, sizeof(CurrentDiskPath));
            FileStart = 0;
            MenuPick = 0;
            ClearFileBuffer();
            ClearFileList();  // efface l'aperçu du contenu du disque (sinon il affichait l'ancien…)
            MenuMaxIndex = FillFileBuffer(FileStart, 10, fileExt, CurrentDiskPath);
            bool found = false;
            for (int page = 0; child[0] && MenuMaxIndex >= 0 && page < 200 && !found; page++)
            {
                for (int i = 0; i <= MenuMaxIndex; i++)
                    if (strcasecmp((const char *)FileArray.FileBuffer[i], child) == 0) { MenuPick = i; found = true; break; }
                if (found) break;
                const int8_t next = FillFileBuffer(FileStart + 11, 10, fileExt, CurrentDiskPath);
                if (next == -1) break;
                FileStart += 11;
                MenuMaxIndex = next;
            }
            if (child[0] && !found)   // pas trouvé : première page
            {
                FileStart = 0;
                MenuPick = 0;
                MenuMaxIndex = FillFileBuffer(0, 10, fileExt, CurrentDiskPath);
            }
            vga->clear(0);
            DrawDiskMenuChoose_1(CurrentDiskPath);
            DrawFiles(MenuPick,0b11100000, 0b00000011);
            DisplayDiskContent();
            DrawFrames();
            vga->show();
            break;
        }

        case MENU_P:                 // dossier partagé du PC <-> carte SD
        {
            if (!s_browse_cart && DriveNumber >= 4) break;   // cartouches aussi (copiées sur la SD)
            int r = NS_OK;
            if (!s_browse_pc) r = Pc_Open();
            if (r == NS_OK) s_browse_pc = !s_browse_pc;
            s_pc_listed[0] = 0;
            s_preview_path[0] = 0;
            DiskPath_GoToRoot(CurrentDiskPath, sizeof(CurrentDiskPath));
            FileStart = 0;
            MenuPick = 0;
            ClearFileBuffer();
            ClearFileList();
            MenuMaxIndex = FillFileBuffer(FileStart, 10, fileExt, CurrentDiskPath);
            vga->clear(0);
            DrawDiskMenuChoose_1(CurrentDiskPath);
            DrawFiles(MenuPick,0b11100000, 0b00000011);
            DisplayDiskContent();
            DrawFrames();
            vga->show();
            if (r != NS_OK) Menu_Toast(NetShare_ErrorText(r), "");
            else if (s_browse_pc && MenuMaxIndex < 0) Menu_Toast(s_browse_cart ? "The PC folder is empty (no .ROM / .CCC file)." : "The PC folder is empty (no .DSK file).", "");
            break;
        }

        case MENU_HOME:
            DiskPath_GoToRoot(CurrentDiskPath, sizeof(CurrentDiskPath));
            FileStart = 0;
            MenuPick = 0;
            ClearFileBuffer();
            ClearFileList();  // efface l'aperçu du contenu du disque (sinon il affichait l'ancien…)
            MenuMaxIndex = FillFileBuffer(FileStart, 10, fileExt, CurrentDiskPath);
            vga->clear(0);
            DrawDiskMenuChoose_1(CurrentDiskPath);
            DrawFiles(MenuPick,0b11100000, 0b00000011);
            DisplayDiskContent();
            DrawFrames();
            vga->show();
            break;

        case MENU_ESC:
            s_browse_pc = false;     // les autres navigateurs (cartouches, firmware) restent sur la SD
            return;
            break;

        default:
            break;
        }
        vTaskDelay(10);

    }

}

char* Firmware_Choose(void)
{
    #define DELAY_MENU_SELECT 80
    int16_t FileStart = 0;
    int16_t MenuPick = 0;

    int8_t MenuMaxIndex;
    char buf[15];
    vga->clear(0);
    vga->show();
    vga->clear(0);
    vga->show();

    DrawFirmwareUpdateMenuChoose();
    MenuMaxIndex =  FillFileBuffer(MenuPick, 10,".FLH", "/");
    DrawFiles(MenuPick,0b11100000, 0b00000011);

    vga->show();
    vTaskDelay(200);
    while(1)
    {

        switch (sf.DIRECT_Key_Code)
        {
        case MENU_DOWN:
            MenuPick++;

            if (MenuPick > MenuMaxIndex)
            {
                FileStart +=11;
                MenuMaxIndex =  FillFileBuffer(FileStart, 10,".FLH", "/");
                if (MenuMaxIndex == -1) // Error
                {
                    MenuPick--;
                    FileStart-=11;
                }
                else
                {
                    MenuPick = 0;
                }
            }
            vga->clear(0);

            DrawFirmwareUpdateMenuChoose();

            DrawFiles(MenuPick,0b11100000, 0b00000011);

            vga->show();
            vTaskDelay(DELAY_MENU_SELECT);
            break;
        case MENU_UP:

            MenuPick--;

            if (MenuPick < 0 && FileStart < 0)
            {
                MenuPick = 0;
                FileStart = 0;
            }
            else if (MenuPick < 0 && FileStart >= 0)
            {
                MenuPick+=11;
                FileStart-=11;
                if (FileStart <0)
                {
                    FileStart = 0;
                    MenuPick = 0;
                }
                MenuMaxIndex =  FillFileBuffer(FileStart, 10,".FLH", "/");
            }
            vga->clear(0);

            DrawFirmwareUpdateMenuChoose();
            DrawFiles(MenuPick,0b11100000, 0b00000011);

            vga->show();
            vTaskDelay(DELAY_MENU_SELECT);
            break;
        case MENU_ENTER:

            FileArray.FileBuffer[15][0] = '/';
            strcpy((char*)&FileArray.FileBuffer[15][1], (const char*)FileArray.FileBuffer[MenuPick]);
            return (char*)FileArray.FileBuffer[15];
            break;

        case MENU_ESC:
            return (char*)"NF";  // No file selected.
            break;
        case MENU_P:             // version précédente
        {
            char v[40];
            if (Fw_OtherVersion(v, sizeof v)) return (char*)"PREV";
            break;
        }

        default:
            break;
        }
        vTaskDelay(10);

    }

}

void DisplayDiskContent(void)
{
    uint16_t LoopFile, LoopChar, Xpos, Ypos, XposHome, YposHome, Loop1;
    char buf[15];

    LoopFile = 0;
    LoopChar = 0;
    Xpos = 0;
    XposHome = 0;
    YposHome = 12;
    Ypos = YposHome;

    for(LoopFile = 0; LoopFile < 68 && FileArray.FileList[LoopFile][0] != 255; LoopFile++)
    {
        Loop1 = 0;
        for(LoopChar = 0; LoopChar !=11; LoopChar++)
        {
            buf[Loop1++] = FileArray.FileList[LoopFile][LoopChar];

            if (Loop1 == 8)
            {
                buf[Loop1++] = '.';
            }
        }
        buf[Loop1] = '\0';
        DrawText(buf, Xpos, Ypos, 3,3,0b11100000, 0);

        Ypos+=1;
        if (Ypos == YposHome + 17)
        {
            Xpos+=13;
            Ypos = YposHome;
        }

    }

}

void DrawFrames(void)
{
    #define FRAME_COL_1 0b00011100
    #define FRAME_OFFSET 2
    line(1,97,319,97, FRAME_COL_1);
    line(1,97,1,235,FRAME_COL_1);
    line(319,97,319,235,FRAME_COL_1);
    line(1,235,319,235,FRAME_COL_1);

   line(1,235,319,235, FRAME_COL_1);

    line(78,97,78,235,FRAME_COL_1);

    line(78+78,97,78+78,235,FRAME_COL_1);
    line(78+78+78,97,78+78+78,235,FRAME_COL_1);
}

uint8_t DrawFiles(uint16_t FileNumber, uint8_t ForeColor, uint8_t BackColor)
{
    uint16_t Loop1;
    uint16_t Ypos = 1;
    for(Loop1 = 0; FileArray.FileBuffer[Loop1][0] != 0; Loop1++)
    {
        if (FileNumber == Loop1)
        {
            DrawText((char*)FileArray.FileBuffer[Loop1],0, Ypos++,0,0,ForeColor, 255);
            if (FileArray.FileBuffer[Loop1][0] != '/' && !s_browse_cart && !s_browse_tape) // pas un dossier, une cartouche ni une bande
                PopulateDiskContent((char*)FileArray.FileBuffer[Loop1]);
            else if (FileArray.FileBuffer[Loop1][0] != '/' && s_browse_tape && !s_browse_pc)   // fichiers de la bande
                PopulateTapeContent((char*)FileArray.FileBuffer[Loop1]);
        }
        else
        {
            DrawText((char*)FileArray.FileBuffer[Loop1], 0,Ypos++,0,0,ForeColor, BackColor);
        }
        if (Loop1 == 12)
        {
            break;
        }
    }
    return Loop1;

}

// =====================================================================
// JOYSTICK WIZARD - GAMEPAD CONFIGURATION TOOL (F12 menu -> J)
// =====================================================================
// MACHINE STATE : ESC always goes back one step (eventually all the way back to F12 menu) 0 SLOT :
// choose Right or Left joystick 1 MODE : choose Digital or Analog (provision for future use) 2 DEVICE

#define JOYMENU_HOLD_MS 1000

// Generic multiple-choice picker (Up/Down + ENTER, ESC = back one step). Returns the chosen index, or
// -1 if ESC was pressed.
static int8_t JoyMenu_PickOption(const char *title, const char *options[], uint8_t count)
{
    int8_t sel = 0;
    Menu_KeySync();                                // l'ENTER qui a ouvert l'écran ne choisit plus rien
    while (1)
    {
        vga->clear(0);
        DrawText(title, 0, 0, 0, 0, 255, 0);
        line(0, 10, 319, 10, 3);

        for (uint8_t i = 0; i != count; i++)
        {
            DrawText(options[i], 2, 3 + i, 0, 0,
                     (i == sel) ? 0b00011100 : 0b11100000,
                     0);
        }
        DrawText("Up/Down: select   ENTER: confirm   ESC: back", 0, 25, 0, 0, 255, 0);
        vga->show();

        switch (Menu_GetKey())
        {
        case MENU_UP:
            if (sel > 0) sel--;
            break;
        case MENU_DOWN:
            if (sel < (int8_t)count - 1) sel++;
            break;
        case MENU_ENTER:
            Menu_KeySync();
            return sel;
        case MENU_ESC:
            Menu_KeySync();
            return -1;
        default:
            break;
        }
        vTaskDelay(20);
    }
}

// Waits for the user to press AND HOLD one button/axis (i.e. keep it in a non-idle state) continuously
// for JOYMENU_HOLD_MS milliseconds. Shows the live raw HID report with a progress readout

// capture plus robuste. Axe : l'octet retenu est celui qui s'écarte LE PLUS de sa valeur au repos (au
// moins 64), pas le dernier qui a bougé
static uint8_t  s_cap_stable[16];
static uint8_t  s_cap_stable_len = 0xFF;
static uint32_t s_cap_stable_ms = 0;
static bool JoyMenu_CaptureWithHold(uint8_t address, const char *instruction, uint8_t &outByte, uint8_t &outMask,
                                    bool axis = false, uint8_t exclude = 0xFF)
{
    s_cap_stable_len = 0xFF;                       // repos à reprendre pour chaque étape
    Menu_KeySync();
    JoyWizard_ResetChangeDetector(address);
    uint8_t idleData[16] = {0};
    bool haveIdle = false;
    bool holding = false;
    uint8_t holdByte = 0;
    uint32_t holdStartMs = 0;
    const uint8_t kMaxB = 14;                      // même plage que la détection côté main.cpp

    while (1)
    {
        vga->clear(0);
        DrawText("Gamepad Configuration", 0, 0, 0, 0, 255, 0);
        line(0, 10, 319, 10, 3);
        DrawText(instruction, 0, 3, 0, 0, 0b11100000, 0);
        DrawText(axis ? "Push the stick or D-pad ALL THE WAY and hold." : "Release first if already pressed/held.", 0, 4, 0, 0, 255, 0);
        DrawText("ESC: back", 0, 25, 0, 0, 255, 0);

        uint8_t data[16], len, changed;
        if (JoyWizard_GetSnapshot(address, data, &len, &changed))
        {
            if (!haveIdle)
            {
                // la position de repos n'est prise que quand la manette ne bouge plus depuis 0,5 s
                // (boutons et sticks relâchés)
                const uint8_t n = (len > kMaxB) ? kMaxB : len;
                bool same = s_cap_stable_len == n;
                for (uint8_t i = 0; same && i < n; i++) same = abs((int)data[i] - (int)s_cap_stable[i]) <= 4;
                if (!same) { memcpy(s_cap_stable, data, n); s_cap_stable_len = n; s_cap_stable_ms = millis(); }
                if (millis() - s_cap_stable_ms < 1500)   // 1,5 s (temps de lire et de se préparer)
                {
                    DrawText("Get ready: release all buttons and sticks...", 0, 9, 0, 0, 0b11111100, 0);
                    vga->show();
                    if (sf.DIRECT_Key_Code == MENU_ESC) { vTaskDelay(200); return false; }
                    vTaskDelay(20);
                    continue;
                }
                memcpy(idleData, data, len);
                haveIdle = true;
                JoyWizard_ResetChangeDetector(address);
            }
            const uint8_t showLen = (len > kMaxB) ? kMaxB : len;

            // octet candidat
            int best = -1, bestDev = 0;
            for (uint8_t i = 0; i < showLen; i++)
            {
                if (i == exclude) continue;
                const int dev = abs((int)data[i] - (int)idleData[i]);
                if (axis)
                {
                    if (dev >= 64 && dev > bestDev) { bestDev = dev; best = i; }
                }
                else if (data[i] != idleData[i])
                {
                    const bool analogIdle = idleData[i] >= 0x20 && idleData[i] <= 0xE0;   // octet d'axe au centre
                    if (analogIdle && dev <= 8) continue;                                  // bruit d'un stick
                    const int score = analogIdle ? dev : 256 + __builtin_popcount(data[i] ^ idleData[i]);
                    if (score > bestDev) { bestDev = score; best = i; }
                }
            }

            DrawText("data:", 0, 6, 0, 0, 255, 0);
            for (uint8_t i = 0; i != showLen; i++)
            {
                char hx[4];
                snprintf(hx, sizeof(hx), "%02X", data[i]);
                const bool highlight = holding ? (i == holdByte) : (i == best);
                DrawText(hx, 6 + i * 3, 6, 0, 0, highlight ? 0b00011100 : (i == exclude ? 0b00000011 : 0b11100000), 0);
            }

            bool stillChanged = false;
            if (holding)
            {
                const int dev = abs((int)data[holdByte] - (int)idleData[holdByte]);
                stillChanged = axis ? (dev >= 48) : (data[holdByte] != idleData[holdByte]);
                if (stillChanged && best >= 0 && best != holdByte && axis && bestDev > dev + 32)
                {
                    holdByte = (uint8_t)best;      // un autre octet bouge nettement plus : c'est lui l'axe
                    holdStartMs = millis();
                }
            }
            else if (best >= 0)
            {
                holding = true;
                holdByte = (uint8_t)best;
                holdStartMs = millis();
                stillChanged = true;
            }

            if (stillChanged)
            {
                uint32_t elapsed = millis() - holdStartMs;
                uint16_t pct = (elapsed >= JOYMENU_HOLD_MS) ? 100 : (uint16_t)((elapsed * 100) / JOYMENU_HOLD_MS);
                char pbuf[32];
                snprintf(pbuf, sizeof(pbuf), "Hold... %u%%  (byte %u)", pct, holdByte);
                DrawText(pbuf, 0, 9, 0, 0, 0b00011100, 0);

                if (elapsed >= JOYMENU_HOLD_MS)
                {
                    outByte = holdByte;
                    outMask = data[holdByte] ^ idleData[holdByte];
                    if (outMask == 0) outMask = 0xFF; // safety fallback, should not happen
                    vga->show();
                    vTaskDelay(200);
                    return true;
                }
            }
            else
            {
                if (holding) JoyWizard_ResetChangeDetector(address);
                holding = false;
                DrawText(axis ? "Not far enough - push all the way and hold" : "Released - press and hold again", 0, 9, 0, 0, 0b11100000, 0);
            }
        }

        vga->show();

        if (sf.DIRECT_Key_Code == MENU_ESC)
        {
            vTaskDelay(200);
            return false;
        }
        vTaskDelay(20);
    }
}

// Shows a short summary and waits for ENTER (confirm -> true) or ESC (back -> false).
static bool JoyMenu_ConfirmScreen(const char *title, const char *detailLine)
{
    Menu_KeySync();                                // une nouvelle pression sur ENTER est demandée
    for (int pass = 0; pass < 2; pass++)
    {
        vga->clear(0);
        DrawText(title, 0, 0, 0, 0, 255, 0);
        line(0, 10, 319, 10, 3);
        DrawText(detailLine, 0, 4, 0, 0, 0b00011100, 0);
        DrawText("Release the gamepad, then:", 0, 7, 0, 0, 255, 0);
        DrawText("ENTER: confirm and go on   ESC: do it again", 0, 25, 0, 0, 255, 0);
        vga->show();
    }
    while (1)
    {
        const uint8_t k = Menu_GetKey();
        if (k == MENU_ENTER) { Menu_KeySync(); return true; }
        if (k == MENU_ESC)   { Menu_KeySync(); return false; }
        vTaskDelay(20);
    }
}

// Transient message screen (no key wait -- avoids any risk of hanging if the key state doesn't cleanly
// drop back to zero between reads).
static void JoyMenu_ShowMessage(const char *line1, const char *line2, uint8_t color1, uint16_t delayMs)
{
    vga->clear(0);
    DrawText(line1, 0, 10, 0, 0, color1, 0);
    if (line2 != nullptr) DrawText(line2, 0, 12, 0, 0, 255, 0);
    vga->show();
    vTaskDelay(delayMs);
}

#define WSTEP_EXIT      -1
#define WSTEP_SLOT       0
#define WSTEP_MODE       1
#define WSTEP_DEVICE     2
#define WSTEP_FIRE1      3
#define WSTEP_FIRE1_OK   4
#define WSTEP_FIRE2      5
#define WSTEP_FIRE2_OK   6
#define WSTEP_AXISX      7
#define WSTEP_AXISX_OK   8
#define WSTEP_AXISY      9
#define WSTEP_AXISY_OK  10
#define WSTEP_SAVE      11

// assistant lancé depuis USB DEVICES pour un port et un appareil déjà choisis (pas d'écran de choix du
// port ni de l'appareil). Rend true si les réglages sont gardés.
static bool JoyWizard_Run(const UsbDevView *dev, int8_t devSlot);
void JoystickMenuChoose(void) { JoyWizard_Run(nullptr, 0); }
bool JoyWizard_ForDevice(uint8_t slot, const UsbDevView &d) { return JoyWizard_Run(&d, (int8_t)slot); }
static bool JoyWizard_Run(const UsbDevView *dev, int8_t devSlot)
{
    int8_t step = dev ? WSTEP_MODE : WSTEP_SLOT;

    int8_t slotSel = dev ? devSlot : 0;
    int8_t modeSel = 0;
    uint16_t vid = dev ? dev->vid : 0, pid = dev ? dev->pid : 0;
    uint8_t address = dev ? dev->key : 0;
    JoystickDeviceConfig cfg = {};
    char confirmDetail[40];
    bool savedOk = false;

    while (step != WSTEP_EXIT)
    {
        switch (step)
        {
        case WSTEP_SLOT:
        {
            char slotLabel[2][44];
            const char *opts[2];
            for (uint8_t s = 0; s != 2; s++)
            {
                const char *name = (s == JOY_SLOT_RIGHT) ? "RIGHT" : "LEFT";
                if (!JoystickCfg[s].configured)
                {
                    snprintf(slotLabel[s], sizeof(slotLabel[s]), "%s (not configured)", name);
                }
                else
                {
                    // Cherche si ce vid/pid est actuellement connecté, pour afficher son adresse USB
                    // live (utile pour vérifier que le bon device physique est bien reconnu).
                    bool connected = false;
                    uint8_t liveAddr = 0;
                    uint8_t devCount = JoyWizard_GetDeviceCount();
                    for (uint8_t i = 0; i != devCount; i++)
                    {
                        uint16_t v, p; uint8_t a;
                        const uint8_t want = (uint8_t)(JoystickCfg[s].mode >> 4);   // 0 = toute interface
                        if (JoyWizard_GetDeviceInfo(i, v, p, a) &&
                            v == JoystickCfg[s].vid && p == JoystickCfg[s].pid &&
                            (want == 0 || (uint8_t)(want - 1) == ((a & 0x80) ? (a & 3) : 0)))
                        {
                            connected = true;
                            liveAddr = a;
                            break;
                        }
                    }
                    if (connected)
                    {
                        if (liveAddr & 0x80)   // 2e manette d'un adaptateur
                            snprintf(slotLabel[s], sizeof(slotLabel[s]), "%s=hid:addr=%u pad %u vid=%04X pid=%04X",
                                     name, (liveAddr >> 2) & 0x1F, (liveAddr & 3) + 1, JoystickCfg[s].vid, JoystickCfg[s].pid);
                        else
                            snprintf(slotLabel[s], sizeof(slotLabel[s]), "%s=hid:addr=%u vid=%04X pid=%04X",
                                     name, liveAddr, JoystickCfg[s].vid, JoystickCfg[s].pid);
                    }
                    else if (JoystickCfg[s].mode >> 4)   // manette 1 / 2 d'un adaptateur
                        snprintf(slotLabel[s], sizeof(slotLabel[s]), "%s=%04X:%04X pad %u (not connected)",
                                 name, JoystickCfg[s].vid, JoystickCfg[s].pid, (unsigned)(JoystickCfg[s].mode >> 4));
                    else
                        snprintf(slotLabel[s], sizeof(slotLabel[s]), "%s=vid=%04X pid=%04X (not connected)",
                                 name, JoystickCfg[s].vid, JoystickCfg[s].pid);
                }
                opts[s] = slotLabel[s];
            }
            int8_t r = JoyMenu_PickOption("Joystick Config - Select Slot", opts, 2);
            if (r < 0) { step = WSTEP_EXIT; }
            else { slotSel = r; step = WSTEP_MODE; }
            break;
        }

        case WSTEP_MODE:
        {
            const char *opts[2] = { "Digital (D-Pad)", "Analog (stick / potentiometer)" };
            int8_t r = JoyMenu_PickOption("Joystick Config - Select Mode", opts, 2);
            if (r < 0) { step = dev ? WSTEP_EXIT : WSTEP_SLOT; }
            else if (dev)                                   // appareil déjà choisi
            {
                modeSel = r;
                cfg.vid = vid;
                cfg.pid = pid;
                cfg.mode = (modeSel == 0) ? JOY_MODE_DIGITAL : JOY_MODE_ANALOG;
                cfg.mode |= (uint8_t)((((address & 0x80) ? (address & 3) : 0) + 1) << 4);
                step = WSTEP_FIRE1;
            }
            else { modeSel = r; step = WSTEP_DEVICE; }
            break;
        }

        case WSTEP_DEVICE:
        {
            uint8_t devCount = JoyWizard_GetDeviceCount();
            if (devCount == 0)
            {
                JoyMenu_ShowMessage("No USB gamepad detected.",
                                     "Plug in a gamepad and try again (key J).",
                                     0b11100000, 2000);
                step = WSTEP_EXIT;
                break;
            }

            #define JOYMENU_MAX_DEVICES 8
            char devLabels[JOYMENU_MAX_DEVICES][32];
            const char *devLabelPtrs[JOYMENU_MAX_DEVICES];
            uint16_t devVid[JOYMENU_MAX_DEVICES];
            uint16_t devPid[JOYMENU_MAX_DEVICES];
            uint8_t devAddr[JOYMENU_MAX_DEVICES];
            uint8_t n = (devCount > JOYMENU_MAX_DEVICES) ? JOYMENU_MAX_DEVICES : devCount;
            for (uint8_t i = 0; i != n; i++)
            {
                JoyWizard_GetDeviceInfo(i, devVid[i], devPid[i], devAddr[i]);
                if (devAddr[i] & 0x80)   // 2e manette d'un adaptateur (interface 1-3)
                    snprintf(devLabels[i], sizeof(devLabels[i]), "VID:%04X PID:%04X (addr %u pad %u)",
                             devVid[i], devPid[i], (devAddr[i] >> 2) & 0x1F, (devAddr[i] & 3) + 1);
                else
                    snprintf(devLabels[i], sizeof(devLabels[i]), "VID:%04X PID:%04X (addr %u)",
                             devVid[i], devPid[i], devAddr[i]);
                devLabelPtrs[i] = devLabels[i];
            }
            int8_t r = JoyMenu_PickOption("Joystick Config - Select USB Device", devLabelPtrs, n);
            if (r < 0) { step = WSTEP_MODE; }
            else
            {
                vid = devVid[r];
                pid = devPid[r];
                address = devAddr[r];
                cfg.vid = vid;
                cfg.pid = pid;
                cfg.mode = (modeSel == 0) ? JOY_MODE_DIGITAL : JOY_MODE_ANALOG;
                cfg.mode |= (uint8_t)((((address & 0x80) ? (address & 3) : 0) + 1) << 4);   // interface retenue
                step = WSTEP_FIRE1;
            }
            break;
        }

        case WSTEP_FIRE1:
        {
            uint8_t b, m;
            if (JoyMenu_CaptureWithHold(address, "Press and HOLD FIRE 1", b, m))
            {
                cfg.fire1Byte = b;
                cfg.fire1Mask = m;
                step = WSTEP_FIRE1_OK;
            }
            else { step = dev ? WSTEP_MODE : WSTEP_DEVICE; }
            break;
        }
        case WSTEP_FIRE1_OK:
        {
            snprintf(confirmDetail, sizeof(confirmDetail), "FIRE1 -> byte %u, mask 0x%02X", cfg.fire1Byte, cfg.fire1Mask);
            if (JoyMenu_ConfirmScreen("Confirm FIRE1", confirmDetail)) step = WSTEP_FIRE2;
            else step = WSTEP_FIRE1;
            break;
        }

        case WSTEP_FIRE2:
        {
            uint8_t b, m;
            if (JoyMenu_CaptureWithHold(address, "Press and HOLD FIRE 2", b, m))
            {
                cfg.fire2Byte = b;
                cfg.fire2Mask = m;
                step = WSTEP_FIRE2_OK;
            }
            else { step = WSTEP_FIRE1_OK; }
            break;
        }
        case WSTEP_FIRE2_OK:
        {
            snprintf(confirmDetail, sizeof(confirmDetail), "FIRE2 -> byte %u, mask 0x%02X", cfg.fire2Byte, cfg.fire2Mask);
            if (JoyMenu_ConfirmScreen("Confirm FIRE2", confirmDetail)) step = WSTEP_AXISX;
            else step = WSTEP_FIRE2;
            break;
        }

        case WSTEP_AXISX:
        {
            uint8_t b, m;
            if (JoyMenu_CaptureWithHold(address, "Move and HOLD AXIS X (Left or Right)", b, m, true))
            {
                cfg.xByte = b;
                cfg.xInvert = false;
                step = WSTEP_AXISX_OK;
            }
            else { step = WSTEP_FIRE2_OK; }
            break;
        }
        case WSTEP_AXISX_OK:
        {
            snprintf(confirmDetail, sizeof(confirmDetail), "AXIS X -> byte %u", cfg.xByte);
            if (JoyMenu_ConfirmScreen("Confirm AXIS X", confirmDetail)) step = WSTEP_AXISY;
            else step = WSTEP_AXISX;
            break;
        }

        case WSTEP_AXISY:
        {
            uint8_t b, m;
            if (JoyMenu_CaptureWithHold(address, "Move and HOLD AXIS Y (Up or Down)", b, m, true, cfg.xByte))
            {
                cfg.yByte = b;
                cfg.yInvert = false;
                step = WSTEP_AXISY_OK;
            }
            else { step = WSTEP_AXISX_OK; }
            break;
        }
        case WSTEP_AXISY_OK:
        {
            snprintf(confirmDetail, sizeof(confirmDetail), "AXIS Y -> byte %u", cfg.yByte);
            if (JoyMenu_ConfirmScreen("Confirm AXIS Y", confirmDetail)) step = WSTEP_SAVE;
            else step = WSTEP_AXISY;
            break;
        }

        case WSTEP_SAVE:
        {
            snprintf(confirmDetail, sizeof(confirmDetail), "%s joystick, VID:%04X PID:%04X",
                     (slotSel == JOY_SLOT_RIGHT) ? "RIGHT" : "LEFT", vid, pid);
            if (JoyMenu_ConfirmScreen("Save this configuration?", confirmDetail))
            {
                cfg.configured = true;
                JoystickCfg[slotSel] = cfg;
                UsbPort_StoreMap((uint8_t)slotSel);       // réglages gardés avec l'appareil (et l'autre port s'il l'a aussi)
                bool saved = SaveJoystickConfigToSD();
                savedOk = true;
                JoyMenu_ShowMessage(saved ? "Configuration saved!" : "Configured, but SD SAVE FAILED.",
                                     "If an axis feels reversed in-game, redo this config.",
                                     saved ? 0b00011100 : 0b11100000, 3000);
                step = WSTEP_EXIT;
            }
            else { step = WSTEP_AXISY_OK; }
            break;
        }

        default:
            step = WSTEP_EXIT;
            break;
        }
    }
    return savedOk;
}

// =====================================================================
// pages de niveau 3 (et NETWORK) dans le moteur du menu F12 navigation aux flèches partout, comme les
// pages de niveau 2.
// =====================================================================
static int Mn_Note(int y, const char *t, uint8_t c = MN_GREEN) { if (y > MN_Y_RB - 12) return y; Mn_Text(t, 1, y, c, 0); return y + 9; }   // pas sur la ligne Reboot / touches

// ---------------------------------------------------------------- DATA STORAGE > CASSETTE
static void V_CasTape(char *b, size_t n, uint8_t &c) { V_Cas(b, n, c); }
static bool E_CasTape(void) { DiskMenuChoose_1(9); if (s_menu_exit_all) { s_menu_exit_all = false; return true; } return false; }
static void V_CasPos(char *b, size_t n, uint8_t &c) { Cas_Position(b, n); c = MN_GREEN; }
static void V_CasMot(char *b, size_t n, uint8_t &c) { snprintf(b, n, "%s", Cas_MotorOn() ? "ON" : "OFF"); c = Cas_MotorOn() ? MN_GREEN : MN_BLUE; }
static bool E_CasRew(void)
{
    if (Cas_Mode() != CAS_PLAY) { s_mn_msg = "No tape to rewind."; s_mn_msg_color = MN_RED; return false; }
    Cas_Rewind(); s_mn_msg = "Tape rewound."; s_mn_msg_color = MN_GREEN; return false;
}
static bool E_CasRec(void)
{
    if (!Cas_RecordNew()) { s_mn_msg = "Cannot create a tape on the SD card."; s_mn_msg_color = MN_RED; return false; }
    s_mn_msg = "Blank tape ready: CSAVE / CSAVEM now."; s_mn_msg_color = MN_GREEN; return false;
}
static bool E_CasEj(void) { Cas_Eject(); s_mn_msg = "Tape ejected."; s_mn_msg_color = MN_GREEN; return false; }
static void V_CasFast(char *b, size_t n, uint8_t &c) { snprintf(b, n, "%s", Cas_GetFast() ? "ON" : "OFF"); c = Cas_GetFast() ? MN_GREEN : MN_RED; }
static void C_CasFast(int) { Cas_SetFast(!Cas_GetFast()); }
static const MnItem k_mn_cas[] = {
    { "TAPE",            22, V_CasTape, nullptr,   E_CasTape, nullptr, false, K_Cas },
    { "POSITION",        22, V_CasPos,  nullptr,   nullptr,   nullptr, true,  nullptr },
    { "MOTOR",           22, V_CasMot,  nullptr,   nullptr,   nullptr, true,  nullptr },
    { "REWIND",          22, nullptr,   nullptr,   E_CasRew,  nullptr, false, nullptr },
    { "RECORD NEW TAPE", 22, nullptr,   nullptr,   E_CasRec,  nullptr, false, nullptr },
    { "EJECT",           22, nullptr,   nullptr,   E_CasEj,   nullptr, false, nullptr },
    { "FAST LOAD",       22, V_CasFast, C_CasFast, nullptr,   nullptr, false, nullptr },
};
static int X_Cas(int y)
{
    y = Mn_Note(y, "CLOAD/CLOADM: pick a .CAS or .WAV tape first.");
    y = Mn_Note(y, "CSAVE: a new .CAS tape is made in /CASSETTE.");
    y = Mn_Note(y, "FAST LOAD: machine runs fast while motor is on.");
    y = Mn_Note(y, "AUDIO ON: hear the tape. No cassette pins used.");
    return y;
}
static const MnPage k_pg_cas = { "MENU > DATA STORAGE > CASSETTE", k_mn_cas, MN_N(k_mn_cas), nullptr, "ENTER: select  DEL on TAPE: eject", X_Cas, nullptr };
static bool Pg_Cas(void) { return Mn_Page(&k_pg_cas); }

// ---------------------------------------------------------------- DATA STORAGE > SAVE / LOAD STATE
static char s_st_msg[64];
static void V_StSw(char *b, size_t n, uint8_t &c)  { State_CurrentName(b, n); if (!b[0]) snprintf(b, n, "(none)"); c = MN_GREEN; }
static bool E_StSw(void)
{
    const int k = State_NextSoftware();
    if (k) snprintf(s_st_msg, sizeof s_st_msg, "%d software(s) with saved states.", k);
    else   snprintf(s_st_msg, sizeof s_st_msg, "No saved state on the SD card.");
    s_mn_msg = s_st_msg; s_mn_msg_color = MN_GREEN;
    return false;
}
static void C_StSw(int)                            { E_StSw(); }
static void V_StSlot(char *b, size_t n, uint8_t &c)
{
    char d[48];
    State_Describe(g_state_slot, d, sizeof d);
    snprintf(b, n, "%u  %s", (unsigned)g_state_slot + 1, d);
    c = strcmp(d, "(empty)") ? MN_GREEN : MN_RED;
}
static void C_StSlot(int dir)                      { g_state_slot = (uint8_t)((g_state_slot + (dir < 0 ? 3 : 1)) & 3); }
static __attribute__((unused)) void V_StNum(char *b, size_t n, uint8_t &c) { snprintf(b, n, "slot %u", (unsigned)g_state_slot + 1); c = MN_GREEN; }
static bool E_StSave(void)
{
    char d[48];
    State_Describe(g_state_slot, d, sizeof d);
    if (strcmp(d, "(empty)") != 0)
    {
        char q[40];
        snprintf(q, sizeof q, "REPLACE SLOT %u?", (unsigned)g_state_slot + 1);
        if (!Mn_AskYN(q)) return false;
    }
    s_mn_msg = "Saving, please wait..."; s_mn_msg_color = MN_YELLOW;
    Mn_Redraw2(Mn_DrawSub);
    const bool ok = State_Save(g_state_slot, g_wait_before_menu);
    s_mn_msg = ok ? "State saved." : "NOT SAVED: SD card missing or full.";
    s_mn_msg_color = ok ? MN_GREEN : MN_RED;
    return false;
}
static bool E_StLoad(void)
{
    s_mn_msg = "Loading, please wait..."; s_mn_msg_color = MN_YELLOW;
    Mn_Redraw2(Mn_DrawSub);
    char err[40];
    bool w = g_wait_before_menu;
    if (State_Load(g_state_slot, &w, err, sizeof err))
    {
        g_wait_before_menu = w;
        s_mn_msg = nullptr;
        return true;                       // retour direct au CoCo
    }
    snprintf(s_st_msg, sizeof s_st_msg, "NOT LOADED: %s", err);
    s_mn_msg = s_st_msg; s_mn_msg_color = MN_RED;
    return false;
}
static void K_StSlot(void)                 // Suppr sur SLOT = effacer cet emplacement
{
    char d[48];
    State_Describe(g_state_slot, d, sizeof d);
    if (strcmp(d, "(empty)") == 0) { s_mn_msg = "This slot is already empty."; s_mn_msg_color = MN_YELLOW; return; }
    char q[40];
    snprintf(q, sizeof q, "DELETE SLOT %u?", (unsigned)g_state_slot + 1);
    if (!Mn_AskYN(q)) return;
    const bool ok = State_Delete(g_state_slot);
    s_mn_msg = ok ? "Slot deleted." : "NOT DELETED: SD card error.";
    s_mn_msg_color = ok ? MN_GREEN : MN_RED;
}
static bool E_StDelAll(void)
{
    char nm[48], q[64];
    State_CurrentName(nm, sizeof nm);
    snprintf(q, sizeof q, "DELETE ALL STATES OF %.16s?", nm);
    if (!Mn_AskYN(q)) return false;
    const int n = State_DeleteAll();
    snprintf(s_st_msg, sizeof s_st_msg, "%d state(s) deleted.", n);
    s_mn_msg = s_st_msg; s_mn_msg_color = MN_GREEN;
    return false;
}
static int X_State(int y)
{
    y += 4;
    for (uint8_t i = 0; i < 4; i++)
    {
        char d[48], b[60];
        State_Describe(i, d, sizeof d);
        snprintf(b, sizeof b, "%c%u %s", i == g_state_slot ? '>' : ' ', (unsigned)i + 1, d);
        Mn_Text(b, 1, y, i == g_state_slot ? MN_WHITE : MN_GREEN, 0);
        y += 9;
    }
    y += 6;
    y = Mn_Note(y, "While running: F5 saves over the oldest slot,");
    y = Mn_Note(y, "F8 loads the newest. A state needs the same CPU,");
    y = Mn_Note(y, "RAM and cartridges; its disks are mounted again.");
    return y;
}
static const MnItem k_mn_state[] = {
    { "SOFTWARE",                   20, V_StSw,   C_StSw,   E_StSw,   nullptr, false, nullptr },
    { "SLOT",                       20, V_StSlot, C_StSlot, nullptr,  nullptr, false, K_StSlot },
    { "SAVE THE COCO TO THIS SLOT", 30, nullptr,  nullptr,  E_StSave, nullptr, false, nullptr },
    { "LOAD THIS SLOT",             30, nullptr,  nullptr,  E_StLoad, nullptr, false, nullptr },
    { "DELETE ALL STATES OF THIS SOFTWARE", 40, nullptr, nullptr, E_StDelAll, nullptr, false, nullptr },
};
static const MnPage k_pg_state = { "MENU > DATA STORAGE > STATE", k_mn_state, 5, nullptr, "DEL on SLOT: delete that slot", X_State, nullptr };
static bool Pg_State(void) { return Mn_Page(&k_pg_state); }

// ---------------------------------------------------------------- CART SLOT & COM (niveau 2, liste variable)
static int s_mn_slot = 0;                  // slot affiché par la page de niveau 3
static bool Slot_Pending1(int i)
{
    const SlotConfig &a = g_slotcfg, &r = g_slotcfg_run;
    if (a.mode[i] != r.mode[i]) return true;
    return SLOT_HAS_ROM(a.mode[i]) && (a.autostart[i] != r.autostart[i] || strcmp(a.name[i], r.name[i]) != 0);
}
static void Slot_Short(int i, char *b, size_t n, uint8_t &c)
{
    const SlotConfig &s = g_slotcfg;
    const uint8_t m = s.mode[i];
    c = MN_GREEN;
    if (m == SLOT_FDC)        snprintf(b, n, "DISK BASIC (FD-502)");
    else if (m == SLOT_EMPTY) { snprintf(b, n, "EMPTY"); c = MN_RED; }
    else if (m == SLOT_RS232) snprintf(b, n, "RS-232 PAK");
    else
    {
        char nm[40];
        Mn_Base(s.name[i], nm, sizeof nm);
        snprintf(b, n, "%s%s%s", m == SLOT_O90 ? "O90 " : (m == SLOT_GMC ? "GMC " : (m == SLOT_CPSG ? "PSG " : "")), nm, s.autostart[i] ? " (auto)" : "");
    }
}
static bool Pg_SlotContent(void);
#define MN_SLOT_FNS(i) \
    static void V_S##i(char *b, size_t n, uint8_t &c) { Slot_Short(i, b, n, c); } \
    static bool R_S##i(void) { return Slot_Pending1(i); } \
    static bool E_S##i(void) { s_mn_slot = i; return Pg_SlotContent(); }
MN_SLOT_FNS(0)
MN_SLOT_FNS(1)
MN_SLOT_FNS(2)
MN_SLOT_FNS(3)
static void V_MpiOn(char *b, size_t n, uint8_t &c) { snprintf(b, n, "%s", g_slotcfg.mpi ? "ON (4 SLOTS)" : "OFF"); c = g_slotcfg.mpi ? MN_GREEN : MN_RED; }
static void C_MpiOn(int)                           { g_slotcfg.mpi ^= 1; }
static bool R_MpiOn(void)                          { return g_slotcfg.mpi != g_slotcfg_run.mpi; }
static void V_Sw(char *b, size_t n, uint8_t &c)    { snprintf(b, n, "SLOT %d", g_slotcfg.sw + 1); c = MN_GREEN; }
static void C_Sw(int dir)                          { g_slotcfg.sw = (uint8_t)((g_slotcfg.sw + (dir < 0 ? 3 : 1)) & 3); }
static bool R_Sw(void)                             { return g_slotcfg.mpi && g_slotcfg.sw != g_slotcfg_run.sw; }
static bool E_Rs232(void)
{
    int s = -1;
    for (int i = 0; i < (g_slotcfg.mpi ? MPI_SLOTS : 1); i++) if (g_slotcfg.mode[i] == SLOT_RS232) s = i;
    if (s < 0) s = g_slotcfg.mpi ? g_slotcfg.sw : 0;
    s_mn_slot = s;
    return Pg_SlotContent();
}
static bool Pg_BitBang(void);
// le modem n'est pas une carte : c'est la liaison MODEM du RS-232 Pak ou du bit banger. Ligne d'état.
static void V_Mdm(char *b, size_t n, uint8_t &c)
{
    const bool rs = g_rs_present && g_rs_link == RS_LINK_MODEM, bb = g_bb_mode == BB_MODEM && !Bb_ModemBlocked();
    if (!rs && !bb) { snprintf(b, n, "unused (choose MODEM above)"); c = MN_BLUE; return; }
    char st[48]; Modem_StatusLine(st, sizeof st);
    snprintf(b, n, "%s: %s", rs ? "RS-232" : "BIT BANGER", st);
    c = strncmp(st, "CONNECTED", 9) == 0 ? MN_GREEN : MN_YELLOW;
}
static int B_Cart(MnItem *o, int max)
{
    (void)max;
    static bool (*const k_e[4])(void) = { E_S0, E_S1, E_S2, E_S3 };
    static void (*const k_v[4])(char *, size_t, uint8_t &) = { V_S0, V_S1, V_S2, V_S3 };
    static bool (*const k_r[4])(void) = { R_S0, R_S1, R_S2, R_S3 };
    static const char *const k_l[4] = { "  SLOT 1", "  SLOT 2", "  SLOT 3", "  SLOT 4" };
    int k = 0;
    o[k++] = MnItem{ "MULTI-PAK", 22, V_MpiOn, C_MpiOn, nullptr, R_MpiOn, false, nullptr };
    if (!g_slotcfg.mpi)
        o[k++] = MnItem{ "CARTRIDGE SLOT", 22, V_S0, nullptr, E_S0, R_S0, false, nullptr };
    else
    {
        o[k++] = MnItem{ "  POWER-ON SWITCH", 22, V_Sw, C_Sw, nullptr, R_Sw, false, nullptr };
        for (int i = 0; i < MPI_SLOTS; i++) o[k++] = MnItem{ k_l[i], 22, k_v[i], nullptr, k_e[i], k_r[i], false, nullptr };
    }
    o[k++] = MnItem{ "BIT BANGER PORT",     22, V_Bb,   nullptr, Pg_BitBang, nullptr, false, nullptr };
    o[k++] = MnItem{ "RS-232 PAK",          22, V_Rs,   nullptr, E_Rs232,    nullptr, false, nullptr };
    o[k++] = MnItem{ "HAYES MODEM (Wi-Fi)", 22, V_Mdm,  nullptr, nullptr,    nullptr, true,  nullptr };   // état
    return k;
}
static const MnPage k_pg_cart = { "MENU > CART SLOT & COM", nullptr, 0, B_Cart, "ENTER: what is in the slot", nullptr, nullptr };
static bool Pg_Cart(void) { return Mn_Page(&k_pg_cart); }

// ---------------------------------------------------------------- CART SLOT & COM > SLOT n (contenu)
struct MnCtType { const char *label; uint8_t mode; };
static const MnCtType k_ct[7] = {
    { "DISK CONTROLLER FD-502", SLOT_FDC  }, { "ROM CARTRIDGE",     SLOT_ROM  },
    { "ORCHESTRA-90",           SLOT_O90  }, { "GAME MASTER CART.", SLOT_GMC  },
    { "COCO PSG (YM2149)",      SLOT_CPSG }, { "RS-232 PAK (6551)", SLOT_RS232 },
    { "EMPTY SLOT",             SLOT_EMPTY } };
static void V_Ct(int t, char *b, size_t n, uint8_t &c)
{
    const SlotConfig &s = g_slotcfg;
    const uint8_t m = k_ct[t].mode;
    c = MN_GREEN;
    b[0] = 0;
    if (s.mode[s_mn_slot] == m)
    {
        if (SLOT_HAS_ROM(m)) Mn_Base(s.name[s_mn_slot], b, n);
        else snprintf(b, n, "INSTALLED");
        return;
    }
    if (SLOT_HAS_ROM(m) && CartLast_Get(m)[0])   // dernier fichier de ce type (blanc : pas en place)
    {
        char nm[40];
        Mn_Base(CartLast_Get(m), nm, sizeof nm);
        snprintf(b, n, "(%s)", nm);
        c = MN_WHITE;
    }
}
static void Ct_Browse(uint8_t m)           // niveau 4 : navigateur de la SD pour une ROM de ce type
{
    ClearFileList();
    s_cart_slot = s_mn_slot;
    s_cart_newmode = m;
    DiskMenuChoose_1(8);                   // installe et retient le fichier choisi (Slot_InstallRom)
    s_browse_cart = false;
    ClearFileList();
}
static bool R_Ct(int t) { return g_slotcfg.mode[s_mn_slot] == k_ct[t].mode && Slot_Pending1(s_mn_slot); }
static bool E_Ct(int t)
{
    SlotConfig &c = g_slotcfg;
    const uint8_t m = k_ct[t].mode;
    if (SLOT_HAS_ROM(m))
    {
        // dernier fichier de ce type mis en place directement ; sinon (ou s'il y est déjà) le
        // navigateur
        if (c.mode[s_mn_slot] != m && CartLast_Get(m)[0]) Slot_InstallRom(s_mn_slot, m, CartLast_Get(m));
        else Ct_Browse(m);
        return false;
    }
    if (m == SLOT_FDC || m == SLOT_RS232)  // un seul contrôleur, un seul RS-232 Pak
        for (int i = 0; i < MPI_SLOTS; i++) if (c.mode[i] == m) c.mode[i] = SLOT_EMPTY;
    c.mode[s_mn_slot] = m;
    return false;
}
#define MN_CT_FNS(t) \
    static void V_Ct##t(char *b, size_t n, uint8_t &c) { V_Ct(t, b, n, c); } \
    static bool R_Ct##t(void) { return R_Ct(t); } \
    static bool E_Ct##t(void) { return E_Ct(t); } \
    static void C_Ct##t(int) { if (SLOT_HAS_ROM(k_ct[t].mode)) Ct_Browse(k_ct[t].mode); }
MN_CT_FNS(0)
MN_CT_FNS(1)
MN_CT_FNS(2)
MN_CT_FNS(3)
MN_CT_FNS(4)
MN_CT_FNS(5)
MN_CT_FNS(6)
static void V_Auto(char *b, size_t n, uint8_t &c) { const bool a = g_slotcfg.autostart[s_mn_slot]; snprintf(b, n, "%s", a ? "ON (game paks)" : "OFF"); c = a ? MN_GREEN : MN_RED; }
static void C_Auto(int)                           { g_slotcfg.autostart[s_mn_slot] ^= 1; }
static void V_RsLink(char *b, size_t n, uint8_t &c)
{
    static const char *const k_lk[RS_LINK_COUNT] = { "NONE", "UART PINS 41 / 42", "LOOPBACK", "MODEM (Wi-Fi)" };
    snprintf(b, n, "%s", k_lk[g_rs_link < RS_LINK_COUNT ? g_rs_link : 0]);
    c = g_rs_link ? MN_GREEN : MN_RED;
}
static void C_RsLink(int dir) { g_rs_link = (uint8_t)((g_rs_link + (dir < 0 ? RS_LINK_COUNT - 1 : 1)) % RS_LINK_COUNT); Rs_ApplyLink(); }
static void V_RsBaud(char *b, size_t n, uint8_t &c)
{
    if (g_rs_speed) snprintf(b, n, "%lu", (unsigned long)k_rs_speed[g_rs_speed]); else snprintf(b, n, "AUTO (programmed)");
    c = MN_GREEN;
}
static void C_RsBaud(int dir) { g_rs_speed = (uint8_t)((g_rs_speed + (dir < 0 ? RS_SPEED_COUNT - 1 : 1)) % RS_SPEED_COUNT); Rs_ApplyLink(); }
static int B_SlotC(MnItem *o, int max)
{
    (void)max;
    static void (*const k_v[7])(char *, size_t, uint8_t &) = { V_Ct0, V_Ct1, V_Ct2, V_Ct3, V_Ct4, V_Ct5, V_Ct6 };
    static bool (*const k_e[7])(void) = { E_Ct0, E_Ct1, E_Ct2, E_Ct3, E_Ct4, E_Ct5, E_Ct6 };
    static bool (*const k_r[7])(void) = { R_Ct0, R_Ct1, R_Ct2, R_Ct3, R_Ct4, R_Ct5, R_Ct6 };
    static void (*const k_c[7])(int) = { C_Ct0, C_Ct1, C_Ct2, C_Ct3, C_Ct4, C_Ct5, C_Ct6 };
    int k = 0;
    for (int t = 0; t < 7; t++) o[k++] = MnItem{ k_ct[t].label, 25, k_v[t], SLOT_HAS_ROM(k_ct[t].mode) ? k_c[t] : nullptr, k_e[t], k_r[t], false, nullptr };
    const uint8_t m = g_slotcfg.mode[s_mn_slot];
    o[k++] = MnItem{ "", 25, nullptr, nullptr, nullptr, nullptr, true, nullptr };
    o[k++] = MnItem{ "AUTOSTART", 25, V_Auto, C_Auto, nullptr, nullptr, !SLOT_HAS_ROM(m), nullptr };
    if (m == SLOT_RS232)
    {
        o[k++] = MnItem{ "RS-232 LINK", 25, V_RsLink, C_RsLink, nullptr, nullptr, false, nullptr };
        o[k++] = MnItem{ "RS-232 BAUD RATE", 25, V_RsBaud, C_RsBaud, nullptr, nullptr, false, nullptr };
    }
    return k;
}
// état du modem sous les pages RS-232 Pak et BIT BANGER
static int Mdm_Notes(int y, bool blocked)
{
    if (blocked) return Mn_Note(y, "MODEM busy: used by the RS-232 Pak.", MN_RED);
    if (!Net_Connected()) y = Mn_Note(y, "Wi-Fi not connected: turn it ON in NETWORK.", MN_RED);
    char st[48], b[64];
    Modem_StatusLine(st, sizeof st);
    snprintf(b, sizeof b, "MODEM: %.44s", st);
    y = Mn_Note(y, b, MN_YELLOW);
    return Mn_Note(y, "Terminal: ATDT host:port to call (MODEM.TXT).");
}
static int X_SlotC(int y)
{
    y = Mn_Note(y, "One disk controller only. A ROM cartridge alone");
    y = Mn_Note(y, "replaces it (DriveWire and VHD still work).");
    y = Mn_Note(y, "ROM files: 32 KB, or up to 512 KB bank-switched.");
    if (g_slotcfg.mode[s_mn_slot] == SLOT_RS232 && g_rs_link == RS_LINK_MODEM) y = Mdm_Notes(y + 4, false);
    return y;
}
static char s_slot_title[40];
static const MnPage k_pg_slotc = { s_slot_title, nullptr, 0, B_SlotC, "ENTER: install   ROM: RIGHT = choose another file", X_SlotC, nullptr };
static bool Pg_SlotContent(void)
{
    if (g_slotcfg.mpi) snprintf(s_slot_title, sizeof s_slot_title, "MENU > CART SLOT > SLOT %d", s_mn_slot + 1);
    else               snprintf(s_slot_title, sizeof s_slot_title, "MENU > CART SLOT > CARTRIDGE");
    return Mn_Page(&k_pg_slotc);
}

// ---------------------------------------------------------------- CART SLOT & COM > BIT BANGER
static const char *const k_bb_lbl[BB_MODE_COUNT] = {
    "OFF", "PRINTER -> SD CARD", "DRIVEWIRE BUILT-IN SERVER", "UART PINS 41 / 42", "DRIVEWIRE ON A PC (Wi-Fi)", "MODEM (Wi-Fi): BBS, TELNET" };
static const uint8_t k_bb_mode_of[BB_MODE_COUNT] = { BB_OFF, BB_PRINTER, BB_DRIVEWIRE, BB_UART, BB_NETDW, BB_MODEM };
static void V_BbM(int i, char *b, size_t n, uint8_t &c) { if (g_bb_mode == k_bb_mode_of[i]) snprintf(b, n, "SELECTED"); else b[0] = 0; c = MN_GREEN; }
static bool E_BbM(int i) { g_bb_mode = k_bb_mode_of[i]; Bb_Reset(); if (g_bb_mode == BB_MODEM) Modem_Use(); return false; }   // modem démarré ici, pas au reset du CoCo
#define MN_BB_FNS(i) \
    static void V_BbM##i(char *b, size_t n, uint8_t &c) { V_BbM(i, b, n, c); } \
    static bool E_BbM##i(void) { return E_BbM(i); }
MN_BB_FNS(0)
MN_BB_FNS(1)
MN_BB_FNS(2)
MN_BB_FNS(3)
MN_BB_FNS(4)
MN_BB_FNS(5)
static void V_BbBaud(char *b, size_t n, uint8_t &c)
{
    if (g_bb_speed) snprintf(b, n, "%lu (fixed)", (unsigned long)k_bb_speed[g_bb_speed]);
    else
    {
        const uint32_t r = Bb_CurrentBaud();
        if (r) snprintf(b, n, "AUTO (now %lu)", (unsigned long)r); else snprintf(b, n, "AUTO");
    }
    c = MN_GREEN;
}
static void C_BbBaud(int dir) { g_bb_speed = (uint8_t)((g_bb_speed + (dir < 0 ? BB_SPEED_COUNT - 1 : 1)) % BB_SPEED_COUNT); Bb_Reset(); }
static const MnItem k_mn_bb[] = {
    { k_bb_lbl[0], 28, V_BbM0, nullptr, E_BbM0, nullptr, false, nullptr },
    { k_bb_lbl[1], 28, V_BbM1, nullptr, E_BbM1, nullptr, false, nullptr },
    { k_bb_lbl[2], 28, V_BbM2, nullptr, E_BbM2, nullptr, false, nullptr },
    { k_bb_lbl[3], 28, V_BbM3, nullptr, E_BbM3, nullptr, false, nullptr },
    { k_bb_lbl[4], 28, V_BbM4, nullptr, E_BbM4, nullptr, false, nullptr },
    { k_bb_lbl[5], 28, V_BbM5, nullptr, E_BbM5, nullptr, false, nullptr },
    { "",          28, nullptr, nullptr, nullptr, nullptr, true, nullptr },
    { "BAUD RATE", 28, V_BbBaud, C_BbBaud, nullptr, nullptr, false, nullptr },
};
static int X_Bb(int y)
{
    y = Mn_Note(y, "PRINTER: LLIST, PRINT #-2 -> /PRINTS/PRTnnnn.TXT");
    y = Mn_Note(y, "DRIVEWIRE: NitrOS-9 'dw' disks, without a PC.");
    y = Mn_Note(y, "UART: 3.3 V TTL serial (PC DriveWire, terminal).");
    y = Mn_Note(y, "Wi-Fi: PC DriveWire server set in NETWORK.");
    y = Mn_Note(y, "AUTO measures the speed sent by the CoCo.");
    if (g_bb_mode == BB_UART && Bb_UartBlocked()) y = Mn_Note(y, "UART busy: used by the RS-232 Pak.", MN_RED);
    if (g_bb_mode == BB_NETDW && !NetDw_Enabled()) y = Mn_Note(y, "Turn DRIVEWIRE BY Wi-Fi ON in NETWORK first.", MN_RED);
    if (g_bb_mode == BB_MODEM) y = Mdm_Notes(y, Bb_ModemBlocked());
    return y;
}
static bool P_Bb(void)                     // vitesse AUTO mesurée : redessin seulement si elle change
{
    static uint32_t last = 0xFFFFFFFFu;
    const uint32_t now = g_bb_speed ? 0 : Bb_CurrentBaud();
    const bool ch = now != last;
    last = now;
    return ch;
}
static const MnPage k_pg_bb = { "MENU > CART SLOT > BIT BANGER", k_mn_bb, 7, nullptr, "ENTER: select", X_Bb, P_Bb };
static bool Pg_BitBang(void) { return Mn_Page(&k_pg_bb); }

// ---------------------------------------------------------------- USB DEVICES > MOUSE
// port = masque (bit 0 droit, bit 1 gauche ; 3 = les deux)
static uint8_t Mouse_PortSel(void) { return g_mouse_mode == 0 ? 0 : (g_mouse_mode <= 2 ? 1 : (g_mouse_mode <= 4 ? 2 : 3)); }
static bool Mouse_HiSel(void)      { return (g_mouse_mode == 2 || g_mouse_mode == 4 || g_mouse_mode == 6) || (g_mouse_mode == 0 && g_mouse_hires_pref); }
static void Mouse_Set(uint8_t port, bool hires)
{
    g_mouse_hires_pref = hires;
    g_mouse_mode = port == 0 ? 0 : port == 1 ? (hires ? 2 : 1) : port == 2 ? (hires ? 4 : 3) : (hires ? 6 : 5);
}
static __attribute__((unused)) void V_MsPort(char *b, size_t n, uint8_t &c) { const uint8_t p = Mouse_PortSel(); snprintf(b, n, "%s", p == 0 ? "OFF" : (p == 1 ? "RIGHT JOYSTICK" : "LEFT JOYSTICK")); c = p ? MN_GREEN : MN_RED; }
static __attribute__((unused)) void C_MsPort(int dir)  { Mouse_Set((uint8_t)((Mouse_PortSel() + (dir < 0 ? 2 : 1)) % 3), Mouse_HiSel()); }
static void V_MsRes(char *b, size_t n, uint8_t &c)  { snprintf(b, n, "%s", Mouse_HiSel() ? "HIGH (Hi-Res interface)" : "LOW (Tandy mouse, 64 steps)"); c = MN_GREEN; }
static void C_MsRes(int)       { Mouse_Set(Mouse_PortSel(), !Mouse_HiSel()); }
static void V_MsSens(char *b, size_t n, uint8_t &c)
{
    char bar[10];
    for (int i = 0; i < 8; i++) bar[i] = (i < g_mouse_sens) ? '#' : '.';
    bar[8] = 0;
    snprintf(b, n, "%s %u", bar, (unsigned)g_mouse_sens);
    c = MN_GREEN;
}
static void C_MsSens(int dir)  { if (dir > 0 && g_mouse_sens < 8) g_mouse_sens++; if (dir < 0 && g_mouse_sens > 1) g_mouse_sens--; }
static const MnItem k_mn_mouse[] = {                 // le port se choisit à la page USB DEVICES
    { "RESOLUTION",  16, V_MsRes,  C_MsRes,  nullptr, nullptr, false, nullptr },
    { "SENSITIVITY", 16, V_MsSens, C_MsSens, nullptr, nullptr, false, nullptr },
};
static int X_Mouse(int y)
{
    y += 4;
    y = Mn_Note(y, "OS-9 / Multi-Vue / GShell default:");
    y = Mn_Note(y, "RIGHT joystick, LOW resolution.");
    y = Mn_Note(y, "HIGH needs the Hi-Res setting in OS-9.");
    y = Mn_Note(y, "Left button = fire 1, right button = fire 2.");
    return y;
}
static const MnPage k_pg_mouse = { "MENU > USB DEVICES > MOUSE", k_mn_mouse, 2, nullptr, nullptr, X_Mouse, nullptr };
static bool Pg_Mouse(void) { return Mn_Page(&k_pg_mouse); }

// ---------------------------------------------------------------- USB DEVICES > KEYBOARD JOYSTICK
static const char *const k_kj_sets[3][3] = {
    { "ARROW KEYS",     "Move: arrow keys",                   "Fire: F1 / F2" },
    { "NUMERIC KEYPAD", "Move: 8 2 4 6 (7 9 1 3 diagonals)", "Fire: 0 / keypad ENTER" },
    { "W A S D",        "Move: W A S D",                      "Fire: SPACE / F" } };

// ---------------------------------------------------------------- NETWORK (niveau 2)
static void V_NWifi(char *b, size_t n, uint8_t &c)  { snprintf(b, n, "%s", Net_Enabled() ? "ON" : "OFF"); c = Net_Enabled() ? MN_GREEN : MN_RED; }
static void C_NWifi(int)       { Net_SetEnabled(!Net_Enabled()); }
static void V_NStat(char *b, size_t n, uint8_t &c)  { Net_StatusLine(b, n); c = Net_Connected() ? MN_GREEN : MN_YELLOW; }
static void V_NSsid(char *b, size_t n, uint8_t &c)  { snprintf(b, n, "%s", Net_Ssid()[0] ? Net_Ssid() : "(none)"); c = Net_Ssid()[0] ? MN_GREEN : MN_RED; }
static bool E_NSsid(void)
{
    NetScanAndConnect();                   // niveau 3 : liste des réseaux (flèches + ENTER)
    Menu_KeySync();
    if (Net_Enabled() && Net_Ssid()[0]) { s_mn_msg = "Connecting... (status updates every second)"; s_mn_msg_color = MN_YELLOW; }
    return false;
}
static void V_NTime(char *b, size_t n, uint8_t &c)
{
    struct tm t;
    if (Net_LocalTime(&t)) { snprintf(b, n, "%04d-%02d-%02d %02d:%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min); c = MN_GREEN; }
    else { snprintf(b, n, "unknown (needs Wi-Fi)"); c = MN_RED; }
}
static void V_NTz(char *b, size_t n, uint8_t &c)    { snprintf(b, n, "%s", k_net_tz_label[Net_Tz() < NET_TZ_COUNT ? Net_Tz() : 0]); c = MN_GREEN; }
static void C_NTz(int dir)     { Net_SetTz((uint8_t)((Net_Tz() + (dir < 0 ? NET_TZ_COUNT - 1 : 1)) % NET_TZ_COUNT)); }
static void V_NDw(char *b, size_t n, uint8_t &c)    { snprintf(b, n, "%s", NetDw_Enabled() ? "ON" : "OFF"); c = NetDw_Enabled() ? MN_GREEN : MN_RED; }
static void C_NDw(int)         { NetDw_SetEnabled(!NetDw_Enabled()); }
static void V_NDwSt(char *b, size_t n, uint8_t &c)  { NetDw_StatusLine(b, n); c = NetDw_Linked() ? MN_GREEN : (NetDw_Enabled() ? MN_YELLOW : MN_RED); }
static void V_NDwSrv(char *b, size_t n, uint8_t &c) { snprintf(b, n, "%s", NetDw_Server()[0] ? NetDw_Server() : "(none)"); c = NetDw_Server()[0] ? MN_GREEN : MN_RED; }
static bool E_NDwSrv(void)
{
    char hp[64];
    strncpy(hp, NetDw_Server(), sizeof hp - 1); hp[sizeof hp - 1] = 0;
    if (Menu_TextInput("Server IP or name (:port if not 65504):", hp, sizeof hp, 21))
    {
        NetDw_SetServer(hp);
        s_mn_msg = "Server saved."; s_mn_msg_color = MN_GREEN;
    }
    Menu_KeySync();
    return false;
}
// page web (webui.cpp)
static void V_NWeb(char *b, size_t n, uint8_t &c)   { snprintf(b, n, "%s", Web_Enabled() ? "ON" : "OFF"); c = Web_Enabled() ? MN_GREEN : MN_RED; }
static void C_NWeb(int)        { Web_SetEnabled(!Web_Enabled()); }
static void V_NWebAd(char *b, size_t n, uint8_t &c) { Web_StatusLine(b, n); c = !Web_Enabled() ? MN_RED : (strncmp(b, "http", 4) == 0 ? MN_GREEN : MN_YELLOW); }
static void V_NWebPin(char *b, size_t n, uint8_t &c) { snprintf(b, n, "%s  (ENTER: new PIN)", Web_Pin()); c = MN_GREEN; }
static bool E_NWebPin(void)    { Web_NewPin(); s_mn_msg = "New PIN saved: type it in the browser."; s_mn_msg_color = MN_GREEN; return false; }
static const MnItem k_mn_net[] = {
    { "Wi-Fi",              20, V_NWifi,  C_NWifi, nullptr,  nullptr, false, nullptr },
    { "  STATUS",           20, V_NStat,  nullptr, nullptr,  nullptr, true,  nullptr },
    { "NETWORK (SSID)",     20, V_NSsid,  nullptr, E_NSsid,  nullptr, false, nullptr },
    { "TIME ZONE",          20, V_NTz,    C_NTz,   nullptr,  nullptr, false, nullptr },
    { "  TIME",             20, V_NTime,  nullptr, nullptr,  nullptr, true,  nullptr },
    { "",                   20, nullptr,  nullptr, nullptr,  nullptr, true,  nullptr },
    { "DRIVEWIRE BY Wi-Fi", 20, V_NDw,    C_NDw,   nullptr,  nullptr, false, nullptr },
    { "  STATUS",           20, V_NDwSt,  nullptr, nullptr,  nullptr, true,  nullptr },
    { "DRIVEWIRE SERVER",   20, V_NDwSrv, nullptr, E_NDwSrv, nullptr, false, nullptr },
    { "",                   20, nullptr,  nullptr, nullptr,  nullptr, true,  nullptr },
    { "WEB PAGE (SD CARD)", 20, V_NWeb,   C_NWeb,  nullptr,  nullptr, false, nullptr },
    { "  ADDRESS",          20, V_NWebAd, nullptr, nullptr,  nullptr, true,  nullptr },
    { "  PIN",              20, V_NWebPin, nullptr, E_NWebPin, nullptr, false, nullptr },
};
static int X_Net(int y)
{
    if (g_bb_mode == BB_NETDW) y = Mn_Note(y, "DriveWire by Wi-Fi is used by the bit banger.", MN_YELLOW);
    else y = Mn_Note(y, "When ON, the Becker port ($FF41) goes to the PC.");
    y = Mn_Note(y, "Name and password are kept in the ESP32 flash.");
    return y;
}
static bool P_Net(void)                    // état, heure (minute) : redessin seulement s'ils changent
{
    static char shown[176] = "";
    char now[176], st[48], dws[48];
    struct tm t;
    Net_StatusLine(st, sizeof st);
    NetDw_StatusLine(dws, sizeof dws);
    char ws[48]; Web_StatusLine(ws, sizeof ws);
    snprintf(now, sizeof now, "%s|%s|%s|%d", st, dws, ws, Net_LocalTime(&t) ? t.tm_hour * 60 + t.tm_min : -1);
    if (strcmp(now, shown) == 0) return false;
    strcpy(shown, now);
    return true;
}
static const MnPage k_pg_net = { "MENU > NETWORK", k_mn_net, MN_N(k_mn_net), nullptr, "ENTER on NETWORK: search and connect", X_Net, P_Net };
static bool Pg_Net(void) { return Mn_Page(&k_pg_net); }

// =====================================================================
// MENU > USB DEVICES (niveau 2) : clavier du CoCo, JOY (R), JOY (L) état USB, appareils en ligne
// (vert) puis vus auparavant (rouge, 8 max). COCO KEYBOARD
// =====================================================================
static uint8_t s_kj_port = 0;              // port de la page KEYBOARD JOYSTICK
static UsbDevView s_usb_rows[16];          // lignes « appareils » de la page

static const char *Usb_TagText(const UsbDevView &d)
{
    switch (d.tag)
    {
    case UTAG_SET: return "WSET";
    case UTAG_NEW: return "YNEW";
    case UTAG_KBD: return "WKBD";
    default:       return d.type == UDEV_MOUSE ? "WMOUSE" : "WAUTO";
    }
}
static void Usb_DevName(const UsbDevView &d, char *b, size_t n)
{
    if (d.type == UDEV_PAD && d.npads > 1) snprintf(b, n, "%.20s #%u", d.name, (unsigned)d.pad);
    else snprintf(b, n, "%s", d.name);
}

// ---- COCO KEYBOARD
static void V_UKbd(char *b, size_t n, uint8_t &c)
{
    UsbDevView d;
    if (!UsbKbd_Get(d)) { snprintf(b, n, "(none)"); c = MN_RED; return; }
    snprintf(b, n, "%s", d.name);
    c = d.online ? MN_GREEN : MN_RED;
}
static void C_UKbd(int dir)
{
    UsbDevView ch[6];
    const int n = UsbKbd_Choices(ch, 6);
    if (n <= 0) return;
    UsbDevView cur;
    int i = -1;
    if (UsbKbd_Get(cur)) for (int k = 0; k < n; k++) if (UsbDev_SameDev(cur, ch[k])) i = k;
    i = (i < 0) ? 0 : (i + (dir < 0 ? n - 1 : 1)) % n;
    UsbKbd_Set(ch[i]);
}
static void V_UKbdWarn(char *b, size_t n, uint8_t &c)
{
    snprintf(b, n, "KEYBOARD CHANGED: select it, then check JOY keys");
    c = MN_RED;
}

// ---- JOY (R) / JOY (L)
static void V_UJoy(uint8_t p, char *b, size_t n, uint8_t &c)
{
    UsbDevView d;
    if (!UsbPort_Device(p, d)) { snprintf(b, n, "ANY GAMEPAD (AUTO)"); c = MN_GREEN; return; }
    char nm[32];
    Usb_DevName(d, nm, sizeof nm);
    if (d.type == UDEV_KBD)
    {
        static const char *const k_set[3] = { "ARROWS", "KEYPAD", "WASD" };
        const uint8_t set = (p == 0 ? g_kbjoy_set : g_kbjoy_set2) % 3;
        snprintf(b, n, "%.14s %s\t%s", nm, k_set[set], Usb_TagText(d));
    }
    else if (d.type == UDEV_MOUSE)                // résolution visible
        snprintf(b, n, "%.16s %s\t%s", nm, (g_mouse_mode == 2 || g_mouse_mode == 4 || g_mouse_mode == 6) ? "HI-RES" : "LOW-RES", Usb_TagText(d));
    else snprintf(b, n, "%s\t%s", nm, Usb_TagText(d));
    c = d.online ? MN_GREEN : MN_RED;
}
static void C_UJoy(uint8_t p, int dir)
{
    UsbDevView ch[12];
    const int n = UsbPort_Choices(p, ch, 12);
    if (n <= 0) return;
    UsbDevView cur;
    int i = 0;
    if (UsbPort_Device(p, cur))
        for (int k = 1; k < n; k++)
            if (ch[k].type == cur.type && (cur.type != UDEV_PAD || UsbDev_SameDev(cur, ch[k]))) { i = k; break; }
    i = (i + (dir < 0 ? n - 1 : 1)) % n;
    UsbPort_Set(p, ch[i]);
    if (ch[i].type == UDEV_PAD && ch[i].tag == UTAG_NEW) { s_mn_msg = "New gamepad: press ENTER to set it up."; s_mn_msg_color = MN_YELLOW; }
    else s_mn_msg = nullptr;
}
static bool Pg_KbJoyPort(void);
static bool E_UJoy(uint8_t p)
{
    switch (UsbPort_Kind(p))
    {
    case UPORT_KBD:   s_kj_port = p; return Pg_KbJoyPort();
    case UPORT_MOUSE: return Pg_Mouse();
    case UPORT_PAD:
    {
        UsbDevView d;
        UsbPort_Device(p, d);
        if (!d.online || !d.key) { s_mn_msg = "Gamepad offline: plug it in first."; s_mn_msg_color = MN_RED; return false; }
        JoyWizard_ForDevice(p, d);
        Menu_KeySync();
        return false;
    }
    default:
        s_mn_msg = "LEFT / RIGHT: choose a device first."; s_mn_msg_color = MN_YELLOW;
        return false;
    }
}
static void V_UJoyR(char *b, size_t n, uint8_t &c) { V_UJoy(0, b, n, c); }
static void V_UJoyL(char *b, size_t n, uint8_t &c) { V_UJoy(1, b, n, c); }
static void C_UJoyR(int dir) { C_UJoy(0, dir); }
static void C_UJoyL(int dir) { C_UJoy(1, dir); }
static bool E_UJoyR(void) { return E_UJoy(0); }
static bool E_UJoyL(void) { return E_UJoy(1); }

// ---- lignes « appareils »
static void V_Ur(int i, char *b, size_t n, uint8_t &c)
{
    const UsbDevView &d = s_usb_rows[i];
    char nm[32];
    Usb_DevName(d, nm, sizeof nm);
    snprintf(b, n, "%-22.22s %s\t%s", nm, !d.online ? "OFFLINE" : d.nochan == 2 ? "NO MEMORY" : d.nochan ? "NO CHANNEL" : "ONLINE", Usb_TagText(d));
    c = !d.online ? MN_RED : d.nochan ? MN_YELLOW : MN_GREEN;
}
static void K_Ur(int i)
{
    UsbDev_Forget(s_usb_rows[i]);
    s_mn_msg = "Device forgotten."; s_mn_msg_color = MN_GREEN;
}
#define MN_UR_FNS(i) \
    static void V_Ur##i(char *b, size_t n, uint8_t &c) { V_Ur(i, b, n, c); } \
    static void K_Ur##i(void) { K_Ur(i); }
MN_UR_FNS(0)  MN_UR_FNS(1)  MN_UR_FNS(2)  MN_UR_FNS(3)
MN_UR_FNS(4)  MN_UR_FNS(5)  MN_UR_FNS(6)  MN_UR_FNS(7)
MN_UR_FNS(8)  MN_UR_FNS(9)  MN_UR_FNS(10) MN_UR_FNS(11)
MN_UR_FNS(12) MN_UR_FNS(13) MN_UR_FNS(14) MN_UR_FNS(15)
static void (*const k_ur_v[16])(char *, size_t, uint8_t &) = {
    V_Ur0, V_Ur1, V_Ur2, V_Ur3, V_Ur4, V_Ur5, V_Ur6, V_Ur7, V_Ur8, V_Ur9, V_Ur10, V_Ur11, V_Ur12, V_Ur13, V_Ur14, V_Ur15 };
static void (*const k_ur_k[16])(void) = {
    K_Ur0, K_Ur1, K_Ur2, K_Ur3, K_Ur4, K_Ur5, K_Ur6, K_Ur7, K_Ur8, K_Ur9, K_Ur10, K_Ur11, K_Ur12, K_Ur13, K_Ur14, K_Ur15 };
static const char *Usb_TypeLabel(uint8_t t) { return t == UDEV_KBD ? "KEYBOARD" : t == UDEV_MOUSE ? "MOUSE" : "GAMEPAD"; }

static int B_Usb(MnItem *o, int max)
{
    (void)max;
    int k = 0;
    o[k++] = MnItem{ "COCO KEYBOARD", 24, V_UKbd, C_UKbd, nullptr, nullptr, false, nullptr };
    o[k++] = MnItem{ "LOWERCASE 32 COL. (T1)", 24, V_Lc, C_Lc, nullptr, nullptr, false, nullptr };
    const bool warn = UsbKbd_Changed();
    if (warn) o[k++] = MnItem{ "", 1, V_UKbdWarn, nullptr, nullptr, nullptr, true, nullptr };
    o[k++] = MnItem{ "JOY (R)", 24, V_UJoyR, C_UJoyR, E_UJoyR, nullptr, false, nullptr };
    o[k++] = MnItem{ "JOY (L)", 24, V_UJoyL, C_UJoyL, E_UJoyL, nullptr, false, nullptr };
    o[k++] = MnItem{ k_mn_sep, 0, nullptr, nullptr, nullptr, nullptr, true, nullptr };
    s_mn_dyn_compact = (uint8_t)k;             // rangées serrées dès « USB STATUS »
    o[k++] = MnItem{ "USB STATUS", 12, V_UsbSt, nullptr, nullptr, nullptr, true, nullptr };
    // place : de y = 16 + rangées de 12 px + 5 (ligne) + 9 (état), jusqu'à 180 ; 5 px pour la 2e ligne
    const int ntop = warn ? 5 : 4;
    int room = (180 - (16 + ntop * MN_PITCH + 5 + 9 + 5)) / 9;
    int non = 0;
    const int nall = UsbDev_List(s_usb_rows, 16, &non);
    int r = 0;
    uint8_t lastType = 0;
    for (; r < non && r < nall && room > 0 && k < MN_DYN_MAX; r++, room--)
    {
        const UsbDevView &d = s_usb_rows[r];
        o[k++] = MnItem{ d.type != lastType ? Usb_TypeLabel(d.type) : "", 12, k_ur_v[r], nullptr, nullptr, nullptr, true, nullptr };
        lastType = d.type;
    }
    if (nall > non && room > 0 && k < MN_DYN_MAX)
    {
        o[k++] = MnItem{ k_mn_sep, 0, nullptr, nullptr, nullptr, nullptr, true, nullptr };
        lastType = 0;
        for (r = non; r < nall && room > 0 && k < MN_DYN_MAX; r++, room--)
        {
            const UsbDevView &d = s_usb_rows[r];
            o[k++] = MnItem{ d.type != lastType ? Usb_TypeLabel(d.type) : "", 12, k_ur_v[r], nullptr, nullptr, nullptr, false, k_ur_k[r] };
            lastType = d.type;
        }
    }
    return k;
}
static bool P_Usb(void)                    // appareils branchés / débranchés : redessin
{
    static uint32_t last = 0;
    UsbDevView d[16];
    int non = 0;
    const int n = UsbDev_List(d, 16, &non);
    uint32_t h = (uint32_t)n * 131u + (uint32_t)non;
    for (int i = 0; i < n; i++) h = h * 33u + d[i].vid * 7u + d[i].pid + d[i].pad + (d[i].online ? 1000u : 0u) + (d[i].nochan ? 5000u : 0u);
    { extern int Usb_StalledPorts(char *, size_t); h = h * 33u + (uint32_t)Usb_StalledPorts(nullptr, 0); }
    const bool ch = h != last;
    last = h;
    return ch;
}
static const MnPage k_pg_usbd = { "MENU > USB DEVICES", nullptr, 0, B_Usb, nullptr, nullptr, P_Usb, 0xFF };
static bool Pg_Usb(void) { return Mn_Page(&k_pg_usbd); }

// ---- JOY (R/L) > KEYBOARD : jeu de touches de ce port
static void V_KjSetP(char *b, size_t n, uint8_t &c)
{
    snprintf(b, n, "%s", k_kj_sets[(s_kj_port == 0 ? g_kbjoy_set : g_kbjoy_set2) % 3][0]);
    c = MN_GREEN;
}
static void C_KjSetP(int dir)
{
    uint8_t &set = s_kj_port == 0 ? g_kbjoy_set : g_kbjoy_set2;
    set = (uint8_t)((set % 3 + (dir < 0 ? 2 : 1)) % 3);
}
static int X_KbJoyP(int y)
{
    const uint8_t set = (s_kj_port == 0 ? g_kbjoy_set : g_kbjoy_set2) % 3;
    y += 4;
    y = Mn_Note(y, k_kj_sets[set][1], MN_WHITE);
    y = Mn_Note(y, k_kj_sets[set][2], MN_WHITE);
    y += 6;
    y = Mn_Note(y, "These keys go to the joystick only, not to");
    y = Mn_Note(y, "the CoCo keyboard. The same keyboard can drive");
    y = Mn_Note(y, "both ports (same or other keys).");
    return y;
}
static const MnItem k_mn_kjp[] = {
    { "KEYS", 16, V_KjSetP, C_KjSetP, nullptr, nullptr, false, nullptr },
};
static char s_kj_title[40];
static const MnPage k_pg_kjp = { s_kj_title, k_mn_kjp, 1, nullptr, nullptr, X_KbJoyP, nullptr, 0 };
static bool Pg_KbJoyPort(void)
{
    snprintf(s_kj_title, sizeof s_kj_title, "MENU > USB > JOY (%c) KEYBOARD", s_kj_port == 0 ? 'R' : 'L');
    return Mn_Page(&k_pg_kjp);
}
