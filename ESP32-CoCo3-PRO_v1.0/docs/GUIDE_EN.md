# ESP32-CoCo3-PRO 1.0 — User guide
# Author : Luc Dion 2026-10-26.

ESP32-CoCo3-PRO emulates a Tandy Color Computer 3 on an ESP32-S3 board. Everything the CoCo uses
(floppies, hard disks, tapes, cartridges) lives as files on the SD card.

Contents: [1. SD card](#1-sd-card) · [2. Keys](#2-keys) · [3. Settings menu (F12)](#3-settings-menu-f12) ·
[4. Disks](#4-disks) · [5. Cassette](#5-cassette) · [6. Cartridges and serial ports](#6-cartridges-and-serial-ports) ·
[7. USB devices](#7-usb-devices) · [8. Network](#8-network) · [9. Boot screen, screen saver, AUTORUN](#9-boot-screen-screen-saver-autorun) ·
[10. Save states](#10-save-states) · [11. Firmware update](#11-firmware-update) · [12. Known software notes](#12-known-software-notes) ·
[13. Reporting a problem](#13-reporting-a-problem)

## 1. SD card

FAT32, 4 to 32 GB. Put your files in any folders. Folders made by the emulator:

| Folder | Use |
|---|---|
| `/ESP-CONFIG` | settings (`config.cfg`, `joyconfig.cfg`, `usbseen.cfg` and their `.bak` copies). **Do not delete.** |
| `/ESP-DEBUG` | logs (`LOGS`) and screenshots (`SNAPS`) |
| `/ESP-STATES` | save states |
| `/ESP-BOOT` | boot screen files, `AUTORUN.TXT` |
| `/CASSETTE` | tapes recorded by CSAVE |
| `/PRINTS` | printer output (bit banger in PRINTER mode) |
| `/FLASHNOW` | firmware to install at the next start |

## 2. Keys

| Key | Action |
|---|---|
| **F12** | settings menu (Esc goes back) |
| **F10** | help (keys and options) |
| Esc / Delete / Backspace | CoCo BREAK / CLEAR / left arrow |
| Pause or `` ` `` | CoCo SHIFT+@ (pause a listing) |
| F1 / F2, Alt, Ctrl | CoCo 3 F1 / F2, ALT, CTRL |
| Caps Lock | upper / lower case |
| **Insert** (held) | turbo: maximum speed while held |
| **F5 / F8** | save / load state |
| **F9** | save a log + screenshot (for problem reports) |
| **Shift+F9** | same, with a short note typed on screen |
| Print Screen | screenshot only (BMP) |
| Ctrl+Alt+Delete | restart the ESP32 |

## 3. Settings menu (F12)

| Page | Items |
|---|---|
| **HARDWARE** | CPU (MC6809 / HD6309), EXPANSION RAM (512 KB, 2 MB, 4 MB), GIME CHIP VERSION (1986 / 1987), OVERCLOCK (OFF = cycle accurate, CPU, CPU + GIME) |
| **MONITOR** | COCO 3 MONITOR (RGB / composite), ARTIFACT COLORS (CoCo 2 modes), AUDIO OUTPUT (mono / stereo) |
| **DATA STORAGE** | DRIVE 0-3, DRIVEWIRE /X0 /X1, HARD DISK VHD 0 / 1, BOOT NITROS-9, SAVE / LOAD STATE, CASSETTE |
| **CART SLOT & COM** | MULTI-PAK, cartridge slots, BIT BANGER PORT, RS-232 settings |
| **USB DEVICES** | keyboard, joysticks (R / L), mouse, gamepad wizard, keyboard joystick, USB STATUS |
| **NETWORK** | Wi-Fi network, time zone, DriveWire server, web page |
| **SYSTEM** | boot screen, screen saver, AUTORUN, HARDWARE HEALTH, FIRMWARE UPDATE, PREVIOUS VERSION, REBOOT, HELP, TESTER LOG |

Settings are saved when you leave the menu. Changes marked *(reboot to apply)* (CPU, RAM, cartridges)
take effect after the restart the menu offers when you leave it.

## 4. Disks

**Floppies (DRIVE 0-3).** ENTER opens the file browser (.DSK). The bottom frame lists the files of
the selected disk. Delete ejects. Disk BASIC 2.1 starts as on a real CoCo 3.

**DriveWire (/X0, /X1)** and **virtual hard disks (VHD 0 / 1)** are disk images on the SD card for
NitrOS-9 and EOU. BOOT NITROS-9 starts NitrOS-9 from /X0. DriveWire can also go to a PC server over
Wi-Fi (section 8) or through the UART pins (bit banger).

**PC shared folder.** In the drive browser, press **P** to browse a folder of your PC (see
`coco_share.py` in the tools). The chosen .DSK is copied into memory and mounted; what the CoCo writes
is sent back to the PC file (a `.bak` copy is kept the first time).

## 5. Cassette

No cassette pins are used: tapes are files on the SD card. **F12 > DATA STORAGE > CASSETTE**:

| Item | Use |
|---|---|
| TAPE | ENTER: choose a tape (.CAS or .WAV; the bottom frame lists its programs). Delete: eject |
| POSITION | progress (%, seconds) |
| MOTOR | driven by the CoCo (ON during CLOAD / CSAVE) |
| REWIND | back to the start |
| RECORD NEW TAPE | blank tape `/CASSETTE/TAPEnnnn.CAS` |
| EJECT | remove the tape (ends a recording) |
| FAST LOAD | ON: the machine runs fast while the motor turns (a CLOAD takes a few seconds) |

- **Load**: choose the tape, then type `CLOAD`, `CLOADM`, or `OPEN "I",#-1,"NAME"`. The tape only
  moves while the motor is on. To read again: REWIND.
- **Save**: just type `CSAVE "NAME"` (or `CSAVEM`). A new tape is created in `/CASSETTE`
  automatically, also when a tape is in play mode (that tape is never erased). Recordings are
  always .CAS.
- **.WAV**: PCM 8 or 16 bits, mono or stereo, 4 to 96 kHz (recordings of a real CoCo or made on a
  PC) — playback only.
- `AUDIO ON` in BASIC lets you hear the tape while it loads.
- Load at normal CPU speed (0.89 MHz), as on a real CoCo.

## 6. Cartridges and serial ports

**F12 > CART SLOT & COM.** With MULTI-PAK on, four slots and the power-on switch are available.
A slot can hold: the FD-502 disk controller, a ROM cartridge (.ROM / .CCC, 32 KB or bank switched up
to 512 KB, AUTOSTART option), Orchestra-90, Game Master Cartridge, CoCo PSG (YM2149) or the RS-232
Pak (6551). Changes apply after a restart.

**RS-232 Pak.** RS-232 LINK: UART pins 41 / 42, loopback, or the Wi-Fi modem. Speed: as set by the
software, or fixed (RS-232 BAUD RATE).

**Bit banger (serial port of the CoCo):** OFF, PRINTER -> SD (`LLIST`, `PRINT #-2` go to
`/PRINTS/PRTnnnn.TXT`), DRIVEWIRE (/x0-/x3), UART 41/42 (3.3 V TTL, e.g. a PC DriveWire server or a
terminal), DRIVEWIRE Wi-Fi, MODEM Wi-Fi.

## 7. USB devices

Keyboard, mouse and gamepads can be plugged in directly or through a hub.

- **Joysticks R / L**: choose a gamepad, the mouse, the keyboard (arrows, numeric keypad or W A S D)
  or nothing for each CoCo joystick port. The gamepad wizard sets the buttons and axes of an
  unknown gamepad.
- **Mouse**: on the right or left port, low resolution or **Hi-Res interface** (OS-9 Multi-Vue,
  GShell in high-resolution mode), with a sensitivity setting.
- **USB channels.** The ESP32-S3 USB host has **8 channels**. Each device takes 1, plus 1 per
  interface in use, plus 1 per hub:

| Device | Channels |
|---|---|
| simple keyboard, mouse, USB gamepad | 2 each |
| Xbox-type gamepad (8BitDo in X mode) | 2 or 3 |
| wireless keyboard + mouse receiver | 3 |
| hub | 2 |
| keyboard with built-in hub and display (e.g. Logitech G15) | 5 |

Example that works: simple keyboard + hub + mouse + gamepad = 8 / 8. The keyboard always gets a
channel first. A device without a channel shows **NO CHANNEL** in USB DEVICES and starts as soon as
a channel is freed. Tip: a small simple hub and a simple keyboard leave the most room.

8BitDo gamepads: power on with Power + B.

## 8. Network

**F12 > NETWORK**: Wi-Fi ON, choose the network and type the password, set the time zone (the clock
comes from NTP).

**DriveWire server on a PC**: DRIVEWIRE SERVER = address of the PC running a DriveWire 4 server.

**Web page (SD card)**: WEB PAGE = ON. Open the ADDRESS shown (e.g. `http://192.168.1.20/`) in a
browser on the same network and type the 6-digit PIN. You can browse, download and upload files
(drag and drop), create folders, rename, delete, list the contents of .DSK images and view BASIC
programs as text. Mounted disks and `/ESP-CONFIG` are protected. While a program runs, transfers are
slowed down (about 130 KB/s) so that the game is not disturbed; open the F12 menu for full speed.
The page is plain http: use it on a trusted network. Chrome may hold back downloads of .CAS files
("not secure"): choose *Keep*.

**Hayes modem (telnet, BBS)**: set the RS-232 LINK or the bit banger to MODEM (Wi-Fi), start a
terminal program on the CoCo and dial:

```
ATDT bbs.example.com:23     dial (port 23 by default)
+++                          back to commands (1 s of silence before and after)
ATO / ATH                    back online / hang up
AT&Z0=bbs.example.com:23     phone book entry 0-3, AT&W saves, ATDS0 dials it
ATS0=1                       auto-answer incoming calls (port 6400, AT*Pn to change)
ATI                          identification and Wi-Fi status
```

Other commands: ATE, ATV, ATQ, AT&F, AT&V, AT&C, AT&D, ATNET0 (raw TCP) / ATNET1 (telnet), A/.

**PC shared folder**: run `coco_share.py` on the PC (or `coco_share.bat` on Windows), note the PIN it
shows, then press P in the drive browser.

## 9. Boot screen, screen saver, AUTORUN

**F12 > SYSTEM.**

- **BOOT SCREEN**: OFF, LOGO + MUSIC, or SD CARD (`/ESP-BOOT`: `SPLASH.CCV` film, else animated
  `*.GIF`, else a `SLIDES/*.BMP` slide show; `SPLASH.WAV` music). ENDS WITH: a key or button, or after
  5 / 10 / 30 s. Esc also skips AUTORUN for this start. `make_splash.py` (tools) converts a GIF or a
  folder of images into `SPLASH.CCV` and makes slides and music files.
- **SCREEN SAVER**: after 5-30 min without any action, the emulation pauses and a black screen, the
  logo or the SD card show appears. Any key, button or mouse movement resumes exactly where the
  CoCo was.
- **AUTORUN**: at start-up the ESP32 types a command in BASIC, as on the keyboard — COMMAND (one
  line, e.g. `RUN"MENU"`) or FILE (`/ESP-BOOT/AUTORUN.TXT`, one command per line, `#` = comment,
  32 lines). Each line waits for the BASIC prompt. A key pressed during typing cancels the rest.
- **HARDWARE HEALTH**: load of each core, emulated CPU speed, chip temperature, memories, SD card,
  Wi-Fi, USB, uptime.

## 10. Save states

**F12 > DATA STORAGE > SAVE / LOAD STATE**: 4 slots per software (the name comes from the disk or
cartridge in use). F5 saves to the current slot, F8 loads the newest. A state needs the same CPU,
RAM size and cartridges. DELETE ALL STATES OF THIS SOFTWARE erases them.

## 11. Firmware update

- **F12 > SYSTEM > FIRMWARE UPDATE**: choose a `.FLH` file at the root of the SD card.
- Or copy the `.FLH` / `.BIN` file into `/FLASHNOW` (also through the web page) and restart: it is
  installed at start-up (black screen ~20 s — **do not cut the power**).
- **PREVIOUS VERSION** starts the firmware kept in the other flash slot.
- New versions are published on the GitHub *Releases* page. `make_flh.py` (tools) makes a `.FLH` from
  a `.bin` you compiled yourself.

## 12. Known software notes

| Software | Note |
|---|---|
| Pop Star Pilot | a graphic glitch can appear (also in other emulators: timing in the game itself); changing the GIME version during the game helps |
| Contra | at GAME OVER, choose C or E **on the keyboard** |
| Gunstar | needs the HD6309 CPU |
| GIMEX | the GIMEX video expansion is not emulated |
| Cassette | load at 0.89 MHz; the double-speed POKE prevents loading, as on a real CoCo |

Hot-plugging a gamepad while Wi-Fi is on can fail (not enough internal memory): plug it in before
starting, or restart.

## 13. Reporting a problem

Press **F9** while the problem is on screen (**Shift+F9**: type a note of up to 3 lines, Enter = OK,
Esc = cancel). Send `LOGnnnn.TXT` and `LOGnnnn.BMP` from `/ESP-DEBUG/LOGS`, the name of the disk or
program and what you did. After a crash, a log is saved automatically at the next start.
Serial port (921 600 baud): **W** full status, **U** USB details, **J** joysticks.
