# ESP32-CoCo3-PRO

**A Tandy Color Computer 3 emulator running bare-metal on an ESP32-S3** (module ESP32-S3-WROOM-1 N8R8),
with VGA output, USB keyboard / mouse / gamepads, SD card storage and Wi-Fi.

Version **1.0** — October 2026 — by **Luc Dion**, inspired by ESP32-CoCo2 by Cedric Beaudoin.

*Version française : [docs/GUIDE_FR.md](docs/GUIDE_FR.md).*

## Features

- **CoCo 3 hardware**: MC6809 or HD6309 CPU (0.89 / 1.79 MHz, overclock), GIME 1986 / 1987 (MMU,
  timer, interrupts, all text and graphics modes, 64-color palette, RGB or composite monitor with
  artifact colors), 512 KB, 2 MB or 4 MB RAM, PIAs, 6-bit DAC and 1-bit sound.
- **Floppy disks**: FD-502 controller, Disk BASIC, 4 drives (.DSK images on the SD card).
- **DriveWire 4** built in (Becker port) and **virtual hard disks** (.VHD): NitrOS-9, EOU.
- **Cassette**: plays .CAS and .WAV tapes, records .CAS (CLOAD, CLOADM, CSAVE, CSAVEM), fast load.
- **Cartridge slot and Multi-Pak**: ROM cartridges (.ROM / .CCC, bank switching), Orchestra-90,
  Game Master Cartridge (SN76489), CoCo PSG (YM2149), RS-232 Pak (6551), CoCo SDC.
- **USB**: keyboard, mouse (low-res or Hi-Res interface), USB and Xbox-type gamepads, hubs.
- **Wi-Fi**: NTP clock, DriveWire server on a PC, Hayes modem (telnet / BBS), web page to manage the
  SD card, disk images shared from a PC folder.
- **Save states** (4 slots per software), boot screen, screen saver, AUTORUN, hardware health page.
- **Problem reports**: F9 saves a dated log and a screenshot to the SD card.

## Installing the firmware

Download `ESP32-CoCo3-PRO_v1.0.FLH` (or `.BIN`) from the [Releases](../../releases) page, copy it into
the `/FLASHNOW` folder of the SD card and power the board: it is installed at start-up (black screen
for about 20 s — do not cut the power). The previous version stays in the other flash slot
(F12 > SYSTEM > PREVIOUS VERSION).

## Quick start

1. Format a 4-32 GB SD card in **FAT32** and copy your disk images (.DSK), tapes (.CAS / .WAV) and
   cartridges (.ROM / .CCC) into any folders.
2. Insert the card, connect a VGA monitor and a USB keyboard, then power the board.
3. Press **F12** for the settings menu and **F10** for help.

The emulator creates its own folders on the card: `/ESP-CONFIG` (settings — never delete it),
`/ESP-DEBUG` (logs and screenshots), `/ESP-STATES` (save states), `/ESP-BOOT` (boot screen,
AUTORUN), `/CASSETTE` (recorded tapes), `/FLASHNOW` (firmware updates).

## Documentation

| File | Contents |
|---|---|
| [docs/GUIDE_EN.md](docs/GUIDE_EN.md) | User guide (keys, menu, disks, cassette, cartridges, USB, network, updates) |
| [docs/GUIDE_FR.md](docs/GUIDE_FR.md) | Guide de l'utilisateur en français |
| [tools/](tools/) | `ESP32-CoCo3-PRO_tools.zip`: PC utilities (shared folder, disk tools, boot screen maker, .FLH maker) |
| [disks/](disks/) | Test disks and an example tape |

## Building from source

- **Arduino IDE** with the **esp32 core 3.2.1** (Espressif). Other core versions are not tested.
- Board **ESP32S3 Dev Module**: CPU 240 MHz, Flash 8 MB **QIO 120 MHz**, **PSRAM OPI**,
  USB Mode *Hardware CDC and JTAG*, USB CDC On Boot *Disabled*, Arduino and events on core 1,
  partition scheme from the sketch's `partitions.csv` (two 3.2 MB application slots).
- Open `firmware/ESP32-CoCo3-PRO/ESP32-CoCo3-PRO.ino` (the program itself is in `main.cpp`).
- Libraries: **ESP32-S3 VGA** (bitluni) with the video modes used by this project, and
  **Adafruit GFX**. The USB host library (EspUsbHost 2.7.0, MIT) is included in `src/EspUsbHost`,
  with the changes described in `src/EspUsbHost/LISEZMOI_CoCo3.txt`.
- **Not included**: `ROMS_Source.h`. The CoCo ROMs and fonts are copyrighted and are **not
  distributed**. The file defines `coco3[32768]`, `extbas11[8192]`, `bas13[8192]`, `disk11[8192]`,
  `disk21[8192]`, `font_gimeChip[128 * 12]` and `cc2_VDG_MAP[128 * 96]` as `const uint8_t` arrays,
  made from dumps of ROMs you own.
- `tools/make_flh.py` turns the compiled `.bin` into a `.FLH` file (checksum checked before
  flashing).

Serial port: 921 600 baud. Commands **W** (full status), **U** (USB details), **J** (joysticks).

## Reporting a problem

Press **F9** while the problem is on screen (**Shift+F9** lets you type a short note). Send the
`LOGnnnn.TXT` and `LOGnnnn.BMP` files from `/ESP-DEBUG/LOGS`, the name of the disk or program, and
what you did. After a crash, a log is saved automatically at the next start.

## Credits and licenses

ESP32-CoCo3-PRO is released under the [MIT license](LICENSE).

- ESP32-CoCo3 Emulator: Luc Dion, inspired by the ESP32-CoCo2 emulator by Cedric Beaudoin.
- MC6809 core: elmerucr (MIT license); HD6309 extension written for this core from the MAME hd6309
  behaviour (BSD-3-Clause).
- Parts ported from **MAME** (BSD-3-Clause): SN76489 (sn76496.cpp), 6551 ACIA (mos6551.cpp,
  coco_rs232.cpp), Hi-Res joystick timing.
- **EspUsbHost** 2.7.0, Tanaka Masayuki (MIT license), modified.
- Tandy, Color Computer and CoCo are trademarks of their respective owners. This project is not
  affiliated with them.
