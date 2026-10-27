# ESP32-CoCo3-PRO 1.0 — Guide de l'utilisateur
# Auteur : Luc Dion 2026-10-26.

L'ESP32-CoCo3-PRO émule un Tandy Color Computer 3 sur une carte ESP32-S3. Tout ce que le CoCo utilise
(disquettes, disques durs, bandes, cartouches) est un fichier sur la carte SD.

Sommaire : [1. Carte SD](#1-carte-sd) · [2. Touches](#2-touches) · [3. Menu des réglages (F12)](#3-menu-des-réglages-f12) ·
[4. Disques](#4-disques) · [5. Cassette](#5-cassette) · [6. Cartouches et ports série](#6-cartouches-et-ports-série) ·
[7. Périphériques USB](#7-périphériques-usb) · [8. Réseau](#8-réseau) · [9. Écran d'accueil, écran de veille, AUTORUN](#9-écran-daccueil-écran-de-veille-autorun) ·
[10. Sauvegardes d'état](#10-sauvegardes-détat) · [11. Mise à jour du firmware](#11-mise-à-jour-du-firmware) ·
[12. Logiciels : remarques](#12-logiciels--remarques) · [13. Signaler un problème](#13-signaler-un-problème)

## 1. Carte SD

FAT32, de 4 à 32 Go. Rangez vos fichiers dans les dossiers de votre choix. Dossiers créés par
l'émulateur :

| Dossier | Rôle |
|---|---|
| `/ESP-CONFIG` | réglages (`config.cfg`, `joyconfig.cfg`, `usbseen.cfg` et leurs copies `.bak`). **Ne pas effacer.** |
| `/ESP-DEBUG` | journaux (`LOGS`) et captures d'écran (`SNAPS`) |
| `/ESP-STATES` | sauvegardes d'état |
| `/ESP-BOOT` | fichiers de l'écran d'accueil, `AUTORUN.TXT` |
| `/CASSETTE` | bandes enregistrées par CSAVE |
| `/PRINTS` | sortie imprimante (bit banger en mode PRINTER) |
| `/FLASHNOW` | firmware à installer au prochain démarrage |

## 2. Touches

| Touche | Action |
|---|---|
| **F12** | menu des réglages (Échap pour revenir) |
| **F10** | aide (touches et options) |
| Échap / Suppr / Retour arrière | BREAK / CLEAR / flèche gauche du CoCo |
| Pause ou `` ` `` | SHIFT+@ du CoCo (pause d'un listing) |
| F1 / F2, Alt, Ctrl | F1 / F2, ALT, CTRL du CoCo 3 |
| Verr. Maj. | majuscules / minuscules |
| **Inser** (maintenue) | turbo : vitesse maximale tant qu'elle est enfoncée |
| **F5 / F8** | sauvegarder / recharger l'état |
| **F9** | journal + capture d'écran (pour signaler un problème) |
| **Maj+F9** | idem, avec une courte note tapée à l'écran |
| Impr. écran | capture d'écran seule (BMP) |
| Ctrl+Alt+Suppr | redémarrage de l'ESP32 |

## 3. Menu des réglages (F12)

| Page | Contenu |
|---|---|
| **HARDWARE** | CPU (MC6809 / HD6309), EXPANSION RAM (512 Ko, 2 Mo, 4 Mo), GIME CHIP VERSION (1986 / 1987), OVERCLOCK (OFF = cycle exact, CPU, CPU + GIME) |
| **MONITOR** | COCO 3 MONITOR (RGB / composite), ARTIFACT COLORS (modes CoCo 2), AUDIO OUTPUT (mono / stéréo) |
| **DATA STORAGE** | DRIVE 0-3, DRIVEWIRE /X0 /X1, HARD DISK VHD 0 / 1, BOOT NITROS-9, SAVE / LOAD STATE, CASSETTE |
| **CART SLOT & COM** | MULTI-PAK, emplacements de cartouches, BIT BANGER PORT, réglages RS-232 |
| **USB DEVICES** | clavier, manettes (R / L), souris, assistant de manette, manette au clavier, USB STATUS |
| **NETWORK** | réseau Wi-Fi, fuseau horaire, serveur DriveWire, page web |
| **SYSTEM** | écran d'accueil, écran de veille, AUTORUN, HARDWARE HEALTH, FIRMWARE UPDATE, PREVIOUS VERSION, REBOOT, HELP, TESTER LOG |

Les réglages sont sauvegardés à la sortie du menu. Les choix marqués *(reboot to apply)* (CPU, RAM,
cartouches) s'appliquent après le redémarrage proposé à la sortie du menu.

## 4. Disques

**Disquettes (DRIVE 0-3).** ENTRÉE ouvre le navigateur (.DSK). Le cadre du bas liste les fichiers du
disque sélectionné. Suppr éjecte. Le Disk BASIC 2.1 démarre comme sur un vrai CoCo 3.

**DriveWire (/X0, /X1)** et **disques durs virtuels (VHD 0 / 1)** : images de disques sur la carte SD
pour NitrOS-9 et EOU. BOOT NITROS-9 démarre NitrOS-9 depuis /X0. DriveWire peut aussi passer par un
serveur sur PC, par le Wi-Fi (section 8) ou par les broches UART (bit banger).

**Dossier partagé d'un PC.** Dans le navigateur des lecteurs, la touche **P** parcourt un dossier du PC
(voir `coco_share.py` dans les outils). Le .DSK choisi est copié en mémoire et monté ; ce que le CoCo
écrit est renvoyé dans le fichier du PC (une copie `.bak` est gardée la première fois).

## 5. Cassette

Aucune broche cassette n'est utilisée : les bandes sont des fichiers sur la carte SD.
**F12 > DATA STORAGE > CASSETTE** :

| Ligne | Rôle |
|---|---|
| TAPE | ENTRÉE : choisir une bande (.CAS ou .WAV ; le cadre du bas liste ses programmes). Suppr : éjecter |
| POSITION | avancement (%, secondes) |
| MOTOR | commandé par le CoCo (ON pendant CLOAD / CSAVE) |
| REWIND | retour au début |
| RECORD NEW TAPE | bande vierge `/CASSETTE/TAPEnnnn.CAS` |
| EJECT | retirer la bande (termine un enregistrement) |
| FAST LOAD | ON : la machine va vite tant que le moteur tourne (un CLOAD prend quelques secondes) |

- **Charger** : choisir la bande, puis taper `CLOAD`, `CLOADM` ou `OPEN "I",#-1,"NOM"`. La bande
  n'avance que moteur en marche. Pour relire : REWIND.
- **Enregistrer** : taper simplement `CSAVE "NOM"` (ou `CSAVEM`). Une nouvelle bande est créée d'elle-même
  dans `/CASSETTE`, même si une bande est en lecture (celle-ci n'est jamais effacée). Les
  enregistrements sont toujours en .CAS.
- **.WAV** : PCM 8 ou 16 bits, mono ou stéréo, de 4 à 96 kHz (enregistrement d'un vrai CoCo ou fait
  sur un PC) — lecture seulement.
- `AUDIO ON` en BASIC fait entendre la bande pendant le chargement.
- Charger à la vitesse normale du CPU (0,89 MHz), comme sur un vrai CoCo.

## 6. Cartouches et ports série

**F12 > CART SLOT & COM.** Avec MULTI-PAK en marche, quatre emplacements et le sélecteur de démarrage
sont disponibles. Un emplacement peut recevoir : le contrôleur de disquettes FD-502, une cartouche
ROM (.ROM / .CCC, 32 Ko ou jusqu'à 512 Ko par banques, option AUTOSTART), l'Orchestra-90, la Game
Master Cartridge, la CoCo PSG (YM2149) ou le RS-232 Pak (6551). Les changements s'appliquent après un
redémarrage.

**RS-232 Pak.** RS-232 LINK : broches UART 41 / 42, boucle, ou modem Wi-Fi. Vitesse : celle programmée
par le logiciel, ou fixe (RS-232 BAUD RATE).

**Bit banger (port série du CoCo)** : OFF, PRINTER -> SD (`LLIST`, `PRINT #-2` vont dans
`/PRINTS/PRTnnnn.TXT`), DRIVEWIRE (/x0-/x3), UART 41/42 (TTL 3,3 V, par exemple un serveur DriveWire
sur PC ou un terminal), DRIVEWIRE Wi-Fi, MODEM Wi-Fi.

## 7. Périphériques USB

Clavier, souris et manettes se branchent directement ou par un hub.

- **Manettes R / L** : pour chaque port manette du CoCo, choisir une manette, la souris, le clavier
  (flèches, pavé numérique ou W A S D) ou rien. L'assistant règle les boutons et les axes d'une manette
  inconnue.
- **Souris** : sur le port droit ou gauche, basse résolution ou **interface Hi-Res** (OS-9 Multi-Vue,
  GShell en haute résolution), avec un réglage de sensibilité.
- **Canaux USB.** L'hôte USB de l'ESP32-S3 a **8 canaux**. Chaque appareil en prend 1, plus 1 par
  interface utilisée, plus 1 par hub :

| Appareil | Canaux |
|---|---|
| clavier simple, souris, manette USB | 2 chacun |
| manette de type Xbox (8BitDo en mode X) | 2 ou 3 |
| récepteur sans fil clavier + souris | 3 |
| hub | 2 |
| clavier avec hub et écran intégrés (ex. Logitech G15) | 5 |

Exemple qui fonctionne : clavier simple + hub + souris + manette = 8 / 8. Le clavier passe toujours en
premier. Un appareil sans canal est affiché **NO CHANNEL** dans USB DEVICES et démarre dès qu'un canal se
libère. Conseil : un petit hub simple et un clavier simple laissent le plus de place.

Manettes 8BitDo : les allumer avec Power + B.

## 8. Réseau

**F12 > NETWORK** : Wi-Fi ON, choisir le réseau et taper le mot de passe, régler le fuseau horaire
(l'heure vient de NTP).

**Serveur DriveWire sur PC** : DRIVEWIRE SERVER = adresse du PC où tourne un serveur DriveWire 4.

**Page web (carte SD)** : WEB PAGE = ON. Ouvrir l'adresse affichée (ADDRESS, ex. `http://192.168.1.20/`)
dans un navigateur du même réseau et taper le PIN à 6 chiffres. On peut parcourir, télécharger,
envoyer des fichiers (glisser-déposer), créer des dossiers, renommer, effacer, lister le contenu des
images .DSK et voir les programmes BASIC en texte. Les disques montés et `/ESP-CONFIG` sont protégés.
Pendant qu'un programme tourne, les transferts sont ralentis (environ 130 Ko/s) pour ne pas gêner le
jeu ; ouvrir le menu F12 pour la pleine vitesse. La page est en http simple : à utiliser sur un réseau
de confiance. Chrome peut retenir le téléchargement des .CAS (« non sécurisé ») : choisir *Conserver*.

**Modem Hayes (telnet, BBS)** : mettre RS-232 LINK ou le bit banger sur MODEM (Wi-Fi), lancer un
logiciel de terminal sur le CoCo et composer :

```
ATDT bbs.exemple.com:23     composer (port 23 par défaut)
+++                          retour aux commandes (1 s de silence avant et après)
ATO / ATH                    retour en ligne / raccrocher
AT&Z0=bbs.exemple.com:23     carnet 0-3, AT&W sauvegarde, ATDS0 compose l'entrée
ATS0=1                       réponse automatique aux appels (port 6400, AT*Pn pour changer)
ATI                          identification et état du Wi-Fi
```

Autres commandes : ATE, ATV, ATQ, AT&F, AT&V, AT&C, AT&D, ATNET0 (TCP brut) / ATNET1 (telnet), A/.

**Dossier partagé d'un PC** : lancer `coco_share.py` sur le PC (ou `coco_share.bat` sous Windows),
noter le PIN affiché, puis la touche P dans le navigateur des lecteurs.

## 9. Écran d'accueil, écran de veille, AUTORUN

**F12 > SYSTEM.**

- **BOOT SCREEN** : OFF, LOGO + MUSIC, ou SD CARD (`/ESP-BOOT` : film `SPLASH.CCV`, sinon GIF animés
  `*.GIF`, sinon diaporama `SLIDES/*.BMP` ; musique `SPLASH.WAV`). ENDS WITH : une touche ou un
  bouton, ou après 5 / 10 / 30 s. Échap saute aussi l'AUTORUN pour ce démarrage. `make_splash.py`
  (outils) convertit un GIF ou un dossier d'images en `SPLASH.CCV` et prépare diaporama et musique.
- **SCREEN SAVER** : après 5 à 30 min sans action, l'émulation se met en pause et un écran noir, le
  logo ou le contenu de la carte SD s'affiche. Une touche, un bouton ou la souris qui bouge reprend
  exactement où était le CoCo.
- **AUTORUN** : au démarrage, l'ESP32 tape une commande dans le BASIC, comme au clavier — COMMAND
  (une ligne, ex. `RUN"MENU"`) ou FILE (`/ESP-BOOT/AUTORUN.TXT`, une commande par ligne, `#` =
  commentaire, 32 lignes). Chaque ligne attend l'invite du BASIC. Une touche pendant la frappe annule
  la suite.
- **HARDWARE HEALTH** : charge de chaque coeur, vitesse du CPU émulé, température de la puce, mémoires,
  carte SD, Wi-Fi, USB, temps de marche.

## 10. Sauvegardes d'état

**F12 > DATA STORAGE > SAVE / LOAD STATE** : 4 emplacements par logiciel (le nom vient du disque ou de
la cartouche en service). F5 sauvegarde dans l'emplacement courant, F8 recharge le plus récent. Un état
demande le même CPU, la même taille de RAM et les mêmes cartouches. DELETE ALL STATES OF THIS SOFTWARE
les efface.

## 11. Mise à jour du firmware

- **F12 > SYSTEM > FIRMWARE UPDATE** : choisir un fichier `.FLH` à la racine de la carte SD.
- Ou copier le fichier `.FLH` / `.BIN` dans `/FLASHNOW` (aussi par la page web) et redémarrer : il est
  installé au démarrage (écran noir ~20 s — **ne pas couper le courant**).
- **PREVIOUS VERSION** démarre le firmware gardé dans l'autre emplacement de la flash.
- Les nouvelles versions sont publiées sur la page *Releases* du dépôt GitHub. `make_flh.py` (outils)
  fabrique un `.FLH` à partir d'un `.bin` compilé soi-même.

## 12. Logiciels : remarques

| Logiciel | Remarque |
|---|---|
| Pop Star Pilot | un défaut graphique peut apparaître (aussi dans d'autres émulateurs : minutage du jeu lui-même) ; changer la version du GIME pendant la partie aide |
| Contra | à GAME OVER, choisir C ou E **au clavier** |
| Gunstar | demande le CPU HD6309 |
| GIMEX | l'extension vidéo GIMEX n'est pas émulée |
| Cassette | charger à 0,89 MHz ; le POKE de double vitesse empêche le chargement, comme sur un vrai CoCo |

Brancher une manette à chaud pendant que le Wi-Fi fonctionne peut échouer (mémoire interne
insuffisante) : la brancher avant le démarrage, ou redémarrer.

## 13. Signaler un problème

Appuyer sur **F9** pendant que le problème est à l'écran (**Maj+F9** : note de 3 lignes au plus,
Entrée = OK, Échap = annuler). Envoyer `LOGnnnn.TXT` et `LOGnnnn.BMP` du dossier `/ESP-DEBUG/LOGS`, le
nom du disque ou du programme et ce que vous avez fait. Après un plantage, un journal est enregistré
automatiquement au démarrage suivant. Port série (921 600 bauds) : **W** état complet, **U** détail USB,
**J** manettes.
