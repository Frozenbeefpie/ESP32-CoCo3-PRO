// ESP32-CoCo3-PRO 1.0 — émulateur Tandy Color Computer 3 sur ESP32-S3 (licence MIT).
// Auteur : Luc Dion 2026-10-26.

// Sortie VGA (LCD_CAM de l'ESP32-S3).

#ifndef VGA_H
#define VGA_H

#include "PinConfig.h"
#include "Mode.h"
#include "DMAVideoBuffer.h"

class VGA
{
	public:
	Mode mode;
	int bufferCount;
	int bits;
	PinConfig pins;
	int backBuffer;
	DMAVideoBuffer *dmaBuffer;
	bool usePsram;
	int dmaChannel;

	public:
	VGA();
	~VGA();
	bool init(const PinConfig pins, const Mode mode, int bits);
	bool Reinit(const PinConfig pins, const Mode mode, int bits);

	// Bascule rapide entre deux modes au MÊME balayage vertical et à la même profondeur (ex. 320x240 4:3
	// <-> 640 GIME)
	bool prepareAlternate(const Mode altMode);
	bool hasAlternate(const Mode m) const;
	// waitSync (optionnel) : appelée juste avant l'arrêt ; doit rendre la main au DÉBUT d'une impulsion
	// VSYNC. La nouvelle trame du LCD_CAM commence alors pile là où l'ancienne l'aurait fait
	bool switchToAlternate(void (*waitSync)(void) = 0);
	Mode altMode;
	DMAVideoBuffer *altBuffer;
	void deinit();
	bool start();
	bool stop();
	bool show();
	void clear(int rgb = 0);
	void dot(int x, int y, uint8_t r, uint8_t g, uint8_t b);
	void dot(int x, int y, int rgb);
	void dotdit(int x, int y, uint8_t r, uint8_t g, uint8_t b);
	uint8_t getPixel(int x, int y);
	int rgb(uint8_t r, uint8_t g, uint8_t b);
	// void drawLineFromMemory8(int y, const uint8_t* memStart)
	void drawLineFromMemory8(int x, int y, const uint8_t* memStart);
	void drawLineFromMemory16(int x, int y, const uint8_t* memStart);
	void drawLineFromMemory256(int x, int y, const uint8_t* memStart);

	protected:
	void attachPinToSignal(int pin, int signal);
	void programTimings(const Mode &mode);

};

#endif // VGA_h
