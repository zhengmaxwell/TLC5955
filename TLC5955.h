/*
   TLC5955 Control Library
   Used to control the TI TLC5955 LED driver chip
   Zack Phillips - zkphil@berkeley.edu
   Product Page: http://www.ti.com/product/tlc5955

   Copyright (c) 2015, Zachary F. Phillips
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
 * Neither the name of Zack Phillips / UC Berkeley nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL Z. PHILLIPS BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TLC5955_H
#define TLC5955_H

#include <stdint.h>
#include <SPI.h>

/* Bit Quantities (Change to match other TLC driver chips) */
#define GS_BITS 16
#define MC_BITS 3
#define BC_BITS 7
#define DC_BITS 7
#define FC_BITS 5
#define CONTROL_ZERO_BITS 389   /* Bits required for correct control reg size */
#define TOTAL_REGISTER_SIZE 76
#define LATCH_DELAY 1
#define CONTROL_WRITE_COUNT 2
#define CONTROL_MODE_ON 1
#define CONTROL_MODE_OFF 0

// LED Current OUTPUT
static const float LED_CURRENT_AMPS = 0.020;

// Line ending for serial output
static const char LINE_ENDING[] = "\n";

class TLC5955
{
public:

/* Initialization */
void init(uint8_t gslat, uint8_t spi_mosi, uint8_t spi_clk, uint8_t gsclk);
void deallocate();

/* Setting individual LED intensities */
void setAllLed(uint16_t gsvalue);
void setAllLedRgb(uint16_t red, uint16_t green, uint16_t blue);
void setLed(uint16_t led_number, uint16_t red, uint16_t green, uint16_t blue);
void setLed(uint16_t led_number, uint16_t rgb);
void setLedAppend(uint16_t led_number, uint16_t red, uint16_t green, uint16_t blue);
void setChannel(uint16_t channel_number, uint16_t value);

/* Get LED Intensities */
uint16_t getChannelValue(uint16_t channelNum, int color_channel_index);
void getLedCurrents(float* currents, uint16_t* gs);

/* Control Mode Parameters */
void setMaxCurrent(uint8_t mc);
void setMaxCurrent(uint8_t mcr, uint8_t mcg, uint8_t mcb);
void getMaxCurrent(uint8_t* maxCurrent);
void setBrightnessControl(uint8_t bc);
void setBrightnessControl(uint8_t bcr, uint8_t bcg, uint8_t bcb);
void getBrightnessControl(uint8_t* brightnessControl);
void setDotCorrection(uint8_t dc);
void setDotCorrection(uint8_t dcr, uint8_t dcg, uint8_t dcb);
void getDotCorrection(uint8_t* dotCorrection);
void setFunctionData(bool DSPRPT, bool TMGRST, bool RFRESH, bool ESPWM, bool LSDVLT);
void setRgbPinOrder(uint8_t rPos, uint8_t grPos, uint8_t bPos);
void setPinOrderSingle(uint16_t channel, uint8_t color_channel_index, uint8_t position);
void setRgbPinOrderSingle(uint16_t channel, uint8_t rPos, uint8_t grPos, uint8_t bPos);

/* Sending data to device (Updating, flushing, latching) */
void setBuffer(uint8_t bit);
void setControlModeBit(bool isControlMode);
void flushBuffer();
void updateLeds();
void clearLeds();
void latch();
void updateControl();
void setSpiBaudRate(uint32_t new_baud_rate);
uint32_t getSpiBaudRate();

void setGsclkFreq(uint32_t new_gsclk_frequency);
uint32_t getGsclkFreq();

/* Diagnostic Methods */
void printByte(uint8_t myByte);

static const uint8_t _tlc_count; // This
static const uint8_t COLOR_CHANNEL_COUNT = 3;
static const uint8_t LEDS_PER_CHIP = 16;
static bool enforce_max_current;
static float max_current_amps;

static uint8_t _rgb_order[][LEDS_PER_CHIP][COLOR_CHANNEL_COUNT];
static uint16_t _grayscale_data[][LEDS_PER_CHIP][COLOR_CHANNEL_COUNT];

uint8_t rgb_order_default[3] = {0, 1, 2};

  // Analog Control Values
  // https://www.ti.com/lit/ds/symlink/tlc5955.pdf?ts=1636036806528&ref_url=https%253A%252F%252Fwww.google.com%252F
  // Page 8 (Table 8)
const float maxCurrentValues_mA[8] = {3.2, 8.0, 11.2, 15.9, 19.1, 23.9, 27.5, 31.9};

private:
  int debug = 0;
  uint8_t _gslat;
  uint8_t _spi_mosi;
  uint8_t _spi_clk;
  uint8_t _gsclk;

  uint8_t _function_data;
  // These are 3 long arrays, one for each color channel
  // red, green and blue
  uint8_t _MC[3];
  uint8_t _BC[3];
  uint8_t _DC[3];

  /* SPI */
  uint8_t _buffer;
  int8_t _buffer_count = 7;
  uint32_t spi_baud_rate = 1000000;
  uint32_t gsclk_frequency = 2500000;

  SPISettings mSettings;
};

#endif
