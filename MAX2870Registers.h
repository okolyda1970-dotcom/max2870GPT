#pragma once

#include <stdint.h>

// Настройки MAX2870.
// Здесь можно менять параметры без изменения main.cpp.

struct MAX2870Settings {
  double referenceMHz = 25.0;
  uint16_t referenceDivider = 2;
  uint8_t ldp = 0;
  uint16_t modulus = 4095;
  uint16_t bandSelect = 500;

  uint8_t chargePump = 4;   // CP: 0..15
  uint8_t muxOut = 6;       // MUXOUT: 0..15
  uint8_t rfDivider = 7;    // DIVA: 0..7

  uint8_t outputPower = 0;  // APWR: 0..3
  bool rfOutputEnable = false;
bool positivePhaseDetector = true;
bool referenceDoubler = true;
bool retune = false;
  uint8_t phase = 1;
  
uint8_t ldMode = 1; // R5[23:22]: 0=LOW, 1=Digital, 2=Analog, 3=HIGH

};

bool buildMAX2870Registers(
  double freqMHz,
  const MAX2870Settings& settings,
  uint32_t registers[6],
  double& actualFreqMHz
);
