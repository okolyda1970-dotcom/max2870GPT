#include "MAX2870Registers.h"
#include <math.h>

bool buildMAX2870Registers(
  double freqMHz,
  const MAX2870Settings& s,
  uint32_t r[6],
  double& actualFreqMHz
) {
  if (!isfinite(freqMHz) ||
      freqMHz < 23.5 || freqMHz > 6000.0) {
    return false;
  }

if (s.referenceMHz <= 0 ||
    s.modulus < 2 || s.modulus > 4095 ||
    s.chargePump > 15 ||
    s.muxOut > 7 ||
    s.rfDivider > 7 ||
    s.outputPower > 3 ||
    s.ldMode > 3 ||
    s.phase > 4095) {
  return false;
}

  // Выбор делителя VCO.
  uint8_t divCode = 0;
  double divider = 1.0;

  while (freqMHz * divider < 3000.0 && divCode < 7) {
    divider *= 2.0;
    ++divCode;
  }

  if (divCode != s.rfDivider) {
    return false;
  }

  double vcoMHz = freqMHz * divider;

  if (vcoMHz < 3000.0 || vcoMHz > 6000.0) {
    return false;
  }


// Расчёт N и FRAC.
double pfdMHz = s.referenceMHz *
    (s.referenceDoubler ? 2.0 : 1.0) /
    s.referenceDivider;
double nValue = vcoMHz / pfdMHz;

uint32_t n = (uint32_t)nValue;

uint32_t frac = (uint32_t)lround(
  (nValue - n) * s.modulus
);

if (frac >= s.modulus) {
  ++n;
  frac = 0;
}

bool integerMode = (frac == 0);

  if ((integerMode && (n < 16 || n > 65535)) ||
      (!integerMode && (n < 19 || n > 4091))) {
    return false;
  }

  // R0: INT, N, FRAC.
  r[0] =
    (integerMode ? (1UL << 31) : 0UL) |
    (n << 15) |
    (frac << 3);

// R1: CPOC, CPL, PHASE, MOD.
r[1] =
    (integerMode ? (1UL << 31) : (1UL << 29)) |
    ((uint32_t)s.phase << 15) |
    ((uint32_t)s.modulus << 3) |
    1UL;

    
// R2: базовая конфигурация
r[2] = 0x00004042UL;

// Reference divider R: bits 23:14
r[2] &= ~(0x3FFUL << 14);
r[2] |= ((uint32_t)(s.referenceDivider & 0x3FFU) << 14);

// LDP: bit 7
r[2] &= ~(1UL << 7);
r[2] |= ((uint32_t)(s.ldp & 1U) << 7);

// MUXOUT: bits 28:26
r[2] &= ~(7UL << 26);
r[2] |= ((uint32_t)(s.muxOut & 7U) << 26);

// Charge Pump: bits 12:9
r[2] &= ~(0x0FUL << 9);
r[2] |= ((uint32_t)(s.chargePump & 0x0FU) << 9);

// Reference doubler: R2 bit 25
if (s.referenceDoubler) {
  r[2] |= (1UL << 25);
} else {
  r[2] &= ~(1UL << 25);
}

// PFD > 32 MHz: R2 bit 31
if (pfdMHz > 32.0) {
  r[2] |= (1UL << 31);
} else {
  r[2] &= ~(1UL << 31);
}
// LDF: bit 8
if (integerMode) {
  r[2] |= (1UL << 8);
} else {
  r[2] &= ~(1UL << 8);
}
  // R3: сохраняем текущую базовую настройку.
  // R3: базовая конфигурация
r[3] = 0x0000000BUL;

// RETUNE: bit 24
if (s.retune) {
  r[3] |= (1UL << 24);
}

  // R4: BS, FB, DIVA, RFOUTA enable, APWR.
  r[4] =
    0x60000000UL |
    (((s.bandSelect >> 8) & 0x03UL) << 24) |
    (1UL << 23) |
    ((uint32_t)divCode << 20) |
    (((uint32_t)s.bandSelect & 0xFFUL) << 12) |
    0x04UL;

  if (s.rfOutputEnable) {
    r[4] |= (1UL << 5);
  }

  r[4] |= ((uint32_t)s.outputPower << 3);


// R5: настройка отдельного вывода LD
r[5] = 0x00400005UL;

// LD[1:0] — биты 23:22
r[5] &= ~(3UL << 22);
r[5] |= ((uint32_t)(s.ldMode & 0x03) << 22);



actualFreqMHz =
  (n + (double)frac / s.modulus) *
  pfdMHz / divider;

  return true;
}
