#include <Arduino.h>
#include <NimBLEDevice.h>
#include "MAX2870Registers.h"

#define LED_PIN    10
#define ADF_SCK     6
#define ADF_MOSI    7
#define ADF_CE      1
#define ADF_LE      3
#define MUXOUT_PIN  5

MAX2870Settings maxSettings;


static const double REF_MHZ = 25.0;
static double requestedFreqMHz = 0.0;
static bool freqValid = false;
static bool pllInitialized = false;
static double actualFreqMHz = 0.0;
static const char* DEVICE_NAME = "MAX2870-ESP32";

static NimBLECharacteristic* statusChar = nullptr;
static bool bleConnected = false;

// Используем адреса и UUID, совпадающие с Android-приложением.
static NimBLEUUID SERVICE_UUID("7b100001-7b10-4a6c-9c42-1234567890ab");
static NimBLEUUID COMMAND_UUID("7b100002-7b10-4a6c-9c42-1234567890ab");
static NimBLEUUID STATUS_UUID ("7b100003-7b10-4a6c-9c42-1234567890ab");

// Стартовые значения только для примера.
// Их необходимо заменить на проверенную конфигурацию платы.
uint32_t regs[6] = {
  0x00500000,
  0x20008001,
  0x00004042,
  0x0000000B,
  0x61AFC034,
  0x00C00005
};

void writeMAX2870(uint32_t value) {
  digitalWrite(ADF_LE, LOW);

  for (int i = 31; i >= 0; --i) {
    digitalWrite(ADF_SCK, LOW);
    digitalWrite(ADF_MOSI, (value >> i) & 1U);
    delayMicroseconds(1);
    digitalWrite(ADF_SCK, HIGH);
    delayMicroseconds(1);
  }

  digitalWrite(ADF_SCK, LOW);
  digitalWrite(ADF_LE, HIGH);
  delayMicroseconds(1);
  digitalWrite(ADF_LE, LOW);
}

uint32_t readMAX2870Reg6() {
  uint32_t value = 0;

  // Передаём адрес регистра 6: последние биты 110.
  digitalWrite(ADF_LE, LOW);

  for (int i = 31; i >= 0; --i) {
    digitalWrite(ADF_SCK, LOW);

    bool bitValue = (i == 2 || i == 1);
    digitalWrite(ADF_MOSI, bitValue ? HIGH : LOW);

    delayMicroseconds(1);
    digitalWrite(ADF_SCK, HIGH);
    delayMicroseconds(1);
  }

  // Завершаем команду чтения.
  digitalWrite(ADF_SCK, LOW);
  digitalWrite(ADF_LE, HIGH);
  delayMicroseconds(1);

  // Дополнительный такт запуска выдачи данных.
  digitalWrite(ADF_SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(ADF_SCK, LOW);
  delayMicroseconds(1);

  // Считываем 32 бита регистра 6 через MUXOUT.
  for (int i = 31; i >= 0; --i) {
    digitalWrite(ADF_SCK, HIGH);
    delayMicroseconds(1);

    digitalWrite(ADF_SCK, LOW);
    delayMicroseconds(1);

    value <<= 1;
    if (digitalRead(MUXOUT_PIN)) {
      value |= 1UL;
    }
  }

  digitalWrite(ADF_LE, LOW);
  return value;
}


void diagnoseMAX2870() {
  uint32_t savedR2 = regs[2];
  uint32_t savedR5 = regs[5];

  // MUX = 1100: чтение регистра 6
  uint32_t readR2 = savedR2;
  uint32_t readR5 = savedR5;

  readR2 &= ~(7UL << 26);
  readR2 |= (4UL << 26);

  readR5 |= (1UL << 18);

  writeMAX2870(readR5);
  writeMAX2870(readR2);

  delayMicroseconds(10);

  // Три последовательных чтения
  for (int i = 0; i < 3; i++) {
    uint32_t r6 = readMAX2870Reg6();

    Serial.printf(
      "READ%d: REG6=0x%08lX POR=%lu ADC=%lu VCO=%lu\n",
      i + 1,
      (unsigned long)r6,
      (r6 >> 23) & 1UL,
      (r6 >> 20) & 7UL,
      (r6 >> 3) & 0x3FUL
    );

    delay(20);
  }

  // Восстанавливаем исходные настройки
  writeMAX2870(savedR5);
  writeMAX2870(savedR2);
}


static const uint32_t MODULUS = 4095;
static const uint16_t BAND_SELECT = 500; // 25 MHz / 50 kHz

bool configureFrequency(double freqMHz) {
  if (!isfinite(freqMHz) ||
      freqMHz < 23.5 || freqMHz > 6000.0) {
    return false;
  }

  // Строим все шесть регистров.
  if (!buildMAX2870Registers(
        freqMHz,
        maxSettings,
        regs,
        actualFreqMHz)) {
    return false;
  }

Serial.printf(
  "SETTINGS: muxOut=%u ldMode=%u chargePump=%u\n",
  (unsigned)maxSettings.muxOut,
  (unsigned)maxSettings.ldMode,
  (unsigned)maxSettings.chargePump
);

Serial.printf(
  "BUILT: R2=0x%08lX R5=0x%08lX\n",
  (unsigned long)regs[2],
  (unsigned long)regs[5]
);

  // Первая полная последовательность: R5 -> R0.
  for (int i = 5; i >= 0; --i) {
    writeMAX2870(regs[i]);
  }

  // После включения питания выдерживаем паузу
  // между первой и второй полной записью регистров.
  if (!pllInitialized) {
    delay(20);

    // Вторая полная последовательность.
    for (int i = 5; i >= 0; --i) {
      writeMAX2870(regs[i]);
    }

    pllInitialized = true;
  }

  requestedFreqMHz = freqMHz;
  freqValid = true;

  delay(500);

  // Выводим фактически сформированные регистры.
  for (int i = 0; i < 6; i++) {
    char msg[40];
    snprintf(msg, sizeof(msg),
             "REG%d=0x%08lX",
             i, (unsigned long)regs[i]);
    Serial.println(msg);
  }

  return true;
}


void sendStatus(const String& message) {
  Serial.println(message);

  if (statusChar != nullptr) {
    statusChar->setValue(message.c_str());
    statusChar->notify();
  }
}

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    bleConnected = true;
    Serial.println("BLE connected");
  }

  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    bleConnected = false;
    Serial.println("BLE disconnected");
    NimBLEDevice::startAdvertising();
  }
};

// Возвращает код MUXOUT 0..15.
// Важно: точное значение режима нужно сверить с таблицей MUXOUT
// официального datasheet для выбранной версии MAX2870.
void setMuxoutCode(uint8_t code) {
  code &= 0x0F;

  // R2[28:26] = младшие 3 бита кода
  regs[2] &= ~(0x7UL << 26);
  regs[2] |= ((uint32_t)(code & 0x07) << 26);

  // R5[18] = старший бит кода
  regs[5] &= ~(1UL << 18);
  regs[5] |= ((uint32_t)((code >> 3) & 1) << 18);

  writeMAX2870(regs[2]);
  writeMAX2870(regs[5]);
}

// LD: 0 = LOW, 1 = digital lock detect,
// 2 = analog lock detect, 3 = HIGH.
void setLdMode(uint8_t mode) {
  mode &= 0x03;
  regs[5] &= ~(0x3UL << 22);
  regs[5] |= ((uint32_t)mode << 22);
  writeMAX2870(regs[5]);
}

void handleCommand(String cmd) {
  cmd.trim();
  Serial.print("RX: ");
  Serial.println(cmd);

if (cmd == "STATUS") {
  int lockPin = digitalRead(MUXOUT_PIN);

  String pllState = "UNKNOWN";
  if (freqValid) {
    pllState = lockPin ? "LOCKED" : "UNLOCKED";
  }

  sendStatus(
      String("STATUS:BLE=") +
      (bleConnected ? "1" : "0") +
      ",GPIO5=" + String(lockPin) +
      ",FREQ_VALID=" + String(freqValid ? 1 : 0) +
      ",FREQ_MHZ=" + String(actualFreqMHz, 6) +
      ",PLL=" + pllState
  );
  return;
}

if (cmd == "RF:ON" || cmd == "RF:OFF") {
  if (!freqValid || !pllInitialized) {
    sendStatus("ERR:FREQ_NOT_CONFIGURED");
    return;
  }

  if (cmd == "RF:ON") {
    if (digitalRead(MUXOUT_PIN) == LOW) {
      sendStatus("ERR:PLL_NOT_LOCKED");
      return;
    }

    // R4 bit 5 = RFA_EN.
    // APWR[1:0] = 00, то есть минимальная настройка -4 dBm.
    regs[4] &= ~(3UL << 3);
    regs[4] |= (1UL << 5);
    writeMAX2870(regs[4]);

    sendStatus("OK:RFOUTA_ON_POWER_MINUS4DBM");
  } else {
    regs[4] &= ~(1UL << 5);
    writeMAX2870(regs[4]);
    sendStatus("OK:RFOUTA_OFF");
  }

  return;
}

  if (cmd.startsWith("LD:")) {
    int value = cmd.substring(3).toInt();
    if (value < 0 || value > 3) {
      sendStatus("ERR:LD_RANGE");
      return;
    }
    setLdMode((uint8_t)value);
    sendStatus("OK:LD");
    return;
  }

  if (cmd.startsWith("MUX:")) {
    int value = cmd.substring(4).toInt();
    if (value < 0 || value > 15) {
      sendStatus("ERR:MUX_RANGE");
      return;
    }
    setMuxoutCode((uint8_t)value);
    sendStatus("OK:MUX_CODE=" + String(value));
    return;
  }

    if (cmd == "HELP") {
    sendStatus("CMDS: FREQ <MHz>, STATUS, HELP, LD:0..3, MUX:0..15");
    return;
  }

if (cmd.startsWith("FREQ ")) {
  String arg = cmd.substring(5);
  arg.trim();

  char *endPtr = nullptr;
  double freqMHz = strtod(arg.c_str(), &endPtr);

  if (arg.length() == 0 ||
      endPtr == arg.c_str() ||
      *endPtr != '\0' ||
      !isfinite(freqMHz)) {
    sendStatus("ERR:FREQ_FORMAT");
    return;
  }

  if (!configureFrequency(freqMHz)) {
    sendStatus("ERR:FREQ_CONFIG_FAILED");
    return;
  }

  sendStatus("OK:FREQ_MHZ=" + String(actualFreqMHz, 6));
  sendStatus(
      String("INFO:GPIO5=") +
      String(digitalRead(MUXOUT_PIN))
  );
  return;
}
  
if (cmd.startsWith("SET_FREQ:")) {
  String arg = cmd.substring(9);
  arg.trim();

  char *endPtr = nullptr;
  double freqMHz = strtod(arg.c_str(), &endPtr);

  if (arg.length() == 0 ||
      endPtr == arg.c_str() ||
      *endPtr != '\0' ||
      !isfinite(freqMHz)) {
    sendStatus("ERR:FREQ_FORMAT");
    return;
  }

  if (!configureFrequency(freqMHz)) {
    sendStatus("ERR:FREQ_CONFIG_FAILED");
    return;
  }

  sendStatus("OK:FREQ_MHZ=" + String(actualFreqMHz, 6));
  return;
}
  sendStatus("ERR:UNKNOWN_COMMAND");
}

class CommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic,
               NimBLEConnInfo&) override {
    std::string value = characteristic->getValue();
    if (value.empty()) return;

    String data(value.c_str());
    int start = 0;

    // Поддерживаем несколько строк в одном BLE-пакете.
    while (start < data.length()) {
      int end = data.indexOf('\n', start);
      if (end < 0) end = data.length();

      String line = data.substring(start, end);
      handleCommand(line);

      start = end + 1;
    }
  }
};

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(ADF_SCK, OUTPUT);
  pinMode(ADF_MOSI, OUTPUT);
  pinMode(ADF_CE, OUTPUT);
  pinMode(ADF_LE, OUTPUT);
  pinMode(MUXOUT_PIN, INPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(ADF_SCK, LOW);
  digitalWrite(ADF_MOSI, LOW);
  digitalWrite(ADF_LE, LOW);
  digitalWrite(ADF_CE, HIGH);

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* service = server->createService(SERVICE_UUID);

  statusChar = service->createCharacteristic(
      STATUS_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  NimBLECharacteristic* commandChar = service->createCharacteristic(
      COMMAND_UUID,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );

  commandChar->setCallbacks(new CommandCallbacks());
  statusChar->setValue("STATUS:READY");

  service->start();

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();

  Serial.println("MAX2870 BLE service started");
  Serial.println("Waiting for Android connection");
  Serial.begin(115200);
delay(1000);

Serial.println("================================");
Serial.println("FIRMWARE: MAX2870-ESP32 NEW");
Serial.println("BUILD: TEST-20261024");
Serial.println("================================");
delay(500);

maxSettings.referenceMHz = 25.0;
maxSettings.modulus = 4095;
maxSettings.chargePump = 4;
maxSettings.muxOut = 6;
maxSettings.rfDivider = 2;
maxSettings.outputPower = 0;
maxSettings.rfOutputEnable = true;
maxSettings.outputPower = 2;
maxSettings.ldMode = 1;
maxSettings.referenceDivider = 1;
maxSettings.ldp = 0;
maxSettings.referenceDoubler = false;
maxSettings.bandSelect = 500;
maxSettings.retune = false;

//if (configureFrequency(34.9609375)) {
if (configureFrequency(1125.0)) {
  delay(100);
  diagnoseMAX2870();
}else {
  sendStatus("ERR:STARTUP_FREQ_FAILED");
}
}

void loop() {
  // USB Serial остаётся доступным для отладки.
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }
  digitalWrite(LED_PIN, HIGH);
  delay(5);
    digitalWrite(LED_PIN, LOW);
      delay(5);
}
