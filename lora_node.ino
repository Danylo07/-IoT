// Етап 1 + 4: LoRaWAN вузол (ESP32 + RFM95/SX1276, EU868, OTAA, Class A)
// Бібліотека: MCCI LoRaWAN LMIC. У project_config/lmic_project_config.h:
//   #define CFG_eu868 1
//   #define CFG_sx1276_radio 1
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>

// ---- Ключі з TTN (LSB для EUI, MSB для AppKey) ----
static const u1_t PROGMEM APPEUI[8] = {0,0,0,0,0,0,0,0};   // JoinEUI (LSB)
static const u1_t PROGMEM DEVEUI[8] = {0,0,0,0,0,0,0,0};   // DevEUI (LSB)
static const u1_t PROGMEM APPKEY[16] = {0};                // AppKey (MSB)
void os_getArtEui(u1_t* b) { memcpy_P(b, APPEUI, 8); }
void os_getDevEui(u1_t* b) { memcpy_P(b, DEVEUI, 8); }
void os_getDevKey(u1_t* b) { memcpy_P(b, APPKEY, 16); }

const lmic_pinmap lmic_pins = {
  .nss = 18, .rxtx = LMIC_UNUSED_PIN, .rst = 14, .dio = {26, 33, 32},
};

// ---- Параметри ----
#define SLEEP_SEC        900      // 15 хв сну; активність ~ кілька секунд => ~99% сну
#define FULL_FRAME_EVERY 10       // кожен 10-й пакет повний, решта - дельта
#define WDT_TIMEOUT_S    60
#define PIN_SOIL_0 34
#define PIN_SOIL_1 35
#define PIN_SOIL_2 36
#define PIN_SOIL_3 39
#define PIN_SOIL_4 32
#define PIN_WATER  33
#define PIN_VBAT   25
#define PIN_WIND   27

// ---- Стан у RTC пам'яті (зберігається при deep sleep) ----
RTC_DATA_ATTR lmic_t RTC_LMIC;
RTC_DATA_ATTR bool   rtcJoined = false;
RTC_DATA_ATTR uint8_t frameCnt = 0;
RTC_DATA_ATTR uint8_t lastSoil[5] = {0};
RTC_DATA_ATTR uint8_t failCount = 0;

static osjob_t sendjob;
static bool txDone = false;

// ---- Заглушки датчиків (замініть на реальні драйвери: BME280, анемометр, GPS) ----
uint8_t readSoilPct(uint8_t pin) {
  int raw = analogRead(pin);                 // 0..4095, суха ~3200, волога ~1300
  return constrain(map(raw, 3200, 1300, 0, 100), 0, 100);
}
float readTempC()    { return 21.5; }        // TODO: BME280
float readHumidity() { return 55.0; }
float readPressure() { return 1013.0; }      // hPa
float readWindMs()   { return 3.2; }         // TODO: імпульсний анемометр на PIN_WIND
uint8_t readWaterPct() { return constrain(map(analogRead(PIN_WATER), 0, 4095, 0, 100), 0, 100); }
uint16_t readVbatMv() { return analogReadMilliVolts(PIN_VBAT) * 2; } // дільник 1:2
bool readGps(int32_t &lat, int32_t &lon) { lat = 50264800; lon = 19023800; return true; } // TODO: TinyGPS++

// ---- Пакування у компактний binary ----
// FULL  (type 0x01, 20 байт): [1][soil x5][temp i16 0.01C][hum u8 0.5%][press u16 (hPa-300)*10]
//                            [wind u8 0.1 m/s][water u8][vbat u8 (mV/20)][lat i32][lon i32] -> з GPS
// DELTA (type 0x02, 12 байт): [2][soil delta i8 x5][temp i16][hum u8][wind u8][water u8][vbat u8] - 13
size_t buildPayload(uint8_t *b) {
  uint8_t soil[5] = {readSoilPct(PIN_SOIL_0), readSoilPct(PIN_SOIL_1), readSoilPct(PIN_SOIL_2),
                     readSoilPct(PIN_SOIL_3), readSoilPct(PIN_SOIL_4)};
  int16_t temp = (int16_t)(readTempC() * 100);
  uint8_t hum = (uint8_t)(readHumidity() * 2);
  uint16_t press = (uint16_t)((readPressure() - 300) * 10);
  uint8_t wind = (uint8_t)constrain(readWindMs() * 10, 0, 255);
  uint8_t water = readWaterPct();
  uint8_t vbat = (uint8_t)(readVbatMv() / 20);
  size_t n = 0;

  bool full = (frameCnt % FULL_FRAME_EVERY == 0);
  if (!full) {                                   // перевірка, чи дельти влазять в int8
    for (int i = 0; i < 5; i++) if (abs((int)soil[i] - (int)lastSoil[i]) > 127) full = true;
  }

  if (full) {
    b[n++] = 0x01;
    for (int i = 0; i < 5; i++) b[n++] = soil[i];
  } else {
    b[n++] = 0x02;
    for (int i = 0; i < 5; i++) b[n++] = (int8_t)((int)soil[i] - (int)lastSoil[i]);
  }
  b[n++] = temp >> 8; b[n++] = temp & 0xFF;
  b[n++] = hum;
  if (full) { b[n++] = press >> 8; b[n++] = press & 0xFF; }
  b[n++] = wind; b[n++] = water; b[n++] = vbat;
  if (full) {
    int32_t lat, lon;
    if (readGps(lat, lon)) {
      for (int i = 3; i >= 0; i--) b[n++] = (lat >> (8 * i)) & 0xFF;
      for (int i = 3; i >= 0; i--) b[n++] = (lon >> (8 * i)) & 0xFF;
    }
  }
  memcpy(lastSoil, soil, 5);
  frameCnt++;
  if (vbat * 20 < 3300) Serial.println("WARN: низький заряд батареї");
  return n;
}

// ---- TTN payload decoder (вставити в Payload formatter -> Uplink, JavaScript) ----
// function decodeUplink(input){var b=input.bytes,i=0,d={};d.type=b[i++];
//  d.soil=[];for(var k=0;k<5;k++){d.soil.push(d.type==1?b[i++]:(b[i++]<<24>>24));}
//  d.temp=((b[i]<<24>>16)|b[i+1])/100;i+=2;d.hum=b[i++]/2;
//  if(d.type==1){d.pressure=((b[i]<<8|b[i+1])/10)+300;i+=2;}
//  d.wind=b[i++]/10;d.water=b[i++];d.vbat_mv=b[i++]*20;
//  if(d.type==1&&b.length>=i+8){d.lat=((b[i]<<24)|(b[i+1]<<16)|(b[i+2]<<8)|b[i+3])/1e6;i+=4;
//   d.lon=((b[i]<<24)|(b[i+1]<<16)|(b[i+2]<<8)|b[i+3])/1e6;}
//  return {data:d};}

// ---- Сон ----
void goToSleep() {
  Serial.flush();
  memcpy(&RTC_LMIC, &LMIC, sizeof(LMIC));        // зберегти сесію LoRaWAN
  rtcJoined = true;
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SEC * 1000000ULL);
  esp_deep_sleep_start();
}

void doSend(osjob_t*) {
  if (LMIC.opmode & OP_TXRXPEND) return;
  uint8_t buf[24];
  size_t len = buildPayload(buf);
  // confirmed кожен 10-й пакет для контролю зв'язку; решта unconfirmed (економія ефіру)
  bool confirmed = (frameCnt % 10 == 1);
  LMIC_setTxData2(1, buf, len, confirmed);
  Serial.printf("TX %u bytes, SF%d\n", (unsigned)len, 12 - (int)getSf(LMIC.rps));
}

void onEvent(ev_t ev) {
  switch (ev) {
    case EV_JOINED:
      Serial.println("OTAA joined");
      LMIC_setLinkCheckMode(0);
      LMIC_setAdrMode(1);                        // ADR увімкнено
      break;
    case EV_TXCOMPLETE:
      Serial.println("TX complete");
      if (LMIC.txrxFlags & TXRX_ACK) failCount = 0;
      if (LMIC.dataLen) {                        // downlink (вікна RX1/RX2 Class A)
        Serial.printf("Downlink port %d, %d bytes\n", LMIC.frame[LMIC.dataBeg - 1], LMIC.dataLen);
        // приклад: порт 2, байт 0 = новий інтервал сну (хв) - реалізується за потреби
      }
      txDone = true;
      break;
    case EV_JOIN_FAILED:
    case EV_REJOIN_FAILED:
      failCount++;
      break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  // Watchdog: перезапуск, якщо зависли
  esp_task_wdt_config_t wdt = {.timeout_ms = WDT_TIMEOUT_S * 1000, .idle_core_mask = 0, .trigger_panic = true};
  esp_task_wdt_reconfigure(&wdt);
  esp_task_wdt_add(NULL);

  setCpuFrequencyMhz(80);                        // динамічне зниження частоти CPU
  analogReadResolution(12);

  os_init();
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); // компенсація неточності RTC після сну

  if (rtcJoined && failCount < 5) {
    memcpy(&LMIC, &RTC_LMIC, sizeof(LMIC));      // відновлення сесії без повторного join
    Serial.println("Session restored");
  } else {
    rtcJoined = false; failCount = 0; frameCnt = 0;
    LMIC_startJoining();                         // OTAA
  }
  // Потужність: ADR сам знижує SF/TX power; задаємо верхню межу 14 dBm (ліміт EU868)
  LMIC_setDrTxpow(DR_SF9, 14);
  if (rtcJoined) doSend(&sendjob);
}

void loop() {
  os_runloop_once();
  esp_task_wdt_reset();
  // Після join - перша відправка
  if (!rtcJoined && (LMIC.opmode & OP_JOINING) == 0 && !(LMIC.opmode & OP_TXRXPEND) && !txDone) {
    if (LMIC.devaddr != 0) { rtcJoined = true; os_setCallback(&sendjob, doSend); }
  }
  if (txDone && !(LMIC.opmode & OP_TXRXPEND)) goToSleep();
}
