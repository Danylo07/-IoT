// Етап 2 + 4: Zigbee mesh (Arduino-ESP32 core 3.x, Zigbee library)
// УВАГА: класичний ESP32 НЕ має радіо 802.15.4. Потрібні ESP32-C6 або ESP32-H2
// (або ESP32 + зовнішній CC2530, тоді Zigbee-стек на CC2530 / Zigbee2MQTT).
// Arduino IDE: Tools -> Zigbee Mode: "Zigbee ZCZR (coordinator/router)" для COORDINATOR/ROUTER,
//              "Zigbee ED (end device)" для END_DEVICE. Partition Scheme: "Zigbee 4MB with spiffs".
#include "Zigbee.h"
#include <DHT.h>

// ---- Оберіть роль перед прошивкою ----
#define ROLE_COORDINATOR 0
#define ROLE_ROUTER      1
#define ROLE_END_DEVICE  2
#define NODE_ROLE ROLE_END_DEVICE

#define SENSOR_ENDPOINT   10
#define LIGHT_ENDPOINT    11
#define RELAY_ENDPOINT    12
#define DHT_PIN           4
#define LIGHT_PIN         3
#define RELAY_PIN         5     // актуатор поливу
#define VBAT_PIN          2
#define REPORT_SEC        60

DHT dht(DHT_PIN, DHT22);

#if NODE_ROLE == ROLE_COORDINATOR
// ---------------- КООРДИНАТОР: створює PAN, відкриває мережу для join ----------------
void setup() {
  Serial.begin(115200);
  // PAN ID/канал вибираються автоматично; ключ мережі генерується стеком (шифрування AES-128)
  Zigbee.setRebootOpenNetwork(180);          // дозволити приєднання нових пристроїв 180 с після старту
  if (!Zigbee.begin(ZIGBEE_COORDINATOR)) { Serial.println("Zigbee start failed"); ESP.restart(); }
  Serial.println("Coordinator up, network open (auto-discovery)");
}
void loop() {
  // Діагностика: топологія та LQI/RSSI
  static uint32_t t = 0;
  if (millis() - t > 10000) {
    t = millis();
    Zigbee.printBoundDevices(Serial);        // список прив'язаних пристроїв
  }
}

#elif NODE_ROLE == ROLE_ROUTER
// ---------------- РОУТЕР: ретрансляція, mesh routing / self-healing виконує стек ----------------
void setup() {
  Serial.begin(115200);
  if (!Zigbee.begin(ZIGBEE_ROUTER)) { Serial.println("Zigbee start failed"); ESP.restart(); }
  while (!Zigbee.connected()) { Serial.print("."); delay(100); }
  Serial.println("\nRouter joined mesh");
}
void loop() { delay(1000); }

#else
// ---------------- END DEVICE: датчики + актуатор поливу, deep sleep ----------------
ZigbeeTempSensor zbTemp(SENSOR_ENDPOINT);
ZigbeeLight zbRelay(RELAY_ENDPOINT);          // полив як on/off пристрій

void onRelay(bool on) { digitalWrite(RELAY_PIN, on ? HIGH : LOW); }

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  dht.begin();

  zbTemp.setManufacturerAndModel("Student", "FarmNode");
  zbTemp.setMinMaxValue(-20, 60);
  zbTemp.setTolerance(0.5);
  zbTemp.addHumiditySensor(0, 100, 1);       // вологість повітря
  zbTemp.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100);
  zbRelay.onLightChange(onRelay);

  Zigbee.addEndpoint(&zbTemp);
  Zigbee.addEndpoint(&zbRelay);

  esp_zb_cfg_t cfg = ZIGBEE_DEFAULT_ED_CONFIG();
  cfg.nwk_cfg.zed_cfg.keep_alive = 3000;
  if (!Zigbee.begin(&cfg, ZIGBEE_END_DEVICE)) { Serial.println("Zigbee start failed"); ESP.restart(); }
  while (!Zigbee.connected()) { Serial.print("."); delay(100); }
  Serial.println("\nEnd device joined");
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= REPORT_SEC * 1000UL) {
    last = millis();
    float t = dht.readTemperature(), h = dht.readHumidity();
    int lux = analogRead(LIGHT_PIN);                       // освітленість (теплиця)
    int vbatMv = analogReadMilliVolts(VBAT_PIN) * 2;
    uint8_t batPct = constrain(map(vbatMv, 3300, 4200, 0, 100), 0, 100);

    if (!isnan(t)) zbTemp.setTemperature(t);
    if (!isnan(h)) zbTemp.setHumidity(h);
    zbTemp.setBatteryPercentage(batPct);
    zbTemp.report();                                       // відправка через mesh
    zbTemp.reportBatteryPercentage();
    Serial.printf("T=%.1f H=%.1f lux=%d bat=%u%%\n", t, h, lux, batPct);
  }
  delay(100);
}
#endif
