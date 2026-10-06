// Етап 3: CoAP сервер на сенсорному вузлі (ESP32, Wi-Fi, бібліотека coap-simple)
// Обмеження coap-simple: немає Observe та Block-wise. Observe реалізовано як
// push-сповіщення (NON) підписникам; для block-wise див. примітку внизу.
#include <WiFi.h>
#include <WiFiUdp.h>
#include <coap-simple.h>

const char* SSID = "YOUR_WIFI";
const char* PASS = "YOUR_PASS";

WiFiUDP udp;
Coap coap(udp);

// Content-Format (RFC 7252 / 8949): text=0, json=50, cbor=60
#define CF_TEXT 0
#define CF_JSON 50
#define CF_CBOR 60

// ---- Дані (заглушки) ----
float temperature = 21.5, humidity = 55, soilMoisture = 40, lightLevel = 800, batteryV = 3.9;
bool irrigation = false, ventilation = false, heating = false;
uint32_t rxPackets = 0, errors = 0;

// ---- Підписники Observe ----
IPAddress obsIp; uint16_t obsPort = 0; bool hasObserver = false;

// Мінімальний CBOR-енкодер для одного float: major 7, additional 26 (float32)
size_t cborFloat(float v, uint8_t* out) {
  out[0] = 0xFA; uint32_t u; memcpy(&u, &v, 4);
  out[1] = u >> 24; out[2] = u >> 16; out[3] = u >> 8; out[4] = u; return 5;
}

// Відповідь із content negotiation за опцією Accept (опція 17)
void replyValue(IPAddress ip, int port, CoapPacket &pkt, const char* key, float v) {
  int accept = CF_TEXT;
  for (int i = 0; i < pkt.optionnum; i++)
    if (pkt.options[i].number == COAP_ACCEPT && pkt.options[i].length)
      accept = pkt.options[i].buffer[0];
  char buf[64]; COAP_CONTENT_TYPE ct;
  if (accept == CF_JSON) { snprintf(buf, sizeof buf, "{\"%s\":%.2f}", key, v); ct = COAP_APPLICATION_JSON; }
  else if (accept == CF_CBOR) {
    uint8_t c[8]; size_t n = cborFloat(v, c);
    coap.sendResponse(ip, port, pkt.messageid, (const char*)c, n, COAP_CONTENT,
                      (COAP_CONTENT_TYPE)CF_CBOR, pkt.token, pkt.tokenlen); return;
  } else { snprintf(buf, sizeof buf, "%.2f", v); ct = COAP_TEXT_PLAIN; }
  coap.sendResponse(ip, port, pkt.messageid, buf, strlen(buf), COAP_CONTENT, ct, pkt.token, pkt.tokenlen);
}

// ---- /sensors/* ----
void rTemp(CoapPacket &p, IPAddress ip, int port)  { rxPackets++; replyValue(ip, port, p, "temperature", temperature); }
void rHum(CoapPacket &p, IPAddress ip, int port)   { rxPackets++; replyValue(ip, port, p, "humidity", humidity); }
void rSoil(CoapPacket &p, IPAddress ip, int port)  { rxPackets++; replyValue(ip, port, p, "soil", soilMoisture); }
void rLight(CoapPacket &p, IPAddress ip, int port) { rxPackets++; replyValue(ip, port, p, "light", lightLevel); }
void rBat(CoapPacket &p, IPAddress ip, int port) {
  rxPackets++;
  // Observe-реєстрація: клієнт надсилає Observe=0; запам'ятовуємо його
  for (int i = 0; i < p.optionnum; i++) if (p.options[i].number == COAP_OBSERVE) { obsIp = ip; obsPort = port; hasObserver = true; }
  replyValue(ip, port, p, "battery_v", batteryV);
}

// ---- /actuators/* (PUT/POST: "1"/"0"; GET: стан) ----
void actuatorHandler(CoapPacket &p, IPAddress ip, int port, bool &state, int pin) {
  rxPackets++;
  if (p.code == COAP_PUT || p.code == COAP_POST) {
    state = (p.payloadlen > 0 && p.payload[0] == '1');
    if (pin >= 0) digitalWrite(pin, state);
    coap.sendResponse(ip, port, p.messageid, "", 0, COAP_CHANGED, COAP_TEXT_PLAIN, p.token, p.tokenlen);
  } else {
    const char* s = state ? "1" : "0";
    coap.sendResponse(ip, port, p.messageid, s, 1, COAP_CONTENT, COAP_TEXT_PLAIN, p.token, p.tokenlen);
  }
}
void rIrr(CoapPacket &p, IPAddress ip, int port)  { actuatorHandler(p, ip, port, irrigation, 5); }
void rVent(CoapPacket &p, IPAddress ip, int port) { actuatorHandler(p, ip, port, ventilation, 18); }
void rHeat(CoapPacket &p, IPAddress ip, int port) { actuatorHandler(p, ip, port, heating, 19); }

// ---- /diagnostics/* ----
void rNet(CoapPacket &p, IPAddress ip, int port) {
  char b[96]; snprintf(b, sizeof b, "{\"rssi\":%d,\"rx\":%lu,\"errors\":%lu}", WiFi.RSSI(), rxPackets, errors);
  coap.sendResponse(ip, port, p.messageid, b, strlen(b), COAP_CONTENT, COAP_APPLICATION_JSON, p.token, p.tokenlen);
}
void rErr(CoapPacket &p, IPAddress ip, int port) {
  const char* s = errors ? "errors present" : "no errors";
  coap.sendResponse(ip, port, p.messageid, s, strlen(s), COAP_CONTENT, COAP_TEXT_PLAIN, p.token, p.tokenlen);
}
void rPerf(CoapPacket &p, IPAddress ip, int port) {
  char b[96]; snprintf(b, sizeof b, "{\"uptime_s\":%lu,\"heap\":%u,\"cpu_mhz\":%u}",
                       millis() / 1000, ESP.getFreeHeap(), getCpuFrequencyMhz());
  coap.sendResponse(ip, port, p.messageid, b, strlen(b), COAP_CONTENT, COAP_APPLICATION_JSON, p.token, p.tokenlen);
}

// ---- Відповіді клієнта (для агрегатора) ----
void onResponse(CoapPacket &p, IPAddress ip, int port) {
  char s[p.payloadlen + 1]; memcpy(s, p.payload, p.payloadlen); s[p.payloadlen] = 0;
  Serial.printf("[%s] %s\n", ip.toString().c_str(), s);
}

void setup() {
  Serial.begin(115200);
  pinMode(5, OUTPUT); pinMode(18, OUTPUT); pinMode(19, OUTPUT);
  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) delay(300);
  Serial.println(WiFi.localIP());

  coap.server(rTemp,  "sensors/temperature");
  coap.server(rHum,   "sensors/humidity");
  coap.server(rSoil,  "sensors/soil-moisture");
  coap.server(rLight, "sensors/light-level");
  coap.server(rBat,   "sensors/battery-status");
  coap.server(rIrr,   "actuators/irrigation");
  coap.server(rVent,  "actuators/ventilation");
  coap.server(rHeat,  "actuators/heating");
  coap.server(rNet,   "diagnostics/network-stats");
  coap.server(rErr,   "diagnostics/error-log");
  coap.server(rPerf,  "diagnostics/performance");
  coap.response(onResponse);
  coap.start();
}

void loop() {
  coap.loop();

  // Observe: NON-повідомлення підписнику при зміні (тут - раз на 10 с)
  static uint32_t t = 0;
  if (hasObserver && millis() - t > 10000) {
    t = millis();
    char b[16]; snprintf(b, sizeof b, "%.2f", batteryV);
    coap.put(obsIp, obsPort, "sensors/battery-status", b);   // спрощений push
  }

  // Клієнт-агрегатор (приклад): GET з сусіднього вузла кожні 30 с
  static uint32_t agg = 0;
  if (millis() - agg > 30000) {
    agg = millis();
    IPAddress peer(192, 168, 1, 50);                         // IP іншого вузла
    coap.get(peer, 5683, "sensors/temperature");
  }
}
// Block-wise (RFC 7959): для великих даних (/diagnostics/error-log) використовуйте
// libcoap/microcoap або реалізуйте опцію Block2 (23) вручну - coap-simple її не підтримує.
