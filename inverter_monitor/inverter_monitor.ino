/*
 * ============================================================
 *  Hybrid Solar Inverter – Modbus RTU Monitor
 *  Hardware : Wemos D1 Mini (ESP8266) + MAX3232 mini board
 *  Protocol : Modbus RTU, Function Code 03, 9600 8N1
 *  Transport: RS232 (MAX3232 level-shifter) → SoftwareSerial
 * ============================================================
 *
 *  Wiring
 *  ──────
 *  Wemos D5 (GPIO14) TX  →  MAX3232 TTL-TX  →  RS232-TX  →  Inverter RX
 *  Wemos D6 (GPIO12) RX  ←  MAX3232 TTL-RX  ←  RS232-RX  ←  Inverter TX
 *  Wemos GND             →  MAX3232 GND
 *  Wemos 3V3             →  MAX3232 VCC  (board has onboard 3.3 V reg)
 *
 *  Libraries (install via Arduino Library Manager)
 *  ────────────────────────────────────────────────
 *  - PubSubClient  by Nick O'Leary
 *  - ESP8266WiFi   (bundled with ESP8266 Arduino core)
 *
 *  Values published to MQTT
 *  ────────────────────────────────────────────────────────
 *  inverter/status           online
 *  inverter/mains_voltage    V
 *  inverter/mains_frequency  Hz
 *  inverter/grid_power_w     W
 *  inverter/pv1_voltage      V
 *  inverter/pv1_current      A
 *  inverter/pv1_power_w      W
 *  inverter/daily_pv_kwh     kWh
 *  inverter/total_pv_kwh     kWh
 *  inverter/battery_voltage  V
 *  inverter/battery_soc      %
 *  inverter/bus_voltage      V
 *  inverter/battery_chg_amps A  (positive only when charging)
 *  inverter/battery_dis_amps A  (positive only when discharging)
 *  inverter/battery_current  A  (signed: + discharge / - charge)
 *  inverter/charging_status  0=None 1=CV/CC 2=Float 3=Equalize
 *  inverter/ac_load_w        W
 *  inverter/daily_output_kwh kWh
 *  inverter/inv_temp_c       °C
 *  inverter/chg_temp_c       °C
 *  inverter/fan_speed_pct    %
 */

#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// ════════════════════════════════════════════════════════════
//  USER CONFIGURATION  ← edit these before flashing
// ════════════════════════════════════════════════════════════

#define WIFI_SSID          "YourWiFiSSID"
#define WIFI_PASSWORD      "YourWiFiPassword"

#define MQTT_SERVER        "192.168.1.x"   // MQTT broker IP (e.g. Raspberry Pi)
#define MQTT_PORT          1883
#define MQTT_USER          ""              // leave empty if broker has no auth
#define MQTT_PASS          ""              // leave empty if broker has no auth
#define MQTT_CLIENT_ID     "inverter_d1"
#define MQTT_BASE_TOPIC    "inverter"      // sub-topics published under this

#define POLL_SECONDS       10              // polling interval (seconds, min 5)

// ════════════════════════════════════════════════════════════
//  HARDWARE — change only if you rewire
// ════════════════════════════════════════════════════════════

#define MODBUS_ADDR        0x01
#define BAUD_RATE          9600
#define RS232_TX_PIN       14    // D5 on Wemos D1 Mini
#define RS232_RX_PIN       12    // D6 on Wemos D1 Mini

// ════════════════════════════════════════════════════════════

SoftwareSerial modbusSerial(RS232_RX_PIN, RS232_TX_PIN);
WiFiClient     wifiClient;
PubSubClient   mqttClient(wifiClient);

// ── Data variables ───────────────────────────────────────────
float   mainsV      = 0;
float   mainsHz     = 0;
int     gridPowerW  = 0;
float   batV        = 0;
float   batChgA     = 0;
float   batDisA     = 0;
float   batCurrent  = 0;
int     batSOC      = 0;
float   busVoltage  = 0;
uint8_t chgStatus   = 0;
float   pv1V        = 0;
float   pv1A        = 0;
int     pv1W        = 0;
float   dailykWh    = 0;
float   totalkWh    = 0;
float   dailyOutkWh = 0;
int     acLoadW     = 0;
float   invTempC    = 0;
float   chgTempC    = 0;
int     fanSpeedPct = 0;

// ── CRC16 Modbus ─────────────────────────────────────────────
uint16_t crc16(uint8_t *d, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= d[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

// ── FC03 Send ────────────────────────────────────────────────
void sendFC03(uint16_t reg, uint16_t count) {
  uint8_t req[8] = {
    MODBUS_ADDR, 0x03,
    (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
    (uint8_t)(count >> 8), (uint8_t)(count & 0xFF),
    0, 0
  };
  uint16_t c = crc16(req, 6);
  req[6] = c & 0xFF;
  req[7] = c >> 8;
  while (modbusSerial.available()) modbusSerial.read();
  modbusSerial.write(req, 8);
  modbusSerial.flush();
}

// ── Read Response ────────────────────────────────────────────
bool readResp(uint8_t *buf, uint8_t expectedLen, uint16_t timeout = 1500) {
  uint8_t n = 0;
  uint32_t t = millis();
  while (millis() - t < timeout && n < expectedLen) {
    if (modbusSerial.available())
      buf[n++] = modbusSerial.read();
  }
  if (n < expectedLen) {
    Serial.printf("  [TIMEOUT] got %d/%d bytes\n", n, expectedLen);
    return false;
  }
  if (buf[1] & 0x80) {
    Serial.printf("  [EXCEPTION] code=0x%02X\n", buf[2]);
    return false;
  }
  uint16_t rxCRC = ((uint16_t)buf[n-1] << 8) | buf[n-2];
  if (crc16(buf, n-2) != rxCRC) {
    Serial.println("  [CRC FAIL]");
    return false;
  }
  return true;
}

// ── Register helpers (renamed to avoid ESP8266 SDK conflict) ──
inline int16_t  getS16(uint8_t *buf, uint8_t off) {
  return (int16_t)((buf[3 + off*2] << 8) | buf[4 + off*2]);
}
inline uint16_t getU16(uint8_t *buf, uint8_t off) {
  return (uint16_t)((buf[3 + off*2] << 8) | buf[4 + off*2]);
}

// ════════════════════════════════════════════════════════════
// BLOCK A: 0x0050–0x0053 — Mains V / Freq / Grid Power
// Response: 13 bytes
// ════════════════════════════════════════════════════════════
bool readMainsData() {
  sendFC03(0x0050, 4);
  delay(150);
  uint8_t buf[16] = {0};
  if (!readResp(buf, 13)) { Serial.println("[A-MAINS] FAIL"); return false; }
  mainsV     = getS16(buf, 0) * 0.1f;   // 0x0050 Mains Voltage  (0.1V)
  mainsHz    = getS16(buf, 2) * 0.01f;  // 0x0052 Mains Frequency (0.01Hz)
  gridPowerW = getS16(buf, 3);           // 0x0053 Grid Power Phase-A (1W)
  return true;
}

// ════════════════════════════════════════════════════════════
// BLOCK B: 0x0080–0x0083 — Battery V + Bidirectional Current
// 0x0081 sign: positive = discharging, negative = charging
// Response: 13 bytes
// ════════════════════════════════════════════════════════════
bool readBatteryData() {
  sendFC03(0x0080, 4);
  delay(150);
  uint8_t buf[16] = {0};
  if (!readResp(buf, 13)) { Serial.println("[B-BATT] FAIL"); return false; }
  batV = getS16(buf, 0) * 0.1f;
  float raw = getS16(buf, 1) * 0.1f;
  if (raw >= 0) {
    batDisA = raw;  batChgA = 0.0f;  batCurrent = raw;
  } else {
    batChgA = -raw; batDisA = 0.0f;  batCurrent = raw;
  }
  return true;
}

// ════════════════════════════════════════════════════════════
// BLOCK B2: 0x0152–0x0154 — SOC + Bus Voltage
//           0x0176        — Charging Status
// ════════════════════════════════════════════════════════════
bool readBattExtData() {
  sendFC03(0x0152, 3);
  delay(150);
  uint8_t buf[14] = {0};
  if (!readResp(buf, 11)) { Serial.println("[B2a-SOC] FAIL"); return false; }
  batSOC     = getS16(buf, 0);           // 0x0152 Battery SOC (1%)
  busVoltage = getS16(buf, 2) * 0.1f;   // 0x0154 Bus Voltage (0.1V)

  sendFC03(0x0176, 1);
  delay(100);
  uint8_t buf2[10] = {0};
  if (!readResp(buf2, 7)) { Serial.println("[B2b-CHG] FAIL"); return false; }
  chgStatus = (uint8_t)getU16(buf2, 0); // 0=None 1=CV/CC 2=Float 3=Equalize
  return true;
}

// ════════════════════════════════════════════════════════════
// BLOCK C: 0x0096–0x009D — PV1 V/A/W + Daily/Total PV Energy
// Response: 21 bytes
// ════════════════════════════════════════════════════════════
bool readPVData() {
  sendFC03(0x0096, 8);
  delay(150);
  uint8_t buf[24] = {0};
  if (!readResp(buf, 21)) { Serial.println("[C-PV] FAIL"); return false; }
  pv1V     = getS16(buf, 0) * 0.1f;   // 0x0096 PV1 Voltage  (0.1V)
  pv1A     = getS16(buf, 1) * 0.01f;  // 0x0097 PV1 Current  (0.01A)
  pv1W     = getS16(buf, 2);           // 0x0098 PV1 Power    (1W)
  dailykWh = getU16(buf, 6) * 0.1f;   // 0x009C Daily PV Gen (100Wh→kWh)
  totalkWh = getU16(buf, 7) * 0.1f;   // 0x009D Total PV Gen (100Wh→kWh)
  return true;
}

// ════════════════════════════════════════════════════════════
// BLOCK D: 0x021C — AC Output Load Power
// Response: 7 bytes
// ════════════════════════════════════════════════════════════
bool readLoadData() {
  sendFC03(0x021C, 1);
  delay(100);
  uint8_t buf[10] = {0};
  if (!readResp(buf, 7)) { Serial.println("[D-LOAD] FAIL"); return false; }
  acLoadW = getS16(buf, 0);
  return true;
}

// ════════════════════════════════════════════════════════════
// BLOCK F: 0x0221–0x0222 — Daily Output Energy (H+L 32-bit)
// Unit: 10Wh per LSB → convert to kWh
// Response: 9 bytes
// ════════════════════════════════════════════════════════════
bool readOutputEnergy() {
  sendFC03(0x0221, 2);
  delay(100);
  uint8_t buf[12] = {0};
  if (!readResp(buf, 9)) { Serial.println("[F-ENERGY] FAIL"); return false; }
  uint32_t raw = ((uint32_t)getU16(buf, 0) << 16) | getU16(buf, 1);
  dailyOutkWh = (raw * 10.0f) / 1000.0f;
  return true;
}

// ════════════════════════════════════════════════════════════
// BLOCK G: 0x0320 — Fan Speed %
// Response: 7 bytes
// ════════════════════════════════════════════════════════════
bool readFanData() {
  sendFC03(0x0320, 1);
  delay(100);
  uint8_t buf[10] = {0};
  if (!readResp(buf, 7)) { Serial.println("[G-FAN] FAIL"); return false; }
  fanSpeedPct = getU16(buf, 0);
  return true;
}

// ════════════════════════════════════════════════════════════
// BLOCK E: 0x0330–0x0332 — INV Temp + CHG Temp
// Unit: 0.1°C  (raw 400 = 40.0°C)
// Response: 11 bytes
// ════════════════════════════════════════════════════════════
bool readTempData() {
  sendFC03(0x0330, 3);
  delay(100);
  uint8_t buf[14] = {0};
  if (!readResp(buf, 11)) { Serial.println("[E-TEMP] FAIL"); return false; }
  invTempC = getS16(buf, 1) * 0.1f;   // 0x0331 INV Temp
  chgTempC = getS16(buf, 2) * 0.1f;   // 0x0332 CHG Temp
  return true;
}

// ── MQTT Publish Helpers ─────────────────────────────────────
void mqttPublish(const char *sub, float val, int dec = 1) {
  char topic[80], payload[20];
  snprintf(topic, sizeof(topic), "%s/%s", MQTT_BASE_TOPIC, sub);
  dtostrf(val, 1, dec, payload);
  mqttClient.publish(topic, payload, true);
}
void mqttPublish(const char *sub, int val) {
  char topic[80], payload[20];
  snprintf(topic, sizeof(topic), "%s/%s", MQTT_BASE_TOPIC, sub);
  snprintf(payload, sizeof(payload), "%d", val);
  mqttClient.publish(topic, payload, true);
}

// ── Publish All Values ───────────────────────────────────────
void publishAll() {
  mqttPublish("mains_voltage",      mainsV,      1);
  mqttPublish("mains_frequency",    mainsHz,     2);
  mqttPublish("grid_power_w",       gridPowerW);
  mqttPublish("battery_voltage",    batV,        1);
  mqttPublish("battery_soc",        batSOC);
  mqttPublish("bus_voltage",        busVoltage,  1);
  mqttPublish("battery_chg_amps",   batChgA,     1);
  mqttPublish("battery_dis_amps",   batDisA,     1);
  mqttPublish("battery_current",    batCurrent,  1);
  mqttPublish("charging_status",    (int)chgStatus);
  mqttPublish("pv1_voltage",        pv1V,        1);
  mqttPublish("pv1_current",        pv1A,        2);
  mqttPublish("pv1_power_w",        pv1W);
  mqttPublish("daily_pv_kwh",       dailykWh,    2);
  mqttPublish("total_pv_kwh",       totalkWh,    1);
  mqttPublish("daily_output_kwh",   dailyOutkWh, 2);
  mqttPublish("ac_load_w",          acLoadW);
  mqttPublish("inv_temp_c",         invTempC,    1);
  mqttPublish("chg_temp_c",         chgTempC,    1);
  mqttPublish("fan_speed_pct",      fanSpeedPct);
  Serial.printf("[MQTT] Published 20 values → %s/#\n", MQTT_BASE_TOPIC);
}

// ── Serial Print ─────────────────────────────────────────────
void printData() {
  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println(  "║       INVERTER DATA SNAPSHOT         ║");
  Serial.println(  "╠══════════════════════════════════════╣");
  Serial.printf(   "║ Mains Voltage     : %6.1f V         ║\n", mainsV);
  Serial.printf(   "║ Mains Frequency   : %6.2f Hz        ║\n", mainsHz);
  Serial.printf(   "║ Grid Power        : %6d W          ║\n", gridPowerW);
  Serial.println(  "╠══════════════════════════════════════╣");
  Serial.printf(   "║ PV1 Voltage       : %6.1f V         ║\n", pv1V);
  Serial.printf(   "║ PV1 Current       : %6.2f A         ║\n", pv1A);
  Serial.printf(   "║ PV1 Power         : %6d W          ║\n", pv1W);
  Serial.println(  "╠══════════════════════════════════════╣");
  Serial.printf(   "║ Battery Voltage   : %6.1f V         ║\n", batV);
  Serial.printf(   "║ Battery SOC       : %6d %%          ║\n", batSOC);
  Serial.printf(   "║ Bus Voltage       : %6.1f V         ║\n", busVoltage);
  Serial.printf(   "║ Chg Current       : %6.1f A         ║\n", batChgA);
  Serial.printf(   "║ Dis Current       : %6.1f A         ║\n", batDisA);
  Serial.printf(   "║ Battery Current   : %6.1f A (+-chg) ║\n", batCurrent);
  Serial.printf(   "║ Charging Status   : %6d            ║\n", (int)chgStatus);
  Serial.println(  "╠══════════════════════════════════════╣");
  Serial.printf(   "║ AC Output Load    : %6d W          ║\n", acLoadW);
  Serial.printf(   "║ Daily Output      : %6.2f kWh       ║\n", dailyOutkWh);
  Serial.println(  "╠══════════════════════════════════════╣");
  Serial.printf(   "║ Daily PV Gen      : %6.2f kWh       ║\n", dailykWh);
  Serial.printf(   "║ Total PV Gen      : %6.1f kWh       ║\n", totalkWh);
  Serial.println(  "╠══════════════════════════════════════╣");
  Serial.printf(   "║ INV Temp          : %6.1f C         ║\n", invTempC);
  Serial.printf(   "║ CHG Temp          : %6.1f C         ║\n", chgTempC);
  Serial.printf(   "║ Fan Speed         : %6d %%          ║\n", fanSpeedPct);
  Serial.println(  "╚══════════════════════════════════════╝\n");
}

// ── WiFi Connect (flash-wear safe) ───────────────────────────
void connectWiFi() {
  WiFi.persistent(false);      // prevent writing credentials to flash
  WiFi.setAutoConnect(false);  // no flash read on boot
  WiFi.setAutoReconnect(true); // handles drops in RAM — no flash write
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected. IP: %s\n",
                WiFi.localIP().toString().c_str());
}

// ── MQTT Connect / Reconnect ─────────────────────────────────
void connectMQTT() {
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  while (!mqttClient.connected()) {
    Serial.printf("[MQTT] Connecting to %s:%d ...", MQTT_SERVER, MQTT_PORT);
    bool ok = (strlen(MQTT_USER) > 0)
      ? mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)
      : mqttClient.connect(MQTT_CLIENT_ID);
    if (ok) {
      Serial.println(" Connected!");
      char topic[80];
      snprintf(topic, sizeof(topic), "%s/status", MQTT_BASE_TOPIC);
      mqttClient.publish(topic, "online", true);
    } else {
      Serial.printf(" Failed (rc=%d). Retry in 5s\n", mqttClient.state());
      delay(5000);
    }
  }
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n╔═════════════════════════════════════╗");
  Serial.println(  "║  Wemos D1 Mini – Inverter Monitor   ║");
  Serial.println(  "╚═════════════════════════════════════╝");
  Serial.printf("Poll interval : %d seconds\n", POLL_SECONDS);
  Serial.printf("MQTT broker   : %s:%d\n",      MQTT_SERVER, MQTT_PORT);
  Serial.printf("Base topic    : %s/<param>\n",  MQTT_BASE_TOPIC);
  modbusSerial.begin(BAUD_RATE);
  delay(300);
  connectWiFi();
  connectMQTT();
  Serial.println("[Ready] Starting poll loop...\n");
}

// ── Loop ─────────────────────────────────────────────────────
void loop() {
  if (!mqttClient.connected()) {
    Serial.println("[MQTT] Disconnected — reconnecting...");
    connectMQTT();
  }
  mqttClient.loop();

  static uint32_t lastPoll = 0;
  if (millis() - lastPoll >= (uint32_t)POLL_SECONDS * 1000UL) {
    lastPoll = millis();
    Serial.printf("[Poll] Reading inverter (interval %ds)...\n", POLL_SECONDS);
    readMainsData();     delay(60);
    readBatteryData();   delay(60);
    readBattExtData();   delay(60);
    readPVData();        delay(60);
    readLoadData();      delay(60);
    readOutputEnergy();  delay(60);
    readFanData();       delay(60);
    readTempData();      delay(60);
    printData();
    publishAll();
  }
}
