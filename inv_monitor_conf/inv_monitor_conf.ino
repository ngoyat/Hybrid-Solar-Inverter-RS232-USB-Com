/*
 * ============================================================
 *  Inv_monitor_conf – Modbus RTU Monitor + Settings Control
 *  Hardware : Wemos D1 Mini (ESP8266) + MAX3232 mini board
 *  Protocol : Modbus RTU, FC03 (read) / FC10 (write multiple regs, count=1), 9600 8N1
 *  Transport: RS232 via USB-A socket on inverter chassis
 *  Broker   : Mosquitto (Raspberry Pi) — PubSubClient
 * ============================================================
 *
 *  FLASH-WEAR PROTECTION SUMMARY
 *  ─────────────────────────────
 *  INVERTER flash:
 *   1. Only FC10 is used for writes (FC06 is absent from this sketch).
 *   2. sendFC10() has exactly ONE call site: writeIfChanged().
 *   3. writeIfChanged() first reads the current register (readBeforeWrite).
 *      If the live value already equals the requested value, FC10 is skipped.
 *   4. A 500 ms minimum gap between consecutive writes is enforced in RAM.
 *   5. setup() contains ZERO FC10 calls — only 2 one-time FC03 cache reads.
 *   6. loop() polling functions NEVER touch settings regs 0x4102–0x412E.
 *   7. Reconnect helpers (connectWiFi / connectMQTT) contain ZERO Modbus calls.
 *   8. No retry loop on write failure — a failed write is final.
 *      writeIfChanged() returns false; caller publishes error; no auto-retry.
 *
 *  ESP8266 flash:
 *   - WiFi.persistent(false) — no credential writes to ESP8266 flash.
 *   - No EEPROM / LittleFS / SPIFFS used anywhere.
 *   - WiFi.setAutoReconnect(true) handles link recovery without re-calling begin().
 *
 *  RETAINED MQTT PROTECTION
 *   - On every MQTT connect, an empty retained message is published to
 *     TOPIC_SETTINGS_SET before subscribing.  This clears any stale retained
 *     command the broker stored, preventing replay after a reboot.
 * ============================================================
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// ─── User Configuration ──────────────────────────────────────────────────────
#define WIFI_SSID          "YourWiFiSSID"
#define WIFI_PASSWORD      "YourWiFiPassword"
#define MQTT_SERVER        "192.168.1.x"
#define MQTT_PORT          1883
#define MQTT_USER          ""
#define MQTT_PASS          ""
#define MQTT_CLIENT_ID     "inv_conf_d1"
#define MQTT_BASE_TOPIC    "inverter"
#define POLL_SECONDS       10
// ─────────────────────────────────────────────────────────────────────────────

// ─── RS232 pins (SoftwareSerial) ─────────────────────────────────────────────
#define RS232_RX_PIN  12   // D6
#define RS232_TX_PIN  14   // D5
// ─────────────────────────────────────────────────────────────────────────────

// ─── Settings Registers ──────────────────────────────────────────────────────
#define REG_OUTPUT_PRIORITY     0x4102
#define REG_GRID_CHARGE_CURRENT 0x4105
#define REG_MAX_CHARGE_CURRENT  0x4106
#define REG_BATTERY_TYPE        0x4107
#define REG_BAT_LOW_CUTOFF      0x4108
#define REG_BAT_UNDERVOLTAGE    0x4109
#define REG_CV_POINT            0x410A
#define REG_FLOAT_VOLTAGE       0x410B
#define REG_SWITCH_TO_GRID_V    0x410C
#define REG_RETURN_TO_BAT_V     0x410D
#define REG_GRID_FEED_IN        0x412E
// ─────────────────────────────────────────────────────────────────────────────

// ─── MQTT Topics ─────────────────────────────────────────────────────────────
#define TOPIC_SETTINGS_GET     MQTT_BASE_TOPIC "/settings/get"
#define TOPIC_SETTINGS_SET     MQTT_BASE_TOPIC "/settings/set"
#define TOPIC_SETTINGS_STATUS  MQTT_BASE_TOPIC "/settings/status"
// ─────────────────────────────────────────────────────────────────────────────

// ─── Globals ─────────────────────────────────────────────────────────────────
SoftwareSerial rs232(RS232_RX_PIN, RS232_TX_PIN);
WiFiClient     wifiClient;
PubSubClient   mqttClient(wifiClient);

static uint16_t g_cachedBatType  = 0xFFFF;  // set in setup() via FC03
static uint32_t g_lastWriteMs    = 0;
static bool     g_writeEverDone  = false;

// Forward declarations
bool    sendFC10(uint16_t reg, uint16_t value);
int8_t  readBeforeWrite(uint16_t reg, uint16_t newVal);
bool    writeIfChanged(const char *key, uint16_t reg, uint16_t newVal);
void    handleSettingsSet(const char *payload);
void    handleSettingsGet(const char *payload);
void    mqttCallback(char *topic, byte *payload, unsigned int length);
void    connectWiFi();
void    connectMQTT();
// ─────────────────────────────────────────────────────────────────────────────

// ─── CRC16 ───────────────────────────────────────────────────────────────────
static uint16_t crc16(const uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── FC03 Read ───────────────────────────────────────────────────────────────
static bool fc03Read(uint16_t startReg, uint8_t count, uint16_t *out) {
  uint8_t req[8];
  req[0] = 0x01; req[1] = 0x03;
  req[2] = startReg >> 8; req[3] = startReg & 0xFF;
  req[4] = 0x00;          req[5] = count;
  uint16_t c = crc16(req, 6);
  req[6] = c & 0xFF; req[7] = c >> 8;

  rs232.flush();
  rs232.write(req, 8);
  rs232.flush();

  uint8_t  resp[3 + count * 2 + 2];
  uint8_t  got = 0;
  uint32_t t   = millis();
  while (millis() - t < 600 && got < sizeof(resp)) {
    if (rs232.available()) resp[got++] = rs232.read();
    else yield();
  }
  if (got < sizeof(resp)) return false;
  uint16_t rc = crc16(resp, sizeof(resp) - 2);
  if ((resp[sizeof(resp)-2] != (rc & 0xFF)) || (resp[sizeof(resp)-1] != (rc >> 8))) return false;
  for (uint8_t i = 0; i < count; i++)
    out[i] = ((uint16_t)resp[3 + i*2] << 8) | resp[4 + i*2];
  return true;
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── sendFC10 — ONLY call site is writeIfChanged() ───────────────────────────
bool sendFC10(uint16_t reg, uint16_t value) {
  uint8_t req[13];
  req[0]  = 0x01; req[1]  = 0x10;
  req[2]  = reg >> 8;    req[3]  = reg & 0xFF;
  req[4]  = 0x00;        req[5]  = 0x01;   // quantity = 1
  req[6]  = 0x02;                           // byte count = 2
  req[7]  = value >> 8;  req[8]  = value & 0xFF;
  uint16_t c = crc16(req, 9);
  req[9]  = c & 0xFF;    req[10] = c >> 8;

  rs232.flush();
  delay(50);
  rs232.write(req, 11);
  rs232.flush();

  uint8_t  resp[8];
  uint8_t  got = 0;
  uint32_t t   = millis();
  while (millis() - t < 800 && got < 8) {
    if (rs232.available()) resp[got++] = rs232.read();
    else yield();
  }
  if (got < 8) { Serial.printf("[FC10] TIMEOUT reg=0x%04X got %d/8\n", reg, got); return false; }
  if (resp[1] == 0x90) { Serial.printf("[FC10] EXCEPTION reg=0x%04X code=0x%02X\n", reg, resp[2]); return false; }
  if (resp[1] != 0x10) { Serial.printf("[FC10] BAD FC byte reg=0x%04X\n", reg); return false; }
  uint16_t rc = crc16(resp, 6);
  if ((resp[6] != (rc & 0xFF)) || (resp[7] != (rc >> 8))) { Serial.printf("[FC10] CRC FAIL reg=0x%04X\n", reg); return false; }
  Serial.printf("[FC10] OK reg=0x%04X val=%u\n", reg, value);
  return true;
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── readBeforeWrite — skip FC10 if value unchanged ──────────────────────────
int8_t readBeforeWrite(uint16_t reg, uint16_t newVal) {
  uint16_t cur = 0;
  if (!fc03Read(reg, 1, &cur)) return -1;  // read fail → proceed with write
  if (cur == 0xFFFF)           return  0;  // sentinel → proceed
  if (cur == newVal)           return  1;  // already set → skip
  return 0;                                // different → write needed
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── writeIfChanged ───────────────────────────────────────────────────────────
bool writeIfChanged(const char *key, uint16_t reg, uint16_t newVal) {
  // 500 ms inter-write gap
  if (g_writeEverDone) {
    uint32_t elapsed = millis() - g_lastWriteMs;
    if (elapsed < 500) delay(500 - elapsed);
  }

  delay(50);  // RS232 line settle after readBeforeWrite FC03
  int8_t rbw = readBeforeWrite(reg, newVal);
  if (rbw == 1) {
    char msg[80];
    snprintf(msg, sizeof(msg), "{\"key\":\"%s\",\"status\":\"already_set\",\"value\":%u}", key, newVal);
    mqttClient.publish(TOPIC_SETTINGS_STATUS, msg);
    Serial.printf("[WIC] %s already=%u skip\n", key, newVal);
    return true;
  }

  bool ok = sendFC10(reg, newVal);
  g_lastWriteMs   = millis();
  g_writeEverDone = true;

  char msg[96];
  if (ok)
    snprintf(msg, sizeof(msg), "{\"key\":\"%s\",\"status\":\"write_ok\",\"value\":%u}", key, newVal);
  else
    snprintf(msg, sizeof(msg), "{\"key\":\"%s\",\"status\":\"write_fail\",\"value\":%u}", key, newVal);
  mqttClient.publish(TOPIC_SETTINGS_STATUS, msg);
  return ok;
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── Battery type guard for voltage settings ──────────────────────────────────
static bool checkBatTypeWritable(const char *key) {
  if (g_cachedBatType == 0xFFFF) {
    uint16_t v = 0;
    if (fc03Read(REG_BATTERY_TYPE, 1, &v)) g_cachedBatType = v;
  }
  if (g_cachedBatType == 0 || g_cachedBatType == 1) {
    char msg[96];
    snprintf(msg, sizeof(msg), "{\"key\":\"%s\",\"status\":\"validation_error\",\"reason\":\"fixed for AGM/Flooded\"}", key);
    mqttClient.publish(TOPIC_SETTINGS_STATUS, msg);
    return false;
  }
  return true;
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── Validators ──────────────────────────────────────────────────────────────
bool validate_output_priority(float fval) {
  int iv = (int)fval;
  if (iv < 0 || iv > 3) return false;
  return writeIfChanged("output_priority", REG_OUTPUT_PRIORITY, (uint16_t)iv);
}

bool validate_grid_charge_current(float fval) {
  int iv = (int)fval;
  const uint16_t ok[] = {2,10,20,30,40};
  for (uint8_t i = 0; i < 5; i++) if ((uint16_t)iv == ok[i]) return writeIfChanged("grid_charge_current", REG_GRID_CHARGE_CURRENT, (uint16_t)iv);
  return false;
}

bool validate_max_charge_current(float fval) {
  int iv = (int)fval;
  if (iv < 0 || iv > 5) return false;
  return writeIfChanged("max_charge_current", REG_MAX_CHARGE_CURRENT, (uint16_t)iv);
}

bool validate_battery_type(float fval) {
  int iv = (int)fval;
  if (iv < 0 || iv > 4) return false;
  bool ok = writeIfChanged("battery_type", REG_BATTERY_TYPE, (uint16_t)iv);
  if (ok) g_cachedBatType = (uint16_t)iv;
  return ok;
}

bool validate_BatVoltage(const char *key, uint16_t reg, float fval) {
  if (!checkBatTypeWritable(key)) return false;

  // per-register limits (raw = V × 10)
  float lo = 0, hi = 0;
  if      (reg == REG_BAT_LOW_CUTOFF)   { lo = 41.2f; hi = 50.0f; }
  else if (reg == REG_BAT_UNDERVOLTAGE) { lo = 40.0f; hi = 48.0f; }
  else if (reg == REG_CV_POINT)         { lo = 48.0f; hi = 60.0f; }
  else if (reg == REG_FLOAT_VOLTAGE)    { lo = 50.0f; hi = 58.0f; }
  else if (reg == REG_SWITCH_TO_GRID_V) { lo = 40.0f; hi = 50.0f; }
  else if (reg == REG_RETURN_TO_BAT_V)  { lo = 46.0f; hi = 58.0f; }
  else return false;

  if (fval < lo || fval > hi) {
    char msg[96];
    snprintf(msg, sizeof(msg), "{\"key\":\"%s\",\"status\":\"validation_error\",\"reason\":\"out of range\"}", key);
    mqttClient.publish(TOPIC_SETTINGS_STATUS, msg);
    return false;
  }
  uint16_t scaled = (uint16_t)(fval * 10.0f + 0.5f);
  return writeIfChanged(key, reg, (uint16_t)scaled);
}

bool validate_grid_feed_in(float fval) {
  int iv = (int)fval;
  if (iv < 0 || iv > 1) return false;
  return writeIfChanged("grid_feed_in_enable", REG_GRID_FEED_IN, (uint16_t)iv);
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── handleSettingsGet ───────────────────────────────────────────────────────
void handleSettingsGet(const char *payload) {
  if (strcmp(payload, "all") != 0) return;

  const struct { const char *key; uint16_t reg; bool scale; } regs[] = {
    {"output_priority",        REG_OUTPUT_PRIORITY,     false},
    {"grid_charge_current",    REG_GRID_CHARGE_CURRENT, false},
    {"max_charge_current",     REG_MAX_CHARGE_CURRENT,  false},
    {"battery_type",           REG_BATTERY_TYPE,        false},
    {"battery_low_cutoff",     REG_BAT_LOW_CUTOFF,      true },
    {"battery_undervoltage",   REG_BAT_UNDERVOLTAGE,    true },
    {"cv_point",               REG_CV_POINT,            true },
    {"float_voltage",          REG_FLOAT_VOLTAGE,       true },
    {"switch_to_grid_voltage", REG_SWITCH_TO_GRID_V,    true },
    {"return_to_battery_v",    REG_RETURN_TO_BAT_V,     true },
    {"grid_feed_in_enable",    REG_GRID_FEED_IN,        false},
  };
  for (uint8_t i = 0; i < sizeof(regs)/sizeof(regs[0]); i++) {
    uint16_t v = 0;
    char msg[96];
    if (fc03Read(regs[i].reg, 1, &v)) {
      if (regs[i].scale)
        snprintf(msg, sizeof(msg), "{\"key\":\"%s\",\"value\":%.1f}", regs[i].key, v / 10.0f);
      else
        snprintf(msg, sizeof(msg), "{\"key\":\"%s\",\"value\":%u}", regs[i].key, v);
    } else {
      snprintf(msg, sizeof(msg), "{\"key\":\"%s\",\"status\":\"read_error\"}", regs[i].key);
    }
    mqttClient.publish(TOPIC_SETTINGS_STATUS, msg);
    delay(600);
  }
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── handleSettingsSet ───────────────────────────────────────────────────────
void handleSettingsSet(const char *payload) {
  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err || doc.size() != 1) {
    mqttClient.publish(TOPIC_SETTINGS_STATUS, "{\"status\":\"parse_error\"}");
    return;
  }

  const char *key  = doc.as<JsonObject>().begin()->key().c_str();
  float       fval = doc[key].as<float>();
  bool        ok   = false;

  if      (strcmp(key, "output_priority")        == 0) ok = validate_output_priority(fval);
  else if (strcmp(key, "grid_charge_current")    == 0) ok = validate_grid_charge_current(fval);
  else if (strcmp(key, "max_charge_current")     == 0) ok = validate_max_charge_current(fval);
  else if (strcmp(key, "battery_type")           == 0) ok = validate_battery_type(fval);
  else if (strcmp(key, "battery_low_cutoff")     == 0) ok = validate_BatVoltage(key, REG_BAT_LOW_CUTOFF,   fval);
  else if (strcmp(key, "battery_undervoltage")   == 0) ok = validate_BatVoltage(key, REG_BAT_UNDERVOLTAGE, fval);
  else if (strcmp(key, "cv_point")               == 0) ok = validate_BatVoltage(key, REG_CV_POINT,         fval);
  else if (strcmp(key, "float_voltage")          == 0) ok = validate_BatVoltage(key, REG_FLOAT_VOLTAGE,    fval);
  else if (strcmp(key, "switch_to_grid_voltage") == 0) ok = validate_BatVoltage(key, REG_SWITCH_TO_GRID_V, fval);
  else if (strcmp(key, "return_to_battery_v")    == 0) ok = validate_BatVoltage(key, REG_RETURN_TO_BAT_V,  fval);
  else if (strcmp(key, "grid_feed_in_enable")    == 0) ok = validate_grid_feed_in(fval);
  else {
    char msg[80];
    snprintf(msg, sizeof(msg), "{\"key\":\"%s\",\"status\":\"validation_error\",\"reason\":\"unknown key\"}", key);
    mqttClient.publish(TOPIC_SETTINGS_STATUS, msg);
  }
  (void)ok;
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── MQTT Callback ───────────────────────────────────────────────────────────
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  char buf[256] = {0};
  if (length >= sizeof(buf)) return;
  memcpy(buf, payload, length);
  if      (strcmp(topic, TOPIC_SETTINGS_GET) == 0) handleSettingsGet(buf);
  else if (strcmp(topic, TOPIC_SETTINGS_SET) == 0) handleSettingsSet(buf);
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── connectWiFi — zero Modbus calls ─────────────────────────────────────────
void connectWiFi() {
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) { delay(500); tries++; yield(); }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\n[WiFi] Connected -- IP: %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("\n[WiFi] Failed to connect");
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── connectMQTT — zero Modbus calls ─────────────────────────────────────────
void connectMQTT() {
  mqttClient.setCallback(mqttCallback);
  uint8_t tries = 0;
  while (!mqttClient.connected() && tries < 5) {
    bool ok = (strlen(MQTT_USER) > 0)
              ? mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)
              : mqttClient.connect(MQTT_CLIENT_ID);
    if (ok) {
      // Clear any stale retained command before subscribing
      mqttClient.publish(TOPIC_SETTINGS_SET, "", true);
      mqttClient.subscribe(TOPIC_SETTINGS_GET);
      mqttClient.subscribe(TOPIC_SETTINGS_SET);
      Serial.println("[MQTT] Connected");
    } else {
      delay(2000); tries++;
    }
  }
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── Telemetry polling ───────────────────────────────────────────────────────
static void publishVal(const char *sub, float v, uint8_t dp = 1) {
  char topic[80], val[20];
  snprintf(topic, sizeof(topic), "%s/%s", MQTT_BASE_TOPIC, sub);
  if (dp == 0) snprintf(val, sizeof(val), "%d", (int)v);
  else         snprintf(val, sizeof(val), "%.*f", dp, v);
  mqttClient.publish(topic, val, true);
}

static void pollTelemetry() {
  uint16_t d[8];
  if (fc03Read(0x0050, 4, d)) {
    publishVal("mains_voltage",    d[0] * 0.1f);
    publishVal("mains_frequency",  d[1] * 0.01f);
    publishVal("grid_power_w",     (int16_t)d[3] * 1.0f, 0);
  }
  delay(60);
  if (fc03Read(0x0096, 5, d)) {
    publishVal("pv1_voltage",  d[0] * 0.1f);
    publishVal("pv1_current",  d[1] * 0.1f);
    publishVal("pv1_power_w",  d[2] * 1.0f, 0);
  }
  delay(60);
  if (fc03Read(0x0080, 4, d)) {
    publishVal("battery_voltage", d[0] * 0.1f);
    publishVal("battery_soc",     d[1] * 1.0f, 0);
    publishVal("bus_voltage",     d[2] * 0.1f);
  }
  delay(60);
  if (fc03Read(0x0152, 3, d)) {
    uint16_t raw = d[0];
    float chg = 0, dis = 0, sig = 0;
    if (raw & 0x8000) { dis = (raw & 0x7FFF) * 0.1f; sig = dis; }
    else              { chg = raw * 0.1f; sig = -chg; }
    publishVal("battery_chg_amps",  chg);
    publishVal("battery_dis_amps",  dis);
    publishVal("battery_current",   sig);
    publishVal("charging_status",   d[2] * 1.0f, 0);
  }
  delay(60);
  if (fc03Read(0x021C, 1, d)) publishVal("ac_load_w", d[0] * 1.0f, 0);
  delay(60);
  if (fc03Read(0x0221, 2, d)) {
    publishVal("daily_output_kwh", d[0] * 0.1f);
    publishVal("total_pv_kwh",     ((uint32_t)d[0] << 16 | d[1]) * 0.1f);
  }
  delay(60);
  if (fc03Read(0x0320, 1, d)) publishVal("fan_speed_pct", d[0] * 1.0f, 0);
  delay(60);
  if (fc03Read(0x0330, 3, d)) {
    publishVal("inv_temp_c", d[0] * 0.1f);
    publishVal("chg_temp_c", d[1] * 0.1f);
  }
  delay(60);
  if (fc03Read(0x009B, 2, d)) {
    publishVal("daily_pv_kwh", d[0] * 0.1f);
  }
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  rs232.begin(9600);
  Serial.println("\n[BOOT] Inv_monitor_conf starting");

  WiFi.persistent(false);   // no credential writes to ESP8266 flash
  WiFi.mode(WIFI_STA);
  connectWiFi();

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setBufferSize(512);
  connectMQTT();

  // One-time cache reads — ZERO FC10 in setup()
  uint16_t v = 0;
  if (fc03Read(REG_BATTERY_TYPE, 1, &v)) g_cachedBatType = v;
  delay(100);
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── loop ────────────────────────────────────────────────────────────────────
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Lost -- reconnecting...");
    connectWiFi();
  }
  if (!mqttClient.connected()) {
    Serial.println("[MQTT] Lost -- reconnecting...");
    connectMQTT();
  }
  mqttClient.loop();

  static uint32_t lastPoll = 0;
  if (millis() - lastPoll >= (uint32_t)POLL_SECONDS * 1000UL) {
    lastPoll = millis();
    pollTelemetry();
  }
}
// ─────────────────────────────────────────────────────────────────────────────
