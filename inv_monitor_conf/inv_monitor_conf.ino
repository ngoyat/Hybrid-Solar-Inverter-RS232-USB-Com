/*
 * ============================================================
 *  Inv_monitor_conf – Modbus RTU Monitor + Settings Control
 *  Hardware : Wemos D1 Mini (ESP8266) + MAX3232 mini board
 *  Protocol : Modbus RTU, FC03 (read) / FC10 (write multiple regs, count=1), 9600 8N1
 *  Transport: RS232 via MAX3232 → SoftwareSerial
 *
 *  WIRING
 *  ─────────────────────────────────────────────────────────────────
 *  Wemos D5 (GPIO14) TX  →  MAX3232 TTL-TX  →  RS232-TX  →  Inverter RX
 *  Wemos D6 (GPIO12) RX  ←  MAX3232 TTL-RX  ←  RS232-RX  ←  Inverter TX
 *  Wemos GND             →  MAX3232 GND
 *  Wemos 3V3             →  MAX3232 VCC
 *
 *  WHY FC10 IS USED FOR WRITES
 *  ─────────────────────────────────────────────────────────────────
 *  This inverter returns exception 0x86 to Write Single Register.
 *  Write Multiple Registers (count=1) is accepted and returns
 *  a clean 8-byte echo. Confirmed on Serial Monitor.
 *
 *  ESP8266 SoftwareSerial NOTE
 *  ─────────────────────────────────────────────────────────────────
 *  ESP8266 has no free hardware UART (UART0 = USB debug). SoftwareSerial
 *  is used for Modbus. At 9600 baud each byte takes ~1042µs.
 *  WiFi background tasks can stall the CPU between bit edges.
 *  To compensate:
 *    - RX timeout is generous
 *    - yield() called inside RX loop to prevent WDT reset
 *    - WiFi.persistent(false) + setAutoConnect(false) prevent
 *      credential writes to ESP flash on reconnects
 *    - All writes are guarded with read-before-write so a failed
 *      FC10 is never silently retried
 *
 *  INVERTER FLASH PROTECTION GUARANTEES
 *  ─────────────────────────────────────────────────────────────────
 *  1. ONE parameter per MQTT transaction — multi-key JSON rejected.
 *  2. 500ms inter-write gap enforced in RAM (g_writeEverDone flag).
 *  3. Retained MQTT payload cleared (empty publish) on every connect.
 *  4. Read-before-write: FC03 reads current value; FC10 fires ONLY
 *     when new value differs. 65535 sentinel → proceed.
 *  5. sendFC10() has exactly ONE call site: writeIfChanged().
 *     writeIfChanged() ← validate_*() ← handleSettingsSet() ← mqttCallback().
 *  6. g_alreadySet flag propagates already_set result upward.
 *  7. setup() has ZERO FC10. Two one-time FC03 cache reads only.
 *  8. loop() poll functions NEVER touch settings regs 0x4102–0x412E.
 *  9. No retry logic anywhere — fail is final; user decides next action.
 *
 *  WEMOS FLASH PROTECTION
 *  ─────────────────────────────────────────────────────────────────
 *  WiFi.persistent(false)      — no credential writes to ESP8266 flash.
 *  WiFi.setAutoConnect(false)  — no auto-connect flash writes on boot.
 *  No EEPROM / LittleFS / SPIFFS used anywhere.
 *
 *  LIBRARIES REQUIRED (Arduino Library Manager)
 *  ─────────────────────────────────────────────────────────────────
 *  PubSubClient >= 2.8  (Nick O'Leary)
 *  ESP8266WiFi          (bundled with ESP8266 Arduino core)
 * ============================================================
 */

#include <math.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#define MQTT_MAX_PACKET_SIZE 512
#include <PubSubClient.h>

#define WIFI_SSID          "YourWiFiSSID"
#define WIFI_PASSWORD      "YourWiFiPassword"
#define MQTT_SERVER        "192.168.1.x"
#define MQTT_PORT          1883
#define MQTT_USER          ""
#define MQTT_PASS          ""
#define MQTT_CLIENT_ID     "inverter_d1"
#define MQTT_BASE_TOPIC    "inverter"
#define POLL_SECONDS       10
#define RS232_TX_PIN       14
#define RS232_RX_PIN       12
#define MODBUS_ADDR        0x01
#define BAUD_RATE          9600
#define WRITE_GAP_MS       500UL

SoftwareSerial modbusSerial(RS232_RX_PIN, RS232_TX_PIN);

#define TOPIC_SETTINGS_GET    MQTT_BASE_TOPIC "/settings/get"
#define TOPIC_SETTINGS_SET    MQTT_BASE_TOPIC "/settings/set"
#define TOPIC_SETTINGS_STATUS MQTT_BASE_TOPIC "/settings/status"
#define TOPIC_SETTINGS_ERROR  MQTT_BASE_TOPIC "/settings/error"

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

#define BAT_AGM     0
#define BAT_FLOODED 1
#define BAT_TERNARY 2
#define BAT_LIFEPO4 3
#define BAT_CUSTOM  4

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

float   mainsV=0, mainsHz=0;
int     gridPowerW=0;
float   batV=0, batChgA=0, batDisA=0, batCurrent=0;
int     batSOC=0;
float   busVoltage=0;
uint16_t chgStatus=0;
float   pv1V=0, pv1A=0;
int     pv1W=0;
float   dailykWh=0, totalkWh=0, dailyOutkWh=0;
int     acLoadW=0;
float   invTempC=0, chgTempC=0;
int     fanSpeedPct=0;

int8_t cachedBatteryType = -1;

bool     g_writeEverDone = false;
uint32_t lastWriteMillis = 0;
bool g_alreadySet = false;

void pubError(const char *key, const char *reason, const char *recv = nullptr);
void pubStatus(const char *key, uint16_t reg, uint16_t raw);
bool sendFC10(uint16_t reg, uint16_t value);
int8_t readBeforeWrite(uint16_t reg, uint16_t newVal);

uint16_t crc16(uint8_t *d, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= d[i];
    for (uint8_t j = 0; j < 8; j++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

void sendFC03(uint16_t reg, uint16_t count) {
  uint8_t req[8] = {
    MODBUS_ADDR, 0x03,
    (uint8_t)(reg >> 8),   (uint8_t)(reg & 0xFF),
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

// sendFC10 — THE ONLY FC10 CALL SITE IN THIS ENTIRE FILE.
// Called only from writeIfChanged().
//
// NOTE ON ESP8266 SOFTWARESERIAL:
// This Modbus port is SoftwareSerial, not a hardware UART FIFO.
// At 9600 baud it is usually workable, but WiFi/background activity
// can still introduce timing jitter. We therefore keep frames short,
// use a generous timeout, and treat any timeout/CRC mismatch as final.
// No retry. Fail is final.
bool sendFC10(uint16_t reg, uint16_t value) {
  uint8_t req[11] = {
    MODBUS_ADDR, 0x10,
    (uint8_t)(reg >> 8),   (uint8_t)(reg & 0xFF),
    0x00, 0x01,
    0x02,
    (uint8_t)(value >> 8), (uint8_t)(value & 0xFF),
    0, 0
  };
  uint16_t c = crc16(req, 9);
  req[9] = c & 0xFF;
  req[10] = c >> 8;

  Serial.printf("[FC10] TX: ");
  for (uint8_t i = 0; i < 11; i++) Serial.printf("%02X ", req[i]);
  Serial.println();

  uint8_t stale = 0;
  while (modbusSerial.available()) { modbusSerial.read(); stale++; }
  if (stale) Serial.printf("[FC10] Flushed %u stale bytes before TX\n", stale);

  modbusSerial.write(req, 11);
  modbusSerial.flush();

  uint8_t resp[8] = {0};
  uint8_t n = 0;
  uint32_t t = millis();
  while (millis() - t < 3000UL && n < 8) {
    if (modbusSerial.available()) resp[n++] = (uint8_t)modbusSerial.read();
    yield();
  }

  Serial.printf("[FC10] RX (%u/8): ", n);
  for (uint8_t i = 0; i < n; i++) Serial.printf("%02X ", resp[i]);
  Serial.println();

  if (n < 8) {
    Serial.printf("[FC10] TIMEOUT reg=0x%04X got %d/8\n", reg, n);
    return false;
  }
  if (resp[1] & 0x80) {
    Serial.printf("[FC10] EXCEPTION 0x%02X\n", resp[2]);
    return false;
  }
  if (resp[0] != MODBUS_ADDR || resp[1] != 0x10 || resp[2] != (uint8_t)(reg >> 8) || resp[3] != (uint8_t)(reg & 0xFF) || resp[4] != 0x00 || resp[5] != 0x01) {
    Serial.println("[FC10] RESPONSE MISMATCH");
    return false;
  }
  uint16_t rx = ((uint16_t)resp[7] << 8) | resp[6];
  if (crc16(resp, 6) != rx) {
    Serial.printf("[FC10] CRC FAIL rx=0x%04X calc=0x%04X\n", rx, crc16(resp,6));
    return false;
  }
  lastWriteMillis = millis();
  g_writeEverDone = true;
  Serial.printf("[FC10] OK reg=0x%04X value=%u\n", reg, value);
  return true;
}

bool readResp(uint8_t *buf, uint8_t elen, uint16_t timeout = 1500) {
  uint8_t n = 0;
  uint32_t t = millis();
  while (millis() - t < timeout && n < elen) {
    if (modbusSerial.available()) buf[n++] = (uint8_t)modbusSerial.read();
    yield();
  }
  if (n < elen) {
    Serial.printf("  [TIMEOUT] %d/%d\n", n, elen);
    return false;
  }
  if (buf[1] & 0x80) {
    Serial.printf("  [EXCEPTION] 0x%02X\n", buf[2]);
    return false;
  }
  uint16_t rx = ((uint16_t)buf[n-1] << 8) | buf[n-2];
  if (crc16(buf, n-2) != rx) {
    Serial.println("  [CRC FAIL]");
    return false;
  }
  return true;
}

inline int16_t  getS16(uint8_t *b, uint8_t o) { return (int16_t)((b[3+o*2] << 8) | b[4+o*2]); }
inline uint16_t getU16(uint8_t *b, uint8_t o) { return (uint16_t)((b[3+o*2] << 8) | b[4+o*2]); }

// Read current raw register first; skip write if already identical.
int8_t readBeforeWrite(uint16_t reg, uint16_t newVal) {
  sendFC03(reg, 1);
  delay(200);
  uint8_t b[10] = {0};
  if (!readResp(b, 7)) {
    Serial.printf("[RBW] FC03 fail reg=0x%04X\n", reg);
    return -1;
  }
  uint16_t cur = getU16(b, 0);
  if (cur == 65535) {
    Serial.printf("[RBW] 65535 sentinel reg=0x%04X → proceed\n", reg);
    return 1;
  }
  if (cur == newVal) {
    Serial.printf("[RBW] reg=0x%04X already=%u → skip FC10\n", reg, cur);
    return 0;
  }
  Serial.printf("[RBW] reg=0x%04X %u→%u → write\n", reg, cur, newVal);
  return 1;
}

void mqttPublishF(const char *sub, float val, int dec = 1) {
  char t[80], pl[20];
  snprintf(t, sizeof(t), "%s/%s", MQTT_BASE_TOPIC, sub);
  dtostrf(val, 1, dec, pl);
  mqttClient.publish(t, pl, true);
}

void mqttPublishI(const char *sub, int val) {
  char t[80], pl[20];
  snprintf(t, sizeof(t), "%s/%s", MQTT_BASE_TOPIC, sub);
  snprintf(pl, sizeof(pl), "%d", val);
  mqttClient.publish(t, pl, true);
}

void pubError(const char *key, const char *reason, const char *recv) {
  char buf[256];
  if (recv) snprintf(buf, sizeof(buf), "{\"error\":\"%s\",\"key\":\"%s\",\"received\":\"%s\"}", reason, key, recv);
  else snprintf(buf, sizeof(buf), "{\"error\":\"%s\",\"key\":\"%s\"}", reason, key);
  mqttClient.publish(TOPIC_SETTINGS_ERROR, buf);
  Serial.printf("[ERR] %s\n", buf);
}

// Raw 0.1V setting registers from the inverter sheet.
bool is01VSettingReg(uint16_t reg) {
  return reg == REG_BAT_LOW_CUTOFF   ||
         reg == REG_BAT_UNDERVOLTAGE ||
         reg == REG_CV_POINT         ||
         reg == REG_FLOAT_VOLTAGE    ||
         reg == REG_SWITCH_TO_GRID_V ||
         reg == REG_RETURN_TO_BAT_V;
}

// Publish raw register value plus human-readable unit where applicable.
void pubStatus(const char *key, uint16_t reg, uint16_t raw) {
  char buf[220];
  if (raw == 65535) {
    snprintf(buf, sizeof(buf), "{\"key\":\"%s\",\"raw_value\":65535,\"note\":\"Post-write sentinel — inverter settling. GET again after 1s.\"}", key);
  } else if (is01VSettingReg(reg)) {
    snprintf(buf, sizeof(buf), "{\"key\":\"%s\",\"raw_value\":%u,\"value_v\":%.1f}", key, raw, raw / 10.0f);
  } else {
    snprintf(buf, sizeof(buf), "{\"key\":\"%s\",\"raw_value\":%u}", key, raw);
  }
  mqttClient.publish(TOPIC_SETTINGS_STATUS, buf);
  Serial.printf("[STATUS] %s\n", buf);
}

bool writeIfChanged(const char *key, uint16_t reg, uint16_t newVal) {
  g_alreadySet = false;
  if (g_writeEverDone) {
    uint32_t elapsed = millis() - lastWriteMillis;
    if (elapsed < WRITE_GAP_MS) {
      char msg[160];
      snprintf(msg, sizeof(msg),
        "{\"error\":\"Write too soon. Wait %lums.\",\"key\":\"%s\",\"remaining_ms\":%lu}",
        (unsigned long)(WRITE_GAP_MS - elapsed), key,
        (unsigned long)(WRITE_GAP_MS - elapsed));
      mqttClient.publish(TOPIC_SETTINGS_ERROR, msg);
      Serial.printf("[GATE] Blocked — gap %lums < 500ms\n", (unsigned long)elapsed);
      return false;
    }
  }

  int8_t rbw = readBeforeWrite(reg, newVal);
  delay(100);
  if (rbw < 0) {
    pubError(key, "RBW read failed. Aborting for safety.");
    return false;
  }
  if (rbw == 0) {
    char msg[128];
    snprintf(msg, sizeof(msg), "{\"result\":\"already_set\",\"key\":\"%s\",\"value\":%u}", key, newVal);
    mqttClient.publish(TOPIC_SETTINGS_STATUS, msg);
    g_alreadySet = true;
    return true;
  }
  return sendFC10(reg, newVal);
}

bool readMainsData() {
  sendFC03(0x0050, 4); delay(150);
  uint8_t b[16] = {0};
  if (!readResp(b, 13)) { Serial.println("[A-MAINS] FAIL"); return false; }
  mainsV     = getS16(b, 0) * 0.1f;
  mainsHz    = getS16(b, 2) * 0.01f;
  gridPowerW = getS16(b, 3);
  return true;
}

bool readBatteryData() {
  sendFC03(0x0080, 4); delay(150);
  uint8_t b[16] = {0};
  if (!readResp(b, 13)) { Serial.println("[B-BATT] FAIL"); return false; }
  batV = getS16(b, 0) * 0.1f;
  float raw = getS16(b, 1) * 0.1f;
  if (raw >= 0) { batDisA = raw;  batChgA = 0.0f; batCurrent =  raw; }
  else          { batChgA = -raw; batDisA = 0.0f; batCurrent =  raw; }
  return true;
}

bool readBattExtData() {
  sendFC03(0x0152, 3); delay(150);
  uint8_t b[14] = {0};
  if (!readResp(b, 11)) { Serial.println("[B2a-SOC] FAIL"); return false; }
  batSOC     = getS16(b, 0);
  busVoltage = getS16(b, 2) * 0.1f;
  sendFC03(0x0176, 1); delay(100);
  uint8_t b2[10] = {0};
  if (!readResp(b2, 7)) { Serial.println("[B2b-CHG] FAIL"); return false; }
  chgStatus = getU16(b2, 0);
  return true;
}

bool readPVData() {
  sendFC03(0x0096, 8); delay(150);
  uint8_t b[24] = {0};
  if (!readResp(b, 21)) { Serial.println("[C-PV] FAIL"); return false; }
  pv1V     = getS16(b, 0) * 0.1f;
  pv1A     = getS16(b, 1) * 0.01f;
  pv1W     = getS16(b, 2);
  dailykWh = getU16(b, 6) * 0.1f;
  totalkWh = getU16(b, 7) * 0.1f;
  return true;
}

bool readLoadData() {
  sendFC03(0x021C, 1); delay(100);
  uint8_t b[10] = {0};
  if (!readResp(b, 7)) { Serial.println("[D-LOAD] FAIL"); return false; }
  acLoadW = getS16(b, 0);
  return true;
}

bool readOutputEnergy() {
  sendFC03(0x0221, 2); delay(100);
  uint8_t b[12] = {0};
  if (!readResp(b, 9)) { Serial.println("[F-ENERGY] FAIL"); return false; }
  uint32_t raw = ((uint32_t)getU16(b, 0) << 16) | getU16(b, 1);
  dailyOutkWh = (raw * 10.0f) / 1000.0f;
  return true;
}

bool readFanData() {
  sendFC03(0x0320, 1); delay(100);
  uint8_t b[10] = {0};
  if (!readResp(b, 7)) { Serial.println("[G-FAN] FAIL"); return false; }
  fanSpeedPct = getU16(b, 0);
  return true;
}

bool readTempData() {
  sendFC03(0x0330, 3); delay(100);
  uint8_t b[14] = {0};
  if (!readResp(b, 11)) { Serial.println("[E-TEMP] FAIL"); return false; }
  invTempC = getS16(b, 1) * 0.1f;
  chgTempC = getS16(b, 2) * 0.1f;
  return true;
}

bool readBatteryTypeCache() {
  sendFC03(REG_BATTERY_TYPE, 1); delay(200);
  uint8_t b[10] = {0};
  if (!readResp(b, 7)) { Serial.println("[BT-CACHE] FAIL"); return false; }
  uint16_t raw = getU16(b, 0);
  if (raw <= 4) cachedBatteryType = (int8_t)raw;
  return true;
}

struct SettingDef { const char *key; uint16_t reg; };
static const SettingDef SETTINGS_MAP[] = {
  { "output_priority",        REG_OUTPUT_PRIORITY     },
  { "grid_charge_current",    REG_GRID_CHARGE_CURRENT },
  { "max_charge_current",     REG_MAX_CHARGE_CURRENT  },
  { "battery_type",           REG_BATTERY_TYPE        },
  { "battery_low_cutoff",     REG_BAT_LOW_CUTOFF      },
  { "battery_undervoltage",   REG_BAT_UNDERVOLTAGE    },
  { "cv_point",               REG_CV_POINT            },
  { "float_voltage",          REG_FLOAT_VOLTAGE       },
  { "switch_to_grid_voltage", REG_SWITCH_TO_GRID_V    },
  { "return_to_battery_v",    REG_RETURN_TO_BAT_V     },
  { "grid_feed_in_enable",    REG_GRID_FEED_IN        },
};
static const uint8_t SETTINGS_COUNT = sizeof(SETTINGS_MAP) / sizeof(SETTINGS_MAP[0]);

uint16_t findRegByKey(const char *key) {
  for (uint8_t i = 0; i < SETTINGS_COUNT; i++) if (strcmp(SETTINGS_MAP[i].key, key) == 0) return SETTINGS_MAP[i].reg;
  return 0;
}

void readSettingRegister(const char *key, uint16_t reg) {
  sendFC03(reg, 1); delay(200);
  uint8_t b[10] = {0};
  if (!readResp(b, 7)) { pubError(key, "Modbus read timeout"); return; }
  pubStatus(key, reg, getU16(b, 0));
}

void handleSettingsGet(const char *payload) {
  if (strcmp(payload, "all") == 0) {
    for (uint8_t i = 0; i < SETTINGS_COUNT; i++) {
      readSettingRegister(SETTINGS_MAP[i].key, SETTINGS_MAP[i].reg);
      delay(500);
      mqttClient.loop();
    }
    return;
  }
  uint16_t reg = findRegByKey(payload);
  if (reg == 0) {
    char msg[128];
    snprintf(msg, sizeof(msg), "{\"error\":\"Unknown key\",\"received\":\"%s\"}", payload);
    mqttClient.publish(TOPIC_SETTINGS_ERROR, msg);
    return;
  }
  readSettingRegister(payload, reg);
}

bool isValidStep01V(float fval, int32_t &out) {
  if (fval < 0.0f) return false;
  float m = fval * 10.0f;
  int32_t r = (int32_t)(m + 0.5f);
  if (fabsf(m - (float)r) > 0.01f) return false;
  out = r;
  return true;
}

bool isValidStep1V(float fval, int32_t &out) {
  if (fval < 0.0f) return false;
  float r = roundf(fval);
  if (fabsf(fval - r) > 0.001f) return false;
  out = (int32_t)r;
  return true;
}

bool checkBatTypeWritable(const char *key) {
  if (cachedBatteryType < 0) {
    pubError(key, "Battery type cache not ready. Send: inverter/settings/get → battery_type first.");
    return false;
  }
  if (cachedBatteryType == BAT_AGM || cachedBatteryType == BAT_FLOODED) {
    char msg[220];
    snprintf(msg, sizeof(msg),
      "{\"error\":\"Parameter fixed for AGM/Flooded. Set battery_type to 2(Ternary),3(LiFePO4),4(Custom) first.\",\"key\":\"%s\",\"current_battery_type\":%d}",
      key, cachedBatteryType);
    mqttClient.publish(TOPIC_SETTINGS_ERROR, msg);
    return false;
  }
  return true;
}

// Return raw register bounds for 0.1V settings, not engineering volts.
bool getBatVoltageRange(uint16_t reg, int32_t &lo, int32_t &hi) {
  int bt = cachedBatteryType;
  switch (reg) {
    case REG_BAT_LOW_CUTOFF:
      if (bt == BAT_TERNARY || bt == BAT_LIFEPO4) { lo=412; hi=500; }
      else if (bt == BAT_CUSTOM)                  { lo=420; hi=540; }
      else return false;
      break;
    case REG_BAT_UNDERVOLTAGE:
      if (bt==BAT_TERNARY||bt==BAT_LIFEPO4||bt==BAT_CUSTOM) { lo=400; hi=480; }
      else return false;
      break;
    case REG_CV_POINT:
      if (bt==BAT_TERNARY||bt==BAT_LIFEPO4||bt==BAT_CUSTOM) { lo=480; hi=600; }
      else return false;
      break;
    case REG_FLOAT_VOLTAGE:
      if (bt == BAT_TERNARY || bt == BAT_LIFEPO4) { lo=500; hi=580; }
      else if (bt == BAT_CUSTOM)                  { lo=480; hi=600; }
      else return false;
      break;
    case REG_SWITCH_TO_GRID_V:
      if (bt==BAT_AGM||bt==BAT_FLOODED) { lo=440; hi=520; }
      else                              { lo=400; hi=500; }
      break;
    case REG_RETURN_TO_BAT_V:
      if (bt==BAT_AGM||bt==BAT_FLOODED) { lo=480; hi=580; }
      else                              { lo=460; hi=580; }
      break;
    default:
      return false;
  }
  return true;
}

bool validate_output_priority(float fval) {
  int32_t iv; char rb[32];
  if (!isValidStep1V(fval, iv) || iv < 0 || iv > 3) {
    snprintf(rb, sizeof(rb), "%.4f", fval);
    pubError("output_priority", "Integer 0=GPB 1=PGB 2=PBG 3=MKS", rb);
    return false;
  }
  return writeIfChanged("output_priority", REG_OUTPUT_PRIORITY, (uint16_t)iv);
}

bool validate_grid_charge_current(float fval) {
  static const uint16_t ok[] = {2, 10, 20, 30, 40};
  int32_t iv; char rb[32];
  if (!isValidStep1V(fval, iv)) {
    snprintf(rb, sizeof(rb), "%.4f", fval);
    pubError("grid_charge_current", "Allowed: 2,10,20,30,40 A", rb);
    return false;
  }
  for (uint8_t i = 0; i < 5; i++) if ((uint16_t)iv == ok[i]) return writeIfChanged("grid_charge_current", REG_GRID_CHARGE_CURRENT, (uint16_t)iv);
  snprintf(rb, sizeof(rb), "%ld", (long)iv);
  pubError("grid_charge_current", "Not in {2,10,20,30,40}", rb);
  return false;
}

bool validate_max_charge_current(float fval) {
  int32_t iv; char rb[32];
  if (!isValidStep1V(fval, iv) || iv < 0 || iv > 5) {
    snprintf(rb, sizeof(rb), "%.4f", fval);
    pubError("max_charge_current", "Index 0-5", rb);
    return false;
  }
  return writeIfChanged("max_charge_current", REG_MAX_CHARGE_CURRENT, (uint16_t)iv);
}

bool validate_battery_type(float fval) {
  int32_t iv; char rb[32];
  if (!isValidStep1V(fval, iv) || iv < 0 || iv > 4) {
    snprintf(rb, sizeof(rb), "%.4f", fval);
    pubError("battery_type", "0=AGM 1=Flooded 2=Ternary 3=LiFePO4 4=Custom", rb);
    return false;
  }
  bool ok = writeIfChanged("battery_type", REG_BATTERY_TYPE, (uint16_t)iv);
  if (ok && !g_alreadySet) cachedBatteryType = (int8_t)iv;
  return ok;
}

// Accept MQTT volts (e.g. 54.4) and scale to raw 0.1V register units.
bool validate_BatVoltage(const char *key, uint16_t reg, float fval) {
  bool needsTypeGuard = (reg == REG_BAT_LOW_CUTOFF  || reg == REG_BAT_UNDERVOLTAGE || reg == REG_CV_POINT || reg == REG_FLOAT_VOLTAGE);
  if (needsTypeGuard && !checkBatTypeWritable(key)) return false;
  if (cachedBatteryType < 0) {
    pubError(key, "Battery type cache not ready. Send: inverter/settings/get -> battery_type first.");
    return false;
  }
  int32_t scaled; char rb[32];
  if (!isValidStep01V(fval, scaled)) {
    snprintf(rb, sizeof(rb), "%.5f", fval);
    pubError(key, "Resolution must be 0.1V (e.g. 52.4 not 52.41)", rb);
    return false;
  }
  int32_t lo, hi;
  if (!getBatVoltageRange(reg, lo, hi)) {
    pubError(key, "Range unavailable for this battery type");
    return false;
  }
  if (scaled < lo || scaled > hi) {
    char reason[100];
    snprintf(reason, sizeof(reason), "Out of range [%.1f-%.1fV] for battery type %d", lo / 10.0f, hi / 10.0f, cachedBatteryType);
    snprintf(rb, sizeof(rb), "%.1f", fval);
    pubError(key, reason, rb);
    return false;
  }
  return writeIfChanged(key, reg, (uint16_t)scaled);
}

bool validate_grid_feed_in(float fval) {
  int32_t iv; char rb[32];
  if (!isValidStep1V(fval, iv) || (iv != 0 && iv != 1)) {
    snprintf(rb, sizeof(rb), "%.4f", fval);
    pubError("grid_feed_in_enable", "0=Disable 1=Enable", rb);
    return false;
  }
  return writeIfChanged("grid_feed_in_enable", REG_GRID_FEED_IN, (uint16_t)iv);
}

bool parseSettingsJson(const char *payload, char *keyOut, size_t keyLen, float &valOut) {
  const char *firstColon = strchr(payload, ':');
  if (!firstColon) return false;
  const char *afterVal = firstColon + 1;
  while (*afterVal == ' ' || *afterVal == '\t') afterVal++;
  while (*afterVal && *afterVal != ',' && *afterVal != '}') afterVal++;
  if (*afterVal == ',') return false;

  const char *ks = strchr(payload, '"');
  if (!ks) return false; ks++;
  const char *ke = strchr(ks, '"');
  if (!ke) return false;
  size_t klen = (size_t)(ke - ks);
  if (klen == 0 || klen >= keyLen) return false;
  memcpy(keyOut, ks, klen);
  keyOut[klen] = '\0';

  const char *vs = strchr(ke + 1, ':');
  if (!vs) return false; vs++;
  while (*vs == ' ' || *vs == '\t') vs++;
  char *end;
  valOut = strtof(vs, &end);
  if (end == vs) return false;
  return true;
}

void handleSettingsSet(const char *payload) {
  g_alreadySet = false;
  char key[48];
  float fval;
  if (!parseSettingsJson(payload, key, sizeof(key), fval)) {
    char msg[180];
    snprintf(msg, sizeof(msg), "{\"error\":\"Invalid JSON. Use single-key: {\\\"key\\\":value}\",\"received\":\"%s\"}", payload);
    mqttClient.publish(TOPIC_SETTINGS_ERROR, msg);
    return;
  }

  bool ok = false;
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
    char msg[128];
    snprintf(msg, sizeof(msg), "{\"error\":\"Unknown key\",\"received\":\"%s\"}", key);
    mqttClient.publish(TOPIC_SETTINGS_ERROR, msg);
    return;
  }

  if (ok && !g_alreadySet) {
    char msg[128];
    snprintf(msg, sizeof(msg), "{\"result\":\"write_ok\",\"key\":\"%s\",\"value\":%.4f}", key, fval);
    mqttClient.publish(TOPIC_SETTINGS_STATUS, msg);
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  if (length == 0 || length > 200) return;
  char buf[201];
  memcpy(buf, payload, length);
  buf[length] = '\0';
  Serial.printf("[MQTT] %s → %s\n", topic, buf);
  if (strcmp(topic, TOPIC_SETTINGS_GET) == 0) handleSettingsGet(buf);
  else if (strcmp(topic, TOPIC_SETTINGS_SET) == 0) handleSettingsSet(buf);
}

void connectWiFi() {
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[WiFi] Connecting");
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500); Serial.print('.'); tries++;
  }
  if (WiFi.status() == WL_CONNECTED) Serial.printf("\n[WiFi] Connected -- IP: %s\n", WiFi.localIP().toString().c_str());
  else Serial.println("\n[WiFi] FAILED -- will retry in loop");
}

void connectMQTT() {
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);
  Serial.print("[MQTT] Connecting");
  uint8_t tries = 0;
  while (!mqttClient.connected() && tries < 5) {
    bool ok = (strlen(MQTT_USER) > 0) ? mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS) : mqttClient.connect(MQTT_CLIENT_ID);
    if (ok) {
      Serial.println(" connected.");
      mqttClient.publish(TOPIC_SETTINGS_SET, "", true);
      mqttClient.subscribe(TOPIC_SETTINGS_GET);
      mqttClient.subscribe(TOPIC_SETTINGS_SET);
    } else {
      Serial.printf(" failed (rc=%d), retry\n", mqttClient.state());
      delay(2000);
    }
    tries++;
  }
}

void printData() {
  Serial.println("---- Inverter Data ----------------------------------------");
  Serial.printf("Mains:    %.1fV  %.2fHz  Grid: %dW\n", mainsV, mainsHz, gridPowerW);
  Serial.printf("Battery:  %.1fV  SOC:%d%%  Chg:%.1fA  Dis:%.1fA\n", batV, batSOC, batChgA, batDisA);
  Serial.printf("Bus:      %.1fV  ChgStatus: 0x%04X\n", busVoltage, chgStatus);
  Serial.printf("PV1:      %.1fV  %.2fA  %dW\n", pv1V, pv1A, pv1W);
  Serial.printf("AC Load:  %dW\n", acLoadW);
  Serial.printf("Energy:   Today=%.1fkWh  Total=%.1fkWh  OutDay=%.2fkWh\n", dailykWh, totalkWh, dailyOutkWh);
  Serial.printf("Temp:     Inv=%.1fC  Chg=%.1fC\n", invTempC, chgTempC);
  Serial.printf("Fan:      %d%%\n", fanSpeedPct);
  Serial.println("-----------------------------------------------------------");
}

void publishAll() {
  mqttPublishF("mains/voltage",        mainsV,      1);
  mqttPublishF("mains/frequency",      mainsHz,     2);
  mqttPublishI("mains/grid_power_w",   gridPowerW    );
  mqttPublishF("battery/voltage",      batV,        1);
  mqttPublishF("battery/charge_a",     batChgA,     1);
  mqttPublishF("battery/discharge_a",  batDisA,     1);
  mqttPublishF("battery/current",      batCurrent,  1);
  mqttPublishI("battery/soc",          batSOC        );
  mqttPublishF("battery/bus_v",        busVoltage,  1);
  mqttPublishI("battery/chg_status",   chgStatus     );
  mqttPublishF("pv1/voltage",          pv1V,        1);
  mqttPublishF("pv1/current",          pv1A,        2);
  mqttPublishI("pv1/power_w",          pv1W          );
  mqttPublishF("energy/daily_kwh",     dailykWh,    1);
  mqttPublishF("energy/total_kwh",     totalkWh,    1);
  mqttPublishF("energy/daily_out_kwh", dailyOutkWh, 2);
  mqttPublishI("load/ac_w",            acLoadW       );
  mqttPublishF("temp/inverter_c",      invTempC,    1);
  mqttPublishF("temp/charger_c",       chgTempC,    1);
  mqttPublishI("fan/speed_pct",        fanSpeedPct   );
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("[BOOT] Inv_monitor_conf starting");
  modbusSerial.begin(BAUD_RATE);
  delay(100);
  Serial.printf("[BOOT] Modbus SoftwareSerial RX=GPIO%d TX=GPIO%d @ %d baud\n", RS232_RX_PIN, RS232_TX_PIN, BAUD_RATE);
  WiFi.persistent(false);
  WiFi.setAutoConnect(false);
  connectWiFi();
  connectMQTT();
  delay(500);
  Serial.println("[BOOT] Reading battery type cache...");
  if (!readBatteryTypeCache()) Serial.println("[BOOT] Battery type cache FAILED -- voltage writes blocked");
  Serial.printf("[BOOT] Cache: battType=%d\n", cachedBatteryType);
  Serial.println("[BOOT] Ready.");
}

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
    readMainsData();
    readBatteryData();
    readBattExtData();
    readPVData();
    readLoadData();
    readOutputEnergy();
    readFanData();
    readTempData();
    printData();
    publishAll();
  }
}
