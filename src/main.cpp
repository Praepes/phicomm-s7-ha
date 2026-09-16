#include <Arduino.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// ============================================================
//  Phicomm S7 Body Fat Scale — ESP8266 Replacement Firmware
//  v1.0  —  Home Assistant MQTT Integration
//
//  Features: Weight / Impedance / Body Fat / Water / BMI /
//            Fat-Free Mass / BMR / Muscle / Bone Mass
//  Hardware: CS1258 BIA AFE + MCU UART + Heater + Web UI
//  License:  MIT
// ============================================================
#define FW_VERSION "1.0"

#if __has_include("config.h")
#include "config.h"
#endif

#ifndef S7_DEVICE_ID
#define S7_DEVICE_ID "phicomm_s7"
#endif
#ifndef S7_DEVICE_NAME
#define S7_DEVICE_NAME "Phicomm S7"
#endif

static constexpr uint8_t PIN_CS  = 16;
static constexpr uint8_t PIN_CLK = 14;
static constexpr uint8_t PIN_DIO = 12;

static constexpr uint32_t EEPROM_MAGIC      = 0x53375631; // S7V1
static constexpr uint16_t DEFAULT_CAL_OFFSET = 1100;
static constexpr uint32_t MQTT_RECONNECT_MS  = 5000;
static constexpr uint32_t WIFI_RETRY_MS      = 10000;
static constexpr uint8_t  UART_MAX_FRAME     = 40;
static constexpr uint32_t MCU_TIMEOUT_MS     = 5000;
static constexpr float    WEIGHT_ON_THRESH   = 3.0f;
static constexpr uint32_t IMPEDANCE_COOL_MS  = 20000;
static constexpr float    WEIGHT_STABLE_DELTA= 0.3f;   // kg — weight change threshold
static constexpr uint32_t WEIGHT_STABLE_MS   = 3000;   // ms — how long weight must be stable

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
ESP8266WebServer  web(80);
ESP8266HTTPUpdateServer httpUpdater;

struct Config {
  uint32_t magic;
  char     ssid[33];
  char     pass[65];
  char     mqttHost[65];
  uint16_t mqttPort;
  char     mqttUser[33];
  char     mqttPass[65];
  char     otaPass[33];
  uint8_t  male;
  uint8_t  age;
  float    heightM;
  uint32_t mcuBaud;
  uint8_t  weightHi;
  uint8_t  weightLo;
  float    weightDiv;
  float    weightOffset;
  float    zScale;
  float    zOffset;
  int8_t   heaterPin;
  uint8_t  heaterHigh;
  uint16_t heaterMaxSec;
};

struct McuFrame { uint8_t buf[UART_MAX_FRAME]; uint8_t pos, expected; bool synced; };
struct ChResult { uint32_t raw; uint16_t ohm; };

struct Meas {
  bool     mcuAlive;
  uint32_t lastFrameMs;
  uint8_t  lastCmd10[UART_MAX_FRAME];
  uint8_t  lastCmd10Len;
  uint16_t weightRaw;
  float    weightKg;
  bool     csOk;
  uint8_t  csInitAttempts;
  char     csLog[600];
  uint32_t shortAdc, calA, calB;
  ChResult lrLeg, rlLeg, lrHand, rLeg, lLeg;
  ChResult lhLl, lhRl, rhLl, rhRl;
  uint16_t bodyRes, z1, z2, z3, z4, z5;
  bool     bodyValid;
  float    zEffOhm;
  float    bmi, ffmKg, fatPct, waterPct, bmr;
  float    musKg, musPct, boneKg;
  uint32_t lastImpedMs, lastScanDurMs;
};

Config cfg; Meas m; McuFrame rx;
uint32_t lastMqttMs, lastWifiMs, heaterOffMs;
bool heaterOn, apOn, otaOn;

// Weight debounce state
static bool     personOnScale    = false;
static bool     measurementDone  = false;
static float    stableWeight     = 0;
static uint32_t weightStableSince= 0;

static void cpStr(char *d, size_t n, const char *s) { snprintf(d, n, "%s", s ? s : ""); }

static String esc(const String &s) {
  String o; o.reserve(s.length() + 8);
  for (unsigned i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') o += F("&amp;"); else if (c == '<') o += F("&lt;");
    else if (c == '>') o += F("&gt;"); else if (c == '"') o += F("&quot;");
    else o += c;
  }
  return o;
}

static String topicBase() { return String("s7/") + S7_DEVICE_ID; }

static String hexDump(const uint8_t *d, uint8_t len) {
  String s; s.reserve(len * 3);
  for (uint8_t i = 0; i < len; i++) { if (i) s += ' '; char h[4]; snprintf(h, sizeof(h), "%02X", d[i]); s += h; }
  return s;
}

// ============================================================
//  CS1258 bit-bang driver
// ============================================================
static void csDelay(uint16_t us) { delayMicroseconds(us); }

static void csBegin() {
  digitalWrite(PIN_CLK, LOW); csDelay(800);
  digitalWrite(PIN_CS, LOW);  csDelay(2);
}
static void csEnd() {
  digitalWrite(PIN_CS, HIGH); csDelay(2);
  digitalWrite(PIN_CLK, HIGH);
  pinMode(PIN_DIO, INPUT);    delay(2);
}

static void csWriteByte(uint8_t v) {
  pinMode(PIN_DIO, OUTPUT);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(PIN_DIO, (v & 0x80) ? HIGH : LOW);
    csDelay(1); digitalWrite(PIN_CLK, HIGH);
    csDelay(1); digitalWrite(PIN_CLK, LOW);
    v <<= 1; csDelay(1);
  }
  csDelay(2);
}

static uint8_t csReadByte() {
  uint8_t v = 0; pinMode(PIN_DIO, INPUT);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(PIN_CLK, HIGH); v <<= 1; csDelay(1);
    digitalWrite(PIN_CLK, LOW);
    if (digitalRead(PIN_DIO)) v |= 1;
    csDelay(1);
  }
  return v;
}

static void csWriteReg(uint8_t reg, uint8_t val) {
  csBegin(); csWriteByte(0x80 | reg); csDelay(2);
  csWriteByte(val); csEnd();
}

static uint8_t csReadReg(uint8_t reg) {
  csBegin(); csWriteByte(reg);
  uint8_t v = csReadByte(); csEnd(); return v;
}

static bool csWriteVerify(uint8_t reg, uint8_t val, uint8_t retries = 2) {
  for (uint8_t i = 0; i < retries; i++) {
    csWriteReg(reg, val); delay(1);
    if (csReadReg(reg) == val) return true;
    delay(1);
  }
  return false;
}

static uint32_t csReadAdc24() {
  csBegin(); csWriteByte(0x09); csDelay(2);
  uint8_t b0 = csReadByte(), b1 = csReadByte(), b2 = csReadByte();
  csEnd();
  return ((uint32_t)(b0 ^ 0x80) << 16) | ((uint32_t)b1 << 8) | b2;
}

static void csReadAdc24Raw(uint8_t *b0, uint8_t *b1, uint8_t *b2) {
  csBegin(); csWriteByte(0x09); csDelay(2);
  *b0 = csReadByte(); *b1 = csReadByte(); *b2 = csReadByte();
  csEnd();
}

static uint32_t csSampleAvg(uint8_t n, uint16_t jump) {
  uint32_t sum = 0, prev = 0;
  uint8_t acc = 0, kept = 0;
  for (uint8_t t = 0; t < 110 && acc < n; t++) {
    delay(3); uint32_t r = csReadAdc24();
    if (acc > 0) {
      uint32_t d = r > prev ? r - prev : prev - r;
      if (d > jump) { prev = r; continue; }
    }
    prev = r; acc++;
    if (acc > 4) { sum += r; kept++; }
    yield();
  }
  return kept ? sum / kept : prev;
}

static bool csResetInit() {
  char *L = m.csLog; int P = 0;
  #define LOG(fmt, ...) P += snprintf(L+P, sizeof(m.csLog)-P, fmt, ##__VA_ARGS__)

  pinMode(PIN_CS, OUTPUT); pinMode(PIN_CLK, OUTPUT); pinMode(PIN_DIO, INPUT);
  digitalWrite(PIN_CS, HIGH); digitalWrite(PIN_CLK, HIGH);
  delay(100);

  csWriteReg(0xEA, 0x96);
  delay(10);
  uint8_t r0A = csReadReg(0x0A);
  LOG("r0A=%02X ", r0A);
  if (!(r0A & 0x40)) { LOG("FAIL:r0A"); return false; }

  if (!csWriteVerify(0x59, 0x6A)) { LOG("FAIL:59"); return false; }
  LOG("59=6A ");

  uint8_t r12 = csReadReg(0x12);
  uint8_t r13 = csReadReg(0x13);
  LOG("r12=%02X r13=%02X ", r12, r13);

  struct { uint8_t reg; uint8_t val; } steps[] = {
    {0x59, 0xA6}, {0x5A, 0x5A}, {0x12, r13}, {0x13, r13}, {0x10, 0xBF}
  };
  for (auto &s : steps) {
    if (!csWriteVerify(s.reg, s.val)) {
      LOG("FAIL:%02X", s.reg);
      return false;
    }
  }
  LOG("init=OK");
  #undef LOG
  return true;
}

static bool csPreBaseline() {
  char *L = m.csLog; int P = strlen(L);
  #define LOG(fmt, ...) P += snprintf(L+P, sizeof(m.csLog)-P, fmt, ##__VA_ARGS__)

  if (!csWriteVerify(0x05, 0x42)) { LOG(" FAIL:05"); return false; }
  csWriteReg(0x00, 0xCA);
  delay(20);
  if (!csWriteVerify(0x02, 0x4E)) { LOG(" FAIL:02"); return false; }
  LOG(" pre=OK");
  #undef LOG
  return true;
}

static uint32_t csReadBaseline(uint8_t reg8val, char *L, int *P) {
  csWriteReg(0x08, reg8val);
  delay(2);
  csWriteReg(0x07, 0x00);
  delay(2);
  csWriteReg(0x00, 0xCF);
  delay(5);

  uint8_t db0, db1, db2;
  csReadAdc24Raw(&db0, &db1, &db2);
  *P += snprintf(L + *P, 600 - *P, "\n  bl_%02X: 1st=%02X%02X%02X",
    reg8val, db0, db1, db2);

  delay(5);
  csReadAdc24Raw(&db0, &db1, &db2);
  *P += snprintf(L + *P, 600 - *P, " 2nd=%02X%02X%02X", db0, db1, db2);

  uint32_t v = csSampleAvg(8, 500);
  *P += snprintf(L + *P, 600 - *P, " avg=%lu", (unsigned long)v);

  csWriteReg(0x00, 0xCA);
  delay(2);
  return v;
}

static uint16_t adcToOhm(uint32_t raw, uint32_t shortA, uint32_t calA, uint32_t calB) {
  int32_t span = (int32_t)(calB - calA) - (int32_t)DEFAULT_CAL_OFFSET;
  if (span > 100) {
    if (raw <= shortA) return 0;
    float z = (float)(raw - shortA) * 7000.0f / (float)span;
    if (z < 0) return 0;
    if (z > 65535) return 65535;
    return (uint16_t)z;
  }
  // Fallback: CS1258 baseline calibration modes return zero on this chip.
  // Direct mapping from ADC delta to ohms.
  static const uint32_t ADC_ZERO = 8388608;
  if (raw <= ADC_ZERO) return 0;
  uint32_t delta = raw - ADC_ZERO;
  uint32_t ohm = delta / 512;
  if (ohm > 65535) return 65535;
  return (uint16_t)ohm;
}

static ChResult csReadChannel(uint8_t ch) {
  ChResult r = {0, 0};
  csWriteVerify(0x08, 0x03);
  csWriteVerify(0x07, ch);
  csWriteReg(0x00, 0xCF);
  delay(1);
  r.raw = csSampleAvg(20, 1000);
  r.ohm = adcToOhm(r.raw, m.shortAdc, m.calA, m.calB);
  csWriteReg(0x00, 0xCA);
  delay(2);
  return r;
}

static uint16_t half2(uint16_t a, uint16_t b) { return (uint16_t)(((uint32_t)a + b) / 2); }
static uint16_t halfS(int32_t v) { return v <= 0 ? 0 : (v > 131070 ? 65535 : (uint16_t)(v / 2)); }

static void csDoImpedanceScan() {
  uint32_t t0 = millis();
  memset(m.csLog, 0, sizeof(m.csLog));
  m.csInitAttempts++;

  m.csOk = csResetInit();
  if (!m.csOk) { m.lastScanDurMs = millis()-t0; m.lastImpedMs = millis(); return; }

  if (!csPreBaseline()) {
    m.csOk = false;
    m.lastScanDurMs = millis()-t0; m.lastImpedMs = millis(); return;
  }

  int lpos = strlen(m.csLog);
  lpos += snprintf(m.csLog+lpos, sizeof(m.csLog)-lpos, "\n--- baselines ---");

  m.shortAdc = csReadBaseline(0x33, m.csLog, &lpos);
  m.calA     = csReadBaseline(0x13, m.csLog, &lpos);
  m.calB     = csReadBaseline(0x23, m.csLog, &lpos);

  lpos += snprintf(m.csLog+lpos, sizeof(m.csLog)-lpos,
    "\nspan=%ld", (long)((int32_t)(m.calB-m.calA)-(int32_t)DEFAULT_CAL_OFFSET));

  m.lrLeg  = csReadChannel(0xEB); m.rlLeg  = csReadChannel(0xBE);
  m.lrHand = csReadChannel(0x14); m.rLeg   = csReadChannel(0xDB);
  m.lLeg   = csReadChannel(0x8E); m.lhLl   = csReadChannel(0x3C);
  m.lhRl   = csReadChannel(0x28); m.rhLl   = csReadChannel(0x7D);
  m.rhRl   = csReadChannel(0x69);

  m.bodyRes = half2(m.rlLeg.ohm, m.lrLeg.ohm);
  m.z1 = halfS((int32_t)m.lrHand.ohm + m.lhRl.ohm - m.rhRl.ohm);
  m.z2 = halfS((int32_t)m.rhRl.ohm   + m.lrHand.ohm - m.lhRl.ohm);
  m.z3 = halfS((int32_t)m.rhLl.ohm   - m.rhRl.ohm + m.bodyRes);
  m.z4 = halfS((int32_t)m.rhRl.ohm   - m.rhLl.ohm + m.bodyRes);
  m.z5 = (uint16_t)(((uint32_t)m.rhLl.ohm + m.lhRl.ohm) / 50);

  m.bodyValid = m.bodyRes >= 50 && m.bodyRes <= 3000;
  m.zEffOhm = m.bodyValid ? m.bodyRes * cfg.zScale + cfg.zOffset : 0;
  m.lastImpedMs = millis();
  m.lastScanDurMs = millis() - t0;
}

static void calcBodyComp() {
  float h = cfg.heightM, hCm = h*100, w = m.weightKg, z = m.zEffOhm;
  float age = (float)cfg.age;
  bool male = cfg.male;
  if (w <= 0 || h <= 0) return;
  m.bmi = w / (h * h);
  if (z <= 0) return;

  // LBM (bodymiscale / Xiaomi)
  float lbm = (hCm * 9.058f / 100.0f) * (hCm / 100.0f)
            + w * 0.32f + 12.226f - z * 0.0068f - age * 0.0542f;
  if (lbm > w * 0.98f) lbm = w * 0.98f;
  if (lbm < 0) lbm = 0;
  m.ffmKg = lbm;

  // Fat% (Xiaomi / Zepp Life)
  float adjust, coeff;
  if (male) {
    adjust = 0.8f;
    coeff = (w < 61.0f) ? 0.98f : 1.0f;
  } else {
    adjust = (age <= 49.0f) ? 9.25f : 7.25f;
    coeff = 1.0f;
    if (w > 60.0f) coeff = 0.96f * ((hCm > 160.0f) ? 1.03f : 1.0f);
    else if (w < 50.0f) coeff = 1.02f * ((hCm > 160.0f) ? 1.03f : 1.0f);
  }
  m.fatPct = (1.0f - ((lbm - adjust) * coeff / w)) * 100.0f;
  if (m.fatPct < 5) m.fatPct = 5;
  if (m.fatPct > 75) m.fatPct = 75;

  // Water%
  m.waterPct = (100.0f - m.fatPct) * 0.7f;
  m.waterPct *= (m.waterPct <= 50.0f) ? 1.02f : 0.98f;
  if (m.waterPct < 35) m.waterPct = 35;
  if (m.waterPct > 75) m.waterPct = 75;

  // BMR (Katch-McArdle)
  m.bmr = 370.0f + 21.6f * lbm;

  // Bone mass (bodymiscale)
  float base = male ? 0.18016894f : 0.245691014f;
  m.boneKg = -1.0f * (base - lbm * 0.05158f);
  m.boneKg += (m.boneKg > 2.2f) ? 0.1f : -0.1f;
  if (m.boneKg < 0.5f) m.boneKg = 0.5f;
  if (m.boneKg > 8.0f) m.boneKg = 8.0f;

  // Muscle mass
  m.musKg = w - (m.fatPct * 0.01f * w) - m.boneKg;
  if (m.musKg < 10) m.musKg = 10;
  m.musPct = m.musKg / w * 100.0f;
}

// ============================================================
//  MCU UART
// ============================================================
static uint8_t xorCheck(const uint8_t *d, uint8_t n) { uint8_t x=0; for(uint8_t i=0;i<n;i++) x^=d[i]; return x; }
static void mcuSend(const uint8_t *d, uint8_t len) { Serial.write(d, len); }

static void mcuSendStatus10() {
  uint8_t f[7]={0xC6,0x04,0x10,0,0,0,0}; f[3]=m.csOk?1:0; f[6]=xorCheck(f,6); mcuSend(f,7);
}

static void handleCmd10(const uint8_t *frame, uint8_t total) {
  uint8_t payLen = frame[1] > 1 ? frame[1] - 1 : 0;
  m.lastCmd10Len = total;
  memcpy(m.lastCmd10, frame, total < UART_MAX_FRAME ? total : UART_MAX_FRAME);
  if (cfg.weightHi < payLen && cfg.weightLo < payLen) {
    m.weightRaw = ((uint16_t)frame[3+cfg.weightHi]<<8) | (uint16_t)frame[3+cfg.weightLo];
    m.weightKg = (float)m.weightRaw / cfg.weightDiv + cfg.weightOffset;
    if (m.weightKg < 0) m.weightKg = 0;
  }
  mcuSendStatus10();
}

static void handleGenericC5(uint8_t cmd) {
  uint8_t f[5]={0xC6,0x02,cmd,0x00,0}; f[4]=xorCheck(f,4); mcuSend(f,5);
}

static void dispatchFrame(const uint8_t *fr, uint8_t total) {
  m.mcuAlive = true; m.lastFrameMs = millis();
  if (fr[0]==0xC5) { if (fr[2]==0x10) handleCmd10(fr,total); else handleGenericC5(fr[2]); }
}

static void uartPoll() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (!rx.synced) { if (b==0xC5||b==0xC6) { rx.buf[0]=b; rx.pos=1; rx.expected=0; rx.synced=true; } continue; }
    rx.buf[rx.pos++] = b;
    if (rx.pos==2) { rx.expected=b+3; if (rx.expected>UART_MAX_FRAME||rx.expected<4) { rx.synced=false; continue; } }
    if (rx.pos>=rx.expected && rx.expected>=4) {
      if (xorCheck(rx.buf,rx.expected-1)==rx.buf[rx.expected-1]) dispatchFrame(rx.buf,rx.expected);
      rx.synced=false;
    }
    if (rx.pos>=UART_MAX_FRAME) rx.synced=false;
  }
  if (m.mcuAlive && millis()-m.lastFrameMs > MCU_TIMEOUT_MS) m.mcuAlive=false;
}

// ============================================================
//  Config
// ============================================================
static void saveConfig() { EEPROM.put(0, cfg); EEPROM.commit(); }
static void loadConfig() {
  EEPROM.begin(512); EEPROM.get(0, cfg);
  if (cfg.magic != EEPROM_MAGIC) {
    memset(&cfg, 0, sizeof(cfg)); cfg.magic = EEPROM_MAGIC;
    cfg.mqttPort = 1883;
    cfg.male = 1; cfg.age = 25; cfg.heightM = 1.75f;
    cfg.mcuBaud = 115200; cfg.weightHi = 0; cfg.weightLo = 1;
    cfg.weightDiv = 100; cfg.weightOffset = 0;
    cfg.zScale = 1; cfg.zOffset = 0;
    cfg.heaterPin = -1; cfg.heaterHigh = 1; cfg.heaterMaxSec = 60;
    saveConfig();
  }
}
static void heaterSet(bool on) {
  if (cfg.heaterPin>=0&&cfg.heaterPin<=16) {
    pinMode((uint8_t)cfg.heaterPin,OUTPUT);
    digitalWrite((uint8_t)cfg.heaterPin,(on==(cfg.heaterHigh!=0))?HIGH:LOW);
    heaterOn=on; heaterOffMs=on?millis()+(uint32_t)cfg.heaterMaxSec*1000UL:0;
  } else { heaterOn=false; heaterOffMs=0; }
}

// ============================================================
//  MQTT
// ============================================================
static String stateJson() {
  // Use last stable weight after stepping off so HA retains the reading
  float wt = m.weightKg;
  if (wt < WEIGHT_ON_THRESH && stableWeight >= WEIGHT_ON_THRESH)
    wt = stableWeight;
  char buf[1500];
  snprintf(buf, sizeof(buf),
    "{\"weight_kg\":%.2f,\"weight_raw\":%u,"
    "\"mcu_alive\":%u,\"impedance_ohm\":%.1f,"
    "\"body_fat_percent\":%.1f,\"water_percent\":%.1f,"
    "\"bmi\":%.1f,\"fat_free_mass_kg\":%.2f,"
    "\"bmr_kcal\":%.0f,\"skeletal_muscle_percent\":%.1f,"
    "\"bone_mass_kg\":%.2f,\"body_res_ohm\":%u,"
    "\"z1\":%u,\"z2\":%u,\"z3\":%u,\"z4\":%u,\"z5\":%u,"
    "\"cs_ok\":%u,\"short_adc\":%lu,\"cal_a\":%lu,\"cal_b\":%lu,"
    "\"scan_ms\":%lu,\"body_valid\":%u,\"heater\":%u,\"rssi\":%d,"
    "\"age\":%u,\"height_m\":%.2f,\"sex\":\"%s\"}",
    wt, m.weightRaw, m.mcuAlive?1:0, m.zEffOhm,
    m.fatPct, m.waterPct, m.bmi, m.ffmKg, m.bmr, m.musPct,
    m.boneKg, m.bodyRes, m.z1,m.z2,m.z3,m.z4,m.z5,
    m.csOk?1:0,
    (unsigned long)m.shortAdc,(unsigned long)m.calA,(unsigned long)m.calB,
    (unsigned long)m.lastScanDurMs, m.bodyValid?1:0, heaterOn?1:0,
    WiFi.status()==WL_CONNECTED?WiFi.RSSI():0,
    cfg.age, cfg.heightM,
    cfg.male ? "\xe7\x94\xb7" : "\xe5\xa5\xb3"); // 男 / 女
  return String(buf);
}

static void pubEntity(const char *domain, const char *key, const char *json) {
  String t = String("homeassistant/") + domain + "/" + S7_DEVICE_ID + "/" + key + "/config";
  mqtt.publish(t.c_str(), json, true);
  yield();
}

static void pubDiscovery() {
  String base = topicBase();
  String avail = base + "/availability";
  String state = base + "/state";
  char p[700];

  // Shared device block
  const char *dev =
    ",\"device\":{\"identifiers\":[\"" S7_DEVICE_ID "\"],"
    "\"name\":\"" S7_DEVICE_NAME "\",\"manufacturer\":\"Phicomm\",\"model\":\"S7\","
    "\"sw_version\":\"" FW_VERSION "\"}";

  // --- Sensors ---
  struct SD { const char *key, *name, *unit, *dc, *tpl, *ic; };
  SD sensors[] = {
    {"weight_kg",       "\xe4\xbd\x93\xe9\x87\x8d",  "kg",   "weight",          "weight_kg",               "mdi:scale-bathroom"},
    {"bmi",             "BMI",     "",     "",                "bmi",                     "mdi:human"},
    {"body_fat_percent","\xe4\xbd\x93\xe8\x84\x82\xe7\x8e\x87","%",    "",                "body_fat_percent",        "mdi:percent"},
    {"water_percent",   "\xe6\xb0\xb4\xe5\x88\x86\xe7\x8e\x87",   "%",    "",                "water_percent",           "mdi:water-percent"},
    {"impedance_ohm",   "\xe9\x98\xbb\xe6\x8a\x97","\xce\xa9","",          "impedance_ohm",           "mdi:flash-triangle"},
    {"fat_free_mass_kg","\xe5\x8e\xbb\xe8\x84\x82\xe4\xbd\x93\xe9\x87\x8d","kg","",             "fat_free_mass_kg",        "mdi:arm-flex"},
    {"bmr_kcal",        "\xe5\x9f\xba\xe7\xa1\x80\xe4\xbb\xa3\xe8\xb0\xa2",     "kcal", "",                "bmr_kcal",                "mdi:fire"},
    {"skeletal_muscle_percent","\xe8\x82\x8c\xe8\x82\x89\xe7\x8e\x87","%","",              "skeletal_muscle_percent", "mdi:arm-flex-outline"},
    {"bone_mass_kg",    "\xe9\xaa\xa8\xe9\x87\x8f","kg",  "",                "bone_mass_kg",            "mdi:bone"},
    {"rssi",            "WiFi RSSI","dBm", "signal_strength","rssi",                    "mdi:wifi"},
  };

  for (auto &s : sensors) {
    snprintf(p, sizeof(p),
      "{\"name\":\"%s\",\"unique_id\":\"%s_%s\","
      "\"state_topic\":\"%s\",\"availability_topic\":\"%s\","
      "\"value_template\":\"{{ value_json.%s }}\"",
      s.name, S7_DEVICE_ID, s.key,
      state.c_str(), avail.c_str(), s.tpl);
    String ps(p);
    if (s.unit[0]) { ps += ",\"unit_of_measurement\":\""; ps += s.unit; ps += "\""; }
    if (s.dc[0])   { ps += ",\"device_class\":\""; ps += s.dc; ps += "\""; }
    ps += ",\"state_class\":\"measurement\"";
    ps += ",\"icon\":\""; ps += s.ic; ps += "\"";
    ps += dev;
    ps += "}";
    pubEntity("sensor", s.key, ps.c_str());
  }

  // --- Binary sensors ---
  snprintf(p, sizeof(p),
    "{\"name\":\"MCU \xe5\x9c\xa8\xe7\xba\xbf\",\"unique_id\":\"%s_mcu_alive\","
    "\"state_topic\":\"%s\",\"availability_topic\":\"%s\","
    "\"value_template\":\"{{ 'ON' if value_json.mcu_alive else 'OFF' }}\","
    "\"device_class\":\"connectivity\",\"icon\":\"mdi:chip\""
    "%s}",
    S7_DEVICE_ID, state.c_str(), avail.c_str(), dev);
  pubEntity("binary_sensor", "mcu_alive", p);

  snprintf(p, sizeof(p),
    "{\"name\":\"CS1258\",\"unique_id\":\"%s_cs_ok\","
    "\"state_topic\":\"%s\",\"availability_topic\":\"%s\","
    "\"value_template\":\"{{ 'ON' if value_json.cs_ok else 'OFF' }}\","
    "\"device_class\":\"connectivity\",\"icon\":\"mdi:integrated-circuit-chip\""
    "%s}",
    S7_DEVICE_ID, state.c_str(), avail.c_str(), dev);
  pubEntity("binary_sensor", "cs_ok", p);

  // --- Switch: heater ---
  snprintf(p, sizeof(p),
    "{\"name\":\"\xe5\x8a\xa0\xe7\x83\xad\xe5\x99\xa8\",\"unique_id\":\"%s_heater\","
    "\"state_topic\":\"%s\",\"availability_topic\":\"%s\","
    "\"command_topic\":\"%s/cmd/heater\","
    "\"value_template\":\"{{ 'ON' if value_json.heater else 'OFF' }}\","
    "\"payload_on\":\"ON\",\"payload_off\":\"OFF\","
    "\"icon\":\"mdi:radiator\""
    "%s}",
    S7_DEVICE_ID, state.c_str(), avail.c_str(), base.c_str(), dev);
  pubEntity("switch", "heater", p);

  // --- Button: scan impedance ---
  snprintf(p, sizeof(p),
    "{\"name\":\"\xe6\xb5\x8b\xe9\x87\x8f\xe9\x98\xbb\xe6\x8a\x97\",\"unique_id\":\"%s_scan_impedance\","
    "\"command_topic\":\"%s/cmd/impedance\","
    "\"availability_topic\":\"%s\","
    "\"payload_press\":\"1\","
    "\"icon\":\"mdi:flash-triangle-outline\""
    "%s}",
    S7_DEVICE_ID, base.c_str(), avail.c_str(), dev);
  pubEntity("button", "scan_impedance", p);

  // --- Number: age (box mode, not slider) ---
  snprintf(p, sizeof(p),
    "{\"name\":\"\xe5\xb9\xb4\xe9\xbe\x84\",\"unique_id\":\"%s_age\","
    "\"state_topic\":\"%s\",\"availability_topic\":\"%s\","
    "\"command_topic\":\"%s/cmd/age\","
    "\"value_template\":\"{{ value_json.age }}\","
    "\"min\":10,\"max\":99,\"step\":1,\"mode\":\"box\","
    "\"icon\":\"mdi:calendar-account\""
    "%s}",
    S7_DEVICE_ID, state.c_str(), avail.c_str(), base.c_str(), dev);
  pubEntity("number", "age", p);

  // --- Number: height (box mode, cm) ---
  snprintf(p, sizeof(p),
    "{\"name\":\"\xe8\xba\xab\xe9\xab\x98\",\"unique_id\":\"%s_height\","
    "\"state_topic\":\"%s\",\"availability_topic\":\"%s\","
    "\"command_topic\":\"%s/cmd/height\","
    "\"value_template\":\"{{ (value_json.height_m * 100) | round(0) }}\","
    "\"min\":100,\"max\":250,\"step\":1,\"mode\":\"box\","
    "\"unit_of_measurement\":\"cm\","
    "\"icon\":\"mdi:human-male-height\""
    "%s}",
    S7_DEVICE_ID, state.c_str(), avail.c_str(), base.c_str(), dev);
  pubEntity("number", "height", p);

  // --- Select: sex (Chinese options) ---
  snprintf(p, sizeof(p),
    "{\"name\":\"\xe6\x80\xa7\xe5\x88\xab\",\"unique_id\":\"%s_sex\","
    "\"state_topic\":\"%s\",\"availability_topic\":\"%s\","
    "\"command_topic\":\"%s/cmd/sex\","
    "\"value_template\":\"{{ value_json.sex }}\","
    "\"options\":[\"\xe7\x94\xb7\",\"\xe5\xa5\xb3\"],"
    "\"icon\":\"mdi:gender-male-female\""
    "%s}",
    S7_DEVICE_ID, state.c_str(), avail.c_str(), base.c_str(), dev);
  pubEntity("select", "sex", p);
}

static void pubState() {
  if (mqtt.connected())
    mqtt.publish((topicBase()+"/state").c_str(), stateJson().c_str(), true);
}
static void pubAvail(bool on) {
  if (mqtt.connected())
    mqtt.publish((topicBase()+"/availability").c_str(), on?"online":"offline", true);
}

static void mqttCb(char *topic, byte *payload, unsigned int length) {
  String t(topic), msg;
  for (unsigned i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  String b = topicBase();

  if (t == b + "/cmd/impedance") {
    csDoImpedanceScan(); calcBodyComp(); pubState();
  }
  else if (t == b + "/cmd/heater") {
    heaterSet(msg == "ON" || msg == "1"); pubState();
  }
  else if (t == b + "/cmd/age") {
    int v = msg.toInt();
    if (v >= 10 && v <= 99) { cfg.age = (uint8_t)v; saveConfig(); calcBodyComp(); pubState(); }
  }
  else if (t == b + "/cmd/height") {
    float v = msg.toFloat();
    if (v > 3.0f) v /= 100.0f;  // accept cm (>3) or m (<3)
    if (v >= 1.0f && v <= 2.5f) { cfg.heightM = v; saveConfig(); calcBodyComp(); pubState(); }
  }
  else if (t == b + "/cmd/sex") {
    // Accept: "男", "male", "1" → male;  "女", "female", "0" → female
    if (msg == "\xe7\x94\xb7" || msg == "male" || msg == "1") {
      cfg.male = 1; saveConfig(); calcBodyComp(); pubState();
    } else if (msg == "\xe5\xa5\xb3" || msg == "female" || msg == "0") {
      cfg.male = 0; saveConfig(); calcBodyComp(); pubState();
    }
  }
}

static void connectMqtt() {
  if (!cfg.mqttHost[0] || WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected() || millis() - lastMqttMs < MQTT_RECONNECT_MS) return;
  lastMqttMs = millis();
  mqtt.setServer(cfg.mqttHost, cfg.mqttPort ? cfg.mqttPort : 1883);
  mqtt.setCallback(mqttCb);
  mqtt.setBufferSize(700);
  String cid = String(S7_DEVICE_ID) + "-" + String(ESP.getChipId(), HEX);
  bool ok = cfg.mqttUser[0]
    ? mqtt.connect(cid.c_str(), cfg.mqttUser, cfg.mqttPass,
                   (topicBase()+"/availability").c_str(), 0, true, "offline")
    : mqtt.connect(cid.c_str(),
                   (topicBase()+"/availability").c_str(), 0, true, "offline");
  if (ok) {
    pubAvail(true);
    mqtt.subscribe((topicBase()+"/cmd/#").c_str());
    pubDiscovery();
    pubState();
  }
}

static void startAP() {
  if (apOn) return;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP((String("S7-") + String(ESP.getChipId(), HEX)).c_str());
  apOn = true;
}
static void stopAP() {
  if (!apOn) return;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  apOn = false;
}
static void setupOTA() {
  if (otaOn) return;
  ArduinoOTA.setHostname(S7_DEVICE_ID);
  if (cfg.otaPass[0]) ArduinoOTA.setPassword(cfg.otaPass);
  ArduinoOTA.onStart([]{ heaterSet(false); });
  ArduinoOTA.begin();
  otaOn = true;
}

// ============================================================
//  Web UI
// ============================================================
static void handleRoot() {
  String p; p.reserve(9500);
  p+=F("<!DOCTYPE html><html><head><meta charset=utf-8>"
       "<meta name=viewport content='width=device-width,initial-scale=1'>"
       "<title>\xe6\x96\x90\xe8\xae\xaf S7 \xe4\xbd\x93\xe8\x84\x82\xe7\xa7\xa4</title><style>"
       "*{box-sizing:border-box;margin:0;padding:0}"
       "body{font-family:-apple-system,system-ui,'PingFang SC','Microsoft YaHei',sans-serif;"
       "background:#f5f5f5;color:#333;line-height:1.6}"
       ".wrap{max-width:800px;margin:0 auto;padding:16px}"
       "header{background:#fff;padding:16px 20px;border-bottom:1px solid #e0e0e0;margin-bottom:16px;"
       "border-radius:0 0 8px 8px;box-shadow:0 1px 3px rgba(0,0,0,.08)}"
       "header h1{font-size:20px;font-weight:600;color:#1a1a1a}"
       "header .sub{font-size:12px;color:#999;margin-top:2px}"
       ".status{display:flex;gap:12px;flex-wrap:wrap;margin-top:8px;font-size:13px}"
       ".status span{display:inline-flex;align-items:center;gap:4px}"
       ".dot{width:8px;height:8px;border-radius:50%;display:inline-block}"
       ".dot.on{background:#52c41a}.dot.off{background:#ff4d4f}"
       ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:10px;margin:16px 0}"
       ".card{background:#fff;border-radius:8px;padding:14px 16px;"
       "box-shadow:0 1px 3px rgba(0,0,0,.06);border:1px solid #f0f0f0}"
       ".card .label{font-size:12px;color:#999;margin-bottom:4px}"
       ".card .val{font-size:24px;font-weight:700;color:#1a1a1a}"
       ".card .unit{font-size:13px;font-weight:400;color:#999;margin-left:2px}"
       ".actions{display:flex;gap:8px;flex-wrap:wrap;margin:16px 0}"
       "button,input[type=submit]{font:inherit;border-radius:6px;border:1px solid #d9d9d9;background:#fff;"
       "color:#333;padding:8px 20px;cursor:pointer;font-size:14px;transition:all .2s}"
       "button:hover,input[type=submit]:hover{border-color:#1677ff;color:#1677ff}"
       ".btn-primary{background:#1677ff;color:#fff;border-color:#1677ff}"
       ".btn-primary:hover{background:#4096ff}"
       ".btn-danger{border-color:#ff4d4f;color:#ff4d4f}"
       ".btn-danger:hover{background:#ff4d4f;color:#fff}"
       ".section{background:#fff;border-radius:8px;padding:20px;"
       "box-shadow:0 1px 3px rgba(0,0,0,.06);border:1px solid #f0f0f0;margin-bottom:16px}"
       ".section h2{font-size:16px;font-weight:600;color:#1a1a1a;padding-bottom:10px;"
       "border-bottom:1px solid #f0f0f0;margin-bottom:14px}"
       "label{display:block;margin-bottom:10px;font-size:14px;color:#666}"
       "label span{display:block;margin-bottom:4px}"
       "input,select{font:inherit;width:100%;border-radius:6px;border:1px solid #d9d9d9;"
       "background:#fff;color:#333;padding:8px 12px;font-size:14px;transition:border .2s}"
       "input:focus,select:focus{outline:none;border-color:#1677ff;box-shadow:0 0 0 2px rgba(22,119,255,.1)}"
       ".row2{display:grid;grid-template-columns:1fr 1fr;gap:12px}"
       "@media(max-width:600px){.row2{grid-template-columns:1fr}}"
       "details{margin-top:16px}summary{cursor:pointer;font-size:14px;color:#1677ff}"
       "pre{background:#fafafa;padding:10px;border-radius:6px;overflow:auto;"
       "font-size:11px;line-height:1.5;border:1px solid #f0f0f0;margin-top:8px;color:#666}"
       "a{color:#1677ff;text-decoration:none}a:hover{text-decoration:underline}"
       ".note{background:#fffbe6;border:1px solid #ffe58f;border-radius:6px;padding:10px 14px;"
       "font-size:13px;color:#ad6800;margin:12px 0}"
       "</style>"
       "<script>"
       "async function act(u){await fetch(u,{method:'POST'});location.reload()}"
       "async function rf(){try{let r=await fetch('/api/state');let j=await r.json();"
       "for(const k in j){let e=document.querySelector('[data-k=\"'+k+'\"]');if(e)e.textContent=j[k]}}catch(e){}}"
       "setInterval(rf,2000);window.onload=rf;"
       "</script></head><body>");

  // Header
  p+=F("<header><div class=wrap>"
       "<h1>\xe6\x96\x90\xe8\xae\xaf S7 \xe4\xbd\x93\xe8\x84\x82\xe7\xa7\xa4</h1>"
       "<div class=sub>\xe5\x9b\xba\xe4\xbb\xb6 v" FW_VERSION " \xc2\xb7 ");
  if (WiFi.status()==WL_CONNECTED) p+=WiFi.localIP().toString();
  else p+=F("\xe6\x9c\xaa\xe8\xbf\x9e\xe6\x8e\xa5");
  p+=F("</div><div class=status>");
  p+=F("<span><i class='dot "); p+=WiFi.status()==WL_CONNECTED?F("on"):F("off");
  p+=F("'></i>WiFi</span>");
  p+=F("<span><i class='dot "); p+=mqtt.connected()?F("on"):F("off");
  p+=F("'></i>MQTT</span>");
  p+=F("<span><i class='dot "); p+=m.mcuAlive?F("on"):F("off");
  p+=F("'></i>MCU</span>");
  p+=F("<span><i class='dot "); p+=m.csOk?F("on"):F("off");
  p+=F("'></i>CS1258</span>");
  p+=F("</div></div></header><div class=wrap>");

  // Measurement cards
  p+=F("<div class=grid>");
  struct Card{const char*k,*l,*u;};
  Card cards[]={
    {"weight_kg",    "\xe4\xbd\x93\xe9\x87\x8d","kg"},
    {"impedance_ohm","\xe9\x98\xbb\xe6\x8a\x97","\xce\xa9"},
    {"body_fat_percent","\xe4\xbd\x93\xe8\x84\x82\xe7\x8e\x87","%"},
    {"water_percent", "\xe6\xb0\xb4\xe5\x88\x86\xe7\x8e\x87","%"},
    {"bmi",           "BMI",""},
    {"fat_free_mass_kg","\xe5\x8e\xbb\xe8\x84\x82\xe4\xbd\x93\xe9\x87\x8d","kg"},
    {"bmr_kcal",      "\xe5\x9f\xba\xe7\xa1\x80\xe4\xbb\xa3\xe8\xb0\xa2","kcal"},
    {"skeletal_muscle_percent","\xe8\x82\x8c\xe8\x82\x89\xe7\x8e\x87","%"},
    {"bone_mass_kg",  "\xe9\xaa\xa8\xe9\x87\x8f","kg"}
  };
  for(auto&c:cards){
    p+=F("<div class=card><div class=label>"); p+=c.l;
    p+=F("</div><div class=val><span data-k='"); p+=c.k;
    p+=F("'>-</span>");
    if(c.u[0]){ p+=F("<span class=unit>"); p+=c.u; p+=F("</span>"); }
    p+=F("</div></div>");
  }
  p+=F("</div>");

  // Action buttons
  p+=F("<div class=actions>"
       "<button class=btn-primary onclick=\"act('/impedance')\">\xe6\xb5\x8b\xe9\x87\x8f\xe9\x98\xbb\xe6\x8a\x97</button>"
       "<button onclick=\"act('/heater?on=1')\">\xe5\xbc\x80\xe5\x90\xaf\xe5\x8a\xa0\xe7\x83\xad</button>"
       "<button class=btn-danger onclick=\"act('/heater?on=0')\">\xe5\x85\xb3\xe9\x97\xad\xe5\x8a\xa0\xe7\x83\xad</button>"
       "</div>");

  // Debounce status hint
  if (personOnScale && !measurementDone) {
    p+=F("<div class=note>\xe8\xaf\xb7\xe4\xbf\x9d\xe6\x8c\x81\xe9\x9d\x99\xe6\xad\xa2\xef\xbc\x8c\xe6\xad\xa3\xe5\x9c\xa8\xe7\xad\x89\xe5\xbe\x85\xe4\xbd\x93\xe9\x87\x8d\xe7\xa8\xb3\xe5\xae\x9a\xe2\x80\xa6</div>"); // 请保持静止，正在等待体重稳定…
  }

  // Settings form
  p+=F("<form method=post action=/save>");

  // Profile
  p+=F("<div class=section><h2>\xe7\x94\xa8\xe6\x88\xb7\xe8\xb5\x84\xe6\x96\x99</h2><div class=row2>");
  p+=F("<label><span>\xe6\x80\xa7\xe5\x88\xab</span><select name=sex>"
       "<option value=1"); if(cfg.male) p+=F(" selected"); p+=F(">\xe7\x94\xb7</option>"
       "<option value=0"); if(!cfg.male) p+=F(" selected"); p+=F(">\xe5\xa5\xb3</option></select></label>");
  p+=F("<label><span>\xe5\xb9\xb4\xe9\xbe\x84</span><input name=age type=number min=10 max=99 value='"); p+=String(cfg.age);
  p+=F("'></label><label><span>\xe8\xba\xab\xe9\xab\x98 (m)</span><input name=ht type=number step=0.01 min=1.0 max=2.5 value='"); p+=String(cfg.heightM,2);
  p+=F("'></label></div></div>");

  // WiFi
  p+=F("<div class=section><h2>WiFi \xe8\xae\xbe\xe7\xbd\xae</h2><div class=row2>");
  p+=F("<label><span>SSID</span><input name=ssid value='"); p+=esc(cfg.ssid);
  p+=F("'></label><label><span>\xe5\xaf\x86\xe7\xa0\x81</span><input name=pass type=password value='"); p+=esc(cfg.pass);
  p+=F("'></label></div></div>");

  // MQTT
  p+=F("<div class=section><h2>MQTT \xe8\xae\xbe\xe7\xbd\xae</h2><div class=row2>");
  p+=F("<label><span>\xe6\x9c\x8d\xe5\x8a\xa1\xe5\x99\xa8</span><input name=mh value='"); p+=esc(cfg.mqttHost);
  p+=F("'></label><label><span>\xe7\xab\xaf\xe5\x8f\xa3</span><input name=mp type=number value='"); p+=String(cfg.mqttPort);
  p+=F("'></label><label><span>\xe7\x94\xa8\xe6\x88\xb7\xe5\x90\x8d</span><input name=mu value='"); p+=esc(cfg.mqttUser);
  p+=F("'></label><label><span>\xe5\xaf\x86\xe7\xa0\x81</span><input name=mw type=password value='"); p+=esc(cfg.mqttPass);
  p+=F("'></label></div></div>");

  // Heater
  p+=F("<div class=section><h2>\xe5\x8a\xa0\xe7\x83\xad\xe5\x99\xa8</h2><div class=row2>");
  p+=F("<label><span>GPIO \xe5\xbc\x95\xe8\x84\x9a (-1=\xe7\xa6\x81\xe7\x94\xa8)</span><input name=hp type=number min=-1 max=16 value='"); p+=String(cfg.heaterPin);
  p+=F("'></label><label><span>\xe6\x9c\x80\xe5\xa4\xa7\xe5\x8a\xa0\xe7\x83\xad\xe6\x97\xb6\xe9\x97\xb4 (\xe7\xa7\x92)</span><input name=hmax type=number min=1 max=600 value='"); p+=String(cfg.heaterMaxSec);
  p+=F("'></label></div></div>");

  // OTA
  p+=F("<div class=section><h2>\xe5\x9b\xba\xe4\xbb\xb6\xe5\x8d\x87\xe7\xba\xa7</h2>");
  p+=F("<label><span>OTA \xe5\xaf\x86\xe7\xa0\x81</span><input name=op type=password value='"); p+=esc(cfg.otaPass);
  p+=F("'></label><p style='margin-top:8px'><a href=/update>\xe7\xbd\x91\xe9\xa1\xb5\xe5\x8d\x87\xe7\xba\xa7 (Web Update)</a></p></div>");

  // Save
  p+=F("<div style='margin:20px 0'><input type=submit class=btn-primary value='"
       "\xe4\xbf\x9d\xe5\xad\x98\xe8\xae\xbe\xe7\xbd\xae'></div></form>");

  // Advanced (collapsible)
  p+=F("<details><summary>\xe9\xab\x98\xe7\xba\xa7\xe8\xae\xbe\xe7\xbd\xae / \xe8\xb0\x83\xe8\xaf\x95\xe4\xbf\xa1\xe6\x81\xaf</summary>");

  p+=F("<div class=section style='margin-top:12px'><h2>\xe4\xb8\xb2\xe5\x8f\xa3\xe6\xa0\xa1\xe5\x87\x86</h2>"
       "<form method=post action=/save><div class=row2>");
  p+=F("<label><span>\xe6\xb3\xa2\xe7\x89\xb9\xe7\x8e\x87</span><select name=baud>");
  const uint32_t bauds[]={9600,19200,38400,57600,74880,115200};
  for(auto b:bauds){p+=F("<option value=");p+=String(b);if(b==cfg.mcuBaud)p+=F(" selected");p+=">";p+=String(b);p+=F("</option>");}
  p+=F("</select></label>");
  p+=F("<label><span>\xe4\xbd\x93\xe9\x87\x8d\xe9\xab\x98\xe5\xad\x97\xe8\x8a\x82</span><input name=whi type=number value='"); p+=String(cfg.weightHi);
  p+=F("'></label><label><span>\xe4\xbd\x93\xe9\x87\x8d\xe4\xbd\x8e\xe5\xad\x97\xe8\x8a\x82</span><input name=wlo type=number value='"); p+=String(cfg.weightLo);
  p+=F("'></label><label><span>\xe9\x99\xa4\xe6\x95\xb0</span><input name=wdiv type=number step=0.01 value='"); p+=String(cfg.weightDiv,2);
  p+=F("'></label><label><span>\xe5\x81\x8f\xe7\xa7\xbb (kg)</span><input name=woff type=number step=0.01 value='"); p+=String(cfg.weightOffset,2);
  p+=F("'></label><label><span>\xe9\x98\xbb\xe6\x8a\x97\xe7\xb3\xbb\xe6\x95\xb0</span><input name=zs type=number step=0.001 value='"); p+=String(cfg.zScale,3);
  p+=F("'></label><label><span>\xe9\x98\xbb\xe6\x8a\x97\xe5\x81\x8f\xe7\xa7\xbb</span><input name=zo type=number step=0.1 value='"); p+=String(cfg.zOffset,1);
  p+=F("'></label></div><input type=submit value='\xe4\xbf\x9d\xe5\xad\x98'></form></div>");

  // CS1258 debug
  p+=F("<div class=section><h2>CS1258 \xe8\xb0\x83\xe8\xaf\x95\xe6\x97\xa5\xe5\xbf\x97</h2><pre>");
  p+=m.csLog;
  p+=F("\nshort_adc: "); p+=String(m.shortAdc);
  p+=F("  cal_a: "); p+=String(m.calA);
  p+=F("  cal_b: "); p+=String(m.calB);
  struct ChD{const char*n;ChResult*r;};
  ChD chs[]={{"EB lrLeg",&m.lrLeg},{"BE rlLeg",&m.rlLeg},{"14 lrHand",&m.lrHand},
    {"DB rLeg",&m.rLeg},{"8E lLeg",&m.lLeg},{"3C lhLl",&m.lhLl},
    {"28 lhRl",&m.lhRl},{"7D rhLl",&m.rhLl},{"69 rhRl",&m.rhRl}};
  for(auto&c:chs){char ln[60];snprintf(ln,sizeof(ln),"\n  %s: raw=%lu ohm=%u",c.n,(unsigned long)c.r->raw,c.r->ohm);p+=ln;}
  p+=F("\nbodyRes:"); p+=String(m.bodyRes);
  p+=F(" zEff:"); p+=String(m.zEffOhm,1);
  p+=F(" valid:"); p+=m.bodyValid?F("YES"):F("NO");
  p+=F("</pre></div>");

  // MCU debug
  p+=F("<div class=section><h2>MCU \xe8\xb0\x83\xe8\xaf\x95</h2><pre>");
  p+=F("\xe6\xb3\xa2\xe7\x89\xb9\xe7\x8e\x87: "); p+=String(cfg.mcuBaud);
  p+=F("\n\xe4\xb8\x8a\xe6\xac\xa1 0x10 ("); p+=String(m.lastCmd10Len); p+=F("B): ");
  p+=m.lastCmd10Len?hexDump(m.lastCmd10,m.lastCmd10Len):F("(\xe6\x97\xa0)");
  if(m.lastCmd10Len>3){uint8_t pl=m.lastCmd10[1]>1?m.lastCmd10[1]-1:0;p+=F("\npayload:");
    for(uint8_t i=0;i<pl&&(3+i)<m.lastCmd10Len;i++){char t[16];snprintf(t,sizeof(t)," [%u]=%02X",i,m.lastCmd10[3+i]);p+=t;}}
  p+=F("</pre></div>");

  p+=F("</details>");

  // Footer
  p+=F("<div style='text-align:center;color:#ccc;font-size:12px;padding:20px 0'>"
       "\xe6\x96\x90\xe8\xae\xaf S7 \xe4\xbd\x93\xe8\x84\x82\xe7\xa7\xa4 &middot; "
       "<a href=/api/state>JSON API</a></div>");

  p+=F("</div></body></html>");
  web.send(200, "text/html; charset=utf-8", p);
}

static void applySave() {
  if(web.hasArg("ssid"))cpStr(cfg.ssid,sizeof(cfg.ssid),web.arg("ssid").c_str());
  if(web.hasArg("pass"))cpStr(cfg.pass,sizeof(cfg.pass),web.arg("pass").c_str());
  if(web.hasArg("mh"))cpStr(cfg.mqttHost,sizeof(cfg.mqttHost),web.arg("mh").c_str());
  if(web.hasArg("mp"))cfg.mqttPort=(uint16_t)web.arg("mp").toInt();
  if(web.hasArg("mu"))cpStr(cfg.mqttUser,sizeof(cfg.mqttUser),web.arg("mu").c_str());
  if(web.hasArg("mw"))cpStr(cfg.mqttPass,sizeof(cfg.mqttPass),web.arg("mw").c_str());
  if(web.hasArg("op"))cpStr(cfg.otaPass,sizeof(cfg.otaPass),web.arg("op").c_str());
  if(web.hasArg("sex"))cfg.male=web.arg("sex").toInt()?1:0;
  if(web.hasArg("age"))cfg.age=(uint8_t)web.arg("age").toInt();
  if(web.hasArg("ht"))cfg.heightM=web.arg("ht").toFloat();
  if(web.hasArg("baud")){uint32_t b=(uint32_t)web.arg("baud").toInt();if(b>=9600&&b<=115200)cfg.mcuBaud=b;}
  if(web.hasArg("whi"))cfg.weightHi=(uint8_t)web.arg("whi").toInt();
  if(web.hasArg("wlo"))cfg.weightLo=(uint8_t)web.arg("wlo").toInt();
  if(web.hasArg("wdiv")){float d=web.arg("wdiv").toFloat();if(d>0)cfg.weightDiv=d;}
  if(web.hasArg("woff"))cfg.weightOffset=web.arg("woff").toFloat();
  if(web.hasArg("zs"))cfg.zScale=web.arg("zs").toFloat();
  if(web.hasArg("zo"))cfg.zOffset=web.arg("zo").toFloat();
  if(web.hasArg("hp"))cfg.heaterPin=(int8_t)web.arg("hp").toInt();
  if(web.hasArg("hmax")){int v=web.arg("hmax").toInt();if(v>0&&v<=600)cfg.heaterMaxSec=(uint16_t)v;}
  if(cfg.mqttPort==0)cfg.mqttPort=1883;
  if(cfg.age<10||cfg.age>99)cfg.age=25;
  if(cfg.heightM<0.5f||cfg.heightM>2.5f)cfg.heightM=1.75f;
  if(cfg.weightDiv<=0)cfg.weightDiv=100;
  if(cfg.heaterMaxSec==0||cfg.heaterMaxSec>600)cfg.heaterMaxSec=60;
  saveConfig();
}

static void redir(){web.sendHeader("Location","/",true);web.send(303,"text/plain","");}

static void sendSaveOk() {
  String p;
  p += F("<!DOCTYPE html><html><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>\xe4\xbf\x9d\xe5\xad\x98\xe6\x88\x90\xe5\x8a\x9f</title>"
    "<style>body{background:#1a1a2e;color:#e0e0e0;font-family:system-ui;display:flex;"
    "justify-content:center;align-items:center;min-height:100vh;margin:0}"
    ".box{text-align:center;background:#16213e;padding:2em 3em;border-radius:12px;"
    "box-shadow:0 4px 20px rgba(0,0,0,.4)}"
    "h2{color:#4fc3f7;margin-top:0}p{margin:.8em 0;font-size:1.1em}"
    ".hint{color:#999;font-size:.9em}a{color:#4fc3f7}</style></head><body>"
    "<div class=box><h2>\xe8\xae\xbe\xe7\xbd\xae\xe5\xb7\xb2\xe4\xbf\x9d\xe5\xad\x98</h2>"
    "<p>\xe6\xad\xa3\xe5\x9c\xa8\xe8\xbf\x9e\xe6\x8e\xa5 WiFi\xe2\x80\xa6</p>");
  // 设置已保存 / 正在连接 WiFi...
  if (cfg.ssid[0]) {
    p += F("<p>SSID: <b>");
    p += esc(String(cfg.ssid));
    p += F("</b></p>"
      "<p class=hint>\xe8\xae\xbe\xe5\xa4\x87\xe5\xb0\x86\xe8\x87\xaa\xe5\x8a\xa8\xe5\x85\xb3\xe9\x97\xad\xe7\x83\xad\xe7\x82\xb9\xe3\x80\x82</p>"
      "<p class=hint>\xe8\xaf\xb7\xe8\xbf\x9e\xe6\x8e\xa5\xe5\x88\xb0\xe5\x90\x8c\xe4\xb8\x80 WiFi \xe5\x90\x8e\xe8\xae\xbf\xe9\x97\xae\xe7\xa7\xa4\xe7\x9a\x84\xe6\x96\xb0 IP\xe3\x80\x82</p>");
    // 设备将自动关闭热点。请连接到同一 WiFi 后访问秤的新 IP。
  } else {
    p += F("<p class=hint>\xe6\x9c\xaa\xe8\xae\xbe\xe7\xbd\xae WiFi\xef\xbc\x8c\xe7\x83\xad\xe7\x82\xb9\xe4\xbb\x8d\xe7\x84\xb6\xe5\xbc\x80\xe5\x90\xaf\xe3\x80\x82</p>");
    // 未设置 WiFi，热点仍然开启。
  }
  p += F("<p style='margin-top:1.5em'><a href='/'>\xe8\xbf\x94\xe5\x9b\x9e\xe9\xa6\x96\xe9\xa1\xb5</a></p>"
    "</div></body></html>");
  // 返回首页
  web.send(200, "text/html", p);
}

static void setupWeb() {
  if(cfg.otaPass[0])httpUpdater.setup(&web,"/update","admin",cfg.otaPass);
  else httpUpdater.setup(&web,"/update");
  web.on("/",HTTP_GET,handleRoot);
  web.on("/api/state",HTTP_GET,[]{web.send(200,"application/json",stateJson());});
  web.on("/save",HTTP_POST,[]{uint32_t ob=cfg.mcuBaud;applySave();if(cfg.mcuBaud!=ob){Serial.end();Serial.begin(cfg.mcuBaud);}mqtt.disconnect();WiFi.disconnect();WiFi.begin(cfg.ssid,cfg.pass);sendSaveOk();});
  web.on("/impedance",HTTP_POST,[]{csDoImpedanceScan();calcBodyComp();pubState();redir();});
  web.on("/heater",HTTP_POST,[]{heaterSet(web.arg("on")=="1");pubState();redir();});
  web.begin();
}

void setup() {
  loadConfig(); Serial.begin(cfg.mcuBaud); delay(50);
  heaterSet(false);
  pinMode(PIN_CS,OUTPUT);pinMode(PIN_CLK,OUTPUT);pinMode(PIN_DIO,INPUT);
  digitalWrite(PIN_CS,HIGH);digitalWrite(PIN_CLK,HIGH);
  WiFi.mode(WIFI_STA);WiFi.hostname(S7_DEVICE_ID);
  if(cfg.ssid[0])WiFi.begin(cfg.ssid,cfg.pass);
  startAP();setupWeb();setupOTA();
  memset(&rx,0,sizeof(rx));memset(&m,0,sizeof(m));
}

void loop() {
  // Core services
  if(cfg.ssid[0]&&WiFi.status()!=WL_CONNECTED&&millis()-lastWifiMs>WIFI_RETRY_MS){
    lastWifiMs=millis();WiFi.disconnect();WiFi.begin(cfg.ssid,cfg.pass);
  }
  connectMqtt(); mqtt.loop(); web.handleClient(); ArduinoOTA.handle(); uartPoll();

  // Auto-close AP once WiFi STA is connected
  static uint32_t staConnectedSince = 0;
  if (WiFi.status() == WL_CONNECTED) {
    if (staConnectedSince == 0) staConnectedSince = millis();
    if (apOn && millis() - staConnectedSince > 10000) stopAP();  // 10s grace period
  } else {
    staConnectedSince = 0;
    if (!apOn && cfg.ssid[0] && millis() > 60000) startAP();  // re-open AP if WiFi lost after 60s
  }

  // Heater auto-off
  if(heaterOn&&heaterOffMs&&(int32_t)(millis()-heaterOffMs)>=0){
    heaterSet(false); pubState();
  }

  // ---- Weight debounce & auto-measurement ----
  bool onScale = m.weightKg >= WEIGHT_ON_THRESH;

  if (onScale) {
    if (!personOnScale) {
      // Just stepped on — start fresh measurement session
      personOnScale    = true;
      measurementDone  = false;
      m.bodyValid      = false;
      stableWeight     = m.weightKg;
      weightStableSince= millis();
    } else {
      // Still on scale — track stability
      float diff = m.weightKg - stableWeight;
      if (diff < 0) diff = -diff;
      if (diff > WEIGHT_STABLE_DELTA) {
        stableWeight     = m.weightKg;
        weightStableSince= millis();
      }
    }

    bool stable = (millis() - weightStableSince >= WEIGHT_STABLE_MS);

    // Auto impedance scan when weight is stable
    if (stable && !measurementDone && millis() - m.lastImpedMs > IMPEDANCE_COOL_MS) {
      csDoImpedanceScan();
      calcBodyComp();
      if (m.bodyValid) measurementDone = true;
      pubState();
    }

    // Periodic MQTT publish while standing on scale (only when stable)
    static uint32_t lp = 0;
    if (stable && m.mcuAlive && millis() - lp > 3000) { lp = millis(); pubState(); }

  } else {
    if (personOnScale) {
      // Just stepped off — do NOT publish (keep last retained state)
      personOnScale = false;
    }
    // Off-scale: no MQTT publish, HA retains last measurement
  }
}
