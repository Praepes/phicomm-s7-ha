#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace cs1258 {

struct ChResult {
  uint32_t raw;
  uint16_t ohm;
};

class CS1258Component : public Component {
 public:
  void setup() override {
    pinMode(pin_cs_, OUTPUT);
    pinMode(pin_clk_, OUTPUT);
    pinMode(pin_dio_, INPUT);
    digitalWrite(pin_cs_, HIGH);
    digitalWrite(pin_clk_, HIGH);
  }

  float get_setup_priority() const override { return setup_priority::DATA; }

  /// Run a full impedance scan. Blocking (~2s). Call from script/lambda.
  void scan() {
    scan_ok_ = false;
    uint32_t t0 = millis();

    if (!reset_init()) {
      scan_duration_ms_ = millis() - t0;
      return;
    }

    if (!pre_baseline()) {
      scan_duration_ms_ = millis() - t0;
      return;
    }

    // Baselines
    short_adc_ = read_baseline(0x33);
    cal_a_     = read_baseline(0x13);
    cal_b_     = read_baseline(0x23);

    // 9 channels
    lr_leg_  = read_channel(0xEB);
    rl_leg_  = read_channel(0xBE);
    lr_hand_ = read_channel(0x14);
    r_leg_   = read_channel(0xDB);
    l_leg_   = read_channel(0x8E);
    lh_ll_   = read_channel(0x3C);
    lh_rl_   = read_channel(0x28);
    rh_ll_   = read_channel(0x7D);
    rh_rl_   = read_channel(0x69);

    // Composite impedances
    body_res_ = half2(rl_leg_.ohm, lr_leg_.ohm);
    z1_ = half_s((int32_t)lr_hand_.ohm + lh_rl_.ohm - rh_rl_.ohm);
    z2_ = half_s((int32_t)rh_rl_.ohm + lr_hand_.ohm - lh_rl_.ohm);
    z3_ = half_s((int32_t)rh_ll_.ohm - rh_rl_.ohm + body_res_);
    z4_ = half_s((int32_t)rh_rl_.ohm - rh_ll_.ohm + body_res_);
    z5_ = (uint16_t)(((uint32_t)rh_ll_.ohm + lh_rl_.ohm) / 50);

    body_valid_ = body_res_ >= 50 && body_res_ <= 3000;
    z_eff_ohm_ = body_valid_ ? body_res_ * z_scale_ + z_offset_ : 0;

    scan_ok_ = true;
    scan_duration_ms_ = millis() - t0;

    // Publish
    if (impedance_sensor_ != nullptr && body_valid_)
      impedance_sensor_->publish_state(z_eff_ohm_);
    if (body_res_sensor_ != nullptr)
      body_res_sensor_->publish_state(body_res_);
  }

  // --- Accessors ---
  bool is_ok() const { return scan_ok_; }
  bool is_body_valid() const { return body_valid_; }
  float get_z_eff_ohm() const { return z_eff_ohm_; }
  uint16_t get_body_res() const { return body_res_; }
  uint16_t get_z1() const { return z1_; }
  uint16_t get_z2() const { return z2_; }
  uint16_t get_z3() const { return z3_; }
  uint16_t get_z4() const { return z4_; }
  uint16_t get_z5() const { return z5_; }

  // --- Config setters ---
  void set_pin_cs(uint8_t p) { pin_cs_ = p; }
  void set_pin_clk(uint8_t p) { pin_clk_ = p; }
  void set_pin_dio(uint8_t p) { pin_dio_ = p; }
  void set_z_scale(float v) { z_scale_ = v; }
  void set_z_offset(float v) { z_offset_ = v; }
  void set_cal_offset(uint16_t v) { cal_offset_ = v; }
  void set_impedance_sensor(sensor::Sensor *s) { impedance_sensor_ = s; }
  void set_body_res_sensor(sensor::Sensor *s) { body_res_sensor_ = s; }

 protected:
  // --- Low-level bit-bang ---
  void cs_delay(uint16_t us) { delayMicroseconds(us); }

  void cs_begin() {
    digitalWrite(pin_clk_, LOW); cs_delay(800);
    digitalWrite(pin_cs_, LOW);  cs_delay(2);
  }

  void cs_end() {
    digitalWrite(pin_cs_, HIGH); cs_delay(2);
    digitalWrite(pin_clk_, HIGH);
    pinMode(pin_dio_, INPUT); delay(2);
  }

  void cs_write_byte(uint8_t v) {
    pinMode(pin_dio_, OUTPUT);
    for (uint8_t i = 0; i < 8; i++) {
      digitalWrite(pin_dio_, (v & 0x80) ? HIGH : LOW);
      cs_delay(1); digitalWrite(pin_clk_, HIGH);
      cs_delay(1); digitalWrite(pin_clk_, LOW);
      v <<= 1; cs_delay(1);
    }
    cs_delay(2);
  }

  uint8_t cs_read_byte() {
    uint8_t v = 0;
    pinMode(pin_dio_, INPUT);
    for (uint8_t i = 0; i < 8; i++) {
      digitalWrite(pin_clk_, HIGH); v <<= 1; cs_delay(1);
      digitalWrite(pin_clk_, LOW);
      if (digitalRead(pin_dio_)) v |= 1;
      cs_delay(1);
    }
    return v;
  }

  void cs_write_reg(uint8_t reg, uint8_t val) {
    cs_begin(); cs_write_byte(0x80 | reg); cs_delay(2);
    cs_write_byte(val); cs_end();
  }

  uint8_t cs_read_reg(uint8_t reg) {
    cs_begin(); cs_write_byte(reg);
    uint8_t v = cs_read_byte(); cs_end();
    return v;
  }

  bool cs_write_verify(uint8_t reg, uint8_t val, uint8_t retries = 2) {
    for (uint8_t i = 0; i < retries; i++) {
      cs_write_reg(reg, val); delay(1);
      if (cs_read_reg(reg) == val) return true;
      delay(1);
    }
    return false;
  }

  uint32_t cs_read_adc24() {
    cs_begin(); cs_write_byte(0x09); cs_delay(2);
    uint8_t b0 = cs_read_byte(), b1 = cs_read_byte(), b2 = cs_read_byte();
    cs_end();
    return ((uint32_t)(b0 ^ 0x80) << 16) | ((uint32_t)b1 << 8) | b2;
  }

  void cs_read_adc24_raw(uint8_t *b0, uint8_t *b1, uint8_t *b2) {
    cs_begin(); cs_write_byte(0x09); cs_delay(2);
    *b0 = cs_read_byte(); *b1 = cs_read_byte(); *b2 = cs_read_byte();
    cs_end();
  }

  uint32_t cs_sample_avg(uint8_t n, uint16_t jump) {
    uint32_t sum = 0, prev = 0;
    uint8_t acc = 0, kept = 0;
    for (uint8_t t = 0; t < 110 && acc < n; t++) {
      delay(3);
      uint32_t r = cs_read_adc24();
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

  // --- Init / Baseline / Channel ---
  bool reset_init() {
    pinMode(pin_cs_, OUTPUT); pinMode(pin_clk_, OUTPUT); pinMode(pin_dio_, INPUT);
    digitalWrite(pin_cs_, HIGH); digitalWrite(pin_clk_, HIGH);
    delay(100);

    cs_write_reg(0xEA, 0x96);
    delay(10);
    uint8_t r0a = cs_read_reg(0x0A);
    if (!(r0a & 0x40)) return false;

    if (!cs_write_verify(0x59, 0x6A)) return false;

    uint8_t r13 = cs_read_reg(0x13);

    struct Step { uint8_t reg; uint8_t val; };
    Step steps[] = {
      {0x59, 0xA6}, {0x5A, 0x5A}, {0x12, r13}, {0x13, r13}, {0x10, 0xBF}
    };
    for (auto &s : steps) {
      if (!cs_write_verify(s.reg, s.val)) return false;
    }
    return true;
  }

  bool pre_baseline() {
    if (!cs_write_verify(0x05, 0x42)) return false;
    cs_write_reg(0x00, 0xCA);
    delay(20);
    if (!cs_write_verify(0x02, 0x4E)) return false;
    return true;
  }

  uint32_t read_baseline(uint8_t reg8val) {
    cs_write_reg(0x08, reg8val);
    delay(2);
    cs_write_reg(0x07, 0x00);
    delay(2);
    cs_write_reg(0x00, 0xCF);
    delay(5);

    // Discard first two readings
    cs_read_adc24();
    delay(5);
    cs_read_adc24();

    uint32_t v = cs_sample_avg(8, 500);
    cs_write_reg(0x00, 0xCA);
    delay(2);
    return v;
  }

  uint16_t adc_to_ohm(uint32_t raw) {
    int32_t span = (int32_t)(cal_b_ - cal_a_) - (int32_t)cal_offset_;
    if (span > 100) {
      if (raw <= short_adc_) return 0;
      float z = (float)(raw - short_adc_) * 7000.0f / (float)span;
      if (z < 0) return 0;
      if (z > 65535) return 65535;
      return (uint16_t)z;
    }
    // Fallback
    static const uint32_t ADC_ZERO = 8388608;
    if (raw <= ADC_ZERO) return 0;
    uint32_t delta = raw - ADC_ZERO;
    uint32_t ohm = delta / 512;
    return ohm > 65535 ? 65535 : (uint16_t)ohm;
  }

  ChResult read_channel(uint8_t ch) {
    ChResult r = {0, 0};
    cs_write_verify(0x08, 0x03);
    cs_write_verify(0x07, ch);
    cs_write_reg(0x00, 0xCF);
    delay(1);
    r.raw = cs_sample_avg(20, 1000);
    r.ohm = adc_to_ohm(r.raw);
    cs_write_reg(0x00, 0xCA);
    delay(2);
    return r;
  }

  static uint16_t half2(uint16_t a, uint16_t b) {
    return (uint16_t)(((uint32_t)a + b) / 2);
  }
  static uint16_t half_s(int32_t v) {
    return v <= 0 ? 0 : (v > 131070 ? 65535 : (uint16_t)(v / 2));
  }

  // --- Pins ---
  uint8_t pin_cs_{16};
  uint8_t pin_clk_{14};
  uint8_t pin_dio_{12};

  // --- Config ---
  float z_scale_{1.0f};
  float z_offset_{0.0f};
  uint16_t cal_offset_{1100};

  // --- Scan state ---
  bool scan_ok_{false};
  bool body_valid_{false};
  uint32_t scan_duration_ms_{0};
  uint32_t short_adc_{0}, cal_a_{0}, cal_b_{0};
  float z_eff_ohm_{0};
  uint16_t body_res_{0};
  uint16_t z1_{0}, z2_{0}, z3_{0}, z4_{0}, z5_{0};

  // Channel results
  ChResult lr_leg_{}, rl_leg_{}, lr_hand_{};
  ChResult r_leg_{}, l_leg_{};
  ChResult lh_ll_{}, lh_rl_{}, rh_ll_{}, rh_rl_{};

  // Sensors
  sensor::Sensor *impedance_sensor_{nullptr};
  sensor::Sensor *body_res_sensor_{nullptr};
};

}  // namespace cs1258
}  // namespace esphome
