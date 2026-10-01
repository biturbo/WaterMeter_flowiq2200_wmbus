// FlowIQ 2200 / Multical 21 wM-Bus decoding (no ESPHome dependencies).
//
// Radio/crypto parts ported from watermeter-flowiq2200 by erikxson (GPLv3), itself derived from
// esp32-multical21 (pthalin) and esp-multical21 (chester4444). Field definitions follow the
// wmbusmeters "kamwater" driver by Fredrik Öhrström (GPLv3). GPLv3 or later.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

namespace esphome {
namespace flowiq2200 {

// --- AES-128 (encrypt direction only; CTR mode needs nothing else) ---------
class Aes128 {
 public:
  void set_key(const uint8_t key[16]);
  void encrypt_block(const uint8_t in[16], uint8_t out[16]) const;

 private:
  uint8_t round_keys_[176];
};

// AES-128-CTR. The counter is the last byte(s) of the IV, big-endian.
void aes128_ctr(const Aes128 &aes, const uint8_t iv[16], const uint8_t *in, uint8_t *out, size_t len);

// EN 13757 CRC-16 (poly 0x3D65, init 0, inverted output).
uint16_t crc16_en13757(const uint8_t *data, size_t len);

// --- Decoded values ---------------------------------------------------------
enum NumericField : uint8_t {
  NUM_TOTAL_M3 = 0,
  NUM_TARGET_M3,
  NUM_FLOW_LPH,
  NUM_MAX_FLOW_LAST_DAY_LPH,
  NUM_MIN_FLOW_LAST_DAY_LPH,
  NUM_MIN_FLOW_TEMP_LAST_DAY,
  NUM_MAX_FLOW_TEMP_LAST_DAY,
  NUM_MIN_EXT_TEMP_LAST_DAY,
  NUM_MAX_EXT_TEMP_LAST_DAY,
  NUM_FIELD_COUNT,
};

enum StatusKind : uint8_t {
  STATUS_NONE = 0,
  STATUS_16,        // 02FF20: older Multical 21
  STATUS_32_COLD,   // 04FF23: "info, cold environment"
  STATUS_32_HOT,    // 04FF24: "info, warm environment"
};

// Status flag bit numbers (wmbusmeters kamwater driver).
enum StatusFlag : uint8_t {
  FLAG_DRY = 0,
  FLAG_REVERSE = 1,
  FLAG_LEAK = 2,
  FLAG_BURST = 3,
  FLAG_TAMPER = 4,         // 32-bit status only
  FLAG_LOW_BATTERY = 5,    // 32-bit status only
  FLAG_AMBIENT_TEMP = 6,   // low (cold env) / high (warm env) ambient temperature, 32-bit only
  FLAG_FLOW_ABOVE_Q4 = 7,  // 32-bit status only
  FLAG_NO_CONSUMPTION = 9, // 32-bit status only
};

enum StatusDuration : uint8_t {
  DUR_DRY = 0,
  DUR_REVERSED,
  DUR_LEAKING,
  DUR_BURSTING,
  DUR_AMBIENT_TEMP,  // 32-bit status only
  DUR_FLOW_ABOVE_Q4, // 32-bit status only
};

struct Reading {
  float num[NUM_FIELD_COUNT];  // NAN when the telegram didn't contain the field

  bool has_target_date{false};
  uint16_t target_year{0};
  uint8_t target_month{0};
  uint8_t target_day{0};

  StatusKind status_kind{STATUS_NONE};
  uint32_t status{0};

  bool has_noise{false};
  uint8_t noise[6]{};

  Reading() {
    for (float &v : num)
      v = NAN;
  }

  // false if the flag isn't present in this meter's status format
  bool flag_available(StatusFlag flag) const;
  bool flag(StatusFlag flag) const { return (this->status >> flag) & 1u; }
  bool duration_available(StatusDuration d) const;
  uint8_t duration_code(StatusDuration d) const;  // 0..7
  std::string status_text() const;                // "OK" or e.g. "LEAK LOW_BATTERY"
};

// "none", "1-8 hours", ..., "22-31 days"
const char *duration_text(uint8_t code);

// --- Frame decoder ------------------------------------------------------------
enum class DecodeResult : uint8_t {
  OK,
  TOO_SHORT,
  CRC_ERROR,        // payload CRC wrong: wrong key or corrupted telegram
  UNKNOWN_FORMAT,   // compact frame whose layout hasn't been seen yet
  FORMAT_MISMATCH,  // compact frame data didn't fit / verify against the layout
  UNSUPPORTED_CI,
};

class FrameDecoder {
 public:
  FrameDecoder();

  // `plain` is the decrypted ELL payload: PayloadCRC(2) CI ...
  DecodeResult decode(const uint8_t *plain, size_t len, Reading *out);

  uint8_t last_ci() const { return this->last_ci_; }
  uint16_t last_signature() const { return this->last_signature_; }
  bool last_learned() const { return this->last_learned_; }
  bool last_data_crc_ok() const { return this->last_data_crc_ok_; }
  size_t layout_count() const { return this->count_; }

 private:
  static const size_t MAX_LAYOUTS = 16;
  static const size_t MAX_DIFVIF = 48;
  struct Layout {
    uint16_t signature;
    uint8_t len;
    uint8_t difvif[MAX_DIFVIF];
  };

  bool add_layout_(const uint8_t *difvif, size_t len);
  const Layout *find_layout_(uint16_t signature) const;

  Layout layouts_[MAX_LAYOUTS];
  size_t count_{0};
  size_t builtin_count_{0};
  size_t next_learned_{0};

  uint8_t last_ci_{0};
  uint16_t last_signature_{0};
  bool last_learned_{false};
  bool last_data_crc_ok_{true};
};

}  // namespace flowiq2200
}  // namespace esphome
