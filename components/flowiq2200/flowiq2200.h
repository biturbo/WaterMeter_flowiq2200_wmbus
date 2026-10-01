// ESPHome component: Kamstrup FlowIQ 2200 water meter via CC1101 (wM-Bus mode C).
//
// Ported from watermeter-flowiq2200 by erikxson (GPLv3), itself derived from
// esp32-multical21 (pthalin) and esp-multical21 (chester4444). GPLv3 or later.
#pragma once

#include <string>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

#include "flowiq_decoder.h"

namespace esphome {
namespace flowiq2200 {

enum TextField : uint8_t {
  TXT_STATUS = 0,
  TXT_TARGET_DATE,
  TXT_TIME_DRY,
  TXT_TIME_REVERSED,
  TXT_TIME_LEAKING,
  TXT_TIME_BURSTING,
  TXT_TIME_AMBIENT_TEMP,
  TXT_TIME_FLOW_ABOVE_Q4,
  TXT_ACOUSTIC_NOISE,
  TXT_FIELD_COUNT,
};

enum BinaryField : uint8_t {
  BIN_DRY = 0,
  BIN_REVERSE,
  BIN_LEAK,
  BIN_BURST,
  BIN_TAMPER,
  BIN_LOW_BATTERY,
  BIN_AMBIENT_TEMP,
  BIN_FLOW_ABOVE_Q4,
  BIN_NO_CONSUMPTION,
  BIN_FIELD_COUNT,
};

class FlowIQ2200Component : public Component,
                            public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                                  spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_4MHZ> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_gdo0_pin(InternalGPIOPin *pin) { this->gdo0_pin_ = pin; }
  void set_meter_id(uint32_t meter_id) { this->meter_id_ = meter_id; }
  void set_key(const std::string &hex_key);

  void set_numeric_sensor(uint8_t field, sensor::Sensor *s) { this->numeric_[field] = s; }
  void set_text_sensor(uint8_t field, text_sensor::TextSensor *s) { this->text_[field] = s; }
  void set_binary_sensor(uint8_t field, binary_sensor::BinarySensor *s) { this->binary_[field] = s; }
  void set_rssi_sensor(sensor::Sensor *s) { this->rssi_sensor_ = s; }

 protected:
  // --- CC1101 access ---
  void reset_radio_();
  void write_reg_(uint8_t reg, uint8_t value);
  uint8_t read_status_(uint8_t reg);
  void strobe_(uint8_t cmd);
  void read_fifo_(uint8_t *buf, size_t len);
  uint8_t rx_bytes_();
  bool wait_marcstate_(uint8_t state, uint32_t timeout_ms);
  bool start_receiver_();

  // Reads one telegram into frame_buf_. Returns the payload length (L field) or 0.
  uint8_t receive_frame_();

  // --- Telegram handling ---
  void handle_frame_(const uint8_t *payload, uint8_t len);
  void handle_plaintext_(const uint8_t *data, size_t len);
  void publish_(const Reading &r);

  static void gpio_intr(FlowIQ2200Component *arg);

  InternalGPIOPin *gdo0_pin_{nullptr};
  volatile bool sync_flag_{false};
  HighFrequencyLoopRequester high_freq_;

  uint32_t meter_id_{0};
  Aes128 aes_;
  bool key_ok_{false};
  FrameDecoder decoder_;

  uint8_t chip_version_{0};
  uint32_t last_health_check_{0};
  int8_t last_rssi_{0};
  uint32_t frames_ok_{0};
  uint32_t frames_other_{0};
  uint32_t frames_bad_{0};
  uint16_t last_unknown_signature_{0};

  // 0x54 0x3D + L + up to 255 payload bytes
  uint8_t frame_buf_[3 + 255];

  sensor::Sensor *numeric_[NUM_FIELD_COUNT]{};
  text_sensor::TextSensor *text_[TXT_FIELD_COUNT]{};
  binary_sensor::BinarySensor *binary_[BIN_FIELD_COUNT]{};
  sensor::Sensor *rssi_sensor_{nullptr};
};

}  // namespace flowiq2200
}  // namespace esphome
