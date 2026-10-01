// ESPHome component: Kamstrup FlowIQ 2200 water meter via CC1101. GPLv3 or later.
#include "flowiq2200.h"

#include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace flowiq2200 {

static const char *const TAG = "flowiq2200";

// CC1101 strobes
static const uint8_t CC1101_SRES = 0x30;
static const uint8_t CC1101_SCAL = 0x33;
static const uint8_t CC1101_SRX = 0x34;
static const uint8_t CC1101_SIDLE = 0x36;
static const uint8_t CC1101_SFRX = 0x3A;

// CC1101 status registers (read with burst bit set)
static const uint8_t CC1101_PARTNUM = 0x30;
static const uint8_t CC1101_VERSION = 0x31;
static const uint8_t CC1101_RSSI = 0x34;
static const uint8_t CC1101_MARCSTATE = 0x35;
static const uint8_t CC1101_RXBYTES = 0x3B;
static const uint8_t CC1101_RXFIFO_BURST = 0xFF;  // 0x3F | READ_BURST

static const uint8_t MARCSTATE_IDLE = 0x01;
static const uint8_t MARCSTATE_RX = 0x0D;

// wM-Bus mode C, frame format B, 868.95 MHz, 100 kbps.
// Register values identical to the original firmware (WaterMeter.h).
struct RegValue {
  uint8_t reg;
  uint8_t value;
};
static const RegValue CC1101_INIT[] = {
    {0x00, 0x2E},  // IOCFG2   GDO2 high impedance
    {0x02, 0x06},  // IOCFG0   GDO0 asserts on sync word
    {0x03, 0x00},  // FIFOTHR
    {0x06, 0x30},  // PKTLEN
    {0x07, 0x00},  // PKTCTRL1
    {0x08, 0x02},  // PKTCTRL0 infinite packet length
    {0x04, 0x54},  // SYNC1
    {0x05, 0x3D},  // SYNC0
    {0x09, 0x00},  // ADDR
    {0x0A, 0x00},  // CHANNR
    {0x0B, 0x08},  // FSCTRL1
    {0x0C, 0x00},  // FSCTRL0
    {0x0D, 0x21},  // FREQ2  (868.95 MHz)
    {0x0E, 0x6B},  // FREQ1
    {0x0F, 0xD0},  // FREQ0
    {0x10, 0x5C},  // MDMCFG4  ~103 kbps
    {0x11, 0x04},  // MDMCFG3
    {0x12, 0x06},  // MDMCFG2  2-FSK, 16/16 sync + carrier sense
    {0x13, 0x22},  // MDMCFG1
    {0x14, 0xF8},  // MDMCFG0
    {0x15, 0x44},  // DEVIATN
    {0x17, 0x00},  // MCSM1
    {0x18, 0x18},  // MCSM0    autocal IDLE->RX
    {0x19, 0x2E},  // FOCCFG
    {0x1A, 0xBF},  // BSCFG
    {0x1B, 0x43},  // AGCCTRL2
    {0x1C, 0x09},  // AGCCTRL1
    {0x1D, 0xB5},  // AGCCTRL0
    {0x21, 0xB6},  // FREND1
    {0x22, 0x10},  // FREND0
    {0x23, 0xEA},  // FSCAL3
    {0x24, 0x2A},  // FSCAL2
    {0x25, 0x00},  // FSCAL1
    {0x26, 0x1F},  // FSCAL0
    {0x29, 0x59},  // FSTEST
    {0x2C, 0x81},  // TEST2
    {0x2D, 0x35},  // TEST1
    {0x2E, 0x09},  // TEST0
};


static inline uint32_t read_le32(const uint8_t *p) {
  return (uint32_t) p[0] | ((uint32_t) p[1] << 8) | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
void FlowIQ2200Component::set_key(const std::string &hex_key) {
  uint8_t key[16];
  this->key_ok_ = hex_key.size() == 32 && parse_hex(hex_key, key, 16);
  if (this->key_ok_)
    this->aes_.set_key(key);
}

void IRAM_ATTR FlowIQ2200Component::gpio_intr(FlowIQ2200Component *arg) { arg->sync_flag_ = true; }

void FlowIQ2200Component::setup() {
  if (!this->key_ok_) {
    ESP_LOGE(TAG, "Invalid AES key");
    this->mark_failed();
    return;
  }

  this->spi_setup();
  this->gdo0_pin_->setup();
  this->reset_radio_();

  const uint8_t partnum = this->read_status_(CC1101_PARTNUM);
  this->chip_version_ = this->read_status_(CC1101_VERSION);
  if (this->chip_version_ == 0x00 || this->chip_version_ == 0xFF) {
    ESP_LOGE(TAG, "CC1101 not responding (PARTNUM=0x%02X VERSION=0x%02X) - check wiring and 3.3V supply", partnum,
             this->chip_version_);
    this->mark_failed();
    return;
  }

  for (const auto &rv : CC1101_INIT)
    this->write_reg_(rv.reg, rv.value);

  this->strobe_(CC1101_SCAL);
  delay(1);

  this->gdo0_pin_->attach_interrupt(&FlowIQ2200Component::gpio_intr, this, gpio::INTERRUPT_RISING_EDGE);

  if (!this->start_receiver_()) {
    ESP_LOGE(TAG, "CC1101 did not enter RX state");
    this->mark_failed();
    return;
  }

  // Telegrams arrive at 100 kbps into a 64-byte FIFO, so the loop must run often.
  this->high_freq_.start();
}

void FlowIQ2200Component::dump_config() {
  ESP_LOGCONFIG(TAG, "FlowIQ 2200 (CC1101 wM-Bus mode C):");
  ESP_LOGCONFIG(TAG, "  Meter ID: %08" PRIX32, this->meter_id_);
  ESP_LOGCONFIG(TAG, "  AES key: %s", this->key_ok_ ? "set" : "INVALID");
  ESP_LOGCONFIG(TAG, "  CC1101 version: 0x%02X", this->chip_version_);
  LOG_SPI_DEVICE(this);
  LOG_PIN("  GDO0 Pin: ", this->gdo0_pin_);
  if (this->is_failed())
    ESP_LOGE(TAG, "  Setup failed - see log above");
  ESP_LOGCONFIG(TAG, "  Known compact frame layouts: %u", (unsigned) this->decoder_.layout_count());
  for (auto *s : this->numeric_)
    LOG_SENSOR("  ", "Sensor", s);
  LOG_SENSOR("  ", "RSSI", this->rssi_sensor_);
  for (auto *s : this->text_)
    LOG_TEXT_SENSOR("  ", "Text sensor", s);
  for (auto *s : this->binary_)
    LOG_BINARY_SENSOR("  ", "Binary sensor", s);
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void FlowIQ2200Component::loop() {
  if (this->sync_flag_ || this->gdo0_pin_->digital_read()) {
    const uint8_t len = this->receive_frame_();
    this->sync_flag_ = false;
    this->start_receiver_();  // restart ASAP so the next telegram isn't missed
    if (len > 0)
      this->handle_frame_(&this->frame_buf_[3], len);
    return;
  }

  // Watchdog: make sure the radio is still listening.
  const uint32_t now = millis();
  if (now - this->last_health_check_ > 5000) {
    this->last_health_check_ = now;
    const uint8_t state = this->read_status_(CC1101_MARCSTATE) & 0x1F;
    if (state != MARCSTATE_RX) {
      ESP_LOGW(TAG, "CC1101 left RX (MARCSTATE=0x%02X), restarting receiver", state);
      this->start_receiver_();
    }
  }
}

// ---------------------------------------------------------------------------
// Reception
// ---------------------------------------------------------------------------
uint8_t FlowIQ2200Component::receive_frame_() {
  // RSSI while the packet is still on air
  const uint8_t rssi_raw = this->read_status_(CC1101_RSSI);
  this->last_rssi_ = (int8_t) (((rssi_raw >= 128) ? ((int) rssi_raw - 256) / 2 : rssi_raw / 2) - 74);

  size_t need = 3;  // 0x54 0x3D L
  size_t got = 0;
  const uint32_t start = millis();

  while (got < need) {
    if (millis() - start > 60) {  // 258 bytes at 100 kbps is ~21 ms
      ESP_LOGV(TAG, "RX timeout (%u/%u bytes)", (unsigned) got, (unsigned) need);
      return 0;
    }

    const uint8_t rxbytes = this->rx_bytes_();
    const bool overflow = (rxbytes & 0x80) != 0;
    const uint8_t avail = rxbytes & 0x7F;

    // CC1101 errata: never empty the FIFO while still receiving. In infinite
    // packet mode the radio keeps clocking in bytes, so there's always one more.
    size_t n = overflow ? avail : (avail > 1 ? avail - 1 : 0);
    if (n > need - got)
      n = need - got;

    if (n == 0) {
      if (overflow) {
        ESP_LOGD(TAG, "RX FIFO overflow (%u/%u bytes) - loop too slow", (unsigned) got, (unsigned) need);
        return 0;
      }
      delayMicroseconds(80);
      continue;
    }

    this->read_fifo_(&this->frame_buf_[got], n);
    got += n;

    if (need == 3 && got >= 3) {
      if (this->frame_buf_[0] != 0x54 || this->frame_buf_[1] != 0x3D) {
        ESP_LOGVV(TAG, "Not a mode C / format B frame: %02X %02X", this->frame_buf_[0], this->frame_buf_[1]);
        return 0;
      }
      need = 3 + (size_t) this->frame_buf_[2];
    }

    if (overflow && got < need) {
      ESP_LOGD(TAG, "RX FIFO overflow (%u/%u bytes) - loop too slow", (unsigned) got, (unsigned) need);
      return 0;
    }
  }
  return this->frame_buf_[2];
}

void FlowIQ2200Component::handle_frame_(const uint8_t *payload, uint8_t len) {
  // payload: C(0) M(1-2) A-ID(3-6) ver(7) type(8) CI=0x8D(9) CC(10) ACC(11) SN(12-15) | enc... | CRC(2)
  if (len < 16 + 2 + 4) {
    this->frames_bad_++;
    return;
  }

  const uint32_t id = read_le32(&payload[3]);
  if (id != this->meter_id_) {
    this->frames_other_++;
    ESP_LOGV(TAG, "Ignoring telegram from meter %08" PRIX32 " (RSSI %d dBm)", id, this->last_rssi_);
    return;
  }

  uint8_t iv[16] = {0};
  memcpy(iv, &payload[1], 8);  // M + A
  iv[8] = payload[10];         // CC
  memcpy(&iv[9], &payload[12], 4);  // SN

  const size_t cipher_len = (size_t) len - 2 - 16;
  uint8_t plain[255];
  aes128_ctr(this->aes_, iv, &payload[16], plain, cipher_len);

#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERY_VERBOSE
  char hex[format_hex_pretty_size(255)];
  ESP_LOGVV(TAG, "Decrypted: %s", format_hex_pretty_to(hex, plain, cipher_len, ' '));
#endif
  this->handle_plaintext_(plain, cipher_len);
}

void FlowIQ2200Component::handle_plaintext_(const uint8_t *data, size_t len) {
  Reading r;
  const DecodeResult res = this->decoder_.decode(data, len, &r);
  const uint8_t ci = this->decoder_.last_ci();
  const uint16_t sig = this->decoder_.last_signature();

  switch (res) {
    case DecodeResult::OK:
      break;
    case DecodeResult::CRC_ERROR:
      this->frames_bad_++;
      ESP_LOGW(TAG, "Payload CRC mismatch (len=%u) - wrong AES key or corrupted telegram", (unsigned) len);
      return;
    case DecodeResult::UNKNOWN_FORMAT:
      if (sig != this->last_unknown_signature_) {
        ESP_LOGW(TAG, "Compact frame with unknown format signature %04X - waiting for a full frame to learn it", sig);
        this->last_unknown_signature_ = sig;
      }
      return;
    case DecodeResult::FORMAT_MISMATCH:
      this->frames_bad_++;
      ESP_LOGW(TAG, "Frame CI=0x%02X (signature %04X) does not match its record layout", ci, sig);
      return;
    case DecodeResult::UNSUPPORTED_CI:
      ESP_LOGD(TAG, "Unhandled CI 0x%02X (len=%u)", ci, (unsigned) len);
      return;
    default:
      this->frames_bad_++;
      return;
  }
  this->frames_ok_++;

  if (ci == 0x78) {
    ESP_LOGD(TAG, "Full frame (format %04X%s), RSSI %d dBm", sig,
             this->decoder_.last_learned() ? ", new layout learned" : "", this->last_rssi_);
  } else {
    ESP_LOGD(TAG, "Compact frame (format %04X), RSSI %d dBm", sig, this->last_rssi_);
    if (!this->decoder_.last_data_crc_ok()) {
      ESP_LOGV(TAG, "Compact frame data CRC differs from rebuilt full frame (layout %04X)", sig);
    }
  }
  this->publish_(r);
}

void FlowIQ2200Component::publish_(const Reading &r) {
  if (this->rssi_sensor_ != nullptr)
    this->rssi_sensor_->publish_state(this->last_rssi_);

  for (int i = 0; i < NUM_FIELD_COUNT; i++) {
    if (!std::isnan(r.num[i]) && this->numeric_[i] != nullptr)
      this->numeric_[i]->publish_state(r.num[i]);
  }
  ESP_LOGD(TAG, "  total=%.3f m3, target=%.3f m3, flow=%.0f L/h, status=%s", r.num[NUM_TOTAL_M3],
           r.num[NUM_TARGET_M3], r.num[NUM_FLOW_LPH], r.status_text().c_str());

  if (r.has_target_date && this->text_[TXT_TARGET_DATE] != nullptr) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%04u-%02u-%02u", (unsigned) (r.target_year % 10000u),
             (unsigned) (r.target_month % 100u), (unsigned) (r.target_day % 100u));
    this->text_[TXT_TARGET_DATE]->publish_state(buf);
  }
  if (r.has_noise && this->text_[TXT_ACOUSTIC_NOISE] != nullptr) {
    char buf[13];
    for (int i = 0; i < 6; i++)
      snprintf(&buf[i * 2], 3, "%02X", r.noise[i]);
    this->text_[TXT_ACOUSTIC_NOISE]->publish_state(buf);
  }

  if (r.status_kind == STATUS_NONE)
    return;
  if (this->text_[TXT_STATUS] != nullptr)
    this->text_[TXT_STATUS]->publish_state(r.status_text());

  static const uint8_t DURATIONS[][2] = {
      {TXT_TIME_DRY, DUR_DRY},           {TXT_TIME_REVERSED, DUR_REVERSED},
      {TXT_TIME_LEAKING, DUR_LEAKING},   {TXT_TIME_BURSTING, DUR_BURSTING},
      {TXT_TIME_AMBIENT_TEMP, DUR_AMBIENT_TEMP}, {TXT_TIME_FLOW_ABOVE_Q4, DUR_FLOW_ABOVE_Q4},
  };
  for (const auto &d : DURATIONS) {
    auto *ts = this->text_[d[0]];
    const auto dur = (StatusDuration) d[1];
    if (ts != nullptr && r.duration_available(dur))
      ts->publish_state(duration_text(r.duration_code(dur)));
  }

  static const uint8_t FLAGS[][2] = {
      {BIN_DRY, FLAG_DRY},           {BIN_REVERSE, FLAG_REVERSE},
      {BIN_LEAK, FLAG_LEAK},         {BIN_BURST, FLAG_BURST},
      {BIN_TAMPER, FLAG_TAMPER},     {BIN_LOW_BATTERY, FLAG_LOW_BATTERY},
      {BIN_AMBIENT_TEMP, FLAG_AMBIENT_TEMP}, {BIN_FLOW_ABOVE_Q4, FLAG_FLOW_ABOVE_Q4},
      {BIN_NO_CONSUMPTION, FLAG_NO_CONSUMPTION},
  };
  for (const auto &f : FLAGS) {
    auto *bs = this->binary_[f[0]];
    const auto flag = (StatusFlag) f[1];
    if (bs != nullptr && r.flag_available(flag))
      bs->publish_state(r.flag(flag));
  }
}

// ---------------------------------------------------------------------------
// CC1101 low level
// ---------------------------------------------------------------------------
void FlowIQ2200Component::reset_radio_() {
  // Manual power-on reset (datasheet 19.1.2): toggle CS, then SRES.
  this->enable();
  delayMicroseconds(10);
  this->disable();
  delayMicroseconds(50);
  this->enable();
  delayMicroseconds(200);
  this->write_byte(CC1101_SRES);
  this->disable();
  delay(5);
}

void FlowIQ2200Component::write_reg_(uint8_t reg, uint8_t value) {
  this->enable();
  this->write_byte(reg);
  this->write_byte(value);
  this->disable();
}

uint8_t FlowIQ2200Component::read_status_(uint8_t reg) {
  this->enable();
  this->write_byte(reg | 0xC0);
  const uint8_t value = this->read_byte();
  this->disable();
  return value;
}

void FlowIQ2200Component::strobe_(uint8_t cmd) {
  this->enable();
  this->write_byte(cmd);
  this->disable();
}

void FlowIQ2200Component::read_fifo_(uint8_t *buf, size_t len) {
  this->enable();
  this->write_byte(CC1101_RXFIFO_BURST);
  this->read_array(buf, len);
  this->disable();
}

uint8_t FlowIQ2200Component::rx_bytes_() {
  // CC1101 errata: RXBYTES can be wrong while updating - read until stable.
  uint8_t a, b = this->read_status_(CC1101_RXBYTES);
  do {
    a = b;
    b = this->read_status_(CC1101_RXBYTES);
  } while (a != b);
  return a;
}

bool FlowIQ2200Component::wait_marcstate_(uint8_t state, uint32_t timeout_ms) {
  const uint32_t start = millis();
  while ((this->read_status_(CC1101_MARCSTATE) & 0x1F) != state) {
    if (millis() - start > timeout_ms)
      return false;
    delayMicroseconds(100);
  }
  return true;
}

bool FlowIQ2200Component::start_receiver_() {
  this->strobe_(CC1101_SIDLE);
  if (!this->wait_marcstate_(MARCSTATE_IDLE, 20))
    return false;
  this->strobe_(CC1101_SFRX);
  this->strobe_(CC1101_SRX);
  return this->wait_marcstate_(MARCSTATE_RX, 20);
}

}  // namespace flowiq2200
}  // namespace esphome
