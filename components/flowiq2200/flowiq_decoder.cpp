// FlowIQ 2200 wM-Bus decoding helpers. GPLv3 or later (see flowiq_decoder.h).
#include "flowiq_decoder.h"

#include <cstdio>
#include <cstring>

namespace esphome {
namespace flowiq2200 {

// ===========================================================================
// AES-128
// ===========================================================================
static const uint8_t SBOX[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76, 0xca, 0x82, 0xc9,
    0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f,
    0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15, 0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07,
    0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3,
    0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58,
    0xcf, 0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3,
    0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec, 0x5f,
    0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73, 0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88,
    0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac,
    0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a,
    0xae, 0x08, 0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, 0x70,
    0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1, 0xf8, 0x98, 0x11,
    0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf, 0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42,
    0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16};

static inline uint8_t xtime(uint8_t x) { return (uint8_t) ((x << 1) ^ ((x & 0x80) ? 0x1B : 0x00)); }

void Aes128::set_key(const uint8_t key[16]) {
  static const uint8_t RCON[11] = {0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1B, 0x36};
  memcpy(this->round_keys_, key, 16);
  for (int i = 4; i < 44; i++) {
    uint8_t t[4];
    memcpy(t, &this->round_keys_[(i - 1) * 4], 4);
    if (i % 4 == 0) {
      const uint8_t first = t[0];
      t[0] = (uint8_t) (SBOX[t[1]] ^ RCON[i / 4]);
      t[1] = SBOX[t[2]];
      t[2] = SBOX[t[3]];
      t[3] = SBOX[first];
    }
    for (int j = 0; j < 4; j++)
      this->round_keys_[i * 4 + j] = (uint8_t) (this->round_keys_[(i - 4) * 4 + j] ^ t[j]);
  }
}

void Aes128::encrypt_block(const uint8_t in[16], uint8_t out[16]) const {
  uint8_t s[16];
  for (int i = 0; i < 16; i++)
    s[i] = (uint8_t) (in[i] ^ this->round_keys_[i]);

  for (int round = 1; round <= 10; round++) {
    // SubBytes + ShiftRows (state is column-major: s[row + 4*col])
    uint8_t t[16];
    for (int c = 0; c < 4; c++)
      for (int r = 0; r < 4; r++)
        t[r + 4 * c] = SBOX[s[r + 4 * ((c + r) % 4)]];

    // MixColumns (skipped in the final round)
    if (round != 10) {
      for (int c = 0; c < 4; c++) {
        uint8_t *col = &t[4 * c];
        const uint8_t a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3];
        const uint8_t all = (uint8_t) (a0 ^ a1 ^ a2 ^ a3);
        col[0] = (uint8_t) (a0 ^ all ^ xtime((uint8_t) (a0 ^ a1)));
        col[1] = (uint8_t) (a1 ^ all ^ xtime((uint8_t) (a1 ^ a2)));
        col[2] = (uint8_t) (a2 ^ all ^ xtime((uint8_t) (a2 ^ a3)));
        col[3] = (uint8_t) (a3 ^ all ^ xtime((uint8_t) (a3 ^ a0)));
      }
    }

    // AddRoundKey
    for (int i = 0; i < 16; i++)
      s[i] = (uint8_t) (t[i] ^ this->round_keys_[round * 16 + i]);
  }
  memcpy(out, s, 16);
}

void aes128_ctr(const Aes128 &aes, const uint8_t iv[16], const uint8_t *in, uint8_t *out, size_t len) {
  uint8_t counter[16];
  uint8_t stream[16];
  memcpy(counter, iv, 16);
  for (size_t pos = 0; pos < len; pos += 16) {
    aes.encrypt_block(counter, stream);
    const size_t n = (len - pos < 16) ? (len - pos) : 16;
    for (size_t i = 0; i < n; i++)
      out[pos + i] = (uint8_t) (in[pos + i] ^ stream[i]);
    // increment counter (big-endian)
    for (int i = 15; i >= 0; i--) {
      if (++counter[i] != 0)
        break;
    }
  }
}

// ===========================================================================
// CRC
// ===========================================================================
uint16_t crc16_en13757(const uint8_t *data, size_t len) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    for (int bit = 0; bit < 8; bit++) {
      if (((crc & 0x8000) >> 8) ^ (b & 0x80)) {
        crc = (uint16_t) ((crc << 1) ^ 0x3D65);
      } else {
        crc = (uint16_t) (crc << 1);
      }
      b <<= 1;
    }
  }
  return (uint16_t) ~crc;
}


// ===========================================================================
// Status helpers (wmbusmeters kamwater driver)
// ===========================================================================
static const char *const DURATION_TEXT[8] = {"none",     "1-8 hours", "9-24 hours", "2-3 days",
                                             "4-7 days", "8-14 days", "15-21 days", "22-31 days"};

const char *duration_text(uint8_t code) { return DURATION_TEXT[code & 7]; }

bool Reading::flag_available(StatusFlag flag) const {
  if (this->status_kind == STATUS_NONE)
    return false;
  if (this->status_kind == STATUS_16)
    return flag <= FLAG_BURST;
  return true;
}

bool Reading::duration_available(StatusDuration d) const {
  if (this->status_kind == STATUS_NONE)
    return false;
  if (this->status_kind == STATUS_16)
    return d <= DUR_BURSTING;
  return true;
}

uint8_t Reading::duration_code(StatusDuration d) const {
  static const uint8_t SHIFT16[4] = {4, 7, 10, 13};
  static const uint8_t SHIFT32[6] = {11, 14, 17, 20, 23, 26};
  if (!this->duration_available(d))
    return 0;
  const uint8_t shift = (this->status_kind == STATUS_16) ? SHIFT16[d] : SHIFT32[d];
  return (uint8_t) ((this->status >> shift) & 7u);
}

std::string Reading::status_text() const {
  if (this->status_kind == STATUS_NONE)
    return "";
  const uint32_t mask = (this->status_kind == STATUS_16) ? 0x0000000Fu : 0xE00003FFu;
  const uint32_t bits = this->status & mask;
  std::string out;
  for (int bit = 0; bit < 32; bit++) {
    if (!(bits & (1u << bit)))
      continue;
    const char *name = nullptr;
    char buf[20];
    switch (bit) {
      case FLAG_DRY: name = "DRY"; break;
      case FLAG_REVERSE: name = "REVERSE"; break;
      case FLAG_LEAK: name = "LEAK"; break;
      case FLAG_BURST: name = "BURST"; break;
      case FLAG_TAMPER: name = "TAMPER"; break;
      case FLAG_LOW_BATTERY: name = "LOW_BATTERY"; break;
      case FLAG_AMBIENT_TEMP:
        name = this->status_kind == STATUS_32_HOT ? "HIGH_AMBIENT_TEMPERATURE" : "LOW_AMBIENT_TEMPERATURE";
        break;
      case FLAG_FLOW_ABOVE_Q4: name = "FLOW_ABOVE_Q4"; break;
      case FLAG_NO_CONSUMPTION: name = "NO_CONSUMPTION"; break;
      default:
        snprintf(buf, sizeof(buf), "RESERVED_BIT_%d", bit);
        name = buf;
        break;
    }
    if (!out.empty())
      out += ' ';
    out += name;
  }
  return out.empty() ? "OK" : out;
}

// ===========================================================================
// DIF/VIF records
// ===========================================================================
namespace {

struct Header {
  uint8_t dif{0};
  uint8_t function{0};   // 0 instantaneous, 1 maximum, 2 minimum, 3 error
  uint32_t storage{0};
  uint8_t vif{0};        // raw first VIF byte
  uint8_t vife{0};       // first VIFE (only meaningful if vif has the extension bit)
  int data_len{0};       // -1 = variable length (LVAR byte follows)
  bool bcd{false};
  bool is_real{false};
  size_t header_len{0};
};

enum class HeaderResult { OK, FILLER, END, ERROR };

// Parses one record header starting at p[pos].
HeaderResult parse_header(const uint8_t *p, size_t len, size_t pos, Header *h) {
  if (pos >= len)
    return HeaderResult::END;
  const size_t start = pos;
  const uint8_t dif = p[pos++];
  if (dif == 0x2F)
    return HeaderResult::FILLER;
  if (dif == 0x0F || dif == 0x1F)
    return HeaderResult::END;  // manufacturer specific data until the end

  static const int8_t LEN[16] = {0, 1, 2, 3, 4, 4, 6, 8, 0, 1, 2, 3, 4, -1, 6, -2};
  const uint8_t lfield = dif & 0x0F;
  if (LEN[lfield] == -2)
    return HeaderResult::ERROR;

  h->dif = dif;
  h->function = (dif >> 4) & 0x03;
  h->data_len = LEN[lfield];
  h->bcd = (lfield >= 0x09 && lfield <= 0x0C) || lfield == 0x0E;
  h->is_real = lfield == 0x05;
  h->storage = (dif >> 6) & 0x01;

  bool ext = (dif & 0x80) != 0;
  uint8_t shift = 1;
  while (ext) {
    if (pos >= len)
      return HeaderResult::ERROR;
    const uint8_t dife = p[pos++];
    h->storage |= (uint32_t) (dife & 0x0F) << shift;
    shift += 4;
    ext = (dife & 0x80) != 0;
  }

  if (pos >= len)
    return HeaderResult::ERROR;
  h->vif = p[pos++];
  h->vife = 0;
  ext = (h->vif & 0x80) != 0;
  bool first = true;
  while (ext) {
    if (pos >= len)
      return HeaderResult::ERROR;
    const uint8_t vife = p[pos++];
    if (first)
      h->vife = vife;
    first = false;
    ext = (vife & 0x80) != 0;
  }
  h->header_len = pos - start;
  return HeaderResult::OK;
}

bool decode_number(const Header &h, const uint8_t *d, int len, double *out) {
  if (h.is_real) {
    float f;
    memcpy(&f, d, 4);
    *out = f;
    return true;
  }
  if (h.bcd) {
    double v = 0, mul = 1;
    for (int i = 0; i < len; i++) {
      const uint8_t lo = d[i] & 0x0F, hi = d[i] >> 4;
      if (lo > 9 || (hi > 9 && !(i == len - 1 && hi == 0x0F)))
        return false;
      v += lo * mul;
      mul *= 10;
      if (hi == 0x0F) {
        v = -v;  // sign nibble
      } else {
        v += hi * mul;
      }
      mul *= 10;
    }
    *out = v;
    return true;
  }
  if (len < 1 || len > 8)
    return false;
  uint64_t u = 0;
  for (int i = 0; i < len; i++)
    u |= (uint64_t) d[i] << (8 * i);
  if (len < 8 && (d[len - 1] & 0x80))
    u |= ~0ULL << (8 * len);  // sign-extend (EN 13757-3 type B integers are signed)
  *out = (double) (int64_t) u;
  return true;
}

void apply_record(const Header &h, const uint8_t *d, int len, Reading *r) {
  // Manufacturer-specific Kamstrup fields
  if (h.vif == 0xFF) {
    const uint8_t code = h.vife & 0x7F;
    if (code == 0x20 && len == 2) {
      r->status_kind = STATUS_16;
      r->status = (uint32_t) d[0] | ((uint32_t) d[1] << 8);
    } else if ((code == 0x23 || code == 0x24) && len == 4) {
      r->status_kind = code == 0x24 ? STATUS_32_HOT : STATUS_32_COLD;
      r->status = (uint32_t) d[0] | ((uint32_t) d[1] << 8) | ((uint32_t) d[2] << 16) | ((uint32_t) d[3] << 24);
    } else if (code == 0x1B && len == 6) {
      r->has_noise = true;
      memcpy(r->noise, d, 6);
    }
    return;
  }

  // Extended VIFs (e.g. E7 FF 0F) change the meaning - ignore them.
  if (h.vif & 0x80)
    return;

  // Date type G: target (billing) date
  if (h.vif == 0x6C) {
    if (len == 2 && h.storage == 1) {
      r->target_day = d[0] & 0x1F;
      r->target_month = d[1] & 0x0F;
      r->target_year = 2000 + (((d[0] & 0xE0) >> 5) | ((d[1] & 0xF0) >> 1));
      r->has_target_date = true;
    }
    return;
  }

  double v;
  if (!decode_number(h, d, len, &v))
    return;
  const uint8_t vif = h.vif;

  if (vif >= 0x10 && vif <= 0x17) {  // volume, 10^(n-6) m3
    const double m3 = v * pow(10.0, (int) (vif & 0x07) - 6);
    if (h.function == 0 && h.storage == 0)
      r->num[NUM_TOTAL_M3] = (float) m3;
    else if (h.function == 0 && h.storage == 1)
      r->num[NUM_TARGET_M3] = (float) m3;
  } else if (vif >= 0x38 && vif <= 0x3F) {  // volume flow, 10^(n-6) m3/h
    const double lph = v * pow(10.0, (int) (vif & 0x07) - 3);
    if (h.function == 0 && h.storage == 0)
      r->num[NUM_FLOW_LPH] = (float) lph;
    else if (h.function == 1 && h.storage == 2)
      r->num[NUM_MAX_FLOW_LAST_DAY_LPH] = (float) lph;
    else if (h.function == 2 && h.storage == 2)
      r->num[NUM_MIN_FLOW_LAST_DAY_LPH] = (float) lph;
  } else if (vif >= 0x58 && vif <= 0x5B) {  // flow (water) temperature, 10^(n-3) C
    const double c = v * pow(10.0, (int) (vif & 0x03) - 3);
    if (h.function == 1 && h.storage == 2)
      r->num[NUM_MAX_FLOW_TEMP_LAST_DAY] = (float) c;
    else if (h.function == 2 && h.storage == 2)
      r->num[NUM_MIN_FLOW_TEMP_LAST_DAY] = (float) c;
  } else if (vif >= 0x64 && vif <= 0x67) {  // external (ambient) temperature, 10^(n-3) C
    const double c = v * pow(10.0, (int) (vif & 0x03) - 3);
    if (h.function == 1 && h.storage == 2)
      r->num[NUM_MAX_EXT_TEMP_LAST_DAY] = (float) c;
    else if (h.function == 2 && h.storage == 2)
      r->num[NUM_MIN_EXT_TEMP_LAST_DAY] = (float) c;
  }
}

// Walks a full record stream (headers + data), applying each record.
// Optionally collects the DIF/VIF header bytes (the "format") into difvif_out.
bool walk_records(const uint8_t *p, size_t len, Reading *r, uint8_t *difvif_out, size_t difvif_cap,
                  size_t *difvif_len) {
  size_t pos = 0, dv = 0;
  bool dv_ok = difvif_out != nullptr;
  while (pos < len) {
    Header h;
    const HeaderResult hr = parse_header(p, len, pos, &h);
    if (hr == HeaderResult::FILLER) {
      pos++;
      continue;
    }
    if (hr == HeaderResult::END)
      break;
    if (hr == HeaderResult::ERROR)
      return false;

    if (dv_ok) {
      if (dv + h.header_len <= difvif_cap) {
        memcpy(&difvif_out[dv], &p[pos], h.header_len);
        dv += h.header_len;
      } else {
        dv_ok = false;
      }
    }
    pos += h.header_len;

    int dlen = h.data_len;
    if (dlen < 0) {  // variable length
      if (pos >= len)
        return false;
      dlen = p[pos++];
      dv_ok = false;  // can't be represented in a compact frame
    }
    if (pos + (size_t) dlen > len)
      return false;
    apply_record(h, &p[pos], dlen, r);
    pos += (size_t) dlen;
  }
  if (difvif_len != nullptr)
    *difvif_len = dv_ok ? dv : 0;
  return true;
}

}  // namespace

// ===========================================================================
// FrameDecoder
// ===========================================================================

// Known Kamstrup compact frame layouts (wmbusmeters kamwater driver).
static const char *const BUILTIN_FORMATS[] = {
    // FlowIQ 2200 (KAW 3a 16) and Kamstrup KWM2231
    "04FF2304134413426C023B92013BA2013B06FF1BA1015B91015BA10167",
    "02FF2004134413615B6167",
    "02FF20041392013BA1015B8101E7FF0F",
    "04FF234413523B06FF1B426C61675167023B04138101E7FF0F",
    "02FF2004134413426C523B615B616751678101E7FF0F06FF1B",
    "02FF2004134413A1015B8101E7FF0F",
    "02FF2004134413615B5167",
    "02FF2004134413",
    "02FF200413523B",
};

FrameDecoder::FrameDecoder() {
  uint8_t buf[MAX_DIFVIF];
  for (const char *hex : BUILTIN_FORMATS) {
    const size_t n = strlen(hex) / 2;
    if (n > MAX_DIFVIF)
      continue;
    for (size_t i = 0; i < n; i++) {
      unsigned v;
      sscanf(&hex[i * 2], "%2x", &v);
      buf[i] = (uint8_t) v;
    }
    this->add_layout_(buf, n);
  }
  this->builtin_count_ = this->count_;
  this->next_learned_ = this->count_;
}

const FrameDecoder::Layout *FrameDecoder::find_layout_(uint16_t signature) const {
  for (size_t i = 0; i < this->count_; i++) {
    if (this->layouts_[i].signature == signature)
      return &this->layouts_[i];
  }
  return nullptr;
}

bool FrameDecoder::add_layout_(const uint8_t *difvif, size_t len) {
  if (len == 0 || len > MAX_DIFVIF)
    return false;
  const uint16_t sig = crc16_en13757(difvif, len);
  if (this->find_layout_(sig) != nullptr)
    return false;
  size_t slot;
  if (this->count_ < MAX_LAYOUTS) {
    slot = this->count_++;
  } else {
    // full: overwrite learned layouts round-robin, never the built-in ones
    if (this->builtin_count_ >= MAX_LAYOUTS)
      return false;
    slot = this->next_learned_;
    this->next_learned_ = (this->next_learned_ + 1 < MAX_LAYOUTS) ? this->next_learned_ + 1 : this->builtin_count_;
  }
  Layout &l = this->layouts_[slot];
  l.signature = sig;
  l.len = (uint8_t) len;
  memcpy(l.difvif, difvif, len);
  return true;
}

DecodeResult FrameDecoder::decode(const uint8_t *plain, size_t len, Reading *out) {
  this->last_learned_ = false;
  this->last_signature_ = 0;
  this->last_data_crc_ok_ = true;
  if (len < 3)
    return DecodeResult::TOO_SHORT;

  const uint16_t crc_read = (uint16_t) plain[0] | ((uint16_t) plain[1] << 8);
  if (crc16_en13757(plain + 2, len - 2) != crc_read)
    return DecodeResult::CRC_ERROR;

  this->last_ci_ = plain[2];

  if (plain[2] == 0x78) {
    // Full frame: plain DIF/VIF records. Learn its layout for later compact frames.
    uint8_t difvif[MAX_DIFVIF];
    size_t difvif_len = 0;
    if (!walk_records(plain + 3, len - 3, out, difvif, sizeof(difvif), &difvif_len))
      return DecodeResult::FORMAT_MISMATCH;
    if (difvif_len > 0) {
      this->last_signature_ = crc16_en13757(difvif, difvif_len);
      this->last_learned_ = this->add_layout_(difvif, difvif_len);
    }
    return DecodeResult::OK;
  }

  if (plain[2] == 0x79) {
    // Compact frame: CI, format signature(2), data CRC(2), then only the values.
    if (len < 7)
      return DecodeResult::TOO_SHORT;
    const uint16_t sig = (uint16_t) plain[3] | ((uint16_t) plain[4] << 8);
    const uint16_t data_crc = (uint16_t) plain[5] | ((uint16_t) plain[6] << 8);
    this->last_signature_ = sig;
    const Layout *layout = this->find_layout_(sig);
    if (layout == nullptr)
      return DecodeResult::UNKNOWN_FORMAT;

    // Rebuild the equivalent full record stream by interleaving headers and values.
    const uint8_t *data = plain + 7;
    const size_t data_len = len - 7;
    uint8_t stream[MAX_DIFVIF + 255];
    size_t s = 0, d = 0, pos = 0;
    while (pos < layout->len) {
      Header h;
      if (parse_header(layout->difvif, layout->len, pos, &h) != HeaderResult::OK || h.data_len < 0)
        return DecodeResult::FORMAT_MISMATCH;
      if (d + (size_t) h.data_len > data_len)
        return DecodeResult::FORMAT_MISMATCH;
      memcpy(&stream[s], &layout->difvif[pos], h.header_len);
      s += h.header_len;
      pos += h.header_len;
      memcpy(&stream[s], &data[d], (size_t) h.data_len);
      s += (size_t) h.data_len;
      d += (size_t) h.data_len;
    }
    // The data CRC normally covers the rebuilt full frame, confirming the layout. Integrity is
    // already guaranteed by the payload CRC, so a mismatch is only reported, not fatal.
    this->last_data_crc_ok_ = crc16_en13757(stream, s) == data_crc;
    if (!walk_records(stream, s, out, nullptr, 0, nullptr))
      return DecodeResult::FORMAT_MISMATCH;
    return DecodeResult::OK;
  }

  return DecodeResult::UNSUPPORTED_CI;
}

}  // namespace flowiq2200
}  // namespace esphome
