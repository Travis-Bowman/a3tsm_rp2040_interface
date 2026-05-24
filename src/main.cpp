// ********************
// * a3tsm_rp2040_interface *
// ********************
//
// FORWARD PATH (host -> RP2040 -> CAN):
// Serial command packet (9 bytes, host -> RP2040):
//   Byte 0:     SOF byte 1 (0xAA)
//   Byte 1:     SOF byte 2 (0x55)
//   Byte 2:     Sequence number (uint8)
//   Byte 3:     Flags (uint8 bit field)
//   Byte 4-5:   frontLeftSpeed  (int16, little-endian, mm/s)
//   Byte 6-7:   frontRightSpeed (int16, little-endian, mm/s)
//   Byte 8:     CRC-8 over bytes 2-7
//
// CAN command frame (8 bytes, RP2040 -> motor controllers) on 0x120 / 0x121.
//
// RETURN PATH (CAN -> RP2040 -> host):
// CAN feedback frame (8 bytes, motor controller -> RP2040) on 0x220 / 0x221:
//   Byte 0-3:  tick_count (int32, little-endian)   [ignored here]
//   Byte 4-5:  velocity   (int16, little-endian, mm/s)
//   Byte 6:    flags (bit0 = magnet_detected)      [ignored here]
//   Byte 7:    AGC                                  [ignored here]
//
// Serial feedback packet (9 bytes, RP2040 -> host) -- MIRRORS the forward
// command packet exactly, just the opposite direction:
//   Byte 0:     SOF byte 1 (0xAA)
//   Byte 1:     SOF byte 2 (0x55)
//   Byte 2:     Sequence number (uint8)
//   Byte 3:     Flags (bit0 = FL stale, bit1 = FR stale)
//   Byte 4-5:   FL velocity (int16, little-endian, mm/s)
//   Byte 6-7:   FR velocity (int16, little-endian, mm/s)
//   Byte 8:     CRC-8 over bytes 2-7
 
#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <SPI.h>
#include "mcp25125_config.h"
#include "neopixel_config.h"
#include <Adafruit_NeoPixel.h>
 
struct Motor {
  uint32_t can_id_tx;
  uint32_t can_id_rx;
  int16_t  cmd_speed; // mm/s
};
 
struct MotorFeedback {
  int16_t       velocity_mm_s;  // measured wheel velocity, mm/s
  unsigned long last_rx_ms;     // when we last heard from this wheel
  bool          ever_seen;
};
 
Motor motors[2] = {
  {mcp25125_config::CAN_ID_FL_TX, mcp25125_config::CAN_ID_FL_RX, 0},
  {mcp25125_config::CAN_ID_FR_TX, mcp25125_config::CAN_ID_FR_RX, 0}
};
 
MotorFeedback feedback[2] = {
  {0, 0, false},
  {0, 0, false}
};
 
// Feedback considered stale if no CAN frame within this window.
static constexpr unsigned long FEEDBACK_STALE_MS = 200;
// How often we push a feedback packet up the serial link.
static constexpr unsigned long FEEDBACK_TX_MS    = 20;   // 50 Hz
 
Adafruit_NeoPixel pixel(1, neopixel_config::NEOPIXEL_DATA_PIN, NEO_GRB + NEO_KHZ800);
 
Adafruit_MCP2515 mcp(mcp25125_config::PIN_CAN_CS,
                     mcp25125_config::PIN_CAN_MOSI,
                     mcp25125_config::PIN_CAN_MISO,
                     mcp25125_config::PIN_CAN_SCK);
 
static uint8_t crc8_atm(const uint8_t* data, size_t len, uint8_t poly = 0x07, uint8_t init = 0x00) {
  uint8_t crc = init;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ poly);
      else crc <<= 1;
    }
  }
  return crc;
}
 
void send_can_packet(uint8_t seq, uint8_t flags) {
  uint8_t data[8];
  data[0] = 0xAA;
  data[1] = 0x55;
  data[2] = seq;
  data[3] = flags;
  data[4] = motors[0].cmd_speed & 0xFF;         // left speed low
  data[5] = (motors[0].cmd_speed >> 8) & 0xFF;  // left speed high
  data[6] = motors[1].cmd_speed & 0xFF;         // right speed low
  data[7] = (motors[1].cmd_speed >> 8) & 0xFF;  // right speed high
 
  mcp.beginPacket(mcp25125_config::CAN_ID_FL_TX);
  mcp.write(data, sizeof(data));
  mcp.endPacket();
 
  mcp.beginPacket(mcp25125_config::CAN_ID_FR_TX);
  mcp.write(data, sizeof(data));
  mcp.endPacket();
}
 
// Returns true if a valid command packet was received and motors[] updated.
bool read_serial_packet(uint8_t& seq, uint8_t& flags) {
  int b0 = Serial.read();
  if (b0 != 0xAA) return false;
 
  while (Serial.available() == 0) {}
  int b1 = Serial.read();
  if (b1 != 0x55) return false;
 
  uint8_t rest[7];
  if (Serial.readBytes((char*)rest, 7) != 7) return false;
 
  uint8_t rx_crc = rest[6];
  uint8_t calc   = crc8_atm(rest, 6);
  if (calc != rx_crc) return false;
 
  seq   = rest[0];
  flags = rest[1];
  for (int i = 0; i < 2; i++) {
    int j = 2 + i * 2;  // 2 bytes per motor (speed only)
    motors[i].cmd_speed = (int16_t)(rest[j] | (rest[j+1] << 8));
  }
  return true;
}
 
// Decode a CAN feedback frame. We only care about velocity (bytes 4-5).
// Returns wheel index (0=FL, 1=FR) on success, or -1 if not a feedback ID.
int decode_can_feedback(uint32_t id, const uint8_t* data, uint8_t len) {
  int idx = -1;
  if      (id == mcp25125_config::CAN_ID_FL_RX) idx = 0;
  else if (id == mcp25125_config::CAN_ID_FR_RX) idx = 1;
  else return -1;
 
  if (len != 8) return -1;
 
  int16_t vel = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
  feedback[idx].velocity_mm_s = vel;
  feedback[idx].last_rx_ms    = millis();
  feedback[idx].ever_seen     = true;
  return idx;
}
 
// Build and TX a 9-byte feedback packet to the host -- mirrors forward packet.
void send_serial_feedback() {
  static uint8_t fb_seq = 0;
  unsigned long now = millis();
 
  bool fl_stale = !feedback[0].ever_seen ||
                  (now - feedback[0].last_rx_ms > FEEDBACK_STALE_MS);
  bool fr_stale = !feedback[1].ever_seen ||
                  (now - feedback[1].last_rx_ms > FEEDBACK_STALE_MS);
 
  uint8_t flags = 0;
  if (fl_stale) flags |= 0x01;
  if (fr_stale) flags |= 0x02;
 
  uint8_t pkt[9];
  pkt[0] = 0xAA;
  pkt[1] = 0x55;
  pkt[2] = fb_seq;
  pkt[3] = flags;
 
  int16_t fl_vel = feedback[0].velocity_mm_s;
  pkt[4] = (uint8_t)( fl_vel       & 0xFF);
  pkt[5] = (uint8_t)((fl_vel >> 8) & 0xFF);
 
  int16_t fr_vel = feedback[1].velocity_mm_s;
  pkt[6] = (uint8_t)( fr_vel       & 0xFF);
  pkt[7] = (uint8_t)((fr_vel >> 8) & 0xFF);
 
  // CRC-8 over bytes 2..7, mirroring forward packet.
  pkt[8] = crc8_atm(&pkt[2], 6);
 
  Serial.write(pkt, sizeof(pkt));
  fb_seq++;
}
 
void setup() {
  Serial.begin(921600);
  while (!Serial) { delay(10); }
 
  pinMode(mcp25125_config::PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(mcp25125_config::PIN_CAN_STANDBY, LOW);   // normal operation
 
  pinMode(mcp25125_config::PIN_CAN_RESET, OUTPUT);
  digitalWrite(mcp25125_config::PIN_CAN_RESET, HIGH);    // not in reset
 
  delay(10);
 
  if (!mcp.begin(mcp25125_config::CAN_BITRATE)) {
    // No debug print on the wire -- it would corrupt the binary stream.
    // Hold here with a red LED instead.
    pinMode(neopixel_config::NEOPIXEL_POWER_PIN, OUTPUT);
    digitalWrite(neopixel_config::NEOPIXEL_POWER_PIN, HIGH);
    pixel.begin();
    pixel.setBrightness(20);
    pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    pixel.show();
    while (1) delay(10);
  }
 
  pinMode(neopixel_config::NEOPIXEL_POWER_PIN, OUTPUT);
  digitalWrite(neopixel_config::NEOPIXEL_POWER_PIN, HIGH);
  pixel.begin();
  pixel.setBrightness(20);
  pixel.clear();
  pixel.show();
}
 
void loop() {
  unsigned long now = millis();
 
  // --- FORWARD: host serial command -> CAN ---
  if (Serial.available() > 0) {
    uint8_t seq, flags;
    if (read_serial_packet(seq, flags)) {
      send_can_packet(seq, flags);
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));  // green on valid cmd
      pixel.show();
      pixel.clear();
      pixel.show();
    }
  }
 
  // --- RETURN: CAN feedback -> cache ---
  int packetSize = mcp.parsePacket();
  if (packetSize > 0) {
    uint32_t id = mcp.packetId();
    uint8_t  data[8] = {0};
    uint8_t  expected = min((uint8_t)packetSize, (uint8_t)8);
    uint8_t  len = 0;
    for (; len < expected; len++) {
      int c = mcp.read();
      if (c < 0) break;
      data[len] = (uint8_t)c;
    }
    decode_can_feedback(id, data, len);
  }
 
  // --- RETURN: cache -> host serial (timed) ---
  static unsigned long last_fb_tx = 0;
  if (now - last_fb_tx >= FEEDBACK_TX_MS) {
    last_fb_tx = now;
    send_serial_feedback();
  }
}
 