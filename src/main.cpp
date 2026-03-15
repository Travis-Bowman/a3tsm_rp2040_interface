// ********************

// * a3tsm_rp2040_interface *

// ********************

// Serial packet format (21 bytes total, host -> RP2040):
// Byte 0:     SOF byte 1 (0xAA)
// Byte 1:     SOF byte 2 (0x55)
// Byte 2:     Sequence number (uint8, 0-255, wraps around)
// Byte 3:     Flags (uint8 bit field)
// Byte 4-5:   frontLeftSpeed   (int16, little-endian, mm/s)
// Byte 6-7:   frontLeftSteer   (int16, little-endian, mrad)
// Byte 8-9:   frontRightSpeed  (int16, little-endian, mm/s)
// Byte 10-11: frontRightSteer  (int16, little-endian, mrad)
// Byte 12-13: rearLeftSpeed    (int16, little-endian, mm/s)
// Byte 14-15: rearLeftSteer    (int16, little-endian, mrad)
// Byte 16-17: rearRightSpeed   (int16, little-endian, mm/s)
// Byte 18-19: rearRightSteer   (int16, little-endian, mrad)
// Byte 20:    CRC-8 checksum over bytes 2-19

// CAN frame format (8 bytes, RP2040 -> motor controller):
// Byte 0:   SOF byte 1 (0xAA)
// Byte 1:   SOF byte 2 (0x55)
// Byte 2:   Sequence number (uint8)
// Byte 3:   Flags (uint8 bit field)
// Byte 4:   Speed low byte  (int16, little-endian, mm/s)
// Byte 5:   Speed high byte
// Byte 6:   Steer low byte  (int16, little-endian, mrad)
// Byte 7:   Steer high byte
// CAN IDs: FL=0x120, FR=0x121, RL=0x122, RR=0x123 (TX)
//          FL=0x220, FR=0x221, RL=0x222, RR=0x223 (RX feedback)

#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <SPI.h>
#include "mcp25125_config.h" 
#include "neopixel_config.h"
#include <Adafruit_NeoPixel.h>

struct Motor{
  uint32_t can_id_tx;
  uint32_t can_id_rx;
  int16_t cmd_speed; // mm/s
  int16_t cmd_steer; // mrad
};

struct MotorFeedback{
  int16_t actual_speed; // mm/s
  int16_t actual_steer; // mrad
  bool fresh;
};

Motor motors[4] = {
  {mcp25125_config::CAN_ID_FL_TX, mcp25125_config::CAN_ID_FL_RX, 0, 0},
  {mcp25125_config::CAN_ID_FR_TX, mcp25125_config::CAN_ID_FR_RX, 0, 0},
  {mcp25125_config::CAN_ID_RL_TX, mcp25125_config::CAN_ID_RL_RX, 0, 0},
  {mcp25125_config::CAN_ID_RR_TX, mcp25125_config::CAN_ID_RR_RX, 0, 0}
};

Adafruit_NeoPixel pixel(1, neopixel_config::NEOPIXEL_DATA_PIN, NEO_GRB + NEO_KHZ800);

// Use the Adafruit_MCP2515 constructor that takes explicit SPI pins
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

void send_motor_command(uint32_t can_id, uint8_t seq, uint8_t flags, int16_t speed, int16_t steer) {
  uint8_t data[8];
  data[0] = 0xAA;
  data[1] = 0x55;
  data[2] = seq;
  data[3] = flags;
  data[4] = speed & 0xFF;
  data[5] = (speed >> 8) & 0xFF;
  data[6] = steer & 0xFF;
  data[7] = (steer >> 8) & 0xFF;

  mcp.beginPacket(can_id);
  mcp.write(data, sizeof(data));
  mcp.endPacket();
}

void send_can_packet(uint8_t seq, uint8_t flags) {
  for (int i = 0; i < 4; i++) {
    send_motor_command(motors[i].can_id_tx, seq, flags,
                       motors[i].cmd_speed, motors[i].cmd_steer);
  }
}
// Returns true if a valid packet was received and motors[] updated
bool read_serial_packet(uint8_t& seq, uint8_t& flags) {
  int b0 = Serial.read();
  if (b0 != 0xAA) return false;

  while (Serial.available() == 0) {}
  int b1 = Serial.read();
  if (b1 != 0x55) return false;

  uint8_t rest[19];
  if (Serial.readBytes((char*)rest, 19) != 19) return false;

  uint8_t rx_crc = rest[18];
  uint8_t calc   = crc8_atm(rest, 18);
  if (calc != rx_crc) {
    Serial.print("CRC BAD rx=");
    Serial.print(rx_crc);
    Serial.print(" calc=");
    Serial.println(calc);
    return false;
  }

  seq   = rest[0];
  flags = rest[1];
  for (int i = 0; i < 4; i++) {
    int j = 2 + i * 4;
    motors[i].cmd_speed = (int16_t)(rest[j]   | (rest[j+1] << 8));
    motors[i].cmd_steer = (int16_t)(rest[j+2] | (rest[j+3] << 8));
  }
  return true;
}

void print_motor_commands() {
  for (int i = 0; i < 4; i++) {
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(": speed=");
    Serial.print(motors[i].cmd_speed);
    Serial.print(" steer=");
    Serial.println(motors[i].cmd_steer);
  }
}

void setup() {

  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("Init CAN...");

  // Make sure the MCP25625 is awake and not held in reset
  pinMode(mcp25125_config::PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(mcp25125_config::PIN_CAN_STANDBY, LOW);   // LOW = normal operation (not standby)

  pinMode(mcp25125_config::PIN_CAN_RESET, OUTPUT);
  digitalWrite(mcp25125_config::PIN_CAN_RESET, HIGH);    // HIGH = not in reset (reset is active-low)

  delay(10);

  // Initialize the controller
  if (!mcp.begin(mcp25125_config::CAN_BITRATE)) {
    Serial.println("CAN init failed");
    while (1) delay(10);
  }

  Serial.println("CAN initialized OK");
  pinMode(neopixel_config::NEOPIXEL_POWER_PIN, OUTPUT);
  digitalWrite(neopixel_config::NEOPIXEL_POWER_PIN, HIGH);
  pixel.begin();
  pixel.setBrightness(20);
  pixel.clear();
  pixel.show();

}

void loop() {
  if (Serial.available() > 0) {
    uint8_t seq, flags;
    if (read_serial_packet(seq, flags)) {
      Serial.print("OK seq="); Serial.print(seq);
      Serial.print(" flags="); Serial.println(flags);
      send_can_packet(seq, flags);
      print_motor_commands();
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));
      pixel.show();
      pixel.clear();
    }
  }
}
