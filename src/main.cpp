// ********************

// * a3tsm_rp2040_interface *

// ********************

// Data packet format (21 bytes total):
// Byte 0:  Start-of-frame byte 1 (0xAA)
// Byte 1:  Start-of-frame byte 2 (0x55)
// Byte 2:  Sequence number (uint8, 0-255, wraps around)
// Byte 3:  Flags (uint8 bit field)
// Byte 4-5:  frontLeftSpeed   (int16, little-endian, mm/s)
// Byte 6-7:  frontLeftSteer   (int16, little-endian, mrad)
// Byte 8-9:  frontRightSpeed  (int16, little-endian, mm/s)
// Byte 10-11: frontRightSteer (int16, little-endian, mrad)
// Byte 12-13: rearLeftSpeed   (int16, little-endian, mm/s)
// Byte 14-15: rearLeftSteer   (int16, little-endian, mrad)
// Byte 16-17: rearRightSpeed  (int16, little-endian, mm/s)
// Byte 18-19: rearRightSteer  (int16, little-endian, mrad)
// Byte 20: CRC-8 checksum computed over bytes 2-19

#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <SPI.h>
#include "mcp25125_config.h" 
#include "neopixel_config.h"
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixel(1, neopixel_config::NEOPIXEL_DATA_PIN, NEO_GRB + NEO_KHZ800);

// Use the Adafruit_MCP2515 constructor that takes explicit SPI pins
Adafruit_MCP2515 mcp(mcp25125_config::PIN_CAN_CS,
                     mcp25125_config::PIN_CAN_MOSI, 
                     mcp25125_config::PIN_CAN_MISO, 
                     mcp25125_config::PIN_CAN_SCK);

// CAN configuration
static constexpr uint32_t CAN_BITRATE = 500000; // match your bus
static constexpr uint32_t CAN_ID_FL = 0x120; // Front Left  (speed + steer)
static constexpr uint32_t CAN_ID_FR = 0x121; // Front Right (speed + steer)
static constexpr uint32_t CAN_ID_RL = 0x122; // Rear Left   (speed + steer)
static constexpr uint32_t CAN_ID_RR = 0x123; // Rear Right  (speed + steer)


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

void send_can_packet(uint8_t seq,
                     uint8_t flags,
                     int16_t frontLeftSpeed, 
                     int16_t frontLeftSteer, 
                     int16_t frontRightSpeed, 
                     int16_t frontRightSteer, 
                     int16_t rearLeftSpeed, 
                     int16_t rearLeftSteer, 
                     int16_t rearRightSpeed, 
                     int16_t rearRightSteer 
                     ) {

// Front Left
send_motor_command(CAN_ID_FL, seq, flags, frontLeftSpeed, frontLeftSteer);
// Front Right
send_motor_command(CAN_ID_FR, seq, flags, frontRightSpeed, frontRightSteer);
// Rear Left
send_motor_command(CAN_ID_RL, seq, flags, rearLeftSpeed, rearLeftSteer);
// Rear Right
send_motor_command(CAN_ID_RR, seq, flags, rearRightSpeed, rearRightSteer);
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

  // Find SOF AA 55
  while (Serial.available() > 0) {

    int b0 = Serial.read();
    if (b0 != 0xAA) continue;

    while (Serial.available() == 0) {}
    int b1 = Serial.read();
    if (b1 != 0x55) continue;

    uint8_t rest[7];
    size_t got = Serial.readBytes((char*)rest, 7);
    if (got != 7) return;

    uint8_t* payload = rest;
    uint8_t rx_crc = rest[6];

    uint8_t calc = crc8_atm(payload, 6);

    if (calc != rx_crc) {
      Serial.print("CRC BAD rx=");
      Serial.print(rx_crc);
      Serial.print(" calc=");
      Serial.println(calc);
      return;
    }

    uint8_t seq = payload[0];
    uint8_t flags = payload[1];
    // Front Motor
    int16_t frontLeftSpeed  = (int16_t)(payload[2] | (payload[3] << 8));
    int16_t frontLeftSteer = (int16_t)(payload[4] | (payload[5] << 8));
    int16_t frontRightSpeed  = (int16_t)(payload[6] | (payload[7] << 8));
    int16_t frontRightSteer = (int16_t)(payload[8] | (payload[9] << 8));
    // Rear Motor
    int16_t rearLeftSpeed  = (int16_t)(payload[10] | (payload[11] << 8));
    int16_t rearLeftSteer = (int16_t)(payload[12] | (payload[13] << 8));
    int16_t rearRightSpeed  = (int16_t)(payload[14] | (payload[15] << 8));
    int16_t rearRightSteer = (int16_t)(payload[16] | (payload[17] << 8));
    // Convert to physical units for debug printing and CAN transmission
    float frontLeftSpeed_f = frontLeftSpeed / 1000.0f;
    float frontLeftSteer_f = frontLeftSteer / 1000.0f;
    float frontRightSpeed_f = frontRightSpeed / 1000.0f;
    float frontRightSteer_f = frontRightSteer / 1000.0f;
    // Rear motor values are also in mm/s and mrad, but we can print them as floats for consistency
    float rearLeftSpeed_f = rearLeftSpeed / 1000.0f;
    float rearLeftSteer_f = rearLeftSteer / 1000.0f;
    float rearRightSpeed_f = rearRightSpeed / 1000.0f;
    float rearRightSteer_f = rearRightSteer / 1000.0f;

    // Debug print
    Serial.print("OK seq=");
    Serial.print(seq);
    Serial.print(" flags=");
    Serial.print(flags);
    Serial.print(" front_left_speed=");
    Serial.print(frontLeftSpeed_f, 3);
    Serial.print(" front_left_steer=");
    Serial.println(frontLeftSteer_f, 3);
    Serial.print(" front_right_speed=");
    Serial.print(frontRightSpeed_f, 3);
    Serial.print(" front_right_steer=");
    Serial.println(frontRightSteer_f, 3);
    Serial.print(" rear_left_speed=");
    Serial.print(rearLeftSpeed_f, 3);
    Serial.print(" rear_left_steer=");
    Serial.println(rearLeftSteer_f, 3);
    Serial.print(" rear_right_speed=");
    Serial.print(rearRightSpeed_f, 3);
    Serial.print(" rear_right_steer=");
    Serial.println(rearRightSteer_f, 3);  

    // Send over CAN
    send_can_packet(seq, flags, frontLeftSpeed_f, 
                                frontLeftSteer_f, 
                                frontRightSpeed_f, 
                                frontRightSteer_f,
                                rearLeftSpeed_f,
                                rearLeftSteer_f,
                                rearRightSpeed_f,
                                rearRightSteer_f 
                                );
  
    pixel.setPixelColor(0, pixel.Color(0, 255, 0)); // green
    pixel.show();
    pixel.clear();
  }
}
