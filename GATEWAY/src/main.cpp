#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// ============================================================
// BHOOMI KAVACH - ESP32 #2 LORA GATEWAY
// SX1278: VCC=3V3, GND=GND, SCK=18, MISO=19, MOSI=23,
// CS=5, RESET=14, DIO0=26.
// USB serial -> laptop -> bridge/serial_bridge.py -> FastAPI.
// ============================================================

constexpr uint8_t LORA_CS = 5;
constexpr uint8_t LORA_RST = 14;
constexpr uint8_t LORA_DIO0 = 26;
constexpr long LORA_FREQUENCY = 433E6;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("BHOOMI_KAVACH_GATEWAY_BOOT");

  SPI.begin(18, 19, 23, 5);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LORA_INIT_FAIL");
    while (true) delay(1000);
  }

  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.enableCrc();

  Serial.println("LORA_GATEWAY_READY");
  Serial.println("EXPECTED_PACKET=BK01,seq,tilt_x,tilt_y,tilt_mag,disp,velocity,vibration,temp,soil,rain");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  String payload;
  while (LoRa.available()) payload += (char)LoRa.read();
  payload.trim();
  if (payload.length() == 0) return;

  // The Python bridge recognizes BK01 lines and converts them to JSON.
  Serial.print("LORA,");
  Serial.print(LoRa.packetRssi());
  Serial.print(',');
  Serial.print(LoRa.packetSnr(), 2);
  Serial.print(',');
  Serial.println(payload);
}
