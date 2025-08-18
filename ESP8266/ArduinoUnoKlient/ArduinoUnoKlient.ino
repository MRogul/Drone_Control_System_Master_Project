#include <SoftwareSerial.h>

SoftwareSerial espSerial(2, 3); // RX, TX (2=RX, 3=TX, podłącz do ESP)

uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  uint8_t poly = 0x1D;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x80) ? ((crc << 1) ^ poly) : (crc << 1);
    }
  }
  return crc;
}

void setup() {
  Serial.begin(115200);
  espSerial.begin(19200);
}

void loop() {
  static uint8_t buffer[32];
  static uint8_t idx = 0;

  while (espSerial.available()) {
    uint8_t b = espSerial.read();

    if (idx == 0 && b != 0xAA) {
      // czekamy na bajt startu
      continue;
    }

    buffer[idx++] = b;

    if (idx >= 2) {
      uint8_t expectedLen = buffer[1];
      if (idx == expectedLen + 3) { // start + len + payload + crc
        uint8_t crc = crc8(buffer + 1, 1 + expectedLen);
        if (crc == buffer[idx - 1]) {
          float kp, ki, kd, tau;
          memcpy(&kp, buffer + 2, 4);
          memcpy(&ki, buffer + 6, 4);
          memcpy(&kd, buffer + 10, 4);
          memcpy(&tau, buffer + 14, 4);

          Serial.print("KP: "); Serial.print(kp);
          Serial.print(" | KI: "); Serial.print(ki);
          Serial.print(" | KD: "); Serial.print(kd);
          Serial.print(" | TAU: "); Serial.println(tau);
        } else {
          Serial.println("CRC ERROR!");
        }
        idx = 0; // resetujemy ramkę
      }
    }
  }
}
