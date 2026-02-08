#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

const int RX_PIN = 16;
const int TX_PIN = 17;


void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT_Dual");
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop() {
  // Serial2.write(128);
  // Serial2.write(132);
  while (Serial2.available()) {
    uint8_t c = Serial2.read();
    Serial2.write(c);



    SerialBT.print("Przesłano: ");
    SerialBT.print(c);
    // SerialBT.print(" (0x");
    // SerialBT.print((uint8_t)c, HEX);
    // SerialBT.println(")");
  }

}
