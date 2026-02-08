#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

const int LED_PIN = 2;  // Wbudowana dioda LED na ESP32

const int RX_PIN = 16;
const int TX_PIN = 17;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT_Dual");
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(LED_PIN, OUTPUT);

  initializeRoomba();

  Serial.println("System gotowy, czekam na komendy...");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    handleCommand(command);
  }

  if (SerialBT.available()) {
    char command = SerialBT.read();
    handleCommand(command);
  }

  // Odczyt danych z Serial2 (RX na GPIO21)
  if (Serial2.available()) {
    char received = Serial2.read();
    Serial.print("Odebrano na Serial2: ");
    Serial.println(received,DEC);
  }
}

void initializeRoomba() {
  Serial2.write(128);  // Start (opcode 128)
  delay(1000);
  Serial2.write(132);  // Full mode
  Serial.println("Roomba zainicjalizowana i gotowa do pracy");
  blinkLED();
}







void resetRoomba() {
  Serial.println("Resetowanie Roomby...");
  initializeRoomba();
}

void handleCommand(char command) {
  switch (command) {
    case 'w':  driveDirect(200, 200); break;
    case 's':  driveDirect(-200, -200); break;
    case 'd':  driveDirect(-100, 100); break;
    case 'a':  driveDirect(100, -100); break;
    case 'x':  driveDirect(0, 0); break;
    case 'p':  Serial2.write(173); Serial.println("Roomba wyłączona"); blinkLED(); break;
    case 'r':  resetRoomba(); break;
    case 'h':  hardResetRoomba(); break; // Twardy reset

    default:   Serial.println("Nieznana komenda");
  }
  blinkLED();
}

void driveDirect(int rightVelocity, int leftVelocity) {
  byte command[5] = {145, 
                     (byte)(rightVelocity >> 8), (byte)(rightVelocity & 0xFF),
                     (byte)(leftVelocity >> 8), (byte)(leftVelocity & 0xFF)};
  Serial2.write(command, 5); // Nadajemy z Serial2 (TX 22)
  Serial.println("Polecenie jazdy wysłane");
}
void hardResetRoomba() {
  Serial2.write(7);  // Reset Opcode
  Serial.println("Wykonano twardy reset Roomby");
  blinkLED();
}
void blinkLED() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
}