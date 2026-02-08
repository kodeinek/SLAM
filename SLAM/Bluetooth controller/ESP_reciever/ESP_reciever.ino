#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

const int LED_PIN = 2; // LED
unsigned long lastCommandTime = 0;
bool isMoving = false;

int goingSpeed=300;
int turningSpeed=150;
int speedSetting=0;


void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Roomba"); 
  Serial2.begin(115200, SERIAL_8N1, 3, 1); // Rx,Tx

  pinMode(LED_PIN, OUTPUT);

  initializeRoomba();

  Serial.println("System gotowy, czekam na komendy...");
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
  goingSpeed=300;
  turningSpeed=150;
  speedSetting=0;
  initializeRoomba();
}

void handleCommand(char command) {
  isMoving = true; 
  lastCommandTime = millis();
  switch (command) {
    // direct
    case 'w':  driveDirect(goingSpeed, goingSpeed); break;
    case 's':  driveDirect(-goingSpeed, -goingSpeed); break;
    case 'd':  driveDirect(-turningSpeed, turningSpeed); break;
    case 'a':  driveDirect(turningSpeed, -turningSpeed); break;
    case 'x':  driveDirect(0, 0); isMoving = false; break;

    // diagonal
    case 0xE1: driveDirect(goingSpeed, turningSpeed); break;
    case 0xE2: driveDirect(turningSpeed, goingSpeed); break;
    case 0xE3: driveDirect(-goingSpeed, -turningSpeed); break;
    case 0xE4: driveDirect(-turningSpeed, -goingSpeed); break;

    case 'p':  Serial2.write(173); Serial.println("Roomba wyłączona"); blinkLED(); break;
    case 'r':  resetRoomba(); break;
    case 'h':  Serial2.write(7); Serial.println("Wykonano twardy reset Roomby"); blinkLED(); break;
    case '`':  switch_speed();
    default:   Serial.println("Nieznana komenda");

  }
  blinkLED();
}

void driveDirect(int rightVelocity, int leftVelocity) {
  byte command[5] = {145, 
                     (byte)(rightVelocity >> 8), (byte)(rightVelocity & 0xFF),
                     (byte)(leftVelocity >> 8), (byte)(leftVelocity & 0xFF)};
  Serial2.write(command, 5);
  Serial.println("Polecenie jazdy wysłane (driveDirect)");
}

void drive(int velocity, int radius) {
  byte command[5] = {137, 
                     (byte)(velocity >> 8), (byte)(velocity & 0xFF),
                     (byte)(radius >> 8), (byte)(radius & 0xFF)};
  Serial2.write(command, 5);
  Serial.println("Polecenie jazdy wysłane (drive)");
}

void blinkLED() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
}

void clearSerialBuffer() {
    unsigned long startTime = millis();
    while (SerialBT.available()) {
        SerialBT.read();
        if (millis() - startTime > 10) { 
            break;
        }
    }
}

void switch_speed(){
  speedSetting+=1;
  switch(speedSetting){
    case(0): goingSpeed=300; turningSpeed=150; break;
    case(1): goingSpeed=400; turningSpeed=200; break;    
    case(2): goingSpeed=500; turningSpeed=250; break;
    default: speedSetting=0; goingSpeed=300; turningSpeed=150; break;
  }
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    handleCommand(command);
  }

  if (SerialBT.available()) {
    char command = SerialBT.read();
    handleCommand(command);

    clearSerialBuffer();
    
  }

  if (Serial2.available()) {
    char received = Serial2.read();
    SerialBT.print("Odebrano na Serial2: ");

    SerialBT.println(received, HEX);
  }

  // stop if no new comands
  if (millis() - lastCommandTime > 150 && isMoving) {
    driveDirect(0, 0);
    isMoving = false; 
  }
}


