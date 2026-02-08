#roomba.py

import serial
import time
#import pygame

class Roomba:

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate=baudrate, timeout=1)
            print("Połączono z portem szeregowym.\n")
            self._set_defaults()
            self.fullMode()
            print("Roomba zainicjalizowana.\n")
        except Exception as e:
            print(f"Błąd otwarcia portu: {e}\n")
            self.ser = None
            self.lastCommandTime = 0
            self.isMoving = False
            self.goingMax = 300
            self.turningMax = 150
            self.speedSetting = 0

            self.goingSpeed = 0
            self.turningSpeed = 0
            self.targetGoingSpeed = 0
            self.targetTurningSpeed = 0

    def _set_defaults(self):
        self.lastCommandTime = 0
        self.isMoving = False
        self.goingMax = 300
        self.turningMax = 150
        self.speedSetting = 0
        self.goingSpeed = 0
        self.turningSpeed = 0
        self.targetGoingSpeed = 0
        self.targetTurningSpeed = 0

    def _send(self, data):
        if self.ser:
            self.ser.write(bytearray(data))
            
    def fullMode(self):
        self._send([128])
        time.sleep(0.1)
        self._send([132])

    def hardReset(self):
        print("hard Resetowanie Roomby...\n")
        self._send([7])

    def reset(self):
        print("Resetowanie Roomby...\n")
        self._set_defaults()
        self.fullMode()

    def powerOff(self):
        self._send([128])
        time.sleep(0.1)
        self._send([173]) 

    def driveDirect(self, rightVelocity: int, leftVelocity: int):
        if rightVelocity ==0:
            print("stop")
        else:
            print("drive") 
        cmd = [
            145,
            (rightVelocity >> 8) & 0xFF, rightVelocity & 0xFF,
            (leftVelocity >> 8) & 0xFF, leftVelocity & 0xFF
        ]
        self._send(cmd)
        self.isMoving = True
        self.lastCommandTime = time.time()

    def stop(self):
        print("stop")
        self.driveDirect(0, 0)
        self.isMoving = False


    def switch_speed(self):
        self.speedSetting += 1
        if self.speedSetting == 1:
            self.goingMax, self.turningMax = 400, 200
        elif self.speedSetting == 2:
            self.goingMax, self.turningMax = 500, 250
        else:
            self.speedSetting = 0
            self.goingMax, self.turningMax = 300, 150
        print(f"Nowe prędkości: going={self.goingSpeed}, turning={self.turningSpeed}\n")
    


    def accelerate(self, step=10, min_speed_threshold=5):
        if abs(self.goingSpeed - self.targetGoingSpeed) <= step:
            self.goingSpeed = self.targetGoingSpeed
        else:
            if self.goingSpeed < self.targetGoingSpeed:
                self.goingSpeed += step
            else:
                self.goingSpeed -= step

        if abs(self.turningSpeed - self.targetTurningSpeed) <= step:
            self.turningSpeed = self.targetTurningSpeed
        else:
            if self.turningSpeed < self.targetTurningSpeed:
                self.turningSpeed += step
            else:
                self.turningSpeed -= step

        if abs(self.goingSpeed) < min_speed_threshold:
            self.goingSpeed = 0
        if abs(self.turningSpeed) < min_speed_threshold:
            self.turningSpeed = 0

    def handleCommand(self, keys):
        self.targetGoingSpeed = 0
        self.targetTurningSpeed = 0

        if keys[pygame.K_w] and keys[pygame.K_a]:
            self.targetGoingSpeed = self.goingMax
            self.targetTurningSpeed = self.turningMax
        elif keys[pygame.K_w] and keys[pygame.K_d]:
            self.targetGoingSpeed = self.turningMax
            self.targetTurningSpeed = self.goingMax
        elif keys[pygame.K_s] and keys[pygame.K_a]:
            self.targetGoingSpeed = -self.goingMax
            self.targetTurningSpeed = -self.turningMax
        elif keys[pygame.K_s] and keys[pygame.K_d]:
            self.targetGoingSpeed = -self.turningMax
            self.targetTurningSpeed = -self.goingMax
        elif keys[pygame.K_w]:
            self.targetGoingSpeed = self.goingMax
            self.targetTurningSpeed = self.goingMax
        elif keys[pygame.K_s]:
            self.targetGoingSpeed = -self.goingMax
            self.targetTurningSpeed = -self.goingMax
        elif keys[pygame.K_a]:
            self.targetGoingSpeed = self.turningMax
            self.targetTurningSpeed = -self.turningMax
        elif keys[pygame.K_d]:
            self.targetGoingSpeed = -self.turningMax
            self.targetTurningSpeed = self.turningMax
        elif keys[pygame.K_x]:
            self.targetGoingSpeed = 0
            self.targetTurningSpeed = 0
            self.stop()
            time.sleep(1)
        elif keys[pygame.K_p]:
            self.powerOff()
            time.sleep(1)
        elif keys[pygame.K_r]:
            self.reset()
            time.sleep(1)
        elif keys[pygame.K_h]:
            self.hardReset()
            time.sleep(1)
        elif keys[pygame.K_BACKQUOTE]:
            self.switch_speed()
            time.sleep(1)

    def updateMovement(self):
        self.accelerate(step=15) 
        if abs(self.goingSpeed) < 5 and abs(self.turningSpeed) < 5:
            self.goingSpeed = 0
            self.turningSpeed = 0
            self.stop()
        else:
            self.driveDirect(int(self.goingSpeed), int(self.turningSpeed))


    def checkTimeout(self, timeout=0.25):
        """Zatrzymaj Roombe jeśli za długo brak nowych komend"""
        if self.isMoving and (time.time() - self.lastCommandTime > timeout):
            print("Brak nowych komend - zatrzymuję Roombę\n")
            self.stop()

    def close(self):
        if self.ser:
            try:
                pass
            finally:
                self.powerOff()
                self.ser.close()
                print("Port szeregowy zamknięty.\n")


def main():
    pygame.init()
    screen = pygame.display.set_mode((200, 100))  
    r = Roomba()
    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        keys = pygame.key.get_pressed()
        r.handleCommand(keys)
        r.updateMovement()
        r.checkTimeout()
        clock.tick(60) 
    r.close()
    pygame.quit()
    print('fin')
    


if __name__ == "__main__":
    main()