import serial
import time
import keyboard


bluetooth_port = 'COM5'  
baud_rate = 115200
repeat_delay = 0.05 


ser = serial.Serial(bluetooth_port, baud_rate, timeout=1)
print("Połączenie z ESP32 na porcie:", bluetooth_port)

def send_command(command):
    ser.write(command.encode())
    print("Wysłano:", command)

try:
    caps_active = False  

    while True:
        # Sprawdzenie stanu Caps Locka
        if keyboard.is_pressed('caps lock'):
            caps_active = not caps_active
            time.sleep(0.3)  



        if keyboard.is_pressed('w') and keyboard.is_pressed('d'):
            ser.write(bytes([0xE2]))
            print("w+d  : 0xE2")
        elif keyboard.is_pressed('w') and keyboard.is_pressed('a'):
            ser.write(bytes([0xE1]))
            print("w+a  : 0xE1")
        elif keyboard.is_pressed('s') and keyboard.is_pressed('d'):
            ser.write(bytes([0xE4]))
            print("s+d  : 0xE4")
        elif keyboard.is_pressed('s') and keyboard.is_pressed('a'):
            ser.write(bytes([0xE3]))
            print("s+a  : 0xE3")
        elif keyboard.is_pressed('w'):
            send_command('w')
        elif keyboard.is_pressed('s'):
            send_command('s')
        elif keyboard.is_pressed('a'):
            send_command('a')
        elif keyboard.is_pressed('d'):
            send_command('d')

       
        elif keyboard.is_pressed('x'):
            send_command('x')
        elif keyboard.is_pressed('`'):#speed change
            send_command('`')
            time.sleep(0.5)



        
        elif keyboard.is_pressed('p'):
            send_command('p')
            time.sleep(0.5)
        elif keyboard.is_pressed('r'):
            send_command('r')
            time.sleep(0.5)
        elif keyboard.is_pressed('h'): #hard reset
            send_command('h')
            time.sleep(0.5)

        time.sleep(repeat_delay)

except KeyboardInterrupt:
    print("\nZakończono działanie skryptu.")
finally:
    ser.close()