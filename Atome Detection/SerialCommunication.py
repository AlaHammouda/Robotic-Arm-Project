import serial
import time


ser=serial.Serial('COM26',baudrate=9600, timeout=1)
while 1:
    ser.write(str.encode('allon'))
    time.sleep(1)

print ('ebd')