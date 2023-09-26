import serial
import time
import string
ser = serial.Serial( port='/dev/ttyACM0', baudrate = 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS,timeout=0)
print("outside loop")
print(ser)
try:
    while True:
        print("inside loop")
        line = ser.readline()
        #print line
        words = line.decode().split(" ")
        print(words)
except KeyboardInterrupt:
    ser.close()
