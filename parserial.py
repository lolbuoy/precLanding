serdev = 'COM43' # serial device of JeVois
  
import serial
import time
  
def read_serial_data():
    with serial.Serial(serdev, 115200) as ser:
        while 1:
        # Read a whole line and strip any trailing line ending character:

            line = ser.readline().decode("utf-8").rstrip()
            #print ("received: {}".format(line))
            return line
