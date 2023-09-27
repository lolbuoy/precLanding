import serial

def read_serial_data():
    ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0
    )
    #print("Outside loop")
    #print(ser)
    try:
        while True:
            print("Inside loop")
            line = ser.readline()
            #print line
            words = line.decode("utf-8").rstrip()
            print(words)
            return words  # Return the 'words' list
    except KeyboardInterrupt:
        ser.close()
