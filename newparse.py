import serial

# Define the serial port (adjust the port name as needed)
serial_port = '/dev/ttyACM0'

try:
    # Open the serial port
    with serial.Serial(serial_port, baudrate=115200, timeout=0) as ser:
        print(f"Connected to {serial_port}")

        while True:
            # Read a line of data from the serial port
            line = ser.readline(8).decode('utf-8').strip()
             # Process the received data as needed
            print(f"Received: {line}")

except serial.SerialException as e:
    print(f"Error: {e}")
except KeyboardInterrupt:
    print("Serial communication stopped.")
