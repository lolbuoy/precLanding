import parserial as parse
import time

def offsetData():
    while True:
        # Generate a random message
        message = parse.read_serial_data()
        print("Generated Message:", message)

        # Split the message by spaces to extract values
        values = message.split()
        
        if len(values) >= 7:
            x = float(values[2])
            y = float(values[3])
            XYOffset = [(x/1000), (y/1000)]
            return XYOffset
        else:
            print("Invalid message format.")

        # Sleep for a while before generating the next message
        time.sleep(0.25)


