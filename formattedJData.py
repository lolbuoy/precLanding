import jevoisData
import time

def offsetData():
    while True:
        # Generate a random message
        message = jevoisData.fakeData()
        print("Generated Message:", message)

        # Split the message by spaces to extract values
        values = message.split()
        
        if len(values) >= 7:
            x = int(values[3])
            y = int(values[4])
            XYOffset = [x, y]
            return XYOffset
        else:
            print("Invalid message format.")

        # Sleep for a while before generating the next message
        time.sleep(0.25)


