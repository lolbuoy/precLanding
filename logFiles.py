import random
import os
from datetime import datetime

# Generate a unique filename with a number, date, and time
def generate_unique_filename(folder):
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    while True:
        filename = os.path.join(folder, f"output_{current_time}.txt")
        if not os.path.exists(filename):
            return filename
    

def loggingData(filename,uid,x,y,z,rc1,rc2):
    with open(filename, 'w') as file:
        # Create an empty list to keep track of used values of 'p'
        used_p_values = []

        # Generate random data and write it to the file
          # Change this number to the desired number of lines
          # Skip this iteration and generate a new 'p'

        #UID Create the message string
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        message = f"{uid}   {current_time}:> x= {x} y= {y} z= {z} rc1= {rc1} rc2 = {rc2}"

        # Write the message to the file
        file.write(f"{message}\n")

# Specify the output folder on the desktop
