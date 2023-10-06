import random
import os
from datetime import datetime
import matplotlib.pyplot as plt
import csv
import pandas as pd
# Generate a unique filename with a number, date, and time
def generate_unique_filename(folder):
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    file_number = 1
    while True:
        filename = os.path.join(folder, f"output_{current_time}_{file_number}.csv")
        file_number += 1
        return filename

def logging(filename,offsetcorrection,rc1,rc2):
    with open(filename, 'a') as csv:
        # Create lists to store x, y, z, rc_channel1, and rc_channel2 values
        offsetcorrection1 = offsetcorrection[0]
        x_values = offsetcorrection1[0]
        y_values = offsetcorrection1[1]
        z_values = offsetcorrection[2]
        rc1_values = rc1
        rc2_values = rc2

        # Write all variable values to the file
        csv.write(f"x= {x_values} y= {y_values} z= {z_values} ")
        csv.write(f"rc_1= {rc1_values} rc_2= {rc2_values} ")

        # Move to the next line for the next iteration
        csv.write('\n')
        print(f"Data has been saved to '{filename}'")

def graph(folder,filenamecsv):
# Generate a unique filename based on the current date and time
    root_name, extension = os.path.splitext(filenamecsv)
    filename = os.path.join(folder, f'{root_name}.png')    
    csv_file_path = filenamecsv
    filename_x = os.path.join(folder, f'{root_name}_zvsx.png')
    filename_y = os.path.join(folder, f'{root_name}_zvsy.png')
    filename_rc1 = os.path.join(folder, f'{root_name}_zvsrc1.png')
    filename_rc2 = os.path.join(folder, f'{root_name}_zvsrc2.png')
    filename_z = os.path.join(folder, f'{root_name}_z.png')


# Initialize lists to store data for variables x, y, z, rc1, and rc2
    x = []
    y = []
    z = []
    rc1 = []
    rc2 = []    

    # Read the CSV file and extract data
    with open(csv_file_path, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        header = next(csv_reader)  # Skip the header row if it exists
        for row in csv_reader:
            # Split each row by space and extract numeric values
            values = row[0].split()
            x.append(float(values[1]))
            y.append(float(values[3]))
            z.append(float(values[5]))
            rc1.append(float(values[7]))
            rc2.append(float(values[9]))
    plt.figure() 
    plt.plot(x, z, marker='o', linestyle='-', color='b', label='z vs x')
    plt.xlabel('X')
    plt.ylabel('z-value')
    plt.title('z-value vs X')
    plt.grid(True)
    plt.legend()
    plt.savefig(filename_x)

    plt.figure() 
    plt.plot(y, z, marker='o', linestyle='-', color='g', label='z vs y')
    plt.xlabel('y')
    plt.ylabel('z-value')
    plt.title('z-value vs y')
    plt.grid(True)
    plt.legend()
    plt.savefig(filename_y)

    plt.figure() 
    plt.plot(rc1, z, marker='o', linestyle='-', color='r', label='z vs rc1')
    plt.xlabel('rc1')
    plt.ylabel('z-value')
    plt.title('z-value vs rc1')
    plt.grid(True)
    plt.legend()
    plt.savefig(filename_rc1)

    plt.figure() 
    plt.plot(rc2, z, marker='o', linestyle='-', color='purple', label='z vs rc2')
    plt.xlabel('rc2')
    plt.ylabel('z-value')
    plt.title('z-value vs rc2')
    plt.grid(True)
    plt.legend()
    plt.savefig(filename_rc2)

    plt.figure()
    plt.plot(z, marker='o', linestyle='-', color='b', label='z')
    plt.xlabel('Data Points')
    plt.ylabel('z-value')
    plt.title('Scatter Plot of Variable Z')
    plt.grid(True)
    plt.legend()
    plt.savefig(filename_z)

# Print a confirmation message
    #print(f'The plot for z has been saved as "{filename}"')
    print(f'The plot for z vs x has been saved as "{filename_x}"')
    print(f'The plot for z vs y has been saved as "{filename_y}"')
    print(f'The plot for z vs rc1 has been saved as "{filename_rc1}"')
    print(f'The plot for z vs rc2 has been saved as "{filename_rc2}"')
    print(f'The plot for z  has been saved as "{filename_z}"')
