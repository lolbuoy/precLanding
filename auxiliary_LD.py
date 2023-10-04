#import moveBool
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import numpy as np
#import formattedJData
from collections import deque
import serial
#import logFiles

#import planeoverride_v2_1

commandLength = 0.27 # in seconds, how long should the RC Override command last for, change acording to airframe
#min,max values for roll and pitch, keep it symmetrical from 1500(neutral)
minPWM = 1350 
maxPWM = 1650 
# Initialize desired position and orientation
desired_position = [0, 0, 0]
desired_orientation = np.zeros(3)

# Tolerance threshold for small variations (adjust as needed)
tolerance_threshold = 0.005
rollingWSize = 10
position_gain = 1.0
orientation_gain = 1.0
rolling_average_x, rolling_average_y = 0.0065, 0.0065
position_errors = deque(maxlen=rollingWSize)

# Initialize PID controller gains
global kp,ki,kd
kp = 0.8 # Proportional gain
ki = 0  # Integral gain
kd = 0 # Derivative gain

# Initialize PID controller variables
prev_error_x = 0.0
prev_error_y = 0.0
integral_x = 0.0
integral_y = 0.0
reposComplete = False


# Set your connection string (e.g., "udp:127.0.0.1:14550" for a local UDP connection
 # serial device of JeVois
  

  
def read_serial_data(serdev):
    with serial.Serial(serdev, 115200) as ser:
        while 1:
        # Read a whole line and strip any trailing line ending character:
            time.sleep(0.15)
            line = ser.readline().decode("utf-8").rstrip()
            #print ("received: {}".format(line))
            return line

def offsetData(serdev):
        # Generate a random message
        if serdev is not None:
            message = read_serial_data(serdev)
        else:
            message = fakeData()
        #print("Generated Message:", message)
        if message is not None:
        # Split the message by spaces to extract values
            values = message.split()
            if len(values) >= 7:
                uid = values[1]
                x = int(values[2])
                y = int(values[3])
                z = int(values[4])
                XYOffset = [x, y]
                return XYOffset,True,z,uid
        else:
            return [0,0],False 
        # Sleep for a while before generating the next message
        time.sleep(0.25)
def offsetCorrection(offsetdata,vehicle):
            # Apply min-max scaling to make x and y values zero-centered
            x_min = -999
            x_max = 999
            y_min = -999
            y_max = 999
            offsetvals = offsetdata[0]
            pidx = offsetvals[0]
            pidy = offsetvals[1]
            x_scaled = ((pidx) / (x_max - x_min)) * (2000 - 1000)
            y_scaled = ((pidy) / (y_max - y_min)) * (2000 - 1000)

            # Ensure scaled values are within the valid RC input range (1000 - 2000)
            rc_channel1 = max(minPWM, min(maxPWM, 1500 + int(x_scaled)))  # Channel 1 (Roll)
            rc_channel2 = max(minPWM, min(maxPWM, 1500 + int(y_scaled)))  # Channel 2 (Pitch)
            #desktop_folder = logFiles.os.path.join(logFiles.os.path.expanduser("~"), "Desktop", "Jevois_Log")

            # Create the "log" folder if it doesn't exist
            #logFiles.os.makedirs(desktop_folder, exist_ok=True)
            #log = logFiles.generate_unique_filename(desktop_folder)
            #logFiles.loggingData(filename=log,uid=offsetdata[3],x=pidx,y=pidy,z=offsetdata[2],rc1=rc_channel1,rc2=rc_channel2)

            # Send RC override messages
            vehicle.channels.overrides[1] = rc_channel1
            vehicle.channels.overrides[2] = rc_channel2
            time.sleep(commandLength)
            resetVehicle(vehicle)
            print("RC Overrides - Channel 1:", rc_channel1, "Channel 2:", rc_channel2)


        # Sleep for a while before generating the next message
            time.sleep(0.15)
def resetQuit(vehicle):
        vehicle.channels.overrides[1] = 1500
        vehicle.channels.overrides[2] = 1500
        vehicle.close()
        quit()

def resetVehicle(vehicle):
       vehicle.channels.overrides[1] = 1500
       vehicle.channels.overrides[2] = 1500
def offsetCorrectionPrint(pidx,pidy,vehicle):
            # Apply min-max scaling to make x and y values zero-centered
            x_min = -999
            x_max = 999
            y_min = -999
            y_max = 999

            x_scaled = (pidx - x_min) / (x_max - x_min) * (2000 - 1000)
            y_scaled = (pidy - y_min) / (y_max - y_min) * (2000 - 1000)

            # Ensure scaled values are within the valid RC input range (1000 - 2000)
            rc_channel1 = max(1350, min(1650, 1500 + int(x_scaled)))  # Channel 1 (Roll)
            rc_channel2 = max(1350, min(1650, 1500 + int(y_scaled)))  # Channel 2 (Pitch)
            print("RC Overrides - Channel 1:", rc_channel1, "Channel 2:", rc_channel2)
        # Sleep for a while before generating the next message
            time.sleep(0.175)
def pid_controller(kp,ki,kd,current_position_x, current_position_y,master):
    global prev_error_x, prev_error_y, integral_x, integral_y

    # Calculate position errors
    error_x = desired_position[0] - current_position_x
    error_y = desired_position[1] - current_position_y

    # Update integral terms
    integral_x += error_x
    integral_y += error_y

    # Calculate control commands
    control_x = kp * error_x + ki * integral_x + kd * (error_x - prev_error_x)
    control_y = kp * error_y + ki * integral_y + kd * (error_y - prev_error_y)

    # Update previous error values
    prev_error_x = error_x
    prev_error_y = error_y

    return control_x, control_y

def control_drone(master):
        # Parse ArUco marker data
    while 1:
        uav_position = offsetData()
        if uav_position is not None:
            print(uav_position)
            position_error_x = desired_position[0] - uav_position[0]
            position_error_y = desired_position[1] - uav_position[1]
            position_errors.append((position_error_x, position_error_y))

            # Calculate the rolling average of position errors
            rolling_average_x = sum(error[0] for error in position_errors) / len(position_errors)
            rolling_average_y = sum(error[1] for error in position_errors) / len(position_errors)

            # Check if translation and rotation are close to zero
            if abs(rolling_average_x) < tolerance_threshold and abs(rolling_average_y) < tolerance_threshold and reposComplete:
                print("Rolling average below threshold. Exiting loop.")
                return True  # Return True when the threshold is met
            else:
                pid_x, pid_y = pid_controller(position_error_x, position_error_y,master)
                offsetCorrection(pid_x,pid_y,master)
                reposComplete = True
                return False
        else:
             resetVehicle(master)
def send_roll_pitch_overrides(vehicle, overrides):
    '''
    Receive list of roll, pitch PWM
    List of PWM in the format: overrides = [roll_pwm, pitch_pwm]
    Send as RC_OVERRIDE to aircraft
    '''

    for channel_num, value in enumerate(overrides, start=1):
        vehicle.channels.overrides[channel_num] = value
        print(f"Channel {channel_num}: {value}")

#use run_repositioning in the main loop
def run_repositioning(vehicle, complete,serdev):
    offset = offsetData(serdev)
    out = offset[0]
    is_avail=offset[1]
    print(offset)
    outx,outy = pid_controller(kp,ki,kd,out[0],out[1],vehicle)
    pidOut = ([outx,outy],is_avail)
    offsetCorrection(pidOut,vehicle)#use pidOut if pid controller has to be tested #use offsetCorrectionPrint for just seeing how the values are behaving
    time.sleep(0.33)



import random

def fakeData():
    # Generate random integers for x, y, and z
    x = random.randint(-10000, 10000)
    y = random.randint(-10000, 10000)
    z = random.randint(-10000, 10000)

# Create the message string
    message = f"N3 U0 {x} {y} {z} 100 100 1"
    return message
