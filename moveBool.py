from pymavlink import mavutil
import numpy as np
import formattedJData
from collections import deque
import movement
import time

# Initialize desired position and orientation
desired_position = [0, 0, 0]
desired_orientation = np.zeros(3)

# Tolerance threshold for small variations (adjust as needed)
tolerance_threshold = 0.005
rollingWSize = 10
connectionString = 'udpin:localhost:14553'
master = mavutil.mavlink_connection(connectionString)
master.wait_heartbeat()
position_gain = 1.0
orientation_gain = 1.0
rolling_average_x, rolling_average_y = 0.0065, 0.0065
position_errors = deque(maxlen=rollingWSize)

# Initialize PID controller gains
kp = 1  # Proportional gain
ki = 0  # Integral gain
kd = 0  # Derivative gain

# Initialize PID controller variables
prev_error_x = 0.0
prev_error_y = 0.0
integral_x = 0.0
integral_y = 0.0

def pid_controller(current_position_x, current_position_y):
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

def control_drone():
    while True:
        # Parse ArUco marker data
        uav_position = formattedJData.offsetData()
        print(uav_position)
        formattedJData.time.sleep(0.25)
        position_error_x = desired_position[0] - uav_position[0]
        position_error_y = desired_position[1] - uav_position[1]
        position_errors.append((position_error_x, position_error_y))

        # Calculate the rolling average of position errors
        rolling_average_x = sum(error[0] for error in position_errors) / len(position_errors)
        rolling_average_y = sum(error[1] for error in position_errors) / len(position_errors)

        # Check if translation and rotation are close to zero
        if abs(rolling_average_x) < tolerance_threshold and abs(rolling_average_y) < tolerance_threshold:
            print("Rolling average below threshold. Exiting loop.")
            return True  # Return True when the threshold is met
            break
        else:
            pid_x, pid_y = pid_controller(position_error_x, position_error_y)
            movement.movement(pid_x, pid_y, master)
            time.sleep(0.5)

