import time
import numpy as np
from collections import deque
from pymavlink import mavutil
import posError

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0

    def update(self, setpoint_x, setpoint_y, current_x, current_y):
        error_x = setpoint_x - current_x
        error_y = setpoint_y - current_y

        self.integral_x += error_x
        self.integral_y += error_y

        derivative_x = error_x - self.prev_error_x
        derivative_y = error_y - self.prev_error_y

        control_x = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
        control_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y

        self.prev_error_x = error_x
        self.prev_error_y = error_y

        return control_x, control_y

def auto_tune_pid(controller, process_function):
    kp = 0.0
    ki = 0.0
    kd = 0.0
    last_error_x = None
    last_error_y = None
    oscillation_started = False

    while True:
        uav_position = process_function()
        setpoint_x = desired_position[0]
        setpoint_y = desired_position[1]

        output_x, output_y = controller.update(setpoint_x, setpoint_y, uav_position[0], uav_position[1])
        time.sleep(0.01)  # Simulate a control loop cycle

        if last_error_x is not None:
            if (output_x * last_output_x < 0) or (output_y * last_output_y < 0):
                if oscillation_started:
                    period_x = time.time() - oscillation_start_time
                    kp = 0.6 * critical_gain
                    ki = 2 * kp / (period_x * 3.14159)
                    kd = kp * (0.125 * period_x)
                    break
                else:
                    oscillation_started = True
                    oscillation_start_time = time.time()
            else:
                oscillation_started = False

        last_error_x = output_x - setpoint_x
        last_error_y = output_y - setpoint_y
        last_output_x = output_x
        last_output_y = output_y

    return kp, ki, kd

if __name__ == "__main__":
    desired_position = [0, 0, 0]
    rollingWSize = 10
    position_errors = deque(maxlen=rollingWSize)

    # PID controller gains (adjust as needed)
    kp = 0.5  # Proportional gain
    ki = 0.5  # Integral gain
    kd = 0.5  # Derivative gain

    controller = PIDController(kp, ki, kd)

    def simulated_process():
        uav_position = posError.posError()  # Replace with your process function
        return uav_position

    critical_gain = 1.0  # Adjust as needed
    kp, ki, kd = auto_tune_pid(controller, simulated_process)
    print(f"Optimal PID gains: Kp = {kp}, Ki = {ki}, Kd = {kd}")
