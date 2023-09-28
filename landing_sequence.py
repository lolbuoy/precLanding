import math
from pymavlink import mavutil
import threading
import time
import moveBool
import movement
connection_string = 'udp:192.168.208.247:690'  # Change to your connection string

# Desired GPS coordinates (latitude and longitude) for reaching the location
desired_latitude =-35.3627141# Replace with your desired latitude
desired_longitude = 149.1651160# Replace with your desired longitude
#desired altitude to switch to guided mode
desiredalt=30
safealt=6.5
descentrate = 1.1

# Connect to the autopilot
the_connection = mavutil.mavlink_connection(connection_string)

def altitude():
   altitude_thread = threading.Thread(target= altitude())
   altitude_thread.start()
def land():
    
  while True:
        gps = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        alt = gps.relative_alt / 1000  # Altitude in meters

        if alt > desiredalt:
            print(f"location reached. Switching to LAND mode...")
            the_connection.mav.command_long_send(
                the_connection.target_system,
                the_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
            #current_mode = LAND_MODE
        elif alt <= desiredalt and alt>safealt :
            print(f"Altitude is below 30 meters ({alt} meters). Switching to GUIDED mode")
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,176, 0, 1, 4, 0, 0, 0, 0, 0)
                        # Send velocity command for descent
            repos_complete = moveBool.control_drone(the_connection,descentrate,alt,safealt)
        elif alt<=safealt or repos_complete:
            print(f"safe altitude reached. Switching back to LAND mode...")
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

while True:
    gps = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = gps.lat / 1e7  # Latitude in degrees
    lon = gps.lon / 1e7  # Longitude in degrees
    alt = gps.relative_alt / 1000  # Altitude in meters
    
    # Get the flight mode
    heartbeat = the_connection.recv_match(type='HEARTBEAT', blocking=True)
    mode = mavutil.mode_string_v10(heartbeat)
    
    print(f"Mode: {mode}, Altitude: {alt} meters, Latitude: {lat}, Longitude: {lon}")

    # Check if the desired GPS coordinates have been reached
    if (
        abs(lat - desired_latitude) <= 0.000005 and
        abs(lon - desired_longitude) <= 0.000005
    ):
        land()  # Switch to LAND mode when the desired location is reached
        break

print("Desired location reached.")

              
def GPS():
    gps = None
    gps = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking = True)
    return gps.lat , gps.lon , gps.hdg
