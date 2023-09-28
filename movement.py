from pymavlink import mavutil

def movement(pidx, pidy, the_connection):
# Start a connection listening to a UDP por

    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                         the_connection.target_component, 9, int(0b010111000000), pidx, pidy,0 , 0, 0, 0, 0, 0, 0, 0, 0))
def guidedLand(the_connection, DRate):
      the_connection.mav.set_position_target_local_ned_send(
                0,  # Timestamp (0 to use current time)
                the_connection.target_system,  # Target system ID
                the_connection.target_component,  # Target component ID
                9,  # Coordinate frame
                0b0000111111000000,  # Type mask - enable only velocity components
                0, 0, 0,  # Position setpoint (not used)
                0, 0, DRate,  # Velocity setpoint in NED (North, East, Down) frame (-1.0 m/s descent)
                0, 0, 0,  # Acceleration setpoint (not used)
                0, 0, 0   # Attitude setpoint (not used)
            )
