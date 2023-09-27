from pymavlink import mavutil

def movement(pidx, pidy, the_connection):
# Start a connection listening to a UDP por

    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                         the_connection.target_component, 9, int(0b010111111000), pidx, pidy,0 , 0, 0, 0, 0, 0, 0, 0, 0))

