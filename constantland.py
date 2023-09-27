from pymavlink import mavutil

def movement(descentrate, the_connection):
# Start a connection listening to a UDP por

    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                         the_connection.target_component, 9, int(0b010111000000),0,0,0 , 0, 0, 0, descentrate, 0, 0, 0, 0))
