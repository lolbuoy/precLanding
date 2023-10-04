from dronekit import connect, VehicleMode
from pymavlink import mavutil
import auxiliary_LE
import threading

connection_string = 'tcp:localhost:5762'  # Change to your connection string
the_connection = connect(connection_string)
print("connected")

global throttle_flag
throttle_flag = False
serdev = None  # Keep it as None for running in SITL
disarmed = False
# Create a flag to track landing completion
def throttle_Disarmed_Or_Not(vehicle,serdev):
    throttle_disarmed = False

    @vehicle.on_message('STATUSTEXT')
    def handle_status_text1(self, name, msg):
        # print('Waiting for throttle disarm')
        if msg.text == "Throttle disarmed":
            print(msg.text)
            nonlocal throttle_disarmed
            throttle_disarmed = True

    while not throttle_disarmed:
        auxiliary.run_repositioning(the_connection,throttle_disarmed,serdev)
    print("Returning control") 
   # auxiliary.resetQuit(the_connection)
    return throttle_disarmed


def land_Complete_Or_Not(vehicle):
    land_comp = False
    throttle_disarmed = False
    @vehicle.on_message('STATUSTEXT')
    def handle_status_text(self, name, msg):
        hasread = auxiliary.offsetData(serdev)
        if msg.text == "Land descend started" and hasread[1]:
             nonlocal land_comp 
             land_comp = True
             the_connection.mode = "QLAND"
    while not land_comp:  #come out of the loop only when both land_comp and throttle_disarmed
        pass

    return land_comp
# Call the land_Complete_Or_Not function to monitor landing completion
isLanding = land_Complete_Or_Not(the_connection)
#disarmed_thread = threading.Thread(target=throttle_Disarmed_Or_Not(the_connection))
# Check if landing is complete and perform additional actions if needed
throttle_Disarmed_Or_Not(vehicle=the_connection,serdev=serdev)
print("Control returned, proceeding to quit") 
auxiliary.resetQuit(the_connection)
# while 1:
#     pass
