# This code uses the pymavlink library to establish a connection with both drones and send MAVLink commands to control their movement. 
# The while loop runs continuously, getting the current position of the first drone and sending a set_position_target_global_int command 
# to the second drone to make it follow the first drone. The time.sleep(0.1) statement is used to wait for a short time between getting the 
# position of the first drone and sending the command to the second drone, to avoid overwhelming the connection.
# Note that this code assumes that both drones are running the ArduPilot firmware and are connected to your computer via UDP at the 
# IP addresses 127.0.0.1:14550 and 127.0.0.1:14551. 


import time
from pymavlink import mavutil
import math

# Set up connection to the first drone
master1 = mavutil.mavlink_connection('udp:127.0.0.1:14550')
# Set up connection to the second drone
master2 = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Arm both drones and put them in guided mode
master1.arducopter_arm()
master1.set_mode('GUIDED', 4) # 4 stands for guided mode
master2.arducopter_arm()
master2.set_mode('GUIDED', 4)

# Define a function to calculate the distance between two GPS coordinates
def distance(lat1, lon1, alt1, lat2, lon2, alt2):
    R = 6371000 # Radius of the earth in meters
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c + abs(alt2 - alt1)
    return d

# Start following loop
while True:
    # Get current position of the first drone
    msg = master1.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
    lat1 = msg.lat / 1e7
    lon1 = msg.lon / 1e7
    alt1 = msg.alt / 1e3

    # Get current position of the second drone
    msg = master2.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
    lat2 = msg.lat / 1e7
    lon2 = msg.lon / 1e7
    alt2 = msg.alt / 1e3

    # Calculate the distance between the two drones
    dist = distance(lat1, lon1, alt1, lat2, lon2, alt2)

    # If the distance is less than 2 meters, move the second drone to a safe position
    if dist < 2:
        # Move the second drone to a position that is 5 meters away from the first drone
        lat = lat1 - 5 * (lat2 - lat1) / dist
        lon = lon1 - 5 * (lon2 - lon1) / dist
        alt = alt2

        # Send position target to the second drone
        master2.mav.set_position_target_global_int(
            int(lat * 1e7),   # Latitude (degrees * 1e7)
            int(lon * 1e7),   # Longitude (degrees * 1e7)
            int(alt * 1e3),   # Altitude (meters * 1e3)
            0,                # X velocity (m/s)
            0,                # Y velocity (m/s)
            0,                # Z velocity (m/s)
            0,                # X acceleration (m/s^2)
            0,                # Y acceleration (m/s^2)
            0,                # Z acceleration (m/s^2)
            0,            # Yaw
            0             # Yaw rate
    )

    # Wait for a short time before getting the position of the first drone again
    time.sleep(0.1)
