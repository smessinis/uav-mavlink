import time
from pymavlink import mavutil

# Set up connection to the first drone
master1 = mavutil.mavlink_connection('udp:127.0.0.1:14550')
# Set up connection to the second drone
master2 = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Arm both drones and put them in guided mode
master1.arducopter_arm()
master1.set_mode('GUIDED', 4) # 4 stands for guided mode
master2.arducopter_arm()
master2.set_mode('GUIDED', 4)

# Start following loop
while True:
    # Get current position of the first drone
    msg = master1.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.alt / 1e3

    # Send set position target command to the second drone
    master2.mav.set_position_target_global_int_send(
        0,            # Timestamp
        1,            # Target system
        1,            # Target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame
        0b0000111111000111,  # Type mask
        int(lat * 1e7),  # Latitude
        int(lon * 1e7),  # Longitude
        int(alt * 1e3),  # Altitude
        0,            # X velocity
        0,            # Y velocity
        0,            # Z velocity
        0,            # X acceleration
        0,            # Y acceleration
        0,            # Z acceleration
        0,            # Yaw
        0             # Yaw rate
    )

    # Wait for a short time before getting the position of the first drone again
    time.sleep(0.1)
