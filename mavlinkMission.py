from pymavlink import mavutil

# Define connection parameters
connection_string = '/dev/ttyUSB0'  # Replace with the correct serial port or UDP address
baudrate = 57600  # Replace with the correct baudrate for the serial connection (if applicable)

# Create MAVLink connection
master = mavutil.mavlink_connection(connection_string, baud=baudrate)

# Wait for the connection to be established
master.wait_heartbeat()

# Define mission waypoints
waypoints = [
    (47.398039859999997, 8.5455725400000002, 10, 0),
    (47.398036222362471, 8.5450146439425509, 10, 0),
]

# Send mission count message
msg = master.mav.mission_count_encode(0, 0, len(waypoints))
master.mav.send(msg)

# Send individual mission item messages
for i, (lat, lon, alt, seq) in enumerate(waypoints):
    msg = master.mav.mission_item_encode(
        0, 0, i, 0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
        lat, lon, alt
    )
    master.mav.send(msg)

# Send mission start message
msg = master.mav.mission_set_current_encode(0, 0, 0)
master.mav.send(msg)

# Send mission acknowledgement message
msg = master.mav.command_ack_encode(
    mavutil.mavlink.MAV_CMD_MISSION_START, mavutil.mavlink.MAV_RESULT_ACCEPTED
)
master.mav.send(msg)
