from dronekit import connect, VehicleMode

# Connect to the vehicle
connection_string = '/dev/ttyUSB0'  # Replace with the correct serial port or UDP address
vehicle = connect(connection_string, baud=57600)

# Wait for the vehicle to be ready
vehicle.wait_ready('autopilot_version')

# Get the current value of the parameter
thr_min = vehicle.parameters['THR_MIN']
print(f"Current THR_MIN value: {thr_min}")

# Set a new value for the parameter
new_thr_min = 50  # Replace with the new value for THR_MIN
vehicle.parameters['THR_MIN'] = new_thr_min
vehicle.flush()

# Verify that the new value was set
thr_min = vehicle.parameters['THR_MIN']
print(f"New THR_MIN value: {thr_min}")

# Close the connection
vehicle.close()
