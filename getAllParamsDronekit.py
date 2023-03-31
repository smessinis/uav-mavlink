from dronekit import connect, VehicleMode

# Connect to the vehicle
connection_string = '/dev/ttyUSB0'  # Replace with the correct serial port or UDP address
vehicle = connect(connection_string, baud=57600)

# Wait for the vehicle to be ready
vehicle.wait_ready('autopilot_version')

# Get all parameters and print them to the console
parameters = vehicle.parameters
for name in sorted(parameters.keys()):
    value = parameters[name]
    print(f"{name}: {value}")

# Close the connection
vehicle.close()
