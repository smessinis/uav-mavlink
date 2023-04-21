from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Set up the connection to the vehicle
vehicle = connect('udp:127.0.0.1:14550')

# Set the vehicle to AUTO mode
vehicle.mode = VehicleMode("AUTO")

# Set the home location to the current location of the vehicle
home_location = vehicle.location.global_frame

# Define a function to trigger the emergency return to home action
def emergency_return_to_home():
    # Set the vehicle to GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")

    # Set the target location to the home location
    target_location = LocationGlobalRelative(home_location.lat, home_location.lon, home_location.alt)

    # Fly the vehicle to the target location
    vehicle.simple_goto(target_location)

    # Wait for the vehicle to reach the target location
    while vehicle.mode.name == "GUIDED":
        time.sleep(1)

    # Set the vehicle back to AUTO mode
    vehicle.mode = VehicleMode("AUTO")

# Example usage: trigger the emergency return to home action
emergency_return_to_home()
