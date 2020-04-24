from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import main

import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/ttyTHS1') #127.0.0.1:14550   /dev/ttyTHS1
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)

def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:
    
    
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
	
def condition_yaw(heading, speed, direction, relative=True):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.
    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        speed,          # param 2, yaw speed deg/s
        direction,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
		
def forwardM(vehicle, x): #--X in METERS
	counter = 0 
	a = x/.5
	while counter < a:
		set_velocity_body(vehicle,.5,0,0)
		time.sleep(1)
		print("moving forward")
		counter = counter + 1
	time.sleep(a)
	
def backwardM(vehicle, x):
	counter = 0 
	a = x/.5
	while counter < a:
		set_velocity_body(vehicle,-.5,0,0)
		time.sleep(1)
		print("moving backward")
		counter = counter + 1
	time.sleep(a)

def rightM(vehicle, x):
	counter = 0 
	a = x/.5
	while counter < a:
		set_velocity_body(vehicle,0,.5,0)
		time.sleep(1)
		print("moving right")
		counter = counter + 1
	time.sleep(a)

def leftM(vehicle, x):
	counter = 0 
	a = x/.5
	while counter < a:
		set_velocity_body(vehicle,0,-.5,0)
		time.sleep(1)
		print("moving left")
		counter = counter + 1
	time.sleep(a)

def upwardM(vehicle, x):
	counter = 0 
	a = x/.5
	while counter < a:
		set_velocity_body(vehicle,0,0,-.5)
		time.sleep(1)
		print("moving up")
		counter = counter + 1
	time.sleep(a)

def downwardM(vehicle):
	counter = 0 
	a = x/.5
	while counter < a:
		set_velocity_body(vehicle,0,0,.3)
		time.sleep(1)
		print("moving down")
		counter = counter + 1
	time.sleep(a)

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):
        
  print ("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("LOITER")
  vehicle.armed   = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    time.sleep(1)

  print ("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print (" Altitude: ", vehicle.location.global_relative_frame.alt) 
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)

# Initialize the takeoff sequence to 1.5 m
arm_and_takeoff(1.5)

print("Set default/target airspeed to .5") #-- METERS/S
vehicle.airspeed = .5
print("Take off complete")

# Hover for 3 seconds
time.sleep(3)

print ("Entering GUIDED")
vehicle.mode = VehicleMode("GUIDED")
time.sleep(5)

'''
Scan for first target. X overlay position is needed and then the drone should yaw until the x is centered within tollerance. 
Yaw right 15 degrees and then left 15 degrees. Once x is centered on the x axis then use same method with downwardM/upwardM 
to center on x axis. When X is centered on both axis command drone to move foward until a ceratin distance from object has been reached.
'''
print("Centering First Target")
condition_yaw(heading, speed, direction, relative=True)


print ("Entering Loiter Mode")
vehicle.mode = VehicleMode("LOITER")
time.sleep(3
print ("Returning")
vehicle.mode = VehicleMode("RTL")
time.sleep(1)
#'''
#print("Now let's land")
#vehicle.mode = VehicleMode("LAND")


# Close vehicle object
vehicle.close()
