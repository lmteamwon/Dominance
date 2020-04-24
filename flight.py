from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from threading import Thread
import time
import argparse  
import datetime
import t265_to_mavlink

M_MANUAL = 0
M_AUTONAV = 1
M_AUTOMANEUVER = 2

# Tuples for directions the vehicle can go
# Allows you to quickly set a velocity in 1 parameter
# and generally makes the code more readable
DIR_RIGHT = [0, 1, 0]
DIR_LEFT = [0, -1, 0]
DIR_UP = [0, 0, -1]
DIR_DOWN = [0, 0, 1]
DIR_FRONT = [1, 0, 0]
DIR_BACK = [-1, 0, 0]
DIR_NONE = [0, 0, 0]

ENABLE_REALSENSE = False
realsenseReady = False


class Drone:
	def __init__(self, callback):
		self.vehicle = conn()
		self.stats = None
		self.objectClass  = None
		self.called = False
		self.callback = callback
	def upData(self, boxStat, boxClass):
		self.stats = boxStat
		self.objectClass = boxClass
		self.called = True

def getCurrentTimeStr():
	return str(datetime.datetime.now().time())[0:10]

def print_time(msg):
	print("[%s] %s" % (getCurrentTimeStr(), msg))

# Scales a velocity vector (list) by scale
def scaleVelocity(dir, scale):
	# Make a copy of dir so we do not modify
	# the direction definitions
	newDir = dir.copy()
	for i in range(0, len(newDir)):
		newDir[i] *= scale
	return newDir

def markReady():
	global realsenseReady
	realsenseReady = True
	print_time("*** RealSense T265 reports ready status")

pylonManeuverDone = False

def flightMain(flightData):
	global realsenseReady
	initialAltitude = 1

	if ENABLE_REALSENSE:
		print_time("Enabling VIO camera")
		process = Thread(target = t265_to_mavlink.runVIO, args= (flightData.vehicle,markReady,))
		process.start()
	else:
		print_time("Flying without RealSense T265 enabled.")

	while not flightData.called:
		print_time("Waiting for object detection")
		time.sleep(10)

	print_time("Object detection available. Waiting 10 seconds.")
	time.sleep(10)
	
	while ENABLE_REALSENSE and not realsenseReady:
		print_time("Waiting for VIO camera")
		time.sleep(5)
	
	print_time("*** ARMING DRONE - INSTRUCT OTHERS TO STAND CLEAR")
	print_time("*** ARMING DRONE - INSTRUCT OTHERS TO STAND CLEAR")
	print_time("*** ARMING DRONE - INSTRUCT OTHERS TO STAND CLEAR")

	# Arm in GUIDED mode
	flightData.vehicle.mode = VehicleMode("LOITER")
	flightData.vehicle.armed   = True

	while not flightData.vehicle.armed:
		print_time(" Waiting for drone to be armed")
		time.sleep(0.5)

	print_time("Taking off!")
	flightData.vehicle.simple_takeoff(initialAltitude)

	# Check that vehicle has reached takeoff altitude
	while True:
		print("Altitude: ", flightData.vehicle.location.global_relative_frame.alt) 
		#Break and return from function just below target altitude.        
		if flightData.vehicle.location.global_relative_frame.alt >= initialAltitude * 0.95: 
			print_time("Reached target altitude")
			break
		time.sleep(0.5)

	flightData.vehicle.mode = VehicleMode("GUIDED")
	print_time("Entered guided mode")
	time.sleep(3)

	print_time("Starting main flight loop")	
	shouldLoop = True
	while shouldLoop:
		# Send altitude & mode to the main script for the HUD
		flightData.callback(flightData.vehicle.location.global_relative_frame.alt * 3.3, M_AUTONAV)

		if flightData.stats is None:
			print_time("No object detected")
			set_velocity_body(flightData.vehicle, DIR_NONE)
			time.sleep(0.5)
		else:
			dist = flightData.stats.get("distance", 0)
			print_time("%sft to %s" % (str(round(dist, 1)), flightData.objectClass))
			if (dist > 6):
				if flightData.stats is None:
					print_time("Object WAS detected but now is not")
					set_velocity_body(flightData.vehicle, DIR_NONE)
					time.sleep(0.05)
					continue
				wCenterVal = flightData.stats.get("centerX", 360) / 720
				# How off-center can the target be (in %)
				wCenterTolerance = 0.1
				# Since the ring is a tight space the centering must be more precise
				if flightData.objectClass == "Ring":
					wCenterTolerance = 0.05
				wDistFromCenter = abs(0.5 - wCenterVal)
				speed = 0.2
				# Set the speed based on how off-centered the drone is
				if wDistFromCenter > 0.3:
					speed = 0.26
				elif wDistFromCenter > 0.2:
					speed = 0.22
				else:
					speed = 0.15
				if wCenterVal < 0.5 - wCenterTolerance or wCenterVal > 0.5 + wCenterTolerance:
					# Center to the right
					if (wCenterVal > 0.5 + wCenterTolerance):
						print_time("Centering by moving right %f" % wCenterVal)
						set_velocity_body(flightData.vehicle, scaleVelocity(DIR_RIGHT, speed))
						time.sleep(.25)
						# Stop and wait for updated values
						set_velocity_body(flightData.vehicle, DIR_NONE)
						time.sleep(.1)
						continue
					# Center to the left
					elif (wCenterVal < 0.5 - wCenterTolerance):
						print_time("Centering by moving left%f" % wCenterVal)
						set_velocity_body(flightData.vehicle, scaleVelocity(DIR_LEFT, speed))
						time.sleep(.25)
						# Stop and wait for updated values
						set_velocity_body(flightData.vehicle, DIR_NONE)
						time.sleep(.1)
						continue

				print_time("Moving forward")
				set_velocity_body(flightData.vehicle, scaleVelocity(DIR_FRONT, 0.6))
				time.sleep(0.2)
			elif (dist > 3):
				# maneuver code
				if flightData.objectClass == "Pylon":
					print_time("Beginning Pylon maneuver")
					pylon_maneuver_circle(flightData)
				elif flightData.objectClass == "Ring":
					print_time("Beginning Ring maneuver")
					ring_maneuver(flightData)
				else:
					print_time("Failed to maneuver - unknown class %s" % flightData.objectClass)
				shouldLoop = False
			else:
				print_time("Too close to obstacle - stopping")
				set_velocity_body(flightData.vehicle, DIR_NONE)
				time.sleep(0.05)
	print_time("Exited main loop. Landing.")
	#if flightData.vehicle.mode.name is "LOITER":
	flightData.vehicle.mode = VehicleMode("LAND")
	time.sleep(2)
	print_time("Goodbye!")
	flightData.vehicle.close()

#-------------------- the rest of flight functions -----------
def conn():
	parser = argparse.ArgumentParser()
	parser.add_argument('--connect', default='/dev/ttyTHS1') #127.0.0.1:14550   /dev/ttyTHS1
	args = parser.parse_args()

	# Connect to the Vehicle
	print ('Connecting to vehicle on: %s' % args.connect)
	vehicle = connect(args.connect, baud=921600, wait_ready=True)
	return vehicle

# Performs the pylon maneuver. Intended to be run from the main flight loop.
# Does NOT work well outside
def pylon_maneuver_circle(flightData):
	shouldLoop = True
	tooClose = False
	flightData.vehicle.mode = VehicleMode("CIRCLE")
	print_time("Set mode to CIRCLE")
	secondsElapsed = 0
	# Seconds it takes for drone to complete a full circle
	fullCircleDuration = 20
	# If drone gets closer than this distance (in ft)
	# cancel the maneuver
	minDistance = 0.75
	while shouldLoop:
		# Send altitude and current mode to the HUD
		flightData.callback(flightData.vehicle.location.global_relative_frame.alt *  3.3, M_AUTOMANEUVER)
		if flightData.stats is None:
			print_time("Warn: No object detected during circle maneuver")
		else:
			dist = flightData.stats.get("distance", 0)
			if dist <= minDistance:
				print_time("Too close to pylon! Abandoning maneuver.")
				set_velocity_body(flightData.vehicle, scaleVelocity(DIR_BACK, 0.3))
				time.sleep(0.5)
				shouldLoop = False
				tooClose = True
				continue
		time.sleep(0.05)
		secondsElapsed += 0.05
		if secondsElapsed > fullCircleDuration:
			print_time("Circle complete")
			shouldLoop = False
	# end function early if too close
	if tooClose:
		return
	flightData.vehicle.mode = VehicleMode("GUIDED")
	print_time("Set mode to GUIDED")
	print_time("Beginning pylon pass & land in 3 seconds")
	time.sleep(3)
	print_time("PYLON - Going right")
	move_for_seconds(vehicle, scaleVelocity(DIR_RIGHT,0.5), 2)
	print_time("PYLON - Going fwd")
	move_for_seconds(vehicle, scaleVelocity(DIR_FRONT,0.5), 5)
	print_time("Ending pylon maneuver")

# Does a square around the pylon and then flies past
def pylon_maneuver(flightData):
	global pylonManeuverDone
	shouldLoop = True
	process = Thread(target = pylon_maneuver_helper, args= (flightData.vehicle,))
	process.start()
	while not pylonManeuverDone:
		# Send altitude and current mode to the HUD
		flightData.callback(flightData.vehicle.location.global_relative_frame.alt *  3.3, M_AUTOMANEUVER)
	
	print_time("Ending pylon maneuver")

# Helper function for the square around the pylon 
def pylon_maneuver_helper(vehicle):
	global pylonManeuverDone
	print_time("PYLON - Going right")
	move_for_seconds(vehicle, scaleVelocity(DIR_RIGHT,0.5), 2.5)
	print_time("PYLON - Going fwd")
	move_for_seconds(vehicle, scaleVelocity(DIR_FRONT,0.5), 6)
	print_time("PYLON - Going left")
	move_for_seconds(vehicle, scaleVelocity(DIR_LEFT,0.5), 5)
	print_time("PYLON - Going back")
	move_for_seconds(vehicle, scaleVelocity(DIR_BACK,0.5), 6)
	print_time("PYLON - Going right")
	move_for_seconds(vehicle, scaleVelocity(DIR_RIGHT,0.5), 3)
	print_time("Square complete. Passing obstacle in two seconds.")
	time.sleep(2)
	print_time("PYLON - Going right")
	move_for_seconds(vehicle, scaleVelocity(DIR_RIGHT,0.5), 2.5)
	print_time("PYLON - Going fwd")
	move_for_seconds(vehicle, scaleVelocity(DIR_FRONT,0.5), 5)
	pylonManeuverDone = True

def ring_maneuver(flightData):
	shouldLoop = True
	while shouldLoop:
		hCenterVal = flightData.stats.get("centerY", 270) / 540
		hCenterTolerance = 0.05
		hDistFromCenter = abs(0.5 - hCenterVal)
		wCenterVal = flightData.stats.get("centerX", 360) / 720
		# How off-center can the target be (in %)
		wCenterTolerance = 0.05
		wDistFromCenter = abs(0.5 - wCenterVal)
		speed = 0.1
		# Set the speed based on how off-centered the drone is
		if hDistFromCenter > 0.3:
			speed = 0.25
		elif hDistFromCenter > 0.2:
			speed = 0.2
		else:
			speed = 0.15
		offCenterW = wCenterVal < 0.5 - wCenterTolerance or wCenterVal > 0.5 + wCenterTolerance
		offCenterH = hCenterVal < 0.5 - hCenterTolerance or hCenterVal > 0.5 + hCenterTolerance
		if offCenterW or offCenterH:
			# Center to the right
			if (hCenterVal > 0.5 + hCenterTolerance):
				print_time("Centering by moving down %f" % hCenterVal)
				set_velocity_body(flightData.vehicle, scaleVelocity(DIR_DOWN, speed))
				time.sleep(.2)
				# Stop and wait for updated values
				set_velocity_body(flightData.vehicle, DIR_NONE)
				time.sleep(.2)
			# Center to the left
			elif (hCenterVal < 0.5 - hCenterTolerance):
				print_time("Centering by moving up %f" % hCenterVal)
				set_velocity_body(flightData.vehicle, scaleVelocity(DIR_UP, speed))
				time.sleep(.2)
				# Stop and wait for updated values
				set_velocity_body(flightData.vehicle, DIR_NONE)
				time.sleep(.2)
			
			# Set the speed based on how off-centered the drone is
			if wDistFromCenter > 0.3:
				speed = 0.2
			elif wDistFromCenter > 0.2:
				speed = 0.15
			else:
				speed = 0.1
			# Center to the right
			if (wCenterVal > 0.5 + wCenterTolerance):
				print_time("Centering by moving right %f" % wCenterVal)
				set_velocity_body(flightData.vehicle, scaleVelocity(DIR_RIGHT, speed))
				time.sleep(.2)
				# Stop and wait for updated values
				set_velocity_body(flightData.vehicle, DIR_NONE)
				time.sleep(.2)
			# Center to the left
			elif (wCenterVal < 0.5 - wCenterTolerance):
				print_time("Centering by moving left%f" % wCenterVal)
				set_velocity_body(flightData.vehicle, scaleVelocity(DIR_LEFT, speed))
				time.sleep(.2)
				# Stop and wait for updated values
				set_velocity_body(flightData.vehicle, DIR_NONE)
				time.sleep(.2)
		else:
				print_time("10 SECONDS TILL MOVING FORWARD")
				time.sleep(10)
				print_time("RING - Moving forward")
				move_for_seconds(flightData.vehicle, scaleVelocity(DIR_FRONT,0.3), 3)
				time.sleep(0.2)
				shouldLoop = False
				print_time("Ending ring maneuver")
		
		flightData.callback(flightData.vehicle.location.global_relative_frame.alt * 3.3, M_AUTOMANEUVER)
		time.sleep(0.5)


""" Remember: vz is positive downward!!!
http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

Bitmask to indicate which dimensions should be ignored by the vehicle 
(a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
none of the setpoint dimensions should be ignored). Mapping: 
bit 1: x,  bit 2: y,  bit 3: z, 
bit 4: vx, bit 5: vy, bit 6: vz, 
bit 7: ax, bit 8: ay, bit 9:
"""
def set_velocity_body(vehicle, velocity):
	vx = velocity[0]
	vy = velocity[1]
	vz = velocity[2]
	if vx == 0 and vy == 0 and vz == 0:
		print_time("Stopping")
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

# Sets the vehicle velocity and holds that velocity for `max_count` seconds
# then stops the vehicle
def move_for_seconds(vehicle, velocity, duration):
	counter = 0 
	while counter < duration:
		set_velocity_body(vehicle, velocity)
		time.sleep(1)
		counter = counter + 1
	set_velocity_body(vehicle, DIR_NONE)
	time.sleep(.1)
