from multiprocessing import Process, Queue
import time
import socket
from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException,Command
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil
import multiprocessing
from multiprocessing import Process, Queue, Pipe
import sys
print(sys.version)
from pseudo_TATM import pseudo_Neural_Network

#Parameters/Initialization
parser = argparse.ArgumentParser(description='azimuth')
parser.add_argument('--azimuth')
args, unknown = parser.parse_known_args()
azimuth = int(args.azimuth)
print("AZIMUTH: %s"%azimuth)

iterations = 1
iteration_counter=0
LOS = 0
#azimuth = 180

velocity = 0.5 #m/s
delta_EW = 10 #meters
delta_NS = 3 #meters
vel_iterations_EW = delta_EW/velocity #quantity of 1 second velocity messages to send
vel_iterations_NS = delta_NS/velocity
RTL_iterations = (2*delta_NS)/velocity

angles1=[]
angle0 = azimuth
angles1.append(angle0)
angle1 = azimuth+270
angles1.append(angle1)
angle2 = angle1+90
angles1.append(angle2)
angle3 = angle2+90
angles1.append(angle3)
angle4 = angle3+270
angles1.append(angle4)


angles=[]
for i in angles1:
	#print(i)
	while i >=360:
		i = i-360
	#print(i)
	print(i)
	angles.append(i)

angle0 = angles[0]
angle1 = angles[1]
angle2 = angles[2]
angle3 = angles[3]
angle4 = angles[4]

def about_face(heading):
	angle = heading+180
	while angle >=360:
		angle = angle-360
	return angle
#Establish connection with TATM
#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#s.connect(("192.168.0.16", 1235))

def define_return_angle(azimuth):
	if azimuth <= 180:
		return_angle = 180 + azimuth
	elif azimuth > 180:
		return_angle = azimuth - 180
	return return_angle
return_angle = define_return_angle(azimuth)


#msg1="0"
def get_input(msg1="0"):
	print("command interface module initialized")
	keystroke=int(input("1 to confirm 0 to deny: "))
	print('You pressed %s'%keystroke)
	if keystroke==0:		
		queue_confirm.put(0)
		queue_confirm.put(0)
	elif keystroke==1:
		queue_confirm.put(1)
		#sys.exit()
	get_input()
    	#flag=0
   	#print('flag is now:', flag)
	#getinput()
#except EOFError:
	#pass
def sentry(parent_conn):
	print("sentry initialized and standing by")
	if parent_conn.recv() == 1:
		print("sentry has received signal from TATM")
		queue.put(1)
	sentry(parent_conn)

def socket_loco(queue):
    while True:
        msg = s.recv(1024)
        #print(msg.decode("utf-8"))
        msg = msg.decode("utf-8")
        #print(msg)
        queue.put(msg)

def socket_confirm(queue_confirm):
    node = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    node.connect(("192.168.0.4", 1235))
    while True:
        msg = s.recv(1024)
        print(msg.decode("utf-8"))
        msg = msg.decode("utf-8")
        print(msg)
        queue.put(msg)

def Locomotion(LOS,iterations,return_angle):

	def connectMyCopter():

		#parser = argparse.ArgumentParser(description='commands')
		#parser.add_argument('--connect')
		#args = parser.parse_args()

		#connection_string = args.connect
	
		#if not connection_string:
			#import dronekit_sitl
			#sitl = dronekit_sitl.start_default()
			#connection_string = sitl.connection_string()

		vehicle = connect('127.0.0.1:5762',wait_ready=True)

		return vehicle

	def arm_and_takeoff(targetHeight):
		while vehicle.is_armable != True:
			print("Waiting for vehicle to become armable")
			time.sleep(1)
		print("vehicle armable")

		vehicle.mode = VehicleMode("GUIDED")

		while vehicle.mode !='GUIDED':
			print("waiting")
			time.sleep(1)
		print("vehicle now in guided mode")

		vehicle.armed = True
		while vehicle.armed==False:
			print("waiting")
			time.sleep(1)
		print("props spinning")
		vehicle.simple_takeoff(targetHeight) ##meters

		while True:
			print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
			if vehicle.location.global_relative_frame.alt>=0.95*targetHeight:
				break
			time.sleep(1)
		print("Target altitude reached")
		return None

	def condition_yaw(degrees,relative,global_heading):
		if relative:
			is_relative = 1
		else:
			is_relative = 0
		print("Yaw: %s"%vehicle.attitude.yaw)
		msg = vehicle.message_factory.command_long_encode(0,0,mavutil.mavlink.MAV_CMD_CONDITION_YAW,0,degrees,0,1,is_relative,0,0,0)
		vehicle.send_mavlink(msg)
		vehicle.flush()
		timeout = time.time()+10
		while vehicle.heading >= 1.1*global_heading or vehicle.heading <= 0.9*global_heading:
			print("Desired heading: %s, Current heading: %s"%(global_heading,vehicle.heading))
			check_kill()
			if time.time()>=timeout:
				print("Heading error occurred, proceeding with current heading")
				break
			time.sleep(1)		


	def RTL_msg():
		msg = vehicle.message_factory.command_long_encode(0,0,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0)
		vehicle.send_mavlink(msg)
		vehicle.flush()

	def send_local_ned_velocity(vx, vy, vz):
		msg = vehicle.message_factory.set_position_target_local_ned_encode(0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,0b0000111111000111,0, 0, 0,vx, vy, vz,0, 0, 0,0, 0)
		vehicle.send_mavlink(msg)
		vehicle.flush()
		print("Velocity command sent")

	def dummy_yaw_initializer():
		lat=vehicle.location.global_relative_frame.lat
		lon=vehicle.location.global_relative_frame.lon
		alt=vehicle.location.global_relative_frame.alt

		aLocation=LocationGlobalRelative(lat,lon,alt)
	
		msg = vehicle.message_factory.set_position_target_global_int_encode(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000111111111000,aLocation.lat*1e7,aLocation.lon*1e7,aLocation.alt,0,0,0,0,0,0,0,0)
		vehicle.send_mavlink(msg)
		vehicle.flush()
	
	def loiter(lat,lon,alt):
		print("loitering..")
		msg = vehicle.message_factory.command_long_encode(0, 0,mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,0,1,lat,lon,alt,0,0,0)    
		vehicle.send_mavlink(msg)
		vehicle.flush()


	def decelerate(max_velocity=0.1):
		while True:
			check_kill()
			#print("Decelerating for turn; Current Airspeed: %d"%vehicle.airspeed)
			#vehicle.parameters['ANGLE_MAX'] = 1000
			if vehicle.airspeed<=max_velocity:
				break
			time.sleep(0.1)

	def traverse_LOS(LOS):
		condition_yaw(azimuth,0,angle0)
		time.sleep(2)
		counter=0
		while counter<(LOS/velocity):
			send_local_ned_velocity(velocity,0,0)
			time.sleep(1)
			#print("Moving NORTH relative to front of drone")
			counter=counter+1

		decelerate()
		#Setup drone for sawtooth sweep function
		condition_yaw(270,1,angle1)

	def check_kill():
		if not queue.empty():
            		print("monitor has recognized that queue is populated")
            		if queue.get() == 1:
				print("kill signal received from TATM")
				print("Potential target at: %s"%vehicle.location.global_relative_frame)
				decelerate()
				lat1=vehicle.location.global_relative_frame.lat
				lon1=vehicle.location.global_relative_frame.lon
				alt1=vehicle.location.global_relative_frame.alt
				while queue_confirm.empty():
					loiter(lat1,lon1,alt1)
					print("standing by for input")
					time.sleep(1)
				if queue_confirm.get()==1:
					print("Target confirmed, landing in close proximity.")
					angle=about_face(vehicle.heading)
					condition_yaw(180,1,angle)
					for i in range (10):
						send_local_ned_velocity(velocity/2,0,0)
						time.sleep(1)
					decelerate()
					vehicle.mode = VehicleMode('LAND')
					while vehicle.mode != 'LAND':
						print("Pathfinder waiting to engage landing mode, Mode: %s"%vehicle.mode)
						time.sleep(1)
					while vehicle.location.global_relative_frame.alt>0.1:
						print("Landing...")
						time.sleep(2)
					print("Pathfinder has landed at: %s"%vehicle.location.global_relative_frame)
					vehicle.close()
					sys.exit()
				elif queue_confirm.get()==0:
					print("you have denied that this is a target")
					vehicle.mode=VehicleMode('GUIDED')
					while vehicle.mode != 'GUIDED':
						print(vehicle.mode)
						time.sleep(2)
				print("continuing with mission")	

	def sawtooth_sweep():
		global velocity
		counter=0
		while counter<vel_iterations_EW/2:
			check_kill()			
			send_local_ned_velocity(velocity,0,0)
			#print("Moving NORTH relative to front of drone")
			counter=counter+1
			time.sleep(1)
		decelerate()
		condition_yaw(90,1,angle2)
		#print("Yawing 90 deg rel to current frame")
		counter=0
		while counter<vel_iterations_NS:
			check_kill()
			send_local_ned_velocity(velocity,0,0)
			#print("Moving NORTH relative to front of drone")
			counter=counter+1
			time.sleep(1)
		decelerate()
		condition_yaw(90,1,angle3)
		#print("Yawing 270 deg rel to current frame")
		#print('%s'%vehicle.attitude)
		counter=0
		while counter<vel_iterations_EW:
			check_kill()
			send_local_ned_velocity(velocity,0,0)
			#print("Moving NORTH relative to front of drone")
			counter=counter+1
			time.sleep(1)
		decelerate()
		condition_yaw(270,1,angle4)
		#print("Yawing 90 deg rel to current frame")
		counter=0
		while counter<vel_iterations_NS:
			check_kill()
			send_local_ned_velocity(velocity,0,0)
			#print("Moving NORTH relative to front of drone")
			counter=counter+1
			time.sleep(1)
		decelerate()
		condition_yaw(270,1,angle1)
		#print("Yawing 90 deg rel to current frame")
		counter=0
		while counter<vel_iterations_EW/2:
			check_kill()
			send_local_ned_velocity(velocity,0,0)
			#print("Moving NORTH relative to front of drone")
			counter=counter+1
			time.sleep(1)
		global iteration_counter
		iteration_counter=iteration_counter+1
	
	#Connect to vehicle and define parameters
	vehicle = connectMyCopter()
	vehicle.parameters['RTL_ALT'] = 3

	#Arm and takeoff
	
	arm_and_takeoff(3)
	dummy_yaw_initializer()
	time.sleep(2)

	#Traverse Line of Sight
	print("Yaw: %s"%vehicle.attitude.yaw)
	traverse_LOS(LOS)
	decelerate()

	#Sawtooth Sweep
	for i in range (iterations):
		sawtooth_sweep()
	decelerate()
	time.sleep(3)
	print("patrol completed returning to rally point")

	#Return to launch site and land
	condition_yaw(270,1,return_angle)
	for i in range (iteration_counter):
		counter=0
		while counter<RTL_iterations:
			check_kill()
			send_local_ned_velocity(velocity,0,0)
			#print("Moving NORTH relative to front of drone")
			counter=counter+1
			time.sleep(1)
	counter=0
	while counter<LOS/velocity:
		check_kill()
		send_local_ned_velocity(velocity,0,0)
		#print("Moving NORTH relative to front of drone")
		counter=counter+1
		time.sleep(1)

	print("Pathfinder has attempted to return to launch and will now use the RTL command")
	RTL_msg()
	while vehicle.location.global_relative_frame.alt>0.1:
		time.sleep(2)
	vehicle.close()
	print("Mission completed")


def stalking_mode():
    print("stalking mode engaged")
    while True:
        print("drone is stalking prey")
        time.sleep(1)


if __name__ == '__main__':
    parent_conn,child_conn = Pipe()
    queue = Queue(maxsize=1)
    queue_confirm = Queue()
    loco_proc = Process(target=Locomotion, args=(LOS,iterations,return_angle))
    sentry_proc = Process(target=sentry, args=(parent_conn,))
    TATM_proc = Process(target=pseudo_Neural_Network, args=(child_conn,))
    #sock_loco_proc = Process(target=socket_loco, args=(queue,))
    #sock_loco_proc.start()
    loco_proc.start()
    TATM_proc.start()
    sentry_proc.start()
    get_input()
    loco_proc.join()
    print("Mission completed")

