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
#import pseudo_TATM

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

def define_return_angle(azimuth):
    global return_angle
    if azimuth <= 180:
        return_angle = 180 + azimuth
    elif azimuth > 180:
        return_angle = azimuth - 180
    return return_angle
return_angle = define_return_angle(azimuth)

def get_input():
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

def sentry(child_conn,queue):
    print("sentry initialized and standing by")
    if parent_conn.recv() == 1:
        print("sentry has received signal from TATM")
        queue.put(1)
    sentry(parent_conn,queue)

def socket_loco(queue):
    while True:
        msg = s.recv(1024)
        msg = msg.decode("utf-8")
        queue.put(msg)


def Locomotion(LOS,iterations,return_angle):

    def connectMyCopter():
        vehicle = connect('127.0.0.1:5762',wait_ready=True)
        return vehicle
    def arm_and_takeoff(targetHeight):
        while not vehicle.is_armable:
            print("Waiting for vehicle to become armable")
            time.sleep(1)
        print("vehicle armable")
        vehicle.mode = VehicleMode("GUIDED")
        while vehicle.mode !='GUIDED':
            print("waiting")
            time.sleep(1)
        print("vehicle now in guided mode")
        vehicle.armed = True
        while not vehicle.armed:
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
            counter=counter+1
        decelerate()
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
                print("Standing by for input; (0) to confirm (1) to deny then (Enter):")
                while queue_confirm.empty():
                    loiter(lat1,lon1,alt1)
                    time.sleep(0.5)
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
            counter=counter+1
            time.sleep(1)
        decelerate()
        condition_yaw(90,1,angle2)
        counter=0
        while counter<vel_iterations_NS:
            check_kill()
            send_local_ned_velocity(velocity,0,0)
            counter=counter+1
            time.sleep(1)
        decelerate()
        condition_yaw(90,1,angle3)
        counter=0
        while counter<vel_iterations_EW:
            check_kill()
            send_local_ned_velocity(velocity,0,0)
            counter=counter+1
            time.sleep(1)
        decelerate()
        condition_yaw(270,1,angle4)
        counter=0
        while counter<vel_iterations_NS:
            check_kill()
            send_local_ned_velocity(velocity,0,0)
            counter=counter+1
            time.sleep(1)
        decelerate()
        condition_yaw(270,1,angle1)
        counter=0
        while counter<vel_iterations_EW/2:
            check_kill()
            send_local_ned_velocity(velocity,0,0)
            counter=counter+1
            time.sleep(1)
        global iteration_counter
        iteration_counter=iteration_counter+1

###This is the condensed form of the Locomotion function###
    #Connect to vehicle and define parameters
    vehicle = connectMyCopter()
    vehicle.parameters['RTL_ALT'] = 3
    #Arm and takeoff
    arm_and_takeoff(3)
    dummy_yaw_initializer()
    time.sleep(2)
    #Traverse Line of Sight
    traverse_LOS(LOS)
    decelerate()
    #Sawtooth Sweep
    for i in range (iterations):
        sawtooth_sweep()
    decelerate()
    time.sleep(3)
    print("Patrol completed returning to rally point")
    #Return to launch site and land
    condition_yaw(270,1,return_angle)
    for i in range (iteration_counter):
        counter=0
        while counter<RTL_iterations:
            check_kill()
            send_local_ned_velocity(velocity,0,0)
            counter=counter+1
            time.sleep(1)
    counter=0
    while counter<LOS/velocity:
        check_kill()
        send_local_ned_velocity(velocity,0,0)
        counter=counter+1
        time.sleep(1)
    print("Pathfinder has attempted to return to launch and will now use the RTL command")
    RTL_msg()
    while vehicle.location.global_relative_frame.alt>0.1:
        time.sleep(2)
    vehicle.close()
    print("Mission completed")

def pseudo_check_kill(queue):
    while True:
        if not queue.empty():
            print("monitor has recognized that queue is populated")
                if queue.get() == 1:
                print("kill signal received from TATM")
        time.sleep(1)
#############################################################3

def pseudo_Neural_Network(queue):
    counter=0
    while True:
        print("TATM seeking targets")
        #p1.start()
        counter=counter+1
        if counter>10:
            print("pseudo TATM has detected target")
            queue.put(1)
            counter=0
        time.sleep(5)

import os
import argparse
import cv2
import numpy as np
import sys
import time
from threading import Thread
import importlib.util

class VideoStream:
    """Camera object that controls video streaming from the Picamera"""
    def __init__(self,resolution=(640,480),framerate=30):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(0)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(3,resolution[0])
        ret = self.stream.set(4,resolution[1])
            
        # Read first frame from the stream
        (self.grabbed, self.frame) = self.stream.read()

    # Variable to control when the camera is stopped
        self.stopped = False

    def start(self):
    # Start the thread that reads frames from the video stream
        Thread(target=self.update,args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.stream.release()
                return

            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
    # Return the most recent frame
        return self.frame

    def stop(self):
    # Indicate that the camera and thread should be stopped
        self.stopped = True

# Define and parse input arguments
parser = argparse.ArgumentParser()
parser.add_argument('--modeldir', help='Folder the .tflite file is located in', default='Sample_TFLite_model')
parser.add_argument('--graph', help='Name of the .tflite file, if different than detect.tflite',
                    default='detect.tflite')
parser.add_argument('--labels', help='Name of the labelmap file, if different than labelmap.txt',
                    default='labelmap.txt')
parser.add_argument('--threshold', help='Minimum confidence threshold for displaying detected objects',
                    default=0.5)
parser.add_argument('--resolution', help='Desired webcam resolution in WxH. If the webcam does not support the resolution entered, errors may occur.',
                    default='1280x720')
parser.add_argument('--edgetpu', help='Use Coral Edge TPU Accelerator to speed up detection',
                    action='store_true')

args = parser.parse_args()

MODEL_NAME = args.modeldir
GRAPH_NAME = args.graph
LABELMAP_NAME = args.labels
min_conf_threshold = float(args.threshold)
resW, resH = args.resolution.split('x')
imW, imH = int(resW), int(resH)
use_TPU = args.edgetpu

# Import TensorFlow libraries
# If tflite_runtime is installed, import interpreter from tflite_runtime, else import from regular tensorflow
# If using Coral Edge TPU, import the load_delegate library
pkg = importlib.util.find_spec('tflite_runtime')
if pkg:
    from tflite_runtime.interpreter import Interpreter
    if use_TPU:
        from tflite_runtime.interpreter import load_delegate
else:
    from tensorflow.lite.python.interpreter import Interpreter
    if use_TPU:
        from tensorflow.lite.python.interpreter import load_delegate

# If using Edge TPU, assign filename for Edge TPU model
if use_TPU:
    # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
    if (GRAPH_NAME == 'detect.tflite'):
        GRAPH_NAME = 'edgetpu.tflite'       

# Get path to current working directory
CWD_PATH = os.getcwd()

# Path to .tflite file, which contains the model that is used for object detection
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)

# Load the label map
with open(PATH_TO_LABELS, 'r') as f:
    labels = [line.strip() for line in f.readlines()]

# Have to do a weird fix for label map if using the COCO "starter model" from
# https://www.tensorflow.org/lite/models/object_detection/overview
# First label is '???', which has to be removed.
if labels[0] == '???':
    del(labels[0])

# Load the Tensorflow Lite model.
# If using Edge TPU, use special load_delegate argument
if use_TPU:
    interpreter = Interpreter(model_path=PATH_TO_CKPT,
                              experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
    print(PATH_TO_CKPT)
else:
    interpreter = Interpreter(model_path=PATH_TO_CKPT)

interpreter.allocate_tensors()

# Get model details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

floating_model = (input_details[0]['dtype'] == np.float32)

input_mean = 127.5
input_std = 127.5

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()

# Initialize video stream
videostream = VideoStream(resolution=(imW,imH),framerate=30).start()
time.sleep(1)

#for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
while True:

    # Start timer (for calculating frame rate)
    t1 = cv2.getTickCount()

    # Grab frame from video stream
    frame1 = videostream.read()

    # Acquire frame and resize to expected shape [1xHxWx3]
    frame = frame1.copy()
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_resized = cv2.resize(frame_rgb, (width, height))
    input_data = np.expand_dims(frame_resized, axis=0)

    # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
    if floating_model:
        input_data = (np.float32(input_data) - input_mean) / input_std

    # Perform the actual detection by running the model with the image as input
    interpreter.set_tensor(input_details[0]['index'],input_data)
    interpreter.invoke()

    # Retrieve detection results
    boxes = interpreter.get_tensor(output_details[0]['index'])[0] # Bounding box coordinates of detected objects
    classes = interpreter.get_tensor(output_details[1]['index'])[0] # Class index of detected objects
    scores = interpreter.get_tensor(output_details[2]['index'])[0] # Confidence of detected objects
    #num = interpreter.get_tensor(output_details[3]['index'])[0]  # Total number of detected objects (inaccurate and not needed)

    # Loop over all detections and draw detection box if confidence is above minimum threshold
    for i in range(len(scores)):
        if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):

            # Get bounding box coordinates and draw box
            # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
            ymin = int(max(1,(boxes[i][0] * imH)))
            xmin = int(max(1,(boxes[i][1] * imW)))
            ymax = int(min(imH,(boxes[i][2] * imH)))
            xmax = int(min(imW,(boxes[i][3] * imW)))
            
            cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

            # Draw label
            object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
            label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
            label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
            cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
            cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
            ##################
            if object_name == 'person':
                queue.put(1)
                print("NN has detected person")
            ##################
    # Draw framerate in corner of frame
    cv2.putText(frame,'FPS: {0:.2f}'.format(frame_rate_calc),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

    # All the results have been drawn on the frame, so it's time to display it.
    #cv2.imshow('Object detector', frame)

    # Calculate framerate
    t2 = cv2.getTickCount()
    time1 = (t2-t1)/freq
    frame_rate_calc= 1/time1

    # Press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break

# Clean up
cv2.destroyAllWindows()
videostream.stop()
###################################################################

if __name__=='__main__':
    parent_conn,child_conn = Pipe()
    queue = Queue(maxsize=1)
    queue_confirm = Queue()
    #############################################
    loco_proc = Process(target=pseudo_Locomotion, args=(LOS,iterations,return_angle))
    TATM_proc = Process(target=Neural_Network, args=(queue,))
    #############################################
    TATM_proc.start()
    time.sleep(1)
    loco_proc.start()
    #sentry_proc.start()
    get_input()
    loco_proc.join()
    print("Mission completed")

