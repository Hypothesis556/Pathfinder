# Pathfinder
Code pertaining to the creation of a hunting companion quad-copter drone with night-vision capabilities.

BACKGROUND:
The project has two main components:
  1) Locomotion: Uses Dronekit's Python API to move the drone in 3D space. Locomotion uses a sweeping pattern to scan the area for potential targets. Locomotion uses an interface to interact with the second module.
  2) Target Acquisition and Tracking Module(TATM): This module deploys a SSD Neural Network trained with Tensorflow's Object Detection      API. It's purpose is to use the attached night-vision enabled picamera to detect hunting targets (in this case deer) and send a signal to the Locomotion module. Once the Locomotion module recieves the signal it holds as still as possible so as to remain undetected, and relay it's coordinates to the operator so he/she can attempt to move to and get a fix on the target. The drone will then wait for a confirmation signal from the operator who must press a (1) to confirm that a valid target has been detected or a (0) to deny that a valid target has been detected, followed by the (Enter) key. How the drone responds after each confirmation is shown below:
    (1) Valid target detected: The drone will rotate 180 degrees, slowly fly away from the target until it reaches a predetermined distance, then land and quietly disarm. The drone will relay the coordinates of it's landing site so the operator can retrieve it.
    (0) No target detected: This command is for false positives cause by another non-target animal or just noise. This command will command the drone to ignore this detection and continue with the mission.
    
PREREQUISITES:

  Materials List:

CONSTRUCTION:

  Hardware:
  
    Install Camera:
    
  Software:
  
    Download this repository:
    
    Setup Ardupilot and DroneKit on RPI:
    
    Setup tensorflow on RPI:
    
    Move files to object_detection/data and object_detection

OPERATION:
  Run Time Arguments:

FURTHER DEVELOPEMENT:
  Optical Flow Meter Based Loitering:
  TFLite Based TATM:
  Propeller Shrouds:
  
