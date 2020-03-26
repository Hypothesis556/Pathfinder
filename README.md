# Pathfinder
Code pertaining to the creation of a hunting companion quad-copter drone.

The project has two main components:
  1) Locomotion: Uses Dronekit's Python API to move the drone in 3D space. Has an interface to interact with the second module.
  2) Target Acquisition and Tracking Module(TATM): This module deploys a SSD Neural Network trained with Tensorflow's Object Detection      API. It's purpose is to detect hunting targets (in this case deer) and send a signal to the Locomotion module. 

Once the Locomotion module receives the signal it pauses and the drone loiters in place. It then requests input from the operator who either confirms or denies that a target has been detected once he has checked the camera feed.
