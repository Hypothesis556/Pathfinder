#!/bin/bash

cd /home/pi/tensorflow1/models/research
protoc object_detection/protos/*.proto --python_out=.
cd /home/pi/tensorflow1/models/research/object_detection
wget http://download.tensorflow.org/models/object_detection/ssdlite_mobilenet_v2_coco_2018_05_09.tar.gz
tar -xzvf ssdlite_mobilenet_v2_coco_2018_05_09.tar.gz