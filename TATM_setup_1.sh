#!/bin/bash

pip3 install tensorflow
sudo apt-get install libatlas-base-dev
sudo pip3 install pillow lxml jupyter matplotlib cython
sudo apt-get install python-tk
sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libxvidcore-dev libx264-dev
sudo apt-get install qt4-dev-tools libatlas-base-dev
sudo pip3 install opencv-python

sudo apt-get install protobuf-compiler
protoc --version

mkdir tensorflow1
cd tensorflow1
git clone --depth 1 https://github.com/tensorflow/models.git
sudo nano ~/.bashrc
