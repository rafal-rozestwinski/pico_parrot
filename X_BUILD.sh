#!/bin/bash -x
set -e
# PICO_SDK_PATH
# cd $HOME
# git clone --recursive https://github.com/raspberrypi/pico-sdk
# export PICO_SDK_PATH=$HOME/pico-sdk

# clone https://github.com/raspberrypi/picotool build and install - or copy to pico block device.



rm -rf build
mkdir -vp build
cd build
#cmake .. -DPICO_SDK_FETCH_FROM_GIT=1
cmake ..
make
cd ..
./picoload.sh build/pico_parrot.uf2 
sleep 3
miniterm /dev/ttyACM0 921600