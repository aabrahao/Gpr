#!/bin/bash
rm -rf build install log
clear
colcon build
source install/setup.bash
