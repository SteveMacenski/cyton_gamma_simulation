#!/bin/bash

source /opt/ros/indigo/setup.bash
source /home/rdtintern/catkin_ws/devel/setup.bash

echo "[1/5] start controller manager"

xterm -hold -e roslaunch cyton_gamma_300_controllers controller_manager.launch &

sleep 10

echo "[2/5] start controllers"

xterm -e  roslaunch cyton_gamma_300_controllers start_controller.launch &

sleep 10

echo "[3/5] start joint state publisher"

xterm -e rosrun cyton_gamma_300_controllers dynamixel_joint_state_publisher.py &

sleep 5

echo "[4/5] start planning execution window"

xterm -e roslaunch cyton_gamma_pkg gamma_300_planning_execution.launch 2> /dev/null &

sleep 20

echo "[5/5] start command front end"

xterm -e rosrun cyton_gamma_pkg combined_front_end.py 2> /dev/null 300 &

echo "ready!"
