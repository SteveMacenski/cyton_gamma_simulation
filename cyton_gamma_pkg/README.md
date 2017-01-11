# cyton_gamma_pkg
This package contains the launch files used for simulation and execution on hardware for cyton series robots in ROS Indigo- and a commanding front-end for operations

## cyton_gamma_pkg/launch
Here are the launch files for each type of execution on the Cyton Gamm 1500 and Gamma 300. 

>simulation_gamma_[300 or 1500].launch

This will launch a simulation environment in RVIZ and Gazebo

>gamma_[1500 or 300]_planning_execution.launch

This is an edited version of the ROS-I generated file. It is tuned with removed parts not relavent for my implementation. Additional parameters were added for hardware execution. 

The remaining files, _gaz and _rviz support the simulation package. controllers.yaml in the index of the directory are for the simulation environment for Gazebo.

## cyton_gamma_pkg/src
Here are the python tests of a trajectory generation program, front-end, and path planning class utilizing OMPL and MoveIt.

> ./[300-1500]_hardware_launch.sh 

Will attempt to launch all of the launch scripts for a full launch making it easier on the user. It currently has some bugs resulting in a 50% success rate. 

> rosrun cyton_gamma_pkg command_front_end.py 2>/dev/null

Will launch the front-end with accompanying OMPL/MoveIt planning backend class in robot_planning_class.py. 

> rosrun cyton_gamma_pkg feedback_front_end.py 2>/dev/null

Will launch the front-end with feedback information from the robot and E-STOP capabilities.

If you prefer a single front-end with all capabilities, this will do that and will include velocity control.

> rosrun cyton_gamma_pkg combined_front_end.py 2>/dev/null

*This front-end is highly recommended*

test_traj.py shows how to fill out the trajectory_msgs JointTrajectory type in python and submit those commands directly to hardware. This allows you to bypass a planning node and is independent of OMPL, requiring only a connection to the hardware. 

## Robot Operations
For full instructions see SteveMacenski/cyton_gamma_300-1500_operation_and_simulation
https://github.com/SteveMacenski/cyton_gamma_300-1500_operation_and_simulation

Video details of robot operations and initial front-end: https://www.youtube.com/watch?v=L6wQS88PJ5U
