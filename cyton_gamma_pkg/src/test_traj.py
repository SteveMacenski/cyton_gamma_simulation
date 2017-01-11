#!/usr/bin/env python

import rospy
import std_msgs
from rospy import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import sys

def functional(axis, angle):
    pub_rosey = rospy.Publisher(
      '/cyton_joint_trajectory_action_controller/command',
      JointTrajectory, queue_size=10)
    rospy.init_node('traj_maker', anonymous=True)
    time.sleep(1)

    axis = int(axis)
    angle = float(angle)

    rate = rospy.Rate(0.01)
    while not rospy.is_shutdown():

           #  first way to define a point
           traj_waypoint_1_rosey = JointTrajectoryPoint()

           traj_waypoint_1_rosey.positions = [0,0,0,0,0,0,0]
           traj_waypoint_1_rosey.time_from_start = Duration(2)
           
           #  second way to define a point
           traj_waypoint_2_rosey = traj_waypoint_1_rosey
           traj_waypoint_2_rosey.time_from_start = Duration(4)
           traj_waypoint_2_rosey.positions[axis-1] = angle
           
           time.sleep(1)

           traj_waypoint_3_rosey = traj_waypoint_2_rosey
           traj_waypoint_3_rosey.time_from_start = Duration(7)

           #traj_waypoint_2_rosey = JointTrajectoryPoint(positions=[.31, -.051, .33, -.55, .28, .60,0],
            #time_from_start = Duration(4))
           #traj_waypoint_3_rosey = JointTrajectoryPoint(positions=[.14726, -.014151, .166507, -.33571, .395997, .38657,0],
            #time_from_start = Duration(6))
           #traj_waypoint_4_rosey = JointTrajectoryPoint(positions=[-.09309, .003150, .003559, .16149, .524427, -.1867,0],
            #time_from_start = Duration(8))
           #traj_waypoint_5_rosey = JointTrajectoryPoint(positions=[-.27752, .077886, -.1828, .38563, .682589, -.44665,0],
            #time_from_start = Duration(10))   
           #traj_waypoint_6_rosey = JointTrajectoryPoint(positions=[0,0,0,0,0,0,0],time_from_start = Duration(12))

           #debug in terminal
           print traj_waypoint_2_rosey.positions
           
           #  making message
           message_rosey = JointTrajectory()
           
           #  required headers
           header_rosey = std_msgs.msg.Header(stamp=rospy.Time.now())
           message_rosey.header = header_rosey
           
           #  adding in joints
           joint_names = ['shoulder_roll_joint', \
  'shoulder_pitch_joint', 'shoulder_yaw_joint', 'elbow_pitch_joint', \
  'elbow_yaw_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
           message_rosey.joint_names = joint_names
           
           #  appending up to 100 points
           # ex. for i in enumerate(len(waypoints)): append(waypoints[i])

           message_rosey.points.append(traj_waypoint_1_rosey)
           message_rosey.points.append(traj_waypoint_2_rosey)
           message_rosey.points.append(traj_waypoint_3_rosey)
           #message_rosey.points.append(traj_waypoint_4_rosey)
           #message_rosey.points.append(traj_waypoint_5_rosey)
           #message_rosey.points.append(traj_waypoint_6_rosey)
           #  publishing to ROS node
           pub_rosey.publish(message_rosey)
         
           rate.sleep()
           
           if rospy.is_shutdown():
               break
               

if __name__ == '__main__':
    axis = sys.argv[1]
    angle = sys.argv[2]
    try:
        functional(axis, angle)
    except rospy.ROSInterruptException:
        pass
