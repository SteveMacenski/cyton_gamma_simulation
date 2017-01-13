#!/usr/bin/env python
from __future__ import division 
import sys
from sensor_msgs.msg import JointState
import copy
import rospy
from std_msgs.msg import Float64
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import PyQt4
import tf
from math import degrees, radians
from actionlib_msgs.msg import GoalID

#take commandline or launch file param for mainpulation group
#and gripper gorups
#TODO
#change to gripper group for release version

class CytonMotion():

    def __init__(self):
        #initalize ROS node, MoveIt Commander, and publishers
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(
                            "manipulator_planning_group")
        self.group.set_goal_position_tolerance(0.001)
        self.group.set_goal_orientation_tolerance(0.01)
        self.group.allow_replanning(True)
        self.num_joints = len(self.group.get_active_joints())
        self.display_trajectory_publisher = rospy.Publisher(
                '/move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory,queue_size=10)
        self.gripper_publisher = rospy.Publisher(
                '/gripper_position_controller/command',
                Float64,queue_size=10)
        self.traj_canceller = rospy.Publisher(
                '/cyton_joint_trajectory_action_controller/follow_joint_trajectory/cancel',
                GoalID,queue_size=10)
        self.velScale = 1


    def visualize(self,plan):
        #to visualize trajectory in RVIZ window

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = \
                                  self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)


    def moveCartesian(self, orientation, pose, execute):
        #move cartesian coordinates

        self.group.clear_pose_targets()

        #cartesian motion
        waypoints = []

        # get
        waypoints.append(self.group.get_current_pose().pose)

        # set
        quaternion = tf.transformations.quaternion_from_euler(
               orientation[0], orientation[1], orientation[2])
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]
        wpose.position.x = pose[0]
        wpose.position.y = pose[1]
        wpose.position.z = pose[2]
        waypoints.append(copy.deepcopy(wpose))

        # plan
        (plan, fraction) = self.group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold

        # control the velocity
        plan = self.timeParamzn(plan, True)

        try:
            if execute:
                # execute path
                self.group.execute(plan, wait=False)
            else:
                self.stored_plan = plan
        except:
            print "plan could not be executed"


    def deltaMoveCartesian(self, orientation, pose, execute):
        #move in a delta move, or change in position movement cartesian

        self.group.clear_pose_targets()
        #cartesian motion
        waypoints = []

        # get
        waypoints.append(self.group.get_current_pose().pose)

        # set
        quaternion = tf.transformations.quaternion_from_euler(
               orientation[0], orientation[1], orientation[2])
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]
        wpose.position.x = pose[0]
        wpose.position.y = pose[1]
        wpose.position.z = pose[2]
        waypoints.append(copy.deepcopy(wpose))

        # plan
        (plan, fraction) = self.group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold

        # control the velocity
        plan = self.timeParamzn(plan, True)

        try:
            if execute:
                # execute path
                self.group.execute(plan, wait=False)
            else:
                self.stored_plan = plan
        except:
            print "plan could not be executed"


    def moveJoint(self, angles, execute):
        #plan a joint space motion 

        self.group.clear_pose_targets()

        try:
            #get
            group_variable_values = \
             self.group.get_current_joint_values()

            #set
            for i in range(self.num_joints):
                group_variable_values[i] = angles[i]

            self.group.set_joint_value_target(
                        group_variable_values)

        except:
            print "cannot set angles"

        try:
            # plan and velocity control
            plan = self.group.plan()
            plan = self.timeParamzn(plan)

            if execute:
                # execute
                self.group.execute(plan, wait=False)
            else:
                self.stored_plan = plan
        except:
            print "plan could not be executed"


    def deltaMoveJoint(self,angles, execute):
        #plan a delta joint space motion, or change in joint angles

        self.group.clear_pose_targets()

        try:
            #get
            group_variable_values = \
             self.group.get_current_joint_values()

            #set
            for i in range(self.num_joints):
                group_variable_values[i] = angles[i]

            self.group.set_joint_value_target(
                        group_variable_values)

        except:
            print "cannot set angles"

        try:
            # plan and velocity control
            plan = self.group.plan()
            plan = self.timeParamzn(plan)

            if execute:
                # execute
                self.group.execute(plan, wait=False)
            else:
                self.stored_plan = plan
        except:
            print "plan could not be executed"


    def executeStoredPath(self):
        #execute this path generated when not executed on planning

        self.group.execute(self.stored_plan, wait=False);


    def moveGripper(self, move):
        #publish command to gripper to move [-.5 1.9]

        self.gripper_publisher.publish(move)


    def stopMotion(self):
        #stops robot motion in execution

        msg = GoalID()
        self.traj_canceller.publish(msg)

   
    def timeParamzn(self, plan, cart=False):
    #paramaterize time based on a velocity scaling factor

        new_traj = moveit_msgs.msg.RobotTrajectory()
        new_traj.joint_trajectory = plan.joint_trajectory
        n_joints = len(plan.joint_trajectory.joint_names)
        n_points = len(plan.joint_trajectory.points)

        spd = self.velScale

        for i in range(n_points):
            new_traj.joint_trajectory.points[i].time_from_start = \
                plan.joint_trajectory.points[i].time_from_start / spd

            if (cart and i==n_points-1):
                return new_traj   

        for i in range(n_points):

            new_traj.joint_trajectory.points[i].velocities = \
                list(new_traj.joint_trajectory.points[i].velocities)

            new_traj.joint_trajectory.points[i].accelerations = \
                list(new_traj.joint_trajectory.points[i].accelerations)

            new_traj.joint_trajectory.points[i].positions = \
                list(new_traj.joint_trajectory.points[i].positions)

            for j in range(n_joints):

                new_traj.joint_trajectory.points[i].velocities[j] = \
                    plan.joint_trajectory.points[i].velocities[j] * spd
                new_traj.joint_trajectory.points[i].accelerations[j] = \
                    plan.joint_trajectory.points[i].accelerations[j] * spd * spd
                new_traj.joint_trajectory.points[i].positions[j] = \
                    plan.joint_trajectory.points[i].positions[j]

            new_traj.joint_trajectory.points[i].velocities = \
                tuple(new_traj.joint_trajectory.points[i].velocities)

            new_traj.joint_trajectory.points[i].accelerations = \
                tuple(new_traj.joint_trajectory.points[i].accelerations)

            new_traj.joint_trajectory.points[i].positions = \
                tuple(new_traj.joint_trajectory.points[i].positions)

        return new_traj


    def changeVelocityScaling(self, velScale):
        #change the velocity scaling values from front-end
        self.velScale = velScale


