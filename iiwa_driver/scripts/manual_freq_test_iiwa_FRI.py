#!/usr/bin/env python3
# license removed for brevity
import rospkg
import rospy
import os
import argparse

import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS message types
from sensor_msgs.msg import JointState

# ROS messages types of the real robot
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

from topic_tools.srv import MuxSelect

# ------------------------------------------------------------
#  REAL ROBOT
# ------------------------------------------------------------
NDOF = 7
# REAL_ROBOT_TARGET_JOINT_COMMAND_TOPIC = "PositionController/command" # commands joint states on this topic
REAL_ROBOT_TARGET_JOINT_COMMAND_TOPIC = "Manual/command" # commands joint states on this topic
REAL_ROBOT_JOINT_STATE_TOPIC = "joint_states"

min_joints = [-169, -100, -169, -119, -169, -119, -173]
max_joints = [ 169,  100,  169,  119,  169,  119, 173]

T = 1
dt = 0.01

# number of interpolation steps
NUM_STEPS  = 1000 #T/dt

# frequency
FREQ = 200

SCALE_STEP = 0.001
SCALE_NUM_KNOTS = 0.1

class JointPosParser():
    """docstring for JointPosParser."""

    def __init__(self, robot_name=None, target_joint_position = np.asarray([0,0,0,0,0,0,0])):

        if robot_name==None:
            rospy.logerr(f"No robot name was given.")
            return False

        select_srv = rospy.ServiceProxy(f"{robot_name}_mux_joint_position/select", MuxSelect)
        # activate manual commanding
        select_srv(f"{robot_name}/Manual/command")

        # Setup subscriber that reads commanded robot state
        subscr_real_state_topic_name =  f"{robot_name}/{REAL_ROBOT_JOINT_STATE_TOPIC}"
        rospy.Subscriber(subscr_real_state_topic_name, JointState, self.readRobotStates)

        # Setup ros name for topics
        real_world_publishers_topic_name = f"{robot_name}/{REAL_ROBOT_TARGET_JOINT_COMMAND_TOPIC}"
        print(real_world_publishers_topic_name)
        self.real_world_target_joint_command_publisher = rospy.Publisher(real_world_publishers_topic_name, Float64MultiArray, queue_size=10, latch=True)

        # frequency testing node
        self.name = "Freq_test"

        # parsing index
        self.index = 0

        self.store_q = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # read the current configuration of the robot
        rospy.loginfo(f"Waiting for {robot_name}/{REAL_ROBOT_JOINT_STATE_TOPIC} topic, to read the curent configuration of the robot.")
        msgRobotState = rospy.wait_for_message(f"{robot_name}/{REAL_ROBOT_JOINT_STATE_TOPIC}", JointState)
        current_joint_position = np.asarray(list(msgRobotState.position))

        # transform degrees configuration to rads
        target_joint_position_rad = np.deg2rad(target_joint_position)

        # interpolate between current configuration and the goal configuration
        first_part = np.linspace(current_joint_position, current_joint_position, 20)

        intermediate_q = SCALE_STEP*(target_joint_position_rad - current_joint_position)
        second_part = np.linspace(current_joint_position, current_joint_position + intermediate_q, SCALE_NUM_KNOTS*NUM_STEPS)
        third_part = np.linspace(current_joint_position + intermediate_q, target_joint_position_rad, (1-SCALE_NUM_KNOTS)*NUM_STEPS)

        joint_values = np.vstack((first_part, second_part[0:,:]))
        self.joint_values = np.vstack((joint_values, third_part[0:,:]))


    def readRobotStates(self, msg):

        # ----------------------------------------------------------------------
        # read state from real robot
        # ----------------------------------------------------------------------
        # update the latest robot state
        self.store_q = np.asarray(list(msg.position))


    def publishdNextJointState(self, event):
        """ Publish 6D information for the respective rigid body """

        if self.index < self.joint_values.shape[0]-1:

            # initial the message
            msg = Float64MultiArray()
            msg.layout.dim.append(MultiArrayDimension())
            # info for reconstruction of the 2D array
            msg.layout.dim[0].label  = "columns"
            msg.layout.dim[0].size   = NDOF

            #  fill in positions of the robot
            msg.data = self.joint_values[self.index,:]

            # send to the robot
            self.real_world_target_joint_command_publisher.publish(msg)

            # increase the index
            self.index += 1

        else:
            rospy.loginfo("%s: All points consumed ", self.name)
            self.cleanShutdown()


    def cleanShutdown(self):
        print('')
        rospy.loginfo("%s: Shutting down manual joint position node ", self.name)
        # Shut down write callback
        # self.writeCallbackTimer.shutdown()

        rospy.sleep(1.0)
        rospy.signal_shutdown("Node is done")

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Description of your program')
    parser.add_argument("--robot_name", help="Give the name of the robot")
    parser.add_argument("--target_config", nargs="+", help="Give the target configuration of the robot", type=float)
    args = parser.parse_args()

    if args.target_config:
        goal_q = np.asarray(list(args.target_config))
        rows_target = goal_q.shape[0]
        if rows_target != NDOF:
            rospy.logerr(f"Target configuration was given to the robot has wrong dimensions")
        else:
            np.clip(goal_q, min_joints, max_joints, out=goal_q)


    # Initialize node
    rospy.init_node("ros_Traj_interface", anonymous=True)
    # Initialize node class
    JointPosParser = JointPosParser(args.robot_name, goal_q)

    # Create timer for periodic publisher
    dur = rospy.Duration(1.0/FREQ)
    JointPosParser.writeCallbackTimer = rospy.Timer(dur, JointPosParser.publishdNextJointState)

    # Ctrl-C will stop the script
    rospy.on_shutdown(JointPosParser.cleanShutdown)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

 # end
