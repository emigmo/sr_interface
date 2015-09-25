#!/usr/bin/env python

# An example of how to print the current joint positions for all joints.
# Useful when used in conjunction with 'teach mode', as new motion sequences can
# be quickly produced from physically moving the robot into position and then
# recording those positions with this script

import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder
from math import degrees

rospy.init_node("print_hand_joints_position", anonymous=True)

# Use the hand finder to get the hand prefix, to allow this script to be used with either left or right hands
hand_finder = HandFinder()
hand_parameters = hand_finder.get_hand_parameters()
prefix = hand_parameters.mapping.values()[0]
hand_serial = hand_parameters.mapping.keys()[0]

group_id = "rh_shadow_hand"

hand_commander = SrHandCommander(name=group_id,hand_parameters=hand_parameters, hand_serial=hand_serial)

print("Joints positions")

all_joints_state = hand_commander.get_joints_position()

all_joints_state.update((joint, degrees(i))
                        for joint, i in all_joints_state.items())

hand_joints_state = {
    k: v for k, v in all_joints_state.items() if k.startswith(prefix + "_")}

print("Hand joints position \n " + str(hand_joints_state) + "\n")
