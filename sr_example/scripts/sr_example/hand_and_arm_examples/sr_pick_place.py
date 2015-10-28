#!/usr/bin/env python

# Example to demonstrate moving to stored/names targets. Both arm and hand movements executed.
# Available named targets can be viewed in MoveIt, on the planning tab.

import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder

rospy.init_node("pick_place", anonymous=True)

hand_finder = HandFinder()

hand_parameters = hand_finder.get_hand_parameters()
hand_serial = hand_parameters.mapping.keys()[0]

hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                 hand_serial=hand_serial)
arm_commander = SrArmCommander()


def wait_for_topic():
    try:
        pass  # input("Press enter to continue")
    except:
        pass

while not rospy.is_shutdown():
    arm_commander.move_to_named_target("arm_pre_bracket", True)
    hand_commander.move_to_named_target("hand_pre_bracket", True)
    wait_for_topic()
    arm_commander.move_to_named_target("arm_grasp_bracket", True)
    wait_for_topic()
    hand_commander.move_to_named_target("hand_grasp_bracket", True)
    wait_for_topic()
    arm_commander.move_to_named_target("arm_pre_bracket", True)
    wait_for_topic()
    arm_commander.move_to_named_target("arm_pre_place_bracket", True)
    wait_for_topic()
    arm_commander.move_to_named_target("arm_place_bracket", True)
    wait_for_topic()
    hand_commander.move_to_named_target("hand_pre_bracket", True)
    wait_for_topic()
    arm_commander.move_to_named_target("arm_pre_place_bracket", True)
    wait_for_topic()
    arm_commander.move_to_named_target("arm_place_bracket", True)
    wait_for_topic()
    hand_commander.move_to_named_target("hand_grasp_bracket", True)
    wait_for_topic()
    arm_commander.move_to_named_target("arm_pre_place_bracket", True)
    wait_for_topic()
    arm_commander.move_to_named_target("arm_pre_bracket", True)
    wait_for_topic()
    arm_commander.move_to_named_target("arm_grasp_bracket", True)
    wait_for_topic()
    hand_commander.move_to_named_target("hand_pre_bracket", True)
    wait_for_topic()
