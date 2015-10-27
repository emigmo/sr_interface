#!/usr/bin/env python

# Example to demonstrate moving to stored/names targets. Both arm and hand movements executed.
# Available named targets can be viewed in MoveIt, on the planning tab.

import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder

rospy.init_node("hand_shake", anonymous=True)

hand_finder = HandFinder()

hand_parameters = hand_finder.get_hand_parameters()
hand_serial = hand_parameters.mapping.keys()[0]

hand_commander = SrHandCommander(hand_parameters=hand_parameters,
                                 hand_serial=hand_serial)
arm_commander = SrArmCommander()

shake_time = 0.3

one_shake =
[{
    'name': 'up_shake',
    'interpolate_time': shake_time
},{
    'name': 'down_shake',
    'interpolate_time': shake_time
}]

shake_trajectory = one_shake*5

def wait_for_topic():
    pass



hand_commander.move_to_named_target("hand_ready_to_shake", False)
arm_commander.move_to_named_target("arm_ready_to_shake", False)

wait_for_topic()

hand_commander.move_to_named_target("close_hand_for_shake", True)
arm_commander.run_named_trajectory_unsafe(shake_trajectory, True)

wait_for_topic()

hand_commander.move_to_named_target("hand_ready_to_shake", False)
