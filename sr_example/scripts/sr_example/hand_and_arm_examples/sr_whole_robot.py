#!/usr/bin/env python

# This example demonstrates how you can send a trajectory created from named poses.

from logitech_r400.msg import LogitechR400
import rospy
from sr_robot_commander.sr_robot_commander import SrRobotCommander
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_utilities.hand_finder import HandFinder


class ManipulationDemo(object):

    def __init__(self):
        self.robot_commander = SrRobotCommander("full_robot")
        self.r400_sub = rospy.Subscriber("/logitech_r400/state", LogitechR400, self.cmd_cb)

        self.last_r400_msg = None

        self.hand_finder = HandFinder()

        self.hand_parameters = self.hand_finder.get_hand_parameters()
        self.hand_serial = self.hand_parameters.mapping.keys()[0]

        self.hand_commander = SrHandCommander(hand_parameters=self.hand_parameters,
                                              hand_serial=self.hand_serial)
        self.arm_commander = SrArmCommander()

        self.shake_time = 0.2

        self.one_shake = [{
            'name': 'up_shake',
            'interpolate_time': self.shake_time
        },{
            'name': 'down_shake',
            'interpolate_time': self.shake_time
        }]

        self.shake_trajectory = self.one_shake*5

    @staticmethod
    def move_exaust_traj(from_pos, to_pos):

        from_pre = "arm_exhaust_%d_pre" % from_pos
        from_approach = "arm_exhaust_%d_approach" % from_pos
        from_grasp = "arm_exhaust_%d_grasp" % from_pos

        to_pre = "arm_exhaust_%d_pre" % to_pos
        to_approach ="arm_exhaust_%d_approach" % to_pos
        to_grasp = "arm_exhaust_%d_grasp" % to_pos

        trajectory = [{
            'name': from_pre,
            'interpolate_time': 1.0
        },
        {
            'name': 'hand_pre_exhaust',
            'interpolate_time': 0.5
        },
        {
            'name': from_approach,
            'interpolate_time': .5
        },
        {
            'name': from_grasp,
            'interpolate_time': .5
        },
        {
            'name': 'hand_grasp_exhaust',
            'interpolate_time': 0.5
        },
        {
            'name': from_pre,
            'interpolate_time': 1
        },
        {
            'name': to_pre,
            'interpolate_time': 1.0
        },
        {
            'name': to_grasp,
            'interpolate_time': 1.5
        },
        {
            'name': 'hand_pre_exhaust',
            'interpolate_time': 1.5
        },
        {
            'name': to_approach,
            'interpolate_time': .2
        },
        {
            'name': to_pre,
            'interpolate_time': .2
        }]
        return trajectory

    @staticmethod
    def move_bracket_traj(from_pos, to_pos):
        from_pre = "arm_bracket_%d_pre" % from_pos
        from_grasp = "arm_bracket_%d_grasp" % from_pos

        to_pre = "arm_bracket_%d_pre" % to_pos
        to_grasp = "arm_bracket_%d_grasp" % to_pos

        trajectory =  [{
            'name': from_pre,
            'interpolate_time': 1.0
        },
        {
            'name': 'hand_pre_bracket',
            'interpolate_time': 0.5
        },
        {
            'name': from_grasp,
            'interpolate_time': .5
        },
        {
            'name': 'hand_grasp_bracket',
            'interpolate_time': .5
        },
        {
            'name': from_pre,
            'interpolate_time': .5
        },
        {
            'name': to_pre,
            'interpolate_time': 1
        },
        {
            'name': to_grasp,
            'interpolate_time': .5,
            'pause_time': .2
        },
        {
            'name': 'hand_pre_bracket',
            'interpolate_time': .5
        },
        {
            'name': to_pre,
            'interpolate_time': .5
        }]
        return trajectory

    def get_bracket_trajectory(self):
        bracket_trajectory = self.move_bracket_traj(1,5) + self.move_bracket_traj(2,6) + self.move_bracket_traj(3,7) + \
                             self.move_bracket_traj(5,1) + self.move_bracket_traj(6,2) + self.move_bracket_traj(7,3)  
        return bracket_trajectory

    def get_exhaust_trajectory(self):
        exhaust_trajectory = (self.move_exaust_traj(1,3) + self.move_exaust_traj(2,4) + self.move_exaust_traj(3,1) + self.move_exaust_traj(4,2)) # *3
        return exhaust_trajectory

    def get_baffle_trajectory(self):
        baffle_trajectory = [
            {
                'name': 'arm_baffle_pre_2',
                'interpolate_time': 1
            },
            {
                'name': 'hand_baffle_pre_2',
                'interpolate_time': 1
            },
            {
                'name': 'arm_baffle_grasp_2',
                'interpolate_time': 1
            },
            {
                'name': 'hand_baffle_grasp_2',
                'interpolate_time': 1
            },
            {
                'name': 'arm_baffle_pre_2',
                'interpolate_time': 1,
                'pause_time': 1
            },
            {
                'name': 'arm_baffle_show',
                'interpolate_time': 1,
                'pause_time': 2
            },
            {
                'name': 'arm_baffle_pre_2',
                'interpolate_time': 1
            },
            {
                'name': 'arm_baffle_grasp_2',
                'interpolate_time': 1
            },
            {
                'name': 'hand_baffle_pre_2',
                'interpolate_time': 1
            },
            {
                'name': 'arm_baffle_pre_2',
                'interpolate_time': 1
            }

        ]
        return baffle_trajectory

    def hand_shake(self):
        self.arm_commander.move_to_named_target("prepare_to_shake", True)
        self.hand_commander.move_to_named_target("hand_ready_to_shake", True)

        self.wait_for_topic()

        self.arm_commander.move_to_named_target("ready_to_shake", True)

        self.wait_for_topic()

        self.hand_commander.move_to_named_target("close_for_shake", True)
        self.arm_commander.run_named_trajectory_unsafe(self.shake_trajectory, True)

        self.hand_commander.move_to_named_target("hand_ready_to_shake", True)
        self.arm_commander.move_to_named_target("prepare_to_shake", True)

        rest_trajectory = [
            {
                'name': 'above_table',
                'interpolate_time': 1
            },
            {
                'name': 'rest_on_table',
                'interpolate_time': 1
            }]
        self.robot_commander.run_named_trajectory(rest_trajectory)

    def run(self):
        while not rospy.is_shutdown():
            self.wait_for_topic()
            # Run trajectories via moveit
            trajectory = self.get_bracket_trajectory()
            self.robot_commander.run_named_trajectory(trajectory)

            self.robot_commander.move_to_named_target("intermediate", True)

            trajectory = self.get_exhaust_trajectory()
            self.robot_commander.run_named_trajectory(trajectory)

            self.robot_commander.move_to_named_target("intermediate", True)


            trajectory = self.get_bracket_trajectory()
            self.robot_commander.run_named_trajectory(trajectory)

            self.robot_commander.move_to_named_target("intermediate", True)

            trajectory = self.get_exhaust_trajectory()
            self.robot_commander.run_named_trajectory(trajectory)

            #trajectory = self.get_baffle_trajectory()
            #self.robot_commander.run_named_trajectory(trajectory)

            self.hand_shake()

    def cmd_cb(self, msg):
        self.last_r400_msg = msg

    def wait_for_topic(self):
        rospy.loginfo("Waiting for input")
        while not rospy.is_shutdown():
            if self.last_r400_msg and self.last_r400_msg.buttons[0]:
                return
            rospy.sleep(0.05)

if __name__ == "__main__":
    rospy.init_node("test")
    demo = ManipulationDemo()
    demo.run()
