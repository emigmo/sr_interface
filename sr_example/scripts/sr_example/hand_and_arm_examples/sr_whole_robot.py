#!/usr/bin/env python

# This example demonstrates how you can send a trajectory created from named poses.

import rospy

from sr_robot_commander.sr_robot_commander import SrRobotCommander

rospy.init_node("test")

robot_commander = SrRobotCommander("full_robot")

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
        'interpolate_time': .5
    },
    {
        'name': to_grasp,
        'interpolate_time': .5
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

bracket_trajectory = move_bracket_traj(3,6) + move_bracket_traj(2,5) + move_bracket_traj(1,4) + move_bracket_traj(4,1) + move_bracket_traj(5,2) + move_bracket_traj(6,3)

exhaust_trajectory = (move_exaust_traj(1,2) + move_exaust_traj(2,1)) # *3

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
    }  ,
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

# Run trajectory via moveit

trajectory = baffle_trajectory

robot_commander.run_named_trajectory(trajectory)
