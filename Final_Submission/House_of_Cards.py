#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import time
import numpy as np
from tf.transformations import *

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

from Model_Spawn import *
from House_Builder import *
from Pick_n_Place import *

def main():
    """Team House of Cards
    A modification of the  the Pick and Place example which uses the
    Rethink Inverse Kinematics Service which returns the joint angles a
    requested Cartesian Pose. This ROS Service client is used to request
    both pick and place poses in the /base frame of the robot.
    """
    rospy.init_node("ik_house_of_cards")

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    # Starting Joint angles for both arms
    left_start = {'left_w0': 0.6699952259595108,
                    'left_w1': 1.030009435085784,
                    'left_w2': -0.4999997247485215,
                    'left_e0': -1.189968899785275,
                    'left_e1': 0.6700238130755056,
                    'left_s0': 0.18000397926829805,
                    'left_s1': -0.9999781166910306}
    right_start = {'right_w0': -0.6699952259595108,
                    'right_w1': 1.030009435085784,
                    'right_w2': 0.4999997247485215,
                    'right_e0': 1.189968899785275,
                    'right_e1': 0.6700238130755056,
                    'right_s0': -0.18000397926829805,
                    'right_s1': -0.9999781166910306}

    # Two pick and place opjects for each of DENIRO's arms
    hocl = PickAndPlace('left')
    hocr = PickAndPlace('right')

    # An orientation for gripper to be above and parallel to the bricks
    v_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)

    # solving for second quaternion
    orig = np.array([-0.0249590815779,
                        0.999649402929,
                        0.00737916180073,
                        0.00486450832011])

    quat = quaternion_multiply(quaternion_from_euler(0,0,1.57),orig)

    # An orientation for gripper to be above and perpendicular to the bricks
    h_orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    lv_pick = Pose(
        position=Point(x=0.100, y=0.7, z=0.23),
        orientation=v_orientation)
    rv_pick = Pose(
        position=Point(x=0.1, y=-0.7, z=0.23),
        orientation=v_orientation)
    lh_pick = Pose(
        position=Point(x=0.3, y=0.7, z=0.13),
        orientation=h_orientation)
    rh_pick = Pose(
        position=Point(x=0.3, y=-0.7, z=0.13),
        orientation=h_orientation)

    # base and height for the generated house of cards
    base = 3
    height = 1

    block_poses = posify(house_coordinates(0.5, 0, 0.1, 5, 3), v_orientation, h_orientation)

    # move to the desired starting angles
    hocl.move_to_start(left_start)
    hocr.move_to_start(right_start)

    # spawn environment
    load_tables()
    time.sleep(3)

    # loop to pick and place the entire structure
    i = 0
    n = 1

    while not rospy.is_shutdown() & i > len(block_poses):
        for j in range(len(block_poses[i])):
            if i % 2:
                print("\nHorizontal block row")
                if j % 2:
                    print("\nUsing left")
                    load_Flat(n,'l')
                    print("\nPicking...")
                    hocl.pick(lh_pick)
                    print("\nPlacing...")
                    hocl.place(block_poses[i][j])
                    print("Returning to start...")
                    hocl.move_to_start(left_start)
                    n+=1
                else:
                    print("\nUsing right")
                    load_Flat(n,'r')
                    print("\nPicking...")
                    hocr.pick(rh_pick)
                    print("\nPlacing...")
                    hocr.place(block_poses[i][j])
                    print("Returning to start...")
                    hocr.move_to_start(right_start)
                    n+=1
            else:
                print("\nVertical block row")
                if j % 2:
                    print("\nUsing left")
                    load_UP(n,'l')
                    print("\nPicking...")
                    hocl.pick(lv_pick)
                    print("\nPlacing...")
                    hocl.place(block_poses[i][j])
                    print("Returning to start...")
                    hocl.move_to_start(left_start)
                    n+=1
                else:
                    print("\nUsing right")
                    load_UP(n,'r')
                    print("\nPicking...")
                    hocr.pick(rv_pick)
                    print("\nPlacing...")
                    hocr.place(block_poses[i][j])
                    print("Returning to start...")
                    hocr.move_to_start(right_start)
                    n+=1
            j += 1
        i += 1
    return

if __name__ == '__main__':
    sys.exit(main())
