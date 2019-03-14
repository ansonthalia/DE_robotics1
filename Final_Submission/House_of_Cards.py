#!/usr/bin/env python

"""
Baxter RSDK House of Cards structure builder.
"""

import argparse
import struct
import sys
import copy
import time
import numpy as np
from tf.transformations import *
from Colour_detect import*

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

# importing all of our code which has been divided into scripts
# all scripts must be in the same directory as this file
from Model_Spawn import *
from House_Builder import *
from Pick_n_Place import *
from Control_Functions import *

def main():
    """Team House of Cards
    We pull on a variety of our own code writen in the House_Builder,
    Model_Spawn, Pick_n_Place, and Control_Functions in order excecute
    the actions in this main function and therfore build the House
    of Cards structure. Modifying inputs as explained in comments allows
    anyone customise the results of their robot build in accordance with
    their environment.
    """

    # Intialise a node to communicate with the master node and therefore DENIRO
    rospy.init_node("ik_house_of_cards")

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    # Starting Joint angles for both arms calculated based on our environment
    # and the baxter robot arm specifics they can easily be modified with
    # reference to http://sdk.rethinkrobotics.com/wiki/Hardware_Specifications

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
    # See the Pick_n_Place.py file for deatils on the class
    hocl = PickAndPlace('left')
    hocr = PickAndPlace('right')

    # An orientation for gripper to be above and parallel to the bricks
    v_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)

    # Solving for second quaternion
    orig = np.array([-0.0249590815779,
                        0.999649402929,
                        0.00737916180073,
                        0.00486450832011])

    quat = quaternion_multiply(quaternion_from_euler(0,0,1.57),orig)

    # An orientation for gripper to be above and perpendicular to the bricks
    h_orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    # A predeterminded pickup location for the four brick locations:
    # Left upright, right upright, left flat, right flat. All positions are
    # reltive to DENIRO's torso which is at (0, 0, 0.63) relative to the world
    lu_pick = Pose(
        position=Point(x=0.100, y=0.7, z=0.23),
        orientation=v_orientation)
    ru_pick = Pose(
        position=Point(x=0.1, y=-0.7, z=0.23),
        orientation=v_orientation)
    lf_pick = Pose(
        position=Point(x=0.3, y=0.7, z=0.13),
        orientation=h_orientation)
    rf_pick = Pose(
        position=Point(x=0.3, y=-0.7, z=0.13),
        orientation=h_orientation)

    # creating the target brick postiions for DENIRO, to see details of the
    # function arguments look in House_Bilder.py
    block_poses = posify(house_coordinates(0.5, 0, 0.1, 1s.5, 1.5),
                                            v_orientation, h_orientation)

    # move arms to the desired starting angles
    hocl.move_to_start(left_start)
    hocr.move_to_start(right_start)

    # spawn environment which is made relative to our project and context
    # please see Model_Spawn to make modifications
    load_tables()

    # wait for tables to spawn
    time.sleep(3)

    # loop to pick and place the entire structure i counts through the levels
    # and n counts the number of bricks placed for spawning puproses
    # please refer to Model_Spawn and Pick_n_Place for functions used to
    # control robot actions
    i = 0
    n = 1

    # variables for error checking please see Control_Functions for more details
    failed = False
    old_per = 0

    while not rospy.is_shutdown() & i > len(block_poses): # iterate layers
        for j in range(len(block_poses[i])): # each brick in a row
            if i % 2: # determine if it's a horizontal or vertical row
                # using image detection to see if the structure has been
                # knocked over by ensuring the red of the image is increasing
                new_per = colour_detect()
                if old_per < new_per:
                    old_per = new_per
                else:
                    failed = True
                    break

                print("\nHorizontal block row")
                if j % 2: # determine if we should use left or right hand
                    print("\nUsing left")
                    load_Flat(n,'l') # spawn a flat brick on the left
                    print("\nPicking...")
                    hocl.pick(lf_pick) # pick up from the left flat brick spot
                    print("\nPlacing...")
                    hocl.place(block_poses[i][j]) # place into structure
                    print("Returning to start...")
                    hocl.move_to_start(left_start) # return to start
                    n+=1
                else:
                    print("\nUsing right")
                    load_Flat(n,'r') # spawn a flat brick on the right
                    print("\nPicking...")
                    hocr.pick(rf_pick) # pick up from the right flat brick spot
                    print("\nPlacing...")
                    hocr.place(block_poses[i][j]) # place into structure
                    print("Returning to start...")
                    hocr.move_to_start(right_start) # return to start
                    n+=1
            else:
                # using image detection to see if the structure has been
                # knocked over by ensuring the red of the image is increasing
                new_per = colour_detect()
                if old_per < new_per:
                    old_per = new_per
                else:
                    failed = True
                    break

                print("\nVertical block row")
                if j % 2: # determine if we should use left or right hand
                    print("\nUsing left")
                    load_UP(n,'l') # spawn a upright brick on the left
                    print("\nPicking...")
                    hocl.pick(lu_pick) # pick up from the left upright brick spot
                    print("\nPlacing...")
                    hocl.place(block_poses[i][j]) # place into structure
                    print("Returning to start...")
                    hocl.move_to_start(left_start) # return to start
                    n+=1
                else:
                    print("\nUsing right")
                    load_UP(n,'r') # pawn a upright brick on the right
                    print("\nPicking...")
                    hocr.pick(ru_pick) # pick up from the right upright brick spot
                    print("\nPlacing...")
                    hocr.place(block_poses[i][j]) # place into structure
                    print("Returning to start...")
                    hocr.move_to_start(right_start) # return to start
                    n+=1
            j += 1
        i += 1
    if failed:
        print("OHH MANNNN! Not again.")
    return


if __name__ == '__main__':
    sys.exit(main())
