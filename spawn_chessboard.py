#!/usr/bin/env python
import pick_and_place_moveit
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy


# http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials
import sys
import copy
import tf2_ros

import tf

import rospy
import rospkg

import moveit_commander
import moveit_msgs

import time


from std_msgs.msg import (
    Empty,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

import baxter_interface
import moveit_commander

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ik_pick_and_place_moveit")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    #load_gazebo_models()
    # Remove models from the scene on shutdown
    #rospy.on_shutdown(delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    #piece_pos_recoder = rospy.get_param('piece_pos_recoder')
    #piece_target_position_map =rospy.get_param('piece_target_position_map')

    #print(type(piece_pos_recoder['R0'][0]))

    limb = 'left'
    hover_distance = 0.15  # meters

    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    # NOTE: Gazebo and Rviz has different origins, even though they are connected. For this
    # we need to compensate for this offset which is 0.93 from the ground in gazebo to
    # the actual 0, 0, 0 in Rviz.

    starting_pose = Pose(
        position=Point(x=0.7, y=0.135, z=0.35),
        orientation=overhead_orientation)
    pnp = pick_and_place_moveit.PickAndPlaceMoveIt(limb, hover_distance)
    #rospy.init_node("spawn_chessboard")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    srv_call = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    pnp.move_to_start(starting_pose)
    
    # Table
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    table_xml = ''
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace('\n', '')

    table_pose=Pose(position=Point(x=0.73, y=0.4, z=0.0))
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_sdf("cafe_table", table_xml, "/", table_pose, "world")
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    
    
    # ChessBoard
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    board_pose = Pose(Point(0.3,0.55,0.78), orient)
    frame_dist = 0.025
    model_path = rospkg.RosPack().get_path('chess_baxter')+"/models/"
    
    with open(model_path + "chessboard/model.sdf", "r") as f:
        board_xml = f.read().replace('\n', '')

    # Add chessboard into the simulation
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    print srv_call("chessboard", board_xml, "", board_pose, "world")

    # Add chesspieces into the simulation
   
    origin_piece = 0.03125

    pieces_xml = dict()
    list_pieces = 'rnbqkpRNBQKP'
    for each in list_pieces:
        with open(model_path + each+".sdf", "r") as f:
            pieces_xml[each] = f.read().replace('\n', '')

    #board_setup = ['rnbqkbnr', 'pppppppp', '', '', '', '', 'PPPPPPPP', 'RNBQKBNR']
    board_setup = ['r******r', '', '**k*****', '', '', '******K*', '', 'R******R']
    

    piece_positionmap = dict()
    piece_names = []
    piece_pos_recoder = dict()
    
    for row, each in enumerate(board_setup):
        # print row
        for col, piece in enumerate(each):
            pose_st=Pose(Point(x=0.53127, y=0.68, z=0.798),overhead_orientation)
            pose_st1 = Pose(Point(x=0.53127, y=0.68, z=0.798-0.94),overhead_orientation)
            pose = deepcopy(board_pose)
            pose.position.x = board_pose.position.x + frame_dist + origin_piece + row * (2 * origin_piece)
            pose.position.y = board_pose.position.y - 0.55 + frame_dist + origin_piece + col * (2 * origin_piece)
            pose.position.z += 0.018

            piece_positionmap[str(row)+str(col)] = [pose.position.x, pose.position.y, pose.position.z] 
            position = Pose(Point(pose.position.x, pose.position.y, pose.position.z-0.93),overhead_orientation)

            if piece in list_pieces:
                piece_names.append("%s%d" % (piece,col))
                temp = "%s%d" % (piece,col)
                rospy.wait_for_service('/gazebo/spawn_sdf_model')
                print srv_call(temp,pieces_xml[piece],"",pose_st,"world")
                piece_pos_recoder[temp] = [pose.position.x, pose.position.y, pose.position.z]
                pnp.move_to_start(starting_pose)
                print("\nPicking...")
                pnp.pick(pose_st1)
                #pnp.move_to_start(starting_pose)
                print("\nPlacing...")
                pnp.place(position)
                print("\nDone")




    rospy.set_param('board_setup', board_setup) # Board setup
    rospy.set_param('list_pieces', list_pieces) # List of unique pieces
    rospy.set_param('piece_target_position_map', piece_positionmap) # 3D positions for each square in the chessboard
    rospy.set_param('piece_names', piece_names) # Pieces that will be part of the game
    rospy.set_param('pieces_xml', pieces_xml) # File paths to Gazebo models, i.e. SDF files
    rospy.set_param('piece_pos_recoder',piece_pos_recoder)
    # print(type(piece_positionmap))
    #rospy.set_param('')
