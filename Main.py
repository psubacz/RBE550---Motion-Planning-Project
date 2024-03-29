#!/usr/bin/env python
# To run hte tim, open a terminal and enter:
# $ ./baxter.sh sim
# $ roslaunch baxter_gazebo baxter_world.launch

# Open a new terminal and enter:
# $ ./baxter.sh sim
# $ rosrun baxter_tools enable_robot.py -e
# $ rosrun baxter_sim_examples ik_pick_and_place_demo.py

# table is centered at at (1,0,0) and is approx 0.75 units tall

# 0------1
# |      |
# |      |
# 3------2
#     B

#B = baxter located at (0,0,0)
# 0 = is the top left corner at (1.4,.4) 
# 1 = is the top right corner at (1.4,-0.45) 
# 2 = is the bottom left corner at (0.55,-0.45)
# 3 = is the bottom left corner at (0.55,0.5)

"""
RBE 550 Baxter Rapidly Exploring Random Trees with Inverse Kinematics for Path planning Project
"""
import argparse
import struct
import sys
import copy
import os
import rospy
import rospkg
import time
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
import baxter_external_devices
from baxter_interface import CHECK_VERSION

class BaxterOperator(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        self.gripper_open()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to waypoint pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        rospy.sleep(0.5)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()

    def gripper_close(self):
        self._gripper.close()

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        # approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def GetCurrentLimbPose(self):
        return self._limb.endpoint_pose()

def load_gazebo_models(table_pose=Pose(position=Point(x=1.215, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.8, y=0.4, z=0.75)),
                       block_reference_frame="world",
                       block1_pose=Pose(position=Point(x=1.1975, y=0.0, z=0.75)),
                       block1_reference_frame="world",
                       bar_pose=Pose(position=Point(x=0.875, y=-0.34, z=0.75)),
                       bar_reference_frame="world",):


    # table is centered at at (1,0,0) and is approx 0.75 units tall

    # 0------1
    # |      |
    # |      |
    # 3------2
    #     B

    #B = baxter located at (0,0,0)
    # 0 = is the top left corner at (1.4,.4) 
    # 1 = is the top right corner at (1.4,-0.45) 
    # 2 = is the bottom left corner at (0.55,-0.45)
    # 3 = is the bottom left corner at (0.55,0.5)

    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')

    #Load Block1 URDF
    block_xml1 = ''
    with open (model_path + "block1/model.urdf", "r") as block_file:
        block_xml1=block_file.read().replace('\n', '')  

    #Load bar URDF
    bar_xml = ''
    with open (model_path + "bar/model.urdf", "r") as block_file:
        bar_xml=block_file.read().replace('\n', '')  

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    # Spawn Block1 URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block1", block_xml1, "/",
                               block1_pose, block1_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn bar URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("bar", bar_xml, "/",
                               bar_pose, bar_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    print("Waiting for sim to settle...")
    time.sleep(5)

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
        resp_delete = delete_model("block1")
        resp_delete = delete_model("bar")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))




###
# This function allows for keyboard overrides were placed on the baxter robot. 
###
def map_keyboard():
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)

    bindings = {
    #   key: (function, args, description)
        'q': (set_j, [left, lj[0], 0.01], "left_s0 increase"),
        'a': (set_j, [left, lj[0], -0.01], "left_s0 decrease"),
        'w': (set_j, [left, lj[1], 0.01], "left_s1 increase"),
        's': (set_j, [left, lj[1], -0.01], "left_s1 decrease"),
        'e': (set_j, [left, lj[2], 0.01], "left_e0 increase"),
        'd': (set_j, [left, lj[2], -0.01], "left_e0 decrease"),
        'r': (set_j, [left, lj[3], 0.01], "left_e1 increase"),
        'f': (set_j, [left, lj[3], -0.01], "left_e1 decrease"),
        't': (set_j, [left, lj[4], 0.01], "left_w0 increase"),
        'g': (set_j, [left, lj[4], -0.01], "left_w0 decrease"),
        'y': (set_j, [left, lj[5], 0.01], "left_w1 increase"),
        'h': (set_j, [left, lj[5], -0.01], "left_w1 decrease"),
        'u': (set_j, [left, lj[6], 0.01], "left_w2 increase"),
        'j': (set_j, [left, lj[6], -0.01], "left_w2 decrease"),
        'i': (grip_left.close, [], "left: gripper close"),
        'k': (grip_left.open, [], "left: gripper open"),
        'o': (grip_left.calibrate, [], "left: gripper calibrate"),

        'z': (set_j, [right, rj[0], 0.01], "right_s0 increase"),
        'x': (set_j, [right, rj[0], -0.01], "right_s0 decrease"),
        'c': (set_j, [right, rj[1], 0.01], "right_s1 increase"),
        'v': (set_j, [right, rj[1], -0.01], "right_s1 decrease"),
        'b': (set_j, [right, rj[2], 0.01], "right_e0 increase"),
        'n': (set_j, [right, rj[2], -0.01], "right_e0 decrease"),
        'm': (set_j, [right, rj[3], 0.01], "right_e1 increase"),
        ',': (set_j, [right, rj[3], -0.01], "right_e1 decrease"),
        '.': (set_j, [right, rj[4], 0.01], "right_w0 increase"),
        '/': (set_j, [right, rj[4], -0.01], "right_w0 decrease"),
        '7': (set_j, [right, rj[5], 0.01], "right_w1 increase"),
        '4': (set_j, [right, rj[5], -0.01], "right_w1 decrease"),
        '8': (set_j, [right, rj[6], 0.01], "right_w2 increase"),
        '5': (set_j, [right, rj[6], -0.01], "right_w2 decrease"),
        '9': (grip_right.close, [], "right: gripper close"),
        '6': (grip_right.open, [], "right: gripper open"),
        '2': (grip_right.calibrate, [], "right: gripper calibrate"),
     }
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                # rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                #expand binding to something like "set_j(right, 's0', 0.1)"
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
                # done = True
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))

def main():
    """


    """
    epilog ="""
    See help inside the example with the '?' key for key bindings.
    """

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
            
    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command) 

    print("Initializing node... ")
    rospy.init_node("rbe550_project")\

    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)

    print('Loading Gazebo Models')
    #Load gazebo models into the simulation enviroment

    load_gazebo_models()
    # add ros shutdown function to remove models from gazebo

    rospy.on_shutdown(delete_gazebo_models)

    # Wait for the models to load and sim to settle
    rospy.wait_for_message("/robot/sim/started", Empty)
    print ('Models loaded')

    
    print("Enabling robot... ")
    init_state = rs.state().enabled

    #Limbs are indepantly controlled by the code, each limb will need have have IK 
    #Calculated and controller each loop of the code. 
    limb1 = 'left'
    limb2 = 'right'

    hover_distance = 0.05 # meters                             
        
    ##Create left arm object
    left_arm = BaxterOperator(limb1, hover_distance)
    right_arm = BaxterOperator(limb2, hover_distance)

    #Set starting joint angles in radians
    r_starting_joint_angles = {'right_s0': 0.2647628147879759,
                             'right_s1': -1.1444878789648056,
                             'right_e0': 0.6217436157112814,
                             'right_e1': 1.6195685462509486,
                             'right_w0': -1.8714029189324384,
                             'right_w1': 0.21796969443362446,
                             'right_w2': -1.5387128606809188}

    r_waypoint_1 = {'right_s0': 0.2677265646844633,
                             'right_s1': -1.025645542177278,
                             'right_e0': 0.6019796530505088,
                             'right_e1': 1.3843463058717722,
                             'right_w0': -0.19968377284189653,
                             'right_w1': 0.5931122221437324,
                             'right_w2': 0.13245042899859794}

    r_waypoint_2 = {'right_s0': 0.3535399031650135,
                             'right_s1': -0.7642200371603014,
                             'right_e0': 0.5626981112717457,
                             'right_e1': 1.3449704741553727,
                             'right_w0': -0.4986121103279588,
                             'right_w1': 0.7508163817062323,
                             'right_w2': 0.13237404399365804}

    r_waypoint_3 = {'right_s0': 0.1828244934776766,
                             'right_s1': -1.4831071761021075,
                             'right_e0': 0.38507416602953626,
                             'right_e1': 1.3095578952791218,
                             'right_w0': 0.7223324571368508,
                             'right_w1': 0.7526735759808059,
                             'right_w2': 0.1324723456965078}

    r_waypoint_4 = {'right_s0': 0.4063774734573835,
                             'right_s1': -0.8527875478321185,
                             'right_e0': 0.47422613162492677,
                             'right_e1': 0.9292457974090382,
                             'right_w0': 1.5083830288720668,
                             'right_w1': 0.8284823668873225,
                             'right_w2': 0.3216264927729684}

    l_waypoint_1 = {'left_s0': -0.4065063038109402,
                             'left_s1': -0.22587672844830475,
                             'left_e0': -1.936441973578404,
                             'left_e1': 0.23011851032745323,
                             'left_w0': 0.270824064278286,
                             'left_w1': 0.9310489948600846,
                             'left_w2': -0.07492474279767425}



    # right_arm.move_to_start(r_starting_joint_angles)
    #
    # map_keyboard()
    # right_arm.move_to_start(r_waypoint_1)
    # map_keyboard()
    # right_arm.move_to_start(r_waypoint_2)
    # map_keyboard()
    # right_arm.move_to_start(r_waypoint_3)
    # map_keyboard()
    # right_arm.move_to_start(r_waypoint_4)
    # map_keyboard()
    # left_arm.move_to_start(l_waypoint_1)
    # map_keyboard()
    # left_arm.move_to_start(starting_joint_angles)

    while not rospy.is_shutdown():

        #get first node of the solution tree
        #extract coordinate soluiton
        #get limb pose
        #calcaluate inverse kinematics and get new joint position

        #Manual Override as required. 
        active_limb = 'right'
        map_keyboard()
        xx = input('\nx: ')
        yy = input('\ny: ')
        zz = input('\nz: ')

        #Look at nodes limb tag 
        if (active_limb == 'right'):
            print('Right Limb Moving')
            #Get the current pose of the limb 
            curr_limb_pose = right_arm.GetCurrentLimbPose()
            #set new pose base on new (X,Y,Z) coord position
            new_limb_pose = Pose(
                position=Point(x=xx, y=yy, z=zz),
                orientation=Quaternion(
                                    x=curr_limb_pose['orientation'].x,
                                    y=curr_limb_pose['orientation'].y,
                                    z=curr_limb_pose['orientation'].z,
                                    w=curr_limb_pose['orientation'].w))
            print('\nMoving limb to position...')
            #Calculate inverse kinematics and move to joint poisition
            right_arm._approach(new_limb_pose)
        elif(active_limb == 'left'):
            print('Left Limb Moving') 
            #Get the current pose of the limb
            curr_limb_pose = left_arm.GetCurrentLimbPose()
            #set new pose base on new (X,Y,Z) coord position
            new_limb_pose = Pose(
                position=Point(x=0.875, y=0.15, z=0.8),
                orientation=Quaternion(
                                    x=curr_limb_pose['orientation'].x,
                                    y=curr_limb_pose['orientation'].y,
                                    z=curr_limb_pose['orientation'].z,
                                    w=curr_limb_pose['orientation'].w))
             #Calculate inverse kinematics and move to joint poisition
            print('\nMoving limb to position...')
            left_arm._approach(new_limb_pose)
        
        rospy.sleep(0.1)

    
    #Call clean shutdown procedures
    rospy.on_shutdown(clean_shutdown)

    return 0

if __name__ == '__main__':
    sys.exit(main())
