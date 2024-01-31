#import stretch_body.robot
from home_robot_hw.remote import StretchClient
import numpy as np
import PyKDL
import rospy
import sys
import os
#from baxter_kdl.kdl_parser import kdl_tree_from_urdf_model

# sys.path.append("../")
# sys.path.append(os.path.abspath("../"))
from urdf_parser_py.urdf import URDF
from scipy.spatial.transform import Rotation as R
import math
import time
import random
import os
from utils.robot_utils import euler_to_quat, urdf_joint_to_kdl_joint, urdf_pose_to_kdl_frame, urdf_inertial_to_kdl_rbi, kdl_tree_from_urdf_model
from global_parameters import *
import global_parameters


OVERRIDE_STATES = {}

class HelloRobot:

    def __init__(self, urdf_file = 'stretch.urdf', stretch_client_urdf_file = 'hab_stretch/urdf', 
    #def __init__(self, urdf_file = 'hab_stretch/urdf/stretch.urdf', stretch_client_urdf_file = 'hab_stretch/urdf',
            gripper_threshold = 7.0, stretch_gripper_max = 0.3, stretch_gripper_min = 0, end_link = GRIPPER_MID_NODE):
        self.STRETCH_GRIPPER_MAX = stretch_gripper_max
        self.STRETCH_GRIPPER_MIN = stretch_gripper_min
        self.urdf_file = urdf_file
        
        self.urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', self.urdf_file) 
        #self.urdf_path = self.urdf_file
        self.GRIPPER_THRESHOLD = gripper_threshold

        #Initializing ROS node
        print("hello robot starting")
        self.head_joint_list = ["joint_fake", "joint_head_pan", "joint_head_tilt"]
        self.init_joint_list = ["joint_fake","joint_lift","3","2","1" ,"0","joint_wrist_yaw","joint_wrist_pitch","joint_wrist_roll", "joint_gripper_finger_left"]

        # end_link is the frame of reference node 
        # Ex: link_raised_gripper -> camera frame of reference and
        # link_gripper_finger_left -> left gripper finger tip frame of refernce
        self.end_link = end_link 
        self.set_end_link(end_link)
        
        self.robot = StretchClient(urdf_path = stretch_client_urdf_file)
        #self.robot.startup()
        self.robot.switch_to_manipulation_mode()

        # Initializing the robot base position
        #self.base_x = self.robot.nav.get_base_pose()[0]
        #self.base_y = self.robot.nav.get_base_pose()[1]
        time.sleep(2)

        # Constraining the robots movement
        self.clamp = lambda n, minn, maxn: max(min(maxn, n), minn)

        # Joint dictionary for Kinematics
        self.setup_kdl()
        self.initialize_home_params()

    def set_end_link(self, link):
        if link == GRIPPER_FINGERTIP_LEFT_NODE or GRIPPER_FINGERTIP_RIGHT_NODE:
            self.joint_list = self.init_joint_list
        else:
            self.joint_list = self.init_joint_list[:-1]

    def initialize_home_params(self, home_lift = 0.43, home_arm = 0.02, home_base = 0.0, home_wrist_yaw = 0.0, home_wrist_pitch = 0.0, home_wrist_roll = 0.0, home_gripper = 1):
        self.home_lift = home_lift
        self.home_arm = home_arm
        self.home_wrist_yaw = home_wrist_yaw
        self.home_wrist_pitch = home_wrist_pitch
        self.home_wrist_roll = home_wrist_roll
        self.home_gripper = home_gripper
        self.home_base = home_base


    def home(self):
        self.move_to_position(self.home_lift, self.home_arm, self.home_base, self.home_wrist_yaw, self.home_wrist_pitch, self.home_wrist_roll, self.home_gripper)


    def get_joints(self):

        ## Names of all 13 joints
        joint_names = self.init_joint_list + ["joint_gripper_finger_right"] + self.head_joint_list[1:]

        ## updating joint values
        self.updateJoints()
        # print(list(self.joints.values()))
        # print(list(self.head_joints.values()))
        joint_values = list(self.joints.values()) + [0] + list(self.head_joints.values())[1:]

        return joint_names, joint_values


    def setup_kdl(self):
        self.joints = {'joint_fake':0}
        self.head_joints = {'joint_fake':0}
        
        robot_model = URDF.from_xml_file(self.urdf_path)
        self.kdl_tree = kdl_tree_from_urdf_model(robot_model)
        self.arm_chain = self.kdl_tree.getChain('base_link', self.end_link)
        self.joint_array = PyKDL.JntArray(self.arm_chain.getNrOfJoints())

        # Forward kinematics
        self.fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self.arm_chain)
        # Inverse Kinematics
        self.ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self.arm_chain)
        self.ik_p_kdl = PyKDL.ChainIkSolverPos_NR(self.arm_chain, self.fk_p_kdl, self.ik_v_kdl) 
    

    def move_to_position(self, lift_pos = None, arm_pos = None, base_trans = 0.0, wrist_yaw = None, wrist_pitch = None, wrist_roll = None, gripper_pos = None, base_theta = None, head_tilt = None, head_pan = None):

        if base_theta is not None:
            self.robot.nav.navigate_to([0, 0, base_theta])
            return
            
        target_state = self.robot.manip.get_joint_positions()
        #BASE_TRANSLATION = 0
        #LIFT = 1
        #ARM = 2
        #WRIST_YAW = 3
        #WRIST_PITCH = 4
        #WRIST_ROLL = 5
        if not gripper_pos is None:
            self.CURRENT_STATE = gripper_pos*(self.STRETCH_GRIPPER_MAX-self.STRETCH_GRIPPER_MIN)+self.STRETCH_GRIPPER_MIN
            self.robot.manip.move_gripper(self.CURRENT_STATE)
        if not arm_pos is None:
            target_state[2] = arm_pos
        if not lift_pos is None:
            target_state[1] = lift_pos
        if base_trans is None:
            base_trans = 0
        target_state[0] = base_trans + target_state[0]
        if not wrist_yaw is None:
            target_state[3] = wrist_yaw
        if not wrist_pitch is None:
            target_state[4] = min(wrist_pitch, 0.1)
        if not wrist_roll is None:
            target_state[5] = wrist_roll    
        
        velocities = [1]*8
        velocities[5:] = [0.01, 0.01, 0.01, 0.01]
        velocities[0] = 0.01
        self.robot.manip.goto_joint_positions(target_state, relative = False)
        # self.robot.manip.goto(target_state)
        target_head_pan, target_head_tilt = self.robot.head.get_pan_tilt()
        if not head_tilt is None:
            target_head_tilt = head_tilt
        if not head_pan is None:
            target_head_pan = head_pan
        self.robot.head.set_pan_tilt(tilt = target_head_tilt, pan = target_head_pan)
        time.sleep(0.7)

    def pickup(self, depth):
        
        # time.sleep(3)
        # target_state = self.robot.manip.get_joint_positions()
        #BASE_TRANSLATION = 0
        #LIFT = 1
        #ARM = 2
        #WRIST_YAW = 3
        #WRIST_PITCH = 4
        #WRIST_ROLL = 5
        # 0.07 is to correct the little error movement of gripper
        # target_state[2] = target_state[2] + 0.27*depth
        # self.robot.manip.goto_joint_positions(target_state)

        # closing the gripper and picking up
        # print(self.robot.get_status())
        # gripper_pos = self.robot.get_status()['stretch_gripper']['pos'] - 15
        # self.robot.manip.move_gripper(self.STRETCH_GRIPPER_MAX / 6)
        # time.sleep(2)

        next_gripper_pos = 0.25
        while True:
            self.robot.manip.move_gripper(next_gripper_pos)
            # time.sleep(2)

            curr_gripper_pose = self.robot.manip.get_gripper_position()
            # print(curr_gripper_pose)
            if next_gripper_pos == -0.2 or (curr_gripper_pose > next_gripper_pos + 0.01):
                break
            
            if next_gripper_pos > 0:
                next_gripper_pos -= 0.05
            else: 
                next_gripper_pos = -0.2

        # move up
        # target_state = self.robot.manip.get_joint_positions()
        # target_state[1] = target_state[1] + 0.1
        # self.robot.manip.goto_joint_positions(target_state)
        # time.sleep(2)

    def updateJoints(self):
        #Update the joint state values in 'self.joints' using hellorobot api calls
        # print('x, y:', self.robot.base.status['x'], self.robot.base.status['y'])

        state = self.robot.manip.get_joint_positions()
        
        #xyt = self.robot.nav.get_base_pose()
        #origin_dist = math.sqrt((self.base_y - xyt[1])**2+(self.base_x - xyt[0])**2)
        origin_dist = state[0]

        #print(f"init, final co-ordinates - {self.base_x, self.base_y, xyt[0], xyt[1]}")

        # print('orig_dist:', origin_dist)
        # far_dist = math.sqrt((self.far_y - self.robot.base.status['y'])**2+(self.far_x - self.robot.base.status['x'])**2)


        ## commented for debugging
        # print('far dist vals:', far_dist, self.far_to_origin)
        # if(far_dist <= self.far_to_origin):
        ## commented for debugging

        
        self.joints['joint_fake'] = origin_dist
        ## commented for debugging
        # else:
        #     self.joints['joints_fake'] = -1*origin_dist
        ## commented for debugging
        
        #print(f"xyt, origin_dist, state - {xyt}, {origin_dist}, {state}")
        self.joints['joint_lift'] = state[1]
        
        armPos = state[2]
        self.joints['3'] = armPos / 4.0
        self.joints['2'] = armPos / 4.0
        self.joints['1'] = armPos / 4.0
        self.joints['0'] = armPos / 4.0
        
        self.joints['joint_wrist_yaw'] = state[3]
        self.joints['joint_wrist_roll'] = state[5]
        self.joints['joint_wrist_pitch'] = OVERRIDE_STATES.get('wrist_pitch', state[4])

        # self.joints['joint_gripper_finger_left'] = self.robot.end_of_arm.status['stretch_gripper']['pos'] * (0.6/3.4) 
        self.joints['joint_gripper_finger_left'] = 0
        # print("gripper pos - ", self.robot.end_of_arm.status['stretch_gripper']['pos'])

        # Head Joints
        pan, tilt = self.robot.head.get_pan_tilt()
        self.head_joints['joint_fake'] = origin_dist
        self.head_joints['joint_head_pan'] = pan
        self.head_joints['joint_head_tilt'] = tilt

    # following function is used to move the robot to a desired joint configuration 
    def move_to_joints(self, joints, gripper, mode=0, velocities = None):
        # update the robot joints to the new values from 'joints'

        ## the commented code adds a wall on the right side of the robot wrt its starting base position
        #joints['joint_fake'] = self.clamp(joints['joint_fake'], 0.0002, 0.20)


        # print('jt_fk:',joints['joint_fake'])
        # self.base_motion += joints['joint_fake']-self.joints['joint_fake']
        # print('base motion:', self.base_motion)

        # print(f"joints - {joints}")
        
        #BASE_TRANSLATION = 0
        #LIFT = 1
        #ARM = 2
        #WRIST_YAW = 3
        #WRIST_PITCH = 4
        #WRIST_ROLL = 5
        state = self.robot.manip.get_joint_positions()
        joints['joint_wrist_pitch'] = self.clamp(joints['joint_wrist_pitch'], -1.57, 0.1)
        target_state = [
            joints['joint_fake'], 
            joints['joint_lift'],
            joints['3'] + 
            joints['2'] + 
            joints['1'] + 
            joints['0'],
            joints['joint_wrist_yaw'],
            joints['joint_wrist_pitch'],
            joints['joint_wrist_roll']]

        print(f"wrist pitch -{joints['joint_wrist_pitch']}")
        # print(f"velocites: {velocities}")
        if mode == 1:
            # Moving only the lift
            target1 = [0 for _ in range(6)]
            target1[1] = target_state[1] - state[1]
            #self.robot.manip.goto_joint_positions(target1, velocities, relative=True)
            self.robot.manip.goto_joint_positions(target1, relative=True, velocities=velocities)
            time.sleep(0.7)

        elif (mode == 2):
            # Moving base first
            target1 = [0 for _ in range(6)]
            target1[0] = target_state[0]
            self.robot.manip.goto_joint_positions(target1, velocities=velocities)
            time.sleep(0.7)

            # Then move lift
            target1 = [0 for _ in range(6)]
            target1[1] = target_state[1] - state[1]
            self.robot.manip.goto_joint_positions(target1, relative=True, velocities=velocities)
            time.sleep(0.7)

        print(f"current state {state}")
        print(f"target state {target_state}")
        self.robot.manip.goto_joint_positions(target_state, velocities=velocities)
        # self.robot.manip.goto_joint_positions(target_state)
        # self.robot.manip.goto(target_state, velocities)
        time.sleep(0.7)
        # time.sleep(2)
        
        #yaw, pitch, roll limits 
        # self.robot.end_of_arm.move_to('wrist_yaw', self.clamp(joints['joint_wrist_yaw'], -0.4, 1.7))
        # self.robot.end_of_arm.move_to('wrist_pitch', self.clamp(joints['joint_wrist_pitch'], -1.57, 0.2))
        # self.robot.end_of_arm.move_to('wrist_roll', self.clamp(joints['joint_wrist_roll'], -1.53, 1.53))

        #NOTE: belwo code is to fix the pitch drift issue in current hello-robot. Remove it if there is no pitch drift issue
        OVERRIDE_STATES['wrist_pitch'] = joints['joint_wrist_pitch']
        

        # gripper[0] value ranges from 0 to 1, 0 being closed and 1 being open. Below code maps the gripper value to the range of the gripper joint
        # self.CURRENT_STATE  = gripper[0]*(self.STRETCH_GRIPPER_MAX-self.STRETCH_GRIPPER_MIN) + self.STRETCH_GRIPPER_MIN

        # self.robot.manip.move_gripper(self.CURRENT_STATE)
        #code below is to map values below certain threshold to negative values to close the gripper much tighter
        #if self.CURRENT_STATE<self.GRIPPER_THRESHOLD:
        #    self.robot.manip.move_gripper(-0.1)

        #sleeping to make sure all the joints are updated correctly (remove if not necessary)
        # time.sleep(2)

    
    def get_joint_transform(self, node1, node2):
        '''
            This function takes two nodes from a robot URDF file as input and 
            outputs the coordinate frame of node2 relative to the coordinate frame of node1.

            Mainly used for transforming co-ordinates from camera frame to gripper frame.

            It assumes that both node1 and node2 are part of the same branch in the robot tree,
            and node2 appears after node1.
        '''

        # Intializing chain -> maintains list of nodes from base link to corresponding nodes
        chain1 = self.kdl_tree.getChain('base_link', node1)
        chain2 = self.kdl_tree.getChain('base_link', node2)

        # Intializing corresponding joint array and forward chain solvers
        joint_array1 = PyKDL.JntArray(chain1.getNrOfJoints())
        joint_array2 = PyKDL.JntArray(chain2.getNrOfJoints())
        # print(chain1.getNrOfJoints(), chain2.getNrOfJoints())

        fk_p_kdl1 = PyKDL.ChainFkSolverPos_recursive(chain1)
        fk_p_kdl2 = PyKDL.ChainFkSolverPos_recursive(chain2)

        # caluculating current robot joint positions in self.joints
        self.updateJoints()

        if node1 == TOP_CAMERA_NODE:
            ref_joints1 = self.head_joints
            ref_joint1_list = self.head_joint_list
        else:
            ref_joints1 = self.joints
            ref_joint1_list = self.joint_list

        # print("hello")
        # print(ref_joints1, ref_joint1_list)
        # print("hello again")
        # print(self.joints, self.joint_list)
            
        # Updating the joint arrays from self.joints
        for joint_index in range(joint_array1.rows()):
            joint_array1[joint_index] = ref_joints1[ref_joint1_list[joint_index]]
            # print(f"{ref_joint1_list[joint_index]} - {joint_array1[joint_index]}")

        for joint_index in range(joint_array2.rows()):
            joint_array2[joint_index] = self.joints[self.joint_list[joint_index]]
            # print(f"{self.joint_list[joint_index]} - {joint_array2[joint_index]}")
        # joint_array2[joint_array2.rows() - 1] = joint_array2[joint_array2.rows() - 1]/2

        # Intializing frames for corresponding to nodes
        frame1 = PyKDL.Frame()
        frame2 = PyKDL.Frame()

        # Calculating current frames of nodes
        fk_p_kdl1.JntToCart(joint_array1, frame1)
        fk_p_kdl2.JntToCart(joint_array2, frame2)

        # print("frame1")
        # print(frame1)
        # print(frame1.p)
        # print(frame1.M.GetRPY())
        # print("frame2")
        # print(frame2)
        # print(frame2.M)
        # print(frame2.M.GetRPY())
        
        # This allows to transform a point in frame1 to frame2
        frame_transform = frame2.Inverse() * frame1
        # transform1 = self.robot._ros_client.get_frame_pose(node1, node2)
        # transform2 = self.robot._ros_client.get_frame_pose(node2, node1)
        # print(transform1)
        # print(transform2)

        return frame_transform, frame2, frame1
    
    def move_to_pose(self, translation_tensor, rotational_tensor, gripper, move_mode=0, velocities=None):
        

        translation = [translation_tensor[0], translation_tensor[1], translation_tensor[2]]
        rotation = rotational_tensor
        print('translation and rotation', translation_tensor, rotational_tensor)
        
        # move logic
        self.updateJoints()
        print(self.joints)
        
        for joint_index in range(self.joint_array.rows()):
            self.joint_array[joint_index] = self.joints[self.joint_list[joint_index]]
            # print(f"{joint_index} - {self.joint_array[joint_index]}")
        print("\n\n")

        curr_pose = PyKDL.Frame()
        del_pose = PyKDL.Frame()
        self.fk_p_kdl.JntToCart(self.joint_array, curr_pose)

        rot_matrix = R.from_euler('xyz', rotation, degrees=False).as_matrix()
        print("rot_matrix", rot_matrix)


#new code from here
        del_rot = PyKDL.Rotation(PyKDL.Vector(rot_matrix[0][0], rot_matrix[1][0], rot_matrix[2][0]),
                                  PyKDL.Vector(rot_matrix[0][1], rot_matrix[1][1], rot_matrix[2][1]),
                                  PyKDL.Vector(rot_matrix[0][2], rot_matrix[1][2], rot_matrix[2][2]))
        # del_rot = rotation_matrix
        del_trans = PyKDL.Vector(translation[0], translation[1], translation[2])
        del_pose.M = del_rot
        del_pose.p = del_trans
        goal_pose_new = curr_pose*del_pose
        # print("cur pose - ", curr_pose )
        # print("del_pose - ", del_pose)
        # print("goal pose - ", goal_pose_new)

        # correction in final x, y, z postions
        # print(f"corrections - {CORRECTION_X, CORRECTION_Y, CORRECTION_Z}")
        print(f"corrections - {global_parameters.CORRECTION_X, global_parameters.CORRECTION_Y, CORRECTION_Z}")
        goal_pose_new.p[0] = goal_pose_new.p[0] + global_parameters.CORRECTION_X
        goal_pose_new.p[1] = goal_pose_new.p[1] + global_parameters.CORRECTION_Y
        goal_pose_new.p[2] = goal_pose_new.p[2] + global_parameters.CORRECTION_Z
        # print("goal pose - ", goal_pose_new)        

        seed_array = PyKDL.JntArray(self.arm_chain.getNrOfJoints())
        self.ik_p_kdl.CartToJnt(seed_array, goal_pose_new, self.joint_array)

        ik_joints = {}
        print("self.joint_array: ", self.joint_array)
        
        print("joint array: ", self.joint_array)
        print(f"joint array length -{self.joint_array.rows()}")
        for joint_index in range(self.joint_array.rows()):
            ik_joints[self.joint_list[joint_index]] = self.joint_array[joint_index]

        print('ik_joints', ik_joints)
        # test_pose = PyKDL.Frame()
        # self.fk_p_kdl.JntToCart(self.joint_array, test_pose)

        # # print(test_pose.p)
        # # print(test_pose.M.GetRPY())

        print("Move to:", ik_joints)
        self.move_to_joints(ik_joints, gripper, move_mode, velocities)
        # time.sleep(2)

        self.updateJoints()
        for joint_index in range(self.joint_array.rows()):
            self.joint_array[joint_index] = self.joints[self.joint_list[joint_index]]
            # print(f"{joint_index} - {self.joint_array[joint_index]}")
        



