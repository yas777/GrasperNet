import stretch_body.robot
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

    def __init__(self, urdf_file = 'stretch_nobase_raised.urdf', gripper_threshold = 7.0, stretch_gripper_max = 60, stretch_gripper_min = 0, end_link = "link_raised_gripper"):
        
        self.STRETCH_GRIPPER_MAX = stretch_gripper_max
        self.STRETCH_GRIPPER_MIN = stretch_gripper_min
        self.urdf_file = urdf_file
        
        self.urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', self.urdf_file) 
        self.GRIPPER_THRESHOLD = gripper_threshold

        #Initializing ROS node
        print("hello robot starting")
        self.head_joint_list = ["joint_fake", "joint_head_pan", "joint_head_tilt"]
        self.init_joint_list = ["joint_fake","joint_lift","joint_arm_l3","joint_arm_l2","joint_arm_l1" ,"joint_arm_l0","joint_wrist_yaw","joint_wrist_pitch","joint_wrist_roll", "joint_gripper_finger_left"]

        # end_link is the frame of reference node 
        # Ex: link_raised_gripper -> camera frame of reference and
        # link_gripper_finger_left -> left gripper finger tip frame of refernce
        self.end_link = end_link 
        self.set_end_link(end_link)
        
        self.robot = stretch_body.robot.Robot()
        self.robot.startup()

        # Initializing the robot base position
        self.base_x = self.robot.base.status['x']
        self.base_y = self.robot.base.status['y']

    
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
    

    def move_to_position(self, lift_pos = None, arm_pos = None, base_trans = 0.0, wrist_yaw = None, wrist_pitch = None, wrist_roll = None, gripper_pos = None):

        if gripper_pos != None:
            self.CURRENT_STATE = self.STRETCH_GRIPPER_MAX if gripper_pos is None \
                             else gripper_pos*(self.STRETCH_GRIPPER_MAX-self.STRETCH_GRIPPER_MIN)+self.STRETCH_GRIPPER_MIN
            self.robot.end_of_arm.move_to('stretch_gripper',self.CURRENT_STATE)
            self.robot.push_command()
            time.sleep(2)
        
        if lift_pos != None:
            self.robot.lift.move_to(lift_pos)
            self.robot.push_command()
            time.sleep(2)
        
        if arm_pos != None:
            while self.robot.get_status()['arm']['pos']>arm_pos+0.002 or self.robot.get_status()['arm']['pos']<arm_pos-0.002:
                # print(self.robot.get_status()['arm']['pos'])
                self.robot.arm.move_to(arm_pos)
                self.robot.push_command()
                time.sleep(2)

        if wrist_pitch != None or wrist_roll != None != wrist_yaw != None:
            self.robot.end_of_arm.move_to('wrist_yaw', wrist_yaw)
            PITCH_VAL = wrist_pitch
            self.robot.end_of_arm.move_to('wrist_pitch', PITCH_VAL)
            #NOTE: belwo code is to fix the pitch drift issue in current hello-robot. Remove it if there is no pitch drift issue
            OVERRIDE_STATES['wrist_pitch'] = PITCH_VAL  
            self.robot.end_of_arm.move_to('wrist_roll', wrist_roll)
            self.robot.base.translate_by(base_trans)
            print('moving to position 3')
            self.robot.push_command()
            time.sleep(2)
            print('moving to position 4')

    def pickup(self, depth):
        
        time.sleep(3)
        # 0.07 is to correct the little error movement of gripper
        arm_pos = self.robot.get_status()['arm']['pos'] + 0.27*depth
        while self.robot.get_status()['arm']['pos']>arm_pos+0.002 or self.robot.get_status()['arm']['pos']<arm_pos-0.002:
            # print(self.robot.get_status()['arm']['pos'])
            self.robot.arm.move_to(arm_pos)
            self.robot.push_command()
            time.sleep(2) 

        # closing the gripper and picking up
        # print(self.robot.get_status())
        # gripper_pos = self.robot.get_status()['stretch_gripper']['pos'] - 15
        self.robot.end_of_arm.move_to('stretch_gripper', 10)
        self.robot.push_command()
        time.sleep(2)

        # move up
        lift_pos = self.robot.get_status()['lift']['pos'] + 0.1
        self.robot.lift.move_to(lift_pos)
        self.robot.push_command()
        time.sleep(2)

    def updateJoints(self):
        #Update the joint state values in 'self.joints' using hellorobot api calls
        # print('x, y:', self.robot.base.status['x'], self.robot.base.status['y'])

        origin_dist = math.sqrt((self.base_y - self.robot.base.status['y'])**2+(self.base_x - self.robot.base.status['x'])**2)

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
        
        self.joints['joint_lift'] = self.robot.lift.status['pos']
        
        armPos = self.robot.arm.status['pos']
        self.joints['joint_arm_l3'] = armPos / 4.0
        self.joints['joint_arm_l2'] = armPos / 4.0
        self.joints['joint_arm_l1'] = armPos / 4.0
        self.joints['joint_arm_l0'] = armPos / 4.0
        
        self.joints['joint_wrist_yaw'] = self.robot.end_of_arm.status['wrist_yaw']['pos']
        self.joints['joint_wrist_roll'] = self.robot.end_of_arm.status['wrist_roll']['pos']
        self.joints['joint_wrist_pitch'] = OVERRIDE_STATES.get('wrist_pitch', self.robot.end_of_arm.status['wrist_pitch']['pos'])

        # self.joints['joint_gripper_finger_left'] = self.robot.end_of_arm.status['stretch_gripper']['pos'] * (0.6/3.4) 
        self.joints['joint_gripper_finger_left'] = 0
        # print("gripper pos - ", self.robot.end_of_arm.status['stretch_gripper']['pos'])

        # Head Joints
        self.head_joints['joint_fake'] = origin_dist
        self.head_joints['joint_head_pan'] = self.robot.head.status['head_pan']['pos']
        self.head_joints['joint_head_tilt'] = self.robot.head.status['head_tilt']['pos']

    # following function is used to move the robot to a desired joint configuration 
    def move_to_joints(self, joints, gripper):
        # update the robot joints to the new values from 'joints'

        ## the commented code adds a wall on the right side of the robot wrt its starting base position
        #joints['joint_fake'] = self.clamp(joints['joint_fake'], 0.0002, 0.20)


        # print('jt_fk:',joints['joint_fake'])
        # self.base_motion += joints['joint_fake']-self.joints['joint_fake']
        # print('base motion:', self.base_motion)

        # print(f"joints - {joints}")
        self.robot.base.translate_by(joints['joint_fake']-self.joints['joint_fake'], 5)
        self.robot.lift.move_to(joints['joint_lift'])
        self.robot.push_command()
        time.sleep(2)
        
        self.robot.arm.move_to(joints['joint_arm_l3'] + 
                            joints['joint_arm_l2'] + 
                            joints['joint_arm_l1'] + 
                            joints['joint_arm_l0'])
        
        
        
        #yaw, pitch, roll limits 
        self.robot.end_of_arm.move_to('wrist_yaw', self.clamp(joints['joint_wrist_yaw'], -0.4, 1.7))
        self.robot.end_of_arm.move_to('wrist_pitch', self.clamp(joints['joint_wrist_pitch'], -1.57, 0.2))
        #NOTE: belwo code is to fix the pitch drift issue in current hello-robot. Remove it if there is no pitch drift issue
        OVERRIDE_STATES['wrist_pitch'] = joints['joint_wrist_pitch']
        self.robot.end_of_arm.move_to('wrist_roll', self.clamp(joints['joint_wrist_roll'], -1.53, 1.53))

        # gripper[0] value ranges from 0 to 1, 0 being closed and 1 being open. Below code maps the gripper value to the range of the gripper joint
        self.CURRENT_STATE  = gripper[0]*(self.STRETCH_GRIPPER_MAX-self.STRETCH_GRIPPER_MIN) + self.STRETCH_GRIPPER_MIN

        self.robot.end_of_arm.move_to('stretch_gripper', self.CURRENT_STATE)
        #code below is to map values below certain threshold to negative values to close the gripper much tighter
        if self.CURRENT_STATE<self.GRIPPER_THRESHOLD:
            self.robot.end_of_arm.move_to('stretch_gripper', -25)

  
        self.robot.push_command()

        #sleeping to make sure all the joints are updated correctly (remove if not necessary)
        time.sleep(.7)

    
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

        return frame_transform, frame2, frame1
    
    def move_to_pose(self, translation_tensor, rotational_tensor, gripper):
        

        translation = [translation_tensor[0], translation_tensor[1], translation_tensor[2]]
        rotation = rotational_tensor
        
        # move logic
        self.updateJoints()

        for joint_index in range(self.joint_array.rows()):
            self.joint_array[joint_index] = self.joints[self.joint_list[joint_index]]
            # print(f"{joint_index} - {self.joint_array[joint_index]}")
        
        print("\n\n")

        curr_pose = PyKDL.Frame()
        del_pose = PyKDL.Frame()
        self.fk_p_kdl.JntToCart(self.joint_array, curr_pose)



        rot_matrix = R.from_euler('xyz', rotation, degrees=False).as_matrix()


#new code from here
        del_rot = PyKDL.Rotation(PyKDL.Vector(rot_matrix[0][0], rot_matrix[1][0], rot_matrix[2][0]),
                                  PyKDL.Vector(rot_matrix[0][1], rot_matrix[1][1], rot_matrix[2][1]),
                                  PyKDL.Vector(rot_matrix[0][2], rot_matrix[1][2], rot_matrix[2][2]))
        del_trans = PyKDL.Vector(translation[0], translation[1], translation[2])
        del_pose.M = del_rot
        del_pose.p = del_trans
        goal_pose_new = curr_pose*del_pose
        # print("cur pose - ", curr_pose )
        # print("del_pose - ", del_pose)
        # print("goal pose - ", goal_pose_new)

        # correction in final x, y, z postions
        print(f"corrections - {CORRECTION_X, CORRECTION_Y, CORRECTION_Z}")
        # print(f"corrections - {CORRECTION_X, global_parameters.CORRECTION_Y, CORRECTION_Z}")
        goal_pose_new.p[0] = goal_pose_new.p[0] + global_parameters.CORRECTION_X
        goal_pose_new.p[1] = goal_pose_new.p[1] + global_parameters.CORRECTION_Y
        goal_pose_new.p[2] = goal_pose_new.p[2] + global_parameters.CORRECTION_Z
        # print("goal pose - ", goal_pose_new)        

        seed_array = PyKDL.JntArray(self.arm_chain.getNrOfJoints())
        # seed_array[self.arm_chain.getNrOfJoints()-1] = self.joint_array[self.arm_chain.getNrOfJoints() - 1]
        # print("seed array - ", seed_array)
        self.ik_p_kdl.CartToJnt(seed_array, goal_pose_new, self.joint_array)

        ik_joints = {}

        # print(f"joint array length -{self.joint_array.rows()}")
        for joint_index in range(self.joint_array.rows()):
            # print(joint_index)
            # print(joint_list[joint_index])
            ik_joints[self.joint_list[joint_index]] = self.joint_array[joint_index]
        # print("ik joints - ", ik_joints)


        # print('ik_joints', ik_joints)
        # test_pose = PyKDL.Frame()
        # self.fk_p_kdl.JntToCart(self.joint_array, test_pose)

        # # print(test_pose.p)
        # # print(test_pose.M.GetRPY())

        self.move_to_joints(ik_joints, gripper)
        
        self.robot.push_command()
        time.sleep(2)

        self.updateJoints()
        for joint_index in range(self.joint_array.rows()):
            self.joint_array[joint_index] = self.joints[self.joint_list[joint_index]]
            # print(f"{joint_index} - {self.joint_array[joint_index]}")
        



