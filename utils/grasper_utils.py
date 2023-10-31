import PyKDL
import time

def move_to_point(robot, point, base_node, gripper_node):
    rotation = PyKDL.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1)

    dest_frame = PyKDL.Frame(rotation, point)
    transform, _, _ = robot.get_joint_transform(base_node, gripper_node)

    print('dest_frame', dest_frame.p)
    # Rotation from gripper frame frame to gripper frame
    transformed_frame = transform * dest_frame

    transformed_frame.p[2] -= 0.2

    robot.move_to_pose(
            [transformed_frame.p[0], transformed_frame.p[1], transformed_frame.p[2]],
            [0, 0, 0],
            # [-transformed_frame.M.GetRPY()[1], transformed_frame.M.GetRPY()[0], transformed_frame.M.GetRPY()[2]+1.53],
            [1],
            move_mode=1
        )

def pickup(robot, rotation, translation, base_node, gripper_node, gripper_height = 0.03, gripper_depth=0.03):
    """
        rotation: Relative rotation of gripper pose w.r.t camera
        translation: Relative translation of gripper pose w.r.t camera
        cam2gripper_transform: Transform for 
    """

    point = PyKDL.Vector(-translation[1], -translation[0], translation[2])

    # Rotation from model frame to pose frame
    rotation1 = PyKDL.Rotation(rotation[0][0], rotation[0][1], rotation[0][2],
                            rotation[1][0],  rotation[1][1], rotation[1][2],
                                rotation[2][0],  rotation[2][1], rotation[2][2])
    
    # Rotation from camera frame to model frame
    rotation1_bottom = PyKDL.Rotation(0.0000000, -1.0000000,  0.0000000,
                                -1.0000000,  0.0000000,  0.0000000, 
                                0.0000000,  0.0000000, 1.0000000)
    
    # Rotation from camera frame to pose frame
    rotation =  rotation1_bottom * rotation1

    dest_frame = PyKDL.Frame(rotation, point) 

    # Can remove base and gripper node
    cam2gripper_transform, _, _ = robot.get_joint_transform(base_node, gripper_node)
    # Rotation from gripper frame frame to gripper frame
    transformed_frame = cam2gripper_transform * dest_frame

    # Lifting the arm to high position
    robot.move_to_position(lift_pos = 1.0, head_pan = None, head_tilt = None)

    # Rotation for aligning gripper frame   to model pose frame
    rotation2_top = PyKDL.Rotation(0, 0, 1, 1, 0, 0, 0, -1, 0)

    # final Rotation of gripper to hold the objet
    final_rotation = transformed_frame.M * rotation2_top

    robot.move_to_pose(
            [0, 0, 0],
            [final_rotation.GetRPY()[0], final_rotation.GetRPY()[1], final_rotation.GetRPY()[2]],
            [1],
        )
    
    cam2gripper_transform, _, _ = robot.get_joint_transform(base_node, gripper_node)

    transformed_point1 = cam2gripper_transform * point

    diff_value = (0.228 - gripper_depth - gripper_height)
    transformed_point1[2] -= (diff_value)
    ref_diff = (diff_value)

    # Moving gripper to pose center
    robot.move_to_pose(
        [transformed_point1.x(), transformed_point1.y(), transformed_point1.z() - 0.2],
        [0, 0, 0],
        # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
        [1],
        move_mode = 1
    )

    time.sleep(1)
    transform, frame2, frame1 = robot.get_joint_transform(base_node, gripper_node)
    transformed_point2 = transform * point
    print(f"transformed point2 : {transform * point}")
    curr_diff = transformed_point2.z()

    diff = abs(curr_diff - ref_diff)
    velocities = [1]*8
    velocities[5:] = [0.007, 0.007, 0.007, 0.007]
    velocities[0] = 0.005
    if diff > 0.08:
        dist = diff - 0.08
        robot.move_to_pose(
            [0, 0, dist],
            [0, 0, 0],
            # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
            [1]
        )
        time.sleep(1)
        transform, frame2, frame1 = robot.get_joint_transform(base_node, gripper_node)
        print(f"transformed point3 : {transform * point}")
        diff = diff - dist
        
    while diff > 0.01:
        dist = min(0.03, diff)
        robot.move_to_pose(
            [0, 0, dist],   
            [0, 0, 0],
            # [rotation.GetRPY()[0], rotation.GetRPY()[1], rotation.GetRPY()[2]],
            [1]
            # velocities=velocities
        )
        time.sleep(1)
        transform, frame2, frame1 = robot.get_joint_transform(base_node, gripper_node)
        print(f"transformed point3 : {transform * point}")
        diff = diff - dist
    
    robot.pickup(abs(0))
        # Put it down for now
    time.sleep(3)
    robot.move_to_position(lift_pos = 1.0, arm_pos = 0)
    robot.move_to_position(wrist_pitch = -1.57, 
                            wrist_yaw = 0,
                            wrist_roll = 0)
    robot.move_to_position(lift_pos = 0.35)


