import argparse

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode",
                    choices = ["move", "pick"], default = "move",
                    help = "Choose the mode of operation."
                            "m  -> moving about a frame of reference to a point"
                            "po -> Picking a object with fixed offset")
    parser.add_argument("-bf", "--base_frame",
                    choices = ["gripper_camera", "top_camera", "gripper_left", "gripper_right"], default = "gripper_camera",
                    help = "Operating frame of reference")
    parser.add_argument("-t", "--transform", 
                    action="store_true", 
                    help = "Boolean for transforming a input co-ordinates to another frame of reference")
    parser.add_argument("-tf", "--transform_node", 
                    choices = ["gripper_camera", "top_camera", "gripper_left", "gripper_right"],
                    default = "gripper_left",
                    help = "Operating frame of reference")
    
    return parser.parse_args()