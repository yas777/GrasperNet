import argparse
import yaml

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode",
                    choices = ["move", "pick", "capture", "place"], default = "move",
                    help = "Choose the mode of operation."
                            "m  -> moving about a frame of reference to a point"
                            "po -> Picking a object with fixed offset")
    parser.add_argument("-o1", "--picking_object", 
                    help = "picking object")
    parser.add_argument("-o2", "--placing_object", 
                    help = "placing object")
    parser.add_argument("-bf", "--base_frame",
                    choices = [ "gripper_camera", "top_camera", 
                                "gripper_fingertip_left", "gripper_fingertip_right"], default = "gripper_camera",
                    help = "Operating frame of reference")
    parser.add_argument("-t", "--transform", 
                    action="store_true", 
                    help = "Boolean for transforming a input co-ordinates to another frame of reference")
    parser.add_argument("-tf", "--transform_node", 
                    choices = [ "gripper_camera", "top_camera", 
                                "gripper_fingertip_left", "gripper_fingertip_right", 
                                "gripper_left", "gripper_mid"],
                    default = "gripper_mid",
                    help = "Operating frame of reference")

    
    # with open("config.yaml", "r") as conf:
    #     return yaml.load(conf)
    return parser.parse_args()