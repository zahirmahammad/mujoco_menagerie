import os
import mujoco

# current directory
MODEL_DIR = os.path.dirname(__file__)       

# Dictionary mapping model names to their file paths
MODEL_PATHS = {
    "UR5e": os.path.join(MODEL_DIR, "universal_robots_ur5e", "scene.xml"),
    "UR10e": os.path.join(MODEL_DIR, "universal_robots_ur10e", "scene.xml"),
    "Spot": os.path.join(MODEL_DIR, "boston_dynamics_spot", "scene.xml"),
    "AgilityCassie": os.path.join(MODEL_DIR, "agility_cassie", "scene.xml"),
    "Aloha": os.path.join(MODEL_DIR, "aloha", "scene.xml"),
    "AnymalB": os.path.join(MODEL_DIR, "anybotics_anymal_b", "scene.xml"),
    "AnymalC": os.path.join(MODEL_DIR, "anybotics_anymal_c", "scene.xml"),
    "Crazyflie2": os.path.join(MODEL_DIR, "bitcraze_crazyflie_2", "scene.xml"),
    "FrankaPanda": os.path.join(MODEL_DIR, "franka_emika_panda", "scene.xml"),
    "FrankaFR3": os.path.join(MODEL_DIR, "franka_fr3", "scene.xml"),
    "BarkourV0": os.path.join(MODEL_DIR, "google_barkour_v0", "scene.xml"),
    "Barkour_vb": os.path.join(MODEL_DIR, "google_barkour_vb", "scene.xml"),
    "GoogleRobot": os.path.join(MODEL_DIR, "google_robot", "scene.xml"),
    "HelloRobotStretch": os.path.join(MODEL_DIR, "hello_robot_stretch", "scene.xml"),
    "HelloRobotStretch3": os.path.join(MODEL_DIR, "hello_robot_stretch_3", "scene.xml"),
    "KinovaGen3": os.path.join(MODEL_DIR, "kinova_gen3", "scene.xml"),
    "KukaIIWA14": os.path.join(MODEL_DIR, "kuka_iiwa_14", "scene.xml"),
    "LeapHand": os.path.join(MODEL_DIR, "leap_hand", "scene.xml"),
    "PalTalos": os.path.join(MODEL_DIR, "pal_talos", "scene.xml"),
    "Realsense": os.path.join(MODEL_DIR, "realsense_d435i", "scene.xml"),
    "Sawyer": os.path.join(MODEL_DIR, "rethink_robotics_sawyer", "scene.xml"),
    "Robotiq-2f85": os.path.join(MODEL_DIR, "robotiq_2f85", "scene.xml"),
    "Robotiq-OP3": os.path.join(MODEL_DIR, "robotis_op3", "scene.xml"),    
    "Husky": os.path.join(MODEL_DIR, "husky", "scene.xml"),    
}

def get_robot(name):
    """
    Load a MuJoCo model by its name.
    :param name: The name of the model (key in the MODEL_PATHS dictionary).
    :return: The MuJoCo model object.
    """
    from mujoco import MjModel

    if name not in MODEL_PATHS:
        raise ValueError(f"Model '{name}' not found. Available models: {list(MODEL_PATHS.keys())}")
    
    model = mujoco.MjModel.from_xml_path(MODEL_PATHS[name])

    # return MjModel.from_xml_path(MODEL_PATHS[name])
    return model
