import mujoco
import mujoco_viewer
# from robots import get_robot
import numpy as np
import os
from mujoco import MjModel
from target import Target
from dm_control import mjcf


# current directory
path = "robots\\franka_emika_panda\\scenes\\scene.xml"
# model = mujoco.MjModel.from_xml_path(path)

robot = mjcf.from_path(path)            # Load the robot model
mjcf_model = mjcf.RootElement()         # Create a new MJCF model
mjcf_model.attach(robot)                # Attach the robot model to the MJCF model
target = Target(mjcf_model)             # Add a target to the MJCF model

physics = mjcf.Physics.from_mjcf_model(mjcf_model)      # Create a physics object from the MJCF model
model = physics.model.ptr                # Get the model pointer from the physics object





# Load the robot model
# --------------------------------------------------
# model = get_robot("UR5e")
data = mujoco.MjData(model)
print(f"Input shape: {data.ctrl.shape}")
joint_positions = np.zeros(data.ctrl.shape[0], dtype=np.float64)
# --------------------------------------------------



# Simulate and visualize the robot
# --------------------------------------------------
# Create a viewer for visualization
viewer = mujoco_viewer.MujocoViewer(model, data)

# Simulate and visualize
while True:
    # if connection is not None:
    #     recv_data = connection.recv()  # Receive command data
    #     if recv_data:
            # joint_positions = np.frombuffer(recv_data, dtype=np.float64)
    #         print(joint_positions)
    data.ctrl[:] = joint_positions  # Update the robot's joint positions
    joint_positions[0] = 1.57
    joint_positions[1] = 1.57
    print(f"Current joint positions: {data.ctrl}")
    mujoco.mj_step(model, data)
    viewer.render()  # Render the simulation (visualization)

    # Exit on ESC
    if viewer.is_alive is False:
        break

viewer.close()
# --------------------------------------------------