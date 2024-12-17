import mujoco
import mujoco_viewer
from robots import get_robot
import socket
import numpy as np
import threading
import zmq


# Load the robot model
# --------------------------------------------------
model = get_robot("UR5e")
data = mujoco.MjData(model)
print(f"Input shape: {data.ctrl.shape}")
joint_positions = np.zeros(data.ctrl.shape[0], dtype=np.float64)
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
    mujoco.mj_step(model, data)
    viewer.render()  # Render the simulation (visualization)

    # Exit on ESC
    if viewer.is_alive is False:
        break

viewer.close()
# --------------------------------------------------

