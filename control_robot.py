import socket  # Example for communication, could be replaced with more advanced IPC
import numpy as np
import zmq


# client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# client_socket.connect(('localhost', 12345))

context = zmq.Context()
client_socket = context.socket(zmq.PUSH)
client_socket.connect("tcp://127.0.0.1:5555")

joint_positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)  # Example joint positions

# Teleoperation
# --------------------------------------------------
from pynput import keyboard

def on_press(key):
    global joint_positions
    if key == keyboard.Key.esc:
        return False
    elif key.char == 'w':
        print("w")
        joint_positions[0] += 0.1
    elif key.char == 's':
        print("s")
        joint_positions[0] -= 0.1
    elif key.char == 'a':
        print("a")
        joint_positions[0] = 0.2
        joint_positions[1] = -0.2
    elif key.char == 'd':
        print("d")
        joint_positions[0] = -0.2
        joint_positions[1] = 0.2

listener = keyboard.Listener(on_press=on_press)
listener.start()
# --------------------------------------------------

while True:
    print(joint_positions)
    client_socket.send(joint_positions)
