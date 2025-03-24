import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to PyBullet with a graphical interface
physicsClient = p.connect(p.GUI)

# Set gravity in the Z-direction
p.setGravity(0, 0, -9.81)

# Load the ground plane from PyBullet's built-in data
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf")

# Load your 6 DOF robot arm URDF, fixed to the base.
# Make sure the URDF file "6dof_robot.urdf" is in your working directory or PyBullet's search path.
robot_id = p.loadURDF("6dof_robot.urdf", basePosition=[0, 0, 0], useFixedBase=True)

# Apply damping to all 6 joints for smoother motion
num_joints = 6
for joint in range(num_joints):
    p.changeDynamics(robot_id, joint, linearDamping=0.1, angularDamping=0.1)

# Simulation loop with oscillating joint angles
t = 0  # Time variable
while True:
    # Calculate joint angles using a sine wave for each joint.
    # We introduce a phase shift for each joint so the motion is not identical.
    angles = [0.5 * np.sin(t + phase) for phase in np.linspace(0, np.pi, num_joints)]

    # Set joint positions using position control for each of the 6 joints.
    for i in range(num_joints):
        p.setJointMotorControl2(
            robot_id, i, p.POSITION_CONTROL, targetPosition=angles[i]
        )

    # Step the simulation
    p.stepSimulation()

    # Compute and print the end-effector position
    # Here we assume the end-effector is attached to the last link (joint index 5).
    link_state = p.getLinkState(robot_id, num_joints - 1)
    link_pos = np.array(link_state[4])  # World position of the link's frame
    link_orn = link_state[5]  # World orientation (quaternion) of the link
    rot_matrix = np.array(p.getMatrixFromQuaternion(link_orn)).reshape(3, 3)

    # Define an offset for the end-effector relative to the last link’s origin.
    # Adjust this offset based on your robot's URDF and desired end-effector location.
    offset = np.array([0.25, 0, 0])
    offset_world = rot_matrix @ offset
    end_effector_pos = link_pos + offset_world
    print(f"End-effector position: {end_effector_pos}")

    # Increment time and sleep to maintain the simulation rate (e.g., 240 Hz)
    t += 1.0 / 240.0
    time.sleep(1.0 / 240.0)
