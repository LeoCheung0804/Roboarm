import ikpy.chain
import numpy as np

# Load your arm’s model (replace with your URDF file)
chain = ikpy.chain.Chain.from_urdf_file("robot_arm.urdf")

# Target position for the end-effector
target = [0.5, 0.5, 0.5]  # x, y, z in meters

# Calculate joint angles
angles = chain.inverse_kinematics(target)

# Convert angles (radians) to ticks (example conversion)
ticks_per_radian = 1000 / (2 * np.pi)  # 1000 ticks per full rotation
positions = [angle * ticks_per_radian for angle in angles]