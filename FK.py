import numpy as np


def dh_matrix(theta, d, a, alpha):
    theta, alpha = np.radians(theta), np.radians(alpha)
    return np.array(
        [
            [
                np.cos(theta),
                -np.sin(theta) * np.cos(alpha),
                np.sin(theta) * np.sin(alpha),
                a * np.cos(theta),
            ],
            [
                np.sin(theta),
                np.cos(theta) * np.cos(alpha),
                -np.cos(theta) * np.sin(alpha),
                a * np.sin(theta),
            ],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1],
        ]
    )


# 6-DOF parameters
theta = [0, 0, 0, 0, 0, 0]  # Joint angles (degrees)
d = [0, 0, 0, 0, 0, 0]  # No offsets
a = [0, 8, 6, 4, 2, 1]  # Link lengths (cm)
alpha = [90, 0, 0, 90, -90, 0]  # Twist angles (degrees)

# Compute FK
T = np.eye(4)
for i in range(6):
    T = T @ dh_matrix(theta[i], d[i], a[i], alpha[i])

print("End-effector position (x, y, z):", T[0:3, 3])
print("Orientation (rotation matrix):", T[0:3, 0:3])
