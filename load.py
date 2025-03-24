import numpy as np

g = 9.81  # m/s^2
L = [0, 0.08, 0.06, 0.04, 0.02, 0.01]  # Link lengths (m)
m = [0.6, 0.5, 0.4, 0.3, 0.2, 0.1]  # Masses (kg)
m_p = 0.2  # Payload (kg)
theta = [0, 0, 0, 0, 0, 0]  # Angles (degrees)

# Forces
F = [m[i] * g for i in range(6)]
F_p = m_p * g

# Torque calculations (static, horizontal)
tau = [0] * 6
F_distal = F_p
for i in range(5, -1, -1):  # Work from Joint 6 to 1
    if i == 5:  # Joint 6
        tau[i] = F_p * L[i]
    else:  # Joints 1-5
        tau[i] = (F[i] * L[i] / 2) + (F_distal * L[i])
    F_distal += F[i]  # Add current link's weight to distal load

# Print results
for i in range(6):
    print(f"Torque at Joint {i+1}: {tau[i]:.4f} Nm")
