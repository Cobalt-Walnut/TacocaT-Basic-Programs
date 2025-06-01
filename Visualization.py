import matplotlib.pyplot as plt
import numpy as np

# Define lengths of segments
L1 = 195  # Length of first segment
L2 = 195  # Length of second segment

# Calculate positions
theta1 = np.radians(-135)
theta2 = np.radians(90)

# Endpoint of first segment
x1 = L1 * np.cos(theta1)
y1 = L1 * np.sin(theta1)

# Endpoint of second segment
x2 = x1 + L2 * np.cos(theta1 + theta2)
y2 = y1 + L2 * np.sin(theta1 + theta2)

# Plotting
plt.figure()
plt.plot([0, x1], [0, y1], 'b-', linewidth=5, label='Segment 1')
plt.plot([x1, x2], [y1, y2], 'r-', linewidth=5, label='Segment 2')
plt.scatter([0, x1, x2], [0, y1, y2], color='black')  # Base and joints
plt.text(0, 0, 'Base', fontsize=12, ha='right')
plt.text(x1, y1, 'Joint 1', fontsize=12, ha='right')
plt.text(x2, y2, 'End Effector', fontsize=12, ha='right')

plt.xlim(-300, 300)
plt.ylim(-300, 300)
plt.axhline(0, color='black',linewidth=0.5, ls='--')
plt.axvline(0, color='black',linewidth=0.5, ls='--')
plt.grid()
plt.title('Robotic Arm Configuration with Angle Offsets')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
