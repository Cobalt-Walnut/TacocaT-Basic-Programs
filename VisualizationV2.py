import matplotlib.pyplot as plt
import numpy as np
import math

# Robot parameters
L1 = 195  # Length of first segment
L2 = 195  # Length of second segment
HYPOTENUSE = -275  # Length of hypotenuse

theta1_offset = 45
theta2_offset = 90

def inverse_kinematics(hypotenuse):
    if hypotenuse > (L1 + L2):
        return None  # Target is out of reach

    # Calculate theta2 using cosine rule
    cos_theta2 = (L1**2 + L2**2 - hypotenuse**2) / (2 * L1 * L2)
    theta2 = math.acos(cos_theta2)

    # Calculate theta1 using geometry
    theta1 = math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))

    # Convert to degrees
    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)

    return (theta1_deg, theta2_deg)

def plot_robot_arm(hypotenuse):
    result = inverse_kinematics(hypotenuse)
    
    if result is None:
        print("Target is out of reach.")
        return
    
    theta1_deg, theta2_deg = result
    theta1_rad = np.radians(theta1_deg)
    theta2_rad = np.radians(theta2_deg)

    # Calculate joint positions based on angles
    x1 = L1 * np.cos(theta1_rad)  # Position of Joint 1
    y1 = -L1 * np.sin(theta1_rad)  # Inverted y-axis for downward movement
    
    x2 = x1 + L2 * np.cos(theta1_rad + theta2_rad)  # Position of End Effector
    y2 = y1 - L2 * np.sin(theta1_rad + theta2_rad)  # Inverted y-axis for downward movement

    # Plotting
    plt.figure()
    plt.plot([0, x1], [0, y1], 'b-', linewidth=5, label='Segment 1')
    plt.plot([x1, x2], [y1, y2], 'r-', linewidth=5, label='Segment 2')
    
    plt.scatter([0, x1, x2], [0, y1, y2], color='black')  # Base and joints
    plt.text(0, 0, 'Base', fontsize=12, ha='right')
    plt.text(x1, y1, 'Joint 1', fontsize=12, ha='right')
    plt.text(x2, y2, 'End Effector', fontsize=12, ha='right')

    plt.xlim(-L1-L2-50, L1+L2+50)
    plt.ylim(-L1-L2-50, 50)  # Adjusted limits to focus on negative y-direction
    
    plt.axhline(0, color='black', linewidth=0.5, ls='--')
    plt.axvline(0, color='black', linewidth=0.5, ls='--')
    
    plt.grid()
    plt.title('Robotic Arm Configuration with Downward Movement')
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    
    plt.show()

    # Print out the calculated angles
    print(f"Calculated Angles: θ1={theta1_deg-theta1_offset:.2f}°, θ2={theta2_deg-theta2_offset:.2f}°")

# Example usage:
if __name__ == "__main__":
    plot_robot_arm(HYPOTENUSE)
