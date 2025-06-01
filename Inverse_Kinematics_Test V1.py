import can
import struct
import math
import time
import numpy as np

# Motor Constants
P_MIN, P_MAX = -95.5, 95.5  # in radians
V_MIN, V_MAX = -45.0, 45.0  # in radians per second
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -18.0, 18.0  # T for Torque (FF_Current)

# Arm Constants in mm
L1 = 195  # Length of the first link (ground link)
L2 = 105  # Length of the input link (crank)
L3 = 195  # Length of the output link (rocker)
L4 = 105  # Length of the coupler link

# Base joint limits
BASE_MIN = -150  # degrees
BASE_MAX = 150   # degrees

def degrees_to_radians(degrees):
    return degrees * (math.pi / 180.0)

def radians_to_degrees(radians):
    return radians * (180.0 / math.pi)

def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    if x < x_min:
        x = x_min
    elif x > x_max:
        x = x_max
    return int((x - x_min) * ((1 << bits) - 1) / span)

def pack_commands(position_deg, velocity_deg_per_sec, kp, kd, torque):
    position_rad = degrees_to_radians(position_deg)
    velocity_rad_per_sec = degrees_to_radians(velocity_deg_per_sec)

    p = float_to_uint(position_rad, P_MIN, P_MAX, 16)
    v = float_to_uint(velocity_rad_per_sec, V_MIN, V_MAX, 12)
    kp = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t = float_to_uint(torque, T_MIN, T_MAX, 12)
    
    packed = struct.pack(">HHHH", 
        p,
        (v << 4) | (kp >> 8),
        ((kp & 0xFF) << 8) | (kd >> 4),
        ((kd & 0xF) << 12) | t
    )
    
    return packed

def send_can_message(bus, can_id, data):
    try:
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        bus.send(msg)
        print(f"Message sent on {bus.channel_info}")
    except can.CanError:
        print("Message NOT sent")

def inverse_kinematics(bus, motor1_id, motor2_id, x_mm, y_mm):
    """
    Compute inverse kinematics and send commands to motors.
    All calculations are in millimeters.
    """
    try:
        # Calculate the angle and distance to the end effector
        r = math.sqrt(x_mm**2 + y_mm**2)
        phi = math.atan2(y_mm, x_mm)

        print(f"Target position: ({x_mm}, {y_mm}) mm, r = {r:.2f} mm")

        # Check if the point is reachable
        #if r > L1 + L2 or r < abs(L1 - L2):
            #raise ValueError(f"Position is out of reach. r = {r:.2f}, L1 + L2 = {L1 + L2}")

        # Calculate the input link angle (theta1)
        cos_theta1 = (r**2 + L1**2 - L2**2) / (2 * r * L1)
        if abs(cos_theta1) > 1.1:
            raise ValueError(f"cos_theta1 out of range: {cos_theta1}")
        theta1 = phi - math.acos(cos_theta1)

        # Calculate the output link angle (theta2)
        cos_theta2 = (L1**2 + L2**2 - r**2) / (2 * L1 * L2)
        if abs(cos_theta2) > 1:
            raise ValueError(f"cos_theta2 out of range: {cos_theta2}")
        theta2 = math.acos(cos_theta2)

        # Calculate the coupler link angle (theta3)
        D = math.sqrt(L1**2 + L4**2 - 2*L1*L4*math.cos(theta2))
        cos_alpha = (D**2 + L2**2 - L3**2) / (2 * D * L2)
        if abs(cos_alpha) > 1:
            raise ValueError(f"cos_alpha out of range: {cos_alpha}")
        alpha = math.acos(cos_alpha)
        theta3 = theta1 + alpha

        # Convert to degrees
        theta1_deg = radians_to_degrees(theta1)
        theta3_deg = radians_to_degrees(theta3)

        print(f"Calculated angles: θ1={theta1_deg:.2f}°, θ2={theta3_deg:.2f}°")

        # Check if base joint is within limits
        if theta1_deg < BASE_MIN or theta1_deg > BASE_MAX:
            raise ValueError(f"Base joint angle {theta1_deg:.2f}° is out of range ({BASE_MIN}° to {BASE_MAX}°)")

        # Send commands to motors
        velocity = 0  # deg/s
        kp = 2
        kd = 0.5
        torque = 0

        send_can_message(bus, motor1_id, pack_commands(theta1_deg, velocity, kp, kd, torque))
        send_can_message(bus, motor2_id, pack_commands(theta3_deg, velocity, kp, kd, torque))

        print(f"Moved to ({x_mm:.2f}, {y_mm:.2f}) mm: θ1={theta1_deg:.2f}°, θ2={theta3_deg:.2f}°")

    except ValueError as e:
        print(f"Error in inverse kinematics: {str(e)}")
    except Exception as e:
        print(f"Unexpected error in inverse kinematics: {str(e)}")

    
def initialize_can_bus(channel='can0', bustype='socketcan'):
    return can.interface.Bus(channel=channel, bustype=bustype)

def enable_motor_mode(bus, can_id):
    send_can_message(bus, can_id, b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC')

def disable_motor_mode(bus, can_id):
    send_can_message(bus, can_id, b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD')

def send_motor_command(bus, can_id, position, velocity, kp, kd, torque):
    packed_data = pack_commands(position, velocity, kp, kd, torque)
    send_can_message(bus, can_id, packed_data)

if __name__ == "__main__":
    can_bus = initialize_can_bus('can1')  # Adjust channel if necessary
    motor1_can_id = 0x002
    motor2_can_id = 0x003

    try:
        x = 1
        
        # Enable motors
        enable_motor_mode(can_bus, motor1_can_id)
        enable_motor_mode(can_bus, motor2_can_id)
        
        # Main loop
        send_motor_command(can_bus, motor1_can_id, 0, 0, 3, 1, 0)
        send_motor_command(can_bus, motor2_can_id, 0, 0, 3, 1, 0)
        print("Zeroing...")
        time.sleep(3)
        print("Starting!")
        for y in range(155, 335, 1):
            try:
                inverse_kinematics(can_bus, motor1_can_id, motor2_can_id, x, y)
                time.sleep(0.01)
            except Exception as e:
                print(f"Error at y={y}: {str(e)}")
        time.sleep(3)
        print("Going back up!")
        time.sleep(1)
        for y in range(335, 155, -1):
            try:
                inverse_kinematics(can_bus, motor1_can_id, motor2_can_id, x, y)
                time.sleep(0.01)
            except Exception as e:
                print(f"Error at y={y}: {str(e)}")
        print("Done!")
        time.sleep(1)
    except KeyboardInterrupt:
        print("Keyboard Interrupt Detected")
    finally:
        send_motor_command(can_bus, motor1_can_id, 0, 0, 3, 1, 0)
        send_motor_command(can_bus, motor2_can_id, 0, 0, 3, 1, 0)
        print("Zeroing motors")
        time.sleep(5)
        # Disable motors
        disable_motor_mode(can_bus, motor1_can_id)
        disable_motor_mode(can_bus, motor2_can_id)
        # Close the CAN bus connection
        can_bus.shutdown()

