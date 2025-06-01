import can
import struct
import math
import time

# Motor Constants
P_MIN, P_MAX = -95.5, 95.5  # in radians
V_MIN, V_MAX = -45.0, 45.0  # in radians per second
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -18.0, 18.0  # T for Torque (FF_Current)

def degrees_to_radians(degrees):
    return degrees * (math.pi / 180.0)

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

def initialize_can_bus(channel='can0', bustype='socketcan'):
    return can.interface.Bus(channel=channel, bustype=bustype)

def enable_motor_mode(bus, can_id):
    send_can_message(bus, can_id, b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC')

def disable_motor_mode(bus, can_id):
    send_can_message(bus, can_id, b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD')

def set_zero_position(bus, can_id):
    send_can_message(bus, can_id, b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE')

def send_motor_command(bus, can_id, position, velocity, kp, kd, torque):
    packed_data = pack_commands(position, velocity, kp, kd, torque)
    send_can_message(bus, can_id, packed_data)

def inverse_kinematics(bus, motor1_id, motor2_id, target_x, target_y):
    L1 = 195  # Length of the first arm segment in mm
    L2 = 195  # Length of the second arm segment in mm
    THETA1_OFFSET = math.radians(90)  # Adjusted for vertical alignment
    THETA2_OFFSET = math.radians(90)   # Adjusted for second motor

    try:
        # Calculate the distance from the base to the target point
        distance = math.sqrt(target_x**2 + target_y**2)

        # Check if the point is reachable
        if distance > (L1 + L2) or distance < abs(L1 - L2):
            print(f"Target position ({target_x}, {target_y}) is out of reach")
            return None

        # Calculate angles using the law of cosines
        cos_theta2 = (distance**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if abs(cos_theta2) > 1:
            print(f"Invalid cos_theta2 value: {cos_theta2}")
            return None
        
        theta2 = math.acos(cos_theta2) - THETA2_OFFSET
        theta1 = math.atan2(target_y, target_x) - math.atan2(L2 * math.sin(theta2 + THETA2_OFFSET), 
                                                               L1 + L2 * math.cos(theta2 + THETA2_OFFSET))
        theta1 -= THETA1_OFFSET  # Adjust for offset

        # Convert radians to degrees
        angle1_deg = math.degrees(theta1)
        angle2_deg = math.degrees(theta2)

        # Send commands to motors
        send_motor_command(bus, motor1_id, angle1_deg, 0, 2.7, 0.7, 0)
        send_motor_command(bus, motor2_id, angle2_deg, 0, 2.7, 0.7, 0)

    except Exception as e:
        print(f"Error in inverse kinematics: {str(e)}")

# Example usage in main:
if __name__ == "__main__":
    can_bus = initialize_can_bus('can1')
    motor1_can_id = 0x003
    motor2_can_id = 0x002

    try:
        enable_motor_mode(can_bus, motor1_can_id)
        enable_motor_mode(can_bus, motor2_can_id)

        print("Moving to position (0, 250)")
        inverse_kinematics(can_bus, motor1_can_id, motor2_can_id, 0, 250)

        time.sleep(3)

        print("Moving back to position (0, 0)")
        inverse_kinematics(can_bus, motor1_can_id, motor2_can_id, 0, 350)
        
        time.sleep(3)
    except KeyboardInterrupt:
        print("Movement interrupted.")
        
    finally:
        disable_motor_mode(can_bus, motor1_can_id)
        disable_motor_mode(can_bus, motor2_can_id)
