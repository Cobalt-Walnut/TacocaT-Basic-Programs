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

# Arm segment lengths (adjust these to match your arm's dimensions)
L1 = 195  # Length of the first arm segment in mm
L2 = 195  # Length of the second arm segment in mm

# Offset angles
THETA1_OFFSET = math.radians(45)  # 45 degree offset for theta1
THETA2_OFFSET = math.radians(90)  # 90 degree offset for theta2

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

def inverse_kinematics(bus, motor1_id, motor2_id, start_x, end_x, y, step=1):
    """
    Calculate and execute inverse kinematics for a range of x values.
    
    Args:
    bus: CAN bus object
    motor1_id: CAN ID for the first motor
    motor2_id: CAN ID for the second motor
    start_x: Starting x coordinate in mm
    end_x: Ending x coordinate in mm
    y: Fixed y coordinate in mm
    step: Step size for x coordinate (default 1)
    """
    ZERO_X = 250  # X-coordinate of the zero position

    for x in range(start_x, end_x, step):
        try:
            # Adjust x-coordinate relative to zero position
            rel_x = ZERO_X - x  # Changed from x - ZERO_X to ZERO_X - x

            # Calculate distance
            distance = math.sqrt(rel_x**2 + y**2)

            # Check if reachable
            if distance > L1 + L2 or distance < abs(L1 - L2):
                print(f"Position ({x}, {y}) is out of reach")
                continue

            # Calculate angles
            cos_theta2 = (distance**2 - L1**2 - L2**2) / (2 * L1 * L2)
            theta2 = math.acos(cos_theta2) - THETA2_OFFSET
            theta1 = math.atan2(y, rel_x) - math.atan2(L2 * math.sin(theta2 + THETA2_OFFSET), 
                                                       L1 + L2 * math.cos(theta2 + THETA2_OFFSET))
            theta1 = THETA1_OFFSET - theta1  # Changed from theta1 - THETA1_OFFSET

            # Convert to degrees
            angle1_deg = math.degrees(theta1)
            angle2_deg = math.degrees(theta2)

            # Send commands to motors
            send_motor_command(bus, motor1_id, angle1_deg, 0, 2, 1, 0)
            send_motor_command(bus, motor2_id, angle2_deg, 0, 2, 1, 0)
            time.sleep(0.01)  # Adjust delay as needed

        except Exception as e:
            print(f"Error at position ({x}, {y}): {str(e)}")

# Example usage in main:
if __name__ == "__main__":
    can_bus = initialize_can_bus('can1')
    motor1_can_id = 0x003
    motor2_can_id = 0x002
    
    try:
        enable_motor_mode(can_bus, motor1_can_id)
        enable_motor_mode(can_bus, motor2_can_id)

        print("Moving forward")
        inverse_kinematics(can_bus, motor1_can_id, motor2_can_id, 250, 450, 0, 1)
        time.sleep(2)

        print("Moving backward")
        inverse_kinematics(can_bus, motor1_can_id, motor2_can_id, 450, 250, 0, -1)
        time.sleep(2)
        send_motor_command(can_bus, motor1_can_id, 0, 0, 2.7, 0.7, 0)
        send_motor_command(can_bus, motor2_can_id, 0, 0, 2.7, 0.7, 0)
        print("Program Done. Sending Motor Home.\n")
        time.sleep(5)
    except KeyboardInterrupt:
        send_motor_command(can_bus, motor1_can_id, 0, 0, 2.7, 0.7, 0)
        send_motor_command(can_bus, motor2_can_id, 0, 0, 2.7, 0.7, 0)
        print("Keyboard Interrput Detected. Sending Motor Home.\n")
        time.sleep(5)
    finally:
        # Disable the motor
        disable_motor_mode(can_bus, motor1_can_id)
        disable_motor_mode(can_bus, motor2_can_id)
