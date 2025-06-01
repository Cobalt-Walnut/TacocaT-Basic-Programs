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

def inverse_kinematics(x, y):
    """
    Calculate the angles for both motors given a target (x, y) position.
    
    Args:
    x (float): Target x-coordinate in mm
    y (float): Target y-coordinate in mm
    
    Returns:
    tuple: (angle1, angle2) in degrees
    """
    try:
        # Adjust y coordinate to account for the mount height
        y = y + L1 * math.sin(THETA1_OFFSET)
        x = x - L1 * math.cos(THETA1_OFFSET)

        # Calculate the distance from the end of L1 to the target point
        distance = math.sqrt(x**2 + y**2)
        
        # Rotate the target point by -45 degrees
        #x_rotated = x * math.cos(math.radians(-45)) - y * math.sin(math.radians(-45))
        #y_rotated = x * math.sin(math.radians(-45)) + y * math.cos(math.radians(-45))
        
        # Use rotated coordinates for calculations
        #distance = math.sqrt(x_rotated**2 + y_rotated**2)
        
        # Check if the point is reachable
        if distance > L2 or distance < 0:
            raise ValueError("Target position is out of reach")

        # Calculate theta2 (angle of L2 relative to L1)
        cos_theta2 = (distance**2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta2 = math.acos(cos_theta2) - THETA2_OFFSET

        # Calculate theta1 (angle of L1 relative to vertical)
        theta1 = math.atan2(x, y) - math.atan2(L2 * math.sin(theta2 + THETA2_OFFSET), 
                                               L1 + L2 * math.cos(theta2 + THETA2_OFFSET))
        theta1 = theta1 - THETA1_OFFSET

        # Convert radians to degrees
        angle1_deg = math.degrees(theta1)
        angle2_deg = math.degrees(theta2)
        
        # Adjust final angles
        #angle1_deg = math.degrees(theta1) + 45
        #angle2_deg = math.degrees(theta2)
        
        return angle1_deg, angle2_deg

    except ValueError as e:
        print(f"Error in inverse kinematics: {str(e)}")
        return None
    except Exception as e:
        print(f"Unexpected error in inverse kinematics: {str(e)}")
        return None

def ik_motor(bus, id1, id2, x, y):
    velocity = 0
    kp = 2.7
    kd = 0.7
    torque = 0
    angles = inverse_kinematics(x, y)
    if angles is not None:
        theta1, theta2 = angles
        send_motor_command(bus, id1, theta1, velocity, kp, kd, torque)
        send_motor_command(bus, id2, theta2, velocity, kp, kd, torque)
    else:
        print(f"Unable to calculate angles for position ({x}, {y})")

if __name__ == "__main__":
    x = 0
    
    # Initialize the CAN bus
    can_bus = initialize_can_bus('can1')  # Adjust channel if necessary
    can_bus1 = initialize_can_bus('can1')  # Adjust channel if necessary
    
    # Set the CAN ID for the motor
    motor1_can_id = 0x003
    motor2_can_id = 0x002
    disable_motor_mode(can_bus, motor1_can_id)
    disable_motor_mode(can_bus1, motor2_can_id)
    time.sleep(1)
    # Enable the motor
    enable_motor_mode(can_bus, motor1_can_id)
    enable_motor_mode(can_bus1, motor2_can_id)
    try:
        print("Starting")
        #250 for x is up and down minimum
        for y in range (250, 450, 1):
            ik_motor(can_bus, motor1_can_id, motor2_can_id, x, y)
            time.sleep(0.01)
        time.sleep(3)
        for y in range(450, 250, -1):
            ik_motor(can_bus, motor1_can_id, motor2_can_id, x, y)
            time.sleep(0.01)
        print("done!")
        time.sleep(30)
        #ik_motor(can_bus, motor1_can_id, motor2_can_id, 55, y)
        #time.sleep(3)
        send_motor_command(can_bus, motor1_can_id, 0, 0, 2.7, 0.7, 0)
        send_motor_command(can_bus1, motor2_can_id, 0, 0, 2.7, 0.7, 0)
        print("Program Done. Sending Motor Home.\n")
        time.sleep(5)
    except KeyboardInterrupt:
        send_motor_command(can_bus, motor1_can_id, 0, 0, 2, 1, 0)
        send_motor_command(can_bus1, motor2_can_id, 0, 0, 2, 1, 0)
        print("Keyboard Interrput Detected. Sending Motor Home.\n")
        time.sleep(5)
    finally:
        # Disable the motor
        disable_motor_mode(can_bus, motor1_can_id)
        disable_motor_mode(can_bus, motor2_can_id)

        # Close the CAN bus connection
        can_bus.shutdown()
