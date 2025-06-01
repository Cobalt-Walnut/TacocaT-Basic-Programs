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
THETA1_OFFSET = math.radians(90)  # Adjusted to vertical alignment
THETA2_OFFSET = math.radians(90)   # Adjusted for second motor

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

def send_motor_command(bus, can_id, position_deg):
    packed_data = pack_commands(position_deg, 0, 2.7, 0.7, 0)
    send_can_message(bus, can_id, packed_data)

def move_leg_vertical(bus1, motor1_can_id, motor2_can_id):
    target_height = 250

    # Move up to target height
    for h in range(0, target_height + 1):   # Incrementally move to target height
        try:
            distance = h

            # Check if reachable
            if distance > (L1 + L2) or distance < abs(L1 - L2):
                print(f"Target height {h} is out of reach")
                continue

            # Calculate angles using simple geometry for vertical movement
            theta2 = math.acos((L1**2 + L2**2 - distance**2) / (2 * L1 * L2))
            theta1 = math.atan2(distance, L1) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))

            # Convert angles to degrees and adjust for offsets
            angle1_deg = math.degrees(theta1) - THETA1_OFFSET * (180 / math.pi)
            angle2_deg = math.degrees(theta2) - THETA2_OFFSET * (180 / math.pi)

            # Send commands to motors
            send_motor_command(bus1,motor1_can_id , angle1_deg)
            send_motor_command(bus1,motor2_can_id , angle2_deg)

            time.sleep(0.01)   # Adjust delay as needed

        except Exception as e:
            print(f"Error moving to height {h}: {str(e)}")

# Example usage in main:
if __name__ == "__main__":
    bus1 = initialize_can_bus('can1')
    bus2 = initialize_can_bus('can1')

    motor1_can_id = 0x002
    motor2_can_id = 0x003

    try:
        enable_motor_mode(bus1,motor1_can_id)
        enable_motor_mode(bus2,motor2_can_id)

        print("Moving up to 250mm")
        move_leg_vertical(bus1,motor1_can_id,motor2_can_id)

        time.sleep(3)

        print("Moving back down to zero position")
        move_leg_vertical(bus1,motor1_can_id,motor2_can_id)

        print("Movement complete.")
        
        send_motor_command(bus1, motor1_can_id, 0)
        send_motor_command(bus1, motor2_can_id, 0)
        print("Program Done. Sending Motor Home.\n")
        time.sleep(5)
    except KeyboardInterrupt:
        send_motor_command(bus1, motor1_can_id, 0)
        send_motor_command(bus1, motor2_can_id, 0)
        print("Keyboard Interrput Detected. Sending Motor Home.\n")
        time.sleep(5)
        
    finally:
        disable_motor_mode(bus1,motor1_can_id)
        disable_motor_mode(bus2,motor2_can_id)
