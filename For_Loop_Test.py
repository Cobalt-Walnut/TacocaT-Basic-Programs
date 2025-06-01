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

# Code that tests if the motor can move in a for loop (helpful in inverse kinematics):
# result: Successful
if __name__ == "__main__":
    # Initialize the CAN bus
    can_bus = initialize_can_bus('can1')  # Adjust channel if necessary
    can_bus1 = initialize_can_bus('can1')  # Adjust channel if necessary
    
    # Set the CAN ID for the motor
    motor2_can_id = 0x003
    disable_motor_mode(can_bus1, motor2_can_id)
    time.sleep(1)
    # Enable the motor
    enable_motor_mode(can_bus1, motor2_can_id)
    print("Zeroing motor")
    send_motor_command(can_bus1, motor2_can_id, 0, 0, 2, 1, 0)
    time.sleep(3)
    print("Starting For Loop")
    #Main code/loop:
    #for x in range(starting number(default 0), ending number, tick amount(default 1))
    for position in range(0, 120, 1):
        send_motor_command(can_bus, motor2_can_id, position, 0, 2, 1, 0)
        time.sleep(0.005)
    time.sleep(3)
    for position in range(120, 0, -1):
        send_motor_command(can_bus, motor2_can_id, position, 0, 2, 1, 0)
        time.sleep(0.005)
    print("For Loop Test Finished")
    time.sleep(3)
    # Disable the motor
    disable_motor_mode(can_bus, motor2_can_id)

    # Close the CAN bus connection
    can_bus.shutdown()
