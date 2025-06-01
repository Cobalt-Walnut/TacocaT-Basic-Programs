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
'''
Maximum vertical y movement is -386mm (Downward)
Minimum vertical y movement is -75mm (Upward)
'''

# Robot parameters
L1 = 195  # Length of first segment
L2 = 195  # Length of second segment

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
        #print(f"Message sent on {bus.channel_info}")
    except can.CanError:
        print("Message NOT sent")

def initialize_can_bus(channel='can0', bustype='socketcan'):
    bus = can.interface.Bus(channel=channel, bustype=bustype)
    bus.flush_tx_buffer() # Clears CAN-BUS buffer
    return bus

def enable_motor_mode(bus, can_id):
    send_can_message(bus, can_id, b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC')

def disable_motor_mode(bus, can_id):
    send_can_message(bus, can_id, b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD')

def set_zero_position(bus, can_id):
    send_can_message(bus, can_id, b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE')

def send_motor_command(bus, can_id, position, velocity=0, kp=3.2, kd=0.6, torque=0):
    packed_data = pack_commands(position, velocity, kp, kd, torque)
    send_can_message(bus, can_id, packed_data)

def inverse_kinematics(x_target, y_target):
    r = math.sqrt(x_target**2 + y_target**2)
    
    if r > L1 + L2:
        return None  # Target is out of reach
    
    theta2 = math.acos((r**2 - L1**2 - L2**2) / (2 * L1 * L2))
    theta1 = math.atan2(y_target, x_target) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    
    # Convert to degrees
    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)
    
    # Apply offsets
    theta1_deg += 180  # -90 degrees offset
    theta2_deg -= 90   # 90 degrees offset
    
    return theta1_deg, theta2_deg

# Add the forward kinematics function
def forward_kinematics(theta1, theta2):
    # Convert angles to radians and remove offsets
    theta1_rad = math.radians(theta1 - 180)
    theta2_rad = math.radians(theta2 + 90)
    
    # Calculate end-effector position
    x = L1 * math.cos(theta1_rad) + L2 * math.cos(theta1_rad + theta2_rad)
    y = L1 * math.sin(theta1_rad) + L2 * math.sin(theta1_rad + theta2_rad)
    
    return x, y
    
def interpolate(start, end, t):
    return start + (end - start) * t

def ease_in_out_cubic(t):
    return 4 * t * t * t if t < 0.5 else 1 - pow(-2 * t + 2, 3) / 2

def walking_trajectory(center_x, center_y, stride_length, step_height, t, direction=1):
    """
    Generates a smooth walking trajectory with fully interpolated steps.
    
    center_x: Center X position for the stride
    center_y: The base Y position (ground level)
    stride_length: Total horizontal distance covered in one step
    step_height: Maximum height of the step
    t: Normalized time [0, 1] for one cycle of motion
    direction: 1 for forward, -1 for backward
    """
    # X position (smooth linear movement)
    x = center_x + direction * stride_length * (t - 0.5)
    
    # Y position (smooth step cycle)
    if t < 0.5:  # Lift and forward/backward phase
        y = center_y + step_height * ease_in_out_cubic(t * 2)
    else:  # Plant phase
        y = center_y + step_height * (1 - ease_in_out_cubic((t - 0.5) * 2))
    
    return x, y

def smooth_motor_control(bus, motor1_id, motor2_id, start_angles, end_angles, duration, steps):
    start_time = time.time()
    
    for step in range(steps):
        t = step / (steps - 1)
        
        # Interpolate between start and end angles
        current_angle1 = interpolate(start_angles[0], end_angles[0], t)
        current_angle2 = interpolate(start_angles[1], end_angles[1], t)
        
        #Negate the thigh and knee angles for left leg
        if motor1_id in [0x005]:  # Check if it's left thigh motor.  CHANGE THIS ID IF NEEDED!
            current_angle1 = -current_angle1
        if motor2_id in [0x006]:  # Check if it's left knee motor.  CHANGE THIS ID IF NEEDED!
            current_angle2 = -current_angle2

        # Send motor commands
        send_motor_command(bus, motor1_id, current_angle1)
        send_motor_command(bus, motor2_id, current_angle2)
        
        # Calculate sleep time to maintain consistent duration
        elapsed = time.time() - start_time
        target_time = duration * t
        sleep_time = max(0, target_time - elapsed)
        time.sleep(sleep_time)
        

def right_leg_step(thigh_can_id, knee_can_id, canbus, duration, direction, center_y, stride_length=250, step_height=90):
    """
    Perform one complete leg step cycle, starting and ending at t=0.35 of the gait cycle.
    
    :param thigh_can_id: CAN ID for the thigh motor
    :param knee_can_id: CAN ID for the knee motor
    :param canbus: CAN bus object
    :param duration: Duration of one step cycle in seconds
    :param direction: 1 for forward, -1 for backward
    :param center_y: The base Y position (ground level)
    :param stride_length: Length of the stride (default 180)
    :param step_height: Height of the step (default 80)
    """
    center_x = 0
    steps_per_cycle = 100
    last_angles = None
    start_time = time.time()
    start_t = 0.55

    # Move to the starting position (t=0.55)
    start_x, start_y = walking_trajectory(center_x, center_y, stride_length, step_height, start_t, direction)
    start_result = inverse_kinematics(start_x, start_y)
    if start_result:
        theta1_deg, theta2_deg = start_result
        theta1_deg = max(-90, min(90, theta1_deg))
        theta2_deg = max(-55, min(55, theta2_deg))
        start_angles = (theta1_deg, theta2_deg)
        smooth_motor_control(canbus, thigh_can_id, knee_can_id, 
                             (0, 55), start_angles, 
                             duration / steps_per_cycle, 10)
        last_angles = start_angles

    # Perform the full gait cycle
    while time.time() - start_time < duration:
        current_time = time.time()
        elapsed_time = current_time - start_time
        t_normalized = (elapsed_time / duration + start_t) % 1.0  # Start at 'start_t' and wrap around to 1

        target_x, target_y = walking_trajectory(center_x, center_y, stride_length, step_height, t_normalized, direction)

        result = inverse_kinematics(target_x, target_y)
        if result:
            theta1_deg, theta2_deg = result
            
            # Apply angle limits
            theta1_deg = max(-90, min(90, theta1_deg))
            theta2_deg = max(-55, min(55, theta2_deg))
            
            target_angles = (theta1_deg, theta2_deg)
            
            smooth_motor_control(canbus, thigh_can_id, knee_can_id, 
                                 last_angles, target_angles, 
                                 duration / steps_per_cycle, 10)
            
            last_angles = target_angles

    # Ensure the leg ends at start_t
    final_x, final_y = walking_trajectory(center_x, center_y, stride_length, step_height, start_t, direction)
    final_result = inverse_kinematics(final_x, final_y)
    if final_result:
        theta1_deg, theta2_deg = final_result
        theta1_deg = max(-90, min(90, theta1_deg))
        theta2_deg = max(-55, min(55, theta2_deg))
        final_angles = (theta1_deg, theta2_deg)
        smooth_motor_control(canbus, thigh_can_id, knee_can_id, 
                             last_angles, final_angles, 
                             duration / steps_per_cycle, 10)

    # Add a small delay to ensure the motion is complete
    time.sleep(0.05)


def left_leg_step(thigh_can_id, knee_can_id, canbus, duration, direction, center_y, stride_length=250, step_height=90):
    """
    Perform one complete leg step cycle, starting and ending at t=0.35 of the gait cycle.
    
    :param thigh_can_id: CAN ID for the thigh motor
    :param knee_can_id: CAN ID for the knee motor
    :param canbus: CAN bus object
    :param duration: Duration of one step cycle in seconds
    :param direction: 1 for forward, -1 for backward
    :param center_y: The base Y position (ground level)
    :param stride_length: Length of the stride (default 180)
    :param step_height: Height of the step (default 80)
    """
    center_x = 0
    steps_per_cycle = 100
    last_angles = None
    start_time = time.time()
    start_t = 0.55

    # Move to the starting position (t=0.55)
    start_x, start_y = walking_trajectory(center_x, center_y, stride_length, step_height, start_t, direction)
    start_result = inverse_kinematics(start_x, start_y)
    if start_result:
        theta1_deg, theta2_deg = start_result
        theta1_deg = max(-90, min(90, theta1_deg))
        theta2_deg = max(-55, min(55, theta2_deg))
        start_angles = (theta1_deg, theta2_deg)
        smooth_motor_control(canbus, thigh_can_id, knee_can_id, 
                             (0, 55), start_angles, 
                             duration / steps_per_cycle, 10)
        last_angles = start_angles

    # Perform the full gait cycle
    while time.time() - start_time < duration:
        current_time = time.time()
        elapsed_time = current_time - start_time
        t_normalized = (elapsed_time / duration + start_t) % 1.0  # Start at 'start_t' and wrap around to 1

        target_x, target_y = walking_trajectory(center_x, center_y, stride_length, step_height, t_normalized, direction)

        result = inverse_kinematics(target_x, target_y)
        if result:
            theta1_deg, theta2_deg = result
        
            # Apply angle limits
            theta1_deg = max(-90, min(90, theta1_deg))
            theta2_deg = max(-55, min(55, theta2_deg))
            
            target_angles = (theta1_deg, theta2_deg)
            
            smooth_motor_control(canbus, thigh_can_id, knee_can_id, 
                                 last_angles, target_angles, 
                                 duration / steps_per_cycle, 10)
            
            last_angles = target_angles

    # Ensure the leg ends at start_t
    final_x, final_y = walking_trajectory(center_x, center_y, stride_length, step_height, start_t, direction)
    final_result = inverse_kinematics(final_x, final_y)
    if final_result:
        theta1_deg, theta2_deg = final_result
        theta1_deg = max(-90, min(90, theta1_deg))
        theta2_deg = max(-55, min(55, theta2_deg))
        final_angles = (theta1_deg, theta2_deg)
        smooth_motor_control(canbus, thigh_can_id, knee_can_id, 
                             last_angles, final_angles, 
                             duration / steps_per_cycle, 10)

    # Add a small delay to ensure the motion is complete
    time.sleep(0.05)
