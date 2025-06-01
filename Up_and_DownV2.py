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
    return can.interface.Bus(channel=channel, bustype=bustype)

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

# Main Execution Block
if __name__ == "__main__":
    # Initialize the CAN bus
    can_bus = initialize_can_bus('can0')  # Adjust channel if necessary
    
    # Set the CAN ID for the motors
    motor1_can_id = 0x002
    motor2_can_id = 0x003
    
    disable_motor_mode(can_bus, motor1_can_id)
    disable_motor_mode(can_bus, motor2_can_id)  
   
    time.sleep(1)  
   
    # Zero the motors
    set_zero_position(can_bus, motor1_can_id)
    set_zero_position(can_bus, motor2_can_id)
    
    time.sleep(1)  
   
    # Enable the motors
    enable_motor_mode(can_bus, motor1_can_id)
    enable_motor_mode(can_bus, motor2_can_id)
    
    print("Motors On") 
    
    time.sleep(1)  
   
    # Calibrate the motors
    send_motor_command(can_bus, motor1_can_id, 0)
    send_motor_command(can_bus, motor2_can_id, 55)
    print("Motors Calibrated")
    time.sleep(1)  
   
    # Zero the motors
    set_zero_position(can_bus, motor1_can_id)
    set_zero_position(can_bus, motor2_can_id)
    print("Motors Zeroed")
    
    time.sleep(2)
    
    print("Starting Code")
    
    while True:
        try:
            print("Up")
            for target_y in range(-75, -386, -1):
                result = inverse_kinematics(0, target_y)
                if result:
                    theta1_deg, theta2_deg = result
                    
                    #print(f"Calculated Angles: θ1=a{theta1_deg:.2f}, θ2={theta2_deg:.2f}")
                    #print(f"Y Axis Distance in mm: {target_y}")
                    
                    # Verify with forward kinematics
                    x_verify, y_verify = forward_kinematics(theta1_deg, theta2_deg)
                    #print(f"Verified Position: x={x_verify:.2f}, y={y_verify:.2f}")
                    
                    # Check if the verified position matches the target
                    #if abs(x_verify) < 1 and abs(y_verify - target_y) < 1:
                        #print("Verification: SUCCESS")
                    #else:
                        #print("Verification: FAILED")
                    
                    # Send motor commands based on calculated angles
                    send_motor_command(can_bus, motor1_can_id, theta1_deg)
                    send_motor_command(can_bus, motor2_can_id, theta2_deg)
                #else:
                    #print("Target is out of reach.")
                
                #print("---")  # Separator for readability
                time.sleep(0.001)  # ~1.5 seconds
            
            time.sleep(0.25) # Up and Down delay time (seconds)
            
            print("Down")
            
            for target_y in range(-386, -75, 1):
                result = inverse_kinematics(0, target_y)
                if result:
                    theta1_deg, theta2_deg = result
                    
                    #print(f"Calculated Angles: θ1={theta1_deg:.2f}, θ2={theta2_deg:.2f}")
                    #print(f"Y Axis Distance in mm: {target_y}")
                    
                    # Verify with forward kinematics
                    x_verify, y_verify = forward_kinematics(theta1_deg, theta2_deg)
                    #print(f"Verified Position: x={x_verify:.2f}, y={y_verify:.2f}")
                    
                    # Check if the verified position matches the target
                    #if abs(x_verify) < 1 and abs(y_verify - target_y) < 1:
                        #print("Verification: SUCCESS")
                    #else:
                        #print("Verification: FAILED")
                    
                    # Send motor commands based on calculated angles
                    send_motor_command(can_bus, motor1_can_id, theta1_deg)
                    send_motor_command(can_bus, motor2_can_id, theta2_deg)
                #else:
                    #print("Target is out of reach.")
                
                #print("---")  # Separator for readability
                time.sleep(0.001)  # Buffer time between degrees
            time.sleep(0.25)
        #send_motor_command(can_bus, motor1_can_id,0)   # Stop motor on interrupt
        #send_motor_command(can_bus, motor2_can_id,0)   # Stop motor on interrupt
        #time.sleep(3)
        except KeyboardInterrupt:
            send_motor_command(can_bus, motor1_can_id,0)   # Stop motor on interrupt
            send_motor_command(can_bus, motor2_can_id,-55)   # Stop motor on interrupt
            print("Keyboard Interrupt Detected. Stopping Motors.")
            time.sleep(3)
            break

        except Exception as e:
            print(f"An error occurred: {e}")
        
    
    # Disable the motors and close the CAN bus connection
    disable_motor_mode(can_bus, motor1_can_id)
    disable_motor_mode(can_bus, motor2_can_id)

    # Close the CAN bus connection
    can_bus.shutdown()
