from Leg_Functions import enable_motor_mode, disable_motor_mode, set_zero_position, send_motor_command
import time

def setup(can_bus, motor_ids):
    """Setup: disable motors, zero them, enable, and calibrate."""
    
    # --- Disable each motor separately
    for motor_id in motor_ids:
        disable_motor_mode(can_bus, motor_id)
    time.sleep(5)
    
    # --- Set zero position for each motor separately
    for motor_id in motor_ids:
        set_zero_position(can_bus, motor_id)
    time.sleep(5)
    
    print("Motors Zeroed")

    # --- Enable each motor separately
    for motor_id in motor_ids:
        enable_motor_mode(can_bus, motor_id)
    time.sleep(1)
    
    print("Motors On")

    # --- Send individual motor calibration commands
    send_motor_command(can_bus, motor_ids[0], -18)  # Rear Right 1
    send_motor_command(can_bus, motor_ids[1], 0)    # Rear Right 2
    send_motor_command(can_bus, motor_ids[2], 0)    # Rear Right 3
    
    send_motor_command(can_bus, motor_ids[3], 18)   # Rear Left 1
    send_motor_command(can_bus, motor_ids[4], 0)    # Rear Left 2
    send_motor_command(can_bus, motor_ids[5], 0)    # Rear Left 3
    
    send_motor_command(can_bus, motor_ids[6], 18)   # Front Right 1
    send_motor_command(can_bus, motor_ids[7], 0)    # Front Right 2
    send_motor_command(can_bus, motor_ids[8], -60)  # Front Right 3
    
    send_motor_command(can_bus, motor_ids[9], -18)  # Front Left 1
    send_motor_command(can_bus, motor_ids[10], 0)   # Front Left 2
    send_motor_command(can_bus, motor_ids[11], 0)   # Front Left 3

def reset_all_motors(can_bus, motor_ids):
    """Reset all motors: disable and optionally re-enable."""
    
    for motor_id in motor_ids:
        disable_motor_mode(can_bus, motor_id)
    time.sleep(2)

    # Send motor command separately for each motor (if needed in reset)
    send_motor_command(can_bus, motor_ids[0], 0)
    send_motor_command(can_bus, motor_ids[1], 0)
    send_motor_command(can_bus, motor_ids[2], 0)
    send_motor_command(can_bus, motor_ids[3], 0)
    send_motor_command(can_bus, motor_ids[4], 0)
    send_motor_command(can_bus, motor_ids[5], 0)
    send_motor_command(can_bus, motor_ids[6], 0)
    send_motor_command(can_bus, motor_ids[7], 0)
    send_motor_command(can_bus, motor_ids[8], 0)
    send_motor_command(can_bus, motor_ids[9], 0)
    send_motor_command(can_bus, motor_ids[10], 0)
    send_motor_command(can_bus, motor_ids[11], 0)

def disable_all_motors(can_bus, motor_ids):
    """Fully disable all motors."""
    
    for motor_id in motor_ids:
        disable_motor_mode(can_bus, motor_id)
    time.sleep(2)

    # Optionally stop all motors individually
    send_motor_command(can_bus, motor_ids[0], 0)
    send_motor_command(can_bus, motor_ids[1], 0)
    send_motor_command(can_bus, motor_ids[2], 0)
    send_motor_command(can_bus, motor_ids[3], 0)
    send_motor_command(can_bus, motor_ids[4], 0)
    send_motor_command(can_bus, motor_ids[5], 0)
    send_motor_command(can_bus, motor_ids[6], 0)
    send_motor_command(can_bus, motor_ids[7], 0)
    send_motor_command(can_bus, motor_ids[8], 0)
    send_motor_command(can_bus, motor_ids[9], 0)
    send_motor_command(can_bus, motor_ids[10], 0)
    send_motor_command(can_bus, motor_ids[11], 0)
