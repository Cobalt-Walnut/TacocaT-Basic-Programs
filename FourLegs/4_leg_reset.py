import can
import time
from Leg_Functions import (
    initialize_can_bus, enable_motor_mode, disable_motor_mode, 
    set_zero_position, send_motor_command
)
from Four_Leg_Functions import setup, reset_all_motors, disable_all_motors

# Define motor IDs
MOTOR_IDS = [
    # Rear Right Leg
    0x001, 0x002, 0x003,
    # Rear Left Leg
    0x004, 0x005, 0x006,
    # Front Right Leg
    0x010, 0x011, 0x012,
    # Front Left Leg
    0x007, 0x008, 0x009
]

def main():
    # Initialize CAN bus
    can_bus = initialize_can_bus('can0')

    # Setup motors
    setup(can_bus, MOTOR_IDS)

    try:
        while True:
            print("Ok")
            time.sleep(1)

    except KeyboardInterrupt:
        print("Keyboard Interrupt Detected. Stopping Motors.")
        reset_all_motors(can_bus, MOTOR_IDS)
        time.sleep(3)

    except Exception as e:
        print(f"An error occurred: {e}")
        reset_all_motors(can_bus, MOTOR_IDS)
        time.sleep(3)

    finally:
        print("Disabling all motors and shutting down CAN bus.")
        disable_all_motors(can_bus, MOTOR_IDS)
        time.sleep(2)
        can_bus.shutdown()

if __name__ == "__main__":
    main()
