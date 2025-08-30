import can
from mit_motor_interface import MITMotorInterface
import time

NODE_ID1 = 0x003
CAN_CHANNEL = 'can0'

def main():
    bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='socketcan')
    motor = MITMotorInterface(NODE_ID1, bus)

    motor.init_motor()

    previous_time = time.time()
    try:
        while True:
            current_time = time.time()
            if current_time - previous_time >= 2:
                previous_time = current_time
                print("Sending Motor CMD...")
                #pos, vel, tor, kp, kd
                motor.send_motor_cmd(0.0, 0.0, 0.0, 2.0, 1.0)

            motor.read_motor_feedback()
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()
