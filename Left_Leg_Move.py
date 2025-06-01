import can
import time
import math
import struct
from Leg_Movement_Functions import left_leg_step, initialize_can_bus, enable_motor_mode, disable_motor_mode, set_zero_position, send_motor_command

# Initialize CAN bus
can_bus = initialize_can_bus('can0')

# Set up motors
motor1_can_id = 0x004
motor2_can_id = 0x005
motor3_can_id = 0x006

disable_motor_mode(can_bus, motor1_can_id)
disable_motor_mode(can_bus, motor2_can_id)
disable_motor_mode(can_bus, motor3_can_id)
time.sleep(1)

set_zero_position(can_bus, motor1_can_id)
set_zero_position(can_bus, motor2_can_id)
set_zero_position(can_bus, motor3_can_id)
print("Motors Zeroed")
time.sleep(1)

enable_motor_mode(can_bus, motor1_can_id)
enable_motor_mode(can_bus, motor2_can_id)
enable_motor_mode(can_bus, motor3_can_id)
print("Motors On")
time.sleep(1)

# Calibrate motors
send_motor_command(can_bus, motor1_can_id, 30)
send_motor_command(can_bus, motor2_can_id, 0)
send_motor_command(can_bus, motor3_can_id, -60)
time.sleep(1)

#set_zero_position(can_bus, motor1_can_id)
set_zero_position(can_bus, motor2_can_id)
set_zero_position(can_bus, motor3_can_id)
print("Motors Calibrated")
time.sleep(2)

print("Ready")
time.sleep(1)

while True:
	try:
		# Perform a forward step
		left_leg_step(thigh_can_id=motor2_can_id, knee_can_id=motor3_can_id, canbus=can_bus, duration=0.5, direction=1, center_y=-260)
				
		print("#1 Done")
		
		#break
		
	except KeyboardInterrupt:
		send_motor_command(can_bus, motor2_can_id, 0)   # Stop motor on interrupt
		send_motor_command(can_bus, motor3_can_id, 60)   # Stop motor on interrupt
		time.sleep(2)
		send_motor_command(can_bus, motor1_can_id, 0)
		print("Keyboard Interrupt Detected. Stopping Motors.")
		time.sleep(3)
		break

	except Exception as e:
		print(f"An error occurred: {e}")
		break


#send_motor_command(can_bus, motor1_can_id, 0)   # Stop motor on interrupt
#send_motor_command(can_bus, motor2_can_id, -55)   # Stop motor on interrupt

#time.sleep(3)

# Disable the motors and close the CAN bus connection
#disable_motor_mode(can_bus, motor1_can_id)
disable_motor_mode(can_bus, motor2_can_id)
disable_motor_mode(can_bus, motor3_can_id)

# Close the CAN bus connection
can_bus.shutdown()
