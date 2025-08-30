import tkinter as tk
from tkinter import ttk
import can
import struct
import math

# Motor Constants
P_MIN, P_MAX = -95.5, 95.5  # in radians
V_MIN, V_MAX = -45.0, 45.0  # in radians per second
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -18.0, 18.0  # T for Torque (FF_Current)

can_channel = 'can0'

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

def send_can_message(can_id, data):
    try:
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        bus.send(msg)
        print(f"Message sent on {bus.channel_info}")
    except can.CanError:
        print("Message NOT sent")

class MotorControlGUI:
    def __init__(self, master):
        self.master = master
        master.title("Motor Control GUI")

        self.create_input_field("CAN ID (hex):", 0)
        self.can_id_hex.insert(0, "001")  # Default CAN ID

        self.create_input_field("Position (deg):", 1)
        self.create_input_field("Velocity (deg/s):", 2)
        self.create_input_field("Kp:", 3)
        self.create_input_field("Kd:", 4)
        self.create_input_field("Torque:", 5)

        self.create_button("Enable Motor Mode", self.enable_motor_mode, 6)
        self.create_button("Disable Motor Mode", self.disable_motor_mode, 7)
        self.create_button("Set Zero Position", self.set_zero_position, 8)
        self.create_button("Send Command", self.send_command, 9)

    def create_input_field(self, label, row):
        ttk.Label(self.master, text=label).grid(row=row, column=0, sticky="e", padx=5, pady=5)
        entry = ttk.Entry(self.master)
        entry.grid(row=row, column=1, padx=5, pady=5)
        setattr(self, label.lower().replace(" ", "_").replace("(", "").replace(")", "").replace(":", ""), entry)

    def create_button(self, text, command, row):
        ttk.Button(self.master, text=text, command=command).grid(row=row, column=0, columnspan=2, pady=5)

    def get_can_id(self):
        try:
            return int(self.can_id_hex.get(), 16)
        except ValueError:
            print("Invalid CAN ID. Using default 0x001.")
            return 0x001

    def enable_motor_mode(self):
        send_can_message(self.get_can_id(), b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC')

    def disable_motor_mode(self):
        send_can_message(self.get_can_id(), b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD')

    def set_zero_position(self):
        send_can_message(self.get_can_id(), b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE')

    def send_command(self):
        try:
            position = float(self.position_deg.get())
            velocity = float(getattr(self, 'velocity_deg/s').get())
            kp = float(self.kp.get())
            kd = float(self.kd.get())
            torque = float(self.torque.get())

            packed_data = pack_commands(position, velocity, kp, kd, torque)
            send_can_message(self.get_can_id(), packed_data)
        except ValueError:
            print("Please enter valid numbers for all fields.")
        except AttributeError as e:
            print(f"Attribute error: {e}. Check the field names.")

if __name__ == "__main__":
    bus = can.interface.Bus(channel=can_channel, bustype='socketcan')
    root = tk.Tk()
    gui = MotorControlGUI(root)
    root.mainloop()
