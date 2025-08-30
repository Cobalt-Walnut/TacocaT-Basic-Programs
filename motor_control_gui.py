import tkinter as tk
from tkinter import ttk
import can
from mit_motor_interface import MITMotorInterface

class MotorControlGUI:
    def __init__(self, master):
        self.master = master
        master.title("GIM8108 Motor Control")

        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.motor = MITMotorInterface(0x01, self.bus)

        self.create_widgets()

    def create_widgets(self90):
        # Enable/Disable Motor Mode
        self.mode_button = ttk.Button(self.master, text="Enable Motor Mode", command=self.toggle_motor_mode)
        self.mode_button.grid(row=0, column=0, columnspan=2, pady=10)

        # Position Control
        ttk.Label(self.master, text="Position (deg):").grid(row=1, column=0)
        self.position_entry = ttk.Entry(self.master)
        self.position_entry.grid(row=1, column=1)

        # Velocity Control
        ttk.Label(self.master, text="Velocity (deg/s):").grid(row=2, column=0)
        self.velocity_entry = ttk.Entry(self.master)
        self.velocity_entry.grid(row=2, column=1)

        # Torque Control
        ttk.Label(self.master, text="Torque:").grid(row=3, column=0)
        self.torque_entry = ttk.Entry(self.master)
        self.torque_entry.grid(row=3, column=1)

        # Kp Control
        ttk.Label(self.master, text="Kp:").grid(row=4, column=0)
        self.kp_entry = ttk.Entry(self.master)
        self.kp_entry.grid(row=4, column=1)

        # Kd Control
        ttk.Label(self.master, text="Kd:").grid(row=5, column=0)
        self.kd_entry = ttk.Entry(self.master)
        self.kd_entry.grid(row=5, column=1)

        # Send Command Button
        self.send_button = ttk.Button(self.master, text="Send Command", command=self.send_command)
        self.send_button.grid(row=6, column=0, columnspan=2, pady=10)

        # Feedback Display
        self.feedback_label = ttk.Label(self.master, text="Motor Feedback:")
        self.feedback_label.grid(row=7, column=0, columnspan=2)

    def toggle_motor_mode(self):
        if self.mode_button.cget('text') == "Enable Motor Mode":
            self.motor.enable_motor_mode()
            self.mode_button.config(text="Disable Motor Mode")
        else:
            self.motor.disable_motor_mode()
            self.mode_button.config(text="Enable Motor Mode")

    def send_command(self):
        try:
            pos = float(self.position_entry.get())
            vel = float(self.velocity_entry.get())
            torque = float(self.torque_entry.get())
            kp = float(self.kp_entry.get())
            kd = float(self.kd_entry.get())
            self.motor.send_motor_cmd(pos, vel, torque, kp, kd)
        except ValueError:
            print("Invalid input. Please enter numeric values.")

    def update_feedback(self):
        self.motor.read_motor_feedback()
        feedback_text = f"Position: {self.motor.get_cur_position():.2f} deg\n"
        feedback_text += f"Speed: {self.motor.get_cur_speed():.2f} deg/s"
        self.feedback_label.config(text=feedback_text)
        self.master.after(200, self.update_feedback)

def main():
    root = tk.Tk()
    app = MotorControlGUI(root)
    app.update_feedback()
    root.mainloop()

if __name__ == "__main__":
    main()
