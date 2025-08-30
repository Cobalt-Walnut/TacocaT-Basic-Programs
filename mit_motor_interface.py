import can
import time
import math
from utils import float_to_uint, uint_to_float

class MITMotorInterface:
    P_MIN, P_MAX = -95.5, 95.5
    V_MIN, V_MAX = -45.0, 45.0
    KP_MIN, KP_MAX = 0.0, 500.0
    KD_MIN, KD_MAX = 0.0, 5.0
    T_MIN, T_MAX = -18.0, 18.0

    def __init__(self, node_id, can_bus):
        self.node_id = node_id
        self.can_bus = can_bus
        self.cur_mot_pos = 0
        self.cur_mot_speed = 0
        self.cur_torque = 0

    def init_motor(self):
        self.disable_motor_mode()
        time.sleep(1)
        self.enable_motor_mode()
        time.sleep(1)
        self.read_motor_feedback()

    def enable_motor_mode(self):
        self._send_mode_command([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], "Enabled")

    def disable_motor_mode(self):
        self._send_mode_command([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD], "Disabled")

    def _send_mode_command(self, cmd, mode):
        message = can.Message(arbitration_id=self.node_id, data=cmd, is_extended_id=False)
        try:
            self.can_bus.send(message)
            print(f"Motor Mode {mode}!")
        except can.CanError:
            print(f"Failed To {mode} Motor Mode...")

    def send_motor_cmd(self, pos, vel, torque, kp, kd):
        p_int = float_to_uint(math.radians(pos), self.P_MIN, self.P_MAX, 16)
        v_int = float_to_uint(math.radians(vel), self.V_MIN, self.V_MAX, 12)
        kp_int = float_to_uint(kp, self.KP_MIN, self.KP_MAX, 12)
        kd_int = float_to_uint(kd, self.KD_MIN, self.KD_MAX, 12)
        t_int = float_to_uint(torque, self.T_MIN, self.T_MAX, 12)

        msg = [
            (p_int >> 8) & 0xFF,
            p_int & 0xFF,
            (v_int >> 4) & 0xFF,
            ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF),
            kp_int & 0xFF,
            (kd_int >> 4) & 0xFF,
            ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF),
            t_int & 0xFF
        ]

        message = can.Message(arbitration_id=self.node_id, data=msg, is_extended_id=False)
        try:
            self.can_bus.send(message)
            print("Motor CMD Sent!")
        except can.CanError:
            print("Failed To Send Motor CMD...")

    def read_motor_feedback(self):
        message = self.can_bus.recv(timeout=1.0)
        if message is None or len(message.data) != 6:
            return

        p_int = (message.data[1] << 8) | message.data[2]
        v_int = (message.data[3] << 4) | (message.data[4] >> 4)
        t_int = ((message.data[4] & 0x0F) << 8) | message.data[5]

        self.cur_mot_pos = uint_to_float(p_int, self.P_MIN, self.P_MAX, 16)
        self.cur_mot_speed = uint_to_float(v_int, self.V_MIN, self.V_MAX, 12)
        self.cur_torque = uint_to_float(t_int, self.T_MIN, self.T_MAX, 12)

        print(f"CAN ID: {message.data[0]} p: {math.degrees(self.cur_mot_pos):.2f} [deg] "
              f"v: {math.degrees(self.cur_mot_speed):.2f} [deg/s] t: {self.cur_torque:.2f}")

    def get_cur_position(self):
        return math.degrees(self.cur_mot_pos)

    def get_cur_speed(self):
        return math.degrees(self.cur_mot_speed)
