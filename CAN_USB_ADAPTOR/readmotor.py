import serial
import time
import struct

# =========================
# SERIAL CONFIG
# =========================
PORT = 'COM5'
BAUD = 2000000
MOTOR_ID = 0x01

# =========================
# MIT LIMITS
# =========================
P_MIN = -12.5
P_MAX = 12.5

V_MIN = -50.0
V_MAX = 50.0

T_MIN = -25.0
T_MAX = 25.0


# =========================
# FLOAT CONVERSION
# =========================
def uint_to_float(x, x_min, x_max, bits):
    span = x_max - x_min
    return float(x * span / ((1 << bits) - 1) + x_min)


def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    x = max(min(x, x_max), x_min)
    return int((x - x_min) * ((1 << bits) - 1) / span)


# =========================
# WAVESHARE DRIVER
# =========================
class USB_CAN_A:

    def __init__(self):
        self.ser = serial.Serial(
            PORT,
            BAUD,
            timeout=0.1
        )

        print("Connected to USB-CAN-A")

        self.configure_adapter()

    def checksum(self, data):
        return sum(data) & 0xFF

    def configure_adapter(self):

        cmd = bytearray([
            0xAA, 0x55,
            0x12,
            0x01,
            0x01,
            0,0,0,0,0,0,0,0,
            0x00,
            0x01,
            0,0,0,0
        ])

        cmd.append(self.checksum(cmd[2:]))

        self.ser.write(cmd)

        time.sleep(0.1)

        print("Adapter Configured")

    def send_can_frame(self, data):

        packet = bytearray(20)

        packet[0] = 0xAA
        packet[1] = 0x55
        packet[2] = 0x01
        packet[3] = 0x01
        packet[4] = 0x01

        packet[5] = MOTOR_ID & 0xFF
        packet[6] = (MOTOR_ID >> 8) & 0xFF

        packet[7] = 0
        packet[8] = 0

        packet[9] = 8

        for i in range(8):
            packet[10+i] = data[i]

        packet[19] = self.checksum(packet[2:19])

        self.ser.write(packet)

    def enable_motor(self):
        cmd = [0xFF]*7 + [0xFC]
        self.send_can_frame(cmd)
        print("Motor Enabled")

    def disable_motor(self):
        cmd = [0xFF]*7 + [0xFD]
        self.send_can_frame(cmd)
        print("Motor Disabled")

    def send_zero_command(self):

        p = 0
        v = 0
        kp = 0
        kd = 0
        t = 0

        p_int  = float_to_uint(p, P_MIN, P_MAX, 16)
        v_int  = float_to_uint(v, V_MIN, V_MAX, 12)
        kp_int = 0
        kd_int = 0
        t_int  = float_to_uint(t, T_MIN, T_MAX, 12)

        data = [0]*8

        data[0] = p_int >> 8
        data[1] = p_int & 0xFF

        data[2] = v_int >> 4
        data[3] = ((v_int & 0xF) << 4)

        data[4] = 0

        data[5] = 0

        data[6] = ((t_int >> 8) & 0xF)

        data[7] = t_int & 0xFF

        self.send_can_frame(data)

    def read_feedback(self):

        raw = self.ser.read(20)

        if len(raw) != 20:
            return None

        try:

            data = raw[10:18]

            p_int = (data[1] << 8) | data[2]

            v_int = (data[3] << 4) | (data[4] >> 4)

            t_int = ((data[4] & 0xF) << 8) | data[5]

            position = uint_to_float(p_int, P_MIN, P_MAX, 16)

            velocity = uint_to_float(v_int, V_MIN, V_MAX, 12)

            torque = uint_to_float(t_int, T_MIN, T_MAX, 12)

            return position, velocity, torque

        except:
            return None


# =========================
# MAIN
# =========================
driver = USB_CAN_A()

driver.enable_motor()

time.sleep(1)

print("\nRotate motor slowly by hand.\n")

try:

    while True:

        # send passive zero command
        driver.send_zero_command()

        feedback = driver.read_feedback()

        if feedback:

            p, v, t = feedback

            print(
                f"\rPosition: {p:.3f} rad | "
                f"Velocity: {v:.3f} rad/s | "
                f"Torque: {t:.3f} Nm",
                end=""
            )

        time.sleep(0.02)

except KeyboardInterrupt:

    print("\nStopping...")

    driver.disable_motor()