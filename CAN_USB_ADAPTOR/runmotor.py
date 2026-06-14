import serial
import time

# SAFE LIMITS
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -50.0, 50.0
T_MIN, T_MAX = -25.0, 25.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0

CMD_POWER_ON  = [0xFF]*7 + [0xFC]
CMD_POWER_OFF = [0xFF]*7 + [0xFD]


def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    x = max(min(x, x_max), x_min)
    return int((x - x_min) * ((1 << bits) - 1) / span)


class AK80_6_Driver:

    def __init__(self, port='COM5', baudrate=2000000, motor_id=1):

        self.motor_id = motor_id

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_TWO,
            timeout=0.01
        )

        print("Connected to:", port)

    def checksum(self, data):
        return sum(data) & 0xFF

    def send_can(self, data):

        packet = bytearray(20)

        packet[0:5] = [0xAA,0x55,0x01,0x01,0x01]

        packet[5] = self.motor_id & 0xFF
        packet[6] = (self.motor_id >> 8) & 0xFF

        packet[7] = 0
        packet[8] = 0

        packet[9] = len(data)

        for i in range(8):
            packet[10+i] = data[i]

        packet[19] = self.checksum(packet[2:19])

        self.ser.write(packet)

    def power_on(self):
        self.send_can(CMD_POWER_ON)
        print("Motor Enabled")

    def power_off(self):
        self.send_can(CMD_POWER_OFF)
        print("Motor Disabled")

    def send_mit(self, p, v, kp, kd, t):

        p_int  = float_to_uint(p,  P_MIN,  P_MAX, 16)
        v_int  = float_to_uint(v,  V_MIN,  V_MAX, 12)
        kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
        kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
        t_int  = float_to_uint(t,  T_MIN,  T_MAX, 12)

        data = [0]*8

        data[0] = (p_int >> 8) & 0xFF
        data[1] = p_int & 0xFF

        data[2] = (v_int >> 4) & 0xFF

        data[3] = ((v_int & 0xF)<<4) | ((kp_int>>8)&0xF)

        data[4] = kp_int & 0xFF

        data[5] = (kd_int >> 4) & 0xFF

        data[6] = ((kd_int & 0xF)<<4) | ((t_int>>8)&0xF)

        data[7] = t_int & 0xFF

        self.send_can(data)


# ==========================
# MAIN PROGRAM
# ==========================

driver = AK80_6_Driver(
    port='COM5',   # CHANGE THIS
    motor_id=1
)

print("Powering motor ON...")
driver.power_on()

time.sleep(1)

print("Sending SAFE command...")

# SAFE VALUES
p  = 3.14
v  = 0.0
kp = 1.5
kd = 0.5
t  = 0.0

for i in range(500):

    driver.send_mit(
        p,
        v,
        kp,
        kd,
        t
    )

    time.sleep(0.002)

print("Stopping motor...")

driver.power_off()