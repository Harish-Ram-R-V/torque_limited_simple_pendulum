import serial
import time
import sys
import argparse

# MIT Limits 
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -50.0, 50.0
T_MIN, T_MAX = -25.0, 25.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0

CMD_POWER_ON  = [0xFF]*7 + [0xFC]
CMD_POWER_OFF = [0xFF]*7 + [0xFD]
CMD_SET_ZERO  = [0xFF]*7 + [0xFE]

def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    x = max(min(x, x_max), x_min)
    return int((x - x_min) * ((1 << bits) - 1) / span)

class AK70_10_Waveshare_Driver:

    def __init__(self, port='COM5', baudrate=2000000, motor_id=1):
        self.motor_id = motor_id
        self.ser = serial.Serial(
            port,
            baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_TWO,
            timeout=0.01
        )
        print("Serial connected")
        self._configure_adapter()

    def _checksum(self, data):
        return sum(data) & 0xFF

    def _configure_adapter(self):
        cmd = bytearray([0xAA, 0x55, 0x12, 0x01, 0x01, 0,0,0,0,0,0,0,0, 0x00, 0x01, 0,0,0,0])
        cmd.append(self._checksum(cmd[2:]))
        self.ser.write(cmd)
        time.sleep(0.1)

    def _send_can(self, data):
        packet = bytearray(20)
        packet[0:5] = [0xAA,0x55,0x01,0x01,0x01]
        packet[5] = self.motor_id & 0xFF
        packet[6] = (self.motor_id >> 8) & 0xFF
        packet[7] = 0
        packet[8] = 0
        packet[9] = len(data)
        for i in range(8):
            packet[10+i] = data[i]
        packet[19] = self._checksum(packet[2:19])
        self.ser.write(packet)

    def power_on(self):
        self._send_can(CMD_POWER_ON)
        print("Motor Enabled")

    def power_off(self):
        self._send_can(CMD_POWER_OFF)
        print("Motor Disabled")

    def set_zero_position(self):
        self._send_can(CMD_SET_ZERO)
        print("Zero position set at current location")
        time.sleep(0.5)

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
        self._send_can(data)

# ZERO POSITION PROMPT

def ask_set_zero(driver):
    print("\n" + "="*45)
    print("  ZERO POSITION SETUP")
    print("="*45)
    print("  Current motor position will be used as")
    print("  the zero reference if you choose YES.")
    print("="*45)

    while True:
        answer = input("\n  Do you want to set current position as zero? (yes/no): ").strip().lower()

        if answer in ("yes", "y"):
            print("\n Setting zero position...")
            driver.set_zero_position()
            print("  Motor will now treat current position as 0 rad.")
            break

        elif answer in ("no", "n"):
            print("\n Skipping zero set. Using existing zero reference.")
            break

        else:
            print("Invalid input. Please type 'yes' or 'no'.")

# CONTROL LOOP

def control_loop(driver):
    print("\n" + "="*45)
    print("  MIT Control Running. Press Ctrl+C to stop.")
    print("="*45 + "\n")

    p  = 3.14 #position target (rad)
    v  = 0.0   # velocity target (rad/s)
    kp = 3.0   # position gain
    kd = 0.5  # velocity gain (damping)
    t  = 0.0   # feedforward torque (Nm)

    try:
        while True:
            driver.send_mit(p, v, kp, kd, t)
            time.sleep(0.002)  # 500 Hz loop

    except KeyboardInterrupt:
        print("\n  Stopping control loop...")

# ---------------------------------------------------

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode",
                        default="position",
                        choices=["position", "velocity", "torque"])
    parser.add_argument("-d", "--device",
                        default="COM5")
    args = parser.parse_args()

    driver = AK70_10_Waveshare_Driver(port=args.device)

    # --- Ask user about zero position BEFORE enabling motor ---
    ask_set_zero(driver)

    print("\n WARNING: Motor may move immediately!")
    driver.power_on()
    time.sleep(1)

    control_loop(driver)

    driver.power_off()