# torque_limited_simple_pendulum

Section 1 — CAN Interface Setup
Bring up CAN interface
bash
# Bring up can0 at 1Mbps (AK series default from manual page 8)
sudo ip link set can0 up type can bitrate 1000000

# Bring up with auto-restart on bus-off (recommended always)
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100

# Increase TX queue to prevent buffer overflow
sudo ip link set can0 txqueuelen 1000

# Bring down
sudo ip link set can0 down
Same for can1
bash
sudo ip link set can1 up type can bitrate 1000000 restart-ms 100
sudo ip link set can1 txqueuelen 1000
sudo ip link set can1 down
Loopback mode (test chip without motor)
bash
# Enable loopback
sudo ip link set can0 up type can bitrate 1000000 loopback on

# Disable loopback (back to normal)
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
Check interface status
bash
# Basic status
ip link show can0

# Detailed with error counters
ip -details link show can0

# Full with TX/RX statistics
ip -d -s link show can0

# Watch live — refreshes every second
watch -n 1 'ip -d -s link show can0'
What the states mean
ERROR-ACTIVE   → healthy, normal operation
ERROR-WARNING  → error counter > 96, still working but degraded
ERROR-PASSIVE  → error counter > 127, limited transmission
BUS-OFF        → error counter > 255, chip stopped completely
STOPPED        → interface is down
Check hardware initialization
bash
# See if MCP2515 chips initialized correctly
sudo dmesg | grep mcp

# Good output looks like:
# mcp251x spi1.1 can0: MCP2515 successfully initialized.
# mcp251x spi1.2 can1: MCP2515 successfully initialized.
Check config.txt overlays
bash
grep -n "mcp\|spi1" /boot/firmware/config.txt
List all network interfaces including CAN
bash
ls /sys/class/net
# Should show: can0  can1  eth0  lo  wlan0
Section 2 — candump (listening to CAN bus)
Basic listen
bash
# Listen on can0, show everything
candump can0

# Listen on can1
candump can1

# Show with timestamps (delta between frames)
candump can0 -t d

# Show with absolute timestamps
candump can0 -t a

# Show with relative timestamps from start
candump can0 -t r
Filter by CAN ID
bash
# Only show frames from motor ID 1 (0x001)
candump can0 can0:001:7FF

# Only show frames from ID 0 (motor replies in MIT mode)
candump can0 can0:000:7FF
Save to file
bash
# Record everything to a file
candump can0 -l candump.log

# Play back a recording
canplayer -I candump.log
Useful monitoring during motor operation
bash
# Watch motor in one terminal while controlling in another
candump can1 -t d | grep -v "FF FF FF FF FF FF FF"
# The grep removes your own sent packets so you only see motor replies
Section 3 — cansend (sending raw CAN frames)
Basic syntax
bash
# Format: cansend <interface> <ID>#<data>
# ID is in hex, data is hex bytes

cansend can0 001#DEADBEEF
cansend can1 001#FFFFFFFFFFFFFFFF
MIT mode special commands (from manual page 60)
bash
# Enable motor (enter MIT control mode)
cansend can1 001#FFFFFFFFFFFFFFFC

# Disable motor (exit MIT control mode)
cansend can1 001#FFFFFFFFFFFFFFFD

# Set current position as zero
cansend can1 001#FFFFFFFFFFFFFFFE

# Read current state (enable + read, motor replies once)
cansend can1 001#FFFFFFFFFFFFFFFC
Send enable repeatedly and watch for reply
bash
# Terminal 1 — watch for motor reply
candump can1 -t d

# Terminal 2 — send enable every 100ms
while true; do cansend can1 001#FFFFFFFFFFFFFFFC; sleep 0.1; done
Zero torque MIT command (hold position loosely)
bash
# pack_mit(0, 0, 0, 0, 0) = all zeros = zero torque, no stiffness
cansend can1 001#0000000000000000
Section 4 — MIT Protocol Quick Reference
From manual page 60 — special commands
Enable motor:           FF FF FF FF FF FF FF FC
Disable motor:          FF FF FF FF FF FF FF FD
Set zero position:      FF FF FF FF FF FF FF FE
Read state (no control):FF FF FF FF FF FF FF FC  (same as enable)
From manual page 60-61 — command frame layout
Byte 0      : position [15:8]    (16-bit total, range ±12.5 rad)
Byte 1      : position [7:0]
Byte 2      : velocity [11:4]    (12-bit total, range ±50 rad/s)
Byte 3[7:4] : velocity [3:0]
Byte 3[3:0] : kp [11:8]          (12-bit total, range 0-500)
Byte 4      : kp [7:0]
Byte 5      : kd [11:4]          (12-bit total, range 0-5)
Byte 6[7:4] : kd [3:0]
Byte 6[3:0] : torque [11:8]      (12-bit total, range ±25 N·m)
Byte 7      : torque [7:0]
From manual page 61 — reply frame layout
Byte 0      : Drive ID (motor ID = 1)
Byte 1      : position [15:8]
Byte 2      : position [7:0]
Byte 3      : velocity [11:4]
Byte 4[7:4] : velocity [3:0]
Byte 4[3:0] : torque [11:8]
Byte 5      : torque [7:0]
Byte 6      : temperature (raw - 40 = celsius)
Byte 7      : error code
From manual page 63 — AK70-10 parameter limits
Position : -12.5 to +12.5 rad    (±716°)
Velocity : -50.0 to +50.0 rad/s
Torque   : -25.0 to +25.0 N·m
kp       :   0.0 to 500.0
kd       :   0.0 to   5.0
CAN rate : 1 Mbps
Frame type: Standard (NOT extended)
Motor ID  : 1 (default)
From manual page 61 — error codes in reply byte 7
0 = No fault
1 = Motor over-temperature
2 = Over-current
3 = Over-voltage
4 = Under-voltage
5 = Encoder fault
6 = MOSFET over-temperature
7 = Motor stall
Section 5 — python-can Quick Reference
Open bus
python
import can

# Open can1
bus = can.interface.Bus(channel='can1', interface='socketcan')

# Always close when done
bus.shutdown()
Send a frame
python
# Standard frame (MIT mode)
msg = can.Message(
    arbitration_id=0x001,
    data=[0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC],
    is_extended_id=False    # ← critical for MIT mode
)
bus.send(msg, timeout=0.2)
Receive a frame
python
# Wait up to 1 second for any frame
msg = bus.recv(timeout=1.0)

if msg is not None:
    print(f"ID: 0x{msg.arbitration_id:03X}")
    print(f"Data: {list(msg.data)}")
    print(f"Extended: {msg.is_extended_id}")
Filter real motor replies
python
def is_real_reply(msg):
    return (len(msg.data) == 8
            and not msg.is_extended_id
            and msg.data[0] == 1        # motor ID
            and msg.data[0] != 0xFF)    # not our own echo
Section 6 — Servo Mode CAN Commands (from manual page 35-36)

The AK series also supports Servo mode which uses extended frames. Useful to know if motor is in servo mode.

CAN ID format for servo mode
Extended CAN ID = (control_mode << 8) | motor_id

Control modes:
0 = Duty cycle mode
1 = Current loop mode
2 = Current brake mode
3 = RPM/velocity mode
4 = Position mode
5 = Set origin
6 = Position-velocity mode
8 = MIT mode switch
Switch motor from servo mode to MIT mode via CAN
bash
# Send mode switch command (extended frame, mode 8)
# This tells motor to switch to MIT mode
# Then power cycle the motor
cansend can1 801#0000000000000000
# 0x801 = (8 << 8) | 1 = MIT mode command for motor ID 1
Check if motor is in servo mode (it will broadcast)
bash
# In servo mode motor auto-broadcasts every few ms
# You will see rapid frames appearing
candump can1 -t d

# If you see lots of frames with extended IDs — servo mode
# If you see nothing — MIT mode (or motor off)
Section 7 — Useful One-liner Scripts
Quick motor enable test
bash
sudo ip link set can1 down && \
sudo ip link set can1 up type can bitrate 1000000 restart-ms 100 && \
sudo ip link set can1 txqueuelen 1000 && \
candump can1 -t d &
sleep 0.5 && \
cansend can1 001#FFFFFFFFFFFFFFFC && \
sleep 2 && \
kill %1
Monitor motor reply frames only (filter out sent frames)
bash
candump can1 | grep -v "FF FF FF FF FF FF FF"
Check CAN error counters continuously
bash
watch -n 0.5 'ip -d -s link show can1 | grep -A2 "re-started\|TX:\|RX:"'
Full CAN setup in one line
bash
sudo ip link set can1 down; sudo ip link set can1 up type can bitrate 1000000 restart-ms 100; sudo ip link set can1 txqueuelen 1000; echo "can1 ready"; ip -details link show can1 | grep state
Add to bashrc for quick access
bash
echo "alias can_up='sudo ip link set can1 down 2>/dev/null; sudo ip link set can1 up type can bitrate 1000000 restart-ms 100 && sudo ip link set can1 txqueuelen 1000 && echo can1 ready'" >> ~/.bashrc
source ~/.bashrc

Then just type can_up anytime.

Section 8 — Troubleshooting Checklist
Problem: candump shows nothing
Fix:
  □ Is motor powered on? Blue LED glowing?
  □ Is 120R jumper on correct terminal block?
  □ Are H→H and L→L wires correct?
  □ Is GND connected to 24V supply negative?
  □ Is can1 (not can0) being used?

Problem: TX errors incrementing
Fix:
  □ No ACK from motor — motor off or wrong mode
  □ H/L wires swapped
  □ Missing termination jumper

Problem: Motor shows passive frames but ignores MIT enable
Fix:
  □ Motor is in servo mode
  □ Use CubeMarsTool to switch to MIT mode

Problem: No buffer space available
Fix:
  □ sudo ip link set can1 txqueuelen 1000
  □ Slow down send rate — sleep 0.05 between frames

Problem: ERROR-PASSIVE state
Fix:
  □ Motor not powered on when interface came up
  □ sudo ip link set can1 down && sudo ip link set can1 up ...

Problem: SSH not responding
Fix:
  □ Physical power cycle the Pi
  □ Wait 30 seconds then reconnect
