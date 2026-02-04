from dynamixel_sdk import *   # Dynamixel SDK library
import time

# -----------------------------
# user settings
# -----------------------------
DXL_ID = 2                 # Your motor ID (usually 1 or 2 by default)
DEVICENAME = '/dev/ttyUSB1'   # USB2Dynamixel or adapter port
BAUDRATE = 115200         # Baud rate (XM430 default 57600 / usually set to 115200 in XArm)
# -----------------------------
# XM430 Control Table Address
# -----------------------------
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION  = 116
ADDR_PRESENT_POSITION = 132

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# -----------------------------
# Create PortHandler and PacketHandler
# -----------------------------
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(2.0)

# -----------------------------
# Open port
# -----------------------------
if portHandler.openPort():
    print("✔ Port opened")
else:
    raise Exception("❌ Failed to open port")

# -----------------------------
# Set baudrate
# -----------------------------
if portHandler.setBaudRate(BAUDRATE):
    print("✔ Baudrate set:", BAUDRATE)
else:
    raise Exception("❌ Failed to set baudrate")

# -----------------------------
# Enable torque
# -----------------------------
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
    portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
)

if dxl_comm_result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print(packetHandler.getRxPacketError(dxl_error))
else:
    print("✔ Torque enabled!")

# -----------------------------
# Set goal position
# -----------------------------
goal_pos = 2000
packetHandler.write4ByteTxRx(
    portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_pos
)
print("👉 Motor moving to:", goal_pos)

# -----------------------------
# Read present position
# -----------------------------
present_pos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
    portHandler, DXL_ID, ADDR_PRESENT_POSITION
)
print("📌 Present Position:", present_pos)

for pos in [500, 2500, 1000, 2000]:
    print("Move to:", pos)
    packetHandler.write4ByteTxRx(portHandler, DXL_ID, 116, pos)
    time.sleep(1)


# -----------------------------
# Disable torque
# -----------------------------
packetHandler.write1ByteTxRx(
    portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
)
print("🔌 Torque disabled")

# -----------------------------
# Close port
# -----------------------------
portHandler.closePort()
print("✔ Port closed")
