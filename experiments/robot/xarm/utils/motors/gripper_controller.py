from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import time


class GripperController:
    """
    Direct Dynamixel XM430 gripper controller (no XArm API required)
    """

    # XM430 control table
    ADDR_OPERATION_MODE = 11
    ADDR_TORQUE_ENABLE = 64
    ADDR_LED_RED = 65
    ADDR_GOAL_CURRENT = 102
    ADDR_GOAL_VELOCITY = 104
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_CURRENT = 126
    ADDR_PRESENT_VELOCITY = 128
    ADDR_PRESENT_POSITION = 132
    ADDR_MOVING = 122
    ADDR_MOVING_STATUS = 123

    def __init__(self, port="/dev/ttyUSB1", dxl_id=1, baudrate=115200):
        self.dxl_id = dxl_id
        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(2.0)

        # Open port
        if not self.portHandler.openPort():
            raise Exception("Failed to open port")

        # Set baudrate
        if not self.portHandler.setBaudRate(baudrate):
            raise Exception("Failed to set baudrate")

        print("GripperController initialized:", port, "ID:", dxl_id)

    # ----------------------------------------------------------------------
    # Basic operations
    # ----------------------------------------------------------------------

    def enable_torque(self):
        return self._write1(self.ADDR_TORQUE_ENABLE, 1)

    def disable_torque(self):
        return self._write1(self.ADDR_TORQUE_ENABLE, 0)

    # Set position ----------------------------------
    def set_position(self, pos: int):
        return self._write4(self.ADDR_GOAL_POSITION, pos)

    def get_position(self):
        return self._read4(self.ADDR_PRESENT_POSITION)

    # Set current -----------------------------------
    def set_current(self, current: int):
        return self._write2(self.ADDR_GOAL_CURRENT, current)

    def get_current(self):
        curr = self._read2(self.ADDR_PRESENT_CURRENT)
        # signed 16-bit conversion
        if curr > 32767:
            curr -= 65536
        return curr

    # Velocity --------------------------------------
    def get_velocity(self):
        return self._read4(self.ADDR_PRESENT_VELOCITY)

    # Operation mode --------------------------------
    def set_op_mode(self, mode: int):
        """
        Valid modes: 0 (current), 1 (velocity), 3 (position), 5 (current-based position)
        """
        return self._write1(self.ADDR_OPERATION_MODE, mode)

    # Moving ----------------------------------------
    def is_moving(self):
        return self._read1(self.ADDR_MOVING) == 1

    # LED -------------------------------------------
    def set_led(self, state: bool):
        return self._write1(self.ADDR_LED_RED, 1 if state else 0)

    # ----------------------------------------------------------------------
    # Gripper high-level actions
    # ----------------------------------------------------------------------

    def open(self, pos=300):
        """Open gripper to given position"""
        print("Opening gripper...")
        self.enable_torque()
        self.set_position(pos)
        time.sleep(0.5)

    def close(self, pos=1800, auto_stop=True, current_threshold=200):
        """
        Close gripper, stop automatically when object is detected.
        current_threshold: detection threshold for "grasped"
        """
        print("Closing gripper...")
        self.enable_torque()
        self.set_position(pos)

        if auto_stop:
            while True:
                cur = self.get_current()
                if abs(cur) > current_threshold:
                    print("Object detected! current =", cur)
                    self.stop()
                    return
                time.sleep(0.02)

    def stop(self):
        """Stop by disabling torque"""
        print("Stopping gripper")
        self.disable_torque()

    # ----------------------------------------------------------------------
    # Low-level read/write wrappers
    # ----------------------------------------------------------------------

    def _write1(self, addr, data):
        res, err = self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id, addr, data)
        return self._check(res, err)

    def _write2(self, addr, data):
        res, err = self.packetHandler.write2ByteTxRx(self.portHandler, self.dxl_id, addr, data)
        return self._check(res, err)

    def _write4(self, addr, data):
        res, err = self.packetHandler.write4ByteTxRx(self.portHandler, self.dxl_id, addr, data)
        return self._check(res, err)

    def _read1(self, addr):
        data, res, err = self.packetHandler.read1ByteTxRx(self.portHandler, self.dxl_id, addr)
        self._check(res, err)
        return data

    def _read2(self, addr):
        data, res, err = self.packetHandler.read2ByteTxRx(self.portHandler, self.dxl_id, addr)
        self._check(res, err)
        return data

    def _read4(self, addr):
        data, res, err = self.packetHandler.read4ByteTxRx(self.portHandler, self.dxl_id, addr)
        self._check(res, err)
        return data

    def _check(self, res, err):
        if res != COMM_SUCCESS:
            raise Exception(self.packetHandler.getTxRxResult(res))
        if err != 0:
            raise Exception(self.packetHandler.getRxPacketError(err))
        return True

    # ----------------------------------------------------------------------

    def close_port(self):
        self.portHandler.closePort()
        print("Port closed.")
