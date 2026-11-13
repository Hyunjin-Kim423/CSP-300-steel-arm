"""Thin Feetech STS3215 bus wrapper on top of scservo_sdk.

Deliberately independent of lerobot/torch so it runs in the ROS system python.
scservo_sdk is installed standalone (no deps) in ~/so101_work/scs_sdk, override with SCS_SDK_PATH.
"""
import os
import sys

_SDK_DIR = os.environ.get("SCS_SDK_PATH", os.path.expanduser("~/so101_work/scs_sdk"))
if os.path.isdir(_SDK_DIR) and _SDK_DIR not in sys.path:
    sys.path.insert(0, _SDK_DIR)

from scservo_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler  # noqa: E402

# STS3215 control table addresses (same as lerobot's STS_SMS_SERIES_CONTROL_TABLE)
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56


class BusError(RuntimeError):
    pass


class FeetechBus:
    def __init__(self, port: str, ids: list[int], baudrate: int = 1_000_000):
        self.ids = list(ids)
        self._port = PortHandler(port)
        self._ph = PacketHandler(0)  # protocol_end=0 for STS series
        self._baud = baudrate
        self._reader = GroupSyncRead(self._port, self._ph, ADDR_PRESENT_POSITION, 2)
        self._writer = GroupSyncWrite(self._port, self._ph, ADDR_GOAL_POSITION, 2)
        for i in self.ids:
            self._reader.addParam(i)

    def open(self) -> None:
        if not self._port.openPort():
            raise BusError(f"cannot open {self._port.port_name}")
        if not self._port.setBaudRate(self._baud):
            raise BusError(f"cannot set baudrate {self._baud}")

    def close(self) -> None:
        self._port.closePort()

    def read_positions(self) -> dict[int, int]:
        result = self._reader.txRxPacket()
        if result != COMM_SUCCESS:
            raise BusError(f"sync read failed: {self._ph.getTxRxResult(result)}")
        out = {}
        for i in self.ids:
            if not self._reader.isAvailable(i, ADDR_PRESENT_POSITION, 2):
                raise BusError(f"no data from motor {i}")
            out[i] = self._reader.getData(i, ADDR_PRESENT_POSITION, 2)
        return out

    def write_goals(self, goals: dict[int, int]) -> None:
        self._writer.clearParam()
        for i, g in goals.items():
            g = int(max(0, min(4095, round(g))))
            self._writer.addParam(i, [g & 0xFF, (g >> 8) & 0xFF])
        result = self._writer.txPacket()
        self._writer.clearParam()
        if result != COMM_SUCCESS:
            raise BusError(f"sync write failed: {self._ph.getTxRxResult(result)}")

    def _write1(self, motor_id: int, addr: int, value: int) -> None:
        result, error = self._ph.write1ByteTxRx(self._port, motor_id, addr, value)
        if result != COMM_SUCCESS:
            raise BusError(f"write to motor {motor_id} addr {addr} failed: {self._ph.getTxRxResult(result)}")

    def set_acceleration(self, value: int) -> None:
        for i in self.ids:
            self._write1(i, ADDR_ACCELERATION, int(value))

    def set_torque(self, enable: bool, ids: list[int] | None = None) -> None:
        for i in ids or self.ids:
            self._write1(i, ADDR_TORQUE_ENABLE, 1 if enable else 0)
