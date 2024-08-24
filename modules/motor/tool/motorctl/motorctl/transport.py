import struct
from dataclasses import dataclass
from enum import Enum
from typing import TypeAlias


class MessageType(Enum):
    Motor = 0
    Encoder = 1
    IMU = 2


class MotorDevice(Enum):
    RearLeft = 0
    RearRight = 1
    FrontLeft = 2
    FrontRight = 3

    def __str__(self):
        return {
            MotorDevice.RearLeft: "Rear Left",
            MotorDevice.RearRight: "Rear Right",
            MotorDevice.FrontLeft: "Front Left",
            MotorDevice.FrontRight: "Front Right",
        }[self]

    def as_id(self):
        return str(self).replace(" ", "-").lower()


class ImuData(Enum):
    AccelX = 0
    AccelY = 1
    AccelZ = 2
    GyroX = 3
    GyroY = 4
    GyroZ = 5
    Temp = 6


@dataclass
class MotorMsg:
    type: MessageType
    param: MotorDevice
    value: int


@dataclass
class EncoderMsg:
    type: MessageType
    param: MotorDevice
    value: int


@dataclass
class ImuMsg:
    type: MessageType
    param: ImuData
    value: int


Msg: TypeAlias = MotorMsg | EncoderMsg | ImuMsg


def get_msg_type_key(msg: Msg) -> str:
    return f"{msg.type.name}-{msg.param}"


def serialize_msg(msg: Msg) -> bytes:
    t = struct.pack("B", min(msg.type.value, 255))
    p = struct.pack("B", min(msg.param.value, 255))
    v = struct.pack(">h", max(-32768, min(msg.value, 32767)))
    buf = v + p + t
    buf += struct.pack("B", sum(buf) % 256)
    return buf


class MsgParser:
    def __init__(self):
        self.buf: list[int] = []

    def parse(self, byte: bytes) -> Msg:
        self.buf.extend(byte)
        while self._has_enough_bytes():
            msg = self._consume()
            if msg is None:
                continue

            yield msg

    def _has_enough_bytes(self):
        return 5 <= len(self.buf)

    def _checksum(self, buf):
        return sum(buf) % 256

    def _consume(self) -> Msg | None:
        if self._checksum(self.buf[:4]) == self.buf[4]:
            v, p, t, c = struct.unpack(">hBBB", bytes(self.buf[:5]))
            self.buf = self.buf[5:]

            msg_type = MessageType(t)
            match msg_type:
                case MessageType.Motor:
                    return MotorMsg(msg_type, MotorDevice(p), v)
                case MessageType.Encoder:
                    return EncoderMsg(msg_type, MotorDevice(p), v)
                case MessageType.IMU:
                    return ImuMsg(msg_type, ImuData(p), v)
                case _:
                    raise ValueError(f"Unknown message type {t}")

        self.buf = self.buf[1:]
        return None
