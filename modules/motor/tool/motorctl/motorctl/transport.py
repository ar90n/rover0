import struct
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, List, Optional, TypeAlias


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
    return buf


class State(Enum):
    HEADER_CHECKSUM = auto()
    DATA1 = auto()
    DATA2 = auto()
    DATA3 = auto()
    DATA4 = auto()
    DATA5 = auto()


class Transport:
    HEADER_BIT = 0x80
    DATA_MASK = 0x7F

    def __init__(self):
        self.writer: Callable[[List[int]], None] = lambda x: None
        self.state = State.HEADER_CHECKSUM
        self.buffer = bytearray(5)
        self.buffer_index = 0
        self.expected_checksum = 0

    def init(self, writer: Callable[[List[int]], None]) -> None:
        self.writer = writer

    def reset(self) -> None:
        self.state = State.HEADER_CHECKSUM
        self.buffer_index = 0

    @staticmethod
    def calculate_checksum(buffer: bytearray) -> int:
        return sum(b & Transport.DATA_MASK for b in buffer) & Transport.DATA_MASK

    def consume(self, byte: int) -> Optional[bytes]:
        ret = None

        if byte & self.HEADER_BIT:
            self.state = State.DATA1
            self.buffer_index = 0
            self.expected_checksum = byte & self.DATA_MASK
            return ret

        if self.state == State.DATA1:
            self.buffer[self.buffer_index] = byte & self.DATA_MASK
            self.buffer_index += 1
            self.state = State.DATA2

        elif self.state == State.DATA2:
            self.buffer[self.buffer_index] = byte & self.DATA_MASK
            self.buffer_index += 1
            self.state = State.DATA3

        elif self.state == State.DATA3:
            self.buffer[self.buffer_index] = byte & self.DATA_MASK
            self.buffer_index += 1
            self.state = State.DATA4

        elif self.state == State.DATA4:
            self.buffer[self.buffer_index] = byte & self.DATA_MASK
            self.buffer_index += 1
            self.state = State.DATA5

        elif self.state == State.DATA5:
            self.buffer[self.buffer_index] = byte & self.DATA_MASK
            if self.calculate_checksum(self.buffer) == self.expected_checksum:
                value = (
                    ((self.buffer[0] & self.DATA_MASK) << 28)
                    | ((self.buffer[1] & self.DATA_MASK) << 21)
                    | ((self.buffer[2] & self.DATA_MASK) << 14)
                    | ((self.buffer[3] & self.DATA_MASK) << 7)
                    | (self.buffer[4] & self.DATA_MASK)
                )
                ret = bytes([(value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
            self.state = State.HEADER_CHECKSUM

        return ret

    def send(self, data: int) -> bytes:
        buffer = bytearray(6)
        buffer[1] = (data >> 28) & self.DATA_MASK
        buffer[2] = (data >> 21) & self.DATA_MASK
        buffer[3] = (data >> 14) & self.DATA_MASK
        buffer[4] = (data >> 7) & self.DATA_MASK
        buffer[5] = data & self.DATA_MASK
        buffer[0] = self.calculate_checksum(buffer[1:]) | self.HEADER_BIT
        return buffer


class MsgParser:
    def __init__(self):
        self._transport = Transport()
        self.buf: list[int] = []

    def parse(self, byte: bytes) -> Msg:
        for b in byte:
            recv_bytes = self._transport.consume(b)
            if recv_bytes is None:
                continue

            msg = self._parse(recv_bytes)
            yield msg

    def serialize(self, msg: Msg) -> bytes:
        buf = serialize_msg(msg)
        return self._transport.send(int.from_bytes(buf, "big"))

    def _parse(self, recv: bytes) -> Msg | None:
        v, p, t = struct.unpack(">hBB", recv)

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
