import sys
from dataclasses import asdict, dataclass
from time import sleep

from serial import Serial

from calibrate_imu.transport import (
    ImuData,
    ImuMsg,
    MessageType,
    MsgParser,
)

output_template: str = """## offset
<param name="imu_linear_acceleration_offset_x">{mean_acc_x}</param>
<param name="imu_linear_acceleration_offset_y">{mean_acc_y}</param>
<param name="imu_linear_acceleration_offset_z">{mean_acc_z}</param>
<param name="imu_angular_velocity_offset_x">{mean_gyro_x}</param>
<param name="imu_angular_velocity_offset_y">{mean_gyro_y}</param>
<param name="imu_angular_velocity_offset_z">{mean_gyro_z}</param>

## covariance
static_covariance_linear_acceleration:
  - {var_acc_x}
  - 0.0
  - 0.0
  - 0.0
  - {var_acc_y}
  - 0.0
  - 0.0
  - 0.0
  - {var_acc_z}
static_covariance_angular_velocity:
  - {var_gyro_x}
  - 0.0
  - 0.0
  - 0.0
  - {var_gyro_y}
  - 0.0
  - 0.0
  - 0.0
  - {var_gyro_z}"""


@dataclass(frozen=True)
class CalibrationData:
    mean_acc_x: float
    mean_acc_y: float
    mean_acc_z: float
    mean_gyro_x: float
    mean_gyro_y: float
    mean_gyro_z: float
    var_acc_x: float
    var_acc_y: float
    var_acc_z: float
    var_gyro_x: float
    var_gyro_y: float
    var_gyro_z: float


class Stats:
    def __init__(self):
        self._acc: float = 0.0
        self._sq_acc: float = 0.0
        self._count: float = 0

    def add(self, value: float) -> None:
        self._acc += value
        self._sq_acc += value * value
        self._count += 1

    def mean(self) -> float:
        return self._acc / self._count

    def variance(self) -> float:
        return self._sq_acc / self._count - self.mean() * self.mean()


class ImuMsgHandler:
    def __init__(self):
        self._acc_coef: float = 2 * 2.0 / 65536.0
        self._gyro_coeff: float = 2 * 250.0 / 65536.0

        self._acc_x_stats: Stats = Stats()
        self._acc_y_stats: Stats = Stats()
        self._acc_z_stats: Stats = Stats()
        self._gyro_x_stats: Stats = Stats()
        self._gyro_y_stats: Stats = Stats()
        self._gyro_z_stats: Stats = Stats()

    def handle(self, msg: ImuMsg) -> None:
        if msg.param == ImuData.AccelX:
            cur_acc = self._acc_coef * msg.value
            self._acc_x_stats.add(cur_acc)
        elif msg.param == ImuData.AccelY:
            cur_acc = self._acc_coef * msg.value
            self._acc_y_stats.add(cur_acc)
        elif msg.param == ImuData.AccelZ:
            cur_acc = self._acc_coef * msg.value
            self._acc_z_stats.add(cur_acc)
        elif msg.param == ImuData.GyroX:
            cur_gyro = self._gyro_coeff * msg.value
            self._gyro_x_stats.add(cur_gyro)
        elif msg.param == ImuData.GyroY:
            cur_gyro = self._gyro_coeff * msg.value
            self._gyro_y_stats.add(cur_gyro)
        elif msg.param == ImuData.GyroZ:
            cur_gyro = self._gyro_coeff * msg.value
            self._gyro_z_stats.add(cur_gyro)
        elif msg.param == ImuData.Temp:
            pass
        else:
            raise ValueError(f"Unknown IMU data: {msg.param}")

    def get_calibration_data(self) -> CalibrationData:
        return CalibrationData(
            mean_acc_x=self._acc_x_stats.mean(),
            mean_acc_y=self._acc_y_stats.mean(),
            mean_acc_z=self._acc_z_stats.mean(),
            mean_gyro_x=self._gyro_x_stats.mean(),
            mean_gyro_y=self._gyro_y_stats.mean(),
            mean_gyro_z=self._gyro_z_stats.mean(),
            var_acc_x=self._acc_x_stats.variance(),
            var_acc_y=self._acc_y_stats.variance(),
            var_acc_z=self._acc_z_stats.variance(),
            var_gyro_x=self._gyro_x_stats.variance(),
            var_gyro_y=self._gyro_y_stats.variance(),
            var_gyro_z=self._gyro_z_stats.variance(),
        )


def main(serial_port: Serial, num_samples: int = 8192) -> None:
    request_query: list[ImuMsg] = [
        ImuMsg(type=MessageType.IMU, param=ImuData.AccelX, value=0),
        ImuMsg(type=MessageType.IMU, param=ImuData.AccelY, value=0),
        ImuMsg(type=MessageType.IMU, param=ImuData.AccelZ, value=0),
        ImuMsg(type=MessageType.IMU, param=ImuData.GyroX, value=0),
        ImuMsg(type=MessageType.IMU, param=ImuData.GyroY, value=0),
        ImuMsg(type=MessageType.IMU, param=ImuData.GyroZ, value=0),
    ]
    imu_handler: ImuMsgHandler = ImuMsgHandler()
    msg_parser: MsgParser = MsgParser()

    for i in range(num_samples):
        for msg in request_query:
            serial_port.write(msg_parser.serialize(msg))
        sleep(0.01)

        cur_bytes = serial_port.read(serial_port.in_waiting)
        for msg in msg_parser.parse(cur_bytes):
            imu_handler.handle(msg)
    calibration_data = imu_handler.get_calibration_data()
    print(output_template.format(**asdict(calibration_data)))


if __name__ == "__main__":
    try:
        port = sys.argv[1]
    except IndexError:
        print("Usage: python main.py <serial-port>", file=sys.stderr)
        sys.exit(1)

    with Serial(port=port, baudrate=115200) as serial_port:
        main(serial_port)
