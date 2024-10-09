import sys
import queue
import time
from enum import Enum
from time import sleep
from typing import Callable

from serial import Serial
from textual import on, work
from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.reactive import reactive
from textual.widget import Widget
from textual.widgets import Button, Footer, Header, Label, Static

from motorctl.transport import (
    EncoderMsg,
    ImuData,
    ImuMsg,
    MessageType,
    MotorDevice,
    MotorMsg,
    MsgParser,
    get_msg_type_key,
    serialize_msg,
)


class Title(Static):
    """A widget to display the title."""

    def __init__(self, title: str):
        super().__init__(title)


class MotorControlValueDisplay(Widget):
    """A widget to display the power of the drive."""

    def __init__(self, label: str, **kwargs):
        super().__init__(**kwargs)
        self._label = label

    def compose(self) -> ComposeResult:
        with Horizontal():
            yield Label(self._format_label(), classes="label")
            yield Label(classes="control-value")

    def update(self, value: float) -> None:
        self.query_one(".control-value").update(self._format_value(value))

    def _format_label(self) -> str:
        return f"{self._label}:"

    def _format_value(self, value: int) -> str:
        return f"{value}"


class MotorControl(Widget):
    """A widget to control the motor."""

    drive_power = reactive(0)
    wheel_velocity = reactive(0)

    def __init__(self, device: MotorDevice, on_driver_power_changed: Callable[[MotorDevice, int], None], **kwargs):
        super().__init__(**kwargs)
        self._device = device
        self._on_driver_power_changed = on_driver_power_changed

    def compose(self) -> ComposeResult:
        with Vertical():
            yield Title(str(self._device))
            with Horizontal():
                yield Button("<<", id="fast-backward")
                yield Button("<", id="backward")
                yield Button("||", id="stop")
                yield Button(">", id="forward")
                yield Button(">>", id="fast-forward")
                yield MotorControlValueDisplay("Power", id="drive-power")
                yield MotorControlValueDisplay("Velocity", id="wheel-velocity")

    def _format_as_id(self, name: str) -> str:
        return f"{self._device.as_id()}-{name}"

    @on(Button.Pressed, "#fast-backward")
    def drive_fast_backward(self) -> None:
        self.drive_power -= 1000
        self.drive_power = max(-32768, self.drive_power)

    @on(Button.Pressed, "#backward")
    def drive_backward(self) -> None:
        self.drive_power -= 100
        self.drive_power = max(-32768, self.drive_power)

    @on(Button.Pressed, "#stop")
    def stop(self) -> None:
        self.drive_power = 0

    @on(Button.Pressed, "#forward")
    def drive_forward(self) -> None:
        self.drive_power += 100
        self.drive_power = min(32767, self.drive_power)

    @on(Button.Pressed, "#fast-forward")
    def drive_fast_forward(self) -> None:
        self.drive_power += 1000
        self.drive_power = min(32767, self.drive_power)

    def watch_drive_power(self, value: int) -> None:
        self.query_one("#drive-power").update(str(value))
        self._on_driver_power_changed(self._device, self.drive_power)

    def watch_wheel_velocity(self, value: int) -> None:
        self.query_one("#wheel-velocity").update(str(value))


class Axis(Enum):
    X = "X"
    Y = "Y"
    Z = "Z"


class SensorOutputDisplay(Widget):
    """A widget to display the sensor output."""

    def __init__(self, axis: Axis, unit: str, **kwargs):
        super().__init__(**kwargs)
        self.axis: Axis = axis
        self.unit: str = unit

    def compose(self) -> ComposeResult:
        with Horizontal():
            yield Label(self._format_label(), classes="axis")
            yield Label(classes="sensor-value")
            yield Label(self._format_unit(), classes="unit")

    def update(self, value: float) -> None:
        self.query_one(".sensor-value").update(self._format_value(value))

    def _format_label(self) -> str:
        return f"{self.axis.value}:"

    def _format_value(self, value: float) -> str:
        return f"{value:.2f}"

    def _format_unit(self) -> str:
        return rf"\[{self.unit}]"


class AccelerometerOutput(Widget):
    """A widget to display the accelerometer data."""

    x = reactive(0.0)
    y = reactive(0.0)
    z = reactive(0.0)

    def compose(self) -> ComposeResult:
        yield Title("IMU - Accelerometer")
        yield SensorOutputDisplay(axis=Axis.X, unit="m/s^2", id="accel-x")
        yield SensorOutputDisplay(axis=Axis.Y, unit="m/s^2", id="accel-y")
        yield SensorOutputDisplay(axis=Axis.Z, unit="m/s^2", id="accel-z")

    def watch_x(self, value: float) -> None:
        self.query_one("#accel-x", SensorOutputDisplay).update(value)

    def watch_y(self, value: float) -> None:
        self.query_one("#accel-y", SensorOutputDisplay).update(value)

    def watch_z(self, value: float) -> None:
        self.query_one("#accel-z", SensorOutputDisplay).update(value)


class GyroscopeOutput(Widget):
    """A widget to display the gyroscope data."""

    x = reactive(0.0)
    y = reactive(0.0)
    z = reactive(0.0)

    def compose(self) -> ComposeResult:
        yield Title("IMU - Gyroscope")
        yield SensorOutputDisplay(axis=Axis.X, unit="rad/s^2", id="gyro-x")
        yield SensorOutputDisplay(axis=Axis.Y, unit="rad/s^2", id="gyro-y")
        yield SensorOutputDisplay(axis=Axis.Z, unit="rad/s^2", id="gyro-z")

    def watch_x(self, value: float) -> None:
        self.query_one("#gyro-x", SensorOutputDisplay).update(value)

    def watch_y(self, value: float) -> None:
        self.query_one("#gyro-y", SensorOutputDisplay).update(value)

    def watch_z(self, value: float) -> None:
        self.query_one("#gyro-z", SensorOutputDisplay).update(value)


class Sensors(Widget):
    def compose(self) -> ComposeResult:
        with Horizontal():
            yield AccelerometerOutput()
            yield GyroscopeOutput()


class MotorCtrlApp(App[None]):
    """A Textual app to manage stopwatches."""

    CSS_PATH = "app.css"
    BINDINGS = [
        ("q", "quit", "Quit"),
    ]

    _vals = {}
    _mpsc = queue.Queue()
    _should_quit = False
    _msg_parser = MsgParser()

    def __init__(self, serial_port: Serial, **kwargs):
        super().__init__(**kwargs)
        self._serial_port: Serial = serial_port
        self._acc_sensor: AccelerometerOutput | None = None
        self._gyro_sensor: GyroscopeOutput | None = None
        self._front_left_motor: MotorControl | None = None
        self._front_right_motor: MotorControl | None = None
        self._rear_left_motor: MotorControl | None = None
        self._rear_right_motor: MotorControl | None = None

    def compose(self):
        yield Header()
        yield Footer()
        yield MotorControl(MotorDevice.FrontLeft, self._on_drive_power_changed, id=MotorDevice.FrontLeft.as_id())
        yield MotorControl(MotorDevice.FrontRight, self._on_drive_power_changed, id=MotorDevice.FrontRight.as_id())
        yield MotorControl(MotorDevice.RearLeft, self._on_drive_power_changed, id=MotorDevice.RearLeft.as_id())
        yield MotorControl(MotorDevice.RearRight, self._on_drive_power_changed, id=MotorDevice.RearRight.as_id())
        yield Sensors()

    def on_mount(self) -> None:
        self._vals = {}
        self.polling_serial()
        self.writing_serial()
        self.update_ui()

        self._acc_sensor = self.query_one(AccelerometerOutput)
        self._gyro_sensor = self.query_one(GyroscopeOutput)
        self._front_left_motor = self.query_one("#front-left", MotorControl)
        self._front_right_motor = self.query_one("#front-right", MotorControl)
        self._rear_left_motor = self.query_one("#rear-left", MotorControl)
        self._rear_right_motor = self.query_one("#rear-right", MotorControl)
        self._sync_timer = self.set_interval(1 / 10, self._sync_data)

    @work(exclusive=False, thread=True)
    def polling_serial(self) -> None:
        while not self._should_quit:
            cur_bytes = self._serial_port.read(self._serial_port.in_waiting)
            for msg in self._msg_parser.parse(cur_bytes):
                self._vals[get_msg_type_key(msg)] = msg
            sleep(0.001)

    @work(exclusive=False, thread=True)
    def writing_serial(self) -> None:
        while not self._should_quit:
            try:
                msg = self._mpsc.get(False)
                self._serial_port.write(serialize_msg(msg))
            except queue.Empty:
                pass

    @work(exclusive=False, thread=True)
    def update_ui(self) -> None:
        # get system time in milliseconds
        while not self._should_quit:
            last_time_ns = time.time_ns()
            for msg in list(self._vals.values()):
                match msg:
                    case EncoderMsg(_, _, _):
                        self._handle_encoder_msg(msg)
                    case ImuMsg(_, _, _):
                        self._handle_imu_msg(msg)
            sleep(self._calc_wait_time(last_time_ns))

    def action_quit(self) -> None:
        self._should_quit = True
        self.workers.cancel_all()
        self.exit()

    def _handle_encoder_msg(self, msg: EncoderMsg) -> None:
        {
            MotorDevice.RearLeft: self._rear_left_motor,
            MotorDevice.RearRight: self._rear_right_motor,
            MotorDevice.FrontLeft: self._front_left_motor,
            MotorDevice.FrontRight: self._front_right_motor,
        }[msg.param].wheel_velocity = msg.value

    def _handle_imu_msg(self, msg: ImuMsg) -> None:
        acc_coef = 2 * 2.0 / 65536.0
        gyro_coeff = 2 * 250.0 / 65536.0
        if msg.param == ImuData.AccelX:
            self._acc_sensor.x = msg.value * acc_coef
        elif msg.param == ImuData.AccelY:
            self._acc_sensor.y = msg.value * acc_coef
        elif msg.param == ImuData.AccelZ:
            self._acc_sensor.z = msg.value * acc_coef
        elif msg.param == ImuData.GyroX:
            self._gyro_sensor.x = msg.value * gyro_coeff
        elif msg.param == ImuData.GyroY:
            self._gyro_sensor.y = msg.value * gyro_coeff
        elif msg.param == ImuData.GyroZ:
            self._gyro_sensor.z = msg.value * gyro_coeff
        elif msg.param == ImuData.Temp:
            pass
        else:
            raise ValueError(f"Unknown IMU data: {msg.param}")

    def _calc_wait_time(self, last_time_ns: int) -> float:
        return max(0, (1.0 / 30.0) - last_time_ns / 1_000_000_000)

    def _on_drive_power_changed(self, device: MotorDevice, value: int) -> None:
        self._mpsc.put(MotorMsg(type=MessageType.Motor, param=device, value=value))

    def _sync_data(self) -> None:
        self._mpsc.put(MotorMsg(type=MessageType.Encoder, param=MotorDevice.RearLeft, value=0))
        self._mpsc.put(MotorMsg(type=MessageType.Encoder, param=MotorDevice.RearRight, value=0))
        self._mpsc.put(MotorMsg(type=MessageType.Encoder, param=MotorDevice.FrontLeft, value=0))
        self._mpsc.put(MotorMsg(type=MessageType.Encoder, param=MotorDevice.FrontRight, value=0))
        self._mpsc.put(MotorMsg(type=MessageType.IMU, param=ImuData.AccelX, value=0))
        self._mpsc.put(MotorMsg(type=MessageType.IMU, param=ImuData.AccelY, value=0))
        self._mpsc.put(MotorMsg(type=MessageType.IMU, param=ImuData.AccelZ, value=0))
        self._mpsc.put(MotorMsg(type=MessageType.IMU, param=ImuData.GyroX, value=0))
        self._mpsc.put(MotorMsg(type=MessageType.IMU, param=ImuData.GyroY, value=0))
        self._mpsc.put(MotorMsg(type=MessageType.IMU, param=ImuData.GyroZ, value=0))



if __name__ == "__main__":
    try:
        port = sys.argv[1]
    except IndexError:
        print("Usage: python app.py <serial-port>", file=sys.stderr)
        sys.exit(1)

    with Serial(port=port, baudrate=115200) as serial_port:
        app = MotorCtrlApp(serial_port=serial_port)
        try:
            app.run()
        except Exception:
            app.workers.cancel_all()
