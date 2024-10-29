# motorctl
A simple imu calibration tool for motor board.

## setup
use poetry in host
```
$ poetry install
```

use poetry in container
```
$ docker compose run calibrate_imu_init
```


## launch
launch in host
```
$ poetry run python motorctl/main.py /dev/ttyACM0
```


launch in container
```
$ docker compose run calibrate_imu
```
