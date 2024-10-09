# motorctl
A simple tool for controling motor board.

## setup
use poetry in host
```
$ poetry install
```

use poetry in container
```
$ docker compose run motorctl_init
```


## launch
launch in host
```
$ poetry run textual run motorctl/app.py /dev/ttyACM0
```


launch in container
```
$ docker compose run motorctl
```


![](https://github.com/ar90n/rover0/blob/assets/sc_motorctl_tool.png?raw=true)

