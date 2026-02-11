# ads1115_adc

ROS 2 Jazzy driver for the TI ADS1115 16-bit 4-channel ADC over I2C.

## Features

- Publishes `std_msgs/Float32` on `adc/channel0` ~ `adc/channel3` (configurable)
- **Fake mode** — generates random voltage data without physical hardware
- Calibrate and reset services for offset bias correction
- Runtime `publish_rate` change via `ros2 param set`
- Configurable PGA gain (±0.256 V to ±6.144 V) and data rate (8 to 860 SPS)
- Selectable active channels via parameter

## Prerequisites

- ROS 2 Jazzy
- Python 3
- `smbus2` (only required when `fake_mode` is `false`)

```bash
pip3 install smbus2
```

## Installation

```bash
cd ~/ros2_ws
colcon build --packages-select ads1115_adc
source install/setup.bash
```

## Usage

### Launch with default parameters (fake mode)

```bash
ros2 launch ads1115_adc ads1115_launch.py
```

### Run with real hardware

```bash
ros2 run ads1115_adc ads1115_node.py --ros-args -p fake_mode:=false
```

### Verify output

```bash
ros2 topic echo /adc/channel0
ros2 topic echo /adc/channel1
```

### Services

```bash
ros2 service call /adc/calibrate std_srvs/srv/Trigger
ros2 service call /adc/reset std_srvs/srv/Trigger
```

### Change publish rate at runtime

```bash
ros2 param set /ads1115_adc_node publish_rate 20.0
```

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `fake_mode` | bool | `true` | `true`: generate random data, `false`: read from real I2C device |
| `i2c_bus` | int | `1` | I2C bus number (`/dev/i2c-N`) |
| `device_address` | int | `0x48` | I2C address: `0x48` (GND), `0x49` (VDD), `0x4A` (SDA), `0x4B` (SCL) |
| `publish_rate` | double | `10.0` | Publishing rate in Hz (runtime changeable) |
| `active_channels` | int[] | `[0, 1, 2, 3]` | List of single-ended channels to read (0-3) |
| `pga_gain` | int | `2` | PGA gain: `0`=±6.144V, `1`=±4.096V, `2`=±2.048V, `3`=±1.024V, `4`=±0.512V, `5`=±0.256V |
| `data_rate` | int | `4` | Data rate: `0`=8, `1`=16, `2`=32, `3`=64, `4`=128, `5`=250, `6`=475, `7`=860 SPS |

## Package Structure

```
ads1115_adc/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── ads1115_params.yaml
├── launch/
│   └── ads1115_launch.py
├── ads1115_adc/
│   ├── __init__.py
│   └── ads1115_driver.py
├── nodes/
│   └── ads1115_node.py
└── test/
    └── test_ads1115_node.py
```

## Test Results

Tested on Ubuntu 24.04 (WSL2) with `fake_mode: true`.

```
$ colcon test --packages-select ads1115_adc
$ colcon test-result --verbose
Summary: 27 tests, 0 errors, 0 failures, 0 skipped
```

| Test Category | Test | Result |
|---|---|---|
| **Topics** | `adc/channel0`~`channel3` publish `std_msgs/Float32` | PASS |
| **Topics** | Voltage in range -10.0 ~ +10.0 V | PASS |
| **Services** | `adc/calibrate` returns `success=True` | PASS |
| **Services** | `adc/reset` returns `success=True` | PASS |
| **Parameters** | `publish_rate` runtime change to 20.0 Hz | PASS |
| **Shutdown** | Clean exit (code 0, -2, or -15) | PASS |
| **Linting** | pep257, flake8, copyright, xmllint | PASS |

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
