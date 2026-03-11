# A BNO05 ROS2 Package 

## Description
A ROS2 driver for the sensor IMU Bosch BNO055.

This repo was based off of [Michael Drwiega's work on the Bosch IMU Driver for ROS 1](https://github.com/mdrwiega/bosch_imu_driver)

---
## Wiring Guide

### Selecting Connection Type

The default mode is I2C.
To select UART mode connect the `3.3V` pin to the `PS1` pin.

### CP2104 USB-to-UART Bridge

When using a CP2104 USB-to-UART Bridge:

| BNO055 | CP2104 Friend    |
| ------ | ---------------- |
| Vin    |   5V             |
| GND    |   GND            |
| SDA    |   RXD            |
| SCL    |   TXD            |

**NOTE: on the CP2104 the pins above refer to the FTDI pins at the opposite end from the USB connector

---
## ROS Node Parameters

To configure with your own settings please adjust the [node parameter file](bno055/params/bno055_params.yaml) and pass it
as an argument when starting the node:

```
ros2 run bno055 bno055 --ros-args --params-file ./src/bno055/bno055/params/bno055_params.yaml
```

### UART Connection

- **connection_type=uart**: Defines UART as sensor connection type; default='uart'
- **uart_port**: The UART port to use; default='/dev/ttyUSB0'
- **uart_baudrate**: The baud rate to use; default=115200
- **uart_timeout**: The timeout for UART transmissions in seconds to use; default=0.1

### I2C Connection

- **connection_type=i2c**: Defines I2C as sensor connection type; default='uart'
- **i2c_bus**: The integer I2C bus number to use; default=0
- **i2c_address**: The hexadecimal I2C address to use; default=0x28
  

### Sensor Configuration

- **frame_id**: coordinate frame id of sensor default='bno055'
- **baudrate**: baudrate of sensor default=115200
- **data_query_frequency**: frequency (HZ) to read and publish data from sensor; default=100 Hz
- **calib_status_frequency**: frequency (HZ) to read and publish calibration status data from sensor; default=0.1 Hz
- **placement_axis_remap**: The sensor placement configuration (Axis remapping) defines the position and orientation of the sensor mount.
See Bosch BNO055 datasheet section "Axis Remap" for valid positions: "P0", "P1" (default), "P2", "P3", "P4", "P5", "P6", "P7".   

### ROS Topic Prefix

- **ros_topic_prefix**: ROS topic prefix to be used. Will be prepended to the default topic names (see below). Default="bno055/"

### Calibration

The current calibration values can be requested via the **calibration_request** service (this puts the imu into **CONFIGMODE** for a short time):

```
ros2 service call /bno055/calibration_request example_interfaces/srv/Trigger
```
---
## ROS Topics

ROS topics published by this ROS2 Node: 

  - **bno055/imu** [(sensor_msgs/Imu)](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
  - **bno055/imu_raw** [(sensor_msgs/Imu)](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
  - **bno055/temp** [(sensor_msgs/Temperature)](http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html); The sensor's ambient temperature
  - **bno055/mag** [(sensor_msgs/MagneticField)](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html)
  - **bno055/grav** [(geometry_msgs/Vector3)](http://docs.ros.org/en/api/geometry_msgs/html/msg/Vector3.html)
  - **bno055/calib_status** [(std_msgs/String)](http://docs.ros.org/en/api/std_msgs/html/msg/String.html) : 
  Sensor Calibration Status as JSON string - e.g. `{"sys": 3, "gyro": 3, "accel": 0, "mag": 3}`
  
  
While _bno055_ is the default ROS topic prefix, it can be configured by following the directions above.

---
  
## Development Workspace Setup

### On a Remote Device
Setup of a ROS2 workspace & IDE for a remote device (for example Raspberry Pi):

#### Clone & Build

Create a ROS2 [workspace](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/) on your remote device - for instance `~/ros2_ws`

Make sure you sourced your ROS2 installation (underlay).

Then clone the project into your workspace's src directory:

    cd ~/ros2_ws/src
    git clone https://github.com/flynneva/bno055.git
    
Perform a build of your workspace
    
    cd ~/ros2_ws
    colcon build

#### Integrate in your IDE

In order to work with the sources in your remote workspace and to integrate them in your IDE, use `sshfs`:

    sudo apt-get install sshfs
    sudo modprobe fuse

Create a IDE project directory and mount the remote ROS2 workspace:

    mkdir -p ~/projects/bno055/ros2_ws
    sshfs ubuntu@192.168.2.153:~/ros2_ws ~/projects/bno055/ros2_ws
    
Create a new project in your IDE from existing sources in `~/projects/bno055/ros2_ws`. 
You can now manipulate the remote ROS2 workspace using your local IDE (including git operations). 

### Running the ROS2 node
Run the `bno055` ROS2 node with default parameters:

    # source your local workspace (overlay) in addition to the ROS2 sourcing (underlay):
    source ~/ros2_ws/install/setup.sh
    # run the node:
    ros2 run bno055 bno055_lifecycle_node
    
Run with customized parameter file:

    ros2 run bno055 bno055_lifecycle_node --ros-args --params-file ./src/bno055/bno055/params/bno055_params.yaml
    
Run launch file (UART connection - default):

    ros2 launch bno055 bno055.launch.py

Run launch file (I2C connection):

    ros2 launch bno055 bno055_i2c.launch.py

Run launch file with manual lifecycle control:

    ros2 launch bno055 bno055_lifecycle.launch.py

The lifecycle launch files automatically configure and activate the node.
For manual lifecycle control, you can use:

    ros2 lifecycle set /bno055 configure
    ros2 lifecycle set /bno055 activate
    

### Performing flake8 Linting

To perform code linting with [flake8](https://gitlab.com/pycqa/flake8), just perform:

    cd ~/ros2_ws/src/bno055
    ament_flake8

See [www.flake8rules.com](https://www.flake8rules.com/) for more detailed information about flake8 rules.

**Note:** We take advantage of [flake8's noqa mechanisim](https://flake8.pycqa.org/en/3.1.1/user/ignoring-errors.html) to selectively ignore some errors. Just search for `# noqa:` in the source code to find them.

## C++ Implementation Bug Fixes

The following fixes were applied to the C++ implementation, referencing the [Adafruit BNO055 Arduino library](https://github.com/adafruit/Adafruit_BNO055) as the canonical implementation.

### 1. Blocking `read()` replaced with `timed_read()` using `select()` (`uart_connector.cpp`, `uart_connector.hpp`)

**Problem:** The low-level `::read()` call could block indefinitely when the BNO055 sensor or USB-serial adapter stopped responding, causing the entire ROS2 node to hang.

**Fix:** Added a new `timed_read()` method that wraps `::read()` with a `select()` call and an explicit timeout (default 150ms per byte). If no data arrives within the timeout, it returns -1 instead of blocking forever. Both `read_response()` and the write ACK path now use `timed_read()`.

### 2. Stale file descriptor detection on device removal (`uart_connector.cpp`)

**Problem:** When the USB cable was physically unplugged, `::read()` and `::write()` returned errors (EIO, ENXIO, EBADF, ENODEV) but the code did not check `errno`. The file descriptor remained open and `is_connected()` continued to return true, preventing reconnection.

**Fix:** After every `::read()` and `::write()` failure, `errno` is now checked for device-removal indicators (`EIO`, `ENXIO`, `EBADF`, `ENODEV`). When detected, `fd_` is immediately set to `-1`, forcing the node to enter the reconnection path.

### 3. CONFIG mode set before hardware reset (`sensor_service.cpp`)

**Problem:** The BNO055 datasheet and the Adafruit library both require the sensor to be in CONFIG mode before issuing a system reset via `SYS_TRIGGER`. If the sensor is in a fusion mode (e.g. NDOF), the reset command may be ignored.

**Fix:** `configure()` now explicitly sets `OPR_MODE = OPERATION_MODE_CONFIG` and waits 25ms **before** sending the reset command, matching the Adafruit `begin()` sequence.

### 4. Chip ID polling instead of fixed post-reset sleep (`sensor_service.cpp`)

**Problem:** After reset, the code used a fixed `1000ms` sleep. This was sometimes too long (wasting time) or too short (sensor not yet ready, causing subsequent commands to fail).

**Fix:** Replaced the fixed sleep with active chip ID polling, matching the Adafruit pattern:
- First poll: 30ms intervals, up to ~2 seconds
- Second poll: 100ms intervals, up to ~2 seconds  
- Final 50ms settle time after chip ID is confirmed (`BNO055_ID = 0xA0`)

### 5. Watchdog timer reduced from 5s to 1s (`bno055_node.cpp`)

**Problem:** The watchdog timer ran every 5 seconds, meaning a disconnection could go undetected for up to 5 seconds, causing stale data and delayed recovery.

**Fix:** Watchdog period changed from `5000ms` to `1000ms` for faster disconnection detection and quicker reconnection.

### 6. UART retry timing improvement (`uart_connector.cpp`)

**Problem:** On read failure, the retry loop waited only 2ms and flushed only input. If the UART stream was desynchronized (e.g. mid-packet), 2ms was too short for a 45-byte frame at 115200 baud (~4ms), and leftover output data could corrupt subsequent transactions.

**Fix:** Retry delay increased from `2ms` to `10ms`. Flush changed from `TCIFLUSH` (input only) to `TCIOFLUSH` (both input and output) to clear any stale data in both directions. Additionally, `fd_` validity is now checked at the start of each retry attempt.

### 7. Missing delays after axis remap and mode transitions (`sensor_service.cpp`)

**Problem:** Per the BNO055 datasheet, after writing axis remap registers and after any mode transition, the sensor needs time to process the command. These delays were missing, which could cause the next register write/read to fail intermittently.

**Fix:**
- Added **10ms** delay after `SYS_TRIGGER` clear (Adafruit uses 10ms)
- Added **10ms** delay after axis remap configuration write
- Added **20ms** delay after the final `setMode()` call (switching from CONFIG to operation mode)

### 8. Write ACK uses `timed_read()` instead of blocking `read()` (`uart_connector.cpp`)

**Problem:** After writing a command to the BNO055, the code waited for an ACK byte (`0xEE 0x01`) using blocking `::read()`. If the sensor was in a bad state or disconnected, this would also block indefinitely.

**Fix:** The write ACK path now uses `timed_read()` with a **300ms** timeout. If no ACK arrives within 300ms, the write is considered failed and the error is propagated, allowing the reconnection logic to kick in.

### 9. Persistent (infinite) retry at all levels (`bno055_node.cpp`, `uart_connector.cpp`, `sensor_service.cpp`)

**Problem:** Retry loops at every level had hard limits (5 in `on_configure`, 3 in `reset()`, 1 per watchdog cycle). When the BNO055 was temporarily unavailable (e.g. USB cable re-plugged, sensor booting slowly), these limits would be exhausted and the node would either crash or stop trying to reconnect.

**Fix:** All retry loops are now **infinite** — the node keeps trying until it successfully communicates with the sensor. Clean shutdown is guaranteed via `rclcpp::ok()` checks:
- **`on_configure()`**: Retries indefinitely with increasing delays (1s → 5s cap). The robot cannot operate without IMU, so the node waits.
- **`UARTConnector::reset()`**: Retries indefinitely with increasing delays (200ms → 3s cap). Logs every 10 attempts to stderr.
- **`check_watchdog()`**: When serial timeout is detected, loops continuously (reset + reconfigure) until data flows again. No more cooldown-based single attempts.
