# BNO055 C++ Lifecycle Node Implementation

## Overview

This is a C++17 implementation of the BNO055 IMU driver using ROS2 Humble's Lifecycle Node (Managed Node) architecture. It provides high-performance sensor data acquisition with proper lifecycle management.

## Features

- **C++17 with rclcpp_lifecycle**: Modern C++ implementation following ROS2 Humble standards
- **Lifecycle Management**: Full support for lifecycle states (configure, activate, deactivate, cleanup, error)
- **Low-level I2C/UART Communication**: Direct system calls for non-blocking hardware communication
- **Performance**: Uses rclcpp::Timer instead of Python threading for better performance
- **Connection Monitoring**: Regular system_status and self_test checks with automatic reconnection
- **Standard IMU Messages**: Publishes sensor_msgs/Imu with Quaternion and Covariance matrices

## Building

### Prerequisites

- ROS2 Humble
- C++17 compatible compiler
- Linux I2C/Serial device access

### Build Instructions

```bash
cd ~/ros2_ws
colcon build --packages-select bno055
source install/setup.bash
```

## Running the Node

### As a Lifecycle Node (Recommended)

The lifecycle node allows controlled state transitions:

```bash
# Launch with automatic configuration and activation
ros2 launch bno055 bno055_lifecycle.launch.py

# Or manually control lifecycle transitions
ros2 run bno055 bno055_lifecycle_node --ros-args --params-file ./src/bno055/bno055/params/bno055_params.yaml

# Then in another terminal, manage the lifecycle:
ros2 lifecycle set /bno055 configure
ros2 lifecycle set /bno055 activate
```

### Lifecycle States

1. **Unconfigured**: Initial state
2. **Inactive**: Hardware connected and configured, but not publishing
3. **Active**: Fully operational, publishing sensor data
4. **Finalized**: Cleanly shut down

### State Transitions

- **configure**: Loads parameters and establishes hardware connection
- **activate**: Starts data publishing timers
- **deactivate**: Stops timers but maintains connection
- **cleanup**: Releases hardware resources

## Configuration

Parameters are defined in `bno055/params/bno055_params.yaml`:

### Connection Parameters

```yaml
connection_type: "uart"  # or "i2c"
uart_port: "/dev/ttyUSB0"
uart_baudrate: 115200
uart_timeout: 0.1
i2c_bus: 0
i2c_addr: 0x28
```

### Sensor Parameters

```yaml
frame_id: "bno055"
data_query_frequency: 10.0  # Hz
calib_status_frequency: 0.1  # Hz
operation_mode: 0x0C  # NDOF mode
placement_axis_remap: "P1"
```

## Published Topics

- `/bno055/imu` - Filtered IMU data with quaternion orientation
- `/bno055/imu_raw` - Raw accelerometer and gyroscope data
- `/bno055/mag` - Magnetometer data
- `/bno055/grav` - Gravity vector
- `/bno055/temp` - Temperature
- `/bno055/calib_status` - Calibration status as JSON

## Services

- `/bno055/calibration_request` - Request current calibration status

## Architecture

### Key Components

1. **BNO055LifecycleNode**: Main lifecycle node managing state transitions
2. **SensorService**: Handles sensor configuration and data acquisition
3. **I2CConnector/UARTConnector**: Low-level hardware communication
4. **Timers**:
   - Data timer: Regular sensor data reading
   - Calibration timer: Periodic calibration status logging
   - Watchdog timer: Connection health monitoring

### Lifecycle State Flow

```
Unconfigured -> on_configure() -> Inactive
    - Load parameters
    - Create hardware connector
    - Configure sensor

Inactive -> on_activate() -> Active
    - Activate publishers
    - Start timers (data, calibration, watchdog)

Active -> on_deactivate() -> Inactive
    - Stop timers
    - Deactivate publishers

Inactive -> on_cleanup() -> Unconfigured
    - Release sensor service
    - Disconnect hardware

Any -> on_error() -> Unconfigured
    - Safe cleanup on error
```

## Differences from Python Implementation

1. **Performance**: C++ implementation is significantly faster with lower CPU overhead
2. **Memory**: More efficient memory management with C++
3. **Lifecycle**: Explicit state management with lifecycle nodes
4. **Threading**: Uses ROS2 timers instead of Python threading
5. **Error Handling**: More robust error handling with lifecycle states

## Troubleshooting

### No data being published

Check lifecycle state:
```bash
ros2 lifecycle get /bno055
```

Ensure the node is in the "active" state.

### Connection errors

- Verify device permissions (I2C: `/dev/i2c-*`, UART: `/dev/ttyUSB*`)
- Check connection type matches hardware configuration
- Verify address/port parameters

### Build errors

Ensure all ROS2 dependencies are installed:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## License

BSD - See LICENSE file for details
