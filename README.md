# kebni_driver

ROS 2 driver for the **Kebni SensAItion** IMU/INS sensor. Reads binary data over a serial port, decodes all sensor values defined in Table 53 of the SensAItion User Manual (D0000447), and publishes them as standard ROS 2 messages.

## Before Building

### 1. Identify the serial port

Connect the Kebni SensAItion device and find the assigned port:

```bash
dmesg | grep tty
```

This will show something like `/dev/ttyUSB0` or `/dev/ttyUSB1`. Note this port number.

### 2. Update the parameters file

Edit `parameters/parameters.yaml` and set `serial_port` to match the port you identified:

```yaml
serial_port: "/dev/ttyUSB{PORT_NUMBER}"
```

## Build

Navigate to the project folder:

```bash
cd kebni_driver
```

Build the Docker image:

```bash
docker build -t kebni-driver .
```

Run a Docker container with the project mounted as a volume:

```bash
docker run -it --rm -v "$(pwd)":/workspace kebni-driver
```

Inside the container, build the project:

```bash
colcon build
```

## Run

Navigate to the project folder:

```bash
cd kebni_driver
```

Run a Docker container with the project mounted:

- With the serial device attached:
  ```bash
  docker run -it --rm --device=/dev/ttyUSB{PORT_NUMBER} -v "$(pwd)":/workspace kebni-driver
  ```

- For development without a physical device:
  ```bash
  docker run -it --rm -v "$(pwd)":/workspace kebni-driver
  ```

Inside the container, source the build and launch the driver:

```bash
source install/setup.bash && ros2 launch kebni_driver kebni_driver.launch.py
```

## Testing

Navigate to the project folder:

```bash
cd kebni_driver
```

Run a Docker container with the project mounted:

```bash
docker run -it --rm -v "$(pwd)":/workspace kebni-driver
```

Inside the container, build with tests enabled and run them:

```bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select kebni_driver --event-handlers console_direct+
```

> **Hint:** If you prefer to build and run without Docker, you can install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) directly on your system and use the `colcon build` and `ros2 launch` commands above without the Docker steps.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB1` | Serial device path |
| `baud_rate` | `460800` | Serial baud rate |
| `configurationString` | _(required)_ | Kebni binary output config string (see below) |
| `frame_id_sensor` | `kebni_sensor` | Frame ID for sensor body measurements |
| `frame_id_ned` | `ned` | Frame ID for NED reference frame |
| `frame_id_ecef` | `ecef` | Frame ID for ECEF position |
| `frame_id_gps` | `gps` | Frame ID for GNSS fix |

## Configuration String Format

The configuration string defines which sensors are output and in what order. Format:

```
o<freq_div>s<payload_len><sensor_chunks><checksum_sig>
```

- `o` + 4 hex chars: frequency divisor (output Hz = 1000 / divisor)
- `s` + 2 hex chars: payload length in bytes
- Sensor chunks: groups of 3 hex chars each (`<sensor_id_2chars><byte_index_1char>`)
- Checksum signature: `x` for XOR8, `X` for CRC16

Example: `o0002s08003002001000033032031030x` configures 500 Hz output, 8-byte payload with accX and gyroX, XOR8 checksum.

## Published Topics

All publishers are **conditionally created** based on which sensors are present in the configuration string. Each publish call is double-guarded: the publisher must exist **and** the measurement data must be present in the current packet.

All message header timestamps use `system_time_ms` from the sensor when available, falling back to ROS time otherwise.

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `kebni_driver/imu` | `sensor_msgs/Imu` | Raw accelerometer, gyroscope, orientation quaternion. Orientation covariance from quality_roll/pitch/heading when available. |
| `kebni_driver/navsat_fix` | `sensor_msgs/NavSatFix` | Latitude, longitude, altitude. Position covariance from quality metrics. Status from gnss fix type. |
| `kebni_driver/velocity` | `geometry_msgs/VelocityStamped` | North/East/Down velocity. Frame: NED. |
| `kebni_driver/pose_ecef` | `geometry_msgs/PointStamped` | ECEF position (X/Y/Z). Frame: ecef. |
| `kebni_driver/rpy` | `geometry_msgs/Vector3Stamped` | Roll, pitch, heading in radians. Frame: NED. |
| `kebni_driver/magnetometer` | `sensor_msgs/MagneticField` | Magnetometer X/Y/Z in Tesla. |
| `kebni_driver/pressure` | `sensor_msgs/FluidPressure` | Barometric pressure in Pa. |
| `kebni_driver/temperature` | `sensor_msgs/Temperature` | IMU temperature in °C. |
| `kebni_driver/calibrated_temperature` | `sensor_msgs/Temperature` | Calibrated IMU temperature in °C. |
| `kebni_driver/inclinometer` | `geometry_msgs/Vector3Stamped` | Inclinometer X/Y/Z in m/s². |
| `kebni_driver/corrected_imu` | `sensor_msgs/Imu` | Corrected accelerometer and gyroscope. No orientation (covariance[0] = -1). |
| `kebni_driver/odometer` | `geometry_msgs/TwistStamped` | Odometer speed in m/s (linear.x). |
| `kebni_driver/gnss_fixed_relpos` | `geometry_msgs/PointStamped` | Fixed baseline relative position (N/E/D) in meters. Frame: NED. |
| `kebni_driver/gnss_moving_relpos` | `geometry_msgs/PointStamped` | Moving baseline relative position (N/E/D) in meters. Frame: NED. |
| `kebni_driver/gnss1_num_sat` | `std_msgs/Int32` | GNSS receiver 1 satellite count. |
| `kebni_driver/gnss2_num_sat` | `std_msgs/Int32` | GNSS receiver 2 satellite count. |
| `kebni_driver/error_flags` | `std_msgs/UInt32` | Raw 26-bit error bitmask (see ErrorFlags struct for bit definitions). |
| `kebni_driver/sensor_valid` | `std_msgs/UInt8` | Raw 8-bit sensor availability bitmask (see SensorValidFlags struct for bit definitions). |
| `kebni_driver/alignment_status` | `std_msgs/UInt8` | Alignment status enum index (0 = WaitingForGnssFix, 1 = NavigationRunning). |
| `kebni_driver/attitude_status` | `std_msgs/UInt32` | Raw attitude status bitfield. |
| `kebni_driver/utc_time` | `sensor_msgs/TimeReference` | UTC time as epoch seconds with sub-second precision. Source: `kebni_utc`. |
| `kebni_driver/rotation_matrix` | `std_msgs/Float64MultiArray` | 3x3 rotation matrix, row-major. Same orientation also available as quaternion in `imu` topic. |
| `kebni_driver/system_time_ms` | `std_msgs/Float64` | Sensor system time in milliseconds. Also used for all message header timestamps. |
| `kebni_driver/system_time_us` | `std_msgs/Float64` | Sensor system time in microseconds. |

## Supported Sensor Data by The Driver

| Category | Sensors | SI Unit |
|----------|---------|---------|
| Accelerometer (X/Y/Z) | 0x00-0x02 | m/s² |
| Gyroscope (X/Y/Z) | 0x03-0x05 | rad/s |
| Inclinometer (X/Y/Z) | 0x06-0x08 | m/s² |
| IMU Temperature | 0x09, 0x0F | °C |
| Magnetometer (X/Y/Z) | 0x0A-0x0C | Tesla |
| Barometer | 0x0D | Pa |
| Odometer Speed | 0x0E | m/s |
| GNSS Fixed Relative Position | 0x23-0x26 | m, s |
| GNSS Moving Relative Position | 0x27-0x2A | m, s |
| GNSS Satellite Counts | 0x2B | count |
| Error Flags / Sensor Valid | 0x2F-0x30 | bitfield |
| Horizontal Position (Lat/Lon) | 0x31-0x32 | degrees |
| Horizontal Velocity (N/E) | 0x33-0x34 | m/s |
| Vertical Position / Velocity | 0x35-0x36 | m, m/s |
| Roll / Pitch / Heading | 0x37-0x39 | rad |
| Corrected Accelerometer (X/Y/Z) | 0x3A-0x3C | m/s² |
| Corrected Gyroscope (X/Y/Z) | 0x3D-0x3F | rad/s |
| Alignment / Attitude Status | 0x40-0x41 | enum/bitfield |
| System Time | 0x42, 0x60 | ms, µs |
| GNSS iTow | 0x43-0x44 | s |
| UTC Time Fields | 0x45-0x46 | year/month/day/h/m/s |
| GNSS Fix Type | 0x47 | enum |
| Sync In Count / Time | 0x48-0x49 | count, s |
| UTC Sub-second / Status | 0x4A-0x4B | s, enum |
| Attitude Quaternion (w/x/y/z) | 0x4C-0x4F | unitless |
| Rotation Matrix (3x3) | 0x50-0x58 | unitless |
| ECEF Position (X/Y/Z) | 0x59-0x5B | m |
| Quality Metrics | 0x61-0x69 | m, m/s, rad |

## Measurements Struct

All decoded sensor values are stored in the `Measurements` struct. Each field is `std::optional` — only fields present in the configured output are populated.

### Standard Fields (`std::optional<double>`)

| Field | ID | SI Unit | Description |
|-------|----|---------|-------------|
| `accX`, `accY`, `accZ` | 0x00-0x02 | m/s² | Raw accelerometer |
| `gyroX`, `gyroY`, `gyroZ` | 0x03-0x05 | rad/s | Raw gyroscope |
| `inclX`, `inclY`, `inclZ` | 0x06-0x08 | m/s² | Inclinometer |
| `magX`, `magY`, `magZ` | 0x0A-0x0C | Tesla | Magnetometer |
| `barometer` | 0x0D | Pa | Barometric pressure |
| `odometer_speed` | 0x0E | m/s | Odometer speed |
| `gnss_fixed_relpos_itow` | 0x23 | s | GNSS fixed relative position iTOW |
| `gnss_fixed_relpos_north` | 0x24 | m | GNSS fixed relative position north |
| `gnss_fixed_relpos_east` | 0x25 | m | GNSS fixed relative position east |
| `gnss_fixed_relpos_down` | 0x26 | m | GNSS fixed relative position down |
| `gnss_moving_relpos_itow` | 0x27 | s | GNSS moving relative position iTOW |
| `gnss_moving_relpos_north` | 0x28 | m | GNSS moving relative position north |
| `gnss_moving_relpos_east` | 0x29 | m | GNSS moving relative position east |
| `gnss_moving_relpos_down` | 0x2A | m | GNSS moving relative position down |
| `horizontal_position_latitude` | 0x31 | degrees | Latitude |
| `horizontal_position_longitude` | 0x32 | degrees | Longitude |
| `horizontal_velocity_north` | 0x33 | m/s | Velocity north |
| `horizontal_velocity_east` | 0x34 | m/s | Velocity east |
| `vertical_position` | 0x35 | m | Vertical position |
| `vertical_velocity_down` | 0x36 | m/s | Vertical velocity down |
| `roll` | 0x37 | rad | Roll angle |
| `pitch` | 0x38 | rad | Pitch angle |
| `heading` | 0x39 | rad | Heading angle |
| `corrected_acc_x`, `_y`, `_z` | 0x3A-0x3C | m/s² | Corrected accelerometer |
| `corrected_gyro_x`, `_y`, `_z` | 0x3D-0x3F | rad/s | Corrected gyroscope |
| `system_time_ms` | 0x42 | ms | System time in milliseconds |
| `gnss1_itow`, `gnss2_itow` | 0x43-0x44 | s | GNSS iTOW |
| `sync_in_count` | 0x48 | count | Sync input counter |
| `sync_in_time` | 0x49 | s | Sync input time |
| `utc_sub_second` | 0x4A | s | UTC sub-second |
| `q_w`, `q_x`, `q_y`, `q_z` | 0x4C-0x4F | unitless | Attitude quaternion |
| `rotation_matrix_11` ... `_33` | 0x50-0x58 | unitless | 3x3 rotation matrix |
| `ecef_pos_x`, `_y`, `_z` | 0x59-0x5B | m | ECEF position |
| `system_time_us` | 0x60 | µs | System time in microseconds |
| `quality_horizontal_pos_lat` | 0x61 | m | Position quality (lat) |
| `quality_horizontal_pos_lon` | 0x62 | m | Position quality (lon) |
| `quality_horizontal_speed_north` | 0x63 | m/s | Speed quality (north) |
| `quality_horizontal_speed_east` | 0x64 | m/s | Speed quality (east) |
| `quality_vertical_position` | 0x65 | m | Vertical position quality |
| `quality_vertical_speed` | 0x66 | m/s | Vertical speed quality |
| `quality_roll`, `quality_pitch`, `quality_heading` | 0x67-0x69 | rad | Angle quality |

### Special-Case Fields (`std::optional<double>`)

| Field | ID | Unit | Description |
|-------|----|------|-------------|
| `imu_temperature` | 0x09 | °C | IMU temperature (non-linear conversion) |
| `calibrated_imu_temperature` | 0x0F | °C | Calibrated IMU temperature |
| `gnss1_num_sat`, `gnss2_num_sat` | 0x2B | count | Satellite counts (packed 2x16-bit) |
| `utc_year`, `utc_month` | 0x45 | — | UTC date (packed) |
| `utc_day`, `utc_hour`, `utc_min`, `utc_sec` | 0x46 | — | UTC time (packed 4x8-bit) |

### Typed Enum/Bitmask Fields

| Field | Type | ID | Description |
|-------|------|----|-------------|
| `error_flags` | `std::optional<ErrorFlags>` | 0x2F | 26 named bools for hardware/software errors |
| `sensor_valid` | `std::optional<SensorValidFlags>` | 0x30 | 8 named bools for sensor availability |
| `alignment_status` | `std::optional<AlignmentStatus>` | 0x40 | `WaitingForGnssFix` (0) or `NavigationRunning` (1) |
| `attitude_status` | `std::optional<uint32_t>` | 0x41 | Raw bitfield (undocumented bits) |
| `gnss1_fix_type`, `gnss2_fix_type` | `std::optional<GnssFixType>` | 0x47 | `NoFix` (0), `Fix2D` (2), `Fix3D` (3) |
| `utc_status` | `std::optional<UtcStatus>` | 0x4B | `NoFix` (0), `Available` (1), `Valid` (2) |