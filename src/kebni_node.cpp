// ROS 2 node: reads Kebni serial data and publishes sensor messages.

#include "kebni_driver/kebni_node.hpp"

#include <boost/asio.hpp>
#include <ctime>

using namespace kebni;

KebniNode::KebniNode() :
    Node("kebni_driver_node") {
    RCLCPP_INFO(get_logger(), "Kebni driver node starting");

    declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
    declare_parameter<std::string>("configurationString", "");
    declare_parameter<bool>("big_endian", false);
    declare_parameter<int>("baud_rate", 460800);
    declare_parameter<std::string>("frame_id_sensor", "kebni_sensor");
    declare_parameter<std::string>("frame_id_ned", "ned");
    declare_parameter<std::string>("frame_id_ecef", "ecef");
    declare_parameter<std::string>("frame_id_gps", "gps");

    const auto serialPort = get_parameter("serial_port").as_string();
    const auto config = get_parameter("configurationString").as_string();
    const int baudRate = get_parameter("baud_rate").as_int();
    frame_id_sensor_ = get_parameter("frame_id_sensor").as_string();
    frame_id_ned_ = get_parameter("frame_id_ned").as_string();
    frame_id_ecef_ = get_parameter("frame_id_ecef").as_string();
    frame_id_gps_ = get_parameter("frame_id_gps").as_string();

    if (config.empty()) {
        RCLCPP_FATAL(get_logger(), "Config parameter is empty");
        throw std::runtime_error("Missing config parameter");
    }

    configuration = configureKebniDriver(config);

    if (configuration.configError != ConfigError::None) {
        RCLCPP_FATAL(get_logger(), "Kebni configuration error: %s", toString(configuration.configError));
    }

    configuration.print();
    createPublishers();
    startSerial(serialPort, baudRate);
}

void KebniNode::createPublishers() {
    auto qos = rclcpp::QoS(10).best_effort().durability_volatile();

    if (configuration.hasSensor(SensorId::ecef_pos_x)) {
        pose_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("kebni_driver/pose_ecef", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/pose_ecef");
    }
    if (configuration.hasSensor(SensorId::accX) || configuration.hasSensor(SensorId::q_w)) {
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("kebni_driver/imu", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/imu");
    }
    if (configuration.hasSensor(SensorId::horizontal_velocity_north)) {
        vel_pub_ = create_publisher<geometry_msgs::msg::VelocityStamped>("kebni_driver/velocity", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/velocity");
    }
    if (configuration.hasSensor(SensorId::horizontal_position_latitude)) {
        navsat_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("kebni_driver/navsat_fix", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/navsat_fix");
    }
    if (configuration.hasSensor(SensorId::roll)) {
        rpy_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("kebni_driver/rpy", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/rpy");
    }
    if (configuration.hasSensor(SensorId::magX)) {
        mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("kebni_driver/magnetometer", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/magnetometer");
    }
    if (configuration.hasSensor(SensorId::barometer)) {
        pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>("kebni_driver/pressure", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/pressure");
    }
    if (configuration.hasSensor(SensorId::imu_temperature)) {
        temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("kebni_driver/temperature", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/temperature");
    }
    if (configuration.hasSensor(SensorId::calibrated_imu_temperature)) {
        calibrated_temp_pub_ =
            create_publisher<sensor_msgs::msg::Temperature>("kebni_driver/calibrated_temperature", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/calibrated_temperature");
    }
    if (configuration.hasSensor(SensorId::inclX)) {
        incl_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("kebni_driver/inclinometer", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/inclinometer");
    }
    if (configuration.hasSensor(SensorId::corrected_acc_x) || configuration.hasSensor(SensorId::corrected_gyro_x)) {
        corrected_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("kebni_driver/corrected_imu", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/corrected_imu");
    }
    if (configuration.hasSensor(SensorId::odometer_speed)) {
        odom_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("kebni_driver/odometer", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/odometer");
    }
    if (configuration.hasSensor(SensorId::gnss_fixed_relpos_north)) {
        gnss_fixed_relpos_pub_ =
            create_publisher<geometry_msgs::msg::PointStamped>("kebni_driver/gnss_fixed_relpos", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/gnss_fixed_relpos");
    }
    if (configuration.hasSensor(SensorId::gnss_moving_relpos_north)) {
        gnss_moving_relpos_pub_ =
            create_publisher<geometry_msgs::msg::PointStamped>("kebni_driver/gnss_moving_relpos", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/gnss_moving_relpos");
    }
    if (configuration.hasSensor(SensorId::error_flags)) {
        error_flags_pub_ = create_publisher<std_msgs::msg::UInt32>("kebni_driver/error_flags", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/error_flags");
    }
    if (configuration.hasSensor(SensorId::sensor_valid)) {
        sensor_valid_pub_ = create_publisher<std_msgs::msg::UInt8>("kebni_driver/sensor_valid", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/sensor_valid");
    }
    if (configuration.hasSensor(SensorId::alignment_status)) {
        alignment_status_pub_ = create_publisher<std_msgs::msg::UInt8>("kebni_driver/alignment_status", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/alignment_status");
    }
    if (configuration.hasSensor(SensorId::attitude_status)) {
        attitude_status_pub_ = create_publisher<std_msgs::msg::UInt32>("kebni_driver/attitude_status", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/attitude_status");
    }
    if (configuration.hasSensor(SensorId::gnss_num_sat)) {
        gnss1_num_sat_pub_ = create_publisher<std_msgs::msg::Int32>("kebni_driver/gnss1_num_sat", qos);
        gnss2_num_sat_pub_ = create_publisher<std_msgs::msg::Int32>("kebni_driver/gnss2_num_sat", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/gnss1_num_sat, kebni_driver/gnss2_num_sat");
    }
    if (configuration.hasSensor(SensorId::utc_year_month) || configuration.hasSensor(SensorId::utc_day_time)) {
        utc_time_pub_ = create_publisher<sensor_msgs::msg::TimeReference>("kebni_driver/utc_time", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/utc_time");
    }
    if (configuration.hasSensor(SensorId::rotation_matrix_11)) {
        rotation_matrix_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("kebni_driver/rotation_matrix", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/rotation_matrix");
    }
    if (configuration.hasSensor(SensorId::system_time_ms)) {
        system_time_ms_pub_ = create_publisher<std_msgs::msg::Float64>("kebni_driver/system_time_ms", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/system_time_ms");
    }
    if (configuration.hasSensor(SensorId::system_time_us)) {
        system_time_us_pub_ = create_publisher<std_msgs::msg::Float64>("kebni_driver/system_time_us", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/system_time_us");
    }

    if (configuration.hasSensor(SensorId::system_time_ms)) {
        RCLCPP_INFO(get_logger(), "Using sensor system_time_ms for message timestamps");
    } else {
        RCLCPP_WARN(get_logger(),
                    "system_time_ms not configured, using ROS time for message timestamps. "
                    "Recommendation: add sensor 0x42 (system_time_ms) to your configuration string "
                    "for accurate timestamps that are consistent across all measurements in a packet");
    }
}

// Opens the serial port, retrying until success or shutdown.
void KebniNode::startSerial(const std::string &port, int baudRate) {
    serial_port_ = port;
    baud_rate_ = baudRate;

    while (rclcpp::ok()) {
        try {
            RCLCPP_INFO(get_logger(), "Trying to open serial port: %s", port.c_str());

            io_ = std::make_unique<boost::asio::io_context>();
            serial_ = std::make_unique<boost::asio::serial_port>(*io_);

            serial_->open(port);
            serial_->set_option(boost::asio::serial_port_base::baud_rate(baudRate));

            RCLCPP_INFO(get_logger(), "Serial port %s opened successfully", port.c_str());

            serial_thread_ = std::thread(&KebniNode::serialThread, this);
            return;

        } catch (const std::exception &e) {
            RCLCPP_WARN(get_logger(), "Serial port not available (%s). Retrying in 1s...", e.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

// Reads bytes from the serial port, reconnects on failure.
void KebniNode::serialThread() {
    uint8_t buf[256];

    while (running_) {
        try {
            size_t n = serial_->read_some(boost::asio::buffer(buf));

            for (size_t i = 0; i < n; ++i) {
                StreamError streamError = processByte(buf[i]);
                if (streamError != StreamError::None) {
                    RCLCPP_WARN(get_logger(), "Stream error: %s", toString(streamError));
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Serial error: %s. Attempting reconnection...", e.what());

            if (serial_ && serial_->is_open()) {
                serial_->close();
            }

            while (running_ && rclcpp::ok()) {
                try {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    io_ = std::make_unique<boost::asio::io_context>();
                    serial_ = std::make_unique<boost::asio::serial_port>(*io_);
                    serial_->open(serial_port_);
                    serial_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
                    RCLCPP_INFO(get_logger(), "Serial port %s reconnected successfully", serial_port_.c_str());
                    break;
                } catch (const std::exception &re) {
                    RCLCPP_WARN(get_logger(), "Reconnection failed (%s). Retrying in 1s...", re.what());
                }
            }
        }
    }

    RCLCPP_INFO(get_logger(), "Serial thread stopped");
}

// Called by the driver when a complete packet is decoded.
// Only publishes a message if the publisher exists and all required fields are present.
void KebniNode::onMeasurements(const kebni::Measurements &m) {

    if (m.invalidChecksum) {
        RCLCPP_WARN(get_logger(), "Invalid Checksum");
        return;
    }

    // Use sensor system_time_ms for header stamps, fall back to ROS time
    builtin_interfaces::msg::Time current_time;
    if (m.system_time_ms.has_value()) {
        double t = m.system_time_ms.value() / 1000.0; // ms to seconds
        current_time.sec = static_cast<int32_t>(t);
        current_time.nanosec = static_cast<uint32_t>((t - current_time.sec) * 1e9);
    } else {
        current_time = now();
    }

    if (pose_pub_ && m.ecef_pos_x.has_value() && m.ecef_pos_y.has_value() && m.ecef_pos_z.has_value()) {
        publishEcefPose(m, current_time);
    }
    if (imu_pub_ && m.q_w.has_value() && m.q_x.has_value() && m.q_y.has_value() && m.q_z.has_value() &&
        m.accX.has_value() && m.accY.has_value() && m.accZ.has_value() && m.gyroX.has_value() && m.gyroY.has_value() &&
        m.gyroZ.has_value()) {
        publishImu(m, current_time);
    }
    if (vel_pub_ && m.horizontal_velocity_north.has_value() && m.horizontal_velocity_east.has_value()) {
        publishVelocity(m, current_time);
    }
    if (navsat_pub_ && m.horizontal_position_latitude.has_value() && m.horizontal_position_longitude.has_value()) {
        publishNavSatFix(m, current_time);
    }
    if (rpy_pub_ && m.roll.has_value() && m.pitch.has_value() && m.heading.has_value()) {
        publishRpy(m, current_time);
    }
    if (mag_pub_ && m.magX.has_value() && m.magY.has_value() && m.magZ.has_value()) {
        publishMagnetometer(m, current_time);
    }
    if (pressure_pub_ && m.barometer.has_value()) {
        publishPressure(m, current_time);
    }
    if (temp_pub_ && m.imu_temperature.has_value()) {
        publishTemperature(m, current_time);
    }
    if (calibrated_temp_pub_ && m.calibrated_imu_temperature.has_value()) {
        publishCalibratedTemperature(m, current_time);
    }
    if (incl_pub_ && m.inclX.has_value() && m.inclY.has_value() && m.inclZ.has_value()) {
        publishInclinometer(m, current_time);
    }
    if (corrected_imu_pub_ && m.corrected_acc_x.has_value() && m.corrected_acc_y.has_value() &&
        m.corrected_acc_z.has_value() && m.corrected_gyro_x.has_value() && m.corrected_gyro_y.has_value() &&
        m.corrected_gyro_z.has_value()) {
        publishCorrectedImu(m, current_time);
    }
    if (odom_pub_ && m.odometer_speed.has_value()) {
        publishOdometer(m, current_time);
    }
    if (gnss_fixed_relpos_pub_ && m.gnss_fixed_relpos_north.has_value() && m.gnss_fixed_relpos_east.has_value() &&
        m.gnss_fixed_relpos_down.has_value()) {
        publishGnssFixedRelpos(m, current_time);
    }
    if (gnss_moving_relpos_pub_ && m.gnss_moving_relpos_north.has_value() && m.gnss_moving_relpos_east.has_value() &&
        m.gnss_moving_relpos_down.has_value()) {
        publishGnssMovingRelpos(m, current_time);
    }
    if (error_flags_pub_ && m.error_flags_raw.has_value()) {
        publishErrorFlags(m);
    }
    if (sensor_valid_pub_ && m.sensor_valid_raw.has_value()) {
        publishSensorValid(m);
    }
    if (alignment_status_pub_ && m.alignment_status.has_value()) {
        publishAlignmentStatus(m);
    }
    if (attitude_status_pub_ && m.attitude_status.has_value()) {
        publishAttitudeStatus(m);
    }
    if (gnss1_num_sat_pub_ || gnss2_num_sat_pub_) {
        publishGnssSatCounts(m);
    }
    if (utc_time_pub_ && m.utc_year.has_value() && m.utc_month.has_value() && m.utc_day.has_value() &&
        m.utc_hour.has_value() && m.utc_min.has_value() && m.utc_sec.has_value()) {
        publishUtcTime(m, current_time);
    }
    if (rotation_matrix_pub_ && m.rotation_matrix_11.has_value() && m.rotation_matrix_12.has_value() &&
        m.rotation_matrix_13.has_value() && m.rotation_matrix_21.has_value() && m.rotation_matrix_22.has_value() &&
        m.rotation_matrix_23.has_value() && m.rotation_matrix_31.has_value() && m.rotation_matrix_32.has_value() &&
        m.rotation_matrix_33.has_value()) {
        publishRotationMatrix(m);
    }
    if (system_time_ms_pub_ || system_time_us_pub_) {
        publishSystemTime(m);
    }
}

// ECEF position in meters (X/Y/Z)
void KebniNode::publishEcefPose(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_ecef_;
    msg.point.x = m.ecef_pos_x.value();
    msg.point.y = m.ecef_pos_y.value();
    msg.point.z = m.ecef_pos_z.value();
    pose_pub_->publish(msg);
}

// Raw accelerometer, gyroscope, and orientation quaternion (NED)
// Covariance diagonal from quality metrics (variance = sigma^2), -1 if unknown
void KebniNode::publishImu(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;

    msg.linear_acceleration.x = m.accX.value();
    msg.linear_acceleration.y = m.accY.value();
    msg.linear_acceleration.z = m.accZ.value();

    msg.angular_velocity.x = m.gyroX.value();
    msg.angular_velocity.y = m.gyroY.value();
    msg.angular_velocity.z = m.gyroZ.value();

    msg.orientation.w = m.q_w.value();
    msg.orientation.x = m.q_x.value();
    msg.orientation.y = m.q_y.value();
    msg.orientation.z = m.q_z.value();

    if (m.quality_roll.has_value() && m.quality_pitch.has_value() && m.quality_heading.has_value()) {
        double qr = m.quality_roll.value();
        double qp = m.quality_pitch.value();
        double qh = m.quality_heading.value();
        msg.orientation_covariance = {qr * qr, 0, 0, 0, qp * qp, 0, 0, 0, qh * qh};
    } else {
        msg.orientation_covariance[0] = -1.0;
    }

    imu_pub_->publish(msg);
}

// North/East/Down velocity in m/s (NED frame)
void KebniNode::publishVelocity(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    geometry_msgs::msg::VelocityStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_ned_;
    msg.body_frame_id = frame_id_sensor_;
    msg.reference_frame_id = frame_id_ned_;

    msg.velocity.linear.x = m.horizontal_velocity_north.value();
    msg.velocity.linear.y = m.horizontal_velocity_east.value();

    if (m.vertical_velocity_down.has_value()) {
        msg.velocity.linear.z = m.vertical_velocity_down.value();
    }

    vel_pub_->publish(msg);
}

// Lat/lon/alt with fix status from both GNSS receivers
// Position covariance diagonal from quality metrics, or UNKNOWN if not available
void KebniNode::publishNavSatFix(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_gps_;

    msg.latitude = m.horizontal_position_latitude.value();
    msg.longitude = m.horizontal_position_longitude.value();

    if (m.vertical_position.has_value()) {
        msg.altitude = m.vertical_position.value();
    }

    // Fix if either GNSS receiver reports 2D or 3D fix
    bool has_fix = (m.gnss1_fix_type.has_value() && isGnssFixed(m.gnss1_fix_type.value())) ||
                   (m.gnss2_fix_type.has_value() && isGnssFixed(m.gnss2_fix_type.value()));
    msg.status.status =
        has_fix ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    if (m.quality_horizontal_pos_lat.has_value() && m.quality_horizontal_pos_lon.has_value()) {
        double qlat = m.quality_horizontal_pos_lat.value();
        double qlon = m.quality_horizontal_pos_lon.value();
        double qalt = m.quality_vertical_position.value_or(0.0);
        msg.position_covariance = {qlat * qlat, 0, 0, 0, qlon * qlon, 0, 0, 0, qalt * qalt};
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    } else {
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }

    navsat_pub_->publish(msg);
}

// Roll, pitch, heading in radians (NED frame)
void KebniNode::publishRpy(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_ned_;
    msg.vector.x = m.roll.value();
    msg.vector.y = m.pitch.value();
    msg.vector.z = m.heading.value();
    rpy_pub_->publish(msg);
}

// Magnetometer X/Y/Z in Tesla
void KebniNode::publishMagnetometer(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    sensor_msgs::msg::MagneticField msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;
    msg.magnetic_field.x = m.magX.value();
    msg.magnetic_field.y = m.magY.value();
    msg.magnetic_field.z = m.magZ.value();
    mag_pub_->publish(msg);
}

// Barometric pressure in Pascal
void KebniNode::publishPressure(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    sensor_msgs::msg::FluidPressure msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;
    msg.fluid_pressure = m.barometer.value();
    pressure_pub_->publish(msg);
}

// Raw IMU temperature in degrees Celsius (non-linear conversion from sensor)
void KebniNode::publishTemperature(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    sensor_msgs::msg::Temperature msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;
    msg.temperature = m.imu_temperature.value();
    temp_pub_->publish(msg);
}

// Factory-calibrated IMU temperature in degrees Celsius
void KebniNode::publishCalibratedTemperature(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    sensor_msgs::msg::Temperature msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;
    msg.temperature = m.calibrated_imu_temperature.value();
    calibrated_temp_pub_->publish(msg);
}

// Inclinometer X/Y/Z in m/s^2
void KebniNode::publishInclinometer(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;
    msg.vector.x = m.inclX.value();
    msg.vector.y = m.inclY.value();
    msg.vector.z = m.inclZ.value();
    incl_pub_->publish(msg);
}

// Sensor-fusion corrected accelerometer and gyroscope
// No orientation available, covariance[0] = -1 marks it as unknown
void KebniNode::publishCorrectedImu(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;

    msg.linear_acceleration.x = m.corrected_acc_x.value();
    msg.linear_acceleration.y = m.corrected_acc_y.value();
    msg.linear_acceleration.z = m.corrected_acc_z.value();

    msg.angular_velocity.x = m.corrected_gyro_x.value();
    msg.angular_velocity.y = m.corrected_gyro_y.value();
    msg.angular_velocity.z = m.corrected_gyro_z.value();

    msg.orientation_covariance[0] = -1.0;

    corrected_imu_pub_->publish(msg);
}

// Odometer speed in m/s (linear.x only)
void KebniNode::publishOdometer(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;
    msg.twist.linear.x = m.odometer_speed.value();
    odom_pub_->publish(msg);
}

// Fixed baseline GNSS relative position N/E/D in meters
void KebniNode::publishGnssFixedRelpos(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_ned_;
    msg.point.x = m.gnss_fixed_relpos_north.value();
    msg.point.y = m.gnss_fixed_relpos_east.value();
    msg.point.z = m.gnss_fixed_relpos_down.value();
    gnss_fixed_relpos_pub_->publish(msg);
}

// Moving baseline GNSS relative position N/E/D in meters
void KebniNode::publishGnssMovingRelpos(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_ned_;
    msg.point.x = m.gnss_moving_relpos_north.value();
    msg.point.y = m.gnss_moving_relpos_east.value();
    msg.point.z = m.gnss_moving_relpos_down.value();
    gnss_moving_relpos_pub_->publish(msg);
}

// 26-bit error bitmask (see ErrorFlags struct for bit definitions)
void KebniNode::publishErrorFlags(const Measurements &m) {
    std_msgs::msg::UInt32 msg;
    msg.data = m.error_flags_raw.value();
    error_flags_pub_->publish(msg);
}

// 8-bit sensor availability bitmask (see SensorValidFlags struct for bit definitions)
void KebniNode::publishSensorValid(const Measurements &m) {
    std_msgs::msg::UInt8 msg;
    msg.data = m.sensor_valid_raw.value();
    sensor_valid_pub_->publish(msg);
}

// 0 = WaitingForGnssFix, 1 = NavigationRunning
void KebniNode::publishAlignmentStatus(const Measurements &m) {
    std_msgs::msg::UInt8 msg;
    msg.data = static_cast<uint8_t>(m.alignment_status.value());
    alignment_status_pub_->publish(msg);
}

// Raw attitude status bitfield
void KebniNode::publishAttitudeStatus(const Measurements &m) {
    std_msgs::msg::UInt32 msg;
    msg.data = m.attitude_status.value();
    attitude_status_pub_->publish(msg);
}

// Satellite count for each GNSS receiver (packed from sensor 0x2B)
void KebniNode::publishGnssSatCounts(const Measurements &m) {
    if (gnss1_num_sat_pub_ && m.gnss1_num_sat.has_value()) {
        std_msgs::msg::Int32 msg;
        msg.data = static_cast<int32_t>(m.gnss1_num_sat.value());
        gnss1_num_sat_pub_->publish(msg);
    }
    if (gnss2_num_sat_pub_ && m.gnss2_num_sat.has_value()) {
        std_msgs::msg::Int32 msg;
        msg.data = static_cast<int32_t>(m.gnss2_num_sat.value());
        gnss2_num_sat_pub_->publish(msg);
    }
}

// UTC calendar fields converted to Unix epoch seconds via timegm()
// Sub-second precision from utc_sub_second if available
void KebniNode::publishUtcTime(const Measurements &m, const builtin_interfaces::msg::Time &t) {
    int year = static_cast<int>(m.utc_year.value());
    int month = static_cast<int>(m.utc_month.value());
    int day = static_cast<int>(m.utc_day.value());
    int hour = static_cast<int>(m.utc_hour.value());
    int min = static_cast<int>(m.utc_min.value());
    int sec = static_cast<int>(m.utc_sec.value());

    std::tm tm_utc{};
    tm_utc.tm_year = year - 1900;
    tm_utc.tm_mon = month - 1;
    tm_utc.tm_mday = day;
    tm_utc.tm_hour = hour;
    tm_utc.tm_min = min;
    tm_utc.tm_sec = sec;

    std::time_t epoch = timegm(&tm_utc);

    RCLCPP_DEBUG(get_logger(),
                 "UTC: %d-%02d-%02d %02d:%02d:%02d -> epoch=%ld",
                 year,
                 month,
                 day,
                 hour,
                 min,
                 sec,
                 static_cast<long>(epoch));

    sensor_msgs::msg::TimeReference msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;
    msg.source = "kebni_utc";
    msg.time_ref.sec = static_cast<int32_t>(epoch);
    msg.time_ref.nanosec = m.utc_sub_second.has_value() ? static_cast<uint32_t>(m.utc_sub_second.value() * 1e9) : 0u;

    utc_time_pub_->publish(msg);
}

// 3x3 rotation matrix in row-major order (stride 3 per row, 1 per column)
void KebniNode::publishRotationMatrix(const Measurements &m) {
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "rows";
    msg.layout.dim[0].size = 3;
    msg.layout.dim[0].stride = 3;
    msg.layout.dim[1].label = "cols";
    msg.layout.dim[1].size = 3;
    msg.layout.dim[1].stride = 1;
    msg.data = {m.rotation_matrix_11.value(),
                m.rotation_matrix_12.value(),
                m.rotation_matrix_13.value(),
                m.rotation_matrix_21.value(),
                m.rotation_matrix_22.value(),
                m.rotation_matrix_23.value(),
                m.rotation_matrix_31.value(),
                m.rotation_matrix_32.value(),
                m.rotation_matrix_33.value()};
    rotation_matrix_pub_->publish(msg);
}

// Raw sensor timestamps (ms and us published separately)
void KebniNode::publishSystemTime(const Measurements &m) {
    if (system_time_ms_pub_ && m.system_time_ms.has_value()) {
        std_msgs::msg::Float64 msg;
        msg.data = m.system_time_ms.value();
        system_time_ms_pub_->publish(msg);
    }
    if (system_time_us_pub_ && m.system_time_us.has_value()) {
        std_msgs::msg::Float64 msg;
        msg.data = m.system_time_us.value();
        system_time_us_pub_->publish(msg);
    }
}

KebniNode::~KebniNode() {
    running_ = false;

    if (serial_ && serial_->is_open()) {
        serial_->cancel();
        serial_->close();
    }

    if (serial_thread_.joinable()) {
        serial_thread_.join();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<KebniNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
