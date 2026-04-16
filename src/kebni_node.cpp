// ROS 2 node: reads Kebni serial data and publishes sensor messages.

#include "kebni_driver/kebni_node.hpp"

#include <ctime>
#include <iostream>
#include <string>

using valueId = sepa::SensorDataValueId;

KebniNode::KebniNode()
    : Node("kebni_driver_node") {

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

    std::string configurationErrorMessage;

    try {
        configuration_ = backend_.parseDataUartConfigString(config, parseInfo_);
    }
    catch (const std::exception &e) {
        configurationErrorMessage = e.what();
    }

    printConfiguration(configurationErrorMessage);

    if (!configurationErrorMessage.empty()) {
        RCLCPP_FATAL(get_logger(), "Kebni configuration error: %s", configurationErrorMessage.c_str());
    }

    createPublishers();

    packetAssembler_.setListener(this);
    packetAssembler_.resetBuffer(parseInfo_);
    startSerial(serialPort, baudRate);
}

void KebniNode::createPublishers() {
    auto qos = rclcpp::QoS(10).best_effort().durability_volatile();

    if (configuration_.test(valueId::ECEF_POS_X)) {
        pose_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("kebni_driver/pose_ecef", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/pose_ecef");
    }
    if (configuration_.test(valueId::ACC_X) ||
        configuration_.test(valueId::Q_W))
    {
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("kebni_driver/imu", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/imu");
    }
    if (configuration_.test(valueId::VEL_NORTH)) {
        vel_pub_ = create_publisher<geometry_msgs::msg::VelocityStamped>("kebni_driver/velocity", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/velocity");
    }
    if (configuration_.test(valueId::POS_LATITUDE)) {
        navsat_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("kebni_driver/navsat_fix", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/navsat_fix");
    }
    if (configuration_.test(valueId::ROLL)) {
        rpy_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("kebni_driver/rpy", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/rpy");
    }
    if (configuration_.test(valueId::MAG_X)) {
        mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("kebni_driver/magnetometer", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/magnetometer");
    }
    if (configuration_.test(valueId::BAROMETER)) {
        pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>("kebni_driver/pressure", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/pressure");
    }
    if (configuration_.test(valueId::TEMP)) {
        temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("kebni_driver/temperature", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/temperature");
    }
    if (configuration_.test(valueId::TEMP_CALIB)) {
        calibrated_temp_pub_ =
            create_publisher<sensor_msgs::msg::Temperature>("kebni_driver/calibrated_temperature", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/calibrated_temperature");
    }
    if (configuration_.test(valueId::INCL_X)) {
        incl_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("kebni_driver/inclinometer", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/inclinometer");
    }
    if (configuration_.test(valueId::CORR_ACC_X) ||
        configuration_.test(valueId::CORR_GYRO_X))
    {
        corrected_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("kebni_driver/corrected_imu", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/corrected_imu");
    }
    if (configuration_.test(valueId::ODOMETER)) {
        odom_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("kebni_driver/odometer", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/odometer");
    }
    if (configuration_.test(valueId::FRTK_NORTH)) {
        gnss_fixed_relpos_pub_ =
            create_publisher<geometry_msgs::msg::PointStamped>("kebni_driver/gnss_fixed_relpos", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/gnss_fixed_relpos");
    }
    if (configuration_.test(valueId::MRTK_NORTH)) {
        gnss_moving_relpos_pub_ =
            create_publisher<geometry_msgs::msg::PointStamped>("kebni_driver/gnss_moving_relpos", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/gnss_moving_relpos");
    }
    if (configuration_.test(valueId::ERROR_FLAGS)) {
        error_flags_pub_ = create_publisher<std_msgs::msg::UInt32>("kebni_driver/error_flags", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/error_flags");
    }
    if (configuration_.test(valueId::SENSOR_VALID)) {
        sensor_valid_pub_ = create_publisher<std_msgs::msg::UInt8>("kebni_driver/sensor_valid", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/sensor_valid");
    }
    if (configuration_.test(valueId::ALIGNMENT_INS)) {
        alignment_status_pub_ = create_publisher<std_msgs::msg::UInt8>("kebni_driver/alignment_status", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/alignment_status");
    }
    if (configuration_.test(valueId::ATTITUDE_STATUS)) {
        attitude_status_pub_ = create_publisher<std_msgs::msg::UInt32>("kebni_driver/attitude_status", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/attitude_status");
    }
    if (configuration_.test(valueId::NUM_SAT)) {
        gnss1_num_sat_pub_ = create_publisher<std_msgs::msg::Int32>("kebni_driver/gnss1_num_sat", qos);
        gnss2_num_sat_pub_ = create_publisher<std_msgs::msg::Int32>("kebni_driver/gnss2_num_sat", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/gnss1_num_sat, kebni_driver/gnss2_num_sat");
    }
    if (configuration_.test(valueId::UTC_YEAR_MONTH) ||
        configuration_.test(valueId::UTC_DAY_H_MIN_S))
    {
        utc_time_pub_ = create_publisher<sensor_msgs::msg::TimeReference>("kebni_driver/utc_time", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/utc_time");
    }
    if (configuration_.test(valueId::ROTMAT_11)) {
        rotation_matrix_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("kebni_driver/rotation_matrix", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/rotation_matrix");
    }
    if (configuration_.test(valueId::TICK)) {
        system_time_ms_pub_ = create_publisher<std_msgs::msg::Float64>("kebni_driver/system_time_ms", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/system_time_ms");
    }
    if (configuration_.test(valueId::PERF_TICK)) {
        system_time_us_pub_ = create_publisher<std_msgs::msg::Float64>("kebni_driver/system_time_us", qos);
        RCLCPP_INFO(get_logger(), "Publishing: kebni_driver/system_time_us");
    }

    if (configuration_.test(valueId::TICK)) {
        RCLCPP_INFO(get_logger(), "Using sensor system_time_ms for message timestamps");
    } else {
        RCLCPP_WARN(get_logger(),
                    "system_time_ms not configured, using ROS time for message timestamps. "
                    "Recommendation: add sensor 0x42 (system_time_ms) to your configuration_ string "
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

// Setup callback and enter loop reading bytes from the serial port
void KebniNode::serialThread() {
    boost::asio::async_read(*serial_,
        boost::asio::buffer(packetAssembler_.inputBuffer(), 1),
        boost::asio::transfer_exactly(1),
        std::bind(&KebniNode::readCompletionHandler, this, 
        std::placeholders::_1, std::placeholders::_2));

    io_->run(); // Until all async requests have completed OR we call io_.stop()
    RCLCPP_INFO(get_logger(), "Serial thread stopped");
}

void KebniNode::readCompletionHandler(const boost::system::error_code &error, std::size_t bytesTransferred)
{
    if (error)
    {
        // We get an operation_aborted error when we close the port,
        // which is expected according to the documentation.
        if (error != boost::asio::error::operation_aborted)
        {
            RCLCPP_WARN(get_logger(), "Data UART read error: %s", error.message().c_str());
        }
        return; // DO NOT start a new read operation, since the port may be closed
    }

    size_t parseSize = packetAssembler_.processBytes(parseInfo_, bytesTransferred);

    // We immediately start the next async read, to keep the io_service running
    boost::asio::async_read(*serial_,
        boost::asio::buffer(packetAssembler_.inputBuffer(), parseSize),
        boost::asio::transfer_exactly(parseSize),
        std::bind(&KebniNode::readCompletionHandler, this, 
                    std::placeholders::_1, std::placeholders::_2));
}

// Called by sensaition_parser lib (PacketAssembler) when a complete packet is decoded.
// Only publishes a message if the publisher exists and all required fields are present.
void KebniNode::processPacket(std::vector<std::uint8_t>& data) {

    bool parseResult = backend_.parseDataUartData(data, parseInfo_);
    sepa::SensorSample sample = backend_.getSensorSample(configuration_);
    if (!parseResult) {
        RCLCPP_WARN(get_logger(), "Parsing failure");
    }

    try {
        // Use sensor system_time_ms for header stamps, fall back to ROS time
        builtin_interfaces::msg::Time current_time;
        if (sample.contains(Measurement::TICK)) {
            double t = sample.at(Measurement::TICK).toUint32() / 1000.0; // ms to seconds
            current_time.sec = static_cast<int32_t>(t);
            current_time.nanosec = static_cast<uint32_t>((t - current_time.sec) * 1e9);
        } else {
            current_time = now();
        }
        if (pose_pub_ &&
            sample.contains(Measurement::ECEF_POS_X) &&
            sample.contains(Measurement::ECEF_POS_Y) &&
            sample.contains(Measurement::ECEF_POS_Z))
        {
            publishEcefPose(sample, current_time);
        }
        if (imu_pub_ &&
            sample.contains(Measurement::Q_W) &&
            sample.contains(Measurement::Q_X) &&
            sample.contains(Measurement::Q_Y) &&
            sample.contains(Measurement::Q_Z) &&
            sample.contains(Measurement::ACCELERATION_X) &&
            sample.contains(Measurement::ACCELERATION_Y) &&
            sample.contains(Measurement::ACCELERATION_Z) &&
            sample.contains(Measurement::GYRO_RATE_X) &&
            sample.contains(Measurement::GYRO_RATE_Y) &&
            sample.contains(Measurement::GYRO_RATE_Z))
        {
            publishImu(sample, current_time);
        }
        if (vel_pub_ &&
            sample.contains(Measurement::VEL_NORTH) &&
            sample.contains(Measurement::VEL_EAST))
        {
            publishVelocity(sample, current_time);
        }
        if (navsat_pub_ &&
            sample.contains(Measurement::POS_LATITUDE) &&
            sample.contains(Measurement::POS_LONGITUDE))
        {
            publishNavSatFix(sample, current_time);
        }
        if (rpy_pub_ &&
            sample.contains(Measurement::ROLL) &&
            sample.contains(Measurement::PITCH) &&
            sample.contains(Measurement::HEADING))
        {
            publishRpy(sample, current_time);
        }
        if (mag_pub_ &&
            sample.contains(Measurement::MAGNETIC_FIELD_X) &&
            sample.contains(Measurement::MAGNETIC_FIELD_Y) &&
            sample.contains(Measurement::MAGNETIC_FIELD_Z))
        {
            publishMagnetometer(sample, current_time);
        }
        if (pressure_pub_ && sample.contains(Measurement::AIR_PRESSURE))
        {
            publishPressure(sample, current_time);
        }
        if (temp_pub_ && sample.contains(Measurement::INTERNAL_TEMP))
        {
            publishTemperature(sample, current_time);
        }
        if (calibrated_temp_pub_ &&
            sample.contains(Measurement::EXTERNAL_TEMP))
        {
            publishCalibratedTemperature(sample, current_time);
        }
        if (incl_pub_ &&
            sample.contains(Measurement::INCLINOMETER_X) &&
            sample.contains(Measurement::INCLINOMETER_Y) &&
            sample.contains(Measurement::INCLINOMETER_Z))
        {
            publishInclinometer(sample, current_time);
        }
        if (corrected_imu_pub_ &&
            sample.contains(Measurement::CORR_ACCELERATION_X) &&
            sample.contains(Measurement::CORR_ACCELERATION_Y) &&
            sample.contains(Measurement::CORR_ACCELERATION_Z) &&
            sample.contains(Measurement::CORR_GYRO_RATE_X) &&
            sample.contains(Measurement::CORR_GYRO_RATE_Y) &&
            sample.contains(Measurement::CORR_GYRO_RATE_Z))
        {
            publishCorrectedImu(sample, current_time);
        }
        if (odom_pub_ && sample.contains(Measurement::ODOMETER_SPEED))
        {
            publishOdometer(sample, current_time);
        }
        if (gnss_fixed_relpos_pub_ &&
            sample.contains(Measurement::FRTK_NORTH) &&
            sample.contains(Measurement::FRTK_EAST) &&
            sample.contains(Measurement::FRTK_DOWN))
        {
            publishGnssFixedRelpos(sample, current_time);
        }
        if (gnss_moving_relpos_pub_ &&
            sample.contains(Measurement::MRTK_NORTH) &&
            sample.contains(Measurement::MRTK_EAST) &&
            sample.contains(Measurement::MRTK_DOWN))
        {
            publishGnssMovingRelpos(sample, current_time);
        }
        if (error_flags_pub_ && sample.contains(Measurement::ERROR_FLAGS))
        {
            publishErrorFlags(sample);
        }
        if (sensor_valid_pub_ && sample.contains(Measurement::SENSOR_VALID))
        {
            publishSensorValid(sample);
        }
        if (alignment_status_pub_ && sample.contains(Measurement::ALIGNMENT_INS)) {
            publishAlignmentStatus(sample);
        }
        if (attitude_status_pub_ && sample.contains(Measurement::ATTITUDE_STATUS)) {
            publishAttitudeStatus(sample);
        }
        if (gnss1_num_sat_pub_ || gnss2_num_sat_pub_) {
            publishGnssSatCounts(sample);
        }
        if (utc_time_pub_ &&
            sample.contains(Measurement::UTC_YEAR) &&
            sample.contains(Measurement::UTC_MONTH) &&
            sample.contains(Measurement::UTC_DAY) &&
            sample.contains(Measurement::UTC_HOUR) &&
            sample.contains(Measurement::UTC_MINUTE) &&
            sample.contains(Measurement::UTC_SECOND))
        {
            publishUtcTime(sample, current_time);
        }
        if (rotation_matrix_pub_ &&
            sample.contains(Measurement::ROTMAT_11) &&
            sample.contains(Measurement::ROTMAT_12) &&
            sample.contains(Measurement::ROTMAT_13) &&
            sample.contains(Measurement::ROTMAT_21) &&
            sample.contains(Measurement::ROTMAT_22) &&
            sample.contains(Measurement::ROTMAT_23) &&
            sample.contains(Measurement::ROTMAT_31) &&
            sample.contains(Measurement::ROTMAT_32) &&
            sample.contains(Measurement::ROTMAT_33))
        {
            publishRotationMatrix(sample);
        }
        if (system_time_ms_pub_ || system_time_us_pub_) {
            publishSystemTime(sample);
        }
    }
    catch (const std::bad_variant_access&) {
        //sepa::SensorSample::MeasurementValue::toDouble/toUint32/toInt32
        RCLCPP_FATAL(get_logger(), "Kebni sensor-sample variant type error");
    }
}

// ECEF position in meters (X/Y/Z)
void KebniNode::publishEcefPose(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_ecef_;
    msg.point.x = sample.at(Measurement::ECEF_POS_X).toDouble();
    msg.point.y = sample.at(Measurement::ECEF_POS_Y).toDouble();
    msg.point.z = sample.at(Measurement::ECEF_POS_Z).toDouble();
    pose_pub_->publish(msg);
}

// Raw accelerometer, gyroscope, and orientation quaternion (NED)
// Covariance diagonal from quality metrics (variance = sigma^2), -1 if unknown
void KebniNode::publishImu(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;

    constexpr double g_to_m_s2 = 9.80665;
    msg.linear_acceleration.x = sample.at(Measurement::ACCELERATION_X).toDouble() * g_to_m_s2;
    msg.linear_acceleration.y = sample.at(Measurement::ACCELERATION_Y).toDouble() * g_to_m_s2;
    msg.linear_acceleration.z = sample.at(Measurement::ACCELERATION_Z).toDouble() * g_to_m_s2;

    constexpr double deg_to_rad = std::numbers::pi / 180.0;
    msg.angular_velocity.x = sample.at(Measurement::GYRO_RATE_X).toDouble() * deg_to_rad;
    msg.angular_velocity.y = sample.at(Measurement::GYRO_RATE_Y).toDouble() * deg_to_rad;
    msg.angular_velocity.z = sample.at(Measurement::GYRO_RATE_Z).toDouble() * deg_to_rad;

    msg.orientation.w = sample.at(Measurement::Q_W).toDouble();
    msg.orientation.x = sample.at(Measurement::Q_X).toDouble();
    msg.orientation.y = sample.at(Measurement::Q_Y).toDouble();
    msg.orientation.z = sample.at(Measurement::Q_Z).toDouble();

    if (sample.contains(Measurement::STD_ROLL) &&
        sample.contains(Measurement::STD_PITCH) &&
        sample.contains(Measurement::STD_HEADING))
    {
        double qr = sample.at(Measurement::STD_ROLL).toDouble() * deg_to_rad;
        double qp = sample.at(Measurement::STD_PITCH).toDouble() * deg_to_rad;
        double qh = sample.at(Measurement::STD_HEADING).toDouble() * deg_to_rad;
        msg.orientation_covariance = {qr * qr, 0, 0, 0, qp * qp, 0, 0, 0, qh * qh};
    } else {
        msg.orientation_covariance[0] = -1.0;
    }
    imu_pub_->publish(msg);
}

// North/East/Down velocity in m/s (NED frame)
void KebniNode::publishVelocity(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    geometry_msgs::msg::VelocityStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_ned_;
    msg.body_frame_id = frame_id_sensor_;
    msg.reference_frame_id = frame_id_ned_;

    msg.velocity.linear.x = sample.at(Measurement::VEL_NORTH).toDouble();
    msg.velocity.linear.y = sample.at(Measurement::VEL_EAST).toDouble();

    if (sample.contains(Measurement::VEL_DOWN)) {
        msg.velocity.linear.z = sample.at(Measurement::VEL_DOWN).toDouble();
    }
    vel_pub_->publish(msg);
}

bool KebniNode::isGnssFixed(const Sample& sample, Measurement measurement) {
    bool hasFix = false;
    if (sample.contains(measurement)) {
        uint32_t gnssFixType = sample.at(measurement).toUint32();
        hasFix = gnssFixType == 2 || gnssFixType == 3; // NoFix = 0, Fix2D = 2, Fix3D = 3
    }
    return hasFix;
}

// Lat/lon/alt with fix status from both GNSS receivers
// Position covariance diagonal from quality metrics, or UNKNOWN if not available
void KebniNode::publishNavSatFix(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_gps_;

    msg.latitude = sample.at(Measurement::POS_LATITUDE).toDouble();
    msg.longitude = sample.at(Measurement::POS_LONGITUDE).toDouble();

    if (sample.contains(Measurement::POS_VERTICAL)) {
        msg.altitude = sample.at(Measurement::POS_VERTICAL).toDouble();
    }

    msg.status.status =
        isGnssFixed(sample, Measurement::GNSS1_FIXTYPE) ||
        isGnssFixed(sample, Measurement::GNSS2_FIXTYPE) ?
            sensor_msgs::msg::NavSatStatus::STATUS_FIX :
            sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    if (sample.contains(Measurement::STD_LATITUDE) &&
        sample.contains(Measurement::STD_LONGITUDE))
    {
        double qlat = sample.at(Measurement::STD_LATITUDE).toDouble();
        double qlon = sample.at(Measurement::STD_LONGITUDE).toDouble();
        double qalt = sample.contains(Measurement::STD_POS_VERTICAL) ?
            sample.at(Measurement::STD_POS_VERTICAL).toDouble() : 0.0;

        msg.position_covariance = {qlat * qlat, 0, 0, 0, qlon * qlon, 0, 0, 0, qalt * qalt};
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    } else {
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
    navsat_pub_->publish(msg);
}

// Roll, pitch, heading in radians (NED frame)
void KebniNode::publishRpy(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_ned_;
    constexpr double deg_to_rad = std::numbers::pi / 180.0;
    msg.vector.x = sample.at(Measurement::ROLL).toDouble() * deg_to_rad;
    msg.vector.y = sample.at(Measurement::PITCH).toDouble() * deg_to_rad;
    msg.vector.z = sample.at(Measurement::HEADING).toDouble() * deg_to_rad;
    rpy_pub_->publish(msg);
}

// Magnetometer X/Y/Z in Tesla
void KebniNode::publishMagnetometer(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    sensor_msgs::msg::MagneticField msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;
    constexpr double gauss_to_tesla = 1e-4;
    msg.magnetic_field.x = sample.at(Measurement::MAGNETIC_FIELD_X).toDouble() * gauss_to_tesla;
    msg.magnetic_field.y = sample.at(Measurement::MAGNETIC_FIELD_Y).toDouble() * gauss_to_tesla;
    msg.magnetic_field.z = sample.at(Measurement::MAGNETIC_FIELD_Z).toDouble() * gauss_to_tesla;
    mag_pub_->publish(msg);
}

// Barometric pressure in Pascal
void KebniNode::publishPressure(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    sensor_msgs::msg::FluidPressure msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;
    constexpr double hecto_pascal_to_pascal = 100.0;
    msg.fluid_pressure = sample.at(Measurement::AIR_PRESSURE).toDouble() * hecto_pascal_to_pascal;
    pressure_pub_->publish(msg);
}

// Raw IMU temperature in degrees Celsius (non-linear conversion from sensor)
void KebniNode::publishTemperature(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    sensor_msgs::msg::Temperature msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;
    msg.temperature = sample.at(Measurement::INTERNAL_TEMP).toDouble();
    temp_pub_->publish(msg);
}

// Factory-calibrated IMU temperature in degrees Celsius
void KebniNode::publishCalibratedTemperature(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    sensor_msgs::msg::Temperature msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;
    msg.temperature = sample.at(Measurement::EXTERNAL_TEMP).toDouble();
    calibrated_temp_pub_->publish(msg);
}

// Inclinometer X/Y/Z in m/s^2
void KebniNode::publishInclinometer(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;
    constexpr double g_to_m_s2 = 9.80665;
    msg.vector.x = sample.at(Measurement::INCLINOMETER_X).toDouble() * g_to_m_s2;
    msg.vector.y = sample.at(Measurement::INCLINOMETER_Y).toDouble() * g_to_m_s2;
    msg.vector.z = sample.at(Measurement::INCLINOMETER_Z).toDouble() * g_to_m_s2;
    incl_pub_->publish(msg);
}

// Sensor-fusion corrected accelerometer and gyroscope
// No orientation available, covariance[0] = -1 marks it as unknown
void KebniNode::publishCorrectedImu(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;

    constexpr double g_to_m_s2 = 9.80665;
    msg.linear_acceleration.x = sample.at(Measurement::CORR_ACCELERATION_X).toDouble() * g_to_m_s2;
    msg.linear_acceleration.y = sample.at(Measurement::CORR_ACCELERATION_Y).toDouble() * g_to_m_s2;
    msg.linear_acceleration.z = sample.at(Measurement::CORR_ACCELERATION_Z).toDouble() * g_to_m_s2;

    constexpr double deg_to_rad = std::numbers::pi / 180.0;
    msg.angular_velocity.x = sample.at(Measurement::CORR_GYRO_RATE_X).toDouble() * deg_to_rad;
    msg.angular_velocity.y = sample.at(Measurement::CORR_GYRO_RATE_Y).toDouble() * deg_to_rad;
    msg.angular_velocity.z = sample.at(Measurement::CORR_GYRO_RATE_Z).toDouble() * deg_to_rad;

    msg.orientation_covariance[0] = -1.0;

    corrected_imu_pub_->publish(msg);
}

// Odometer speed in m/s (linear.x only)
void KebniNode::publishOdometer(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_sensor_;
    msg.twist.linear.x = sample.at(Measurement::ODOMETER_SPEED).toDouble();
    odom_pub_->publish(msg);
}

// Fixed baseline GNSS relative position N/E/D in meters
void KebniNode::publishGnssFixedRelpos(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_ned_;
    msg.point.x = sample.at(Measurement::FRTK_NORTH).toDouble();
    msg.point.y = sample.at(Measurement::FRTK_EAST).toDouble();
    msg.point.z = sample.at(Measurement::FRTK_DOWN).toDouble();
    gnss_fixed_relpos_pub_->publish(msg);
}

// Moving baseline GNSS relative position N/E/D in meters
void KebniNode::publishGnssMovingRelpos(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id_ned_;
    msg.point.x = sample.at(Measurement::MRTK_NORTH).toDouble();
    msg.point.y = sample.at(Measurement::MRTK_EAST).toDouble();
    msg.point.z = sample.at(Measurement::MRTK_DOWN).toDouble();
    gnss_moving_relpos_pub_->publish(msg);
}

// 26-bit error bitmask (see ErrorFlags struct for bit definitions)
void KebniNode::publishErrorFlags(const Sample& sample) {
    std_msgs::msg::UInt32 msg;
    msg.data = sample.at(Measurement::ERROR_FLAGS).toUint32();
    error_flags_pub_->publish(msg);
}

// 8-bit sensor availability bitmask (see SensorValidFlags struct for bit definitions)
void KebniNode::publishSensorValid(const Sample& sample) {
    std_msgs::msg::UInt8 msg;
    msg.data = static_cast<uint8_t>(sample.at(Measurement::SENSOR_VALID).toUint32());
    sensor_valid_pub_->publish(msg);
}

// 0 = WaitingForGnssFix, 1 = NavigationRunning
void KebniNode::publishAlignmentStatus(const Sample& sample) {
    std_msgs::msg::UInt8 msg;
    msg.data = static_cast<uint8_t>(sample.at(Measurement::ALIGNMENT_INS).toUint32());
    alignment_status_pub_->publish(msg);
}

void KebniNode::publishAttitudeStatus(const Sample& sample) {
    std_msgs::msg::UInt32 msg;
    msg.data = sample.at(Measurement::ATTITUDE_STATUS).toUint32();
    attitude_status_pub_->publish(msg);
}

// Raw attitude status bitfield
// Satellite count for each GNSS receiver (packed from sensor 0x2B)
void KebniNode::publishGnssSatCounts(const Sample& sample) {
    if (gnss1_num_sat_pub_ && sample.contains(Measurement::GNSS1_NUM_SAT)) {
        std_msgs::msg::Int32 msg;
        msg.data = static_cast<int32_t>(sample.at(Measurement::GNSS1_NUM_SAT).toUint32());
        gnss1_num_sat_pub_->publish(msg);
    }
    if (gnss2_num_sat_pub_ && sample.contains(Measurement::GNSS2_NUM_SAT)) {
        std_msgs::msg::Int32 msg;
        msg.data = static_cast<int32_t>(sample.at(Measurement::GNSS2_NUM_SAT).toUint32());
        gnss2_num_sat_pub_->publish(msg);
    }
}

// UTC calendar fields converted to Unix epoch seconds via timegm()
// Sub-second precision from utc_sub_second if available
void KebniNode::publishUtcTime(const Sample& sample, const builtin_interfaces::msg::Time &t) {
    int year = static_cast<int>(sample.at(Measurement::UTC_YEAR).toInt32());
    int month = static_cast<int>(sample.at(Measurement::UTC_MONTH).toInt32());
    int day = static_cast<int>(sample.at(Measurement::UTC_DAY).toInt32());
    int hour = static_cast<int>(sample.at(Measurement::UTC_HOUR).toInt32());
    int min = static_cast<int>(sample.at(Measurement::UTC_MINUTE).toInt32());
    int sec = static_cast<int>(sample.at(Measurement::UTC_SECOND).toInt32());

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
    msg.time_ref.nanosec = sample.contains(Measurement::UTC_US) ?
        sample.at(Measurement::UTC_US).toUint32() * 1e3 : 0u;
    utc_time_pub_->publish(msg);
}

// 3x3 rotation matrix in row-major order (stride 3 per row, 1 per column)
void KebniNode::publishRotationMatrix(const Sample& sample) {
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "rows";
    msg.layout.dim[0].size = 3;
    msg.layout.dim[0].stride = 3;
    msg.layout.dim[1].label = "cols";
    msg.layout.dim[1].size = 3;
    msg.layout.dim[1].stride = 1;
    msg.data = {
        sample.at(Measurement::ROTMAT_11).toDouble(),
        sample.at(Measurement::ROTMAT_12).toDouble(),
        sample.at(Measurement::ROTMAT_13).toDouble(),
        sample.at(Measurement::ROTMAT_21).toDouble(),
        sample.at(Measurement::ROTMAT_22).toDouble(),
        sample.at(Measurement::ROTMAT_23).toDouble(),
        sample.at(Measurement::ROTMAT_31).toDouble(),
        sample.at(Measurement::ROTMAT_32).toDouble(),
        sample.at(Measurement::ROTMAT_33).toDouble()
    };
    rotation_matrix_pub_->publish(msg);
}

// Raw sensor timestamps (ms and us published separately)
void KebniNode::publishSystemTime(const Sample& sample) {
    if (system_time_ms_pub_ && sample.contains(Measurement::TICK)) {
        std_msgs::msg::Float64 msg;
        msg.data = static_cast<double>(sample.at(Measurement::TICK).toUint32());
        system_time_ms_pub_->publish(msg);
    }
    if (system_time_us_pub_ && sample.contains(Measurement::PERF_TICK)) {
        std_msgs::msg::Float64 msg;
        msg.data = static_cast<double>(sample.at(Measurement::PERF_TICK).toInt32());
        system_time_us_pub_->publish(msg);
    }
}

// Debug print
void KebniNode::printConfiguration(const std::string& confError) const {

    bool configured = true;
    uint16_t freq_div = parseInfo_.messages[0].dataRate;

    std::cout << "\nKebni Driver Configuration:" << std::endl;
    std::cout << "Configured: " << (configured ? "yes" : "no") << std::endl;
    std::cout << "Config error: " <<
        (confError.empty() ? "No error" : confError.c_str()) << std::endl;
    std::cout << "Frequency divisor: " << freq_div << std::endl;
    std::cout << "Frequency: " << 1000.0/freq_div << " Hz\n";
    std::cout << "Checksum type: " <<
        (parseInfo_.messages.empty() ? "N/A" : 
         parseInfo_.messages[0].checksumBytes == 1 ? "XOR8" :
         parseInfo_.messages[0].checksumBytes == 2 ? "CRC16" : "N/A") <<
        " ('" << (parseInfo_.messages.empty() ? " " :
                  parseInfo_.messages[0].checksumBytes == 1 ? "x" :
                  parseInfo_.messages[0].checksumBytes == 2 ? "X" : " ") << "')\n";
    std::cout << std::endl;
}

KebniNode::~KebniNode() {

    if (serial_ && serial_->is_open()) {
        serial_->cancel();
        serial_->close();
    }

    if (serial_thread_.joinable()) {
        serial_thread_.join();
    }
}
