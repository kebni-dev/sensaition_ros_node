#pragma once

#include <atomic>
#include <boost/asio.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/velocity_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <thread>

#include "kebni_driver/kebni_driver.hpp"

class KebniNode : public rclcpp::Node, public kebni::KebniDriver {
  public:
    KebniNode();
    ~KebniNode();

    void onMeasurements(const kebni::Measurements &measurements) override;

  private:
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::VelocityStamped>::SharedPtr vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr calibrated_temp_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr incl_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr corrected_imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr gnss_fixed_relpos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr gnss_moving_relpos_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr error_flags_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr sensor_valid_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr alignment_status_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr attitude_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gnss1_num_sat_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gnss2_num_sat_pub_;
    rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr utc_time_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rotation_matrix_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr system_time_ms_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr system_time_us_pub_;

    void createPublishers();
    void startSerial(const std::string &port, int baudRate);
    void serialThread();

    void publishEcefPose(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishImu(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishVelocity(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishNavSatFix(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishRpy(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishMagnetometer(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishPressure(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishTemperature(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishCalibratedTemperature(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishInclinometer(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishCorrectedImu(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishOdometer(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishGnssFixedRelpos(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishGnssMovingRelpos(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishErrorFlags(const kebni::Measurements &m);
    void publishSensorValid(const kebni::Measurements &m);
    void publishAlignmentStatus(const kebni::Measurements &m);
    void publishAttitudeStatus(const kebni::Measurements &m);
    void publishGnssSatCounts(const kebni::Measurements &m);
    void publishUtcTime(const kebni::Measurements &m, const builtin_interfaces::msg::Time &t);
    void publishRotationMatrix(const kebni::Measurements &m);
    void publishSystemTime(const kebni::Measurements &m);

    std::unique_ptr<boost::asio::serial_port> serial_;
    std::unique_ptr<boost::asio::io_context> io_;
    std::atomic<bool> running_{true};
    kebni::Configuration configuration;
    std::thread serial_thread_;
    std::string serial_port_;
    int baud_rate_;
    std::string frame_id_sensor_;
    std::string frame_id_ned_;
    std::string frame_id_ecef_;
    std::string frame_id_gps_;
};
