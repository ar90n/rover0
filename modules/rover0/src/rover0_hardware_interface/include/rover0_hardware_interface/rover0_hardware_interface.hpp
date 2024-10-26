#pragma once

#include <array>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/macros.hpp"

namespace rover0_hardware_interface
{
    class Rover0HardwareInterface : public hardware_interface::SystemInterface
    {
        struct Config
        {
            std::string front_left_wheel_joint_name{"front_left_wheel_joint"};
            std::string front_right_wheel_joint_name{"front_right_wheel_joint"};
            std::string rear_left_wheel_joint_name{"rear_left_wheel_joint"};
            std::string rear_right_wheel_joint_name{"rear_right_wheel_joint"};
            std::string imu_sensor_name{"imu_sensor"};
            std::string serial_port{"/dev/ttyUSB0"};
            int timeout{1000};
        };

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Rover0HardwareInterface)

        Rover0HardwareInterface() = default;
        virtual ~Rover0HardwareInterface() = default;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

    private:
        int fd_{-1};
        Config config;
        std::array<double, 4> orientation_{0.0, 0.0, 0.0, 0.0};
        std::array<double, 3> angular_velocity_{0.0, 0.0, 0.0};
        std::array<double, 3> linear_acceleration_{0.0, 0.0, 0.0};
        std::map<message::ImuData, double &> const imu_data_map{
            {message::ImuData::ACCEL_X, linear_acceleration_[0]},
            {message::ImuData::ACCEL_Y, linear_acceleration_[1]},
            {message::ImuData::ACCEL_Z, linear_acceleration_[2]},
            {message::ImuData::GYRO_X, angular_velocity_[0]},
            {message::ImuData::GYRO_Y, angular_velocity_[1]},
            {message::ImuData::GYRO_Z, angular_velocity_[2]},
        };
        std::array<double, 4> wheel_velocities_{0.0, 0.0, 0.0, 0.0};
        std::array<double, 4> wheel_positions_{0.0, 0.0, 0.0, 0.0};
        std::map<message::MotorDevice, double &> const wheel_data_map{
            {message::MotorDevice::FRONT_LEFT, wheel_velocities_[0]},
            {message::MotorDevice::FRONT_RIGHT, wheel_velocities_[1]},
            {message::MotorDevice::REAR_LEFT, wheel_velocities_[2]},
            {message::MotorDevice::REAR_RIGHT, wheel_velocities_[3]},
        };
    };
}
