#pragma once

#include <array>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/macros.hpp"

namespace rover0_hardware_interface
{

    class WheelState
    {
    public:
        double position;
        double velocity;

        WheelState(std::string &name, int16_t encoder_tics_per_revolution);
        ~WheelState() = default;

        void insert_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces);
        void update(int16_t delta_tics, double delta_secs);

    private:
        std::string name;
        int16_t const encoder_tics_per_revolution;

        hardware_interface::StateInterface create_position_state_interface();
        hardware_interface::StateInterface create_velocity_state_interface();
    };

    class WheelCommand
    {
    public:
        double velocity;

        WheelCommand(std::string &name);
        ~WheelCommand() = default;

        void insert_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces);

    private:
        std::string name;

        hardware_interface::CommandInterface create_velocity_command_interface();
    };

    class IMUState
    {
    public:
        IMUState(
            std::string &name,
            std::array<double, 3> const &linear_acceleration_offset,
            std::array<double, 3> const &angular_velocity_offset);
        ~IMUState() = default;

        void insert_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces);
        void update(message::ImuData param, int16_t value);

    private:
        std::string name;
        double orientation_{std::numeric_limits<double>::quiet_NaN()};
        std::array<double, 3> linear_acceleration_{0.0, 0.0, 0.0};
        std::array<double, 3> angular_velocity_{0.0, 0.0, 0.0};
        std::array<double, 3> linear_acceleration_offset_{0.0, 0.0, 0.0};
        std::array<double, 3> angular_velocity_offset_{0.0, 0.0, 0.0};

        double calc_linear_acceleration(int16_t value, double offset) const;
        double calc_angular_velocity(int16_t value, double offset) const;
        hardware_interface::StateInterface create_orientation_x_state_interface();
        hardware_interface::StateInterface create_orientation_y_state_interface();
        hardware_interface::StateInterface create_orientation_z_state_interface();
        hardware_interface::StateInterface create_orientation_w_state_interface();
        hardware_interface::StateInterface create_angular_velocity_x_state_interface();
        hardware_interface::StateInterface create_angular_velocity_y_state_interface();
        hardware_interface::StateInterface create_angular_velocity_z_state_interface();
        hardware_interface::StateInterface create_linear_acceleration_x_state_interface();
        hardware_interface::StateInterface create_linear_acceleration_y_state_interface();
        hardware_interface::StateInterface create_linear_acceleration_z_state_interface();
    };

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
            int16_t encoder_tics_per_revolution{64};
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
        uint64_t read_call_count_{0};
        Config config_;
        std::optional<IMUState> imu_state_{std::nullopt};
        std::map<message::MotorDevice, WheelState> wheel_states_{};
        std::map<message::MotorDevice, WheelCommand> wheel_commands_{};

        double seconds_from_last_motor_query_{0.0};

        bool should_query_motor() const;
        void handle_imu_message(const message::ImuMsg &imu_msg);
        void handle_encoder_message(const message::EncoderMsg &encoder_msg, double delta_secs);
    };
}
