#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <variant>

#include <iostream>
#include <map>
#include <numbers>

#include <transport.hpp>
#include <message.hpp>

#include "rover0_hardware_interface/rover0_hardware_interface.hpp"

namespace
{
    static constexpr uint SKIP_READ_COUNT_FOR_MOTOR_QUERY{10};
    std::array<uint32_t, 6> const imu_queries{
        message::serialize(message::ImuMsg{
            .param = message::ImuData::ACCEL_X,
        }),
        message::serialize(message::ImuMsg{
            .param = message::ImuData::ACCEL_Y,
        }),
        message::serialize(message::ImuMsg{
            .param = message::ImuData::ACCEL_Z,
        }),
        message::serialize(message::ImuMsg{
            .param = message::ImuData::GYRO_X,
        }),
        message::serialize(message::ImuMsg{
            .param = message::ImuData::GYRO_Y,
        }),
        message::serialize(message::ImuMsg{
            .param = message::ImuData::GYRO_Z,
        })};
    std::array<uint32_t, 4> const motor_queries{
        message::serialize(message::EncoderMsg{
            .param = message::MotorDevice::REAR_LEFT,
        }),
        message::serialize(message::EncoderMsg{
            .param = message::MotorDevice::REAR_RIGHT,
        }),
        message::serialize(message::EncoderMsg{
            .param = message::MotorDevice::FRONT_LEFT,
        }),
        message::serialize(message::EncoderMsg{
            .param = message::MotorDevice::FRONT_RIGHT,
        }),
    };

    double calc_motor_query_delta_secs(rclcpp::Duration const &period)
    {
        return SKIP_READ_COUNT_FOR_MOTOR_QUERY * period.seconds();
    }
}

rover0_hardware_interface::WheelState::WheelState(std::string &name, int16_t encoder_tics_per_revolution)
    : position{0.0}, velocity{0.0}, name{name}, encoder_tics_per_revolution{encoder_tics_per_revolution} {}

void rover0_hardware_interface::WheelState::insert_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces)
{
    state_interfaces.emplace_back(create_position_state_interface());
    state_interfaces.emplace_back(create_velocity_state_interface());
}

void rover0_hardware_interface::WheelState::update(int16_t delta_tics, double delta_secs)
{
    double const delta_position = 2.0 * std::numbers::pi * static_cast<double>(delta_tics) / encoder_tics_per_revolution;
    position += delta_position;
    velocity = delta_position / delta_secs;
}

hardware_interface::StateInterface rover0_hardware_interface::WheelState::create_position_state_interface()
{
    return hardware_interface::StateInterface(name, hardware_interface::HW_IF_POSITION, &position);
}

hardware_interface::StateInterface rover0_hardware_interface::WheelState::create_velocity_state_interface()
{
    return hardware_interface::StateInterface(name, hardware_interface::HW_IF_VELOCITY, &velocity);
}

rover0_hardware_interface::WheelCommand::WheelCommand(std::string &name)
    : velocity{0.0}, name{name}
{
}

void rover0_hardware_interface::WheelCommand::insert_command_interfaces(std::vector<hardware_interface::CommandInterface> &command_interfaces)
{
    command_interfaces.emplace_back(create_velocity_command_interface());
}

hardware_interface::CommandInterface rover0_hardware_interface::WheelCommand::create_velocity_command_interface()
{
    return hardware_interface::CommandInterface(name, hardware_interface::HW_IF_VELOCITY, &velocity);
}

rover0_hardware_interface::IMUState::IMUState(
    std::string &name,
    std::array<double, 3> const &linear_acceleration_offset,
    std::array<double, 3> const &angular_velocity_offset)
    : name{name},
      linear_acceleration_offset_{linear_acceleration_offset},
      angular_velocity_offset_{angular_velocity_offset}
{
}

void rover0_hardware_interface::IMUState::insert_state_interfaces(std::vector<hardware_interface::StateInterface> &state_interfaces)
{
    state_interfaces.emplace_back(create_orientation_x_state_interface());
    state_interfaces.emplace_back(create_orientation_y_state_interface());
    state_interfaces.emplace_back(create_orientation_z_state_interface());
    state_interfaces.emplace_back(create_orientation_w_state_interface());
    state_interfaces.emplace_back(create_angular_velocity_x_state_interface());
    state_interfaces.emplace_back(create_angular_velocity_y_state_interface());
    state_interfaces.emplace_back(create_angular_velocity_z_state_interface());
    state_interfaces.emplace_back(create_linear_acceleration_x_state_interface());
    state_interfaces.emplace_back(create_linear_acceleration_y_state_interface());
    state_interfaces.emplace_back(create_linear_acceleration_z_state_interface());
}

void rover0_hardware_interface::IMUState::update(message::ImuData param, int16_t value)
{
    switch (param)
    {
    case message::ImuData::GYRO_X:
        angular_velocity_[0] = calc_angular_velocity(value, angular_velocity_offset_[0]);
        break;
    case message::ImuData::GYRO_Y:
        angular_velocity_[1] = calc_angular_velocity(value, angular_velocity_offset_[1]);
        break;
    case message::ImuData::GYRO_Z:
        angular_velocity_[2] = calc_angular_velocity(value, angular_velocity_offset_[2]);
        break;
    case message::ImuData::ACCEL_X:
        linear_acceleration_[0] = calc_linear_acceleration(value, linear_acceleration_offset_[0]);
        break;
    case message::ImuData::ACCEL_Y:
        linear_acceleration_[1] = calc_linear_acceleration(value, linear_acceleration_offset_[1]);
        break;
    case message::ImuData::ACCEL_Z:
        linear_acceleration_[2] = calc_linear_acceleration(value, 0.0); // skip offset for z-axis because of gravity
        break;
    }
}

double rover0_hardware_interface::IMUState::calc_linear_acceleration(int16_t value, double offset) const
{
    static constexpr double acc_coef{2 * 2.0 / 65536.0};
    return acc_coef * static_cast<double>(value) - offset;
}

double rover0_hardware_interface::IMUState::calc_angular_velocity(int16_t value, double offset) const
{
    static constexpr double gyro_coef{2 * 250 / 65536.0};
    return gyro_coef * static_cast<double>(value) - offset;
}

hardware_interface::StateInterface rover0_hardware_interface::IMUState::create_orientation_x_state_interface()
{
    return hardware_interface::StateInterface(name, "orientation.x", &orientation_);
}

hardware_interface::StateInterface rover0_hardware_interface::IMUState::create_orientation_y_state_interface()
{
    return hardware_interface::StateInterface(name, "orientation.y", &orientation_);
}

hardware_interface::StateInterface rover0_hardware_interface::IMUState::create_orientation_z_state_interface()
{
    return hardware_interface::StateInterface(name, "orientation.z", &orientation_);
}

hardware_interface::StateInterface rover0_hardware_interface::IMUState::create_orientation_w_state_interface()
{
    return hardware_interface::StateInterface(name, "orientation.w", &orientation_);
}

hardware_interface::StateInterface rover0_hardware_interface::IMUState::create_angular_velocity_x_state_interface()
{
    return hardware_interface::StateInterface(name, "angular_velocity.x", &angular_velocity_[0]);
}

hardware_interface::StateInterface rover0_hardware_interface::IMUState::create_angular_velocity_y_state_interface()
{
    return hardware_interface::StateInterface(name, "angular_velocity.y", &angular_velocity_[1]);
}

hardware_interface::StateInterface rover0_hardware_interface::IMUState::create_angular_velocity_z_state_interface()
{
    return hardware_interface::StateInterface(name, "angular_velocity.z", &angular_velocity_[2]);
}

hardware_interface::StateInterface rover0_hardware_interface::IMUState::create_linear_acceleration_x_state_interface()
{
    return hardware_interface::StateInterface(name, "linear_acceleration.x", &linear_acceleration_[0]);
}

hardware_interface::StateInterface rover0_hardware_interface::IMUState::create_linear_acceleration_y_state_interface()
{
    return hardware_interface::StateInterface(name, "linear_acceleration.y", &linear_acceleration_[1]);
}

hardware_interface::StateInterface rover0_hardware_interface::IMUState::create_linear_acceleration_z_state_interface()
{
    return hardware_interface::StateInterface(name, "linear_acceleration.z", &linear_acceleration_[2]);
}

hardware_interface::CallbackReturn rover0_hardware_interface::Rover0HardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    config_.front_left_wheel_joint_name = info.hardware_parameters.at("front_left_wheel_joint_name");
    config_.front_right_wheel_joint_name = info.hardware_parameters.at("front_right_wheel_joint_name");
    config_.rear_left_wheel_joint_name = info.hardware_parameters.at("rear_left_wheel_joint_name");
    config_.rear_right_wheel_joint_name = info.hardware_parameters.at("rear_right_wheel_joint_name");
    config_.imu_sensor_name = info.hardware_parameters.at("imu_sensor_name");
    config_.serial_port = info.hardware_parameters.at("serial_port");
    config_.encoder_tics_per_revolution = std::stoi(info.hardware_parameters.at("encoder_tics_per_revolution"));

    std::array<double, 3> const linear_acceleration_offset{
        std::stod(info.hardware_parameters.at("imu_linear_acceleration_offset_x")),
        std::stod(info.hardware_parameters.at("imu_linear_acceleration_offset_y")),
        std::stod(info.hardware_parameters.at("imu_linear_acceleration_offset_z"))};
    std::array<double, 3> const angular_velocity_offset{
        std::stod(info.hardware_parameters.at("imu_angular_velocity_offset_x")),
        std::stod(info.hardware_parameters.at("imu_angular_velocity_offset_y")),
        std::stod(info.hardware_parameters.at("imu_angular_velocity_offset_z"))};
    imu_state_ = IMUState(config_.imu_sensor_name, linear_acceleration_offset, angular_velocity_offset);

    wheel_states_.emplace(message::MotorDevice::REAR_LEFT, WheelState(config_.rear_left_wheel_joint_name, config_.encoder_tics_per_revolution));
    wheel_states_.emplace(message::MotorDevice::REAR_RIGHT, WheelState(config_.rear_right_wheel_joint_name, config_.encoder_tics_per_revolution));
    wheel_states_.emplace(message::MotorDevice::FRONT_LEFT, WheelState(config_.front_left_wheel_joint_name, config_.encoder_tics_per_revolution));
    wheel_states_.emplace(message::MotorDevice::FRONT_RIGHT, WheelState(config_.front_right_wheel_joint_name, config_.encoder_tics_per_revolution));

    wheel_commands_.emplace(message::MotorDevice::REAR_LEFT, WheelCommand(config_.rear_left_wheel_joint_name));
    wheel_commands_.emplace(message::MotorDevice::REAR_RIGHT, WheelCommand(config_.rear_right_wheel_joint_name));
    wheel_commands_.emplace(message::MotorDevice::FRONT_LEFT, WheelCommand(config_.front_left_wheel_joint_name));
    wheel_commands_.emplace(message::MotorDevice::FRONT_RIGHT, WheelCommand(config_.front_right_wheel_joint_name));

    for (const hardware_interface::ComponentInfo &joint : info.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(rclcpp::get_logger("rover0_hardware_interface"), "Joint '%s' has %d command interfaces, 1 expected", joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(rclcpp::get_logger("rover0_hardware_interface"), "Joint '%s' has command interface '%s', 'velocity' expected", joint.name.c_str(), joint.command_interfaces[0].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1)
        {
            RCLCPP_FATAL(rclcpp::get_logger("rover0_hardware_interface"), "Joint '%s' has %d state interfaces, 1 expected", joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("rover0_hardware_interface"), "Joint '%s' has state interface '%s', 'position' expected", joint.name.c_str(), joint.state_interfaces[0].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn rover0_hardware_interface::Rover0HardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    fd_ = open(config_.serial_port.c_str(), O_RDWR);
    if (fd_ < 0)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    struct termios tio
    {
        .c_cflag = CREAD | CLOCAL | CS8,
    };
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    cfmakeraw(&tio);
    tcsetattr(fd_, TCSANOW, &tio);
    ioctl(fd_, TCSETS, &tio);
    fcntl(fd_, F_SETFL, O_NONBLOCK);

    transport::init([this](const uint8_t *data, size_t size)
                    { ::write(this->fd_, data, size); });

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn rover0_hardware_interface::Rover0HardwareInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    auto const ret{close(fd_)};
    if (ret < 0)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn rover0_hardware_interface::Rover0HardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn rover0_hardware_interface::Rover0HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> rover0_hardware_interface::Rover0HardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    imu_state_->insert_state_interfaces(state_interfaces);
    wheel_states_.at(message::MotorDevice::FRONT_LEFT).insert_state_interfaces(state_interfaces);
    wheel_states_.at(message::MotorDevice::FRONT_RIGHT).insert_state_interfaces(state_interfaces);
    wheel_states_.at(message::MotorDevice::REAR_LEFT).insert_state_interfaces(state_interfaces);
    wheel_states_.at(message::MotorDevice::REAR_RIGHT).insert_state_interfaces(state_interfaces);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> rover0_hardware_interface::Rover0HardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    wheel_commands_.at(message::MotorDevice::FRONT_LEFT).insert_command_interfaces(command_interfaces);
    wheel_commands_.at(message::MotorDevice::FRONT_RIGHT).insert_command_interfaces(command_interfaces);
    wheel_commands_.at(message::MotorDevice::REAR_LEFT).insert_command_interfaces(command_interfaces);
    wheel_commands_.at(message::MotorDevice::REAR_RIGHT).insert_command_interfaces(command_interfaces);

    return command_interfaces;
}

hardware_interface::return_type rover0_hardware_interface::Rover0HardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    read_call_count_++;

    transport::reset();
    for (auto const &query : imu_queries)
    {
        transport::send(query);
    }

    if (should_query_motor())
    {
        for (auto const &query : motor_queries)
        {
            transport::send(query);
        }
    }

    uint8_t buf[256];
    double const motor_query_delta_secs{calc_motor_query_delta_secs(period)};
    while (true)
    {
        auto const len = ::read(fd_, buf, sizeof(buf));
        if (len < 0)
        {
            break;
        }

        for (int i = 0; i < len; i++)
        {
            auto const recv{transport::consume(buf[i])};
            if (!recv.has_value())
            {
                continue;
            }

            if (message::get_msg_type(recv.value()) == message::MsgType::IMU)
            {
                handle_imu_message(message::deserialize<message::ImuMsg>(recv.value()));
            }
            else if (message::get_msg_type(recv.value()) == message::MsgType::ENCODER)
            {
                handle_encoder_message(message::deserialize<message::EncoderMsg>(recv.value()), motor_query_delta_secs);
            }
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type rover0_hardware_interface::Rover0HardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    for (auto &command : wheel_commands_)
    {
        double const rotate_per_sec = command.second.velocity / (2.0 * std::numbers::pi); // command velocity is in rad/s
        double const ticks_per_sec = config_.encoder_tics_per_revolution * rotate_per_sec;
        int16_t const tics_per_sec_q7 = static_cast<int16_t>(ticks_per_sec * 128.0 + 0.5);
        transport::send(message::serialize(message::MotorMsg{
            .param = command.first,
            .value = tics_per_sec_q7}));
    }
    return hardware_interface::return_type::OK;
}

bool rover0_hardware_interface::Rover0HardwareInterface::should_query_motor() const
{
    return (read_call_count_ % SKIP_READ_COUNT_FOR_MOTOR_QUERY) == 0;
}

void rover0_hardware_interface::Rover0HardwareInterface::handle_imu_message(const message::ImuMsg &imu_msg)
{
    imu_state_->update(imu_msg.param, imu_msg.value);
}

void rover0_hardware_interface::Rover0HardwareInterface::handle_encoder_message(const message::EncoderMsg &encoder_msg, double delta_secs)
{
    if (wheel_states_.find(encoder_msg.param) != wheel_states_.end())
    {
        wheel_states_.at(encoder_msg.param).update(encoder_msg.value, delta_secs);
    }
}

PLUGINLIB_EXPORT_CLASS(rover0_hardware_interface::Rover0HardwareInterface, hardware_interface::SystemInterface)
