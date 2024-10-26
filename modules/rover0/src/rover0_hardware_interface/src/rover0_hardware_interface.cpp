#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <variant>

#include <iostream>
#include <map>

#include <transport.hpp>
#include <message.hpp>

#include "rover0_hardware_interface/rover0_hardware_interface.hpp"

namespace
{
    std::array<uint32_t, 10> const queries{
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
        }),
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
}

hardware_interface::CallbackReturn rover0_hardware_interface::Rover0HardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    config.front_left_wheel_joint_name = info.hardware_parameters.at("front_left_wheel_joint_name");
    config.front_right_wheel_joint_name = info.hardware_parameters.at("front_right_wheel_joint_name");
    config.rear_left_wheel_joint_name = info.hardware_parameters.at("rear_left_wheel_joint_name");
    config.rear_right_wheel_joint_name = info.hardware_parameters.at("rear_right_wheel_joint_name");
    config.imu_sensor_name = info.hardware_parameters.at("imu_sensor_name");
    config.serial_port = info.hardware_parameters.at("serial_port");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn rover0_hardware_interface::Rover0HardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    fd_ = open(config.serial_port.c_str(), O_RDWR);
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

    state_interfaces.emplace_back(hardware_interface::StateInterface(config.imu_sensor_name, "orientation.x", &orientation_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.imu_sensor_name, "orientation.y", &orientation_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.imu_sensor_name, "orientation.z", &orientation_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.imu_sensor_name, "orientation.w", &orientation_[3]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.imu_sensor_name, "angular_velocity.x", &angular_velocity_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.imu_sensor_name, "angular_velocity.y", &angular_velocity_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.imu_sensor_name, "angular_velocity.z", &angular_velocity_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.imu_sensor_name, "linear_acceleration.x", &linear_acceleration_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.imu_sensor_name, "linear_acceleration.y", &linear_acceleration_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.imu_sensor_name, "linear_acceleration.z", &linear_acceleration_[2]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(config.front_left_wheel_joint_name, hardware_interface::HW_IF_VELOCITY, &wheel_velocities_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.front_right_wheel_joint_name, hardware_interface::HW_IF_VELOCITY, &wheel_velocities_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.rear_left_wheel_joint_name, hardware_interface::HW_IF_VELOCITY, &wheel_velocities_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.rear_right_wheel_joint_name, hardware_interface::HW_IF_VELOCITY, &wheel_velocities_[3]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(config.front_left_wheel_joint_name, hardware_interface::HW_IF_POSITION, &wheel_positions_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.front_right_wheel_joint_name, hardware_interface::HW_IF_POSITION, &wheel_positions_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.rear_left_wheel_joint_name, hardware_interface::HW_IF_POSITION, &wheel_positions_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(config.rear_right_wheel_joint_name, hardware_interface::HW_IF_POSITION, &wheel_positions_[3]));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> rover0_hardware_interface::Rover0HardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    return command_interfaces;
}

hardware_interface::return_type rover0_hardware_interface::Rover0HardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    transport::reset();
    for (auto const &query : queries)
    {
        transport::send(query);
    }

    uint8_t buf[256];
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
                auto const &imu_msg{message::deserialize<message::ImuMsg>(recv.value())};
                imu_data_map.at(imu_msg.param) = imu_msg.value;
            }
            else if (message::get_msg_type(recv.value()) == message::MsgType::ENCODER)
            {
                auto const &encoder_msg{message::deserialize<message::EncoderMsg>(recv.value())};
                wheel_data_map.at(encoder_msg.param) = encoder_msg.value;
            }
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type rover0_hardware_interface::Rover0HardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(rover0_hardware_interface::Rover0HardwareInterface, hardware_interface::SystemInterface)
