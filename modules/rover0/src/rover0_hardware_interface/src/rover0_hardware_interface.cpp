#include "rover0_hardware_interface/rover0_hardware_interface.hpp"

hardware_interface::CallbackReturn rover0_hardware_interface::Rover0HardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn rover0_hardware_interface::Rover0HardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn rover0_hardware_interface::Rover0HardwareInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
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

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> rover0_hardware_interface::Rover0HardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    return command_interfaces;
}

hardware_interface::return_type rover0_hardware_interface::Rover0HardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type rover0_hardware_interface::Rover0HardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(rover0_hardware_interface::Rover0HardwareInterface, hardware_interface::SystemInterface)
