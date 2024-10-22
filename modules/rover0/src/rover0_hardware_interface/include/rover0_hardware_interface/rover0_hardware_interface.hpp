#pragma once

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/macros.hpp"

namespace rover0_hardware_interface
{
    class Rover0HardwareInterface : public hardware_interface::SystemInterface
    {
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
    };
}
