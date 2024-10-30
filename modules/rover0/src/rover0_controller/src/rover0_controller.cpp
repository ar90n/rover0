#include "hardware_interface/system_interface.hpp"

#include "rover0_controller/rover0_controller.hpp"

rover0_controller::Rover0Controller::Rover0Controller()
    : controller_interface::ControllerInterface(),
      has_new_msg_{false}
{
}

controller_interface::return_type rover0_controller::Rover0Controller::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    if (has_new_msg_)
    {
        cmd_vel_twist_ = *cmd_vel_twist_ptr_.readFromRT();
        has_new_msg_ = false;
    }

    if (cmd_vel_twist_ == nullptr)
    {
        // no command received yet
        return controller_interface::return_type::OK;
    }

    double const r = (config.wheel_radius + config.track_width) / 2;

    double front_left_wheel_velocity  = (cmd_vel_twist_->linear.x - cmd_vel_twist_->linear.y - r * cmd_vel_twist_->angular.z)/ config.wheel_radius;
    double front_right_wheel_velocity = (cmd_vel_twist_->linear.x + cmd_vel_twist_->linear.y + r * cmd_vel_twist_->angular.z)/ config.wheel_radius;
    double rear_left_wheel_velocity   = (cmd_vel_twist_->linear.x + cmd_vel_twist_->linear.y - r * cmd_vel_twist_->angular.z)/ config.wheel_radius;
    double rear_right_wheel_velocity  = (cmd_vel_twist_->linear.x - cmd_vel_twist_->linear.y + r * cmd_vel_twist_->angular.z)/ config.wheel_radius;

    command_interface_map_.at(config.front_left_wheel_joint_name).get().set_value(front_left_wheel_velocity);
    command_interface_map_.at(config.front_right_wheel_joint_name).get().set_value(front_right_wheel_velocity);
    command_interface_map_.at(config.rear_left_wheel_joint_name).get().set_value(rear_left_wheel_velocity);
    command_interface_map_.at(config.rear_right_wheel_joint_name).get().set_value(rear_right_wheel_velocity);
    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn rover0_controller::Rover0Controller::on_init()
{
    config.front_left_wheel_joint_name = auto_declare<std::string>("front_left_wheel_joint_name", config.front_left_wheel_joint_name);
    config.front_right_wheel_joint_name = auto_declare<std::string>("front_right_wheel_joint_name", config.front_right_wheel_joint_name);
    config.rear_left_wheel_joint_name = auto_declare<std::string>("rear_left_wheel_joint_name", config.rear_left_wheel_joint_name);
    config.rear_right_wheel_joint_name = auto_declare<std::string>("rear_right_wheel_joint_name", config.rear_right_wheel_joint_name);
    config.wheel_radius = auto_declare<double>("wheel_radius", config.wheel_radius);
    config.wheelbase = auto_declare<double>("wheelbase", config.wheelbase);
    config.track_width = auto_declare<double>("track_width", config.track_width);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rover0_controller::Rover0Controller::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            cmd_vel_twist_ptr_.writeFromNonRT(msg);
            has_new_msg_ = true;
        });

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rover0_controller::Rover0Controller::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    for (auto &interface : command_interfaces_)
    {
        command_interface_map_.emplace(interface.get_prefix_name(), interface);
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rover0_controller::Rover0Controller::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rover0_controller::Rover0Controller::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rover0_controller::Rover0Controller::on_error(const rclcpp_lifecycle::State &previous_state)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rover0_controller::Rover0Controller::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration rover0_controller::Rover0Controller::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config{
        controller_interface::interface_configuration_type::INDIVIDUAL,
        {
            config.front_left_wheel_joint_name + "/" + hardware_interface::HW_IF_VELOCITY,
            config.front_right_wheel_joint_name + "/" + hardware_interface::HW_IF_VELOCITY,
            config.rear_left_wheel_joint_name + "/" + hardware_interface::HW_IF_VELOCITY,
            config.rear_right_wheel_joint_name + "/" + hardware_interface::HW_IF_VELOCITY,
        }};

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration rover0_controller::Rover0Controller::state_interface_configuration() const
{
    return controller_interface::InterfaceConfiguration{};
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rover0_controller::Rover0Controller, controller_interface::ControllerInterface)
