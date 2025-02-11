#include <iostream>

#include "hardware_interface/system_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "rover0_controller/rover0_controller.hpp"

rover0_controller::Rover0Controller::Rover0Controller()
    : controller_interface::ControllerInterface(),
      mecanum_kinematics_{std::nullopt},
      wheel_odometry_{std::nullopt},
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

    if (!mecanum_kinematics_) {
        return controller_interface::return_type::ERROR;
    }

    if (!wheel_odometry_) {
        return controller_interface::return_type::ERROR;
    }

    auto const wheel_angular_velocity = mecanum_kinematics_->inverse(
        rover0_controller::mecanum_kinematics::Twist {
            cmd_vel_twist_->linear.x ,
            cmd_vel_twist_->linear.y,
            cmd_vel_twist_->angular.z
        }
    );

    double const dt = period.seconds();
    wheel_odometry_->update(
        wheel_angular_velocity.front_left,
        wheel_angular_velocity.front_right,
        wheel_angular_velocity.rear_left,
        wheel_angular_velocity.rear_right,
        dt
    );

    (void)command_interface_map_.at(config.front_left_wheel_joint_name).get().set_value(wheel_angular_velocity.front_left);
    (void)command_interface_map_.at(config.front_right_wheel_joint_name).get().set_value(wheel_angular_velocity.front_right);
    (void)command_interface_map_.at(config.rear_left_wheel_joint_name).get().set_value(wheel_angular_velocity.rear_left);
    (void)command_interface_map_.at(config.rear_right_wheel_joint_name).get().set_value(wheel_angular_velocity.rear_right);

    if( should_publish_odom(time) && rt_odometry_pub_->trylock() ) {
        last_odom_publish_time_ = time;
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, wheel_odometry_->getHeading());

        auto & odometry_msg = rt_odometry_pub_->msg_;
        odometry_msg.header.stamp = time;
        odometry_msg.pose.pose.position.x = wheel_odometry_->getX();
        odometry_msg.pose.pose.position.y = wheel_odometry_->getY();
        odometry_msg.pose.pose.orientation.x = orientation.x();
        odometry_msg.pose.pose.orientation.y = orientation.y();
        odometry_msg.pose.pose.orientation.z = orientation.z();
        odometry_msg.pose.pose.orientation.w = orientation.w();
        odometry_msg.twist.twist.linear.x = wheel_odometry_->getLinearX();
        odometry_msg.twist.twist.linear.y = wheel_odometry_->getLinearY();
        odometry_msg.twist.twist.angular.z = wheel_odometry_->getAngular();
        rt_odometry_pub_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn rover0_controller::Rover0Controller::on_init()
{
    config.front_left_wheel_joint_name = auto_declare<std::string>("front_left_wheel_joint_name", config.front_left_wheel_joint_name);
    config.front_right_wheel_joint_name = auto_declare<std::string>("front_right_wheel_joint_name", config.front_right_wheel_joint_name);
    config.rear_left_wheel_joint_name = auto_declare<std::string>("rear_left_wheel_joint_name", config.rear_left_wheel_joint_name);
    config.rear_right_wheel_joint_name = auto_declare<std::string>("rear_right_wheel_joint_name", config.rear_right_wheel_joint_name);
    config.wheel_radius = auto_declare<double>("wheel_radius", config.wheel_radius);
    config.wheel_base = auto_declare<double>("wheelbase", config.wheel_base);
    config.track_width = auto_declare<double>("track_width", config.track_width);
    config.odom_topic = auto_declare<std::string>("odom_topic", config.odom_topic);
    config.odom_publish_rate = auto_declare<double>("odom_publish_rate", config.odom_publish_rate);
    config.odom_frame_id = auto_declare<std::string>("odom_frame_id", config.odom_frame_id);
    config.base_frame_id = auto_declare<std::string>("base_frame_id", config.base_frame_id);

    mecanum_kinematics_.emplace(config.wheel_radius, config.wheel_base, config.track_width);
    wheel_odometry_.emplace(config.wheel_radius, config.wheel_base, config.track_width, 10);

    last_odom_publish_time_ = get_node()->now();

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rover0_controller::Rover0Controller::on_configure(const rclcpp_lifecycle::State &)
{
    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            cmd_vel_twist_ptr_.writeFromNonRT(msg);
            has_new_msg_ = true;
        });

    auto odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
      config.odom_topic, rclcpp::SystemDefaultsQoS());
    rt_odometry_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
        odometry_publisher_);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rover0_controller::Rover0Controller::on_activate(const rclcpp_lifecycle::State &)
{
    for (auto &interface : command_interfaces_)
    {
        command_interface_map_.emplace(interface.get_prefix_name(), interface);
    }

    for (auto &interface : state_interfaces_)
    {
        state_interface_map_.emplace(interface.get_prefix_name(), interface);
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rover0_controller::Rover0Controller::on_deactivate(const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rover0_controller::Rover0Controller::on_cleanup(const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rover0_controller::Rover0Controller::on_error(const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn rover0_controller::Rover0Controller::on_shutdown(const rclcpp_lifecycle::State &)
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
        }
    };

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration rover0_controller::Rover0Controller::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config{
        controller_interface::interface_configuration_type::INDIVIDUAL,
        {
            config.front_left_wheel_joint_name + "/" + hardware_interface::HW_IF_VELOCITY,
            config.front_right_wheel_joint_name + "/" + hardware_interface::HW_IF_VELOCITY,
            config.rear_left_wheel_joint_name + "/" + hardware_interface::HW_IF_VELOCITY,
            config.rear_right_wheel_joint_name + "/" + hardware_interface::HW_IF_VELOCITY,
        }
    };

    return state_interfaces_config;
}

bool rover0_controller::Rover0Controller::should_publish_odom(rclcpp::Time const &time) const
{
    rclcpp::Duration const time_since_last_odom_publish = time - last_odom_publish_time_;
    return (1.0 < time_since_last_odom_publish.seconds() * config.odom_publish_rate);
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rover0_controller::Rover0Controller, controller_interface::ControllerInterface)
