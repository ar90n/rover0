#pragma once

#include <optional>

#include <geometry_msgs/msg/twist.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "nav_msgs/msg/odometry.hpp"

#include <rclcpp/subscription.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <controller_interface/controller_interface.hpp>

#include "rover0_controller/mecanum_kinematics.hpp"
#include "wheel_odometry.hpp"

namespace rover0_controller
{
    class Rover0Controller : public controller_interface::ControllerInterface
    {

        struct Config
        {
            std::string front_left_wheel_joint_name{"front_left_wheel_joint"};
            std::string front_right_wheel_joint_name{"front_right_wheel_joint"};
            std::string rear_left_wheel_joint_name{"rear_left_wheel_joint"};
            std::string rear_right_wheel_joint_name{"rear_right_wheel_joint"};
            double wheel_radius{0.028};
            double wheel_base{0.14};
            double track_width{0.11};
            std::string odom_topic{"/wheel_odom"};
            double odom_publish_rate{10.0};
            std::string odom_frame_id{"odom"};
            std::string base_frame_id{"base_link"};
        };

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Rover0Controller)

        CONTROLLER_INTERFACE_PUBLIC
        Rover0Controller();
        virtual ~Rover0Controller() = default;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_init() override;
        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    private:
        Config config;
        std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interface_map_{};
        std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interface_map_{};

        std::optional<mecanum_kinematics::MecanumKinematics> mecanum_kinematics_;
        std::optional<WheelOdometry> wheel_odometry_;

        bool has_new_msg_{false};
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> cmd_vel_twist_ptr_;
        std::shared_ptr<geometry_msgs::msg::Twist> cmd_vel_twist_;

        rclcpp::Time last_odom_publish_time_{0};
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
          rt_odometry_pub_{nullptr};

        bool should_publish_odom(rclcpp::Time const &time) const;
    };
}
