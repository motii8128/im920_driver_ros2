#ifndef IM920_DRIVER_ROS2_HPP_
#define IM920_DRIVER_ROS2_HPP_

#include "im920_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

namespace im920_driver_ros2
{
    class IM920DriverROS2 : public rclcpp::Node
    {
        public:
        explicit IM920DriverROS2(const rclcpp::NodeOptions& options=rclcpp::NodeOptions());

        void write_callback(const std_msgs::msg::String::SharedPtr msg);
        void read_callback();

        private:
        std::string port_name_;
        int baud_rate;
        bool enable_write_log_, enable_read_log_;
        std::shared_ptr<im920_driver_ros2::IM920Serial> im920_serial_;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}
#endif