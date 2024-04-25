#ifndef IM920_DRIVER_ROS2_HPP_
#define IM920_DRIVER_ROS2_HPP_

#include "im920_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace im920_driver_ros2
{
    class IM920Driver : public rclcpp::Node
    {
        public:
        explicit IM920Driver(const rclcpp::NodeOptions& options=rclcpp::NodeOptions());

        void write_callback(const std_msgs::msg::String::SharedPtr msg);
        void read_callback();

        
    };
}
#endif