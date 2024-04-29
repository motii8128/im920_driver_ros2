#ifndef IM920_DRIVER_ROS2_HPP_
#define IM920_DRIVER_ROS2_HPP_

#include "im920_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace im920_driver_ros2
{
    class IM920DriverROS2 : public rclcpp::Node
    {
        public:
        explicit IM920DriverROS2(const rclcpp::NodeOptions& options=rclcpp::NodeOptions());

        void topic_callback(const std_msgs::msg::String::SharedPtr msg);
        void write_callback();
        void read_callback();

        private:
        std::string port_name_, tx_packet_;
        int baud_rate_, child_id_;
        bool enable_write_log_, enable_read_log_;
        std::shared_ptr<im920_driver_ros2::IM920Serial> im920_serial_;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr read_timer_, write_timer;
    };
}
#endif