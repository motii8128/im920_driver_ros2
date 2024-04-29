#include "im920_driver_ros2.hpp"

namespace im920_driver_ros2
{
    IM920DriverROS2::IM920DriverROS2(const rclcpp::NodeOptions& options):rclcpp::Node("im920_driver_ros2_node", options)
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/im920/write",
            0,
            std::bind(&im920_driver_ros2::IM920DriverROS2::topic_callback, this, _1));
        
        publisher_ = this->create_publisher<std_msgs::msg::String>("/im920/read", 0);

        write_timer = this->create_wall_timer(
            50ms,
            std::bind(&im920_driver_ros2::IM920DriverROS2::write_callback, this));
        read_timer_ = this->create_wall_timer(
            50ms,
            std::bind(&im920_driver_ros2::IM920DriverROS2::read_callback, this));

        this->declare_parameter("port_name", "/dev/ttyACM0");
        this->get_parameter("port_name", port_name_);
        
        this->declare_parameter("baud_rate", 115200);
        this->get_parameter("baud_rate", baud_rate_);

        this->declare_parameter("enable_write_log", false);
        this->get_parameter("enable_write_log", enable_write_log_);

        this->declare_parameter("enable_read_log", false);
        this->get_parameter("enable_read_log", enable_read_log_);

        this->declare_parameter("child_id", 2);
        this->get_parameter("child_id", child_id_);

        if(child_id_ > 9999)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed child id");
        }

        im920_serial_ = std::shared_ptr<im920_driver_ros2::IM920Serial>(im920_driver_ros2::IM920Serial::init_im920_serial(port_name_, baud_rate_));

        RCLCPP_INFO(this->get_logger(), "Open serial port");
        int err = im920_serial_->open_port();
        if(err < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port");
            im920_serial_->close_port();
        }

        RCLCPP_INFO(this->get_logger(), "Start IM920DriverROS2 port:%s, baud_rate:%d", im920_serial_->get_port_name().c_str(), im920_serial_->get_baud_rate());
    }

    void IM920DriverROS2::topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        tx_packet_ = "TXDU" + im920_serial_->id_to_string(child_id_) + msg->data;
    }

    void IM920DriverROS2::write_callback()
    {
        int err = im920_serial_->write_serial(tx_packet_);
        if(err < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write");
        }

        if(enable_write_log_)
        {
            RCLCPP_INFO(this->get_logger(), "Write:%s", tx_packet_.c_str());
        }
    }

    void IM920DriverROS2::read_callback()
    {
        std::string rx_packet;

        int err = im920_serial_->read_serial(&rx_packet);
        if(err < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read");
        }
        auto msg = std_msgs::msg::String();
        msg.data = rx_packet;
        publisher_->publish(msg);

        if(enable_read_log_)
        {
            RCLCPP_INFO(this->get_logger(), "Read:%s", rx_packet.c_str());
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(im920_driver_ros2::IM920DriverROS2)