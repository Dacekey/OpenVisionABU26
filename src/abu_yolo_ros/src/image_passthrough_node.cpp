#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImagePassthroughNode : public rclcpp::Node {
public:
    ImagePassthroughNode() : Node("image_passthrough_node") {
        auto qos = rclcpp::SensorDataQoS();

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            qos,
            std::bind(&ImagePassthroughNode::imageCallback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/yolo/image_debug",
            qos
        );

        RCLCPP_INFO(this->get_logger(), "Subscribed to /image_raw");
        RCLCPP_INFO(this->get_logger(), "Publishing to /yolo/image_debug");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        pub_->publish(*msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePassthroughNode>());
    rclcpp::shutdown();
    return 0;
}