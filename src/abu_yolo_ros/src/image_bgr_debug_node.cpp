#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

// Debug utility node:
// converts the raw /image_raw feed into a BGR debug topic for inspection in
// developer tools. It is not part of the main production perception pipeline.
class ImageBGRDebugNode : public rclcpp::Node {
public:
    ImageBGRDebugNode() : Node("image_bgr_debug_node") {
        auto qos = rclcpp::SensorDataQoS();

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            qos,
            std::bind(&ImageBGRDebugNode::imageCallback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/yolo/image_bgr_debug",
            qos
        );

        RCLCPP_INFO(this->get_logger(), "Subscribed to /image_raw");
        RCLCPP_INFO(this->get_logger(), "Publishing BGR image to /yolo/image_bgr_debug");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            int width = msg->width;
            int height = msg->height;

            // Tạo cv::Mat từ raw YUY2 buffer
            cv::Mat yuy2(
                height,
                width,
                CV_8UC2,
                const_cast<unsigned char*>(msg->data.data())
            );

            // Convert sang BGR
            cv::Mat bgr;
            cv::cvtColor(yuy2, bgr, cv::COLOR_YUV2BGR_YUY2);

            // Publish ảnh BGR
            auto out_msg = cv_bridge::CvImage(
                msg->header,
                "bgr8",
                bgr
            ).toImageMsg();

            pub_->publish(*out_msg);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Conversion failed: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageBGRDebugNode>());
    rclcpp::shutdown();
    return 0;
}
