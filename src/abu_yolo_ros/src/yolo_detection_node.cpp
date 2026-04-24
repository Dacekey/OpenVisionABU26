#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "abu_yolo_ros/team_color_filter.hpp"
#include "abu_yolo_ros/yolo_detector.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

class YoloDetectionNode : public rclcpp::Node {
public:
    YoloDetectionNode()
        : Node("yolo_detection_node"),
          frame_count_(0) {

        auto qos = rclcpp::SensorDataQoS();

        // Parameters
        this->declare_parameter<std::string>("model_path", "");
        this->declare_parameter<std::string>("class_names_path", "");
        this->declare_parameter<std::string>("input_topic", "/image_raw");
        this->declare_parameter<std::string>("output_topic", "/yolo/image_annotated");
        this->declare_parameter<bool>("use_gpu", true);
        this->declare_parameter<bool>("visualize", true);
        this->declare_parameter<bool>("log_timing", false);
        this->declare_parameter<int>("skip_frames", 0);
        this->declare_parameter<bool>("enable_team_color_filter", true);
        this->declare_parameter<std::string>("team_color", "red");

        this->get_parameter("model_path", model_path_);
        this->get_parameter("class_names_path", class_names_path_);
        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("use_gpu", use_gpu_);
        this->get_parameter("visualize", visualize_);
        this->get_parameter("log_timing", log_timing_);
        this->get_parameter("skip_frames", skip_frames_);
        this->get_parameter(
            "enable_team_color_filter",
            enable_team_color_filter_);
        this->get_parameter("team_color", team_color_string_);

        if (model_path_.empty())
            throw std::runtime_error("model_path is empty");

        if (class_names_path_.empty())
            throw std::runtime_error("class_names_path is empty");

        if (skip_frames_ < 0)
            skip_frames_ = 0;

        my_team_ =
            abu_yolo_ros::parseTeamColor(team_color_string_);
        if (my_team_ == abu_yolo_ros::TeamColor::UNKNOWN) {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid team_color '%s'; defaulting to red",
                team_color_string_.c_str());
            my_team_ = abu_yolo_ros::TeamColor::RED;
            team_color_string_ = "red";
        }

        RCLCPP_INFO(this->get_logger(), "Loading YOLO model...");
        RCLCPP_INFO(this->get_logger(), "Using GPU: %s",
                    use_gpu_ ? "true" : "false");
        RCLCPP_INFO(
            this->get_logger(),
            "Team color filter: %s | team=%s",
            enable_team_color_filter_ ? "enabled" : "disabled",
            abu_yolo_ros::teamColorToString(my_team_).c_str());

        detector_ = std::make_unique<abu_yolo_ros::YOLODetector>(
            model_path_,
            class_names_path_,
            use_gpu_);

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_,
            qos,
            std::bind(
                &YoloDetectionNode::imageCallback,
                this,
                std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            output_topic_,
            qos);
        
        detection_pub_ =
            this->create_publisher<
                vision_msgs::msg::Detection2DArray
            >(
                "/yolo/detections",
                qos
            );

        RCLCPP_INFO(this->get_logger(), "YOLO Detection Node started");
    }

private:

    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg) {

        frame_count_++;

        // Skip frame logic
        if (skip_frames_ > 0) {
            int period = skip_frames_ + 1;
            if ((frame_count_ % period) != 1)
                return;
        }

        try {
            auto t0 = std::chrono::steady_clock::now();

            int width = msg->width;
            int height = msg->height;

            // YUY2 -> cv::Mat
            cv::Mat yuy2(
                height,
                width,
                CV_8UC2,
                const_cast<unsigned char*>(msg->data.data()));

            // Convert to BGR
            cv::Mat bgr;
            cv::cvtColor(
                yuy2,
                bgr,
                cv::COLOR_YUV2BGR_YUY2);

            auto t1 = std::chrono::steady_clock::now();

            // Inference
            auto detections =
                detector_->infer(bgr);

            maybeLogTeamColorResults(bgr, detections);
            
            publishDetections(detections);

            auto t2 = std::chrono::steady_clock::now();
            
            // auto duration =
            //     std::chrono::duration_cast<
            //         std::chrono::milliseconds
            //     >(t2 - t1).count();
            
            // 
            // RCLCPP_INFO(
            //     this->get_logger(),
            //     "Inference time: %ld ms",
            //     duration
            // );

            // Visualization
            cv::Mat output;

            if (visualize_) {
                output =
                    detector_->drawDetections(
                        bgr,
                        detections);
            }
            else {
                output = bgr;
            }

            // auto t3 = std::chrono::steady_clock::now();

            auto out_msg =
                cv_bridge::CvImage(
                    msg->header,
                    "bgr8",
                    output
                ).toImageMsg();

            pub_->publish(*out_msg);

            auto t4 = std::chrono::steady_clock::now();

            // Timing log (THROTTLED)
            if (log_timing_) {

                // double convert_ms =
                //     std::chrono::duration<double, std::milli>(
                //         t1 - t0).count();

                double infer_ms =
                    std::chrono::duration<double, std::milli>(
                        t2 - t1).count();

                // double draw_ms =
                //     std::chrono::duration<double, std::milli>(
                //         t3 - t2).count();

                // double publish_ms =
                //     std::chrono::duration<double, std::milli>(
                //         t4 - t3).count();

                double total_ms =
                    std::chrono::duration<double, std::milli>(
                        t4 - t0).count();

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Det=%zu | infer=%.2f ms | total=%.2f ms",
                    detections.size(),
                    infer_ms,
                    total_ms
                );
            }

        }
        catch (const std::exception& e) {

            RCLCPP_ERROR(
                this->get_logger(),
                "Inference error: %s",
                e.what());
        }
    }

    void publishDetections(
        const std::vector<abu_yolo_ros::Detection>& detections)
    {
        vision_msgs::msg::Detection2DArray msg;

        msg.header.stamp = now();
        msg.header.frame_id = "camera";

        for (const auto& det : detections)
        {
            vision_msgs::msg::Detection2D detection;

            detection.bbox.center.position.x =
                det.x + det.w / 2.0;

            detection.bbox.center.position.y =
                det.y + det.h / 2.0;
            
            detection.bbox.center.theta = 0.0;

            detection.bbox.size_x =
                det.w;

            detection.bbox.size_y =
                det.h;

            vision_msgs::msg::ObjectHypothesisWithPose hyp;

            hyp.hypothesis.class_id =
                std::to_string(det.class_id);

            hyp.hypothesis.score =
                det.confidence;

            detection.results.push_back(hyp);

            msg.detections.push_back(detection);
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Published %ld detections",
            msg.detections.size()
        );

        detection_pub_->publish(msg);
    }

    void maybeLogTeamColorResults(
        const cv::Mat& bgr,
        const std::vector<abu_yolo_ros::Detection>& detections)
    {
        if (!enable_team_color_filter_) {
            return;
        }

        std::size_t match_count = 0;
        std::size_t red_count = 0;
        std::size_t blue_count = 0;
        std::size_t unknown_count = 0;

        std::ostringstream details;
        bool has_details = false;

        for (std::size_t i = 0; i < detections.size(); ++i) {
            const auto result =
                abu_yolo_ros::filterByTeamColor(
                    bgr,
                    detections[i],
                    my_team_);

            if (result.matches_team) {
                ++match_count;
            }

            switch (result.detected_team) {
            case abu_yolo_ros::TeamColor::RED:
                ++red_count;
                break;
            case abu_yolo_ros::TeamColor::BLUE:
                ++blue_count;
                break;
            case abu_yolo_ros::TeamColor::UNKNOWN:
            default:
                ++unknown_count;
                break;
            }

            if (result.confidence >= 0.30f && has_details == false) {
                details << "det[" << i << "]="
                        << abu_yolo_ros::teamColorToString(
                               result.detected_team)
                        << " conf=" << std::fixed << std::setprecision(2)
                        << result.confidence
                        << " red=" << result.red_coverage
                        << " blue=" << result.blue_coverage
                        << " match="
                        << (result.matches_team ? "true" : "false");
                has_details = true;
            }
        }

        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "TeamColorFilter team=%s det=%zu match=%zu red=%zu blue=%zu unknown=%zu %s",
            abu_yolo_ros::teamColorToString(my_team_).c_str(),
            detections.size(),
            match_count,
            red_count,
            blue_count,
            unknown_count,
            has_details ? details.str().c_str() : "");
    }

private:

    std::unique_ptr<abu_yolo_ros::YOLODetector> detector_;

    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr sub_;

    rclcpp::Publisher<
        sensor_msgs::msg::Image>::SharedPtr pub_;

    rclcpp::Publisher<
        vision_msgs::msg::Detection2DArray
    >::SharedPtr detection_pub_;

    std::string model_path_;
    std::string class_names_path_;
    std::string input_topic_;
    std::string output_topic_;

    bool use_gpu_;
    bool visualize_;
    bool log_timing_;
    bool enable_team_color_filter_;

    int skip_frames_;
    uint64_t frame_count_;
    std::string team_color_string_;
    abu_yolo_ros::TeamColor my_team_;
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<YoloDetectionNode>());

    rclcpp::shutdown();

    return 0;
}
