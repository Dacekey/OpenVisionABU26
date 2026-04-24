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
        this->declare_parameter<bool>("debug_detections", true);
        this->declare_parameter<bool>("enable_team_color_filter", true);
        this->declare_parameter<std::string>("team_color", "red");

        // TeamColorFilter parameters
        this->declare_parameter<int>("red_h_low_1", 0);
        this->declare_parameter<int>("red_h_high_1", 12);
        this->declare_parameter<int>("red_h_low_2", 168);
        this->declare_parameter<int>("red_h_high_2", 180);
        this->declare_parameter<int>("red_s_low", 130);
        this->declare_parameter<int>("red_v_low", 80);

        this->declare_parameter<int>("blue_h_low", 100);
        this->declare_parameter<int>("blue_h_high", 130);
        this->declare_parameter<int>("blue_s_low", 150);
        this->declare_parameter<int>("blue_v_low", 80);

        this->declare_parameter<double>("min_coverage_ratio", 0.15);
        this->declare_parameter<double>("confidence_scale", 3.0);
        this->declare_parameter<double>("min_match_confidence", 0.30);

        this->get_parameter("model_path", model_path_);
        this->get_parameter("class_names_path", class_names_path_);
        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("use_gpu", use_gpu_);
        this->get_parameter("visualize", visualize_);
        this->get_parameter("log_timing", log_timing_);
        this->get_parameter("skip_frames", skip_frames_);
        this->get_parameter("debug_detections", debug_detections_);
        this->get_parameter(
            "enable_team_color_filter",
            enable_team_color_filter_);
        this->get_parameter("team_color", team_color_string_);

        // Read TeamColorFilter config
        this->get_parameter("red_h_low_1", tcf_config_.red_h_low_1);
        this->get_parameter("red_h_high_1", tcf_config_.red_h_high_1);
        this->get_parameter("red_h_low_2", tcf_config_.red_h_low_2);
        this->get_parameter("red_h_high_2", tcf_config_.red_h_high_2);
        this->get_parameter("red_s_low", tcf_config_.red_s_low);
        this->get_parameter("red_v_low", tcf_config_.red_v_low);

        this->get_parameter("blue_h_low", tcf_config_.blue_h_low);
        this->get_parameter("blue_h_high", tcf_config_.blue_h_high);
        this->get_parameter("blue_s_low", tcf_config_.blue_s_low);
        this->get_parameter("blue_v_low", tcf_config_.blue_v_low);

        this->get_parameter("min_coverage_ratio", tcf_config_.min_coverage_ratio);
        this->get_parameter("confidence_scale", tcf_config_.confidence_scale);
        this->get_parameter("min_match_confidence", tcf_config_.min_match_confidence);

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

        if (enable_team_color_filter_) {
            RCLCPP_INFO(this->get_logger(), "TeamColorFilter Config:");
            RCLCPP_INFO(this->get_logger(), "  Red H1: [%d, %d], H2: [%d, %d], S_low: %d, V_low: %d",
                        tcf_config_.red_h_low_1, tcf_config_.red_h_high_1,
                        tcf_config_.red_h_low_2, tcf_config_.red_h_high_2,
                        tcf_config_.red_s_low, tcf_config_.red_v_low);
            RCLCPP_INFO(this->get_logger(), "  Blue H: [%d, %d], S_low: %d, V_low: %d",
                        tcf_config_.blue_h_low, tcf_config_.blue_h_high,
                        tcf_config_.blue_s_low, tcf_config_.blue_v_low);
            RCLCPP_INFO(this->get_logger(), "  Min Coverage: %.2f, Conf Scale: %.2f, Min Match Conf: %.2f",
                        tcf_config_.min_coverage_ratio, tcf_config_.confidence_scale,
                        tcf_config_.min_match_confidence);
        }

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

            const auto team_color_results =
                evaluateTeamColorResults(bgr, detections);
            maybeLogTeamColorResults(
                detections,
                team_color_results);
            maybeLogDetectionDetails(
                detections,
                team_color_results);
            
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

    std::vector<abu_yolo_ros::TeamColorResult> evaluateTeamColorResults(
        const cv::Mat& bgr,
        const std::vector<abu_yolo_ros::Detection>& detections) const
    {
        std::vector<abu_yolo_ros::TeamColorResult> results;

        if (!enable_team_color_filter_) {
            return results;
        }

        results.reserve(detections.size());
        for (const auto& detection : detections) {
            results.push_back(
                abu_yolo_ros::filterByTeamColor(
                    bgr,
                    detection,
                    my_team_,
                    tcf_config_));
        }

        return results;
    }

    void maybeLogTeamColorResults(
        const std::vector<abu_yolo_ros::Detection>& detections,
        const std::vector<abu_yolo_ros::TeamColorResult>& team_color_results)
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
            const auto& result = team_color_results[i];

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
                        << " hsv=(" << result.mean_h
                        << "," << result.mean_s
                        << "," << result.mean_v
                        << ")"
                        << " red=" << result.red_coverage
                        << " blue=" << result.blue_coverage
                        << " dominant=" << result.dominant_coverage
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

    void maybeLogDetectionDetails(
        const std::vector<abu_yolo_ros::Detection>& detections,
        const std::vector<abu_yolo_ros::TeamColorResult>& team_color_results)
    {
        if (!debug_detections_) {
            return;
        }

        std::ostringstream stream;
        stream << "Detection details count=" << detections.size();

        for (std::size_t i = 0; i < detections.size(); ++i) {
            const auto& detection = detections[i];

            stream << "\n  [" << i << "]"
                   << " class_id=" << detection.class_id
                   << " conf=" << std::fixed << std::setprecision(2)
                   << detection.confidence
                   << " bbox(x=" << detection.x
                   << ", y=" << detection.y
                   << ", w=" << detection.w
                   << ", h=" << detection.h
                   << ")";

            if (enable_team_color_filter_ &&
                i < team_color_results.size()) {
                const auto& result = team_color_results[i];
                stream << " team=" << abu_yolo_ros::teamColorToString(
                                        result.detected_team)
                       << " match="
                       << (result.matches_team ? "true" : "false")
                       << " mean_h=" << result.mean_h
                       << " mean_s=" << result.mean_s
                       << " mean_v=" << result.mean_v
                       << " red=" << result.red_coverage
                       << " blue=" << result.blue_coverage
                       << " dominant=" << result.dominant_coverage
                       << " color_conf=" << result.confidence;
            }
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "%s",
            stream.str().c_str());
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
    bool debug_detections_;
    bool enable_team_color_filter_;

    int skip_frames_;
    uint64_t frame_count_;
    std::string team_color_string_;
    abu_yolo_ros::TeamColor my_team_;
    abu_yolo_ros::TeamColorFilterConfig tcf_config_;
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<YoloDetectionNode>());

    rclcpp::shutdown();

    return 0;
}
