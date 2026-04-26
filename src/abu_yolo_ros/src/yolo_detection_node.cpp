#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "abu_yolo_ros/decision_engine.hpp"
#include "abu_yolo_ros/kfs_instance_aggregator.hpp"
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

        // DecisionEngine parameters
        this->declare_parameter<double>("r1_conf_threshold", 0.55);
        this->declare_parameter<double>("real_conf_threshold", 0.60);
        this->declare_parameter<double>("fake_conf_threshold", 0.45);
        this->declare_parameter<double>("collect_min_confidence", 0.60);
        this->declare_parameter<double>("yolo_confidence_weight", 0.60);
        this->declare_parameter<double>("color_confidence_weight", 0.40);
        this->declare_parameter<bool>("require_team_color_match", true);
        this->declare_parameter<bool>("unknown_on_low_confidence", true);

        // KFSInstanceAggregator parameters
        this->declare_parameter<bool>("kfs_instance_aggregation.enabled", true);
        this->declare_parameter<bool>("kfs_instance_aggregation.debug_instances", true);
        this->declare_parameter<bool>("kfs_instance_aggregation.publish_debug_image", false);
        this->declare_parameter<std::string>(
            "kfs_instance_aggregation.debug_image_topic",
            "/yolo/kfs_instances/image_annotated");
        this->declare_parameter<bool>("kfs_instance_aggregation.draw_roi", true);
        this->declare_parameter<std::vector<int64_t>>(
            "kfs_instance_aggregation.roi_color_bgr",
            std::vector<int64_t>{0, 255, 255});
        this->declare_parameter<int>("kfs_instance_aggregation.roi_thickness", 2);
        this->declare_parameter<bool>("kfs_instance_aggregation.enable_roi_filter", true);
        this->declare_parameter<double>("kfs_instance_aggregation.roi_x_min_norm", 0.10);
        this->declare_parameter<double>("kfs_instance_aggregation.roi_x_max_norm", 0.78);
        this->declare_parameter<double>("kfs_instance_aggregation.roi_y_min_norm", 0.22);
        this->declare_parameter<double>("kfs_instance_aggregation.roi_y_max_norm", 1.00);
        this->declare_parameter<double>("kfs_instance_aggregation.min_symbol_height_px", 60.0);
        this->declare_parameter<double>("kfs_instance_aggregation.min_symbol_area_px", 4000.0);
        this->declare_parameter<double>("kfs_instance_aggregation.min_confidence", 0.25);
        this->declare_parameter<bool>("kfs_instance_aggregation.use_square_symbol_bbox", true);
        this->declare_parameter<double>("kfs_instance_aggregation.square_symbol_scale", 1.0);
        this->declare_parameter<double>("kfs_instance_aggregation.max_square_side_ratio_of_image", 0.35);
        this->declare_parameter<double>("kfs_instance_aggregation.expand_scale_x", 1.8);
        this->declare_parameter<double>("kfs_instance_aggregation.expand_scale_y", 1.8);
        this->declare_parameter<double>("kfs_instance_aggregation.max_expanded_area_ratio", 0.25);
        this->declare_parameter<double>("kfs_instance_aggregation.cluster_distance_scale", 2.5);
        this->declare_parameter<double>("kfs_instance_aggregation.min_cluster_distance_px", 35.0);
        this->declare_parameter<double>("kfs_instance_aggregation.max_cluster_distance_px", 180.0);
        this->declare_parameter<double>("kfs_instance_aggregation.min_height_similarity_for_grouping", 0.60);
        this->declare_parameter<double>("kfs_instance_aggregation.min_area_similarity_for_grouping", 0.30);
        this->declare_parameter<double>("kfs_instance_aggregation.max_bottom_y_diff_ratio", 1.20);
        this->declare_parameter<int>("kfs_instance_aggregation.max_symbols_per_instance", 3);
        this->declare_parameter<bool>("kfs_instance_aggregation.enable_basic_merge", true);
        this->declare_parameter<bool>("kfs_instance_aggregation.merge_same_group_only", true);
        this->declare_parameter<int>("kfs_instance_aggregation.max_symbols_after_merge", 3);
        this->declare_parameter<double>("kfs_instance_aggregation.min_expanded_iou", 0.03);
        this->declare_parameter<double>("kfs_instance_aggregation.min_expanded_intersection_over_min_area", 0.12);
        this->declare_parameter<double>("kfs_instance_aggregation.max_expanded_gap_px", 40.0);
        this->declare_parameter<double>("kfs_instance_aggregation.min_color_mask_coverage_for_merge", 0.15);
        this->declare_parameter<double>("kfs_instance_aggregation.max_merged_union_width_scale", 2.2);
        this->declare_parameter<double>("kfs_instance_aggregation.max_merged_union_height_scale", 2.6);
        this->declare_parameter<bool>("kfs_instance_aggregation.enable_low_mask_adjacent_same_group_fallback", true);
        this->declare_parameter<double>("kfs_instance_aggregation.low_mask_min_height_similarity", 0.55);
        this->declare_parameter<double>("kfs_instance_aggregation.low_mask_max_bottom_y_diff_ratio", 1.35);
        this->declare_parameter<double>("kfs_instance_aggregation.low_mask_max_gap_px", 45.0);
        this->declare_parameter<double>("kfs_instance_aggregation.low_mask_max_gap_ratio_to_symbol_height", 0.55);
        this->declare_parameter<double>("kfs_instance_aggregation.low_mask_max_merged_aspect_ratio", 2.2);
        this->declare_parameter<double>("kfs_instance_aggregation.low_mask_max_merged_width_scale", 2.2);
        this->declare_parameter<double>("kfs_instance_aggregation.low_mask_max_merged_height_scale", 2.6);
        this->declare_parameter<bool>("kfs_instance_aggregation.drop_ambiguous_clusters", true);
        this->declare_parameter<std::string>("kfs_instance_aggregation.hsv_profile", "competition_blue");
        this->declare_parameter<int>("kfs_instance_aggregation.red_h_low_1", 0);
        this->declare_parameter<int>("kfs_instance_aggregation.red_h_high_1", 12);
        this->declare_parameter<int>("kfs_instance_aggregation.red_h_low_2", 168);
        this->declare_parameter<int>("kfs_instance_aggregation.red_h_high_2", 180);
        this->declare_parameter<int>("kfs_instance_aggregation.red_s_low", 130);
        this->declare_parameter<int>("kfs_instance_aggregation.red_v_low", 80);
        this->declare_parameter<int>("kfs_instance_aggregation.blue_h_low", 90);
        this->declare_parameter<int>("kfs_instance_aggregation.blue_h_high", 140);
        this->declare_parameter<int>("kfs_instance_aggregation.blue_s_low", 40);
        this->declare_parameter<int>("kfs_instance_aggregation.blue_v_low", 80);
        this->declare_parameter<int>("kfs_instance_aggregation.dark_blue_h_low", 90);
        this->declare_parameter<int>("kfs_instance_aggregation.dark_blue_h_high", 140);
        this->declare_parameter<int>("kfs_instance_aggregation.dark_blue_s_low", 20);
        this->declare_parameter<int>("kfs_instance_aggregation.dark_blue_v_low", 15);
        this->declare_parameter<int>("kfs_instance_aggregation.dark_blue_v_high", 160);
        this->declare_parameter<bool>("kfs_instance_aggregation.contour_enabled", true);
        this->declare_parameter<int>("kfs_instance_aggregation.min_contour_area_px", 300);
        this->declare_parameter<int>("kfs_instance_aggregation.morphology_kernel_size", 5);
        this->declare_parameter<int>("kfs_instance_aggregation.morphology_iterations", 1);
        this->declare_parameter<std::string>("kfs_instance_aggregation.contour_selection", "closest_to_symbol_union_center");
        this->declare_parameter<double>("kfs_instance_aggregation.contour_max_center_offset_ratio", 1.0);
        this->declare_parameter<double>("kfs_instance_aggregation.contour_min_refined_to_union_area_ratio", 0.5);
        this->declare_parameter<bool>("kfs_instance_aggregation.neighbor_aware_clamp", false);
        this->declare_parameter<int>("kfs_instance_aggregation.neighbor_overlap_margin_px", 10);
        this->declare_parameter<bool>("kfs_instance_aggregation.protect_other_cluster_symbols", true);
        this->declare_parameter<std::string>("kfs_instance_aggregation.neighbor_protection_bbox_source", "geometry");
        this->declare_parameter<int>("kfs_instance_aggregation.edge_clip_margin_px", 3);

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

        this->get_parameter(
            "r1_conf_threshold",
            decision_config_.r1_conf_threshold);
        this->get_parameter(
            "real_conf_threshold",
            decision_config_.real_conf_threshold);
        this->get_parameter(
            "fake_conf_threshold",
            decision_config_.fake_conf_threshold);
        this->get_parameter(
            "collect_min_confidence",
            decision_config_.collect_min_confidence);
        this->get_parameter(
            "yolo_confidence_weight",
            decision_config_.yolo_confidence_weight);
        this->get_parameter(
            "color_confidence_weight",
            decision_config_.color_confidence_weight);
        this->get_parameter(
            "require_team_color_match",
            decision_config_.require_team_color_match);
        this->get_parameter(
            "unknown_on_low_confidence",
            decision_config_.unknown_on_low_confidence);
        this->get_parameter("kfs_instance_aggregation.enabled", aggregator_config_.enable_aggregation);
        this->get_parameter("kfs_instance_aggregation.debug_instances", aggregator_config_.debug_instances);
        this->get_parameter("kfs_instance_aggregation.publish_debug_image", kfs_publish_debug_image_);
        this->get_parameter("kfs_instance_aggregation.debug_image_topic", kfs_debug_image_topic_);
        this->get_parameter("kfs_instance_aggregation.draw_roi", aggregator_config_.draw_roi);
        std::vector<int64_t> roi_color_bgr_param;
        this->get_parameter("kfs_instance_aggregation.roi_color_bgr", roi_color_bgr_param);
        if (roi_color_bgr_param.size() == 3) {
            aggregator_config_.roi_color_bgr = {
                static_cast<int>(roi_color_bgr_param[0]),
                static_cast<int>(roi_color_bgr_param[1]),
                static_cast<int>(roi_color_bgr_param[2])};
        }
        this->get_parameter("kfs_instance_aggregation.roi_thickness", aggregator_config_.roi_thickness);
        this->get_parameter("kfs_instance_aggregation.enable_roi_filter", aggregator_config_.enable_roi_filter);
        this->get_parameter("kfs_instance_aggregation.roi_x_min_norm", aggregator_config_.roi_x_min_norm);
        this->get_parameter("kfs_instance_aggregation.roi_x_max_norm", aggregator_config_.roi_x_max_norm);
        this->get_parameter("kfs_instance_aggregation.roi_y_min_norm", aggregator_config_.roi_y_min_norm);
        this->get_parameter("kfs_instance_aggregation.roi_y_max_norm", aggregator_config_.roi_y_max_norm);
        this->get_parameter("kfs_instance_aggregation.min_symbol_height_px", aggregator_config_.min_symbol_height_px);
        this->get_parameter("kfs_instance_aggregation.min_symbol_area_px", aggregator_config_.min_symbol_area_px);
        this->get_parameter("kfs_instance_aggregation.min_confidence", aggregator_config_.min_confidence);
        this->get_parameter("kfs_instance_aggregation.use_square_symbol_bbox", aggregator_config_.use_square_symbol_bbox);
        this->get_parameter("kfs_instance_aggregation.square_symbol_scale", aggregator_config_.square_symbol_scale);
        this->get_parameter("kfs_instance_aggregation.max_square_side_ratio_of_image", aggregator_config_.max_square_side_ratio_of_image);
        this->get_parameter("kfs_instance_aggregation.expand_scale_x", aggregator_config_.expand_scale_x);
        this->get_parameter("kfs_instance_aggregation.expand_scale_y", aggregator_config_.expand_scale_y);
        this->get_parameter("kfs_instance_aggregation.max_expanded_area_ratio", aggregator_config_.max_expanded_area_ratio);
        this->get_parameter("kfs_instance_aggregation.cluster_distance_scale", aggregator_config_.cluster_distance_scale);
        this->get_parameter("kfs_instance_aggregation.min_cluster_distance_px", aggregator_config_.min_cluster_distance_px);
        this->get_parameter("kfs_instance_aggregation.max_cluster_distance_px", aggregator_config_.max_cluster_distance_px);
        this->get_parameter("kfs_instance_aggregation.min_height_similarity_for_grouping", aggregator_config_.min_height_similarity_for_grouping);
        this->get_parameter("kfs_instance_aggregation.min_area_similarity_for_grouping", aggregator_config_.min_area_similarity_for_grouping);
        this->get_parameter("kfs_instance_aggregation.max_bottom_y_diff_ratio", aggregator_config_.max_bottom_y_diff_ratio);
        this->get_parameter("kfs_instance_aggregation.max_symbols_per_instance", aggregator_config_.max_symbols_per_instance);
        this->get_parameter("kfs_instance_aggregation.enable_basic_merge", aggregator_config_.enable_basic_merge);
        this->get_parameter("kfs_instance_aggregation.merge_same_group_only", aggregator_config_.merge_same_group_only);
        this->get_parameter("kfs_instance_aggregation.max_symbols_after_merge", aggregator_config_.max_symbols_after_merge);
        this->get_parameter("kfs_instance_aggregation.min_expanded_iou", aggregator_config_.min_expanded_iou);
        this->get_parameter("kfs_instance_aggregation.min_expanded_intersection_over_min_area", aggregator_config_.min_expanded_intersection_over_min_area);
        this->get_parameter("kfs_instance_aggregation.max_expanded_gap_px", aggregator_config_.max_expanded_gap_px);
        this->get_parameter("kfs_instance_aggregation.min_color_mask_coverage_for_merge", aggregator_config_.min_color_mask_coverage_for_merge);
        this->get_parameter("kfs_instance_aggregation.max_merged_union_width_scale", aggregator_config_.max_merged_union_width_scale);
        this->get_parameter("kfs_instance_aggregation.max_merged_union_height_scale", aggregator_config_.max_merged_union_height_scale);
        this->get_parameter("kfs_instance_aggregation.enable_low_mask_adjacent_same_group_fallback", aggregator_config_.enable_low_mask_adjacent_same_group_fallback);
        this->get_parameter("kfs_instance_aggregation.low_mask_min_height_similarity", aggregator_config_.low_mask_min_height_similarity);
        this->get_parameter("kfs_instance_aggregation.low_mask_max_bottom_y_diff_ratio", aggregator_config_.low_mask_max_bottom_y_diff_ratio);
        this->get_parameter("kfs_instance_aggregation.low_mask_max_gap_px", aggregator_config_.low_mask_max_gap_px);
        this->get_parameter("kfs_instance_aggregation.low_mask_max_gap_ratio_to_symbol_height", aggregator_config_.low_mask_max_gap_ratio_to_symbol_height);
        this->get_parameter("kfs_instance_aggregation.low_mask_max_merged_aspect_ratio", aggregator_config_.low_mask_max_merged_aspect_ratio);
        this->get_parameter("kfs_instance_aggregation.low_mask_max_merged_width_scale", aggregator_config_.low_mask_max_merged_width_scale);
        this->get_parameter("kfs_instance_aggregation.low_mask_max_merged_height_scale", aggregator_config_.low_mask_max_merged_height_scale);
        this->get_parameter("kfs_instance_aggregation.drop_ambiguous_clusters", aggregator_config_.drop_ambiguous_clusters);
        this->get_parameter("kfs_instance_aggregation.hsv_profile", aggregator_config_.hsv_profile);
        this->get_parameter("kfs_instance_aggregation.red_h_low_1", aggregator_config_.red_h_low_1);
        this->get_parameter("kfs_instance_aggregation.red_h_high_1", aggregator_config_.red_h_high_1);
        this->get_parameter("kfs_instance_aggregation.red_h_low_2", aggregator_config_.red_h_low_2);
        this->get_parameter("kfs_instance_aggregation.red_h_high_2", aggregator_config_.red_h_high_2);
        this->get_parameter("kfs_instance_aggregation.red_s_low", aggregator_config_.red_s_low);
        this->get_parameter("kfs_instance_aggregation.red_v_low", aggregator_config_.red_v_low);
        this->get_parameter("kfs_instance_aggregation.blue_h_low", aggregator_config_.blue_h_low);
        this->get_parameter("kfs_instance_aggregation.blue_h_high", aggregator_config_.blue_h_high);
        this->get_parameter("kfs_instance_aggregation.blue_s_low", aggregator_config_.blue_s_low);
        this->get_parameter("kfs_instance_aggregation.blue_v_low", aggregator_config_.blue_v_low);
        this->get_parameter("kfs_instance_aggregation.dark_blue_h_low", aggregator_config_.dark_blue_h_low);
        this->get_parameter("kfs_instance_aggregation.dark_blue_h_high", aggregator_config_.dark_blue_h_high);
        this->get_parameter("kfs_instance_aggregation.dark_blue_s_low", aggregator_config_.dark_blue_s_low);
        this->get_parameter("kfs_instance_aggregation.dark_blue_v_low", aggregator_config_.dark_blue_v_low);
        this->get_parameter("kfs_instance_aggregation.dark_blue_v_high", aggregator_config_.dark_blue_v_high);
        this->get_parameter("kfs_instance_aggregation.contour_enabled", aggregator_config_.contour_enabled);
        this->get_parameter("kfs_instance_aggregation.min_contour_area_px", aggregator_config_.min_contour_area_px);
        this->get_parameter("kfs_instance_aggregation.morphology_kernel_size", aggregator_config_.morphology_kernel_size);
        this->get_parameter("kfs_instance_aggregation.morphology_iterations", aggregator_config_.morphology_iterations);
        this->get_parameter("kfs_instance_aggregation.contour_selection", aggregator_config_.contour_selection);
        this->get_parameter("kfs_instance_aggregation.contour_max_center_offset_ratio", aggregator_config_.contour_max_center_offset_ratio);
        this->get_parameter("kfs_instance_aggregation.contour_min_refined_to_union_area_ratio", aggregator_config_.contour_min_refined_to_union_area_ratio);
        this->get_parameter("kfs_instance_aggregation.neighbor_aware_clamp", aggregator_config_.neighbor_aware_clamp);
        this->get_parameter("kfs_instance_aggregation.neighbor_overlap_margin_px", aggregator_config_.neighbor_overlap_margin_px);
        this->get_parameter("kfs_instance_aggregation.protect_other_cluster_symbols", aggregator_config_.protect_other_cluster_symbols);
        this->get_parameter("kfs_instance_aggregation.neighbor_protection_bbox_source", aggregator_config_.neighbor_protection_bbox_source);
        this->get_parameter("kfs_instance_aggregation.edge_clip_margin_px", aggregator_config_.edge_clip_margin_px);

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

        const double decision_weight_sum =
            decision_config_.yolo_confidence_weight +
            decision_config_.color_confidence_weight;
        if (std::abs(decision_weight_sum - 1.0) > 1e-3) {
            RCLCPP_WARN(
                this->get_logger(),
                "DecisionEngine weights sum to %.3f; normalizing for safe use",
                decision_weight_sum);
        }
        decision_config_ =
            abu_yolo_ros::normalizeDecisionConfig(decision_config_);

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
        RCLCPP_INFO(
            this->get_logger(),
            "DecisionEngine Config: r1=%.2f real=%.2f fake=%.2f collect=%.2f yolo_w=%.2f color_w=%.2f require_color=%s unknown_low_conf=%s",
            decision_config_.r1_conf_threshold,
            decision_config_.real_conf_threshold,
            decision_config_.fake_conf_threshold,
            decision_config_.collect_min_confidence,
            decision_config_.yolo_confidence_weight,
            decision_config_.color_confidence_weight,
            decision_config_.require_team_color_match ? "true" : "false",
            decision_config_.unknown_on_low_confidence ? "true" : "false");
        RCLCPP_INFO(
            this->get_logger(),
            "KFSInstanceAggregator: %s | debug=%s | roi=%s | hsv_profile=%s | square_bbox=%s | merge=%s | drop_ambiguous=%s",
            aggregator_config_.enable_aggregation ? "enabled" : "disabled",
            aggregator_config_.debug_instances ? "true" : "false",
            aggregator_config_.enable_roi_filter ? "true" : "false",
            aggregator_config_.hsv_profile.c_str(),
            aggregator_config_.use_square_symbol_bbox ? "true" : "false",
            aggregator_config_.enable_basic_merge ? "true" : "false",
            aggregator_config_.drop_ambiguous_clusters ? "true" : "false");
        RCLCPP_INFO(
            this->get_logger(),
            "KFS debug image: %s | topic=%s | draw_roi=%s",
            (aggregator_config_.enable_aggregation &&
             aggregator_config_.debug_instances &&
             kfs_publish_debug_image_) ? "enabled" : "disabled",
            kfs_debug_image_topic_.c_str(),
            aggregator_config_.draw_roi ? "true" : "false");

        detector_ = std::make_unique<abu_yolo_ros::YOLODetector>(
            model_path_,
            class_names_path_,
            use_gpu_);
        aggregator_ =
            std::make_unique<abu_yolo_ros::KFSInstanceAggregator>(
                aggregator_config_);

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

        kfs_debug_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>(
                kfs_debug_image_topic_,
                qos);

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
            const auto decision_results =
                evaluateDecisionResults(
                    detections,
                    team_color_results);
            maybeLogTeamColorResults(
                detections,
                team_color_results);
            maybeLogDecisionResults(
                decision_results);
            maybeLogDetectionDetails(
                detections,
                team_color_results,
                decision_results);
            const auto kfs_instances =
                evaluateKFSInstances(
                    bgr,
                    detections);
            maybeLogKFSInstances(kfs_instances);
            
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
            maybePublishKFSDebugImage(
                msg->header,
                bgr,
                kfs_instances);

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
        const std::vector<abu_yolo_ros::TeamColorResult>& team_color_results,
        const std::vector<abu_yolo_ros::DecisionResult>& decision_results)
    {
        if (!debug_detections_) {
            return;
        }

        std::ostringstream stream;
        stream << "Detection details count=" << detections.size();

        for (std::size_t i = 0; i < detections.size(); ++i) {
            const auto& detection = detections[i];
            const std::string class_label =
                detector_->getClassLabel(detection.class_id);

            stream << "\n  [" << i << "]"
                   << " class_id=" << detection.class_id
                   << " class_name=" << class_label
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

            if (i < decision_results.size()) {
                const auto& result = decision_results[i];
                stream << " decision="
                       << abu_yolo_ros::decisionToString(result.decision)
                       << " final_conf=" << result.final_confidence
                       << " reason=\"" << result.reason << "\"";
            }
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "%s",
            stream.str().c_str());
    }

    std::vector<abu_yolo_ros::DecisionResult> evaluateDecisionResults(
        const std::vector<abu_yolo_ros::Detection>& detections,
        const std::vector<abu_yolo_ros::TeamColorResult>& team_color_results) const
    {
        std::vector<abu_yolo_ros::DecisionResult> results;
        results.reserve(detections.size());

        for (std::size_t i = 0; i < detections.size(); ++i) {
            const abu_yolo_ros::TeamColorResult* color_result = nullptr;
            const std::string class_name =
                detector_->getClassLabel(detections[i].class_id);
            if (enable_team_color_filter_ &&
                i < team_color_results.size()) {
                color_result = &team_color_results[i];
            }

            results.push_back(
                abu_yolo_ros::classifyKFS(
                    detections[i].class_id,
                    class_name,
                    detections[i].confidence,
                    color_result,
                    decision_config_));
        }

        return results;
    }

    void maybeLogDecisionResults(
        const std::vector<abu_yolo_ros::DecisionResult>& decision_results)
    {
        std::size_t collect_count = 0;
        std::size_t avoid_count = 0;
        std::size_t unknown_count = 0;

        std::ostringstream details;
        bool has_details = false;

        for (std::size_t i = 0; i < decision_results.size(); ++i) {
            const auto& result = decision_results[i];

            switch (result.decision) {
            case abu_yolo_ros::KFSDecision::COLLECT:
                ++collect_count;
                break;
            case abu_yolo_ros::KFSDecision::AVOID:
                ++avoid_count;
                break;
            case abu_yolo_ros::KFSDecision::UNKNOWN:
            default:
                ++unknown_count;
                break;
            }

            if (!has_details) {
                details << "det[" << i << "]="
                        << abu_yolo_ros::decisionToString(result.decision)
                        << " final_conf=" << std::fixed << std::setprecision(2)
                        << result.final_confidence
                        << " reason=\"" << result.reason << "\"";
                has_details = true;
            }
        }

        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "DecisionEngine det=%zu collect=%zu avoid=%zu unknown=%zu %s",
            decision_results.size(),
            collect_count,
            avoid_count,
            unknown_count,
            has_details ? details.str().c_str() : "");
    }

    std::vector<abu_yolo_ros::SymbolCandidate> makeSymbolCandidates(
        const std::vector<abu_yolo_ros::Detection>& detections) const
    {
        std::vector<abu_yolo_ros::SymbolCandidate> symbols;
        symbols.reserve(detections.size());

        for (std::size_t i = 0; i < detections.size(); ++i) {
            const auto& detection = detections[i];
            abu_yolo_ros::SymbolCandidate symbol;
            symbol.index = static_cast<int>(i);
            symbol.class_id = detection.class_id;
            symbol.class_name = detector_->getClassLabel(detection.class_id);
            symbol.confidence = detection.confidence;
            symbol.raw_bbox = cv::Rect(
                static_cast<int>(std::round(detection.x - detection.w * 0.5F)),
                static_cast<int>(std::round(detection.y - detection.h * 0.5F)),
                std::max(1, static_cast<int>(std::round(detection.w))),
                std::max(1, static_cast<int>(std::round(detection.h))));
            symbol.geometry_bbox = symbol.raw_bbox;
            symbol.group_type =
                abu_yolo_ros::classifySymbolGroup(symbol.class_name);
            symbols.push_back(symbol);
        }

        return symbols;
    }

    std::vector<abu_yolo_ros::KFSInstance> evaluateKFSInstances(
        const cv::Mat& bgr,
        const std::vector<abu_yolo_ros::Detection>& detections)
    {
        if (!aggregator_ || !aggregator_config_.enable_aggregation) {
            return {};
        }

        const auto symbols = makeSymbolCandidates(detections);
        return aggregator_->aggregate(bgr, symbols);
    }

    void maybeLogKFSInstances(
        const std::vector<abu_yolo_ros::KFSInstance>& instances)
    {
        if (!aggregator_config_.debug_instances) {
            return;
        }

        const auto& last_symbols = aggregator_->lastSymbols();
        const auto& last_clusters = aggregator_->lastClusters();
        const auto& dropped_clusters = aggregator_->lastDroppedClusters();

        std::size_t kept_symbols = 0;
        for (const auto& symbol : last_symbols) {
            if (symbol.keep) {
                ++kept_symbols;
            }
        }

        std::ostringstream stream;
        stream << "KFS instances raw_symbols=" << last_symbols.size()
               << " kept_symbols=" << kept_symbols
               << " final_clusters=" << last_clusters.size()
               << " final_instances=" << instances.size()
               << " dropped_clusters=" << dropped_clusters.size();

        for (const auto& instance : instances) {
            stream << "\n  C" << instance.cluster_id
                   << " group=" << abu_yolo_ros::kfsGroupTypeToString(
                                      instance.group_type)
                   << " symbols=[";
            for (std::size_t i = 0; i < instance.symbol_indices.size(); ++i) {
                if (i > 0) {
                    stream << ",";
                }
                stream << instance.symbol_indices[i];
            }
            stream << "] classes=[";
            for (std::size_t i = 0; i < instance.class_names.size(); ++i) {
                if (i > 0) {
                    stream << ",";
                }
                stream << instance.class_names[i];
            }
            stream << "] refined_bbox=("
                   << instance.refined_bbox.x << ","
                   << instance.refined_bbox.y << ","
                   << instance.refined_bbox.width << ","
                   << instance.refined_bbox.height << ")"
                   << " quality=" << instance.bbox_quality
                   << " color_cov=" << std::fixed << std::setprecision(2)
                   << instance.color_mask_coverage;
        }

        for (const auto& dropped : dropped_clusters) {
            stream << "\n  dropped C" << dropped.cluster_id
                   << " group=" << abu_yolo_ros::kfsGroupTypeToString(
                                      dropped.group_type)
                   << " reason=\"" << dropped.ambiguous_reason << "\"";
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "%s",
            stream.str().c_str());
    }

    cv::Scalar kfsGroupColor(abu_yolo_ros::KFSGroupType group_type) const
    {
        switch (group_type) {
        case abu_yolo_ros::KFSGroupType::REAL:
            return cv::Scalar(0, 255, 0);
        case abu_yolo_ros::KFSGroupType::FAKE:
            return cv::Scalar(0, 0, 255);
        case abu_yolo_ros::KFSGroupType::R1:
            return cv::Scalar(255, 0, 0);
        case abu_yolo_ros::KFSGroupType::AMBIGUOUS:
            return cv::Scalar(0, 215, 255);
        case abu_yolo_ros::KFSGroupType::UNKNOWN:
        default:
            return cv::Scalar(255, 255, 255);
        }
    }

    cv::Mat drawKFSInstances(
        const cv::Mat& bgr,
        const std::vector<abu_yolo_ros::KFSInstance>& instances) const
    {
        cv::Mat canvas = bgr.clone();

        for (const auto& instance : instances) {
            const cv::Scalar color = kfsGroupColor(instance.group_type);
            const cv::Rect& bbox = instance.refined_bbox;
            cv::rectangle(canvas, bbox, color, 2);

            std::ostringstream label;
            label << "C" << instance.cluster_id
                  << " "
                  << abu_yolo_ros::kfsGroupTypeToString(instance.group_type);
            if (instance.symbol_indices.size() > 1) {
                label << " merged_symbols=" << instance.symbol_indices.size();
            }
            if (!instance.bbox_quality.empty()) {
                label << " " << instance.bbox_quality;
            }

            cv::putText(
                canvas,
                label.str(),
                cv::Point(
                    bbox.x,
                    std::max(20, bbox.y - 8)),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
                cv::LINE_AA);
        }

        return canvas;
    }

    void drawKFSROI(cv::Mat& canvas) const
    {
        if (!aggregator_ || !aggregator_config_.draw_roi) {
            return;
        }

        const cv::Rect& roi = aggregator_->lastEffectiveROI();
        if (roi.empty()) {
            return;
        }

        cv::Scalar color(0, 255, 255);
        if (aggregator_config_.roi_color_bgr.size() == 3) {
            color = cv::Scalar(
                aggregator_config_.roi_color_bgr[0],
                aggregator_config_.roi_color_bgr[1],
                aggregator_config_.roi_color_bgr[2]);
        }

        const int thickness = std::max(1, aggregator_config_.roi_thickness);
        cv::rectangle(canvas, roi, color, thickness);
        cv::putText(
            canvas,
            "ROI",
            cv::Point(
                roi.x,
                std::max(20, roi.y - 8)),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv::LINE_AA);
    }

    void maybePublishKFSDebugImage(
        const std_msgs::msg::Header& header,
        const cv::Mat& bgr,
        const std::vector<abu_yolo_ros::KFSInstance>& instances)
    {
        if (!aggregator_config_.enable_aggregation ||
            !aggregator_config_.debug_instances ||
            !kfs_publish_debug_image_ ||
            !kfs_debug_pub_) {
            return;
        }

        cv::Mat debug_image = bgr.clone();
        drawKFSROI(debug_image);
        debug_image = drawKFSInstances(debug_image, instances);
        auto debug_msg =
            cv_bridge::CvImage(
                header,
                "bgr8",
                debug_image).toImageMsg();
        kfs_debug_pub_->publish(*debug_msg);
    }

private:

    std::unique_ptr<abu_yolo_ros::YOLODetector> detector_;
    std::unique_ptr<abu_yolo_ros::KFSInstanceAggregator> aggregator_;

    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr sub_;

    rclcpp::Publisher<
        sensor_msgs::msg::Image>::SharedPtr pub_;

    rclcpp::Publisher<
        vision_msgs::msg::Detection2DArray
    >::SharedPtr detection_pub_;
    rclcpp::Publisher<
        sensor_msgs::msg::Image>::SharedPtr kfs_debug_pub_;

    std::string model_path_;
    std::string class_names_path_;
    std::string input_topic_;
    std::string output_topic_;

    bool use_gpu_;
    bool visualize_;
    bool log_timing_;
    bool debug_detections_;
    bool enable_team_color_filter_;
    bool kfs_publish_debug_image_;

    int skip_frames_;
    uint64_t frame_count_;
    std::string team_color_string_;
    std::string kfs_debug_image_topic_;
    abu_yolo_ros::TeamColor my_team_;
    abu_yolo_ros::TeamColorFilterConfig tcf_config_;
    abu_yolo_ros::DecisionEngineConfig decision_config_;
    abu_yolo_ros::KFSInstanceAggregatorConfig aggregator_config_;
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<YoloDetectionNode>());

    rclcpp::shutdown();

    return 0;
}
