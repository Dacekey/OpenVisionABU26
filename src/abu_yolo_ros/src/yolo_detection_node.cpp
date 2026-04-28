#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "abu_yolo_ros/msg/kfs_instance.hpp"
#include "abu_yolo_ros/msg/kfs_instance_array.hpp"
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

        // Parameters
        this->declare_parameter<std::string>("model_path", "");
        this->declare_parameter<std::string>("class_names_path", "");
        this->declare_parameter<std::string>("input_topic", "/image_raw");
        this->declare_parameter<std::string>("output_topic", "/yolo/image_annotated");
        this->declare_parameter<std::string>("detection_topic", "/yolo/detections");
        this->declare_parameter<bool>("use_gpu", true);
        this->declare_parameter<bool>("visualize", true);
        this->declare_parameter<bool>("log_timing", false);
        this->declare_parameter<int>("skip_frames", 0);
        this->declare_parameter<bool>("debug_detections", true);
        this->declare_parameter<bool>("enable_team_color_filter", true);
        this->declare_parameter<std::string>("team_color", "red");
        this->declare_parameter<bool>("runtime_safety.qos.use_sensor_data_qos", true);
        this->declare_parameter<int>("runtime_safety.qos.queue_depth", 1);
        this->declare_parameter<bool>(
            "runtime_safety.threading.protect_inference_with_mutex",
            true);
        this->declare_parameter<bool>(
            "runtime_safety.threading.drop_frame_when_busy",
            true);
        this->declare_parameter<double>(
            "runtime_safety.threading.busy_log_throttle_sec",
            1.0);
        this->declare_parameter<bool>(
            "runtime_safety.circuit_breaker.enabled",
            true);
        this->declare_parameter<int>(
            "runtime_safety.circuit_breaker.failure_threshold",
            5);
        this->declare_parameter<int>(
            "runtime_safety.circuit_breaker.success_threshold",
            3);
        this->declare_parameter<double>(
            "runtime_safety.circuit_breaker.reset_timeout_sec",
            2.0);
        this->declare_parameter<double>(
            "runtime_safety.circuit_breaker.inference_timeout_ms",
            100.0);
        this->declare_parameter<double>(
            "runtime_safety.circuit_breaker.log_throttle_sec",
            1.0);

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
        this->declare_parameter<bool>("kfs_instance_aggregation.publish_instances", true);
        this->declare_parameter<std::string>(
            "kfs_instance_aggregation.instances_topic",
            "/yolo/kfs_instances");
        this->declare_parameter<bool>("kfs_instance_aggregation.publish_debug_image", false);
        this->declare_parameter<std::string>(
            "kfs_instance_aggregation.debug_image_topic",
            "/yolo/kfs_instances/image_annotated");
        this->declare_parameter<bool>("kfs_instance_aggregation.enable_instance_team_color_filter", true);
        this->declare_parameter<std::string>(
            "kfs_instance_aggregation.instance_color_primary_bbox",
            "refined_bbox");
        this->declare_parameter<std::string>(
            "kfs_instance_aggregation.instance_color_fallback_bbox",
            "expanded_bbox");
        this->declare_parameter<double>(
            "kfs_instance_aggregation.instance_color_min_confidence",
            0.30);
        this->declare_parameter<int>(
            "kfs_instance_aggregation.instance_color_min_crop_area_px",
            400);
        this->declare_parameter<bool>(
            "kfs_instance_aggregation.debug_instance_color",
            false);
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
        this->get_parameter("detection_topic", detection_topic_);
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
        this->get_parameter(
            "runtime_safety.qos.use_sensor_data_qos",
            use_sensor_data_qos_);
        this->get_parameter(
            "runtime_safety.qos.queue_depth",
            runtime_qos_depth_);
        this->get_parameter(
            "runtime_safety.threading.protect_inference_with_mutex",
            protect_inference_with_mutex_);
        this->get_parameter(
            "runtime_safety.threading.drop_frame_when_busy",
            drop_frame_when_busy_);
        this->get_parameter(
            "runtime_safety.threading.busy_log_throttle_sec",
            busy_log_throttle_sec_);
        this->get_parameter(
            "runtime_safety.circuit_breaker.enabled",
            circuit_breaker_.enabled);
        this->get_parameter(
            "runtime_safety.circuit_breaker.failure_threshold",
            circuit_breaker_.failure_threshold);
        this->get_parameter(
            "runtime_safety.circuit_breaker.success_threshold",
            circuit_breaker_.success_threshold);
        this->get_parameter(
            "runtime_safety.circuit_breaker.reset_timeout_sec",
            circuit_breaker_.reset_timeout_sec);
        this->get_parameter(
            "runtime_safety.circuit_breaker.inference_timeout_ms",
            inference_timeout_ms_);
        this->get_parameter(
            "runtime_safety.circuit_breaker.log_throttle_sec",
            circuit_breaker_log_throttle_sec_);
        this->get_parameter("kfs_instance_aggregation.enabled", aggregator_config_.enable_aggregation);
        this->get_parameter("kfs_instance_aggregation.debug_instances", aggregator_config_.debug_instances);
        this->get_parameter("kfs_instance_aggregation.publish_instances", kfs_publish_instances_);
        this->get_parameter("kfs_instance_aggregation.instances_topic", kfs_instances_topic_);
        this->get_parameter("kfs_instance_aggregation.publish_debug_image", kfs_publish_debug_image_);
        this->get_parameter("kfs_instance_aggregation.debug_image_topic", kfs_debug_image_topic_);
        this->get_parameter(
            "kfs_instance_aggregation.enable_instance_team_color_filter",
            aggregator_config_.enable_instance_team_color_filter);
        this->get_parameter(
            "kfs_instance_aggregation.instance_color_primary_bbox",
            aggregator_config_.instance_color_primary_bbox);
        this->get_parameter(
            "kfs_instance_aggregation.instance_color_fallback_bbox",
            aggregator_config_.instance_color_fallback_bbox);
        this->get_parameter(
            "kfs_instance_aggregation.instance_color_min_confidence",
            aggregator_config_.instance_color_min_confidence);
        this->get_parameter(
            "kfs_instance_aggregation.instance_color_min_crop_area_px",
            aggregator_config_.instance_color_min_crop_area_px);
        this->get_parameter(
            "kfs_instance_aggregation.debug_instance_color",
            aggregator_config_.debug_instance_color);
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
        if (runtime_qos_depth_ < 1) {
            runtime_qos_depth_ = 1;
        }
        if (busy_log_throttle_sec_ < 0.0) {
            busy_log_throttle_sec_ = 1.0;
        }
        if (circuit_breaker_.failure_threshold < 1) {
            circuit_breaker_.failure_threshold = 1;
        }
        if (circuit_breaker_.success_threshold < 1) {
            circuit_breaker_.success_threshold = 1;
        }
        if (circuit_breaker_.reset_timeout_sec < 0.0) {
            circuit_breaker_.reset_timeout_sec = 0.0;
        }
        if (inference_timeout_ms_ < 0.0) {
            inference_timeout_ms_ = 0.0;
        }
        if (circuit_breaker_log_throttle_sec_ < 0.0) {
            circuit_breaker_log_throttle_sec_ = 1.0;
        }

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
            "KFS instances topic: %s | topic=%s | type=abu_yolo_ros/msg/KfsInstanceArray",
            (aggregator_config_.enable_aggregation &&
             kfs_publish_instances_) ? "enabled" : "disabled",
            kfs_instances_topic_.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "KFS instance color: %s | primary=%s | fallback=%s | min_conf=%.2f",
            aggregator_config_.enable_instance_team_color_filter ? "enabled" : "disabled",
            aggregator_config_.instance_color_primary_bbox.c_str(),
            aggregator_config_.instance_color_fallback_bbox.c_str(),
            aggregator_config_.instance_color_min_confidence);
        RCLCPP_INFO(
            this->get_logger(),
            "KFS debug image: %s | topic=%s | draw_roi=%s",
            (aggregator_config_.enable_aggregation &&
             aggregator_config_.debug_instances &&
             kfs_publish_debug_image_) ? "enabled" : "disabled",
            kfs_debug_image_topic_.c_str(),
            aggregator_config_.draw_roi ? "true" : "false");
        RCLCPP_INFO(
            this->get_logger(),
            "Runtime safety QoS: mode=%s depth=%d",
            use_sensor_data_qos_ ? "sensor_data/best_effort" : "default",
            runtime_qos_depth_);
        RCLCPP_INFO(
            this->get_logger(),
            "Runtime safety threading: protect_inference_with_mutex=%s drop_frame_when_busy=%s busy_log_throttle_sec=%.2f",
            protect_inference_with_mutex_ ? "true" : "false",
            drop_frame_when_busy_ ? "true" : "false",
            busy_log_throttle_sec_);
        RCLCPP_INFO(
            this->get_logger(),
            "Runtime safety circuit breaker: %s failure_threshold=%d success_threshold=%d reset_timeout_sec=%.2f inference_timeout_ms=%.2f",
            circuit_breaker_.enabled ? "enabled" : "disabled",
            circuit_breaker_.failure_threshold,
            circuit_breaker_.success_threshold,
            circuit_breaker_.reset_timeout_sec,
            inference_timeout_ms_);

        detector_ = std::make_unique<abu_yolo_ros::YOLODetector>(
            model_path_,
            class_names_path_,
            use_gpu_);
        aggregator_ =
            std::make_unique<abu_yolo_ros::KFSInstanceAggregator>(
                aggregator_config_);

        const auto qos = makePerceptionQoS();

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
                detection_topic_,
                qos
            );
        kfs_instances_pub_ =
            this->create_publisher<
                abu_yolo_ros::msg::KfsInstanceArray
            >(
                kfs_instances_topic_,
                qos
            );

        kfs_debug_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>(
                kfs_debug_image_topic_,
                qos);

        RCLCPP_INFO(this->get_logger(), "YOLO Detection Node started");
    }

private:

    struct CircuitBreaker {
        enum class State {
            CLOSED,
            OPEN,
            HALF_OPEN
        };

        bool enabled = true;
        int failure_threshold = 5;
        int success_threshold = 3;
        double reset_timeout_sec = 2.0;
        State state = State::CLOSED;
        int consecutive_failures = 0;
        int consecutive_successes = 0;
        std::chrono::steady_clock::time_point opened_at{};

        bool allowInference(
            const std::chrono::steady_clock::time_point& now,
            State* transition_to = nullptr)
        {
            if (!enabled) {
                return true;
            }
            if (state != State::OPEN) {
                return true;
            }

            const double open_duration_sec =
                std::chrono::duration<double>(now - opened_at).count();
            if (open_duration_sec < reset_timeout_sec) {
                return false;
            }

            state = State::HALF_OPEN;
            consecutive_successes = 0;
            consecutive_failures = 0;
            if (transition_to != nullptr) {
                *transition_to = state;
            }
            return true;
        }

        void recordSuccess(State* transition_to = nullptr)
        {
            if (!enabled) {
                return;
            }

            consecutive_failures = 0;
            if (state == State::HALF_OPEN) {
                ++consecutive_successes;
                if (consecutive_successes >= success_threshold) {
                    state = State::CLOSED;
                    consecutive_successes = 0;
                    if (transition_to != nullptr) {
                        *transition_to = state;
                    }
                }
                return;
            }

            state = State::CLOSED;
            consecutive_successes = 0;
        }

        void recordFailure(
            const std::chrono::steady_clock::time_point& now,
            State* transition_to = nullptr)
        {
            if (!enabled) {
                return;
            }

            consecutive_successes = 0;
            if (state == State::HALF_OPEN) {
                state = State::OPEN;
                opened_at = now;
                consecutive_failures = 1;
                if (transition_to != nullptr) {
                    *transition_to = state;
                }
                return;
            }

            ++consecutive_failures;
            if (state == State::CLOSED &&
                consecutive_failures >= failure_threshold) {
                state = State::OPEN;
                opened_at = now;
                if (transition_to != nullptr) {
                    *transition_to = state;
                }
            }
        }

        static const char* stateString(State state)
        {
            switch (state) {
            case State::CLOSED:
                return "CLOSED";
            case State::OPEN:
                return "OPEN";
            case State::HALF_OPEN:
                return "HALF_OPEN";
            default:
                return "UNKNOWN";
            }
        }
    };

    struct InstanceColorEvaluation {
        abu_yolo_ros::TeamColorResult result{
            false,
            abu_yolo_ros::TeamColor::UNKNOWN,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F,
            0.0F};
        std::string source = "unavailable";
        std::string fallback_note;
        bool used_fallback = false;
    };

    rclcpp::QoS makePerceptionQoS() const
    {
        rclcpp::QoS qos{rclcpp::KeepLast(runtime_qos_depth_)};
        if (use_sensor_data_qos_) {
            qos.best_effort();
            qos.durability_volatile();
        }
        return qos;
    }

    int throttleMs(double seconds) const
    {
        return std::max(
            1,
            static_cast<int>(std::round(seconds * 1000.0)));
    }

    bool beginInferenceGuard()
    {
        const auto now = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(circuit_breaker_mutex_);
            const auto previous_state = circuit_breaker_.state;
            auto transition_state = previous_state;
            if (!circuit_breaker_.allowInference(now, &transition_state)) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    throttleMs(circuit_breaker_log_throttle_sec_),
                    "Circuit breaker OPEN; skipping frame publish and inference");
                return false;
            }

            if (transition_state != previous_state) {
                logCircuitBreakerTransition(previous_state, transition_state);
            }
        }
        return true;
    }

    bool acquireInferenceLock(std::unique_lock<std::mutex>& lock)
    {
        if (!protect_inference_with_mutex_) {
            return true;
        }

        lock = std::unique_lock<std::mutex>(inference_mutex_, std::defer_lock);
        if (drop_frame_when_busy_) {
            if (!lock.try_lock()) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    throttleMs(busy_log_throttle_sec_),
                    "Inference busy; dropping frame");
                return false;
            }
            return true;
        }

        lock.lock();
        return true;
    }

    void logCircuitBreakerTransition(
        CircuitBreaker::State from,
        CircuitBreaker::State to) const
    {
        if (from == to) {
            return;
        }
        RCLCPP_WARN(
            this->get_logger(),
            "Circuit breaker %s -> %s",
            CircuitBreaker::stateString(from),
            CircuitBreaker::stateString(to));
    }

    void recordInferenceSuccess()
    {
        if (!circuit_breaker_.enabled) {
            return;
        }

        std::lock_guard<std::mutex> lock(circuit_breaker_mutex_);
        const auto previous_state = circuit_breaker_.state;
        CircuitBreaker::State transition_state = previous_state;
        circuit_breaker_.recordSuccess(&transition_state);
        if (transition_state != previous_state) {
            logCircuitBreakerTransition(previous_state, transition_state);
        }
    }

    void recordInferenceFailure(
        double latency_ms,
        const std::string& reason)
    {
        const auto now = std::chrono::steady_clock::now();
        if (latency_ms >= 0.0) {
            RCLCPP_WARN(
                this->get_logger(),
                "Inference timeout/failure latency=%.2f ms reason=%s",
                latency_ms,
                reason.c_str());
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "Inference failure reason=%s",
                reason.c_str());
        }

        if (!circuit_breaker_.enabled) {
            return;
        }

        std::lock_guard<std::mutex> lock(circuit_breaker_mutex_);
        const auto previous_state = circuit_breaker_.state;
        CircuitBreaker::State transition_state = previous_state;
        circuit_breaker_.recordFailure(now, &transition_state);
        if (transition_state != previous_state) {
            logCircuitBreakerTransition(previous_state, transition_state);
        }
    }

    void logSkipPublishForFailedInferenceFrame() const
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            throttleMs(circuit_breaker_log_throttle_sec_),
            "Skipping publish for failed inference frame");
    }

    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg) {

        frame_count_++;

        // Skip frame logic
        if (skip_frames_ > 0) {
            int period = skip_frames_ + 1;
            if ((frame_count_ % period) != 1)
                return;
        }

        if (!beginInferenceGuard()) {
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
            std::vector<abu_yolo_ros::Detection> detections;
            {
                std::unique_lock<std::mutex> inference_lock;
                if (!acquireInferenceLock(inference_lock)) {
                    return;
                }
                detections =
                    detector_->infer(bgr);
            }

            auto t2 = std::chrono::steady_clock::now();
            const double infer_ms =
                std::chrono::duration<double, std::milli>(
                    t2 - t1).count();
            if (inference_timeout_ms_ > 0.0 &&
                infer_ms > inference_timeout_ms_) {
                std::ostringstream reason;
                reason << "inference timeout threshold exceeded state="
                       << currentCircuitBreakerStateString();
                recordInferenceFailure(infer_ms, reason.str());
                logSkipPublishForFailedInferenceFrame();
                return;
            }
            recordInferenceSuccess();

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
            const auto kfs_instance_colors =
                evaluateKFSInstanceColors(
                    bgr,
                    kfs_instances);
            maybeLogKFSInstances(
                kfs_instances,
                kfs_instance_colors);
            
            publishDetections(detections);
            publishKFSInstances(
                msg->header,
                kfs_instances,
                kfs_instance_colors);

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
            recordInferenceFailure(-1.0, e.what());
            logSkipPublishForFailedInferenceFrame();
            RCLCPP_ERROR(
                this->get_logger(),
                "Inference error: %s",
                e.what());
        }
    }

    std::string currentCircuitBreakerStateString() const
    {
        std::lock_guard<std::mutex> lock(circuit_breaker_mutex_);
        return CircuitBreaker::stateString(circuit_breaker_.state);
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
            "Published %ld symbol detections to %s",
            msg.detections.size(),
            detection_topic_.c_str()
        );

        detection_pub_->publish(msg);
    }

    vision_msgs::msg::BoundingBox2D toBoundingBox2D(
        const cv::Rect& bbox) const
    {
        vision_msgs::msg::BoundingBox2D msg;
        msg.center.position.x = bbox.x + bbox.width * 0.5;
        msg.center.position.y = bbox.y + bbox.height * 0.5;
        msg.center.theta = 0.0;
        msg.size_x = bbox.width;
        msg.size_y = bbox.height;
        return msg;
    }

    const abu_yolo_ros::KFSCluster* findClusterById(int cluster_id) const
    {
        if (!aggregator_) {
            return nullptr;
        }

        const auto& clusters = aggregator_->lastClusters();
        for (const auto& cluster : clusters) {
            if (cluster.cluster_id == cluster_id) {
                return &cluster;
            }
        }
        return nullptr;
    }

    float computeInstanceConfidence(
        const abu_yolo_ros::KFSInstance& instance) const
    {
        if (!aggregator_) {
            return 0.0F;
        }

        const auto& symbols = aggregator_->lastSymbols();
        float best_confidence = 0.0F;
        for (const int symbol_index : instance.symbol_indices) {
            for (const auto& symbol : symbols) {
                if (symbol.index == symbol_index) {
                    best_confidence = std::max(best_confidence, symbol.confidence);
                    break;
                }
            }
        }

        // TODO(dacekey): replace this placeholder with calibrated instance-level confidence aggregation.
        return best_confidence;
    }

    abu_yolo_ros::KFSInstanceDecisionInput buildKFSInstanceDecisionInput(
        const abu_yolo_ros::KFSInstance& instance,
        const InstanceColorEvaluation& color_evaluation) const
    {
        abu_yolo_ros::KFSInstanceDecisionInput input;
        input.group_type = abu_yolo_ros::kfsGroupTypeToString(instance.group_type);
        input.symbol_confidence = computeInstanceConfidence(instance);
        input.team_color_match = color_evaluation.result.matches_team;
        input.color_confidence = color_evaluation.result.confidence;
        input.color_mask_coverage = instance.color_mask_coverage;
        input.bbox_quality = instance.bbox_quality.empty() ? "unknown" : instance.bbox_quality;
        const auto* cluster = findClusterById(instance.cluster_id);
        input.ambiguous = cluster ? cluster->ambiguous : false;
        input.ambiguous_reason = cluster ? cluster->ambiguous_reason : "";
        return input;
    }

    abu_yolo_ros::msg::KfsInstance toKfsInstanceMsg(
        const abu_yolo_ros::KFSInstance& instance,
        const InstanceColorEvaluation& color_evaluation,
        const abu_yolo_ros::DecisionResult& decision_result) const
    {
        abu_yolo_ros::msg::KfsInstance msg;
        msg.cluster_id = instance.cluster_id;
        msg.group_type = abu_yolo_ros::kfsGroupTypeToString(instance.group_type);
        msg.bbox = toBoundingBox2D(instance.refined_bbox);
        msg.bbox_quality = instance.bbox_quality.empty() ? "unknown" : instance.bbox_quality;
        msg.symbol_indices = instance.symbol_indices;
        msg.class_names = instance.class_names;
        msg.team_color = abu_yolo_ros::teamColorToString(my_team_);
        msg.team_color_match = color_evaluation.result.matches_team;
        msg.color_confidence = color_evaluation.result.confidence;
        msg.color_mask_coverage = instance.color_mask_coverage;

        const auto* cluster = findClusterById(instance.cluster_id);
        msg.ambiguous = cluster ? cluster->ambiguous : false;
        msg.ambiguous_reason = cluster ? cluster->ambiguous_reason : "";
        msg.confidence = static_cast<float>(decision_result.final_confidence);
        msg.decision = abu_yolo_ros::decisionToString(decision_result.decision);
        return msg;
    }

    void publishKFSInstances(
        const std_msgs::msg::Header& header,
        const std::vector<abu_yolo_ros::KFSInstance>& instances,
        const std::vector<InstanceColorEvaluation>& color_evaluations)
    {
        if (!aggregator_config_.enable_aggregation ||
            !kfs_publish_instances_ ||
            !kfs_instances_pub_) {
            return;
        }

        abu_yolo_ros::msg::KfsInstanceArray msg;
        msg.header = header;
        msg.team_color = abu_yolo_ros::teamColorToString(my_team_);
        msg.instances.reserve(instances.size());
        for (std::size_t i = 0; i < instances.size(); ++i) {
            const InstanceColorEvaluation fallback_evaluation;
            const InstanceColorEvaluation& color_evaluation =
                i < color_evaluations.size()
                    ? color_evaluations[i]
                    : fallback_evaluation;
            const auto decision_input =
                buildKFSInstanceDecisionInput(
                    instances[i],
                    color_evaluation);
            const auto decision_result =
                abu_yolo_ros::classifyKFSInstance(
                    decision_input,
                    decision_config_);
            msg.instances.push_back(
                toKfsInstanceMsg(
                    instances[i],
                    color_evaluation,
                    decision_result));
        }

        kfs_instances_pub_->publish(msg);

        if (aggregator_config_.debug_instances) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Published %ld KFS instances to %s",
                msg.instances.size(),
                kfs_instances_topic_.c_str());
        }
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
                stream << " symbol_decision="
                       << abu_yolo_ros::decisionToString(result.decision)
                       << " symbol_final_conf=" << result.final_confidence
                       << " symbol_reason=\"" << result.reason << "\"";
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
        std::size_t legal_count = 0;
        std::size_t illegal_count = 0;
        std::size_t unknown_count = 0;

        std::ostringstream details;
        bool has_details = false;

        for (std::size_t i = 0; i < decision_results.size(); ++i) {
            const auto& result = decision_results[i];

            switch (result.decision) {
            case abu_yolo_ros::KFSDecision::LEGAL:
                ++legal_count;
                break;
            case abu_yolo_ros::KFSDecision::ILLEGAL:
                ++illegal_count;
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
            "DecisionEngine det=%zu legal=%zu illegal=%zu unknown=%zu %s",
            decision_results.size(),
            legal_count,
            illegal_count,
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

    static cv::Rect clampRectToImage(
        const cv::Rect& rect,
        const cv::Size& image_size)
    {
        const int x1 = std::clamp(rect.x, 0, image_size.width);
        const int y1 = std::clamp(rect.y, 0, image_size.height);
        const int x2 = std::clamp(rect.x + rect.width, 0, image_size.width);
        const int y2 = std::clamp(rect.y + rect.height, 0, image_size.height);
        return cv::Rect(
            x1,
            y1,
            std::max(0, x2 - x1),
            std::max(0, y2 - y1));
    }

    static float rectAreaRatio(
        const cv::Rect& current,
        const cv::Rect& original)
    {
        const float original_area =
            static_cast<float>(std::max(0, original.area()));
        if (original_area <= 1e-6F) {
            return 0.0F;
        }
        return static_cast<float>(std::max(0, current.area())) / original_area;
    }

    cv::Rect selectClusterBBox(
        const abu_yolo_ros::KFSCluster& cluster,
        const std::string& source_name) const
    {
        if (source_name == "refined_bbox") {
            return cluster.refined_bbox;
        }
        if (source_name == "expanded_bbox") {
            return cluster.expanded_bbox;
        }
        if (source_name == "union_bbox") {
            return cluster.union_bbox;
        }
        return cv::Rect();
    }

    bool isValidInstanceColorCrop(
        const cv::Rect& requested_bbox,
        const cv::Rect& clamped_bbox) const
    {
        if (requested_bbox.width <= 0 || requested_bbox.height <= 0) {
            return false;
        }
        if (clamped_bbox.width <= 0 || clamped_bbox.height <= 0) {
            return false;
        }
        if (clamped_bbox.area() < aggregator_config_.instance_color_min_crop_area_px) {
            return false;
        }
        return rectAreaRatio(clamped_bbox, requested_bbox) >= 0.70F;
    }

    InstanceColorEvaluation evaluateInstanceColorForCluster(
        const cv::Mat& bgr,
        const abu_yolo_ros::KFSCluster& cluster) const
    {
        InstanceColorEvaluation evaluation;
        if (!enable_team_color_filter_ ||
            !aggregator_config_.enable_instance_team_color_filter ||
            bgr.empty()) {
            evaluation.fallback_note = "instance_team_color_disabled";
            return evaluation;
        }

        const auto try_bbox_source =
            [&](const std::string& source_name) {
                InstanceColorEvaluation attempt;
                attempt.source = source_name.empty() ? "unavailable" : source_name;
                if (source_name.empty()) {
                    attempt.fallback_note = "empty_bbox_source";
                    return attempt;
                }

                const cv::Rect requested_bbox =
                    selectClusterBBox(cluster, source_name);
                if (requested_bbox.width <= 0 || requested_bbox.height <= 0) {
                    attempt.fallback_note = source_name + "_invalid";
                    return attempt;
                }

                const cv::Rect clamped_bbox =
                    clampRectToImage(requested_bbox, bgr.size());
                if (!isValidInstanceColorCrop(requested_bbox, clamped_bbox)) {
                    attempt.fallback_note = source_name + "_crop_invalid";
                    return attempt;
                }

                attempt.result =
                    abu_yolo_ros::filterByTeamColor(
                        bgr,
                        clamped_bbox,
                        my_team_,
                        tcf_config_);
                return attempt;
            };

        const auto primary =
            try_bbox_source(aggregator_config_.instance_color_primary_bbox);
        if (primary.result.confidence >= aggregator_config_.instance_color_min_confidence) {
            return primary;
        }

        if (aggregator_config_.instance_color_fallback_bbox.empty() ||
            aggregator_config_.instance_color_fallback_bbox ==
                aggregator_config_.instance_color_primary_bbox) {
            InstanceColorEvaluation result = primary;
            if (result.fallback_note.empty() &&
                result.source != "unavailable") {
                std::ostringstream note;
                note << result.source
                     << " weak conf="
                     << std::fixed
                     << std::setprecision(2)
                     << result.result.confidence;
                result.fallback_note = note.str();
            }
            return result;
        }

        auto fallback =
            try_bbox_source(aggregator_config_.instance_color_fallback_bbox);
        fallback.used_fallback = true;

        if (primary.source != "unavailable") {
            std::ostringstream note;
            if (!primary.fallback_note.empty()) {
                note << primary.fallback_note;
            } else {
                note << primary.source
                     << " weak conf="
                     << std::fixed
                     << std::setprecision(2)
                     << primary.result.confidence;
            }
            note << " -> fallback " << fallback.source
                 << " conf=" << std::fixed << std::setprecision(2)
                 << fallback.result.confidence;
            fallback.fallback_note = note.str();
        }

        if (fallback.result.confidence >= aggregator_config_.instance_color_min_confidence) {
            return fallback;
        }

        if (primary.result.confidence >= fallback.result.confidence) {
            InstanceColorEvaluation result = primary;
            result.used_fallback = fallback.used_fallback;
            if (result.fallback_note.empty()) {
                result.fallback_note = fallback.fallback_note;
            }
            return result;
        }

        return fallback;
    }

    std::vector<InstanceColorEvaluation> evaluateKFSInstanceColors(
        const cv::Mat& bgr,
        const std::vector<abu_yolo_ros::KFSInstance>& instances) const
    {
        std::vector<InstanceColorEvaluation> evaluations;
        evaluations.reserve(instances.size());

        for (const auto& instance : instances) {
            const auto* cluster = findClusterById(instance.cluster_id);
            if (!cluster) {
                InstanceColorEvaluation evaluation;
                evaluation.fallback_note = "cluster_not_found";
                evaluations.push_back(evaluation);
                continue;
            }
            evaluations.push_back(
                evaluateInstanceColorForCluster(
                    bgr,
                    *cluster));
        }

        return evaluations;
    }

    void maybeLogKFSInstances(
        const std::vector<abu_yolo_ros::KFSInstance>& instances,
        const std::vector<InstanceColorEvaluation>& color_evaluations)
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
            const std::size_t instance_index =
                static_cast<std::size_t>(&instance - instances.data());
            const InstanceColorEvaluation* color_eval =
                instance_index < color_evaluations.size()
                    ? &color_evaluations[instance_index]
                    : nullptr;
            const InstanceColorEvaluation fallback_evaluation;
            const InstanceColorEvaluation& effective_color_eval =
                color_eval != nullptr ? *color_eval : fallback_evaluation;
            const auto decision_input =
                buildKFSInstanceDecisionInput(
                    instance,
                    effective_color_eval);
            const auto decision_result =
                abu_yolo_ros::classifyKFSInstance(
                    decision_input,
                    decision_config_);
            stream << "\n  [[C" << instance.cluster_id << "]]"
                   << " group=" << abu_yolo_ros::kfsGroupTypeToString(
                                      instance.group_type)
                   << " instance_decision=" << abu_yolo_ros::decisionToString(decision_result.decision)
                   << " instance_final_conf=" << std::fixed << std::setprecision(2)
                   << decision_result.final_confidence
                   << " instance_reason=\"" << decision_result.reason << "\""
                   << " team=" << abu_yolo_ros::teamColorToString(my_team_)
                   << " color_match=" << (effective_color_eval.result.matches_team ? "true" : "false")
                   << " color_conf=" << std::fixed << std::setprecision(2)
                   << effective_color_eval.result.confidence
                   << " color_source=" << effective_color_eval.source
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
            if (aggregator_config_.debug_instance_color &&
                !effective_color_eval.fallback_note.empty()) {
                stream << "\n    color_debug " << effective_color_eval.fallback_note;
            }
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
        abu_yolo_ros::msg::KfsInstanceArray
    >::SharedPtr kfs_instances_pub_;
    rclcpp::Publisher<
        sensor_msgs::msg::Image>::SharedPtr kfs_debug_pub_;

    std::string model_path_;
    std::string class_names_path_;
    std::string input_topic_;
    std::string output_topic_;
    std::string detection_topic_ = "/yolo/detections";

    bool use_gpu_;
    bool visualize_;
    bool log_timing_;
    bool debug_detections_;
    bool enable_team_color_filter_;
    bool kfs_publish_instances_;
    bool kfs_publish_debug_image_;

    int skip_frames_;
    int runtime_qos_depth_ = 1;
    uint64_t frame_count_;
    double busy_log_throttle_sec_ = 1.0;
    double inference_timeout_ms_ = 100.0;
    double circuit_breaker_log_throttle_sec_ = 1.0;
    std::string team_color_string_;
    std::string kfs_instances_topic_;
    std::string kfs_debug_image_topic_;
    bool use_sensor_data_qos_ = true;
    bool protect_inference_with_mutex_ = true;
    bool drop_frame_when_busy_ = true;
    abu_yolo_ros::TeamColor my_team_;
    abu_yolo_ros::TeamColorFilterConfig tcf_config_;
    abu_yolo_ros::DecisionEngineConfig decision_config_;
    abu_yolo_ros::KFSInstanceAggregatorConfig aggregator_config_;
    mutable std::mutex inference_mutex_;
    mutable std::mutex circuit_breaker_mutex_;
    CircuitBreaker circuit_breaker_;
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<YoloDetectionNode>());

    rclcpp::shutdown();

    return 0;
}
