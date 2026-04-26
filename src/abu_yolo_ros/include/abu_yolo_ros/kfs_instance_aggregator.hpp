#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace abu_yolo_ros {

enum class KFSGroupType {
    REAL,
    FAKE,
    R1,
    UNKNOWN,
    AMBIGUOUS
};

struct SymbolCandidate {
    int index = 0;
    int class_id = -1;
    std::string class_name;
    float confidence = 0.0F;
    cv::Rect raw_bbox;
    cv::Rect geometry_bbox;
    bool keep = true;
    std::string drop_reason;
    KFSGroupType group_type = KFSGroupType::UNKNOWN;
};

struct KFSCluster {
    int cluster_id = 0;
    std::vector<int> symbol_indices;
    std::vector<std::string> class_names;
    KFSGroupType group_type = KFSGroupType::UNKNOWN;
    bool ambiguous = false;
    std::string ambiguous_reason;
    cv::Rect union_bbox;
    cv::Rect expanded_bbox;
    float color_mask_coverage = 0.0F;
    cv::Rect refined_bbox;
    std::string bbox_quality = "normal";
    std::vector<int> merged_from;
    bool is_merged_cluster = false;
};

struct KFSInstance {
    int cluster_id = 0;
    std::vector<int> symbol_indices;
    std::vector<std::string> class_names;
    KFSGroupType group_type = KFSGroupType::UNKNOWN;
    cv::Rect refined_bbox;
    std::string bbox_quality = "normal";
    float color_mask_coverage = 0.0F;
};

struct KFSInstanceAggregatorConfig {
    bool enable_aggregation = true;
    bool debug_instances = true;
    bool draw_roi = true;
    std::vector<int> roi_color_bgr = {0, 255, 255};
    int roi_thickness = 2;

    bool enable_roi_filter = true;
    double roi_x_min_norm = 0.10;
    double roi_x_max_norm = 0.78;
    double roi_y_min_norm = 0.22;
    double roi_y_max_norm = 1.00;

    double min_symbol_height_px = 60.0;
    double min_symbol_area_px = 4000.0;
    double min_confidence = 0.25;

    double cluster_distance_scale = 2.5;
    double min_cluster_distance_px = 35.0;
    double max_cluster_distance_px = 180.0;
    double min_height_similarity_for_grouping = 0.60;
    double min_area_similarity_for_grouping = 0.30;
    double max_bottom_y_diff_ratio = 1.20;
    int max_symbols_per_instance = 3;

    bool use_square_symbol_bbox = true;
    double square_symbol_scale = 1.0;
    double max_square_side_ratio_of_image = 0.35;

    double expand_scale_x = 1.8;
    double expand_scale_y = 1.8;
    double max_expanded_area_ratio = 0.25;

    bool enable_basic_merge = true;
    bool merge_same_group_only = true;
    int max_symbols_after_merge = 3;
    double min_expanded_iou = 0.03;
    double min_expanded_intersection_over_min_area = 0.12;
    double max_expanded_gap_px = 40.0;
    double min_color_mask_coverage_for_merge = 0.15;
    double max_merged_union_width_scale = 2.2;
    double max_merged_union_height_scale = 2.6;

    bool enable_low_mask_adjacent_same_group_fallback = true;
    double low_mask_min_height_similarity = 0.55;
    double low_mask_max_bottom_y_diff_ratio = 1.35;
    double low_mask_max_gap_px = 45.0;
    double low_mask_max_gap_ratio_to_symbol_height = 0.55;
    double low_mask_max_merged_aspect_ratio = 2.2;
    double low_mask_max_merged_width_scale = 2.2;
    double low_mask_max_merged_height_scale = 2.6;

    bool drop_ambiguous_clusters = true;

    std::string hsv_profile = "competition_blue";
    int red_h_low_1 = 0;
    int red_h_high_1 = 12;
    int red_h_low_2 = 168;
    int red_h_high_2 = 180;
    int red_s_low = 130;
    int red_v_low = 80;
    int blue_h_low = 90;
    int blue_h_high = 140;
    int blue_s_low = 40;
    int blue_v_low = 80;
    int dark_blue_h_low = 90;
    int dark_blue_h_high = 140;
    int dark_blue_s_low = 20;
    int dark_blue_v_low = 15;
    int dark_blue_v_high = 160;

    bool contour_enabled = true;
    int min_contour_area_px = 300;
    int morphology_kernel_size = 5;
    int morphology_iterations = 1;
    std::string contour_selection = "closest_to_symbol_union_center";
    double contour_max_center_offset_ratio = 1.0;
    double contour_min_refined_to_union_area_ratio = 0.5;

    bool neighbor_aware_clamp = false;
    int neighbor_overlap_margin_px = 10;
    bool protect_other_cluster_symbols = true;
    std::string neighbor_protection_bbox_source = "geometry";

    int edge_clip_margin_px = 3;
};

class KFSInstanceAggregator {
public:
    explicit KFSInstanceAggregator(const KFSInstanceAggregatorConfig& config);

    std::vector<KFSInstance> aggregate(
        const cv::Mat& bgr_image,
        const std::vector<SymbolCandidate>& symbols);

    const std::vector<SymbolCandidate>& lastSymbols() const;
    const std::vector<KFSCluster>& lastClusters() const;
    const std::vector<KFSCluster>& lastDroppedClusters() const;
    const cv::Rect& lastEffectiveROI() const;

private:
    KFSInstanceAggregatorConfig config_;
    std::vector<SymbolCandidate> last_symbols_;
    std::vector<KFSCluster> last_clusters_;
    std::vector<KFSCluster> last_dropped_clusters_;
    cv::Rect last_effective_roi_;
};

std::string kfsGroupTypeToString(KFSGroupType group_type);
KFSGroupType classifySymbolGroup(const std::string& class_name);

}  // namespace abu_yolo_ros
