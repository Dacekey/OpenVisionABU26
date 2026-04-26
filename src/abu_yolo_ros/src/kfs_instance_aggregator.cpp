#include "abu_yolo_ros/kfs_instance_aggregator.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace abu_yolo_ros {
namespace {

struct MergeCandidate {
    bool allowed = false;
    bool used_low_mask_fallback = false;
    std::string reason;
    float iou = 0.0F;
    float intersection_over_min_area = 0.0F;
    float gap_px = 0.0F;
};

cv::Rect clampRect(const cv::Rect& rect, int image_width, int image_height)
{
    const int max_x = std::max(0, image_width - 1);
    const int max_y = std::max(0, image_height - 1);

    const int x1 = std::clamp(rect.x, 0, max_x);
    const int y1 = std::clamp(rect.y, 0, max_y);
    const int x2 = std::clamp(rect.x + rect.width, 0, image_width);
    const int y2 = std::clamp(rect.y + rect.height, 0, image_height);

    return cv::Rect(
        x1,
        y1,
        std::max(0, x2 - x1),
        std::max(0, y2 - y1));
}

cv::Rect rectFromCenter(float center_x, float center_y, float width, float height)
{
    const int x1 = static_cast<int>(std::round(center_x - width * 0.5F));
    const int y1 = static_cast<int>(std::round(center_y - height * 0.5F));
    const int w = std::max(1, static_cast<int>(std::round(width)));
    const int h = std::max(1, static_cast<int>(std::round(height)));
    return cv::Rect(x1, y1, w, h);
}

float rectArea(const cv::Rect& rect)
{
    return static_cast<float>(std::max(1, rect.width) * std::max(1, rect.height));
}

cv::Point2f rectCenter(const cv::Rect& rect)
{
    return cv::Point2f(
        static_cast<float>(rect.x) + rect.width * 0.5F,
        static_cast<float>(rect.y) + rect.height * 0.5F);
}

float similarityRatio(float first, float second)
{
    const float high = std::max(first, second);
    const float low = std::min(first, second);
    if (high <= 1e-6F) {
        return 0.0F;
    }
    return low / high;
}

float bboxIou(const cv::Rect& first, const cv::Rect& second)
{
    const cv::Rect intersection = first & second;
    if (intersection.empty()) {
        return 0.0F;
    }
    const float inter_area = rectArea(intersection);
    const float union_area = rectArea(first) + rectArea(second) - inter_area;
    if (union_area <= 1e-6F) {
        return 0.0F;
    }
    return inter_area / union_area;
}

float bboxIntersectionOverMinArea(const cv::Rect& first, const cv::Rect& second)
{
    const cv::Rect intersection = first & second;
    if (intersection.empty()) {
        return 0.0F;
    }
    return rectArea(intersection) / std::min(rectArea(first), rectArea(second));
}

float bboxGapPx(const cv::Rect& first, const cv::Rect& second)
{
    const float dx = std::max(
        {static_cast<float>(second.x - (first.x + first.width)),
         static_cast<float>(first.x - (second.x + second.width)),
         0.0F});
    const float dy = std::max(
        {static_cast<float>(second.y - (first.y + first.height)),
         static_cast<float>(first.y - (second.y + second.height)),
         0.0F});
    return std::hypot(dx, dy);
}

bool semanticGroupsCompatible(
    KFSGroupType first,
    KFSGroupType second,
    bool merge_same_group_only)
{
    if (first == KFSGroupType::AMBIGUOUS || second == KFSGroupType::AMBIGUOUS) {
        return false;
    }

    if (merge_same_group_only) {
        if (first == KFSGroupType::UNKNOWN || second == KFSGroupType::UNKNOWN) {
            return first == KFSGroupType::UNKNOWN && second == KFSGroupType::UNKNOWN;
        }
        return first == second;
    }

    if (first == second) {
        return true;
    }

    if (first == KFSGroupType::UNKNOWN || second == KFSGroupType::UNKNOWN) {
        return false;
    }

    return false;
}

KFSGroupType aggregateGroupType(
    const std::vector<SymbolCandidate>& symbols,
    bool ambiguous)
{
    if (ambiguous) {
        return KFSGroupType::AMBIGUOUS;
    }

    bool has_r1 = false;
    bool has_fake = false;
    bool all_real = !symbols.empty();
    for (const auto& symbol : symbols) {
        if (symbol.class_name == "R1") {
            has_r1 = true;
        }
        if (symbol.class_name.rfind("FAKE_", 0) == 0) {
            has_fake = true;
        }
        if (symbol.class_name.rfind("REAL_", 0) != 0) {
            all_real = false;
        }
    }

    if (has_r1) {
        return KFSGroupType::R1;
    }
    if (has_fake) {
        return KFSGroupType::FAKE;
    }
    if (all_real) {
        return KFSGroupType::REAL;
    }
    return KFSGroupType::UNKNOWN;
}

cv::Rect unionRect(const std::vector<SymbolCandidate>& symbols, bool use_geometry_bbox)
{
    if (symbols.empty()) {
        return cv::Rect();
    }

    const cv::Rect& first_rect = use_geometry_bbox ? symbols.front().geometry_bbox : symbols.front().raw_bbox;
    int x1 = first_rect.x;
    int y1 = first_rect.y;
    int x2 = first_rect.x + first_rect.width;
    int y2 = first_rect.y + first_rect.height;

    for (const auto& symbol : symbols) {
        const cv::Rect& rect = use_geometry_bbox ? symbol.geometry_bbox : symbol.raw_bbox;
        x1 = std::min(x1, rect.x);
        y1 = std::min(y1, rect.y);
        x2 = std::max(x2, rect.x + rect.width);
        y2 = std::max(y2, rect.y + rect.height);
    }

    return cv::Rect(x1, y1, std::max(1, x2 - x1), std::max(1, y2 - y1));
}

cv::Rect makeSquareBBox(
    const cv::Rect& bbox,
    int image_width,
    int image_height,
    float scale,
    float max_square_side_ratio_of_image)
{
    const cv::Point2f center = rectCenter(bbox);
    float side = std::max(static_cast<float>(bbox.width), static_cast<float>(bbox.height));
    side *= std::max(1e-6F, static_cast<float>(scale));
    const float max_side = std::max(
        1.0F,
        static_cast<float>(max_square_side_ratio_of_image) *
            static_cast<float>(std::max(image_width, image_height)));
    side = std::min(side, max_side);
    const int side_px = std::max(1, static_cast<int>(std::round(side)));
    return clampRect(
        rectFromCenter(center.x, center.y, static_cast<float>(side_px), static_cast<float>(side_px)),
        image_width,
        image_height);
}

cv::Rect computeEffectiveROI(
    int image_width,
    int image_height,
    const KFSInstanceAggregatorConfig& config)
{
    if (image_width <= 0 || image_height <= 0) {
        return cv::Rect();
    }

    if (!config.enable_roi_filter) {
        return cv::Rect(0, 0, image_width, image_height);
    }

    const int x1 = static_cast<int>(std::floor(config.roi_x_min_norm * image_width));
    const int y1 = static_cast<int>(std::floor(config.roi_y_min_norm * image_height));
    const int x2 = static_cast<int>(std::ceil(config.roi_x_max_norm * image_width));
    const int y2 = static_cast<int>(std::ceil(config.roi_y_max_norm * image_height));

    return clampRect(
        cv::Rect(
            x1,
            y1,
            std::max(1, x2 - x1),
            std::max(1, y2 - y1)),
        image_width,
        image_height);
}

bool pointInsideNormalizedBox(
    const cv::Point2f& center,
    int image_width,
    int image_height,
    const KFSInstanceAggregatorConfig& config)
{
    if (!config.enable_roi_filter) {
        return true;
    }

    const float x_norm = center.x / static_cast<float>(std::max(1, image_width));
    const float y_norm = center.y / static_cast<float>(std::max(1, image_height));
    return x_norm >= config.roi_x_min_norm &&
           x_norm <= config.roi_x_max_norm &&
           y_norm >= config.roi_y_min_norm &&
           y_norm <= config.roi_y_max_norm;
}

cv::Rect expandRectWithLimit(
    const cv::Rect& rect,
    int image_width,
    int image_height,
    const KFSInstanceAggregatorConfig& config)
{
    const cv::Point2f center = rectCenter(rect);
    const float width = static_cast<float>(std::max(1, rect.width));
    const float height = static_cast<float>(std::max(1, rect.height));
    float scale_x = std::max(1.0F, static_cast<float>(config.expand_scale_x));
    float scale_y = std::max(1.0F, static_cast<float>(config.expand_scale_y));
    const float image_area = static_cast<float>(std::max(1, image_width * image_height));

    cv::Rect expanded = rect;
    for (int i = 0; i < 3; ++i) {
        expanded = clampRect(
            rectFromCenter(center.x, center.y, width * scale_x, height * scale_y),
            image_width,
            image_height);
        const float ratio = rectArea(expanded) / image_area;
        if (ratio <= static_cast<float>(config.max_expanded_area_ratio)) {
            return expanded;
        }
        const float scale_factor = std::sqrt(
            static_cast<float>(config.max_expanded_area_ratio) /
            std::max(ratio, 1e-6F));
        scale_x = std::max(1.0F, scale_x * scale_factor);
        scale_y = std::max(1.0F, scale_y * scale_factor);
    }
    return expanded;
}

std::vector<cv::Mat> selectColorMasks(
    const cv::Mat& hsv,
    const KFSInstanceAggregatorConfig& config)
{
    std::vector<cv::Mat> masks;
    const std::string& profile = config.hsv_profile;

    if (profile == "competition_blue" || profile == "blue") {
        cv::Mat blue_mask;
        cv::inRange(
            hsv,
            cv::Scalar(config.blue_h_low, config.blue_s_low, config.blue_v_low),
            cv::Scalar(config.blue_h_high, 255, 255),
            blue_mask);
        masks.push_back(blue_mask);
        return masks;
    }

    if (profile == "red") {
        cv::Mat red_1;
        cv::inRange(
            hsv,
            cv::Scalar(config.red_h_low_1, config.red_s_low, config.red_v_low),
            cv::Scalar(config.red_h_high_1, 255, 255),
            red_1);
        cv::Mat red_2;
        cv::inRange(
            hsv,
            cv::Scalar(config.red_h_low_2, config.red_s_low, config.red_v_low),
            cv::Scalar(config.red_h_high_2, 255, 255),
            red_2);
        masks.push_back(red_1 | red_2);
        return masks;
    }

    if (profile == "dark_blue_debug" || profile == "dark_blue") {
        cv::Mat dark_blue_mask;
        cv::inRange(
            hsv,
            cv::Scalar(config.dark_blue_h_low, config.dark_blue_s_low, config.dark_blue_v_low),
            cv::Scalar(config.dark_blue_h_high, 255, config.dark_blue_v_high),
            dark_blue_mask);
        masks.push_back(dark_blue_mask);
        return masks;
    }

    // Conservative fallback.
    cv::Mat fallback_mask;
    cv::inRange(
        hsv,
        cv::Scalar(config.blue_h_low, config.blue_s_low, config.blue_v_low),
        cv::Scalar(config.blue_h_high, 255, 255),
        fallback_mask);
    masks.push_back(fallback_mask);
    return masks;
}

cv::Mat applyMorphology(const cv::Mat& mask, const KFSInstanceAggregatorConfig& config)
{
    const int kernel_size = std::max(1, config.morphology_kernel_size);
    const int iterations = std::max(1, config.morphology_iterations);
    const cv::Mat kernel = cv::Mat::ones(kernel_size, kernel_size, CV_8U);

    cv::Mat result;
    cv::morphologyEx(mask, result, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), iterations);
    cv::morphologyEx(result, result, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), iterations);
    return result;
}

cv::Point2f contourCentroid(const std::vector<cv::Point>& contour)
{
    const cv::Moments moments = cv::moments(contour);
    if (std::abs(moments.m00) <= 1e-6) {
        const cv::Rect rect = cv::boundingRect(contour);
        return rectCenter(rect);
    }
    return cv::Point2f(
        static_cast<float>(moments.m10 / moments.m00),
        static_cast<float>(moments.m01 / moments.m00));
}

bool validNeighborClampBBox(const cv::Rect& bbox, const cv::Point2f& own_center)
{
    return bbox.width >= 5 &&
           bbox.height >= 5 &&
           own_center.x >= bbox.x &&
           own_center.x <= bbox.x + bbox.width &&
           own_center.y >= bbox.y &&
           own_center.y <= bbox.y + bbox.height;
}

std::vector<std::vector<int>> clusterSymbols(
    const std::vector<SymbolCandidate>& symbols,
    const KFSInstanceAggregatorConfig& config)
{
    if (symbols.empty()) {
        return {};
    }

    std::vector<int> parent(symbols.size());
    std::iota(parent.begin(), parent.end(), 0);
    const auto find_root = [&parent](int value) {
        int root = value;
        while (parent[root] != root) {
            root = parent[root];
        }
        while (parent[value] != value) {
            const int next = parent[value];
            parent[value] = root;
            value = next;
        }
        return root;
    };
    const auto unite = [&parent, &find_root](int first, int second) {
        const int root_first = find_root(first);
        const int root_second = find_root(second);
        if (root_first != root_second) {
            parent[root_second] = root_first;
        }
    };

    for (std::size_t i = 0; i < symbols.size(); ++i) {
        for (std::size_t j = i + 1; j < symbols.size(); ++j) {
            const auto& first = symbols[i];
            const auto& second = symbols[j];
            const cv::Point2f first_center = rectCenter(first.raw_bbox);
            const cv::Point2f second_center = rectCenter(second.raw_bbox);
            const float center_distance = std::hypot(
                first_center.x - second_center.x,
                first_center.y - second_center.y);
            const float average_symbol_size = 0.5F * (
                std::sqrt(rectArea(first.raw_bbox)) + std::sqrt(rectArea(second.raw_bbox)));
            float threshold_px = static_cast<float>(config.cluster_distance_scale) * average_symbol_size;
            threshold_px = std::clamp(
                threshold_px,
                static_cast<float>(config.min_cluster_distance_px),
                static_cast<float>(config.max_cluster_distance_px));

            const float height_similarity = similarityRatio(
                static_cast<float>(first.raw_bbox.height),
                static_cast<float>(second.raw_bbox.height));
            const float area_similarity = similarityRatio(
                rectArea(first.raw_bbox),
                rectArea(second.raw_bbox));
            const float avg_height = 0.5F * (
                static_cast<float>(first.raw_bbox.height) +
                static_cast<float>(second.raw_bbox.height));
            const float first_bottom = static_cast<float>(first.raw_bbox.y + first.raw_bbox.height);
            const float second_bottom = static_cast<float>(second.raw_bbox.y + second.raw_bbox.height);
            const float bottom_diff = std::abs(first_bottom - second_bottom);

            const bool allowed =
                center_distance <= threshold_px &&
                height_similarity >= static_cast<float>(config.min_height_similarity_for_grouping) &&
                area_similarity >= static_cast<float>(config.min_area_similarity_for_grouping) &&
                bottom_diff <= static_cast<float>(config.max_bottom_y_diff_ratio) * avg_height;

            if (allowed) {
                unite(static_cast<int>(i), static_cast<int>(j));
            }
        }
    }

    std::vector<std::vector<int>> grouped(symbols.size());
    for (std::size_t i = 0; i < symbols.size(); ++i) {
        grouped[find_root(static_cast<int>(i))].push_back(static_cast<int>(i));
    }

    std::vector<std::vector<int>> compact_groups;
    for (auto& group : grouped) {
        if (!group.empty()) {
            compact_groups.push_back(group);
        }
    }
    return compact_groups;
}

}  // namespace

std::string kfsGroupTypeToString(KFSGroupType group_type)
{
    switch (group_type) {
    case KFSGroupType::REAL:
        return "REAL";
    case KFSGroupType::FAKE:
        return "FAKE";
    case KFSGroupType::R1:
        return "R1";
    case KFSGroupType::AMBIGUOUS:
        return "AMBIGUOUS";
    case KFSGroupType::UNKNOWN:
    default:
        return "UNKNOWN";
    }
}

KFSGroupType classifySymbolGroup(const std::string& class_name)
{
    if (class_name == "R1") {
        return KFSGroupType::R1;
    }
    if (class_name.rfind("REAL_", 0) == 0) {
        return KFSGroupType::REAL;
    }
    if (class_name.rfind("FAKE_", 0) == 0) {
        return KFSGroupType::FAKE;
    }
    return KFSGroupType::UNKNOWN;
}

KFSInstanceAggregator::KFSInstanceAggregator(const KFSInstanceAggregatorConfig& config)
    : config_(config)
{
}

const std::vector<SymbolCandidate>& KFSInstanceAggregator::lastSymbols() const
{
    return last_symbols_;
}

const std::vector<KFSCluster>& KFSInstanceAggregator::lastClusters() const
{
    return last_clusters_;
}

const std::vector<KFSCluster>& KFSInstanceAggregator::lastDroppedClusters() const
{
    return last_dropped_clusters_;
}

const cv::Rect& KFSInstanceAggregator::lastEffectiveROI() const
{
    return last_effective_roi_;
}

std::vector<KFSInstance> KFSInstanceAggregator::aggregate(
    const cv::Mat& bgr_image,
    const std::vector<SymbolCandidate>& symbols)
{
    last_symbols_ = symbols;
    last_clusters_.clear();
    last_dropped_clusters_.clear();
    last_effective_roi_ = cv::Rect();

    std::vector<KFSInstance> instances;
    if (!config_.enable_aggregation || bgr_image.empty()) {
        return instances;
    }

    const int image_width = bgr_image.cols;
    const int image_height = bgr_image.rows;
    last_effective_roi_ = computeEffectiveROI(image_width, image_height, config_);

    for (auto& symbol : last_symbols_) {
        symbol.group_type = classifySymbolGroup(symbol.class_name);
        symbol.keep = true;
        symbol.drop_reason.clear();

        if (symbol.confidence < config_.min_confidence) {
            symbol.keep = false;
            symbol.drop_reason = "confidence_below_min";
        } else if (symbol.raw_bbox.height < config_.min_symbol_height_px) {
            symbol.keep = false;
            symbol.drop_reason = "bbox_height_below_min";
        } else if (rectArea(symbol.raw_bbox) < config_.min_symbol_area_px) {
            symbol.keep = false;
            symbol.drop_reason = "bbox_area_below_min";
        } else if (!pointInsideNormalizedBox(rectCenter(symbol.raw_bbox), image_width, image_height, config_)) {
            symbol.keep = false;
            symbol.drop_reason = "outside_roi";
        }

        if (config_.use_square_symbol_bbox) {
            symbol.geometry_bbox = makeSquareBBox(
                symbol.raw_bbox,
                image_width,
                image_height,
                config_.square_symbol_scale,
                config_.max_square_side_ratio_of_image);
        } else {
            symbol.geometry_bbox = symbol.raw_bbox;
        }
    }

    std::vector<SymbolCandidate> kept_symbols;
    kept_symbols.reserve(last_symbols_.size());
    for (const auto& symbol : last_symbols_) {
        if (symbol.keep) {
            kept_symbols.push_back(symbol);
        }
    }

    if (kept_symbols.empty()) {
        return instances;
    }

    const auto grouped_indices = clusterSymbols(kept_symbols, config_);
    std::vector<std::vector<SymbolCandidate>> groups;
    for (const auto& group_indices : grouped_indices) {
        std::vector<SymbolCandidate> group;
        group.reserve(group_indices.size());
        for (const int index : group_indices) {
            group.push_back(kept_symbols[static_cast<std::size_t>(index)]);
        }
        groups.push_back(group);
    }

    auto build_cluster = [&](int cluster_id,
                             const std::vector<SymbolCandidate>& group,
                             const std::vector<int>& merged_from) {
        KFSCluster cluster;
        cluster.cluster_id = cluster_id;
        cluster.symbol_indices.reserve(group.size());
        cluster.class_names.reserve(group.size());
        for (const auto& symbol : group) {
            cluster.symbol_indices.push_back(symbol.index);
            cluster.class_names.push_back(symbol.class_name);
        }
        std::sort(cluster.symbol_indices.begin(), cluster.symbol_indices.end());
        cluster.ambiguous = static_cast<int>(group.size()) > config_.max_symbols_per_instance;
        cluster.ambiguous_reason = cluster.ambiguous ?
            "too_many_symbols_possible_merged_kfs" : "";
        cluster.group_type = aggregateGroupType(group, cluster.ambiguous);
        cluster.union_bbox = unionRect(group, config_.use_square_symbol_bbox);
        cluster.expanded_bbox = expandRectWithLimit(
            cluster.union_bbox,
            image_width,
            image_height,
            config_);
        cluster.refined_bbox = cluster.expanded_bbox;
        cluster.color_mask_coverage = 0.0F;
        cluster.bbox_quality = "normal";
        cluster.merged_from = merged_from;
        cluster.is_merged_cluster = merged_from.size() > 1;

        const cv::Rect crop_rect = clampRect(cluster.expanded_bbox, image_width, image_height);
        if (!crop_rect.empty()) {
            const cv::Mat crop = bgr_image(crop_rect).clone();
            cv::Mat hsv;
            cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);

            const auto masks = selectColorMasks(hsv, config_);
            cv::Mat color_mask = masks.front().clone();
            for (std::size_t i = 1; i < masks.size(); ++i) {
                cv::bitwise_or(color_mask, masks[i], color_mask);
            }
            color_mask = applyMorphology(color_mask, config_);
            cluster.color_mask_coverage =
                static_cast<float>(cv::countNonZero(color_mask)) /
                static_cast<float>(std::max(1, color_mask.rows * color_mask.cols));

            if (config_.contour_enabled) {
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(
                    color_mask,
                    contours,
                    cv::RETR_EXTERNAL,
                    cv::CHAIN_APPROX_SIMPLE);

                std::vector<std::vector<cv::Point>> valid_contours;
                for (const auto& contour : contours) {
                    if (cv::contourArea(contour) >= config_.min_contour_area_px) {
                        valid_contours.push_back(contour);
                    }
                }

                if (!valid_contours.empty()) {
                    std::vector<cv::Point> selected = valid_contours.front();
                    if (config_.contour_selection == "largest") {
                        selected = *std::max_element(
                            valid_contours.begin(),
                            valid_contours.end(),
                            [](const auto& left, const auto& right) {
                                return cv::contourArea(left) < cv::contourArea(right);
                            });
                    } else {
                        const cv::Point2f union_center = rectCenter(cluster.union_bbox) -
                            cv::Point2f(static_cast<float>(crop_rect.x), static_cast<float>(crop_rect.y));
                        selected = *std::min_element(
                            valid_contours.begin(),
                            valid_contours.end(),
                            [&union_center](const auto& left, const auto& right) {
                                const auto left_center = contourCentroid(left);
                                const auto right_center = contourCentroid(right);
                                return cv::norm(left_center - union_center) <
                                       cv::norm(right_center - union_center);
                            });
                    }

                    const cv::Rect local_bbox = cv::boundingRect(selected);
                    cv::Rect refined(
                        crop_rect.x + local_bbox.x,
                        crop_rect.y + local_bbox.y,
                        local_bbox.width,
                        local_bbox.height);

                    const float union_diag = std::hypot(
                        static_cast<float>(cluster.union_bbox.width),
                        static_cast<float>(cluster.union_bbox.height));
                    const float center_offset = cv::norm(
                        rectCenter(refined) - rectCenter(cluster.union_bbox));
                    const float refined_area = rectArea(refined);
                    const float union_area = rectArea(cluster.union_bbox);
                    if (center_offset <= config_.contour_max_center_offset_ratio * union_diag &&
                        refined_area >= config_.contour_min_refined_to_union_area_ratio * union_area) {
                        cluster.refined_bbox = clampRect(refined, image_width, image_height);
                    }
                }
            }
        }

        const auto update_edge_quality = [&](const cv::Rect& rect) {
            return rect.x <= config_.edge_clip_margin_px ||
                   rect.y <= config_.edge_clip_margin_px ||
                   rect.x + rect.width >= image_width - config_.edge_clip_margin_px ||
                   rect.y + rect.height >= image_height - config_.edge_clip_margin_px;
        };
        if (update_edge_quality(cluster.union_bbox) || update_edge_quality(cluster.refined_bbox)) {
            cluster.bbox_quality = "partial_visible";
        }

        return cluster;
    };

    std::vector<KFSCluster> clusters;
    std::vector<std::vector<SymbolCandidate>> cluster_symbols = groups;
    for (std::size_t i = 0; i < groups.size(); ++i) {
        clusters.push_back(build_cluster(
            static_cast<int>(i),
            groups[i],
            {static_cast<int>(i)}));
    }

    if (config_.enable_basic_merge) {
        bool merged_any = true;
        while (merged_any) {
            merged_any = false;
            bool has_best_pair = false;
            std::pair<int, int> best_pair(0, 0);
            MergeCandidate best_candidate;
            float best_score = 0.0F;

            for (std::size_t i = 0; i < clusters.size(); ++i) {
                for (std::size_t j = i + 1; j < clusters.size(); ++j) {
                    const auto& first = clusters[i];
                    const auto& second = clusters[j];
                    const std::vector<SymbolCandidate> merged_symbols =
                        [&]() {
                            auto result = cluster_symbols[i];
                            result.insert(
                                result.end(),
                                cluster_symbols[j].begin(),
                                cluster_symbols[j].end());
                            return result;
                        }();

                    MergeCandidate candidate;
                    candidate.iou = bboxIou(first.expanded_bbox, second.expanded_bbox);
                    candidate.intersection_over_min_area =
                        bboxIntersectionOverMinArea(first.expanded_bbox, second.expanded_bbox);
                    candidate.gap_px = bboxGapPx(first.expanded_bbox, second.expanded_bbox);

                    if (first.ambiguous || second.ambiguous) {
                        candidate.reason = "ambiguous_cluster_not_mergeable";
                    } else if (!semanticGroupsCompatible(
                                   first.group_type,
                                   second.group_type,
                                   config_.merge_same_group_only)) {
                        candidate.reason = "semantic_groups_incompatible";
                    } else if (static_cast<int>(merged_symbols.size()) > config_.max_symbols_after_merge) {
                        candidate.reason = "too_many_symbols_after_merge";
                    } else {
                        const cv::Rect merged_union = unionRect(merged_symbols, config_.use_square_symbol_bbox);
                        float largest_symbol_width = 1.0F;
                        float largest_symbol_height = 1.0F;
                        for (const auto& symbol : merged_symbols) {
                            largest_symbol_width = std::max(
                                largest_symbol_width,
                                static_cast<float>(std::max(1, symbol.geometry_bbox.width)));
                            largest_symbol_height = std::max(
                                largest_symbol_height,
                                static_cast<float>(std::max(1, symbol.geometry_bbox.height)));
                        }

                        const float width_scale =
                            static_cast<float>(merged_union.width) / largest_symbol_width;
                        const float height_scale =
                            static_cast<float>(merged_union.height) / largest_symbol_height;
                        const bool compactness_pass =
                            width_scale <= config_.max_merged_union_width_scale &&
                            height_scale <= config_.max_merged_union_height_scale;
                        const bool strong_spatial_pass =
                            candidate.iou >= config_.min_expanded_iou ||
                            candidate.intersection_over_min_area >=
                                config_.min_expanded_intersection_over_min_area ||
                            candidate.gap_px <= config_.max_expanded_gap_px;

                        if (first.color_mask_coverage < config_.min_color_mask_coverage_for_merge ||
                            second.color_mask_coverage < config_.min_color_mask_coverage_for_merge) {
                            candidate.reason =
                                first.color_mask_coverage < config_.min_color_mask_coverage_for_merge
                                    ? "first_cluster_color_mask_too_low"
                                    : "second_cluster_color_mask_too_low";
                        } else if (!compactness_pass) {
                            candidate.reason = "merged_union_not_compact";
                        } else if (!strong_spatial_pass) {
                            candidate.reason = "spatial_conditions_not_met";
                        } else {
                            candidate.allowed = true;
                            candidate.reason = "basic_same_group_merge";
                        }

                        if (!candidate.allowed &&
                            config_.enable_low_mask_adjacent_same_group_fallback &&
                            (candidate.reason == "first_cluster_color_mask_too_low" ||
                             candidate.reason == "second_cluster_color_mask_too_low") &&
                            first.group_type == second.group_type) {
                            const float first_height = static_cast<float>(std::max(1, first.union_bbox.height));
                            const float second_height = static_cast<float>(std::max(1, second.union_bbox.height));
                            const float height_similarity = similarityRatio(first_height, second_height);
                            const float avg_height = 0.5F * (first_height + second_height);
                            const float first_bottom = static_cast<float>(first.union_bbox.y + first.union_bbox.height);
                            const float second_bottom = static_cast<float>(second.union_bbox.y + second.union_bbox.height);
                            const float bottom_diff = std::abs(first_bottom - second_bottom);
                            const float gap_limit = std::min(
                                config_.low_mask_max_gap_px,
                                config_.low_mask_max_gap_ratio_to_symbol_height * avg_height);
                            const cv::Rect merged_union = unionRect(
                                merged_symbols,
                                config_.use_square_symbol_bbox);
                            const float merged_width = static_cast<float>(std::max(1, merged_union.width));
                            const float merged_height = static_cast<float>(std::max(1, merged_union.height));
                            const float merged_aspect_ratio =
                                std::max(merged_width / merged_height, merged_height / merged_width);
                            const float largest_symbol_width = std::max(
                                static_cast<float>(std::max(1, merged_symbols[0].geometry_bbox.width)),
                                static_cast<float>(std::max(1, merged_symbols[0].geometry_bbox.height)));
                            (void)largest_symbol_width;

                            if (height_similarity >= config_.low_mask_min_height_similarity &&
                                bottom_diff <= config_.low_mask_max_bottom_y_diff_ratio * avg_height &&
                                candidate.gap_px <= gap_limit &&
                                merged_aspect_ratio <= config_.low_mask_max_merged_aspect_ratio &&
                                width_scale <= config_.low_mask_max_merged_width_scale &&
                                height_scale <= config_.low_mask_max_merged_height_scale) {
                                candidate.allowed = true;
                                candidate.used_low_mask_fallback = true;
                                candidate.reason = "low_mask_adjacent_same_group_fallback";
                            }
                        }
                    }

                    if (!candidate.allowed) {
                        continue;
                    }

                    const float score =
                        candidate.iou +
                        candidate.intersection_over_min_area -
                        0.01F * candidate.gap_px;
                    if (!has_best_pair || score > best_score) {
                        best_pair = std::make_pair(static_cast<int>(i), static_cast<int>(j));
                        best_candidate = candidate;
                        best_score = score;
                        has_best_pair = true;
                    }
                }
            }

            if (!has_best_pair) {
                break;
            }

            merged_any = true;
            const int left = best_pair.first;
            const int right = best_pair.second;

            auto merged_symbols = cluster_symbols[left];
            merged_symbols.insert(
                merged_symbols.end(),
                cluster_symbols[right].begin(),
                cluster_symbols[right].end());
            std::vector<int> merged_from = clusters[left].merged_from;
            merged_from.insert(
                merged_from.end(),
                clusters[right].merged_from.begin(),
                clusters[right].merged_from.end());
            std::sort(merged_from.begin(), merged_from.end());
            merged_from.erase(std::unique(merged_from.begin(), merged_from.end()), merged_from.end());

            clusters[left] = build_cluster(clusters[left].cluster_id, merged_symbols, merged_from);
            cluster_symbols[left] = std::move(merged_symbols);
            clusters.erase(clusters.begin() + right);
            cluster_symbols.erase(cluster_symbols.begin() + right);
            (void)best_candidate;
        }
    }

    if (config_.neighbor_aware_clamp && config_.protect_other_cluster_symbols) {
        for (std::size_t i = 0; i < clusters.size(); ++i) {
            auto& cluster = clusters[i];
            cv::Rect current = cluster.refined_bbox;
            const cv::Point2f own_center = rectCenter(cluster.union_bbox);

            for (std::size_t j = 0; j < clusters.size(); ++j) {
                if (i == j) {
                    continue;
                }

                cv::Rect protected_bbox = unionRect(
                    cluster_symbols[j],
                    config_.neighbor_protection_bbox_source == "geometry");
                const cv::Rect intersection = current & protected_bbox;
                if (intersection.empty()) {
                    continue;
                }

                const cv::Point2f a = rectCenter(cluster.union_bbox);
                const cv::Point2f b = rectCenter(clusters[j].union_bbox);
                cv::Rect candidate = current;
                if (std::abs(a.x - b.x) >= std::abs(a.y - b.y)) {
                    if (a.x < b.x) {
                        candidate.width = std::max(
                            1,
                            protected_bbox.x - config_.neighbor_overlap_margin_px - candidate.x);
                    } else {
                        const int new_x = protected_bbox.x + protected_bbox.width + config_.neighbor_overlap_margin_px;
                        candidate.width = std::max(1, candidate.x + candidate.width - new_x);
                        candidate.x = new_x;
                    }
                } else {
                    if (a.y < b.y) {
                        candidate.height = std::max(
                            1,
                            protected_bbox.y - config_.neighbor_overlap_margin_px - candidate.y);
                    } else {
                        const int new_y = protected_bbox.y + protected_bbox.height + config_.neighbor_overlap_margin_px;
                        candidate.height = std::max(1, candidate.y + candidate.height - new_y);
                        candidate.y = new_y;
                    }
                }

                candidate = clampRect(candidate, image_width, image_height);
                if (validNeighborClampBBox(candidate, own_center)) {
                    current = candidate;
                }
            }

            cluster.refined_bbox = current;
        }
    }

    for (auto& cluster : clusters) {
        if (cluster.group_type == KFSGroupType::AMBIGUOUS && config_.drop_ambiguous_clusters) {
            last_dropped_clusters_.push_back(cluster);
            continue;
        }
        last_clusters_.push_back(cluster);
    }

    for (const auto& cluster : last_clusters_) {
        KFSInstance instance;
        instance.cluster_id = cluster.cluster_id;
        instance.symbol_indices = cluster.symbol_indices;
        instance.class_names = cluster.class_names;
        instance.group_type = cluster.group_type;
        instance.refined_bbox = cluster.refined_bbox;
        instance.bbox_quality = cluster.bbox_quality;
        instance.color_mask_coverage = cluster.color_mask_coverage;
        instances.push_back(instance);
    }

    return instances;
}

}  // namespace abu_yolo_ros
