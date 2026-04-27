#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

#include "rclcpp/rclcpp.hpp"

#include "abu_yolo_ros/msg/localized_kfs_instance.hpp"
#include "abu_yolo_ros/msg/localized_kfs_instance_array.hpp"

namespace {

constexpr double kEpsilon = 1e-6;
constexpr double kRadToDeg = 180.0 / M_PI;

bool isFinite(double value)
{
    return std::isfinite(value);
}

double stampToSeconds(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1e-9;
}

bool isLocalizedOk(const abu_yolo_ros::msg::LocalizedKfsInstance& instance)
{
    return instance.localized &&
           instance.localization_status == "ok" &&
           isFinite(instance.position_robot_mm.x) &&
           isFinite(instance.position_robot_mm.y);
}

std::string pointToString(double x, double y)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(1)
           << x << "," << y;
    return stream.str();
}

}  // namespace

class KfsLocalizationStabilizerNode : public rclcpp::Node {
public:
    KfsLocalizationStabilizerNode()
        : Node("kfs_localization_stabilizer_node")
    {
        const auto qos = rclcpp::SensorDataQoS();

        this->declare_parameter<bool>("kfs_localization_stabilizer.enabled", true);
        this->declare_parameter<std::string>(
            "kfs_localization_stabilizer.input_topic",
            "/yolo/kfs_instances_localized");
        this->declare_parameter<std::string>(
            "kfs_localization_stabilizer.output_topic",
            "/yolo/kfs_instances_stabilized");
        this->declare_parameter<bool>("kfs_localization_stabilizer.debug", true);

        this->declare_parameter<std::string>(
            "kfs_localization_stabilizer.filter.model",
            "kalman_cv_2d");
        this->declare_parameter<double>(
            "kfs_localization_stabilizer.filter.process_noise_pos",
            25.0);
        this->declare_parameter<double>(
            "kfs_localization_stabilizer.filter.process_noise_vel",
            100.0);
        this->declare_parameter<double>(
            "kfs_localization_stabilizer.filter.measurement_noise_pos",
            100.0);
        this->declare_parameter<double>(
            "kfs_localization_stabilizer.filter.initial_position_variance",
            400.0);
        this->declare_parameter<double>(
            "kfs_localization_stabilizer.filter.initial_velocity_variance",
            1000.0);
        this->declare_parameter<double>(
            "kfs_localization_stabilizer.filter.max_dt_sec",
            0.2);
        this->declare_parameter<double>(
            "kfs_localization_stabilizer.filter.min_dt_sec",
            0.001);

        this->declare_parameter<double>(
            "kfs_localization_stabilizer.gating.max_association_distance_mm",
            350.0);
        this->declare_parameter<double>(
            "kfs_localization_stabilizer.gating.max_jump_distance_mm",
            600.0);
        this->declare_parameter<bool>(
            "kfs_localization_stabilizer.gating.require_group_type_match",
            true);
        this->declare_parameter<bool>(
            "kfs_localization_stabilizer.gating.require_decision_compatibility",
            false);
        this->declare_parameter<int>(
            "kfs_localization_stabilizer.gating.min_class_name_overlap",
            0);

        this->declare_parameter<int>(
            "kfs_localization_stabilizer.track_lifecycle.max_missed_frames",
            10);
        this->declare_parameter<double>(
            "kfs_localization_stabilizer.track_lifecycle.max_track_age_sec_without_update",
            1.0);
        this->declare_parameter<int>(
            "kfs_localization_stabilizer.track_lifecycle.min_hits_for_stable",
            2);

        this->declare_parameter<bool>(
            "kfs_localization_stabilizer.output.publish_unlocalized_instances",
            true);
        this->declare_parameter<bool>(
            "kfs_localization_stabilizer.output.pass_through_failed_instances",
            true);
        this->declare_parameter<bool>(
            "kfs_localization_stabilizer.output.append_quality_suffix",
            true);

        this->get_parameter("kfs_localization_stabilizer.enabled", enabled_);
        this->get_parameter("kfs_localization_stabilizer.input_topic", input_topic_);
        this->get_parameter("kfs_localization_stabilizer.output_topic", output_topic_);
        this->get_parameter("kfs_localization_stabilizer.debug", debug_);

        this->get_parameter("kfs_localization_stabilizer.filter.model", filter_model_);
        this->get_parameter(
            "kfs_localization_stabilizer.filter.process_noise_pos",
            process_noise_pos_);
        this->get_parameter(
            "kfs_localization_stabilizer.filter.process_noise_vel",
            process_noise_vel_);
        this->get_parameter(
            "kfs_localization_stabilizer.filter.measurement_noise_pos",
            measurement_noise_pos_);
        this->get_parameter(
            "kfs_localization_stabilizer.filter.initial_position_variance",
            initial_position_variance_);
        this->get_parameter(
            "kfs_localization_stabilizer.filter.initial_velocity_variance",
            initial_velocity_variance_);
        this->get_parameter(
            "kfs_localization_stabilizer.filter.max_dt_sec",
            max_dt_sec_);
        this->get_parameter(
            "kfs_localization_stabilizer.filter.min_dt_sec",
            min_dt_sec_);

        this->get_parameter(
            "kfs_localization_stabilizer.gating.max_association_distance_mm",
            max_association_distance_mm_);
        this->get_parameter(
            "kfs_localization_stabilizer.gating.max_jump_distance_mm",
            max_jump_distance_mm_);
        this->get_parameter(
            "kfs_localization_stabilizer.gating.require_group_type_match",
            require_group_type_match_);
        this->get_parameter(
            "kfs_localization_stabilizer.gating.require_decision_compatibility",
            require_decision_compatibility_);
        this->get_parameter(
            "kfs_localization_stabilizer.gating.min_class_name_overlap",
            min_class_name_overlap_);

        this->get_parameter(
            "kfs_localization_stabilizer.track_lifecycle.max_missed_frames",
            max_missed_frames_);
        this->get_parameter(
            "kfs_localization_stabilizer.track_lifecycle.max_track_age_sec_without_update",
            max_track_age_sec_without_update_);
        this->get_parameter(
            "kfs_localization_stabilizer.track_lifecycle.min_hits_for_stable",
            min_hits_for_stable_);

        this->get_parameter(
            "kfs_localization_stabilizer.output.publish_unlocalized_instances",
            publish_unlocalized_instances_);
        this->get_parameter(
            "kfs_localization_stabilizer.output.pass_through_failed_instances",
            pass_through_failed_instances_);
        this->get_parameter(
            "kfs_localization_stabilizer.output.append_quality_suffix",
            append_quality_suffix_);

        if (filter_model_ != "kalman_cv_2d") {
            RCLCPP_WARN(
                this->get_logger(),
                "Unsupported filter model '%s'; falling back to kalman_cv_2d",
                filter_model_.c_str());
            filter_model_ = "kalman_cv_2d";
        }

        if (min_dt_sec_ <= 0.0) {
            min_dt_sec_ = 0.001;
        }
        if (max_dt_sec_ < min_dt_sec_) {
            max_dt_sec_ = min_dt_sec_;
        }
        if (max_missed_frames_ < 0) {
            max_missed_frames_ = 0;
        }
        if (min_hits_for_stable_ < 1) {
            min_hits_for_stable_ = 1;
        }

        sub_ = this->create_subscription<abu_yolo_ros::msg::LocalizedKfsInstanceArray>(
            input_topic_,
            qos,
            std::bind(
                &KfsLocalizationStabilizerNode::localizedCallback,
                this,
                std::placeholders::_1));
        pub_ = this->create_publisher<abu_yolo_ros::msg::LocalizedKfsInstanceArray>(
            output_topic_,
            qos);

        RCLCPP_INFO(
            this->get_logger(),
            "KFS localization stabilizer: %s | input=%s | output=%s | model=%s",
            enabled_ ? "enabled" : "disabled",
            input_topic_.c_str(),
            output_topic_.c_str(),
            filter_model_.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "KFS localization stabilizer gating: assoc=%.1fmm | jump=%.1fmm | min_hits=%d",
            max_association_distance_mm_,
            max_jump_distance_mm_,
            min_hits_for_stable_);
    }

private:
    struct Track {
        int track_id = 0;
        cv::KalmanFilter kf;
        double last_update_sec = 0.0;
        int missed_frames = 0;
        int hits = 0;
        std::string group_type;
        std::string decision;
        std::vector<std::string> class_names;
        int last_cluster_id = -1;
        cv::Point2f last_smoothed_xy{0.0F, 0.0F};
    };

    struct PredictedTrack {
        std::size_t track_index = 0;
        cv::Point2f predicted_xy{0.0F, 0.0F};
        double dt_sec = 0.0;
    };

    struct OutputLogEntry {
        int cluster_id = -1;
        bool localized = false;
        bool pass_through_failed = false;
        std::string failure_reason;
        cv::Point2f raw_xy{0.0F, 0.0F};
        cv::Point2f smooth_xy{0.0F, 0.0F};
        float distance_mm = 0.0F;
        float bearing_deg = 0.0F;
        int track_id = -1;
        int hits = 0;
        std::string quality;
    };

    void localizedCallback(
        const abu_yolo_ros::msg::LocalizedKfsInstanceArray::SharedPtr msg)
    {
        if (!enabled_) {
            return;
        }

        const double frame_time_sec = frameTimeSeconds(msg->header);
        predictTracks(frame_time_sec);

        abu_yolo_ros::msg::LocalizedKfsInstanceArray output;
        output.header = msg->header;
        output.team_color = msg->team_color;
        std::vector<bool> track_used(tracks_.size(), false);
        std::vector<OutputLogEntry> log_entries;
        log_entries.reserve(msg->instances.size());

        if (msg->instances.empty()) {
            cleanupTracks(frame_time_sec, track_used);
            pub_->publish(output);
            maybeLogOutput(output, log_entries);
            return;
        }

        for (const auto& instance : msg->instances) {
            if (!isLocalizedOk(instance)) {
                if (publish_unlocalized_instances_ &&
                    pass_through_failed_instances_) {
                    output.instances.push_back(instance);
                    OutputLogEntry entry;
                    entry.cluster_id = instance.source_instance.cluster_id;
                    entry.localized = false;
                    entry.pass_through_failed = true;
                    entry.failure_reason = instance.failure_reason;
                    log_entries.push_back(entry);
                }
                continue;
            }

            const cv::Point2f measurement_xy(
                static_cast<float>(instance.position_robot_mm.x),
                static_cast<float>(instance.position_robot_mm.y));

            const int matched_track_index =
                findBestTrack(instance, measurement_xy, track_used);

            abu_yolo_ros::msg::LocalizedKfsInstance stabilized = instance;
            OutputLogEntry entry;
            entry.cluster_id = instance.source_instance.cluster_id;
            entry.localized = true;
            entry.raw_xy = measurement_xy;

            if (matched_track_index >= 0) {
                Track& track = tracks_[static_cast<std::size_t>(matched_track_index)];
                const double jump_distance =
                    cv::norm(measurement_xy - track.last_smoothed_xy);
                if (jump_distance > max_jump_distance_mm_) {
                    stabilized.localization_quality =
                        updateQualityString(
                            instance.localization_quality,
                            "raw_gating_rejected");
                    const int new_track_index =
                        createTrack(instance, measurement_xy, frame_time_sec);
                    Track& new_track = tracks_[static_cast<std::size_t>(new_track_index)];
                    track_used[static_cast<std::size_t>(new_track_index)] = true;
                    applySmoothedPosition(stabilized, measurement_xy, static_cast<float>(instance.position_robot_mm.z));
                    entry.smooth_xy = measurement_xy;
                    entry.track_id = new_track.track_id;
                    entry.hits = new_track.hits;
                    entry.quality = stabilized.localization_quality;
                } else {
                    correctTrack(track, measurement_xy, instance, frame_time_sec);
                    track_used[static_cast<std::size_t>(matched_track_index)] = true;
                    applySmoothedPosition(stabilized, track.last_smoothed_xy, static_cast<float>(instance.position_robot_mm.z));
                    const std::string quality_suffix =
                        track.hits >= min_hits_for_stable_ ?
                            "stabilized_kalman" :
                            "stabilizing_kalman";
                    stabilized.localization_quality =
                        updateQualityString(
                            instance.localization_quality,
                            quality_suffix);
                    entry.smooth_xy = track.last_smoothed_xy;
                    entry.track_id = track.track_id;
                    entry.hits = track.hits;
                    entry.quality = stabilized.localization_quality;
                }
            } else {
                const int new_track_index =
                    createTrack(instance, measurement_xy, frame_time_sec);
                Track& track = tracks_[static_cast<std::size_t>(new_track_index)];
                if (track_used.size() < tracks_.size()) {
                    track_used.resize(tracks_.size(), false);
                }
                track_used[static_cast<std::size_t>(new_track_index)] = true;
                applySmoothedPosition(stabilized, measurement_xy, static_cast<float>(instance.position_robot_mm.z));
                stabilized.localization_quality =
                    updateQualityString(
                        instance.localization_quality,
                        "stabilizing_kalman");
                entry.smooth_xy = measurement_xy;
                entry.track_id = track.track_id;
                entry.hits = track.hits;
                entry.quality = stabilized.localization_quality;
            }

            entry.distance_mm = stabilized.distance_mm;
            entry.bearing_deg = stabilized.bearing_deg;
            output.instances.push_back(stabilized);
            log_entries.push_back(entry);
        }

        cleanupTracks(frame_time_sec, track_used);
        pub_->publish(output);
        maybeLogOutput(output, log_entries);
    }

    double frameTimeSeconds(const std_msgs::msg::Header& header) const
    {
        const double stamp_sec = stampToSeconds(header.stamp);
        if (stamp_sec > 0.0) {
            return stamp_sec;
        }
        return this->now().seconds();
    }

    void predictTracks(double frame_time_sec)
    {
        predicted_tracks_.clear();
        predicted_tracks_.reserve(tracks_.size());

        for (std::size_t i = 0; i < tracks_.size(); ++i) {
            Track& track = tracks_[i];
            const double dt_sec = computeDt(frame_time_sec - track.last_update_sec);
            updateTransitionMatrix(track.kf, dt_sec);
            const cv::Mat prediction = track.kf.predict();
            PredictedTrack predicted;
            predicted.track_index = i;
            predicted.dt_sec = dt_sec;
            predicted.predicted_xy = cv::Point2f(
                prediction.at<float>(0),
                prediction.at<float>(1));
            predicted_tracks_.push_back(predicted);
        }
    }

    double computeDt(double raw_dt_sec) const
    {
        if (!std::isfinite(raw_dt_sec)) {
            return min_dt_sec_;
        }
        return std::clamp(raw_dt_sec, min_dt_sec_, max_dt_sec_);
    }

    void updateTransitionMatrix(cv::KalmanFilter& kf, double dt_sec) const
    {
        kf.transitionMatrix = (cv::Mat_<float>(4, 4) <<
            1.0F, 0.0F, static_cast<float>(dt_sec), 0.0F,
            0.0F, 1.0F, 0.0F, static_cast<float>(dt_sec),
            0.0F, 0.0F, 1.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 1.0F);
    }

    int findBestTrack(
        const abu_yolo_ros::msg::LocalizedKfsInstance& instance,
        const cv::Point2f& measurement_xy,
        const std::vector<bool>& track_used) const
    {
        int best_index = -1;
        bool best_cluster_match = false;
        double best_distance = std::numeric_limits<double>::max();

        for (const auto& predicted : predicted_tracks_) {
            if (predicted.track_index >= track_used.size() ||
                track_used[predicted.track_index]) {
                continue;
            }

            const Track& track = tracks_[predicted.track_index];
            if (require_group_type_match_ &&
                track.group_type != instance.source_instance.group_type) {
                continue;
            }
            if (require_decision_compatibility_ &&
                track.decision != instance.source_instance.decision) {
                continue;
            }
            if (classNameOverlap(track.class_names, instance.source_instance.class_names) <
                min_class_name_overlap_) {
                continue;
            }

            const double distance =
                cv::norm(predicted.predicted_xy - measurement_xy);
            if (distance > max_association_distance_mm_) {
                continue;
            }

            const bool cluster_match =
                track.last_cluster_id == instance.source_instance.cluster_id;
            if (best_index < 0 ||
                (cluster_match && !best_cluster_match) ||
                (cluster_match == best_cluster_match && distance < best_distance)) {
                best_index = static_cast<int>(predicted.track_index);
                best_cluster_match = cluster_match;
                best_distance = distance;
            }
        }

        return best_index;
    }

    int classNameOverlap(
        const std::vector<std::string>& left,
        const std::vector<std::string>& right) const
    {
        int overlap = 0;
        for (const auto& left_name : left) {
            for (const auto& right_name : right) {
                if (left_name == right_name) {
                    ++overlap;
                    break;
                }
            }
        }
        return overlap;
    }

    int createTrack(
        const abu_yolo_ros::msg::LocalizedKfsInstance& instance,
        const cv::Point2f& measurement_xy,
        double frame_time_sec)
    {
        Track track;
        track.track_id = next_track_id_++;
        track.group_type = instance.source_instance.group_type;
        track.decision = instance.source_instance.decision;
        track.class_names = instance.source_instance.class_names;
        track.last_cluster_id = instance.source_instance.cluster_id;
        track.last_update_sec = frame_time_sec;
        track.hits = 1;
        track.missed_frames = 0;
        track.last_smoothed_xy = measurement_xy;

        initializeKalmanFilter(track.kf, measurement_xy);

        tracks_.push_back(track);
        predicted_tracks_.push_back(
            PredictedTrack{
                tracks_.size() - 1U,
                measurement_xy,
                min_dt_sec_});
        return static_cast<int>(tracks_.size() - 1U);
    }

    void initializeKalmanFilter(
        cv::KalmanFilter& kf,
        const cv::Point2f& measurement_xy) const
    {
        kf.init(4, 2, 0, CV_32F);
        kf.measurementMatrix = (cv::Mat_<float>(2, 4) <<
            1.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 1.0F, 0.0F, 0.0F);
        updateTransitionMatrix(kf, min_dt_sec_);
        kf.processNoiseCov = cv::Mat::zeros(4, 4, CV_32F);
        kf.processNoiseCov.at<float>(0, 0) = static_cast<float>(process_noise_pos_);
        kf.processNoiseCov.at<float>(1, 1) = static_cast<float>(process_noise_pos_);
        kf.processNoiseCov.at<float>(2, 2) = static_cast<float>(process_noise_vel_);
        kf.processNoiseCov.at<float>(3, 3) = static_cast<float>(process_noise_vel_);
        kf.measurementNoiseCov = cv::Mat::zeros(2, 2, CV_32F);
        kf.measurementNoiseCov.at<float>(0, 0) = static_cast<float>(measurement_noise_pos_);
        kf.measurementNoiseCov.at<float>(1, 1) = static_cast<float>(measurement_noise_pos_);
        kf.errorCovPost = cv::Mat::zeros(4, 4, CV_32F);
        kf.errorCovPost.at<float>(0, 0) = static_cast<float>(initial_position_variance_);
        kf.errorCovPost.at<float>(1, 1) = static_cast<float>(initial_position_variance_);
        kf.errorCovPost.at<float>(2, 2) = static_cast<float>(initial_velocity_variance_);
        kf.errorCovPost.at<float>(3, 3) = static_cast<float>(initial_velocity_variance_);
        kf.statePost = (cv::Mat_<float>(4, 1) <<
            measurement_xy.x, measurement_xy.y, 0.0F, 0.0F);
    }

    void correctTrack(
        Track& track,
        const cv::Point2f& measurement_xy,
        const abu_yolo_ros::msg::LocalizedKfsInstance& instance,
        double frame_time_sec) const
    {
        const cv::Mat measurement = (cv::Mat_<float>(2, 1) <<
            measurement_xy.x,
            measurement_xy.y);
        const cv::Mat corrected = track.kf.correct(measurement);
        track.last_smoothed_xy = cv::Point2f(
            corrected.at<float>(0),
            corrected.at<float>(1));
        track.last_update_sec = frame_time_sec;
        track.missed_frames = 0;
        track.hits += 1;
        track.group_type = instance.source_instance.group_type;
        track.decision = instance.source_instance.decision;
        track.class_names = instance.source_instance.class_names;
        track.last_cluster_id = instance.source_instance.cluster_id;
    }

    void applySmoothedPosition(
        abu_yolo_ros::msg::LocalizedKfsInstance& instance,
        const cv::Point2f& smoothed_xy,
        float raw_z) const
    {
        instance.position_robot_mm.x = smoothed_xy.x;
        instance.position_robot_mm.y = smoothed_xy.y;
        instance.position_robot_mm.z = raw_z;
        instance.distance_mm = static_cast<float>(std::hypot(smoothed_xy.x, smoothed_xy.y));
        instance.bearing_deg = static_cast<float>(std::atan2(smoothed_xy.y, smoothed_xy.x) * kRadToDeg);
        instance.localized = true;
        instance.localization_status = "ok";
    }

    std::string updateQualityString(
        const std::string& base_quality,
        const std::string& suffix) const
    {
        if (!append_quality_suffix_) {
            return suffix;
        }
        if (base_quality.empty()) {
            return suffix;
        }
        return base_quality + "+" + suffix;
    }

    void cleanupTracks(double frame_time_sec, const std::vector<bool>& track_used)
    {
        for (std::size_t i = 0; i < tracks_.size(); ++i) {
            if (i < track_used.size() && track_used[i]) {
                continue;
            }
            tracks_[i].missed_frames += 1;
        }

        tracks_.erase(
            std::remove_if(
                tracks_.begin(),
                tracks_.end(),
                [&](const Track& track) {
                    const double age_sec = frame_time_sec - track.last_update_sec;
                    return track.missed_frames > max_missed_frames_ ||
                           age_sec > max_track_age_sec_without_update_;
                }),
            tracks_.end());
    }

    void maybeLogOutput(
        const abu_yolo_ros::msg::LocalizedKfsInstanceArray& output,
        const std::vector<OutputLogEntry>& entries)
    {
        if (!debug_) {
            return;
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "KFS localization stabilizer count=%zu topic=%s tracks=%zu",
            output.instances.size(),
            output_topic_.c_str(),
            tracks_.size());

        for (const auto& entry : entries) {
            std::ostringstream stream;
            stream << "[[C" << entry.cluster_id << "]]";
            if (!entry.localized && entry.pass_through_failed) {
                stream << " status=failed pass_through reason=\""
                       << entry.failure_reason << "\"";
            } else if (entry.localized) {
                stream << " status=ok raw=("
                       << pointToString(entry.raw_xy.x, entry.raw_xy.y)
                       << ")mm smooth=("
                       << pointToString(entry.smooth_xy.x, entry.smooth_xy.y)
                       << ")mm dist=" << std::fixed << std::setprecision(1)
                       << entry.distance_mm
                       << " bearing=" << entry.bearing_deg
                       << " track=" << entry.track_id
                       << " hits=" << entry.hits
                       << " quality=" << entry.quality;
            } else {
                stream << " status=skipped";
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "%s",
                stream.str().c_str());
        }
    }

    bool enabled_ = true;
    bool debug_ = true;
    bool require_group_type_match_ = true;
    bool require_decision_compatibility_ = false;
    bool publish_unlocalized_instances_ = true;
    bool pass_through_failed_instances_ = true;
    bool append_quality_suffix_ = true;

    int min_class_name_overlap_ = 0;
    int max_missed_frames_ = 10;
    int min_hits_for_stable_ = 2;
    int next_track_id_ = 1;

    double process_noise_pos_ = 25.0;
    double process_noise_vel_ = 100.0;
    double measurement_noise_pos_ = 100.0;
    double initial_position_variance_ = 400.0;
    double initial_velocity_variance_ = 1000.0;
    double max_dt_sec_ = 0.2;
    double min_dt_sec_ = 0.001;
    double max_association_distance_mm_ = 350.0;
    double max_jump_distance_mm_ = 600.0;
    double max_track_age_sec_without_update_ = 1.0;

    std::string input_topic_;
    std::string output_topic_;
    std::string filter_model_;

    std::vector<Track> tracks_;
    std::vector<PredictedTrack> predicted_tracks_;

    rclcpp::Subscription<abu_yolo_ros::msg::LocalizedKfsInstanceArray>::SharedPtr sub_;
    rclcpp::Publisher<abu_yolo_ros::msg::LocalizedKfsInstanceArray>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KfsLocalizationStabilizerNode>());
    rclcpp::shutdown();
    return 0;
}
