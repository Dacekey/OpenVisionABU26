#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include "abu_yolo_ros/msg/kfs_instance_array.hpp"
#include "abu_yolo_ros/msg/localized_kfs_instance.hpp"
#include "abu_yolo_ros/msg/localized_kfs_instance_array.hpp"

namespace {

constexpr double kEpsilon = 1e-6;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

bool isFinite(double value)
{
    return std::isfinite(value);
}

cv::Matx33d rotationX(double radians)
{
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return cv::Matx33d(
        1.0, 0.0, 0.0,
        0.0, c, -s,
        0.0, s, c);
}

cv::Matx33d rotationY(double radians)
{
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return cv::Matx33d(
        c, 0.0, s,
        0.0, 1.0, 0.0,
        -s, 0.0, c);
}

cv::Matx33d rotationZ(double radians)
{
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return cv::Matx33d(
        c, -s, 0.0,
        s, c, 0.0,
        0.0, 0.0, 1.0);
}

cv::Vec3d normalizeVec3(const cv::Vec3d& vector)
{
    const double norm = cv::norm(vector);
    if (norm <= kEpsilon) {
        return cv::Vec3d(0.0, 0.0, 0.0);
    }
    return vector / norm;
}

std::string pointToString(const cv::Vec3d& point)
{
    std::ostringstream stream;
    stream << std::lround(point[0]) << ","
           << std::lround(point[1]) << ","
           << std::lround(point[2]);
    return stream.str();
}

}  // namespace

class Kfs3DLocalizerNode : public rclcpp::Node {
public:
    Kfs3DLocalizerNode()
        : Node("kfs_3d_localizer_node")
    {
        const auto qos = rclcpp::SensorDataQoS();

        this->declare_parameter<bool>("kfs_3d_localizer.enabled", true);
        this->declare_parameter<std::string>("kfs_3d_localizer.input_topic", "/yolo/kfs_instances");
        this->declare_parameter<std::string>("kfs_3d_localizer.output_topic", "/yolo/kfs_instances_localized");
        this->declare_parameter<bool>("kfs_3d_localizer.debug", true);

        this->declare_parameter<std::string>("kfs_3d_localizer.frames.robot_frame", "base_link");
        this->declare_parameter<std::string>("kfs_3d_localizer.frames.camera_frame", "camera");

        this->declare_parameter<int>("kfs_3d_localizer.image.width", 1280);
        this->declare_parameter<int>("kfs_3d_localizer.image.height", 720);

        this->declare_parameter<double>("kfs_3d_localizer.camera_matrix.fx", 900.0);
        this->declare_parameter<double>("kfs_3d_localizer.camera_matrix.fy", 900.0);
        this->declare_parameter<double>("kfs_3d_localizer.camera_matrix.cx", 640.0);
        this->declare_parameter<double>("kfs_3d_localizer.camera_matrix.cy", 360.0);

        this->declare_parameter<std::string>("kfs_3d_localizer.distortion.model", "none");
        this->declare_parameter<std::vector<double>>(
            "kfs_3d_localizer.distortion.pinhole_coeffs",
            std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>(
            "kfs_3d_localizer.distortion.fisheye_coeffs",
            std::vector<double>{0.0, 0.0, 0.0, 0.0});

        this->declare_parameter<double>("kfs_3d_localizer.camera_pose_robot.x_mm", 0.0);
        this->declare_parameter<double>("kfs_3d_localizer.camera_pose_robot.y_mm", 0.0);
        this->declare_parameter<double>("kfs_3d_localizer.camera_pose_robot.z_mm", 300.0);
        this->declare_parameter<double>("kfs_3d_localizer.camera_pose_robot.roll_deg", 0.0);
        this->declare_parameter<double>("kfs_3d_localizer.camera_pose_robot.pitch_deg", -20.0);
        this->declare_parameter<double>("kfs_3d_localizer.camera_pose_robot.yaw_deg", 0.0);

        this->declare_parameter<std::string>("kfs_3d_localizer.projection.primary_point", "bottom_center");
        this->declare_parameter<std::string>("kfs_3d_localizer.projection.fallback_point", "center");
        this->declare_parameter<bool>("kfs_3d_localizer.projection.clamp_to_image", true);
        this->declare_parameter<double>("kfs_3d_localizer.projection.bottom_center_y_offset_ratio", 0.0);

        this->declare_parameter<std::string>("kfs_3d_localizer.plane.mode", "fixed");
        this->declare_parameter<double>("kfs_3d_localizer.plane.default_z_height_mm", 0.0);
        this->declare_parameter<std::vector<double>>(
            "kfs_3d_localizer.plane.valid_z_heights_mm",
            std::vector<double>{0.0, 200.0, 400.0, 600.0});

        this->declare_parameter<double>("kfs_3d_localizer.validation.min_distance_mm", 100.0);
        this->declare_parameter<double>("kfs_3d_localizer.validation.max_distance_mm", 3000.0);
        this->declare_parameter<double>("kfs_3d_localizer.validation.max_abs_y_mm", 2000.0);
        this->declare_parameter<bool>("kfs_3d_localizer.validation.reject_if_ray_parallel", true);

        this->get_parameter("kfs_3d_localizer.enabled", enabled_);
        this->get_parameter("kfs_3d_localizer.input_topic", input_topic_);
        this->get_parameter("kfs_3d_localizer.output_topic", output_topic_);
        this->get_parameter("kfs_3d_localizer.debug", debug_);

        this->get_parameter("kfs_3d_localizer.frames.robot_frame", robot_frame_);
        this->get_parameter("kfs_3d_localizer.frames.camera_frame", camera_frame_);

        this->get_parameter("kfs_3d_localizer.image.width", image_width_);
        this->get_parameter("kfs_3d_localizer.image.height", image_height_);

        this->get_parameter("kfs_3d_localizer.camera_matrix.fx", fx_);
        this->get_parameter("kfs_3d_localizer.camera_matrix.fy", fy_);
        this->get_parameter("kfs_3d_localizer.camera_matrix.cx", cx_);
        this->get_parameter("kfs_3d_localizer.camera_matrix.cy", cy_);

        this->get_parameter("kfs_3d_localizer.distortion.model", distortion_model_);
        this->get_parameter("kfs_3d_localizer.distortion.pinhole_coeffs", pinhole_coeffs_);
        this->get_parameter("kfs_3d_localizer.distortion.fisheye_coeffs", fisheye_coeffs_);

        this->get_parameter("kfs_3d_localizer.camera_pose_robot.x_mm", camera_origin_robot_[0]);
        this->get_parameter("kfs_3d_localizer.camera_pose_robot.y_mm", camera_origin_robot_[1]);
        this->get_parameter("kfs_3d_localizer.camera_pose_robot.z_mm", camera_origin_robot_[2]);

        double roll_deg = 0.0;
        double pitch_deg = 0.0;
        double yaw_deg = 0.0;
        this->get_parameter("kfs_3d_localizer.camera_pose_robot.roll_deg", roll_deg);
        this->get_parameter("kfs_3d_localizer.camera_pose_robot.pitch_deg", pitch_deg);
        this->get_parameter("kfs_3d_localizer.camera_pose_robot.yaw_deg", yaw_deg);

        this->get_parameter("kfs_3d_localizer.projection.primary_point", primary_point_mode_);
        this->get_parameter("kfs_3d_localizer.projection.fallback_point", fallback_point_mode_);
        this->get_parameter("kfs_3d_localizer.projection.clamp_to_image", clamp_to_image_);
        this->get_parameter(
            "kfs_3d_localizer.projection.bottom_center_y_offset_ratio",
            bottom_center_y_offset_ratio_);

        this->get_parameter("kfs_3d_localizer.plane.mode", plane_mode_);
        this->get_parameter("kfs_3d_localizer.plane.default_z_height_mm", default_plane_z_height_mm_);
        this->get_parameter("kfs_3d_localizer.plane.valid_z_heights_mm", valid_z_heights_mm_);

        this->get_parameter("kfs_3d_localizer.validation.min_distance_mm", min_distance_mm_);
        this->get_parameter("kfs_3d_localizer.validation.max_distance_mm", max_distance_mm_);
        this->get_parameter("kfs_3d_localizer.validation.max_abs_y_mm", max_abs_y_mm_);
        this->get_parameter("kfs_3d_localizer.validation.reject_if_ray_parallel", reject_if_ray_parallel_);

        if (image_width_ <= 0) {
            image_width_ = 1280;
        }
        if (image_height_ <= 0) {
            image_height_ = 720;
        }
        if (std::abs(fx_) <= kEpsilon) {
            fx_ = 900.0;
        }
        if (std::abs(fy_) <= kEpsilon) {
            fy_ = 900.0;
        }

        normalizeCoeffVector(pinhole_coeffs_, 5U);
        normalizeCoeffVector(fisheye_coeffs_, 4U);

        const double roll_rad = roll_deg * kDegToRad;
        const double pitch_rad = pitch_deg * kDegToRad;
        const double yaw_rad = yaw_deg * kDegToRad;
        // OpenCV optical frame assumption: x right, y down, z forward.
        // Robot frame convention: x forward, y lateral, z up.
        // Extrinsic rotation is applied as yaw(Z) * pitch(Y) * roll(X).
        rotation_robot_camera_ =
            rotationZ(yaw_rad) * rotationY(pitch_rad) * rotationX(roll_rad);

        if (plane_mode_ != "fixed" &&
            plane_mode_ != "instance_hint" &&
            plane_mode_ != "block_map") {
            RCLCPP_WARN(
                this->get_logger(),
                "Unsupported plane mode '%s'; falling back to fixed",
                plane_mode_.c_str());
            plane_mode_ = "fixed";
        }

        sub_ = this->create_subscription<abu_yolo_ros::msg::KfsInstanceArray>(
            input_topic_,
            qos,
            std::bind(
                &Kfs3DLocalizerNode::instancesCallback,
                this,
                std::placeholders::_1));
        pub_ = this->create_publisher<abu_yolo_ros::msg::LocalizedKfsInstanceArray>(
            output_topic_,
            qos);

        RCLCPP_INFO(
            this->get_logger(),
            "KFS 3D Localizer: %s | input=%s | output=%s | robot_frame=%s | camera_frame=%s",
            enabled_ ? "enabled" : "disabled",
            input_topic_.c_str(),
            output_topic_.c_str(),
            robot_frame_.c_str(),
            camera_frame_.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "KFS 3D projection: primary=%s | fallback=%s | distortion=%s | plane_mode=%s",
            primary_point_mode_.c_str(),
            fallback_point_mode_.c_str(),
            distortion_model_.c_str(),
            plane_mode_.c_str());
        if (plane_mode_ != "fixed") {
            RCLCPP_WARN(
                this->get_logger(),
                "Plane mode '%s' is not implemented yet; runtime will fall back to fixed plane height",
                plane_mode_.c_str());
        }
    }

private:
    struct ProjectionPoint {
        bool valid = false;
        std::string mode = "unavailable";
        float u = 0.0F;
        float v = 0.0F;
        std::string failure_reason;
    };

    void normalizeCoeffVector(std::vector<double>& values, std::size_t expected_size)
    {
        if (values.size() < expected_size) {
            values.resize(expected_size, 0.0);
        } else if (values.size() > expected_size) {
            values.resize(expected_size);
        }
    }

    void instancesCallback(const abu_yolo_ros::msg::KfsInstanceArray::SharedPtr msg)
    {
        if (!enabled_) {
            return;
        }

        abu_yolo_ros::msg::LocalizedKfsInstanceArray output;
        output.header = msg->header;
        output.header.frame_id = robot_frame_;
        output.team_color = msg->team_color;
        output.instances.reserve(msg->instances.size());

        for (const auto& source_instance : msg->instances) {
            output.instances.push_back(localizeInstance(source_instance));
        }

        pub_->publish(output);

        if (debug_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "KFS 3D localized count=%zu topic=%s frame=%s",
                output.instances.size(),
                output_topic_.c_str(),
                robot_frame_.c_str());

            if (!output.instances.empty()) {
                for (const auto& localized : output.instances) {
                    std::ostringstream stream;
                    stream << "[[C" << localized.source_instance.cluster_id << "]]";
                    if (localized.localized) {
                        stream << " status=ok pos=("
                               << pointToString(
                                      cv::Vec3d(
                                          localized.position_robot_mm.x,
                                          localized.position_robot_mm.y,
                                          localized.position_robot_mm.z))
                               << ")mm dist=" << std::fixed << std::setprecision(1)
                               << localized.distance_mm
                               << " bearing=" << localized.bearing_deg
                               << " plane_z=" << localized.plane_z_height_mm
                               << " point=" << localized.projection_point_mode
                               << " quality=" << localized.localization_quality;
                    } else {
                        stream << " status=failed reason=\"" << localized.failure_reason << "\"";
                    }
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000,
                        "%s",
                        stream.str().c_str());
                }
            }
        }
    }

    abu_yolo_ros::msg::LocalizedKfsInstance localizeInstance(
        const abu_yolo_ros::msg::KfsInstance& source_instance) const
    {
        abu_yolo_ros::msg::LocalizedKfsInstance localized;
        localized.source_instance = source_instance;
        localized.localized = false;
        localized.localization_status = "failed";
        localized.localization_quality = "invalid_bbox";
        localized.failure_reason.clear();
        localized.projection_point_mode = "unavailable";
        localized.plane_height_source = "fixed";

        const auto& bbox = source_instance.bbox;
        if (!isValidBBox(bbox)) {
            return failLocalized(localized, "invalid_bbox", "invalid_bbox");
        }

        const ProjectionPoint projection = selectProjectionPoint(bbox);
        localized.projection_point_mode = projection.mode;
        localized.projection_u = projection.u;
        localized.projection_v = projection.v;
        if (!projection.valid) {
            return failLocalized(localized, "invalid_projection", projection.failure_reason);
        }

        cv::Point2d normalized_point;
        std::string undistort_reason;
        if (!undistortPoint(projection.u, projection.v, normalized_point, undistort_reason)) {
            return failLocalized(localized, "undistort_failed", undistort_reason);
        }

        const cv::Vec3d ray_camera = normalizeVec3(
            cv::Vec3d(normalized_point.x, normalized_point.y, 1.0));
        localized.ray_camera_x = static_cast<float>(ray_camera[0]);
        localized.ray_camera_y = static_cast<float>(ray_camera[1]);
        localized.ray_camera_z = static_cast<float>(ray_camera[2]);

        const cv::Vec3d ray_robot = normalizeVec3(rotation_robot_camera_ * ray_camera);
        localized.ray_robot_x = static_cast<float>(ray_robot[0]);
        localized.ray_robot_y = static_cast<float>(ray_robot[1]);
        localized.ray_robot_z = static_cast<float>(ray_robot[2]);

        double plane_z_height_mm = default_plane_z_height_mm_;
        std::string plane_height_source = "fixed";
        if (plane_mode_ != "fixed") {
            plane_height_source = "fixed";
        }
        localized.plane_z_height_mm = static_cast<float>(plane_z_height_mm);
        localized.plane_height_source = plane_height_source;

        if (reject_if_ray_parallel_ && std::abs(ray_robot[2]) <= kEpsilon) {
            return failLocalized(localized, "ray_parallel_to_plane", "ray_parallel_to_plane");
        }
        if (std::abs(ray_robot[2]) <= kEpsilon) {
            return failLocalized(localized, "ray_parallel_to_plane", "ray_parallel_to_plane");
        }

        const double t = (plane_z_height_mm - camera_origin_robot_[2]) / ray_robot[2];
        if (!std::isfinite(t) || t <= 0.0) {
            return failLocalized(localized, "intersection_behind_camera", "intersection_behind_camera");
        }

        const cv::Vec3d point_robot = camera_origin_robot_ + t * ray_robot;
        const double planar_distance_mm = std::hypot(point_robot[0], point_robot[1]);
        if (planar_distance_mm < min_distance_mm_ ||
            planar_distance_mm > max_distance_mm_ ||
            std::abs(point_robot[1]) > max_abs_y_mm_) {
            return failLocalized(localized, "out_of_range", "out_of_range");
        }

        localized.position_robot_mm = toGeometryPoint(point_robot);
        localized.distance_mm = static_cast<float>(planar_distance_mm);
        localized.bearing_deg = static_cast<float>(std::atan2(point_robot[1], point_robot[0]) * kRadToDeg);
        localized.localized = true;
        localized.localization_status = "ok";
        localized.localization_quality = "estimated_fixed_plane";
        localized.failure_reason.clear();
        return localized;
    }

    abu_yolo_ros::msg::LocalizedKfsInstance failLocalized(
        abu_yolo_ros::msg::LocalizedKfsInstance localized,
        const std::string& quality,
        const std::string& failure_reason) const
    {
        localized.localized = false;
        localized.localization_status = "failed";
        localized.localization_quality = quality;
        localized.failure_reason = failure_reason;
        return localized;
    }

    bool isValidBBox(const vision_msgs::msg::BoundingBox2D& bbox) const
    {
        return isFinite(bbox.center.position.x) &&
               isFinite(bbox.center.position.y) &&
               isFinite(bbox.size_x) &&
               isFinite(bbox.size_y) &&
               bbox.size_x > 0.0 &&
               bbox.size_y > 0.0;
    }

    ProjectionPoint selectProjectionPoint(const vision_msgs::msg::BoundingBox2D& bbox) const
    {
        ProjectionPoint projection = computeProjectionPoint(bbox, primary_point_mode_);
        if (projection.valid) {
            return projection;
        }

        if (fallback_point_mode_.empty() ||
            fallback_point_mode_ == primary_point_mode_) {
            return projection;
        }

        ProjectionPoint fallback = computeProjectionPoint(bbox, fallback_point_mode_);
        if (fallback.valid) {
            return fallback;
        }

        if (!projection.failure_reason.empty()) {
            fallback.failure_reason =
                projection.failure_reason + " -> fallback_failed:" + fallback.failure_reason;
        }
        return fallback;
    }

    ProjectionPoint computeProjectionPoint(
        const vision_msgs::msg::BoundingBox2D& bbox,
        const std::string& mode) const
    {
        ProjectionPoint projection;
        projection.mode = mode;

        double u = bbox.center.position.x;
        double v = bbox.center.position.y;
        if (mode == "bottom_center") {
            v += bbox.size_y * (0.5 + bottom_center_y_offset_ratio_);
        } else if (mode == "center" || mode == "fallback_center") {
            projection.mode = "center";
        } else {
            projection.failure_reason = "unsupported_projection_mode";
            return projection;
        }

        if (!isFinite(u) || !isFinite(v)) {
            projection.failure_reason = "non_finite_projection_point";
            return projection;
        }

        if (clamp_to_image_) {
            u = std::clamp(u, 0.0, static_cast<double>(std::max(0, image_width_ - 1)));
            v = std::clamp(v, 0.0, static_cast<double>(std::max(0, image_height_ - 1)));
        } else if (u < 0.0 ||
                   u >= image_width_ ||
                   v < 0.0 ||
                   v >= image_height_) {
            projection.failure_reason = "projection_point_outside_image";
            return projection;
        }

        projection.valid = true;
        projection.u = static_cast<float>(u);
        projection.v = static_cast<float>(v);
        return projection;
    }

    bool undistortPoint(
        float u,
        float v,
        cv::Point2d& normalized_point,
        std::string& failure_reason) const
    {
        if (distortion_model_ == "none") {
            normalized_point.x = (static_cast<double>(u) - cx_) / fx_;
            normalized_point.y = (static_cast<double>(v) - cy_) / fy_;
            return true;
        }

        const cv::Matx33d camera_matrix(
            fx_, 0.0, cx_,
            0.0, fy_, cy_,
            0.0, 0.0, 1.0);
        std::vector<cv::Point2f> src_points{cv::Point2f(u, v)};
        std::vector<cv::Point2f> dst_points;

        try {
            if (distortion_model_ == "pinhole") {
                const cv::Mat dist_coeffs(pinhole_coeffs_, true);
                cv::undistortPoints(src_points, dst_points, camera_matrix, dist_coeffs);
            } else if (distortion_model_ == "fisheye") {
                const cv::Mat dist_coeffs(fisheye_coeffs_, true);
                cv::fisheye::undistortPoints(src_points, dst_points, camera_matrix, dist_coeffs);
            } else {
                failure_reason = "unsupported_distortion_model";
                return false;
            }
        } catch (const cv::Exception& exception) {
            failure_reason = exception.what();
            return false;
        }

        if (dst_points.empty() ||
            !isFinite(dst_points.front().x) ||
            !isFinite(dst_points.front().y)) {
            failure_reason = "invalid_undistorted_point";
            return false;
        }

        normalized_point.x = dst_points.front().x;
        normalized_point.y = dst_points.front().y;
        return true;
    }

    geometry_msgs::msg::Point toGeometryPoint(const cv::Vec3d& point) const
    {
        geometry_msgs::msg::Point output;
        output.x = point[0];
        output.y = point[1];
        output.z = point[2];
        return output;
    }

    bool enabled_ = true;
    bool debug_ = true;
    bool clamp_to_image_ = true;
    bool reject_if_ray_parallel_ = true;
    int image_width_ = 1280;
    int image_height_ = 720;
    double fx_ = 900.0;
    double fy_ = 900.0;
    double cx_ = 640.0;
    double cy_ = 360.0;
    double bottom_center_y_offset_ratio_ = 0.0;
    double default_plane_z_height_mm_ = 0.0;
    double min_distance_mm_ = 100.0;
    double max_distance_mm_ = 3000.0;
    double max_abs_y_mm_ = 2000.0;

    std::string input_topic_;
    std::string output_topic_;
    std::string robot_frame_;
    std::string camera_frame_;
    std::string distortion_model_;
    std::string primary_point_mode_;
    std::string fallback_point_mode_;
    std::string plane_mode_;

    std::vector<double> pinhole_coeffs_;
    std::vector<double> fisheye_coeffs_;
    std::vector<double> valid_z_heights_mm_;

    cv::Vec3d camera_origin_robot_{0.0, 0.0, 300.0};
    cv::Matx33d rotation_robot_camera_ = cv::Matx33d::eye();

    rclcpp::Subscription<abu_yolo_ros::msg::KfsInstanceArray>::SharedPtr sub_;
    rclcpp::Publisher<abu_yolo_ros::msg::LocalizedKfsInstanceArray>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Kfs3DLocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
