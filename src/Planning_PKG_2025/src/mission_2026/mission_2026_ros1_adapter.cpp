#include "mission_2026_ros1_adapter.hpp"

#include <sensor_msgs/NavSatStatus.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

/**
 * @file mission_2026_ros1_adapter.cpp
 * @brief mission_2026 순수 C++ 코어를 기존 ROS1 planning_node에 연결
 *
 * 이 파일만 ROS1 메시지와 토픽을 이해한다. GPS 터널과 정적 장애물
 * 알고리즘은 ROS에 의존하지 않으므로 향후 토픽명이나 메시지 형식이
 * 변경되어도 이 어댑터만 수정하면 된다.
 */
namespace mission_2026
{
namespace
{

double messageStampOrNow(const ros::Time &stamp)
{
    return stamp.isZero() ? ros::Time::now().toSec() : stamp.toSec();
}

bool isFresh(double now, double stamp, double timeout_s)
{
    // rosbag 반복 재생처럼 /clock이 과거로 되감긴 경우, 미래 timestamp의
    // 캐시를 새 데이터로 오판하지 않는다.
    return std::isfinite(stamp) &&
           stamp <= now &&
           now - stamp <= timeout_s;
}

std::vector<Point2D> sampleLineSegment(const Point2D &start,
                                       const Point2D &end,
                                       double step_m = 0.5)
{
    std::vector<Point2D> points;
    const double length = std::hypot(end.x - start.x, end.y - start.y);
    const int count = std::max(4, static_cast<int>(std::ceil(length / step_m)));
    points.reserve(static_cast<std::size_t>(count + 1));
    for (int i = 0; i <= count; ++i)
    {
        const double ratio =
            static_cast<double>(i) / static_cast<double>(count);
        points.push_back({
            start.x + ratio * (end.x - start.x),
            start.y + ratio * (end.y - start.y)});
    }
    return points;
}

void appendWorldPath(const std::vector<PathPoint> &path,
                     Mission2026Override &output)
{
    output.positions.reserve(path.size() * 2);
    output.yaws.reserve(path.size());
    output.curvatures.reserve(path.size());
    for (const auto &point : path)
    {
        output.positions.push_back(point.x);
        output.positions.push_back(point.y);
        output.yaws.push_back(point.yaw);
        output.curvatures.push_back(point.curvature);
    }
}

} // namespace

TunnelConfig Mission2026Ros1Adapter::loadTunnelConfig(
    ros::NodeHandle &node)
{
    TunnelConfig config;
    node.param("mission_2026/tunnel/calibration_duration_s",
               config.calibration_duration_s,
               config.calibration_duration_s);
    node.param("mission_2026/tunnel/known_length_m",
               config.known_tunnel_length_m,
               config.known_tunnel_length_m);
    node.param("mission_2026/tunnel/normal_speed_mps",
               config.normal_speed_mps,
               config.normal_speed_mps);
    node.param("mission_2026/tunnel/predicted_speed_mps",
               config.predicted_speed_mps,
               config.predicted_speed_mps);
    node.param("mission_2026/tunnel/dead_reckoning_speed_mps",
               config.dead_reckoning_speed_mps,
               config.dead_reckoning_speed_mps);
    node.param("mission_2026/tunnel/wall_hold_duration_s",
               config.wall_hold_duration_s,
               config.wall_hold_duration_s);
    node.param("mission_2026/tunnel/dead_reckoning_limit_s",
               config.dead_reckoning_limit_s,
               config.dead_reckoning_limit_s);
    node.param("mission_2026/tunnel/path_lookahead_m",
               config.path_lookahead_m,
               config.path_lookahead_m);
    node.param("mission_2026/tunnel/path_step_m",
               config.path_step_m,
               config.path_step_m);
    node.param("mission_2026/tunnel/maximum_lateral_correction_m",
               config.maximum_lateral_correction_m,
               config.maximum_lateral_correction_m);
    node.param("mission_2026/tunnel/gps_timeout_s",
               config.gps_timeout_s,
               config.gps_timeout_s);
    node.param("mission_2026/tunnel/exit_search_margin_m",
               config.exit_search_margin_m,
               config.exit_search_margin_m);
    node.param("mission_2026/tunnel/near_start_m",
               config.near_start_m,
               config.near_start_m);
    node.param("mission_2026/tunnel/near_end_m",
               config.near_end_m,
               config.near_end_m);
    node.param("mission_2026/tunnel/far_start_m",
               config.far_start_m,
               config.far_start_m);
    node.param("mission_2026/tunnel/far_end_m",
               config.far_end_m,
               config.far_end_m);
    node.param("mission_2026/tunnel/maximum_wall_rmse_m",
               config.wall_fit.maximum_rmse,
               config.wall_fit.maximum_rmse);
    node.param("mission_2026/tunnel/minimum_wall_x_span_m",
               config.wall_fit.minimum_x_span,
               config.wall_fit.minimum_x_span);
    node.param("mission_2026/tunnel/default_left_wall_distance_m",
               config.default_left_wall_distance_m,
               config.default_left_wall_distance_m);
    node.param("mission_2026/tunnel/default_right_wall_distance_m",
               config.default_right_wall_distance_m,
               config.default_right_wall_distance_m);
    node.param("mission_2026/tunnel/gps_recovery_maximum_innovation_m",
               config.gps_recovery_maximum_innovation_m,
               config.gps_recovery_maximum_innovation_m);
    node.param("mission_2026/tunnel/gps_recovery_required_samples",
               config.gps_recovery_required_samples,
               config.gps_recovery_required_samples);
    node.param("mission_2026/tunnel/wall_filter_alpha",
               config.wall_filter_alpha,
               config.wall_filter_alpha);

    // /LiDAR/wall_dist는 한 벽당 선분 끝점 두 개만 제공한다. callback에서
    // 선분을 등간격 샘플로 변환하므로 core의 robust fit을 그대로 사용한다.
    config.wall_fit.minimum_points = 4;
    config.path_step_m = std::max(0.05, config.path_step_m);
    config.path_lookahead_m =
        std::max(config.path_step_m, config.path_lookahead_m);
    config.near_start_m = std::max(0.0, config.near_start_m);
    config.near_end_m =
        std::max(config.near_start_m + 0.5, config.near_end_m);
    config.far_start_m =
        std::max(config.near_end_m, config.far_start_m);
    config.far_end_m =
        std::max(config.far_start_m + 0.5, config.far_end_m);
    config.wall_fit.minimum_x_span =
        std::max(0.1, config.wall_fit.minimum_x_span);
    config.wall_fit.maximum_rmse =
        std::max(0.01, config.wall_fit.maximum_rmse);
    config.maximum_lateral_correction_m =
        std::max(0.0, config.maximum_lateral_correction_m);
    config.wall_filter_alpha =
        std::clamp(config.wall_filter_alpha, 0.0, 1.0);
    config.gps_timeout_s = std::max(0.01, config.gps_timeout_s);
    config.wall_hold_duration_s =
        std::max(0.0, config.wall_hold_duration_s);
    config.dead_reckoning_limit_s =
        std::max(config.wall_hold_duration_s,
                 config.dead_reckoning_limit_s);
    config.gps_recovery_required_samples =
        std::max(1, config.gps_recovery_required_samples);
    return config;
}

StaticTrafficConfig Mission2026Ros1Adapter::loadStaticConfig(
    ros::NodeHandle &node)
{
    StaticTrafficConfig config;
    node.param("mission_2026/static/vehicle_width_m",
               config.vehicle_width_m,
               config.vehicle_width_m);
    node.param("mission_2026/static/vehicle_length_m",
               config.vehicle_length_m,
               config.vehicle_length_m);
    node.param("mission_2026/static/lateral_control_error_m",
               config.lateral_control_error_m,
               config.lateral_control_error_m);
    node.param("mission_2026/static/perception_lateral_error_m",
               config.perception_lateral_error_m,
               config.perception_lateral_error_m);
    node.param("mission_2026/static/lateral_safety_margin_m",
               config.lateral_safety_margin_m,
               config.lateral_safety_margin_m);
    node.param("mission_2026/static/longitudinal_safety_margin_m",
               config.longitudinal_safety_margin_m,
               config.longitudinal_safety_margin_m);
    node.param("mission_2026/static/boundary_margin_m",
               config.boundary_margin_m,
               config.boundary_margin_m);
    node.param("mission_2026/static/detection_range_m",
               config.detection_range_m,
               config.detection_range_m);
    node.param("mission_2026/static/obstacle_timeout_s",
               config.obstacle_timeout_s,
               config.obstacle_timeout_s);
    node.param("mission_2026/static/signal_timeout_s",
               config.signal_timeout_s,
               config.signal_timeout_s);
    node.param("mission_2026/static/unknown_signal_stop_distance_m",
               config.unknown_signal_stop_distance_m,
               config.unknown_signal_stop_distance_m);
    node.param("mission_2026/static/approach_speed_mps",
               config.approach_speed_mps,
               config.approach_speed_mps);
    node.param("mission_2026/static/avoidance_speed_mps",
               config.avoidance_speed_mps,
               config.avoidance_speed_mps);
    node.param("mission_2026/static/maximum_curvature_per_m",
               config.maximum_curvature_per_m,
               config.maximum_curvature_per_m);
    node.param("mission_2026/static/maximum_lateral_acceleration_mps2",
               config.maximum_lateral_acceleration_mps2,
               config.maximum_lateral_acceleration_mps2);
    node.param("mission_2026/static/required_obstacle_confirmations",
               config.required_obstacle_confirmations,
               config.required_obstacle_confirmations);
    node.param("mission_2026/static/required_green_confirmations",
               config.required_green_confirmations,
               config.required_green_confirmations);
    node.param("mission_2026/static/stop_line_buffer_m",
               config.stop_line_buffer_m,
               config.stop_line_buffer_m);
    node.param("mission_2026/static/path_step_m",
               config.path_step_m,
               config.path_step_m);
    node.param("mission_2026/static/lateral_candidate_step_m",
               config.lateral_candidate_step_m,
               config.lateral_candidate_step_m);
    node.param("mission_2026/static/minimum_entry_length_m",
               config.minimum_entry_length_m,
               config.minimum_entry_length_m);
    node.param("mission_2026/static/comfortable_deceleration_mps2",
               config.comfortable_deceleration_mps2,
               config.comfortable_deceleration_mps2);
    node.param("mission_2026/static/emergency_stop_distance_m",
               config.emergency_stop_distance_m,
               config.emergency_stop_distance_m);
    node.param("mission_2026/static/return_speed_mps",
               config.return_speed_mps,
               config.return_speed_mps);

    // 후보 조합은 YAML 배열로 쉽게 늘리거나 줄일 수 있다. 빈 배열은
    // 모든 후보가 사라지므로 기존 기본값을 유지한다.
    std::vector<double> values;
    if (node.getParam(
            "mission_2026/static/early_merge_distances_m", values) &&
        !values.empty())
    {
        config.early_merge_distances_m = values;
    }
    values.clear();
    if (node.getParam(
            "mission_2026/static/return_lengths_m", values) &&
        !values.empty())
    {
        config.return_lengths_m = values;
    }

    config.vehicle_width_m = std::max(0.1, config.vehicle_width_m);
    config.vehicle_length_m = std::max(0.1, config.vehicle_length_m);
    config.path_step_m = std::max(0.05, config.path_step_m);
    config.lateral_candidate_step_m =
        std::max(0.05, config.lateral_candidate_step_m);
    config.detection_range_m = std::max(1.0, config.detection_range_m);
    config.obstacle_timeout_s =
        std::max(0.01, config.obstacle_timeout_s);
    config.signal_timeout_s = std::max(0.01, config.signal_timeout_s);
    config.required_obstacle_confirmations =
        std::max(1, config.required_obstacle_confirmations);
    config.required_green_confirmations =
        std::max(1, config.required_green_confirmations);
    config.comfortable_deceleration_mps2 =
        std::max(0.1, config.comfortable_deceleration_mps2);
    config.early_merge_distances_m.erase(
        std::remove_if(
            config.early_merge_distances_m.begin(),
            config.early_merge_distances_m.end(),
            [](double value)
            {
                return !std::isfinite(value) || value < 0.0;
            }),
        config.early_merge_distances_m.end());
    config.return_lengths_m.erase(
        std::remove_if(
            config.return_lengths_m.begin(),
            config.return_lengths_m.end(),
            [&](double value)
            {
                return !std::isfinite(value) ||
                       value < config.path_step_m;
            }),
        config.return_lengths_m.end());
    if (config.early_merge_distances_m.empty())
    {
        config.early_merge_distances_m = {0.0};
    }
    if (config.return_lengths_m.empty())
    {
        config.return_lengths_m = {8.0};
    }
    return config;
}

Mission2026Ros1Adapter::Mission2026Ros1Adapter(
    ros::NodeHandle &node,
    ros::NodeHandle &private_node)
    : node_(node),
      private_node_(private_node),
      tunnel_config_(loadTunnelConfig(private_node_)),
      static_config_(loadStaticConfig(private_node_)),
      tunnel_mission_(tunnel_config_),
      static_traffic_mission_(static_config_)
{
    if (!private_node_.getParam("mission_2026/tunnel_mission_ids",
                               tunnel_mission_ids_))
    {
        tunnel_mission_ids_ = {23};
    }
    if (!private_node_.getParam("mission_2026/static_traffic_mission_ids",
                               static_traffic_mission_ids_))
    {
        static_traffic_mission_ids_ = {14, 31};
    }
    if (!private_node_.getParam("mission_2026/traffic_mission_ids",
                               traffic_mission_ids_))
    {
        traffic_mission_ids_ = {31};
    }

    private_node_.param("mission_2026/pose_timeout_s",
                        pose_timeout_s_,
                        pose_timeout_s_);
    private_node_.param("mission_2026/wall_topic_timeout_s",
                        wall_topic_timeout_s_,
                        wall_topic_timeout_s_);
    private_node_.param("mission_2026/stop_line_timeout_s",
                        stop_line_timeout_s_,
                        stop_line_timeout_s_);
    private_node_.param("mission_2026/static/default_obstacle_width_m",
                        obstacle_default_width_m_,
                        obstacle_default_width_m_);
    private_node_.param("mission_2026/static/default_obstacle_length_m",
                        obstacle_default_length_m_,
                        obstacle_default_length_m_);
    private_node_.param("mission_2026/static/road_bounds_valid",
                        road_bounds_valid_,
                        road_bounds_valid_);
    private_node_.param("mission_2026/static/road_left_bound_m",
                        road_left_bound_m_,
                        road_left_bound_m_);
    private_node_.param("mission_2026/static/road_right_bound_m",
                        road_right_bound_m_,
                        road_right_bound_m_);

    std::string obstacle_topic = "/LiDAR/object_cen";
    private_node_.param<std::string>("mission_2026/static/obstacle_topic",
                                     obstacle_topic,
                                     obstacle_topic);

    fix_sub_ = node_.subscribe(
        "/fix", 5, &Mission2026Ros1Adapter::fixCallback, this);
    utm_sub_ = node_.subscribe(
        "/Local/utm", 5, &Mission2026Ros1Adapter::utmCallback, this);
    heading_sub_ = node_.subscribe(
        "/Local/heading", 5,
        &Mission2026Ros1Adapter::headingCallback, this);
    imu_sub_ = node_.subscribe(
        "/imu", 10, &Mission2026Ros1Adapter::imuCallback, this);
    status_sub_ = node_.subscribe(
        "/ERP/serial_data", 10,
        &Mission2026Ros1Adapter::statusCallback, this);
    wall_sub_ = node_.subscribe(
        "/LiDAR/wall_dist", 10,
        &Mission2026Ros1Adapter::wallCallback, this);
    tunnel_end_sub_ = node_.subscribe(
        "/LiDAR/tunnel_end", 5,
        &Mission2026Ros1Adapter::tunnelEndCallback, this);
    traffic_sub_ = node_.subscribe(
        "/Vision/traffic_sign", 5,
        &Mission2026Ros1Adapter::trafficCallback, this);
    stop_line_sub_ = node_.subscribe(
        "/Vision/stopline", 5,
        &Mission2026Ros1Adapter::stopLineCallback, this);
    obstacle_sub_ = node_.subscribe(
        obstacle_topic, 5,
        &Mission2026Ros1Adapter::obstacleCallback, this);

    ROS_INFO("mission_2026 ROS1 adapter initialized (obstacle_topic=%s)",
             obstacle_topic.c_str());
}

void Mission2026Ros1Adapter::fixCallback(
    const sensor_msgs::NavSatFix::ConstPtr &message)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.gps.valid =
        message->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX;
    cache_.gps.stamp = messageStampOrNow(message->header.stamp);
}

void Mission2026Ros1Adapter::utmCallback(
    const geometry_msgs::PointStamped::ConstPtr &message)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.utm = {message->point.x, message->point.y};
    cache_.utm_stamp = messageStampOrNow(message->header.stamp);
}

void Mission2026Ros1Adapter::headingCallback(
    const std_msgs::Float64::ConstPtr &message)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.heading_rad = message->data;
    cache_.heading_stamp = ros::Time::now().toSec();
}

void Mission2026Ros1Adapter::imuCallback(
    const sensor_msgs::Imu::ConstPtr &message)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.yaw_rate_rps = message->angular_velocity.z;
    cache_.imu_stamp = messageStampOrNow(message->header.stamp);
}

void Mission2026Ros1Adapter::statusCallback(
    const std_msgs::Float32MultiArray::ConstPtr &message)
{
    if (message->data.size() <= 3)
    {
        ROS_WARN_THROTTLE(
            1.0,
            "mission_2026: /ERP/serial_data requires speed at index 3");
        return;
    }
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.speed_mps = std::max(0.0F, message->data[3]);
    cache_.speed_stamp = ros::Time::now().toSec();
}

void Mission2026Ros1Adapter::wallCallback(
    const std_msgs::Float64MultiArray::ConstPtr &message)
{
    if (message->data.size() < 4)
    {
        return;
    }
    const std::size_t checked_size = std::min<std::size_t>(message->data.size(), 8);
    if (!std::all_of(
            message->data.begin(),
            message->data.begin() + checked_size,
            [](double value)
            {
                return std::isfinite(value);
            }))
    {
        ROS_WARN_THROTTLE(
            1.0,
            "mission_2026: non-finite /LiDAR/wall_dist ignored");
        return;
    }

    const Point2D first_start{message->data[0], message->data[1]};
    const Point2D first_end{message->data[2], message->data[3]};
    const double first_mean_y = 0.5 * (first_start.y + first_end.y);

    std::vector<Point2D> left;
    std::vector<Point2D> right;
    if (message->data.size() >= 8)
    {
        const Point2D second_start{message->data[4], message->data[5]};
        const Point2D second_end{message->data[6], message->data[7]};
        const double second_mean_y = 0.5 * (second_start.y + second_end.y);
        if (first_mean_y >= second_mean_y)
        {
            left = sampleLineSegment(first_start, first_end);
            right = sampleLineSegment(second_start, second_end);
        }
        else
        {
            left = sampleLineSegment(second_start, second_end);
            right = sampleLineSegment(first_start, first_end);
        }
    }
    else if (first_mean_y >= 0.0)
    {
        left = sampleLineSegment(first_start, first_end);
    }
    else
    {
        right = sampleLineSegment(first_start, first_end);
    }

    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.left_wall_points = std::move(left);
    cache_.right_wall_points = std::move(right);
    cache_.wall_stamp = ros::Time::now().toSec();
}

void Mission2026Ros1Adapter::tunnelEndCallback(
    const std_msgs::Bool::ConstPtr &message)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.tunnel_end = message->data;
    cache_.tunnel_end_stamp = ros::Time::now().toSec();
}

void Mission2026Ros1Adapter::trafficCallback(
    const std_msgs::Int32MultiArray::ConstPtr &message)
{
    TrafficSignal signal = TrafficSignal::kUnknown;
    if (message->data.size() >= 4)
    {
        if (message->data[0] != 0)
        {
            signal = TrafficSignal::kRed;
        }
        else if (message->data[1] != 0)
        {
            signal = TrafficSignal::kYellow;
        }
        else if (message->data[3] != 0)
        {
            signal = TrafficSignal::kGreen;
        }
    }

    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.signal = {
        signal,
        ros::Time::now().toSec(),
        signal == TrafficSignal::kUnknown ? 0.0 : 1.0};
}

void Mission2026Ros1Adapter::stopLineCallback(
    const geometry_msgs::PointStamped::ConstPtr &message)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.stop_line_utm = {message->point.x, message->point.y};
    cache_.stop_line_valid =
        std::isfinite(message->point.x) && std::isfinite(message->point.y);
    cache_.stop_line_stamp = messageStampOrNow(message->header.stamp);
}

void Mission2026Ros1Adapter::obstacleCallback(
    const std_msgs::Float64MultiArray::ConstPtr &message)
{
    std::vector<Point2D> points;
    points.reserve(message->data.size() / 2);
    for (std::size_t i = 0; i + 1 < message->data.size(); i += 2)
    {
        if (std::isfinite(message->data[i]) &&
            std::isfinite(message->data[i + 1]))
        {
            points.push_back({message->data[i], message->data[i + 1]});
        }
    }

    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_.obstacle_vehicle_points = std::move(points);
    cache_.obstacle_stamp = ros::Time::now().toSec();
}

bool Mission2026Ros1Adapter::isMission(
    int mission_number,
    const std::vector<int> &mission_ids) const
{
    return std::find(mission_ids.begin(),
                     mission_ids.end(),
                     mission_number) != mission_ids.end();
}

Mission2026Ros1Adapter::SensorCache
Mission2026Ros1Adapter::snapshot() const
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return cache_;
}

bool Mission2026Ros1Adapter::updateReference(const Path &path)
{
    const auto &positions = path.getRefPosArr();
    const auto &yaws = path.getRefYawArr();
    const auto &curvatures = path.getRefKArr();
    if (positions.size() < 2 ||
        yaws.size() < positions.size() ||
        curvatures.size() < positions.size())
    {
        reference_.clear();
        return false;
    }

    const std::string name = path.getPathName();
    if (name == reference_name_ && positions.size() == reference_size_ &&
        !reference_.empty())
    {
        return true;
    }

    reference_.clear();
    reference_.reserve(positions.size());
    double cumulative_s = 0.0;
    for (std::size_t i = 0; i < positions.size(); ++i)
    {
        if (i > 0)
        {
            cumulative_s += std::hypot(
                positions[i][0] - positions[i - 1][0],
                positions[i][1] - positions[i - 1][1]);
        }
        reference_.push_back({
            positions[i][0],
            positions[i][1],
            yaws[i],
            curvatures[i],
            cumulative_s});
    }
    reference_name_ = name;
    reference_size_ = positions.size();
    return true;
}

FrenetPoint Mission2026Ros1Adapter::worldToFrenet(
    const Point2D &world) const
{
    FrenetPoint result;
    if (reference_.empty())
    {
        return result;
    }

    std::size_t closest = 0;
    double minimum_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < reference_.size(); ++i)
    {
        const double distance = std::hypot(
            world.x - reference_[i].x,
            world.y - reference_[i].y);
        if (distance < minimum_distance)
        {
            minimum_distance = distance;
            closest = i;
        }
    }

    const auto &base = reference_[closest];
    const double dx = world.x - base.x;
    const double dy = world.y - base.y;
    result.s = base.s;
    result.d = -std::sin(base.yaw) * dx + std::cos(base.yaw) * dy;
    return result;
}

Point2D Mission2026Ros1Adapter::vehicleToWorld(
    const Point2D &vehicle,
    const SensorCache &cache) const
{
    return {
        cache.utm.x + std::cos(cache.heading_rad) * vehicle.x -
            std::sin(cache.heading_rad) * vehicle.y,
        cache.utm.y + std::sin(cache.heading_rad) * vehicle.x +
            std::cos(cache.heading_rad) * vehicle.y};
}

Mission2026Ros1Adapter::ReferenceSample
Mission2026Ros1Adapter::interpolateReference(double s) const
{
    if (reference_.empty())
    {
        return {};
    }
    if (s <= reference_.front().s)
    {
        return reference_.front();
    }
    if (s >= reference_.back().s)
    {
        return reference_.back();
    }

    const auto upper = std::lower_bound(
        reference_.begin(),
        reference_.end(),
        s,
        [](const ReferenceSample &sample, double target_s)
        {
            return sample.s < target_s;
        });
    const auto lower = std::prev(upper);
    const double span = std::max(upper->s - lower->s, 1e-9);
    const double ratio = (s - lower->s) / span;

    ReferenceSample result;
    result.s = s;
    result.x = lower->x + ratio * (upper->x - lower->x);
    result.y = lower->y + ratio * (upper->y - lower->y);
    result.yaw = normalizeAngle(
        lower->yaw +
        ratio * normalizeAngle(upper->yaw - lower->yaw));
    result.curvature =
        lower->curvature +
        ratio * (upper->curvature - lower->curvature);
    return result;
}

std::vector<PathPoint> Mission2026Ros1Adapter::frenetToWorld(
    const std::vector<FrenetPoint> &path) const
{
    std::vector<PathPoint> world;
    world.reserve(path.size());
    for (const auto &point : path)
    {
        const auto reference = interpolateReference(point.s);
        world.push_back({
            reference.x - std::sin(reference.yaw) * point.d,
            reference.y + std::cos(reference.yaw) * point.d,
            normalizeAngle(reference.yaw + point.yaw_error),
            reference.curvature + point.curvature});
    }
    return world;
}

Mission2026Override Mission2026Ros1Adapter::safeStop(
    const std::string &reason) const
{
    Mission2026Override output;
    output.handles_mission = true;
    output.active = true;
    output.target_speed_mps = 0.0;
    output.diagnostic = reason;
    output.positions = {
        kRelativePathMarker, kRelativePathMarker,
        0.0, 0.0,
        1.0, 0.0,
        2.0, 0.0,
        3.0, 0.0};
    output.yaws = {0.0, 0.0, 0.0, 0.0};
    output.curvatures = {0.0, 0.0, 0.0, 0.0};
    return output;
}

Mission2026Override Mission2026Ros1Adapter::evaluateTunnel(
    const SensorCache &cache,
    double now)
{
    TunnelInput input;
    input.now = now;
    input.mission_active = true;
    input.vehicle_speed_mps =
        isFresh(now, cache.speed_stamp, pose_timeout_s_)
            ? cache.speed_mps
            : 0.0;
    input.imu_yaw_rate_rps =
        isFresh(now, cache.imu_stamp, pose_timeout_s_)
            ? cache.yaw_rate_rps
            : 0.0;

    input.gps = cache.gps;
    const bool pose_fresh =
        isFresh(now, cache.utm_stamp, pose_timeout_s_) &&
        isFresh(now, cache.heading_stamp, pose_timeout_s_);
    input.gps.valid = input.gps.valid && pose_fresh;
    input.gps.pose = {
        cache.utm.x,
        cache.utm.y,
        cache.heading_rad};
    input.gps.stamp =
        std::min(cache.gps.stamp,
                 std::min(cache.utm_stamp, cache.heading_stamp));

    if (isFresh(now, cache.wall_stamp, wall_topic_timeout_s_))
    {
        input.left_wall_points = cache.left_wall_points;
        input.right_wall_points = cache.right_wall_points;
    }
    input.exit_hint =
        cache.tunnel_end &&
        isFresh(now,
                cache.tunnel_end_stamp,
                wall_topic_timeout_s_);

    const auto result = tunnel_mission_.update(input);
    Mission2026Override output;
    output.handles_mission = true;
    output.active = result.override_normal_planning;
    output.request_absolute_path = result.request_absolute_path;
    output.target_speed_mps = result.target_speed_mps;
    output.positions = result.relative_path.toPlanningPositionArray();
    output.yaws = result.relative_path.yawArray();
    output.curvatures = result.relative_path.curvatureArray();
    output.diagnostic = result.diagnostic;

    if (output.active && output.positions.empty())
    {
        return safeStop("tunnel mission returned an empty relative path");
    }
    return output;
}

Mission2026Override Mission2026Ros1Adapter::evaluateStaticTraffic(
    const SensorCache &cache,
    double now,
    const std::optional<Point2D> &mapped_stop_line,
    bool enable_traffic_policy)
{
    const bool pose_fresh =
        isFresh(now, cache.utm_stamp, pose_timeout_s_) &&
        isFresh(now, cache.heading_stamp, pose_timeout_s_);
    if (!pose_fresh || reference_.empty())
    {
        return safeStop("static mission localization/reference path is stale");
    }

    StaticTrafficInput input;
    input.now = now;
    input.mission_active = true;
    input.vehicle_speed_mps =
        isFresh(now, cache.speed_stamp, pose_timeout_s_)
            ? cache.speed_mps
            : 0.0;
    const auto current = worldToFrenet(cache.utm);
    input.current_s = current.s;
    input.current_d = current.d;
    input.road_bounds = {
        road_left_bound_m_,
        road_right_bound_m_,
        road_bounds_valid_};
    if (enable_traffic_policy)
    {
        input.traffic_signal = cache.signal;
        if (cache.stop_line_valid &&
            isFresh(now, cache.stop_line_stamp, stop_line_timeout_s_))
        {
            input.stop_line = {
                worldToFrenet(cache.stop_line_utm).s,
                true};
        }
        else if (mapped_stop_line.has_value())
        {
            input.stop_line = {
                worldToFrenet(*mapped_stop_line).s,
                true};
        }
    }

    if (isFresh(now,
                cache.obstacle_stamp,
                static_config_.obstacle_timeout_s))
    {
        int obstacle_id = 0;
        for (const auto &relative : cache.obstacle_vehicle_points)
        {
            const auto world = vehicleToWorld(relative, cache);
            const auto frenet = worldToFrenet(world);
            input.obstacles.push_back({
                obstacle_id++,
                frenet.s,
                frenet.d,
                obstacle_default_length_m_,
                obstacle_default_width_m_,
                1.0,
                cache.obstacle_stamp});
        }
    }

    const auto result = static_traffic_mission_.update(input);
    Mission2026Override output;
    output.handles_mission = true;
    output.active = result.override_normal_planning;
    output.target_speed_mps = result.target_speed_mps;
    output.diagnostic = result.diagnostic;
    appendWorldPath(frenetToWorld(result.frenet_path), output);

    if (output.active && output.positions.empty())
    {
        return safeStop("static mission returned an empty path");
    }
    return output;
}

Mission2026Override Mission2026Ros1Adapter::evaluate(
    int mission_number,
    const Path &reference_path,
    const std::optional<Point2D> &mapped_stop_line)
{
    const double now = ros::Time::now().toSec();
    const auto cache = snapshot();
    const bool tunnel_active =
        isMission(mission_number, tunnel_mission_ids_);
    const bool static_active =
        isMission(mission_number, static_traffic_mission_ids_);
    const bool traffic_active =
        isMission(mission_number, traffic_mission_ids_);

    if (tunnel_active)
    {
        StaticTrafficInput inactive_static;
        inactive_static.now = now;
        static_traffic_mission_.update(inactive_static);
        return evaluateTunnel(cache, now);
    }

    TunnelInput inactive_tunnel;
    inactive_tunnel.now = now;
    tunnel_mission_.update(inactive_tunnel);

    if (static_active)
    {
        if (!updateReference(reference_path))
        {
            return safeStop("failed to build Frenet reference path");
        }
        return evaluateStaticTraffic(cache, now, mapped_stop_line,
                                     traffic_active);
    }

    StaticTrafficInput inactive_static;
    inactive_static.now = now;
    static_traffic_mission_.update(inactive_static);
    return {};
}

} // namespace mission_2026
