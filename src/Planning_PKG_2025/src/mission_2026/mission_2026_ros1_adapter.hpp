#pragma once

#include "gps_shadow_tunnel.hpp"
#include "static_traffic_mission.hpp"

#include "path.hpp"

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <mutex>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace mission_2026
{

/**
 * planning_node가 기존 출력 대신 발행할 수 있도록 ROS 메시지 직전 형태로
 * 변환된 미션 결과다.
 */
struct Mission2026Override
{
    // true이면 이 주기에는 기존 Planning::chooseFunc()를 실행하지 않는다.
    // 터널 종료 직후처럼 새 미션이 절대 경로 복귀만 요청하는 주기에도
    // 기존의 동일 미션 로직이 다시 실행되는 것을 방지하기 위한 플래그다.
    bool handles_mission = false;
    bool active = false;
    bool request_absolute_path = false;
    double target_speed_mps = 0.0;
    std::vector<double> positions;
    std::vector<double> yaws;
    std::vector<double> curvatures;
    std::string diagnostic;
};

/**
 * 순수 C++ 미션 코어와 기존 ROS1 토픽 사이의 얇은 변환 계층.
 *
 * 이 클래스는 센서 토픽을 구독하고 최신 timestamp를 보관한다. evaluate()
 * 호출 시 현재 미션 번호에 맞는 코어 입력을 구성하고, 결과 경로를 기존
 * /Planning/* 발행 형식으로 변환한다.
 */
class Mission2026Ros1Adapter
{
public:
    Mission2026Ros1Adapter(ros::NodeHandle &node,
                           ros::NodeHandle &private_node);

    /**
     * @param mission_number 현재 mission_list의 미션 번호
     * @param reference_path 현재 미션의 UTM 기준 전역 경로
     * @param mapped_stop_line 기존 param.json에서 읽은 UTM 정지선.
     *        최신 /Vision/stopline이 있으면 인지 좌표를 우선 사용한다.
     */
    Mission2026Override evaluate(int mission_number,
                                 const Path &reference_path,
                                 const std::optional<Point2D>
                                     &mapped_stop_line = std::nullopt);

private:
    struct SensorCache
    {
        GpsSample gps;
        Point2D utm;
        double utm_stamp = -std::numeric_limits<double>::infinity();
        double heading_rad = 0.0;
        double heading_stamp = -std::numeric_limits<double>::infinity();
        double yaw_rate_rps = 0.0;
        double imu_stamp = -std::numeric_limits<double>::infinity();
        double speed_mps = 0.0;
        double speed_stamp = -std::numeric_limits<double>::infinity();

        std::vector<Point2D> left_wall_points;
        std::vector<Point2D> right_wall_points;
        double wall_stamp = -std::numeric_limits<double>::infinity();
        bool tunnel_end = false;
        double tunnel_end_stamp = -std::numeric_limits<double>::infinity();

        TimedTrafficSignal signal;
        Point2D stop_line_utm;
        bool stop_line_valid = false;
        double stop_line_stamp = -std::numeric_limits<double>::infinity();
        std::vector<Point2D> obstacle_vehicle_points;
        double obstacle_stamp = -std::numeric_limits<double>::infinity();
    };

    struct ReferenceSample
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        double curvature = 0.0;
        double s = 0.0;
    };

    ros::NodeHandle node_;
    ros::NodeHandle private_node_;

    ros::Subscriber fix_sub_;
    ros::Subscriber utm_sub_;
    ros::Subscriber heading_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber status_sub_;
    ros::Subscriber wall_sub_;
    ros::Subscriber tunnel_end_sub_;
    ros::Subscriber traffic_sub_;
    ros::Subscriber stop_line_sub_;
    ros::Subscriber obstacle_sub_;

    mutable std::mutex cache_mutex_;
    SensorCache cache_;

    TunnelConfig tunnel_config_;
    StaticTrafficConfig static_config_;
    GpsShadowTunnel tunnel_mission_;
    StaticTrafficMission static_traffic_mission_;

    std::vector<int> tunnel_mission_ids_;
    std::vector<int> static_traffic_mission_ids_;
    std::vector<int> traffic_mission_ids_;
    double pose_timeout_s_ = 0.35;
    double wall_topic_timeout_s_ = 0.30;
    double stop_line_timeout_s_ = 2.0;
    double obstacle_default_width_m_ = 1.0;
    double obstacle_default_length_m_ = 1.0;
    bool road_bounds_valid_ = false;
    double road_left_bound_m_ = 3.0;
    double road_right_bound_m_ = -3.0;

    std::vector<ReferenceSample> reference_;
    std::string reference_name_;
    std::size_t reference_size_ = 0;

    void fixCallback(const sensor_msgs::NavSatFix::ConstPtr &message);
    void utmCallback(const geometry_msgs::PointStamped::ConstPtr &message);
    void headingCallback(const std_msgs::Float64::ConstPtr &message);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &message);
    void statusCallback(const std_msgs::Float32MultiArray::ConstPtr &message);
    void wallCallback(const std_msgs::Float64MultiArray::ConstPtr &message);
    void tunnelEndCallback(const std_msgs::Bool::ConstPtr &message);
    void trafficCallback(const std_msgs::Int32MultiArray::ConstPtr &message);
    void stopLineCallback(const geometry_msgs::PointStamped::ConstPtr &message);
    void obstacleCallback(const std_msgs::Float64MultiArray::ConstPtr &message);

    [[nodiscard]] bool isMission(
        int mission_number,
        const std::vector<int> &mission_ids) const;
    [[nodiscard]] SensorCache snapshot() const;

    bool updateReference(const Path &path);
    [[nodiscard]] FrenetPoint worldToFrenet(const Point2D &world) const;
    [[nodiscard]] Point2D vehicleToWorld(const Point2D &vehicle,
                                         const SensorCache &cache) const;
    [[nodiscard]] std::vector<PathPoint> frenetToWorld(
        const std::vector<FrenetPoint> &path) const;
    [[nodiscard]] ReferenceSample interpolateReference(double s) const;

    Mission2026Override evaluateTunnel(const SensorCache &cache,
                                       double now);
    Mission2026Override evaluateStaticTraffic(const SensorCache &cache,
                                              double now,
                                              const std::optional<Point2D>
                                                  &mapped_stop_line,
                                              bool enable_traffic_policy);
    [[nodiscard]] Mission2026Override safeStop(
        const std::string &reason) const;

    static TunnelConfig loadTunnelConfig(ros::NodeHandle &node);
    static StaticTrafficConfig loadStaticConfig(ros::NodeHandle &node);
};

} // namespace mission_2026
