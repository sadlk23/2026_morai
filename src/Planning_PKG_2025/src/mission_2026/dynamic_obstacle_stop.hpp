#ifndef DYNAMIC_OBSTACLE_STOP_H_
#define DYNAMIC_OBSTACLE_STOP_H_

#include "control.hpp"
#include "frenet_frame.hpp"
#include "lidar.hpp"
#include "local.hpp"
#include "mission_data.hpp"
#include "path.hpp"
#include "vision.hpp"
#include "nlohmann/json.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

/**
 * @brief verC: 혼용 동적 장애물 회피 미션 (FrenetFrame 기반)
 *
 * /LiDAR/car_dis (거리)와 /LiDAR/dynamic_obstacle_pos (velodyne 좌표 [x, y])를 활용하여
 * 전방 동적 장애물을 감지하고, TTC(Time To Collision)를 추정하여 다음 4가지 동작을 혼합 수행한다.
 *
 * 1. TTC < 2초: 긴급 정지
 * 2. 2초 <= TTC <= 5초 + 회피 공간 있음: FrenetFrame 기반 회피 경로 생성
 * 3. 장애물 UTM 절대위치가 3초간 유지: FrenetFrame 기반 회피
 * 4. 회피 경로 생성 실패: 정지 유지 후 1초 간격 재시도
 *
 * velodyne 좌표를 UTM으로 변환해 차량이 이동 중이어도
 * 장애물 자체의 정지 여부를 판별한다.
 */
class DynamicObstacleStop
{
private:
    enum class DynamicObstacleState
    {
        INIT,
        DRIVE,
        EMERGENCY_STOP,
        APPROACH_SLOW,
        WAIT_STOPPED_OBSTACLE,
        AVOID_PATH,
        DONE
    };

    Local &local_;
    Control &control_;
    Path &local_path_;
    Lidar &lidar_;
    Vision &vision_;
    MissionData &mission_data_;

    // FrenetFrame 기반 회피 경로 생성용 (Planning의 FrenetFrame과 분리하여 독립 사용)
    FrenetFrame frenet_frame_;

    double target_vel_;
    bool vel_flag_;
    DynamicObstacleState state_;

    // TTC 계산용 거리 이력
    struct DistanceSample
    {
        double distance;
        std::chrono::steady_clock::time_point time;
    };
    std::deque<DistanceSample> distance_history_;
    // 새 LiDAR 프레임(약 10Hz)만 저장해 최근 약 1초의 거리 변화로 TTC를 구한다.
    static constexpr size_t kMaxHistorySize = 12;
    static constexpr size_t kMinHistorySize = 3;
    static constexpr double kMinHistoryDuration = 0.2;

    // 차량 접근으로 상대거리가 줄어도 정지 장애물을 판별할 수 있도록
    // Velodyne 좌표를 UTM으로 변환한 절대위치 이력을 별도로 저장한다.
    struct ObstaclePositionSample
    {
        double x;
        double y;
        double relative_x;
        double relative_y;
        std::chrono::steady_clock::time_point time;
    };
    std::deque<ObstaclePositionSample> obstacle_position_history_;
    static constexpr size_t kMaxPositionHistorySize = 1000;

    // 여러 LiDAR 클러스터를 개별 궤적으로 연관시켜, 최근 장애물 하나가
    // 측면 돌진 객체를 가리지 않도록 한다.
    struct ObstacleTrack
    {
        std::uint64_t id;
        std::deque<ObstaclePositionSample> history;
        std::chrono::steady_clock::time_point last_update;
    };
    std::vector<ObstacleTrack> side_obstacle_tracks_;
    std::uint64_t next_obstacle_track_id_;
    std::uint64_t last_dynamic_obstacle_generation_;

    // 장애물 정지 대기 타이머
    std::chrono::steady_clock::time_point wait_start_time_;
    bool is_waiting_;
    std::chrono::steady_clock::time_point obstacle_lost_start_time_;
    bool obstacle_lost_pending_;

    // 측면에서 차로를 횡단하는 장애물의 2차원 추적/정지 상태
    bool side_collision_stop_;
    std::chrono::steady_clock::time_point last_side_collision_time_;

    // 신호 인지는 빨간/주황/좌회전 단독 신호에서 정지를 latch하고
    // 직진 초록 신호가 새로 수신될 때만 해제한다.
    std::uint64_t last_traffic_light_generation_;
    bool traffic_stop_latched_;

    struct TrafficStopLine
    {
        std::vector<double> point;
        int path_index;
        double path_s;
    };
    std::vector<std::vector<double>> raw_traffic_stop_lines_;
    std::vector<TrafficStopLine> traffic_stop_lines_;
    std::vector<double> traffic_path_s_;
    bool traffic_stop_lines_initialized_;
    double traffic_check_distance_;
    double traffic_stop_distance_;
    double traffic_slow_velocity_;
    double traffic_done_distance_;
    double traffic_max_projection_distance_;

    // 회피 경로 관련
    bool is_avoiding_;
    bool stationary_avoidance_;
    int avoidance_direction_; // +1: left, -1: right
    std::vector<double> active_avoidance_obstacle_utm_;
    std::vector<std::vector<double>> original_pos_arr_;
    std::vector<double> original_yaw_arr_;
    std::vector<double> original_k_arr_;

    // ---- 동적 장애물 판단 파라미터 ----
    // Perception 인지 코드에서 장애물이 감지되지 않을 때 발행하는 값
    static constexpr double kNoObstacleDistance = 25.0;
    // 거리 기반 정지/감속 임계값 (TTC 판단 외 fallback)
    static constexpr double kStopDistance = 5.0;
    static constexpr double kSlowDistance = 12.0;
    static constexpr double kSlowRatio = 0.3;
    // TTC 임계값 [s]
    static constexpr double kEmergencyTTC = 2.0;
    static constexpr double kAvoidTTCMin = 2.0;
    static constexpr double kAvoidTTCMax = 5.0;
    // UTM 좌표가 3초 동안 지름 0.6m 이내에 머물면 정지 장애물로 확정한다.
    static constexpr double kStationaryConfirmTime = 3.0;
    static constexpr double kStationaryPositionTolerance = 0.6;
    // 차량이 정차한 상태에서는 GPS/heading 흔들림 영향이 없는
    // LiDAR 상대좌표도 정지 판정의 예외 기준으로 사용한다.
    static constexpr double kStoppedEgoVelocity = 0.2;
    static constexpr double kStationaryRelativeTolerance = 0.4;
    static constexpr double kObstacleTrackJumpDistance = 2.0;
    static constexpr double kObstacleObservationGap = 0.5;
    // Frenet 예측 경로가 장애물까지 도달할 수 있는 거리 안에서만 회피를 시작한다.
    static constexpr double kStationaryAvoidDistance = 10.0;
    // 회피 경로 생성 실패 시 정지 후 재시도 주기 [s]
    static constexpr double kAvoidanceRetryInterval = 1.0;
    // 회피 중 LiDAR 단일 프레임 누락으로 원본 경로에 복귀하지 않게 한다.
    static constexpr double kObstacleClearConfirmTime = 0.5;
    // Frenet 생성 실패 시 글로벌 경로에 부드러운 평행 오프셋을 적용하는 fallback.
    static constexpr double kFallbackAvoidanceOffset = 2.0;
    static constexpr double kFallbackTransitionDistance = 5.0;
    static constexpr double kFallbackHoldDistance = 3.0;
    static constexpr double kFallbackRecoveryDistance = 6.0;
    static constexpr double kFallbackMinClearance = 1.2;
    // 장애물 중심과 실제 제어 경로 사이의 거리가 이 값 이내일 때만
    // 거리/TTC/정지 회피 판단을 적용한다.
    static constexpr double kPathCollisionHalfWidth = 1.8;
    static constexpr double kPathBehindTolerance = 1.0;
    // 측면 돌진 보행자는 더 넓은 영역에서 관측하고 2차원 CPA로 별도 판단한다.
    static constexpr double kObservationMinX = -3.0;
    static constexpr double kObservationHalfWidth = 11.5;
    static constexpr double kSideVelocityWindow = 0.8;
    static constexpr double kSideMinHistoryDuration = 0.15;
    static constexpr double kSideMinLateralSpeed = 0.25;
    static constexpr double kSidePredictionHorizon = 5.0;
    static constexpr double kSideCollisionDistance = 2.8;
    static constexpr double kSideClearHoldTime = 1.0;
    static constexpr double kTrackAssociationDistance = 2.0;
    static constexpr double kTrackStaleTime = 0.7;
    static constexpr std::size_t kMaxSideTracks = 64;
    static constexpr double kAvoidancePassDistance = 2.0;
    // 미션 종료 거리 [m]
    static constexpr double kPathChangeDistance = 2.0;
    // 글로벌 패스 시작점이 차량과 멀 때 진입 경로를 만드는 기준.
    // 잘 추종되는 2025_Simulation_psh의 dynamic_obstacle 미션과 동일한 값을 사용한다.
    static constexpr double kApproachPathSpacing = 0.1;
    static constexpr double kDirectFollowDistance = 3.0;
    static constexpr double kPi = 3.14159265358979323846;

    // TTC 및 회피 관련 내부 함수
    double computeClosureRate() const;
    double computeTTC(double distance) const;
    bool isObstacleStopped(double distance) const;
    void updateObstaclePositionHistory(const std::vector<double> &obstacle_utm,
                                       double relative_x, double relative_y);
    void updateSideObstacleTracks(
        const std::vector<std::vector<double>> &relative_obstacles);
    double getStationaryObservationDuration() const;
    bool isObservedObstacle(double velodyne_x, double velodyne_y) const;
    bool getObstaclePathRelation(double velodyne_x, double velodyne_y,
                                 double &forward_path_distance,
                                 double &lateral_path_distance) const;
    bool isFrontalObstacle(double velodyne_x, double velodyne_y) const;
    std::vector<double> convertObstacleToUTM(double velodyne_x,
                                             double velodyne_y) const;
    std::vector<std::vector<double>> getFrontalObstaclesInUTM() const;
    bool predictSideCollision(double planned_velocity,
                              double &collision_time,
                              double &closest_distance,
                              double &relative_velocity_x,
                              double &relative_velocity_y) const;
    bool hasAvoidanceSpace() const;
    bool buildFallbackAvoidancePath(
        const std::vector<double> &obstacle_utm,
        std::vector<std::vector<double>> &avoid_pos,
        std::vector<double> &avoid_yaw,
        std::vector<double> &avoid_k) const;
    bool generateAvoidancePath(bool stationary_triggered = false);
    bool hasPassedAvoidanceObstacle() const;
    void restoreOriginalPath();
    void initializeFollowingPath();
    bool checkMissionEnd() const;
    void updateTrafficSignalState();
    void initializeTrafficStopLines();

public:
    DynamicObstacleStop(Local &local, Control &control, Path &local_path,
                        Lidar &lidar, Vision &vision,
                        MissionData &mission_data,
                        const nlohmann::json &traffic_param);

    /**
     * @brief /LiDAR/car_dis 거리를 기반으로 안전한 목표 속도를 판단한다.
     * @param distance  /LiDAR/car_dis 값 (음수 또는 25.0이면 장애물 미감지)
     * @param desired_vel  원래 주행해야 할 미션 목표 속도
     * @return 판단 후의 안전한 목표 속도
     */
    double judgeDynamicObstacleVelocity(double distance, double desired_vel);

    /**
     * @brief verC: 혼용 동적 장애물 회피 미션 메인 함수
     * @param desired_vel  이 미션에서 주행할 기준 속도 (mission_list 또는 param으로부터)
     */
    void doMission(double desired_vel);

    /**
     * @brief 장애물 판단이 모두 끝난 뒤 호출하는 최종 신호 우선 제약.
     */
    void applyTrafficPriority();

    /**
     * @brief 상태 리셋 (같은 미션 반복 실행 시 필요)
     */
    void resetStatus();
};

#endif // DYNAMIC_OBSTACLE_STOP_H_
