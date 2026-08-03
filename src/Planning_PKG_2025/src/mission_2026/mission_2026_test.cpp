#include "gps_shadow_tunnel.hpp"
#include "static_traffic_mission.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

/**
 * @file mission_2026_test.cpp
 * @brief ROS 없이 핵심 알고리즘을 검증하는 합성 회귀시험
 *
 * 실제 시뮬레이터 연결 전에도 다음 안전 속성을 빠르게 확인하기 위한
 * 테스트다.
 *
 * - 정상 센서에서 올바른 모드를 선택하는가
 * - 센서가 사라질 때 단계적으로 fallback하는가
 * - 오래된 GPS/신호/장애물을 정상값으로 오인하지 않는가
 * - 선택된 경로가 확장 장애물을 통과하지 않는가
 * - 회피 공간이 없을 때 정지하는가
 *
 * 외부 테스트 프레임워크에 의존하지 않으므로 단일 실행 파일로 컴파일해
 * 개발 PC와 대회 PC에서 동일하게 실행할 수 있다.
 */
namespace
{
using namespace mission_2026;

// 조건을 만족하지 않으면 즉시 예외를 발생시켜 실패 원인을 한 줄로 출력한다.
void require(bool condition, const std::string &message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

std::vector<Point2D> makeWall(double intercept, double slope = 0.0)
{
    /**
     * 직선 벽 합성 데이터.
     *
     * - x=2~24m 구간을 생성해 near/far 적합을 모두 검증
     * - 작은 결정론적 노이즈를 넣어 현실적인 잔차 생성
     * - 큰 이상치 1개를 넣어 robust fit의 이상치 제거 검증
     */
    std::vector<Point2D> points;
    for (int i = 0; i <= 22; ++i)
    {
        const double x = 2.0 + static_cast<double>(i);
        const double deterministic_noise =
            0.015 * std::sin(static_cast<double>(i) * 0.7);
        points.push_back({x, slope * x + intercept + deterministic_noise});
    }
    // A deterministic outlier verifies robust rejection.
    points.push_back({12.0, intercept + 1.5});
    return points;
}

void testTunnelDualWallAndFallback()
{
    /**
     * 검증 시나리오
     *
     * 1. 좌우 벽 정상 → DUAL_WALL
     * 2. 캘리브레이션 시간 경과 → WALL_TRACKING
     * 3. 벽 손실 0.15초 → PREDICTED
     * 4. 벽 손실 0.50초 → DEAD_RECKONING
     * 5. 벽 손실 1초 초과 → SAFE_STOP, 목표속도 0
     */
    TunnelConfig config;
    config.calibration_duration_s = 0.20;
    config.known_tunnel_length_m = 20.0;
    config.wall_fit.minimum_points = 4;
    config.wall_fit.minimum_x_span = 2.0;
    config.wall_fit.maximum_rmse = 0.10;

    GpsShadowTunnel mission(config);
    TunnelInput input;
    input.mission_active = true;
    input.gps.valid = true;
    input.gps.pose = {0.0, 0.0, 0.0};
    input.left_wall_points = makeWall(3.0, 0.01);
    input.right_wall_points = makeWall(-3.0, 0.01);

    input.now = 0.0;
    input.gps.stamp = 0.0;
    auto output = mission.update(input);
    require(output.tracking_mode == TunnelTrackingMode::kDualWall,
            "dual walls were not selected during calibration");
    require(output.relative_path.valid, "dual-wall relative path is invalid");

    input.now = 0.25;
    input.gps.stamp = 0.25;
    output = mission.update(input);
    require(output.state == TunnelState::kWallTracking,
            "calibration did not transition to wall tracking");
    require(output.relative_path.toPlanningPositionArray().at(0) ==
                kRelativePathMarker,
            "relative-path marker is missing");

    input.left_wall_points.clear();
    input.right_wall_points.clear();
    input.gps.valid = false;

    input.now = 0.40;
    output = mission.update(input);
    require(output.tracking_mode == TunnelTrackingMode::kPredicted,
            "short wall dropout did not use predicted mode");

    input.now = 0.75;
    output = mission.update(input);
    require(output.tracking_mode == TunnelTrackingMode::kDeadReckoning,
            "medium wall dropout did not use dead reckoning");

    input.now = 1.40;
    output = mission.update(input);
    require(output.tracking_mode == TunnelTrackingMode::kSafeStop,
            "long wall dropout did not enter safe stop");
    require(output.target_speed_mps == 0.0,
            "safe stop did not request zero speed");
}

void testTunnelGpsRecovery()
{
    /**
     * 알려진 터널 길이까지 속도를 적분한 뒤 exit_hint와 정상 GPS를
     * 연속 입력한다. 설정한 연속 샘플 수 이후 DONE과 절대경로 복귀
     * 요청이 출력되는지 확인한다.
     */
    TunnelConfig config;
    config.calibration_duration_s = 0.10;
    config.known_tunnel_length_m = 3.0;
    config.exit_search_margin_m = 0.5;
    config.gps_recovery_required_samples = 3;
    config.gps_recovery_maximum_innovation_m = 2.0;
    config.wall_fit.maximum_rmse = 0.10;

    GpsShadowTunnel mission(config);
    TunnelInput input;
    input.mission_active = true;
    input.vehicle_speed_mps = 2.0;
    input.left_wall_points = makeWall(3.0);
    input.right_wall_points = makeWall(-3.0);
    input.gps.valid = true;
    input.gps.pose = {0.0, 0.0, 0.0};

    for (int i = 0; i < 8; ++i)
    {
        input.now = 0.2 * static_cast<double>(i);
        input.gps.stamp = input.now;
        input.gps.pose.x = 2.0 * input.now;
        mission.update(input);
    }

    input.exit_hint = true;
    TunnelOutput output;
    for (int i = 8; i < 12; ++i)
    {
        input.now = 0.2 * static_cast<double>(i);
        input.gps.stamp = input.now;
        input.gps.pose.x = 2.0 * input.now;
        output = mission.update(input);
    }

    require(output.state == TunnelState::kDone,
            "GPS recovery did not complete");
    require(output.request_absolute_path,
            "GPS recovery did not request the absolute path");
}

ObstacleObservation obstacle(double now,
                             int id = 7,
                             double s = 15.0,
                             double d = 0.0)
{
    // 여러 정적 장애물 시험에서 사용하는 기본 장애물 생성 함수.
    ObstacleObservation result;
    result.id = id;
    result.s = s;
    result.d = d;
    result.length = 1.0;
    result.width = 1.0;
    result.confidence = 0.95;
    result.stamp = now;
    return result;
}

void testStaticObstacleCandidate()
{
    /**
     * 폭 8m 도로 중앙에 정적 장애물을 두고 두 프레임 연속 검출한다.
     * 선택된 회피 경로의 모든 점을 다시 확장 장애물과 비교하여
     * 실제 충돌하지 않는 경로인지 독립적으로 확인한다.
     */
    StaticTrafficConfig config;
    config.required_obstacle_confirmations = 2;
    StaticTrafficMission mission(config);

    StaticTrafficInput input;
    input.mission_active = true;
    input.road_bounds = {4.0, -4.0, true};
    input.traffic_signal = {
        TrafficSignal::kGreen,
        0.0,
        0.95};

    input.now = 0.0;
    input.obstacles = {obstacle(input.now)};
    mission.update(input);

    input.now = 0.1;
    input.traffic_signal.stamp = input.now;
    input.obstacles = {obstacle(input.now)};
    auto output = mission.update(input);

    require(output.state == StaticTrafficState::kAvoidance,
            "confirmed static obstacle did not trigger avoidance");
    require(!output.frenet_path.empty(),
            "avoidance path is empty");
    require(output.selected_side != 0,
            "avoidance did not select a side");
    require(output.target_speed_mps > 0.0,
            "valid avoidance requested a stop");

    const double expanded_min_s =
        15.0 - 0.5 - 0.5 * config.vehicle_length_m -
        config.longitudinal_safety_margin_m;
    const double expanded_max_s =
        15.0 + 0.5 + 0.5 * config.vehicle_length_m +
        config.longitudinal_safety_margin_m;
    const double expanded_half_width =
        0.5 + 0.5 * config.vehicle_width_m +
        config.lateral_control_error_m +
        config.perception_lateral_error_m +
        config.lateral_safety_margin_m;

    for (const auto &point : output.frenet_path)
    {
        const bool within_longitudinal =
            point.s >= expanded_min_s && point.s <= expanded_max_s;
        const bool within_lateral =
            std::abs(point.d) <= expanded_half_width;
        require(!(within_longitudinal && within_lateral),
                "selected path intersects the expanded obstacle");
    }
}

void testTrafficStopAndUnknownFailSafe()
{
    /**
     * 1. 적색 신호에서 STOP_FOR_SIGNAL 선택
     * 2. 정지선 버퍼에 도달하면 목표속도 0
     * 3. 오래된 녹색 신호는 UNKNOWN으로 폐기
     * 4. 정지선 근처 UNKNOWN은 진행하지 않고 정지
     */
    StaticTrafficConfig config;
    StaticTrafficMission mission(config);

    StaticTrafficInput input;
    input.mission_active = true;
    input.road_bounds = {4.0, -4.0, true};
    input.stop_line = {10.0, true};
    input.traffic_signal = {TrafficSignal::kRed, 0.0, 0.95};

    input.now = 0.0;
    auto output = mission.update(input);
    require(output.state == StaticTrafficState::kStopForSignal,
            "red signal did not select stop state");
    require(!output.frenet_path.empty(),
            "red signal did not provide a stop path");

    input.current_s = 9.2;
    input.now = 0.1;
    input.traffic_signal.stamp = input.now;
    output = mission.update(input);
    require(output.target_speed_mps == 0.0,
            "vehicle did not stop before the stop-line buffer");

    mission.reset();
    input.current_s = 0.0;
    input.now = 1.0;
    input.traffic_signal = {
        TrafficSignal::kGreen,
        0.0, // stale on purpose
        0.95};
    output = mission.update(input);
    require(output.effective_signal == TrafficSignal::kUnknown,
            "stale traffic signal was not changed to UNKNOWN");
    require(output.state == StaticTrafficState::kStopForSignal,
            "UNKNOWN signal near a stop line did not fail safe");
}

void testNoAvoidanceSpaceStops()
{
    /**
     * 차량 한 대가 간신히 들어가는 좁은 도로에 중앙 장애물을 배치한다.
     * 좌우 어느 후보도 차량 footprint와 경계를 만족할 수 없으므로
     * SAFE_STOP이 선택되어야 한다.
     */
    StaticTrafficConfig config;
    config.required_obstacle_confirmations = 1;
    StaticTrafficMission mission(config);

    StaticTrafficInput input;
    input.mission_active = true;
    input.now = 0.0;
    input.road_bounds = {1.2, -1.2, true};
    input.obstacles = {obstacle(input.now)};

    const auto output = mission.update(input);
    require(output.state == StaticTrafficState::kSafeStop,
            "blocked narrow road did not enter safe stop");
    require(output.target_speed_mps == 0.0,
            "blocked narrow road did not request zero speed");
}

void testRepeatedRosMessageIsNotMultipleConfirmations()
{
    /**
     * planning_node는 100 Hz지만 인지 토픽은 그보다 느릴 수 있다.
     * 어댑터가 보관한 동일 메시지를 여러 판단 주기에 전달하더라도
     * 장애물/녹색 확인 횟수가 부풀려지면 안 된다.
     */
    StaticTrafficConfig config;
    config.required_obstacle_confirmations = 2;
    config.required_green_confirmations = 2;
    StaticTrafficMission mission(config);

    StaticTrafficInput input;
    input.mission_active = true;
    input.road_bounds = {4.0, -4.0, true};
    input.stop_line = {10.0, true};
    input.now = 0.0;
    input.obstacles = {obstacle(0.0)};
    input.traffic_signal = {TrafficSignal::kGreen, 0.0, 0.95};
    mission.update(input);

    // 판단 시간만 진행시키고 센서 timestamp는 그대로 둔다.
    input.now = 0.05;
    auto output = mission.update(input);
    require(output.state != StaticTrafficState::kAvoidance,
            "one cached obstacle frame was counted twice");
    require(output.effective_signal == TrafficSignal::kUnknown,
            "one cached green frame was counted twice");

    // 실제로 새 센서 프레임이 도착하면 두 번째 확인으로 인정한다.
    input.now = 0.10;
    input.obstacles = {obstacle(input.now)};
    input.traffic_signal.stamp = input.now;
    output = mission.update(input);
    require(output.state == StaticTrafficState::kAvoidance,
            "new obstacle frame was not confirmed");
    require(output.effective_signal == TrafficSignal::kGreen,
            "new green frame was not confirmed");
}

} // namespace

int main()
{
    // 각 시험은 독립적인 미션 객체를 사용해 이전 상태가 다음 시험에
    // 영향을 주지 않는다. 하나라도 실패하면 비정상 종료코드를 반환한다.
    try
    {
        testTunnelDualWallAndFallback();
        testTunnelGpsRecovery();
        testStaticObstacleCandidate();
        testTrafficStopAndUnknownFailSafe();
        testNoAvoidanceSpaceStops();
        testRepeatedRosMessageIsNotMultipleConfirmations();
        std::cout << "mission_2026 tests: PASS\n";
        return EXIT_SUCCESS;
    }
    catch (const std::exception &exception)
    {
        std::cerr << "mission_2026 tests: FAIL: "
                  << exception.what() << '\n';
        return EXIT_FAILURE;
    }
}
