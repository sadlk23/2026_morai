#include "static_traffic_mission.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

/**
 * @file static_traffic_mission.cpp
 * @brief 정적 장애물 회피와 신호등 정지를 함께 판단하는 미션 구현
 *
 * 좌표계
 * ------
 * 모든 장애물과 경로는 기준경로의 Frenet 좌표를 사용한다.
 *
 * - s: 기준경로를 따라 진행한 거리
 * - d: 기준경로 왼쪽 방향의 횡거리
 *
 * 이 설계는 일반 GPS 구간에서는 UTM 기준경로를 사용하고, GPS 음영
 * 터널에서는 터널 상대 s/d 좌표를 사용할 수 있게 한다.
 *
 * 경로 생성 방식
 * --------------
 * 사용자가 제안한 "안전지대 오프셋"과 "사전 회피"를 별도 코드로
 * 나누지 않는다. 조기 합류 거리, 횡 오프셋, 복귀 길이가 다른 여러
 * 5차 다항식 후보를 만들고 다음 조건을 만족하는 최저 비용 경로를 고른다.
 *
 * - 확장 장애물과 충돌하지 않음
 * - 차량 전체 폭이 도로 경계 안에 있음
 * - 최대 곡률을 넘지 않음
 * - 적색/황색 신호의 정지선을 넘지 않음
 *
 * 유효한 후보가 없으면 좁은 틈을 억지로 통과하지 않고 SAFE_STOP을
 * 반환하는 것이 가장 중요한 안전 원칙이다.
 */
namespace mission_2026
{

StaticTrafficMission::StaticTrafficMission(StaticTrafficConfig config)
    : config_(std::move(config))
{
    reset();
}

void StaticTrafficMission::reset()
{
    // 이전 미션에서 선택한 회피 방향과 장애물 트랙이 다음 미션 판단에
    // 영향을 주지 않도록 완전히 초기화한다.
    state_ = StaticTrafficState::kIdle;
    tracked_obstacles_.clear();
    next_generated_id_ = 1000000;
    selected_side_ = 0;
    selected_obstacle_id_ = -1;
    green_confirmation_count_ = 0;
    last_processed_signal_stamp_ =
        -std::numeric_limits<double>::infinity();
}

StaticTrafficState StaticTrafficMission::state() const
{
    return state_;
}

int StaticTrafficMission::findAssociatedTrack(
    const ObstacleObservation &observation) const
{
    /**
     * 인지 객체를 기존 트랙과 연결한다.
     *
     * 1. 인지에서 안정적인 ID를 제공하면 ID를 우선 사용한다.
     * 2. ID가 없거나 찾지 못하면 Frenet s/d 거리로 가장 가까운 트랙을 찾는다.
     *
     * association_distance보다 먼 객체는 새 장애물로 생성된다. 이 값이
     * 너무 작으면 같은 장애물이 여러 개로 분리되고, 너무 크면 가까운 두
     * 장애물이 하나로 합쳐지므로 인지 위치 노이즈에 맞춰 튜닝해야 한다.
     */
    if (observation.id >= 0)
    {
        for (std::size_t i = 0; i < tracked_obstacles_.size(); ++i)
        {
            if (tracked_obstacles_[i].observation.id == observation.id)
            {
                return static_cast<int>(i);
            }
        }
    }

    double best_distance = config_.association_distance_m;
    int best_index = -1;
    for (std::size_t i = 0; i < tracked_obstacles_.size(); ++i)
    {
        const auto &tracked = tracked_obstacles_[i].observation;
        const double distance =
            std::hypot(observation.s - tracked.s, observation.d - tracked.d);
        if (distance < best_distance)
        {
            best_distance = distance;
            best_index = static_cast<int>(i);
        }
    }
    return best_index;
}

void StaticTrafficMission::updateTracks(const StaticTrafficInput &input)
{
    /**
     * 장애물 트랙 갱신 순서
     *
     * 1. confidence, timestamp, 인지거리 검증
     * 2. 기존 트랙과 ID/거리 연관
     * 3. 위치와 크기를 저역통과 필터로 갱신
     * 4. 연속 확인 횟수 증가
     * 5. timeout을 넘긴 트랙 삭제
     *
     * 장애물이 한 프레임 검출됐다는 이유만으로 경로를 바꾸지 않도록
     * confirmations를 누적한다.
     */
    for (auto &track : tracked_obstacles_)
    {
        track.seen_this_cycle = false;
    }

    for (auto observation : input.obstacles)
    {
        if (!isFinite(observation.s) || !isFinite(observation.d) ||
            observation.confidence < config_.minimum_obstacle_confidence ||
            observation.stamp > input.now ||
            input.now - observation.stamp > config_.obstacle_timeout_s)
        {
            continue;
        }

        const double forward_distance = observation.s - input.current_s;
        if (forward_distance < -config_.vehicle_length_m ||
            forward_distance > config_.detection_range_m)
        {
            continue;
        }

        int index = findAssociatedTrack(observation);
        if (index < 0)
        {
            if (observation.id < 0)
            {
                observation.id = next_generated_id_++;
            }
            tracked_obstacles_.push_back(
                {observation, 1, observation.stamp, true});
            continue;
        }

        auto &track = tracked_obstacles_[static_cast<std::size_t>(index)];
        // 같은 센서 메시지를 판단 주기(예: 100 Hz)마다 다시 처리해
        // confirmations가 부풀려지는 것을 막는다. 확인 횟수는 새로운
        // timestamp의 인지 프레임이 들어왔을 때만 증가한다.
        if (observation.stamp <= track.observation.stamp + 1e-9)
        {
            continue;
        }

        // 위치가 튀어 매 주기 회피 경로가 흔들리는 것을 줄이기 위한 필터.
        constexpr double kTrackAlpha = 0.60;
        track.observation.s =
            lowPass(track.observation.s, observation.s, kTrackAlpha);
        track.observation.d =
            lowPass(track.observation.d, observation.d, kTrackAlpha);
        track.observation.length =
            std::max(0.1, lowPass(track.observation.length,
                                  observation.length,
                                  kTrackAlpha));
        track.observation.width =
            std::max(0.1, lowPass(track.observation.width,
                                  observation.width,
                                  kTrackAlpha));
        track.observation.confidence = observation.confidence;
        track.observation.stamp = observation.stamp;
        track.last_seen = observation.stamp;
        track.seen_this_cycle = true;
        ++track.confirmations;
    }

    tracked_obstacles_.erase(
        std::remove_if(
            tracked_obstacles_.begin(),
            tracked_obstacles_.end(),
            [&](const TrackedObstacle &track)
            {
                return input.now - track.last_seen >
                       config_.obstacle_timeout_s;
            }),
        tracked_obstacles_.end());
}

std::vector<ObstacleObservation> StaticTrafficMission::confirmedObstacles(
    const StaticTrafficInput &input) const
{
    // 설정된 연속 검출 횟수와 freshness를 모두 만족한 트랙만 경로계획에
    // 사용한다. 오래된 마지막 좌표를 장애물로 계속 쓰지 않는다.
    std::vector<ObstacleObservation> result;
    for (const auto &track : tracked_obstacles_)
    {
        if (track.confirmations >= config_.required_obstacle_confirmations &&
            input.now - track.last_seen <= config_.obstacle_timeout_s)
        {
            result.push_back(track.observation);
        }
    }
    return result;
}

TrafficSignal StaticTrafficMission::effectiveSignal(
    const StaticTrafficInput &input)
{
    /**
     * 신호등 fail-safe 정책
     *
     * - timeout을 넘긴 신호는 이전 색을 유지하지 않고 UNKNOWN으로 변경
     * - RED/YELLOW는 한 번의 유효 검출로도 정지 판단 가능
     * - GREEN은 연속 N회 확인 후에만 확정
     *
     * 잘못된 녹색 한 프레임으로 차량이 출발하는 상황을 방지한다.
     */
    const bool fresh =
        input.traffic_signal.stamp <= input.now &&
        input.now - input.traffic_signal.stamp <= config_.signal_timeout_s;
    const TrafficSignal raw =
        fresh ? input.traffic_signal.state : TrafficSignal::kUnknown;

    const bool new_signal_sample =
        fresh &&
        input.traffic_signal.stamp > last_processed_signal_stamp_ + 1e-9;
    if (new_signal_sample)
    {
        last_processed_signal_stamp_ = input.traffic_signal.stamp;
        if (raw == TrafficSignal::kGreen)
        {
            ++green_confirmation_count_;
        }
        else
        {
            green_confirmation_count_ = 0;
        }
    }

    if (raw == TrafficSignal::kGreen &&
        green_confirmation_count_ < config_.required_green_confirmations)
    {
        return TrafficSignal::kUnknown;
    }
    return raw;
}

StaticTrafficMission::ExpandedObstacle
StaticTrafficMission::expandObstacle(
    const ObstacleObservation &obstacle) const
{
    /**
     * 장애물 확장은 차량을 점이 아닌 실제 footprint로 취급하기 위한
     * Minkowski-sum 근사다.
     *
     * 횡방향:
     *   장애물 반폭 + 차량 반폭 + 제어오차 + 인지오차 + 안전여유
     *
     * 종방향:
     *   장애물 반길이 + 차량 반길이 + 종방향 안전여유
     *
     * 이후 경로의 중심점이 이 확장 사각형에 들어가는지만 검사해도
     * 원래 차량 외곽과 장애물 외곽 사이의 충돌을 보수적으로 판정할 수 있다.
     */
    ExpandedObstacle expanded;
    expanded.id = obstacle.id;
    expanded.minimum_s =
        obstacle.s - 0.5 * obstacle.length -
        0.5 * config_.vehicle_length_m -
        config_.longitudinal_safety_margin_m;
    expanded.maximum_s =
        obstacle.s + 0.5 * obstacle.length +
        0.5 * config_.vehicle_length_m +
        config_.longitudinal_safety_margin_m;
    expanded.center_d = obstacle.d;
    expanded.lateral_half_width =
        0.5 * obstacle.width +
        0.5 * config_.vehicle_width_m +
        config_.lateral_control_error_m +
        config_.perception_lateral_error_m +
        config_.lateral_safety_margin_m;
    return expanded;
}

bool StaticTrafficMission::obstacleThreatensReference(
    const ExpandedObstacle &obstacle,
    const StaticTrafficInput &input) const
{
    // 기준경로 d=0의 차량이 그대로 주행했을 때 확장 장애물과 겹치는
    // 장애물만 1차 위협 대상으로 선택한다. 다른 후보 경로를 검사할 때는
    // 주변의 모든 확장 장애물을 다시 충돌 검사한다.
    return obstacle.maximum_s >= input.current_s &&
           obstacle.minimum_s - input.current_s <= config_.detection_range_m &&
           std::abs(obstacle.center_d) <= obstacle.lateral_half_width;
}

bool StaticTrafficMission::signalRequiresStop(
    const StaticTrafficInput &input,
    TrafficSignal signal) const
{
    /**
     * 정지 필요 조건
     * - 정지선을 이미 충분히 통과했으면 현재 미션에서는 무시
     * - RED/YELLOW이면 정지
     * - UNKNOWN이고 정지선이 가까우면 보수적으로 정지
     * - GREEN은 effectiveSignal에서 연속 확인된 경우에만 전달됨
     */
    if (!input.stop_line.valid)
    {
        return false;
    }

    const double distance = input.stop_line.s - input.current_s;
    if (distance < -config_.stop_line_buffer_m)
    {
        return false;
    }

    if (signal == TrafficSignal::kRed ||
        signal == TrafficSignal::kYellow)
    {
        return true;
    }
    return signal == TrafficSignal::kUnknown &&
           distance <= config_.unknown_signal_stop_distance_m;
}

double StaticTrafficMission::stoppingTargetSpeed(
    const StaticTrafficInput &input) const
{
    /**
     * 남은 정지거리로 허용 목표속도를 계산한다.
     *
     * v <= sqrt(2 * comfortable_deceleration * usable_distance)
     *
     * 이는 제어기에 직접 제동량을 주는 것이 아니라, 판단부가 정지선까지
     * 감속 가능한 상한 속도를 제공하는 방식이다.
     */
    if (!input.stop_line.valid)
    {
        return config_.approach_speed_mps;
    }

    const double usable_distance =
        input.stop_line.s - input.current_s - config_.stop_line_buffer_m;
    if (usable_distance <= 0.50)
    {
        return 0.0;
    }

    return std::min(
        config_.approach_speed_mps,
        std::sqrt(std::max(
            0.0,
            2.0 * config_.comfortable_deceleration_mps2 * usable_distance)));
}

std::vector<double> StaticTrafficMission::lateralTargets(
    const StaticTrafficInput &input) const
{
    /**
     * 도로 경계 안에서 차량 중심이 위치할 수 있는 d 후보를 만든다.
     *
     * 차량 중심 후보 한계:
     *   maximum_d = left_boundary - vehicle_half_width - margin
     *   minimum_d = right_boundary + vehicle_half_width + margin
     *
     * 따라서 후보 중심이 경계 안에 있다는 이유만으로 차량 외곽이 인도나
     * 반대 경계를 넘는 문제가 발생하지 않는다.
     */
    const double left_bound =
        input.road_bounds.valid
            ? input.road_bounds.left_d
            : config_.default_left_bound_m;
    const double right_bound =
        input.road_bounds.valid
            ? input.road_bounds.right_d
            : config_.default_right_bound_m;
    const double footprint_margin =
        0.5 * config_.vehicle_width_m + config_.boundary_margin_m;
    const double maximum_d = left_bound - footprint_margin;
    const double minimum_d = right_bound + footprint_margin;

    std::vector<double> targets;
    if (minimum_d > maximum_d ||
        config_.lateral_candidate_step_m <= 0.0)
    {
        return targets;
    }

    for (double d = minimum_d;
         d <= maximum_d + 1e-9;
         d += config_.lateral_candidate_step_m)
    {
        targets.push_back(d);
    }
    if (0.0 >= minimum_d && 0.0 <= maximum_d)
    {
        targets.push_back(0.0);
    }
    return targets;
}

bool StaticTrafficMission::pointWithinRoad(
    double d,
    const StaticTrafficInput &input) const
{
    // 후보 경로의 각 점에서 차량 전체 폭과 boundary margin을 적용한다.
    const double left_bound =
        input.road_bounds.valid
            ? input.road_bounds.left_d
            : config_.default_left_bound_m;
    const double right_bound =
        input.road_bounds.valid
            ? input.road_bounds.right_d
            : config_.default_right_bound_m;
    const double footprint_margin =
        0.5 * config_.vehicle_width_m + config_.boundary_margin_m;

    return d + footprint_margin <= left_bound + 1e-9 &&
           d - footprint_margin >= right_bound - 1e-9;
}

bool StaticTrafficMission::pathCollides(
    const std::vector<FrenetPoint> &path,
    const std::vector<ExpandedObstacle> &obstacles) const
{
    // 경로의 차량 중심점이 확장 장애물 사각형에 포함되면 충돌로 판정한다.
    // 장애물이 여러 개일 때 선택한 주 장애물뿐 아니라 모두 검사한다.
    for (const auto &point : path)
    {
        for (const auto &obstacle : obstacles)
        {
            if (point.s >= obstacle.minimum_s &&
                point.s <= obstacle.maximum_s &&
                std::abs(point.d - obstacle.center_d) <=
                    obstacle.lateral_half_width)
            {
                return true;
            }
        }
    }
    return false;
}

StaticTrafficMission::Candidate StaticTrafficMission::generateCandidate(
    const StaticTrafficInput &input,
    const ExpandedObstacle &primary_obstacle,
    const std::vector<ExpandedObstacle> &all_obstacles,
    double target_d,
    double early_merge_distance,
    double return_length,
    bool stop_required) const
{
    /**
     * 하나의 회피 후보 생성
     *
     * 구간 1: current_s → entry_end_s
     *   현재 d에서 target_d까지 5차 smooth-step으로 이동
     *
     * 구간 2: entry_end_s → obstacle.maximum_s
     *   target_d를 유지하며 장애물을 통과
     *
     * 구간 3: obstacle.maximum_s → return_end_s
     *   target_d에서 기준경로 d=0으로 5차 smooth-step 복귀
     *
     * early_merge_distance가 클수록 장애물보다 더 일찍 target_d에 도달해
     * 사전 차선변경 형태가 된다. return_length가 길수록 복귀 곡률이 작아진다.
     */
    Candidate candidate;
    candidate.target_d = target_d;
    candidate.side = target_d > 0.05 ? 1 : (target_d < -0.05 ? -1 : 0);

    const double entry_end_s =
        primary_obstacle.minimum_s -
        config_.obstacle_entry_buffer_m -
        std::max(0.0, early_merge_distance);
    const double entry_length = entry_end_s - input.current_s;
    if (entry_length < config_.minimum_entry_length_m)
    {
        // 장애물 직전에서 급하게 횡이동하는 경로를 만들지 않는다.
        candidate.rejection_reason = "insufficient entry length";
        return candidate;
    }

    const double pass_end_s = primary_obstacle.maximum_s;
    const double return_end_s = pass_end_s + std::max(1.0, return_length);
    double path_end_s = return_end_s;
    if (stop_required && input.stop_line.valid)
    {
        // 적색/황색/UNKNOWN 정지 조건에서는 후보 경로 끝을 정지선 버퍼
        // 이전으로 자른다. 회피 때문에 정지선을 넘어가는 것을 막는다.
        path_end_s =
            std::min(path_end_s,
                     input.stop_line.s - config_.stop_line_buffer_m);
    }
    if (path_end_s <= input.current_s + config_.path_step_m)
    {
        candidate.rejection_reason = "stop line leaves no path";
        return candidate;
    }

    const double return_span = std::max(1e-6, return_end_s - pass_end_s);
    for (double s = input.current_s;
         s <= path_end_s + 1e-9;
         s += config_.path_step_m)
    {
        double d = target_d;
        double d_first = 0.0;
        double d_second = 0.0;

        if (s <= entry_end_s)
        {
            // 진입 구간의 d, d'(s), d''(s). 시작과 끝에서 1·2차 미분이
            // 0이므로 기존 경로 및 오프셋 직선과 부드럽게 연결된다.
            const double u = (s - input.current_s) / entry_length;
            const double delta = target_d - input.current_d;
            d = input.current_d + delta * smoothStep5(u);
            d_first = delta * smoothStep5First(u) / entry_length;
            d_second =
                delta * smoothStep5Second(u) /
                (entry_length * entry_length);
        }
        else if (s > pass_end_s)
        {
            // 장애물 통과 후 기준경로 d=0으로 부드럽게 복귀한다.
            const double u = (s - pass_end_s) / return_span;
            d = target_d * (1.0 - smoothStep5(u));
            d_first =
                -target_d * smoothStep5First(u) / return_span;
            d_second =
                -target_d * smoothStep5Second(u) /
                (return_span * return_span);
        }

        // Frenet 그래프 d(s)의 곡률:
        // kappa = d'' / (1 + d'^2)^(3/2)
        const double denominator =
            std::pow(1.0 + d_first * d_first, 1.5);
        const double curvature =
            denominator > 1e-9 ? d_second / denominator : 0.0;

        if (!pointWithinRoad(d, input))
        {
            candidate.rejection_reason = "road boundary violation";
            return candidate;
        }
        if (std::abs(curvature) > config_.maximum_curvature_per_m)
        {
            candidate.rejection_reason = "curvature limit";
            return candidate;
        }

        candidate.maximum_curvature =
            std::max(candidate.maximum_curvature, std::abs(curvature));
        candidate.path.push_back(
            {s, d, std::atan(d_first), curvature});
    }

    if (pathCollides(candidate.path, all_obstacles))
    {
        candidate.rejection_reason = "expanded-obstacle collision";
        candidate.path.clear();
        return candidate;
    }

    const double side_change_penalty =
        selected_side_ != 0 && candidate.side != 0 &&
                candidate.side != selected_side_
            ? config_.side_change_cost
            : 0.0;
    /**
     * 후보 비용은 안전조건을 통과한 경로 사이의 선호도만 결정한다.
     * 충돌이나 경계 위반을 큰 비용으로 처리하지 않고 아예 후보에서
     * 제거하므로, 비용 가중치 튜닝 오류가 안전조건을 뒤집지 못한다.
     */
    candidate.cost =
        config_.lateral_offset_cost * std::abs(target_d) +
        config_.curvature_cost * candidate.maximum_curvature +
        config_.path_length_cost * (path_end_s - input.current_s) +
        side_change_penalty;
    candidate.valid = true;
    return candidate;
}

std::vector<FrenetPoint> StaticTrafficMission::buildNominalPath(
    const StaticTrafficInput &input,
    double end_s) const
{
    /**
     * 장애물이 없거나 신호 정지만 필요한 경우의 기준경로.
     * 차량이 현재 d=0에서 벗어나 있다면 end_s까지 부드럽게 중앙으로
     * 복귀한다. 정지 상태에서도 제어기에 빈 경로를 보내지 않기 위해
     * 최소 한 점을 반환한다.
     */
    std::vector<FrenetPoint> path;
    if (end_s <= input.current_s)
    {
        path.push_back({input.current_s, input.current_d, 0.0, 0.0});
        return path;
    }

    const double length = end_s - input.current_s;
    for (double s = input.current_s;
         s <= end_s + 1e-9;
         s += config_.path_step_m)
    {
        const double u = (s - input.current_s) / length;
        const double d = input.current_d * (1.0 - smoothStep5(u));
        const double d_first =
            -input.current_d * smoothStep5First(u) / length;
        const double d_second =
            -input.current_d * smoothStep5Second(u) / (length * length);
        const double curvature =
            d_second / std::pow(1.0 + d_first * d_first, 1.5);
        path.push_back({s, d, std::atan(d_first), curvature});
    }
    return path;
}

StaticTrafficOutput StaticTrafficMission::update(
    const StaticTrafficInput &input)
{
    /**
     * 한 주기의 전체 의사결정 순서
     *
     * 1. 미션 활성 여부 확인
     * 2. 장애물 트랙 갱신 및 확정 장애물 추출
     * 3. 신호 freshness와 녹색 연속 확인
     * 4. 기준경로를 막는 가장 가까운 장애물 선택
     * 5. 정지선이 장애물보다 먼저면 신호 정지
     * 6. 장애물이 너무 가까우면 긴급정지
     * 7. 여러 회피 후보 생성·검사·비용 비교
     * 8. 최적 경로와 곡률 기반 목표속도 출력
     */
    if (!input.mission_active)
    {
        if (state_ != StaticTrafficState::kIdle)
        {
            reset();
        }
        return {};
    }

    if (state_ == StaticTrafficState::kIdle)
    {
        state_ = StaticTrafficState::kApproach;
    }

    updateTracks(input);
    const auto signal = effectiveSignal(input);
    const bool stop_required = signalRequiresStop(input, signal);

    StaticTrafficOutput output;
    output.override_normal_planning = true;
    output.effective_signal = signal;
    output.target_speed_mps = config_.approach_speed_mps;

    auto confirmed = confirmedObstacles(input);
    std::vector<ExpandedObstacle> expanded;
    expanded.reserve(confirmed.size());
    for (const auto &obstacle : confirmed)
    {
        expanded.push_back(expandObstacle(obstacle));
    }

    std::sort(
        expanded.begin(),
        expanded.end(),
        [](const ExpandedObstacle &a, const ExpandedObstacle &b)
        {
            return a.minimum_s < b.minimum_s;
        });

    auto threat_it = std::find_if(
        expanded.begin(),
        expanded.end(),
        [&](const ExpandedObstacle &obstacle)
        {
            return obstacleThreatensReference(obstacle, input);
        });

    // 정지선이 첫 위협 장애물보다 먼저라면 신호 준수가 우선이다.
    // 이 경우 회피 경로를 만들지 않고 정지선 이전의 정상 경로를 보낸다.
    if (stop_required &&
        (threat_it == expanded.end() ||
         input.stop_line.s - config_.stop_line_buffer_m <=
             threat_it->minimum_s))
    {
        state_ = StaticTrafficState::kStopForSignal;
        output.state = state_;
        output.target_speed_mps = stoppingTargetSpeed(input);
        output.frenet_path =
            buildNominalPath(
                input,
                std::max(input.current_s,
                         input.stop_line.s - config_.stop_line_buffer_m));
        output.diagnostic =
            "signal stop: " + std::string(toString(signal));
        return output;
    }

    if (threat_it == expanded.end())
    {
        // 확정된 위협 장애물이 없으면 기존 회피 방향 latch를 해제한다.
        // 신호 정지가 필요하면 속도만 정지 가능 상한으로 제한한다.
        selected_side_ = 0;
        selected_obstacle_id_ = -1;
        state_ = StaticTrafficState::kApproach;
        output.state = state_;
        output.target_speed_mps =
            stop_required
                ? stoppingTargetSpeed(input)
                : config_.approach_speed_mps;
        const double nominal_end =
            stop_required && input.stop_line.valid
                ? input.stop_line.s - config_.stop_line_buffer_m
                : input.current_s + config_.detection_range_m;
        output.frenet_path =
            buildNominalPath(input, std::max(input.current_s, nominal_end));
        output.diagnostic = "no confirmed path-threatening obstacle";
        return output;
    }

    const auto &primary = *threat_it;
    const double forward_clearance = primary.minimum_s - input.current_s;
    output.selected_obstacle_id = primary.id;

    if (forward_clearance <= config_.emergency_stop_distance_m)
    {
        // 최소 회피 진입거리보다 훨씬 가까운 장애물에는 새 횡경로를
        // 생성하지 않는다. 급조한 회피보다 정지가 안전하다.
        state_ = StaticTrafficState::kSafeStop;
        output.state = state_;
        output.target_speed_mps = 0.0;
        output.frenet_path =
            buildNominalPath(input, input.current_s + 1.0);
        output.diagnostic = "emergency stop: obstacle too close";
        return output;
    }

    Candidate best;
    const auto targets = lateralTargets(input);
    for (const double target_d : targets)
    {
        // 횡 오프셋, 사전 합류 거리, 복귀 길이의 조합을 전부 평가한다.
        // Config 벡터만 바꾸면 후보 종류를 추가할 수 있어 로직 수정이 적다.
        for (const double early_merge : config_.early_merge_distances_m)
        {
            for (const double return_length : config_.return_lengths_m)
            {
                auto candidate =
                    generateCandidate(input,
                                      primary,
                                      expanded,
                                      target_d,
                                      early_merge,
                                      return_length,
                                      stop_required);
                if (candidate.valid && candidate.cost < best.cost)
                {
                    best = std::move(candidate);
                }
            }
        }
    }

    if (!best.valid)
    {
        // 회피 공간이 없거나 모든 후보가 충돌/경계/곡률 조건을 위반한 경우.
        state_ = StaticTrafficState::kSafeStop;
        output.state = state_;
        output.target_speed_mps = 0.0;
        output.frenet_path =
            buildNominalPath(input, input.current_s + 1.0);
        output.diagnostic = "safe stop: no collision-free candidate";
        return output;
    }

    selected_side_ = best.side;
    selected_obstacle_id_ = primary.id;
    state_ = input.current_s <= primary.maximum_s
                 ? StaticTrafficState::kAvoidance
                 : StaticTrafficState::kReturn;

    output.state = state_;
    output.frenet_path = std::move(best.path);
    output.selected_side = best.side;
    output.selected_obstacle_id = primary.id;
    output.candidate_cost = best.cost;

    double curvature_speed = config_.avoidance_speed_mps;
    if (best.maximum_curvature > 1e-6)
    {
        // 횡가속도 a_y = v^2 * kappa 제한에서 허용속도를 역산한다.
        // v <= sqrt(max_lateral_acceleration / max_curvature)
        curvature_speed =
            std::sqrt(config_.maximum_lateral_acceleration_mps2 /
                      best.maximum_curvature);
    }
    output.target_speed_mps =
        std::min(config_.avoidance_speed_mps, curvature_speed);
    if (stop_required)
    {
        output.target_speed_mps =
            std::min(output.target_speed_mps, stoppingTargetSpeed(input));
    }

    std::ostringstream diagnostic;
    diagnostic << "avoid obstacle=" << primary.id
               << ", side=" << best.side
               << ", cost=" << best.cost
               << ", max_curvature=" << best.maximum_curvature;
    output.diagnostic = diagnostic.str();
    return output;
}

const char *toString(StaticTrafficState state)
{
    switch (state)
    {
    case StaticTrafficState::kIdle:
        return "IDLE";
    case StaticTrafficState::kApproach:
        return "APPROACH";
    case StaticTrafficState::kStopForSignal:
        return "STOP_FOR_SIGNAL";
    case StaticTrafficState::kAvoidance:
        return "AVOIDANCE";
    case StaticTrafficState::kReturn:
        return "RETURN";
    case StaticTrafficState::kDone:
        return "DONE";
    case StaticTrafficState::kSafeStop:
        return "SAFE_STOP";
    }
    return "UNKNOWN";
}

const char *toString(TrafficSignal signal)
{
    switch (signal)
    {
    case TrafficSignal::kUnknown:
        return "UNKNOWN";
    case TrafficSignal::kRed:
        return "RED";
    case TrafficSignal::kYellow:
        return "YELLOW";
    case TrafficSignal::kGreen:
        return "GREEN";
    }
    return "UNKNOWN";
}

} // namespace mission_2026
