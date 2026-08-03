/**
 * @brief verC: 혼용 동적 장애물 회피 미션 구현 (FrenetFrame 기반)
 *
 * /LiDAR/car_dis (거리)와 /LiDAR/dynamic_obstacle_pos (velodyne 좌표 [x, y])를 활용하여
 * 전방 동적 장애물을 감지하고, TTC(Time To Collision)를 추정하여 다음 4가지 동작을 혼합 수행한다.
 *
 * 1. TTC < 2초: 긴급 정지
 * 2. 2초 <= TTC <= 5초 + 회피 공간 있음: FrenetFrame 기반 회피 경로 생성
 * 3. 장애물 UTM 절대위치가 3초간 유지: FrenetFrame 기반 회피
 * 4. 회피 경로 생성 실패: 정지 유지 후 1초 간격 재시도
 *
 * 추가로 velodyne 좌표를 UTM으로 변환하고 전방 및 측면 진입 영역 필터를 적용하여,
 * ROI 외곽의 잡음은 제외하면서 차로로 진입하는 보행자는 미리 판단한다.
 */

#include "dynamic_obstacle_stop.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <ros/ros.h>
#include <sstream>

using namespace std::chrono;

DynamicObstacleStop::DynamicObstacleStop(Local &local, Control &control,
                                         Path &local_path, Lidar &lidar,
                                         Vision &vision,
                                         MissionData &mission_data,
                                         const nlohmann::json &traffic_param)
    : local_(local), control_(control), local_path_(local_path), lidar_(lidar),
      vision_(vision), mission_data_(mission_data),
      target_vel_(0.0), vel_flag_(true), state_(DynamicObstacleState::INIT),
      next_obstacle_track_id_(1), last_dynamic_obstacle_generation_(0),
      is_waiting_(false), obstacle_lost_pending_(false),
      side_collision_stop_(false), last_traffic_light_generation_(0),
      traffic_stop_latched_(false),
      raw_traffic_stop_lines_(traffic_param.value(
          "stop_line_list", std::vector<std::vector<double>>{})),
      traffic_stop_lines_initialized_(false),
      traffic_check_distance_(traffic_param.value("tf_check_distance_", 18.0)),
      traffic_stop_distance_(traffic_param.value("stop_point_distance", 4.5)),
      traffic_slow_velocity_(traffic_param.value("slow_velocity", 3.0)),
      traffic_done_distance_(traffic_param.value("done_distance", 11.0)),
      traffic_max_projection_distance_(
          traffic_param.value("dynamic_path_projection_distance", 35.0)),
      is_avoiding_(false), stationary_avoidance_(false),
      avoidance_direction_(0)
{
}

double DynamicObstacleStop::judgeDynamicObstacleVelocity(double distance, double desired_vel)
{
    // 장애물 미감지: 인지 코드에서 25.0 또는 음수를 발행
    if (distance < 0.0 || distance >= kNoObstacleDistance)
    {
        return desired_vel;
    }

    // 정지 거리 이내: 완전 정지
    if (distance < kStopDistance)
    {
        std::cout << "[Dynamic Obstacle] STOP! distance = " << distance << " m" << std::endl;
        return 0.0;
    }

    // 감속 거리 이내: 감속
    if (distance < kSlowDistance)
    {
        std::cout << "[Dynamic Obstacle] SLOW! distance = " << distance << " m" << std::endl;
        return desired_vel * kSlowRatio;
    }

    // 그 외: 정상 주행
    return desired_vel;
}

double DynamicObstacleStop::computeClosureRate() const
{
    if (distance_history_.size() < kMinHistorySize)
    {
        return 0.0;
    }

    const auto &first = distance_history_.front();
    const auto &last = distance_history_.back();
    double dt = duration_cast<duration<double>>(last.time - first.time).count();

    if (dt < kMinHistoryDuration)
    {
        return 0.0;
    }

    // closure rate: 양수면 접근 중, 음수면 멀어짐
    return -(last.distance - first.distance) / dt;
}

double DynamicObstacleStop::computeTTC(double distance) const
{
    double closure_rate = computeClosureRate();

    // 접근 중이 아니면 큰 값 반환 (TTC 임계값보다 크게)
    if (closure_rate <= 0.0)
    {
        return kAvoidTTCMax + 1.0;
    }

    return distance / closure_rate;
}

bool DynamicObstacleStop::isObstacleStopped(double distance) const
{
    if (distance < 0.0 || distance > kStationaryAvoidDistance ||
        obstacle_position_history_.size() < 2)
    {
        return false;
    }

    if (getStationaryObservationDuration() < kStationaryConfirmTime)
    {
        return false;
    }

    double min_x = obstacle_position_history_.front().x;
    double max_x = min_x;
    double min_y = obstacle_position_history_.front().y;
    double max_y = min_y;
    double min_relative_x = obstacle_position_history_.front().relative_x;
    double max_relative_x = min_relative_x;
    double min_relative_y = obstacle_position_history_.front().relative_y;
    double max_relative_y = min_relative_y;
    for (const auto &sample : obstacle_position_history_)
    {
        min_x = std::min(min_x, sample.x);
        max_x = std::max(max_x, sample.x);
        min_y = std::min(min_y, sample.y);
        max_y = std::max(max_y, sample.y);
        min_relative_x = std::min(min_relative_x, sample.relative_x);
        max_relative_x = std::max(max_relative_x, sample.relative_x);
        min_relative_y = std::min(min_relative_y, sample.relative_y);
        max_relative_y = std::max(max_relative_y, sample.relative_y);
    }

    const double utm_spread = std::hypot(max_x - min_x, max_y - min_y);
    const double relative_spread =
        std::hypot(max_relative_x - min_relative_x,
                   max_relative_y - min_relative_y);
    const bool ego_stopped =
        std::abs(local_.getCurCarVelocity()) <= kStoppedEgoVelocity;

    return utm_spread <= kStationaryPositionTolerance ||
           (ego_stopped &&
            relative_spread <= kStationaryRelativeTolerance);
}

void DynamicObstacleStop::updateObstaclePositionHistory(
    const std::vector<double> &obstacle_utm,
    double relative_x, double relative_y)
{
    if (obstacle_utm.size() < 2 ||
        !std::isfinite(obstacle_utm[0]) || !std::isfinite(obstacle_utm[1]) ||
        !std::isfinite(relative_x) || !std::isfinite(relative_y))
    {
        obstacle_position_history_.clear();
        return;
    }

    const auto now = steady_clock::now();
    if (!obstacle_position_history_.empty())
    {
        const auto &last = obstacle_position_history_.back();
        const double observation_gap =
            duration_cast<duration<double>>(now - last.time).count();
        const double position_jump =
            std::hypot(obstacle_utm[0] - last.x, obstacle_utm[1] - last.y);

        // 인지가 끊겼거나 선택된 클러스터가 바뀐 경우 3초 타이머를 다시 시작한다.
        if (observation_gap > kObstacleObservationGap ||
            position_jump > kObstacleTrackJumpDistance)
        {
            obstacle_position_history_.clear();
        }
    }

    obstacle_position_history_.push_back(
        {obstacle_utm[0], obstacle_utm[1], relative_x, relative_y, now});

    const double history_window = kStationaryConfirmTime + 0.1;
    while (!obstacle_position_history_.empty() &&
           duration_cast<duration<double>>(
               now - obstacle_position_history_.front().time).count() > history_window)
    {
        obstacle_position_history_.pop_front();
    }
    while (obstacle_position_history_.size() > kMaxPositionHistorySize)
    {
        obstacle_position_history_.pop_front();
    }
}

void DynamicObstacleStop::updateSideObstacleTracks(
    const std::vector<std::vector<double>> &relative_obstacles)
{
    const auto now = steady_clock::now();

    // 일정 시간 보이지 않은 궤적은 먼저 제거한다.
    side_obstacle_tracks_.erase(
        std::remove_if(
            side_obstacle_tracks_.begin(), side_obstacle_tracks_.end(),
            [&now](const ObstacleTrack &track)
            {
                return duration_cast<duration<double>>(
                           now - track.last_update).count() > kTrackStaleTime;
            }),
        side_obstacle_tracks_.end());

    const std::size_t existing_track_count = side_obstacle_tracks_.size();
    std::vector<bool> matched(existing_track_count, false);

    for (const auto &relative_obstacle : relative_obstacles)
    {
        if (relative_obstacle.size() < 2)
        {
            continue;
        }

        const std::vector<double> obstacle_utm = convertObstacleToUTM(
            relative_obstacle[0], relative_obstacle[1]);
        if (obstacle_utm.size() < 2)
        {
            continue;
        }

        std::size_t best_track = existing_track_count;
        double best_distance = kTrackAssociationDistance;
        for (std::size_t index = 0; index < existing_track_count; ++index)
        {
            if (matched[index] || side_obstacle_tracks_[index].history.empty())
            {
                continue;
            }

            const auto &last = side_obstacle_tracks_[index].history.back();
            const double association_distance = std::hypot(
                obstacle_utm[0] - last.x, obstacle_utm[1] - last.y);
            if (association_distance < best_distance)
            {
                best_distance = association_distance;
                best_track = index;
            }
        }

        const ObstaclePositionSample sample{
            obstacle_utm[0], obstacle_utm[1],
            relative_obstacle[0], relative_obstacle[1], now};

        if (best_track < existing_track_count)
        {
            auto &track = side_obstacle_tracks_[best_track];
            matched[best_track] = true;
            track.history.push_back(sample);
            track.last_update = now;
        }
        else if (side_obstacle_tracks_.size() < kMaxSideTracks)
        {
            ObstacleTrack track;
            track.id = next_obstacle_track_id_++;
            track.history.push_back(sample);
            track.last_update = now;
            side_obstacle_tracks_.push_back(std::move(track));
        }
    }

    // 속도 계산에 필요한 최근 구간만 보관한다.
    for (auto &track : side_obstacle_tracks_)
    {
        while (!track.history.empty() &&
               duration_cast<duration<double>>(
                   now - track.history.front().time).count() >
                   kSideVelocityWindow + 0.2)
        {
            track.history.pop_front();
        }
    }
}

double DynamicObstacleStop::getStationaryObservationDuration() const
{
    if (obstacle_position_history_.size() < 2)
    {
        return 0.0;
    }

    return duration_cast<duration<double>>(
               obstacle_position_history_.back().time -
               obstacle_position_history_.front().time)
        .count();
}

bool DynamicObstacleStop::isFrontalObstacle(double velodyne_x, double velodyne_y) const
{
    double forward_path_distance = 0.0;
    double lateral_path_distance = std::numeric_limits<double>::infinity();
    const bool on_driving_path = getObstaclePathRelation(
        velodyne_x, velodyne_y,
        forward_path_distance, lateral_path_distance);

    if (!on_driving_path)
    {
        ROS_INFO_THROTTLE(
            0.5,
            "[DynamicObstacleStop] OFF-PATH obstacle ignored: lidar=(%.2f, %.2f), path_forward=%.2f m, path_lateral=%.2f m",
            velodyne_x, velodyne_y,
            forward_path_distance, lateral_path_distance);
    }
    else
    {
        ROS_INFO_THROTTLE(
            0.5,
            "[DynamicObstacleStop] ON-PATH obstacle: lidar=(%.2f, %.2f), path_forward=%.2f m, path_lateral=%.2f m",
            velodyne_x, velodyne_y,
            forward_path_distance, lateral_path_distance);
    }
    return on_driving_path;
}

bool DynamicObstacleStop::isObservedObstacle(double velodyne_x, double velodyne_y) const
{
    return std::isfinite(velodyne_x) && std::isfinite(velodyne_y) &&
           velodyne_x > kObservationMinX &&
           std::abs(velodyne_y) < kObservationHalfWidth;
}

bool DynamicObstacleStop::getObstaclePathRelation(
    double velodyne_x, double velodyne_y,
    double &forward_path_distance,
    double &lateral_path_distance) const
{
    forward_path_distance = 0.0;
    lateral_path_distance = std::numeric_limits<double>::infinity();
    if (!isObservedObstacle(velodyne_x, velodyne_y))
    {
        return false;
    }

    const double *car_utm = local_.getAddCurCarUTMPos();
    const double car_yaw = local_.getCurCarYaw();
    const double obstacle_utm[2] = {
        car_utm[0] + velodyne_x * std::cos(car_yaw) -
            velodyne_y * std::sin(car_yaw),
        car_utm[1] + velodyne_x * std::sin(car_yaw) +
            velodyne_y * std::cos(car_yaw)};

    // 컨트롤러가 실제 추종하는 local_path를 우선 사용한다. 미션 초기화 전처럼
    // local_path가 비어 있으면 global_path로 안전하게 fallback한다.
    const Path *driving_path = &local_path_;
    const auto *path_pos = &local_path_.getRefPosArr();
    const auto *path_yaw = &local_path_.getRefYawArr();
    if (path_pos->size() < 2 || path_yaw->size() != path_pos->size())
    {
        driving_path = &local_.getRefCurGlobalPath();
        path_pos = &driving_path->getRefPosArr();
        path_yaw = &driving_path->getRefYawArr();
    }
    if (path_pos->size() < 2 || path_yaw->size() != path_pos->size())
    {
        return false;
    }

    const int car_index = driving_path->getClosestIndex(car_utm);
    const int obstacle_index = driving_path->getClosestIndex(obstacle_utm);
    if (car_index < 0 || obstacle_index < 0 ||
        static_cast<std::size_t>(car_index) >= path_pos->size() ||
        static_cast<std::size_t>(obstacle_index) >= path_pos->size())
    {
        return false;
    }

    lateral_path_distance = std::hypot(
        obstacle_utm[0] - (*path_pos)[obstacle_index][0],
        obstacle_utm[1] - (*path_pos)[obstacle_index][1]);

    if (obstacle_index >= car_index)
    {
        for (int index = car_index + 1; index <= obstacle_index; ++index)
        {
            forward_path_distance += std::hypot(
                (*path_pos)[index][0] - (*path_pos)[index - 1][0],
                (*path_pos)[index][1] - (*path_pos)[index - 1][1]);
        }
    }
    else
    {
        // KD-tree가 곡선이나 근접 평행구간에서 바로 이전 인덱스를 선택할 수
        // 있으므로 차량 위치의 경로 접선 투영으로 앞/뒤를 재확인한다.
        const double path_heading = (*path_yaw)[car_index];
        forward_path_distance =
            (obstacle_utm[0] - car_utm[0]) * std::cos(path_heading) +
            (obstacle_utm[1] - car_utm[1]) * std::sin(path_heading);
    }

    return forward_path_distance >= -kPathBehindTolerance &&
           lateral_path_distance <= kPathCollisionHalfWidth;
}

std::vector<double> DynamicObstacleStop::convertObstacleToUTM(
    double velodyne_x, double velodyne_y) const
{
    if (!isObservedObstacle(velodyne_x, velodyne_y))
    {
        return {};
    }

    const double *car_utm = local_.getAddCurCarUTMPos();
    const double car_yaw = local_.getCurCarYaw();
    return {
        car_utm[0] + velodyne_x * std::cos(car_yaw) - velodyne_y * std::sin(car_yaw),
        car_utm[1] + velodyne_x * std::sin(car_yaw) + velodyne_y * std::cos(car_yaw)};
}

std::vector<std::vector<double>> DynamicObstacleStop::getFrontalObstaclesInUTM() const
{
    std::vector<std::pair<double, std::vector<double>>> ordered_obstacles;

    const auto &obs_pos = lidar_.getRefDynamicObstaclePos();
    if (obs_pos.size() < 2)
    {
        return {};
    }

    for (std::size_t index = 0; index + 1 < obs_pos.size(); index += 2)
    {
        const double velodyne_x = obs_pos[index];
        const double velodyne_y = obs_pos[index + 1];
        double forward_path_distance = 0.0;
        double lateral_path_distance = std::numeric_limits<double>::infinity();
        if (!getObstaclePathRelation(
                velodyne_x, velodyne_y,
                forward_path_distance, lateral_path_distance) ||
            forward_path_distance < 0.0)
        {
            continue;
        }

        const std::vector<double> obstacle_utm =
            convertObstacleToUTM(velodyne_x, velodyne_y);
        if (obstacle_utm.size() < 2)
        {
            continue;
        }
        ordered_obstacles.push_back({forward_path_distance, obstacle_utm});
    }

    std::sort(
        ordered_obstacles.begin(), ordered_obstacles.end(),
        [](const auto &left, const auto &right)
        {
            return left.first < right.first;
        });

    std::vector<std::vector<double>> utm_obstacles;
    utm_obstacles.reserve(ordered_obstacles.size());
    for (const auto &obstacle : ordered_obstacles)
    {
        utm_obstacles.push_back(obstacle.second);
    }

    ROS_INFO_THROTTLE(
        1.0,
        "[DynamicObstacleStop] %zu on-path obstacle(s) selected from %zu LiDAR cluster(s)",
        utm_obstacles.size(), obs_pos.size() / 2);

    return utm_obstacles;
}

bool DynamicObstacleStop::predictSideCollision(
    double planned_velocity,
    double &collision_time,
    double &closest_distance,
    double &relative_velocity_x,
    double &relative_velocity_y) const
{
    collision_time = std::numeric_limits<double>::infinity();
    closest_distance = std::numeric_limits<double>::infinity();
    relative_velocity_x = 0.0;
    relative_velocity_y = 0.0;

    bool collision_risk = false;
    std::uint64_t risk_track_id = 0;

    for (const auto &track : side_obstacle_tracks_)
    {
        if (track.history.size() < 2)
        {
            continue;
        }

        const auto &last = track.history.back();
        auto first_it = track.history.begin();
        for (auto it = track.history.begin(); it != track.history.end(); ++it)
        {
            const double age = duration_cast<duration<double>>(
                                   last.time - it->time).count();
            if (age <= kSideVelocityWindow)
            {
                first_it = it;
                break;
            }
        }

        const double dt = duration_cast<duration<double>>(
                              last.time - first_it->time).count();
        if (dt < kSideMinHistoryDuration)
        {
            continue;
        }

        const double obstacle_velocity_utm_x =
            (last.x - first_it->x) / dt;
        const double obstacle_velocity_utm_y =
            (last.y - first_it->y) / dt;
        const double car_yaw = local_.getCurCarYaw();
        const double obstacle_velocity_forward =
            obstacle_velocity_utm_x * std::cos(car_yaw) +
            obstacle_velocity_utm_y * std::sin(car_yaw);
        const double obstacle_velocity_lateral =
            -obstacle_velocity_utm_x * std::sin(car_yaw) +
            obstacle_velocity_utm_y * std::cos(car_yaw);

        const double candidate_velocity_x =
            obstacle_velocity_forward - std::max(0.0, planned_velocity);
        const double candidate_velocity_y = obstacle_velocity_lateral;
        if (std::abs(candidate_velocity_y) < kSideMinLateralSpeed ||
            last.relative_y * candidate_velocity_y >= 0.0)
        {
            continue;
        }

        const double relative_speed_squared =
            candidate_velocity_x * candidate_velocity_x +
            candidate_velocity_y * candidate_velocity_y;
        if (relative_speed_squared < 1e-6)
        {
            continue;
        }

        const double candidate_collision_time = -(
            last.relative_x * candidate_velocity_x +
            last.relative_y * candidate_velocity_y) /
            relative_speed_squared;
        if (candidate_collision_time < 0.0 ||
            candidate_collision_time > kSidePredictionHorizon)
        {
            continue;
        }

        const double predicted_x =
            last.relative_x + candidate_velocity_x * candidate_collision_time;
        const double predicted_y =
            last.relative_y + candidate_velocity_y * candidate_collision_time;
        const double candidate_closest_distance =
            std::hypot(predicted_x, predicted_y);
        if (predicted_x < -1.0 ||
            candidate_closest_distance > kSideCollisionDistance)
        {
            continue;
        }

        if (!collision_risk || candidate_collision_time < collision_time)
        {
            collision_risk = true;
            risk_track_id = track.id;
            collision_time = candidate_collision_time;
            closest_distance = candidate_closest_distance;
            relative_velocity_x = candidate_velocity_x;
            relative_velocity_y = candidate_velocity_y;
        }
    }

    if (collision_risk)
    {
        ROS_WARN_THROTTLE(
            0.25,
            "[DynamicObstacleStop] side track=%llu collision candidate: TTC=%.2f s, CPA=%.2f m",
            static_cast<unsigned long long>(risk_track_id),
            collision_time, closest_distance);
    }
    return collision_risk;
}

bool DynamicObstacleStop::hasAvoidanceSpace() const
{
    // FrenetFrame 기반으로 회피 경로를 생성할 수 있으면 true
    // 실제 회피 가능 여부는 FrenetFrame이 valid 경로를 반환하는지로 판단
    return true;
}

bool DynamicObstacleStop::buildFallbackAvoidancePath(
    const std::vector<double> &obstacle_utm,
    std::vector<std::vector<double>> &avoid_pos,
    std::vector<double> &avoid_yaw,
    std::vector<double> &avoid_k) const
{
    if (obstacle_utm.size() < 2)
    {
        return false;
    }

    const Path &global_path = local_.getRefCurGlobalPath();
    const auto &global_pos = global_path.getRefPosArr();
    const auto &global_yaw = global_path.getRefYawArr();
    if (global_pos.size() < 3 || global_yaw.size() != global_pos.size())
    {
        return false;
    }

    const double *car_utm = local_.getAddCurCarUTMPos();
    double obstacle_position[2] = {obstacle_utm[0], obstacle_utm[1]};
    const int car_index = global_path.getClosestIndex(car_utm);
    const int obstacle_index = global_path.getClosestIndex(obstacle_position);
    if (car_index < 0 || obstacle_index < 0 ||
        static_cast<std::size_t>(car_index) >= global_pos.size() ||
        static_cast<std::size_t>(obstacle_index) >= global_pos.size())
    {
        ROS_ERROR("[DynamicObstacleStop] Fallback path rejected: invalid global-path index");
        return false;
    }

    int avoidance_obstacle_index = obstacle_index;
    double obstacle_s = 0.0;
    if (obstacle_index > car_index)
    {
        for (int index = car_index + 1; index <= obstacle_index; ++index)
        {
            obstacle_s += std::hypot(
                global_pos[index][0] - global_pos[index - 1][0],
                global_pos[index][1] - global_pos[index - 1][1]);
        }
    }
    else
    {
        // 장애물이 경로 옆에 있으면 KD-tree 최근접 인덱스가 차량과
        // 같거나 한 점 뒤로 잡힐 수 있다. 차량 앞쪽에 있는지를 경로
        // 접선 투영거리로 한 번 더 판단해 fallback 회피를 허용한다.
        const double car_path_yaw = global_yaw[car_index];
        const double forward_distance =
            (obstacle_utm[0] - car_utm[0]) * std::cos(car_path_yaw) +
            (obstacle_utm[1] - car_utm[1]) * std::sin(car_path_yaw);
        if (forward_distance <= 0.0)
        {
            ROS_ERROR("[DynamicObstacleStop] Fallback path rejected: obstacle is behind vehicle");
            return false;
        }

        obstacle_s = forward_distance;
        avoidance_obstacle_index = car_index;
        double accumulated_s = 0.0;
        while (static_cast<std::size_t>(avoidance_obstacle_index + 1) < global_pos.size() &&
               accumulated_s < obstacle_s)
        {
            ++avoidance_obstacle_index;
            accumulated_s += std::hypot(
                global_pos[avoidance_obstacle_index][0] -
                    global_pos[avoidance_obstacle_index - 1][0],
                global_pos[avoidance_obstacle_index][1] -
                    global_pos[avoidance_obstacle_index - 1][1]);
        }
    }
    if (obstacle_s <= kFallbackMinClearance)
    {
        ROS_ERROR("[DynamicObstacleStop] Fallback path rejected: obstacle too close (%.2f m along path)",
                  obstacle_s);
        return false;
    }

    const double obstacle_path_yaw = global_yaw[avoidance_obstacle_index];
    const double normal_x = -std::sin(obstacle_path_yaw);
    const double normal_y = std::cos(obstacle_path_yaw);
    const double obstacle_lateral =
        (obstacle_utm[0] - global_pos[avoidance_obstacle_index][0]) * normal_x +
        (obstacle_utm[1] - global_pos[avoidance_obstacle_index][1]) * normal_y;
    // 장애물이 왼쪽에 있으면 오른쪽, 오른쪽에 있으면 왼쪽으로 회피한다.
    const double avoidance_offset =
        (obstacle_lateral > 0.2 ? -1.0 : 1.0) * kFallbackAvoidanceOffset;

    const double avoid_start =
        std::max(0.0, obstacle_s - kFallbackTransitionDistance);
    const double full_offset_start = std::max(avoid_start + 0.5, obstacle_s - 1.0);
    const double full_offset_end = obstacle_s + kFallbackHoldDistance;
    const double avoid_end = full_offset_end + kFallbackRecoveryDistance;

    const auto smooth_step = [](double value)
    {
        const double clamped = std::max(0.0, std::min(1.0, value));
        return clamped * clamped * (3.0 - 2.0 * clamped);
    };

    avoid_pos.clear();
    avoid_pos.reserve(global_pos.size() - static_cast<std::size_t>(car_index));
    double path_s = 0.0;
    for (std::size_t index = static_cast<std::size_t>(car_index);
         index < global_pos.size(); ++index)
    {
        if (index > static_cast<std::size_t>(car_index))
        {
            path_s += std::hypot(
                global_pos[index][0] - global_pos[index - 1][0],
                global_pos[index][1] - global_pos[index - 1][1]);
        }

        double offset_ratio = 0.0;
        if (path_s >= avoid_start && path_s < full_offset_start)
        {
            offset_ratio = smooth_step(
                (path_s - avoid_start) /
                std::max(0.1, full_offset_start - avoid_start));
        }
        else if (path_s >= full_offset_start && path_s <= full_offset_end)
        {
            offset_ratio = 1.0;
        }
        else if (path_s > full_offset_end && path_s < avoid_end)
        {
            offset_ratio = 1.0 - smooth_step(
                (path_s - full_offset_end) /
                std::max(0.1, avoid_end - full_offset_end));
        }

        const double path_normal_x = -std::sin(global_yaw[index]);
        const double path_normal_y = std::cos(global_yaw[index]);
        const double offset = avoidance_offset * offset_ratio;
        avoid_pos.push_back({
            global_pos[index][0] + path_normal_x * offset,
            global_pos[index][1] + path_normal_y * offset});
    }

    if (avoid_pos.size() < 3)
    {
        return false;
    }

    double minimum_clearance = std::numeric_limits<double>::infinity();
    for (const auto &point : avoid_pos)
    {
        minimum_clearance = std::min(
            minimum_clearance,
            std::hypot(point[0] - obstacle_utm[0],
                       point[1] - obstacle_utm[1]));
    }
    if (minimum_clearance < kFallbackMinClearance)
    {
        ROS_ERROR("[DynamicObstacleStop] Fallback path rejected: clearance %.2f < %.2f m",
                  minimum_clearance, kFallbackMinClearance);
        avoid_pos.clear();
        return false;
    }

    avoid_yaw.assign(avoid_pos.size(), 0.0);
    avoid_k.assign(avoid_pos.size(), 0.0);
    const auto normalize_angle = [](double angle)
    {
        while (angle > kPi)
            angle -= 2.0 * kPi;
        while (angle < -kPi)
            angle += 2.0 * kPi;
        return angle;
    };

    for (std::size_t index = 0; index + 1 < avoid_pos.size(); ++index)
    {
        avoid_yaw[index] = std::atan2(
            avoid_pos[index + 1][1] - avoid_pos[index][1],
            avoid_pos[index + 1][0] - avoid_pos[index][0]);
    }
    avoid_yaw.back() = avoid_yaw[avoid_yaw.size() - 2];

    for (std::size_t index = 0; index + 1 < avoid_pos.size(); ++index)
    {
        const double segment_length = std::hypot(
            avoid_pos[index + 1][0] - avoid_pos[index][0],
            avoid_pos[index + 1][1] - avoid_pos[index][1]);
        if (segment_length > 1e-6)
        {
            avoid_k[index] =
                normalize_angle(avoid_yaw[index + 1] - avoid_yaw[index]) /
                segment_length;
        }
    }
    avoid_k.back() = avoid_k[avoid_k.size() - 2];

    ROS_WARN("[DynamicObstacleStop] FALLBACK avoidance path generated: offset=%.2f m, clearance=%.2f m",
             avoidance_offset, minimum_clearance);
    return true;
}

bool DynamicObstacleStop::generateAvoidancePath(bool stationary_triggered)
{
    if (is_avoiding_)
    {
        return true;
    }

    // 동적 장애물 좌표를 UTM으로 변환
    std::vector<std::vector<double>> obstacles = getFrontalObstaclesInUTM();
    if (obstacles.empty())
    {
        ROS_ERROR("[DynamicObstacleStop] Cannot generate avoidance path: no valid obstacle UTM");
        return false;
    }

    // FrenetFrame 기준 경로 설정 (global_path 기준)
    frenet_frame_.setDefault(local_.getRefCurGlobalPath(), obstacles);

    // 회피 경로 생성
    const double *cur_utm = local_.getAddCurCarUTMPos();
    const double *cur_accel = local_.getAddCurCarUTMAcceleration();
    double cur_velocity = local_.getCurCarVelocity();
    double cur_yaw = local_.getCurCarYaw();

    Path avoid_path = frenet_frame_.getOptimalTrajectory(cur_utm, cur_velocity, cur_accel, cur_yaw, obstacles);

    std::vector<std::vector<double>> avoid_pos = avoid_path.getRefPosArr();
    std::vector<double> avoid_yaw = avoid_path.getRefYawArr();
    std::vector<double> avoid_k = avoid_path.getRefKArr();
    bool frenet_path_valid =
        avoid_pos.size() >= 2 && avoid_yaw.size() == avoid_pos.size() &&
        avoid_k.size() == avoid_pos.size();

    // 생성된 Frenet 경로가 장애물 앞에서 끝나는 경우도 실패로 본다.
    // 장애물을 실제로 통과하며 최소 간격을 만족해야 회피 경로로 채택한다.
    if (frenet_path_valid)
    {
        const Path &global_path = local_.getRefCurGlobalPath();
        double obstacle_position[2] = {obstacles.front()[0], obstacles.front()[1]};
        const int obstacle_index = global_path.getClosestIndex(obstacle_position);
        const auto &global_pos = global_path.getRefPosArr();
        const auto &global_yaw = global_path.getRefYawArr();
        if (obstacle_index < 0 ||
            static_cast<std::size_t>(obstacle_index) >= global_pos.size() ||
            global_yaw.size() != global_pos.size())
        {
            frenet_path_valid = false;
        }
        else
        {
            double minimum_clearance = std::numeric_limits<double>::infinity();
            for (const auto &point : avoid_pos)
            {
                minimum_clearance = std::min(
                    minimum_clearance,
                    std::hypot(point[0] - obstacles.front()[0],
                               point[1] - obstacles.front()[1]));
            }

            const double tangent_x = std::cos(global_yaw[obstacle_index]);
            const double tangent_y = std::sin(global_yaw[obstacle_index]);
            const double passed_distance =
                (avoid_pos.back()[0] - obstacles.front()[0]) * tangent_x +
                (avoid_pos.back()[1] - obstacles.front()[1]) * tangent_y;
            frenet_path_valid =
                minimum_clearance >= kFallbackMinClearance &&
                passed_distance >= 1.0;

            if (!frenet_path_valid)
            {
                ROS_WARN("[DynamicObstacleStop] Frenet path incomplete: clearance=%.2f m, passed=%.2f m",
                         minimum_clearance, passed_distance);
            }
        }
    }

    if (!frenet_path_valid)
    {
        ROS_WARN("[DynamicObstacleStop] Invalid/empty Frenet path; try fallback avoidance path");
        if (!buildFallbackAvoidancePath(
                obstacles.front(), avoid_pos, avoid_yaw, avoid_k))
        {
            ROS_ERROR("[DynamicObstacleStop] Frenet/fallback avoidance failed; keep stopped");
            return false;
        }
    }

    // 유효한 회피 경로가 생성된 뒤에만 원본 경로를 백업하고 교체한다.
    original_pos_arr_ = local_path_.getRefPosArr();
    original_yaw_arr_ = local_path_.getRefYawArr();
    original_k_arr_ = local_path_.getRefKArr();

    // local_path_를 회피 경로로 교체
    local_path_.setPosArr(avoid_pos);
    local_path_.setYawArr(avoid_yaw);
    local_path_.setKArr(avoid_k);
    local_path_.updateKDTree();

    is_avoiding_ = true;
    stationary_avoidance_ = stationary_triggered;
    active_avoidance_obstacle_utm_ = obstacles.front();
    obstacle_lost_pending_ = false;

    std::cout << "[DynamicObstacleStop] FrenetFrame AVOID path generated"
              << (stationary_triggered ? " (stationary obstacle)" : "")
              << std::endl;
    return true;
}

bool DynamicObstacleStop::hasPassedAvoidanceObstacle() const
{
    if (active_avoidance_obstacle_utm_.size() < 2)
    {
        return false;
    }

    const Path &global_path = local_.getRefCurGlobalPath();
    const auto &global_pos = global_path.getRefPosArr();
    const auto &global_yaw = global_path.getRefYawArr();
    if (global_pos.empty() || global_yaw.size() != global_pos.size())
    {
        return false;
    }

    double obstacle_position[2] = {
        active_avoidance_obstacle_utm_[0],
        active_avoidance_obstacle_utm_[1]};
    const int obstacle_index = global_path.getClosestIndex(obstacle_position);
    if (obstacle_index < 0 ||
        static_cast<std::size_t>(obstacle_index) >= global_yaw.size())
    {
        return false;
    }

    const double *car_utm = local_.getAddCurCarUTMPos();
    const double tangent_x = std::cos(global_yaw[obstacle_index]);
    const double tangent_y = std::sin(global_yaw[obstacle_index]);
    const double passed_distance =
        (car_utm[0] - active_avoidance_obstacle_utm_[0]) * tangent_x +
        (car_utm[1] - active_avoidance_obstacle_utm_[1]) * tangent_y;
    return passed_distance >= kAvoidancePassDistance;
}

void DynamicObstacleStop::restoreOriginalPath()
{
    if (!is_avoiding_)
    {
        return;
    }

    local_path_.setPosArr(original_pos_arr_);
    local_path_.setYawArr(original_yaw_arr_);
    local_path_.setKArr(original_k_arr_);
    local_path_.updateKDTree();

    is_avoiding_ = false;
    stationary_avoidance_ = false;
    active_avoidance_obstacle_utm_.clear();
    obstacle_lost_pending_ = false;
    avoidance_direction_ = 0;

    std::cout << "[DynamicObstacleStop] Original path restored" << std::endl;
}

void DynamicObstacleStop::initializeFollowingPath()
{
    const Path &global_path = local_.getRefCurGlobalPath();
    const auto &global_pos = global_path.getRefPosArr();
    const auto &global_yaw = global_path.getRefYawArr();
    const auto &global_k = global_path.getRefKArr();

    if (global_pos.empty() ||
        global_yaw.size() != global_pos.size() ||
        global_k.size() != global_pos.size())
    {
        ROS_ERROR_THROTTLE(1.0,
                           "[DynamicObstacleStop] Cannot initialize global following path: invalid path arrays");
        return;
    }

    const double *car_pos = local_.getAddCurCarUTMPos();
    const int closest_index = global_path.getClosestIndex(car_pos);
    const auto &join_point = global_pos[closest_index];
    const double join_distance =
        std::hypot(join_point[0] - car_pos[0], join_point[1] - car_pos[1]);

    if (join_distance <= kDirectFollowDistance)
    {
        local_path_.setPosArr(global_pos);
        local_path_.setYawArr(global_yaw);
        local_path_.setKArr(global_k);
        local_path_.updateKDTree();
        ROS_INFO("[DynamicObstacleStop] Direct global path follow: join distance %.3f m",
                 join_distance);
        return;
    }

    // Spawn 위치와 글로벌 패스가 떨어져 있으면 차량 위치에서 출발하여
    // 글로벌 패스의 최근접점으로 연결되는 직선 진입 경로를 앞에 붙인다.
    const std::size_t approach_point_count = std::max<std::size_t>(
        2, static_cast<std::size_t>(std::ceil(join_distance / kApproachPathSpacing)));
    const double approach_yaw =
        std::atan2(join_point[1] - car_pos[1], join_point[0] - car_pos[0]);

    std::vector<std::vector<double>> following_pos;
    std::vector<double> following_yaw;
    std::vector<double> following_k;
    following_pos.reserve(approach_point_count + global_pos.size() - closest_index);
    following_yaw.reserve(approach_point_count + global_yaw.size() - closest_index);
    following_k.reserve(approach_point_count + global_k.size() - closest_index);

    for (std::size_t index = 0; index < approach_point_count; ++index)
    {
        const double ratio =
            static_cast<double>(index) / static_cast<double>(approach_point_count);
        following_pos.push_back({
            car_pos[0] + (join_point[0] - car_pos[0]) * ratio,
            car_pos[1] + (join_point[1] - car_pos[1]) * ratio});
        following_yaw.push_back(approach_yaw);
        following_k.push_back(0.0);
    }

    following_pos.insert(following_pos.end(),
                         global_pos.begin() + closest_index, global_pos.end());
    following_yaw.insert(following_yaw.end(),
                         global_yaw.begin() + closest_index, global_yaw.end());
    following_k.insert(following_k.end(),
                       global_k.begin() + closest_index, global_k.end());

    local_path_.setPosArr(following_pos);
    local_path_.setYawArr(following_yaw);
    local_path_.setKArr(following_k);
    local_path_.updateKDTree();

    ROS_INFO("[DynamicObstacleStop] Approach path generated: %.3f m, %zu points",
             join_distance, approach_point_count);
}

bool DynamicObstacleStop::checkMissionEnd() const
{
    const auto &path_pos = local_.getRefCurGlobalPath().getRefPosArr();
    if (path_pos.empty())
    {
        return false;
    }

    const auto &goal_pos = path_pos.back();
    const double *car_utm = local_.getAddCurCarUTMPos();
    double goal_distance = std::hypot(car_utm[0] - goal_pos[0], car_utm[1] - goal_pos[1]);

    return goal_distance < kPathChangeDistance;
}

void DynamicObstacleStop::updateTrafficSignalState()
{
    const std::uint64_t generation = vision_.getTrafficLightGeneration();
    if (generation == 0 || generation == last_traffic_light_generation_)
    {
        return;
    }
    last_traffic_light_generation_ = generation;

    const auto &traffic_light = vision_.getRefVLight();
    if (traffic_light.size() < 4)
    {
        return;
    }

    const bool red = traffic_light[0] != 0;
    const bool orange = traffic_light[1] != 0;
    const bool left_only = traffic_light[2] != 0 && traffic_light[3] == 0;
    const bool green = traffic_light[3] != 0;

    if (green)
    {
        if (traffic_stop_latched_)
        {
            ROS_INFO("[DynamicObstacleStop] TRAFFIC GREEN -> release signal stop");
        }
        traffic_stop_latched_ = false;
    }
    else if (red || orange || left_only)
    {
        traffic_stop_latched_ = true;
        ROS_WARN("[DynamicObstacleStop] TRAFFIC STOP latched: red=%d orange=%d left=%d green=%d",
                 traffic_light[0], traffic_light[1],
                 traffic_light[2], traffic_light[3]);
    }
    else
    {
        // 후처리 heartbeat는 인식이 없거나 오래되면 모두 0을 보낸다.
        // 신호를 모르는 상태에서 교차로를 통과하지 않도록 fail-safe 정지한다.
        traffic_stop_latched_ = true;
        ROS_WARN_THROTTLE(
            1.0,
            "[DynamicObstacleStop] UNKNOWN traffic signal -> fail-safe STOP latch");
    }
}

void DynamicObstacleStop::initializeTrafficStopLines()
{
    traffic_stop_lines_.clear();
    traffic_path_s_.clear();

    const Path &global_path = local_.getRefCurGlobalPath();
    const auto &path_pos = global_path.getRefPosArr();
    if (path_pos.empty())
    {
        return;
    }

    traffic_path_s_.assign(path_pos.size(), 0.0);
    for (std::size_t index = 1; index < path_pos.size(); ++index)
    {
        traffic_path_s_[index] = traffic_path_s_[index - 1] +
                                 std::hypot(
                                     path_pos[index][0] - path_pos[index - 1][0],
                                     path_pos[index][1] - path_pos[index - 1][1]);
    }

    for (const auto &raw_stop_line : raw_traffic_stop_lines_)
    {
        if (raw_stop_line.size() < 2 ||
            !std::isfinite(raw_stop_line[0]) ||
            !std::isfinite(raw_stop_line[1]) ||
            (std::abs(raw_stop_line[0]) < 1e-6 &&
             std::abs(raw_stop_line[1]) < 1e-6))
        {
            continue;
        }

        double raw_position[2] = {raw_stop_line[0], raw_stop_line[1]};
        const int path_index = global_path.getClosestIndex(raw_position);
        if (path_index < 0 ||
            static_cast<std::size_t>(path_index) >= path_pos.size())
        {
            continue;
        }

        const double projection_distance = std::hypot(
            raw_stop_line[0] - path_pos[path_index][0],
            raw_stop_line[1] - path_pos[path_index][1]);
        if (projection_distance > traffic_max_projection_distance_)
        {
            continue;
        }

        // 서로 다른 차로의 정지선이 같은 경로 지점으로 투영되면 하나만 유지한다.
        const bool duplicate = std::any_of(
            traffic_stop_lines_.begin(), traffic_stop_lines_.end(),
            [path_index](const TrafficStopLine &line)
            {
                return std::abs(line.path_index - path_index) < 100;
            });
        if (duplicate)
        {
            continue;
        }

        traffic_stop_lines_.push_back(
            {{path_pos[path_index][0], path_pos[path_index][1]},
             path_index, traffic_path_s_[path_index]});
    }

    std::sort(
        traffic_stop_lines_.begin(), traffic_stop_lines_.end(),
        [](const TrafficStopLine &lhs, const TrafficStopLine &rhs)
        {
            return lhs.path_s < rhs.path_s;
        });

    traffic_stop_lines_initialized_ = true;
    ROS_INFO("[DynamicObstacleStop] traffic stop lines on current path: %zu",
             traffic_stop_lines_.size());
    for (const auto &line : traffic_stop_lines_)
    {
        ROS_INFO("[DynamicObstacleStop] traffic stop line: index=%d, UTM=(%.3f, %.3f)",
                 line.path_index, line.point[0], line.point[1]);
    }
}

void DynamicObstacleStop::applyTrafficPriority()
{
    // chooseFunc() 내 장애물 로직이 모두 속도를 설정한 뒤 호출되므로
    // 이 함수의 속도 제약이 항상 최종값이 된다.
    updateTrafficSignalState();
    if (!traffic_stop_lines_initialized_)
    {
        initializeTrafficStopLines();
    }

    // 동적 장애물 상태와 신호 우선 상태를 최종 목표 속도와 함께 출력한다.
    // applyTrafficPriority()가 case 24의 마지막 판단이므로 이 로그의 target이
    // planning_node에서 publish되는 값과 동일하다.
    const auto print_mission_status =
        [this](const std::string &traffic_status,
               int car_index,
               const TrafficStopLine *active_line,
               double signed_distance)
        {
            const char *obstacle_status = "알 수 없음";
            switch (state_)
            {
            case DynamicObstacleState::INIT:
                obstacle_status = "초기화중";
                break;
            case DynamicObstacleState::DRIVE:
                obstacle_status = "글로벌패스 추종중";
                break;
            case DynamicObstacleState::EMERGENCY_STOP:
                obstacle_status = side_collision_stop_
                                      ? "측면 충돌위험 긴급정지"
                                      : "장애물 긴급정지";
                break;
            case DynamicObstacleState::APPROACH_SLOW:
                obstacle_status = "장애물 접근 감속중";
                break;
            case DynamicObstacleState::WAIT_STOPPED_OBSTACLE:
                obstacle_status = "회피경로 생성 대기중";
                break;
            case DynamicObstacleState::AVOID_PATH:
                obstacle_status = "장애물 회피경로 추종중";
                break;
            case DynamicObstacleState::DONE:
                obstacle_status = "동적장애물 미션 종료";
                break;
            }

            const auto &signal = vision_.getRefVLight();
            const double *car_utm = local_.getAddCurCarUTMPos();
            std::ostringstream output;
            output << std::fixed << std::setprecision(2)
                   << "[Mission24Status] obstacle=" << obstacle_status
                   << " | traffic=" << traffic_status
                   << " | signal=[";
            for (std::size_t index = 0; index < 4; ++index)
            {
                if (index != 0)
                {
                    output << ",";
                }
                output << (index < signal.size() ? signal[index] : -1);
            }
            output << "] generation=" << last_traffic_light_generation_
                   << " latch=" << (traffic_stop_latched_ ? "STOP" : "GO")
                   << " | car_utm=(" << car_utm[0] << "," << car_utm[1] << ")"
                   << " stop_lines=" << traffic_stop_lines_.size();

            if (car_index >= 0)
            {
                output << " car_idx=" << car_index;
            }
            else
            {
                output << " car_idx=N/A";
            }

            if (active_line != nullptr)
            {
                output << " stop_idx=" << active_line->path_index
                       << " stop_utm=(" << active_line->point[0]
                       << "," << active_line->point[1] << ")"
                       << " stop_dist=" << signed_distance << "m";
            }
            else
            {
                output << " stop_idx=N/A stop_dist=N/A";
            }

            output << " | target=" << control_.getTargetVelocity() << "m/s";
            std::cout << output.str() << std::endl;
        };

    if (last_traffic_light_generation_ == 0)
    {
        print_mission_status("신호 토픽 미수신", -1, nullptr,
                             std::numeric_limits<double>::infinity());
        return;
    }
    if (traffic_stop_lines_.empty())
    {
        print_mission_status("정지선 미설정/현재 맵과 불일치", -1, nullptr,
                             std::numeric_limits<double>::infinity());
        return;
    }
    if (traffic_path_s_.empty())
    {
        print_mission_status("글로벌패스 정지선 초기화 대기", -1, nullptr,
                             std::numeric_limits<double>::infinity());
        return;
    }

    const Path &global_path = local_.getRefCurGlobalPath();
    const int car_index = global_path.getClosestIndex(local_.getAddCurCarUTMPos());
    if (car_index < 0 ||
        static_cast<std::size_t>(car_index) >= traffic_path_s_.size())
    {
        print_mission_status("차량 경로 인덱스 계산 실패", car_index, nullptr,
                             std::numeric_limits<double>::infinity());
        return;
    }

    const TrafficStopLine *active_line = nullptr;
    double signed_distance = std::numeric_limits<double>::infinity();
    for (const auto &line : traffic_stop_lines_)
    {
        const double candidate_distance =
            line.path_s - traffic_path_s_[car_index];
        if (candidate_distance >= -traffic_done_distance_)
        {
            active_line = &line;
            signed_distance = candidate_distance;
            break;
        }
    }

    if (active_line == nullptr)
    {
        mission_data_.clearStopPoint();
        print_mission_status("남은 정지선 없음", car_index, nullptr,
                             std::numeric_limits<double>::infinity());
        return;
    }

    mission_data_.setStopPoint(active_line->point);
    mission_data_.setStopDistance(traffic_check_distance_);

    if (!traffic_stop_latched_ || signed_distance > traffic_check_distance_)
    {
        const std::string traffic_status =
            traffic_stop_latched_
                ? "정지 신호 감지-정지선 접근 전"
                : "진행 신호/신호 대기 없음";
        ROS_INFO_THROTTLE(
            1.0,
            "[DynamicObstacleStop] traffic monitor: distance=%.2f m, stop=%d",
            signed_distance, traffic_stop_latched_ ? 1 : 0);
        print_mission_status(traffic_status, car_index, active_line,
                             signed_distance);
        return;
    }

    if (signed_distance <= traffic_stop_distance_)
    {
        mission_data_.setStopDistance(traffic_stop_distance_);
        control_.setTargetVelocity(0.0);
        ROS_WARN_THROTTLE(
            0.5,
            "[DynamicObstacleStop] TRAFFIC PRIORITY STOP at line: distance=%.2f m",
            signed_distance);
        print_mission_status("신호 대기중-정지", car_index, active_line,
                             signed_distance);
        return;
    }

    const double traffic_limited_velocity = std::min(
        control_.getTargetVelocity(), traffic_slow_velocity_);
    control_.setTargetVelocity(std::max(0.0, traffic_limited_velocity));
    ROS_WARN_THROTTLE(
        0.5,
        "[DynamicObstacleStop] TRAFFIC PRIORITY SLOW: distance=%.2f m, target=%.2f m/s",
        signed_distance, control_.getTargetVelocity());
    print_mission_status("신호 정지선 접근 감속중", car_index, active_line,
                         signed_distance);
}

void DynamicObstacleStop::doMission(double desired_vel)
{
    double distance = kNoObstacleDistance;

    if (vel_flag_)
    {
        target_vel_ = desired_vel;
        vel_flag_ = false;
    }
    // 카메라 입력 복구 전까지 미션 24의 신호 latch를 갱신하지 않는다.
    // UNKNOWN 신호가 동적 장애물 회피 속도를 0으로 덮어쓰지 않게 한다.
    // updateTrafficSignalState();

    // LiDAR가 발행한 모든 클러스터를 살펴본다. 경로 위 최근 장애물은
    // 종방향 거리/TTC에 사용하고, 전체 관측 객체는 개별 측면 궤적으로 추적한다.
    const auto &obstacle_positions = lidar_.getRefDynamicObstaclePos();
    const std::uint64_t obstacle_generation =
        lidar_.getDynamicObstacleGeneration();
    const bool new_obstacle_frame =
        obstacle_generation != 0 &&
        obstacle_generation != last_dynamic_obstacle_generation_;
    if (new_obstacle_frame)
    {
        last_dynamic_obstacle_generation_ = obstacle_generation;
    }
    std::vector<std::vector<double>> observed_obstacles;
    double direct_obstacle_x = 0.0;
    double direct_obstacle_y = 0.0;
    double direct_lateral_distance = std::numeric_limits<double>::infinity();
    std::size_t on_path_obstacle_count = 0;

    for (std::size_t index = 0;
         index + 1 < obstacle_positions.size(); index += 2)
    {
        const double obstacle_x = obstacle_positions[index];
        const double obstacle_y = obstacle_positions[index + 1];
        if (!isObservedObstacle(obstacle_x, obstacle_y))
        {
            continue;
        }

        observed_obstacles.push_back({obstacle_x, obstacle_y});

        double forward_path_distance = 0.0;
        double lateral_path_distance = std::numeric_limits<double>::infinity();
        if (!getObstaclePathRelation(
                obstacle_x, obstacle_y,
                forward_path_distance, lateral_path_distance) ||
            forward_path_distance < 0.0)
        {
            continue;
        }

        ++on_path_obstacle_count;
        if (forward_path_distance < distance)
        {
            distance = forward_path_distance;
            direct_lateral_distance = lateral_path_distance;
            direct_obstacle_x = obstacle_x;
            direct_obstacle_y = obstacle_y;
        }
    }

    // Planning 주기(100Hz)에서 같은 LiDAR 프레임(약 10Hz)을 여러 번
    // 속도 표본으로 넣지 않는다. 새 프레임이 없을 때는 stale track만 정리한다.
    if (new_obstacle_frame)
    {
        updateSideObstacleTracks(observed_obstacles);
    }
    else
    {
        updateSideObstacleTracks({});
    }

    const bool observed_obstacle_detected = !observed_obstacles.empty();
    const bool direct_obstacle_detected =
        std::isfinite(distance) && distance >= 0.0 &&
        distance < kNoObstacleDistance;

    // 미션 24에 진입했지만 아직 동적 장애물 판단이 성립하지 않은 구간에서는
    // 원본 글로벌 패스를 그대로 추종하고 있음을 명확히 표시한다.
    if (!observed_obstacle_detected &&
        (state_ == DynamicObstacleState::INIT || state_ == DynamicObstacleState::DRIVE))
    {
        ROS_INFO_THROTTLE(1.0,
                          "[DynamicObstacleStop] 글패 추종중 (동적 장애물 미인지, target=%.2f m/s)",
                          desired_vel);
    }

    // 정지 장애물 판정 이력은 경로 위에서 가장 가까운 객체만 유지한다.
    if (direct_obstacle_detected && new_obstacle_frame)
    {
        const std::vector<double> obstacle_utm =
            convertObstacleToUTM(direct_obstacle_x, direct_obstacle_y);
        updateObstaclePositionHistory(
            obstacle_utm, direct_obstacle_x, direct_obstacle_y);
    }
    else if (new_obstacle_frame)
    {
        obstacle_position_history_.clear();
    }

    // 기존 1차원 TTC와 거리 fallback은 정면 폭 내부에만 적용한다.
    if (!direct_obstacle_detected)
    {
        distance = kNoObstacleDistance;
        if (new_obstacle_frame)
        {
            distance_history_.clear();
        }
    }
    else if (new_obstacle_frame)
    {
        distance_history_.push_back({distance, steady_clock::now()});
        if (distance_history_.size() > kMaxHistorySize)
        {
            distance_history_.pop_front();
        }
    }

    double ttc = computeTTC(distance);

    double side_collision_time = std::numeric_limits<double>::infinity();
    double side_closest_distance = std::numeric_limits<double>::infinity();
    double side_relative_velocity_x = 0.0;
    double side_relative_velocity_y = 0.0;
    const bool side_collision_risk = predictSideCollision(
        target_vel_, side_collision_time,
        side_closest_distance,
        side_relative_velocity_x,
        side_relative_velocity_y);

    if (direct_obstacle_detected)
    {
        ROS_INFO_THROTTLE(
            0.5,
            "[DynamicObstacleStop] on-path=%zu, nearest=(%.2f, %.2f), lateral=%.2f m, stationary timer: %.2f / %.2f s, distance: %.2f m",
            on_path_obstacle_count, direct_obstacle_x, direct_obstacle_y,
            direct_lateral_distance,
            std::min(getStationaryObservationDuration(), kStationaryConfirmTime),
            kStationaryConfirmTime, distance);
    }
    else if (observed_obstacle_detected)
    {
        ROS_INFO_THROTTLE(
            0.5,
            "[DynamicObstacleStop] 측면 다중 추적중: visible=%zu, tracks=%zu, rel_v=(%.2f, %.2f) m/s",
            observed_obstacles.size(), side_obstacle_tracks_.size(),
            side_relative_velocity_x, side_relative_velocity_y);
    }

    // 회피 경로가 유효할 때만 AVOID_PATH로 전환한다.
    // 생성에 실패하면 원래 경로로 진입하지 않고 정지 상태에서
    // 1초 간격으로 재시도한다.
    const auto try_start_avoidance =
        [this](bool stationary_triggered)
        {
            if (generateAvoidancePath(stationary_triggered))
            {
                state_ = DynamicObstacleState::AVOID_PATH;
                is_waiting_ = false;
                control_.setTargetVelocity(target_vel_ * kSlowRatio);
                return true;
            }

            state_ = DynamicObstacleState::WAIT_STOPPED_OBSTACLE;
            wait_start_time_ = steady_clock::now();
            is_waiting_ = true;
            control_.setTargetVelocity(0.0);
            return false;
        };

    // 미션 종료 조건
    if (checkMissionEnd())
    {
        state_ = DynamicObstacleState::DONE;
        control_.setTargetVelocity(0.0);
        std::cout << "[DynamicObstacleStop] mission_end" << std::endl;
        return;
    }

    // 횡방향 속도를 포함한 CPA(Closest Point of Approach)가
    // 안전 반경 내부이면 회피 조향보다 우선 정지한다.
    if (side_collision_risk && state_ != DynamicObstacleState::INIT)
    {
        side_collision_stop_ = true;
        last_side_collision_time_ = steady_clock::now();
        state_ = DynamicObstacleState::EMERGENCY_STOP;
        control_.setTargetVelocity(0.0);
        ROS_WARN_THROTTLE(
            0.25,
            "[DynamicObstacleStop] SIDE COLLISION RISK -> STOP: CPA time=%.2f s, distance=%.2f m, rel_v=(%.2f, %.2f) m/s",
            side_collision_time, side_closest_distance,
            side_relative_velocity_x, side_relative_velocity_y);
        return;
    }

    switch (state_)
    {
    case DynamicObstacleState::INIT:
    {
        // 추종이 검증된 글로벌/진입 경로를 먼저 local_path_ 에 설정한다.
        initializeFollowingPath();

        // FrenetFrame 기준 경로 초기화
        std::vector<std::vector<double>> obstacles = getFrontalObstaclesInUTM();
        frenet_frame_.setDefault(local_.getRefCurGlobalPath(), obstacles);

        state_ = DynamicObstacleState::DRIVE;
        std::cout << "[DynamicObstacleStop] INIT -> DRIVE" << std::endl;
        break;
    }
    case DynamicObstacleState::DRIVE:
    {
        // 장애물 미감지
        if (distance < 0.0 || distance >= kNoObstacleDistance)
        {
            control_.setTargetVelocity(target_vel_);
            if (is_avoiding_)
            {
                restoreOriginalPath();
            }
            break;
        }

        // UTM 절대좌표가 3초간 유지된 장애물은 정지 장애물로 확정하고
        // 충돌 예측 경로 범위(10m) 안에서 즉시 회피한다.
        if (isObstacleStopped(distance))
        {
            ROS_WARN("[DynamicObstacleStop] STATIONARY for 3.0 s -> AVOID");
            try_start_avoidance(true);
            break;
        }

        // 1. TTC < 2초: 긴급 정지
        if (ttc < kEmergencyTTC)
        {
            std::cout << "[DynamicObstacleStop] EMERGENCY STOP! TTC = " << ttc << " s" << std::endl;
            state_ = DynamicObstacleState::EMERGENCY_STOP;
            control_.setTargetVelocity(0.0);
            break;
        }

        // 2. 2초 <= TTC <= 5초 + 회피 공간: FrenetFrame 회피 경로 생성
        if (ttc >= kAvoidTTCMin && ttc <= kAvoidTTCMax)
        {
            if (hasAvoidanceSpace())
            {
                std::cout << "[DynamicObstacleStop] AVOID (TTC = " << ttc << " s)" << std::endl;
                try_start_avoidance(false);
            }
            else
            {
                std::cout << "[DynamicObstacleStop] No avoidance space, SLOW (TTC = " << ttc << " s)" << std::endl;
                state_ = DynamicObstacleState::APPROACH_SLOW;
            }
            break;
        }

        // Fallback: 기존 거리 기반 판단
        control_.setTargetVelocity(judgeDynamicObstacleVelocity(distance, target_vel_));
        break;
    }
    case DynamicObstacleState::EMERGENCY_STOP:
    {
        control_.setTargetVelocity(0.0);

        if (side_collision_stop_)
        {
            const double safe_elapsed = duration_cast<duration<double>>(
                                            steady_clock::now() -
                                            last_side_collision_time_)
                                            .count();
            if (safe_elapsed >= kSideClearHoldTime)
            {
                side_collision_stop_ = false;
                state_ = DynamicObstacleState::DRIVE;
                ROS_INFO("[DynamicObstacleStop] SIDE obstacle clear for %.1f s -> DRIVE",
                         kSideClearHoldTime);
            }
            break;
        }

        // 기존에는 긴급정지 상태에서 정지 장애물 판정을 다시 하지 않아
        // 가까운 고정 장애물 앞에서 영구 정지할 수 있었다. 경로 위 동일
        // 장애물이 3초간 유지되면 회피 경로를 생성해 통과한다.
        if (isObstacleStopped(distance))
        {
            ROS_WARN("[DynamicObstacleStop] EMERGENCY STOP held for stationary obstacle -> AVOID");
            try_start_avoidance(true);
            break;
        }

        // 장애물이 사라지거나 TTC가 늘어나면 DRIVE 복귀
        if (distance < 0.0 || distance >= kNoObstacleDistance || ttc > kEmergencyTTC)
        {
            std::cout << "[DynamicObstacleStop] Emergency released" << std::endl;
            state_ = DynamicObstacleState::DRIVE;
        }
        break;
    }
    case DynamicObstacleState::APPROACH_SLOW:
    {
        control_.setTargetVelocity(target_vel_ * kSlowRatio);

        if (isObstacleStopped(distance))
        {
            ROS_WARN("[DynamicObstacleStop] STATIONARY for 3.0 s while slowing -> AVOID");
            try_start_avoidance(true);
        }
        else if (ttc < kEmergencyTTC)
        {
            state_ = DynamicObstacleState::EMERGENCY_STOP;
        }
        else if (ttc > kAvoidTTCMax)
        {
            state_ = DynamicObstacleState::DRIVE;
        }
        else if (ttc >= kAvoidTTCMin && ttc <= kAvoidTTCMax && hasAvoidanceSpace())
        {
            try_start_avoidance(false);
        }
        break;
    }
    case DynamicObstacleState::WAIT_STOPPED_OBSTACLE:
    {
        control_.setTargetVelocity(0.0);

        // 장애물이 사라짐
        if (distance < 0.0 || distance >= kNoObstacleDistance)
        {
            state_ = DynamicObstacleState::DRIVE;
            if (is_avoiding_)
            {
                restoreOriginalPath();
            }
            break;
        }

        const bool stationary_obstacle = isObstacleStopped(distance);

        // 장애물이 이동해 충돌 위험이 사라지면 정상 주행으로 복귀한다.
        if (!stationary_obstacle && ttc > kAvoidTTCMax)
        {
            std::cout << "[DynamicObstacleStop] Obstacle moving/safe again, DRIVE" << std::endl;
            state_ = DynamicObstacleState::DRIVE;
            is_waiting_ = false;
            break;
        }

        // 경로 생성에 실패했다면 정지를 유지하며 1초 간격으로 재시도한다.
        double elapsed = duration_cast<duration<double>>(steady_clock::now() - wait_start_time_).count();
        if (elapsed >= kAvoidanceRetryInterval)
        {
            std::cout << "[DynamicObstacleStop] Retry AVOID path generation" << std::endl;
            wait_start_time_ = steady_clock::now();
            if (generateAvoidancePath(stationary_obstacle))
            {
                state_ = DynamicObstacleState::AVOID_PATH;
                is_waiting_ = false;
                control_.setTargetVelocity(target_vel_ * kSlowRatio);
            }
        }
        break;
    }
    case DynamicObstacleState::AVOID_PATH:
    {
        // 회피 경로 주행 (감속)
        control_.setTargetVelocity(target_vel_ * kSlowRatio);

        // 정지 장애물 회피는 TTC가 크게 나와도 즉시 풀지 않고,
        // 장애물이 전방 ROI에서 사라진 뒤에만 글로벌 경로로 복귀한다.
        const bool obstacle_gone =
            distance < 0.0 || distance >= kNoObstacleDistance;
        bool obstacle_gone_confirmed = false;
        if (obstacle_gone)
        {
            if (!obstacle_lost_pending_)
            {
                obstacle_lost_pending_ = true;
                obstacle_lost_start_time_ = steady_clock::now();
            }
            obstacle_gone_confirmed =
                duration_cast<duration<double>>(
                    steady_clock::now() - obstacle_lost_start_time_).count() >=
                kObstacleClearConfirmTime;
        }
        else
        {
            obstacle_lost_pending_ = false;
        }

        // TTC가 커지거나 LiDAR에서 잠시 사라졌다는 이유로 회피를
        // 중단하지 않고, 저장한 장애물 위치를 2m 통과할 때까지 유지한다.
        const bool obstacle_passed = hasPassedAvoidanceObstacle();
        if (obstacle_passed)
        {
            std::cout << "[DynamicObstacleStop] Avoidance obstacle passed, restore path" << std::endl;
            state_ = DynamicObstacleState::DRIVE;
            restoreOriginalPath();
        }
        else if (obstacle_gone_confirmed)
        {
            ROS_INFO_THROTTLE(
                1.0,
                "[DynamicObstacleStop] obstacle temporarily lost; keep avoidance path until passed");
        }
        break;
    }
    case DynamicObstacleState::DONE:
    {
        std::cout << "[DynamicObstacleStop] DONE" << std::endl;
        control_.setTargetVelocity(0.0);
        break;
    }
    }
}

void DynamicObstacleStop::resetStatus()
{
    state_ = DynamicObstacleState::INIT;
    vel_flag_ = true;
    is_waiting_ = false;
    distance_history_.clear();
    obstacle_position_history_.clear();
    side_obstacle_tracks_.clear();
    next_obstacle_track_id_ = 1;
    last_dynamic_obstacle_generation_ = 0;
    side_collision_stop_ = false;
    last_traffic_light_generation_ = 0;
    traffic_stop_latched_ = false;
    traffic_stop_lines_initialized_ = false;
    traffic_stop_lines_.clear();
    traffic_path_s_.clear();
    mission_data_.clearStopPoint();
    stationary_avoidance_ = false;
    obstacle_lost_pending_ = false;
    if (is_avoiding_)
    {
        restoreOriginalPath();
    }
}
