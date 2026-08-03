#include "gps_shadow_tunnel.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

/**
 * @file gps_shadow_tunnel.cpp
 * @brief GPS 음영 터널의 위치추정, 벽 추종, fallback 상태 전환 구현
 *
 * 전체 설계 원칙
 * ----------------
 * 판단 패키지는 조향각을 직접 만들지 않는다. 이 코드는 LiDAR 벽에서
 * 횡오차와 진행각 오차를 추정한 다음 차량 기준 상대 로컬패스를 생성한다.
 * 실제 조향각은 기존 control 패키지가 이 경로를 추종하며 계산한다.
 *
 * 센서 역할
 * ----------
 * - 양쪽 벽 LiDAR: 횡방향 위치와 진행각의 주 관측값
 * - 한쪽 벽 LiDAR: 반대편 벽이 가려졌을 때의 대체 관측값
 * - IMU yaw rate: 벽이 잠깐 사라진 동안 방향 변화를 예측
 * - 차량속도: 터널 내부 누적 진행거리 계산
 * - GPS: 진입 초기화와 출구 복구 확인
 * - 카메라: LiDAR 횡오차와 일치할 때만 소량 반영하는 교차검증
 *
 * 실패 대응
 * ----------
 * 벽이 사라졌을 때 마지막 조향이나 경로 포인트를 그대로 재사용하지 않는다.
 * 마지막 정상 벽 상태를 IMU로 예측하고 매 주기 새 상대경로를 생성한다.
 * 손실시간이 길어지면 PREDICTED → DEAD_RECKONING → SAFE_STOP 순서로
 * 목표속도를 낮춘다.
 */
namespace mission_2026
{
namespace
{

/**
 * 벽 포인트 중 설정된 전방 x 범위에 있는 유효 포인트만 선택한다.
 *
 * near/far를 따로 적합하면 차량이 벽과 평행한지뿐 아니라 벽 형상이
 * 불규칙하거나 장애물 포인트가 섞였는지 2차로 확인할 수 있다.
 */
std::vector<Point2D> pointsInRange(const std::vector<Point2D> &points,
                                   double minimum_x,
                                   double maximum_x)
{
    std::vector<Point2D> selected;
    selected.reserve(points.size());
    for (const auto &point : points)
    {
        if (point.x >= minimum_x && point.x <= maximum_x &&
            isFinite(point.x) && isFinite(point.y))
        {
            selected.push_back(point);
        }
    }
    return selected;
}

double wallQuality(const LineFitResult &line, const LineFitConfig &config)
{
    /**
     * 벽 품질 점수 구성
     * - residual_score: 직선과 포인트의 잔차가 작은가
     * - span_score: 벽이 x방향으로 충분히 길게 검출됐는가
     * - count_score: 인라이어 포인트가 충분한가
     *
     * 품질은 양쪽 벽 각도를 평균할 때 가중치로 사용한다.
     */
    if (!line.valid)
    {
        return 0.0;
    }

    const double residual_score =
        1.0 - clamp(line.rmse / std::max(config.maximum_rmse, 1e-6), 0.0, 1.0);
    const double span_score =
        clamp(line.x_span / std::max(3.0 * config.minimum_x_span, 1e-6), 0.0, 1.0);
    const double count_score =
        clamp(static_cast<double>(line.inlier_count) /
                  std::max(2.0 * static_cast<double>(config.minimum_points), 1.0),
              0.0,
              1.0);
    return 0.50 * residual_score + 0.30 * span_score + 0.20 * count_score;
}

} // namespace

GpsShadowTunnel::GpsShadowTunnel(TunnelConfig config)
    : config_(std::move(config))
{
    reset();
}

void GpsShadowTunnel::reset()
{
    // 새 미션 진입 또는 미션 비활성화 시 이전 터널의 추정값이 다음
    // 터널에 남지 않도록 모든 누적값과 상태를 초기화한다.
    state_ = TunnelState::kIdle;
    tracking_mode_ = TunnelTrackingMode::kNone;
    mission_start_time_ = 0.0;
    last_update_time_ = 0.0;
    last_valid_wall_time_ = -std::numeric_limits<double>::infinity();
    tunnel_progress_m_ = 0.0;
    estimated_heading_rad_ = 0.0;
    imu_bias_rps_ = 0.0;
    imu_bias_sum_ = 0.0;
    imu_bias_count_ = 0;
    entry_pose_ = {};
    entry_pose_valid_ = false;
    target_left_distance_m_ = config_.default_left_wall_distance_m;
    target_right_distance_m_ = config_.default_right_wall_distance_m;
    left_distance_sum_ = 0.0;
    right_distance_sum_ = 0.0;
    left_distance_count_ = 0;
    right_distance_count_ = 0;
    filtered_lateral_correction_m_ = 0.0;
    filtered_heading_correction_rad_ = 0.0;
    have_filtered_wall_state_ = false;
    gps_recovery_count_ = 0;
}

TunnelState GpsShadowTunnel::state() const
{
    return state_;
}

TunnelTrackingMode GpsShadowTunnel::trackingMode() const
{
    return tracking_mode_;
}

void GpsShadowTunnel::beginMission(const TunnelInput &input)
{
    /**
     * 미션 최초 진입 처리.
     *
     * GPS가 아직 정상인 시점에 진입 위치와 헤딩을 저장해야 출구에서
     * 누적거리 기반 예상 위치와 복구 GPS를 비교할 수 있다.
     */
    reset();
    state_ = TunnelState::kCalibrating;
    mission_start_time_ = input.now;
    last_update_time_ = input.now;
    if (gpsFresh(input))
    {
        entry_pose_ = input.gps.pose;
        entry_pose_valid_ = true;
        estimated_heading_rad_ = input.gps.pose.yaw;
    }
}

bool GpsShadowTunnel::gpsFresh(const TunnelInput &input) const
{
    // NavSatFix status만 보지 않고 마지막 메시지의 age를 검사한다.
    // GPS가 완전히 끊기면 status callback도 오지 않으므로 시간 검사가 필수다.
    return input.gps.valid && isFinite(input.gps.stamp) &&
           input.now >= input.gps.stamp &&
           input.now - input.gps.stamp <= config_.gps_timeout_s;
}

void GpsShadowTunnel::updateDeadReckoning(const TunnelInput &input, double dt)
{
    /**
     * 추측항법에서 유지하는 값은 터널 진행거리와 적분 헤딩이다.
     *
     * progress += speed * dt
     * heading  += (yaw_rate - bias) * dt
     *
     * 콜백 지연이나 일시 정지 후 큰 dt가 한 번에 적분되는 것을 막기 위해
     * 한 주기의 dt를 최대 0.2초로 제한한다.
     */
    const double safe_dt = clamp(dt, 0.0, 0.20);
    const double speed = std::max(0.0, input.vehicle_speed_mps);
    const double corrected_yaw_rate = input.imu_yaw_rate_rps - imu_bias_rps_;
    tunnel_progress_m_ += speed * safe_dt;
    estimated_heading_rad_ =
        normalizeAngle(estimated_heading_rad_ + corrected_yaw_rate * safe_dt);
}

WallEstimate GpsShadowTunnel::estimateWall(
    const std::vector<Point2D> &points,
    bool expect_left_side) const
{
    /**
     * 하나의 벽을 full/near/far 세 구간으로 적합한다.
     *
     * full: 최종 거리와 각도 계산
     * near/far: 두 구간의 각도 차이로 벽 형상과 검출 안정성 확인
     *
     * 왼쪽 벽은 차량 좌표계에서 절편이 양수, 오른쪽 벽은 음수여야 한다.
     * 반대 부호면 좌우 분류가 잘못됐거나 차량을 가로지르는 물체이므로 폐기한다.
     */
    WallEstimate estimate;
    auto all = pointsInRange(points, config_.near_start_m, config_.far_end_m);
    auto near = pointsInRange(points, config_.near_start_m, config_.near_end_m);
    auto far = pointsInRange(points, config_.far_start_m, config_.far_end_m);

    estimate.full = robustLineFit(all, config_.wall_fit);
    estimate.near = robustLineFit(near, config_.wall_fit);
    estimate.far = robustLineFit(far, config_.wall_fit);

    if (!estimate.full.valid)
    {
        return estimate;
    }

    const bool side_is_correct =
        expect_left_side ? estimate.full.intercept > 0.0
                         : estimate.full.intercept < 0.0;
    if (!side_is_correct)
    {
        return estimate;
    }

    double consistency_score = 1.0;
    if (estimate.near.valid && estimate.far.valid)
    {
        // 근거리와 원거리 벽 방향이 크게 다르면 꺾인 구조물, 장애물 혼입,
        // 또는 클러스터 결합 오류로 판단한다.
        const double angle_difference =
            std::abs(normalizeAngle(estimate.near.angle - estimate.far.angle));
        if (angle_difference > config_.maximum_near_far_angle_rad)
        {
            return estimate;
        }
        consistency_score =
            1.0 - clamp(angle_difference /
                            std::max(config_.maximum_near_far_angle_rad, 1e-6),
                        0.0,
                        1.0);
    }

    estimate.quality =
        0.75 * wallQuality(estimate.full, config_.wall_fit) +
        0.25 * consistency_score;
    estimate.valid = estimate.quality > 0.20;
    return estimate;
}

void GpsShadowTunnel::collectCalibration(const TunnelInput &input,
                                         const WallEstimate &left,
                                         const WallEstimate &right)
{
    /**
     * 진입 캘리브레이션에서 다음 기준값을 모은다.
     *
     * - 직진에 가까울 때의 yaw rate 평균 → IMU 바이어스
     * - 좌우 벽 평균거리 → 터널에서 유지할 목표 벽 거리
     * - 마지막 정상 GPS pose → 출구 GPS innovation 기준
     *
     * 목표를 무조건 터널 정중앙으로 두지 않고 진입 시 정상 경로의
     * 좌우 거리 관계를 유지하므로, 지도 경로와 터널 중심이 다른 상황에 대응한다.
     */
    if (std::abs(input.imu_yaw_rate_rps) <=
        config_.maximum_bias_sample_yaw_rate_rps)
    {
        imu_bias_sum_ += input.imu_yaw_rate_rps;
        ++imu_bias_count_;
    }

    if (left.valid)
    {
        left_distance_sum_ += left.full.perpendicular_distance;
        ++left_distance_count_;
    }
    if (right.valid)
    {
        right_distance_sum_ += right.full.perpendicular_distance;
        ++right_distance_count_;
    }

    if (!entry_pose_valid_ && gpsFresh(input))
    {
        entry_pose_ = input.gps.pose;
        entry_pose_valid_ = true;
        estimated_heading_rad_ = input.gps.pose.yaw;
    }
}

void GpsShadowTunnel::finishCalibration()
{
    // 캘리브레이션 표본이 없으면 Config의 기본값을 그대로 사용한다.
    // 덕분에 한쪽 벽만 보이는 상태로 진입해도 모듈이 동작할 수 있다.
    if (imu_bias_count_ > 0)
    {
        imu_bias_rps_ = imu_bias_sum_ / static_cast<double>(imu_bias_count_);
    }
    if (left_distance_count_ > 0)
    {
        target_left_distance_m_ =
            left_distance_sum_ / static_cast<double>(left_distance_count_);
    }
    if (right_distance_count_ > 0)
    {
        target_right_distance_m_ =
            right_distance_sum_ / static_cast<double>(right_distance_count_);
    }
    state_ = TunnelState::kWallTracking;
}

bool GpsShadowTunnel::exitRegionReached(const TunnelInput &input) const
{
    // 알려진 터널 길이는 출구 "확정"이 아니라 GPS 복구 검사를 시작하는
    // 구간을 결정하는 데만 사용한다. 독립적인 exit_hint도 함께 허용한다.
    const double search_start =
        std::max(0.0,
                 config_.known_tunnel_length_m - config_.exit_search_margin_m);
    return input.exit_hint || tunnel_progress_m_ >= search_start;
}

double GpsShadowTunnel::gpsInnovation(const TunnelInput &input) const
{
    /**
     * 직선 터널이라는 대회 조건을 이용한 단순 예상 출구 위치:
     *
     * expected = entry_position
     *          + progress * [cos(entry_yaw), sin(entry_yaw)]
     *
     * 복구 GPS가 이 예상 위치에서 지나치게 멀면 순간 튐으로 보고 거부한다.
     * 향후 곡선 터널을 지원할 경우 터널 기준경로의 s 좌표로 대체해야 한다.
     */
    if (!entry_pose_valid_ || !gpsFresh(input))
    {
        return std::numeric_limits<double>::infinity();
    }

    const double expected_x =
        entry_pose_.x + tunnel_progress_m_ * std::cos(entry_pose_.yaw);
    const double expected_y =
        entry_pose_.y + tunnel_progress_m_ * std::sin(entry_pose_.yaw);
    return std::hypot(input.gps.pose.x - expected_x,
                      input.gps.pose.y - expected_y);
}

TunnelOutput GpsShadowTunnel::buildTrackingOutput(
    const TunnelInput &input,
    const WallEstimate &left,
    const WallEstimate &right,
    double dt)
{
    /**
     * 한 주기의 핵심 판단 순서
     *
     * 1. 양쪽 벽의 유효성과 평행도 검사
     * 2. DUAL/LEFT/RIGHT 중 관측 모드 선택
     * 3. 횡오차와 진행각 오차 계산
     * 4. 카메라와 일치할 때만 작은 가중치로 보정
     * 5. 저역통과 필터 적용
     * 6. 벽 손실 시 시간에 따라 fallback 모드 선택
     * 7. 필터링된 상태로 상대 로컬패스와 목표속도 생성
     */
    TunnelOutput output;
    output.override_normal_planning = true;
    output.state = state_;
    output.left_wall = left;
    output.right_wall = right;
    output.tunnel_progress_m = tunnel_progress_m_;

    const bool dual_walls =
        left.valid && right.valid &&
        std::abs(normalizeAngle(left.full.angle - right.full.angle)) <=
            config_.maximum_dual_wall_angle_difference_rad;

    bool current_wall_valid = false;
    double measured_lateral = filtered_lateral_correction_m_;
    double measured_heading = filtered_heading_correction_rad_;

    if (dual_walls)
    {
        /**
         * 횡보정 부호 정의
         * - 차량이 오른쪽으로 치우치면 왼쪽 벽 거리는 증가하고
         *   오른쪽 벽 거리는 감소한다.
         * - 이때 lateral_correction은 양수(왼쪽 경로 보정)가 되어야 한다.
         *
         * lateral = 0.5 * (left_error - right_error)
         */
        const double left_error =
            left.full.perpendicular_distance - target_left_distance_m_;
        const double right_error =
            right.full.perpendicular_distance - target_right_distance_m_;
        measured_lateral = 0.5 * (left_error - right_error);
        measured_heading =
            (left.quality * left.full.angle + right.quality * right.full.angle) /
            std::max(left.quality + right.quality, 1e-6);
        tracking_mode_ = TunnelTrackingMode::kDualWall;
        current_wall_valid = true;
    }
    else if (left.valid &&
             (!right.valid || left.quality >= right.quality))
    {
        // 왼쪽 벽만 있을 때 현재 왼쪽 거리가 목표보다 크면 차량이 오른쪽에
        // 있으므로 양수(왼쪽) 보정을 생성한다.
        measured_lateral =
            left.full.perpendicular_distance - target_left_distance_m_;
        measured_heading = left.full.angle;
        tracking_mode_ = TunnelTrackingMode::kLeftWall;
        current_wall_valid = true;
    }
    else if (right.valid)
    {
        // 오른쪽 벽만 있을 때 현재 거리가 목표보다 작으면 차량이 오른쪽에
        // 가까우므로 양수(왼쪽) 보정을 생성한다.
        measured_lateral =
            target_right_distance_m_ - right.full.perpendicular_distance;
        measured_heading = right.full.angle;
        tracking_mode_ = TunnelTrackingMode::kRightWall;
        current_wall_valid = true;
    }

    if (current_wall_valid)
    {
        // 카메라는 LiDAR와 일정 범위 안에서 일치할 때만 반영한다.
        // 큰 불일치는 어느 센서가 틀렸는지 알 수 없으므로 융합하지 않는다.
        if (input.camera_lateral_valid &&
            std::abs(input.camera_lateral_error - measured_lateral) <=
                config_.maximum_camera_wall_disagreement_m)
        {
            measured_lateral =
                (1.0 - config_.camera_correction_weight) * measured_lateral +
                config_.camera_correction_weight * input.camera_lateral_error;
        }

        if (!have_filtered_wall_state_)
        {
            filtered_lateral_correction_m_ = measured_lateral;
            filtered_heading_correction_rad_ = measured_heading;
            have_filtered_wall_state_ = true;
        }
        else
        {
            filtered_lateral_correction_m_ =
                lowPass(filtered_lateral_correction_m_,
                        measured_lateral,
                        config_.wall_filter_alpha);
            filtered_heading_correction_rad_ =
                lowPassAngle(filtered_heading_correction_rad_,
                             measured_heading,
                             config_.wall_filter_alpha);
        }
        last_valid_wall_time_ = input.now;
        output.target_speed_mps = config_.normal_speed_mps;
    }
    else
    {
        /**
         * 벽 미검출 fallback
         *
         * PREDICTED:
         *   매우 짧은 손실. 마지막 벽 상태를 IMU 회전량으로 보정.
         *
         * DEAD_RECKONING:
         *   손실이 조금 더 길어짐. 같은 예측을 사용하되 속도를 추가 감소.
         *
         * SAFE_STOP:
         *   벽을 신뢰할 수 없는 시간이 한계를 넘음. 보정량을 서서히
         *   중립으로 보내면서 정지를 요청.
         */
        const double wall_age = input.now - last_valid_wall_time_;
        const double corrected_yaw_rate = input.imu_yaw_rate_rps - imu_bias_rps_;

        // 오래된 조향 명령이나 상대경로 점을 그대로 유지하지 않는다.
        // 차량이 회전한 양만큼 벽의 상대 방향은 반대로 변하므로 빼준다.
        filtered_heading_correction_rad_ =
            normalizeAngle(filtered_heading_correction_rad_ -
                           corrected_yaw_rate * clamp(dt, 0.0, 0.20));

        if (have_filtered_wall_state_ &&
            wall_age <= config_.wall_hold_duration_s)
        {
            tracking_mode_ = TunnelTrackingMode::kPredicted;
            output.target_speed_mps = config_.predicted_speed_mps;
        }
        else if (have_filtered_wall_state_ &&
                 wall_age <= config_.dead_reckoning_limit_s)
        {
            tracking_mode_ = TunnelTrackingMode::kDeadReckoning;
            output.target_speed_mps = config_.dead_reckoning_speed_mps;
        }
        else
        {
            tracking_mode_ = TunnelTrackingMode::kSafeStop;
            output.target_speed_mps = config_.safe_stop_speed_mps;
            filtered_lateral_correction_m_ =
                lowPass(filtered_lateral_correction_m_, 0.0, 0.10);
            filtered_heading_correction_rad_ =
                lowPassAngle(filtered_heading_correction_rad_, 0.0, 0.10);
        }
    }

    output.tracking_mode = tracking_mode_;
    output.lateral_correction_m = filtered_lateral_correction_m_;
    output.heading_correction_rad = filtered_heading_correction_rad_;
    output.relative_path =
        buildRelativeMergePath(filtered_lateral_correction_m_,
                               filtered_heading_correction_rad_,
                               config_.path_lookahead_m,
                               config_.path_step_m,
                               config_.maximum_lateral_correction_m);

    std::ostringstream diagnostic;
    diagnostic << toString(tracking_mode_)
               << ", progress=" << tunnel_progress_m_
               << "m, lateral=" << filtered_lateral_correction_m_
               << "m, heading=" << filtered_heading_correction_rad_ << "rad";
    output.diagnostic = diagnostic.str();
    return output;
}

TunnelOutput GpsShadowTunnel::buildRecoveryOutput(
    const TunnelInput &input,
    const WallEstimate &left,
    const WallEstimate &right,
    double dt)
{
    /**
     * GPS 복구는 한 프레임으로 확정하지 않는다.
     *
     * 조건을 만족하는 GPS가 연속 N회 들어왔을 때만 DONE으로 전환한다.
     * 중간에 오래되거나 예상 위치에서 벗어난 GPS가 한 번이라도 들어오면
     * 카운트를 0으로 되돌린다.
     */
    auto output = buildTrackingOutput(input, left, right, dt);
    output.state = TunnelState::kGpsRecovery;

    const bool plausible_fix =
        gpsFresh(input) &&
        gpsInnovation(input) <= config_.gps_recovery_maximum_innovation_m;
    gps_recovery_count_ = plausible_fix ? gps_recovery_count_ + 1 : 0;

    if (gps_recovery_count_ >= config_.gps_recovery_required_samples)
    {
        state_ = TunnelState::kDone;
        tracking_mode_ = TunnelTrackingMode::kNone;
        output.state = state_;
        output.tracking_mode = tracking_mode_;
        output.override_normal_planning = false;
        output.request_absolute_path = true;
        output.relative_path = {};
        output.diagnostic = "GPS recovery confirmed; switch to absolute path";
    }
    return output;
}

TunnelOutput GpsShadowTunnel::update(const TunnelInput &input)
{
    /**
     * 외부에서 매 주기 호출하는 유일한 공개 실행 함수.
     *
     * 입력 mission_active가 false이면 출력 override도 false이므로 기존
     * 일반 경로계획이 그대로 동작할 수 있다. 미션이 활성화된 동안만
     * 이 모듈이 상대경로와 목표속도를 우선 출력한다.
     */
    if (!input.mission_active)
    {
        if (state_ != TunnelState::kIdle)
        {
            reset();
        }
        return {};
    }

    if (state_ == TunnelState::kIdle)
    {
        beginMission(input);
    }

    const double dt = clamp(input.now - last_update_time_, 0.0, 0.20);
    last_update_time_ = input.now;
    updateDeadReckoning(input, dt);

    const auto left = estimateWall(input.left_wall_points, true);
    const auto right = estimateWall(input.right_wall_points, false);

    if (state_ == TunnelState::kCalibrating)
    {
        collectCalibration(input, left, right);
        // GPS가 예상보다 빨리 끊기면 캘리브레이션 시간을 기다리지 않고
        // 지금까지 수집된 표본과 기본 설정값으로 즉시 벽 추종을 시작한다.
        if (input.now - mission_start_time_ >= config_.calibration_duration_s ||
            !gpsFresh(input))
        {
            finishCalibration();
        }
    }

    if (state_ == TunnelState::kDone)
    {
        TunnelOutput output;
        output.request_absolute_path = true;
        output.state = state_;
        output.diagnostic = "mission done";
        return output;
    }

    if (state_ == TunnelState::kWallTracking &&
        exitRegionReached(input) && gpsFresh(input))
    {
        state_ = TunnelState::kGpsRecovery;
        gps_recovery_count_ = 0;
    }

    if (state_ == TunnelState::kGpsRecovery)
    {
        return buildRecoveryOutput(input, left, right, dt);
    }

    auto output = buildTrackingOutput(input, left, right, dt);
    output.state = state_;
    if (state_ == TunnelState::kCalibrating)
    {
        output.diagnostic = "calibrating: " + output.diagnostic;
    }
    return output;
}

const char *toString(TunnelState state)
{
    switch (state)
    {
    case TunnelState::kIdle:
        return "IDLE";
    case TunnelState::kCalibrating:
        return "CALIBRATING";
    case TunnelState::kWallTracking:
        return "WALL_TRACKING";
    case TunnelState::kGpsRecovery:
        return "GPS_RECOVERY";
    case TunnelState::kDone:
        return "DONE";
    }
    return "UNKNOWN";
}

const char *toString(TunnelTrackingMode mode)
{
    switch (mode)
    {
    case TunnelTrackingMode::kNone:
        return "NONE";
    case TunnelTrackingMode::kDualWall:
        return "DUAL_WALL";
    case TunnelTrackingMode::kLeftWall:
        return "LEFT_WALL";
    case TunnelTrackingMode::kRightWall:
        return "RIGHT_WALL";
    case TunnelTrackingMode::kPredicted:
        return "PREDICTED";
    case TunnelTrackingMode::kDeadReckoning:
        return "DEAD_RECKONING";
    case TunnelTrackingMode::kSafeStop:
        return "SAFE_STOP";
    }
    return "UNKNOWN";
}

} // namespace mission_2026
