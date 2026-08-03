#ifndef HIGHWAY_LANE_CHANGE_H_
#define HIGHWAY_LANE_CHANGE_H_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iostream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "control.hpp"
#include "lidar.hpp"
#include "local.hpp"
#include "nlohmann/json.hpp"
#include "path.hpp"
#include "utility_function.hpp"

/**
 * @brief 미션 100 고속도로 좌측 연속 차선변경을 담당한다.
 *
 * 맵 파일 인덱스 n-0, n-1, n-2, n-3은 각각 4, 3, 2, 1차로에 대응한다.
 * 한 번의 차선변경이 실제로 끝난 뒤에만 다음 맵으로 전환하고 현재 차로를 갱신한다.
 */
class HighwayLaneChange
{
private:
    enum class State
    {
        INIT,           // 미션 진입 직후 파라미터와 차선 맵을 초기화하는 상태
        WAIT_FOR_START, // 지정 좌표를 현재 경로에 투영한 차선변경 시작점까지 주행
        WAIT_FOR_CLEAR, // 좌측 차량과 종방향 슬롯을 만든 뒤 예측 간격·TTC 확인
        LANE_CHANGE,    // 생성한 5차 다항식 로컬패스를 따라 왼쪽 차로로 이동
        CRUISE,         // 목표 차로 도착 후 현재 글로벌패스를 따라 주행
        SAFE_STOP,      // 맵 끝 안전정지 또는 경로 생성 실패로 정지
        DONE            // 미션 경로 주행 완료
    };

    enum class LaneChangeProfile
    {
        REGULAR, // 주행 차로의 일반 목표속도로 계산하는 차선변경
        COMPACT  // 50m 안전정지 후 전용속도로 계산하는 차선변경
    };

    enum class GapMode
    {
        NONE,          // 나란한 좌측 차량이 없어 기존 안전판정만 수행
        ASSESS,        // 동일 차량과 선택 방향이 연속 확인되는지 검사
        PASS_AHEAD,    // 현재 차로에서 가속해 좌측 차량 앞 슬롯을 생성
        YIELD_BEHIND,  // 완만하게 감속해 좌측 차량 뒤 슬롯을 생성
        READY,         // 종방향 슬롯이 확보되어 기존 전체 안전판정을 수행
        HOLD           // 트랙 무효·복수 후보 등으로 슬롯 결정을 보류
    };

    enum class TtcBenefitReason
    {
        MANDATORY_MERGE,          // 4→3 합류는 TTC 이득 판단을 적용하지 않음
        CURRENT_STREAM_UNKNOWN,   // 현재 차로 전방 트랙이 무효/시간초과
        CURRENT_TRACK_UNKNOWN,    // 현재 차로 전방 트랙 속도가 아직 미확정
        NO_CURRENT_FRONT,         // 비교 범위 안에 현재 차로 선행차가 없음
        CURRENT_NOT_CLOSING,      // 선행차와 거리가 줄어들지 않음
        CURRENT_TTC_COMFORTABLE,  // 현재 차로 TTC가 설정 문턱보다 큼
        LEFT_STREAM_UNKNOWN,      // 왼쪽 차로 트랙이 무효/시간초과
        LEFT_TRACK_UNKNOWN,       // 왼쪽 전방 트랙 속도가 아직 미확정
        INSUFFICIENT_ADVANTAGE,   // 왼쪽 TTC 우위가 설정값보다 작음
        BENEFIT_HOLDING,          // TTC 우위 연속 유지시간 확인 중
        BENEFICIAL                // 왼쪽 TTC 우위가 연속 확인됨
    };

    struct FrontTtcEstimate
    {
        bool stream_valid = false;       // 트랙 메시지가 최신이며 유효한지
        bool data_available = false;     // 범위 내 모든 전방 트랙의 속도가 확정됐는지
        bool front_present = false;      // 비교 범위 안에 전방 트랙이 있는지
        std::size_t track_count = 0;     // 해당 토픽에 포함된 전체 트랙 수
        double gap_m = std::numeric_limits<double>::infinity();
        double relative_velocity_mps = 0.0;
        double ttc_sec = std::numeric_limits<double>::infinity();
    };

    struct TtcBenefitAssessment
    {
        FrontTtcEstimate current_front;  // 현재 주행차로 전방
        FrontTtcEstimate left_front;     // 진입하려는 왼쪽 차로 전방
        bool benefit_required = false;   // 느린 선행차 때문에 TTC 비교가 활성화됐는지
        bool raw_benefit = false;        // 유지시간 적용 전 순간 TTC 우위
        double advantage_sec = 0.0;      // 왼쪽 TTC - 현재 TTC
        TtcBenefitReason reason =
            TtcBenefitReason::CURRENT_STREAM_UNKNOWN;
    };

    static constexpr int kLeftmostLaneIndex = 1;  // 가장 왼쪽 차로 번호
    static constexpr int kRightmostLaneIndex = 4; // 합류 차로를 포함한 시작 차로 번호

    Local &local_;      // 차량 상태, 현재 맵, 인접 맵을 제공하는 공용 위치 객체
    Lidar &lidar_;      // 좌측 차로 점유 여부와 전방 차량 거리를 제공하는 공용 객체
    Control &control_;  // 최종 목표속도와 전방 거리 제어를 적용하는 공용 객체
    Path &local_path_;  // 실제 추종할 글로벌/차선변경 결합 로컬패스

    State current_state_ = State::INIT;                  // 현재 미션 상태
    int current_lane_idx_ = kRightmostLaneIndex;         // 완료가 확인된 현재 차로
    int pending_lane_step_ = 0;                          // 완료 시 반영할 차로 변화량(-1은 왼쪽)
    bool mission_complete_ = false;                      // 전체 미션 완료 여부
    bool lane_maps_valid_ = false;                       // 필요한 n-0~n-3 맵 검증 결과
    bool high_pass_slow_reached_ = false;                // 하이패스 감속 시작점 통과 여부
    bool merge_stop_required_ = false;                   // 4차로 맵 끝 50m 안전정지 처리 중인지 표시
    bool merge_stop_satisfied_ = false;                  // 실제 속도가 정지 기준 이하가 됐는지 표시
    bool lane_map_transition_failed_ = false;            // 횡이동 완료 후 맵 전환 실패로 복구 금지 여부
    bool left_safety_candidate_active_ = false;          // 연속 안전시간 측정 중인지 표시
    std::chrono::steady_clock::time_point
        left_safety_candidate_start_time_;               // 현재 안전 후보가 시작된 시각
    bool ttc_benefit_candidate_active_ = false;           // TTC 우위 연속 확인 중인지
    std::chrono::steady_clock::time_point
        ttc_benefit_candidate_start_time_;                // TTC 우위 후보 시작 시각

    GapMode gap_mode_ = GapMode::NONE;                    // WAIT_FOR_CLEAR의 종방향 슬롯 보조상태
    GapMode gap_candidate_mode_ = GapMode::NONE;          // 연속 확인 중인 앞/뒤 슬롯 후보
    GapMode gap_intent_mode_ = GapMode::NONE;             // READY가 되기 전 고정한 실제 슬롯 방향
    int gap_anchor_track_id_ = 0;                         // 나란한 좌측 차량의 추적 ID
    bool gap_candidate_active_ = false;                   // 같은 방향 후보의 확인시간 측정 여부
    std::chrono::steady_clock::time_point
        gap_candidate_start_time_;                        // 슬롯 방향 후보가 시작된 시각
    bool gap_hold_release_candidate_active_ = false;      // 센서 HOLD 해제 전 유효상태 연속 확인 여부
    std::chrono::steady_clock::time_point
        gap_hold_release_candidate_start_time_;           // HOLD 해제 후보가 시작된 시각
    std::chrono::steady_clock::time_point
        gap_mode_start_time_;                             // 현재 슬롯 모드가 시작된 시각
    bool gap_velocity_override_active_ = false;           // WAIT_FOR_CLEAR 슬롯 속도 적용 여부
    double gap_command_velocity_mps_ = 0.0;               // 가감속 제한을 적용한 슬롯 목표속도
    double gap_desired_velocity_mps_ = 0.0;               // 현재 모드가 요구하는 원래 목표속도
    bool gap_velocity_update_started_ = false;            // 슬롯 속도 갱신시간 초기화 여부
    std::chrono::steady_clock::time_point
        last_gap_velocity_update_time_;                   // 마지막 슬롯 속도 갱신 시각
    bool gap_pass_abandoned_ = false;                     // 같은 차량에서 앞 슬롯 재선택을 막는 latch
    double last_gap_anchor_min_x_m_ = 0.0;                // 로그용 선택 차량 후면 X
    double last_gap_anchor_max_x_m_ = 0.0;                // 로그용 선택 차량 전면 X
    double last_gap_anchor_relative_velocity_mps_ = 0.0;  // 로그용 선택 차량 상대속도
    double last_gap_anchor_speed_mps_ = 0.0;               // 로그용 선택 차량 추정 절대속도
    double last_gap_actual_m_ = 0.0;                       // 로그용 실제 앞/뒤 간격
    double last_gap_required_m_ = 0.0;                     // 로그용 요구 앞/뒤 간격
    double gap_progress_reference_x_m_ = 0.0;              // 앞/뒤 슬롯 timeout 구간의 진행 비교 X
    bool last_gap_pass_feasible_ = false;                  // 로그용 앞 슬롯 가능 여부
    std::string gap_reason_ = "슬롯 미사용";               // 마지막 슬롯 판단 이유

    bool status_log_started_ = false;                     // 주기 상태 로그가 시작됐는지 표시
    std::chrono::steady_clock::time_point
        last_status_log_time_;                            // 마지막 주기 상태 로그 출력 시각
    double last_front_distance_m_ = -1.0;                 // 마지막 전방 차량 거리(m)
    double last_mission_target_velocity_mps_ = 0.0;       // 거리 제어 전 미션 목표속도(m/s)
    double active_lane_change_velocity_mps_ = 0.0;        // 현재 5차 경로와 안전예측에 공통 사용한 속도
    double active_lane_change_profile_velocity_mps_ = 0.0; // 거리제어 전 차선변경 속도 상한
    std::vector<double> lane_change_start_path_pos_; // 시작 UTM을 현재 경로에 투영한 좌표
    std::vector<double> high_pass_slow_path_pos_;    // 감속 UTM을 현재 경로에 투영한 좌표
    std::vector<double> lane_change_end_path_pos_;   // 5차 차선변경 경로의 종료 좌표
    std::size_t lane_change_boundary_local_idx_ = 0; // 로컬패스에서 5차 경로가 끝나는 인덱스

    const int kTargetLaneIdx_;                 // 최종 목표 차로 번호(기본 1차로)
    const double kMergeVelocityMps_;           // 4차로 목표속도(m/s)
    const double kMainVelocityMps_;            // 3~1차로 목표속도(m/s)
    const double kHighPassVelocityMps_;        // 하이패스 진입 전 감속 목표속도(m/s)
    const double kMergeStopLaneChangeVelocityMps_; // 안전정지 후 축소 합류경로 전용속도(m/s)
    const std::vector<double> kLaneChangeStartPoint_; // 차선변경 허용 시작 UTM 원본
    const std::vector<double> kHighPassSlowPoint_;    // 하이패스 감속 시작 UTM 원본
    const double kMergeLaneStopDistance_;      // 4차로 유효 맵 끝 전 안전정지 거리(m)
    const double kLaneChangeDurationSec_;      // 일반 차선변경 종방향 계산 시간(s)
    const double kQuinticInterval_;            // 5차 다항식 경로의 출력 점 간격(m)
    const double kLaneChangeCompleteDistance_; // 목표 차로 도착 허용 오차(m)
    const double kTriggerReachedDistance_;     // 투영 좌표 도달 판정 오차(m)
    const double kStopVelocityThresholdMps_;   // 완전 정지로 인정하는 속도 상한(m/s)
    const std::size_t kPathTailPointCount_;    // 맵 뒤쪽에서 주행에 쓰지 않는 꼬리 점 개수
    const double kLeftPredictionHorizonSec_;        // 일반 차선변경 예측 시간(s)
    const double kCompactLeftPredictionHorizonSec_; // 축소 합류경로 예측 시간(s)
    const int kLeftMinConfirmedHits_;               // 신뢰 트랙 최소 연속 검출 수
    const int kLeftMaxMissedScans_;                 // 안전판정에서 허용할 누락 수
    const double kLeftHardFrontGapM_;               // 전방 절대 최소 간격(m)
    const double kLeftHardRearGapM_;                // 후방 절대 최소 간격(m)
    const double kLeftFrontTimeHeadwaySec_;         // 전방 속도 연동 여유시간(s)
    const double kLeftRearTimeHeadwaySec_;          // 후방 속도 연동 여유시간(s)
    const double kLeftMinFrontTtcSec_;              // 전방 최소 TTC(s)
    const double kLeftMinRearTtcSec_;               // 후방 최소 TTC(s)
    const double kLeftClosingSpeedEpsilonMps_;      // 접근 여부 판정 속도 문턱(m/s)
    const double kLeftSafeHoldSec_;                 // 안전 조건 연속 유지시간(s)
    const double kCurrentFrontTtcTriggerSec_;       // TTC 우위 비교를 시작할 현재 차로 TTC(s)
    const double kLeftFrontMinTtcAdvantageSec_;     // 왼쪽 차로가 더 가져야 할 최소 TTC(s)
    const double kFrontTtcCompareRangeM_;           // 두 차로에 동일 적용할 비교 거리(m)
    const double kFrontTtcDisplayCapSec_;           // 무한 TTC를 로그에 표시할 기준값(s)
    const double kTtcBenefitHoldSec_;               // TTC 우위 연속 유지시간(s)
    const double kSideBySideFrontExtentM_;          // LiDAR 원점 기준 자차 전방 나란함 범위(m)
    const double kSideBySideRearExtentM_;           // LiDAR 원점 기준 자차 후방 나란함 범위(m)
    const double kParallelRelativeSpeedMaxMps_;     // 거의 평행 주행으로 보는 상대속도 범위(m/s)
    const double kSlotSpeedBiasMps_;                 // 옆차보다 빠르거나 느리게 만들 속도차(m/s)
    const double kSlotReadyMarginM_;                 // 슬롯 완료 간격에 더하는 히스테리시스(m)
    const double kSlotIntentHoldSec_;                // 같은 슬롯 방향 연속 확인시간(s)
    const double kSlotModeMinHoldSec_;               // 선택 모드를 유지하는 최소시간(s)
    const double kSlotTimeoutSec_;                   // 앞/뒤 슬롯의 진전 여부를 다시 확인할 주기(s)
    const double kSlotAccelLimitMps2_;               // 슬롯 목표속도 상승률 제한(m/s^2)
    const double kSlotDecelLimitMps2_;               // 슬롯 목표속도 하강률 제한(m/s^2)

public:
    HighwayLaneChange(Local &local, Lidar &lidar, Control &control, Path &local_path,
                      nlohmann::json &param)
        : local_(local),
          lidar_(lidar),
          control_(control),
          local_path_(local_path),
          kTargetLaneIdx_(param.at("target_lane_idx")),
          kMergeVelocityMps_(param.at("merge_velocity_mps")),
          kMainVelocityMps_(param.at("main_velocity_mps")),
          kHighPassVelocityMps_(param.at("high_pass_velocity_mps")),
          kMergeStopLaneChangeVelocityMps_(
              param.at("merge_stop_lane_change_velocity_mps")),
          kLaneChangeStartPoint_(param.at("lane_change_start_point").get<std::vector<double>>()),
          kHighPassSlowPoint_(param.at("high_pass_slow_point").get<std::vector<double>>()),
          kMergeLaneStopDistance_(param.at("merge_lane_stop_distance")),
          kLaneChangeDurationSec_(param.at("lane_change_duration_sec")),
          kQuinticInterval_(param.at("quintic_interval")),
          kLaneChangeCompleteDistance_(param.at("lane_change_complete_distance")),
          kTriggerReachedDistance_(param.at("trigger_reached_distance")),
          kStopVelocityThresholdMps_(param.at("stop_velocity_threshold_mps")),
          kPathTailPointCount_(param.at("path_tail_point_count")),
          kLeftPredictionHorizonSec_(
              param.at("left_prediction_horizon_sec")),
          kCompactLeftPredictionHorizonSec_(
              param.at("compact_left_prediction_horizon_sec")),
          kLeftMinConfirmedHits_(param.at("left_min_confirmed_hits")),
          kLeftMaxMissedScans_(param.at("left_max_missed_scans")),
          kLeftHardFrontGapM_(param.at("left_hard_front_gap_m")),
          kLeftHardRearGapM_(param.at("left_hard_rear_gap_m")),
          kLeftFrontTimeHeadwaySec_(param.at("left_front_time_headway_sec")),
          kLeftRearTimeHeadwaySec_(param.at("left_rear_time_headway_sec")),
          kLeftMinFrontTtcSec_(param.at("left_min_front_ttc_sec")),
          kLeftMinRearTtcSec_(param.at("left_min_rear_ttc_sec")),
          kLeftClosingSpeedEpsilonMps_(param.at("left_closing_speed_epsilon_mps")),
          kLeftSafeHoldSec_(param.at("left_safe_hold_sec")),
          kCurrentFrontTtcTriggerSec_(
              param.at("current_front_ttc_trigger_sec")),
          kLeftFrontMinTtcAdvantageSec_(
              param.at("left_front_min_ttc_advantage_sec")),
          kFrontTtcCompareRangeM_(
              param.at("front_ttc_compare_range_m")),
          kFrontTtcDisplayCapSec_(
              param.at("front_ttc_display_cap_sec")),
          kTtcBenefitHoldSec_(
              param.at("ttc_benefit_hold_sec")),
          kSideBySideFrontExtentM_(
              param.at("side_by_side_front_extent_m")),
          kSideBySideRearExtentM_(
              param.at("side_by_side_rear_extent_m")),
          kParallelRelativeSpeedMaxMps_(
              param.at("parallel_relative_speed_max_mps")),
          kSlotSpeedBiasMps_(param.at("slot_speed_bias_mps")),
          kSlotReadyMarginM_(param.at("slot_ready_margin_m")),
          kSlotIntentHoldSec_(param.at("slot_intent_hold_sec")),
          kSlotModeMinHoldSec_(param.at("slot_mode_min_hold_sec")),
          kSlotTimeoutSec_(param.at("slot_timeout_sec")),
          kSlotAccelLimitMps2_(param.at("slot_accel_limit_mps2")),
          kSlotDecelLimitMps2_(param.at("slot_decel_limit_mps2"))
    {
    }

    void resetStatus()
    {
        // 미션을 다시 진입할 때 이전 차선변경의 상태와 투영 좌표를 모두 초기화한다.
        current_state_ = State::INIT;
        current_lane_idx_ = kRightmostLaneIndex;
        pending_lane_step_ = 0;
        mission_complete_ = false;
        lane_maps_valid_ = false;
        high_pass_slow_reached_ = false;
        merge_stop_required_ = false;
        merge_stop_satisfied_ = false;
        lane_map_transition_failed_ = false;
        lane_change_start_path_pos_.clear();
        high_pass_slow_path_pos_.clear();
        lane_change_end_path_pos_.clear();
        lane_change_boundary_local_idx_ = 0;
        left_safety_candidate_active_ = false;
        left_safety_candidate_start_time_ = {};
        ttc_benefit_candidate_active_ = false;
        ttc_benefit_candidate_start_time_ = {};
        resetGapManeuver();
        status_log_started_ = false;
        last_status_log_time_ = {};
        last_front_distance_m_ = -1.0;
        last_mission_target_velocity_mps_ = 0.0;
        active_lane_change_velocity_mps_ = 0.0;
        active_lane_change_profile_velocity_mps_ = 0.0;
    }

    bool isMissionComplete() const
    {
        return mission_complete_;
    }

    int getCurrentLaneIdx() const
    {
        return current_lane_idx_;
    }

    std::vector<double> getProjectedTriggerPoints() const
    {
        // 플로팅 노드에 표시할 실제 경로상 시작점과 하이패스 감속점을 반환한다.
        std::vector<double> trigger_points;
        if (lane_change_start_path_pos_.size() < 2)
            return trigger_points;

        trigger_points.insert(
            trigger_points.end(),
            lane_change_start_path_pos_.begin(),
            lane_change_start_path_pos_.begin() + 2);

        if (current_lane_idx_ == kTargetLaneIdx_ &&
            high_pass_slow_path_pos_.size() >= 2)
        {
            // 감속점은 최종 목표 차로에 진입한 이후에만 표시한다.
            trigger_points.insert(
                trigger_points.end(),
                high_pass_slow_path_pos_.begin(),
                high_pass_slow_path_pos_.begin() + 2);
        }
        return trigger_points;
    }

    void drive()
    {
        // planning 주기마다 한 번 호출되는 미션 100의 최상위 상태 머신이다.
        if (current_state_ == State::INIT)
        {
            initializeMission();
        }

        if (current_state_ != State::INIT)
        {
            // 어느 주행 상태에서도 하이패스 감속점 통과 여부를 계속 확인한다.
            updateHighPassSlowStatus();
        }

        switch (current_state_)
        {
        case State::INIT:
            break;

        case State::WAIT_FOR_START:
            if (mustStopAtMergeLaneEnd())
            {
                // 시작점에 못 갔더라도 n-0 유효 맵 끝 50m 이내면 정지가 우선이다.
                enterMergeLaneSafetyStop();
            }
            else if (hasReachedPathPoint(local_path_, lane_change_start_path_pos_))
            {
                setState(State::WAIT_FOR_CLEAR, "lane-change start point reached");
            }
            break;

        case State::WAIT_FOR_CLEAR:
            handleLaneChangeReadyState();
            break;

        case State::LANE_CHANGE:
            if (isLaneChangePathComplete())
            {
                // 로컬패스 통과와 목표 차로 접근이 모두 확인된 뒤 맵을 바꾼다.
                completeLaneChange();
            }
            break;

        case State::CRUISE:
            if (current_lane_idx_ != kTargetLaneIdx_)
            {
                // 목표 차로가 남아 있으면 다음 한 칸 변경을 다시 준비한다.
                setState(State::WAIT_FOR_CLEAR, "additional lane change required");
            }
            else if (local_.checkMissionComplete() && isCurrentLocalPathNearEnd())
            {
                mission_complete_ = true;
                setState(State::DONE, "mission path complete");
            }
            break;

        case State::SAFE_STOP:
            handleSafeStopState();
            break;

        case State::DONE:
            break;
        }

        // 모든 상태에서 미션 목표속도보다 전방 거리 제어 결과를 우선 적용한다.
        applyCommonDistanceControl();
        // 다른 미션처럼 핵심 상태와 차선 정보는 planning 매 주기마다 표시한다.
        printCurrentState();
        // 상태 전환 로그와 별도로 현재 판단값을 1초마다 터미널에 요약한다.
        printPeriodicStatus();
    }

private:
    void initializeMission()
    {
        // n-0→4차로, n-1→3차로, n-2→2차로, n-3→1차로로 환산한다.
        current_lane_idx_ =
            kRightmostLaneIndex - static_cast<int>(local_.getCurPathIndex());
        current_lane_idx_ =
            std::clamp(current_lane_idx_, kLeftmostLaneIndex, kRightmostLaneIndex);

        // 파라미터 또는 필요한 맵 중 하나라도 잘못되면 주행하지 않고 정지한다.
        lane_maps_valid_ = validateParameters() && validateLaneMaps();
        if (!lane_maps_valid_)
        {
            setState(State::SAFE_STOP, "invalid parameters or missing lane map");
            return;
        }

        lane_change_start_path_pos_ =
            local_path_.getClosestPos(kLaneChangeStartPoint_);
        // param.json의 UTM 원본을 직접 판정하지 않고 현재 경로의 최근접점으로 사용한다.

        if (laneIndexError() == 0)
        {
            setState(State::CRUISE, "already in target lane");
        }
        else if (current_lane_idx_ == kRightmostLaneIndex &&
                 !hasReachedPathPoint(local_path_, lane_change_start_path_pos_))
        {
            setState(State::WAIT_FOR_START, "waiting for lane-change start point");
        }
        else
        {
            setState(State::WAIT_FOR_CLEAR, "lane changes enabled");
        }
    }

    bool validateParameters() const
    {
        // 차로 번호, UTM 좌표 개수, 거리·속도 파라미터의 물리적 범위를 확인한다.
        const bool target_lane_valid =
            kTargetLaneIdx_ >= kLeftmostLaneIndex &&
            kTargetLaneIdx_ <= kRightmostLaneIndex;
        const bool points_valid =
            kLaneChangeStartPoint_.size() >= 2 &&
            kHighPassSlowPoint_.size() >= 2;
        const bool distances_valid =
            kMergeLaneStopDistance_ > 0.0 &&
            kLaneChangeDurationSec_ > 0.0 &&
            kQuinticInterval_ > 0.0 &&
            kLaneChangeCompleteDistance_ >= 0.0 &&
            kTriggerReachedDistance_ >= 0.0 &&
            kStopVelocityThresholdMps_ >= 0.0 &&
            kPathTailPointCount_ > 0;
        const bool velocities_valid =
            kMergeVelocityMps_ > 0.0 &&
            kMainVelocityMps_ > 0.0 &&
            kHighPassVelocityMps_ >= 0.0 &&
            kMergeStopLaneChangeVelocityMps_ > 0.0;
        const bool left_safety_valid =
            kLeftPredictionHorizonSec_ > 0.0 &&
            kCompactLeftPredictionHorizonSec_ > 0.0 &&
            kLeftPredictionHorizonSec_ >= kLaneChangeDurationSec_ &&
            kCompactLeftPredictionHorizonSec_ >=
                kLaneChangeDurationSec_ &&
            kLeftMinConfirmedHits_ >= 2 &&
            kLeftMaxMissedScans_ >= 0 &&
            kLeftHardFrontGapM_ >= 0.0 &&
            kLeftHardRearGapM_ >= 0.0 &&
            kLeftFrontTimeHeadwaySec_ >= 0.0 &&
            kLeftRearTimeHeadwaySec_ >= 0.0 &&
            kLeftMinFrontTtcSec_ >= 0.0 &&
            kLeftMinRearTtcSec_ >= 0.0 &&
            kLeftClosingSpeedEpsilonMps_ >= 0.0 &&
            kLeftSafeHoldSec_ >= 0.0 &&
            kCurrentFrontTtcTriggerSec_ > 0.0 &&
            kLeftFrontMinTtcAdvantageSec_ >= 0.0 &&
            kFrontTtcCompareRangeM_ > 0.0 &&
            kFrontTtcDisplayCapSec_ >=
                kCurrentFrontTtcTriggerSec_ &&
            kTtcBenefitHoldSec_ >= 0.0;
        const bool gap_maneuver_valid =
            std::isfinite(kSideBySideFrontExtentM_) &&
            std::isfinite(kSideBySideRearExtentM_) &&
            std::isfinite(kParallelRelativeSpeedMaxMps_) &&
            std::isfinite(kSlotSpeedBiasMps_) &&
            std::isfinite(kSlotReadyMarginM_) &&
            std::isfinite(kSlotIntentHoldSec_) &&
            std::isfinite(kSlotModeMinHoldSec_) &&
            std::isfinite(kSlotTimeoutSec_) &&
            std::isfinite(kSlotAccelLimitMps2_) &&
            std::isfinite(kSlotDecelLimitMps2_) &&
            kSideBySideFrontExtentM_ >= 0.0 &&
            kSideBySideRearExtentM_ >= 0.0 &&
            kParallelRelativeSpeedMaxMps_ >= 0.0 &&
            kSlotSpeedBiasMps_ > 0.0 &&
            kSlotReadyMarginM_ >= 0.0 &&
            kSlotIntentHoldSec_ >= 0.0 &&
            kSlotModeMinHoldSec_ >= 0.0 &&
            kSlotTimeoutSec_ >= kSlotModeMinHoldSec_ &&
            kSlotAccelLimitMps2_ > 0.0 &&
            kSlotDecelLimitMps2_ > 0.0;

        return target_lane_valid && points_valid && distances_valid &&
               velocities_valid && left_safety_valid &&
               gap_maneuver_valid;
    }

    bool validateLaneMaps()
    {
        // 목표 차로를 n-x 파일의 x 인덱스로 변환한다(1차로이면 x=3).
        const int target_path_idx =
            kRightmostLaneIndex - kTargetLaneIdx_;
        const int current_path_idx =
            static_cast<int>(local_.getCurPathIndex());

        if (target_path_idx < current_path_idx)
        {
            // 현재 LiDAR ROI는 왼쪽만 확인하므로 오른쪽 차선변경은 허용하지 않는다.
            std::cerr << "[Mission 100] Right lane changes are not supported by "
                         "the configured left-side LiDAR ROI.\n";
            return false;
        }

        const std::string mission_prefix =
            std::to_string(local_.getCurMissionIndex()) + "-";
        // 미션 번호를 하드코딩하지 않고 현재 n에 해당하는 n-0~n-3을 검사한다.
        for (int path_idx = current_path_idx; path_idx <= target_path_idx; ++path_idx)
        {
            const Path &path = local_.getRefIdxPath(path_idx);
            const std::string expected_name =
                mission_prefix + std::to_string(path_idx);
            if (path.getPathName() != expected_name ||
                path.getRefPosArr().size() <= kPathTailPointCount_ + 1 ||
                path.getRefPosArr().size() != path.getRefYawArr().size() ||
                path.getRefPosArr().size() != path.getRefKArr().size())
            {
                // 위치·yaw·곡률 배열 크기가 다르거나 유효 길이가 부족하면 정지한다.
                std::cerr << "[Mission 100] Invalid lane map: "
                          << expected_name << "\n";
                return false;
            }
        }
        return true;
    }

    void handleLaneChangeReadyState()
    {
        if (mustStopAtMergeLaneEnd())
        {
            // 좌측 차선 확인보다 4차로 유효 맵 끝 안전정지를 우선한다.
            enterMergeLaneSafetyStop();
            return;
        }

        // lane_error = target-current: 음수는 좌측, 0은 도착, 양수는 우측이다.
        const int lane_error = laneIndexError();
        if (lane_error == 0)
        {
            setState(State::CRUISE, "target lane reached");
            return;
        }

        if (lane_error > 0)
        {
            setState(State::SAFE_STOP, "right lane change requested");
            return;
        }

        if (!prepareLongitudinalGapForLaneChange())
        {
            // 나란한 좌측 차량과 앞/뒤 종방향 간격을 만들기 전에는 횡이동하지 않는다.
            resetLeftLaneSafetyConfirmation();
            resetTtcBenefitConfirmation();
            return;
        }

        if (!ttcBenefitAllowsLaneChange())
        {
            // 3·2차로에서 느린 선행차를 피할 때는 왼쪽 전방 TTC가 더 좋아야 한다.
            return;
        }

        const double lane_change_velocity =
            calculateLaneChangeVelocityMps(
                LaneChangeProfile::REGULAR);
        if (lane_change_velocity <= kDistanceEpsilon_)
        {
            // 전방 거리 제어가 정지를 요구하면 횡이동 경로를 만들지 않는다.
            resetLeftLaneSafetyConfirmation();
            return;
        }

        if (isLeftLaneChangeSafe(
                LaneChangeProfile::REGULAR,
                lane_change_velocity))
        {
            // 모든 좌측 트랙의 현재·예측 간격과 TTC가 안전할 때만 경로를 만든다.
            if (createLaneChangePath(
                    lane_change_velocity))
            {
                // 슬롯 결과와 거리제어 상한을 경로·안전예측·실제 횡이동에 같은 값으로 확정한다.
                active_lane_change_velocity_mps_ =
                    lane_change_velocity;
                active_lane_change_profile_velocity_mps_ =
                    laneChangeProfileVelocityMps(
                        LaneChangeProfile::REGULAR);
                setState(State::LANE_CHANGE, "left lane prediction is safe");
            }
            else
            {
                const double regular_forward_distance =
                    lane_change_velocity *
                    kLaneChangeDurationSec_;
                if (current_lane_idx_ == kRightmostLaneIndex &&
                    local_.getCurPathIndex() == 0 &&
                    remainingMapDistance() +
                            kDistanceEpsilon_ <
                        regular_forward_distance)
                {
                    // 50m 직전에는 일반 경로 길이가 부족할 수 있으므로 정지 후 축소 경로로 복구한다.
                    merge_stop_required_ = true;
                }
                setState(State::SAFE_STOP, "lane-change path generation failed");
            }
        }
    }

    void resetLeftLaneSafetyConfirmation()
    {
        left_safety_candidate_active_ = false;
        left_safety_candidate_start_time_ = {};
    }

    void resetTtcBenefitConfirmation()
    {
        ttc_benefit_candidate_active_ = false;
        ttc_benefit_candidate_start_time_ = {};
    }

    void resetGapManeuver()
    {
        gap_mode_ = GapMode::NONE;
        gap_candidate_mode_ = GapMode::NONE;
        gap_intent_mode_ = GapMode::NONE;
        gap_anchor_track_id_ = 0;
        gap_candidate_active_ = false;
        gap_candidate_start_time_ = {};
        gap_hold_release_candidate_active_ = false;
        gap_hold_release_candidate_start_time_ = {};
        gap_mode_start_time_ = {};
        gap_velocity_override_active_ = false;
        gap_command_velocity_mps_ = 0.0;
        gap_desired_velocity_mps_ = 0.0;
        gap_velocity_update_started_ = false;
        last_gap_velocity_update_time_ = {};
        gap_pass_abandoned_ = false;
        last_gap_anchor_min_x_m_ = 0.0;
        last_gap_anchor_max_x_m_ = 0.0;
        last_gap_anchor_relative_velocity_mps_ = 0.0;
        last_gap_anchor_speed_mps_ = 0.0;
        last_gap_actual_m_ = 0.0;
        last_gap_required_m_ = 0.0;
        gap_progress_reference_x_m_ = 0.0;
        last_gap_pass_feasible_ = false;
        gap_reason_ = "슬롯 미사용";
    }

    void setGapMode(GapMode next_mode, const std::string &reason)
    {
        gap_reason_ = reason;
        if (gap_mode_ == next_mode)
            return;

        const GapMode previous_mode = gap_mode_;
        gap_mode_ = next_mode;
        gap_mode_start_time_ = std::chrono::steady_clock::now();
        std::cout << "[Mission 100] 종방향 슬롯 전환: "
                  << gapModeName(previous_mode) << " -> "
                  << gapModeName(next_mode)
                  << " | 이유: " << reason << "\n";
    }

    bool isSlotTrackGeometryValid(
        const Mission100LeftTrack &track) const
    {
        return std::isfinite(track.center_x) &&
               std::isfinite(track.center_y) &&
               std::isfinite(track.min_x) &&
               std::isfinite(track.max_x) &&
               std::isfinite(track.relative_velocity_x) &&
               track.min_x <= track.max_x &&
               track.center_x >= track.min_x - kDistanceEpsilon_ &&
               track.center_x <= track.max_x + kDistanceEpsilon_;
    }

    bool isTrustedSlotTrack(
        const Mission100LeftTrack &track) const
    {
        // missed track의 bbox는 마지막 관측 위치이므로 슬롯 완료 증거로 사용하지 않는다.
        return isSlotTrackGeometryValid(track) &&
               track.hit_count >= kLeftMinConfirmedHits_ &&
               track.missed_scans == 0 &&
               track.velocity_valid;
    }

    bool isSideBySideTrack(
        const Mission100LeftTrack &track) const
    {
        // LiDAR 원점 한 점이 아니라 자차의 앞·뒤 차체 범위와 겹치는지 확인한다.
        return isSlotTrackGeometryValid(track) &&
               track.min_x <= kSideBySideFrontExtentM_ &&
               track.max_x >= -kSideBySideRearExtentM_;
    }

    bool findGapTrackById(
        const std::vector<Mission100LeftTrack> &tracks,
        int track_id,
        Mission100LeftTrack &selected_track) const
    {
        for (const auto &track : tracks)
        {
            if (track.id == track_id)
            {
                selected_track = track;
                return true;
            }
        }
        return false;
    }

    bool selectSideBySideAnchor(
        const std::vector<Mission100LeftTrack> &tracks,
        Mission100LeftTrack &selected_track,
        bool &ambiguous) const
    {
        ambiguous = false;
        int trusted_candidate_count = 0;

        for (const auto &track : tracks)
        {
            if (!isSideBySideTrack(track))
                continue;

            if (!isTrustedSlotTrack(track))
            {
                // 나란한 범위의 미확정 물체를 무시하고 다른 ID를 고르지 않는다.
                ambiguous = true;
                continue;
            }

            ++trusted_candidate_count;
            selected_track = track;
        }

        if (trusted_candidate_count > 1)
            ambiguous = true;
        return trusted_candidate_count == 1 && !ambiguous;
    }

    bool hasConflictingSideBySideTrack(
        const std::vector<Mission100LeftTrack> &tracks,
        int anchor_track_id) const
    {
        // anchor를 고른 뒤 새 차량이 나란한 영역에 들어오면 단일차량 가정을 즉시 폐기한다.
        for (const auto &track : tracks)
        {
            if (track.id == anchor_track_id)
                continue;
            if (isSideBySideTrack(track))
                return true;
        }
        return false;
    }

    double baseCommandedVelocity() const
    {
        if (!lane_maps_valid_ || current_state_ == State::SAFE_STOP)
            return 0.0;
        if (merge_stop_satisfied_ &&
            current_state_ == State::LANE_CHANGE &&
            current_lane_idx_ == kRightmostLaneIndex)
        {
            return kMergeStopLaneChangeVelocityMps_;
        }
        if (high_pass_slow_reached_)
            return kHighPassVelocityMps_;
        if (current_lane_idx_ == kRightmostLaneIndex)
            return kMergeVelocityMps_;
        return kMainVelocityMps_;
    }

    void updateGapCommandVelocity(double desired_velocity_mps)
    {
        const double base_velocity = baseCommandedVelocity();
        desired_velocity_mps =
            std::max(0.0, std::min(base_velocity, desired_velocity_mps));
        gap_desired_velocity_mps_ = desired_velocity_mps;

        const auto now = std::chrono::steady_clock::now();
        if (!gap_velocity_update_started_)
        {
            const double measured_velocity = local_.getCurCarVelocity();
            gap_command_velocity_mps_ =
                std::isfinite(measured_velocity)
                    ? std::max(
                          0.0,
                          std::min(base_velocity, measured_velocity))
                    : desired_velocity_mps;
            gap_velocity_update_started_ = true;
            last_gap_velocity_update_time_ = now;
        }

        double elapsed_sec =
            std::chrono::duration<double>(
                now - last_gap_velocity_update_time_)
                .count();
        elapsed_sec = std::max(0.0, std::min(0.1, elapsed_sec));
        last_gap_velocity_update_time_ = now;

        if (desired_velocity_mps > gap_command_velocity_mps_)
        {
            gap_command_velocity_mps_ =
                std::min(
                    desired_velocity_mps,
                    gap_command_velocity_mps_ +
                        kSlotAccelLimitMps2_ * elapsed_sec);
        }
        else
        {
            gap_command_velocity_mps_ =
                std::max(
                    desired_velocity_mps,
                    gap_command_velocity_mps_ -
                        kSlotDecelLimitMps2_ * elapsed_sec);
        }
        gap_command_velocity_mps_ =
            std::max(0.0, std::min(base_velocity, gap_command_velocity_mps_));
        gap_velocity_override_active_ = true;
    }

    void holdLastGapCommand(
        double base_velocity,
        double fallback_velocity,
        const std::string &reason)
    {
        // 순간적인 트랙 유실이 곧바로 100/120km/h 목표 복귀로 이어지지 않게 마지막 명령을 유지한다.
        const double hold_velocity =
            gap_velocity_override_active_
                ? gap_command_velocity_mps_
                : std::max(
                      0.0,
                      std::min(base_velocity, fallback_velocity));
        gap_candidate_active_ = false;
        gap_candidate_mode_ = GapMode::NONE;
        gap_intent_mode_ = GapMode::NONE;
        gap_hold_release_candidate_active_ = false;
        gap_hold_release_candidate_start_time_ = {};
        setGapMode(GapMode::HOLD, reason);
        updateGapCommandVelocity(hold_velocity);
    }

    bool releaseGapHoldAfterValidEmpty(double base_velocity)
    {
        if (gap_mode_ != GapMode::HOLD ||
            !gap_velocity_override_active_)
        {
            return true;
        }

        // 트랙이 다시 빈 상태로 보인 한 프레임만으로 HOLD를 풀지 않고 기존 안전 유지시간을 재사용한다.
        const auto now = std::chrono::steady_clock::now();
        if (!gap_hold_release_candidate_active_)
        {
            gap_hold_release_candidate_active_ = true;
            gap_hold_release_candidate_start_time_ = now;
        }
        updateGapCommandVelocity(
            std::min(base_velocity, gap_command_velocity_mps_));
        const double valid_empty_duration_sec =
            std::chrono::duration<double>(
                now - gap_hold_release_candidate_start_time_)
                .count();
        return valid_empty_duration_sec >= kLeftSafeHoldSec_;
    }

    double egoAdditionalDistanceForTarget(
        double current_velocity,
        double target_velocity,
        double acceleration_limit,
        double horizon_sec) const
    {
        if (!std::isfinite(current_velocity) ||
            !std::isfinite(target_velocity) ||
            acceleration_limit <= 0.0 ||
            horizon_sec <= 0.0)
        {
            return 0.0;
        }

        const double velocity_delta = target_velocity - current_velocity;
        if (std::abs(velocity_delta) <= kDistanceEpsilon_)
            return 0.0;

        const double acceleration =
            velocity_delta > 0.0
                ? acceleration_limit
                : -acceleration_limit;
        const double target_reach_time =
            std::min(
                horizon_sec,
                std::abs(velocity_delta) / acceleration_limit);
        return 0.5 * acceleration *
                   target_reach_time * target_reach_time +
               velocity_delta *
                   std::max(0.0, horizon_sec - target_reach_time);
    }

    bool hasMergeGapPreparationTime(double ego_velocity) const
    {
        if (current_lane_idx_ != kRightmostLaneIndex ||
            local_.getCurPathIndex() != 0 ||
            merge_stop_satisfied_)
        {
            return true;
        }

        const double available_distance =
            remainingMapDistance() - kMergeLaneStopDistance_;
        if (available_distance <= 0.0)
            return false;

        const double reference_velocity =
            std::max(kStopVelocityThresholdMps_, ego_velocity);
        const double available_time =
            available_distance / reference_velocity;
        const double required_time =
            kSlotIntentHoldSec_ +
            kSlotModeMinHoldSec_ +
            kLeftSafeHoldSec_;
        return available_time >= required_time;
    }

    bool isPassAheadFeasible(
        const Mission100LeftTrack &anchor,
        double ego_velocity,
        double side_vehicle_velocity,
        double base_velocity,
        double &pass_target_velocity,
        std::string &reason)
    {
        pass_target_velocity =
            std::min(
                base_velocity,
                std::max(
                    ego_velocity,
                    side_vehicle_velocity + kSlotSpeedBiasMps_));

        // 순간 과속이 아니라 미션 상한 안에서 지속 가능한 속도차로 추월 가능성을 판정한다.
        const double available_speed_advantage =
            pass_target_velocity - side_vehicle_velocity;
        if (available_speed_advantage + kDistanceEpsilon_ <
            kSlotSpeedBiasMps_)
        {
            reason = "미션 속도 상한으로 옆차보다 충분히 빨라질 수 없음";
            return false;
        }

        const double scalar_front_distance =
            lidar_.getRefCarFrontCarDistance();
        const double required_front_gap =
            kLeftHardFrontGapM_ +
            kLeftFrontTimeHeadwaySec_ *
                std::max(ego_velocity, pass_target_velocity);
        if (!std::isfinite(scalar_front_distance) ||
            scalar_front_distance < required_front_gap)
        {
            reason = "현재차로 전방 거리 부족 또는 거리 토픽 무효";
            return false;
        }

        std::vector<Mission100LeftTrack> current_front_tracks;
        if (!lidar_.getMission100CurrentFrontTracks(
                current_front_tracks))
        {
            reason = "현재차로 전방 트랙 무효/시간초과";
            return false;
        }

        const double horizon_sec = kLeftPredictionHorizonSec_;
        const double ego_extra_distance =
            egoAdditionalDistanceForTarget(
                ego_velocity,
                pass_target_velocity,
                kSlotAccelLimitMps2_,
                horizon_sec);
        const double reachable_velocity_delta =
            std::min(
                std::max(0.0, pass_target_velocity - ego_velocity),
                kSlotAccelLimitMps2_ * horizon_sec);

        for (const auto &front_track : current_front_tracks)
        {
            if (!isSlotTrackGeometryValid(front_track) ||
                front_track.max_x <= 0.0)
            {
                reason = "현재차로 전방 트랙 형식이 신뢰 불가";
                return false;
            }
            if (front_track.min_x > kFrontTtcCompareRangeM_)
                continue;
            if (!isTrustedSlotTrack(front_track))
            {
                reason = "현재차로 선행차 속도 확정 대기";
                return false;
            }

            const double front_gap = std::max(0.0, front_track.min_x);
            const double front_vehicle_velocity =
                std::max(
                    0.0,
                    ego_velocity +
                        front_track.relative_velocity_x);
            if (front_vehicle_velocity <=
                side_vehicle_velocity +
                    kParallelRelativeSpeedMaxMps_)
            {
                reason = "현재차로 선행차가 옆차와 비슷하거나 더 느림";
                return false;
            }

            const double predicted_front_gap =
                front_gap +
                front_track.relative_velocity_x * horizon_sec -
                ego_extra_distance;
            const double predicted_relative_velocity =
                front_track.relative_velocity_x -
                reachable_velocity_delta;
            const bool closing =
                predicted_relative_velocity <
                -kLeftClosingSpeedEpsilonMps_;
            const double predicted_ttc =
                closing
                    ? front_gap /
                          (-predicted_relative_velocity)
                    : std::numeric_limits<double>::infinity();

            if (predicted_front_gap < required_front_gap ||
                predicted_ttc < kLeftMinFrontTtcSec_)
            {
                reason = "가속 후 현재차로 선행차 간격/TTC 부족";
                return false;
            }
        }

        // anchor는 현재 좌측차로 물체이며 current-front tracker의 ID와 비교하지 않는다.
        (void)anchor;
        reason = "현재차로 앞 공간이 안전해 앞 슬롯 선택 가능";
        return true;
    }

    bool isSelectedSlotActuallyReady(
        const Mission100LeftTrack &anchor,
        GapMode intent_mode,
        double ego_velocity,
        double side_vehicle_velocity,
        double base_velocity)
    {
        if (intent_mode == GapMode::PASS_AHEAD)
        {
            last_gap_actual_m_ =
                anchor.max_x < 0.0 ? -anchor.max_x : 0.0;
            last_gap_required_m_ =
                kLeftHardRearGapM_ +
                kLeftRearTimeHeadwaySec_ *
                    side_vehicle_velocity +
                kSlotReadyMarginM_;
            return anchor.max_x < 0.0 &&
                   last_gap_actual_m_ >=
                       last_gap_required_m_;
        }

        if (intent_mode == GapMode::YIELD_BEHIND)
        {
            last_gap_actual_m_ =
                anchor.min_x > 0.0 ? anchor.min_x : 0.0;
            last_gap_required_m_ =
                kLeftHardFrontGapM_ +
                kLeftFrontTimeHeadwaySec_ *
                    std::max(ego_velocity, base_velocity) +
                kSlotReadyMarginM_;
            return anchor.min_x > 0.0 &&
                   last_gap_actual_m_ >=
                       last_gap_required_m_;
        }

        last_gap_actual_m_ = 0.0;
        last_gap_required_m_ = 0.0;
        return false;
    }

    void updateGapAnchorDiagnostics(
        const Mission100LeftTrack &anchor,
        double side_vehicle_velocity)
    {
        last_gap_anchor_min_x_m_ = anchor.min_x;
        last_gap_anchor_max_x_m_ = anchor.max_x;
        last_gap_anchor_relative_velocity_mps_ =
            anchor.relative_velocity_x;
        last_gap_anchor_speed_mps_ = side_vehicle_velocity;
    }

    bool prepareLongitudinalGapForLaneChange()
    {
        std::vector<Mission100LeftTrack> left_tracks;
        if (!lidar_.getMission100LeftTracks(left_tracks))
        {
            gap_anchor_track_id_ = 0;
            gap_pass_abandoned_ = false;
            const double base_velocity = baseCommandedVelocity();
            const double measured_velocity = local_.getCurCarVelocity();
            holdLastGapCommand(
                base_velocity,
                std::isfinite(measured_velocity)
                    ? std::max(0.0, measured_velocity)
                    : base_velocity,
                "좌측 트랙 무효/시간초과로 슬롯 판단 보류");
            return false;
        }

        const double ego_velocity = local_.getCurCarVelocity();
        if (!std::isfinite(ego_velocity))
        {
            gap_anchor_track_id_ = 0;
            gap_pass_abandoned_ = false;
            const double base_velocity = baseCommandedVelocity();
            holdLastGapCommand(
                base_velocity,
                base_velocity,
                "자차 속도 무효로 슬롯 판단 보류");
            return false;
        }
        const double nonnegative_ego_velocity =
            std::max(0.0, ego_velocity);
        const double base_velocity = baseCommandedVelocity();

        Mission100LeftTrack anchor;
        if (gap_anchor_track_id_ > 0)
        {
            if (!findGapTrackById(
                    left_tracks,
                    gap_anchor_track_id_,
                    anchor))
            {
                gap_anchor_track_id_ = 0;
                gap_pass_abandoned_ = false;
                holdLastGapCommand(
                    base_velocity,
                    nonnegative_ego_velocity,
                    "기존 슬롯 차량 ID 소실; 유효 빈 상태 재확인");
                resetLeftLaneSafetyConfirmation();
                return false;
            }
            if (!isTrustedSlotTrack(anchor))
            {
                holdLastGapCommand(
                    base_velocity,
                    nonnegative_ego_velocity,
                    "선택 차량이 누락됐거나 속도가 미확정");
                return false;
            }
            if (hasConflictingSideBySideTrack(
                    left_tracks,
                    gap_anchor_track_id_))
            {
                gap_anchor_track_id_ = 0;
                gap_pass_abandoned_ = false;
                holdLastGapCommand(
                    base_velocity,
                    nonnegative_ego_velocity,
                    "선택 차량 외 나란한 트랙이 추가되어 재평가");
                resetLeftLaneSafetyConfirmation();
                return false;
            }
        }
        else
        {
            bool ambiguous = false;
            const bool anchor_selected =
                selectSideBySideAnchor(
                    left_tracks, anchor, ambiguous);
            if (ambiguous)
            {
                holdLastGapCommand(
                    base_velocity,
                    nonnegative_ego_velocity,
                    "나란한 범위에 복수 또는 미확정 트랙 존재");
                return false;
            }
            if (!anchor_selected)
            {
                // 센서 HOLD 직후라면 유효한 빈 상태를 연속 확인한 뒤 기존 안전판정으로 넘긴다.
                if (!releaseGapHoldAfterValidEmpty(base_velocity))
                {
                    gap_reason_ =
                        "좌측 유효 빈 상태 연속 확인 중";
                    return false;
                }
                resetGapManeuver();
                return true;
            }

            gap_anchor_track_id_ = anchor.id;
            gap_pass_abandoned_ = false;
            gap_candidate_active_ = false;
            gap_hold_release_candidate_active_ = false;
            gap_hold_release_candidate_start_time_ = {};
            gap_candidate_mode_ = GapMode::NONE;
            gap_intent_mode_ = GapMode::NONE;
            setGapMode(
                GapMode::ASSESS,
                "나란한 좌측 차량 확인; 앞/뒤 슬롯 평가 시작");
        }

        const double side_vehicle_velocity =
            std::max(
                0.0,
                nonnegative_ego_velocity +
                    anchor.relative_velocity_x);
        updateGapAnchorDiagnostics(
            anchor, side_vehicle_velocity);

        if (!hasMergeGapPreparationTime(
                nonnegative_ego_velocity))
        {
            setGapMode(
                GapMode::HOLD,
                "4차로 50m 안전정지 전 슬롯 준비시간 부족");
            updateGapCommandVelocity(
                std::min(
                    base_velocity,
                    nonnegative_ego_velocity));
            return false;
        }

        double pass_target_velocity = base_velocity;
        std::string pass_reason;
        const bool pass_feasible =
            !gap_pass_abandoned_ &&
            isPassAheadFeasible(
                anchor,
                nonnegative_ego_velocity,
                side_vehicle_velocity,
                base_velocity,
                pass_target_velocity,
                pass_reason);
        last_gap_pass_feasible_ = pass_feasible;

        if (gap_mode_ == GapMode::NONE ||
            gap_mode_ == GapMode::HOLD)
        {
            gap_candidate_active_ = false;
            gap_candidate_mode_ = GapMode::NONE;
            setGapMode(
                GapMode::ASSESS,
                "신뢰 가능한 선택 차량으로 슬롯 방향 재평가");
        }

        if (gap_mode_ == GapMode::ASSESS)
        {
            GapMode preferred_mode =
                pass_feasible
                    ? GapMode::PASS_AHEAD
                    : GapMode::YIELD_BEHIND;

            // 옆차가 이미 앞으로 빠지는 중이면 불필요하게 다시 추월하지 않는다.
            if (anchor.relative_velocity_x >
                kParallelRelativeSpeedMaxMps_)
            {
                preferred_mode = GapMode::YIELD_BEHIND;
            }

            const auto now = std::chrono::steady_clock::now();
            if (!gap_candidate_active_ ||
                gap_candidate_mode_ != preferred_mode)
            {
                gap_candidate_active_ = true;
                gap_candidate_mode_ = preferred_mode;
                gap_candidate_start_time_ = now;
            }

            updateGapCommandVelocity(
                std::min(
                    base_velocity,
                    nonnegative_ego_velocity));
            const double candidate_duration =
                std::chrono::duration<double>(
                    now - gap_candidate_start_time_)
                    .count();
            if (candidate_duration < kSlotIntentHoldSec_)
            {
                gap_reason_ =
                    preferred_mode == GapMode::PASS_AHEAD
                        ? "앞 슬롯 선택 연속 확인 중"
                        : "뒤 슬롯 선택 연속 확인 중";
                return false;
            }

            gap_intent_mode_ = preferred_mode;
            gap_candidate_active_ = false;
            setGapMode(
                preferred_mode,
                preferred_mode == GapMode::PASS_AHEAD
                    ? pass_reason
                    : "앞 슬롯 불가 또는 옆차가 전진하여 뒤 슬롯 선택");
            if (preferred_mode == GapMode::PASS_AHEAD)
                gap_progress_reference_x_m_ = anchor.max_x;
            else
                gap_progress_reference_x_m_ = anchor.min_x;
        }

        if (gap_mode_ == GapMode::PASS_AHEAD)
        {
            const double mode_elapsed_sec =
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() -
                    gap_mode_start_time_)
                    .count();
            if (!pass_feasible)
            {
                gap_pass_abandoned_ = true;
                gap_intent_mode_ = GapMode::YIELD_BEHIND;
                gap_progress_reference_x_m_ = anchor.min_x;
                setGapMode(
                    GapMode::YIELD_BEHIND,
                    pass_reason);
            }
            else if (mode_elapsed_sec >= kSlotTimeoutSec_)
            {
                const double progress_m =
                    gap_progress_reference_x_m_ -
                    anchor.max_x;
                if (progress_m >= kSlotReadyMarginM_)
                {
                    // 앞쪽으로 실제 진행 중이면 고정 timeout 때문에 직전에 포기하지 않게 연장한다.
                    gap_progress_reference_x_m_ =
                        anchor.max_x;
                    gap_mode_start_time_ =
                        std::chrono::steady_clock::now();
                    gap_reason_ =
                        "앞 슬롯 진행 확인; 준비시간 연장";
                }
                else
                {
                    gap_pass_abandoned_ = true;
                    gap_intent_mode_ = GapMode::YIELD_BEHIND;
                    gap_progress_reference_x_m_ =
                        anchor.min_x;
                    setGapMode(
                        GapMode::YIELD_BEHIND,
                        "앞 슬롯 준비시간 동안 진전 없음; 뒤 슬롯으로 전환");
                }
            }
            else
            {
                gap_intent_mode_ = GapMode::PASS_AHEAD;
                updateGapCommandVelocity(
                    pass_target_velocity);
            }
        }

        if (gap_mode_ == GapMode::YIELD_BEHIND)
        {
            gap_intent_mode_ = GapMode::YIELD_BEHIND;
            const double mode_elapsed_sec =
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() -
                    gap_mode_start_time_)
                    .count();
            if (mode_elapsed_sec >= kSlotTimeoutSec_)
            {
                const double progress_m =
                    anchor.min_x -
                    gap_progress_reference_x_m_;
                if (progress_m >= kSlotReadyMarginM_)
                {
                    // 실제로 뒤 슬롯 쪽으로 벌어지는 중이면 같은 시간만큼 계속 관찰한다.
                    gap_progress_reference_x_m_ = anchor.min_x;
                    gap_mode_start_time_ =
                        std::chrono::steady_clock::now();
                }
                else
                {
                    gap_anchor_track_id_ = 0;
                    gap_pass_abandoned_ = false;
                    holdLastGapCommand(
                        base_velocity,
                        nonnegative_ego_velocity,
                        "뒤 슬롯 준비시간 동안 진전 없음; 차량과 방향 재평가");
                    return false;
                }
            }

            const double yield_target_velocity =
                std::max(
                    0.0,
                    std::min(
                        base_velocity,
                        side_vehicle_velocity -
                            kSlotSpeedBiasMps_));
            updateGapCommandVelocity(
                yield_target_velocity);
            gap_reason_ =
                "옆차 뒤 슬롯 확보를 위해 완만하게 감속 중";
        }

        const bool actual_slot_ready =
            isSelectedSlotActuallyReady(
                anchor,
                gap_intent_mode_,
                nonnegative_ego_velocity,
                side_vehicle_velocity,
                base_velocity);
        const double current_mode_elapsed_sec =
            gap_mode_ == GapMode::READY
                ? kSlotModeMinHoldSec_
                : std::chrono::duration<double>(
                      std::chrono::steady_clock::now() -
                      gap_mode_start_time_)
                      .count();
        if (actual_slot_ready &&
            current_mode_elapsed_sec >=
                kSlotModeMinHoldSec_)
        {
            setGapMode(
                GapMode::READY,
                gap_intent_mode_ == GapMode::PASS_AHEAD
                    ? "옆차 앞쪽 후방 안전간격 확보"
                    : "옆차 뒤쪽 전방 안전간격 확보");
            return true;
        }

        if (gap_mode_ == GapMode::READY)
        {
            setGapMode(
                gap_intent_mode_,
                "확보했던 종방향 슬롯 간격이 다시 부족해짐");
        }
        return false;
    }

    FrontTtcEstimate estimateFrontTtc(
        const std::vector<Mission100LeftTrack> &tracks,
        bool stream_valid) const
    {
        FrontTtcEstimate estimate;
        estimate.stream_valid = stream_valid;
        estimate.track_count = tracks.size();
        if (!stream_valid)
            return estimate;

        estimate.data_available = true;
        bool estimate_selected = false;
        for (const auto &track : tracks)
        {
            if (!std::isfinite(track.min_x) ||
                !std::isfinite(track.max_x) ||
                !std::isfinite(track.relative_velocity_x) ||
                track.min_x > track.max_x)
            {
                estimate.data_available = false;
                continue;
            }

            // 왼쪽 토픽의 후방 트랙은 TTC 이득 비교에서 제외하되 안전판정에는 남긴다.
            if (track.max_x <= 0.0)
                continue;

            const double gap_m = std::max(0.0, track.min_x);
            if (gap_m > kFrontTtcCompareRangeM_)
                continue;

            estimate.front_present = true;
            if (track.hit_count < kLeftMinConfirmedHits_ ||
                track.missed_scans != 0 ||
                !track.velocity_valid)
            {
                // bbox가 갱신되지 않은 missed track은 TTC 비교 자료로 사용하지 않는다.
                estimate.data_available = false;
                continue;
            }

            const bool closing =
                track.relative_velocity_x <
                -kLeftClosingSpeedEpsilonMps_;
            const double ttc_sec =
                closing
                    ? gap_m / (-track.relative_velocity_x)
                    : std::numeric_limits<double>::infinity();
            const bool both_infinite =
                !std::isfinite(ttc_sec) &&
                !std::isfinite(estimate.ttc_sec);
            if (!estimate_selected ||
                ttc_sec < estimate.ttc_sec ||
                (both_infinite && gap_m < estimate.gap_m))
            {
                // 여러 선행차 중 가장 작은 TTC가 그 차로의 대표 위험도가 된다.
                estimate_selected = true;
                estimate.gap_m = gap_m;
                estimate.relative_velocity_mps =
                    track.relative_velocity_x;
                estimate.ttc_sec = ttc_sec;
            }
        }

        return estimate;
    }

    TtcBenefitAssessment evaluateTtcBenefit() const
    {
        TtcBenefitAssessment assessment;
        std::vector<Mission100LeftTrack> current_tracks;
        std::vector<Mission100LeftTrack> left_tracks;
        const bool current_stream_valid =
            lidar_.getMission100CurrentFrontTracks(current_tracks);
        const bool left_stream_valid =
            lidar_.getMission100LeftTracks(left_tracks);
        assessment.current_front =
            estimateFrontTtc(current_tracks, current_stream_valid);
        assessment.left_front =
            estimateFrontTtc(left_tracks, left_stream_valid);

        if (current_lane_idx_ == kRightmostLaneIndex)
        {
            // 4→3 합류는 맵 끝 안전정지를 포함한 기존 좌측 안전조건만 사용한다.
            assessment.reason = TtcBenefitReason::MANDATORY_MERGE;
            return assessment;
        }

        if (!assessment.current_front.stream_valid)
        {
            // TTC 자료 하나의 누락 때문에 4→1 목표 주행이 영구 정지하지 않게 기존 동작으로 복귀한다.
            assessment.reason =
                TtcBenefitReason::CURRENT_STREAM_UNKNOWN;
            return assessment;
        }
        if (!assessment.current_front.data_available)
        {
            assessment.reason =
                TtcBenefitReason::CURRENT_TRACK_UNKNOWN;
            return assessment;
        }
        if (!assessment.current_front.front_present)
        {
            assessment.reason = TtcBenefitReason::NO_CURRENT_FRONT;
            return assessment;
        }
        if (!std::isfinite(assessment.current_front.ttc_sec))
        {
            assessment.reason = TtcBenefitReason::CURRENT_NOT_CLOSING;
            return assessment;
        }
        if (assessment.current_front.ttc_sec >
            kCurrentFrontTtcTriggerSec_)
        {
            assessment.reason =
                TtcBenefitReason::CURRENT_TTC_COMFORTABLE;
            return assessment;
        }

        // 현재 차로 선행차를 빠르게 따라잡을 때만 왼쪽 차로의 TTC 우위를 추가 요구한다.
        assessment.benefit_required = true;
        if (!assessment.left_front.stream_valid)
        {
            assessment.reason = TtcBenefitReason::LEFT_STREAM_UNKNOWN;
            return assessment;
        }
        if (!assessment.left_front.data_available)
        {
            assessment.reason = TtcBenefitReason::LEFT_TRACK_UNKNOWN;
            return assessment;
        }

        assessment.advantage_sec =
            assessment.left_front.ttc_sec -
            assessment.current_front.ttc_sec;
        assessment.raw_benefit =
            !std::isfinite(assessment.left_front.ttc_sec) ||
            assessment.advantage_sec >=
                kLeftFrontMinTtcAdvantageSec_;
        assessment.reason =
            assessment.raw_benefit
                ? TtcBenefitReason::BENEFICIAL
                : TtcBenefitReason::INSUFFICIENT_ADVANTAGE;
        return assessment;
    }

    bool ttcBenefitAllowsLaneChange()
    {
        const TtcBenefitAssessment assessment =
            evaluateTtcBenefit();

        if (assessment.reason ==
                TtcBenefitReason::CURRENT_STREAM_UNKNOWN ||
            assessment.reason ==
                TtcBenefitReason::CURRENT_TRACK_UNKNOWN)
        {
            // 현재차로 전방을 모르는 상태는 차량 없음과 다르므로 횡이동을 허용하지 않는다.
            resetTtcBenefitConfirmation();
            resetLeftLaneSafetyConfirmation();
            return false;
        }

        if (!assessment.benefit_required)
        {
            // 유효 빈 상태이거나 TTC 위협이 없으면 기존 목표차로 주행을 유지한다.
            resetTtcBenefitConfirmation();
            return true;
        }

        if (!assessment.raw_benefit)
        {
            // TTC 이득이 사라지면 좌측 안전 유지시간도 함께 처음부터 다시 확인한다.
            resetTtcBenefitConfirmation();
            resetLeftLaneSafetyConfirmation();
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        if (!ttc_benefit_candidate_active_)
        {
            ttc_benefit_candidate_active_ = true;
            ttc_benefit_candidate_start_time_ = now;
        }

        const double benefit_duration_sec =
            std::chrono::duration<double>(
                now - ttc_benefit_candidate_start_time_)
                .count();
        if (benefit_duration_sec < kTtcBenefitHoldSec_)
        {
            // 순간적인 트랙 교체나 속도 튐으로 경로가 생성되지 않게 연속 우위를 요구한다.
            resetLeftLaneSafetyConfirmation();
            return false;
        }
        return true;
    }

    double laneChangeProfileVelocityMps(
        LaneChangeProfile profile) const
    {
        if (profile == LaneChangeProfile::COMPACT)
            return kMergeStopLaneChangeVelocityMps_;
        if (high_pass_slow_reached_)
            return kHighPassVelocityMps_;
        return current_lane_idx_ == kRightmostLaneIndex
                   ? kMergeVelocityMps_
                   : kMainVelocityMps_;
    }

    double calculateLaneChangeVelocityMps(
        LaneChangeProfile profile) const
    {
        // 현재 속도와 공통 전방 거리 제어 상한을 함께 사용해 5차 경로의 실제 추종속도를 정한다.
        const double profile_velocity =
            laneChangeProfileVelocityMps(profile);
        const double measured_velocity =
            local_.getCurCarVelocity();
        const double front_distance =
            lidar_.getRefCarFrontCarDistance();
        if (!std::isfinite(profile_velocity) ||
            !std::isfinite(measured_velocity) ||
            !std::isfinite(front_distance) ||
            profile_velocity <= 0.0)
        {
            return 0.0;
        }

        const double distance_controlled_velocity =
            control_.previewDistanceControlledVelocity(
                front_distance,
                profile_velocity);
        const double minimum_path_velocity =
            std::min(
                profile_velocity,
                kMergeStopLaneChangeVelocityMps_);
        if (!std::isfinite(distance_controlled_velocity) ||
            distance_controlled_velocity +
                    kDistanceEpsilon_ <
                minimum_path_velocity)
        {
            // 지나치게 짧고 급한 5차 경로 또는 정지 명령과의 충돌을 만들지 않는다.
            return 0.0;
        }

        // 소수 변환 오차로 upper가 lower보다 아주 작게 작아지는 경계에서는 lower로 정규화한다.
        const double clamped_upper_velocity =
            std::max(
                minimum_path_velocity,
                distance_controlled_velocity);
        return std::clamp(
            std::max(0.0, measured_velocity),
            minimum_path_velocity,
            clamped_upper_velocity);
    }

    bool isLeftLaneChangeSafe(
        LaneChangeProfile profile,
        double planned_velocity)
    {
        std::vector<Mission100LeftTrack> tracks;
        if (!lidar_.getMission100LeftTracks(tracks))
        {
            resetLeftLaneSafetyConfirmation();
            return false;
        }

        const bool compact_path =
            profile == LaneChangeProfile::COMPACT;
        const double prediction_horizon =
            compact_path
                ? kCompactLeftPredictionHorizonSec_
                : kLeftPredictionHorizonSec_;
        const double measured_ego_velocity =
            local_.getCurCarVelocity();
        if (!std::isfinite(measured_ego_velocity) ||
            !std::isfinite(planned_velocity) ||
            planned_velocity <= 0.0)
        {
            resetLeftLaneSafetyConfirmation();
            return false;
        }
        const double current_forward_velocity =
            std::max(0.0, measured_ego_velocity);
        const double ego_reference_velocity =
            std::max(
                current_forward_velocity,
                planned_velocity);

        bool safety_candidate = true;
        for (const auto &track : tracks)
        {
            if (track.hit_count < kLeftMinConfirmedHits_ ||
                track.missed_scans > kLeftMaxMissedScans_ ||
                !track.velocity_valid ||
                !std::isfinite(track.min_x) ||
                !std::isfinite(track.max_x) ||
                !std::isfinite(track.relative_velocity_x) ||
                track.min_x > track.max_x)
            {
                safety_candidate = false;
                break;
            }

            const double measured_relative_velocity =
                track.relative_velocity_x;
            if (track.min_x <= 0.0 && track.max_x >= 0.0)
            {
                // 자차와 종방향으로 나란하거나 겹치는 차량은 즉시 위험이다.
                safety_candidate = false;
                break;
            }

            if (track.min_x > 0.0)
            {
                // 전방은 현재 상태와 즉시 가속 상태 중 더 빨리 닫히는 쪽을 사용한다.
                const double accelerated_relative_velocity =
                    measured_relative_velocity -
                    std::max(
                        0.0,
                        planned_velocity - current_forward_velocity);
                const double predicted_relative_velocity =
                    std::min(
                        measured_relative_velocity,
                        accelerated_relative_velocity);
                const double current_gap = track.min_x;
                const double predicted_gap =
                    current_gap +
                    predicted_relative_velocity * prediction_horizon;
                const double required_predicted_gap =
                    kLeftHardFrontGapM_ +
                    kLeftFrontTimeHeadwaySec_ *
                        ego_reference_velocity;
                const bool closing =
                    predicted_relative_velocity <
                    -kLeftClosingSpeedEpsilonMps_;
                const double ttc =
                    closing
                        ? current_gap / (-predicted_relative_velocity)
                        : std::numeric_limits<double>::infinity();

                if (current_gap < kLeftHardFrontGapM_ ||
                    predicted_gap < required_predicted_gap ||
                    ttc < kLeftMinFrontTtcSec_)
                {
                    safety_candidate = false;
                    break;
                }
            }
            else if (track.max_x < 0.0)
            {
                // 후방은 현재 상태와 즉시 감속 상태 중 더 빨리 닫히는 쪽을 사용한다.
                const double decelerated_relative_velocity =
                    measured_relative_velocity +
                    std::max(
                        0.0,
                        current_forward_velocity - planned_velocity);
                const double predicted_relative_velocity =
                    std::max(
                        measured_relative_velocity,
                        decelerated_relative_velocity);
                const double current_gap = -track.max_x;
                const double predicted_gap =
                    current_gap -
                    predicted_relative_velocity * prediction_horizon;
                const double estimated_rear_vehicle_velocity =
                    std::max(
                        0.0,
                        current_forward_velocity +
                            measured_relative_velocity);
                const double required_predicted_gap =
                    kLeftHardRearGapM_ +
                    kLeftRearTimeHeadwaySec_ *
                        estimated_rear_vehicle_velocity;
                const bool closing =
                    predicted_relative_velocity >
                    kLeftClosingSpeedEpsilonMps_;
                const double ttc =
                    closing
                        ? current_gap / predicted_relative_velocity
                        : std::numeric_limits<double>::infinity();

                if (current_gap < kLeftHardRearGapM_ ||
                    predicted_gap < required_predicted_gap ||
                    ttc < kLeftMinRearTtcSec_)
                {
                    safety_candidate = false;
                    break;
                }
            }
            else
            {
                safety_candidate = false;
                break;
            }
        }

        if (!safety_candidate)
        {
            resetLeftLaneSafetyConfirmation();
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        if (!left_safety_candidate_active_)
        {
            left_safety_candidate_active_ = true;
            left_safety_candidate_start_time_ = now;
        }

        const double safe_duration =
            std::chrono::duration<double>(
                now - left_safety_candidate_start_time_)
                .count();
        return safe_duration >= kLeftSafeHoldSec_;
    }

    bool createLaneChangePath(double lane_change_velocity)
    {
        /*
         * 시작 경계 = 현재 실차 UTM + 현재 차로 최근접점의 접선각/곡률
         * 종료 경계 = 속도×시간 거리 앞의 왼쪽 차로 위치 + 해당 지점의 접선각/곡률
         * 두 경계를 모두 만족하는 5차 다항식 경로 뒤에 목표 차로 경로를 이어 붙인다.
         */
        if (laneIndexError() >= 0)
            return false;

        // 목표속도로 설정 시간 동안 전진하는 거리를 5차 경로의 종방향 길이로 사용한다.
        const double lane_change_forward_distance =
            lane_change_velocity * kLaneChangeDurationSec_;
        if (!std::isfinite(lane_change_velocity) ||
            !std::isfinite(lane_change_forward_distance) ||
            lane_change_velocity <= 0.0 ||
            lane_change_forward_distance <= kDistanceEpsilon_)
            return false;

        const Path &current_path = local_.getRefCurGlobalPath(); // 현재 차로 n-x
        const Path &target_path = local_.getRefNextGlobalPath(); // 바로 왼쪽 차로 n-(x+1)
        if (&current_path == &target_path)
            return false;

        const auto &current_points = current_path.getRefPosArr();
        const auto &current_yaws = current_path.getRefYawArr();
        const auto &current_curvatures = current_path.getRefKArr();
        const auto &target_points = target_path.getRefPosArr();
        const auto &target_yaws = target_path.getRefYawArr();
        const auto &target_curvatures = target_path.getRefKArr();
        if (current_points.size() < 2 ||
            current_yaws.size() != current_points.size() ||
            current_curvatures.size() != current_points.size() ||
            target_points.size() < 2 ||
            target_yaws.size() != target_points.size() ||
            target_curvatures.size() != target_points.size())
            return false;

        const std::size_t current_map_end_idx =
            mapEndIndex(current_points);
        const std::size_t target_map_end_idx =
            mapEndIndex(target_points);
        const double *car_utm = local_.getAddCurCarUTMPos();
        if (!std::isfinite(car_utm[0]) || !std::isfinite(car_utm[1]))
            return false;

        const std::size_t current_start_idx =
            static_cast<std::size_t>(std::max(0, current_path.getClosestIndex(car_utm)));
        if (current_start_idx > current_map_end_idx)
            return false;

        // 현재 차로를 따라 설정 거리만큼 앞선 지점을 목표 차로 종료점 탐색 기준으로 쓴다.
        const std::size_t current_forward_idx =
            indexAheadByDistance(current_points, current_start_idx,
                                 lane_change_forward_distance);
        if (current_forward_idx > current_map_end_idx ||
            pathDistance(current_points, current_start_idx, current_forward_idx) +
                    kDistanceEpsilon_ <
                lane_change_forward_distance)
        {
            return false;
        }

        const std::vector<double> &current_forward_point =
            current_points[current_forward_idx];
        // 전방 기준점과 가장 가까운 바로 왼쪽 차로 지점을 5차 경로의 종료점으로 사용한다.
        const std::size_t goal_idx =
            static_cast<std::size_t>(
                std::max(0, target_path.getClosestIndex(current_forward_point)));
        if (goal_idx > target_map_end_idx)
            return false;

        const std::vector<double> &goal_point = target_points[goal_idx];
        if (goal_point.size() < 2 ||
            !std::isfinite(goal_point[0]) ||
            !std::isfinite(goal_point[1]) ||
            std::hypot(
                goal_point[0] - car_utm[0],
                goal_point[1] - car_utm[1]) <= kDistanceEpsilon_)
        {
            return false;
        }

        // 짧은 저속 경로에서도 횡방향 이동량보다 미분 스케일이 작아지지 않게 한다.
        const double quintic_nominal_length =
            std::max(
                lane_change_forward_distance,
                std::hypot(
                    goal_point[0] - car_utm[0],
                    goal_point[1] - car_utm[1]));
        const QuinticPathBoundary start_boundary{
            car_utm[0],
            car_utm[1],
            current_yaws[current_start_idx],
            current_curvatures[current_start_idx]};
        const QuinticPathBoundary goal_boundary{
            goal_point[0],
            goal_point[1],
            target_yaws[goal_idx],
            target_curvatures[goal_idx]};

        try
        {
            auto quintic_result =
                generateQuinticPath(
                    start_boundary,
                    goal_boundary,
                    quintic_nominal_length,
                    kQuinticInterval_);
            auto &quintic_positions = std::get<0>(quintic_result);
            auto &quintic_yaws = std::get<1>(quintic_result);
            auto &quintic_curvatures = std::get<2>(quintic_result);
            if (quintic_positions.size() < 2 ||
                quintic_yaws.size() != quintic_positions.size() ||
                quintic_curvatures.size() != quintic_positions.size())
                return false;

            Path lane_change_path(
                "mission100_quintic_lane_change",
                quintic_positions,
                quintic_yaws,
                quintic_curvatures);
            // 기존 로컬패스 전체를 현재 위치에서 시작하는 5차 차선변경 경로로 교체한다.
            local_path_.updatePath(
                0, local_path_.getRefPosArr().size(),
                lane_change_path, 0, quintic_positions.size());
            // 공용 5차 함수가 종료점을 포함하므로 마지막 생성점이 완료 경계다.
            lane_change_boundary_local_idx_ = quintic_positions.size() - 1;
            // 종료점 중복을 피하고 그 다음 점부터 목표 차로의 남은 경로를 연결한다.
            local_path_.updatePath(
                local_path_.getRefPosArr().size(),
                local_path_.getRefPosArr().size(),
                target_path, goal_idx + 1, target_points.size());

            lane_change_end_path_pos_ =
                local_path_.getClosestPos(goal_point);
            // 아직 차로 번호를 바꾸지 않고 완료 시 반영할 왼쪽 한 칸만 예약한다.
            pending_lane_step_ = -1;
            return true;
        }
        catch (const std::exception &exception)
        {
            std::cerr << "[Mission 100] Failed to generate lane-change path: "
                      << exception.what() << "\n";
            return false;
        }
    }

    bool isLaneChangePathComplete() const
    {
        // 차선변경 끝점 또는 로컬패스가 없으면 완료로 판단할 수 없다.
        if (lane_change_end_path_pos_.size() < 2 ||
            local_path_.getRefPosArr().empty())
        {
            return false;
        }

        const double *car_utm = local_.getAddCurCarUTMPos();
        const std::vector<double> path_car_pos =
            local_path_.getClosestPos(car_utm);
        const std::vector<double> path_end_pos =
            local_path_.getClosestPos(lane_change_end_path_pos_);
        const int car_idx = local_path_.getClosestIndex(path_car_pos);
        const double end_distance =
            pointDistance(path_car_pos, path_end_pos);
        const double target_lane_lateral_distance =
            local_.getRefNextGlobalPath().getClosestDistance(car_utm);
        // 차량이 5차 경로 종료 인덱스를 통과했거나 끝점 허용 오차 안에 있어야 한다.
        const bool path_end_reached =
            car_idx >= static_cast<int>(lane_change_boundary_local_idx_) ||
            (end_distance <= kLaneChangeCompleteDistance_ &&
             car_idx >=
                 std::max(0, static_cast<int>(lane_change_boundary_local_idx_) - 1));

        // 경로상 진행 조건과 실제 목표 차로까지의 횡방향 거리 조건을 모두 요구한다.
        return path_end_reached &&
               target_lane_lateral_distance <= kLaneChangeCompleteDistance_;
    }

    void completeLaneChange()
    {
        // 완료가 확인된 뒤에만 n-x에서 n-(x+1) 맵으로 한 단계 전환한다.
        if (pending_lane_step_ != -1 || !local_.changeNextPath())
        {
            lane_map_transition_failed_ = true;
            setState(State::SAFE_STOP, "lane map transition failed");
            return;
        }

        // 맵 전환 성공 후 새 현재 글로벌패스를 기본 로컬패스로 설정한다.
        local_path_ = local_.getRefCurGlobalPath();
        // 차선변경 명령 시점이 아니라 완료 확인 시점에만 현재 차로를 한 칸 갱신한다.
        current_lane_idx_ += pending_lane_step_;
        pending_lane_step_ = 0;
        lane_change_end_path_pos_.clear();
        lane_change_boundary_local_idx_ = 0;

        resetLeftLaneSafetyConfirmation();
        resetTtcBenefitConfirmation();

        // 갱신된 현재 차로를 기준으로 남은 변경 횟수를 다시 계산한다.
        const int remaining_changes = std::abs(laneIndexError());
        std::cout << "[Mission 100] lane change complete: current_lane_idx="
                  << current_lane_idx_ << ", target_lane_idx="
                  << kTargetLaneIdx_ << ", remaining=" << remaining_changes
                  << "\n";

        if (laneIndexError() == 0)
        {
            setState(State::CRUISE, "target lane reached");
        }
        else
        {
            setState(State::WAIT_FOR_CLEAR, "checking next left lane");
        }
    }

    void updateHighPassSlowStatus()
    {
        // 감속점을 한 번 통과한 뒤에는 다시 투영하거나 판정하지 않는다.
        if (high_pass_slow_reached_ || local_path_.getRefPosArr().empty())
            return;

        // 원본 UTM을 직접 비교하지 않고 매번 현재 로컬패스의 최근접점으로 투영한다.
        high_pass_slow_path_pos_ =
            local_path_.getClosestPos(kHighPassSlowPoint_);
        if (hasReachedPathPoint(local_path_, high_pass_slow_path_pos_))
        {
            high_pass_slow_reached_ = true;
            std::cout << "[Mission 100] high-pass deceleration point reached\n";
        }
    }

    bool hasReachedPathPoint(
        const Path &path, const std::vector<double> &path_point) const
    {
        // 차량과 기준점을 같은 경로에 투영해 인덱스 순서와 거리로 통과 여부를 판단한다.
        if (path_point.size() < 2 || path.getRefPosArr().empty())
            return false;

        const double *car_utm = local_.getAddCurCarUTMPos();
        const std::vector<double> path_car_pos =
            path.getClosestPos(car_utm);
        const std::vector<double> snapped_path_point =
            path.getClosestPos(path_point);
        const int car_idx = path.getClosestIndex(path_car_pos);
        const int point_idx = path.getClosestIndex(snapped_path_point);
        const double distance =
            pointDistance(path_car_pos, snapped_path_point);

        // 기준점 인덱스를 지났거나 같은 인덱스에서 허용 거리 이내면 도달이다.
        return car_idx > point_idx ||
               (car_idx == point_idx &&
                distance <= kTriggerReachedDistance_);
    }

    double remainingMapDistance() const
    {
        // 현재 차량의 경로 투영점부터 주행 가능한 맵 끝까지 누적 거리를 계산한다.
        const Path &current_global_path = local_.getRefCurGlobalPath();
        const auto &points = current_global_path.getRefPosArr();
        if (points.size() <= kPathTailPointCount_ + 1)
            return 0.0;

        // 파일 끝의 tail 점은 실제 주행 구간에서 제외한 유효 끝점을 사용한다.
        const std::size_t map_end_idx = mapEndIndex(points);
        const double *car_utm = local_.getAddCurCarUTMPos();
        const std::vector<double> path_car_pos =
            current_global_path.getClosestPos(car_utm);
        const std::size_t car_idx =
            static_cast<std::size_t>(
                std::max(0, current_global_path.getClosestIndex(path_car_pos)));
        return pathDistance(points, std::min(car_idx, map_end_idx), map_end_idx);
    }

    bool mustStopAtMergeLaneEnd() const
    {
        // 시작 맵 n-0의 4차로에서만, 최초 한 번 유효 맵 끝 설정 거리 전에 정지한다.
        return lane_maps_valid_ &&
               current_lane_idx_ == kRightmostLaneIndex &&
               local_.getCurPathIndex() == 0 &&
               !merge_stop_satisfied_ &&
               remainingMapDistance() <= kMergeLaneStopDistance_;
    }

    void enterMergeLaneSafetyStop()
    {
        // SAFE_STOP 목표속도 0을 적용하고 정지 완료 및 재출발 조건 확인을 시작한다.
        resetLeftLaneSafetyConfirmation();
        resetTtcBenefitConfirmation();
        merge_stop_required_ = true;
        setState(State::SAFE_STOP, "merge-lane end safety stop");
    }

    void handleSafeStopState()
    {
        // 맵이 잘못됐거나 오른쪽 차선변경이 필요한 경우에는 재출발하지 않는다.
        if (!lane_maps_valid_ ||
            lane_map_transition_failed_ ||
            laneIndexError() >= 0)
            return;

        if (!merge_stop_required_ &&
            mustStopAtMergeLaneEnd())
        {
            // 일반 복구 대기 중 50m 경계에 들어와도 정지 후 축소 합류 분기로 승격한다.
            merge_stop_required_ = true;
            resetLeftLaneSafetyConfirmation();
            resetTtcBenefitConfirmation();
            std::cout
                << "[Mission 100] SAFE_STOP을 4차로 축소 합류 복구로 전환\n";
        }

        if (merge_stop_required_)
        {
            // 속도 피드백이 설정값 이하가 된 뒤에만 50m 안전정지를 완료로 인정한다.
            if (std::abs(local_.getCurCarVelocity()) <=
                kStopVelocityThresholdMps_)
            {
                merge_stop_satisfied_ = true;
            }

            const double compact_lane_change_velocity =
                calculateLaneChangeVelocityMps(
                    LaneChangeProfile::COMPACT);
            // 완전 정지와 축소 경로 시간까지 반영한 좌측 예측 안전 조건을
            // 모두 만족해야 재출발한다.
            if (!merge_stop_satisfied_ ||
                compact_lane_change_velocity <=
                    kDistanceEpsilon_ ||
                !isLeftLaneChangeSafe(
                    LaneChangeProfile::COMPACT,
                    compact_lane_change_velocity))
                return;

            // 정지 후 전용속도와 설정시간으로 계산한 5차 경로를 새로 만든다.
            if (!createLaneChangePath(
                    compact_lane_change_velocity))
            {
                // 실패하면 플래그를 유지해 일반 경로로 빠지지 않고 계속 정지한다.
                return;
            }

            // 축소 경로 생성에 성공한 경우에만 차선변경 주행을 다시 시작한다.
            merge_stop_required_ = false;
            active_lane_change_velocity_mps_ =
                compact_lane_change_velocity;
            active_lane_change_profile_velocity_mps_ =
                laneChangeProfileVelocityMps(
                    LaneChangeProfile::COMPACT);
            setState(
                State::LANE_CHANGE,
                "safety stop complete; compact lane-change path generated");
            return;
        }

        const double regular_lane_change_velocity =
            calculateLaneChangeVelocityMps(
                LaneChangeProfile::REGULAR);
        if (regular_lane_change_velocity <=
            kDistanceEpsilon_)
        {
            resetLeftLaneSafetyConfirmation();
            return;
        }

        // 일반 경로 생성 실패로 들어온 SAFE_STOP은 일반 속도 계산 경로로 복구를 시도한다.
        if (ttcBenefitAllowsLaneChange() &&
            isLeftLaneChangeSafe(
                LaneChangeProfile::REGULAR,
                regular_lane_change_velocity) &&
            createLaneChangePath(
                regular_lane_change_velocity))
        {
            active_lane_change_velocity_mps_ =
                regular_lane_change_velocity;
            active_lane_change_profile_velocity_mps_ =
                laneChangeProfileVelocityMps(
                    LaneChangeProfile::REGULAR);
            setState(State::LANE_CHANGE, "left lane became prediction-safe");
        }
    }

    bool isCurrentLocalPathNearEnd() const
    {
        // 목표 차로 주행 후 남은 로컬패스 점이 tail 개수보다 적은지 확인한다.
        const auto &points = local_path_.getRefPosArr();
        if (points.empty())
            return false;

        const double *car_utm = local_.getAddCurCarUTMPos();
        const std::vector<double> path_car_pos =
            local_path_.getClosestPos(car_utm);
        const int car_idx = local_path_.getClosestIndex(path_car_pos);
        return points.size() - static_cast<std::size_t>(std::max(0, car_idx)) <
               kPathTailPointCount_;
    }

    double commandedVelocity(double front_distance) const
    {
        // 기본 미션속도를 상한으로 사용해 슬롯 로직이 100/120km/h 제한을 넘지 않게 한다.
        const double base_velocity = baseCommandedVelocity();
        if (base_velocity <= 0.0)
            return 0.0;

        if (current_state_ == State::WAIT_FOR_CLEAR &&
            gap_velocity_override_active_)
        {
            // 슬롯 준비 중 목표는 공통 전방 거리 제어에 들어가기 전 단계의 속도 상한이다.
            return std::max(
                0.0,
                std::min(
                    base_velocity,
                    gap_command_velocity_mps_));
        }

        if (current_state_ == State::LANE_CHANGE &&
            active_lane_change_velocity_mps_ >
                kDistanceEpsilon_ &&
            active_lane_change_profile_velocity_mps_ >
                kDistanceEpsilon_)
        {
            // 같은 거리제어 비율을 다시 적용해도 경로 계획속도가 나오도록 입력 목표를 역산한다.
            const double profile_controlled_velocity =
                control_.previewDistanceControlledVelocity(
                    front_distance,
                    active_lane_change_profile_velocity_mps_);
            if (profile_controlled_velocity <=
                kDistanceEpsilon_)
            {
                // 정지 구간에서는 공통 distance_control이 최종 0을 선택하도록 프로파일 상한을 넘긴다.
                return active_lane_change_profile_velocity_mps_;
            }

            const double distance_control_ratio =
                profile_controlled_velocity /
                active_lane_change_profile_velocity_mps_;
            return std::max(
                0.0,
                std::min(
                    active_lane_change_profile_velocity_mps_,
                    active_lane_change_velocity_mps_ /
                        distance_control_ratio));
        }

        return base_velocity;
    }

    void applyCommonDistanceControl()
    {
        // 미션 100 전용 전방 ROI의 /LiDAR/car_front_car_dis를 받는다.
        last_front_distance_m_ =
            lidar_.getRefCarFrontCarDistance();
        const double mission_target_velocity =
            commandedVelocity(last_front_distance_m_);
        last_mission_target_velocity_mps_ = mission_target_velocity;
        // 미션 목표속도를 먼저 지정한 뒤 전방 거리 제어가 더 낮은 속도를 선택하게 한다.
        control_.setTargetVelocity(mission_target_velocity);
        control_.distance_control(
            last_front_distance_m_, mission_target_velocity);
    }

    void printCurrentState() const
    {
        // 100Hz 공통 출력 사이에서도 현재 상태를 놓치지 않도록 핵심 정보만 한 줄로 표시한다.
        const int lane_error = laneIndexError();
        const char *lane_direction =
            lane_error < 0 ? "좌측" :
            lane_error > 0 ? "우측" : "도착";
        std::cout
            << "[Mission 100] 현재 상태: " << stateName(current_state_)
            << " | 현재 맵: " << local_.getCurFileName()
            << " | 현재 차선: " << current_lane_idx_
            << " | 목표 차선: " << kTargetLaneIdx_
            << " | lane_index_error: " << lane_error
            << " (" << lane_direction << ")"
            << " | 남은 변경: " << std::abs(lane_error) << "회\n";
    }

    void printPeriodicStatus()
    {
        // planning 주기는 100Hz이므로 다른 출력이 묻히지 않게 1초마다 한 번만 표시한다.
        const auto now = std::chrono::steady_clock::now();
        if (status_log_started_)
        {
            const double elapsed_sec =
                std::chrono::duration<double>(
                    now - last_status_log_time_)
                    .count();
            if (elapsed_sec < kStatusLogIntervalSec_)
                return;
        }
        status_log_started_ = true;
        last_status_log_time_ = now;

        const int lane_error = laneIndexError();
        const int remaining_changes = std::abs(lane_error);
        const char *lane_direction =
            lane_error < 0 ? "좌측" :
            lane_error > 0 ? "우측" : "도착";

        // getter는 수신 상태를 변경하지 않으며, 현재 LiDAR 메시지의 유효성과
        // 안전판정에 사용되는 트랙 개수만 진단용으로 확인한다.
        std::vector<Mission100LeftTrack> tracks;
        const bool tracks_valid =
            lidar_.getMission100LeftTracks(tracks);
        const TtcBenefitAssessment ttc_assessment =
            evaluateTtcBenefit();

        double safe_hold_elapsed_sec = 0.0;
        if (left_safety_candidate_active_)
        {
            safe_hold_elapsed_sec =
                std::chrono::duration<double>(
                    now - left_safety_candidate_start_time_)
                .count();
        }
        double ttc_benefit_hold_elapsed_sec = 0.0;
        if (ttc_benefit_candidate_active_)
        {
            ttc_benefit_hold_elapsed_sec =
                std::chrono::duration<double>(
                    now - ttc_benefit_candidate_start_time_)
                .count();
        }
        double gap_mode_elapsed_sec = 0.0;
        if (gap_mode_ != GapMode::NONE)
        {
            gap_mode_elapsed_sec =
                std::chrono::duration<double>(
                    now - gap_mode_start_time_)
                    .count();
        }

        const char *left_safety_status = "판정 대기";
        if (current_state_ == State::LANE_CHANGE)
        {
            left_safety_status = "차선변경 진행 중";
        }
        else if (lane_error >= 0)
        {
            left_safety_status = "판정 불필요";
        }
        else if (!tracks_valid)
        {
            left_safety_status = "불가(LiDAR 무효/시간초과)";
        }
        else if (!left_safety_candidate_active_)
        {
            left_safety_status =
                tracks.empty()
                    ? "판정 대기"
                    : "불가(트랙/간격/TTC 조건)";
        }
        else if (safe_hold_elapsed_sec >= kLeftSafeHoldSec_)
        {
            left_safety_status = "가능";
        }
        else
        {
            left_safety_status = "연속 안전시간 확인 중";
        }

        // Planning::printStatus()가 cout의 정밀도를 바꾸므로 별도 스트림에서
        // 고속도로 미션 진단값만 읽기 쉬운 자릿수로 구성한다.
        std::ostringstream status;
        status << std::fixed << std::setprecision(2);
        status
            << "[Mission 100] 현재 로직 상태: "
            << stateName(current_state_) << " ("
            << stateDescription(current_state_) << ")\n"
            << "[Mission 100] 현재 맵: " << local_.getCurFileName()
            << " | 현재 차선: " << current_lane_idx_
            << " | 목표 차선: " << kTargetLaneIdx_
            << " | lane_index_error: " << lane_error
            << " (" << lane_direction << ")"
            << " | 남은 차선변경: " << remaining_changes << "회\n"
            << "[Mission 100] 좌측 LiDAR: "
            << (tracks_valid ? "유효" : "무효/시간초과")
            << " | 트랙: " << tracks.size() << "개"
            << " | 차선변경 안전판정: " << left_safety_status;
        if (left_safety_candidate_active_)
        {
            status << " (" << safe_hold_elapsed_sec
                   << "/" << kLeftSafeHoldSec_ << "초)";
        }
        status << "\n[Mission 100] 종방향 슬롯: ";
        status << gapModeName(gap_mode_)
               << " | 선택 ID: " << gap_anchor_track_id_
               << " | bbox X: [" << last_gap_anchor_min_x_m_
               << ", " << last_gap_anchor_max_x_m_ << "]m"
               << " | 상대속도: "
               << last_gap_anchor_relative_velocity_mps_ << "m/s"
               << " | 옆차 추정속도: "
               << last_gap_anchor_speed_mps_ << "m/s"
               << " | 간격: " << last_gap_actual_m_
               << "/" << last_gap_required_m_ << "m"
               << " | 앞 슬롯 가능: "
               << (last_gap_pass_feasible_ ? "예" : "아니오")
               << " | 슬롯 목표속도: "
               << gap_command_velocity_mps_ << "m/s"
               << " | 경과: " << gap_mode_elapsed_sec
               << "초 | 판단: " << gap_reason_;
        status << "\n[Mission 100] TTC 비교 현재차로: ";
        appendFrontTtcStatus(
            status, ttc_assessment.current_front);
        status << " | 왼쪽차로: ";
        appendFrontTtcStatus(
            status, ttc_assessment.left_front);
        status << " | 판단: ";
        if (ttc_benefit_candidate_active_ &&
            ttc_assessment.raw_benefit &&
            ttc_benefit_hold_elapsed_sec <
                kTtcBenefitHoldSec_)
        {
            status << ttcBenefitReasonDescription(
                TtcBenefitReason::BENEFIT_HOLDING);
        }
        else
        {
            status << ttcBenefitReasonDescription(
                ttc_assessment.reason);
        }
        if (ttc_assessment.benefit_required &&
            ttc_assessment.current_front.data_available &&
            ttc_assessment.left_front.data_available)
        {
            status << " | TTC 우위: ";
            if (std::isfinite(ttc_assessment.advantage_sec))
                status << ttc_assessment.advantage_sec << "초";
            else
                status << "INF";
            status << " (요구 "
                   << kLeftFrontMinTtcAdvantageSec_ << "초)";
        }
        if (ttc_benefit_candidate_active_)
        {
            status << " | 우위 유지: "
                   << ttc_benefit_hold_elapsed_sec
                   << "/" << kTtcBenefitHoldSec_ << "초";
        }
        status
            << " | 합류 안전정지: "
            << (merge_stop_required_ ? "작동" : "미작동")
            << " | 정지 확인: "
            << (merge_stop_satisfied_ ? "완료" : "미완료");
        if (lane_maps_valid_ &&
            current_lane_idx_ == kRightmostLaneIndex)
        {
            status << " | 4차로 맵 끝까지: "
                   << remainingMapDistance() << "m";
        }
        status
            << "\n[Mission 100] 미션 목표속도: "
            << last_mission_target_velocity_mps_ << "m/s ("
            << last_mission_target_velocity_mps_ * 3.6 << "km/h)"
            << " | 거리제어 후 목표속도: "
            << control_.getTargetVelocity() << "m/s ("
            << control_.getTargetVelocity() * 3.6 << "km/h)";
        if (active_lane_change_velocity_mps_ >
            kDistanceEpsilon_)
        {
            status << " | 5차 경로 계획속도: "
                   << active_lane_change_velocity_mps_
                   << "m/s"
                   << " | 차선변경 속도상한: "
                   << active_lane_change_profile_velocity_mps_
                   << "m/s";
        }
        status << " | 전방 차량 거리: ";
        if (last_front_distance_m_ < 0.0)
            status << "차량 없음(" << last_front_distance_m_ << ")";
        else
            status << last_front_distance_m_ << "m";
        status << "\n";

        std::cout << status.str();
    }

    void appendFrontTtcStatus(
        std::ostringstream &status,
        const FrontTtcEstimate &estimate) const
    {
        status << "트랙 " << estimate.track_count << "개";
        if (!estimate.stream_valid)
        {
            status << ", 무효/시간초과";
            return;
        }
        if (!estimate.front_present)
        {
            status << ", 차량 없음, TTC=INF(>"
                   << kFrontTtcDisplayCapSec_ << "초)";
            return;
        }
        if (!estimate.data_available)
        {
            status << ", 속도 확정 대기";
            return;
        }

        status << ", 간격=" << estimate.gap_m
               << "m, 상대속도="
               << estimate.relative_velocity_mps << "m/s, TTC=";
        if (std::isfinite(estimate.ttc_sec))
            status << estimate.ttc_sec << "초";
        else
            status << "INF(>" << kFrontTtcDisplayCapSec_ << "초)";
    }

    static const char *ttcBenefitReasonDescription(
        TtcBenefitReason reason)
    {
        switch (reason)
        {
        case TtcBenefitReason::MANDATORY_MERGE:
            return "4→3 필수 합류(TTC 이득 비교 제외)";
        case TtcBenefitReason::CURRENT_STREAM_UNKNOWN:
            return "현재차로 TTC 무효, 변경 대기";
        case TtcBenefitReason::CURRENT_TRACK_UNKNOWN:
            return "현재차로 속도 미확정, 변경 대기";
        case TtcBenefitReason::NO_CURRENT_FRONT:
            return "현재차로 선행차 없음, 목표차로 진행";
        case TtcBenefitReason::CURRENT_NOT_CLOSING:
            return "현재차로 선행차 비접근, 목표차로 진행";
        case TtcBenefitReason::CURRENT_TTC_COMFORTABLE:
            return "현재차로 TTC 문턱 초과, 목표차로 진행";
        case TtcBenefitReason::LEFT_STREAM_UNKNOWN:
            return "왼쪽 TTC 무효, 변경 대기";
        case TtcBenefitReason::LEFT_TRACK_UNKNOWN:
            return "왼쪽 전방 속도 미확정, 변경 대기";
        case TtcBenefitReason::INSUFFICIENT_ADVANTAGE:
            return "왼쪽 TTC 우위 부족, 변경 대기";
        case TtcBenefitReason::BENEFIT_HOLDING:
            return "왼쪽 TTC 우위 연속 확인 중";
        case TtcBenefitReason::BENEFICIAL:
            return "왼쪽 TTC 우위 확인";
        }
        return "TTC 판단 알 수 없음";
    }

    static const char *stateName(State state)
    {
        switch (state)
        {
        case State::INIT:           return "INIT";
        case State::WAIT_FOR_START: return "WAIT_FOR_START";
        case State::WAIT_FOR_CLEAR: return "WAIT_FOR_CLEAR";
        case State::LANE_CHANGE:    return "LANE_CHANGE";
        case State::CRUISE:         return "CRUISE";
        case State::SAFE_STOP:      return "SAFE_STOP";
        case State::DONE:           return "DONE";
        }
        return "UNKNOWN";
    }

    static const char *gapModeName(GapMode mode)
    {
        switch (mode)
        {
        case GapMode::NONE:         return "NONE";
        case GapMode::ASSESS:       return "ASSESS";
        case GapMode::PASS_AHEAD:   return "PASS_AHEAD";
        case GapMode::YIELD_BEHIND: return "YIELD_BEHIND";
        case GapMode::READY:        return "READY";
        case GapMode::HOLD:         return "HOLD";
        }
        return "UNKNOWN";
    }

    static const char *stateDescription(State state)
    {
        switch (state)
        {
        case State::INIT:           return "미션 초기화";
        case State::WAIT_FOR_START: return "차선변경 시작점까지 주행";
        case State::WAIT_FOR_CLEAR: return "좌측 종방향 슬롯 및 안전 확인";
        case State::LANE_CHANGE:    return "차선변경 경로 추종";
        case State::CRUISE:         return "목표 차로 직진";
        case State::SAFE_STOP:      return "안전정지";
        case State::DONE:           return "미션 완료";
        }
        return "알 수 없음";
    }

    int laneIndexError() const
    {
        // 음수면 왼쪽, 양수면 오른쪽, 0이면 목표 차로 도착이다.
        return kTargetLaneIdx_ - current_lane_idx_;
    }

    static std::size_t indexAheadByDistance(
        const std::vector<std::vector<double>> &points,
        std::size_t start_idx, double distance)
    {
        // 경로 점 간 실제 XY 거리를 누적해 지정 거리 이상 앞선 첫 인덱스를 찾는다.
        if (points.empty())
            return 0;

        start_idx = std::min(start_idx, points.size() - 1);
        double accumulated_distance = 0.0;
        for (std::size_t idx = start_idx + 1; idx < points.size(); ++idx)
        {
            accumulated_distance += pointDistance(points[idx - 1], points[idx]);
            if (accumulated_distance >= distance)
                return idx;
        }
        return points.size() - 1;
    }

    static double pathDistance(
        const std::vector<std::vector<double>> &points,
        std::size_t start_idx, std::size_t end_idx)
    {
        // 두 경로 인덱스 사이의 XY 구간 길이를 모두 더한다.
        if (points.empty() || start_idx >= end_idx)
            return 0.0;

        end_idx = std::min(end_idx, points.size() - 1);
        double distance = 0.0;
        for (std::size_t idx = start_idx + 1; idx <= end_idx; ++idx)
        {
            distance += pointDistance(points[idx - 1], points[idx]);
        }
        return distance;
    }

    std::size_t mapEndIndex(
        const std::vector<std::vector<double>> &points) const
    {
        // 맵 파일 뒤에 붙은 tail 점을 제외한 마지막 주행 가능 인덱스를 반환한다.
        if (points.size() <= kPathTailPointCount_ + 1)
            return 0;
        return points.size() - kPathTailPointCount_ - 1;
    }

    static double pointDistance(
        const std::vector<double> &first,
        const std::vector<double> &second)
    {
        // 두 UTM XY 좌표 사이의 유클리드 거리를 계산한다.
        if (first.size() < 2 || second.size() < 2)
            return std::numeric_limits<double>::infinity();
        return std::hypot(first[0] - second[0], first[1] - second[1]);
    }

    void setState(State next_state, const char *reason)
    {
        // 같은 상태로의 중복 전환 로그는 출력하지 않는다.
        if (current_state_ == next_state)
            return;
        const State previous_state = current_state_;

        if (previous_state == State::WAIT_FOR_CLEAR &&
            next_state != State::WAIT_FOR_CLEAR)
        {
            // 슬롯 속도와 선택 ID가 LANE_CHANGE·SAFE_STOP으로 새지 않게 한다.
            resetGapManeuver();
        }
        if (previous_state != State::WAIT_FOR_CLEAR &&
            next_state == State::WAIT_FOR_CLEAR)
        {
            // 새 차로의 좌측 트랙은 Perception에서 다시 초기화되므로 처음부터 평가한다.
            resetGapManeuver();
        }
        if (previous_state == State::LANE_CHANGE &&
            next_state != State::LANE_CHANGE)
        {
            active_lane_change_velocity_mps_ = 0.0;
            active_lane_change_profile_velocity_mps_ = 0.0;
        }
        current_state_ = next_state;
        std::cout << "[Mission 100] 상태 전환: "
                  << stateName(previous_state) << " -> "
                  << stateName(next_state)
                  << " | 이유: " << reason << "\n";
    }

    static constexpr double kDistanceEpsilon_ = 1e-6; // 부동소수점 거리 비교 허용 오차
    static constexpr double kStatusLogIntervalSec_ = 1.0; // 상태 요약 출력 주기(s)
};

#endif
