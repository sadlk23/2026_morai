#pragma once

#include "mission_math.hpp"
#include "mission_types.hpp"

#include <limits>
#include <string>
#include <vector>

namespace mission_2026
{

enum class StaticTrafficState
{
    kIdle,
    kApproach,
    kStopForSignal,
    kAvoidance,
    kReturn,
    kDone,
    kSafeStop
};

struct StaticTrafficInput
{
    double now = 0.0;
    bool mission_active = false;

    double current_s = 0.0;
    double current_d = 0.0;
    double vehicle_speed_mps = 0.0;

    RoadBounds road_bounds;
    StopLine stop_line;
    TimedTrafficSignal traffic_signal;
    std::vector<ObstacleObservation> obstacles;
};

struct StaticTrafficConfig
{
    // Vehicle and safety envelope
    double vehicle_width_m = 1.90;
    double vehicle_length_m = 4.65;
    double lateral_control_error_m = 0.20;
    double perception_lateral_error_m = 0.15;
    double lateral_safety_margin_m = 0.20;
    double longitudinal_safety_margin_m = 0.80;
    double boundary_margin_m = 0.15;

    // Perception and tracking
    double detection_range_m = 25.0;
    double obstacle_timeout_s = 0.35;
    int required_obstacle_confirmations = 2;
    double minimum_obstacle_confidence = 0.50;
    double association_distance_m = 1.50;

    // Signal handling
    double signal_timeout_s = 0.35;
    int required_green_confirmations = 4;
    double unknown_signal_stop_distance_m = 15.0;
    double stop_line_buffer_m = 1.0;

    // Path candidates
    double path_step_m = 0.25;
    double lateral_candidate_step_m = 0.25;
    double default_left_bound_m = 3.0;
    double default_right_bound_m = -3.0;
    double minimum_entry_length_m = 8.0;
    double obstacle_entry_buffer_m = 1.0;
    std::vector<double> early_merge_distances_m = {0.0, 2.0, 4.0};
    std::vector<double> return_lengths_m = {8.0, 12.0};

    // Kinematic limits
    double maximum_curvature_per_m = 0.16;
    double maximum_lateral_acceleration_mps2 = 2.0;
    double comfortable_deceleration_mps2 = 2.5;
    double emergency_stop_distance_m = 5.0;

    // Speed policy
    double approach_speed_mps = 5.0;
    double avoidance_speed_mps = 4.0;
    double return_speed_mps = 4.5;

    // Candidate cost
    double lateral_offset_cost = 2.0;
    double curvature_cost = 20.0;
    double path_length_cost = 0.05;
    double side_change_cost = 3.0;
};

struct StaticTrafficOutput
{
    bool override_normal_planning = false;
    double target_speed_mps = 0.0;
    StaticTrafficState state = StaticTrafficState::kIdle;
    TrafficSignal effective_signal = TrafficSignal::kUnknown;
    std::vector<FrenetPoint> frenet_path;
    int selected_obstacle_id = -1;
    int selected_side = 0; // +1 left, -1 right, 0 none
    double candidate_cost = std::numeric_limits<double>::infinity();
    std::string diagnostic;
};

/**
 * Static obstacle and traffic-light mission core.
 *
 * Inputs are expressed in Frenet coordinates so the same planner can be used
 * with normal UTM localization and with a tunnel-relative s/d frame.
 */
class StaticTrafficMission
{
public:
    explicit StaticTrafficMission(StaticTrafficConfig config = {});

    StaticTrafficOutput update(const StaticTrafficInput &input);
    void reset();

    [[nodiscard]] StaticTrafficState state() const;

private:
    struct TrackedObstacle
    {
        ObstacleObservation observation;
        int confirmations = 0;
        double last_seen = 0.0;
        bool seen_this_cycle = false;
    };

    struct ExpandedObstacle
    {
        int id = -1;
        double minimum_s = 0.0;
        double maximum_s = 0.0;
        double center_d = 0.0;
        double lateral_half_width = 0.0;
    };

    struct Candidate
    {
        bool valid = false;
        std::vector<FrenetPoint> path;
        double target_d = 0.0;
        double cost = std::numeric_limits<double>::infinity();
        double maximum_curvature = 0.0;
        int side = 0;
        std::string rejection_reason;
    };

    StaticTrafficConfig config_;
    StaticTrafficState state_ = StaticTrafficState::kIdle;
    std::vector<TrackedObstacle> tracked_obstacles_;
    int next_generated_id_ = 1000000;
    int selected_side_ = 0;
    int selected_obstacle_id_ = -1;
    int green_confirmation_count_ = 0;
    double last_processed_signal_stamp_ =
        -std::numeric_limits<double>::infinity();

    void updateTracks(const StaticTrafficInput &input);
    int findAssociatedTrack(const ObstacleObservation &observation) const;
    [[nodiscard]] std::vector<ObstacleObservation> confirmedObstacles(
        const StaticTrafficInput &input) const;
    [[nodiscard]] TrafficSignal effectiveSignal(
        const StaticTrafficInput &input);
    [[nodiscard]] ExpandedObstacle expandObstacle(
        const ObstacleObservation &obstacle) const;
    [[nodiscard]] bool obstacleThreatensReference(
        const ExpandedObstacle &obstacle,
        const StaticTrafficInput &input) const;
    [[nodiscard]] bool signalRequiresStop(
        const StaticTrafficInput &input,
        TrafficSignal signal) const;
    [[nodiscard]] double stoppingTargetSpeed(
        const StaticTrafficInput &input) const;

    [[nodiscard]] std::vector<double> lateralTargets(
        const StaticTrafficInput &input) const;
    [[nodiscard]] Candidate generateCandidate(
        const StaticTrafficInput &input,
        const ExpandedObstacle &primary_obstacle,
        const std::vector<ExpandedObstacle> &all_obstacles,
        double target_d,
        double early_merge_distance,
        double return_length,
        bool stop_required) const;
    [[nodiscard]] std::vector<FrenetPoint> buildNominalPath(
        const StaticTrafficInput &input,
        double end_s) const;

    [[nodiscard]] bool pointWithinRoad(
        double d,
        const StaticTrafficInput &input) const;
    [[nodiscard]] bool pathCollides(
        const std::vector<FrenetPoint> &path,
        const std::vector<ExpandedObstacle> &obstacles) const;
};

const char *toString(StaticTrafficState state);
const char *toString(TrafficSignal signal);

} // namespace mission_2026
