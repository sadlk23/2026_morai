#pragma once

#include "mission_math.hpp"
#include "mission_types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace mission_2026
{

enum class TunnelState
{
    kIdle,
    kCalibrating,
    kWallTracking,
    kGpsRecovery,
    kDone
};

enum class TunnelTrackingMode
{
    kNone,
    kDualWall,
    kLeftWall,
    kRightWall,
    kPredicted,
    kDeadReckoning,
    kSafeStop
};

struct GpsSample
{
    bool valid = false;
    Pose2D pose;
    double stamp = -std::numeric_limits<double>::infinity();
};

struct TunnelInput
{
    double now = 0.0;
    bool mission_active = false;

    GpsSample gps;
    double vehicle_speed_mps = 0.0;
    double imu_yaw_rate_rps = 0.0;

    // Points must be expressed in the vehicle/LiDAR coordinate system:
    // x forward, y left.
    std::vector<Point2D> left_wall_points;
    std::vector<Point2D> right_wall_points;

    bool camera_lateral_valid = false;
    double camera_lateral_error = 0.0;

    // May be connected to /LiDAR/tunnel_end or another independent exit cue.
    bool exit_hint = false;
};

struct TunnelConfig
{
    // Wall fitting
    LineFitConfig wall_fit;
    double near_start_m = 2.0;
    double near_end_m = 8.0;
    double far_start_m = 8.0;
    double far_end_m = 25.0;
    double maximum_near_far_angle_rad = 5.0 * kPi / 180.0;
    double maximum_dual_wall_angle_difference_rad = 5.0 * kPi / 180.0;

    // Entry calibration and fallback defaults
    double calibration_duration_s = 1.0;
    double maximum_bias_sample_yaw_rate_rps = 0.08;
    double default_left_wall_distance_m = 3.0;
    double default_right_wall_distance_m = 3.0;

    // Freshness and recovery
    double gps_timeout_s = 0.35;
    double wall_hold_duration_s = 0.30;
    double dead_reckoning_limit_s = 1.00;
    int gps_recovery_required_samples = 5;
    double gps_recovery_maximum_innovation_m = 8.0;

    // Tunnel geometry
    double known_tunnel_length_m = 100.0;
    double exit_search_margin_m = 15.0;

    // Path and speed
    double path_lookahead_m = 12.0;
    double path_step_m = 0.5;
    double maximum_lateral_correction_m = 1.5;
    double normal_speed_mps = 4.0;
    double predicted_speed_mps = 3.0;
    double dead_reckoning_speed_mps = 1.5;
    double safe_stop_speed_mps = 0.0;

    // Filtering and camera cross-check
    double wall_filter_alpha = 0.35;
    double camera_correction_weight = 0.10;
    double maximum_camera_wall_disagreement_m = 0.60;
};

struct WallEstimate
{
    bool valid = false;
    LineFitResult full;
    LineFitResult near;
    LineFitResult far;
    double quality = 0.0;
};

struct TunnelOutput
{
    bool override_normal_planning = false;
    bool request_absolute_path = false;
    double target_speed_mps = 0.0;
    double tunnel_progress_m = 0.0;
    double lateral_correction_m = 0.0;
    double heading_correction_rad = 0.0;
    RelativePath relative_path;
    TunnelState state = TunnelState::kIdle;
    TunnelTrackingMode tracking_mode = TunnelTrackingMode::kNone;
    WallEstimate left_wall;
    WallEstimate right_wall;
    std::string diagnostic;
};

/**
 * GPS-shadow tunnel mission core.
 *
 * The class has no ROS dependency. A thin adapter can translate ROS topics into
 * TunnelInput and publish TunnelOutput. This keeps the estimation and fallback
 * logic unit-testable and easy to develop independently.
 */
class GpsShadowTunnel
{
public:
    explicit GpsShadowTunnel(TunnelConfig config = {});

    TunnelOutput update(const TunnelInput &input);
    void reset();

    [[nodiscard]] TunnelState state() const;
    [[nodiscard]] TunnelTrackingMode trackingMode() const;

private:
    TunnelConfig config_;
    TunnelState state_ = TunnelState::kIdle;
    TunnelTrackingMode tracking_mode_ = TunnelTrackingMode::kNone;

    double mission_start_time_ = 0.0;
    double last_update_time_ = 0.0;
    double last_valid_wall_time_ = -std::numeric_limits<double>::infinity();
    double tunnel_progress_m_ = 0.0;
    double estimated_heading_rad_ = 0.0;
    double imu_bias_rps_ = 0.0;
    double imu_bias_sum_ = 0.0;
    std::size_t imu_bias_count_ = 0;

    Pose2D entry_pose_;
    bool entry_pose_valid_ = false;
    double target_left_distance_m_ = 0.0;
    double target_right_distance_m_ = 0.0;
    double left_distance_sum_ = 0.0;
    double right_distance_sum_ = 0.0;
    std::size_t left_distance_count_ = 0;
    std::size_t right_distance_count_ = 0;

    double filtered_lateral_correction_m_ = 0.0;
    double filtered_heading_correction_rad_ = 0.0;
    bool have_filtered_wall_state_ = false;
    int gps_recovery_count_ = 0;

    void beginMission(const TunnelInput &input);
    void updateDeadReckoning(const TunnelInput &input, double dt);
    void collectCalibration(const TunnelInput &input,
                            const WallEstimate &left,
                            const WallEstimate &right);
    void finishCalibration();

    [[nodiscard]] WallEstimate estimateWall(
        const std::vector<Point2D> &points,
        bool expect_left_side) const;
    [[nodiscard]] bool gpsFresh(const TunnelInput &input) const;
    [[nodiscard]] bool exitRegionReached(const TunnelInput &input) const;
    [[nodiscard]] double gpsInnovation(const TunnelInput &input) const;

    TunnelOutput buildTrackingOutput(const TunnelInput &input,
                                     const WallEstimate &left,
                                     const WallEstimate &right,
                                     double dt);
    TunnelOutput buildRecoveryOutput(const TunnelInput &input,
                                     const WallEstimate &left,
                                     const WallEstimate &right,
                                     double dt);
};

const char *toString(TunnelState state);
const char *toString(TunnelTrackingMode mode);

} // namespace mission_2026

