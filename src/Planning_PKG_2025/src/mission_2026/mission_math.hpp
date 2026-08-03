#pragma once

#include "mission_types.hpp"

#include <cstddef>
#include <vector>

namespace mission_2026
{

struct LineFitResult
{
    bool valid = false;
    double slope = 0.0;
    double intercept = 0.0;
    double angle = 0.0;
    double perpendicular_distance = 0.0;
    double rmse = 0.0;
    double x_span = 0.0;
    std::size_t inlier_count = 0;
};

struct LineFitConfig
{
    std::size_t minimum_points = 4;
    double minimum_x_span = 1.0;
    double maximum_rmse = 0.20;
    double outlier_floor = 0.08;
    double mad_scale = 2.5;
};

double clamp(double value, double minimum, double maximum);
double normalizeAngle(double angle);
double lowPass(double previous, double current, double alpha);
double lowPassAngle(double previous, double current, double alpha);

double smoothStep5(double u);
double smoothStep5First(double u);
double smoothStep5Second(double u);

/**
 * Deterministic robust fit of y = slope*x + intercept.
 *
 * A first least-squares fit is followed by a MAD-based outlier rejection and a
 * second fit. It is intentionally deterministic so replay tests produce the
 * same result every time.
 */
LineFitResult robustLineFit(const std::vector<Point2D> &points,
                            const LineFitConfig &config);

/**
 * Builds a short path in the vehicle coordinate system. The path starts at
 * (0, 0) and joins the wall-derived target line smoothly at lookahead.
 */
RelativePath buildRelativeMergePath(double lateral_correction,
                                    double heading_correction,
                                    double lookahead,
                                    double step,
                                    double maximum_lateral_correction);

/**
 * Populates yaw and curvature from XY samples. It is shared by tunnel and
 * future relative-path missions.
 */
void computePathGeometry(std::vector<PathPoint> &points);

} // namespace mission_2026

