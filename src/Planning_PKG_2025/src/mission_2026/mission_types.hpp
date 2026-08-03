#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace mission_2026
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kRelativePathMarker = -82.82;

struct Point2D
{
    double x = 0.0;
    double y = 0.0;
};

struct Pose2D
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct PathPoint
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double curvature = 0.0;
};

struct RelativePath
{
    std::vector<PathPoint> points;
    bool valid = false;
    std::string reason;

    /**
     * Existing control code treats [-82.82, -82.82] as the marker for a
     * vehicle-relative path. This adapter keeps the core algorithm independent
     * from ROS message types while preserving that interface.
     */
    [[nodiscard]] std::vector<double> toPlanningPositionArray() const
    {
        std::vector<double> data;
        if (!valid || points.empty())
        {
            return data;
        }

        data.reserve(points.size() * 2 + 2);
        data.push_back(kRelativePathMarker);
        data.push_back(kRelativePathMarker);
        for (const auto &point : points)
        {
            data.push_back(point.x);
            data.push_back(point.y);
        }
        return data;
    }

    [[nodiscard]] std::vector<double> yawArray() const
    {
        std::vector<double> data;
        data.reserve(points.size());
        for (const auto &point : points)
        {
            data.push_back(point.yaw);
        }
        return data;
    }

    [[nodiscard]] std::vector<double> curvatureArray() const
    {
        std::vector<double> data;
        data.reserve(points.size());
        for (const auto &point : points)
        {
            data.push_back(point.curvature);
        }
        return data;
    }
};

struct FrenetPoint
{
    double s = 0.0;
    double d = 0.0;
    double yaw_error = 0.0;
    double curvature = 0.0;
};

enum class TrafficSignal
{
    kUnknown,
    kRed,
    kYellow,
    kGreen
};

struct TimedTrafficSignal
{
    TrafficSignal state = TrafficSignal::kUnknown;
    double stamp = -std::numeric_limits<double>::infinity();
    double confidence = 0.0;
};

struct RoadBounds
{
    // Positive d is the left side of the reference path.
    double left_d = 3.0;
    double right_d = -3.0;
    bool valid = false;
};

struct StopLine
{
    double s = 0.0;
    bool valid = false;
};

struct ObstacleObservation
{
    int id = -1;
    double s = 0.0;
    double d = 0.0;
    double length = 1.0;
    double width = 1.0;
    double confidence = 1.0;
    double stamp = -std::numeric_limits<double>::infinity();
};

inline bool isFinite(double value)
{
    return std::isfinite(value);
}

} // namespace mission_2026

