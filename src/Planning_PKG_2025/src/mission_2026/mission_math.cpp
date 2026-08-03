#include "mission_math.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

/**
 * @file mission_math.cpp
 * @brief 2026 미션들이 공통으로 사용하는 수학 기능 구현
 *
 * 이 파일은 ROS 메시지나 기존 Planning 클래스에 의존하지 않는다.
 * 센서 입력을 단순한 Point2D와 숫자로 받아 다음 기능을 제공한다.
 *
 * 1. 터널 벽 포인트의 강건한 직선 적합
 * 2. 센서값과 각도의 저역통과 필터
 * 3. 정적 장애물 회피 경로에 사용하는 5차 smooth-step
 * 4. 터널 주행용 차량 상대경로 생성
 * 5. XY 경로로부터 yaw와 곡률 계산
 *
 * 알고리즘 코어를 ROS 입출력과 분리한 이유:
 * - rosbag 없이도 단위시험이 가능하다.
 * - 센서 토픽 이름이 변경되어도 이 파일은 수정할 필요가 없다.
 * - 같은 입력에 대해 항상 같은 결과가 나오는 회귀시험이 가능하다.
 */
namespace mission_2026
{
namespace
{

/**
 * 중앙값 계산 보조 함수.
 *
 * 평균은 한두 개의 큰 이상치에 크게 흔들리므로 벽 직선 적합에서
 * 이상치 크기를 계산할 때 중앙값을 사용한다. 전체 정렬을 하지 않고
 * nth_element를 사용하므로 평균적으로 O(N)에 가까운 비용으로 동작한다.
 */
double median(std::vector<double> values)
{
    if (values.empty())
    {
        return 0.0;
    }

    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    double result = *middle;
    if (values.size() % 2 == 0)
    {
        const auto lower = std::max_element(values.begin(), middle);
        result = 0.5 * (result + *lower);
    }
    return result;
}

/**
 * 선택된 포인트 인덱스만 사용하여 y = slope*x + intercept를 적합한다.
 *
 * 터널 벽은 차량 진행방향인 x축과 대체로 평행하다는 전제를 사용한다.
 * 완전한 수직선 적합이 필요한 환경에서는 TLS/PCA 방식으로 교체해야 한다.
 *
 * 유효성 조건:
 * - 최소 포인트 수 충족
 * - x방향 검출 길이 충족
 * - RMSE가 설정값 이하
 */
LineFitResult leastSquares(const std::vector<Point2D> &points,
                           const std::vector<std::size_t> &indices,
                           const LineFitConfig &config)
{
    LineFitResult result;
    if (indices.size() < config.minimum_points)
    {
        return result;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double minimum_x = std::numeric_limits<double>::infinity();
    double maximum_x = -std::numeric_limits<double>::infinity();

    for (const auto index : indices)
    {
        const auto &point = points[index];
        sum_x += point.x;
        sum_y += point.y;
        minimum_x = std::min(minimum_x, point.x);
        maximum_x = std::max(maximum_x, point.x);
    }

    const double count = static_cast<double>(indices.size());
    const double mean_x = sum_x / count;
    const double mean_y = sum_y / count;

    double covariance = 0.0;
    double variance_x = 0.0;
    for (const auto index : indices)
    {
        const auto &point = points[index];
        const double dx = point.x - mean_x;
        covariance += dx * (point.y - mean_y);
        variance_x += dx * dx;
    }

    if (variance_x < 1e-9)
    {
        return result;
    }

    result.slope = covariance / variance_x;
    result.intercept = mean_y - result.slope * mean_x;
    result.angle = std::atan(result.slope);
    result.perpendicular_distance =
        std::abs(result.intercept) / std::sqrt(1.0 + result.slope * result.slope);
    result.x_span = maximum_x - minimum_x;
    result.inlier_count = indices.size();

    double squared_error = 0.0;
    for (const auto index : indices)
    {
        const auto &point = points[index];
        const double residual = point.y - (result.slope * point.x + result.intercept);
        squared_error += residual * residual;
    }
    result.rmse = std::sqrt(squared_error / count);
    result.valid = result.x_span >= config.minimum_x_span &&
                   result.rmse <= config.maximum_rmse;
    return result;
}

} // namespace

double clamp(double value, double minimum, double maximum)
{
    return std::max(minimum, std::min(value, maximum));
}

double normalizeAngle(double angle)
{
    // 각도 차이를 그대로 빼면 +pi/-pi 경계에서 약 2pi가 튈 수 있다.
    // 모든 내부 각도는 [-pi, pi] 범위로 정규화한다.
    while (angle > kPi)
    {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi)
    {
        angle += 2.0 * kPi;
    }
    return angle;
}

double lowPass(double previous, double current, double alpha)
{
    // alpha가 클수록 새 측정값을 빠르게 반영한다.
    // alpha=0: 이전값 유지, alpha=1: 필터 없이 현재값 사용.
    return previous + clamp(alpha, 0.0, 1.0) * (current - previous);
}

double lowPassAngle(double previous, double current, double alpha)
{
    // 일반 숫자 필터와 달리 각도 차이를 먼저 [-pi, pi]로 정규화한다.
    // 예: +179도에서 -179도로 바뀌었을 때 -358도가 아니라 +2도로 처리.
    return normalizeAngle(previous +
                          clamp(alpha, 0.0, 1.0) *
                              normalizeAngle(current - previous));
}

double smoothStep5(double u)
{
    // p(u)=10u^3-15u^4+6u^5
    // u=0과 u=1에서 위치뿐 아니라 1·2차 미분도 0이 된다.
    // 따라서 회피 진입/복귀 연결부에서 조향과 곡률이 갑자기 튀지 않는다.
    const double t = clamp(u, 0.0, 1.0);
    return 10.0 * std::pow(t, 3) - 15.0 * std::pow(t, 4) +
           6.0 * std::pow(t, 5);
}

double smoothStep5First(double u)
{
    // smoothStep5의 u에 대한 1차 미분.
    // Frenet 경로에서는 d'(s)를 계산해 경로 yaw를 구할 때 사용한다.
    const double t = clamp(u, 0.0, 1.0);
    return 30.0 * std::pow(t, 2) - 60.0 * std::pow(t, 3) +
           30.0 * std::pow(t, 4);
}

double smoothStep5Second(double u)
{
    // smoothStep5의 u에 대한 2차 미분.
    // Frenet 경로에서는 d''(s)를 계산해 곡률을 구할 때 사용한다.
    const double t = clamp(u, 0.0, 1.0);
    return 60.0 * t - 180.0 * std::pow(t, 2) + 120.0 * std::pow(t, 3);
}

LineFitResult robustLineFit(const std::vector<Point2D> &points,
                            const LineFitConfig &config)
{
    /**
     * 처리 순서
     *
     * 1. NaN/Inf 포인트 제거
     * 2. 전체 포인트로 1차 최소제곱 적합
     * 3. 각 포인트의 절대 잔차 계산
     * 4. MAD(Median Absolute Deviation) 기반으로 이상치 경계 계산
     * 5. 인라이어만 사용해 2차 최소제곱 적합
     *
     * 무작위 RANSAC 대신 결정론적 방식을 선택한 이유는 rosbag을 반복
     * 재생할 때 매번 동일한 결과가 나와야 튜닝과 회귀시험이 쉽기 때문이다.
     */
    std::vector<Point2D> finite_points;
    finite_points.reserve(points.size());
    for (const auto &point : points)
    {
        if (isFinite(point.x) && isFinite(point.y))
        {
            finite_points.push_back(point);
        }
    }

    std::vector<std::size_t> all_indices(finite_points.size());
    std::iota(all_indices.begin(), all_indices.end(), 0);
    auto initial = leastSquares(finite_points, all_indices, config);
    if (finite_points.size() < config.minimum_points)
    {
        return initial;
    }

    std::vector<double> absolute_residuals;
    absolute_residuals.reserve(finite_points.size());
    for (const auto &point : finite_points)
    {
        absolute_residuals.push_back(
            std::abs(point.y - (initial.slope * point.x + initial.intercept)));
    }

    const double residual_median = median(absolute_residuals);
    std::vector<double> deviations;
    deviations.reserve(absolute_residuals.size());
    for (const double residual : absolute_residuals)
    {
        deviations.push_back(std::abs(residual - residual_median));
    }

    // 1.4826은 정규분포에서 MAD를 표준편차와 비슷한 크기로 변환하는 계수다.
    const double robust_sigma = 1.4826 * median(deviations);
    const double threshold = std::max(
        config.outlier_floor,
        residual_median + config.mad_scale * robust_sigma);

    std::vector<std::size_t> inlier_indices;
    inlier_indices.reserve(finite_points.size());
    for (std::size_t i = 0; i < absolute_residuals.size(); ++i)
    {
        if (absolute_residuals[i] <= threshold)
        {
            inlier_indices.push_back(i);
        }
    }

    if (inlier_indices.size() < config.minimum_points)
    {
        return initial;
    }
    return leastSquares(finite_points, inlier_indices, config);
}

RelativePath buildRelativeMergePath(double lateral_correction,
                                    double heading_correction,
                                    double lookahead,
                                    double step,
                                    double maximum_lateral_correction)
{
    /**
     * 터널 상대경로 생성 방식
     *
     * 차량 현재 위치를 (0, 0)으로 두고, lookahead 지점에서 벽으로부터
     * 계산된 목표 직선에 부드럽게 합류한다.
     *
     * 목표 직선:
     *   target_y(x) = lateral_correction + tan(heading_correction) * x
     *
     * 실제 경로:
     *   y(x) = smoothStep5(x/lookahead) * target_y(x)
     *
     * 이 구조 때문에 경로 시작점은 항상 차량 위치와 연결되며, 센서값이
     * 조금 바뀌더라도 경로 시작부가 갑자기 옆으로 이동하지 않는다.
     */
    RelativePath path;
    if (!isFinite(lateral_correction) || !isFinite(heading_correction) ||
        lookahead <= 0.0 || step <= 0.0)
    {
        path.reason = "invalid relative-path input";
        return path;
    }

    const double lateral =
        clamp(lateral_correction,
              -std::abs(maximum_lateral_correction),
              std::abs(maximum_lateral_correction));
    const double heading = clamp(heading_correction, -0.35, 0.35);

    path.points.reserve(static_cast<std::size_t>(lookahead / step) + 2);
    path.points.push_back({0.0, 0.0, 0.0, 0.0});

    for (double x = step; x <= lookahead + 1e-9; x += step)
    {
        const double u = x / lookahead;
        const double target_line_y = lateral + std::tan(heading) * x;
        const double y = smoothStep5(u) * target_line_y;
        path.points.push_back({x, y, 0.0, 0.0});
    }

    computePathGeometry(path.points);
    path.valid = path.points.size() >= 3;
    path.reason = path.valid ? "ok" : "relative path has too few points";
    return path;
}

void computePathGeometry(std::vector<PathPoint> &points)
{
    /**
     * 인접 XY 포인트에서 경로 방향과 곡률을 계산한다.
     *
     * yaw_i = atan2(y_{i+1}-y_i, x_{i+1}-x_i)
     *
     * 중앙차분 곡률 근사:
     * curvature_i = normalize(yaw_{i+1}-yaw_{i-1}) / ds
     *
     * 곡률은 이후 제어기 feed-forward 또는 속도 제한에 사용할 수 있다.
     */
    if (points.size() < 2)
    {
        return;
    }

    for (std::size_t i = 0; i + 1 < points.size(); ++i)
    {
        const double dx = points[i + 1].x - points[i].x;
        const double dy = points[i + 1].y - points[i].y;
        points[i].yaw = std::atan2(dy, dx);
    }
    points.back().yaw = points[points.size() - 2].yaw;

    if (points.size() < 3)
    {
        return;
    }

    points.front().curvature = 0.0;
    for (std::size_t i = 1; i + 1 < points.size(); ++i)
    {
        const double ds = std::hypot(
            points[i + 1].x - points[i - 1].x,
            points[i + 1].y - points[i - 1].y);
        points[i].curvature =
            ds > 1e-6
                ? normalizeAngle(points[i + 1].yaw - points[i - 1].yaw) / ds
                : 0.0;
    }
    points.back().curvature = points[points.size() - 2].curvature;
}

} // namespace mission_2026
