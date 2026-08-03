/***
 * 작성자: 조윤서
 * 설명: 기타 잡다한 응용 함수들
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef UTIL_FUNC_H_
#define UTIL_FUNC_H_

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include "local.hpp"
#include "cubic_spline.hpp"
#include "polynomial.hpp"
#include <random>

// 스플라인 패스 반환
// TODO : 나중에 Path 객체 안에 Path 객체 생성자로 넣기, index 유효성 검사 추가하기
inline std::tuple<std::vector<std::vector<double>>, std::vector<double>, std::vector<double>> generateSplinePath(const std::vector<std::vector<double>> &path_raw, double ds = 0.1)
{
    std::vector<double> x, y;
    std::vector<double> last_point = {-1, -1};
    for (const auto &point : path_raw)
    {
        if (last_point == point)
            continue;
        x.push_back(point[0]);
        y.push_back(point[1]);
        last_point = point;
    }

    CubicSpline2D csp(x, y);
    std::vector<double> s;
    for (double i = 0; i < csp.calc_s(x, y).back(); i += ds)
    {
        s.push_back(i);
    }

    std::vector<std::vector<double>> path;
    std::vector<double> ryaw, rk;
    for (const auto &i_s : s)
    {
        auto pos = csp.calc_position(i_s);
        path.push_back({pos.first, pos.second});
        ryaw.push_back(csp.calc_yaw(i_s));
        rk.push_back(csp.calc_curvature(i_s));
    }

    return std::make_tuple(path, ryaw, rk);
}

// 5차 다항식 경로의 시작/종료 경계 조건
// tangent_yaw는 차체 자세가 아니라 경로 진행방향의 접선각이며 curvature의 단위는 1/m이다.
struct QuinticPathBoundary
{
    double x;
    double y;
    double tangent_yaw;
    double curvature;
};

// 시작/종료 위치, 접선각, 곡률을 만족하는 5차 다항식 경로 반환
// 반환 형식은 generateSplinePath()와 동일하므로 다른 미션에서도 같은 방식으로 Path를 만들 수 있다.
inline std::tuple<std::vector<std::vector<double>>, std::vector<double>, std::vector<double>>
generateQuinticPath(
    const QuinticPathBoundary &start,
    const QuinticPathBoundary &goal,
    double nominal_length,
    double ds = 0.1)
{
    const auto boundary_is_finite = [](const QuinticPathBoundary &boundary)
    {
        return std::isfinite(boundary.x) &&
               std::isfinite(boundary.y) &&
               std::isfinite(boundary.tangent_yaw) &&
               std::isfinite(boundary.curvature);
    };

    if (!boundary_is_finite(start) || !boundary_is_finite(goal))
        throw std::invalid_argument("5차 다항식 경계 조건은 유한한 값이어야 합니다.");
    if (!std::isfinite(nominal_length) || nominal_length <= 0.0)
        throw std::invalid_argument("5차 다항식 nominal_length는 0보다 커야 합니다.");
    if (!std::isfinite(ds) || ds <= 0.0)
        throw std::invalid_argument("5차 다항식 ds는 0보다 커야 합니다.");

    constexpr std::size_t kMinimumDenseSegments = 200;
    constexpr std::size_t kDenseSamplesPerOutputSegment = 10;
    constexpr std::size_t kMaximumDenseSegments = 1000000;
    constexpr std::size_t kMaximumOutputSegments = 1000000;
    constexpr double kDerivativeNormSquaredEpsilon = 1.0e-12;
    constexpr double kLengthEpsilon = 1.0e-9;

    const double nominal_segment_count = std::ceil(nominal_length / ds);
    const double maximum_nominal_segment_count =
        static_cast<double>(kMaximumDenseSegments / kDenseSamplesPerOutputSegment);
    if (!std::isfinite(nominal_segment_count) ||
        nominal_segment_count > maximum_nominal_segment_count)
    {
        throw std::length_error("5차 다항식 경로의 요청 샘플 수가 너무 많습니다.");
    }

    const std::size_t dense_segment_count = std::max(
        kMinimumDenseSegments,
        static_cast<std::size_t>(nominal_segment_count) *
            kDenseSamplesPerOutputSegment);

    // 큰 UTM 절대좌표로 계수를 풀 때 생기는 수치오차를 줄이기 위해
    // 시작점을 원점으로 옮기고 정규화 매개변수 u=[0, 1]에서 계산한다.
    const double length_squared = nominal_length * nominal_length;
    const double delta_x = goal.x - start.x;
    const double delta_y = goal.y - start.y;

    Polynomial x_polynomial(
        0.0,
        nominal_length * std::cos(start.tangent_yaw),
        -length_squared * start.curvature * std::sin(start.tangent_yaw),
        delta_x,
        nominal_length * std::cos(goal.tangent_yaw),
        -length_squared * goal.curvature * std::sin(goal.tangent_yaw),
        1.0);
    Polynomial y_polynomial(
        0.0,
        nominal_length * std::sin(start.tangent_yaw),
        length_squared * start.curvature * std::cos(start.tangent_yaw),
        delta_y,
        nominal_length * std::sin(goal.tangent_yaw),
        length_squared * goal.curvature * std::cos(goal.tangent_yaw),
        1.0);

    struct QuinticSample
    {
        double x;
        double y;
        double yaw;
        double curvature;
    };

    const auto evaluate = [&](double u)
    {
        const double x = start.x + x_polynomial.position(u);
        const double y = start.y + y_polynomial.position(u);
        const double dx = x_polynomial.velocity(u);
        const double dy = y_polynomial.velocity(u);
        const double ddx = x_polynomial.acceleration(u);
        const double ddy = y_polynomial.acceleration(u);
        const double derivative_norm_squared = dx * dx + dy * dy;

        if (!std::isfinite(x) || !std::isfinite(y) ||
            !std::isfinite(dx) || !std::isfinite(dy) ||
            !std::isfinite(ddx) || !std::isfinite(ddy))
        {
            throw std::runtime_error("5차 다항식 계산 중 유한하지 않은 값이 생성되었습니다.");
        }
        if (derivative_norm_squared <= kDerivativeNormSquaredEpsilon)
            throw std::runtime_error("5차 다항식 경로에 진행방향을 정할 수 없는 점이 있습니다.");

        const double yaw = std::atan2(dy, dx);
        const double curvature =
            (dx * ddy - dy * ddx) /
            std::pow(derivative_norm_squared, 1.5);
        if (!std::isfinite(yaw) || !std::isfinite(curvature))
            throw std::runtime_error("5차 다항식의 yaw 또는 곡률 계산에 실패했습니다.");

        return QuinticSample{x, y, yaw, curvature};
    };

    // generateSplinePath()의 ds와 같은 거리 간격을 제공하기 위해
    // 먼저 조밀하게 계산한 뒤 누적 호 길이를 기준으로 다시 샘플링한다.
    std::vector<double> cumulative_length(dense_segment_count + 1, 0.0);
    QuinticSample previous_sample = evaluate(0.0);
    for (std::size_t i = 1; i <= dense_segment_count; ++i)
    {
        const double u =
            static_cast<double>(i) / static_cast<double>(dense_segment_count);
        const QuinticSample current_sample = evaluate(u);
        cumulative_length[i] = cumulative_length[i - 1] +
                               std::hypot(current_sample.x - previous_sample.x,
                                          current_sample.y - previous_sample.y);
        previous_sample = current_sample;
    }

    const double path_length = cumulative_length.back();
    if (!std::isfinite(path_length) || path_length <= kLengthEpsilon)
        throw std::runtime_error("5차 다항식 경로 길이가 0이거나 유효하지 않습니다.");

    const double output_segment_count_double = std::ceil(path_length / ds);
    if (!std::isfinite(output_segment_count_double) ||
        output_segment_count_double > static_cast<double>(kMaximumOutputSegments))
    {
        throw std::length_error("5차 다항식 경로의 출력 샘플 수가 너무 많습니다.");
    }
    const std::size_t output_segment_count =
        static_cast<std::size_t>(output_segment_count_double);

    std::vector<std::vector<double>> path;
    std::vector<double> yaw;
    std::vector<double> curvature;
    path.reserve(output_segment_count + 1);
    yaw.reserve(output_segment_count + 1);
    curvature.reserve(output_segment_count + 1);

    std::size_t dense_upper_index = 1;
    for (std::size_t i = 0; i <= output_segment_count; ++i)
    {
        const double target_length =
            (i == output_segment_count)
                ? path_length
                : std::min(static_cast<double>(i) * ds, path_length);
        while (dense_upper_index < dense_segment_count &&
               cumulative_length[dense_upper_index] < target_length)
        {
            ++dense_upper_index;
        }

        const std::size_t dense_lower_index = dense_upper_index - 1;
        const double segment_length =
            cumulative_length[dense_upper_index] -
            cumulative_length[dense_lower_index];
        const double interpolation_ratio =
            (segment_length > kLengthEpsilon)
                ? (target_length - cumulative_length[dense_lower_index]) /
                      segment_length
                : 0.0;
        const double u =
            (i == output_segment_count)
                ? 1.0
                : (static_cast<double>(dense_lower_index) +
                   interpolation_ratio) /
                      static_cast<double>(dense_segment_count);
        const QuinticSample sample = evaluate(u);
        path.push_back({sample.x, sample.y});
        yaw.push_back(sample.yaw);
        curvature.push_back(sample.curvature);
    }

    return std::make_tuple(path, yaw, curvature);
}

/* 벡터 슬라이싱 */
// 파이썬의 list 슬라이싱과 기능 동일 (idx m부터 idx n-1까지)
template <typename T>
void vectorSlice(T &ans, T const &v, int m, int n)
{
    ans.clear();
    // 유효한 인덱스인지 체크 - 런타임 디버깅용
    // m = std::max(0, m);
    // n = std::min(static_cast<int>(v.size()), n);
    ans.insert(ans.end(), v.begin() + m, v.begin() + n);
}

/* Python NumPy vstack method 구현*/
template <typename T>
void vstackCPP(T &vstack, T &vec01, T &vec02, bool clear = true)
{
    if (clear)
    {
        vstack.clear();
    }
    vstack.reserve(vec01.size() + vec02.size());
    vstack.insert(vstack.end(), vec01.begin(), vec01.end());
    vstack.insert(vstack.end(), vec02.begin(), vec02.end());
    return;
}

/* Python NumPy flatten method 구현*/
template <typename T>
void flatten(T &result, std::vector<T> &vec2D)
{
    result.clear();
    for (const auto &inner_vec : vec2D)
    {
        result.insert(result.end(), inner_vec.begin(), inner_vec.end());
    }
}

/* 1차원 벡터 2차원으로 바꿔주는 벡터*/
template <typename T>
void reshape1Dto2D(std::vector<T> &result, T &vec1D)
{
    result.clear();
    int vsize = vec1D.size();
    if (vsize & 1)
    {
        vec1D.emplace_back(vec1D[0] - vec1D[0]);
    } // 홀수일 경우 강제적으로 짝수로 만들어줌.
      //  0이 아니라 첫번째 인덱스에서 첫번째 인덱스를 뺀 값을 넣는 이유는 vector 자료형 일치 때문.
    for (int i = 0; i < (vsize / 2); i++)
    {
        T pair = {vec1D[2 * i], vec1D[2 * i + 1]};
        result.emplace_back(pair);
    }
}

/* 2차원 벡터 출력~ */
template <typename T>
void print2Dvec(T &vec2D, std::string head_msg = " ")
{
    std::cout << head_msg << "/n";
    for (auto vec1D = vec2D.begin(); vec1D != vec2D.end(); vec1D++)
    {
        for (int idx = 0; idx != (*vec1D).size(); idx++)
        {
            std::cout << (*vec1D)[idx] << " ";
        }
        std::cout << "\n";
    }
}

inline double outerProduct(const std::vector<double> &obj_edge_point, const std::vector<double> &closest_global_point, const std::vector<double> &offset)
{
    // Calculate the components of the vectors
    double v1x = obj_edge_point[0] - closest_global_point[0];
    double v1y = obj_edge_point[1] - closest_global_point[1];

    double v2x = offset[0] - closest_global_point[0];
    double v2y = offset[1] - closest_global_point[1];

    // Calculate the cross product
    double cross_product = (v1x * v2y) - (v1y * v2x);

    return cross_product;
}

/* 난수 생성기 */
inline double randomNumCreator(double min, double max)
{
    std::random_device rd;  // 시드에 사용할 장치
    std::mt19937 gen(rd()); // Mersenne Twister 엔진 초기화
    std::uniform_real_distribution<double> rNum(min, max);
    return rNum(gen);
}

#endif
