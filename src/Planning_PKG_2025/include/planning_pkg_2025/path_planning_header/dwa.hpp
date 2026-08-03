/***
 * 작성자: 정성윤
 * 설명: DWA를 사용하여 경로를 생성하는 클래스
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef INCLUDE_DYNAMICWINDOWAPPROACH_HPP_
#define INCLUDE_DYNAMICWINDOWAPPROACH_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "local.hpp"
#include "lidar.hpp"
#include "control.hpp"
#include "path.hpp"
#include "median_filter.hpp"
#include "utility_function.hpp"

class DynamicWindowApproach
{
private:
    Local &local_;
    Lidar &lidar_;
    Control &control_;
    Path &local_path_;

    MedianFilter yaw_filter_;

    double pi_ = 3.14159; // 파이 값

    const int dwa_state_size_ = 5; // 상태 벡터 크기

    // car 파라미터
    const double kCarRadius_; // 로봇 반경

    // 하드웨어 속도 최대 최소
    const double kMaxLinearVelocity_;  // 최대 선속도
    const double kMinLinearVelocity_;  // 최소 선속도
    const double kMaxAngularVelocity_; // 최대 각속도

    const double kMaxLinearAcceleration_;  // 최대 선가속도
    const double kMaxAngularAcceleration_; // 최대 각가속도

    // dynamic window 파라미터
    const double kTimeResolution_;            // 시간 해상도
    const double kWindowTime_;                // 동적 창의 시간 범위
    const double kLinearVelocityResolution_;  // 선속도 해상도
    const double kAngularVelocityResolution_; // 각속도 해상도

    // cost 함수 파라미터
    const double kGoalCostWeight_;       // 목표 비용 계수
    const double kVelocityCostWeight_;   // 전역 경로 비용 계수
    const double kObstacleCostWeight_;   // 장애물 비용 계수
    const double kSmoothnessCostWeight_; // 부드러움 비용 계수

    Eigen::VectorXd dwa_current_state_;          // 현재 상태 벡터 (x, y, theta(yaw), 선속도, 각속도)
    std::vector<std::vector<double>> obstacles_; // 장애물 좌표 목록
    std::vector<double> current_window_;         // 동적 창에서의 현재 속도 및 각속도 범위
    Eigen::Vector2d goal_;                       // 목표 위치 (x, y)

    std::vector<std::vector<std::vector<double>>> all_paths_vectors_;

public:
    // 생성자: JSON으로부터 파라미터를 받아와 초기화
    DynamicWindowApproach(Local &local, Lidar &lidar, Control &control, Path &local_path, nlohmann::json &param)
        : local_(local), lidar_(lidar), control_(control), local_path_(local_path),
          kCarRadius_(param["car_radius"]),
          kMaxLinearVelocity_(param["max_linear_velocity"]),
          kMinLinearVelocity_(param["min_linear_velocity"]),
          kMaxAngularVelocity_(param["max_angular_velocity"]),
          kMaxLinearAcceleration_(param["max_linear_acceleration"]),
          kMaxAngularAcceleration_(param["max_angular_acceleration"]),
          kTimeResolution_(param["dt"]),
          kWindowTime_(param["window_time"]),
          kLinearVelocityResolution_(param["linear_velocity_resolution"]),
          kAngularVelocityResolution_(param["angular_velocity_resolution"]),
          kGoalCostWeight_(param["goal_cost_weight"]),
          kVelocityCostWeight_(param["velocity_cost_weight"]),
          kObstacleCostWeight_(param["obstacle_cost_weight"]),
          kSmoothnessCostWeight_(param["smoothness_cost_weight"]),
          yaw_filter_(param["yaw_filter_param"])
    {
        dwa_current_state_.resize(dwa_state_size_);
        dwa_current_state_.setZero();
        goal_.resize(2);
        goal_.setZero();
    }

    // 소멸자
    ~DynamicWindowApproach() {}

    // 제어 입력을 이용해 다음 상태를 예측하는 함수
    void predictState(Eigen::Vector2d &control, Eigen::VectorXd &state)
    {
        state(2) += control(1) * kTimeResolution_;                      // 각도(세타) 업데이트
        state(0) += control(0) * std::cos(state(2)) * kTimeResolution_; // x 좌표 업데이트
        state(1) += control(0) * std::sin(state(2)) * kTimeResolution_; // y 좌표 업데이트
        state(3) = control(0);                                          // 선속도 업데이트
        state(4) = control(1);                                          // 각속도 업데이트
    }

    // 동적 창 계산 함수: 가능한 선속도와 각속도 범위를 결정
    void updateDynamicWindow()
    {
        current_window_.clear();
        double cur_linear_velocity = dwa_current_state_(3);  // 현재 선속도
        double cur_angular_velocity = dwa_current_state_(4); // 현재 각속도

        // 최대 선가속도와 각가속도를 이용하여 가능한 속도 및 각속도 범위 계산
        double linear_velocity_increment = kMaxLinearAcceleration_ * kTimeResolution_;
        double angular_velocity_increment = kMaxAngularAcceleration_ * kTimeResolution_;

        // 가능한 선속도 범위
        current_window_.push_back(std::max(cur_linear_velocity - linear_velocity_increment, kMinLinearVelocity_));
        current_window_.push_back(std::min(cur_linear_velocity + linear_velocity_increment, kMaxLinearVelocity_));

        // 가능한 각속도 범위
        current_window_.push_back(std::max(cur_angular_velocity - angular_velocity_increment, -kMaxAngularVelocity_));
        current_window_.push_back(std::min(cur_angular_velocity + angular_velocity_increment, kMaxAngularVelocity_));
    }

    // 주어진 제어 입력으로 경로를 예측하는 함수
    std::vector<Eigen::VectorXd> generateTrajectory(Eigen::Vector2d &dwa_control)
    {
        std::vector<Eigen::VectorXd> trajectory;
        double time = 0;
        Eigen::VectorXd state = dwa_current_state_;
        trajectory.push_back(state);
        while (time <= kWindowTime_)
        {
            predictState(dwa_control, state); // 상태 예측
            trajectory.push_back(state);      // 경로에 상태 추가

            time = time + kTimeResolution_;
        }
        return trajectory;
    }

    // 동적 창에서 모든 경로를 예측하고 최적 경로를 선택하는 함수
    std::vector<Eigen::VectorXd> getOptimalTrajectory()
    {
        all_paths_vectors_.clear();
        double cost = 0;
        double min_cost = INT_MAX; // 최소 비용 초기화
        std::vector<Eigen::VectorXd> best_trajectory;
        // 동적 창에서 가능한 모든 속도 및 각속도 조합에 대한 경로 계산
        for (double linear_v = current_window_[0]; linear_v <= current_window_[1]; linear_v = linear_v + kLinearVelocityResolution_)
        {
            for (double angular_v = current_window_[2]; angular_v <= current_window_[3]; angular_v = angular_v + kAngularVelocityResolution_)
            {
                Eigen::Vector2d control(linear_v, angular_v);
                std::vector<Eigen::VectorXd> trajectory = generateTrajectory(control);

                // 경로의 비용 계산
                cost = computeTrajectoryCost(trajectory);
                // 비용이 가장 적은 경로 선택
                if (cost < min_cost)
                {
                    min_cost = cost;
                    best_trajectory = trajectory;
                }

                std::vector<std::vector<double>> dwa_pos;
                for (int i = 0; i < trajectory.size(); ++i)
                {
                    double x = trajectory[i](0);
                    double y = trajectory[i](1);
                    std::vector<double> pos = {x, y};
                    dwa_pos.push_back(pos);
                }
                all_paths_vectors_.push_back(dwa_pos);
            }
        }
        return best_trajectory;
    }

    // 목적함수
    double computeTrajectoryCost(const std::vector<Eigen::VectorXd> &trajectory)
    {
        return computeDistanceToGoalCost(trajectory) * kGoalCostWeight_ + computeDistanceToObstacleCost(trajectory) * kObstacleCostWeight_ +
               computeVelocityCost(trajectory) * kVelocityCostWeight_ + computeSmoothnessCost(trajectory) * kSmoothnessCostWeight_;
    }

    // 목표까지의 거리 기반 비용 계산
    double computeDistanceToGoalCost(const std::vector<Eigen::VectorXd> &trajectory)
    {
        double goal_cost;
        goal_cost = std::hypot(trajectory.back()(0) - goal_(0), trajectory.back()(1) - goal_(1));
        return goal_cost;
    }

    // 장애물과의 거리 기반 비용 계산
    double computeDistanceToObstacleCost(const std::vector<Eigen::VectorXd> &trajectory)
    {
        double obstacle_cost = INT_MAX;
        double minimum_separation = INT_MAX;

        // 경로에서 장애물과의 최소 거리를 계산하여 비용 산출
        for (int i = 0; i < trajectory.size(); i = i + 2)
        {
            for (int j = 0; j < obstacles_.size(); j++)
            {
                double oX = obstacles_[j][0];
                double oY = obstacles_[j][1];
                double dx = trajectory[i][0] - oX;
                double dy = trajectory[i][1] - oY;

                double separation = std::sqrt(dx * dx + dy * dy);
                if (separation <= kCarRadius_) // 차량 반경 내에 장애물이 있으면 최대 비용 반환
                {
                    return obstacle_cost;
                }

                if (minimum_separation >= separation) // 최소 거리 갱신
                {
                    minimum_separation = separation;
                }
            }
        }
        obstacle_cost = 1 / minimum_separation; // 최소 거리의 역수로 비용 계산

        return obstacle_cost;
    }

    // 장애물과의 분산 기반 비용 계산
    double computeDistanceVarianceCost(const std::vector<Eigen::VectorXd> &trajectory)
    {
        std::vector<double> distances; // 장애물과의 거리 저장
        double total_distance = 0.0;

        // 경로에서 장애물과의 거리를 모두 계산
        for (int i = 0; i < trajectory.size(); i = i + 2)
        {
            for (int j = 0; j < obstacles_.size(); j++)
            {
                double oX = obstacles_[j][0];
                double oY = obstacles_[j][1];
                double dx = trajectory[i][0] - oX;
                double dy = trajectory[i][1] - oY;

                double separation = std::sqrt(dx * dx + dy * dy);

                if (separation <= kCarRadius_) // 차량 반경 내에 장애물이 있으면 최대 비용 반환
                {
                    return INT_MAX;
                }

                distances.push_back(separation); // 거리 저장
                total_distance += separation;    // 총 거리 계산
            }
        }

        // 평균 거리 계산
        double mean_distance = total_distance / distances.size();

        // 분산 계산
        double variance_sum = 0.0;
        for (double d : distances)
        {
            variance_sum += std::pow(d - mean_distance, 2);
        }
        double variance_cost = variance_sum / distances.size();

        return variance_cost;
    }

    // 속도 기반 비용 계산 (선속도가 최대 속도에서 얼마나 떨어졌는지)
    double computeVelocityCost(const std::vector<Eigen::VectorXd> &trajectory)
    {
        double velocity_cost = kMaxLinearVelocity_ - trajectory.back()[3];
        return velocity_cost;
    }

    // yaw 변화량에 대한 비용 계산(변화량이 얼마나 작은지)
    double computeSmoothnessCost(const std::vector<Eigen::VectorXd> &trajectory)
    {
        double smoothness_cost = 0.0;
        for (size_t i = 1; i < trajectory.size(); ++i)
        {
            double yaw_diff = std::abs(trajectory[i][2] - trajectory[i - 1][2]);
            smoothness_cost += yaw_diff;
        }
        return smoothness_cost;
    }

    // 동적 창 알고리즘을 실행하는 함수
    void doDynamicWindowApproach(std::vector<double> &goal_pos, std::vector<std::vector<double>> &obstacles)
    {
        // 초기 상태 설정 (x, y, theta, 선속도, 각속도)
        const double *cur_car_utm = local_.getAddCurCarUTMPos();
        double cur_yaw = local_.getCurCarYaw();
        double cur_velocity = local_.getCurCarVelocity();
        double cur_yaw_rate = local_.getCurYawRate();
        // filtered_yaw_ = yaw_filter_.filter(cur_yaw); // TODO: ??????

        // 장애물 업데이트
        obstacles_ = obstacles;

        dwa_current_state_ << cur_car_utm[0], cur_car_utm[1], cur_yaw, cur_velocity, cur_yaw_rate;

        // 목표 위치 설정
        goal_(0) = goal_pos[0];
        goal_(1) = goal_pos[1];

        // 동적 창을 계산
        updateDynamicWindow();

        // 가능한 경로 중 비용이 가장 적은 경로를 선택
        std::vector<Eigen::VectorXd> bestTrajectory;
        bestTrajectory = getOptimalTrajectory();

        // local_path 업데이트
        std::vector<std::vector<double>> dwa_pos;
        std::vector<double> dwa_yaw;
        std::vector<double> dwa_k;
        for (int i = 0; i < bestTrajectory.size(); ++i)
        {
            double x = bestTrajectory[i](0);
            double y = bestTrajectory[i](1);
            double yaw = bestTrajectory[i](2);
            double k = 0;
            std::vector<double> pos = {x, y};
            dwa_pos.push_back(pos);
            dwa_yaw.push_back(yaw);
            dwa_k.push_back(k);
        }

        Path dwa_path("dwaPath", dwa_pos, dwa_yaw, dwa_k);

        local_path_.updatePath(0, local_path_.getRefPosArr().size(), dwa_path, 0, dwa_pos.size());

        // 다음 속도 전달
        // control_.setTargetVelocity(bestTrajectory[1](3));
        // control_.setTargetVelocity(2);
        std::cout << "path_index: " << dwa_pos.size() << "\n";
    }

    /* dwa trajectory plot test */
    const std::vector<std::vector<std::vector<double>>> &getAllPathVector() const
    {
        return all_paths_vectors_;
    }

    const std::vector<double> getRefPoint() const
    {
        return {goal_(0), goal_(1)};
    }
};

#endif //  INCLUDE_DYNAMICWINDOWAPPROACH_HPP_
