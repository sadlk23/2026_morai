#ifndef INCLUDE_OPTIMALTRAJECTORYPLANNER_HPP_
#define INCLUDE_OPTIMALTRAJECTORYPLANNER_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include <iostream>
#include <polynomial.hpp>
#include <frenetPath.hpp>
#include <Eigen/Dense>

// OptimalTrajectoryPlanner 클래스는 최적의 경로를 생성하고 검증하는 데 사용
class OptimalTrajectoryPlanner
{
public:
    // 기본 생성자 (Default Constructor)
    OptimalTrajectoryPlanner();

    // 소멸자 (Destructor)
    ~OptimalTrajectoryPlanner();

    // 최적의 경로를 생성 (Generate the optimal trajectory)
    // 초기 횡방향 상태(d0, dv0, da0)와 초기 종방향 상태(s0, sv0), 차선 중심(centerLane), 장애물(obstacles)을 사용
    // 모든 경로(allPaths)를 반환
    FrenetPath optimalTrajectory(double d0, double dv0, double da0,
                                 double s0, double sv0,
                                 std::vector<std::vector<double>> &centerLane,
                                 std::vector<std::vector<double>> &obstacles,
                                 std::vector<FrenetPath> &allPaths);

    // 경로의 비용을 계산 (Calculate the cost of the trajectory)
    void trajectoryCost(FrenetPath &path);

    // 경로와 장애물 간의 충돌 여부를 확인 (Check for collision between the path and obstacles)
    bool isColliding(FrenetPath &path, std::vector<std::vector<double>> &obstacles);

    // 경로가 운동학적 제약 조건을 만족하는지 확인 (Check if the path satisfies kinematic constraints)
    bool isWithinKinematicConstraints(FrenetPath &path);

    // 유효한 경로를 반환 (Return valid paths)
    // 주어진 경로(paths)와 장애물(obstacles)을 사용
    std::vector<FrenetPath> isValid(std::vector<FrenetPath> &paths, std::vector<std::vector<double>> &obstacles);

    // Frenet 경로를 세계 좌표계로 변환 (Convert Frenet paths to world frame)
    void convertToWorldFrame(std::vector<FrenetPath> &paths, std::vector<std::vector<double>> &centerLane);

    // 회전을 계산 (Calculate rotation)
    Eigen::Vector2d rotation(double theta, double x, double y);

    // Frenet 변환 (위치)
    std::vector<double> convertToFrenet(double x, double y, const std::vector<std::vector<double>> &centerLane);

    // Frenet 변환 함수 (속도)
    std::vector<double> convertVelocityToFrenet(double x, double y, double speed, double yaw, const std::vector<std::vector<double>> &centerLane);

    // Frenet 변환 (가속도)
    std::vector<double> convertAccelerationToFrenet(double x, double y, double ax, double ay, const std::vector<std::vector<double>> &centerLane);

private:
    double pi_ = 3.14159;            // 원주율 (Pi value)
    double maxVelocity_ = 6.9;       // 최대 속도 (Max velocity in m/s)
    double maxAcceleration_ = 2;     // 최대 가속도 (Max acceleration in m/s^2)
    double maxSteeringAngle_ = 0.5;  // 최대 조향 각도 (Max steering angle in radians)
    double maxCurvature_ = 1;        // 최대 곡률 (Max curvature)
    double robotFootprint_ = 1.5;    // 로봇 발자국 (Robot footprint radius)
    double maxPredictionStep_ = 4;   // 최대 예측 단계 (Max prediction step time in seconds)
    double minPredictionStep_ = 1.5; // 최소 예측 단계 (Min prediction step time in seconds)
    double noOfLanes_ = 3;           // 차선 수 (Number of lanes)
    double laneWidth_ = 1.5;         // 차선 너비 (Lane width in meters)
    double targetVelocity_ = 2.7;    // 목표 속도 (Target velocity in m/s)
    double velocityStep_ = 1;        // 속도 단계 (Velocity step in m/s)
    double timeStep_ = 0.1;          // 시간 단계 (Time step in seconds)
    double klat_ = 2;                // 횡방향 비용 가중치 (Lateral cost weight)
    double klon_ = 1;                // 종방향 비용 가중치 (Longitudinal cost weight)
    double kjd_ = 0.1;               // 횡방향 jerk 비용 가중치 (Lateral jerk cost weight)
    double ktd_ = 0.1;               // 횡방향 시간 비용 가중치 (Lateral time cost weight)
    double ksd_ = 2;                 // 횡방향 거리 비용 가중치 (Lateral distance cost weight)
    double kjs_ = 0.1;               // 종방향 jerk 비용 가중치 (Longitudinal jerk cost weight)
    double kts_ = 0.1;               // 종방향 시간 비용 가중치 (Longitudinal time cost weight)
    double kss_ = 2;                 // 종방향 거리 비용 가중치 (Longitudinal distance cost weight)
    double safe_distance_ = 1;
};

#endif //  INCLUDE_OPTIMALTRAJECTORYPLANNER_HPP_
