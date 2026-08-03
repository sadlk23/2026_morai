/***
 * 작성자: 조아진
 * 설명: Control 클래스 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef CONTROL_H_
#define CONTROL_H_

#include <iostream>
#include <chrono>
#include "nlohmann/json.hpp"

enum class Velocity
{
    STOP,    // 정지
    NORMAL,  // 속도 2
    REVERSE, // 후진
    FAST     // 속도 4
};

class Control
{
private:
    enum class State
    {
        NORMAL,
        DECELERATION,
        WAIT
    };

    State current_state_ = State::NORMAL;
    double cur_target_velocity_; // 속도참고 [저속주행 : 5~6, 일반 : 20, 최대 25]
    double mission_target_velocity_;
    bool distance_control_applied_ = false;
    std::chrono::system_clock::time_point wait_begin_time_;

    double cur_braking_distance_ = 5;
    double deceleration_start_velocity_ = 2; // 감속 시작 속도
    const double kMinDistance;               // 멈추기 전 최소 거리
    const double kMinVelocity;               // 최소 속도
    const double kBrakeCoefficient;          // 브레이크 계수
    const double kBrakeDistance;             // 감속 시작 거리
    const double kSafeDistance;              // 안전거리

public:
    Control(nlohmann::json &param, double velocity = 0) : cur_target_velocity_{velocity},
                                                          mission_target_velocity_{velocity},
                                                          kMinDistance(param["min_distance"]),
                                                          kMinVelocity(param["min_velocity"]),
                                                          kBrakeCoefficient(param["brake_coefficient"]),
                                                          kBrakeDistance(param["brake_distance"]),
                                                          kSafeDistance(param["safe_distance"])
    {
    }

    /* getter 함수 */
    double getTargetVelocity() const
    {
        return cur_target_velocity_;
    }

    double getMissionTargetVelocity() const
    {
        return mission_target_velocity_;
    }

    /* setter 함수 */
    void setTargetVelocity(double velocity)
    {
        cur_target_velocity_ = velocity;
        mission_target_velocity_ = velocity;
        distance_control_applied_ = false;
    }

    void setTargetVelocity(Velocity v)
    {
        distance_control_applied_ = false;
        switch (v)
        {
        case Velocity::STOP:
            cur_target_velocity_ = 0;
            break;
        case Velocity::NORMAL:
            cur_target_velocity_ = 2;
            break;
        case Velocity::REVERSE:
            cur_target_velocity_ = -1;
            break;
        case Velocity::FAST:
            cur_target_velocity_ = 4;
            break;
        default:
            std::cout << "잘못된 값 들어옴, target_velocity=1\n";
            cur_target_velocity_ = 1;
            break;
        }
        mission_target_velocity_ = cur_target_velocity_;
    }

    // 시뮬레이션용 함수

    void beginControlCycle()
    {
        cur_target_velocity_ = mission_target_velocity_;
        distance_control_applied_ = false;
    }

    bool wasDistanceControlApplied() const
    {
        return distance_control_applied_;
    }

    double previewDistanceControlledVelocity(
        double distance, double mission_target_velocity) const
    {
        // distance_control()과 같은 기준으로 최종 목표속도를 미리 계산하되 상태는 변경하지 않는다.
        if (mission_target_velocity < 0.0 || distance == -1)
            return mission_target_velocity;
        if (distance < kBrakeDistance)
            return 0.0;
        if (distance < 7.0)
            return mission_target_velocity / 8.0;
        if (distance < 10.0)
            return mission_target_velocity / 6.0;
        if (distance < 14.0)
            return mission_target_velocity / 4.0;
        if (distance < 20.0)
            return mission_target_velocity / 2.0;
        return mission_target_velocity;
    }

    void distance_control(double distance, double current_vel = 0.99){
        std::cout << "distance_control 돌긴함" << std::endl;

        if (current_vel == 0.99)
        {
            current_vel = getTargetVelocity();
        }

        // /LiDAR/car_dis는 차량 전방 ROI이므로 후진 명령의 부호나
        // downstream의 -1.0 후진 프로토콜을 변경하지 않는다.
        if (current_vel < 0.0)
        {
            distance_control_applied_ = true;
            return;
        }

        if (distance != -1)
        {
            std::cout << "여기 들어옴? 그니까 컨트롤 속도조절 들어오는 부분" << std::endl;
            if (distance < kBrakeDistance)
                std::cout << distance << std::endl;
            cur_target_velocity_ =
                previewDistanceControlledVelocity(
                    distance, current_vel);
        }
        distance_control_applied_ = true;
    }

    bool checkWaitTime(auto wait_begin_time, int wait_duration = 1) // 특정 시간이 지났는지 확인해주는 함수
    {
        std::chrono::seconds cur_wait_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - wait_begin_time);
        if (cur_wait_time.count() >= wait_duration)
        {
            return true;
        }
        else
        {
            std::cout << "대기한 시간(sec) : " << cur_wait_time.count() << "\n";
            return false;
        }
    }

    bool waitAndDepart(int wait_time, double depart_velocity) // 차량을 일정 시간동안 멈췄다 다시 출발하는 함수
    {
        if (current_state_ != State::WAIT)
        {
            wait_begin_time_ = std::chrono::system_clock::now();
            setTargetVelocity(Velocity::STOP);
            current_state_ = State::WAIT;
        }
        std::cout << "현재 속도 -> " << getTargetVelocity() << "\n";

        if (checkWaitTime(wait_begin_time_, wait_time))
        {
            if (current_state_ == State::WAIT)
            {
                current_state_ = State::NORMAL;
            }

            if (depart_velocity != -2024)
            {
                setTargetVelocity(depart_velocity);
            }
            else
            {
                std::cout << "잘못된 속도값!!\n";
            }

            return true;
        }
        else
        {
            return false;
        }
    }

    double getBrakingDistance()
    {
        return cur_target_velocity_ * kBrakeCoefficient;
    }

    double calculateDeceleration(double distance)
    {
        if (distance < kMinDistance)
        {
            return kMinVelocity;
        }
        else if (distance > cur_braking_distance_)
        {
            return deceleration_start_velocity_;
        }
        else
        {
            double ratio = (distance - kMinDistance) / (cur_braking_distance_ - kMinDistance);
            return kMinVelocity + ratio * (deceleration_start_velocity_ - kMinVelocity);
        }
    }

    bool decelByDistanceToStop(double distance)
    {
        if (current_state_ != State::DECELERATION)
        {
            deceleration_start_velocity_ = cur_target_velocity_;
            cur_braking_distance_ = getBrakingDistance();
            current_state_ = State::DECELERATION;
        }
        else
        {
            setTargetVelocity(calculateDeceleration(distance));
            if (distance < kMinDistance)
            {
                current_state_ = State::NORMAL;
                return true;
            }
        }
        return false;
    }

    void decelByDistance(double distance, double min_velocity)
    {
        cur_braking_distance_ = getBrakingDistance();
        double decel_velocity = calculateDeceleration(distance);
        if (decel_velocity < min_velocity)
        {
            setTargetVelocity(min_velocity);
        }
        else
        {
            setTargetVelocity(decel_velocity);
        }
    }

    /* test용 함수 */
    void printAllData()
    {
        std::cout << "############## velocity ##############\n";
        std::cout << cur_target_velocity_ << "\n";
    }
};

#endif
