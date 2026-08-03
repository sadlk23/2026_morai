#ifndef LON_CONTROLLER_HPP
#define LON_CONTROLLER_HPP

#include <iostream>
#include <memory>  // shared_ptr 사용을 위해 필요

// ROS1 환경 기반 헤더들
#include "control/callback_data_manage.hpp"
#include "control/PIDController.hpp"

/*
제작 : 예원태
설명 : 
ROS1 Noetic 변환 버전.
종방향 제어를 위한 PID 제어기 및 Gas/Brake 로직이 정의되어 있다.
*/

class LonController
{
private:
    // ROS1/C++11 표준 스마트 포인터 유지
    std::shared_ptr<PIDController> gas_pid;
    std::shared_ptr<PIDController> brake_pid;

    std::shared_ptr<CallbackClass> callback_data_ptr;
    CallbackClass *cb_data;

    float current_speed_;
    float target_speed_;

    float gas_scale_;   // gas : 0 ~ 6.9444 m/s (약 25km/h)
    float brake_scale_; // brake : 0 ~ 200

    float pre_calc_speed = 0.0;
    float pre_calc_brake = 0.0;
    float pre_target_speed = 0.0;
    float enable_brake_error = 0.6;

public:
    LonController()
        : gas_scale_(6.9), brake_scale_(200)
    {
        // ROS1 환경에서도 CallbackClass는 동일하게 작동하도록 구성됨
        callback_data_ptr = std::make_shared<CallbackClass>();
        cb_data = callback_data_ptr.get();

        gas_pid = std::make_shared<PIDController>(-1.0, 1.0);
        brake_pid = std::make_shared<PIDController>(-1.0, 1.0);
    }

    void set_lon_data(float current_speed)
    {
        this->current_speed_ = current_speed;
    }

    void set_lon_target_speed(float target_speed)
    {
        this->pre_target_speed = this->target_speed_; // 이전 타겟 속도 저장
        this->target_speed_ = target_speed;
    }

    double get_lon_target_speed() { return this->target_speed_; }
    double get_enable_brake_error() { return this->enable_brake_error; }
    void set_enable_brake_error(float enable_error) { this->enable_brake_error = enable_error; }

    void set_lon_PD_gain(double ga_kp, double ga_kd, double br_kp, double br_kd)
    {
        // ROS1에서도 PID 연산 로직은 동일
        gas_pid->set_PID_gain(ga_kp, ga_kd);
        brake_pid->set_PID_gain(br_kp, br_kd);

        // 급감속 시나리오에서 Brake Gain 보정 로직
        if (this->pre_target_speed != this->target_speed_ && this->target_speed_ < 2.5 && this->target_speed_ > 1.7)
        {
            float speed_gap = std::abs(this->pre_target_speed - this->target_speed_);
            brake_pid->set_PID_gain(br_kp + speed_gap * 0.004, br_kd);
        }
    }

    GasAndBrake calc_gas_n_brake()
    {
        float error = target_speed_ - current_speed_;
        GasAndBrake return_val;
        
        // 제어 주기(dt)는 0.1s로 가정 (메인 루프 주기에 맞게 조정 필요)
        double gas_val = gas_pid->compute(target_speed_, current_speed_, 0.1);

        // [상태별 제어 로직]
        // 1. 가속 구간
        if (-this->enable_brake_error < error)
        {
            return_val.gas = target_speed_ + gas_val * gas_scale_;
            return_val.brake = 0;
        }
        // 2. 데드존 (속도 유지)
        else if (-2 * this->enable_brake_error < error && error <= -this->enable_brake_error)
        {
            return_val.gas = target_speed_;
            return_val.brake = 0;
        }
        // 3. 감속 구간 (Brake 작동)
        else
        {
            double brake_val = brake_pid->compute(target_speed_, current_speed_, 0.3);
            return_val.gas = 0;
            return_val.brake = 30 - brake_val * brake_scale_;
        }

        // 출력 제한 (Saturation)
        return_val.gas = clip(return_val.gas, 0.0, 6.9);

        // 고속 주행 시 미션 번호에 따른 정밀 가스 제어
        if (current_speed_ >= 6.0)
        {
            // ROS1용 CallbackClass의 get_mission_num() 호출
            int mission = cb_data->get_mission_num();
            double speed_factor = (mission == 12 || mission == 35) ? 0.0050 : 0.0321;

            if (std::abs(current_speed_ - target_speed_) < target_speed_ * speed_factor * target_speed_ && error > 0)
            {
                return_val.gas = clip(return_val.gas, 0.0, static_cast<double>(target_speed_ * (1 - speed_factor * target_speed_)));
            }
            else if (error <= 0)
            {
                return_val.gas = clip(return_val.gas, 0.0, static_cast<double>(target_speed_ * (1 - speed_factor * target_speed_) + error));
            }
        }

        // 급격한 출력 변화 방지를 위한 LPF
        return_val.gas = low_pass_filter(return_val.gas, this->pre_calc_speed, 0.6);
        
        if (return_val.brake > 0.0)  //수정사항
        {
            // 저속 구간에서의 최소 제동력 보장
            if (this->target_speed_ < 1.7) return_val.brake = clip(return_val.brake, 10.0, 199.0);
            else return_val.brake = clip(return_val.brake, 20.0, 199.0);
        }
        
        

        this->pre_calc_speed = return_val.gas;
        this->pre_calc_brake = return_val.brake;

        return_val.gas = return_val.gas / 6.9;       // 최대 속도(6.9)로 나눔
        return_val.brake = return_val.brake / 200.0; // 최대 브레이크(200)로 나눔

        return return_val;
    }

    float get_target_speed() { return this->target_speed_; }
};

#endif // LON_CONTROLLER_HPP