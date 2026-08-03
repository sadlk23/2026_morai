#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <cmath>
#include <algorithm>

/*
제작 : 예원태
설명 : 
ROS1 Noetic 변환 버전.
기본 좌표 구조체(Point), 제어 출력 구조체(GasAndBrake), 
유틸리티 함수(clip, LPF, angle normalization) 및 PID 제어기 클래스가 정의되어 있다.
*/

// 기초 구조체 정의
struct Point {
    double x;
    double y;
};

struct GasAndBrake {
    double gas;
    double brake;
};

// --- 유틸리티 함수 ---

// 두 점을 연결하는 벡터의 외적 계산 (횡방향 에러의 방향 판별 등에 사용)
inline double cross_product(Point p1, Point p2) {
    return p1.x * p2.y - p1.y * p2.x;
}

// 기준선 대비 타겟 점이 어느 쪽에 있는지 판별
inline double determine_side(Point reference, Point target, Point origin) {
    Point refVector = {reference.x - origin.x, reference.y - origin.y};
    Point targetVector = {target.x - origin.x, target.y - origin.y};
    return cross_product(refVector, targetVector);
}

// 값을 min_val과 max_val 사이로 제한
template <typename T>
inline T clip(const T &value, const T &min_val, const T &max_val) {
    return std::max(min_val, std::min(value, max_val));
}

// Low Pass Filter (신호 노이즈 제거 및 부드러운 제어 출력용)
inline double low_pass_filter(double val, double pre_val, float alpha) {
    return val * alpha + pre_val * (1.0 - (double)alpha);
}

// 각도를 -PI ~ PI 사이로 정규화
inline double nomalize_angle(double rad_angle) {
    double rad_ang = rad_angle;
    while (rad_ang > M_PI) rad_ang -= 2.0 * M_PI;
    while (rad_ang < -M_PI) rad_ang += 2.0 * M_PI;
    return rad_ang;
}

// --- PID 제어기 클래스 ---

class PIDController
{
private:
    double kp_, ki_, kd_;
    double min_output_, max_output_;
    double integral_;
    double prev_error_;
    double pre_derivative_;
    bool has_prev_;

public:
    PIDController(double min_output, double max_output)
        : min_output_(min_output), max_output_(max_output),
          integral_(0.0), prev_error_(0.0), pre_derivative_(0.0), has_prev_(false) {
        kp_ = 0.0; ki_ = 0.0; kd_ = 0.0;
    }

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        pre_derivative_ = 0.0;
        has_prev_ = false;
    }

    void set_PID_gain(double kp, double kd, double ki = 0.0) {
        kp_ = kp;
        kd_ = kd;
        ki_ = ki; 
    }

    // PID 연산 루틴
    double compute(double target_value, double measured_value, float alpha, double dt = 0.05) {
        double error = target_value - measured_value;
        
        // I 제어: 적분항 누적
        integral_ += error * dt;

        // D 제어: 미분항 계산 (에러의 변화량)
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        // 미분항 노이즈 제거를 위한 LPF 적용
        derivative = low_pass_filter(derivative, pre_derivative_, alpha);
        pre_derivative_ = derivative;

        // 최종 출력 산출
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        
        // 출력 제한 (Saturation)
        return clip(output, min_output_, max_output_);
    }

    // 디버깅용 게인 반환 함수
    float get_p_gain() const { return (float)kp_; }
    float get_d_gain() const { return (float)kd_; }
};

#endif  // PID_CONTROLLER_HPP