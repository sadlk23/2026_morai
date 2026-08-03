#ifndef INCLUDE_POLYNOMIAL_HPP_
#define INCLUDE_POLYNOMIAL_HPP_

#include <vector>
#include <cmath>
#include <iostream>
#include <string>
#include <Eigen/Dense>

// Polynomial 클래스는 특정 시간 t에서 위치, 속도, 가속도 및 jerk를 계산하는 데 사용
class Polynomial
{
public:
    // 기본 생성자 (Default Constructor)
    Polynomial();

    // 5차 다항식 생성자 (Quintic Polynomial Constructor)
    // 초기 위치(x0), 초기 속도(v0), 초기 가속도(a0), 목표 위치(xT), 목표 속도(vT), 목표 가속도(aT), 시간(T)
    Polynomial(double x0, double v0, double a0, double xT, double vT, double aT, double T);

    // 4차 다항식 생성자 (Quartic Polynomial Constructor)
    // 초기 위치(x0), 초기 속도(v0), 초기 가속도(a0), 목표 속도(vT), 목표 가속도(aT), 시간(T)
    Polynomial(double x0, double v0, double a0, double vT, double aT, double T);

    // 소멸자 (Destructor)
    ~Polynomial();

    // 특정 시간 t에서 위치를 계산 (Calculates the position at time t)
    double position(double t);

    // 특정 시간 t에서 속도를 계산 (Calculates the velocity at time t)
    double velocity(double t);

    // 특정 시간 t에서 가속도를 계산 (Calculates the acceleration at time t)
    double acceleration(double t);

    // 특정 시간 t에서 jerk를 계산 (Calculates the jerk at time t)
    double jerk(double t);

private:
    // 다항식 계수들 (Coefficients of the polynomial)
    double a1_, a2_, a3_, a4_, a5_, a6_;

    // 초기 및 목표 상태 변수들 (Initial and target state variables)
    double x0_, v0_, a0_, xT_, vT_, aT_, T_;

    // 다항식 유형 (Type of the polynomial: "quintic" or "quartic")
    std::string type_;
};

#endif //  INCLUDE_POLYNOMIAL_HPP_
