#ifndef INCLUDE_FRENETPATH_HPP_
#define INCLUDE_FRENETPATH_HPP_

#include <vector>
#include <cmath>
#include <iostream>

// FrenetPath 클래스는 Frenet 좌표계에서 궤적을 나타내는 데 사용
class FrenetPath
{
public:
    // 종방향 데이터 (Longitudinal Data)
    // s는 종방향 위치, 속도, 가속도, jerk와 시간의 벡터
    std::vector<std::vector<double>> s;

    // 횡방향 데이터 (Latitudinal Data)
    // d는 횡방향 위치, 속도, 가속도, jerk와 시간의 벡터
    std::vector<std::vector<double>> d;

    // 세계 좌표계 데이터 (World Frame Data)
    // world는 각 점의 x, y 좌표와 yaw, 거리의 벡터
    std::vector<std::vector<double>> world;

    // 예측 시간 (Prediction Time)
    // 궤적을 예측할 시간 길이
    double T;

    // 비용 (Cost)
    // 궤적의 총 비용
    double cf;

    // 종방향, 횡방향 jerk 합계 (Sum of Jerk in Longitudinal and Latitudinal Directions)
    // 각각 종방향과 횡방향의 jerk 비용
    double jd, js;

    // 최대 속도 (Maximum Velocity)
    // 궤적 중 최대 속도를 저장
    double maxVelocity = INT_MIN;

    // 최대 가속도 (Maximum Acceleration)
    // 궤적 중 최대 가속도를 저장
    double maxAcceleration = INT_MIN;

    // 최대 곡률 (Maximum Curvature)
    // 궤적 중 최대 곡률을 저장
    double maxCurvature = INT_MIN;
};

#endif //  INCLUDE_FRENETPATH_HPP_
