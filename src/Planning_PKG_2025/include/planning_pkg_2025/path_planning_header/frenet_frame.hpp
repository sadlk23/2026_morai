/***
 * 작성자: 정성윤
 * 설명: frenet_frame trajectory 관련 클래스
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef FRENET_FRAME_H_
#define FRENET_FRAME_H_

#include <vector>
#include "local.hpp"
#include "optimalTrajectoryPlanner.hpp"

class FrenetFrame
{
private:
    OptimalTrajectoryPlanner otp_;

    bool start_flag_ = false; // startFlag

    int make_path_count_ = 0;

    std::vector<std::vector<double>> center_lane_; // frenet frame용 center lane
    std::vector<std::vector<double>> obstacles_;

    double d0;  // 초기 횡방향 위치
    double dv0; // 초기 횡방향 속도
    double da0; // 초기 횡방향 가속도
    double s0;  // 초기 종방향 위치
    double sv0; // 초기 종방향 속도 (10 km/h를 m/s로 변환)

    /* test 용 vector */
    std::vector<std::vector<std::vector<double>>> all_paths_vectors;

public:
    bool getStartFlag()
    {
        return start_flag_;
    }

    int getMakePathCount()
    {
        return make_path_count_;
    }

    void setDefault(const Path &cur_global_path, const std::vector<std::vector<double>> &obstacles)
    {
        // 같은 미션에서 경로를 다시 생성할 때 이전 기준 경로가 누적되지 않게 한다.
        center_lane_.clear();
        all_paths_vectors.clear();
        make_path_count_ = 0;

        // centerLane을 global_path로부터 생성
        double distance_traced = 0.0;
        double prev_x = cur_global_path.getRefPosArr()[0][0];
        double prev_y = cur_global_path.getRefPosArr()[0][1];

        const auto &global_pos_arr = cur_global_path.getRefPosArr();
        const auto &global_yaw_arr = cur_global_path.getRefYawArr();
        const auto &global_k_arr = cur_global_path.getRefKArr();

        for (size_t i = 0; i < global_pos_arr.size(); ++i)
        {
            double x = global_pos_arr[i][0];
            double y = global_pos_arr[i][1];
            double yaw = global_yaw_arr[i];
            double curvature = global_k_arr[i];

            distance_traced += std::sqrt(std::pow(x - prev_x, 2) + std::pow(y - prev_y, 2));
            center_lane_.push_back({x, y, yaw, curvature, distance_traced});

            prev_x = x;
            prev_y = y;
        }

        obstacles_ = obstacles;

        /* 세팅필요!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1 */
        d0 = 0;  // 초기 횡방향 위치
        dv0 = 0; // 초기 횡방향 속도, 고정?
        da0 = 0; // 초기 횡방향 가속도, 고정?
        s0 = 0;  // 초기 종방향 위치
        sv0 = 0; // 초기 종방향 속도 (10 km/h를 m/s로 변환), r고정?

        start_flag_ = true;
    }

    Path getOptimalTrajectory(const double *cur_utm_pos, const double cur_velocity, const double *cur_utm_accel, const double yaw, const std::vector<std::vector<double>> &obstacles)
    {
        obstacles_ = obstacles;
        std::vector<FrenetPath> all_paths;

        // frenet 좌표계로 변환
        std::vector<double> frenet_pos = otp_.convertToFrenet(cur_utm_pos[0], cur_utm_pos[1], center_lane_);
        std::vector<double> frenet_velocity = otp_.convertVelocityToFrenet(cur_utm_pos[0], cur_utm_pos[1], cur_velocity, yaw, center_lane_);
        std::vector<double> frenet_accel = otp_.convertAccelerationToFrenet(cur_utm_pos[0], cur_utm_pos[1], cur_utm_accel[0], cur_utm_accel[1], center_lane_);

        // 현재 상태 정보 초기화
        s0 = frenet_pos[0];
        d0 = frenet_pos[1];
        sv0 = frenet_velocity[0];
        dv0 = frenet_velocity[1];
        da0 = frenet_accel[0];

        // 최적 경로 생성
        FrenetPath optimal_path = otp_.optimalTrajectory(d0, dv0, da0, s0, sv0, center_lane_, obstacles_, all_paths);

        // 생성된 경로를 local_path에 적용
        std::vector<std::vector<double>> new_path = optimal_path.world;
        std::vector<double> new_path_yaw, new_path_k;
        for (const auto &point : optimal_path.world)
        {
            new_path_yaw.push_back(point[2]); // yaw
            new_path_k.push_back(point[3]);   // curvature
        }

        /* **********************test용************************** */
        all_paths_vectors.clear();
        for (FrenetPath &p : all_paths)
        {
            all_paths_vectors.push_back(p.world);
        }
        /* 여기까지 test */

        Path optimal_trajectory("optimalTrajectory", new_path, new_path_yaw, new_path_k);
        make_path_count_ = (++make_path_count_) % 5;
        return optimal_trajectory;
    }

    /* frenet frame trajectory plot test */
    const std::vector<std::vector<std::vector<double>>> &getAllPathVector() const
    {
        return all_paths_vectors;
    }
};

#endif
