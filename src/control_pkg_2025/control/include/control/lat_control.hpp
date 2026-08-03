#ifndef LAT_CONTROLLERS_HPP
#define LAT_CONTROLLERS_HPP

#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "PIDController.hpp"        // ROS1 version - direct include
#include "callback_data_manage.hpp" // Fixed: was callback_data_manage.hpp

/*
제작 : 예원태
문의 : solnox99@koreatech.ac.kr

설명 :
2023년 창작차 대회에서 사용한 Control 코드이다.
횡방향 제어를 위한 Stanley, Pure Pursuit,
그리고 이 둘을 합친 Combine, Stanley preview,
후진 제어를 위한 Reverse Pure Pursuit,
클래스들이 정의되어 있다.

ROS1 버전으로 변환됨.
*/

class Stanley
{
private:
    double kp_;             // Proportional gain
    double ki_;             // Integral gain
    double kd_;             // Derivative gain
    double target_heading_; // Target heading
    double prevCrossTrackError;
    double integral;
    double default_i_val = 90;

    double anti_windup_max_;

    double crossTrackError_;
    float speed_;
    float ego_heading_;
    double curvature_;
    double heading_gain_;

public:
    Stanley()
        : prevCrossTrackError(0.0), integral(0.0)
    {
        kp_ = 1.6;
        ki_ = 0.0;
        kd_ = 1.5;
        target_heading_ = 0.0;
        ego_heading_ = 0.0;

        crossTrackError_ = 0.0;
        speed_ = 0.0;
        ego_heading_ = 0.0;
        curvature_ = 0.0;

        anti_windup_max_ = 0.0;
        heading_gain_ = 0.0;
    }

    void set_stanly_gain(double kp, double ki, double kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void set_heading_gain(double h_gain)
    {
        this->heading_gain_ = h_gain;
    }

    void set_anti_windup_max(double anti_windup_max)
    {
        this->anti_windup_max_ = anti_windup_max;
    }

    void set_stanly_data(float speed, double target_heading, float ego_heading,
                         double crossTrackError, double curvature = 0.0)
    {
        this->speed_ = clip(speed, 0.9F, 10.0F);
        // this->speed_ = speed;
        this->target_heading_ = target_heading;
        this->ego_heading_ = ego_heading;
        this->crossTrackError_ = crossTrackError;
        this->curvature_ = curvature; // 튜닝에 사용하는 param 일단은 0.0
    }

    double calc_stanly_steer(double dt = 1)
    {
        // Calculate the heading error
        double tmp_headin_error = nomalize_angle(target_heading_ - ego_heading_); // Fixed typo

        double headingError = this->heading_gain_ * (tmp_headin_error);

        // Normalize the heading error to the range [-pi, pi]
        headingError = nomalize_angle(headingError); // Fixed typo

        // Calculate the proportional term
        double proportionalTerm = kp_ * crossTrackError_;

        // Calculate the cross track error derivative
        double crossTrackErrorDerivative = (crossTrackError_ - prevCrossTrackError) / dt;

        // Calculate the derivative term
        double derivativeTerm = kd_ * crossTrackErrorDerivative;

        // Update the previous cross track error for the next iteration
        prevCrossTrackError = crossTrackError_;

        // Calculate the integral term
        integral += crossTrackError_ * dt;
        integral = clip(integral, -anti_windup_max_, anti_windup_max_);

        if (this->speed_ < 1)
        {
            integral = 0;
        }

        double integralTerm = ki_ * integral;

        // Calculate the steering angle using Stanley Method
        double PID_steer = proportionalTerm + derivativeTerm + integralTerm;

        double steeringAngle = headingError + atan2(PID_steer, (0.1 + this->speed_));
        steeringAngle = nomalize_angle(steeringAngle); // Fixed typo

        return steeringAngle;
    }

    double get_stanley_P_gain()
    {
        return kp_;
    }

    double get_stanley_integral_val()
    {
        return this->integral;
    }

    float get_speed_in_stanley()
    {
        return this->speed_;
    }

    void set_stanley_integral_val(double inte_init) // Fixed missing return type
    {
        this->integral = inte_init;
    }
};

class PurePursuit
{
private:
    float lookaheadDistance;
    float lookahead_gain_;
    Point targetPoint_;

    Point current_odom_;
    float speed_;
    double yaw_;
    std::vector<Point> path_; // Fixed namespace
    float wheel_base_;

    float ld_min_val;
    float ld_max_val;

    // tuning param
    float PP_gain_;

public:
    PurePursuit()
        : PP_gain_(0.18)
    {
        lookaheadDistance = 0.0;
        lookahead_gain_ = 0.0;
        targetPoint_.x = 0.0;
        targetPoint_.y = 0.0;

        current_odom_.x = 0.0;
        current_odom_.y = 0.0;
        speed_ = 0.0;
        yaw_ = 0.0;
        wheel_base_ = 1.04;

        ld_min_val = 7.5;
        ld_max_val = 11;
    }

    void set_pp_gain(double pp_gain, float lookahead)
    {
        this->PP_gain_ = pp_gain;
        this->lookahead_gain_ = lookahead;
    }

    void set_pp_LD_threshold(float min, float max)
    {
        this->ld_min_val = min;
        this->ld_max_val = max;
    }

    void set_pp_data(float speed, double yaw, const Point &current_odom, std::vector<Point> path) // Fixed namespace
    {
        this->speed_ = speed;
        this->yaw_ = yaw;
        this->current_odom_ = current_odom;
        this->path_ = path;
    }

    void calc_lookahead_distance(float min_dist, float max_dist)
    {
        lookaheadDistance = lookahead_gain_ * speed_;
        lookaheadDistance = std::min(std::max(lookaheadDistance, min_dist), max_dist);
    }

    Point calc_LA_point(float min_dist, float max_dist, bool is_reverse = false)
    {
        calc_lookahead_distance(min_dist, max_dist);

        double minDistance = 1e9; // 아주 큰 값으로 초기화
        Point closestPoint;

        // 경로 상의 점들과 나와의 거리 계산
        for (const Point &p : path_)
        {
            double dist_diff = std::abs(LA_distance(p, is_reverse) - lookaheadDistance); // Fixed namespace
            if (dist_diff < minDistance)
            {
                minDistance = dist_diff;
                closestPoint = p;
            }

            // TODO: minDistance가 일정 값 이상으로 증가하고 있으면
            // break;
            // 유턴이나 이런 부분에서의 에러를 잡을 수 있을 것 같다.
        }

        return closestPoint;
    }

    double calc_PP_Steer()
    {
        // Calculate the relative target point position
        this->targetPoint_ = calc_LA_point(ld_min_val, ld_max_val);

        // Calculate the distance to the target point
        double distanceToTarget = LA_distance(targetPoint_);

        // Calculate the angle between the current pose and the target point
        double numerator = 2 * wheel_base_ * targetPoint_.y;
        double denominator = distanceToTarget * distanceToTarget * PP_gain_;
        double angleToTarget = std::atan2(numerator, denominator);

        return angleToTarget;
    }

    double LA_distance(const Point &p, bool is_reverse = false)
    {
        if (is_reverse == false)
        {
            return std::sqrt(((p.x + wheel_base_) * (p.x + wheel_base_)) + (p.y * p.y));
        }
        else
        {
            return std::sqrt(((p.x - wheel_base_) * (p.x - wheel_base_)) + (p.y * p.y));
        }
    }

    double get_LA_distance()
    {
        return this->lookaheadDistance;
    }

    float get_wheel_base()
    {
        return this->wheel_base_;
    }

    Point get_target_point()
    {
        return this->targetPoint_;
    }
};

class CombinedSteer : public PurePursuit, public Stanley
{
private:
    CallbackClass *cb_data_;

    float pp_weight_, stanly_weight_;
    double deltaMax;

    double pre_com_steer;
    float com_steer_alpha;

    double PP_steer;
    double stanly_steer;

    Stanley preview_instance;
    float preview_dt = 0.0;
    int preview_num = 0;
    float preview_h_e_gain = 1.0;
    std::vector<float> preview_gain; // Fixed namespace
    std::vector<double> steers;      // Fixed namespace

public:
    CombinedSteer(CallbackClass *cb_data)
        : PurePursuit(), Stanley(), pp_weight_(0.45),
          stanly_weight_(0.55), deltaMax(0.491642)
    {

        this->cb_data_ = cb_data;

        pre_com_steer = 0;

        com_steer_alpha = 0.8;
    }

    void set_combine_PP_ratio(float pp_weight)
    {
        pp_weight_ = pp_weight;
        stanly_weight_ = 1 - pp_weight;
    }

    void set_preview_param(float dt, std::vector<float> gain) // Fixed namespace
    {
        this->preview_dt = dt;
        this->preview_num = gain.size();
        this->preview_gain = gain;
    }

    void set_preview_heading_error_gain(float h_e_g)
    {
        this->preview_h_e_gain = h_e_g;
    }

    double calc_combined_steer()
    {
        double purePursuitSteering = calc_PP_Steer();
        PP_steer = clip(purePursuitSteering, -deltaMax, deltaMax);

        if (cb_data_->get_odom_sub_flag())
        {
            // stanley preview
            double preview_steer = this->calc_stanly_preview();
            preview_steer = clip(preview_steer, -deltaMax, deltaMax);
            // stanley
            double now_stanley_steer = calc_stanly_steer();
            now_stanley_steer = clip(now_stanley_steer, -deltaMax, deltaMax);

            float sum_preview_gain = 0.0;
            for (size_t i = 0; i < preview_gain.size(); i++)
            {
                sum_preview_gain += preview_gain[i];
            }

            // sum both steer
            double both_steer = (1 - sum_preview_gain) * now_stanley_steer + preview_steer;
            this->stanly_steer = clip(both_steer, -deltaMax, deltaMax);

            cb_data_->set_down_odom_sub_flag();
        }

        // Combine the steering angles
        double combinedSteering = pp_weight_ * PP_steer + stanly_weight_ * stanly_steer;
        double combined_steer = clip(combinedSteering, -deltaMax, deltaMax);

        double com_lpf_steer = low_pass_filter(combined_steer, pre_com_steer, com_steer_alpha);
        pre_com_steer = com_lpf_steer;

        return -com_lpf_steer;
    }

    double calc_stanly_preview()
    {
        this->steers.clear();

        double sum_preview_steer = 0.0;
        // sync stanly n preview p gain
        preview_instance.set_stanly_gain(get_stanley_P_gain(), 0.0, 0.0);
        preview_instance.set_heading_gain(this->preview_h_e_gain);
        // preview_instance.set_heading_gain(1.0);

        cb_data_->calc_predict_odometry_for_stanley(preview_dt, preview_num);

        std::vector<Point> predict_pos = cb_data_->get_stanley_predict_pos(); // Fixed namespace
        std::vector<float> predict_yaw = cb_data_->get_stanley_predict_yaw(); // Fixed namespace

        for (int i = 0; i < preview_num; i++)
        {
            // calc_n_get_lat_error를 호출하면 closest point를 찾고 그 인덱스는
            // closest_index에 저장된다.
            double lat_error = cb_data_->calc_n_get_lat_error(predict_pos[i]);
            float tmp_speed = get_speed_in_stanley();
            double target_yaw = cb_data_->get_pd_path_yaw();

            preview_instance.set_stanly_data(tmp_speed, target_yaw, predict_yaw[i], lat_error);
            double steer = preview_instance.calc_stanly_steer();
            steer = clip(steer, -deltaMax, deltaMax);

            sum_preview_steer += steer * this->preview_gain[i];

            steers.push_back(steer);
        }

        return sum_preview_steer;
    }

    double get_PP_steer()
    {
        return PP_steer;
    }

    double get_stanly_steer()
    {
        return stanly_steer;
    }

    std::vector<double> get_preview_steers() // Fixed namespace
    {
        return this->steers;
    }
};

class ReversePurePursuit : public PurePursuit
{
private:
    double RPP_gain_ = 0.0;
    Point R_target_point_;
    double deltaMax = 0.491642;
    double reverse_yaw = 0.0;

    float R_ld_min_val = 3.0;
    float R_ld_max_val = 5.0;

public:
    ReversePurePursuit()
    {
    }

    void set_R_PP_data(float speed, double yaw, const Point &current_odom, std::vector<Point> path) // Fixed namespace
    {
        reverse_yaw = nomalize_angle(yaw - M_PI); // Fixed typo
        this->set_pp_data(speed, reverse_yaw, current_odom, path);
    }

    void set_R_pp_LD_threshold(float min, float max)
    {
        this->R_ld_min_val = min;
        this->R_ld_max_val = max;
    }

    void set_R_PP_gain(double rpp_gain, float lookahead)
    {
        this->RPP_gain_ = rpp_gain;
        this->set_pp_gain(rpp_gain, lookahead);
    }

    double calc_R_PP_steer()
    {
        this->R_target_point_ = this->calc_LA_point(R_ld_min_val, R_ld_max_val, true);

        float wheel_base = this->get_wheel_base();
        double tmp_x = R_target_point_.x;
        double tmp_y = R_target_point_.y;

        double la_dist = LA_distance(R_target_point_, true);

        double tmp_val = ((tmp_x / 2) - wheel_base) / tmp_y;
        double radius = la_dist * std::sqrt(tmp_val * tmp_val + 1 / 4); // Fixed namespace

        double numerator = 2 * wheel_base * tmp_y;
        double denominator = la_dist * la_dist * RPP_gain_;
        double angleToTarget = std::atan2(numerator, denominator);

        std::cout << std::endl // Fixed namespace
                  << std::endl
                  << std::endl
                  << std::endl;
        std::cout << "radius : " << radius << std::endl;

        double delta = angleToTarget;

        std::cout << "delta : " << delta << std::endl;

        double steer = clip(delta, -deltaMax, deltaMax);

        return -steer;
    }

    double get_reverse_yaw()
    {
        return this->reverse_yaw;
    }
};

#endif // LAT_CONTROLLERS_HPP
