#ifndef DATA_MANAGE_HPP
#define DATA_MANAGE_HPP

#include <vector>
#include <iostream>
#include <cmath>     // For M_PI and math functions
#include <algorithm> // For std::max, std::min

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <ros/ros.h> // ROS1 header

#include <nav_msgs/Odometry.h> // ROS1 message types
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/PointStamped.h>

#include "PIDController.hpp" // ROS1 version - direct include

using namespace std;

/*
제작 : 예원태
문의 : solnox99@koreatech.ac.kr

설명 :
2023년 창작차 대회에서 사용한 Control 코드이다.
ROS에서 SUB 한 데이터를 처리해 주는 callback 함수를 정의하는 Class이다.

ROS1 버전으로 변환됨.
*/

class CallbackClass
{
private:
    Point lo_odom;
    bool odom_sub_flag;
    double lo_yaw;
    double lo_yaw_rate;
    double lo_beta;
    // ERP Data
    bool ERP_topic_published_;
    float control_mode;
    float e_stop;
    float gear;
    float speed;
    float steer;
    float brake;
    float enc;
    double car_width = 0.985;
    double cneter_speed_yaw_rate = 0.0;
    double pre_cneter_speed_yaw_rate = 0.0;
    bool serial_sub_flag;

    vector<Point> pl_local_path;     // relative coordinate
    vector<Point> pl_local_path_abs; // abs coordinate
    vector<double> pl_local_path_yaw;
    int pl_mission_num;
    float pl_control_switch;
    float pl_path_yaw;

    int closest_index;
    int pd_closest_index;
    double pridict_dist;
    double path_curvature = 0.0;
    double pre_path_curvature = 0.0;
    vector<Point> stanley_predict_pos;
    vector<float> stanley_predict_yaw;
    Point zero_point;

    Point AxisTrans_abs2rel(const Point &p)
    {
        Point rel;
        rel.x = (p.x - lo_odom.x) * cos(lo_yaw) + (p.y - lo_odom.y) * sin(lo_yaw);
        rel.y = -(p.x - lo_odom.x) * sin(lo_yaw) + (p.y - lo_odom.y) * cos(lo_yaw);
        return rel;
    }

public:
    CallbackClass()
    {
        // TODO: 생성자 작성 필요
        lo_odom.x = 0.0;
        lo_odom.y = 0.0;
        odom_sub_flag = false;

        ERP_topic_published_ = false;
        control_mode = 0.0;
        e_stop = 0.0;
        gear = 0.0;
        speed = 0.0;
        steer = 0.0;
        brake = 0.0;
        enc = 0.0;
        serial_sub_flag = false;

        closest_index = 0;
        pd_closest_index = 0;
        zero_point.x = 0.0;
        zero_point.y = 0.0;

        pl_control_switch = 0;
        pl_mission_num = -404; // 있을 수 없는 아무 숫자로 초기화해 줘야함.
    }

    Point AxisTrans_rel2abs(const Point &p)
    {
        Point rel;
        rel.x = p.x * cos(lo_yaw) - p.y * sin(lo_yaw) + lo_odom.x;
        rel.y = p.x * sin(lo_yaw) + p.y * cos(lo_yaw) + lo_odom.y;
        return rel;
    }

    // ROS1 callback signature: const boost::shared_ptr<const MessageType>& msg
    void lo_odom_cb(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        // info : [x, y], UTM coordinate
        this->lo_odom.x = msg->point.x;
        this->lo_odom.y = msg->point.y;
        this->odom_sub_flag = true;
    }

    void lo_yaw_cb(const std_msgs::Float64::ConstPtr &msg)
    {
        // info : radian, -pi ~ +pi
        this->lo_yaw = msg->data;
    }

    void lo_imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
    {
        // Extract yaw rate (z-axis angular velocity)
        this->lo_yaw_rate = msg->angular_velocity.z;

        this->cneter_speed_yaw_rate = low_pass_filter(this->lo_yaw_rate, this->pre_cneter_speed_yaw_rate, 0.7);
        this->pre_cneter_speed_yaw_rate = this->cneter_speed_yaw_rate;
        // lo_imu_cb()에 디버그 출력 추가
        // cout << "Raw yaw_rate: " << msg->angular_velocity.z << endl;
        // cout << "Filtered yaw_rate: " << this->cneter_speed_yaw_rate << endl;
        // cout << "Center speed: " << this->speed << endl;
    }

    void lo_beta_cb(const std_msgs::Float64::ConstPtr &msg)
    {
        this->lo_beta = msg->data;
    }

    void pl_local_path_cb(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if (msg->data.empty())
        {
            for (int i = 0; i < 20; i++)
            {
                cout << "시부레 뭔일이래??? : " << msg->data.size() << endl;
            }
            return;
        }

        // Check if the data size is divisible by 2
        if (msg->data.size() % 2 != 0)
        {
            cout << "Invalid number of data points" << endl;
            return;
        }
        vector<Point> points;
        vector<Point> points_abs;
        if (msg->data[0] == -82.82 || msg->data[1] == -82.82) // [-82.82, -82.82] -> 상대경로를 보내겠다는 시그널
        {
            //cout << endl
            //      << "상대 경로로 받고있음" << endl
            //      << endl;
            for (size_t i = 2; i < msg->data.size(); i += 2)
            {
                Point point{msg->data[i], msg->data[i + 1]};
                points.push_back(point);
            }
        }
        else
        {
            // Convert the received message to a 2D vector
            for (size_t i = 0; i < msg->data.size(); i += 2)
            {
                Point point{msg->data[i], msg->data[i + 1]};

                points_abs.push_back(point);

                Point pl_rel = AxisTrans_abs2rel(point);
                points.push_back(pl_rel);
            }
            this->pl_local_path_abs = points_abs;
        }

        this->pl_local_path = points;
    }

    void pl_local_path_yaws_cb(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if (msg->data.empty())
        {
            for (int i = 0; i < 20; i++)
            {
                cout << "yaw 길이가 0이랍니다. : " << msg->data.size() << endl;
            }
            return;
        }

        vector<double> yaws;

        for (size_t i = 0; i < msg->data.size(); i += 1)
        {
            yaws.push_back(msg->data[i]);
        }

        this->pl_local_path_yaw = yaws;
    }

    void pl_mission_num_cb(const std_msgs::Int16::ConstPtr &msg)
    {
        // info : TODO!!!
        this->pl_mission_num = msg->data;
    }

    void pl_control_switch_cb(const std_msgs::Float64::ConstPtr &msg)
    {
        // info : 0:STOP, 1:LOW SPEED, 2:NORMAL SPEED
        this->pl_control_switch = msg->data;
    }

    void pl_path_yaw_cb(const std_msgs::Float64::ConstPtr &msg)
    {
        // info : 0:STOP, 1:LOW SPEED, 2:NORMAL SPEED
        this->pl_path_yaw = msg->data;
    }

    void co_ERP_data_cb(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        this->control_mode = msg->data[0];
        this->e_stop = msg->data[1];
        this->gear = msg->data[2];
        this->speed = this->center_speed(msg->data[3]);
        // this->speed = msg->data[3];
        this->steer = msg->data[4];
        this->brake = msg->data[5];
        this->enc = msg->data[6];
        this->serial_sub_flag = true;
    }

    float center_speed(float left_speed)
    {
        double tmp_cs;
        if (!(std::isfinite(left_speed))) return 0.0;


        double cneter_yaw_rate = this->cneter_speed_yaw_rate;
        if (!std::isfinite(cneter_yaw_rate)) cneter_yaw_rate = 0.0;
   

        if (left_speed >= 0)
        {
           tmp_cs = left_speed + 0.5 * car_width * cneter_yaw_rate;
        }
        else 
        {
            tmp_cs = left_speed - 0.5 * car_width * cneter_yaw_rate;

        }

        return tmp_cs;
    }

    double calc_n_get_lat_error()
    {
        // 함수 오버로딩을 이용해 일반적인 상황과 stanly preview를 위한 상황을 구분한다.
        // 인자를 주지 않으면 이 함수가 호출
        // 인자를 주면 아래의 함수를 호출
        return calc_n_get_lat_error(this->zero_point);
    }

    double calc_n_get_lat_error(Point pridict_pose)
    {
        if (pl_local_path.empty())
        {
            cerr << "Error: The local path sub yet!!!!." << endl;
            return 404; // Return 404 to indicate an error.
        }
        else if (lo_odom.x == 0.0 || lo_odom.y == 0.0)
        {
            cerr << "Error: The odom UTM sub yet!!!!." << endl;
            return 404; // Return 404 to indicate an error.
        }

        closest_index = 0; // 계산 결과로 나오는 값인데 이 값도 사용된다.
        double min_distance = std::sqrt(std::pow(pl_local_path[0].x - pridict_pose.x, 2) + std::pow(pl_local_path[0].y - pridict_pose.y, 2));

        for (size_t i = 1; i < pl_local_path.size(); i++)
        {
            double distance = std::sqrt(std::pow(pl_local_path[i].x - pridict_pose.x, 2) + std::pow(pl_local_path[i].y - pridict_pose.y, 2));
            if (distance < min_distance)
            {
                closest_index = static_cast<int>(i);
                min_distance = distance;
            }
        }

        // lat_error의 좌 우를 구분함.
        double l_or_r = determine_side(pl_local_path[closest_index + 1], pridict_pose, pl_local_path[closest_index]);
        if (l_or_r > 0)
        {
            min_distance = min_distance * -1;
        }

        if (min_distance > 50)
        {
            cerr << "Error: lateral error is too large!!!!." << endl;
            cerr << " 경로와의 최소 거리 : " << min_distance << endl;
            return 404; // Return 404 to indicate an error.
        }
        return min_distance;
    }

    double calc_path_curvature(float time_delay = 0.0, float diff_s = 1.5)
    {
        float td = clip(time_delay, 0.1F, 2.0F);
        pridict_dist = td * clip(this->speed, 2.5F, 9.0F); // (s)*(m/s)
        double second_pridict_dist = pridict_dist + diff_s;
        int f_pri_yaw_index = 0;
        int s_pri_yaw_index = 0;

        double s = 0.0;        // 경로의 길이 (Frenet s)
        double min_dist = 1e9; // 아주 큰 값으로 초기화
        double second_min_dist = 1e9;

        for (size_t i = 1; i < pl_local_path.size(); i++)
        {
            double dx = pl_local_path[i].x - pl_local_path[i - 1].x;
            double dy = pl_local_path[i].y - pl_local_path[i - 1].y;
            s += std::sqrt(dx * dx + dy * dy); // 경로의 길이 누적

            double f_dist = abs(s - pridict_dist);
            double s_dist = abs(s - second_pridict_dist);

            if (f_dist < min_dist)
            {
                f_pri_yaw_index = static_cast<int>(i);
                min_dist = f_dist;
            }
            if (s_dist < second_min_dist)
            {
                s_pri_yaw_index = static_cast<int>(i);
                second_min_dist = s_dist;
            }
        }

        // TODO: 여기 path_yaw의 인덱스와 local_path의 인덱스가 다를 경우 값이 일치하지 않을 수 있음.
        double yaw_diff = abs(nomalize_angle(pl_local_path_yaw[f_pri_yaw_index] - pl_local_path_yaw[s_pri_yaw_index])); // Fixed typo

        double tmp_curv = yaw_diff / diff_s;
        this->path_curvature = low_pass_filter(tmp_curv, pre_path_curvature, 0.9);
        this->pre_path_curvature = this->path_curvature;

        return path_curvature;
    }

    void calc_predict_odometry_for_stanley(float dt, int n)
    {
        vector<Point> predict_pos;

        double dx = cos(lo_beta - lo_yaw) * this->speed * dt;
        double dy = sin(lo_beta - lo_yaw) * this->speed * dt;

        for (int i = 1; i <= n; i++)
        {
            Point tmp_odom;

            tmp_odom.x = dx * i;
            tmp_odom.y = dy * i;
            predict_pos.push_back(tmp_odom);
        }

        vector<float> predict_yaw;

        for (int i = 1; i <= n; i++)
        {
            float tmp_yaw = this->lo_yaw + this->lo_yaw_rate * dt;
            predict_yaw.push_back(tmp_yaw);
        }

        this->stanley_predict_pos = predict_pos;
        this->stanley_predict_yaw = predict_yaw;
    }

    Point get_odom()
    {
        return this->lo_odom;
    }

    bool get_odom_sub_flag()
    {
        return this->odom_sub_flag;
    }

    void set_down_odom_sub_flag()
    {
        this->odom_sub_flag = false;
    }

    double get_yaw()
    {
        return this->lo_yaw;
    }

    float get_speed()
    {
        return this->speed;
    }

    float get_steer()
    {
        return this->steer;
    }

    double get_pd_path_yaw()
    {
        if (this->pl_local_path_yaw.empty())
        {
            cout << "pl_local_path_yaw is empty!!!" << endl;
            return 404;
        }
        return this->pl_local_path_yaw[this->closest_index];
    }

    int get_pd_closest_index()
    {
        return this->pd_closest_index;
    }

    int get_mission_num()
    {
        return this->pl_mission_num;
    }

    float get_control_switch()
    {
        return this->pl_control_switch;
    }

    float get_path_yaw()
    {
        return this->pl_path_yaw;
    }

    vector<double> get_local_path_yaw()
    {
        return this->pl_local_path_yaw;
    }

    float get_yawrate()
    {
        return this->lo_yaw_rate;
    }

    double get_path_curvature()
    {
        return this->path_curvature;
    }

    bool get_serial_sub_flag()
    {
        return this->serial_sub_flag;
    }

    void set_down_serial_sub_flag()
    {
        this->serial_sub_flag = false;
    }

    vector<Point> get_relative_path()
    {
        return this->pl_local_path;
    }

    vector<Point> get_stanley_predict_pos()
    {
        return this->stanley_predict_pos;
    }

    vector<float> get_stanley_predict_yaw()
    {
        return this->stanley_predict_yaw;
    }
};

#endif // DATA_MANAGE_HPP
