#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>

#include "control/callback_data_manage.hpp"
#include "control/PIDController.hpp"
#include "control/lat_control.hpp"
#include "control/lon_control.hpp"
#include "control/mission_param.hpp"

#include <chrono>
#include <memory>
#include <vector>

using namespace std;

bool IS_PRINT = true;

/*
 * ROS1 Noetic 변환 버전 (Ubuntu 20.04)
 */

float wheel_adaptation(float Yawrate_rps, float delta_offset_pre)
{
    float delta_offset = delta_offset_pre;
    if (Yawrate_rps > 0.01) delta_offset = delta_offset_pre - 0.0001;
    else if (Yawrate_rps < -0.01) delta_offset = delta_offset_pre + 0.0001;
    return delta_offset;
}

class ERPControl
{
private:
    std::shared_ptr<CallbackClass> callback_data_ptr;

    std::shared_ptr<CombinedSteer> lat_control_ptr;

    std::shared_ptr<ReversePurePursuit> R_lat_control_ptr;

    std::shared_ptr<LonController> lon_control_ptr;

    std::shared_ptr<ControlGainTuning> param_manage_ptr;

    std::shared_ptr<PIDController> pid_control_ptr; // 종방향 제어 PID 디버깅용
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    // 객체 관리
    CallbackClass *cb_data;
    CombinedSteer *lat_control;
    ReversePurePursuit *R_lat_control;
    LonController *lon_control;
    ControlGainTuning *param_manage;
    PIDController *pid_control;

    // Subscriber
    ros::Subscriber lo_odom_sub;
    ros::Subscriber lo_curr_sub;
    ros::Subscriber lo_imu__sub;
    ros::Subscriber lo_beta_sub;
    ros::Subscriber pl_loca_sub;
    ros::Subscriber pl_miss_sub;
    ros::Subscriber pl_cont_sub;
    ros::Subscriber pl_pyaws_sub;
    ros::Subscriber er_data_sub;

    // Publisher
    ros::Publisher ERP_data_pub;
    ros::Publisher tmp_data_pub;
    ros::Publisher Rel_path_pub;
    ros::Publisher control_data_pub;

    ros::Timer timer_;

    float pre_offset_delta = 0;
    GasAndBrake acc_val;
    ros::Time start_time_;

public:
    ERPControl() : pnh("~")
    {
        // <클레스들 인스턴스화>------------------------------------------------------
        callback_data_ptr = std::make_shared<CallbackClass>();
        cb_data = callback_data_ptr.get();

        lat_control_ptr = std::make_shared<CombinedSteer>(cb_data);
        lat_control = lat_control_ptr.get();
        R_lat_control_ptr = std::make_shared<ReversePurePursuit>();
        R_lat_control = R_lat_control_ptr.get();

        lon_control_ptr = std::make_shared<LonController>();
        lon_control = lon_control_ptr.get();

        std::string control_profile;
        pnh.param<std::string>("control_profile", control_profile, "B_Simul");
        param_manage_ptr = std::make_shared<ControlGainTuning>(cb_data, lat_control, R_lat_control,
                                                               lon_control, control_profile);
        param_manage = param_manage_ptr.get();

        pid_control_ptr = std::make_shared<PIDController>(-1, 1); // 종방향 제어 PID 디버깅용
        pid_control = pid_control_ptr.get(); // 종방향 제어 PID 디버깅용

        // <SUBSCRIBER> ROS1 방식
        lo_odom_sub = nh.subscribe("/Local/utm", 1, &CallbackClass::lo_odom_cb, cb_data);
        lo_curr_sub = nh.subscribe("/Local/heading", 1, &CallbackClass::lo_yaw_cb, cb_data);
        lo_imu__sub = nh.subscribe("/imu", 1, &CallbackClass::lo_imu_cb, cb_data);
        lo_beta_sub = nh.subscribe("/beta", 1, &CallbackClass::lo_beta_cb, cb_data);

        pl_loca_sub = nh.subscribe("/Planning/local_path", 1, &CallbackClass::pl_local_path_cb, cb_data);
        pl_miss_sub = nh.subscribe("/Planning/mission", 1, &CallbackClass::pl_mission_num_cb, cb_data);
        pl_cont_sub = nh.subscribe("/Planning/target_velocity", 1, &CallbackClass::pl_control_switch_cb, cb_data);
        pl_pyaws_sub = nh.subscribe("/Planning/path_yaw", 1, &CallbackClass::pl_local_path_yaws_cb, cb_data);

        er_data_sub = nh.subscribe("/ERP/serial_data", 1, &CallbackClass::co_ERP_data_cb, cb_data);

        // <PUBLISHER> ROS1 방식
        ERP_data_pub = nh.advertise<std_msgs::Float32MultiArray>("/Control/serial_data", 1);
        tmp_data_pub = nh.advertise<std_msgs::Float64MultiArray>("/Control/tmp_plot_val", 1);
        Rel_path_pub = nh.advertise<std_msgs::Float64MultiArray>("/Control/rel_path", 1);
        control_data_pub = nh.advertise<std_msgs::Float64>("/Control/center_speed", 1);

        // <TIMER> 100Hz (0.01초 주기)
        timer_ = nh.createTimer(ros::Duration(0.01), &ERPControl::timer_callback, this);
    }

    void timer_callback(const ros::TimerEvent&)
    {
        float curr_speed = cb_data->get_speed();
        float yaw = cb_data->get_yaw();
        Point odom = cb_data->get_odom();
        double lat_error = cb_data->calc_n_get_lat_error();
        double yaw_rate = cb_data->get_yawrate();
        double pd_path_yaw = cb_data->get_pd_path_yaw();
        double path_curature = 0.0, curv_1 = 0.0, curv_2 = 0.0, curv_3 = 0.0;

        if (lat_error == 404 || pd_path_yaw == 404) {
            ROS_WARN_THROTTLE(1, "404 Data Missing!!!");
            return;
        }

        // 제어 파라미터 갱신
        int changed_param_num = param_manage->set_mission_param();
        float cs = cb_data->get_control_switch();
        float steer = 0.0;
        int gear_state = 0;

        // 횡방향 제어 연산
        if (cs < 0.0) {
            gear_state = 2;
            R_lat_control->set_R_PP_data(curr_speed, yaw, odom, cb_data->get_relative_path());
            steer = R_lat_control->calc_R_PP_steer();
        } else {
            if(cb_data->get_mission_num() == 998 && abs(lat_error) < 0.05) {
                lat_control->set_stanley_integral_val(0.0);
            }          
            lat_control->set_stanly_data(curr_speed, pd_path_yaw, yaw, lat_error);
            lat_control->set_pp_data(curr_speed, yaw, odom, cb_data->get_relative_path());
            steer = lat_control->calc_combined_steer();
        }

        // 종방향 속도 설정
        if (cs == 0) {
            acc_val.gas = 0.0; acc_val.brake = 180;
            lon_control->set_lon_target_speed(0);
        } else if (cs > 0.0) {
            lon_control->set_lon_data(curr_speed);
            lon_control->set_lon_target_speed(cs);
        } else if (cs == -1) {
            lon_control->set_lon_data(curr_speed);
            lon_control->set_lon_target_speed(1.5);
        }

        // 곡률 기반 속도 감속 (협로 미션 등)
        if (cb_data->get_mission_num() == 998) {
            curv_1 = abs(cb_data->calc_path_curvature(0.1, 1.5));
            curv_2 = abs(cb_data->calc_path_curvature(0.3, 1.5));
            curv_3 = abs(cb_data->calc_path_curvature(0.6, 1.5));
            path_curature = max(curv_3, max(curv_1, curv_2));
            if(abs(path_curature) >= 0.1) param_manage->target_speed_reducing_by_curature(path_curature);
        }

        if (cs != -1 && abs(path_curature) < 0.1) {
            param_manage->target_speed_reducing_by_steer(steer);
        }

        // 종방향 가속/감속 연산
        if (cb_data->get_serial_sub_flag() && cs != 0) {
            acc_val = lon_control->calc_gas_n_brake();
            cb_data->set_down_serial_sub_flag();
        }

        // <ROS TOPIC PUB>
        ros::Time now = ros::Time::now();

        // 1. ERP Serial Data Publish
        std_msgs::Float32MultiArray ERP_data_msg;
        ERP_data_msg.data = {1.0, 0.0, (float)gear_state, acc_val.gas, steer, acc_val.brake, 0.0, (float)lon_control->get_lon_target_speed()};
        ERP_data_pub.publish(ERP_data_msg);

        // 2. Center Speed Publish
        std_msgs::Float64 center_speed_msg;
        center_speed_msg.data = cb_data->get_speed();
        control_data_pub.publish(center_speed_msg);

        // 3. Debug Plot Data Publish
        std_msgs::Float64MultiArray tmp_plot_msg;
        tmp_plot_msg.data.push_back(lat_error);
        tmp_plot_msg.data.push_back(lon_control->get_target_speed());
        tmp_plot_msg.data.push_back(curv_1);
        tmp_plot_msg.data.push_back(curv_2);
        tmp_plot_msg.data.push_back(path_curature);
        tmp_plot_msg.data.push_back(lat_control->get_stanley_integral_val());
        tmp_plot_msg.data.push_back(cb_data->get_speed());
        tmp_data_pub.publish(tmp_plot_msg);

        if (IS_PRINT) {
            cout << "==================" << endl;
            cout << "lat_error : " << lat_error << " | steer : " << steer << endl;
            cout << "curr_speed : " << curr_speed * 3.6 << "km/h | target : " << lon_control->get_target_speed() * 3.6 << "km/h" << endl;
            cout << "brake : " << acc_val.brake << " | gas : " << acc_val.gas << endl;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "erp_control");
    ERPControl erp_control_node;
    ros::spin();
    return 0;
}
