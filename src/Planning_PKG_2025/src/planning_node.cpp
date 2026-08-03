/***
 * 작성자: 판단 파트
 * 설명: ROS1 catkin 기반 Planning Node 구현
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

// rosbag play <bag파일 이름> --loop

/* c++ 헤더파일 */
#include <iostream>
#include <vector>
#include <unordered_map>
#include <memory>
#include <stdexcept>
#include <cmath>

/* ROS1 헤더파일 */
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>

// /* matplotcpp 헤더파일 */
// #include <matplot/matplot.h>

/* 사용자 정의 헤더파일 */
#include "planning.hpp"
#include "local.hpp"
#include "vision.hpp"
#include "lidar.hpp"
#include "control.hpp"
#include "mission_data.hpp"
#include "cubic_spline.hpp"
#include "nlohmann/json.hpp"

class PlanningNode
{
private:
    /* subscriber 객체*/
    // ERP_SERIAL
    ros::Subscriber erp_data_sub_;

    // Local
    ros::Subscriber local_car_utm_sub_;
    ros::Subscriber local_car_yaw_sub_;
    ros::Subscriber local_car_imu_sub_;
    ros::Subscriber local_nav_status_sub_; // Tunnel

    // Lidar flag(bool)
    ros::Subscriber dynamic_stop_sub_;
    ros::Subscriber parallel_stop_sub_;
    ros::Subscriber tunnel_end_sub_;
    ros::Subscriber deli_start_sub_;
    // Lidar RCS(relative coordinate system)
    ros::Subscriber tunnel_wall_points_rc_sub_;
    ros::Subscriber tunnel_lane_points_rc_sub_;
    ros::Subscriber tunnel_center_rc_sub_;
    ros::Subscriber tunnel_ceiling_rc_sub_;
    ros::Subscriber tunnel_small_object_rc_sub_;
    // Lidar ACS(absolute coordinate system: UTM coordinate systme)
    ros::Subscriber small_object_utm_sub_;
    ros::Subscriber big_object_utm_sub_;
    ros::Subscriber parallel_park_points_utm_sub_;
    ros::Subscriber parallel_parking_edge_out_utm_sub_;
    ros::Subscriber u_turn_point_utm_sub_;
    ros::Subscriber cut_in_sub_;
    ros::Subscriber left_lane_clear_sub_;
    ros::Subscriber mission100_left_tracks_sub_;
    ros::Subscriber mission100_current_front_tracks_sub_;
    ros::Subscriber delivery_point_utm_sub_;

    // Vision
    ros::Subscriber traffic_light_sub_;
    ros::Subscriber distance_to_stopline_sub_;

    // 사선 주차
    ros::Subscriber angled_parking_all_point_utm_sub_;
    ros::Subscriber angled_parking_two_point_utm_sub_;

    // 시뮬레이션 구현부분
    ros::Subscriber dynamic_car_distance_sub_;
    ros::Subscriber dynamic_obstacle_pos_sub_;
    ros::Subscriber car_front_car_distance_sub_;
    // 시뮬레이션 구현부분

    /* publisher 객체 */
    ros::Publisher local_path_pos_pub_;
    ros::Publisher local_path_yaw_pub_;
    ros::Publisher local_path_k_pub_;
    ros::Publisher control_target_velocity_pub_;
    ros::Publisher mission_number_pub_;
    ros::Publisher mission100_current_lane_pub_;
    ros::Publisher tunnel_small_static_pub_;

    /* plot용 publisher 객체 */
    // rcs
    ros::Publisher plot_file_path_pub_;
    ros::Publisher plot_small_static_utm_pub_;
    ros::Publisher plot_all_path_pub_;
    ros::Publisher plot_parking_center_pub_;
    ros::Publisher plot_parking_edge_out_pub_;
    ros::Publisher plot_big_static_utm_pub_;
    ros::Publisher plot_u_turn_point_utm_pub_;
    ros::Publisher plot_global_path_point_pub_;
    // acs
    ros::Publisher plot_tunnel_L_wall_pub_;
    ros::Publisher plot_tunnel_R_wall_pub_;
    ros::Publisher plot_tunnel_small_static_pub_;
    ros::Publisher plot_tunnel_valid_obs_pub_;
    ros::Publisher plot_tunnel_evade_point_pub_;
    ros::Publisher plot_gps_mode_pub_;

    // 사선 주차
    ros::Publisher plot_angled_parking_all_point_pub_;
    ros::Publisher plot_angled_parking_two_point_pub_;

    // 정지 포인트
    ros::Publisher plot_stop_point_pub_;
    ros::Publisher plot_stop_distance_pub_;

    /* ROS private parameters (~map_name, ~mission_list, ~param_file) */
    std::string gp_text_adress_;
    std::vector<int> mission_list_;

    std::unordered_map<int, std::string> mission_dic_ =
        {{0, "safe_stop"},
         {1, "route_cruise"},
         {14, "static_obstacle_2026"},
         {23, "gps_shadow_tunnel_2026"},
         {24, "dynamic_obstacle_2026"},
         {31, "static_traffic_2026"},
         {60, "highway_entry"},
         {100, "highway_lane_change"}};

    /* Planning 클래스 스마트 포인터 */
    std::shared_ptr<Planning> planning_smtptr_;
    Planning *planning_ptr_;

    /* data 클래스들 관련 */
    Local *local_ptr_;
    Vision *vision_ptr_;
    Lidar *lidar_ptr_;
    Control *control_ptr_;
    MissionData *mission_data_ptr_;

    /* 타이머 동기화 함수(계속 실행되는 함수) 포인터 */
    ros::Timer timer_;

    /* start flag */
    bool node_start_flag_ = false;

    /* 파라미터 json */
    nlohmann::json json_data;

public:
    PlanningNode(ros::NodeHandle &nh)
    {
        ros::NodeHandle pnh("~");
        const std::string package_path = ros::package::getPath("planning_pkg_2025");
        if (package_path.empty())
        {
            throw std::runtime_error("planning_pkg_2025 package path를 찾을 수 없습니다.");
        }

        std::string map_name;
        pnh.param<std::string>("map_name", map_name, "map_2026");
        gp_text_adress_ = package_path + "/map/" + map_name;

        mission_list_ = {1, 31, 24, 1, 1, 1, 60, 100, 23, 1};
        pnh.getParam("mission_list", mission_list_);
        if (mission_list_.empty())
        {
            throw std::runtime_error("~mission_list 파라미터가 비어 있습니다.");
        }

        std::string param_file;
        pnh.param<std::string>("param_file", param_file, package_path + "/data/param.json");

        /* 파라미터 읽어오기 */
        std::ifstream inparam(param_file);
        if (!inparam.is_open())
        {
            throw std::runtime_error("Planning 파라미터 파일을 열 수 없습니다: " + param_file);
        }
        else
        {
            inparam >> json_data;
            inparam.close();

        }

        /* 클래스 인스턴스화 */
        planning_smtptr_ = std::make_shared<Planning>(nh, pnh, gp_text_adress_, mission_list_, mission_dic_, json_data["basic_data"], json_data["missions"]);
        planning_ptr_ = planning_smtptr_.get();
        local_ptr_ = &(planning_ptr_->getLocal());
        vision_ptr_ = &(planning_ptr_->getVision());
        lidar_ptr_ = &(planning_ptr_->getLidar());
        control_ptr_ = &(planning_ptr_->getControl());
        mission_data_ptr_ = &(planning_ptr_->getMissionData());

        // ROS2→ROS1: removed QoSProfile; ROS1 has no QoS concept

        /* --------------------------subscriber 구현--------------------------- */
        // ERP Data Subscriptions
        // ROS2→ROS1: replaced create_subscription
        erp_data_sub_ = nh.subscribe("/ERP/serial_data", 1, &Local::erpDataCallback, local_ptr_);

        // Local Subscriptions
        local_car_utm_sub_ = nh.subscribe("/Local/utm", 1, &Local::localCurUTMCallback, local_ptr_);
        local_car_yaw_sub_ = nh.subscribe("/Local/heading", 1, &Local::localCurYawCallback, local_ptr_);
        local_car_imu_sub_ = nh.subscribe("/imu", 1, &Local::localImuCallback, local_ptr_);
        local_nav_status_sub_ = nh.subscribe("/fix", 1, &Local::localNavStatusCallback, local_ptr_);

        // Lidar Flag Subscriptions
        dynamic_stop_sub_ = nh.subscribe("/LiDAR/dynamic_stop", 1, &Lidar::lidarDynamicCallback, lidar_ptr_);
        parallel_stop_sub_ = nh.subscribe("/LiDAR/park_ok2", 1, &Lidar::lidarParkingStopCallback, lidar_ptr_);
        deli_start_sub_ = nh.subscribe("/LiDAR/deli_start", 1, &Lidar::lidarDeliStartCallback, lidar_ptr_);
        tunnel_end_sub_ = nh.subscribe("/LiDAR/tunnel_end", 1, &Lidar::lidarTunnelEndCallback, lidar_ptr_);
        cut_in_sub_ = nh.subscribe("/LiDAR/CutInOk", 1, &Lidar::lidarCutInCallback, lidar_ptr_);
        left_lane_clear_sub_ = nh.subscribe(
            "/LiDAR/left_lane_clear", 1,
            &Lidar::lidarLeftLaneClearCallback, lidar_ptr_);
        mission100_left_tracks_sub_ = nh.subscribe(
            "/LiDAR/mission100_left_tracks", 1,
            &Lidar::lidarMission100LeftTracksCallback, lidar_ptr_);
        mission100_current_front_tracks_sub_ = nh.subscribe(
            "/LiDAR/mission100_current_front_tracks", 1,
            &Lidar::lidarMission100CurrentFrontTracksCallback, lidar_ptr_);
        // Lidar RCS(relative coordinate system) Subscriptions
        tunnel_ceiling_rc_sub_ = nh.subscribe("/LiDAR/ceiling_end", 1, &Lidar::lidarTunnelCeilingCallback, lidar_ptr_);
        tunnel_small_object_rc_sub_ = nh.subscribe("/LiDAR/object_cen", 1, &Lidar::lidarTunnelSmallStaticCallback, lidar_ptr_);
        tunnel_wall_points_rc_sub_ = nh.subscribe("/LiDAR/wall_dist", 1, &Lidar::lidarTunnelWallDistCallback, lidar_ptr_);
        tunnel_lane_points_rc_sub_ = nh.subscribe("/LiDAR/lane", 1, &Lidar::lidarTunnelLaneCallback, lidar_ptr_);
        tunnel_center_rc_sub_ = nh.subscribe("/LiDAR/center_lane", 1, &Lidar::lidarTunnelCenterCallback, lidar_ptr_);
        // Lidar ACS(absolute coordinate system: UTM coordinate systme) Subscriptions
        small_object_utm_sub_ = nh.subscribe("/Convert/small_object_UTM", 1, &Lidar::lidarSmallStaticCallback, lidar_ptr_);
        big_object_utm_sub_ = nh.subscribe("/Convert/big_object_UTM", 1, &Lidar::lidarBigStaticCallback, lidar_ptr_);
        parallel_park_points_utm_sub_ = nh.subscribe("/Convert/prl_points", 1, &Lidar::lidarParkingPointCallback, lidar_ptr_);
        parallel_parking_edge_out_utm_sub_ = nh.subscribe("/Convert/parking_edge", 1, &Lidar::lidarParkingEdgeOutCallback, lidar_ptr_);
        u_turn_point_utm_sub_ = nh.subscribe("/Convert/u_turn_point", 1, &Lidar::lidarUTurnPointCallback, lidar_ptr_);
        delivery_point_utm_sub_ = nh.subscribe("/Convert/deli_UTM", 1, &Lidar::lidarDelibaryFlagCallback, lidar_ptr_);

        // Vision Subscriptions
        traffic_light_sub_ = nh.subscribe("/Vision/traffic_sign", 1, &Vision::visionTrafficLightCallback, vision_ptr_);
        distance_to_stopline_sub_ = nh.subscribe("/Vision/stopline", 1, &Vision::visionStoplineCallback, vision_ptr_);

        // 사선 주차

        angled_parking_all_point_utm_sub_ = nh.subscribe("/Convert/angled_parking_all_point_UTM", 1, &Lidar::lidarAngledParkingAllPointCallback, lidar_ptr_);
        angled_parking_two_point_utm_sub_ = nh.subscribe("/Convert/angled_parking_two_point_UTM", 1, &Lidar::lidarAngledParkingTwoPointCallback, lidar_ptr_);

        // 시뮬레이션 구현 sub, pub

        dynamic_car_distance_sub_ = nh.subscribe("/LiDAR/car_dis", 1, &Lidar::lidarDynamicCarDistanceCallBack, lidar_ptr_);
        dynamic_obstacle_pos_sub_ = nh.subscribe(
            "/LiDAR/dynamic_obstacle_pos", 1,
            &Lidar::lidarDynamicObstaclePosCallback, lidar_ptr_);
        car_front_car_distance_sub_ = nh.subscribe(
            "/LiDAR/car_front_car_dis", 1,
            &Lidar::lidarCarFrontCarDistanceCallback, lidar_ptr_);

        // 시뮬레이션 구현 sub, pub
        /* --------------------------publisher 구현---------------------------- */
        // 제어에 pub 하는 publisher
        // ROS2→ROS1: replaced create_publisher
        local_path_pos_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/local_path", 1);
        local_path_yaw_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/path_yaw", 1);
        local_path_k_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/curvature", 1);
        control_target_velocity_pub_ = nh.advertise<std_msgs::Float64>("/Planning/target_velocity", 1);
        mission_number_pub_ = nh.advertise<std_msgs::Int16>("/Planning/mission", 1);
        mission100_current_lane_pub_ =
            nh.advertise<std_msgs::Int16>(
                "/Planning/mission100_current_lane", 1);

        tunnel_small_static_pub_ = nh.advertise<std_msgs::Bool>("/Planning/is_small_static", 1); // 원래 is_dynamic에서 is_small_static으로 변경

        // plot용 publisher
        plot_file_path_pub_ = nh.advertise<std_msgs::String>("/Planning/plot_file_path", 1);
        plot_small_static_utm_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/plot_small_static_utm", 1);
        plot_all_path_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/plot_all_path", 1);
        plot_parking_center_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/plot_parking_center", 1);
        plot_parking_edge_out_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/plot_parking_edge_out", 1);
        plot_big_static_utm_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/plot_big_static_utm", 1);
        plot_u_turn_point_utm_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/plot_u_turn_point_utm", 1);
        plot_global_path_point_pub_ =
            nh.advertise<std_msgs::Float64MultiArray>(
                "/Planning/plot_global_path_point", 1);
        // tunnel
        plot_gps_mode_pub_ = nh.advertise<std_msgs::Bool>("/Planning/plot_gps_mode", 1);
        plot_tunnel_L_wall_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/plot_tunnel_L_wall", 1);
        plot_tunnel_R_wall_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/plot_tunnel_R_wall", 1);
        plot_tunnel_small_static_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/plot_tunnel_small_static", 1);
        plot_tunnel_valid_obs_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/plot_tunnel_valid_obstacle", 1);
        plot_tunnel_evade_point_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/plot_tunnel_evade_point", 1);

        // 사선 주차
        plot_angled_parking_all_point_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/plot_angled_parking_all_point", 1);
        plot_angled_parking_two_point_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/plot_angled_parking_two_point", 1);

        // 정지 포인트
        plot_stop_point_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/Planning/plot_stop_point", 1);
        plot_stop_distance_pub_ = nh.advertise<std_msgs::Float64>("/Planning/plot_stop_distance", 1);

        /* time_spine 함수 실행 */
        // ROS2→ROS1: Timer changed, added 'event' argument
        timer_ = nh.createTimer(ros::Duration(0.01), &PlanningNode::timer_callback, this);
    }

    void timer_callback(const ros::TimerEvent &event)
    {
        if (!node_start_flag_)
        {
            // local_ptr_->waitAllCallbacks();
            if (local_ptr_->callbacksIsCalled())
            {
                node_start_flag_ = true;
                planning_ptr_->setDefaultStatus();
            }
            else
            {
                ROS_WARN_THROTTLE(1.0, "utm, heading, serial, imu 안들어옴");
            }
        }
        else
        {
            /* 다음 맵으로 변경, 끝났으면 종료 */
            if (!(planning_ptr_->judgeMapChange()))
            {
                // In ROS1, we can stop the timer
                timer_.stop();
                return;
            }

            /* 상태 표시 */
            planning_ptr_->printStatus();

            planning_ptr_->chooseFunc();

            /* 시각화 노드 publish 함수 */
            plotPathPubFunc();

            /* 인지, 제어 노드에 publish 함수 */
            publishFunc();
        }
    }

    void publishFunc()
    {
        std_msgs::Int16 mission_flag;
        std_msgs::Int16 mission100_current_lane;
        std_msgs::Float64MultiArray local_pos;
        std_msgs::Float64MultiArray local_k;
        std_msgs::Float64MultiArray local_yaw;
        std_msgs::Float64 target_velocity;

        mission_flag.data = local_ptr_->getCurMissionNumber();
        mission100_current_lane.data =
            mission_flag.data == 100
                ? planning_ptr_->getHighwayCurrentLaneIdx()
                : -1;

        const Path &local_path = planning_ptr_->getRefLocalPath();

        const std::vector<std::vector<double>> &local_pos_arr = local_path.getRefPosArr();
        const std::vector<double> &local_k_arr = local_path.getRefKArr();
        const std::vector<double> &local_yaw_arr = local_path.getRefYawArr();

        if (local_pos_arr.empty())
        {
            ROS_ERROR_THROTTLE(1.0, "Planning local path is empty");
            target_velocity.data = 0.0;
            mission_number_pub_.publish(mission_flag);
            mission100_current_lane_pub_.publish(mission100_current_lane);
            control_target_velocity_pub_.publish(target_velocity);
            return;
        }

        if (local_pos_arr[0][0] != -82.82)
        {
            const double *utm_point = local_ptr_->getAddCurCarUTMPos();
            int closest_index = local_path.getClosestIndex(utm_point);
            int end_index = closest_index + 300;
            if (static_cast<int>(local_pos_arr.size()) <= closest_index + 300)
            {
                end_index = static_cast<int>(local_pos_arr.size());
            }

            for (int i = closest_index; i < end_index; i++) // 내 위치기준 300개의 점을 선택
            {
                if (i < static_cast<int>(local_pos_arr.size()) && i < static_cast<int>(local_k_arr.size()) && i < static_cast<int>(local_yaw_arr.size()))
                {
                    local_pos.data.push_back(local_pos_arr[i][0]);
                    local_pos.data.push_back(local_pos_arr[i][1]);
                    local_k.data.push_back(local_k_arr[i]);
                    local_yaw.data.push_back(local_yaw_arr[i]);
                }
                else
                {
                    std::cerr << "인덱스 범위 초과: " << i << "\n";
                }
            }
        }
        else
        {
            for (std::size_t i = 0; i < local_pos_arr.size(); ++i)
            {
                local_pos.data.push_back(local_pos_arr[i][0]);
                local_pos.data.push_back(local_pos_arr[i][1]);
            }
            for (double curvature : local_k_arr)
            {
                local_k.data.push_back(curvature);
            }
            for (double yaw : local_yaw_arr)
            {
                local_yaw.data.push_back(yaw);
            }
        }

        // 박성환 수정영역
        std::cout << "[DEBUG] control_ptr_->getTargetVelocity() = " << control_ptr_->getTargetVelocity() << std::endl;
        target_velocity.data = control_ptr_->getTargetVelocity();
        

        if (mission_flag.data == 23)
        {
            std_msgs::Bool small_static_mode_msg;
            small_static_mode_msg.data = lidar_ptr_->getSmallStaticMode();
            tunnel_small_static_pub_.publish(small_static_mode_msg);
        }

        mission_number_pub_.publish(mission_flag);
        mission100_current_lane_pub_.publish(
            mission100_current_lane);
        local_path_pos_pub_.publish(local_pos);
        local_path_yaw_pub_.publish(local_yaw);
        local_path_k_pub_.publish(local_k);

        //박성환 수정영역
        std::cout << "[DEBUG] /Planning/target_velocity publish: target_velocity.data = " << target_velocity.data << std::endl;
        control_target_velocity_pub_.publish(target_velocity);
    }

    void plotPathPubFunc()
    {
        /* plot publish 위한 포인터 */
        std_msgs::String file_path;
        std_msgs::Float64MultiArray small_static_points;
        std_msgs::Float64MultiArray all_path_pos;
        std_msgs::Float64MultiArray uturn_point;
        std_msgs::Float64MultiArray parking_center;
        std_msgs::Float64MultiArray parking_edge_out;
        std_msgs::Float64MultiArray big_static_points;
        std_msgs::Float64MultiArray global_path_points;

        // 사선 주차
        std_msgs::Float64MultiArray angled_parking_all_point;
        std_msgs::Float64MultiArray angled_parking_two_point;

        // 정지 포인트
        std_msgs::Float64MultiArray stop_point;
        std_msgs::Float64 stop_distance;

        /* publish 준비 */
        std::string file_p = gp_text_adress_;
        file_p.append("/");
        file_p.append(local_ptr_->getCurFileName());
        file_path.data = file_p;

        const std::vector<std::vector<double>> &ref_small_static_points = lidar_ptr_->getRefSmallStaticPoint();
        for (size_t i = 0; i < ref_small_static_points.size(); ++i)
        {
            small_static_points.data.push_back(ref_small_static_points[i][0]);
            small_static_points.data.push_back(ref_small_static_points[i][1]);
        }

        /* frenet_frame_trajectory plot test */
        const std::vector<std::vector<std::vector<double>>> &ref_all_path_vec = planning_ptr_->getAllPathVector();
        ROS_INFO_THROTTLE(1.0, "trajectory size: %zu", ref_all_path_vec.size());
        for (size_t i = 0; i < ref_all_path_vec.size(); ++i)
        {
            for (size_t j = 0; j < ref_all_path_vec[i].size(); ++j)
            {
                all_path_pos.data.push_back(ref_all_path_vec[i][j][0]);
                all_path_pos.data.push_back(ref_all_path_vec[i][j][1]);
            }
        }

        const std::vector<double> &ref_parking_center = lidar_ptr_->getRefParkingPoint();
        if (ref_parking_center.size() > 0)
        {
            parking_center.data.push_back(ref_parking_center[0]);
            parking_center.data.push_back(ref_parking_center[1]);
        }

        const std::vector<std::vector<double>> &ref_parking_edge_out = lidar_ptr_->getRefParkingEdgeOut();
        if (ref_parking_edge_out.size() > 0)
        {
            for (size_t i = 0; i < ref_parking_edge_out.size(); ++i)
            {
                parking_edge_out.data.push_back(ref_parking_edge_out[i][0]);
                parking_edge_out.data.push_back(ref_parking_edge_out[i][1]);
            }
        }

        const std::vector<std::vector<double>> &ref_uturn_point = lidar_ptr_->getRefUTurnPoint();
        if (ref_uturn_point.size() > 0)
        {
            for (size_t i = 0; i < ref_uturn_point.size(); ++i)
            {
                uturn_point.data.push_back(ref_uturn_point[i][0]);
                uturn_point.data.push_back(ref_uturn_point[i][1]);
            }
        }

        const std::vector<double> &ref_big_static_points = lidar_ptr_->getRefBigStaticPoint();
        for (size_t i = 0; i < ref_big_static_points.size(); ++i)
            big_static_points.data.push_back(ref_big_static_points[i]);

        // 사선 주차
        const std::vector<std::vector<double>> &ref_angled_parking_all_point = lidar_ptr_->getRefAngledParkingAllPoint();
        for (size_t i = 0; i < ref_angled_parking_all_point.size(); ++i)
        {
            angled_parking_all_point.data.push_back(ref_angled_parking_all_point[i][0]);
            angled_parking_all_point.data.push_back(ref_angled_parking_all_point[i][1]);
        }

        const std::vector<std::vector<double>> &ref_angled_parking_two_point = lidar_ptr_->getRefAngledParkingTwoPoint();
        for (size_t i = 0; i < ref_angled_parking_two_point.size(); ++i)
        {
            angled_parking_two_point.data.push_back(ref_angled_parking_two_point[i][0]);
            angled_parking_two_point.data.push_back(ref_angled_parking_two_point[i][1]);
        }

        // 정지 포인트
        const std::vector<double> &ref_stop_point = mission_data_ptr_->getRefStopPoint();
        for (size_t i = 0; i < ref_stop_point.size(); ++i)
        {
            stop_point.data.push_back(ref_stop_point[i]);
        }

        stop_distance.data = mission_data_ptr_->getStopDistance();

        const int mission_num = local_ptr_->getCurMissionNumber();
        if (mission_num == 100)
        {
            global_path_points.data =
                planning_ptr_->getHighwayProjectedTriggerPoints();
        }

        // 기존
        plot_file_path_pub_.publish(file_path);
        plot_small_static_utm_pub_.publish(small_static_points);
        plot_all_path_pub_.publish(all_path_pos);
        plot_parking_center_pub_.publish(parking_center);
        plot_parking_edge_out_pub_.publish(parking_edge_out);
        plot_big_static_utm_pub_.publish(big_static_points);
        plot_u_turn_point_utm_pub_.publish(uturn_point);
        plot_global_path_point_pub_.publish(global_path_points);

        // 사선 주차
        plot_angled_parking_all_point_pub_.publish(angled_parking_all_point);
        plot_angled_parking_two_point_pub_.publish(angled_parking_two_point);

        // 배달
        plot_stop_point_pub_.publish(stop_point);
        plot_stop_distance_pub_.publish(stop_distance);

        // 터널일때만 publish
        if (mission_num == 23)
        {
            std_msgs::Bool is_gps_msg;
            is_gps_msg.data = !(local_ptr_->getRcsMode());
            plot_gps_mode_pub_.publish(is_gps_msg);

            std_msgs::Float64MultiArray L_wall_msg;
            std_msgs::Float64MultiArray R_wall_msg;

            const std::vector<double> &L_wall = lidar_ptr_->getRefTunnelLWall();
            const std::vector<double> &R_wall = lidar_ptr_->getRefTunnelRWall();

            for (size_t i = 0; i < L_wall.size(); i++)
            {
                L_wall_msg.data.emplace_back(L_wall[i]);
            }
            for (size_t i = 0; i < R_wall.size(); i++)
            {
                R_wall_msg.data.emplace_back(R_wall[i]);
            }

            plot_tunnel_L_wall_pub_.publish(L_wall_msg);
            plot_tunnel_R_wall_pub_.publish(R_wall_msg); // 여기까지 벽 퍼블리시

            // 터널 정적 소형
            std_msgs::Float64MultiArray tunnel_small_msg;
            const std::vector<double> &ref_tunnel_small_static = lidar_ptr_->getRefTunnelSmallStaticPoint();
            for (size_t i = 0; i < ref_tunnel_small_static.size(); i++)
            {
                tunnel_small_msg.data.push_back(ref_tunnel_small_static[i]);
            }
            plot_tunnel_small_static_pub_.publish(tunnel_small_msg);

            // 터널 유효점과 회피점 플롯
            std_msgs::Float64MultiArray tunnel_valid_point_msg;
            std_msgs::Float64MultiArray tunnel_evade_point_msg;
            const std::vector<double> &ref_tunnel_valid_point = lidar_ptr_->getRefTunnelValidObstacle();
            const std::vector<double> &ref_tunnel_evade_point = lidar_ptr_->getRefTunnelEvadePoint();
            for (size_t i = 0; i < ref_tunnel_valid_point.size(); i++)
            {
                tunnel_valid_point_msg.data.push_back(ref_tunnel_valid_point[i]);
            }
            for (size_t i = 0; i < ref_tunnel_evade_point.size(); i++)
            {
                tunnel_evade_point_msg.data.push_back(ref_tunnel_evade_point[i]);
            }
            plot_tunnel_valid_obs_pub_.publish(tunnel_valid_point_msg);
            plot_tunnel_evade_point_pub_.publish(tunnel_evade_point_msg);
        }
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;
    try
    {
        PlanningNode node(nh);
        ros::spin();
        return 0;
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("planning_node 초기화 실패: %s", e.what());
        return 1;
    }
}
