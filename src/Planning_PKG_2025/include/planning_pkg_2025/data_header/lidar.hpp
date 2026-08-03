/***
 * 작성자: 박용현
 * 설명: Lidar 클래스 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef LIDAR_H_
#define LIDAR_H_

#include <std_msgs/Float32.h>
#include <ros/time.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <algorithm>
#include <climits>
#include "nlohmann/json.hpp"

#include "utility_function.hpp"
#include "static_clusters.hpp"
#include "dynamic_clusters.hpp"

struct Mission100LeftTrack
{
    int id = 0;
    double center_x = 0.0;
    double center_y = 0.0;
    double center_z = 0.0;
    double min_x = 0.0;
    double max_x = 0.0;
    double relative_velocity_x = 0.0;
    int hit_count = 0;
    int missed_scans = 0;
    bool velocity_valid = false;
};

class Lidar
{
private:
    // 정적 소형
    StaticPosCluster small_static_cluster_;

    // 동적
    bool is_dynamic_stop_ = false;
    std::vector<double> dynamic_obstacle_pos_;
    std::uint64_t dynamic_obstacle_generation_ = 0;
    bool tunnel_small_static_mode_ = false;

    // 정적 대형
    std::vector<double> big_static_point_; // [x_left, y_left, x_right, x_right]
    StaticPosCluster big_static_cluster_;

    // 주차
    std::vector<double> parking_center_point;
    StaticPosCluster parking_point_cluster_;           // 주차 칸 중심점 하나
    std::vector<std::vector<double>> parking_edge_out; // 주차 꼭짓점 4개
    bool is_parking_stop_ = false;

    //배달
    bool deli_start_ = false;

    // 터널
    std::vector<double> tunnel_small_static_point_ = {};
    std::vector<double> tunnel_wall_ = {};
    std::vector<double> L_wall_ = {};
    std::vector<double> R_wall_ = {};
    std::vector<double> tunnel_valid_obstacle_ = {};
    std::vector<double> tunnel_evade_point_ = {};
    double Dynamic_car_distance_ = -1.0; // /LiDAR/car_dis 수신 전에는 거리 제어 미적용
    double car_front_car_distance_ = 0.0; // 미션 100 전용 선행 차량 거리
    bool car_front_car_distance_received_ = false;
    double car_front_car_distance_timeout_sec_ = 0.2;
    std::chrono::steady_clock::time_point car_front_car_distance_update_time_;
    double L_len_ = -1; // 길이를 라이다쪽에 저장할 이유가?
    double R_len_ = -1;
    std::vector<double> tunnel_lane_ = {};
    double tunnel_ceiling_ = 0;
    std::vector<double> tunnel_center_ = {};
    bool is_tunnel_end_ = false;
    int end_count_ = 0;

    // 유턴
    StaticPosCluster u_turn_point_cluster_;

    // 끼어들기
    bool cut_in_ok_ = false;

    // 미션 100 좌측 차선
    bool left_lane_clear_ = false;
    bool left_lane_clear_received_ = false;
    double left_lane_clear_timeout_sec_ = 0.2;
    std::chrono::steady_clock::time_point left_lane_clear_update_time_;

    struct Mission100TrackBuffer
    {
        std::vector<Mission100LeftTrack> tracks;
        bool received = false;
        bool valid = false;
        double timeout_sec = 0.3;
        std::chrono::steady_clock::time_point update_time;
        ros::Time source_stamp;
    };

    Mission100TrackBuffer mission100_left_track_buffer_;
    Mission100TrackBuffer mission100_current_front_track_buffer_;

    bool getMission100Tracks(
        const Mission100TrackBuffer &buffer,
        std::vector<Mission100LeftTrack> &tracks) const;
    void updateMission100Tracks(
        const std_msgs::Float64MultiArray::ConstPtr &msg,
        Mission100TrackBuffer &buffer);
    static void clearMission100Tracks(
        Mission100TrackBuffer &buffer);

    // 배달
    std::vector<double> delivery_flag_a;
    std::vector<double> delivery_flag_b; // TODO: 형식 인지랑 상의
    StaticClassCluster delivery_flag_a_cluster_;
    StaticClassCluster delivery_flag_b_cluster_;

    // 사선 주차
    StaticPosCluster angled_parking_all_cluster_;
    StaticPosCluster angled_parking_two_cluster_;

public:
    /* 생성자 */
    // 클러스터링하는 기준 epsilon, min_point_size, max_point_size 조정 필요!!!!
    Lidar(nlohmann::json &param) : small_static_cluster_(param["small_static_eps"], param["small_static_min"], param["small_static_max"]),
                                   big_static_cluster_(param["big_static_eps"], param["big_static_min"], param["big_static_max"]),
                                   parking_point_cluster_(param["parking_point_eps"], param["parking_point_min"], param["parking_point_max"], 0.9999),
                                   delivery_flag_a_cluster_(param["delivery_flag_a_eps"], param["delivery_flag_a_min"], param["delivery_flag_a_max"]),
                                   delivery_flag_b_cluster_(param["delivery_flag_b_eps"], param["delivery_flag_b_min"], param["delivery_flag_b_max"]),
                                   u_turn_point_cluster_(param["u_turn_point_eps"], param["u_turn_point_min"], param["u_turn_point_max"]),
                                   angled_parking_all_cluster_(param["angled_parking_eps"], param["angled_parking_min"], param["angled_parking_max"]),
                                   angled_parking_two_cluster_(param["angled_parking_eps"], param["angled_parking_min"], param["angled_parking_max"])
    {
        left_lane_clear_timeout_sec_ =
            param.value("left_lane_clear_timeout_sec", 0.2);
        mission100_left_track_buffer_.timeout_sec =
            param.value("mission100_left_tracks_timeout_sec", 0.3);
        mission100_current_front_track_buffer_.timeout_sec =
            param.value(
                "mission100_current_front_tracks_timeout_sec", 0.3);
        car_front_car_distance_timeout_sec_ =
            param.value("car_front_car_distance_timeout_sec", 0.2);
        if (!std::isfinite(car_front_car_distance_timeout_sec_) ||
            car_front_car_distance_timeout_sec_ <= 0.0)
        {
            car_front_car_distance_timeout_sec_ = 0.2;
        }
        if (!std::isfinite(
                mission100_left_track_buffer_.timeout_sec) ||
            mission100_left_track_buffer_.timeout_sec <= 0.0)
        {
            mission100_left_track_buffer_.timeout_sec = 0.3;
        }
        if (!std::isfinite(
                mission100_current_front_track_buffer_.timeout_sec) ||
            mission100_current_front_track_buffer_.timeout_sec <= 0.0)
        {
            mission100_current_front_track_buffer_.timeout_sec = 0.3;
        }
    }

    /* getter 함수 */

    // 시뮬레이션 구현부분
    const double &getRefDynamicCarDistance() const
    {
        return Dynamic_car_distance_;
    }

    const std::vector<double> &getRefDynamicObstaclePos() const
    {
        return dynamic_obstacle_pos_;
    }

    std::uint64_t getDynamicObstacleGeneration() const
    {
        return dynamic_obstacle_generation_;
    }

    double getRefCarFrontCarDistance() const
    {
        if (!car_front_car_distance_received_)
        {
            return 0.0;
        }

        const double elapsed_sec =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() -
                car_front_car_distance_update_time_)
                .count();
        if (elapsed_sec > car_front_car_distance_timeout_sec_)
        {
            return 0.0;
        }
        return car_front_car_distance_;
    }

    void setDynamicCarDistance(double distance)
    {
        Dynamic_car_distance_ = distance;
    }

    void lidarDynamicCarDistanceCallBack(const std_msgs::Float32::ConstPtr& msg)
    {
        if (std::isfinite(msg->data)) {
            Dynamic_car_distance_ = msg->data;
            std::cout << "콜백으로부터 값 들어옴: " << Dynamic_car_distance_ << std::endl;

        } 

        else {
            Dynamic_car_distance_ = 25;
            std::cout << Dynamic_car_distance_ << endl;
            std::cout << "거리값이 유효하지 않습니다." << std::endl;
        }
    }

    void lidarDynamicObstaclePosCallback(
        const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        dynamic_obstacle_pos_.clear();
        dynamic_obstacle_pos_.reserve(msg->data.size());
        for (std::size_t index = 0; index + 1 < msg->data.size(); index += 2)
        {
            const double x = msg->data[index];
            const double y = msg->data[index + 1];
            if (std::isfinite(x) && std::isfinite(y))
            {
                dynamic_obstacle_pos_.push_back(x);
                dynamic_obstacle_pos_.push_back(y);
            }
        }

        ++dynamic_obstacle_generation_;
        if (dynamic_obstacle_generation_ == 0)
        {
            dynamic_obstacle_generation_ = 1;
        }
    }

    void lidarCarFrontCarDistanceCallback(
        const std_msgs::Float32::ConstPtr &msg)
    {
        if (std::isfinite(msg->data) && msg->data >= 0.0f)
        {
            car_front_car_distance_ = msg->data;
        }
        else
        {
            car_front_car_distance_ = 0.0;
        }
        car_front_car_distance_received_ = true;
        car_front_car_distance_update_time_ =
            std::chrono::steady_clock::now();
    }
    // 시뮬레이션 구현부분 끝
    

    const std::vector<double> &getRefTunnelValidObstacle() const
    {
        return tunnel_valid_obstacle_;
    }

    const std::vector<double> &getRefTunnelEvadePoint() const
    {
        return tunnel_evade_point_;
    }

    const std::vector<std::vector<double>> &getRefSmallStaticPoint() const
    {
        return small_static_cluster_.getRefClusterTrustPoints();
    }

    const std::vector<double> &getRefBigStaticPoint() const
    {
        return big_static_point_;
    }

    const std::vector<double> &getRefParkingPoint() const
    {
        // return parking_point_cluster_.getRefClusterTrustPoints()[0];
        return parking_center_point;
    }

    const std::vector<std::vector<double>> &getRefParkingEdgeOut() const
    {
        return parking_edge_out;
    }

    bool getIsParkingStop() const
    {
        return is_parking_stop_;
    }

    bool getDeliStart() const
    {
        return deli_start_;
    }

    const bool getIsDynamicStop() const
    {
        return is_dynamic_stop_;
    }

    const bool getCutInOk() const
    {
        return cut_in_ok_;
    }

    bool getLeftLaneClear() const
    {
        if (!left_lane_clear_ || !left_lane_clear_received_ ||
            left_lane_clear_timeout_sec_ <= 0.0)
        {
            return false;
        }

        const double elapsed_sec =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() -
                left_lane_clear_update_time_)
                .count();
        return elapsed_sec <= left_lane_clear_timeout_sec_;
    }

    bool getMission100LeftTracks(
        std::vector<Mission100LeftTrack> &tracks) const
    {
        return getMission100Tracks(
            mission100_left_track_buffer_, tracks);
    }

    bool getMission100CurrentFrontTracks(
        std::vector<Mission100LeftTrack> &tracks) const
    {
        return getMission100Tracks(
            mission100_current_front_track_buffer_, tracks);
    }

    const bool getSmallStaticMode()
    {
        return tunnel_small_static_mode_;
    }

    const std::vector<double> &getRefTunnelSmallStaticPoint() const
    {
        return tunnel_small_static_point_;
    }
    const std::vector<double> &getRefTunnelWall()
    {
        return tunnel_wall_;
    }
    const std::vector<double> &getRefTunnelLWall()
    {
        return L_wall_;
    }
    const std::vector<double> &getRefTunnelRWall()
    {
        return R_wall_;
    }
    const double getTunnelLLen()
    {
        return L_len_;
    }
    const double getTunnelRLen()
    {
        return R_len_;
    }
    const std::vector<double> &getRefTunnelLane()
    {
        return tunnel_lane_;
    }
    const double getTunnelCeiling()
    {
        return tunnel_ceiling_;
    }
    const std::vector<double> &getRefTunnelCenter()
    {
        return tunnel_center_;
    }
    const bool getIsTunnelEnd()
    {
        return is_tunnel_end_;
    }
    const std::vector<std::vector<double>> &getRefUTurnPoint()
    {
        return u_turn_point_cluster_.getRefClusterTrustPoints();
    }
    const std::vector<double> &getRefDeliveryFlagA()
    {
        // return delivery_flag_a_cluster_.getRefClusterTrustPoints()[0];
        return delivery_flag_a;
    }
    const std::vector<double> &getRefDeliveryFlagB()
    {
        return delivery_flag_b;
    }
    const std::vector<std::vector<double>> &getRefAllDeliveryFlagB()
    {
        return delivery_flag_b_cluster_.getRefClusterTrustPoints();
    }

    // 사선 주차
    const std::vector<std::vector<double>> &getRefAngledParkingAllPoint() const
    {
        return angled_parking_all_cluster_.getRefClusterTrustPoints();
    }

    const std::vector<std::vector<double>> &getRefAngledParkingTwoPoint() const
    {
        return angled_parking_two_cluster_.getRefClusterTrustPoints();
    }

    /* setter 함수 */
    void setSmallStaticMode(bool does_small_static_start)
    {
        tunnel_small_static_mode_ = does_small_static_start;
    }

    void setTunnelValidObstacle(std::vector<double> vec)
    {
        tunnel_valid_obstacle_ = vec;
    }

    void setTunnelEvadePoint(std::vector<double> vec)
    {
        tunnel_evade_point_ = vec;
    }

    /* callback 함수 */
    void lidarSmallStaticCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) // 정적소형
    {
        for (size_t i = 0; i < msg->data.size(); i += 2)
        {
            std::vector<double> point(2, 0);
            point[0] = msg->data[i];
            point[1] = msg->data[i + 1];
            small_static_cluster_.update(point);
        }
    }

    void lidarDynamicCallback(const std_msgs::Bool::ConstPtr &msg) // 동적 장애물
    {
        is_dynamic_stop_ = msg->data;
    }

    void lidarBigStaticCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) // 정적 대형
    {
        // for (size_t i = 0; i < msg->data.size(); i += 2)
        // {
        //     std::vector<double> point(2, 0);
        //     point[0] = msg->data[i];
        //     point[1] = msg->data[i + 1];
        //     big_static_cluster_.update(point);
        // }
        // if (big_static_cluster_.getRefClusterTrustPoints().size() > 3)
        // {
        //     big_static_point_.clear();
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[2][0]);
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[2][1]);
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[3][0]);
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[3][1]);
        // }
        // else if (big_static_cluster_.getRefClusterTrustPoints().size() > 1)
        // {
        //     big_static_point_.clear();
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[0][0]);
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[0][1]);
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[1][0]);
        //     big_static_point_.push_back(big_static_cluster_.getRefClusterTrustPoints()[1][1]);
        // }
        big_static_point_ = msg->data;
    }

    void lidarDelibaryFlagCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) // 배달 미션
    {
        for (size_t i = 0; i < msg->data.size(); i += 3)
        {
            std::vector<double> point(3, 0);
            point[0] = msg->data[i];
            point[1] = msg->data[i + 1];
            point[2] = msg->data[i + 2];
            if (point[2] > 0 && point[2] < 4) // 1~3: a flag
            {
                delivery_flag_a_cluster_.update(point);
            }
            if (point[2] > 3 && point[2] < 7) // 4~6: b flag
            {
                point[2] -= 3;
                delivery_flag_b_cluster_.update(point);
            }
        }
        if (delivery_flag_a_cluster_.getRefClusterTrustPoints().size() > 0)
            delivery_flag_a = delivery_flag_a_cluster_.getRefClusterTrustPoints()[0];
        if (delivery_flag_b_cluster_.getRefClusterTrustPoints().size() > 0)
            delivery_flag_b = delivery_flag_b_cluster_.getRefClusterTrustPoints()[0]; // TODO : 인지랑 협의 후 수정 필요
    }

    void lidarParkingPointCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) // 평행주차 주차박스 중점 포인트
    {
        if (msg->data.size() > 0)
        {
            std::vector<double> point(2, 0);
            point[0] = msg->data[0];
            point[1] = msg->data[1];
            parking_point_cluster_.update(point);
        }
        if (parking_point_cluster_.getRefClusterTrustPoints().size() > 0)
            parking_center_point = parking_point_cluster_.getRefClusterTrustPoints()[0];
    }

    void lidarParkingEdgeOutCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) // 평행주차 주차박스 중점 포인트
    {
        parking_edge_out.clear();
        for (size_t i = 0; i < msg->data.size(); i += 2)
        {
            std::vector<double> point(2, 0);
            point[0] = msg->data[i];
            point[1] = msg->data[i + 1];
            parking_edge_out.push_back(point);
        }
    }

    void lidarParkingStopCallback(const std_msgs::Bool::ConstPtr &msg) // 평주차 정지 메시지
    {
        // is_parking_stop_ = msg->data;
        if (msg->data)
        {
            is_parking_stop_ = true;
        }
    }

    void lidarCutInCallback(const std_msgs::Bool::ConstPtr &msg) // 끼어들기 확인 메시지
    {
        if (msg->data)
        {
            cut_in_ok_ = msg->data;
        }
    }

    void lidarLeftLaneClearCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        left_lane_clear_ = msg->data;
        left_lane_clear_received_ = true;
        left_lane_clear_update_time_ = std::chrono::steady_clock::now();
    }

    void lidarMission100LeftTracksCallback(
        const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        updateMission100Tracks(
            msg, mission100_left_track_buffer_);
    }

    void lidarMission100CurrentFrontTracksCallback(
        const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        updateMission100Tracks(
            msg, mission100_current_front_track_buffer_);
    }

    void lidarDeliStartCallback(const std_msgs::Bool::ConstPtr &msg) // 배달 시작 메세지
    {
        if (msg->data)
        {
            deli_start_ = true;
        }
    }

    void lidarUTurnPointCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) // U턴 포인트
    {
        for (size_t i = 0; i < msg->data.size(); i += 2)
        {
            std::vector<double> point(2, 0);
            point[0] = msg->data[i];
            point[1] = msg->data[i + 1];
            u_turn_point_cluster_.update(point);
        }
    }

    // 사선 주차
    void lidarAngledParkingAllPointCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        for (size_t i = 0; i < msg->data.size(); i += 2)
        {
            std::vector<double> point(2, 0);
            point[0] = msg->data[i];
            point[1] = msg->data[i + 1];
            angled_parking_all_cluster_.update(point);
        }
    }

    void lidarAngledParkingTwoPointCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        for (size_t i = 0; i < msg->data.size(); i += 2)
        {
            std::vector<double> point(2, 0);
            point[0] = msg->data[i];
            point[1] = msg->data[i + 1];
            angled_parking_two_cluster_.update(point);
        }
    }

    // 터널
    void lidarTunnelSmallStaticCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if ((msg->data).size() > 0)
        {
            tunnel_small_static_point_.clear(); // 초기화 후 업데이트
            for (size_t i = 0; i < msg->data.size(); i++)
            {
                tunnel_small_static_point_.emplace_back(msg->data[i]);
            }
        }
        else
        {
            std::cout << "터널 내부 정적 소형 감지 X" << std::endl;
        }
    }

    void lidarTunnelWallDistCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if ((msg->data).size() > 7)
        {
            tunnel_wall_.clear(); // 초기화 후 업데이트
            for (size_t i = 0; i < msg->data.size(); i++)
            {
                tunnel_wall_.emplace_back(msg->data[i]);
            }

            std::vector<double> wall01;
            std::vector<double> wall02;
            vectorSlice(wall01, tunnel_wall_, 0, 4);
            vectorSlice(wall02, tunnel_wall_, 4, 8);

            if (wall01[1] > wall02[1])
            {
                L_wall_ = wall01;
                R_wall_ = wall02;
            }
            else
            {
                L_wall_ = wall02;
                R_wall_ = wall01;
            }

            L_len_ = std::hypot(L_wall_[2] - L_wall_[0], L_wall_[3] - L_wall_[1]);
            R_len_ = std::hypot(R_wall_[2] - R_wall_[0], R_wall_[3] - R_wall_[1]);
        }
    }

    void lidarTunnelLaneCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if (!(msg->data).empty())
        {
            tunnel_lane_.clear(); // 초기화 후 업데이트
            for (size_t i = 0; i < msg->data.size(); i++)
            {
                tunnel_lane_.emplace_back(msg->data[i]);
            }
        }
    }

    void lidarTunnelCeilingCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if (!(msg->data).empty())
        {
            tunnel_ceiling_ = msg->data[0];
        }
        else
        {
            tunnel_ceiling_ = 0;
        }
    }

    void lidarTunnelCenterCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        if (!(msg->data).empty())
        {
            tunnel_center_.clear(); // 초기화 후 업데이트

            for (size_t i = 0; i < msg->data.size(); i++)
            {
                tunnel_center_.emplace_back(msg->data[i]);
            }
        }
    }

    void lidarTunnelEndCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        // 50번 연속 true(토픽이 10hz로 돌기에 5초 대기)여야 끝으로 간주
        // 실험해보고 30번도 해보기
        end_count_ = std::clamp((msg->data ? end_count_ + 1 : 0), 0, INT_MAX - 1);
        is_tunnel_end_ = ((end_count_ >= 50) ? true : false);
    }

    /* 기능 관련 함수 */
    void clearAllClusters()
    {
        big_static_cluster_.clearCluster();
        small_static_cluster_.clearCluster();
        parking_point_cluster_.clearCluster();
        delivery_flag_a_cluster_.clearCluster();
        delivery_flag_b_cluster_.clearCluster();
        u_turn_point_cluster_.clearCluster();
        left_lane_clear_ = false;
        left_lane_clear_received_ = false;
        clearMission100Tracks(mission100_left_track_buffer_);
        clearMission100Tracks(
            mission100_current_front_track_buffer_);
        car_front_car_distance_ = 0.0;
        car_front_car_distance_received_ = false;
        dynamic_obstacle_pos_.clear();
        dynamic_obstacle_generation_ = 0;
    }

    /* test용 함수 */
    void printAllData()
    {
        std::cout << "############## small static point ##############\n";
        for (auto n : small_static_cluster_.getRefClusterTrustPoints())
            std::cout << n[0] << " " << n[1];
        std::cout << "\n";
        std::cout << "############## big static point ##############\n";
        for (auto n : big_static_point_)
            std::cout << n << " ";
        std::cout << "\n";
        std::cout << "############## parking point ##############\n";
        for (auto n : parking_center_point)
            std::cout << n << " ";
        std::cout << "\n";
        // std::cout << "############## tunnel small static point ##############\n";
        // for (auto n : tunnel_small_static_point_)
        //     std::cout << n[0] << " " << n[1];
        // std::cout << "\n";
    }
};

inline void Lidar::clearMission100Tracks(
    Mission100TrackBuffer &buffer)
{
    buffer.tracks.clear();
    buffer.received = false;
    buffer.valid = false;
    buffer.update_time = {};
    buffer.source_stamp = ros::Time(0);
}

inline bool Lidar::getMission100Tracks(
    const Mission100TrackBuffer &buffer,
    std::vector<Mission100LeftTrack> &tracks) const
{
    tracks.clear();
    if (!buffer.received ||
        !buffer.valid ||
        buffer.source_stamp.isZero() ||
        buffer.timeout_sec <= 0.0)
    {
        return false;
    }

    const double elapsed_sec =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() -
            buffer.update_time)
            .count();
    if (elapsed_sec > buffer.timeout_sec)
    {
        return false;
    }

    const ros::Time ros_now = ros::Time::now();
    if (!ros_now.isZero())
    {
        const double source_age_sec =
            (ros_now - buffer.source_stamp).toSec();
        if (std::abs(source_age_sec) > buffer.timeout_sec)
        {
            return false;
        }
    }

    tracks = buffer.tracks;
    return true;
}

inline void Lidar::updateMission100Tracks(
    const std_msgs::Float64MultiArray::ConstPtr &msg,
    Mission100TrackBuffer &buffer)
{
    constexpr std::size_t kMetadataSize = 5;
    constexpr std::size_t kTrackStride = 10;
    constexpr std::size_t kMaxTrackCount = 1000;

    const auto invalidate =
        [&buffer]()
        {
            clearMission100Tracks(buffer);
            buffer.received = true;
            buffer.update_time =
                std::chrono::steady_clock::now();
        };

    if (!msg || msg->data.size() < kMetadataSize)
    {
        invalidate();
        return;
    }

    const double version = msg->data[0];
    const double frame_valid = msg->data[1];
    const double track_count_value = msg->data[2];
    const double stamp_sec = msg->data[3];
    const double stamp_nsec = msg->data[4];
    if (!std::isfinite(version) ||
        std::abs(version - 1.0) > 1.0e-6 ||
        !std::isfinite(frame_valid) ||
        !std::isfinite(track_count_value) ||
        !std::isfinite(stamp_sec) ||
        !std::isfinite(stamp_nsec) ||
        stamp_sec < 0.0 ||
        stamp_nsec < 0.0 ||
        stamp_nsec >= 1.0e9 ||
        frame_valid < 0.5)
    {
        invalidate();
        return;
    }

    const long long rounded_stamp_sec =
        std::llround(stamp_sec);
    const long long rounded_stamp_nsec =
        std::llround(stamp_nsec);
    if (rounded_stamp_sec <= 0 ||
        rounded_stamp_sec >
            static_cast<long long>(
                std::numeric_limits<uint32_t>::max()) ||
        rounded_stamp_nsec < 0 ||
        rounded_stamp_nsec >= 1000000000LL ||
        std::abs(stamp_sec - rounded_stamp_sec) > 1.0e-6 ||
        std::abs(stamp_nsec - rounded_stamp_nsec) > 1.0e-6)
    {
        invalidate();
        return;
    }

    const long long rounded_track_count =
        std::llround(track_count_value);
    if (rounded_track_count < 0 ||
        static_cast<std::size_t>(rounded_track_count) >
            kMaxTrackCount ||
        std::abs(
            track_count_value -
            static_cast<double>(rounded_track_count)) > 1.0e-6)
    {
        invalidate();
        return;
    }

    const std::size_t track_count =
        static_cast<std::size_t>(rounded_track_count);
    if (msg->data.size() !=
        kMetadataSize + track_count * kTrackStride)
    {
        invalidate();
        return;
    }

    std::vector<Mission100LeftTrack> parsed_tracks;
    parsed_tracks.reserve(track_count);
    for (std::size_t index = 0; index < track_count; ++index)
    {
        const std::size_t offset =
            kMetadataSize + index * kTrackStride;
        bool fields_finite = true;
        for (std::size_t field = 0;
             field < kTrackStride;
             ++field)
        {
            fields_finite =
                fields_finite &&
                std::isfinite(msg->data[offset + field]);
        }
        if (!fields_finite)
        {
            invalidate();
            return;
        }

        Mission100LeftTrack track;
        track.id = static_cast<int>(
            std::llround(msg->data[offset]));
        track.center_x = msg->data[offset + 1];
        track.center_y = msg->data[offset + 2];
        track.center_z = msg->data[offset + 3];
        track.min_x = msg->data[offset + 4];
        track.max_x = msg->data[offset + 5];
        track.relative_velocity_x = msg->data[offset + 6];
        track.hit_count = static_cast<int>(
            std::llround(msg->data[offset + 7]));
        track.missed_scans = static_cast<int>(
            std::llround(msg->data[offset + 8]));
        track.velocity_valid =
            msg->data[offset + 9] >= 0.5;

        if (track.id <= 0 ||
            track.hit_count < 1 ||
            track.missed_scans < 0 ||
            track.min_x > track.max_x ||
            track.center_x < track.min_x - 1.0e-3 ||
            track.center_x > track.max_x + 1.0e-3)
        {
            invalidate();
            return;
        }
        parsed_tracks.push_back(track);
    }

    buffer.tracks = std::move(parsed_tracks);
    buffer.received = true;
    buffer.valid = true;
    buffer.update_time = std::chrono::steady_clock::now();
    buffer.source_stamp =
        ros::Time(
            static_cast<uint32_t>(rounded_stamp_sec),
            static_cast<uint32_t>(rounded_stamp_nsec));
}

#endif
