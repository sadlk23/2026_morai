/***
 * 작성자: 정성윤
 * 설명: Local 관련 클래스 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef LOCAL_H_
#define LOCAL_H_

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include "kdtree.hpp"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include "path.hpp"
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <deque>
#include <sensor_msgs/NavSatFix.h>
#include "utility_function.hpp"

class Local
{
private:
    const std::string global_path_text_adress_;
    std::vector<int> mission_list_;
    std::unordered_map<int, std::string> mission_dic_;

    std::vector<std::vector<Path>> global_path_matrix_;

    /* 차량 상태 관련 변수 */
    double cur_car_utm_pos_[2] = {0, 0};
    double cur_car_yaw_ = 0;
    double cur_car_velocity_ = 0;
    double cur_car_utm_acceleration_[2] = {0, 0};

    /* Global Path 관련 변수 */
    size_t cur_mission_index_ = 0; // 현재 mission의 인덱스
    size_t cur_path_index_ = 0;    // 현재 path의 인덱스

    /* 조건 플래그 */
    bool utm_called_ = false;    // UTM 콜백 호출 플래그
    bool yaw_called_ = false;    // Yaw 콜백 호출 플래그
    bool serial_called_ = false; // serial 콜백 호출 플래그
    bool imu_called_ = false;    // imu 콜백 호출 플래그

    /* 좌표계 관련 변수 */
    bool rcs_mode_ = false; // 상대 좌표계 모드 플래그

    /* GPS 관련 변수 */
    std::deque<std::vector<double>> gps_queue;
    const int kGpsQueueSize_ = 5;
    const int kGpsAnomalMin_ = 2;
    const double kMaxSpeed = 30.0; // 비정상적인 속도의 임계값 (m/s)
    int nav_status_ = -3;          // NavStatus에 아예 없는 값으로 초기화

    /* 속도 관련 정보 변수 */
    double lo_yaw_rate_;
    const double car_width_ = 0.985;
    double center_speed_yaw_rate_ = 0.0;
    double pre_center_speed_yaw_rate_ = 0.0;
    double cur_acceleration_[2] = {0, 0};
    double pre_acceleration_[2] = {0, 0};

public:
    Local(std::string gp_text_adress, std::vector<int> &mission_list, std::unordered_map<int, std::string> &mission_dic);

    /* getter 함수 */
    const std::vector<int> &getRefMissionList() const;
    const std::unordered_map<int, std::string> &getRefMissionDic() const;
    const Path &getRefCurGlobalPath() const;
    const double *getAddCurCarUTMPos() const;
    const double getCurCarYaw() const;
    const double getCurCarVelocity() const;
    const double *getAddCurCarUTMAcceleration() const;
    size_t getCurMissionIndex() const;
    size_t getCurPathIndex() const;
    std::string getCurFileName() const;
    bool getRcsMode() const;
    double getCurYawRate() const;
    const int getNavStatus() const;

    /* setter 함수 */
    bool setPathfromTxt(size_t mission_index, size_t file_index); // Path 초기화 함수
    void setGlobalPathMatrix();                                   // global_path_matrix_ 초기화 함수
    void setCurYaw(double cur_yaw);
    void setRcsMode(bool rcs_mode);

    /* callback 함수 */
    void localCurUTMCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void localCurYawCallback(const std_msgs::Float64::ConstPtr &msg);
    void erpDataCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void localImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void localNavStatusCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    /* 기능 관련 함수 */
    bool changeNextPath();         // 다음 Path로 바꾸는 함수
    bool checkPathComplete();      // Path가 끝났는지 확인하는 함수
    bool changeNextMission();      // 다음 미션으로 바꾸는 함수
    bool checkMissionComplete();   // 미션 끝났는지 확인하는 함수
    size_t getCloseMissionIndex(); // 현재위치에서 가장 가까운 미션 인덱스 반환하는 함수
    void setCurMissionIndex();     // 현재위치에 대해 미션 인덱스를 설정하는 함수
    void updateCurKDTree();        // 현재 kd_tree_ 최신화 함수
    int getCurMissionNumber();     // 현재 미션 번호 반환 함수
    std::string getCurMission();   // 현재 미션 반환 함수
    template <typename T>
    std::vector<double> getCurClosestPos(const T(&pos)) const // 현재 path에서 가장 가까운 좌표 반환 함수
    {
        return global_path_matrix_[cur_mission_index_][cur_path_index_].getClosestPos(pos);
    }
    template <typename T>
    double getCurClosestDistance(const T(&pos)) const // 현재 path에서 가장 가까운 거리 반환 함수
    {
        return global_path_matrix_[cur_mission_index_][cur_path_index_].getClosestDistance(pos);
    }
    template <typename T>
    int getCurClosestIndex(const T(&pos)) const // 현재 path에서 가장 가까운 index 반환 함수
    {
        return global_path_matrix_[cur_mission_index_][cur_path_index_].getClosestIndex(pos);
    }
    bool callbacksIsCalled();               // 모든 콜백이 호출되는 것을 기다리는 함수
    Path &getRefNextGlobalPath();           // 다음 패스를 참조로 반환해주는 함수
    Path &getRefIdxPath(size_t path_index); // 인덱스의 패스를 참조로 반환해주는 함수
    void changeRcsMode();                   // 상대 좌표 모드로 변경하는 함수
    void changeAcsMode();                   // 절대 좌표 모드로 변경하는 함수

    /* 속도 관련 계산 함수 */
    double lowPassFilter(double val, double pre_val, float alpha);
    double calcCenterSpeed(double left_speed);

    /* gps 상태 관련 함수 */
    bool checkGpsNormal(); // gp가 정상상태인지 체크하는 함수

    /* test용 함수 */
    void printAllData() const; // 모든 값 출력 함수
};

#endif
