/***
 * 작성자: 정성윤
 * 설명: Local 관련 클래스 정의
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#include "local.hpp"

Local::Local(std::string gp_text_adress, std::vector<int> &mission_list, std::unordered_map<int, std::string> &mission_dic) : global_path_text_adress_{gp_text_adress}, global_path_matrix_(mission_list.size())
{
    mission_list_ = mission_list;
    mission_dic_ = mission_dic;
    setGlobalPathMatrix();
}

const std::vector<int> &Local::getRefMissionList() const
{
    return mission_list_;
}

const std::unordered_map<int, std::string> &Local::getRefMissionDic() const
{
    return mission_dic_;
}

const Path &Local::getRefCurGlobalPath() const
{
    return global_path_matrix_[cur_mission_index_][cur_path_index_];
}

const double *Local::getAddCurCarUTMPos() const
{
    return cur_car_utm_pos_;
}

const double Local::getCurCarYaw() const
{
    return cur_car_yaw_;
}

const double Local::getCurCarVelocity() const
{
    return cur_car_velocity_;
}

const double *Local::getAddCurCarUTMAcceleration() const
{
    return cur_car_utm_acceleration_;
}

size_t Local::getCurMissionIndex() const
{
    return cur_mission_index_;
}

size_t Local::getCurPathIndex() const
{
    return cur_path_index_;
}

std::string Local::getCurFileName() const
{
    const Path &global_path = getRefCurGlobalPath();
    std::string path_name = global_path.getPathName();
    return path_name;
}

bool Local::getRcsMode() const
{
    return rcs_mode_;
}

double Local::getCurYawRate() const
{
    return center_speed_yaw_rate_;
}

bool Local::changeNextPath() // 다음 패스로 바꾸는 함수
{
    if (cur_path_index_ < global_path_matrix_[cur_mission_index_].size() - 1)
    {
        cur_path_index_++;
        return true;
    }
    else
    {
        std::cout << "더 이상 패스 없음!\n";
        return false;
    }
}

bool Local::checkPathComplete() // path 끝났는지 확인하는 함수
{
    /* path index size는 최소 340개 (약 17m)
        최소 20m이상을 기준으로 맵을 따는 것을 권장
    */
    const std::vector<std::vector<double>> &pos_arr = global_path_matrix_[cur_mission_index_][cur_path_index_].getRefPosArr();
    int cur_index = global_path_matrix_[cur_mission_index_][cur_path_index_].getClosestIndex(cur_car_utm_pos_);

    return ((pos_arr.size() - cur_index) < 340) ? true : false;
}

bool Local::changeNextMission() // 다음 미션으로 바꾸는 함수
{
    if (cur_mission_index_ < global_path_matrix_.size() - 1)
    {
        cur_mission_index_++;
        cur_path_index_ = 0;
        return true;
    }
    else
    {
        std::cout << "모든 미션 끝남!\n";
        return false;
    }
}

bool Local::checkMissionComplete() // 미션 끝났는지 확인하는 함수
{
    if (checkPathComplete())
        return true;
    return false;
}

size_t Local::getCloseMissionIndex()
{
    double closest_dis = global_path_matrix_[0][0].getClosestDistance(cur_car_utm_pos_);
    size_t closest_index = 0;

    for (size_t i = 1; i < global_path_matrix_.size(); ++i)
    {
        if (closest_dis > global_path_matrix_[i][0].getClosestDistance(cur_car_utm_pos_))
        {
            closest_dis = global_path_matrix_[i][0].getClosestDistance(cur_car_utm_pos_);
            closest_index = i;
        }
    }
    std::cout << "car_utm: " << cur_car_utm_pos_[0] << " " << cur_car_utm_pos_[1] << "\n";
    std::cout << "closest_index: " << closest_index << "\n";
    return closest_index;
}

void Local::setCurMissionIndex()
{
    // roslaunch has no interactive stdin. Pick the closest configured mission
    // automatically so the complete workspace starts without blocking.
    cur_mission_index_ = getCloseMissionIndex();
    cur_path_index_ = 0;
    std::cout << "Automatically selected mission index: "
              << cur_mission_index_ << " (mission "
              << mission_list_[cur_mission_index_] << ")\n";
    return;

#if 0
    size_t mission_i = getCloseMissionIndex();
    int chose = 0;
    while (1)
    {
        std::cout << "가장 가까운 미션: " << mission_dic_[mission_list_[mission_i]] << "\n";
        std::cout << "계속 진행(1), 미션 인덱스 입력(2): ";
        std::cin >> chose;
        if (chose == 1)
        {
            cur_mission_index_ = mission_i;
            cur_path_index_ = 0;
            break;
        }
        else if (chose == 2)
        {
            std::cout << "미션 인덱스 입력(미션 번호 x): ";
            std::cin >> mission_i;
            if (mission_i >= getRefMissionList().size())
            {
                std::cout << "미션 인덱스 초과, 다시 입력!\n";
                continue;
            }
            cur_mission_index_ = mission_i;
            cur_path_index_ = 0;
            break;
        }
        else
        {
            std::cout << "잘못된 입력, 다시 입력 바람\n";
        }
    }
#endif
}

void Local::setCurYaw(double cur_yaw)
{
    cur_car_yaw_ = cur_yaw;
}

int Local::getCurMissionNumber() // 현재 미션 번호 반환 함수
{
    return mission_list_[cur_mission_index_];
}

std::string Local::getCurMission()
{
    return mission_dic_[mission_list_[cur_mission_index_]];
}

bool Local::setPathfromTxt(size_t mission_index, size_t path_index)
{
    std::string file_name = global_path_text_adress_;
    file_name.append("/");
    std::string next_file_name = file_name;
    std::string path_name = std::to_string(mission_index);
    path_name.append("-");
    path_name.append(std::to_string(path_index));
    file_name.append(path_name);
    file_name.append(".txt");
    std::ifstream file(file_name);

    if (!file.is_open())
    {
        if (path_index == 0)
        {
            std::cout << path_name << " 파일 열리지 않음." << std::endl;
            return false;
        }
        std::cout << path_index - 1 << "번 txt 파일까지 열림." << std::endl;
        return false;
    }

    if (mission_index >= global_path_matrix_.size())
    {
        throw std::runtime_error("지도 미션 수가 mission_list보다 많습니다: mission index " + std::to_string(mission_index));
    }

    std::vector<std::vector<double>> p_arr;
    std::vector<double> yaw_arr;
    std::vector<double> k_arr;
    double x, y, yaw, k;
    char delimiter; // 쉼표를 구분자로 사용

    while (file >> x >> delimiter >> y >> delimiter >> yaw >> delimiter >> k)
    {
        p_arr.push_back({x, y});
        yaw_arr.push_back(yaw);
        k_arr.push_back(k);
    }
    file.close();

    if (p_arr.size() < 2)
    {
        throw std::runtime_error("경로 파일에 유효한 점이 2개 미만입니다: " + file_name);
    }

    // 340개 인덱스 추가
    int total_points = 0;
    int next_mission_index = mission_index + 1;
    while (total_points < 340)
    {
        std::string next_path_name = std::to_string(next_mission_index);
        next_path_name.append("-0");
        next_file_name = global_path_text_adress_ + "/" + next_path_name + ".txt";
        std::ifstream next_file(next_file_name);

        if (!next_file.is_open())
        {
            std::cout << next_path_name << " 파일 열리지 않음." << std::endl;
            break;
        }

        int i = 0;
        while (next_file >> x >> delimiter >> y >> delimiter >> yaw >> delimiter >> k)
        {
            if (total_points >= 340)
                break;

            p_arr.push_back({x, y});
            yaw_arr.push_back(yaw);
            k_arr.push_back(k);
            ++total_points;
        }
        next_file.close();

        // 만약 읽은 데이터가 부족하다면 다음 파일을 열기 위해 path_index 증가
        if (total_points < 340)
        {
            ++next_mission_index;
        }
    }

    // 충분한 데이터가 확보되지 않았을 경우 마지막 점을 복제
    if (total_points < 340)
    {
        std::vector<double> vec(2, 0);
        vec[0] = p_arr[p_arr.size() - 1][0] - p_arr[p_arr.size() - 2][0];
        vec[1] = p_arr[p_arr.size() - 1][1] - p_arr[p_arr.size() - 2][1];

        std::vector<double> last_p_arr = p_arr[p_arr.size() - 1];
        double last_yaw = yaw_arr[yaw_arr.size() - 1];
        double last_k = 0;
        for (int i = total_points; i < 340; ++i)
        {
            std::vector<double> pos(2, 0);
            pos[0] = last_p_arr[0] + vec[0] * (i - total_points + 1);
            pos[1] = last_p_arr[1] + vec[1] * (i - total_points + 1);
            p_arr.push_back(pos);
            yaw_arr.push_back(last_yaw);
            k_arr.push_back(last_k);
        }
    }

    global_path_matrix_[mission_index].emplace_back(path_name, p_arr, yaw_arr, k_arr);

    return true;
}

void Local::setGlobalPathMatrix()
{
    size_t mission_index = 0;
    while (1)
    {
        size_t path_index = 0;
        while (setPathfromTxt(mission_index, path_index))
        {
            ++path_index;
        }
        if (path_index == 0)
            break;
        ++mission_index;
    }
    if (mission_index == 0)
    {
        throw std::runtime_error("지도 경로를 하나도 읽지 못했습니다: " + global_path_text_adress_);
    }
    if (mission_index != mission_list_.size())
    {
        throw std::runtime_error("지도 미션 수(" + std::to_string(mission_index) +
                                 ")와 mission_list 수(" + std::to_string(mission_list_.size()) + ")가 다릅니다.");
    }
}

void Local::printAllData() const // test용
{
    std::cout << "GLOBAL_PATH_ADRESS"
              << "\n";
    std::cout << global_path_text_adress_ << "\n";

    std::cout << "MISSION_LIST"
              << "\n";
    for (size_t i = 0; i < mission_list_.size(); ++i)
    {
        std::cout << mission_list_[i] << " ";
    }
    std::cout << "\n";

    std::cout << "MISSION_DIC"
              << "\n";
    for (auto &i : mission_dic_)
    {
        std::cout << i.first << " " << i.second << "\n";
    }

    std::cout << "GLOBAL_PATH_LIST"
              << "\n";
    for (size_t i = 0; i < global_path_matrix_.size(); ++i)
    {
        for (size_t j = 0; j < global_path_matrix_[i].size(); ++j)
        {
            global_path_matrix_[i][j].printAllData();
        }
    }

    std::cout << "CUR_UTM_POS"
              << "\n";
    std::cout << cur_car_utm_pos_[0] << " " << cur_car_utm_pos_[1] << "\n";

    std::cout << "CUR_YAW"
              << "\n";
    std::cout << cur_car_yaw_ << "\n";

    std::cout << "CUR_MISSION_INDEX"
              << "\n";
    std::cout << cur_mission_index_ << "\n";

    std::cout << "CUR_PATH_INDEX"
              << "\n";
    std::cout << cur_path_index_ << "\n";
}

void Local::setRcsMode(bool rcs_mode)
{
    rcs_mode_ = rcs_mode;
}

void Local::updateCurKDTree()
{
    global_path_matrix_[cur_mission_index_][cur_path_index_].updateKDTree();
}

void Local::localCurUTMCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    // 현재 GPS 좌표를 저장
    cur_car_utm_pos_[0] = msg->point.x;
    cur_car_utm_pos_[1] = msg->point.y;

    // GPS Queue에 새 데이터를 추가
    std::vector<double> gps_data = {msg->point.x, msg->point.y, msg->header.stamp.toSec()};
    gps_queue.push_back(gps_data);

    // Queue의 크기가 설정된 크기를 초과하면 가장 오래된 데이터를 제거
    if (gps_queue.size() > kGpsQueueSize_)
    {
        gps_queue.pop_front();
    }

    utm_called_ = true;
}

void Local::localCurYawCallback(const std_msgs::Float64::ConstPtr &msg)
{
    cur_car_yaw_ = msg->data;
    yaw_called_ = true;
}

bool Local::callbacksIsCalled()
{
    if (utm_called_ && yaw_called_ && serial_called_ && imu_called_)
        return true;
    return false;
}

Path &Local::getRefNextGlobalPath()
{
    if (cur_path_index_ < global_path_matrix_[cur_mission_index_].size() - 1)
    {
        return global_path_matrix_[cur_mission_index_][cur_path_index_ + 1];
    }
    else
    {
        std::cout << "패스 없음, 현재 패스 반환!!!\n";
        return global_path_matrix_[cur_mission_index_][cur_path_index_];
    }
}

Path &Local::getRefIdxPath(size_t path_index)
{
    if (path_index < global_path_matrix_[cur_mission_index_].size() && path_index >= 0)
    {
        return global_path_matrix_[cur_mission_index_][path_index];
    }
    else
    {
        std::cout << "패스 없음, 현재 패스 반환!!!\n";
        return global_path_matrix_[cur_mission_index_][cur_path_index_];
    }
}

/* 속도 관련 callback 함수 */
void Local::erpDataCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    cur_car_velocity_ = calcCenterSpeed(msg->data[3]);
    serial_called_ = true;
}

void Local::localImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // Extract yaw rate (z-axis angular velocity)
    lo_yaw_rate_ = msg->angular_velocity.z;

    center_speed_yaw_rate_ = lowPassFilter(lo_yaw_rate_, pre_center_speed_yaw_rate_, 0.7);
    pre_center_speed_yaw_rate_ = center_speed_yaw_rate_;

    cur_acceleration_[0] = lowPassFilter(msg->linear_acceleration.x, pre_acceleration_[0], 0.9);
    cur_acceleration_[1] = lowPassFilter(-(msg->linear_acceleration.y), pre_acceleration_[1], 0.9);
    pre_acceleration_[0] = cur_acceleration_[0];
    pre_acceleration_[1] = cur_acceleration_[1];

    cur_car_utm_acceleration_[0] = cur_acceleration_[0] * cos(cur_car_yaw_) - cur_acceleration_[1] * sin(cur_car_yaw_);
    cur_car_utm_acceleration_[1] = cur_acceleration_[0] * sin(cur_car_yaw_) + cur_acceleration_[1] * cos(cur_car_yaw_);
    imu_called_ = true;
}

/* 속도 계산 관련 함수 */
double Local::lowPassFilter(double val, double pre_val, float alpha)
{
    return val * alpha + pre_val * (1 - alpha);
}

double Local::calcCenterSpeed(double left_speed)
{
    if (left_speed <= 0.0)
    {
        return 0.0F;
    }

    // 차량의 각속도
    double yaw_rate = this->center_speed_yaw_rate_;

    // 센터스피드 계산
    double center_speed = left_speed + (car_width_ / 2.0) * abs(yaw_rate);

    return center_speed;
}

bool Local::checkGpsNormal()
{
    if (gps_queue.size() < 2)
    {
        // GPS 데이터가 충분하지 않으면 일단 정상으로 판단
        return true;
    }
    int anomal_num = 0;

    auto prev_it = gps_queue.begin();
    for (auto it = std::next(gps_queue.begin()); it != gps_queue.end(); ++it)
    {
        double distance = std::hypot((*it)[0] - (*prev_it)[0], (*it)[1] - (*prev_it)[1]);
        double time_diff = (*it)[2] - (*prev_it)[2]; // topic의 시간차 계산

        if (time_diff <= 0)
        {
            // 비정상적인 시간차 발견 시 비정상으로 판단
            anomal_num++;
        }

        double speed = distance / time_diff; // 속도 계산

        if (speed > kMaxSpeed)
        {
            // 비정상적으로 높은 속도 발견 시 GPS 비정상 판단
            anomal_num++;
        }

        prev_it = it;
    }
    if (anomal_num < kGpsAnomalMin_)
        return true;
    else
        return false; // anomalmin값 이상이면 anomal로 판단
}

void Local::localNavStatusCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    nav_status_ = static_cast<int>(msg->status.status);
    // std::cout << "GPS 상태 체크합니다~~~ : " << nav_status_ << "\n";
}

const int Local::getNavStatus() const
{
    return nav_status_;
}

void Local::changeRcsMode()
{
    setRcsMode(true);
}

void Local::changeAcsMode()
{
    setRcsMode(false);
    cur_mission_index_ = getCloseMissionIndex();
    cur_path_index_ = 0;
}
