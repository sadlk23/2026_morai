#ifndef GAIN_TUNING_HPP
#define GAIN_TUNING_HPP

#include <fstream>
#include <vector>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <ros/package.h>

// ROS1으로 변환된 헤더 참조
#include "control/callback_data_manage.hpp"
#include "control/lat_control.hpp"
#include "control/lon_control.hpp"

using json = nlohmann::json;

/*
제작 : 예원태
설명 : 
ROS1 Noetic 변환 버전.
Json 파일을 읽어 Normal/Mission별 게인을 동적으로 할당한다.
*/

class ControlGainTuning
{
private:
    CallbackClass *cb_data_;
    CombinedSteer *lat_con_;
    ReversePurePursuit *R_lat_con_;
    LonController *long_con_;

    float gear = 0.0; // 0: 전진, 1: 중립, 2: 후진
    json file_data;
    json missions_data;

    float curvature_gain = 0.0;
    float pre_curv_speed_clip = 0.0;

public:
    ControlGainTuning(CallbackClass *cb_data, CombinedSteer *lat_con, ReversePurePursuit *R_lat_con,
                      LonController *long_con, const std::string &profile_name)
    {
        this->cb_data_ = cb_data;
        this->lat_con_ = lat_con;
        this->R_lat_con_ = R_lat_con;
        this->long_con_ = long_con;

        if (read_json(profile_name) != 0)
        {
            throw std::runtime_error("제어 프로파일을 불러오지 못했습니다: " + profile_name);
        }
    }

    int read_json(const std::string &profile_name)
    {
        const std::string package_path = ros::package::getPath("control");
        if (package_path.empty())
        {
            std::cerr << "[Error] control 패키지 경로를 찾을 수 없습니다." << std::endl;
            return 1;
        }

        std::string file_name = profile_name;
        if (file_name.size() < 5 || file_name.substr(file_name.size() - 5) != ".json")
        {
            file_name += ".json";
        }
        std::string file_path = package_path + "/config/" + file_name;

        std::ifstream input_file(file_path);
        if (!input_file.is_open())
        {
            // 소스 공간에서 실행할 때의 catkin 패키지 레이아웃
            file_path = package_path + "/include/control/" + file_name;
            input_file.open(file_path);
        }
        if (!input_file.is_open())
        {
            std::cerr << "\033[1;31m[Error]\033[0m JSON 파일을 열 수 없습니다: " << file_path << std::endl;
            return 1;
        }

        try
        {
            input_file >> file_data;
        }
        catch (const json::parse_error &e)
        {
            std::cerr << "JSON 파싱 에러: " << e.what() << std::endl;
            input_file.close();
            return 1;
        }

        this->missions_data = file_data["missions"];
        this->curvature_gain = this->file_data["safety_factor"]["curvature_gain"];

        return 0;
    }

    // 기본(Normal) 파라미터 세팅
    void set_normal_param()
    {
        json normal_data = file_data["normal"];

        // 종방향(Longitudinal) 파라미터
        json lon_param = normal_data["______lllllllllllllll______"];
        long_con_->set_lon_PD_gain(lon_param["PD_gas_gain"][0], lon_param["PD_gas_gain"][1],
                                   lon_param["PD_brake_gain"][0], lon_param["PD_brake_gain"][1]);
        long_con_->set_enable_brake_error(lon_param["enable_brake_error"]);
        this->gear = lon_param["gear"];

        // 횡방향(Lateral) 파라미터
        json lat_param = normal_data["______---------------______"];
        
        // Stanley 관련 게인
        lat_con_->set_stanly_gain(lat_param["stanly_gain"][0], lat_param["stanly_gain"][1], lat_param["stanly_gain"][2]);
        lat_con_->set_heading_gain(lat_param["stanly_heading_gain"]);
        lat_con_->set_anti_windup_max(lat_param["anti_windup_val"]);

        // Stanley Preview 관련
        std::vector<float> preview_gain = lat_param["stanley_preview_gain"];
        lat_con_->set_preview_param(lat_param["stanley_preview_dt"], preview_gain);
        lat_con_->set_preview_heading_error_gain(lat_param["stanley_preview_h_e_gain"]);

        // Pure Pursuit 관련
        lat_con_->set_pp_LD_threshold(lat_param["PP_LD"][0], lat_param["PP_LD"][1]);
        lat_con_->set_pp_gain(lat_param["PP_gain"][0], lat_param["PP_gain"][1]);

        // Reverse Pure Pursuit 관련
        R_lat_con_->set_R_PP_gain(lat_param["R_PP_gain"][0], lat_param["R_PP_gain"][1]);
        R_lat_con_->set_R_pp_LD_threshold(lat_param["reverse_PP_LD"][0], lat_param["reverse_PP_LD"][1]);

        // Combined Steer (PP vs Stanley 비율)
        lat_con_->set_combine_PP_ratio(lat_param["combine_PP_ratio"]);
    }

    // 미션 번호에 따른 파라미터 오버라이드
    int set_mission_param()
    {
        this->set_normal_param(); // 먼저 노멀 세팅 후 덮어쓰기

        int param_change_count = 0;
        int mission = cb_data_->get_mission_num();
        std::string mission_key = std::to_string(mission);

        if (missions_data.find(mission_key) != missions_data.end())
        {
            json m_data = missions_data[mission_key];

            // 미션별 종방향 게인 덮어쓰기
            if (m_data.find("______lllllllllllllll______") != m_data.end())
            {
                json lon_param = m_data["______lllllllllllllll______"];
                if (lon_param.find("PD_gas_gain") != lon_param.end())
                {
                    long_con_->set_lon_PD_gain(lon_param["PD_gas_gain"][0], lon_param["PD_gas_gain"][1],
                                               lon_param["PD_brake_gain"][0], lon_param["PD_brake_gain"][1]);
                    param_change_count++;
                }
                if (lon_param.find("enable_brake_error") != lon_param.end())
                {
                    long_con_->set_enable_brake_error(lon_param["enable_brake_error"]);
                    param_change_count++;
                }
            }

            // 미션별 횡방향 게인 덮어쓰기
            if (m_data.find("______---------------______") != m_data.end())
            {
                json lat_param = m_data["______---------------______"];
                if (lat_param.find("stanly_gain") != lat_param.end()) {
                    lat_con_->set_stanly_gain(lat_param["stanly_gain"][0], lat_param["stanly_gain"][1], lat_param["stanly_gain"][2]);
                    param_change_count++;
                }
                if (lat_param.find("PP_LD") != lat_param.end()) {
                    lat_con_->set_pp_LD_threshold(lat_param["PP_LD"][0], lat_param["PP_LD"][1]);
                    param_change_count++;
                }
                if (lat_param.find("combine_PP_ratio") != lat_param.end()) {
                    lat_con_->set_combine_PP_ratio(lat_param["combine_PP_ratio"]);
                    param_change_count++;
                }
                // (필요에 따라 다른 파라미터 추가 가능)
            }
        }
        return param_change_count;
    }

    // 곡률에 따른 속도 감속 로직
    void target_speed_reducing_by_curature(double curvature)
    {
        float mission_speed = long_con_->get_target_speed();
        double curv = clip(curvature, 0.1, 1.5);
        
        // 원심력 기반 속도 산출: v = sqrt(gain / curvature)
        float curvature_speed = std::sqrt(this->curvature_gain / curv);
        float curv_speed_clip = clip(curvature_speed, 1.9F, mission_speed);

        double reducing_speed = low_pass_filter(curv_speed_clip, pre_curv_speed_clip, 0.9);
        pre_curv_speed_clip = (float)reducing_speed;

        long_con_->set_lon_target_speed((float)reducing_speed);
    }

    // 조향각에 따른 속도 감속 로직
    void target_speed_reducing_by_steer(float tmp_steer)
    {
        float max_steer = std::abs(tmp_steer);
        float erp_steer = std::abs(cb_data_->get_steer());
        if (erp_steer > max_steer) max_steer = erp_steer;

        float mission_speed = long_con_->get_target_speed();
        double speed_gain = 1.0;

        // 조향각 구간별 감속 비율 (Rad 단위 계산)
        double steer_rad = std::abs(max_steer);
        if (steer_rad > 26.0 * M_PI / 180.0) speed_gain = 0.87;
        else if (steer_rad > 22.0 * M_PI / 180.0) speed_gain = 0.91;
        else if (steer_rad > 17.0 * M_PI / 180.0) speed_gain = 0.93;
        else if (steer_rad > 12.0 * M_PI / 180.0) speed_gain = 0.95;

        long_con_->set_lon_target_speed(mission_speed * (float)speed_gain);
    }

    float get_gear() { return this->gear; }
};

#endif // GAIN_TUNING_HPP
