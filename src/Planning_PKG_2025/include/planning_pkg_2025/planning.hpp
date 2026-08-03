#ifndef PLANNING_H_
#define PLANNING_H_

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "nlohmann/json.hpp"
#include <ros/ros.h>

#include "control.hpp"
#include "dynamic_obstacle_stop.hpp"
#include "highway_lane_change.hpp"
#include "lidar.hpp"
#include "local.hpp"
#include "mission_data.hpp"
#include "mission_2026_ros1_adapter.hpp"
#include "path.hpp"
#include "vision.hpp"

/**
 * @brief 2026 미션 1~3 전용 Planning 클래스.
 *
 * 모든 미션은 chooseFunc()를 통해 선택되고, 결과는 공용 Path와
 * Control 객체에 반영된다. Mission 2026 코어도 같은 출력 경로를
 * 사용하므로 planning_node에는 별도 발행 분기가 없다.
 */
class Planning
{
private:
    Local local_;
    Lidar lidar_;
    Vision vision_;
    Control control_;
    Path local_path_;
    MissionData mission_data_;

    DynamicObstacleStop dynamic_obstacle_stop_;
    HighwayLaneChange highway_lane_change_;
    std::unique_ptr<mission_2026::Mission2026Ros1Adapter>
        mission_2026_adapter_;
    std::vector<std::optional<mission_2026::Point2D>>
        mission_2026_mapped_stop_lines_;

    int previous_mission_index_ = -1;
    const double default_velocity_;
    const double velocity_60_;
    const double dynamic_obstacle_velocity_;

    std::vector<std::vector<std::vector<double>>> empty_path_vectors_;

public:
    Planning(ros::NodeHandle &node,
             ros::NodeHandle &private_node,
             std::string &global_path_address,
             std::vector<int> &mission_list,
             std::unordered_map<int, std::string> &mission_dictionary,
             nlohmann::json &basic_data,
             nlohmann::json &mission_param)
        : local_(global_path_address, mission_list, mission_dictionary),
          lidar_(basic_data.at("lidar")),
          control_(basic_data.at("control")),
          local_path_("local_path"),
          mission_data_(),
          dynamic_obstacle_stop_(
              local_, control_, local_path_, lidar_, vision_, mission_data_,
              mission_param.at("stop_traffic")),
          highway_lane_change_(
              local_, lidar_, control_, local_path_,
              mission_param.at("highway_lane_change")),
          default_velocity_(
              basic_data.at("planning").at("default_velocity")),
          velocity_60_(
              basic_data.at("planning").at("velocity_60")),
          dynamic_obstacle_velocity_(
              mission_param.at("dynamic_obstacle").value(
                  "target_velocity", default_velocity_))
    {
        mission_2026_adapter_ =
            std::make_unique<mission_2026::Mission2026Ros1Adapter>(
                node, private_node);

        try
        {
            const auto &stop_lines = mission_param.at("stop_traffic")
                                         .at("stop_line_list");
            mission_2026_mapped_stop_lines_.reserve(stop_lines.size());
            for (const auto &line : stop_lines)
            {
                const double x = line.at(0).get<double>();
                const double y = line.at(1).get<double>();
                if (std::isfinite(x) && std::isfinite(y) &&
                    std::hypot(x, y) > 1.0)
                {
                    mission_2026_mapped_stop_lines_.push_back(
                        mission_2026::Point2D{x, y});
                }
                else
                {
                    mission_2026_mapped_stop_lines_.push_back(
                        std::nullopt);
                }
            }
        }
        catch (const std::exception &error)
        {
            ROS_WARN(
                "mission_2026 stop-line parameters are unavailable: %s",
                error.what());
            mission_2026_mapped_stop_lines_.clear();
        }
    }

    Local &getLocal() { return local_; }
    Lidar &getLidar() { return lidar_; }
    Vision &getVision() { return vision_; }
    Control &getControl() { return control_; }
    MissionData &getMissionData() { return mission_data_; }

    const Path &getRefLocalPath() const
    {
        return local_path_;
    }

    std::vector<double> getHighwayProjectedTriggerPoints() const
    {
        return highway_lane_change_.getProjectedTriggerPoints();
    }

    int getHighwayCurrentLaneIdx() const
    {
        return highway_lane_change_.getCurrentLaneIdx();
    }

    const std::vector<std::vector<std::vector<double>>> &
    getAllPathVector() const
    {
        return empty_path_vectors_;
    }

    void curGlobalToLocalCopy()
    {
        const Path &global_path = local_.getRefCurGlobalPath();
        local_path_.setPosArr(global_path.getRefPosArr());
        local_path_.setYawArr(global_path.getRefYawArr());
        local_path_.setKArr(global_path.getRefKArr());
        local_path_.updateKDTree();
    }

    void setDefaultStatus()
    {
        local_.setCurMissionIndex();
        curGlobalToLocalCopy();
        previous_mission_index_ =
            static_cast<int>(local_.getCurMissionIndex());
    }

    bool checkLocalPathComplete() const
    {
        const auto &positions = local_path_.getRefPosArr();
        if (positions.empty())
        {
            return false;
        }

        const int closest_index =
            local_path_.getClosestIndex(local_.getAddCurCarUTMPos());
        if (closest_index < 0)
        {
            return false;
        }

        const std::size_t remaining =
            positions.size() - std::min(
                positions.size(),
                static_cast<std::size_t>(closest_index));
        return remaining < 340;
    }

    bool judgeMapChange()
    {
        if (local_.getCurMissionNumber() == 100 &&
            !highway_lane_change_.isMissionComplete())
        {
            return true;
        }

        if (local_.checkMissionComplete() && checkLocalPathComplete())
        {
            if (local_.changeNextMission())
            {
                curGlobalToLocalCopy();
                lidar_.clearAllClusters();
                return true;
            }
            return false;
        }
        return true;
    }

    bool checkMissionChange()
    {
        const int current_index =
            static_cast<int>(local_.getCurMissionIndex());
        if (previous_mission_index_ == current_index)
        {
            return false;
        }

        previous_mission_index_ = current_index;
        return true;
    }

    void printStatus() const
    {
        const double *position = local_.getAddCurCarUTMPos();
        ROS_INFO_THROTTLE(
            1.0,
            "Planning mission=%d (%s), UTM=(%.3f, %.3f), yaw=%.3f, velocity=%.3f",
            local_.getCurMissionNumber(),
            local_.getCurMission().c_str(),
            position[0], position[1],
            local_.getCurCarYaw(),
            local_.getCurCarVelocity());
    }

    void applyMission2026Override(
        const mission_2026::Mission2026Override &output)
    {
        if (!output.active)
        {
            if (output.request_absolute_path)
            {
                curGlobalToLocalCopy();
            }
            control_.setTargetVelocity(default_velocity_);
            return;
        }

        if (output.positions.size() < 2 ||
            output.positions.size() % 2 != 0)
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "mission_2026 returned an invalid position array");
            control_.setTargetVelocity(0.0);
            return;
        }

        std::vector<std::vector<double>> positions;
        positions.reserve(output.positions.size() / 2);
        for (std::size_t index = 0;
             index + 1 < output.positions.size(); index += 2)
        {
            positions.push_back(
                {output.positions[index], output.positions[index + 1]});
        }

        const bool relative_path =
            positions.front()[0] == mission_2026::kRelativePathMarker &&
            positions.front()[1] == mission_2026::kRelativePathMarker;
        const std::size_t path_point_count =
            positions.size() - (relative_path ? 1U : 0U);
        if (output.yaws.size() != path_point_count ||
            output.curvatures.size() != path_point_count)
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "mission_2026 path/yaw/curvature sizes do not match");
            control_.setTargetVelocity(0.0);
            return;
        }

        local_path_.setPosArr(positions);
        local_path_.setYawArr(output.yaws);
        local_path_.setKArr(output.curvatures);
        if (!relative_path)
        {
            local_path_.updateKDTree();
        }
        control_.setTargetVelocity(output.target_speed_mps);
    }

    void chooseFunc()
    {
        const int mission_number = local_.getCurMissionNumber();
        const double common_front_distance =
            lidar_.getRefDynamicCarDistance();

        control_.beginControlCycle();

        std::optional<mission_2026::Point2D> mapped_stop_line;
        const std::size_t mission_index = local_.getCurMissionIndex();
        if (mission_index < mission_2026_mapped_stop_lines_.size())
        {
            mapped_stop_line =
                mission_2026_mapped_stop_lines_[mission_index];
        }

        const auto mission_2026_output = mission_2026_adapter_->evaluate(
            mission_number,
            local_.getRefCurGlobalPath(),
            mapped_stop_line);
        if (mission_2026_output.handles_mission)
        {
            applyMission2026Override(mission_2026_output);
            ROS_INFO_THROTTLE(
                1.0, "mission_2026: %s",
                mission_2026_output.diagnostic.c_str());
            return;
        }

        switch (mission_number)
        {
        case 1:
            // Non-mission connector sections in the 2026 competition map.
            control_.setTargetVelocity(default_velocity_);
            break;

        case 24:
            if (checkMissionChange())
            {
                dynamic_obstacle_stop_.resetStatus();
            }
            dynamic_obstacle_stop_.doMission(
                dynamic_obstacle_velocity_);
            break;

        case 60:
            control_.setTargetVelocity(velocity_60_);
            break;

        case 100:
            if (checkMissionChange())
            {
                highway_lane_change_.resetStatus();
            }
            highway_lane_change_.drive();
            break;

        case 14:
        case 23:
        case 31:
            // 어댑터 설정에서 미션 번호가 제외된 경우의 안전 fallback.
            control_.setTargetVelocity(default_velocity_);
            break;

        default:
            ROS_ERROR_THROTTLE(
                1.0,
                "지원하지 않는 미션 번호 %d: 안전 정지",
                mission_number);
            control_.setTargetVelocity(0.0);
            break;
        }

        if (mission_number != 60 &&
            !control_.wasDistanceControlApplied())
        {
            control_.distance_control(
                common_front_distance,
                control_.getMissionTargetVelocity());
        }
    }
};

#endif
