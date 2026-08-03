#ifndef LIDAR_MISSION_EXAMPLE_HPP
#define LIDAR_MISSION_EXAMPLE_HPP

#include "data_header/mission.hpp"

// ROS1 미션 클래스 작성 예시. Mission의 상태 코드에 따라 PointCloud 구독이
// 자동으로 활성화/비활성화된다.
class Example : public Mission
{
public:
    Example();

private:
    void callback(const sensor_msgs::PointCloud2::ConstPtr &input) override;
    int get_status_code() override;

    ros::Publisher pointcloud_pub_;
    ros::Publisher other_type_pub_;
    int check_pointcloud_ = 0;
};

#endif
