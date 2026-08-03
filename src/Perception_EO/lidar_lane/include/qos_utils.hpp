// qos_utils.hpp  (ROS1 / roscpp 전용 최소 변경본)

#ifndef LIDAR_QOS_UTILS_HPP_
#define LIDAR_QOS_UTILS_HPP_

#include <ros/ros.h>

namespace lidar
{
    // ROS1에서는 QoS 개념이 없으므로 queue_size와 TransportHints만 노출
    struct Qos
    {
        int queue_size;
        bool latch;
        ros::TransportHints hints;

        explicit Qos(int q = 10, bool l = false,
                     const ros::TransportHints &h = ros::TransportHints())
            : queue_size(q), latch(l), hints(h) {}
    };

    // 요청: 큐 10, tcpNoDelay
    inline Qos reliable_qos()
    {
        return Qos(10, false, ros::TransportHints().tcpNoDelay());
    }

    // 요청: 큐 10, tcpNoDelay (ROS1엔 best_effort가 없으니 동일 처리)
    inline Qos best_effort_qos()
    {
        return Qos(10, false, ros::TransportHints().tcpNoDelay());
    }

} // namespace lidar

#endif // LIDAR_QOS_UTILS_HPP_