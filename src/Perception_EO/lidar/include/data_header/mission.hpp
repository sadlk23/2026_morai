#ifndef MISSION_HPP
#define MISSION_HPP

#include <ros/ros.h>
#include <chrono>
#include <memory>
#include <functional>
#include <string>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

// Types
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

// Visualization
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/centroid.h> 
#include <pcl/common/common.h>   

class Mission{
public:
    Mission(const std::string &/*node_name*/)
    {
        // ROS 1 node handle
        // Using global or private node handle based on usage
        status_subscriber_ = nh_.subscribe(
            "/Planning/mission",
            10,
            &Mission::status_callback,
            this,
            ros::TransportHints().tcpNoDelay());

        timer_ = nh_.createTimer(ros::Duration(1.0), &Mission::timer_callback, this);
    }
    
    virtual ~Mission() {}

protected:
    ros::NodeHandle nh_;

private:
    // func
    virtual void callback(const sensor_msgs::PointCloud2::ConstPtr& input) = 0;
    
    void status_callback(const std_msgs::Int16::ConstPtr& msg);
    void create_lidar_topic_subscription();
    void remove_lidar_topic_subscription();
    void activate_node();
    void deactivate_node();
    void timer_callback(const ros::TimerEvent& event);

    // sub
    ros::Subscriber sub;
    ros::Subscriber status_subscriber_;
    ros::Timer timer_;

    // var
    virtual int get_status_code() = 0;
    int mission_check = -1; // -1: initial, 0: not mission, 1: mission
    int mission_count = 0;  
};

void Mission::timer_callback(const ros::TimerEvent& /*event*/)
{
    if (mission_check == -1)
    {
        mission_count = 0;
        ROS_INFO("\nerror : 🚨🚨 미션 어딨어? 미션 어딨어? 🚨🚨");
    }
    else if (mission_check > 0 && mission_count < 1)
    {
        ROS_INFO("\n미션 간다 ~!🥇 미션 간다 ~!🥇");
        mission_count = 1;
    }
    mission_check = -1;
}

void Mission::status_callback(const std_msgs::Int16::ConstPtr& msg)
{
    mission_check = 1;
    int status_code = msg->data;
    if (status_code == get_status_code())
    {
        activate_node();
    }
    else
    {
        deactivate_node();
    }
}

void Mission::create_lidar_topic_subscription()
{
    if (!sub)
    {
        sub = nh_.subscribe("/velodyne_points", 1, &Mission::callback, this);
    }
}

void Mission::remove_lidar_topic_subscription()
{
    if (sub)
    {
        sub.shutdown();
    }
}

void Mission::activate_node()
{
    // ROS_INFO("Node activated");
    create_lidar_topic_subscription();
}

void Mission::deactivate_node()
{
    // ROS_INFO("Node deactivated");
    remove_lidar_topic_subscription();
}

#endif // MISSION_HPP
