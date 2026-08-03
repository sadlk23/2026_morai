#include <ros/ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <cmath>
#include <vector>
#include <typeinfo>
#include <cstdio>
#include <fstream>
#include <filesystem>

#include "data_header/utility_function.hpp"

using namespace std;
using namespace pcl;
using PointT = pcl::PointXYZI;
std::chrono::duration<double> cut_in_cheack_time;
std::chrono::system_clock::time_point start_time_;

int planning_flag = 0;

class LiDARBIG{
public:
  sensor_msgs::PointCloud2 output;
  int check_pointcloud = 0;

  LiDARBIG(const std::string& /*node_name*/ = "velodyne_preprocess")
  : nh_(), pnh_("~")
  {
    pnh_.param("mission100/roi_rear", mission100_roi_rear_, 60.0);
    pnh_.param("mission100/roi_front", mission100_roi_front_, 60.0);
    pnh_.param("mission100/roi_left_min", mission100_roi_left_min_, 1.0);
    pnh_.param("mission100/roi_left_max", mission100_roi_left_max_, 4.0);
    pnh_.param("mission100/roi_z_min", mission100_roi_z_min_, -1.1);
    pnh_.param("mission100/roi_z_max", mission100_roi_z_max_, 1.0);
    pnh_.param("mission100/voxel_size", mission100_voxel_size_, 0.1);
    pnh_.param("mission100/cluster_tolerance", mission100_cluster_tolerance_, 0.55);
    pnh_.param("mission100/min_cluster_size", mission100_min_cluster_size_, 3);
    pnh_.param(
        "mission100/tracking/match_distance",
        mission100_track_match_distance_, 4.0);
    pnh_.param(
        "mission100/tracking/timeout_sec",
        mission100_track_timeout_sec_, 0.5);
    pnh_.param(
        "mission100/tracking/max_missed_scans",
        mission100_track_max_missed_scans_, 2);
    pnh_.param(
        "mission100/tracking/velocity_filter_alpha",
        mission100_velocity_filter_alpha_, 0.4);
    pnh_.param(
        "mission100/tracking/min_velocity_dt_sec",
        mission100_min_velocity_dt_sec_, 0.03);
    pnh_.param(
        "mission100/tracking/max_abs_relative_velocity_mps",
        mission100_max_abs_relative_velocity_mps_, 45.0);
    pnh_.param(
        "mission100/front_vehicle/roi_rear",
        car_front_roi_rear_, 0.0);
    pnh_.param(
        "mission100/front_vehicle/roi_front",
        car_front_roi_front_, 40.0);
    pnh_.param(
        "mission100/front_vehicle/roi_y_min",
        car_front_roi_y_min_, -0.5);
    pnh_.param(
        "mission100/front_vehicle/roi_y_max",
        car_front_roi_y_max_, 0.5);
    pnh_.param(
        "mission100/front_vehicle/roi_z_min",
        car_front_roi_z_min_, -1.1);
    pnh_.param(
        "mission100/front_vehicle/roi_z_max",
        car_front_roi_z_max_, 1.0);
    pnh_.param(
        "mission100/front_vehicle/voxel_size",
        car_front_voxel_size_, 0.1);
    pnh_.param(
        "mission100/front_vehicle/cluster_tolerance",
        car_front_cluster_tolerance_, 0.5);
    pnh_.param(
        "mission100/front_vehicle/min_cluster_size",
        car_front_min_cluster_size_, 5);
    pnh_.param(
        "mission100/front_vehicle/max_cluster_size",
        car_front_max_cluster_size_, 2000);
    pnh_.param(
        "mission100/front_vehicle/min_width_y",
        car_front_min_width_y_, 0.35);
    pnh_.param(
        "mission100/front_vehicle/min_height_z",
        car_front_min_height_z_, 0.30);
    pnh_.param(
        "mission100/front_vehicle/no_detection_distance",
        car_front_no_detection_distance_, 40.0);

    planning_mission_sub = nh_.subscribe<std_msgs::Int16>(
        "/Planning/mission", 10,
        &LiDARBIG::mission_cb, this,
        ros::TransportHints().tcpNoDelay());

    mission100_lane_sub_ = nh_.subscribe<std_msgs::Int16>(
        "/Planning/mission100_current_lane", 1,
        &LiDARBIG::mission100LaneCallback, this,
        ros::TransportHints().tcpNoDelay());

    LiDAR_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "/lidar/velodyne_points", 10,
        &LiDARBIG::LiDARCallback, this,
        ros::TransportHints().tcpNoDelay());

    ROS_INFO("[LiDAR] 노드 초기화: 토픽 /lidar/velodyne_points 구독 시작");

    ROS_INFO("[LiDAR] 노드 초기화: 토픽 /lidar/velodyne_points 구독 시작");
  
      big_obj_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/LiDAR/BigObject", 10); // 포인트클라우드용
      big_obj_pub_    = nh_.advertise<std_msgs::Float64MultiArray>("/LiDAR/bigcone_point", 10); 
      marker_big_pub_ = nh_.advertise<visualization_msgs::Marker>("/Marker/bigobject", 10);
      marker_big_roi_pub_ = nh_.advertise<visualization_msgs::Marker>("/Marker/big_roi", 10);

      // Small Static (Mission 14)
      small_obj_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/LiDAR/object_cen", 10);
      marker_small_pub_ = nh_.advertise<visualization_msgs::Marker>("/Marker/small_object", 10);
      marker_small_roi_pub_ = nh_.advertise<visualization_msgs::Marker>("/Marker/small_roi", 10);

      // U-turn (Mission 22)
      uturn_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/Cluster/uturn", 10);
      uturn_center_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/LiDAR/center_point", 10);
      marker_uturn_roi_pub_ = nh_.advertise<visualization_msgs::Marker>("/Marker/uturn_roi", 10);
      marker_uturn_pts_pub_ = nh_.advertise<visualization_msgs::Marker>("/Marker/uturn_pts", 10);

      cut_in_ok = nh_.advertise<std_msgs::Bool>("/LiDAR/CutInOk", 10);
      left_lane_clear_pub_ = nh_.advertise<std_msgs::Bool>("/LiDAR/left_lane_clear", 10);
      mission100_left_roi_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
          "/LiDAR/mission100_left_roi", 1);
      mission100_left_clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
          "/LiDAR/mission100_left_clusters", 1);
      mission100_left_obstacles_marker_pub_ =
          nh_.advertise<visualization_msgs::MarkerArray>(
              "/Marker/mission100_left_obstacles", 1);
      mission100_left_roi_marker_pub_ =
          nh_.advertise<visualization_msgs::Marker>(
              "/Marker/mission100_left_roi", 1);
      mission100_left_tracks_pub_ =
          nh_.advertise<std_msgs::Float64MultiArray>(
              "/LiDAR/mission100_left_tracks", 1);
      mission100_current_front_tracks_pub_ =
          nh_.advertise<std_msgs::Float64MultiArray>(
              "/LiDAR/mission100_current_front_tracks", 1);
      car_front_car_distance_pub_ =
          nh_.advertise<std_msgs::Float32>(
              "/LiDAR/car_front_car_dis", 1);
      car_front_roi_pub_ =
          nh_.advertise<sensor_msgs::PointCloud2>(
              "/LiDAR/car_front_roi", 1);
      car_front_clusters_pub_ =
          nh_.advertise<sensor_msgs::PointCloud2>(
              "/LiDAR/car_front_clusters", 1);
      car_front_obstacles_marker_pub_ =
          nh_.advertise<visualization_msgs::MarkerArray>(
              "/Marker/car_front_obstacles", 1);
      car_front_roi_marker_pub_ =
          nh_.advertise<visualization_msgs::Marker>(
              "/Marker/car_front_roi", 1);
      
      // Mission 21: Angle Parking
      pointcloud_pub_angle_ = nh_.advertise<sensor_msgs::PointCloud2>("/Cluster/angle", 1);
      angled_front_point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/LiDAR/angled_front_point", 1);
      next_front_point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/LiDAR/next_front_point", 1);
      idx_pub_ = nh_.advertise<std_msgs::Int16>("/LiDAR/parking_idx", 1);
      is_empty_ = nh_.advertise<std_msgs::Bool>("/LiDAR/is_empty", 1);
      marker_pub_angle_ = nh_.advertise<visualization_msgs::MarkerArray>("/Marker/parking_cen_mark", 1);
      marker_pub2_ = nh_.advertise<visualization_msgs::Marker>("/Marker/angle_roi_1", 1);
      marker_pub3_ = nh_.advertise<visualization_msgs::Marker>("/Marker/angle_roi_2", 1);
      marker_pub4_ = nh_.advertise<visualization_msgs::Marker>("/Marker/angle_roi_3", 1);
      marker_pub5_ = nh_.advertise<visualization_msgs::Marker>("/Marker/angle_roi_4", 1);
      marker_pub6_ = nh_.advertise<visualization_msgs::MarkerArray>("/Marker/angle_roi_4_clusters", 1);
      two_roi_point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/LiDAR/two_parking_point", 1);
      all_roi_point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/LiDAR/all_parking_point", 1);
      other_type_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/LiDAR/parking_edge_out", 1);
  }
  ~LiDARBIG()
  {
    ROS_INFO("Velodyne Preprocess Node has been terminated");
  }

  void LiDARCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  
  void mission_cb(const std_msgs::Int16::ConstPtr& msgs)
  {
    if (planning_flag != 100 && msgs->data == 100)
    {
      resetMission100Tracking(mission100_left_tracker_);
      resetMission100Tracking(mission100_current_front_tracker_);
      mission100_last_lane_idx_ = -1;
    }

    if (planning_flag == 100 && msgs->data != 100)
    {
      resetMission100Tracking(mission100_left_tracker_);
      resetMission100Tracking(mission100_current_front_tracker_);
      mission100_last_lane_idx_ = -1;

      std_msgs::Bool clear_msg;
      clear_msg.data = false;
      left_lane_clear_pub_.publish(clear_msg);

      std_msgs::Header clear_header;
      clear_header.stamp = ros::Time::now();
      clear_header.frame_id = "velodyne";
      const pcl::PointCloud<PointT> empty_cloud;
      const std::vector<Mission100ObstacleDetection> no_obstacles;
      publishMission100ObstacleMarkers(
          no_obstacles,
          mission100_left_obstacles_marker_pub_,
          clear_header,
          "mission100_left_obstacles",
          1.0, 0.0, 1.0);
      publishMission100ObstacleMarkers(
          no_obstacles,
          car_front_obstacles_marker_pub_,
          clear_header,
          "car_front_obstacles",
          1.0, 0.1, 0.0);
      if (mission100_left_roi_pub_.getNumSubscribers() > 0)
      {
        publishMission100Cloud(
            empty_cloud, mission100_left_roi_pub_, clear_header);
      }
      if (mission100_left_clusters_pub_.getNumSubscribers() > 0)
      {
        publishMission100Cloud(
            empty_cloud, mission100_left_clusters_pub_, clear_header);
      }

      visualization_msgs::Marker delete_marker;
      delete_marker.header = clear_header;
      delete_marker.ns = "mission100_left_roi";
      delete_marker.id = 0;
      delete_marker.action = visualization_msgs::Marker::DELETE;
      mission100_left_roi_marker_pub_.publish(delete_marker);

      publishMission100Tracks(
          mission100_left_tracker_,
          mission100_left_tracks_pub_,
          clear_header,
          false);
      publishMission100Tracks(
          mission100_current_front_tracker_,
          mission100_current_front_tracks_pub_,
          clear_header,
          false);

      std_msgs::Float32 reset_distance_msg;
      reset_distance_msg.data = 0.0f;
      car_front_car_distance_pub_.publish(reset_distance_msg);

      if (car_front_roi_pub_.getNumSubscribers() > 0)
      {
        publishMission100Cloud(
            empty_cloud, car_front_roi_pub_, clear_header);
      }
      if (car_front_clusters_pub_.getNumSubscribers() > 0)
      {
        publishMission100Cloud(
            empty_cloud, car_front_clusters_pub_, clear_header);
      }

      visualization_msgs::Marker delete_front_marker;
      delete_front_marker.header = clear_header;
      delete_front_marker.ns = "car_front_roi";
      delete_front_marker.id = 0;
      delete_front_marker.action = visualization_msgs::Marker::DELETE;
      car_front_roi_marker_pub_.publish(delete_front_marker);
    }

    planning_flag = msgs->data;
    ROS_DEBUG("[LiDAR] Mission flag 변경: %d", planning_flag);
  }

  void mission100LaneCallback(
      const std_msgs::Int16::ConstPtr& msg)
  {
    if (planning_flag != 100 ||
        msg->data < 1 ||
        msg->data > 4)
    {
      mission100_last_lane_idx_ = -1;
      return;
    }

    if (mission100_last_lane_idx_ != -1 &&
        mission100_last_lane_idx_ != msg->data)
    {
      const int previous_lane_idx = mission100_last_lane_idx_;
      resetMission100Tracking(mission100_left_tracker_);
      resetMission100Tracking(mission100_current_front_tracker_);
      std_msgs::Header reset_header;
      reset_header.stamp = ros::Time::now();
      reset_header.frame_id = "velodyne";
      publishMission100Tracks(
          mission100_left_tracker_,
          mission100_left_tracks_pub_,
          reset_header,
          false);
      publishMission100Tracks(
          mission100_current_front_tracker_,
          mission100_current_front_tracks_pub_,
          reset_header,
          false);
      publishMission100ObstacleMarkers(
          std::vector<Mission100ObstacleDetection>(),
          mission100_left_obstacles_marker_pub_,
          reset_header,
          "mission100_left_obstacles",
          1.0, 0.0, 1.0);
      ROS_INFO(
          "[LiDAR][Mission100] Lane changed %d -> %d; left tracks reset",
          previous_lane_idx, msg->data);
    }
    mission100_last_lane_idx_ = msg->data;
  }

  void Check_Left_Lane(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void detectcarFrontVehicle(
      const sensor_msgs::PointCloud2::ConstPtr& msg);
  void Find_big(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void Find_Small(const sensor_msgs::PointCloud2::ConstPtr& msg); // Mission 14
  void U_Turn(const sensor_msgs::PointCloud2::ConstPtr& msg);     // Mission 22
  void Cut_in_41(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void Cut_in_42(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void Cut_in_43(const sensor_msgs::PointCloud2::ConstPtr& msg);  
  
  // Mission 21: Angle Parking
  void Find_AngleParking(const sensor_msgs::PointCloud2::ConstPtr& input);
  void define_hull_indices(
        std::vector<pcl::Vertices> &hull_indices,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &hull_points);
  void visualize_roi(const pcl::PointCloud<PointT>::Ptr &hull_points, const std::string &ns);
  void publish_cluster_centroids(const std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> &clusters_vec);
  void publish_filtered_markers(
        const std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>> &clusters_vec,
        float min_x, float max_x, float min_y, float max_y);

private:
  ros::Publisher big_obj_pub_;
  ros::Publisher big_obj_pc_pub_;
  ros::Publisher marker_big_pub_;
  ros::Publisher marker_big_roi_pub_;

  ros::Publisher small_obj_pub_;   // Mission 14
  ros::Publisher marker_small_pub_;
  ros::Publisher marker_small_roi_pub_;
  ros::Publisher uturn_pub_;       // Mission 22
  ros::Publisher uturn_center_pub_;// Mission 22
  ros::Publisher marker_uturn_roi_pub_;
  ros::Publisher marker_uturn_pts_pub_;

  ros::Publisher cut_in_ok;
  ros::Publisher left_lane_clear_pub_;
  ros::Publisher mission100_left_roi_pub_;
  ros::Publisher mission100_left_clusters_pub_;
  ros::Publisher mission100_left_obstacles_marker_pub_;
  ros::Publisher mission100_left_roi_marker_pub_;
  ros::Publisher mission100_left_tracks_pub_;
  ros::Publisher mission100_current_front_tracks_pub_;
  ros::Publisher car_front_car_distance_pub_;
  ros::Publisher car_front_roi_pub_;
  ros::Publisher car_front_clusters_pub_;
  ros::Publisher car_front_obstacles_marker_pub_;
  ros::Publisher car_front_roi_marker_pub_;

  // Mission 21: Angle Parking Publishers
  ros::Publisher pointcloud_pub_angle_;
  ros::Publisher angled_front_point_pub_;
  ros::Publisher next_front_point_pub_;
  ros::Publisher fourth_roi_point_pub_;
  ros::Publisher idx_pub_;
  ros::Publisher is_empty_;
  ros::Publisher marker_pub2_;
  ros::Publisher marker_pub3_;
  ros::Publisher marker_pub4_;
  ros::Publisher marker_pub5_;
  ros::Publisher marker_pub6_;
  ros::Publisher marker_pub_angle_;
  ros::Publisher other_type_pub_;
  ros::Publisher two_roi_point_pub_;
  ros::Publisher all_roi_point_pub_;

  ros::Subscriber LiDAR_sub_;

  ros::Subscriber planning_mission_sub;
  ros::Subscriber mission100_lane_sub_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  double mission100_roi_rear_;
  double mission100_roi_front_;
  double mission100_roi_left_min_;
  double mission100_roi_left_max_;
  double mission100_roi_z_min_;
  double mission100_roi_z_max_;
  double mission100_voxel_size_;
  double mission100_cluster_tolerance_;
  int mission100_min_cluster_size_;
  double mission100_track_match_distance_;
  double mission100_track_timeout_sec_;
  int mission100_track_max_missed_scans_;
  double mission100_velocity_filter_alpha_;
  double mission100_min_velocity_dt_sec_;
  double mission100_max_abs_relative_velocity_mps_;

  double car_front_roi_rear_;
  double car_front_roi_front_;
  double car_front_roi_y_min_;
  double car_front_roi_y_max_;
  double car_front_roi_z_min_;
  double car_front_roi_z_max_;
  double car_front_voxel_size_;
  double car_front_cluster_tolerance_;
  int car_front_min_cluster_size_;
  int car_front_max_cluster_size_;
  double car_front_min_width_y_;
  double car_front_min_height_z_;
  double car_front_no_detection_distance_;

  struct Mission100ObstacleDetection
  {
    geometry_msgs::Point center;
    geometry_msgs::Point min_point;
    geometry_msgs::Point max_point;
  };

  struct Mission100Track
  {
    int id = 0;
    geometry_msgs::Point center;
    geometry_msgs::Point min_point;
    geometry_msgs::Point max_point;
    double relative_velocity_x = 0.0;
    double velocity_reference_x = 0.0;
    bool velocity_valid = false;
    int hit_count = 1;
    int missed_scans = 0;
    ros::Time last_seen;
    ros::Time velocity_reference_stamp;
  };

  using Mission100ClusterVector = std::vector<
      pcl::PointCloud<PointT>::Ptr,
      Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>>;

  struct Mission100TrackerState
  {
    std::vector<Mission100Track> tracks;
    int next_track_id = 1;
    ros::Time last_scan_stamp;
  };

  Mission100TrackerState mission100_left_tracker_;
  Mission100TrackerState mission100_current_front_tracker_;
  int mission100_last_lane_idx_ = -1;

  void resetMission100Tracking(Mission100TrackerState &tracker);
  bool mission100TrackingParamsValid() const;
  void updateMission100Tracks(
      const std::vector<Mission100ObstacleDetection> &detections,
      const ros::Time &scan_stamp,
      Mission100TrackerState &tracker);
  void publishMission100Tracks(
      const Mission100TrackerState &tracker,
      ros::Publisher &publisher,
      const std_msgs::Header &header,
      bool frame_valid)
  {
    constexpr std::size_t kMetadataSize = 5;
    constexpr std::size_t kTrackStride = 10;

    std_msgs::Float64MultiArray tracks_msg;
    tracks_msg.data.reserve(
        kMetadataSize +
        (frame_valid ? tracker.tracks.size() * kTrackStride : 0));

    const ros::Time stamp =
        header.stamp.isZero() ? ros::Time::now() : header.stamp;
    tracks_msg.data.push_back(1.0); // 메시지 형식 버전
    tracks_msg.data.push_back(frame_valid ? 1.0 : 0.0);
    tracks_msg.data.push_back(
        frame_valid ? static_cast<double>(tracker.tracks.size()) : 0.0);
    tracks_msg.data.push_back(static_cast<double>(stamp.sec));
    tracks_msg.data.push_back(static_cast<double>(stamp.nsec));

    if (frame_valid)
    {
      for (const auto &track : tracker.tracks)
      {
        tracks_msg.data.push_back(static_cast<double>(track.id));
        tracks_msg.data.push_back(track.center.x);
        tracks_msg.data.push_back(track.center.y);
        tracks_msg.data.push_back(track.center.z);
        tracks_msg.data.push_back(track.min_point.x);
        tracks_msg.data.push_back(track.max_point.x);
        tracks_msg.data.push_back(track.relative_velocity_x);
        tracks_msg.data.push_back(static_cast<double>(track.hit_count));
        tracks_msg.data.push_back(static_cast<double>(track.missed_scans));
        tracks_msg.data.push_back(track.velocity_valid ? 1.0 : 0.0);
      }
    }
    publisher.publish(tracks_msg);
  }

  void publishMission100Cloud(
      const pcl::PointCloud<PointT> &cloud,
      ros::Publisher &publisher,
      const std_msgs::Header &header)
  {
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(cloud, output_msg);
    output_msg.header = header;
    if (output_msg.header.frame_id.empty())
    {
      output_msg.header.frame_id = "velodyne";
    }
    publisher.publish(output_msg);
  }

  void publishMission100ObstacleMarkers(
      const std::vector<Mission100ObstacleDetection> &detections,
      ros::Publisher &publisher,
      const std_msgs::Header &header,
      const std::string &marker_namespace,
      double color_r,
      double color_g,
      double color_b)
  {
    if (publisher.getNumSubscribers() == 0)
    {
      return;
    }

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker delete_all_marker;
    delete_all_marker.header = header;
    delete_all_marker.ns = marker_namespace;
    delete_all_marker.id = 0;
    delete_all_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all_marker);

    int marker_id = 0;
    for (const auto &detection : detections)
    {
      const bool bounds_valid =
          std::isfinite(detection.min_point.x) &&
          std::isfinite(detection.min_point.y) &&
          std::isfinite(detection.min_point.z) &&
          std::isfinite(detection.max_point.x) &&
          std::isfinite(detection.max_point.y) &&
          std::isfinite(detection.max_point.z) &&
          detection.max_point.x >= detection.min_point.x &&
          detection.max_point.y >= detection.min_point.y &&
          detection.max_point.z >= detection.min_point.z;
      if (!bounds_valid)
      {
        continue;
      }

      auto obstacle_marker = util_func_.showRoi_CUBE(
          marker_namespace + "_box",
          marker_id++,
          Eigen::Vector4f(
              static_cast<float>(detection.min_point.x),
              static_cast<float>(detection.min_point.y),
              static_cast<float>(detection.min_point.z),
              0.0f),
          Eigen::Vector4f(
              static_cast<float>(detection.max_point.x),
              static_cast<float>(detection.max_point.y),
              static_cast<float>(detection.max_point.z),
              0.0f),
          header.frame_id.empty() ? "velodyne" : header.frame_id,
          color_r,
          color_g,
          color_b,
          0.75);
      obstacle_marker.header = header;
      if (obstacle_marker.header.frame_id.empty())
      {
        obstacle_marker.header.frame_id = "velodyne";
      }
      obstacle_marker.pose.orientation.w = 1.0;
      obstacle_marker.scale.x = std::max(obstacle_marker.scale.x, 0.1);
      obstacle_marker.scale.y = std::max(obstacle_marker.scale.y, 0.1);
      obstacle_marker.scale.z = std::max(obstacle_marker.scale.z, 0.1);
      obstacle_marker.lifetime = ros::Duration(0.25);
      marker_array.markers.push_back(obstacle_marker);
    }

    publisher.publish(marker_array);
  }

  Utility_Function util_func_;
  std_msgs::Float64MultiArray obj;
  int pre_class = -1;
  bool ch_time_ ;
  std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> clusters;

};


/* ╔══════════════════════════════════════════════════════════════════════════════╗
   ║                            콜백 (미션 분기)                                ║
   ╚══════════════════════════════════════════════════════════════════════════════╝ */
void LiDARBIG::LiDARCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{   
  if(planning_flag == 12)
    Find_big(msg);
  else if(planning_flag == 14)
    Find_Small(msg);
  else if(planning_flag == 22)
    U_Turn(msg);
  else if(planning_flag == 41){
    Cut_in_41(msg);
  }else if(planning_flag == 42){
    Cut_in_42(msg);
  }else if(planning_flag == 43){
    Cut_in_43(msg);
  }
  else if(planning_flag == 21){
    Find_AngleParking(msg);
  }
  else if(planning_flag == 100){
    Check_Left_Lane(msg);
    detectcarFrontVehicle(msg);
  }
}


/* ╔══════════════════════════════════════════════════════════════════════════════╗
   ║                    Mission 100 : 좌측 차선 변경 공간 확인                  ║
   ╚══════════════════════════════════════════════════════════════════════════════╝ */
void LiDARBIG::resetMission100Tracking(
    Mission100TrackerState &tracker)
{
  tracker.tracks.clear();
  tracker.next_track_id = 1;
  tracker.last_scan_stamp = ros::Time(0);
}

bool LiDARBIG::mission100TrackingParamsValid() const
{
  return mission100_track_match_distance_ > 0.0 &&
         mission100_track_timeout_sec_ > 0.0 &&
         mission100_track_max_missed_scans_ >= 0 &&
         mission100_velocity_filter_alpha_ > 0.0 &&
         mission100_velocity_filter_alpha_ <= 1.0 &&
         mission100_min_velocity_dt_sec_ > 0.0 &&
         mission100_min_velocity_dt_sec_ <
             mission100_track_timeout_sec_ &&
         mission100_max_abs_relative_velocity_mps_ > 0.0;
}

void LiDARBIG::updateMission100Tracks(
    const std::vector<Mission100ObstacleDetection> &detections,
    const ros::Time &scan_stamp,
    Mission100TrackerState &tracker)
{
  tracker.tracks.erase(
      std::remove_if(
          tracker.tracks.begin(),
          tracker.tracks.end(),
          [this, &scan_stamp](const Mission100Track &track)
          {
            const double age = (scan_stamp - track.last_seen).toSec();
            return age < 0.0 || age > mission100_track_timeout_sec_;
          }),
      tracker.tracks.end());

  struct MatchCandidate
  {
    std::size_t track_index;
    std::size_t detection_index;
    double distance;
  };

  std::vector<MatchCandidate> candidates;
  for (std::size_t track_index = 0;
       track_index < tracker.tracks.size();
       ++track_index)
  {
    const auto &track = tracker.tracks[track_index];
    const double dt = (scan_stamp - track.last_seen).toSec();
    const double predicted_x =
        track.center.x +
        (track.velocity_valid ? track.relative_velocity_x * dt : 0.0);
    for (std::size_t detection_index = 0;
         detection_index < detections.size();
         ++detection_index)
    {
      const double dx =
          detections[detection_index].center.x - predicted_x;
      const double dy =
          detections[detection_index].center.y - track.center.y;
      const double distance = std::hypot(dx, dy);
      if (distance <= mission100_track_match_distance_)
      {
        candidates.push_back(
            {track_index, detection_index, distance});
      }
    }
  }

  std::sort(
      candidates.begin(), candidates.end(),
      [](const MatchCandidate &lhs, const MatchCandidate &rhs)
      {
        return lhs.distance < rhs.distance;
      });

  std::vector<bool> track_matched(
      tracker.tracks.size(), false);
  std::vector<bool> detection_matched(detections.size(), false);

  for (const auto &candidate : candidates)
  {
    if (track_matched[candidate.track_index] ||
        detection_matched[candidate.detection_index])
    {
      continue;
    }

    auto &track = tracker.tracks[candidate.track_index];
    const auto &detection = detections[candidate.detection_index];
    const double velocity_dt =
        (scan_stamp - track.velocity_reference_stamp).toSec();

    if (velocity_dt >= mission100_min_velocity_dt_sec_ &&
        velocity_dt <= mission100_track_timeout_sec_)
    {
      const double measured_velocity =
          (detection.center.x - track.velocity_reference_x) /
          velocity_dt;
      if (std::isfinite(measured_velocity) &&
          std::abs(measured_velocity) <=
              mission100_max_abs_relative_velocity_mps_)
      {
        if (track.velocity_valid)
        {
          track.relative_velocity_x =
              mission100_velocity_filter_alpha_ * measured_velocity +
              (1.0 - mission100_velocity_filter_alpha_) *
                  track.relative_velocity_x;
        }
        else
        {
          track.relative_velocity_x = measured_velocity;
        }
        track.velocity_valid = true;
        track.hit_count = std::min(track.hit_count + 1, 1000000);
      }
      else
      {
        track.relative_velocity_x = 0.0;
        track.velocity_valid = false;
        track.hit_count = 1;
      }
      track.velocity_reference_x = detection.center.x;
      track.velocity_reference_stamp = scan_stamp;
    }

    track.center = detection.center;
    track.min_point = detection.min_point;
    track.max_point = detection.max_point;
    track.last_seen = scan_stamp;
    track.missed_scans = 0;
    track_matched[candidate.track_index] = true;
    detection_matched[candidate.detection_index] = true;
  }

  for (std::size_t track_index = 0;
       track_index < tracker.tracks.size();
       ++track_index)
  {
    if (!track_matched[track_index])
    {
      ++tracker.tracks[track_index].missed_scans;
    }
  }

  for (std::size_t detection_index = 0;
       detection_index < detections.size();
       ++detection_index)
  {
    if (detection_matched[detection_index])
    {
      continue;
    }

    Mission100Track new_track;
    new_track.id = tracker.next_track_id++;
    new_track.center = detections[detection_index].center;
    new_track.min_point = detections[detection_index].min_point;
    new_track.max_point = detections[detection_index].max_point;
    new_track.velocity_reference_x =
        detections[detection_index].center.x;
    new_track.last_seen = scan_stamp;
    new_track.velocity_reference_stamp = scan_stamp;
    tracker.tracks.push_back(new_track);
  }

  tracker.tracks.erase(
      std::remove_if(
          tracker.tracks.begin(),
          tracker.tracks.end(),
          [this, &scan_stamp](const Mission100Track &track)
          {
            const double age = (scan_stamp - track.last_seen).toSec();
            return track.missed_scans >
                       mission100_track_max_missed_scans_ ||
                   age < 0.0 || age > mission100_track_timeout_sec_;
          }),
      tracker.tracks.end());
}

void LiDARBIG::Check_Left_Lane(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std_msgs::Header cloud_header;
  cloud_header.stamp = ros::Time::now();
  cloud_header.frame_id = "velodyne";
  if (msg)
  {
    cloud_header = msg->header;
    if (cloud_header.frame_id.empty())
    {
      cloud_header.frame_id = "velodyne";
    }
  }
  if (cloud_header.stamp.isZero())
  {
    cloud_header.stamp = ros::Time::now();
  }

  pcl::PointCloud<PointT> roi_visualization_cloud;
  pcl::PointCloud<PointT> cluster_visualization_cloud;
  const bool roi_visualization_requested =
      mission100_left_roi_pub_.getNumSubscribers() > 0;
  const bool cluster_visualization_requested =
      mission100_left_clusters_pub_.getNumSubscribers() > 0;

  const bool params_valid =
      mission100_roi_rear_ >= 0.0 &&
      mission100_roi_front_ > 0.0 &&
      mission100_roi_left_min_ >= 0.0 &&
      mission100_roi_left_max_ > mission100_roi_left_min_ &&
      mission100_roi_z_max_ > mission100_roi_z_min_ &&
      mission100_voxel_size_ > 0.0 &&
      mission100_cluster_tolerance_ > 0.0 &&
      mission100_min_cluster_size_ >= 1 &&
      mission100TrackingParamsValid();
  if (!params_valid)
  {
    resetMission100Tracking(mission100_left_tracker_);
    std_msgs::Bool clear_msg;
    clear_msg.data = false;
    left_lane_clear_pub_.publish(clear_msg);
    publishMission100Tracks(
        mission100_left_tracker_,
        mission100_left_tracks_pub_,
        cloud_header,
        false);
    if (roi_visualization_requested)
    {
      publishMission100Cloud(
          roi_visualization_cloud, mission100_left_roi_pub_, cloud_header);
    }
    if (cluster_visualization_requested)
    {
      publishMission100Cloud(
          cluster_visualization_cloud,
          mission100_left_clusters_pub_,
          cloud_header);
    }

    publishMission100ObstacleMarkers(
        std::vector<Mission100ObstacleDetection>(),
        mission100_left_obstacles_marker_pub_,
        cloud_header,
        "mission100_left_obstacles",
        1.0, 0.0, 1.0);
    visualization_msgs::Marker delete_marker;
    delete_marker.header = cloud_header;
    delete_marker.ns = "mission100_left_roi";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETE;
    mission100_left_roi_marker_pub_.publish(delete_marker);

    ROS_ERROR_THROTTLE(1.0, "[LiDAR][Mission100] Invalid ROI/cluster parameters");
    return;
  }

  if (!mission100_left_tracker_.last_scan_stamp.isZero() &&
      cloud_header.stamp <= mission100_left_tracker_.last_scan_stamp)
  {
    resetMission100Tracking(mission100_left_tracker_);
    std_msgs::Bool clear_msg;
    clear_msg.data = false;
    left_lane_clear_pub_.publish(clear_msg);
    publishMission100Tracks(
        mission100_left_tracker_,
        mission100_left_tracks_pub_,
        cloud_header,
        false);
    publishMission100ObstacleMarkers(
        std::vector<Mission100ObstacleDetection>(),
        mission100_left_obstacles_marker_pub_,
        cloud_header,
        "mission100_left_obstacles",
        1.0, 0.0, 1.0);
    ROS_WARN_THROTTLE(
        1.0,
        "[LiDAR][Mission100] Non-monotonic LiDAR timestamp; tracking reset");
    return;
  }
  mission100_left_tracker_.last_scan_stamp = cloud_header.stamp;

  if (mission100_left_roi_marker_pub_.getNumSubscribers() > 0)
  {
    auto roi_marker = util_func_.showRoi_CUBE(
        "mission100_left_roi",
        0,
        Eigen::Vector4f(
            static_cast<float>(-mission100_roi_rear_),
            static_cast<float>(mission100_roi_left_min_),
            static_cast<float>(mission100_roi_z_min_),
            0.0f),
        Eigen::Vector4f(
            static_cast<float>(mission100_roi_front_),
            static_cast<float>(mission100_roi_left_max_),
            static_cast<float>(mission100_roi_z_max_),
            0.0f),
        cloud_header.frame_id,
        0.0,
        1.0,
        0.0,
        0.3);
    roi_marker.header = cloud_header;
    roi_marker.pose.orientation.w = 1.0;
    mission100_left_roi_marker_pub_.publish(roi_marker);
  }

  bool valid_frame_processed = false;
  bool roi_is_empty = false;
  std::vector<Mission100ObstacleDetection> detections;

  if (msg && msg->width > 0 && msg->height > 0 && !msg->data.empty())
  {
    auto cloud = util_func_.PointCloudROS(msg);
    pcl::PointCloud<PointT>::Ptr valid_cloud(new pcl::PointCloud<PointT>);
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*cloud, *valid_cloud, valid_indices);

    if (!valid_cloud->empty())
    {
      const float voxel_size = static_cast<float>(mission100_voxel_size_);
      auto cloud_down = util_func_.DownSamplingVoxel(
          valid_cloud, voxel_size, voxel_size, voxel_size);

      if (!cloud_down->empty())
      {
        valid_frame_processed = true;
        auto roi_cloud = util_func_.CropFilter(
            cloud_down,
            Eigen::Vector4f(
                static_cast<float>(-mission100_roi_rear_),
                static_cast<float>(mission100_roi_left_min_),
                static_cast<float>(mission100_roi_z_min_),
                0.0f),
            Eigen::Vector4f(
                static_cast<float>(mission100_roi_front_),
                static_cast<float>(mission100_roi_left_max_),
                static_cast<float>(mission100_roi_z_max_),
                0.0f));
        if (roi_visualization_requested)
        {
          roi_visualization_cloud = *roi_cloud;
        }

        pcl::PointCloud<PointT>::Ptr xy_cloud(
            new pcl::PointCloud<PointT>(*roi_cloud));
        for (auto &point : xy_cloud->points)
        {
          point.z = 0.0f;
        }

        std::vector<pcl::PointIndices> cluster_indices;
        if (!xy_cloud->empty())
        {
          cluster_indices = util_func_.ClusterEuclidean(
              xy_cloud,
              false,
              static_cast<float>(mission100_cluster_tolerance_),
              mission100_min_cluster_size_,
              0);
        }
        roi_is_empty = cluster_indices.empty();

        Mission100ClusterVector detected_clusters;
        util_func_.ExtractClusters(
            cluster_visualization_cloud,
            roi_cloud,
            cluster_indices,
            detected_clusters);

        detections.reserve(detected_clusters.size());
        for (std::size_t index = 0;
             index < detected_clusters.size();
             ++index)
        {
          Mission100ObstacleDetection detection;
          // 기존 공용 대표점/최소·최대점 함수를 그대로 재사용한다.
          util_func_.GetSenCloudPoint(
              detected_clusters, detection.center, index);
          util_func_.GetMinMaxCloudPoint(
              detected_clusters,
              detection.min_point,
              detection.max_point,
              index);
          detections.push_back(detection);
        }
      }
    }
  }

  if (valid_frame_processed)
  {
    updateMission100Tracks(
        detections,
        cloud_header.stamp,
        mission100_left_tracker_);
    publishMission100Tracks(
        mission100_left_tracker_,
        mission100_left_tracks_pub_,
        cloud_header,
        true);
  }
  else
  {
    resetMission100Tracking(mission100_left_tracker_);
    publishMission100Tracks(
        mission100_left_tracker_,
        mission100_left_tracks_pub_,
        cloud_header,
        false);
  }

  std_msgs::Bool clear_msg;
  // 기존 토픽은 하위 호환을 위해 “클러스터가 완전히 없음” 의미를 유지한다.
  clear_msg.data = valid_frame_processed && roi_is_empty;
  left_lane_clear_pub_.publish(clear_msg);
  publishMission100ObstacleMarkers(
      detections,
      mission100_left_obstacles_marker_pub_,
      cloud_header,
      "mission100_left_obstacles",
      1.0, 0.0, 1.0);
  if (roi_visualization_requested)
  {
    publishMission100Cloud(
        roi_visualization_cloud, mission100_left_roi_pub_, cloud_header);
  }
  if (cluster_visualization_requested)
  {
    publishMission100Cloud(
        cluster_visualization_cloud,
        mission100_left_clusters_pub_,
        cloud_header);
  }
}

/* ╔══════════════════════════════════════════════════════════════════════════════╗
   ║                  Mission 100 : 선행 차량 후면 거리 검출                   ║
   ╚══════════════════════════════════════════════════════════════════════════════╝ */
void LiDARBIG::detectcarFrontVehicle(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std_msgs::Header cloud_header;
  cloud_header.stamp = ros::Time::now();
  cloud_header.frame_id = "velodyne";
  if (msg)
  {
    cloud_header = msg->header;
    if (cloud_header.frame_id.empty())
    {
      cloud_header.frame_id = "velodyne";
    }
  }
  if (cloud_header.stamp.isZero())
  {
    cloud_header.stamp = ros::Time::now();
  }

  const auto publish_distance =
      [this](float distance)
      {
        std_msgs::Float32 distance_msg;
        distance_msg.data = distance;
        car_front_car_distance_pub_.publish(distance_msg);
      };

  pcl::PointCloud<PointT> roi_visualization_cloud;
  pcl::PointCloud<PointT> cluster_visualization_cloud;
  const bool roi_visualization_requested =
      car_front_roi_pub_.getNumSubscribers() > 0;
  const bool cluster_visualization_requested =
      car_front_clusters_pub_.getNumSubscribers() > 0;

  const double roi_width_y =
      car_front_roi_y_max_ - car_front_roi_y_min_;
  const double roi_height_z =
      car_front_roi_z_max_ - car_front_roi_z_min_;
  const bool params_valid =
      car_front_roi_rear_ >= 0.0 &&
      car_front_roi_front_ > 0.0 &&
      car_front_roi_y_max_ > car_front_roi_y_min_ &&
      car_front_roi_z_max_ > car_front_roi_z_min_ &&
      car_front_voxel_size_ > 0.0 &&
      car_front_cluster_tolerance_ > 0.0 &&
      car_front_min_cluster_size_ >= 1 &&
      car_front_max_cluster_size_ >= car_front_min_cluster_size_ &&
      car_front_min_width_y_ > 0.0 &&
      car_front_min_width_y_ <= roi_width_y &&
      car_front_min_height_z_ > 0.0 &&
      car_front_min_height_z_ <= roi_height_z &&
      car_front_no_detection_distance_ >= car_front_roi_front_ &&
      mission100TrackingParamsValid();

  if (!params_valid)
  {
    resetMission100Tracking(mission100_current_front_tracker_);
    publishMission100Tracks(
        mission100_current_front_tracker_,
        mission100_current_front_tracks_pub_,
        cloud_header,
        false);
    publish_distance(0.0f);
    if (roi_visualization_requested)
    {
      publishMission100Cloud(
          roi_visualization_cloud, car_front_roi_pub_, cloud_header);
    }
    if (cluster_visualization_requested)
    {
      publishMission100Cloud(
          cluster_visualization_cloud,
          car_front_clusters_pub_,
          cloud_header);
    }

    publishMission100ObstacleMarkers(
        std::vector<Mission100ObstacleDetection>(),
        car_front_obstacles_marker_pub_,
        cloud_header,
        "car_front_obstacles",
        1.0, 0.1, 0.0);
    visualization_msgs::Marker delete_marker;
    delete_marker.header = cloud_header;
    delete_marker.ns = "car_front_roi";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETE;
    car_front_roi_marker_pub_.publish(delete_marker);
    ROS_ERROR_THROTTLE(
        1.0, "[LiDAR][Mission100][FrontVehicle] Invalid parameters");
    return;
  }

  if (!mission100_current_front_tracker_.last_scan_stamp.isZero() &&
      cloud_header.stamp <=
          mission100_current_front_tracker_.last_scan_stamp)
  {
    resetMission100Tracking(mission100_current_front_tracker_);
    publishMission100Tracks(
        mission100_current_front_tracker_,
        mission100_current_front_tracks_pub_,
        cloud_header,
        false);
    publish_distance(0.0f);
    publishMission100ObstacleMarkers(
        std::vector<Mission100ObstacleDetection>(),
        car_front_obstacles_marker_pub_,
        cloud_header,
        "car_front_obstacles",
        1.0, 0.1, 0.0);
    ROS_WARN_THROTTLE(
        1.0,
        "[LiDAR][Mission100][FrontVehicle] "
        "Non-monotonic LiDAR timestamp; tracking reset");
    return;
  }
  mission100_current_front_tracker_.last_scan_stamp =
      cloud_header.stamp;

  if (car_front_roi_marker_pub_.getNumSubscribers() > 0)
  {
    auto roi_marker = util_func_.showRoi_CUBE(
        "car_front_roi",
        0,
        Eigen::Vector4f(
            static_cast<float>(-car_front_roi_rear_),
            static_cast<float>(car_front_roi_y_min_),
            static_cast<float>(car_front_roi_z_min_),
            0.0f),
        Eigen::Vector4f(
            static_cast<float>(car_front_roi_front_),
            static_cast<float>(car_front_roi_y_max_),
            static_cast<float>(car_front_roi_z_max_),
            0.0f),
        cloud_header.frame_id,
        0.0,
        0.5,
        1.0,
        0.3);
    roi_marker.header = cloud_header;
    roi_marker.pose.orientation.w = 1.0;
    car_front_roi_marker_pub_.publish(roi_marker);
  }

  bool valid_frame_processed = false;
  std::vector<Mission100ObstacleDetection> detections;
  float nearest_vehicle_distance =
      static_cast<float>(car_front_no_detection_distance_);

  if (msg && msg->width > 0 && msg->height > 0 && !msg->data.empty())
  {
    auto cloud = util_func_.PointCloudROS(msg);
    pcl::PointCloud<PointT>::Ptr valid_cloud(new pcl::PointCloud<PointT>);
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*cloud, *valid_cloud, valid_indices);

    if (!valid_cloud->empty())
    {
      const float voxel_size =
          static_cast<float>(car_front_voxel_size_);
      auto cloud_down = util_func_.DownSamplingVoxel(
          valid_cloud, voxel_size, voxel_size, voxel_size);

      if (!cloud_down->empty())
      {
        valid_frame_processed = true;
        auto roi_cloud = util_func_.CropFilter(
            cloud_down,
            Eigen::Vector4f(
                static_cast<float>(-car_front_roi_rear_),
                static_cast<float>(car_front_roi_y_min_),
                static_cast<float>(car_front_roi_z_min_),
                0.0f),
            Eigen::Vector4f(
                static_cast<float>(car_front_roi_front_),
                static_cast<float>(car_front_roi_y_max_),
                static_cast<float>(car_front_roi_z_max_),
                0.0f));

        if (roi_visualization_requested)
        {
          roi_visualization_cloud = *roi_cloud;
        }

        if (!roi_cloud->empty())
        {
          // 높이는 형상 검사에 남겨 두고, 별도 복사본의 Z만 0으로 만들어
          // 선행 차량 후면을 XY 기준으로 클러스터링한다.
          pcl::PointCloud<PointT>::Ptr xy_cloud(
              new pcl::PointCloud<PointT>(*roi_cloud));
          for (auto &point : xy_cloud->points)
          {
            point.z = 0.0f;
          }

          const auto cluster_indices = util_func_.ClusterEuclidean(
              xy_cloud,
              true,
              static_cast<float>(car_front_cluster_tolerance_),
              car_front_min_cluster_size_,
              car_front_max_cluster_size_);

          int accepted_cluster_id = 1;
          for (const auto &cluster_indices_item : cluster_indices)
          {
            if (cluster_indices_item.indices.empty())
            {
              continue;
            }

            float min_x = std::numeric_limits<float>::infinity();
            float min_y = std::numeric_limits<float>::infinity();
            float min_z = std::numeric_limits<float>::infinity();
            float max_x = -std::numeric_limits<float>::infinity();
            float max_y = -std::numeric_limits<float>::infinity();
            float max_z = -std::numeric_limits<float>::infinity();

            for (const int point_idx : cluster_indices_item.indices)
            {
              const PointT &point = roi_cloud->points[point_idx];
              min_x = std::min(min_x, point.x);
              min_y = std::min(min_y, point.y);
              min_z = std::min(min_z, point.z);
              max_x = std::max(max_x, point.x);
              max_y = std::max(max_y, point.y);
              max_z = std::max(max_z, point.z);
            }

            const float width_y = max_y - min_y;
            const float height_z = max_z - min_z;
            if (width_y < static_cast<float>(car_front_min_width_y_) ||
                height_z < static_cast<float>(car_front_min_height_z_))
            {
              continue;
            }

            nearest_vehicle_distance = std::min(
                nearest_vehicle_distance,
                std::max(0.0f, min_x));

            Mission100ObstacleDetection detection;
            detection.min_point.x = min_x;
            detection.min_point.y = min_y;
            detection.min_point.z = min_z;
            detection.max_point.x = max_x;
            detection.max_point.y = max_y;
            detection.max_point.z = max_z;
            detection.center.x =
                0.5 * (detection.min_point.x + detection.max_point.x);
            detection.center.y =
                0.5 * (detection.min_point.y + detection.max_point.y);
            detection.center.z =
                0.5 * (detection.min_point.z + detection.max_point.z);
            detections.push_back(detection);

            if (cluster_visualization_requested)
            {
              for (const int point_idx : cluster_indices_item.indices)
              {
                PointT visualization_point =
                    roi_cloud->points[point_idx];
                visualization_point.intensity =
                    static_cast<float>(accepted_cluster_id);
                cluster_visualization_cloud.push_back(
                    visualization_point);
              }
            }
            ++accepted_cluster_id;
          }
        }
      }
    }
  }

  if (!valid_frame_processed)
  {
    // 손상되거나 비어 있는 입력 프레임은 전방 거리 0m로 처리해 정지시킨다.
    nearest_vehicle_distance = 0.0f;
    resetMission100Tracking(mission100_current_front_tracker_);
    publishMission100Tracks(
        mission100_current_front_tracker_,
        mission100_current_front_tracks_pub_,
        cloud_header,
        false);
    ROS_WARN_THROTTLE(
        1.0, "[LiDAR][Mission100][FrontVehicle] Invalid point cloud");
  }
  else
  {
    updateMission100Tracks(
        detections,
        cloud_header.stamp,
        mission100_current_front_tracker_);
    publishMission100Tracks(
        mission100_current_front_tracker_,
        mission100_current_front_tracks_pub_,
        cloud_header,
        true);
  }

  publish_distance(nearest_vehicle_distance);
  publishMission100ObstacleMarkers(
      detections,
      car_front_obstacles_marker_pub_,
      cloud_header,
      "car_front_obstacles",
      1.0, 0.1, 0.0);
  if (roi_visualization_requested)
  {
    publishMission100Cloud(
        roi_visualization_cloud, car_front_roi_pub_, cloud_header);
  }
  if (cluster_visualization_requested)
  {
    publishMission100Cloud(
        cluster_visualization_cloud,
        car_front_clusters_pub_,
        cloud_header);
  }
}


/* ╔══════════════════════════════════════════════════════════════════════════════╗
   ║                     Mission 12 : 대형 정적 장애물 (Find_big)               ║
   ╚══════════════════════════════════════════════════════════════════════════════╝ */
void LiDARBIG::Find_big(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if (check_pointcloud == 0)
  {
    ROS_INFO("[LiDAR-Find_big] 포인트클라우드 구독 시작");
    check_pointcloud = 1;
  }
  
  // 1. 변환
  auto cloud = util_func_.PointCloudROS(msg);
  // ROS_DEBUG("[LiDAR-Find_big] 입력 포인트 수: %lu", cloud->size());
  
  // 2. 다운샘플링
  auto cloud_filtered = util_func_.DownSamplingVoxel(cloud, 0.1f, 0.1f, 0.1f);

  // 3. Crop 필터 (범위 확대)
  auto cloud_filtered2 = util_func_.CropFilter(
      cloud_filtered,
      Eigen::Vector4f(0.0, -4.5, -0.4, 0.0),
      Eigen::Vector4f(20.0, 4.5, 2.0, 0.0));

  visualization_msgs::Marker marker_roi;
  marker_roi.header.frame_id = msg->header.frame_id;
  marker_roi.header.stamp = ros::Time::now();
  marker_roi.ns = "big_object_roi";
  marker_roi.id = 0;
  marker_roi.type = visualization_msgs::Marker::CUBE;
  marker_roi.action = visualization_msgs::Marker::ADD;
  marker_roi.pose.position.x = (20.0 + 0.0) / 2.0;    
  marker_roi.pose.position.y = (4.5 + (-4.5)) / 2.0;  
  marker_roi.pose.position.z = (2.0 + (-0.4)) / 2.0; 
  marker_roi.pose.orientation.w = 1.0;
  marker_roi.scale.x = 20.0 - 0.0;    
  marker_roi.scale.y = 4.5 - (-4.5);  
  marker_roi.scale.z = 2.0 - (-0.4); 
  marker_roi.color.r = 0.0;
  marker_roi.color.g = 1.0;
  marker_roi.color.b = 0.0;
  marker_roi.color.a = 0.3; // Transparency
  marker_big_roi_pub_.publish(marker_roi);
  
  // ROS_DEBUG("[LiDAR-Find_big] Crop 필터 후 포인트 수: %lu", cloud_filtered2->size());
  
  // 4. 클러스터링
  auto cluster_indices = util_func_.ClusterEuclidean(cloud_filtered2, false, 0.7, 10, 3000);
  
  // 5. 클러스터 추출 및 색 입히기
  pcl::PointCloud<pcl::PointXYZI> TotalCloud;
  clusters.clear();
  util_func_.ExtractClusters(TotalCloud, cloud_filtered2, cluster_indices, clusters);

  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(TotalCloud, cloud_p);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_p, output);
  output.header.frame_id = "velodyne";
  big_obj_pc_pub_.publish(output);

  if (clusters.size() > 0)
  {
    for (size_t i = 0; i < clusters.size(); i++)
    {
      // cout << "find_big" << endl;
      geometry_msgs::Point center_point, min_point, max_point;
      util_func_.GetSenCloudPoint(clusters, center_point, i);
      util_func_.GetMinMaxCloudPoint(clusters, min_point, max_point, i);

      float left_x = min_point.x;
      float left_y = max_point.y;
      float right_x = min_point.x;
      float right_y = min_point.y;

      // float cen_x = center_point.x;
      // float cen_y = center_point.y;

      // 좌우(y축) 폭이 0.8m 초과 && 3.0m 미만  &&  상하(z축) 폭이 0.3m 초과 && 1.6m 미만
      if ((max_point.y - min_point.y) > 0.8 && (max_point.y - min_point.y) < 3.0 && (max_point.z - min_point.z) > 0.3 && (max_point.z - min_point.z) < 1.6)
      {
        obj.data.push_back(left_x);
        obj.data.push_back(left_y);
        obj.data.push_back(right_x);
        obj.data.push_back(right_y);
      }
    }
    if (!obj.data.empty())
    {
      big_obj_pub_.publish(obj);
      obj.data.clear();
    }
  }
}


/* ╔══════════════════════════════════════════════════════════════════════════════╗
   ║                    Mission 14 : 소형 정적 장애물 (Find_Small)              ║
   ╚══════════════════════════════════════════════════════════════════════════════╝ */
void LiDARBIG::Find_Small(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if (check_pointcloud == 0)
  {
    ROS_INFO("[LiDAR-Find_Small] 포인트클라우드 구독 시작");
    check_pointcloud = 1;
  }
  
  // 1. 변환
  auto cloud = util_func_.PointCloudROS(msg);
  // 2. 다운샘플링
  auto cloud_filtered = util_func_.DownSamplingVoxel(cloud, 0.1f, 0.1f, 0.1f);

  // 3. Crop 필터 (ROI 영역)
  auto cloud_filtered2 = util_func_.CropFilter(
      cloud_filtered,
      Eigen::Vector4f(-0, -3, -0.35, 0), // ROI 영역
      Eigen::Vector4f(15, 3, 1.0, 0));     // x, y, z, 1
  
  visualization_msgs::Marker marker_roi;
  marker_roi.header.frame_id = msg->header.frame_id;
  marker_roi.header.stamp = ros::Time::now();
  marker_roi.ns = "small_object_roi";
  marker_roi.id = 0;
  marker_roi.type = visualization_msgs::Marker::CUBE;
  marker_roi.action = visualization_msgs::Marker::ADD;
  marker_roi.pose.position.x = (15.0 - 0.0) / 2.0;    // Center X
  marker_roi.pose.position.y = (3.0 + (-3.0)) / 2.0;  // Center Y
  marker_roi.pose.position.z = (1.0 + (-0.35)) / 2.0; // Center Z
  marker_roi.pose.orientation.w = 1.0;
  marker_roi.scale.x = 15.0 - 0.0;    // Size X
  marker_roi.scale.y = 3.0 - (-3.0);  // Size Y
  marker_roi.scale.z = 1.0 - (-0.35); // Size Z
  marker_roi.color.r = 0.0;
  marker_roi.color.g = 1.0;
  marker_roi.color.b = 0.0;
  marker_roi.color.a = 0.3; // Transparency
  marker_small_roi_pub_.publish(marker_roi);

  // 4. 클러스터링
  auto cluster_indices = util_func_.ClusterEuclidean(cloud_filtered2, false, 0.3, 3, 1000);
  
  // 5. 클러스터 추출
  pcl::PointCloud<pcl::PointXYZI> TotalCloud;
  clusters.clear();
  util_func_.ExtractClusters(TotalCloud, cloud_filtered2, cluster_indices, clusters);

  visualization_msgs::Marker marker_pts;
  marker_pts.header.frame_id = msg->header.frame_id;
  marker_pts.header.stamp = ros::Time::now();
  marker_pts.ns = "small_object_center";
  marker_pts.id = 0;
  marker_pts.type = visualization_msgs::Marker::POINTS;
  marker_pts.action = visualization_msgs::Marker::ADD;
  marker_pts.pose.orientation.w = 1.0;
  
  marker_pts.scale.x = 0.5; 
  marker_pts.scale.y = 0.5;
  
  marker_pts.color.r = 1.0; 
  marker_pts.color.a = 1.0;

  if (clusters.size() > 0)
  {
      for (size_t i = 0; i < clusters.size(); i++)
      {
        geometry_msgs::Point center_point, min_point, max_point;
        util_func_.GetSenCloudPoint(clusters, center_point, i);
        util_func_.GetMinMaxCloudPoint(clusters, min_point, max_point, i);

        float cen_x = center_point.x;
        float cen_y = center_point.y;

        // 좌우(y축) 폭이 0.1m 초과 && 1.5m 미만  &&  상하(z축) 높이가 1.5m 미만
        if ((max_point.y - min_point.y) > 0.1 && (max_point.y - min_point.y) < 1.5 && max_point.z < 1.5)
        {
          obj.data.push_back(cen_x);
          obj.data.push_back(cen_y);

          geometry_msgs::Point p;
          p.x = cen_x;
          p.y = cen_y;
          p.z = center_point.z;
          marker_pts.points.push_back(p);
        }
      }
  }

  marker_small_pub_.publish(marker_pts);

  obj.data.push_back(-1000); 
  start_time_ = std::chrono::system_clock::now();
  small_obj_pub_.publish(obj);
  obj.data.clear();
}


/* ╔══════════════════════════════════════════════════════════════════════════════╗
   ║                        Mission 22 : U-Turn (U_Turn)                        ║
   ╚══════════════════════════════════════════════════════════════════════════════╝ */
void LiDARBIG::U_Turn(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
   if (check_pointcloud == 0)
    {
        ROS_INFO("LiDAR_uturn node sub pointcloud");
        check_pointcloud = 1;
    }

    // 1. 변환
    auto cloud_origin = util_func_.PointCloudROS(msg);

    // 2. 다운샘플링
    auto cloud_down = util_func_.DownSamplingVoxel(cloud_origin, 0.1f, 0.1f, 0.1f);

    // 3. Crop 필터
    auto cloud_crop = util_func_.CropFilter(cloud_down, 
                                            Eigen::Vector4f(0, -6, -0.7, 0), 
                                            Eigen::Vector4f(8, 0, 1, 0));

    visualization_msgs::Marker marker_roi;
    marker_roi.header.frame_id = msg->header.frame_id;
    marker_roi.header.stamp = ros::Time::now();
    marker_roi.ns = "uturn_roi";
    marker_roi.id = 0;
    marker_roi.type = visualization_msgs::Marker::CUBE;
    marker_roi.action = visualization_msgs::Marker::ADD;
    marker_roi.pose.position.x = (15.0 + 0.0) / 2.0;    
    marker_roi.pose.position.y = (0.0 + (-10.0)) / 2.0;  
    marker_roi.pose.position.z = (1.0 + (-0.7)) / 2.0; 
    marker_roi.pose.orientation.w = 1.0;
    marker_roi.scale.x = 15.0 - 0.0;    
    marker_roi.scale.y = 0.0 - (-10.0);  
    marker_roi.scale.z = 1.0 - (-0.7); 
    marker_roi.color.r = 0.0;
    marker_roi.color.g = 1.0;
    marker_roi.color.b = 0.0;
    marker_roi.color.a = 0.3; 
    marker_uturn_roi_pub_.publish(marker_roi);

    // 4. 클러스터링
    auto cloud_clust = util_func_.ClusterEuclidean(cloud_crop, false, 0.3, 1, 1000);

    //============================  < pointcloud 색깔 입히는 과정 >==========================
    pcl::PointCloud<pcl::PointXYZI> TotalCloud;
    clusters.clear();
    util_func_.ExtractClusters(TotalCloud, cloud_crop, cloud_clust, clusters);

    //============================< 콘 cluster >==============================================
    std_msgs::Float64MultiArray cone_center;
    geometry_msgs::Point center_point;

    visualization_msgs::Marker marker_pts;
    marker_pts.header.frame_id = msg->header.frame_id;
    marker_pts.header.stamp = ros::Time::now();
    marker_pts.ns = "uturn_points";
    marker_pts.id = 0;
    marker_pts.type = visualization_msgs::Marker::POINTS;
    marker_pts.action = visualization_msgs::Marker::ADD;
    marker_pts.pose.orientation.w = 1.0;
    marker_pts.scale.x = 0.5; marker_pts.scale.y = 0.5;
    marker_pts.color.r = 1.0; marker_pts.color.a = 1.0;

    for (size_t i = 0; i < clusters.size(); i++)
    {
        geometry_msgs::Point center_point, min_point, max_point;

        util_func_.GetMinMaxCloudPoint(clusters, min_point, max_point, i);
        util_func_.GetSenCloudPoint(clusters, center_point, i);

        // float ob_size = sqrt(pow(max_point.x - min_point.x, 2) + pow(max_point.y - min_point.y, 2));
        float z_size = (max_point.z - min_point.z);

        if ((max_point.y - min_point.y) < 0.5 && z_size > 0.2 && z_size < 0.7 && max_point.x > 0.5)
        {
            cone_center.data.push_back(center_point.x);
            cone_center.data.push_back(center_point.y);

            geometry_msgs::Point p;
            p.x = center_point.x;
            p.y = center_point.y;
            p.z = center_point.z;
            marker_pts.points.push_back(p);
            
            ROS_INFO("  >>> 콘 발견: (x: %f, y: %f)", center_point.x, center_point.y);
        }
    }
    
    marker_uturn_pts_pub_.publish(marker_pts);

    start_time_ = std::chrono::system_clock::now();
    uturn_center_pub_.publish(cone_center);
    
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(TotalCloud, cloud_p);
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "velodyne";

    start_time_ = std::chrono::system_clock::now();
    uturn_pub_.publish(output);
}


/* ╔══════════════════════════════════════════════════════════════════════════════╗
   ║                    Mission 41 : 끼어들기 구간 1 (Cut_in_41)                ║
   ╚══════════════════════════════════════════════════════════════════════════════╝ */
void LiDARBIG::Cut_in_41(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if (check_pointcloud == 0)
  {
    ROS_INFO("[LiDAR-Cut_in_41] 포인트클라우드 구독 시작");
    check_pointcloud = 1;
  }
  // 1. 변환
  auto cloud = util_func_.PointCloudROS(msg);

  // 2. 다운샘플링
  auto cloud_filtered = util_func_.DownSamplingVoxel(cloud, 0.1f, 0.1f, 0.1f);
  // 3. Crop 필터
  auto cloud_filtered2 = util_func_.CropFilter(
  cloud_filtered,
  Eigen::Vector4f(0.0, -9, -0.55, 0.0),
  Eigen::Vector4f(9.0, 12, 1.3, 0.0));
  
  if (cloud_filtered2->empty()) {
     // No points -> No clusters
     clusters.clear();
  } else {
    // 4. 클러스터링
    auto cluster_indices = util_func_.ClusterEuclidean(cloud_filtered2, false, 0.7, 10, 3000);
    // 5. 클러스터 추출 및 색 입히기
    pcl::PointCloud<pcl::PointXYZI> TotalCloud;
    clusters.clear();
    util_func_.ExtractClusters(TotalCloud, cloud_filtered2, cluster_indices, clusters);
  }
  
  ROS_DEBUG("[LiDAR-Cut_in_41] 클러스터 수: %lu", clusters.size());

  if(ch_time_){
    start_time_ = std::chrono::system_clock::now();
    ch_time_ = false;
  }
  if (clusters.size() == 0)
  { 
    std::chrono::system_clock::time_point wait_time_ = std::chrono::system_clock::now();
    cut_in_cheack_time = wait_time_ - start_time_;
    ROS_DEBUG("[LiDAR-Cut_in_41] 클러스터 없음: %.1f초", cut_in_cheack_time.count());
    if(cut_in_cheack_time.count() > 3.0) {
        ROS_INFO("[LiDAR-Cut_in_41] 3초 경과 - 통과 신호 발송");
        std_msgs::Bool truth;
        truth.data = true;
        cut_in_ok.publish(truth);
    }
  }else
  {
    start_time_ = std::chrono::system_clock::now();
    std_msgs::Bool truth;
    truth.data = false;
    cut_in_ok.publish(truth);
    ch_time_ = true;
  }
}


/* ╔══════════════════════════════════════════════════════════════════════════════╗
   ║                    Mission 42 : 끼어들기 구간 2 (Cut_in_42)                ║
   ╚══════════════════════════════════════════════════════════════════════════════╝ */
void LiDARBIG::Cut_in_42(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if (check_pointcloud == 0)
  {
    ROS_INFO("[LiDAR-Cut_in_42] 포인트클라우드 구독 시작");
    check_pointcloud = 1;
  }
  // 1. 변환
  auto cloud = util_func_.PointCloudROS(msg);
  // 2. 다운샘플링
  auto cloud_filtered = util_func_.DownSamplingVoxel(cloud, 0.1f, 0.1f, 0.1f);
  // 3. Crop 필터
  auto cloud_filtered2 = util_func_.CropFilter(
  cloud_filtered,
  Eigen::Vector4f(0.0, -12, -0.55, 0.0),
  Eigen::Vector4f(12.0, 5, 1.3, 0.0));
  
  if (cloud_filtered2->empty()) {
     clusters.clear();
  } else {
    // 4. 클러스터링
    auto cluster_indices = util_func_.ClusterEuclidean(cloud_filtered2, false, 0.7, 10, 3000);
    // 5. 클러스터 추출 및 색 입히기
    pcl::PointCloud<pcl::PointXYZI> TotalCloud;
    clusters.clear();
    util_func_.ExtractClusters(TotalCloud, cloud_filtered2, cluster_indices, clusters);
  }
  
  ROS_DEBUG("[LiDAR-Cut_in_42] 클러스터 수: %lu", clusters.size());

  if(ch_time_){
    start_time_ = std::chrono::system_clock::now();
    ch_time_ = false;
  }
  if (clusters.size() == 0)
  { 
    std::chrono::system_clock::time_point wait_time_ = std::chrono::system_clock::now();
    cut_in_cheack_time = wait_time_ - start_time_;
    ROS_DEBUG("[LiDAR-Cut_in_42] 클러스터 없음: %.1f초", cut_in_cheack_time.count());
    if(cut_in_cheack_time.count() > 3.0) {
        ROS_INFO("[LiDAR-Cut_in_42] 3초 경과 - 통과 신호 발송");
        std_msgs::Bool truth;
        truth.data = true;
        cut_in_ok.publish(truth);
    }
  }else
  {
    start_time_ = std::chrono::system_clock::now();
    std_msgs::Bool truth;
    truth.data = false;
    cut_in_ok.publish(truth);
    ch_time_ = true;
  }
}


/* ╔══════════════════════════════════════════════════════════════════════════════╗
   ║                    Mission 43 : 끼어들기 구간 3 (Cut_in_43)                ║
   ╚══════════════════════════════════════════════════════════════════════════════╝ */
void LiDARBIG::Cut_in_43(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if (check_pointcloud == 0)
  {
    ROS_INFO("[LiDAR-Cut_in_43] 포인트클라우드 구독 시작");
    check_pointcloud = 1;
  }
  // 1. 변환
  auto cloud = util_func_.PointCloudROS(msg);
  // 2. 다운샘플링
  auto cloud_filtered = util_func_.DownSamplingVoxel(cloud, 0.1f, 0.1f, 0.1f);
  // 3. Crop 필터
  auto cloud_filtered2 = util_func_.CropFilter(
  cloud_filtered,
  Eigen::Vector4f(7.0, -19, -0.55, 0.0),
  Eigen::Vector4f(34.0, 16, 1.3, 0.0));
  
  if (cloud_filtered2->empty()) {
     clusters.clear();
  } else {
    // 4. 클러스터링
    auto cluster_indices = util_func_.ClusterEuclidean(cloud_filtered2, false, 0.7, 10, 3000);
    // 5. 클러스터 추출 및 색 입히기
    pcl::PointCloud<pcl::PointXYZI> TotalCloud;
    clusters.clear();
    util_func_.ExtractClusters(TotalCloud, cloud_filtered2, cluster_indices, clusters);
  }
  
  ROS_DEBUG("[LiDAR-Cut_in_43] 클러스터 수: %lu", clusters.size());

  if(ch_time_){
    start_time_ = std::chrono::system_clock::now();
    ch_time_ = false;
  }
  if (clusters.size() == 0)
  { 
    std::chrono::system_clock::time_point wait_time_ = std::chrono::system_clock::now();
    cut_in_cheack_time = wait_time_ - start_time_;
    std::cout << "얼마나 비어있냐 : " << cut_in_cheack_time.count() << "s\n";
    // std::cout << "type : " << cut_in_cheack_time.count().type() << "s\n";
    if(cut_in_cheack_time.count() > 3.0) {
        std::cout << "3초 경과" << std::endl;
        std_msgs::Bool truth;
        truth.data = true;
        cut_in_ok.publish(truth);
    }
  }else
  {
    start_time_ = std::chrono::system_clock::now();
    std_msgs::Bool truth;
    truth.data = false;
    cut_in_ok.publish(truth);
    ch_time_ = true;
  }
}


/* ╔══════════════════════════════════════════════════════════════════════════════╗
   ║                 Mission 21 : 사선 주차 (Find_AngleParking)                 ║
   ╚══════════════════════════════════════════════════════════════════════════════╝ */
void LiDARBIG::Find_AngleParking(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*input, *cloud);

    pcl::VoxelGrid<PointT> vg;
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.filter(*cloud_filtered);

    // -------------------- Ground Removal --------------------
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr ground_coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.15); 
    seg.setInputCloud(cloud_filtered);
    seg.segment(*ground_inliers, *ground_coefficients);

    if (!ground_inliers->indices.empty())
    {
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(ground_inliers);
        extract.setNegative(true); 
        pcl::PointCloud<PointT>::Ptr cloud_no_ground(new pcl::PointCloud<PointT>);
        extract.filter(*cloud_no_ground);
        cloud_filtered = cloud_no_ground;
    }

    float z_min = -0.8; // 더 넓은 z 범위로 조정
    float z_max = 0.3;

    // -------------------- ROI 1 --------------------
    pcl::PointCloud<PointT>::Ptr hull_points1(new pcl::PointCloud<PointT>);
    PointT p1, p2, p3, p4, p5, p6, p7, p8;

    // ROI1: 기존보다 x, y 범위 확장 (예시)
    float x1 = 5.6, y1 = -2.2;
    float x2 = 5.9, y2 = -2.9;
    float x3 = 3.3, y3 = -4.3;
    float x4 = 3.0, y4 = -3.7;

    p1.x = x1; p1.y = y1; p1.z = z_min;
    p2.x = x2; p2.y = y2; p2.z = z_min;
    p3.x = x3; p3.y = y3; p3.z = z_min;
    p4.x = x4; p4.y = y4; p4.z = z_min;

    p5 = p1; p5.z = z_max;
    p6 = p2; p6.z = z_max;
    p7 = p3; p7.z = z_max;
    p8 = p4; p8.z = z_max;

    hull_points1->push_back(p1); hull_points1->push_back(p2); hull_points1->push_back(p3); hull_points1->push_back(p4);
    hull_points1->push_back(p5); hull_points1->push_back(p6); hull_points1->push_back(p7); hull_points1->push_back(p8);

    std::vector<pcl::Vertices> hull_indices1;
    define_hull_indices(hull_indices1, hull_points1);

    pcl::CropHull<PointT> crop_hull1;
    crop_hull1.setInputCloud(cloud_filtered);
    crop_hull1.setHullCloud(hull_points1);
    crop_hull1.setHullIndices(hull_indices1);
    crop_hull1.setDim(3);

    pcl::PointCloud<PointT>::Ptr roi_cloud1(new pcl::PointCloud<PointT>);
    crop_hull1.filter(*roi_cloud1);

    visualize_roi(hull_points1, "roi_1");

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(roi_cloud1);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.4);
    ec.setMinClusterSize(4);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(roi_cloud1);
    ec.extract(cluster_indices);

    clusters.clear();
    pcl::PointCloud<PointT>::Ptr merged_clusters(new pcl::PointCloud<PointT>);
    std_msgs::Float64MultiArray cluster_coordinates;

    int cluster_id = 1;
    for (auto &indices : cluster_indices)
    {
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        Eigen::Vector4f centroid;
        for (int idx : indices.indices)
        {
            PointT point = (*roi_cloud1)[idx];
            point.intensity = static_cast<float>(cluster_id);
            cluster->push_back(point);
        }
        pcl::compute3DCentroid(*cluster, centroid);
        cluster_coordinates.data.push_back(centroid[0]);
        cluster_coordinates.data.push_back(centroid[1]);
        cluster_coordinates.data.push_back(centroid[2]);

        *merged_clusters += *cluster;
        clusters.push_back(cluster);
        cluster_id++;
    }

    sensor_msgs::PointCloud2 output1;
    pcl::toROSMsg(*merged_clusters, output1);
    output1.header.frame_id = "velodyne";
    output1.header.stamp = ros::Time::now();
    pointcloud_pub_angle_.publish(output1);

    publish_cluster_centroids(clusters);
    angled_front_point_pub_.publish(cluster_coordinates);

    // -------------------- ROI 2: is_empty check --------------------
    pcl::PointCloud<PointT>::Ptr hull_points2(new pcl::PointCloud<PointT>);

    // ROI2: 기존보다 x, y 범위 확장 (예시)
    p1.x = 5.3;  p1.y = -2.6;   p1.z = z_min;
    p2.x = 5.6;  p2.y = -3.2;   p2.z = z_min;
    p3.x = 3.1;  p3.y = -4.1;   p3.z = z_min;
    p4.x = 2.8;  p4.y = -3.6;   p4.z = z_min;

    p5 = p1; p5.z = z_max;
    p6 = p2; p6.z = z_max;
    p7 = p3; p7.z = z_max;
    p8 = p4; p8.z = z_max;

    hull_points2->push_back(p1); hull_points2->push_back(p2); hull_points2->push_back(p3); hull_points2->push_back(p4);
    hull_points2->push_back(p5); hull_points2->push_back(p6); hull_points2->push_back(p7); hull_points2->push_back(p8);

    std::vector<pcl::Vertices> hull_indices2;
    define_hull_indices(hull_indices2, hull_points2);

    pcl::CropHull<PointT> crop_hull2;
    crop_hull2.setInputCloud(cloud_filtered);
    crop_hull2.setHullCloud(hull_points2);
    crop_hull2.setHullIndices(hull_indices2);
    crop_hull2.setDim(3);

    pcl::PointCloud<PointT>::Ptr roi_cloud2(new pcl::PointCloud<PointT>);
    crop_hull2.filter(*roi_cloud2);

    visualize_roi(hull_points2, "roi_2");

    std_msgs::Bool is_empty_msg;
    is_empty_msg.data = roi_cloud2->empty();
    is_empty_.publish(is_empty_msg);

    // -------------------- ROI 3 --------------------
    pcl::PointCloud<PointT>::Ptr hull_points3(new pcl::PointCloud<PointT>);

    // ROI3: 기존보다 x, y 범위 확장 (예시)
    p1.x = 8.6;  p1.y = -2.2;     p1.z = z_min;
    p2.x = 8.9;  p2.y = -2.9;     p2.z = z_min;
    p3.x = 6.3;  p3.y = -4.3;     p3.z = z_min;
    p4.x = 6.0;  p4.y = -3.7;     p4.z = z_min;

    p5 = p1; p5.z = z_max;
    p6 = p2; p6.z = z_max;
    p7 = p3; p7.z = z_max;
    p8 = p4; p8.z = z_max;

    hull_points3->push_back(p1); hull_points3->push_back(p2); hull_points3->push_back(p3); hull_points3->push_back(p4);
    hull_points3->push_back(p5); hull_points3->push_back(p6); hull_points3->push_back(p7); hull_points3->push_back(p8);

    std::vector<pcl::Vertices> hull_indices3;
    define_hull_indices(hull_indices3, hull_points3);

    pcl::CropHull<PointT> crop_hull3;
    crop_hull3.setInputCloud(cloud_filtered);
    crop_hull3.setHullCloud(hull_points3);
    crop_hull3.setHullIndices(hull_indices3);
    crop_hull3.setDim(3);

    pcl::PointCloud<PointT>::Ptr roi_cloud3(new pcl::PointCloud<PointT>);
    crop_hull3.filter(*roi_cloud3);

    visualize_roi(hull_points3, "roi_3");

    pcl::search::KdTree<PointT>::Ptr tree3(new pcl::search::KdTree<PointT>);
    tree3->setInputCloud(roi_cloud3);

    std::vector<pcl::PointIndices> cluster_indices3;
    pcl::EuclideanClusterExtraction<PointT> ec3;
    ec3.setClusterTolerance(0.4);
    ec3.setMinClusterSize(4);
    ec3.setMaxClusterSize(1000);
    ec3.setSearchMethod(tree3);
    ec3.setInputCloud(roi_cloud3);
    ec3.extract(cluster_indices3);

    pcl::PointCloud<PointT>::Ptr merged_clusters3(new pcl::PointCloud<PointT>);
    std_msgs::Float64MultiArray cluster_coordinates3;

    int cluster_id3 = 1;
    for (auto &indices : cluster_indices3)
    {
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        Eigen::Vector4f centroid;
        for (int idx : indices.indices)
        {
            PointT point = (*roi_cloud3)[idx];
            point.intensity = static_cast<float>(cluster_id3);
            cluster->push_back(point);
        }
        pcl::compute3DCentroid(*cluster, centroid);
        cluster_coordinates3.data.push_back(centroid[0]);
        cluster_coordinates3.data.push_back(centroid[1]);
        cluster_coordinates3.data.push_back(centroid[2]);

        *merged_clusters3 += *cluster;
        cluster_id3++;
    }

    sensor_msgs::PointCloud2 output3;
    pcl::toROSMsg(*merged_clusters3, output3);
    output3.header.frame_id = "velodyne";
    output3.header.stamp = ros::Time::now();
    pointcloud_pub_angle_.publish(output3);

    next_front_point_pub_.publish(cluster_coordinates3);

    // -------------------- ROI 4: Crop Box --------------------
    pcl::CropBox<PointT> crop_box;
    // ROI4: CropBox도 y, z 범위 확장 (예시)
    crop_box.setMin(Eigen::Vector4f(-1.0, -5.0, -0.8, 1.0));
    crop_box.setMax(Eigen::Vector4f(13.0, 0.0, 0.3, 1.0));
    crop_box.setInputCloud(cloud_filtered);

    pcl::PointCloud<PointT>::Ptr roi_cloud4(new pcl::PointCloud<PointT>);
    crop_box.filter(*roi_cloud4);

    visualization_msgs::Marker roi_marker;
    roi_marker.header.frame_id = "velodyne";
    roi_marker.header.stamp = ros::Time::now();
    roi_marker.ns = "roi_visualization";
    roi_marker.id = 0;
    roi_marker.type = visualization_msgs::Marker::CUBE;
    roi_marker.action = visualization_msgs::Marker::ADD;
    roi_marker.scale.x = 8.3;
    roi_marker.scale.y = 2.0;
    roi_marker.scale.z = 0.85;
    roi_marker.color.r = 0.0;
    roi_marker.color.g = 1.0;
    roi_marker.color.b = 0.0;
    roi_marker.color.a = 0.5;
    roi_marker.pose.position.x = 7.5;
    roi_marker.pose.position.y = -3.0;
    roi_marker.pose.position.z = -0.225;
    marker_pub5_.publish(roi_marker);

    pcl::search::KdTree<PointT>::Ptr tree4(new pcl::search::KdTree<PointT>);
    tree4->setInputCloud(roi_cloud4);

    std::vector<pcl::PointIndices> cluster_indices4;
    pcl::EuclideanClusterExtraction<PointT> ec4;
    ec4.setClusterTolerance(0.4);
    ec4.setMinClusterSize(4);
    ec4.setMaxClusterSize(1000);
    ec4.setSearchMethod(tree4);
    ec4.setInputCloud(roi_cloud4);
    ec4.extract(cluster_indices4);

    std::vector<Eigen::Vector4f> cluster_centroids;
    std::vector<pcl::PointCloud<PointT>::Ptr> clusters4;
    std_msgs::Float64MultiArray cluster_coordinates4;

    int cluster_id4 = 1;
    for (auto &indices : cluster_indices4)
    {
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        Eigen::Vector4f centroid;
        for (int idx : indices.indices)
        {
            PointT point = (*roi_cloud4)[idx];
            point.intensity = static_cast<float>(cluster_id4);
            cluster->push_back(point);
        }
        pcl::compute3DCentroid(*cluster, centroid);
        cluster_centroids.push_back(centroid);
        clusters4.push_back(cluster);

        cluster_coordinates4.data.push_back(centroid[0]);
        cluster_coordinates4.data.push_back(centroid[1]);
        
        cluster_id4++;
    }

    std::sort(cluster_centroids.begin(), cluster_centroids.end(),
              [](const Eigen::Vector4f &a, const Eigen::Vector4f &b)
              { return a[0] < b[0]; });

    std::vector<Eigen::Vector4f> closest_centroids;
    if (cluster_centroids.size() >= 2)
    {
        closest_centroids.push_back(cluster_centroids[0]);
        closest_centroids.push_back(cluster_centroids[1]);
    }

    std_msgs::Float64MultiArray closest_points_msg;
    for (const auto &centroid : closest_centroids)
    {
        closest_points_msg.data.push_back(centroid[0]);
        closest_points_msg.data.push_back(centroid[1]);
    }

    two_roi_point_pub_.publish(closest_points_msg);

    std_msgs::Int16 idx_msg;
    if (!closest_centroids.empty())
    {
        idx_msg.data = (closest_centroids[0][0] < 5.8) ? 0 : 1;
    }
    else
    {
        idx_msg.data = -1;
    }
    idx_pub_.publish(idx_msg);

    ROS_INFO("Closest centroids published. Index: %d", idx_msg.data);

    visualization_msgs::MarkerArray roi4_cluster_markers;
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    roi4_cluster_markers.markers.push_back(delete_marker);

    int id4 = 0;
    for (size_t i = 0; i < clusters4.size(); ++i)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*clusters4[i], centroid);

        visualization_msgs::Marker cluster_marker;
        cluster_marker.header.frame_id = "velodyne";
        cluster_marker.header.stamp = ros::Time::now();
        cluster_marker.ns = "roi_4_clusters";
        cluster_marker.id = id4++;
        cluster_marker.type = visualization_msgs::Marker::SPHERE;
        cluster_marker.action = visualization_msgs::Marker::ADD;
        cluster_marker.scale.x = 0.3;
        cluster_marker.scale.y = 0.3;
        cluster_marker.scale.z = 0.3;
        cluster_marker.color.r = 1.0;
        cluster_marker.color.g = 0.0;
        cluster_marker.color.b = 0.0;
        cluster_marker.color.a = 1.0;
        cluster_marker.pose.position.x = centroid[0];
        cluster_marker.pose.position.y = centroid[1];
        cluster_marker.pose.position.z = centroid[2];

        roi4_cluster_markers.markers.push_back(cluster_marker);
    }

    marker_pub6_.publish(roi4_cluster_markers);

    ROS_INFO("Fourth ROI cluster coordinates size: %zu", cluster_coordinates4.data.size());
    all_roi_point_pub_.publish(cluster_coordinates4);
}


/* ╔══════════════════════════════════════════════════════════════════════════════╗
   ║                           유틸리티 / 헬퍼 함수                             ║
   ╚══════════════════════════════════════════════════════════════════════════════╝ */
void LiDARBIG::define_hull_indices(
    std::vector<pcl::Vertices> &hull_indices,
    const pcl::PointCloud<PointT>::Ptr &hull_points)
{
  (void)hull_points; // Suppress unused parameter warning
    pcl::Vertices v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12;
    v1.vertices = {0, 1, 2}; v2.vertices = {0, 2, 3};
    v3.vertices = {4, 5, 6}; v4.vertices = {4, 6, 7};
    v5.vertices = {0, 1, 5}; v6.vertices = {0, 5, 4};
    v7.vertices = {1, 2, 6}; v8.vertices = {1, 6, 5};
    v9.vertices = {2, 3, 7}; v10.vertices = {2, 7, 6};
    v11.vertices = {3, 0, 4}; v12.vertices = {3, 4, 7};

    hull_indices.push_back(v1); hull_indices.push_back(v2);
    hull_indices.push_back(v3); hull_indices.push_back(v4);
    hull_indices.push_back(v5); hull_indices.push_back(v6);
    hull_indices.push_back(v7); hull_indices.push_back(v8);
    hull_indices.push_back(v9); hull_indices.push_back(v10);
    hull_indices.push_back(v11); hull_indices.push_back(v12);
}

void LiDARBIG::visualize_roi(const pcl::PointCloud<PointT>::Ptr &hull_points, const std::string &ns)
{
    visualization_msgs::Marker roi_marker;
    roi_marker.header.frame_id = "velodyne";
    roi_marker.header.stamp = ros::Time::now();
    roi_marker.ns = ns;
    roi_marker.id = 0;
    roi_marker.type = visualization_msgs::Marker::LINE_STRIP;
    roi_marker.action = visualization_msgs::Marker::ADD;
    roi_marker.scale.x = 0.1;
    roi_marker.color.r = 0.0; roi_marker.color.g = 1.0; roi_marker.color.b = 0.0; roi_marker.color.a = 1.0;

    for (const auto &point : hull_points->points)
    {
        geometry_msgs::Point p;
        p.x = point.x; p.y = point.y; p.z = point.z;
        roi_marker.points.push_back(p);
    }
    geometry_msgs::Point p;
    p.x = hull_points->points[0].x; p.y = hull_points->points[0].y; p.z = hull_points->points[0].z;
    roi_marker.points.push_back(p);

    if (ns == "roi_1") marker_pub2_.publish(roi_marker);
    else if (ns == "roi_2") marker_pub3_.publish(roi_marker);
    else if (ns == "roi_3") marker_pub4_.publish(roi_marker);
    else if (ns == "roi_4") marker_pub5_.publish(roi_marker);
}

void LiDARBIG::publish_cluster_centroids(const std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> &clusters)
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int id = 0;
    for (auto &cluster : clusters)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);

        visualization_msgs::Marker m;
        m.header.frame_id = "velodyne";
        m.header.stamp = ros::Time::now();
        m.ns = "cluster_centroids";
        m.id = id++;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.scale.x = 0.3; m.scale.y = 0.3; m.scale.z = 0.3;
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0;
        m.pose.position.x = centroid[0];
        m.pose.position.y = centroid[1];
        m.pose.position.z = centroid[2];
        marker_array.markers.push_back(m);
    }
    marker_pub_angle_.publish(marker_array);
}

void LiDARBIG::publish_filtered_markers(
    const std::vector<std::shared_ptr<pcl::PointCloud<PointT>>> &clusters,
    float min_x, float max_x, float min_y, float max_y)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    for (auto &cluster : clusters)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);

        if (centroid[0] >= min_x && centroid[0] <= max_x &&
            centroid[1] >= min_y && centroid[1] <= max_y)
        {
            visualization_msgs::Marker m;
            m.header.frame_id = "velodyne";
            m.header.stamp = ros::Time::now();
            m.ns = "filtered_clusters";
            m.id = id++;
            m.type = visualization_msgs::Marker::SPHERE;
            m.action = visualization_msgs::Marker::ADD;
            m.scale.x = 0.3; m.scale.y = 0.3; m.scale.z = 0.3;
            m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0;
            m.pose.position.x = centroid[0];
            m.pose.position.y = centroid[1];
            m.pose.position.z = centroid[2];
            marker_array.markers.push_back(m);
        }
    }
    marker_pub_angle_.publish(marker_array);
}


/* ╔══════════════════════════════════════════════════════════════════════════════╗
   ║                                   main                                    ║
   ╚══════════════════════════════════════════════════════════════════════════════╝ */
int main(int argc, char **argv)
  {
    ros::init(argc, argv, "lidarbig");
    LiDARBIG node;
    ros::spin();
    return 0;
  }
