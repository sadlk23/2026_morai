#include <ros/ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
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


class LiDARSTANDARD{
public:
  sensor_msgs::PointCloud2 output;
  int check_pointcloud = 0;

  LiDARSTANDARD(const std::string& node_name = "velodyne_preprocess")
  : nh_(), pnh_("~")
  {
    LiDAR_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "/lidar/velodyne_points", 10,
        &LiDARSTANDARD::LiDARCallback, this,
        ros::TransportHints().tcpNoDelay());
  
      dis_pub_ =  nh_.advertise<std_msgs::Float32>("/LiDAR/car_dis", 10);
      dynamic_obstacle_pos_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/LiDAR/dynamic_obstacle_pos", 10);
      select_PC_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/LiDAR/bigcone_", 10); // 포인트클라우드용
      marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/Marker/big_static_BIG_POINT", 10);  
      }
  ~LiDARSTANDARD()
  {
    ROS_INFO("Velodyne Preprocess Node has been terminated");
  }

  void LiDARCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  
  void FrontCar(const sensor_msgs::PointCloud2::ConstPtr& msg);
  
private:
  ros::Publisher select_PC_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher dis_pub_;
  ros::Publisher dynamic_obstacle_pos_pub_;
  ros::Subscriber LiDAR_sub_;

  ros::Subscriber planning_mission_sub;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  Utility_Function util_func_;
  std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> clusters;
  float car_dis;
  int pre_class = -1;
  
};

void LiDARSTANDARD::LiDARCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{   
    FrontCar(msg);
}

void LiDARSTANDARD::FrontCar(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  constexpr float kMaxReportedDistance = 50.0f;

  if (check_pointcloud == 0)
  {
    check_pointcloud = 1;
  }
  
  float min_dist = kMaxReportedDistance; // Default safe distance
  float min_obstacle_x = 0.0; // 가장 가까운 장애물 x (velodyne 좌표계)
  float min_obstacle_y = 0.0; // 가장 가까운 장애물 y (velodyne 좌표계)
  std::vector<double> obstacle_positions; // [x1, y1, x2, y2, ...]

  // 측면에서 차로로 진입하는 보행자를 미리 관측하기 위한 ROI.
  // 좌표 토픽에는 넓은 ROI의 모든 장애물을 유지하되, 경로 정보가 없는
  // legacy /LiDAR/car_dis는 차량 진행축 주변의 좁은 구간만 사용한다.
  constexpr float kRoiMinX = -3.0f;
  constexpr float kRoiMaxX = kMaxReportedDistance;
  constexpr float kRoiMinY = -12.0f;
  constexpr float kRoiMaxY = 12.0f;
  constexpr float kRoiMinZ = -0.55f;
  constexpr float kRoiMaxZ = 2.0f;
  constexpr float kObservationHalfWidth = 11.5f;
  constexpr float kCarDistanceHalfWidth = 1.8f;

  // 거리와 좌표를 항상 같이 발행해 직전 프레임의 장애물이
  // Planning/plotting에 남지 않게 한다. 미감지 시 좌표 array는 empty이다.
  const auto publish_obstacle_result =
      [this](float distance, const std::vector<double>& positions)
      {
        std_msgs::Float32 distance_msg;
        distance_msg.data = distance;
        dis_pub_.publish(distance_msg);

        std_msgs::Float64MultiArray position_msg;
        position_msg.data = positions;
        dynamic_obstacle_pos_pub_.publish(position_msg);
      };

  // 1. 변환
  auto cloud = util_func_.PointCloudROS(msg);
  if (cloud->empty()) {
     publish_obstacle_result(min_dist, {});
     return;
  }

  // 2. 다운샘플링
  auto cloud_filtered = util_func_.DownSamplingVoxel(cloud, 0.1f, 0.1f, 0.1f);
  if (cloud_filtered->empty()) {
     publish_obstacle_result(min_dist, {});
     return;
  }

  // 3. Crop 필터: 고속도로 전방 50m와 측면 진입 물체를 함께 관측
  auto cloud_filtered2 = util_func_.CropFilter(
  cloud_filtered,
  Eigen::Vector4f(kRoiMinX, kRoiMinY, kRoiMinZ, 0.0),
  Eigen::Vector4f(kRoiMaxX, kRoiMaxY, kRoiMaxZ, 0.0));

  if (cloud_filtered2->empty()) {
     // Publish empty cloud to clear RViz
     sensor_msgs::PointCloud2 output;
     pcl_conversions::fromPCL(pcl::PCLPointCloud2(), output);
     output.header.frame_id = "velodyne";
     output.header.stamp = msg->header.stamp; // Sync timestamp
     select_PC_pub_.publish(output);

     publish_obstacle_result(min_dist, {});
     return;
  }

  // Debug: Visualize ROI Box
  visualization_msgs::Marker marker_roi;
  marker_roi.header.frame_id = "velodyne";
  marker_roi.ns = "roi_box";
  marker_roi.id = 0;
  marker_roi.header.stamp = msg->header.stamp;
  marker_roi.action = visualization_msgs::Marker::ADD;
  marker_roi.type = visualization_msgs::Marker::CUBE;
  marker_roi.scale.x = kRoiMaxX - kRoiMinX;
  marker_roi.scale.y = kRoiMaxY - kRoiMinY;
  marker_roi.scale.z = kRoiMaxZ - kRoiMinZ;
  marker_roi.pose.position.x = (kRoiMaxX + kRoiMinX) / 2.0f;
  marker_roi.pose.position.y = (kRoiMaxY + kRoiMinY) / 2.0f;
  marker_roi.pose.position.z = (kRoiMaxZ + kRoiMinZ) / 2.0f;
  marker_roi.color.r = 1.0; marker_roi.color.g = 1.0; marker_roi.color.b = 0.0; marker_roi.color.a = 0.1;
  marker_pub_.publish(marker_roi);

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
  output.header.stamp = msg->header.stamp; // Sync timestamp
  select_PC_pub_.publish(output);

  // ////////////////////// < ROI를 나타내는 직육면체 마커 > //////////////////////////////////
  // visualization_msgs::Marker marker;
  // marker.header.frame_id = "velodyne";
  // marker.ns = "my_roi";
  // marker.id = 0; // ID는 0으로 설정 (하나의 마커만 사용)
  // marker.header.stamp = ros::Time::now();
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.type = visualization_msgs::Marker::CUBE;
  // marker.scale.x = (30.0 - 0.0);
  // marker.scale.y = (10.0 - (-10.0));
  // marker.scale.z = (2.0 - (-0.55));
  // marker.pose.position.x = (30.0 + 0.0) / 2;
  // marker.pose.position.y = (10.0 + (-10.0)) / 2;
  // marker.pose.position.z = (2.0 + (-0.55)) / 2;
  // marker.color.r = 0.0;
  // marker.color.g = 1.0;
  // marker.color.b = 0.0;
  // marker.color.a = 0.1;
  // marker_pub_->publish(marker);
  // ///////////////////////////////////////////////////////////////////////////////////


  if (clusters.size() > 0)
  {
    std::cout << "car cluster! : " << clusters.size() << std::endl;  
      
    for (size_t i = 0; i < clusters.size(); i++)
    {
      // cout << "find_big" << endl;
      geometry_msgs::Point center_point, min_point, max_point;
      util_func_.GetSenCloudPoint(clusters, center_point, i);
      util_func_.GetMinMaxCloudPoint(clusters, min_point, max_point, i);

      float cen_x = center_point.x;

      // Debug prints
      // std::cout << "max_point.y - min_point.y : " << max_point.y - min_point.y << std::endl;

      // Filter: Broadened range to detect obstacles like cones (0.2m) and others
      float width_y = max_point.y - min_point.y;
      if (width_y > 0.1 && width_y < 12.0) 
      {
         // 한 개의 최근 물체만 발행하면 다른 클러스터가 측면 돌진 객체를
         // 가릴 수 있다. 판단 ROI의 모든 중심점을 평탄 배열로 발행한다.
         if (std::isfinite(center_point.x) && std::isfinite(center_point.y) &&
             std::abs(center_point.y) < kObservationHalfWidth)
         {
            obstacle_positions.push_back(center_point.x);
            obstacle_positions.push_back(center_point.y);

            // 단순히 전방에 있다는 이유만으로 감속하지 않도록
            // /LiDAR/car_dis는 차량 진행축에서 좌우 1.8 m 이내만 선택한다.
            // 넓은 측면 장애물 좌표는 위의 obstacle_positions에 그대로 남아
            // Planning의 경로 투영 및 측면 충돌 예측에서 별도로 처리된다.
            const float object_distance = std::max(
                0.0f,
                std::min(static_cast<float>(min_point.x),
                         kMaxReportedDistance));
            if (cen_x > 0.0f && object_distance < min_dist &&
                std::abs(center_point.y) <= kCarDistanceHalfWidth)
            {
              min_dist = object_distance;
              min_obstacle_x = center_point.x;
              min_obstacle_y = center_point.y;
            }
         }

        ////////////////// < 장애물 바운딩 박스 나타내는 마커 > ///////////////////////////
       visualization_msgs::Marker marker3;
      visualization_msgs::MarkerArray markerArray3;

      marker3.header.frame_id = "velodyne";
      marker3.header.stamp = ros::Time::now();
      marker3.ns = "my_marker";
      marker3.id = i;
      marker3.type = visualization_msgs::Marker::CUBE;
      marker3.action = visualization_msgs::Marker::ADD;

      // 마커의 생존 시간 (0이면 계속 유지)
      marker3.lifetime = ros::Duration(0.2); 

      // 크기 (x,y,z는 반드시 0보다 커야 RViz에서 보임)
      float scale_x = max_point.x - min_point.x;
      float scale_y = max_point.y - min_point.y;
      float scale_z = max_point.z - min_point.z;
      
      // scale 최소값 보장 (0이면 안 보임)
      marker3.scale.x = (scale_x > 0.01) ? scale_x : 0.1;
      marker3.scale.y = (scale_y > 0.01) ? scale_y : 0.1;
      marker3.scale.z = (scale_z > 0.01) ? scale_z : 0.1;

      // 색상
      marker3.color.r = 0.5;
      marker3.color.g = 0.5;
      marker3.color.b = 0.5;
      marker3.color.a = 0.5;

      // 위치/자세
      marker3.pose.position = center_point;
      

      markerArray3.markers.push_back(marker3);
      marker_pub_.publish(markerArray3);
      // std::cout << "Marker published! scale: (" << marker3.scale.x << ", " << marker3.scale.y << ", " << marker3.scale.z << ")" << std::endl;
        ////////////////////////////////////////////////////////////////////////////
      }
    }
  }

  const bool obstacle_detected =
      min_dist < kMaxReportedDistance && min_obstacle_x > 0.0f &&
      std::isfinite(min_obstacle_x) && std::isfinite(min_obstacle_y);
  publish_obstacle_result(min_dist, obstacle_positions);

  std::cout << "car_dis pub: " << min_dist << std::endl; 
  if (obstacle_detected)
  {
    std::cout << "dynamic_obstacle_pos pub: "
              << obstacle_positions.size() / 2
              << " obstacles, nearest=(" << min_obstacle_x << ", "
              << min_obstacle_y << ")" << std::endl;
  }
}

  // visualization_msgs::MarkerArray p_array;
  // geometry_msgs::Point obj1;
  // visualization_msgs::Marker marker1;
  // marker1.ns = "deli_signs";
  // marker1.action = visualization_msgs::Marker::ADD;
  // marker1.type = visualization_msgs::Marker::POINTS;
  // marker1.id = 0;
  // marker1.pose.orientation.w = 1.0;
  // marker1.scale.x = 0.5;
  // marker1.scale.y = 0.5;
  // marker1.color.a = 1.0;
  // marker1.color.r = 1.0f;
  // obj1.x = p.data[0];
  // obj1.y = p.data[1];
  // obj1.z = 0.0;
  // marker1.points.push_back(obj1);
  // marker1.header.frame_id = "velodyne";
  // p_array.markers.push_back(marker1);

  // geometry_msgs::Point obj2;
  // visualization_msgs::Marker marker2;
  // marker2.ns = "deli_signs";
  // marker2.action = visualization_msgs::Marker::ADD;
  // marker2.type = visualization_msgs::Marker::POINTS;
  // marker2.id = 1;
  // marker2.pose.orientation.w = 1.0;
  // marker2.scale.x = 0.5;
  // marker2.scale.y = 0.5;
  // marker2.color.a = 1.0;
  // marker2.color.b = 1.0f;
  // obj2.x = p.data[2];
  // obj2.y = p.data[3];
  // obj2.z = 0.0;
  // marker2.points.push_back(obj2);
  // marker2.header.frame_id = "velodyne";
  // p_array.markers.push_back(marker2);

  // geometry_msgs::Point obj3;
  // visualization_msgs::Marker marker3;
  // marker3.ns = "deli_signs";
  // marker3.action = visualization_msgs::Marker::ADD;
  // marker3.type = visualization_msgs::Marker::POINTS;
  // marker3.id = 2;
  // marker3.pose.orientation.w = 1.0;
  // marker3.scale.x = 0.5;
  // marker3.scale.y = 0.5;
  // marker3.color.a = 1.0;
  // marker3.color.g = 1.0f;
  // obj3.x = p.data[4];
  // obj3.y = p.data[5];
  // obj3.z = 0.0;
  // marker3.points.push_back(obj3);
  // marker3.header.frame_id = "velodyne";
  // p_array.markers.push_back(marker3);

  // deli_marker.publish(p_array);

  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "lidar_standard");
    LiDARSTANDARD node;
    ros::spin();
    return 0;
  }
