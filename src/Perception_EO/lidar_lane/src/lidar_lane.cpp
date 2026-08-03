// 기본 (바꿈)
#include <ros/ros.h>

// 자료형(바꿈)
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/sac_model_line.h>

// 시각화(바꿈)
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// 계산
#include <cmath>
#include <vector>
#include <algorithm>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include "qos_utils.hpp"
#include <limits>

using PointT = pcl::PointXYZI;
using namespace std;

class LiDAR_lane
{
public:
    explicit LiDAR_lane(ros::NodeHandle &nh) : nh_(nh)
    {
        mission_flag_sub_ = nh_.subscribe<std_msgs::Int16>(
            "/Planning/mission", 10,
            &LiDAR_lane::mission_flag_cb, this,
            lidar::best_effort_qos().hints); // (ros1용)

        timer_ = nh_.createTimer(ros::Duration(1.0),
                                 &LiDAR_lane::timer_callback, this); // (ros1용)

        lane_ = nh_.advertise<visualization_msgs::MarkerArray>("/Marker/center_lane_yj", 10);         // (ros1용)
        lane_dot_ = nh_.advertise<visualization_msgs::MarkerArray>("/Marker/lane_dot_yj", 10);        // (ros1용)
        coord_dot_ = nh_.advertise<visualization_msgs::MarkerArray>("/Marker/ERP_center_dot_yj", 10); // (ros1용)
        crop_box_ = nh_.advertise<visualization_msgs::Marker>("/Marker/range_tunnel_end", 10);        // (ros1용)
        lane_dot_MA = nh_.advertise<std_msgs::Float64MultiArray>("/LiDAR/lane", 10);                  // (ros1용)
        center_ = nh_.advertise<std_msgs::Float64MultiArray>("/LiDAR/center_lane", 10);               // (ros1용)
        wall_distance = nh_.advertise<std_msgs::Float64MultiArray>("/LiDAR/wall_dist", 10);           // (ros1용)
        ceiling_end_ = nh_.advertise<std_msgs::Float64MultiArray>("/LiDAR/ceiling_end", 10);          // (ros1용)
        pub_cluster = nh_.advertise<sensor_msgs::PointCloud2>("/Cluster/tunnel_line", 10);            // (ros1용)
        pub_ceiling = nh_.advertise<sensor_msgs::PointCloud2>("/Cluster/tunnel_ceiling", 10);         // (ros1용)
        wall_cluster = nh_.advertise<sensor_msgs::PointCloud2>("/Cluster/tunnel_wall", 10);           // (ros1용)
        zone_center = nh_.advertise<visualization_msgs::Marker>("/Marker/orange_lane_zone_yj", 10);   // (ros1용)
        tunnel_end_ = nh_.advertise<std_msgs::Bool>("/LiDAR/tunnel_end", 10);                         // (ros1용)
        tunnel_center_roi_ = nh_.advertise<visualization_msgs::Marker>("/Marker/center_roi_yj", 10);  // (ros1용)

        sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/lidar/velodyne_points", 10,
            &LiDAR_lane::cloud_cb, this,
            lidar::best_effort_qos().hints); // (ros1용)
    }

private:
    ros::NodeHandle nh_; // (ros1용)
    int mission_flag = 0;
    float lane_intensity_threshold = 50.0f; // 여기바꿔 -------------------------------실험해봐야함~~~ㅜㅜ (08/30)
    bool ceiling_nan = false;

    float ceiling_end_check = 0.0f;

    int mission_check = -1;
    int mission_count = 0;

    // 터널 끝 검출 모드: 6만 사용(3D LiDAR ROI point 수 기반)
    static constexpr int kTunnelEndMode = 6;

    int chk = 0; // 0: 밖, 1: 안, 2: 끝

    // ===== 멤버 타입 ROS1식으로 교체 =====
    ros::Publisher pub_cluster;        // (ros1용)
    ros::Publisher pub_ceiling;        // (ros1용)
    ros::Publisher lane_;              // (ros1용)
    ros::Publisher lane_dot_;          // (ros1용)
    ros::Publisher coord_dot_;         // (ros1용)
    ros::Publisher lane_dot_MA;        // (ros1용)
    ros::Publisher center_;            // (ros1용)
    ros::Publisher wall_distance;      // (ros1용)
    ros::Publisher wall_cluster;       // (ros1용)
    ros::Subscriber sub_;              // (ros1용)
    ros::Publisher zone_center;        // (ros1용)
    ros::Publisher crop_box_;          // (ros1용)
    ros::Subscriber mission_flag_sub_; // (ros1용)
    ros::Publisher tunnel_end_;        // (ros1용)
    ros::Publisher tunnel_center_roi_; // (ros1용)
    ros::Publisher ceiling_end_;       // (ros1용)
    ros::Timer timer_;                 // (ros1용)

    void timer_callback(const ros::TimerEvent &)
    {
        if (mission_check == -1)
        {
            mission_count = 0;
            ROS_INFO(u8"is it tunnel??"); // (ros1용)
        }
        else if (mission_check > 0 && mission_count < 1)
        {
            ROS_INFO(u8"ARE WE IN TUNNEL??"); // (ros1용)ㄱ
            mission_count = 1;
        }
        // mission_check = -1;
    }

    void mission_flag_cb(const std_msgs::Int16::ConstPtr &msg) // (ros1용)
    {
        this->mission_flag = msg->data;
        if (mission_flag == 23) // || mission_flag == 17) //미션번호 수정 (08.19) 예진 //원래는 23 이고 오늘은 02.10
        {
            mission_check = 1;
        }
    }

    void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &input) // (ros1용)
    {
        if (mission_flag != 23) // 미션번호 수정 (08.19) 예진
            return;

        // ROS msg -> PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*input, *cloud);

        // -------------------- 벽 --------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>());
        down_sampling(cloud, cloud_downsampled);

        pcl::PointCloud<pcl::PointXYZI>::Ptr wall_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        seethewall(cloud_downsampled, wall_filtered);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZI>());
        project_to_2d_plane(wall_filtered, cloud_projected);

        pcl::PointCloud<pcl::PointXYZI>::Ptr largest_clusters_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        find_two_largest_clusters(cloud_projected, largest_clusters_cloud);

        vector_wall(largest_clusters_cloud);

        // -------------------- 차선 --------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>());
        cropGround(cloud, cloud_filtered2);

        pcl::PointCloud<pcl::PointXYZI>::Ptr lanes_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        extractLanesByIntensity(cloud_filtered2, lanes_cloud, lane_intensity_threshold);

        clusterLanes(lanes_cloud, 0.2, 3, 50); // ------여기 바꿔 -------------

        // -------------------- 천장/터널끝 --------------------
        pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        I_want_know_tunnel_end(cloud, ceiling_filtered);

        if (ceiling_nan)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_projected(new pcl::PointCloud<pcl::PointXYZI>());
            clustering_tunnel_ceiling(ceiling_filtered, ceiling_projected);
        }

        if (kTunnelEndMode == 6)
        { // std::cout << "tunnel_end_check : " << "ceiling_end_check" << std::endl;
            pcl::PointCloud<pcl::PointXYZI>::Ptr tunnel_end_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            crop_tunnel_end_cloud(cloud, tunnel_end_cloud);
        }

        // -------------------- 벽 클러스터 publish --------------------
        sensor_msgs::PointCloud2 output_wall; // (ros1용)
        pcl::toROSMsg(*largest_clusters_cloud, output_wall);
        output_wall.header = input->header; // stamp + frame_id 동기
        wall_cluster.publish(output_wall);  // (ros1용)

        // -------------------- ROI 시각화(중앙 차선) --------------------

        visualization_msgs::Marker marker; // (ros1용)
        marker.header.frame_id = "velodyne";
        marker.ns = "my_roi";
        marker.id = 0;
        marker.header.stamp = ros::Time::now();          // (ros1용)
        marker.action = visualization_msgs::Marker::ADD; // (ros1용)
        marker.type = visualization_msgs::Marker::CUBE;  // (ros1용)

        // 왼쪽 차선 보고 주행할 때 디버깅용 시각화 -----------------
        marker.scale.x = 8;
        marker.scale.y = 4;
        marker.scale.z = 1;
        marker.pose.position.x = 4;
        marker.pose.position.y = 2;
        marker.pose.position.z = -1.075; //-0.5;(기존..)

        // // 오른쪽 차선 보고 주행할 때 디버깅용 시각화 -----------------
        // marker.scale.x = 8;
        // marker.scale.y = 3.5;  // (-0.5 ~ -4.0)
        // marker.scale.z = 1.25; // (-1.7 ~ -0.45)
        // marker.pose.position.x = 4;
        // marker.pose.position.y = -2.25;  // ( -4.0 + -0.5 ) / 2
        // marker.pose.position.z = -1.075; // ( -1.7 + -0.45 ) / 2
        //----------------------------------------------------------
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;
        // marker.lifetime = ros::Duration(0.0); // 깜빡임 방지 (ros1용 - 필요시 주석 해제)
        zone_center.publish(marker); // (ros1용)
    }

    // -------------------- 유틸 함수들 --------------------
    void down_sampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled)
    {
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_grid.filter(*cloud_downsampled);
    }

    void crop_tunnel_end_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr tunnel_end_cloud)
    { // 천장/터널 끝 검출용 ROI
        // std::cout << "tunnel_end_cloud" << std::endl;
        // Remove points close to the ground plane
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(cloud);
        // 여기 바꿔 -----------
        cropFilter.setMin(Eigen::Vector4f(-2, -20, 2, 1));
        cropFilter.setMax(Eigen::Vector4f(0, 20, 3, 1));
        //-----------------------------
        cropFilter.filter(*tunnel_end_cloud); // 여기에 저장

        std_msgs::Bool msg; // (ros1용)
        msg.data = false;

        /*
     0 : 현재 터널 밖
     1 : 현재 터널 안
     2 : 현재 터널 끝
    */
        if (tunnel_end_cloud->points.size() >= 100)
        {
            chk = 1; // 터널 안
        }
        else if (tunnel_end_cloud->points.size() < 45 && chk == 1)
        {
            msg.data = true; // 터널 끝
            chk = 2;
        }
        tunnel_end_.publish(msg); // (ros1용)

        visualization_msgs::Marker marker; // (ros1용)
        marker.header.frame_id = "velodyne";
        marker.ns = "my_roi";
        marker.id = 0;
        marker.header.stamp = ros::Time::now(); // (ros1용)

        marker.action = visualization_msgs::Marker::ADD; // (ros1용)
        marker.type = visualization_msgs::Marker::CUBE;  // (ros1용)
        marker.scale.x = cropFilter.getMax()[0] - cropFilter.getMin()[0];
        marker.scale.y = cropFilter.getMax()[1] - cropFilter.getMin()[1];
        marker.scale.z = cropFilter.getMax()[2] - cropFilter.getMin()[2];
        marker.pose.position.x = (cropFilter.getMax()[0] + cropFilter.getMin()[0]) / 2;
        marker.pose.position.y = (cropFilter.getMax()[1] + cropFilter.getMin()[1]) / 2;
        marker.pose.position.z = (cropFilter.getMax()[2] + cropFilter.getMin()[2]) / 2;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;
        // marker.lifetime = ros::Duration(0.0); // (ros1용)
        crop_box_.publish(marker); // (ros1용)
    }

    void cropGround(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2)
    { // 터널 시작/중/후 별로 양옆의 범위를 달리하기 위해서, 첫번째로 가장 먼저 자르는 곳
        // Remove points close to the ground plane
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(cloud);
        if (chk != 2) // 터널 끝이 아닐 때
        {             // 여기 바꿔 ----------------가장 먼저 잘라버리는 곳----------------------------
            cropFilter.setMin(Eigen::Vector4f(0, -10, -1.7, 1));
            cropFilter.setMax(Eigen::Vector4f(8, 10, -0.45, 1));
        }
        else // 터널 끝구간에서만 적게 본다 ! (막 벗어나는 구간,,, 잡음 확 줄이려고, 가드레일이든 연석이든 뭐든)
        {
            cropFilter.setMin(Eigen::Vector4f(0, -3, -1.7, 1));
            cropFilter.setMax(Eigen::Vector4f(8, 3, -0.45, 1));
        }
        //---------------------------------------------------------

        visualization_msgs::Marker marker; // (ros1용)
        marker.header.frame_id = "velodyne";
        marker.ns = "my_roi";
        marker.id = 0;
        marker.header.stamp = ros::Time::now(); // (ros1용)

        marker.action = visualization_msgs::Marker::ADD; // (ros1용)
        marker.type = visualization_msgs::Marker::CUBE;  // (ros1용)
        marker.scale.x = cropFilter.getMax()[0] - cropFilter.getMin()[0];
        marker.scale.y = cropFilter.getMax()[1] - cropFilter.getMin()[1];
        marker.scale.z = cropFilter.getMax()[2] - cropFilter.getMin()[2];
        marker.pose.position.x = (cropFilter.getMax()[0] + cropFilter.getMin()[0]) / 2;
        marker.pose.position.y = (cropFilter.getMax()[1] + cropFilter.getMin()[1]) / 2;
        marker.pose.position.z = (cropFilter.getMax()[2] + cropFilter.getMin()[2]) / 2;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.2;
        // marker.lifetime = ros::Duration(0.0); // (ros1용)
        tunnel_center_roi_.publish(marker); // (ros1용)

        cropFilter.filter(*cloud_filtered2);
    }

    void seethewall(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr wall_filtered)
    {
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(cloud);
        // seethewall() 에서 cropbox로 특정 높이와 전방 영역만 남긴다
        // z=0.8~3.0m 범위의 포인트만 남기므로 바닥/차선은 제외해두고, 벽 천장 포인트만 남김
        cropFilter.setMin(Eigen::Vector4f(-20, -20, 0.8, 0));
        // cropFilter.setMax(Eigen::Vector4f(100, 20, 2.0, 0));  // sanhak
        cropFilter.setMax(Eigen::Vector4f(100, 20, 3.0, 0));
        cropFilter.filter(*wall_filtered);
    }

    void I_want_know_tunnel_end(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_projected)
    { // 천장 보임/안보임 판단하ㅓ기 위한 z=0 으로 투영하는 부분
        pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(cloud);
        cropFilter.setMin(Eigen::Vector4f(15, -30, 3.0, 1));
        cropFilter.setMax(Eigen::Vector4f(30, 30, 10.0, 1));
        cropFilter.filter(*ceiling_filtered);

        // z=0으로 투영
        Eigen::Matrix4f projection_matrix = Eigen::Matrix4f::Identity();
        projection_matrix(2, 2) = 0;

        std::vector<int> indices_remove;
        pcl::removeNaNFromPointCloud(*ceiling_filtered, *ceiling_filtered, indices_remove);

        if (!ceiling_filtered->empty())
        {
            ceiling_nan = true;
            pcl::transformPointCloud(*ceiling_filtered, *ceiling_projected, projection_matrix);
        }
        else
        {
            ceiling_nan = false;
            std::cout << "No valid points after removing NaNs." << std::endl;
        }
    }

    void clustering_tunnel_ceiling(pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_projected, pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_projected)
    { // 천장 클러스터링 및 퍼블리시, 위 함수에서 얻은 ceiling_projected를 입력으로 받음 -> 클러스터링
        // 클러스터링을 위한 KDTree 오브젝트 생성
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
        kdtree->setInputCloud(ceiling_projected);

        // 클러스터링 결과를 저장할 벡터
        std::vector<pcl::PointIndices> ceiling_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(20.0);
        ec.setMinClusterSize(10);
        // ec.setMaxClusterSize(25000);
        ec.setSearchMethod(kdtree);
        ec.setInputCloud(ceiling_projected);
        ec.extract(ceiling_indices);

        int ii = 0;
        pcl::PointCloud<PointT> TotalCloud;
        std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> clusters;
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        // std::cout << "before" << std::endl;
        for (vector<pcl::PointIndices>::const_iterator it = ceiling_indices.begin(); it != ceiling_indices.end(); ++it, ++ii)
        {
            // pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>); // <-- 여기!-> 각 클러스터가 지금까지 섞이는 핵심 버그가 있다함 일단 .. 이거 적용해볼 것 (08/30)

            for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                cluster->points.push_back(ceiling_projected->points[*pit]);
                PointT pt = ceiling_projected->points[*pit];
                PointT pt2;
                pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
                pt2.intensity = (float)(ii + 1);
                TotalCloud.push_back(pt2);
            }
            cluster->width = cluster->size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }
        // std::cout << "mid" << std::endl>
        pcl::PCLPointCloud2 cloud_p;
        pcl::toPCLPointCloud2(TotalCloud, cloud_p);
        sensor_msgs::PointCloud2 clu; // (ros1용)
        pcl_conversions::fromPCL(cloud_p, clu);
        clu.header.frame_id = "velodyne";
        pub_ceiling.publish(clu); // (ros1용)

        // std::cout << "after" << std::endl;

        std_msgs::Float64MultiArray ceiling_end; // (ros1용)
        for (int i = 0; i < int(clusters.size()); i++)
        {
            Eigen::Vector4f centroid, min_p, max_p;
            pcl::compute3DCentroid(*clusters[i], centroid);
            pcl::getMinMax3D(*clusters[i], min_p, max_p);

            geometry_msgs::Point center_point; // (ros1용)
            center_point.x = centroid[0];
            center_point.y = centroid[1];
            center_point.z = centroid[2];

            geometry_msgs::Point min_point; // (ros1용)
            min_point.x = min_p[0];
            min_point.y = min_p[1];
            min_point.z = min_p[2];

            geometry_msgs::Point max_point; // (ros1용)
            max_point.x = max_p[0];
            max_point.y = max_p[1];
            max_point.z = max_p[2];
            float max_px = max_point.x;
            ceiling_end.data.push_back(max_px);

            ceiling_end_check = max_px;
        }
        // ceiling_end.data.push_back(50.0);
        // ceiling_end_.publish(ceiling_end); // (ros1용) 아래 줄이 실제 pub
        ceiling_end_.publish(ceiling_end); // 길이 판단으로 펍. 터널 천장 끝까지 검출된 길이
        // 일정 임계 이하로 떨어지면 터널 끝으로 상태 전환
        ceiling_end.data.clear();
    }

    void project_to_2d_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected)
    { // 벽 포인트들을 z=0 으로 투영
        // Z 축 값이 0이 되도록 변환 행렬을 생성합니다.
        Eigen::Matrix4f projection_matrix = Eigen::Matrix4f::Identity();

        projection_matrix(2, 2) = 0;
        std::vector<int> indices_remove; // NaN 값을 제거한 후 유지할 포인트 인덱스를 저장
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices_remove);
        // 변환 행렬을 적용합니다.
        if (!cloud->empty())
        {
            ceiling_nan = true;
            pcl::transformPointCloud(*cloud, *cloud_projected, projection_matrix);
        }
        else
        {
            ceiling_nan = false;
            std::cout << "No valid points after removing NaNs." << std::endl;
        }
    }

    void find_two_largest_clusters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr largest_clusters_cloud)
    { // 벽 보는 함수
        // 클러스터링을 위한 KDTree 오브젝트 생성
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
        kdtree->setInputCloud(cloud);

        // 클러스터링 결과를 저장할 벡터
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.5);
        ec.setMinClusterSize(10);
        // ec.setMaxClusterSize(25000);
        ec.setSearchMethod(kdtree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // 클러스터의 길이를 저장할 벡터
        std::vector<std::pair<size_t, size_t>> cluster_lengths;

        // 클러스터의 길이 계산
        for (size_t i = 0; i < cluster_indices.size(); ++i)
        {
            cluster_lengths.emplace_back(cluster_indices[i].indices.size(), i); // emplace_back 함수는 stl에서 제공하는 벡터 등의 컨테이너에서 사용할 수 있는 멤버 함수 like push_back
        } // 첫번째 size_t는 길이 , 두번째 size_t는 index

        // 여기 수정했음(08.26)
        // if (cluster_lengths.size() >= 2) 적용해볼거 (08/30)
        if (cluster_lengths.size() > 2)
        {
            // 클러스터 길이를 기준으로 정렬(내림차순)
            std::sort(cluster_lengths.begin(), cluster_lengths.end(), std::greater<>()); // greater<>는 첫번째 클러스터의 길이를 비교하여 첫번째가 마지막보다 클 때 참값 반환->내림차순

            size_t num_clusters_to_publish = std::min<size_t>(2, cluster_lengths.size());
            // 가장 긴 두 개의 클러스터에 속한 포인트를 하나의 포인트 클라우드로 병합
            for (size_t i = 0; i < num_clusters_to_publish; ++i)
            {
                size_t cluster_idx = cluster_lengths[i].second;
                for (const auto &point_idx : cluster_indices[cluster_idx].indices)
                {                                                           // 클러스터에 포함된 점들의 인덱스에 대해 반복
                    largest_clusters_cloud->push_back((*cloud)[point_idx]); // law data에서 해당 인덱스에 해당하는 point들 가져옴.
                }
            }
        }
        else
        {
            std::cout << "wall line 수 부족!!!!!!!!!!!!!!!!" << std::endl;
        }
    }

    void vector_wall(pcl::PointCloud<pcl::PointXYZI>::Ptr largest_clusters_cloud)
    { // 벽 클러스터링 후, 각 클러스터의 x축 기준 가장 작은 점과 큰 점을 찾아서 벽 좌표로 퍼블리시
        // Create KD-tree object for efficient clustering
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
        kdtree->setInputCloud(largest_clusters_cloud);

        // Euclidean clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;

        ec.setClusterTolerance(0.5); // 50cm 이내 포인트는 같은 클러스터
        ec.setMinClusterSize(10);    // 최소 10 포인트 이상이어야 클러스터로 인정

        // ec.setMaxClusterSize(50);
        ec.setSearchMethod(kdtree);
        ec.setInputCloud(largest_clusters_cloud);
        ec.extract(cluster_indices);

        std_msgs::Float64MultiArray wall_d; // (ros1용)

        // For each of the largest clusters...

        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
            for (const auto &index : indices.indices)
            {
                cluster->points.push_back(largest_clusters_cloud->points[index]);
            }

            // Find the min and max points along x-axis (or whichever axis represents your 'horizontal')
            // x축을 기준으로 가장 가까운 점과 큰 점을 찾고 그에 대응하는 y값 저장
            // 왼쪽 벽과 오른쪽 벽의 양 끝 좌표를 /LiDAR/wall_dist 토픽으로 퍼블리시

            // 뒤쪽 끝 좌표를 찾기 위한 세팅
            float x_min = std::numeric_limits<float>::max();
            float y_at_x_min = 0;
            // 앞쪽 끝 좌표를 찾기 위한 세팅
            float x_max = std::numeric_limits<float>::lowest(); // (ros1용 권장: min()->lowest())
            float y_at_x_max = 0;

            for (const auto &point : cluster->points) // 클러스터 내의 포인터들에 대해 반복
            {
                if (point.x < x_min)
                {
                    x_min = point.x;
                    y_at_x_min = point.y;
                }

                if (point.x > x_max)
                {
                    x_max = point.x;
                    y_at_x_max = point.y;
                }
            }
            wall_d.data.push_back(x_min);
            wall_d.data.push_back(y_at_x_min);
            wall_d.data.push_back(x_max);
            wall_d.data.push_back(y_at_x_max);
        }
        wall_distance.publish(wall_d); // (ros1용)
        wall_d.data.clear();
    }

    void extractLanesByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr lanes_cloud,
                                 float threshold)
    {
        for (const auto &point : cloud_filtered2->points)
        {
            if (point.intensity >= threshold)
                lanes_cloud->points.push_back(point);
        }
    }

    // -!!!!!!!!!!대망의 클러스터레인즈다 ~~~~~~~~~~~~~~~~~~~~------------------------------
    //  clusterLanes() → intensity ≥ threshold 포인트만 뽑음 → 클러스터링 → 각 클러스터 중심점을 계산.
    //  이후 CropBox(-0,0 ~ 8,4)로 앞쪽(x>0, y>0) 일부 영역만 다시 잘라서 centercenters라는 후보 집합을 만듦.
    //  란삭 알고리즘에서 내가 손댈 수 있는 거? -> 직선 자체는 pcl ransac이 판단하고 , 우리는 얼마나 빡세게 할지만 조정하는 거임 ㄱㄱ
    //  distancThreshold  : 직선에서 몇 미터 이내면 직선 위 점(인라이어)라고 볼지
    //  인라이어 최소 개수 : 너무 점이 적은 직선은 버려
    //  세그먼트 길이 : 인라이어들이 연속적으로 안 이어지면 버려

    void clusterLanes(pcl::PointCloud<pcl::PointXYZI>::Ptr lanes_cloud, double tolerance, int min_size, int max_size)
    {
        // Create KD-tree for efficient clustering
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(lanes_cloud);

        // Perform Euclidean clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(tolerance);
        ec.setMinClusterSize(min_size);
        ec.setMaxClusterSize(max_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(lanes_cloud);

        // Extract the clusters out of the input data
        ec.extract(cluster_indices); // 여기에 저장

        int ii = 0;
        pcl::PointCloud<PointT> TotalCloud;

        std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> clusters; // allocator 메모리 정렬용
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++ii)
        {
            for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                cluster->points.push_back(lanes_cloud->points[*pit]);
                PointT pt = lanes_cloud->points[*pit];
                PointT pt2;
                pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
                pt2.intensity = (float)(ii + 1);
                TotalCloud.push_back(pt2);
            }
            cluster->width = cluster->size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }

        pcl::PCLPointCloud2 cloud_p;
        pcl::toPCLPointCloud2(TotalCloud, cloud_p);
        sensor_msgs::PointCloud2 clu; // (ros1용)
        pcl_conversions::fromPCL(cloud_p, clu);
        clu.header.frame_id = "velodyne";
        pub_cluster.publish(clu); // (ros1용)

        // Cluster centers and clusters for visualization
        pcl::PointCloud<pcl::PointXYZI>::Ptr centers(new pcl::PointCloud<pcl::PointXYZI>);

        visualization_msgs::MarkerArray marker_array1; // (ros1용)
        visualization_msgs::MarkerArray marker_array2; // (ros1용)
        visualization_msgs::MarkerArray marker_array3; // (ros1용)

        std_msgs::ColorRGBA color; // (ros1용)
        color.r = 1.f;
        color.g = 0.f;
        color.b = 0.f;
        color.a = 1.f;

        std_msgs::ColorRGBA color2; // (ros1용)
        color2.r = 0.f;
        color2.g = 0.f;
        color2.b = 1.f;
        color2.a = 1.f;

        std_msgs::Float64MultiArray lane_coord; // (ros1용)

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*lanes_cloud, it->indices, centroid);

            pcl::PointXYZI center_point;
            center_point.x = centroid[0];
            center_point.y = centroid[1];
            center_point.z = centroid[2];

            lane_coord.data.push_back(center_point.x);
            lane_coord.data.push_back(center_point.y);

            centers->points.push_back(center_point);

            visualization_msgs::Marker marker; // (ros1용)
            marker.header.frame_id = "velodyne";
            marker.ns = "clusters";                                // namespace
            marker.id = it - cluster_indices.begin();              // 현재 클러스터의 인덱스
            marker.type = visualization_msgs::Marker::SPHERE_LIST; // (ros1용)
            marker.action = visualization_msgs::Marker::ADD;       // (ros1용)
            marker.lifetime = ros::Duration(0.1);                  // (ros1용) 0.1초가 지나면 marker 사라짐

            geometry_msgs::Point point_msg; // (ros1용)
            point_msg.x = center_point.x;
            point_msg.y = center_point.y;
            point_msg.z = center_point.z;

            marker.points.push_back(point_msg);
            marker.colors.push_back(color);

            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;

            marker_array1.markers.push_back(marker);
        }

        lane_dot_MA.publish(lane_coord); // 클러스터링 된 lane의 중심점 좌표 pub (ros1용)
        lane_coord.data.clear();
        lane_dot_.publish(marker_array1); // 클러스터링 된 lane의 중심점 좌표 시각화 (ros1용)
        marker_array1.markers.clear();

        // ----여기 바꿔-----------------------------------------------------
        // center 후보 ROI - 빨간점 , y>0) — 필요시 오른쪽/왼쪽으로 바꾸기 !!!!!!!!!
        pcl::PointCloud<pcl::PointXYZI>::Ptr centercenters(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(centers);
        cropFilter.setMin(Eigen::Vector4f(0, 0.5, -1.7, 1)); //왼쪽 차선만
        cropFilter.setMax(Eigen::Vector4f(8, 4, -0.45, 1));

        // cropFilter.setMin(Eigen::Vector4f(0, -3.5, -1.7, 1)); // 오른쪽 차선만
        // cropFilter.setMax(Eigen::Vector4f(8, -0.5, -0.45, 1));
        cropFilter.filter(*centercenters);

        // ---------------- 란삭 알고리즘으로 직선 모델 추정 ---------------
        //-----------------------------------------------------------------------------
        if (centercenters->points.size() > 2) // 여기 바꿔
        {
            std_msgs::Float64MultiArray center_lane; // (ros1용)
            std_msgs::Float64MultiArray your_point;  // (ros1용)
            for (int i = 0; i < 1; ++i)
            {
                pcl::SampleConsensusModelLine<PointT>::Ptr line_model(new pcl::SampleConsensusModelLine<PointT>(centercenters));
                pcl::ProgressiveSampleConsensus<PointT> ransac(line_model);
                // 샘플 컨센서스 모델을 사용해서 포인트 클라우드에서 선을 추정하기 위한 모델을 생성

                //////////////////////////////////////////////////////////////{
                if (chk != 2)
                {
                    ransac.setDistanceThreshold(0.5); // 여기 바꿔 / 터널 안 (일반 상황)
                }
                else
                {
                    ransac.setDistanceThreshold(0.8); // 여기 바꿔 / 터널 끝 감지 이후 (더 관대)
                }
                //////////////////////////////////////////////////////////////

                std::vector<int> inliers_;
                printf("ransac 생성\n");
                if (ransac.computeModel())
                {
                    printf("ransac 성공\n");
                    ransac.getInliers(inliers_); // 인라이어를 inliers_에 저장

                    // -----추가 1 : 후보 평가 유틸 함수들 -----
                    auto sort_by_x = [&](std::vector<int> &idxs)
                    {
                        std::sort(idxs.begin(), idxs.end(),
                                  [&](int a, int b)
                                  { return centercenters->points[a].x < centercenters->points[b].x; });
                    };

                    auto longest_segment_len = [&](const std::vector<int> &idxs) -> double
                    {
                        if (idxs.size() < 2)
                            return 0.0;
                        double best = 0.0, cur = 0.0;
                        for (size_t j = 1; j < idxs.size(); ++j)
                        {
                            const auto &p0 = centercenters->points[idxs[j - 1]];
                            const auto &p1 = centercenters->points[idxs[j]];
                            double seg = std::sqrt(std::pow(p1.x - p0.x, 2) +
                                                   std::pow(p1.y - p0.y, 2) +
                                                   std::pow(p1.z - p0.z, 2));
                            // 연속성 임계(3.0m) 그대로 사용
                            if (seg > 3.0)
                            {
                                best = std::max(best, cur);
                                cur = 0.0;
                            }
                            else
                            {
                                cur += seg;
                            }
                        }
                        return std::max(best, cur);
                    };

                    // 왼쪽 후보만 유지하는 필터 (y >= 0 영역, 0에 가까울수록 좋음)
                    auto left_lane_filter_ok = [&](const std::vector<int> &idxs) -> bool
                    {
                        bool has_left = false;
                        float best_y = std::numeric_limits<float>::infinity(); // 가장 0에 가까운 +y 찾기

                        for (int id : idxs)
                        {
                        const auto &p = centercenters->points[id];
                        if (p.y >= 0.0f)  // 왼쪽 영역
                        {
                            has_left = true;
                            if (p.y < best_y) best_y = p.y;  // 0에 더 가까운 +y
                        }
                        }
                        if (!has_left)
                        {
                        printf("오른쪽(y<=0)만 있고 왼쪽 후보 없음, 이번 직선 패스\n");
                        return false;
                        }
                        // 왼쪽으로 너무 멀면(차선이 너무 왼쪽) 버림 — 필요시 3.5→4.0 등 조정
                        if (best_y > 3.5f)
                        {
                        printf("y가 너무 큼(왼쪽으로 과도), 이번 직선 패스\n");
                        return false;
                        }
                        return true;
                    };

                    auto continuity_ok = [&](const std::vector<int> &idxs) -> bool
                    {
                        for (size_t j = 1; j < idxs.size(); ++j)
                        {
                            const auto &prev_point = centercenters->points[idxs[j - 1]];
                            const auto &curr_point = centercenters->points[idxs[j]];
                            float segment_length = std::sqrt(std::pow(curr_point.x - prev_point.x, 2) +
                                                             std::pow(curr_point.y - prev_point.y, 2) +
                                                             std::pow(curr_point.z - prev_point.z, 2));
                            if (segment_length > 3.0) // 연속성 검사는 완화
                            // 이전 점이랑 현재 점이랑 거리가 임계값을 넘으면 인라이어 다시 잡음
                            {
                                printf("거리 너무 길어\n");
                                return false;
                            }
                        }
                        return true;
                    };
                    // --------------------------------------

                    // -----추가 2 : 후보 #1 (cand1 = 현재 ransac 결과) -----
                    std::vector<int> cand1 = inliers_;
                    sort_by_x(cand1);
                    bool cand1_valid = left_lane_filter_ok(cand1) && continuity_ok(cand1);
                    double cand1_len = cand1_valid ? longest_segment_len(cand1) : 0.0;

                    // -----추가 3 : 후보 #2 (cand2 = cand1 제외하고 다시 RANSAC) -----
                    std::vector<int> inlier_mask(centercenters->points.size(), 0);
                    for (int idx : cand1)
                        inlier_mask[idx] = 1;

                    pcl::PointCloud<PointT>::Ptr remain(new pcl::PointCloud<PointT>);
                    remain->points.reserve(centercenters->points.size());
                    std::vector<int> map_old2new(centercenters->points.size(), -1);
                    for (size_t k = 0; k < centercenters->points.size(); ++k)
                    {
                        if (!cand1_valid || inlier_mask[k] == 0)
                        {
                            map_old2new[k] = static_cast<int>(remain->points.size());
                            remain->points.push_back(centercenters->points[k]);
                        }
                    }
                    remain->width = remain->points.size();
                    remain->height = 1;
                    remain->is_dense = true;

                    std::vector<int> cand2;
                    double cand2_len = 0.0;
                    bool cand2_valid = false;

                    if (remain->points.size() > 2)
                    {
                        pcl::SampleConsensusModelLine<PointT>::Ptr line_model2(new pcl::SampleConsensusModelLine<PointT>(remain));
                        pcl::ProgressiveSampleConsensus<PointT> ransac2(line_model2);
                        //////////////////////////////////////////////////////////////{
                        if (chk != 2)
                        {
                            ransac2.setDistanceThreshold(0.5); // 여기 바꿔 / 터널 안 (일반 상황)
                        }
                        else
                        {
                            ransac2.setDistanceThreshold(0.8); // 여기 바꿔 / 터널 끝 감지 이후 (더 관대)
                        }
                        //////////////////////////////////////////////////////////////
                        if (ransac2.computeModel())
                        {
                            std::vector<int> in2;
                            ransac2.getInliers(in2);

                            std::vector<int> map_new2old(remain->points.size(), -1);
                            for (size_t old = 0; old < map_old2new.size(); ++old)
                                if (map_old2new[old] >= 0)
                                    map_new2old[map_old2new[old]] = static_cast<int>(old);

                            cand2.reserve(in2.size());
                            for (int ridx : in2)
                                cand2.push_back(map_new2old[ridx]);

                            sort_by_x(cand2);
                            cand2_valid = left_lane_filter_ok(cand2) && continuity_ok(cand2);
                            cand2_len = cand2_valid ? longest_segment_len(cand2) : 0.0;
                        }
                    }

                    // -----추가 4 : 최종 선택 로직 (인라이어 최다 → 동률이면 길이 긴 것) -----
                    std::vector<int> used_inliers;
                    if (cand1_valid && cand2_valid)
                    {
                        if (cand1.size() != cand2.size())
                            used_inliers = (cand1.size() > cand2.size() ? cand1 : cand2);
                        else
                            used_inliers = (cand1_len >= cand2_len ? cand1 : cand2);
                    }
                    else if (cand1_valid)
                        used_inliers = cand1;
                    else if (cand2_valid)
                        used_inliers = cand2;
                    else
                        used_inliers.clear();

                    // 최종 후보 없으면 스킵
                    if (used_inliers.empty())
                    {
                        continue; // 이 RANSAC 모델 스킵
                    }

                    // -----추가 5 : 기존 연속성(isTooFar) 체크를 최종 후보에 대해 그대로 수행 -----
                    bool isTooFar = false;
                    for (size_t j = 1; j < used_inliers.size(); ++j)
                    {
                        const auto &prev_point = centercenters->points[used_inliers[j - 1]];
                        const auto &curr_point = centercenters->points[used_inliers[j]];

                        float segment_length = std::sqrt(std::pow(curr_point.x - prev_point.x, 2) +
                                                         std::pow(curr_point.y - prev_point.y, 2) +
                                                         std::pow(curr_point.z - prev_point.z, 2));

                        if (segment_length > 3.0) // 연속성 검사는 완화
                        // 이전 점이랑 현재 점이랑 거리가 임계값을 넘으면 인라이어 다시 잡음
                        {
                            printf("거리 너무 길어\n");
                            isTooFar = true;
                            break;
                        }
                    }
                    // -------------------------------------------------------------

                    if (!isTooFar)
                    {
                        printf("선 생성 해줄게\n");
                        // Create and add markers to visualize the line segment
                        visualization_msgs::Marker line_marker; // (ros1용)
                        line_marker.header.frame_id = "velodyne";
                        line_marker.ns = "clusters";
                        line_marker.type = visualization_msgs::Marker::LINE_STRIP; // (ros1용)
                        line_marker.action = visualization_msgs::Marker::ADD;      // (ros1용)
                        line_marker.scale.x = 0.2;
                        line_marker.pose.orientation.w = 1.0;
                        line_marker.lifetime = ros::Duration(0.1); // (ros1용)

                        // Set the color of the line segment marker
                        std_msgs::ColorRGBA color_line_segment; // (ros1용)
                        color_line_segment.r = 0.f;
                        color_line_segment.g = 1.f;
                        color_line_segment.b = 0.f;
                        color_line_segment.a = 1.0;

                        std::vector<geometry_msgs::Point> points;
                        for (const auto &index : used_inliers) // -----추가 6 : inliers_ → used_inliers
                        {
                            // printf("선에 좌표 부여 해줄게\n");
                            geometry_msgs::Point point_msg; // (ros1용)
                            point_msg.x = centercenters->points[index].x;
                            point_msg.y = centercenters->points[index].y;
                            point_msg.z = centercenters->points[index].z;
                            //---------------여기 바꿔--------ERP 추종 경로(중앙 기준 오른쪽/왼쪽으로 1.75m) — 필요시 조정....현장튜닝 ㄱㄱ
                            // 왼쪽 차선 기준
                            geometry_msgs::Point point_yours;
                            point_yours.x = point_msg.x;
                            point_yours.y = point_msg.y - 1.75;
                            point_yours.z = point_msg.z;
                            // 오른쪽 차선 기준---------------------------------------------------
                            // geometry_msgs::Point point_yours; // (ros1용)
                            // point_yours.x = point_msg.x;
                            // point_yours.y = point_msg.y + 1.75;
                            // point_yours.z = point_msg.z;

                            points.push_back(point_yours); // 차선에서 오른쪽으로 2만큼 옮긴 점들(erp 점)

                            line_marker.points.push_back(point_msg);
                            line_marker.colors.push_back(color_line_segment);

                            // Assign a unique id for each cluster
                            static int marker_id_counter = 0;
                            line_marker.id = marker_id_counter++;

                            visualization_msgs::Marker marker_yours; // (ros1용)
                            marker_yours.header.frame_id = "velodyne";
                            marker_yours.ns = "coord";
                            marker_yours.id = marker_id_counter++;
                            marker_yours.type = visualization_msgs::Marker::SPHERE_LIST; // (ros1용)
                            marker_yours.action = visualization_msgs::Marker::ADD;       // (ros1용)
                            marker_yours.lifetime = ros::Duration(0.1);                  // (ros1용)

                            geometry_msgs::Point point_coord; // (ros1용)
                            point_coord.x = point_yours.x;
                            point_coord.y = point_yours.y;
                            point_coord.z = point_yours.z;

                            marker_yours.points.push_back(point_coord);
                            marker_yours.colors.push_back(color2);

                            marker_yours.scale.x = 0.5;
                            marker_yours.scale.y = 0.5;
                            marker_yours.scale.z = 0.5;

                            marker_array3.markers.push_back(marker_yours);
                        }
                        //////요기에 인라이어
                        visualization_msgs::Marker inlier_mark; // (ros1용)
                        inlier_mark.header.frame_id = "velodyne";
                        inlier_mark.ns = "inliers";
                        inlier_mark.id = 0;                                         // 매 프레임 덮어쓰기
                        inlier_mark.type = visualization_msgs::Marker::SPHERE_LIST; // (ros1용)
                        inlier_mark.action = visualization_msgs::Marker::ADD;       // (ros1용)
                        inlier_mark.lifetime = ros::Duration(0.1);                  // (ros1용)
                        inlier_mark.scale.x = 0.25;
                        inlier_mark.scale.y = 0.25;
                        inlier_mark.scale.z = 0.25;
                        inlier_mark.color.r = 1.0;
                        inlier_mark.color.g = 1.0;
                        inlier_mark.color.b = 0.0;
                        inlier_mark.color.a = 1.0;

                        for (const auto &idx : used_inliers) // -----추가 7 : inliers_ → used_inliers
                        {
                            geometry_msgs::Point p; // (ros1용)
                            p.x = centercenters->points[idx].x;
                            p.y = centercenters->points[idx].y;
                            p.z = centercenters->points[idx].z;
                            inlier_mark.points.push_back(p);
                        }

                        marker_array2.markers.push_back(inlier_mark);
                        ////////////////////
                        std::sort(points.begin(), points.end(), [](const geometry_msgs::Point &a, const geometry_msgs::Point &b)
                                  { return a.x < b.x; });

                        for (const auto &sorted_point : points)
                        {
                            center_lane.data.push_back(sorted_point.x);
                            center_lane.data.push_back(sorted_point.y);
                        }
                        center_.publish(center_lane); // (ros1용)
                        center_lane.data.clear();

                        marker_array2.markers.push_back(line_marker);
                        lane_.publish(marker_array2); // (ros1용)
                        marker_array2.markers.clear();

                        coord_dot_.publish(marker_array3); // (ros1용)
                        marker_array3.markers.clear();
                    }
                }
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LiDAR_lane"); // (ros1용)
    ros::NodeHandle nh;                  // (ros1용)
    LiDAR_lane node(nh);                 // (ros1용)
    ros::spin();                         // (ros1용)
    return 0;
}