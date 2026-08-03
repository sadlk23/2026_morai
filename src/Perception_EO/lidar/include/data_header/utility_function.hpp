/***
 * 작성자: 정주훈
 * 설명: LiDAR의 raw_data, down_sampling, crop_Filter의 함수화
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef UTILITY_FUNCTION_H_
#define UTILITY_FUNCTION_H_

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <ros/ros.h>

// 타입
#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

class Utility_Function
{
private:
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> clusters;

public:
    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudROS(
        const sensor_msgs::PointCloud2::ConstPtr& input)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_origin(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input, *cloud_origin);
        return cloud_origin;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr DownSamplingVoxel(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_origin, float size_x, float size_y, float size_z)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> VG;
        VG.setInputCloud(cloud_origin);
        VG.setLeafSize(size_x, size_y, size_z);
        VG.filter(*cloud_down);
        return cloud_down;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr CropFilter(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_down, Eigen::Vector4f min_pt, Eigen::Vector4f max_pt)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_crop(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(cloud_down);
        cropFilter.setMin(min_pt); // x,y,z,1
        cropFilter.setMax(max_pt);
        cropFilter.filter(*cloud_crop);
        return cloud_crop;
    }
    std::vector<pcl::PointIndices> ClusterEuclidean(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_crop, bool max_ok, float tolerance, int min_size, int max_size)
    {
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud_crop);
        std::vector<pcl::PointIndices> cloud_clust;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setInputCloud(cloud_crop);
        ec.setClusterTolerance(tolerance); // 여기도 수정해야 함.
        ec.setMinClusterSize(min_size);
        if (max_ok)
        {
            ec.setMaxClusterSize(max_size);
        }
        ec.setSearchMethod(tree);
        ec.extract(cloud_clust);

        return cloud_clust;
    }
    void ExtractClusters(
        pcl::PointCloud<pcl::PointXYZI> &TotalCloud,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_crop,
        const std::vector<pcl::PointIndices> &cloud_clust,
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> &clusters)
    {
        int ii = 0;
        clusters.clear();
        for (auto it = cloud_clust.begin(); it != cloud_clust.end(); ++it, ++ii)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
            for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                cluster->points.push_back(cloud_crop->points[*pit]);
                pcl::PointXYZI pt = cloud_crop->points[*pit];
                pcl::PointXYZI pt2;
                pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
                pt2.intensity = (float)(ii + 1);
                TotalCloud.push_back(pt2);
            }
            cluster->width = cluster->size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }
    }

    void GetMinMaxCloudPoint(
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> &clusters,
        geometry_msgs::Point &min_point,
        geometry_msgs::Point &max_point,
        size_t i)
    {
        Eigen::Vector4f min_p, max_p;

        pcl::getMinMax3D(*clusters[i], min_p, max_p);

        min_point.x = min_p[0];
        min_point.y = min_p[1];
        min_point.z = min_p[2];

        max_point.x = max_p[0];
        max_point.y = max_p[1];
        max_point.z = max_p[2];
    }
    void GetSenCloudPoint(
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> &clusters,
        geometry_msgs::Point &center_point,
        size_t i)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*clusters[i], centroid);
        center_point.x = centroid[0];
        center_point.y = centroid[1];
        center_point.z = centroid[2];
    }

    visualization_msgs::Marker showRoi_CUBE(
        std::string ns_, int32_t id_, Eigen::Vector4f min_pt, Eigen::Vector4f max_pt,
        std::string frameid = "velodyne", double color_r = 0.0, double color_g = 1.0, double color_b = 0.0, double color_a = 0.3)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frameid;
        marker.ns = ns_;
        marker.id = id_;
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = max_pt[0] - min_pt[0];
        marker.scale.y = max_pt[1] - min_pt[1];
        marker.scale.z = max_pt[2] - min_pt[2];
        marker.pose.position.x = (max_pt[0] + min_pt[0]) / 2;
        marker.pose.position.y = (max_pt[1] + min_pt[1]) / 2;
        marker.pose.position.z = (max_pt[2] + min_pt[2]) / 2;
        marker.color.r = color_r;
        marker.color.g = color_g;
        marker.color.b = color_b;
        marker.color.a = color_a;
        return marker;
    }

    visualization_msgs::Marker showRoi_POINT(
        std::string ns_, int32_t id_,
        double color_r = 0.5, double color_g = 0.5, double color_b = 0.5, double color_a = 1,
        std::string frameid = "velodyne")
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frameid;
        marker.ns = ns_;
        marker.id = id_;
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.color.r = color_r;
        marker.color.g = color_g;
        marker.color.b = color_b;
        marker.color.a = color_a;
        return marker;
    }

    visualization_msgs::Marker showRoi_LINE(
        std::string ns_, int32_t id_,
        double color_r = 0.5, double color_g = 0.5, double color_b = 0.5, double color_a = 0.4,
        std::string frameid = "velodyne")
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frameid;
        marker.ns = ns_;
        marker.id = id_;
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.1;
        marker.color.r = color_r;
        marker.color.g = color_g;
        marker.color.b = color_b;
        marker.color.a = color_a;
        return marker;
    }
    void sortClustersByDistance(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> &clusters)
    {
        std::vector<std::pair<float, pcl::PointCloud<pcl::PointXYZI>::Ptr>> distance_cluster_pairs;

        for (const auto &cluster : clusters)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);
            float distance = std::sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1] + centroid[2] * centroid[2]);
            distance_cluster_pairs.push_back(std::make_pair(distance, cluster));
        }

        // 거리 기준으로 정렬
        std::sort(distance_cluster_pairs.begin(), distance_cluster_pairs.end(),
                  [](const auto &a, const auto &b)
                  { return a.first < b.first; });

        // 정렬된 클러스터를 다시 clusters 벡터에 저장
        clusters.clear();
        for (const auto &pair : distance_cluster_pairs)
        {
            clusters.push_back(pair.second);
        }
    }
};
#endif
