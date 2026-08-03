/***
 * 작성자: 정성윤
 * 설명: 다양한 데이터에 대한 클러스터링을 수행하는 클래스들
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef STATIC_CLUSTERS_H_
#define STATIC_CLUSTERS_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

/***
 * 클래스: StaticBaseCluster
 * 설명: 클러스터링의 기본 기능을 제공하는 추상 클래스
 ***/
class StaticBaseCluster
{
protected:
    double epsilon_;
    int min_points_size_;
    int max_points_size_;
    int next_cluster_id_;
    std::vector<std::vector<double>> points_;
    std::vector<std::vector<std::vector<double>>> clusters_;
    std::vector<std::vector<double>> cluster_means_;
    std::vector<std::vector<double>> noise_;
    const double new_weight_;

    // 두 점 사이의 유클리드 거리 계산
    double distance(const std::vector<double> &p1, const std::vector<double> &p2) const
    {
        return std::hypot(p1[0] - p2[0], p1[1] - p2[1]);
    }

    // 클러스터의 포인트와 주어진 포인트의 적합성 검사 (가상 함수)
    virtual bool checkCluster(const std::vector<double> &point, const std::vector<std::vector<double>> &cluster) = 0;

    // 클러스터와 같은 클래스인지 확인 (가상 함수)
    virtual bool checkSameClass(const std::vector<double> &point, const std::vector<std::vector<double>> &cluster) = 0;

    // 주어진 포인트와 인접한 포인트를 검색 (가상 함수)
    virtual void regionQuery(const std::vector<double> &point, const std::vector<std::vector<double>> &points,
                             std::vector<std::vector<double>> &query_points, std::vector<std::vector<double>> &else_points) = 0;

    // 클러스터 평균 계산 (가상 함수)
    virtual void calculateClusterTrustPoint(std::vector<double> &point, int cluster_id) = 0;

    // 클러스터에 포인트 추가 (가상 함수)
    virtual void addPointToCluster(std::vector<double> &point, int cluster_id) = 0;

    // 새로운 클러스터 생성 처리 (가상 함수)
    virtual void handleNewCluster(std::vector<std::vector<double>> &neighbors, std::vector<double> &point) = 0;

    // 클러스터 확장
    bool expandCluster(std::vector<double> &point)
    {
        for (int i = 0; i < clusters_.size(); ++i)
        {
            if (checkCluster(point, clusters_[i]))
            {
                if (clusters_[i].size() >= max_points_size_ || !checkSameClass(point, clusters_[i]))
                {
                    return false;
                }
                // 포인트를 클러스터에 추가하고 평균 갱신 (가중치 적용)
                addPointToCluster(point, i);
                calculateClusterTrustPoint(point, i);
                return true;
            }
        }

        // 새로운 클러스터 생성 여부 판단
        std::vector<std::vector<double>> neighbors;
        std::vector<std::vector<double>> not_neighbors;
        regionQuery(point, noise_, neighbors, not_neighbors);

        if (neighbors.size() >= min_points_size_ - 1)
        {
            handleNewCluster(neighbors, point);
            noise_ = not_neighbors;
            return true;
        }
        else
        {
            noise_.push_back(point);
        }

        return false;
    }

public:
    StaticBaseCluster(double epsilon, int min_points, int max_points, double weight_for_new = 0.9)
        : epsilon_(epsilon), min_points_size_(min_points), max_points_size_(max_points), new_weight_(weight_for_new), next_cluster_id_(0) {}

    // 클러스터 평균 반환
    const std::vector<std::vector<double>> &getRefClusterTrustPoints() const
    {
        return cluster_means_;
    }

    // 새로운 포인트로 클러스터 업데이트
    void update(std::vector<double> point)
    {
        points_.push_back(point);
        expandCluster(points_.back());
    }

    void clearCluster()
    {
        points_.clear();
        clusters_.clear();
        cluster_means_.clear();
        noise_.clear();

        next_cluster_id_ = 0;
    }

    // 클러스터 결과 출력
    virtual void printClusters() const
    {
        std::cout << std::fixed;
        std::cout.precision(10);
        std::cout << "cluster_means_size: " << cluster_means_.size() << "\n";
        for (int i = 0; i < next_cluster_id_; ++i)
        {
            std::cout << "Cluster " << i << ":\n";
            for (const auto &point : clusters_[i])
            {
                std::cout << "  (" << point[0] << ", " << point[1] << ")\n";
            }
            std::cout << "   mean: (" << cluster_means_[i][0] << ", " << cluster_means_[i][1] << ")\n";
        }
        std::cout << "Noise points:\n";
        for (const auto &point : noise_)
        {
            std::cout << "  (" << point[0] << ", " << point[1] << ")\n";
        }
    }
};

/***
 * 클래스: StaticPosCluster
 * 설명: 좌표를 기반으로 클러스터링을 수행하는 클래스
 ***/
class StaticPosCluster : public StaticBaseCluster
{
protected:
    // 클러스터의 적합성 검사 구현
    bool checkCluster(const std::vector<double> &point, const std::vector<std::vector<double>> &cluster) override
    {
        for (const auto &p : cluster)
        {
            if (distance(point, p) > epsilon_)
            {
                return false;
            }
        }
        return true;
    }

    // 클러스터와 같은 클래스인지 확인 (가상 함수)
    bool checkSameClass(const std::vector<double> &point, const std::vector<std::vector<double>> &cluster) override
    {
        return true;
    }

    // 인접한 포인트 검색 구현
    void regionQuery(const std::vector<double> &point, const std::vector<std::vector<double>> &points,
                     std::vector<std::vector<double>> &query_points, std::vector<std::vector<double>> &else_points) override
    {
        for (const auto &p : points)
        {
            if (distance(point, p) <= epsilon_)
            {
                query_points.push_back(p);
            }
            else
            {
                else_points.push_back(p);
            }
        }
    }

    // 클러스터 평균 계산 (가중치 적용)
    void calculateClusterTrustPoint(std::vector<double> &point, int cluster_id) override
    {
        int cluster_size = clusters_[cluster_id].size();
        double old_weight = 1.0 - new_weight_;

        cluster_means_[cluster_id][0] = (cluster_means_[cluster_id][0] * old_weight * (cluster_size - 1) + point[0] * new_weight_) / (old_weight * (cluster_size - 1) + new_weight_);
        cluster_means_[cluster_id][1] = (cluster_means_[cluster_id][1] * old_weight * (cluster_size - 1) + point[1] * new_weight_) / (old_weight * (cluster_size - 1) + new_weight_);
    }

    // 클러스터에 포인트 추가 구현
    void addPointToCluster(std::vector<double> &point, int cluster_id) override
    {
        clusters_[cluster_id].push_back(point);
    }

    // 새로운 클러스터 생성 처리 구현
    void handleNewCluster(std::vector<std::vector<double>> &neighbors, std::vector<double> &point) override
    {
        clusters_.push_back(neighbors);
        clusters_[next_cluster_id_].push_back(point);

        double old_weight = 1.0 - new_weight_;

        double mean_x = clusters_[next_cluster_id_][0][0];
        double mean_y = clusters_[next_cluster_id_][0][1];

        // 기존 포인트들에 대해서는 old_weight, 새로운 포인트에 대해서는 new_weight 적용
        for (size_t i = 1; i < clusters_[next_cluster_id_].size(); ++i)
        {
            mean_x = (mean_x * old_weight * i + clusters_[next_cluster_id_][i][0] * new_weight_) / (old_weight * i + new_weight_);
            mean_y = (mean_y * old_weight * i + clusters_[next_cluster_id_][i][1] * new_weight_) / (old_weight * i + new_weight_);
        }

        cluster_means_.push_back({mean_x, mean_y});
        next_cluster_id_++;
    }

public:
    StaticPosCluster(double epsilon, int min_points, int max_points, double weight_for_new = 0.9)
        : StaticBaseCluster(epsilon, min_points, max_points, weight_for_new) {}
};

/***
 * 클래스: StaticClassCluster
 * 설명: 플래그를 기반으로 클러스터링을 수행하는 클래스
 ***/
class StaticClassCluster : public StaticBaseCluster
{
protected:
    // 클러스터의 적합성 검사 구현
    bool checkCluster(const std::vector<double> &point, const std::vector<std::vector<double>> &cluster) override
    {
        for (const auto &p : cluster)
        {
            if (distance(point, p) > epsilon_)
            {
                return false;
            }
        }
        return true;
    }

    // 클러스터와 같은 클래스인지 확인 (가상 함수)
    bool checkSameClass(const std::vector<double> &point, const std::vector<std::vector<double>> &cluster) override
    {
        return point[2] == cluster[0][2];
    }

    // 인접한 포인트 검색 구현
    void regionQuery(const std::vector<double> &point, const std::vector<std::vector<double>> &points,
                     std::vector<std::vector<double>> &query_points, std::vector<std::vector<double>> &else_points) override
    {
        for (const auto &p : points)
        {
            if (distance(point, p) <= epsilon_ && point[2] == p[2])
            {
                query_points.push_back(p);
            }
            else
            {
                else_points.push_back(p);
            }
        }
    }

    // 클러스터 평균 계산 (가중치 적용)
    void calculateClusterTrustPoint(std::vector<double> &point, int cluster_id) override
    {
        int cluster_size = clusters_[cluster_id].size();
        double old_weight = 1.0 - new_weight_;

        cluster_means_[cluster_id][0] = (cluster_means_[cluster_id][0] * old_weight * (cluster_size - 1) + point[0] * new_weight_) / (old_weight * (cluster_size - 1) + new_weight_);
        cluster_means_[cluster_id][1] = (cluster_means_[cluster_id][1] * old_weight * (cluster_size - 1) + point[1] * new_weight_) / (old_weight * (cluster_size - 1) + new_weight_);
    }

    // 클러스터에 포인트 추가 구현
    void addPointToCluster(std::vector<double> &point, int cluster_id) override
    {
        clusters_[cluster_id].push_back(point);
    }

    // 새로운 클러스터 생성 처리 구현
    void handleNewCluster(std::vector<std::vector<double>> &neighbors, std::vector<double> &point) override
    {
        clusters_.push_back(neighbors);
        clusters_[next_cluster_id_].push_back(point);

        double old_weight = 1.0 - new_weight_;

        double mean_x = clusters_[next_cluster_id_][0][0];
        double mean_y = clusters_[next_cluster_id_][0][1];

        // 기존 포인트들에 대해서는 old_weight, 새로운 포인트에 대해서는 new_weight 적용
        for (size_t i = 1; i < clusters_[next_cluster_id_].size(); ++i)
        {
            mean_x = (mean_x * old_weight * i + clusters_[next_cluster_id_][i][0] * new_weight_) / (old_weight * i + new_weight_);
            mean_y = (mean_y * old_weight * i + clusters_[next_cluster_id_][i][1] * new_weight_) / (old_weight * i + new_weight_);
        }

        cluster_means_.push_back({mean_x, mean_y, neighbors[0][2]});
        next_cluster_id_++;
    }

public:
    StaticClassCluster(double epsilon, int min_points, int max_points, double weight_for_new = 0.9)
        : StaticBaseCluster(epsilon, min_points, max_points, weight_for_new) {}

    // 클러스터 결과 출력 (플래그 포함)
    void printClusters() const override
    {
        std::cout << std::fixed;
        std::cout.precision(10);
        std::cout << "cluster_means_size: " << cluster_means_.size() << "\n";
        for (int i = 0; i < next_cluster_id_; ++i)
        {
            std::cout << "Cluster " << i << ":\n";
            for (const auto &point : clusters_[i])
            {
                std::cout << "  (" << point[0] << ", " << point[1] << ")\n";
            }
            std::cout << "   mean: (" << cluster_means_[i][0] << ", " << cluster_means_[i][1] << ")   flag: " << cluster_means_[i][2] << "\n";
        }
        std::cout << "Noise points:\n";
        for (const auto &point : noise_)
        {
            std::cout << "  (" << point[0] << ", " << point[1] << ")   " << point[2] << "\n ";
        }
    }
};

#endif
