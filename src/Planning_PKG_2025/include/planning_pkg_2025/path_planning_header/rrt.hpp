/***
 * 작성자: 정성윤
 * 설명: rrt* 관련 클래스 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef RRT_H_
#define RRT_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include <tuple>
#include <random>
#include <limits>
#include <sstream>
#include <iomanip>

class Node
{
public:
    double x;
    double y;
    double yaw;
    double cost;
    int parent; // Index of parent node in nodeList

    Node(double x_, double y_, double yaw_)
        : x(x_), y(y_), yaw(yaw_), cost(0.0), parent(-1) {}

    std::string to_string() const
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << x << "," << y << "," << yaw * 180.0 / M_PI << "," << cost;
        return ss.str();
    }

    bool operator==(const Node &other) const
    {
        return x == other.x && y == other.y && yaw == other.yaw && cost == other.cost;
    }
};

class RRT
{
public:
    Node start;
    double startYaw;
    double planDistance;
    double expandDis;
    double turnAngle;
    int maxDepth;
    int maxIter;
    std::vector<std::tuple<double, double, double>> obstacleList;
    std::vector<std::tuple<double, double, double>> rrtTargets;

    std::vector<Node> nodeList;
    std::vector<Node> leafNodes;

    std::default_random_engine generator;

    RRT(std::vector<double> start_, double planDistance_, std::vector<std::tuple<double, double, double>> obstacleList_,
        double expandDis_ = 0.5, double turnAngle_ = 30, int maxIter_ = 50, std::vector<std::tuple<double, double, double>> rrtTargets_ = {})
        : start(start_[0], start_[1], start_[2]),
          startYaw(start_[2]),
          planDistance(planDistance_),
          expandDis(expandDis_),
          turnAngle(turnAngle_ * M_PI / 180.0),
          maxDepth(static_cast<int>(planDistance_ / expandDis_)),
          maxIter(maxIter_),
          obstacleList(obstacleList_),
          rrtTargets(rrtTargets_)
    {
        nodeList.push_back(start);
        generator.seed(std::time(0));
    }

    std::pair<std::vector<Node>, std::vector<Node>> Planning()
    {
        nodeList.clear();
        nodeList.push_back(start);
        leafNodes.clear();

        for (int i = 0; i < maxIter; ++i)
        {
            std::vector<double> rnd = get_random_point_from_target_list();

            int nind = GetNearestListIndex(nodeList, rnd);

            Node nearestNode = nodeList[nind];

            if (nearestNode.cost >= planDistance)
            {
                continue;
            }

            Node newNode = steerConstrained(rnd, nind);

            if (std::find(nodeList.begin(), nodeList.end(), newNode) != nodeList.end())
            {
                continue;
            }

            if (__CollisionCheck(newNode, obstacleList))
            {
                nodeList.push_back(newNode);

                if (newNode.cost >= planDistance)
                {
                    leafNodes.push_back(newNode);
                }
            }
        }

        return std::make_pair(nodeList, leafNodes);
    }

    std::vector<double> get_random_point_from_target_list()
    {
        double maxTargetAroundDist = 3.0;

        if (rrtTargets.empty())
        {
            return get_random_point();
        }

        std::uniform_int_distribution<int> target_dist(0, rrtTargets.size() - 1);
        int targetId = target_dist(generator);

        double x = std::get<0>(rrtTargets[targetId]);
        double y = std::get<1>(rrtTargets[targetId]);
        double oSize = std::get<2>(rrtTargets[targetId]);

        std::uniform_real_distribution<double> angle_dist(0.0, 2 * M_PI);
        double randAngle = angle_dist(generator);

        std::uniform_real_distribution<double> dist_dist(oSize, maxTargetAroundDist);
        double randDist = dist_dist(generator);

        double finalX = x + randDist * std::cos(randAngle);
        double finalY = y + randDist * std::sin(randAngle);

        return {finalX, finalY};
    }

    std::vector<double> get_random_point()
    {
        std::uniform_real_distribution<double> x_dist(0.0, planDistance);
        std::uniform_real_distribution<double> y_dist(-planDistance, planDistance);

        double randX = x_dist(generator);
        double randY = y_dist(generator);
        std::vector<double> rnd = {randX, randY};

        double cos_yaw = std::cos(startYaw);
        double sin_yaw = std::sin(startYaw);
        double rotatedX = cos_yaw * rnd[0] - sin_yaw * rnd[1];
        double rotatedY = sin_yaw * rnd[0] + cos_yaw * rnd[1];

        rotatedX += start.x;
        rotatedY += start.y;

        return {rotatedX, rotatedY};
    }

    int GetNearestListIndex(const std::vector<Node> &nodeList, const std::vector<double> &rnd)
    {
        double minDist = std::numeric_limits<double>::max();
        int minInd = -1;
        for (size_t i = 0; i < nodeList.size(); ++i)
        {
            double dx = nodeList[i].x - rnd[0];
            double dy = nodeList[i].y - rnd[1];
            double dist = dx * dx + dy * dy;
            if (dist < minDist)
            {
                minDist = dist;
                minInd = i;
            }
        }
        return minInd;
    }

    Node steerConstrained(const std::vector<double> &rnd, int nind)
    {
        Node nearestNode = nodeList[nind];
        double theta = std::atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x);

        double angleChange = pi_2_pi(theta - nearestNode.yaw);

        double angle30degree = 30.0 * M_PI / 180.0;

        if (angleChange > angle30degree)
        {
            angleChange = turnAngle;
        }
        else if (angleChange >= -angle30degree)
        {
            angleChange = 0.0;
        }
        else
        {
            angleChange = -turnAngle;
        }

        Node newNode = nearestNode;
        newNode.yaw += angleChange;
        newNode.x += expandDis * std::cos(newNode.yaw);
        newNode.y += expandDis * std::sin(newNode.yaw);

        newNode.cost += expandDis;
        newNode.parent = nind;

        return newNode;
    }

    double pi_2_pi(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    bool __CollisionCheck(const Node &node, const std::vector<std::tuple<double, double, double>> &obstacleList)
    {
        for (const auto &obstacle : obstacleList)
        {
            double ox = std::get<0>(obstacle);
            double oy = std::get<1>(obstacle);
            double size = std::get<2>(obstacle);
            double dx = ox - node.x;
            double dy = oy - node.y;
            double d = dx * dx + dy * dy;
            if (d <= size * size)
            {
                return false; // collision
            }
        }
        return true; // safe
    }
};

#endif
