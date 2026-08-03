/***
 * 작성자: 정성윤
 * 설명: 들로네 삼각분할 관련 클래스 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef DELAUNAYCLASSIFIER_H_
#define DELAUNAYCLASSIFIER_H_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <map>
#include <vector>
#include <iostream>

// CGAL 커널 정의
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Point_2 Point;

// DelaunayClassifier 클래스 정의
class DelaunayClassifier // TODO: class 0번에 대해서 처리 추가
{
private:
    // Delaunay 삼각분할 객체
    Delaunay delaunay_triangulation;

    // 좌표와 클래스 정보를 저장할 맵
    std::map<Point, int> point_class_map;

    // 분류된 선분 벡터
    std::vector<std::pair<std::vector<double>, std::vector<double>>> class1_edges;
    std::vector<std::pair<std::vector<double>, std::vector<double>>> class2_edges;
    std::vector<std::pair<std::vector<double>, std::vector<double>>> mixed_class_edges;

    // 클래스 분류용 함수
    int getClass(const std::vector<double> &p1, const std::vector<double> &p2) const
    {
        int class1 = point_class_map.at(Point(p1[0], p1[1]));
        int class2 = point_class_map.at(Point(p2[0], p2[1]));

        if (class1 == 1 && class2 == 1)
            return 1; // 1번 클래스끼리 연결된 선분
        else if (class1 == 2 && class2 == 2)
            return 2; // 2번 클래스끼리 연결된 선분
        else
            return 3; // 두 클래스가 섞여있는 선분
    }

public:
    // 점을 추가하고 분류 정보를 저장하는 함수
    void addPoint(const std::vector<double> &point)
    {
        int class_id = static_cast<int>(point[2]);
        Point p(point[0], point[1]); // x, y 좌표 사용
        delaunay_triangulation.insert(p);
        point_class_map[p] = class_id;
    }

    // 점과 클래스 정보를 바탕으로 선분을 분류하는 함수
    void classifyEdges()
    {
        class1_edges.clear();
        class2_edges.clear();
        mixed_class_edges.clear();

        for (auto it = delaunay_triangulation.finite_edges_begin(); it != delaunay_triangulation.finite_edges_end(); ++it)
        {
            auto segment = delaunay_triangulation.segment(*it);

            std::vector<double> p1 = {segment.source().x(), segment.source().y()};
            std::vector<double> p2 = {segment.target().x(), segment.target().y()};

            int edge_class = getClass(p1, p2);
            if (edge_class == 1)
                class1_edges.emplace_back(p1, p2);
            else if (edge_class == 2)
                class2_edges.emplace_back(p1, p2);
            else
                mixed_class_edges.emplace_back(p1, p2);
        }
    }

    // 분류된 선분을 얻는 함수
    const std::vector<std::pair<std::vector<double>, std::vector<double>>> &getClass1Edges() const { return class1_edges; }
    const std::vector<std::pair<std::vector<double>, std::vector<double>>> &getClass2Edges() const { return class2_edges; }
    const std::vector<std::pair<std::vector<double>, std::vector<double>>> &getMixedClassEdges() const { return mixed_class_edges; }
};

#endif
