/***
 * 작성자: 정성윤
 * 설명: KDTree 클래스 헤더파일 (참고: cgal 이용)
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef KDTREE_H_
#define KDTREE_H_

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <vector>
#include <limits>
#include <cmath>
#include <exception>
#include <functional>

template <class PointT>
class KDTree
{
private:
    typedef CGAL::Simple_cartesian<double> K;
    typedef CGAL::Search_traits_2<K> Traits;
    typedef CGAL::Kd_tree<Traits> Tree;
    typedef CGAL::Fuzzy_sphere<Traits> Sphere;
    typedef CGAL::Orthogonal_k_neighbor_search<Traits> KnnSearch;
    typedef typename KnnSearch::Tree::Point_d Point;

    class Exception : public std::exception
    {
        using std::exception::exception;
    };

    Tree *tree_;
    std::vector<Point> points_;

    Point toCGALPoint(const std::vector<double> &p) const
    {
        return Point(p[0], p[1]);
    }

    Point toCGALPoint(const double *p) const
    {
        return Point(p[0], p[1]);
    }

    PointT fromCGALPoint(const Point &p) const
    {
        return PointT({p[0], p[1]});
    }

public:
    KDTree() : tree_(nullptr) {};
    KDTree(const std::vector<PointT> &points) : tree_(nullptr) { build(points); }

    ~KDTree() { clear(); }

    void build(const std::vector<PointT> &points) // 새로운 데이터에 대한 kd tree update 함수
    {
        clear();
        points_.reserve(points.size());

        for (const auto &point : points)
            points_.emplace_back(toCGALPoint(point));

        tree_ = new Tree(points_.begin(), points_.end());
    }

    void clear() // kd tree 초기화 함수
    {
        delete tree_;
        tree_ = nullptr;
        points_.clear();
    }

    KDTree(const KDTree &other) : tree_(nullptr) // 복사 생성자
    {
        std::cout << "KD Tree Copy Constructor called" << std::endl;
        if (other.tree_)
        {
            points_ = other.points_; // 점 데이터 복사
            tree_ = new Tree(points_.begin(), points_.end());
        }
    }

    KDTree(KDTree &&other) noexcept : tree_(other.tree_), points_(std::move(other.points_)) // 이동 생성자
    {
        std::cout << "KD Tree Move Constructor called" << std::endl;
        other.tree_ = nullptr;
    }

    KDTree &operator=(const KDTree &other) // 할당 연산자
    {
        if (this != &other)
        {
            clear();                 // 기존 트리 클리어
            points_ = other.points_; // 점 데이터 복사
            tree_ = new Tree(points_.begin(), points_.end());
        }
        std::cout << "KD Tree Copy Assignment Operator called" << std::endl;
        return *this;
    }

    KDTree &operator=(KDTree &&other) noexcept // 이동 할당 연산자
    {
        if (this != &other)
        {
            clear(); // 기존 리소스 해제

            // 소유권 이동
            tree_ = other.tree_;
            points_ = std::move(other.points_);

            // 원본 객체 초기화
            other.tree_ = nullptr;
        }
        std::cout << "KD Tree Move Assignment Operator called" << std::endl;
        return *this;
    }

    bool validate() const
    {
        // CGAL Kd_tree는 기본적으로 유효성을 보장
        return true;
    }

    template <typename T>
    PointT getClosestPos(const T(&query), double *minDist = nullptr) const // 최근접 이웃 탐색 함수
    {
        if (!tree_)
            throw Exception();

        Point query_point = toCGALPoint(query);
        KnnSearch search(*tree_, query_point, 1);
        auto it = search.begin();
        double dist = std::sqrt(it->second);

        if (minDist)
            *minDist = dist;

        return fromCGALPoint(it->first);
    }

    template <typename T>
    int getClosestIndex(const T(&query), double *minDist = nullptr) const
    {
        if (!tree_)
            throw Exception();

        Point query_point = toCGALPoint(query);
        KnnSearch search(*tree_, query_point, 1);
        auto it = search.begin();
        double dist = std::sqrt(it->second);

        if (minDist)
            *minDist = dist;

        // 찾은 포인트의 인덱스를 points_ 벡터에서 검색
        auto found = std::find(points_.begin(), points_.end(), it->first);
        if (found != points_.end())
        {
            return std::distance(points_.begin(), found);
        }
        else
        {
            throw Exception(); // 찾은 포인트가 points_ 벡터에 없는 경우 예외 발생
        }
    }

    std::vector<int> knnSearch(const PointT &query, int k) const
    {
        if (!tree_)
            throw Exception();

        Point query_point = toCGALPoint(query);
        KnnSearch search(*tree_, query_point, k);
        std::vector<int> indices;
        for (auto it = search.begin(); it != search.end(); ++it)
        {
            auto found = std::find(points_.begin(), points_.end(), it->first);
            indices.push_back(std::distance(points_.begin(), found));
        }
        return indices;
    }

    std::vector<int> radiusSearch(const PointT &query, double radius) const
    {
        if (!tree_)
            throw Exception();

        Point query_point = toCGALPoint(query);
        Sphere sphere(query_point, radius);
        std::vector<Point> result;
        tree_->search(std::back_inserter(result), sphere);

        std::vector<int> indices;
        for (const auto &point : result)
        {
            auto found = std::find(points_.begin(), points_.end(), point);
            indices.push_back(std::distance(points_.begin(), found));
        }
        return indices;
    }
};

#endif
