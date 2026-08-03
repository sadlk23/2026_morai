/***
 * 작성자: 정성윤
 * 설명: Path 관련 클래스 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef PATH_H_
#define PATH_H_

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <fstream>
#include "kdtree.hpp"

class Path
{
private:
    std::string path_name_;
    std::vector<std::vector<double>> pos_arr_;
    std::vector<double> yaw_arr_;
    std::vector<double> k_arr_;
    KDTree<std::vector<double>> kd_tree_;

public:
    /* 생성자 */
    Path(std::string f_name, std::vector<std::vector<double>> &pos_arr, std::vector<double> &yaw_arr, std::vector<double> &k_arr);
    Path(const std::string f_name);
    // 복사 생성자
    Path(const Path &other);
    // 이동 생성자
    Path(Path &&other) noexcept;
    // 복사 할당 연산자
    Path &operator=(const Path &other);
    // 이동 할당 연산자
    Path &operator=(Path &&other) noexcept;

    /* getter 함수 */
    const std::string getPathName() const;
    const std::vector<std::vector<double>> &getRefPosArr() const;
    const std::vector<double> &getRefYawArr() const;
    const std::vector<double> &getRefKArr() const;
    const KDTree<std::vector<double>> &getRefKDTree() const;

    /* setter 함수 */
    void setPosArr(const std::vector<std::vector<double>> &pos_arr);
    void setYawArr(const std::vector<double> &yaw_arr);
    void setKArr(const std::vector<double> &k_arr);

    /* 기능 관련 함수 */
    void updateKDTree();                                                                                                                   // kd_tree 최신화 함수
    void updatePath(size_t start_index, size_t end_index, const Path &replace_path);                                                       // Path 일부분을 교체하는 함수(두 Path 크기 같을 때)
    void updatePath(size_t start_index, size_t end_index, const Path &replace_path, size_t replace_start_index, size_t replace_end_index); // path 일부분을 교체하는 함수(두 Path 크기 다를 때)
    template <typename T>
    std::vector<double> getClosestPos(const T(&pos)) const // 가장 가까운 좌표 반환 함수
    {
        std::vector<double> closest_pos = kd_tree_.getClosestPos(pos);
        return closest_pos;
    }
    template <typename T>
    double getClosestDistance(const T(&pos)) const // 가장 가까운 거리 반환 함수
    {
        std::vector<double> closest_pos = kd_tree_.getClosestPos(pos);
        double closest_dis = std::hypot(closest_pos[0] - pos[0], closest_pos[1] - pos[1]);
        return closest_dis;
    }
    template <typename T>
    int getClosestIndex(const T(&pos)) const // 가장 가까운 index 반환 함수
    {
        int closest_index = kd_tree_.getClosestIndex(pos);
        return closest_index;
    }

    /* test용 함수 */
    void printAllData() const; // 모든 값 출력 함수
};

#endif
