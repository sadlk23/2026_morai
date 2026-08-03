/***
 * 작성자: 정성윤
 * 설명: Path 관련 클래스 정의
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#include "path.hpp"

Path::Path(std::string f_name, std::vector<std::vector<double>> &pos_arr, std::vector<double> &yaw_arr, std::vector<double> &k_arr) : path_name_{f_name}, pos_arr_{pos_arr}, yaw_arr_{yaw_arr}, k_arr_{k_arr}, kd_tree_{pos_arr}
{
    std::cout << f_name << " path 초기화\n";
}
Path::Path(const std::string f_name) : path_name_{f_name}
{
    std::cout << f_name << " path 초기화\n";
}

// 복사 생성자
Path::Path(const Path &other)
    : path_name_(other.path_name_), pos_arr_(other.pos_arr_), yaw_arr_(other.yaw_arr_), k_arr_(other.k_arr_), kd_tree_(other.kd_tree_)
{
    std::cout << "Path Copy Constructor called" << std::endl;
    std::cout << path_name_ << " path Copy\n";
}

// 이동 생성자
Path::Path(Path &&other) noexcept
    : path_name_(std::move(other.path_name_)), pos_arr_(std::move(other.pos_arr_)), yaw_arr_(std::move(other.yaw_arr_)), k_arr_(std::move(other.k_arr_)), kd_tree_(std::move(other.kd_tree_))
{
    std::cout << "Path Move Constructor called" << std::endl;
    std::cout << path_name_ << " path Move\n";
}

// 복사 할당 연산자
Path &Path::operator=(const Path &other)
{
    if (this != &other)
    {
        path_name_ = other.path_name_;
        pos_arr_ = other.pos_arr_;
        yaw_arr_ = other.yaw_arr_;
        k_arr_ = other.k_arr_;
        kd_tree_ = other.kd_tree_;
    }
    std::cout << "Path Copy Assignment Operator called" << std::endl;
    return *this;
}

// 이동 할당 연산자
Path &Path::operator=(Path &&other) noexcept
{
    if (this != &other)
    {
        path_name_ = std::move(other.path_name_);
        pos_arr_ = std::move(other.pos_arr_);
        yaw_arr_ = std::move(other.yaw_arr_);
        k_arr_ = std::move(other.k_arr_);
        kd_tree_ = std::move(other.kd_tree_);
    }
    std::cout << "Path Move Assignment Operator called" << std::endl;
    return *this;
}

const std::string Path::getPathName() const
{
    return path_name_;
}

const std::vector<std::vector<double>> &Path::getRefPosArr() const
{
    return pos_arr_;
}

const std::vector<double> &Path::getRefYawArr() const
{
    return yaw_arr_;
}

const std::vector<double> &Path::getRefKArr() const
{
    return k_arr_;
}

const KDTree<std::vector<double>> &Path::getRefKDTree() const
{
    return kd_tree_;
}

void Path::setPosArr(const std::vector<std::vector<double>> &pos_arr)
{
    pos_arr_ = pos_arr;
}

void Path::setYawArr(const std::vector<double> &yaw_arr)
{
    yaw_arr_ = yaw_arr;
}

void Path::setKArr(const std::vector<double> &k_arr)
{
    k_arr_ = k_arr;
}

void Path::updateKDTree() // kd_tree 최신화 함수
{
    kd_tree_.build(pos_arr_);
}

void Path::updatePath(size_t start_index, size_t end_index, const Path &replace_path) // Path 일부분을 교체하는 함수(두 Path 크기 같을 때)
{
    const auto &replace_pos_arr = replace_path.getRefPosArr();
    const auto &replace_yaw_arr = replace_path.getRefYawArr();
    const auto &replace_k_arr = replace_path.getRefKArr();

    pos_arr_.erase(pos_arr_.begin() + start_index, pos_arr_.begin() + end_index);
    yaw_arr_.erase(yaw_arr_.begin() + start_index, yaw_arr_.begin() + end_index);
    k_arr_.erase(k_arr_.begin() + start_index, k_arr_.begin() + end_index);

    pos_arr_.insert(pos_arr_.begin() + start_index, replace_pos_arr.begin(), replace_pos_arr.end());
    yaw_arr_.insert(yaw_arr_.begin() + start_index, replace_yaw_arr.begin(), replace_yaw_arr.end());
    k_arr_.insert(k_arr_.begin() + start_index, replace_k_arr.begin(), replace_k_arr.end());

    updateKDTree();
}
void Path::updatePath(size_t start_index, size_t end_index, const Path &replace_path, size_t replace_start_index, size_t replace_end_index) // path 일부분을 교체하는 함수(두 Path 크기 다를 때)
{
    const auto &replace_pos_arr = replace_path.getRefPosArr();
    const auto &replace_yaw_arr = replace_path.getRefYawArr();
    const auto &replace_k_arr = replace_path.getRefKArr();

    // 교체 범위 삭제
    pos_arr_.erase(pos_arr_.begin() + start_index, pos_arr_.begin() + end_index);
    yaw_arr_.erase(yaw_arr_.begin() + start_index, yaw_arr_.begin() + end_index);
    k_arr_.erase(k_arr_.begin() + start_index, k_arr_.begin() + end_index);

    // 교체할 범위 추가
    pos_arr_.insert(pos_arr_.begin() + start_index, replace_pos_arr.begin() + replace_start_index, replace_pos_arr.begin() + replace_end_index);
    yaw_arr_.insert(yaw_arr_.begin() + start_index, replace_yaw_arr.begin() + replace_start_index, replace_yaw_arr.begin() + replace_end_index);
    k_arr_.insert(k_arr_.begin() + start_index, replace_k_arr.begin() + replace_start_index, replace_k_arr.begin() + replace_end_index);

    updateKDTree();
}

void Path::printAllData() const // test용 함수
{
    std::cout << path_name_ << " 번 파일 ###############" << std::endl;
    for (size_t i = 0; i < pos_arr_.size(); i++)
    {
        std::cout << pos_arr_[i][0] << " " << pos_arr_[i][1] << " " << yaw_arr_[i] << " " << k_arr_[i] << std::endl;
    }
}