/***
 * 작성자: 성재현
 * 설명: 미션 데이터 관련 클래스 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef MISSION_DATA_H_
#define MISSION_DATA_H_

#include <vector>

class MissionData
{
private:
    // 정지 포인트
    std::vector<double> stop_point_;
    double stop_distance_;

public:
    /* getter 함수 */

    // 정지 포인트
    const std::vector<double> &getRefStopPoint() const
    {
        return stop_point_;
    }

    const double getStopDistance() const
    {
        return stop_distance_;
    }

    /* setter 함수 */

    // 정지 포인트
    void setStopPoint(std::vector<double> stop_point)
    {
        stop_point_.clear();

        for (double d : stop_point)
        {
            stop_point_.push_back(d);
        }
    }

    void setStopDistance(double distance)
    {
        stop_distance_ = distance;
    }

    /* clear 함수 */

    // 정지포인트
    void clearStopPoint()
    {
        stop_point_.clear();
    }
};

#endif