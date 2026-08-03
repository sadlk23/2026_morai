/***
 * 작성자: 박용현
 * 설명: Median_filter 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef MEDIAN_FILTER_H_
#define MEDIAN_FILTER_H_

#include <deque>
#include <algorithm>

class MedianFilter
{
private:
    std::deque<double> window_;
    size_t window_size_;

public:
    // Constructor: set the window size for filtering
    MedianFilter(size_t size) : window_size_(size) {}

    // Apply median filter and return the filtered value
    double filter(double new_value)
    {
        window_.push_back(new_value);
        if (window_.size() > window_size_)
        {
            window_.pop_front(); // Remove the oldest value if window size is exceeded
        }

        // Sort the window and return the median value
        std::deque<double> sorted_window = window_;
        std::sort(sorted_window.begin(), sorted_window.end());
        return sorted_window[sorted_window.size() / 2];
    }
};

#endif
