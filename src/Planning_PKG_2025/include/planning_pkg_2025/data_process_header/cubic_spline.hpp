/***
 * 작성자: 조윤서
 * 설명: Cubic Spline 관련 클래스 헤더파일
 * Style
 * -클래스: PascalCase
 * -멤버변수: snake_case_   (snake_case 뒤에 밑줄)(파이썬 사용 x)
 * -함수: camelCase
 * -변수: snake_case
 * -상수: kPascalCase   (PascalCase앞에 k 접두어)
 ***/

#ifndef CUBICSPLINE_H_
#define CUBICSPLINE_H_

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <algorithm>

using namespace std;

class CubicSpline1D
{
public:
    CubicSpline1D(const vector<double> &x, const vector<double> &y)
    {
        if (x.size() != y.size())
        {
            throw invalid_argument("number of x, y is not matched!");
        }
        nx = x.size();
        this->x = x;
        this->y = y;
        this->h.resize(nx - 1);

        for (int i = 0; i < nx - 1; i++)
        {
            h[i] = x[i + 1] - x[i];
            if (h[i] < 0)
            {
                throw invalid_argument("x coordinate is not sotred");
            }
        }
        this->a = y;

        Eigen::MatrixXd A = calc_A(h);
        Eigen::MatrixXd B = calc_B(h, a);
        Eigen::VectorXd c_eigen = A.colPivHouseholderQr().solve(B);
        this->c.resize(nx);

        for (int i = 0; i < nx; i++)
        {
            c[i] = c_eigen(i);
        }

        this->b.resize(nx - 1);
        this->d.resize(nx - 1);
        for (int i = 0; i < nx - 1; i++)
        {
            d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);
            b[i] = 1.0 / h[i] * (a[i + 1] - a[i]) - h[i] / 3.0 * (2.0 * c[i] + c[i + 1]);
        }
    }
    double calc_position(double x_val)
    {
        if (x_val < this->x[0] || x_val > this->x[nx - 1])
        {
            return nan("");
        }
        int i = search_index(x_val);
        double dx = x_val - x[i];
        double position = a[i] + (b[i] * dx) + (c[i] * dx * dx) + (d[i] * dx * dx * dx);

        return position;
    }

    double calc_first_derivative(double x_val)
    {
        if (x_val < x[0] || x_val > x[nx - 1])
        {
            return nan("");
        }
        int i = search_index(x_val);
        double dx = x_val - x[i];
        double dy = b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * dx * dx;

        return dy;
    }

    double calc_second_derivative(double x_val)
    {
        if (x_val < x[0] || x_val > x[nx - 1])
        {
            return nan("");
        }

        int i = search_index(x_val);
        double dx = x_val - x[i];
        double ddy = 2.0 * c[i] + 6.0 * d[i] * dx;

        return ddy;
    }

private:
    vector<double> x, y, a, b, c, d, h;
    int nx;

    int search_index(double x_val)
    {
        auto it = lower_bound(x.begin(), x.end(), x_val);
        int index = max(int(it - x.begin()) - 1, 0);
        return index;
    }

    Eigen::MatrixXd calc_A(const vector<double> &h)
    {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx, nx);
        A(0, 0) = 1.0;
        for (int i = 0; i < nx - 1; i++)
        {
            if (i != nx - 2)
            {
                A(i + 1, i + 1) = 2.0 * (h[i] + h[i + 1]);
            }
            A(i + 1, i) = h[i];
            A(i, i + 1) = h[i];
        }
        A(0, 1) = 0.0;
        A(nx - 1, nx - 2) = 0.0;
        A(nx - 1, nx - 1) = 1.0;
        return A;
    }

    Eigen::VectorXd calc_B(const vector<double> &h, const vector<double> &a)
    {
        Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);
        for (int i = 0; i < nx - 2; i++)
        {
            B(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];
        }
        return B;
    }
};

class CubicSpline2D
{
private:
    vector<double> s;
    CubicSpline1D sx;
    CubicSpline1D sy;

public:
    CubicSpline2D(const vector<double> &x, const vector<double> &y) : s(calc_s(x, y)), sx(s, x), sy(s, y) {}

    vector<double> calc_s(const vector<double> &x, const vector<double> &y)
    {
        vector<double> s{0.0};
        for (size_t i = 1; i < x.size(); ++i)
        {
            double dx = x[i] - x[i - 1];
            double dy = y[i] - y[i - 1];
            s.push_back(s.back() + hypot(dx, dy));
        }
        return s;
    }

    pair<double, double> calc_position(double s_val)
    {
        double x = sx.calc_position(s_val);
        double y = sy.calc_position(s_val);
        return make_pair(x, y);
    }

    double calc_curvature(double s_val)
    {
        double dx = sx.calc_first_derivative(s_val);
        double ddx = sx.calc_second_derivative(s_val);
        double dy = sy.calc_first_derivative(s_val);
        double ddy = sy.calc_second_derivative(s_val);
        return (ddy * dx - ddx * dy) / pow(dx * dx + dy * dy, 1.5);
    }

    double calc_yaw(double s_val)
    {
        double dx = sx.calc_first_derivative(s_val);
        double dy = sy.calc_first_derivative(s_val);
        return atan2(dy, dx);
    }
};

#endif