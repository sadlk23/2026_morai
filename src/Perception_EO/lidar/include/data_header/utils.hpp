#ifndef UTILS_HPP
#define UTILS_HPP

#include <cmath>
#include <vector>
#define X first
#define Y second
#define PI 3.141592653589793238462643383279

// erp를 포인트 클라우드에서 지우기 위해 필요한 것
struct erpPoint { float x, y; };

class MyPoint
{
public:
  float X;
  float Y;
  MyPoint(float x, float y)
  {
    X = x;
    Y = y;
  }
  void setCoord(float x, float y)
  {
    X = x;
    Y = y;
  }
};

// 반시계 방향일 때, 양수 a -> b-> obs

//      <-
//       --
//        --
//          --
//           --
//------------->
//
int ccw(const MyPoint &a, const MyPoint &b, const MyPoint &obs)
{
  float vec = (b.X - a.X) * (obs.Y - a.Y) - (b.Y - a.Y) * (obs.X - a.X);
  if (vec < 0)
    return -1;
  else if (vec > 0)
    return 1;
  else
    return 0;
}

float distance_zero(std::pair<float, float> p1)
{
  return sqrt(pow(p1.X, 2) + pow(p1.Y, 2));
}

bool compare(std::pair<float, float> p1, std::pair<float, float> p2)
{ // sort 함수 커스텀
  if (distance_zero(p1) < distance_zero(p2))
  {
    // 길이가 짧은 경우 1순위로 정렬
    return 1;
  }
  else if (distance_zero(p1) < distance_zero(p2))
  {
    return 0;
  }
  else
  {
    // 길이가 같은 경우
    return 0;
  }
}


double yaw = 90 * (PI / 180);
double delta = 0.f;


class VehicleState
{
private:
  double cx;
  double cy;

public:
  VehicleState(double cx_, double cy_)
  {
    cx = cx_;
    cy = cy_;
  }
  void Calculate()
  {
    double phi = 0.f;
    double deltaMax = 28 * (PI / 180);

    if (cy < 0)
      phi = atan2(cx, -cy);
    else if (cy > 0)
      phi = PI - atan2(cx, cy);
    delta = phi - yaw;
    if (delta > deltaMax)
      delta = deltaMax;
    else if (delta < -deltaMax)
      delta = -deltaMax;
  }
};

#endif // UTILS_HPP