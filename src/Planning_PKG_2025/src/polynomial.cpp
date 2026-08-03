#include <polynomial.hpp>

// 5차 다항식 생성자 (Quintic Polynomial Constructor)
// 초기 상태와 목표 상태를 바탕으로 5차 다항식의 계수를 계산
Polynomial::Polynomial(double x0, double v0, double a0, double xT, double vT, double aT, double T) : type_("quintic"), x0_(x0), v0_(v0), a0_(a0), xT_(xT), vT_(vT), aT_(aT), T_(T), a1_(x0), a2_(v0), a3_(a0 / 2)
{
	Eigen::Matrix3d A;
	Eigen::Vector3d B;

	// 5차 다항식을 위한 행렬 A와 벡터 B를 정의 (Define matrices A and B for solving the polynomial coefficients)
	A << std::pow(T_, 3), std::pow(T_, 4), std::pow(T_, 5),
		3 * std::pow(T_, 2), 4 * std::pow(T_, 3), 5 * std::pow(T_, 4),
		6 * T_, 12 * std::pow(T_, 2), 20 * std::pow(T_, 3);

	B << xT - a1_ - a2_ * T_ - a3_ * std::pow(T_, 2),
		vT - a2_ - 2 * a3_ * T_,
		aT - 2 * a3_;

	// Ax=B 형태의 방정식을 풀어 다항식 계수를 계산 (Solve for the coefficients in Ax=B)
	Eigen::Vector3d coefficients = A.colPivHouseholderQr().solve(B);
	a4_ = coefficients(0);
	a5_ = coefficients(1);
	a6_ = coefficients(2);
}

// 4차 다항식 생성자 (Quartic Polynomial Constructor)
// 초기 상태와 목표 속도 및 가속도를 바탕으로 4차 다항식의 계수를 계산
Polynomial::Polynomial(double x0, double v0, double a0, double vT, double aT, double T) : type_("quartic"), x0_(x0), v0_(v0), a0_(a0), vT_(vT), aT_(aT), T_(T), a1_(x0), a2_(v0), a3_(a0 / 2)
{
	Eigen::Matrix2d A;
	Eigen::Vector2d B;

	// 4차 다항식을 위한 행렬 A와 벡터 B를 정의 (Define matrices A and B for solving the polynomial coefficients)
	A << 3 * std::pow(T_, 2), 4 * std::pow(T_, 3),
		6 * T_, 12 * std::pow(T_, 2);

	B << vT - a2_ - 2 * a3_ * T_,
		aT - 2 * a3_;

	// Ax=B 형태의 방정식을 풀어 다항식 계수를 계산 (Solve for the coefficients in Ax=B)
	Eigen::Vector2d coefficients = A.colPivHouseholderQr().solve(B);
	a4_ = coefficients(0);
	a5_ = coefficients(1);
}

// 소멸자 (Destructor)
Polynomial::~Polynomial() {}

// 특정 시간 t에서 위치를 계산 (Calculates the position at time t)
double Polynomial::position(double t)
{
	if (type_ == "quintic")
	{
		return a1_ + a2_ * t + a3_ * std::pow(t, 2) + a4_ * std::pow(t, 3) + a5_ * std::pow(t, 4) + a6_ * std::pow(t, 5);
	}
	return a1_ + a2_ * t + a3_ * std::pow(t, 2) + a4_ * std::pow(t, 3) + a5_ * std::pow(t, 4);
}

// 특정 시간 t에서 속도를 계산 (Calculates the velocity at time t)
double Polynomial::velocity(double t)
{
	if (type_ == "quintic")
	{
		return a2_ + 2 * a3_ * t + 3 * a4_ * std::pow(t, 2) + 4 * a5_ * std::pow(t, 3) + 5 * a6_ * std::pow(t, 4);
	}
	return a2_ + 2 * a3_ * t + 3 * a4_ * std::pow(t, 2) + 4 * a5_ * std::pow(t, 3);
}

// 특정 시간 t에서 가속도를 계산 (Calculates the acceleration at time t)
double Polynomial::acceleration(double t)
{
	if (type_ == "quintic")
	{
		return 2 * a3_ + 6 * a4_ * t + 12 * a5_ * std::pow(t, 2) + 20 * a6_ * std::pow(t, 3);
	}
	return 2 * a3_ + 6 * a4_ * t + 12 * a5_ * std::pow(t, 2);
}

// 특정 시간 t에서 jerk를 계산 (Calculates the jerk at time t)
double Polynomial::jerk(double t)
{
	if (type_ == "quintic")
	{
		return 6 * a4_ + 24 * a5_ * t + 60 * a6_ * std::pow(t, 2);
	}
	return 6 * a4_ + 24 * a5_ * t;
}
