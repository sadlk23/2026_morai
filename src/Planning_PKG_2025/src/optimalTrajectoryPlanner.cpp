#include "optimalTrajectoryPlanner.hpp"

// OptimalTrajectoryPlanner 기본 생성자
OptimalTrajectoryPlanner::OptimalTrajectoryPlanner()
{
	// 최소 회전 반경과 최대 곡률 초기화 (초기화는 필요에 따라 주석 처리됨)
	// minimumTurningRadius_ = wheelBase_/tan(maxSteeringAngle_)
	// maxCurvature_ = 1/minimumTurningRadius_;
}

// OptimalTrajectoryPlanner 소멸자
OptimalTrajectoryPlanner::~OptimalTrajectoryPlanner() {}

// 최적의 경로를 생성하는 함수
// 초기 횡방향 상태(d0, dv0, da0)와 초기 종방향 상태(s0, sv0), 차선 중심(centerLane), 장애물(obstacles)을 사용
FrenetPath OptimalTrajectoryPlanner::optimalTrajectory(double d0, double dv0, double da0,
													   double s0, double sv0,
													   std::vector<std::vector<double>> &centerLane,
													   std::vector<std::vector<double>> &obstacles,
													   std::vector<FrenetPath> &allPaths)
{
	// TNB/Frenet 좌표계에서 모든 가능한 경로를 저장할 벡터
	std::vector<FrenetPath> paths;

	// TNB/Frenet 좌표계에서 모든 가능한 경로 계산
	// 다양한 예측 시간에 대해 반복
	for (double T = minPredictionStep_; T < maxPredictionStep_; T = T + 0.2)
	{
		// 다양한 차선에 대해 반복
		for (double dT = -((noOfLanes_ - 1) * laneWidth_) / 2; dT <= ((noOfLanes_ - 1) * laneWidth_) / 2; dT = dT + laneWidth_)
		{
			double dvT = 0, daT = 0;
			std::vector<std::vector<double>> latitudionalTrajectory;
			Polynomial quintic(d0, dv0, da0, dT, dvT, daT, T);
			double jd = 0;
			// 주어진 T와 차선에 대해 횡방향 궤적 생성
			for (double t = 0; t <= T; t = t + 0.1)
			{
				std::vector<double> data = {quintic.position(t), quintic.velocity(t), quintic.acceleration(t), quintic.jerk(t), t};
				jd += std::pow(data[3], 2);
				latitudionalTrajectory.push_back(data);
			}

			// 다양한 종료 속도에 대해 반복
			// for (double svT = targetVelocity_ - velocityStep_; svT <= targetVelocity_ + velocityStep_; svT = svT + velocityStep_)
			// {
			double svT = targetVelocity_;
			FrenetPath path;
			path.T = T;
			path.d = latitudionalTrajectory;
			path.jd = jd;
			std::vector<std::vector<double>> longitudionalTrajectory;
			Polynomial quartic(s0, sv0, 0, svT, 0, T);
			double js = 0;
			// 주어진 v와 횡방향 궤적에 대해 종방향 궤적 생성
			for (double t = 0; t <= T; t = t + 0.1)
			{
				std::vector<double> data = {quartic.position(t), quartic.velocity(t), quartic.acceleration(t), quartic.jerk(t), t};
				js += std::pow(data[3], 2);
				if (data[1] > path.maxVelocity)
					path.maxVelocity = data[1];
				if (data[2] > path.maxAcceleration)
					path.maxAcceleration = data[2];
				longitudionalTrajectory.push_back(data);
			}
			path.s = longitudionalTrajectory;
			path.js = js;

			// 궤적의 비용 계산
			trajectoryCost(path);

			// Frenet 경로 저장
			paths.push_back(path);
			// }
		}
	}

	// Frenet 좌표계에서 세계/전역 좌표계로 경로 변환
	convertToWorldFrame(paths, centerLane);

	// 운동학적 제약 조건과 충돌을 기반으로 경로가 유효한지 확인
	std::vector<FrenetPath> validPaths;
	validPaths = isValid(paths, obstacles);
	allPaths = paths;

	// 모든 유효한 경로 중 비용을 기준으로 최적의 경로 찾기
	FrenetPath optimalTrajectory;
	double cost = INT_MAX;
	for (FrenetPath &path : validPaths)
	{
		if (cost >= path.cf)
		{
			cost = path.cf;
			optimalTrajectory = path;
		}
	}

	return optimalTrajectory;
}

// 궤적의 비용을 계산하는 함수
void OptimalTrajectoryPlanner::trajectoryCost(FrenetPath &path)
{
	double cd = (path.jd * kjd_ + path.T * ktd_ + std::pow(path.d.back()[0], 2) * ksd_) / (kjd_ + ktd_ + ksd_);
	double cv = (path.js * kjs_ + path.T * kts_ + std::pow(path.s.front()[0] - path.s.back()[0], 2) * kss_) / (kjs_ + kts_ + kss_);
	path.cf = (klat_ * cd + klon_ * cv) / (klat_ + klon_);
}

// 주어진 경로와 장애물 간의 충돌 여부를 확인하는 함수
bool OptimalTrajectoryPlanner::isColliding(FrenetPath &path, std::vector<std::vector<double>> &obstacles)
{
	for (int i = 0; i < path.world.size(); i++)
	{
		for (int j = 0; j < obstacles.size(); j++)
		{
			if (std::sqrt(std::pow(path.world[i][0] - obstacles[j][0], 2) + std::pow(path.world[i][1] - obstacles[j][1], 2)) <= safe_distance_)
				return true;
		}
	}
	return false;
}

// 경로가 운동학적 제약 조건을 만족하는지 확인하는 함수
bool OptimalTrajectoryPlanner::isWithinKinematicConstraints(FrenetPath &path)
{
	if (path.maxVelocity > maxVelocity_ || path.maxAcceleration > maxAcceleration_ || path.maxCurvature > maxCurvature_)
	{
		return false;
	}
	return true;
}

// 주어진 경로들이 유효한지 확인하는 함수 (장애물 포함)
std::vector<FrenetPath> OptimalTrajectoryPlanner::isValid(std::vector<FrenetPath> &paths, std::vector<std::vector<double>> &obstacles)
{
	std::vector<FrenetPath> validPaths;
	for (FrenetPath &path : paths)
	{
		if (!isColliding(path, obstacles) && isWithinKinematicConstraints(path))
		{
			validPaths.push_back(path);
		}
	}
	return validPaths;
}

// Frenet 경로를 세계 좌표계로 변환하는 함수
void OptimalTrajectoryPlanner::convertToWorldFrame(std::vector<FrenetPath> &paths, std::vector<std::vector<double>> &centerLane)
{
	for (FrenetPath &path : paths)
	{
		// 세계 좌표계에서 x, y 계산
		int j = 0;
		for (int i = 0; i < path.s.size(); i++)
		{
			double x, y, yaw;
			for (; j < centerLane.size(); j++)
			{
				if (std::abs(path.s[i][0] - centerLane[j][4]) <= 0.1)
				{
					x = centerLane[j][0];
					y = centerLane[j][1];
					yaw = centerLane[j][2];
					break;
				}
			}
			double d = path.d[i][0];
			path.world.push_back({x + d * std::cos(yaw + pi_ / 2), y + d * std::sin(yaw + pi_ / 2), 0, 0});
		}

		// 세계 좌표계에서 Yaw 계산
		for (int i = 0; i < path.world.size() - 1; i++)
		{
			path.world[i][2] = std::atan2((path.world[i + 1][1] - path.world[i][1]), (path.world[i + 1][0] - path.world[i][0]));
			path.world[i][3] = std::sqrt(std::pow(path.world[i + 1][0] - path.world[i][0], 2) + std::pow(path.world[i + 1][1] - path.world[i][1], 2));
		}
		path.world[path.world.size() - 1][2] = path.world[path.world.size() - 2][2];
		path.world[path.world.size() - 1][3] = path.world[path.world.size() - 2][3];

		// 궤적의 최대 곡률 계산
		double curvature = INT_MIN;
		for (int i = 0; i < path.world.size() - 1; i++)
		{
			double tempCurvature = abs((path.world[i + 1][2] - path.world[i][2]) / (path.world[i][3]));
			if (curvature < tempCurvature)
				curvature = tempCurvature;
		}
		path.maxCurvature = curvature;
	}
}

// 회전을 계산하는 함수
Eigen::Vector2d OptimalTrajectoryPlanner::rotation(double theta, double x, double y)
{
	Eigen::Matrix2d rotationZ;
	rotationZ << std::cos(theta), -1 * std::sin(theta),
		std::sin(theta), std::cos(theta);
	Eigen::Vector2d point;
	point << x, y;
	return rotationZ * point;
}

// Frenet 변환 (위치)
std::vector<double> OptimalTrajectoryPlanner::convertToFrenet(double x, double y, const std::vector<std::vector<double>> &centerLane)
{
	double closestDist = std::numeric_limits<double>::max();
	int closestIdx = 0;

	// 가장 가까운 중심선의 점 찾기
	for (int i = 0; i < centerLane.size(); ++i)
	{
		double dist = std::sqrt(std::pow(x - centerLane[i][0], 2) + std::pow(y - centerLane[i][1], 2));
		if (dist < closestDist)
		{
			closestDist = dist;
			closestIdx = i;
		}
	}

	// 종방향 거리(s) 계산
	double s = centerLane[closestIdx][4];

	// 횡방향 거리(d) 계산
	double dx = x - centerLane[closestIdx][0];
	double dy = y - centerLane[closestIdx][1];
	double yaw = centerLane[closestIdx][2];
	double d = dx * std::cos(yaw + pi_ / 2) + dy * std::sin(yaw + pi_ / 2);

	// 종, 횡
	return {s, d};
}

// Frenet 변환 함수 (속도)
std::vector<double> OptimalTrajectoryPlanner::convertVelocityToFrenet(double x, double y, double speed, double yaw, const std::vector<std::vector<double>> &centerLane)
{
	// 가장 가까운 중심선의 점 찾기
	double closestDist = std::numeric_limits<double>::max();
	int closestIdx = 0;

	for (int i = 0; i < centerLane.size(); ++i)
	{
		double dist = std::sqrt(std::pow(x - centerLane[i][0], 2) + std::pow(y - centerLane[i][1], 2));
		if (dist < closestDist)
		{
			closestDist = dist;
			closestIdx = i;
		}
	}

	// 중심선의 yaw 값을 사용하여 Frenet 프레임의 tangent와 normal 벡터 계산
	double centerYaw = centerLane[closestIdx][2];
	double tangent_x = std::cos(centerYaw);
	double tangent_y = std::sin(centerYaw);
	double normal_x = -std::sin(centerYaw);
	double normal_y = std::cos(centerYaw);

	// 속도 변환
	double v_x = speed * std::cos(yaw);
	double v_y = speed * std::sin(yaw);

	// 세계 좌표계에서 Frenet 좌표계로 속도 변환
	double v_t = v_x * tangent_x + v_y * tangent_y;
	double v_n = v_x * normal_x + v_y * normal_y;

	// 종, 횡
	return {v_t, v_n};
}

// Frenet 변환(가속도)
std::vector<double> OptimalTrajectoryPlanner::convertAccelerationToFrenet(double x, double y, double ax, double ay, const std::vector<std::vector<double>> &centerLane)
{
	// 가장 가까운 중심선의 점 찾기
	double closestDist = std::numeric_limits<double>::max();
	int closestIdx = 0;

	for (int i = 0; i < centerLane.size(); ++i)
	{
		double dist = std::sqrt(std::pow(x - centerLane[i][0], 2) + std::pow(y - centerLane[i][1], 2));
		if (dist < closestDist)
		{
			closestDist = dist;
			closestIdx = i;
		}
	}

	// 중심선의 yaw 값을 사용하여 Frenet 프레임의 tangent와 normal 벡터 계산
	double yaw = centerLane[closestIdx][2];
	double tangent_x = std::cos(yaw);
	double tangent_y = std::sin(yaw);
	double normal_x = -std::sin(yaw);
	double normal_y = std::cos(yaw);

	// 세계 좌표계에서 Frenet 좌표계로 가속도 변환
	double a_t = ax * tangent_x + ay * tangent_y;
	double a_n = ax * normal_x + ay * normal_y;

	// 종, 횡
	return {a_t, a_n};
}