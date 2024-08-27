#pragma once
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "16868C/util/math.hpp"
#include <string>

namespace lib16868C {
class Pose : public Point {
	public:
		double theta;
		uint time;

		Pose();
		Pose(okapi::QLength x, okapi::QLength y);
		Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle theta);
		Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle theta, uint time);
		Pose(const Pose& pose);

		okapi::QLength distTo(Pose p);
		okapi::QAngle angleTo(Pose p);

		std::string toStr();

		Pose operator+(Point rhs);
		Pose operator+(Pose rhs);
		Pose operator-(Point rhs);
		Pose operator-(Pose rhs);
		Pose operator*(double rhs);
		Pose operator/(double rhs);

		Pose& operator=(const Pose& other);

	private:
		Pose(Point pos, double theta, uint time);
		Pose(double x, double y, double theta, uint time);
};
} // namespace lib16868C