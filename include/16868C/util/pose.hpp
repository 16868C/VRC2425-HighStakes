#pragma once
#include "okapi/api.hpp"
#include "16868C/util/math.hpp"
#include <string>

namespace lib16868C {
class Pose {
	public:
		Point pos;
		double theta;
		uint time;

		Pose();
		Pose(Point pos, double theta, uint time);
		Pose(double x, double y, double theta, uint time);
		Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle theta, uint time);
		Pose(Pose& p);

		okapi::QLength distTo(Point p);
		okapi::QLength distTo(Pose p);
		okapi::QAngle angleTo(Point p);
		okapi::QAngle angleTo(Pose p);

		std::string toStr();
};
} // namespace lib16868C