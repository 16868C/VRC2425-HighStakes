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
		Pose(okapi::QLength x, okapi::QLength y);
		Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle theta);
		Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle theta, uint time);

		okapi::QLength distTo(Point p);
		okapi::QLength distTo(Pose p);
		okapi::QAngle angleTo(Point p);
		okapi::QAngle angleTo(Pose p);

		std::string toStr();

	private:
		Pose(Point pos, double theta, uint time);
		Pose(double x, double y, double theta, uint time);
};
} // namespace lib16868C