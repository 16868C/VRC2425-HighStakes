#pragma once
#include "okapi/api.hpp"
#include "16868C/util/math.hpp"
#include <string>

namespace lib16868C {
class Pose {
	public:
		double& x = _pos.x;
		double& y = _pos.y;
		double& theta = _theta;
		uint& time = _time;

		Pose();
		Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle theta, uint time);
		Pose(const Pose& p);

		okapi::QLength distTo(Point p);
		okapi::QLength distTo(Pose p);
		okapi::QAngle angleTo(Point p);
		okapi::QAngle angleTo(Pose p);

		std::string toStr();

		Pose& operator=(const Pose& p);

	private:
		Point _pos;
		double _theta;
		uint _time;
		
		Pose(Point pos, double theta, uint time);
		Pose(double x, double y, double theta, uint time);
};
} // namespace lib16868C