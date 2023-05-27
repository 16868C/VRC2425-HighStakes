#pragma once
#include "16868Z/util/util.hpp"

namespace lib16868Z {
class Pose {
	public:
		double x, y, theta;
		uint time;

		inline double distTo(Pose point) {
			return hypot(point.x - x, point.y - y);
		}
		inline double angleTo(Pose point) {
			return atan2(point.y - y, point.x - x);
		}
		
		inline Pose operator+(Pose point) {
			return {x + point.x, y + point.y, theta + point.theta, time};
		}
		inline Pose operator-(Pose point) {
			return {x - point.x, y - point.y, theta - point.theta, time};
		}
		
		inline Pose operator*(double factor) {
			return {x * factor, y * factor, theta * factor, time};
		}
		inline Pose operator/(double divisor) {
			return {x / divisor, y / divisor, theta / divisor, time};
		}
};
} // namespace lib16868Z