#pragma once
#include "okapi/api.hpp"
#include "16868C/util/util.hpp"

namespace lib16868C {
class Pose {
	public:
		okapi::QLength x, y;
		okapi::QAngle theta;
		uint time;

		inline okapi::QLength distTo(Pose point) {
			return hypot(point.x - x, point.y - y);
		}
		inline okapi::QAngle angleTo(Pose point) {
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

		inline void print() {
			std::cout << "t: " << time << " x: " << x.convert(okapi::inch) << " y: " << y.convert(okapi::inch) << " theta: " << theta.convert(okapi::degree) << "\n";
		}
};
} // namespace lib16868C