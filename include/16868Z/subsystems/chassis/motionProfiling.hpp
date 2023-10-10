#pragma once
#include "16868Z/util/util.hpp"
#include <vector>

namespace lib16868C {
struct MotionLimit {
	double maxVel;
	double maxAccel;

	inline MotionLimit operator*(double factor) {
		return {maxVel * factor, maxAccel * factor};
	}
	inline MotionLimit operator/(double divisor) {
		return {maxVel / divisor, maxAccel / divisor};
	}
};

struct MotionData {
	double time;
	double distance;
	double velocity;
	double acceleration;
};

struct MotionProfile {
	std::vector<MotionData> profile;
	double accelTime;

	inline MotionData operator[](int i) {
		return profile[i];
	}

	inline MotionProfile operator+=(const MotionProfile& other) {
		profile.insert(profile.end(), other.profile.begin(), other.profile.end());
		return *this;
	}
};

class MotionProfiling {
	public:
		static MotionProfile generateAccel(double dist, MotionLimit motionLimit);
		static MotionProfile generateTrapezoidal(double dist, MotionLimit motionLimit);
};
} // namespace lib16868C