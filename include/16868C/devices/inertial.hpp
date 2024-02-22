#pragma once
#include "okapi/api.hpp"
#include "16868C/util/math.hpp"

namespace lib16868C {
enum class AngleUnit {
	DEG,
	RAD
};

class Inertial : public pros::Imu {
	public:
		Inertial(uint port);

		void calibrate();
		double get_rotation(AngleUnit unit) const;
		void set_rotation(okapi::QAngle heading);
	
	private:
		double DRIFT_THRESHOLD = 1;
};
}