#pragma once
#include "api.h"
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
	
	private:
		double DRIFT_THRESHOLD = 1;
};
}