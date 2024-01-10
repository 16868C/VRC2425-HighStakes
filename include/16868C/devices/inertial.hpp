#pragma once
#include "api.h"
#include "16868C/util/math.hpp"

namespace lib16868C {
class Inertial : public pros::Imu {
	public:
		Inertial(uint port);

		void calibrate();
		double get_rotation() const override;
	
	private:
		double DRIFT_THRESHOLD = 1;
};
}