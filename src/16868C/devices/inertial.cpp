#include "inertial.hpp"
#include <limits>

using namespace lib16868C;

Inertial::Inertial(uint port) : pros::Imu(port) {}

void Inertial::calibrate() {
	bool inertialResetFailed = true;
	do {
		if (!inertialResetFailed) {
			std::cerr << "Inertial Reset Failed" << std::endl;
			pros::lcd::print(0, "Inertial Reset Failed");
		}
		inertialResetFailed = reset(true) - 1;
	} while (inertialResetFailed);

	double h1 = get_rotation();
	pros::delay(500);
	double h2 = get_rotation();
	if (std::abs(h1 - h2) > DRIFT_THRESHOLD) {
		std::cerr << "Inertial Drift Detected: " << std::abs(h1 - h2) << " deg difference in 500ms" << std::endl;
		pros::lcd::print(0, "Inertial Drift Detected: %f deg difference in 500ms", std::abs(h1 - h2));
	}
}

double Inertial::get_rotation() const {
	double a = pros::Imu::get_rotation();
	if (std::isinf(a)) {
		std::cerr << "[Inertial] Rotation of Infinity" << std::endl;
		while (std::isinf(a)) a = pros::Imu::get_rotation();
	}

	return a;
}