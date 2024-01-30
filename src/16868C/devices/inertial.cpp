#include "inertial.hpp"
#include "16868C/util/util.hpp"
#include <limits>

using namespace lib16868C;

Inertial::Inertial(uint port) : pros::Imu(port) {}

void Inertial::calibrate() {
	uint t = pros::millis();
	bool resetSuccess = true;
	do {
		if (!resetSuccess) {
			std::cerr << "Inertial Reset Failed" << std::endl;
			pros::lcd::print(0, "Inertial Reset Failed");
		}
		resetSuccess = reset(true) - 1;
	} while (!resetSuccess);

	double h1 = get_rotation(AngleUnit::DEG);
	pros::delay(500);
	double h2 = get_rotation(AngleUnit::DEG);
	if (std::abs(h1 - h2) > DRIFT_THRESHOLD) {
		std::cerr << "Inertial Drift Detected: " << std::abs(h1 - h2) << " deg difference in 500ms" << std::endl;
		pros::lcd::print(0, "Inertial Drift Detected: %f deg difference in 500ms", std::abs(h1 - h2));
	}
	std::cout << "Calibration time: " << pros::millis() - t << "\n";
}

double Inertial::get_rotation(AngleUnit unit) const {
	double a = pros::Imu::get_rotation();
	if (std::isinf(a)) {
		std::cerr << "[Inertial] Rotation of Infinity" << std::endl;
		while (std::isinf(a)) a = pros::Imu::get_rotation();
	}

	if (unit == AngleUnit::RAD) a = Util::degToRad(a);
	return a;
}