#include "inertial.hpp"
#include "16868C/util/logger.hpp"
#include "16868C/util/util.hpp"
#include <limits>

using namespace lib16868C;

Inertial::Inertial(uint port) : pros::Imu(port) {}

void Inertial::calibrate() {
	uint st = pros::millis();
	bool resetSuccess = true;
	do {
		if (!resetSuccess) {
			printError("[Inertial] Inertial Reset Failed\n");
			pros::lcd::print(0, "Inertial Reset Failed");
		}
		resetSuccess = reset(true) - 1;
	} while (!resetSuccess);

	double h1 = get_rotation(AngleUnit::DEG);
	pros::delay(500);
	double h2 = get_rotation(AngleUnit::DEG);
	if (std::abs(h1 - h2) > DRIFT_THRESHOLD) {
		printError("[Inertial] Inertial Drift Detected: %.4f deg difference in 500 ms\n", std::abs(h1 - h2));
		pros::lcd::print(0, "Inertial Drift Detected: %f deg difference in 500ms", std::abs(h1 - h2));
	}
	printDebug("[Inertial] Calibration time: %d ms\n", pros::millis() - st);
}

double Inertial::get_rotation(AngleUnit unit) const {
	double a = pros::Imu::get_rotation();
	if (std::isinf(a)) {
		printError("[Inertial] Rotation of Infinity\n");
		while (std::isinf(a)) a = pros::Imu::get_rotation();
	}

	if (unit == AngleUnit::RAD) a = Util::degToRad(a);
	return a;
}