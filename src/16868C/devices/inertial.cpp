#include "inertial.hpp"
#include "pros/llemu.hpp"
#include "16868C/util/logger.hpp"
#include "16868C/util/util.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "pros/misc.h"
#include <cmath>

using namespace lib16868C;

Inertial::Inertial(uint port) : pros::Imu(port) {}

void Inertial::calibrate() {
	uint st = pros::millis();
	int attempts = 0;
	do {
		reset(true);
		while (get_status() != pros::c::imu_status_e_t::E_IMU_STATUS_ERROR && is_calibrating()) pros::delay(10);

		if (!std::isinf(pros::Imu::get_rotation())) break;
		// std::cout << resetSuccess << "\n";
	} while (attempts++ <= 5);

	if (attempts > 5) {
		pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
		printError("[Inertial::calibrate] Inertial Reset Failed\n");
		pros::lcd::print(0, "Inertial Reset Failed");
		return;
	}

	double h1 = get_rotation(AngleUnit::DEG);
	pros::delay(500);
	double h2 = get_rotation(AngleUnit::DEG);
	if (std::abs(h1 - h2) > DRIFT_THRESHOLD) {
		// Inertial reading has changed too much in 500 ms
		pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
		printError("[Inertial] Inertial Drift Detected: %f deg difference in 500 ms\n", std::abs(h1 - h2));
		pros::lcd::print(0, "Inertial Drift Detected: %f deg difference in 500ms", std::abs(h1 - h2));
	}

	printDebug("[Inertial] Calibration time: %d ms\n", pros::millis() - st);
}

double Inertial::get_rotation(AngleUnit unit) {
	double a = pros::Imu::get_rotation();
	if (std::isinf(a)) {
		printError("[Inertial] Reading of Infinity\n");
		infDetected = true;
		while (std::isinf(a)) {
			a = pros::Imu::get_rotation();
			pros::delay(20);
		}
	}

	if (unit == AngleUnit::RAD) a = Util::degToRad(a);
	return -a;
}
void Inertial::set_rotation(okapi::QAngle heading) {
	pros::Imu::set_rotation(-heading.convert(okapi::degree));
}