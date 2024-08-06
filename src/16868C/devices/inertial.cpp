#include "inertial.hpp"
#include "pros/llemu.hpp"
#include "16868C/util/logger.hpp"
#include "16868C/util/util.hpp"
#include "okapi/api/units/QAngle.hpp"
#include <cmath>

using namespace lib16868C;

Inertial::Inertial(uint port) : pros::Imu(port) {}
Inertial::Inertial(uint port, okapi::Controller* controller) : pros::Imu(port), controller(controller) {}

void Inertial::calibrate() {
	uint st = pros::millis();
	bool resetSuccess = true;
	do {
		if (!resetSuccess) {
			// Inertial failed to reset
			printError("[Inertial::calibrate] Inertial Reset Failed\n");
			pros::lcd::print(0, "Inertial Reset Failed");
		}
		reset(true);
		while (is_calibrating()) pros::delay(10);
		// std::cout << resetSuccess << "\n";
	} while (!resetSuccess);

	double h1 = get_rotation(AngleUnit::DEG);
	pros::delay(500);
	double h2 = get_rotation(AngleUnit::DEG);
	if (std::abs(h1 - h2) > DRIFT_THRESHOLD) {
		// Inertial reading has changed too much in 500 ms
		printError("[Inertial] Inertial Drift Detected: %f deg difference in 500 ms\n", std::abs(h1 - h2));
		pros::lcd::print(0, "Inertial Drift Detected: %f deg difference in 500ms", std::abs(h1 - h2));
		if (controller) controller->rumble("--------");
	}
	if (std::isinf(pros::Imu::get_rotation())) calibrate();

	printDebug("[Inertial] Calibration time: %d ms\n", pros::millis() - st);

	if (pros::millis() - st < 1950) {
		if (controller) controller->rumble("--------");
		calibrate();
	}
	if (infDetected) {
		infDetected = false;
		if (controller) controller->rumble("--------");
		calibrate();
	}
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