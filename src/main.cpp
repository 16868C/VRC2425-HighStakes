#include "main.h"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"

using namespace lib16868C;

void initialize() {
	pros::lcd::initialize();

	odometry.init();
	// inertial.reset(true);
	chassis.coast();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	uint st = pros::millis();

	printDebug("Auton took %d ms\n", pros::millis() - st);
}

void opcontrol() {
	chassis.moveToPoint({24_in, 24_in}, 200_rpm, {0.1, 0, 0.1}, {1, 0, 0.1}, 3_in, false, true, 0);

	while (true) {
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		chassis.driveTank(left, right);

		pros::delay(50);
	}
}