#include "main.h"
#include "16868C/devices/inertial.hpp"
#include "16868C/util/pose.hpp"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"
#include <type_traits>

using namespace lib16868C;

void initialize() {
	pros::lcd::initialize();

	odometry.init();
	// odometry.init(Pose(0_in, 0_in, 0_deg));
	// inertial.reset(true);
	// chassis.coast();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	uint st = pros::millis();

	printDebug("Auton took %d ms\n", pros::millis() - st);
}

void opcontrol() {
	// odometry.update(true, true, true, true);
	// std::cout << odometry.getPose().toStr() << "\n";
	chassis.moveToPoint({48_in, -24_in}, 600_rpm, {0.02, 0, 0.1}, {1, 0, 0.1}, 5_in, false, 0);
	chassis.moveToPoint({65_in, -72_in}, 600_rpm, {0.02, 0, 0.1}, {1, 0, 0.1}, 5_in, false, 0);
	chassis.moveToPoint({48_in, -80_in}, 600_rpm, {0.02, 0, 0.1}, {1, 0, 0.1}, 5_in, true, 0);
	chassis.moveToPoint({0_in, -24_in}, 600_rpm, {0.02, 0, 0.1}, {1, 0, 0.1}, 3_in, 0);

	while (true) {
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		chassis.driveTank(left, right);

		pros::delay(50);
	}
}