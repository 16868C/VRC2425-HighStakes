#include "main.h"
#include "16868C/util/pose.hpp"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"
#include <type_traits>

using namespace lib16868C;

void initialize() {
	pros::lcd::initialize();

	uint st = pros::millis();
	inertial.reset(true);
	while (inertial.is_calibrating()) {
		pros::delay(10);
		std::cout << "waiting\n";
	}
	std::cout << pros::millis() - st << "\n";
	odometry.init();
	// odometry.init(Pose(0_in, 0_in, 0_deg));
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
	// chassis.moveToPoint({24_in, 48_in}, 100_rpm, {0.1, 0, 0.1}, {1, 0, 0.1}, 3_in, false, true, 0);

	while (true) {
		// printDebug("%f\n", inertial.get_rotation(AngleUnit::DEG));
		pros::delay(50);
	}

	while (true) {
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		chassis.driveTank(left, right);

		pros::delay(50);
	}
}