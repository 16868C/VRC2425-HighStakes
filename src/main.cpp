#include "main.h"
#include "robotconfig.hpp"
#include "16868C/controllers/PIDController.hpp"
#include "16868C/subsystems/chassis/motionProfiling.hpp"
#include "16868C/util/util.hpp"
#include "routes.hpp"
#include <algorithm>
#include <fstream>

using namespace lib16868C;

void initialize() {
	pros::lcd::initialize();

	#ifdef ANSONBOT
	bool inertialResetFailed = true, inertialDrift = false;
	do {
		if (!inertialResetFailed) {
			std::cerr << "Inertial Reset Failed" << std::endl;
			master.setText(0, 0, "Inertial Reset Failed");
			pros::lcd::print(0, "Inertial Reset Failed");
			master.rumble("-");
		}
		inertialResetFailed = inertial.reset(true) - 1;
	} while (inertialResetFailed);
	double h1 = inertial.get_rotation();
	pros::delay(300);
	double h2 = inertial.get_rotation();
	if (std::abs(h1 - h2) > 0.5) {
		inertialDrift = true;
		std::cerr << "Inertial Drift Detected: " << std::abs(h1 - h2) << " deg difference in 300ms" << std::endl;
		master.setText(0, 0, "Inertial Drift Detected: " + std::to_string(std::abs(h1 - h2)) + " deg difference in 300ms");
		pros::lcd::print(0, "Inertial Drift Detected: %f deg difference in 300ms", std::abs(h1 - h2));
		master.rumble("-");
	}
	#endif

	
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	int st = pros::millis();

	std::cout << "Auton took " << pros::millis() - st << " ms" << std::endl;
}

void opcontrol() {
	while (true) {
		pros::delay(20);
	}
}