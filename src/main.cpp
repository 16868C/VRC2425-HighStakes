#include "main.h"
#include "robotconfig.hpp"
#include "16868Z/controllers/PIDController.hpp"
#include "16868Z/subsystems/chassis/motionProfiling.hpp"
#include "16868Z/util/util.hpp"
#include "routes.hpp"
#include <fstream>

using namespace lib16868C;

void initialize() {
	pros::lcd::initialize();
	
	// bool inertialResetFailed = true, inertialDrift = false;
	// do {
	// 	if (!inertialResetFailed) {
	// 		std::cerr << "Inertial Reset Failed" << std::endl;
	// 		master.setText(0, 0, "Inertial Reset Failed");
	// 		pros::lcd::print(0, "Inertial Reset Failed");
	// 		master.rumble("-");
	// 	}
	// 	inertialResetFailed = inertial.reset(true) - 1;
	// } while (inertialResetFailed);
	// double h1 = inertial.get_rotation();
	// pros::delay(300);
	// double h2 = inertial.get_rotation();
	// if (std::abs(h1 - h2) > 0.5) {
	// 	inertialDrift = true;
	// 	std::cerr << "Inertial Drift Detected: " << std::abs(h1 - h2) << " deg difference in 300ms" << std::endl;
	// 	master.setText(0, 0, "Inertial Drift Detected: " + std::to_string(std::abs(h1 - h2)) + " deg difference in 300ms");
	// 	pros::lcd::print(0, "Inertial Drift Detected: %f deg difference in 300ms", std::abs(h1 - h2));
	// 	master.rumble("-");
	// }

	leftDrive.tarePosition();
	rightDrive.tarePosition();
	// turretMotor.tarePosition();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	int st = pros::millis();
	
	// goalSide();
	// goalSideBar();
	// matchloadAWP();

	std::cout << "Auton took " << pros::millis() - st << " ms" << std::endl;
}

void opcontrol() {
	okapi::ControllerButton intakeTgl(okapi::ControllerDigital::R1);
	okapi::ControllerButton outtakeTgl(okapi::ControllerDigital::R2);
	int intakeDir = 0;

	okapi::ControllerButton cataFire(okapi::ControllerDigital::L1);
	okapi::ControllerButton cataIntake(okapi::ControllerDigital::L2);

	okapi::ControllerButton tomTgl(okapi::ControllerDigital::B);

	while (true) {
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		chassis.driveTank(left, right);

		if (intakeTgl.changedToPressed()) intakeDir = intakeDir == 1 ? 0 : 1;
		else if (outtakeTgl.changedToPressed()) intakeDir = intakeDir == -1 ? 0 : -1;
		intakeMtr.moveVelocity(intakeDir * 600);

		if (cataFire.changedToPressed()) catapult.fire();
		else if (cataIntake.changedToPressed()) catapult.intake();

		if (tomTgl.changedToPressed()) tom.toggle();

		pros::delay(20);
	}
}