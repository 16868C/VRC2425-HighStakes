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

	// inertial.reset(true);
	leftDrive.tarePosition();
	rightDrive.tarePosition();

	#ifdef ANSONBOT
	// cataEnc.resetZero();
	#endif
	
	#ifdef ODOMBOT
	// odomThreeEnc.init();
	// odomTwoEnc.init();
	// odomDriveEnc.init();
	#endif
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
	#ifdef ODOMBOT

	while (inertial.get_rotation() == INFINITY) pros::delay(20);
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

	int initX = gps.get_status().x, initY = gps.get_status().y, initTheta = gps.get_status().yaw;

	// chassis.moveDistance(48_in, 200_rpm, {0.11, 0, 0.5}, 150, 0_deg, 100_rpm, {0.1, 0, 0}, 0);
	chassis.turnAbsolute(-90_deg, 50_rpm, {0.1, 0, 0.1}, 1.05, 2, 5, lib16868C::TurnWheel::RIGHT, 0);

	std::cout << "[Odom] ";
	odomDriveEnc.getPose().print();

	std::cout << "[GPS] x: " << Util::mToIn(gps.get_status().x - initX) << " y: " << Util::mToIn(gps.get_status().y - initY) << " theta: " << gps.get_status().yaw - initTheta << "\n";

	while (true) {
		double forward = master.getAnalog(okapi::ControllerAnalog::leftY);
		double turn = master.getAnalog(okapi::ControllerAnalog::rightX);
		chassis.driveArcade(forward, turn);

		pros::delay(20);
	}
	#endif

	#ifdef ANSONBOT
	// chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

	// chassis.moveDistance(48_in, 600_rpm, {0.04, 0, 0.3}, 300, 0_deg, 300_rpm, {0.1, 0, 0}, 0);

	// pros::delay(1000);
	// double avgTicks = std::abs((leftDrive.getEncoder()->get() + rightDrive.getEncoder()->get()) / 2.0);
	// std::cout << avgTicks / 300 * (WHEEL_DIAMETER * okapi::pi).convert(okapi::inch) * GEAR_RATIO;

	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

	okapi::ControllerButton intakeTgl(okapi::ControllerDigital::R1);
	okapi::ControllerButton outtakeTgl(okapi::ControllerDigital::R2);
	int intakeDir = 0;

	okapi::ControllerButton cataFire(okapi::ControllerDigital::L1);
	okapi::ControllerButton cataIntake(okapi::ControllerDigital::L2);
	okapi::ControllerButton matchloadTgl(okapi::ControllerDigital::up);
	bool matchloading = false;

	// okapi::ControllerButton tomTgl(okapi::ControllerDigital::B);
	okapi::ControllerButton wingTgl(okapi::ControllerDigital::A);

	while (true) {
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		chassis.driveTank(left, right);

		if (intakeTgl.changedToPressed()) intakeDir = intakeDir == 1 ? 0 : 1;
		else if (outtakeTgl.changedToPressed()) intakeDir = intakeDir == -1 ? 0 : -1;
		intakeMtrs.moveVelocity(intakeDir * 600);

		if (matchloadTgl.changedToPressed()) {
			if (catapult.isSettled()) catapult.matchload();
			else catapult.stop();
		}

		if (cataFire.changedToPressed()) catapult.fire();
		if (cataIntake.changedToPressed()) catapult.intake();

		// if (wingTgl.changedToPressed()) wings.toggle();

		// if (tomTgl.changedToPressed()) tom.toggle();

		pros::delay(20);
	}
	#endif
}