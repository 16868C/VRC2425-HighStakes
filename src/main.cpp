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
	pros::lcd::set_text(0, "Initializing...");
	
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
	
	// odomThreeEnc.init();
	// odomTwoEnc.init();
	odomDriveEnc.init();
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

	double kp = 0.115, accelRate = 1.2;
	double vel = 1000;
	while (odomDriveEnc.getPose().y < 24_in) {
		if (vel < 9000) vel *= accelRate;
		chassis.moveArcade(vel * std::clamp((24_in - odomDriveEnc.getPose().y).convert(okapi::inch) * kp, -1.0, 1.0), 0);
		pros::delay(20);
	}
	chassis.moveArcade(0, 0);

	pros::delay(1000);

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
		pros::lcd::print(2, "Left: %f, Right: %f", left, right);
		chassis.driveTank(left, right);

		if (intakeTgl.changedToPressed()) intakeDir = intakeDir == 1 ? 0 : 1;
		else if (outtakeTgl.changedToPressed()) intakeDir = intakeDir == -1 ? 0 : -1;
		intakeMtr.moveVelocity(intakeDir * 600);

		if (matchloadTgl.changedToPressed()) {
			if (catapult.isSettled()) catapult.matchload();
			else catapult.stop();
		}

		if (cataFire.changedToPressed()) catapult.fire();
		if (cataIntake.changedToPressed()) {
			if (cataDist.get() > 20 && cataDist.get() < 10) catapult.intake();
			else catapult.stop();
		}
		pros::lcd::print(0, "dist: %f", cataDist.get());

		if (wingTgl.changedToPressed()) wings.toggle();

		// if (tomTgl.changedToPressed()) tom.toggle();

		pros::delay(20);
	}
	#endif
}