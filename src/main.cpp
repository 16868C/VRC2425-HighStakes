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

	inertial.reset(true);
	leftDrive.tarePosition();
	rightDrive.tarePosition();

	#ifdef ANSONBOT
	cataEnc.resetZero();
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
	
	// goalAWPBar();
	// goalAWP();
	// matchloadAWPBar();
	matchloadRush();

	// skills();

	std::cout << "Auton took " << pros::millis() - st << " ms" << std::endl;
}

void opcontrol() {
	#ifdef ODOMBOT

	// while (inertial.get_rotation() == INFINITY) pros::delay(20);
	// chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

	// int initX = gps.get_status().x, initY = gps.get_status().y, initTheta = gps.get_status().yaw;

	// // chassis.moveDistance(48_in, 200_rpm, {0.11, 0, 0.5}, 150, 0_deg, 100_rpm, {0.1, 0, 0}, 0);
	// chassis.turnAbsolute(-90_deg, 50_rpm, {0.1, 0, 0.1}, 1.05, 2, 5, lib16868C::TurnWheel::RIGHT, 0);

	// std::cout << "[Odom] ";
	// odomDriveEnc.getPose().print();

	// std::cout << "[GPS] x: " << Util::mToIn(gps.get_status().x - initX) << " y: " << Util::mToIn(gps.get_status().y - initY) << " theta: " << gps.get_status().yaw - initTheta << "\n";

	okapi::ControllerButton calcPos(okapi::ControllerDigital::A);

	while (true) {
		double forward = master.getAnalog(okapi::ControllerAnalog::leftY);
		double turn = master.getAnalog(okapi::ControllerAnalog::rightX);
		chassis.driveArcade(forward, turn);

		double hDist = Util::mToIn(horizontalDistance.get() / 1000);
		double vDist = Util::mToIn(verticalDistance.get() / 1000);
		pros::lcd::print(0, "V: %.2f, H: %.2f", hDist, vDist);
		pros::lcd::print(1, "Theta: %.2f", inertial.get_rotation());

		double xPos = hDist * std::cos(inertial.get_rotation() * okapi::pi / 180.0);
		double yPos = vDist * std::cos(inertial.get_rotation() * okapi::pi / 180.0);
		pros::lcd::print(2, "X: %.2f, Y: %.2f", xPos, yPos);

		pros::delay(20);
	}
	#endif

	#ifdef ANSONBOT
	// skillsStart();
	
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

	okapi::ControllerButton intakeTgl(okapi::ControllerDigital::R1);
	okapi::ControllerButton outtakeTgl(okapi::ControllerDigital::R2);
	int intakeDir = 0;

	okapi::ControllerButton cataFire(okapi::ControllerDigital::L1);
	okapi::ControllerButton cataIntake(okapi::ControllerDigital::L2);
	okapi::ControllerButton matchloadTgl(okapi::ControllerDigital::up);
	bool matchloading = true;

	okapi::ControllerButton intakeRaiserTgl(okapi::ControllerDigital::left);
	okapi::ControllerButton hangTgl(okapi::ControllerDigital::down);
	okapi::ControllerButton wingTgl(okapi::ControllerDigital::X);
	okapi::ControllerButton leftWingTgl(okapi::ControllerDigital::Y);
	okapi::ControllerButton rightWingTgl(okapi::ControllerDigital::A);

	okapi::ControllerButton matchloadComplete(okapi::ControllerDigital::right);

	uint st = pros::millis();
	while (true) {
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		if (hang.getState()) {
			left = std::clamp(left, -0.55, 0.55);
			right = std::clamp(right, -0.55, 0.55);
		}
		chassis.driveTank(left, right);
		// double forward = master.getAnalog(okapi::ControllerAnalog::leftY);
		// double turn = master.getAnalog(okapi::ControllerAnalog::rightX);
		// chassis.driveArcade(forward, turn);

		if (intakeTgl.changedToPressed()) intakeDir = intakeDir == 1 ? 0 : 1;
		else if (outtakeTgl.changedToPressed()) intakeDir = intakeDir == -1 ? 0 : -1;
		intake.moveVelocity(intakeDir * 600);

		if (matchloadTgl.changedToPressed()) {
			if (catapult.isSettled()) catapult.matchload();
			else catapult.stop();
		}

		if (cataFire.changedToPressed()) catapult.fire();
		if (cataIntake.changedToPressed()) catapult.intake();

		if (hangTgl.changedToPressed()) hang.toggle();

		if (intakeRaiserTgl.changedToPressed()) intakeRaiser.toggle();

		if (wingTgl.changedToPressed()) { leftWing.toggle(); rightWing.toggle(); }
		if (leftWingTgl.changedToPressed()) leftWing.toggle();
		if (rightWingTgl.changedToPressed()) rightWing.toggle();
		
		if (matchloadComplete.changedToPressed()) chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

		pros::lcd::print(0, "Time: %d", pros::millis() - st);
		pros::lcd::print(1, "Left Temp: %.0f, Right Temp: %.0f", leftDrive.getTemperature(), rightDrive.getTemperature());
		pros::lcd::print(2, "Intake Temp: %.0f", intakeMtr.getTemperature());
		pros::lcd::print(3, "Cata Temp: %.0f", cataMtr.getTemperature());
		pros::lcd::print(4, "Cata Enc: %.2f", cataEnc.get());
		pros::lcd::print(5, "Left Wing: %d, Right Wing: %d", leftWing.getState(), rightWing.getState());
		pros::lcd::print(6, "Inertial: %.2f", inertial.get_rotation());

		pros::delay(20);
	}
	#endif
}