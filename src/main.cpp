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
	arm.moveVoltage(-12000);
	pros::delay(500);
	arm.moveVoltage(0);
	pros::delay(100);
	arm.resetZero();
	arm.moveAbsolute(0, 200);

	okapi::ControllerButton shift(okapi::ControllerDigital::R2);

	okapi::ControllerButton intakeTgl(okapi::ControllerDigital::R1);
	okapi::ControllerButton outtakeTgl(okapi::ControllerDigital::L1);
	okapi::ControllerButton clampTgl(okapi::ControllerDigital::L2);
	okapi::ControllerButton tiltTgl(okapi::ControllerDigital::right);
	okapi::ControllerButton hangRelease(okapi::ControllerDigital::Y);

	okapi::ControllerButton armIdle(okapi::ControllerDigital::L2);
	okapi::ControllerButton armWallStake(okapi::ControllerDigital::L1);
	okapi::ControllerButton armAllianceStake(okapi::ControllerDigital::right);
	okapi::ControllerButton armDescoreStake(okapi::ControllerDigital::Y);

	int intakeDir = 0;

	while (true) {
		// Drivetrain -> Tank Drive
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		chassis.driveTank(left, right);
		pros::lcd::print(1, "%.2f %.2f", leftDrive.getTemperature(), rightDrive.getTemperature());

		// Intake: L1 -> Intake, R1 -> Outtake, Press again to stop
		if (!shift.isPressed() && intakeTgl.changedToPressed()) intakeDir = intakeDir == 1 ? 0 : 1;
		else if (!shift.isPressed() && outtakeTgl.changedToPressed()) intakeDir = intakeDir == -1 ? 0 : -1;
		intake.moveVoltage(intakeDir * 10000);

		if (!shift.isPressed() && clampTgl.changedToPressed()) {
			clamp.toggle();
			if (clamp.getState()) tilter.retract();
			else tilter.extend();
		}
		if (!shift.isPressed() && tiltTgl.changedToPressed()) {
			tilter.toggle();
		}

		if (shift.isPressed() && armIdle.changedToPressed()) {
			arm.moveAbsolute(0, 200);
		}
		if (shift.isPressed() && armWallStake.changedToPressed()) {
			arm.moveAbsolute(1150, 200);
		}
		if (shift.isPressed() && armAllianceStake.changedToPressed()) {
			arm.moveAbsolute(600, 200);
		}
		if (shift.isPressed() && armDescoreStake.changedToPressed()) {
			arm.moveAbsolute(750, 200);
		}
		pros::lcd::print(0, "%f", arm.getPosition());

		pros::delay(50);
	}
}