#include "main.h"
#include "16868C/util/pose.hpp"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"

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

	chassis.moveToPoint({60_in, -56_in}, 600_rpm, {0.06, 0, 0.1}, {0.8, 0, 1}, 3_in, false, false, 0);
	chassis.moveToPoint({48_in, -72_in}, 600_rpm, {0.06, 0, 0.1}, {0.8, 0, 1}, 3_in, false, true, 0);
	pros::delay(1000);
	chassis.moveToPoint({60_in, -45_in}, 600_rpm, {0.05, 0, 0.1}, {0.8, 0, 1}, 3_in, true, false, 0);
	chassis.moveToPoint({0_in, 0_in}, 600_rpm, {0.06, 0, 0.1}, {0.8, 0, 1}, 3_in, true, true, 0);
	

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
		intake.moveVoltage(intakeDir * 12000);
		pros::lcd::print(2, "%.2f, %.2f", intake.getTemperature(), arm.getTemperature());

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
			intakeDir = 0;
		}
		if (shift.isPressed() && armAllianceStake.changedToPressed()) {
			arm.moveAbsolute(600, 200);
			intakeDir = 0;
		}
		if (shift.isPressed() && armDescoreStake.changedToPressed()) {
			arm.moveAbsolute(750, 200);
			intakeDir = 0;
		}
		pros::lcd::print(0, "%f", arm.getPosition());

		pros::delay(50);
	}
}