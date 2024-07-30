#include "main.h"
#include "16868C/util/pose.hpp"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"
#include "routes.hpp"
#include <functional>

using namespace lib16868C;

std::function<void()> auton = redSoloAWP;

void initialize() {
	pros::lcd::initialize();

	odometry.init();
	// odometry.init(Pose(0_in, 0_in, 0_deg));
	// inertial.reset(true);
	// chassis.coast();

	/*while (true) {
		master.clearLine(0);

		int a = autonSelector.get_value();
		if (a > 2927 && a <= 3535) {
			auton = redSoloAWP;
			master.setText(0, 0, "Red Solo AWP");
			pros::lcd::print(7, "Red Solo AWP");
		} else if (a > 3535 && a <= 4095) {
			auton = redSoloAWP;
			master.setText(0, 0, "Blue Solo AWP");
			pros::lcd::print(7, "Blue Solo AWP");
		} else if (a > 4095 && a <= 194) {
			auton = redSoloAWP;
			master.setText(0, 0, "Red Rush");
			pros::lcd::print(7, "Red Rush");
		} else if (a > 194 && a <= 660) {
			auton = redSoloAWP;
			master.setText(0, 0, "Blue Rush");
			pros::lcd::print(7, "Blue Rush");
		} else if (a > 660 && a <= 1044) {
			auton = redSoloAWP;
			master.setText(0, 0, "Red Left Score");
			pros::lcd::print(7, "Red Left Score");
		} else if (a > 1044 && a <= 1353) {
			auton = redSoloAWP;
			master.setText(0, 0, "Blue Right Score");
			pros::lcd::print(7, "Blue Right Score");
		} else if (a > 1353 && a <= 1797) {
			auton = redSoloAWP;
			master.setText(0, 0, "Red Right Score");
			pros::lcd::print(7, "Red Right Score");
		} else if (a > 1797 && a <= 2131) {
			auton = redSoloAWP;
			master.setText(0, 0, "Blue Left Score");
			pros::lcd::print(7, "Blue Left Score");
		} else if (a > 2131 && a <= 2515) {
			auton = redSoloAWP;
			master.setText(0, 0, "Skills 1");
			pros::lcd::print(7, "Skills 1");
		} else {
			auton = redSoloAWP;
			master.setText(0, 0, "Skills 2");
			pros::lcd::print(7, "Skills 2");
		}

		pros::delay(100);
	}*/
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	uint st = pros::millis();
	arm.resetPosition();

	// auton();
	// redSoloAWP();

	// chassis.moveDistance(48_in, 600_rpm, {0.04, 0, 1.5}, 0_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	// chassis.turnAbsolute(90_deg, 600_rpm, {0.02, 0, 0.95}, 3, 5, TurnWheel::BOTH, 0);

	// chassis.moveToPoint({60_in, -56_in}, 600_rpm, {0.06, 0, 0.1}, {0.8, 0, 1}, 3_in, false, false, 0);
	// chassis.moveToPoint({48_in, -72_in}, 600_rpm, {0.06, 0, 0.1}, {0.8, 0, 1}, 3_in, false, true, 0);
	// pros::delay(1000);
	// chassis.moveToPoint({60_in, -45_in}, 600_rpm, {0.05, 0, 0.1}, {0.8, 0, 1}, 3_in, true, false, 0);
	// chassis.moveToPoint({0_in, 0_in}, 600_rpm, {0.06, 0, 0.1}, {0.8, 0, 1}, 3_in, true, true, 0);
	
	printDebug("Auton took %d ms\n", pros::millis() - st);
}

void opcontrol() {
	uint st = pros::millis();
	arm.resetPosition();
	// redSoloAWP();
	// blueSoloAWP();
	printDebug("%d ms\n", pros::millis() - st);

	okapi::ControllerButton shift(okapi::ControllerDigital::R2);

	okapi::ControllerButton intakeTgl(okapi::ControllerDigital::R1);
	okapi::ControllerButton outtakeTgl(okapi::ControllerDigital::L1);
	okapi::ControllerButton intakeBasketTgl(okapi::ControllerDigital::Y);
	okapi::ControllerButton targetBlueTgl(okapi::ControllerDigital::down);
	okapi::ControllerButton targetRedTgl(okapi::ControllerDigital::B);

	okapi::ControllerButton clampTgl(okapi::ControllerDigital::L2);
	okapi::ControllerButton tiltTgl(okapi::ControllerDigital::right);
	okapi::ControllerButton hangRelease(okapi::ControllerDigital::A);

	okapi::ControllerButton armIdle(okapi::ControllerDigital::L2);
	okapi::ControllerButton armWallStake(okapi::ControllerDigital::L1);
	okapi::ControllerButton armAllianceStake(okapi::ControllerDigital::right);
	okapi::ControllerButton armDescoreStake(okapi::ControllerDigital::Y);

	while (true) {
		// Drivetrain -> Tank Drive
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		chassis.driveTank(left, right);
		pros::lcd::print(1, "%.2f %.2f", leftDrive.getTemperature(), rightDrive.getTemperature());

		// Intake: L1 -> Intake, R1 -> Outtake, Press again to stop
		// std::cout << armMtrs.getPosition() << "\n";
		if (!shift.isPressed() && intakeTgl.changedToPressed() && (arm.getState() == ArmPosition::DEFAULT || arm.getState() == ArmPosition::IDLE)) {
			if (intake.getState() == IntakeState::INTAKE_MOGO) intake.stop();
			else intake.intakeMogo();
		}
		else if (!shift.isPressed() && outtakeTgl.changedToPressed() && (arm.getState() == ArmPosition::DEFAULT || arm.getState() == ArmPosition::IDLE)) {
			if (intake.getState() == IntakeState::OUTTAKE) intake.stop();
			else intake.outtake();
		}
		else if (!shift.isPressed() && intakeBasketTgl.changedToPressed() && (arm.getState() == ArmPosition::DEFAULT || arm.getState() == ArmPosition::IDLE)) {
			if (intake.getState() == IntakeState::INTAKE_BASKET) intake.stop();
			else intake.intakeBasket();
		}
		pros::lcd::print(2, "%.2f, %.2f", intakeMtr.getTemperature(), armMtrs.getTemperature());

		if (targetBlueTgl.changedToPressed())
			intake.setTarget(intake.getTarget() == TargetRing::BLUE ? TargetRing::NONE : TargetRing::BLUE);
		if (targetRedTgl.changedToPressed())
			intake.setTarget(intake.getTarget() == TargetRing::RED ? TargetRing::NONE : TargetRing::RED);

		if (!shift.isPressed() && clampTgl.changedToPressed()) {
			clamp.toggle();
			if (clamp.getState()) tilter.retract();
			else tilter.extend();
		}
		if (!shift.isPressed() && tiltTgl.changedToPressed()) {
			tilter.toggle();
		}

		if (shift.isPressed() && armIdle.changedToPressed()) {
			arm.defaultPos();
		}
		if (shift.isPressed() && armWallStake.changedToPressed()) {
			arm.wallStake();
			intake.stop();
		}
		if (shift.isPressed() && armAllianceStake.changedToPressed()) {
			arm.allianceStake();
			intake.stop();
		}
		if (shift.isPressed() && armDescoreStake.changedToPressed()) {
			arm.descoreStake();
			intake.stop();
		}

		pros::delay(100);
	}
}