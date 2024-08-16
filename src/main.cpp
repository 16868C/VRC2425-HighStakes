#include "main.h"
#include "16868C/util/pose.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"
#include "routes.hpp"
#include <functional>

using namespace lib16868C;

std::function<void()> auton = redSoloAWP;
pros::Task autonSelect = pros::Task([]() {
	while (true) {
		master.clearLine(0);

		int a = autonSelector.get_value();
		if (a > 2899 && a <= 3554) { // 1
			auton = redSoloAWP;
			master.setText(0, 0, "Red Solo AWP");
			pros::lcd::print(7, "Red Solo AWP");
			printDebug("Red Solo AWP\n");
		} else if (a > 3554 && a <= 4095) { // 2
			auton = blueSoloAWP;
			master.setText(0, 0, "Blue Solo AWP");
			pros::lcd::print(7, "Blue Solo AWP");
			printDebug("Blue Solo AWP\n");
		} else if (a > 4095 || a <= 146) { // 3
			auton = redRightAWP;
			master.setText(0, 0, "Red Right AWP");
			pros::lcd::print(7, "Red Right AWP");
			printDebug("Red Right AWP\n");
		} else if (a > 146 && a <= 594) { // 4
			auton = blueLeftAWP;
			master.setText(0, 0, "Blue Left AWP");
			pros::lcd::print(7, "Blue Left AWP");
			printDebug("Blue Left AWP\n");
		} else if (a > 594 && a <= 1007) { // 5
			auton = skills;
			master.setText(0, 0, "Skills");
			pros::lcd::print(7, "Skills");
			printDebug("Skills\n");
		} else if (a > 1007 && a <= 1363) { // 6
			auton = redSoloAWPSafe;
			master.setText(0, 0, "Red Solo AWP Safe");
			pros::lcd::print(7, "Red Solo AWP Safe");
			printDebug("Red Solo AWP Safe\n");
		} else if (a > 1363 && a <= 1793) { // 7
			auton = blueSoloAWPSafe;
			master.setText(0, 0, "Blue Solo AWP Safe");
			pros::lcd::print(7, "Blue Solo AWP Safe");
			printDebug("Blue Solo AWP Safe\n");
		} else if (a > 1793 && a <= 2188) { // 8
			auton = blueSoloAWPSafeAdjusted;
			master.setText(0, 0, "blueSoloAWPSafeAdjusted");
			pros::lcd::print(7, "blueSoloAWPSafeAdjusted");
			printDebug("blueSoloAWPSafeAdjusted\n");
		} else if (a > 2188 && a <= 2548) { // 9
			auton = redElimScore;
			master.setText(0, 0, "redElimScore");
			pros::lcd::print(7, "redElimScore");
			printDebug("redElimScore\n");
		} else if (a > 2548 && a <= 2899) { // 10
			auton = blueElimScore;
			master.setText(0, 0, "blueElimScore");
			pros::lcd::print(7, "blueElimScore");
			printDebug("blueElimScore\n");
		} else {
			master.setText(0, 0, "No Auton");
			pros::lcd::print(7, "No Auton");
			printDebug("No Auton: %d\n", a);
		}

		pros::delay(100);
	}
}, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Autonomous Selector");

void initialize() {
	pros::lcd::initialize();

	// odometry.init();
	// armMtrs.resetZero();
	// odometry.init(Pose(0_in, 0_in, 0_deg));
	// inertial.reset(true);
	// chassis.coast();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	autonSelect.suspend();
	uint st = pros::millis();
	// arm.resetPosition();

	auton();
	// redRightAWP();
	// redSoloAWP();
	// blueSoloAWP();

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
	autonSelect.suspend();
	
	okapi::ControllerButton shift(okapi::ControllerDigital::R2);

	okapi::ControllerButton intakeTgl(okapi::ControllerDigital::R1);
	okapi::ControllerButton outtakeTgl(okapi::ControllerDigital::L1);
	okapi::ControllerButton intakeBasketTgl(okapi::ControllerDigital::Y);
	okapi::ControllerButton targetBlueTgl(okapi::ControllerDigital::down);
	okapi::ControllerButton targetRedTgl(okapi::ControllerDigital::B);

	okapi::ControllerButton clampTgl(okapi::ControllerDigital::L2);
	okapi::ControllerButton stickTgl(okapi::ControllerDigital::right);
	okapi::ControllerButton hangRelease(okapi::ControllerDigital::A);

	okapi::ControllerButton armIdle(okapi::ControllerDigital::L2);
	okapi::ControllerButton armWallStake(okapi::ControllerDigital::L1);
	okapi::ControllerButton armAllianceStake(okapi::ControllerDigital::right);
	okapi::ControllerButton armDescoreStake(okapi::ControllerDigital::Y);

	while (true) {
		// Drivetrain -> Tank Drive
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		if (hang.is_extended()) {
			left *= 0.5;
			right *= 0.5;
		}
		chassis.driveTank(left, right);
		pros::lcd::print(1, "%.2f %.2f", leftDrive.getTemperature(), rightDrive.getTemperature());

		// Intake: L1 -> Intake, R1 -> Outtake, Press again to stop
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

		// if (targetBlueTgl.changedToPressed())
		// 	intake.setTarget(intake.getTarget() == TargetRing::BLUE ? TargetRing::NONE : TargetRing::BLUE);
		// if (targetRedTgl.changedToPressed())
		// 	intake.setTarget(intake.getTarget() == TargetRing::RED ? TargetRing::NONE : TargetRing::RED);

		if (!shift.isPressed() && clampTgl.changedToPressed()) {
			clamp.toggle();
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

		if (!shift.isPressed() && stickTgl.changedToPressed())
			stick.toggle();

		if (!shift.isPressed() && hangRelease.changedToPressed()) {
			arm.defaultPos();
			hang.extend();
		}

		int leftDriveTemp = std::max(leftDrive.getTemperature() / 5 - 10, 0.0);
		int rightDriveTemp = std::max(rightDrive.getTemperature() / 5 - 10, 0.0);
		int intakeTemp = std::max(intakeMtr.getTemperature() / 5 - 10, 0.0);
		int armTemp = std::max(armMtrs.getTemperature() / 5 - 10, 0.0);
		master.setText(0, 0, std::to_string(leftDriveTemp) + " " + std::to_string(rightDriveTemp) + " " + std::to_string(intakeTemp) + " " + std::to_string(armTemp));

		pros::delay(100);
	}
}