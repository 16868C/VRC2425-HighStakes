#include "main.h"
#include "16868C/util/pose.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"
#include "routes.hpp"
#include <functional>

using namespace lib16868C;

// std::function<void()> auton = redSoloAWP;
// pros::Task autonSelect = pros::Task([]() {
	/*
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
	*/
// }, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Autonomous Selector");

void initialize() {
	pros::lcd::initialize();

	armEnc.resetZero();
	intakeEnc.resetZero();
	odometry.init();
	// odometry.init(Pose(0_in, 0_in, 0_deg));
	// inertial.reset(true);
	// chassis.coast();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// autonSelect.suspend();
	uint st = pros::millis();
	// arm.resetPosition();

	// auton();
	// redRightAWP();
	// redSoloAWP();
	// blueSoloAWP();

	// chassis.moveToPoint({})

	// chassis.moveDistance(48_in, 600_rpm, {0.04, 0, 1.5}, 0_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	// chassis.turnAbsolute(90_deg, 600_rpm, {0.02, 0, 0.95}, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveToPoint({24_in, 24_in}, 600_rpm, {0.04, 0, 1.5}, {1, 0, 1}, 3_in, false, true, 0);

	// chassis.moveToPoint({60_in, -56_in}, 600_rpm, {0.06, 0, 0.1}, {0.8, 0, 1}, 3_in, false, false, 0);
	// chassis.moveToPoint({48_in, -72_in}, 600_rpm, {0.06, 0, 0.1}, {0.8, 0, 1}, 3_in, false, true, 0);
	// pros::delay(1000);
	// chassis.moveToPoint({60_in, -45_in}, 600_rpm, {0.05, 0, 0.1}, {0.8, 0, 1}, 3_in, true, false, 0);
	// chassis.moveToPoint({0_in, 0_in}, 600_rpm, {0.06, 0, 0.1}, {0.8, 0, 1}, 3_in, true, true, 0);
	
	printDebug("Auton took %d ms\n", pros::millis() - st);
}

void opcontrol() {
	// autonSelect.suspend();
	
	okapi::ControllerButton shift(okapi::ControllerDigital::R2);
	okapi::ControllerButton ptoTgl(okapi::ControllerDigital::B);

	okapi::ControllerButton intakeTgl(okapi::ControllerDigital::R1);
	okapi::ControllerButton outtakeTgl(okapi::ControllerDigital::L1);
	okapi::ControllerButton intakeRedirect(okapi::ControllerDigital::Y);
	okapi::ControllerButton targetBlueTgl(okapi::ControllerDigital::left);
	okapi::ControllerButton targetRedTgl(okapi::ControllerDigital::up);

	okapi::ControllerButton intakeHoldTgl(okapi::ControllerDigital::left);

	okapi::ControllerButton clampTgl(okapi::ControllerDigital::L2);
	okapi::ControllerButton doinkerTgl(okapi::ControllerDigital::right);
	okapi::ControllerButton hangRelease(okapi::ControllerDigital::A);

	okapi::ControllerButton armIdle(okapi::ControllerDigital::L2);
	okapi::ControllerButton armWallStake(okapi::ControllerDigital::L1);
	okapi::ControllerButton armAllianceStake(okapi::ControllerDigital::right);
	okapi::ControllerButton armDescoreStake(okapi::ControllerDigital::down);

	while (true) {
		// Drivetrain -> Tank Drive
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		if (hang.is_extended()) {
			left *= 0.5;
			right *= 0.5;
		}
		chassis.driveTank(left, right, 0.05);
		// pros::lcd::print(1, "%.2f %.2f", leftDrive.getTemperature(), rightDrive.getTemperature());

		if (ptoTgl.changedToPressed()) {
			pto.toggle();
		}

		// Intake: L1 -> Intake, R1 -> Outtake, Press again to stop
		// std::cout << static_cast<int>(intake.getState()) << " " << static_cast<int>(IntakeState::MOGO) << "\n";
		if (!shift.isPressed() && intakeTgl.changedToPressed()) {
			pros::Task([&] {
				if (!pto.is_extended()) {
					pto.extend();
					arm.move(0);
					pros::delay(500);
				}
				if (intake.getState() == IntakeState::MOGO) intake.stop();
				else intake.mogo();
			});
		} else if (!shift.isPressed() && outtakeTgl.changedToPressed()) {
			pros::Task([&] {
				if (!pto.is_extended()) {
					pto.extend();
					arm.move(0);
					pros::delay(500);
				}
				if (intake.getState() == IntakeState::OUTTAKE) intake.stop();
				else intake.outtake();
			});
		} else if (!shift.isPressed() && intakeRedirect.changedToPressed()) {
			pros::Task([&] {
				if (!pto.is_extended()) {
					pto.extend();
					arm.move(0);
					pros::delay(500);
				}
				if (intake.getState() == IntakeState::REDIRECT) intake.stop();
				else intake.redirect();
			});
		} else if (!shift.isPressed() && intakeHoldTgl.changedToPressed()) {
			pros::Task([&] {
				if (!pto.is_extended()) {
					pto.extend();
					arm.move(0);
					pros::delay(500);
				}
				if (intake.getState() == IntakeState::INTAKE) intake.stop();
				else intake.intake();
			});
		}
		// pros::lcd::print(2, "%.2f, %.2f", intakeFirst.getTemperature(), intakeSecond.getTemperature());

		if (targetBlueTgl.changedToPressed())
			intake.setTargetRing(intake.getTargetRing() == RingColour::BLUE ? RingColour::NONE : RingColour::BLUE);
		if (targetRedTgl.changedToPressed())
			intake.setTargetRing(intake.getTargetRing() == RingColour::RED ? RingColour::NONE : RingColour::RED);

		if (!shift.isPressed() && clampTgl.changedToPressed()) {
			clamp.toggle();
		}

		if (shift.isPressed() && armIdle.changedToPressed()) {
			pros::Task([&] {
				if (pto.is_extended()) {
					pto.retract();
					intake.stop();
					pros::delay(500);
				}
				arm.defaultPos();
			});
		}
		if (shift.isPressed() && armWallStake.changedToPressed()) {
			pros::Task([&] {
				if (pto.is_extended()) {
					pto.retract();
					intake.stop();
					pros::delay(500);
				}
				arm.wallStake();
			});
		}
		if (shift.isPressed() && armAllianceStake.changedToPressed()) {
			pros::Task([&] {
				if (pto.is_extended()) {
					pto.retract();
					intake.stop();
					pros::delay(500);
				}
				arm.allianceStake();
			});
		}
		if (shift.isPressed() && armDescoreStake.changedToPressed()) {
			pros::Task([&] {
				if (pto.is_extended()) {
					pto.retract();
					intake.stop();
					pros::delay(500);
				}
				arm.descoreStake();
			});
		}

		if (!shift.isPressed() && doinkerTgl.changedToPressed())
			doinker.toggle();

		if (!shift.isPressed() && hangRelease.changedToPressed()) {
			arm.defaultPos();
			hang.extend();
		}

		int leftDriveTemp = std::max(leftDrive.getTemperature() / 5 - 10, 0.0);
		int rightDriveTemp = std::max(rightDrive.getTemperature() / 5 - 10, 0.0);
		int intakeFirstTemp = std::max(intakeFirst.getTemperature() / 5 - 10, 0.0);
		int intakeSecondTemp = std::max(intakeSecond.getTemperature() / 5 - 10, 0.0);
		master.setText(0, 0, std::to_string(leftDriveTemp) + " " + std::to_string(rightDriveTemp) + " " + std::to_string(intakeFirstTemp) + " " + std::to_string(intakeSecondTemp));

		pros::delay(20);
	}
}