#include "main.h"
#include "okapi/impl/device/controllerUtil.hpp"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"
#include "routes.hpp"
#include <functional>

using namespace lib16868C;

// std::function<void()> auton = redSoloAWP;
// pros::Task autonSelect = pros::Task([]() {
// 	while (true) {
// 		master.clearLine(0);

// 		int a = autonSelector.get_value();
// 		if (a > 1776 && a <= 2290) { // 1
// 			auton = skills; //redSoloAWP
// 			master.setText(0, 0, "skills");
// 			pros::lcd::print(7, "skills");
// 			printDebug("skills\n");
// 		} else if (a > 2290 && a <= 2790) { // 2
// 			auton = blueSoloAWP;
// 			master.setText(0, 0, "blueSoloAWP");
// 			pros::lcd::print(7, "blueSoloAWP");
// 			printDebug("blueSoloAWP\n");
// 		} else if (a > 2790 && a <= 3367) { // 3
// 			auton = redRush;
// 			master.setText(0, 0, "redRush");
// 			pros::lcd::print(7, "redRush");
// 			printDebug("redRush\n");
// 		} else if (a > 3367 && a <= 3975) { // 4
// 			auton = blueRush;
// 			master.setText(0, 0, "blueRush");
// 			pros::lcd::print(7, "blueRush");
// 			printDebug("blueRush\n");
// 		} else if (a > 3975 || a <= 47) { // 5
// 			auton = redSoloAWPSig;
// 			master.setText(0, 0, "redSoloAWPSig");
// 			pros::lcd::print(7, "redSoloAWPSig");
// 			printDebug("redSoloAWPSig\n");
// 		} else if (a > 47 && a <= 414) { // 6
// 			auton = blueSoloAWPSig;
// 			master.setText(0, 0, "blueSoloAWPSig");
// 			pros::lcd::print(7, "blueSoloAWPSig");
// 			printDebug("blueSoloAWPSig\n");
// 		} else if (a > 414 && a <= 791) { // 7
// 			auton = redGoalAWP;
// 			master.setText(0, 0, "redGoalAWP");
// 			pros::lcd::print(7, "redGoalAWP");
// 			printDebug("redGoalAWP\n");
// 		} else if (a > 791 && a <= 1160) { // 8
// 			auton = blueRush;
// 			master.setText(0, 0, "blueRush");
// 			pros::lcd::print(7, "blueRush");
// 			printDebug("blueRush\n");
// 		} else if (a > 1160 && a <= 1487) { // 9
// 			auton = redRingAWP;
// 			master.setText(0, 0, "redRingAWP");
// 			pros::lcd::print(7, "redRingAWP");
// 			printDebug("redRingAWP\n");
// 		} else if (a > 1487 && a <= 1776) { // 10
// 			auton = blueRingAWP;
// 			master.setText(0, 0, "blueRingAWP");
// 			pros::lcd::print(7, "blueRingAWP");
// 			printDebug("blueRingAWP\n");
// 		} else {
// 			master.setText(0, 0, "No Auton");
// 			pros::lcd::print(7, "No Auton");
// 			printDebug("No Auton: %d\n", a);
// 		}

// 		pros::delay(100);
// 	}
// }, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Autonomous Selector");

void initialize() {
	pros::lcd::initialize();

	auton.add("Red Solo AWP", redSoloAWP, 1776, 2290);
	auton.add("Blue Solo AWP", blueSoloAWP, 2290, 2790);
	auton.add("Red Rush", redRush, 2790, 3367);
	auton.add("Blue Rush", blueRush, 3367, 3975);
	auton.add("Red Solo AWP Sig", redSoloAWPSig, 3975, 47);
	auton.add("Blue Solo AWP Sig", blueSoloAWPSig, 47, 414);
	auton.add("Red Goal AWP", redGoalAWP, 414, 791);
	auton.add("Blue Goal AWP", blueGoalAWP, 791, 1160);
	auton.add("Red Ring AWP", redRingAWP, 1160, 1487);
	auton.add("Blue Ring AWP", blueRingAWP, 1487, 1776);
	auton.start();

	armEnc.resetZero();
	intakeEnc.resetZero();
	odometry.init();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	uint st = pros::millis();

	auton.run();
	
	printDebug("Auton took %d ms\n", pros::millis() - st);
}

void opcontrol() {
	if (!pto.is_extended()) arm.defaultPos();
	intakeRaiser.extend();
	hang.retract();
  
	okapi::ControllerButton shift(okapi::ControllerDigital::R2);
	okapi::ControllerButton ptoTgl(okapi::ControllerDigital::B);

	okapi::ControllerButton intakeTgl(okapi::ControllerDigital::R1);
	okapi::ControllerButton outtakeTgl(okapi::ControllerDigital::L1);
	okapi::ControllerButton intakeRedirect(okapi::ControllerDigital::Y);
	okapi::ControllerButton targetBlueTgl(okapi::ControllerDigital::X);
	okapi::ControllerButton targetRedTgl(okapi::ControllerDigital::up);

	okapi::ControllerButton intakeHoldTgl(okapi::ControllerDigital::left);

	okapi::ControllerButton clampTgl(okapi::ControllerDigital::L2);
	okapi::ControllerButton doinkerCtrl(okapi::ControllerDigital::right);
	okapi::ControllerButton hangRelease(okapi::ControllerDigital::A);
	okapi::ControllerButton intakeRaiserTgl(okapi::ControllerDigital::down);

	okapi::ControllerButton armIdle(okapi::ControllerDigital::L2);
	okapi::ControllerButton armWallStake(okapi::ControllerDigital::L1);
	okapi::ControllerButton armAllianceStake(okapi::ControllerDigital::right);
	okapi::ControllerButton armDescoreStake(okapi::ControllerDigital::down);

	bool doinkerTgl = false;

	while (true) {
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		chassis.driveTank(left, right);

		if (!shift.isPressed()) {
			if (intakeTgl.changedToPressed()) {
				pros::Task([&] {
					if (!pto.is_extended()) {
						pto.extend();
						arm.move(0);
						pros::delay(500);
					}
					if (intake.getState() == IntakeState::MOGO) intake.stop();
					else intake.mogo();
				});
			} else if (outtakeTgl.changedToPressed()) {
				pros::Task([&] {
					if (!pto.is_extended()) {
						pto.extend();
						arm.move(0);
						pros::delay(500);
					}
					if (intake.getState() == IntakeState::OUTTAKE) intake.stop();
					else intake.outtake();
				});
			} else if (intakeRedirect.changedToPressed()) {
				pros::Task([&] {
					if (!pto.is_extended()) {
						pto.extend();
						arm.move(0);
						pros::delay(500);
					}
					if (intake.getState() == IntakeState::REDIRECT) intake.stop();
					else intake.redirect();
				});
			} else if (intakeHoldTgl.changedToPressed()) {
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

			if (clampTgl.changedToPressed()) clamp.toggle();

			if (doinkerCtrl.changedToPressed()) {
				claw.extend();
				doinkerTgl = !doinkerTgl;
				if (doinkerTgl) doinker.extend();
			}
			if (doinkerCtrl.changedToReleased()) {
				claw.retract();
				if (!doinkerTgl) doinker.retract();
			}

			if (hangRelease.changedToPressed()) {
				arm.defaultPos();
				hang.toggle();
			}

			if (intakeRaiserTgl.changedToPressed()) intakeRaiser.toggle();
		} else {
			if (armIdle.changedToPressed()) {
				pros::Task([&] {
					if (pto.is_extended()) {
						pto.retract();
						intake.stop();
						pros::delay(500);
					}
					arm.defaultPos();
				});
			}
			if (armWallStake.changedToPressed()) {
				pros::Task([&] {
					if (pto.is_extended()) {
						pto.retract();
						intake.stop();
						pros::delay(500);
					}
					arm.wallStake();
				});
			}
			if (armAllianceStake.changedToPressed()) {
				pros::Task([&] {
					if (pto.is_extended()) {
						pto.retract();
						intake.stop();
						pros::delay(500);
					}
					arm.allianceStake();
				});
			}
			if (armDescoreStake.changedToPressed()) {
				pros::Task([&] {
					if (pto.is_extended()) {
						pto.retract();
						intake.stop();
						pros::delay(500);
					}
					arm.descoreStake();
				});
			}
		}
		
		if (ptoTgl.changedToPressed())
			pto.toggle();

		if (targetBlueTgl.changedToPressed())
			intake.setTargetRing(intake.getTargetRing() == RingColour::BLUE ? RingColour::NONE : RingColour::BLUE);
		if (targetRedTgl.changedToPressed())
			intake.setTargetRing(intake.getTargetRing() == RingColour::RED ? RingColour::NONE : RingColour::RED);

		std::string leftDriveTemp = std::to_string(std::max(leftDrive.getTemperature() / 5 - 10, 0.0));
		std::string rightDriveTemp = std::to_string(std::max(rightDrive.getTemperature() / 5 - 10, 0.0));
		std::string intakeFirstTemp = std::to_string(std::max(intakeFirst.getTemperature() / 5 - 10, 0.0));
		std::string intakeSecondTemp = std::to_string(std::max(intakeSecond.getTemperature() / 5 - 10, 0.0));
		master.setText(0, 0, leftDriveTemp + " " + rightDriveTemp + " " + intakeFirstTemp + " " + intakeSecondTemp);

		pros::delay(20);
	}
}