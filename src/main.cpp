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
		if (a > 1776 && a <= 2290) { // 1
			auton = skills; //redSoloAWP
			master.setText(0, 0, "skills");
			pros::lcd::print(7, "skills");
			printDebug("skills\n");
		} else if (a > 2290 && a <= 2790) { // 2
			auton = blueSoloAWP;
			master.setText(0, 0, "blueSoloAWP");
			pros::lcd::print(7, "blueSoloAWP");
			printDebug("blueSoloAWP\n");
		} else if (a > 2790 && a <= 3367) { // 3
			auton = redRush;
			master.setText(0, 0, "redRush");
			pros::lcd::print(7, "redRush");
			printDebug("redRush\n");
		} else if (a > 3367 && a <= 3975) { // 4
			auton = blueRush;
			master.setText(0, 0, "blueRush");
			pros::lcd::print(7, "blueRush");
			printDebug("blueRush\n");
		} else if (a > 3975 || a <= 47) { // 5
			auton = redSoloAWPSig;
			master.setText(0, 0, "redSoloAWPSig");
			pros::lcd::print(7, "redSoloAWPSig");
			printDebug("redSoloAWPSig\n");
		} else if (a > 47 && a <= 414) { // 6
			auton = blueSoloAWPSig;
			master.setText(0, 0, "blueSoloAWPSig");
			pros::lcd::print(7, "blueSoloAWPSig");
			printDebug("blueSoloAWPSig\n");
		} else if (a > 414 && a <= 791) { // 7
			auton = redGoalAWP;
			master.setText(0, 0, "redGoalAWP");
			pros::lcd::print(7, "redGoalAWP");
			printDebug("redGoalAWP\n");
		} else if (a > 791 && a <= 1160) { // 8
			auton = blueRush;
			master.setText(0, 0, "blueRush");
			pros::lcd::print(7, "blueRush");
			printDebug("blueRush\n");
		} else if (a > 1160 && a <= 1487) { // 9
			auton = redRingAWP;
			master.setText(0, 0, "redRingAWP");
			pros::lcd::print(7, "redRingAWP");
			printDebug("redRingAWP\n");
		} else if (a > 1487 && a <= 1776) { // 10
			auton = blueRingAWP;
			master.setText(0, 0, "blueRingAWP");
			pros::lcd::print(7, "blueRingAWP");
			printDebug("blueRingAWP\n");
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
	autonSelect.suspend();
	uint st = pros::millis();
	// clamp.toggle();
	// pros::delay(500);
	// chassis.turnAbsolute(3600_deg, 0, {.maxRPM=300_rpm, .gains={0.39, 0.01, 2}, .slewRate=6000});
	// chassis.moveDistance(48_in, 0_deg, 0, {.distGains={0.04, 0, 1.5}, .slewRate=6000});
	// chassis.moveToPoint({48_in, 48_in}, 0, {.distGains={0.041, 0, 1.5}, .headingGains={0.39, 0, 2}, .reverse=true, .slewRate=6000});
	// chassis.moveToPose({24_in, 48_in, 0_deg}, 0, {.distGains={0.04, 0, 1.5}, .headingGains={0.45, 0, 2}, .dlead=0.8, .slewRate=6000});
	// arm.resetPosition();

	// chassis.moveToPoint({24_in, 24_in}, 0, {.distGains={0.04, 0, 3}, .headingGains={0.5, 0, 1.5}, .exitRadius=1_in});
	// chassis.moveToPose({24_in, 24_in, 90_deg}, 0, {.distGains={0.2, 0, 3}, .headingGains={0.5, 0, 1.5}, .settleRadius=7.5_in, .horiDrift=0.2, .dlead=0.6, .glead=0.4});

	// chassis.moveToPoint({48_in, 0_in}, 0, {.minRPM=300_rpm, .distGains={0.035, 0, 3}, .headingGains={0.6, 0, 1}, .exitRadius=5_in});
	// chassis.moveToPoint({72_in, 24_in}, 0, {.minRPM=300_rpm, .distGains={.kP=0.035, .kD=3}, .headingGains={.kP=0.6, .kD=1}, .exitRadius=5_in, .turnDeadzone=3_in});
	// chassis.moveToPoint({96_in, 0_in}, 0, {.minRPM=300_rpm, .distGains={.kP=0.035, .kD=3}, .headingGains={.kP=0.6, .kD=1}, .exitRadius=5_in, .turnDeadzone=3_in});
	// chassis.turnAbsolute(90_deg, 0, {.minRPM=200_rpm, .gains={0.03, 0, 3}, .errorMargin=10_deg, .numInMargin=1});
	// chassis.moveToPoint({96_in, 24_in}, 0, {.distGains={.kP=0.07, .kD=3}, .headingGains={.kP=0.6, .kD=1}, .exitRadius=1_in});
	// chassis.moveToPose({72_in, 72_in, 180_deg}, 0, {.distGains={0.05, 0, 3}, .headingGains={0.5, 0, 1}, .settleRadius=7.5_in, .horiDrift=1.5*52*52, .dlead=0.6, .glead=0.5, .slewRate=2000});
	// chassis.turnAbsolute(90_deg, 0, {.gains={.kP=0.04, .kD=3}, .turnWheel=TurnWheel::BOTH});

	// chassis.moveToPose({24_in, 24_in, 90_deg}, 0, {.maxRPM=600_rpm, .distGains={.kP=0.05, .kD=3}, .headingGains={.kP=0.5, .kD=1}, .settleRadius=7.5_in, .horiDrift=1.5 * 52 * 52, .dlead=0.6, .glead=0});

	// pros::Task odomMonitor([&] {
	// 	uint32_t t = pros::millis();
	// 	while (true) {
	// 		master.setText(0, 0, odometry.getPose().toStr());
	// 		pros::Task::delay_until(&t, 50);
	// 	}
	// });
	auton();
	// redRightAWP();
	// redSoloAWP();
	// blueSoloAWP();

	// chassis.moveToPoint({})

	// chassis.moveDistance(48_in, 600_rpm, {0.04, 0, 1.5}, 0_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	// chassis.turnAbsolute(90_deg, 600_rpm, {0.02, 0, 0.95}, 3, 5, TurnWheel::BOTH, 0);
	// chassis.moveToPoint({24_in, 24_in}, 600_rpm, {0.04, 0, 1.5}, {1, 0, 1}, 3_in, false, true, 0);

	// chassis.moveToPoint({60_in, -56_in}, 600_rpm, {0.06, 0, 0.1}, {0.8, 0, 1}, 3_in, false, false, 0);
	// chassis.moveToPoint({48_in, -72_in}, 600_rpm, {0.06, 0, 0.1}, {0.8, 0, 1}, 3_in, false, true, 0);
	// pros::delay(1000);
	// chassis.moveToPoint({60_in, -45_in}, 600_rpm, {0.05, 0, 0.1}, {0.8, 0, 1}, 3_in, true, false, 0);
	// chassis.moveToPoint({0_in, 0_in}, 600_rpm, {0.06, 0, 0.1}, {0.8, 0, 1}, 3_in, true, true, 0);
	
	printDebug("Auton took %d ms\n", pros::millis() - st);
}

void opcontrol() {
	autonSelect.suspend();
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
	okapi::ControllerButton doinkerTgl(okapi::ControllerDigital::right);
	okapi::ControllerButton hangRelease(okapi::ControllerDigital::A);
	okapi::ControllerButton intakeRaiserTgl(okapi::ControllerDigital::down);

	okapi::ControllerButton armIdle(okapi::ControllerDigital::L2);
	okapi::ControllerButton armWallStake(okapi::ControllerDigital::L1);
	okapi::ControllerButton armAllianceStake(okapi::ControllerDigital::right);
	okapi::ControllerButton armDescoreStake(okapi::ControllerDigital::down);

	while (true) {
		// Drivetrain -> Tank Drive
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		chassis.driveTank(left, right);
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

		if (!shift.isPressed() && doinkerTgl.changedToPressed()) {
			if (doinker.is_extended()) claw.retract();
			else doinker.extend();
		}
		if (!shift.isPressed() && doinkerTgl.changedToReleased()) {
			if (doinker.is_extended()) doinker.retract();
			else claw.extend();
		}

		if (!shift.isPressed() && hangRelease.changedToPressed()) {
			arm.defaultPos();
			hang.toggle();
			std::cout << intakeEnc.get() / 10 << "\n";
		}
		if (!shift.isPressed() && intakeRaiserTgl.changedToPressed()) {
			intakeRaiser.toggle();
		}

		int leftDriveTemp = std::max(leftDrive.getTemperature() / 5 - 10, 0.0);
		int rightDriveTemp = std::max(rightDrive.getTemperature() / 5 - 10, 0.0);
		int intakeFirstTemp = std::max(intakeFirst.getTemperature() / 5 - 10, 0.0);
		int intakeSecondTemp = std::max(intakeSecond.getTemperature() / 5 - 10, 0.0);
		master.setText(0, 0, std::to_string(leftDriveTemp) + " " + std::to_string(rightDriveTemp) + " " + std::to_string(intakeFirstTemp) + " " + std::to_string(intakeSecondTemp));

		pros::delay(20);
	}
}