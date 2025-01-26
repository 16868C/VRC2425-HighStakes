#include "main.h"
#include "16868C/subsystems/intake.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"
#include "routes.hpp"
#include <functional>

using namespace lib16868C;

void initialize() {
	pros::lcd::initialize();

	auton.add(1, "redGoalAWP", redGoalAWP);
	auton.add(2, "blueGoalAWP", blueGoalAWP);
	auton.add(3, "redRingAWP", redRingAWP);
	auton.add(4, "blueRingAWP", blueRingAWP);
	auton.add(5, "Skills", skills);
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
	auton.stop();
	if (auton.getSelectedIdx() == 5)
		intake.setTargetRing(RingColour::RED);

	if (!pto.is_extended()) arm.defaultPos();
	intakeRaiser.extend();
	hang.retract();
  
	okapi::ControllerButton shift(okapi::ControllerDigital::R2);
	okapi::ControllerButton ptoTgl(okapi::ControllerDigital::B);

	okapi::ControllerButton intakeTgl(okapi::ControllerDigital::R1);
	okapi::ControllerButton outtakeTgl(okapi::ControllerDigital::L1);
	okapi::ControllerButton intakeRedirect(okapi::ControllerDigital::Y);
	okapi::ControllerButton targetRingTgl(okapi::ControllerDigital::X);

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
	int i = 0;

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

		if (targetRingTgl.changedToPressed())
			intake.setTargetRing(intake.getTargetRing() == RingColour::BLUE ? RingColour::RED : RingColour::BLUE);

		std::string leftDriveTemp = std::to_string((int) std::max(leftDrive.getTemperature() / 5 - 7, 0.0));
		std::string rightDriveTemp = std::to_string((int) std::max(rightDrive.getTemperature() / 5 - 7, 0.0));
		std::string intakeFirstTemp = std::to_string((int) std::max(intakeFirst.getTemperature() / 5 - 7, 0.0));
		std::string intakeSecondTemp = std::to_string((int) std::max(intakeSecond.getTemperature() / 5 - 7, 0.0));
		master.setText(0, 0, leftDriveTemp + " " + rightDriveTemp + " " + intakeFirstTemp + " " + intakeSecondTemp + " " + (intake.getTargetRing() == RingColour::BLUE ? "B" : "R"));

		pros::delay(20);
	}
}