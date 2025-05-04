#include "main.h"
#include "16868C/subsystems/intake.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "pros/apix.h"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"
#include "routes.hpp"
#include <functional>

using namespace lib16868C;

void initialize() {
	pros::lcd::initialize();

	auton.add(1, "redGoalStake", redGoalStake);
	auton.add(2, "blueGoalStake", blueGoalStake);
	auton.add(3, "redRingStake", redRingStake);
	auton.add(4, "blueRingStake", blueRingStake);
	auton.add(5, "redGoalRush", redGoalRush);
	auton.add(6, "blueGoalRush", blueGoalRush);
	auton.start();

	armEnc.resetZero();
	intakeEnc.resetZero();
	odometry.calibrate();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	uint st = pros::millis();

	auton.run();
	// skills();
	
	printDebug("Auton took %d ms\n", pros::millis() - st);
}

void opcontrol() {
	auton.stop();
	if (auton.getSelectedIdx() == 5)
		intake.setTargetRing(RingColour::RED);

	intakeRaiser.extend();
	arm.defaultPos();

	// odometry.init();

	// chassis.moveDistance(48_in, 0_deg, 0, {.distGains={0.064, 0, 0.004}, .headingGains={0.5, 0, 0.01}});

	// chassis.moveToPoint({24_in, 48_in}, 0, {.distGains={0.064, 0, 0.004}, .headingGains={0.5, 0, 0.01}});

	// chassis.turnAbsolute(90_deg, 0, {.gains={1.1, 0.2, 0.09}}); // 180 deg: 1.1, 0.1, 0.1

	// chassis.turnAbsolute(90_deg, 0, {.gains={1.25, 0.1, 0.12}, .turnWheel=TurnWheel::RIGHT});

	// for (int i = 1; i <= 4; i++) {
	// 	double t, a, v;
	// 	for (int j = 0; j < 10; j++) {
	// 		chassis.turnAbsolute(inertial.get_rotation(AngleUnit::DEG) * okapi::degree + i * 45_deg, 0, {.gains={1.2, 0.2, 0.1}});
	// 		a += std::abs(chassis.a - a);
	// 		v += chassis.v;
	// 		t += chassis.t;
	// 		pros::delay(500);
	// 	}
	// 	std::cout << a / 10 << " " << v / 10 << " " << t / 10 << "\n"; 
	// }
  
	okapi::ControllerButton shift(okapi::ControllerDigital::R2);

	okapi::ControllerButton intakeTgl(okapi::ControllerDigital::R1);
	okapi::ControllerButton outtakeTgl(okapi::ControllerDigital::L1);
	okapi::ControllerButton intakeHoldTgl(okapi::ControllerDigital::left);
	okapi::ControllerButton targetRingTgl(okapi::ControllerDigital::X);

	okapi::ControllerButton redirectTgl(okapi::ControllerDigital::Y);

	okapi::ControllerButton clampTgl(okapi::ControllerDigital::L2);
	okapi::ControllerButton leftDoinkerTgl(okapi::ControllerDigital::down);
	okapi::ControllerButton rightDoinkerTgl(okapi::ControllerDigital::B);
	okapi::ControllerButton intakeRaiserTgl(okapi::ControllerDigital::right);

	okapi::ControllerButton armIdle(okapi::ControllerDigital::L2);
	okapi::ControllerButton armWallStake(okapi::ControllerDigital::L1);
	okapi::ControllerButton armAllianceStake(okapi::ControllerDigital::right);
	okapi::ControllerButton armHang(okapi::ControllerDigital::A);

	bool doinkerTgl = false;
	int i = 0;

	while (true) {
		// double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		// double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		// chassis.driveTank(left, right);
		double fwd = master.getAnalog(okapi::ControllerAnalog::leftY);
		double turn = master.getAnalog(okapi::ControllerAnalog::rightX);
		chassis.driveArcade(fwd, turn);

		if (!shift.isPressed()) {
			if (intakeTgl.changedToPressed()) {
				if (intake.getState() == IntakeState::INTAKING) intake.stop();
				else intake.intake();
			} else if (outtakeTgl.changedToPressed()) {
				if (intake.getState() == IntakeState::OUTTAKING) intake.stop();
				else intake.outtake();
			} else if (intakeHoldTgl.changedToPressed()) {
				if (intake.getState() == IntakeState::HOLDING) intake.stop();
				else intake.hold();
			}

			if (redirectTgl.changedToPressed()) {
				intake.setColourFilter(false);
				switch (arm.getState()) {
					case ArmPosition::LOAD:
						pros::Task([&] {
							intake.outtake();
							pros::delay(50);
							intake.stop();
							arm.load2();
						});
						break;
					default:
						arm.load();
						break;
				}
			}

			if (clampTgl.changedToPressed()) clamp.toggle();

			if (leftDoinkerTgl.changedToPressed()) leftDoinker.toggle();
			if (rightDoinkerTgl.changedToPressed()) rightDoinker.toggle();

			if (armHang.changedToPressed()) arm.hang();

			if (okapi::ControllerButton(okapi::ControllerDigital::A).changedToPressed()) {
				// std::cout << autonSelector.get_value() << "\n";
				// std::cout << intakeEnc.get() << "\n";
				// std::cout << vertEnc.getDist() << " " << inertial.get_rotation(AngleUnit::RAD) << " " << vertEnc.getDist() / inertial.get_rotation(AngleUnit::RAD) << "\n";
				// std::cout << hortEnc.getDist() << " " << inertial.get_rotation(AngleUnit::RAD) << " " << hortEnc.getDist() / inertial.get_rotation(AngleUnit::RAD) << "\n";
			}

			if (intakeRaiserTgl.changedToPressed()) intakeRaiser.toggle();
		} else {
			if (armIdle.changedToPressed()) {
				arm.defaultPos();
				intake.setColourFilter(true);
			}
			if (armWallStake.changedToPressed()) {
				intake.setColourFilter(true);
				pros::Task([&] {
					intake.outtake();
					pros::delay(50);
					intake.stop();
					arm.wallStake();
				});
			}
			if (armAllianceStake.changedToPressed()) {
				intake.setColourFilter(true);
				arm.allianceStake();
			}
		}

		if (targetRingTgl.changedToPressed())
			intake.setTargetRing(intake.getTargetRing() == RingColour::BLUE ? RingColour::RED : RingColour::BLUE);

		std::string leftDriveTemp = std::to_string((int) std::max(leftDrive.getTemperature() / 5 - 7, 0.0));
		std::string rightDriveTemp = std::to_string((int) std::max(rightDrive.getTemperature() / 5 - 7, 0.0));
		std::string intakeTemp = std::to_string((int) std::max(intakeMtr.getTemperature() / 5 - 7, 0.0));
		std::string armLeftTemp = std::to_string((int) std::max(armLeft.getTemperature() / 5 - 7, 0.0));
		std::string armRightTemp = std::to_string((int) std::max(armRight.getTemperature() / 5 - 7, 0.0));
		master.setText(0, 0, leftDriveTemp + " " + rightDriveTemp + " " + intakeTemp + " " + armLeftTemp + " " + armRightTemp + " " + (intake.getTargetRing() == RingColour::BLUE ? "B" : "R"));
		
		pros::delay(20);
	}
}