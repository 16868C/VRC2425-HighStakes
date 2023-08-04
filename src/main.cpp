#include "main.h"
#include "robotconfig.hpp"
#include "16868Z/controllers/PIDController.hpp"
#include "16868Z/subsystems/chassis/motionProfiling.hpp"
#include "16868Z/util/util.hpp"
#include "routes.hpp"
#include <fstream>

using namespace lib16868Z;

void initialize() {
	pros::lcd::initialize();
	
	bool inertialResetFailed = true, inertialDrift = false;
	do {
		if (!inertialResetFailed) {
			std::cerr << "Inertial Reset Failed" << std::endl;
			master.setText(0, 0, "Inertial Reset Failed");
			pros::lcd::print(0, "Inertial Reset Failed");
			master.rumble("-");
		}
		inertialResetFailed = inertial.reset(true) - 1;
	} while (inertialResetFailed);
	double h1 = inertial.get_rotation();
	pros::delay(300);
	double h2 = inertial.get_rotation();
	if (std::abs(h1 - h2) > 0.5) {
		inertialDrift = true;
		std::cerr << "Inertial Drift Detected: " << std::abs(h1 - h2) << " deg difference in 300ms" << std::endl;
		master.setText(0, 0, "Inertial Drift Detected: " + std::to_string(std::abs(h1 - h2)) + " deg difference in 300ms");
		pros::lcd::print(0, "Inertial Drift Detected: %f deg difference in 300ms", std::abs(h1 - h2));
		master.rumble("-");
	}

	leftDrive.tarePosition();
	rightDrive.tarePosition();
	turretMotor.tarePosition();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	int st = pros::millis();
	
	// goalSide();
	goalSideBar();
	// matchloadAWP();

	std::cout << "Auton took " << pros::millis() - st << " ms" << std::endl;
}

void opcontrol() {
	okapi::ControllerButton intakeTgl(okapi::ControllerDigital::R1);
	okapi::ControllerButton outtakeTgl(okapi::ControllerDigital::R2);
	okapi::ControllerButton shootTgl(okapi::ControllerDigital::L1);
	okapi::ControllerButton matchloadTgl(okapi::ControllerDigital::L2);

	okapi::ControllerButton wingsTgl(okapi::ControllerDigital::B);
	okapi::ControllerButton mouthTgl(okapi::ControllerDigital::X);
	okapi::ControllerButton clotheslineTgl(okapi::ControllerDigital::up);
	okapi::ControllerButton turretShifterTgl(okapi::ControllerDigital::down);

	while (true) {
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		chassis.driveTank(left, right);

		// double forward = master.getAnalog(okapi::ControllerAnalog::leftY);
		// double turn = master.getAnalog(okapi::ControllerAnalog::rightX);
		// chassis.driveArcade(forward, turn);

		if (matchloadTgl.changedToPressed()) {
			if (intake.getState() == IntakeState::MATCHLOAD) intake.stop();
			else intake.matchload();
		} else if (intakeTgl.changedToPressed()) {
			if (intake.getState() == IntakeState::INTAKE) intake.stop();
			else intake.intake(false);
		} else if (outtakeTgl.changedToPressed()) {
			if (intake.getState() == IntakeState::OUTTAKE) intake.stop();
			else intake.outtake(true, 1000, false);
		} else if (shootTgl.changedToPressed()) {
			if (intake.getState() == IntakeState::SHOOT) intake.stop();
			else intake.shoot();
		}

		if (wingsTgl.changedToPressed()) wings.toggle();
		if (mouthTgl.changedToPressed()) mouth.toggle();
		if (clotheslineTgl.changedToPressed()) clothesline.toggle();
		if (turretShifterTgl.changedToPressed()) turretShifter.toggle();

		// if (turretLeft.isPressed()) turret.spin(-150);
		// else if (turretRight.isPressed()) turret.spin(150);
		// else turret.spin(0);
		// std::cout << frontIntake.getVoltage() << " " << frontIntake.getActualVelocity() << " " << rearIntake.getVoltage() << " " << rearIntake.getActualVelocity() << "\n";

		pros::lcd::print(0, "Left Drive: %f", leftDrive.getTemperature());
		pros::lcd::print(1, "Right Drive: %f", rightDrive.getTemperature());
		pros::lcd::print(2, "Front Intake: %f", frontIntake.getTemperature());
		pros::lcd::print(3, "Rear Intake: %f", rearIntake.getTemperature());
		pros::lcd::print(4, "Turret: %f", turretMotor.getTemperature());

		if (leftDrive.isOverTemp() || rightDrive.isOverTemp() || frontIntake.isOverTemp() || rearIntake.isOverTemp() || turretMotor.isOverTemp()) master.rumble("-");

		pros::delay(20);
	}}