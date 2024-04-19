#include "main.h"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"
#include "routes.hpp"

using namespace lib16868C;

// #define SKILLS
// #define ANSON
#define WINSTON

void initialize() {
	pros::lcd::initialize();

	// odometry.init();
	inertial.reset(true);
	chassis.coast();
}

void disabled() {
	
}

void competition_initialize() {}

void autonomous() {
	uint st = pros::millis();

	// chassis.turnAbsolute(90_deg, 600_rpm, {0.09, 0, 9}, 5, 3, 5, TurnWheel::BOTH, 0);
	// chassis.moveDistance(-72_in, 600_rpm, {0.07, 0, 4}, 2400, 0_deg, 300_rpm, {0.035, 0, 0.6}, 0);

	// pros::delay(5000);
	// double avgTicks = std::abs((leftDrive.getPosition() + rightDrive.getPosition()) / 2.0);
	// double currDist = avgTicks / 300.0 * (WHEEL_DIAM * okapi::pi).convert(okapi::inch) * GEAR_RATIO;
	// printDebug("[Inline Move Distance] Finished with distance of %f\" with a heading of %f deg, taking %d ms\n", currDist, inertial.get_rotation(AngleUnit::DEG), pros::millis() - st);


	#ifndef SKILLS
	// nearAWPBar();
	farAWP();
	// farAWPBar();
	// nearRush();
	// nearDisrupt();
	// nearBowlAWP();
	// nearBowlRush();
	#endif
	#ifdef SKILLS
	skills2();
	#endif

	printDebug("Auton took %d ms\n", pros::millis() - st);
}

void opcontrol() {
	#ifdef SKILLS
	skillsStart();
	#endif

	okapi::ControllerButton intakeTgl(okapi::ControllerDigital::R1);
	okapi::ControllerButton outtakeTgl(okapi::ControllerDigital::L1);
	okapi::ControllerButton kickerTgl(okapi::ControllerDigital::left);
	okapi::ControllerButton leftWingTgl(okapi::ControllerDigital::L2);
	okapi::ControllerButton rightWingTgl(okapi::ControllerDigital::R2);
	okapi::ControllerButton frontWingsTgl(okapi::ControllerDigital::A);
	okapi::ControllerButton winchPtoTgl(okapi::ControllerDigital::Y);
	okapi::ControllerButton horiHangTgl(okapi::ControllerDigital::X);
	okapi::ControllerButton intakeRaiserTgl(okapi::ControllerDigital::B);
	okapi::ControllerButton switchDrive(okapi::ControllerDigital::right);
	okapi::ControllerButton intakeSlow(okapi::ControllerDigital::left);

	#ifdef SKILLS
	okapi::ControllerButton skillsMacro(okapi::ControllerDigital::up);
	#endif

	double intakeDir = 0;
	int kickerSpd = 200;
	bool drive = false;
	#ifndef SKILLS
	bool matchloading = false;
	#endif
	#ifdef SKILLS
	bool matchloading = true;
	#endif
	while (true) {
		if (drive) {
			// Drivetrain -> Split Arcade
		double forward = master.getAnalog(okapi::ControllerAnalog::leftY);
		double turn = master.getAnalog(okapi::ControllerAnalog::rightX);
		chassis.driveArcade(forward, turn);
		} else {
			// Drivetrain -> Tank Drive
			double left = master.getAnalog(okapi::ControllerAnalog::leftY);
			double right = master.getAnalog(okapi::ControllerAnalog::rightY);
			chassis.driveTank(left, right);
		}
		std::cout << leftDrive.getTemperature() << " " << rightDrive.getTemperature() << "\n";

		#ifdef WINSTON
		// Intake: L1 -> Intake, R1 -> Outtake, Release to stop
		if (intakeTgl.isPressed()) intakeDir = 1;
		else if (outtakeTgl.isPressed()) intakeDir = -1;
		else if (intakeSlow.isPressed()) intakeDir = -0.5;
		else intakeDir = 0;
		#endif
		#ifdef ANSON
		// Intake: L1 -> Intake, R1 -> Outtake, Press again to stop
		if (intakeTgl.changedToPressed()) intakeDir = intakeDir == 1 ? 0 : 1;
		else if (outtakeTgl.changedToPressed()) intakeDir = intakeDir == -1 ? 0 : -1;
		#endif
		intake.moveVoltage(intakeDir * 12000);
		
		// Wings: L2 -> Left wing (Toggle), R2 -> Right wing (Toggle), A -> Front wings (Toggle), Y -> Vertical wings (Toggle)
		if (leftWingTgl.changedToPressed()) leftWing.toggle();
		if (rightWingTgl.changedToPressed()) rightWing.toggle();
		if (frontWingsTgl.changedToPressed()) { leftWing.toggle(); rightWing.toggle(); }
		
		// if (winchPtoTgl.changedToPressed()) winchPTO.toggle();

		if (intakeRaiserTgl.changedToPressed()) intakeRaiser.toggle();

		// // Horizontal Hang: X (Toggle)
		// if (horiHangTgl.changedToPressed()) horiHang.toggle();

		if (switchDrive.changedToPressed()) drive = !drive;

		// Skill macro: Up arrow -> Stop matchloading, start outtaking
		#ifdef SKILLS
		if (skillsMacro.changedToPressed()) {
			matchloading = false;
			intakeDir = -1;
		}
		#endif

		pros::delay(50);
	}
}