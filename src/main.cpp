#include "main.h"
#include "robotconfig.hpp"
#include "16868C/controllers/PIDController.hpp"
#include "16868C/subsystems/chassis/motionProfiling.hpp"
#include "16868C/util/logger.hpp"
#include "16868C/util/util.hpp"
#include "routes.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace lib16868C;

#define SKILLS
#define ANSON
// #define WINSTON

void initialize() {
	pros::lcd::initialize();

	// odometry.init();
	inertial.reset(true);
	chassis.coast();
}

void disabled() {
	horiHang.retract();
}

void competition_initialize() {}

void autonomous() {
	uint st = pros::millis();

	#ifndef SKILLS
	// nearAWPBar();
	// farAWP();
	// farAWPBar();
	// nearRush();
	// nearDisrupt();
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
	okapi::ControllerButton vertWingsTgl(okapi::ControllerDigital::Y);
	okapi::ControllerButton horiHangTgl(okapi::ControllerDigital::X);

	#ifdef SKILLS
	okapi::ControllerButton skillsMacro(okapi::ControllerDigital::up);
	#endif

	int intakeDir = 0;
	int kickerSpd = 110;
	#ifndef SKILLS
	bool matchloading = false;
	#endif
	#ifdef SKILLS
	bool matchloading = true;
	#endif
	while (true) {
		#ifdef WINSTON
		// double forward = master.getAnalog(okapi::ControllerAnalog::leftY);
		// double turn = master.getAnalog(okapi::ControllerAnalog::rightX);
		// chassis.driveArcade(forward, turn);
		#endif
		#ifdef ANSON
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		chassis.driveTank(left, right);
		#endif

		#ifdef WINSTON
		if (intakeTgl.isPressed()) intakeDir = 1;
		else if (outtakeTgl.isPressed()) intakeDir = -1;
		else intakeDir = 0;
		#endif
		#ifdef ANSON
		if (intakeTgl.changedToPressed()) intakeDir = intakeDir == 1 ? 0 : 1;
		else if (outtakeTgl.changedToPressed()) intakeDir = intakeDir == -1 ? 0 : -1;
		intake.moveVoltage(intakeDir * 12000);
		#endif

		if (kickerTgl.changedToPressed()) matchloading = !matchloading;
		kicker.move(matchloading * kickerSpd);
		
		if (leftWingTgl.changedToPressed()) leftWing.toggle();
		if (rightWingTgl.changedToPressed()) rightWing.toggle();
		if (frontWingsTgl.changedToPressed()) { leftWing.toggle(); rightWing.toggle(); }
		if (vertWingsTgl.changedToPressed()) vertWings.toggle();

		if (horiHangTgl.changedToPressed()) horiHang.toggle();

		#ifdef SKILLS
		if (skillsMacro.changedToPressed()) {
			matchloading = false;
			intakeDir = -1;
		}
		#endif

		pros::delay(50);
	}
}