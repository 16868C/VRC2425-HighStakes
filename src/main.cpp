#include "main.h"
#include "robotconfig.hpp"
#include "16868C/controllers/PIDController.hpp"
#include "16868C/subsystems/chassis/motionProfiling.hpp"
#include "16868C/util/util.hpp"
#include "routes.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace lib16868C;

void initialize() {
	pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	int st = pros::millis();

	std::cout << "Auton took " << pros::millis() - st << " ms" << std::endl;
}

void opcontrol() {
	int kickerSpd = 100;
	bool displayKicker = true;
	pros::Task displayController([&] {
		uint st = pros::millis();
		master.clear();
		while (true) {
			std::stringstream l0; l0 << std::fixed << std::setprecision(1) << "t: " << (pros::millis() - st) / 1000.0 << "  B: " << pros::battery::get_capacity();
			master.setText(0, 0, l0.str());
			pros::delay(50);
				
			if (!displayKicker) {
				master.setText(1, 0, "L: " + std::to_string((int) leftDrive.getTemperature()) + "  R: " + std::to_string((int) rightDrive.getTemperature()));
				pros::delay(50);
				master.setText(2, 0, "I: " + std::to_string((int) intake.getTemperature()) + "  K: " + std::to_string((int) kicker.getTemperature()));
				pros::delay(50);
			} else {
				master.setText(2, 0, "v: " + std::to_string((int) kickerSpd) + "  T: " + std::to_string((int) kicker.getTemperature()) + "  ");
				pros::delay(50);
			}
		}
	});

	okapi::ControllerButton intakeTgl(okapi::ControllerDigital::R1);
	okapi::ControllerButton outtakeTgl(okapi::ControllerDigital::R2);
	okapi::ControllerButton kickerTgl(okapi::ControllerDigital::A);

	okapi::ControllerButton increaseKickerSpd(okapi::ControllerDigital::up);
	okapi::ControllerButton decreaseKickerSpd(okapi::ControllerDigital::down);
	okapi::ControllerButton switchDisplay(okapi::ControllerDigital::X);

	int intakeDir = 0;
	bool matchloading = false;
	while (true) {
		// double forward = master.getAnalog(okapi::ControllerAnalog::leftY);
		// double turn = master.getAnalog(okapi::ControllerAnalog::rightX);
		// chassis.driveArcade(forward, turn);
		double left = master.getAnalog(okapi::ControllerAnalog::leftY);
		double right = master.getAnalog(okapi::ControllerAnalog::rightY);
		chassis.driveTank(left, right);

		if (intakeTgl.changedToPressed()) intakeDir = intakeDir == 1 ? 0 : 1;
		if (outtakeTgl.changedToPressed()) intakeDir = intakeDir == -1 ? 0 : -1;
		intake.moveVoltage(intakeDir * 12000);

		if (kickerTgl.changedToPressed()) matchloading = !matchloading;
		kicker.moveVelocity(matchloading * kickerSpd);
		if (increaseKickerSpd.changedToPressed()) if (kickerSpd + 10 <= 200) kickerSpd += 10;
		if (decreaseKickerSpd.changedToPressed()) if (kickerSpd - 10 >= 0) kickerSpd -= 10;

		if (switchDisplay.changedToPressed()) { master.clearLine(1); master.clearLine(2); displayKicker = !displayKicker; }

		pros::delay(50);
	}
}