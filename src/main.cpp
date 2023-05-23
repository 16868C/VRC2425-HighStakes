#include "main.h"
#include "robotconfig.hpp"
#include "16868X/controllers/PIDController.hpp"
#include "16868X/devices/motor.hpp"
#include "16868X/devices/motorGroup.hpp"
#include "16868X/subsystems/chassis/motionProfiling.hpp"
#include "16868X/util/filters/emaFilter.hpp"
#include "16868X/util/filters/medianFilter.hpp"
#include "16868X/util/filters/rangeExtremaFilter.hpp"
#include "16868X/util/filters/smaFilter.hpp"
#include "16868X/util/util.hpp"

using namespace lib16868X;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

void moveDistanceStraight(double dist, double maxRPM) {
	double currDist = 0;
	while (dist - currDist > 0) {
		double avgTicks = abs(leftDrive.getPosition() + rightDrive.getPosition()) / 2.0;
		currDist = avgTicks / 900.0 * WHEEL_DIAMETER * M_PI;
		leftDrive.moveVoltage(maxRPM / 200 * 12000);
		rightDrive.moveVoltage(maxRPM / 200 * 12000);
		pros::delay(20);
	}
	
	leftDrive.moveVoltage(0);
	rightDrive.moveVoltage(0);
}

void moveDistancePID(double dist, double maxRPM, double heading, double turnRPM, double accel, PIDGains decelGains, PIDGains headingGains) {
	MotionLimit motionLimit = {maxRPM * WHEEL_DIAMETER * M_PI / 60, accel};
	MotionProfile profile = MotionProfiling::generateTrapezoidal(dist, motionLimit);

	PIDController distPID(decelGains, dist);
	PIDController headingPID(headingGains, heading);

	double currDist = 0;
	int i = 0;
	uint32_t time = pros::millis(), st = pros::millis(), t = pros::millis();
	int dir = Util::sgn(dist);
	while (std::abs(dist) - currDist > 0.1) {
		t = pros::millis();
		double headingCtrl = headingPID.calculate(inertial.get_rotation());
		double avgTicks = abs(leftDrive.getPosition() + rightDrive.getPosition()) / 2.0;
		currDist = avgTicks / 900.0 * WHEEL_DIAMETER * M_PI;
		double distCtrl = distPID.calculate(currDist);

		double accelRPM = (t - st <= profile.accelTime ? profile[i++].velocity * 60 / (M_PI * WHEEL_DIAMETER) : 600);
		double decelRPM = maxRPM * distCtrl;
		double vel = std::min(maxRPM, std::min(accelRPM, decelRPM));
		leftDrive.moveVoltage((dir * vel + turnRPM * headingCtrl) / 200 * 12000);
		rightDrive.moveVoltage((dir * vel - turnRPM * headingCtrl) / 200 * 12000);

		pros::Task::delay_until(&time, 20);
	}

	leftDrive.moveVoltage(0);
	rightDrive.moveVoltage(0);
}

void moveDistanceProfiledPID(double dist, MotionLimit motionLimit, double kV, double kA, double kP) {
	MotionProfile profile = MotionProfiling::generateTrapezoidal(dist, motionLimit);

	for (auto& data : profile.profile) {
		leftDrive.moveVoltage(data.velocity * kV + data.acceleration * kA + (data.velocity - leftDrive.getActualVelocity()) * kP);
		rightDrive.moveVoltage(data.velocity * kV + data.acceleration * kA + (data.velocity - rightDrive.getActualVelocity()) * kP);
	}

	leftDrive.moveVoltage(0);
	rightDrive.moveVoltage(0);
}

void moveDistanceHalfLife(double dist, double maxRPM, double decelZone) {
	double currDist = 0;
	double prevVel = maxRPM;
	while (dist - currDist > 0) {
		double avgTicks = abs(leftDrive.getPosition() + rightDrive.getPosition()) / 2.0;
		currDist = avgTicks / 900.0 * WHEEL_DIAMETER * M_PI;
		double vel = dist - currDist > decelZone ? maxRPM : prevVel / 2.0;
		prevVel = vel;
		leftDrive.moveVoltage(vel / 200 * 12000);
		rightDrive.moveVoltage(vel / 200 * 12000);
		pros::delay(20);
	}

	leftDrive.moveVoltage(0);
	rightDrive.moveVoltage(0);
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	okapi::Motor mtr(12, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
	mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	okapi::ControllerButton scooper(okapi::ControllerDigital::A);
	while (true) {
		if (scooper.changedToPressed()) {
			pros::Task scoop([&] {
				mtr.moveVoltage(12000);
				while (mtr.getPosition() < 500) {
					pros::delay(20);
				}
				mtr.moveVoltage(0);
			});
		}

		pros::delay(20);
	}
}
