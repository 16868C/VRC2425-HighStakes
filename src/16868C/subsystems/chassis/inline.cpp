#include "inline.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "16868C/subsystems/chassis/motionProfiling.hpp"
#include "16868C/util/logger.hpp"
#include "16868C/util/math.hpp"
#include "16868C/util/util.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <algorithm>

using namespace lib16868C;

Inline::Inline(MotorGroup& leftMtrs, MotorGroup& rightMtrs, Inertial* inertial, Odometry* odom, okapi::QLength wheelDiam, double gearRatio)
	: leftMtrs(leftMtrs), rightMtrs(rightMtrs), inertial(inertial), odom(odom), wheelDiam(wheelDiam), gearRatio(gearRatio) {

	tpr = leftMtrs.getGearing() == okapi::AbstractMotor::gearset::red ? 1800 : 
			leftMtrs.getGearing() == okapi::AbstractMotor::gearset::green ? 900 :
																			300;
	
	leftMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	rightMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
}

void Inline::moveTank(double left, double right, double slewRate) {
	if (slewRate > 0 && std::abs(left) - std::abs(prevLeft) > slewRate) left = prevLeft + slewRate * Util::sgn(left);
	if (slewRate > 0 && std::abs(right) - std::abs(prevRight) > slewRate) right = prevRight + slewRate * Util::sgn(right);
	prevLeft = left, prevRight = right;

	double max = std::max(std::abs(left), std::abs(right));
	if (max > MAX_VOLT) {
		left *= MAX_VOLT / max;
		right *= MAX_VOLT / max;
	}

	leftMtrs.moveVoltage(left);
	rightMtrs.moveVoltage(right);
}

void Inline::moveArcade(double forward, double turn, double slewRate) {
	double left = forward + turn;
	double right = forward - turn;

	moveTank(left, right, slewRate);
}

void Inline::driveTank(double left, double right, double deadzone) {
	if (std::abs(left) < deadzone) left = 0;
	if (std::abs(right) < deadzone) right = 0;
	moveTank(left * 12000, right * 12000);
}

void Inline::driveArcade(double forward, double turn, double deadzone) {
	if (std::abs(forward) < deadzone) forward = 0;
	if (std::abs(turn) < deadzone) turn = 0;
	moveArcade(forward * 12000, turn * 12000);
}

void Inline::moveDistance(okapi::QLength dist, okapi::QAngularSpeed maxRPM, PIDGains distGains, okapi::QAngle heading, okapi::QAngularSpeed turnRPM, PIDGains headingGains, int timeout) {
	leftMtrs.tarePosition();
	rightMtrs.tarePosition();

	PIDController distPID(distGains, 0, 0);
	PIDController headingPID(headingGains, 1, -1);

	double currDist = 0;

	int dir = Util::sgn(dist.convert(okapi::inch));

	int i = 0;
	uint st = pros::millis();
	while (dist.abs().convert(okapi::inch) - currDist > 0) {
		// Timeout
		if (pros::millis() - st > timeout && timeout > 0) {
			printError("[Inline::MoveDistance] Timeout: %d\n", pros::millis() - st);
			break;
		}

		// Calculate distance
		double avgTicks = abs((leftMtrs.getPosition() + rightMtrs.getPosition()) / 2.0);
		currDist = avgTicks / tpr * (wheelDiam * okapi::pi).convert(okapi::inch) * gearRatio;

		// Calculate linear and angular power
		double distCtrl = distPID.calculate(dist.abs().convert(okapi::inch), std::abs(currDist));
		double headingCtrl = headingPID.calculate(heading.convert(okapi::degree), inertial->get_rotation(AngleUnit::DEG));
		double decelRPM = maxRPM.convert(okapi::rpm) * distCtrl;
		double vel = std::min(maxRPM.convert(okapi::rpm), decelRPM);
		double volts = vel / static_cast<int>(leftMtrs.getGearing()) * 12000;
		double turnVolts = turnRPM.convert(okapi::rpm) / static_cast<int>(leftMtrs.getGearing()) * 12000;
		moveArcade(volts * dir, -turnVolts * headingCtrl, 6000);

		pros::delay(20);
	}

	moveTank(0, 0);
	printDebug("[Inline::MoveDistance] Finished with distance of %f\" with a heading of %f deg, taking %d ms\n", currDist, inertial->get_rotation(AngleUnit::DEG), pros::millis() - st);
}

void Inline::turnAbsolute(okapi::QAngle angle, okapi::QAngularSpeed maxRPM, lib16868C::PIDGains gains, double errorMargin, int numInMargin, TurnWheel turnWheel, int timeout) {
	PIDController turnPID(gains, 1, -1);

	double currAngle;
	uint st = pros::millis();
	int inMargin = 0;
	while (inMargin < numInMargin) {
		// Timeout
		if (pros::millis() - st > timeout && timeout > 0) {
			printError("[Inline::TurnAbsolute] Timeout: %d\n", pros::millis() - st);
			break;
		}

		currAngle = inertial->get_rotation(AngleUnit::DEG);

		// Checks if robot is within the error margin to terminate
		if (std::abs(angle.convert(okapi::degree) - currAngle) < errorMargin) inMargin++;
		else inMargin = 0;

		// Calculate turn power
		double turnCtrl = turnPID.calculate(angle.convert(okapi::degree), currAngle);
		double maxVolts = maxRPM.convert(okapi::rpm) / static_cast<int>(leftMtrs.getGearing()) * 12000;
		double volts = maxVolts * turnCtrl;

		// Determine turn type
		switch (turnWheel) {
			case TurnWheel::LEFT:
				rightMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
				moveTank(-volts, 0);
				break;
			case TurnWheel::RIGHT:
				leftMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
				moveTank(0, volts);
				break;
			case TurnWheel::BOTH:
				moveTank(-volts, volts);
				break;
		}

		pros::delay(20);
	}

	leftMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	rightMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	moveTank(0, 0);
	printDebug("[Inline::TurnAbsolute] Finished with heading of %f deg, taking %d ms\n", inertial->get_rotation(AngleUnit::DEG), pros::millis() - st);
}

void Inline::turnAbsolute(okapi::QAngle angle, okapi::QAngularSpeed maxRPM, PIDGains gains, TurnDirection turnDir, double errorMargin, int numInMargin, TurnWheel turnWheel, int timeout) {
	PIDController turnPID(gains);

	if (turnDir == TurnDirection::SHORTEST) {
		if (std::abs(angle.convert(okapi::degree) - inertial->get_rotation(AngleUnit::DEG)) < 180) turnDir = TurnDirection::COUNTER_CLOCKWISE;
		else turnDir = TurnDirection::CLOCKWISE;
	}

	double heading = inertial->get_rotation(AngleUnit::DEG);
	double target = angle.convert(okapi::degree);
	if (turnDir == TurnDirection::CLOCKWISE) target = ReduceAngle::reduce(target, heading + 360.0, heading);
	if (turnDir == TurnDirection::COUNTER_CLOCKWISE) target = ReduceAngle::reduce(target, heading, heading - 360.0);

	uint st = pros::millis(), t = pros::millis();
	int inMargin = 0;
	while (inMargin < numInMargin) {
		t = pros::millis();
		if (t - st > timeout && timeout > 0) {
			printError("[Inline::TurnAbsolute] Timeout: %d\n", pros::millis() - st);
			break;
		}

		heading = inertial->get_rotation(AngleUnit::DEG);

		if (std::abs(target - heading) < errorMargin) inMargin++;
		else inMargin = 0;

		double turnCtrl = turnPID.calculate(target, heading);
		double volts = maxRPM.convert(okapi::rpm) / static_cast<int>(leftMtrs.getGearing()) * 12000 * turnCtrl;

		switch (turnWheel) {
			case TurnWheel::LEFT:
				rightMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
				moveTank(-volts, 0, 6000);
				break;
			case TurnWheel::RIGHT:
				leftMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
				moveTank(0, volts, 6000);
				break;
			case TurnWheel::BOTH:
				moveTank(-volts, volts, 6000);
				break;
		}

		pros::delay(20);
	}

	leftMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	rightMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	moveTank(0, 0);
	std::cout << "[Inline Turn Absolute] Finished with heading of " << inertial->get_rotation(AngleUnit::DEG) << " deg, taking " << pros::millis() - st << "ms" << std::endl;
}

void Inline::moveToPoint(Pose target, okapi::QAngularSpeed maxRPM, PIDGains distGains, PIDGains headingGains, okapi::QLength endRadius, bool backward, bool stopMtrs, int timeout) {
	if (!odom) {
		printError("[Inline::MoveToPoint] No Odometry class was provided, unable to call moveToPoint\n");
		return;
	}

	PIDController distPID(distGains, 0, 0);
	PIDController headingPID(headingGains, 0, 0);

	Pose pose = odom->getPose();

	int dir = backward ? -1 : 1;

	uint st = pros::millis();
	int i = 0;
	while (pose.distTo(target) >= endRadius) {
		// Timeout
		if (pros::millis() - st > timeout && timeout > 0) {
			printError("[Inline::MoveToPoint] Timeout: %d\n", pros::millis() - st);
			break;
		}

		// Calculate Distance and Heading
		pose = odom->getPose();
		double curHeading = inertial->get_rotation(AngleUnit::RAD);
		double distLeft = pose.distTo(target).convert(okapi::inch);
		double heading = angleErrorRad(pose.angleTo(target).convert(okapi::radian), curHeading);
		if (backward) heading = angleErrorRad(target.angleTo(pose).convert(okapi::radian), curHeading);

		// Calculate PID control values
		double distCtrl = distPID.calculate(distLeft);
		double headingErr = heading - curHeading;
		double headingCtrl = headingPID.calculate(headingErr);

		// Determine heading deadzone
		double turnDeadzone = M_PI_2 - std::atan2(distLeft, (endRadius * 0.8).convert(okapi::inch));
		if (std::abs(heading) < std::abs(turnDeadzone)) headingCtrl = 0;

		// Calculate final power
		double volts = maxRPM.convert(okapi::rpm) / static_cast<int>(leftMtrs.getGearing()) * 12000;
		moveArcade(volts * distCtrl * dir * std::abs(std::cos(headingErr)), -volts * headingCtrl, 3000);
		printDebug("%s, %f\n", pose.toStr(), Util::radToDeg(heading));

		pros::delay(20);
	}

	if (stopMtrs) moveTank(0, 0);
	printDebug("[Inline::MoveToPoint] Finished with pose of %s, taking %d ms\n", odom->getPose().toStr(), pros::millis() - st);
}

void Inline::setBrakeMode(okapi::AbstractMotor::brakeMode mode) {
	leftMtrs.setBrakeMode(mode);
	rightMtrs.setBrakeMode(mode);
}
void Inline::coast() {
	setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
}
void Inline::brake() {
	setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
}
void Inline::hold() {
	setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
}