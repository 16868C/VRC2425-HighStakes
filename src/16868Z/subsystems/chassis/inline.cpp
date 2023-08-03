#include "inline.hpp"
#include "16868Z/subsystems/chassis/motionProfiling.hpp"
#include "16868Z/util/util.hpp"

using namespace lib16868Z;

Inline::Inline(okapi::MotorGroup& leftMtrs, okapi::MotorGroup& rightMtrs, pros::Imu& inertial, double wheelDiam, double gearRatio)
	: leftMtrs(leftMtrs), rightMtrs(rightMtrs), inertial(inertial), wheelDiam(wheelDiam), gearRatio(gearRatio) {

	tpr = leftMtrs.getGearing() == okapi::AbstractMotor::gearset::red ? 1800 : 
			leftMtrs.getGearing() == okapi::AbstractMotor::gearset::green ? 900 :
																			300;
	
	leftMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	rightMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
}

void Inline::moveTank(double left, double right) {
	leftMtrs.moveVoltage(left);
	rightMtrs.moveVoltage(right);
}

void Inline::moveArcade(double forward, double turn) {
	leftMtrs.moveVoltage(forward + turn);
	rightMtrs.moveVoltage(forward - turn);
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

void Inline::moveDistance(double dist, double maxRPM, lib16868Z::PIDGains distGains, double maxAccelRPM, double heading, double turnRPM, lib16868Z::PIDGains headingGains, int timeout) {
	leftMtrs.tarePosition();
	rightMtrs.tarePosition();

	double maxVel = maxRPM * (wheelDiam * M_PI) / 60.0 / gearRatio;
	double maxAccel = maxAccelRPM * (wheelDiam * M_PI) / 60.0 / gearRatio;
	MotionProfile accelProfile = MotionProfiling::generateAccel(dist, {maxVel, maxAccel});

	PIDController distPID(distGains, 0, 0);
	PIDController headingPID(headingGains, 1, -1);

	double currDist = 0;

	int dir = Util::sgn(dist);

	int i = 0;
	uint32_t time = pros::millis();
	uint st = pros::millis(), t = pros::millis();
	while (std::abs(dist) - currDist > 0) {
		t = pros::millis();
		if (t - st > timeout && timeout > 0) {
			std::cout << "[Inline Move Distance] Timeout: " << t - st << std::endl;
			break;
		}

		double avgTicks = std::abs((leftMtrs.getEncoder()->get() + rightMtrs.getEncoder()->get()) / 2.0);
		currDist = avgTicks / tpr * (wheelDiam * M_PI) * gearRatio;

		double distCtrl = distPID.calculate(std::abs(dist), std::abs(currDist));
		double headingCtrl = headingPID.calculate(heading, inertial.get_rotation());

		double accelRPM = t - st <= accelProfile.accelTime ?
							accelProfile.profile[i++].velocity * 60.0 / (wheelDiam * M_PI) : maxRPM;
		double decelRPM = maxRPM * distCtrl;
		double vel = std::min(maxRPM, std::min(accelRPM, decelRPM));
		double volts = vel / static_cast<int>(leftMtrs.getGearing()) * 12000;
		double turnVolts = turnRPM / static_cast<int>(leftMtrs.getGearing()) * 12000;
		// std::cout << volts << "\n";
		moveArcade(volts * dir, turnVolts * headingCtrl);

		pros::Task::delay_until(&time, 20);
	}

	moveTank(0, 0);
	std::cout << "[Inline Move Distance] Finished with distance of " << currDist << "\" with a heading of " << inertial.get_rotation() << " deg" << std::endl;
}

void Inline::turnAbsolute(double angle, double maxRPM, lib16868Z::PIDGains gains, double accelRate, double errorMargin, int numInMargin, TurnWheel turnWheel, int timeout) {
	PIDController turnPID(gains, 1, -1);

	double currAngle = inertial.get_rotation();
	double prevVolts = 3000;
	uint st = pros::millis(), t = pros::millis();
	int inMargin = 0;
	while (inMargin < numInMargin) {
		t = pros::millis();
		if (t - st > timeout && timeout > 0) {
			std::cout << "[Inline Turn Absolute] Timeout: " << t - st << std::endl;
			break;
		}

		currAngle = inertial.get_rotation();

		if (std::abs(angle - currAngle) < errorMargin) inMargin++;
		else inMargin = 0;

		double turnCtrl = turnPID.calculate(angle, currAngle);
		double volts = maxRPM / static_cast<int>(leftMtrs.getGearing()) * 12000;

		volts = std::min(volts * turnCtrl, prevVolts * accelRate * Util::sgn(turnCtrl));
		prevVolts = volts;

		// std::cout << volts << " " << turnCtrl << " ";
		// std::cout << inertial.get_rotation() << "\n";

		switch (turnWheel) {
			case TurnWheel::LEFT:
				rightMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
				moveTank(volts, 0);
				break;
			case TurnWheel::RIGHT:
				leftMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
				moveTank(0, volts);
				break;
			case TurnWheel::BOTH:
				moveTank(volts, -volts);
				break;
		}

		pros::delay(20);
	}

	leftMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	rightMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	moveTank(0, 0);
	std::cout << "[Inline Turn Absolute] Finished with heading of " << inertial.get_rotation() << " deg" << std::endl;
}

void Inline::setBrakeMode(okapi::AbstractMotor::brakeMode mode) {
	leftMtrs.setBrakeMode(mode);
	rightMtrs.setBrakeMode(mode);
}