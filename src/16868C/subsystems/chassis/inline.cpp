#include "inline.hpp"
#include "16868C/subsystems/chassis/motionProfiling.hpp"
#include "16868C/util/logger.hpp"
#include "16868C/util/math.hpp"
#include "16868C/util/util.hpp"

using namespace lib16868C;

Inline::Inline(MotorGroup& leftMtrs, MotorGroup& rightMtrs, Inertial& inertial, Odometry* odom, okapi::QLength wheelDiam, double gearRatio)
	: leftMtrs(leftMtrs), rightMtrs(rightMtrs), inertial(inertial), odom(odom), wheelDiam(wheelDiam), gearRatio(gearRatio) {

	tpr = leftMtrs.getGearing() == okapi::AbstractMotor::gearset::red ? 1800 : 
			leftMtrs.getGearing() == okapi::AbstractMotor::gearset::green ? 900 :
																			300;
	
	leftMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	rightMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
}

void Inline::moveTank(double left, double right) {
	double max = std::max(std::abs(left), std::abs(right));
	if (max > MAX_VOLT) {
		left *= MAX_VOLT / max;
		right *= MAX_VOLT / max;
	}

	leftMtrs.moveVoltage(left);
	rightMtrs.moveVoltage(right);
}

void Inline::moveArcade(double forward, double turn) {
	double left = forward + turn;
	double right = forward - turn;

	moveTank(left, right);
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

void Inline::moveDistance(okapi::QLength dist, okapi::QAngularSpeed maxRPM, lib16868C::PIDGains distGains, double maxAccelRPM, okapi::QAngle heading, okapi::QAngularSpeed turnRPM, lib16868C::PIDGains headingGains, int timeout) {
	leftMtrs.tarePosition();
	rightMtrs.tarePosition();

	double maxVel = maxRPM.convert(okapi::rpm) * (wheelDiam * okapi::pi).convert(okapi::inch) / 60.0 / gearRatio;
	double maxAccel = maxAccelRPM * (wheelDiam * okapi::pi).convert(okapi::inch) / 60.0 / gearRatio;
	MotionProfile accelProfile = MotionProfiling::generateAccel(dist.convert(okapi::inch), {maxVel, maxAccel});

	PIDController distPID(distGains, 0, 0);
	PIDController headingPID(headingGains, 1, -1);

	double currDist = 0;

	int dir = Util::sgn(dist.convert(okapi::inch));

	int i = 0;
	uint32_t time = pros::millis();
	uint st = pros::millis(), t = pros::millis();
	while (dist.abs().convert(okapi::inch) - currDist > 0) {
		t = pros::millis();
		if (t - st > timeout && timeout > 0) {
			printDebug("[Inline Move Distance] Timeout: %d ms\n", t - st);
			break;
		}

		double avgTicks = std::abs((leftMtrs.getPosition() + rightMtrs.getPosition()) / 2.0);
		currDist = avgTicks / tpr * (wheelDiam * okapi::pi).convert(okapi::inch) * gearRatio;

		double distCtrl = distPID.calculate(dist.abs().convert(okapi::inch), std::abs(currDist));
		double headingCtrl = headingPID.calculate(heading.convert(okapi::degree), inertial.get_rotation(AngleUnit::DEG));

		double accelRPM = t - st < accelProfile.accelTime * 1000 ?
							accelProfile.profile[i++].velocity * 60.0 / (wheelDiam * okapi::pi).convert(okapi::inch) : maxRPM.convert(okapi::rpm);
		double decelRPM = maxRPM.convert(okapi::rpm) * distCtrl;
		double vel = std::min(maxRPM.convert(okapi::rpm), std::min(accelRPM, decelRPM));
		double volts = vel / static_cast<int>(leftMtrs.getGearing()) * 12000;
		double turnVolts = turnRPM.convert(okapi::rpm) / static_cast<int>(leftMtrs.getGearing()) * 12000;
		moveArcade(volts * dir, turnVolts * headingCtrl * (i / accelProfile.profile.size() + 0.5));

		pros::Task::delay_until(&time, 20);
	}

	moveTank(0, 0);
	double avgTicks = std::abs((leftMtrs.getPosition() + rightMtrs.getPosition()) / 2.0);
	currDist = avgTicks / tpr * (wheelDiam * okapi::pi).convert(okapi::inch) * gearRatio;
	printDebug("[Inline Move Distance] Finished with distance of %f\" with a heading of %f deg, taking %d ms\n", currDist, inertial.get_rotation(AngleUnit::DEG), pros::millis() - st);
}

void Inline::turnAbsolute(okapi::QAngle angle, okapi::QAngularSpeed maxRPM, lib16868C::PIDGains gains, double accelRate, double errorMargin, int numInMargin, TurnWheel turnWheel, int timeout) {
	PIDController turnPID(gains, 1, -1);

	double currAngle = inertial.get_rotation(AngleUnit::DEG);
	double accelVolts = 3000;
	uint st = pros::millis(), t = pros::millis();
	int inMargin = 0;
	while (inMargin < numInMargin) {
		t = pros::millis();
		if (t - st > timeout && timeout > 0) {
			printDebug("[Inline Turn Absolute] Timeout: %d\n", t - st);
			break;
		}

		currAngle = inertial.get_rotation(AngleUnit::DEG);

		if (std::abs(angle.convert(okapi::degree) - currAngle) < errorMargin) inMargin++;
		else inMargin = 0;

		double turnCtrl = turnPID.calculate(angle.convert(okapi::degree), currAngle);
		double maxVolts = maxRPM.convert(okapi::rpm) / static_cast<int>(leftMtrs.getGearing()) * 12000;
		accelVolts *= accelRate;
		double volts = std::min(maxVolts * std::abs(turnCtrl), accelVolts) * Util::sgn(turnCtrl);

		switch (turnWheel) {
			case TurnWheel::LEFT:
				rightMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
				moveTank(volts, 0);
				break;
			case TurnWheel::RIGHT:
				leftMtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
				moveTank(0, -volts);
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
	printDebug("[Inline Turn Absolute] Finished with heading of %f deg, taking %d ms\n", inertial.get_rotation(AngleUnit::DEG), pros::millis() - st);
}

void Inline::moveToPoint(Pose target, okapi::QAngularSpeed maxRPM, lib16868C::PIDGains distGains, double maxAccelRPM, okapi::QAngularSpeed turnRPM, lib16868C::PIDGains headingGains, int timeout) {
	if (!odom) {
		printError("[Inline Move to Point] No Odometry class was provided, unable to call moveToPoint\n");
		return;
	}
	
	Pose curPose = odom->getPose();
	okapi::QLength dist = curPose.distTo(target);
	okapi::QAngle heading = curPose.angleTo(target);

	okapi::QAngle lo = std::floor(curPose.theta / (2 * M_PI)) * 2 * M_PI * okapi::radian;
	okapi::QAngle hi = std::ceil(curPose.theta / (2 * M_PI)) * 2 * M_PI * okapi::radian;
	heading = ReduceAngle::reduce(heading, lo, hi);

	if (maxRPM.convert(okapi::rpm) < 0) {
		dist *= -1;
		heading -= okapi::pi * okapi::radian;
	}
	
	moveDistance(dist, abs(maxRPM), distGains, maxAccelRPM, heading, turnRPM, headingGains, timeout);

	target.theta = inertial.get_rotation(AngleUnit::RAD);
	odom->update(target);
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