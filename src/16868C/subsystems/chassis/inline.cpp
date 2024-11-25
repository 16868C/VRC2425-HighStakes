#include "inline.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "16868C/util/logger.hpp"
#include "16868C/util/math.hpp"
#include "16868C/util/util.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <algorithm>

using namespace lib16868C;

Inline::Inline(MotorGroup& leftMtrs, MotorGroup& rightMtrs, Inertial* inertial, Odometry* odom, okapi::QLength wheelDiam, double gearRatio)
	: leftMtrs(leftMtrs), rightMtrs(rightMtrs), inertial(inertial), odom(odom), wheelDiam(wheelDiam), gearRatio(gearRatio) {

	tpr = leftMtrs.getTPR();
	
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

void Inline::moveDistance(okapi::QLength dist, okapi::QAngle heading, int timeout, MoveDistanceParams params) {
	PIDController distPID(params.distGains, 0, 0);
	PIDController headingPID(params.headingGains, 1, -1);

	double currDist = 0;
	int dir = Util::sgn(dist.convert(okapi::inch));
	dist = dist.abs();

	bool useOdom = odom->getEncoders()[0]->getType() != TrackingWheelType::INVALID;
	double stDist = odom->getEncoders()[0]->getDist();
	if (!useOdom) {
		leftMtrs.tarePosition();
		rightMtrs.tarePosition();
	}

	int i = 0;
	uint st = pros::millis();
	while (dist.convert(okapi::inch) - currDist > params.exitDist.convert(okapi::inch)) {
		// Timeout
		if (pros::millis() - st > timeout && timeout > 0) {
			printError("[Inline::MoveDistance] Timeout: %d\n", pros::millis() - st);
			break;
		}

		// Calculate Distance Using Either Tracking Wheels or IME 
		if (useOdom) { // Tracking Wheel
			currDist = std::abs(odom->getEncoders()[0]->getDist() - stDist);
		} else { // IME
			double avgTicks = std::abs((leftMtrs.getPosition() + rightMtrs.getPosition()) / 2);
			currDist = avgTicks / tpr * (wheelDiam * okapi::pi).convert(okapi::inch) * gearRatio;
		}

		// PID Calculations
		double distCtrl = distPID.calculate(dist.convert(okapi::inch), currDist);
		double headingCtrl = headingPID.calculate(heading.convert(okapi::radian), inertial->get_rotation(AngleUnit::RAD));
		
		// Calculate Linear Power
		double linearRPM = params.maxRPM.convert(okapi::rpm) * distCtrl;
		double linearVel = std::clamp(linearRPM, params.minRPM.convert(okapi::rpm), params.maxRPM.convert(okapi::rpm));
		double linearPower = linearVel / static_cast<int>(leftMtrs.getGearing()) * MAX_VOLT * dir;
		
		// Calculate Angular Power
		double angularRPM = params.turnRPM.convert(okapi::rpm) * headingCtrl;
		double angularVel = std::max(std::abs(angularRPM), params.minRPM.abs().convert(okapi::rpm)) * Util::sgn(headingCtrl);
		double angularPower = angularVel / static_cast<int>(leftMtrs.getGearing()) * MAX_VOLT;
		
		// Sets Motor Speeds with Slew Rate for Acceleration
		moveArcade(linearPower, -angularPower, params.slewRate);

		pros::delay(20);
	}

	if (params.minRPM.convert(okapi::rpm) == 0) moveTank(0, 0);
	printDebug("[Inline::MoveDistance] Finished with distance of %f\" with a heading of %f deg, taking %d ms\n", currDist, inertial->get_rotation(AngleUnit::DEG), pros::millis() - st);
}

void Inline::turnAbsolute(okapi::QAngle angle, int timeout, TurnAbsoluteParams params) {
	PIDController turnPID(params.gains, 1, -1);

	double currAngle = inertial->get_rotation(AngleUnit::RAD);
	double target = getTargetHeading(angle.convert(okapi::radian), currAngle, false, params.dir);
	uint st = pros::millis();
	int inMargin = 0;
	while (inMargin < params.numInMargin) {
		// Timeout
		if (pros::millis() - st > timeout && timeout > 0) {
			printError("[Inline::TurnAbsolute] Timeout: %d\n", pros::millis() - st);
			break;
		}

		currAngle = inertial->get_rotation(AngleUnit::RAD);

		// Checks if robot is within the error margin to terminate
		if (std::abs(angle.convert(okapi::radian) - currAngle) < params.errorMargin.convert(okapi::radian)) inMargin++;
		else inMargin = 0;

		// Calculate turn power
		double turnCtrl = turnPID.calculate(target, currAngle);
		double turnRPM = params.maxRPM.convert(okapi::rpm) * turnCtrl;
		turnRPM = std::max(std::abs(turnRPM), params.minRPM.convert(okapi::rpm)) * Util::sgn(turnRPM);
		double volts = turnRPM / static_cast<int>(leftMtrs.getGearing()) * MAX_VOLT;

		// Determine turn type
		switch (params.turnWheel) {
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


void Inline::turnToPoint(Pose target, int timeout, TurnToPointParams params) {
	if (!odom) {
		printError("[Inline::TurnToPoint] No Odometry class was provided, unable to call moveToPoint\n");
		return;
	}

	double tgtHeading = odom->getPose().angleTo(target);
	tgtHeading = getTargetHeading(tgtHeading, inertial->get_rotation(AngleUnit::RAD));
	TurnAbsoluteParams turnAbsoluteParams = {params.maxRPM,
											params.minRPM, 
											params.gains, 
											params.errorMargin, 
											params.numInMargin,
											params.turnWheel,
											params.dir,
											params.slewRate};

	turnAbsolute(tgtHeading * okapi::radian, timeout, turnAbsoluteParams);
}

void Inline::moveToPoint(Pose target, int timeout, MoveToPointParams params) {
	if (!odom) {
		printError("[Inline::MoveToPoint] No Odometry class was provided, unable to call moveToPoint\n");
		return;
	}

	PIDController distPID(params.distGains, 0, 0);
	PIDController headingPID(params.headingGains, 1, -1);

	Pose pose = odom->getPose();

	int dir = params.reverse ? -1 : 1;
	// bool prevSide = sin(pose.theta) * (pose.y - target.y) > -cos(pose.theta) * (pose.x - target.x) - params.earlyEndRadius.convert(okapi::inch);

	uint st = pros::millis();
	while (true) {
		// Timeout
		if (pros::millis() - st > timeout && timeout > 0) {
			printError("[Inline::MoveToPoint] Timeout: %d\n", pros::millis() - st);
			break;
		}

		// Calculate Distance and Heading
		pose = odom->getPose();
		double curHeading = pose.theta;
		double distLeft = pose.distTo(target);
		double heading = getTargetHeading(pose.angleTo(target), curHeading, true);
		if (params.reverse) heading = getTargetHeading(target.angleTo(pose), curHeading, true);

		// bool currSide = sin(pose.angleTo(target)) * (pose.y - target.y) > -cos(pose.angleTo(target)) * (pose.x - target.x);
		// if (currSide != prevSide && (params.minRPM > 0_rpm && pose.distTo(Point(target.x - params.earlyEndRadius.convert(okapi::inch) * sin(pose.theta), target.y - params.earlyEndRadius.convert(okapi::inch) * cos(pose.theta))) < params.earlyEndRadius.convert(okapi::inch))) break;
		// if (currSide != prevSide && (pose.distTo(target) < (params.minRPM > 0_rpm ? params.earlyEndRadius.convert(okapi::inch) : params.endRadius.convert(okapi::inch)))) break;
		// prevSide = currSide;
		if (pose.distTo(target) < params.exitRadius.convert(okapi::inch)) break;

		// if (params.minRPM > 0_rpm && pose.distTo(target) < params.earlyEndRadius.convert(okapi::inch)) break;

		// Calculate PID control values
		double distCtrl = distPID.calculate(distLeft);
		double headingErr = heading - curHeading;
		double headingCtrl = headingPID.calculate(headingErr);

		// Determine heading deadzone
		// double turnDeadzone = M_PI_2 - std::atan2(distLeft, (params.endRadius * params.turnDeadzone).convert(okapi::inch));
		// if (std::abs(heading) < std::abs(turnDeadzone)) headingCtrl = 0;
		if (std::abs(distLeft) < params.turnDeadzone.abs().convert(okapi::inch)) headingCtrl = 0;

		// Calculate final power
		double fwdRPM = params.maxRPM.convert(okapi::rpm) * distCtrl;
		fwdRPM = std::clamp(fwdRPM, params.minRPM.convert(okapi::rpm), params.maxRPM.convert(okapi::rpm));
		double fwdVolts = fwdRPM / static_cast<int>(leftMtrs.getGearing()) * MAX_VOLT;
		double fwdPower = fwdVolts * dir * std::abs(std::cos(headingErr));

		double turnRPM = params.maxRPM.convert(okapi::rpm) * headingCtrl;
		double turnVolts = turnRPM / static_cast<int>(leftMtrs.getGearing()) * MAX_VOLT;
		double turnPower = -turnVolts;
		
		double overturn = abs(fwdPower) + abs(turnPower) - MAX_VOLT;
		if (overturn > 0) fwdPower -= overturn * dir;

		moveArcade(fwdPower, turnPower, params.slewRate);

		pros::delay(20);
	}

	moveArcade(params.minRPM.convert(okapi::rpm) * dir, 0, 0);
	printDebug("[Inline::MoveToPoint] Finished with pose of %s, taking %d ms\n", odom->getPose().toStr(), pros::millis() - st);
}

void Inline::moveToPose(Pose target, int timeout, MoveToPoseParams params) {
	if (!odom) {
		printError("[Inline::MoveToPoint] No Odometry class was provided, unable to call moveToPoint\n");
		return;
	}

	int n = 1;

	PIDController distPID(params.distGains, 0, 0);
	PIDController headingPID(params.headingGains, 1, -1);

	Pose pose = odom->getPose();
	double dist = pose.distTo(target);
	Point carrot = target - Point(dist * cos(target.theta), dist * sin(target.theta)) * params.dlead;
	Point ghost = Point::lerp(target, carrot, params.glead);
	Point& tgt = ghost;
	
	int dir = params.reverse ? -1 : 1;
	bool settling = false;
	bool prevSide = true;

	uint st = pros::millis();
	// FILE* data = fopen("/usd/Data.csv", "w");
	// printDebug("x,y,theta,crtx,crty,fwdPower,distErr,distCtrl,turnPower,headingErr,headingCtrl\n");
	// fputs("x,y,theta,crtx,crty,fwdPower,distErr,distCtrl,turnPower,headingErr,headingCtrl\n", data);
	while (true) {
		if (pros::millis() - st > timeout && timeout > 0) {
			printError("[Inline::MoveToPose] Timeout: %d\n", pros::millis() - st);
			break;
		}

		pose = odom->getPose();

		if ((pose.distTo(ghost) < params.gRadius.convert(okapi::inch) || pose.distTo(carrot) < params.gRadius.convert(okapi::inch))
				&& &tgt == &ghost) tgt = target;

		dist = pose.distTo(tgt);
		carrot = tgt - Point(dist * cos(target.theta), dist * sin(target.theta)) * params.dlead;

		if (pose.distTo(target) < params.settleRadius.convert(okapi::inch)) settling = true;

		// printDebug("%f,%f,%f,%f,%f,%f,%f\n", pose.x, pose.y, carrot.x, carrot.y, target.x, target.y, target.theta);
		bool botSide = sin(target.theta) * (pose.y - target.y) > -cos(target.theta) * (pose.x - target.x) - params.earlyExitDist.convert(okapi::inch);
		bool crtSide = sin(target.theta) * (carrot.y - target.y) > -cos(target.theta) * (carrot.x - target.x) - params.earlyExitDist.convert(okapi::inch);
		bool sameSide = botSide == crtSide;
		if (sameSide != prevSide && settling) break;
		prevSide = sameSide;

		if (settling) carrot = target;

		double distErr = std::max(pose.distTo(target), pose.distTo(carrot));
		// if (settling) distErr = std::max(pose.distTo(target) * std::cos(target.theta - pose.angleTo(target)), 0.0);
		// double distErr = pose.distTo(target);

		double headingErr = getTargetHeading(pose.angleTo(tgt), pose.theta, true) - pose.theta;
		if (params.reverse) headingErr = getTargetHeading(tgt.angleTo(pose), pose.theta, true) - pose.theta;
		if (settling) headingErr = getTargetHeading(target.theta, pose.theta, true) - pose.theta;

		double distCtrl = distPID.calculate(distErr);
		double headingCtrl = headingPID.calculate(headingErr);

		double fwdRPM = params.maxRPM.convert(okapi::rpm) * distCtrl;
		double turnRPM = params.maxRPM.convert(okapi::rpm) * headingCtrl;
		if (params.minRPM == 0_rpm)
			fwdRPM = std::clamp(std::abs(fwdRPM), params.minRPM.convert(okapi::rpm), params.maxRPM.convert(okapi::rpm)) * Util::sgn(distCtrl);
		else
			fwdRPM = std::clamp(fwdRPM, params.minRPM.convert(okapi::rpm), params.maxRPM.convert(okapi::rpm));
		turnRPM = std::min(std::abs(turnRPM), params.maxRPM.convert(okapi::rpm)) * Util::sgn(turnRPM);

		double fwdVolts = fwdRPM / static_cast<int>(leftMtrs.getGearing()) * MAX_VOLT * dir;
		double turnVolts = turnRPM / static_cast<int>(leftMtrs.getGearing()) * MAX_VOLT;

		double fwdPower = fwdVolts * abs(cos(headingErr));
		// if (settling) fwdPower *= abs(cos(headingErr));
		double turnPower = -turnVolts;

		double radius = getRadius(pose, carrot);
		double maxSlipSpeed = sqrt(params.horiDrift * radius * 9.81);
		double maxSlipPower = (maxSlipSpeed * 39.37 * 60 / (wheelDiam.convert(okapi::inch) * M_PI) / gearRatio) / static_cast<int>(leftMtrs.getGearing()) * MAX_VOLT;
		if (!settling) fwdPower = std::clamp(fwdPower, -maxSlipPower, maxSlipPower);
		// printDebug("%f, %f, %f\n", fwdPower, radius, maxSlipSpeed);

		double overturn = abs(fwdPower) + abs(turnPower) - MAX_VOLT;
		if (overturn > 0) fwdPower -= overturn * dir;

		moveArcade(fwdPower, turnPower, params.slewRate);

		// if (n++ % 5 == 0) {
		// 	n = 1;
			// printDebug("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", pose.x, pose.y, Util::radToDeg(pose.theta), carrot.x, carrot.y, ghost.x, ghost.y, fwdPower, distErr, turnPower, Util::radToDeg(headingErr), overturn, radius, maxSlipSpeed);
		// }
		// fprintf(data, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", pose.x, pose.y, Util::radToDeg(pose.theta), carrot.x, carrot.y, fwdPower, distErr, distCtrl, turnPower, Util::radToDeg(headingErr), headingCtrl);
		// fflush(data);
		// if (n++ % 100 == 0) {
		// 	fclose(data);
		// 	data = fopen("/usd/Data.csv", "a");
		// }

		pros::delay(10);
	}

	moveTank(0, 0);
	printDebug("[Inline::MoveToPose] Finished with pose of %s, taking %d ms\n", odom->getPose().toStr(), pros::millis() - st);
	// fclose(data);
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