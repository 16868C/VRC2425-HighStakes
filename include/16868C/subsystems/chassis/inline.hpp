#pragma once
#include "okapi/api.hpp"
#include "16868C/controllers/pidController.hpp"
#include "16868C/devices/inertial.hpp"
#include "16868C/devices/motorGroup.hpp"
#include "16868C/subsystems/chassis/odometry.hpp"
#include "16868C/util/math.hpp"
#include "16868C/util/pose.hpp"

namespace lib16868C {
enum class TurnWheel {
	LEFT,
	RIGHT,
	BOTH
};

class Inline {
	public:
		Inline(MotorGroup& left, MotorGroup& right, Inertial& inertial, okapi::QLength wheelDiam, double gearRatio = 1, Odometry& odom);

		void moveTank(double left, double right);
		void moveArcade(double forward, double turn);

		void driveTank(double left, double right, double deadzone = 0);
		void driveArcade(double forward, double turn, double deadzone = 0);

		void moveDistance(okapi::QLength dist, okapi::QAngularSpeed maxRPM, PIDGains distGains, double accel, okapi::QAngle heading, okapi::QAngularSpeed turnRPM, PIDGains headingGains, int timeout = 0);
		void turnAbsolute(okapi::QAngle angle, okapi::QAngularSpeed maxRPM, PIDGains gains, double accelRate = 1.03, double errorMargin = 1, int numInMargin = 5, TurnWheel turnWheel = TurnWheel::BOTH, int timeout = 0);
		void moveToPoint(Point target, okapi::QAngularSpeed maxRPM, PIDGains distGains, double accel, okapi::QAngularSpeed turnRPM, PIDGains headingGains, int timeout = 0);

		void setBrakeMode(okapi::AbstractMotor::brakeMode mode);
		void coast();
		void brake();
		void hold();

	private:
		MotorGroup& leftMtrs, rightMtrs;
		Inertial& inertial;
		Odometry& odom;
		okapi::QLength wheelDiam;
		double gearRatio;
		double tpr;
};
} // namespace lib16868C