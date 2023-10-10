#pragma once
#include "okapi/api.hpp"
#include "16868Z/controllers/pidController.hpp"

namespace lib16868C {
enum class TurnWheel {
	LEFT,
	RIGHT,
	BOTH
};

class Inline {
	public:
		Inline(okapi::MotorGroup& left, okapi::MotorGroup& right, pros::Imu& inertial, double wheelDiam, double gearRatio = 1);

		void moveTank(double left, double right);
		void moveArcade(double forward, double turn);

		void driveTank(double left, double right, double deadzone = 0);
		void driveArcade(double forward, double turn, double deadzone = 0);

		void moveDistance(double dist, double maxRPM, lib16868C::PIDGains distGains, double accel, double heading, double turnRPM, lib16868C::PIDGains headingGains, int timeout = 0);
		void turnAbsolute(double angle, double maxRPM, lib16868C::PIDGains gains, double accelRate = 1.03, double errorMargin = 1, int numInMargin = 5, TurnWheel turnWheel = TurnWheel::BOTH, int timeout = 0);

		void setBrakeMode(okapi::AbstractMotor::brakeMode mode);

	private:
		okapi::MotorGroup& leftMtrs, rightMtrs;
		pros::Imu& inertial;
		double wheelDiam;
		double gearRatio;
		double tpr;
};
} // namespace lib16868C