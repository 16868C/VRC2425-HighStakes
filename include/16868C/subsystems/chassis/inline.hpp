#pragma once
#include "okapi/api/units/QAngularSpeed.hpp"
#include "16868C/controllers/pidController.hpp"
#include "16868C/devices/inertial.hpp"
#include "16868C/devices/motorGroup.hpp"
#include "16868C/subsystems/chassis/odometry.hpp"
#include "16868C/util/pose.hpp"
#include "okapi/api/units/QLength.hpp"

namespace lib16868C {
enum class TurnWheel {
	LEFT,
	RIGHT,
	BOTH
};

struct MoveDistanceParams {
	okapi::QAngularSpeed maxRPM = 600_rpm;
	okapi::QAngularSpeed minRPM = 0_rpm;
	okapi::QAngularSpeed turnRPM = 300_rpm;

	PIDGains distGains = {0, 0, 0};
	PIDGains headingGains = {0, 0, 0};

	okapi::QLength exitDist = 0_in;

	double slewRate = 0;
};
struct TurnAbsoluteParams {
	okapi::QAngularSpeed maxRPM = 600_rpm;
	okapi::QAngularSpeed minRPM = 0_rpm;

	PIDGains gains = {};

	okapi::QAngle errorMargin = 3_deg;
	double numInMargin = 5;

	TurnWheel turnWheel = TurnWheel::BOTH;
	TurnDirection dir = TurnDirection::SHORTEST;

	double slewRate = 0;
};
struct TurnToPointParams {
	okapi::QAngularSpeed maxRPM = 600_rpm;
	okapi::QAngularSpeed minRPM = 0_rpm;

	PIDGains gains = {0, 0, 0};

	okapi::QAngle errorMargin = 3_deg;
	double numInMargin = 5;

	TurnWheel turnWheel = TurnWheel::BOTH;
	TurnDirection dir = TurnDirection::SHORTEST;

	double slewRate = 0;
};
struct MoveToPointParams {
	okapi::QAngularSpeed maxRPM = 600_rpm;
	okapi::QAngularSpeed minRPM = 0_rpm;

	PIDGains distGains = {};
	PIDGains headingGains = {};

	okapi::QLength exitRadius = 1_in;
	okapi::QLength turnDeadzone = 6_in;

	bool reverse = false;

	double slewRate = 0;
};
struct MoveToPoseParams {
	okapi::QAngularSpeed maxRPM = 600_rpm;
	okapi::QAngularSpeed minRPM = 0_rpm;

	PIDGains distGains = {};
	PIDGains headingGains = {};

	bool reverse = false;

	okapi::QLength settleRadius = 7.5_in;
	okapi::QLength earlyExitDist = 0_in;

	double horiDrift = 1 * 52 * 52;

	double dlead = 0.6;
	double glead = 0;

	okapi::QLength gRadius = 8_in;

	double slewRate = 0;
};

class Inline {
public:

	Inline(MotorGroup& left, MotorGroup& right, Inertial* inertial, Odometry* odom, okapi::QLength wheelDiam, double gearRatio = 1.0);

	void moveTank(double left, double right, double slewRate = 0);
	void moveArcade(double forward, double turn, double slewRate = 0);

	void driveTank(double left, double right, double deadzone = 0);
	void driveArcade(double forward, double turn, double deadzone = 0);

	void moveDistance(okapi::QLength dist, okapi::QAngle heading, int timeout = 0, MoveDistanceParams params = {});
	void turnAbsolute(okapi::QAngle angle, int timeout = 0, TurnAbsoluteParams params = {});

	void turnToPoint(Pose target, int timeout = 0, TurnToPointParams params = {});
	void moveToPoint(Pose target, int timeout = 0, MoveToPointParams params = {});
	void moveToPose(Pose target, int timeout = 0, MoveToPoseParams params = {});

	void setBrakeMode(okapi::AbstractMotor::brakeMode mode);
	void coast();
	void brake();
	void hold();

private:
	MotorGroup& leftMtrs, rightMtrs;
	Inertial* inertial { nullptr };
	Odometry* odom { nullptr };
	okapi::QLength wheelDiam;
	double gearRatio;
	double tpr;

	// Move Slew Rate Values
	double prevLeft = 0, prevRight = 0;

	const int MAX_VOLT = 12000;
};
} // namespace lib16868C