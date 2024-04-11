#include "robotconfig.hpp"

okapi::Controller master(okapi::ControllerId::master);

// Motors
lib16868C::Motor leftFront(LEFT_FRONT, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor leftWeak(LEFT_WEAK, okapi::AbstractMotor::gearset::green);
lib16868C::Motor leftMiddle(LEFT_MIDDLE, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor leftRear(LEFT_REAR, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor rightFront(RIGHT_FRONT, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor rightWeak(RIGHT_WEAK, okapi::AbstractMotor::gearset::green);
lib16868C::Motor rightMiddle(RIGHT_MIDDLE, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor rightRear(RIGHT_REAR, okapi::AbstractMotor::gearset::blue);
lib16868C::MotorGroup leftDrive({leftFront, leftWeak, leftMiddle, leftRear});
lib16868C::MotorGroup rightDrive({rightFront, rightWeak, rightMiddle, rightRear});
lib16868C::Motor intake(INTAKE, okapi::AbstractMotor::gearset::blue);
// lib16868C::Motor kickerMtr(KICKER, okapi::AbstractMotor::gearset::green);

// Pneumatics
// lib16868C::Pneumatic horiHang(HORI_HANG, false);
lib16868C::Pneumatic leftWing(LEFT_WING, false);
lib16868C::Pneumatic rightWing(RIGHT_WING);
lib16868C::Pneumatic intakeRaiser(INTAKE_RAISER);
// lib16868C::Pneumatic winchPTO(WINCH_PTO);

// Sensors
lib16868C::Inertial inertial(INERTIAL);

// Subsystems
// lib16868C::Odometry odometry(
// 	std::array<lib16868C::TrackingWheel, 3>{leftEnc, rightEnc, middleEnc},
// 	std::array<lib16868C::DistanceSensor, 4>{{{}, backDist, leftDist, rightDist}},
// 	&inertial);
lib16868C::Inline chassis(leftDrive, rightDrive, &inertial, nullptr, WHEEL_DIAM, GEAR_RATIO);