#include "robotconfig.hpp"
#include <memory>

okapi::Controller master(okapi::ControllerId::master);

// Motors
lib16868C::Motor leftFront(LEFT_FRONT, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor leftMiddle(LEFT_MIDDLE, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor leftRear(LEFT_REAR, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor rightFront(RIGHT_FRONT, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor rightMiddle(RIGHT_MIDDLE, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor rightRear(RIGHT_REAR, okapi::AbstractMotor::gearset::blue);
lib16868C::MotorGroup leftDrive({leftFront, leftMiddle, leftRear});
lib16868C::MotorGroup rightDrive({rightFront, rightMiddle, rightRear});
lib16868C::Motor intake(INTAKE, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor kickerMtr(KICKER, okapi::AbstractMotor::gearset::green);

// Pneumatics
lib16868C::Pneumatic horiHang(HORI_HANG, true);
lib16868C::Pneumatic leftWing(LEFT_WING, false);
lib16868C::Pneumatic rightWing(RIGHT_WING);
lib16868C::Pneumatic vertWings(VERT_WINGS);

// Sensors
lib16868C::Inertial inertial(INERTIAL);
lib16868C::Rotation middleRot(-MIDDLE_ROT);
lib16868C::TrackingWheel leftEnc(&leftDrive, 3.25_in, 5.5_in, GEAR_RATIO);
lib16868C::TrackingWheel rightEnc(&rightDrive, 3.25_in, 5.5_in, GEAR_RATIO);
lib16868C::TrackingWheel middleEnc(&middleRot, 2.75_in, 3.5_in);
okapi::DistanceSensor rightDistance(RIGHT_DIST);
okapi::DistanceSensor backDistance(BACK_DIST);
okapi::DistanceSensor leftDistance(LEFT_DIST);
lib16868C::DistanceSensor rightDist(&rightDistance, 4.5_in);
lib16868C::DistanceSensor backDist(&backDistance, 5_in);
lib16868C::DistanceSensor leftDist(&leftDistance, 4.5_in);

// Subsystems
lib16868C::Odometry odometry(
	std::array<lib16868C::TrackingWheel, 3>{leftEnc, rightEnc, middleEnc},
	std::array<lib16868C::DistanceSensor, 4>{{{}, backDist, leftDist, rightDist}},
	&inertial);
lib16868C::Inline chassis(leftDrive, rightDrive, inertial, &odometry, WHEEL_DIAM, GEAR_RATIO);
lib16868C::Kicker kicker(kickerMtr);