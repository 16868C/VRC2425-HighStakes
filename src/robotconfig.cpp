#include "robotconfig.hpp"
#include "16868C/devices/trackingWheel.hpp"

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

// Pneumatics

// Sensors
lib16868C::Inertial inertial(INERTIAL);
lib16868C::Rotation vertRot(VERT_ROT);
lib16868C::Rotation hortRot(HORT_ROT);
lib16868C::TrackingWheel vertEnc(&vertRot, 2_in, 3.03_in);
lib16868C::TrackingWheel hortEnc(&hortRot, 2_in, 5.03_in);
okapi::DistanceSensor rightDistance(RIGHT_DIST);
okapi::DistanceSensor backDistance(BACK_DIST);
okapi::DistanceSensor leftDistance(LEFT_DIST);
lib16868C::DistanceSensor rightDist(&rightDistance, 4.5_in);
lib16868C::DistanceSensor backDist(&backDistance, 5_in);
lib16868C::DistanceSensor leftDist(&leftDistance, 4.5_in);

// Subsystems
lib16868C::Odometry odometry(
	std::array<lib16868C::TrackingWheel, 3>{vertEnc, {}, hortEnc},
	std::array<lib16868C::DistanceSensor, 4>{{{}, backDist, leftDist, rightDist}},
	&inertial);
lib16868C::Inline chassis(leftDrive, rightDrive, inertial, &odometry, WHEEL_DIAM, GEAR_RATIO);