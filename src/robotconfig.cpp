#include "robotconfig.hpp"
#include "16868C/devices/pneumatic.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"

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

lib16868C::Motor armLeft(ARM_LEFT, okapi::AbstractMotor::gearset::green);
lib16868C::Motor armRight(ARM_RIGHT, okapi::AbstractMotor::gearset::green);
lib16868C::MotorGroup arm({armLeft, armRight});

// Pneumatics
lib16868C::Pneumatic clamp(MOGO_CLAMP);
lib16868C::Pneumatic tilter(MOGO_TILTER);

// Sensors
lib16868C::Inertial inertial(INERTIAL);
okapi::DistanceSensor hookDist(HOOK_DISTANCE_SNSR);
okapi::OpticalSensor ringDetect(RING_OPTICAL_SNSR);

// Subsystems
// lib16868C::Odometry odometry(
// 	std::array<lib16868C::TrackingWheel, 3>{leftEnc, rightEnc, middleEnc},
// 	std::array<lib16868C::DistanceSensor, 4>{{{}, backDist, leftDist, rightDist}},
// 	&inertial);
lib16868C::Inline chassis(leftDrive, rightDrive, &inertial, nullptr, WHEEL_DIAM, GEAR_RATIO);