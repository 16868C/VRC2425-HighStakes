#include "robotconfig.hpp"
#include "16868C/devices/trackingWheel.hpp"
#include "16868C/subsystems/chassis/odometry.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"

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

lib16868C::Motor intakeMtr(INTAKE, okapi::AbstractMotor::gearset::blue);

lib16868C::Motor armLeft(ARM_LEFT, okapi::AbstractMotor::gearset::green);
lib16868C::Motor armRight(ARM_RIGHT, okapi::AbstractMotor::gearset::green);
lib16868C::MotorGroup armMtrs({armLeft, armRight});

// Pneumatics
pros::adi::Pneumatics clamp(MOGO_CLAMP, false, true);
pros::adi::Pneumatics leftDoinker(LEFT_DOINKER, false);
pros::adi::Pneumatics rightDoinker(RIGHT_DOINKER, false);
pros::adi::Pneumatics intakeRaiser(INTAKE_RAISER, true, true);

// Sensors
lib16868C::Inertial inertial(INERTIAL);

lib16868C::Rotation vertRot(VERT_ENC);
lib16868C::Rotation hortRot(HORT_ENC);
lib16868C::TrackingWheel vertEnc(&vertRot, 2.75_in, 0.695501_in);
lib16868C::TrackingWheel hortEnc(&hortRot, 2_in, -3.47526_in);

okapi::DistanceSensor frontDistance(DIST_FRONT);
okapi::DistanceSensor rearDistance(DIST_REAR);
okapi::DistanceSensor leftDistance(DIST_LEFT);
okapi::DistanceSensor rightDistance(DIST_RIGHT);
lib16868C::DistanceSensor frontDist(&frontDistance, 2_in);
lib16868C::DistanceSensor rearDist(&rearDistance, 6.5_in);
lib16868C::DistanceSensor leftDist(&leftDistance, 6.375_in);
lib16868C::DistanceSensor rightDist(&rightDistance, 6.375_in);

lib16868C::Rotation intakeEnc(INTAKE_ENC);
okapi::OpticalSensor ringOptical(RING_OPTICAL);
lib16868C::Rotation armEnc(ARM_ENC);

pros::adi::Potentiometer autonSelector(AUTON_SELECTOR, pros::E_ADI_POT_V2);
lib16868C::AutonSelector auton(autonSelector);

// Subsystems
lib16868C::Odometry odometry(
	std::array<lib16868C::TrackingWheel, 3>{vertEnc, {}, hortEnc},
	std::array<lib16868C::DistanceSensor, 4>{{}}, // frontDist, leftDist, rearDist, rightDist
	&inertial);
lib16868C::Inline chassis(leftDrive, rightDrive, &inertial, &odometry, WHEEL_DIAM, GEAR_RATIO);

// lib16868C::Intake intake(intakeMtr, ringDetect, hookDist);
lib16868C::Intake intake(intakeMtr, intakeEnc, ringOptical, {0.02, 0, 0.001}, 2);
//lib16868C::Arm arm(armMtrs, armEnc, {0.01325, 0, 0.0008});
lib16868C::Arm arm(armMtrs, armEnc, {0.013, 0, 0.000825});