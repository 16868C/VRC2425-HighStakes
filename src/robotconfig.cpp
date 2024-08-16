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
pros::adi::Pneumatics clamp(MOGO_CLAMP, false);
pros::adi::Pneumatics hang(HANG, false);
pros::adi::Pneumatics stick(STICK, false);

// Sensors
lib16868C::Inertial inertial(INERTIAL, &master);
okapi::DistanceSensor hookDist(HOOK_DISTANCE_SNSR);
okapi::OpticalSensor ringDetect(RING_OPTICAL_SNSR);

pros::adi::Potentiometer autonSelector(AUTON_SELECTOR, pros::E_ADI_POT_V2);

lib16868C::Rotation vertRot(VERT_ENC);
lib16868C::Rotation hortRot(HORT_ENC);
lib16868C::TrackingWheel vertEnc(&vertRot, 2_in, 0.5_in);
lib16868C::TrackingWheel hortEnc(&hortRot, 2_in, 3.75_in);

// Subsystems
lib16868C::Odometry odometry(
	std::array<lib16868C::TrackingWheel, 3>{vertEnc, {}, hortEnc},
	std::array<lib16868C::DistanceSensor, 4>{{{}, {}, {}, {}}},
	&inertial);
lib16868C::Inline chassis(leftDrive, rightDrive, &inertial, &odometry, WHEEL_DIAM, GEAR_RATIO);

lib16868C::Intake intake(intakeMtr, ringDetect, hookDist);
lib16868C::Arm arm(armMtrs);