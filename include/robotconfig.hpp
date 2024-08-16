#pragma once
#include "16868C/subsystems/chassis/odometry.hpp"
#include "okapi/impl/device/controller.hpp"
#include "16868C/devices/inertial.hpp"
#include "16868C/devices/motor.hpp"
#include "16868C/devices/motorGroup.hpp"
#include "16868C/subsystems/chassis/inline.hpp"
#include "16868C/subsystems/chassis/odometry.hpp"
#include "16868C/subsystems/intake.hpp"
#include "16868C/subsystems/arm.hpp"
#include "okapi/impl/device/opticalSensor.hpp"
#include "pros/adi.hpp"

using namespace okapi::literals;

// Controllers
extern okapi::Controller master;

// Ports
const int LEFT_FRONT = -4;
const int LEFT_MIDDLE = -5;
const int LEFT_REAR = -6;
const int RIGHT_FRONT = 1;
const int RIGHT_MIDDLE = 2;
const int RIGHT_REAR = 3;

const int INTAKE = 7;

const int ARM_LEFT = -8;
const int ARM_RIGHT = 9;

const int INERTIAL = 14;
const int HOOK_DISTANCE_SNSR = 13;
const int RING_OPTICAL_SNSR = 16;

const int VERT_ENC = 12;
const int HORT_ENC = 11;

const char MOGO_CLAMP = 'A';
const char HANG = 'B';
const char STICK = 'D';

const char AUTON_SELECTOR = 'C';

// Robot Constants
const okapi::QLength WHEEL_DIAM = 3.25_in;
const double GEAR_RATIO = 3/4.0;

// Motors
extern lib16868C::Motor leftFront;
extern lib16868C::Motor leftMiddle;
extern lib16868C::Motor leftRear;
extern lib16868C::Motor rightFront;
extern lib16868C::Motor rightMiddle;
extern lib16868C::Motor rightRear;
extern lib16868C::MotorGroup leftDrive;
extern lib16868C::MotorGroup rightDrive;

extern lib16868C::Motor intakeMtr;

extern lib16868C::Motor armLeft;
extern lib16868C::Motor armRight;
extern lib16868C::MotorGroup armMtrs;

// Pneumatics
extern pros::adi::Pneumatics clamp;
extern pros::adi::Pneumatics hang;
extern pros::adi::Pneumatics stick;

// Sensors
extern lib16868C::Inertial inertial;
extern okapi::DistanceSensor hookDist;
extern okapi::OpticalSensor ringDetect;

extern pros::adi::Potentiometer autonSelector;

extern lib16868C::Rotation vertRot;
extern lib16868C::Rotation hortRot;
extern lib16868C::TrackingWheel vertEnc;
extern lib16868C::TrackingWheel hortEnc;

// Subsystems
extern lib16868C::Odometry odometry;
extern lib16868C::Inline chassis;

extern lib16868C::Intake intake;
extern lib16868C::Arm arm;