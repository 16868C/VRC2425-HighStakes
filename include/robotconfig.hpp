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
#include "16868C/util/autonSelector.hpp"
#include "okapi/impl/device/opticalSensor.hpp"
#include "pros/adi.hpp"

using namespace okapi::literals;

// Controllers
extern okapi::Controller master;

// Ports
const int LEFT_FRONT = -18;
const int LEFT_MIDDLE = 10;
const int LEFT_REAR = -9;
const int RIGHT_FRONT = 20;
const int RIGHT_MIDDLE = -8;
const int RIGHT_REAR = 7;

const int INTAKE = 13;

const int ARM_LEFT = -11;
const int ARM_RIGHT = 12;

const char MOGO_CLAMP = 'C';
const char HANG = 'A';
const char DOINKER = 'D';
const char PTO = 'G';
const char INTAKE_RAISER = 'E';
const char CLAW = 'B';

const int INERTIAL = 3;
const int VERT_ENC = 1;
const int HORT_ENC = 5;

const int DIST_LEFT = 18;
const int DIST_RIGHT = 9;
const int DIST_FRONT = 8;
const int DIST_REAR = 7;

const int DIST_INTAKE = 10;

const int INTAKE_ENC = -2;
const int RING_OPTICAL = 6;
const int ARM_ENC = -16;

const char AUTON_SELECTOR = 'F';

// Robot Constants
const okapi::QLength WHEEL_DIAM = 3.25_in;
const double GEAR_RATIO = 4/6.0;

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
extern pros::adi::Pneumatics doinker;
extern pros::adi::Pneumatics pto;
extern pros::adi::Pneumatics intakeRaiser;
extern pros::adi::Pneumatics claw;

// Sensors
extern lib16868C::Inertial inertial;
extern lib16868C::Rotation vertRot;
extern lib16868C::Rotation hortRot;
extern lib16868C::TrackingWheel vertEnc;
extern lib16868C::TrackingWheel hortEnc;

extern okapi::DistanceSensor frontDistance;
extern okapi::DistanceSensor rearDistance;
extern okapi::DistanceSensor leftDistance;
extern okapi::DistanceSensor rightDistance;
extern lib16868C::DistanceSensor frontDist;
extern lib16868C::DistanceSensor rearDist;
extern lib16868C::DistanceSensor leftDist;
extern lib16868C::DistanceSensor rightDist;

extern lib16868C::Rotation intakeEnc;
extern okapi::OpticalSensor ringOptical;
extern lib16868C::Rotation armEnc;

extern pros::adi::Potentiometer autonSelector;
extern lib16868C::AutonSelector auton;

// Subsystems
extern lib16868C::Odometry odometry;
extern lib16868C::Inline chassis;

extern lib16868C::Intake intake;
extern lib16868C::Arm arm;