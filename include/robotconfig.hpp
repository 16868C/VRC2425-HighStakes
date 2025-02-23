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
const int LEFT_FRONT = -11;
const int LEFT_MIDDLE = -12;
const int LEFT_REAR = 13;
const int RIGHT_FRONT = 17;
const int RIGHT_MIDDLE = -16;
const int RIGHT_REAR = 18;

const int INTAKE_FIRST = -1;
const int INTAKE_SECOND = -2;

const int ARM_LEFT = -1;
const int ARM_RIGHT = 2;

const char MOGO_CLAMP = 'C';
const char HANG = 'A';
const char DOINKER = 'D';
const char PTO = 'G';
const char INTAKE_RAISER = 'E';
const char CLAW = 'B';

const int INERTIAL = 20;
const int HORT_ENC = 5;

const int INTAKE_ENC = 21;
const int RING_OPTICAL = 6;
const char RING_IR = 'H';
const int ARM_ENC = 3;

const char AUTON_SELECTOR = 'F';

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

extern lib16868C::Motor intakeFirst;
extern lib16868C::Motor intakeSecond;

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
extern lib16868C::Rotation hortRot;
extern lib16868C::TrackingWheel vertEnc;
extern lib16868C::TrackingWheel hortEnc;

extern lib16868C::Rotation intakeEnc;
extern okapi::OpticalSensor ringOptical;
extern pros::adi::LineSensor ringIR;
extern lib16868C::Rotation armEnc;

extern pros::adi::Potentiometer autonSelector;
extern lib16868C::AutonSelector auton;

// Subsystems
extern lib16868C::Odometry odometry;
extern lib16868C::Inline chassis;

extern lib16868C::Intake intake;
extern lib16868C::Arm arm;