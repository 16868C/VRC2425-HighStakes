#pragma once
#include "okapi/impl/device/controller.hpp"
#include "16868C/devices/inertial.hpp"
#include "16868C/devices/motor.hpp"
#include "16868C/devices/motorGroup.hpp"
#include "16868C/devices/pneumatic.hpp"
#include "16868C/subsystems/chassis/inline.hpp"

using namespace okapi::literals;

// Controllers
extern okapi::Controller master;

// Ports
const int LEFT_FRONT = 11;
const int LEFT_WEAK = 7;
const int LEFT_MIDDLE = 13;
const int LEFT_REAR = -14;
const int RIGHT_FRONT = -5;
const int RIGHT_WEAK = -19;
const int RIGHT_MIDDLE = 18;
const int RIGHT_REAR = -17;
const int INTAKE = 2;
const int INERTIAL = 1;
const char LEFT_WING = 'A';
const char RIGHT_WING = 'B';
const char INTAKE_RAISER = 'D';
const char PARK = 'E';

// Robot Constants
const okapi::QLength WHEEL_DIAM = 2.75_in;
const double GEAR_RATIO = 3/3.0;

// Motors
extern lib16868C::Motor leftFront;
extern lib16868C::Motor leftWeak;
extern lib16868C::Motor leftMiddle;
extern lib16868C::Motor leftRear;
extern lib16868C::Motor rightFront;
extern lib16868C::Motor rightWeak;
extern lib16868C::Motor rightMiddle;
extern lib16868C::Motor rightRear;
extern lib16868C::MotorGroup leftDrive;
extern lib16868C::MotorGroup rightDrive;
extern lib16868C::Motor intake;

// Pneumatics
// extern lib16868C::Pneumatic horiHang;
extern lib16868C::Pneumatic leftWing;
extern lib16868C::Pneumatic rightWing;
extern lib16868C::Pneumatic intakeRaiser;
extern lib16868C::Pneumatic park;
// extern lib16868C::Pneumatic vertWings;

// Sensors
extern lib16868C::Inertial inertial;

// Subsystems
// extern lib16868C::Odometry odometry;
extern lib16868C::Inline chassis;