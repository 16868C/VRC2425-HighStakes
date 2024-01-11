#pragma once

#include "okapi/api.hpp"
#include "16868C/devices/abstractEncoder.hpp"
#include "16868C/devices/inertial.hpp"
#include "16868C/devices/motor.hpp"
#include "16868C/devices/motorGroup.hpp"
#include "16868C/devices/pneumatic.hpp"
#include "16868C/devices/rotation.hpp"
#include "16868C/subsystems/chassis/inline.hpp"
#include "16868C/subsystems/chassis/odometry.hpp"
#include "16868C/subsystems/catapult.hpp"
#include "16868C/subsystems/intake.hpp"

using namespace okapi::literals;

// Controllers
extern okapi::Controller master;

// Ports
const int LEFT_FRONT = -13;
const int LEFT_REAR = -12;
const int LEFT_TOP = 14;
const int RIGHT_FRONT = 19;
const int RIGHT_REAR = 20;
const int RIGHT_TOP = -18;
const int KICKER = 2;
const int INTAKE = 7;

// Robot Constants
const okapi::QLength WHEEL_DIAM = 3.25_in;
const double GEAR_RATIO = 3/4.0;

// Motors
extern lib16868C::Motor leftFront;
extern lib16868C::Motor leftRear;
extern lib16868C::Motor leftTop;
extern lib16868C::Motor rightFront;
extern lib16868C::Motor rightRear;
extern lib16868C::Motor rightTop;
extern lib16868C::MotorGroup leftDrive;
extern lib16868C::MotorGroup rightDrive;
extern lib16868C::Motor intake;
extern lib16868C::Motor kicker;

// Subsystems
extern lib16868C::Odometry odometry;
extern lib16868C::Inline chassis;

// Sensors
extern lib16868C::Inertial inertial;