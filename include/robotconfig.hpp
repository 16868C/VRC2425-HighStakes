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
#include "16868C/subsystems/kicker.hpp"

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
const int INERTIAL = 17;
const int MIDDLE_ROT = 6;
const int BACK_DIST = 15;
const int LEFT_DIST = 1;
const int RIGHT_DIST = 5;
const char HORI_HANG = 'A';
const char LEFT_WING = 'C';
const char RIGHT_WING = 'B';
const char VERT_WINGS = 'D';

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
extern lib16868C::Motor kickerMtr;

// Pneumatics
extern lib16868C::Pneumatic horiHang;
extern lib16868C::Pneumatic leftWing;
extern lib16868C::Pneumatic rightWing;
extern lib16868C::Pneumatic vertWings;

// Sensors
extern lib16868C::Inertial inertial;
extern lib16868C::Rotation middleRot;
extern lib16868C::TrackingWheel leftEnc;
extern lib16868C::TrackingWheel rightEnc;
extern lib16868C::TrackingWheel middleEnc;
extern okapi::DistanceSensor rightDistance;
extern okapi::DistanceSensor backDistance;
extern okapi::DistanceSensor leftDistance;
extern lib16868C::DistanceSensor rightDist;
extern lib16868C::DistanceSensor backDist;
extern lib16868C::DistanceSensor leftDist;

// Subsystems
extern lib16868C::Odometry odometry;
extern lib16868C::Inline chassis;
extern lib16868C::Kicker kicker;