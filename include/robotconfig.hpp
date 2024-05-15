#pragma once

#include "16868C/devices/inertial.hpp"
#include "16868C/devices/motor.hpp"
#include "16868C/devices/motorGroup.hpp"
#include "16868C/devices/pneumatic.hpp"
#include "16868C/devices/rotation.hpp"
#include "16868C/subsystems/chassis/inline.hpp"
#include "16868C/subsystems/chassis/odometry.hpp"

using namespace okapi::literals;

// Controllers
extern okapi::Controller master;

// Ports
const int LEFT_FRONT = -20;
const int LEFT_MIDDLE = -18;
const int LEFT_REAR = -10;
const int RIGHT_FRONT = 12;
const int RIGHT_MIDDLE = 11;
const int RIGHT_REAR = 1;
const int INERTIAL = 3;
const int VERT_ROT = -5;
const int HORT_ROT = -4;
const int BACK_DIST = 15;
const int LEFT_DIST = 1;
const int RIGHT_DIST = 5;

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

// Pneumatics

// Sensors
extern lib16868C::Inertial inertial;
extern lib16868C::Rotation vertRot;
extern lib16868C::Rotation hortRot;
extern lib16868C::TrackingWheel vertEnc;
extern lib16868C::TrackingWheel hortEnc;
extern okapi::DistanceSensor rightDistance;
extern okapi::DistanceSensor backDistance;
extern okapi::DistanceSensor leftDistance;
extern lib16868C::DistanceSensor rightDist;
extern lib16868C::DistanceSensor backDist;
extern lib16868C::DistanceSensor leftDist;

// Subsystems
extern lib16868C::Odometry odometry;
extern lib16868C::Inline chassis;