#pragma once
#define ODOMBOT
// #define ANSONBOT

#include "okapi/api.hpp"
#include "16868C/subsystems/chassis/odometry.hpp"
#include "16868C/subsystems/chassis/inline.hpp"
#include "16868C/subsystems/intake.hpp"
#include "16868C/subsystems/catapult.hpp"
#include "16868C/devices/abstractEncoder.hpp"
#include "16868C/devices/pneumatic.hpp"
#include "16868C/devices/rotation.hpp"

using namespace okapi::literals;

#ifdef ODOMBOT
// Ports
const int FRONT_LEFT_PORT = 8;
const int REAR_LEFT_PORT = 2;
const int FRONT_RIGHT_PORT = 11;
const int REAR_RIGHT_PORT = 19;

const int LEFT_ROTATION_PORT = 10;
const int RIGHT_ROTATION_PORT = 15;
const int REAR_ROTATION_PORT = 12;
const int DRIVE_ROTATION_PORT = 13;
const int INERTIAL_PORT = 1;
const int GPS_PORT = 4;

// Constants
const okapi::QLength DRIVE_DIAMETER = 4_in;
const okapi::QLength DRIVE_TRACK = 10.625_in;
const okapi::QLength ROTATION_TRACK = 10.5_in;
const okapi::QLength ROTATION_RADIUS = 1.5_in;
const okapi::QLength ROTATION_DIAMETER = 2.75_in;

const int MAX_RPM = 200;
const double GEAR_RATIO = 1;

// Controllers
extern okapi::Controller master;

// Motors
extern okapi::Motor frontLeftMotor;
extern okapi::Motor rearLeftMotor;
extern okapi::Motor frontRightMotor;
extern okapi::Motor rearRightMotor;
extern okapi::MotorGroup leftDrive;
extern okapi::MotorGroup rightDrive;

// Subsystems
extern lib16868C::Odometry odomThreeEnc;
extern lib16868C::Odometry odomTwoEnc;
extern lib16868C::Odometry odomDriveEnc;
extern lib16868C::Inline chassis;

// Sensors
extern pros::Imu inertial;
extern lib16868C::Rotation leftRotation;
extern lib16868C::Rotation rightRotation;
extern lib16868C::Rotation rearRotation;
extern lib16868C::Rotation driveRotation;
extern pros::Gps gps;
#endif

#ifdef ANSONBOT
// Ports
const int FRONT_LEFT_PORT = 2;
const int REAR_LEFT_PORT = 1;
const int FRONT_RIGHT_PORT = 16;
const int REAR_RIGHT_PORT = 11;
const int INTAKE_1_PORT = 4;
const int INTAKE_2_PORT = 8;
const int CATA_PORT = 9;

const int INERTIAL_PORT = 19;
const int CATA_DIST_PORT = 20;

const char WING_PORT = 'A';

// Constants
const okapi::QLength WHEEL_DIAMETER = 3.25_in;
const int CHASSIS_WIDTH = 12;
const int MAX_RPM = 600;
const double GEAR_RATIO = 3/5.0;

// Controllers
extern okapi::Controller master;

// Motors
extern okapi::Motor frontLeftMotor;
extern okapi::Motor rearLeftMotor;
extern okapi::Motor frontRightMotor;
extern okapi::Motor rearRightMotor;
extern okapi::MotorGroup leftDrive;
extern okapi::MotorGroup rightDrive;

extern okapi::Motor intakeMtr1;
extern okapi::Motor intakeMtr2;
extern okapi::MotorGroup intakeMtrs;

extern okapi::Motor cataMtr;
extern okapi::MotorGroup cataMtrs;

// Pneumatics
// extern lib16868C::Pneumatic tom;
extern lib16868C::Pneumatic wings;

// Subsystems
extern lib16868C::Inline chassis;
extern lib16868C::Catapult catapult;

// Sensors
extern pros::Imu inertial;
extern okapi::DistanceSensor cataDist;
#endif