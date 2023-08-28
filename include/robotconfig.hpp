#pragma once
// #define MOABOT
#define ANSONBOT

#include "okapi/api.hpp"
#include "16868Z/subsystems/chassis/inline.hpp"
#include "16868Z/subsystems/intake.hpp"
#include "16868Z/subsystems/turret.hpp"
#include "16868Z/subsystems/catapult.hpp"
#include "16868Z/devices/pneumatic.hpp"

#ifdef MOABOT
// Ports
const int FRONT_LEFT_MOTOR = 11;
const int REAR_LEFT_MOTOR = 2;
const int FRONT_RIGHT_MOTOR = 14;
const int REAR_RIGHT_MOTOR = 1;
const int FRONT_INTAKE_MOTOR = 19;
const int REAR_INTAKE_MOTOR = 13;
const int TURRET_MOTOR = 9;

// Constants
const double WHEEL_DIAMETER = 4.0;
const int CHASSIS_WIDTH = 12;
const int MAX_RPM = 600;
const double GEAR_RATIO = 3/6.0;

// Controllers
extern okapi::Controller master;

// Motors
extern okapi::Motor frontLeftMotor;
extern okapi::Motor rearLeftMotor;
extern okapi::Motor frontRightMotor;
extern okapi::Motor rearRightMotor;
extern okapi::MotorGroup leftDrive;
extern okapi::MotorGroup rightDrive;

extern okapi::Motor frontIntake;
extern okapi::Motor rearIntake;

extern okapi::Motor turretMotor;

// Pneumatics
extern lib16868Z::Pneumatic wings;
extern lib16868Z::Pneumatic mouth;
extern lib16868Z::Pneumatic clothesline;
extern lib16868Z::Pneumatic turretShifter;

// Subsystems
extern lib16868Z::Inline chassis;
extern lib16868Z::Intake intake;
extern lib16868Z::Turret turret;

// Sensors
extern pros::Imu inertial;
extern okapi::DistanceSensor distance;
#endif

#ifdef ANSONBOT
// Ports
const int FRONT_LEFT_PORT = 1;
const int REAR_LEFT_PORT = 3;
const int FRONT_RIGHT_PORT = 11;
const int REAR_RIGHT_PORT = 13;
const int INTAKE_PORT = 20;
const int CATA_PORT_1 = 16;
const int CATA_PORT_2 = 18;

const int INERTIAL_PORT = 5;
const int CATA_DIST_PORT = 19;

// Constants
const double WHEEL_DIAMETER = 3.25;
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

extern okapi::Motor intakeMtr;

extern okapi::Motor cataMtr1;
extern okapi::Motor cataMtr2;
extern okapi::MotorGroup cataMtrs;

// Pneumatics
extern lib16868Z::Pneumatic tom;

// Subsystems
extern lib16868Z::Inline chassis;
extern lib16868Z::Catapult catapult;

// Sensors
extern pros::Imu inertial;
extern okapi::DistanceSensor cataDist;
#endif