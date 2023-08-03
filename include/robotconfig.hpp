#pragma once
#include "okapi/api.hpp"
#include "16868Z/subsystems/chassis/inline.hpp"
#include "16868Z/subsystems/intake.hpp"
#include "16868Z/subsystems/turret.hpp"
#include "16868Z/devices/pneumatic.hpp"

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

// Subsystems
extern lib16868Z::Inline chassis;
extern lib16868Z::Intake intake;
extern lib16868Z::Turret turret;

// Sensors
extern pros::Imu inertial;
extern okapi::DistanceSensor distance;