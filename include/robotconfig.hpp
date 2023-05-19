#pragma once
#include "okapi/api.hpp"

// Ports
const int FRONT_LEFT_MOTOR = 1;
const int REAR_LEFT_MOTOR = 2;
const int FRONT_RIGHT_MOTOR = 3;
const int REAR_RIGHT_MOTOR = 4;

// Constants
const int WHEEL_DIAMETER = 4;
const int CHASSIS_WIDTH = 12;
const int MAX_RPM = 200;

// Controllers
extern okapi::Controller master;

// Motors
extern okapi::Motor frontLeftMotor;
extern okapi::Motor rearLeftMotor;
extern okapi::Motor frontRightMotor;
extern okapi::Motor rearRightMotor;
extern okapi::MotorGroup leftDrive;
extern okapi::MotorGroup rightDrive;

// Sensors
extern pros::Imu inertial;