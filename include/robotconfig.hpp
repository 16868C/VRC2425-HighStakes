#pragma once
#include "okapi/api.hpp"

// Ports
const int FRONT_LEFT_MOTOR = 17;
const int MIDDLE_LEFT_MOTOR = 4;
const int REAR_LEFT_MOTOR = 19;
const int FRONT_RIGHT_MOTOR = 2;
const int MIDDLE_RIGHT_MOTOR = 1;
const int REAR_RIGHT_MOTOR = 18;

// Constants
const double WHEEL_DIAMETER = 2.75;
const int CHASSIS_WIDTH = 12;
const int MAX_RPM = 600;

// Controllers
extern okapi::Controller master;

// Motors
extern okapi::Motor frontLeftMotor;
extern okapi::Motor middleLeftMotor;
extern okapi::Motor rearLeftMotor;
extern okapi::Motor frontRightMotor;
extern okapi::Motor middleRightMotor;
extern okapi::Motor rearRightMotor;
extern okapi::MotorGroup leftDrive;
extern okapi::MotorGroup rightDrive;

// Sensors
extern pros::Imu inertial;
extern okapi::ADIEncoder leftEncoder;
extern okapi::ADIEncoder rightEncoder;