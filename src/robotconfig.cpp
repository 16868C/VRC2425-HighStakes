#include "robotconfig.hpp"

okapi::Controller master(okapi::ControllerId::master);

okapi::Motor frontLeftMotor(FRONT_LEFT_MOTOR);
okapi::Motor rearLeftMotor(REAR_LEFT_MOTOR);
okapi::Motor frontRightMotor(-FRONT_RIGHT_MOTOR);
okapi::Motor rearRightMotor(-REAR_RIGHT_MOTOR);
okapi::MotorGroup leftDrive({frontLeftMotor, rearLeftMotor});
okapi::MotorGroup rightDrive({frontRightMotor, rearRightMotor});