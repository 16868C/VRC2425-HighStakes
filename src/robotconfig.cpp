#include "robotconfig.hpp"

okapi::Controller master(okapi::ControllerId::master);

okapi::Motor frontLeftMotor(FRONT_LEFT_MOTOR, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor middleLeftMotor(MIDDLE_LEFT_MOTOR, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor rearLeftMotor(REAR_LEFT_MOTOR, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor frontRightMotor(FRONT_RIGHT_MOTOR, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor middleRightMotor(MIDDLE_RIGHT_MOTOR, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor rearRightMotor(REAR_RIGHT_MOTOR, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::MotorGroup leftDrive({frontLeftMotor, middleLeftMotor, rearLeftMotor});
okapi::MotorGroup rightDrive({frontRightMotor, middleRightMotor, rearRightMotor});

pros::Imu inertial(13);
okapi::ADIEncoder leftEncoder('G', 'H');
okapi::ADIEncoder rightEncoder('E', 'F', true);