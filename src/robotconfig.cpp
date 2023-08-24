#include "robotconfig.hpp"

okapi::Controller master(okapi::ControllerId::master);

#ifdef MOABOT
okapi::Motor frontLeftMotor(FRONT_LEFT_MOTOR, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor rearLeftMotor(REAR_LEFT_MOTOR, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor frontRightMotor(FRONT_RIGHT_MOTOR, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor rearRightMotor(REAR_RIGHT_MOTOR, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::MotorGroup leftDrive({frontLeftMotor, rearLeftMotor});
okapi::MotorGroup rightDrive({frontRightMotor, rearRightMotor});

okapi::Motor frontIntake(FRONT_INTAKE_MOTOR, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor rearIntake(REAR_INTAKE_MOTOR, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);

okapi::Motor turretMotor(TURRET_MOTOR, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);

lib16868Z::Inline chassis(leftDrive, rightDrive, inertial, WHEEL_DIAMETER, GEAR_RATIO);
lib16868Z::Intake intake(frontIntake, rearIntake, distance, mouth, 1000);
lib16868Z::Turret turret(turretMotor, inertial, 36/234.0);

pros::Imu inertial(15);
okapi::DistanceSensor distance(18);

lib16868Z::Pneumatic wings('C');
lib16868Z::Pneumatic mouth('B');
lib16868Z::Pneumatic clothesline('A');
lib16868Z::Pneumatic turretShifter('H');
#endif

#ifdef ANSONBOT
okapi::Motor frontLeftMotor(FRONT_LEFT_MOTOR, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor rearLeftMotor(REAR_LEFT_MOTOR, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor frontRightMotor(FRONT_RIGHT_MOTOR, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor rearRightMotor(REAR_RIGHT_MOTOR, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::MotorGroup leftDrive({frontLeftMotor, rearLeftMotor});
okapi::MotorGroup rightDrive({frontRightMotor, rearRightMotor});

okapi::Motor intakeMtr(INTAKE_MOTOR, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);

okapi::Motor catapultMotor(CATA_MOTOR, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts);

lib16868Z::Inline chassis(leftDrive, rightDrive, inertial, WHEEL_DIAMETER, GEAR_RATIO);
lib16868Z::Catapult catapult(catapultMotor, cataLimit);

pros::Imu inertial(15);

lib16868Z::Pneumatic tom('A');
pros::ADIDigitalIn cataLimit('B');
#endif