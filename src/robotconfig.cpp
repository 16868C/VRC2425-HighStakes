#include "robotconfig.hpp"
#include <memory>

okapi::Controller master(okapi::ControllerId::master);

#ifdef ODOMBOT
okapi::Motor frontLeftMotor(FRONT_LEFT_PORT, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor rearLeftMotor(REAR_LEFT_PORT, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor frontRightMotor(FRONT_RIGHT_PORT, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor rearRightMotor(REAR_RIGHT_PORT, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts);
okapi::MotorGroup leftDrive({frontLeftMotor, rearLeftMotor});
okapi::MotorGroup rightDrive({frontRightMotor, rearRightMotor});

lib16868C::Rotation leftRotation(LEFT_ROTATION_PORT);
lib16868C::Rotation rightRotation(RIGHT_ROTATION_PORT, true);
lib16868C::Rotation rearRotation(REAR_ROTATION_PORT, true);
lib16868C::Rotation driveRotation(DRIVE_ROTATION_PORT);

pros::Imu inertial(INERTIAL_PORT);
pros::Gps gps(GPS_PORT);

lib16868C::Odometry odomThreeEnc({leftRotation, rightRotation, rearRotation}, {ROTATION_DIAMETER, ROTATION_DIAMETER, ROTATION_TRACK, ROTATION_DIAMETER, ROTATION_RADIUS});
lib16868C::Odometry odomTwoEnc({leftRotation, rearRotation}, inertial, {ROTATION_DIAMETER, ROTATION_DIAMETER, ROTATION_RADIUS});
lib16868C::Odometry odomDriveEnc({driveRotation, rearRotation}, inertial, {DRIVE_DIAMETER, ROTATION_DIAMETER, ROTATION_RADIUS});

lib16868C::Inline chassis(leftDrive, rightDrive, inertial, DRIVE_DIAMETER, GEAR_RATIO);
#endif

#ifdef ANSONBOT
okapi::Motor frontLeftMotor(FRONT_LEFT_PORT, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor middleLeftMotor(MIDDLE_LEFT_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor rearLeftMotor(REAR_LEFT_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor frontRightMotor(FRONT_RIGHT_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor middleRightMotor(MIDDLE_RIGHT_PORT, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor rearRightMotor(REAR_RIGHT_PORT, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::MotorGroup leftDrive({frontLeftMotor, middleLeftMotor, rearLeftMotor});
okapi::MotorGroup rightDrive({frontRightMotor, middleRightMotor, rearRightMotor});

okapi::Motor intakeMtr(INTAKE_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::MotorGroup intakeMtrs({intakeMtr});

okapi::Motor cataMtr(CATA_PORT, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::counts);
okapi::MotorGroup cataMtrs({cataMtr});

lib16868C::Inline chassis(leftDrive, rightDrive, inertial, WHEEL_DIAMETER, GEAR_RATIO);
lib16868C::Catapult catapult(cataMtrs, cataEnc);

pros::Imu inertial(INERTIAL_PORT);

// lib16868C::Pneumatic tom('A');
lib16868C::Pneumatic wings(WING_PORT);
lib16868C::Rotation cataEnc(CATA_ENC_PORT);
#endif