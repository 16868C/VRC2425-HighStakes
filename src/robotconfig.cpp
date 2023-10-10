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

lib16868C::Inline chassis(leftDrive, rightDrive, inertial, WHEEL_DIAMETER, GEAR_RATIO);
lib16868C::Intake intake(frontIntake, rearIntake, distance, mouth, 1000);
lib16868C::Turret turret(turretMotor, inertial, 36/234.0);

pros::Imu inertial(15);
okapi::DistanceSensor distance(18);

lib16868C::Pneumatic wings('C');
lib16868C::Pneumatic mouth('B');
lib16868C::Pneumatic clothesline('A');
lib16868C::Pneumatic turretShifter('H');
#endif

#ifdef ANSONBOT
okapi::Motor frontLeftMotor(FRONT_LEFT_PORT, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor rearLeftMotor(REAR_LEFT_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor frontRightMotor(FRONT_RIGHT_PORT, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor rearRightMotor(REAR_RIGHT_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);
okapi::MotorGroup leftDrive({frontLeftMotor, rearLeftMotor});
okapi::MotorGroup rightDrive({frontRightMotor, rearRightMotor});

okapi::Motor intakeMtr(INTAKE_PORT, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::counts);

okapi::Motor cataMtr1(CATA_PORT_1, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts);
okapi::Motor cataMtr2(CATA_PORT_2, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::counts);
okapi::MotorGroup cataMtrs({cataMtr1, cataMtr2});

lib16868C::Inline chassis(leftDrive, rightDrive, inertial, WHEEL_DIAMETER, GEAR_RATIO);
lib16868C::Catapult catapult(cataMtrs, cataDist);

pros::Imu inertial(INERTIAL_PORT);

lib16868C::Pneumatic tom('A');
okapi::DistanceSensor cataDist(CATA_DIST_PORT);
#endif