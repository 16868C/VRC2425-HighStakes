#include "robotconfig.hpp"
#include <memory>

okapi::Controller master(okapi::ControllerId::master);

lib16868C::Motor leftFront(LEFT_FRONT, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor leftRear(LEFT_REAR, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor leftTop(LEFT_TOP, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor rightFront(RIGHT_FRONT, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor rightRear(RIGHT_REAR, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor rightTop(RIGHT_TOP, okapi::AbstractMotor::gearset::blue);
lib16868C::MotorGroup leftDrive({leftFront, leftRear, leftTop});
lib16868C::MotorGroup rightDrive({rightFront, rightRear, rightTop});
lib16868C::Motor intake(INTAKE, okapi::AbstractMotor::gearset::blue);
lib16868C::Motor kicker(KICKER, okapi::AbstractMotor::gearset::green);

lib16868C::Odometry odometry({nullptr, nullptr, nullptr}, {});
lib16868C::Inline chassis(leftDrive, rightDrive, inertial, odometry, WHEEL_DIAM, GEAR_RATIO);

lib16868C::Inertial inertial(1);