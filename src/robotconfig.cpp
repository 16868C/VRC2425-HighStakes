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
lib16868C::Motor kickerMtr(KICKER, okapi::AbstractMotor::gearset::green);

lib16868C::Pneumatic horiHang(HORI_HANG);
lib16868C::Pneumatic leftWing(LEFT_WING);
lib16868C::Pneumatic rightWing(RIGHT_WING);
lib16868C::Pneumatic vertWings(VERT_WINGS);

lib16868C::Inertial inertial(INERTIAL);
okapi::DistanceSensor frontDist(FRONT_DIST);
okapi::DistanceSensor backDist(BACK_DIST);
okapi::DistanceSensor leftDist(LEFT_DIST);
okapi::DistanceSensor rightDist(RIGHT_DIST);

// lib16868C::DistanceSnsrs dists 
lib16868C::Odometry odometry(lib16868C::DistanceSnsrs{
	std::make_shared<okapi::DistanceSensor>(FRONT_DIST),
	std::make_shared<okapi::DistanceSensor>(RIGHT_DIST),
	std::make_shared<okapi::DistanceSensor>(BACK_DIST),
	std::make_shared<okapi::DistanceSensor>(LEFT_DIST),
}, inertial, {5_in, 5_in, 4.5_in, 4.5_in});
lib16868C::Inline chassis(leftDrive, rightDrive, inertial, odometry, WHEEL_DIAM, GEAR_RATIO);
lib16868C::Kicker kicker(kickerMtr);