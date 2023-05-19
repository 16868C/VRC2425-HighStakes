#include "16868X/devices/motorGroup.hpp"

using namespace lib16868X;

MotorGroup::MotorGroup(const MotorGroup& motorGroup) {
	copied = true;

	Motor::addMotor(std::make_shared<MotorGroup>(*this));
	motors = motorGroup.motors;
	cartridge = motorGroup.cartridge;

	if (mtrManagerTask == NULL)
		mtrManagerTask = pros::c::task_create(mtrManager, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Motor Manager");
}
MotorGroup::MotorGroup(std::initializer_list<Motor> motors) {
	AbstractMotor::addMotor(std::make_shared<MotorGroup>(*this));
	cartridge = motors.begin()->getCartridge();
	for (auto& motor : motors) {
		if (motor.getCartridge() != cartridge)
			throw std::invalid_argument("All motors in a motor group must have the same cartridge");
		this->motors.push_back(std::make_shared<Motor>(motor));
	}

	if (mtrManagerTask == NULL)
		mtrManagerTask = pros::c::task_create(mtrManager, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Motor Manager");
}
MotorGroup::MotorGroup(std::vector<Motor> motors) {
	AbstractMotor::addMotor(std::make_shared<MotorGroup>(*this));
	cartridge = motors.begin()->getCartridge();
	for (auto& motor : motors) {
		if (motor.getCartridge() != this->cartridge)
			throw std::invalid_argument("All motors in a motor group must have the same cartridge");
		this->motors.push_back(std::make_shared<Motor>(motor));
	}

	if (mtrManagerTask == NULL)
		mtrManagerTask = pros::c::task_create(mtrManager, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Motor Manager");
}