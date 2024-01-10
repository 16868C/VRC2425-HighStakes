#include "motorGroup.hpp"

using namespace lib16868C;

MotorGroup::MotorGroup(const std::initializer_list<Motor&> mtrs) : mtrs(mtrs) {
	if (mtrs.size() == 0) std::cerr << "[MotorGroup] No motors in group" << std::endl;
}

void MotorGroup::moveVoltage(double volts) {
	for (Motor& mtr : mtrs) mtr.moveVoltage(volts);
}
void MotorGroup::moveVelocity(double vel) {
	for (Motor& mtr : mtrs) mtr.moveVelocity(vel);
}

double MotorGroup::getPosition() {
	return mtrs[0].getPosition();
}

void MotorGroup::tarePosition() {
	for (Motor& mtr : mtrs) mtr.tarePosition();
}

okapi::AbstractMotor::gearset MotorGroup::getGearing() {
	return mtrs[0].getGearing();
}

double MotorGroup::getActualVelocity() {
	return mtrs[0].getActualVelocity();
}
double MotorGroup::getTemperature() {
	return mtrs[0].getTemperature();
}

void MotorGroup::setBrakeMode(okapi::AbstractMotor::brakeMode mode) {
	for (Motor& mtr : mtrs) mtr.setBrakeMode(mode);
}

int MotorGroup::getSize() {
	return mtrs.size();
}