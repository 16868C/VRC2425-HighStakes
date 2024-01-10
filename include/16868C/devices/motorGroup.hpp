#pragma once
#include "okapi/api.hpp"
#include "16868C/devices/motor.hpp"
#include "16868C/util/math.hpp"

namespace lib16868C {
class MotorGroup {
	public:
	MotorGroup(const std::initializer_list<Motor&> mtrs);

	void moveVoltage(double volts);
	void moveVelocity(double vel);

	double getPosition();

	void tarePosition();

	double getActualVelocity();
	double getTemperature();

	okapi::AbstractMotor::gearset getGearing();

	void setBrakeMode(okapi::AbstractMotor::brakeMode mode);

	int getSize();

	private:
	std::vector<Motor&> mtrs;
};
} // namespace lib16868C