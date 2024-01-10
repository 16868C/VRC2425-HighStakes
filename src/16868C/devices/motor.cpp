#include "motor.hpp"

using namespace lib16868C;

Motor::Motor(int port, okapi::AbstractMotor::gearset gearset)
	: okapi::Motor(std::abs(port), port < 0, gearset, okapi::AbstractMotor::encoderUnits::counts) {}
Motor::Motor(uint port, bool reversed, okapi::AbstractMotor::gearset gearset)
	: okapi::Motor(port, reversed, gearset, okapi::AbstractMotor::encoderUnits::counts) {}

double Motor::getPosition() {
	double ticks = okapi::Motor::getPosition();
	
	if (std::isinf(ticks)) {
		std::cerr << "[Motor] Position of Infinity" << std::endl;
		while (std::isinf(ticks)) ticks = okapi::Motor::getPosition();
	}

	return ticks;
}