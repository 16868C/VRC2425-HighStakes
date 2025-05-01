#include "16868C/devices/rotation.hpp"

using namespace lib16868C;

Rotation::Rotation(int port) : pros::Rotation(port) {
	tpr = 360;
}
Rotation::Rotation(uint port, bool reversed) : pros::Rotation(port * (reversed ? 1 : -1)) {
	tpr = 360;
}

double Rotation::get() {
	return pros::Rotation::get_position() / 100.0;
}
void Rotation::resetZero() {
	pros::Rotation::reset_position();
}

void Rotation::setPosition(double ticks) {
	pros::Rotation::set_position(ticks * 100);
}

double Rotation::getVelocity() {
	#define USE_SENSOR_VELOCITY
	//#define USE_ENCODER_VELOCITY

	#ifdef USE_SENSOR_VELOCITY
	return pros::Rotation::get_velocity() / 100.0;
	#endif

	#ifdef USE_ENCODER_VELOCITY
	double dTicks = get() - prevTicks;
	uint dt = pros::millis() - prevTime;
	prevTicks = get();
	prevTime = pros::millis();

	return dTicks / dt * 1000; // Multiply by 1000 to convert from ticks/ms to ticks/s
	#endif
}