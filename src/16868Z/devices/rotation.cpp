#include "16868Z/devices/rotation.hpp"

using namespace lib16868Z;

Rotation::Rotation(int port) : okapi::RotationSensor(port, Util::sgn(port) > 0 ? true : false) {
	tpr = 360;
}
Rotation::Rotation(int port, bool reversed) : okapi::RotationSensor(port, reversed) {
	tpr = 360;
}

double Rotation::getVelocity() {
	#define USE_OKAPI_VELOCITY
	//#define USE_ENCODER_VELOCITY

	#ifdef USE_OKAPI_VELOCITY
	return okapi::RotationSensor::getVelocity();
	#endif

	#ifdef USE_ENCODER_VELOCITY
	double dTicks = get() - prevTicks;
	uint dt = pros::millis() - prevTime;
	prevTicks = get();
	prevTime = pros::millis();

	return dTicks / dt * 1000; // Multiply by 1000 to convert from ticks/ms to ticks/s
	#endif
}