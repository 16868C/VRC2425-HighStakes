#include "trackingWheel.hpp"
#include "16868C/util/util.hpp"

using namespace lib16868C;

TrackingWheel::TrackingWheel() {}
TrackingWheel::TrackingWheel(Rotation* enc, okapi::QLength wheelDiameter, okapi::QLength offset, double gearRatio) : enc(enc) {
	this->wheelDiameter = wheelDiameter.convert(okapi::inch);
	this->offset = offset.convert(okapi::inch);
	this->gearRatio = gearRatio;
	
	type = TrackingWheelType::ROTATION;
}
TrackingWheel::TrackingWheel(OpticalEncoder* enc, okapi::QLength wheelDiameter, okapi::QLength offset, double gearRatio) : enc(enc) {
	this->wheelDiameter = wheelDiameter.convert(okapi::inch);
	this->offset = offset.convert(okapi::inch);
	this->gearRatio = gearRatio;

	type = TrackingWheelType::OPTICAL_ENCODER;
}
TrackingWheel::TrackingWheel(Motor* enc, okapi::QLength wheelDiameter, okapi::QLength offset, double gearRatio) : enc(enc) {
	this->wheelDiameter = wheelDiameter.convert(okapi::inch);
	this->offset = offset.convert(okapi::inch);
	this->gearRatio = gearRatio;

	type = TrackingWheelType::MOTOR;
}
TrackingWheel::TrackingWheel(MotorGroup* enc, okapi::QLength wheelDiameter, okapi::QLength offset, double gearRatio) : enc(enc) {
	this->wheelDiameter = wheelDiameter.convert(okapi::inch);
	this->offset = offset.convert(okapi::inch);
	this->gearRatio = gearRatio;

	type = TrackingWheelType::MOTOR_GROUP;
}

double TrackingWheel::getDist() {
	return (getRaw() / enc->getTPR() * wheelDiameter * M_PI) / gearRatio;
}
double TrackingWheel::getRaw() {
	return enc->get();
}
double TrackingWheel::getOffset() {
	return offset;
}
void TrackingWheel::reset() {
	enc->resetZero();
}

TrackingWheelType TrackingWheel::getType() {
	return type;
}