#include "okapi/api.hpp"
#include "catapult.hpp"
#include "16868Z/util/util.hpp"

using namespace lib16868C;

void lib16868C::CataMain(void* param) {
	Catapult* cata = (Catapult*) param;

	uint32_t time = pros::millis();
	while (true) {
		if (cata->cataState != CataState::SETTLED) cata->mtr.moveVoltage(12000);
		else cata->stop();

		if (cata->cataState == CataState::FIRING) {
			if (cata->distance.get() < 90) cata->cataState = CataState::SETTLED;
		} else if (cata->cataState == CataState::INTAKE) {
			if (cata->distance.get() < 120) cata->cataState = CataState::SETTLED;
		}

		pros::Task::delay_until(&time, 10);
	}
}

Catapult::Catapult(okapi::MotorGroup& mtr, okapi::DistanceSensor& distance) : mtr(mtr), distance(distance) {
	mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	pros::c::task_create(CataMain, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Catapult Control");
}
Catapult::~Catapult() {
	stop();
}

void Catapult::fire() {
	cataState = CataState::FIRING;
}
void Catapult::intake() {
	cataState = CataState::INTAKE;
}
void Catapult::stop() {
	mtr.moveVoltage(0);
	cataState = CataState::SETTLED;
}

void Catapult::moveVelocity(double vel) {
	mtr.moveVelocity(vel);
}

bool Catapult::isSettled() {
	return cataState == CataState::SETTLED;
}

void Catapult::waitForSettled() {
	do pros::delay(10);
	while (!isSettled());
}