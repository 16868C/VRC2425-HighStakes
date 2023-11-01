#include "okapi/api.hpp"
#include "16868C/subsystems/catapult.hpp"
#include "16868C/util/util.hpp"

using namespace lib16868C;

void lib16868C::CataMain(void* param) {
	Catapult* cata = (Catapult*) param;

	bool fireDebounce = false;

	uint32_t time = pros::millis();
	while (true) {
		if (cata->cataState == CataState::FIRING) {
			if (cata->enc.get() > 55 && !fireDebounce) {
				cata->cataState = CataState::SETTLED;
				fireDebounce = true;
			} else if (cata->enc.get() < 55 && fireDebounce) fireDebounce = false;
		} else if (cata->cataState == CataState::INTAKE) {
			if (cata->enc.get() > 35 && cata->enc.get() < 45 && cata->enc.getVelocity() >= 0) cata->cataState = CataState::SETTLED;
		}

		if (cata->cataState != CataState::SETTLED || cata->cataState == CataState::MATCHLOAD) cata->mtrs.moveVelocity(100);
		else cata->stop();

		pros::Task::delay_until(&time, 10);
	}
}

Catapult::Catapult(okapi::MotorGroup& mtrs, Rotation& enc) : mtrs(mtrs), enc(enc) {
	mtrs.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
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
void Catapult::matchload() {
	cataState = CataState::MATCHLOAD;
}
void Catapult::stop() {
	mtrs.moveVoltage(0);
	cataState = CataState::SETTLED;
}

void Catapult::moveVelocity(double vel) {
	mtrs.moveVelocity(vel);
}

bool Catapult::isSettled() {
	return cataState == CataState::SETTLED;
}

void Catapult::waitForSettled() {
	do pros::delay(10);
	while (!isSettled());
}