#include "16868C/subsystems/kicker.hpp"

using namespace lib16868C;

void Kicker::kickerControl(void* param) {
	Kicker* kicker = static_cast<Kicker*>(param);

	bool fireDebounce = false;
	uint32_t time = pros::millis();
	while (true) {
		if (kicker->mtr.getCurrentDraw() < 1000 && !fireDebounce) {
			kicker->cnt++;
			if (kicker->fireCnt > 0) kicker->fireCnt--;
			fireDebounce = true;
		} else if (kicker->mtr.getCurrentDraw() > 1000 && fireDebounce) fireDebounce = false;

		switch (kicker->cmd) {
			case KickerCmd::MOVE:
				break;
			case KickerCmd::FIRE:
				if (kicker->fireCnt == 0) kicker->tgtVel = 0;
				break;
			case KickerCmd::STOP:
				kicker->tgtVel = 0;
				break;
		}

		kicker->mtr.moveVelocity(kicker->tgtVel);

		pros::Task::delay_until(&time, 20);
	}
}

Kicker::Kicker(Motor& mtr) : mtr(mtr) {
	mtr.coast();
	pros::c::task_create(kickerControl, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Kicker Control");
}

void Kicker::move(double vel) {
	tgtVel = vel;
	cmd = KickerCmd::MOVE;
}
void Kicker::fireCount(double vel, int count) {
	tgtVel = vel;
	fireCnt = count;
	cmd = KickerCmd::FIRE;
}
void Kicker::stop() {
	tgtVel = 0;
	cmd = KickerCmd::STOP;
}

int Kicker::getCount() const {
	return cnt;
}

bool Kicker::isSettled() const {
	return tgtVel == 0 || cmd == KickerCmd::STOP;
}
void Kicker::waitUntilSettled() const {
	do pros::delay(50);
	while (!isSettled());
}

double Kicker::getTemperature() const {
	return mtr.getTemperature();
}