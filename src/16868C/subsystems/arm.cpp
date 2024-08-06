#include "arm.hpp"
#include "robotconfig.hpp"

using namespace lib16868C;

void Arm::armManager(void* params) {
	Arm* arm = (Arm*) params;

	int n = 0;
	uint32_t time = pros::millis();
	while (true) {
		if (arm->mtrs.getCurrentDraw() > 2100 && abs(arm->tgt - arm->mtrs.getPosition()) < 50) n++;
		if (n > 5) {
			n = 0;
			arm->state = ArmPosition::IDLE;
		}

		if (intake.isBasket())
			arm->mtrs.moveVoltage(-12000);
		else if (arm->state != ArmPosition::IDLE)
			arm->mtrs.moveAbsolute(arm->tgt, arm->volts);
		else
			arm->mtrs.moveVoltage(0);

		pros::Task::delay_until(&time, 50);
	}
}

Arm::Arm(MotorGroup& mtrs) : mtrs(mtrs) {}

void Arm::moveTo(double tgt, double volts) {
	this->tgt = tgt;
	this->volts = volts;
}

void Arm::defaultPos(double volts) {
	this->volts = volts;
	state = ArmPosition::DEFAULT;
	tgt = static_cast<int>(state);
}
void Arm::descoreStake(double volts) {
	this->volts = volts;
	state = ArmPosition::DESECORE_STAKE;
	tgt = static_cast<int>(state);
}
void Arm::allianceStake(double volts) {
	this->volts = volts;
	state = ArmPosition::ALLIANCE_STAKE;
	tgt = static_cast<int>(state);
}
void Arm::wallStake(double volts) {
	this->volts = volts;
	state = ArmPosition::WALL_STAKE;
	tgt = static_cast<int>(state);
}

void Arm::resetPosition() {
	armTask.suspend();

	int n = 0;
	while (n < 10) {
		mtrs.moveVoltage(-12000);
		if (mtrs.getCurrentDraw() > 2300) n++;
		pros::delay(50);
	}
	mtrs.resetZero();
	mtrs.moveVoltage(0);
	armTask.resume();
	
	defaultPos();
}

ArmPosition Arm::getState() {
	return state;
}