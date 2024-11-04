#include "arm.hpp"

using namespace lib16868C;

void Arm::armManager(void* params) {
	Arm* arm = (Arm*) params;
	PIDController armPID(arm->pid);

	int n = 0;
	uint32_t time = pros::millis();
	while (true) {
		if (arm->pto.is_extended()) {
			pros::Task::delay_until(&time, 20);
			continue;
		}

		if (arm->mtrs.getCurrentDraw() > 2100 && abs(arm->tgt - arm->mtrs.getPosition()) < 50) n++;
		if (n > 5) {
			n = 0;
			arm->state = ArmPosition::IDLE;
		}

		if (arm->state != ArmPosition::IDLE) {
			arm->error = static_cast<int>(arm->getState()) - arm->enc.get();
			double ctrl = armPID.calculate(arm->error);
			// std::cout << ctrl << " " << static_cast<int>(arm->getState()) << " " << arm->enc.get() << "\n";
			arm->mtrs.moveVoltage(arm->volts * ctrl);
		} else {
			arm->error = 0;
		}

		pros::Task::delay_until(&time, 50);
	}
}

Arm::Arm(MotorGroup& mtrs, Rotation& enc, pros::adi::Pneumatics& pto, PIDGains gains) : mtrs(mtrs), enc(enc), pto(pto), pid(gains) {}

void Arm::move(double volts) {
	state = ArmPosition::IDLE;
	mtrs.moveVoltage(volts);
}
void Arm::moveTo(double tgt, double volts) {
	this->tgt = tgt;
	this->volts = volts;
	error = static_cast<int>(state) - enc.get();
}

void Arm::defaultPos(double volts) {
	this->volts = volts;
	state = ArmPosition::DEFAULT;
	tgt = static_cast<int>(state);
	error = static_cast<int>(state) - enc.get();
}
void Arm::descoreStake(double volts) {
	this->volts = volts;
	state = ArmPosition::DESECORE_STAKE;
	tgt = static_cast<int>(state);
	error = static_cast<int>(state) - enc.get();
}
void Arm::allianceStake(double volts) {
	this->volts = volts;
	state = ArmPosition::ALLIANCE_STAKE;
	tgt = static_cast<int>(state);
	error = static_cast<int>(state) - enc.get();
}
void Arm::wallStake(double volts) {
	this->volts = volts;
	state = ArmPosition::WALL_STAKE;
	tgt = static_cast<int>(state);
	error = static_cast<int>(state) - enc.get();
}

double Arm::getError() {
	return error;
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