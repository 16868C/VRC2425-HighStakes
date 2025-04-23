#include "arm.hpp"

using namespace lib16868C;

void Arm::armManager(void* params) {
	Arm* arm = (Arm*) params;
	PIDController armPID(arm->pid);

	int n = 0;
	uint32_t time = pros::millis();
	while (true) {
		if (arm->mtrs.getCurrentDraw() > 2100 && abs(arm->tgt - arm->mtrs.getPosition()) < 50) n++;
		else n = 0;
		if (n > 5) {
			n = 0;
			arm->state = ArmPosition::IDLE;
		}
		// pros::lcd::print(0, "%f", arm->enc.get());

		if (arm->state != ArmPosition::IDLE) {
			arm->error = static_cast<int>(arm->getState()) - arm->enc.get();
			double ctrl = armPID.calculate(arm->error);
			// std::cout << ctrl << " " << static_cast<int>(arm->getState()) << " " << arm->enc.get() << "\n";
			arm->mtrs.moveVoltage(arm->volts * ctrl);
		} else {
			arm->error = 0;
			arm->mtrs.moveVoltage(0);
		}

		pros::Task::delay_until(&time, 20);
	}
}

Arm::Arm(MotorGroup& mtrs, Rotation& enc, PIDGains gains) : mtrs(mtrs), enc(enc), pid(gains) {}

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
void Arm::load(double volts) {
	this->volts = volts;
	state = ArmPosition::LOAD;
	tgt = static_cast<int>(state);
	error = static_cast<int>(state) - enc.get();
}
void Arm::load2(double volts) {
	this->volts = volts;
	state = ArmPosition::LOAD2;
	tgt = static_cast<int>(state);
	error = static_cast<int>(state) - enc.get();
}
void Arm::hold(double volts) {
	this->volts = volts;
	state = ArmPosition::HOLD;
	tgt = static_cast<int>(state);
	error = static_cast<int>(state) - enc.get();
}
void Arm::wallStake(double volts) {
	this->volts = volts;
	state = ArmPosition::WALL_STAKE;
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