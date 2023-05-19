#include "16868X/devices/abstractMotor.hpp"
#include "16868X/util/util.hpp"

using namespace lib16868X;

void AbstractMotor::mtrManager(void* param) {
	uint32_t time = pros::millis();
	while (true) {
		for (auto& mtr : AbstractMotor::mtrs) {
			AbstractMotor::calcVelocity(mtr);
			AbstractMotor::velocityPID(mtr);
		}
		pros::Task::delay_until(&time, 10);
	}
}

double AbstractMotor::calcVelocity(std::shared_ptr<lib16868X::AbstractMotor> mtr) {
	uint32_t mtrClock;
	int mtrTicks = mtr->getRawPosition(&mtrClock);

	double dT = 5 * std::round((mtrClock - mtr->prevMtrClock) / 5.0);
	if (dT == 0) return mtr->vel;

	double dTicks = mtrTicks - mtr->prevMtrTicks;
	mtr->prevMtrClock = mtrClock; mtr->prevMtrTicks = mtrTicks;

	double calcVel = (dTicks / 50) / dT / 60000;
	if (calcVel > 5000) return mtr->vel;
	
	double vel = calcVel;
	vel = mtr->smaVel.filter(vel);
	vel = mtr->medianVel.filter(vel);

	double accel = (mtr->prevCalcVel - vel) / dT;
	mtr->prevCalcVel = vel;

	double maxAccel = mtr->maxAccel.filter(accel);
	accel = mtr->smaAccel.filter(accel);

	double emaA = 0.75 * (1 - (1 / ((maxAccel * maxAccel / 50) + 1.013)));
	vel = mtr->emaVel.filter(vel, emaA);
	mtr->vel = vel * mtr->getMaxRPM() / 3600;
	mtr->accel = (mtr->prevFilteredVel - vel) / dT;

	return mtr->vel;
}

void AbstractMotor::velocityPID(std::shared_ptr<lib16868X::AbstractMotor> mtr) {
	double error = mtr->targetVel - mtr->vel;
	double output = mtr->vel * mtr->kV + mtr->accel * mtr->kA + error * mtr->kP;
	mtr->moveVoltage(output);
}