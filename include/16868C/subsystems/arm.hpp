#pragma once
#include "16868C/devices/motorGroup.hpp"
#include "16868C/devices/rotation.hpp"
#include "16868C/controllers/pidController.hpp"
#include "pros/rtos.h"

namespace lib16868C {
enum class ArmPosition {
	IDLE = -1,
	DEFAULT = 0,
	DESECORE_STAKE = 20,
	ALLIANCE_STAKE = 35,
	WALL_STAKE = 53
};

class Arm {
public:
	Arm(lib16868C::MotorGroup& mtrs, lib16868C::Rotation& enc, pros::adi::Pneumatics& pto, PIDGains gains);

	void move(double volts);
	void moveTo(double tgt, double volts = 12000);

	void defaultPos(double volts = 12000);
	void descoreStake(double volts = 12000);
	void allianceStake(double volts = 12000);
	void wallStake(double volts = 12000);

	double getError();

	void resetPosition();

	ArmPosition getState();

private:
	lib16868C::MotorGroup& mtrs;
	lib16868C::Rotation& enc;
	pros::adi::Pneumatics& pto;

	double volts = 12000;
	double tgt = 0;
	double error = 1e5;
	ArmPosition state = ArmPosition::IDLE;
	PIDGains pid;

	pros::Task armTask = pros::Task(armManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Arm Manager");
	static void armManager(void* params);
};
} // namespace lib16868C