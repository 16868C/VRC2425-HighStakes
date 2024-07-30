#pragma once
#include "16868C/devices/motorGroup.hpp"
#include "pros/rtos.h"

namespace lib16868C {
enum class ArmPosition {
	IDLE = -1,
	DEFAULT = 0,
	DESECORE_STAKE = 400,
	ALLIANCE_STAKE = 800,
	WALL_STAKE = 1200
};

class Arm {
public:
	Arm(lib16868C::MotorGroup& mtrs);

	void defaultPos(double volts = 12000);
	void descoreStake(double volts = 12000);
	void allianceStake(double volts = 12000);
	void wallStake(double volts = 12000);

	void resetPosition();

	ArmPosition getState();

private:
	lib16868C::MotorGroup& mtrs;
	double volts = 12000;
	ArmPosition state = ArmPosition::IDLE;

	pros::Task armTask = pros::Task(armManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Arm Manager");
	static void armManager(void* params);
};
} // namespace lib16868C