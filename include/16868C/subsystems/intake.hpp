#pragma once
#include "16868C/devices/rotation.hpp"
#include "okapi/impl/device/opticalSensor.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "pros/rtos.h"

namespace lib16868C {
enum class IntakeState {
	INTAKING,
	OUTTAKING,
	HOLDING,
	EJECTING,
	UNJAMMING,
	OFF
};
enum class RingColour {
	RED = 1,
	BLUE = -1,
	NONE = 0
};

class Intake {
	public:
		Intake(okapi::Motor& mtr, lib16868C::Rotation& enc, okapi::OpticalSensor& color);

		void intake();
		void outtake();
		void hold();
		void stop();

		IntakeState getState();

		void setTargetRing(RingColour colour);
		RingColour getTargetRing();
		std::array<RingColour, 2> getCurrRings();

		RingColour getColour();

		int getCurrHook();
		int getRedirectHook();

	private:
		void eject();
		void unjam();
		void update();

		bool isJamming();

		okapi::Motor& mtr;
		lib16868C::Rotation& enc;
		okapi::OpticalSensor& color;

		const double TPR = 1050.09;
		const std::array<double, 3> HOOK_TICKS = {0, 530, 1050.09};
		const std::array<double, 2> EJECT_POS = {500, 1030};
		const std::array<double, 2> HOLD_POS = {100, 260};

		IntakeState state = IntakeState::OFF;

		RingColour targetRing = RingColour::BLUE;
		std::array<RingColour, 2> hookRings = {RingColour::NONE, RingColour::NONE};

		pros::Task managerTask = pros::Task(intakeManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Ring Filter");
		static void intakeManager(void* param);
};
} // namespace lib16868C