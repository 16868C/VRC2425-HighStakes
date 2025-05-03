#pragma once
#include "16868C/controllers/pidController.hpp"
#include "16868C/devices/rotation.hpp"
#include "16868C/subsystems/arm.hpp"
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
		Intake(okapi::Motor& mtr, lib16868C::Rotation& enc, okapi::OpticalSensor& color, PIDGains gains, int numHook, ArmPosition* armState);

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

		void setColourFilter(bool state);
		void toggleColourFilter();

	private:
		void eject();
		void unjam();
		void update();

		bool isJamming();

		okapi::Motor& mtr;
		lib16868C::Rotation& enc;
		okapi::OpticalSensor& color;

		ArmPosition* armState;

		bool colourFilter = true;

		PIDGains gains;

		int numHook;

		const double TPR = 1140.18;
		const std::array<double, 3> HOOK_TICKS = {0, 570, 1140.18};
		const std::array<double, 2> EJECT_POS = {560, 1100};
		const std::array<double, 2> HOLD_POS = {100, 260};

		IntakeState state = IntakeState::OFF;

		RingColour targetRing = RingColour::NONE;
		std::array<RingColour, 2> hookRings = {RingColour::NONE, RingColour::NONE};

		pros::Task managerTask = pros::Task(intakeManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Ring Filter");
		static void intakeManager(void* param);
};
} // namespace lib16868C