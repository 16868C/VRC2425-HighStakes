#pragma once
#include "16868C/devices/rotation.hpp"
#include "okapi/impl/device/opticalSensor.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.h"

namespace lib16868C {
enum class IntakeState {
	INTAKE,
	MOGO,
	REDIRECT,
	OUTTAKE,
	HOLDING,
	EJECTING,
	UNJAMMING,
	OFF
};
enum class RingColour {
	RED,
	BLUE,
	NONE
};

class Intake {
	public:
		Intake(okapi::Motor& firstStage, okapi::Motor& secondStage, lib16868C::Rotation& enc, okapi::OpticalSensor& color, pros::adi::LineSensor& ring, pros::adi::Pneumatics& pto);

		void intake();
		void mogo();
		void redirect();
		void hold();
		void outtake();
		void stop();

		IntakeState getState();

		void setTargetRing(RingColour colour);
		RingColour getTargetRing();
		std::array<RingColour, 4> getCurrRings();

		RingColour getColour();

		int getCurrHook();
		int getRedirectHook();

	private:
		void eject();
		void unjam();
		void update();

		bool isJamming();

		okapi::Motor& firstStage, secondStage;
		lib16868C::Rotation& enc;
		okapi::OpticalSensor& color;
		pros::adi::LineSensor& ring;
		pros::adi::Pneumatics& pto;

		const double TPR = 4199.2; // 10.74 * 360;
		const std::array<double, 5> HOOK_TICKS = {0, 1079.8, 2 * 1079.8, 2 * 1079.8 + 1019.8, 2 * (1079.8 + 1019.8)};
		const std::array<double, 4> EJECT_POS = {2050, 3070, 4070, 1010};
		const double REDIRECT_POS = 0;
		const double ERROR_MARGIN = 0;

		IntakeState state = IntakeState::OFF;

		RingColour filteredRing = RingColour::NONE;
		RingColour targetRing = RingColour::NONE;
		std::array<RingColour, 4> hookRings = {RingColour::NONE, RingColour::NONE, RingColour::NONE, RingColour::NONE};

		pros::Task managerTask = pros::Task(intakeManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Ring Filter");
		static void intakeManager(void* param);
};

} // namespace lib16868C