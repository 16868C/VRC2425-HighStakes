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
		RingColour getCurrentRing();

		int getNumRings();

	private:
		void eject();
		void unjam();
		void update();

		bool isJamming();
		static RingColour getColour(double hue);

		okapi::Motor& firstStage, secondStage;
		lib16868C::Rotation& enc;
		okapi::OpticalSensor& color;
		pros::adi::LineSensor& ring;
		pros::adi::Pneumatics& pto;

		int numRings = 0;
		double tgtPos = -1;

		const double TPR = 4020.57; // 10.74 * 360;
		const double EJECT_POS = 953;
		const double REDIRECT_POS = 250;
		const double ERROR_MARGIN = 10;

		IntakeState state = IntakeState::OFF;

		RingColour tgtRing = RingColour::NONE;
		RingColour curRing = RingColour::NONE;

		pros::Task managerTask = pros::Task(intakeManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Ring Filter");
		static void intakeManager(void* param);
};

} // namespace lib16868C