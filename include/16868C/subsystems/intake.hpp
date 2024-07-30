#pragma once
#include "okapi/impl/device/distanceSensor.hpp"
#include "okapi/impl/device/opticalSensor.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "pros/rtos.h"

namespace lib16868C {
enum class IntakeState {
	INTAKE_MOGO,
	INTAKE_BASKET,
	OUTTAKE,
	OFF
};
enum class TargetRing {
	RED,
	BLUE,
	NONE
};

class Intake {
	public:
		Intake(okapi::Motor& mtr, okapi::OpticalSensor& ringDetector, okapi::DistanceSensor& hookDetector);

		void intakeMogo();
		void intakeBasket();
		void outtake();
		void stop();

		void setTarget(TargetRing tgt);
		TargetRing getTarget();

		IntakeState getState();

		void setNumRings(int n);
		int getNumRings();

		bool isBasket();

	private:
		okapi::Motor& mtr;
		okapi::OpticalSensor& ringDetector;
		okapi::DistanceSensor& hookDetector;

		bool basket = false;

		int numRings = 0;

		IntakeState state = IntakeState::OFF;
		TargetRing tgt = TargetRing::NONE;

		pros::Task intakeTask = pros::Task(intakeManager, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake");
		static void intakeManager(void* param);
};

} // namespace lib16868C