#pragma once
#include "okapi/api.hpp"
#include "16868Z/devices/pneumatic.hpp"

namespace lib16868Z {
enum class IntakeState {
	INTAKE,
	OUTTAKE,
	SHOOT,
	MATCHLOAD,
	OFF
};

class Intake {
	public:
		Intake(okapi::Motor& front, okapi::Motor& rear, okapi::DistanceSensor& distSnsr, Pneumatic& mouth, double slewRate);

		void spin(double power);
		void spinFront(double power);
		void spinRear(double power);

		void stop();

		void intake(bool blocking = false);
		void outtake(bool openMouth = true, int delay = 0, bool blocking = false);
		void shoot();
		void matchload();

		bool inline hasBall() const {
			return distSnsr.get() < 150;
		}

		IntakeState inline getState() const {
			return state;
		}

	private:
		okapi::Motor& front, rear;
		okapi::DistanceSensor& distSnsr;
		Pneumatic& mouth;

		IntakeState state = IntakeState::OFF;

		double frontTarget, rearTarget;
		double slewRate;

		pros::task_t intakeTask;

		friend void intakeSlewRate(void* param);
};

void intakeSlewRate(void* param);
} // namespace lib16868Z