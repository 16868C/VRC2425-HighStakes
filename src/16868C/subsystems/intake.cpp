#include "16868C/subsystems/intake.hpp"

using namespace lib16868C;

void Intake::intakeManager(void* param) {
	Intake* intake = static_cast<Intake*>(param);

	intake->ringDetector.setLedPWM(100);
	
	uint32_t time = pros::millis();
	int n = 0;
	while (true) {
		if (intake->mtr.getCurrentDraw() > 2300 && intake->mtr.getActualVelocity() < 30) n++;
		else if (intake->mtr.getCurrentDraw() < 2300 || intake->mtr.getActualVelocity() > 30) n = 0;
		if (n >= 5) {
			n = 0;
			intake->mtr.moveVoltage(-12000);
			pros::delay(300);
		}

		switch(intake->getState()) {
		case IntakeState::INTAKE_MOGO:
			switch(intake->getTarget()) {
			case TargetRing::BLUE:
				if (intake->ringDetector.getHue() > 160 && intake->ringDetector.getHue() < 250) {
					pros::delay(150);
					intake->mtr.moveVoltage(0);
					pros::delay(400);
					intake->mtr.moveVoltage(12000);
					pros::delay(100);
				} else {
					intake->mtr.moveVoltage(12000);
				}
				break;
			case TargetRing::RED:
				if (intake->ringDetector.getHue() < 30) {
					pros::delay(150);
					intake->mtr.moveVoltage(0);
					pros::delay(400);
					intake->mtr.moveVoltage(12000);
					pros::delay(100);
				} else {
					intake->mtr.moveVoltage(12000);
				}
				break;
			case TargetRing::NONE:
				intake->mtr.moveVoltage(12000);
				break;
			}
			break;
		case IntakeState::INTAKE_BASKET:
			if (intake->ringDetector.getProximity() > 200) {
				intake->basket = true;
				intake->mtr.moveVoltage(6000);
				pros::delay(270);
				intake->mtr.moveVoltage(-12000);
				pros::delay(1500);
				intake->basket = false;
				break;
			}

			switch(intake->getTarget()) {
			case TargetRing::BLUE:
				if (intake->ringDetector.getHue() > 160 && intake->ringDetector.getHue() < 250) {
					pros::delay(150);
					intake->mtr.moveVoltage(0);
					pros::delay(400);
					intake->mtr.moveVoltage(12000);
					pros::delay(100);
				} else {
					intake->mtr.moveVoltage(12000);
				}
				break;
			case TargetRing::RED:
				if (intake->ringDetector.getHue() < 30) {
					pros::delay(150);
					intake->mtr.moveVoltage(0);
					pros::delay(400);
					intake->mtr.moveVoltage(12000);
					pros::delay(100);
				} else {
					intake->mtr.moveVoltage(12000);
				}
				break;
			case TargetRing::NONE:
				intake->mtr.moveVoltage(12000);
				break;
			}
			break;
		case IntakeState::OUTTAKE:
			intake->mtr.moveVoltage(-12000);
			break;
		case IntakeState::OFF:
			if (intake->mtr.getActualVelocity() != 0) intake->mtr.moveVoltage(3000);
			while (intake->hookDetector.get() > 85) {
				if (intake->getState() != IntakeState::OFF) break;
				pros::delay(10);
			}
			intake->mtr.moveVoltage(0);
			break;
		}

		pros::Task::delay_until(&time, 50);
	}
}

Intake::Intake(okapi::Motor& mtr, okapi::OpticalSensor& ringDetector, okapi::DistanceSensor& hookDetector)
	: mtr(mtr), ringDetector(ringDetector), hookDetector(hookDetector) {}


void Intake::intakeMogo() {
	state = IntakeState::INTAKE_MOGO;
}
void Intake::intakeBasket() {
	state = IntakeState::INTAKE_BASKET;
}
void Intake::outtake() {
	state = IntakeState::OUTTAKE;
}
void Intake::stop() {
	state = IntakeState::OFF;
}

void Intake::setTarget(TargetRing tgt) {
	this->tgt = tgt;
}
TargetRing Intake::getTarget() {
	return tgt;
}

IntakeState Intake::getState() {
	return state;
}

void Intake::setNumRings(int n) {
	numRings = n;
}
int Intake::getNumRings() {
	return numRings;
}

bool Intake::isBasket() {
	return basket;
}