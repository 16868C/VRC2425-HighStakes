#include "16868C/subsystems/intake.hpp"
#include "16868C/controllers/pidController.hpp"
#include "pros/adi.hpp"

using namespace lib16868C;

void Intake::intakeManager(void* param) {
	Intake* intake = static_cast<Intake*>(param);
	PIDController intakePID({0.025, 0, 0.1});

	intake->color.setLedPWM(100);
	
	IntakeState state = intake->getState();
	uint32_t time = pros::millis();
	int n = 0;
	while (true) {
		if (!intake->pto.is_extended()) {
			pros::Task::delay_until(&time, 20);
			continue;
		}
		
		if (intake->getState() != IntakeState::EJECTING && intake->getState() != IntakeState::UNJAMMING) {
			state = intake->getState();
		}

		intake->curRing = Intake::getColour(intake->color.getHue());
		double encPos = remainder(intake->enc.get(), intake->TPR / 4);

		if (intake->tgtRing != RingColour::NONE && intake->curRing == intake->tgtRing && (intake->state == IntakeState::MOGO || intake->state == IntakeState::INTAKE)) {
			intake->eject();
			// intake->tgtPos = intake->enc.get() + intake->EJECT_POS;
		}
		if (intake->state == IntakeState::EJECTING && intake->ring.get_value() < 2000) {
			intake->tgtPos = intake->EJECT_POS;
		} else if (intake->state != IntakeState::EJECTING) {
			intake->tgtPos = -1;
		}

		// std::cout << intake->enc.get() << " " << std::fmod(intake->enc.get(), intake->TPR / 4) << " " << intake->tgtPos << "\n";
		if (intake->state == IntakeState::EJECTING && intake->tgtPos != -1 && std::fmod(intake->enc.get(), intake->TPR / 4) > intake->tgtPos) {
			intake->secondStage.moveVoltage(-12000);
			pros::delay(500);
			intake->state = state;
			intake->update();
		}

		if (intake->state == IntakeState::INTAKE && intake->ring.get_value() < 2000) {
			intake->hold();
		}

		if (intake->state == IntakeState::REDIRECT && abs(encPos - intake->REDIRECT_POS) > intake->ERROR_MARGIN) {
			intake->secondStage.moveVoltage(-4000 * intakePID.calculate(encPos - intake->REDIRECT_POS));
		} else if (intake->state == IntakeState::REDIRECT) {
			intake->secondStage.moveVoltage(0);
		}

		if (intake->isJamming()) n++;
		else n = 0;
		
		// if (n >= 5) {
		// 	intake->unjam();
		// 	pros::delay(500);
		// 	intake->state = state;
		// 	intake->update();
		// }

		pros::Task::delay_until(&time, 10);
	}
}

Intake::Intake(okapi::Motor& firstStage, okapi::Motor& secondStage, lib16868C::Rotation& enc, okapi::OpticalSensor& color, pros::adi::LineSensor& ring, pros::adi::Pneumatics& pto)
	: firstStage(firstStage), secondStage(secondStage), enc(enc), color(color), ring(ring), pto(pto) {}


void Intake::intake() {
	state = IntakeState::INTAKE;
	update();
}
void Intake::mogo() {
	state = IntakeState::MOGO;
	update();
}
void Intake::redirect() {
	state = IntakeState::REDIRECT;
	update();
}
void Intake::outtake() {
	state = IntakeState::OUTTAKE;
	update();
}
void Intake::hold() {
	state = IntakeState::HOLDING;
	update();
}
void Intake::eject() {
	state = IntakeState::EJECTING;
	update();
}
void Intake::unjam() {
	state = IntakeState::UNJAMMING;
	update();
}
void Intake::stop() {
	if (state == IntakeState::EJECTING || state == IntakeState::UNJAMMING) {
		pros::Task([&] {
			do pros::delay(50);
			while (state == IntakeState::EJECTING || state == IntakeState::UNJAMMING);
			stop();
		});
		return;
	}

	tgtPos = 0;
	state = IntakeState::OFF;
	update();
}

void Intake::update() {
	switch(state) {
	case IntakeState::MOGO:
		firstStage.moveVoltage(12000);
		secondStage.moveVoltage(12000);
		break;
	case IntakeState::REDIRECT:
		firstStage.moveVoltage(12000);
		break;
	case IntakeState::INTAKE:
		firstStage.moveVoltage(12000);
		secondStage.moveVoltage(12000);
		break;
	case IntakeState::OUTTAKE:
		firstStage.moveVoltage(-12000);
		secondStage.moveVoltage(-12000);
		break;
	case IntakeState::HOLDING:
		firstStage.moveVoltage(12000);
		secondStage.moveVoltage(0);
		break;
	case IntakeState::EJECTING:
		break;
	case IntakeState::UNJAMMING:
		firstStage.moveVoltage(-12000);
		secondStage.moveVoltage(-12000);
		break;
	case IntakeState::OFF:
		firstStage.moveVoltage(0);
		secondStage.moveVoltage(0);
		break;
	}
}

IntakeState Intake::getState() {
	return state;
}

void Intake::setTargetRing(RingColour colour) {
	tgtRing = colour;
}
RingColour Intake::getTargetRing() {
	return tgtRing;
}
RingColour Intake::getCurrentRing() {
	return curRing;
}

int Intake::getNumRings() {
	return numRings;
}

bool Intake::isJamming() {
	return secondStage.getCurrentDraw() > 2300 && secondStage.getActualVelocity() < 10;
}

RingColour Intake::getColour(double hue) {
	if (hue > 160 && hue < 250) return RingColour::BLUE;
	if (hue < 30) return RingColour::RED;
	return RingColour::NONE;
}