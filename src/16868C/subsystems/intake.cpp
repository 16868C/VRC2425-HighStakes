#include "16868C/subsystems/intake.hpp"
#include "16868C/controllers/pidController.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"

using namespace lib16868C;

void Intake::intakeManager(void* param) {
	Intake* intake = static_cast<Intake*>(param);
	PIDController intakePID({0.002, 0, 0.001});

	intake->color.setLedPWM(100);
	
	IntakeState state = intake->getState();
	uint32_t time = pros::millis();
	int n = 0;
	bool ring = false;
	int tgtHook = -1;
	intake->color.disableGestures();

	while (true) {
		if (!intake->pto.is_extended()) {
			pros::Task::delay_until(&time, 20);
			continue;
		}

		double encPos = ReduceAngle::reduce(intake->enc.get(), intake->TPR, 0.0);
		int hookNum = intake->getCurrHook() - 1;
		int prevHook = hookNum - 1 < 0 ? 3 : hookNum - 1;
		intake->hookRings[hookNum] = intake->getColour();
		for (int i = 0; i < 4; i++) {
			if (i != hookNum && i != prevHook) {
				intake->hookRings[i] = RingColour::NONE;
			}
		}

		if (intake->getColour() != RingColour::NONE && !ring) {
			ring = true;
			std::cout << "#" << hookNum << " " << encPos << " Ring Detected: " << (intake->getColour() == RingColour::BLUE ? "Blue" : "Red") << " { ";
			for (int i = 0; i < 4; i++) {
				std::cout << (intake->hookRings[i] == RingColour::NONE ? "None" : intake->hookRings[i] == RingColour::BLUE ? "Blue" : "Red") << ", ";
			}
			std::cout << "}\n";
		}
		if (intake->getColour() == RingColour::NONE) {
			ring = false;
		}
		
		if (intake->getState() != IntakeState::EJECTING && intake->getState() != IntakeState::UNJAMMING) {
			state = intake->getState();
		}

		if (intake->filteredRing != RingColour::NONE && intake->hookRings[prevHook] == intake->filteredRing && encPos - intake->EJECT_POS[prevHook] < 500 && encPos - intake->EJECT_POS[prevHook] > 0) {
			std::cout << "eject " << encPos << " " << intake->EJECT_POS[prevHook] << "\n";
			intake->secondStage.moveVoltage(-12000);
			pros::delay(200);
			intake->state = state;
			intake->update();
			intake->hookRings[prevHook] = RingColour::NONE;
		}

		if (intake->state == IntakeState::HOLDING) {
			intakePID.setGains({0.0038, 0, 0.001});
		} else {
			tgtHook = -1;
		}
		if (intake->state == IntakeState::REDIRECT) {
			intakePID.setGains({0.032, 0, 0});
		}

		// if (intake->state == IntakeState::INTAKE && intake->ring.get_value() < 2000) {
		// 	intake->hold();
		// }
		if (intake->state == IntakeState::INTAKE && intake->targetRing != RingColour::NONE && intake->hookRings[prevHook] == intake->targetRing) {
			intake->hold();
			tgtHook = prevHook;
		}

		double redirectError = encPos - intake->REDIRECT_POS[intake->getRedirectHook()];
		double holdError = encPos - (tgtHook == -1 ? 0 : intake->HOLD_POS[tgtHook]);
		// std::cout << encPos << " " << intake->getRedirectHook() << " " << intake->HOOK_TICKS[intake->getRedirectHook()] << "\n";
		if (intake->state == IntakeState::REDIRECT && abs(redirectError) > intake->ERROR_MARGIN) {
			intake->secondStage.moveVoltage(-4000 * intakePID.calculate(redirectError));
		} else if (intake->state == IntakeState::HOLDING && abs(holdError)) {
			intake->secondStage.moveVoltage(-12000 * intakePID.calculate(holdError));
		} else if (intake->state == IntakeState::REDIRECT || intake->state == IntakeState::HOLDING) {
			intake->secondStage.moveVoltage(0);
		}
		// std::cout << redirectError << "\n";

		if (intake->isJamming()) n++;
		else n = 0;
		
		// if (n >= 5) {
		// 	intake->unjam();
		// 	pros::delay(500);
		// 	intake->state = state;
		// 	intake->update();
		// }

		// std::cout << ejectHookPos << " " << intake->HOOK_TICKS[hookNum] << "\n";
		pros::Task::delay_until(&time, 20);
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
	if (std::abs(ReduceAngle::reduce(enc.get(), TPR, 0.0) - HOOK_TICKS[getRedirectHook()] + REDIRECT_POS[getRedirectHook()]) < 100) {
		pros::Task([&] {
			do pros::delay(50);
			while (std::abs(ReduceAngle::reduce(enc.get(), TPR, 0.0) - HOOK_TICKS[getRedirectHook()] + REDIRECT_POS[getRedirectHook()]) < 100);
			stop();
		});
		return;
	}

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
	targetRing = colour;
	switch (colour) {
		case RingColour::BLUE:
			filteredRing = RingColour::RED;
			break;
		case RingColour::RED:
			filteredRing = RingColour::BLUE;
			break;
		default:
			filteredRing = colour;
			break;
	}
}
RingColour Intake::getTargetRing() {
	return targetRing;
}
std::array<RingColour, 4> Intake::getCurrRings() {
	return hookRings;
}

bool Intake::isJamming() {
	return secondStage.getCurrentDraw() > 2300 && secondStage.getActualVelocity() < 10;
}

RingColour Intake::getColour() {
	if (color.getProximity() < 150) return RingColour::NONE;

	if (color.getHue() > 200) return RingColour::BLUE;
	if (color.getHue() < 80) return RingColour::RED;
	return RingColour::NONE;
}
int Intake::getCurrHook() {
	double encPos = fmod(enc.get(), TPR);
	for (int i = 1; i <= 4; i++) {
		if (encPos < HOOK_TICKS[i]) return i;
	}
	return 0;
}
int Intake::getRedirectHook() {
	double encPos = ReduceAngle::reduce(fmod(enc.get(), TPR), TPR, 0.0);
	for (int i = 0; i < 4; i++) {
		if (encPos < sma(HOOK_TICKS[i], HOOK_TICKS[i + 1])) return i;
	}
	return 4;
}