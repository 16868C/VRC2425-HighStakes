#include "16868C/subsystems/intake.hpp"

using namespace lib16868C;

void Intake::intakeManager(void* param) {
	Intake* intake = static_cast<Intake*>(param);
	PIDController intakePID(intake->gains);

	intake->color.setLedPWM(100);
	
	IntakeState state = intake->getState();
	uint32_t time = pros::millis();
	int n = 0;
	bool ring = false;
	int tgtHook = -1;
	RingColour filteredRing = static_cast<RingColour>(-static_cast<int>(intake->getTargetRing()));
	intake->color.disableGestures();

	while (true) {
		double encPos = ReduceAngle::reduce(intake->enc.get(), intake->TPR, 0.0);
		int hookNum = intake->getCurrHook();
		if (hookNum != -1) intake->hookRings[hookNum] = intake->getColour();
		for (int i = 0; i < intake->hookRings.size(); i++) {
			int ind = i + 1;
			if (ind == intake->hookRings.size()) ind = 0;
			if (encPos > intake->HOOK_TICKS[ind] + 100 && encPos < intake->HOOK_TICKS[ind + 1] - 100) intake->hookRings[i] = RingColour::NONE;
		}

		filteredRing = static_cast<RingColour>(-static_cast<int>(intake->getTargetRing()));
		if (hookNum != -1 && intake->hookRings[hookNum] != RingColour::NONE && !ring) {
			ring = true;
			std::cout << "#" << hookNum << " " << encPos << " Ring Detected: " << (intake->hookRings[hookNum] == RingColour::NONE ? "None" : intake->hookRings[hookNum] == RingColour::BLUE ? "Blue" : "Red") << " { ";
			for (int i = 0; i < intake->hookRings.size(); i++) {
				std::cout << (intake->hookRings[i] == RingColour::NONE ? "None" : intake->hookRings[i] == RingColour::BLUE ? "Blue" : "Red") << ", ";
			}
			std::cout << "}\n";
		}
		if (intake->hookRings[hookNum] == RingColour::NONE) {
			ring = false;
		}
		
		if (intake->getState() != IntakeState::EJECTING && intake->getState() != IntakeState::UNJAMMING) {
			state = intake->getState();
		}

		if (intake->colourFilter && filteredRing != RingColour::NONE) {
			for (int i = 0; i < intake->hookRings.size(); i++) {
				bool inEjectPos = encPos > intake->EJECT_POS[i];
				if (i == intake->hookRings.size() - 1) inEjectPos = inEjectPos || encPos < 200;
				else inEjectPos = inEjectPos && encPos < intake->EJECT_POS[i] + 200;
				
				if (intake->hookRings[i] == filteredRing && inEjectPos) {
					std::cout << "eject " << i << " " << encPos << " " << intake->EJECT_POS[i] << "\n";
					intake->eject();
					pros::delay(200);
					intake->state = state;
					intake->update();
					intake->hookRings[i] = RingColour::NONE;
				}
			}
		}

		if (intake->state == IntakeState::HOLDING) {
			if (intake->hookRings[hookNum] == intake->targetRing) {
				double holdError = encPos - intake->HOLD_POS[tgtHook];
				intake->mtr.moveVoltage(-12000 * intakePID.calculate(holdError));
			} else {
				intake->mtr.moveVoltage(12000);
			} 
		}
		if (intake->isJamming()) n++;
		else n = 0;
		
		if (n >= 5) {
			// intake->unjam();
			// pros::delay(500);
			// intake->state = state;
			// intake->update();
			intake->stop();
		}

		pros::Task::delay_until(&time, 10);
	}
}

Intake::Intake(okapi::Motor& mtr, lib16868C::Rotation& enc, okapi::OpticalSensor& color, PIDGains gains, int numHook)
			: mtr(mtr), enc(enc), color(color), gains(gains), numHook(numHook) {}

void Intake::intake() {
	state = IntakeState::INTAKING;
	update();
}
void Intake::outtake() {
	state = IntakeState::OUTTAKING;
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

	state = IntakeState::OFF;
	update();
}

void Intake::update() {
	switch(state) {
	case IntakeState::INTAKING:
		mtr.moveVoltage(12000);
		break;
	case IntakeState::OUTTAKING:
		mtr.moveVoltage(-12000);
		break;
	case IntakeState::HOLDING:
		break;
	case IntakeState::EJECTING:
		mtr.moveVoltage(-12000);
		break;
	case IntakeState::UNJAMMING:
		mtr.moveVoltage(-12000);
		break;
	case IntakeState::OFF:
		mtr.moveVoltage(0);
		break;
	}
}

IntakeState Intake::getState() {
	return state;
}

void Intake::setTargetRing(RingColour colour) {
	targetRing = colour;
}
RingColour Intake::getTargetRing() {
	return targetRing;
}
std::array<RingColour, 2> Intake::getCurrRings() {
	return hookRings;
}

bool Intake::isJamming() {
	return mtr.getCurrentDraw() > 2300 && mtr.getActualVelocity() < 10;
}

RingColour Intake::getColour() {
	if (color.getProximity() < 150) return RingColour::NONE;

	if (color.getHue() > 170) return RingColour::BLUE;
	if (color.getHue() < 50) return RingColour::RED;
	return RingColour::NONE;
}
int Intake::getCurrHook() {
	double encPos = fmod(enc.get(), TPR);
	for (int i = 0; i < HOOK_TICKS.size(); i++) {
		if (encPos >= HOOK_TICKS[i] + 50 && encPos <= HOOK_TICKS[i] + 500) return i;
	}
	return -1;
}

void Intake::setColourFilter(bool state) {
	colourFilter = state;
}
void Intake::toggleColourFilter() {
	colourFilter = !colourFilter;
}