#include "routes.hpp"
#include "robotconfig.hpp"
#include "16868C/util/util.hpp"

using namespace lib16868C;

void waitUntilButton(okapi::ControllerDigital btn = okapi::ControllerDigital::A) {
	while (!master.getDigital(btn)) pros::delay(10);
}

void nearAWPBar() {
	intake.moveVoltage(12000);
	chassis.moveDistance(7_in, 600_rpm, {0.12, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 800);
	// chassis.turnAbsolute(3_deg, 200_rpm, {0.2, 0, 3}, 2, 3, 5, TurnWheel::BOTH, 500);
	vertWings.extend();
	pros::delay(1000);
	// chassis.moveDistance(-10_in, 300_rpm, {0.3, 0, 10}, 1200, 3_deg, 300_rpm, {0.1, 0, 0.1}, 1200);
	chassis.turnAbsolute(-45_deg, 300_rpm, {0.12, 0, 2}, 2, 3, 5, TurnWheel::LEFT, 1000);
	vertWings.retract();
	pros::delay(1000);
	chassis.turnAbsolute(0_deg, 300_rpm, {0.12, 0, 2}, 2, 3, 5, TurnWheel::LEFT, 1000);
	// chassis.moveDistance(3_in, 600_rpm, {0.3, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 1200);
	chassis.moveDistance(-6_in, 600_rpm, {0.3, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 1200);
	// chassis.turnAbsolute(-20_deg, 600_rpm, {0.035, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 900);
	chassis.turnAbsolute(-40_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::LEFT, 1000);
	chassis.moveDistance(-35_in, 600_rpm, {0.12, 0, 10}, 1200, -40_deg, 300_rpm, {0.1, 0, 0.1}, 1300);
	// chassis.moveDistance(-18_in, 600_rpm, {0.12, 0, 10}, 1200, -45_deg, 300_rpm, {0.1, 0, 0.1}, 1300);
	chassis.moveDistance(10_in, 600_rpm, {0.12, 0, 10}, 1200, -40_deg, 300_rpm, {0.1, 0, 0.1}, 900);
	chassis.turnAbsolute(135_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1200);
	intake.moveVoltage(-12000);
	chassis.moveDistance(5.5_in, 200_rpm, {0.6, 0, 10}, 1200, 135_deg, 300_rpm, {0.1, 0, 0.1}, 800);
}

void nearRush() {
	intake.moveVoltage(12000);
	chassis.moveDistance(42_in, 600_rpm, {0.15, 0, 10}, 1200, 15_deg, 150_rpm, {0.2, 0, 10}, 1500);
	chassis.turnAbsolute(90_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	rightWing.extend();
	intake.moveVoltage(-12000);
	chassis.moveDistance(23_in, 600_rpm, {0.15, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.moveDistance(-20_in, 600_rpm, {0.15, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	rightWing.retract();
	// chassis.moveDistance(-10_in, 600_rpm, {0.15, 0, 10}, 1200, 90_deg, 300_rpm, {0.01, 0, 0.1}, 1000);
	// vertWings.extend();

	chassis.turnAbsolute(30_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-43_in, 600_rpm, {0.15, 0, 10}, 1200, 30_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.turnAbsolute(-40_deg, 400_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::LEFT, 1000);
	chassis.moveDistance(7_in, 600_rpm, {0.15, 0, 10}, 1200, -40_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	vertWings.extend();
	pros::delay(1000);
	chassis.moveDistance(-8_in, 300_rpm, {0.15, 0, 10}, 1200, -40_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	vertWings.retract();
	chassis.moveDistance(3_in, 600_rpm, {0.15, 0, 10}, 1200, -40_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	pros::delay(500);
	chassis.moveDistance(-4_in, 600_rpm, {0.2, 0, 10}, 1200, -40_deg, 300_rpm, {0.1, 0, 0.1}, 1000);

	chassis.turnAbsolute(-90_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::LEFT, 1000);
	chassis.moveDistance(-27_in, 600_rpm, {0.12, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 1300);
	chassis.moveDistance(15_in, 600_rpm, {0.12, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 900);
	chassis.turnAbsolute(90_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1200);
	intake.moveVoltage(-12000);
	chassis.moveDistance(16_in, 200_rpm, {0.6, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 800);
}

void farAWPBar() {
	inertial.set_rotation(-140);
	intake.moveVoltage(12000);
	// pros::Task extractMatchload([&]() {
	// 	vertWings.extend();
	// 	pros::delay(400);
	// 	vertWings.retract();
	// });
	vertWings.extend();
	chassis.moveDistance(-10_in, 600_rpm, {0.13, 0, 10}, 1200, -140_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	intake.moveVoltage(0);
	vertWings.retract();
	chassis.turnAbsolute(-180_deg, 600_rpm, {0.035, 0, 2}, 2, 3, 5, TurnWheel::LEFT, 0);
	chassis.moveDistance(-15_in, 600_rpm, {0.12, 0, 10}, 1200, -180_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	
	chassis.moveDistance(15_in, 600_rpm, {0.14, 0, 10}, 1200, -180_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.turnAbsolute(-140_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(18_in, 600_rpm, {0.13, 0, 10}, 1200, -140_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	intake.moveVoltage(12000);
	chassis.turnAbsolute(-100_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(25_in, 600_rpm, {0.15, 0, 10}, 1200, -100_deg, 300_rpm, {0.1, 0, 0.1}, 2000);
	chassis.moveDistance(5.5_in, 200_rpm, {0.6, 0, 10}, 1200, -100_deg, 300_rpm, {0.1, 0, 0.1}, 800);
}

void farAWP() {
	inertial.set_rotation(-90);
	intake.moveVoltage(12000);
	pros::delay(200);

	chassis.moveDistance(-31_in, 600_rpm, {0.15, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.turnAbsolute(-140_deg, 400_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::LEFT, 1000);
	
	vertWings.extend();
	chassis.moveDistance(-10_in, 600_rpm, {0.13, 0, 10}, 1200, -140_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	intake.moveVoltage(0);
	vertWings.retract();
	chassis.turnAbsolute(-180_deg, 600_rpm, {0.035, 0, 2}, 2, 3, 5, TurnWheel::LEFT, 0);
	chassis.turnAbsolute(-160_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::LEFT, 0);
	// chassis.moveDistance(5_in, 600_rpm, {0.13, 0, 10}, 1200, -160_deg, 300_rpm, {0.1, 0, 10}, 1000);
	// pros::delay(200);
	chassis.moveDistance(-20_in, 600_rpm, {0.2, 0, 10}, 1200, -180_deg, 150_rpm, {0.1, 0, 10}, 1500);

	chassis.moveDistance(11_in, 600_rpm, {0.2, 0, 10}, 1200, -180_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.turnAbsolute(0_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(15_in, 600_rpm, {0.2, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.moveDistance(-11_in, 600_rpm, {0.2, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(-70_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	intake.moveVoltage(12000);
	chassis.moveDistance(46_in, 600_rpm, {0.14, 0, 10}, 1200, -70_deg, 300_rpm, {0.3, 0, 10}, 3000);
	chassis.turnAbsolute(65_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	// pros::delay(500);
	intake.moveVoltage(-11000);
	pros::delay(500);

	chassis.turnAbsolute(-20_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	intake.moveVoltage(12000);
	chassis.moveDistance(20_in, 600_rpm, {0.13, 0, 10}, 1200, -20_deg, 300_rpm, {0.1, 0, 10}, 1000);
	chassis.turnAbsolute(90_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	leftWing.extend();
	rightWing.extend();
	intake.moveVoltage(-12000);
	chassis.moveDistance(50_in, 600_rpm, {0.15, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 10}, 2000);
	leftWing.retract();
	rightWing.retract();
	chassis.moveDistance(-10_in, 600_rpm, {0.13, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 10}, 1000);
}

void farRush() {
	intake.moveVoltage(12000);
	chassis.moveDistance(42_in, 600_rpm, {0.15, 0, 10}, 1200, -15_deg, 150_rpm, {0.2, 0, 10}, 1500);
	
	chassis.turnAbsolute(90_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
}

void skillsStart() {
	chassis.moveDistance(-30_in, 600_rpm, {0.15, 0, 10}, 1200, -35_deg, 300_rpm, {0.08, 0, 10}, 1100);
	chassis.moveDistance(12_in, 600_rpm, {0.15, 0, 10}, 1200, -35_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(63_deg, 600_rpm, {0.05, 0, 3.3}, 2, 1, 8, TurnWheel::BOTH, 2000);
	chassis.moveDistance(-4_in, 600_rpm, {0.15, 0, 10}, 1200, 63_deg, 300_rpm, {0.2, 0, 10}, 1000);

	vertWings.extend();
	kicker.move(100);
	// kicker.fireCount(100, 44);
}
void skills() {
	pros::Task releaseIntake([&]() {
		intake.moveVoltage(12000);
		pros::delay(1000);
		intake.moveVoltage(0);
	});
	skillsStart();

	pros::delay(30000);
	kicker.stop();
	vertWings.retract();

	chassis.moveDistance(40_in, 600_rpm, {0.15, 0, 10}, 1200, 100_deg, 150_rpm, {0.05, 0, 10}, 1500);
	chassis.turnAbsolute(50_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	leftWing.extend();
	rightWing.extend();
	intake.moveVoltage(-12000);
	chassis.moveDistance(18_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 10}, 1000);
	leftWing.retract();
	rightWing.retract();

	chassis.moveDistance(-16_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 10}, 1000);
	chassis.turnAbsolute(140_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	intake.moveVoltage(12000);
	chassis.moveDistance(34_in, 600_rpm, {0.15, 0, 10}, 1200, 140_deg, 300_rpm, {0.1, 0, 10}, 2000);
	chassis.turnAbsolute(50_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	leftWing.extend();
	rightWing.extend();
	intake.moveVoltage(-12000);
	chassis.moveDistance(20_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 10}, 1000);
	leftWing.retract();
	rightWing.retract();
	chassis.moveDistance(-35_in, 600_rpm, {0.15, 0, 10}, 1200, 110_deg, 400_rpm, {0.1, 0, 10}, 1000);
	chassis.turnAbsolute(50_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 2000);

	chassis.moveTank(12000, 12000);
	int a = 0;
	while (a < 3) {
		if (inertial.get_roll() < -5 && a == 0) { a++; std::cout << "Upward: " << inertial.get_roll() << "\n";  }
		else if (inertial.get_roll() > 2 && a == 1) { a++; std::cout << "Downard: " << inertial.get_roll() << "\n"; }
		else if (std::abs(inertial.get_roll()) < 20 && a == 2) { a++; std::cout << "Flat: " << inertial.get_roll() << "\n"; }

		pros::delay(100);
	}
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	chassis.moveTank(0, 0);
	pros::delay(500);
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

	chassis.moveDistance(30_in, 400_rpm, {0.3, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.moveDistance(-30_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	// chassis.moveDistance(-10_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 10}, 1000);
	// chassis.moveDistance(3_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 10}, 500);
	chassis.turnAbsolute(-40_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-32_in, 600_rpm, {0.15, 0, 10}, 1200, -40_deg, 300_rpm, {0.1, 0, 10}, 1000);
	// chassis.turnAbsolute(10_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, )
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(50_in, 600_rpm, {0.15, 0, 10}, 800, 50_deg, 380_rpm, {0.4, 0, 10}, 2100);
	leftWing.retract();
	rightWing.retract();
	pros::delay(300);
	chassis.moveDistance(-30_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(140_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-40_in, 600_rpm, {0.16, 0, 10}, 1200, 140_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(50_in, 600_rpm, {0.15, 0, 10}, 800, 50_deg, 350_rpm, {0.4, 0, 10}, 2100);
	leftWing.retract();
	rightWing.retract();
	pros::delay(300);

	chassis.moveDistance(-30_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 200_rpm, {0.4, 0, 10}, 1500);
	chassis.turnAbsolute(140_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(20_in, 600_rpm, {0.15, 0, 10}, 1200, 140_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(30_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(40_in, 600_rpm, {0.15, 0, 10}, 1200, 30_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.moveDistance(-10_in, 600_rpm, {0.15, 0, 10}, 1200, 30_deg, 300_rpm, {0.1, 0, 0.1}, 1000);

	// chassis.moveDistance(-28_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	// chassis.turnAbsolute(5_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	// leftWing.extend();
	// rightWing.extend();
	// chassis.moveDistance(35_in, 600_rpm, {0.15, 0, 10}, 800, 5_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	// leftWing.retract();
	// rightWing.retract();
	// pros::delay(300);
	// chassis.moveDistance(-5_in, 600_rpm, {0.15, 0, 10}, 1200, 5_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	// chassis.turnAbsolute(-40_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	// chassis.moveDistance(24_in, 600_rpm, {0.15, 0, 10}, 1200, -40_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	// chassis.turnAbsolute(-100_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	// vertWings.extend();
	// chassis.moveDistance(-40_in, 600_rpm, {0.15, 0, 10}, 1200, -40_deg, 150_rpm, {0.05, 0, 10}, 1500);
	// chassis.moveDistance(10_in, 600_rpm, {0.15, 0, 10},  1200, -40_deg, 150_rpm, {0.05, 0, 10}, 1000);
	// chassis.moveDistance(-20_in, 600_rpm, {0.15, 0, 10},  1200, -50_deg, 150_rpm, {0.05, 0, 10}, 1000);
}

void skills2() {
	pros::Task releaseIntake([&]() {
		intake.moveVoltage(12000);
		pros::delay(1000);
		intake.moveVoltage(0);
	});
	skillsStart();

	pros::delay(1000);
	kicker.stop();
	vertWings.retract();

	chassis.moveDistance(40_in, 600_rpm, {0.15, 0, 10}, 1200, 100_deg, 150_rpm, {0.05, 0, 10}, 1500);
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(45_in, 600_rpm, {0.16, 0, 10}, 1200, 140_deg, 400_rpm, {0.2, 0, 10}, 2500);
	chassis.turnAbsolute(50_deg, 400_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::RIGHT, 1000);
	chassis.moveDistance(-15_in, 600_rpm, {0.14, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	intake.moveVoltage(-12000);
	chassis.moveDistance(20_in, 600_rpm, {0.14, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.moveDistance(-40_in, 600_rpm, {0.14, 0, 10}, 1200, 100_deg, 400_rpm, {1, 0, 10}, 1000);
	leftWing.retract();
	rightWing.retract();
	chassis.turnAbsolute(50_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);

	/*
	chassis.moveDistance(40_in, 600_rpm, {0.15, 0, 10}, 1200, 100_deg, 150_rpm, {0.05, 0, 10}, 1500);
	chassis.turnAbsolute(50_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	leftWing.extend();
	rightWing.extend();
	intake.moveVoltage(-12000);
	chassis.moveDistance(18_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 10}, 1000);
	leftWing.retract();
	rightWing.retract();

	chassis.moveDistance(-16_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 10}, 1000);
	chassis.turnAbsolute(140_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	intake.moveVoltage(12000);
	chassis.moveDistance(36_in, 600_rpm, {0.15, 0, 10}, 1200, 140_deg, 300_rpm, {0.1, 0, 10}, 2000);
	chassis.turnAbsolute(50_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	leftWing.extend();
	rightWing.extend();
	intake.moveVoltage(-12000);
	chassis.moveDistance(20_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 10}, 1000);
	leftWing.retract();
	rightWing.retract();
	chassis.moveDistance(-10_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 400_rpm, {0.1, 0, 10}, 1000);
*/
	chassis.moveTank(12000, 12000);
	int a = 0;
	while (a < 3) {
		if (inertial.get_roll() < -5 && a == 0) { a++; std::cout << "Upward: " << inertial.get_roll() << "\n";  }
		else if (inertial.get_roll() > 2 && a == 1) { a++; std::cout << "Downard: " << inertial.get_roll() << "\n"; }
		else if (std::abs(inertial.get_roll()) < 15 && a == 2) { a++; std::cout << "Flat: " << inertial.get_roll() << "\n"; }

		pros::delay(100);
	}
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	chassis.moveTank(0, 0);
	pros::delay(500);
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

	chassis.moveDistance(30_in, 400_rpm, {0.3, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.moveDistance(-30_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	// chassis.moveDistance(-10_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 10}, 1000);
	// chassis.moveDistance(3_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 10}, 500);
	chassis.turnAbsolute(-40_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-32_in, 600_rpm, {0.15, 0, 10}, 1200, -40_deg, 300_rpm, {0.1, 0, 10}, 1000);
	// chassis.turnAbsolute(10_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, )
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(50_in, 600_rpm, {0.15, 0, 10}, 800, 50_deg, 380_rpm, {0.4, 0, 10}, 2100);
	leftWing.retract();
	rightWing.retract();
	pros::delay(300);
	chassis.moveDistance(-30_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(140_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-40_in, 600_rpm, {0.16, 0, 10}, 1200, 140_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(50_in, 600_rpm, {0.15, 0, 10}, 800, 50_deg, 350_rpm, {0.4, 0, 10}, 2100);
	leftWing.retract();
	rightWing.retract();
	pros::delay(300);

	chassis.moveDistance(-30_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 200_rpm, {0.4, 0, 10}, 1500);
	chassis.turnAbsolute(140_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(20_in, 600_rpm, {0.15, 0, 10}, 1200, 140_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(30_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(40_in, 600_rpm, {0.15, 0, 10}, 1200, 30_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.moveDistance(-10_in, 600_rpm, {0.15, 0, 10}, 1200, 30_deg, 300_rpm, {0.1, 0, 0.1}, 1000);

	/*
	chassis.moveDistance(-10_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 10}, 1000);
	chassis.moveDistance(3_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 10}, 500);
	leftWing.extend();
	chassis.turnAbsolute(-40_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	// chassis.moveDistance(-15_in, 600_rpm, {0.15, 0, 10}, 1200, -40_deg, 300_rpm, {0.1, 0, 10}, 1000);
	rightWing.extend();
	chassis.moveDistance(50_in, 600_rpm, {0.15, 0, 10}, 800, 50_deg, 380_rpm, {0.4, 0, 10}, 2100);
	leftWing.retract();
	rightWing.retract();
	pros::delay(300);
	chassis.moveDistance(-30_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	
	chassis.turnAbsolute(-40_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	// chassis.moveDistance(-35_in, 600_rpm, {0.15, 0, 10}, 1200, 140_deg, 300_rpm, {0.1, 0, 0.1}, 2000);
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(50_in, 600_rpm, {0.15, 0, 10}, 800, 50_deg, 380_rpm, {0.4, 0, 10}, 2100);
	leftWing.retract();
	rightWing.retract();
	pros::delay(300);
	chassis.moveDistance(-30_in, 600_rpm, {0.15, 0, 10}, 1200, 50_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	*/
}