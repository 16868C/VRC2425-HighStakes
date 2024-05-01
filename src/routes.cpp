#include "routes.hpp"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"

using namespace lib16868C;

void waitUntilButton(okapi::ControllerDigital btn = okapi::ControllerDigital::A) {
	while (!master.getDigital(btn)) pros::delay(10);
}

void nearAWPBar() {
	intake.moveVoltage(12000); // Ensure preload is in the robot
	// Move in front of the matchload triball
	chassis.moveDistance(8_in, 600_rpm, {0.2, 0, 16}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 800);
	leftWing.extend();
	pros::delay(1000);
	// Knock out the matchload triball
	chassis.turnAbsolute(-45_deg, 300_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::LEFT, 1000);
	leftWing.retract();
	pros::delay(1000);
	// Turn back to let the matchload triball roll in front of the robot
	chassis.turnAbsolute(0_deg, 300_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::LEFT, 1000);
	// Push the matchload zone triball to the elevation bar
	chassis.moveDistance(-6_in, 600_rpm, {0.3, 0, 16}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 1200);
	chassis.turnAbsolute(-40_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::LEFT, 1000);
	chassis.moveDistance(-35_in, 600_rpm, {0.2, 0, 16}, 1200, -40_deg, 300_rpm, {0.1, 0, 0.1}, 1300);
	// Move away from the elevation bar
	chassis.moveDistance(10_in, 600_rpm, {0.2, 0, 16}, 1200, -40_deg, 300_rpm, {0.1, 0, 0.1}, 900);
	// Face the intake towards the elevation bar
	chassis.turnAbsolute(135_deg, 600_rpm, {0.055, 0, 5.6}, 2, 3, 5, TurnWheel::BOTH, 1200);
	intake.moveVoltage(-12000);
	// Touch the elevation bar slowly and accurately
	chassis.moveDistance(5.5_in, 200_rpm, {0.2, 0, 16}, 1200, 135_deg, 300_rpm, {0.1, 0, 0.1}, 800);
}

void nearRush() {
	// Intake one of the middle triball
	intake.moveVoltage(12000);
	chassis.moveDistance(42_in, 600_rpm, {0.15, 0, 10}, 1200, 15_deg, 150_rpm, {0.2, 0, 10}, 1500);
	// Push both middle triballs over the low barrier
	chassis.turnAbsolute(90_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	rightWing.extend();
	intake.moveVoltage(-12000);
	chassis.moveDistance(23_in, 600_rpm, {0.15, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.moveDistance(-20_in, 600_rpm, {0.15, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	rightWing.retract();

	// Move to the matchload bar
	chassis.turnAbsolute(30_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-43_in, 600_rpm, {0.15, 0, 10}, 1200, 30_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.turnAbsolute(-40_deg, 400_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::LEFT, 1000);
	chassis.moveDistance(7_in, 600_rpm, {0.15, 0, 10}, 1200, -40_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	// Knock out the matchload zone triball
	leftWing.extend();
	pros::delay(1000);
	chassis.moveDistance(-8_in, 300_rpm, {0.15, 0, 10}, 1200, -40_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	leftWing.retract();
	chassis.moveDistance(3_in, 600_rpm, {0.15, 0, 10}, 1200, -40_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	pros::delay(500);
	chassis.moveDistance(-4_in, 600_rpm, {0.2, 0, 10}, 1200, -40_deg, 300_rpm, {0.1, 0, 0.1}, 1000);

	// Push all the triballs under the elevation bar to our offensive zone
	chassis.turnAbsolute(-90_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::LEFT, 1000);
	chassis.moveDistance(-31_in, 600_rpm, {0.12, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 1300);
	chassis.moveDistance(5_in, 600_rpm, {0.12, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 900);
}

void nearDisrupt() {
	// Keeps track of the beginning time of the autonomous period
	uint st = pros::millis();

	// Rushes for the middle triball
	intake.moveVoltage(12000);
	chassis.moveDistance(42_in, 600_rpm, {0.15, 0, 10}, 1200, 15_deg, 150_rpm, {0.2, 0, 10}, 1500);
	chassis.turnAbsolute(90_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	// Pushes both middle triball over the barrier
	rightWing.extend();
	intake.moveVoltage(-12000);
	chassis.moveDistance(23_in, 600_rpm, {0.15, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	// Back up and wait in front of the goal
	chassis.moveDistance(-20_in, 600_rpm, {0.15, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	rightWing.retract();
	do pros::delay(100);
	while (pros::millis() - st < 14000);

	// Push any remaining triballs over the barrier
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(30_in, 600_rpm, {0.15, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 2000);
}

void nearBowlAWP() {
	inertial.set_rotation(130_deg);
	
	leftWing.extend();
	pros::delay(200);
	chassis.turnAbsolute(70_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::RIGHT, 1000);
	leftWing.retract();
	pros::delay(500);
	chassis.turnAbsolute(10_deg, 400_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::RIGHT, 1000);
	
	intake.moveVoltage(12000);
	chassis.moveDistance(48_in, 600_rpm, {0.2, 0, 16}, 1200, 19_deg, 300_rpm, {0.01, 0, 10}, 1500);
	chassis.moveDistance(-10_in, 600_rpm, {0.3, 0, 16}, 1200, 19_deg, 300_rpm, {0.01, 0, 10}, 1000);
	chassis.turnAbsolute(180_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::BOTH, 1000);
	intake.moveVoltage(-12000);
	chassis.moveDistance(5_in, 600_rpm, {0.3, 0, 16}, 1200, 180_deg, 300_rpm, {0.01, 0, 10}, 1000);
	pros::delay(500);

	chassis.turnAbsolute(48_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::BOTH, 1000);
	intake.moveVoltage(12000);
	chassis.moveDistance(26_in, 600_rpm, {0.2, 0, 16}, 1200, 48_deg, 300_rpm, {0.01, 0, 10}, 1000);
	chassis.moveDistance(-30_in, 600_rpm, {0.2, 0, 16}, 1200, 30_deg, 300_rpm, {0.1, 0, 10}, 1000);
	chassis.turnAbsolute(-135_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(60_in, 600_rpm, {0.2, 0, 16}, 1200, -270_deg, 200_rpm, {0.15, 0, 10}, 1500);
	intake.moveVoltage(-12000);
	chassis.moveDistance(6_in, 200_rpm, {0.2, 0, 16}, 1200, -270_deg, 300_rpm, {0.1, 0, 0.1}, 800);
}

void nearBowlRush() {
	leftWing.extend();
	pros::delay(200);
	leftWing.retract();
	
	intake.moveVoltage(12000);
	chassis.moveDistance(47.5_in, 600_rpm, {0.15, 0, 10}, 1200, 15.5_deg, 150_rpm, {0.22, 0, 16}, 1500);
	chassis.moveDistance(-10_in, 600_rpm, {0.3, 0, 16}, 1200, 15.5_deg, 300_rpm, {0.01, 0, 10}, 1000);
	chassis.turnAbsolute(180_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::BOTH, 1000);
	intake.moveVoltage(-12000);
	chassis.moveDistance(5_in, 600_rpm, {0.3, 0, 16}, 1200, 180_deg, 300_rpm, {0.01, 0, 10}, 1000);
	pros::delay(500);

	chassis.turnAbsolute(46_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::BOTH, 1000);
	intake.moveVoltage(12000);
	chassis.moveDistance(25.5_in, 400_rpm, {0.2, 0, 16}, 1000, 47_deg, 300_rpm, {0.01, 0, 10}, 1000);
	chassis.moveDistance(-35_in, 600_rpm, {0.2, 0, 16}, 1200, 30_deg, 300_rpm, {0.1, 0, 10}, 1000);
	chassis.turnAbsolute(-90_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(20_in, 600_rpm, {0.2, 0, 16}, 1200, -90_deg, 300_rpm, {0.01, 0, 10}, 1000);
	chassis.turnAbsolute(-180_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::RIGHT, 1000);
	chassis.moveDistance(-50_in, 600_rpm, {0.2, 0, 16}, 1200, -180_deg, 300_rpm, {0.1, 0, 10}, 1000);
	
	chassis.moveDistance(30_in, 600_rpm, {0.2, 0, 16}, 1200, -225_deg, 225_rpm, {0.1, 0, 10}, 1000);
	leftWing.extend();
	pros::delay(200);
	chassis.turnAbsolute(-300_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::BOTH, 1000);
	leftWing.retract();
	pros::delay(500);
	chassis.turnAbsolute(-250_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::RIGHT, 1000);
	intake.moveVoltage(-12000);
	chassis.moveDistance(38_in, 600_rpm, {0.2, 0, 16}, 1200, -270_deg, 100_rpm, {0.08, 0, 10}, 1500);
	chassis.moveDistance(-31_in, 600_rpm, {0.2, 0, 16}, 1200, -270_deg, 300_rpm, {0.1, 0, 10}, 1000);
}

void farAWPBar() {
	inertial.set_rotation(-140_deg);
	
	// Ensure preload is held securely in the intake
	intake.moveVoltage(12000);
	// Knock out the matchload zone triball
	leftWing.extend();
	chassis.moveDistance(-10_in, 600_rpm, {0.13, 0, 10}, 1200, -140_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	intake.moveVoltage(-12000);
	leftWing.retract();
	// Push the matchload zone and preload triball into the goal
	chassis.turnAbsolute(-180_deg, 600_rpm, {0.035, 0, 2}, 2, 3, 5, TurnWheel::LEFT, 0);
	chassis.moveDistance(-15_in, 600_rpm, {0.12, 0, 10}, 1200, -180_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	
	// Back up and line up with the elevation bar
	chassis.moveDistance(15_in, 600_rpm, {0.14, 0, 10}, 1200, -180_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.turnAbsolute(-140_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(18_in, 600_rpm, {0.13, 0, 10}, 1200, -140_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	intake.moveVoltage(12000);
	chassis.turnAbsolute(-100_deg, 600_rpm, {0.03, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 1000);
	// Drive into the elevation bar
	chassis.moveDistance(25_in, 600_rpm, {0.15, 0, 10}, 1200, -100_deg, 300_rpm, {0.1, 0, 0.1}, 2000);
	chassis.moveDistance(5.5_in, 200_rpm, {0.6, 0, 10}, 1200, -100_deg, 300_rpm, {0.1, 0, 0.1}, 800);
}

void far6Ball() {
	inertial.set_rotation(-90_deg);
	// Intake triball under the elevation bar
	intake.moveVoltage(8000);
	chassis.moveDistance(3_in, 600_rpm, {0.2, 0, 4}, 2400, -90_deg, 300_rpm, {0.035, 0, 0.6}, 400);

	chassis.moveDistance(-34_in, 600_rpm, {0.07, 0, 4}, 2400, -90_deg, 300_rpm, {0.035, 0, 0.6}, 1000);
	chassis.turnAbsolute(-135_deg, 600_rpm, {0.095, 0, 9}, 5, 3, 5, TurnWheel::LEFT, 600);
	leftWing.extend();
	chassis.moveDistance(-10_in, 600_rpm, {0.08, 0, 4}, 2400, -135_deg, 300_rpm, {0.035, 0, 0.6}, 600);
	chassis.turnAbsolute(-190_deg, 400_rpm, {0.1, 0, 9}, 5, 3, 5, TurnWheel::LEFT, 900);
	
	leftWing.retract();
	chassis.turnAbsolute(-150_deg, 600_rpm, {0.095, 0, 9}, 5, 3, 5, TurnWheel::LEFT, 500);
	chassis.moveDistance(-21_in, 600_rpm, {0.1, 0, 4}, 3000, -180_deg, 300_rpm, {0.02, 0, 0.6}, 1000);
	chassis.moveDistance(15_in, 600_rpm, {0.09, 0, 4}, 2400, -145_deg, 300_rpm, {0.035, 0, 0.6}, 800);
	chassis.turnAbsolute(0_deg, 600_rpm, {0.09, 0, 9}, 5, 3, 5, TurnWheel::BOTH, 800);

	intake.moveVoltage(-12000);
	pros::delay(600);
	chassis.moveDistance(13_in, 600_rpm, {1, 0, 4}, 3000, 0_deg, 300_rpm, {0.032, 0, 0.6}, 600);
	chassis.moveDistance(-12_in, 600_rpm, {0.085, 0, 4}, 2400, 0_deg, 300_rpm, {0.035, 0, 0.6}, 700);

	chassis.turnAbsolute(-69_deg, 400_rpm, {0.05, 0, 3}, 3, 3, 5, TurnWheel::BOTH, 700);
	intake.moveVoltage(12000);
	chassis.moveDistance(43_in, 600_rpm, {0.4, 0, 5}, 2400, -69_deg, 300_rpm, {0.035, 0, 0.6}, 1100);
	pros::delay(200);
	chassis.turnAbsolute(60_deg, 600_rpm, {0.09, 0, 9}, 5, 3, 5, TurnWheel::BOTH, 750);
	intake.moveVoltage(-12000);
	pros::delay(200);
	chassis.moveDistance(3_in, 600_rpm, {0.8, 0, 5}, 3000, 60_deg, 300_rpm, {0.035, 0, 0.6}, 350);
	pros::delay(300);
	
	chassis.turnAbsolute(-44_deg, 600_rpm, {0.095, 0, 9}, 5, 3, 5, TurnWheel::BOTH, 600);
	intake.moveVoltage(12000);
	chassis.moveDistance(16_in, 300_rpm, {0.5, 0, 5}, 800, -44_deg, 300_rpm, {0.035, 0, 0.6}, 850);
	
	chassis.turnAbsolute(90_deg, 400_rpm, {0.095, 0, 9}, 5, 3, 5, TurnWheel::BOTH, 650);
	intake.moveVoltage(-12000);
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(32_in, 600_rpm, {0.8, 0, 5}, 2400, 90_deg, 300_rpm, {0.035, 0, 0.6}, 900);
	pros::delay(300);
	leftWing.retract();
	rightWing.retract();
	// chassis.moveDistance(-22_in, 600_rpm, {0.8, 0, 4}, 2400, 90_deg, 300_rpm, {0.035, 0, 0.6}, 900);
	chassis.turnAbsolute(0_deg, 600_rpm, {0.09, 0, 9}, 5, 3, 5, TurnWheel::LEFT, 950);
}

void far5Ball() {
	inertial.set_rotation(40_deg);
	// Intake triball under the elevation bar
	// intake.moveVoltage(8000);
	// chassis.moveDistance(3_in, 600_rpm, {0.2, 0, 4}, 2400, -90_deg, 300_rpm, {0.035, 0, 0.6}, 400);

	// chassis.moveDistance(-34_in, 600_rpm, {0.07, 0, 4}, 2400, -90_deg, 300_rpm, {0.035, 0, 0.6}, 1000);
	// chassis.turnAbsolute(-135_deg, 600_rpm, {0.095, 0, 9}, 5, 3, 5, TurnWheel::LEFT, 600);
	intakeRaiser.retract();
	rightWing.extend();
	chassis.moveDistance(8_in, 600_rpm, {0.08, 0, 4}, 2400, 40_deg, 300_rpm, {0.035, 0, 0.6}, 600);
	chassis.turnAbsolute(10_deg, 600_rpm, {0.1, 0, 9}, 6, 3, 5, TurnWheel::RIGHT, 900);
	
	rightWing.retract();
	chassis.turnAbsolute(40_deg, 600_rpm, {0.095, 0, 9}, 5, 3, 5, TurnWheel::RIGHT, 500);
	intake.moveVoltage(-12000);
	chassis.moveDistance(24_in, 600_rpm, {0.1, 0, 4}, 3000, 0_deg, 300_rpm, {0.03, 0, 0.6}, 1000);
	chassis.moveDistance(-10_in, 600_rpm, {0.09, 0, 4}, 2400, 0_deg, 300_rpm, {0.035, 0, 0.6}, 0);
	// chassis.turnAbsolute(0_deg, 600_rpm, {0.09, 0, 9}, 5, 3, 5, TurnWheel::BOTH, 800);

	// intake.moveVoltage(-12000);
	// pros::delay(600);
	chassis.moveDistance(6_in, 600_rpm, {1, 0, 4}, 3000, 0_deg, 300_rpm, {0.032, 0, 0.6}, 600);
	chassis.moveDistance(-8_in, 600_rpm, {0.085, 0, 4}, 2400, 0_deg, 300_rpm, {0.035, 0, 0.6}, 0);

	chassis.turnAbsolute(-71_deg, 400_rpm, {0.05, 0, 3}, 3, 3, 5, TurnWheel::BOTH, 700);
	intakeRaiser.extend();
	intake.moveVoltage(12000);
	chassis.moveDistance(43_in, 400_rpm, {0.4, 0, 5}, 2000, -71_deg, 300_rpm, {0.035, 0, 0.6}, 1100);
	pros::delay(200);
	chassis.turnAbsolute(75_deg, 600_rpm, {0.09, 0, 9}, 5, 3, 5, TurnWheel::BOTH, 1000);
	pros::delay(200);
	intake.moveVoltage(-10000);
	pros::delay(100);
	chassis.moveDistance(3_in, 600_rpm, {0.8, 0, 5}, 3000, 60_deg, 300_rpm, {0.035, 0, 0.6}, 350);
	pros::delay(300);
	
	chassis.turnAbsolute(-44_deg, 600_rpm, {0.095, 0, 9}, 5, 3, 5, TurnWheel::BOTH, 600);
	intake.moveVoltage(12000);
	chassis.moveDistance(16_in, 300_rpm, {0.5, 0, 5}, 800, -44_deg, 300_rpm, {0.035, 0, 0.6}, 850);
	
	chassis.turnAbsolute(90_deg, 400_rpm, {0.095, 0, 9}, 5, 3, 5, TurnWheel::BOTH, 650);
	intake.moveVoltage(-12000);
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(32_in, 600_rpm, {0.8, 0, 5}, 2400, 90_deg, 300_rpm, {0.035, 0, 0.6}, 900);
	pros::delay(300);
	leftWing.retract();
	rightWing.retract();
	// chassis.moveDistance(-22_in, 600_rpm, {0.8, 0, 4}, 2400, 90_deg, 300_rpm, {0.035, 0, 0.6}, 900);
	chassis.turnAbsolute(0_deg, 600_rpm, {0.09, 0, 9}, 5, 3, 5, TurnWheel::LEFT, 950);
}

void farRushMid() {
	inertial.set_rotation(0_deg);

	intake.moveVoltage(12000);
	pros::Task flickPreload([&](){
		rightWing.extend();
		pros::delay(200);
		rightWing.retract();
	});

	chassis.moveDistance(45_in, 600_rpm, {0.8, 0, 6}, 3000, -9_deg, 300_rpm, {0.09, 0, 0.6}, 0);
	chassis.moveDistance(-29_in, 600_rpm, {0.8, 0, 5}, 2400, -30_deg, 300_rpm, {0.03, 0, 0.6}, 0);
	chassis.turnAbsolute(48_deg, 600_rpm, {0.095, 0, 9}, 5, 3, 5, TurnWheel::BOTH, 0);
	intake.moveVoltage(-12000);
	pros::delay(500);
	
	chassis.turnAbsolute(-66_deg, 400_rpm, {0.0434, 0, 4}, 2, 3, 5, TurnWheel::BOTH, 0);
	intake.moveVoltage(12000);
	chassis.moveDistance(31_in, 600_rpm, {0.5, 0, 7}, 2400, -66_deg, 300_rpm, {0.035, 0, 0.6}, 0);
	chassis.moveDistance(-32.5_in, 600_rpm, {0.8, 0, 5}, 2400, -40_deg, 300_rpm, {0.01, 0, 0.6}, 0);
	
	chassis.turnAbsolute(45_deg, 600_rpm, {0.095, 0, 9}, 5, 3, 5, TurnWheel::BOTH, 0);
	intake.moveVoltage(-12000);
	leftWing.extend();
	chassis.moveDistance(31_in, 600_rpm, {0.1, 0, 4}, 2400, 0_deg, 300_rpm, {0.012, 0, 0.6}, 1000);
	leftWing.retract();

	chassis.moveDistance(-25_in, 400_rpm, {0.5, 0, 6}, 1200, 45_deg, 300_rpm, {0.015, 0, 0.6}, 0);
	rightWing.extend();
	chassis.turnAbsolute(-20_deg, 250_rpm, {0.1, 0, 9}, 2, 3, 5, TurnWheel::RIGHT, 0);
	rightWing.retract();

	intake.moveVoltage(12000);
	chassis.turnAbsolute(-44.5_deg, 400_rpm, {0.05, 0, 3}, 2, 2, 5, TurnWheel::LEFT, 0);
	chassis.moveDistance(58_in, 600_rpm, {0.8, 0, 5}, 2400, -44.5_deg, 300_rpm, {0.035, 0, 0.6}, 0);

	chassis.moveDistance(-24_in, 600_rpm, {0.8, 0, 5}, 2400, -33_deg, 300_rpm, {0.035, 0, 0.6}, 0);
	chassis.turnAbsolute(-180_deg, 600_rpm, {0.095, 0, 9}, 5, 3, 5, TurnWheel::BOTH, 0);
	
	chassis.moveDistance(13_in, 600_rpm, {0.8, 0, 5}, 2400, -180_deg, 300_rpm, {0.035, 0, 0.6}, 0);
	chassis.turnAbsolute(-315_deg, 400_rpm, {0.1, 0, 9}, 4, 3, 5, TurnWheel::RIGHT, 0);
	intake.moveVoltage(-12000);
	leftWing.extend();
	chassis.moveDistance(40_in, 600_rpm, {0.8, 0, 5}, 2400, -360_deg, 300_rpm, {0.006, 0, 0.6}, 900);
	chassis.moveDistance(-10_in, 600_rpm, {0.8, 0, 5}, 2400, -360_deg, 300_rpm, {0.02, 0, 0.6}, 0);
	leftWing.retract();
	chassis.turnAbsolute(-405_deg, 600_rpm, {0.09, 0, 9}, 5, 3, 5, TurnWheel::LEFT, 0);
}

void closeAWPSafe() {
	inertial.set_rotation(-90_deg);

	// rightWing.extend();
	// pros::delay(200);
	// rightWing.retract();

	// chassis.moveDistance(8_in, 400_rpm, {0.7, 0, 4}, 1800, -90_deg, 300_rpm, {0.035, 0, 0.6}, 0);
	chassis.turnAbsolute(-45_deg, 600_rpm, {0.095, 0, 9}, 5, 3, 5, TurnWheel::LEFT, 600);
	leftWing.extend();
	chassis.turnAbsolute(-120_deg, 400_rpm, {0.1, 0, 9}, 2, 3, 5, TurnWheel::LEFT, 900);

	chassis.turnAbsolute(-60_deg, 300_rpm, {0.1, 0, 9}, 2, 3, 5, TurnWheel::LEFT, 900);
	leftWing.retract();
	chassis.moveDistance(-33_in, 600_rpm, {0.8, 0, 9}, 2400, -90_deg, 350_rpm, {0.045, 0, 0.6}, 0);
	intakeRaiser.retract();
	chassis.moveDistance(4_in, 300_rpm, {0.8, 0, 5}, 600, -90_deg, 300_rpm, {0.03, 0, 0.6}, 0);
	intake.moveVoltage(-12000);
	chassis.turnAbsolute(90_deg, 300_rpm, {0.15, 0, 9}, 2, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(9_in, 200_rpm, {0.6, 0, 10}, 600, 90_deg, 300_rpm, {0.035, 0, 0.6}, 0);
}

void closeRushMid() {
	inertial.set_rotation(0_deg);

	intake.moveVoltage(12000);

	chassis.moveDistance(41.5_in, 600_rpm, {0.8, 0, 6}, 3000, 8_deg, 300_rpm, {0.09, 0, 0.6}, 0);
	chassis.moveDistance(-20_in, 600_rpm, {0.8, 0, 5}, 2400, 30_deg, 300_rpm, {0.03, 0, 0.6}, 0);
	chassis.turnAbsolute(150_deg, 600_rpm, {0.095, 0, 9}, 5, 3, 5, TurnWheel::BOTH, 0);	
	intake.moveVoltage(-12000);
	pros::delay(500);

	chassis.turnAbsolute(41_deg, 600_rpm, {0.095, 0, 9}, 5, 3, 5, TurnWheel::BOTH, 0);
	intake.moveVoltage(12000);
	chassis.moveDistance(45_in, 200_rpm, {0.8, 0, 8}, 1300, 41_deg, 300_rpm, {0.03, 0, 0.6}, 0);
	chassis.moveDistance(-50_in, 400_rpm, {0.6, 0, 8}, 2400, 50_deg, 300_rpm, {0.03, 0, 0.6}, 0);
	
	chassis.turnAbsolute(135_deg, 600_rpm, {0.07, 0, 8}, 5, 3, 5, TurnWheel::BOTH, 0);
	rightWing.extend();
	chassis.moveDistance(3_in, 600_rpm, {0.8, 0, 6}, 2400, 135_deg, 300_rpm, {0.03, 0, 0.6}, 0);
	chassis.turnAbsolute(90_deg, 600_rpm, {0.06, 0, 9}, 6, 3, 5, TurnWheel::RIGHT, 0);
	rightWing.retract();
	chassis.turnAbsolute(135_deg, 600_rpm, {0.06, 0, 9}, 5, 3, 5, TurnWheel::RIGHT, 0);

	intake.moveVoltage(-12000);
	leftWing.extend();
	chassis.moveDistance(36_in, 600_rpm, {0.7, 0, 8}, 2400, 90_deg, 300_rpm, {0.025, 0, 0.6}, 0);
	leftWing.retract();
	chassis.moveDistance(-22_in, 200_rpm, {0.7, 0, 6}, 1000, 90_deg, 300_rpm, {0.025, 0, 0.6}, 0);
}
/*
static double heading;
void skillsStart() {
	inertial.set_rotation(-145_deg);

	// Push the two preloads into the goal
	chassis.moveDistance(-30_in, 600_rpm, {0.2, 0, 16}, 1500, -180_deg, 300_rpm, {0.08, 0, 10}, 1000);
	// Set up matchloading position
	chassis.moveDistance(12_in, 600_rpm, {0.3, 0, 16}, 1500, -160_deg, 300_rpm, {0.05, 0, 10}, 1000);
	chassis.turnAbsolute(-72_deg, 600_rpm, {0.055, 0, 6.1}, 2, 1.5, 6, TurnWheel::BOTH, 2000);
	// chassis.moveDistance(-2_in, 600_rpm, {0.4, 0, 16}, 1500, -75_deg, 300_rpm, {0.2, 0, 10}, 400);
	printDebug("%f\n", inertial.get_rotation(AngleUnit::DEG));
	heading = inertial.get_rotation(AngleUnit::DEG);

	vertWings.extend();
	kicker.move(200);
	// kicker.fireCount(100, 44);
}

void skills2() {
	pros::Task releaseIntake([&]() {
		intake.moveVoltage(12000);
		pros::delay(1000);
		intake.moveVoltage(0);
	});
	skillsStart();

	pros::delay(22000);
	kicker.stop();
	printDebug("%f\n", inertial.get_rotation(AngleUnit::DEG));
	inertial.set_rotation(heading * okapi::degree);
	vertWings.retract();
	kicker.holdAt(200, 200);

	chassis.moveDistance(31_in, 600_rpm, {0.21, 0, 16}, 1200, -145_deg, 600_rpm, {0.06, 0, 10}, 1000);
	rightWing.extend();
	chassis.moveDistance(64_in, 600_rpm, {0.2, 0, 16}, 1200, -90_deg, 600_rpm, {0.06, 0, 10}, 1600);
	intake.moveVoltage(-12000);
	kicker.stop();
	chassis.moveDistance(50_in, 400_rpm, {0.3, 0, 16}, 1500, 0_deg, 250_rpm, {0.01, 0, 10}, 1200);
	chassis.moveDistance(-10_in, 600_rpm, {0.21, 0, 16}, 1200, 0_deg, 300_rpm, {0.01, 0, 10}, 700);
	chassis.moveDistance(15_in, 600_rpm, {0.21, 0, 16}, 1200, 0_deg, 300_rpm, {0.01, 0, 10}, 1000);
	rightWing.retract();
	pros::delay(200);

	chassis.turnAbsolute(73_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::RIGHT, 1000);
	// rightWing.extend();
	intake.moveVoltage(-12000);
	pros::Task wings1([&]() {
		pros::delay(200);
		leftWing.extend();
	});
	chassis.moveDistance(35_in, 300_rpm, {0.22, 0, 16}, 800, 75_deg, 150_rpm, {0.01, 0, 10}, 1600);
	// rightWing.retract();
	// leftWing.retract();
	rightWing.extend();
	chassis.turnAbsolute(-80_deg, 300_rpm, {0.06, 0, 5.5}, 2, 3, 5, TurnWheel::RIGHT, 1500);
	// intake.moveVoltage(-12000);
	rightWing.retract();
	chassis.moveDistance(50_in, 600_rpm, {0.55, 0, 16}, 1500, -80_deg, 0_rpm, {0.01, 0, 10}, 1000);
	leftWing.retract();
	pros::delay(200);

	// chassis.moveDistance(-35_in, 600_rpm, {0.2, 0, 16}, 1200, 0_deg, 600_rpm, {0.1, 0, 10}, 1000);
	// chassis.moveDistance(30_in, 600_rpm, {0.2, 0, 16}, 1200, -90_deg, 600_rpm, {0.01, 0, 10}, 2000);
	chassis.moveDistance(-19_in, 600_rpm, {0.2, 0, 16}, 1200, -90_deg, 300_rpm, {0.1, 0, 10}, 1000);
	// vertWings.extend();
	chassis.turnAbsolute(-270_deg, 300_rpm, {0.2, 0, 16}, 2, 3, 5, TurnWheel::LEFT, 3000);
	// chassis.turnAbsolute(0_deg, 600_rpm, {0.05, 0, 4.4}, 2, 3, 5, TurnWheel::BOTH, 1000);
	// leftWing.extend();
	// vertWings.extend();
	intake.moveVoltage(12000);
	vertWings.extend();
	chassis.moveDistance(-50_in, 600_rpm, {0.4, 0, 16}, 1500, -270_deg, 600_rpm, {0.05, 0, 10}, 1200);
	vertWings.retract();
	// leftWing.retract();
	// pros::delay(200);

	chassis.moveDistance(23_in, 600_rpm, {0.2, 0, 16}, 1200, -270_deg, 600_rpm, {0.1, 0, 10}, 1000);
	chassis.turnAbsolute(-480_deg, 300_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::RIGHT, 2000);
	leftWing.extend();
	rightWing.extend();
	intake.moveVoltage(-12000);
	chassis.moveDistance(50_in, 600_rpm, {0.4, 0, 16}, 1500, -480_deg, 0_rpm, {0.05, 0, 10}, 1200);
	// chassis.moveDistance(-10_in, 600_rpm, {0.2, 0, 16}, 1200, -450_deg, 300_rpm, {0.01, 0, 10}, 1000);
	// chassis.moveDistance(40_in, 600_rpm, {0.5, 0, 16}, 1500, -450_deg, 0_rpm, {0.01, 0, 10}, 1000);
	leftWing.retract();
	rightWing.retract();
	// pros::delay(200);

	// chassis.moveDistance(-15_in, 600_rpm, {0.2, 0, 16}, 1200, -450_deg, 600_rpm, {0.05, 0, 10}, 1000);
	// chassis.moveDistance(30_in, 400_rpm, {0.2, 0, 16}, 1200, -480_deg, 0_rpm, {0.05, 0, 10}, 1000);
	chassis.moveDistance(-28_in, 600_rpm, {0.2, 0, 16}, 1200, -450_deg, 600_rpm, {0.05, 0, 10}, 1000);
	intake.moveVoltage(12000);
	chassis.turnAbsolute(-360_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::BOTH, 1000);
	leftWing.extend();
	rightWing.extend();
	intake.moveVoltage(-12000);
	// chassis.moveDistance(10_in, 600_rpm, {0.2, 0, 10}, 1200, -360_deg, 600_rpm, {0.05, 0, 10}, 1000);
	chassis.moveDistance(37_in, 400_rpm, {0.2, 0, 16}, 1200, -450_deg, 200_rpm, {0.1, 0, 10}, 1500);
	leftWing.retract();
	rightWing.retract();
	// chassis.moveDistance(-5_in, 600_rpm, {0.2, 0, 16}, 1200, -450_deg, 300_rpm, {0.1, 0, 10}, 1000);
	intake.moveVoltage(12000);
	chassis.turnAbsolute(-360_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::BOTH, 1000);	
	chassis.moveDistance(25_in, 600_rpm, {0.2, 0, 16}, 1200, -360_deg, 300_rpm, {0.1, 0, 10}, 1000);
	chassis.turnAbsolute(-450_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::BOTH, 1000);
	leftWing.extend();
	intake.moveVoltage(-12000);
	chassis.moveDistance(40_in, 400_rpm, {0.5, 0, 16}, 1500, -540_deg, 300_rpm, {0.01, 0, 10}, 1800);
	chassis.moveDistance(-20_in, 600_rpm, {0.2, 0, 16}, 1200, -495_deg, 600_rpm, {0.02, 0, 10}, 1500);
	leftWing.retract();
	chassis.turnAbsolute(-315_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-25_in, 600_rpm, {0.5, 0, 16}, 1200, -360_deg, 200_rpm, {0.01, 0, 10}, 1000);
	chassis.moveDistance(20_in, 600_rpm, {0.2, 0, 16}, 1200, -330_deg, 600_rpm, {0.02, 0, 10}, 1500);
	chassis.moveDistance(-25_in, 600_rpm, {0.5, 0, 16}, 1200, -360_deg, 350_rpm, {0.01, 0, 10}, 1000);
	chassis.moveDistance(20_in, 600_rpm, {0.2, 0, 16}, 1200, -330_deg, 600_rpm, {0.02, 0, 10}, 1500);
	chassis.moveDistance(-25_in, 600_rpm, {0.5, 0, 16}, 1200, -360_deg, 400_rpm, {0.01, 0, 10}, 1000);
	
	horiHang.extend();
	chassis.moveDistance(40_in, 600_rpm, {0.2, 0, 16}, 1200, -315_deg, 600_rpm, {0.02, 0, 10}, 1500);
	chassis.turnAbsolute(-270_deg, 600_rpm, {0.055, 0, 5.5}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(50_in, 600_rpm, {0.2, 0, 16}, 1200, -270_deg, 600_rpm, {0.01, 0, 10}, 3000);
	horiHang.retract();
}*/