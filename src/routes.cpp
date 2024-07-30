#include "routes.hpp"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"

using namespace lib16868C;

void waitUntilButton(okapi::ControllerDigital btn = okapi::ControllerDigital::A) {
	while (!master.getDigital(btn)) pros::delay(10);
}

void redSoloAWP() {
	odometry.update({124_in, 60_in, 25_deg});

	pros::Task relArm([&]() {
		intake.outtake();
		pros::delay(200);
		intake.stop();
		arm.allianceStake();
	});
	pros::delay(500);
	chassis.moveDistance(11_in, 600_rpm, {0.07, 0, 1.5}, 25_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	arm.defaultPos(8000);
	pros::delay(300);
	chassis.moveToPoint({118_in, 54_in}, 400_rpm, {0.055, 0, 0.1}, {1, 0, 0.05}, 1_in, true, true, 0);
	chassis.turnAbsolute(0_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(-15_in, 150_rpm, {1, 0, 1}, 0_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	// pros::delay(200);
	clamp.extend();
	tilter.retract();
	pros::delay(100);
	chassis.turnAbsolute(-90_deg, 600_rpm, {0.03, 0, 0.8}, 3, 5, TurnWheel::BOTH, 0);

	intake.intakeMogo();
	chassis.moveToPoint({106_in, 20_in}, 600_rpm, {0.045, 0, 0.1}, {1, 0, 0.05}, 1_in, false, true, 0);
	chassis.turnAbsolute(-160_deg, 600_rpm, {0.03, 0, 0.8}, 3, 5, TurnWheel::BOTH, 0);
	// waitUntilButton();
	pros::delay(400);
	chassis.moveToPoint({98_in, 19.5_in}, 600_rpm, {0.045, 0, 0.1}, {0.7, 0, 0.05}, 1_in, false, true, 0);
	pros::delay(800);
	chassis.moveDistance(-10_in, 600_rpm, {0.07, 0, 1.5}, -180_deg, 300_rpm, {0.05, 0, 0.1}, 0);

	chassis.turnAbsolute(-210_deg, 600_rpm, {0.04, 0, 0.7}, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(10_in, 600_rpm, {0.07, 0, 1.5}, -210_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	pros::delay(500);

	chassis.moveToPoint({105_in, 20_in}, 600_rpm, {0.045, 0, 0.1}, {1, 0, 0.05}, 1_in, true, true, 0);
	// pros::delay(800);
	// intake.stop();
	// pros::delay(500);
	arm.allianceStake();
	chassis.moveToPoint({102_in, 56_in}, 600_rpm, {0.045, 0, 0.1}, {1, 0, 0.05}, 5_in, false, true);
	chassis.turnAbsolute(-225_deg, 600_rpm, {0.03, 0, 0.8}, 3, 5, TurnWheel::BOTH, 0);
	arm.descoreStake();
}
void blueSoloAWP() {
	odometry.update({16_in, 60_in, 155_deg});

	pros::Task relArm([&]() {
		intake.outtake();
		pros::delay(200);
		intake.stop();
		arm.allianceStake();
	});
	pros::delay(500);
	chassis.moveDistance(11_in, 600_rpm, {0.07, 0, 1.5}, 155_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	arm.defaultPos(8000);
	pros::delay(300);
	chassis.moveToPoint({18_in, 46_in}, 400_rpm, {0.055, 0, 0.1}, {0.5, 0, 0.05}, 1_in, true, true, 0);
	// waitUntilButton();
	chassis.turnAbsolute(180_deg, 600_rpm, {0.025, 0, 0.8}, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(-15_in, 150_rpm, {1, 0, 1}, 180_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	// pros::delay(200);
	clamp.extend();
	tilter.retract();
	pros::delay(100);
	chassis.turnAbsolute(270_deg, 600_rpm, {0.03, 0, 0.8}, 3, 5, TurnWheel::BOTH, 0);

	intake.intakeMogo();
	chassis.moveToPoint({30_in, 15_in}, 600_rpm, {0.045, 0, 0.1}, {0.5, 0, 0.05}, 1_in, false, true, 0);
	chassis.turnAbsolute(340_deg, 600_rpm, {0.03, 0, 0.8}, 3, 5, TurnWheel::BOTH, 0);
	// waitUntilButton();
	pros::delay(400);
	chassis.moveToPoint({42_in, 14_in}, 600_rpm, {0.045, 0, 0.1}, {0.7, 0, 0.05}, 1_in, false, true, 0);
	pros::delay(800);
	chassis.moveDistance(-10_in, 600_rpm, {0.07, 0, 1.5}, 360_deg, 300_rpm, {0.05, 0, 0.1}, 0);

	chassis.turnAbsolute(390_deg, 600_rpm, {0.04, 0, 0.7}, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(10_in, 600_rpm, {0.07, 0, 1.5}, 390_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	pros::delay(500);

	chassis.moveToPoint({35_in, 20_in}, 600_rpm, {0.045, 0, 0.1}, {0.5, 0, 0.05}, 1_in, true, true, 0);
	// pros::delay(800);
	// intake.stop();
	// pros::delay(500);
	arm.allianceStake();
	chassis.moveToPoint({38_in, 56_in}, 600_rpm, {0.045, 0, 0.1}, {0.5, 0, 0.05}, 5_in, false, true);
	chassis.turnAbsolute(405_deg, 600_rpm, {0.03, 0, 0.8}, 3, 5, TurnWheel::BOTH, 0);
	arm.descoreStake();
}