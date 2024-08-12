#include "routes.hpp"
#include "16868C/subsystems/chassis/inline.hpp"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"

using namespace lib16868C;

void waitUntilButton(okapi::ControllerDigital btn = okapi::ControllerDigital::A) {
	while (!master.getDigital(btn)) pros::delay(10);
}
/*
void redSoloAWP() {
	odometry.update({124_in, 60_in, 25_deg});

	pros::Task relArm([&]() {
		intake.outtake();
		pros::delay(200);
		intake.stop();
		arm.allianceStake();
	});
	pros::delay(500);
	chassis.moveDistance(11_in, 600_rpm, {0.06, 0, 1.5}, 25_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	arm.defaultPos(8000);
	pros::delay(300);
	chassis.moveToPoint({113_in, 58_in}, 400_rpm, {0.07, 0, 0.1}, {4, 0, 0.05}, 1_in, true, true, 1500);
	chassis.turnAbsolute(20_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-15_in, 150_rpm, {1, 0, 1}, 20_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	// pros::delay(200);
	clamp.extend();
	pros::delay(100);
	// waitUntilButton();
	chassis.turnAbsolute(-90_deg, 600_rpm, {0.022, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1200);

	intake.intakeMogo();
	chassis.moveDistance(25_in, 600_rpm, {0.07, 0, 1.5}, -90_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	// chassis.moveToPoint({106_in, 18_in}, 600_rpm, {0.04, 0, 0.15}, {2.5, 0, 0.05}, 1_in, false, true, 0);
	// pros::delay(500);
	chassis.turnAbsolute(-170_deg, 600_rpm, {0.015, 0, 0.95}, 3, 5, TurnWheel::BOTH, 800);
	// waitUntilButton();
	pros::delay(400);
	chassis.moveDistance(16_in, 600_rpm, {0.07, 0, 1.5}, -180_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	// chassis.moveToPoint({98_in, 19.5_in}, 600_rpm, {0.045, 0, 0.1}, {0.7, 0, 0.05}, 1_in, false, true, 0);
	pros::delay(800);
	chassis.moveDistance(-10_in, 600_rpm, {0.07, 0, 1.5}, -180_deg, 300_rpm, {0.05, 0, 0.1}, 1000);

	chassis.turnAbsolute(-210_deg, 600_rpm, {0.04, 0, 0.7}, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(11_in, 600_rpm, {0.07, 0, 1.5}, -210_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	pros::delay(400);

	// chassis.moveToPoint({110_in, 20_in}, 600_rpm, {0.045, 0, 0.1}, {1, 0, 0.05}, 3_in, true, true, 1500);
	// pros::delay(800);
	// intake.stop();
	// pros::delay(500);
	chassis.moveDistance(-15_in, 600_rpm, {0.07, 0, 1.5}, -180_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	chassis.turnAbsolute(-275_deg, 600_rpm, {0.015, 0, 0.9}, 3, 5, TurnWheel::BOTH, 800);
	arm.allianceStake();
	chassis.moveToPoint({96_in, 62_in}, 600_rpm, {0.045, 0, 0.1}, {1.2, 0, 0.05}, 5_in, false, true, 1500);
	chassis.turnAbsolute(-225_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
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
	chassis.moveDistance(11_in, 600_rpm, {0.07, 0, 1.5}, 155_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	arm.defaultPos(8000);
	pros::delay(300);
	chassis.moveToPoint({21_in, 52_in}, 400_rpm, {0.07, 0, 0.1}, {5, 0, 0.1}, 1_in, true, true, 0);
	chassis.turnAbsolute(150_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-10_in, 300_rpm, {0.8, 0, 1}, 150_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	// pros::delay(200);
	clamp.extend();
	pros::delay(100);
	// waitUntilButton();
	chassis.turnAbsolute(270_deg, 600_rpm, {0.022, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1200);

	intake.intakeMogo();
	chassis.moveDistance(22_in, 600_rpm, {0.07, 0, 1.5}, 270_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	// chassis.moveToPoint({106_in, 18_in}, 600_rpm, {0.04, 0, 0.15}, {2.5, 0, 0.05}, 1_in, false, true, 0);
	// pros::delay(500);
	chassis.turnAbsolute(350_deg, 600_rpm, {0.015, 0, 0.95}, 3, 5, TurnWheel::BOTH, 800);
	// waitUntilButton();
	pros::delay(400);
	chassis.moveDistance(14_in, 600_rpm, {0.07, 0, 1.5}, 360_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	// chassis.moveToPoint({98_in, 19.5_in}, 600_rpm, {0.045, 0, 0.1}, {0.7, 0, 0.05}, 1_in, false, true, 0);
	pros::delay(800);
	chassis.moveDistance(-11_in, 600_rpm, {0.07, 0, 1.5}, 360_deg, 300_rpm, {0.05, 0, 0.1}, 1000);

	chassis.turnAbsolute(390_deg, 600_rpm, {0.045, 0, 0.7}, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(13_in, 600_rpm, {0.07, 0, 1.5}, 390_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	pros::delay(400);

	// chassis.moveToPoint({110_in, 20_in}, 600_rpm, {0.045, 0, 0.1}, {1, 0, 0.05}, 3_in, true, true, 1500);
	// pros::delay(800);
	// intake.stop();
	// pros::delay(500);
	chassis.moveDistance(-15_in, 600_rpm, {0.07, 0, 1.5}, 360_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	chassis.turnAbsolute(450_deg, 600_rpm, {0.015, 0, 0.9}, 3, 5, TurnWheel::BOTH, 800);
	arm.allianceStake();
	pros::Task relGoal = pros::Task([&]() {
		pros::delay(500);
		clamp.retract();
	});
	chassis.moveToPoint({35_in, 53_in}, 600_rpm, {0.045, 0, 0.1}, {1, 0, 0.05}, 5_in, false, true, 1500);
	chassis.turnAbsolute(405_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
	arm.descoreStake();

	// pros::Task relArm([&]() {
	// 	intake.outtake();
	// 	pros::delay(200);
	// 	intake.stop();
	// 	arm.allianceStake();
	// });
	// pros::delay(500);
	// chassis.moveDistance(11_in, 600_rpm, {0.07, 0, 1.5}, 155_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	// arm.defaultPos(8000);
	// pros::delay(300);
	// chassis.moveToPoint({18_in, 46_in}, 400_rpm, {0.055, 0, 0.1}, {0.5, 0, 0.05}, 1_in, true, true, 0);
	// // waitUntilButton();
	// chassis.turnAbsolute(180_deg, 600_rpm, {0.025, 0, 0.8}, 3, 5, TurnWheel::BOTH, 0);
	// chassis.moveDistance(-15_in, 150_rpm, {1, 0, 1}, 180_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	// // pros::delay(200);
	// clamp.retract();
	// pros::delay(100);
	// chassis.turnAbsolute(270_deg, 600_rpm, {0.03, 0, 0.8}, 3, 5, TurnWheel::BOTH, 0);

	// intake.intakeMogo();
	// chassis.moveToPoint({30_in, 15_in}, 600_rpm, {0.045, 0, 0.1}, {0.5, 0, 0.05}, 1_in, false, true, 0);
	// chassis.turnAbsolute(340_deg, 600_rpm, {0.03, 0, 0.8}, 3, 5, TurnWheel::BOTH, 0);
	// // waitUntilButton();
	// pros::delay(400);
	// chassis.moveToPoint({42_in, 14_in}, 600_rpm, {0.045, 0, 0.1}, {0.7, 0, 0.05}, 1_in, false, true, 0);
	// pros::delay(800);
	// chassis.moveDistance(-10_in, 600_rpm, {0.07, 0, 1.5}, 360_deg, 300_rpm, {0.05, 0, 0.1}, 0);

	// chassis.turnAbsolute(390_deg, 600_rpm, {0.04, 0, 0.7}, 3, 5, TurnWheel::BOTH, 0);
	// chassis.moveDistance(10_in, 600_rpm, {0.07, 0, 1.5}, 390_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	// pros::delay(500);

	// chassis.moveToPoint({35_in, 20_in}, 600_rpm, {0.045, 0, 0.1}, {0.5, 0, 0.05}, 1_in, true, true, 0);
	// // pros::delay(800);
	// // intake.stop();
	// // pros::delay(500);
	// arm.allianceStake();
	// chassis.moveToPoint({38_in, 56_in}, 600_rpm, {0.045, 0, 0.1}, {0.5, 0, 0.05}, 5_in, false, true);
	// chassis.turnAbsolute(405_deg, 600_rpm, {0.03, 0, 0.8}, 3, 5, TurnWheel::BOTH, 0);
	// arm.descoreStake();
}

void redSoloAWPSafe() {
	odometry.update({124_in, 60_in, 25_deg});

	pros::Task relArm([&]() {
		intake.outtake();
		pros::delay(200);
		intake.stop();
		arm.allianceStake();
	});
	pros::delay(500);
	chassis.moveDistance(13_in, 300_rpm, {0.07, 0, 1.5}, 25_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	arm.defaultPos(6000);
	pros::delay(500);

	chassis.moveDistance(-25_in, 600_rpm, {0.07, 0, 1.5}, 20_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	chassis.turnAbsolute(20_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-10_in, 150_rpm, {1, 0, 1}, 20_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	// pros::delay(200);
	clamp.extend();
	pros::delay(100);
	// waitUntilButton();

	chassis.turnAbsolute(28_deg, 600_rpm, {0.03, 0, 0.8}, 3, 5, TurnWheel::BOTH, 1200);
	chassis.moveDistance(14_in, 600_rpm, {0.07, 0, 1.5}, 28_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	stick.extend();
	pros::delay(300);
	chassis.moveDistance(-10_in, 600_rpm, {0.07, 0, 1.5}, 30_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	stick.retract();
	intake.intakeMogo();
	chassis.turnAbsolute(80_deg, 600_rpm, {0.022, 0, 0.9}, 3, 5, TurnWheel::RIGHT, 1000);
	chassis.moveDistance(10_in, 300_rpm, {0.07, 0, 1.5}, 80_deg, 300_rpm, {0.05, 0, 0.1}, 500);
	chassis.moveDistance(-10_in, 600_rpm, {0.08, 0, 1.5}, 80_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	pros::delay(800);

	chassis.turnAbsolute(245_deg, 600_rpm, {0.02, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1500);
	chassis.moveDistance(31_in, 600_rpm, {0.07, 0, 1.5}, 245_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	pros::delay(500);

	chassis.turnAbsolute(90_deg, 600_rpm, {0.021, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
	arm.allianceStake();
	pros::Task relGoal = pros::Task([&]() {
		pros::delay(500);
		clamp.retract();
	});
	chassis.moveDistance(28_in, 600_rpm, {0.07, 0, 1.5}, 90_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	chassis.turnAbsolute(135_deg, 600_rpm, {0.022, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
	arm.descoreStake();
}
void blueSoloAWPSafe() {
	odometry.update({124_in, 60_in, -25_deg});

	pros::delay(1500);

	pros::Task relArm([&]() {
		intake.outtake();
		pros::delay(200);
		intake.stop();
		arm.allianceStake();
	});
	pros::delay(500);
	chassis.moveDistance(13_in, 300_rpm, {0.07, 0, 1.5}, -25_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	arm.defaultPos(6000);
	pros::delay(500);

	chassis.moveDistance(-25_in, 600_rpm, {0.07, 0, 1.5}, -20_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	chassis.turnAbsolute(-20_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-10_in, 150_rpm, {1, 0, 1}, -20_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	// pros::delay(200);
	clamp.extend();
	pros::delay(100);
	// waitUntilButton();

	chassis.turnAbsolute(-55_deg, 600_rpm, {0.03, 0, 0.8}, 3, 5, TurnWheel::BOTH, 1200);
	chassis.moveDistance(14_in, 600_rpm, {0.07, 0, 1.5}, -55_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	stick.extend();
	pros::delay(300);
	chassis.moveDistance(-10_in, 600_rpm, {0.07, 0, 1.5}, -30_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	stick.retract();
	intake.intakeMogo();
	chassis.turnAbsolute(30_deg, 600_rpm, {0.022, 0, 0.9}, 3, 5, TurnWheel::RIGHT, 1000);
	pros::delay(1000);

	chassis.turnAbsolute(115_deg, 600_rpm, {0.02, 0, 0.8}, 3, 5, TurnWheel::BOTH, 1500);
	chassis.moveDistance(31_in, 600_rpm, {0.07, 0, 1.5}, 115_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	pros::delay(200);

	chassis.turnAbsolute(270_deg, 600_rpm, {0.021, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
	arm.allianceStake();
	pros::Task relGoal = pros::Task([&]() {
		pros::delay(500);
		clamp.retract();
	});
	chassis.moveDistance(26_in, 600_rpm, {0.07, 0, 1.5}, 270_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	chassis.turnAbsolute(225_deg, 600_rpm, {0.022, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
	arm.descoreStake();
}

void redRush() {
	odometry.update({112_in, 98_in, -20_deg});

	pros::Task relArm([&]() {
		intake.outtake();
		pros::delay(200);
		intake.stop();
		arm.defaultPos();
	});
	chassis.moveDistance(-45_in, 600_rpm, {0.07, 0, 1.5}, -20_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	clamp.extend();
	intake.intakeMogo();
	chassis.turnAbsolute(60_deg, 400_rpm, {0.05, 0, 0.8}, 3, 5, TurnWheel::RIGHT, 0);
	
	arm.allianceStake();
	chassis.turnAbsolute(-45_deg, 600_rpm, {0.03, 0, 0.9}, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveToPoint({110_in, 76_in}, 400_rpm, {0.045, 0, 0.1}, {2, 0, 0.05}, 1_in, false, true, 0);

	intake.stop();
	// chassis.turnAbsolute(0_deg, 600_rpm, {0.04, 0, 0.8}, 3, 5, TurnWheel::BOTH, 0);
	arm.defaultPos(8000);
}

void redRightAWP() {
	odometry.update({124_in, 72_in, -30_deg});

	pros::Task relArm([&]() {
		intake.outtake();
		pros::delay(200);
		intake.stop();
		arm.allianceStake();
	});
	pros::delay(500);
	chassis.moveDistance(11_in, 600_rpm, {0.07, 0, 1.5}, -25_deg, 300_rpm, {0.05, 0, 0.1}, 800);
	arm.defaultPos(8000);
	pros::delay(300);
	chassis.moveToPoint({118_in, 79_in}, 400_rpm, {0.055, 0, 0.1}, {1.5, 0, 0.05}, 1_in, true, true, 1500);
	
	chassis.turnAbsolute(0_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 700);
	chassis.moveDistance(-15_in, 150_rpm, {1.1, 0, 1}, 0_deg, 300_rpm, {0.05, 0, 0.1}, 2000);
	// pros::delay(200);
	clamp.extend();
	pros::delay(100);
	// waitUntilButton();
	chassis.turnAbsolute(90_deg, 600_rpm, {0.023, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);

	intake.intakeMogo();
	chassis.moveDistance(20_in, 600_rpm, {0.07, 0, 1.5}, 90_deg, 300_rpm, {0.05, 0, 0.1}, 1000);

	chassis.turnAbsolute(30_deg, 600_rpm, {0.025, 0, 0.8}, 3, 5, TurnWheel::BOTH, 1000);
	arm.moveTo(50);
	chassis.moveDistance(40_in, 600_rpm, {0.07, 0, 1.5}, 30_deg, 300_rpm, {0.05, 0, 0.1}, 3000);
	chassis.moveDistance(-10_in, 600_rpm, {0.07, 0, 1.5}, 30_deg, 300_rpm, {0.05, 0, 0.1}, 800);
	chassis.turnAbsolute(-135_deg, 600_rpm, {0.02, 0, 0.8}, 3, 5, TurnWheel::BOTH, 1500);
	arm.allianceStake();
	chassis.moveDistance(40_in, 600_rpm, {0.07, 0, 1.5}, -135_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	// chassis.moveToPoint({100_in, 75_in}, 600_rpm, {0.045, 0, 0.1}, {0.5, 0, 0.05}, 1_in, false, true, 0);
	// chassis.turnAbsolute(-135_deg, 600_rpm, {0.025, 0, 0.8}, 3, 5, TurnWheel::BOTH, 0);
	arm.descoreStake();
	// chassis.moveToPoint({140_in, 140_in}, 600_rpm, {0.045, 0, 0.1}, {1, 0, 0.05}, 1_in, false, true, 0);
}
void blueLeftAWP() {
	odometry.update({20_in, 72_in, -150_deg});

	pros::Task relArm([&]() {
		intake.outtake();
		pros::delay(200);
		intake.stop();
		arm.allianceStake();
	});
	pros::delay(500);
	chassis.moveDistance(11_in, 600_rpm, {0.07, 0, 1.5}, -150_deg, 300_rpm, {0.05, 0, 0.1}, 800);
	arm.defaultPos(8000);
	pros::delay(300);
	chassis.moveToPoint({19_in, 85_in}, 400_rpm, {0.055, 0, 0.1}, {1.5, 0, 0.05}, 1_in, true, true, 1500);
	
	chassis.turnAbsolute(-180_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 700);
	chassis.moveDistance(-15_in, 150_rpm, {1.1, 0, 1}, -180_deg, 300_rpm, {0.05, 0, 0.1}, 2000);
	// pros::delay(200);
	clamp.extend();
	pros::delay(100);
	// waitUntilButton();
	chassis.turnAbsolute(-270_deg, 600_rpm, {0.023, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);

	intake.intakeMogo();
	chassis.moveDistance(20_in, 600_rpm, {0.07, 0, 1.5}, -270_deg, 300_rpm, {0.05, 0, 0.1}, 1000);

	// chassis.turnAbsolute(-210_deg, 600_rpm, {0.025, 0, 0.8}, 3, 5, TurnWheel::BOTH, 1000);
	// arm.moveTo(50);
	// chassis.moveDistance(40_in, 600_rpm, {0.07, 0, 1.5}, -210_deg, 300_rpm, {0.05, 0, 0.1}, 3000);
	// chassis.moveDistance(-10_in, 600_rpm, {0.07, 0, 1.5}, -210_deg, 300_rpm, {0.05, 0, 0.1}, 800);
	// chassis.turnAbsolute(-45_deg, 600_rpm, {0.02, 0, 0.8}, 3, 5, TurnWheel::BOTH, 1500);
	arm.allianceStake();
	chassis.moveDistance(-24_in, 600_rpm, {0.07, 0, 1.5}, -240_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	chassis.turnAbsolute(-405_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1300);
	// chassis.moveDistance(5_in, 600_rpm, {0.07, 0, 1.5}, -405_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	// chassis.moveToPoint({100_in, 75_in}, 600_rpm, {0.045, 0, 0.1}, {0.5, 0, 0.05}, 1_in, false, true, 0);
	// chassis.turnAbsolute(-135_deg, 600_rpm, {0.025, 0, 0.8}, 3, 5, TurnWheel::BOTH, 0);
	arm.descoreStake();
}

void skills() {
	odometry.update({122_in, 72_in, 0_deg});

	pros::Task relArm([&]() {
		intake.outtake();
		pros::delay(200);
		intake.stop();
		arm.allianceStake();
	});
	pros::delay(500);
	chassis.moveDistance(7_in, 600_rpm, {0.07, 0, 1.5}, 0_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	arm.defaultPos(8000);
	pros::delay(300);
	chassis.moveDistance(-9_in, 600_rpm, {0.08, 0, 1.5}, 0_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	chassis.turnAbsolute(90_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(-17_in, 300_rpm, {0.5, 0, 1}, 90_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	// chassis.moveToPoint({124_in, 54_in}, 600_rpm, {0.045, 0, 0.1}, {1, 0, 0.5}, 1_in, true, true, 0);
	clamp.extend();
	pros::delay(100);

	intake.intakeMogo();
	chassis.turnAbsolute(180_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(24_in, 600_rpm, {0.07, 0, 1.5}, 180_deg, 300_rpm, {0.05, 0, 0.1}, 0);

	chassis.turnAbsolute(315_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(65_in, 600_rpm, {0.07, 0, 1.5}, 315_deg, 300_rpm, {0.05, 0, 0.1}, 0);
}

void blueSoloAWPSafeAdjusted() {
	odometry.update({124_in, 60_in, -25_deg});

	// pros::delay(1500);

	pros::Task relArm([&]() {
		intake.outtake();
		pros::delay(200);
		intake.stop();
		// arm.allianceStake();
	});
	// pros::delay(500);
	// chassis.moveDistance(13_in, 300_rpm, {0.07, 0, 1.5}, -25_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	// arm.defaultPos(6000);
	// pros::delay(500);

	chassis.moveDistance(-15_in, 600_rpm, {0.07, 0, 1.5}, -20_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	chassis.turnAbsolute(-20_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-10_in, 150_rpm, {1, 0, 1}, -20_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	// pros::delay(200);
	clamp.extend();
	pros::delay(100);
	// waitUntilButton();

	chassis.turnAbsolute(-55_deg, 600_rpm, {0.03, 0, 0.8}, 3, 5, TurnWheel::BOTH, 1200);
	intake.intakeMogo();
	chassis.moveDistance(15_in, 600_rpm, {0.07, 0, 1.5}, -55_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	stick.extend();
	pros::delay(300);
	chassis.moveDistance(-10_in, 600_rpm, {0.07, 0, 1.5}, -30_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	stick.retract();
	chassis.turnAbsolute(30_deg, 600_rpm, {0.022, 0, 0.9}, 3, 5, TurnWheel::RIGHT, 1000);
	chassis.moveDistance(10_in, 600_rpm, {0.07, 0, 1.5}, 30_deg, 300_rpm, {0.05, 0, 0.1}, 500);
	pros::delay(1000);

	chassis.turnAbsolute(130_deg, 600_rpm, {0.02, 0, 0.8}, 3, 5, TurnWheel::BOTH, 1500);
	chassis.moveDistance(31_in, 600_rpm, {0.07, 0, 1.5}, 130_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	pros::delay(200);

	chassis.turnAbsolute(270_deg, 600_rpm, {0.021, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
	arm.allianceStake();
	pros::Task relGoal = pros::Task([&]() {
		pros::delay(500);
		clamp.retract();
	});
	chassis.moveDistance(26_in, 600_rpm, {0.07, 0, 1.5}, 270_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	chassis.turnAbsolute(225_deg, 600_rpm, {0.022, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
	arm.descoreStake();
}

void redElimScore() {
	odometry.update({124_in, 60_in, -25_deg});

	pros::Task relArm([&]() {
		intake.outtake();
		pros::delay(200);
		intake.stop();
		arm.allianceStake();
	});
	pros::delay(500);
	chassis.moveDistance(13_in, 300_rpm, {0.07, 0, 1.5}, -25_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	arm.defaultPos(6000);
	pros::delay(500);

	chassis.moveDistance(-25_in, 600_rpm, {0.07, 0, 1.5}, -20_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	chassis.turnAbsolute(-20_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-10_in, 150_rpm, {1, 0, 1}, -20_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	// pros::delay(200);
	clamp.extend();
	pros::delay(100);
	// waitUntilButton();

	chassis.turnAbsolute(-60_deg, 600_rpm, {0.03, 0, 0.8}, 3, 5, TurnWheel::BOTH, 1200);
	chassis.moveDistance(14_in, 600_rpm, {0.07, 0, 1.5}, -60_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	stick.extend();
	pros::delay(300);
	chassis.moveDistance(-10_in, 600_rpm, {0.07, 0, 1.5}, -60_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	stick.retract();
	intake.intakeMogo();
	chassis.turnAbsolute(-80_deg, 600_rpm, {0.022, 0, 0.9}, 3, 5, TurnWheel::RIGHT, 1000);
	chassis.moveDistance(10_in, 300_rpm, {0.07, 0, 1.5}, -80_deg, 300_rpm, {0.05, 0, 0.1}, 500);
	chassis.moveDistance(-10_in, 600_rpm, {0.08, 0, 1.5}, -80_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	pros::delay(800);

	chassis.turnAbsolute(90_deg, 600_rpm, {0.02, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1500);
	chassis.moveDistance(31_in, 600_rpm, {0.07, 0, 1.5}, 90_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	pros::delay(500);

	chassis.turnAbsolute(20_deg, 600_rpm, {0.02, 0, 0.8}, 3, 5, TurnWheel::BOTH, 1000);
	stick.extend();
	pros::Task relGoal = pros::Task([&]() {
		pros::delay(500);
		clamp.retract();
	});
	chassis.moveDistance(26_in, 600_rpm, {0.07, 0, 1.5}, 20_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	chassis.turnAbsolute(-20_deg, 600_rpm, {0.03, 0, 0.7}, 3, 5, TurnWheel::BOTH, 1000);
	stick.retract();
}
void blueElimScore() {
	odometry.update({124_in, 60_in, 25_deg});

	pros::Task relArm([&]() {
		intake.outtake();
		pros::delay(200);
		intake.stop();
		arm.allianceStake();
	});
	pros::delay(500);
	chassis.moveDistance(13_in, 300_rpm, {0.07, 0, 1.5}, 25_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	// chassis.turnAbsolute(22_deg, 300_rpm, {0.02, 0, 0.8}, 1, 5, TurnWheel::LEFT, 0);
	arm.defaultPos(3000);
	pros::delay(500);
	// chassis.turnAbsolute(28_deg, 300_rpm, {0.02, 0, 0.8}, 1, 5, TurnWheel::LEFT, 0);

	chassis.moveDistance(-25_in, 600_rpm, {0.07, 0, 1.5}, 20_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	chassis.turnAbsolute(20_deg, 600_rpm, {0.025, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-10_in, 150_rpm, {1, 0, 1}, 20_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	// pros::delay(200);
	clamp.extend();
	pros::delay(100);
	// waitUntilButton();

	chassis.turnAbsolute(28_deg, 600_rpm, {0.03, 0, 0.8}, 3, 5, TurnWheel::BOTH, 1200);
	chassis.moveDistance(13_in, 600_rpm, {0.07, 0, 1.5}, 28_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	stick.extend();
	pros::delay(300);
	chassis.moveDistance(-10_in, 600_rpm, {0.07, 0, 1.5}, 30_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	stick.retract();
	intake.intakeMogo();
	chassis.turnAbsolute(80_deg, 600_rpm, {0.022, 0, 0.9}, 3, 5, TurnWheel::RIGHT, 1000);
	chassis.moveDistance(10_in, 300_rpm, {0.07, 0, 1.5}, 80_deg, 300_rpm, {0.05, 0, 0.1}, 500);
	chassis.moveDistance(-10_in, 600_rpm, {0.08, 0, 1.5}, 80_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	pros::delay(800);

	chassis.turnAbsolute(240_deg, 600_rpm, {0.02, 0, 0.9}, 3, 5, TurnWheel::BOTH, 1500);
	chassis.moveDistance(31_in, 600_rpm, {0.07, 0, 1.5}, 240_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	pros::delay(500);

	chassis.turnAbsolute(340_deg, 600_rpm, {0.02, 0, 0.8}, 3, 5, TurnWheel::BOTH, 1000);
	stick.extend();
	pros::Task relGoal = pros::Task([&]() {
		pros::delay(500);
		clamp.retract();
	});
	chassis.moveDistance(26_in, 600_rpm, {0.07, 0, 1.5}, 340_deg, 300_rpm, {0.05, 0, 0.1}, 1000);
	chassis.turnAbsolute(270_deg, 600_rpm, {0.03, 0, 0.7}, 3, 5, TurnWheel::BOTH, 1000);
	stick.retract();
}*/