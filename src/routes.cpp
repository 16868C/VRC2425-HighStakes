#include "routes.hpp"
#include "16868C/subsystems/chassis/inline.hpp"
#include "robotconfig.hpp"

using namespace lib16868C;

void waitUntilButton(okapi::ControllerDigital btn = okapi::ControllerDigital::A) {
	while (!master.getDigital(btn)) pros::delay(10);
}

void redRingAWP() {
	odometry.update({30_in, 21.5_in, 90_deg});
	inertial.set_rotation(90_deg);
	intake.setTargetRing(RingColour::RED);
	
	intake.intake();
	chassis.moveToPoint({25_in, 61_in}, 850, {.minRPM=100_rpm, .earlyExitRadius=3_in});
	chassis.turnAbsolute(180_deg, 1000, {.maxRPM=400_rpm, .gains={1.1, 0, 1.5}, .turnWheel=TurnWheel::RIGHT});
	chassis.moveDistance(10_in, 180_deg, 700, {});

	pros::Task([&] {
		pros::delay(800);
		intake.hold();
	});
	chassis.moveToPoint({34.5_in, 48_in}, 1700, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(200);
	intake.mogo();

	chassis.turnAbsolute(175_deg, 850, {.gains={0.56, 0, 2}});
	chassis.moveToPoint({14_in, 50_in}, 1000, {});
	
	chassis.turnAbsolute(270_deg, 950, {.gains={0.36, 0, 2}});
	chassis.moveToPoint({17_in, 35_in}, 700, {.minRPM=300_rpm, .earlyExitRadius=3_in});
	chassis.moveToPoint({35_in, 26_in}, 1050, {.headingGains={0.7, 0, 1}});
	pros::delay(1100);
	clamp.retract();
	intake.intake();

	chassis.turnAbsolute(35_deg, 750, {.gains={1.1, 0, 2}, .turnWheel=TurnWheel::RIGHT});
	intakeRaiser.retract();
	chassis.moveDistance(13_in, 35_deg, 750, {});
	intakeRaiser.extend();
	pros::delay(200);
	pros::Task([&] {
		pros::delay(600);
		intake.hold();
		intakeFirst.moveVoltage(-12000);
	});
	chassis.moveDistance(-12_in, 0_deg, 1000, {.distGains={0.08, 0, 1.5}, .headingGains={0.45, 0, 1}});
	chassis.moveDistance(16_in, 0_deg, 800, {});
	chassis.turnAbsolute(90_deg, 900, {.gains={0.42, 0, 2}, .errorMargin=1_deg});
	pros::Task([&] {
		intake.outtake();
		pros::delay(100);
		intake.stop();
	});
	chassis.moveDistance(-8_in, 90_deg, 900, {.maxRPM=300_rpm});
	intake.mogo();
	pros::delay(700);
	intake.outtake();
	pros::Task([&] {
		pros::delay(300);
		intake.mogo();
	});
	chassis.moveDistance(16_in, 90_deg, 800, {.maxRPM=400_rpm, .minRPM=200_rpm, .headingGains={0, 0, 0}, .exitDist=3_in});
}
void blueRingAWP() {
	odometry.update({114_in, 21.5_in, 90_deg});
	inertial.set_rotation(90_deg);
	intake.setTargetRing(RingColour::BLUE);
	
	intake.intake();
	chassis.moveToPoint({110_in, 60.5_in}, 850, {.minRPM=100_rpm, .earlyExitRadius=3_in});
	chassis.turnAbsolute(0_deg, 1100, {.maxRPM=400_rpm, .gains={1.1, 0, 1.5}, .turnWheel=TurnWheel::LEFT});
	chassis.moveDistance(10_in, 0_deg, 700, {});

	pros::Task([&] {
		pros::delay(700);
		intake.hold();
	});
	chassis.moveToPoint({97_in, 51_in}, 1700, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(200);
	intake.mogo();

	chassis.turnAbsolute(10_deg, 800, {.gains={0.56, 0, 2}});
	chassis.moveToPoint({125_in, 53_in}, 1200, {});
	
	chassis.turnAbsolute(-90_deg, 900, {.gains={0.43, 0, 2}});
	chassis.moveToPoint({123_in, 37_in}, 750, {.minRPM=300_rpm, .earlyExitRadius=3_in});
	chassis.moveToPoint({100_in, 24_in}, 1100, {.headingGains={0.7, 0, 1}});
	pros::delay(1000);
	clamp.retract();
	intake.intake();

	chassis.turnAbsolute(150_deg, 550, {.gains={1.6, 0, 2}, .turnWheel=TurnWheel::LEFT});
	intakeRaiser.retract();
	chassis.moveDistance(13_in, 150_deg, 750, {});
	intakeRaiser.extend();
	pros::delay(200);
	pros::Task([&] {
		pros::delay(700);
		intake.hold();
		intakeFirst.moveVoltage(-12000);
	});
	chassis.moveDistance(-13_in, 180_deg, 750, {.distGains={0.08, 0, 1.5}, .headingGains={0.45, 0, 1}});
	chassis.moveDistance(10.5_in, 180_deg, 750, {});
	chassis.turnAbsolute(90_deg, 1000, {.gains={0.415, 0, 2}, .errorMargin=1_deg, .angularVelThreshold=2_deg});
	pros::Task([&] {
		intake.outtake();
		pros::delay(100);
		intake.stop();
	});
	chassis.moveDistance(-8_in, 90_deg, 950, {.maxRPM=400_rpm});
	intake.mogo();
	pros::delay(700);
	intake.outtake();
	pros::Task([&] {
		pros::delay(300);
		intake.mogo();
	});
	chassis.moveDistance(18_in, 90_deg, 650, {.maxRPM=400_rpm, .minRPM=200_rpm, .headingGains={0, 0, 0}, .exitDist=3_in});
}

void redGoalAWP() {
	odometry.update({104_in, 19_in, 56_deg});
	inertial.set_rotation(56_deg);
	intake.setTargetRing(RingColour::RED);

	intake.intake();
	doinker.extend();
	claw.extend();
	chassis.moveToPoint({117_in, 44_in}, 750, {.minRPM=600_rpm, .earlyExitRadius=3_in});
	chassis.moveToPoint({123.5_in, 61_in}, 350, {.minRPM=200_rpm});
	claw.retract();

	chassis.moveToPoint({105_in, 30_in}, 1300, {.reverse=true});
	claw.extend();
	pros::Task([&] {
		pros::delay(400);
		doinker.retract();
		claw.retract();
	});
	chassis.turnAbsolute(235_deg, 1150, {});
	
	chassis.moveToPoint({118_in, 58_in}, 1250, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(200);
	intake.mogo();
	pros::delay(500);

	clamp.retract();
	chassis.turnAbsolute(-10_deg, 1100, {});
	chassis.moveToPoint({95_in, 52_in}, 1150, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(50);

	chassis.turnAbsolute(225_deg, 1150, {.gains={1.2, 0.2, 0.1}});
	intakeRaiser.retract();
	intake.mogo();
	chassis.moveDistance(23_in, 225_deg, 850, {});
	// chassis.moveToPoint({80_in, 39_in}, 0, {.maxRPM=300_rpm, .distGains={0.12, 0, 1.5}});
	pros::delay(300);
	intakeRaiser.extend();

	chassis.turnAbsolute(270_deg, 1100, {.gains={1.2, 0.2, 0.1}, .turnWheel=TurnWheel::LEFT});
	intake.stop();
	pros::Task([&] {
		pto.retract();
		pros::delay(500);
		arm.allianceStake();
	});
	pros::delay(100);
	chassis.moveDistance(9.5_in, 270_deg, 750, {});
	chassis.turnAbsolute(257_deg, 850, {.gains={1.2, 0.2, 0.1}, .errorMargin=1_deg, .angularVelThreshold=2_deg});
	chassis.moveDistance(10_in, 257_deg, 850, {.distGains={0.09, 0, 0.011}});
	arm.defaultPos();
	pros::delay(200);

	chassis.moveToPoint({90_in, 30_in}, 1000, {.minRPM=200_rpm, .earlyExitRadius=2_in, .reverse=true});
	pto.extend();
	chassis.turnAbsolute(121_deg, 900, {.gains={1.2, 0.2, 0.1}});
	chassis.moveDistance(14_in, 121_deg, 700, {.minRPM=150_rpm, .distGains={0.09, 0, 0.011}, .headingGains={0, 0, 0}, .exitDist=2_in});

	// chassis.turnAbsolute(170_deg, 1000, {.gains={0.36, 0, 2}});
	// intake.outtake();
	// chassis.moveDistance(10_in, 170_deg, 0, {.maxRPM=200_rpm});

	// odometry.update({135_in, 21.5_in, 90_deg});
	// inertial.set_rotation(90_deg);
	// intake.setTargetRing(RingColour::RED);

	// doinker.extend();
	// claw.extend();
	// chassis.moveToPoint({132_in, 55_in}, 400, {.minRPM=600_rpm, .earlyExitRadius=3_in});
	// chassis.moveToPoint({127_in, 71_in}, 500, {.minRPM=200_rpm, .distGains={0.06, 0, 1.5}});
	// // chassis.moveToPose({126_in, 66_in, 110_deg}, 500, {.distGains={0.1, 0, 1.5}, .dlead=4_in, .glead=0, .gRadius=5_in});
	// claw.retract();
	// pros::delay(50);
	// chassis.moveToPoint({129_in, 45_in}, 1000, {.minRPM=600_rpm, .distGains={0.15, 0, 1.5}, .earlyExitRadius=5_in, .reverse=true});
	// chassis.moveToPoint({122_in, 30_in}, 0, {.distGains={0.2, 0, 1.5}, .headingGains={2, 0, 1}, .reverse=true});
	// claw.extend();
	// pros::Task([&] {
	// 	pros::delay(200);
	// 	doinker.retract();
	// 	claw.retract();
	// });
	// chassis.turnAbsolute(250_deg, 0, {.gains={0.32, 0, 2}, .dir=TurnDirection::CCW}, false, true);
	// pros::Task([] {
	// 	while (odometry.getPose().distTo(Pose(128_in, 48_in)) > 1) pros::delay(100);
	// 	clamp.extend();
	// });
	// chassis.moveToPoint({128_in, 48_in}, 0, {.maxRPM=400_rpm, .distGains={0.15, 0, 1.5}, .reverse=true});
	// clamp.extend();

	// chassis.moveToPoint({120_in, 40_in}, 0, {.distGains={0.15, 0, 1.5}});
	// chassis.turnAbsolute(110_deg, 0, {.gains={0.37, 0, 2}}, false, true);
	// intake.mogo();

	// chassis.moveToPoint({110_in, 60_in}, 0, {});
	// pros::delay(1000);
	// clamp.retract();
	// chassis.turnAbsolute(30_deg, 1000, {.gains={0.48, 0, 2}, .errorMargin=2.5_deg}, false, true);
	// pros::Task([] {
	// 	pros::delay(1100);
	// 	clamp.extend();
	// });
	// chassis.moveToPoint({87_in, 57_in}, 0, {.maxRPM=300_rpm, .distGains={0.15, 0, 1.5}, .reverse=true});
	
	// chassis.turnAbsolute(210_deg, 0, {.gains={0.34, 0, 2}}, false, true);
	// intakeRaiser.retract();
	// intake.mogo();
	// chassis.moveToPoint({68_in, 39_in}, 0, {.maxRPM=400_rpm, .distGains={0.12, 0, 1.5}});
	// pros::delay(700);
	// intakeRaiser.extend();
	// intake.intake();
	// clamp.retract();

	// pto.retract();
	// pros::delay(300);
	// arm.allianceStake();
	// chassis.turnAbsolute(270_deg, 0, {.gains={0.6, 0, 2}, .turnWheel=TurnWheel::RIGHT});
	// chassis.moveDistance(30_in, 270_deg, 0, {});
	// arm.defaultPos();
	// pros::delay(200);
	// chassis.moveDistance(-50_in, 270_deg, 0, {});

	// chassis.turnAbsolute(170_deg, 1000, {.gains={0.36, 0, 2}});
	// intake.outtake();
	// chassis.moveDistance(10_in, 170_deg, 0, {.maxRPM=200_rpm});
}
void blueGoalAWP() {
	odometry.update({12_in, 20_in, 67_deg});
	inertial.set_rotation(67_deg);
	intake.setTargetRing(RingColour::BLUE);

	intake.intake();
	doinker.extend();
	claw.extend();
	chassis.moveToPoint({22_in, 54_in}, 800, {.minRPM=600_rpm, .earlyExitRadius=4_in});
	chassis.moveToPoint({24_in, 64_in}, 350, {.minRPM=200_rpm, .distGains={0.06, 0, 1.5}, .headingGains={1.2, 0, 1}});
	claw.retract();

	chassis.moveToPoint({21_in, 40_in}, 1300, {.reverse=true});
	intake.hold();
	intakeFirst.moveVoltage(-12000);
	claw.extend();
	pros::Task([&] {
		pros::delay(400);
		doinker.retract();
		claw.retract();
	});
	chassis.turnAbsolute(-70_deg, 1300, {.gains={0.35, 0, 2}});

	chassis.moveToPoint({21_in, 60_in}, 1150, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(200);
	intake.mogo();
	pros::delay(600);
	// chassis.moveToPoint({22_in, 50_in}, 1000, {});

	clamp.retract();
	chassis.turnAbsolute(190_deg, 1150, {.gains={0.71, 0, 2}, .turnWheel=TurnWheel::LEFT});
	chassis.moveToPoint({49_in, 58_in}, 1300, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(50);

	chassis.turnAbsolute(-45_deg, 1100, {.gains={0.34, 0, 2}});
	intakeRaiser.retract();
	intake.mogo();
	chassis.moveDistance(25_in, -45_deg, 900, {.distGains={0.06, 0, 1.5}});
	pros::delay(500);
	intakeRaiser.extend();

	chassis.turnAbsolute(-90_deg, 1050, {.gains={0.8, 0, 2}, .turnWheel=TurnWheel::RIGHT});
	intake.stop();
	pros::Task([&] {
		pto.retract();
		pros::delay(500);
		arm.allianceStake();
	});
	pros::delay(300);
	chassis.moveDistance(11_in, -90_deg, 750, {});
	chassis.turnAbsolute(-60_deg, 850, {.gains={1.4, 0, 2}, .errorMargin=1_deg, .angularVelThreshold=2_deg});
	chassis.moveDistance(8_in, -60_deg, 850, {.distGains={0.08, 0, 1.5}});
	arm.defaultPos();
	pros::delay(200);

	chassis.moveToPoint({58_in, 40_in}, 900, {.minRPM=200_rpm, .earlyExitRadius=2_in, .reverse=true});
	pto.extend();
	chassis.turnAbsolute(60_deg, 900, {.gains={0.35, 0, 2}});
	intake.mogo();
	chassis.moveDistance(14_in, 60_deg, 700, {.minRPM=150_rpm, .distGains={0.06, 0, 1.5}, .headingGains={0, 0, 0}, .exitDist=2_in});

	// odometry.update({35_in, 21.5_in, 90_deg});
	// inertial.set_rotation(90_deg);
	// intake.setTargetRing(RingColour::BLUE);

	// doinker.extend();
	// claw.extend();
	// chassis.moveToPoint({30_in, 46_in}, 0, {.minRPM=600_rpm, .earlyExitRadius=3_in});
	// chassis.moveToPoint({26_in, 72_in}, 550, {.minRPM=200_rpm, .distGains={0.06, 0, 1.5}, .headingGains={1.2, 0, 2}});
	// claw.retract();
	// pros::delay(50);
	// pros::Task([] {
	// 	pros::delay(1450);
	// 	claw.extend();
	// });
	// chassis.moveToPoint({30_in, 43_in}, 1000, {.minRPM=600_rpm, .distGains={0.15, 0, 1.5}, .earlyExitRadius=5_in, .reverse=true});
	// chassis.moveToPoint({27_in, 32_in}, 800, {.distGains={0.15, 0, 1.5}, .headingGains={2, 0, 1}, .reverse=true});

	// chassis.turnAbsolute(-90_deg, 850, {.gains={0.39, 0, 2}});
	// doinker.retract();
	// claw.retract();
	// pros::Task([] {
	// 	pros::delay(1100);
	// 	clamp.extend();
	// });
	// chassis.moveToPoint({25_in, 58_in}, 1200, {.maxRPM=400_rpm, .distGains={0.15, 0, 1.5}, .reverse=true});
	// pros::delay(50);
	// intake.mogo();

	// chassis.moveToPoint({20_in, 40_in}, 800, {.distGains={0.15, 0, 1.5}});
	// intake.outtake();
	// chassis.turnAbsolute(80_deg, 850, {.gains={0.38, 0, 2}});
	// clamp.retract();

	// intake.intake();
	// chassis.moveToPoint({18_in, 65_in}, 800, {});
	// chassis.turnAbsolute(170_deg, 1000, {});
	// intake.hold();
	// intakeFirst.moveVoltage(-12000);
	// pros::Task([] {
	// 	pros::delay(900);
	// 	clamp.extend();
	// });
	// chassis.moveToPoint({50_in, 55_in}, 1000, {.maxRPM=400_rpm, .distGains={0.15, 0, 1.5}, .reverse=true});
	// pros::delay(50);
	// intake.mogo();
	// chassis.moveToPoint({44_in, 55_in}, 400, {.distGains={0.15, 0, 1.5}});

	// // chassis.turnAbsolute(-50_deg, 1000, {.gains={0.4, 0, 2}});
	// // intake.outtake();
	// // clamp.retract();
	// // intakeRaiser.retract();
	// // pros::Task([] {
	// // 	pros::delay(500);
	// // 	intake.intake();
	// // });
	// // chassis.moveToPoint({61_in, 37_in}, 1000, {.maxRPM=400_rpm, .headingGains={0.8, 0, 2}});
	// // chassis.turnAbsolute(-90_deg, 600, {.gains={0.5, 0, 2}});
	// // intakeRaiser.extend();
	// // chassis.turnAbsolute(20_deg, 600, {.gains={0.5, 0, 2}, .turnWheel=TurnWheel::RIGHT});
	// // chassis.turnAbsolute(-20_deg, 600, {.gains={0.7, 0, 2}, .angularVelThreshold=20_deg, .turnWheel=TurnWheel::RIGHT});
	// // intakeFirst.moveVoltage(-12000);
	// // chassis.turnAbsolute(90_deg, 1000, {.gains={0.45, 0, 2}});
	// // pros::Task([] {
	// // 	pros::delay(500);
	// // 	intake.mogo();
	// // });
	// // chassis.moveDistance(-20_in, 90_deg, 800, {.maxRPM=200_rpm});
	// // chassis.moveDistance(40_in, 90_deg, 0, {.maxRPM=400_rpm});

	// chassis.turnAbsolute(10_deg, 1000, {.gains={0.36, 0, 2}});
	// intake.outtake();
	// chassis.moveDistance(10_in, 10_deg, 0, {.maxRPM=200_rpm});
}

void redSoloAWP() {
	odometry.update({80_in, 20_in, -130_deg});
	inertial.set_rotation(-130_deg);
	intake.setTargetRing(RingColour::RED);

	pto.retract();
	pros::delay(500);
	arm.allianceStake();
	uint t = pros::millis();
	do {
		pros::delay(100);
		std::cout << arm.getError() << "\n";
	} while (arm.getError() > 0 && pros::millis() - t < 1000);
	chassis.moveDistance(8.5_in, -130_deg, 600, {.maxRPM=300_rpm, .distGains={0.4, 0, 1.5}, .headingGains={0.2, 0, 1.5}});
	arm.defaultPos();
	pros::delay(200);
	pto.extend();
	pros::Task([&] {
		pros::delay(500);
		arm.move(0);
		pto.extend();
		intake.mogo();
		pros::delay(250);
		intake.stop();
		pros::delay(250);
		intake.mogo();
	});

	chassis.moveToPoint({97_in, 40_in}, 1100, {.minRPM=400_rpm, .earlyExitRadius=2_in, .reverse=true});
	chassis.moveToPoint({99_in, 52_in}, 750, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(50);

	intake.mogo();
	chassis.turnAbsolute(-10_deg, 1000, {.gains={0.4, 0, 2}});
	chassis.moveToPoint({113_in, 50_in}, 950, {});

	chassis.turnAbsolute(-145_deg, 1100, {.gains={0.35, 0, 2}});
	intakeRaiser.retract();
	chassis.moveToPoint({80_in, 24_in}, 1100, {});
	chassis.moveToPoint({77_in, 23_in}, 700, {.maxRPM=200_rpm});
	intakeRaiser.extend();

	chassis.turnAbsolute(0_deg, 1400, {.gains={0.38, 0, 2}});
	chassis.moveToPoint({46_in, 21_in}, 1200, {.reverse=true});
	clamp.retract();
	intake.stop();
	pros::delay(50);

	chassis.turnAbsolute(-80_deg, 1000, {.gains={0.75, 0, 2}, .turnWheel=TurnWheel::LEFT});
	chassis.moveToPoint({47_in, 50_in}, 1400, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	intake.mogo();
	pros::delay(50);

	chassis.turnAbsolute(-170_deg, 1100, {.gains={0.4, 0, 2}});
	chassis.moveToPoint({24_in, 43_in}, 0, {});
	pros::delay(300);
	chassis.moveToPoint({70_in, 55_in}, 0, {.minRPM=400_rpm, .reverse=true});
}
void blueSoloAWP() {
	odometry.update({80_in, 20_in, -130_deg});
	inertial.set_rotation(-130_deg);
	intake.setTargetRing(RingColour::RED);

	pto.retract();
	pros::delay(500);
	arm.allianceStake();
	uint t = pros::millis();
	do {
		pros::delay(100);
		std::cout << arm.getError() << "\n";
	} while (arm.getError() > 0 && pros::millis() - t < 1000);
	chassis.moveDistance(8.5_in, -130_deg, 600, {.maxRPM=300_rpm, .distGains={0.4, 0, 1.5}, .headingGains={0.2, 0, 1.5}});
	arm.defaultPos();
	pros::delay(200);
	pto.extend();
	pros::Task([&] {
		pros::delay(500);
		arm.move(0);
		pto.extend();
		intake.mogo();
		pros::delay(250);
		intake.stop();
		pros::delay(250);
		intake.mogo();
	});

	chassis.moveToPoint({97_in, 40_in}, 1100, {.minRPM=400_rpm, .earlyExitRadius=2_in, .reverse=true});
	chassis.moveToPoint({99_in, 52_in}, 750, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(50);

	intake.mogo();
	chassis.turnAbsolute(-10_deg, 1000, {.gains={0.4, 0, 2}});
	chassis.moveToPoint({113_in, 50_in}, 950, {});

	// odometry.update({54_in, 20_in, -50_deg});
	// inertial.set_rotation(-50_deg);
	// intake.setTargetRing(RingColour::BLUE);

	// pto.retract();
	// pros::delay(500);
	// arm.allianceStake();
	// uint t = pros::millis();
	// do {
	// 	pros::delay(100);
	// 	std::cout << arm.getError() << "\n";
	// } while (arm.getError() > 0 && pros::millis() - t < 1000);
	// chassis.moveDistance(8.5_in, -50_deg, 600, {.maxRPM=300_rpm, .distGains={0.4, 0, 1.5}, .headingGains={0.2, 0, 1.5}});
	// arm.defaultPos();
	// pros::delay(200);
	// pto.extend();
	// pros::Task([&] {
	// 	pros::delay(500);
	// 	arm.move(0);
	// 	pto.extend();
	// 	intake.mogo();
	// 	pros::delay(250);
	// 	intake.stop();
	// 	pros::delay(250);
	// 	intake.mogo();
	// });

	// chassis.moveToPoint({48_in, 40_in}, 1100, {.minRPM=400_rpm, .earlyExitRadius=2_in, .reverse=true});
	// chassis.moveToPoint({46_in, 56_in}, 750, {.maxRPM=400_rpm, .reverse=true});
	// clamp.extend();
	// pros::delay(50);

	// intake.mogo();
	// chassis.turnAbsolute(-180_deg, 1000, {.gains={0.4, 0, 2}});
	// chassis.moveToPoint({30_in, 56_in}, 950, {});

	// chassis.turnAbsolute(-20_deg, 1100, {.gains={0.35, 0, 2}});
	// intakeRaiser.retract();
	// chassis.moveToPoint({64_in, 32_in}, 1100, {});
	// chassis.moveToPoint({67_in, 30_in}, 700, {.maxRPM=200_rpm});
	// intakeRaiser.extend();

	// chassis.turnAbsolute(180_deg, 1400, {.gains={0.38, 0, 2}});
	// chassis.moveToPoint({98_in, 21_in}, 1200, {.reverse=true});
	// clamp.retract();
	// intake.stop();
	// pros::delay(50);

	// chassis.turnAbsolute(-120_deg, 1000, {.gains={0.75, 0, 2}, .turnWheel=TurnWheel::RIGHT});
	// chassis.moveToPoint({100_in, 50_in}, 1400, {.maxRPM=400_rpm, .reverse=true});
	// clamp.extend();
	// intake.mogo();
	// pros::delay(50);

	// chassis.turnAbsolute(-10_deg, 1100, {.gains={0.4, 0, 2}});
	// chassis.moveToPoint({122_in, 48_in}, 0, {});
	// pros::delay(300);
	// chassis.moveToPoint({74_in, 55_in}, 0, {.minRPM=400_rpm, .reverse=true});
}

void redGoalSide() {
	odometry.update({135_in, 21.5_in, 90_deg});
	inertial.set_rotation(90_deg);
	intake.setTargetRing(RingColour::RED);

	//grab mogo from middle
	doinker.extend();
	claw.extend();
	chassis.moveToPoint({132_in, 55_in}, 400, {.minRPM=600_rpm, .earlyExitRadius=3_in});
	chassis.moveToPoint({126_in, 72_in}, 500, {.minRPM=200_rpm, .distGains={0.06, 0, 1.5}});
	// chassis.moveToPose({126_in, 66_in, 110_deg}, 500, {.distGains={0.1, 0, 1.5}, .dlead=4_in, .glead=0, .gRadius=5_in});
	claw.retract();
	pros::delay(50);
	chassis.moveToPoint({129_in, 45_in}, 1000, {.minRPM=600_rpm, .distGains={0.15, 0, 1.5}, .earlyExitRadius=5_in, .reverse=true});
	pros::Task([&] {
		pros::delay(600);
		claw.extend();
	});
	chassis.moveToPoint({122_in, 30_in}, 700, {.distGains={0.2, 0, 1.5}, .headingGains={2, 0, 1}, .reverse=true});

	//Release middle mogo
	pros::Task([&] {
		pros::delay(450);
		doinker.retract();
		claw.retract();
	});
	chassis.turnAbsolute(250_deg, 1000, {.gains={0.32, 0, 2}, .dir=TurnDirection::CCW});

	//grab middle mogo
	pros::Task([] {
		while (odometry.getPose().distTo(Pose(128_in, 48_in)) > 1) pros::delay(100);
		clamp.extend();
	});
	chassis.moveToPoint({128_in, 46_in}, 1600, {.maxRPM=200_rpm, .distGains={0.15, 0, 1.5}, .reverse=true});
	clamp.extend();
	intake.mogo();//Score on middle mogo

	//face ring stack and release mogo
	chassis.moveToPoint({120_in, 40_in}, 750, {.distGains={0.15, 0, 1.5}});
	chassis.turnAbsolute(110_deg, 1000, {.gains={0.37, 0, 2}});
    clamp.retract();

	//grab ring and hold in intake
	intake.intake();
	chassis.moveToPoint({114_in, 56_in}, 1000, {});
	pros::delay(300);

	//grab second mogo
	chassis.turnAbsolute(0_deg, 1000, {.gains={0.48, 0, 2}, .errorMargin=2.5_deg});
	intake.stop();
	pros::Task([] {
		pros::delay(1100);
		clamp.extend();
	});
	chassis.moveToPoint({87_in, 52_in}, 1350, {.maxRPM=300_rpm, .distGains={0.15, 0, 1.5}, .reverse=true});
	clamp.extend();
	intake.mogo();
	pros::delay(800);

	chassis.turnAbsolute(230_deg, 1300, {.gains={0.34, 0, 2}});
	intakeRaiser.retract();

	// the two tile (with red on top) beside the red alliance stake
	chassis.moveToPoint({69_in, 33_in}, 1100, {.maxRPM=400_rpm, .distGains={0.12, 0, 1.5}});
	pros::delay(150);
	intakeRaiser.extend();

	//face tower
	chassis.turnAbsolute(90_deg, 1000, {.gains={0.34, 0, 2}});
	chassis.moveDistance(13_in, 90_deg, 0, {.maxRPM=300_rpm});
	// chassis.moveToPoint({87_in, 57_in}, 0, {.maxRPM=300_rpm, .distGains={0.15, 0, 1.5}, .reverse=true});
	// intake.hold();
	// chassis.moveDistance(-5_in, 270_deg, 0, {});
	// intake.intake();
	// clamp.retract();

    // chassis.turnAbsolute(-60_deg, 0, {.gains={0.7, 0, 2}, .dir=TurnDirection::CW}, false, true);
    // clamp.retract();

	// // move to corner
    // chassis.moveToPoint({132_in, 15_in}, 1000, {.maxRPM=400_rpm, .distGains={0.12, 0, 1.5}});
	// doinker.extend();
    // chassis.turnAbsolute(-30_deg, 0, {.gains={0.32, 0, 2}, .dir=TurnDirection::CCW}, false, true);
	// chassis.turnAbsolute(-90_deg, 0, {.gains={0.32, 0, 2}, .dir=TurnDirection::CW}, false, true);
	// doinker.retract();

	// // pto.retract();
	// // pros::delay(300);
	// // arm.allianceStake();
	// chassis.turnAbsolute(360_deg, 0, {.gains={0.6, 0, 2}});
	// chassis.moveToPoint({135_in, 12_in}, 0, {.distGains={0.12, 0, 1.5}});
	// chassis.moveDistance(5_in, 315_deg, 0, {});
	// // arm.defaultPos();
	// // pros::delay(200);
	// // chassis.moveDistance(-50_in, 270_deg, 0, {});
	// chassis.turnAbsolute(495_deg, 1000, {.gains={0.36, 0, 2}});
	// // intake.outtake();
	// // chassis.moveDistance(10_in, 170_deg, 0, {.maxRPM=200_rpm});
	// chassis.moveToPoint({80_in, 40_in}, 0, {.distGains={0.12, 0, 1.5}});
	// clamp.retract();
}
void blueGoalSide() {
	odometry.update({12_in, 20_in, 67_deg});
	inertial.set_rotation(67_deg);
	intake.setTargetRing(RingColour::BLUE);

	intake.intake();
	doinker.extend();
	claw.extend();
	chassis.moveToPoint({22_in, 54_in}, 800, {.minRPM=600_rpm, .earlyExitRadius=4_in});
	chassis.moveToPoint({24_in, 64_in}, 350, {.minRPM=200_rpm, .distGains={0.06, 0, 1.5}, .headingGains={1.2, 0, 1}});
	claw.retract();

	chassis.moveToPoint({21_in, 40_in}, 1300, {.reverse=true});
	intake.hold();
	claw.extend();
	pros::Task([&] {
		pros::delay(400);
		doinker.retract();
		claw.retract();
	});
	chassis.turnAbsolute(-70_deg, 1300, {.gains={0.35, 0, 2}});

	chassis.moveToPoint({22_in, 60_in}, 1150, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(200);
	intake.mogo();
	pros::delay(1500);
	// chassis.moveToPoint({22_in, 50_in}, 1000, {});

	clamp.retract();
	chassis.turnAbsolute(190_deg, 1150, {.gains={0.71, 0, 2}, .turnWheel=TurnWheel::LEFT});
	chassis.moveToPoint({49_in, 58_in}, 1300, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(50);

	chassis.turnAbsolute(-45_deg, 1100, {.gains={0.34, 0, 2}});
	intakeRaiser.retract();
	intake.mogo();
	chassis.moveDistance(25_in, -45_deg, 900, {.distGains={0.06, 0, 1.5}});
	pros::delay(500);
	intakeRaiser.extend();

	chassis.moveToPoint({58_in, 40_in}, 900, {.minRPM=200_rpm, .earlyExitRadius=2_in, .reverse=true});
	chassis.turnAbsolute(60_deg, 900, {.gains={0.35, 0, 2}});
	chassis.moveDistance(14_in, 60_deg, 700, {.minRPM=150_rpm, .distGains={0.06, 0, 1.5}, .headingGains={0, 0, 0}, .exitDist=2_in});
}

void redGoalCorner() {
	odometry.update({104_in, 19_in, 56_deg});
	inertial.set_rotation(56_deg);
	intake.setTargetRing(RingColour::RED);

	intake.hold();
	doinker.extend();
	claw.extend();
	chassis.moveToPoint({117_in, 44_in}, 750, {.minRPM=600_rpm, .earlyExitRadius=3_in});
	chassis.moveToPoint({123_in, 61_in}, 350, {.minRPM=200_rpm, .headingGains={0.7, 0, 0.01}});
	claw.retract();

	chassis.moveToPoint({105_in, 30_in}, 1300, {.reverse=true});
	claw.extend();
	pros::Task([&] {
		pros::delay(400);
		doinker.retract();
		claw.retract();
	});
	chassis.turnAbsolute(235_deg, 1150, {});
	
	chassis.moveToPoint({120_in, 58_in}, 1250, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	intake.mogo();
	pros::delay(600);
	intake.intake();

	clamp.retract();
	intake.stop();
	chassis.turnAbsolute(-20_deg, 1100, {.gains={0.8, 0.2, 0.06}, .turnWheel=TurnWheel::RIGHT});
	chassis.moveToPoint({95_in, 54_in}, 1150, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	intake.mogo();
	pros::delay(50);

	chassis.turnAbsolute(225_deg, 1150, {.gains={1.2, 0.2, 0.1}});
	intakeRaiser.retract();
	intake.mogo();
	chassis.moveDistance(23_in, 225_deg, 850, {});
	// chassis.moveToPoint({80_in, 39_in}, 0, {.maxRPM=300_rpm, .distGains={0.12, 0, 1.5}});
	pros::delay(300);
	intakeRaiser.extend();

	// pros::Task([&] {
	// 	pros::delay(800);
	// 	intake.stop();
	// 	pto.retract();
	// 	pros::delay(400);
	// 	arm.allianceStake();
	// });	
	chassis.moveToPoint({130_in, 33_in}, 1150, {.minRPM=200_rpm, .earlyExitRadius=2_in, .reverse=true});
	doinker.extend();
	claw.extend();
	// chassis.moveToPoint({138_in, 40_in}, 1150, {.reverse=true});
	chassis.turnAbsolute(-45_deg, 1100, {.gains={1.2, 0.2, 0.1}, .turnWheel=TurnWheel::LEFT});
	chassis.moveToPoint({145_in, 23_in}, 1150, {});
	chassis.turnAbsolute(230_deg, 1100, {.maxRPM=450_rpm, .gains={1.3, 0.2, 0.1}, .turnWheel=TurnWheel::RIGHT});
	chassis.turnAbsolute(135_deg, 1100, {.gains={1.2, 0.2, 0.1}});
	clamp.retract();
	doinker.retract();
	claw.retract();
	chassis.moveToPoint({100_in, 70_in}, 1150, {});
	// chassis.moveDistance(0_in, 135_deg, 700, {});
	// chassis.moveTank(10000, 10000);

	// pros::delay(400);
	// pto.extend();
	// pros::delay(75);
	// intake.intake();
	// pros::delay(75);
	// intake.stop();
	// pros::delay(150);
	// intake.intake();
	// pros::delay(200);
	// // chassis.moveToPoint({140_in, 30_in}, 1150, {.maxRPM=600_rpm, .reverse=true});
	// chassis.moveDistance(-3_in, -45_deg, 850, {});
	// chassis.moveTank(6000, 6000);
	// pros::delay(500);
	// chassis.moveTank(-12000, -12000);

	// pros::Task([&] {
	// 	pto.retract();
	// 	pros::delay(500);
	// 	arm.allianceStake();
	// });
	// pros::delay(100);
	// chassis.moveDistance(9.5_in, 270_deg, 750, {});
	// chassis.turnAbsolute(257_deg, 850, {.gains={1.2, 0.2, 0.1}, .errorMargin=1_deg, .angularVelThreshold=2_deg});
	// chassis.moveDistance(10_in, 257_deg, 850, {.distGains={0.09, 0, 0.011}});
	// arm.defaultPos();
	// pros::delay(200);

	// chassis.moveToPoint({90_in, 30_in}, 1000, {.minRPM=200_rpm, .earlyExitRadius=2_in, .reverse=true});
	// pto.extend();
	// chassis.turnAbsolute(121_deg, 900, {.gains={1.2, 0.2, 0.1}});
	// chassis.moveDistance(14_in, 121_deg, 700, {.minRPM=150_rpm, .distGains={0.09, 0, 0.011}, .headingGains={0, 0, 0}, .exitDist=2_in});
}

void skills() {

}