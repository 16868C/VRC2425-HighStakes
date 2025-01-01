#include "routes.hpp"
#include "16868C/subsystems/chassis/inline.hpp"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"

using namespace lib16868C;

void waitUntilButton(okapi::ControllerDigital btn = okapi::ControllerDigital::A) {
	while (!master.getDigital(btn)) pros::delay(10);
}

void redSoloAWP() {
	odometry.update({58_in, 20_in, -50_deg});
	inertial.set_rotation(-50_deg);

	pto.retract();
	pros::delay(500);
	arm.allianceStake();
	do {
		pros::delay(100);
	} while (arm.getError() > 0);
	chassis.moveDistance(7_in, -50_deg, 600, {.maxRPM=300_rpm, .distGains={0.4, 0, 1.5}});
	arm.defaultPos();
	pros::delay(500);
	pto.extend();
	pros::Task([&] {
		arm.move(0);
		pto.extend();
		intake.mogo();
		pros::delay(250);
		intake.stop();
		pros::delay(250);
		intake.mogo();
	});

	chassis.moveDistance(-25_in, -90_deg, 900, {});
	chassis.moveDistance(-10_in, -90_deg, 400, {.maxRPM=200_rpm, .distGains={0.4, 0, 1.5}});
	clamp.extend();

	intake.mogo();
	chassis.turnAbsolute(-170_deg, 1200, {});
	chassis.moveDistance(22_in, -170_deg, 750, {});

	chassis.turnAbsolute(-270_deg, 1100, {});
	chassis.moveDistance(8.5_in, -270_deg, 500, {.maxRPM=400_rpm});
	chassis.turnAbsolute(-360_deg, 850, {.maxRPM=400_rpm, .gains={1.2, 0, 1}, .turnWheel=TurnWheel::LEFT});
	chassis.moveDistance(12_in, -365_deg, 0, {.maxRPM=300_rpm, .distGains={0.45, 0, 1.5}});
	
	chassis.moveDistance(-5_in, -355_deg, 450, {});
	chassis.moveDistance(30_in, -450_deg, 1000, {});
	pros::delay(1500);
	intake.stop();
	pros::Task raiseArm([&] {
		pto.retract();
		pros::delay(500);
		arm.allianceStake();
	});
	clamp.retract();
	chassis.turnAbsolute(-315_deg, 1000, {});
	chassis.moveDistance(10_in, -315_deg, 500, {});
}

void redRingAWP() {
	odometry.update({30_in, 21.5_in, 90_deg});
	inertial.set_rotation(90_deg);
	
	intake.intake();
	chassis.moveToPose({27_in, 68_in, 150_deg}, 1500, {.minRPM=100_rpm, .distGains={0.041, 0, 1.5}, .headingGains={0.4, 0, 2}, .dlead=2_in, .glead=1, .slewRate=6000});
	chassis.turnAbsolute(180_deg, 1000, {.maxRPM=400_rpm, .gains={1.5, 0, 1}, .turnWheel=TurnWheel::RIGHT, .slewRate=6000});
	chassis.moveToPoint({10_in, 69_in}, 1300, {.maxRPM=400_rpm, .distGains={0.1, 0, 1.5}, .headingGains={}, .slewRate=6000});

	chassis.moveToPoint({44_in, 48_in}, 0, {.distGains={0.045, 0, 1.3}, .headingGains={0.4, 0, 2}, .earlyExitRadius=5_in, .reverse=true, .slewRate=6000});
	pros::delay(100);
	clamp.extend();
	pros::delay(200);
	intake.mogo();

	pros::delay(500);
	chassis.moveToPoint({15_in, 45_in}, 0, {.distGains={0.041, 0, 1.5}, .headingGains={0.35, 0, 2}, .slewRate=6000});
}
void blueRingAWP() {
	odometry.update({58_in, 20_in, -50_deg});
	inertial.set_rotation(-90_deg);

	// pto.retract();
	// pros::delay(4000);
	// arm.allianceStake();
	// uint t = pros::millis();
	// do {
	// 	pros::delay(100);
	// } while (arm.getError() > 0 && pros::millis() - t < 1000);
	// chassis.moveDistance(9_in, -130_deg, 600, {.maxRPM=300_rpm, .distGains={0.4, 0, 1.5}, .headingGains={0.2, 0, 1.5}});
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

	chassis.moveDistance(-8_in, -90_deg, 900, {.headingGains={0.9, 0, 0.8}});
	chassis.moveDistance(-15_in, -90_deg, 600, {.maxRPM=200_rpm, .distGains={0.4, 0, 1.5}});
	clamp.extend();

	intake.mogo();

	chassis.turnAbsolute(50_deg, 1250, {.gains={0.72, 0, 1.1}});
	chassis.moveDistance(15_in, 50_deg, 0, {.maxRPM=200_rpm, .headingGains={1.5, 0, 1}, .exitDist=2_in});
	chassis.moveDistance(28_in, 0_deg, 1800, {.maxRPM=150_rpm, .headingGains={0.8, 0, 1.3}});
	// chassis.moveDistance(3_in, -170_deg, 0, {.maxRPM=250_rpm});
	// chassis.turnAbsolute(-70_deg, 850, {.maxRPM=400_rpm, .gains={1.2, 0, 1}});
	chassis.moveDistance(-12_in, 10_deg, 0, {.maxRPM=300_rpm, .exitDist=2_in});
	chassis.moveDistance(-8_in, 50_deg, 0, {.headingGains={1.5, 0, 1}});
	chassis.turnAbsolute(-40_deg, 1300, {.gains={0.95, 0, 1.4}});
	chassis.moveDistance(16_in, -35_deg, 1100, {});
	chassis.turnAbsolute(-160_deg, 1500, {.gains={0.4, 0, 1.4}, .errorMargin=2_deg});
	intakeRaiser.retract();
	chassis.moveDistance(23_in, -160_deg, 0, {});
	chassis.moveDistance(15_in, -160_deg, 0, {.maxRPM=300_rpm});
	// pros::delay(200);
	// intakeRaiser.extend();
	pros::delay(800);

	// chassis.turnAbsolute(160_deg, 1100, {});
	chassis.moveDistance(-5_in, -150_deg, 0, {.maxRPM=400_rpm, .exitDist=2_in});
	intakeRaiser.extend();
	chassis.moveDistance(-10_in, -45_deg, 0, {.maxRPM=400_rpm, .headingGains={0.8, 0, 1}});
	pros::delay(800);
	// hang.extend();
	intake.stop();
	chassis.moveDistance(-23_in, -45_deg, 0, {.maxRPM=300_rpm});
}

void blueSoloAWP() {
	odometry.update({-58_in, 20_in, -130_deg});
	inertial.set_rotation(-130_deg);

	pto.retract();
	pros::delay(500);
	arm.allianceStake();
	do {
		pros::delay(100);
	} while (arm.getError() > 0);
	chassis.moveDistance(7.5_in, -130_deg, 600, {.maxRPM=200_rpm, .distGains={0.4, 0, 1.5}});
	arm.defaultPos();
	pros::delay(500);
	pto.extend();

	// chassis.moveToPoint({48_in, 45_in}, 0, {.distGains={0.045, 0, 1.5}, .headingGains={0.6, 0, 1}, .exitRadius=3_in, .reverse=true});
	chassis.moveDistance(-30_in, -90_deg, 900, {});
	chassis.moveDistance(-10_in, -90_deg, 400, {.maxRPM=200_rpm, .distGains={0.4, 0, 1.5}});
	clamp.extend();

	intake.mogo();
	chassis.turnAbsolute(5_deg, 1250, {});
	chassis.moveDistance(22_in, 5_deg, 750, {});

	chassis.turnAbsolute(90_deg, 1100, {});
	chassis.moveDistance(2_in, 90_deg, 500, {.maxRPM=400_rpm});
	chassis.turnAbsolute(180_deg, 900, {.maxRPM=400_rpm, .gains={1.2, 0, 1}, .turnWheel=TurnWheel::RIGHT});
	chassis.moveDistance(12_in, 185_deg, 0, {.maxRPM=300_rpm, .distGains={0.45, 0, 1.5}});
	
	chassis.moveDistance(-5_in, 175_deg, 450, {});
	chassis.moveDistance(18_in, 270_deg, 900, {});
	pros::delay(1500);
	intake.stop();
	pros::Task raiseArm([&] {
		pto.retract();
		pros::delay(500);
		arm.allianceStake();
	});
	clamp.retract();
	chassis.turnAbsolute(135_deg, 0, {.gains={1.1, 0, 1}});
	chassis.moveDistance(10_in, 135_deg, 0, {});
}

void redRush() {
	inertial.set_rotation(90_deg);

	pros::Task doink([&] {
		pros::delay(680);
		doinker.extend();
	});
	chassis.moveDistance(34_in, 100_deg, 0, {.distGains={0.37, 0, 1}, .headingGains={0.2, 0, 3}});
	chassis.moveDistance(-25_in, 90_deg, 0, {.exitDist=2_in});
	chassis.turnAbsolute(0_deg, 0, {.turnWheel=TurnWheel::RIGHT});
	doinker.retract();
}
void blueRush() {
	inertial.set_rotation(90_deg);

	pros::Task doink([&] {
		pros::delay(700);
		doinker.extend();
	});
	chassis.moveDistance(34_in, 95_deg, 0, {.distGains={0.38, 0, 1}, .headingGains={0.2, 0, 3}});
	chassis.moveDistance(-25_in, 90_deg, 0, {.exitDist=2_in, .slewRate=4000});
	pros::Task undoink([&] {
		pros::delay(500);
		doinker.retract();
	});
	chassis.turnAbsolute(270_deg, 0, {.turnWheel=TurnWheel::LEFT});
}

void redGoalAWP() {
	odometry.update({135.5_in, 21.5_in, 90_deg});
	inertial.set_rotation(90_deg);

	doinker.extend();
	claw.extend();
	chassis.moveToPose({128_in, 67_in, 110_deg}, 0, {.distGains={0.07, 0, 1.5}, .headingGains={0.42, 0, 2}, .dlead=6_in, .glead=1, .gRadius=5_in});
	claw.retract();
	pros::delay(100);

	intake.intake();
	pros::Task([&] {
		pros::delay(500);
		claw.extend();
		doinker.retract();
	});
	chassis.turnToPoint({115_in, 40_in}, 0, {.gains={0.39, 0, 2}, .slewRate=6000});
	chassis.moveToPose({110_in, 40_in, 270_deg}, 0, {.distGains={0.04, 0, 1.5}, .headingGains={0.4, 0, 2}, .dlead=4_in, .glead=1, .slewRate=6000});
}

void redSoloAWPSig() {
	odometry.update({-58_in, 20_in, -130_deg});
	inertial.set_rotation(-130_deg);

	pto.retract();
	pros::delay(400);
	arm.allianceStake();
	do {
		pros::delay(100);
	} while (arm.getError() > 0);
	chassis.moveDistance(7.5_in, -130_deg, 500, {.maxRPM=300_rpm, .distGains={0.4, 0, 1.5}});
	arm.defaultPos();
	pros::delay(200);
	pros::Task([&] {
		// arm.defaultPos();
		pros::delay(500);
		arm.move(0);
		pto.extend();
		intake.mogo();
		pros::delay(250);
		intake.stop();
		pros::delay(250);
		intake.mogo();
	});
	// pros::delay(300);
	chassis.moveDistance(-25_in, -90_deg, 700, {.headingGains={1.07, 0, 1}});
	chassis.moveDistance(-16_in, -90_deg, 800, {.maxRPM=150_rpm, .distGains={0.8, 0, 1.5}, .headingGains={0.8, 0, 1}});
	clamp.extend();
	// pros::delay(100);
	intake.mogo();
	chassis.turnAbsolute(7_deg, 1100, {.gains={0.9, 0, 1.5}});
	chassis.moveDistance(9_in, 7_deg, 500, {});
	pros::delay(500);
	chassis.turnAbsolute(-157_deg, 1300, {.gains={0.9, 0, 1.6}});

	intakeRaiser.retract();
	// chassis.turnAbsolute(-137_deg, 1250, {.gains={0.87, 0, 1.1}});
	chassis.moveDistance(28_in, -157_deg, 850, {.slewRate=2000});
	chassis.moveDistance(7_in, -157_deg, 250, {.maxRPM=300_rpm, .distGains={0.6, 0, 1.5}});
	pros::delay(400);
	intakeRaiser.extend();
	// chassis.moveDistance(-2_in, -150_deg, 750, {});
	// pros::delay(1200);
	// clamp.retract();
	chassis.turnAbsolute(-20_deg, 1300, {.gains={0.7, 0, 1.4}, .errorMargin=5_deg});
	// intake.outtake();
	chassis.moveDistance(-23_in, -15_deg, 800, {.headingGains={1.5, 0, 1}});
	clamp.retract();
	// chassis.moveDistance(-10_in, -75_deg, 0, {.headingGains={1, 0, 1.1}});
	chassis.turnAbsolute(-80_deg, 1100, {.gains={0.85, 0, 1}, .turnWheel=TurnWheel::LEFT});

	chassis.moveDistance(-12_in, -75_deg, 600, {.headingGains={1.2, 0, 1}});
	chassis.moveDistance(-8_in, -75_deg, 500, {.maxRPM=200_rpm, .distGains={0.4, 0, 1.5}});
	clamp.extend();
	pros::delay(100);
	// intake.mogo();
	chassis.turnAbsolute(-183_deg, 1100, {.gains={0.9, 0, 1.5}});
	chassis.moveDistance(12_in, -175_deg, 600, {});
	pros::delay(800);

	hang.extend();
	chassis.moveDistance(-25_in, -150_deg, 0, {.headingGains={1.1, 0, 1}});
	pros::delay(700);
	clamp.retract();
	chassis.moveDistance(-5_in, -140_deg, 0, {.maxRPM=200_rpm});
}

void blueSoloAWPSig() {
	odometry.update({-58_in, 20_in, -50_deg});
	inertial.set_rotation(-50_deg);

	pto.retract();
	pros::delay(500);
	arm.allianceStake();
	do {
		pros::delay(100);
	} while (arm.getError() > 0);
	chassis.moveDistance(7.5_in, -50_deg, 0, {.maxRPM=200_rpm, .distGains={0.4, 0, 1.5}});
	arm.defaultPos();
	pros::delay(800);
	pros::Task([&] {
		arm.move(0);
		pto.extend();
		intake.mogo();
		pros::delay(250);
		intake.stop();
		pros::delay(250);
		intake.mogo();
	});

	chassis.moveDistance(-18_in, -90_deg, 0, {.headingGains={1.2, 0, 1}});
	chassis.moveDistance(-11.5_in, -90_deg, 0, {.maxRPM=150_rpm, .distGains={0.7, 0, 1.5}, .headingGains={1, 0, 1}});
	clamp.extend();
	pros::delay(200);
	intake.mogo();

	intakeRaiser.retract();
	chassis.turnAbsolute(-55_deg, 1250, {.gains={0.87, 0, 1.1}});
	chassis.moveDistance(14_in, -55_deg, 0, {.slewRate=2000});
	chassis.moveDistance(8.5_in, -55_deg, 0, {.maxRPM=200_rpm, .distGains={0.6, 0, 1.5}});
	pros::delay(500);
	intakeRaiser.extend();
	chassis.moveDistance(-2_in, -30_deg, 750, {});
	pros::delay(1200);
	clamp.retract();
	chassis.turnAbsolute(-160_deg, 1250, {.gains={0.7, 0, 1}});
	intake.outtake();
	chassis.moveDistance(-10_in, -160_deg, 0, {.minRPM=200_rpm, .headingGains={1, 0, 1}, .exitDist=2_in});
	chassis.moveDistance(-7_in, -140_deg, 0, {.headingGains={1, 0, 1.1}});

	chassis.moveDistance(-21_in, -120_deg, 0, {.maxRPM=200_rpm, .distGains={0.4, 0, 1.5}});
	clamp.extend();
	intake.mogo();
	chassis.turnAbsolute(-5_deg, 1250, {.gains={0.55, 0, 1.1}});
	chassis.moveDistance(10_in, -5_deg, 0, {});
	pros::delay(600);

	chassis.moveDistance(-19_in, -30_deg, 0, {.maxRPM=400_rpm, .headingGains={1.1, 0, 1}});
	pros::delay(500);
	clamp.retract();
	chassis.moveDistance(-5_in, -40_deg, 0, {.maxRPM=200_rpm});
}

void redRushSig() {
	inertial.set_rotation(90_deg);

	pros::Task doink([&] {
		pros::delay(680);
		doinker.extend();
	});
	chassis.moveDistance(34_in, 100_deg, 0, {.distGains={0.37, 0, 1}, .headingGains={0.2, 0, 3}});
	chassis.moveDistance(-25_in, 90_deg, 0, {.exitDist=2_in});
	chassis.turnAbsolute(-30_deg, 0, {.turnWheel=TurnWheel::RIGHT});
	doinker.retract();
	chassis.moveDistance(-25_in, -30_deg, 0, {});
	clamp.extend();
	intake.mogo();
	chassis.moveDistance(10_in, 0_deg, 0, {.maxRPM=200_rpm});
	
	pros::Task raiseIntake([&] {
		pros::delay(600);
		intakeRaiser.toggle();
	});
	chassis.turnAbsolute(-145_deg, 1100, {});
	chassis.moveDistance(25_in, -145_deg, 0, {.maxRPM=300_rpm});
	intakeRaiser.toggle();
	pros::delay(600);

	chassis.moveDistance(-15_in, -225_deg, 0, {});
	clamp.retract();
	pto.retract();
	pros::delay(500);
	arm.allianceStake();
	do {
		pros::delay(100);
		std::cout << arm.getError() << "\n";
	} while (arm.getError() > 0);
	chassis.moveDistance(10_in, -225_deg, 0, {});
}

void skills(){
	odometry.update({-58_in, 20_in, -50_deg});
	inertial.set_rotation(-90_deg);

	pto.retract();
	pros::delay(500);
	arm.allianceStake();
	do {
		pros::delay(100);
	} while (arm.getError() > 0);
	chassis.moveDistance(6_in, -90_deg, 0, {.maxRPM=200_rpm, .distGains={0.4, 0, 1.5}});
	arm.defaultPos();
	pros::delay(800);
	pros::Task([&] {
		arm.move(0);
		pto.extend();
		intake.mogo();
		pros::delay(250);
		intake.stop();
		pros::delay(250);
		intake.mogo();
	});
	chassis.moveDistance(-6_in, -90_deg, 0, {.maxRPM={400_rpm}, .headingGains={1, 0, 1}});
	chassis.turnAbsolute(-180_deg, 1100, {.gains={0.7, 0, 1.1}});
	chassis.moveDistance(-12_in, -180_deg, 0, {.maxRPM={400_rpm}, .headingGains={1, 0, 1}});
	chassis.moveDistance(-7_in, -180_deg, 0, {.maxRPM={200_rpm}, .headingGains={1, 0, 1}});
	clamp.extend();
	chassis.turnAbsolute(-270_deg, 1100, {.gains={0.7, 0, 1.1}});

	chassis.moveDistance(12_in, -270_deg, 0, {.maxRPM={400_rpm}, .headingGains={1, 0, 1}});
	pros::delay(400);
	chassis.turnAbsolute(-370_deg, 1100, {.gains={0.8, 0, 1.2}});
	chassis.moveDistance(16_in, -370_deg, 0, {.maxRPM={400_rpm}, .headingGains={1.1, 0, 1}});
	pros::delay(400);
	chassis.turnAbsolute(-480_deg, 1100, {.gains={0.8, 0, 1.2}});
	chassis.moveDistance(7_in, -480_deg, 0, {.maxRPM={350_rpm}, .headingGains={1, 0, 1}});
	chassis.moveDistance(25_in, -450_deg, 0, {.maxRPM={150_rpm}, .headingGains={1.1, 0, 1}});
	chassis.moveDistance(-7_in, -440_deg, 0, {.maxRPM={300_rpm}, .headingGains={1, 0, 1}});
	chassis.moveDistance(-20_in, -380_deg, 0, {.maxRPM={300_rpm}, .headingGains={0.7, 0, 1}});
	chassis.moveDistance(12_in, -395_deg, 0, {.maxRPM={200_rpm}, .headingGains={1, 0, 1}});
	pros::delay(400);
	chassis.turnAbsolute(-240_deg, 1100, {});
	chassis.moveDistance(-9_in, -240_deg, 0, {.maxRPM={500_rpm}, .headingGains={1, 0, 1}});
	clamp.retract();
	chassis.moveDistance(10_in, -180_deg, 0, {.maxRPM={400_rpm}, .headingGains={1, 0, 1}});
}