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
	pros::delay(300);
	pros::Task([&] {
		pros::delay(600);
		intake.hold();
		intakeFirst.moveVoltage(-12000);
	});
	chassis.moveDistance(-12_in, 0_deg, 1000, {.distGains={0.08, 0, 1.5}, .headingGains={0.45, 0, 1}});
	chassis.moveDistance(17_in, 0_deg, 800, {});
	chassis.turnAbsolute(90_deg, 900, {.gains={0.42, 0, 2}, .errorMargin=1_deg});
	pros::Task([&] {
		intake.outtake();
		pros::delay(100);
		intake.stop();
	});
	chassis.moveDistance(-8_in, 90_deg, 900, {.maxRPM=300_rpm});
	intake.mogo();
	pros::delay(500);
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
	intake.setTargetRing(RingColour::RED);
	
	intake.intake();
	chassis.moveToPoint({119_in, 61_in}, 850, {.minRPM=100_rpm, .earlyExitRadius=3_in});
	chassis.turnAbsolute(0_deg, 1000, {.maxRPM=400_rpm, .gains={1.1, 0, 1.5}, .turnWheel=TurnWheel::RIGHT});
	chassis.moveDistance(10_in, 0_deg, 700, {});

	pros::Task([&] {
		pros::delay(800);
		intake.hold();
	});
	chassis.moveToPoint({109.5_in, 48_in}, 1700, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(200);
	intake.mogo();

	chassis.turnAbsolute(5_deg, 850, {.gains={0.56, 0, 2}});
	chassis.moveToPoint({130_in, 50_in}, 1000, {});
	
	chassis.turnAbsolute(-90_deg, 950, {.gains={0.36, 0, 2}});
	chassis.moveToPoint({127_in, 35_in}, 700, {.minRPM=300_rpm, .earlyExitRadius=3_in});
	chassis.moveToPoint({109_in, 26_in}, 1050, {.headingGains={0.7, 0, 1}});
	pros::delay(1100);
	clamp.retract();
	intake.intake();

	chassis.turnAbsolute(145_deg, 750, {.gains={1.1, 0, 2}, .turnWheel=TurnWheel::RIGHT});
	intakeRaiser.retract();
	chassis.moveDistance(13_in, 145_deg, 750, {});
	intakeRaiser.extend();
	pros::delay(300);
	pros::Task([&] {
		pros::delay(600);
		intake.hold();
		intakeFirst.moveVoltage(-12000);
	});
	chassis.moveDistance(-12_in, 180_deg, 1000, {.distGains={0.08, 0, 1.5}, .headingGains={0.45, 0, 1}});
	chassis.moveDistance(17_in, 180_deg, 800, {});
	chassis.turnAbsolute(90_deg, 900, {.gains={0.42, 0, 2}, .errorMargin=1_deg});
	pros::Task([&] {
		intake.outtake();
		pros::delay(100);
		intake.stop();
	});
	chassis.moveDistance(-8_in, 90_deg, 900, {.maxRPM=300_rpm});
	intake.mogo();
	pros::delay(500);
	intake.outtake();
	pros::Task([&] {
		pros::delay(300);
		intake.mogo();
	});
	chassis.moveDistance(16_in, 90_deg, 800, {.maxRPM=400_rpm, .minRPM=200_rpm, .headingGains={0, 0, 0}, .exitDist=3_in});
}

void redGoalAWP() {
	odometry.update({105_in, 20_in, 56_deg});
	inertial.set_rotation(56_deg);
	intake.setTargetRing(RingColour::RED);

	intake.intake();
	doinker.extend();
	claw.extend();
	chassis.moveToPoint({120_in, 47_in}, 750, {.minRPM=600_rpm, .earlyExitRadius=3_in});
	chassis.moveToPoint({123_in, 60_in}, 350, {.minRPM=200_rpm, .distGains={0.06, 0, 1.5}});
	claw.retract();

	chassis.moveToPoint({123_in, 30_in}, 1300, {.reverse=true});
	intake.hold();
	claw.extend();
	pros::Task([&] {
		pros::delay(400);
		doinker.retract();
		claw.retract();
	});
	chassis.turnAbsolute(290_deg, 1150, {.gains={0.35, 0, 2}});
	
	chassis.moveToPoint({120_in, 60_in}, 1250, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(200);
	intake.mogo();
	pros::delay(500);

	clamp.retract();
	chassis.turnAbsolute(15_deg, 1100, {.gains={0.5, 0, 2}});
	chassis.moveToPoint({95_in, 52_in}, 1150, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(50);

	chassis.turnAbsolute(222_deg, 1150, {.gains={0.32, 0, 2}});
	intakeRaiser.retract();
	intake.mogo();
	chassis.moveDistance(23_in, 222_deg, 850, {.distGains={0.07, 0, 1.5}});
	// chassis.moveToPoint({80_in, 39_in}, 0, {.maxRPM=300_rpm, .distGains={0.12, 0, 1.5}});
	pros::delay(300);
	intakeRaiser.extend();

	chassis.turnAbsolute(270_deg, 1100, {.gains={0.76, 0, 2}, .turnWheel=TurnWheel::LEFT});
	intake.stop();
	pros::Task([&] {
		pto.retract();
		pros::delay(500);
		arm.allianceStake();
	});
	pros::delay(100);
	chassis.moveDistance(9.5_in, 270_deg, 750, {});
	chassis.turnAbsolute(254_deg, 850, {.gains={1.5, 0, 2}, .errorMargin=1_deg});
	chassis.moveDistance(7.5_in, 254_deg, 850, {.distGains={0.08, 0, 1.5}});
	arm.defaultPos();
	pros::delay(200);

	chassis.moveToPoint({90_in, 30_in}, 1000, {.minRPM=200_rpm, .earlyExitRadius=2_in, .reverse=true});
	pto.extend();
	chassis.turnAbsolute(121_deg, 900, {.gains={0.4, 0, 2}});
	chassis.moveDistance(14_in, 121_deg, 700, {.minRPM=150_rpm, .distGains={0.06, 0, 1.5}, .headingGains={0, 0, 0}, .exitDist=2_in});

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
	odometry.update({11.5_in, 24_in, 63_deg});
	inertial.set_rotation(63_deg);
	intake.setTargetRing(RingColour::BLUE);

	intake.intake();
	doinker.extend();
	claw.extend();
	chassis.moveToPoint({24_in, 55_in}, 800, {.minRPM=600_rpm, .earlyExitRadius=4_in});
	chassis.moveToPoint({25_in, 65_in}, 350, {.minRPM=200_rpm, .distGains={0.06, 0, 1.5}});
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
	chassis.turnAbsolute(-80_deg, 1300, {.gains={0.35, 0, 2}});

	chassis.moveToPoint({23_in, 60_in}, 1150, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(200);
	intake.mogo();
	pros::delay(400);

	clamp.retract();
	chassis.turnAbsolute(180_deg, 1150, {.gains={0.7, 0, 2}, .turnWheel=TurnWheel::LEFT});
	chassis.moveToPoint({49_in, 58_in}, 1300, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(50);

	chassis.turnAbsolute(-37_deg, 1000, {.gains={0.33, 0, 2}});
	intakeRaiser.retract();
	intake.mogo();
	chassis.moveDistance(25_in, -37_deg, 900, {.distGains={0.07, 0, 1.5}});
	pros::delay(500);
	intakeRaiser.extend();

	chassis.turnAbsolute(-90_deg, 1050, {.gains={0.76, 0, 2}, .turnWheel=TurnWheel::RIGHT});
	intake.stop();
	pros::Task([&] {
		pto.retract();
		pros::delay(500);
		arm.allianceStake();
	});
	pros::delay(100);
	chassis.moveDistance(10_in, -90_deg, 750, {});
	chassis.turnAbsolute(-50_deg, 850, {.gains={1.5, 0, 2}, .errorMargin=1_deg});
	chassis.moveDistance(8_in, -68_deg, 850, {.distGains={0.08, 0, 1.5}});
	arm.defaultPos();
	pros::delay(200);

	chassis.moveToPoint({58_in, 40_in}, 900, {.minRPM=200_rpm, .earlyExitRadius=2_in, .reverse=true});
	pto.extend();
	chassis.turnAbsolute(40_deg, 900, {.gains={0.38, 0, 2}});
	intake.mogo();
	chassis.moveDistance(14_in, 61_deg, 700, {.minRPM=150_rpm, .distGains={0.06, 0, 1.5}, .headingGains={0, 0, 0}, .exitDist=2_in});

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

void skills() {

}