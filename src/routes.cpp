#include "routes.hpp"
#include "16868C/subsystems/chassis/inline.hpp"
#include "robotconfig.hpp"

using namespace lib16868C;

void waitUntilButton(okapi::ControllerDigital btn = okapi::ControllerDigital::A) {
	while (!master.getDigital(btn)) pros::delay(10);
}

void redGoalStake() {
	odometry.init({84_in, 15.5_in, -144_deg});
	intake.setTargetRing(RingColour::RED);
	armEnc.setPosition(18);

	arm.allianceStake();
	pros::delay(500);
	
	pros::Task([] {
		pros::delay(300);
		arm.defaultPos();
	});
	chassis.moveToPoint({98_in, 46_in}, 1900, {.maxRPM=300_rpm, .distGains={0.07, 0, 0.004}, .headingGains={0.8, 0, 0.02},.reverse=true});
	clamp.extend();
	pros::delay(100);

	chassis.turnAbsolute(10_deg, 850, {.gains={1.1, 0.2, 0.09}});
	intake.intake();
	chassis.moveToPoint({132_in, 52_in}, 1000, {});

	chassis.turnAbsolute(-90_deg, 650, {});
	chassis.moveToPoint({140_in, 4_in}, 1800, {});
	
	// chassis.moveToPoint({125_in, 24_in}, 0, {.reverse=true});
	chassis.moveDistance(-20_in, -90_deg, 1000, {.headingGains={0.4, 0, 0.01}});
	chassis.turnAbsolute(-90_deg, 750, {});
	// chassis.turnAbsolute(-180_deg, 0, {});
	odometry.update(true, true, false, false);
	std::cout << odometry.getPose().toStr() << "\n";
	chassis.turnAbsolute(-230_deg, 1100, {.gains={1.1, 0.1, 0.1}});
	// pros::Task([] {
	// 	pros::delay(800);
	// 	intake.stop();
	// });
	// chassis.moveToPoint({84_in, 72_in}, 1450, {});
	// chassis.turnAbsolute(-210_deg, 1250, {.gains={0.9, 0.1, 0.1}, .turnWheel=TurnWheel::RIGHT});
	// rightDoinker.extend();
	// pros::delay(250);
	// chassis.moveDistance(-25_in, -225_deg, 1200, {.maxRPM=300_rpm, .distGains={0.1, 0, 0.004}});
	// rightDoinker.retract();
	// intake.intake();
	// pros::delay(100);
	// chassis.turnAbsolute(-240_deg, 850, {});
	chassis.moveToPoint({92_in, 68_in}, 850, {});
	arm.move(4000);
	pros::delay(500);
	arm.move(0);

	// chassis.moveDistance(-10_in, -180_deg, 500, {});
	// chassis.turnAbsolute(-270_deg, 0, {});
	// chassis.moveDistance(15_in, -270_deg, 0, {});
	// chassis.moveTank(4000, 4000);
}
void blueGoalStake() {
	odometry.init({60_in, 15.5_in, -36_deg});
	intake.setTargetRing(RingColour::BLUE);
	armEnc.setPosition(18);

	arm.allianceStake();
	pros::delay(500);
	
	pros::Task([] {
		pros::delay(300);
		arm.defaultPos();
	});
	chassis.moveToPoint({46_in, 46_in}, 1900, {.maxRPM=300_rpm, .distGains={0.07, 0, 0.004}, .headingGains={0.8, 0, 0.02},.reverse=true});
	clamp.extend();
	pros::delay(100);

	chassis.turnAbsolute(170_deg, 850, {.gains={1.1, 0.2, 0.09}});
	intake.intake();
	chassis.moveToPoint({12_in, 52_in}, 1000, {});

	chassis.turnAbsolute(-115_deg, 600, {});
	chassis.moveToPoint({0_in, 4_in}, 1800, {.minRPM=600_rpm});
	
	// chassis.moveToPoint({125_in, 24_in}, 0, {.reverse=true});
	chassis.moveDistance(-20_in, -90_deg, 1000, {.headingGains={0.4, 0, 0.01}});
	// chassis.turnAbsolute(-90_deg, 750, {});
	// chassis.turnAbsolute(-180_deg, 0, {});
	// odometry.update(true, false, false, true);
	// std::cout << odometry.getPose().toStr() << "\n";
	chassis.turnAbsolute(50_deg, 1100, {.gains={1.1, 0.1, 0.1}});
	pros::Task([] {
		pros::delay(500);
		intake.stop();
	});
	// chassis.moveToPoint({60_in, 72_in}, 1450, {});
	// chassis.turnAbsolute(40_deg, 1250, {.gains={0.9, 0.1, 0.1}, .turnWheel=TurnWheel::LEFT});
	// leftDoinker.extend();
	// pros::delay(250);
	// chassis.moveDistance(-25_in, 45_deg, 1200, {.maxRPM=300_rpm, .distGains={0.1, 0, 0.004}});
	// leftDoinker.retract();
	// intake.intake();
	// pros::delay(100);
	// chassis.turnAbsolute(60_deg, 850, {});
	chassis.moveToPoint({52_in, 68_in}, 850, {});
	arm.move(4000);
	pros::delay(500);
	arm.move(0);
}

void redRingStake() {
	odometry.init({60_in, 15.5_in, -36_deg});
	intake.setTargetRing(RingColour::RED);
	armEnc.setPosition(18);

	arm.allianceStake();
	pros::delay(500);
	
	pros::Task([] {
		pros::delay(300);
		arm.defaultPos();
	});
	chassis.moveToPoint({46_in, 46_in}, 1900, {.maxRPM=300_rpm, .distGains={0.07, 0, 0.004}, .headingGains={0.8, 0, 0.02},.reverse=true});
	clamp.extend();
	//chassis.moveToPoint({46_in, 47_in}, 0, {.maxRPM=400_rpm, .distGains={0.07, 0, 0.004}, .headingGains={0.8, 0, 0.02}, .reverse=true});
	pros::delay(100);

	chassis.turnAbsolute(120_deg, 0, {.gains={1.1, 0.1, 0.11}});
	intake.intake();
	chassis.moveDistance(15.5_in, 120_deg, 0, {.minRPM=200_rpm, .distGains={0.0725, 0, 0.004}, .exitDist = 1.55_in});
	chassis.turnAbsolute(177_deg, 850, {.minRPM=100_rpm, .errorMargin=5_deg, .angularVelThreshold=10_deg, .turnWheel=TurnWheel::RIGHT});
	chassis.moveDistance(15_in, 182_deg, 700, {.minRPM=200_rpm, .distGains={0.0725, 0, 0.004}, .exitDist = 2_in});

	chassis.turnAbsolute(350_deg, 1200, {.gains={1.1, 0.2, 0.1}, .errorMargin=3_deg, .turnWheel=TurnWheel::RIGHT});
	chassis.moveToPoint({30_in, 38_in}, 850, {.minRPM=100_rpm, .distGains={0.07, 0, 0.004}, .earlyExitRadius=1_in});
	// chassis.moveDistance(17_in, 330_deg, 0, {.distGains={0.0725, 0, 0.004}});
	chassis.turnAbsolute(-135_deg, 650, {.gains={1.15, 0.1, 0.1}});
	chassis.moveToPoint({4_in, 4_in}, 1500, {});

	chassis.moveDistance(-20_in, -135_deg, 900, {});
	chassis.turnAbsolute(0_deg, 900, {.gains={1.1, 0.1, 0.1}});
	odometry.update(false, false, true, true);
	std::cout << odometry.getPose().toStr() << "\n";
	intakeRaiser.retract();
	chassis.moveToPoint({65_in, 24_in}, 1200, {});
	intakeRaiser.extend();

	chassis.turnAbsolute(160_deg, 1000, {.errorMargin=15_deg});
	pros::Task([] {
		pros::delay(500);
		arm.allianceStake();
	});
	chassis.moveToPoint({130_in, 15_in}, 0, {.reverse=true});
	chassis.turnAbsolute(-10_deg, 0, {});
	// rightDoinker.extend();

	// chassis.turnAbsolute(70_deg, 850, {.errorMargin=5_deg, .turnWheel=TurnWheel::LEFT});
	// pros::Task([] {
	// 	pros::delay(400);
	// 	arm.move(4000);
	// 	pros::delay(800);
	// 	arm.move(0);
	// });
	// chassis.moveDistance(19_in, 70_deg, 800, {});
}
void blueRingStake() {
	odometry.init({84_in, 15.5_in, -144_deg});
	intake.setTargetRing(RingColour::BLUE);
	armEnc.setPosition(18);

	arm.allianceStake();
	pros::delay(500);
	
	pros::Task([] {
		pros::delay(300);
		arm.defaultPos();
	});
	chassis.moveToPoint({98_in, 46_in}, 1900, {.maxRPM=300_rpm, .distGains={0.07, 0, 0.004}, .headingGains={0.8, 0, 0.02},.reverse=true});
	clamp.extend();
	//chassis.moveToPoint({46_in, 47_in}, 0, {.maxRPM=400_rpm, .distGains={0.07, 0, 0.004}, .headingGains={0.8, 0, 0.02}, .reverse=true});
	pros::delay(100);

	chassis.turnAbsolute(60_deg, 950, {.gains={1.1, 0.1, 0.11}});
	intake.intake();
	chassis.moveDistance(16_in, 60_deg, 650, {.minRPM=200_rpm, .distGains={0.0725, 0, 0.004}, .exitDist = 1.55_in});
	chassis.turnAbsolute(3_deg, 850, {.minRPM=100_rpm, .errorMargin=5_deg, .angularVelThreshold=10_deg, .turnWheel=TurnWheel::LEFT});
	chassis.moveDistance(14_in, -2_deg, 700, {.minRPM=200_rpm, .distGains={0.0725, 0, 0.004}, .exitDist = 2_in});

	chassis.turnAbsolute(-170_deg, 1200, {.gains={1.1, 0.2, 0.1}, .errorMargin=3_deg, .turnWheel=TurnWheel::LEFT});
	chassis.moveToPoint({114_in, 38_in}, 850, {.minRPM=100_rpm, .distGains={0.07, 0, 0.004}, .earlyExitRadius=1_in});
	// chassis.moveDistance(17_in, 330_deg, 0, {.distGains={0.0725, 0, 0.004}});
	chassis.turnAbsolute(-45_deg, 650, {.gains={1.15, 0.1, 0.1}});
	intake.hold();
	arm.allianceStake();
	chassis.moveToPoint({144_in, 4_in}, 1500, {});

	chassis.moveDistance(-20_in, -45_deg, 900, {});
	arm.defaultPos();
	chassis.turnAbsolute(180_deg, 900, {.gains={1.1, 0.1, 0.1}});
	intake.intake();
	odometry.update(false, true, true, false);
	std::cout << odometry.getPose().toStr() << "\n";
	intakeRaiser.retract();
	chassis.moveToPoint({79_in, 23_in}, 1200, {});
	intakeRaiser.extend();
	pros::delay(100);

	chassis.turnAbsolute(20_deg, 1000, {.errorMargin=15_deg});
	pros::Task([] {
		pros::delay(500);
		arm.allianceStake();
	});
	chassis.moveToPoint({14_in, 15_in}, 0, {.reverse=true});
	chassis.turnAbsolute(190_deg, 0, {});
	// leftDoinker.extend();
	// chassis.moveToPoint({4_in, 4_in}, 1500, {.maxRPM=400_rpm});
	// chassis.moveDistance(-10_in, -135_deg, 0, {});

	// chassis.turnAbsolute(110_deg, 850, {.errorMargin=5_deg, .turnWheel=TurnWheel::RIGHT});
	// pros::Task([] {
	// 	pros::delay(400);
	// 	arm.move(4000);
	// 	pros::delay(800);
	// 	arm.move(0);
	// });
	// chassis.moveDistance(19_in, 110_deg, 800, {});
}

void redGoalRush() {
	odometry.init({112_in, 22_in, 67_deg});
	intake.setTargetRing(RingColour::BLUE);

	leftDoinker.extend();
	pros::Task([] {
		pros::delay(100);
		intake.hold();
	});
	chassis.moveToPoint({124_in, 55_in}, 0, {.minRPM=600_rpm});
	pros::Task([] {
		pros::delay(800);
		intake.stop();
	});
	chassis.moveDistance(-20_in, 60_deg, 1200, {});
	// chassis.moveToPoint({120_in, 35_in}, 1200, {.distGains={0.9, 0, 0.004}, .settleRadius=12_in, .reverse=true});
	chassis.turnAbsolute(0_deg, 0, {});
	odometry.update(true, false, false, true);
	std::cout << odometry.getPose().toStr() << "\n";
	leftDoinker.retract();

	chassis.turnAbsolute(-65_deg, 700, {.gains={1.1, 0.1, 0.1}});

	chassis.moveToPoint({93_in, 37_in}, 1000, {.maxRPM=400_rpm, .distGains={0.064, 0, 0.004}, .headingGains={0.8, 0, 0.02}, .reverse=true});
	clamp.extend();
	pros::delay(100);
	
	intake.intake();
	chassis.turnAbsolute(-90_deg, 700, {});
	chassis.moveToPoint({90_in, 12_in}, 0, {.minRPM=200_rpm, .earlyExitRadius=2_in});
	chassis.moveToPoint({132_in, 0_in}, 1000, {.minRPM=400_rpm, .earlyExitRadius=1_in});
	chassis.moveToPoint({144_in, 0_in}, 700, {.maxRPM=400_rpm});
	// chassis.turnAbsolute(-135_deg, 0, {.errorMargin=5_deg, .turnWheel=TurnWheel::LEFT});
	// chassis.moveDistance(20_in, -135_deg, 1000, {.maxRPM=400_rpm, .headingGains={}});
	
	// chassis.moveToPoint({103_in, 10_in}, 0, {.reverse=true});
	chassis.moveDistance(-40_in, -10_deg, 1000, {});
	clamp.retract();
	chassis.turnAbsolute(-120_deg, 700, {.gains={1.2, 0.2, 0.09}, .turnWheel=TurnWheel::LEFT});
	chassis.moveToPoint({104_in, 39_in}, 0, {.maxRPM=400_rpm, .distGains={0.07, 0, 0.004}, .reverse=true});
	clamp.extend();
	pros::delay(200);

	chassis.turnAbsolute(170_deg, 0, {});
	intake.stop();
	chassis.moveToPoint({69_in, 60_in}, 0, {});
	rightDoinker.extend();
	pros::delay(200);
	chassis.moveDistance(-25_in, 150_deg, 1200, {.maxRPM=300_rpm, .distGains={0.1, 0, 0.004}});
	rightDoinker.retract();
	pros::delay(50);
	pros::Task([] {
		pros::delay(500);
		arm.move(4000);
		pros::delay(500);
		arm.move(0);
	});
	chassis.moveDistance(10_in, 120_deg, 1000, {});
}
void blueGoalRush() {
	odometry.init({32_in, 22_in, 113_deg});
	intake.setTargetRing(RingColour::BLUE);

	rightDoinker.extend();
	pros::Task([] {
		pros::delay(100);
		intake.hold();
	});
	chassis.moveToPoint({20_in, 55_in}, 0, {.minRPM=600_rpm});
	pros::Task([] {
		pros::delay(1000);
		intake.stop();
	});
	chassis.moveDistance(-20_in, 120_deg, 1200, {});
	// chassis.moveToPoint({120_in, 35_in}, 1200, {.distGains={0.9, 0, 0.004}, .settleRadius=12_in, .reverse=true});
	chassis.turnAbsolute(180_deg, 0, {});
	odometry.update(true, true, false, false);
	std::cout << odometry.getPose().toStr() << "\n";
	rightDoinker.retract();

	chassis.turnAbsolute(-112_deg, 700, {.gains={1.1, 0.1, 0.1}});

	chassis.moveToPoint({51_in, 37_in}, 1000, {.maxRPM=400_rpm, .distGains={0.064, 0, 0.004}, .headingGains={0.8, 0, 0.02}, .reverse=true});
	clamp.extend();
	pros::delay(100);
	
	intake.intake();
	chassis.turnAbsolute(-90_deg, 700, {});
	chassis.moveToPoint({54_in, 14_in}, 0, {.minRPM=200_rpm, .earlyExitRadius=2_in});
	chassis.moveToPoint({12_in, 0_in}, 1000, {.minRPM=400_rpm, .earlyExitRadius=1_in});
	chassis.moveToPoint({0_in, 0_in}, 700, {.maxRPM=400_rpm});
	// chassis.turnAbsolute(-135_deg, 0, {.errorMargin=5_deg, .turnWheel=TurnWheel::LEFT});
	// chassis.moveDistance(20_in, -135_deg, 1000, {.maxRPM=400_rpm, .headingGains={}});
	
	// chassis.moveToPoint({103_in, 10_in}, 0, {.reverse=true});
	chassis.moveDistance(-40_in, -180_deg, 1000, {});
	clamp.retract();
	chassis.turnAbsolute(-60_deg, 700, {.gains={1.2, 0.2, 0.09}, .turnWheel=TurnWheel::RIGHT});
	chassis.moveToPoint({38_in, 39_in}, 0, {.maxRPM=400_rpm, .distGains={0.07, 0, 0.004}, .reverse=true});
	clamp.extend();
	pros::delay(200);

	chassis.turnAbsolute(10_deg, 0, {});
	intake.stop();
	chassis.moveToPoint({69.5_in, 60_in}, 0, {});
	leftDoinker.extend();
	pros::delay(200);
	chassis.moveDistance(-25_in, 45_deg, 1200, {.maxRPM=300_rpm, .distGains={0.1, 0, 0.004}});
	leftDoinker.retract();
	pros::delay(50);
	chassis.turnAbsolute(60_deg, 0, {});
	pros::Task([] {
		pros::delay(500);
		arm.move(4000);
		pros::delay(500);
		arm.move(0);
	});
	chassis.moveToPoint({54_in, 58_in}, 0, {});
}

void redGoalRush2() {
	odometry.init({115_in, 24_in, 67_deg});
	intake.setTargetRing(RingColour::RED);
	// armEnc.setPosition(18);

	// arm.hold();
	leftDoinker.extend();
	pros::Task([] {
		pros::delay(100);
		intake.hold();
	});
	chassis.moveToPoint({126_in, 56_in}, 0, {.minRPM=600_rpm});
	chassis.moveDistance(-20_in, 60_deg, 1200, {});
	// chassis.moveToPoint({120_in, 35_in}, 1200, {.distGains={0.9, 0, 0.004}, .settleRadius=12_in, .reverse=true});
	chassis.turnAbsolute(0_deg, 0, {});
	odometry.update(true, false, false, true);
	std::cout << odometry.getPose().toStr() << "\n";
	leftDoinker.retract();

	chassis.turnAbsolute(-115_deg, 0, {});
	// chassis.turnToPoint({105_in, 40_in}, 0, {.reverse=true});

	// chassis.moveToPoint({105_in, 40_in}, 0, {.maxRPM=400_rpm, .headingGains={0.8, 0, 0.02}, .settleRadius=8_in, .reverse=true});
	chassis.moveDistance(-15_in, -115_deg, 0, {.distGains={0.075, 0, 0.004}});
	clamp.extend();
	pros::delay(100);
	
	intake.intake();
	pros::delay(500);
	clamp.retract();
	chassis.turnAbsolute(-40_deg, 0, {.turnWheel=TurnWheel::RIGHT});
	chassis.moveToPoint({95_in, 40_in}, 0, {.reverse=true});
	chassis.turnAbsolute(-100_deg, 0, {});
	chassis.moveToPoint({90_in, 14_in}, 0, {.minRPM=200_rpm, .earlyExitRadius=2_in});
	chassis.moveToPoint({120_in, 10_in}, 0, {.minRPM=300_rpm, .earlyExitRadius=5_in});
	chassis.turnAbsolute(-45_deg, 0, {.errorMargin=5_deg});
	chassis.moveDistance(15_in, -45_deg, 800, {.minRPM=600_rpm, .headingGains={}});
	chassis.moveDistance(-6_in, -45_deg, 600, {});
	intakeRaiser.retract();
	pros::Task([] {
		pros::delay(400);
		intakeRaiser.extend();
	});
	chassis.moveDistance(10_in, -45_deg, 1000, {.minRPM=600_rpm, .headingGains={}});
	
	// chassis.moveToPoint({103_in, 10_in}, 0, {.reverse=true});
	chassis.moveDistance(-15_in, 0_deg, 1000, {.maxRPM=200_rpm});

	chassis.turnAbsolute(-215_deg, 0, {});
	chassis.moveToPoint({80_in, 50_in}, 0, {});
	arm.move(4000);
	pros::delay(500);
	arm.move(0);
}