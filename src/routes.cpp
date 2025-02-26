#include "routes.hpp"
#include "16868C/subsystems/chassis/inline.hpp"
#include "robotconfig.hpp"

using namespace lib16868C;

void waitUntilButton(okapi::ControllerDigital btn = okapi::ControllerDigital::A) {
	while (!master.getDigital(btn)) pros::delay(10);
}

void redSoloAWP() {
	//initialize
	odometry.update({81_in, 21_in, -130_deg});
	inertial.set_rotation(-130_deg);
	intake.setTargetRing(RingColour::RED);

	pto.retract();
	pros::delay(300);
	arm.allianceStake();
	uint t = pros::millis();
	do {
		pros::delay(100);
		std::cout << arm.getError() << "\n";
	} while (arm.getError() > 0 && pros::millis() - t < 1000);
	chassis.moveDistance(8_in, -130_deg, 750, {.maxRPM=300_rpm, .distGains={0.15, 0, 0.011}, .headingGains={0.6, 0, 0.01}});
	arm.defaultPos();
	pros::delay(250);
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

	chassis.moveToPoint({93_in, 40_in}, 0, {.minRPM=300_rpm, .earlyExitRadius=2_in, .reverse=true});
	chassis.moveToPoint({93_in, 52_in}, 0, {.maxRPM=400_rpm, .headingGains={0.4, 0, 0.015}, .reverse=true});
	// chassis.moveDistance(-13_in, -90_deg, 0, {.maxRPM=400_rpm, .minRPM=100_rpm, .distGains={0.13, 0, 0.011}});
	clamp.extend();
	pros::delay(50);

	intake.mogo();
	chassis.turnAbsolute(-10_deg, 0, {.gains={1.2, 0.2, 0.1}});
	chassis.moveToPoint({114_in, 50_in}, 0, {});

	chassis.turnAbsolute(-147_deg, 0, {.gains={1.2, 0.2, 0.1}});
	intakeRaiser.retract();
	chassis.moveToPoint({78_in, 25_in}, 0, {});
	intakeRaiser.extend();
	pros::delay(100);

	chassis.turnAbsolute(0_deg, 0, {.gains={1.2, 0.2, 0.1}});
	chassis.moveToPoint({46_in, 21_in}, 0, {.reverse=true});
	clamp.retract();
	intake.stop();
	pros::delay(50);

	chassis.turnAbsolute(-70_deg, 0, {.gains={1.1, 0.1, 0.085}, .turnWheel=TurnWheel::LEFT});
	chassis.moveToPoint({47_in, 50_in}, 0, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	intake.mogo();
	pros::delay(50);

	chassis.turnAbsolute(-167_deg, 0, {.gains={1.2, 0.2, 0.1}});
	chassis.moveToPoint({24_in, 42_in}, 0, {});
	pros::delay(300);
	chassis.moveToPoint({55_in, 55_in}, 0, {.reverse=true});
}
void blueSoloAWP() {
	odometry.update({81_in, 21_in, -130_deg});
	inertial.set_rotation(-130_deg);
	intake.setTargetRing(RingColour::BLUE);

	pto.retract();
	pros::delay(300);
	arm.allianceStake();
	uint t = pros::millis();
	do {
		pros::delay(100);
		std::cout << arm.getError() << "\n";
	} while (arm.getError() > 0 && pros::millis() - t < 1000);
	chassis.moveDistance(8_in, -130_deg, 750, {.maxRPM=300_rpm, .distGains={0.15, 0, 0.011}, .headingGains={0.6, 0, 0.01}});
	arm.defaultPos();
	pros::delay(250);
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

	// chassis.moveToPoint({93_in, 40_in}, 0, {.minRPM=300_rpm, .earlyExitRadius=2_in, .reverse=true});
	// chassis.moveToPoint({93_in, 52_in}, 0, {.maxRPM=400_rpm, .headingGains={0.4, 0, 0.015}, .reverse=true});
	chassis.moveToPoint({95_in, 52_in}, 1750, {.maxRPM=400_rpm, .reverse=true}); // .headingGains={0.1, 0, 0.03}, 
	// chassis.moveDistance(-13_in, -90_deg, 0, {.maxRPM=400_rpm, .minRPM=100_rpm, .distGains={0.13, 0, 0.011}});
	clamp.extend();
	pros::delay(50);

	intake.mogo();
	chassis.turnAbsolute(0_deg, 750, {.gains={1.2, 0.2, 0.1}});
	chassis.moveToPoint({114_in, 50_in}, 1100, {});

	chassis.turnAbsolute(-147_deg, 1000, {.gains={1.2, 0.2, 0.1}});
	intakeRaiser.retract();
	chassis.moveToPoint({78_in, 25_in}, 1550, {});
	intakeRaiser.extend();
	pros::delay(100);

	chassis.turnAbsolute(0_deg, 0, {.gains={1.2, 0.2, 0.1}});
	chassis.moveToPoint({46_in, 21_in}, 0, {.reverse=true});
	clamp.retract();
	intake.stop();
	pros::delay(50);

	chassis.turnAbsolute(-70_deg, 0, {.gains={1.1, 0.1, 0.085}, .turnWheel=TurnWheel::LEFT});
	chassis.moveToPoint({47_in, 50_in}, 0, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	intake.mogo();
	pros::delay(50);

	chassis.turnAbsolute(-167_deg, 0, {.gains={1.2, 0.2, 0.1}});
	chassis.moveToPoint({24_in, 42_in}, 0, {});
	pros::delay(300);
	chassis.moveToPoint({55_in, 55_in}, 0, {.reverse=true});
}

void redGoalRush() {
	//initialize
	odometry.update({104_in, 19_in, 56_deg});
	inertial.set_rotation(56_deg);
	intake.setTargetRing(RingColour::RED);
	int st = pros::millis();

	//goal rush
	intake.intake();
	doinker.extend();
	claw.extend();
	chassis.moveToPoint({118_in, 44_in}, 750, {.minRPM=600_rpm, .earlyExitRadius=3_in});
	chassis.moveToPoint({124_in, 63_in}, 450, {.minRPM=200_rpm, .headingGains={0.7, 0, 0.01}});
	claw.retract();

	pros::Task([&] {
		pros::delay(900);
		claw.extend();
	});

	//move goal back and clamp
	chassis.moveToPoint({110_in, 38_in}, 1300, {.reverse=true});//30_in
	intake.stop();
	chassis.turnAbsolute(230_deg, 1150, {.dir=TurnDirection::CW});
	doinker.retract();
	claw.retract();
	chassis.moveToPoint({125_in, 60_in}, 1250, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();

	//ring intake 1
	intake.mogo();
	pros::delay(600);
	intake.intake();
	clamp.retract();
	intake.stop();

	//grab other mogo ring intake 2
	chassis.turnAbsolute(-15_deg, 1100, {.gains={1.1, 0.1, 0.08}, .turnWheel=TurnWheel::RIGHT});
	chassis.moveToPoint({95_in, 55_in}, 1150, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(50);
	intake.mogo();

	//grab stake ring intake
	// intakeRaiser.retract();
	// chassis.turnAbsolute(223_deg, 1150, {.gains={1.2, 0.2, 0.1}});
	// intake.mogo()
	// chassis.moveDistance(24_in, 225_deg, 850, {});
	// chassis.moveToPoint({81_in, 32_in}, 0, {});
	// pros::delay(300);
	// intakeRaiser.extend();

	//corner clear
	chassis.moveToPoint({128_in, 34_in}, 1300, {.minRPM=200_rpm, .earlyExitRadius=2_in});
	// chassis.moveToPoint({138_in, 40_in}, 1150, {.reverse=true});
	doinker.extend();
	claw.extend();
	chassis.turnAbsolute(-70_deg, 1100, {.gains={1.2, 0.1, 0.12}, .errorMargin=2_deg, .turnWheel=TurnWheel::RIGHT});
	chassis.moveToPoint({150_in, 20_in}, 900, {.minRPM=600_rpm});
	chassis.turnAbsolute(250_deg, 1200, {.minRPM=300_rpm, .gains={1.6, 0.1, 0.12}, .errorMargin=3_deg, .turnWheel=TurnWheel::RIGHT});
	doinker.retract();
	claw.retract();

	//ladder touch
	chassis.turnAbsolute(160_deg, 1500, {.gains={1.2, 0.2, 0.1}, .errorMargin=3_deg, .turnWheel=TurnWheel::LEFT});
	clamp.retract();
	intake.intake();
	std::cout << pros::millis()-st << "\n";
	chassis.moveToPoint({87_in, 63_in}, 0, {.maxRPM=400_rpm});

	// chassis.turnAbsolute(-90_deg, 1500, {.gains={1.25, 0.1, 0.12}, .turnWheel=TurnWheel::RIGHT});
	// chassis.moveToPoint({127_in, 60_in}, 1500, {.maxRPM=400_rpm, .reverse=true});
	// clamp.extend();
}
void blueGoalRush() {
	//initialize
	odometry.update({12_in, 20_in, 67_deg});
	inertial.set_rotation(67_deg);
	intake.setTargetRing(RingColour::BLUE);

	//goal rush
	intake.intake();
	doinker.extend();
	claw.extend();
	chassis.moveToPoint({21_in, 54_in}, 750, {.minRPM=600_rpm, .earlyExitRadius=3_in});
	chassis.moveToPoint({23_in, 67_in}, 450, {.minRPM=200_rpm, .headingGains={0.7, 0, 0.01}});
	claw.retract();

	pros::Task([&] {
		pros::delay(900);
		claw.extend();
	});

	//move goal back and clamp
	chassis.moveToPoint({21_in, 38_in}, 1300, {.reverse=true});//30_in
	intake.stop();
	chassis.turnAbsolute(-80_deg, 1150, {.dir=TurnDirection::CW});
	doinker.retract();
	claw.retract();
	chassis.moveToPoint({21_in, 63_in}, 1250, {.maxRPM=400_rpm, .reverse=true});

	//ring intake 1
	clamp.extend();
	intake.mogo();
	pros::delay(600);
	intake.intake();
	clamp.retract();
	intake.stop();

	//grab other mogo ring intake 2
	chassis.turnAbsolute(200_deg, 1100, {.gains={1.1, 0.1, 0.08}, .turnWheel=TurnWheel::LEFT});
	chassis.moveToPoint({50_in, 55_in}, 1150, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(50);
	intake.mogo();

	//grab stake ring intake
	// intakeRaiser.retract();
	// chassis.turnAbsolute(-35_deg, 1150, {.gains={1.2, 0.2, 0.1}});
	// intake.mogo();
	// chassis.moveDistance(24_in, 225_deg, 850, {});
	// chassis.moveToPoint({80_in, 34_in}, 0, {});
	// pros::delay(300);
	// intakeRaiser.extend();

	//corner clear
	chassis.turnAbsolute(-90_deg, 0, {.gains={1.2, 0.2, 0.1}});
	doinker.extend();
	claw.extend();
	chassis.moveToPoint({10_in, 10_in}, 1500, {.maxRPM=300_rpm, .minRPM=300_rpm});
	chassis.turnAbsolute(180_deg, 1500, {.minRPM=300_rpm, .gains={1.6, 0.1, 0.12}, .turnWheel=TurnWheel::LEFT});
	chassis.turnAbsolute(150_deg, 0, {.minRPM=300_rpm, .gains={1.6, 0.1, 0.12}, .errorMargin=3_deg, .turnWheel=TurnWheel::RIGHT});
	doinker.retract();
	claw.retract();
	chassis.turnAbsolute(90_deg, 0, {.gains={1.2, 0.2, 0.12}, .errorMargin=3_deg, .turnWheel=TurnWheel::LEFT});
	clamp.retract();
	intake.intake();

	//ladder touch
	// chassis.moveToPoint({52_in, 51_in}, 0, {.maxRPM=400_rpm});
	chassis.turnAbsolute(-90_deg, 1500, {.gains={0.8, 0.2, 0.06}});
	chassis.moveToPoint({21_in, 63_in}, 1500, {.reverse=true});
	clamp.extend();
}

void redGoalStake() {
	//initialize
	odometry.update({81_in, 21_in, -130_deg});
	inertial.set_rotation(-130_deg);
	intake.setTargetRing(RingColour::RED);

	//stake
	pto.retract();
	pros::delay(300);
	arm.allianceStake();
	uint t = pros::millis();
	do {
		pros::delay(100);
		std::cout << arm.getError() << "\n";
	} while (arm.getError() > 2 && pros::millis() - t < 800);
	chassis.moveDistance(9_in, -130_deg, 900, {.maxRPM=300_rpm, .distGains={0.15, 0, 0.011}, .headingGains={0.6, 0, 0.01}});
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

	// chassis.moveToPoint({93_in, 40_in}, 0, {.minRPM=300_rpm, .earlyExitRadius=2_in, .reverse=true});
	chassis.moveToPoint({95_in, 52_in}, 1750, {.maxRPM=400_rpm, .reverse=true}); // .headingGains={0.1, 0, 0.03}, 
	// chassis.moveDistance(-13_in, -90_deg, 0, {.maxRPM=400_rpm, .minRPM=100_rpm, .distGains={0.13, 0, 0.011}});
	clamp.extend();
	pros::delay(50);

	intake.mogo();
	chassis.turnAbsolute(0_deg, 750, {.gains={1.2, 0.2, 0.1}});
	chassis.moveToPoint({114_in, 50_in}, 1100, {});

	chassis.turnAbsolute(-147_deg, 1000, {.gains={1.2, 0.2, 0.1}});
	intakeRaiser.retract();
	chassis.moveToPoint({80_in, 25_in}, 1550, {});
	intakeRaiser.extend();
	pros::delay(100);

	chassis.moveToPoint({100_in, 35_in}, 0, {.reverse=true});
	chassis.turnAbsolute(135_deg, 0, {.gains={1.2, 0.2, 0.1}, .errorMargin=3_deg,});
	chassis.moveToPoint({87_in, 52_in}, 0, {.maxRPM=300_rpm});

	// chassis.moveToPoint({120_in, 34_in}, 1300, {.minRPM=200_rpm, .earlyExitRadius=2_in, .reverse=true});
	// // chassis.moveToPoint({138_in, 40_in}, 1150, {.reverse=true});
	// chassis.turnAbsolute(-80_deg, 1100, {.gains={1.4, 0.1, 0.12}, .turnWheel=TurnWheel::LEFT});
	// doinker.extend();
	// claw.extend();
	// chassis.moveToPoint({140_in, 23_in}, 1000, {.minRPM=600_rpm});
	// chassis.turnAbsolute(250_deg, 1200, {.minRPM=300_rpm, .gains={1.5, 0.1, 0.12}, .errorMargin=3_deg, .turnWheel=TurnWheel::RIGHT});
	// doinker.retract();
	// claw.retract();
	// chassis.turnAbsolute(145_deg, 1500, {.gains={1.2, 0.2, 0.1}, .errorMargin=3_deg, .turnWheel=TurnWheel::LEFT});
	// chassis.moveToPoint({87_in, 62_in}, 0, {});
}
void blueGoalStake() {
	odometry.update({63_in, 21_in, -50_deg});
	inertial.set_rotation(-50_deg);
	intake.setTargetRing(RingColour::BLUE);

	pto.retract();
	pros::delay(300);
	arm.allianceStake();
	uint t = pros::millis();
	do {
		pros::delay(100);
		std::cout << arm.getError() << "\n";
	} while (arm.getError() > 0 && pros::millis() - t < 1000);
	chassis.moveDistance(10_in, -50_deg, 750, {.maxRPM=300_rpm, .distGains={0.15, 0, 0.011}, .headingGains={0.6, 0, 0.01}});
	arm.defaultPos();
	pros::delay(250);
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

	// chassis.moveToPoint({93_in, 40_in}, 0, {.minRPM=300_rpm, .earlyExitRadius=2_in, .reverse=true});
	// chassis.moveToPoint({93_in, 52_in}, 0, {.maxRPM=400_rpm, .headingGains={0.4, 0, 0.015}, .reverse=true});
	chassis.moveToPoint({57_in, 63_in}, 1750, {.maxRPM=400_rpm, .reverse=true}); // .headingGains={0.1, 0, 0.03}, 
	// chassis.moveDistance(-13_in, -90_deg, 0, {.maxRPM=400_rpm, .minRPM=100_rpm, .distGains={0.13, 0, 0.011}});
	clamp.extend();
	pros::delay(50);

	intake.mogo();
	chassis.turnAbsolute(180_deg, 750, {.gains={1.2, 0.2, 0.1}});
	chassis.moveToPoint({33_in, 55_in}, 1100, {});

	chassis.turnAbsolute(-20_deg, 1000, {.gains={1.2, 0.2, 0.1}});
	intakeRaiser.retract();
	chassis.moveToPoint({75_in, 34_in}, 1550, {});
	intakeRaiser.extend();
	pros::delay(100);

	chassis.turnAbsolute(-160_deg, 0, {.gains={1.2, 0.2, 0.1}});
	doinker.extend();
	claw.extend();
	chassis.moveToPoint({0_in, -6_in}, 1800, {.maxRPM=400_rpm, .minRPM=400_rpm});
	chassis.turnAbsolute(150_deg, 0, {.minRPM=300_rpm, .gains={1.6, 0.1, 0.12}, .errorMargin=3_deg, .turnWheel=TurnWheel::RIGHT});
	doinker.retract();
	claw.retract();
	chassis.turnAbsolute(20_deg, 0, {.gains={1.2, 0.2, 0.12}, .errorMargin=3_deg, .turnWheel=TurnWheel::LEFT});
	clamp.retract();
	intake.intake();
	chassis.moveToPoint({71_in, 47_in}, 0, {.maxRPM=400_rpm});
}

void redRingRush(){
	odometry.update({30_in, 21.5_in, 90_deg});
	inertial.set_rotation(90_deg);
	intake.setTargetRing(RingColour::RED);
	
	intake.intake();
	chassis.moveToPoint({25_in, 63_in}, 950, {.minRPM=100_rpm, .earlyExitRadius=3_in});
	chassis.turnAbsolute(180_deg, 1000, {.maxRPM=600_rpm, .gains={1.8, 0.1, 0.07}, .turnWheel=TurnWheel::RIGHT});
	chassis.moveDistance(13_in, 180_deg, 800, {.distGains={0.1, 0, 0.011}});

	pros::Task([&] {
		pros::delay(800);
		intake.hold();
	});
	chassis.moveToPoint({34.5_in, 48_in}, 1800, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(200);
	intake.mogo();

	chassis.turnAbsolute(175_deg, 850, {.gains={1.2, 0.2, 0.1}});
	chassis.moveToPoint({13_in, 49_in}, 1100, {});
	
	chassis.turnAbsolute(300_deg, 900, {.gains={1.2, 0.2, 0.1}});
	chassis.moveToPoint({35_in, 32_in}, 1350, {});
	chassis.turnAbsolute(0_deg, 100, {.gains={1.2, 0.2, 0.15}});
	intakeRaiser.retract();
	chassis.moveToPoint({53_in, 30_in}, 1050, {});
	intakeRaiser.extend();
	
	chassis.moveToPoint({30_in, 30_in}, 1050, {.reverse=true});
	chassis.turnAbsolute(65_deg, 1250, {.gains={1.2, 0.2, 0.15}});
	chassis.moveToPoint({48_in, 61_in}, 1300, {});
}
void blueRingRush(){
	
}
void redRingStake() {
	
}
void blueRingStake() {
	odometry.update({114_in, 21.5_in, 90_deg});
	inertial.set_rotation(90_deg);
	intake.setTargetRing(RingColour::BLUE);
	
	intake.intake();
	chassis.moveToPoint({110_in, 60.5_in}, 850, {.minRPM=100_rpm, .earlyExitRadius=3_in});
	chassis.turnAbsolute(0_deg, 1100, {.maxRPM=400_rpm, .gains={1.1, 0, 1.5}, .turnWheel=TurnWheel::LEFT});
	chassis.moveDistance(10_in, 0_deg, 700, {.distGains={0.1, 0, 0.011}});

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

void skills() {
	odometry.init({72_in, 10_in, 90_deg});
	intake.setTargetRing(RingColour::RED);

	intake.mogo();
	pros::delay(700);
	intake.outtake();
	chassis.moveDistance(14.5_in, 90_deg, 0, {});
	intake.stop();
	std::cout << odometry.getPose().toStr() << "\n";
	chassis.turnAbsolute(0_deg, 0, {.gains={0.8, 0.2, 0.06}});
	std::cout << odometry.getPose().toStr() << "\n";

	//chassis.moveToPoint({48_in, 27_in}, 0, {.minRPM=300_rpm, .earlyExitRadius=3_in, .reverse=true});
	chassis.moveDistance(-23_in, 0_deg, 0, {.maxRPM=400_rpm, .minRPM=200_rpm, .headingGains={0, 0, 0}, .exitDist=2.5_in});
	clamp.extend();
	chassis.moveDistance(-3_in, 0_deg, 0, {.headingGains={0, 0, 0}});
	//pros::delay(100);
	chassis.turnAbsolute(90_deg, 0, {.gains={0.8, 0.2, 0.06}});
	//intake.redirect();
	intake.mogo();
	chassis.moveDistance(24_in, 90_deg, 0, {});

	chassis.turnAbsolute(149_deg, 0, {.gains={1.25, 0.1, 0.12}, .turnWheel=TurnWheel::RIGHT});
	intake.mogo();
	chassis.moveDistance(34_in, 149_deg, 0, {});

	//chassis.turnAbsolute(180_deg, 0, {.gains={1.25, 0.1, 0.12}});
	chassis.moveDistance(-10_in, 149_deg, 0, {});

	// pros::delay(200);
	// intake.stop();
	// pto.retract();
	// arm.wallStake();
	chassis.turnAbsolute(-90_deg, 0, {.gains={1.25, 0.1, 0.12}});
	chassis.moveDistance(53_in, -90_deg, 0, {.maxRPM=230_rpm, .distGains{0.17, 0, 0.01}});
	pros::delay(200);
	chassis.turnAbsolute(180_deg, 0, {.gains={1.25, 0.1, 0.12}, .turnWheel=TurnWheel::RIGHT});
	chassis.moveDistance(12_in, 180_deg, 0, {});
	// pros::Task([&] {
	// 	pros::delay(500);
	// 	intake.stop();
	// });
	pros::delay(1200);
	chassis.turnAbsolute(70_deg, 0, {.gains={1.25, 0.1, 0.12}});
	//intake.stop();
	chassis.moveDistance(-18_in, 70_deg, 500, {});//can be tuned if no time currently using timeout 
	clamp.retract();

	chassis.moveDistance(10_in, 70_deg, 0, {});
	chassis.turnAbsolute(180_deg, 0, {});

	//2/4
	chassis.moveToPoint({95_in, 23_in},0, {.earlyExitRadius=3_in, .reverse=true});

	// chassis.moveDistance(-73_in, 180_deg, 0, {.maxRPM=450_rpm, .minRPM=200_rpm, .headingGains={0.05, 0.05, 0.01}, .exitDist=2.5_in});
	clamp.extend();
	chassis.moveDistance(-3_in, 180_deg, 0, {.headingGains={0, 0, 0}});
}