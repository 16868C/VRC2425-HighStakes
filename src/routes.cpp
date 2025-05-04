#include "routes.hpp"
#include "16868C/subsystems/chassis/inline.hpp"
#include "robotconfig.hpp"

using namespace lib16868C;

void waitUntilButton(okapi::ControllerDigital btn = okapi::ControllerDigital::A) {
	while (!master.getDigital(btn)) pros::delay(10);
}

void redGoalStake() {
	odometry.init({84_in, 15_in, -144_deg});
	intake.setTargetRing(RingColour::RED);
	armEnc.setPosition(18);

	arm.allianceStake();
	pros::delay(500);
	
	pros::Task([] {
		pros::delay(300);
		arm.defaultPos();
	});
	chassis.moveToPoint({99_in, 43.5_in}, 0, {.maxRPM=400_rpm, .distGains={0.07, 0, 0.004}, .headingGains={0.8, 0, 0.02}, .reverse=true});
	clamp.extend();
	pros::delay(100);

	chassis.turnAbsolute(10_deg, 0, {.gains={1.1, 0.2, 0.09}});
	intake.intake();
	chassis.moveToPoint({132_in, 52_in}, 0, {});

	chassis.turnAbsolute(-70_deg, 0, {});
	chassis.moveToPoint({130_in, 25_in}, 0, {.minRPM=300_rpm, .earlyExitRadius=2_in});
	chassis.turnAbsolute(-45_deg, 0, {.turnWheel=TurnWheel::RIGHT});
	chassis.moveDistance(15_in, -45_deg, 1000, {.minRPM=600_rpm, .headingGains={}});
	
	// chassis.moveToPoint({125_in, 24_in}, 0, {.reverse=true});
	chassis.moveDistance(-20_in, -45_deg, 1000, {});
	// chassis.turnAbsolute(-180_deg, 0, {});
	// odometry.update(true, true, true, true);
	// std::cout << odometry.getPose().toStr() << "\n";
	chassis.turnAbsolute(-225_deg, 0, {.gains={1.1, 0.1, 0.1}});
	chassis.moveToPoint({83_in, 72_in}, 0, {});
	rightDoinker.extend();
	pros::delay(250);
	chassis.moveDistance(-25_in, -225_deg, 1200, {.maxRPM=300_rpm, .distGains={0.1, 0, 0.004}});
	rightDoinker.retract();
	pros::delay(100);
	chassis.turnAbsolute(-240_deg, 0, {});
	chassis.moveToPoint({92_in, 68_in}, 0, {});
	arm.move(12000);
	pros::delay(1500);
	arm.move(0);

	// chassis.moveDistance(-10_in, -180_deg, 500, {});
	// chassis.turnAbsolute(-270_deg, 0, {});
	// chassis.moveDistance(15_in, -270_deg, 0, {});
	// chassis.moveTank(4000, 4000);
}

void redRingStake(){
	odometry.init({60_in, 15.5_in, -36_deg});
	intake.setTargetRing(RingColour::RED);
	armEnc.setPosition(18);

	arm.allianceStake();
	pros::delay(500);
	
	pros::Task([] {
		pros::delay(300);
		arm.defaultPos();
	});
	chassis.moveToPoint({46_in, 47_in}, 0, {.maxRPM=300_rpm, .distGains={0.07, 0, 0.004}, .headingGains={0.8, 0, 0.02},.reverse=true});
	pros::delay(100);
	clamp.extend();
	//chassis.moveToPoint({46_in, 47_in}, 0, {.maxRPM=400_rpm, .distGains={0.07, 0, 0.004}, .headingGains={0.8, 0, 0.02}, .reverse=true});
	pros::delay(100);

	chassis.turnAbsolute(120_deg, 0, {.gains={1.1, 0.1, 0.1}});
	intake.intake();
	chassis.moveDistance(16_in, 120_deg, 0, {.distGains={0.0725, 0, 0.004}, .exitDist = 1.55_in});
	chassis.turnAbsolute(180_deg, 0, {.turnWheel=TurnWheel::RIGHT});
	chassis.moveDistance(18_in, 180_deg, 0, {.distGains={0.0725, 0, 0.004}, .exitDist = 2_in});

	chassis.turnAbsolute(330_deg, 0, {.turnWheel=TurnWheel::RIGHT});
	chassis.moveDistance(15_in, 330_deg, 0, {.distGains={0.0725, 0, 0.004}, .exitDist = 2_in});
	chassis.turnAbsolute(-45_deg, 0, {.gains={1.1, 0.1, 0.1}});
	chassis.moveToPoint({4_in, 4_in}, 2000, {.maxRPM=350_rpm});
	pros::delay(500);
	pros::Task([]{
		pros::delay(300);
		arm.allianceStake();
	});
	chassis.moveToPoint({20_in, 20_in}, 0, {.maxRPM=400_rpm, .reverse=true});

}

void redGoalRush() {
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

	chassis.turnAbsolute(-65_deg, 0, {});

	chassis.moveToPoint({93_in, 37_in}, 0, {.maxRPM=400_rpm, .headingGains={0.8, 0, 0.02}, .reverse=true});
	clamp.extend();
	pros::delay(100);
	
	intake.intake();
	chassis.turnAbsolute(-90_deg, 0, {});
	chassis.moveToPoint({90_in, 12_in}, 0, {.minRPM=200_rpm, .earlyExitRadius=2_in});
	chassis.moveToPoint({120_in, 5_in}, 2000, {.minRPM=400_rpm, .earlyExitRadius=5_in});
	chassis.moveDistance(20_in, -10_deg, 1000, {.minRPM=600_rpm, .headingGains={}});
	
	// chassis.moveToPoint({103_in, 10_in}, 0, {.reverse=true});
	chassis.moveDistance(-40_in, 0_deg, 0, {});
	clamp.retract();
	chassis.turnAbsolute(-90_deg, 0, {.gains={1.2, 0.2, 0.09}, .turnWheel=TurnWheel::LEFT});
	chassis.moveToPoint({106_in, 40_in}, 0, {.maxRPM=400_rpm, .reverse=true});
	clamp.extend();
	pros::delay(100);

	// chassis.turnAbsolute(-220_deg, 0, {.turnWheel=TurnWheel::LEFT});
	chassis.moveToPoint({74_in, 60_in}, 0, {});
	rightDoinker.extend();
	pros::delay(200);
	chassis.moveDistance(-25_in, -225_deg, 1200, {.maxRPM=300_rpm, .distGains={0.1, 0, 0.004}});
	rightDoinker.retract();
	pros::delay(50);
	chassis.turnAbsolute(-240_deg, 0, {});
	pros::Task([] {
		pros::delay(500);
		arm.move(12000);
		pros::delay(1500);
		arm.move(0);
	});
	chassis.moveToPoint({90_in, 58_in}, 0, {});
}