#include "routes.hpp"
#include "16868C/subsystems/chassis/inline.hpp"
#include "robotconfig.hpp"

using namespace lib16868C;

void waitUntilButton(okapi::ControllerDigital btn = okapi::ControllerDigital::A) {
	while (!master.getDigital(btn)) pros::delay(10);
}

void redGoalStake() {
	odometry.init({84_in, 15_in, -136_deg});
	intake.setTargetRing(RingColour::RED);
	armEnc.setPosition(18);

	arm.allianceStake();
	pros::delay(500);
	
	pros::Task([] {
		pros::delay(300);
		arm.defaultPos();
		pros::delay(800);
		clamp.extend();
	});
	chassis.moveToPoint({98_in, 46.5_in}, 0, {.maxRPM=400_rpm, .distGains={0.07, 0, 0.004}, .headingGains={0.8, 0, 0.02}, .reverse=true});
	// clamp.extend();
	pros::delay(200);

	chassis.turnAbsolute(-5_deg, 0, {.gains={1.1, 0.2, 0.09}});
	intake.intake();
	chassis.moveToPoint({132_in, 48_in}, 0, {});

	chassis.turnAbsolute(-100_deg, 0, {});
	chassis.moveToPoint({125_in, 15_in}, 0, {});
	chassis.turnAbsolute(-35_deg, 0, {});
	chassis.moveDistance(15_in, -35_deg, 1000, {.minRPM=600_rpm, .headingGains={}});
	chassis.moveDistance(-12_in, -35_deg, 1000, {.maxRPM=300_rpm});
	intakeRaiser.retract();
	chassis.moveDistance(15_in, -35_deg, 1000, {.minRPM=600_rpm, .headingGains={}});
	
	intake.intake();
	// chassis.moveToPoint({125_in, 24_in}, 0, {.reverse=true});
	chassis.moveDistance(-20_in, -35_deg, 800, {});
	chassis.turnAbsolute(-180_deg, 0, {});
	odometry.update(true, true, true, true);
	std::cout << odometry.getPose().toStr() << "\n";
	chassis.moveToPoint({75_in, 25_in}, 0, {});
	intakeRaiser.extend();
	pros::delay(100);

	chassis.moveDistance(-10_in, -180_deg, 500, {});
	chassis.turnAbsolute(-270_deg, 0, {});
	chassis.moveDistance(15_in, -270_deg, 0, {});
	chassis.moveTank(4000, 4000);
}

void redRingStake(){
	odometry.init({60_in, 18_in, -58_deg});
	intake.setTargetRing(RingColour::RED);
	armEnc.setPosition(18);
	chassis.moveDistance(4_in, -58_deg, 500, {});
	arm.allianceStake();
	pros::delay(500);

	pros::Task([] {
		pros::delay(300);
		arm.defaultPos();
		pros::delay(800);
		clamp.extend();
	});
	chassis.moveToPoint({47_in, 46.5_in}, 0, {.maxRPM=300_rpm, .distGains={0.07, 0, 0.004}, .headingGains={0.8, 0, 0.02}, .reverse=true});
	pros::delay(200);
	chassis.turnAbsolute(110_deg, 0, {.gains={1.1, 0.2, 0.09}});
	
	chassis.moveDistance(10_in, 110_deg, 0, {});
	intake.intake();
	chassis.turnAbsolute(180_deg, 0, {.turnWheel=TurnWheel::RIGHT});
	chassis.moveDistance(10_in, 180_deg, 0, {});
	chassis.turnAbsolute(90_deg, 0, {.turnWheel=TurnWheel::RIGHT});
}