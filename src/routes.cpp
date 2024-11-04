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
	chassis.moveDistance(7_in, -50_deg, 600, {.maxRPM=200_rpm, .distGains={0.4, 0, 1.5}});
	arm.defaultPos();
	pros::delay(500);
	pto.extend();

	// chassis.moveToPoint({48_in, 45_in}, 0, {.distGains={0.045, 0, 1.5}, .headingGains={0.6, 0, 1}, .exitRadius=3_in, .reverse=true});
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