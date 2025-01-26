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
	
}

void redGoalAWP() {
	odometry.update({135_in, 21.5_in, 90_deg});
	inertial.set_rotation(90_deg);
	intake.setTargetRing(RingColour::RED);

	doinker.extend();
	claw.extend();
	chassis.moveToPoint({132_in, 55_in}, 400, {.minRPM=600_rpm, .earlyExitRadius=3_in});
	chassis.moveToPoint({127_in, 71_in}, 500, {.minRPM=200_rpm, .distGains={0.06, 0, 1.5}});
	// chassis.moveToPose({126_in, 66_in, 110_deg}, 500, {.distGains={0.1, 0, 1.5}, .dlead=4_in, .glead=0, .gRadius=5_in});
	claw.retract();
	pros::delay(50);
	chassis.moveToPoint({129_in, 45_in}, 1000, {.minRPM=600_rpm, .distGains={0.15, 0, 1.5}, .earlyExitRadius=5_in, .reverse=true});
	chassis.moveToPoint({122_in, 30_in}, 0, {.distGains={0.2, 0, 1.5}, .headingGains={2, 0, 1}, .reverse=true});
	claw.extend();
	pros::Task([&] {
		pros::delay(200);
		doinker.retract();
		claw.retract();
	});
	chassis.turnAbsolute(250_deg, 0, {.gains={0.32, 0, 2}, .dir=TurnDirection::CCW}, false, true);
	pros::Task([] {
		while (odometry.getPose().distTo(Pose(128_in, 48_in)) > 1) pros::delay(100);
		clamp.extend();
	});
	chassis.moveToPoint({128_in, 48_in}, 0, {.maxRPM=400_rpm, .distGains={0.15, 0, 1.5}, .reverse=true});
	clamp.extend();

	chassis.moveToPoint({120_in, 40_in}, 0, {.distGains={0.15, 0, 1.5}});
	chassis.turnAbsolute(110_deg, 0, {.gains={0.37, 0, 2}}, false, true);
	intake.mogo();

	chassis.moveToPoint({110_in, 60_in}, 0, {});
	pros::delay(1000);
	clamp.retract();
	chassis.turnAbsolute(30_deg, 1000, {.gains={0.48, 0, 2}, .errorMargin=2.5_deg}, false, true);
	pros::Task([] {
		pros::delay(1100);
		clamp.extend();
	});
	chassis.moveToPoint({87_in, 57_in}, 0, {.maxRPM=300_rpm, .distGains={0.15, 0, 1.5}, .reverse=true});
	
	chassis.turnAbsolute(210_deg, 0, {.gains={0.34, 0, 2}}, false, true);
	intakeRaiser.retract();
	intake.mogo();
	chassis.moveToPoint({68_in, 39_in}, 0, {.maxRPM=400_rpm, .distGains={0.12, 0, 1.5}});
	pros::delay(700);
	intakeRaiser.extend();
	intake.intake();
	clamp.retract();

	pto.retract();
	pros::delay(300);
	arm.allianceStake();
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
	odometry.update({35_in, 21.5_in, 90_deg});
	inertial.set_rotation(90_deg);
	intake.setTargetRing(RingColour::BLUE);

	doinker.extend();
	claw.extend();
	chassis.moveToPoint({30_in, 46_in}, 0, {.minRPM=600_rpm, .earlyExitRadius=3_in});
	chassis.moveToPoint({26_in, 72_in}, 550, {.minRPM=200_rpm, .distGains={0.06, 0, 1.5}, .headingGains={1.2, 0, 2}});
	claw.retract();
	pros::delay(50);
	pros::Task([] {
		pros::delay(1450);
		claw.extend();
	});
	chassis.moveToPoint({30_in, 43_in}, 1000, {.minRPM=600_rpm, .distGains={0.15, 0, 1.5}, .earlyExitRadius=5_in, .reverse=true});
	chassis.moveToPoint({27_in, 32_in}, 800, {.distGains={0.15, 0, 1.5}, .headingGains={2, 0, 1}, .reverse=true});

	chassis.turnAbsolute(-90_deg, 850, {.gains={0.39, 0, 2}});
	doinker.retract();
	claw.retract();
	pros::Task([] {
		pros::delay(1100);
		clamp.extend();
	});
	chassis.moveToPoint({25_in, 58_in}, 1200, {.maxRPM=400_rpm, .distGains={0.15, 0, 1.5}, .reverse=true});
	pros::delay(50);
	intake.mogo();

	chassis.moveToPoint({20_in, 40_in}, 800, {.distGains={0.15, 0, 1.5}});
	intake.outtake();
	chassis.turnAbsolute(80_deg, 850, {.gains={0.38, 0, 2}});
	clamp.retract();

	intake.intake();
	chassis.moveToPoint({18_in, 65_in}, 800, {});
	chassis.turnAbsolute(170_deg, 1000, {});
	intake.hold();
	intakeFirst.moveVoltage(-12000);
	pros::Task([] {
		pros::delay(900);
		clamp.extend();
	});
	chassis.moveToPoint({50_in, 55_in}, 1000, {.maxRPM=400_rpm, .distGains={0.15, 0, 1.5}, .reverse=true});
	pros::delay(50);
	intake.mogo();
	chassis.moveToPoint({44_in, 55_in}, 400, {.distGains={0.15, 0, 1.5}});

	// chassis.turnAbsolute(-50_deg, 1000, {.gains={0.4, 0, 2}});
	// intake.outtake();
	// clamp.retract();
	// intakeRaiser.retract();
	// pros::Task([] {
	// 	pros::delay(500);
	// 	intake.intake();
	// });
	// chassis.moveToPoint({61_in, 37_in}, 1000, {.maxRPM=400_rpm, .headingGains={0.8, 0, 2}});
	// chassis.turnAbsolute(-90_deg, 600, {.gains={0.5, 0, 2}});
	// intakeRaiser.extend();
	// chassis.turnAbsolute(20_deg, 600, {.gains={0.5, 0, 2}, .turnWheel=TurnWheel::RIGHT});
	// chassis.turnAbsolute(-20_deg, 600, {.gains={0.7, 0, 2}, .angularVelThreshold=20_deg, .turnWheel=TurnWheel::RIGHT});
	// intakeFirst.moveVoltage(-12000);
	// chassis.turnAbsolute(90_deg, 1000, {.gains={0.45, 0, 2}});
	// pros::Task([] {
	// 	pros::delay(500);
	// 	intake.mogo();
	// });
	// chassis.moveDistance(-20_in, 90_deg, 800, {.maxRPM=200_rpm});
	// chassis.moveDistance(40_in, 90_deg, 0, {.maxRPM=400_rpm});

	chassis.turnAbsolute(10_deg, 1000, {.gains={0.36, 0, 2}});
	intake.outtake();
	chassis.moveDistance(10_in, 10_deg, 0, {.maxRPM=200_rpm});
}

void skills() {

}