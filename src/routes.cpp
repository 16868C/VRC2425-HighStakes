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
void blueGoalAWP() {

}

void skills() {

}