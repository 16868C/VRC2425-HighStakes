#include "routes.hpp"
#include "robotconfig.hpp"
#include "16868C/util/util.hpp"

using namespace lib16868C;
void waitUntilButton(okapi::ControllerDigital btn = okapi::ControllerDigital::A) {
	while (!master.getDigital(btn)) pros::delay(10);
}

#ifdef ANSONBOT
void goalAWPBar() {
	catapult.intake();
	intake.moveVoltage(12000);
	intakeRaiser.extend();

	pros::delay(300);
	chassis.moveDistance(29_in, 600_rpm, {0.2, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(-45_deg, 600_rpm, {0.028, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 0);
	rightWing.extend();
	chassis.moveDistance(12_in, 600_rpm, {0.21, 0, 10}, 1200, -45_deg, 300_rpm, {0.1, 0, 0.1}, 600);
	rightWing.retract();
	chassis.turnAbsolute(-90_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 0);

	chassis.moveDistance(18_in, 600_rpm, {0.2, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.moveDistance(-10_in, 600_rpm, {0.21, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(90_deg, 600_rpm, {0.025, 0, 1.5}, 2, 3, 5, TurnWheel::BOTH, 0);
	intake.moveVoltage(-12000);
	intakeRaiser.retract();
	chassis.moveDistance(-12_in, 600_rpm, {0.2, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.moveDistance(6_in, 600_rpm, {0.21, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 0);

	chassis.turnAbsolute(180_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 0);
	leftWing.extend();
	chassis.moveDistance(50_in, 600_rpm, {0.2, 0, 10}, 1200, 180_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(190_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 0);
}

void matchloadAWPBar() {
	catapult.intake();
	intakeRaiser.extend();
	intake.moveVoltage(12000);

	chassis.moveDistance(-12_in, 600_rpm, {0.21, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(45_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 0);
	intake.moveVoltage(-12000);
	intakeRaiser.retract();
	chassis.moveDistance(-24_in, 600_rpm, {0.2, 0, 10}, 1200, 45_deg, 300_rpm, {0.1, 0, 0.1}, 800);

	chassis.moveDistance(18_in, 600_rpm, {0.2, 0, 10}, 1200, 45_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(0_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 0);
	rightWing.extend();
	chassis.moveDistance(12_in, 600_rpm, {0.21, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 600);
	rightWing.retract();
	chassis.turnAbsolute(-45_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 0);

	leftWing.extend();
	chassis.moveDistance(35_in, 600_rpm, {0.2, 0, 10}, 1200, -45_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(-25_deg, 600_rpm, {0.025, 0, 1.25}, 2, 3, 5, TurnWheel::RIGHT, 0);
}

void skills() {
	static Pose pose {0_in, 0_in, 0_deg};

	catapult.fire();
	
	chassis.moveDistance(30_in, 600_rpm, {0.2, 0, 10}, 1200, -30_deg, 150_rpm, {0.07, 0, 10}, 1200);
	chassis.turnAbsolute(-110_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 1000);
	catapult.matchload();
	int c = 0;
	while (catapult.getNumFired() < 44 || c < 27000) { pros::delay(50); c += 50; }
	catapult.intake();

	// Turn to 0, 90, 180, or 270
	// pose.x = horizontalDist.get(); pose.y = verticalDist.get(); pose.theta = inertial.get_rotation();
	// QAngle ang = pose.angleTo({x_in, y_in, 0_deg});
	// chassis.turnAbsolute(ang, 600_rpm, {0.02, 0, 1}, 2, 3, 5, TurnWheel::BOTH, 0); --> may need to add 90 or 180 or 270 deg depending on where origin is
	// QLength dist = pose.distTo({x_in, y_in, 0_deg});
	// chassis.moveDistance(dist, 600_rpm, {0.2, 0, 10}, 1200, inertial.get_rotation(), 300_rpm, {0.1, 0, 0.1}, 0);
}
#endif