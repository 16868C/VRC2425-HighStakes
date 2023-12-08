#include "routes.hpp"
#include "robotconfig.hpp"
#include "16868C/util/util.hpp"

using namespace lib16868C;
void waitUntilButton(okapi::ControllerDigital btn = okapi::ControllerDigital::A) {
	while (!master.getDigital(btn)) pros::delay(10);
}

#ifdef ANSONBOT
void goalAWPBar() {
	// pros::Task heading([&]() {
	// 	while (true) {
	// 		std::cout << inertial.get_rotation() << "\n";
	// 		pros::delay(50);
	// 	}
	// });

	catapult.intake();
	// intake.moveVoltage(12000);
	intakeRaiser.extend();

	// pros::delay(300);
	// chassis.moveDistance(29_in, 600_rpm, {0.2, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	// chassis.turnAbsolute(-45_deg, 600_rpm, {0.028, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 0);
	rightWing.extend();
	chassis.moveDistance(18_in, 600_rpm, {0.22, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 800);
	rightWing.retract();
	chassis.turnAbsolute(-35_deg, 600_rpm, {0.025, 0, 1.2}, 2, 3, 5, TurnWheel::RIGHT, 0);

	chassis.moveDistance(10_in, 600_rpm, {0.2, 0, 10}, 1200, -35_deg, 300_rpm, {0.1, 0, 0.1}, 1200);
	chassis.moveDistance(-6_in, 600_rpm, {0.21, 0, 10}, 1200, -35_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	// chassis.moveDistance(-12_in, 600_rpm, {0.21, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	// chassis.turnAbsolute(90_deg, 600_rpm, {0.019, 0, 1.4}, 2, 3, 5, TurnWheel::BOTH, 0);
	// intake.moveVoltage(-12000);
	// intakeRaiser.retract();
	// chassis.moveDistance(-13_in, 600_rpm, {0.2, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1200);
	// chassis.moveDistance(5_in, 600_rpm, {0.21, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 0);

	chassis.turnAbsolute(55_deg, 600_rpm, {0.023, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 0);
	// leftWing.extend();
	intakeRaiser.extend();
	chassis.moveDistance(-48_in, 600_rpm, {0.2, 0, 10}, 1200, 55_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	// chassis.turnAbsolute(200_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 0);
}

void goalAWP() {
	catapult.intake();
	intake.moveVoltage(12000);
	intakeRaiser.extend();

	pros::delay(300);
	chassis.moveDistance(29_in, 600_rpm, {0.2, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(-45_deg, 600_rpm, {0.028, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 0);
	rightWing.extend();
	chassis.moveDistance(12_in, 600_rpm, {0.21, 0, 10}, 1200, -45_deg, 300_rpm, {0.1, 0, 0.1}, 600);
	rightWing.retract();
	chassis.turnAbsolute(-90_deg, 600_rpm, {0.025, 0, 1.2}, 2, 3, 5, TurnWheel::RIGHT, 0);

	chassis.moveDistance(18_in, 600_rpm, {0.2, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.moveDistance(-12_in, 600_rpm, {0.21, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(90_deg, 600_rpm, {0.02, 0, 1.6}, 2, 3, 5, TurnWheel::BOTH, 0);
	intake.moveVoltage(-12000);
	intakeRaiser.retract();
	pros::delay(300);
	chassis.moveDistance(-14_in, 600_rpm, {0.2, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.moveDistance(15_in, 600_rpm, {0.22, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 900);
	
	chassis.turnAbsolute(45_deg, 600_rpm, {0.028, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 0);
	intake.moveVoltage(12000);
	intakeRaiser.extend();
	chassis.moveDistance(-62_in, 600_rpm, {0.19, 0, 10}, 1200, 45_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(0_deg, 600_rpm, {0.028, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 0);
	chassis.moveDistance(36_in, 600_rpm, {0.2, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 0);
}

void goalRush() {
	catapult.intake();
	intakeRaiser.extend();
	intake.moveVoltage(12000);

	pros::Task wingOpen([&]() {
		pros::delay(700);
		rightWing.extend();
	});
	chassis.moveDistance(-47_in, 600_rpm, {0.2, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.turnAbsolute(100_deg, 600_rpm, {0.021, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	rightWing.retract();
	intake.moveVoltage(-12000);
	intakeRaiser.retract();
	chassis.moveDistance(-20_in, 600_rpm, {0.2, 0, 10}, 1200, 100_deg, 300_rpm, {0.1, 0, 0.1}, 1000);

	chassis.moveDistance(15_in, 600_rpm, {0.2, 0, 10}, 1200, 100_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.turnAbsolute(10_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1500);
	chassis.moveDistance(55_in, 600_rpm, {0.2, 0, 10}, 1200, 10_deg, 300_rpm, {0.1, 0, 0.1}, 1200);
	chassis.turnAbsolute(-125_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(30_in, 600_rpm, {0.2, 0, 10}, 1200, -125_deg, 300_rpm, {0.1, 0, 0.1}, 3000);
	chassis.moveDistance(-5_in, 600_rpm, {0.2, 0, 10}, 1200, -125_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
}

void matchloadAWPBar() {
	catapult.intake();
	// intakeRaiser.extend();
	// intake.moveVoltage(12000);

	chassis.moveDistance(14_in, 600_rpm, {0.21, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 800);
	chassis.turnAbsolute(45_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 800);
	// intake.moveVoltage(-12000);
	// intakeRaiser.retract();
	chassis.moveDistance(9.5_in, 600_rpm, {0.2, 0, 10}, 1200, 45_deg, 300_rpm, {0.1, 0, 0.1}, 1000);

	chassis.moveDistance(-5_in, 600_rpm, {0.22, 0, 10}, 1200, 45_deg, 300_rpm, {0.1, 0, 0.1}, 700);
	chassis.turnAbsolute(0_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 800);
	chassis.turnAbsolute(-130_deg, 600_rpm, {0.035, 0, 1.5}, 2, 3, 5, TurnWheel::BOTH, 1100);
	chassis.turnAbsolute(-180_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 1000);

	rightWing.extend();
	chassis.moveDistance(14.5_in, 600_rpm, {0.22, 0, 10}, 1200, -180_deg, 300_rpm, {0.1, 0, 0.1}, 1200);
	rightWing.retract();
	chassis.turnAbsolute(-215_deg, 600_rpm, {0.026, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 900);
	chassis.moveDistance(34_in, 600_rpm, {0.2, 0, 10}, 1200, -215_deg, 300_rpm, {0.1, 0, 0.1}, 1300);
	chassis.moveDistance(-15_in, 600_rpm, {0.22, 0, 10}, 1200, -223_deg, 300_rpm, {0.1, 0, 0.1}, 900);
	chassis.turnAbsolute(-45_deg, 600_rpm, {0.032, 0, 1.5}, 2, 3, 5, TurnWheel::BOTH, 1200);
	intakeRaiser.extend();
	chassis.moveDistance(-12_in, 200_rpm, {0.6, 0, 10}, 1200, -45_deg, 300_rpm, {0.1, 0, 0.1}, 800);
	intakeRaiser.retract();
}

void matchloadRush() {
	catapult.fire();
	intakeRaiser.extend();
	intake.moveVoltage(12000);

	chassis.moveDistance(-46_in, 600_rpm, {0.2, 0, 10}, 12000, 0_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(-150_deg, 300_rpm, {0.05, 0, 1.2}, 2, 3, 5, TurnWheel::RIGHT, 0);
	catapult.fire();
	pros::delay(500);

	chassis.turnAbsolute(20_deg, 600_rpm, {0.02, 0, 1.2}, 2, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(-10_in, 600_rpm, {0.2, 0, 10}, 12000, 20_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	// chassis.moveDistance(40_in, 600_rpm, {0.2, 0, 10}, 12000, 45_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	
	// intake.moveVoltage(0);
	// chassis.turnAbsolute(-135_deg, 600_rpm, {0.02, 0, 1.2}, 2, 3, 5, TurnWheel::LEFT, 0);
}

void skills() {
	static Pose pose {0_in, 0_in, 0_deg};

	// catapult.fire();
	
	// chassis.moveDistance(30_in, 600_rpm, {0.2, 0, 10}, 1200, -35_deg, 150_rpm, {0.07, 0, 10}, 1200);
	// chassis.turnAbsolute(-115_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 1000);

	chassis.moveDistance(30_in, 600_rpm, {0.2, 0, 10}, 1200, -35_deg, 150_rpm, {0.07, 0, 10}, 1100);
	chassis.moveDistance(-3_in, 600_rpm, {0.21, 0, 10}, 1200, -35_deg, 150_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(-105_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 1000);

	// pros::delay(1000);
	double startHeading = inertial.get_rotation();
	leftWing.extend();
	catapult.matchload();
	int c = 0;
	waitUntilButton();
	// while (catapult.getNumFired() < 35 || c < 28000) { pros::delay(50); c += 50; }
	catapult.intake();
	leftWing.retract();
	inertial.set_rotation(startHeading - 10);

	chassis.turnAbsolute(-170_deg, 600_rpm, {0.035, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(21_in, 600_rpm, {0.2, 0, 10}, 1200, -170_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(-210_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(10_in, 600_rpm, {0.2, 0, 10}, 1200, -210_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	pros::delay(200);
	inertial.set_rotation(-210);
	pros::delay(200);

	// rightWing.extend();
	// chassis.turnAbsolute(-125_deg, 400_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 0);
	chassis.moveDistance(-3_in, 600_rpm, {0.2, 0, 10}, 1200, -210_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(-120_deg, 600_rpm, {0.025, 0, 1.2}, 2, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(75_in, 600_rpm, {0.19, 0, 10}, 1200, -120_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	
	chassis.turnAbsolute(-70_deg, 600_rpm, {0.027, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 1000);
	intakeRaiser.retract();
	chassis.moveDistance(30_in, 600_rpm, {0.2, 0, 10}, 1200, -30_deg, 100_rpm, {0.1, 0, 10}, 1200);
	chassis.moveDistance(-13_in, 600_rpm, {0.2, 0, 10}, 1200, -30_deg, 150_rpm, {0.1, 0, 0.1}, 1000);
	chassis.moveDistance(15_in, 600_rpm, {0.2, 0, 10}, 1200, -30_deg, 300_rpm, {0.15, 0, 10}, 1200);
	rightWing.retract();
	chassis.moveDistance(-5_in, 600_rpm, {0.2, 0, 10}, 1200, -30_deg, 300_rpm, {0.1, 0, 10}, 0);

	chassis.turnAbsolute(55_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(40_in, 600_rpm, {0.2, 0, 10}, 1200, 55_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(-30_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(36_in, 600_rpm, {0.2, 0, 10}, 1200, -30_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(-120_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 0);
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(50_in, 600_rpm, {0.2, 0, 10}, 1200, -120_deg, 300_rpm, {0.1, 0, 0.1}, 0);

	// pose.x = (vertDist.get() * okapi::millimeter).convert(okapi::inch) * okapi::inch;
	// pose.y = (horiDist.get() * okapi::millimeter).convert(okapi::inch) * okapi::inch;
	// pose.theta = inertial.get_rotation() * okapi::degree;
	// okapi::QAngle ang = pose.angleTo({0_in, 0_in, 0_deg});
	// Turn to 0, 90, 180, or 270
	// pose.x = horizontalDist.get(); pose.y = verticalDist.get(); pose.theta = inertial.get_rotation();
	// QAngle ang = pose.angleTo({x_in, y_in, 0_deg});
	// chassis.turnAbsolute(ang, 600_rpm, {0.02, 0, 1}, 2, 3, 5, TurnWheel::BOTH, 0); --> may need to add 90 or 180 or 270 deg depending on where origin is
	// QLength dist = pose.distTo({x_in, y_in, 0_deg});
	// chassis.moveDistance(dist, 600_rpm, {0.2, 0, 10}, 1200, inertial.get_rotation(), 300_rpm, {0.1, 0, 0.1}, 0);
}

void skills2() {
	chassis.moveDistance(30_in, 600_rpm, {0.2, 0, 10}, 1200, -35_deg, 150_rpm, {0.07, 0, 10}, 1100);
	chassis.moveDistance(-3_in, 600_rpm, {0.21, 0, 10}, 1200, -35_deg, 150_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(-105_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 1000);

	// pros::delay(1000);
	double startHeading = inertial.get_rotation();
	// leftWing.extend();
	catapult.matchload();
	int c = 0;
	// waitUntilButton();
	while (catapult.getNumFired() < 35 || c < 28000) { pros::delay(50); c += 50; }
	catapult.intake();
	// leftWing.retract();
	inertial.set_rotation(startHeading - 10);

	chassis.turnAbsolute(-160_deg, 600_rpm, {0.035, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(22_in, 600_rpm, {0.2, 0, 10}, 1200, -160_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(-210_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(10_in, 600_rpm, {0.2, 0, 10}, 1200, -210_deg, 0_rpm, {0.1, 0, 0.1}, 1000);
	chassis.moveTank(6000, 6000);
	pros::delay(200);
	inertial.set_rotation(0);
	pros::delay(200);

	chassis.moveDistance(-63_in, 600_rpm, {0.18, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	// chassis.turnAbsolute(90_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 0);
	// chassis.moveDistance(24_in, 600_rpm, {0.2, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	// chassis.moveDistance(-10_in, 600_rpm, {0.22, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(-90_deg, 600_rpm, {0.022, 0, 1.6}, 2, 3, 5, TurnWheel::BOTH, 0);
	chassis.moveDistance(-65_in, 600_rpm, {0.21, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 0);

	chassis.moveDistance(5_in, 600_rpm, {0.2, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(90_deg, 600_rpm, {0.021, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 0);
	// leftWing.extend();
	// rightWing.extend();
	chassis.moveDistance(30_in, 300_rpm, {0.3, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.moveDistance(-30_in, 600_rpm, {0.2, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1000);

	chassis.turnAbsolute(0_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(24_in, 600_rpm, {0.2, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(-135_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1200);
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(40_in, 600_rpm, {0.2, 0, 10}, 1200, -135_deg, 0_rpm, {0.1, 0, 0.1}, 1500);

	leftWing.retract();
	rightWing.retract();
	chassis.moveDistance(-30_in, 600_rpm, {0.2, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(0_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-30_in, 600_rpm, {0.2, 0, 10}, 1200, -180_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(-45_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(50_in, 600_rpm, {0.2, 0, 10}, 1200, -45_deg, 0_rpm, {0.1, 0, 0.1}, 0);
}

void skillsStart() {
	chassis.moveDistance(30_in, 600_rpm, {0.2, 0, 10}, 1200, -35_deg, 150_rpm, {0.07, 0, 10}, 1100);
	chassis.moveDistance(-3_in, 600_rpm, {0.21, 0, 10}, 1200, -35_deg, 150_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(-105_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 1000);
	// chassis.moveDistance(-12_in, 600_rpm, {0.2, 0, 10}, 1200, -70_deg, 350_rpm, {0.3, 0, 10}, 1500);

	leftWing.extend();
	catapult.matchload();
}
#endif