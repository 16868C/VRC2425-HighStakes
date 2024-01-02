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
	chassis.moveDistance(-8_in, 600_rpm, {0.21, 0, 10}, 1200, -35_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	// chassis.moveDistance(-12_in, 600_rpm, {0.21, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	// chassis.turnAbsolute(90_deg, 600_rpm, {0.019, 0, 1.4}, 2, 3, 5, TurnWheel::BOTH, 0);
	// intake.moveVoltage(-12000);
	// intakeRaiser.retract();
	// chassis.moveDistance(-13_in, 600_rpm, {0.2, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1200);
	// chassis.moveDistance(5_in, 600_rpm, {0.21, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 0);

	chassis.turnAbsolute(55_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	// leftWing.extend();
	intakeRaiser.extend();
	chassis.moveDistance(-48_in, 600_rpm, {0.2, 0, 10}, 1200, 55_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	// chassis.turnAbsolute(200_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 0);
}

void goalAWP() {
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
	chassis.moveDistance(-8_in, 600_rpm, {0.21, 0, 10}, 1200, -35_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	// chassis.moveDistance(-12_in, 600_rpm, {0.21, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	// chassis.turnAbsolute(90_deg, 600_rpm, {0.019, 0, 1.4}, 2, 3, 5, TurnWheel::BOTH, 0);
	// intake.moveVoltage(-12000);
	// intakeRaiser.retract();
	// chassis.moveDistance(-13_in, 600_rpm, {0.2, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1200);
	// chassis.moveDistance(5_in, 600_rpm, {0.21, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 0);

	chassis.turnAbsolute(55_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	// leftWing.extend();
	intakeRaiser.extend();
	chassis.moveDistance(-36_in, 600_rpm, {0.2, 0, 10}, 1200, 55_deg, 300_rpm, {0.05, 0, 0.1}, 0);
	chassis.turnAbsolute(135_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	intake.moveVoltage(12000);
	chassis.moveDistance(-25_in, 600_rpm, {0.2, 0, 10}, 1200, 145_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
	chassis.turnAbsolute(220_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 0);
	intake.moveVoltage(-12000);
	pros::delay(500);
	intakeRaiser.retract();
	chassis.moveDistance(-15_in, 600_rpm, {0.2, 0, 10}, 1200, 220_deg, 300_rpm, {0.05, 0, 0.1}, 1500);
}

void goalRush() {
	catapult.intake();
	intakeRaiser.extend();
	intake.moveVoltage(12000);

	pros::Task wingOpen([&]() {
		leftWing.extend();
		pros::delay(300);
		leftWing.retract();
		pros::delay(400);
		// rightWing.extend();
	});
	chassis.moveDistance(-47_in, 600_rpm, {0.2, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.moveDistance(8_in, 600_rpm, {0.2, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.turnAbsolute(100_deg, 600_rpm, {0.021, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	// rightWing.retract();
	intake.moveVoltage(-12000);
	pros::delay(500);
	// intakeRaiser.retract();
	// pros::delay(300);

	chassis.turnAbsolute(240_deg, 600_rpm, {0.019, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1700);
	intakeRaiser.extend();
	intake.moveVoltage(12000);
	chassis.moveDistance(-16_in, 600_rpm, {0.2, 0, 10}, 1200, 240_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.moveDistance(16_in, 600_rpm, {0.2, 0, 10}, 1200, 240_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(100_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	intake.moveVoltage(-12000);
	pros::delay(400);
	chassis.moveDistance(-20_in, 600_rpm, {0.2, 0, 10}, 1200, 100_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	
	intakeRaiser.retract();
	chassis.moveDistance(8_in, 600_rpm, {0.2, 0, 10}, 1200, 100_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.turnAbsolute(190_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-48_in, 600_rpm, {0.2, 0, 10}, 1200, 190_deg, 300_rpm, {0.1, 0, 0.1}, 2000);
	chassis.turnAbsolute(265_deg, 600_rpm, {0.021, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(17_in, 600_rpm, {0.2, 0, 10}, 1200, 265_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	rightWing.extend();
	chassis.turnAbsolute(200_deg, 600_rpm, {0.017, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 1000);
	rightWing.retract();
	chassis.turnAbsolute(210_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(20_in, 600_rpm, {0.2, 0, 10}, 1200, 210_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.moveDistance(-5_in, 600_rpm, {0.2, 0, 10}, 1200, 210_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	intakeRaiser.retract();
}

void matchloadAWPBar() {
	catapult.intake();
	// intakeRaiser.extend();
	// intake.moveVoltage(12000);

	chassis.moveDistance(-5_in, 600_rpm, {0.21, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 800);
	// chassis.turnAbsolute(45_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 800);
	// intake.moveVoltage(-12000);
	// intakeRaiser.retract();
	// chassis.moveDistance(9.5_in, 600_rpm, {0.2, 0, 10}, 1200, 45_deg, 300_rpm, {0.1, 0, 0.1}, 1000);

	// chassis.moveDistance(-5_in, 600_rpm, {0.22, 0, 10}, 1200, 45_deg, 300_rpm, {0.1, 0, 0.1}, 700);
	// chassis.turnAbsolute(0_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 800);
	// chassis.turnAbsolute(-130_deg, 600_rpm, {0.035, 0, 1.5}, 2, 3, 5, TurnWheel::BOTH, 1100);
	// chassis.turnAbsolute(-45_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 1000);

	rightWing.extend();
	chassis.moveDistance(6_in, 600_rpm, {0.22, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 1200);
	rightWing.retract();
	chassis.turnAbsolute(-30_deg, 600_rpm, {0.026, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 900);
	chassis.moveDistance(33.5_in, 600_rpm, {0.2, 0, 10}, 1200, -30_deg, 300_rpm, {0.1, 0, 0.1}, 1300);
	chassis.moveDistance(-10_in, 600_rpm, {0.22, 0, 10}, 1200, -30_deg, 300_rpm, {0.1, 0, 0.1}, 900);
	chassis.turnAbsolute(135_deg, 600_rpm, {0.032, 0, 1.5}, 2, 3, 5, TurnWheel::BOTH, 1200);
	intakeRaiser.extend();
	chassis.moveDistance(-9_in, 200_rpm, {0.6, 0, 10}, 1200, 135_deg, 300_rpm, {0.1, 0, 0.1}, 800);
	intakeRaiser.retract();
}

void matchloadRush() {
	catapult.intake();
	intakeRaiser.extend();
	intake.moveVoltage(12000);
	inertial.set_rotation(8);

	chassis.moveDistance(-48_in, 600_rpm, {0.2, 0, 10}, 12000, 8_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(-120_deg, 400_rpm, {0.03, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 2000);
	leftWing.extend();
	chassis.moveDistance(15_in, 600_rpm, {0.21, 0, 10}, 1200, -90_deg, 200_rpm, {0.05, 0, 0.1}, 1500);
	
	chassis.turnAbsolute(-140_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 1000);
	leftWing.retract();
	chassis.moveDistance(-55.5_in, 600_rpm, {0.19, 0, 10}, 1200, -140_deg, 300_rpm, {0.1, 0, 0.1}, 2000);
	chassis.turnAbsolute(-210_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	
	chassis.turnAbsolute(-270_deg, 600_rpm, {0.026, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 1500);
	pros::Task outtake([&]() {
		// pros::delay(500);
		intake.moveVoltage(-12000);
		intakeRaiser.retract();
	});
	chassis.moveDistance(-40_in, 600_rpm, {0.2, 0, 10}, 1200, -270_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
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
	// chassis.moveDistance(-3_in, 600_rpm, {0.21, 0, 10}, 1200, -35_deg, 150_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(-107_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 1000);
	pros::Task touchBar([&]() {
		chassis.moveTank(-2000, -2000);
		pros::delay(500);
		chassis.moveTank(0, 0);
	});
	chassis.moveTank(-2000, -2000);

	// pros::delay(1000);
	double startHeading = inertial.get_rotation();
	// leftWing.extend();
	catapult.matchload();
	int c = 0;
	while (catapult.getNumFired() < 45 || c < 28000) { pros::delay(50); c += 50; }
	std::cout << catapult.getNumFired() << "\n";
	// pros::delay(1000);
	catapult.intake();
	// leftWing.retract();
	inertial.set_rotation(startHeading - 5);

	chassis.turnAbsolute(-150_deg, 600_rpm, {0.035, 0, 2}, 2, 3, 5, TurnWheel::BOTH, 700);
	chassis.moveDistance(23_in, 600_rpm, {0.2, 0, 10}, 1200, -155_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	chassis.turnAbsolute(-210_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(10_in, 600_rpm, {0.2, 0, 10}, 1200, -210_deg, 0_rpm, {0.1, 0, 0.1}, 1000);
	chassis.moveTank(6000, 6000);
	pros::delay(500);
	inertial.set_rotation(0);
	pros::delay(200);

	chassis.moveDistance(-63_in, 600_rpm, {0.18, 0, 10}, 1200, 0_deg, 300_rpm, {0.1, 0, 0.1}, 1700);
	// chassis.turnAbsolute(90_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 0);
	// chassis.moveDistance(24_in, 600_rpm, {0.2, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	// chassis.moveDistance(-10_in, 600_rpm, {0.22, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 0);
	// chassis.turnAbsolute(90_deg, 600_rpm, {0.023, 0, 1.6}, 2, 3, 5, TurnWheel::BOTH, 1700);
	// chassis.moveDistance(33_in, 600_rpm, {0.2, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	// chassis.moveDistance(-10_in, 600_rpm, {0.2, 0, 10}, 1200, 90_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(-90_deg, 600_rpm, {0.022, 0, 1.6}, 2, 3, 5, TurnWheel::BOTH, 1500);
	chassis.moveDistance(-30_in, 600_rpm, {0.2, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.moveDistance(13_in, 600_rpm, {0.2, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 1000);

	intake.moveVoltage(-12000);
	intakeRaiser.extend();
	chassis.moveTank(-12000, -12000);
	int a = 0;
	while (a < 3) {
		if (inertial.get_roll() < -5 && a == 0) a++;
		else if (inertial.get_roll() > 5 && a == 1) a++;
		else if (std::abs(inertial.get_roll()) < 20 && a == 2) a++;
		
		pros::delay(100);
	}
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	chassis.moveTank(0, 0);
	chassis.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	// chassis.moveDistance(-65_in, 600_rpm, {0.21, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 0);

	// chassis.moveDistance(5.5_in, 600_rpm, {0.21, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 600);
	// chassis.turnAbsolute(90_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	// leftWing.extend();
	// rightWing.extend();
	// pros::Task dropIntake([&]() {
	// 	pros::delay(200);
	// 	intakeRaiser.extend();
	// 	intake.moveVoltage(-12000);
	// });
	chassis.moveDistance(-30_in, 450_rpm, {0.3, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 1300);
	intakeRaiser.retract();
	intake.moveVoltage(0);
	chassis.moveDistance(29_in, 600_rpm, {0.2, 0, 10}, 1200, -90_deg, 300_rpm, {0.1, 0, 0.1}, 1500);

	chassis.turnAbsolute(-180_deg, 600_rpm, {0.022, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-35_in, 600_rpm, {0.21, 0, 10}, 1200, -180_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.turnAbsolute(-220_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1400);
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(34_in, 600_rpm, {0.2, 0, 10}, 1200, -270_deg, 150_rpm, {0.05, 0, 10}, 1600);
	// chassis.turnAbsolute(90_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 1500);

	leftWing.retract();
	rightWing.retract();
	pros::delay(100);
	chassis.moveDistance(-30_in, 600_rpm, {0.21, 0, 10}, 1200, -270_deg, 300_rpm, {0.1, 0, 0.1}, 1500);
	chassis.turnAbsolute(-360_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	chassis.moveDistance(-42_in, 600_rpm, {0.2, 0, 10}, 1200, -360_deg, 300_rpm, {0.1, 0, 0.1}, 1200);
	chassis.turnAbsolute(-320_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(45_in, 600_rpm, {0.2, 0, 10}, 1200, -270_deg, 150_rpm, {0.5, 0, 10}, 1500);
	// chassis.turnAbsolute(90_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::RIGHT, 1500);
	leftWing.retract();
	rightWing.retract();
	pros::delay(100);

	chassis.moveDistance(-20_in, 600_rpm, {0.2, 0, 10}, 1200, -270_deg, 300_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(-300_deg, 600_rpm, {0.02, 0, 1.3}, 2, 3, 5, TurnWheel::BOTH, 1000);
	leftWing.extend();
	rightWing.extend();
	chassis.moveDistance(40_in, 600_rpm, {0.2, 0, 10}, 1200, -270_deg, 150_rpm, {0.5, 0, 10}, 1000);
	leftWing.retract();
	rightWing.retract();
	chassis.moveDistance(-5_in, 600_rpm, {0.2, 0, 10}, 1200, -270_deg, 300_rpm, {0.1, 0, 0.1}, 0);
}

void skillsStart() {
	chassis.moveDistance(30_in, 600_rpm, {0.2, 0, 10}, 1200, -35_deg, 150_rpm, {0.07, 0, 10}, 1100);
	// chassis.moveDistance(-3_in, 600_rpm, {0.21, 0, 10}, 1200, -35_deg, 150_rpm, {0.1, 0, 0.1}, 1000);
	chassis.turnAbsolute(-107_deg, 600_rpm, {0.025, 0, 1.3}, 2, 3, 5, TurnWheel::LEFT, 1000);
	// chassis.moveDistance(-12_in, 600_rpm, {0.2, 0, 10}, 1200, -70_deg, 350_rpm, {0.3, 0, 10}, 1500);

	// leftWing.extend();
	catapult.matchload();
}
#endif