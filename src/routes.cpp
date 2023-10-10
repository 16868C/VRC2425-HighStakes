#include "routes.hpp"
#include "robotconfig.hpp"
#include "16868Z/util/util.hpp"

using namespace lib16868C;

/*
void goalSide() {
	inertial.set_rotation(0);
	
	// Intake triball #1
	intake.intake(false);
	chassis.moveDistance(34, 600, {0.0532, 0, 0.6}, 300, 0, 300, {0.015, 0, 0.01}, 1000);
	chassis.moveDistance(-33, 600, {0.045, 0, 0.6}, 400, 0, 300, {0.015, 0, 0}, 1000);
	intake.stop();

	// Push preload triball
	chassis.turnAbsolute(-45, 600, {0.018, 0, 0.8}, 100, 2, 5, TurnWheel::LEFT, 800);
	chassis.moveDistance(-15, 600, {0.1, 0, 0.6}, 500, -45, 300, {0.015, 0, 0}, 700);
	chassis.turnAbsolute(-90, 600, {0.016, 0, 0.9}, 100, 3, 5, TurnWheel::LEFT, 700);
	// Push preload triball into goal
	chassis.moveTank(-12000, -12000);
	pros::delay(400);
	chassis.moveTank(0, 0);

	// Push triball #1 into goal
	chassis.moveDistance(5, 600, {0.15, 0, 0.6}, 400, -90, 300, {0.015, 0, 0}, 800);
	chassis.turnAbsolute(90, 600, {0.01165, 0, 0.9}, 100, 2, 5, TurnWheel::BOTH, 900);
	intake.outtake(true, 500, true);
	pros::delay(100);
	chassis.moveTank(6000, 6000);
	pros::delay(400);
	chassis.moveTank(0, 0);

	// Intake triball #2
	chassis.moveDistance(-8, 600, {0.15, 0, 0.6}, 400, 90, 300, {0.01, 0, 0}, 600);
	chassis.turnAbsolute(17, 600, {0.0115, 0, 0.9}, 100, 2, 5, TurnWheel::BOTH);
	intake.intake(false);
	chassis.moveDistance(52, 600, {0.044, 0, 0.6}, 600, 14, 300, {0.015, 0, 0}, 1300);
	chassis.turnAbsolute(155, 600, {0.0147, 0, 0.9}, 100, 2, 5, TurnWheel::BOTH, 800);
	intake.stop();
	// Push triball #2 into goal
	chassis.moveDistance(20, 600, {0.0525, 0, 0.6}, 400, 155, 300, {0.015, 0, 0}, 800);
	intake.outtake(true, 500, true);
	chassis.moveTank(6000, 6000);
	pros::delay(400);
	chassis.moveTank(0, 0);

	// Intake triball #3
	chassis.moveDistance(-12, 600, {0.056, 0, 0.6}, 400, 155, 300, {0.015, 0, 0}, 800);
	intake.intake(false);
	chassis.turnAbsolute(90, 600, {0.0117, 0, 0.9}, 100, 2, 5, TurnWheel::BOTH, 500);
	// Push triball #3 into goal
	chassis.moveDistance(12, 600, {0.055, 0, 0.6}, 200, 90, 300, {0.015, 0, 0}, 900);
	chassis.turnAbsolute(180, 600, {0.0118, 0, 0.9}, 100, 2, 5, TurnWheel::BOTH, 600);
	intake.outtake(true, 500, true);
	chassis.moveTank(6000, 6000);
	pros::delay(400);
	chassis.moveTank(0, 0);
}

void goalSideBar() {
	inertial.set_rotation(0);

	// Intake triball #1
	intake.intake(false);
	chassis.moveDistance(34, 600, {0.0532, 0, 0.6}, 300, 0, 300, {0.015, 0, 0.01}, 1000);
	chassis.moveDistance(-33, 600, {0.045, 0, 0.6}, 400, 0, 300, {0.015, 0, 0}, 1000);
	intake.stop();
	
	// Push preload triball
	chassis.turnAbsolute(-45, 600, {0.018, 0, 0.8}, 100, 2, 5, TurnWheel::LEFT, 800);
	chassis.moveDistance(-15, 600, {0.1, 0, 0.6}, 500, -45, 300, {0.015, 0, 0}, 700);
	chassis.turnAbsolute(-90, 600, {0.016, 0, 0.9}, 100, 3, 5, TurnWheel::LEFT, 700);
	// Push preload triball into goal
	chassis.moveTank(-12000, -12000);
	pros::delay(400);
	chassis.moveTank(0, 0);

	// Push triball #1 into goal
	chassis.moveDistance(5, 600, {0.15, 0, 0.6}, 400, -90, 300, {0.015, 0, 0}, 800);
	chassis.turnAbsolute(90, 600, {0.01165, 0, 0.9}, 100, 2, 5, TurnWheel::BOTH, 900);
	intake.outtake(true, 500, true);
	pros::delay(100);
	chassis.moveTank(6000, 6000);
	pros::delay(400);
	chassis.moveTank(0, 0);

	// Intake triball #2
	chassis.moveDistance(-8, 600, {0.15, 0, 0.6}, 400, 90, 300, {0.01, 0, 0}, 600);
	chassis.turnAbsolute(17, 600, {0.0115, 0, 0.9}, 100, 2, 5, TurnWheel::BOTH);
	intake.intake(false);
	chassis.moveDistance(52, 600, {0.044, 0, 0.6}, 600, 14, 300, {0.015, 0, 0}, 1300);
	chassis.turnAbsolute(155, 600, {0.0147, 0, 0.9}, 100, 2, 5, TurnWheel::BOTH, 800);
	intake.stop();
	chassis.moveDistance(20, 600, {0.0525, 0, 0.6}, 400, 155, 300, {0.015, 0, 0}, 800);
	intake.outtake(true, 500, true);

	chassis.moveTank(6000, 6000);
	pros::delay(400);
	chassis.moveTank(0, 0);

	chassis.turnAbsolute(315, 600, {0.012, 0, 0.9}, 100, 2, 5, TurnWheel::BOTH);
	chassis.moveDistance(40, 600, {0.044, 0, 0.6}, 600, 315, 300, {0.015, 0, 0}, 1300);
	chassis.turnAbsolute(0, 600, {0.0117, 0, 0.9}, 100, 2, 5, TurnWheel::BOTH, 500);
}

void matchloadAWP() {
	inertial.set_rotation(-135);

	intake.spinRear(-6000);

	clothesline.extend();
	pros::Task matchloadTriball([&] {
		pros::delay(300);
		clothesline.retract();
	});
	chassis.moveDistance(6, 600, {0.15, 0, 0.6}, 300, -135, 300, {0.015, 0, 0.01});
	chassis.turnAbsolute(-90, 600, {0.018, 0, 0.8}, 100, 2, 5, TurnWheel::BOTH);
	chassis.turnAbsolute(-135, 600, {0.0135, 0, 0.8}, 100, 2, 5, TurnWheel::BOTH);

	intake.stop();

	chassis.moveDistance(-13, 600, {0.0525, 0, 0.6}, 400, -135, 300, {0.015, 0, 0});
	chassis.turnAbsolute(-180, 400, {0.026, 0, 0.7}, 100, 2, 5, TurnWheel::LEFT);
	chassis.moveDistance(-32, 600, {0.0532, 0, 0.6}, 400, -180, 300, {0.015, 0, 0});
}
*/