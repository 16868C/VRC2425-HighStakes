#include "main.h"
#include "robotconfig.hpp"
#include "16868C/util/logger.hpp"
#include "routes.hpp"

using namespace lib16868C;

void initialize() {
	pros::lcd::initialize();
}

void disabled() {
	
}

void competition_initialize() {}

void autonomous() {
	uint st = pros::millis();

	print("Auton took %d ms\n", pros::millis() - st);
}

void opcontrol() {
	Logger log("test.txt");
	log.print("test\n");

	while (true) {
		pros::delay(50);
	}
}