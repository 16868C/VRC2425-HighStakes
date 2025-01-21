#include "autonSelector.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

using namespace lib16868C;

AutonSelector::AutonSelector(pros::adi::Potentiometer& pot) : pot(pot) {}

void AutonSelector::add(int idx, std::string name, std::function<void()> route) {
	routes[idx - 1].name = name;
	routes[idx - 1].route = route;
	if (idx == 5) {
		routes[10].name = name;
		routes[10].route = route;
	}
}

int AutonSelector::getSelectedIdx() {
	for (int i = 0; i <= 10; i++) {
		if (pot.get_value() >= routes[i].min && pot.get_value() <= routes[i].max) {
			if (i == 10) return 5;
			return i + 1;
		}
	}
	return 0;
}
Route& AutonSelector::getSelectedRoute() {
	return routes[getSelectedIdx() - 1];
}
double AutonSelector::getReading() {
	return pot.get_value();
}
std::array<Route, 11> AutonSelector::getAllRoutes() {
	return routes;
}

void AutonSelector::run() {
	stop();
	getSelectedRoute().run();
}

void AutonSelector::start() {
	selecting = true;
	pros::Task([&] {
		while (selecting) {
			if (getSelectedIdx() != prev) {
				std::cout << getSelectedRoute().name << "\n";
				pros::c::controller_clear_line(pros::E_CONTROLLER_MASTER, 0);
				pros::c::controller_set_text(pros::E_CONTROLLER_MASTER, 0, 0, getSelectedRoute().name.c_str());
				pros::lcd::print(7, getSelectedRoute().name.c_str());
			}

			prev = getSelectedIdx();
			pros::delay(200);
		}
	});
}
void AutonSelector::stop() {
	selecting = false;
}