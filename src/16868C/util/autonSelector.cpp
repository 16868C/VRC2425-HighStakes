#include "autonSelector.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

using namespace lib16868C;

AutonSelector::AutonSelector(pros::adi::Potentiometer& pot) : pot(pot) {}

void AutonSelector::add(Route route) {
	if (routes.size() == 10) return;

	if (route.max < route.min) {
		add(route.name, route.route, 0, route.max);
		add(route.name, route.route, route.min, TPR);
	}
	routes.push_back(route);
}
void AutonSelector::add(std::string name, std::function<void()> route, double min, double max) {
	add({name, route, min, max});
}

Route& AutonSelector::getSelectedRoute() {
	for (Route& route : routes) {
		if (pot.get_value() >= route.min && pot.get_value() <= route.max) {
			return route;
		}
	}
	return routes[0];
}
double AutonSelector::getReading() {
	return pot.get_value();
}
std::vector<Route> AutonSelector::getAllRoutes() {
	return routes;
}

void AutonSelector::run() {
	getSelectedRoute().run();
}

void AutonSelector::start() {
	pros::Task([this] {
		while (pros::competition::is_disabled()) {
			if (getSelectedRoute() != prevRoute) {
				std::cout << getSelectedRoute().name << "\n";
				pros::c::controller_clear_line(pros::E_CONTROLLER_MASTER, 0);
				pros::c::controller_set_text(pros::E_CONTROLLER_MASTER, 0, 0, getSelectedRoute().name.c_str());
				pros::lcd::print(7, getSelectedRoute().name.c_str());
			}

			prevRoute = getSelectedRoute();
			pros::delay(200);
		}
	});
}