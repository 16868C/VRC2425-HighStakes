#pragma once
#include "pros/adi.hpp"
#include <functional>

namespace lib16868C {
struct Route {
	std::string name;
	std::function<void()> route;
	double min, max;

	inline void run() {
		route();
	}
};

class AutonSelector {
	public:
	AutonSelector(pros::adi::Potentiometer& pot);

	void add(Route route);
	void add(std::string name, std::function<void()> route, double min, double max);
	
	Route& getSelectedRoute();
	double getReading();
	std::vector<Route> getAllRoutes();

	void run();

	void start();

	private:
	const double TPR = 4095;

	pros::adi::Potentiometer& pot;
	std::vector<Route> routes;

	Route& prevRoute = routes[0];
};
} // namespace lib16868C