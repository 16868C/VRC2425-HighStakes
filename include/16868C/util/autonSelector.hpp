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

	void add(int idx, std::string name, std::function<void()> route);
	
	int getSelectedIdx();
	Route& getSelectedRoute();
	double getReading();
	std::array<Route, 11> getAllRoutes();

	void run();

	void start();
	void stop();

	private:
	const double TPR = 4095;
	pros::adi::Potentiometer& pot;
	std::array<Route, 11> routes = {
		Route("", [] {}, 1776, 2290),
		Route("", [] {}, 2290, 2790),
		Route("", [] {}, 2790, 3367),
		Route("", [] {}, 3367, 3975),
		Route("", [] {}, 3975, 4095),
		Route("", [] {}, 47, 414),
		Route("", [] {}, 414, 791),
		Route("", [] {}, 791, 1160),
		Route("", [] {}, 1160, 1487),
		Route("", [] {}, 1487, 1776),
		Route("", [] {}, 0, 47),
	};
	bool selecting = true;

	int prev = -1;
};
} // namespace lib16868C