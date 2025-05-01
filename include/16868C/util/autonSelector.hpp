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
		Route("", [] {}, 846, 1349),
		Route("", [] {}, 1350, 1810),
		Route("", [] {}, 1811, 2314),
		Route("", [] {}, 2315, 2786),
		Route("", [] {}, 2787, 3287),
		Route("", [] {}, 3288, 3740),
		Route("", [] {}, 3741, 4095),
		Route("", [] {}, 0, 234),
		Route("", [] {}, 235, 594),
		Route("", [] {}, 595, 845),
		Route("", [] {}, -1, -1),
	};
	bool selecting = true;

	int prev = -1;
};
} // namespace lib16868C