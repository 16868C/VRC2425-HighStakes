#include "16868Z/util/util.hpp"
#include "api.h"

using namespace lib16868C;

void Util::runAsBlocking(std::function<void()> fn, std::function<bool()> endCond, int timeout, int pollRate, int paddingDelay) {
	fn();
	Util::blocking(endCond, timeout, pollRate, paddingDelay);
}
void Util::blocking(std::function<bool()> endCond, int timeout, int pollRate, int paddingDelay) {
	pros::delay(paddingDelay);
	int st = pros::millis();
	uint32_t time = pros::millis();
	while (!endCond()) {
		if (timeout != -1 && pros::millis() - st > timeout) return;
		pros::Task::delay_until(&time, pollRate);
	}
	pros::delay(paddingDelay);
}

pros::Task Util::runAsync(std::function<void()> fn) {
	return pros::Task(fn, "Async Task");
}