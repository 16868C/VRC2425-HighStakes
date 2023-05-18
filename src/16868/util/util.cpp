#include "16868X/util/util.hpp"
#include "api.h"

using namespace lib16868X;

template<typename T> std::vector<T> queueToVector(const std::queue<T> q) {
	std::vector<T> v;
	std::queue<T> temp = q;
	while (!temp.empty()) {
		v.push_back(temp.front());
		temp.pop();
	}
	return v;
}
template<typename T> std::queue<T> vectorToQueue(const std::vector<T> v) {
	std::queue<T> q;
	for (size_t i = 0; i < v.size(); i++) {
		q.push(v[i]);
	}
	return q;
}

void runAsBlocking(std::function<void()> fn, std::function<bool()> endCond, int timeout, int pollRate) {
	fn();
	blocking(endCond, timeout, pollRate);
}
void blocking(std::function<bool()> endCond, int timeout, int pollRate, int paddingDelay) {
	pros::delay(paddingDelay);
	int st = pros::millis();
	uint32_t time = pros::millis();
	while (!endCond()) {
		if (timeout != -1 && pros::millis() - st > timeout) return;
		pros::Task::delay_until(&time, pollRate);
	}
	pros::delay(paddingDelay);
}

pros::Task runAsync(std::function<void()> fn) {
	return pros::Task(fn, "Async Task");
}