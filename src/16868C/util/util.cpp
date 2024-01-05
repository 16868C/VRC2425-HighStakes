#include "16868C/util/util.hpp"

using namespace lib16868C;

template<typename T> int Util::sgn(T n) {
	return (n > T(0)) - (n < T(0));
}

double Util::degToRad(double deg) {
	return deg * M_PI / 180;
}
double Util::radToDeg(double rad) {
	return rad * 180 / M_PI;
}

template<typename T> T Util::avg(std::vector<T> v) {
	T sum = T(0);
	for (size_t i = 0; i < v.size(); i++) sum += v[i];
	return sum / v.size();
}
template<typename T> T Util::avg(std::initializer_list<T> l) {
	return avg(std::vector<T>(l));
}

template<typename T> std::vector<T> Util::queueToVector(const std::queue<T>& q) {
	std::vector<T> v;
	std::queue<T> tmp = q;
	while (!tmp.empty()) {
		v.push_back(tmp.front());
		tmp.pop();
	}
	return v;
}
template<typename T> std::queue<T> Util::vectorToQueue(const std::vector<T>& v) {
	std::queue<T> q;
	for (size_t i = 0; i < v.size(); i++) {
		q.push(v[i]);
	}
	return q;
}

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