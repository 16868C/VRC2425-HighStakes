#pragma once
#include "api.h"
#include <cmath>
#include <functional>
#include <queue>
#include <vector>

namespace lib16868Z {
typedef unsigned int uint;

namespace Util {
template<typename T> inline int sgn(T n) {
	return (n > T(0)) - (n < T(0));
}

inline double degToRad(double deg) {
	return deg / 180 * M_PI;
}
inline double radToDeg(double rad) {
	return rad / M_PI * 180;
}

template<typename T> inline T avg(std::vector<T> v) {
	T sum = 0;
	for (T d : v) sum += d;
	return sum / v.size();
}

template<typename T> inline std::vector<T> queueToVector(const std::queue<T>& q) {
	std::vector<T> v;
	std::queue<T> temp = q;
	while (!temp.empty()) {
		v.push_back(temp.front());
		temp.pop();
	}
	return v;
}
template<typename T> inline std::queue<T> vectorToQueue(const std::vector<T>& v) {
	std::queue<T> q;
	for (size_t i = 0; i < v.size(); i++) {
		q.push(v[i]);
	}
	return q;
}

void runAsBlocking(std::function<void()> fn, std::function<bool()> endCond, int timeout = -1, int pollRate = 10, int paddingDelay = 5);
void blocking(std::function<bool()> endCond, int timeout = -1, int pollRate = 10, int paddingDelay = 5);
pros::Task runAsync(std::function<void()> fn);
} // namespace lib16868Z::Util
} // namespace Util