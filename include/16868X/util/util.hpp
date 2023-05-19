#pragma once
#include "api.h"
#include <cmath>
#include <functional>
#include <queue>
#include <vector>

namespace lib16868X {
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

template<typename T> std::vector<T> queueToVector(const std::queue<T> q);
template<typename T> std::queue<T> vectorToQueue(const std::vector<T> v);

void runAsBlocking(std::function<void()> fn, std::function<bool()> endCond, int timeout = -1, int pollRate = 10, int paddingDelay = 5);
void blocking(std::function<bool()> endCond, int timeout = -1, int pollRate = 10, int paddingDelay = 5);
pros::Task runAsync(std::function<void()> fn);
} // namespace lib16868X::Util
} // namespace Util