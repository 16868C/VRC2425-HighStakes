#pragma once
#include "api.h"
#include <cmath>
#include <functional>
#include <queue>
#include <vector>

namespace lib16868C {
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

inline double mToIn(double m) {
	return m * 39.3701;
}
inline double inToM(double in) {
	return in / 39.3701;
}

template<typename T> inline T avg(std::vector<T> v) {
	T sum = 0;
	for (T d : v) sum += d;
	return sum / v.size();
}
template<typename T> inline T avg(std::initializer_list<T> l) {
	avg(std::vector<T>(l));
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

namespace ReduceAngle {
template<typename T> inline T reduce(T ang, T max, T min) {
	T range = max - min;
	while (ang > max) ang -= range;
	while (ang < min) ang += range;
	return ang;
}
template<typename T> inline T deg360(T deg) {
	return reduce(deg, static_cast<T>(360), static_cast<T>(0));
}
template<typename T> inline T deg180(T deg) {
	return reduce(deg, static_cast<T>(180), static_cast<T>(-180));
}
template<typename T> inline T deg90(T deg) {
	return reduce(deg, static_cast<T>(90), static_cast<T>(-90));
}
template<typename T> inline T rad2Pi(T rad) {
	return reduce(rad, static_cast<T>(2 * M_PI), static_cast<T>(0));
}
template<typename T> inline T radPi(T rad) {
	return reduce(rad, static_cast<T>(M_PI), static_cast<T>(-M_PI));
}
template<typename T> inline T radPi2(T rad) {
	return reduce(rad, static_cast<T>(M_PI_2), static_cast<T>(-M_PI_2));
}
}

void runAsBlocking(std::function<void()> fn, std::function<bool()> endCond, int timeout = -1, int pollRate = 10, int paddingDelay = 5);
void blocking(std::function<bool()> endCond, int timeout = -1, int pollRate = 10, int paddingDelay = 5);
pros::Task runAsync(std::function<void()> fn);
} // namespace lib16868C::Util
} // namespace Util