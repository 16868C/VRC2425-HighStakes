#pragma once
#include <cmath>

namespace lib16868X {
typedef unsigned int uint;

template<typename T> inline int sgn(T n) {
	return (n > T(0)) - (n < T(0));
}

template<typename T> inline T clamp(T n, T min, T max) {
	return std::min(std::max(n, min), max);
}

inline double degToRad(double deg) {
	return deg / 180 * M_PI;
}
inline double radToDeg(double rad) {
	return rad / M_PI * 180;
}
} // namespace lib16868X