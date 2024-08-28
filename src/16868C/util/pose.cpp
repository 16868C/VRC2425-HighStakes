#include "pose.hpp"
#include "util.hpp"

using namespace lib16868C;

lib16868C::Pose::Pose() : Pose(Point(), 0, 0) {}
lib16868C::Pose::Pose(Point pos, double theta, uint time) : Point(pos), theta(theta), time(time) {}
lib16868C::Pose::Pose(double x, double y, double theta, uint time) : Pose(Point(x, y), theta, time) {}
lib16868C::Pose::Pose(okapi::QLength x, okapi::QLength y) : Pose(x.convert(okapi::inch), y.convert(okapi::inch), 0, 0) {}
lib16868C::Pose::Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle theta)
	: Pose(x.convert(okapi::inch), y.convert(okapi::inch), theta.convert(okapi::radian), 0) {}
lib16868C::Pose::Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle theta, uint time)
	: Pose(x.convert(okapi::inch), y.convert(okapi::inch), theta.convert(okapi::radian), time) {}
lib16868C::Pose::Pose(const Pose& p) : Pose(p.x, p.y, p.theta, p.time) {}

std::string lib16868C::Pose::toStr() {
	return "{" + toStr() + ", theta: " + std::to_string(Util::radToDeg(theta)) + ", time: " + std::to_string(time) + "}";
}

Pose Pose::operator+(Point rhs) {
	return Pose(x + rhs.x, y + rhs.y, theta, time);
}
Pose Pose::operator+(Pose rhs) {
	return Pose(x + rhs.x, y + rhs.y, theta, time);
}
Pose Pose::operator-(Point rhs) {
	return Pose(x - rhs.x, y - rhs.y, theta, time);
}
Pose Pose::operator-(Pose rhs) {
	return Pose(x - rhs.x, y - rhs.y, theta, time);
}
Pose Pose::operator*(double rhs) {
	return Pose(x * rhs, y * rhs, theta, time);
}
Pose Pose::operator/(double rhs) {
	return Pose(x / rhs, y / rhs, theta, time);
}

Pose& lib16868C::Pose::operator=(const Pose& p) {
	x = p.x;
	y = p.y;
	theta = p.theta;
	time = p.time;
	return *this;
}