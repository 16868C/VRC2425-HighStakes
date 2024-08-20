#include "pose.hpp"
#include "util.hpp"

using namespace lib16868C;

lib16868C::Pose::Pose() : Pose(Point(), 0, 0) {}

lib16868C::Pose::Pose(Point pos, double theta, uint time) : pos(pos), theta(theta), time(time) {}

lib16868C::Pose::Pose(double x, double y, double theta, uint time) : Pose(Point(x, y), theta, time) {}

lib16868C::Pose::Pose(okapi::QLength x, okapi::QLength y) : Pose(x.convert(okapi::inch), y.convert(okapi::inch), 0, 0) {}

lib16868C::Pose::Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle theta)
	: Pose(x.convert(okapi::inch), y.convert(okapi::inch), theta.convert(okapi::radian), 0) {}

lib16868C::Pose::Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle theta, uint time)
	: Pose(x.convert(okapi::inch), y.convert(okapi::inch), theta.convert(okapi::radian), time) {}

// lib16868C::Pose::Pose(const Pose& p) : Pose(p.pos, p.theta, p.time) {}

okapi::QLength lib16868C::Pose::distTo(Point p) {
	return pos.distTo(p) * okapi::inch;
}
okapi::QLength lib16868C::Pose::distTo(Pose p) {
	return distTo(p.pos);
}
okapi::QAngle lib16868C::Pose::angleTo(Point p) {
	return pos.angleTo(p) * okapi::radian;
}
okapi::QAngle lib16868C::Pose::angleTo(Pose p) {
	return angleTo(p.pos);
}

std::string lib16868C::Pose::toStr() {
	return "{" + pos.toStr() + ", theta: " + std::to_string(Util::radToDeg(theta)) + ", time: " + std::to_string(time) + "}";
}

Pose Pose::operator+(Pose rhs) {
	return Pose(pos + rhs.pos, theta, time);
}
Pose Pose::operator+(Point rhs) {
	return Pose(pos + rhs, theta, time);
}
Pose Pose::operator-(Pose rhs) {
	return Pose(pos - rhs.pos, theta, time);
}
Pose Pose::operator-(Point rhs) {
	return Pose(pos - rhs, theta, time);
}
Pose Pose::operator*(double rhs) {
	return Pose(pos * rhs, theta, time);
}
Pose Pose::operator/(double rhs) {
	return Pose (pos / rhs, theta, time);
}

Pose& lib16868C::Pose::operator=(const Pose& p) {
	pos = p.pos;
	x = p.pos.x;
	y = p.pos.y;
	theta = p.theta;
	time = p.time;
	return *this;
}