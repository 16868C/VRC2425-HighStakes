#include "pose.hpp"

using namespace lib16868C;

lib16868C::Pose::Pose() : Pose(Point(), 0, 0) {}
lib16868C::Pose::Pose(Point pos, double theta, uint time) : _pos(pos), theta(theta), time(time) {}
lib16868C::Pose::Pose(double x, double y, double theta, uint time) : Pose(Point(x, y), theta, time) {}
lib16868C::Pose::Pose(okapi::QLength x, okapi::QLength y) : Pose(x.convert(okapi::inch), y.convert(okapi::inch), 0, 0) {}
lib16868C::Pose::Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle theta, uint time)
	: Pose(x.convert(okapi::inch), y.convert(okapi::inch), theta.convert(okapi::radian), time) {}
lib16868C::Pose::Pose(const Pose& p) : Pose(p._pos, p.theta, p.time) {}

okapi::QLength lib16868C::Pose::distTo(Point p) {
	return _pos.distTo(p) * okapi::inch;
}
okapi::QLength lib16868C::Pose::distTo(Pose p) {
	return distTo(p._pos);
}
okapi::QAngle lib16868C::Pose::angleTo(Point p) {
	return _pos.angleTo(p) * okapi::radian;
}
okapi::QAngle lib16868C::Pose::angleTo(Pose p) {
	return angleTo(p._pos);
}

std::string lib16868C::Pose::toStr() {
	return "{" + _pos.toStr() + ", theta: " + std::to_string(theta) + ", time: " + std::to_string(time) + "}";
}

Pose& lib16868C::Pose::operator=(const Pose& p) {
	_pos = p._pos;
	theta = p.theta;
	time = p.time;
	return *this;
}