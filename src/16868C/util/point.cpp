#include "point.hpp"
#include <cmath>

Point::Point() : Point(0, 0) {}
Point::Point(double x, double y) : x(x), y(y) {}

double Point::distTo(Point p) {
	return std::hypot(p.x - x, p.y - y);
}
double Point::angleTo(Point p) {
	return std::atan2(p.y - y, p.x - x);
}

std::string Point::toStr() {
	return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
}

Point Point::operator+(Point rhs) {
	return Point(x + rhs.x, y + rhs.y);
}
Point Point::operator-(Point rhs) {
	return Point(x - rhs.x, y - rhs.y);
}
Point Point::operator*(double rhs) {
	return Point(x * rhs, y * rhs);
}
Point Point::operator/(double rhs) {
	return Point(x / rhs, y / rhs);
}

Point Point::lerp(Point a, Point b, double t) {
	return a * (1 - t) + b * t;
}