#include "math.hpp"

using namespace lib16868C;

template<typename T> T lib16868C::ReduceAngle::reduce(T ang, T max, T min) {
	T range = max - min;
	while (ang > max) ang -= range;
	while (ang < min) ang += range;
	return ang;
}
template<typename T> T lib16868C::ReduceAngle::deg360(T deg) {
	return reduce(deg, T(360), T(0));
}
template<typename T> T lib16868C::ReduceAngle::deg180(T deg) {
	return reduce(deg, T(180), T(0));
}
template<typename T> T lib16868C::ReduceAngle::deg90(T deg) {
	return reduce(deg, T(90), T(-90));
}
template<typename T> T lib16868C::ReduceAngle::rad2Pi(T rad) {
	return reduce(rad, T(M_PI * 2), T(0));
}
template<typename T> T lib16868C::ReduceAngle::radPi(T rad) {
	return reduce(rad, T(M_PI), T(0));
}
template<typename T> T lib16868C::ReduceAngle::radPi2(T rad) {
	return reduce(rad, T(M_PI_2), T(-M_PI_2));
}

/** Point **/
lib16868C::Point::Point() : Point(0, 0) {}
lib16868C::Point::Point(double x, double y) : x(x), y(y) {}
lib16868C::Point::Point(const Point& p) : Point(p.x, p.y) {}

double lib16868C::Point::distTo(Point p) {
	return std::hypot(p.x - x, p.y - y);
}
double lib16868C::Point::angleTo(Point p) {
	return std::atan2(p.y - y, p.x - x);
}

std::string lib16868C::Point::toStr() {
	return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
}

Point& lib16868C::Point::operator=(const Point& p) {
	x = p.x;
	y = p.y;
	return *this;
}

/** Line **/
lib16868C::Line::Line() : Line(-1, 1, 0) {}
lib16868C::Line::Line(double A, double B, double C) : A(A), B(B), C(C) {}
lib16868C::Line::Line(Point p1, Point p2) {
	A = -(p2.y - p1.y) / (p2.x - p1.x);
	B = 1;
	C = -p1.y + A * p1.x;
}
lib16868C::Line::Line(double m, Point p) {
	A = -m;
	B = 1;
	C = -p.y + m * p.x;
}
lib16868C::Line::Line(const Line& l) : Line(l.A, l.B, l.C) {}

double lib16868C::Line::getSlope() const {
	return -A / B;
}

Point lib16868C::Line::getIntersection(Line l) {
	double det = A * l.B - l.A * B;
	if (det == 0) return Point(NAN, NAN);

	double x = (l.B * C - B * l.C) / det;
	double y = (l.A * C - A * l.C) / det;
	return Point(x, y);
}

Line lib16868C::Line::getPerpendicular(Point p) {
	if (getSlope() == 0) return Line(INFINITY, p);
	return Line(-1 / getSlope(), p);
}
Point lib16868C::Line::getProjectionPoint(Point p) {
	if (getSlope() == 0) return Point(p.x, -C / B);
	return getIntersection(getPerpendicular(p));
}
double lib16868C::Line::getProjectionDist(Point p) {
	return p.distTo(getProjectionPoint(p));
}

bool lib16868C::Line::isOnLine(Point p) {
	return std::abs(A * p.x + B * p.y + C) <= 1e-3;
}

std::string lib16868C::Line::toStr() {
	return std::to_string(A) + "x + " + std::to_string(B) + "y + " + std::to_string(C) + " = 0";
}

Line& lib16868C::Line::operator=(const Line& l) {
	A = l.A;
	B = l.B;
	C = l.C;
	return *this;
}