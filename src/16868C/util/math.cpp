#include "math.hpp"

using namespace lib16868C;

template<typename T> T ReduceAngle::reduce(T ang, T max, T min) {
	T range = max - min;
	while (ang > max) ang -= range;
	while (ang < min) ang += range;
	return ang;
}
template<typename T> T ReduceAngle::deg360(T deg) {
	return reduce(deg, T(360), T(0));
}
template<typename T> T ReduceAngle::deg180(T deg) {
	return reduce(deg, T(180), T(0));
}
template<typename T> T ReduceAngle::deg90(T deg) {
	return reduce(deg, T(90), T(-90));
}
template<typename T> T ReduceAngle::rad2Pi(T rad) {
	return reduce(rad, T(M_PI * 2), T(0));
}
template<typename T> T ReduceAngle::radPi(T rad) {
	return reduce(rad, T(M_PI), T(0));
}
template<typename T> T ReduceAngle::radPi2(T rad) {
	return reduce(rad, T(M_PI_2), T(-M_PI_2));
}

/** Point **/
Point::Point() : Point(0, 0) {}
Point::Point(double x, double y) : x(x), y(y) {}
Point::Point(const Point& p) : Point(p.x, p.y) {}

double Point::distTo(Point p) {
	return std::hypot(p.x - x, p.y - y);
}
double Point::angleTo(Point p) {
	return std::atan2(p.y - y, p.x - x);
}

std::string Point::toStr() {
	return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
}

Point& Point::operator=(const Point& p) {
	x = p.x;
	y = p.y;
	return *this;
}

/** Line **/
Line::Line() : Line(-1, 1, 0) {}
Line::Line(double A, double B, double C) : A(A), B(B), C(C) {}
Line::Line(Point p1, Point p2) {
	A = -(p2.y - p1.y) / (p2.x - p1.x);
	B = 1;
	C = -p1.y + A * p1.x;
}
Line::Line(double m, Point p) {
	A = -m;
	B = 1;
	C = -p.y + m * p.x;
}
Line::Line(const Line& l) : Line(l.A, l.B, l.C) {}

double Line::getSlope() const {
	return -A / B;
}

Point Line::getIntersection(Line l) {
	double det = A * l.B - l.A * B;
	if (det == 0) return Point(NAN, NAN);

	double x = (l.B * C - B * l.C) / det;
	double y = (l.A * C - A * l.C) / det;
	return Point(x, y);
}

Line Line::getPerpendicular(Point p) {
	if (getSlope() == 0) return Line(INFINITY, p);
	return Line(-1 / getSlope(), p);
}
Point Line::getProjectionPoint(Point p) {
	if (getSlope() == 0) return Point(p.x, -C / B);
	return getIntersection(getPerpendicular(p));
}
double Line::getProjectionDist(Point p) {
	return p.distTo(getProjectionPoint(p));
}

bool Line::isOnLine(Point p) {
	return std::abs(A * p.x + B * p.y + C) <= 1e-3;
}

std::string Line::toStr() {
	return std::to_string(A) + "x + " + std::to_string(B) + "y + " + std::to_string(C) + " = 0";
}

Line& Line::operator=(const Line& l) {
	A = l.A;
	B = l.B;
	C = l.C;
	return *this;
}

/** LineSegment **/
LineSegment::LineSegment() : LineSegment(Point(), Point()) {}
LineSegment::LineSegment(Point p1, Point p2) : Line(p1, p2), p1(p1), p2(p2) {}
LineSegment::LineSegment(const LineSegment& l) : LineSegment(l.p1, l.p2) {}

Line LineSegment::getLine() const {
	return Line(p1, p2);
}

double LineSegment::getLength() {
	return p1.distTo(p2);
}
Point LineSegment::getMidpoint() const {
	return { (p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0 };
}

bool LineSegment::isInsideSegment(Point p) {
	return p.distTo(p1) + p.distTo(p2) <= getLength();
}

std::string LineSegment::toStr() {
	return p1.toStr() + " -> " + p2.toStr();
}

LineSegment& LineSegment::operator=(const LineSegment& l) {
	p1 = l.p1;
	p2 = l.p2;
	return *this;
}