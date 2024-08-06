#include "math.hpp"
#include "logger.hpp"
#include <limits>
#include <iomanip>
#include <sstream>

using namespace lib16868C;

double lib16868C::sma(double input, double prev) {
	return (input + prev) / 2.0;
}
double lib16868C::ema(double input, double prev, double a) {
	return a * input + (1 - a) * prev;
}

double lib16868C::normalizeAngleDeg(double deg) {
	double mod = fmod(deg, 360);
	if (mod < 0) mod += 360;
	return mod;
}
double lib16868C::normalizeAngleRad(double rad) {
	double mod = fmod(rad, 2 * M_PI);
	if (mod < 0) mod += 2 * M_PI;
	return mod;
}

double lib16868C::angleErrorDeg(double target, double current, TurnDirection direction) {
	double error = std::abs(normalizeAngleDeg(target) - normalizeAngleDeg(current));
	if (error <= 180) {
		if (normalizeAngleDeg(current) <= normalizeAngleDeg(target)) return current + error;
		else return current - error;
	} else {
		if (normalizeAngleDeg(current) <= normalizeAngleDeg(target)) return current - (360 - error);
		else return current + (360 - error);
	}
}
double lib16868C::angleErrorRad(double target, double current, TurnDirection direction) {
	double error = std::abs(normalizeAngleRad(target) - normalizeAngleRad(current));
	if (error <= M_PI) {
		if (normalizeAngleRad(current) <= normalizeAngleRad(target)) return current + error;
		else return current - error;
	} else {
		if (normalizeAngleRad(current) <= normalizeAngleRad(target)) return current - (2 * M_PI - error);
		else return current + (2 * M_PI - error);
	}
}

/** Point **/
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

/** Line **/
Line::Line() : Line(-1, 1, 0) {}
Line::Line(double A, double B, double C) : A(A), B(B), C(C) {}
Line::Line(Point p1, Point p2) {
	if (p1.x == p2.x) {
		A = 1;
		B = 0;
		C = -p1.x;
	} else {
		A = -(p2.y - p1.y) / (p2.x - p1.x);
		B = 1;
		C = -p1.y + A * p1.x;
	}
}
Line::Line(double m, Point p) {
	if (std::isinf(m)) {
		A = 1;
		B = 0;
		C = -p.x;
	} else {
		A = -m;
		B = 1;
		C = -p.y + m * p.x;
	}
}
Line::Line(const Line& l) : Line(l.A, l.B, l.C) {}

double Line::getSlope() const {
	if (B == 0) return std::numeric_limits<double>::infinity();
	return -A / B;
}

Point Line::getIntersection(Line l) {
	double det = A * l.B - l.A * B;
	if (det == 0) return Point(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());

	double x = (B * l.C - l.B * C) / det;
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
	std::stringstream ss;
	ss << std::fixed << std::setprecision(2) << A << "x + " << B << "y + " << C << " = 0";
	return ss.str();
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