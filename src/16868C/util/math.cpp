#include "math.hpp"
#include "logger.hpp"
#include <cmath>
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

double lib16868C::normalizeAngle(double ang, bool rad) {
	double maxAng = rad ? 2 * M_PI : 360;
	double mod = fmod(ang, maxAng);
	if (mod < 0) mod += maxAng;
	return mod;
}

double lib16868C::getTargetHeading(double target, double current, bool rad, TurnDirection direction) {
	double maxAng = rad ? 2 * M_PI : 360;
	double error = target - current;
	while (error > maxAng / 2) error -= maxAng;
	while (error < -maxAng / 2) error += maxAng;
	return current + error;
}

double lib16868C::getRadius(Pose p1, Point p2) {
	if (fmod(p1.theta, M_PI) == 0) p1.theta -= 1e-5;

	double a = 0.5 * (p2.x * p2.x - p1.x * p1.x + p2.y * p2.y + p1.y * p1.y);
	double b = p2.y * p1.y + p1.x * (p2.y - p1.y) * 1/tan(p1.theta);
	double c = p2.x - p1.x - (p2.y - p1.y) * 1/tan(p1.theta);
	double h = (a - b) / c;
	double k = -1/tan(p1.theta) * (h - p1.x) + p1.y;
	Point o(h, k);

	return p1.distTo(o);
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