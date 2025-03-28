#pragma once
#include "16868C/util/pose.hpp"
#include "16868C/util/point.hpp"
#include <string>

namespace lib16868C {
enum class TurnDirection {
	CW = -1,
	CCW = 1,
	SHORTEST = 0
};

double sma(double input, double prev);
double ema(double input, double prev, double a);

double normalizeAngle(double ang, bool rad = true);
double getTargetHeading(double target, double current, bool rad = true, TurnDirection dir = TurnDirection::SHORTEST);

namespace ReduceAngle {
	template<typename T> inline T reduce(T ang, T max, T min) {
		T range = max - min;
		while (ang > max) ang -= range;
		while (ang < min) ang += range;
		return ang;
	}

	template<typename T> inline T deg360(T deg) {
		return reduce(deg, T(360), T(0));
	}
	template<typename T> inline T deg180(T deg) {
		return reduce(deg, T(180), T(0));
	}
	template<typename T> inline T deg90(T deg) {
		return reduce(deg, T(90), T(-90));
	}
	template<typename T> inline T rad2Pi(T rad) {
		return reduce(rad, T(M_PI * 2), T(0));
	}
	template<typename T> inline T radPi(T rad) {
		return reduce(rad, T(M_PI), T(0));
	}
	template<typename T> inline T radPi2(T rad) {
		return reduce(rad, T(M_PI_2), T(-M_PI_2));
	}
}

double getRadius(Pose p1, Point p2);

class Line {
	public:
	double A, B, C;

	Line();
	Line(double A, double B, double C);
	Line(Point p1, Point p2);
	Line(double m, Point p);
	Line(const Line& l);

	double getSlope() const;

	Point getIntersection(Line l);

	Line getPerpendicular(Point p);
	Point getProjectionPoint(Point p);
	double getProjectionDist(Point p);

	bool isOnLine(Point p);

	std::string toStr();

	Line& operator=(const Line& l);
};

class LineSegment : public Line{
	public:
	Point p1, p2;

	LineSegment();
	LineSegment(Point p1, Point p2);
	LineSegment(const LineSegment& l);

	Line getLine() const;

	double getLength();
	Point getMidpoint() const;

	bool isInsideSegment(Point p);

	std::string toStr();

	LineSegment& operator=(const LineSegment& l);
};
} // namespace lib16868C