#pragma once
#include <cmath>
#include <string>

namespace lib16868C {
typedef unsigned int uint;

namespace ReduceAngle {
	template<typename T> T reduce(T ang, T max, T min);

	template<typename T> inline T deg360(T deg);
	template<typename T> inline T deg180(T deg);
	template<typename T> inline T deg90(T deg);
	template<typename T> inline T rad2Pi(T rad);
	template<typename T> inline T radPi(T rad);
	template<typename T> inline T radPi2(T rad);
}

class Point {
	public:
	double x, y;

	Point();
	Point(double x, double y);
	Point(Point& p);

	double distTo(Point p);
	double angleTo(Point p);

	std::string toStr();
};

class Line {
	public:
	double A, B, C;

	Line();
	Line(double A, double B, double C);
	Line(Point p1, Point p2);
	Line(double m, Point p);
	Line(Line& l);

	double getSlope() const;

	Point getIntersection(Line l);

	Line getPerpendicular(Point p);
	Point getProjectionPoint(Point p);
	double getProjectionDist(Point p);

	bool isOnLine(Point p);

	std::string toStr();
};
} // namespace lib16868C