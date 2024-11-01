#pragma once
#include <string>

class Point {
	public:
	double x, y;

	Point();
	Point(double x, double y);

	double distTo(Point p);
	double angleTo(Point p);

	std::string toStr();

	Point operator+(Point rhs);
	Point operator-(Point rhs);
	Point operator*(double rhs);
	Point operator/(double rhs);

	static Point lerp(Point a, Point b, double t);
};