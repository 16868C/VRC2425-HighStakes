#pragma once

namespace lib16868C {
class Vector2d {
	public:
	double x, y;

	Vector2d();
	Vector2d(double x, double y);

	double dot(Vector2d rhs);
	double mag();
	double angle();

	void normalize();
	Vector2d noramlized();

	Vector2d operator+(Vector2d) const;
	Vector2d operator-(Vector2d) const;
	Vector2d operator*(double) const;
	Vector2d operator/(double) const;

	Vector2d& operator+=(Vector2d);
	Vector2d& operator-=(Vector2d);
	Vector2d& operator*=(double);
	Vector2d& operator/=(double);
	Vector2d& operator=(Vector2d);

	bool operator==(const Vector2d&) const;
	bool operator!=(const Vector2d&) const;
};
} // namespace lib16868C