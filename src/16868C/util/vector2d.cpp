#include "16868C/util/vector2d.hpp"
#include <cmath>

using namespace lib16868C;

Vector2d::Vector2d() : x(0), y(0) {}
Vector2d::Vector2d(double x, double y) : x(x), y(y) {}

double Vector2d::dot(Vector2d rhs) {
	return x * rhs.x + y * rhs.y;
}
double Vector2d::mag() {
	return std::hypot(x, y);
}
double Vector2d::angle() {
	return std::atan2(y, x);
}

void Vector2d::normalize() {
	*this /= mag();
}
Vector2d Vector2d::noramlized() {
	return *this / mag();
}

Vector2d Vector2d::operator+(Vector2d rhs) const {
	return {x + rhs.x, y + rhs.y};
}
Vector2d Vector2d::operator-(Vector2d rhs) const {
	return {x - rhs.x, y - rhs.y};
};
Vector2d Vector2d::operator*(double scalar) const {
	return {x * scalar, y * scalar};
}
Vector2d Vector2d::operator/(double scalar) const {
	return {x / scalar, y / scalar};
};

Vector2d& Vector2d::operator+=(Vector2d rhs) {
	x += rhs.x;
	y += rhs.y;
	return *this;
}
Vector2d& Vector2d::operator-=(Vector2d rhs) {
	x -= rhs.x;
	y -= rhs.y;
	return *this;
}
Vector2d& Vector2d::operator*=(double scalar) {
	x *= scalar;
	y *= scalar;
	return *this;
};
Vector2d& Vector2d::operator/=(double scalar) {
	x /= scalar;
	y /= scalar;
	return *this;
}
Vector2d& Vector2d::operator=(Vector2d v) {
	x = v.x;
	y = v.y;
	return *this;
}

bool Vector2d::operator==(const Vector2d& rhs) const {
	return x == rhs.x && y == rhs.y;
}
bool Vector2d::operator!=(const Vector2d& rhs) const {
	return !(*this == rhs);
}