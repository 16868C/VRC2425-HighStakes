#pragma once
#include "okapi/api.hpp"
#include "16868C/util/math.hpp"

namespace lib16868C {
class Motor : public okapi::Motor {
	public:
	Motor(int port, okapi::AbstractMotor::gearset gearset);
	Motor(uint port, bool reverse, okapi::AbstractMotor::gearset gearset);

	double getPosition() override;
};
} // namespace lib16868C