#pragma once
#include "16868Z/devices/abstractEncoder.hpp"
#include "16868Z/util/util.hpp"
#include "okapi/api.hpp"

namespace lib16868Z {
class Rotation : private okapi::RotationSensor, public AbstractEncoder {
	public:
		Rotation(int port);
		Rotation(int port, bool reversed);

		inline double get() override {
			return okapi::RotationSensor::get();
		};
		inline void resetZero() override {
			okapi::RotationSensor::reset();
		};

		double getVelocity();
	
	private:
		double prevTicks = 0;
		uint prevTime = 0;
};
} // namespace lib16868Z