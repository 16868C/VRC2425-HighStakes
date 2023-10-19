#pragma once
#include "16868C/devices/abstractEncoder.hpp"
#include "16868C/util/util.hpp"
#include "okapi/api.hpp"

namespace lib16868C {
class Rotation : private okapi::RotationSensor, public AbstractEncoder {
	public:
		Rotation(int port);
		Rotation(int port, bool reversed);

		/**
		 * @brief Returns the current position of the encoder.
		 * 
		 * @return double 
		 */
		inline double get() override {
			return (okapi::RotationSensor::get() - initPos) * reversed;
		};
		/**
		 * @brief Manually sets the zero of the encoder.
		 * * Does not actually reset the encoder.
		 */
		inline void resetZero() override {
			initPos = get() * reversed;
		};

		/**
		 * @brief Gets the velocity of the encoder.
		 * ? I do not know how accurate this is
		 * 
		 * @return double 
		 */
		double getVelocity();
	
	private:
		int initPos;
		int reversed;

		double prevTicks = 0;
		uint prevTime = 0;
};
} // namespace lib16868C