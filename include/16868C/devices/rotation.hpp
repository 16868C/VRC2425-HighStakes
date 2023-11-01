#pragma once
#include "api.h"
#include "16868C/devices/abstractEncoder.hpp"
#include "16868C/util/util.hpp"

namespace lib16868C {
class Rotation : private pros::Rotation, public AbstractEncoder {
	public:
		Rotation(int port);
		Rotation(int port, bool reversed);

		/**
		 * @brief Returns the current position of the encoder in degrees.
		 * 
		 * @return double 
		 */
		inline double get() override {
			return get_position() / 100.0;
		};
		/**
		 * @brief Manually sets the zero of the encoder.
		 * * Does not actually reset the encoder.
		 */
		inline void resetZero() override {
			reset_position();
		};

		/**
		 * @brief Gets the velocity of the encoder.
		 * ? I do not know how accurate this is
		 * 
		 * @return double 
		 */
		double getVelocity();
	
	private:
		double prevTicks = 0;
		uint prevTime = 0;
};
} // namespace lib16868C