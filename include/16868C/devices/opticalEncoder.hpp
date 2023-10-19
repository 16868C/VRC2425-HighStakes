#pragma once
#include "16868C/devices/abstractEncoder.hpp"
#include "16868C/util/util.hpp"
#include "okapi/api.hpp"

namespace lib16868C {
class OpticalEncoder : private okapi::ADIEncoder, public AbstractEncoder {
	public:
		OpticalEncoder(const OpticalEncoder& enc);
		OpticalEncoder(int portTop, int portBottom, bool reverse);
		OpticalEncoder(int smartPort, int portTop, int portBottom, bool reverse);

		inline double get() override {
			return okapi::ADIEncoder::get();
		}
		inline void resetZero() override {
			okapi::ADIEncoder::reset();
		}
		double getVelocity() override;
	
	private:
		double prevTicks = 0;
		uint prevTime = 0;
};
} // namespace lib16868C