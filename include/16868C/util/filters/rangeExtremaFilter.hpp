#pragma once
#include <queue>

namespace lib16868C {
class RangeExtremaFilter {
	public:
		RangeExtremaFilter(int sampleSize);
		double filter(double input);
		inline double getOutput() { return output; }

	private:
		int sampleSize;
		double output;
		std::queue<double> samples;
};
} // namespace lib16868C