#pragma once
#include <queue>

namespace lib16868X {
class SMAFilter {
	public:
		SMAFilter(int sampleSize);
		double filter(double input);
		inline double getOutput() { return output; };

	private:
		double sum = 0;
		double output = 0;
		int sampleSize;
		std::queue<double> samples;
};
} // namespace lib16868X