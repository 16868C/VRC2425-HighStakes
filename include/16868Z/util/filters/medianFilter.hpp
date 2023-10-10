#pragma once
#include <queue>

namespace lib16868C {
class MedianFilter {
	public:
		MedianFilter(int sampleSize);
		double filter(double input);
		inline double getOutput() { return output; };
	
	private:
		int sampleSize;
		double output;
		std::queue<double> samples;
};
} // namespace lib16868C