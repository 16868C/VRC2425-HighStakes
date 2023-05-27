#include "16868Z/util/filters/medianFilter.hpp"
#include "16868Z/util/util.hpp"
#include <algorithm>
#include <vector>

using namespace lib16868Z;

MedianFilter::MedianFilter(int sampleSize) : sampleSize(sampleSize) {}

double MedianFilter::filter(double input) {
	samples.push(input);
	if (samples.size() > sampleSize) {
		samples.pop();
	}

	std::vector<double> v = Util::queueToVector(samples);

	size_t medianIndex = v.size() / 2;
	std::nth_element(v.begin(), v.begin() + medianIndex, v.end());
	output = (v.size() & 1) ? (v[medianIndex])
							: ((v[medianIndex] + *std::min_element(v.begin(), v.begin() + medianIndex - 1)) / 2.0);
	return output;
}